/*
 * Ultra-thin tactile sensor firmware (ESP32-S3)
 *
 *   MLX90393 magnetometer : I2C, SDA = GPIO8, SCL = GPIO9, address 0x0C
 *   2x 74HC4067 + ADC     : 32 piezoresistive channels (see mux_adc.c)
 *
 * Everything is sampled at SAMPLE_HZ (100 Hz) from an esp_timer: each frame
 * reads all 32 ADC channels, then collects the MLX90393 result started in the
 * previous frame and starts the next conversion (~7 ms, fits in the 10 ms
 * period). A lower-priority task prints the latest frame to the serial
 * monitor PRINT_HZ times per second.
 *
 * Stream mode (for visualizer/tactile_viz.py): send 'b' over the serial port
 * and every frame is sent as a binary packet (stream_pkt_t, 100 Hz, ~8.7 kB/s,
 * fits in 115200 baud) instead of the text printout. 't' switches back. Every
 * reset starts in text mode, so the VS Code monitor keeps working.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_timer.h"
#include "mlx90393.h"
#include "mux_adc.h"

#define SAMPLE_HZ           100
#define PRINT_HZ            5
#define PIN_SDA             8
#define PIN_SCL             9
#define MLX_ADDR            0x0C
#define MLX_BASELINE_N      20      /* samples averaged at startup for dX/dY/dZ */
#define MLX_MAX_ERRORS      10      /* consecutive errors before re-initialising */
#define MLX_RETRY_FRAMES    SAMPLE_HZ   /* retry a missing sensor once per second */
#define CONSOLE_UART        UART_NUM_0
#define STREAM_QUEUE_LEN    16

/* Binary stream packet, little endian. sum = 8-bit sum of all bytes from ver to mlx[3]. */
#define STREAM_SYNC0        0xA5
#define STREAM_SYNC1        0x5A
#define STREAM_VERSION      1
#define STREAM_FLAG_MLX_OK  0x01
#define STREAM_FLAG_DROPPED 0x02    /* packets were dropped since the previous one */

typedef struct __attribute__((packed)) {
    uint8_t sync[2];
    uint8_t ver;
    uint16_t seq;
    uint8_t flags;
    uint16_t adc[MUX_COUNT][MUX_CHANNELS];
    float mlx[4];                   /* X, Y, Z in uT, T in deg C (raw, no baseline) */
    uint8_t sum;
} stream_pkt_t;

typedef struct {
    uint32_t seq;
    int64_t t_us;
    uint16_t adc[MUX_COUNT][MUX_CHANNELS];
    bool mlx_ok;
    mlx90393_data_t mlx;
    mlx90393_data_t mlx_delta;      /* relative to the startup baseline */
    /* stats over the last print interval */
    float rate_hz;
    int64_t worst_frame_us;
    uint32_t missed;
} frame_t;

static i2c_master_bus_handle_t bus;
static mlx90393_t mlx;
static mlx90393_data_t mlx_base;
static bool mlx_ok;

static TaskHandle_t sample_task, print_task;
static frame_t snapshot;
static QueueHandle_t stream_q;
static volatile bool stream_mode;

/* Init + baseline. Blocks for ~0.3 s. Boards without an MLX90393 are
 * supported: the failure is reported once and retried quietly every second. */
static bool mlx_setup(void)
{
    static esp_err_t last_err = ESP_OK;
    esp_err_t err = mlx90393_init(&mlx, bus, MLX_ADDR);
    if (err != ESP_OK) {
        if (err != last_err) {
            printf("MLX90393: not detected (%s), MLX data will show N.A.\n", esp_err_to_name(err));
        }
        last_err = err;
        return false;
    }
    last_err = ESP_OK;
    mlx90393_data_t d, sum = {0};
    mlx90393_measure(&mlx, &d);     /* first conversion after reset may flag an error */
    for (int i = 0; i < MLX_BASELINE_N; i++) {
        if ((err = mlx90393_measure(&mlx, &d)) != ESP_OK) {
            printf("MLX90393: baseline read failed (%s)\n", esp_err_to_name(err));
            return false;
        }
        sum.x += d.x;
        sum.y += d.y;
        sum.z += d.z;
    }
    mlx_base.x = sum.x / MLX_BASELINE_N;
    mlx_base.y = sum.y / MLX_BASELINE_N;
    mlx_base.z = sum.z / MLX_BASELINE_N;
    printf("MLX90393: ok, gain=%d res=%d conv=%lu ms, baseline X=%.1f Y=%.1f Z=%.1f uT\n",
           mlx.gain_sel, mlx.res, (unsigned long)mlx.conv_ms, mlx_base.x, mlx_base.y, mlx_base.z);
    return mlx90393_start(&mlx) == ESP_OK;
}

static void on_timer(void *arg)
{
    xTaskNotifyGive(sample_task);
}

static void sample_loop(void *arg)
{
    frame_t f = {0};
    stream_pkt_t pkt = {.sync = {STREAM_SYNC0, STREAM_SYNC1}, .ver = STREAM_VERSION};
    bool dropped = false;
    uint32_t mlx_errors = 0, missed = 0, frames_in_window = 0;
    int64_t worst = 0, window_start = esp_timer_get_time();

    while (1) {
        uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending > 1) {
            missed += pending - 1;
        }
        int64_t t0 = esp_timer_get_time();
        f.seq++;
        f.t_us = t0;

        mux_adc_read_all(f.adc);

        if (mlx_ok) {
            if (mlx90393_read(&mlx, &f.mlx) == ESP_OK) {
                mlx_errors = 0;
                f.mlx_delta.x = f.mlx.x - mlx_base.x;
                f.mlx_delta.y = f.mlx.y - mlx_base.y;
                f.mlx_delta.z = f.mlx.z - mlx_base.z;
            } else {
                mlx_errors++;
            }
            if (mlx90393_start(&mlx) != ESP_OK) {
                mlx_errors++;
            }
            if (mlx_errors >= MLX_MAX_ERRORS) {
                printf("MLX90393: %d consecutive errors, sensor lost\n", MLX_MAX_ERRORS);
                mlx_ok = false;
            }
        } else if (f.seq % MLX_RETRY_FRAMES == 0) {
            mlx_ok = mlx_setup();
            mlx_errors = 0;
        }
        f.mlx_ok = mlx_ok && mlx_errors == 0;

        if (stream_mode) {
            pkt.seq = (uint16_t)f.seq;
            pkt.flags = (f.mlx_ok ? STREAM_FLAG_MLX_OK : 0) | (dropped ? STREAM_FLAG_DROPPED : 0);
            memcpy(pkt.adc, f.adc, sizeof(pkt.adc));
            if (f.mlx_ok) {
                pkt.mlx[0] = f.mlx.x;
                pkt.mlx[1] = f.mlx.y;
                pkt.mlx[2] = f.mlx.z;
                pkt.mlx[3] = f.mlx.t;
            } else {
                pkt.mlx[0] = pkt.mlx[1] = pkt.mlx[2] = pkt.mlx[3] = NAN;
            }
            uint8_t sum = 0;
            for (const uint8_t *p = &pkt.ver; p < &pkt.sum; p++) {
                sum += *p;
            }
            pkt.sum = sum;
            dropped = xQueueSend(stream_q, &pkt, 0) != pdTRUE;
        }

        int64_t dt = esp_timer_get_time() - t0;
        if (dt > worst) {
            worst = dt;
        }

        if (++frames_in_window >= SAMPLE_HZ / PRINT_HZ) {
            int64_t now = esp_timer_get_time();
            f.rate_hz = frames_in_window * 1e6f / (now - window_start);
            f.worst_frame_us = worst;
            f.missed = missed;
            snapshot = f;
            xTaskNotifyGive(print_task);
            frames_in_window = 0;
            worst = 0;
            missed = 0;
            window_start = now;
        }
    }
}

static void print_loop(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (stream_mode) {
            continue;
        }
        const frame_t *f = &snapshot;

        printf("\n---- #%lu  t=%.2f s  %.1f Hz  frame %.1f ms  missed %lu\n",
               (unsigned long)f->seq, f->t_us / 1e6, f->rate_hz,
               f->worst_frame_us / 1000.0, (unsigned long)f->missed);
        if (f->mlx_ok) {
            printf("MLX   X=%8.1f Y=%8.1f Z=%8.1f uT | dX=%+7.1f dY=%+7.1f dZ=%+7.1f uT | T=%.1f C\n",
                   f->mlx.x, f->mlx.y, f->mlx.z,
                   f->mlx_delta.x, f->mlx_delta.y, f->mlx_delta.z, f->mlx.t);
        } else {
            printf("MLX   X=    N.A. Y=    N.A. Z=    N.A. uT | dX=   N.A. dY=   N.A. dZ=   N.A. uT | T=N.A.\n");
        }
        printf("CH  ");
        for (int c = 0; c < MUX_CHANNELS; c++) {
            printf("%5d", c);
        }
        printf("\n");
        for (int m = 0; m < MUX_COUNT; m++) {
            printf("MUX%d", m + 1);
            for (int c = 0; c < MUX_CHANNELS; c++) {
                printf("%5u", f->adc[m][c]);
            }
            printf("\n");
        }
    }
}

/* Sends queued packets; uart_write_bytes() writes a whole packet at once, so
 * the occasional printf (e.g. "sensor lost") only lands between packets. */
static void stream_loop(void *arg)
{
    stream_pkt_t pkt;
    while (1) {
        if (xQueueReceive(stream_q, &pkt, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(CONSOLE_UART, &pkt, sizeof(pkt));
        }
    }
}

/* 'b' = binary stream mode, 't' = text mode */
static void command_loop(void *arg)
{
    uint8_t c;
    while (1) {
        if (uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == 'b') {
            stream_mode = true;
        } else if (c == 't') {
            stream_mode = false;
            xQueueReset(stream_q);
        }
    }
}

void app_main(void)
{
    /* Console through the UART driver: needed to receive commands and to send
     * binary packets (the default console path turns 0x0A into 0x0D 0x0A). */
    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 256, 4096, 0, NULL, 0));
    uart_vfs_dev_use_driver(CONSOLE_UART);
    stream_q = xQueueCreate(STREAM_QUEUE_LEN, sizeof(stream_pkt_t));

    printf("\n===== Ultra-thin tactile sensor: MLX90393 + 32 ch ADC @ %d Hz =====\n", SAMPLE_HZ);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_ERROR_CHECK(mux_adc_init());
    mlx_ok = mlx_setup();

    xTaskCreatePinnedToCore(sample_loop, "sample", 4096, NULL, 10, &sample_task, 1);
    xTaskCreatePinnedToCore(print_loop, "print", 4096, NULL, 2, &print_task, 0);
    xTaskCreatePinnedToCore(stream_loop, "stream", 4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(command_loop, "cmd", 2048, NULL, 2, NULL, 0);

    esp_timer_handle_t timer;
    esp_timer_create_args_t targs = {.callback = on_timer, .name = "sample"};
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / SAMPLE_HZ));
}
