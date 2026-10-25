/*
 * 2x 74HC4067 analog mux test for ESP32-S3 (32 channels @ 100 Hz each)
 *
 *   MUX1: S0-S3 = GPIO4,5,6,7     SIG -> GPIO1 (ADC1_CH0)
 *   MUX2: S0-S3 = GPIO11,12,13,14 SIG -> GPIO2 (ADC1_CH1)
 *
 * An esp_timer fires every 10 ms; each frame steps both muxes through
 * channels 0-15 together and reads both ADC inputs, so every one of the
 * 32 channels is sampled at 100 Hz. Once per second a table with the
 * mean/min/max of every channel is printed, plus a pass/fail check for
 * MUX1 CH0-3, which are expected to sit at 3.3 V (raw 4095).
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define NUM_MUX         2
#define CH_PER_MUX      16
#define FRAME_PERIOD_US 10000   /* 100 Hz per channel */
#define SETTLE_US       20      /* mux switching + ADC input settling */
#define REPORT_FRAMES   100     /* print once per second */

#define EXPECT_HIGH_RAW 4000    /* MUX1 CH0-3 should read >= this */

static const int SEL_PINS[NUM_MUX][4] = {
    {4, 5, 6, 7},
    {11, 12, 13, 14},
};
static const adc_channel_t ADC_CH[NUM_MUX] = {ADC_CHANNEL_0, ADC_CHANNEL_1};  /* GPIO1, GPIO2 */
static const int SIG_PINS[NUM_MUX] = {1, 2};

static adc_oneshot_unit_handle_t adc;
static adc_cali_handle_t cali;
static TaskHandle_t sample_task;
static TaskHandle_t report_task;

typedef struct {
    uint32_t sum;
    uint16_t min, max;
} ch_stats_t;

typedef struct {
    ch_stats_t ch[NUM_MUX][CH_PER_MUX];
    uint32_t frames, missed;
    int64_t elapsed_us, worst_frame_us;
} window_t;

/* sampling fills `live`; at the end of each window it is copied to `snapshot`
 * and printed by a lower-priority task, so slow UART output never delays sampling */
static window_t live, snapshot;

static void select_channel(int ch)
{
    for (int m = 0; m < NUM_MUX; m++) {
        for (int b = 0; b < 4; b++) {
            gpio_set_level(SEL_PINS[m][b], (ch >> b) & 1);
        }
    }
}

static void reset_window(window_t *w)
{
    memset(w, 0, sizeof(*w));
    for (int m = 0; m < NUM_MUX; m++) {
        for (int c = 0; c < CH_PER_MUX; c++) {
            w->ch[m][c].min = 0xFFFF;
        }
    }
}

static int raw_to_mv(int raw)
{
    int mv = 0;
    adc_cali_raw_to_voltage(cali, raw, &mv);
    return mv;
}

static void report(const window_t *w)
{
    printf("\n==== %.1f Hz per channel (%lu frames in %.3f s), worst frame %lld us, missed %lu ====\n",
           w->frames * 1e6 / w->elapsed_us, (unsigned long)w->frames, w->elapsed_us / 1e6,
           w->worst_frame_us, (unsigned long)w->missed);
    printf(" CH |      MUX1 (GPIO1)  raw mean  min-max    mV |      MUX2 (GPIO2)  raw mean  min-max    mV\n");
    for (int c = 0; c < CH_PER_MUX; c++) {
        printf(" %2d |", c);
        for (int m = 0; m < NUM_MUX; m++) {
            const ch_stats_t *s = &w->ch[m][c];
            int mean = s->sum / w->frames;
            printf("                    %4d  %4u-%-4u  %4d |", mean, s->min, s->max, raw_to_mv(mean));
        }
        printf("\n");
    }

    bool pass = true;
    for (int c = 0; c < 4; c++) {
        if (w->ch[0][c].min < EXPECT_HIGH_RAW) {
            pass = false;
            printf(" MUX1 CH%d: FAIL, min %u < %d (expected ~4095 / 3.3 V)\n", c, w->ch[0][c].min, EXPECT_HIGH_RAW);
        }
    }
    if (pass) {
        printf(" MUX1 CH0-3: PASS, all samples >= %d (3.3 V, ADC saturated)\n", EXPECT_HIGH_RAW);
    }
}

static void report_loop(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        report(&snapshot);
    }
}

/* Check select lines before driving them: as inputs with the internal
 * pull-down / pull-up, a healthy line (only CMOS mux inputs on it) follows
 * the pull. Returns false if any line is tied to 3.3 V or GND. */
static bool select_line_test(void)
{
    bool ok = true;
    printf("\n[select line test] S0-S3 as inputs with internal pull-down / pull-up\n");
    for (int m = 0; m < NUM_MUX; m++) {
        for (int b = 0; b < 4; b++) {
            int pin = SEL_PINS[m][b];
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_INPUT);
            gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
            esp_rom_delay_us(500);
            int pd = gpio_get_level(pin);
            gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
            esp_rom_delay_us(500);
            int pu = gpio_get_level(pin);
            gpio_set_pull_mode(pin, GPIO_FLOATING);

            const char *verdict = "ok";
            if (pd && pu) {
                verdict = "TIED TO 3.3V";
                ok = false;
            } else if (!pd && !pu) {
                verdict = "TIED TO GND";
                ok = false;
            }
            printf("   MUX%d S%d (GPIO%-2d): pd->%d pu->%d  %s\n", m + 1, b, pin, pd, pu, verdict);
        }
    }
    return ok;
}

static int read_avg(adc_channel_t ch)
{
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int raw;
        adc_oneshot_read(adc, ch, &raw);
        sum += raw;
    }
    return sum / 8;
}

/* Read every channel with the SIG pin's internal pull-down and then pull-up
 * enabled. A channel really driven through the mux (~100 ohm) ignores the
 * ~45k pulls; a floating / disconnected SIG line follows them. */
static void pull_test(void)
{
    printf("\n[pull test] SIG read with internal pull-down / pull-up (raw)\n");
    for (int m = 0; m < NUM_MUX; m++) {
        int driven = 0;
        printf(" MUX%d (GPIO%d):\n", m + 1, SIG_PINS[m]);
        for (int c = 0; c < CH_PER_MUX; c++) {
            select_channel(c);
            gpio_pullup_dis(SIG_PINS[m]);
            gpio_pulldown_en(SIG_PINS[m]);
            vTaskDelay(pdMS_TO_TICKS(2));
            int pd = read_avg(ADC_CH[m]);
            gpio_pulldown_dis(SIG_PINS[m]);
            gpio_pullup_en(SIG_PINS[m]);
            vTaskDelay(pdMS_TO_TICKS(2));
            int pu = read_avg(ADC_CH[m]);
            gpio_pullup_dis(SIG_PINS[m]);

            const char *verdict;
            if (pd < 300 && pu > 3800) {
                verdict = "floating (nothing driving SIG)";
            } else if (pu - pd < 300) {
                verdict = "driven";
                driven++;
            } else {
                verdict = "driven by a high-impedance source (e.g. resistor divider)";
                driven++;
            }
            printf("   CH%-2d pd=%4d pu=%4d  %s\n", c, pd, pu, verdict);
        }
        if (!driven) {
            printf("   => no channel drives SIG: check 74HC4067 VCC/GND, E (pin 15) must be LOW, SIG->GPIO%d trace\n",
                   SIG_PINS[m]);
        }
    }
}

static void on_timer(void *arg)
{
    xTaskNotifyGive(sample_task);
}

static void sample_loop(void *arg)
{
    int64_t window_start = esp_timer_get_time();
    reset_window(&live);

    while (1) {
        /* more than one pending notification means we fell behind */
        uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending > 1) {
            live.missed += pending - 1;
        }

        int64_t t0 = esp_timer_get_time();
        for (int c = 0; c < CH_PER_MUX; c++) {
            select_channel(c);
            esp_rom_delay_us(SETTLE_US);
            for (int m = 0; m < NUM_MUX; m++) {
                int raw;
                adc_oneshot_read(adc, ADC_CH[m], &raw);
                ch_stats_t *s = &live.ch[m][c];
                s->sum += raw;
                if (raw < s->min) s->min = raw;
                if (raw > s->max) s->max = raw;
            }
        }
        int64_t dt = esp_timer_get_time() - t0;
        if (dt > live.worst_frame_us) {
            live.worst_frame_us = dt;
        }

        if (++live.frames >= REPORT_FRAMES) {
            int64_t now = esp_timer_get_time();
            live.elapsed_us = now - window_start;
            snapshot = live;
            xTaskNotifyGive(report_task);
            window_start = now;
            reset_window(&live);
        }
    }
}

void app_main(void)
{
    printf("\n===== 74HC4067 x2 ADC test: 32 ch @ 100 Hz =====\n");

    uint64_t mask = 0;
    for (int m = 0; m < NUM_MUX; m++) {
        for (int b = 0; b < 4; b++) {
            mask |= 1ULL << SEL_PINS[m][b];
        }
    }
    if (!select_line_test()) {
        printf("\n=> A select line is shorted. Not driving the select pins (would fight the short).\n"
               "   Fix the hardware, then press RST.\n");
        return;
    }
    gpio_config_t io = {.pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&io));

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));
    adc_oneshot_chan_cfg_t ch_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    for (int m = 0; m < NUM_MUX; m++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, ADC_CH[m], &ch_cfg));
    }
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali));

    pull_test();

    xTaskCreatePinnedToCore(sample_loop, "sample", 4096, NULL, 10, &sample_task, 1);
    xTaskCreatePinnedToCore(report_loop, "report", 4096, NULL, 2, &report_task, 0);

    esp_timer_handle_t timer;
    esp_timer_create_args_t targs = {.callback = on_timer, .name = "frame"};
    ESP_ERROR_CHECK(esp_timer_create(&targs, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, FRAME_PERIOD_US));
}
