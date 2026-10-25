/*
 * MLX90393 connection test for ESP32-S3
 *
 *   SDA = GPIO8, SCL = GPIO9
 *
 * Every round:
 *   1. Pin diagnostics (before I2C is started): short to GND, SDA/SCL short,
 *      presence of external pull-up resistors.
 *   2. I2C bus scan, report every address that ACKs.
 *   3. If an MLX90393 is found: reset it, read its config registers, then
 *      continuously print X/Y/Z (uT) and temperature.
 * If nothing is found the round repeats every 3 s, so you can re-solder and
 * watch the result live.
 */
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#define PIN_SDA         8
#define PIN_SCL         9
#define I2C_FREQ_HZ     100000
#define I2C_TIMEOUT_MS  50

/* MLX90393 commands (zyxt = 0xF: all three axes + temperature) */
#define MLX_CMD_EX      0x80    /* exit mode */
#define MLX_CMD_RT      0xF0    /* reset */
#define MLX_CMD_SM      0x3F    /* start single measurement */
#define MLX_CMD_RM      0x4F    /* read measurement */
#define MLX_CMD_RR      0x50    /* read register */
#define MLX_STATUS_ERR  0x10

#define MAX_CONSECUTIVE_ERRORS 10

static const char *TAG = "mlx_test";

/* uT/LSB for HALLCONF = 0xC, RES = 0, indexed by GAIN_SEL (datasheet table) */
static const float SENS_XY[8] = {0.751f, 0.601f, 0.451f, 0.376f, 0.300f, 0.250f, 0.200f, 0.150f};
static const float SENS_Z[8]  = {1.210f, 0.968f, 0.726f, 0.605f, 0.484f, 0.403f, 0.323f, 0.242f};

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t  addr;
    uint8_t  gain_sel;
    uint8_t  res_x, res_y, res_z;
    uint16_t tref;
    uint32_t conv_ms;
} mlx_t;

/* ---------- 1. pin diagnostics ---------- */

static int read_pin(int pin, bool pullup, bool pulldown)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = pullup,
        .pull_down_en = pulldown,
    };
    gpio_config(&cfg);
    esp_rom_delay_us(200);
    return gpio_get_level(pin);
}

/* Returns false if a hard fault (short) was found. */
static bool pin_diagnostics(void)
{
    bool ok = true;
    const int pins[2] = {PIN_SDA, PIN_SCL};
    const char *names[2] = {"SDA(GPIO8)", "SCL(GPIO9)"};
    bool ext_pullup[2];

    printf("\n[1] Pin diagnostics\n");
    for (int i = 0; i < 2; i++) {
        int with_pu = read_pin(pins[i], true, false);
        ext_pullup[i] = read_pin(pins[i], false, true);
        if (!with_pu) {
            printf("   %s: LOW even with pull-up -> shorted to GND, or the sensor is holding it low\n", names[i]);
            ok = false;
        } else {
            printf("   %s: idle HIGH ok, external pull-up: %s\n", names[i], ext_pullup[i] ? "yes" : "NO");
        }
    }

    if (ok) {
        /* drive SCL low, see whether SDA follows */
        read_pin(PIN_SDA, true, false);
        gpio_set_direction(PIN_SCL, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_SCL, 0);
        esp_rom_delay_us(200);
        int sda = gpio_get_level(PIN_SDA);
        if (!sda) {
            printf("   SDA follows SCL -> SDA and SCL are SHORTED together (solder bridge?)\n");
            ok = false;
        } else {
            printf("   SDA/SCL not shorted together: ok\n");
        }
    }

    if (ok && (!ext_pullup[0] || !ext_pullup[1])) {
        printf("   Note: no external pull-up found, using ESP32 internal ~45k pull-ups.\n"
               "         Fine for short wires at 100 kHz; add 4.7k-10k to 3.3V if reads are flaky.\n");
    }

    gpio_reset_pin(PIN_SDA);
    gpio_reset_pin(PIN_SCL);
    return ok;
}

/* ---------- 2. bus scan ---------- */

/* Returns the first address in the MLX90393 range (0x0C-0x1B), or 0. */
static uint8_t scan_bus(i2c_master_bus_handle_t bus)
{
    uint8_t mlx_addr = 0;
    int found = 0;

    printf("\n[2] I2C scan (100 kHz)\n");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        esp_err_t err = i2c_master_probe(bus, a, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            bool is_mlx_range = (a >= 0x0C && a <= 0x1B);
            printf("   found device at 0x%02X%s\n", a, is_mlx_range ? "  <- MLX90393 address range" : "");
            if (is_mlx_range && !mlx_addr) {
                mlx_addr = a;
            }
            found++;
        } else if (err == ESP_ERR_TIMEOUT) {
            printf("   bus timeout at 0x%02X -> bus stuck (SDA or SCL held low)\n", a);
            i2c_master_bus_reset(bus);
            return 0;
        }
    }
    if (!found) {
        printf("   no device responded.\n"
               "   Check: VDD/VDDIO = 3.3V, GND, SDA/SCL solder joints, CS pin tied HIGH (I2C mode).\n");
    } else if (!mlx_addr) {
        printf("   devices found, but none in 0x0C-0x1B (MLX90393 range)\n");
    }
    return mlx_addr;
}

/* ---------- 3. MLX90393 ---------- */

/* Repeated-start (write+read) by default; falls back to write, STOP, then read
 * (what Arduino Wire does) if the sensor rejects the repeated start. */
static bool use_stop_between = false;

static esp_err_t mlx_xfer(mlx_t *m, const uint8_t *w, size_t wlen, uint8_t *r, size_t rlen)
{
    if (!use_stop_between) {
        esp_err_t err = i2c_master_transmit_receive(m->dev, w, wlen, r, rlen, I2C_TIMEOUT_MS);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        printf("   repeated-start transfer failed (%s), switching to write/STOP/read\n", esp_err_to_name(err));
        use_stop_between = true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    esp_err_t err = i2c_master_transmit(m->dev, w, wlen, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    esp_rom_delay_us(100);
    return i2c_master_receive(m->dev, r, rlen, I2C_TIMEOUT_MS);
}

static esp_err_t mlx_cmd(mlx_t *m, uint8_t cmd, uint8_t *status)
{
    return mlx_xfer(m, &cmd, 1, status, 1);
}

static esp_err_t mlx_read_reg(mlx_t *m, uint8_t reg, uint16_t *val, uint8_t *status)
{
    uint8_t w[2] = {MLX_CMD_RR, (uint8_t)(reg << 2)};
    uint8_t r[3];
    esp_err_t err = mlx_xfer(m, w, 2, r, 3);
    if (err == ESP_OK) {
        *status = r[0];
        *val = (r[1] << 8) | r[2];
    }
    return err;
}

static bool mlx_init(mlx_t *m)
{
    uint8_t st;
    uint16_t reg0, reg2;

    printf("\n[3] MLX90393 at 0x%02X\n", m->addr);

    mlx_cmd(m, MLX_CMD_EX, &st);            /* leave burst/WOC mode if active */
    vTaskDelay(pdMS_TO_TICKS(2));
    if (mlx_cmd(m, MLX_CMD_RT, &st) != ESP_OK) {
        printf("   reset command failed (I2C error)\n");
        return false;
    }
    printf("   reset ok, status=0x%02X\n", st);
    vTaskDelay(pdMS_TO_TICKS(100));

    const uint8_t regs[3] = {0x00, 0x02, 0x24};
    uint16_t *vals[3] = {&reg0, &reg2, &m->tref};
    for (int i = 0; i < 3; i++) {
        esp_err_t err = mlx_read_reg(m, regs[i], vals[i], &st);
        if (err != ESP_OK) {
            printf("   read reg 0x%02X failed: %s\n", regs[i], esp_err_to_name(err));
            return false;
        }
        if (st & MLX_STATUS_ERR) {
            printf("   read reg 0x%02X: sensor ERROR status=0x%02X\n", regs[i], st);
        }
    }
    printf("   reg0=0x%04X reg2=0x%04X TREF=0x%04X\n", reg0, reg2, m->tref);

    uint8_t hallconf = reg0 & 0x0F;
    m->gain_sel = (reg0 >> 4) & 0x07;
    m->res_x = (reg2 >> 5) & 0x03;
    m->res_y = (reg2 >> 7) & 0x03;
    m->res_z = (reg2 >> 9) & 0x03;
    uint8_t dig_filt = (reg2 >> 2) & 0x07;
    uint8_t osr = reg2 & 0x03;
    printf("   GAIN_SEL=%d HALLCONF=0x%X RES=%d/%d/%d DIG_FILT=%d OSR=%d\n",
           m->gain_sel, hallconf, m->res_x, m->res_y, m->res_z, dig_filt, osr);
    if (hallconf != 0xC) {
        printf("   warning: HALLCONF != 0xC, uT scaling below may be off\n");
    }
    if (reg0 == 0x0000 || reg0 == 0xFFFF) {
        printf("   warning: suspicious register value, chip may not be powered correctly\n");
    }

    /* conversion time per axis (datasheet): 67 + 64 * 2^OSR * (2 + 2^DIG_FILT) us, x3 axes + temp */
    uint32_t t_axis_us = 67 + 64 * (1u << osr) * (2 + (1u << dig_filt));
    m->conv_ms = (4 * t_axis_us) / 1000 + 5;
    printf("   expected conversion time ~%lu ms\n", (unsigned long)m->conv_ms);
    return true;
}

static float to_ut(uint16_t raw, uint8_t res, float sens)
{
    int32_t v;
    switch (res) {
    case 2:  v = (int32_t)raw - 32768; break;
    case 3:  v = (int32_t)raw - 16384; break;
    default: v = (int16_t)raw;         break;
    }
    return v * sens * (1 << res);
}

/* Returns ESP_OK and fills values, or an error. */
static esp_err_t mlx_measure(mlx_t *m, float *x, float *y, float *z, float *t, uint8_t *status)
{
    uint8_t st;
    esp_err_t err = mlx_cmd(m, MLX_CMD_SM, &st);
    if (err != ESP_OK) {
        return err;
    }
    if (st & MLX_STATUS_ERR) {
        *status = st;
        return ESP_ERR_INVALID_STATE;
    }
    vTaskDelay(pdMS_TO_TICKS(m->conv_ms));

    uint8_t cmd = MLX_CMD_RM;
    uint8_t r[9];
    err = mlx_xfer(m, &cmd, 1, r, 9);
    if (err != ESP_OK) {
        return err;
    }
    *status = r[0];
    if (r[0] & MLX_STATUS_ERR) {
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t traw = (r[1] << 8) | r[2];
    uint16_t xraw = (r[3] << 8) | r[4];
    uint16_t yraw = (r[5] << 8) | r[6];
    uint16_t zraw = (r[7] << 8) | r[8];

    *t = 35.0f + ((float)traw - (float)m->tref) / 45.2f;
    *x = to_ut(xraw, m->res_x, SENS_XY[m->gain_sel]);
    *y = to_ut(yraw, m->res_y, SENS_XY[m->gain_sel]);
    *z = to_ut(zraw, m->res_z, SENS_Z[m->gain_sel]);
    return ESP_OK;
}

static void mlx_run(mlx_t *m)
{
    uint32_t n = 0, errors = 0, consecutive = 0;

    printf("\n   Reading continuously. Move a magnet near the sensor: X/Y/Z should change.\n");
    printf("   Earth field alone gives |B| roughly 25-65 uT.\n\n");

    while (consecutive < MAX_CONSECUTIVE_ERRORS) {
        float x, y, z, t;
        uint8_t st = 0;
        esp_err_t err = mlx_measure(m, &x, &y, &z, &t, &st);
        n++;
        if (err == ESP_OK) {
            consecutive = 0;
            float b = sqrtf(x * x + y * y + z * z);
            printf("#%-5lu X=%9.1f  Y=%9.1f  Z=%9.1f uT  |B|=%8.1f  T=%5.1f C  err=%lu/%lu\n",
                   (unsigned long)n, x, y, z, b, t, (unsigned long)errors, (unsigned long)n);
            if (t < -40 || t > 125) {
                printf("       temperature out of range -> data looks invalid\n");
            }
        } else {
            errors++;
            consecutive++;
            if (err == ESP_ERR_INVALID_STATE) {
                printf("#%-5lu sensor reported ERROR, status=0x%02X\n", (unsigned long)n, st);
            } else {
                printf("#%-5lu I2C error: %s  (intermittent errors = bad solder joint / weak pull-up)\n",
                       (unsigned long)n, esp_err_to_name(err));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    printf("\n   %d consecutive failures, sensor lost. Restarting test...\n", MAX_CONSECUTIVE_ERRORS);
}

void app_main(void)
{
    esp_log_level_set("i2c.master", ESP_LOG_NONE);   /* scan NACKs are expected, keep output readable */
    printf("\n===== MLX90393 connection test  SDA=GPIO%d SCL=GPIO%d =====\n", PIN_SDA, PIN_SCL);

    for (int round = 1;; round++) {
        printf("\n---------------- round %d ----------------", round);
        if (!pin_diagnostics()) {
            printf("\n=> Fix the wiring above. Retrying in 3 s...\n");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        i2c_master_bus_handle_t bus;
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = PIN_SDA,
            .scl_io_num = PIN_SCL,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

        mlx_t m = {.addr = scan_bus(bus)};
        if (m.addr) {
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = m.addr,
                .scl_speed_hz = I2C_FREQ_HZ,
            };
            ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &m.dev));
            if (mlx_init(&m)) {
                printf("\n=> MLX90393 communication OK\n");
                mlx_run(&m);
            } else {
                printf("\n=> Chip ACKs its address but commands fail: check VDD and solder joints.\n");
            }
            i2c_master_bus_rm_device(m.dev);
        }
        i2c_del_master_bus(bus);
        ESP_LOGD(TAG, "round %d done", round);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
