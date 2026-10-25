#include "mlx90393.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_FREQ_HZ     400000
#define I2C_TIMEOUT_MS  20

#define CMD_EX          0x80    /* exit mode */
#define CMD_RT          0xF0    /* reset */
#define CMD_SM          0x3F    /* single measurement, zyxt */
#define CMD_RM          0x4F    /* read measurement, zyxt */
#define CMD_RR          0x50    /* read register */
#define CMD_WR          0x60    /* write register */
#define STATUS_ERROR    0x10

/*
 * Least sensitive range: the magnet sits very close to the chip.
 * GAIN_SEL = 0 with RES = 2 gives the same full scale as RES = 3
 * (XY ~ +-66 mT, Z ~ +-106 mT) at twice the resolution (XY 3.0 uT/LSB,
 * Z 4.8 uT/LSB). TCMP_EN must stay off: it is not supported with RES = 2/3.
 */
#define REG0_VALUE      0x000C  /* GAIN_SEL = 0, HALLCONF = 0xC */
#define REG1_VALUE      0x0000  /* TCMP_EN = 0 */
#define REG2_VALUE      0x0554  /* RES_X/Y/Z = 2, DIG_FILT = 5, OSR = 0 */

/* uT/LSB for HALLCONF = 0xC, RES = 0, indexed by GAIN_SEL (datasheet) */
static const float SENS_XY[8] = {0.751f, 0.601f, 0.451f, 0.376f, 0.300f, 0.250f, 0.200f, 0.150f};
static const float SENS_Z[8]  = {1.210f, 0.968f, 0.726f, 0.605f, 0.484f, 0.403f, 0.323f, 0.242f};

static esp_err_t xfer(mlx90393_t *m, const uint8_t *w, size_t wlen, uint8_t *r, size_t rlen)
{
    esp_err_t err = i2c_master_transmit_receive(m->dev, w, wlen, r, rlen, I2C_TIMEOUT_MS);
    if (err == ESP_OK && (r[0] & STATUS_ERROR)) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}

static esp_err_t cmd(mlx90393_t *m, uint8_t c)
{
    uint8_t st;
    return xfer(m, &c, 1, &st, 1);
}

static esp_err_t read_reg(mlx90393_t *m, uint8_t reg, uint16_t *val)
{
    uint8_t w[2] = {CMD_RR, (uint8_t)(reg << 2)};
    uint8_t r[3];
    esp_err_t err = xfer(m, w, 2, r, 3);
    if (err == ESP_OK) {
        *val = (r[1] << 8) | r[2];
    }
    return err;
}

static esp_err_t write_reg(mlx90393_t *m, uint8_t reg, uint16_t val)
{
    uint8_t w[4] = {CMD_WR, val >> 8, val & 0xFF, (uint8_t)(reg << 2)};
    uint8_t st;
    return xfer(m, w, 4, &st, 1);
}

esp_err_t mlx90393_init(mlx90393_t *m, i2c_master_bus_handle_t bus, uint8_t addr)
{
    esp_err_t err = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    if (!m->dev) {
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        err = i2c_master_bus_add_device(bus, &cfg, &m->dev);
        if (err != ESP_OK) {
            return err;
        }
    }

    cmd(m, CMD_EX);                         /* leave burst/WOC mode if active */
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd(m, CMD_RT);                         /* reset status may carry the RS bit only */
    vTaskDelay(pdMS_TO_TICKS(100));         /* chip needs time after reset */

    uint16_t reg0, reg1, reg2;
    if ((err = write_reg(m, 0x00, REG0_VALUE)) != ESP_OK ||
        (err = write_reg(m, 0x01, REG1_VALUE)) != ESP_OK ||
        (err = write_reg(m, 0x02, REG2_VALUE)) != ESP_OK ||
        (err = read_reg(m, 0x00, &reg0)) != ESP_OK ||
        (err = read_reg(m, 0x01, &reg1)) != ESP_OK ||
        (err = read_reg(m, 0x02, &reg2)) != ESP_OK ||
        (err = read_reg(m, 0x24, &m->tref)) != ESP_OK) {
        return err;
    }
    if (reg0 != REG0_VALUE || reg1 != REG1_VALUE || reg2 != REG2_VALUE) {
        return ESP_ERR_INVALID_STATE;       /* write did not stick */
    }

    m->gain_sel = (reg0 >> 4) & 0x07;
    m->res = (reg2 >> 5) & 0x03;
    m->tcmp = reg1 & 0x0400;
    uint8_t dig_filt = (reg2 >> 2) & 0x07;
    uint8_t osr = reg2 & 0x03;
    /* datasheet: per axis 67 + 64 * 2^OSR * (2 + 2^DIG_FILT) us; 3 axes + temperature */
    uint32_t t_axis_us = 67 + 64 * (1u << osr) * (2 + (1u << dig_filt));
    m->conv_ms = (4 * t_axis_us) / 1000 + 2;
    return ESP_OK;
}

esp_err_t mlx90393_start(mlx90393_t *m)
{
    return cmd(m, CMD_SM);
}

static float to_ut(uint16_t raw, const mlx90393_t *m, float sens)
{
    int32_t v;
    if (m->res == 2) {
        v = (int32_t)raw - 32768;
    } else if (m->res == 3) {
        v = (int32_t)raw - 16384;
    } else if (m->tcmp) {
        v = (int32_t)raw - 32768;           /* TCMP_EN: unsigned, 0 uT = 2^15 */
    } else {
        v = (int16_t)raw;
    }
    return v * sens * (1 << m->res);
}

esp_err_t mlx90393_read(mlx90393_t *m, mlx90393_data_t *out)
{
    uint8_t c = CMD_RM;
    uint8_t r[9];
    esp_err_t err = xfer(m, &c, 1, r, 9);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t t = (r[1] << 8) | r[2];
    out->t = 35.0f + ((float)t - (float)m->tref) / 45.2f;
    out->x = to_ut((r[3] << 8) | r[4], m, SENS_XY[m->gain_sel]);
    out->y = to_ut((r[5] << 8) | r[6], m, SENS_XY[m->gain_sel]);
    out->z = to_ut((r[7] << 8) | r[8], m, SENS_Z[m->gain_sel]);
    return ESP_OK;
}

esp_err_t mlx90393_measure(mlx90393_t *m, mlx90393_data_t *out)
{
    esp_err_t err = mlx90393_start(m);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(m->conv_ms + 1));
    return mlx90393_read(m, out);
}
