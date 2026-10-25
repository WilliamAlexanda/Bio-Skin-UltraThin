#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

typedef struct {
    float x, y, z;      /* uT */
    float t;            /* deg C */
} mlx90393_data_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint16_t tref;
    uint8_t gain_sel;
    uint8_t res;        /* same RES for X/Y/Z */
    bool tcmp;          /* temperature compensation on -> unsigned output */
    uint32_t conv_ms;
} mlx90393_t;

/* Reset, write the config (GAIN_SEL=0, RES=2, DIG_FILT=5: largest range) and read back TREF. */
esp_err_t mlx90393_init(mlx90393_t *m, i2c_master_bus_handle_t bus, uint8_t addr);

/* Start a single X/Y/Z/T conversion (SM). */
esp_err_t mlx90393_start(mlx90393_t *m);

/* Read the result of the last conversion (RM). Call >= conv_ms after mlx90393_start(). */
esp_err_t mlx90393_read(mlx90393_t *m, mlx90393_data_t *out);

/* Blocking start + wait + read, used for baseline averaging at startup. */
esp_err_t mlx90393_measure(mlx90393_t *m, mlx90393_data_t *out);
