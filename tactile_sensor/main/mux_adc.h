#pragma once

#include <stdint.h>
#include "esp_err.h"

#define MUX_COUNT       2
#define MUX_CHANNELS    16

esp_err_t mux_adc_init(void);

/* Step both muxes through channels 0-15 and read both SIG inputs.
 * out[m][c] = raw 12-bit value (0-4095) of MUX m+1, channel c. */
void mux_adc_read_all(uint16_t out[MUX_COUNT][MUX_CHANNELS]);
