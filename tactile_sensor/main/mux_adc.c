#include "mux_adc.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_rom_sys.h"

/*
 * MUX1: S0-S3 = GPIO4,5,6,7     SIG -> GPIO1 (ADC1_CH0)
 * MUX2: S0-S3 = GPIO11,12,13,14 SIG -> GPIO2 (ADC1_CH1)
 *
 * The sensors are resistive dividers of 100k and up, so the ADC input is
 * slow to settle after switching: wait SETTLE_US after selecting a channel
 * and throw away the first conversion of each input (it still carries
 * charge from the previously sampled signal).
 */
#define SETTLE_US       100
#define DUMMY_READS     1

static const int SEL_PINS[MUX_COUNT][4] = {
    {4, 5, 6, 7},
    {11, 12, 13, 14},
};
static const adc_channel_t ADC_CH[MUX_COUNT] = {ADC_CHANNEL_0, ADC_CHANNEL_1};

static adc_oneshot_unit_handle_t adc;

esp_err_t mux_adc_init(void)
{
    uint64_t mask = 0;
    for (int m = 0; m < MUX_COUNT; m++) {
        for (int b = 0; b < 4; b++) {
            mask |= 1ULL << SEL_PINS[m][b];
        }
    }
    gpio_config_t io = {.pin_bit_mask = mask, .mode = GPIO_MODE_OUTPUT};
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = ADC_UNIT_1};
    if ((err = adc_oneshot_new_unit(&unit_cfg, &adc)) != ESP_OK) {
        return err;
    }
    adc_oneshot_chan_cfg_t ch_cfg = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12};
    for (int m = 0; m < MUX_COUNT; m++) {
        if ((err = adc_oneshot_config_channel(adc, ADC_CH[m], &ch_cfg)) != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

static void select_channel(int ch)
{
    for (int m = 0; m < MUX_COUNT; m++) {
        for (int b = 0; b < 4; b++) {
            gpio_set_level(SEL_PINS[m][b], (ch >> b) & 1);
        }
    }
}

void mux_adc_read_all(uint16_t out[MUX_COUNT][MUX_CHANNELS])
{
    for (int c = 0; c < MUX_CHANNELS; c++) {
        select_channel(c);
        esp_rom_delay_us(SETTLE_US);
        for (int m = 0; m < MUX_COUNT; m++) {
            int raw;
            for (int i = 0; i <= DUMMY_READS; i++) {
                adc_oneshot_read(adc, ADC_CH[m], &raw);
            }
            out[m][c] = raw;
        }
    }
}
