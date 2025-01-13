#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include <stdbool.h>

bool validate_adc_unit(adc_unit_t iAdc_unit);

bool validate_adc_channel(adc_channel_t iAdc_channel);

bool init_adc_unit(adc_unit_t iAdc_unit);

bool configure_adc_channel(adc_unit_t iAdc_unit, adc_channel_t iAdc_channel);

bool is_adc_unit_handle_valid(adc_unit_t iAdc_unit);

adc_oneshot_unit_handle_t get_adc_handle(adc_unit_t iAdc_unit);

int read_adc(adc_unit_t iAdc_unit, adc_channel_t iAdc_channel);
