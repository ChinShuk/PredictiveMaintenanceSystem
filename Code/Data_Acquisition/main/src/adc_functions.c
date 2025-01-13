#include "adc_functions.h"

#define ADC_WIDTH ADC_BITWIDTH_12

static const char *TAG = "ADC";
static adc_oneshot_unit_handle_t adcHandle[ADC_UNIT_2] = {};

/**
 * validate_adc_unit - validates adc unit
 *
 * @param iAdc_unit - ADC unit number
 * @return true if ADC unit number os valid, else false
 */
bool validate_adc_unit(adc_unit_t iAdc_unit)
{
    if ((iAdc_unit < ADC_UNIT_1) || (iAdc_unit > ADC_UNIT_2))
    {
        ESP_LOGI(TAG, "Invalid ADC unit");
        return false;
    }

    return true;
}

/**
 * validate_adc_channel - validates adc channel
 *
 * @param iAdc_channel - ADC unit number
 * @return true if ADC unit number os valid, else false
 */
bool validate_adc_channel(adc_channel_t iAdc_channel)
{
    if ((iAdc_channel < ADC_CHANNEL_0) || (iAdc_channel > ADC_CHANNEL_9))
    {
        ESP_LOGI(TAG, "Invalid ADC channel");
        return false;
    }
    return true;
}

/**
 * init_adc - function used to initialise adc for provided unit and channel
 *
 * @param iAdc_unit - ADC number
 * @param iAdc_channel - ADC channel
 * @return returns true if initialised successfull else false
 */
bool init_adc_unit(adc_unit_t iAdc_unit)
{
    adc_oneshot_unit_handle_t tAdc_handle = NULL;

    if (NULL == get_adc_handle(iAdc_unit))
    {
        // Configure ADC1 channels
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = iAdc_unit,
        };

        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &tAdc_handle));

        adcHandle[iAdc_unit] = tAdc_handle;
        return true;
    }
    return false;
}

/**
 * configure_adc_channel - configures adc channel
 *
 * @param iAdc_unit - ADC Unit for which channel has to be configured
 * @param iAdc_channel - channel to be configured
 * @return true if configuration successfull else false
 */
bool configure_adc_channel(adc_unit_t iAdc_unit, adc_channel_t iAdc_channel)
{
    adc_oneshot_unit_handle_t tAdc_handle = NULL;
    if (false == validate_adc_channel(iAdc_channel))
    {
        ESP_LOGI(TAG, "Invalid ADC channel");
        return false;
    }

    if (NULL == (tAdc_handle = get_adc_handle(iAdc_unit)))
    {
        ESP_LOGI(TAG, "ADC Handle not initialised for adc_unit = %d", iAdc_unit);
        return false;
    }

    // Configure ADC channel
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN_DB_0, // Set attenuation
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(tAdc_handle, iAdc_channel, &channel_config));
    return true;
}

/**
 * is_adc_unit_handle_valid - validates adc unit handle
 *
 * @param iAdc_unit - ADC number
 * @return true if handle is valid else false
 */
bool is_adc_unit_handle_valid(adc_unit_t iAdc_unit)
{
    if (validate_adc_unit(iAdc_unit))
    {
        if (NULL == adcHandle[iAdc_unit])
        {
            return false;
        }
        return true;
    }
    return false;
}

/**
 * get_adc_handle - function used to get adc handle
 *
 * @param iAdc_unit - ADC number
 * @return if valid adc handle is availabl for provided arg. then returns handle else NULL
 */
adc_oneshot_unit_handle_t get_adc_handle(adc_unit_t iAdc_unit)
{
    if (is_adc_unit_handle_valid(iAdc_unit))
    {
        return adcHandle[iAdc_unit];
    }
    return NULL;
}

/**
 * function_name - Description of the function
 *
 * @param param1 - Description of param1
 * @param param2 - Description of param2
 * @return Description of return value
 */
int read_adc(adc_unit_t iAdc_unit, adc_channel_t iAdc_channel)
{
    int adc_read_value = 0;
    adc_oneshot_unit_handle_t adc_dandle = get_adc_handle(iAdc_unit);

    if (adc_dandle == NULL || false == validate_adc_channel(iAdc_channel))
    {
        return 0;
    }

    ESP_ERROR_CHECK(adc_oneshot_read(adc_dandle, iAdc_channel, &adc_read_value)); // Read ADC value from channel 0
    return adc_read_value;
}