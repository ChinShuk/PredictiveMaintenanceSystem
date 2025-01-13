/***** Include files *****/
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "soc/gpio_num.h"
#include "esp_err.h"
#include <cstddef>
#include <math.h>
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <sys/time.h>
#include "cJSON.h"
#include "rom/ets_sys.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc_cal_types_legacy.h"
#include "driver/adc.h"
#include <inttypes.h>
#include "adc_functions.h"
#include "ds18b20.h"
#include "BMI088.h"

/***** Feature Enable/Disable Macros *****/
//#define ANALOG_TEMP
//#define DIGITAL_TEMP
//#define DEBUG

inline int64_t xx_time_get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

/***** Wi-Fi Macros *****/
// Wi-Fi and MQTT configurations (use environment variables or secure storage)
#define WIFI_CONNECTED_BIT 				BIT0
#define WIFI_SSID 						"Embedded Wizard"
#define WIFI_PASS 						"Embeddedwizard@1416c"
#define MQTT_BROKER_URI \
    "mqtt://10.0.0.140"   
/*#define WIFI_SSID 					"20" 
#define WIFI_PASS 						"20242024"
#define MQTT_BROKER_URI 				"mqtt://192.168.106.145"
*/

/***** Vibration Sensor Macros *****/
#define NUM_READINGS 					100
#define INTERVAL						10

/***** Current Sensor Macros *****/
#define CURRENT_ADC_CHANNEL 			ADC_CHANNEL_3
#define NO_OF_SAMPLES 					64 
#define ACS712_VCC 						5
#define ACS712_ZERO_CURRENT 			(ACS712_VCC / 2) 
#define ACS712_SENSITIVITY 				0.185
#define DEFAULT_VREF 					1100


/***** Temperature Sensor Macros *****/
#define TEMPERATURE_ADC_CHANNEL 		ADC_CHANNEL_0

/***** General Macros *****/
#define LED_GPIO_PIN 					GPIO_NUM_45 // Change this to the GPIO pin connected to your LED

/***** Static Variables *****/
static const char *TAG 				=	"WiFi_MQTT_Example";
static EventGroupHandle_t 				s_wifi_event_group; 
static esp_adc_cal_characteristics_t 	*adc_chars;
static i2c_port_t Wire 				=	I2C_NUM_0;
static float ACS712CalbrateCurrent	= 	2.5; 

/***** Private datatypes *****/
typedef struct
{
    double ax[NUM_READINGS];
    double ay[NUM_READINGS];
    double az[NUM_READINGS];
}VibrationSensorData;

typedef struct 
{
	float current;
}CurrentSensorData;

/***** Function Declaration *****/

