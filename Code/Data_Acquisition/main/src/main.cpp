/*
 * Project: Data Acquisition and Transmission
 * File: Initial_Software.c
 * Description: This code handles Wi-Fi initialization, MQTT client setup, sensor data acquisition
 *              (from current, vibration, and temperature sensors), and data publishing over MQTT.
 *              The code operates on an ESP32 platform and uses FreeRTOS for task scheduling.
 *
 * Team Name: Predictive Pioneer
 *
 * Dependencies:
 * - ESP32 SDK (esp-idf)
 * - FreeRTOS for task management
 * - ds18b20.c for temperature sensor handling
 * - adc_functions.c for ADC management
 */

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

extern "C"
{
	#include "adc_functions.h"
	#include "ds18b20.h"
	#include "BMI088.h"
}

//#define ANALOG_TEMP
#define DIGITAL_TEMP
// Definitions for ADC unit and channels
#define ADC_UNIT 						ADC_UNIT_2
#define CURRENT_ADC_CHANNEL 			ADC_CHANNEL_3
#define TEMPERATURE_ADC_CHANNEL 		ADC_CHANNEL_9
#define WIFI_CONNECTED_BIT 				BIT0
#define LED_GPIO_PIN 					GPIO_NUM_45 // Change this to the GPIO pin connected to your LED
#define NUM_READINGS 					50
#define INTERVAL						10.0
#define NO_OF_SAMPLES 					50 

// ACS712-05B Specifications
#define ACS712_VCC 5            // Power supply voltage (set to 3.3 if using 3.3V)
#define ACS712_ZERO_CURRENT (ACS712_VCC / 2) // 2.5V at zero current for 5V supply
#define ACS712_SENSITIVITY 0.185    // Sensitivity in V/A (185mV/A)
#define DEFAULT_VREF 1100           

// Wi-Fi and MQTT configurations (use environment variables or secure storage)
#define WIFI_SSID "Embedded Wizard"
#define WIFI_PASS "Embeddedwizard@1416c" 
// #define WIFI_SSID "Pixel_5899"      // Use project configuration for Wi-Fi SSID
// #define WIFI_PASS "20242025" // Use project configuration for Wi-Fi password

#define MQTT_BROKER_URI "mqtt://10.0.0.140"
//#define MQTT_BROKER_URI "mqtt://192.168.187.72"    // Use project configuration for MQTT broker URI

static const char *TAG = "WiFi_MQTT_Example"; // Tag for logging
static EventGroupHandle_t s_wifi_event_group; // Event group for Wi-Fi status
static esp_adc_cal_characteristics_t *adc_chars;
static i2c_port_t Wire 					=	I2C_NUM_0;
static float ACS712CalbrateCurrent = 2500 ; // Default, will be recalibrated

QueueHandle_t VibrationSensorDataQueue;
QueueHandle_t CurrentSensorDataQueue;

typedef enum
{
	DATA_VIBRATION = 0,
	DATA_CURRENT,
	DATA_TEMPERATURE,
	DATA_MAX,
}Data_e;

typedef struct 
{
    double ax[NUM_READINGS];
    double ay[NUM_READINGS];
    double az[NUM_READINGS];
} VibrationSensorData;

typedef struct 
{
	float current;
}CurrentSensorData;

/* accel object */
Bmi088Accel accel(Wire, 0x18);
void blink_led();

/* gyro object */
//Bmi088Gyro gyro(Wire, 0x68);

int64_t xx_time_get_time() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

/*
 * Function: mqtt_event_handler
 * ----------------------------
 * Handles MQTT events such as connection, data receipt, publishing, and errors.
 *
 * Parameters:
 *   void *handler_args: Arguments passed when registering the event handler (unused).
 *   esp_event_base_t base: Event base identifier for MQTT events.
 *   int32_t event_id: The specific MQTT event ID.
 *   void *event_data: Event-specific data (e.g., message data).
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event =(esp_mqtt_event_handle_t) event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        // Publish messages to different topics upon connection
        msg_id = esp_mqtt_client_publish(client, "vibration", "data_vibration", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "vibration", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_publish(client, "current", "data_current", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "current", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

        msg_id = esp_mqtt_client_publish(client, "temperature", "data_temperature", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        msg_id = esp_mqtt_client_subscribe(client, "temperature", 0);
        ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        //ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}


/*******************************************************************************
 * FUNCTION      : getTempratureValue()
 * PARAMETERS    : uint32_t iSensorValue
 * RETURNS       : float
 * DESCRIPTION   : This Function Calculates the Temparature value from value readed
 * 				from sensor and returns float value which will be on °C.
 *******************************************************************************
 */
/*static float getTempratureValue(uint32_t iSensorValue)
{
	return (((3.3 * iSensorValue) / 4096.0) - 0.5) * 100;
}*/


/*******************************************************************************
 * FUNCTION      : getVelocityValue()
 * PARAMETERS    : uint32_t iAccelValue
 * RETURNS       : float
 * DESCRIPTION   : This Function Calculates the velocity value from value readed
 * 				from sensor and returns float value which will be on m/s.
 *******************************************************************************
 */
/*static float getVelocityValue(float iAccelValue)
{
	return (INTERVAL * iAccelValue) / 1000;
}
*/
/*
 * Function: publish_task
 * ----------------------
 * Task responsible for reading sensor data (potentiometer and temperature)
 * and publishing it to the MQTT broker at regular intervals.
 *
 * Parameters:
 *   void *pvParameters: MQTT client handle passed as task parameters.
 */
void publish_task(void *pvParameters)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameters;
    
    float cTemp = 20.0;
    char pub_str[12]; // Buffer to hold the ADC string
    char formattedVibration[12];
    VibrationSensorData receivedVibrationData;
    CurrentSensorData   receivedCurrentData;    
    int data_len;
    
    #ifdef ANALOG_TEMP
    int temp_adc_value = 0;
    #endif

    #ifdef DIGITAL_TEMP
    DeviceAddress tempSensors[2];
    ds18b20_init(GPIO_NUM_47);
    ds18b20_setResolution(tempSensors, 0, 12);
	#endif

    #ifdef ANALOG_TEMP
    if (false == init_adc_unit(ADC_UNIT))
    {
        return;
    }

    if (false == configure_adc_channel(ADC_UNIT, TEMPERATURE_ADC_CHANNEL))
    {
        return;
    }
	#endif

    while (1)
    {
		for(int i = 0; i < 3; i++)
		{
			Data_e dataSeq = static_cast<Data_e>(i);
			switch(dataSeq)
			{
				case DATA_VIBRATION:
				{
				    if (VibrationSensorDataQueue == NULL)
				    {
				        ESP_LOGE(TAG, "sensor queue null");
				        break;
				    }
				
				    if (xQueueReceive(VibrationSensorDataQueue, &receivedVibrationData, portMAX_DELAY) == pdPASS)
				    {
				        cJSON *json_array = cJSON_CreateArray();
				        float Vi_x = 0.0, Vi_y = 0.0, Vi_z = 0.0; // Initial velocities for X, Y, Z axes
				        const float interval_ms = INTERVAL;      // Interval in milliseconds
				        const float gravity_correction = 9.81;  // Gravitational constant (m/s^2)
				        const float tolerance = 2.0;            // Tolerance for detecting gravity
				
				        for (int index = 0; index < NUM_READINGS; index++)
				        {
				            cJSON *json_object = cJSON_CreateObject();
				
				            // Correct ax if necessary
				            float corrected_ax = receivedVibrationData.ax[index];
				            if (fabs(receivedVibrationData.ax[index] - gravity_correction) <= tolerance ||
				                fabs(receivedVibrationData.ax[index] + gravity_correction) <= tolerance)
				            {
				                corrected_ax -= (receivedVibrationData.ax[index] > 0) ? gravity_correction : -gravity_correction;
				            }
				
				            // Compute final velocity for X axis
				            float Vo_x = Vi_x + corrected_ax * (interval_ms / 1000.0);
				            snprintf(formattedVibration, sizeof(formattedVibration), "%.8f", Vo_x);
				            cJSON_AddNumberToObject(json_object, "Vx", atof(formattedVibration));
				            Vi_x = Vo_x; // Update initial velocity for the next iteration
				
				            // Correct ay if necessary
				            float corrected_ay = receivedVibrationData.ay[index];
				            if (fabs(receivedVibrationData.ay[index] - gravity_correction) <= tolerance ||
				                fabs(receivedVibrationData.ay[index] + gravity_correction) <= tolerance)
				            {
				                corrected_ay -= (receivedVibrationData.ay[index] > 0) ? gravity_correction : -gravity_correction;
				            }
				
				            // Compute final velocity for Y axis
				            float Vo_y = Vi_y + corrected_ay * (interval_ms / 1000.0);
				            snprintf(formattedVibration, sizeof(formattedVibration), "%.8f", Vo_y);
				            cJSON_AddNumberToObject(json_object, "Vy", atof(formattedVibration));
				            Vi_y = Vo_y; // Update initial velocity for the next iteration
				
				            // Correct az if necessary
				            float corrected_az = receivedVibrationData.az[index];
				            if (fabs(receivedVibrationData.az[index] - gravity_correction) <= tolerance ||
				                fabs(receivedVibrationData.az[index] + gravity_correction) <= tolerance)
				            {
				                corrected_az -= (receivedVibrationData.az[index] > 0) ? gravity_correction : -gravity_correction;
				            }
				
				            // Compute final velocity for Z axis
				            float Vo_z = Vi_z + corrected_az * (interval_ms / 1000.0);
				            snprintf(formattedVibration, sizeof(formattedVibration), "%.8f", Vo_z);
				            cJSON_AddNumberToObject(json_object, "Vz", atof(formattedVibration));
				            Vi_z = Vo_z; // Update initial velocity for the next iteration
				
				            cJSON_AddItemToArray(json_array, json_object);
				        }
				
				        char *array_string = cJSON_Print(json_array);
				
				        if (array_string == NULL)
				        {
				            printf("Failed to generate JSON string.\n");
				            cJSON_Delete(json_array); // Clean up JSON array
				            continue;
				        }
				
				        // Publish the JSON array via MQTT
				        esp_err_t result = esp_mqtt_client_publish(client, "vibration", array_string, strlen(array_string), 0, 0);
				        if (result < 0)
				        {
				            ESP_LOGE(TAG, "Failed to publish vibration value: %d", result);
				        }
				
				        free(array_string);
				        cJSON_Delete(json_array);
				    }
				    break;
				}
				
				case DATA_CURRENT : 
				{
					if (CurrentSensorDataQueue == NULL) 
			    	{
			        	ESP_LOGE(TAG, "current data sensor queue null");
			        	break;
			    	}
			    	
					if (xQueueReceive(CurrentSensorDataQueue, &receivedCurrentData, portMAX_DELAY) == pdPASS)
					{
						cJSON *json_object = cJSON_CreateObject();  // Create a new JSON object
						char formattedCurrent[20];
						snprintf(formattedCurrent,sizeof(formattedCurrent), "%.8f", receivedCurrentData.current);

						cJSON_AddNumberToObject(json_object, "current", atof(formattedCurrent));  // Add the "current" field with the value
		
						char *object_string = cJSON_Print(json_object);
						if (object_string == NULL) 
						{
							printf("Failed to generate JSON string.\n");
							cJSON_Delete(json_object); // Clean up JSON array
							break;
						}        
						//printf("\nJSON Object:\n%s\n", object_string);
						
						// Publish Current value
			        	esp_err_t result = esp_mqtt_client_publish(client, "current", object_string, strlen(object_string), 0, 0);
				        if (result < 0)
				        {
				            ESP_LOGE(TAG, "Failed to publish current value : %d",result);	
				        }
				        else
				        {
				           // ESP_LOGI(TAG, "Published current value: %s", object_string);
				        }
					    free(object_string);
					    cJSON_Delete(json_object);
				    }
			        break;
				}
				
				case DATA_TEMPERATURE : 
				{
					#ifdef ANALOG_TEMP		
			        // Read potentiometer value
			        temp_adc_value = read_adc(ADC_UNIT, TEMPERATURE_ADC_CHANNEL);
			        if (temp_adc_value < 0)
			        {
			            continue; // Skip iteration if read failed
			        }
				        
			        cTemp = getTempratureValue(temp_adc_value);
			
			        // Convert ADC value to string
			        int data_len = snprintf(pub_str, sizeof(pub_str), "%0.2f", cTemp);
					#endif
					
					#ifdef DIGITAL_TEMP
					static float prevTemp = -1000.0; // Initialize with an out-of-range value
					float cTemp = ds18b20_get_temp();
					
					// Check if the difference is less than 50
					if (prevTemp == -1000.0 || fabs(cTemp - prevTemp) < 50.0)
					{
					    char pub_str[16]; // Buffer for temperature string
					    int data_len = snprintf(pub_str, sizeof(pub_str), "%.1f", cTemp);
					
					    cJSON *json_object = cJSON_CreateObject();  // Create a new JSON object
					    cJSON_AddNumberToObject(json_object, "temperature", atof(pub_str));  // Add the "temperature" field
					
					    char *object_string = cJSON_Print(json_object);
					    if (object_string == NULL)
					    {
					        printf("Failed to generate JSON string.\n");
					        cJSON_Delete(json_object); // Clean up JSON object
					    }
					    else
					    {
					        //printf("object_string = %s\n", object_string);
					
					        // Publish temperature value
					        esp_err_t result = esp_mqtt_client_publish(client, "temperature", object_string, strlen(object_string), 0, 0);
					        if (result < 0)
					        {
					            ESP_LOGE(TAG, "Failed to publish temperature value: %d", result);
					        }
					
					        free(object_string);
					    }
					    cJSON_Delete(json_object);
					
					    // Update the previous temperature
					    prevTemp = cTemp;
					}
					else
					{
					    printf("Temperature difference too high, not sending data. Current: %.1f, Previous: %.1f\n", cTemp, prevTemp);
					}
					#endif
				}
	
				default:
				{
					break;
				}
			}
		}
    	blink_led();
        vTaskDelay(pdMS_TO_TICKS(20)); // Delay for 10 seconds
    }
}


/*
 * Function: mqtt_app_start
 * ------------------------
 * Initializes and starts the MQTT client, registering the event handler for MQTT events.
 */
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);

    if (client == NULL) 
    {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client,(esp_mqtt_event_id_t) ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));                                                   // Start the client

    // Create the publish task after the client is initialized
    xTaskCreate(&publish_task, "publish_task", 1024*20, client, 5, NULL);
}



// Define the Wi-Fi event handler
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) 
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) 
    {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) 
    {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

extern "C" void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait for Wi-Fi connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdTRUE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) 
    {
        ESP_LOGI(TAG, "Connected to Wi-Fi.");
        mqtt_app_start();
    }
    else 
    {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi.");
    }
}


void blink_led()
{
    gpio_set_level(LED_GPIO_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50)); // LED on for 500 ms
    gpio_set_level(LED_GPIO_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(50)); // LED off for 500 ms
}

// Initialize ADC
void init_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)CURRENT_ADC_CHANNEL, ADC_ATTEN_DB_12); // 0-3.3V range
    adc_chars = (esp_adc_cal_characteristics_t*)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

// Get ADC reading with multisampling
uint32_t get_adc_reading() 
{
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) 
    {
        adc_reading += adc1_get_raw((adc1_channel_t)CURRENT_ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;
    return adc_reading;
}

// Convert ADC reading to voltage in volts
float adc_to_voltage(uint32_t adc_reading) 
{
    uint32_t voltage_mV = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage_mV / 1000.0; // Convert mV to V
}

float calibrate_zero_current() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)CURRENT_ADC_CHANNEL);
    }
    adc_reading /= NO_OF_SAMPLES;

    // Convert ADC value to voltage
    float voltage = adc_to_voltage(adc_reading);
    printf("Calibrated Zero Current Voltage: %.3f V\n", voltage);
    return voltage;
}

// Calculate current from voltage
float calculate_current(float measured_voltage) 
{
    return ((measured_voltage - ACS712CalbrateCurrent) / ACS712_SENSITIVITY); // Current in Amps
}

void CurrentSensorTask(void *parameter)
{
	CurrentSensorData  currentSensorData;

	init_adc();
	ACS712CalbrateCurrent = calibrate_zero_current();
    while (1) 
    {
        uint32_t adc_reading = get_adc_reading();
        float measured_voltage = adc_to_voltage(adc_reading); // Voltage in volts
        currentSensorData.current = calculate_current(measured_voltage);  // Current in amps

        //printf("adc_reading = %u, Measured Voltage: %.2f V, Current: %.2f A\n", adc_reading,measured_voltage,currentSensorData.current );

        if (xQueueSend(CurrentSensorDataQueue, &currentSensorData, portMAX_DELAY) != pdPASS) 
        {
            ESP_LOGE(TAG, "Failed to send current data to queue");
        }
	
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay 100 milisecond
    }
}


void bmiTask(void *parameter)
{
    //TickType_t xLastWakeTime = xTaskGetTickCount(); // Get the current tick count
	//const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 1 ms frequency
	VibrationSensorData  sensorData;
	int index = 0;
	
	while(1)
	{
	    // Read accelerometer data
	    accel.readSensor();
	    accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
	    // Print the Data
	    sensorData.ax[index] = accel.getAccelX_mss();
	    sensorData.ay[index] = accel.getAccelY_mss();
	    sensorData.az[index] = accel.getAccelZ_mss();
        index++;
	    //printf("Time : %lli ,\t Accel X: %f,\t Y: %f,\t Z: %f \n",xx_time_get_time(),sensorData.ax[index], sensorData.ay[index], sensorData.az[index]);

        if (index == NUM_READINGS) 
        {
            if (xQueueSend(VibrationSensorDataQueue, &sensorData, portMAX_DELAY) != pdPASS) 
            {
                ESP_LOGE(TAG, "Failed to send data to queue");
            }
            index = 0; // Reset index for next batch of readings
       }

	    // Read gyroscope data
	    //gyro.readSensor();
	    // Print the data
	    //printf("Gyro X: %f,\t Y: %f,\t Z: %f \n",gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads());
	    //printf("Temperature: %f", accel.getTemperature_C());
	    // Delay for a short period (20 ms)
    	//ets_delay_us(1000);

	    vTaskDelay(pdMS_TO_TICKS(INTERVAL));
        //0vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Ensures task runs at 1 ms intervals

	}
	vTaskDelete(NULL);
}

extern "C" void app_main(void)
{
	TaskHandle_t bmiTaskHandle,ACS712TaskHandle;
    int status;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    VibrationSensorDataQueue = xQueueCreate(10, sizeof(VibrationSensorData));
    if (VibrationSensorDataQueue == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create vibration data queue");
        return;
    }    
    
   CurrentSensorDataQueue = xQueueCreate(10, sizeof(CurrentSensorData));
    if (CurrentSensorDataQueue == NULL) 
    {
        ESP_LOGE(TAG, "Failed to create current data queue");
        return;
    }  
    
    wifi_init_sta();
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    
    // Initialize the accelerometer
    status = accel.begin();
    if (status < 0) 
    {
        printf("Accel Initialization Error: %d", status);
    }
	/*    
	status = gyro.begin();
	if (status < 0) 
  	{
        printf("Gyro Initialization Error %d", status);
    }
    */
    gpio_set_direction(GPIO_NUM_45, GPIO_MODE_OUTPUT);
    
    // Main loop to read and print sensor data
	xTaskCreate(bmiTask, "BMI", 1024*10 , NULL, 5,&bmiTaskHandle);
	xTaskCreate(CurrentSensorTask, "CurrentSensor", 1024 * 5 , NULL, 5,&ACS712TaskHandle);

	//xTaskCreatePinnedToCore(bmiTask, "BMI", 10240* 10, NULL, 5, &bmiTaskHandle, 1);
}