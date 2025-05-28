
// main.c — logic for sleep, wakeup, sensor reading, and MQTT reporting for ESP32 (ESP-32S)

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "mqtt_client.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "i2cdev.h"
#include "bh1750.h"
#include "bme280.h"

// GPIO assignments
#define PIR_GPIO 4
#define LED_GPIO 2
#define BUZZER_GPIO 17
#define SENSOR_PWR_GPIO 18
#define BATTERY_ADC_GPIO 34
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22

#define TAG "sensdot"

// Alarm state stored in RTC memory across deep sleep cycles
RTC_DATA_ATTR static time_t last_alarm_time = 0;

// Configuration constants (from sdkconfig)
#define WAKE_INTERVAL_SEC CONFIG_SENS_WAKE_INTERVAL_SEC
#define ALARM_HOLD_SEC CONFIG_SENS_ALARM_HOLD_SEC
#define MQTT_URI CONFIG_SENS_MQTT_URI
#define MQTT_TOPIC_PREFIX CONFIG_SENS_TOPIC_PREFIX
#define WIFI_SSID CONFIG_SENS_WIFI_SSID
#define WIFI_PASS CONFIG_SENS_WIFI_PASS
#define LOW_BATT_THRESHOLD 3.3f          // Voltage threshold for low battery warning
#define BATT_DIVIDER_RATIO 2.0f          // Voltage divider ratio

static esp_mqtt_client_handle_t mqtt_client = NULL;

// Blink LED and beep buzzer three times (used for low battery warning only)
static void blink_and_beep() {
    for (int i = 0; i < 3; i++) {
        gpio_set_level(LED_GPIO, 1);
        gpio_set_level(BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        gpio_set_level(BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Measure battery voltage via ADC (GPIO34) and return result in volts
static float read_battery_voltage() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO34 = ADC1_CH6
    int raw = adc1_get_raw(ADC1_CHANNEL_6);
    float voltage = ((float)raw / 4095.0f) * 3.3f * BATT_DIVIDER_RATIO;
    return voltage;
}

// Publish to topic relative to base prefix
static void mqtt_publish_rel(const char *topic_suffix, const char *value) {
    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s/%s", MQTT_TOPIC_PREFIX, topic_suffix);
    mqtt_publish(full_topic, value);
}

// Publish to full MQTT topic
static void mqtt_publish(const char *topic, const char *value) {
    if (mqtt_client) {
        esp_mqtt_client_publish(mqtt_client, topic, value, 0, 1, 0);
    }
}

// Register MQTT discovery configurations for Home Assistant
static void mqtt_discovery() {
    mqtt_publish("homeassistant/sensor/sensdot_temp/config",
        "{"name":"SensDot Temp","state_topic":"sensdot/temp","unit_of_measurement":"°C","device_class":"temperature"}");
    mqtt_publish("homeassistant/sensor/sensdot_hum/config",
        "{"name":"SensDot Humidity","state_topic":"sensdot/hum","unit_of_measurement":"%","device_class":"humidity"}");
    mqtt_publish("homeassistant/sensor/sensdot_press/config",
        "{"name":"SensDot Pressure","state_topic":"sensdot/press","unit_of_measurement":"hPa","device_class":"pressure"}");
    mqtt_publish("homeassistant/sensor/sensdot_light/config",
        "{"name":"SensDot Light","state_topic":"sensdot/light","unit_of_measurement":"lx","device_class":"illuminance"}");
    mqtt_publish("homeassistant/binary_sensor/sensdot_alarm/config",
        "{"name":"SensDot Alarm","state_topic":"sensdot/alarm","device_class":"motion","payload_on":"on","payload_off":"off"}");
    mqtt_publish("homeassistant/sensor/sensdot_battery/config",
        "{"name":"SensDot Battery","state_topic":"sensdot/battery","unit_of_measurement":"V","device_class":"voltage"}");
    mqtt_publish("homeassistant/binary_sensor/sensdot_low_batt/config",
        "{"name":"SensDot Low Battery","state_topic":"sensdot/low_batt","device_class":"battery","payload_on":"on","payload_off":"off"}");
}
