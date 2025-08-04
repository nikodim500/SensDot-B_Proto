/**
 * @file sensdot_common.h
 * @brief Common definitions, constants and data structures for SensDot project (ESP32-C3 Super Mini)
 */

#ifndef SENSDOT_COMMON_H
#define SENSDOT_COMMON_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"

// Project version
#define SENSDOT_VERSION_MAJOR 1
#define SENSDOT_VERSION_MINOR 0
#define SENSDOT_VERSION_PATCH 0
#define SENSDOT_VERSION_STRING "1.0.0"

// GPIO assignments for ESP32-C3 Super Mini
#define PIR_GPIO          4   // Motion sensor (RTC wake-up capable)
#define LED_GPIO          1   // Status LED (safe pin)
#define BUZZER_GPIO       10  // Buzzer/sound signal
#define SENSOR_PWR_GPIO   3   // Sensor power control
#define BATTERY_ADC_GPIO  0   // Battery ADC (ADC1_CH0)
#define I2C_SDA_GPIO      21  // I2C data
#define I2C_SCL_GPIO      5   // I2C clock

// Built-in hardware on ESP32-C3 Super Mini
#define BUILTIN_LED_GPIO  8   // Built-in LED (inverted logic)
#define BOOT_BUTTON_GPIO  9   // Built-in BOOT button (optional for user input)

// I2C configuration
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 100000

// Battery monitoring for ESP32-C3
#define BATT_DIVIDER_RATIO (CONFIG_SENS_VOLTAGE_DIVIDER_RATIO / 100.0f)
#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO0 on ESP32-C3

// Timing constants
#define SETUP_TIMEOUT_MS (CONFIG_SENS_SETUP_TIMEOUT_MIN * 60 * 1000)
#define MAX_WIFI_RETRIES CONFIG_SENS_MAX_WIFI_RETRIES
#define MQTT_DISCOVERY_INTERVAL CONFIG_SENS_MQTT_DISCOVERY_INTERVAL

// Setup mode constants  
#define SETUP_AP_SSID_PREFIX "SensDot-Setup-"
#define SETUP_AP_PASSWORD CONFIG_SENS_SETUP_AP_PASSWORD
#define SETUP_AP_MAX_CONNECTIONS 1
#define SETUP_AP_IP "192.168.4.1"

// NVS storage keys
#define NVS_NAMESPACE "sensdot_cfg"
#define NVS_WIFI_SSID "wifi_ssid"
#define NVS_WIFI_PASS "wifi_pass"
#define NVS_MQTT_URI "mqtt_uri"
#define NVS_MQTT_USER "mqtt_user"
#define NVS_MQTT_PASS "mqtt_pass"
#define NVS_MQTT_PREFIX "mqtt_prefix"
#define NVS_WAKE_INTERVAL "wake_interval"
#define NVS_ALARM_HOLD "alarm_hold"
#define NVS_LOW_BATT_THRESH "low_batt_thresh"
#define NVS_RETRY_SLEEP_SEC "retry_sleep_sec"
#define NVS_BATTERY_DIVIDER "batt_divider"
#define NVS_CONFIGURED "configured"

// String length limits
#define MAX_SSID_LEN 32
#define MAX_PASSWORD_LEN 64
#define MAX_URI_LEN 128
#define MAX_USERNAME_LEN 32
#define MAX_TOPIC_LEN 32
#define MAX_DEVICE_ID_LEN 32

// Device states
typedef enum {
    DEVICE_STATE_INIT,
    DEVICE_STATE_SETUP,
    DEVICE_STATE_CONNECTING,
    DEVICE_STATE_NORMAL,
    DEVICE_STATE_SLEEP,
    DEVICE_STATE_ERROR
} device_state_t;

// Wake up reasons
typedef enum {
    WAKEUP_REASON_TIMER,
    WAKEUP_REASON_PIR,
    WAKEUP_REASON_RESET,
    WAKEUP_REASON_UNKNOWN
} wakeup_reason_t;

// Sensor data structure
typedef struct {
    float temperature;      // °C
    float humidity;        // %
    float pressure;        // hPa
    uint16_t light;        // lx
    bool motion;           // PIR state
    float battery_voltage; // V
    bool low_battery;      // Low battery flag
    time_t timestamp;      // Unix timestamp
} sensor_data_t;

// Device configuration structure
typedef struct {
    char wifi_ssid[MAX_SSID_LEN];
    char wifi_pass[MAX_PASSWORD_LEN];
    char mqtt_uri[MAX_URI_LEN];
    char mqtt_user[MAX_USERNAME_LEN];
    char mqtt_pass[MAX_PASSWORD_LEN];
    char mqtt_prefix[MAX_TOPIC_LEN];
    int wake_interval_sec;
    int alarm_hold_sec;
    float low_batt_threshold;
    int retry_sleep_sec;        // Sleep interval when retrying after errors
    bool battery_divider_enabled;  // Battery voltage divider present
    bool configured;
} device_config_t;

// WiFi scan result
typedef struct {
    char ssid[MAX_SSID_LEN];
    int8_t rssi;
    bool auth_required;
} wifi_scan_result_t;

// Error codes
typedef enum {
    SENSDOT_OK = 0,
    SENSDOT_ERR_INVALID_ARG = -1,
    SENSDOT_ERR_NO_MEM = -2,
    SENSDOT_ERR_TIMEOUT = -3,
    SENSDOT_ERR_NOT_FOUND = -4,
    SENSDOT_ERR_INVALID_STATE = -5,
    SENSDOT_ERR_WIFI_FAIL = -6,
    SENSDOT_ERR_MQTT_FAIL = -7,
    SENSDOT_ERR_SENSOR_FAIL = -8,
    SENSDOT_ERR_CONFIG_FAIL = -9
} sensdot_err_t;

// Status indication patterns
typedef enum {
    STATUS_READY = 1,           // 1 blink
    STATUS_CONNECTING = 2,      // 2 blinks  
    STATUS_SUCCESS = 3,         // 3 blinks
    STATUS_ERROR = 4,           // 4 blinks
    STATUS_FACTORY_RESET = 5    // 5 blinks
} status_pattern_t;

// Logging macros with module tags
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#define SENSDOT_LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#define SENSDOT_LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
#define SENSDOT_LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
#define SENSDOT_LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
#define SENSDOT_LOGV(tag, format, ...) ESP_LOGV(tag, format, ##__VA_ARGS__)

// Utility macros
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define CLAMP(val, min, max) MAX(min, MIN(max, val))

// Bit manipulation helpers
#define SET_BIT(reg, bit) ((reg) |= (1 << (bit)))
#define CLEAR_BIT(reg, bit) ((reg) &= ~(1 << (bit)))
#define GET_BIT(reg, bit) (((reg) >> (bit)) & 1)

// Time helpers
#define MS_TO_TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define SECONDS_TO_US(sec) ((sec) * 1000000ULL)
#define MINUTES_TO_MS(min) ((min) * 60 * 1000)

// Default configuration values
#define DEFAULT_WAKE_INTERVAL_SEC CONFIG_SENS_WAKE_INTERVAL_SEC
#define DEFAULT_ALARM_HOLD_SEC CONFIG_SENS_ALARM_HOLD_SEC
#define DEFAULT_LOW_BATT_THRESHOLD (CONFIG_SENS_LOW_BATT_THRESHOLD / 1000.0f)
#define DEFAULT_BATTERY_DIVIDER_ENABLED CONFIG_SENS_BATTERY_DIVIDER_DEFAULT
#define DEFAULT_MQTT_PREFIX "sensdot"
#define DEFAULT_MQTT_URI "mqtt://192.168.1.100:1883"
#define DEFAULT_RETRY_SLEEP_SEC 60  // Default retry sleep interval

// Hardware validation macros
#define CHECK_GPIO_VALID(gpio) do { \
    if (!GPIO_IS_VALID_GPIO(gpio)) { \
        SENSDOT_LOGE("GPIO", "Invalid GPIO number: %d", gpio); \
        return SENSDOT_ERR_INVALID_ARG; \
    } \
} while(0)

#define CHECK_NULL_PTR(ptr) do { \
    if ((ptr) == NULL) { \
        SENSDOT_LOGE("NULL_CHECK", "Null pointer detected"); \
        return SENSDOT_ERR_INVALID_ARG; \
    } \
} while(0)

// Function result checking
#define CHECK_ERROR_RETURN(x) do { \
    esp_err_t err_rc_ = (x); \
    if (err_rc_ != ESP_OK) { \
        SENSDOT_LOGE("ERROR_CHECK", "Error at %s:%d: %s", __FILE__, __LINE__, esp_err_to_name(err_rc_)); \
        return err_rc_; \
    } \
} while(0)

// Memory allocation helpers
#define MALLOC_CHECK(ptr, size) do { \
    (ptr) = malloc(size); \
    if ((ptr) == NULL) { \
        SENSDOT_LOGE("MALLOC", "Failed to allocate %zu bytes", (size_t)(size)); \
        return SENSDOT_ERR_NO_MEM; \
    } \
} while(0)

#define FREE_AND_NULL(ptr) do { \
    if ((ptr) != NULL) { \
        free(ptr); \
        (ptr) = NULL; \
    } \
} while(0)

#endif // SENSDOT_COMMON_H