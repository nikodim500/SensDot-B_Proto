/**
 * @file sensors.h
 * @brief Sensor management for SensDot device
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "sensdot_common.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Sensor types
typedef enum {
    SENSOR_BME280,
    SENSOR_BH1750,
    SENSOR_PIR,
    SENSOR_BATTERY,
    SENSOR_COUNT
} sensor_type_t;

// Sensor status
typedef enum {
    SENSOR_STATUS_OK,
    SENSOR_STATUS_ERROR,
    SENSOR_STATUS_NOT_CONNECTED,
    SENSOR_STATUS_CALIBRATING
} sensor_status_t;

// Individual sensor data structures
typedef struct {
    float temperature;
    float humidity;
    float pressure;
    sensor_status_t status;
} bme280_data_t;

typedef struct {
    uint16_t illuminance;
    sensor_status_t status;
} bh1750_data_t;

typedef struct {
    bool motion_detected;
    time_t last_trigger;
    sensor_status_t status;
} pir_data_t;

typedef struct {
    float voltage;
    bool low_battery;
    uint16_t raw_adc;
    sensor_status_t status;
} battery_data_t;

// Sensor calibration data
typedef struct {
    float temperature_offset;
    float humidity_offset;
    float pressure_offset;
    float battery_calibration;
} sensor_calibration_t;

/**
 * @brief Initialize all sensors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_init(void);

/**
 * @brief Deinitialize all sensors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_deinit(void);

/**
 * @brief Power on sensors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_power_on(void);

/**
 * @brief Power off sensors
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_power_off(void);

/**
 * @brief Read all sensor data
 * @param data Pointer to sensor data structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_read_all(sensor_data_t *data);

/**
 * @brief Read BME280 sensor (temperature, humidity, pressure)
 * @param data Pointer to BME280 data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_read_bme280(bme280_data_t *data);

/**
 * @brief Read BH1750 sensor (light)
 * @param data Pointer to BH1750 data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_read_bh1750(bh1750_data_t *data);

/**
 * @brief Read PIR sensor (motion)
 * @param data Pointer to PIR data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_read_pir(pir_data_t *data);

/**
 * @brief Read battery voltage
 * @param data Pointer to battery data structure
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_read_battery(battery_data_t *data);

/**
 * @brief Check if specific sensor is available
 * @param sensor_type Type of sensor to check
 * @return true if available, false otherwise
 */
bool sensor_is_available(sensor_type_t sensor_type);

/**
 * @brief Get sensor status
 * @param sensor_type Type of sensor
 * @return Sensor status
 */
sensor_status_t sensor_get_status(sensor_type_t sensor_type);

/**
 * @brief Set sensor calibration data
 * @param calibration Pointer to calibration data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_set_calibration(const sensor_calibration_t *calibration);

/**
 * @brief Get sensor calibration data
 * @param calibration Pointer to store calibration data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_get_calibration(sensor_calibration_t *calibration);

/**
 * @brief Reset sensor calibration to defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensors_reset_calibration(void);

/**
 * @brief Perform sensor self-test
 * @return ESP_OK if all sensors pass, error code otherwise
 */
esp_err_t sensors_self_test(void);

/**
 * @brief Get sensor information string
 * @param sensor_type Type of sensor
 * @param info_str Buffer to store information string
 * @param max_len Maximum length of info string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t sensor_get_info(sensor_type_t sensor_type, char *info_str, size_t max_len);

/**
 * @brief Check if motion alarm should be triggered
 * @param current_time Current timestamp
 * @param alarm_hold_sec Alarm hold time in seconds
 * @return true if alarm should be triggered, false otherwise
 */
bool sensor_should_trigger_alarm(time_t current_time, int alarm_hold_sec);

/**
 * @brief Update motion alarm timestamp
 * @param trigger_time Timestamp when motion was detected
 */
void sensor_update_alarm_time(time_t trigger_time);

/**
 * @brief Get last motion detection time
 * @return Last motion detection timestamp
 */
time_t sensor_get_last_motion_time(void);

/**
 * @brief Check if battery is low
 * @param threshold Low battery threshold in volts
 * @return true if battery is low, false otherwise
 */
bool sensor_is_battery_low(float threshold);

/**
 * @brief Get battery percentage estimate
 * @param voltage Current battery voltage
 * @return Battery percentage (0-100)
 */
uint8_t sensor_get_battery_percentage(float voltage);

#ifdef __cplusplus
}
#endif

#endif // SENSORS_H