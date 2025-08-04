/**
 * @file sensors.c
 * @brief Sensor management - PIR and Battery only version (for step-by-step testing)
 */

#include "sensors.h"
#include "sensdot_common.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// TODO: Add later when connecting sensors
// #include "bme280.h"
// #include "bh1750.h"

static const char *TAG = "sensors";

// Sensor state
static bool g_sensors_initialized = false;
static bool g_sensors_powered = false;
static sensor_calibration_t g_calibration = {0};

// ADC handles for ESP32-C3
static adc_oneshot_unit_handle_t g_adc_handle = NULL;
static adc_cali_handle_t g_adc_cali_handle = NULL;

// Sensor status tracking
static sensor_status_t g_sensor_status[SENSOR_COUNT] = {SENSOR_STATUS_NOT_CONNECTED};

// Motion detection state (stored in RTC memory)
RTC_DATA_ATTR static time_t last_motion_time = 0;

// Forward declarations
static esp_err_t init_adc_c3(void);
static esp_err_t init_pir_c3(void);
static esp_err_t deinit_adc_c3(void);

/**
 * @brief Initialize ADC for battery monitoring on ESP32-C3
 */
static esp_err_t init_adc_c3(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGD(TAG, "Initializing ADC on ESP32-C3 (GPIO0)");
    
    // Configure ADC unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    ret = adc_oneshot_new_unit(&init_config, &g_adc_handle);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel (GPIO0 on ESP32-C3)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    
    ret = adc_oneshot_config_channel(g_adc_handle, ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(g_adc_handle);
        return ret;
    }
    
    // Initialize ADC calibration for ESP32-C3
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &g_adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &g_adc_cali_handle);
#endif
    
    if (ret != ESP_OK) {
        SENSDOT_LOGW(TAG, "ADC calibration failed, using raw values: %s", esp_err_to_name(ret));
        g_adc_cali_handle = NULL;
    }
    
    SENSDOT_LOGD(TAG, "ADC initialized successfully on ESP32-C3");
    return ESP_OK;
}

/**
 * @brief Initialize PIR sensor for ESP32-C3
 */
static esp_err_t init_pir_c3(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGD(TAG, "Initializing PIR sensor on GPIO4 (ESP32-C3)");
    
    // Configure PIR GPIO as input
    gpio_config_t pir_config = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&pir_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure PIR GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    g_sensor_status[SENSOR_PIR] = SENSOR_STATUS_OK;
    
    SENSDOT_LOGI(TAG, "PIR sensor initialized on GPIO4");
    return ESP_OK;
}

/**
 * @brief Deinitialize ADC
 */
static esp_err_t deinit_adc_c3(void)
{
    if (g_adc_cali_handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(g_adc_cali_handle);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(g_adc_cali_handle);
#endif
        g_adc_cali_handle = NULL;
    }
    
    if (g_adc_handle) {
        adc_oneshot_del_unit(g_adc_handle);
        g_adc_handle = NULL;
    }
    
    return ESP_OK;
}

/**
 * @brief Initialize sensors - PIR and Battery only
 */
esp_err_t sensors_init(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Initializing sensors for ESP32-C3 (PIR + Battery only)");
    
    if (g_sensors_initialized) {
        SENSDOT_LOGW(TAG, "Sensors already initialized");
        return ESP_OK;
    }
    
    // Initialize sensor power GPIO (not used yet, but keep for future)
    gpio_config_t power_config = {
        .pin_bit_mask = (1ULL << SENSOR_PWR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    ret = gpio_config(&power_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure sensor power GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Turn off sensor power initially (will be used later for I2C sensors)
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    g_sensors_powered = false;
    
    // Initialize ADC for battery monitoring
    ret = init_adc_c3();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize ADC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize PIR sensor
    ret = init_pir_c3();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize PIR: %s", esp_err_to_name(ret));
        deinit_adc_c3();
        return ret;
    }
    
    // Initialize calibration with defaults
    sensors_reset_calibration();
    
    // Set sensor status
    g_sensor_status[SENSOR_BATTERY] = SENSOR_STATUS_OK;
    g_sensor_status[SENSOR_PIR] = SENSOR_STATUS_OK;
    g_sensor_status[SENSOR_BME280] = SENSOR_STATUS_NOT_CONNECTED; // Not connected yet
    g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_NOT_CONNECTED; // Not connected yet
    
    g_sensors_initialized = true;
    
    SENSDOT_LOGI(TAG, "Sensors initialized successfully (PIR + Battery)");
    return ESP_OK;
}

/**
 * @brief Deinitialize all sensors
 */
esp_err_t sensors_deinit(void)
{
    if (!g_sensors_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Deinitializing sensors");
    
    // Power off sensors
    sensors_power_off();
    
    // Deinitialize ADC
    deinit_adc_c3();
    
    // Reset sensor status
    for (int i = 0; i < SENSOR_COUNT; i++) {
        g_sensor_status[i] = SENSOR_STATUS_NOT_CONNECTED;
    }
    
    g_sensors_initialized = false;
    
    SENSDOT_LOGI(TAG, "Sensors deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Power on sensors (currently no I2C sensors)
 */
esp_err_t sensors_power_on(void)
{
    if (!g_sensors_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    if (g_sensors_powered) {
        return ESP_OK; // Already powered on
    }
    
    SENSDOT_LOGD(TAG, "Powering on sensors (currently only for future I2C sensors)");
    
    // Turn on sensor power (for future I2C sensors)
    gpio_set_level(SENSOR_PWR_GPIO, 1);
    g_sensors_powered = true;
    
    // Wait for sensors to stabilize (not needed now, but keep for consistency)
    vTaskDelay(pdMS_TO_TICKS(50));
    
    SENSDOT_LOGD(TAG, "Sensors powered on");
    return ESP_OK;
}

/**
 * @brief Power off sensors
 */
esp_err_t sensors_power_off(void)
{
    if (!g_sensors_initialized || !g_sensors_powered) {
        return ESP_OK;
    }
    
    SENSDOT_LOGD(TAG, "Powering off sensors");
    
    // Turn off sensor power
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    g_sensors_powered = false;
    
    SENSDOT_LOGD(TAG, "Sensors powered off");
    return ESP_OK;
}

/**
 * @brief Read BME280 sensor - stub version (returns default values)
 */
esp_err_t sensor_read_bme280(bme280_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    // Return safe default values for missing sensor
    data->temperature = 20.0f + g_calibration.temperature_offset; // Room temperature
    data->humidity = 50.0f + g_calibration.humidity_offset;       // Moderate humidity
    data->pressure = 1013.25f + g_calibration.pressure_offset;   // Sea level pressure
    data->status = SENSOR_STATUS_NOT_CONNECTED;
    
    SENSDOT_LOGD(TAG, "BME280: Not connected, using defaults: T=%.2f°C, H=%.2f%%, P=%.2fhPa", 
                data->temperature, data->humidity, data->pressure);
    
    return SENSDOT_ERR_SENSOR_FAIL; // Indicate sensor not available
}

/**
 * @brief Read BH1750 sensor - stub version (returns default values)
 */
esp_err_t sensor_read_bh1750(bh1750_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    // Return safe default value for missing sensor
    data->illuminance = 100; // Indoor lighting level
    data->status = SENSOR_STATUS_NOT_CONNECTED;
    
    SENSDOT_LOGD(TAG, "BH1750: Not connected, using default: %u lx", data->illuminance);
    
    return SENSDOT_ERR_SENSOR_FAIL; // Indicate sensor not available
}

/**
 * @brief Read PIR sensor - fully functional
 */
esp_err_t sensor_read_pir(pir_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    if (!g_sensors_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    // Read PIR GPIO state
    data->motion_detected = gpio_get_level(PIR_GPIO) == 1;
    data->last_trigger = last_motion_time;
    data->status = g_sensor_status[SENSOR_PIR];
    
    SENSDOT_LOGD(TAG, "PIR: %s", data->motion_detected ? "Motion detected" : "No motion");
    
    return ESP_OK;
}

/**
 * @brief Read battery voltage - fully functional
 */
esp_err_t sensor_read_battery(battery_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    if (!g_sensors_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    int raw_value;
    
    // Read raw ADC value
    ret = adc_oneshot_read(g_adc_handle, ADC_CHANNEL, &raw_value);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
        data->status = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    data->raw_adc = raw_value;
    
    // Convert to voltage
    if (g_adc_cali_handle) {
        int voltage_mv;
        ret = adc_cali_raw_to_voltage(g_adc_cali_handle, raw_value, &voltage_mv);
        if (ret == ESP_OK) {
            data->voltage = voltage_mv / 1000.0f; // Convert mV to V
        } else {
            // Fallback calculation for ESP32-C3
            data->voltage = ((float)raw_value / 4095.0f) * 3.3f;
        }
    } else {
        // Simple calculation without calibration
        data->voltage = ((float)raw_value / 4095.0f) * 3.3f;
    }
    
    // Apply voltage divider if enabled
    const device_config_t *config = config_get_current();
    if (config->battery_divider_enabled) {
        data->voltage *= BATT_DIVIDER_RATIO;
        SENSDOT_LOGD(TAG, "Applied voltage divider ratio: %.1f", BATT_DIVIDER_RATIO);
    } else {
        SENSDOT_LOGD(TAG, "Direct voltage measurement (no divider)");
    }
    
    // Apply calibration
    data->voltage += g_calibration.battery_calibration;
    
    // Determine low battery status using config threshold
    data->low_battery = data->voltage < config->low_batt_threshold;
    data->status = g_sensor_status[SENSOR_BATTERY];
    
    SENSDOT_LOGD(TAG, "Battery: %.2fV (raw=%d, divider=%s, low=%s)", 
                data->voltage, raw_value, 
                config->battery_divider_enabled ? "ON" : "OFF",
                data->low_battery ? "yes" : "no");
    
    return ESP_OK;
}

/**
 * @brief Read all sensor data - PIR + Battery working, others return defaults
 */
esp_err_t sensors_read_all(sensor_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    esp_err_t ret = ESP_OK;
    bme280_data_t bme_data;
    bh1750_data_t bh_data;
    pir_data_t pir_data;
    battery_data_t batt_data;
    
    SENSDOT_LOGD(TAG, "Reading all sensors (PIR + Battery active)");
    
    // Initialize data structure
    memset(data, 0, sizeof(sensor_data_t));
    data->timestamp = time(NULL);
    
    // Read BME280 (returns defaults since not connected)
    sensor_read_bme280(&bme_data);
    data->temperature = bme_data.temperature;
    data->humidity = bme_data.humidity;
    data->pressure = bme_data.pressure;
    
    // Read BH1750 (returns defaults since not connected)
    sensor_read_bh1750(&bh_data);
    data->light = bh_data.illuminance;
    
    // Read PIR (fully functional)
    if (sensor_read_pir(&pir_data) == ESP_OK) {
        data->motion = pir_data.motion_detected;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read PIR");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    // Read battery (fully functional)
    if (sensor_read_battery(&batt_data) == ESP_OK) {
        data->battery_voltage = batt_data.voltage;
        data->low_battery = batt_data.low_battery;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read battery");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    SENSDOT_LOGI(TAG, "Sensor data: T=%.2f°C*, H=%.2f%%*, P=%.2fhPa*, L=%ulx*, M=%s, B=%.2fV (* = default values)", 
                data->temperature, data->humidity, data->pressure, data->light,
                data->motion ? "YES" : "NO", data->battery_voltage);
    
    return ret;
}

/**
 * @brief Check if specific sensor is available
 */
bool sensor_is_available(sensor_type_t sensor_type)
{
    if (sensor_type >= SENSOR_COUNT) {
        return false;
    }
    
    return g_sensor_status[sensor_type] == SENSOR_STATUS_OK;
}

/**
 * @brief Get sensor status
 */
sensor_status_t sensor_get_status(sensor_type_t sensor_type)
{
    if (sensor_type >= SENSOR_COUNT) {
        return SENSOR_STATUS_ERROR;
    }
    
    return g_sensor_status[sensor_type];
}

/**
 * @brief Set sensor calibration data
 */
esp_err_t sensors_set_calibration(const sensor_calibration_t *calibration)
{
    CHECK_NULL_PTR(calibration);
    
    memcpy(&g_calibration, calibration, sizeof(sensor_calibration_t));
    
    SENSDOT_LOGI(TAG, "Sensor calibration updated");
    return ESP_OK;
}

/**
 * @brief Get sensor calibration data
 */
esp_err_t sensors_get_calibration(sensor_calibration_t *calibration)
{
    CHECK_NULL_PTR(calibration);
    
    memcpy(calibration, &g_calibration, sizeof(sensor_calibration_t));
    return ESP_OK;
}

/**
 * @brief Reset sensor calibration to defaults
 */
esp_err_t sensors_reset_calibration(void)
{
    memset(&g_calibration, 0, sizeof(sensor_calibration_t));
    
    SENSDOT_LOGI(TAG, "Sensor calibration reset to defaults");
    return ESP_OK;
}

/**
 * @brief Perform sensor self-test
 */
esp_err_t sensors_self_test(void)
{
    SENSDOT_LOGI(TAG, "Performing sensor self-test (PIR + Battery)");
    
    esp_err_t ret = ESP_OK;
    bool was_powered = g_sensors_powered;
    
    // Power on sensors if needed
    if (!g_sensors_powered) {
        sensors_power_on();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Test available sensors only
    sensor_data_t test_data;
    if (sensors_read_all(&test_data) != ESP_OK) {
        SENSDOT_LOGE(TAG, "Sensor self-test failed");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    } else {
        SENSDOT_LOGI(TAG, "Sensor self-test passed (available sensors working)");
    }
    
    // Restore power state
    if (!was_powered) {
        sensors_power_off();
    }
    
    return ret;
}

/**
 * @brief Get sensor information string
 */
esp_err_t sensor_get_info(sensor_type_t sensor_type, char *info_str, size_t max_len)
{
    CHECK_NULL_PTR(info_str);
    
    const char *sensor_names[] = {
        "BME280 (Temperature, Humidity, Pressure)",
        "BH1750 (Light)",
        "PIR (Motion)",
        "ADC (Battery)"
    };
    
    const char *status_names[] = {
        "OK", "Error", "Not Connected", "Calibrating"
    };
    
    if (sensor_type >= SENSOR_COUNT) {
        strncpy(info_str, "Unknown sensor", max_len - 1);
    } else {
        snprintf(info_str, max_len, "%s - Status: %s", 
                sensor_names[sensor_type],
                status_names[g_sensor_status[sensor_type]]);
    }
    
    info_str[max_len - 1] = '\0';
    return ESP_OK;
}

/**
 * @brief Check if motion alarm should be triggered
 */
bool sensor_should_trigger_alarm(time_t current_time, int alarm_hold_sec)
{
    return (current_time - last_motion_time) > alarm_hold_sec;
}

/**
 * @brief Update motion alarm timestamp
 */
void sensor_update_alarm_time(time_t trigger_time)
{
    last_motion_time = trigger_time;
}

/**
 * @brief Get last motion detection time
 */
time_t sensor_get_last_motion_time(void)
{
    return last_motion_time;
}

/**
 * @brief Check if battery is low (using config threshold)
 */
bool sensor_is_battery_low(float voltage)
{
    const device_config_t *config = config_get_current();
    return voltage < config->low_batt_threshold;
}

/**
 * @brief Get battery percentage estimate
 */
uint8_t sensor_get_battery_percentage(float voltage)
{
    // Optimized mapping for Li-ion battery
    // 4.2V = 100%, 3.0V = 0%
    const float max_voltage = 4.2f;
    const float min_voltage = 3.0f;
    
    if (voltage >= max_voltage) return 100;
    if (voltage <= min_voltage) return 0;
    
    float percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;
    return (uint8_t)CLAMP(percentage, 0, 100);
}