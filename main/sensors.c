/**
 * @file sensors.c
 * @brief Sensor management implementation for SensDot device (ESP32-C3 + BMP280/BME280)
 */

#include "sensors.h"
#include "sensdot_common.h"
#include "config.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "sensors";

// Sensor state
static bool g_sensors_initialized = false;
static bool g_sensors_powered = false;
static sensor_calibration_t g_calibration = {0};

// ADC handles
static adc_oneshot_unit_handle_t g_adc_handle = NULL;
static adc_cali_handle_t g_adc_cali_handle = NULL;

// Sensor device handles
static bmp280_t g_bmp280_dev;
static i2c_dev_t g_bh1750_dev;

// Sensor status tracking
static sensor_status_t g_sensor_status[SENSOR_COUNT] = {SENSOR_STATUS_NOT_CONNECTED};

// Sensor capabilities
static bool g_has_humidity = false; // true if BME280, false if BMP280

// Motion detection state (stored in RTC memory)
RTC_DATA_ATTR static time_t last_motion_time = 0;

// Forward declarations
static esp_err_t init_adc(void);
static esp_err_t init_i2c(void);
static esp_err_t init_bmp280(void);
static esp_err_t init_bh1750(void);
static esp_err_t init_pir(void);
static esp_err_t deinit_adc(void);
static esp_err_t deinit_i2c(void);

/**
 * @brief Initialize ADC for battery monitoring
 */
static esp_err_t init_adc(void)
{
    esp_err_t ret;
    
    // Configure ADC
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    
    ret = adc_oneshot_new_unit(&init_config, &g_adc_handle);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channel for ESP32-C3
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
    
    return ESP_OK;
}

/**
 * @brief Initialize I2C bus
 */
static esp_err_t init_i2c(void)
{
    esp_err_t ret;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize BMP280/BME280 sensor
 */
static esp_err_t init_bmp280(void)
{
    esp_err_t ret;
    
    // Initialize BMP280 device descriptor
    memset(&g_bmp280_dev, 0, sizeof(bmp280_t));
    g_bmp280_dev.i2c_dev.port = I2C_MASTER_NUM;
    g_bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;  // Default I2C address (0x76)
    
    // Initialize I2C descriptor
    ret = i2c_dev_create_mutex(&g_bmp280_dev.i2c_dev);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to create BMP280 I2C mutex: %s", esp_err_to_name(ret));
        g_sensor_status[SENSOR_BMP280] = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    // Initialize BMP280/BME280 - the driver auto-detects the sensor type
    ret = bmp280_init(&g_bmp280_dev);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize BMP280/BME280: %s", esp_err_to_name(ret));
        g_sensor_status[SENSOR_BMP280] = SENSOR_STATUS_ERROR;
        i2c_dev_delete_mutex(&g_bmp280_dev.i2c_dev);
        return ret;
    }
    
    // Check if BME280 (with humidity) was detected
    uint8_t chip_id = 0;
    ret = bmp280_get_chip_id(&g_bmp280_dev, &chip_id);
    if (ret == ESP_OK) {
        if (chip_id == 0x60) {
            g_has_humidity = true;
            SENSDOT_LOGI(TAG, "BME280 detected (with humidity support)");
        } else if (chip_id == 0x58) {
            g_has_humidity = false;
            SENSDOT_LOGI(TAG, "BMP280 detected (no humidity support)");
        } else {
            SENSDOT_LOGW(TAG, "Unknown sensor chip ID: 0x%02X", chip_id);
        }
    }
    
    // Configure BMP280/BME280 for low power operation
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;     // Forced mode for energy efficiency
    params.filter = BMP280_FILTER_2;      // Light filtering
    params.oversampling_pressure = BMP280_STANDARD;
    params.oversampling_temperature = BMP280_STANDARD;
    params.standby = BMP280_STANDBY_1000; // 1 second standby
    
    // Set humidity oversampling for BME280
    if (g_has_humidity) {
        params.oversampling_humidity = BMP280_STANDARD;
    }
    
    ret = bmp280_set_params(&g_bmp280_dev, &params);
    if (ret != ESP_OK) {
        SENSDOT_LOGW(TAG, "Failed to set BMP280/BME280 params: %s", esp_err_to_name(ret));
    }
    
    g_sensor_status[SENSOR_BMP280] = SENSOR_STATUS_OK;
    SENSDOT_LOGI(TAG, "BMP280/BME280 sensor initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize BH1750 sensor
 */
static esp_err_t init_bh1750(void)
{
    esp_err_t ret;
    
    // Initialize BH1750 device descriptor
    memset(&g_bh1750_dev, 0, sizeof(i2c_dev_t));
    g_bh1750_dev.port = I2C_MASTER_NUM;
    g_bh1750_dev.addr = BH1750_ADDR_LO;  // Default I2C address (0x23)
    
    // Initialize I2C descriptor
    ret = i2c_dev_create_mutex(&g_bh1750_dev);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to create BH1750 I2C mutex: %s", esp_err_to_name(ret));
        g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    // Initialize BH1750 in one-time high resolution mode for energy efficiency
    ret = bh1750_setup(&g_bh1750_dev, BH1750_MODE_ONE_TIME_HIGH_RES);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize BH1750: %s", esp_err_to_name(ret));
        g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_ERROR;
        i2c_dev_delete_mutex(&g_bh1750_dev);
        return ret;
    }
    
    g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_OK;
    SENSDOT_LOGI(TAG, "BH1750 sensor initialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize PIR sensor
 */
static esp_err_t init_pir(void)
{
    esp_err_t ret;
    
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
    
    SENSDOT_LOGI(TAG, "PIR sensor initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize ADC
 */
static esp_err_t deinit_adc(void)
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
 * @brief Deinitialize I2C
 */
static esp_err_t deinit_i2c(void)
{
    return i2c_driver_delete(I2C_MASTER_NUM);
}

/**
 * @brief Initialize all sensors
 */
esp_err_t sensors_init(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Initializing sensors");
    
    if (g_sensors_initialized) {
        SENSDOT_LOGW(TAG, "Sensors already initialized");
        return ESP_OK;
    }
    
    // Initialize sensor power GPIO
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
    
    // Turn off sensor power initially
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    g_sensors_powered = false;
    
    // Initialize I2C bus
    ret = init_i2c();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ADC for battery monitoring
    ret = init_adc();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize ADC: %s", esp_err_to_name(ret));
        deinit_i2c();
        return ret;
    }
    
    // Initialize PIR sensor
    ret = init_pir();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize PIR: %s", esp_err_to_name(ret));
        deinit_adc();
        deinit_i2c();
        return ret;
    }
    
    // Initialize calibration with defaults
    sensors_reset_calibration();
    
    g_sensors_initialized = true;
    g_sensor_status[SENSOR_BATTERY] = SENSOR_STATUS_OK;
    
    SENSDOT_LOGI(TAG, "Sensors initialized successfully");
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
    
    // Deinitialize I2C device mutexes
    if (g_sensor_status[SENSOR_BMP280] == SENSOR_STATUS_OK) {
        i2c_dev_delete_mutex(&g_bmp280_dev.i2c_dev);
    }
    if (g_sensor_status[SENSOR_BH1750] == SENSOR_STATUS_OK) {
        i2c_dev_delete_mutex(&g_bh1750_dev);
    }
    
    // Deinitialize ADC
    deinit_adc();
    
    // Deinitialize I2C
    deinit_i2c();
    
    // Reset sensor status
    for (int i = 0; i < SENSOR_COUNT; i++) {
        g_sensor_status[i] = SENSOR_STATUS_NOT_CONNECTED;
    }
    
    g_sensors_initialized = false;
    
    SENSDOT_LOGI(TAG, "Sensors deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Power on sensors
 */
esp_err_t sensors_power_on(void)
{
    if (!g_sensors_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    if (g_sensors_powered) {
        return ESP_OK; // Already powered on
    }
    
    SENSDOT_LOGD(TAG, "Powering on sensors");
    
    // Turn on sensor power
    gpio_set_level(SENSOR_PWR_GPIO, 1);
    g_sensors_powered = true;
    
    // Wait for sensors to stabilize (longer for BMP280/BME280)
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize I2C sensors
    esp_err_t ret_bmp = init_bmp280();
    esp_err_t ret_bh = init_bh1750();
    
    if (ret_bmp != ESP_OK) {
        SENSDOT_LOGW(TAG, "BMP280/BME280 initialization failed: %s", esp_err_to_name(ret_bmp));
        g_sensor_status[SENSOR_BMP280] = SENSOR_STATUS_ERROR;
    }
    
    if (ret_bh != ESP_OK) {
        SENSDOT_LOGW(TAG, "BH1750 initialization failed: %s", esp_err_to_name(ret_bh));
        g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_ERROR;
    }
    
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
    
    // Delete I2C device mutexes before power off
    if (g_sensor_status[SENSOR_BMP280] == SENSOR_STATUS_OK) {
        i2c_dev_delete_mutex(&g_bmp280_dev.i2c_dev);
    }
    if (g_sensor_status[SENSOR_BH1750] == SENSOR_STATUS_OK) {
        i2c_dev_delete_mutex(&g_bh1750_dev);
    }
    
    // Turn off sensor power
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    g_sensors_powered = false;
    
    // Update sensor status
    g_sensor_status[SENSOR_BMP280] = SENSOR_STATUS_NOT_CONNECTED;
    g_sensor_status[SENSOR_BH1750] = SENSOR_STATUS_NOT_CONNECTED;
    
    SENSDOT_LOGD(TAG, "Sensors powered off");
    return ESP_OK;
}

/**
 * @brief Read BMP280/BME280 sensor
 */
esp_err_t sensor_read_bmp280(bmp280_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    if (!g_sensors_powered) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    float temperature, pressure, humidity = 0.0f;
    
    // Read BMP280/BME280 data
    if (g_has_humidity) {
        // BME280 - read temperature, pressure, and humidity
        ret = bmp280_read_float(&g_bmp280_dev, &temperature, &pressure, &humidity);
    } else {
        // BMP280 - read temperature and pressure only
        ret = bmp280_read_float(&g_bmp280_dev, &temperature, &pressure, NULL);
    }
    
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to read BMP280/BME280: %s", esp_err_to_name(ret));
        data->status = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    // Apply calibration
    data->temperature = temperature + g_calibration.temperature_offset;
    data->pressure = pressure + g_calibration.pressure_offset;
    data->humidity = humidity + g_calibration.humidity_offset;
    data->has_humidity = g_has_humidity;
    data->status = g_sensor_status[SENSOR_BMP280];
    
    SENSDOT_LOGD(TAG, "%s: T=%.2f°C, P=%.2fhPa%s", 
                g_has_humidity ? "BME280" : "BMP280",
                data->temperature, data->pressure,
                g_has_humidity ? ", H=%.2f%%" : "");
    
    return ESP_OK;
}

/**
 * @brief Read BH1750 sensor
 */
esp_err_t sensor_read_bh1750(bh1750_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    if (!g_sensors_powered) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    uint16_t illuminance;
    
    // Trigger new measurement in one-time mode
    ret = bh1750_setup(&g_bh1750_dev, BH1750_MODE_ONE_TIME_HIGH_RES);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to setup BH1750 measurement: %s", esp_err_to_name(ret));
        data->status = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    // Wait for measurement to complete (typical 120ms for high resolution)
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // Read BH1750 data
    ret = bh1750_read(&g_bh1750_dev, &illuminance);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to read BH1750: %s", esp_err_to_name(ret));
        data->status = SENSOR_STATUS_ERROR;
        return ret;
    }
    
    data->illuminance = illuminance;
    data->status = g_sensor_status[SENSOR_BH1750];
    
    SENSDOT_LOGD(TAG, "BH1750: %u lx", data->illuminance);
    
    return ESP_OK;
}

/**
 * @brief Read PIR sensor
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
 * @brief Read battery voltage
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
            data->voltage = (voltage_mv / 1000.0f) * BATT_DIVIDER_RATIO;
        } else {
            // Fallback to simple calculation for ESP32-C3
            data->voltage = ((float)raw_value / 4095.0f) * 3.3f * BATT_DIVIDER_RATIO;
        }
    } else {
        // Simple calculation without calibration for ESP32-C3
        data->voltage = ((float)raw_value / 4095.0f) * 3.3f * BATT_DIVIDER_RATIO;
    }
    
    // Apply calibration
    data->voltage += g_calibration.battery_calibration;
    
    // Determine low battery status
    data->low_battery = sensor_is_battery_low(data->voltage);
    data->status = g_sensor_status[SENSOR_BATTERY];
    
    SENSDOT_LOGD(TAG, "Battery: %.2fV (raw=%d, low=%s)", 
                data->voltage, raw_value, data->low_battery ? "yes" : "no");
    
    return ESP_OK;
}

/**
 * @brief Read all sensor data
 */
esp_err_t sensors_read_all(sensor_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    esp_err_t ret = ESP_OK;
    bmp280_data_t bmp_data;
    bh1750_data_t bh_data;
    pir_data_t pir_data;
    battery_data_t batt_data;
    
    SENSDOT_LOGD(TAG, "Reading all sensors");
    
    // Initialize data structure
    memset(data, 0, sizeof(sensor_data_t));
    data->timestamp = time(NULL);
    
    // Read BMP280/BME280
    if (sensor_read_bmp280(&bmp_data) == ESP_OK) {
        data->temperature = bmp_data.temperature;
        data->humidity = bmp_data.humidity;
        data->pressure = bmp_data.pressure;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read BMP280/BME280");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    // Read BH1750
    if (sensor_read_bh1750(&bh_data) == ESP_OK) {
        data->light = bh_data.illuminance;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read BH1750");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    // Read PIR
    if (sensor_read_pir(&pir_data) == ESP_OK) {
        data->motion = pir_data.motion_detected;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read PIR");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    // Read battery
    if (sensor_read_battery(&batt_data) == ESP_OK) {
        data->battery_voltage = batt_data.voltage;
        data->low_battery = batt_data.low_battery;
    } else {
        SENSDOT_LOGW(TAG, "Failed to read battery");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    }
    
    SENSDOT_LOGI(TAG, "Sensor data: T=%.2f°C, H=%.2f%%, P=%.2fhPa, L=%ulx, M=%s, B=%.2fV", 
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
 * @brief Check if BME280 (with humidity) is detected
 */
bool sensor_has_humidity_support(void)
{
    return g_has_humidity;
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
    SENSDOT_LOGI(TAG, "Performing sensor self-test");
    
    esp_err_t ret = ESP_OK;
    bool was_powered = g_sensors_powered;
    
    // Power on sensors if needed
    if (!g_sensors_powered) {
        sensors_power_on();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for stabilization
    }
    
    // Test each sensor
    sensor_data_t test_data;
    if (sensors_read_all(&test_data) != ESP_OK) {
        SENSDOT_LOGE(TAG, "Sensor self-test failed");
        ret = SENSDOT_ERR_SENSOR_FAIL;
    } else {
        SENSDOT_LOGI(TAG, "Sensor self-test passed");
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
        "BMP280/BME280 (Temperature, Pressure, Humidity)",
        "BH1750 (Light)",
        "PIR (Motion)",
        "ADC (Battery)"
    };
    
    if (sensor_type >= SENSOR_COUNT) {
        strncpy(info_str, "Unknown sensor", max_len - 1);
    } else {
        const char *status_str = (g_sensor_status[sensor_type] == SENSOR_STATUS_OK) ? "OK" : "Error";
        
        if (sensor_type == SENSOR_BMP280) {
            // Include sensor type information
            const char *sensor_model = g_has_humidity ? "BME280" : "BMP280";
            snprintf(info_str, max_len, "%s (%s detected) - Status: %s", 
                    sensor_names[sensor_type], sensor_model, status_str);
        } else {
            snprintf(info_str, max_len, "%s - Status: %s", sensor_names[sensor_type], status_str);
        }
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
 * @brief Check if battery is low
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
    // Simple linear mapping for Li-ion battery
    // 4.2V = 100%, 3.0V = 0%
    const float max_voltage = 4.2f;
    const float min_voltage = 3.0f;
    
    if (voltage >= max_voltage) return 100;
    if (voltage <= min_voltage) return 0;
    
    float percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;
    return (uint8_t)CLAMP(percentage, 0, 100);
}