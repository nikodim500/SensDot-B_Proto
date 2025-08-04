/**
 * @file config.c
 * @brief Configuration management implementation for SensDot device (ESP32-C3)
 */

#include "config.h"
#include "sensdot_common.h"

static const char *TAG = "config";

// Current configuration instance
static device_config_t g_config;
static bool g_config_loaded = false;

/**
 * @brief Initialize configuration system
 */
esp_err_t config_init(void)
{
    SENSDOT_LOGI(TAG, "Initializing configuration system");
    
    // Initialize with defaults
    config_init_defaults(&g_config);
    g_config_loaded = false;
    
    return ESP_OK;
}

/**
 * @brief Initialize configuration with default values
 */
void config_init_defaults(device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    SENSDOT_LOGD(TAG, "Initializing configuration with defaults");
    
    memset(config, 0, sizeof(device_config_t));
    
    // Set default values
    strncpy(config->wifi_ssid, "", sizeof(config->wifi_ssid) - 1);
    strncpy(config->wifi_pass, "", sizeof(config->wifi_pass) - 1);
    strncpy(config->mqtt_uri, DEFAULT_MQTT_URI, sizeof(config->mqtt_uri) - 1);
    strncpy(config->mqtt_user, "", sizeof(config->mqtt_user) - 1);
    strncpy(config->mqtt_pass, "", sizeof(config->mqtt_pass) - 1);
    strncpy(config->mqtt_prefix, DEFAULT_MQTT_PREFIX, sizeof(config->mqtt_prefix) - 1);
    
    config->wake_interval_sec = DEFAULT_WAKE_INTERVAL_SEC;
    config->alarm_hold_sec = DEFAULT_ALARM_HOLD_SEC;
    config->low_batt_threshold = DEFAULT_LOW_BATT_THRESHOLD;
    config->retry_sleep_sec = DEFAULT_RETRY_SLEEP_SEC;
    config->battery_divider_enabled = DEFAULT_BATTERY_DIVIDER_ENABLED;  // From Kconfig
    config->configured = false;
    
    SENSDOT_LOGD(TAG, "Default configuration initialized");
}

/**
 * @brief Load configuration from NVS storage
 */
esp_err_t config_load(device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    SENSDOT_LOGD(TAG, "Loading configuration from NVS");
    
    // Initialize with defaults first
    config_init_defaults(config);
    
    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        SENSDOT_LOGW(TAG, "NVS not found, using defaults: %s", esp_err_to_name(err));
        return ESP_OK; // Not an error, just use defaults
    }
    
    size_t required_size;
    
    // Load string values
    required_size = sizeof(config->wifi_ssid);
    nvs_get_str(nvs_handle, NVS_WIFI_SSID, config->wifi_ssid, &required_size);
    
    required_size = sizeof(config->wifi_pass);
    nvs_get_str(nvs_handle, NVS_WIFI_PASS, config->wifi_pass, &required_size);
    
    required_size = sizeof(config->mqtt_uri);
    nvs_get_str(nvs_handle, NVS_MQTT_URI, config->mqtt_uri, &required_size);
    
    required_size = sizeof(config->mqtt_user);
    nvs_get_str(nvs_handle, NVS_MQTT_USER, config->mqtt_user, &required_size);
    
    required_size = sizeof(config->mqtt_pass);
    nvs_get_str(nvs_handle, NVS_MQTT_PASS, config->mqtt_pass, &required_size);
    
    required_size = sizeof(config->mqtt_prefix);
    nvs_get_str(nvs_handle, NVS_MQTT_PREFIX, config->mqtt_prefix, &required_size);
    
    // Load integer values
    nvs_get_i32(nvs_handle, NVS_WAKE_INTERVAL, &config->wake_interval_sec);
    nvs_get_i32(nvs_handle, NVS_ALARM_HOLD, &config->alarm_hold_sec);
    nvs_get_i32(nvs_handle, NVS_RETRY_SLEEP_SEC, &config->retry_sleep_sec);
    
    // Load float value (stored as blob)
    required_size = sizeof(config->low_batt_threshold);
    nvs_get_blob(nvs_handle, NVS_LOW_BATT_THRESH, &config->low_batt_threshold, &required_size);
    
    // Load boolean values
    uint8_t configured = 0;
    nvs_get_u8(nvs_handle, NVS_CONFIGURED, &configured);
    config->configured = (configured == 1);
    
    uint8_t battery_divider = 0;
    nvs_get_u8(nvs_handle, NVS_BATTERY_DIVIDER, &battery_divider);
    config->battery_divider_enabled = (battery_divider == 1);
    
    nvs_close(nvs_handle);
    
    // Copy to global config
    memcpy(&g_config, config, sizeof(device_config_t));
    g_config_loaded = true;
    
    SENSDOT_LOGI(TAG, "Configuration loaded: SSID=%s, Configured=%d, Battery Divider=%d", 
                config->wifi_ssid, config->configured, config->battery_divider_enabled);
    
    return ESP_OK;
}

/**
 * @brief Save configuration to NVS storage
 */
esp_err_t config_save(const device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    nvs_handle_t nvs_handle;
    esp_err_t err;
    
    SENSDOT_LOGI(TAG, "Saving configuration to NVS");
    
    // Validate configuration before saving
    err = config_validate(config);
    if (err != ESP_OK) {
        SENSDOT_LOGE(TAG, "Configuration validation failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Open NVS
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        SENSDOT_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return err;
    }
    
    // Save string values
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_WIFI_SSID, config->wifi_ssid));
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_WIFI_PASS, config->wifi_pass));
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_MQTT_URI, config->mqtt_uri));
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_MQTT_USER, config->mqtt_user));
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_MQTT_PASS, config->mqtt_pass));
    CHECK_ERROR_RETURN(nvs_set_str(nvs_handle, NVS_MQTT_PREFIX, config->mqtt_prefix));
    
    // Save integer values
    CHECK_ERROR_RETURN(nvs_set_i32(nvs_handle, NVS_WAKE_INTERVAL, config->wake_interval_sec));
    CHECK_ERROR_RETURN(nvs_set_i32(nvs_handle, NVS_ALARM_HOLD, config->alarm_hold_sec));
    CHECK_ERROR_RETURN(nvs_set_i32(nvs_handle, NVS_RETRY_SLEEP_SEC, config->retry_sleep_sec));
    
    // Save float value as blob
    CHECK_ERROR_RETURN(nvs_set_blob(nvs_handle, NVS_LOW_BATT_THRESH, 
                                   &config->low_batt_threshold, sizeof(config->low_batt_threshold)));
    
    // Save boolean flags
    CHECK_ERROR_RETURN(nvs_set_u8(nvs_handle, NVS_CONFIGURED, config->configured ? 1 : 0));
    CHECK_ERROR_RETURN(nvs_set_u8(nvs_handle, NVS_BATTERY_DIVIDER, config->battery_divider_enabled ? 1 : 0));
    
    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        SENSDOT_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }
    
    nvs_close(nvs_handle);
    
    // Update global config
    memcpy(&g_config, config, sizeof(device_config_t));
    g_config_loaded = true;
    
    SENSDOT_LOGI(TAG, "Configuration saved successfully");
    return ESP_OK;
}

/**
 * @brief Validate configuration parameters
 */
esp_err_t config_validate(const device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    SENSDOT_LOGD(TAG, "Validating configuration");
    
    // Check WiFi SSID
    if (config->configured && strlen(config->wifi_ssid) == 0) {
        SENSDOT_LOGE(TAG, "WiFi SSID is required when configured");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->wifi_ssid) >= MAX_SSID_LEN) {
        SENSDOT_LOGE(TAG, "WiFi SSID too long: %zu", strlen(config->wifi_ssid));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check WiFi password length
    if (strlen(config->wifi_pass) >= MAX_PASSWORD_LEN) {
        SENSDOT_LOGE(TAG, "WiFi password too long: %zu", strlen(config->wifi_pass));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check MQTT URI
    if (strlen(config->mqtt_uri) == 0) {
        SENSDOT_LOGE(TAG, "MQTT URI is required");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_uri) >= MAX_URI_LEN) {
        SENSDOT_LOGE(TAG, "MQTT URI too long: %zu", strlen(config->mqtt_uri));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check MQTT credentials length
    if (strlen(config->mqtt_user) >= MAX_USERNAME_LEN) {
        SENSDOT_LOGE(TAG, "MQTT username too long: %zu", strlen(config->mqtt_user));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_pass) >= MAX_PASSWORD_LEN) {
        SENSDOT_LOGE(TAG, "MQTT password too long: %zu", strlen(config->mqtt_pass));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check MQTT prefix
    if (strlen(config->mqtt_prefix) == 0) {
        SENSDOT_LOGE(TAG, "MQTT prefix is required");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_prefix) >= MAX_TOPIC_LEN) {
        SENSDOT_LOGE(TAG, "MQTT prefix too long: %zu", strlen(config->mqtt_prefix));
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check timing values
    if (config->wake_interval_sec < 60 || config->wake_interval_sec > 3600) {
        SENSDOT_LOGE(TAG, "Wake interval out of range: %d", config->wake_interval_sec);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (config->alarm_hold_sec < 10 || config->alarm_hold_sec > 300) {
        SENSDOT_LOGE(TAG, "Alarm hold time out of range: %d", config->alarm_hold_sec);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (config->retry_sleep_sec < 30 || config->retry_sleep_sec > 600) {
        SENSDOT_LOGE(TAG, "Retry sleep interval out of range: %d", config->retry_sleep_sec);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check battery threshold
    if (config->low_batt_threshold < 3.0f || config->low_batt_threshold > 4.2f) {
        SENSDOT_LOGE(TAG, "Battery threshold out of range: %.2f", config->low_batt_threshold);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    SENSDOT_LOGD(TAG, "Configuration validation passed");
    return ESP_OK;
}

/**
 * @brief Reset configuration to factory defaults
 */
esp_err_t config_factory_reset(void)
{
    SENSDOT_LOGI(TAG, "Performing factory reset");
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    
    // Reset global config
    config_init_defaults(&g_config);
    g_config_loaded = true;
    
    SENSDOT_LOGI(TAG, "Factory reset completed");
    return ESP_OK;
}

/**
 * @brief Check if device is configured
 */
bool config_is_configured(void)
{
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    return g_config.configured && strlen(g_config.wifi_ssid) > 0;
}

/**
 * @brief Get current configuration
 */
const device_config_t* config_get_current(void)
{
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    return &g_config;
}

/**
 * @brief Update string configuration parameter
 */
esp_err_t config_set_string(const char *key, const char *value)
{
    CHECK_NULL_PTR(key);
    CHECK_NULL_PTR(value);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Update the appropriate field
    if (strcmp(key, NVS_WIFI_SSID) == 0) {
        strncpy(g_config.wifi_ssid, value, sizeof(g_config.wifi_ssid) - 1);
    } else if (strcmp(key, NVS_WIFI_PASS) == 0) {
        strncpy(g_config.wifi_pass, value, sizeof(g_config.wifi_pass) - 1);
    } else if (strcmp(key, NVS_MQTT_URI) == 0) {
        strncpy(g_config.mqtt_uri, value, sizeof(g_config.mqtt_uri) - 1);
    } else if (strcmp(key, NVS_MQTT_USER) == 0) {
        strncpy(g_config.mqtt_user, value, sizeof(g_config.mqtt_user) - 1);
    } else if (strcmp(key, NVS_MQTT_PASS) == 0) {
        strncpy(g_config.mqtt_pass, value, sizeof(g_config.mqtt_pass) - 1);
    } else if (strcmp(key, NVS_MQTT_PREFIX) == 0) {
        strncpy(g_config.mqtt_prefix, value, sizeof(g_config.mqtt_prefix) - 1);
    } else {
        SENSDOT_LOGE(TAG, "Unknown string key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return config_save(&g_config);
}

/**
 * @brief Update integer configuration parameter
 */
esp_err_t config_set_int(const char *key, int value)
{
    CHECK_NULL_PTR(key);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Update the appropriate field
    if (strcmp(key, NVS_WAKE_INTERVAL) == 0) {
        g_config.wake_interval_sec = value;
    } else if (strcmp(key, NVS_ALARM_HOLD) == 0) {
        g_config.alarm_hold_sec = value;
    } else if (strcmp(key, NVS_RETRY_SLEEP_SEC) == 0) {
        g_config.retry_sleep_sec = value;
    } else {
        SENSDOT_LOGE(TAG, "Unknown int key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return config_save(&g_config);
}

/**
 * @brief Update float configuration parameter
 */
esp_err_t config_set_float(const char *key, float value)
{
    CHECK_NULL_PTR(key);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Update the appropriate field
    if (strcmp(key, NVS_LOW_BATT_THRESH) == 0) {
        g_config.low_batt_threshold = value;
    } else {
        SENSDOT_LOGE(TAG, "Unknown float key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return config_save(&g_config);
}

/**
 * @brief Update boolean configuration parameter
 */
esp_err_t config_set_bool(const char *key, bool value)
{
    CHECK_NULL_PTR(key);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Update the appropriate field
    if (strcmp(key, NVS_CONFIGURED) == 0) {
        g_config.configured = value;
    } else if (strcmp(key, NVS_BATTERY_DIVIDER) == 0) {
        g_config.battery_divider_enabled = value;
    } else {
        SENSDOT_LOGE(TAG, "Unknown bool key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return config_save(&g_config);
}

/**
 * @brief Get string configuration parameter
 */
esp_err_t config_get_string(const char *key, char *value, size_t max_len)
{
    CHECK_NULL_PTR(key);
    CHECK_NULL_PTR(value);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    const char *source = NULL;
    
    // Get the appropriate field
    if (strcmp(key, NVS_WIFI_SSID) == 0) {
        source = g_config.wifi_ssid;
    } else if (strcmp(key, NVS_WIFI_PASS) == 0) {
        source = g_config.wifi_pass;
    } else if (strcmp(key, NVS_MQTT_URI) == 0) {
        source = g_config.mqtt_uri;
    } else if (strcmp(key, NVS_MQTT_USER) == 0) {
        source = g_config.mqtt_user;
    } else if (strcmp(key, NVS_MQTT_PASS) == 0) {
        source = g_config.mqtt_pass;
    } else if (strcmp(key, NVS_MQTT_PREFIX) == 0) {
        source = g_config.mqtt_prefix;
    } else {
        SENSDOT_LOGE(TAG, "Unknown string key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (source) {
        strncpy(value, source, max_len - 1);
        value[max_len - 1] = '\0';
    }
    
    return ESP_OK;
}

/**
 * @brief Get integer configuration parameter
 */
esp_err_t config_get_int(const char *key, int *value)
{
    CHECK_NULL_PTR(key);
    CHECK_NULL_PTR(value);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Get the appropriate field
    if (strcmp(key, NVS_WAKE_INTERVAL) == 0) {
        *value = g_config.wake_interval_sec;
    } else if (strcmp(key, NVS_ALARM_HOLD) == 0) {
        *value = g_config.alarm_hold_sec;
    } else if (strcmp(key, NVS_RETRY_SLEEP_SEC) == 0) {
        *value = g_config.retry_sleep_sec;
    } else {
        SENSDOT_LOGE(TAG, "Unknown int key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Get float configuration parameter
 */
esp_err_t config_get_float(const char *key, float *value)
{
    CHECK_NULL_PTR(key);
    CHECK_NULL_PTR(value);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Get the appropriate field
    if (strcmp(key, NVS_LOW_BATT_THRESH) == 0) {
        *value = g_config.low_batt_threshold;
    } else {
        SENSDOT_LOGE(TAG, "Unknown float key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Get boolean configuration parameter
 */
esp_err_t config_get_bool(const char *key, bool *value)
{
    CHECK_NULL_PTR(key);
    CHECK_NULL_PTR(value);
    
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    // Get the appropriate field
    if (strcmp(key, NVS_CONFIGURED) == 0) {
        *value = g_config.configured;
    } else if (strcmp(key, NVS_BATTERY_DIVIDER) == 0) {
        *value = g_config.battery_divider_enabled;
    } else {
        SENSDOT_LOGE(TAG, "Unknown bool key: %s", key);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Print current configuration (for debugging)
 */
void config_print(void)
{
    if (!g_config_loaded) {
        config_load(&g_config);
    }
    
    SENSDOT_LOGI(TAG, "=== Current Configuration ===");
    SENSDOT_LOGI(TAG, "WiFi SSID: %s", g_config.wifi_ssid);
    SENSDOT_LOGI(TAG, "WiFi Pass: %s", strlen(g_config.wifi_pass) > 0 ? "***" : "(empty)");
    SENSDOT_LOGI(TAG, "MQTT URI: %s", g_config.mqtt_uri);
    SENSDOT_LOGI(TAG, "MQTT User: %s", g_config.mqtt_user);
    SENSDOT_LOGI(TAG, "MQTT Pass: %s", strlen(g_config.mqtt_pass) > 0 ? "***" : "(empty)");
    SENSDOT_LOGI(TAG, "MQTT Prefix: %s", g_config.mqtt_prefix);
    SENSDOT_LOGI(TAG, "Wake Interval: %d sec", g_config.wake_interval_sec);
    SENSDOT_LOGI(TAG, "Alarm Hold: %d sec", g_config.alarm_hold_sec);
    SENSDOT_LOGI(TAG, "Retry Sleep: %d sec", g_config.retry_sleep_sec);
    SENSDOT_LOGI(TAG, "Low Batt Threshold: %.2f V", g_config.low_batt_threshold);
    SENSDOT_LOGI(TAG, "Battery Divider: %s", g_config.battery_divider_enabled ? "Enabled" : "Disabled");
    SENSDOT_LOGI(TAG, "Configured: %s", g_config.configured ? "Yes" : "No");
    SENSDOT_LOGI(TAG, "=============================");
}