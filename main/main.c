/**
 * @file main.c
 * @brief Main application logic for SensDot IoT sensor device (ESP32-C3 Super Mini)
 */

#include "sensdot_common.h"
#include "config.h"
#include "sensors.h"
#include "wifi_manager.h"
#include "mqtt_client.h"
#include "web_server.h"
#include "power_management.h"
#include "indicators.h"

// Application tag for logging
static const char *TAG = "main";

// Global state variables stored in RTC memory
RTC_DATA_ATTR static uint32_t boot_count = 0;
RTC_DATA_ATTR static time_t last_alarm_time = 0;
RTC_DATA_ATTR static device_state_t device_state = DEVICE_STATE_INIT;

// Current device configuration
static device_config_t current_config;

/**
 * @brief Check for factory reset request (using built-in BOOT button)
 * @return true if factory reset requested, false otherwise
 */
static bool check_factory_reset_request(void)
{
    // Use built-in BOOT button on GPIO9 for factory reset
    gpio_config_t button_config = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    esp_err_t ret = gpio_config(&button_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure boot button GPIO: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Check if boot button is held during startup (button is active LOW)
    if (gpio_get_level(BOOT_BUTTON_GPIO) == 0) {
        SENSDOT_LOGI(TAG, "Factory reset button detected");
        
        // Wait for button to be held for 5 seconds
        int hold_time = 0;
        while (gpio_get_level(BOOT_BUTTON_GPIO) == 0 && hold_time < 5000) {
            vTaskDelay(pdMS_TO_TICKS(100));
            hold_time += 100;
        }
        
        if (hold_time >= 5000) {
            SENSDOT_LOGI(TAG, "Factory reset requested");
            indicators_show_factory_reset();
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Initialize system components
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t system_init(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Initializing SensDot v%s for ESP32-C3", SENSDOT_VERSION_STRING);
    SENSDOT_LOGI(TAG, "Boot count: %lu", boot_count);
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize configuration management
    ret = config_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize configuration: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize indicators early for status feedback
    ret = indicators_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize indicators: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize power management
    ret = power_management_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize power management: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize sensors
    ret = sensors_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize WiFi manager
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize WiFi manager: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "System initialization completed");
    return ESP_OK;
}

/**
 * @brief Perform factory reset
 */
static void perform_factory_reset(void)
{
    SENSDOT_LOGI(TAG, "Performing factory reset");
    
    // Reset all configurations
    config_factory_reset();
    power_reset_config();
    indicators_reset_config();
    
    // Clear RTC memory
    boot_count = 0;
    last_alarm_time = 0;
    device_state = DEVICE_STATE_INIT;
    
    SENSDOT_LOGI(TAG, "Factory reset completed, restarting...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
}

/**
 * @brief Enter setup mode for device configuration
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t enter_setup_mode(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Entering setup mode");
    device_state = DEVICE_STATE_SETUP;
    
    // Show setup mode indication
    indicators_show_setup_mode();
    
    // Generate unique AP SSID
    char unique_ssid[MAX_SSID_LEN];
    ret = wifi_generate_unique_ssid(SETUP_AP_SSID_PREFIX, unique_ssid, sizeof(unique_ssid));
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to generate unique SSID");
        return ret;
    }
    
    // Start WiFi in AP mode
    ret = wifi_start_ap(unique_ssid, SETUP_AP_PASSWORD);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi AP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start web server
    ret = webserver_start();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start web server: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "Setup mode active - connect to %s", unique_ssid);
    SENSDOT_LOGI(TAG, "Open http://%s to configure device", SETUP_AP_IP);
    
    // Wait in setup mode with timeout
    uint32_t timeout_ms = SETUP_TIMEOUT_MS;
    while (device_state == DEVICE_STATE_SETUP && timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout_ms -= 1000;
        
        // Periodic status indication
        if (timeout_ms % 5000 == 0) {
            indicators_show_status(STATUS_READY, false);
        }
    }
    
    // Stop web server and WiFi
    webserver_stop();
    wifi_stop();
    
    if (timeout_ms <= 0) {
        SENSDOT_LOGW(TAG, "Setup timeout, entering deep sleep");
        power_enter_deep_sleep(current_config.retry_sleep_sec);
    }
    
    return ESP_OK;
}

/**
 * @brief Connect to WiFi and MQTT
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t connect_to_services(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Connecting to services");
    device_state = DEVICE_STATE_CONNECTING;
    
    indicators_show_wifi_connecting();
    
    // Start WiFi in station mode
    ret = wifi_start_sta(current_config.wifi_ssid, current_config.wifi_pass);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi station: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for WiFi connection
    ret = wifi_wait_connection(WIFI_CONNECT_TIMEOUT_MS);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "WiFi connection failed: %s", esp_err_to_name(ret));
        
        // Check if maximum retries reached
        if (wifi_max_retries_reached(MAX_WIFI_RETRIES)) {
            SENSDOT_LOGW(TAG, "Maximum WiFi retries reached, entering setup mode");
            wifi_reset_retry_count();
            return SENSDOT_ERR_WIFI_FAIL;
        }
        
        return ret;
    }
    
    indicators_show_wifi_connected();
    SENSDOT_LOGI(TAG, "WiFi connected");
    
    // Initialize and connect MQTT
    indicators_show_mqtt_connecting();
    
    ret = mqtt_client_init(&current_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = mqtt_connect(5000); // 5 second timeout
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "MQTT connection failed: %s", esp_err_to_name(ret));
        return SENSDOT_ERR_MQTT_FAIL;
    }
    
    indicators_show_mqtt_connected();
    SENSDOT_LOGI(TAG, "MQTT connected");
    
    device_state = DEVICE_STATE_CONNECTED;
    return ESP_OK;
}

/**
 * @brief Read sensors and publish data
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t read_and_publish_data(void)
{
    esp_err_t ret;
    sensor_data_t sensor_data;
    
    SENSDOT_LOGI(TAG, "Reading sensors and publishing data");
    
    // Power on sensors
    ret = sensors_power_on();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to power on sensors: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for sensors to stabilize (longer for BMP280)
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Read all sensor data
    ret = sensors_read_all(&sensor_data);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to read sensors: %s", esp_err_to_name(ret));
        sensors_power_off();
        return ret;
    }
    
    // Power off sensors to save energy
    sensors_power_off();
    
    // Check for motion alarm
    time_t current_time = time(NULL);
    if (sensor_data.motion && sensor_should_trigger_alarm(current_time, current_config.alarm_hold_sec)) {
        sensor_update_alarm_time(current_time);
        SENSDOT_LOGI(TAG, "Motion detected!");
    }
    
    // Check battery status
    if (sensor_data.low_battery) {
        indicators_show_low_battery();
        SENSDOT_LOGW(TAG, "Low battery: %.2fV", sensor_data.battery_voltage);
    }
    
    // Publish sensor data via MQTT
    indicators_show_data_transmission();
    
    ret = mqtt_publish_sensor_data(&sensor_data);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to publish sensor data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register with Home Assistant periodically
    if (boot_count % MQTT_DISCOVERY_INTERVAL == 0) {
        char device_id[MAX_DEVICE_ID_LEN];
        mqtt_generate_device_id(device_id, sizeof(device_id));
        
        ret = mqtt_ha_discovery_register(device_id);
        if (ret != ESP_OK) {
            SENSDOT_LOGW(TAG, "Failed to register HA discovery: %s", esp_err_to_name(ret));
        }
    }
    
    // Show successful data transmission
    indicators_show_status(STATUS_SUCCESS, false);
    
    SENSDOT_LOGI(TAG, "Data published successfully");
    return ESP_OK;
}

/**
 * @brief Normal operation mode
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t normal_operation(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Starting normal operation");
    device_state = DEVICE_STATE_NORMAL;
    
    // Connect to WiFi and MQTT
    ret = connect_to_services();
    if (ret == SENSDOT_ERR_WIFI_FAIL) {
        // Enter setup mode if WiFi connection repeatedly fails
        return enter_setup_mode();
    } else if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to connect to services: %s", esp_err_to_name(ret));
        // Continue with offline operation or retry after configurable interval
        power_enter_deep_sleep(current_config.retry_sleep_sec);
        return ret;
    }
    
    // Read sensors and publish data
    ret = read_and_publish_data();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to read and publish data: %s", esp_err_to_name(ret));
        // Continue operation despite sensor errors
    }
    
    // Cleanup connections
    mqtt_disconnect();
    mqtt_client_deinit();
    wifi_stop();
    
    // Prepare for deep sleep
    SENSDOT_LOGI(TAG, "Entering deep sleep for %d seconds", current_config.wake_interval_sec);
    device_state = DEVICE_STATE_SLEEP;
    
    // Configure wake up sources (ESP32-C3 specific)
    uint32_t wakeup_sources = WAKEUP_SOURCE_TIMER | WAKEUP_SOURCE_PIR;
    power_prepare_for_sleep(current_config.wake_interval_sec, wakeup_sources);
    
    boot_count++;
    power_enter_deep_sleep(current_config.wake_interval_sec);
    
    // This function should not return
    return ESP_OK;
}

/**
 * @brief Determine operating mode based on configuration and state
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t determine_operating_mode(void)
{
    esp_err_t ret;
    
    // Load current configuration
    ret = config_load(&current_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to load configuration: %s", esp_err_to_name(ret));
        config_init_defaults(&current_config);
    }
    
    // Check if device is configured
    if (!current_config.configured || strlen(current_config.wifi_ssid) == 0) {
        SENSDOT_LOGI(TAG, "Device not configured, entering setup mode");
        return enter_setup_mode();
    }
    
    // Check wake up reason
    wakeup_reason_t wakeup_reason = power_get_wakeup_reason();
    
    switch (wakeup_reason) {
        case WAKEUP_REASON_TIMER:
            SENSDOT_LOGI(TAG, "Woke up from timer");
            break;
            
        case WAKEUP_REASON_PIR:
            SENSDOT_LOGI(TAG, "Woke up from PIR sensor");
            break;
            
        case WAKEUP_REASON_RESET:
            SENSDOT_LOGI(TAG, "Woke up from reset");
            break;
            
        default:
            SENSDOT_LOGI(TAG, "Unknown wake up reason");
            break;
    }
    
    // Handle wake up and proceed with normal operation
    power_handle_wakeup();
    return normal_operation();
}

/**
 * @brief Main application entry point
 */
void app_main(void)
{
    esp_err_t ret;
    
    // Increment boot counter
    boot_count++;
    
    // Check for factory reset request (using BOOT button)
    if (check_factory_reset_request()) {
        perform_factory_reset();
        return; // Should not reach here
    }
    
    // Initialize system components
    ret = system_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "System initialization failed: %s", esp_err_to_name(ret));
        indicators_show_error(ret, true);
        
        // Try to continue with limited functionality
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
    }
    
    // Show ready status
    indicators_show_status(STATUS_READY, false);
    
    // Determine and execute operating mode
    ret = determine_operating_mode();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Operating mode failed: %s", esp_err_to_name(ret));
        indicators_show_error(ret, false);
        
        // Sleep and retry
        config_load(&current_config); // Reload config in case it wasn't loaded
        power_enter_deep_sleep(current_config.retry_sleep_sec > 0 ? 
                              current_config.retry_sleep_sec : DEFAULT_RETRY_SLEEP_SEC);
    }
    
    // Should not reach here in normal operation
    SENSDOT_LOGW(TAG, "Unexpected end of main function");
    esp_restart();
}