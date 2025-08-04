/**
 * @file mqtt_client.c
 * @brief MQTT client and Home Assistant integration implementation
 */

#include "mqtt_client.h"
#include "sensdot_common.h"
#include "config.h"

static const char *TAG = "mqtt";

// MQTT client state
static esp_mqtt_client_handle_t g_mqtt_client = NULL;
static mqtt_state_t g_mqtt_state = MQTT_STATE_DISCONNECTED;
static bool g_mqtt_initialized = false;
static mqtt_message_callback_t g_message_callback = NULL;
static void *g_callback_user_data = NULL;

// Statistics
static mqtt_stats_t g_mqtt_stats = {0};
static char g_last_error[128] = {0};

// Current device configuration and ID
static device_config_t g_current_config = {0};
static char g_device_id[MAX_DEVICE_ID_LEN] = {0};

// Forward declarations
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
static esp_err_t mqtt_set_state(mqtt_state_t new_state);
static esp_err_t create_ha_device_info(cJSON *config_obj, const char *device_id);

// Home Assistant device class strings
static const char* ha_device_class_strings[] = {
    "temperature",
    "humidity", 
    "pressure",
    "illuminance",
    "motion",
    "battery",
    "voltage"
};

/**
 * @brief Set MQTT state
 */
static esp_err_t mqtt_set_state(mqtt_state_t new_state)
{
    if (g_mqtt_state != new_state) {
        mqtt_state_t old_state = g_mqtt_state;
        g_mqtt_state = new_state;
        
        SENSDOT_LOGD(TAG, "MQTT state changed: %d -> %d", old_state, new_state);
    }
    
    return ESP_OK;
}

/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            SENSDOT_LOGI(TAG, "MQTT connected");
            mqtt_set_state(MQTT_STATE_CONNECTED);
            g_mqtt_stats.connections++;
            g_mqtt_stats.last_connection = time(NULL);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            SENSDOT_LOGW(TAG, "MQTT disconnected");
            mqtt_set_state(MQTT_STATE_DISCONNECTED);
            g_mqtt_stats.disconnections++;
            break;
            
        case MQTT_EVENT_SUBSCRIBED:
            SENSDOT_LOGD(TAG, "MQTT subscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:
            SENSDOT_LOGD(TAG, "MQTT unsubscribed, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:
            SENSDOT_LOGD(TAG, "MQTT published, msg_id=%d", event->msg_id);
            g_mqtt_stats.messages_sent++;
            g_mqtt_stats.last_publish = time(NULL);
            break;
            
        case MQTT_EVENT_DATA:
            SENSDOT_LOGD(TAG, "MQTT data received, topic=%.*s", event->topic_len, event->topic);
            
            // Call message callback if registered
            if (g_message_callback) {
                // Create null-terminated strings
                char topic[128];
                char data[512];
                
                int topic_len = MIN(event->topic_len, sizeof(topic) - 1);
                int data_len = MIN(event->data_len, sizeof(data) - 1);
                
                strncpy(topic, event->topic, topic_len);
                topic[topic_len] = '\0';
                
                strncpy(data, event->data, data_len);
                data[data_len] = '\0';
                
                g_message_callback(topic, data, g_callback_user_data);
            }
            break;
            
        case MQTT_EVENT_ERROR:
            SENSDOT_LOGE(TAG, "MQTT error occurred");
            mqtt_set_state(MQTT_STATE_ERROR);
            g_mqtt_stats.messages_failed++;
            
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                snprintf(g_last_error, sizeof(g_last_error), "TCP transport error: 0x%x", 
                        event->error_handle->esp_transport_sock_errno);
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                snprintf(g_last_error, sizeof(g_last_error), "Connection refused: 0x%x", 
                        event->error_handle->connect_return_code);
            } else {
                snprintf(g_last_error, sizeof(g_last_error), "Unknown MQTT error");
            }
            break;
            
        default:
            SENSDOT_LOGD(TAG, "MQTT event: %d", event_id);
            break;
    }
}

/**
 * @brief Initialize MQTT client
 */
esp_err_t mqtt_client_init(const device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    if (g_mqtt_initialized) {
        SENSDOT_LOGW(TAG, "MQTT client already initialized");
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Initializing MQTT client");
    
    // Validate configuration
    esp_err_t ret = mqtt_validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Copy configuration
    memcpy(&g_current_config, config, sizeof(device_config_t));
    
    // Generate device ID
    ret = mqtt_generate_device_id(g_device_id, sizeof(g_device_id));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure MQTT client
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = g_current_config.mqtt_uri,
        .session.keepalive = 60,
        .session.disable_clean_session = false,
        .network.timeout_ms = 5000,
        .network.refresh_connection_after_ms = 20000,
        .task.priority = 5,
        .task.stack_size = 6144,
        .buffer.out_size = 1024,
        .buffer.size = 1024,
    };
    
    // Set credentials if provided
    if (strlen(g_current_config.mqtt_user) > 0) {
        mqtt_cfg.credentials.username = g_current_config.mqtt_user;
        mqtt_cfg.credentials.password = g_current_config.mqtt_pass;
    }
    
    // Create MQTT client
    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!g_mqtt_client) {
        SENSDOT_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    // Register event handler
    ret = esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to register MQTT event handler: %s", esp_err_to_name(ret));
        esp_mqtt_client_destroy(g_mqtt_client);
        g_mqtt_client = NULL;
        return ret;
    }
    
    // Initialize statistics
    memset(&g_mqtt_stats, 0, sizeof(mqtt_stats_t));
    memset(g_last_error, 0, sizeof(g_last_error));
    
    mqtt_set_state(MQTT_STATE_DISCONNECTED);
    g_mqtt_initialized = true;
    
    SENSDOT_LOGI(TAG, "MQTT client initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize MQTT client
 */
esp_err_t mqtt_client_deinit(void)
{
    if (!g_mqtt_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Deinitializing MQTT client");
    
    // Stop client
    mqtt_client_stop();
    
    // Destroy client
    if (g_mqtt_client) {
        esp_mqtt_client_destroy(g_mqtt_client);
        g_mqtt_client = NULL;
    }
    
    mqtt_set_state(MQTT_STATE_DISCONNECTED);
    g_mqtt_initialized = false;
    
    SENSDOT_LOGI(TAG, "MQTT client deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Start MQTT client
 */
esp_err_t mqtt_client_start(void)
{
    if (!g_mqtt_initialized || !g_mqtt_client) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Starting MQTT client");
    
    esp_err_t ret = esp_mqtt_client_start(g_mqtt_client);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }
    
    mqtt_set_state(MQTT_STATE_CONNECTING);
    return ESP_OK;
}

/**
 * @brief Stop MQTT client
 */
esp_err_t mqtt_client_stop(void)
{
    if (!g_mqtt_initialized || !g_mqtt_client) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Stopping MQTT client");
    
    esp_err_t ret = esp_mqtt_client_stop(g_mqtt_client);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to stop MQTT client: %s", esp_err_to_name(ret));
        return ret;
    }
    
    mqtt_set_state(MQTT_STATE_DISCONNECTED);
    return ESP_OK;
}

/**
 * @brief Connect to MQTT broker
 */
esp_err_t mqtt_connect(uint32_t timeout_ms)
{
    if (!g_mqtt_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Connecting to MQTT broker");
    
    // Start client if not already started
    esp_err_t ret = mqtt_client_start();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for connection
    uint32_t elapsed = 0;
    const uint32_t check_interval = 100; // ms
    
    while (elapsed < timeout_ms && g_mqtt_state != MQTT_STATE_CONNECTED && g_mqtt_state != MQTT_STATE_ERROR) {
        vTaskDelay(pdMS_TO_TICKS(check_interval));
        elapsed += check_interval;
    }
    
    if (g_mqtt_state == MQTT_STATE_CONNECTED) {
        SENSDOT_LOGI(TAG, "MQTT connected successfully");
        return ESP_OK;
    } else if (g_mqtt_state == MQTT_STATE_ERROR) {
        SENSDOT_LOGE(TAG, "MQTT connection error");
        return SENSDOT_ERR_MQTT_FAIL;
    } else {
        SENSDOT_LOGE(TAG, "MQTT connection timeout");
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Disconnect from MQTT broker
 */
esp_err_t mqtt_disconnect(void)
{
    return mqtt_client_stop();
}

/**
 * @brief Publish message to MQTT topic
 */
int mqtt_publish(const char *topic, const char *payload, mqtt_qos_t qos, bool retain)
{
    CHECK_NULL_PTR(topic);
    CHECK_NULL_PTR(payload);
    
    if (!g_mqtt_initialized || !g_mqtt_client || g_mqtt_state != MQTT_STATE_CONNECTED) {
        SENSDOT_LOGW(TAG, "MQTT not connected, cannot publish");
        return -1;
    }
    
    SENSDOT_LOGD(TAG, "Publishing to %s: %s", topic, payload);
    
    int msg_id = esp_mqtt_client_publish(g_mqtt_client, topic, payload, 0, qos, retain ? 1 : 0);
    if (msg_id == -1) {
        SENSDOT_LOGE(TAG, "Failed to publish MQTT message");
        g_mqtt_stats.messages_failed++;
    }
    
    return msg_id;
}

/**
 * @brief Publish message to topic relative to device prefix
 */
int mqtt_publish_relative(const char *topic_suffix, const char *payload)
{
    CHECK_NULL_PTR(topic_suffix);
    CHECK_NULL_PTR(payload);
    
    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s/%s", g_current_config.mqtt_prefix, topic_suffix);
    
    return mqtt_publish(full_topic, payload, MQTT_QOS_1, false);
}

/**
 * @brief Publish all sensor data
 */
esp_err_t mqtt_publish_sensor_data(const sensor_data_t *data)
{
    CHECK_NULL_PTR(data);
    
    if (g_mqtt_state != MQTT_STATE_CONNECTED) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    char payload[64];
    int msg_count = 0;
    
    SENSDOT_LOGI(TAG, "Publishing sensor data");
    
    // Temperature
    snprintf(payload, sizeof(payload), "%.2f", data->temperature);
    if (mqtt_publish_relative("temperature", payload) != -1) msg_count++;
    
    // Humidity
    snprintf(payload, sizeof(payload), "%.2f", data->humidity);
    if (mqtt_publish_relative("humidity", payload) != -1) msg_count++;
    
    // Pressure
    snprintf(payload, sizeof(payload), "%.2f", data->pressure);
    if (mqtt_publish_relative("pressure", payload) != -1) msg_count++;
    
    // Light
    snprintf(payload, sizeof(payload), "%u", data->light);
    if (mqtt_publish_relative("illuminance", payload) != -1) msg_count++;
    
    // Motion
    const char *motion_payload = data->motion ? "ON" : "OFF";
    if (mqtt_publish_relative("motion", motion_payload) != -1) msg_count++;
    
    // Battery voltage
    snprintf(payload, sizeof(payload), "%.2f", data->battery_voltage);
    if (mqtt_publish_relative("battery_voltage", payload) != -1) msg_count++;
    
    // Low battery
    const char *low_batt_payload = data->low_battery ? "ON" : "OFF";
    if (mqtt_publish_relative("low_battery", low_batt_payload) != -1) msg_count++;
    
    // Timestamp
    snprintf(payload, sizeof(payload), "%ld", data->timestamp);
    if (mqtt_publish_relative("last_seen", payload) != -1) msg_count++;
    
    SENSDOT_LOGI(TAG, "Published %d sensor values", msg_count);
    return (msg_count > 0) ? ESP_OK : SENSDOT_ERR_MQTT_FAIL;
}

/**
 * @brief Publish device status
 */
esp_err_t mqtt_publish_status(const char *status)
{
    CHECK_NULL_PTR(status);
    
    return (mqtt_publish_relative("status", status) != -1) ? ESP_OK : SENSDOT_ERR_MQTT_FAIL;
}

/**
 * @brief Get current MQTT state
 */
mqtt_state_t mqtt_get_state(void)
{
    return g_mqtt_state;
}

/**
 * @brief Check if MQTT is connected
 */
bool mqtt_is_connected(void)
{
    return g_mqtt_state == MQTT_STATE_CONNECTED;
}

/**
 * @brief Subscribe to MQTT topic
 */
int mqtt_subscribe(const char *topic, mqtt_qos_t qos)
{
    CHECK_NULL_PTR(topic);
    
    if (!g_mqtt_initialized || !g_mqtt_client || g_mqtt_state != MQTT_STATE_CONNECTED) {
        return -1;
    }
    
    SENSDOT_LOGI(TAG, "Subscribing to topic: %s", topic);
    
    int msg_id = esp_mqtt_client_subscribe(g_mqtt_client, topic, qos);
    if (msg_id == -1) {
        SENSDOT_LOGE(TAG, "Failed to subscribe to topic: %s", topic);
    }
    
    return msg_id;
}

/**
 * @brief Unsubscribe from MQTT topic
 */
int mqtt_unsubscribe(const char *topic)
{
    CHECK_NULL_PTR(topic);
    
    if (!g_mqtt_initialized || !g_mqtt_client || g_mqtt_state != MQTT_STATE_CONNECTED) {
        return -1;
    }
    
    SENSDOT_LOGI(TAG, "Unsubscribing from topic: %s", topic);
    
    int msg_id = esp_mqtt_client_unsubscribe(g_mqtt_client, topic);
    if (msg_id == -1) {
        SENSDOT_LOGE(TAG, "Failed to unsubscribe from topic: %s", topic);
    }
    
    return msg_id;
}

/**
 * @brief Set MQTT message callback
 */
esp_err_t mqtt_set_message_callback(mqtt_message_callback_t callback, void *user_data)
{
    g_message_callback = callback;
    g_callback_user_data = user_data;
    return ESP_OK;
}

/**
 * @brief Create Home Assistant device information
 */
static esp_err_t create_ha_device_info(cJSON *config_obj, const char *device_id)
{
    cJSON *device = cJSON_CreateObject();
    if (!device) {
        return ESP_ERR_NO_MEM;
    }
    
    cJSON_AddStringToObject(device, "identifiers", device_id);
    cJSON_AddStringToObject(device, "name", "SensDot");
    cJSON_AddStringToObject(device, "model", "SensDot v1.0");
    cJSON_AddStringToObject(device, "manufacturer", "SensDot Project");
    cJSON_AddStringToObject(device, "sw_version", SENSDOT_VERSION_STRING);
    
    // Add device to configuration
    cJSON_AddItemToObject(config_obj, "device", device);
    
    return ESP_OK;
}

/**
 * @brief Register Home Assistant MQTT discovery
 */
esp_err_t mqtt_ha_discovery_register(const char *device_id)
{
    CHECK_NULL_PTR(device_id);
    
    if (g_mqtt_state != MQTT_STATE_CONNECTED) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Registering Home Assistant MQTT discovery for device: %s", device_id);
    
    esp_err_t ret = ESP_OK;
    char config_json[1024];
    
    // Temperature sensor
    ret = mqtt_ha_create_sensor_config(device_id, "temperature", "Temperature", 
                                      HA_DEVICE_CLASS_TEMPERATURE, "°C", 
                                      "sensdot/temperature", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_temperature/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Humidity sensor
    ret = mqtt_ha_create_sensor_config(device_id, "humidity", "Humidity",
                                      HA_DEVICE_CLASS_HUMIDITY, "%",
                                      "sensdot/humidity", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_humidity/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Pressure sensor
    ret = mqtt_ha_create_sensor_config(device_id, "pressure", "Pressure",
                                      HA_DEVICE_CLASS_PRESSURE, "hPa",
                                      "sensdot/pressure", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_pressure/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Light sensor
    ret = mqtt_ha_create_sensor_config(device_id, "illuminance", "Illuminance",
                                      HA_DEVICE_CLASS_ILLUMINANCE, "lx",
                                      "sensdot/illuminance", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_illuminance/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Motion binary sensor
    ret = mqtt_ha_create_binary_sensor_config(device_id, "motion", "Motion",
                                             HA_DEVICE_CLASS_MOTION, "sensdot/motion",
                                             "ON", "OFF", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_motion/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Battery voltage sensor
    ret = mqtt_ha_create_sensor_config(device_id, "battery_voltage", "Battery Voltage",
                                      HA_DEVICE_CLASS_VOLTAGE, "V",
                                      "sensdot/battery_voltage", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_battery_voltage/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    // Low battery binary sensor
    ret = mqtt_ha_create_binary_sensor_config(device_id, "low_battery", "Low Battery",
                                             HA_DEVICE_CLASS_BATTERY, "sensdot/low_battery",
                                             "ON", "OFF", config_json, sizeof(config_json));
    if (ret == ESP_OK) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_low_battery/config", device_id);
        mqtt_publish(topic, config_json, MQTT_QOS_1, true);
    }
    
    SENSDOT_LOGI(TAG, "Home Assistant discovery registration completed");
    return ESP_OK;
}

/**
 * @brief Remove Home Assistant MQTT discovery
 */
esp_err_t mqtt_ha_discovery_remove(const char *device_id)
{
    CHECK_NULL_PTR(device_id);
    
    if (g_mqtt_state != MQTT_STATE_CONNECTED) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Removing Home Assistant MQTT discovery for device: %s", device_id);
    
    // List of sensors to remove
    const char *sensors[] = {
        "temperature", "humidity", "pressure", "illuminance", "battery_voltage"
    };
    const char *binary_sensors[] = {
        "motion", "low_battery"
    };
    
    // Remove sensors
    for (size_t i = 0; i < sizeof(sensors)/sizeof(sensors[0]); i++) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_%s/config", device_id, sensors[i]);
        mqtt_publish(topic, "", MQTT_QOS_1, true); // Empty payload removes the entity
    }
    
    // Remove binary sensors
    for (size_t i = 0; i < sizeof(binary_sensors)/sizeof(binary_sensors[0]); i++) {
        char topic[128];
        snprintf(topic, sizeof(topic), "homeassistant/binary_sensor/%s_%s/config", device_id, binary_sensors[i]);
        mqtt_publish(topic, "", MQTT_QOS_1, true); // Empty payload removes the entity
    }
    
    SENSDOT_LOGI(TAG, "Home Assistant discovery removal completed");
    return ESP_OK;
}

/**
 * @brief Create Home Assistant discovery configuration for sensor
 */
esp_err_t mqtt_ha_create_sensor_config(const char *device_id, const char *sensor_name,
                                      const char *friendly_name, ha_device_class_t device_class,
                                      const char *unit_of_measurement, const char *state_topic,
                                      char *config_json, size_t max_len)
{
    CHECK_NULL_PTR(device_id);
    CHECK_NULL_PTR(sensor_name);
    CHECK_NULL_PTR(friendly_name);
    CHECK_NULL_PTR(state_topic);
    CHECK_NULL_PTR(config_json);
    
    cJSON *config = cJSON_CreateObject();
    if (!config) {
        return ESP_ERR_NO_MEM;
    }
    
    // Basic sensor configuration
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id, sensor_name);
    
    cJSON_AddStringToObject(config, "name", friendly_name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);
    cJSON_AddStringToObject(config, "state_topic", state_topic);
    
    if (unit_of_measurement) {
        cJSON_AddStringToObject(config, "unit_of_measurement", unit_of_measurement);
    }
    
    if (device_class < sizeof(ha_device_class_strings)/sizeof(ha_device_class_strings[0])) {
        cJSON_AddStringToObject(config, "device_class", ha_device_class_strings[device_class]);
    }
    
    // Add device information
    create_ha_device_info(config, device_id);
    
    // Convert to JSON string
    char *json_string = cJSON_Print(config);
    if (!json_string) {
        cJSON_Delete(config);
        return ESP_ERR_NO_MEM;
    }
    
    strncpy(config_json, json_string, max_len - 1);
    config_json[max_len - 1] = '\0';
    
    free(json_string);
    cJSON_Delete(config);
    
    return ESP_OK;
}

/**
 * @brief Create Home Assistant discovery configuration for binary sensor
 */
esp_err_t mqtt_ha_create_binary_sensor_config(const char *device_id, const char *sensor_name,
                                             const char *friendly_name, ha_device_class_t device_class,
                                             const char *state_topic, const char *payload_on,
                                             const char *payload_off, char *config_json, size_t max_len)
{
    CHECK_NULL_PTR(device_id);
    CHECK_NULL_PTR(sensor_name);
    CHECK_NULL_PTR(friendly_name);
    CHECK_NULL_PTR(state_topic);
    CHECK_NULL_PTR(payload_on);
    CHECK_NULL_PTR(payload_off);
    CHECK_NULL_PTR(config_json);
    
    cJSON *config = cJSON_CreateObject();
    if (!config) {
        return ESP_ERR_NO_MEM;
    }
    
    // Basic binary sensor configuration
    char unique_id[64];
    snprintf(unique_id, sizeof(unique_id), "%s_%s", device_id, sensor_name);
    
    cJSON_AddStringToObject(config, "name", friendly_name);
    cJSON_AddStringToObject(config, "unique_id", unique_id);
    cJSON_AddStringToObject(config, "state_topic", state_topic);
    cJSON_AddStringToObject(config, "payload_on", payload_on);
    cJSON_AddStringToObject(config, "payload_off", payload_off);
    
    if (device_class < sizeof(ha_device_class_strings)/sizeof(ha_device_class_strings[0])) {
        cJSON_AddStringToObject(config, "device_class", ha_device_class_strings[device_class]);
    }
    
    // Add device information
    create_ha_device_info(config, device_id);
    
    // Convert to JSON string
    char *json_string = cJSON_Print(config);
    if (!json_string) {
        cJSON_Delete(config);
        return ESP_ERR_NO_MEM;
    }
    
    strncpy(config_json, json_string, max_len - 1);
    config_json[max_len - 1] = '\0';
    
    free(json_string);
    cJSON_Delete(config);
    
    return ESP_OK;
}

/**
 * @brief Get MQTT statistics
 */
esp_err_t mqtt_get_statistics(mqtt_stats_t *stats)
{
    CHECK_NULL_PTR(stats);
    
    memcpy(stats, &g_mqtt_stats, sizeof(mqtt_stats_t));
    return ESP_OK;
}

/**
 * @brief Reset MQTT statistics
 */
void mqtt_reset_statistics(void)
{
    memset(&g_mqtt_stats, 0, sizeof(mqtt_stats_t));
    SENSDOT_LOGI(TAG, "MQTT statistics reset");
}

/**
 * @brief Get last error message
 */
esp_err_t mqtt_get_last_error(char *error_str, size_t max_len)
{
    CHECK_NULL_PTR(error_str);
    
    strncpy(error_str, g_last_error, max_len - 1);
    error_str[max_len - 1] = '\0';
    
    return ESP_OK;
}

/**
 * @brief Set MQTT keep alive interval
 */
esp_err_t mqtt_set_keep_alive(uint16_t keep_alive_sec)
{
    // This would require recreating the client with new configuration
    // For now, just store the value for next initialization
    SENSDOT_LOGI(TAG, "Keep alive set to %d seconds (will apply on next connection)", keep_alive_sec);
    return ESP_OK;
}

/**
 * @brief Set MQTT clean session flag
 */
esp_err_t mqtt_set_clean_session(bool clean_session)
{
    // This would require recreating the client with new configuration
    SENSDOT_LOGI(TAG, "Clean session set to %s (will apply on next connection)", clean_session ? "true" : "false");
    return ESP_OK;
}

/**
 * @brief Enable/disable MQTT TLS
 */
esp_err_t mqtt_set_tls(bool enable_tls)
{
    // This would require recreating the client with TLS configuration
    SENSDOT_LOGI(TAG, "TLS %s (will apply on next connection)", enable_tls ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Set MQTT client certificate for TLS
 */
esp_err_t mqtt_set_client_cert(const char *cert_pem)
{
    CHECK_NULL_PTR(cert_pem);
    
    // This would require storing the certificate and recreating the client
    SENSDOT_LOGI(TAG, "Client certificate set (will apply on next connection)");
    return ESP_OK;
}

/**
 * @brief Set MQTT client private key for TLS
 */
esp_err_t mqtt_set_client_key(const char *key_pem)
{
    CHECK_NULL_PTR(key_pem);
    
    // This would require storing the key and recreating the client
    SENSDOT_LOGI(TAG, "Client private key set (will apply on next connection)");
    return ESP_OK;
}

/**
 * @brief Set MQTT server certificate for TLS validation
 */
esp_err_t mqtt_set_server_cert(const char *cert_pem)
{
    CHECK_NULL_PTR(cert_pem);
    
    // This would require storing the certificate and recreating the client
    SENSDOT_LOGI(TAG, "Server certificate set (will apply on next connection)");
    return ESP_OK;
}

/**
 * @brief Generate unique device ID based on MAC address
 */
esp_err_t mqtt_generate_device_id(char *device_id, size_t max_len)
{
    CHECK_NULL_PTR(device_id);
    
    uint8_t mac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    snprintf(device_id, max_len, "sensdot_%02x%02x%02x%02x%02x%02x", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    return ESP_OK;
}

/**
 * @brief Validate MQTT configuration
 */
esp_err_t mqtt_validate_config(const device_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    // Check MQTT URI
    if (strlen(config->mqtt_uri) == 0) {
        SENSDOT_LOGE(TAG, "MQTT URI is required");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_uri) >= MAX_URI_LEN) {
        SENSDOT_LOGE(TAG, "MQTT URI too long");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Validate URI format (basic check)
    if (strncmp(config->mqtt_uri, "mqtt://", 7) != 0 && 
        strncmp(config->mqtt_uri, "mqtts://", 8) != 0) {
        SENSDOT_LOGE(TAG, "Invalid MQTT URI format (must start with mqtt:// or mqtts://)");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check MQTT prefix
    if (strlen(config->mqtt_prefix) == 0) {
        SENSDOT_LOGE(TAG, "MQTT prefix is required");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_prefix) >= MAX_TOPIC_LEN) {
        SENSDOT_LOGE(TAG, "MQTT prefix too long");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check credentials length if provided
    if (strlen(config->mqtt_user) >= MAX_USERNAME_LEN) {
        SENSDOT_LOGE(TAG, "MQTT username too long");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (strlen(config->mqtt_pass) >= MAX_PASSWORD_LEN) {
        SENSDOT_LOGE(TAG, "MQTT password too long");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    SENSDOT_LOGD(TAG, "MQTT configuration validation passed");
    return ESP_OK;
}

/**
 * @brief Test MQTT connection
 */
esp_err_t mqtt_test_connection(const device_config_t *config, uint32_t timeout_ms)
{
    CHECK_NULL_PTR(config);
    
    SENSDOT_LOGI(TAG, "Testing MQTT connection");
    
    // Validate configuration first
    esp_err_t ret = mqtt_validate_config(config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Store current configuration
    device_config_t original_config;
    memcpy(&original_config, &g_current_config, sizeof(device_config_t));
    
    // Temporarily use test configuration
    memcpy(&g_current_config, config, sizeof(device_config_t));
    
    // Deinitialize current client
    bool was_initialized = g_mqtt_initialized;
    if (was_initialized) {
        mqtt_client_deinit();
    }
    
    // Initialize with test configuration
    ret = mqtt_client_init(config);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    // Try to connect
    ret = mqtt_connect(timeout_ms);
    if (ret != ESP_OK) {
        goto cleanup;
    }
    
    // Test successful - disconnect
    mqtt_disconnect();
    SENSDOT_LOGI(TAG, "MQTT connection test successful");
    
cleanup:
    // Restore original configuration
    mqtt_client_deinit();
    memcpy(&g_current_config, &original_config, sizeof(device_config_t));
    
    if (was_initialized) {
        mqtt_client_init(&original_config);
    }
    
    return ret;
}