/**
 * @file mqtt_client.h
 * @brief MQTT client and Home Assistant integration for SensDot device
 */

#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include "sensdot_common.h"
#include "mqtt_client.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// MQTT connection states
typedef enum {
    MQTT_STATE_DISCONNECTED,
    MQTT_STATE_CONNECTING,
    MQTT_STATE_CONNECTED,
    MQTT_STATE_ERROR
} mqtt_state_t;

// MQTT Quality of Service levels
typedef enum {
    MQTT_QOS_0 = 0,  // At most once
    MQTT_QOS_1 = 1,  // At least once
    MQTT_QOS_2 = 2   // Exactly once
} mqtt_qos_t;

// Home Assistant device classes
typedef enum {
    HA_DEVICE_CLASS_TEMPERATURE,
    HA_DEVICE_CLASS_HUMIDITY,
    HA_DEVICE_CLASS_PRESSURE,
    HA_DEVICE_CLASS_ILLUMINANCE,
    HA_DEVICE_CLASS_MOTION,
    HA_DEVICE_CLASS_BATTERY,
    HA_DEVICE_CLASS_VOLTAGE
} ha_device_class_t;

// MQTT message structure
typedef struct {
    char topic[128];
    char payload[512];
    mqtt_qos_t qos;
    bool retain;
} mqtt_message_t;

// MQTT statistics
typedef struct {
    uint32_t messages_sent;
    uint32_t messages_failed;
    uint32_t connections;
    uint32_t disconnections;
    time_t last_publish;
    time_t last_connection;
} mqtt_stats_t;

/**
 * @brief Initialize MQTT client
 * @param config Device configuration containing MQTT settings
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_client_init(const device_config_t *config);

/**
 * @brief Deinitialize MQTT client
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_client_deinit(void);

/**
 * @brief Start MQTT client
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_client_start(void);

/**
 * @brief Stop MQTT client
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_client_stop(void);

/**
 * @brief Connect to MQTT broker
 * @param timeout_ms Connection timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_connect(uint32_t timeout_ms);

/**
 * @brief Disconnect from MQTT broker
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_disconnect(void);

/**
 * @brief Publish message to MQTT topic
 * @param topic Topic to publish to
 * @param payload Message payload
 * @param qos Quality of service level
 * @param retain Retain flag
 * @return Message ID on success, -1 on error
 */
int mqtt_publish(const char *topic, const char *payload, mqtt_qos_t qos, bool retain);

/**
 * @brief Publish message to topic relative to device prefix
 * @param topic_suffix Topic suffix (will be prefixed with device topic)
 * @param payload Message payload
 * @return Message ID on success, -1 on error
 */
int mqtt_publish_relative(const char *topic_suffix, const char *payload);

/**
 * @brief Publish all sensor data
 * @param data Sensor data to publish
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_publish_sensor_data(const sensor_data_t *data);

/**
 * @brief Publish device status
 * @param status Status string to publish
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_publish_status(const char *status);

/**
 * @brief Get current MQTT state
 * @return Current MQTT state
 */
mqtt_state_t mqtt_get_state(void);

/**
 * @brief Check if MQTT is connected
 * @return true if connected, false otherwise
 */
bool mqtt_is_connected(void);

/**
 * @brief Subscribe to MQTT topic
 * @param topic Topic to subscribe to
 * @param qos Quality of service level
 * @return Message ID on success, -1 on error
 */
int mqtt_subscribe(const char *topic, mqtt_qos_t qos);

/**
 * @brief Unsubscribe from MQTT topic
 * @param topic Topic to unsubscribe from
 * @return Message ID on success, -1 on error
 */
int mqtt_unsubscribe(const char *topic);

/**
 * @brief Set MQTT message callback
 * @param callback Callback function for received messages
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*mqtt_message_callback_t)(const char *topic, const char *payload, void *user_data);
esp_err_t mqtt_set_message_callback(mqtt_message_callback_t callback, void *user_data);

/**
 * @brief Register Home Assistant MQTT discovery
 * @param device_id Unique device identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_ha_discovery_register(const char *device_id);

/**
 * @brief Remove Home Assistant MQTT discovery
 * @param device_id Unique device identifier
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_ha_discovery_remove(const char *device_id);

/**
 * @brief Create Home Assistant discovery configuration for sensor
 * @param device_id Unique device identifier
 * @param sensor_name Sensor name (e.g., "temp", "hum")
 * @param friendly_name Human-readable name
 * @param device_class Device class for Home Assistant
 * @param unit_of_measurement Unit of measurement
 * @param state_topic Topic where sensor publishes data
 * @param config_json Buffer to store generated JSON configuration
 * @param max_len Maximum length of configuration JSON
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_ha_create_sensor_config(
    const char *device_id,
    const char *sensor_name,
    const char *friendly_name,
    ha_device_class_t device_class,
    const char *unit_of_measurement,
    const char *state_topic,
    char *config_json,
    size_t max_len
);

/**
 * @brief Create Home Assistant discovery configuration for binary sensor
 * @param device_id Unique device identifier
 * @param sensor_name Sensor name (e.g., "motion", "low_batt")
 * @param friendly_name Human-readable name
 * @param device_class Device class for Home Assistant
 * @param state_topic Topic where sensor publishes data
 * @param payload_on Payload for "on" state
 * @param payload_off Payload for "off" state
 * @param config_json Buffer to store generated JSON configuration
 * @param max_len Maximum length of configuration JSON
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_ha_create_binary_sensor_config(
    const char *device_id,
    const char *sensor_name,
    const char *friendly_name,
    ha_device_class_t device_class,
    const char *state_topic,
    const char *payload_on,
    const char *payload_off,
    char *config_json,
    size_t max_len
);

/**
 * @brief Get MQTT statistics
 * @param stats Pointer to store statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_get_statistics(mqtt_stats_t *stats);

/**
 * @brief Reset MQTT statistics
 */
void mqtt_reset_statistics(void);

/**
 * @brief Get last error message
 * @param error_str Buffer to store error message
 * @param max_len Maximum length of error string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_get_last_error(char *error_str, size_t max_len);

/**
 * @brief Set MQTT keep alive interval
 * @param keep_alive_sec Keep alive interval in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_keep_alive(uint16_t keep_alive_sec);

/**
 * @brief Set MQTT clean session flag
 * @param clean_session Clean session flag
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_clean_session(bool clean_session);

/**
 * @brief Enable/disable MQTT TLS
 * @param enable_tls Enable TLS flag
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_tls(bool enable_tls);

/**
 * @brief Set MQTT client certificate for TLS
 * @param cert_pem Client certificate in PEM format
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_client_cert(const char *cert_pem);

/**
 * @brief Set MQTT client private key for TLS
 * @param key_pem Private key in PEM format
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_client_key(const char *key_pem);

/**
 * @brief Set MQTT server certificate for TLS validation
 * @param cert_pem Server certificate in PEM format
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_set_server_cert(const char *cert_pem);

/**
 * @brief Generate unique device ID based on MAC address
 * @param device_id Buffer to store generated device ID
 * @param max_len Maximum length of device ID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t mqtt_generate_device_id(char *device_id, size_t max_len);

/**
 * @brief Validate MQTT configuration
 * @param config Configuration to validate
 * @return ESP_OK if valid, error code otherwise
 */
esp_err_t mqtt_validate_config(const device_config_t *config);

/**
 * @brief Test MQTT connection
 * @param config Configuration to test
 * @param timeout_ms Connection timeout in milliseconds
 * @return ESP_OK if connection successful, error code otherwise
 */
esp_err_t mqtt_test_connection(const device_config_t *config, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // MQTT_CLIENT_H