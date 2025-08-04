/**
 * @file wifi_manager.h
 * @brief WiFi connection management for SensDot device
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "sensdot_common.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi connection states
typedef enum {
    WIFI_STATE_IDLE,
    WIFI_STATE_CONNECTING,
    WIFI_STATE_CONNECTED,
    WIFI_STATE_DISCONNECTED,
    WIFI_STATE_AP_MODE,
    WIFI_STATE_ERROR
} wifi_state_t;

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define WIFI_SCAN_DONE_BIT BIT2

// Maximum scan results
#define MAX_SCAN_RESULTS 20

// Connection timeout
#define WIFI_CONNECT_TIMEOUT_MS 10000

/**
 * @brief Initialize WiFi manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Deinitialize WiFi manager
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_manager_deinit(void);

/**
 * @brief Start WiFi in station mode
 * @param ssid WiFi network SSID
 * @param password WiFi network password
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_start_sta(const char *ssid, const char *password);

/**
 * @brief Start WiFi in access point mode
 * @param ssid AP SSID (will append MAC if needed)
 * @param password AP password (can be NULL for open network)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_start_ap(const char *ssid, const char *password);

/**
 * @brief Stop WiFi
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_stop(void);

/**
 * @brief Connect to WiFi network
 * @param timeout_ms Connection timeout in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_connect(uint32_t timeout_ms);

/**
 * @brief Disconnect from WiFi network
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_disconnect(void);

/**
 * @brief Scan for available WiFi networks
 * @param results Array to store scan results
 * @param max_results Maximum number of results to return
 * @param actual_results Pointer to store actual number of results found
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_scan_networks(wifi_scan_result_t *results, size_t max_results, size_t *actual_results);

/**
 * @brief Get current WiFi state
 * @return Current WiFi state
 */
wifi_state_t wifi_get_state(void);

/**
 * @brief Check if WiFi is connected
 * @return true if connected, false otherwise
 */
bool wifi_is_connected(void);

/**
 * @brief Get WiFi connection info
 * @param info Pointer to store connection info
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_connection_info(wifi_ap_record_t *info);

/**
 * @brief Get IP address
 * @param ip_str Buffer to store IP address string
 * @param max_len Maximum length of IP string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_ip_address(char *ip_str, size_t max_len);

/**
 * @brief Get MAC address
 * @param mac_str Buffer to store MAC address string
 * @param max_len Maximum length of MAC string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_mac_address(char *mac_str, size_t max_len);

/**
 * @brief Get signal strength (RSSI)
 * @return RSSI value in dBm
 */
int8_t wifi_get_rssi(void);

/**
 * @brief Set WiFi power saving mode
 * @param enabled true to enable power saving, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_set_power_save(bool enabled);

/**
 * @brief Get WiFi power saving status
 * @return true if power saving enabled, false otherwise
 */
bool wifi_get_power_save(void);

/**
 * @brief Set WiFi event callback
 * @param callback Callback function to call on WiFi events
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*wifi_event_callback_t)(wifi_state_t state, void *user_data);
esp_err_t wifi_set_event_callback(wifi_event_callback_t callback, void *user_data);

/**
 * @brief Wait for WiFi connection
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK if connected, ESP_ERR_TIMEOUT if timeout
 */
esp_err_t wifi_wait_connection(uint32_t timeout_ms);

/**
 * @brief Get connection retry count
 * @return Number of connection retries
 */
uint32_t wifi_get_retry_count(void);

/**
 * @brief Reset connection retry count
 */
void wifi_reset_retry_count(void);

/**
 * @brief Check if maximum retries reached
 * @param max_retries Maximum number of retries allowed
 * @return true if max retries reached, false otherwise
 */
bool wifi_max_retries_reached(uint32_t max_retries);

/**
 * @brief Get WiFi configuration
 * @param config Pointer to store WiFi configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_config(wifi_config_t *config);

/**
 * @brief Set WiFi configuration
 * @param config Pointer to WiFi configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_set_config(const wifi_config_t *config);

/**
 * @brief Generate unique AP SSID based on MAC address
 * @param base_ssid Base SSID string
 * @param unique_ssid Buffer to store generated unique SSID
 * @param max_len Maximum length of unique SSID
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_generate_unique_ssid(const char *base_ssid, char *unique_ssid, size_t max_len);

/**
 * @brief Validate WiFi credentials
 * @param ssid SSID to validate
 * @param password Password to validate
 * @return ESP_OK if valid, error code otherwise
 */
esp_err_t wifi_validate_credentials(const char *ssid, const char *password);

/**
 * @brief Get WiFi status string for debugging
 * @param status_str Buffer to store status string
 * @param max_len Maximum length of status string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t wifi_get_status_string(char *status_str, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MANAGER_H