/**
 * @file wifi_manager.c
 * @brief WiFi connection management implementation for SensDot device
 */

#include "wifi_manager.h"
#include "sensdot_common.h"

static const char *TAG = "wifi_mgr";

// WiFi manager state
static bool g_wifi_initialized = false;
static wifi_state_t g_wifi_state = WIFI_STATE_IDLE;
static EventGroupHandle_t g_wifi_event_group = NULL;
static uint32_t g_retry_count = 0;
static wifi_event_callback_t g_event_callback = NULL;
static void *g_callback_user_data = NULL;

// Current WiFi configuration
static wifi_config_t g_wifi_config = {0};
static bool g_power_save_enabled = true;

// Forward declarations
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t wifi_set_state(wifi_state_t new_state);

/**
 * @brief Set WiFi state and call callback
 */
static esp_err_t wifi_set_state(wifi_state_t new_state)
{
    if (g_wifi_state != new_state) {
        wifi_state_t old_state = g_wifi_state;
        g_wifi_state = new_state;
        
        SENSDOT_LOGD(TAG, "WiFi state changed: %d -> %d", old_state, new_state);
        
        // Call event callback if registered
        if (g_event_callback) {
            g_event_callback(new_state, g_callback_user_data);
        }
    }
    
    return ESP_OK;
}

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                SENSDOT_LOGI(TAG, "WiFi station started");
                wifi_set_state(WIFI_STATE_IDLE);
                break;
                
            case WIFI_EVENT_STA_STOP:
                SENSDOT_LOGI(TAG, "WiFi station stopped");
                wifi_set_state(WIFI_STATE_IDLE);
                break;
                
            case WIFI_EVENT_STA_CONNECTED:
                SENSDOT_LOGI(TAG, "WiFi connected to AP");
                wifi_set_state(WIFI_STATE_CONNECTING);
                break;
                
            case WIFI_EVENT_STA_DISCONNECTED:
                {
                    wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                    SENSDOT_LOGW(TAG, "WiFi disconnected, reason: %d", event->reason);
                    
                    g_retry_count++;
                    wifi_set_state(WIFI_STATE_DISCONNECTED);
                    
                    if (g_wifi_event_group) {
                        xEventGroupSetBits(g_wifi_event_group, WIFI_FAIL_BIT);
                    }
                }
                break;
                
            case WIFI_EVENT_AP_START:
                SENSDOT_LOGI(TAG, "WiFi AP started");
                wifi_set_state(WIFI_STATE_AP_MODE);
                break;
                
            case WIFI_EVENT_AP_STOP:
                SENSDOT_LOGI(TAG, "WiFi AP stopped");
                wifi_set_state(WIFI_STATE_IDLE);
                break;
                
            case WIFI_EVENT_AP_STACONNECTED:
                {
                    wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                    SENSDOT_LOGI(TAG, "Station connected to AP, MAC: " MACSTR, MAC2STR(event->mac));
                }
                break;
                
            case WIFI_EVENT_AP_STADISCONNECTED:
                {
                    wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                    SENSDOT_LOGI(TAG, "Station disconnected from AP, MAC: " MACSTR, MAC2STR(event->mac));
                }
                break;
                
            case WIFI_EVENT_SCAN_DONE:
                SENSDOT_LOGD(TAG, "WiFi scan completed");
                if (g_wifi_event_group) {
                    xEventGroupSetBits(g_wifi_event_group, WIFI_SCAN_DONE_BIT);
                }
                break;
                
            default:
                SENSDOT_LOGD(TAG, "Unhandled WiFi event: %ld", event_id);
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                {
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                    SENSDOT_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
                    
                    g_retry_count = 0;
                    wifi_set_state(WIFI_STATE_CONNECTED);
                    
                    if (g_wifi_event_group) {
                        xEventGroupSetBits(g_wifi_event_group, WIFI_CONNECTED_BIT);
                    }
                }
                break;
                
            case IP_EVENT_STA_LOST_IP:
                SENSDOT_LOGW(TAG, "Lost IP address");
                wifi_set_state(WIFI_STATE_DISCONNECTED);
                break;
                
            default:
                SENSDOT_LOGD(TAG, "Unhandled IP event: %ld", event_id);
                break;
        }
    }
}

/**
 * @brief Initialize WiFi manager
 */
esp_err_t wifi_manager_init(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Initializing WiFi manager");
    
    if (g_wifi_initialized) {
        SENSDOT_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }
    
    // Create event group
    g_wifi_event_group = xEventGroupCreate();
    if (g_wifi_event_group == NULL) {
        SENSDOT_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_FAIL;
    }
    
    // Initialize TCP/IP stack
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize netif: %s", esp_err_to_name(ret));
        vEventGroupDelete(g_wifi_event_group);
        return ret;
    }
    
    // Create default event loop
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        SENSDOT_LOGE(TAG, "Failed to create event loop: %s", esp_err_to_name(ret));
        vEventGroupDelete(g_wifi_event_group);
        return ret;
    }
    
    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize WiFi: %s", esp_err_to_name(ret));
        vEventGroupDelete(g_wifi_event_group);
        return ret;
    }
    
    // Register event handlers
    ret = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to register WiFi event handler: %s", esp_err_to_name(ret));
        esp_wifi_deinit();
        vEventGroupDelete(g_wifi_event_group);
        return ret;
    }
    
    ret = esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to register IP event handler: %s", esp_err_to_name(ret));
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        esp_wifi_deinit();
        vEventGroupDelete(g_wifi_event_group);
        return ret;
    }
    
    // Initialize state
    g_wifi_state = WIFI_STATE_IDLE;
    g_retry_count = 0;
    g_power_save_enabled = true;
    
    g_wifi_initialized = true;
    
    SENSDOT_LOGI(TAG, "WiFi manager initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize WiFi manager
 */
esp_err_t wifi_manager_deinit(void)
{
    if (!g_wifi_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Deinitializing WiFi manager");
    
    // Stop WiFi if running
    wifi_stop();
    
    // Unregister event handlers
    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
    
    // Deinitialize WiFi
    esp_wifi_deinit();
    
    // Delete event group
    if (g_wifi_event_group) {
        vEventGroupDelete(g_wifi_event_group);
        g_wifi_event_group = NULL;
    }
    
    g_wifi_initialized = false;
    g_wifi_state = WIFI_STATE_IDLE;
    
    SENSDOT_LOGI(TAG, "WiFi manager deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Start WiFi in station mode
 */
esp_err_t wifi_start_sta(const char *ssid, const char *password)
{
    CHECK_NULL_PTR(ssid);
    
    if (!g_wifi_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Starting WiFi station mode, SSID: %s", ssid);
    
    // Validate credentials
    ret = wifi_validate_credentials(ssid, password);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Stop WiFi if already running
    wifi_stop();
    
    // Create station netif
    esp_netif_create_default_wifi_sta();
    
    // Configure WiFi
    memset(&g_wifi_config, 0, sizeof(wifi_config_t));
    strncpy((char *)g_wifi_config.sta.ssid, ssid, sizeof(g_wifi_config.sta.ssid) - 1);
    if (password) {
        strncpy((char *)g_wifi_config.sta.password, password, sizeof(g_wifi_config.sta.password) - 1);
    }
    
    g_wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    g_wifi_config.sta.pmf_cfg.capable = true;
    g_wifi_config.sta.pmf_cfg.required = false;
    
    // Set WiFi mode and configuration
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_STA, &g_wifi_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set power saving mode
    wifi_set_power_save(g_power_save_enabled);
    
    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    wifi_set_state(WIFI_STATE_IDLE);
    
    SENSDOT_LOGI(TAG, "WiFi station started successfully");
    return ESP_OK;
}

/**
 * @brief Start WiFi in access point mode
 */
esp_err_t wifi_start_ap(const char *ssid, const char *password)
{
    CHECK_NULL_PTR(ssid);
    
    if (!g_wifi_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Starting WiFi AP mode, SSID: %s", ssid);
    
    // Stop WiFi if already running
    wifi_stop();
    
    // Create AP netif
    esp_netif_create_default_wifi_ap();
    
    // Configure WiFi AP
    memset(&g_wifi_config, 0, sizeof(wifi_config_t));
    strncpy((char *)g_wifi_config.ap.ssid, ssid, sizeof(g_wifi_config.ap.ssid) - 1);
    g_wifi_config.ap.ssid_len = strlen(ssid);
    g_wifi_config.ap.channel = 1;
    g_wifi_config.ap.max_connection = SETUP_AP_MAX_CONNECTIONS;
    
    if (password && strlen(password) > 0) {
        strncpy((char *)g_wifi_config.ap.password, password, sizeof(g_wifi_config.ap.password) - 1);
        g_wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    } else {
        g_wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    // Set WiFi mode and configuration
    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_set_config(WIFI_IF_AP, &g_wifi_config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to set WiFi config: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "WiFi AP started successfully");
    return ESP_OK;
}

/**
 * @brief Stop WiFi
 */
esp_err_t wifi_stop(void)
{
    if (!g_wifi_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGD(TAG, "Stopping WiFi");
    
    esp_err_t ret = esp_wifi_stop();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STARTED) {
        SENSDOT_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    wifi_set_state(WIFI_STATE_IDLE);
    g_retry_count = 0;
    
    SENSDOT_LOGD(TAG, "WiFi stopped");
    return ESP_OK;
}

/**
 * @brief Connect to WiFi network
 */
esp_err_t wifi_connect(uint32_t timeout_ms)
{
    if (!g_wifi_initialized || g_wifi_state != WIFI_STATE_IDLE) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Connecting to WiFi network");
    
    wifi_set_state(WIFI_STATE_CONNECTING);
    
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi connection: %s", esp_err_to_name(ret));
        wifi_set_state(WIFI_STATE_ERROR);
        return ret;
    }
    
    return wifi_wait_connection(timeout_ms);
}

/**
 * @brief Disconnect from WiFi network
 */
esp_err_t wifi_disconnect(void)
{
    if (!g_wifi_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGI(TAG, "Disconnecting from WiFi");
    
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    wifi_set_state(WIFI_STATE_DISCONNECTED);
    return ESP_OK;
}

/**
 * @brief Scan for available WiFi networks
 */
esp_err_t wifi_scan_networks(wifi_scan_result_t *results, size_t max_results, size_t *actual_results)
{
    CHECK_NULL_PTR(results);
    CHECK_NULL_PTR(actual_results);
    
    if (!g_wifi_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Scanning for WiFi networks");
    
    // Clear event bits
    if (g_wifi_event_group) {
        xEventGroupClearBits(g_wifi_event_group, WIFI_SCAN_DONE_BIT);
    }
    
    // Start scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };
    
    ret = esp_wifi_scan_start(&scan_config, false);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start WiFi scan: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Wait for scan completion
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group, WIFI_SCAN_DONE_BIT, 
                                          pdTRUE, pdFALSE, pdMS_TO_TICKS(10000));
    
    if (!(bits & WIFI_SCAN_DONE_BIT)) {
        SENSDOT_LOGE(TAG, "WiFi scan timeout");
        return ESP_ERR_TIMEOUT;
    }
    
    // Get scan results
    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get AP count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (ap_count == 0) {
        *actual_results = 0;
        SENSDOT_LOGW(TAG, "No WiFi networks found");
        return ESP_OK;
    }
    
    // Limit to available buffer size
    uint16_t get_count = MIN(ap_count, max_results);
    wifi_ap_record_t *ap_info = malloc(sizeof(wifi_ap_record_t) * get_count);
    if (!ap_info) {
        SENSDOT_LOGE(TAG, "Failed to allocate memory for scan results");
        return ESP_ERR_NO_MEM;
    }
    
    ret = esp_wifi_scan_get_ap_records(&get_count, ap_info);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get AP records: %s", esp_err_to_name(ret));
        free(ap_info);
        return ret;
    }
    
    // Convert to our format
    for (uint16_t i = 0; i < get_count; i++) {
        strncpy(results[i].ssid, (char *)ap_info[i].ssid, sizeof(results[i].ssid) - 1);
        results[i].ssid[sizeof(results[i].ssid) - 1] = '\0';
        results[i].rssi = ap_info[i].rssi;
        results[i].auth_required = (ap_info[i].authmode != WIFI_AUTH_OPEN);
    }
    
    *actual_results = get_count;
    free(ap_info);
    
    SENSDOT_LOGI(TAG, "WiFi scan completed, found %d networks", get_count);
    return ESP_OK;
}

/**
 * @brief Get current WiFi state
 */
wifi_state_t wifi_get_state(void)
{
    return g_wifi_state;
}

/**
 * @brief Check if WiFi is connected
 */
bool wifi_is_connected(void)
{
    return g_wifi_state == WIFI_STATE_CONNECTED;
}

/**
 * @brief Get WiFi connection info
 */
esp_err_t wifi_get_connection_info(wifi_ap_record_t *info)
{
    CHECK_NULL_PTR(info);
    
    if (!wifi_is_connected()) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = esp_wifi_sta_get_ap_info(info);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get AP info: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Get IP address
 */
esp_err_t wifi_get_ip_address(char *ip_str, size_t max_len)
{
    CHECK_NULL_PTR(ip_str);
    
    if (!wifi_is_connected()) {
        strncpy(ip_str, "0.0.0.0", max_len - 1);
        ip_str[max_len - 1] = '\0';
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return ESP_FAIL;
    }
    
    esp_netif_ip_info_t ip_info;
    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        return ret;
    }
    
    snprintf(ip_str, max_len, IPSTR, IP2STR(&ip_info.ip));
    return ESP_OK;
}

/**
 * @brief Get MAC address
 */
esp_err_t wifi_get_mac_address(char *mac_str, size_t max_len)
{
    CHECK_NULL_PTR(mac_str);
    
    uint8_t mac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, mac);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    snprintf(mac_str, max_len, MACSTR, MAC2STR(mac));
    return ESP_OK;
}

/**
 * @brief Get signal strength (RSSI)
 */
int8_t wifi_get_rssi(void)
{
    if (!wifi_is_connected()) {
        return -100; // Very weak signal
    }
    
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        return ap_info.rssi;
    }
    
    return -100;
}

/**
 * @brief Set WiFi power saving mode
 */
esp_err_t wifi_set_power_save(bool enabled)
{
    g_power_save_enabled = enabled;
    
    wifi_ps_type_t ps_type = enabled ? WIFI_PS_MIN_MODEM : WIFI_PS_NONE;
    esp_err_t ret = esp_wifi_set_ps(ps_type);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to set power save mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "WiFi power save %s", enabled ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Get WiFi power saving status
 */
bool wifi_get_power_save(void)
{
    return g_power_save_enabled;
}

/**
 * @brief Set WiFi event callback
 */
esp_err_t wifi_set_event_callback(wifi_event_callback_t callback, void *user_data)
{
    g_event_callback = callback;
    g_callback_user_data = user_data;
    return ESP_OK;
}

/**
 * @brief Wait for WiFi connection
 */
esp_err_t wifi_wait_connection(uint32_t timeout_ms)
{
    if (!g_wifi_event_group) {
        return ESP_FAIL;
    }
    
    EventBits_t bits = xEventGroupWaitBits(g_wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdTRUE, pdFALSE, 
                                          pdMS_TO_TICKS(timeout_ms));
    
    if (bits & WIFI_CONNECTED_BIT) {
        SENSDOT_LOGI(TAG, "WiFi connected successfully");
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        SENSDOT_LOGE(TAG, "WiFi connection failed");
        wifi_set_state(WIFI_STATE_ERROR);
        return SENSDOT_ERR_WIFI_FAIL;
    } else {
        SENSDOT_LOGE(TAG, "WiFi connection timeout");
        wifi_set_state(WIFI_STATE_ERROR);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief Get connection retry count
 */
uint32_t wifi_get_retry_count(void)
{
    return g_retry_count;
}

/**
 * @brief Reset connection retry count
 */
void wifi_reset_retry_count(void)
{
    g_retry_count = 0;
}

/**
 * @brief Check if maximum retries reached
 */
bool wifi_max_retries_reached(uint32_t max_retries)
{
    return g_retry_count >= max_retries;
}

/**
 * @brief Get WiFi configuration
 */
esp_err_t wifi_get_config(wifi_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    memcpy(config, &g_wifi_config, sizeof(wifi_config_t));
    return ESP_OK;
}

/**
 * @brief Set WiFi configuration
 */
esp_err_t wifi_set_config(const wifi_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    memcpy(&g_wifi_config, config, sizeof(wifi_config_t));
    return ESP_OK;
}

/**
 * @brief Generate unique AP SSID based on MAC address
 */
esp_err_t wifi_generate_unique_ssid(const char *base_ssid, char *unique_ssid, size_t max_len)
{
    CHECK_NULL_PTR(base_ssid);
    CHECK_NULL_PTR(unique_ssid);
    
    uint8_t mac[6];
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_AP, mac);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to get MAC address: %s", esp_err_to_name(ret));
        return ret;
    }
    
    snprintf(unique_ssid, max_len, "%s%02X%02X%02X", base_ssid, mac[3], mac[4], mac[5]);
    return ESP_OK;
}

/**
 * @brief Validate WiFi credentials
 */
esp_err_t wifi_validate_credentials(const char *ssid, const char *password)
{
    CHECK_NULL_PTR(ssid);
    
    // Check SSID length
    size_t ssid_len = strlen(ssid);
    if (ssid_len == 0 || ssid_len >= 32) {
        SENSDOT_LOGE(TAG, "Invalid SSID length: %zu", ssid_len);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Check password length if provided
    if (password) {
        size_t pass_len = strlen(password);
        if (pass_len > 0 && (pass_len < 8 || pass_len >= 64)) {
            SENSDOT_LOGE(TAG, "Invalid password length: %zu", pass_len);
            return SENSDOT_ERR_INVALID_ARG;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Get WiFi status string for debugging
 */
esp_err_t wifi_get_status_string(char *status_str, size_t max_len)
{
    CHECK_NULL_PTR(status_str);
    
    const char *state_names[] = {
        "IDLE", "CONNECTING", "CONNECTED", "DISCONNECTED", "AP_MODE", "ERROR"
    };
    
    const char *state_name = (g_wifi_state < sizeof(state_names)/sizeof(state_names[0])) ? 
                            state_names[g_wifi_state] : "UNKNOWN";
    
    char ip_str[16] = "0.0.0.0";
    if (wifi_is_connected()) {
        wifi_get_ip_address(ip_str, sizeof(ip_str));
    }
    
    snprintf(status_str, max_len, "State: %s, IP: %s, Retries: %lu, RSSI: %d dBm", 
            state_name, ip_str, g_retry_count, wifi_get_rssi());
    
    return ESP_OK;
}