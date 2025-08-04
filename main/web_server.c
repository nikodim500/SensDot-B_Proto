/**
 * @file web_server.c
 * @brief HTTP web server for device configuration implementation
 */

#include "web_server.h"
#include "sensdot_common.h"
#include "config.h"
#include "wifi_manager.h"
#include "indicators.h"

static const char *TAG = "webserver";

// Web server state
static httpd_handle_t g_server = NULL;
static webserver_state_t g_server_state = WEBSERVER_STATE_STOPPED;
static webserver_timeout_callback_t g_timeout_callback = NULL;
static void *g_timeout_user_data = NULL;

// Statistics
static uint32_t g_requests_count = 0;
static uint32_t g_errors_count = 0;
static time_t g_start_time = 0;

// Configuration
static bool g_cors_enabled = true;
static char g_404_page[512] = "<!DOCTYPE html><html><body><h1>404 Not Found</h1></body></html>";

// Forward declarations
static esp_err_t add_cors_headers(httpd_req_t *req);
static esp_err_t get_content_type(const char *uri, char *content_type, size_t max_len);

/**
 * @brief Add CORS headers to response
 */
static esp_err_t add_cors_headers(httpd_req_t *req)
{
    if (!g_cors_enabled) {
        return ESP_OK;
    }
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type, Authorization");
    httpd_resp_set_hdr(req, "Access-Control-Max-Age", "86400");
    
    return ESP_OK;
}

/**
 * @brief Get content type based on URI extension
 */
static esp_err_t get_content_type(const char *uri, char *content_type, size_t max_len)
{
    const char *ext = strrchr(uri, '.');
    if (!ext) {
        strncpy(content_type, "text/html", max_len - 1);
    } else if (strcmp(ext, ".html") == 0 || strcmp(ext, ".htm") == 0) {
        strncpy(content_type, "text/html", max_len - 1);
    } else if (strcmp(ext, ".css") == 0) {
        strncpy(content_type, "text/css", max_len - 1);
    } else if (strcmp(ext, ".js") == 0) {
        strncpy(content_type, "application/javascript", max_len - 1);
    } else if (strcmp(ext, ".json") == 0) {
        strncpy(content_type, "application/json", max_len - 1);
    } else {
        strncpy(content_type, "text/plain", max_len - 1);
    }
    
    content_type[max_len - 1] = '\0';
    return ESP_OK;
}

/**
 * @brief Initialize web server
 */
esp_err_t webserver_init(void)
{
    SENSDOT_LOGI(TAG, "Initializing web server");
    
    g_server_state = WEBSERVER_STATE_STOPPED;
    g_requests_count = 0;
    g_errors_count = 0;
    g_start_time = 0;
    
    SENSDOT_LOGI(TAG, "Web server initialized");
    return ESP_OK;
}

/**
 * @brief Start web server
 */
esp_err_t webserver_start(void)
{
    if (g_server_state == WEBSERVER_STATE_RUNNING) {
        SENSDOT_LOGW(TAG, "Web server already running");
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Starting web server");
    g_server_state = WEBSERVER_STATE_STARTING;
    
    // Configure HTTP server
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.task_priority = WEBSERVER_TASK_PRIORITY;
    config.stack_size = WEBSERVER_STACK_SIZE;
    config.max_uri_handlers = 16;
    config.max_open_sockets = 4;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    
    // Start HTTP server
    esp_err_t ret = httpd_start(&g_server, &config);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        g_server_state = WEBSERVER_STATE_ERROR;
        return ret;
    }
    
    // Register URI handlers
    httpd_uri_t setup_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = webserver_handle_setup_page,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &setup_uri);
    
    httpd_uri_t scan_uri = {
        .uri = "/scan",
        .method = HTTP_GET,
        .handler = webserver_handle_wifi_scan,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &scan_uri);
    
    httpd_uri_t save_uri = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = webserver_handle_config_save,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &save_uri);
    
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = webserver_handle_status,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &status_uri);
    
    httpd_uri_t restart_uri = {
        .uri = "/restart",
        .method = HTTP_GET,
        .handler = webserver_handle_restart,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &restart_uri);
    
    httpd_uri_t factory_reset_uri = {
        .uri = "/factory_reset",
        .method = HTTP_POST,
        .handler = webserver_handle_factory_reset,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(g_server, &factory_reset_uri);
    
    g_server_state = WEBSERVER_STATE_RUNNING;
    g_start_time = time(NULL);
    
    SENSDOT_LOGI(TAG, "Web server started successfully on port %d", config.server_port);
    return ESP_OK;
}

/**
 * @brief Stop web server
 */
esp_err_t webserver_stop(void)
{
    if (g_server_state == WEBSERVER_STATE_STOPPED) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Stopping web server");
    
    if (g_server) {
        esp_err_t ret = httpd_stop(g_server);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to stop HTTP server: %s", esp_err_to_name(ret));
            return ret;
        }
        g_server = NULL;
    }
    
    g_server_state = WEBSERVER_STATE_STOPPED;
    
    SENSDOT_LOGI(TAG, "Web server stopped");
    return ESP_OK;
}

/**
 * @brief Deinitialize web server
 */
esp_err_t webserver_deinit(void)
{
    webserver_stop();
    
    SENSDOT_LOGI(TAG, "Web server deinitialized");
    return ESP_OK;
}

/**
 * @brief Get web server state
 */
webserver_state_t webserver_get_state(void)
{
    return g_server_state;
}

/**
 * @brief Check if web server is running
 */
bool webserver_is_running(void)
{
    return g_server_state == WEBSERVER_STATE_RUNNING;
}

/**
 * @brief Set setup mode timeout callback
 */
void webserver_set_timeout_callback(webserver_timeout_callback_t callback, void *user_data)
{
    g_timeout_callback = callback;
    g_timeout_user_data = user_data;
}

/**
 * @brief Handle setup page request (GET /)
 */
esp_err_t webserver_handle_setup_page(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGD(TAG, "Handling setup page request");
    
    add_cors_headers(req);
    
    char html_content[4096];
    esp_err_t ret = webserver_get_setup_html(html_content, sizeof(html_content));
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "Failed to generate setup page");
    }
    
    return webserver_send_response(req, HTTP_200_OK, "text/html", html_content);
}

/**
 * @brief Handle WiFi scan request (GET /scan)
 */
esp_err_t webserver_handle_wifi_scan(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGD(TAG, "Handling WiFi scan request");
    
    add_cors_headers(req);
    
    // Perform WiFi scan
    wifi_scan_result_t scan_results[MAX_SCAN_RESULTS];
    size_t actual_results = 0;
    
    esp_err_t ret = wifi_scan_networks(scan_results, MAX_SCAN_RESULTS, &actual_results);
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "WiFi scan failed");
    }
    
    // Generate JSON response
    char json_response[2048];
    ret = webserver_generate_scan_json(scan_results, actual_results, json_response, sizeof(json_response));
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "Failed to generate scan results");
    }
    
    return webserver_send_response(req, HTTP_200_OK, "application/json", json_response);
}

/**
 * @brief Handle configuration save request (POST /save)
 */
esp_err_t webserver_handle_config_save(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGD(TAG, "Handling config save request");
    
    add_cors_headers(req);
    
    // Parse JSON from request body
    cJSON *json_config = NULL;
    esp_err_t ret = webserver_parse_json_body(req, &json_config);
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_400_BAD_REQUEST, "Invalid JSON in request body");
    }
    
    // Validate and convert configuration
    device_config_t new_config;
    ret = webserver_validate_config(json_config, &new_config);
    if (ret != ESP_OK) {
        g_errors_count++;
        cJSON_Delete(json_config);
        return webserver_send_error(req, HTTP_400_BAD_REQUEST, "Invalid configuration parameters");
    }
    
    cJSON_Delete(json_config);
    
    // Save configuration
    ret = config_save(&new_config);
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "Failed to save configuration");
    }
    
    // Send success response
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Configuration saved successfully");
    
    ret = webserver_send_json_response(req, HTTP_200_OK, response);
    cJSON_Delete(response);
    
    // Indicate successful configuration save
    indicators_show_config_saved();
    
    return ret;
}

/**
 * @brief Handle device restart request (GET /restart)
 */
esp_err_t webserver_handle_restart(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGI(TAG, "Handling restart request");
    
    add_cors_headers(req);
    
    const char *response_text = "Device restarting...";
    webserver_send_response(req, HTTP_200_OK, "text/plain", response_text);
    
    // Delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Restart device
    esp_restart();
    
    return ESP_OK; // Should not reach here
}

/**
 * @brief Handle device status request (GET /status)
 */
esp_err_t webserver_handle_status(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGD(TAG, "Handling status request");
    
    add_cors_headers(req);
    
    char status_json[1024];
    esp_err_t ret = webserver_generate_status_json(status_json, sizeof(status_json));
    if (ret != ESP_OK) {
        g_errors_count++;
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "Failed to generate status");
    }
    
    return webserver_send_response(req, HTTP_200_OK, "application/json", status_json);
}

/**
 * @brief Handle factory reset request (POST /factory_reset)
 */
esp_err_t webserver_handle_factory_reset(httpd_req_t *req)
{
    g_requests_count++;
    
    SENSDOT_LOGI(TAG, "Handling factory reset request");
    
    add_cors_headers(req);
    
    // Send response before reset
    cJSON *response = cJSON_CreateObject();
    cJSON_AddBoolToObject(response, "success", true);
    cJSON_AddStringToObject(response, "message", "Factory reset initiated");
    
    webserver_send_json_response(req, HTTP_200_OK, response);
    cJSON_Delete(response);
    
    // Delay to allow response to be sent
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Perform factory reset
    indicators_show_factory_reset();
    config_factory_reset();
    
    // Restart device
    esp_restart();
    
    return ESP_OK; // Should not reach here
}

/**
 * @brief Send HTTP response
 */
esp_err_t webserver_send_response(httpd_req_t *req, http_status_code_t status_code, 
                                 const char *content_type, const char *content)
{
    CHECK_NULL_PTR(req);
    CHECK_NULL_PTR(content_type);
    CHECK_NULL_PTR(content);
    
    // Set status code
    char status_str[16];
    snprintf(status_str, sizeof(status_str), "%d", status_code);
    httpd_resp_set_status(req, status_str);
    
    // Set content type
    httpd_resp_set_type(req, content_type);
    
    // Send response
    esp_err_t ret = httpd_resp_send(req, content, strlen(content));
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to send HTTP response: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * @brief Send JSON response
 */
esp_err_t webserver_send_json_response(httpd_req_t *req, http_status_code_t status_code, cJSON *json_obj)
{
    CHECK_NULL_PTR(req);
    CHECK_NULL_PTR(json_obj);
    
    char *json_string = cJSON_Print(json_obj);
    if (!json_string) {
        return webserver_send_error(req, HTTP_500_INTERNAL_ERROR, "Failed to serialize JSON");
    }
    
    esp_err_t ret = webserver_send_response(req, status_code, "application/json", json_string);
    free(json_string);
    
    return ret;
}

/**
 * @brief Send error response
 */
esp_err_t webserver_send_error(httpd_req_t *req, http_status_code_t status_code, const char *error_message)
{
    CHECK_NULL_PTR(req);
    CHECK_NULL_PTR(error_message);
    
    cJSON *error_obj = cJSON_CreateObject();
    cJSON_AddBoolToObject(error_obj, "success", false);
    cJSON_AddStringToObject(error_obj, "error", error_message);
    
    esp_err_t ret = webserver_send_json_response(req, status_code, error_obj);
    cJSON_Delete(error_obj);
    
    return ret;
}

/**
 * @brief Read request body
 */
esp_err_t webserver_read_body(httpd_req_t *req, char *buffer, size_t max_len, size_t *actual_len)
{
    CHECK_NULL_PTR(req);
    CHECK_NULL_PTR(buffer);
    CHECK_NULL_PTR(actual_len);
    
    size_t content_len = req->content_len;
    if (content_len == 0) {
        *actual_len = 0;
        buffer[0] = '\0';
        return ESP_OK;
    }
    
    if (content_len >= max_len) {
        SENSDOT_LOGE(TAG, "Request body too large: %zu bytes", content_len);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    int ret = httpd_req_recv(req, buffer, content_len);
    if (ret <= 0) {
        SENSDOT_LOGE(TAG, "Failed to receive request body: %d", ret);
        return ESP_FAIL;
    }
    
    buffer[ret] = '\0';
    *actual_len = ret;
    
    return ESP_OK;
}

/**
 * @brief Parse JSON from request body
 */
esp_err_t webserver_parse_json_body(httpd_req_t *req, cJSON **json_obj)
{
    CHECK_NULL_PTR(req);
    CHECK_NULL_PTR(json_obj);
    
    char buffer[1024];
    size_t actual_len;
    
    esp_err_t ret = webserver_read_body(req, buffer, sizeof(buffer), &actual_len);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (actual_len == 0) {
        SENSDOT_LOGE(TAG, "Empty request body");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    *json_obj = cJSON_Parse(buffer);
    if (!*json_obj) {
        SENSDOT_LOGE(TAG, "Failed to parse JSON from request body");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Validate configuration from web request
 */
esp_err_t webserver_validate_config(cJSON *json_config, device_config_t *config)
{
    CHECK_NULL_PTR(json_config);
    CHECK_NULL_PTR(config);
    
    // Initialize with defaults
    config_init_defaults(config);
    
    // Extract WiFi SSID (required)
    cJSON *wifi_ssid = cJSON_GetObjectItem(json_config, "wifi_ssid");
    if (!cJSON_IsString(wifi_ssid) || strlen(wifi_ssid->valuestring) == 0) {
        SENSDOT_LOGE(TAG, "WiFi SSID is required");
        return SENSDOT_ERR_INVALID_ARG;
    }
    strncpy(config->wifi_ssid, wifi_ssid->valuestring, sizeof(config->wifi_ssid) - 1);
    
    // Extract WiFi password (optional)
    cJSON *wifi_pass = cJSON_GetObjectItem(json_config, "wifi_pass");
    if (cJSON_IsString(wifi_pass)) {
        strncpy(config->wifi_pass, wifi_pass->valuestring, sizeof(config->wifi_pass) - 1);
    }
    
    // Extract MQTT URI (required)
    cJSON *mqtt_uri = cJSON_GetObjectItem(json_config, "mqtt_uri");
    if (cJSON_IsString(mqtt_uri) && strlen(mqtt_uri->valuestring) > 0) {
        strncpy(config->mqtt_uri, mqtt_uri->valuestring, sizeof(config->mqtt_uri) - 1);
    }
    
    // Extract MQTT credentials (optional)
    cJSON *mqtt_user = cJSON_GetObjectItem(json_config, "mqtt_user");
    if (cJSON_IsString(mqtt_user)) {
        strncpy(config->mqtt_user, mqtt_user->valuestring, sizeof(config->mqtt_user) - 1);
    }
    
    cJSON *mqtt_pass = cJSON_GetObjectItem(json_config, "mqtt_pass");
    if (cJSON_IsString(mqtt_pass)) {
        strncpy(config->mqtt_pass, mqtt_pass->valuestring, sizeof(config->mqtt_pass) - 1);
    }
    
    // Extract MQTT prefix (optional)
    cJSON *mqtt_prefix = cJSON_GetObjectItem(json_config, "mqtt_prefix");
    if (cJSON_IsString(mqtt_prefix) && strlen(mqtt_prefix->valuestring) > 0) {
        strncpy(config->mqtt_prefix, mqtt_prefix->valuestring, sizeof(config->mqtt_prefix) - 1);
    }
    
    // Extract timing parameters (optional)
    cJSON *wake_interval = cJSON_GetObjectItem(json_config, "wake_interval");
    if (cJSON_IsNumber(wake_interval)) {
        config->wake_interval_sec = wake_interval->valueint;
    }
    
    cJSON *alarm_hold = cJSON_GetObjectItem(json_config, "alarm_hold");
    if (cJSON_IsNumber(alarm_hold)) {
        config->alarm_hold_sec = alarm_hold->valueint;
    }
    
    cJSON *retry_sleep = cJSON_GetObjectItem(json_config, "retry_sleep");
    if (cJSON_IsNumber(retry_sleep)) {
        config->retry_sleep_sec = retry_sleep->valueint;
    }
    
    // Extract battery threshold (optional)
    cJSON *low_batt_threshold = cJSON_GetObjectItem(json_config, "low_batt_threshold");
    if (cJSON_IsNumber(low_batt_threshold)) {
        config->low_batt_threshold = (float)low_batt_threshold->valuedouble;
    }
    
    // Mark as configured
    config->configured = true;
    
    // Validate the configuration
    return config_validate(config);
}

/**
 * @brief Generate WiFi scan results JSON
 */
esp_err_t webserver_generate_scan_json(const wifi_scan_result_t *results, size_t count, 
                                      char *json_str, size_t max_len)
{
    CHECK_NULL_PTR(results);
    CHECK_NULL_PTR(json_str);
    
    cJSON *root = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();
    
    for (size_t i = 0; i < count; i++) {
        cJSON *network = cJSON_CreateObject();
        cJSON_AddStringToObject(network, "ssid", results[i].ssid);
        cJSON_AddNumberToObject(network, "rssi", results[i].rssi);
        cJSON_AddBoolToObject(network, "auth_required", results[i].auth_required);
        
        // Add signal quality indicator
        const char *quality;
        if (results[i].rssi > -50) {
            quality = "excellent";
        } else if (results[i].rssi > -60) {
            quality = "good";
        } else if (results[i].rssi > -70) {
            quality = "fair";
        } else {
            quality = "poor";
        }
        cJSON_AddStringToObject(network, "quality", quality);
        
        cJSON_AddItemToArray(networks, network);
    }
    
    cJSON_AddItemToObject(root, "networks", networks);
    cJSON_AddNumberToObject(root, "count", count);
    
    char *json_string = cJSON_Print(root);
    if (!json_string) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    
    strncpy(json_str, json_string, max_len - 1);
    json_str[max_len - 1] = '\0';
    
    free(json_string);
    cJSON_Delete(root);
    
    return ESP_OK;
}

/**
 * @brief Generate device status JSON
 */
esp_err_t webserver_generate_status_json(char *status_json, size_t max_len)
{
    CHECK_NULL_PTR(status_json);
    
    cJSON *status = cJSON_CreateObject();
    
    // Device information
    cJSON_AddStringToObject(status, "device_name", "SensDot");
    cJSON_AddStringToObject(status, "version", SENSDOT_VERSION_STRING);
    cJSON_AddNumberToObject(status, "uptime", time(NULL) - g_start_time);
    
    // Configuration status
    const device_config_t *config = config_get_current();
    cJSON_AddBoolToObject(status, "configured", config->configured);
    
    // WiFi status
    cJSON *wifi_status = cJSON_CreateObject();
    wifi_state_t wifi_state = wifi_get_state();
    const char *wifi_state_str;
    switch (wifi_state) {
        case WIFI_STATE_CONNECTED: wifi_state_str = "connected"; break;
        case WIFI_STATE_CONNECTING: wifi_state_str = "connecting"; break;
        case WIFI_STATE_DISCONNECTED: wifi_state_str = "disconnected"; break;
        case WIFI_STATE_AP_MODE: wifi_state_str = "ap_mode"; break;
        default: wifi_state_str = "idle"; break;
    }
    cJSON_AddStringToObject(wifi_status, "state", wifi_state_str);
    
    if (wifi_is_connected()) {
        char ip_str[16];
        if (wifi_get_ip_address(ip_str, sizeof(ip_str)) == ESP_OK) {
            cJSON_AddStringToObject(wifi_status, "ip_address", ip_str);
        }
        cJSON_AddNumberToObject(wifi_status, "rssi", wifi_get_rssi());
    }
    cJSON_AddItemToObject(status, "wifi", wifi_status);
    
    // Web server statistics
    cJSON *server_stats = cJSON_CreateObject();
    cJSON_AddNumberToObject(server_stats, "requests", g_requests_count);
    cJSON_AddNumberToObject(server_stats, "errors", g_errors_count);
    cJSON_AddItemToObject(status, "server", server_stats);
    
    // Memory information
    cJSON *memory = cJSON_CreateObject();
    cJSON_AddNumberToObject(memory, "free_heap", esp_get_free_heap_size());
    cJSON_AddNumberToObject(memory, "min_free_heap", esp_get_minimum_free_heap_size());
    cJSON_AddItemToObject(status, "memory", memory);
    
    char *json_string = cJSON_Print(status);
    if (!json_string) {
        cJSON_Delete(status);
        return ESP_ERR_NO_MEM;
    }
    
    strncpy(status_json, json_string, max_len - 1);
    status_json[max_len - 1] = '\0';
    
    free(json_string);
    cJSON_Delete(status);
    
    return ESP_OK;
}

/**
 * @brief Get HTML content for setup page
 */
esp_err_t webserver_get_setup_html(char *html_content, size_t max_len)
{
    CHECK_NULL_PTR(html_content);
    
    const char *html_template = 
        "<!DOCTYPE html>"
        "<html><head>"
        "<title>SensDot Setup</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:20px;background:#f5f5f5}"
        ".container{max-width:600px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
        "h1{color:#333;text-align:center;margin-bottom:30px}"
        ".form-group{margin-bottom:20px}"
        "label{display:block;margin-bottom:5px;font-weight:bold;color:#555}"
        "input,select{width:100%%;padding:12px;border:1px solid #ddd;border-radius:6px;font-size:16px;box-sizing:border-box}"
        "input:focus,select:focus{outline:none;border-color:#007bff;box-shadow:0 0 0 3px rgba(0,123,255,0.1)}"
        "button{background:#007bff;color:white;padding:12px 24px;border:none;border-radius:6px;cursor:pointer;font-size:16px;width:100%%;margin-top:10px}"
        "button:hover{background:#0056b3}"
        "button:disabled{background:#ccc;cursor:not-allowed}"
        ".status{margin-top:20px;padding:15px;border-radius:6px;text-align:center}"
        ".success{background:#d4edda;color:#155724;border:1px solid #c3e6cb}"
        ".error{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}"
        ".loading{background:#cce7ff;color:#004085;border:1px solid #99ccff}"
        ".advanced{margin-top:20px;padding:15px;background:#f8f9fa;border-radius:6px}"
        ".advanced h3{margin-top:0;color:#495057}"
        ".help-text{font-size:14px;color:#6c757d;margin-top:5px}"
        "</style>"
        "</head><body>"
        "<div class='container'>"
        "<h1>🌐 SensDot Configuration</h1>"
        "<form id='configForm'>"
        
        "<div class='form-group'>"
        "<label for='wifi_ssid'>WiFi Network *</label>"
        "<select id='wifi_ssid' name='wifi_ssid' required>"
        "<option value=''>Select WiFi network...</option>"
        "</select>"
        "<div class='help-text'>Choose your WiFi network from the list</div>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='wifi_pass'>WiFi Password</label>"
        "<input type='password' id='wifi_pass' name='wifi_pass' placeholder='Enter WiFi password'>"
        "<div class='help-text'>Leave empty for open networks</div>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='mqtt_uri'>MQTT Broker URI *</label>"
        "<input type='text' id='mqtt_uri' name='mqtt_uri' value='mqtt://192.168.1.100:1883' required>"
        "<div class='help-text'>Example: mqtt://192.168.1.100:1883 or mqtts://broker.hivemq.com:8883</div>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='mqtt_user'>MQTT Username</label>"
        "<input type='text' id='mqtt_user' name='mqtt_user' placeholder='MQTT username (optional)'>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='mqtt_pass'>MQTT Password</label>"
        "<input type='password' id='mqtt_pass' name='mqtt_pass' placeholder='MQTT password (optional)'>"
        "</div>"
        
        "<div class='advanced'>"
        "<h3>⚙️ Advanced Settings</h3>"
        
        "<div class='form-group'>"
        "<label for='mqtt_prefix'>MQTT Topic Prefix</label>"
        "<input type='text' id='mqtt_prefix' name='mqtt_prefix' value='sensdot'>"
        "<div class='help-text'>Base topic for MQTT messages</div>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='wake_interval'>Wake Interval (seconds)</label>"
        "<input type='number' id='wake_interval' name='wake_interval' value='300' min='60' max='3600'>"
        "<div class='help-text'>How often the device wakes up (60-3600 seconds)</div>"
        "</div>"
        
        "<div class='form-group'>"
        "<label for='low_batt_threshold'>Low Battery Threshold (V)</label>"
        "<input type='number' id='low_batt_threshold' name='low_batt_threshold' value='3.3' min='3.0' max='4.2' step='0.1'>"
        "<div class='help-text'>Voltage threshold for low battery warning</div>"
        "</div>"
        
        "</div>"
        
        "<button type='button' onclick='saveConfig()'>💾 Save Configuration</button>"
        "<button type='button' onclick='testConnection()' style='background:#28a745;margin-top:10px'>🔧 Test Connection</button>"
        
        "</form>"
        
        "<div id='status'></div>"
        
        "</div>"
        
        "<script>"
        "let networks = [];"
        
        "function showStatus(message, type) {"
        "  const statusDiv = document.getElementById('status');"
        "  statusDiv.className = 'status ' + type;"
        "  statusDiv.innerHTML = message;"
        "}"
        
        "function loadNetworks() {"
        "  showStatus('🔍 Scanning WiFi networks...', 'loading');"
        "  fetch('/scan')"
        "    .then(r => r.json())"
        "    .then(data => {"
        "      networks = data.networks || [];"
        "      const select = document.getElementById('wifi_ssid');"
        "      select.innerHTML = '<option value=\"\">Select WiFi network...</option>';"
        "      networks.forEach(net => {"
        "        const option = document.createElement('option');"
        "        option.value = net.ssid;"
        "        option.textContent = `${net.ssid} (${net.quality}) ${net.auth_required ? '🔒' : '📶'}`;"
        "        select.appendChild(option);"
        "      });"
        "      showStatus(`✅ Found ${networks.length} networks`, 'success');"
        "    })"
        "    .catch(e => {"
        "      console.error('Scan error:', e);"
        "      showStatus('❌ Failed to scan networks: ' + e.message, 'error');"
        "    });"
        "}"
        
        "function saveConfig() {"
        "  const formData = new FormData(document.getElementById('configForm'));"
        "  const data = Object.fromEntries(formData);"
        "  "
        "  if (!data.wifi_ssid) {"
        "    showStatus('❌ Please select a WiFi network', 'error');"
        "    return;"
        "  }"
        "  "
        "  showStatus('💾 Saving configuration...', 'loading');"
        "  "
        "  fetch('/save', {"
        "    method: 'POST',"
        "    headers: {'Content-Type': 'application/json'},"
        "    body: JSON.stringify(data)"
        "  })"
        "  .then(r => r.json())"
        "  .then(result => {"
        "    if (result.success) {"
        "      showStatus('✅ Configuration saved! Device will restart in 5 seconds...', 'success');"
        "      setTimeout(() => {"
        "        window.location.href = '/restart';"
        "      }, 5000);"
        "    } else {"
        "      showStatus('❌ Error: ' + (result.error || 'Unknown error'), 'error');"
        "    }"
        "  })"
        "  .catch(e => {"
        "    showStatus('❌ Network error: ' + e.message, 'error');"
        "  });"
        "}"
        
        "function testConnection() {"
        "  showStatus('🔧 Testing connection...', 'loading');"
        "  fetch('/status')"
        "    .then(r => r.json())"
        "    .then(data => {"
        "      showStatus('✅ Connection test successful!', 'success');"
        "    })"
        "    .catch(e => {"
        "      showStatus('❌ Connection test failed: ' + e.message, 'error');"
        "    });"
        "}"
        
        "// Initialize page"
        "document.addEventListener('DOMContentLoaded', function() {"
        "  loadNetworks();"
        "});"
        
        "</script>"
        "</body></html>";
    
    int written = snprintf(html_content, max_len, "%s", html_template);
    if (written >= max_len) {
        SENSDOT_LOGE(TAG, "HTML content truncated");
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    return ESP_OK;
}

/**
 * @brief Get CSS styles for setup page
 */
esp_err_t webserver_get_setup_css(char *css_content, size_t max_len)
{
    // CSS is embedded in HTML for simplicity
    css_content[0] = '\0';
    return ESP_OK;
}

/**
 * @brief Get JavaScript for setup page
 */
esp_err_t webserver_get_setup_js(char *js_content, size_t max_len)
{
    // JavaScript is embedded in HTML for simplicity
    js_content[0] = '\0';
    return ESP_OK;
}

/**
 * @brief Set web server configuration
 */
esp_err_t webserver_set_config(size_t max_uri_handlers, size_t max_open_sockets, 
                              size_t stack_size, uint8_t task_priority)
{
    // Configuration would be applied on next start
    SENSDOT_LOGI(TAG, "Web server configuration updated");
    return ESP_OK;
}

/**
 * @brief Enable/disable CORS headers
 */
void webserver_set_cors_enabled(bool enable)
{
    g_cors_enabled = enable;
    SENSDOT_LOGI(TAG, "CORS %s", enable ? "enabled" : "disabled");
}

/**
 * @brief Set custom 404 error page
 */
esp_err_t webserver_set_404_page(const char *error_html)
{
    CHECK_NULL_PTR(error_html);
    
    strncpy(g_404_page, error_html, sizeof(g_404_page) - 1);
    g_404_page[sizeof(g_404_page) - 1] = '\0';
    
    return ESP_OK;
}

/**
 * @brief Get server statistics
 */
esp_err_t webserver_get_stats(uint32_t *requests_count, uint32_t *errors_count, uint32_t *uptime_sec)
{
    if (requests_count) *requests_count = g_requests_count;
    if (errors_count) *errors_count = g_errors_count;
    if (uptime_sec) *uptime_sec = (uint32_t)(time(NULL) - g_start_time);
    
    return ESP_OK;
}

/**
 * @brief Reset server statistics
 */
void webserver_reset_stats(void)
{
    g_requests_count = 0;
    g_errors_count = 0;
    g_start_time = time(NULL);
    
    SENSDOT_LOGI(TAG, "Web server statistics reset");
}