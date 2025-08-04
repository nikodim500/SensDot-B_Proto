/**
 * @file web_server.h
 * @brief HTTP web server for device configuration
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include "sensdot_common.h"
#include "esp_http_server.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Web server configuration
#define WEBSERVER_MAX_URI_LEN 512
#define WEBSERVER_MAX_HEADER_LEN 1024
#define WEBSERVER_STACK_SIZE 8192
#define WEBSERVER_TASK_PRIORITY 5

// HTTP response codes
typedef enum {
    HTTP_200_OK = 200,
    HTTP_400_BAD_REQUEST = 400,
    HTTP_401_UNAUTHORIZED = 401,
    HTTP_404_NOT_FOUND = 404,
    HTTP_500_INTERNAL_ERROR = 500
} http_status_code_t;

// Web server state
typedef enum {
    WEBSERVER_STATE_STOPPED,
    WEBSERVER_STATE_STARTING,
    WEBSERVER_STATE_RUNNING,
    WEBSERVER_STATE_ERROR
} webserver_state_t;

// Request context for handlers
typedef struct {
    httpd_req_t *req;
    void *user_data;
    char *body;
    size_t body_len;
} webserver_request_t;

/**
 * @brief Initialize web server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_init(void);

/**
 * @brief Start web server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_start(void);

/**
 * @brief Stop web server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_stop(void);

/**
 * @brief Deinitialize web server
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_deinit(void);

/**
 * @brief Get web server state
 * @return Current web server state
 */
webserver_state_t webserver_get_state(void);

/**
 * @brief Check if web server is running
 * @return true if running, false otherwise
 */
bool webserver_is_running(void);

/**
 * @brief Set setup mode timeout callback
 * @param callback Callback function to call when setup times out
 * @param user_data User data to pass to callback
 */
typedef void (*webserver_timeout_callback_t)(void *user_data);
void webserver_set_timeout_callback(webserver_timeout_callback_t callback, void *user_data);

/**
 * @brief Handle setup page request (GET /)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_setup_page(httpd_req_t *req);

/**
 * @brief Handle WiFi scan request (GET /scan)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_wifi_scan(httpd_req_t *req);

/**
 * @brief Handle configuration save request (POST /save)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_config_save(httpd_req_t *req);

/**
 * @brief Handle device restart request (GET /restart)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_restart(httpd_req_t *req);

/**
 * @brief Handle device status request (GET /status)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_status(httpd_req_t *req);

/**
 * @brief Handle factory reset request (POST /factory_reset)
 * @param req HTTP request
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_handle_factory_reset(httpd_req_t *req);

/**
 * @brief Send HTTP response with status code and content
 * @param req HTTP request
 * @param status_code HTTP status code
 * @param content_type Content type header
 * @param content Response content
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_send_response(httpd_req_t *req, http_status_code_t status_code, 
                                 const char *content_type, const char *content);

/**
 * @brief Send JSON response
 * @param req HTTP request
 * @param status_code HTTP status code
 * @param json_obj JSON object to send
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_send_json_response(httpd_req_t *req, http_status_code_t status_code, cJSON *json_obj);

/**
 * @brief Send error response
 * @param req HTTP request
 * @param status_code HTTP status code
 * @param error_message Error message
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_send_error(httpd_req_t *req, http_status_code_t status_code, const char *error_message);

/**
 * @brief Read request body
 * @param req HTTP request
 * @param buffer Buffer to store body content
 * @param max_len Maximum length of buffer
 * @param actual_len Pointer to store actual length read
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_read_body(httpd_req_t *req, char *buffer, size_t max_len, size_t *actual_len);

/**
 * @brief Parse JSON from request body
 * @param req HTTP request
 * @param json_obj Pointer to store parsed JSON object
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_parse_json_body(httpd_req_t *req, cJSON **json_obj);

/**
 * @brief Validate configuration from web request
 * @param json_config JSON configuration object
 * @param config Pointer to store validated configuration
 * @return ESP_OK if valid, error code otherwise
 */
esp_err_t webserver_validate_config(cJSON *json_config, device_config_t *config);

/**
 * @brief Generate WiFi scan results JSON
 * @param results Array of scan results
 * @param count Number of results
 * @param json_str Buffer to store JSON string
 * @param max_len Maximum length of JSON string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_generate_scan_json(const wifi_scan_result_t *results, size_t count, 
                                      char *json_str, size_t max_len);

/**
 * @brief Generate device status JSON
 * @param status_json Buffer to store status JSON
 * @param max_len Maximum length of status JSON
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_generate_status_json(char *status_json, size_t max_len);

/**
 * @brief Get HTML content for setup page
 * @param html_content Buffer to store HTML content
 * @param max_len Maximum length of HTML content
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_get_setup_html(char *html_content, size_t max_len);

/**
 * @brief Get CSS styles for setup page
 * @param css_content Buffer to store CSS content
 * @param max_len Maximum length of CSS content
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_get_setup_css(char *css_content, size_t max_len);

/**
 * @brief Get JavaScript for setup page
 * @param js_content Buffer to store JavaScript content
 * @param max_len Maximum length of JavaScript content
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_get_setup_js(char *js_content, size_t max_len);

/**
 * @brief Set web server configuration
 * @param max_uri_handlers Maximum number of URI handlers
 * @param max_open_sockets Maximum number of open sockets
 * @param stack_size Task stack size
 * @param task_priority Task priority
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_set_config(size_t max_uri_handlers, size_t max_open_sockets, 
                              size_t stack_size, uint8_t task_priority);

/**
 * @brief Enable/disable CORS headers
 * @param enable true to enable CORS, false to disable
 */
void webserver_set_cors_enabled(bool enable);

/**
 * @brief Set custom 404 error page
 * @param error_html HTML content for 404 page
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_set_404_page(const char *error_html);

/**
 * @brief Get server statistics
 * @param requests_count Pointer to store request count
 * @param errors_count Pointer to store error count
 * @param uptime_sec Pointer to store uptime in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t webserver_get_stats(uint32_t *requests_count, uint32_t *errors_count, uint32_t *uptime_sec);

/**
 * @brief Reset server statistics
 */
void webserver_reset_stats(void);

#ifdef __cplusplus
}
#endif

#endif // WEB_SERVER_H