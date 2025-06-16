// main.c — logic for sleep, wakeup, sensor reading, and MQTT reporting for ESP32 (ESP-32S)
// Added: First-time setup with AP mode and web configuration interface

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "mqtt_client.h"
#include "i2cdev.h"
#include "bh1750.h"
#include "bme280.h"
#include "cJSON.h"

// GPIO assignments
#define PIR_GPIO 4
#define LED_GPIO 2
#define BUZZER_GPIO 17
#define SENSOR_PWR_GPIO 18
#define BATTERY_ADC_GPIO 34
#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define RESET_BUTTON_GPIO 0  // Boot button for factory reset

#define TAG "sensdot"

// Configuration storage keys
#define NVS_NAMESPACE "sensdot_cfg"
#define NVS_WIFI_SSID "wifi_ssid"
#define NVS_WIFI_PASS "wifi_pass"
#define NVS_MQTT_URI "mqtt_uri"
#define NVS_MQTT_USER "mqtt_user"
#define NVS_MQTT_PASS "mqtt_pass"
#define NVS_MQTT_PREFIX "mqtt_prefix"
#define NVS_WAKE_INTERVAL "wake_interval"
#define NVS_ALARM_HOLD "alarm_hold"
#define NVS_LOW_BATT_THRESH "low_batt_thresh"
#define NVS_CONFIGURED "configured"

// Default values from Kconfig
#define DEFAULT_WAKE_INTERVAL_SEC CONFIG_SENS_WAKE_INTERVAL_SEC
#define DEFAULT_ALARM_HOLD_SEC CONFIG_SENS_ALARM_HOLD_SEC  
#define DEFAULT_LOW_BATT_THRESHOLD (CONFIG_SENS_LOW_BATT_THRESHOLD / 1000.0f)  // Convert mV to V
#define BATT_DIVIDER_RATIO (CONFIG_SENS_VOLTAGE_DIVIDER_RATIO / 100.0f)       // Convert from x100
#define SETUP_TIMEOUT_MS (CONFIG_SENS_SETUP_TIMEOUT_MIN * 60 * 1000)         // Convert min to ms
#define MAX_WIFI_RETRIES CONFIG_SENS_MAX_WIFI_RETRIES
#define MQTT_DISCOVERY_INTERVAL CONFIG_SENS_MQTT_DISCOVERY_INTERVAL

// Setup mode constants
#define SETUP_AP_SSID_PREFIX "SensDot-Setup-"
#define SETUP_AP_PASSWORD CONFIG_SENS_SETUP_AP_PASSWORD
#define SETUP_AP_MAX_CONNECTIONS 1

// WiFi event bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

// Alarm state stored in RTC memory across deep sleep cycles
RTC_DATA_ATTR static time_t last_alarm_time = 0;
RTC_DATA_ATTR static int boot_count = 0;

// Global variables
static EventGroupHandle_t wifi_event_group;
static esp_mqtt_client_handle_t mqtt_client = NULL;
static httpd_handle_t setup_server = NULL;
static bool setup_mode = false;
static int wifi_retry_count = 0;

// Configuration structure
typedef struct {
    char wifi_ssid[32];
    char wifi_pass[64];
    char mqtt_uri[128];
    char mqtt_user[32];
    char mqtt_pass[64];
    char mqtt_prefix[32];
    int wake_interval_sec;
    int alarm_hold_sec;
    float low_batt_threshold;
    bool configured;
} device_config_t;

static device_config_t config = {};

// Function prototypes
static void wifi_init_sta(void);
static void wifi_init_ap(void);
static void setup_web_server(void);
static void load_config(void);
static void save_config(void);
static void factory_reset(void);
static void indicate_status(int blink_count, bool with_sound);
static void enter_setup_mode(void);
static void normal_operation(void);

// LED indication patterns
static void indicate_status(int blink_count, bool with_sound) {
    for (int i = 0; i < blink_count; i++) {
        gpio_set_level(LED_GPIO, 1);
        if (with_sound) {
            gpio_set_level(BUZZER_GPIO, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);
        gpio_set_level(BUZZER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Initialize GPIO pins
static void gpio_init(void) {
    // Configure LED pin
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_config);

    // Configure buzzer pin
    gpio_config_t buzzer_config = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&buzzer_config);

    // Configure sensor power pin
    gpio_config_t sensor_pwr_config = {
        .pin_bit_mask = (1ULL << SENSOR_PWR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&sensor_pwr_config);

    // Configure PIR pin
    gpio_config_t pir_config = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pir_config);

    // Configure reset button pin
    gpio_config_t reset_config = {
        .pin_bit_mask = (1ULL << RESET_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&reset_config);

    // Initialize pins to safe state
    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_level(SENSOR_PWR_GPIO, 0);
}

// Load configuration from NVS
static void load_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Set defaults first
    strcpy(config.wifi_ssid, "");
    strcpy(config.wifi_pass, "");
    strcpy(config.mqtt_uri, "mqtt://192.168.1.100:1883");
    strcpy(config.mqtt_user, "");
    strcpy(config.mqtt_pass, "");
    strcpy(config.mqtt_prefix, "sensdot");
    config.wake_interval_sec = DEFAULT_WAKE_INTERVAL_SEC;
    config.alarm_hold_sec = DEFAULT_ALARM_HOLD_SEC;
    config.low_batt_threshold = DEFAULT_LOW_BATT_THRESHOLD;
    config.configured = false;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS not found, using defaults");
        return;
    }

    size_t required_size = 0;
    
    // Load WiFi SSID
    required_size = sizeof(config.wifi_ssid);
    nvs_get_str(nvs_handle, NVS_WIFI_SSID, config.wifi_ssid, &required_size);
    
    // Load WiFi password
    required_size = sizeof(config.wifi_pass);
    nvs_get_str(nvs_handle, NVS_WIFI_PASS, config.wifi_pass, &required_size);
    
    // Load MQTT URI
    required_size = sizeof(config.mqtt_uri);
    nvs_get_str(nvs_handle, NVS_MQTT_URI, config.mqtt_uri, &required_size);
    
    // Load MQTT credentials
    required_size = sizeof(config.mqtt_user);
    nvs_get_str(nvs_handle, NVS_MQTT_USER, config.mqtt_user, &required_size);
    required_size = sizeof(config.mqtt_pass);
    nvs_get_str(nvs_handle, NVS_MQTT_PASS, config.mqtt_pass, &required_size);
    
    // Load MQTT prefix
    required_size = sizeof(config.mqtt_prefix);
    nvs_get_str(nvs_handle, NVS_MQTT_PREFIX, config.mqtt_prefix, &required_size);
    
    // Load numeric values
    nvs_get_i32(nvs_handle, NVS_WAKE_INTERVAL, &config.wake_interval_sec);
    nvs_get_i32(nvs_handle, NVS_ALARM_HOLD, &config.alarm_hold_sec);
    
    // Load float value (stored as blob)
    required_size = sizeof(config.low_batt_threshold);
    nvs_get_blob(nvs_handle, NVS_LOW_BATT_THRESH, &config.low_batt_threshold, &required_size);
    
    // Load configured flag
    uint8_t configured = 0;
    nvs_get_u8(nvs_handle, NVS_CONFIGURED, &configured);
    config.configured = (configured == 1);

    nvs_close(nvs_handle);
    
    ESP_LOGI(TAG, "Configuration loaded: SSID=%s, Configured=%d", config.wifi_ssid, config.configured);
}

// Save configuration to NVS
static void save_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    // Save all configuration values
    nvs_set_str(nvs_handle, NVS_WIFI_SSID, config.wifi_ssid);
    nvs_set_str(nvs_handle, NVS_WIFI_PASS, config.wifi_pass);
    nvs_set_str(nvs_handle, NVS_MQTT_URI, config.mqtt_uri);
    nvs_set_str(nvs_handle, NVS_MQTT_USER, config.mqtt_user);
    nvs_set_str(nvs_handle, NVS_MQTT_PASS, config.mqtt_pass);
    nvs_set_str(nvs_handle, NVS_MQTT_PREFIX, config.mqtt_prefix);
    nvs_set_i32(nvs_handle, NVS_WAKE_INTERVAL, config.wake_interval_sec);
    nvs_set_i32(nvs_handle, NVS_ALARM_HOLD, config.alarm_hold_sec);
    nvs_set_blob(nvs_handle, NVS_LOW_BATT_THRESH, &config.low_batt_threshold, sizeof(config.low_batt_threshold));
    nvs_set_u8(nvs_handle, NVS_CONFIGURED, config.configured ? 1 : 0);

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Configuration saved");
}

// Factory reset - clear all configuration
static void factory_reset(void) {
    ESP_LOGI(TAG, "Factory reset initiated");
    
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    
    // Indicate factory reset with 5 blinks
    indicate_status(5, false);
    
    ESP_LOGI(TAG, "Factory reset completed, restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

// Measure battery voltage via ADC
static float read_battery_voltage(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // GPIO34 = ADC1_CH6
    int raw = adc1_get_raw(ADC1_CHANNEL_6);
    float voltage = ((float)raw / 4095.0f) * 3.3f * BATT_DIVIDER_RATIO;
    return voltage;
}

// WiFi event handler
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (wifi_retry_count < 3) {
            esp_wifi_connect();
            wifi_retry_count++;
            ESP_LOGI(TAG, "Retry to connect to the AP");
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to the AP failed");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize WiFi in Station mode
static void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    strcpy((char *)wifi_config.sta.ssid, config.wifi_ssid);
    strcpy((char *)wifi_config.sta.password, config.wifi_pass);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi init finished, connecting to %s", config.wifi_ssid);
}

// Initialize WiFi in AP mode for setup
static void wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Get MAC address for unique AP name
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    
    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "%s%02X%02X%02X", 
             SETUP_AP_SSID_PREFIX, mac[3], mac[4], mac[5]);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ap_ssid),
            .channel = 1,
            .password = SETUP_AP_PASSWORD,
            .max_connection = SETUP_AP_MAX_CONNECTIONS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    strcpy((char *)wifi_config.ap.ssid, ap_ssid);

    // Open network if no password set
    if (strlen(SETUP_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started: %s", ap_ssid);
}

// HTTP GET handler for main setup page
static esp_err_t setup_get_handler(httpd_req_t *req) {
    const char* html_page = 
        "<!DOCTYPE html>"
        "<html><head><title>SensDot Setup</title>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f0f0f0}"
        ".container{max-width:500px;margin:0 auto;background:white;padding:20px;border-radius:10px}"
        "h1{color:#333;text-align:center}"
        "label{display:block;margin-top:10px;font-weight:bold}"
        "input,select{width:100%;padding:8px;margin-top:5px;border:1px solid #ddd;border-radius:4px}"
        "button{background:#007bff;color:white;padding:10px 20px;border:none;border-radius:4px;cursor:pointer;margin-top:15px}"
        "button:hover{background:#0056b3}"
        ".status{margin-top:15px;padding:10px;border-radius:4px}"
        ".success{background:#d4edda;color:#155724;border:1px solid #c3e6cb}"
        ".error{background:#f8d7da;color:#721c24;border:1px solid #f5c6cb}"
        "</style></head><body>"
        "<div class='container'>"
        "<h1>SensDot Configuration</h1>"
        "<form id='configForm'>"
        "<label>WiFi Network:</label>"
        "<select id='wifi_ssid' name='wifi_ssid' onchange='updatePassword()'>"
        "<option value=''>Select network...</option>"
        "</select>"
        "<label>WiFi Password:</label>"
        "<input type='password' id='wifi_pass' name='wifi_pass'>"
        "<label>MQTT Broker URI:</label>"
        "<input type='text' id='mqtt_uri' name='mqtt_uri' value='mqtt://192.168.1.100:1883'>"
        "<label>MQTT Username:</label>"
        "<input type='text' id='mqtt_user' name='mqtt_user'>"
        "<label>MQTT Password:</label>"
        "<input type='password' id='mqtt_pass' name='mqtt_pass'>"
        "<label>Topic Prefix:</label>"
        "<input type='text' id='mqtt_prefix' name='mqtt_prefix' value='sensdot'>"
        "<label>Wake Interval (seconds):</label>"
        "<input type='number' id='wake_interval' name='wake_interval' value='300' min='60' max='3600'>"
        "<label>Low Battery Threshold (V):</label>"
        "<input type='number' id='low_batt' name='low_batt' value='3.3' min='3.0' max='4.2' step='0.1'>"
        "<button type='button' onclick='saveConfig()'>Save Configuration</button>"
        "</form>"
        "<div id='status'></div>"
        "</div>"
        "<script>"
        "function loadNetworks(){"
        "fetch('/scan').then(r=>r.json()).then(data=>{"
        "const select=document.getElementById('wifi_ssid');"
        "data.networks.forEach(net=>{"
        "const option=document.createElement('option');"
        "option.value=net.ssid;option.textContent=net.ssid+(net.auth?' 🔒':'');"
        "select.appendChild(option);"
        "});"
        "}).catch(e=>console.error('Error loading networks:',e));"
        "}"
        "function updatePassword(){"
        "const ssid=document.getElementById('wifi_ssid').value;"
        "document.getElementById('wifi_pass').disabled=!ssid;"
        "}"
        "function saveConfig(){"
        "const formData=new FormData(document.getElementById('configForm'));"
        "const data=Object.fromEntries(formData);"
        "document.getElementById('status').innerHTML='<div class=\"status\">Saving configuration...</div>';"
        "fetch('/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})"
        ".then(r=>r.json()).then(result=>{"
        "if(result.success){"
        "document.getElementById('status').innerHTML='<div class=\"status success\">Configuration saved! Device will restart in 5 seconds.</div>';"
        "setTimeout(()=>{window.location.href='/restart';},5000);"
        "}else{"
        "document.getElementById('status').innerHTML='<div class=\"status error\">Error: '+result.error+'</div>';"
        "}"
        "}).catch(e=>{"
        "document.getElementById('status').innerHTML='<div class=\"status error\">Network error: '+e.message+'</div>';"
        "});"
        "}"
        "loadNetworks();"
        "</script></body></html>";

    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP GET handler for WiFi scan
static esp_err_t scan_get_handler(httpd_req_t *req) {
    // Perform WiFi scan
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    esp_wifi_scan_start(&scan_config, true);
    
    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    
    wifi_ap_record_t *ap_info = malloc(sizeof(wifi_ap_record_t) * ap_count);
    esp_wifi_scan_get_ap_records(&ap_count, ap_info);

    // Create JSON response
    cJSON *root = cJSON_CreateObject();
    cJSON *networks = cJSON_CreateArray();
    
    for (int i = 0; i < ap_count; i++) {
        cJSON *network = cJSON_CreateObject();
        cJSON_AddStringToObject(network, "ssid", (char *)ap_info[i].ssid);
        cJSON_AddBoolToObject(network, "auth", ap_info[i].authmode != WIFI_AUTH_OPEN);
        cJSON_AddNumberToObject(network, "rssi", ap_info[i].rssi);
        cJSON_AddItemToArray(networks, network);
    }
    
    cJSON_AddItemToObject(root, "networks", networks);
    char *json_string = cJSON_Print(root);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_string, strlen(json_string));
    
    free(json_string);
    cJSON_Delete(root);
    free(ap_info);
    
    return ESP_OK;
}

// HTTP POST handler for saving configuration
static esp_err_t save_post_handler(httpd_req_t *req) {
    char buf[1024];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (json == NULL) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Parse and validate configuration
    cJSON *wifi_ssid = cJSON_GetObjectItem(json, "wifi_ssid");
    cJSON *wifi_pass = cJSON_GetObjectItem(json, "wifi_pass");
    cJSON *mqtt_uri = cJSON_GetObjectItem(json, "mqtt_uri");
    cJSON *mqtt_user = cJSON_GetObjectItem(json, "mqtt_user");
    cJSON *mqtt_pass = cJSON_GetObjectItem(json, "mqtt_pass");
    cJSON *mqtt_prefix = cJSON_GetObjectItem(json, "mqtt_prefix");
    cJSON *wake_interval = cJSON_GetObjectItem(json, "wake_interval");
    cJSON *low_batt = cJSON_GetObjectItem(json, "low_batt");

    bool valid = true;
    cJSON *response = cJSON_CreateObject();

    if (!cJSON_IsString(wifi_ssid) || strlen(wifi_ssid->valuestring) == 0) {
        valid = false;
        cJSON_AddStringToObject(response, "error", "WiFi SSID is required");
    } else {
        // Update configuration
        strncpy(config.wifi_ssid, wifi_ssid->valuestring, sizeof(config.wifi_ssid) - 1);
        strncpy(config.wifi_pass, cJSON_IsString(wifi_pass) ? wifi_pass->valuestring : "", sizeof(config.wifi_pass) - 1);
        strncpy(config.mqtt_uri, cJSON_IsString(mqtt_uri) ? mqtt_uri->valuestring : "mqtt://192.168.1.100:1883", sizeof(config.mqtt_uri) - 1);
        strncpy(config.mqtt_user, cJSON_IsString(mqtt_user) ? mqtt_user->valuestring : "", sizeof(config.mqtt_user) - 1);
        strncpy(config.mqtt_pass, cJSON_IsString(mqtt_pass) ? mqtt_pass->valuestring : "", sizeof(config.mqtt_pass) - 1);
        strncpy(config.mqtt_prefix, cJSON_IsString(mqtt_prefix) ? mqtt_prefix->valuestring : "sensdot", sizeof(config.mqtt_prefix) - 1);
        config.wake_interval_sec = cJSON_IsNumber(wake_interval) ? wake_interval->valueint : DEFAULT_WAKE_INTERVAL_SEC;
        config.low_batt_threshold = cJSON_IsNumber(low_batt) ? (float)low_batt->valuedouble : DEFAULT_LOW_BATT_THRESHOLD;
        config.configured = true;

        // Save configuration
        save_config();
        cJSON_AddBoolToObject(response, "success", true);
        
        // Indicate successful configuration
        indicate_status(3, true);
    }

    if (!valid) {
        cJSON_AddBoolToObject(response, "success", false);
    }

    char *json_response = cJSON_Print(response);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, json_response, strlen(json_response));

    free(json_response);
    cJSON_Delete(response);
    cJSON_Delete(json);

    return ESP_OK;
}

// HTTP GET handler for restart
static esp_err_t restart_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, "Restarting device...", HTTPD_RESP_USE_STRLEN);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

// Start web server for configuration
static void setup_web_server(void) {
    httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
    server_config.lru_purge_enable = true;

    if (httpd_start(&setup_server, &server_config) == ESP_OK) {
        // Setup page
        httpd_uri_t setup_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = setup_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(setup_server, &setup_uri);

        // WiFi scan endpoint
        httpd_uri_t scan_uri = {
            .uri = "/scan",
            .method = HTTP_GET,
            .handler = scan_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(setup_server, &scan_uri);

        // Save configuration endpoint
        httpd_uri_t save_uri = {
            .uri = "/save",
            .method = HTTP_POST,
            .handler = save_post_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(setup_server, &save_uri);

        // Restart endpoint
        httpd_uri_t restart_uri = {
            .uri = "/restart",
            .method = HTTP_GET,
            .handler = restart_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(setup_server, &restart_uri);

        ESP_LOGI(TAG, "Setup web server started");
    } else {
        ESP_LOGE(TAG, "Error starting setup web server");
    }
}

// Enter setup mode with AP and web server
static void enter_setup_mode(void) {
    ESP_LOGI(TAG, "Entering setup mode");
    setup_mode = true;
    
    // Turn on LED to indicate setup mode
    gpio_set_level(LED_GPIO, 1);
    
    // Initialize WiFi in AP mode
    wifi_init_ap();
    
    // Start web server
    setup_web_server();
    
    ESP_LOGI(TAG, "Setup mode active - connect to SensDot-Setup-XXXXXX");
    ESP_LOGI(TAG, "Open http://192.168.4.1 to configure device");
    
    // Wait in setup mode with timeout
    int timeout_ms = SETUP_TIMEOUT_MS;
    while (setup_mode && timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        timeout_ms -= 1000;
        
        // Blink LED every 5 seconds to show we're alive
        if (timeout_ms % 5000 == 0) {
            gpio_set_level(LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_GPIO, 1);
        }
    }
    
    if (timeout_ms <= 0) {
        ESP_LOGI(TAG, "Setup timeout, entering deep sleep");
        gpio_set_level(LED_GPIO, 0);
        esp_sleep_enable_timer_wakeup(60 * 1000000ULL); // Wake up in 1 minute
        esp_deep_sleep_start();
    }
}

// MQTT event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Error");
            break;
        default:
            break;
    }
}

// Initialize MQTT client
static bool mqtt_init(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = config.mqtt_uri,
    };
    
    // Add credentials if provided
    if (strlen(config.mqtt_user) > 0) {
        mqtt_cfg.credentials.username = config.mqtt_user;
        mqtt_cfg.credentials.password = config.mqtt_pass;
    }
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return false;
    }
    
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(mqtt_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start MQTT client");
        return false;
    }
    
    // Wait for connection
    vTaskDelay(pdMS_TO_TICKS(2000));
    return true;
}

// Publish to MQTT topic
static void mqtt_publish(const char *topic, const char *value) {
    if (mqtt_client) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, topic, value, 0, 1, 0);
        ESP_LOGI(TAG, "Published to %s: %s (msg_id=%d)", topic, value, msg_id);
    }
}

// Publish to topic relative to base prefix
static void mqtt_publish_rel(const char *topic_suffix, const char *value) {
    char full_topic[128];
    snprintf(full_topic, sizeof(full_topic), "%s/%s", config.mqtt_prefix, topic_suffix);
    mqtt_publish(full_topic, value);
}

// Register MQTT discovery configurations for Home Assistant
static void mqtt_discovery(void) {
    char device_id[32];
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(device_id, sizeof(device_id), "sensdot_%02x%02x%02x", mac[3], mac[4], mac[5]);
    
    // Temperature sensor
    char config_topic[128];
    char config_payload[512];
    
    snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s_temp/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Temperature\",\"state_topic\":\"%s/temp\",\"unit_of_measurement\":\"°C\",\"device_class\":\"temperature\",\"unique_id\":\"%s_temp\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Humidity sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s_hum/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Humidity\",\"state_topic\":\"%s/hum\",\"unit_of_measurement\":\"%%\",\"device_class\":\"humidity\",\"unique_id\":\"%s_hum\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Pressure sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s_press/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Pressure\",\"state_topic\":\"%s/press\",\"unit_of_measurement\":\"hPa\",\"device_class\":\"pressure\",\"unique_id\":\"%s_press\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Light sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s_light/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Light\",\"state_topic\":\"%s/light\",\"unit_of_measurement\":\"lx\",\"device_class\":\"illuminance\",\"unique_id\":\"%s_light\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Motion sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/binary_sensor/%s_motion/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Motion\",\"state_topic\":\"%s/alarm\",\"device_class\":\"motion\",\"payload_on\":\"on\",\"payload_off\":\"off\",\"unique_id\":\"%s_motion\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Battery voltage sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/sensor/%s_battery/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Battery\",\"state_topic\":\"%s/battery\",\"unit_of_measurement\":\"V\",\"device_class\":\"voltage\",\"unique_id\":\"%s_battery\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
    
    // Low battery sensor
    snprintf(config_topic, sizeof(config_topic), "homeassistant/binary_sensor/%s_low_batt/config", device_id);
    snprintf(config_payload, sizeof(config_payload), 
        "{\"name\":\"%s Low Battery\",\"state_topic\":\"%s/low_batt\",\"device_class\":\"battery\",\"payload_on\":\"on\",\"payload_off\":\"off\",\"unique_id\":\"%s_low_batt\"}", 
        device_id, config.mqtt_prefix, device_id);
    mqtt_publish(config_topic, config_payload);
}

// Read sensors and publish data
static void read_sensors_and_publish(void) {
    char payload[64];
    
    // Power on sensors
    gpio_set_level(SENSOR_PWR_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100)); // Let sensors stabilize
    
    // Initialize I2C
    ESP_ERROR_CHECK(i2cdev_init());
    
    // Initialize and read BME280
    bme280_t bme280_dev;
    memset(&bme280_dev, 0, sizeof(bme280_t));
    
    ESP_ERROR_CHECK(bme280_init_desc(&bme280_dev, BME280_I2C_ADDR_PRIM, 0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(bme280_init(&bme280_dev, NULL));
    
    float temperature, pressure, humidity;
    esp_err_t res = bme280_read_float(&bme280_dev, &temperature, &pressure, &humidity);
    if (res == ESP_OK) {
        snprintf(payload, sizeof(payload), "%.2f", temperature);
        mqtt_publish_rel("temp", payload);
        
        snprintf(payload, sizeof(payload), "%.2f", humidity);
        mqtt_publish_rel("hum", payload);
        
        snprintf(payload, sizeof(payload), "%.2f", pressure / 100.0f); // Convert Pa to hPa
        mqtt_publish_rel("press", payload);
        
        ESP_LOGI(TAG, "BME280: T=%.2f°C, H=%.2f%%, P=%.2fhPa", temperature, humidity, pressure / 100.0f);
    } else {
        ESP_LOGE(TAG, "Failed to read BME280: %s", esp_err_to_name(res));
    }
    
    // Initialize and read BH1750
    bh1750_t bh1750_dev;
    memset(&bh1750_dev, 0, sizeof(bh1750_t));
    
    ESP_ERROR_CHECK(bh1750_init_desc(&bh1750_dev, BH1750_ADDR_LO, 0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    ESP_ERROR_CHECK(bh1750_setup(&bh1750_dev, BH1750_MODE_CONTINUOUS, BH1750_RES_HIGH));
    
    uint16_t lux;
    res = bh1750_read(&bh1750_dev, &lux);
    if (res == ESP_OK) {
        snprintf(payload, sizeof(payload), "%u", lux);
        mqtt_publish_rel("light", payload);
        ESP_LOGI(TAG, "BH1750: %u lx", lux);
    } else {
        ESP_LOGE(TAG, "Failed to read BH1750: %s", esp_err_to_name(res));
    }
    
    // Power off sensors
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    
    // Check PIR sensor
    bool pir_state = gpio_get_level(PIR_GPIO);
    time_t now = time(NULL);
    
    if (pir_state && (now - last_alarm_time) > config.alarm_hold_sec) {
        last_alarm_time = now;
        mqtt_publish_rel("alarm", "on");
        ESP_LOGI(TAG, "Motion detected!");
    } else if (!pir_state) {
        mqtt_publish_rel("alarm", "off");
    }
    
    // Check battery voltage
    float battery_voltage = read_battery_voltage();
    snprintf(payload, sizeof(payload), "%.2f", battery_voltage);
    mqtt_publish_rel("battery", payload);
    
    if (battery_voltage < config.low_batt_threshold) {
        mqtt_publish_rel("low_batt", "on");
        indicate_status(3, true); // Blink and beep for low battery
        ESP_LOGW(TAG, "Low battery: %.2fV", battery_voltage);
    } else {
        mqtt_publish_rel("low_batt", "off");
    }
    
    ESP_LOGI(TAG, "Battery: %.2fV", battery_voltage);
    
    // Single blink to indicate successful data transmission
    indicate_status(1, false);
}

// Normal operation mode
static void normal_operation(void) {
    ESP_LOGI(TAG, "Starting normal operation");
    
    // Initialize WiFi and connect
    wifi_init_sta();
    
    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(10000));
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to WiFi");
        
        // Initialize MQTT
        if (mqtt_init()) {
            // Register with Home Assistant (only on first boot or periodically)
            if (boot_count % MQTT_DISCOVERY_INTERVAL == 0) { // Configurable interval
                mqtt_discovery();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
            
            // Read sensors and publish data
            read_sensors_and_publish();
            
            // Clean shutdown
            esp_mqtt_client_stop(mqtt_client);
            esp_mqtt_client_destroy(mqtt_client);
        } else {
            ESP_LOGE(TAG, "Failed to initialize MQTT");
        }
        
        esp_wifi_stop();
    } else {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        // If we can't connect multiple times, enter setup mode
        static int wifi_fail_count = 0;
        wifi_fail_count++;
        if (wifi_fail_count >= MAX_WIFI_RETRIES) {
            ESP_LOGW(TAG, "Multiple WiFi failures, entering setup mode");
            enter_setup_mode();
            return;
        }
    }
    
    // Prepare for deep sleep
    ESP_LOGI(TAG, "Entering deep sleep for %d seconds", config.wake_interval_sec);
    
    // Configure wake up sources
    esp_sleep_enable_timer_wakeup(config.wake_interval_sec * 1000000ULL);
    esp_sleep_enable_ext0_wakeup(PIR_GPIO, 1); // Wake on PIR motion
    
    // Configure power domains for minimum power consumption
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
    
    boot_count++;
    esp_deep_sleep_start();
}

// Main application entry point
void app_main(void) {
    ESP_LOGI(TAG, "SensDot starting... Boot count: %d", boot_count);
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize GPIO
    gpio_init();
    
    // Check for factory reset (hold boot button on startup)
    if (gpio_get_level(RESET_BUTTON_GPIO) == 0) {
        ESP_LOGI(TAG, "Factory reset button pressed");
        int hold_time = 0;
        while (gpio_get_level(RESET_BUTTON_GPIO) == 0 && hold_time < 5000) {
            vTaskDelay(pdMS_TO_TICKS(100));
            hold_time += 100;
        }
        if (hold_time >= 5000) {
            factory_reset();
            return;
        }
    }
    
    // Load configuration
    load_config();
    
    // Determine operation mode
    if (!config.configured || strlen(config.wifi_ssid) == 0) {
        // First time setup or invalid configuration
        enter_setup_mode();
    } else {
        // Normal operation
        normal_operation();
    }
}