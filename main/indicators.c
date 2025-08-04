/**
 * @file indicators.c
 * @brief LED and buzzer indication management implementation
 */

#include "indicators.h"
#include "sensdot_common.h"

static const char *TAG = "indicators";

// Indication state
static indication_state_t g_indication_state = {0};
static bool g_indicators_initialized = false;
static bool g_muted = false;
static uint8_t g_volume = 100;
static bool g_override_mode = false;
static indicators_callback_t g_callback = NULL;
static void *g_callback_user_data = NULL;

// Timer for indication patterns
static TimerHandle_t g_indication_timer = NULL;
static TimerHandle_t g_pattern_timer = NULL;

// Current pattern state
static uint8_t g_current_blink_count = 0;
static uint8_t g_target_blink_count = 0;
static bool g_led_state = false;
static bool g_buzzer_state = false;

// Forward declarations
static void indication_timer_callback(TimerHandle_t xTimer);
static void pattern_timer_callback(TimerHandle_t xTimer);
static esp_err_t start_blink_pattern(indicator_type_t type, uint8_t count, uint16_t on_time, uint16_t off_time);
static esp_err_t gpio_init_indicators(void);

/**
 * @brief Initialize GPIO for indicators
 */
static esp_err_t gpio_init_indicators(void)
{
    esp_err_t ret;
    
    // Configure LED GPIO
    gpio_config_t led_config = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&led_config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure buzzer GPIO
    gpio_config_t buzzer_config = {
        .pin_bit_mask = (1ULL << BUZZER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&buzzer_config);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Initialize to off state
    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(BUZZER_GPIO, 0);
    
    return ESP_OK;
}

/**
 * @brief Initialize indicators
 */
esp_err_t indicators_init(void)
{
    esp_err_t ret;
    
    SENSDOT_LOGI(TAG, "Initializing indicators");
    
    if (g_indicators_initialized) {
        SENSDOT_LOGW(TAG, "Indicators already initialized");
        return ESP_OK;
    }
    
    // Initialize GPIO
    ret = gpio_init_indicators();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to initialize GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create timers
    g_indication_timer = xTimerCreate("indication_timer", pdMS_TO_TICKS(100), pdTRUE, 
                                     NULL, indication_timer_callback);
    if (g_indication_timer == NULL) {
        SENSDOT_LOGE(TAG, "Failed to create indication timer");
        return ESP_FAIL;
    }
    
    g_pattern_timer = xTimerCreate("pattern_timer", pdMS_TO_TICKS(1000), pdFALSE, 
                                  NULL, pattern_timer_callback);
    if (g_pattern_timer == NULL) {
        SENSDOT_LOGE(TAG, "Failed to create pattern timer");
        vTimerDelete(g_indication_timer, 0);
        return ESP_FAIL;
    }
    
    // Initialize state
    memset(&g_indication_state, 0, sizeof(g_indication_state));
    g_muted = false;
    g_volume = 100;
    g_override_mode = false;
    
    g_indicators_initialized = true;
    
    SENSDOT_LOGI(TAG, "Indicators initialized successfully");
    return ESP_OK;
}

/**
 * @brief Deinitialize indicators
 */
esp_err_t indicators_deinit(void)
{
    if (!g_indicators_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Deinitializing indicators");
    
    // Stop all indications
    indicators_stop_all();
    
    // Delete timers
    if (g_indication_timer) {
        xTimerDelete(g_indication_timer, portMAX_DELAY);
        g_indication_timer = NULL;
    }
    
    if (g_pattern_timer) {
        xTimerDelete(g_pattern_timer, portMAX_DELAY);
        g_pattern_timer = NULL;
    }
    
    // Turn off all indicators
    led_off();
    buzzer_off();
    
    g_indicators_initialized = false;
    
    SENSDOT_LOGI(TAG, "Indicators deinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Turn LED on
 */
esp_err_t led_on(void)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    gpio_set_level(LED_GPIO, 1);
    g_led_state = true;
    return ESP_OK;
}

/**
 * @brief Turn LED off
 */
esp_err_t led_off(void)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    gpio_set_level(LED_GPIO, 0);
    g_led_state = false;
    return ESP_OK;
}

/**
 * @brief Toggle LED state
 */
esp_err_t led_toggle(void)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    g_led_state = !g_led_state;
    gpio_set_level(LED_GPIO, g_led_state ? 1 : 0);
    return ESP_OK;
}

/**
 * @brief Set LED brightness (basic implementation)
 */
esp_err_t led_set_brightness(uint8_t brightness)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    // Simple on/off implementation (could be enhanced with PWM)
    if (brightness > 127) {
        return led_on();
    } else {
        return led_off();
    }
}

/**
 * @brief Turn buzzer on
 */
esp_err_t buzzer_on(void)
{
    if (!g_indicators_initialized || g_muted) {
        return ESP_OK; // Silently ignore if muted
    }
    
    gpio_set_level(BUZZER_GPIO, 1);
    g_buzzer_state = true;
    return ESP_OK;
}

/**
 * @brief Turn buzzer off
 */
esp_err_t buzzer_off(void)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    gpio_set_level(BUZZER_GPIO, 0);
    g_buzzer_state = false;
    return ESP_OK;
}

/**
 * @brief Generate buzzer tone
 */
esp_err_t buzzer_tone(uint16_t frequency, uint16_t duration_ms)
{
    if (!g_indicators_initialized || g_muted) {
        return ESP_OK;
    }
    
    // Simple implementation - just turn on for duration
    // Could be enhanced with PWM for actual frequency generation
    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    buzzer_off();
    
    return ESP_OK;
}

/**
 * @brief Generate buzzer beep sequence
 */
esp_err_t buzzer_beep(uint8_t count, uint16_t duration_ms, uint16_t interval_ms)
{
    if (!g_indicators_initialized || g_muted) {
        return ESP_OK;
    }
    
    for (uint8_t i = 0; i < count; i++) {
        buzzer_on();
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        buzzer_off();
        
        if (i < count - 1) { // Don't delay after last beep
            vTaskDelay(pdMS_TO_TICKS(interval_ms));
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Timer callback for indication patterns
 */
static void indication_timer_callback(TimerHandle_t xTimer)
{
    if (!g_indication_state.is_active) {
        return;
    }
    
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if indication duration has expired
    if (g_indication_state.duration_ms > 0 && 
        (current_time - g_indication_state.start_time) >= g_indication_state.duration_ms) {
        
        // Stop indication
        indicators_stop(g_indication_state.active_indicators);
        return;
    }
    
    // Handle different patterns
    switch (g_indication_state.current_pattern) {
        case PATTERN_BLINK_SLOW:
            led_toggle();
            if (g_indication_state.active_indicators == INDICATOR_BOTH) {
                if (g_led_state) buzzer_on();
                else buzzer_off();
            }
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(1000), 0); // 1 second
            break;
            
        case PATTERN_BLINK_FAST:
            led_toggle();
            if (g_indication_state.active_indicators == INDICATOR_BOTH) {
                if (g_led_state) buzzer_on();
                else buzzer_off();
            }
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(250), 0); // 250ms
            break;
            
        case PATTERN_PULSE:
            led_toggle();
            xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), 0); // 100ms
            break;
            
        case PATTERN_HEARTBEAT:
            // Double blink pattern
            static uint8_t heartbeat_state = 0;
            switch (heartbeat_state) {
                case 0:
                    led_on();
                    heartbeat_state = 1;
                    xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), 0);
                    break;
                case 1:
                    led_off();
                    heartbeat_state = 2;
                    xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), 0);
                    break;
                case 2:
                    led_on();
                    heartbeat_state = 3;
                    xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), 0);
                    break;
                case 3:
                    led_off();
                    heartbeat_state = 0;
                    xTimerChangePeriod(xTimer, pdMS_TO_TICKS(1000), 0);
                    break;
            }
            break;
            
        default:
            break;
    }
}

/**
 * @brief Timer callback for pattern completion
 */
static void pattern_timer_callback(TimerHandle_t xTimer)
{
    // Pattern completed, stop indication
    indicators_stop(g_indication_state.active_indicators);
}

/**
 * @brief Start blink pattern
 */
static esp_err_t start_blink_pattern(indicator_type_t type, uint8_t count, uint16_t on_time, uint16_t off_time)
{
    g_current_blink_count = 0;
    g_target_blink_count = count;
    
    // Start first blink
    if (type == INDICATOR_LED || type == INDICATOR_BOTH) {
        led_on();
    }
    if (type == INDICATOR_BUZZER || type == INDICATOR_BOTH) {
        buzzer_on();
    }
    
    // Set timer for off phase
    xTimerChangePeriod(g_indication_timer, pdMS_TO_TICKS(on_time), 0);
    xTimerStart(g_indication_timer, 0);
    
    return ESP_OK;
}

/**
 * @brief Start indication pattern
 */
esp_err_t indicators_start_pattern(indicator_type_t type, indication_pattern_t pattern, 
                                  indication_priority_t priority, uint32_t duration_ms)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    // Check priority - only allow higher or equal priority to override
    if (g_indication_state.is_active && !g_override_mode && 
        priority < g_indication_state.priority) {
        SENSDOT_LOGD(TAG, "Indication blocked by higher priority");
        return ESP_OK; // Not an error, just ignored
    }
    
    // Stop any current indication
    indicators_stop_all();
    
    // Set new indication state
    g_indication_state.current_pattern = pattern;
    g_indication_state.active_indicators = type;
    g_indication_state.priority = priority;
    g_indication_state.start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_indication_state.duration_ms = duration_ms;
    g_indication_state.is_active = true;
    
    SENSDOT_LOGD(TAG, "Starting pattern %d for type %d, priority %d, duration %lu", 
                pattern, type, priority, duration_ms);
    
    // Handle different patterns
    switch (pattern) {
        case PATTERN_OFF:
            indicators_stop_all();
            break;
            
        case PATTERN_ON:
            if (type == INDICATOR_LED || type == INDICATOR_BOTH) {
                led_on();
            }
            if (type == INDICATOR_BUZZER || type == INDICATOR_BOTH) {
                buzzer_on();
            }
            break;
            
        case PATTERN_BLINK_SLOW:
        case PATTERN_BLINK_FAST:
        case PATTERN_PULSE:
        case PATTERN_HEARTBEAT:
            xTimerStart(g_indication_timer, 0);
            break;
            
        case PATTERN_SOS:
            // SOS pattern: ... --- ...
            // Implementation would require more complex state machine
            start_blink_pattern(type, 3, 200, 200); // Simplified to 3 short blinks
            break;
            
        default:
            SENSDOT_LOGW(TAG, "Unknown pattern: %d", pattern);
            return SENSDOT_ERR_INVALID_ARG;
    }
    
    // Call callback if registered
    if (g_callback) {
        g_callback(pattern, type, g_callback_user_data);
    }
    
    // Set duration timer if specified
    if (duration_ms > 0) {
        xTimerChangePeriod(g_pattern_timer, pdMS_TO_TICKS(duration_ms), 0);
        xTimerStart(g_pattern_timer, 0);
    }
    
    return ESP_OK;
}

/**
 * @brief Stop indication
 */
esp_err_t indicators_stop(indicator_type_t type)
{
    if (!g_indicators_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGD(TAG, "Stopping indication type %d", type);
    
    // Stop timers
    xTimerStop(g_indication_timer, 0);
    xTimerStop(g_pattern_timer, 0);
    
    // Turn off indicators
    if (type == INDICATOR_LED || type == INDICATOR_BOTH) {
        led_off();
    }
    if (type == INDICATOR_BUZZER || type == INDICATOR_BOTH) {
        buzzer_off();
    }
    
    // Clear state if stopping current indication
    if (type == g_indication_state.active_indicators || type == INDICATOR_BOTH) {
        memset(&g_indication_state, 0, sizeof(g_indication_state));
    }
    
    return ESP_OK;
}

/**
 * @brief Stop all indications
 */
esp_err_t indicators_stop_all(void)
{
    return indicators_stop(INDICATOR_BOTH);
}

/**
 * @brief Indicate device status
 */
esp_err_t indicators_show_status(status_pattern_t status, bool with_sound)
{
    indicator_type_t type = with_sound ? INDICATOR_BOTH : INDICATOR_LED;
    
    switch (status) {
        case STATUS_READY:
            return indicators_start_pattern(type, PATTERN_BLINK_SLOW, PRIORITY_LOW, 2000);
            
        case STATUS_CONNECTING:
            return indicators_start_pattern(type, PATTERN_BLINK_FAST, PRIORITY_NORMAL, 0);
            
        case STATUS_SUCCESS:
            return start_blink_pattern(type, 3, 100, 100);
            
        case STATUS_ERROR:
            return start_blink_pattern(type, 4, 200, 200);
            
        case STATUS_FACTORY_RESET:
            return start_blink_pattern(type, 5, 200, 200);
            
        default:
            return SENSDOT_ERR_INVALID_ARG;
    }
}

/**
 * @brief Indicate error condition
 */
esp_err_t indicators_show_error(int error_code, bool critical)
{
    indication_priority_t priority = critical ? PRIORITY_CRITICAL : PRIORITY_HIGH;
    uint32_t duration = critical ? 0 : 5000; // Critical errors stay on
    
    return indicators_start_pattern(INDICATOR_BOTH, PATTERN_BLINK_FAST, priority, duration);
}

/**
 * @brief Indicate low battery condition
 */
esp_err_t indicators_show_low_battery(void)
{
    return start_blink_pattern(INDICATOR_BOTH, 3, 100, 100);
}

/**
 * @brief Indicate setup mode
 */
esp_err_t indicators_show_setup_mode(void)
{
    return indicators_start_pattern(INDICATOR_LED, PATTERN_ON, PRIORITY_HIGH, 0);
}

/**
 * @brief Indicate WiFi connecting
 */
esp_err_t indicators_show_wifi_connecting(void)
{
    return indicators_start_pattern(INDICATOR_LED, PATTERN_BLINK_FAST, PRIORITY_NORMAL, 0);
}

/**
 * @brief Indicate WiFi connected
 */
esp_err_t indicators_show_wifi_connected(void)
{
    return start_blink_pattern(INDICATOR_LED, 2, 200, 200);
}

/**
 * @brief Indicate MQTT connecting
 */
esp_err_t indicators_show_mqtt_connecting(void)
{
    return indicators_start_pattern(INDICATOR_LED, PATTERN_PULSE, PRIORITY_NORMAL, 0);
}

/**
 * @brief Indicate MQTT connected
 */
esp_err_t indicators_show_mqtt_connected(void)
{
    return start_blink_pattern(INDICATOR_LED, 1, 500, 0);
}

/**
 * @brief Indicate data transmission
 */
esp_err_t indicators_show_data_transmission(void)
{
    return start_blink_pattern(INDICATOR_LED, 1, 100, 0);
}

/**
 * @brief Indicate factory reset
 */
esp_err_t indicators_show_factory_reset(void)
{
    return start_blink_pattern(INDICATOR_BOTH, 5, 200, 200);
}

/**
 * @brief Indicate configuration saved
 */
esp_err_t indicators_show_config_saved(void)
{
    return start_blink_pattern(INDICATOR_BOTH, 3, 100, 100);
}

/**
 * @brief Get current indication state
 */
esp_err_t indicators_get_state(indication_state_t *state)
{
    CHECK_NULL_PTR(state);
    
    memcpy(state, &g_indication_state, sizeof(indication_state_t));
    return ESP_OK;
}

/**
 * @brief Check if indicators are active
 */
bool indicators_is_active(indicator_type_t type)
{
    if (!g_indication_state.is_active) {
        return false;
    }
    
    return (g_indication_state.active_indicators == type || 
            g_indication_state.active_indicators == INDICATOR_BOTH);
}

/**
 * @brief Set indication override mode
 */
esp_err_t indicators_set_override_mode(bool enable)
{
    g_override_mode = enable;
    SENSDOT_LOGD(TAG, "Override mode %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

/**
 * @brief Check if override mode is active
 */
bool indicators_is_override_mode(void)
{
    return g_override_mode;
}

/**
 * @brief Set indication volume/intensity
 */
esp_err_t indicators_set_volume(uint8_t volume)
{
    g_volume = MIN(volume, 100);
    SENSDOT_LOGD(TAG, "Volume set to %d", g_volume);
    return ESP_OK;
}

/**
 * @brief Get indication volume/intensity
 */
uint8_t indicators_get_volume(void)
{
    return g_volume;
}

/**
 * @brief Mute all indications
 */
esp_err_t indicators_set_mute(bool mute)
{
    g_muted = mute;
    SENSDOT_LOGD(TAG, "Indicators %s", mute ? "muted" : "unmuted");
    
    if (mute) {
        buzzer_off(); // Turn off buzzer immediately when muted
    }
    
    return ESP_OK;
}

/**
 * @brief Check if indications are muted
 */
bool indicators_is_muted(void)
{
    return g_muted;
}

/**
 * @brief Test all indicators
 */
esp_err_t indicators_test_all(void)
{
    SENSDOT_LOGI(TAG, "Testing all indicators");
    
    // Test LED
    led_on();
    vTaskDelay(pdMS_TO_TICKS(500));
    led_off();
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test buzzer
    buzzer_tone(1000, 500);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Test combined
    start_blink_pattern(INDICATOR_BOTH, 3, 200, 200);
    
    return ESP_OK;
}

/**
 * @brief Test specific indicator
 */
esp_err_t indicators_test(indicator_type_t type)
{
    SENSDOT_LOGI(TAG, "Testing indicator type %d", type);
    
    return start_blink_pattern(type, 3, 300, 300);
}

/**
 * @brief Set indication callback
 */
esp_err_t indicators_set_callback(indicators_callback_t callback, void *user_data)
{
    g_callback = callback;
    g_callback_user_data = user_data;
    return ESP_OK;
}

/**
 * @brief Get indication pattern name for debugging
 */
esp_err_t indicators_get_pattern_name(indication_pattern_t pattern, char *pattern_name, size_t max_len)
{
    CHECK_NULL_PTR(pattern_name);
    
    const char *name;
    switch (pattern) {
        case PATTERN_OFF: name = "OFF"; break;
        case PATTERN_ON: name = "ON"; break;
        case PATTERN_BLINK_SLOW: name = "BLINK_SLOW"; break;
        case PATTERN_BLINK_FAST: name = "BLINK_FAST"; break;
        case PATTERN_PULSE: name = "PULSE"; break;
        case PATTERN_HEARTBEAT: name = "HEARTBEAT"; break;
        case PATTERN_SOS: name = "SOS"; break;
        case PATTERN_CUSTOM: name = "CUSTOM"; break;
        default: name = "UNKNOWN"; break;
    }
    
    strncpy(pattern_name, name, max_len - 1);
    pattern_name[max_len - 1] = '\0';
    
    return ESP_OK;
}

/**
 * @brief Print indicators status for debugging
 */
void indicators_print_status(void)
{
    char pattern_name[16];
    indicators_get_pattern_name(g_indication_state.current_pattern, pattern_name, sizeof(pattern_name));
    
    SENSDOT_LOGI(TAG, "=== Indicators Status ===");
    SENSDOT_LOGI(TAG, "Initialized: %s", g_indicators_initialized ? "Yes" : "No");
    SENSDOT_LOGI(TAG, "Active: %s", g_indication_state.is_active ? "Yes" : "No");
    SENSDOT_LOGI(TAG, "Pattern: %s", pattern_name);
    SENSDOT_LOGI(TAG, "Type: %d", g_indication_state.active_indicators);
    SENSDOT_LOGI(TAG, "Priority: %d", g_indication_state.priority);
    SENSDOT_LOGI(TAG, "LED State: %s", g_led_state ? "ON" : "OFF");
    SENSDOT_LOGI(TAG, "Buzzer State: %s", g_buzzer_state ? "ON" : "OFF");
    SENSDOT_LOGI(TAG, "Muted: %s", g_muted ? "Yes" : "No");
    SENSDOT_LOGI(TAG, "Volume: %d", g_volume);
    SENSDOT_LOGI(TAG, "Override Mode: %s", g_override_mode ? "Yes" : "No");
    SENSDOT_LOGI(TAG, "========================");
}

/**
 * @brief Save indication configuration to NVS
 */
esp_err_t indicators_save_config(void)
{
    // Implementation would save volume, mute state, etc. to NVS
    // For now, just return success
    return ESP_OK;
}

/**
 * @brief Load indication configuration from NVS
 */
esp_err_t indicators_load_config(void)
{
    // Implementation would load volume, mute state, etc. from NVS
    // For now, just return success
    return ESP_OK;
}

/**
 * @brief Reset indication configuration to defaults
 */
esp_err_t indicators_reset_config(void)
{
    g_volume = 100;
    g_muted = false;
    g_override_mode = false;
    
    SENSDOT_LOGI(TAG, "Indicators configuration reset to defaults");
    return ESP_OK;
}