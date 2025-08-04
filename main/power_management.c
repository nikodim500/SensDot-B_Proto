/**
 * @file power_management.c
 * @brief Power management and sleep functionality for ESP32-C3 (no ULP)
 */

#include "power_management.h"
#include "sensdot_common.h"
#include "config.h"

static const char *TAG = "power_mgr";

// Power management state
static bool g_power_initialized = false;
static power_mode_t g_current_mode = POWER_MODE_NORMAL;
static power_config_t g_power_config = {0};

// Sleep statistics (stored in RTC memory)
RTC_DATA_ATTR static sleep_stats_t g_sleep_stats = {0};
RTC_DATA_ATTR static uint64_t g_last_sleep_start = 0;

// Wake up information
static wakeup_reason_t g_last_wakeup_reason = WAKEUP_REASON_UNKNOWN;
static int g_last_wakeup_gpio = -1;

// Forward declarations
static wakeup_reason_t determine_wakeup_reason(void);
static esp_err_t configure_power_domains(void);
static esp_err_t update_sleep_stats(void);

/**
 * @brief Determine wake up reason from ESP32-C3 wake up cause
 */
static wakeup_reason_t determine_wakeup_reason(void)
{
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    
    switch (wakeup_cause) {
        case ESP_SLEEP_WAKEUP_TIMER:
            return WAKEUP_REASON_TIMER;
            
        case ESP_SLEEP_WAKEUP_EXT0:
            // EXT0 configured for PIR sensor on GPIO4
            g_last_wakeup_gpio = PIR_GPIO;
            return WAKEUP_REASON_PIR;
            
        case ESP_SLEEP_WAKEUP_EXT1:
            // ESP32-C3 has limited EXT1 support, not used in this project
            return WAKEUP_REASON_UNKNOWN;
            
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            return WAKEUP_REASON_RESET; // Power-on or reset
    }
}

/**
 * @brief Configure power domains for optimal energy consumption on ESP32-C3
 */
static esp_err_t configure_power_domains(void)
{
    esp_err_t ret = ESP_OK;
    
    // Configure RTC peripherals power domain
    ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, g_power_config.rtc_periph_pd);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure RTC periph power domain: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure RTC slow memory power domain (keep for RTC_DATA_ATTR variables)
    ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, g_power_config.rtc_slow_mem_pd);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure RTC slow mem power domain: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure RTC fast memory power domain
    ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, g_power_config.rtc_fast_mem_pd);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to configure RTC fast mem power domain: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure flash power down if enabled (for minimal power consumption)
    if (g_power_config.power_down_flash) {
        ret = esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_OFF);
        if (ret != ESP_OK) {
            SENSDOT_LOGW(TAG, "Failed to configure flash power down: %s", esp_err_to_name(ret));
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Update sleep statistics
 */
static esp_err_t update_sleep_stats(void)
{
    uint64_t current_time = esp_timer_get_time();
    
    if (g_last_sleep_start > 0) {
        uint64_t sleep_duration = current_time - g_last_sleep_start;
        g_sleep_stats.total_sleep_time_us += sleep_duration;
    }
    
    g_sleep_stats.last_wakeup_time = time(NULL);
    g_sleep_stats.last_wakeup_reason = g_last_wakeup_reason;
    
    return ESP_OK;
}

/**
 * @brief Initialize power management for ESP32-C3
 */
esp_err_t power_management_init(void)
{
    SENSDOT_LOGI(TAG, "Initializing power management for ESP32-C3");
    
    if (g_power_initialized) {
        SENSDOT_LOGW(TAG, "Power management already initialized");
        return ESP_OK;
    }
    
    // Determine wake up reason
    g_last_wakeup_reason = determine_wakeup_reason();
    update_sleep_stats();
    
    // Initialize default power configuration
    power_reset_config();
    
    // Configure power domains
    esp_err_t ret = configure_power_domains();
    if (ret != ESP_OK) {
        return ret;
    }
    
    g_power_initialized = true;
    g_current_mode = POWER_MODE_NORMAL;
    
    SENSDOT_LOGI(TAG, "Power management initialized, wake reason: %d", g_last_wakeup_reason);
    return ESP_OK;
}

/**
 * @brief Deinitialize power management
 */
esp_err_t power_management_deinit(void)
{
    if (!g_power_initialized) {
        return ESP_OK;
    }
    
    SENSDOT_LOGI(TAG, "Deinitializing power management");
    
    g_power_initialized = false;
    g_current_mode = POWER_MODE_NORMAL;
    
    return ESP_OK;
}

/**
 * @brief Set power mode
 */
esp_err_t power_set_mode(power_mode_t mode)
{
    if (!g_power_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGD(TAG, "Setting power mode: %d", mode);
    
    esp_err_t ret = ESP_OK;
    
    switch (mode) {
        case POWER_MODE_NORMAL:
            // Normal operation - no special power saving
            break;
            
        case POWER_MODE_LOW_POWER:
            // Enable automatic light sleep on ESP32-C3
            ret = power_set_automatic_light_sleep(true, 80); // Min 80MHz
            break;
            
        case POWER_MODE_DEEP_SLEEP:
            // This mode is handled by power_enter_deep_sleep()
            break;
            
        case POWER_MODE_LIGHT_SLEEP:
            // This mode is handled by power_enter_light_sleep()
            break;
            
        default:
            SENSDOT_LOGE(TAG, "Invalid power mode: %d", mode);
            return SENSDOT_ERR_INVALID_ARG;
    }
    
    if (ret == ESP_OK) {
        g_current_mode = mode;
    }
    
    return ret;
}

/**
 * @brief Get current power mode
 */
power_mode_t power_get_mode(void)
{
    return g_current_mode;
}

/**
 * @brief Enter deep sleep (main sleep mode for ESP32-C3)
 */
void power_enter_deep_sleep(uint32_t sleep_time_sec)
{
    SENSDOT_LOGI(TAG, "Entering deep sleep for %lu seconds", sleep_time_sec);
    
    // Record sleep start time
    g_last_sleep_start = esp_timer_get_time();
    g_sleep_stats.last_sleep_time = time(NULL);
    g_sleep_stats.deep_sleep_count++;
    
    // Configure wake up sources based on configuration
    uint32_t sources = WAKEUP_SOURCE_TIMER;
    if (g_power_config.enable_pir_wakeup) {
        sources |= WAKEUP_SOURCE_PIR;
    }
    
    power_prepare_for_sleep(sleep_time_sec, sources);
    
    // Enter deep sleep - this function does not return
    esp_deep_sleep_start();
}

/**
 * @brief Enter light sleep
 */
esp_err_t power_enter_light_sleep(uint32_t sleep_time_ms)
{
    if (!g_power_initialized) {
        return SENSDOT_ERR_INVALID_STATE;
    }
    
    SENSDOT_LOGD(TAG, "Entering light sleep for %lu ms", sleep_time_ms);
    
    // Record sleep start time
    uint64_t sleep_start = esp_timer_get_time();
    g_sleep_stats.light_sleep_count++;
    
    // Configure timer wake up
    esp_err_t ret = esp_sleep_enable_timer_wakeup(sleep_time_ms * 1000ULL);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Enter light sleep
    ret = esp_light_sleep_start();
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Light sleep failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Update sleep statistics
    uint64_t sleep_duration = esp_timer_get_time() - sleep_start;
    g_sleep_stats.total_sleep_time_us += sleep_duration;
    
    SENSDOT_LOGD(TAG, "Light sleep completed, duration: %llu us", sleep_duration);
    return ESP_OK;
}

/**
 * @brief Configure wake up sources for ESP32-C3
 */
esp_err_t power_configure_wakeup_sources(uint32_t sources)
{
    esp_err_t ret = ESP_OK;
    
    SENSDOT_LOGD(TAG, "Configuring wakeup sources: 0x%lx", sources);
    
    // Disable all wakeup sources first
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    
    // Configure timer wakeup
    if (sources & WAKEUP_SOURCE_TIMER && g_power_config.enable_timer_wakeup) {
        ret = power_enable_timer_wakeup(g_power_config.wake_interval_sec);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Configure PIR sensor wakeup (EXT0) - main feature for motion detection
    if (sources & WAKEUP_SOURCE_PIR && g_power_config.enable_pir_wakeup) {
        ret = power_enable_ext0_wakeup(PIR_GPIO, 1); // Wake on HIGH level
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to enable PIR wakeup: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Enable timer wake up
 */
esp_err_t power_enable_timer_wakeup(uint32_t time_sec)
{
    if (time_sec == 0) {
        SENSDOT_LOGW(TAG, "Timer wakeup disabled (0 seconds)");
        return ESP_OK;
    }
    
    uint64_t time_us = (uint64_t)time_sec * 1000000ULL;
    esp_err_t ret = esp_sleep_enable_timer_wakeup(time_us);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to enable timer wakeup: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGD(TAG, "Timer wakeup enabled for %lu seconds", time_sec);
    return ESP_OK;
}

/**
 * @brief Enable external wake up (PIR sensor) - critical for ESP32-C3
 */
esp_err_t power_enable_ext0_wakeup(gpio_num_t gpio_num, int level)
{
    // Configure GPIO as RTC IO (GPIO4 supports RTC on ESP32-C3)
    if (rtc_gpio_is_valid_gpio(gpio_num)) {
        rtc_gpio_init(gpio_num);
        rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
        rtc_gpio_pulldown_en(gpio_num);
        rtc_gpio_pullup_dis(gpio_num);
    } else {
        SENSDOT_LOGE(TAG, "GPIO%d does not support RTC wake-up on ESP32-C3", gpio_num);
        return SENSDOT_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = esp_sleep_enable_ext0_wakeup(gpio_num, level);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to enable EXT0 wakeup: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGD(TAG, "EXT0 wakeup enabled on GPIO%d, level=%d", gpio_num, level);
    return ESP_OK;
}

/**
 * @brief Enable multiple external wake up sources (limited on ESP32-C3)
 */
esp_err_t power_enable_ext1_wakeup(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode)
{
    // ESP32-C3 has limited EXT1 support, not implemented for this project
    SENSDOT_LOGW(TAG, "EXT1 wakeup not used on ESP32-C3, using EXT0 for PIR");
    return ESP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Disable wake up source
 */
esp_err_t power_disable_wakeup_source(wakeup_source_t source)
{
    esp_sleep_wakeup_cause_t esp_source;
    
    switch (source) {
        case WAKEUP_SOURCE_TIMER:
            esp_source = ESP_SLEEP_WAKEUP_TIMER;
            break;
        case WAKEUP_SOURCE_PIR:
            esp_source = ESP_SLEEP_WAKEUP_EXT0;
            break;
        default:
            return SENSDOT_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = esp_sleep_disable_wakeup_source(esp_source);
    if (ret != ESP_OK) {
        SENSDOT_LOGE(TAG, "Failed to disable wakeup source: %s", esp_err_to_name(ret));
        return ret;
    }
    
    SENSDOT_LOGD(TAG, "Wakeup source %d disabled", source);
    return ESP_OK;
}

/**
 * @brief Get wake up reason
 */
wakeup_reason_t power_get_wakeup_reason(void)
{
    return g_last_wakeup_reason;
}

/**
 * @brief Get wake up GPIO number
 */
int power_get_wakeup_gpio(void)
{
    return g_last_wakeup_gpio;
}

/**
 * @brief Power down peripherals before sleep
 */
esp_err_t power_down_peripherals(void)
{
    SENSDOT_LOGD(TAG, "Powering down peripherals");
    
    // Turn off sensor power
    gpio_set_level(SENSOR_PWR_GPIO, 0);
    
    // Turn off LED and buzzer
    gpio_set_level(LED_GPIO, 0);
    gpio_set_level(BUZZER_GPIO, 0);
    
    // Isolate GPIOs to prevent current leakage during deep sleep
    esp_sleep_config_gpio_isolate();
    gpio_deep_sleep_hold_en();
    
    return ESP_OK;
}

/**
 * @brief Power up peripherals after wake up
 */
esp_err_t power_up_peripherals(void)
{
    SENSDOT_LOGD(TAG, "Powering up peripherals");
    
    // Disable GPIO hold after wake up
    gpio_deep_sleep_hold_dis();
    
    return ESP_OK;
}

/**
 * @brief Set power configuration
 */
esp_err_t power_set_config(const power_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    memcpy(&g_power_config, config, sizeof(power_config_t));
    
    // Apply configuration
    esp_err_t ret = configure_power_domains();
    if (ret != ESP_OK) {
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "Power configuration updated");
    return ESP_OK;
}

/**
 * @brief Get power configuration
 */
esp_err_t power_get_config(power_config_t *config)
{
    CHECK_NULL_PTR(config);
    
    memcpy(config, &g_power_config, sizeof(power_config_t));
    return ESP_OK;
}

/**
 * @brief Reset power configuration to defaults for ESP32-C3
 */
esp_err_t power_reset_config(void)
{
    // Load defaults from device configuration
    const device_config_t *device_config = config_get_current();
    
    g_power_config.wake_interval_sec = device_config->wake_interval_sec;
    g_power_config.enable_timer_wakeup = true;
    g_power_config.enable_pir_wakeup = true;       // Critical for motion detection
    g_power_config.enable_button_wakeup = false;   // No button on ESP32-C3 Super Mini
    g_power_config.power_down_flash = true;        // Aggressive power saving
    g_power_config.enable_ulp = false;             // ESP32-C3 has no ULP
    g_power_config.rtc_periph_pd = ESP_PD_OPTION_ON;  // Keep RTC peripherals for PIR
    g_power_config.rtc_slow_mem_pd = ESP_PD_OPTION_ON; // Keep RTC memory for variables
    g_power_config.rtc_fast_mem_pd = ESP_PD_OPTION_ON; // Keep RTC fast memory
    
    SENSDOT_LOGI(TAG, "Power configuration reset to ESP32-C3 defaults");
    return ESP_OK;
}

/**
 * @brief Get sleep statistics
 */
esp_err_t power_get_sleep_stats(sleep_stats_t *stats)
{
    CHECK_NULL_PTR(stats);
    
    memcpy(stats, &g_sleep_stats, sizeof(sleep_stats_t));
    return ESP_OK;
}

/**
 * @brief Reset sleep statistics
 */
void power_reset_sleep_stats(void)
{
    memset(&g_sleep_stats, 0, sizeof(sleep_stats_t));
    SENSDOT_LOGI(TAG, "Sleep statistics reset");
}

/**
 * @brief Check if wake up was from deep sleep
 */
bool power_is_wakeup_from_deep_sleep(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    return cause != ESP_SLEEP_WAKEUP_UNDEFINED;
}

/**
 * @brief Get time spent in sleep
 */
uint64_t power_get_sleep_time_us(void)
{
    return g_sleep_stats.total_sleep_time_us;
}

/**
 * @brief Get estimated battery life for ESP32-C3
 */
uint32_t power_estimate_battery_life(uint32_t battery_capacity_mah, float current_consumption_ma)
{
    if (current_consumption_ma <= 0) {
        return 0;
    }
    
    uint32_t battery_life_hours = (uint32_t)(battery_capacity_mah / current_consumption_ma);
    return battery_life_hours;
}

/**
 * @brief Calculate average power consumption for ESP32-C3
 */
float power_calculate_average_consumption(uint32_t measurement_period_sec)
{
    // Optimized values for ESP32-C3
    float active_time_ratio = 0.005f; // 0.5% active time (very efficient)
    float active_current_ma = 95.0f;  // ESP32-C3 active consumption
    float sleep_current_ua = 10.0f;   // ESP32-C3 deep sleep consumption
    
    float average_ma = (active_time_ratio * active_current_ma) + 
                      ((1.0f - active_time_ratio) * sleep_current_ua / 1000.0f);
    
    return average_ma;
}

/**
 * @brief Set RTC memory retention
 */
esp_err_t power_set_rtc_memory_retention(bool retain_slow_mem, bool retain_fast_mem)
{
    esp_err_t ret = ESP_OK;
    
    if (retain_slow_mem) {
        ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to retain RTC slow memory: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    }
    
    if (retain_fast_mem) {
        ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to retain RTC fast memory: %s", esp_err_to_name(ret));
            return ret;
        }
    } else {
        ret = esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    }
    
    SENSDOT_LOGI(TAG, "RTC memory retention: slow=%s, fast=%s", 
                retain_slow_mem ? "ON" : "OFF", retain_fast_mem ? "ON" : "OFF");
    
    return ESP_OK;
}

/**
 * @brief Prepare for sleep (optimized for ESP32-C3)
 */
esp_err_t power_prepare_for_sleep(uint32_t sleep_time_sec, uint32_t sources)
{
    SENSDOT_LOGD(TAG, "Preparing for sleep: %lu sec, sources=0x%lx", sleep_time_sec, sources);
    
    // Power down peripherals
    esp_err_t ret = power_down_peripherals();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure wake up sources
    ret = power_configure_wakeup_sources(sources);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Configure power domains
    ret = configure_power_domains();
    if (ret != ESP_OK) {
        return ret;
    }
    
    return ESP_OK;
}

/**
 * @brief Handle wake up
 */
esp_err_t power_handle_wakeup(void)
{
    SENSDOT_LOGD(TAG, "Handling wakeup");
    
    // Update wake up reason and statistics
    g_last_wakeup_reason = determine_wakeup_reason();
    update_sleep_stats();
    
    // Power up peripherals
    esp_err_t ret = power_up_peripherals();
    if (ret != ESP_OK) {
        return ret;
    }
    
    SENSDOT_LOGI(TAG, "Wakeup handled, reason: %d", g_last_wakeup_reason);
    return ESP_OK;
}

/**
 * @brief Enable/disable automatic light sleep
 */
esp_err_t power_set_automatic_light_sleep(bool enable, uint32_t min_freq_mhz)
{
    if (enable) {
        esp_pm_config_t pm_config = {
            .max_freq_mhz = 160,        // ESP32-C3 max frequency
            .min_freq_mhz = min_freq_mhz,
            .light_sleep_enable = true
        };
        
        esp_err_t ret = esp_pm_configure(&pm_config);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to enable automatic light sleep: %s", esp_err_to_name(ret));
            return ret;
        }
        
        SENSDOT_LOGI(TAG, "Automatic light sleep enabled, min freq: %lu MHz", min_freq_mhz);
    } else {
        esp_err_t ret = esp_pm_configure(NULL);
        if (ret != ESP_OK) {
            SENSDOT_LOGE(TAG, "Failed to disable power management: %s", esp_err_to_name(ret));
            return ret;
        }
        
        SENSDOT_LOGI(TAG, "Automatic light sleep disabled");
    }
    
    return ESP_OK;
}

/**
 * @brief Get current power consumption estimate for ESP32-C3
 */
float power_get_current_consumption_estimate(void)
{
    // Optimized estimates for ESP32-C3
    switch (g_current_mode) {
        case POWER_MODE_NORMAL:
            return 60.0f; // mA (lower than ESP32-S)
        case POWER_MODE_LOW_POWER:
            return 20.0f; // mA
        case POWER_MODE_DEEP_SLEEP:
            return 0.010f; // mA (10 μA - excellent!)
        case POWER_MODE_LIGHT_SLEEP:
            return 0.8f; // mA
        default:
            return 80.0f; // mA
    }
}

/**
 * @brief Print power configuration and statistics
 */
void power_print_info(void)
{
    SENSDOT_LOGI(TAG, "=== Power Management Info (ESP32-C3) ===");
    SENSDOT_LOGI(TAG, "Current Mode: %d", g_current_mode);
    SENSDOT_LOGI(TAG, "Last Wakeup Reason: %d", g_last_wakeup_reason);
    SENSDOT_LOGI(TAG, "Last Wakeup GPIO: %d", g_last_wakeup_gpio);
    SENSDOT_LOGI(TAG, "Deep Sleep Count: %lu", g_sleep_stats.deep_sleep_count);
    SENSDOT_LOGI(TAG, "Light Sleep Count: %lu", g_sleep_stats.light_sleep_count);
    SENSDOT_LOGI(TAG, "Total Sleep Time: %llu us", g_sleep_stats.total_sleep_time_us);
    SENSDOT_LOGI(TAG, "Wake Interval: %lu sec", g_power_config.wake_interval_sec);
    SENSDOT_LOGI(TAG, "Timer Wakeup: %s", g_power_config.enable_timer_wakeup ? "ON" : "OFF");
    SENSDOT_LOGI(TAG, "PIR Wakeup: %s", g_power_config.enable_pir_wakeup ? "ON" : "OFF");
    SENSDOT_LOGI(TAG, "Flash Power Down: %s", g_power_config.power_down_flash ? "ON" : "OFF");
    SENSDOT_LOGI(TAG, "Est. Current: %.3f mA", power_get_current_consumption_estimate());
    SENSDOT_LOGI(TAG, "=======================================");
}

/**
 * @brief Get power mode string for debugging
 */
esp_err_t power_get_mode_string(power_mode_t mode, char *mode_str, size_t max_len)
{
    CHECK_NULL_PTR(mode_str);
    
    const char *mode_names[] = {
        "NORMAL", "LOW_POWER", "DEEP_SLEEP", "LIGHT_SLEEP"
    };
    
    if (mode < sizeof(mode_names)/sizeof(mode_names[0])) {
        strncpy(mode_str, mode_names[mode], max_len - 1);
    } else {
        strncpy(mode_str, "UNKNOWN", max_len - 1);
    }
    mode_str[max_len - 1] = '\0';
    
    return ESP_OK;
}

/**
 * @brief Get wake up reason string for debugging
 */
esp_err_t power_get_wakeup_reason_string(wakeup_reason_t reason, char *reason_str, size_t max_len)
{
    CHECK_NULL_PTR(reason_str);
    
    const char *reason_names[] = {
        "TIMER", "PIR", "RESET", "UNKNOWN"
    };
    
    if (reason < sizeof(reason_names)/sizeof(reason_names[0])) {
        strncpy(reason_str, reason_names[reason], max_len - 1);
    } else {
        strncpy(reason_str, "INVALID", max_len - 1);
    }
    reason_str[max_len - 1] = '\0';
    
    return ESP_OK;
}