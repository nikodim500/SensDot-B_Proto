/**
 * @file power_management.h
 * @brief Power management and sleep functionality for ESP32-C3 (no ULP)
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "sensdot_common.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "driver/rtc_io.h"

#ifdef __cplusplus
extern "C" {
#endif

// Power modes
typedef enum {
    POWER_MODE_NORMAL,
    POWER_MODE_LOW_POWER,
    POWER_MODE_DEEP_SLEEP,
    POWER_MODE_LIGHT_SLEEP
} power_mode_t;

// Wake up sources (ESP32-C3 specific)
typedef enum {
    WAKEUP_SOURCE_TIMER = BIT0,
    WAKEUP_SOURCE_PIR = BIT1,
    // Note: ESP32-C3 has limited wake-up sources compared to ESP32-S
    // ULP and some other sources not available
} wakeup_source_t;

// Power domain configuration
typedef enum {
    POWER_DOMAIN_RTC_PERIPH,
    POWER_DOMAIN_RTC_SLOW_MEM,
    POWER_DOMAIN_RTC_FAST_MEM,
    POWER_DOMAIN_XTAL,
    POWER_DOMAIN_CPU
    // Note: ESP32-C3 has fewer power domains than ESP32-S
} power_domain_t;

// Sleep statistics
typedef struct {
    uint32_t deep_sleep_count;
    uint32_t light_sleep_count;
    uint64_t total_sleep_time_us;
    uint64_t total_awake_time_us;
    wakeup_reason_t last_wakeup_reason;
    time_t last_sleep_time;
    time_t last_wakeup_time;
} sleep_stats_t;

// Power configuration (optimized for ESP32-C3)
typedef struct {
    uint32_t wake_interval_sec;
    bool enable_timer_wakeup;
    bool enable_pir_wakeup;        // Critical for motion detection
    bool enable_button_wakeup;     // Not used on ESP32-C3 Super Mini
    bool power_down_flash;         // Flash power down for ultra-low power
    bool enable_ulp;               // Always false for ESP32-C3
    esp_sleep_pd_option_t rtc_periph_pd;
    esp_sleep_pd_option_t rtc_slow_mem_pd;
    esp_sleep_pd_option_t rtc_fast_mem_pd;
} power_config_t;

/**
 * @brief Initialize power management for ESP32-C3
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_init(void);

/**
 * @brief Deinitialize power management
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_management_deinit(void);

/**
 * @brief Set power mode
 * @param mode Power mode to set
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_set_mode(power_mode_t mode);

/**
 * @brief Get current power mode
 * @return Current power mode
 */
power_mode_t power_get_mode(void);

/**
 * @brief Enter deep sleep (main sleep mode for ESP32-C3)
 * @param sleep_time_sec Sleep time in seconds (0 for indefinite)
 * @return This function does not return on success
 */
void power_enter_deep_sleep(uint32_t sleep_time_sec);

/**
 * @brief Enter light sleep
 * @param sleep_time_ms Sleep time in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_enter_light_sleep(uint32_t sleep_time_ms);

/**
 * @brief Configure wake up sources for ESP32-C3
 * @param sources Bitfield of wake up sources to enable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_configure_wakeup_sources(uint32_t sources);

/**
 * @brief Enable timer wake up
 * @param time_sec Wake up time in seconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_enable_timer_wakeup(uint32_t time_sec);

/**
 * @brief Enable external wake up (PIR sensor) - critical for ESP32-C3
 * @param gpio_num GPIO number for external wake up (must be RTC GPIO)
 * @param level Trigger level (0 or 1)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_enable_ext0_wakeup(gpio_num_t gpio_num, int level);

/**
 * @brief Enable multiple external wake up sources (limited on ESP32-C3)
 * @param mask GPIO mask for external wake up
 * @param mode Wake up mode
 * @return ESP_ERR_NOT_SUPPORTED on ESP32-C3
 */
esp_err_t power_enable_ext1_wakeup(uint64_t mask, esp_sleep_ext1_wakeup_mode_t mode);

/**
 * @brief Disable wake up source
 * @param source Wake up source to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_disable_wakeup_source(wakeup_source_t source);

/**
 * @brief Get wake up reason
 * @return Wake up reason
 */
wakeup_reason_t power_get_wakeup_reason(void);

/**
 * @brief Get wake up GPIO number (for external wake up)
 * @return GPIO number that caused wake up, -1 if not external wake up
 */
int power_get_wakeup_gpio(void);

/**
 * @brief Power down peripherals before sleep
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_down_peripherals(void);

/**
 * @brief Power up peripherals after wake up
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_up_peripherals(void);

/**
 * @brief Set power configuration
 * @param config Pointer to power configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_set_config(const power_config_t *config);

/**
 * @brief Get power configuration
 * @param config Pointer to store power configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_get_config(power_config_t *config);

/**
 * @brief Reset power configuration to ESP32-C3 defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_reset_config(void);

/**
 * @brief Get sleep statistics
 * @param stats Pointer to store sleep statistics
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_get_sleep_stats(sleep_stats_t *stats);

/**
 * @brief Reset sleep statistics
 */
void power_reset_sleep_stats(void);

/**
 * @brief Check if wake up was from deep sleep
 * @return true if woke up from deep sleep, false otherwise
 */
bool power_is_wakeup_from_deep_sleep(void);

/**
 * @brief Get time spent in sleep (microseconds)
 * @return Time spent in last sleep cycle
 */
uint64_t power_get_sleep_time_us(void);

/**
 * @brief Get estimated battery life based on current consumption
 * @param battery_capacity_mah Battery capacity in mAh
 * @param current_consumption_ma Current consumption in mA
 * @return Estimated battery life in hours
 */
uint32_t power_estimate_battery_life(uint32_t battery_capacity_mah, float current_consumption_ma);

/**
 * @brief Calculate average power consumption for ESP32-C3
 * @param measurement_period_sec Measurement period in seconds
 * @return Average power consumption in mA
 */
float power_calculate_average_consumption(uint32_t measurement_period_sec);

/**
 * @brief Set RTC memory retention
 * @param retain_slow_mem Retain RTC slow memory
 * @param retain_fast_mem Retain RTC fast memory
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_set_rtc_memory_retention(bool retain_slow_mem, bool retain_fast_mem);

/**
 * @brief Prepare for sleep (save state, configure wake up sources)
 * @param sleep_time_sec Sleep time in seconds
 * @param sources Wake up sources to enable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_prepare_for_sleep(uint32_t sleep_time_sec, uint32_t sources);

/**
 * @brief Handle wake up (restore state, check wake up reason)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_handle_wakeup(void);

/**
 * @brief Enable/disable automatic light sleep
 * @param enable true to enable automatic light sleep, false to disable
 * @param min_freq_mhz Minimum CPU frequency for light sleep
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_set_automatic_light_sleep(bool enable, uint32_t min_freq_mhz);

/**
 * @brief Get current power consumption estimate for ESP32-C3
 * @return Estimated current consumption in mA
 */
float power_get_current_consumption_estimate(void);

/**
 * @brief Force immediate power saving mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_force_power_save(void);

/**
 * @brief Exit power saving mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_exit_power_save(void);

/**
 * @brief Check if device is in power saving mode
 * @return true if in power saving mode, false otherwise
 */
bool power_is_power_save_active(void);

/**
 * @brief Set wake up stub function
 * @param wake_stub Function to call immediately after wake up
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_set_wake_stub(esp_deep_sleep_wake_stub_fn_t wake_stub);

/**
 * @brief Print power configuration and statistics
 */
void power_print_info(void);

/**
 * @brief Get power mode string for debugging
 * @param mode Power mode
 * @param mode_str Buffer to store mode string
 * @param max_len Maximum length of mode string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_get_mode_string(power_mode_t mode, char *mode_str, size_t max_len);

/**
 * @brief Get wake up reason string for debugging
 * @param reason Wake up reason
 * @param reason_str Buffer to store reason string
 * @param max_len Maximum length of reason string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t power_get_wakeup_reason_string(wakeup_reason_t reason, char *reason_str, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif // POWER_MANAGEMENT_H