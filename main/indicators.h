/**
 * @file indicators.h
 * @brief LED and buzzer indication management
 */

#ifndef INDICATORS_H
#define INDICATORS_H

#include "sensdot_common.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#ifdef __cplusplus
extern "C" {
#endif

// Indication types
typedef enum {
    INDICATOR_LED,
    INDICATOR_BUZZER,
    INDICATOR_BOTH
} indicator_type_t;

// Indication patterns
typedef enum {
    PATTERN_OFF,
    PATTERN_ON,
    PATTERN_BLINK_SLOW,
    PATTERN_BLINK_FAST,
    PATTERN_PULSE,
    PATTERN_HEARTBEAT,
    PATTERN_SOS,
    PATTERN_CUSTOM
} indication_pattern_t;

// Indication priorities (higher number = higher priority)
typedef enum {
    PRIORITY_LOW = 1,
    PRIORITY_NORMAL = 2,
    PRIORITY_HIGH = 3,
    PRIORITY_CRITICAL = 4
} indication_priority_t;

// Custom pattern definition
typedef struct {
    uint16_t on_time_ms;
    uint16_t off_time_ms;
    uint8_t repeat_count;  // 0 = infinite
    bool fade_in;
    bool fade_out;
} custom_pattern_t;

// Indication sequence for complex patterns
typedef struct {
    indication_pattern_t pattern;
    uint16_t duration_ms;
    uint8_t repeat_count;
} indication_sequence_t;

// Current indication state
typedef struct {
    indication_pattern_t current_pattern;
    indicator_type_t active_indicators;
    indication_priority_t priority;
    uint32_t start_time;
    uint32_t duration_ms;
    bool is_active;
    bool override_mode;
} indication_state_t;

/**
 * @brief Initialize indicators (LED and buzzer)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_init(void);

/**
 * @brief Deinitialize indicators
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_deinit(void);

/**
 * @brief Turn LED on
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_on(void);

/**
 * @brief Turn LED off
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_off(void);

/**
 * @brief Toggle LED state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_toggle(void);

/**
 * @brief Set LED brightness (if PWM supported)
 * @param brightness Brightness level (0-255)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t led_set_brightness(uint8_t brightness);

/**
 * @brief Turn buzzer on
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buzzer_on(void);

/**
 * @brief Turn buzzer off
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buzzer_off(void);

/**
 * @brief Generate buzzer tone
 * @param frequency Frequency in Hz
 * @param duration_ms Duration in milliseconds
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buzzer_tone(uint16_t frequency, uint16_t duration_ms);

/**
 * @brief Generate buzzer beep
 * @param count Number of beeps
 * @param duration_ms Duration of each beep
 * @param interval_ms Interval between beeps
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t buzzer_beep(uint8_t count, uint16_t duration_ms, uint16_t interval_ms);

/**
 * @brief Start indication pattern
 * @param type Indicator type (LED, buzzer, or both)
 * @param pattern Indication pattern
 * @param priority Indication priority
 * @param duration_ms Duration in milliseconds (0 = infinite)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_start_pattern(indicator_type_t type, indication_pattern_t pattern, 
                                  indication_priority_t priority, uint32_t duration_ms);

/**
 * @brief Stop current indication
 * @param type Indicator type to stop
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_stop(indicator_type_t type);

/**
 * @brief Stop all indications
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_stop_all(void);

/**
 * @brief Set custom indication pattern
 * @param type Indicator type
 * @param pattern Custom pattern definition
 * @param priority Indication priority
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_set_custom_pattern(indicator_type_t type, const custom_pattern_t *pattern, 
                                       indication_priority_t priority);

/**
 * @brief Play indication sequence
 * @param type Indicator type
 * @param sequence Array of indication sequences
 * @param sequence_count Number of sequences
 * @param priority Indication priority
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_play_sequence(indicator_type_t type, const indication_sequence_t *sequence, 
                                  size_t sequence_count, indication_priority_t priority);

/**
 * @brief Indicate device status
 * @param status Status pattern to indicate
 * @param with_sound Include buzzer indication
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_status(status_pattern_t status, bool with_sound);

/**
 * @brief Indicate error condition
 * @param error_code Error code to indicate
 * @param critical true if critical error, false for warning
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_error(int error_code, bool critical);

/**
 * @brief Indicate low battery condition
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_low_battery(void);

/**
 * @brief Indicate setup mode
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_setup_mode(void);

/**
 * @brief Indicate WiFi connecting
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_wifi_connecting(void);

/**
 * @brief Indicate WiFi connected
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_wifi_connected(void);

/**
 * @brief Indicate MQTT connecting
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_mqtt_connecting(void);

/**
 * @brief Indicate MQTT connected
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_mqtt_connected(void);

/**
 * @brief Indicate data transmission
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_data_transmission(void);

/**
 * @brief Indicate factory reset
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_factory_reset(void);

/**
 * @brief Indicate configuration saved
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_show_config_saved(void);

/**
 * @brief Get current indication state
 * @param state Pointer to store current state
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_get_state(indication_state_t *state);

/**
 * @brief Check if indicators are active
 * @param type Indicator type to check
 * @return true if active, false otherwise
 */
bool indicators_is_active(indicator_type_t type);

/**
 * @brief Set indication override mode
 * @param enable true to enable override mode, false to disable
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_set_override_mode(bool enable);

/**
 * @brief Check if override mode is active
 * @return true if override mode active, false otherwise
 */
bool indicators_is_override_mode(void);

/**
 * @brief Set indication volume/intensity
 * @param volume Volume level (0-100)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_set_volume(uint8_t volume);

/**
 * @brief Get indication volume/intensity
 * @return Current volume level (0-100)
 */
uint8_t indicators_get_volume(void);

/**
 * @brief Mute all indications
 * @param mute true to mute, false to unmute
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_set_mute(bool mute);

/**
 * @brief Check if indications are muted
 * @return true if muted, false otherwise
 */
bool indicators_is_muted(void);

/**
 * @brief Test all indicators
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_test_all(void);

/**
 * @brief Test specific indicator
 * @param type Indicator type to test
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_test(indicator_type_t type);

/**
 * @brief Set indication callback for external events
 * @param callback Callback function
 * @param user_data User data to pass to callback
 * @return ESP_OK on success, error code otherwise
 */
typedef void (*indicators_callback_t)(indication_pattern_t pattern, indicator_type_t type, void *user_data);
esp_err_t indicators_set_callback(indicators_callback_t callback, void *user_data);

/**
 * @brief Get indication pattern name for debugging
 * @param pattern Indication pattern
 * @param pattern_name Buffer to store pattern name
 * @param max_len Maximum length of pattern name
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_get_pattern_name(indication_pattern_t pattern, char *pattern_name, size_t max_len);

/**
 * @brief Print indicators status for debugging
 */
void indicators_print_status(void);

/**
 * @brief Save indication configuration to NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_save_config(void);

/**
 * @brief Load indication configuration from NVS
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_load_config(void);

/**
 * @brief Reset indication configuration to defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t indicators_reset_config(void);

#ifdef __cplusplus
}
#endif

#endif // INDICATORS_H