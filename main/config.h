/**
 * @file config.h
 * @brief Configuration management for SensDot device (ESP32-C3)
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "sensdot_common.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize configuration system
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_init(void);

/**
 * @brief Load configuration from NVS storage
 * @param config Pointer to configuration structure to fill
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_load(device_config_t *config);

/**
 * @brief Save configuration to NVS storage
 * @param config Pointer to configuration structure to save
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_save(const device_config_t *config);

/**
 * @brief Initialize configuration with default values
 * @param config Pointer to configuration structure to initialize
 */
void config_init_defaults(device_config_t *config);

/**
 * @brief Validate configuration parameters
 * @param config Pointer to configuration structure to validate
 * @return ESP_OK if valid, error code otherwise
 */
esp_err_t config_validate(const device_config_t *config);

/**
 * @brief Reset configuration to factory defaults
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_factory_reset(void);

/**
 * @brief Check if device is configured
 * @return true if configured, false otherwise
 */
bool config_is_configured(void);

/**
 * @brief Get current configuration
 * @return Pointer to current configuration (read-only)
 */
const device_config_t* config_get_current(void);

/**
 * @brief Update specific configuration parameter (string)
 * @param key Configuration key to update
 * @param value New string value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_set_string(const char *key, const char *value);

/**
 * @brief Update specific configuration parameter (integer)
 * @param key Configuration key to update
 * @param value New integer value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_set_int(const char *key, int value);

/**
 * @brief Update specific configuration parameter (float)
 * @param key Configuration key to update
 * @param value New float value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_set_float(const char *key, float value);

/**
 * @brief Update specific configuration parameter (boolean)
 * @param key Configuration key to update
 * @param value New boolean value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_set_bool(const char *key, bool value);

/**
 * @brief Get specific configuration parameter (string)
 * @param key Configuration key to get
 * @param value Buffer to store the string value
 * @param max_len Maximum length of value buffer
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_get_string(const char *key, char *value, size_t max_len);

/**
 * @brief Get specific configuration parameter (integer)
 * @param key Configuration key to get
 * @param value Pointer to store the integer value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_get_int(const char *key, int *value);

/**
 * @brief Get specific configuration parameter (float)
 * @param key Configuration key to get
 * @param value Pointer to store the float value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_get_float(const char *key, float *value);

/**
 * @brief Get specific configuration parameter (boolean)
 * @param key Configuration key to get
 * @param value Pointer to store the boolean value
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_get_bool(const char *key, bool *value);

/**
 * @brief Print current configuration (for debugging)
 */
void config_print(void);

/**
 * @brief Export configuration as JSON string
 * @param json_str Buffer to store JSON string
 * @param max_len Maximum length of JSON string
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_export_json(char *json_str, size_t max_len);

/**
 * @brief Import configuration from JSON string
 * @param json_str JSON string containing configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t config_import_json(const char *json_str);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H