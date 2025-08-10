# main/CMakeLists.txt for SensDot ESP32-C3 (ESP-IDF 5.5)

idf_component_register(
    SRCS "config.c" "indicators.c" "main.c" "mqtt_client.c" "power_management.c" "sensors.c" "web_server.c" "wifi_manager.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_common
        esp_system
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        esp_http_client
        nvs_flash
        esp_pm
        esp_adc
        driver
        freertos
        log
        # External components
        i2cdev
        bmp280
        bh1750
)
