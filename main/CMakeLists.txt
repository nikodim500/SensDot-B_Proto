# main/CMakeLists.txt for SensDot ESP32-C3

idf_component_register(
    SRCS "power_management.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        # Core ESP-IDF components
        esp_common
        esp_system          # Contains sleep functions
        esp_timer
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        esp_http_client
        nvs_flash
        driver
        spi_flash
        
        # MQTT client
        mqtt
        
        # JSON support
        json
        
        # Logging
        log
        
        # FreeRTOS
        freertos
        
        # Power management
        esp_pm
        
        # ADC for battery monitoring (updated for ESP-IDF 5.x)
        esp_adc
        
    PRIV_REQUIRES
        # Private requirements
        # Additional private components can be added here
)