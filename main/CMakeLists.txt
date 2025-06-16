# main/CMakeLists.txt for SensDot

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        # Core ESP-IDF components
        esp_common
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
        
        # Sleep and power management
        esp_pm
        
        # ADC for battery monitoring
        esp_adc
        
        # I2C for sensors (if using esp-idf-lib)
        # i2cdev
        
    PRIV_REQUIRES
        # Private requirements
        esp_system
        esp_sleep
)