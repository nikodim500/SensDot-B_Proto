# main/CMakeLists.txt for SensDot ESP32-C3 Super Mini

idf_component_register(
    SRCS 
        "main.c"
        "config.c"
        "sensors.c"
        "wifi_manager.c"
        "mqtt_client.c"
        "web_server.c"
        "power_management.c"
        "indicators.c"
    
    INCLUDE_DIRS 
        "."
        "../components"
    
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
        
        # ADC for battery monitoring (ESP32-C3)
        esp_adc
        
        # esp-idf-lib sensor components
        bme280
        bh1750
        i2cdev
        
    PRIV_REQUIRES
        # Private requirements
        esp_system
        esp_sleep
)