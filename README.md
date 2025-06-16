# SensDot - IoT Sensor Device

Energy-efficient ESP32-based sensor device with wireless configuration and MQTT reporting.

## Features

- **Sensors**: BME280 (temperature, humidity, pressure), BH1750 (light), PIR (motion)
- **Wireless Configuration**: Web-based setup interface via WiFi AP
- **Energy Efficient**: Deep sleep mode with configurable wake intervals
- **Home Assistant**: Automatic MQTT discovery integration
- **Battery Monitoring**: Low battery detection with audio/visual alerts
- **OTA Ready**: Firmware update capability

## Hardware Requirements

- ESP32 (ESP32-S supported)
- BME280 sensor (I2C)
- BH1750 sensor (I2C) 
- PIR motion sensor
- LED indicator
- Buzzer
- Battery voltage divider circuit

## GPIO Configuration

| GPIO | Function |
|------|----------|
| 4    | PIR sensor input |
| 2    | LED indicator |
| 17   | Buzzer output |
| 18   | Sensor power control |
| 34   | Battery voltage (ADC) |
| 21   | I2C SDA |
| 22   | I2C SCL |
| 0    | Factory reset button (BOOT) |

## Setup Instructions

1. **First Boot**: Device creates WiFi AP "SensDot-Setup-XXXXXX"
2. **Connect**: Join AP and open http://192.168.4.1
3. **Configure**: Set WiFi credentials and MQTT settings
4. **Deploy**: Device restarts and begins normal operation

## Configuration

Use `idf.py menuconfig` and navigate to "SensDot Configuration" for:
- Wake intervals
- Battery thresholds  
- Setup timeouts
- MQTT discovery intervals

## Building

```bash
idf.py build
idf.py flash monitor