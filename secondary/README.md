# Secondary ESP32 - CanSat Communication & Control Module

## Overview
This is the secondary flight computer for the CanSat mission, responsible for local sensor data collection, LoRa telemetry transmission, and deployment mechanism control.

## Hardware Configuration

### Board
- ESP32 DevKit (30 pins)

### Sensors & Peripherals
- **BMP390**: I2C (high-precision pressure/temperature sensor)
- **LoRa SX1278**: SPI communication module (433MHz)
- **Servo Motor**: GPIO25 (deployment mechanism control)
- **UART2**: Communication with Primary ESP32 (GPIO16-RX, GPIO17-TX)

### Status Indicators
- **LED D2 (GPIO2)**: 
  - ON during initialization
  - BLINK when receiving data from Primary ESP32
- **LED D13 (GPIO13)**:
  - ON if LoRa transmission fails
  - BLINK if no data received from Primary ESP32

## Pin Configuration

### LoRa SX1278 (SPI)
- SCK: GPIO18
- MISO: GPIO19
- MOSI: GPIO23
- SS: GPIO5
- RST: GPIO4
- DIO0: GPIO26
- Frequency: 433MHz

### Other Connections
- **BMP390**: I2C (default SDA/SCL pins)
- **Servo**: GPIO25
- **Primary UART**: GPIO16 (RX), GPIO17 (TX)
- **Status LEDs**: GPIO2, GPIO13

## Functionality

1. **Sensor Data Collection**
   - Reads BMP390 pressure, temperature, and altitude data
   - Monitors servo position and status
   - Collects PID control output data

2. **Inter-Board Communication**
   - Receives data requests from Primary ESP32 via UART2
   - Sends local sensor data to Primary ESP32
   - Receives consolidated CSV telemetry from Primary ESP32

3. **LoRa Transmission**
   - Transmits consolidated telemetry data to ground station
   - Operates on 433MHz frequency
   - Provides real-time mission data relay

4. **Deployment Control**
   - Controls servo motor for mechanism deployment
   - Provides position feedback to Primary ESP32

## Dependencies

The following libraries are automatically installed by PlatformIO:
- `Adafruit BMP3XX Library` - BMP390 sensor interface
- `LoRa` - LoRa communication module
- `ESP32Servo` - Servo motor control

## Building and Uploading

```bash
# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## Data Flow

1. Primary ESP32 requests sensor data via UART2
2. Secondary reads BMP390, servo status, and PID data
3. Secondary sends raw data to Primary via UART2
4. Secondary receives consolidated CSV telemetry from Primary
5. Secondary transmits CSV data via LoRa to ground station

## Error Handling

- LED indicators provide visual status feedback
- LoRa transmission errors are logged and indicated
- Communication timeouts with Primary ESP32 are handled gracefully
- Sensor initialization failures are detected and reported

## Configuration Notes

- LoRa frequency is set to 433MHz (configurable)
- UART communication runs at 115200 baud
- BMP390 I2C address: 0x77
- System operates at 1Hz data collection cycle

---

**Part of**: CanSat Mission Control System  
**Competition**: IN-SPACe CANSAT India Student Competition 2024-25 (2nd Edition)
