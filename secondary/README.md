# Secondary ESP32 - Communication Relay & Primary Sensor Module

## Overview
High-speed communication relay providing primary BMP390 altitude data, LoRa telemetry transmission, and deployment control for the CanSat mission. Optimized for 435MHz LoRa transmission at maximum data rate.

## Hardware Configuration

### Board
- **ESP32 DevKit** (30 pins) - Communication relay and sensor hub
- **Clock Speed**: 240MHz for maximum LoRa throughput
- **Memory**: 4MB Flash/520KB RAM

### Primary Sensors & Communication
- **BMP390**: I2C high-precision pressure/temperature/altitude sensor (PRIMARY altitude source)
- **LoRa SX1278**: SPI 435MHz transmission module (SF7/250kHz for speed)
- **Servo Motor**: GPIO25 deployment mechanism control
- **UART2**: 115200 baud Primary ESP32 communication (50ms sync cycle)

### Advanced Status System
- **Red LED (GPIO2)**: 
  - Solid ON during initialization (5 seconds)
  - Rapid blink on LoRa transmission failures
  - OFF during normal operation
- **Green LED (GPIO13)**:
  - Quick flash pattern (25ms ON/975ms OFF) during normal operation
  - Synchronized with Primary ESP32 green LED
  - OFF during communication failures or errors

## Pin Configuration & Connections

### LoRa SX1278 (Optimized SPI)
- **SCK**: GPIO18 (SPI Clock)
- **MISO**: GPIO19 (Master In Slave Out)
- **MOSI**: GPIO23 (Master Out Slave In)
- **SS**: GPIO5 (Slave Select)
- **RST**: GPIO4 (Reset)
- **DIO0**: GPIO26 (Digital I/O 0)
- **Frequency**: 435MHz (Competition frequency)
- **Spreading Factor**: SF7 (maximum speed)
- **Bandwidth**: 250kHz (high data rate)

### Critical Connections
- **BMP390**: I2C (SDA/SCL default pins) - Primary altitude sensor
- **Servo**: GPIO25 (PWM control for deployment mechanisms)
- **Primary UART**: GPIO16 (RX), GPIO17 (TX) @ 115200 baud
- **Status LEDs**: GPIO2 (Red), GPIO13 (Green) with synchronized patterns

## High-Performance Functionality

### 1. **Primary Altitude Sensing** 🎯
- **BMP390 Data**: High-precision pressure, temperature, altitude (50ms read cycle)
- **Primary Source**: Main altitude sensor for Primary ESP32 (backup: BMP280)
- **Failover Support**: Provides baseline altitude for BMP280 calibration during sensor failures
- **Precision**: ±0.5m altitude accuracy for critical flight state detection

### 2. **Ultra-Fast Communication** ⚡
- **50ms UART Sync**: Rapid bidirectional data exchange with Primary ESP32
- **Request/Response**: Efficient protocol for sensor data requests
- **CSV Relay**: Receives consolidated telemetry for LoRa transmission
- **Clean Logging**: Professional serial output with [INIT], [STATUS], [DATA] tags

### 3. **Optimized LoRa Transmission** 📡
- **435MHz Operation**: Competition frequency with SF7/250kHz for maximum speed
- **Async Transmission**: Non-blocking LoRa sends for 50ms cycle maintenance
- **CSV Visibility**: Logs transmitted data for debugging and verification
- **Ground Link**: Real-time telemetry relay to ground control station

### 4. **Deployment System Control** 🪂
- **Servo Control**: GPIO25 PWM for parachute/aerobrake deployment
- **Position Feedback**: Real-time servo status monitoring
- **Remote Commands**: Responds to Primary ESP32 deployment instructions
- **Safety Interlocks**: Prevents accidental deployment during ascent

### 5. **Synchronized Status System** 💡
- **LED Patterns**: Perfectly synchronized with Primary ESP32 indicators
- **System Health**: Visual confirmation of LoRa, sensor, and communication status
- **Error Indication**: Immediate visual feedback for subsystem failures
- **Professional Output**: Clean serial logging matching Primary format

## Performance Specifications & Data Flow

### System Performance
- **Sensor Read Cycle**: 50ms (BMP390, servo status, PID data)
- **UART Communication**: 115200 baud @ 50ms sync with Primary ESP32
- **LoRa Data Rate**: SF7/250kHz for maximum throughput (435MHz)
- **Loop Efficiency**: Optimized for real-time telemetry relay
- **Memory Usage**: Efficient buffer management for continuous operation

### Enhanced Data Flow
1. **High-Speed Sensing**: BMP390 pressure/temperature/altitude @ 50ms intervals
2. **UART Protocol**: Responds to Primary ESP32 data requests instantly
3. **Sensor Consolidation**: Packages BMP390, servo, and PID data for transmission
4. **CSV Reception**: Receives complete 23-field telemetry from Primary ESP32
5. **LoRa Relay**: Transmits consolidated CSV to ground station (435MHz)
6. **Status Logging**: Clean output showing transmitted data for verification

### Dependencies & Libraries
PlatformIO automatically installs:
- **Adafruit BMP3XX Library**: BMP390 high-precision sensor interface
- **LoRa Library**: SX1278 radio module control (435MHz optimized)
- **ESP32Servo**: Servo motor PWM control and feedback

### Building & Deployment
```bash
# Navigate to secondary directory
cd secondary

# Build optimized firmware
pio run

# Upload to Secondary ESP32
pio run --target upload

# Monitor real-time output
pio device monitor
```

### Error Handling & Recovery
- **LED Status**: Red (initialization/errors), Green (synchronized operation pattern)
- **LoRa Monitoring**: Error logging with retry mechanisms
- **Communication Timeout**: UART failsafe for Primary ESP32 synchronization
- **Sensor Validation**: BMP390 data integrity checks before transmission
- **Clean Output**: Professional logging format matching Primary ESP32 style
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
