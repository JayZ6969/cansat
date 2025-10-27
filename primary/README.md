# Primary ESP32 - CanSat Mission Control

## Overview

Primary flight computer for the CanSat mission, serving as the main mission controller responsible for sensor data collection, GPS tracking, SD logging, flight state management, and system coordination with advanced recovery features.

## Hardware Configuration

### Board

- ESP32 DevKit (30 pins)

### Sensors & Peripherals

- **GPS L89**: UART2 (GPIO16-RX, GPIO17-TX) - Position tracking and navigation
- **BMP280**: I2C (GPIO21-SDA, GPIO22-SCL) - Backup pressure/altitude sensor with failover capability
- **MPU6050**: I2C (GPIO21-SDA, GPIO22-SCL) - 6-axis accelerometer and gyroscope
- **SD Card Module**: SPI (GPIO23-MOSI, GPIO19-MISO, GPIO18-SCK, GPIO5-CS) - Mission data logging
- **Servo**: GPIO27 - Parachute deployment mechanism

### Indicators & Output

- **Buzzer**: GPIO2 - Audio alerts and recovery beacon
- **Status LEDs** (Advanced Pattern System):
  - **D12 (RED)**: Boot state / SD card errors
  - **D13 (YELLOW)**: GPS lock status (blink when no GPS)
  - **D14 (GREEN)**: All systems OK (quick flash pattern)

### Communication

- **UART0**: Primary ↔ Secondary ESP32 communication at 115200 baud
- **UART2**: GPS L89 communication at 9600 baud

## Pin Configuration

### GPS NEO-6M (UART2)

- RX: GPIO16
- TX: GPIO17
- Baud Rate: 9600

### I2C Sensors (BMP280 & MPU6050)

- SDA: GPIO21
- SCL: GPIO22
- Pull-up resistors: 4.7kΩ recommended

### SD Card Module (SPI)

- MOSI: GPIO23
- MISO: GPIO19
- SCK: GPIO18
- CS: GPIO5

### Status Indicators

- Red LED: GPIO12
- Yellow LED: GPIO13
- Green LED: GPIO14
- Buzzer: GPIO0

### Analog Input

- Battery Monitor: GPIO36 (ADC1_CH0)

## Advanced Features

### 1. **Dual Sensor Failover System** 🛡️

- **Primary Altitude**: BMP390 on Secondary ESP32 (higher accuracy)
- **Backup Altitude**: BMP280 on Primary ESP32 (automatic failover)
- **CSV-based Calibration**: Reads last altitude from telemetry.csv to calibrate BMP280 during failover
- **Seamless Switching**: Altitude continuity maintained during sensor failures

### 2. **Flight State Recovery** 🔄

- **Power Failure Protection**: Reads last flight state from CSV on every boot
- **Mission Continuity**: Resumes from last known state after power loss/reset
- **Recovery Logging**: Documents all power reset events in telemetry
- **State Validation**: Handles mid-flight recovery correctly for all states

### 3. **High-Speed Data Collection** ⚡

- **10Hz Telemetry**: 100ms data collection interval (optimized from 1Hz)
- **50ms UART Sync**: Fast communication with Secondary ESP32
- **1ms Loop Delay**: Maximum responsiveness
- **Async Operations**: Non-blocking sensor reads and SD writes

### 4. **Flight State Management**

Advanced state machine with recovery support:

- **BOOT (0)**: System initialization and sensor checks
- **TEST_MODE (1)**: Pre-flight testing and calibration
- **LAUNCH_PAD (2)**: Ready for launch, waiting for takeoff detection (>50m)
- **ASCENT (3)**: Active flight phase, monitoring altitude gain
- **ROCKET_DEPLOY (4)**: Parachute deployment at apogee (>700m)
- **DESCENT (5)**: Descent phase monitoring
- **AEROBRAKE_RELEASE (6)**: Aerobrake deployment (<50% max altitude)
- **IMPACT (7)**: Landed, recovery beeper active

### 5. **Smart Communication Protocol**

- **Request/Response**: Efficient data exchange with Secondary ESP32
- **Timeout Handling**: Robust error recovery mechanisms
- **CSV Relay**: Consolidated telemetry transmission to Secondary for LoRa
- **Error Tracking**: Comprehensive subsystem error monitoring

## Data Format & Recovery Systems

### CSV Telemetry Structure (23 Fields)

The system generates comprehensive telemetry in CSV format with the following structure:

```
TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GNSS_TIME,GNSS_LAT,
GNSS_LONG,GNSS_ALT,GNSS_SATS,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,
GYRO_SPIN_RATE,FLIGHT_STATE,SERVO_STATUS,ERROR_CODE,GNSS_SPEED
```

### Field Specifications with Units

| Field # | Name               | Unit               | Description                               | Source                              |
| ------- | ------------------ | ------------------ | ----------------------------------------- | ----------------------------------- |
| 1       | **TEAM_ID**        | String             | Team identifier                           | "2024-ASI-CANSAT-049"               |
| 2       | **TIMESTAMP**      | ms                 | Mission time since boot                   | millis()                            |
| 3       | **PACKET_COUNT**   | count              | Sequential packet counter                 | Increments each transmission        |
| 4       | **ALTITUDE**       | m                  | Altitude above baseline                   | BMP390 (primary) or BMP280 (backup) |
| 5       | **PRESSURE**       | hPa                | Atmospheric pressure                      | BMP390 or BMP280                    |
| 6       | **TEMP**           | °C                 | Atmospheric temperature                   | BMP390 or BMP280                    |
| 7       | **VOLTAGE**        | V                  | Battery voltage                           | Secondary ESP32 ADC                 |
| 8       | **GNSS_TIME**      | HH:MM:SS           | GPS UTC time                              | GPS L89                             |
| 9       | **GNSS_LAT**       | degrees            | Latitude (decimal)                        | GPS L89                             |
| 10      | **GNSS_LONG**      | degrees            | Longitude (decimal)                       | GPS L89                             |
| 11      | **GNSS_ALT**       | m                  | GPS altitude (MSL)                        | GPS L89                             |
| 12      | **GNSS_SATS**      | count              | Number of satellites                      | GPS L89                             |
| 13      | **ACCEL_X**        | g                  | X-axis acceleration                       | MPU6050                             |
| 14      | **ACCEL_Y**        | g                  | Y-axis acceleration                       | MPU6050                             |
| 15      | **ACCEL_Z**        | g                  | Z-axis acceleration                       | MPU6050                             |
| 16      | **GYRO_X**         | °/s                | X-axis rotation rate                      | MPU6050                             |
| 17      | **GYRO_Y**         | °/s                | Y-axis rotation rate                      | MPU6050                             |
| 18      | **GYRO_Z**         | °/s                | Z-axis rotation rate                      | MPU6050                             |
| 19      | **GYRO_SPIN_RATE** | PWM (-255 to +255) | Reaction wheel PID output (motor control) | PID controller                      |
| 20      | **FLIGHT_STATE**   | enum               | Current flight state (0-7)                | State machine                       |
| 21      | **SERVO_STATUS**   | 0/1                | Parachute servo (0=closed, 1=open)        | Servo control                       |
| 22      | **ERROR_CODE**     | String             | Concatenated error codes                  | Error monitoring                    |
| 23      | **GNSS_SPEED**     | m/s                | Speed from acceleration                   | MPU6050 integration                 |

### Flight State Values

- **0**: BOOT - System initialization
- **1**: TEST_MODE - Pre-flight testing
- **2**: LAUNCH_PAD - Ready for launch
- **3**: ASCENT - Active flight phase
- **4**: ROCKET_DEPLOY - Parachute deployment
- **5**: DESCENT - Descending phase
- **6**: AEROBRAKE_RELEASE - Aerobrake deployed
- **7**: IMPACT - Landed state

### Recovery Data Usage

- **telemetry.csv**: Mission continuity database
- **Flight State Recovery**: Reads last state on boot after power failure
- **Altitude Calibration**: Uses last CSV altitude to calibrate BMP280 during sensor failover
- **Variable Recovery**: Restores packet count, altitude index, max altitude, and servo status
- **Example Row**:
  ```
  2024-ASI-CANSAT-049,125430,1254,1250.45,850.23,22.5,7.2,10:34:22,40.712800,-74.006000,1248.70,8,0.05,-0.12,1.02,5.2,-2.1,45.3,12.5,3,0,0,15.2
  ```

## Dependencies

The following libraries are automatically installed by PlatformIO:

- `Adafruit Unified Sensor` - Common sensor interface
- `Adafruit BMP280 Library` - BMP280 pressure sensor
- `Adafruit MPU6050` - MPU6050 accelerometer/gyroscope
- `ESP32Servo` - Servo motor control library
- `TinyGPSPlus` - GPS parsing and utilities

## Building and Uploading

```bash
# Navigate to primary directory
cd primary

# Build the project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial output
pio device monitor
```

## System Performance & Error Handling

### Performance Specifications

- **Data Rate**: 10Hz telemetry (100ms intervals)
- **UART Communication**: 50ms Secondary ESP32 sync
- **Loop Responsiveness**: 1ms delay for maximum efficiency
- **SD Card**: Continuous CSV logging with error recovery
- **Memory**: Optimized for 4MB Flash/520KB RAM ESP32

### Error Code System

Concatenated error monitoring for subsystem failures:

- **1**: MPU6050 accelerometer/gyroscope failure
- **2**: BMP280 backup pressure sensor failure
- **3**: SD card read/write failure
- **5**: GPS L89 communication failure
- **6**: PID control system failure
- **7**: Camera/imaging system failure
- **8**: LoRa communication failure (via Secondary)
- **9**: BMP390 primary sensor failure (triggers failover)

### System Initialization with Recovery

1. **Power-On Recovery**

   - Read last flight state from telemetry.csv
   - Resume mission from last known state
   - Handle mid-flight power reset gracefully

2. **Dual Sensor Initialization**

   - Primary: BMP390 via Secondary ESP32 (higher accuracy)
   - Backup: Local BMP280 with CSV calibration failover
   - MPU6050 accelerometer/gyroscope initialization
   - GPS L89 module configuration

3. **Communication Setup**

   - UART2 @ 115200 baud to Secondary ESP32
   - I2C sensors (MPU6050, BMP280)
   - SD card SPI interface
   - LED status indicators

4. **Mission Readiness**
   - Enter appropriate flight state based on recovery
   - Begin 10Hz data collection cycle
   - Activate LED status patterns

### Advanced LED Status System

Synchronized with Secondary ESP32 patterns:

- **Red LED (GPIO12)**:
  - Solid ON during BOOT state (5 seconds)
  - Rapid blink on critical system errors
  - OFF during normal operation
- **Yellow LED (GPIO13)**:
  - ON when GPS lock not acquired (<4 satellites)
  - Slow blink during GPS acquisition
  - OFF with valid GPS fix (≥4 satellites)
- **Green LED (GPIO14)**:
  - Quick flash pattern (25ms ON/975ms OFF) during normal operation
  - Matches Secondary ESP32 green LED timing
  - OFF during system errors

### Recovery & Troubleshooting

1. **Altitude Sensor Failover**

   - Automatic BMP390→BMP280 switching on sensor failure
   - CSV-based calibration maintains altitude continuity
   - Error code 10 logged for BMP390 failures

2. **Flight State Recovery**

   - Power failure mid-flight: System reads last state from CSV
   - Resumes appropriate flight mode automatically
   - Prevents mission abort due to power glitches

3. **SD Card Issues**

   - Check FAT32 formatting and 32GB maximum size
   - Verify SPI wiring (MOSI/MISO/SCK/CS pins)
   - Monitor error code 3 for write failures

4. **GPS Lock Problems**

   - Yellow LED indicates acquisition status
   - Requires clear sky view for satellite lock
   - Error code 5 for communication failures

5. **GPS Not Acquiring Lock**

   - Ensure clear view of sky
   - Check antenna connection
   - Allow sufficient time for cold start (up to 5 minutes)

6. **Sensor Initialization Errors**

   - Verify I2C connections and pull-up resistors
   - Check sensor power supply (3.3V)
   - Ensure proper grounding

7. **Communication Timeout with Secondary**
   - Check UART wiring between boards
   - Verify both boards are powered and running
   - Check baud rate settings (115200)

## Performance Specifications

- **Data Collection Rate**: 1 Hz (configurable)
- **GPS Update Rate**: 1 Hz
- **SD Card Write Speed**: ~10ms per record
- **Power Consumption**: ~200mA @ 3.3V (typical)
- **Operating Temperature**: -20°C to +70°C
- **Altitude Range**: 0 to 30,000 meters

## Configuration Notes

- System operates on 1 Hz data collection cycle
- GPS baud rate: 9600
- I2C frequency: 100kHz (standard mode)
- SPI frequency: 4MHz for SD card
- UART2 communication: 115200 baud
- ADC resolution: 12-bit for battery monitoring

---

**Part of**: CanSat Mission Control System  
**Competition**: IN-SPACe CANSAT India Student Competition 2024-25 (2nd Edition)  
**Team**: AVINYA
