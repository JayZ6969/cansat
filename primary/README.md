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

### Enhanced CSV Telemetry Structure (23 Fields)
The system generates comprehensive telemetry in standardized CSV format:
```
1013,HH:MM:SS.sss,PACKET,MODE,STATE,ALTITUDE,AIRSPEED,HS_DEPLOYED,PC_DEPLOYED,
MAST_RAISED,TEMP,VOLTAGE,PRESSURE,GPS_TIME,GPS_ALT,GPS_LAT,GPS_LONG,GPS_SATS,
TILT_X,TILT_Y,ROT_Z,CMD_ECHO,ERROR_CODES
```

### Field Specifications
- **TEAM_ID**: "1013" (Competition identifier)
- **MISSION_TIME**: HH:MM:SS.sss with milliseconds
- **PACKET_COUNT**: Sequential transmission counter
- **MODE**: Flight state (0=BOOT, 1=TEST, 2=LAUNCH_PAD, 3=ASCENT, 4=ROCKET_DEPLOY, 5=DESCENT, 6=AEROBRAKE_RELEASE, 7=IMPACT)
- **STATE**: System status descriptor
- **ALTITUDE**: Current altitude (meters) - BMP390 primary, BMP280 backup with CSV calibration
- **AIR_SPEED**: Calculated airspeed (m/s)
- **HS_DEPLOYED**: Heat shield status ("P"=deployed, "N"=not deployed)
- **PC_DEPLOYED**: Parachute status ("P"=deployed, "N"=not deployed)
- **MAST_RAISED**: Mast deployment ("P"=raised, "N"=not raised)
- **TEMPERATURE**: Atmospheric temperature (°C)
- **VOLTAGE**: Battery voltage (V)
- **PRESSURE**: Atmospheric pressure (hPa)
- **GPS_TIME**: UTC timestamp (HH:MM:SS)
- **GPS_ALTITUDE**: GPS altitude (meters)
- **GPS_LATITUDE**: Latitude (decimal degrees)
- **GPS_LONGITUDE**: Longitude (decimal degrees)
- **GPS_SATELLITES**: Satellite count
- **TILT_X/Y**: Tilt angles (degrees)
- **ROTATION_Z**: Z-axis angular velocity (°/s)
- **CMD_ECHO**: Ground command acknowledgment
- **ERROR_CODES**: Concatenated system error status

### Recovery Data Usage
- **telemetry.csv**: Mission continuity database
- **Flight State Recovery**: Reads last state on boot after power failure
- **Altitude Calibration**: Uses last CSV altitude to calibrate BMP280 during sensor failover
- **Example**: `1013,10:34:23.456,1254,3,ASCENT,1250.4,15.2,N,N,N,22.5,7.2,1013.2,10:34:22,1248.7,40.7128,-74.0060,8,5.2,-2.1,45.3,NONE,`

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
- **8**: Serial communication failure (UART2)
- **9**: LoRa communication failure (via Secondary)
- **10**: BMP390 primary sensor failure (triggers failover)

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

2. **GPS Not Acquiring Lock**
   - Ensure clear view of sky
   - Check antenna connection
   - Allow sufficient time for cold start (up to 5 minutes)

3. **Sensor Initialization Errors**
   - Verify I2C connections and pull-up resistors
   - Check sensor power supply (3.3V)
   - Ensure proper grounding

4. **Communication Timeout with Secondary**
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
