# Primary ESP32 - CanSat Mission Data Controller

## Overview
This is the primary flight computer for the CanSat mission, serving as the main data controller responsible for sensor data collection, flight state management, data logging, and system coordination.

## Hardware Configuration

### Board
- ESP32 DevKit (30 pins)

### Sensors & Peripherals
- **GPS NEO-6M**: UART2 (GPIO16-RX, GPIO17-TX) - Position tracking and navigation
- **BMP280**: I2C (GPIO21-SDA, GPIO22-SCL) - Pressure, temperature, and altitude sensing
- **MPU6050**: I2C (GPIO21-SDA, GPIO22-SCL) - 6-axis accelerometer and gyroscope
- **SD Card Module**: SPI (GPIO23-MOSI, GPIO19-MISO, GPIO18-SCK, GPIO5-CS) - Data logging
- **Battery Monitor**: GPIO36 (VP pin) - ADC voltage monitoring

### Indicators & Output
- **Buzzer**: GPIO0 (Boot pin via transistor, active HIGH) - Audio alerts
- **Status LEDs**:
  - **D12 (RED)**: Boot state indicator
  - **D13 (YELLOW)**: GPS lock status (ON = no GPS lock)
  - **D14 (GREEN)**: System OK + blinks during data operations

### Communication
- **UART0**: Debug/programming interface
- **UART2**: GPS communication and Secondary ESP32 data exchange

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

## Functionality

### 1. **Sensor Data Collection** (1Hz cycle)
- **GPS Tracking**: Position, altitude, time, satellite count
- **Environmental Monitoring**: Atmospheric pressure, temperature, calculated altitude
- **Motion Detection**: 3-axis acceleration and gyroscope data
- **System Health**: Battery voltage monitoring
- **Spin Rate Calculation**: Derived from gyroscope data

### 2. **Data Consolidation**
- Requests additional sensor data from Secondary ESP32 via UART2
- Receives BMP390, servo status, and PID control data
- Consolidates all telemetry into standardized CSV format
- Implements data validation and error checking

### 3. **Data Storage**
- Logs all telemetry data to SD card in CSV format
- Creates timestamped log files
- Implements robust file handling with error recovery
- Supports continuous logging throughout mission

### 4. **Flight State Management**
The system tracks and manages the following flight states:

- **BOOT (0)**: System initialization and sensor checks (5 seconds)
- **TEST_MODE (1)**: Pre-flight testing and calibration
- **LAUNCH_PAD (2)**: Ready for launch, waiting for takeoff detection
- **ASCENDING (3)**: Active flight phase, monitoring altitude gain
- **DESCENDING (4)**: Descent phase, parachute deployment monitoring
- **LANDED (5)**: Post-landing data collection and recovery mode

### 5. **Communication Protocol**
- Sends data requests to Secondary ESP32
- Receives consolidated responses
- Transmits complete CSV telemetry back to Secondary for LoRa transmission
- Implements timeout handling and retry mechanisms

## Data Format

The system generates telemetry data in the following CSV format:
```
TEAM_ID, TIMESTAMP, PACKET_COUNT, ALTITUDE, PRESSURE, TEMP, VOLTAGE,
GNSS_TIME, GNSS_LAT, GNSS_LONG, GNSS_ALT, GNSS_SATS,
ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z,
GYRO_SPIN_RATE, FLIGHT_STATE, OPTIONAL_DATA
```

### Field Descriptions
- **TEAM_ID**: Unique team identifier
- **TIMESTAMP**: Mission elapsed time (seconds)
- **PACKET_COUNT**: Sequential packet number
- **ALTITUDE**: Calculated altitude (meters)
- **PRESSURE**: Atmospheric pressure (hPa)
- **TEMP**: Temperature (°C)
- **VOLTAGE**: Battery voltage (V)
- **GNSS_TIME**: GPS timestamp
- **GNSS_LAT/LONG**: GPS coordinates (decimal degrees)
- **GNSS_ALT**: GPS altitude (meters)
- **GNSS_SATS**: Number of satellites in view
- **ACCEL_X/Y/Z**: Acceleration data (m/s²)
- **GYRO_X/Y/Z**: Angular velocity (°/s)
- **GYRO_SPIN_RATE**: Calculated spin rate (°/s)
- **FLIGHT_STATE**: Current flight state (0-5)
- **OPTIONAL_DATA**: Additional mission-specific data

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

## System Initialization Sequence

1. **Hardware Initialization**
   - Initialize serial communications
   - Configure GPIO pins and LED indicators
   - Initialize I2C bus for sensors

2. **Sensor Setup**
   - Initialize and calibrate BMP280
   - Initialize and calibrate MPU6050
   - Configure GPS module
   - Initialize SD card system

3. **System Checks**
   - Verify sensor connectivity
   - Check SD card availability
   - Test communication with Secondary ESP32
   - Perform initial calibrations

4. **Mission Ready**
   - Enter TEST_MODE for pre-flight checks
   - Await launch detection
   - Begin data logging cycle

## Error Handling

### LED Status Indicators
- **Red LED (GPIO12)**: 
  - Solid ON during BOOT state
  - Blinks on critical system errors
- **Yellow LED (GPIO13)**:
  - ON when GPS lock is not acquired
  - OFF when GPS has valid fix
- **Green LED (GPIO14)**:
  - ON when all systems operational
  - Blinks during data read/write operations

### Common Issues & Solutions

1. **SD Card Initialization Failed**
   - Check SD card formatting (FAT32 required)
   - Verify SPI wiring connections
   - Ensure adequate power supply

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
