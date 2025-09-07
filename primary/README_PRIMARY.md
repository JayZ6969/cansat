# PRIMARY ESP32 - CanSat Mission Control System

## Overview
This is a complete, production-ready Arduino C++ code for the PRIMARY ESP32 in a CanSat mission, built for PlatformIO with ESP32 DevKit (30 pins).

## Hardware Configuration

### Sensors & Peripherals
- **GPS NEO-6M**: UART2 (GPIO16-RX, GPIO17-TX) - Position tracking
- **BMP280**: I2C (GPIO21-SDA, GPIO22-SCL) - Fallback altitude/pressure sensor  
- **MPU6050**: I2C (GPIO21-SDA, GPIO22-SCL) - Accelerometer & gyroscope
- **SD Card**: SPI (GPIO23-MOSI, GPIO19-MISO, GPIO18-SCK, GPIO5-CS) - Data logging
- **Battery Monitor**: GPIO36 (VP pin) - ADC voltage monitoring

### Indicators & Output
- **Buzzer**: GPIO0 (Boot pin via transistor, active HIGH)
- **LEDs**:
  - D12 (RED): Boot state indicator
  - D13 (YELLOW): GPS not locked
  - D14 (GREEN): All sensors OK + blinks during data operations

### Communication
- **UART0**: Communication with Secondary ESP32
- **UART2**: GPS communication

## Data Flow Architecture

### 1. Data Collection Cycle (1Hz)
- Reads onboard sensors (GPS, BMP280, MPU6050, battery)
- Requests BMP390 + servo + PID data from Secondary ESP32
- Consolidates all telemetry into CSV format

### 2. CSV Data Format
```
TEAM_ID, TIMESTAMP, PACKET_COUNT, ALTITUDE, PRESSURE, TEMP, VOLTAGE,
GNSS_TIME, GNSS_LAT, GNSS_LONG, GNSS_ALT, GNSS_SATS,
ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z,
GYRO_SPIN_RATE, FLIGHT_STATE, OPTIONAL_DATA
```

### 3. Data Storage & Transmission
- Writes each CSV row to SD card (`/telemetry.csv`)
- Sends consolidated data back to Secondary ESP32 via UART0

## Flight State Management

### States
- **0 = BOOT**: Initialization phase (5 seconds)
- **1 = TEST_MODE**: System check and calibration
- **2 = LAUNCH_PAD**: Ready for launch, waiting for takeoff
- **3 = ASCENT**: Rising altitude detected
- **4 = ROCKET_DEPLOY**: Apogee reached, deploying from rocket
- **5 = DESCENT**: Falling phase
- **6 = AEROBRAKE_RELEASE**: Mid-descent aerobrake deployment
- **7 = IMPACT**: Landing detected

### State Transition Logic
- Automatic transitions based on altitude, acceleration, and time
- Tracks maximum altitude for apogee detection
- Uses sensor fusion for reliable state determination

## LED & Buzzer Control Logic

### LEDs
- **RED (D12)**: ON during BOOT state
- **YELLOW (D13)**: ON while GPS has no satellite lock
- **GREEN (D14)**: 
  - Steady ON when all sensors are operational
  - BLINKS (100ms) during data write/transmission operations

### Buzzer
- Mirrors LED states during boot and GPS acquisition
- **Beacon Mode**: When altitude < 50m after apogee detection
  - 500ms ON, 1500ms OFF pattern for location assistance

## Key Features

### 1. Robust Error Handling
- Graceful sensor failure handling with timeouts and retries
- Fallback from BMP390 (Secondary) to BMP280 (Primary) for altitude
- UART communication error recovery

### 2. Non-blocking Operation
- Uses `millis()` for all timing operations
- Continuous GPS reading without blocking main loop
- Asynchronous LED blinking and buzzer control

### 3. Power Management
- WiFi disabled for power conservation
- Optimized sensor reading intervals
- Efficient I2C and SPI communication

### 4. Data Integrity
- Packet counter for data sequence verification
- CSV header creation on first run
- Data validation before transmission

## UART Communication Protocol

### Primary → Secondary (Request)
```
"REQ_DATA\n"
```

### Secondary → Primary (Response)
```
"DATA:ALT:123.45,PRESS:1013.25,TEMP:25.5,SERVO:OK,PID:0.75\n"
```

### Primary → Secondary (Consolidated Data)
```
"CSV:[complete_csv_row]\n"
```

## Critical Safety Features

1. **Dual Altitude Sources**: BMP390 (primary) with BMP280 fallback
2. **GPS Lock Monitoring**: Visual indication of satellite acquisition
3. **System Health Monitoring**: Continuous sensor status checking
4. **Recovery Beacon**: Automatic activation below 50m after apogee
5. **Data Redundancy**: Both SD card storage and UART transmission

## Dependencies (PlatformIO)
```ini
lib_deps = 
    adafruit/Adafruit Unified Sensor@^1.1.14
    adafruit/Adafruit BMP280 Library@^2.6.8
    adafruit/Adafruit MPU6050@^2.2.6
    madhephaestus/ESP32Servo@^3.0.5
    mikalhart/TinyGPSPlus@^1.1.0
```

## Performance Characteristics
- **Data Rate**: 1 Hz telemetry collection
- **UART Rate**: 800ms secondary data requests
- **GPS Update**: Continuous processing
- **State Update**: 500ms flight state evaluation
- **Memory Usage**: Optimized for ESP32 constraints

## Build Status
✅ **Compilation**: SUCCESS - No errors or warnings
✅ **Dependencies**: All libraries resolved
✅ **Hardware Support**: ESP32 DevKit compatible
✅ **Production Ready**: Full feature implementation

This code provides a complete, robust solution for CanSat primary mission control with professional-grade error handling, state management, and data integrity features.
