# CanSat Mission Control System

## IN-SPACe CANSAT India Student Competition 2024-25 (2nd Edition)

This repository contains the complete flight software for a dual ESP32-based CanSat mission system designed for the IN-SPACe CANSAT India Student Competition 2024-25.

## System Architecture

The CanSat consists of two ESP32 DevKit boards working in tandem:

### 🚀 **Primary ESP32** - Mission Data Controller
- **Role**: Main flight computer handling sensors, data logging, and flight state management
- **Location**: `/primary/` directory
- **Key Functions**:
  - GPS tracking and navigation
  - Altitude and pressure monitoring (BMP280)
  - Motion detection (MPU6050 accelerometer/gyroscope)
  - SD card data logging
  - Battery voltage monitoring
  - Flight state management
  - UART communication coordination

### 📡 **Secondary ESP32** - Communication & Control
- **Role**: Local sensor data collection and LoRa telemetry transmission
- **Location**: `/secondary/` directory
- **Key Functions**:
  - High-precision altitude sensing (BMP390)
  - LoRa long-range data transmission
  - Servo control for deployment mechanisms
  - Real-time telemetry relay
  - Backup sensor data collection

## Hardware Requirements

### Primary ESP32 Components
- ESP32 DevKit (30 pins)
- GPS NEO-6M module
- BMP280 pressure/temperature sensor
- MPU6050 accelerometer/gyroscope
- MicroSD card module
- Buzzer and LED indicators
- Battery voltage monitoring circuit

### Secondary ESP32 Components
- ESP32 DevKit (30 pins)
- LoRa SX1278 433MHz module
- BMP390 high-precision pressure sensor
- Servo motor for deployment
- LED status indicators

## Data Flow

1. **Primary ESP32** collects onboard sensor data (GPS, BMP280, MPU6050, battery)
2. **Primary** requests additional sensor data from **Secondary** via UART
3. **Secondary** responds with BMP390, servo status, and PID data
4. **Primary** consolidates all data into CSV format and logs to SD card
5. **Primary** sends consolidated telemetry back to **Secondary**
6. **Secondary** transmits telemetry via LoRa to ground station

## Getting Started

### Prerequisites
- [PlatformIO IDE](https://platformio.org/) or PlatformIO Core
- ESP32 DevKit boards (2x)
- Required sensors and components (see hardware lists above)

### Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/JayZ6969/cansat
   cd cansat
   ```

2. Open each project in PlatformIO:
   - Primary: Open `/primary/` folder
   - Secondary: Open `/secondary/` folder

3. Install dependencies (automatically handled by PlatformIO):
   ```bash
   # For primary
   cd primary
   pio lib install

   # For secondary
   cd secondary
   pio lib install
   ```

### Building and Uploading

#### Primary ESP32
```bash
cd primary
pio run                 # Build
pio run --target upload # Upload to ESP32
pio device monitor      # Monitor serial output
```

#### Secondary ESP32
```bash
cd secondary
pio run                 # Build
pio run --target upload # Upload to ESP32
pio device monitor      # Monitor serial output
```

## Configuration

### Primary ESP32 Pin Configuration
- **GPS**: UART2 (GPIO16-RX, GPIO17-TX)
- **I2C Sensors**: GPIO21-SDA, GPIO22-SCL
- **SD Card**: SPI (GPIO23-MOSI, GPIO19-MISO, GPIO18-SCK, GPIO5-CS)
- **Battery Monitor**: GPIO36 (ADC)
- **LEDs**: GPIO12 (RED), GPIO13 (YELLOW), GPIO14 (GREEN)
- **Buzzer**: GPIO0

### Secondary ESP32 Pin Configuration
- **LoRa SX1278**: SPI (GPIO18-SCK, GPIO19-MISO, GPIO23-MOSI, GPIO5-SS, GPIO4-RST, GPIO26-DIO0)
- **BMP390**: I2C (default pins)
- **Servo**: GPIO25
- **UART2**: GPIO16-RX, GPIO17-TX (Primary communication)
- **LEDs**: GPIO2, GPIO13

## Flight States

The system manages the following flight states:
- **BOOT** (0): Initialization and system checks
- **TEST_MODE** (1): Pre-flight testing and calibration
- **LAUNCH_PAD** (2): Ready for launch
- **ASCENDING** (3): Active flight phase
- **DESCENDING** (4): Parachute deployment phase
- **LANDED** (5): Post-landing data collection

## Data Format

Telemetry data is stored and transmitted in CSV format:
```
TEAM_ID, TIMESTAMP, PACKET_COUNT, ALTITUDE, PRESSURE, TEMP, VOLTAGE,
GNSS_TIME, GNSS_LAT, GNSS_LONG, GNSS_ALT, GNSS_SATS,
ACCEL_X, ACCEL_Y, ACCEL_Z, GYRO_X, GYRO_Y, GYRO_Z,
GYRO_SPIN_RATE, FLIGHT_STATE, OPTIONAL_DATA
```

## Troubleshooting

### Common Issues
1. **SD Card not detected**: Check wiring and card formatting (FAT32)
2. **GPS not acquiring lock**: Ensure clear sky view and proper antenna connection
3. **LoRa transmission fails**: Verify antenna connection and frequency settings

### LED Status Indicators

**Primary ESP32:**
- Red LED: Boot state indicator
- Yellow LED: GPS lock status (on = no lock)
- Green LED: System OK + data operation blinks

**Secondary ESP32:**
- GPIO2 LED: Initialization/data reception indicator
- GPIO13 LED: LoRa status/error indicator

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

This project is developed for educational purposes as part of the IN-SPACe CANSAT India Student Competition 2024-25.

---

**Competition**: IN-SPACe CANSAT India Student Competition 2024-25 (2nd Edition)  
**Team**: AVINYA  
**Date**: 2024-25 Academic Year
