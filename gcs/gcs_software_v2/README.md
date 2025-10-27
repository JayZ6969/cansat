# CanSat Ground Control Station V2 - Professional Web Interface

Modern Next.js-based Ground Control Station for real-time CanSat telemetry monitoring with advanced visualization and data logging capabilities.

## 🚀 Professional Mission Control System

### Advanced Features
- **Real-time Telemetry**: Live 23-field CSV data processing at 10Hz
- **Interactive Dashboard**: Modern React components with real-time charts
- **Data Visualization**: Altitude, GPS tracking, sensor graphs, and flight state monitoring
- **Mission Logging**: Complete flight data storage and export capabilities
- **Professional UI**: Competition-grade interface for mission monitoring

## 📡 Enhanced Communication System

### LoRa Reception & Processing
- **435MHz Reception**: High-speed telemetry from Secondary ESP32 LoRa transmission
- **Connection Monitoring**: Real-time link status with 30-second timeout detection
- **Data Validation**: Complete 23-field CSV format verification and error handling
- **Signal Quality**: RSSI/SNR monitoring for link assessment

### Data Stream Types
1. **Primary Telemetry**: 23-field CSV format with complete sensor suite data
2. **System Status**: Operational messages from CanSat subsystems
3. **Error Reporting**: Real-time system health monitoring with concatenated error codes
4. **Flight State**: Mission phase tracking from BOOT through IMPACT

## 📊 Complete Telemetry Specification

### Current System Architecture
- **Primary ESP32**: Flight computer with GPS L89, MPU6050, BMP280 backup, SD logging, flight state management
- **Secondary ESP32**: BMP390 primary sensor, LoRa 435MHz transmission, servo control, UART relay
- **Data Rate**: 10Hz telemetry (100ms intervals) with 50ms UART synchronization
- **Recovery Features**: Dual sensor failover (BMP390→BMP280), flight state recovery from CSV

---

## 🚨 Enhanced Error Code System

**Concatenated error monitoring** for comprehensive system health tracking:

### Current Error Mapping

| Error ID | Subsystem | Description | Critical Level |
|----------|-----------|-------------|----------------|
| **1** | MPU6050 | Accelerometer/Gyroscope failure (invalid/old data) | High |
| **2** | BMP280 | Backup pressure sensor failure (failover active) | Medium |
| **3** | SD Card | Telemetry logging failure (file system errors) | High |
| **5** | GPS L89 | GPS communication failure (<4 satellites/timeout) | Medium |
| **6** | PID | PID control system malfunction | Medium |
| **7** | Camera | Imaging system error (placeholder for future) | Low |
| **8** | Serial | UART2 communication failure (Primary↔Secondary) | Critical |
| **9** | LoRa | Transmission failure (435MHz link down) | Critical |
| **10** | BMP390 | Primary altitude sensor failure (triggers failover) | High |

### Advanced Error Examples

- **"0"** = All systems operational (green status)
- **"10"** = BMP390 failure → Automatic BMP280 failover with CSV calibration
- **"28"** = BMP280 backup + Serial communication failures (critical)
- **"1358"** = MPU6050 + SD + GPS + Serial failures (mission critical)
- **"10123"** = Multiple system failures requiring immediate attention

> **Critical**: Error parsing must handle two-digit codes correctly. "10" ≠ "1" + "0"

---

## 📋 Enhanced CSV Data Format (23 Fields + GCS Metadata)

**Core telemetry transmitted via 435MHz LoRa** from Secondary ESP32:

| # | Field Name | Source | Format | Description | Example |
|---|------------|--------|--------|-------------|---------|
| **1** | TEAM_ID | Constant | String | Competition team identifier | 1013 |
| **2** | MISSION_TIME | millis() | String (HH:MM:SS.sss) | Mission elapsed time with milliseconds | 10:34:23.456 |
| **3** | PACKET_COUNT | Counter | Integer | Sequential packet number | 1254 |
| **4** | MODE | State Machine | Integer (0-7) | Current flight mode | 3 |
| **5** | STATE | Descriptor | String | Flight state name | ASCENT |
| **6** | ALTITUDE | BMP390/BMP280 | Float (1 decimal) | Primary altitude (BMP390) with BMP280 failover | 1250.4 |
| **7** | AIR_SPEED | Calculated | Float (1 decimal) | Calculated airspeed in m/s | 15.2 |
| **8** | HS_DEPLOYED | Deployment | Char | Heat shield status (P=deployed, N=not) | N |
| **9** | PC_DEPLOYED | Deployment | Char | Parachute status (P=deployed, N=not) | N |
| **10** | MAST_RAISED | Deployment | Char | Mast deployment (P=raised, N=not) | N |
| **11** | TEMPERATURE | BMP390/BMP280 | Float (1 decimal) | Atmospheric temperature in °C | 22.5 |
| **12** | VOLTAGE | Secondary ESP32 | Float (1 decimal) | Battery voltage in Volts | 7.2 |
| **13** | PRESSURE | BMP390/BMP280 | Float (1 decimal) | Atmospheric pressure in hPa | 1013.2 |
| **14** | GPS_TIME | GPS L89 | String | GPS UTC time (HH:MM:SS) | 10:34:22 |
| **15** | GPS_ALTITUDE | GPS L89 | Float (1 decimal) | GPS altitude in meters | 1248.7 |
| **16** | GPS_LATITUDE | GPS L89 | Float (6 decimals) | Latitude coordinate | 40.712800 |
| **17** | GPS_LONGITUDE | GPS L89 | Float (6 decimals) | Longitude coordinate | -74.006000 |
| **18** | GPS_SATELLITES | GPS L89 | Integer | Number of connected satellites | 8 |
| **19** | TILT_X | MPU6050 | Float (1 decimal) | X-axis tilt angle in degrees | 5.2 |
| **20** | TILT_Y | MPU6050 | Float (1 decimal) | Y-axis tilt angle in degrees | -2.1 |
| **21** | ROTATION_Z | MPU6050 | Float (1 decimal) | Z-axis angular velocity in °/s | 45.3 |
| **22** | CMD_ECHO | Command | String | Ground command acknowledgment | NONE |
| **23** | ERROR_CODES | Error System | String | Concatenated error status codes | (empty for no errors) |

### GCS Enhanced Format (26 Fields Total)
**Additional fields added by Ground Control Station:**

| # | Field Name | Source | Format | Description | Example |
|---|------------|--------|--------|-------------|---------|
| **24** | RECORDING_TIME | GCS | ISO Timestamp | GCS reception timestamp | 2025-01-20T15:45:32.123Z |
| **25** | RSSI | LoRa | Float (1 decimal) | Received Signal Strength Indicator (dBm) | -65.0 |
| **26** | SNR | LoRa | Float (1 decimal) | Signal-to-Noise Ratio (dB) | 8.5 |

---

## 🛩️ Advanced Flight State System

Enhanced state machine with recovery support for power failures:

| State | Name | Trigger Conditions | Recovery Features |
|-------|------|-------------------|-------------------|
| **0** | BOOT | System initialization (5 seconds) | Reads last state from CSV for recovery |
| **1** | TEST_MODE | Post-initialization testing | Pre-flight sensor validation |
| **2** | LAUNCH_PAD | Ready for launch, stable altitude | Baseline altitude establishment |
| **3** | ASCENT | Altitude > 50m above baseline | Active flight monitoring |
| **4** | ROCKET_DEPLOY | Altitude > 700m (parachute trigger) | Apogee detection and deployment |
| **5** | DESCENT | Descending with deployed parachute | Descent rate monitoring |
| **6** | AEROBRAKE_RELEASE | Altitude < 50% of max altitude | Secondary deployment system |
| **7** | IMPACT | Landed (stable low altitude) | Recovery beeper activation |

---

## Example CSV Row

```csv
2024-ASI-CANSAT-049,12345,100,125.30,1013.25,24.50,7.84,12:34:56,12.345678,77.654321,125.00,8,0.120,-0.050,9.810,0.010,-0.020,0.030,0.00,2,0,0,0.15
```

### Breakdown:

- **Team:** 2024-ASI-CANSAT-049
- **Time:** 12.345 seconds since boot
- **Packet:** #100
- **Altitude:** 125.3m
- **Pressure:** 1013.25 hPa
- **Temp:** 24.5°C
- **Battery:** 7.84V
- **GPS:** 12:34:56, Lat 12.345678°, Long 77.654321°, Alt 125m, 8 sats
- **Accel:** 0.12g, -0.05g, 9.81g (X,Y,Z)
- **Gyro:** 0.01, -0.02, 0.03 rad/s (X,Y,Z)
- **PID:** 0.0
- **Flight State:** 2 (LAUNCH_PAD)
- **Servo:** 0 (closed)
- **Errors:** 0 (none)
- **Speed:** 0.15 m/s

---

## Implementation Notes

### Error Code Parsing

When parsing error codes:
1. Check for two-digit code "10" first
2. Then parse remaining single-digit codes
3. "0" means no errors

Example parsing logic:
```typescript
function parseErrorCodes(errorStr: string): string[] {
  if (!errorStr || errorStr === '0') return []
  
  const codes: string[] = []
  let remaining = errorStr
  
  // Check for two-digit codes first (10)
  if (remaining.includes('10')) {
    codes.push('10')
    remaining = remaining.replace('10', '')
  }
  
  // Parse single-digit codes
  for (const char of remaining) {
    if (char !== '0') codes.push(char)
  }
  
  return codes
}
```

## LoRa Transmission Format

Data is received by GCS in the following format:

```
RX #XXX @ XXXXX ms | XXX bytes | RSSI=XX dB | SNR=X.X | CSV:TEAM_ID,TIMESTAMP,PACKET_COUNT,...
```

**Example:**
```
RX #123 @ 45678 ms | 150 bytes | RSSI=-45 dB | SNR=8.5 | CSV:2024-ASI-CANSAT-049,12345,100,125.30,1013.25,24.50,7.84,12:34:56,12.345678,77.654321,125.00,8,0.120,-0.050,9.810,0.010,-0.020,0.030,0.00,2,0,0,0.15
```

**STATUS Messages:**
```
[STATUS] System operational
[STATUS] GPS acquiring fix
[STATUS] Landing detected
```

### Field Name Mapping

The GCS should map CSV fields to telemetry data structure:

```typescript
{
  teamId: TEAM_ID,
  timestamp: TIMESTAMP,
  packetCount: PACKET_COUNT,
  altitude: ALTITUDE,
  pressure: PRESSURE,
  temperature: TEMP,
  batteryVoltage: VOLTAGE,
  gpsTime: GNSS_TIME,
  latitude: GNSS_LAT,
  longitude: GNSS_LONG,
  gpsAltitude: GNSS_ALT,
  gpsSatellites: GNSS_SATS,
  accelX: ACCEL_X,
  accelY: ACCEL_Y,
  accelZ: ACCEL_Z,
  gyroX: GYRO_X,
  gyroY: GYRO_Y,
  gyroZ: GYRO_Z,
  gyroSpinRate: GYRO_SPIN_RATE,
  flightState: FLIGHT_STATE,
  servoStatus: SERVO_STATUS,
  errorCode: ERROR_CODE,
  gpsSpeed: GNSS_SPEED
}
```

---

## Data Validation

### Critical Checks:
- GPS fix valid when GNSS_SATS >= 4
- Altitude changes > 50m indicate ASCENT state
- Error code "0" indicates all systems nominal
- Battery voltage should be monitored (critical < 6.5V)
- Packet count should increment sequentially

### Sensor Health:
- MPU6050 healthy: error code doesn't contain "1"
- BMP280 healthy: error code doesn't contain "2"
- BMP390 healthy: error code doesn't contain "10"
- GPS healthy: error code doesn't contain "5"

---

## GCS Extended Format (25 Fields)

The GCS adds two additional fields when recording data:

| # | Field Name | Description | Example |
|---|------------|-------------|---------|
| **24** | RECORDING_TIME | GCS timestamp when data was recorded | 2025-10-22T16:51:07.123Z |
| **25** | RSSI | Received Signal Strength Indicator | -65.0 |
| **26** | SNR | Signal-to-Noise Ratio | 10.2 |

### GCS CSV Header
```
team_id,mission_time,packet_count,altitude,pressure,temperature,battery_voltage,gnss_time,gnss_lat,gnss_long,gnss_alt,gnss_sats,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,pid_output,flight_state,servo_status,error_code,gnss_speed,recording_time,rssi,snr
```

## Architecture

The GCS consists of two main components:

### 1. Backend (Python + Flask-SocketIO)
- **Location**: `backend/main.py`
- **Port**: `5000`
- **Purpose**: 
  - Manages serial communication with CanSat
  - Processes telemetry data
  - Provides WebSocket API for real-time data streaming
  - Handles data storage and export

### 2. Frontend (Next.js + React)
- **Location**: `web/`
- **Port**: `3000` (development)
- **Purpose**:
  - Modern, responsive dashboard UI
  - Real-time data visualization
  - Interactive charts and maps
  - Connection management interface

## Setup

### Prerequisites
- Python 3.10+ (Python 3.12 recommended)
- Node.js 18+ 
- npm

### Installation

#### 1. Setup Python Backend

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment (Windows)
venv\Scripts\activate

# Install Python dependencies
pip install -r backend/requirements.txt
```

#### 2. Setup Frontend

```bash
# Install Node.js dependencies
npm install --legacy-peer-deps
```

#### 3. Download Satellite Imagery (First-Time Setup)

**Important**: After cloning the repository, download cached satellite tiles for offline map operation:

```bash
# Windows
setup_satellite_tiles.bat

# macOS/Linux
chmod +x setup_satellite_tiles.sh
./setup_satellite_tiles.sh

# Or manually
python scripts/download_satellite_tiles.py
```

This downloads satellite imagery for the mission area (26.720333°N, 84.303806°E, 3km radius).
- **First download**: ~5-15 minutes
- **Storage**: ~100-300 MB
- **Result**: Offline satellite maps

See [SATELLITE_IMAGERY_SETUP.md](SATELLITE_IMAGERY_SETUP.md) for detailed information.

## Running the Application

### Quick Start
```bash
npm run dev
```

This will:
- Start the backend WebSocket server on `http://localhost:5000`
- Start the Next.js frontend on `http://localhost:3000`
- Automatically open your browser to the dashboard

### Run Servers Individually

```bash
# Backend only
npm run dev:backend

# Frontend only
npm run dev:frontend
```

This will:
1. Start the backend server on `http://localhost:5000`
2. Start the Next.js frontend on `http://localhost:3000`
3. Automatically open your browser to the dashboard

### Manual Start

1. Activate virtual environment:
```bash
venv\Scripts\activate
```

2. Run the application:
```bash
python main.py
```

## Project Structure

```
gcs_software_v2/
├── main.py                    # Main orchestrator (launches backend & frontend)
├── package.json               # Frontend dependencies
├── next.config.mjs            # Next.js configuration
├── tsconfig.json              # TypeScript configuration
├── backend/
│   ├── main.py               # Flask-SocketIO WebSocket server
│   └── requirements.txt      # Backend dependencies
├── app/                      # Next.js pages
├── components/               # React components
│   ├── charts/              # Chart components
│   └── ui/                  # UI components
├── lib/                      # Frontend utilities and hooks
├── hooks/                    # React hooks
├── public/                   # Public assets
├── styles/                   # CSS styles
├── src/
│   ├── telemetry_handler.py # Serial communication
│   └── data_manager.py      # Data storage & export
└── data/                     # Mission data storage
```

## Features

### Backend Features
- ✅ Serial port communication
- ✅ Real-time WebSocket data streaming
- ✅ Telemetry data parsing
- ✅ CSV data export
- ✅ Mission control (start/stop)
- ✅ Connection management

### Frontend Features
- ✅ Real-time telemetry dashboard
- ✅ Interactive charts (altitude, temperature, pressure, etc.)
- ✅ GPS map visualization
- ✅ Connection status monitoring
- ✅ Mission controls
- ✅ Data export interface
- ✅ System diagnostics
- ✅ Responsive design

## API Endpoints

### WebSocket Events (Socket.IO)

#### Client → Server
- `connect` - Initial connection
- `get_ports` - Request available COM ports
- `connect_serial` - Connect to serial port
  ```json
  { "port": "COM3", "baudrate": 9600 }
  ```
- `disconnect_serial` - Disconnect from serial port
- `start_mission` - Start data streaming
- `stop_mission` - Stop data streaming
- `export_data` - Export mission data
  ```json
  { "format": "csv" }
  ```

#### Server → Client
- `connection_status` - Connection confirmation
- `ports_list` - Available COM ports
- `serial_status` - Serial connection status
- `mission_status` - Mission state updates
- `telemetry_data` - Real-time telemetry packets
- `export_result` - Export operation result
- `error` - Error messages

## Telemetry Data Format

```typescript
{
  teamId: number
  missionTime: number
  packetCount: number
  mode: string
  state: string
  altitude: number
  temperature: number
  pressure: number
  voltage: number
  gps: {
    latitude: number
    longitude: number
    altitude: number
    satellites: number
  }
  // ... additional fields
}
```

## Configuration

### Backend Configuration
Edit `backend/main.py`:
- Change port: Modify `socketio.run(app, port=5000)`
- Adjust data rate: Modify `time.sleep(0.1)` in telemetry loop

### Frontend Configuration
Edit `lib/use-telemetry.ts`:
- WebSocket URL: Change `wsUrl` parameter
- Mock data: Toggle `useMockData` flag

## Troubleshooting

### Backend won't start
- Ensure virtual environment is activated
- Install backend dependencies: `pip install -r backend/requirements.txt`
- Check port 5000 is not in use

### Frontend won't start
- Install Node.js dependencies: `pnpm install`
- Check port 3000 is not in use
- Try `pnpm dev` manually in root directory

### No serial ports found
- Check USB connection
- Install serial drivers if needed
- Run as Administrator if necessary

### WebSocket connection fails
- Ensure backend is running
- Check CORS settings in `backend/main.py`
- Verify firewall settings

## Development

### Adding New Telemetry Fields
1. Update telemetry parser in `src/telemetry_handler.py`
2. Update TypeScript types in `lib/telemetry-types.ts`
3. Add visualization in frontend components

### Creating New Dashboard Panels
1. Create component in `components/`
2. Add to layout in `components/dashboard-layout.tsx`
3. Connect to telemetry hook with `useTelemetry()`

## License

[Add your license here]

## Contributors

[Add contributors here]
