# CanSat Ground Control Station (GCS)

A modern Ground Control Station for CanSat missions with real-time telemetry monitoring.

## CanSat Telemetry Data Format Reference

### Overview
This section describes the complete data format for telemetry transmitted from the CanSat via LoRa to the Ground Control Station (GCS).

---

## Error Code System

The error code system uses **concatenated numbers** where each digit represents a specific subsystem error.

### Error Code Mapping

| Error ID | Subsystem | Description |
|----------|-----------|-------------|
| **1** | MPU6050 | Accelerometer/Gyroscope sensor failure (invalid data) |
| **2** | BMP280 | Primary barometric pressure sensor failure (invalid data) |
| **3** | SD Card | SD card access failure (file not found) |
| **5** | GNSS/GPS | GPS failure (no valid fix, old data, or <4 satellites) |
| **6** | PID | PID control system error |
| **7** | Camera | Camera module error (placeholder) |
| **8** | Serial | UART communication failure with Secondary ESP32 |
| **9** | LoRa | LoRa transmission error |
| **10** | BMP390 | Secondary barometric pressure sensor failure |

### Error Code Format Examples

- **"0"** = No errors (all systems OK)
- **"1"** = Only MPU6050 error
- **"23"** = BMP280 + SD Card errors
- **"1510"** = MPU6050 + GNSS + BMP390 errors
- **"12389"** = MPU6050, BMP280, SD Card, Serial, and LoRa all have errors

> **Note:** Multiple errors are concatenated. Two-digit error code "10" must be parsed correctly.

---

## CSV Data Format (23 Fields)

Data transmitted via LoRa in exact order:

| # | Field Name | Source | Format | Description | Example |
|---|------------|--------|--------|-------------|---------|
| **1** | TEAM_ID | Constant | String | Team identifier | 2024-ASI-CANSAT-049 |
| **2** | TIMESTAMP | millis() | Integer (ms) | Time since boot | 12345 |
| **3** | PACKET_COUNT | Counter | Integer | Packet sequence number | 100 |
| **4** | ALTITUDE | BMP390/BMP280 | Float (2 decimals) | Altitude in meters (from BMP390 if available, else BMP280) | 125.30 |
| **5** | PRESSURE | BMP390/BMP280 | Float (2 decimals) | Atmospheric pressure in hPa | 1013.25 |
| **6** | TEMP | BMP390/BMP280 | Float (2 decimals) | Temperature in °C | 24.50 |
| **7** | VOLTAGE | Secondary ESP32 | Float (2 decimals) | Battery voltage in Volts | 7.84 |
| **8** | GNSS_TIME | GPS | String | GPS time (HH:MM:SS) | 12:34:56 |
| **9** | GNSS_LAT | GPS | Float (6 decimals) | Latitude in degrees | 12.345678 |
| **10** | GNSS_LONG | GPS | Float (6 decimals) | Longitude in degrees | 77.654321 |
| **11** | GNSS_ALT | GPS | Float (2 decimals) | GPS altitude in meters | 125.00 |
| **12** | GNSS_SATS | GPS | Integer | Number of satellites | 8 |
| **13** | ACCEL_X | MPU6050 | Float (3 decimals) | X-axis acceleration in g | 0.120 |
| **14** | ACCEL_Y | MPU6050 | Float (3 decimals) | Y-axis acceleration in g | -0.050 |
| **15** | ACCEL_Z | MPU6050 | Float (3 decimals) | Z-axis acceleration in g | 9.810 |
| **16** | GYRO_X | MPU6050 | Float (3 decimals) | X-axis rotation in rad/s | 0.010 |
| **17** | GYRO_Y | MPU6050 | Float (3 decimals) | Y-axis rotation in rad/s | -0.020 |
| **18** | GYRO_Z | MPU6050 | Float (3 decimals) | Z-axis rotation in rad/s | 0.030 |
| **19** | GYRO_SPIN_RATE | PID Output | Float (2 decimals) | PID output value (replaces spin rate) | 0.00 |
| **20** | FLIGHT_STATE | State Machine | Integer (0-7) | Current flight state (see below) | 2 |
| **21** | SERVO_STATUS | Servo | Integer (0 or 1) | Parachute servo: 0=closed, 1=open | 0 |
| **22** | ERROR_CODE | Error System | String | Concatenated error IDs (see above) | 0 or 12389 |
| **23** | GNSS_SPEED | MPU6050 | Float (2 decimals) | Speed calculated from acceleration in m/s | 0.15 |

---

## Flight States

| State Value | State Name | Description |
|-------------|------------|-------------|
| **0** | BOOT | System initializing, sensors starting up |
| **1** | TEST_MODE | Sensors initialized, pre-flight testing |
| **2** | LAUNCH_PAD | Ready for launch, waiting on pad |
| **3** | ASCENT | Rocket ascending (altitude > 50m) |
| **4** | ROCKET_DEPLOY | Parachute deployed at apogee (altitude > 700m) |
| **5** | DESCENT | Descending with parachute |
| **6** | AEROBRAKE_RELEASE | Aerobrake released (altitude < 50% max) |
| **7** | IMPACT | Landed, recovery beeper active |

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
