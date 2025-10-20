# GCS Software - Usage Guide

## Overview

Ground Control Station for CanSat missions that receives and displays telemetry data from serial port.

## Features

- ✅ Real-time serial data reception
- ✅ Automatic parsing of LoRa telemetry format
- ✅ Live dashboard with charts and GPS map
- ✅ Data export functionality
- ✅ AVINYA branding with blackish UI theme

## Data Format

### Expected Serial Input Format

Your software expects data in this format:

```
RX #638 @ 170114 ms | 153 bytes | RSSI=-43 dB | SNR=1.5 | 2024-ASI-CANSAT-049,159874,658,-11.33,908.12,26.29,0.28,00:02:21,0.000000,0.000000,0.00,0,0.040,-0.003,-0.862,-4.695,1.707,-3.384,6.034,1,CLOSED,1.80,345
```

The software will:

1. Extract the CSV data after the last `|` symbol
2. Parse RSSI and SNR values
3. Parse all 23 telemetry fields

### 23 Telemetry Fields (in order)

1. TEAM_ID - Team identifier (e.g., "2024-ASI-CANSAT-049")
2. TIMESTAMP - Mission timestamp (e.g., "159874")
3. PACKET_COUNT - Packet number (e.g., "658")
4. ALTITUDE - Altitude in meters (e.g., "-11.33")
5. PRESSURE - Pressure in hPa (e.g., "908.12")
6. TEMP - Temperature in °C (e.g., "26.29")
7. VOLTAGE - Battery voltage (e.g., "0.28")
8. GNSS_TIME - GPS time (e.g., "00:02:21")
9. GNSS_LAT - GPS latitude (e.g., "0.000000")
10. GNSS_LONG - GPS longitude (e.g., "0.000000")
11. GNSS_ALT - GPS altitude (e.g., "0.00")
12. GNSS_SATS - Number of satellites (e.g., "0")
13. ACCEL_X - Acceleration X-axis (e.g., "0.040")
14. ACCEL_Y - Acceleration Y-axis (e.g., "-0.003")
15. ACCEL_Z - Acceleration Z-axis (e.g., "-0.862")
16. GYRO_X - Gyroscope X-axis (e.g., "-4.695")
17. GYRO_Y - Gyroscope Y-axis (e.g., "1.707")
18. GYRO_Z - Gyroscope Z-axis (e.g., "-3.384")
19. GYRO_SPIN_RATE - Spin rate (e.g., "6.034")
20. FLIGHT_STATE - Flight state number (e.g., "1")
21. SERVO_STATUS - Servo status (e.g., "CLOSED")
22. PID_SPEED - PID speed (e.g., "1.80")
23. ERROR_CODE - Error code (e.g., "345")

## How to Use

### 1. Setup (First Time Only)

```powershell
cd C:\Users\redmi\Downloads\gcs\gcs_software
.\setup.bat
```

### 2. Run the GCS

```powershell
.\run.bat
```

OR

```powershell
python main.py
```

### 3. Connect to COM Port

1. Click the **Scan Ports** button in the dashboard
2. Select your COM port from the dropdown
3. Click **Connect**
4. You'll see "✓ Connected to COMX at 9600 baud" in the terminal

### 4. Start Mission

1. Click the **START** button
2. Data will start appearing on the dashboard as it's received

### 5. Monitor Data

- **Live Charts**: Altitude, temperature, pressure, acceleration, gyroscope
- **GPS Map**: Real-time position tracking
- **Status Panels**: Battery, flight state, packet count, signal quality

### 6. Stop Mission

1. Click the **STOP** button to pause data display
2. Click **Disconnect** to close the serial connection

## Troubleshooting

### COM Port Won't Connect

- Make sure no other program is using the port
- Check if the correct baud rate is set (default: 9600)
- Verify the device is properly connected

### No Data Appearing

- Check if mission is started (START button clicked)
- Verify serial connection is active
- Look at terminal output for error messages
- Make sure data format matches expected format

### Dashboard Not Updating

- Check browser console (F12) for JavaScript errors
- Restart the application
- Clear browser cache

## Serial Port Settings

- **Baud Rate**: 9600 (default)
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None

## Flight States

- 0: BOOT
- 1: TEST_MODE
- 2: LAUNCH_PAD
- 3: ASCENT
- 4: ROCKET_DEPLOY
- 5: DESCENT
- 6: AEROBRAKE_RELEASE
- 7: IMPACT

## Project Structure

```
gcs_software/
├── main.py                 # Main application
├── src/
│   ├── telemetry_handler.py   # Serial communication & parsing
│   └── data_manager.py         # Data storage & export
├── web/
│   ├── index.html          # Dashboard interface
│   ├── AVINYA.png          # Logo
│   ├── css/
│   │   └── style.css       # Blackish theme styles
│   └── js/
│       ├── main.js         # Main logic
│       ├── dashboard.js    # Dashboard functionality
│       ├── charts.js       # Chart plotting
│       └── map.js          # GPS mapping
├── data/
│   └── AVINYA.png          # Logo backup
├── requirements.txt        # Python dependencies
├── run.bat                 # Quick start script
└── setup.bat               # Installation script
```

## Terminal Output Examples

### Successful Connection

```
✓ Connected to COM3 at 9600 baud
Serial reading thread started...
```

### Receiving Data

```
Received: RX #638 @ 170114 ms | 153 bytes | RSSI=-43 dB | SNR=1.5 | 2024-ASI-CANSAT-049,159874,658,-11.33,908.12,26.29,0.28,00:02:21,0.000000,0.000000,0.00,0,0.040,-0.003,-0.862,-4.695,1.707,-3.384,6.034,1,CLOSED,1.80,345
✓ Parsed packet #658
```

### Connection Error

```
✗ Serial error connecting to COM3: [Errno 2] could not open port 'COM3': FileNotFoundError(2, 'The system cannot find the file specified.', None, 2)
```

## Support

- Team: 2024-ASI-CANSAT-049
- Mission: AVINYA CanSat Project
- Updated: October 20, 2025
