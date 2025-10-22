import type { TelemetryData } from "./telemetry-types"

/**
 * Maps Python snake_case telemetry data to TypeScript camelCase
 * @param pythonData - Raw data from Python backend with snake_case keys
 * @returns Mapped telemetry data with camelCase keys
 */
export function mapPythonToTelemetry(pythonData: any): TelemetryData {
  return {
    timestamp: pythonData.timestamp || Date.now(),
    missionTime: pythonData.mission_time || pythonData.flight_time_ms || 0,

    // Reaction Wheel Data (PID output - single value, not X/Y/Z)
    // Setting all axes to the same value since we only have one PID output
    reactionWheelX: pythonData.reaction_wheel || pythonData.pid_output || 0,
    reactionWheelY: pythonData.reaction_wheel || pythonData.pid_output || 0,
    reactionWheelZ: pythonData.reaction_wheel || pythonData.pid_output || 0,

    // Gyroscope Data (from MPU - separate from reaction wheel)
    gyroX: pythonData.gyro_x || 0,
    gyroY: pythonData.gyro_y || 0,
    gyroZ: pythonData.gyro_z || 0,

    // Accelerometer Data (from MPU)
    accelX: pythonData.accel_x || 0,
    accelY: pythonData.accel_y || 0,
    accelZ: pythonData.accel_z || 0,

    // Altitude & Pressure
    altitude: pythonData.altitude || 0,
    pressure: pythonData.pressure || 0,

    // Temperature
    temperature: pythonData.temperature || 0,

    // GPS Data
    latitude: pythonData.gps_latitude || pythonData.latitude || 0,
    longitude: pythonData.gps_longitude || pythonData.longitude || 0,
    gpsAltitude: pythonData.gps_altitude || pythonData.gnss_altitude || 0,
    gpsSpeed: pythonData.gps_speed || pythonData.gnss_speed || 0,
    gpsSatellites: pythonData.gps_satellites || pythonData.gnss_sats || 0,

    // Power
    batteryVoltage: pythonData.battery_voltage || pythonData.voltage || 0,
    batteryPercentage: pythonData.battery_percentage || 0,

    // Status
    systemStatus: mapFlightStateToStatus(pythonData.flight_state),
    signalStrength: pythonData.signal_strength || 50,
    dataRate: pythonData.data_rate || 10,
    
    // LoRa Communication
    rssi: pythonData.rssi,
    snr: pythonData.snr,
    
    // Flight State
    flightState: pythonData.flight_state,
    mode: pythonData.mode || pythonData.flight_mode,
  }
}

function mapFlightStateToStatus(flightState: number | undefined): "idle" | "armed" | "ascending" | "descending" | "landed" {
  if (flightState === undefined || flightState === null) return "idle"
  
  switch (flightState) {
    case 0:
    case 1:
      return "idle"
    case 2:
      return "armed"
    case 3:
    case 4:
      return "ascending"
    case 5:
    case 6:
      return "descending"
    case 7:
      return "landed"
    default:
      return "idle"
  }
}
