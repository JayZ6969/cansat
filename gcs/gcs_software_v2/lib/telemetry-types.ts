export interface TelemetryData {
  timestamp: number
  missionTime: number

  // Reaction Wheel Data
  reactionWheelX: number
  reactionWheelY: number
  reactionWheelZ: number

  // Gyroscope Data
  gyroX: number
  gyroY: number
  gyroZ: number

  // Accelerometer Data
  accelX: number
  accelY: number
  accelZ: number

  // Altitude & Pressure
  altitude: number
  pressure: number

  // Temperature
  temperature: number

  // GPS Data
  latitude: number
  longitude: number
  gpsAltitude: number
  gpsSpeed: number
  gpsSatellites: number

  // Power
  batteryVoltage: number
  batteryPercentage: number

  // Status
  systemStatus: "idle" | "armed" | "ascending" | "descending" | "landed"
  signalStrength: number
  dataRate: number
  
  // LoRa Communication
  rssi?: number  // Received Signal Strength Indicator (dB)
  snr?: number   // Signal-to-Noise Ratio
  
  // Flight State
  flightState?: number  // 0=pre-launch, 1=ascending, 2=descending, etc.
  mode?: string
  
  // Error Codes
  errorCodes?: Array<string | { code: string; message?: string }>
}

export interface MissionStatus {
  missionName: string
  launchTime: number
  currentTime: number
  status: "pre-launch" | "ascending" | "apogee" | "descending" | "landed" | "recovery"
  maxAltitude: number
  flightTime: number
  recoveryLocation?: { lat: number; lng: number }
}

export interface ConnectionState {
  isConnected: boolean
  connectionType: "websocket" | "http" | "none"
  lastUpdate: number
  messageCount: number
  errorCount: number
  espConnected: boolean  // ESP is connected to GCS
  loraConnected: boolean // LoRa successfully connected to CanSat
}
