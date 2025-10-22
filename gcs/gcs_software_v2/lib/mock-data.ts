import type { TelemetryData, MissionStatus } from "./telemetry-types"

const LAUNCH_TIME = Date.now() - 300000 // 5 minutes ago

export function generateMockTelemetry(missionTime: number): TelemetryData {
  const progress = Math.min(missionTime / 600000, 1) // 10 minute mission
  const phase = progress < 0.3 ? "ascending" : progress < 0.5 ? "apogee" : "descending"

  let altitude = 0
  if (phase === "ascending") {
    altitude = 1000 + (missionTime / 600000) * 3000 + Math.random() * 100
  } else if (phase === "apogee") {
    altitude = 4000 - Math.abs(Math.sin((missionTime - 180000) / 60000)) * 200
  } else {
    altitude = Math.max(0, 4000 - ((missionTime - 300000) / 300000) * 4000 + Math.random() * 100)
  }

  return {
    timestamp: Date.now(),
    missionTime,
    reactionWheelX: Math.sin(missionTime / 10000) * 50 + Math.random() * 10,
    reactionWheelY: Math.cos(missionTime / 12000) * 50 + Math.random() * 10,
    reactionWheelZ: Math.sin(missionTime / 15000) * 50 + Math.random() * 10,
    gyroX: Math.sin(missionTime / 8000) * 30 + Math.random() * 5,
    gyroY: Math.cos(missionTime / 10000) * 30 + Math.random() * 5,
    gyroZ: Math.sin(missionTime / 12000) * 30 + Math.random() * 5,
    accelX: Math.sin(missionTime / 5000) * 15 + Math.random() * 2,
    accelY: Math.cos(missionTime / 6000) * 15 + Math.random() * 2,
    accelZ: 9.81 + Math.sin(missionTime / 7000) * 5 + Math.random() * 2,
    altitude,
    pressure: 101325 * Math.exp(-altitude / 8435),
    temperature: 20 - (altitude / 1000) * 6.5 + Math.random() * 2,
    latitude: 37.7749 + (Math.random() - 0.5) * 0.01,
    longitude: -122.4194 + (Math.random() - 0.5) * 0.01,
    gpsAltitude: altitude + (Math.random() - 0.5) * 50,
    gpsSpeed: Math.sqrt(
      Math.pow(Math.sin(missionTime / 10000) * 50, 2) + Math.pow(Math.cos(missionTime / 12000) * 50, 2),
    ),
    gpsSatellites: 12 + Math.floor(Math.random() * 4),
    batteryVoltage: 12 - (missionTime / 600000) * 2 + Math.random() * 0.5,
    batteryPercentage: Math.max(0, 100 - (missionTime / 600000) * 20),
    systemStatus: phase === "ascending" ? "ascending" : phase === "apogee" ? "armed" : "descending",
    signalStrength: 85 + Math.random() * 15,
    dataRate: 50 + Math.random() * 10,
  }
}

export function getMissionStatus(missionTime: number): MissionStatus {
  const progress = Math.min(missionTime / 600000, 1)
  let status: MissionStatus["status"] = "pre-launch"

  if (progress < 0.3) status = "ascending"
  else if (progress < 0.5) status = "apogee"
  else if (progress < 0.9) status = "descending"
  else status = "landed"

  return {
    missionName: "CanSat Mission 2025",
    launchTime: LAUNCH_TIME,
    currentTime: Date.now(),
    status,
    maxAltitude: 4000,
    flightTime: missionTime,
    recoveryLocation: progress > 0.9 ? { lat: 37.7749, lng: -122.4194 } : undefined,
  }
}
