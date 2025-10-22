import type { TelemetryData } from "./telemetry-types"

const MAX_DATA_POINTS = 100

export class ChartDataManager {
  private reactionWheelData: Array<{ time: number; x: number; y: number; z: number }> = []
  private gyroData: Array<{ time: number; x: number; y: number; z: number }> = []
  private accelData: Array<{ time: number; x: number; y: number; z: number }> = []
  private altitudeData: Array<{ time: number; value: number }> = []
  private temperatureData: Array<{ time: number; value: number }> = []
  private pressureData: Array<{ time: number; value: number }> = []
  private batteryData: Array<{ time: number; voltage: number; percentage: number }> = []

  addTelemetry(telemetry: TelemetryData) {
    const time = telemetry.missionTime

    this.reactionWheelData.push({
      time,
      x: telemetry.reactionWheelX,
      y: telemetry.reactionWheelY,
      z: telemetry.reactionWheelZ,
    })

    this.gyroData.push({
      time,
      x: telemetry.gyroX,
      y: telemetry.gyroY,
      z: telemetry.gyroZ,
    })

    this.accelData.push({
      time,
      x: telemetry.accelX,
      y: telemetry.accelY,
      z: telemetry.accelZ,
    })

    this.altitudeData.push({
      time,
      value: telemetry.altitude,
    })

    this.temperatureData.push({
      time,
      value: telemetry.temperature,
    })

    this.pressureData.push({
      time,
      value: telemetry.pressure,
    })

    this.batteryData.push({
      time,
      voltage: telemetry.batteryVoltage,
      percentage: telemetry.batteryPercentage,
    })

    // Keep only last MAX_DATA_POINTS
    this.trimData()
  }

  private trimData() {
    if (this.reactionWheelData.length > MAX_DATA_POINTS) {
      this.reactionWheelData = this.reactionWheelData.slice(-MAX_DATA_POINTS)
      this.gyroData = this.gyroData.slice(-MAX_DATA_POINTS)
      this.accelData = this.accelData.slice(-MAX_DATA_POINTS)
      this.altitudeData = this.altitudeData.slice(-MAX_DATA_POINTS)
      this.temperatureData = this.temperatureData.slice(-MAX_DATA_POINTS)
      this.pressureData = this.pressureData.slice(-MAX_DATA_POINTS)
      this.batteryData = this.batteryData.slice(-MAX_DATA_POINTS)
    }
  }

  getReactionWheelData() {
    return [...this.reactionWheelData]
  }

  getGyroData() {
    return [...this.gyroData]
  }

  getAccelData() {
    return [...this.accelData]
  }

  getAltitudeData() {
    return [...this.altitudeData]
  }

  getTemperatureData() {
    return [...this.temperatureData]
  }

  getPressureData() {
    return [...this.pressureData]
  }

  getBatteryData() {
    return [...this.batteryData]
  }

  clear() {
    this.reactionWheelData = []
    this.gyroData = []
    this.accelData = []
    this.altitudeData = []
    this.temperatureData = []
    this.pressureData = []
    this.batteryData = []
  }
}
