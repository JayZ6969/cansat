"use client"

import { useEffect, useState } from "react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { ChartDataManager } from "@/lib/chart-data-manager"
import { TelemetryChart } from "@/components/charts/telemetry-chart"
import { MultiAxisChart } from "@/components/charts/multi-axis-chart"
import { GaugeChart } from "@/components/charts/gauge-chart"

interface CenterPanelProps {
  telemetry?: TelemetryData
}

export function CenterPanel({ telemetry }: CenterPanelProps) {
  const [chartManager] = useState(() => new ChartDataManager())
  const [reactionWheelData, setReactionWheelData] = useState<any[]>([])
  const [gyroData, setGyroData] = useState<any[]>([])
  const [accelData, setAccelData] = useState<any[]>([])
  const [altitudeData, setAltitudeData] = useState<any[]>([])
  const [temperatureData, setTemperatureData] = useState<any[]>([])
  const [batteryData, setBatteryData] = useState<any[]>([])

  useEffect(() => {
    if (telemetry) {
      chartManager.addTelemetry(telemetry)
      setReactionWheelData([...chartManager.getReactionWheelData()])
      setGyroData([...chartManager.getGyroData()])
      setAccelData([...chartManager.getAccelData()])
      setAltitudeData([...chartManager.getAltitudeData()])
      setTemperatureData([...chartManager.getTemperatureData()])
      setBatteryData([...chartManager.getBatteryData()])
    }
  }, [telemetry, chartManager])

  // Show message when no data is available
  if (!telemetry && reactionWheelData.length === 0) {
    return (
      <div className="flex items-center justify-center h-full p-4">
        <div className="text-center space-y-2">
          <p className="text-lg font-semibold text-muted-foreground">No Telemetry Data</p>
          <p className="text-sm text-muted-foreground">Connect to a COM port to start receiving data</p>
        </div>
      </div>
    )
  }

  return (
    <div className="grid grid-cols-2 gap-4 p-4">
      <MultiAxisChart
        title="Reaction Wheel"
        subtitle="X/Y/Z Angular Velocity"
        data={reactionWheelData}
        series={[
          { key: "x", name: "X", color: "var(--color-chart-1)" },
          { key: "y", name: "Y", color: "var(--color-chart-2)" },
          { key: "z", name: "Z", color: "var(--color-chart-3)" },
        ]}
      />

      <MultiAxisChart
        title="Gyroscope"
        subtitle="Rotation Rates"
        data={gyroData}
        series={[
          { key: "x", name: "X", color: "var(--color-chart-1)" },
          { key: "y", name: "Y", color: "var(--color-chart-2)" },
          { key: "z", name: "Z", color: "var(--color-chart-3)" },
        ]}
      />

      <TelemetryChart
        title="Altitude"
        subtitle="Height vs Time"
        data={altitudeData}
        color="var(--color-chart-2)"
        unit="m"
      />

      <TelemetryChart
        title="Temperature"
        subtitle="Thermal Profile"
        data={temperatureData}
        color="var(--color-chart-5)"
        unit="°C"
      />

      <MultiAxisChart
        title="Accelerometer"
        subtitle="Linear Acceleration"
        data={accelData}
        series={[
          { key: "x", name: "X", color: "var(--color-chart-1)" },
          { key: "y", name: "Y", color: "var(--color-chart-2)" },
          { key: "z", name: "Z", color: "var(--color-chart-3)" },
        ]}
      />

      {batteryData.length > 0 && (
        <GaugeChart
          title="Battery Status"
          value={batteryData[batteryData.length - 1]?.percentage || 0}
          max={100}
          unit="%"
          color="var(--color-mission-active)"
        />
      )}
    </div>
  )
}
