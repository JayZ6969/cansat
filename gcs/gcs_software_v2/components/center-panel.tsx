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
  const [pressureData, setPressureData] = useState<any[]>([])

  useEffect(() => {
    if (telemetry) {
      chartManager.addTelemetry(telemetry)
      setReactionWheelData([...chartManager.getReactionWheelData()])
      setGyroData([...chartManager.getGyroData()])
      setAccelData([...chartManager.getAccelData()])
      setAltitudeData([...chartManager.getAltitudeData()])
      setTemperatureData([...chartManager.getTemperatureData()])
      setPressureData([...chartManager.getPressureData()])
    }
  }, [telemetry, chartManager])

  return (
    <div className="grid grid-cols-2 gap-4 p-4">
      {/* 1. Gyroscope */}
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

      {/* 2. Accelerometer */}
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

      {/* 3. Reaction Wheel */}
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

      {/* 4. Altitude */}
      <TelemetryChart
        title="Altitude"
        subtitle="Height vs Time"
        data={altitudeData}
        color="var(--color-chart-2)"
        unit="m"
      />

      {/* 5. Temperature */}
      <TelemetryChart
        title="Temperature"
        subtitle="Thermal Profile"
        data={temperatureData}
        color="var(--color-chart-5)"
        unit="°C"
      />

      {/* 6. Pressure */}
      <TelemetryChart
        title="Pressure"
        subtitle="Atmospheric Pressure"
        data={pressureData}
        color="var(--color-chart-4)"
        unit="kPa"
      />
    </div>
  )
}
