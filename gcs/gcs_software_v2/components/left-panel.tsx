"use client"

import type { TelemetryData } from "@/lib/telemetry-types"
import { GPSMap } from "@/components/gps-map"

interface LeftPanelProps {
  telemetry?: TelemetryData
}

export function LeftPanel({ telemetry }: LeftPanelProps) {
  return (
    <div className="flex flex-col gap-4 p-4 h-full">
      {/* Map */}
      <div className="flex-1 min-h-0">
        <GPSMap telemetry={telemetry} />
      </div>
    </div>
  )
}
