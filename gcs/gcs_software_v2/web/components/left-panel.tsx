"use client"

import { Settings } from "lucide-react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { GPSMap } from "@/components/gps-map"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"

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

      {/* Controls */}
      <Card className="p-4">
        <h3 className="text-sm font-semibold text-foreground mb-3">Mission Controls</h3>
        <div className="flex flex-col gap-2">
          <Button variant="outline" size="sm" className="w-full bg-transparent">
            Arm System
          </Button>
          <Button variant="outline" size="sm" className="w-full bg-transparent">
            Disarm System
          </Button>
          <Button variant="outline" size="sm" className="w-full bg-transparent">
            Emergency Stop
          </Button>
        </div>
      </Card>

      {/* Settings */}
      <Card className="p-4">
        <div className="flex items-center gap-2 mb-3">
          <Settings className="h-4 w-4 text-muted-foreground" />
          <h3 className="text-sm font-semibold text-foreground">Settings</h3>
        </div>
        <div className="space-y-2 text-xs text-muted-foreground">
          <div>Update Rate: 50 Hz</div>
          <div>Buffer Size: 1000 points</div>
          <div>Recording: Enabled</div>
        </div>
      </Card>
    </div>
  )
}
