"use client"

import { AlertCircle, CheckCircle2 } from "lucide-react"
import { Card } from "@/components/ui/card"

interface SystemDiagnosticsPanelProps {
  telemetry?: any
}

export function SystemDiagnosticsPanel({ telemetry }: SystemDiagnosticsPanelProps) {
  const diagnostics = [
    {
      name: "GPS Lock",
      status: telemetry?.gpsSatellites >= 10 ? "ok" : "warning",
      value: `${telemetry?.gpsSatellites || 0} satellites`,
    },
    {
      name: "Battery",
      status: telemetry?.batteryPercentage > 20 ? "ok" : "critical",
      value: `${telemetry?.batteryPercentage.toFixed(0) || 0}%`,
    },
    {
      name: "Signal",
      status: telemetry?.signalStrength > 50 ? "ok" : "warning",
      value: `${telemetry?.signalStrength.toFixed(0) || 0}%`,
    },
    {
      name: "Sensors",
      status: "ok",
      value: "All nominal",
    },
  ]

  return (
    <Card className="p-4">
      <h3 className="text-sm font-semibold text-foreground mb-3">System Diagnostics</h3>
      <div className="space-y-2">
        {diagnostics.map((diag) => (
          <div key={diag.name} className="flex items-center justify-between text-xs">
            <div className="flex items-center gap-2">
              {diag.status === "ok" ? (
                <CheckCircle2 className="h-3 w-3 text-mission-active" />
              ) : diag.status === "warning" ? (
                <AlertCircle className="h-3 w-3 text-mission-warning" />
              ) : (
                <AlertCircle className="h-3 w-3 text-destructive" />
              )}
              <span className="text-muted-foreground">{diag.name}</span>
            </div>
            <span className="font-mono font-semibold text-foreground">{diag.value}</span>
          </div>
        ))}
      </div>
    </Card>
  )
}
