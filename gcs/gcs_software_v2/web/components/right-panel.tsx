"use client"

import type { TelemetryData } from "@/lib/telemetry-types"
import { Card } from "@/components/ui/card"
import { ConnectionPanel } from "@/components/connection-panel"
import { DataRecordingPanel } from "@/components/data-recording-panel"
import { SystemDiagnosticsPanel } from "@/components/system-diagnostics-panel"

interface RightPanelProps {
  telemetry?: TelemetryData
  isConnected?: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

export function RightPanel({ telemetry, isConnected = true, onConnect, onDisconnect }: RightPanelProps) {
  const formatValue = (value: number, decimals = 2) => value.toFixed(decimals)

  return (
    <div className="flex flex-col gap-4 p-4 h-full overflow-y-auto">
      {/* Connection Panel */}
      <ConnectionPanel isConnected={isConnected} onConnect={onConnect} onDisconnect={onDisconnect} />

      {/* Data Recording Panel */}
      <DataRecordingPanel />

      {/* System Diagnostics */}
      <SystemDiagnosticsPanel telemetry={telemetry} />

      {/* Real-time Telemetry */}
      <Card className="p-4">
        <h3 className="text-sm font-semibold text-foreground mb-3">Real-Time Telemetry</h3>
        <div className="space-y-2 text-xs">
          {telemetry ? (
            <>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Altitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.altitude)} m</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Temperature:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.temperature)} °C</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Battery:</span>
                <span className="font-mono font-semibold text-foreground">
                  {formatValue(telemetry.batteryPercentage)} %
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">GPS Satellites:</span>
                <span className="font-mono font-semibold text-foreground">{telemetry.gpsSatellites}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Data Rate:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.dataRate)} Hz</span>
              </div>
            </>
          ) : (
            <p className="text-muted-foreground">Waiting for data...</p>
          )}
        </div>
      </Card>

      {/* GPS Position */}
      <Card className="p-4">
        <h3 className="text-sm font-semibold text-foreground mb-3">GPS Position</h3>
        <div className="space-y-2 text-xs">
          {telemetry ? (
            <>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Latitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.latitude, 6)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Longitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.longitude, 6)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">GPS Altitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.gpsAltitude)} m</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Speed:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.gpsSpeed)} m/s</span>
              </div>
            </>
          ) : (
            <p className="text-muted-foreground">Waiting for GPS...</p>
          )}
        </div>
      </Card>

      {/* System Status */}
      <Card className="p-4">
        <h3 className="text-sm font-semibold text-foreground mb-3">System Status</h3>
        <div className="space-y-2 text-xs">
          {telemetry ? (
            <>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Status:</span>
                <span className="font-mono font-semibold text-foreground uppercase">{telemetry.systemStatus}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Battery Voltage:</span>
                <span className="font-mono font-semibold text-foreground">
                  {formatValue(telemetry.batteryVoltage)} V
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Pressure:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.pressure)} Pa</span>
              </div>
            </>
          ) : (
            <p className="text-muted-foreground">Waiting for status...</p>
          )}
        </div>
      </Card>

      {/* Event Timeline */}
      <Card className="p-4 flex-1">
        <h3 className="text-sm font-semibold text-foreground mb-3">Event Timeline</h3>
        <div className="space-y-2 text-xs text-muted-foreground overflow-y-auto max-h-40">
          <div className="flex gap-2">
            <span className="text-mission-active">●</span>
            <span>System armed</span>
          </div>
          <div className="flex gap-2">
            <span className="text-mission-active">●</span>
            <span>Launch detected</span>
          </div>
          <div className="flex gap-2">
            <span className="text-mission-warning">●</span>
            <span>Apogee reached</span>
          </div>
        </div>
      </Card>
    </div>
  )
}
