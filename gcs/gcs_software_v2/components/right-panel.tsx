"use client"

import { useState } from "react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { Card } from "@/components/ui/card"
import { SystemDiagnosticsPanel } from "@/components/system-diagnostics-panel"
import { MissionControlPanel } from "@/components/mission-control-panel"
import { SettingsSidebar } from "@/components/settings-sidebar"
import { Terminal } from "@/components/terminal"
import { isValidGPSCoordinate } from "@/lib/gps-utils"

interface RightPanelProps {
  telemetry?: TelemetryData
  isConnected?: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

export function RightPanel({ telemetry, isConnected = true, onConnect, onDisconnect }: RightPanelProps) {
  const [isSettingsOpen, setIsSettingsOpen] = useState(false)
  
  const formatValue = (value: number | undefined, decimals = 2) => {
    if (value === undefined || value === null || isNaN(value)) return '0.00'
    return value.toFixed(decimals)
  }

  return (
    <>
      <div className="flex flex-col gap-4 p-4 h-full overflow-y-auto">
        {/* Mission Control Panel */}
        <MissionControlPanel 
          onSettingsClick={() => setIsSettingsOpen(true)}
          telemetry={telemetry}
        />

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
                <span className="text-muted-foreground">Pressure:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.pressure)} Pa</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Battery:</span>
                <span className="font-mono font-semibold text-foreground">
                  {formatValue(telemetry.batteryPercentage)} %
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Battery Voltage:</span>
                <span className="font-mono font-semibold text-foreground">
                  {formatValue(telemetry.batteryVoltage)} V
                </span>
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

      {/* GNSS Data */}
      <Card className="p-4">
        <h3 className="text-sm font-semibold text-foreground mb-3">GNSS Data</h3>
        <div className="space-y-2 text-xs">
          {telemetry ? (
            <>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Latitude:</span>
                <span className={`font-mono font-semibold ${
                  isValidGPSCoordinate(telemetry.latitude, telemetry.longitude) 
                    ? "text-foreground" 
                    : "text-red-500"
                }`}>
                  {isValidGPSCoordinate(telemetry.latitude, telemetry.longitude) 
                    ? formatValue(telemetry.latitude, 6) 
                    : "No GPS Lock"
                  }
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Longitude:</span>
                <span className={`font-mono font-semibold ${
                  isValidGPSCoordinate(telemetry.latitude, telemetry.longitude) 
                    ? "text-foreground" 
                    : "text-red-500"
                }`}>
                  {isValidGPSCoordinate(telemetry.latitude, telemetry.longitude) 
                    ? formatValue(telemetry.longitude, 6) 
                    : "No GPS Lock"
                  }
                </span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">GNSS Altitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.gpsAltitude)} m</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Speed:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.gpsSpeed)} m/s</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Satellites:</span>
                <span className="font-mono font-semibold text-foreground">{telemetry.gpsSatellites}</span>
              </div>
            </>
          ) : (
            <p className="text-muted-foreground">Waiting for GNSS...</p>
          )}
        </div>
      </Card>

      {/* Serial Terminal */}
      <Terminal className="mt-4" />

    </div>

      {/* Settings Sidebar */}
      <SettingsSidebar 
        isOpen={isSettingsOpen} 
        onClose={() => setIsSettingsOpen(false)}
        isConnected={isConnected}
        onConnect={onConnect}
        onDisconnect={onDisconnect}
      />
    </>
  )
}
