"use client"

import { useState, useEffect, useRef } from "react"
import type { TelemetryData } from "@/lib/telemetry-types"
import { Card } from "@/components/ui/card"
import { SystemDiagnosticsPanel } from "@/components/system-diagnostics-panel"
import { MissionControlPanel } from "@/components/mission-control-panel"
import { SettingsSidebar } from "@/components/settings-sidebar"
import { Terminal } from "lucide-react"

interface RightPanelProps {
  telemetry?: TelemetryData
  isConnected?: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

interface LogEntry {
  timestamp: string
  message: string
  type: 'info' | 'warning' | 'error' | 'success'
}

export function RightPanel({ telemetry, isConnected = true, onConnect, onDisconnect }: RightPanelProps) {
  const [isSettingsOpen, setIsSettingsOpen] = useState(false)
  const [logs, setLogs] = useState<LogEntry[]>([])
  const logEndRef = useRef<HTMLDivElement>(null)
  
  const formatValue = (value: number | undefined, decimals = 2) => {
    if (value === undefined || value === null || isNaN(value)) return '0.00'
    return value.toFixed(decimals)
  }

  // Add log entry
  const addLog = (message: string, type: LogEntry['type'] = 'info') => {
    const timestamp = new Date().toLocaleTimeString('en-US', { hour12: false })
    setLogs(prev => [...prev.slice(-99), { timestamp, message, type }]) // Keep last 100 logs
  }

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [logs])

  // Monitor telemetry for log messages
  useEffect(() => {
    if (telemetry) {
      // Check for log messages in telemetry
      if ((telemetry as any).logMessage) {
        addLog((telemetry as any).logMessage, 'info')
      }
      
      // Log flight state changes
      const flightStateNames = ['BOOT', 'TEST_MODE', 'LAUNCH_PAD', 'ASCENT', 'ROCKET_DEPLOY', 'DESCENT', 'AEROBRAKE_RELEASE', 'IMPACT']
      const currentState = flightStateNames[telemetry.flightState || 0]
      
      // Store previous state to detect changes
      const prevState = (window as any).__prevFlightState
      if (prevState !== undefined && prevState !== telemetry.flightState) {
        addLog(`Flight state: ${currentState}`, 'success')
      }
      (window as any).__prevFlightState = telemetry.flightState
      
      // Log error code changes
      const errorCode = (telemetry as any).errorCode || (telemetry as any).ERROR_CODE || '0'
      const prevErrorCode = (window as any).__prevErrorCode
      if (prevErrorCode !== undefined && prevErrorCode !== errorCode && errorCode !== '0') {
        addLog(`Error detected: ${errorCode}`, 'error')
      }
      (window as any).__prevErrorCode = errorCode
    }
  }, [telemetry])

  const getLogColor = (type: LogEntry['type']) => {
    switch (type) {
      case 'error': return 'text-destructive'
      case 'warning': return 'text-mission-warning'
      case 'success': return 'text-mission-active'
      default: return 'text-foreground'
    }
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
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.latitude, 6)}</span>
              </div>
              <div className="flex justify-between">
                <span className="text-muted-foreground">Longitude:</span>
                <span className="font-mono font-semibold text-foreground">{formatValue(telemetry.longitude, 6)}</span>
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

      {/* CanSat Log Terminal */}
      <Card className="p-4 flex-1 flex flex-col min-h-[200px]">
        <div className="flex items-center gap-2 mb-3">
          <Terminal className="h-4 w-4" />
          <h3 className="text-sm font-semibold text-foreground">CanSat Logs</h3>
        </div>
        <div className="flex-1 overflow-y-auto bg-black/50 rounded p-2 font-mono text-xs space-y-1 min-h-0">
          {logs.length > 0 ? (
            <>
              {logs.map((log, index) => (
                <div key={index} className="flex gap-2">
                  <span className="text-muted-foreground shrink-0">[{log.timestamp}]</span>
                  <span className={getLogColor(log.type)}>{log.message}</span>
                </div>
              ))}
              <div ref={logEndRef} />
            </>
          ) : (
            <div className="text-muted-foreground">Waiting for logs...</div>
          )}
        </div>
      </Card>
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
