"use client"

import { useState, useEffect } from "react"
import type { MissionStatus, TelemetryData, ConnectionState } from "@/lib/telemetry-types"

interface MissionStatusBarProps {
  status: MissionStatus
  telemetry?: TelemetryData
  connection?: ConnectionState
}

export function MissionStatusBar({ status, telemetry, connection }: MissionStatusBarProps) {
  const [currentTime, setCurrentTime] = useState<string>("")

  useEffect(() => {
    const updateTime = () => {
      setCurrentTime(new Date().toLocaleTimeString())
    }
    updateTime()
    const interval = setInterval(updateTime, 1000)
    return () => clearInterval(interval)
  }, [])

  const formatTime = (ms: number) => {
    const seconds = Math.floor((ms / 1000) % 60)
    const minutes = Math.floor((ms / 60000) % 60)
    const hours = Math.floor(ms / 3600000)
    return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`
  }

  const getFlightState = () => {
    // Use telemetry flight state if available
    if (telemetry?.flightState !== undefined && telemetry?.flightState !== null) {
      const stateMap: { [key: number]: string } = {
        0: "BOOT",
        1: "TEST_MODE",
        2: "LAUNCH_PAD",
        3: "ASCENT",
        4: "ROCKET_DEPLOY",
        5: "DESCENT",
        6: "AEROBRAKE_RELEASE",
        7: "IMPACT"
      }
      
      return stateMap[telemetry.flightState] || "UNKNOWN"
    }
    
    // Fallback to status
    return status.status || "IDLE"
  }

  const getStatusColor = (flightState: string) => {
    switch (flightState) {
      case "ASCENT":
      case "ROCKET_DEPLOY":
        return "bg-mission-active"
      case "DESCENT":
      case "AEROBRAKE_RELEASE":
        return "bg-mission-warning"
      case "IMPACT":
        return "bg-destructive"
      case "BOOT":
      case "TEST_MODE":
      case "LAUNCH_PAD":
        return "bg-blue-500"
      default:
        return "bg-muted"
    }
  }

  const currentFlightState = getFlightState()
  const flightTime = telemetry?.missionTime ?? status?.flightTime ?? 0
  const maxAltitude = telemetry?.altitude ?? status?.maxAltitude ?? 0

  return (
    <div className="border-b border-border bg-card px-6 py-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          {/* Flight State */}
          <div className="flex items-center gap-2">
            <div className={`h-3 w-3 rounded-full ${getStatusColor(currentFlightState)}`} />
            <span className="text-sm font-semibold uppercase text-foreground">{currentFlightState}</span>
          </div>

          <div className="h-4 w-px bg-border" />

          {/* Flight Time & Max Altitude */}
          <div className="flex gap-6 text-sm">
            <div>
              <span className="text-muted-foreground">Flight Time: </span>
              <span className="font-mono font-semibold text-foreground">{formatTime(flightTime)}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Max Altitude: </span>
              <span className="font-mono font-semibold text-foreground">{maxAltitude.toFixed(0)} m</span>
            </div>
          </div>
        </div>

        <div className="text-right text-xs text-muted-foreground">
          <div>Mission: {status?.missionName || "CanSat Mission"}</div>
          <div>UTC: {currentTime || "--:--:--"}</div>
        </div>
      </div>
    </div>
  )
}
