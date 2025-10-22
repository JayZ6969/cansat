"use client"

import type { MissionStatus } from "@/lib/telemetry-types"

interface MissionStatusBarProps {
  status: MissionStatus
}

export function MissionStatusBar({ status }: MissionStatusBarProps) {
  const formatTime = (ms: number) => {
    const seconds = Math.floor((ms / 1000) % 60)
    const minutes = Math.floor((ms / 60000) % 60)
    const hours = Math.floor(ms / 3600000)
    return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case "boot":
      case "test-mode":
        return "bg-muted"
      case "launchpad":
        return "bg-yellow-500"
      case "ascending":
      case "rocket-deploy":
        return "bg-mission-active"
      case "descending":
      case "aerobrake-release":
        return "bg-secondary"
      case "landed":
        return "bg-green-500"
      default:
        return "bg-muted"
    }
  }

  return (
    <div className="border-b border-border bg-card px-6 py-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <div className={`h-3 w-3 rounded-full ${getStatusColor(status.status)}`} />
            <span className="text-sm font-semibold uppercase text-foreground">{status.status}</span>
          </div>

          <div className="h-4 w-px bg-border" />

          <div className="flex gap-6 text-sm">
            <div>
              <span className="text-muted-foreground">Flight Time: </span>
              <span className="font-mono font-semibold text-foreground">{formatTime(status.flightTime || 0)}</span>
            </div>
            <div>
              <span className="text-muted-foreground">Max Altitude: </span>
              <span className="font-mono font-semibold text-foreground">{(status.maxAltitude || 0).toFixed(0)} m</span>
            </div>
          </div>
        </div>

        <div className="text-right text-xs text-muted-foreground">
          <div>Mission: {status.missionName}</div>
          <div>UTC: {new Date(status.currentTime).toLocaleTimeString()}</div>
        </div>
      </div>
    </div>
  )
}
