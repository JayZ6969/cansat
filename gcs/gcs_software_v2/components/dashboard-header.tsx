"use client"

import { useState, useEffect } from "react"
import { Battery, BatteryMedium, BatteryLow } from "lucide-react"
import Image from "next/image"
import type { ConnectionState, TelemetryData } from "@/lib/telemetry-types"
import { ThemeToggle } from "./theme-toggle"

interface DashboardHeaderProps {
  isConnected: boolean
  signalStrength: number
  batteryPercentage?: number
  connection?: ConnectionState
  telemetry?: TelemetryData
}

export function DashboardHeader({ isConnected, signalStrength, batteryPercentage = 0, connection, telemetry }: DashboardHeaderProps) {
  const [currentTime, setCurrentTime] = useState<string>("")
  const [currentDate, setCurrentDate] = useState<string>("")

  useEffect(() => {
    const updateTime = () => {
      // Convert to IST (UTC+5:30)
      const date = new Date()
      const istTime = new Date(date.toLocaleString('en-US', { timeZone: 'Asia/Kolkata' }))
      setCurrentTime(istTime.toLocaleTimeString('en-US', { hour12: false }))
      setCurrentDate(istTime.toLocaleDateString('en-US', { 
        day: '2-digit', 
        month: 'short', 
        year: 'numeric' 
      }))
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
    return "IDLE"
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

  const getBatteryIcon = (percentage: number) => {
    if (percentage >= 60) {
      return <Battery className="h-5 w-5 fill-current" />
    } else if (percentage >= 30) {
      return <BatteryMedium className="h-5 w-5 fill-current" />
    } else {
      return <BatteryLow className="h-5 w-5 fill-current" />
    }
  }

  const getBatteryColor = (percentage: number) => {
    if (percentage >= 60) return "text-mission-active"
    if (percentage >= 30) return "text-mission-warning"
    return "text-destructive"
  }

  const getSignalStrengthIcon = (rssi: number) => {
    // RSSI ranges: -30 or better = excellent, -50 = good, -70 = fair, -90 = weak, -100 or worse = very weak
    const bars = rssi >= -30 ? 4 : rssi >= -50 ? 3 : rssi >= -70 ? 2 : rssi >= -90 ? 1 : 0
    const color = rssi >= -50 ? "text-mission-active" : rssi >= -70 ? "text-mission-warning" : "text-destructive"
    
    return (
      <div className={`flex items-end gap-[2px] h-4 ${color}`}>
        <div className={`w-1 h-1 rounded-sm ${bars >= 1 ? 'bg-current' : 'bg-muted-foreground/30'}`} />
        <div className={`w-1 h-2 rounded-sm ${bars >= 2 ? 'bg-current' : 'bg-muted-foreground/30'}`} />
        <div className={`w-1 h-3 rounded-sm ${bars >= 3 ? 'bg-current' : 'bg-muted-foreground/30'}`} />
        <div className={`w-1 h-4 rounded-sm ${bars >= 4 ? 'bg-current' : 'bg-muted-foreground/30'}`} />
      </div>
    )
  }

  const getConnectionStatus = () => {
    if (!connection?.espConnected) {
      return { text: "Disconnected", color: "text-destructive" }
    }
    if (!connection?.loraConnected) {
      return { text: "ESP Connected (LoRa Disconnected)", color: "text-mission-warning" }
    }
    return { text: "Connected", color: "text-mission-active" }
  }

  const connectionStatus = getConnectionStatus()
  const currentFlightState = getFlightState()
  const flightTime = telemetry?.missionTime ?? 0
  const maxAltitude = telemetry?.altitude ?? 0
  const velocity = telemetry?.gpsSpeed ?? 0

  return (
    <header className="border-b border-border bg-card">
      <div className="flex items-center justify-between px-6 py-4">
        {/* Left: Logo, Title, and Flight Info */}
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-3">
            <div className="flex h-10 w-10 items-center justify-center rounded-lg overflow-hidden bg-white">
              <Image 
                src="/AVINYA.png" 
                alt="Team Avinya Logo" 
                width={40} 
                height={40}
                className="object-contain"
              />
            </div>
            <div>
              <h1 className="text-xl font-bold text-foreground">CanSat GCS</h1>
              <p className="text-sm text-muted-foreground">Team Avinya</p>
            </div>
          </div>

          <div className="h-8 w-px bg-border" />

          {/* Flight State */}
          <div className="flex items-center gap-2">
            <div className={`h-3 w-3 rounded-full ${getStatusColor(currentFlightState)}`} />
            <span className="text-sm font-semibold uppercase text-foreground">{currentFlightState}</span>
          </div>

          <div className="h-6 w-px bg-border" />

          {/* Flight Time */}
          <div className="text-sm">
            <span className="text-muted-foreground">Flight Time: </span>
            <span className="font-mono font-semibold text-foreground">{formatTime(flightTime)}</span>
          </div>

          <div className="h-6 w-px bg-border" />

          {/* Max Altitude */}
          <div className="text-sm">
            <span className="text-muted-foreground">Max Altitude: </span>
            <span className="font-mono font-semibold text-foreground">{maxAltitude.toFixed(0)} m</span>
          </div>

          <div className="h-6 w-px bg-border" />

          {/* Velocity */}
          <div className="text-sm">
            <span className="text-muted-foreground">Velocity: </span>
            <span className="font-mono font-semibold text-foreground">{velocity.toFixed(1)} m/s</span>
          </div>
        </div>

        {/* Right: Connection, RSSI/SNR, Battery, Mission Info, IST */}
        <div className="flex items-center gap-4">
          {/* Connection Status with Network Icon */}
          <div className="flex flex-col items-end gap-1">
            <div className="flex items-center gap-2">
              {connection?.espConnected ? (
                getSignalStrengthIcon(telemetry?.rssi || -100)
              ) : (
                <div className="flex items-end gap-[2px] h-4 text-muted-foreground/30">
                  <div className="w-1 h-1 rounded-sm bg-current" />
                  <div className="w-1 h-2 rounded-sm bg-current" />
                  <div className="w-1 h-3 rounded-sm bg-current" />
                  <div className="w-1 h-4 rounded-sm bg-current" />
                </div>
              )}
              <span className={`text-sm font-medium ${connectionStatus.color}`}>
                {connectionStatus.text}
              </span>
            </div>
            {/* RSSI and SNR values */}
            {telemetry?.rssi !== undefined && (
              <div className="flex items-center gap-3 text-xs text-muted-foreground">
                <span>RSSI: <span className="font-mono font-semibold text-foreground">{telemetry.rssi.toFixed(0)} dB</span></span>
                {telemetry?.snr !== undefined && (
                  <span>SNR: <span className="font-mono font-semibold text-foreground">{telemetry.snr.toFixed(1)}</span></span>
                )}
              </div>
            )}
          </div>

          <div className="h-8 w-px bg-border" />

          {/* Battery */}
          <div className="flex items-center gap-2">
            <span className={getBatteryColor(batteryPercentage)}>
              {getBatteryIcon(batteryPercentage)}
            </span>
            <span className="text-sm font-medium text-foreground">{Math.round(batteryPercentage)}%</span>
          </div>

          <div className="h-8 w-px bg-border" />

          {/* Date and IST Time */}
          <div className="text-right text-xs">
            <div className="text-muted-foreground">Date: <span className="text-foreground font-medium">{currentDate || "--"}</span></div>
            <div className="text-muted-foreground">IST: <span className="font-mono text-foreground">{currentTime || "--:--:--"}</span></div>
          </div>

          <div className="h-8 w-px bg-border" />

          {/* Theme Toggle */}
          <ThemeToggle />
        </div>
      </div>
    </header>
  )
}
