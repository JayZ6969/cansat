"use client"

import { Signal, Wifi, WifiOff, WifiLow, WifiHigh, BatteryCharging } from "lucide-react"
import { useEffect, useState } from "react"

interface DashboardHeaderProps {
  isConnected: boolean
  signalStrength: number
  missionName: string
  cansatBattery?: number
  rssi?: number
  snr?: number
}

interface SystemBattery {
  has_battery: boolean
  percentage: number
  is_charging: boolean
}

// Custom Battery Icon Component with dynamic fill
const BatteryIcon = ({ percentage, className }: { percentage: number; className?: string }) => {
  const isFull = percentage >= 50
  const color = percentage < 20 ? "text-red-500" : percentage < 50 ? "text-yellow-500" : "text-green-500"
  
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      width="20"
      height="20"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
      className={className || color}
    >
      {/* Battery outline */}
      <rect width="18" height="12" x="2" y="6" rx="2" ry="2" />
      <path d="M22 10v4" />
      
      {/* Battery fill - dynamic based on percentage */}
      {isFull ? (
        // Full battery (50-100%)
        <rect width="14" height="8" x="4" y="8" rx="1" fill="currentColor" opacity="0.8" />
      ) : (
        // Half battery (0-50%)
        <rect width="7" height="8" x="4" y="8" rx="1" fill="currentColor" opacity="0.8" />
      )}
    </svg>
  )
}

export function DashboardHeader({ 
  isConnected, 
  signalStrength, 
  missionName,
  cansatBattery = 0,
  rssi,
  snr
}: DashboardHeaderProps) {
  const [systemBattery, setSystemBattery] = useState<SystemBattery | null>(null)

  useEffect(() => {
    // Fetch GCS laptop battery status
    const fetchBattery = async () => {
      try {
        const response = await fetch(`${process.env.NEXT_PUBLIC_API_URL || 'http://localhost:8000'}/api/system/battery`)
        const data = await response.json()
        setSystemBattery(data)
      } catch (error) {
        console.error('Failed to fetch system battery:', error)
      }
    }

    fetchBattery()
    // Update every 30 seconds
    const interval = setInterval(fetchBattery, 30000)
    return () => clearInterval(interval)
  }, [])

  // Get WiFi icon based on RSSI
  const getSignalIcon = () => {
    if (!isConnected || rssi === undefined) {
      return <WifiOff className="h-5 w-5 text-destructive" />
    }

    // RSSI values typically range from -120 (worst) to 0 (best)
    // Good: > -50, Fair: -50 to -70, Poor: < -70
    if (rssi > -50) {
      return <Wifi className="h-5 w-5 text-mission-active" />
    } else if (rssi > -70) {
      return <WifiHigh className="h-5 w-5 text-yellow-500" />
    } else {
      return <WifiLow className="h-5 w-5 text-orange-500" />
    }
  }

  const getSignalColor = () => {
    if (!isConnected || rssi === undefined) return "text-destructive"
    if (rssi > -50) return "text-mission-active"
    if (rssi > -70) return "text-yellow-500"
    return "text-orange-500"
  }

  return (
    <header className="border-b border-border bg-card">
      <div className="flex items-center justify-between px-6 py-4">
        <div className="flex items-center gap-3">
          <div className="flex h-10 w-10 items-center justify-center rounded-lg bg-primary">
            <Signal className="h-5 w-5 text-primary-foreground" />
          </div>
          <div>
            <h1 className="text-xl font-bold text-foreground">CanSat GCS</h1>
            <p className="text-sm text-muted-foreground">{missionName}</p>
          </div>
        </div>

        <div className="flex items-center gap-4">
          {/* Signal Strength with RSSI/SNR */}
          <div className="flex items-center gap-2">
            {getSignalIcon()}
            <div className="flex flex-col">
              <span className={`text-sm font-medium ${getSignalColor()}`}>
                {isConnected ? 'Connected' : 'Disconnected'}
              </span>
              {rssi !== undefined && snr !== undefined && (
                <span className="text-xs text-muted-foreground">
                  RSSI: {rssi}dB | SNR: {snr.toFixed(1)}
                </span>
              )}
            </div>
          </div>

          <div className="h-8 w-px bg-border" />

          {/* CanSat Battery */}
          <div className="flex items-center gap-2">
            <BatteryIcon percentage={cansatBattery} />
            <div className="flex flex-col">
              <span className="text-xs text-muted-foreground">CanSat</span>
              <span className="text-sm font-medium text-foreground">{cansatBattery.toFixed(0)}%</span>
            </div>
          </div>

          {/* GCS Laptop Battery */}
          {systemBattery && systemBattery.has_battery && (
            <>
              <div className="h-8 w-px bg-border" />
              <div className="flex items-center gap-2">
                {systemBattery.is_charging ? (
                  <BatteryCharging className="h-5 w-5 text-green-500" />
                ) : (
                  <BatteryIcon percentage={systemBattery.percentage} />
                )}
                <div className="flex flex-col">
                  <span className="text-xs text-muted-foreground">GCS</span>
                  <span className="text-sm font-medium text-foreground">{systemBattery.percentage}%</span>
                </div>
              </div>
            </>
          )}
        </div>
      </div>
    </header>
  )
}
