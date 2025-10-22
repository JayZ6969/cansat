"use client"

import { useState, useEffect, useRef } from "react"
import { DashboardHeader } from "@/components/dashboard-header"
import { MissionStatusBar } from "@/components/mission-status-bar"
import { DashboardLayout } from "@/components/dashboard-layout"
import { LeftPanel } from "@/components/left-panel"
import { CenterPanel } from "@/components/center-panel"
import { RightPanel } from "@/components/right-panel"
import { useTelemetry } from "@/lib/use-telemetry"
import type { MissionStatus } from "@/lib/telemetry-types"

export default function Dashboard() {
  const { telemetry, connectionState, missionTime, connect, disconnect } = useTelemetry({
    useMockData: false, // Use real WebSocket connection
    wsUrl: process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:8000/ws/telemetry',
  })

  const [missionStatus, setMissionStatus] = useState<MissionStatus | null>(null)
  const maxAltitudeRef = useRef<number>(0)
  const missionStartTimeRef = useRef<number>(0)

  useEffect(() => {
    if (telemetry) {
      // Track maximum altitude
      if (telemetry.altitude > maxAltitudeRef.current) {
        console.log(`[Max Altitude Updated] ${maxAltitudeRef.current.toFixed(2)} → ${telemetry.altitude.toFixed(2)} m`)
        maxAltitudeRef.current = telemetry.altitude
      }

      // Set mission start time on first telemetry packet
      if (missionStartTimeRef.current === 0) {
        missionStartTimeRef.current = Date.now()
      }

      // Mission time is already in milliseconds from the CanSat
      const flightTime = telemetry.missionTime

      // Create mission status from telemetry
      const newStatus: MissionStatus = {
        missionName: "CanSat Mission 2025",
        launchTime: missionStartTimeRef.current,
        currentTime: Date.now(),
        status: telemetry.systemStatus,
        maxAltitude: maxAltitudeRef.current,
        flightTime: flightTime,
        recoveryLocation: telemetry.systemStatus === "landed" 
          ? { lat: telemetry.latitude, lng: telemetry.longitude } 
          : undefined,
      }
      
      setMissionStatus(newStatus)
    }
  }, [telemetry])

  return (
    <DashboardLayout
      header={
        <DashboardHeader
          isConnected={connectionState.isConnected}
          signalStrength={telemetry?.signalStrength || 0}
          missionName={missionStatus?.missionName || "CanSat Mission"}
          cansatBattery={telemetry?.batteryPercentage || 0}
          rssi={telemetry?.rssi}
          snr={telemetry?.snr}
        />
      }
      statusBar={<MissionStatusBar status={missionStatus || ({} as MissionStatus)} />}
      leftPanel={<LeftPanel telemetry={telemetry || undefined} />}
      centerPanel={<CenterPanel telemetry={telemetry || undefined} />}
      rightPanel={
        <RightPanel
          telemetry={telemetry || undefined}
          isConnected={connectionState.isConnected}
          onConnect={connect}
          onDisconnect={disconnect}
        />
      }
    />
  )
}
