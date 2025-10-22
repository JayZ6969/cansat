"use client"

import { useState, useEffect } from "react"
import { DashboardHeader } from "@/components/dashboard-header"
import { DashboardLayout } from "@/components/dashboard-layout"
import { LeftPanel } from "@/components/left-panel"
import { CenterPanel } from "@/components/center-panel"
import { RightPanel } from "@/components/right-panel"
import { useTelemetry } from "@/lib/use-telemetry"
import { getMissionStatus } from "@/lib/mock-data"
import type { MissionStatus } from "@/lib/telemetry-types"

export default function Dashboard() {
  const { telemetry, connectionState, missionTime, connect, disconnect } = useTelemetry({
    useMockData: false, // Using real backend WebSocket connection
    wsUrl: process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:5000',
  })

  const [missionStatus, setMissionStatus] = useState<MissionStatus | null>(null)

  useEffect(() => {
    if (missionTime) {
      const newStatus = getMissionStatus(missionTime)
      setMissionStatus(newStatus)
    }
  }, [missionTime])

  return (
    <DashboardLayout
      header={
        <DashboardHeader
          isConnected={connectionState.isConnected}
          signalStrength={telemetry?.signalStrength || 0}
          batteryPercentage={telemetry?.batteryPercentage || 0}
          connection={connectionState}
          telemetry={telemetry || undefined}
        />
      }
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
