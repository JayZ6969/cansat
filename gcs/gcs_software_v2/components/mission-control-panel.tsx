"use client"

import { useState, useEffect } from "react"
import { Play, Square, Settings } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import type { TelemetryData } from "@/lib/telemetry-types"

interface MissionControlPanelProps {
  onSettingsClick: () => void
  telemetry?: TelemetryData
}

export function MissionControlPanel({ onSettingsClick, telemetry }: MissionControlPanelProps) {
  const [isMissionActive, setIsMissionActive] = useState(false)
  const [missionStartTime, setMissionStartTime] = useState<number | null>(null)
  const [recordCount, setRecordCount] = useState(0)
  const [csvWriter, setCsvWriter] = useState<any>(null)

  const startMission = async () => {
    try {
      // Get settings from localStorage
      const saved = localStorage.getItem('gcs_settings')
      let csvPath = './telemetry_data.csv'
      if (saved) {
        const settings = JSON.parse(saved)
        csvPath = settings.csvPath || csvPath
      }

      // Generate timestamp-based filename
      const timestamp = new Date().toISOString().replace(/[:.]/g, '-').slice(0, -5)
      const filename = csvPath.replace('.csv', `_${timestamp}.csv`)

      // Request backend to start CSV recording
      const response = await fetch('http://localhost:5000/api/start-recording', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          filename: filename,
        }),
      })

      if (response.ok) {
        setIsMissionActive(true)
        setMissionStartTime(Date.now())
        setRecordCount(0)
        console.log('[Mission Control] Mission started, recording to:', filename)
      } else {
        console.error('[Mission Control] Failed to start recording')
        alert('Failed to start mission recording')
      }
    } catch (error) {
      console.error('[Mission Control] Error starting mission:', error)
      alert('Error starting mission: ' + error)
    }
  }

  const stopMission = async () => {
    try {
      // Request backend to stop CSV recording
      const response = await fetch('http://localhost:5000/api/stop-recording', {
        method: 'POST',
      })

      if (response.ok) {
        setIsMissionActive(false)
        setMissionStartTime(null)
        console.log('[Mission Control] Mission stopped, total records:', recordCount)
      } else {
        console.error('[Mission Control] Failed to stop recording')
      }
    } catch (error) {
      console.error('[Mission Control] Error stopping mission:', error)
    }
  }

  // Count records when mission is active
  useEffect(() => {
    if (isMissionActive && telemetry) {
      setRecordCount((prev) => prev + 1)
    }
  }, [telemetry, isMissionActive])

  const formatMissionTime = () => {
    if (!missionStartTime) return '00:00:00'
    const elapsed = Math.floor((Date.now() - missionStartTime) / 1000)
    const hours = Math.floor(elapsed / 3600)
    const minutes = Math.floor((elapsed % 3600) / 60)
    const seconds = elapsed % 60
    return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`
  }

  // Update mission time display
  const [missionTimeDisplay, setMissionTimeDisplay] = useState('00:00:00')
  useEffect(() => {
    if (isMissionActive) {
      const interval = setInterval(() => {
        setMissionTimeDisplay(formatMissionTime())
      }, 1000)
      return () => clearInterval(interval)
    }
  }, [isMissionActive, missionStartTime])

  return (
    <Card className="p-4">
      <div className="flex items-center justify-between mb-3">
        <h3 className="text-sm font-semibold text-foreground">Mission Control</h3>
        <Button variant="ghost" size="sm" onClick={onSettingsClick}>
          <Settings className="h-4 w-4" />
        </Button>
      </div>

      <div className="space-y-3">
        {/* Mission Status */}
        <div className="flex items-center gap-2">
          <span className="text-xs text-muted-foreground">Status:</span>
          <Badge variant={isMissionActive ? "default" : "outline"} className={isMissionActive ? "bg-mission-active" : ""}>
            {isMissionActive ? "ACTIVE" : "IDLE"}
          </Badge>
        </div>

        {/* Mission Time */}
        {isMissionActive && (
          <>
            <div className="flex items-center justify-between">
              <span className="text-xs text-muted-foreground">Mission Time:</span>
              <span className="text-sm font-mono font-semibold">{missionTimeDisplay}</span>
            </div>

            <div className="flex items-center justify-between">
              <span className="text-xs text-muted-foreground">Records:</span>
              <span className="text-sm font-mono font-semibold">{recordCount}</span>
            </div>
          </>
        )}

        {/* Control Buttons */}
        <div className="flex gap-2 pt-2">
          <Button
            className="flex-1 min-w-0"
            variant={isMissionActive ? "outline" : "default"}
            onClick={startMission}
            disabled={isMissionActive}
          >
            <Play className="h-4 w-4 mr-1.5 shrink-0" />
            <span className="truncate">Start Mission</span>
          </Button>
          <Button
            className="flex-1 min-w-0"
            variant={isMissionActive ? "destructive" : "outline"}
            onClick={stopMission}
            disabled={!isMissionActive}
          >
            <Square className="h-4 w-4 mr-1.5 shrink-0" />
            <span className="truncate">Stop Mission</span>
          </Button>
        </div>

        {isMissionActive && (
          <p className="text-xs text-muted-foreground text-center pt-2">
            Recording telemetry data to CSV...
          </p>
        )}
      </div>
    </Card>
  )
}
