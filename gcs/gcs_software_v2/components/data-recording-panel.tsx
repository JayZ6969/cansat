"use client"

import { useState } from "react"
import { HardDrive, Play, Square, Download } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"

export function DataRecordingPanel() {
  const [isRecording, setIsRecording] = useState(false)
  const [recordedPoints, setRecordedPoints] = useState(0)

  return (
    <Card className="p-4">
      <div className="flex items-center gap-2 mb-4">
        <HardDrive className="h-4 w-4 text-muted-foreground" />
        <h3 className="text-sm font-semibold text-foreground">Data Recording</h3>
      </div>

      <div className="space-y-3">
        <div className="text-xs">
          <div className="flex justify-between mb-1">
            <span className="text-muted-foreground">Status:</span>
            <span className={`font-semibold ${isRecording ? "text-mission-active" : "text-muted-foreground"}`}>
              {isRecording ? "Recording" : "Idle"}
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-muted-foreground">Points:</span>
            <span className="font-mono font-semibold text-foreground">{recordedPoints}</span>
          </div>
        </div>

        <div className="h-px bg-border" />

        <div className="flex gap-2">
          <Button
            onClick={() => setIsRecording(!isRecording)}
            size="sm"
            className="flex-1"
            variant={isRecording ? "destructive" : "default"}
          >
            {isRecording ? (
              <>
                <Square className="h-3 w-3 mr-1" />
                Stop
              </>
            ) : (
              <>
                <Play className="h-3 w-3 mr-1" />
                Start
              </>
            )}
          </Button>
          <Button size="sm" variant="outline" className="flex-1 bg-transparent">
            <Download className="h-3 w-3 mr-1" />
            Export
          </Button>
        </div>
      </div>
    </Card>
  )
}
