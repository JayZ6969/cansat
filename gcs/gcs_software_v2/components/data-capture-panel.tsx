"use client"

import { useState } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Download, Play, Square, Usb } from "lucide-react"
import { Badge } from "@/components/ui/badge"

interface DataCapturePanelProps {
  isConnected?: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

export function DataCapturePanel({ isConnected, onConnect, onDisconnect }: DataCapturePanelProps) {
  const [isCapturing, setIsCapturing] = useState(false)
  const [port, setPort] = useState("COM3")
  const [fileName, setFileName] = useState(`mission_${new Date().toISOString().split('T')[0]}.csv`)
  const [recordCount, setRecordCount] = useState(0)

  const handleStartCapture = () => {
    setIsCapturing(true)
    setRecordCount(0)
    // TODO: Implement actual CSV writing logic
    console.log("Starting data capture to:", fileName)
  }

  const handleStopCapture = () => {
    setIsCapturing(false)
    console.log("Stopped data capture. Records:", recordCount)
  }

  const handleSerialConnect = () => {
    if (onConnect) {
      onConnect()
    }
  }

  const handleSerialDisconnect = () => {
    if (onDisconnect) {
      onDisconnect()
    }
  }

  return (
    <Card className="h-full flex flex-col">
      <CardHeader className="pb-3">
        <CardTitle className="text-lg flex items-center gap-2">
          <Download className="h-5 w-5" />
          Data Capture
        </CardTitle>
        <CardDescription>Serial connection & CSV recording</CardDescription>
      </CardHeader>
      <CardContent className="flex-1 space-y-4">
        {/* Serial Connection */}
        <div className="space-y-2">
          <Label htmlFor="port" className="text-xs">Serial Port</Label>
          <div className="flex gap-2">
            <Input
              id="port"
              value={port}
              onChange={(e) => setPort(e.target.value)}
              placeholder="COM3"
              className="flex-1"
              disabled={isConnected}
            />
            <Button
              size="sm"
              variant={isConnected ? "destructive" : "default"}
              onClick={isConnected ? handleSerialDisconnect : handleSerialConnect}
            >
              <Usb className="h-4 w-4 mr-1" />
              {isConnected ? "Disconnect" : "Connect"}
            </Button>
          </div>
          {isConnected && (
            <Badge variant="default" className="bg-mission-active">
              Connected to {port}
            </Badge>
          )}
        </div>

        {/* CSV Recording */}
        <div className="space-y-2">
          <Label htmlFor="filename" className="text-xs">Output File</Label>
          <Input
            id="filename"
            value={fileName}
            onChange={(e) => setFileName(e.target.value)}
            placeholder="mission_data.csv"
            disabled={isCapturing}
          />
        </div>

        {/* Capture Controls */}
        <div className="space-y-2">
          <Button
            className="w-full"
            variant={isCapturing ? "destructive" : "default"}
            onClick={isCapturing ? handleStopCapture : handleStartCapture}
            disabled={!isConnected}
          >
            {isCapturing ? (
              <>
                <Square className="h-4 w-4 mr-2" />
                Stop Recording
              </>
            ) : (
              <>
                <Play className="h-4 w-4 mr-2" />
                Start Recording
              </>
            )}
          </Button>

          {isCapturing && (
            <div className="text-center">
              <div className="text-2xl font-bold text-mission-active">{recordCount}</div>
              <div className="text-xs text-muted-foreground">Records captured</div>
            </div>
          )}
        </div>

        {/* Status */}
        <div className="pt-2 border-t border-border">
          <div className="flex items-center justify-between text-xs">
            <span className="text-muted-foreground">Status:</span>
            <Badge variant={isCapturing ? "default" : "outline"}>
              {isCapturing ? "Recording" : "Idle"}
            </Badge>
          </div>
        </div>
      </CardContent>
    </Card>
  )
}
