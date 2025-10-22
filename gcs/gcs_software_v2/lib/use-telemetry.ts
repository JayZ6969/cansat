"use client"

import { useEffect, useState, useCallback, useRef } from "react"
import type { TelemetryData, ConnectionState } from "./telemetry-types"
import { WebSocketClient } from "./websocket-client"
import { generateMockTelemetry } from "./mock-data"

interface UseTelemetryOptions {
  useMockData?: boolean
  wsUrl?: string
}

export function useTelemetry(options: UseTelemetryOptions = {}) {
  const { useMockData = true, wsUrl } = options
  const [telemetry, setTelemetry] = useState<TelemetryData | null>(null)
  const [connectionState, setConnectionState] = useState<ConnectionState>({
    isConnected: false,
    connectionType: "none",
    lastUpdate: 0,
    messageCount: 0,
    errorCount: 0,
    espConnected: false,
    loraConnected: false,
  })
  const [missionTime, setMissionTime] = useState(0)
  const wsClientRef = useRef<WebSocketClient | null>(null)
  const missionTimeRef = useRef(0)

  // Initialize WebSocket if URL provided
  useEffect(() => {
    if (!useMockData && wsUrl) {
      wsClientRef.current = new WebSocketClient(wsUrl)

      wsClientRef.current.on("connected", () => {
        setConnectionState((prev) => ({
          ...prev,
          isConnected: true,
          connectionType: "websocket",
          espConnected: true,
        }))
      })

      wsClientRef.current.on("serial_status", (event) => {
        if (event.data && typeof event.data === 'object') {
          const statusData = event.data as any
          console.log("[GCS] Serial status changed:", statusData)
          
          setConnectionState((prev) => ({
            ...prev,
            espConnected: statusData.status === 'connected',
          }))
        }
      })

      wsClientRef.current.on("data", (event) => {
        if (event.data && typeof event.data !== 'string') {
          setTelemetry(event.data)
          
          // Check if LoRa is connected based on presence of RSSI/SNR
          const loraConnected = event.data.rssi !== undefined && event.data.rssi !== null
          
          setConnectionState((prev) => ({
            ...prev,
            lastUpdate: Date.now(),
            messageCount: prev.messageCount + 1,
            loraConnected,
          }))
        }
      })

      wsClientRef.current.on("disconnected", () => {
        setConnectionState((prev) => ({
          ...prev,
          isConnected: false,
          connectionType: "none",
          espConnected: false,
          loraConnected: false,
        }))
      })

      wsClientRef.current.on("error", () => {
        setConnectionState((prev) => ({
          ...prev,
          errorCount: prev.errorCount + 1,
        }))
      })

      wsClientRef.current.connect().catch((error) => {
        console.error("[v0] Failed to connect WebSocket:", error)
      })

      return () => {
        wsClientRef.current?.disconnect()
      }
    }
  }, [useMockData, wsUrl])

  // Mock data generation with timer
  useEffect(() => {
    if (!useMockData) return

    const interval = setInterval(() => {
      const newTime = missionTimeRef.current + 1000
      missionTimeRef.current = newTime
      setMissionTime(newTime)
      
      // Generate and set telemetry data
      const newTelemetry = generateMockTelemetry(newTime)
      setTelemetry(newTelemetry)
      setConnectionState((prev) => ({
        ...prev,
        isConnected: true,
        connectionType: "http",
        lastUpdate: Date.now(),
        messageCount: prev.messageCount + 1,
      }))
    }, 1000)

    return () => clearInterval(interval)
  }, [useMockData])

  const connect = useCallback(async () => {
    if (wsClientRef.current && !wsClientRef.current.isConnected()) {
      try {
        await wsClientRef.current.connect()
      } catch (error) {
        console.error("[v0] Connection failed:", error)
      }
    }
  }, [])

  const disconnect = useCallback(() => {
    if (wsClientRef.current) {
      wsClientRef.current.disconnect()
    }
  }, [])

  return {
    telemetry,
    connectionState,
    missionTime,
    connect,
    disconnect,
  }
}
