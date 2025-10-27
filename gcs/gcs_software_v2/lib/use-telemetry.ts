"use client"

import { useEffect, useState, useCallback, useRef } from "react"
import type { TelemetryData, ConnectionState } from "./telemetry-types"
import { WebSocketClient } from "./websocket-client"
import { generateMockTelemetry } from "./mock-data"
import NotificationService from "./notification-service"
import { sendLandingNotification, initializeEmailJS } from "./email-service"

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
  const lastLoRaDataRef = useRef<number>(0)
  const loraTimeoutRef = useRef<NodeJS.Timeout | null>(null)
  const lastLoRaStatusRef = useRef<boolean>(false)
  
  // Email notification state
  const emailSentRef = useRef<boolean>(false)
  const landingDetectedRef = useRef<boolean>(false)

  // Check for landing and send email notification
  const checkLandingAndSendEmail = useCallback((data: TelemetryData) => {
    // Check if landing detected (flight_state === 7)
    const isLanded = data.flightState === 7
    
    if (isLanded && !landingDetectedRef.current) {
      console.log('[EmailJS] 🎯 Landing detected! Flight state:', data.flightState)
      landingDetectedRef.current = true
      
      // Send email notification if not already sent
      if (!emailSentRef.current) {
        console.log('[EmailJS] Sending landing notification email...')
        
        // Convert data to email-compatible format
        const emailData = {
          timestamp: data.missionTime?.toString() || '',
          flight_state: data.flightState,
          altitude: data.altitude,
          pressure: data.pressure,
          temperature: data.temperature,
          voltage: data.batteryVoltage,
          battery_percentage: data.batteryPercentage,
          gnss_lat: data.latitude,
          gnss_long: data.longitude,
          gnss_alt: data.gpsAltitude,
          gnss_sats: data.gpsSatellites,
          gnss_speed: data.gpsSpeed,
          mode: data.mode,
          rssi: data.rssi,
          snr: data.snr,
        }
        
        sendLandingNotification(emailData)
          .then((success) => {
            if (success) {
              emailSentRef.current = true
              console.log('[EmailJS] ✅ Landing notification sent successfully')
            } else {
              console.warn('[EmailJS] ⚠️ Failed to send landing notification')
            }
          })
          .catch((error) => {
            console.error('[EmailJS] ❌ Error sending landing notification:', error)
          })
      }
    }
  }, [])

  // Initialize EmailJS on mount
  useEffect(() => {
    initializeEmailJS()
  }, [])

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
          
          // Check for landing and send email notification
          checkLandingAndSendEmail(event.data)
          
          // Update last LoRa data timestamp for any data received (CSV or non-CSV)
          lastLoRaDataRef.current = Date.now()
          
          // Check if LoRa is connected based on presence of RSSI/SNR OR recent data
          const hasRSSI = event.data.rssi !== undefined && event.data.rssi !== null
          const loraConnected = true // Any data means LoRa is connected
          
          setConnectionState((prev) => ({
            ...prev,
            lastUpdate: Date.now(),
            messageCount: prev.messageCount + 1,
            loraConnected,
          }))
          
          // Reset LoRa timeout
          if (loraTimeoutRef.current) {
            clearTimeout(loraTimeoutRef.current)
          }
          
          // Set new timeout for 30 seconds
          loraTimeoutRef.current = setTimeout(() => {
            console.log("[GCS] LoRa timeout - no data for 30 seconds")
            setConnectionState((prev) => {
              if (prev.loraConnected && lastLoRaStatusRef.current) {
                NotificationService.showLoRaStatus(false)
                lastLoRaStatusRef.current = false
              }
              return {
                ...prev,
                loraConnected: false,
              }
            })
          }, 30000) // 30 seconds
          
          // Notify if LoRa status changed to connected
          if (!lastLoRaStatusRef.current) {
            NotificationService.showLoRaStatus(true)
            lastLoRaStatusRef.current = true
          }
        }
      })

      // Handle log messages and status messages (keeps LoRa connection alive)
      wsClientRef.current.on("log_message", (event) => {
        if (event.data) {
          console.log("[GCS] Log/Status message received:", event.data)
          
          // Update last LoRa data timestamp for log/status messages too
          lastLoRaDataRef.current = Date.now()
          
          // Reset LoRa timeout for any received data
          if (loraTimeoutRef.current) {
            clearTimeout(loraTimeoutRef.current)
          }
          
          // Set new timeout for 30 seconds
          loraTimeoutRef.current = setTimeout(() => {
            console.log("[GCS] LoRa timeout - no data for 30 seconds")
            setConnectionState((prev) => {
              if (prev.loraConnected && lastLoRaStatusRef.current) {
                NotificationService.showLoRaStatus(false)
                lastLoRaStatusRef.current = false
              }
              return {
                ...prev,
                loraConnected: false,
              }
            })
          }, 30000) // 30 seconds
          
          // Keep LoRa connected since we received data and notify if status changed
          setConnectionState((prev) => {
            if (!lastLoRaStatusRef.current) {
              NotificationService.showLoRaStatus(true)
              lastLoRaStatusRef.current = true
            }
            return {
              ...prev,
              loraConnected: true,
              lastUpdate: Date.now(),
            }
          })
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
