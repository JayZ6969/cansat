import type { TelemetryData } from "./telemetry-types"
import { io, Socket } from "socket.io-client"
import { mapPythonToTelemetry } from "./telemetry-mapper"
import NotificationService from "./notification-service"
import { addTerminalLog } from "@/components/terminal"

export type WebSocketEventType = "connected" | "disconnected" | "data" | "error" | "serial_status" | "log_message"

export interface WebSocketEvent {
  type: WebSocketEventType
  data?: TelemetryData | string
}

export class WebSocketClient {
  private socket: Socket | null = null
  private url: string
  private reconnectAttempts = 0
  private maxReconnectAttempts = 5
  private reconnectDelay = 3000
  private listeners: Map<WebSocketEventType, Set<(event: WebSocketEvent) => void>> = new Map()
  private isManuallyDisconnected = false

  constructor(url: string) {
    // Convert ws:// to http:// for Socket.IO
    this.url = url.replace('ws://', 'http://').replace('wss://', 'https://')
  }

  connect(): Promise<void> {
    return new Promise((resolve, reject) => {
      try {
        this.isManuallyDisconnected = false
        
        // Create Socket.IO connection
        this.socket = io(this.url, {
          transports: ['websocket', 'polling'],
          reconnection: false, // We'll handle reconnection manually
        })

        this.socket.on("connect", () => {
          console.log("[GCS] Socket.IO connected to backend")
          this.reconnectAttempts = 0
          this.notifyListeners("connected", { type: "connected" })
          addTerminalLog({
            source: 'System',
            level: 'INFO',
            message: 'WebSocket connected to backend'
          })
          resolve()
        })

        // Listen for telemetry data from backend
        this.socket.on("telemetry_data", (rawData: any) => {
          const data = mapPythonToTelemetry(rawData)
          console.log("[GCS] ✅ Received telemetry:", data.altitude, data.batteryPercentage, data.rssi)
          this.notifyListeners("data", { type: "data", data })
        })

        // Listen for serial status changes from backend
        this.socket.on("serial_status", (statusData: any) => {
          console.log("[GCS] Serial status update:", statusData)
          this.notifyListeners("serial_status", { type: "serial_status", data: statusData })
          
          // Add notification and terminal log for status changes
          if (statusData.connected) {
            NotificationService.showConnectionSuccess(statusData.port || 'Serial Port')
            addTerminalLog({
              source: 'System',
              level: 'INFO',
              message: `Connected to ${statusData.port || 'Serial Port'}`
            })
          } else {
            NotificationService.showDisconnected()
            addTerminalLog({
              source: 'System',
              level: 'INFO',
              message: 'Serial port disconnected'
            })
          }
        })

        // Listen for log messages from backend
        this.socket.on("log_message", (logData: any) => {
          console.log("[GCS] Log message:", logData.log_message)
          this.notifyListeners("log_message", { type: "log_message", data: logData })
          
          // Parse log message and add to terminal
          const message = logData.log_message || ''
          let source: 'GCS' | 'CanSat' | 'System' = 'System'
          let level: 'INFO' | 'WARNING' | 'ERROR' | 'STATUS' | 'DEBUG' = 'INFO'
          
          // Determine source and level from message content
          if (message.includes('[STATUS]')) {
            level = 'STATUS'
            source = 'CanSat'
          } else if (message.includes('[ERROR]')) {
            level = 'ERROR'
            source = message.includes('GCS') ? 'GCS' : 'CanSat'
          } else if (message.includes('[WARNING]')) {
            level = 'WARNING'
            source = message.includes('GCS') ? 'GCS' : 'CanSat'
          } else if (message.includes('[INFO]')) {
            level = 'INFO'
            source = message.includes('GCS') ? 'GCS' : 'CanSat'
          } else if (message.includes('CSV:') || message.includes('RX #')) {
            level = 'DEBUG'
            source = 'CanSat'
          }
          
          // Add to terminal
          addTerminalLog({
            source,
            level,
            message: message,
            raw: message
          })
          
          // Show notification for important messages
          if (level === 'ERROR' || level === 'WARNING' || level === 'STATUS') {
            const notificationSource = source === 'System' ? 'GCS' : source
            NotificationService.showLogMessage(notificationSource, message, level)
          }
        })

        this.socket.on("connect_error", (error) => {
          console.error("[GCS] Socket.IO connection error:", error)
          this.notifyListeners("error", { type: "error", data: "Connection error" })
          NotificationService.showConnectionError("Failed to connect to backend service")
          addTerminalLog({
            source: 'System',
            level: 'ERROR',
            message: `Backend connection failed: ${error.message || error}`
          })
          reject(error)
        })

        this.socket.on("disconnect", (reason) => {
          console.log("[GCS] Socket.IO disconnected:", reason)
          this.notifyListeners("disconnected", { type: "disconnected" })
          addTerminalLog({
            source: 'System',
            level: 'WARNING',
            message: `Backend connection lost: ${reason}`
          })
          if (!this.isManuallyDisconnected && reason === "io server disconnect") {
            // Server initiated disconnect, attempt reconnect
            this.attemptReconnect()
          }
        })

        this.socket.on("error", (error) => {
          // Only log errors if we're not in the middle of disconnecting
          if (!this.isManuallyDisconnected) {
            console.error("[GCS] Socket.IO error:", error)
            this.notifyListeners("error", { type: "error", data: error })
          } else {
            console.log("[GCS] Socket.IO error during disconnect (expected):", error)
          }
        })

      } catch (error) {
        reject(error)
      }
    })
  }

  disconnect(): void {
    this.isManuallyDisconnected = true
    if (this.socket) {
      this.socket.disconnect()
      this.socket = null
    }
  }

  private attemptReconnect(): void {
    if (this.reconnectAttempts < this.maxReconnectAttempts) {
      this.reconnectAttempts++
      console.log(`[GCS] Attempting to reconnect (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`)
      setTimeout(() => {
        this.connect().catch((error) => {
          console.error("[GCS] Reconnection failed:", error)
        })
      }, this.reconnectDelay)
    } else {
      console.error("[GCS] Max reconnection attempts reached")
      this.notifyListeners("error", { type: "error", data: "Failed to reconnect after multiple attempts" })
    }
  }

  on(type: WebSocketEventType, callback: (event: WebSocketEvent) => void): void {
    if (!this.listeners.has(type)) {
      this.listeners.set(type, new Set())
    }
    this.listeners.get(type)!.add(callback)
  }

  off(type: WebSocketEventType, callback: (event: WebSocketEvent) => void): void {
    this.listeners.get(type)?.delete(callback)
  }

  private notifyListeners(type: WebSocketEventType, event: WebSocketEvent): void {
    this.listeners.get(type)?.forEach((callback) => callback(event))
  }

  isConnected(): boolean {
    return this.socket?.connected || false
  }

  // Helper method to send data to backend
  send(eventName: string, data?: any): void {
    if (this.socket?.connected) {
      this.socket.emit(eventName, data)
    }
  }
}
