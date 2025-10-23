"use client"

import { useState, useEffect } from "react"
import { Settings, X, RefreshCw, Wifi, WifiOff } from "lucide-react"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Card } from "@/components/ui/card"
import { Separator } from "@/components/ui/separator"
import NotificationService from "@/lib/notification-service"
import { addTerminalLog } from "@/components/terminal"

interface SettingsSidebarProps {
  isOpen: boolean
  onClose: () => void
  isConnected?: boolean
  onConnect?: (port: string, baudRate: number) => void
  onDisconnect?: () => void
}

interface ComPort {
  port: string
  description: string
}

export function SettingsSidebar({ isOpen, onClose, isConnected = false, onConnect, onDisconnect }: SettingsSidebarProps) {
  const [availablePorts, setAvailablePorts] = useState<ComPort[]>([])
  const [selectedPort, setSelectedPort] = useState<string>("COM3")
  const [baudRate, setBaudRate] = useState<string>("115200")
  const [csvPath, setCsvPath] = useState<string>("./telemetry_data.csv")
  const [isLoadingPorts, setIsLoadingPorts] = useState(false)
  const [isConnecting, setIsConnecting] = useState(false)

  const commonBaudRates = ["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"]

  const fetchAvailablePorts = async () => {
    setIsLoadingPorts(true)
    try {
      const response = await fetch('http://localhost:5000/api/ports')
      const data = await response.json()
      if (data.success && data.ports) {
        setAvailablePorts(data.ports)
        if (data.ports.length > 0 && !selectedPort) {
          setSelectedPort(data.ports[0].port)
        }
      }
    } catch (error) {
      console.error("Failed to fetch ports:", error)
    } finally {
      setIsLoadingPorts(false)
    }
  }

  useEffect(() => {
    if (isOpen) {
      fetchAvailablePorts()
    }
  }, [isOpen])

  const handleSaveSettings = () => {
    // Save settings to localStorage
    localStorage.setItem('gcs_settings', JSON.stringify({
      port: selectedPort,
      baudRate: parseInt(baudRate),
      csvPath
    }))
    onClose()
  }

  useEffect(() => {
    // Load settings from localStorage
    const saved = localStorage.getItem('gcs_settings')
    if (saved) {
      try {
        const settings = JSON.parse(saved)
        setSelectedPort(settings.port || "COM3")
        setBaudRate(settings.baudRate?.toString() || "115200")
        setCsvPath(settings.csvPath || "./telemetry_data.csv")
      } catch (e) {
        console.error("Failed to load settings:", e)
      }
    }
  }, [])

  const handleToggleConnection = async () => {
    setIsConnecting(true)
    
    if (isConnected) {
      // Disconnect - use the main WebSocket connection if available
      try {
        // Try to use existing connection first, fall back to new connection
        const { io } = await import('socket.io-client')
        const socket = io('http://localhost:5000', {
          timeout: 5000,
          reconnection: false
        })
        
        let disconnected = false
        
        // Set a timeout to disconnect locally if server doesn't respond
        const disconnectTimeout = setTimeout(() => {
          if (!disconnected) {
            console.log('[GCS] Disconnect timeout, forcing local disconnect')
            disconnected = true
            onDisconnect?.()
            socket.disconnect()
          }
        }, 3000) // 3 second timeout
        
        socket.on('connect', () => {
          console.log('[GCS] Connected for disconnect operation')
          socket.emit('disconnect_serial')
        })
        
        socket.emit('disconnect_serial')
        
        socket.on('serial_status', (data: any) => {
          console.log('[GCS] Serial status update:', data)
          if (!disconnected) {
            clearTimeout(disconnectTimeout)
            disconnected = true
            if (data.status === 'disconnected') {
              onDisconnect?.()
            }
            socket.disconnect()
          }
        })
        
        // Handle connection errors gracefully during disconnect
        socket.on('connect_error', (error: any) => {
          if (!disconnected) {
            console.log('[GCS] Socket connection error during disconnect (expected):', error)
            clearTimeout(disconnectTimeout)
            disconnected = true
            // Still call onDisconnect since we wanted to disconnect anyway
            onDisconnect?.()
            socket.disconnect()
          }
        })
        
      } catch (error) {
        console.log('[GCS] Failed to create disconnect socket:', error)
        // Even if disconnect fails, we should update the UI state
        onDisconnect?.()
      }
    } else {
      // Connect
      if (selectedPort) {
        try {
          const { io } = await import('socket.io-client')
          const socket = io('http://localhost:5000', {
            timeout: 10000,
            reconnection: false
          })
          
          socket.on('connect', () => {
            console.log('[GCS] Socket.IO connected, sending serial connection request...')
            socket.emit('connect_serial', { port: selectedPort, baudrate: parseInt(baudRate) })
          })
          
          socket.on('serial_status', (data: any) => {
            console.log('[GCS] Serial connection status:', data)
            if (data.status === 'connected') {
              console.log(`[GCS] Successfully connected to ${data.port} at ${data.baudrate} baud`)
              NotificationService.showConnectionSuccess(`${data.port} (${data.baudrate} baud)`)
              addTerminalLog({
                source: 'System',
                level: 'INFO',
                message: `Connected to ${data.port} at ${data.baudrate} baud`
              })
              onConnect?.(selectedPort, parseInt(baudRate))
              socket.disconnect()
            } else if (data.status === 'error') {
              console.error('[GCS] Serial connection error:', data.message || 'Connection failed')
              const errorMsg = data.message || 'Unknown error'
              NotificationService.showConnectionError(errorMsg)
              addTerminalLog({
                source: 'System',
                level: 'ERROR',
                message: `Serial connection failed: ${errorMsg}`
              })
              onDisconnect?.()
              socket.disconnect()
            }
          })
          
          socket.on('connect_error', (error: any) => {
            console.error('[GCS] Socket.IO connection error:', error)
            NotificationService.showConnectionError('Failed to connect to backend service')
            addTerminalLog({
              source: 'System',
              level: 'ERROR',
              message: `Backend connection error: ${error.message || error}`
            })
            onDisconnect?.()
          })
          
          socket.on('error', (error: any) => {
            console.error('[GCS] Socket.IO error during connection:', error)
            NotificationService.showConnectionError(error.message || 'Socket error occurred')
            addTerminalLog({
              source: 'System',
              level: 'ERROR',
              message: `Socket error during connection: ${error.message || error}`
            })
            onDisconnect?.()
            socket.disconnect()
          })
        } catch (error) {
          console.error('[GCS] Failed to connect:', error)
          NotificationService.showConnectionError(String(error))
          addTerminalLog({
            source: 'System',
            level: 'ERROR',
            message: `Connection attempt failed: ${error}`
          })
        }
      } else {
        NotificationService.showConnectionError('Please select a COM port first')
      }
    }
    
    setTimeout(() => setIsConnecting(false), 3000) // Increased timeout
  }

  return (
    <>
      {/* Overlay */}
      {isOpen && (
        <div 
          className="fixed inset-0 bg-black/50 z-40 transition-opacity"
          onClick={onClose}
        />
      )}

      {/* Sidebar */}
      <div
        className={`fixed top-0 right-0 h-screen w-96 bg-background border-l border-border z-50 transform transition-transform duration-300 ease-in-out ${
          isOpen ? 'translate-x-0' : 'translate-x-full'
        }`}
      >
        <div className="flex flex-col h-full">
          {/* Header */}
          <div className="flex items-center justify-between p-4 border-b border-border">
            <div className="flex items-center gap-2">
              <Settings className="h-5 w-5" />
              <h2 className="text-lg font-semibold">Settings</h2>
            </div>
            <Button variant="ghost" size="sm" onClick={onClose}>
              <X className="h-4 w-4" />
            </Button>
          </div>

          {/* Content */}
          <div className="flex-1 overflow-y-auto p-4 space-y-6">
            {/* ESP32 Connection Settings */}
            <Card className="p-4">
              <h3 className="text-sm font-semibold mb-4">ESP32 Connection</h3>
              
              <div className="space-y-4">
                {/* COM Port */}
                <div className="space-y-2">
                  <Label htmlFor="com-port">COM Port</Label>
                  <div className="flex gap-2">
                    <Select value={selectedPort} onValueChange={setSelectedPort}>
                      <SelectTrigger id="com-port" className="overflow-hidden">
                        <SelectValue placeholder="Select port" className="truncate block overflow-hidden text-ellipsis" />
                      </SelectTrigger>
                      <SelectContent>
                        {availablePorts.length > 0 ? (
                          availablePorts.map((port) => (
                            <SelectItem key={port.port} value={port.port}>
                              <div className="flex flex-col items-start max-w-full overflow-hidden">
                                <span className="font-semibold truncate max-w-full">{port.port}</span>
                                <span className="text-xs text-muted-foreground truncate max-w-[250px]">
                                  {port.description}
                                </span>
                              </div>
                            </SelectItem>
                          ))
                        ) : (
                          <SelectItem value="none" disabled>
                            No ports found
                          </SelectItem>
                        )}
                      </SelectContent>
                    </Select>
                    <Button
                      variant="outline"
                      size="icon"
                      onClick={fetchAvailablePorts}
                      disabled={isLoadingPorts}
                    >
                      <RefreshCw className={`h-4 w-4 ${isLoadingPorts ? 'animate-spin' : ''}`} />
                    </Button>
                  </div>
                </div>

                {/* Baud Rate */}
                <div className="space-y-2">
                  <Label htmlFor="baud-rate">Baud Rate</Label>
                  <Select value={baudRate} onValueChange={setBaudRate}>
                    <SelectTrigger id="baud-rate">
                      <SelectValue placeholder="Select baud rate" />
                    </SelectTrigger>
                    <SelectContent>
                      {commonBaudRates.map((rate) => (
                        <SelectItem key={rate} value={rate}>
                          {rate} bps
                        </SelectItem>
                      ))}
                    </SelectContent>
                  </Select>
                </div>
              </div>
            </Card>

            {/* Connection Control */}
            <Card className="p-4">
              <h3 className="text-sm font-semibold mb-4">Connection</h3>
              <div className="space-y-3">
                <div className="flex items-center justify-between text-xs">
                  <span className="text-muted-foreground">Status:</span>
                  <span className={isConnected ? "text-mission-active font-semibold" : "text-muted-foreground"}>
                    {isConnected ? "Connected" : "Disconnected"}
                  </span>
                </div>
                <Button
                  onClick={handleToggleConnection}
                  disabled={isConnecting}
                  variant={isConnected ? "destructive" : "default"}
                  className="w-full"
                >
                  {isConnected ? (
                    <>
                      <WifiOff className="h-4 w-4 mr-2" />
                      Disconnect
                    </>
                  ) : (
                    <>
                      <Wifi className="h-4 w-4 mr-2" />
                      Connect
                    </>
                  )}
                </Button>
              </div>
            </Card>

            {/* CSV Settings */}
            <Card className="p-4">
              <h3 className="text-sm font-semibold mb-4">Data Recording</h3>
              
              <div className="space-y-4">
                {/* CSV File Path */}
                <div className="space-y-2">
                  <Label htmlFor="csv-path">CSV File Location</Label>
                  <Input
                    id="csv-path"
                    type="text"
                    value={csvPath}
                    onChange={(e) => setCsvPath(e.target.value)}
                    placeholder="./telemetry_data.csv"
                  />
                  <p className="text-xs text-muted-foreground">
                    Path where telemetry data will be saved
                  </p>
                </div>
              </div>
            </Card>

            {/* Additional Settings */}
            <Card className="p-4">
              <h3 className="text-sm font-semibold mb-4">Advanced</h3>
              
              <div className="space-y-4">
                <div className="space-y-2">
                  <Label htmlFor="websocket-url">WebSocket URL</Label>
                  <Input
                    id="websocket-url"
                    type="text"
                    defaultValue="ws://localhost:5000"
                    placeholder="ws://localhost:5000"
                  />
                </div>

                <div className="space-y-2">
                  <Label htmlFor="data-rate">Data Rate (Hz)</Label>
                  <Input
                    id="data-rate"
                    type="number"
                    defaultValue="10"
                    placeholder="10"
                  />
                </div>
              </div>
            </Card>
          </div>

          {/* Footer */}
          <div className="p-4 border-t border-border">
            <div className="flex gap-2">
              <Button variant="outline" onClick={onClose} className="flex-1">
                Cancel
              </Button>
              <Button onClick={handleSaveSettings} className="flex-1">
                Save Settings
              </Button>
            </div>
          </div>
        </div>
      </div>
    </>
  )
}
