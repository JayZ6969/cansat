"use client"

import { useState, useEffect } from "react"
import { Wifi, WifiOff, RefreshCw } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Label } from "@/components/ui/label"
import { Input } from "@/components/ui/input"

interface ConnectionPanelProps {
  isConnected: boolean
  onConnect?: (port: string, baudRate: number) => void
  onDisconnect?: () => void
}

interface ComPort {
  port: string
  description: string
}

export function ConnectionPanel({ isConnected, onConnect, onDisconnect }: ConnectionPanelProps) {
  const [isConnecting, setIsConnecting] = useState(false)
  const [availablePorts, setAvailablePorts] = useState<ComPort[]>([])
  const [selectedPort, setSelectedPort] = useState<string>("")
  const [baudRate, setBaudRate] = useState<number>(115200)
  const [isLoadingPorts, setIsLoadingPorts] = useState(false)

  const commonBaudRates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]

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
      // Fallback: Try Socket.IO event as backup
      try {
        const { io } = await import('socket.io-client')
        const socket = io('http://localhost:5000')
        socket.emit('get_ports')
        socket.on('ports_list', (data: { ports: ComPort[] }) => {
          setAvailablePorts(data.ports || [])
          if (data.ports && data.ports.length > 0 && !selectedPort) {
            setSelectedPort(data.ports[0].port)
          }
          socket.disconnect()
        })
      } catch (socketError) {
        console.error("Failed to fetch ports via Socket.IO:", socketError)
      }
    } finally {
      setIsLoadingPorts(false)
    }
  }

  useEffect(() => {
    fetchAvailablePorts()
  }, [])

  const handleToggleConnection = async () => {
    setIsConnecting(true)
    
    if (isConnected) {
      // Disconnect
      try {
        const { io } = await import('socket.io-client')
        const socket = io('http://localhost:5000')
        socket.emit('disconnect_serial')
        socket.on('serial_status', (data: any) => {
          console.log('[GCS] Serial disconnected:', data)
          socket.disconnect()
        })
        onDisconnect?.()
      } catch (error) {
        console.error('[GCS] Failed to disconnect:', error)
      }
    } else {
      // Connect
      if (selectedPort) {
        try {
          const { io } = await import('socket.io-client')
          const socket = io('http://localhost:5000')
          
          socket.on('connect', () => {
            console.log('[GCS] Socket.IO connected, sending serial connection request...')
            socket.emit('connect_serial', { port: selectedPort, baudrate: baudRate })
          })
          
          socket.on('serial_status', (data: any) => {
            console.log('[GCS] Serial connection status:', data)
            if (data.status === 'connected') {
              console.log(`[GCS] Successfully connected to ${data.port} at ${data.baudrate} baud`)
              onConnect?.(selectedPort, baudRate)
            } else if (data.status === 'error') {
              console.error('[GCS] Serial connection error:', data.message)
            }
            // Keep socket open for telemetry
          })
          
          socket.on('error', (error: any) => {
            console.error('[GCS] Socket.IO error:', error)
          })
        } catch (error) {
          console.error('[GCS] Failed to connect:', error)
        }
      }
    }
    
    setTimeout(() => setIsConnecting(false), 500)
  }

  return (
    <Card className="p-4">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-sm font-semibold text-foreground">Connection</h3>
        <div className={`h-2 w-2 rounded-full ${isConnected ? "bg-mission-active" : "bg-destructive"}`} />
      </div>

      <div className="space-y-3">
        {/* Connection Status */}
        <div className="text-xs">
          <div className="flex justify-between mb-1">
            <span className="text-muted-foreground">Status:</span>
            <span className={`font-semibold ${isConnected ? "text-mission-active" : "text-destructive"}`}>
              {isConnected ? "Connected" : "Disconnected"}
            </span>
          </div>
          <div className="flex justify-between mb-1">
            <span className="text-muted-foreground">Type:</span>
            <span className="font-semibold text-foreground">Serial</span>
          </div>
        </div>

        <div className="h-px bg-border" />

        {/* COM Port Selection */}
        {!isConnected && (
          <>
            <div className="space-y-2">
              <Label htmlFor="com-port" className="text-xs">COM Port</Label>
              <div className="flex gap-2">
                <Select value={selectedPort} onValueChange={setSelectedPort} disabled={isConnected}>
                  <SelectTrigger id="com-port" className="text-xs">
                    <SelectValue placeholder="Select port..." className="truncate" />
                  </SelectTrigger>
                  <SelectContent>
                    {availablePorts.map((port) => (
                      <SelectItem key={port.port} value={port.port} className="text-xs">
                        <div className="flex flex-col items-start max-w-full">
                          <span className="font-semibold">{port.port}</span>
                          <span className="text-[10px] text-muted-foreground truncate max-w-full">
                            {port.description}
                          </span>
                        </div>
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
                <Button
                  size="sm"
                  variant="outline"
                  onClick={fetchAvailablePorts}
                  disabled={isLoadingPorts || isConnected}
                  className="px-2"
                >
                  <RefreshCw className={`h-3 w-3 ${isLoadingPorts ? "animate-spin" : ""}`} />
                </Button>
              </div>
            </div>

            <div className="space-y-2">
              <Label htmlFor="baud-rate" className="text-xs">Baud Rate</Label>
              <Select 
                value={baudRate.toString()} 
                onValueChange={(value) => setBaudRate(Number(value))}
                disabled={isConnected}
              >
                <SelectTrigger id="baud-rate" className="text-xs">
                  <SelectValue placeholder="Select baud rate..." />
                </SelectTrigger>
                <SelectContent>
                  {commonBaudRates.map((rate) => (
                    <SelectItem key={rate} value={rate.toString()} className="text-xs">
                      {rate} bps
                    </SelectItem>
                  ))}
                </SelectContent>
              </Select>
            </div>

            <div className="h-px bg-border" />
          </>
        )}

        {/* Connect/Disconnect Button */}
        <Button
          onClick={handleToggleConnection}
          disabled={isConnecting || (!isConnected && !selectedPort)}
          className="w-full"
          variant={isConnected ? "destructive" : "default"}
        >
          {isConnecting ? (
            <>
              <RefreshCw className="h-4 w-4 mr-2 animate-spin" />
              {isConnected ? "Disconnecting..." : "Connecting..."}
            </>
          ) : (
            <>
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
            </>
          )}
        </Button>
      </div>
    </Card>
  )
}
