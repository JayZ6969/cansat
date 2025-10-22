"use client"

import { useState, useEffect } from "react"
import { Wifi, WifiOff, RefreshCw, Plug, PlugZap } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { useToast } from "@/hooks/use-toast"
import { getAvailablePorts, connectToPort, disconnectFromPort, type PortInfo } from "@/lib/api-client"

interface ConnectionPanelProps {
  isConnected: boolean
  onConnect?: () => void
  onDisconnect?: () => void
}

export function ConnectionPanel({ isConnected, onConnect, onDisconnect }: ConnectionPanelProps) {
  const [isConnecting, setIsConnecting] = useState(false)
  const [ports, setPorts] = useState<PortInfo[]>([])
  const [selectedPort, setSelectedPort] = useState<string>("")
  const [selectedBaudrate, setSelectedBaudrate] = useState<string>("9600")
  const [isScanning, setIsScanning] = useState(false)
  const { toast } = useToast()

  const baudrates = ["9600", "19200", "38400", "57600", "115200"]

  // Fetch ports on component mount
  useEffect(() => {
    fetchPorts()
  }, [])

  const fetchPorts = async () => {
    setIsScanning(true)
    try {
      const availablePorts = await getAvailablePorts()
      setPorts(availablePorts)
      
      if (availablePorts.length > 0 && !selectedPort) {
        setSelectedPort(availablePorts[0].port)
      }
      
      toast({
        title: "Ports scanned",
        description: `Found ${availablePorts.length} COM port(s)`,
      })
    } catch (error) {
      toast({
        title: "Error scanning ports",
        description: error instanceof Error ? error.message : "Failed to scan ports",
        variant: "destructive",
      })
    } finally {
      setIsScanning(false)
    }
  }

  const handleToggleConnection = async () => {
    setIsConnecting(true)
    
    try {
      if (isConnected) {
        await disconnectFromPort()
        toast({
          title: "Disconnected",
          description: "Disconnected from COM port",
        })
        onDisconnect?.()
      } else {
        if (!selectedPort) {
          toast({
            title: "No port selected",
            description: "Please select a COM port first",
            variant: "destructive",
          })
          setIsConnecting(false)
          return
        }
        
        await connectToPort(selectedPort, parseInt(selectedBaudrate))
        toast({
          title: "Connected",
          description: `Connected to ${selectedPort} at ${selectedBaudrate} baud`,
        })
        onConnect?.()
      }
    } catch (error) {
      toast({
        title: isConnected ? "Disconnect failed" : "Connection failed",
        description: error instanceof Error ? error.message : "Operation failed",
        variant: "destructive",
      })
    } finally {
      setIsConnecting(false)
    }
  }

  return (
    <Card className="p-4">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-sm font-semibold text-foreground">Connection</h3>
        <div className={`h-2 w-2 rounded-full ${isConnected ? "bg-mission-active" : "bg-destructive"}`} />
      </div>

      <div className="space-y-3">
        {/* COM Port Selection */}
        <div>
          <label className="text-xs text-muted-foreground mb-1 block">COM Port</label>
          <div className="flex gap-2">
            <Select
              value={selectedPort}
              onValueChange={setSelectedPort}
              disabled={isConnected || isConnecting}
            >
              <SelectTrigger className="flex-1">
                <SelectValue placeholder="Select port..." />
              </SelectTrigger>
              <SelectContent>
                {ports.length === 0 ? (
                  <SelectItem value="none" disabled>
                    No ports found
                  </SelectItem>
                ) : (
                  ports.map((port) => (
                    <SelectItem key={port.port} value={port.port}>
                      {port.port} - {port.description}
                    </SelectItem>
                  ))
                )}
              </SelectContent>
            </Select>
            <Button
              variant="outline"
              size="icon"
              onClick={fetchPorts}
              disabled={isConnected || isConnecting || isScanning}
              title="Refresh ports"
            >
              <RefreshCw className={`h-4 w-4 ${isScanning ? "animate-spin" : ""}`} />
            </Button>
          </div>
        </div>

        {/* Baud Rate Selection */}
        <div>
          <label className="text-xs text-muted-foreground mb-1 block">Baud Rate</label>
          <Select
            value={selectedBaudrate}
            onValueChange={setSelectedBaudrate}
            disabled={isConnected || isConnecting}
          >
            <SelectTrigger>
              <SelectValue />
            </SelectTrigger>
            <SelectContent>
              {baudrates.map((rate) => (
                <SelectItem key={rate} value={rate}>
                  {rate}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>

        <div className="h-px bg-border" />

        {/* Connection Status */}
        <div className="text-xs">
          <div className="flex justify-between mb-1">
            <span className="text-muted-foreground">Status:</span>
            <span className={`font-semibold ${isConnected ? "text-mission-active" : "text-destructive"}`}>
              {isConnected ? "Connected" : "Disconnected"}
            </span>
          </div>
          {isConnected && selectedPort && (
            <div className="flex justify-between mb-1">
              <span className="text-muted-foreground">Port:</span>
              <span className="font-mono font-semibold text-foreground">{selectedPort}</span>
            </div>
          )}
        </div>

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
                  <PlugZap className="h-4 w-4 mr-2" />
                  Disconnect
                </>
              ) : (
                <>
                  <Plug className="h-4 w-4 mr-2" />
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
