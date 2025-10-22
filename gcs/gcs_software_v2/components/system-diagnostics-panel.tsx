"use client"

import { AlertCircle, CheckCircle2, AlertTriangle } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"

interface SystemDiagnosticsPanelProps {
  telemetry?: any
}

// Error code mapping
const ERROR_CODE_MAP: { [key: string]: string } = {
  '1': 'MPU6050 (Accel/Gyro)',
  '2': 'BMP280 (Pressure)',
  '3': 'SD Card',
  '5': 'GNSS/GPS',
  '6': 'PID Control',
  '7': 'Camera',
  '8': 'Serial (UART)',
  '9': 'LoRa',
  '10': 'BMP390 (Pressure)',
}

export function SystemDiagnosticsPanel({ telemetry }: SystemDiagnosticsPanelProps) {
  // Parse error codes from telemetry
  const errorCodeString = telemetry?.errorCode || telemetry?.ERROR_CODE || '0'
  const hasErrors = errorCodeString !== '0' && errorCodeString !== ''
  
  // Parse individual error codes
  const parseErrorCodes = (errorStr: string): string[] => {
    if (!errorStr || errorStr === '0') return []
    
    const codes: string[] = []
    let remaining = errorStr
    
    // Check for two-digit codes first (10)
    if (remaining.includes('10')) {
      codes.push('10')
      remaining = remaining.replace('10', '')
    }
    
    // Then parse single-digit codes
    for (const char of remaining) {
      if (char !== '0') {
        codes.push(char)
      }
    }
    
    return codes
  }
  
  const errorCodes = parseErrorCodes(errorCodeString)
  
  // Check for sensor-specific errors
  const hasSensorError = errorCodes.some(code => ['1', '2', '10'].includes(code))
  
  // Build sensor status message
  const getSensorStatus = () => {
    const sensorErrors = errorCodes.filter(code => ['1', '2', '10'].includes(code))
    if (sensorErrors.length === 0) return { status: 'ok', value: 'All nominal' }
    
    const errorNames = sensorErrors.map(code => ERROR_CODE_MAP[code] || code).join(', ')
    return { status: 'critical', value: errorNames }
  }
  
  const sensorStatus = getSensorStatus()
  
  const diagnostics = [
    {
      name: "Sensors",
      status: sensorStatus.status,
      value: sensorStatus.value,
    },
  ]

  return (
    <Card className="p-4">
      <h3 className="text-sm font-semibold text-foreground mb-3">System Diagnostics</h3>
      <div className="space-y-3">
        {/* Diagnostics */}
        <div className="space-y-2">
          {diagnostics.map((diag) => (
            <div key={diag.name} className="flex items-center justify-between text-xs">
              <div className="flex items-center gap-2">
                {diag.status === "ok" ? (
                  <CheckCircle2 className="h-3 w-3 text-mission-active" />
                ) : diag.status === "warning" ? (
                  <AlertCircle className="h-3 w-3 text-mission-warning" />
                ) : (
                  <AlertCircle className="h-3 w-3 text-destructive" />
                )}
                <span className="text-muted-foreground">{diag.name}</span>
              </div>
              <span className={`font-mono font-semibold text-xs ${diag.status === 'ok' ? 'text-foreground' : 'text-destructive'}`}>
                {diag.value}
              </span>
            </div>
          ))}
        </div>

        {/* Error Codes Section */}
        <div className="pt-2 border-t border-border">
          <div className="flex items-center justify-between mb-2">
            <span className="text-xs font-semibold text-muted-foreground">Error Codes</span>
            {hasErrors && (
              <Badge variant="destructive" className="h-5 text-[10px]">
                {errorCodes.length}
              </Badge>
            )}
          </div>
          
          {hasErrors ? (
            <div className="space-y-1 max-h-32 overflow-y-auto">
              {errorCodes.map((code: string, index: number) => (
                <div key={index} className="flex items-start gap-2 text-xs p-2 rounded bg-destructive/10 border border-destructive/20">
                  <AlertTriangle className="h-3 w-3 text-destructive mt-0.5 shrink-0" />
                  <div className="flex-1 min-w-0">
                    <div className="font-mono font-semibold text-destructive">
                      Error {code}
                    </div>
                    <div className="text-[10px] text-muted-foreground">
                      {ERROR_CODE_MAP[code] || 'Unknown error'}
                    </div>
                  </div>
                </div>
              ))}
            </div>
          ) : (
            <div className="flex items-center gap-2 text-xs text-muted-foreground">
              <CheckCircle2 className="h-3 w-3 text-mission-active" />
              <span>No errors detected</span>
            </div>
          )}
        </div>
      </div>
    </Card>
  )
}
