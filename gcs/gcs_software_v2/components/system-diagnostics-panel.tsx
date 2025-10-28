"use client"

import { AlertCircle, CheckCircle2, AlertTriangle } from "lucide-react"
import { Card } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"

interface SystemDiagnosticsPanelProps {
  telemetry?: any
}

// Error code mapping
const ERROR_CODE_MAP: { [key: string]: string } = {
  '1': 'MPU6050 Failure',
  '2': 'BMP280 Failure',
  '3': 'SD Card Failure',
  '4': 'Unknown Failure',
  '5': 'GNSS Failure',
  '6': 'PID Failure',
  '7': 'Camera Failure',
  '8': 'LoRa Failure',
  '9': 'BMP390 Failure',
}

export function SystemDiagnosticsPanel({ telemetry }: SystemDiagnosticsPanelProps) {
  // Parse error codes from telemetry - check multiple possible field names
  const errorCodeString = telemetry?.errorCode || telemetry?.ERROR_CODE || '0'
  
  // Also check for the errorCodes array field from the new parser
  const errorCodesArray = telemetry?.errorCodes || []
  
  // Convert errorCodes array to displayable format
  const getErrorCodesFromArray = (): Array<{ code: string; message: string }> => {
    if (!errorCodesArray || errorCodesArray.length === 0) return []
    
    return errorCodesArray.map((item: any) => {
      if (typeof item === 'string') {
        return { code: item, message: ERROR_CODE_MAP[item] || 'Unknown error' }
      } else if (typeof item === 'object' && item.code) {
        return { code: item.code, message: item.message || ERROR_CODE_MAP[item.code] || 'Unknown error' }
      }
      return { code: '0', message: 'Invalid error format' }
    }).filter((err: { code: string; message: string }) => err.code !== '0')
  }
  
  const hasErrors = errorCodeString !== '0' && errorCodeString !== '' || errorCodesArray.length > 0
  
  // Parse individual error codes from string format
  const parseErrorCodes = (errorStr: string): string[] => {
    if (!errorStr || errorStr === '0') return []
    
    const codes: string[] = []
    const seen = new Set<string>()
    
    // Parse each digit as a separate error code
    // Example: "59" means errors 5 and 9, "349" means errors 3, 4, and 9
    for (const char of errorStr) {
      if (char !== '0' && char >= '1' && char <= '9' && !seen.has(char)) {
        codes.push(char)
        seen.add(char)
      }
    }
    
    return codes
  }
  
  // Get error codes from string format
  const errorCodesFromString = parseErrorCodes(errorCodeString)
  
  // Get error codes from array format (new parser)
  const errorCodesFromArray = getErrorCodesFromArray()
  
  // Combine both sources (prioritize array format if available)
  const errorCodes = errorCodesFromArray.length > 0 
    ? errorCodesFromArray 
    : errorCodesFromString.map(code => ({ code, message: ERROR_CODE_MAP[code] || 'Unknown error' }))
  
  // Check for sensor-specific errors
  const hasSensorError = errorCodes.some(err => ['1', '2', '9'].includes(err.code))
  
  // Build sensor status message
  const getSensorStatus = () => {
    const sensorErrors = errorCodes.filter(err => ['1', '2', '9'].includes(err.code))
    if (sensorErrors.length === 0) return { status: 'ok', value: 'All nominal' }
    
    const errorNames = sensorErrors.map(err => err.message).join(', ')
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
              {errorCodes.map((err: { code: string; message: string }, index: number) => (
                <div key={index} className="flex items-start gap-2 text-xs p-2 rounded bg-destructive/10 border border-destructive/20">
                  <AlertTriangle className="h-3 w-3 text-destructive mt-0.5 shrink-0" />
                  <div className="flex-1 min-w-0">
                    <div className="font-mono font-semibold text-destructive">
                      Error {err.code}
                    </div>
                    <div className="text-[10px] text-muted-foreground">
                      {err.message}
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
