# Error Code Parsing Fix Summary

## Problem
The dashboard was not parsing or displaying error codes 8 and 58 (GPS Not Locked status).

## Root Cause
1. The telemetry data included an `error_code` field (from backend/telemetry_handler.py field 21)
2. The TypeScript interface had an `errorCodes` field but it wasn't being populated
3. The System Diagnostics Panel had error code 8 mapped to "Serial (UART)" instead of GPS lock status
4. Error code 58 (GPS Not Locked - Ground) was missing entirely

## Solution Implemented

### 1. Telemetry Mapper (`lib/telemetry-mapper.ts`)
Added `parseErrorCodes()` function to convert numeric error codes to human-readable messages:

```typescript
function parseErrorCodes(errorCode: number | undefined): Array<string | { code: string; message?: string }> {
  if (errorCode === undefined || errorCode === null || errorCode === 0) return []
  
  const errorMap: Record<number, string> = {
    8: "GPS Not Locked (Flight)",
    58: "GPS Not Locked (Ground)",
  }
  
  const message = errorMap[errorCode] || `Unknown Error Code: ${errorCode}`
  return [{ code: errorCode.toString(), message: message }]
}
```

Updated `mapPythonToTelemetry()` to include:
```typescript
errorCodes: parseErrorCodes(pythonData.error_code),
```

### 2. System Diagnostics Panel (`components/system-diagnostics-panel.tsx`)
Updated the ERROR_CODE_MAP to include GPS lock status codes:

```typescript
const ERROR_CODE_MAP: { [key: string]: string } = {
  '1': 'MPU6050 (Accel/Gyro)',
  '2': 'BMP280 (Pressure)',
  '3': 'SD Card',
  '5': 'GNSS/GPS',
  '6': 'PID Control',
  '7': 'Camera',
  '8': 'GPS Not Locked (Flight)',      // Changed from "Serial (UART)"
  '9': 'LoRa',
  '10': 'BMP390 (Pressure)',
  '58': 'GPS Not Locked (Ground)',     // Added new code
}
```

Enhanced error code parsing to handle both formats:
- Legacy string format: `errorCode` or `ERROR_CODE` fields
- New array format: `errorCodes` field (from the new parser)

Added support for parsing two-digit error codes (10, 58):
```typescript
// Check for two-digit codes first (10, 58)
if (remaining.includes('58')) {
  codes.push('58')
  remaining = remaining.replace('58', '')
}
```

## Data Flow

```
Backend (Python)
  ├─ telemetry_handler.py parses CSV field 21 → error_code: int
  └─ main.py broadcasts via WebSocket → pythonData.error_code

Frontend (TypeScript)
  ├─ telemetry-mapper.ts
  │   └─ parseErrorCodes(error_code) → errorCodes: [{ code: "8", message: "GPS Not Locked (Flight)" }]
  ├─ use-telemetry.ts receives mapped data
  └─ system-diagnostics-panel.tsx displays errors with icons and messages
```

## Error Code Reference

| Code | Meaning | Type |
|------|---------|------|
| 0 | No Error | System OK |
| 1 | MPU6050 (Accel/Gyro) | Hardware |
| 2 | BMP280 (Pressure) | Hardware |
| 3 | SD Card | Storage |
| 5 | GNSS/GPS | Hardware |
| 6 | PID Control | Software |
| 7 | Camera | Hardware |
| **8** | **GPS Not Locked (Flight)** | **GPS Status** |
| 9 | LoRa | Communication |
| 10 | BMP390 (Pressure) | Hardware |
| **58** | **GPS Not Locked (Ground)** | **GPS Status** |

## Testing

To verify the fix:
1. Start the GCS dashboard
2. Connect to the CanSat (or use mock data with error codes)
3. Check the **System Diagnostics Panel** (right panel)
4. When error code 8 or 58 is received, you should see:
   - Error badge showing the count
   - Red error box with icon
   - "Error 8" or "Error 58" label
   - "GPS Not Locked (Flight)" or "GPS Not Locked (Ground)" message

## Files Modified

1. `lib/telemetry-mapper.ts` - Added parseErrorCodes() function
2. `components/system-diagnostics-panel.tsx` - Updated error code mapping and display logic

## Notes

- Error codes are parsed from the backend's CSV field 21 (`error_code`)
- The dashboard now supports both single-digit (1-9) and two-digit (10, 58) error codes
- GPS lock status errors (8, 58) are now correctly displayed to help operators monitor GPS acquisition
- The panel handles both legacy string format and new array format for backward compatibility
