# LED Status Fixes - PRIMARY ESP32

## Issue Summary
The LED status indicators were not working as intended. The following fixes have been implemented:

## LED Pin Assignments (Confirmed)
- **D12 (GPIO 12)**: RED LED
- **D13 (GPIO 13)**: YELLOW LED  
- **D14 (GPIO 14)**: GREEN LED

## Fixed LED Behavior

### RED LED (D12)
- **Function**: Boot state indicator
- **Behavior**: 
  - ON during BOOT state only
  - OFF in all other states (TEST_MODE, LAUNCH_PAD, ASCENT, etc.)

### YELLOW LED (D13)
- **Function**: GPS lock status indicator
- **Behavior**:
  - ON when GPS is NOT locked (no satellites or invalid location)
  - OFF when GPS is locked (valid location + ≥4 satellites + data age <5 seconds)

### GREEN LED (D14)
- **Function**: System health and data operation indicator
- **Behavior**:
  - Steady ON when all sensors are OK (`sensorsOK = true`)
  - Steady OFF when sensors have issues
  - **BLINKS** during data operations (SD write + transmission to secondary)
  - Blink pattern: 50ms ON, 50ms OFF for 100ms total duration

## Key Fixes Implemented

### 1. Improved GPS Lock Detection
```cpp
// Fixed GPS lock logic - now properly sets both true and false states
if (gps.location.isValid() && gps.location.age() < 5000 && primaryData.gnssSats >= 4) {
  gpsLocked = true;
} else {
  gpsLocked = false;
}
```

### 2. Enhanced Green LED Blinking
```cpp
// Green LED now blinks with 50ms intervals during data operations
if (greenBlinkStart > 0 && (millis() - greenBlinkStart) < GREEN_BLINK_DURATION) {
  unsigned long blinkElapsed = millis() - greenBlinkStart;
  bool blinkState = (blinkElapsed / 50) % 2 == 0; // Toggle every 50ms
  digitalWrite(LED_GREEN_PIN, blinkState ? HIGH : LOW);
}
```

### 3. Proper Data Operation Triggering
```cpp
// Green LED blinks when BOTH SD write AND transmission start
triggerGreenBlink(); // Called before SD write and transmission
```

### 4. Added Debug Output
```cpp
Serial.println("LED Status - RED: " + String(currentState == BOOT ? "ON" : "OFF") + 
               ", YELLOW: " + String(!gpsLocked ? "ON" : "OFF") + 
               ", GREEN: " + String(sensorsOK ? "ON" : "OFF"));
```

## Expected LED Behavior by State

### During Boot (First 5 seconds)
- RED: ON
- YELLOW: ON (GPS not locked yet)
- GREEN: OFF (sensors initializing)

### Normal Operation (GPS Locked, Sensors OK)
- RED: OFF
- YELLOW: OFF  
- GREEN: ON (steady), BLINKS during data operations

### GPS Issues (No Lock)
- RED: OFF
- YELLOW: ON
- GREEN: ON/OFF (depends on other sensors)

### Sensor Issues
- RED: OFF
- YELLOW: ON/OFF (depends on GPS)
- GREEN: OFF

## Troubleshooting

If LEDs still don't work as expected:

1. **Check Hardware Connections**: Verify GPIO 12, 13, 14 are properly connected
2. **Check Serial Monitor**: Look for debug output showing LED states
3. **Check GPS**: Monitor satellite count and lock status
4. **Check Sensors**: Verify BMP280, MPU6050, and SD card initialization

## Testing Procedure

1. **Power On**: RED should be ON for 5 seconds
2. **Wait for GPS**: YELLOW should be ON until GPS gets ≥4 satellites
3. **Check Sensors**: GREEN should be ON when all systems operational
4. **Data Operations**: GREEN should blink every second during SD write/transmission

The LED system now provides clear visual feedback for system status and data operations.
