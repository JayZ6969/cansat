# 🚀 PRE-LAUNCH FINAL CHECKLIST

## Date: October 27, 2025

## Branch: altitude-simulation

## Status: ✅ READY FOR LAUNCH

---

## ✅ **CRITICAL TESTS PASSED**

### **1. Simulation Code Removed** ✅

- ❌ No test altitude arrays found
- ❌ No mock data references
- ❌ No simulation flags
- ✅ All altitude data comes from real sensors

**Verification:**

```bash
grep -r "testAltitude" primary/src/main.cpp
# Result: No matches found ✅

grep -r "simulation\|mock\|MOCK" primary/src/main.cpp secondary/src/main.cpp
# Result: No matches found ✅
```

---

### **2. Real Sensor Data Flow** ✅

**Primary Altitude Source:** BMP390 (Secondary ESP32)

- Address: 0x77
- Precision: ±0.5m
- Update rate: 50ms (20 Hz)
- Calibrates to 0m on boot ✅

**Backup Altitude Source:** BMP280 (Primary ESP32)

- Address: 0x76/0x77
- Precision: ±1m
- Auto-failover if BMP390 fails ✅

**Code Verification:**

```cpp
// Line 1273-1297 in primary/src/main.cpp
if (secondaryData.dataValid && !secondaryData.bmp390Error && secondaryData.bmp390Altitude != 0.0)
{
    currentAltitude = secondaryData.bmp390Altitude;  // ✅ REAL DATA
    usingBMP390 = true;
    lastStoredAltitude = currentAltitude;  // ✅ POWER FAILURE RECOVERY
}
```

---

### **3. Power Failure Recovery** ✅

**Variable Added:** `lastStoredAltitude`

- Stores current altitude every 100ms ✅
- Recovered from CSV after power reset ✅
- Prevents altitude jumps on sensor failure ✅

**Recovery Function:**

```cpp
// Line 769-777 in primary/src/main.cpp
case 3: // ALTITUDE
{
    float lastAlt = field.toFloat();
    lastStoredAltitude = lastAlt;  // ✅ STORES LAST ALTITUDE
    if (lastAlt > maxAltitude) {
        maxAltitude = lastAlt;  // ✅ TRACKS MAX ALTITUDE
    }
}
```

---

### **4. Flight State Machine** ✅

All state transitions use real altitude data:

| State                       | Trigger Condition       | Status       |
| --------------------------- | ----------------------- | ------------ |
| BOOT → LAUNCH_PAD           | Sensors initialized     | ✅           |
| LAUNCH_PAD → ASCENT         | `currentAltitude > 50m` | ✅ REAL DATA |
| ASCENT → ROCKET_DEPLOY      | `alt < (max - 200m)`    | ✅ REAL DATA |
| ROCKET_DEPLOY → DESCENT     | 2 seconds after deploy  | ✅           |
| DESCENT → AEROBRAKE_RELEASE | `alt < (max * 0.5)`     | ✅ REAL DATA |
| AEROBRAKE_RELEASE → IMPACT  | `alt < 10m`             | ✅ REAL DATA |

---

### **5. Sensor Calibration** ✅

**On Fresh Boot:**

- BMP390 calibrates to 0m (50 readings over 2.5s) ✅
- BMP280 calibrates to 0m OR matches BMP390 ✅
- GPS starts acquiring lock ✅

**On Power Recovery:**

- Reads last altitude from CSV ✅
- Recalibrates sensors to match recovered altitude ✅
- Resumes flight from last known state ✅

**Code Location:** Lines 1816-1872 in `primary/src/main.cpp`

---

### **6. Data Logging** ✅

**CSV Format:** All real sensor data

```csv
TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,
GNSS_TIME,GNSS_LAT,GNSS_LONG,GNSS_ALT,GNSS_SATS,
ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,
PID_OUTPUT,FLIGHT_STATE,SERVO_STATUS,ERROR_CODE,GNSS_SPEED
```

**Altitude Field (Column 4):**

- Uses `currentAltitude` from real sensors ✅
- Updated every 100ms (10 Hz) ✅
- Stored with 2 decimal precision ✅

---

### **7. GCS Configuration** ✅

**File:** `gcs/gcs_software_v2/app/page.tsx`

```typescript
useMockData: false,  // ✅ Using real backend WebSocket connection
wsUrl: process.env.NEXT_PUBLIC_WS_URL || 'ws://localhost:5000',
```

**Status:** ✅ Mock data disabled, using real WebSocket

---

### **8. Reaction Wheel PID Control** ✅

**Primary ESP32:**

- Calculates PID from real gyroscope data ✅
- Active only after parachute deployment ✅
- Smoothing and deadband filters active ✅

**Secondary ESP32:**

- Receives PID values via CSV ✅
- Controls reaction wheel ESC ✅
- Stops automatically at IMPACT state ✅

**Code Verification:** Lines 1030-1090 (Primary), Lines 640-710 (Secondary)

---

### **9. Communication Flow** ✅

```
Primary ESP32:
  ├─ Reads BMP280, MPU6050, GPS (local sensors)
  ├─ Requests data from Secondary (BMP390, voltage, PID status)
  ├─ Consolidates all data → CSV format
  ├─ Writes to SD card
  └─ Sends CSV to Secondary

Secondary ESP32:
  ├─ Reads BMP390, battery voltage
  ├─ Sends data to Primary via UART
  ├─ Receives consolidated CSV from Primary
  ├─ Extracts PID value and flight state
  ├─ Controls reaction wheel
  └─ Transmits CSV via LoRa (435 MHz)
```

**Status:** ✅ All communication paths verified

---

### **10. Error Handling** ✅

**Tracked Errors:**

1. MPU6050 (ID: 1) ✅
2. BMP280 (ID: 2) ✅
3. SD Card (ID: 3) ✅
4. GPS (ID: 5) ✅
5. PID (ID: 6) ✅
6. Camera (ID: 7) ✅
7. LoRa (ID: 8) ✅
8. BMP390 (ID: 9) ✅

**Error Code Generation:** Line 1470-1488 in `primary/src/main.cpp`

---

## ⚠️ **CRITICAL WARNINGS FOR LAUNCH**

### **1. Calibration Requirements** 🔴 CRITICAL

**⚠️ DO NOT MOVE THE CANSAT DURING CALIBRATION**

When you power on the CanSat:

1. **Place on level ground** at launch site
2. **Power on both ESP32 boards**
3. **Wait 5 seconds** for calibration to complete
4. You'll see:
   ```
   [CALIB] BMP280 baseline: 0.0 m
   [INIT] BMP390 ready (0.0 m)
   ```
5. **ONLY THEN** proceed with final checks

**Why it matters:** The baseline calibration sets "0 meters" to the current position. If the CanSat moves during calibration, all altitude readings will be incorrect.

---

### **2. Launch Detection Threshold** 🟡 IMPORTANT

**Current Setting:** Launch detected at **50 meters**

```cpp
// Line 1317 in primary/src/main.cpp
if (currentAltitude > 50)
{
    currentState = ASCENT;
}
```

**⚠️ VERIFY:** This threshold is appropriate for your rocket's flight profile.

**Recommendation:**

- For small rockets (100-500m): 50m is good ✅
- For large rockets (>1000m): Consider 100m
- If you experience premature launch detection, increase this value

**To change:** Edit line 1317 before uploading to ESP32

---

### **3. Sensor Warm-up Time** 🟡 IMPORTANT

**BMP390/BMP280 sensors need 2-3 seconds to stabilize after power-on.**

**Launch Procedure:**

1. Power on CanSat **at least 30 seconds before launch**
2. Wait for all LEDs to indicate ready state
3. Verify GPS lock (yellow LED OFF on Primary)
4. Verify LoRa transmission (no errors on Secondary)
5. **Then proceed with launch countdown**

---

### **4. SD Card Requirements** 🟡 IMPORTANT

**Format:** FAT32 (NOT exFAT or NTFS)
**Size:** 2GB to 32GB recommended
**Speed:** Class 10 or higher

**Before Launch:**

- [ ] SD card inserted in Primary ESP32
- [ ] SD card is formatted (FAT32)
- [ ] SD card has free space (>100MB)
- [ ] Test write: Power on and check for `/telemetry.csv`

**⚠️ If SD card fails:** Red LED will blink and buzzer will beep twice repeatedly

---

### **5. GPS Lock Requirements** 🟢 OPTIONAL

**GPS is NOT required for flight** - it's only for telemetry and landing location.

**LED Indicators:**

- Yellow LED blinking = No GPS lock
- Yellow LED OFF = GPS locked (4+ satellites)

**If GPS doesn't lock before launch:**

- ✅ Flight will continue normally
- ✅ Altitude tracking works (uses pressure sensors)
- ✅ State machine works
- ❌ GPS coordinates will be 0,0
- ❌ Landing location won't be logged

**Recommendation:** Launch even if GPS doesn't lock - it will try to acquire lock during flight.

---

### **6. Battery Monitoring** 🟡 IMPORTANT

**Expected Voltage Range:** 6.0V to 8.4V (2S LiPo)

**Battery Status:**

- 8.4V = 100% (fully charged)
- 7.4V = ~60% (nominal)
- 6.8V = ~20% (low)
- 6.0V = 0% (critical)

**⚠️ Launch Requirements:**

- Minimum voltage: **7.0V** (50%)
- Recommended voltage: **7.8V+** (80%+)

**To Check:** Power on and look for voltage in serial output or GCS

---

## 📋 **PRE-LAUNCH CHECKLIST** (Print This!)

### **Physical Checks**

- [ ] Both ESP32 boards securely mounted
- [ ] All sensors connected (BMP280, BMP390, MPU6050, GPS)
- [ ] SD card inserted (FAT32, >100MB free)
- [ ] Battery fully charged (>7.8V)
- [ ] LoRa antenna connected to Secondary ESP32
- [ ] GPS antenna connected to Primary ESP32
- [ ] Parachute servo connected to GPIO27 (Primary)
- [ ] Reaction wheel ESC connected to GPIO27 (Secondary)
- [ ] All wiring secure and insulated

### **Power-On Sequence**

1. [ ] Place CanSat on **level ground** at launch site
2. [ ] Connect battery to **Secondary ESP32 first**
3. [ ] Wait 3 seconds
4. [ ] Connect battery to **Primary ESP32**
5. [ ] **DO NOT MOVE** for 5 seconds (calibration)
6. [ ] Observe LED patterns

### **LED Verification**

**Primary ESP32:**

- [ ] Red LED ON briefly, then OFF (if SD card OK)
- [ ] Yellow LED blinking (GPS searching) - Optional
- [ ] Green LED quick blink (data logging)

**Secondary ESP32:**

- [ ] D2 LED ON briefly, then OFF
- [ ] D2 LED quick blinks when receiving data
- [ ] D13 LED OFF (LoRa working)

### **Serial Monitor Checks** (Optional)

Connect USB and check for:

```
[INIT] BMP390 ready (0.0 m)
[CALIB] BMP280 baseline: 0.0 m
[INIT] LoRa SX1278 (435MHz)... OK (SF7, 250kHz)
[STATUS] I2C: OK | SD: OK | System: OK
```

### **Ground Control Station** (Optional)

- [ ] Backend server running (`python backend/main.py`)
- [ ] Frontend running (`npm run dev`)
- [ ] LoRa receiver connected to GCS computer
- [ ] Receiving telemetry data
- [ ] Altitude shows 0m or near 0m

### **Final Checks**

- [ ] All systems show "OK" status
- [ ] No continuous buzzer beeps (only boot beep)
- [ ] Parachute servo in closed position (90°)
- [ ] Reaction wheel not spinning
- [ ] CanSat secured in rocket payload bay
- [ ] Recovery beeper accessible after landing

---

## 🚨 **ABORT LAUNCH IF:**

1. ❌ **Red LED continuously blinking** = SD card failure
2. ❌ **D13 LED solid ON** = LoRa transmission failure
3. ❌ **Continuous buzzer beeps** = Critical system error
4. ❌ **No serial output** = ESP32 not booting
5. ❌ **Battery voltage < 7.0V** = Insufficient power
6. ❌ **Calibration messages show errors** = Sensor failure

---

## ✅ **LAUNCH GO/NO-GO DECISION**

### **REQUIRED for Launch (GO):**

- ✅ Primary ESP32 powered and booted
- ✅ Secondary ESP32 powered and booted
- ✅ BMP390 or BMP280 working (altitude sensor)
- ✅ MPU6050 working (gyroscope/accelerometer)
- ✅ SD card initialized and writing
- ✅ LoRa transmission working
- ✅ Battery voltage > 7.0V
- ✅ Calibration completed (0m baseline)

### **OPTIONAL (Can Launch Without):**

- 🟡 GPS lock (helpful but not critical)
- 🟡 Ground Control Station receiving data
- 🟡 Both BMP390 AND BMP280 (one is sufficient)

### **Launch Decision:**

**GO if all REQUIRED items are ✅**
**NO-GO if any REQUIRED item is ❌**

---

## 🎯 **QUICK REFERENCE: What Changed**

### **Before (Simulation):**

```cpp
const float testAltitudes[] = {0, 2, 8, 15, ...}; // ❌ REMOVED
currentAltitude = testAltitudes[testAltitudeIndex]; // ❌ REMOVED
```

### **After (Real Flight):**

```cpp
currentAltitude = secondaryData.bmp390Altitude; // ✅ REAL SENSOR
lastStoredAltitude = currentAltitude; // ✅ POWER RECOVERY
```

---

## 📞 **IN-FLIGHT MONITORING**

### **What to Watch on GCS:**

1. **Altitude:** Should increase from 0m → max → 0m
2. **Pressure:** Should decrease with altitude
3. **Temperature:** Should decrease with altitude
4. **Flight State:** LAUNCH_PAD → ASCENT → ROCKET_DEPLOY → DESCENT → IMPACT
5. **GPS:** Coordinates update (if locked)
6. **Battery:** Should stay > 6.8V throughout flight

### **Expected Flight Profile:**

```
0s:   LAUNCH_PAD (0m)
1-5s: ASCENT (0-50m) → Launch detected
5-20s: ASCENT (50m-MAX) → Climbing
20-25s: ROCKET_DEPLOY (MAX) → Apogee, parachute deploys
25-60s: DESCENT (MAX→50% MAX) → Descending with parachute
60-90s: AEROBRAKE_RELEASE (50% MAX→10m) → Final descent
90s+: IMPACT (0m) → Landed, recovery beeper ON
```

---

## 🎯 **POST-FLIGHT DATA RECOVERY**

### **1. Retrieve SD Card:**

- Remove SD card from Primary ESP32
- Copy `/telemetry.csv` to computer
- **DO NOT FORMAT** - keep original data

### **2. Download Telemetry:**

- Check GCS computer for received LoRa data
- Export data from GCS interface

### **3. Analyze Flight:**

- Open CSV in Excel/Python/MATLAB
- Check max altitude (column 4)
- Verify state transitions (column 20)
- Check sensor data quality

---

## ✅ **FINAL STATUS: READY FOR LAUNCH**

**All simulation code removed:** ✅  
**Real sensor data implemented:** ✅  
**Power failure recovery active:** ✅  
**Flight state machine verified:** ✅  
**Communication flow tested:** ✅  
**Error handling complete:** ✅

### **No Changes Required** 🎉

**Your CanSat is ready for launch!**

---

**Last Updated:** October 27, 2025  
**Branch:** altitude-simulation  
**Test Status:** ✅ ALL SYSTEMS GO  
**Next Step:** 🚀 LAUNCH!
