/*
 * PRIMARY ESP32 - CanSat Mission Control
 * Board: ESP32 Devkit (30 pins)
 * Role: Main telemetry collection, GPS tracking, SD logging, UART coordination
 *
 * Hardware Configuration:
 * - GPS NEO-6M: UART2 (GPIO16-RX, GPIO17-TX)
 * - BMP280: I2C (GPIO21-SDA, GPIO22-SCL) - Fallback sensor
 * - MPU6050: I2C (GPIO21-SDA, GPIO22-SCL)
 * - SD Card: SPI (GPIO23-MOSI, GPIO19-MISO, GPIO18-SCK, GPIO5-CS)
 * - Buzzer: GPIO0 (Boot pin via transistor, active HIGH)
 * - LEDs: D12(RED), D13(YELLOW), D14(GREEN)
 * - Servo: GPIO27 (Parachute deployment)
 * - UART0: Communication with Secondary ESP32
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <HardwareSerial.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>

// ==================== PIN DEFINITIONS ====================
// LEDs
#define LED_RED_PIN 12
#define LED_YELLOW_PIN 13
#define LED_GREEN_PIN 14

// Buzzer
#define BUZZER_PIN 2 // GPIO2

// ==================== ERROR CODE DEFINITIONS ====================
// Subsystem IDs for error tracking
#define ERROR_MPU6050 1
#define ERROR_BMP280 2
#define ERROR_SD_CARD 3
#define ERROR_BATTERY 4
#define ERROR_GNSS 5
#define ERROR_PID 6
#define ERROR_CAMERA 7
#define ERROR_SERIAL 8
#define ERROR_LORA 9
#define ERROR_BMP390 10

// SPI SD Card
#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

// I2C
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// UART GPS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Servo
#define SERVO_PIN 27

// ==================== GLOBAL OBJECTS ====================
HardwareSerial gpsSerial(2);       // UART2 for GPS
HardwareSerial secondarySerial(0); // UART0 for Secondary ESP32
TinyGPSPlus gps;
Adafruit_BMP280 bmp280;
Adafruit_MPU6050 mpu6050;
Servo lidServo; // Servo for parachute deployment

// ==================== GLOBAL VARIABLES ====================
// Mission data
const String TEAM_ID = "2024-ASI-CANSAT-049";
uint32_t packetCount = 0;
unsigned long missionStartTime = 0;

// Flight states
enum FlightState
{
  BOOT = 0,
  TEST_MODE = 1,
  LAUNCH_PAD = 2,
  ASCENT = 3,
  ROCKET_DEPLOY = 4,
  DESCENT = 5,
  AEROBRAKE_RELEASE = 6,
  IMPACT = 7
};
FlightState currentState = BOOT;

// Sensor data structures
struct SensorData
{
  float altitude = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
  String gnssTime = "";
  float gnssLat = 0.0;
  float gnssLong = 0.0;
  float gnssAlt = 0.0;
  int gnssSats = 0;
  float gnssSpeed = 0.0; // GNSS speed in m/s
  float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
  float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
  float gyroSpinRate = 0.0;
  String optionalData = "";
} primaryData;

struct SecondaryData
{
  float bmp390Altitude = 0.0;
  float bmp390Pressure = 0.0;
  float bmp390Temperature = 0.0;
  float voltage = 0.0;
  String servoStatus = "";
  String pidOutput = "";
  bool dataValid = false;
  bool pidError = false;    // Error status from Secondary
  bool loraError = false;   // Error status from Secondary
  bool bmp390Error = false; // Error status from Secondary
} secondaryData;

// Error tracking structure
struct SystemErrors
{
  bool mpu6050Error = false; // ID: 1
  bool bmp280Error = false;  // ID: 2
  bool sdCardError = false;  // ID: 3
  bool gnssError = false;    // ID: 5
  bool pidError = false;     // ID: 6 (from Secondary)
  bool cameraError = false;  // ID: 7
  bool serialError = false;  // ID: 8
  bool loraError = false;    // ID: 9 (from Secondary)
  bool bmp390Error = false;  // ID: 10 (from Secondary)
} systemErrors;

// System status
bool sensorsOK = false;
bool gpsLocked = false;
bool sdCardOK = false;

// Timing variables
unsigned long lastDataCollection = 0;
unsigned long lastGreenBlink = 0;
unsigned long lastBuzzerUpdate = 0;
unsigned long lastUARTRequest = 0;
unsigned long lastStateUpdate = 0;

// LED/Buzzer control
bool greenLedState = false;
bool buzzerState = false;
unsigned long greenBlinkStart = 0;
unsigned long yellowBlinkStart = 0;
unsigned long buzzerBeaconStart = 0;
unsigned long buzzerPatternStart = 0;
int buzzerBeepCount = 0;
int buzzerTargetBeeps = 0;

// Buzzer patterns for different states
enum BuzzerPattern
{
  BUZZER_OFF = 0,
  BUZZER_BOOT = 1,    // 3 short beeps at startup
  BUZZER_STATUS = 2,  // Status indication (1-3 beeps based on system health)
  BUZZER_RECOVERY = 3 // Long beeps for recovery after landing
};
BuzzerPattern currentBuzzerPattern = BUZZER_OFF;

// Data collection intervals - OPTIMIZED FOR MAXIMUM SPEED
const unsigned long DATA_INTERVAL = 100;         // 10Hz data collection (maximum speed)
const unsigned long UART_INTERVAL = 50;          // UART request every 50ms (fast sync)
const unsigned long STATE_UPDATE_INTERVAL = 100; // State check every 100ms
const unsigned long GREEN_BLINK_DURATION = 100;  // 100ms blink duration
const unsigned long YELLOW_BLINK_DURATION = 100; // 100ms blink duration

// Buzzer timing - short and simple
const unsigned long BUZZER_BEEP_SHORT = 150;         // 150ms short beep
const unsigned long BUZZER_BEEP_LONG = 500;          // 500ms long beep
const unsigned long BUZZER_GAP_SHORT = 200;          // 200ms gap between beeps
const unsigned long BUZZER_RECOVERY_INTERVAL = 3000; // 3s between recovery beeps

// Flight logic variables
float maxAltitude = 0.0;
bool apogeeReached = false;
float currentAltitude = 0.0;
unsigned long ascentStartTime = 0;
bool servoOpen = false; // Track servo position

// BMP280 baseline calibration
float baselineAltitudeBMP280 = 0.0;

// ==================== INITIALIZATION FUNCTIONS ====================

void initializePins();
bool initializeI2C();
bool initializeSD();
void initializeUART();
void calibrateBaselineBMP280();
void initializeServo();

// ==================== SENSOR READING FUNCTIONS ====================
void readGPS();
bool readMPU6050();
bool readBMP280();

// ==================== UART COMMUNICATION ====================
bool requestSecondaryData();
void sendConsolidatedData(String csvRow);

// ==================== DATA MANAGEMENT ====================
String createCSVRow();
bool writeToSD(String csvRow);

// ==================== FLIGHT STATE MANAGEMENT ====================
void updateFlightState();

// ==================== LED AND BUZZER CONTROL ====================
void updateLEDs();
void updateBuzzer();
void triggerGreenBlink();
void executeBuzzerBeeps(unsigned long beepDuration, unsigned long gapDuration);

// ==================== SYSTEM STATUS CHECK ====================
void checkSystemStatus();

// ==================== ERROR CHECKING FUNCTIONS ====================
void checkAllSubsystemErrors();
String generateErrorCode();
void checkMPU6050Status();
void checkBMP280Status();
void checkSDCardStatus();
void checkGNSSStatus();
void checkSerialStatus();
void checkCameraStatus();

// ==================== INITIALIZATION FUNCTIONS ====================

void initializePins()
{
  // LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  // Buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // Initial state - RED on during boot
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW); // Buzzer starts OFF, will beep according to pattern

  Serial.println("Pins initialized");
}

bool initializeI2C()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400kHz

  // Initialize BMP280
  if (!bmp280.begin(0x76))
  {
    if (!bmp280.begin(0x77))
    {
      Serial.println("BMP280 initialization failed!");
      return false;
    }
  }

  // Configure BMP280
  bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);

  // Initialize MPU6050
  if (!mpu6050.begin())
  {
    Serial.println("MPU6050 initialization failed!");
    return false;
  }

  // Configure MPU6050
  mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu6050.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("I2C sensors initialized successfully");
  return true;
}

bool initializeSD()
{
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("SD Card initialization failed!");
    return false;
  }

  // Create CSV header if file doesn't exist
  if (!SD.exists("/telemetry.csv"))
  {
    File file = SD.open("/telemetry.csv", FILE_WRITE);
    if (file)
    {
      file.println("TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GNSS_TIME,GNSS_LAT,GNSS_LONG,GNSS_ALT,GNSS_SATS,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,GYRO_SPIN_RATE,FLIGHT_STATE,SERVO_STATUS,ERROR_CODE,GNSS_SPEED");
      file.close();
      Serial.println("CSV header created");
    }
    else
    {
      Serial.println("Failed to create CSV file");
      return false;
    }
  }

  Serial.println("SD Card initialized successfully");
  return true;
}

void initializeUART()
{
  // GPS on UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Secondary ESP32 on UART0 (default Serial)
  Serial.begin(115200);

  Serial.println("UART initialized");
}

void calibrateBaselineBMP280()
{
  Serial.println("Calibrating BMP280 baseline altitude...");
  float altSum = 0;
  int validReadings = 0;

  // Take 50 readings over 2.5 seconds at startup
  for (int i = 0; i < 50; i++)
  {
    float alt = bmp280.readAltitude(1013.25);
    if (!isnan(alt))
    {
      altSum += alt;
      validReadings++;
    }
    delay(50);
  }

  if (validReadings > 0)
  {
    baselineAltitudeBMP280 = altSum / validReadings;
    Serial.print("BMP280 baseline altitude set to: ");
    Serial.println(baselineAltitudeBMP280);
  }
  else
  {
    Serial.println("Failed to calibrate BMP280 baseline - no valid readings");
    baselineAltitudeBMP280 = 0.0;
  }
}

void initializeServo()
{
  Serial.print("Initializing parachute servo...");

  lidServo.attach(SERVO_PIN);
  lidServo.write(90); // Start with parachute closed (90 degrees)
  servoOpen = false;

  Serial.println(" SUCCESS");
}

// ==================== SENSOR READING FUNCTIONS ====================

void readGPS()
{
  unsigned long start = millis();
  while (gpsSerial.available() && (millis() - start) < 100)
  {
    if (gps.encode(gpsSerial.read()))
    {
      if (gps.location.isValid())
      {
        primaryData.gnssLat = gps.location.lat();
        primaryData.gnssLong = gps.location.lng();
      }

      if (gps.altitude.isValid())
      {
        primaryData.gnssAlt = gps.altitude.meters();
      }

      if (gps.satellites.isValid())
      {
        primaryData.gnssSats = gps.satellites.value();
      }

      if (gps.speed.isValid())
      {
        primaryData.gnssSpeed = gps.speed.mps(); // Speed in meters/second
      }

      if (gps.time.isValid())
      {
        char timeStr[20];
        sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        primaryData.gnssTime = String(timeStr);
      }
    }
  }

  // Update GPS lock status - locked if we have valid location data and satellites
  if (gps.location.isValid() && gps.location.age() < 5000 && primaryData.gnssSats >= 4)
  {
    gpsLocked = true;
  }
  else
  {
    gpsLocked = false;
  }
}

bool readMPU6050()
{
  sensors_event_t accel, gyro, temp;

  mpu6050.getEvent(&accel, &gyro, &temp);

  // Store accelerometer data (already in m/s²)
  primaryData.accelX = accel.acceleration.x / 9.81; // Convert to g-force
  primaryData.accelY = accel.acceleration.y / 9.81;
  primaryData.accelZ = accel.acceleration.z / 9.81;

  // Store gyroscope data (already in rad/s, convert to °/s)
  primaryData.gyroX = gyro.gyro.x * 180.0 / PI;
  primaryData.gyroY = gyro.gyro.y * 180.0 / PI;
  primaryData.gyroZ = gyro.gyro.z * 180.0 / PI;

  // Calculate spin rate (magnitude of gyro vector)
  primaryData.gyroSpinRate = sqrt(primaryData.gyroX * primaryData.gyroX +
                                  primaryData.gyroY * primaryData.gyroY +
                                  primaryData.gyroZ * primaryData.gyroZ);

  return true;
}

bool readBMP280()
{
  primaryData.temperature = bmp280.readTemperature();
  primaryData.pressure = bmp280.readPressure() / 100.0;                         // Convert to hPa
  primaryData.altitude = bmp280.readAltitude(1013.25) - baselineAltitudeBMP280; // Zero-referenced altitude

  return (!isnan(primaryData.temperature) && !isnan(primaryData.pressure));
}

// ==================== UART COMMUNICATION ====================

bool requestSecondaryData()
{
  // Send request to secondary ESP32
  Serial.println("REQ_DATA");

  unsigned long timeout = millis() + 500; // 500ms timeout
  String response = "";

  while (millis() < timeout)
  {
    if (Serial.available())
    {
      response = Serial.readStringUntil('\n');
      response.trim();
      break;
    }
    delay(10);
  }

  if (response.length() == 0)
  {
    secondaryData.dataValid = false;
    return false;
  }

  // Parse secondary data: "ALT:123.45,PRESS:1013.25,TEMP:25.5,VOLT:7.20,PID:0.75,ERR:090"
  if (response.startsWith("DATA:"))
  {
    response = response.substring(5); // Remove "DATA:" prefix

    int altIndex = response.indexOf("ALT:");
    int pressIndex = response.indexOf("PRESS:");
    int tempIndex = response.indexOf("TEMP:");
    int voltIndex = response.indexOf("VOLT:");
    int pidIndex = response.indexOf("PID:");
    int errIndex = response.indexOf("ERR:");

    if (altIndex >= 0 && pressIndex >= 0 && tempIndex >= 0)
    {
      secondaryData.bmp390Altitude = response.substring(altIndex + 4, response.indexOf(',', altIndex)).toFloat();
      secondaryData.bmp390Pressure = response.substring(pressIndex + 6, response.indexOf(',', pressIndex)).toFloat();
      secondaryData.bmp390Temperature = response.substring(tempIndex + 5, response.indexOf(',', tempIndex)).toFloat();

      if (voltIndex >= 0)
      {
        secondaryData.voltage = response.substring(voltIndex + 5, response.indexOf(',', voltIndex)).toFloat();
      }

      if (pidIndex >= 0)
      {
        secondaryData.pidOutput = response.substring(pidIndex + 4, response.indexOf(',', pidIndex));
      }

      // Parse error status from Secondary
      if (errIndex >= 0)
      {
        String errorStatus = response.substring(errIndex + 4);
        // Error format: "ERR:xyz" where x=BMP390, y=LoRa, z=PID (0=OK, 1=Error)
        if (errorStatus.length() >= 3)
        {
          secondaryData.bmp390Error = (errorStatus.charAt(0) == '1');
          secondaryData.loraError = (errorStatus.charAt(1) == '1');
          secondaryData.pidError = (errorStatus.charAt(2) == '1');
        }
      }
      else
      {
        // Default to no errors if not provided
        secondaryData.bmp390Error = false;
        secondaryData.loraError = false;
        secondaryData.pidError = false;
      }

      secondaryData.dataValid = true;
      return true;
    }
  }

  secondaryData.dataValid = false;
  return false;
}

void sendConsolidatedData(String csvRow)
{
  Serial.println("CSV:" + csvRow);
}

// ==================== DATA MANAGEMENT ====================

String createCSVRow()
{
  String csvRow = "";

  // Use BMP390 data from secondary if available, otherwise fallback to BMP280
  float useAltitude = secondaryData.dataValid ? secondaryData.bmp390Altitude : primaryData.altitude;
  float usePressure = secondaryData.dataValid ? secondaryData.bmp390Pressure : primaryData.pressure;
  float useTemperature = secondaryData.dataValid ? secondaryData.bmp390Temperature : primaryData.temperature;

  csvRow += TEAM_ID + ",";
  csvRow += String(millis()) + ",";
  csvRow += String(packetCount) + ",";
  csvRow += String(useAltitude, 2) + ",";
  csvRow += String(usePressure, 2) + ",";
  csvRow += String(useTemperature, 2) + ",";
  csvRow += String(secondaryData.voltage, 2) + ",";
  csvRow += primaryData.gnssTime + ",";
  csvRow += String(primaryData.gnssLat, 6) + ",";
  csvRow += String(primaryData.gnssLong, 6) + ",";
  csvRow += String(primaryData.gnssAlt, 2) + ",";
  csvRow += String(primaryData.gnssSats) + ",";
  csvRow += String(primaryData.accelX, 3) + ",";
  csvRow += String(primaryData.accelY, 3) + ",";
  csvRow += String(primaryData.accelZ, 3) + ",";
  csvRow += String(primaryData.gyroX, 3) + ",";
  csvRow += String(primaryData.gyroY, 3) + ",";
  csvRow += String(primaryData.gyroZ, 3) + ",";
  csvRow += String(primaryData.gyroSpinRate, 3) + ",";
  csvRow += String(currentState) + ",";

  // Servo status: 0 = closed, 1 = open
  String servoStatus = servoOpen ? "1" : "0";

  csvRow += servoStatus + ",";

  // Add ErrorCode as 22nd column
  csvRow += generateErrorCode() + ",";

  // Add GNSS_SPEED as the last column
  csvRow += String(primaryData.gnssSpeed, 2);

  return csvRow;
}

bool writeToSD(String csvRow)
{
  File file = SD.open("/telemetry.csv", FILE_APPEND);
  if (file)
  {
    file.println(csvRow);
    file.close();
    return true;
  }
  return false;
}

// ==================== FLIGHT STATE MANAGEMENT ====================

void updateFlightState()
{
  // Update current altitude for state decisions
  currentAltitude = secondaryData.dataValid ? secondaryData.bmp390Altitude : primaryData.altitude;

  switch (currentState)
  {
  case BOOT:
    if (millis() - missionStartTime > 5000)
    { // 5 seconds boot time
      currentState = TEST_MODE;
      Serial.println("FLIGHT STATE: BOOT -> TEST_MODE");
    }
    break;

  case TEST_MODE:
    // Transition to LAUNCH_PAD when sensors are stable (GPS lock not required)
    if (sensorsOK && currentAltitude < 100)
    {
      currentState = LAUNCH_PAD;
    }
    break;

  case LAUNCH_PAD:
    // Detect launch by significant altitude increase
    if (currentAltitude > 50)
    {
      currentState = ASCENT;
      ascentStartTime = millis();
    }
    break;

  case ASCENT:
    // Track maximum altitude
    if (currentAltitude > maxAltitude)
    {
      maxAltitude = currentAltitude;
    }

    // Detect apogee (altitude decrease after significant ascent)
    if (currentAltitude < (maxAltitude - 100) && maxAltitude > 100)
    {
      currentState = ROCKET_DEPLOY;
      apogeeReached = true;
    }
    break;

  case ROCKET_DEPLOY:
    // Deploy parachute immediately when entering this state
    if (!servoOpen)
    {
      lidServo.write(0); // Open parachute (0 degrees)
      servoOpen = true;
      Serial.println("PARACHUTE DEPLOYED - Apogee detected");
    }

    // Transition to descent after a brief period
    if (millis() - ascentStartTime > 2000)
    {
      currentState = DESCENT;
    }
    break;

  case DESCENT:
    // Detect aerobrake release (could be based on altitude or time)
    if (currentAltitude < (maxAltitude * 0.5))
    {
      currentState = AEROBRAKE_RELEASE;
    }
    break;

  case AEROBRAKE_RELEASE:
    // Detect impact (low altitude + low vertical velocity)
    if (currentAltitude < 10 && abs(primaryData.accelZ) > 2.0)
    {
      currentState = IMPACT;
    }
    break;

  case IMPACT:
    // Stay in impact state
    break;
  }
}

// ==================== LED AND BUZZER CONTROL ====================

void updateLEDs()
{
  // LED Priority Logic - Only one LED can be active at a time

  // Priority 1: RED LED during BOOT state only
  if (currentState == BOOT)
  {
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_YELLOW_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }

  // All other states: RED LED is OFF
  digitalWrite(LED_RED_PIN, LOW);

  // Priority 2: GREEN LED when everything is OK and GPS is locked
  if (sensorsOK && gpsLocked)
  {
    // GREEN LED blinks once every 2 seconds when all systems are good
    if (greenBlinkStart == 0)
    {
      greenBlinkStart = millis();
    }

    unsigned long greenElapsed = millis() - greenBlinkStart;

    if (greenElapsed < GREEN_BLINK_DURATION)
    {
      // Blinking - ON for 25ms, OFF for 75ms within the 100ms blink period
      if (greenElapsed < 25)
      {
        digitalWrite(LED_GREEN_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_GREEN_PIN, LOW);
      }
    }
    else if (greenElapsed >= 2000)
    {
      // Start new blink cycle every 2 seconds
      greenBlinkStart = millis();
    }
    else
    {
      // Wait period - LED stays off
      digitalWrite(LED_GREEN_PIN, LOW);
    }

    // Ensure other LEDs are off
    digitalWrite(LED_YELLOW_PIN, LOW);
    return;
  }

  // Priority 3: YELLOW LED when GPS is not locked (and not in boot)
  if (!gpsLocked)
  {
    // YELLOW LED blinks once every 1.5 seconds when GPS is not locked
    if (yellowBlinkStart == 0)
    {
      yellowBlinkStart = millis();
    }

    unsigned long yellowElapsed = millis() - yellowBlinkStart;

    if (yellowElapsed < YELLOW_BLINK_DURATION)
    {
      // ON for first half of duration (50ms ON, 50ms OFF)
      if (yellowElapsed < YELLOW_BLINK_DURATION / 2)
      {
        digitalWrite(LED_YELLOW_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_YELLOW_PIN, LOW);
      }
    }
    else if (yellowElapsed >= 1500)
    {
      // Start new blink cycle every 1.5 seconds
      yellowBlinkStart = millis();
    }
    else
    {
      // Wait period - LED stays off
      digitalWrite(LED_YELLOW_PIN, LOW);
    }

    // Ensure other LEDs are off
    digitalWrite(LED_GREEN_PIN, LOW);
    return;
  }

  // Priority 4: All LEDs OFF when GPS is locked but sensors are not OK
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void updateBuzzer()
{
  // Determine which pattern should be active
  BuzzerPattern targetPattern = BUZZER_OFF;
  unsigned long timeSinceStart = millis() - missionStartTime;

  if (currentState == BOOT)
  {
    targetPattern = BUZZER_BOOT;
  }
  else if (currentState == TEST_MODE && timeSinceStart > 5000 && timeSinceStart < 10000)
  {
    // Status indication: 5-10 seconds after boot, only in TEST_MODE
    targetPattern = BUZZER_STATUS;
  }
  else if (apogeeReached && currentAltitude < 50 &&
           (currentState == DESCENT || currentState == AEROBRAKE_RELEASE || currentState == IMPACT))
  {
    // Recovery mode - only after flight stages are complete and low altitude
    targetPattern = BUZZER_RECOVERY;
  }

  // If pattern changed, reset timing
  if (targetPattern != currentBuzzerPattern)
  {
    currentBuzzerPattern = targetPattern;
    buzzerPatternStart = millis();
    buzzerBeepCount = 0;
    buzzerTargetBeeps = 0;
    digitalWrite(BUZZER_PIN, LOW);
  }

  // Execute current pattern
  unsigned long now = millis();
  unsigned long patternElapsed = now - buzzerPatternStart;

  switch (currentBuzzerPattern)
  {
  case BUZZER_OFF:
    digitalWrite(BUZZER_PIN, LOW);
    break;

  case BUZZER_BOOT:
    // 3 short beeps at startup - simplified direct control
    if (buzzerTargetBeeps == 0)
    {
      buzzerTargetBeeps = 3;
    }

    // Direct beep control for boot - more reliable
    if (buzzerBeepCount < buzzerTargetBeeps)
    {
      unsigned long beepCycle = (BUZZER_BEEP_SHORT + BUZZER_GAP_SHORT) * buzzerBeepCount;

      if (patternElapsed >= beepCycle && patternElapsed < (beepCycle + BUZZER_BEEP_SHORT))
      {
        // Beep ON
        digitalWrite(BUZZER_PIN, HIGH);
      }
      else if (patternElapsed >= (beepCycle + BUZZER_BEEP_SHORT) && patternElapsed < (beepCycle + BUZZER_BEEP_SHORT + BUZZER_GAP_SHORT))
      {
        // Beep OFF (gap)
        digitalWrite(BUZZER_PIN, LOW);
      }

      // Move to next beep when cycle is complete
      if (patternElapsed >= (beepCycle + BUZZER_BEEP_SHORT + BUZZER_GAP_SHORT))
      {
        buzzerBeepCount++;
      }
    }
    else
    {
      // All beeps completed
      digitalWrite(BUZZER_PIN, LOW);
    }
    break;

  case BUZZER_STATUS:
    // Different number of beeps based on system status
    if (buzzerTargetBeeps == 0)
    {
      if (sensorsOK && gpsLocked)
      {
        // 1 beep - all systems OK
        buzzerTargetBeeps = 1;
      }
      else if (sensorsOK && !gpsLocked)
      {
        // 2 beeps - sensors OK, GPS not locked
        buzzerTargetBeeps = 2;
      }
      else
      {
        // 3 beeps - sensor issues
        buzzerTargetBeeps = 3;
      }
    }
    executeBuzzerBeeps(BUZZER_BEEP_SHORT, BUZZER_GAP_SHORT); // 150ms on, 200ms off
    break;

  case BUZZER_RECOVERY:
    // 1 long beep every 3 seconds for recovery
    if (patternElapsed < BUZZER_BEEP_LONG)
    {
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else if (patternElapsed >= BUZZER_RECOVERY_INTERVAL)
    {
      // Reset pattern every 3 seconds
      buzzerPatternStart = now;
    }
    else
    {
      digitalWrite(BUZZER_PIN, LOW);
    }
    break;
  }
}

void executeBuzzerBeeps(unsigned long beepDuration, unsigned long gapDuration)
{
  unsigned long now = millis();
  unsigned long patternElapsed = now - buzzerPatternStart;

  if (buzzerBeepCount < buzzerTargetBeeps)
  {
    unsigned long beepCycleTime = beepDuration + gapDuration;
    unsigned long currentBeepStart = buzzerBeepCount * beepCycleTime;

    if (patternElapsed >= currentBeepStart && patternElapsed < (currentBeepStart + beepDuration))
    {
      // Beep ON phase
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else if (patternElapsed >= (currentBeepStart + beepDuration) && patternElapsed < (currentBeepStart + beepCycleTime))
    {
      // Beep OFF phase
      digitalWrite(BUZZER_PIN, LOW);
    }

    // Check if current beep cycle is complete
    if (patternElapsed >= (currentBeepStart + beepCycleTime))
    {
      buzzerBeepCount++;
    }
  }
  else
  {
    // All beeps completed
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void triggerGreenBlink()
{
  greenBlinkStart = millis();
  // Green LED will blink for GREEN_BLINK_DURATION (100ms) when data operations occur
}

void triggerYellowBlink()
{
  yellowBlinkStart = millis();
  // Yellow LED will start blinking when GPS is not locked
}

// ==================== SYSTEM STATUS CHECK ====================

void checkSystemStatus()
{
  // Check if all critical sensors are working
  bool bmp280OK = !isnan(primaryData.temperature) && !isnan(primaryData.pressure);
  bool mpu6050OK = !isnan(primaryData.accelX) && !isnan(primaryData.gyroX);
  bool sdOK = SD.exists("/telemetry.csv");

  sensorsOK = (bmp280OK || secondaryData.dataValid) && mpu6050OK && sdOK;
}

// ==================== ERROR CHECKING FUNCTIONS ====================

void checkAllSubsystemErrors()
{
  checkMPU6050Status();
  checkBMP280Status();
  checkSDCardStatus();
  checkGNSSStatus();
  checkSerialStatus();
  checkCameraStatus();

  // Update Secondary subsystem errors from received data
  systemErrors.pidError = secondaryData.pidError;
  systemErrors.loraError = secondaryData.loraError;
  systemErrors.bmp390Error = secondaryData.bmp390Error;
}

String generateErrorCode()
{
  String errorCode = "";

  if (systemErrors.mpu6050Error)
    errorCode += "1";
  if (systemErrors.bmp280Error)
    errorCode += "2";
  if (systemErrors.sdCardError)
    errorCode += "3";
  if (systemErrors.gnssError)
    errorCode += "5";
  if (systemErrors.pidError)
    errorCode += "6";
  if (systemErrors.cameraError)
    errorCode += "7";
  if (systemErrors.serialError)
    errorCode += "8";
  if (systemErrors.loraError)
    errorCode += "9";
  if (systemErrors.bmp390Error)
    errorCode += "10";

  return (errorCode.length() > 0) ? errorCode : "0";
}

void checkMPU6050Status()
{
  // Check if MPU6050 data is valid
  systemErrors.mpu6050Error = (isnan(primaryData.accelX) || isnan(primaryData.gyroX) ||
                               isnan(primaryData.accelY) || isnan(primaryData.gyroY) ||
                               isnan(primaryData.accelZ) || isnan(primaryData.gyroZ));
}

void checkBMP280Status()
{
  // Check if BMP280 data is valid
  systemErrors.bmp280Error = (isnan(primaryData.temperature) || isnan(primaryData.pressure) ||
                              isnan(primaryData.altitude));
}

void checkSDCardStatus()
{
  // Check if SD card is accessible and telemetry file exists
  systemErrors.sdCardError = !SD.exists("/telemetry.csv");
}

void checkGNSSStatus()
{
  // Check if GPS has valid data and is not too old
  systemErrors.gnssError = (!gps.location.isValid() || gps.location.age() > 10000 ||
                            primaryData.gnssSats < 4);
}

void checkSerialStatus()
{
  // Check if communication with Secondary ESP32 is working
  static unsigned long lastValidData = 0;
  if (secondaryData.dataValid)
  {
    lastValidData = millis();
  }
  systemErrors.serialError = (millis() - lastValidData > 10000); // No valid data for 10 seconds
}

void checkCameraStatus()
{
  // Placeholder for camera status check - set to false for now since no camera module
  systemErrors.cameraError = false; // Set to true if camera initialization fails
}

// ==================== MAIN SETUP FUNCTION ====================

void setup()
{
  // Initialize WiFi off for power saving
  WiFi.mode(WIFI_OFF);

  // Record mission start time
  missionStartTime = millis();

  // Initialize subsystems
  initializePins();
  initializeUART();
  initializeServo();

  delay(1000); // Allow systems to settle

  bool i2cOK = initializeI2C();
  sdCardOK = initializeSD();

  // Calibrate BMP280 baseline if I2C is working
  if (i2cOK)
  {
    calibrateBaselineBMP280();
  }

  // Initial sensor readings
  if (i2cOK)
  {
    readMPU6050();
    readBMP280();
  }

  // System status check
  checkSystemStatus();

  Serial.println("PRIMARY ESP32 - CanSat Mission Control Initialized");
  Serial.print("I2C Sensors: ");
  Serial.println(i2cOK ? "OK" : "FAILED");
  Serial.print("SD Card: ");
  Serial.println(sdCardOK ? "OK" : "FAILED");
  Serial.print("System Status: ");
  Serial.println(sensorsOK ? "OK" : "ERROR");

  // Boot complete - transition from boot state will happen in loop
}

// ==================== MAIN LOOP FUNCTION ====================

void loop()
{
  unsigned long currentTime = millis();

  // Continuous GPS reading
  readGPS();

  // Request data from secondary ESP32 periodically
  if (currentTime - lastUARTRequest >= UART_INTERVAL)
  {
    requestSecondaryData();
    lastUARTRequest = currentTime;
  }

  // Main data collection and transmission cycle
  if (currentTime - lastDataCollection >= DATA_INTERVAL)
  {
    // Read all sensors
    readMPU6050();
    readBMP280();

    // Check system status
    checkSystemStatus();

    // Check all subsystem errors for ErrorCode generation
    checkAllSubsystemErrors();

    // Update flight state
    if (currentTime - lastStateUpdate >= STATE_UPDATE_INTERVAL)
    {
      updateFlightState();
      lastStateUpdate = currentTime;
    }

    // Create CSV row
    String csvRow = createCSVRow();

    // Trigger green LED blink at start of data operations
    triggerGreenBlink();

    // Write to SD card
    bool sdWriteSuccess = writeToSD(csvRow);

    // Send to secondary ESP32
    sendConsolidatedData(csvRow);

    // Increment packet counter
    packetCount++;

    lastDataCollection = currentTime;

    // Debug output
    Serial.println("Data packet " + String(packetCount) + " processed");
    Serial.println("State: " + String(currentState) + ", Alt: " + String(currentAltitude) + "m");
    Serial.println("LED Status - RED: " + String(currentState == BOOT ? "ON" : "OFF") +
                   ", YELLOW: " + String(!gpsLocked ? "ON" : "OFF") +
                   ", GREEN: " + String(sensorsOK ? "ON" : "OFF"));
    Serial.println("GPS Locked: " + String(gpsLocked) + ", Sensors OK: " + String(sensorsOK));
    Serial.println("LED Debug - State: " + String(currentState) + ", Boot Time: " + String(millis() - missionStartTime) + "ms");
    Serial.println("GPS Sats: " + String(primaryData.gnssSats) + ", GPS Age: " + String(gps.location.age()));
    Serial.println("Buzzer Pattern: " + String(currentBuzzerPattern) + ", Beep Count: " + String(buzzerBeepCount));
    Serial.println("---");
  }

  // Update LEDs and buzzer
  updateLEDs();
  updateBuzzer();

  // LED DEBUG OUTPUT (every 2 seconds)
  static unsigned long lastLEDDebug = 0;
  if (millis() - lastLEDDebug > 2000)
  {
    Serial.println("=== LED DEBUG ===");
    Serial.println("currentState: " + String(currentState) + " (BOOT=0, TEST_MODE=1, LAUNCH_PAD=2, ...)");
    Serial.println("sensorsOK: " + String(sensorsOK));
    Serial.println("gpsLocked: " + String(gpsLocked));
    Serial.println("missionTime: " + String(millis() - missionStartTime) + "ms");
    Serial.println("Expected LEDs - RED: " + String(currentState == BOOT ? "ON" : "OFF") +
                   ", YELLOW: " + String(!gpsLocked && currentState != BOOT ? "BLINK" : "OFF") +
                   ", GREEN: " + String(sensorsOK && gpsLocked && currentState != BOOT ? "BLINK" : "OFF"));
    Serial.println("=================");
    lastLEDDebug = millis();
  }

  // MINIMAL delay for maximum speed - OPTIMIZED
  delay(1);
}
