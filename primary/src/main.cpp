/*
 * PRIMARY ESP32 - CanSat Mission Control
 * Board: ESP32 Devkit (30 pins)
 * Role: Main telemetry collection, GPS tracking, SD logging, UART coordination
 *
 * Hardware Configuration:
 * - GPS L89: UART2 (GPIO16-RX, GPIO17-TX)
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
  float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
  float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
  float gyroSpinRate = 0.0;
  float speed = 0.0;              // Speed calculated from MPU6050 acceleration
  float lastAccelMagnitude = 0.0; // For speed calculation
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
bool bmp390Ready = false;        // Track if BMP390 from secondary is available
bool usingBMP390 = false;        // Track which sensor we're using for altitude
float lastKnownAltitude = 0.0;   // Last known good altitude for fallback calibration

// Timing variables
unsigned long lastDataCollection = 0;
unsigned long lastGreenBlink = 0;
unsigned long lastBuzzerUpdate = 0;
unsigned long lastUARTRequest = 0;
unsigned long lastStateUpdate = 0;
unsigned long lastDebugOutput = 0;

// NEW LED/BUZZER CONTROL VARIABLES
bool sensorsInitialized = false;
bool bootComplete = false;
bool bootBeepDone = false;
bool sdErrorBeepActive = false;
bool allOkBeepDone = false;
bool recoveryBeepActive = false;

unsigned long yellowBlinkTimer = 0;
unsigned long redBlinkTimer = 0;
unsigned long greenBlinkTimer = 0;
unsigned long buzzerTimer = 0;
unsigned long recoveryBeepTimer = 0;

bool yellowLedState = false;
bool redLedState = false;
bool greenLedState = false;
bool buzzerState = false;

int sdErrorBeepCount = 0;
int allOkBeepCount = 0;

// Data collection intervals - OPTIMIZED FOR MAXIMUM SPEED
const unsigned long DATA_INTERVAL = 100;         // 10Hz data collection (maximum speed)
const unsigned long UART_INTERVAL = 50;          // UART request every 50ms (fast sync)
const unsigned long STATE_UPDATE_INTERVAL = 100; // State check every 100ms
const unsigned long DEBUG_OUTPUT_INTERVAL = 5000; // Debug output every 5 seconds

// NEW LED/BUZZER TIMING CONSTANTS
const unsigned long LED_BLINK_INTERVAL = 500;    // 500ms blink cycle (on/off)
const unsigned long GREEN_BLINK_DURATION = 100;  // 100ms green blink when writing data
const unsigned long BUZZER_BEEP_SHORT = 200;     // 200ms short beep
const unsigned long BUZZER_BEEP_LONG = 1000;     // 1000ms long beep for recovery
const unsigned long BUZZER_GAP = 300;            // 300ms gap between beeps
const unsigned long RECOVERY_BEEP_INTERVAL = 2000; // 2s between recovery beeps

// Flight logic variables
float maxAltitude = 0.0;
bool apogeeReached = false;
float currentAltitude = 0.0;
unsigned long ascentStartTime = 0;
bool servoOpen = false; // Track servo position

// BMP280 baseline calibration
float baselineAltitudeBMP280 = 0.0;
bool bmp280Calibrated = false;    // Track if BMP280 has been calibrated

// PID Controller Variables (for logging/telemetry only, not for hardware control)
float pidSetpoint = 0.0;       // Target spin rate (0 = stable, no rotation)
float pidKp = 0.5;             // Proportional gain
float pidKi = 0.1;             // Integral gain
float pidKd = 0.2;             // Derivative gain
float pidIntegral = 0.0;       // Integral accumulator
float pidLastError = 0.0;      // Last error for derivative calculation
float pidOutput = 0.0;         // PID output value (for logging only)
unsigned long pidLastTime = 0; // Last PID calculation time

// ==================== INITIALIZATION FUNCTIONS ====================

void initializePins();
bool initializeI2C();
bool initializeSD();
void initializeUART();
void calibrateBaselineBMP280();
void calibrateBMP280ToLastAltitude();
float readLastAltitudeFromCSV();
void initializeServo();

// ==================== SENSOR READING FUNCTIONS ====================
void readGPS();
bool readMPU6050();
bool readBMP280();
float calculatePID(); // Calculate PID value from gyro spin rate (for logging only)

// ==================== UART COMMUNICATION ====================
bool requestSecondaryData();
void sendConsolidatedData(String csvRow);

// ==================== DATA MANAGEMENT ====================
String createCSVRow();
bool writeToSD(String csvRow);

// ==================== FLIGHT STATE MANAGEMENT ====================
void updateFlightState();
FlightState readLastFlightStateFromCSV();
const char* flightStateToString(FlightState state);
FlightState stringToFlightState(String stateStr);

// ==================== NEW LED AND BUZZER CONTROL ====================
void checkSensorInitialization();
void handleLEDs();
void handleBuzzer();
void triggerGreenBlink();

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

  // Initial state - RED on during boot, all others off
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_YELLOW_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("[INIT] GPIO pins configured");
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
      Serial.println("[ERROR] BMP280 init failed");
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
    Serial.println("[ERROR] MPU6050 init failed");
    return false;
  }

  // Configure MPU6050
  mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu6050.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("[INIT] I2C sensors initialized");
  return true;
}

bool initializeSD()
{
  // GPS INTERFERENCE MITIGATION: 
  // 1. Delay before SD init to let GPS stabilize first
  delay(2000);
  
  // 2. Lower SPI speed to reduce electromagnetic interference
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  SPI.setFrequency(400000); // 400kHz - much slower to reduce interference
  
  if (!SD.begin(SD_CS_PIN, SPI, 400000)) // Force low frequency
  {
    Serial.println("[ERROR] SD Card init failed");
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
    }
    else
    {
      Serial.println("[ERROR] CSV file creation failed");
      return false;
    }
  }

  Serial.println("[INIT] SD Card ready");
  return true;
}

void initializeUART()
{
  // GPS on UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Secondary ESP32 on UART0 (default Serial)
  Serial.begin(115200);

  Serial.println("[INIT] UART configured");
}

void calibrateBaselineBMP280()
{
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
    bmp280Calibrated = true;
    Serial.print("[CALIB] BMP280 baseline: ");
    Serial.print(baselineAltitudeBMP280);
    Serial.println(" m");
  }
  else
  {
    Serial.println("[ERROR] BMP280 calibration failed");
    baselineAltitudeBMP280 = 0.0;
    bmp280Calibrated = false;
  }
}

float readLastAltitudeFromCSV()
{
  if (!SD.exists("/telemetry.csv"))
  {
    return 0.0;
  }

  File file = SD.open("/telemetry.csv", FILE_READ);
  if (!file)
  {
    return 0.0;
  }

  String lastLine = "";
  
  // OPTIMIZED: Read from end of file backwards to find last valid line
  size_t fileSize = file.size();
  if (fileSize == 0)
  {
    file.close();
    return 0.0;
  }
  
  // Start reading from end, going backwards in chunks
  const int CHUNK_SIZE = 512;
  char buffer[CHUNK_SIZE + 1];
  int bytesToRead = min(CHUNK_SIZE, (int)fileSize);
  
  file.seek(fileSize - bytesToRead);
  int bytesRead = file.readBytes(buffer, bytesToRead);
  buffer[bytesRead] = '\0';
  
  String fileEnd = String(buffer);
  
  // Find last complete line (ignore incomplete last line if any)
  int lastNewline = fileEnd.lastIndexOf('\n');
  if (lastNewline > 0)
  {
    // Find second-to-last newline to get the complete last line
    int secondLastNewline = fileEnd.lastIndexOf('\n', lastNewline - 1);
    if (secondLastNewline >= 0)
    {
      lastLine = fileEnd.substring(secondLastNewline + 1, lastNewline);
    }
    else
    {
      lastLine = fileEnd.substring(0, lastNewline);
    }
  }
  
  file.close();
  lastLine.trim();

  if (lastLine.length() == 0)
  {
    return 0.0;
  }

  // Parse the altitude from CSV (4th field: TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,...)
  int commaCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  
  for (int i = 0; i < lastLine.length(); i++)
  {
    if (lastLine.charAt(i) == ',')
    {
      commaCount++;
      if (commaCount == 3)
      {
        startIndex = i + 1;
      }
      else if (commaCount == 4)
      {
        endIndex = i;
        break;
      }
    }
  }

  if (startIndex > 0 && endIndex > startIndex)
  {
    String altStr = lastLine.substring(startIndex, endIndex);
    float lastAlt = altStr.toFloat();
    Serial.print("[RECOVER] Last altitude: ");
    Serial.print(lastAlt);
    Serial.println(" m");
    return lastAlt;
  }

  return 0.0;
}

void calibrateBMP280ToLastAltitude()
{
  // Read last known altitude from CSV
  float lastAlt = readLastAltitudeFromCSV();
  
  if (lastAlt == 0.0)
  {
    Serial.println("[FALLBACK] Using standard BMP280 calibration");
    calibrateBaselineBMP280();
    return;
  }

  // Read current BMP280 altitude
  float currentBMP280Reading = bmp280.readAltitude(1013.25);
  
  if (isnan(currentBMP280Reading))
  {
    Serial.println("[ERROR] BMP280 read failed");
    bmp280Calibrated = false;
    return;
  }

  // Calculate baseline to make BMP280 match last known altitude
  baselineAltitudeBMP280 = currentBMP280Reading - lastAlt;
  bmp280Calibrated = true;
  
  Serial.print("[CALIB] BMP280 matched to ");
  Serial.print(lastAlt);
  Serial.println(" m");
}

// ==================== FLIGHT STATE RECOVERY FUNCTIONS ====================

const char* flightStateToString(FlightState state)
{
  switch (state)
  {
    case BOOT: return "BOOT";
    case TEST_MODE: return "TEST_MODE";
    case LAUNCH_PAD: return "LAUNCH_PAD";
    case ASCENT: return "ASCENT";
    case ROCKET_DEPLOY: return "ROCKET_DEPLOY";
    case DESCENT: return "DESCENT";
    case AEROBRAKE_RELEASE: return "AEROBRAKE_RELEASE";
    case IMPACT: return "IMPACT";
    default: return "UNKNOWN";
  }
}

FlightState stringToFlightState(String stateStr)
{
  stateStr.trim();
  
  // Try numeric format first (backward compatibility with old CSV format)
  int stateNum = stateStr.toInt();
  if (stateNum >= 0 && stateNum <= 7)
  {
    return (FlightState)stateNum;
  }
  
  // Try string format
  if (stateStr == "BOOT" || stateStr == "0") return BOOT;
  if (stateStr == "TEST_MODE" || stateStr == "1") return TEST_MODE;
  if (stateStr == "LAUNCH_PAD" || stateStr == "2") return LAUNCH_PAD;
  if (stateStr == "ASCENT" || stateStr == "3") return ASCENT;
  if (stateStr == "ROCKET_DEPLOY" || stateStr == "4") return ROCKET_DEPLOY;
  if (stateStr == "DESCENT" || stateStr == "5") return DESCENT;
  if (stateStr == "AEROBRAKE_RELEASE" || stateStr == "6") return AEROBRAKE_RELEASE;
  if (stateStr == "IMPACT" || stateStr == "7") return IMPACT;
  
  return BOOT; // Default to BOOT if unknown
}

FlightState readLastFlightStateFromCSV()
{
  if (!SD.exists("/telemetry.csv"))
  {
    return BOOT;
  }

  File file = SD.open("/telemetry.csv", FILE_READ);
  if (!file)
  {
    return BOOT;
  }

  String lastLine = "";
  
  // OPTIMIZED: Read from end of file backwards to find last valid line
  size_t fileSize = file.size();
  if (fileSize == 0)
  {
    file.close();
    return BOOT;
  }
  
  // Start reading from end, going backwards in chunks
  const int CHUNK_SIZE = 512;
  char buffer[CHUNK_SIZE + 1];
  int bytesToRead = min(CHUNK_SIZE, (int)fileSize);
  
  file.seek(fileSize - bytesToRead);
  int bytesRead = file.readBytes(buffer, bytesToRead);
  buffer[bytesRead] = '\0';
  
  String fileEnd = String(buffer);
  
  // Find last complete line (ignore incomplete last line if any)
  int lastNewline = fileEnd.lastIndexOf('\n');
  if (lastNewline > 0)
  {
    // Find second-to-last newline to get the complete last line
    int secondLastNewline = fileEnd.lastIndexOf('\n', lastNewline - 1);
    if (secondLastNewline >= 0)
    {
      lastLine = fileEnd.substring(secondLastNewline + 1, lastNewline);
    }
    else
    {
      lastLine = fileEnd.substring(0, lastNewline);
    }
  }
  
  file.close();
  lastLine.trim();

  if (lastLine.length() == 0)
  {
    return BOOT;
  }

  // Parse the flight state from CSV (20th field: TEAM_ID,TIMESTAMP,...,FLIGHT_STATE,...)
  int commaCount = 0;
  int startIndex = 0;
  int endIndex = 0;
  
  for (int i = 0; i < lastLine.length(); i++)
  {
    if (lastLine.charAt(i) == ',')
    {
      commaCount++;
      if (commaCount == 19) // After 19 commas, next field is FLIGHT_STATE (20th column)
      {
        startIndex = i + 1;
      }
      else if (commaCount == 20) // After 20 commas, we've passed FLIGHT_STATE
      {
        endIndex = i;
        break;
      }
    }
  }

  if (startIndex > 0 && endIndex > startIndex)
  {
    String stateStr = lastLine.substring(startIndex, endIndex);
    FlightState recoveredState = stringToFlightState(stateStr);
    
    Serial.print("[RECOVER] Last state: ");
    Serial.println(flightStateToString(recoveredState));
    
    return recoveredState;
  }

  return BOOT;
}

void initializeServo()
{
  lidServo.attach(SERVO_PIN);
  lidServo.write(90); // Start with parachute closed (90 degrees)
  servoOpen = false;

  Serial.println("[INIT] Servo ready");
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

      // GNSS speed reading removed - now using MPU6050 acceleration-based speed calculation
      // if (gps.speed.isValid())
      // {
      //   primaryData.gnssSpeed = gps.speed.mps(); // Speed in meters/second
      // }

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
  static unsigned long lastReadTime = 0;
  unsigned long currentTime = millis();

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

  // Calculate speed from acceleration magnitude (integration over time)
  // Get acceleration magnitude in m/s² (using raw values before g-force conversion)
  float accelMagnitude = sqrt(accel.acceleration.x * accel.acceleration.x +
                              accel.acceleration.y * accel.acceleration.y +
                              accel.acceleration.z * accel.acceleration.z);

  // Remove gravity component (9.81 m/s²)
  accelMagnitude = abs(accelMagnitude - 9.81);

  // Calculate time delta in seconds
  if (lastReadTime > 0)
  {
    float deltaTime = (currentTime - lastReadTime) / 1000.0; // Convert to seconds

    // Integrate acceleration to get velocity: v = v0 + a*dt
    primaryData.speed += accelMagnitude * deltaTime;

    // Apply damping factor to prevent drift (0.98 means 2% decay per reading)
    // This helps account for measurement errors and friction
    primaryData.speed *= 0.98;
  }

  lastReadTime = currentTime;
  primaryData.lastAccelMagnitude = accelMagnitude;

  return true;
}

bool readBMP280()
{
  primaryData.temperature = bmp280.readTemperature();
  primaryData.pressure = bmp280.readPressure() / 100.0;                         // Convert to hPa
  primaryData.altitude = bmp280.readAltitude(1013.25) - baselineAltitudeBMP280; // Zero-referenced altitude

  return (!isnan(primaryData.temperature) && !isnan(primaryData.pressure));
}

float calculatePID()
{
  // PID controller for reaction wheel stabilization (LOGGING ONLY - NOT USED FOR CONTROL)
  // Uses gyroSpinRate from MPU6050 to calculate control output
  // Target: maintain zero spin rate (stable orientation)

  unsigned long currentTime = millis();

  // Initialize on first run
  if (pidLastTime == 0)
  {
    pidLastTime = currentTime;
    pidLastError = 0.0;
    pidIntegral = 0.0;
    return 0.0;
  }

  // Calculate time delta in seconds
  float deltaTime = (currentTime - pidLastTime) / 1000.0;

  // Avoid division by zero or too-frequent updates
  if (deltaTime < 0.01) // Less than 10ms
  {
    return pidOutput; // Return last calculated value
  }

  // Calculate error (setpoint - current value)
  // Setpoint is 0 (we want no rotation for stable flight)
  float error = pidSetpoint - primaryData.gyroSpinRate;

  // Proportional term
  float P = pidKp * error;

  // Integral term (with anti-windup limiting)
  pidIntegral += error * deltaTime;
  // Limit integral to prevent windup
  if (pidIntegral > 100.0)
    pidIntegral = 100.0;
  if (pidIntegral < -100.0)
    pidIntegral = -100.0;
  float I = pidKi * pidIntegral;

  // Derivative term
  float derivative = (error - pidLastError) / deltaTime;
  float D = pidKd * derivative;

  // Calculate total PID output
  pidOutput = P + I + D;

  // Limit output range (typical PWM range for motor control would be -255 to +255)
  if (pidOutput > 255.0)
    pidOutput = 255.0;
  if (pidOutput < -255.0)
    pidOutput = -255.0;

  // Update for next iteration
  pidLastError = error;
  pidLastTime = currentTime;

  return pidOutput;
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
  csvRow += String(pidOutput, 2) + ","; // PID output (replaces gyroSpinRate for logging)
  csvRow += String(currentState) + ",";

  // Servo status: 0 = closed, 1 = open
  String servoStatus = servoOpen ? "1" : "0";

  csvRow += servoStatus + ",";

  // Add ErrorCode as 22nd column
  csvRow += generateErrorCode() + ",";

  // Add GNSS_SPEED as the last column (now calculated from MPU6050 acceleration)
  csvRow += String(primaryData.speed, 2);

  return csvRow;
}

bool writeToSD(String csvRow)
{
  // MINIMIZE GPS INTERFERENCE during SD writes:
  // 1. Set SPI to lowest possible speed before writing
  SPI.setFrequency(200000); // 200kHz for minimal interference
  
  File file = SD.open("/telemetry.csv", FILE_APPEND);
  if (file)
  {
    file.println(csvRow);
    file.flush(); // Force immediate write to reduce time with active SPI
    file.close();
    
    // 2. Small delay after SD write to let GPS recover
    delay(10);
    
    return true;
  }
  return false;
}

// ==================== FLIGHT STATE MANAGEMENT ====================

void updateFlightState()
{
  // Handle BMP390 failure detection and failover to BMP280
  if (usingBMP390 && secondaryData.dataValid)
  {
    // Check if BMP390 has failed
    if (secondaryData.bmp390Error != 0)
    {
      Serial.println("[FAILOVER] BMP390 failed - switching to BMP280");
      usingBMP390 = false;
      
      // Calibrate BMP280 to match last known altitude from BMP390
      if (bmp280Calibrated)
      {
        calibrateBMP280ToLastAltitude();
      }
    }
    else
    {
      // BMP390 is working - update last known altitude
      lastKnownAltitude = secondaryData.bmp390Altitude;
    }
  }
  
  // Update current altitude based on active sensor
  if (usingBMP390 && secondaryData.dataValid && secondaryData.bmp390Error == 0)
  {
    // Use BMP390 from Secondary (primary altitude sensor)
    currentAltitude = secondaryData.bmp390Altitude;
  }
  else
  {
    // Use BMP280 from Primary (fallback altitude sensor)
    currentAltitude = primaryData.altitude;
  }

  switch (currentState)
  {
  case BOOT:
    if (bootComplete)
    { // Boot complete when sensors are initialized
      currentState = TEST_MODE;
      Serial.println("[STATE] BOOT -> TEST_MODE");
    }
    break;

  case TEST_MODE:
    // Transition to LAUNCH_PAD when sensors are stable (GPS lock not required)
    // After power recovery, allow staying in TEST_MODE or moving forward
    if (sensorsOK && currentAltitude < 100)
    {
      currentState = LAUNCH_PAD;
    }
    break;

  case LAUNCH_PAD:
    // Detect launch by significant altitude increase
    // After power recovery, if already in LAUNCH_PAD, wait for actual launch
    if (currentAltitude > 50)
    {
      currentState = ASCENT;
      ascentStartTime = millis();
      Serial.println("[STATE] LAUNCH_PAD -> ASCENT");
    }
    break;

  case ASCENT:
    // Track maximum altitude
    // After power recovery during ASCENT, continue tracking altitude
    if (currentAltitude > maxAltitude)
    {
      maxAltitude = currentAltitude;
    }

    // Detect apogee (altitude decrease after significant ascent)
    if (currentAltitude < (maxAltitude - 100) && maxAltitude > 100)
    {
      currentState = ROCKET_DEPLOY;
      apogeeReached = true;
      Serial.println("[STATE] ASCENT -> ROCKET_DEPLOY");
    }
    break;

  case ROCKET_DEPLOY:
    // Deploy parachute immediately when entering this state
    if (!servoOpen)
    {
      lidServo.write(0); // Open parachute (0 degrees)
      servoOpen = true;
      Serial.println("[DEPLOY] Parachute deployed");
    }

    // Transition to descent after a brief period
    // After power recovery in ROCKET_DEPLOY, continue to DESCENT
    if (millis() - ascentStartTime > 2000)
    {
      currentState = DESCENT;
    }
    break;

  case DESCENT:
    // Detect aerobrake release (could be based on altitude or time)
    // After power recovery in DESCENT, continue monitoring for aerobrake release
    if (currentAltitude < (maxAltitude * 0.5))
    {
      currentState = AEROBRAKE_RELEASE;
    }
    break;

  case AEROBRAKE_RELEASE:
    // Detect impact (low altitude + low vertical velocity)
    // After power recovery in AEROBRAKE_RELEASE, continue monitoring for impact
    if (currentAltitude < 10 && abs(primaryData.accelZ) > 2.0)
    {
      currentState = IMPACT;
      Serial.println("[STATE] AEROBRAKE_RELEASE -> IMPACT");
    }
    break;

  case IMPACT:
    // Stay in impact state - recovery beeper will activate
    // After power recovery in IMPACT, beeper will reactivate in setup()
    break;
  }
}

// ==================== NEW LED AND BUZZER CONTROL ====================

void checkSensorInitialization()
{
  // Check if sensors are initialized (exclude GPS and SD card as per requirements)
  bool mpu6050OK = !isnan(primaryData.accelX) && !isnan(primaryData.gyroX);
  bool bmp280OK = !isnan(primaryData.temperature) && !isnan(primaryData.pressure);
  
  sensorsInitialized = mpu6050OK && bmp280OK;
  
  if (sensorsInitialized && !bootComplete)
  {
    bootComplete = true;
    Serial.println("[BOOT] Sensors ready");
    digitalWrite(LED_RED_PIN, LOW); // Turn off red LED
  }
}

void handleLEDs()
{
  unsigned long currentTime = millis();
  
  // 1. RED LED: ON during boot, BLINKING if SD card error after boot
  if (!bootComplete)
  {
    // Keep RED LED on during boot
    digitalWrite(LED_RED_PIN, HIGH);
  }
  else if (!sdCardOK)
  {
    // Blink RED LED if SD card not OK
    if (currentTime - redBlinkTimer >= LED_BLINK_INTERVAL)
    {
      redLedState = !redLedState;
      digitalWrite(LED_RED_PIN, redLedState);
      redBlinkTimer = currentTime;
    }
  }
  else
  {
    // Turn off RED LED if SD card is OK
    digitalWrite(LED_RED_PIN, LOW);
  }
  
  // 2. YELLOW LED: BLINKING when GPS not locked (after boot)
  if (bootComplete && !gpsLocked)
  {
    if (currentTime - yellowBlinkTimer >= LED_BLINK_INTERVAL)
    {
      yellowLedState = !yellowLedState;
      digitalWrite(LED_YELLOW_PIN, yellowLedState);
      yellowBlinkTimer = currentTime;
    }
  }
  else
  {
    digitalWrite(LED_YELLOW_PIN, LOW);
  }
  
  // 3. GREEN LED: Quick blink when everything is OK (GPS locked + SD card OK)
  // Pattern: 25ms ON, 75ms OFF, repeating every 1 second (matches Secondary D2)
  if (bootComplete && gpsLocked && sdCardOK)
  {
    if (greenBlinkTimer == 0)
    {
      greenBlinkTimer = currentTime;
    }

    unsigned long greenElapsed = currentTime - greenBlinkTimer;

    if (greenElapsed < GREEN_BLINK_DURATION)
    {
      // Quick blink - ON for 25ms, OFF for 75ms within the 100ms blink period
      if (greenElapsed < 25)
      {
        digitalWrite(LED_GREEN_PIN, HIGH);
      }
      else
      {
        digitalWrite(LED_GREEN_PIN, LOW);
      }
    }
    else if (greenElapsed >= 1000) // Repeat every 1 second
    {
      // Start new blink cycle
      greenBlinkTimer = currentTime;
    }
    else
    {
      // Wait period - LED stays off
      digitalWrite(LED_GREEN_PIN, LOW);
    }
  }
  else
  {
    digitalWrite(LED_GREEN_PIN, LOW);
    greenBlinkTimer = 0; // Reset timer when conditions not met
  }
}

void handleBuzzer()
{
  unsigned long currentTime = millis();
  static unsigned long buzzerSequenceStart = 0;
  static int beepPhase = 0;
  
  // 1. Boot complete beep (single beep when sensors initialized)
  if (bootComplete && !bootBeepDone)
  {
    if (buzzerSequenceStart == 0)
    {
      buzzerSequenceStart = currentTime;
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else if (currentTime - buzzerSequenceStart >= BUZZER_BEEP_SHORT)
    {
      digitalWrite(BUZZER_PIN, LOW);
      bootBeepDone = true;
      buzzerSequenceStart = 0;
    }
    return;
  }
  
  // 2. SD Card error beeps (double beep pattern)
  if (bootComplete && !sdCardOK && !sdErrorBeepActive)
  {
    if (buzzerSequenceStart == 0)
    {
      buzzerSequenceStart = currentTime;
      sdErrorBeepCount = 0;
      beepPhase = 0;
      sdErrorBeepActive = true;
    }
  }
  
  if (sdErrorBeepActive)
  {
    unsigned long elapsed = currentTime - buzzerSequenceStart;
    
    if (sdErrorBeepCount < 2) // Two beeps for SD error
    {
      if (beepPhase == 0 && elapsed < BUZZER_BEEP_SHORT)
      {
        digitalWrite(BUZZER_PIN, HIGH);
      }
      else if (beepPhase == 0 && elapsed >= BUZZER_BEEP_SHORT)
      {
        digitalWrite(BUZZER_PIN, LOW);
        beepPhase = 1;
      }
      else if (beepPhase == 1 && elapsed >= (BUZZER_BEEP_SHORT + BUZZER_GAP))
      {
        sdErrorBeepCount++;
        beepPhase = 0;
        buzzerSequenceStart = currentTime;
      }
    }
    else if (elapsed >= 2000) // Repeat every 2 seconds
    {
      sdErrorBeepCount = 0;
      beepPhase = 0;
      buzzerSequenceStart = currentTime;
    }
    
    if (sdCardOK)
    {
      sdErrorBeepActive = false;
      digitalWrite(BUZZER_PIN, LOW);
      buzzerSequenceStart = 0;
    }
    return;
  }
  
  // 3. All OK beeps (triple beep when everything is OK)
  if (bootComplete && gpsLocked && sdCardOK)
  {
    if (!allOkBeepDone)
    {
      if (buzzerSequenceStart == 0)
      {
        buzzerSequenceStart = currentTime;
        allOkBeepCount = 0;
        beepPhase = 0;
      }
      
      unsigned long elapsed = currentTime - buzzerSequenceStart;
      
      if (allOkBeepCount < 3) // Three beeps for all OK
      {
        if (beepPhase == 0 && elapsed < BUZZER_BEEP_SHORT)
        {
          digitalWrite(BUZZER_PIN, HIGH);
        }
        else if (beepPhase == 0 && elapsed >= BUZZER_BEEP_SHORT)
        {
          digitalWrite(BUZZER_PIN, LOW);
          beepPhase = 1;
        }
        else if (beepPhase == 1 && elapsed >= (BUZZER_BEEP_SHORT + BUZZER_GAP))
        {
          allOkBeepCount++;
          beepPhase = 0;
          buzzerSequenceStart = currentTime;
        }
      }
      else
      {
        allOkBeepDone = true;
        buzzerSequenceStart = 0;
      }
      return;
    }
  }
  else
  {
    // Reset the all OK beep flag if conditions are no longer met
    allOkBeepDone = false;
  }
  
  // 4. Recovery beeps (continuous long beeps after landing)
  if (currentState == IMPACT && !recoveryBeepActive)
  {
    recoveryBeepActive = true;
    recoveryBeepTimer = currentTime;
    Serial.println("[RECOVERY] Beeper activated");
  }
  
  if (recoveryBeepActive)
  {
    unsigned long elapsed = currentTime - recoveryBeepTimer;
    
    if (elapsed < BUZZER_BEEP_LONG)
    {
      digitalWrite(BUZZER_PIN, HIGH);
    }
    else if (elapsed < (BUZZER_BEEP_LONG + BUZZER_GAP))
    {
      digitalWrite(BUZZER_PIN, LOW);
    }
    else
    {
      recoveryBeepTimer = currentTime; // Reset for next beep
    }
    return;
  }
  
  // Default: buzzer off
  if (!sdErrorBeepActive && !recoveryBeepActive && bootBeepDone && allOkBeepDone)
  {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void triggerGreenBlink()
{
  greenBlinkTimer = millis();
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

  // GPS INTERFERENCE MITIGATION STRATEGY:
  // Initialize GPS-friendly subsystems first, SD card last
  
  initializePins();
  delay(500); // Let power stabilize
  
  // 1. Initialize UART (including GPS) first - highest priority
  initializeUART();
  Serial.println("\n[PRIMARY] CanSat Mission Control");
  Serial.println("========================================");
  
  // 2. Give GPS time to start receiving data before other subsystems
  unsigned long gpsStartTime = millis();
  while (millis() - gpsStartTime < 3000) {
    readGPS(); // Start collecting GPS data immediately
    delay(100);
  }
  
  // 3. Initialize I2C sensors (lower interference)
  bool i2cOK = initializeI2C();
  
  // 3.5. Initialize servo (low interference)
  initializeServo();
  
  // 4. Initialize SD card LAST (highest interference potential)
  sdCardOK = initializeSD();
  
  // 5. Give GPS time to re-acquire after SD interference
  delay(2000);

  // Wait for Secondary ESP32 BMP390 to initialize (primary altitude sensor)
  Serial.println("[INIT] Waiting for BMP390...");
  unsigned long bmp390WaitStart = millis();
  const unsigned long BMP390_WAIT_TIMEOUT = 10000; // 10 second timeout
  
  while (!bmp390Ready && (millis() - bmp390WaitStart < BMP390_WAIT_TIMEOUT))
  {
    // Request data from Secondary to check BMP390 status
    requestSecondaryData();
    delay(500);
    
    // Check if BMP390 is operational (no error and valid data)
    if (secondaryData.bmp390Error == 0 && secondaryData.bmp390Altitude != 0.0)
    {
      bmp390Ready = true;
      usingBMP390 = true;
      lastKnownAltitude = secondaryData.bmp390Altitude;
      Serial.print("[INIT] BMP390 ready (");
      Serial.print(lastKnownAltitude);
      Serial.println(" m)");
      break;
    }
  }
  
  if (!bmp390Ready)
  {
    Serial.println("[WARN] BMP390 timeout - using BMP280");
  }

  // Calibrate BMP280 baseline if I2C is working
  if (i2cOK)
  {
    if (!bmp390Ready)
    {
      // BMP390 not available - use standard BMP280 calibration
      calibrateBaselineBMP280();
      usingBMP390 = false;
    }
    else
    {
      // BMP390 is available - calibrate BMP280 as backup to match BMP390
      float currentBMP280Reading = bmp280.readAltitude(1013.25);
      if (!isnan(currentBMP280Reading))
      {
        baselineAltitudeBMP280 = currentBMP280Reading - lastKnownAltitude;
        bmp280Calibrated = true;
      }
    }
  }

  // Initial sensor readings
  if (i2cOK)
  {
    readMPU6050();
    readBMP280();
  }

  // System status check
  checkSystemStatus();

  Serial.println("========================================");
  Serial.print("[STATUS] I2C: ");
  Serial.print(i2cOK ? "OK" : "FAIL");
  Serial.print(" | SD: ");
  Serial.print(sdCardOK ? "OK" : "FAIL");
  Serial.print(" | System: ");
  Serial.println(sensorsOK ? "OK" : "ERROR");

  // ========== FLIGHT STATE RECOVERY (POWER FAILURE HANDLING) ==========
  // Check if this is a power reset during flight by reading last state from CSV
  if (sdCardOK)
  {
    FlightState recoveredState = readLastFlightStateFromCSV();
    
    // Only recover state if it's beyond BOOT (indicates we were in active flight)
    if (recoveredState != BOOT)
    {
      Serial.println("========================================");
      Serial.println("[POWER RESET DETECTED]");
      Serial.print("[RECOVER] Resuming from: ");
      Serial.println(flightStateToString(recoveredState));
      Serial.println("========================================");
      
      currentState = recoveredState;
      bootComplete = true; // Skip boot state
      sensorsInitialized = true; // Sensors already initialized
      
      // Special handling for IMPACT state (already landed)
      if (currentState == IMPACT)
      {
        recoveryBeepActive = true;
      }
      
      // Log the recovery event to CSV
      String recoveryLog = TEAM_ID + ",";
      recoveryLog += String(millis()) + ",";
      recoveryLog += String(packetCount) + ",";
      recoveryLog += "POWER_RESET,,,,,,,,,,,,,,,,,,,";
      writeToSD(recoveryLog);
    }
  }

  Serial.println("[READY] Mission control initialized\n");
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

    // Calculate PID output from gyro data (for logging/telemetry only)
    calculatePID();

    // Check system status
    checkSystemStatus();

    // Check all subsystem errors for ErrorCode generation
    checkAllSubsystemErrors();

    // Update flight state (includes altitude sensor failover handling)
    if (currentTime - lastStateUpdate >= STATE_UPDATE_INTERVAL)
    {
      updateFlightState();
      lastStateUpdate = currentTime;
    }

    // Create CSV row
    String csvRow = createCSVRow();

    // Write to SD card
    bool sdWriteSuccess = writeToSD(csvRow);

    // Send to secondary ESP32
    sendConsolidatedData(csvRow);

    // Increment packet counter
    packetCount++;

    lastDataCollection = currentTime;
  }

  // Check sensor initialization status
  checkSensorInitialization();
  
  // Update LEDs and buzzer with new logic
  handleLEDs();
  handleBuzzer();

  // CONSOLIDATED DEBUG OUTPUT (every 5 seconds)
  if (currentTime - lastDebugOutput >= DEBUG_OUTPUT_INTERVAL)
  {
    Serial.println("\n========== DEBUG REPORT ==========");
    Serial.print("[PKT] #");
    Serial.print(packetCount);
    Serial.print(" | State: ");
    Serial.print(flightStateToString(currentState));
    Serial.print(" | Alt: ");
    Serial.print(currentAltitude, 1);
    Serial.println(" m");
    
    Serial.print("[SENSOR] ");
    Serial.print(usingBMP390 ? "BMP390" : "BMP280");
    Serial.print(" | MPU: ");
    Serial.print(!isnan(primaryData.accelX) ? "OK" : "FAIL");
    Serial.print(" | Init: ");
    Serial.println(sensorsInitialized ? "YES" : "NO");
    
    Serial.print("[GPS] Lock: ");
    Serial.print(gpsLocked ? "YES" : "NO");
    Serial.print(" | Sats: ");
    Serial.print(primaryData.gnssSats);
    Serial.print(" | Lat: ");
    Serial.print(primaryData.gnssLat, 6);
    Serial.print(" | Lon: ");
    Serial.println(primaryData.gnssLong, 6);
    
    Serial.print("[SYSTEM] Boot: ");
    Serial.print(bootComplete ? "YES" : "NO");
    Serial.print(" | SD: ");
    Serial.print(sdCardOK ? "OK" : "FAIL");
    Serial.print(" | Err: ");
    Serial.println(generateErrorCode());
    
    Serial.print("[LED] R:");
    Serial.print(!bootComplete || !sdCardOK ? "ON" : "OFF");
    Serial.print(" Y:");
    Serial.print(bootComplete && !gpsLocked ? "BLINK" : "OFF");
    Serial.print(" G:");
    Serial.println(bootComplete && gpsLocked && sdCardOK ? "BLINK" : "OFF");
    
    Serial.println("==================================\n");
    lastDebugOutput = currentTime;
  }

  // MINIMAL delay for maximum speed - OPTIMIZED
  delay(1);
}
