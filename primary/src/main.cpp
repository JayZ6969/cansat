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
 * - UART0: Communication with Secondary ESP32
 * - Battery ADC: GPIO36 (VP pin)
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

// ==================== PIN DEFINITIONS ====================
// LEDs
#define LED_RED_PIN     12
#define LED_YELLOW_PIN  13  
#define LED_GREEN_PIN   14

// Buzzer
#define BUZZER_PIN      0   // Boot pin

// SPI SD Card
#define SD_CS_PIN       5
#define SD_MOSI_PIN     23
#define SD_MISO_PIN     19
#define SD_SCK_PIN      18

// I2C
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// UART GPS
#define GPS_RX_PIN      16
#define GPS_TX_PIN      17

// ADC Battery
#define BATTERY_ADC_PIN 36

// ==================== GLOBAL OBJECTS ====================
HardwareSerial gpsSerial(2);       // UART2 for GPS
HardwareSerial secondarySerial(0); // UART0 for Secondary ESP32
TinyGPSPlus gps;
Adafruit_BMP280 bmp280;
Adafruit_MPU6050 mpu6050;

// ==================== GLOBAL VARIABLES ====================
// Mission data
const String TEAM_ID = "2024-ASI-CANSAT-049";
uint32_t packetCount = 0;
unsigned long missionStartTime = 0;

// Flight states
enum FlightState {
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
struct SensorData {
  float altitude = 0.0;
  float pressure = 0.0;
  float temperature = 0.0;
  float voltage = 0.0;
  String gnssTime = "";
  float gnssLat = 0.0;
  float gnssLong = 0.0;
  float gnssAlt = 0.0;
  int gnssSats = 0;
  float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
  float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
  float gyroSpinRate = 0.0;
  String optionalData = "";
} primaryData;

struct SecondaryData {
  float bmp390Altitude = 0.0;
  float bmp390Pressure = 0.0;
  float bmp390Temperature = 0.0;
  String servoStatus = "";
  String pidOutput = "";
  bool dataValid = false;
} secondaryData;

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
unsigned long buzzerBeaconStart = 0;

// Data collection intervals
const unsigned long DATA_INTERVAL = 1000;        // 1Hz data collection
const unsigned long UART_INTERVAL = 800;         // UART request every 800ms
const unsigned long STATE_UPDATE_INTERVAL = 500; // State check every 500ms
const unsigned long GREEN_BLINK_DURATION = 100;  // 100ms blink
const unsigned long BUZZER_BEACON_INTERVAL = 2000; // 2s beacon interval

// Flight logic variables
float maxAltitude = 0.0;
bool apogeeReached = false;
float currentAltitude = 0.0;
unsigned long ascentStartTime = 0;

// ==================== INITIALIZATION FUNCTIONS ====================

void initializePins() {
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
  digitalWrite(BUZZER_PIN, HIGH); // Buzzer mirrors RED during boot
  
  Serial.println("Pins initialized");
}

bool initializeI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400kHz
  
  // Initialize BMP280
  if (!bmp280.begin(0x76)) {
    if (!bmp280.begin(0x77)) {
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
  if (!mpu6050.begin()) {
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

bool initializeSD() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    return false;
  }
  
  // Create CSV header if file doesn't exist
  if (!SD.exists("/telemetry.csv")) {
    File file = SD.open("/telemetry.csv", FILE_WRITE);
    if (file) {
      file.println("TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GNSS_TIME,GNSS_LAT,GNSS_LONG,GNSS_ALT,GNSS_SATS,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,GYRO_SPIN_RATE,FLIGHT_STATE,OPTIONAL_DATA");
      file.close();
      Serial.println("CSV header created");
    } else {
      Serial.println("Failed to create CSV file");
      return false;
    }
  }
  
  Serial.println("SD Card initialized successfully");
  return true;
}

void initializeUART() {
  // GPS on UART2
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  
  // Secondary ESP32 on UART0 (default Serial)
  Serial.begin(115200);
  
  Serial.println("UART initialized");
}

// ==================== SENSOR READING FUNCTIONS ====================

void readGPS() {
  unsigned long start = millis();
  while (gpsSerial.available() && (millis() - start) < 100) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        primaryData.gnssLat = gps.location.lat();
        primaryData.gnssLong = gps.location.lng();
      }
      
      if (gps.altitude.isValid()) {
        primaryData.gnssAlt = gps.altitude.meters();
      }
      
      if (gps.satellites.isValid()) {
        primaryData.gnssSats = gps.satellites.value();
      }
      
      if (gps.time.isValid()) {
        char timeStr[20];
        sprintf(timeStr, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
        primaryData.gnssTime = String(timeStr);
      }
    }
  }
  
  // Update GPS lock status - locked if we have valid location data and satellites
  if (gps.location.isValid() && gps.location.age() < 5000 && primaryData.gnssSats >= 4) {
    gpsLocked = true;
  } else {
    gpsLocked = false;
  }
}

bool readMPU6050() {
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

bool readBMP280() {
  primaryData.temperature = bmp280.readTemperature();
  primaryData.pressure = bmp280.readPressure() / 100.0; // Convert to hPa
  primaryData.altitude = bmp280.readAltitude(1013.25); // Sea level pressure
  
  return (!isnan(primaryData.temperature) && !isnan(primaryData.pressure));
}

void readBattery() {
  int adcValue = analogRead(BATTERY_ADC_PIN);
  // Convert ADC to voltage (assuming 3.3V reference and voltage divider)
  primaryData.voltage = (adcValue / 4095.0) * 3.3 * 2.0; // Assuming 2:1 voltage divider
}

// ==================== UART COMMUNICATION ====================

bool requestSecondaryData() {
  // Send request to secondary ESP32
  Serial.println("REQ_DATA");
  
  unsigned long timeout = millis() + 500; // 500ms timeout
  String response = "";
  
  while (millis() < timeout) {
    if (Serial.available()) {
      response = Serial.readStringUntil('\n');
      response.trim();
      break;
    }
    delay(10);
  }
  
  if (response.length() == 0) {
    secondaryData.dataValid = false;
    return false;
  }
  
  // Parse secondary data: "ALT:123.45,PRESS:1013.25,TEMP:25.5,SERVO:OK,PID:0.75"
  if (response.startsWith("DATA:")) {
    response = response.substring(5); // Remove "DATA:" prefix
    
    int altIndex = response.indexOf("ALT:");
    int pressIndex = response.indexOf("PRESS:");
    int tempIndex = response.indexOf("TEMP:");
    int servoIndex = response.indexOf("SERVO:");
    int pidIndex = response.indexOf("PID:");
    
    if (altIndex >= 0 && pressIndex >= 0 && tempIndex >= 0) {
      secondaryData.bmp390Altitude = response.substring(altIndex + 4, response.indexOf(',', altIndex)).toFloat();
      secondaryData.bmp390Pressure = response.substring(pressIndex + 6, response.indexOf(',', pressIndex)).toFloat();
      secondaryData.bmp390Temperature = response.substring(tempIndex + 5, response.indexOf(',', tempIndex)).toFloat();
      
      if (servoIndex >= 0) {
        secondaryData.servoStatus = response.substring(servoIndex + 6, response.indexOf(',', servoIndex));
      }
      
      if (pidIndex >= 0) {
        secondaryData.pidOutput = response.substring(pidIndex + 4);
      }
      
      secondaryData.dataValid = true;
      return true;
    }
  }
  
  secondaryData.dataValid = false;
  return false;
}

void sendConsolidatedData(String csvRow) {
  Serial.println("CSV:" + csvRow);
}

// ==================== DATA MANAGEMENT ====================

String createCSVRow() {
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
  csvRow += String(primaryData.voltage, 2) + ",";
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
  
  // Optional data includes secondary status
  String optional = "";
  if (secondaryData.dataValid) {
    optional += "SERVO:" + secondaryData.servoStatus + ";PID:" + secondaryData.pidOutput;
  }
  csvRow += optional;
  
  return csvRow;
}

bool writeToSD(String csvRow) {
  File file = SD.open("/telemetry.csv", FILE_APPEND);
  if (file) {
    file.println(csvRow);
    file.close();
    return true;
  }
  return false;
}

// ==================== FLIGHT STATE MANAGEMENT ====================

void updateFlightState() {
  static unsigned long bootTime = millis();
  
  // Update current altitude for state decisions
  currentAltitude = secondaryData.dataValid ? secondaryData.bmp390Altitude : primaryData.altitude;
  
  switch (currentState) {
    case BOOT:
      if (millis() - bootTime > 5000) { // 5 seconds boot time
        currentState = TEST_MODE;
      }
      break;
      
    case TEST_MODE:
      // Transition to LAUNCH_PAD when sensors are stable and GPS is locked
      if (sensorsOK && gpsLocked && currentAltitude < 100) {
        currentState = LAUNCH_PAD;
      }
      break;
      
    case LAUNCH_PAD:
      // Detect launch by significant altitude increase
      if (currentAltitude > 50) {
        currentState = ASCENT;
        ascentStartTime = millis();
      }
      break;
      
    case ASCENT:
      // Track maximum altitude
      if (currentAltitude > maxAltitude) {
        maxAltitude = currentAltitude;
      }
      
      // Detect apogee (altitude decrease after significant ascent)
      if (currentAltitude < (maxAltitude - 10) && maxAltitude > 100) {
        currentState = ROCKET_DEPLOY;
        apogeeReached = true;
      }
      break;
      
    case ROCKET_DEPLOY:
      // Transition to descent after a brief period
      if (millis() - ascentStartTime > 2000) {
        currentState = DESCENT;
      }
      break;
      
    case DESCENT:
      // Detect aerobrake release (could be based on altitude or time)
      if (currentAltitude < (maxAltitude * 0.5)) {
        currentState = AEROBRAKE_RELEASE;
      }
      break;
      
    case AEROBRAKE_RELEASE:
      // Detect impact (low altitude + low vertical velocity)
      if (currentAltitude < 10 && abs(primaryData.accelZ) > 2.0) {
        currentState = IMPACT;
      }
      break;
      
    case IMPACT:
      // Stay in impact state
      break;
  }
}

// ==================== LED AND BUZZER CONTROL ====================

void updateLEDs() {
  // RED LED (D12) - ON during BOOT state only
  digitalWrite(LED_RED_PIN, (currentState == BOOT) ? HIGH : LOW);
  
  // YELLOW LED (D13) - ON while GPS not locked
  digitalWrite(LED_YELLOW_PIN, !gpsLocked ? HIGH : LOW);
  
  // GREEN LED (D14) - ON when all sensors OK, BLINK during data operations
  if (greenBlinkStart > 0 && (millis() - greenBlinkStart) < GREEN_BLINK_DURATION) {
    // Blinking during data operations (50ms ON, 50ms OFF pattern)
    unsigned long blinkElapsed = millis() - greenBlinkStart;
    bool blinkState = (blinkElapsed / 50) % 2 == 0; // Toggle every 50ms
    digitalWrite(LED_GREEN_PIN, blinkState ? HIGH : LOW);
  } else {
    // Steady state - ON when all sensors are OK
    greenBlinkStart = 0;
    digitalWrite(LED_GREEN_PIN, sensorsOK ? HIGH : LOW);
  }
}

void updateBuzzer() {
  bool buzzerShouldBeOn = false;
  
  // Mirror LED states
  if (currentState == BOOT) {
    buzzerShouldBeOn = true;
  } else if (!gpsLocked) {
    buzzerShouldBeOn = true;
  } else if (sensorsOK) {
    // Beacon pattern when below 50m after apogee
    if (apogeeReached && currentAltitude < 50) {
      unsigned long now = millis();
      if (buzzerBeaconStart == 0) {
        buzzerBeaconStart = now;
      }
      
      unsigned long elapsed = now - buzzerBeaconStart;
      if (elapsed < 500) { // 500ms on
        buzzerShouldBeOn = true;
      } else if (elapsed < BUZZER_BEACON_INTERVAL) { // 1500ms off
        buzzerShouldBeOn = false;
      } else {
        buzzerBeaconStart = now; // Reset cycle
      }
    }
  }
  
  digitalWrite(BUZZER_PIN, buzzerShouldBeOn ? HIGH : LOW);
}

void triggerGreenBlink() {
  greenBlinkStart = millis();
  // Green LED will blink for GREEN_BLINK_DURATION (100ms) when data operations occur
}

// ==================== SYSTEM STATUS CHECK ====================

void checkSystemStatus() {
  // Check if all critical sensors are working
  bool bmp280OK = !isnan(primaryData.temperature) && !isnan(primaryData.pressure);
  bool mpu6050OK = !isnan(primaryData.accelX) && !isnan(primaryData.gyroX);
  bool sdOK = SD.exists("/telemetry.csv");
  
  sensorsOK = (bmp280OK || secondaryData.dataValid) && mpu6050OK && sdOK;
}

// ==================== MAIN SETUP FUNCTION ====================

void setup() {
  // Initialize WiFi off for power saving
  WiFi.mode(WIFI_OFF);
  
  // Record mission start time
  missionStartTime = millis();
  
  // Initialize subsystems
  initializePins();
  initializeUART();
  
  delay(1000); // Allow systems to settle
  
  bool i2cOK = initializeI2C();
  sdCardOK = initializeSD();
  
  // Initial sensor readings
  readBattery();
  if (i2cOK) {
    readMPU6050();
    readBMP280();
  }
  
  // System status check
  checkSystemStatus();
  
  Serial.println("PRIMARY ESP32 - CanSat Mission Control Initialized");
  Serial.print("I2C Sensors: "); Serial.println(i2cOK ? "OK" : "FAILED");
  Serial.print("SD Card: "); Serial.println(sdCardOK ? "OK" : "FAILED");
  Serial.print("System Status: "); Serial.println(sensorsOK ? "OK" : "ERROR");
  
  // Boot complete - transition from boot state will happen in loop
}

// ==================== MAIN LOOP FUNCTION ====================

void loop() {
  unsigned long currentTime = millis();
  
  // Continuous GPS reading
  readGPS();
  
  // Request data from secondary ESP32 periodically
  if (currentTime - lastUARTRequest >= UART_INTERVAL) {
    requestSecondaryData();
    lastUARTRequest = currentTime;
  }
  
  // Main data collection and transmission cycle
  if (currentTime - lastDataCollection >= DATA_INTERVAL) {
    // Read all sensors
    readBattery();
    readMPU6050();
    readBMP280();
    
    // Check system status
    checkSystemStatus();
    
    // Update flight state
    if (currentTime - lastStateUpdate >= STATE_UPDATE_INTERVAL) {
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
    Serial.println("GPS Sats: " + String(primaryData.gnssSats) + ", GPS Age: " + String(gps.location.age()));
    Serial.println("---");
  }
  
  // Update LEDs and buzzer
  updateLEDs();
  updateBuzzer();
  
  // Small delay to prevent watchdog issues
  delay(10);
}
