/*
 * SECONDARY ESP32 - CanSat Mission Controller
 * Board: ESP32 Devkit (30 pins)
 * Role: Local sensor data collection + LoRa transmission of Primary's CSV data
 * 
 * Hardware:
 * - LoRa SX1278 (SPI)
 * - BMP390 sensor (I2C)
 * - Servo for lid control
 * - LEDs: D2 (init/rx indicator), D13 (LoRa/error indicator)
 * - UART2 communication with Primary ESP32
 * 
 * Data Flow:
 * 1. Read local sensors (BMP390, servo status, PID output)
 * 2. Send raw sensor data to Primary via UART2
 * 3. Wait for consolidated CSV telemetry from Primary
 * 4. Transmit CSV over LoRa SX1278
 * 5. Handle errors with LED indicators
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <LoRa.h>
#include <ESP32Servo.h>

// LoRa SX1278 Pin Definitions (SPI)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 26
#define LORA_BAND 433E6

// Servo Pin
#define SERVO_PIN 25

// LED Pin Definitions
#define LED_D2 2    // ON during init, BLINK when receiving from Primary
#define LED_D13 13  // ON if LoRa fails, BLINK if no Primary data

// I2C BMP390 Address
#define BMP390_I2C_ADDR 0x77

// UART2 Pins for Primary Communication
#define UART2_RX 16
#define UART2_TX 17

// Timing Constants
#define SENSOR_READ_INTERVAL 500    // Read sensors every 500ms
#define PRIMARY_DATA_TIMEOUT 5000   // Consider Primary lost after 5 seconds
#define LORA_TX_TIMEOUT 2000        // LoRa transmission timeout
#define LED_BLINK_INTERVAL 500      // LED blink period

// Global Objects
Adafruit_BMP3XX bmp390;
Servo lidServo;
HardwareSerial primarySerial(2); // UART2 for primary communication

// System State Variables
bool sensorsInitialized = false;
bool loraInitialized = false;
unsigned long bootTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastPrimaryData = 0;

// Local Sensor Data
float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;
float baselineAltitude = 0.0;
bool lidOpen = false;
float pidOutput = 0.0; // Dummy PID output or implement actual control

// Communication State
bool primaryDataReceived = false;
bool loraTransmissionFailed = false;

// LED Blinking State (Non-blocking)
unsigned long lastLedBlink = 0;
bool ledBlinkState = false;

// Function Declarations
void initializeSensors();
void initializeLoRa();
void initializeServo();
void readLocalSensors();
void sendDataToPrimary();
void receiveFromPrimary();
void transmitViaLoRa(String csvData);
void updateLEDs();
void calibrateBaseline();

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    // Initialize UART2 for Primary communication
    primarySerial.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
    
    // Initialize LED pins
    pinMode(LED_D2, OUTPUT);
    pinMode(LED_D13, OUTPUT);
    
    // Turn ON D2 during initialization
    digitalWrite(LED_D2, HIGH);
    digitalWrite(LED_D13, LOW);
    
    bootTime = millis();
    lastPrimaryData = millis();
    
    Serial.println("=== SECONDARY ESP32 INITIALIZING ===");
    
    // Initialize all systems
    initializeSensors();
    initializeLoRa();
    initializeServo();
    calibrateBaseline();
    
    // Initialization complete - turn OFF D2
    digitalWrite(LED_D2, LOW);
    
    Serial.println("=== SECONDARY ESP32 READY ===");
}

void loop() {
    // Always check for commands from Primary first (highest priority)
    receiveFromPrimary();
    
    // Read local sensors periodically (reduced frequency since REQ_DATA provides on-demand updates)
    if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL * 2) { // 1 second instead of 500ms
        readLocalSensors();
        // Don't auto-send data - let Primary request it via REQ_DATA when needed
        lastSensorRead = millis();
    }
    
    // Update LED status indicators
    updateLEDs();
    
    delay(50); // Small delay to prevent excessive CPU usage
}

void initializeSensors() {
    Serial.print("Initializing BMP390...");
    
    if (bmp390.begin_I2C(BMP390_I2C_ADDR)) {
        // Configure BMP390 for optimal performance
        bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp390.setOutputDataRate(BMP3_ODR_50_HZ);
        
        sensorsInitialized = true;
        Serial.println(" SUCCESS");
    } else {
        sensorsInitialized = false;
        Serial.println(" FAILED");
        // Continue operation even if sensor fails
    }
}

void initializeLoRa() {
    Serial.print("Initializing LoRa SX1278...");
    
    // Initialize SPI for LoRa
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    
    if (LoRa.begin(LORA_BAND)) {
        // Configure LoRa parameters
        LoRa.setSpreadingFactor(7);        // SF7 for faster transmission
        LoRa.setSignalBandwidth(125E3);    // 125 kHz bandwidth
        LoRa.setCodingRate4(5);            // 4/5 coding rate
        LoRa.setPreambleLength(8);         // 8 symbol preamble
        LoRa.setSyncWord(0x12);            // Sync word
        LoRa.enableCrc();                  // Enable CRC
        
        loraInitialized = true;
        Serial.println(" SUCCESS");
    } else {
        loraInitialized = false;
        Serial.println(" FAILED");
    }
}

void initializeServo() {
    Serial.print("Initializing lid servo...");
    
    lidServo.attach(SERVO_PIN);
    lidServo.write(0); // Start with lid closed
    lidOpen = false;
    
    Serial.println(" SUCCESS");
}

void calibrateBaseline() {
    if (!sensorsInitialized) {
        baselineAltitude = 0.0;
        return;
    }
    
    Serial.print("Calibrating baseline altitude...");
    
    float altSum = 0.0;
    int validReadings = 0;
    
    // Take 50 readings over 2.5 seconds
    for (int i = 0; i < 50; i++) {
        if (bmp390.performReading()) {
            altSum += bmp390.readAltitude(1013.25); // Standard sea level pressure
            validReadings++;
        }
        delay(50);
    }
    
    if (validReadings > 25) {
        baselineAltitude = altSum / validReadings;
        Serial.println(" SUCCESS (" + String(baselineAltitude, 1) + "m)");
    } else {
        baselineAltitude = 0.0;
        Serial.println(" FAILED (insufficient readings)");
    }
}

void readLocalSensors() {
    // Read BMP390 sensor
    if (sensorsInitialized && bmp390.performReading()) {
        temperature = bmp390.temperature;
        pressure = bmp390.pressure / 100.0; // Convert Pa to hPa
        altitude = bmp390.readAltitude(1013.25) - baselineAltitude;
    } else {
        // Use default values if sensor fails
        temperature = 0.0;
        pressure = 0.0;
        altitude = 0.0;
    }
    
    // Read servo status (lid position)
    // This is a simple implementation - in reality, you might have feedback
    // For now, we track the commanded position
    
    // Dummy PID output (placeholder for actual control loop)
    // In a real implementation, this would be calculated based on control requirements
    pidOutput = sin(millis() / 1000.0) * 10.0; // Example: sinusoidal output for demo
}

void sendDataToPrimary() {
    // Send local sensor data to Primary ESP32 via UART2
    String sensorData = "SECONDARY_DATA:";
    sensorData += String(temperature, 2) + ",";
    sensorData += String(pressure, 2) + ",";
    sensorData += String(altitude, 2) + ",";
    sensorData += String(lidOpen ? 1 : 0) + ",";
    sensorData += String(pidOutput, 2);
    
    primarySerial.println(sensorData);
    
    // Debug output
    Serial.println("Sent to Primary: " + sensorData);
}

void receiveFromPrimary() {
    if (primarySerial.available()) {
        String receivedData = primarySerial.readStringUntil('\n');
        receivedData.trim();
        
        if (receivedData.length() > 0) {
            lastPrimaryData = millis();
            primaryDataReceived = true;
            
            // Check if this is consolidated CSV telemetry data
            if (receivedData.startsWith("CSV:")) {
                String csvData = receivedData.substring(4); // Remove "CSV:" prefix
                
                // Transmit the CSV data via LoRa
                transmitViaLoRa(csvData);
                
                Serial.println("Received CSV from Primary: " + csvData);
            } else if (receivedData.startsWith("SERVO_OPEN")) {
                // Command to open lid
                lidServo.write(90);
                lidOpen = true;
                Serial.println("Lid opened by Primary command");
            } else if (receivedData.startsWith("SERVO_CLOSE")) {
                // Command to close lid
                lidServo.write(0);
                lidOpen = false;
                Serial.println("Lid closed by Primary command");
            } else if (receivedData.equals("REQ_DATA")) {
                // Primary is requesting fresh sensor data
                readLocalSensors(); // Get latest sensor readings
                sendDataToPrimary(); // Send immediate response
                Serial.println("Data request from Primary - sending fresh data");
            } else {
                Serial.println("Received from Primary: " + receivedData);
            }
        }
    }
}

void transmitViaLoRa(String csvData) {
    if (!loraInitialized) {
        loraTransmissionFailed = true;
        Serial.println("LoRa transmission failed: LoRa not initialized");
        return;
    }
    
    Serial.print("Transmitting via LoRa: ");
    
    // Begin LoRa packet
    if (LoRa.beginPacket()) {
        LoRa.print(csvData);
        
        // End packet and check if transmission was successful
        if (LoRa.endPacket()) {
            loraTransmissionFailed = false;
            Serial.println("SUCCESS");
        } else {
            loraTransmissionFailed = true;
            Serial.println("FAILED (endPacket failed)");
        }
    } else {
        loraTransmissionFailed = true;
        Serial.println("FAILED (beginPacket failed)");
    }
}

void updateLEDs() {
    // D2 LED Logic:
    // - ON during sensor initialization (handled in setup())
    // - BLINK when receiving data from Primary
    
    if (primaryDataReceived) {
        // Blink D2 when receiving data from Primary
        if (millis() - lastLedBlink >= LED_BLINK_INTERVAL) {
            ledBlinkState = !ledBlinkState;
            digitalWrite(LED_D2, ledBlinkState);
            lastLedBlink = millis();
        }
    } else {
        // Turn OFF D2 when not receiving data
        digitalWrite(LED_D2, LOW);
    }
    
    // D13 LED Logic:
    // - ON if LoRa transmission fails
    // - BLINK if not receiving data from Primary
    // - OFF otherwise
    
    if (loraTransmissionFailed) {
        // Solid ON if LoRa transmission failed
        digitalWrite(LED_D13, HIGH);
    } else if (millis() - lastPrimaryData > PRIMARY_DATA_TIMEOUT) {
        // Blink if not receiving data from Primary
        if (millis() - lastLedBlink >= LED_BLINK_INTERVAL) {
            ledBlinkState = !ledBlinkState;
            digitalWrite(LED_D13, ledBlinkState);
            lastLedBlink = millis();
        }
    } else {
        // Turn OFF D13 in normal operation
        digitalWrite(LED_D13, LOW);
        loraTransmissionFailed = false; // Reset transmission failure flag
    }
    
    // Reset primaryDataReceived flag for next cycle
    if (millis() - lastPrimaryData < 100) {
        // Keep flag true for a short period to ensure LED blink is visible
    } else {
        primaryDataReceived = false;
    }
}
