/*
 * SECONDARY ESP32 - CanSat Mission Controller
 * Board: ESP32 Devkit (30 pins)
 * Role: Local sensor data collection + LoRa transmission of Primary's CSV data
 *
 * Hardware:
 * - LoRa SX1278 (SPI)
 * - BMP390 sensor (I2C)
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

// LoRa SX1278 Pin Definitions (SPI)
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 4
#define LORA_DIO0 26
#define LORA_BAND 435E6 // 435 MHz for optimized transmission

// LED Pin Definitions
#define LED_D2 2   // ON during init, BLINK when receiving from Primary
#define LED_D13 13 // ON if LoRa fails, BLINK if no Primary data

// I2C BMP390 Address
#define BMP390_I2C_ADDR 0x77

// UART2 Pins for Primary Communication
#define UART2_RX 16
#define UART2_TX 17

// Voltage Sensor Pin (moved from Primary GPIO36 to Secondary GPIO25)
#define BATTERY_ADC_PIN 25

// Timing Constants - OPTIMIZED FOR MAXIMUM SPEED
#define SENSOR_READ_INTERVAL 50   // Read sensors every 50ms (20Hz maximum speed)
#define PRIMARY_DATA_TIMEOUT 2000 // Consider Primary lost after 2 seconds
#define LORA_TX_TIMEOUT 500       // LoRa transmission timeout (faster)

// LED Timing Constants (matching Primary ESP32 timing logic)
const unsigned long LED_D2_BLINK_DURATION = 100;  // 100ms blink duration
const unsigned long LED_D13_BLINK_DURATION = 100; // 100ms blink duration

// Global Objects
Adafruit_BMP3XX bmp390;
HardwareSerial primarySerial(2); // UART2 for primary communication

// System State Variables
bool sensorsInitialized = false;
bool loraInitialized = false;
unsigned long bootTime = 0;
unsigned long lastSensorRead = 0;
unsigned long lastPrimaryData = 0;
unsigned long lastVoltageRead = 0; // Track last voltage reading time

// Local Sensor Data
float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;
float baselineAltitude = 0.0;
float voltage = 0.0;
float pidOutput = 0.0; // Dummy PID output or implement actual control

// Communication State
bool primaryDataReceived = false;
bool loraTransmissionFailed = false;

// Error tracking for Secondary subsystems
bool bmp390Error = false; // BMP390 sensor error
bool loraError = false;   // LoRa transmission error
bool pidError = false;    // PID control error

// LED Timing State Variables (matching Primary ESP32 timing logic)
unsigned long ledD2BlinkStart = 0;
unsigned long ledD13BlinkStart = 0;
bool ledBlinkState = false;

// Function Declarations
void initializeSensors();
void initializeLoRa();
void readLocalSensors();
void readBattery();
void sendDataToPrimary();
void receiveFromPrimary();
void transmitViaLoRa(String csvData);
void updateLEDs();
void calibrateBaseline();
void checkSecondaryErrors();
String getErrorStatus();

void setup()
{
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
    lastVoltageRead = millis(); // Initialize to allow immediate first read

    Serial.println("\n[SECONDARY] CanSat Communication Module");
    Serial.println("========================================");

    // Initialize all systems
    initializeSensors();
    initializeLoRa();
    calibrateBaseline();

    // Initialization complete - turn OFF D2
    digitalWrite(LED_D2, LOW);

    Serial.println("[INIT] Secondary ESP32 ready");
    Serial.println("========================================\n");
}

void loop()
{
    // Always check for commands from Primary first (highest priority)
    receiveFromPrimary();

    // Read local sensors at maximum speed - no delay multiplier
    if (millis() - lastSensorRead >= SENSOR_READ_INTERVAL)
    {
        readLocalSensors();
        // Check all subsystem errors
        checkSecondaryErrors();
        // Don't auto-send data - let Primary request it via REQ_DATA when needed
        lastSensorRead = millis();
    }

    // Update LED status indicators
    updateLEDs();

    // Periodic status output (every 5 seconds)
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint >= 5000)
    {
        Serial.print("[STATUS] ALT:");
        Serial.print(altitude, 1);
        Serial.print("m | TEMP:");
        Serial.print(temperature, 1);
        Serial.print("C | VOLT:");
        Serial.print(voltage, 2);
        Serial.print("V | LoRa:");
        Serial.print(loraError ? "ERR" : "OK");
        Serial.print(" | BMP390:");
        Serial.println(bmp390Error ? "ERR" : "OK");
        lastStatusPrint = millis();
    }

    // Minimal delay - OPTIMIZED FOR MAXIMUM SPEED
    delay(1);
}

void initializeSensors()
{
    Serial.print("[INIT] BMP390 sensor...");

    if (bmp390.begin_I2C(BMP390_I2C_ADDR))
    {
        // Configure BMP390 for optimal performance
        bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp390.setOutputDataRate(BMP3_ODR_50_HZ);

        sensorsInitialized = true;
        Serial.println(" OK");
    }
    else
    {
        sensorsInitialized = false;
        bmp390Error = true;
        Serial.println(" FAILED");
    }
}

void initializeLoRa()
{
    Serial.print("[INIT] LoRa SX1278 (435MHz)...");

    // Initialize SPI for LoRa
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (LoRa.begin(LORA_BAND))
    {
        // Configure LoRa parameters - OPTIMIZED FOR MAXIMUM SPEED
        LoRa.setSpreadingFactor(7);     // SF7 for fastest transmission
        LoRa.setSignalBandwidth(250E3); // 250 kHz bandwidth (faster than 125E3)
        LoRa.setCodingRate4(5);         // 4/5 coding rate (fastest)
        LoRa.setPreambleLength(6);      // 6 symbol preamble (minimum for reliability)
        LoRa.setSyncWord(0x12);         // Sync word
        LoRa.setTxPower(20);            // Maximum TX power for range
        LoRa.enableCrc();               // Enable CRC for error detection

        loraInitialized = true;
        Serial.println(" OK (SF7, 250kHz)");
    }
    else
    {
        loraInitialized = false;
        loraError = true;
        Serial.println(" FAILED");
    }
}

void calibrateBaseline()
{
    if (!sensorsInitialized)
    {
        baselineAltitude = 0.0;
        Serial.println("[WARN] BMP390 not ready - skipping baseline");
        return;
    }

    Serial.print("[INIT] Calibrating baseline...");

    float altSum = 0.0;
    int validReadings = 0;

    // Take 50 readings over 2.5 seconds
    for (int i = 0; i < 50; i++)
    {
        if (bmp390.performReading())
        {
            altSum += bmp390.readAltitude(1013.25); // Standard sea level pressure
            validReadings++;
        }
        delay(50);
    }

    if (validReadings > 25)
    {
        baselineAltitude = altSum / validReadings;
        Serial.print(" OK (");
        Serial.print(String(baselineAltitude, 1));
        Serial.println(" m)");
    }
    else
    {
        baselineAltitude = 0.0;
        Serial.println(" FAILED");
    }
}

void readLocalSensors()
{
    // Read BMP390 sensor
    if (sensorsInitialized && bmp390.performReading())
    {
        temperature = bmp390.temperature;
        pressure = bmp390.pressure / 100.0; // Convert Pa to hPa
        altitude = bmp390.readAltitude(1013.25) - baselineAltitude;
    }
    else
    {
        // Use default values if sensor fails
        temperature = 0.0;
        pressure = 0.0;
        altitude = 0.0;
    }

    // Read battery voltage
    readBattery();

    // Dummy PID output (placeholder for actual control loop)
    // In a real implementation, this would be calculated based on control requirements
    pidOutput = sin(millis() / 1000.0) * 10.0; // Example: sinusoidal output for demo
}

void readBattery()
{
    // Only update voltage once per minute to reduce fluctuations
    if (millis() - lastVoltageRead >= 60000) // 60 seconds
    {
        int adcValue = analogRead(BATTERY_ADC_PIN);
        // Convert ADC to voltage for 18650 x 2 with voltage divider
        // ADC reading * (3.3V / 4095) * calibrated_multiplier
        // Calibrated multiplier: 5.33 (adjusted to match actual battery voltage)
        voltage = (adcValue / 4095.0) * 3.3 * 5.33;
        lastVoltageRead = millis();
    }
    // Else, keep the previous voltage value
}

void sendDataToPrimary()
{
    // Check all subsystem errors before sending
    checkSecondaryErrors();

    // Send local sensor data to Primary ESP32 via UART2
    String sensorData = "DATA:";
    sensorData += "ALT:" + String(altitude, 2) + ",";
    sensorData += "PRESS:" + String(pressure, 2) + ",";
    sensorData += "TEMP:" + String(temperature, 2) + ",";
    sensorData += "VOLT:" + String(voltage, 2) + ",";
    sensorData += "PID:" + String(pidOutput, 2) + ",";
    sensorData += "ERR:" + getErrorStatus();

    primarySerial.println(sensorData);

    // Debug output removed for cleaner serial monitor
    // Serial.println("[TX] Data sent to Primary");
}

void receiveFromPrimary()
{
    if (primarySerial.available())
    {
        String receivedData = primarySerial.readStringUntil('\n');
        receivedData.trim();

        if (receivedData.length() > 0)
        {
            lastPrimaryData = millis();
            primaryDataReceived = true;

            // Check if this is consolidated CSV telemetry data
            if (receivedData.startsWith("CSV:"))
            {
                String csvData = receivedData.substring(4); // Remove "CSV:" prefix

                // Transmit the CSV data via LoRa
                transmitViaLoRa(csvData);

                // Serial.println("[RX] CSV from Primary"); // Debug disabled
            }
            else if (receivedData.equals("REQ_DATA"))
            {
                // Primary is requesting fresh sensor data
                readLocalSensors();  // Get latest sensor readings
                sendDataToPrimary(); // Send immediate response
                // Serial.println("[RX] Data request"); // Debug disabled
            }
            // else: Unknown command - ignore silently
        }
    }
}

void transmitViaLoRa(String csvData)
{
    if (!loraInitialized)
    {
        loraTransmissionFailed = true;
        loraError = true;
        Serial.println("[ERROR] LoRa not initialized");
        return;
    }

    // Print the CSV data being transmitted (matching Primary format)
    Serial.println("CSV:" + csvData);

    // Begin LoRa packet - OPTIMIZED FOR SPEED
    if (LoRa.beginPacket())
    {
        LoRa.print(csvData);

        // Use async (non-blocking) transmission for maximum speed
        if (LoRa.endPacket(true)) // true = async/non-blocking
        {
            loraTransmissionFailed = false;
            loraError = false;
            // Serial.println("[TX] LoRa OK"); // Debug disabled for cleaner output
        }
        else
        {
            loraTransmissionFailed = true;
            loraError = true;
            Serial.println("[ERROR] LoRa TX failed (endPacket)");
        }
    }
    else
    {
        loraTransmissionFailed = true;
        loraError = true;
        Serial.println("[ERROR] LoRa TX failed (beginPacket)");
    }
}

void updateLEDs()
{
    // D2 LED Logic (matching Primary ESP32 timing logic):
    // - ON during sensor initialization (handled in setup())
    // - BLINK when receiving data from Primary (25ms ON, 75ms OFF pattern)

    if (primaryDataReceived)
    {
        // D2 LED blinks with same pattern as Primary's Green LED when receiving data
        if (ledD2BlinkStart == 0)
        {
            ledD2BlinkStart = millis();
        }

        unsigned long d2Elapsed = millis() - ledD2BlinkStart;

        if (d2Elapsed < LED_D2_BLINK_DURATION)
        {
            // Blinking - ON for 25ms, OFF for 75ms within the 100ms blink period
            if (d2Elapsed < 25)
            {
                digitalWrite(LED_D2, HIGH);
            }
            else
            {
                digitalWrite(LED_D2, LOW);
            }
        }
        else if (d2Elapsed >= 1000) // Repeat every 1 second when receiving data
        {
            // Start new blink cycle
            ledD2BlinkStart = millis();
        }
        else
        {
            // Wait period - LED stays off
            digitalWrite(LED_D2, LOW);
        }
    }
    else
    {
        // Turn OFF D2 when not receiving data and reset timing
        digitalWrite(LED_D2, LOW);
        ledD2BlinkStart = 0;
    }

    // D13 LED Logic (matching Primary ESP32 timing logic):
    // - ON if LoRa transmission fails (solid)
    // - BLINK if not receiving data from Primary (50ms ON, 50ms OFF pattern)
    // - OFF otherwise

    if (loraTransmissionFailed)
    {
        // Solid ON if LoRa transmission failed
        digitalWrite(LED_D13, HIGH);
        ledD13BlinkStart = 0; // Reset blink timing
    }
    else if (millis() - lastPrimaryData > PRIMARY_DATA_TIMEOUT)
    {
        // D13 LED blinks with same pattern as Primary's Yellow LED when no Primary data
        if (ledD13BlinkStart == 0)
        {
            ledD13BlinkStart = millis();
        }

        unsigned long d13Elapsed = millis() - ledD13BlinkStart;

        if (d13Elapsed < LED_D13_BLINK_DURATION)
        {
            // ON for first half of duration (50ms ON, 50ms OFF)
            if (d13Elapsed < LED_D13_BLINK_DURATION / 2)
            {
                digitalWrite(LED_D13, HIGH);
            }
            else
            {
                digitalWrite(LED_D13, LOW);
            }
        }
        else if (d13Elapsed >= 1500) // Repeat every 1.5 seconds (same as Primary Yellow)
        {
            // Start new blink cycle
            ledD13BlinkStart = millis();
        }
        else
        {
            // Wait period - LED stays off
            digitalWrite(LED_D13, LOW);
        }
    }
    else
    {
        // Turn OFF D13 in normal operation and reset timing
        digitalWrite(LED_D13, LOW);
        ledD13BlinkStart = 0;
        loraTransmissionFailed = false; // Reset transmission failure flag
    }

    // Reset primaryDataReceived flag for next cycle
    if (millis() - lastPrimaryData < 100)
    {
        // Keep flag true for a short period to ensure LED blink is visible
    }
    else
    {
        primaryDataReceived = false;
    }
}

void checkSecondaryErrors()
{
    // Check BMP390 sensor status
    if (sensorsInitialized)
    {
        // Test if BMP390 is responding
        bmp390Error = !bmp390.performReading();
    }
    else
    {
        bmp390Error = true; // Sensor not initialized
    }

    // LoRa error is already tracked by loraTransmissionFailed
    loraError = loraTransmissionFailed || !loraInitialized;

    // Check PID control status - placeholder implementation
    // In real implementation, this would check if PID controller is functioning properly
    pidError = false; // Set to true if PID control fails
}

String getErrorStatus()
{
    // Return error status as 3-character string: BMP390, LoRa, PID
    String errorStatus = "";
    errorStatus += (bmp390Error ? "1" : "0");
    errorStatus += (loraError ? "1" : "0");
    errorStatus += (pidError ? "1" : "0");
    return errorStatus;
}
