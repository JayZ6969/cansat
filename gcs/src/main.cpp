#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// LoRa pin definitions for ESP32
#define LORA_SCK  18  // SCK pin
#define LORA_MISO 19  // MISO pin
#define LORA_MOSI 23  // MOSI pin
#define LORA_SS   5   // NSS pin
#define LORA_RST  27  // RST pin
#define LORA_DIO0 15  // DIO0 pin

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("LoRa Ground Station Initializing...");

  // Setup LoRa transceiver module
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa with 433MHz frequency
  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // Set LoRa parameters
  LoRa.setSpreadingFactor(7);           // ranges from 6-12, default 7
  LoRa.setSignalBandwidth(125E3);       // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3
  LoRa.setCodingRate4(5);               // ranges from 5-8, default 5
  LoRa.setSyncWord(0xF3);               // ranges from 0-0xFF, default 0x34
  
  Serial.println("LoRa Ground Station Initialized Successfully!");
  Serial.println("Listening for incoming packets...");
}

void loop() {
  // Check for incoming LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Read packet
    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }
    
    // Print received data
    Serial.print("Received packet: ");
    Serial.print(receivedData);
    Serial.print(" | RSSI: ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" | SNR: ");
    Serial.println(LoRa.packetSnr());
    
    // Optional: Send acknowledgment back
    // sendAcknowledgment();
  }
  
  delay(100); // Small delay to prevent excessive polling
}

// Optional function to send acknowledgment
void sendAcknowledgment() {
  LoRa.beginPacket();
  LoRa.print("ACK");
  LoRa.endPacket();
  Serial.println("Acknowledgment sent");
}