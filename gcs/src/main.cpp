// MAXIMUM SPEED LoRa receiver for GCS - OPTIMIZED
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

const long LORA_FREQ = 435E6;
const int LORA_SCK  = 18;
const int LORA_MISO = 19;
const int LORA_MOSI = 23;
const int LORA_SS   = 5;   // CS
const int LORA_RST  = 4;
const int LORA_DIO0 = 26;

unsigned long lastHeartbeat = 0;
uint32_t rxCount = 0;

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("MAXIMUM SPEED LoRa RX starting...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed - check wiring and module");
    while (1) delay(1000);
  }
  
  // OPTIMIZE LORA FOR MAXIMUM SPEED RECEPTION - MATCH TX SETTINGS
  LoRa.setSpreadingFactor(7);     // SF7 fastest
  LoRa.setSignalBandwidth(250E3); // 250kHz fastest bandwidth
  LoRa.setCodingRate4(5);         // 4/5 fastest coding rate
  LoRa.setPreambleLength(6);      // Minimum preamble
  LoRa.setSyncWord(0x12);
  
  Serial.println("GCS receiver optimized for MAXIMUM SPEED reception");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    rxCount++;
    String payload = "";
    while (LoRa.available()) payload += (char)LoRa.read();
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    unsigned long now = millis();
    Serial.printf("RX #%lu @ %lu ms | %d bytes | RSSI=%d dB | SNR=%.1f | %s\n", 
                  (unsigned long)rxCount, now, packetSize, rssi, snr, payload.c_str());
  }
  
  // Periodic heartbeat every 10 seconds
  if (millis() - lastHeartbeat > 10000) {
    Serial.printf("GCS alive - received %lu packets\n", (unsigned long)rxCount);
    lastHeartbeat = millis();
  }
  
  // No delay - maximum reception speed
}