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
  
  Serial.print("[LOGS] LoRa receiver (435MHz)...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(" FAILED");
    Serial.println("[LOGS] Check wiring and module");
    while (1) delay(1000);
  }
  
  // OPTIMIZE LORA FOR MAXIMUM SPEED RECEPTION - MATCH TX SETTINGS
  LoRa.setSpreadingFactor(7);     // SF7 fastest
  LoRa.setSignalBandwidth(250E3); // 250kHz fastest bandwidth
  LoRa.setCodingRate4(5);         // 4/5 fastest coding rate
  LoRa.setPreambleLength(6);      // Minimum preamble
  LoRa.setSyncWord(0x12);
  
  Serial.println(" OK (SF7, 250kHz)");
  Serial.println("[LOGS] Ground station ready");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    rxCount++;
    String payload = "";
    while (LoRa.available()) payload += (char)LoRa.read();
    
    // Print received CSV data with RSSI and SNR on same line
    Serial.print("[CSV] " + payload);
    Serial.print(" | RSSI: ");
    Serial.print(LoRa.packetRssi());
    Serial.print(" dBm | SNR: ");
    Serial.print(LoRa.packetSnr());
    Serial.println(" dB");
  }
  
  // Periodic status every 30 seconds (less frequent than Primary/Secondary)
  if (millis() - lastHeartbeat > 30000) {
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    Serial.print("[LOGS] Packets: ");
    Serial.print(rxCount);
    Serial.print(" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dB | SNR: ");
    Serial.print(snr, 1);
    Serial.println(" dB");
    lastHeartbeat = millis();
  }
  
  // No delay - maximum reception speed
}