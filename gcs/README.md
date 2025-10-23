# CanSat Ground Control Station (GCS) - Professional Telemetry Receiver

High-performance ground station for real-time CanSat telemetry reception using ESP32 and optimized LoRa SX1278 radio communication at 435MHz.

## 📡 Mission Overview

Professional ground control system designed for CanSat competition telemetry reception. Receives 23-field CSV telemetry data at maximum data rate using SF7/250kHz LoRa configuration for real-time mission monitoring and data logging.

## 🛠️ Hardware Configuration

### Core Components
- **ESP32 Development Board** (30 pins) - High-performance receiver controller
- **LoRa SX1278 Module** - 435MHz competition-grade transceiver
- **USB Power/Data** - Computer connection for telemetry display
- **Stable Power Supply** - Critical for continuous reception

### Optimized Pin Configuration

| LoRa SX1278 Pin | ESP32 GPIO | Function | Notes |
|-----------------|------------|----------|--------|
| RST             | GPIO 27    | Reset    | Hardware reset control |
| DIO0            | GPIO 15    | Digital I/O 0 | Packet detection |
| MISO            | GPIO 19    | SPI MISO | Data from LoRa |
| SCK             | GPIO 18    | SPI Clock | High-speed SPI clock |
| NSS (CS)        | GPIO 5     | Chip Select | SPI slave select |
| MOSI            | GPIO 23    | SPI MOSI | Data to LoRa |
| VCC             | 3.3V       | Power Supply | Stable 3.3V required |
| GND             | GND        | Ground | Clean ground connection |

### LoRa Configuration Specifications
- **Frequency**: 435MHz (Competition standard)
- **Spreading Factor**: SF7 (Maximum speed)
- **Bandwidth**: 250kHz (High data rate)
- **Coding Rate**: 4/5 (Balanced error correction)
- **Sync Word**: 0x12 (Standard LoRa sync)

## 📋 Professional Setup & Configuration

### Development Environment
- **PlatformIO** (recommended for competition-grade development)
- **ESP32 Board Package** (latest stable version)
- **Serial Monitor** @ 115200 baud for clean telemetry display

### Required Libraries
- **LoRa Library**: `sandeepmistry/LoRa@^0.8.0` - Optimized SX1278 interface
- **ESP32 Core**: Latest stable for maximum performance

## 🚀 Quick Deployment

### 1. **Hardware Setup**
1. Connect LoRa SX1278 to ESP32 using pin configuration above
2. Verify 3.3V power supply stability (critical for 435MHz operation)
3. Ensure antenna connection for optimal 435MHz reception
4. USB connection to computer for telemetry display

### 2. **Software Deployment**
```bash
# Navigate to GCS directory
cd cansat/gcs

# Build optimized firmware
pio run

# Upload to ESP32
pio run --target upload

# Monitor telemetry reception
pio device monitor
```

### 3. **Operational Verification**
1. **Initialization**: Check [INIT] messages for successful LoRa setup
2. **Reception Test**: Verify 435MHz SF7/250kHz configuration
3. **Signal Quality**: Monitor RSSI and SNR values for link quality
4. **Data Validation**: Confirm 23-field CSV telemetry parsing

## 📊 Enhanced Features

### Professional Telemetry Reception
- **435MHz Reception**: Competition frequency with SF7/250kHz for maximum data rate
- **Real-time CSV Processing**: Handles 23-field telemetry format from CanSat
- **Signal Quality Monitoring**: RSSI/SNR display for link assessment
- **Clean Output Format**: Professional logging matching Primary/Secondary format

### Advanced LoRa Configuration
- **Frequency**: 435MHz (Competition standard)
- **Spreading Factor**: SF7 (Maximum speed, shorter range)
- **Bandwidth**: 250kHz (High data rate)
- **Coding Rate**: 4/5 (Balanced error correction)
- **Sync Word**: 0x12 (Standard LoRa synchronization)

## 🖥️ Professional Serial Output

### Clean Telemetry Display
```
[INIT] LoRa Ground Station Initializing...
[INIT] Frequency: 435.000 MHz, SF: 7, BW: 250 kHz
[STATUS] Ground Station Ready - Listening for telemetry...

[DATA] 1013,10:34:23.456,1254,3,ASCENT,1250.4,15.2,N,N,N,22.5,7.2,1013.2,10:34:22,1248.7,40.7128,-74.0060,8,5.2,-2.1,45.3,NONE, | RSSI: -65 | SNR: 8.5

[DATA] 1013,10:34:23.556,1255,3,ASCENT,1251.8,15.4,N,N,N,22.3,7.2,1013.1,10:34:23,1249.2,40.7129,-74.0061,8,5.1,-2.2,46.1,NONE, | RSSI: -67 | SNR: 7.8
```

### Telemetry Format (23 Fields)
Receives standardized CSV format from CanSat Secondary ESP32:
- **Team ID, Mission Time, Packet Count, Mode, State**
- **Altitude, Airspeed, Deployment Status (HS/PC/Mast)**
- **Temperature, Voltage, Pressure**
- **GPS Data (Time, Altitude, Lat/Lon, Satellites)**
- **Orientation (Tilt X/Y, Rotation Z)**
- **Command Echo, Error Codes**

## 🔧 Advanced Configuration

### Competition-Optimized Parameters
```cpp
// Competition frequency and maximum speed settings
LoRa.setFrequency(435E6);        // 435 MHz competition frequency
LoRa.setSpreadingFactor(7);      // SF7 for maximum data rate
LoRa.setSignalBandwidth(250E3);  // 250kHz for high throughput
LoRa.setCodingRate4(5);          // 4/5 coding rate (balanced)
LoRa.setSyncWord(0x12);          // Standard LoRa sync word
```

### Range vs Speed Optimization
```cpp
// Long Range (slower): SF9-12, BW=125kHz
// Balanced: SF7-8, BW=250kHz (current configuration)
// Maximum Speed: SF6-7, BW=500kHz (experimental)
```

## 🛠️ Professional Troubleshooting

### System Diagnostics

1. **LoRa Initialization Failure**
   ```
   [ERROR] LoRa initialization failed!
   ```
   - Verify SPI connections (MISO/MOSI/SCK/CS pins)
   - Check 3.3V power stability (measure with multimeter)
   - Confirm GPIO pin assignments match hardware

2. **No Telemetry Reception**
   ```
   [STATUS] Listening... (no packets received)
   ```
   - Verify 435MHz frequency match with Secondary ESP32
   - Check SF7/250kHz parameter synchronization
   - Ensure LoRa antenna is properly connected
   - Verify Secondary ESP32 is transmitting

3. **Poor Signal Quality**
   ```
   [DATA] ... | RSSI: -90 | SNR: -5.2
   ```
   - RSSI < -80dBm: Move closer or improve antenna
   - SNR < 0: Reduce interference or increase power
   - Check antenna orientation and polarization

4. **CSV Format Errors**
   - Monitor for incomplete packets
   - Verify 23-field format compatibility
   - Check for transmission corruption

## 📁 Project Structure

```
gcs/
├── src/
│   └── main.cpp          # Main application code
├── include/
│   └── README            # Include directory info
├── lib/
│   └── README            # Library directory info
├── test/
│   └── README            # Test directory info
├── platformio.ini        # PlatformIO configuration
└── README.md            # This file
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 👥 Authors

- **JayZ6969** - *Initial work* - [GitHub Profile](https://github.com/JayZ6969)

## 📞 Support

If you have any questions or issues, please open an issue on GitHub or contact the maintainers.

---

**Happy CanSat tracking! 🛰️**
