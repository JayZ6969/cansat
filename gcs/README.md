# CanSat Ground Control Station (GCS)

A Ground Control Station for receiving telemetry data from CanSat satellites using ESP32 microcontroller and LoRa RA-02 (SX1278) radio module.

## 📡 Project Overview

This project implements a ground station that receives telemetry data from a CanSat (Can-sized Satellite) using LoRa (Long Range) radio communication. The ground station displays received data along with signal quality metrics like RSSI and SNR.

## 🛠️ Hardware Requirements

### Main Components
- **ESP32 Development Board** - Main microcontroller
- **LoRa RA-02 Module (SX1278)** - 433MHz radio transceiver
- **Jumper wires** - For connections
- **Breadboard/PCB** - For prototyping

### Pin Configuration

| LoRa RA-02 Pin | ESP32 GPIO | Function |
|----------------|------------|----------|
| RST            | GPIO 27    | Reset    |
| DIO0           | GPIO 15    | Digital I/O 0 |
| MISO           | GPIO 19    | SPI MISO |
| SCK            | GPIO 18    | SPI Clock |
| NSS            | GPIO 5     | SPI Chip Select |
| MOSI           | GPIO 23    | SPI MOSI |
| VCC            | 3.3V       | Power Supply |
| GND            | GND        | Ground |

## 📋 Software Requirements

### Development Environment
- **PlatformIO** (recommended) or Arduino IDE
- **ESP32 Board Package**

### Libraries
- `sandeepmistry/LoRa@^0.8.0` - LoRa communication library

## 🚀 Getting Started

### 1. Hardware Setup
1. Connect the LoRa RA-02 module to ESP32 according to the pin configuration table above
2. Ensure all connections are secure and power supply is stable
3. Connect ESP32 to your computer via USB

### 2. Software Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/JayZ6969/cansat.git
   cd cansat/gcs
   ```

2. Open the project in PlatformIO or copy the code to Arduino IDE

3. The required libraries will be automatically downloaded when building

### 3. Upload and Run
1. Build and upload the code to your ESP32
2. Open Serial Monitor at **115200 baud rate**
3. The ground station will initialize and start listening for packets

## 📊 Features

### Core Functionality
- **LoRa Reception**: Continuously listens for incoming LoRa packets
- **Signal Quality Monitoring**: Displays RSSI (Received Signal Strength Indicator) and SNR (Signal-to-Noise Ratio)
- **Real-time Data Display**: Shows received telemetry data on serial monitor
- **Optional Acknowledgment**: Can send ACK packets back to transmitter

### LoRa Configuration
- **Frequency**: 433 MHz
- **Spreading Factor**: 7 (adjustable 6-12)
- **Bandwidth**: 125 kHz
- **Coding Rate**: 4/5
- **Sync Word**: 0xF3

## 🖥️ Serial Output Example

```
LoRa Ground Station Initializing...
LoRa Ground Station Initialized Successfully!
Listening for incoming packets...
Received packet: TEMP:25.6,ALT:1250,LAT:12.345,LON:77.123 | RSSI: -65 | SNR: 8.5
Received packet: TEMP:24.8,ALT:1300,LAT:12.346,LON:77.124 | RSSI: -67 | SNR: 7.2
```

## 🔧 Configuration

### Adjusting LoRa Parameters
You can modify the LoRa settings in `main.cpp`:

```cpp
LoRa.setSpreadingFactor(7);     // Range: 6-12 (higher = longer range, slower)
LoRa.setSignalBandwidth(125E3); // Options: 7.8E3 to 250E3
LoRa.setCodingRate4(5);         // Range: 5-8 (higher = more error correction)
LoRa.setSyncWord(0xF3);         // Sync word for network identification
```

### Changing Frequency
Modify the frequency in the `setup()` function:
```cpp
LoRa.begin(433E6);  // 433 MHz (ensure it matches your transmitter)
```

## 🛠️ Troubleshooting

### Common Issues

1. **"Starting LoRa failed!" error**
   - Check all wiring connections
   - Verify power supply is stable (3.3V)
   - Ensure LoRa module is properly seated

2. **No packets received**
   - Verify transmitter is using same frequency and parameters
   - Check antenna connections
   - Ensure transmitter is within range

3. **Poor signal quality (low RSSI/SNR)**
   - Improve antenna positioning
   - Reduce distance between transmitter and receiver
   - Check for interference sources

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
