# CanSat Ground Control Station (GCS) Development Guide

## Project Overview

This document outlines the development of a Ground Control Station (GCS) software using Python Tkinter to receive, process, and visualize real-time telemetry data from the CanSat mission.

## System Architecture

```
CanSat (LoRa TX) → ESP32/LoRa Receiver → Serial/USB → Laptop → Python GCS → Dashboard
```

## Data Structure

Based on your CanSat code, the GCS will receive 23 comma-separated values in this order:

```
TEAM_ID,TIMESTAMP,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,GNSS_TIME,GNSS_LAT,GNSS_LONG,GNSS_ALT,GNSS_SATS,ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,GYRO_SPIN_RATE,FLIGHT_STATE,SERVO_STATUS,PID_SPEED,ERROR_CODE
```

## Required Features (Based on Dashboard Image)

### 1. Main Dashboard Layout
- **Map View**: Interactive map showing CanSat location with GPS coordinates
- **Real-time Data Panel**: Current telemetry values display
- **Control Buttons**: Start/Stop plotting, Telemetry, Calibration, Reset Dashboard
- **Mission Info**: Team ID, mission time, packet count display

### 2. Real-time Plots (4 Main Charts)
- **Altitude Plot**: Real-time altitude vs time
- **Pressure Plot**: Atmospheric pressure vs time  
- **Temperature Plot**: Temperature readings vs time
- **Accelerometer Plot**: 3-axis acceleration (X, Y, Z)
- **Gyroscope Plot**: 3-axis gyroscope data (X, Y, Z)
- **Reaction Wheel Plot**: PID output for reaction wheel control

### 3. Data Display Panel
```
Time (mins): 10:25          Mission Time (sec): 309
Camera Recording: Status    Calibration: Done
Parachute Deployed: Status  Mode: Descent
Battery Percentage (%): 95  Battery Voltage (V): 8.30
Altitude (m): 939.86        Pressure (pa): 913.88
Packet Count: 3025          Speed: 0.16
Temperature (°C): 27.7      No of Satellites: 13
Error Indicator: All Good
```

### 4. Flight State Timeline
- Pre Launch → Calibration → Ready to Launch → Ascent → Descent → Touch Down

## Implementation Plan

### Phase 1: Basic Setup and Serial Communication

#### 1.1 Environment Setup
```bash
pip install tkinter matplotlib pyserial folium tkinterweb pandas numpy
```

#### 1.2 Project Structure
```
cansat-gcs/
├── main.py                 # Main GCS application
├── data_parser.py          # Data parsing and validation
├── serial_handler.py       # Serial communication
├── dashboard.py           # Main dashboard GUI
├── plots.py               # Real-time plotting
├── map_view.py            # GPS map integration
├── config.py              # Configuration settings
├── data_logger.py         # Data logging to files
└── requirements.txt       # Dependencies
```

### Phase 2: Core Components Development

#### 2.1 Serial Communication Handler (`serial_handler.py`)
```python
import serial
import threading
import queue
from typing import Optional, Callable

class SerialHandler:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        self.data_queue = queue.Queue()
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
    def connect(self) -> bool:
        """Connect to serial port"""
        
    def disconnect(self):
        """Disconnect from serial port"""
        
    def start_reading(self):
        """Start reading data in background thread"""
        
    def read_data(self) -> Optional[str]:
        """Read data from queue"""
        
    def get_available_ports(self) -> list:
        """Get list of available COM ports"""
```

#### 2.2 Data Parser (`data_parser.py`)
```python
from dataclasses import dataclass
from typing import Optional
import datetime

@dataclass
class CanSatData:
    team_id: str
    timestamp: int
    packet_count: int
    altitude: float
    pressure: float
    temperature: float
    voltage: float
    gnss_time: str
    gnss_lat: float
    gnss_long: float
    gnss_alt: float
    gnss_sats: int
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    gyro_spin_rate: float
    flight_state: int
    servo_status: str
    pid_speed: float
    error_code: str
    
    @property
    def flight_state_name(self) -> str:
        """Convert flight state number to name"""
        states = {
            0: "BOOT", 1: "TEST_MODE", 2: "LAUNCH_PAD", 
            3: "ASCENT", 4: "ROCKET_DEPLOY", 5: "DESCENT",
            6: "AEROBRAKE_RELEASE", 7: "IMPACT"
        }
        return states.get(self.flight_state, "UNKNOWN")
    
    @property
    def mission_time_formatted(self) -> str:
        """Format mission time as MM:SS"""
        
    @property
    def battery_percentage(self) -> float:
        """Calculate battery percentage from voltage"""

class DataParser:
    def __init__(self):
        self.expected_fields = 23
        
    def parse_csv_line(self, csv_line: str) -> Optional[CanSatData]:
        """Parse CSV line into CanSatData object"""
        
    def validate_data(self, data: CanSatData) -> bool:
        """Validate data ranges and types"""
```

#### 2.3 Real-time Plotting (`plots.py`)
```python
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from collections import deque
import numpy as np

class RealTimePlot:
    def __init__(self, parent, title: str, ylabel: str, max_points: int = 100):
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.values = deque(maxlen=max_points)
        
        # Create matplotlib figure
        self.fig, self.ax = plt.subplots(figsize=(6, 3))
        self.ax.set_title(title)
        self.ax.set_ylabel(ylabel)
        self.ax.set_xlabel("Time")
        
        # Create Tkinter canvas
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        
    def add_data_point(self, time_val: float, value: float):
        """Add new data point to plot"""
        
    def update_plot(self):
        """Update the plot display"""
        
    def clear_plot(self):
        """Clear all data from plot"""

class MultiAxisPlot(RealTimePlot):
    """For accelerometer and gyroscope 3-axis data"""
    def __init__(self, parent, title: str, ylabel: str, max_points: int = 100):
        super().__init__(parent, title, ylabel, max_points)
        self.x_values = deque(maxlen=max_points)
        self.y_values = deque(maxlen=max_points) 
        self.z_values = deque(maxlen=max_points)
        
    def add_data_points(self, time_val: float, x: float, y: float, z: float):
        """Add 3-axis data points"""
```

#### 2.4 Map Integration (`map_view.py`)
```python
import folium
import tkinter as tk
from tkinterweb import HtmlFrame
import tempfile
import os

class MapView:
    def __init__(self, parent):
        self.parent = parent
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.map_file = os.path.join(tempfile.gettempdir(), "cansat_map.html")
        
        # Create HTML frame for map
        self.html_frame = HtmlFrame(parent, messages_enabled=False)
        
        # Initialize map
        self.init_map()
        
    def init_map(self):
        """Initialize folium map"""
        
    def update_location(self, lat: float, lon: float, altitude: float):
        """Update CanSat location on map"""
        
    def add_flight_path(self, coordinates: list):
        """Add flight path to map"""
```

#### 2.5 Main Dashboard (`dashboard.py`)
```python
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
from datetime import datetime

class CanSatDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CanSat Ground Control Station")
        self.root.geometry("1400x900")
        
        # Data storage
        self.latest_data = None
        self.data_history = []
        self.is_plotting = False
        
        # Initialize components
        self.setup_ui()
        self.setup_serial()
        self.setup_plots()
        
    def setup_ui(self):
        """Setup main UI layout"""
        
    def setup_serial(self):
        """Setup serial communication"""
        
    def setup_plots(self):
        """Setup real-time plots"""
        
    def create_data_panel(self):
        """Create data display panel"""
        
    def create_control_panel(self):
        """Create control buttons panel"""
        
    def create_status_timeline(self):
        """Create flight state timeline"""
        
    def update_displays(self):
        """Update all display elements"""
        
    def start_plotting(self):
        """Start real-time plotting"""
        
    def stop_plotting(self):
        """Stop real-time plotting"""
        
    def calibration_mode(self):
        """Enter calibration mode"""
        
    def reset_dashboard(self):
        """Reset all displays and data"""
        
    def save_data(self):
        """Save current session data"""
```

### Phase 3: Advanced Features

#### 3.1 Data Logging and Export
```python
class DataLogger:
    def __init__(self, filename_prefix: str = "cansat_mission"):
        self.filename_prefix = filename_prefix
        self.current_session_file = None
        
    def start_logging(self):
        """Start logging data to file"""
        
    def log_data_point(self, data: CanSatData):
        """Log single data point"""
        
    def export_to_csv(self, filename: str):
        """Export data to CSV"""
        
    def export_to_kml(self, filename: str):
        """Export GPS track to KML for Google Earth"""
```

#### 3.2 Alert System
```python
class AlertSystem:
    def __init__(self, dashboard):
        self.dashboard = dashboard
        self.alerts = []
        
    def check_altitude_limits(self, altitude: float):
        """Check if altitude is within safe limits"""
        
    def check_battery_level(self, voltage: float):
        """Check battery level"""
        
    def check_communication_timeout(self, last_packet_time: float):
        """Check for communication timeout"""
        
    def show_alert(self, message: str, level: str = "WARNING"):
        """Display alert to user"""
```

#### 3.3 Configuration Management
```python
# config.py
class Config:
    # Serial settings
    DEFAULT_BAUDRATE = 115200
    DEFAULT_PORT = "COM3"  # Windows
    
    # Plot settings
    MAX_PLOT_POINTS = 100
    UPDATE_INTERVAL_MS = 100
    
    # Alert thresholds
    MIN_BATTERY_VOLTAGE = 6.0
    MAX_ALTITUDE = 1000.0
    COMMUNICATION_TIMEOUT = 5.0  # seconds
    
    # Map settings
    DEFAULT_LAT = 40.7128
    DEFAULT_LON = -74.0060
    
    # Flight states
    FLIGHT_STATES = {
        0: "BOOT", 1: "TEST_MODE", 2: "LAUNCH_PAD",
        3: "ASCENT", 4: "ROCKET_DEPLOY", 5: "DESCENT", 
        6: "AEROBRAKE_RELEASE", 7: "IMPACT"
    }
```

### Phase 4: UI Layout Implementation

#### 4.1 Main Window Layout
```python
def setup_main_layout(self):
    # Create main frames
    self.left_frame = ttk.Frame(self.root, width=800)
    self.right_frame = ttk.Frame(self.root, width=600)
    
    self.left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    self.right_frame.pack(side=tk.RIGHT, fill=tk.BOTH)
    
    # Left frame - plots and map
    self.setup_plots_frame()
    self.setup_map_frame()
    
    # Right frame - data and controls
    self.setup_data_frame()
    self.setup_control_frame()
    self.setup_timeline_frame()
```

#### 4.2 Data Display Layout
```python
def create_data_display(self):
    # Mission info
    self.mission_frame = ttk.LabelFrame(self.right_frame, text="Mission Info")
    self.mission_frame.pack(fill=tk.X, padx=5, pady=5)
    
    # Real-time values
    self.values_frame = ttk.LabelFrame(self.right_frame, text="Telemetry")
    self.values_frame.pack(fill=tk.X, padx=5, pady=5)
    
    # Status indicators
    self.status_frame = ttk.LabelFrame(self.right_frame, text="Status")
    self.status_frame.pack(fill=tk.X, padx=5, pady=5)
```

## Testing Strategy

### 1. Unit Testing
- Test data parsing with sample CSV data
- Test serial communication with mock data
- Test plot updates with generated data

### 2. Integration Testing
- Test complete data flow from serial to display
- Test error handling for malformed data
- Test UI responsiveness under high data rates

### 3. Field Testing
- Test with actual CanSat hardware
- Test communication range and reliability
- Test data logging and export functions

## Deployment

### 1. Executable Creation
```bash
pip install pyinstaller
pyinstaller --onefile --windowed main.py
```

### 2. Requirements File
```txt
tkinter
matplotlib>=3.5.0
pyserial>=3.5
folium>=0.12.0
tkinterweb>=3.0.0
pandas>=1.3.0
numpy>=1.21.0
pillow>=8.0.0
```

## Error Handling

### 1. Serial Communication Errors
- Port not found
- Connection lost
- Invalid baud rate
- Permission denied

### 2. Data Parsing Errors
- Malformed CSV data
- Missing fields
- Invalid data types
- Checksum validation

### 3. UI Errors
- Plot update failures
- Map rendering issues
- File I/O errors

## Performance Optimization

### 1. Data Handling
- Use deque for efficient data storage
- Limit plot history to prevent memory issues
- Implement data compression for logging

### 2. UI Responsiveness
- Use threading for serial communication
- Batch UI updates to reduce refresh rate
- Implement lazy loading for historical data

## Future Enhancements

1. **Real-time 3D Visualization**: Add 3D attitude display
2. **Advanced Analytics**: Implement flight analysis algorithms
3. **Multi-CanSat Support**: Support multiple CanSats simultaneously
4. **Web Interface**: Create web-based GCS for remote access
5. **Database Integration**: Store mission data in database
6. **Automated Report Generation**: Generate mission reports automatically

## Development Timeline

- **Week 1**: Basic serial communication and data parsing
- **Week 2**: UI layout and basic plotting
- **Week 3**: Map integration and advanced features
- **Week 4**: Testing, debugging, and documentation

## Getting Started

1. Clone the repository
2. Install required dependencies
3. Configure serial port settings
4. Run the main application
5. Connect your LoRa receiver
6. Start monitoring CanSat telemetry

This comprehensive GCS will provide real-time monitoring, data logging, and analysis capabilities for your CanSat mission, matching the professional dashboard shown in your reference image.