"""
Telemetry Handler Module
Handles serial communication and telemetry data parsing for CanSat missions
"""

import serial
import serial.tools.list_ports
import threading
import time
import random
import re
from datetime import datetime

class TelemetryHandler:
    def __init__(self):
        self.serial_connection = None
        self.is_connected = False
        self.port = None
        self.baudrate = 9600
        self.reading_thread = None
        self.stop_reading = False
        self.latest_packet = None
        self.buffer = ""
        self.log_messages = []  # Queue for storing log messages
        
        # Telemetry data template (23 fields)
        self.telemetry_template = {
            'team_id': '2024-ASI-CANSAT-049',
            'timestamp': '00:00:00',
            'packet_count': 0,
            'altitude': 0.0,
            'pressure': 1013.25,
            'temperature': 25.0,
            'voltage': 7.4,
            'gnss_time': '00:00:00',
            'gnss_lat': 0.0,
            'gnss_long': 0.0,
            'gnss_alt': 0.0,
            'gnss_sats': 0,
            'gnss_speed': 0.0,
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': -9.8,
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0,
            'gyro_spin_rate': 0.0,
            'flight_state': 0,
            'servo_status': 'SERVO:CLOSED',
            'error_code': 0,
            'battery_percentage': 95,
            'mode': 'TEST_MODE'
        }
    
    def find_available_ports(self):
        """Find all available COM ports"""
        ports = []
        try:
            for port in serial.tools.list_ports.comports():
                ports.append({
                    'port': port.device,
                    'description': port.description,
                    'hwid': port.hwid
                })
        except Exception as e:
            print(f"Error finding ports: {e}")
        return ports
    
    def start_serial_connection(self, port, baudrate=9600):
        """Start serial connection to specified port"""
        self.port = port
        self.baudrate = baudrate
            
        try:
            # Close existing connection if any
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
                
            # Open new connection
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1
            )
            
            # Clear buffers
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            
            self.is_connected = True
            self.stop_reading = False
            self.buffer = ""
            
            # Start reading thread
            if self.reading_thread and self.reading_thread.is_alive():
                self.stop_reading = True
                self.reading_thread.join(timeout=2)
                
            self.reading_thread = threading.Thread(target=self._read_serial_data, daemon=True)
            self.reading_thread.start()
            
            print(f"[OK] Connected to {self.port} at {self.baudrate} baud")
            return True
            
        except serial.SerialException as e:
            print(f"[ERROR] Serial error connecting to {self.port}: {e}")
            self.is_connected = False
            return False
        except Exception as e:
            print(f"[ERROR] Error connecting to {self.port}: {e}")
            self.is_connected = False
            return False
    
    def stop_serial_connection(self):
        """Stop serial connection"""
        self.stop_reading = True
        self.is_connected = False
        
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=1)
            
        print("Serial connection stopped")
    
    def _read_serial_data(self):
        """Background thread to read serial data"""
        print("Serial reading thread started...")
        
        while not self.stop_reading and self.is_connected:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    # Read available bytes
                    if self.serial_connection.in_waiting > 0:
                        data = self.serial_connection.read(self.serial_connection.in_waiting)
                        self.buffer += data.decode('utf-8', errors='ignore')
                        
                        # Process complete lines
                        while '\n' in self.buffer:
                            line, self.buffer = self.buffer.split('\n', 1)
                            line = line.strip()
                            
                            if line:
                                print(f"Received: {line}")
                                
                                # Check if this is a log message or telemetry data
                                log_message = self.parse_log_message(line)
                                if log_message:
                                    self.latest_packet = log_message
                                    print(f"[LOG] {log_message.get('log_message', '')}")
                                else:
                                    # Try to parse as telemetry data
                                    parsed_data = self.parse_telemetry_string(line)
                                    if parsed_data:
                                        self.latest_packet = parsed_data
                                        print(f"[OK] Parsed packet #{parsed_data.get('packet_count', '?')}")
                                    
            except serial.SerialException as e:
                print(f"[ERROR] Serial error: {e}")
                self.is_connected = False
                break
            except Exception as e:
                print(f"[ERROR] Error reading serial data: {e}")
            
            time.sleep(0.05)  # Small delay to prevent CPU overload
        
        print("Serial reading thread stopped.")
    
    def parse_log_message(self, data_string):
        """
        Parse log messages from CanSat
        Log messages are identified by specific patterns like:
        - "LOG: message"
        - "[LOG] message"
        - "DEBUG: message"
        - Lines that don't match telemetry CSV format
        """
        try:
            # Check for explicit log prefixes
            log_prefixes = ['LOG:', '[LOG]', 'DEBUG:', '[DEBUG]', 'INFO:', '[INFO]', 'ERROR:', '[ERROR]', 'WARN:', '[WARN]']
            
            for prefix in log_prefixes:
                if data_string.startswith(prefix):
                    log_data = {
                        'type': 'log',
                        'log_message': data_string,
                        'timestamp': datetime.now().isoformat(),
                        'recording_time': datetime.now().isoformat()
                    }
                    # Store in log queue
                    self.log_messages.append(log_data)
                    return log_data
            
            # Check if line contains telemetry-like data (has commas and numbers)
            # If it doesn't look like CSV telemetry data, treat as log
            if ',' not in data_string or not any(char.isdigit() for char in data_string):
                # Skip empty lines and common serial noise
                if len(data_string.strip()) > 5 and not data_string.startswith('RX #'):
                    log_data = {
                        'type': 'log',
                        'log_message': data_string.strip(),
                        'timestamp': datetime.now().isoformat(),
                        'recording_time': datetime.now().isoformat()
                    }
                    # Store in log queue
                    self.log_messages.append(log_data)
                    return log_data
                    
        except Exception as e:
            print(f"[ERROR] Error parsing log message: {e}")
        
        return None
    
    def parse_telemetry_string(self, data_string):
        """
        Parse telemetry data string into structured format
        Expected format: RX #XXX @ XXXXX ms | XXX bytes | RSSI=XX dB | SNR=X.X | CSV_DATA
        or just: CSV_DATA
        
        CSV fields (22): TEAM_ID,MISSION_TIME,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,
                        GPS_TIME,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,GPS_SATS,
                        ACCEL_X,ACCEL_Y,ACCEL_Z,GYRO_X,GYRO_Y,GYRO_Z,PID_OUTPUT,
                        STATE,SERVO,ERROR_CODE,GNSS_SPEED
        """
        try:
            # Extract CSV data from the line (after the last '|' if present)
            csv_data = data_string
            if '|' in data_string:
                # Split by '|' and get the last part (which should be the CSV data)
                parts = data_string.split('|')
                csv_data = parts[-1].strip()
            
            # Now parse the CSV data
            fields = csv_data.strip().split(',')
            
            if len(fields) >= 22:
                parsed_data = {
                    'team_id': fields[0].strip(),
                    'timestamp': fields[1].strip(),  # MISSION_TIME
                    'packet_count': int(fields[2]) if self._is_number(fields[2]) else 0,
                    'altitude': float(fields[3]) if self._is_float(fields[3]) else 0.0,
                    'pressure': float(fields[4]) if self._is_float(fields[4]) else 0.0,
                    'temperature': float(fields[5]) if self._is_float(fields[5]) else 0.0,
                    'voltage': float(fields[6]) if self._is_float(fields[6]) else 0.0,
                    'gnss_time': fields[7].strip(),  # GPS_TIME
                    'gnss_lat': float(fields[8]) if self._is_float(fields[8]) else 0.0,  # GPS_LATITUDE
                    'gnss_long': float(fields[9]) if self._is_float(fields[9]) else 0.0,  # GPS_LONGITUDE
                    'gnss_alt': float(fields[10]) if self._is_float(fields[10]) else 0.0,  # GPS_ALTITUDE
                    'gnss_sats': int(fields[11]) if self._is_number(fields[11]) else 0,  # GPS_SATS
                    'accel_x': float(fields[12]) if self._is_float(fields[12]) else 0.0,
                    'accel_y': float(fields[13]) if self._is_float(fields[13]) else 0.0,
                    'accel_z': float(fields[14]) if self._is_float(fields[14]) else 0.0,
                    'gyro_x': float(fields[15]) if self._is_float(fields[15]) else 0.0,
                    'gyro_y': float(fields[16]) if self._is_float(fields[16]) else 0.0,
                    'gyro_z': float(fields[17]) if self._is_float(fields[17]) else 0.0,
                    'pid_output': float(fields[18]) if self._is_float(fields[18]) else 0.0,  # PID_OUTPUT for reaction wheel
                    'flight_state': int(fields[19]) if self._is_number(fields[19]) else 0,  # STATE
                    'servo_status': fields[20].strip(),  # SERVO
                    'error_code': int(fields[21]) if self._is_number(fields[21]) else 0,
                    'gnss_speed': float(fields[22]) if len(fields) > 22 and self._is_float(fields[22]) else 0.0
                }
                
                # Extract RSSI and SNR if present in the original string
                parsed_data['rssi'] = 0.0  # Default values
                parsed_data['snr'] = 0.0
                
                if '|' in data_string:
                    rssi_match = re.search(r'RSSI=(-?\d+\.?\d*)\s*dB', data_string)
                    snr_match = re.search(r'SNR=(-?\d+\.?\d*)', data_string)
                    
                    if rssi_match:
                        parsed_data['rssi'] = float(rssi_match.group(1))
                    if snr_match:
                        parsed_data['snr'] = float(snr_match.group(1))
                
                # Add computed fields and aliases for dashboard compatibility
                parsed_data['battery_percentage'] = self._voltage_to_percentage(parsed_data['voltage'])
                parsed_data['battery_voltage'] = parsed_data['voltage']
                parsed_data['mode'] = self._flight_state_to_mode(parsed_data['flight_state'])
                parsed_data['datetime'] = datetime.now().isoformat()
                
                # Add GPS aliases
                parsed_data['gps_latitude'] = parsed_data['gnss_lat']
                parsed_data['gps_longitude'] = parsed_data['gnss_long']
                parsed_data['gps_altitude'] = parsed_data['gnss_alt']
                parsed_data['num_satellites'] = parsed_data['gnss_sats']
                
                # Add nested objects for charts (gyroscope and accelerometer)
                parsed_data['gyroscope'] = {
                    'x': parsed_data['gyro_x'],
                    'y': parsed_data['gyro_y'],
                    'z': parsed_data['gyro_z']
                }
                
                parsed_data['accelerometer'] = {
                    'x': parsed_data['accel_x'],
                    'y': parsed_data['accel_y'],
                    'z': parsed_data['accel_z']
                }
                
                # Add reaction wheel data (PID output for chart)
                # Store as single value for time-series plotting
                parsed_data['reaction_wheel'] = parsed_data['pid_output']
                
                # Add mission time (convert timestamp to seconds if it's a number)
                try:
                    parsed_data['mission_time'] = int(parsed_data['timestamp']) if self._is_number(parsed_data['timestamp']) else 0
                except:
                    parsed_data['mission_time'] = 0
                
                # Add status fields
                parsed_data['camera_recording'] = parsed_data['flight_state'] >= 3  # Recording during flight
                parsed_data['parachute_deployed'] = parsed_data['servo_status'].upper() == 'OPEN'
                parsed_data['calibration_done'] = parsed_data['flight_state'] > 1
                parsed_data['error_indicator'] = f"Error {parsed_data['error_code']}" if parsed_data['error_code'] != 0 else "All Good"
                parsed_data['speed'] = abs(parsed_data.get('gnss_speed', 0.0))  # Use GNSS speed as speed indicator
                
                return parsed_data
            else:
                print(f"[ERROR] Invalid data: expected 23 fields, got {len(fields)}")
                
        except Exception as e:
            print(f"[ERROR] Error parsing telemetry data: {e}")
            print(f"   Raw data: {data_string}")
            
        return None
    
    def get_latest_packet(self):
        """Get the latest received telemetry packet"""
        return self.latest_packet
    
    def get_latest_log(self):
        """Get the latest received log message"""
        if self.log_messages:
            return self.log_messages.pop(0)  # Return and remove the oldest log
        return None
    
    def read_data(self):
        """Read the latest data packet (for compatibility with main.py)"""
        packet = self.latest_packet
        if packet:
            # Clear after reading to avoid duplicates
            self.latest_packet = None
        return packet
    
    def generate_test_data(self):
        """Generate test telemetry data for simulation"""
        test_data = self.telemetry_template.copy()
        
        # Add some randomization for realistic testing
        test_data.update({
            'timestamp': datetime.now().strftime('%H:%M:%S'),
            'packet_count': random.randint(1, 1000),
            'altitude': random.uniform(100, 1000),
            'pressure': random.uniform(900, 1100),
            'temperature': random.uniform(20, 35),
            'voltage': random.uniform(6.5, 8.4),
            'gnss_speed': random.uniform(0, 50),
            'accel_x': random.uniform(-1, 1),
            'accel_y': random.uniform(-1, 1),
            'accel_z': random.uniform(-10, -9),
            'gyro_x': random.uniform(-5, 5),
            'gyro_y': random.uniform(-5, 5),
            'gyro_z': random.uniform(-5, 5),
            'flight_state': random.randint(0, 7),
            'datetime': datetime.now().isoformat()
        })
        
        test_data['battery_percentage'] = self._voltage_to_percentage(test_data['voltage'])
        test_data['mode'] = self._flight_state_to_mode(test_data['flight_state'])
        
        return test_data
    
    def calibrate_sensors(self):
        """Calibrate sensors"""
        return {"status": "success", "message": "Sensors calibrated"}
    
    def _is_number(self, value):
        """Check if a string can be converted to int"""
        try:
            int(value)
            return True
        except ValueError:
            return False
    
    def _is_float(self, value):
        """Check if a string can be converted to float"""
        try:
            float(value)
            return True
        except ValueError:
            return False
    
    def _voltage_to_percentage(self, voltage):
        """Convert voltage to battery percentage (assumes 2S LiPo: 6.0V-8.4V)"""
        try:
            min_voltage = 6.0  # 2S LiPo minimum
            max_voltage = 8.4  # 2S LiPo maximum
            percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
            return max(0, min(100, int(percentage)))
        except:
            return 0
    
    def _flight_state_to_mode(self, flight_state):
        """Convert flight state number to mode string"""
        states = {
            0: "BOOT",
            1: "TEST_MODE", 
            2: "LAUNCH_PAD",
            3: "ASCENT",
            4: "ROCKET_DEPLOY",
            5: "DESCENT",
            6: "AEROBRAKE_RELEASE",
            7: "IMPACT"
        }
        return states.get(flight_state, "UNKNOWN")