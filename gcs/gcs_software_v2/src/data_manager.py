"""
Data Manager Module
Handles data storage, processing, and export functionality
"""

import json
import csv
import pandas as pd
from datetime import datetime
import os

class DataManager:
    def __init__(self):
        self.telemetry_data = []
        self.session_start_time = None
        self.data_file_path = "data/mission_data.json"
        self.csv_file = None
        self.csv_writer = None
        self.csv_recording = False
        self.ensure_data_directory()
    
    def ensure_data_directory(self):
        """Ensure data directory exists"""
        os.makedirs("data", exist_ok=True)
    
    def store_data(self, telemetry_packet):
        """Store incoming telemetry data or log messages"""
        if not self.session_start_time:
            self.session_start_time = datetime.now()
        
        # Add session timestamp
        telemetry_packet['session_time'] = (datetime.now() - self.session_start_time).total_seconds()
        
        # Add recording time (current timestamp when data was recorded by GCS)
        if 'recording_time' not in telemetry_packet:
            telemetry_packet['recording_time'] = datetime.now().isoformat()
        
        # Store in memory
        self.telemetry_data.append(telemetry_packet)
        
        # Write to CSV if recording is active
        if self.csv_recording and self.csv_writer:
            try:
                # Handle log messages differently
                if telemetry_packet.get('type') == 'log':
                    self.write_log_to_csv(telemetry_packet)
                else:
                    self.csv_writer.writerow(telemetry_packet)
                self.csv_file.flush()  # Ensure data is written immediately
            except Exception as e:
                print(f"[ERROR] Failed to write to CSV: {e}")
        
        # Keep only last 1000 data points in memory to prevent memory issues
        if len(self.telemetry_data) > 1000:
            self.telemetry_data = self.telemetry_data[-1000:]
        
        # Optionally save to file every 10 packets
        if len(self.telemetry_data) % 10 == 0:
            self.save_session_data()
    
    def write_log_to_csv(self, log_packet):
        """Write log message as a separate line in CSV file"""
        # Create a log entry row with the log message and timestamp
        log_row = {}
        # Initialize all CSV columns with empty values
        for header in self.csv_headers:
            log_row[header] = ''
        
        # Fill in log-specific data
        log_row['recording_time'] = log_packet.get('recording_time', datetime.now().isoformat())
        log_row['team_id'] = f"LOG: {log_packet.get('log_message', 'Unknown log')}"
        log_row['mission_time'] = log_packet.get('timestamp', '')
        
        # Write the log row
        self.csv_writer.writerow(log_row)
    
    def get_latest_data(self):
        """Get the latest telemetry data"""
        if self.telemetry_data:
            return self.telemetry_data[-1]
        return None
    
    def get_data_range(self, start_index=0, count=100):
        """Get a range of telemetry data"""
        end_index = min(start_index + count, len(self.telemetry_data))
        return self.telemetry_data[start_index:end_index]
    
    def get_all_data(self):
        """Get all stored telemetry data"""
        return self.telemetry_data
    
    def get_chart_data(self, parameter, time_window=300):
        """Get data formatted for charts (last 'time_window' seconds)"""
        if not self.telemetry_data:
            return {"labels": [], "data": []}
        
        current_time = datetime.now()
        chart_data = []
        labels = []
        
        for packet in self.telemetry_data:
            packet_time = datetime.fromisoformat(packet['timestamp'].replace('Z', '+00:00'))
            time_diff = (current_time - packet_time).total_seconds()
            
            if time_diff <= time_window:
                chart_data.append(packet.get(parameter, 0))
                labels.append(packet_time.strftime('%H:%M:%S'))
        
        return {"labels": labels, "data": chart_data}
    
    def get_statistics(self):
        """Get statistical summary of the data"""
        if not self.telemetry_data:
            return {}
        
        # Convert to DataFrame for easy statistics
        df = pd.DataFrame(self.telemetry_data)
        
        numeric_columns = ['altitude', 'pressure', 'temperature', 'battery_percentage', 
                          'battery_voltage', 'speed', 'mission_time']
        
        stats = {}
        for col in numeric_columns:
            if col in df.columns:
                stats[col] = {
                    'min': float(df[col].min()),
                    'max': float(df[col].max()),
                    'mean': float(df[col].mean()),
                    'current': float(df[col].iloc[-1])
                }
        
        # Mission duration
        if self.session_start_time:
            stats['mission_duration'] = (datetime.now() - self.session_start_time).total_seconds()
        
        # Data packet count
        stats['total_packets'] = len(self.telemetry_data)
        
        return stats
    
    def export_data(self, format_type="csv", filename=None):
        """Export data to file"""
        if not self.telemetry_data:
            return {"status": "error", "message": "No data to export"}
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        try:
            if format_type.lower() == "csv":
                if not filename:
                    filename = f"data/mission_export_{timestamp}.csv"
                
                # Flatten nested data for CSV
                flattened_data = []
                for packet in self.telemetry_data:
                    flat_packet = packet.copy()
                    
                    # Flatten gyroscope data
                    if 'gyroscope' in packet:
                        flat_packet['gyro_x'] = packet['gyroscope'].get('x', 0)
                        flat_packet['gyro_y'] = packet['gyroscope'].get('y', 0)
                        flat_packet['gyro_z'] = packet['gyroscope'].get('z', 0)
                        del flat_packet['gyroscope']
                    
                    # Flatten accelerometer data
                    if 'accelerometer' in packet:
                        flat_packet['accel_x'] = packet['accelerometer'].get('x', 0)
                        flat_packet['accel_y'] = packet['accelerometer'].get('y', 0)
                        flat_packet['accel_z'] = packet['accelerometer'].get('z', 0)
                        del flat_packet['accelerometer']
                    
                    # Convert reaction wheel to string
                    if 'reaction_wheel' in packet:
                        flat_packet['reaction_wheel'] = str(packet['reaction_wheel'])
                    
                    flattened_data.append(flat_packet)
                
                df = pd.DataFrame(flattened_data)
                df.to_csv(filename, index=False)
                
            elif format_type.lower() == "json":
                if not filename:
                    filename = f"data/mission_export_{timestamp}.json"
                
                export_data = {
                    'export_timestamp': datetime.now().isoformat(),
                    'session_start': self.session_start_time.isoformat() if self.session_start_time else None,
                    'total_packets': len(self.telemetry_data),
                    'data': self.telemetry_data
                }
                
                with open(filename, 'w') as f:
                    json.dump(export_data, f, indent=2)
            
            else:
                return {"status": "error", "message": f"Unsupported format: {format_type}"}
            
            return {
                "status": "success", 
                "message": f"Data exported to {filename}",
                "filename": filename,
                "records": len(self.telemetry_data)
            }
            
        except Exception as e:
            return {"status": "error", "message": f"Export failed: {str(e)}"}
    
    def save_session_data(self):
        """Save current session data to file"""
        try:
            session_data = {
                'session_start': self.session_start_time.isoformat() if self.session_start_time else None,
                'last_update': datetime.now().isoformat(),
                'data': self.telemetry_data
            }
            
            with open(self.data_file_path, 'w') as f:
                json.dump(session_data, f, indent=2)
                
        except Exception as e:
            print(f"Failed to save session data: {e}")
    
    def load_session_data(self):
        """Load previous session data"""
        try:
            if os.path.exists(self.data_file_path):
                with open(self.data_file_path, 'r') as f:
                    session_data = json.load(f)
                
                self.telemetry_data = session_data.get('data', [])
                session_start_str = session_data.get('session_start')
                if session_start_str:
                    self.session_start_time = datetime.fromisoformat(session_start_str)
                
                return len(self.telemetry_data)
                
        except Exception as e:
            print(f"Failed to load session data: {e}")
        
        return 0
    
    def reset_data(self):
        """Reset all stored data"""
        self.telemetry_data = []
        self.session_start_time = None
        
        # Remove session file
        if os.path.exists(self.data_file_path):
            try:
                os.remove(self.data_file_path)
            except Exception as e:
                print(f"Failed to remove session file: {e}")
    
    def get_mission_summary(self):
        """Get mission summary information"""
        if not self.telemetry_data:
            return {}
        
        summary = {
            'total_packets': len(self.telemetry_data),
            'session_duration': 0,
            'max_altitude': 0,
            'min_altitude': float('inf'),
            'max_speed': 0,
            'battery_used': 0,
            'distance_traveled': 0
        }
        
        if self.session_start_time:
            summary['session_duration'] = (datetime.now() - self.session_start_time).total_seconds()
        
        # Calculate statistics
        for packet in self.telemetry_data:
            altitude = packet.get('altitude', 0)
            speed = packet.get('speed', 0)
            
            summary['max_altitude'] = max(summary['max_altitude'], altitude)
            summary['min_altitude'] = min(summary['min_altitude'], altitude)
            summary['max_speed'] = max(summary['max_speed'], speed)
        
        # Battery usage (assuming started at 100%)
        if self.telemetry_data:
            current_battery = self.telemetry_data[-1].get('battery_percentage', 100)
            summary['battery_used'] = 100 - current_battery
        
        return summary
    
    def start_csv_recording(self, filename):
        """Start recording telemetry data to CSV file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
            
            # Open CSV file for writing
            self.csv_file = open(filename, 'w', newline='')
            
            # Define CSV headers to match the CanSat telemetry format exactly
            fieldnames = [
                'team_id', 'mission_time', 'packet_count', 'altitude', 'pressure', 'temperature', 
                'battery_voltage', 'gnss_time', 'gnss_lat', 'gnss_long', 'gnss_alt', 'gnss_sats',
                'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'pid_output',
                'flight_state', 'servo_status', 'error_code', 'gnss_speed', 'recording_time',
                'rssi', 'snr'
            ]
            
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=fieldnames, extrasaction='ignore')
            self.csv_writer.writeheader()
            self.csv_file.flush()
            
            self.csv_recording = True
            print(f"[DATA MANAGER] Started CSV recording to: {filename}")
            
        except Exception as e:
            print(f"[ERROR] Failed to start CSV recording: {e}")
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
            raise
    
    def stop_csv_recording(self):
        """Stop recording telemetry data to CSV file"""
        try:
            self.csv_recording = False
            
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None
                print("[DATA MANAGER] Stopped CSV recording")
            
        except Exception as e:
            print(f"[ERROR] Failed to stop CSV recording: {e}")
            raise
