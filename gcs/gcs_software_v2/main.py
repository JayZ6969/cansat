"""
Main GCS Application Entry Point
Ground Control Station for CanSat missions
Receives telemetry data from serial port and displays it
"""

import eel
import threading
import time
from datetime import datetime
from src.telemetry_handler import TelemetryHandler
from src.data_manager import DataManager

class GCSApp:
    def __init__(self):
        self.telemetry_handler = TelemetryHandler()
        self.data_manager = DataManager()
        self.is_running = False
        
        # Initialize Eel
        eel.init('web')
        
        # Setup Eel exposed functions
        self.setup_eel_functions()
    
    def setup_eel_functions(self):
        """Setup all Eel exposed functions for frontend communication"""
        
        @eel.expose
        def get_available_ports():
            """Get list of available COM ports"""
            try:
                ports = self.telemetry_handler.find_available_ports()
                return ports
            except Exception as e:
                print(f"Error getting available ports: {e}")
                return []
        
        @eel.expose
        def connect_to_port(port, baudrate=9600):
            """Connect to specified COM port"""
            try:
                success = self.telemetry_handler.start_serial_connection(port, baudrate)
                if success:
                    return {"status": "success", "message": f"Connected to {port}"}
                else:
                    return {"status": "error", "message": f"Failed to connect to {port}"}
            except Exception as e:
                return {"status": "error", "message": f"Connection error: {str(e)}"}
        
        @eel.expose
        def disconnect_from_port():
            """Disconnect from current COM port"""
            try:
                self.telemetry_handler.stop_serial_connection()
                return {"status": "success", "message": "Disconnected successfully"}
            except Exception as e:
                return {"status": "error", "message": f"Disconnect error: {str(e)}"}
        
        @eel.expose
        def start_mission():
            """Start mission and begin data display"""
            self.is_running = True
            return {"status": "success", "message": "Mission started"}
        
        @eel.expose
        def stop_mission():
            """Stop mission and end data display"""
            self.is_running = False
            return {"status": "success", "message": "Mission stopped"}
        
        @eel.expose
        def export_data(format_type="csv"):
            """Export mission data"""
            return self.data_manager.export_data(format_type)
    
    def data_update_loop(self):
        """Main loop for reading and updating data from serial port"""
        print("Data update loop started...")
        print("Waiting for serial connection and mission start...")
        
        while True:
            if self.is_running:
                # Only read from serial port
                if self.telemetry_handler.is_connected:
                    packet = self.telemetry_handler.get_latest_packet()
                    if packet:
                        # Store and display data
                        self.data_manager.store_data(packet)
                        try:
                            eel.update_dashboard(packet)
                        except Exception as e:
                            print(f"Error updating dashboard: {e}")
                else:
                    print("⚠ No serial connection. Please connect to a COM port.")
                    time.sleep(2)  # Wait before checking again
            
            time.sleep(0.5)  # Update every 0.5 seconds
    
    def run(self):
        """Start the GCS application"""
        print("Starting Ground Control Station...")
        
        # Start background data update thread
        data_thread = threading.Thread(target=self.data_update_loop, daemon=True)
        data_thread.start()
        
        # Start Eel web application
        try:
            eel.start('index.html', size=(1400, 900), port=8080)
        except Exception as e:
            print(f"Error starting application: {e}")

if __name__ == "__main__":
    app = GCSApp()
    app.run()