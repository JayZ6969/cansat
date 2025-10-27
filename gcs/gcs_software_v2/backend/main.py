"""
Backend WebSocket Server for CanSat GCS
Provides WebSocket API for the Next.js frontend
"""

from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_socketio import SocketIO, emit
import threading
import time
import sys
import os

# Add parent directory to path to import src modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.telemetry_handler import TelemetryHandler
from src.data_manager import DataManager

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}})
socketio = SocketIO(
    app, 
    cors_allowed_origins="*", 
    async_mode='threading',
    logger=False,
    engineio_logger=False,
    ping_timeout=60,
    ping_interval=25,
    transports=['polling', 'websocket'],
    allow_upgrades=True
)

# Global instances
telemetry_handler = TelemetryHandler()
data_manager = DataManager()
is_mission_active = False

# REST API Endpoints
@app.route('/api/ports', methods=['GET'])
def get_ports():
    """REST API endpoint to get available COM ports"""
    try:
        ports = telemetry_handler.find_available_ports()
        return jsonify({'ports': ports, 'success': True})
    except Exception as e:
        return jsonify({'error': str(e), 'success': False}), 500

@app.route('/api/start-recording', methods=['POST'])
def start_recording():
    """REST API endpoint to start CSV recording"""
    try:
        data = request.get_json() or {}
        filename = data.get('filename', 'telemetry_data.csv')
        
        # Start CSV recording in data_manager
        data_manager.start_csv_recording(filename)
        
        global is_mission_active
        is_mission_active = True
        
        print(f"[MISSION] Started recording to: {filename}")
        return jsonify({'success': True, 'filename': filename, 'message': 'Recording started'})
    except Exception as e:
        print(f"[ERROR] Failed to start recording: {e}")
        return jsonify({'error': str(e), 'success': False}), 500

@app.route('/api/stop-recording', methods=['POST'])
def stop_recording():
    """REST API endpoint to stop CSV recording"""
    try:
        # Stop CSV recording in data_manager
        data_manager.stop_csv_recording()
        
        global is_mission_active
        is_mission_active = False
        
        print(f"[MISSION] Stopped recording")
        return jsonify({'success': True, 'message': 'Recording stopped'})
    except Exception as e:
        print(f"[ERROR] Failed to stop recording: {e}")
        return jsonify({'error': str(e), 'success': False}), 500

# Socket.IO Event Handlers
@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f"[CLIENT] Client connected")
    emit('connection_status', {'status': 'connected', 'message': 'Connected to GCS backend'})

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f"Client disconnected")

@socketio.on('get_ports')
def handle_get_ports():
    """Get available COM ports"""
    try:
        ports = telemetry_handler.find_available_ports()
        emit('ports_list', {'ports': ports})
    except Exception as e:
        emit('error', {'message': f"Error getting ports: {str(e)}"})

@socketio.on('connect_serial')
def handle_connect_serial(data):
    """Connect to serial port"""
    try:
        port = data.get('port')
        baudrate = data.get('baudrate', 115200)
        
        print(f"[DEBUG] Attempting to connect to {port} at {baudrate} baud...")
        success = telemetry_handler.start_serial_connection(port, baudrate)
        if success:
            print(f"[SUCCESS] Connected to {port} at {baudrate} baud")
            emit('serial_status', {'status': 'connected', 'port': port, 'baudrate': baudrate})
        else:
            print(f"[ERROR] Failed to connect to {port}")
            emit('serial_status', {'status': 'error', 'message': f'Failed to connect to {port}. Port may be busy or unavailable.'})
    except PermissionError as e:
        error_msg = f"Permission denied accessing {port}. Port may be in use by another application."
        print(f"[ERROR] Permission error: {error_msg}")
        emit('serial_status', {'status': 'error', 'message': error_msg})
    except FileNotFoundError as e:
        error_msg = f"Port {port} not found. Please check if the device is connected."
        print(f"[ERROR] Port not found: {error_msg}")
        emit('serial_status', {'status': 'error', 'message': error_msg})
    except Exception as e:
        error_msg = f"Connection error: {str(e)}"
        print(f"[ERROR] {error_msg}")
        emit('serial_status', {'status': 'error', 'message': error_msg})

@socketio.on('disconnect_serial')
def handle_disconnect_serial():
    """Disconnect from serial port"""
    try:
        print(f"[DEBUG] Disconnect request received")
        telemetry_handler.stop_serial_connection()
        print(f"[SUCCESS] Serial connection stopped")
        emit('serial_status', {'status': 'disconnected', 'message': 'Successfully disconnected'})
    except Exception as e:
        print(f"[WARNING] Disconnect error (may be expected): {str(e)}")
        # Even if there's an error, we still want to report as disconnected
        # because the user requested a disconnect
        emit('serial_status', {'status': 'disconnected', 'message': 'Disconnected (with warnings)'})
        # Don't emit error event for disconnect operations

@socketio.on('start_mission')
def handle_start_mission():
    """Start mission data streaming"""
    global is_mission_active
    is_mission_active = True
    emit('mission_status', {'status': 'started', 'active': True})
    print("Mission started")

@socketio.on('stop_mission')
def handle_stop_mission():
    """Stop mission data streaming"""
    global is_mission_active
    is_mission_active = False
    emit('mission_status', {'status': 'stopped', 'active': False})
    print("Mission stopped")

@socketio.on_error_default
def default_error_handler(e):
    """Handle SocketIO errors"""
    print(f"[SOCKETIO ERROR] {str(e)}")
    return False

@app.errorhandler(Exception)
def handle_exception(e):
    """Handle Flask errors"""
    print(f"[FLASK ERROR] {str(e)}")
    return jsonify(error=str(e)), 500

@socketio.on('export_data')
def handle_export_data(data):
    """Export mission data"""
    try:
        format_type = data.get('format', 'csv')
        result = data_manager.export_data(format_type)
        emit('export_result', result)
    except Exception as e:
        emit('error', {'message': f"Export error: {str(e)}"})

def telemetry_broadcast_loop():
    """Background thread to broadcast telemetry data"""
    print("[TELEMETRY] Broadcast loop started...")
    packet_count = 0
    last_connection_status = False
    
    while True:
        # Monitor connection status changes
        current_connection_status = telemetry_handler.is_connected
        if current_connection_status != last_connection_status:
            status = 'connected' if current_connection_status else 'disconnected'
            print(f"[CONNECTION] Serial status changed to: {status}")
            socketio.emit('serial_status', {'status': status})
            last_connection_status = current_connection_status
        
        if telemetry_handler.is_connected:
            try:
                # Handle telemetry packets
                packet = telemetry_handler.get_latest_packet()
                if packet and packet.get('type') != 'log':  # Ensure we don't emit log messages as telemetry
                    packet_count += 1
                    
                    # Store data if mission is active
                    if is_mission_active:
                        data_manager.store_data(packet)
                    
                    # Broadcast to all connected clients
                    socketio.emit('telemetry_data', packet)
                    
                    # Log every 10th packet to avoid spam
                    if packet_count % 10 == 0:
                        print(f"[TELEMETRY] Broadcast #{packet_count}: Alt={packet.get('altitude', 0):.1f}m, Batt={packet.get('battery_percentage', 0):.0f}%, RSSI={packet.get('rssi', 'N/A')}")
                
                # Handle log messages
                log_message = telemetry_handler.get_latest_log()
                if log_message:
                    # Store log if mission is active
                    if is_mission_active:
                        data_manager.store_data(log_message)
                    
                    # Broadcast log to all connected clients
                    socketio.emit('log_message', log_message)
                    print(f"[LOG] {log_message.get('log_message', 'Unknown log')}")
                    
            except Exception as e:
                print(f"[TELEMETRY] Error in broadcast loop: {e}")
        
        time.sleep(0.1)  # 10 Hz update rate

def start_backend_server():
    """Start the backend WebSocket server"""
    print("=" * 60)
    print("Starting CanSat GCS Backend Server")
    print("=" * 60)
    print(f"WebSocket Server: http://localhost:5000")
    print(f"CORS enabled for all origins")
    print("Waiting for connections...")
    print("=" * 60)
    
    # Start telemetry broadcast thread
    broadcast_thread = threading.Thread(target=telemetry_broadcast_loop, daemon=True)
    broadcast_thread.start()
    
    # Start Flask-SocketIO server
    socketio.run(app, host='0.0.0.0', port=5000, debug=False, allow_unsafe_werkzeug=True)

if __name__ == "__main__":
    start_backend_server()
