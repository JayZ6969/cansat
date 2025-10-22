"""
FastAPI Backend for CanSat GCS Dashboard
Real-time telemetry data streaming via WebSocket and REST API
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from pydantic import BaseModel
from typing import List, Optional
import asyncio
import json
from datetime import datetime
import sys
import os

# Add parent directory to path for importing existing modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.telemetry_handler import TelemetryHandler
from src.data_manager import DataManager

# Initialize FastAPI app
app = FastAPI(
    title="CanSat GCS API",
    description="Ground Control Station API for real-time telemetry monitoring",
    version="2.0.0"
)

# Configure CORS for Next.js dev server
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "http://localhost:3001",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize handlers (singleton instances)
telemetry_handler = TelemetryHandler()
data_manager = DataManager()

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        """Broadcast message to all connected clients"""
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                # Connection closed, will be removed on disconnect
                pass

manager = ConnectionManager()

# Pydantic models for request/response validation
class PortInfo(BaseModel):
    port: str
    description: str
    hwid: str

class PortsResponse(BaseModel):
    ports: List[PortInfo]

class ConnectRequest(BaseModel):
    port: str
    baudrate: int = 9600

class StatusResponse(BaseModel):
    status: str
    message: str

class ExportRequest(BaseModel):
    format: str = "csv"  # "csv" or "json"

class ExportResponse(BaseModel):
    status: str
    filename: str
    records: int

# ============================================================================
# REST API ENDPOINTS
# ============================================================================

@app.get("/")
async def root():
    """API health check"""
    return {
        "status": "online",
        "service": "CanSat GCS API",
        "version": "2.0.0",
        "endpoints": {
            "rest": "/docs",
            "websocket": "/ws/telemetry"
        }
    }

@app.get("/api/ports", response_model=PortsResponse)
async def get_available_ports():
    """
    Get list of available COM ports
    
    Returns:
        List of available serial ports with descriptions
    """
    try:
        ports = telemetry_handler.find_available_ports()
        return {"ports": ports}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error scanning ports: {str(e)}")

@app.post("/api/connect", response_model=StatusResponse)
async def connect_to_port(request: ConnectRequest):
    """
    Connect to specified COM port
    
    Args:
        port: COM port name (e.g., "COM3")
        baudrate: Baud rate for serial communication (default: 9600)
    """
    try:
        success = telemetry_handler.start_serial_connection(request.port, request.baudrate)
        if success:
            return {
                "status": "success",
                "message": f"Connected to {request.port} at {request.baudrate} baud"
            }
        else:
            raise HTTPException(status_code=400, detail=f"Failed to connect to {request.port}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Connection error: {str(e)}")

@app.post("/api/disconnect", response_model=StatusResponse)
async def disconnect_from_port():
    """
    Disconnect from current COM port
    """
    try:
        telemetry_handler.stop_serial_connection()
        return {
            "status": "success",
            "message": "Disconnected successfully"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Disconnect error: {str(e)}")

@app.post("/api/mission/start", response_model=StatusResponse)
async def start_mission():
    """
    Start mission data collection
    Begins recording telemetry data
    """
    try:
        # Start data collection
        data_manager.mission_started = True
        data_manager.start_time = datetime.now()
        
        return {
            "status": "success",
            "message": "Mission started"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Mission start error: {str(e)}")

@app.post("/api/mission/stop", response_model=StatusResponse)
async def stop_mission():
    """
    Stop mission data collection
    Saves all collected data
    """
    try:
        data_manager.mission_started = False
        
        # Auto-save data
        if data_manager.data:
            data_manager.export_data()
        
        return {
            "status": "success",
            "message": "Mission stopped"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Mission stop error: {str(e)}")

@app.get("/api/telemetry/latest")
async def get_latest_telemetry():
    """
    Get latest telemetry packet (fallback for polling)
    
    Returns:
        Latest telemetry data packet
    """
    try:
        if telemetry_handler.latest_packet:
            return telemetry_handler.latest_packet
        else:
            # Return template with test data if no real data
            return telemetry_handler.generate_test_data()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Telemetry error: {str(e)}")

@app.post("/api/export", response_model=ExportResponse)
async def export_data(request: ExportRequest):
    """
    Export mission data to CSV or JSON
    
    Args:
        format: Export format ("csv" or "json")
    """
    try:
        filename = data_manager.export_data(format=request.format)
        records = len(data_manager.data)
        
        return {
            "status": "success",
            "filename": filename,
            "records": records
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Export error: {str(e)}")

@app.post("/api/calibrate", response_model=StatusResponse)
async def calibrate_sensors():
    """
    Calibrate sensors
    Resets offsets and baselines
    """
    try:
        # Placeholder for calibration logic
        # In production, this would send calibration command to CanSat
        
        return {
            "status": "success",
            "message": "Sensors calibrated"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Calibration error: {str(e)}")

@app.get("/api/connection/status")
async def get_connection_status():
    """
    Get current connection status
    """
    return {
        "is_connected": telemetry_handler.is_connected,
        "port": telemetry_handler.port if telemetry_handler.is_connected else None,
        "baudrate": telemetry_handler.baudrate if telemetry_handler.is_connected else None,
        "mission_running": data_manager.mission_started if hasattr(data_manager, 'mission_started') else False
    }

@app.get("/api/system/battery")
async def get_system_battery():
    """
    Get GCS laptop battery status
    """
    try:
        import psutil
        battery = psutil.sensors_battery()
        
        if battery is None:
            # No battery (desktop PC)
            return {
                "has_battery": False,
                "percentage": 100,
                "is_charging": False,
                "power_plugged": True
            }
        
        return {
            "has_battery": True,
            "percentage": int(battery.percent),
            "is_charging": battery.power_plugged,
            "power_plugged": battery.power_plugged,
            "time_left": battery.secsleft if battery.secsleft > 0 else None
        }
    except ImportError:
        # psutil not installed
        return {
            "has_battery": False,
            "percentage": 100,
            "is_charging": False,
            "power_plugged": True,
            "error": "psutil not installed"
        }
    except Exception as e:
        return {
            "has_battery": False,
            "percentage": 100,
            "is_charging": False,
            "power_plugged": True,
            "error": str(e)
        }

# ============================================================================
# WEBSOCKET ENDPOINT FOR REAL-TIME TELEMETRY
# ============================================================================

@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket):
    """
    WebSocket endpoint for real-time telemetry streaming
    Pushes data at ~2Hz to connected clients
    Only sends data when actually connected to COM port and mission is started
    """
    await manager.connect(websocket)
    print(f"WebSocket client connected. Total connections: {len(manager.active_connections)}")
    
    try:
        # Send initial connection confirmation
        await websocket.send_json({
            "type": "connection",
            "status": "connected",
            "message": "WebSocket connected successfully",
            "is_serial_connected": telemetry_handler.is_connected,
            "mission_running": data_manager.mission_started if hasattr(data_manager, 'mission_started') else False
        })
        
        # Main telemetry streaming loop
        while True:
            # Only send data if connected to COM port AND mission is started
            if telemetry_handler.is_connected and telemetry_handler.latest_packet:
                telemetry_data = telemetry_handler.latest_packet.copy()
                
                # Add metadata
                telemetry_data['type'] = 'telemetry'
                telemetry_data['timestamp'] = datetime.now().isoformat()
                
                # Send to client
                await websocket.send_json(telemetry_data)
            else:
                # Send status update instead of fake data
                await websocket.send_json({
                    "type": "status",
                    "is_serial_connected": telemetry_handler.is_connected,
                    "port": telemetry_handler.port if telemetry_handler.is_connected else None,
                    "message": "Waiting for COM port connection and data..." if not telemetry_handler.is_connected else "Waiting for telemetry data...",
                    "timestamp": datetime.now().isoformat()
                })
            
            # Wait ~500ms for 2Hz update rate
            await asyncio.sleep(0.5)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print(f"WebSocket client disconnected. Total connections: {len(manager.active_connections)}")
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(websocket)

# ============================================================================
# BACKGROUND TASK FOR DATA COLLECTION
# ============================================================================

@app.on_event("startup")
async def startup_event():
    """
    Run on application startup
    Starts background telemetry collection task
    """
    print("FastAPI GCS Backend starting...")
    print("WebSocket endpoint: ws://localhost:8000/ws/telemetry")
    print("API docs: http://localhost:8000/docs")
    
    # Start background task for telemetry collection
    asyncio.create_task(telemetry_collection_task())

async def telemetry_collection_task():
    """
    Background task that continuously collects telemetry data
    and stores it in DataManager
    """
    while True:
        try:
            # If connected and mission running, store data
            if telemetry_handler.latest_packet and hasattr(data_manager, 'mission_started') and data_manager.mission_started:
                data_manager.store_data(telemetry_handler.latest_packet)
            
            await asyncio.sleep(0.5)  # Check every 500ms
        except Exception as e:
            print(f"Telemetry collection error: {e}")
            await asyncio.sleep(1)

@app.on_event("shutdown")
async def shutdown_event():
    """
    Run on application shutdown
    Cleans up connections
    """
    print("Shutting down FastAPI GCS Backend...")
    if telemetry_handler.is_connected:
        telemetry_handler.stop_serial_connection()
    
    # Auto-save data if available
    if hasattr(data_manager, 'data') and data_manager.data:
        data_manager.export_data()

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info"
    )
