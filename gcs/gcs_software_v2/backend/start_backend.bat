@echo off
REM Start FastAPI Backend for CanSat GCS
REM This script activates the virtual environment and starts the FastAPI server

echo ============================================
echo   CanSat GCS - FastAPI Backend Startup
echo ============================================
echo.

REM Activate virtual environment
echo [1/3] Activating virtual environment...
call ..\venv\Scripts\activate.bat

REM Navigate to backend directory
echo [2/3] Navigating to backend directory...
cd /d "%~dp0"

REM Start FastAPI server with Uvicorn
echo [3/3] Starting FastAPI server on http://localhost:8000
echo.
echo Backend API Docs: http://localhost:8000/docs
echo WebSocket endpoint: ws://localhost:8000/ws/telemetry
echo.
echo Press Ctrl+C to stop the server
echo ============================================
echo.

python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload

pause
