@echo off
echo Setting up Ground Control Station...
echo.

REM Check if Python is installed
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7+ from https://python.org
    pause
    exit /b 1
)

echo Python found. Creating virtual environment...
python -m venv venv

echo Activating virtual environment...
call venv\Scripts\activate.bat

echo Installing dependencies...
pip install -r requirements.txt

echo.
echo Setup complete!
echo.
echo To run the application:
echo 1. Run: setup.bat (this file) - only needed once
echo 2. Run: run.bat - to start the application
echo.
echo Or manually:
echo 1. venv\Scripts\activate.bat
echo 2. python main.py
echo.
pause