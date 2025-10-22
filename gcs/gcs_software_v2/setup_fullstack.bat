@echo off
REM Complete Setup Script for CanSat GCS Dashboard
REM This script installs all dependencies for both backend and frontend

echo ============================================
echo   CanSat GCS - Complete Setup
echo ============================================
echo.

echo This will install:
echo   - Backend: FastAPI dependencies in virtual environment
echo   - Frontend: Next.js dependencies using pnpm
echo.
pause

REM ============================================
REM Part 1: Backend Setup
REM ============================================

echo.
echo ============================================
echo [BACKEND SETUP]
echo ============================================
echo.

REM Check if venv exists
if not exist "venv\" (
    echo [1/3] Creating virtual environment...
    python -m venv venv
) else (
    echo [1/3] Virtual environment already exists, skipping...
)

REM Activate venv
echo [2/3] Activating virtual environment...
call venv\Scripts\activate.bat

REM Install backend dependencies
echo [3/3] Installing FastAPI backend dependencies...
cd backend
pip install -r requirements.txt
cd ..

echo.
echo ✓ Backend setup complete!
echo.

REM ============================================
REM Part 2: Frontend Setup
REM ============================================

echo.
echo ============================================
echo [FRONTEND SETUP]
echo ============================================
echo.

REM Check if pnpm is installed
where pnpm >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: pnpm is not installed!
    echo.
    echo Please install pnpm first:
    echo   npm install -g pnpm
    echo.
    echo Then run this script again.
    pause
    exit /b 1
)

echo [1/2] Installing Next.js dependencies with pnpm...
cd web
pnpm install
cd ..

echo.
echo ✓ Frontend setup complete!
echo.

REM ============================================
REM Setup Complete
REM ============================================

echo.
echo ============================================
echo   SETUP COMPLETE!
echo ============================================
echo.
echo Next steps:
echo.
echo 1. Start the backend (in one terminal):
echo    cd backend
echo    start_backend.bat
echo.
echo 2. Start the frontend (in another terminal):
echo    cd web
echo    start_frontend.bat
echo.
echo 3. Open your browser:
echo    http://localhost:3000
echo.
echo ============================================
echo.

pause
