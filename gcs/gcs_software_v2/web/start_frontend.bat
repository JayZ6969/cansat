@echo off
REM Start Next.js Frontend for CanSat GCS
REM This script starts the Next.js development server

echo ============================================
echo   CanSat GCS - Next.js Frontend Startup
echo ============================================
echo.

REM Navigate to web directory
echo [1/2] Navigating to web directory...
cd /d "%~dp0"

REM Start Next.js development server
echo [2/2] Starting Next.js development server on http://localhost:3000
echo.
echo Frontend Dashboard: http://localhost:3000
echo.
echo Press Ctrl+C to stop the server
echo ============================================
echo.

pnpm dev

pause
