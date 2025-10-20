@echo off
echo Starting Ground Control Station...
echo.

REM Activate virtual environment if it exists
if exist venv\Scripts\activate.bat (
    call venv\Scripts\activate.bat
    echo Virtual environment activated.
) else (
    echo Warning: Virtual environment not found. Run setup.bat first.
)

echo.
echo Launching GCS Application...
echo The dashboard will open in your web browser automatically.
echo Press Ctrl+C to stop the application.
echo.

python main.py

pause