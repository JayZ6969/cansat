#!/bin/bash
# Satellite Imagery Setup Script for macOS/Linux

echo "================================================================"
echo "CanSat GCS - Satellite Imagery Setup"
echo "================================================================"
echo ""
echo "This script will download satellite imagery tiles for the mission area."
echo "Center: 26.720333, 84.303806"
echo "Radius: 3 km"
echo ""
echo "This may take 5-15 minutes depending on your internet connection."
echo ""
read -p "Press Enter to continue..."

echo ""
echo "Checking Python installation..."
python3 --version
if [ $? -ne 0 ]; then
    echo "ERROR: Python is not installed"
    echo "Please install Python from https://www.python.org/"
    exit 1
fi

echo ""
echo "Installing required packages..."
pip3 install requests

echo ""
echo "Starting satellite tile download..."
python3 scripts/download_satellite_tiles.py

echo ""
echo "================================================================"
echo "Setup Complete!"
echo "================================================================"
echo ""
echo "The map will now use cached satellite imagery."
echo "You can run the application offline."
echo ""
