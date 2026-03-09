#!/bin/bash
# Quick start script for Drone Tracking System

echo "╔════════════════════════════════════════════════════════╗"
echo "║     Drone Tracking System - Quick Start               ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Check Python version
echo "Checking Python version..."
python3 --version

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo ""
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Install dependencies
echo ""
echo "Installing Python dependencies..."
pip install -q -r requirements.txt

# Check for YOLO models
echo ""
echo "Checking YOLO models..."
if [ ! -f "models/yolov8n.pt" ]; then
    echo "⚠ YOLO model not found. It will be downloaded on first run."
else
    echo "✓ YOLO model found"
fi

# Display options
echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║ Setup Complete! Choose an option:                     ║"
echo "╠════════════════════════════════════════════════════════╣"
echo "║ 1. Run with webcam                                     ║"
echo "║ 2. Run with Gazebo simulation                          ║"
echo "║ 3. Test camera stream                                  ║"
echo "║ 4. Exit                                                ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "Starting with webcam..."
        echo "Note: Edit config/settings.py and set USE_UDP_STREAM = False"
        python main.py
        ;;
    2)
        echo ""
        echo "Starting Gazebo simulation..."
        echo "Make sure Gazebo and ArduPilot SITL are running!"
        echo "Run: scripts/run_simulation.sh in another terminal"
        sleep 2
        python main.py
        ;;
    3)
        echo ""
        echo "Testing camera stream..."
        python tests/test_camera_stream.py
        ;;
    4)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice. Exiting..."
        exit 1
        ;;
esac
