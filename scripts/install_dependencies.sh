#!/bin/bash
# Script to install system dependencies

echo "Installing system dependencies..."
echo "=================================="

# Update package list
sudo apt update

# Install Gazebo Harmonic with plugins
echo "Installing Gazebo Harmonic..."
sudo apt install -y libgz-sim8-dev rapidjson-dev

# Install GStreamer for camera streaming
echo "Installing GStreamer..."
sudo apt install -y libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad \
    gstreamer1.0-libav gstreamer1.0-gl

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

echo "=================================="
echo "Installation complete!"
echo "Next steps:"
echo "1. Clone and build ardupilot_gazebo"
echo "2. Set environment variables in ~/.bashrc"
echo "3. Run the simulation with scripts/run_simulation.sh"
