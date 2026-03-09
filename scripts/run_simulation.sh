#!/bin/bash
# Script to run Gazebo simulation with ArduPilot SITL

echo "Starting Gazebo Simulation..."
echo "================================"

# Start Gazebo in background
echo "1. Starting Gazebo..."
gz sim -v4 -r iris_runway.sdf &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 5

# Start ArduPilot SITL
echo "2. Starting ArduPilot SITL..."
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 \
    --map --console

# Cleanup on exit
trap "kill $GAZEBO_PID 2>/dev/null" EXIT
