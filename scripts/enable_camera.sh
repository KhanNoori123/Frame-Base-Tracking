#!/bin/bash
# Script to enable Gazebo camera streaming

echo "Enabling Gazebo camera stream..."
gz topic -t /world/gimbal/model/mount/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming \
    -m gz.msgs.Boolean -p "data: 1"

echo "Camera stream enabled on UDP port 5602"
echo "Run test_camera_stream.py to verify"
