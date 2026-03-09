# Configuration Guide

This directory contains all system configuration files.

## settings.py

Main configuration file with the following sections:

### TrackingConfig
- YOLO model selection and parameters
- Tracking algorithm settings
- Kalman filter configuration

### CameraConfig
- Video source (UDP stream or webcam)
- Resolution and FPS settings
- Frame processing optimization

### DroneConfig
- MAVLink connection settings
- Control gains and limits
- Flight mode configuration

### UIConfig
- Display colors and styles
- Information overlay settings

### IBVSConfig
- Visual servoing parameters
- Target area and distance settings
- Control gains and speed limits
- Stopping algorithm parameters

## Customization

Edit `settings.py` to customize system behavior. Common adjustments:

1. **Camera Source**: Change `USE_UDP_STREAM` to switch between simulation and real camera
2. **Tracking Sensitivity**: Adjust `CONFIDENCE_THRESHOLD` for detection sensitivity
3. **Control Gains**: Tune `FORWARD_GAIN`, `YAW_GAIN` for drone responsiveness
4. **Target Distance**: Modify `TARGET_AREA` to change optimal tracking distance

## Examples

### Use Webcam Instead of UDP Stream
```python
USE_UDP_STREAM = False
VIDEO_SOURCE = 0  # Default webcam
```

### Increase Detection Confidence
```python
CONFIDENCE_THRESHOLD = 0.5  # Higher = fewer false positives
```

### Adjust Drone Speed
```python
MAX_FORWARD_SPEED = 3.0  # Slower approach
FORWARD_GAIN = 0.00005  # Less aggressive
```
