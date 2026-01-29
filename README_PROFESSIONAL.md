# Professional Drone Tracking System

A modular, production-ready drone tracking system using YOLOv8 object detection and MAVLink drone control.

## Architecture

The system is built with a clean, modular architecture separating concerns:

```
primary/
├── main.py                 # Main application entry point
├── config.py              # Centralized configuration
├── camera_manager.py      # Camera/video input handling
├── tracker.py             # Object detection and tracking logic
├── drone_interface.py     # MAVLink drone communication
├── ui_renderer.py         # User interface rendering
└── requirements.txt       # Python dependencies
```

## Features

### Object Tracking
- **YOLOv8 Integration**: State-of-the-art object detection
- **Persistent Tracking**: Built-in YOLO tracking for temporal consistency
- **Multi-factor Matching**: IoU, distance, and size-based object matching
- **Lost Frame Tolerance**: Handles temporary occlusions (15 frames)
- **Click-to-Lock**: Manual object selection with robust tracking

### Drone Control
- **MAVLink Protocol**: Industry-standard drone communication
- **Dual-axis Control**: Yaw (rotation) and Z (altitude) control
- **Hybrid Control**: Velocity commands + RC override for optimal performance
- **Safety Features**: Deadzone, rate limiting, control enable/disable
- **Real-time Feedback**: Live position and velocity metrics

### User Interface
- **Professional Display**: Clean, informative overlay
- **Real-time Metrics**: FPS, position, velocity, distance
- **Visual Feedback**: Color-coded status indicators
- **Interactive**: Click-to-select, keyboard controls

## Module Documentation

### config.py
Centralized configuration for all system parameters:
- `TrackingConfig`: YOLO model, confidence, tracking parameters
- `CameraConfig`: Video source, resolution, FPS
- `DroneConfig`: Connection, control gains, deadzones
- `UIConfig`: Display colors, window settings

### camera_manager.py
Handles video capture and frame management:
- `CameraManager`: Video input abstraction
- Frame dimension management
- FPS calculation
- Resource cleanup

### tracker.py
Core object detection and tracking:
- `ObjectTracker`: YOLO-based detection
- Multi-factor object matching
- Lost frame handling
- Velocity calculation
- Click-to-lock functionality

### drone_interface.py
MAVLink drone communication:
- `DroneInterface`: Drone control abstraction
- Connection management
- Mode and arm/disarm control
- Velocity command generation
- RC override for yaw control

### ui_renderer.py
User interface rendering:
- `UIRenderer`: Frame rendering and display
- Mouse event handling
- Status overlays
- Visual feedback

### main.py
Main application orchestration:
- `DroneTrackingSystem`: System coordinator
- Component initialization
- Main processing loop
- Keyboard input handling
- Resource cleanup

## Configuration

Edit `config.py` to customize system behavior:

```python
# Tracking
TrackingConfig.CONFIDENCE_THRESHOLD = 0.35
TrackingConfig.MAX_LOST_FRAMES = 15

# Drone Control
DroneConfig.YAW_GAIN = 0.05
DroneConfig.Z_GAIN = 0.005
DroneConfig.CONNECTION_STRING = 'udp:172.30.144.1:14551'

# Camera
CameraConfig.VIDEO_SOURCE = 0  # Webcam
```

## Usage

### Basic Usage
```bash
python main.py
```

### With Custom Configuration
Edit `config.py` before running, or create a custom config:

```python
from config import DroneConfig
DroneConfig.CONNECTION_STRING = 'tcp:127.0.0.1:5760'
```

### Controls
- **SPACE**: Enable/disable drone control
- **Click**: Lock tracking to object
- **r**: Reset tracking
- **i**: Invert yaw direction
- **k**: Invert Z direction
- **g**: Set GUIDED mode
- **l**: Toggle tracking mode
- **q**: Quit

## Installation

```bash
pip install -r requirements.txt
```

## System Requirements

- Python 3.8+
- OpenCV 4.8+
- PyTorch 2.0+
- Ultralytics YOLOv8
- PyMAVLink 2.4+

## Drone Setup

### SITL (Simulation)
```bash
sim_vehicle.py -v ArduCopter --console --map
```

### MAVProxy Connection
```bash
mavproxy.py --master=udp:172.30.144.1:14550 --out=udp:172.30.144.1:14551
```

### Required Commands
```
mode GUIDED
arm throttle
```

## Performance

- **Detection**: 30+ FPS (YOLOv8n on GPU)
- **Tracking**: Robust to 15 frame occlusions
- **Control**: 10Hz command rate
- **Latency**: <100ms end-to-end

## Safety Features

1. **Control Enable/Disable**: Manual activation required
2. **Deadzone**: Prevents jitter near center
3. **Rate Limiting**: Maximum yaw/Z velocity limits
4. **Auto-stop**: Stops on target loss
5. **Mode Checking**: Verifies GUIDED mode
6. **Arm Status**: Checks drone armed state

## Troubleshooting

### No Detections
- Lower `CONFIDENCE_THRESHOLD` in config
- Check camera connection
- Verify lighting conditions

### Drone Not Responding
- Verify GUIDED mode: `mode GUIDED`
- Check armed status: `arm throttle`
- Verify connection string
- Check MAVProxy output

### Tracking Lost Easily
- Increase `MAX_LOST_FRAMES`
- Lower `MIN_MATCH_SCORE`
- Increase `MAX_TRACKING_DISTANCE`

## Development

### Adding New Features
1. Create new module in appropriate file
2. Add configuration to `config.py`
3. Integrate in `main.py`
4. Update documentation

### Testing
```bash
# Test camera
python -c "from camera_manager import CameraManager; cam = CameraManager(); print('OK')"

# Test tracker
python -c "from tracker import ObjectTracker; tracker = ObjectTracker(); print('OK')"

# Test drone connection
python -c "from drone_interface import DroneInterface; drone = DroneInterface(); print('OK')"
```

## License

Professional Drone Tracking System
Copyright (c) 2024

## Support

For issues, questions, or contributions, please refer to the project documentation.
