# Project Structure

```
drone-tracking-system/
│
├── 📁 config/                      # Configuration Management
│   ├── __init__.py
│   ├── settings.py                 # Main configuration file
│   └── README.md                   # Configuration guide
│
├── 📁 camera/                      # Camera Management
│   ├── __init__.py
│   └── camera_manager.py          # Camera/video stream management
│
├── 📁 tracking/                    # Object Detection & Tracking
│   ├── __init__.py
│   ├── tracker.py                 # Main tracking logic
│   └── object_tracker.py          # YOLO detection wrapper
│
├── 📁 drone/                       # Drone Control Systems
│   ├── __init__.py
│   ├── drone_interface.py         # MAVLink interface
│   ├── drone_controller.py        # High-level control
│   └── ibvs_controller.py         # IBVS algorithm
│
├── 📁 interface/                   # User Interface
│   ├── __init__.py
│   └── ui_renderer.py             # OpenCV visualization
│
├── 📁 simulation/                  # Gazebo Integration
│   ├── __init__.py
│   └── gazebo_camera_on.py        # Camera stream enabler
│
├── 📁 models/                      # YOLO Model Weights
│   ├── __init__.py
│   ├── .gitkeep
│   ├── yolov8n.pt                 # Nano model (fastest)
│   └── yolov8s.pt                 # Small model
│
├── 📁 tests/                       # Test Scripts
│   ├── __init__.py
│   └── test_camera_stream.py      # Camera stream testing
│
├── 📁 docs/                        # Documentation
│   ├── __init__.py
│   ├── README.md                  # Gazebo setup guide
│   ├── README_DRONE.md            # Drone setup
│   ├── README_PROFESSIONAL.md     # Professional features
│   ├── GPU_SETUP.md               # GPU configuration
│   ├── CAMERA_TROUBLESHOOTING.md  # Camera issues
│   ├── DETECTION_IMPROVEMENTS.md  # Detection tuning
│   ├── QUICK_TUNING_GUIDE.md      # Quick reference
│   ├── STOPPING_ALGORITHMS.md     # Control algorithms
│   └── gazebo_ardupilot_sitl_setup.docx
│
├── 📁 scripts/                     # Utility Scripts
│   ├── install_dependencies.sh    # System setup
│   ├── run_simulation.sh          # Launch simulation
│   ├── enable_camera.sh           # Enable Gazebo camera
│   └── quick_start.sh             # Interactive launcher
│
├── 📁 gazebo_cars/                 # Gazebo Car Models
│   ├── models/                    # 3D car models (15 car models)
│   │   ├── car_008/
│   │   ├── car_012/
│   │   ├── car_018/
│   │   ├── car_019/
│   │   ├── car_046/
│   │   ├── car_121/
│   │   ├── car_144/
│   │   ├── car_158/
│   │   ├── car_199/
│   │   ├── car_beetle/
│   │   ├── car_golf/
│   │   ├── car_lexus/
│   │   ├── car_opel/
│   │   ├── car_polo/
│   │   └── car_volvo/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── LICENSE
│   └── README.md
│
├── 📄 main.py                      # Main Entry Point
├── 📄 requirements.txt             # Python Dependencies
├── 📄 setup.py                     # Package Setup
├── 📄 LICENSE                      # MIT License
├── 📄 README.md                    # Main Documentation
├── 📄 CONTRIBUTING.md              # Contribution Guide
├── 📄 PROJECT_STRUCTURE.md         # This file
├── 📄 STRUCTURE_SUMMARY.md         # Restructuring summary
└── 📄 .gitignore                   # Git ignore rules
```

## Module Descriptions

### Configuration

**config/settings.py**
- System-wide configuration
- Tracking, camera, drone, UI, and IBVS settings
- Easy customization point

### Camera Management

**camera/camera_manager.py**
- Video stream management (UDP/webcam)
- Frame capture and processing
- FPS calculation

### Tracking System

**tracking/tracker.py**
- Object tracking logic
- Target selection algorithms
- Kalman filtering
- Lock-on tracking

**tracking/object_tracker.py**
- YOLO detection wrapper
- Bounding box processing
- Class filtering

### Drone Control System

**drone/drone_interface.py**
- MAVLink communication
- Velocity commands
- Mode switching
- Arming/disarming

**drone/ibvs_controller.py**
- Image-Based Visual Servoing
- Position error calculation
- Control law implementation
- Stopping algorithms

**drone/drone_controller.py**
- High-level drone control
- Command rate limiting
- Safety checks

### User Interface

**interface/ui_renderer.py**
- OpenCV visualization
- Overlay rendering
- Mouse interaction
- Status display

### Simulation

**simulation/gazebo_camera_on.py**
- Gazebo camera enabler
- UDP stream configuration
- Topic management

## Data Flow

```
Camera → Detection → Tracking → IBVS → Drone Control
   ↓         ↓          ↓         ↓         ↓
  UDP    YOLOv8    Kalman    Control    MAVLink
Stream  Detection  Filter     Law      Commands
```

## Key Features by Module

### Tracking
- Multi-object detection
- Lock-on tracking
- Kalman prediction
- IOU-based matching

### Control
- IBVS control law
- Predictive braking
- Safety limits
- Mode management

### UI
- Real-time metrics
- Click-to-track
- Status overlays
- FPS display

### Simulation
- Gazebo integration
- ArduPilot SITL
- Camera streaming
- Model support

## Configuration Hierarchy

```
config/settings.py
    ├── TrackingConfig    (YOLO, tracking params)
    ├── CameraConfig      (Video source, resolution)
    ├── DroneConfig       (MAVLink, control gains)
    ├── UIConfig          (Display settings)
    └── IBVSConfig        (Visual servoing params)
```

## Entry Points

1. **main.py** - Main application
2. **tests/test_camera_stream.py** - Camera testing
3. **scripts/run_simulation.sh** - Simulation launcher
4. **scripts/enable_camera.sh** - Camera enabler

## Dependencies

- **OpenCV**: Video processing and UI
- **Ultralytics**: YOLOv8 detection
- **PyTorch**: Neural network backend
- **PyMAVLink**: Drone communication
- **NumPy**: Numerical operations
