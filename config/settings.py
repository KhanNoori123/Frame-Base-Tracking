"""
Configuration module for drone tracking system
"""

class TrackingConfig:
    """Configuration for object tracking"""
    # YOLO Model
    MODEL_PATH = 'models/yolov8n.pt'
    CONFIDENCE_THRESHOLD = 0.20  # Lower threshold for better detection
    USE_GPU = True  # Enable GPU acceleration if available (CUDA)
    
    # Tracking parameters - IMPROVED FOR ROBUSTNESS
    MAX_LOST_FRAMES = 50  # Much longer persistence (was 30)
    IOU_WEIGHT = 0.6  # Higher IOU importance for better matching
    DISTANCE_WEIGHT = 0.25
    SIZE_WEIGHT = 0.15
    MIN_MATCH_SCORE = 0.20  # Lower threshold to maintain tracking (was 0.3)
    MAX_TRACKING_DISTANCE = 300  # Allow larger movement between frames
    
    # Detection enhancement
    TRACK_LARGEST = True
    SELECTED_CLASS = None  # None = all classes
    
    # Temporal smoothing
    USE_KALMAN_FILTER = True  # Enable Kalman filtering for prediction
    PREDICTION_WEIGHT = 0.3  # Weight for predicted position when lost


class CameraConfig:
    """Configuration for camera/video input"""
    # UDP Camera Stream Configuration
    USE_UDP_STREAM = True  # Set to False to use regular webcam
    UDP_HOST = "127.0.0.1"
    UDP_PORT = 5602
    
    # Fallback video source (used when USE_UDP_STREAM = False)
    VIDEO_SOURCE = 0  # 0 for webcam, or path to video file
    
    FRAME_WIDTH = None  # None = use default
    FRAME_HEIGHT = None  # None = use default
    FPS = 30
    
    # Stream optimization
    SKIP_FRAMES = 1  # Process every Nth frame (1 = process all, 2 = skip every other frame)


class DroneConfig:
    """Configuration for drone control"""
    # Connection
    CONNECTION_STRING = 'udpin:0.0.0.0:14550'
    
    # Control parameters
    YAW_GAIN = 0.05
    Z_GAIN = 0.005
    MAX_YAW_RATE = 45  # degrees/second
    MAX_Z_VELOCITY = 1.0  # m/s
    
    # Deadzones
    YAW_DEADZONE = 20  # pixels
    Z_DEADZONE = 20  # pixels
    
    # Control rate
    COMMAND_RATE = 0.1  # seconds (10Hz)
    
    # Flight mode
    DEFAULT_MODE = 'GUIDED'
    AUTO_ARM = False


class UIConfig:
    """Configuration for user interface"""
    WINDOW_NAME = 'Professional Drone Tracker'
    
    # Colors (BGR)
    COLOR_CENTER_CROSSHAIR = (0, 255, 0)
    COLOR_TRACKED_OBJECT = (0, 255, 0)
    COLOR_DETECTED_OBJECT = (100, 100, 100)
    COLOR_LOCKED_OBJECT = (0, 255, 0)
    COLOR_TEXT = (255, 255, 255)
    COLOR_WARNING = (0, 0, 255)
    
    # Display
    SHOW_ALL_DETECTIONS = True
    SHOW_FPS = True
    SHOW_TRACKING_INFO = True


class IBVSConfig:
    """Configuration for Image-Based Visual Servoing"""
    # Target object size in image (pixels squared)
    TARGET_AREA = 15000   # Optimal distance - target to maintain (reduced from 80000)
    MIN_AREA = 1000       # Too far - move forward fast (reduced from 5000)
    MAX_AREA = 30000      # Too close - stop or back up (reduced from 150000)
    AREA_DEADZONE = 2000  # Area tolerance for "optimal" distance (reduced from 8000)
    
    # Control gains
    FORWARD_GAIN = 0.00008   # Forward velocity gain - INCREASED for faster response
    YAW_GAIN = 0.05          # Yaw rate gain (deg/s per pixel)
    ALTITUDE_GAIN = 0.003    # Altitude velocity gain (m/s per pixel)
    
    # Speed limits - TUNED FOR FASTER APPROACH
    MAX_FORWARD_SPEED = 4.0   # Maximum forward speed (m/s) - INCREASED from 2.5
    MIN_FORWARD_SPEED = 0.2   # Minimum forward speed (m/s) - lower for final approach
    MAX_YAW_RATE = 45         # Maximum yaw rate (deg/s)
    MAX_ALTITUDE_SPEED = 0.5  # Maximum altitude change speed (m/s)
    
    # Deadzones
    YAW_DEADZONE = 20         # Horizontal centering deadzone (pixels)
    ALTITUDE_DEADZONE = 20    # Vertical centering deadzone (pixels)
    
    # Stopping algorithm parameters
    STOP_CONFIRMATION_FRAMES = 5  # Frames to confirm at target (hysteresis)
    PREDICTIVE_BRAKING_THRESHOLD = 10  # Frames ahead to start braking
    SAFETY_BUFFER_RATIO = 0.7  # Start extra slowing at 70% of target area
    
    # Control mode
    USE_ALTITUDE_CONTROL = False  # Disable by default - toggle with 'z' key
    USE_IBVS_CONTROL = True      # Use IBVS instead of basic control
