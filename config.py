"""
Configuration module for drone tracking system
"""

class TrackingConfig:
    """Configuration for object tracking"""
    # YOLO Model
    MODEL_PATH = 'yolov8n.pt'
    CONFIDENCE_THRESHOLD = 0.25  # Lowered for detecting simple 3D models
    
    # Tracking parameters
    MAX_LOST_FRAMES = 30  # Increased from 15 for better persistence
    IOU_WEIGHT = 0.5  # Increased IOU importance
    DISTANCE_WEIGHT = 0.3
    SIZE_WEIGHT = 0.2
    MIN_MATCH_SCORE = 0.3  # Increased from 0.25 for more stable matching
    MAX_TRACKING_DISTANCE = 250  # Increased from 200 pixels
    
    # Detection
    TRACK_LARGEST = True
    SELECTED_CLASS = None  # None = all classes


class CameraConfig:
    """Configuration for camera/video input"""
    # UDP Camera Stream Configuration
    USE_UDP_STREAM = True  # Set to False to use regular webcam
    UDP_HOST = "127.0.0.1"
    UDP_PORT = 5600
    
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
