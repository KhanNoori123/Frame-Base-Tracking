"""
Configuration module for drone tracking system
"""

class TrackingConfig:
    """Configuration for object tracking"""
    # YOLO Model
    MODEL_PATH = 'yolov8n.pt'
    CONFIDENCE_THRESHOLD = 0.35
    
    # Tracking parameters
    MAX_LOST_FRAMES = 15
    IOU_WEIGHT = 0.4
    DISTANCE_WEIGHT = 0.4
    SIZE_WEIGHT = 0.2
    MIN_MATCH_SCORE = 0.25
    MAX_TRACKING_DISTANCE = 200  # pixels
    
    # Detection
    TRACK_LARGEST = True
    SELECTED_CLASS = None  # None = all classes


class CameraConfig:
    """Configuration for camera/video input"""
    VIDEO_SOURCE = 0  # 0 for webcam, or path to video file
    FRAME_WIDTH = None  # None = use default
    FRAME_HEIGHT = None  # None = use default
    FPS = 30


class DroneConfig:
    """Configuration for drone control"""
    # Connection
    CONNECTION_STRING = 'udp:172.30.144.1:14551'
    
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
