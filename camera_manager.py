"""
Camera management module
Handles video capture and frame processing
"""

import cv2
import time
from config import CameraConfig


class CameraManager:
    """Manages camera/video input"""
    
    def __init__(self, video_source=None):
        """
        Initialize camera manager
        
        Args:
            video_source: Camera index or video file path
        """
        self.video_source = video_source or CameraConfig.VIDEO_SOURCE
        self.cap = None
        self.frame_width = None
        self.frame_height = None
        self.center_x = None
        self.center_y = None
        self.fps = CameraConfig.FPS
        self.frame_count = 0
        self.start_time = time.time()
        
        self._initialize_camera()
    
    def _initialize_camera(self):
        """Initialize video capture"""
        self.cap = cv2.VideoCapture(self.video_source)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open video source: {self.video_source}")
        
        # Get frame dimensions
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Set custom dimensions if specified
        if CameraConfig.FRAME_WIDTH:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CameraConfig.FRAME_WIDTH)
            self.frame_width = CameraConfig.FRAME_WIDTH
            self.center_x = self.frame_width // 2
        
        if CameraConfig.FRAME_HEIGHT:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CameraConfig.FRAME_HEIGHT)
            self.frame_height = CameraConfig.FRAME_HEIGHT
            self.center_y = self.frame_height // 2
        
        print(f"Camera initialized: {self.frame_width}x{self.frame_height}")
    
    def read_frame(self):
        """
        Read a frame from camera
        
        Returns:
            tuple: (success, frame)
        """
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
        return ret, frame
    
    def get_fps(self):
        """Calculate current FPS"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            return self.frame_count / elapsed
        return 0
    
    def get_frame_center(self):
        """Get frame center coordinates"""
        return self.center_x, self.center_y
    
    def get_frame_dimensions(self):
        """Get frame dimensions"""
        return self.frame_width, self.frame_height
    
    def release(self):
        """Release camera resources"""
        if self.cap:
            self.cap.release()
            print("Camera released")
    
    def __del__(self):
        """Cleanup on deletion"""
        self.release()
