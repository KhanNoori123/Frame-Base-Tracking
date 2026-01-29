"""
Object tracking module
Handles YOLO detection and object tracking logic
"""

import numpy as np
from ultralytics import YOLO
from config import TrackingConfig


class ObjectTracker:
    """Professional object tracker using YOLO"""
    
    def __init__(self, model_path=None):
        """
        Initialize object tracker
        
        Args:
            model_path: Path to YOLO model
        """
        self.model_path = model_path or TrackingConfig.MODEL_PATH
        self.model = None
        self.confidence_threshold = TrackingConfig.CONFIDENCE_THRESHOLD
        
        # Tracking state
        self.tracking_locked = False
        self.selected_bbox = None
        self.selected_class = TrackingConfig.SELECTED_CLASS
        self.track_largest = TrackingConfig.TRACK_LARGEST
        
        # Lost frame handling
        self.lost_frames = 0
        self.max_lost_frames = TrackingConfig.MAX_LOST_FRAMES
        self.last_known_bbox = None
        
        # Velocity tracking
        self.prev_position = None
        self.prev_time = None
        self.prev_area = None
        
        self._load_model()
    
    def _load_model(self):
        """Load YOLO model"""
        print(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        print("Model loaded successfully")
    
    def detect(self, frame):
        """
        Detect objects in frame
        
        Args:
            frame: Input frame
            
        Returns:
            list: List of detection dictionaries
        """
        results = self.model.track(frame, persist=True, verbose=False, 
                                   conf=self.confidence_threshold)
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf >= self.confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    track_id = int(box.id[0]) if box.id is not None else None
                    
                    detections.append({
                        'bbox': (x1, y1, x2, y2),
                        'confidence': conf,
                        'class': cls,
                        'class_name': class_name,
                        'center_x': (x1 + x2) // 2,
                        'center_y': (y1 + y2) // 2,
                        'area': (x2 - x1) * (y2 - y1),
                        'track_id': track_id
                    })
        
        return detections
    
    def select_target(self, detections, frame_center):
        """
        Select target object from detections
        
        Args:
            detections: List of detections
            frame_center: Tuple of (center_x, center_y)
            
        Returns:
            dict: Selected detection or None
        """
        if not detections:
            if self.tracking_locked:
                self.lost_frames += 1
                if self.lost_frames > self.max_lost_frames:
                    print("Lost tracked object - timeout")
                    self.reset_tracking()
            return None
        
        # Locked tracking mode
        if self.tracking_locked and self.selected_bbox is not None:
            return self._find_locked_object(detections)
        
        # Filter by class if specified
        if self.selected_class is not None:
            detections = [d for d in detections if d['class'] == self.selected_class]
            if not detections:
                return None
        
        # Select based on mode
        if self.track_largest:
            return max(detections, key=lambda d: d['area'])
        else:
            center_x, center_y = frame_center
            return min(detections, key=lambda d: 
                      np.sqrt((d['center_x'] - center_x)**2 + 
                             (d['center_y'] - center_y)**2))
    
    def _find_locked_object(self, detections):
        """Find locked object in detections"""
        best_match = None
        best_score = 0
        
        for det in detections:
            score = self._calculate_match_score(self.selected_bbox, det['bbox'])
            
            if score > best_score and score > TrackingConfig.MIN_MATCH_SCORE:
                best_score = score
                best_match = det
        
        if best_match:
            self.selected_bbox = best_match['bbox']
            self.last_known_bbox = best_match['bbox']
            self.lost_frames = 0
            return best_match
        else:
            self.lost_frames += 1
            if self.lost_frames <= self.max_lost_frames:
                print(f"Searching... ({self.lost_frames}/{self.max_lost_frames})")
                return None
            else:
                print("Lost tracked object")
                self.reset_tracking()
                return None
    
    def _calculate_match_score(self, bbox1, bbox2):
        """Calculate matching score between two bboxes"""
        iou = self._calculate_iou(bbox1, bbox2)
        distance = self._calculate_bbox_distance(bbox1, bbox2)
        distance_score = max(0, 1 - (distance / TrackingConfig.MAX_TRACKING_DISTANCE))
        
        old_area = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        new_area = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        size_ratio = min(old_area, new_area) / max(old_area, new_area) if max(old_area, new_area) > 0 else 0
        
        score = (iou * TrackingConfig.IOU_WEIGHT + 
                distance_score * TrackingConfig.DISTANCE_WEIGHT + 
                size_ratio * TrackingConfig.SIZE_WEIGHT)
        
        return score
    
    @staticmethod
    def _calculate_iou(bbox1, bbox2):
        """Calculate Intersection over Union"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i < x1_i or y2_i < y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    @staticmethod
    def _calculate_bbox_distance(bbox1, bbox2):
        """Calculate center distance between bboxes"""
        cx1 = (bbox1[0] + bbox1[2]) / 2
        cy1 = (bbox1[1] + bbox1[3]) / 2
        cx2 = (bbox2[0] + bbox2[2]) / 2
        cy2 = (bbox2[1] + bbox2[3]) / 2
        
        return np.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)
    
    def lock_to_object(self, detection):
        """Lock tracking to specific object"""
        if detection:
            self.tracking_locked = True
            self.selected_bbox = detection['bbox']
            self.last_known_bbox = detection['bbox']
            self.lost_frames = 0
            print(f"Locked to: {detection['class_name']}")
    
    def reset_tracking(self):
        """Reset tracking state"""
        self.tracking_locked = False
        self.selected_bbox = None
        self.last_known_bbox = None
        self.lost_frames = 0
        self.prev_position = None
        self.prev_time = None
        self.prev_area = None
    
    def find_clicked_object(self, detections, click_x, click_y):
        """Find object at click position"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                return det
        return None
    
    def calculate_position_metrics(self, detection, frame_center):
        """Calculate position and distance metrics"""
        cx = detection['center_x']
        cy = detection['center_y']
        area = detection['area']
        
        center_x, center_y = frame_center
        dx = cx - center_x
        dy = cy - center_y
        distance_2d = np.sqrt(dx**2 + dy**2)
        
        return {
            'x': cx,
            'y': cy,
            'dx': dx,
            'dy': dy,
            'distance_2d': distance_2d,
            'area': area
        }
    
    def calculate_velocity(self, position, current_time):
        """Calculate velocity metrics"""
        if self.prev_position is None or self.prev_time is None:
            self.prev_position = position
            self.prev_time = current_time
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        dt = current_time - self.prev_time
        if dt == 0:
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        vx = (position['dx'] - self.prev_position['dx']) / dt
        vy = (position['dy'] - self.prev_position['dy']) / dt
        
        vz = 0
        if self.prev_area is not None and self.prev_area > 0:
            area_change = (position['area'] - self.prev_area) / self.prev_area
            vz = area_change / dt
        
        self.prev_position = position
        self.prev_time = current_time
        self.prev_area = position['area']
        
        return {'vx': vx, 'vy': vy, 'vz': vz}
