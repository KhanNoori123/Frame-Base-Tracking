"""
Object tracking module
Handles YOLO detection and object tracking logic
"""

import numpy as np
import cv2
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
        self.is_manual_tracking = False  # Flag for manual tracking
        
        # Lost frame handling
        self.lost_frames = 0
        self.max_lost_frames = TrackingConfig.MAX_LOST_FRAMES
        self.max_lost_frames_manual = 200  # Much longer for manual selections
        self.last_known_bbox = None
        
        # Velocity tracking
        self.prev_position = None
        self.prev_time = None
        self.prev_area = None
        
        # Optical flow tracking for manual selections
        self.prev_gray = None
        self.tracking_points = None
        
        # OpenCV tracker for manual selections (better than optical flow)
        self.opencv_tracker = None
        self.opencv_tracker_type = 'CSRT'  # CSRT is best for accuracy
        
        # Kalman filter for prediction (simple 2D position + velocity)
        self.use_kalman = TrackingConfig.USE_KALMAN_FILTER
        self.kalman = None
        self.kalman_initialized = False
        
        # Detection history for temporal consistency
        self.detection_history = []  # Store last N detections
        self.history_size = 5
        self.frame_counter = 0  # Track frame count for periodic reinitialization
        
        self._load_model()
    
    def _load_model(self):
        """Load YOLO model with GPU support"""
        import torch
        
        print(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        
        # Check for GPU availability and configure device based on config
        if TrackingConfig.USE_GPU and torch.cuda.is_available():
            self.device = 'cuda'
            self.model.to('cuda')
            print(f"GPU detected: {torch.cuda.get_device_name(0)}")
            print("Model loaded successfully (using GPU)")
        else:
            self.device = 'cpu'
            self.model.to('cpu')
            if TrackingConfig.USE_GPU and not torch.cuda.is_available():
                print("GPU requested but not available, using CPU")
            else:
                print("Model loaded successfully (using CPU)")
    
    def detect(self, frame):
        """
        Detect objects in frame with enhanced robustness
        
        Args:
            frame: Input frame
            
        Returns:
            list: List of detection dictionaries
        """
        self.frame_counter += 1  # Increment frame counter
        
        # COCO class IDs: car=2, traffic light=9
        ALLOWED_CLASSES = [2, 9]  # Only detect cars and traffic lights
        
        # Use tracker mode with optimized parameters for stability
        results = self.model.track(
            frame, 
            persist=True, 
            verbose=False, 
            conf=self.confidence_threshold,
            iou=0.4,  # Lower IOU for better re-identification
            tracker="bytetrack.yaml",  # ByteTrack for robustness
            classes=ALLOWED_CLASSES,
            device=self.device,  # Use GPU if available
            # Additional parameters for better detection
            agnostic_nms=True,  # Class-agnostic NMS
            max_det=10  # Limit detections for performance
        )
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                # Double-check class filtering
                if conf >= self.confidence_threshold and cls in ALLOWED_CLASSES:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
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
        
        # Store in history for temporal consistency
        self.detection_history.append(detections)
        if len(self.detection_history) > self.history_size:
            self.detection_history.pop(0)
        
        return detections
    
    def select_target(self, detections, frame_center, frame=None):
        """
        Select target object from detections
        
        Args:
            detections: List of detections
            frame_center: Tuple of (center_x, center_y)
            frame: Current frame (needed for optical flow tracking)
            
        Returns:
            dict: Selected detection or None
        """
        # If manual tracking is active, try hybrid approach
        if self.is_manual_tracking and frame is not None:
            # First, try to match with YOLO detections (preferred for moving objects)
            if detections and self.selected_bbox:
                matched_detection = self._match_manual_to_detection(detections)
                if matched_detection:
                    # Update bbox to YOLO detection
                    self.selected_bbox = matched_detection['bbox']
                    self.lost_frames = 0
                    # Reinitialize optical flow with new bbox
                    if self.frame_counter % 10 == 0:  # Update tracking points periodically
                        self._initialize_optical_flow(frame, self.selected_bbox)
                    return matched_detection
            
            # Fallback to optical flow if no YOLO match
            manual_result = self._track_manual_selection(frame)
            if manual_result is not None:
                return manual_result
            
            # If both fail, check if we should give up
            max_lost = self.max_lost_frames_manual
            if self.lost_frames > max_lost:
                print(f"Lost manual tracking after {max_lost} frames")
                self.reset_tracking()
            return None
        
        if not detections:
            if self.tracking_locked:
                self.lost_frames += 1
                max_lost = self.max_lost_frames_manual if self.is_manual_tracking else self.max_lost_frames
                if self.lost_frames > max_lost:
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
    
    def _match_manual_to_detection(self, detections):
        """
        Match manual selection bbox to YOLO detections
        This helps track moving objects by using YOLO when available
        
        Args:
            detections: List of YOLO detections
            
        Returns:
            Best matching detection or None
        """
        if not self.selected_bbox:
            return None
        
        best_match = None
        best_score = 0
        min_score_threshold = 0.15  # Lower threshold for manual selections
        
        for det in detections:
            # Calculate IoU
            iou = self._calculate_iou(self.selected_bbox, det['bbox'])
            
            # Calculate center distance
            distance = self._calculate_bbox_distance(self.selected_bbox, det['bbox'])
            max_distance = 150  # Allow larger movement for manual tracking
            distance_score = max(0, 1 - (distance / max_distance))
            
            # Calculate size similarity
            old_area = (self.selected_bbox[2] - self.selected_bbox[0]) * (self.selected_bbox[3] - self.selected_bbox[1])
            new_area = det['area']
            size_ratio = min(old_area, new_area) / max(old_area, new_area) if max(old_area, new_area) > 0 else 0
            
            # Combined score with emphasis on position
            score = (iou * 0.5) + (distance_score * 0.4) + (size_ratio * 0.1)
            
            if score > best_score and score > min_score_threshold:
                best_score = score
                best_match = det
        
        if best_match:
            # Mark as manual tracking but with YOLO assistance
            best_match_copy = best_match.copy()
            best_match_copy['class_name'] = f"Manual ({best_match['class_name']})"
            return best_match_copy
        
        return None
    
    def _find_locked_object(self, detections):
        """Find locked object in detections with Kalman prediction"""
        # If using Kalman filter and we have predictions
        predicted_bbox = None
        if self.use_kalman and self.kalman_initialized and self.lost_frames > 0:
            predicted_bbox = self._predict_bbox()
        
        best_match = None
        best_score = 0
        
        # Reference bbox: use prediction if available, otherwise last known
        reference_bbox = predicted_bbox if predicted_bbox else self.selected_bbox
        
        for det in detections:
            score = self._calculate_match_score(reference_bbox, det['bbox'])
            
            # If we're using prediction, be more lenient with matching
            min_score = TrackingConfig.MIN_MATCH_SCORE
            if predicted_bbox and self.lost_frames > 0:
                min_score *= 0.7  # Lower threshold when using prediction
            
            if score > best_score and score > min_score:
                best_score = score
                best_match = det
        
        if best_match:
            self.selected_bbox = best_match['bbox']
            self.last_known_bbox = best_match['bbox']
            self.lost_frames = 0
            
            # Update Kalman filter
            if self.use_kalman:
                self._update_kalman(best_match['bbox'])
            
            return best_match
        else:
            self.lost_frames += 1
            
            # Use prediction to maintain tracking
            if predicted_bbox and self.lost_frames <= self.max_lost_frames:
                print(f"Using prediction... ({self.lost_frames}/{self.max_lost_frames})")
                # Create virtual detection from prediction
                x1, y1, x2, y2 = predicted_bbox
                return {
                    'bbox': predicted_bbox,
                    'confidence': 0.5,  # Lower confidence for predicted
                    'class': -2,  # Special class for predicted
                    'class_name': 'Predicted',
                    'center_x': (x1 + x2) // 2,
                    'center_y': (y1 + y2) // 2,
                    'area': (x2 - x1) * (y2 - y1),
                    'track_id': None
                }
            elif self.lost_frames <= self.max_lost_frames:
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
            
            # Check if it's a manual selection
            if detection.get('class') == -1:
                self.is_manual_tracking = True
                print(f"Locked to: Manual Selection")
            else:
                self.is_manual_tracking = False
                print(f"Locked to: {detection['class_name']}")
    
    def _initialize_optical_flow(self, frame, bbox):
        """Initialize optical flow tracking points with dense grid"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        x1, y1, x2, y2 = bbox
        
        # Ensure bbox is within frame bounds
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(frame.shape[1], x2)
        y2 = min(frame.shape[0], y2)
        
        # Create a mask for the bbox region
        mask = np.zeros_like(gray)
        mask[y1:y2, x1:x2] = 255
        
        # Detect good features to track with more points for robustness
        points = cv2.goodFeaturesToTrack(
            gray, 
            maxCorners=200,  # More points for better tracking
            qualityLevel=0.01, 
            minDistance=5,  # Closer points for denser coverage
            mask=mask,
            blockSize=3
        )
        
        if points is None or len(points) < 10:
            # If not enough features, create a grid of points
            print("Not enough features detected, creating grid...")
            grid_points = []
            step = 8  # Smaller step for more points
            for y in range(y1 + 5, y2 - 5, step):
                for x in range(x1 + 5, x2 - 5, step):
                    grid_points.append([[float(x), float(y)]])
            
            if len(grid_points) > 0:
                points = np.array(grid_points, dtype=np.float32)
            else:
                # Bbox too small, create at least center point
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                points = np.array([[[cx, cy]]], dtype=np.float32)
                print("Warning: Bbox too small, using center point only")
        
        self.tracking_points = points
        self.prev_gray = gray
        
        # Only print if significantly different from last time
        if not hasattr(self, '_last_point_count') or abs(len(points) - self._last_point_count) > 10:
            print(f"Initialized optical flow with {len(points)} tracking points")
            self._last_point_count = len(points)
        
    def _track_manual_selection(self, frame):
        """Track manually selected region using optical flow with enhanced robustness"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Initialize tracking points if needed
        if self.tracking_points is None or self.prev_gray is None:
            if self.selected_bbox:
                self._initialize_optical_flow(frame, self.selected_bbox)
                if self.tracking_points is None or len(self.tracking_points) < 4:
                    print("Failed to initialize tracking points")
                    self.lost_frames += 1
                    return None
            else:
                return None
        
        # Calculate optical flow with better parameters
        new_points, status, error = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, self.tracking_points, None,
            winSize=(21, 21),  # Larger window for better tracking
            maxLevel=3,  # More pyramid levels
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
            flags=cv2.OPTFLOW_LK_GET_MIN_EIGENVALS,
            minEigThreshold=0.0001
        )
        
        # Select good points with error threshold
        if new_points is not None and status is not None:
            # Filter by status and error
            good_new = []
            good_old = []
            for i, (new, st, err) in enumerate(zip(new_points, status, error)):
                if st == 1 and err < 50:  # Error threshold
                    good_new.append(new[0])  # Extract the point from nested array
                    good_old.append(self.tracking_points[i][0])
            
            if len(good_new) == 0:
                self.lost_frames += 1
                if self.lost_frames > 10 and self.selected_bbox:
                    print(f"Reinitializing tracking... (lost frames: {self.lost_frames})")
                    self._initialize_optical_flow(frame, self.selected_bbox)
                    return None
                elif self.lost_frames > self.max_lost_frames_manual:
                    print("Lost manual tracking - no good points")
                    self.reset_tracking()
                    return None
                else:
                    # Return last known bbox
                    if self.selected_bbox:
                        x1, y1, x2, y2 = self.selected_bbox
                        return {
                            'bbox': self.selected_bbox,
                            'confidence': 0.5,
                            'class': -1,
                            'class_name': 'Manual (searching)',
                            'center_x': (x1 + x2) // 2,
                            'center_y': (y1 + y2) // 2,
                            'area': (x2 - x1) * (y2 - y1),
                            'track_id': None
                        }
                    return None
            
            good_new = np.array(good_new)
            good_old = np.array(good_old)
            
            # Need at least 4 points for tracking
            min_points = 4
            if len(good_new) < min_points:
                # Try to reinitialize if we still have the bbox
                self.lost_frames += 1
                if self.lost_frames > 10 and self.selected_bbox:
                    print(f"Reinitializing tracking... (lost frames: {self.lost_frames})")
                    self._initialize_optical_flow(frame, self.selected_bbox)
                    return None
                elif self.lost_frames > self.max_lost_frames_manual:
                    print("Lost manual tracking - not enough points")
                    self.reset_tracking()
                    return None
                else:
                    # Return last known bbox
                    if self.selected_bbox:
                        x1, y1, x2, y2 = self.selected_bbox
                        return {
                            'bbox': self.selected_bbox,
                            'confidence': 0.5,
                            'class': -1,
                            'class_name': 'Manual (searching)',
                            'center_x': (x1 + x2) // 2,
                            'center_y': (y1 + y2) // 2,
                            'area': (x2 - x1) * (y2 - y1),
                            'track_id': None
                        }
                    return None
            else:
                self.lost_frames = 0  # Reset lost frame counter
                
                # Calculate new bounding box from points with outlier rejection
                # Ensure good_new is 2D array with shape (N, 2)
                if good_new.ndim == 1:
                    good_new = good_new.reshape(-1, 2)
                
                x_coords = good_new[:, 0]
                y_coords = good_new[:, 1]
                
                # Remove outliers using percentiles
                x_min_percentile = np.percentile(x_coords, 5)
                x_max_percentile = np.percentile(x_coords, 95)
                y_min_percentile = np.percentile(y_coords, 5)
                y_max_percentile = np.percentile(y_coords, 95)
                
                # Filter points within percentiles
                mask = (
                    (x_coords >= x_min_percentile) & (x_coords <= x_max_percentile) &
                    (y_coords >= y_min_percentile) & (y_coords <= y_max_percentile)
                )
                
                if np.sum(mask) >= min_points:
                    filtered_x = x_coords[mask]
                    filtered_y = y_coords[mask]
                else:
                    filtered_x = x_coords
                    filtered_y = y_coords
                
                x1 = int(np.min(filtered_x))
                y1 = int(np.min(filtered_y))
                x2 = int(np.max(filtered_x))
                y2 = int(np.max(filtered_y))
                
                # Add padding
                padding = 15
                x1 = max(0, x1 - padding)
                y1 = max(0, y1 - padding)
                x2 = min(frame.shape[1], x2 + padding)
                y2 = min(frame.shape[0], y2 + padding)
                
                # Smooth bbox changes to prevent jitter
                if self.selected_bbox:
                    old_x1, old_y1, old_x2, old_y2 = self.selected_bbox
                    alpha = 0.7  # Smoothing factor
                    x1 = int(alpha * x1 + (1 - alpha) * old_x1)
                    y1 = int(alpha * y1 + (1 - alpha) * old_y1)
                    x2 = int(alpha * x2 + (1 - alpha) * old_x2)
                    y2 = int(alpha * y2 + (1 - alpha) * old_y2)
                
                self.selected_bbox = (x1, y1, x2, y2)
                self.tracking_points = good_new.reshape(-1, 1, 2)
                self.prev_gray = gray.copy()
                
                # Only refresh if points are getting low (not time-based)
                if len(self.tracking_points) < 15:
                    self._initialize_optical_flow(frame, self.selected_bbox)
                
                # Create detection-like object
                return {
                    'bbox': (x1, y1, x2, y2),
                    'confidence': 1.0,
                    'class': -1,
                    'class_name': 'Manual',
                    'center_x': (x1 + x2) // 2,
                    'center_y': (y1 + y2) // 2,
                    'area': (x2 - x1) * (y2 - y1),
                    'track_id': None
                }
        
        return None
    
    def reset_tracking(self):
        """Reset tracking state"""
        self.tracking_locked = False
        self.selected_bbox = None
        self.last_known_bbox = None
        self.lost_frames = 0
        self.prev_position = None
        self.prev_time = None
        self.prev_area = None
        self.is_manual_tracking = False
        self.tracking_points = None
        self.prev_gray = None
        self.kalman = None
        self.kalman_initialized = False
        self.detection_history = []
    
    def _init_kalman(self, bbox):
        """Initialize Kalman filter for bbox tracking"""
        # Simple 2D Kalman filter: [x, y, vx, vy]
        self.kalman = cv2.KalmanFilter(4, 2)  # 4 state vars, 2 measurement vars
        
        # State transition matrix (constant velocity model)
        self.kalman.transitionMatrix = np.array([
            [1, 0, 1, 0],  # x = x + vx
            [0, 1, 0, 1],  # y = y + vy
            [0, 0, 1, 0],  # vx = vx
            [0, 0, 0, 1]   # vy = vy
        ], dtype=np.float32)
        
        # Measurement matrix (we measure x, y)
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Process noise covariance
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
        
        # Measurement noise covariance
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 10
        
        # Error covariance
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        
        # Initialize state with bbox center
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        self.kalman.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
        
        # Store bbox size for prediction
        self.kalman_bbox_size = (bbox[2] - bbox[0], bbox[3] - bbox[1])
        
        self.kalman_initialized = True
    
    def _update_kalman(self, bbox):
        """Update Kalman filter with new measurement"""
        if not self.kalman_initialized:
            self._init_kalman(bbox)
            return
        
        # Measure center
        cx = (bbox[0] + bbox[2]) / 2
        cy = (bbox[1] + bbox[3]) / 2
        measurement = np.array([[cx], [cy]], dtype=np.float32)
        
        # Update
        self.kalman.correct(measurement)
        
        # Update bbox size (smooth update)
        w = bbox[2] - bbox[0]
        h = bbox[3] - bbox[1]
        old_w, old_h = self.kalman_bbox_size
        self.kalman_bbox_size = (0.7 * old_w + 0.3 * w, 0.7 * old_h + 0.3 * h)
    
    def _predict_bbox(self):
        """Predict bbox using Kalman filter"""
        if not self.kalman_initialized:
            return None
        
        # Predict
        prediction = self.kalman.predict()
        cx = int(prediction[0])
        cy = int(prediction[1])
        
        # Reconstruct bbox
        w, h = self.kalman_bbox_size
        x1 = int(cx - w / 2)
        y1 = int(cy - h / 2)
        x2 = int(cx + w / 2)
        y2 = int(cy + h / 2)
        
        return (x1, y1, x2, y2)
    
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
