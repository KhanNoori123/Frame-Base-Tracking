import cv2
import numpy as np
import time
from ultralytics import YOLO

class ObjectTracker:
    def __init__(self, video_source=0, model_path='yolov8n.pt'):
        self.cap = cv2.VideoCapture(video_source)
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Load YOLO model
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        
        # Previous position and time for velocity calculation
        self.prev_position = None
        self.prev_time = None
        self.prev_area = None
        
        # Selected object tracking
        self.selected_class = None  # Track specific class
        self.track_largest = True  # Track largest detected object
        self.selected_object_id = None  # ID of manually selected object
        self.selected_bbox = None  # Bounding box of selected object
        self.tracking_locked = False  # Lock tracking to selected object
        self.lost_frames = 0  # Counter for lost frames
        self.max_lost_frames = 15  # Maximum frames before giving up tracking
        self.last_known_bbox = None  # Last known position for prediction
        
        # Mouse callback
        cv2.namedWindow('YOLO Object Tracker')
        cv2.setMouseCallback('YOLO Object Tracker', self.mouse_callback)
        
    def detect_objects_yolo(self, frame, confidence_threshold=0.5):
        """Detect objects using YOLO"""
        # Use tracking mode for better temporal consistency
        results = self.model.track(frame, persist=True, verbose=False, conf=confidence_threshold)
        detections = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                conf = float(box.conf[0])
                if conf >= confidence_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    
                    # Get tracking ID if available
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
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks to select objects"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x = x
            self.click_y = y
            self.click_detected = True
    
    def find_clicked_object(self, detections, click_x, click_y):
        """Find which object was clicked"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                return det
        return None
    
    def calculate_iou(self, bbox1, bbox2):
        """Calculate Intersection over Union between two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i < x1_i or y2_i < y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def calculate_bbox_distance(self, bbox1, bbox2):
        """Calculate center distance between two bounding boxes"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        cx1 = (x1_1 + x2_1) / 2
        cy1 = (y1_1 + y2_1) / 2
        cx2 = (x1_2 + x2_2) / 2
        cy2 = (y1_2 + y2_2) / 2
        
        return np.sqrt((cx1 - cx2)**2 + (cy1 - cy2)**2)
    
    def select_target_object(self, detections):
        """Select which object to track from detections"""
        if not detections:
            # Increment lost frame counter
            if self.tracking_locked:
                self.lost_frames += 1
                if self.lost_frames > self.max_lost_frames:
                    print("Lost tracked object - too many frames")
                    self.tracking_locked = False
                    self.selected_bbox = None
                    self.lost_frames = 0
            return None
        
        # If tracking is locked to a selected object, find it with improved matching
        if self.tracking_locked and self.selected_bbox is not None:
            best_match = None
            best_score = 0
            
            for det in detections:
                # Calculate IoU
                iou = self.calculate_iou(self.selected_bbox, det['bbox'])
                
                # Calculate distance between centers
                distance = self.calculate_bbox_distance(self.selected_bbox, det['bbox'])
                max_distance = 200  # Maximum allowed distance in pixels
                distance_score = max(0, 1 - (distance / max_distance))
                
                # Calculate size similarity
                old_area = (self.selected_bbox[2] - self.selected_bbox[0]) * (self.selected_bbox[3] - self.selected_bbox[1])
                new_area = det['area']
                size_ratio = min(old_area, new_area) / max(old_area, new_area) if max(old_area, new_area) > 0 else 0
                
                # Combined score (weighted)
                score = (iou * 0.4) + (distance_score * 0.4) + (size_ratio * 0.2)
                
                if score > best_score and score > 0.25:  # Lower threshold for better tracking
                    best_score = score
                    best_match = det
            
            if best_match:
                self.selected_bbox = best_match['bbox']  # Update bbox
                self.last_known_bbox = best_match['bbox']
                self.lost_frames = 0  # Reset lost frame counter
                return best_match
            else:
                # Don't immediately lose tracking - keep trying for a few frames
                self.lost_frames += 1
                if self.lost_frames <= self.max_lost_frames:
                    print(f"Tracking confidence low ({self.lost_frames}/{self.max_lost_frames}), searching...")
                    return None
                else:
                    print("Lost tracked object - giving up")
                    self.tracking_locked = False
                    self.selected_bbox = None
                    self.lost_frames = 0
                    return None
        
        # Filter by selected class if specified
        if self.selected_class is not None:
            detections = [d for d in detections if d['class'] == self.selected_class]
            if not detections:
                return None
        
        # Track the largest object or closest to center
        if self.track_largest:
            return max(detections, key=lambda d: d['area'])
        else:
            # Track object closest to center
            return min(detections, key=lambda d: 
                      np.sqrt((d['center_x'] - self.center_x)**2 + 
                             (d['center_y'] - self.center_y)**2))
    
    def calculate_position_and_distance(self, detection):
        """Calculate object center position and distance from frame center"""
        cx = detection['center_x']
        cy = detection['center_y']
        area = detection['area']
        
        # Distance from center
        dx = cx - self.center_x
        dy = cy - self.center_y
        distance_2d = np.sqrt(dx**2 + dy**2)
        
        return {
            'x': cx,
            'y': cy,
            'dx': dx,
            'dy': dy,
            'distance_2d': distance_2d,
            'area': area
        }
    
    def calculate_velocity(self, current_pos, current_time):
        """Calculate velocity in X, Y, and Z (depth) axes"""
        if self.prev_position is None or self.prev_time is None:
            self.prev_position = current_pos
            self.prev_time = current_time
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        dt = current_time - self.prev_time
        if dt == 0:
            return {'vx': 0, 'vy': 0, 'vz': 0}
        
        # Velocity in pixels per second
        vx = (current_pos['dx'] - self.prev_position['dx']) / dt
        vy = (current_pos['dy'] - self.prev_position['dy']) / dt
        
        # Z-axis velocity (depth) estimated from area change
        vz = 0
        if self.prev_area is not None and self.prev_area > 0:
            area_change = (current_pos['area'] - self.prev_area) / self.prev_area
            vz = area_change / dt  # Normalized area change per second
        
        self.prev_position = current_pos
        self.prev_time = current_time
        self.prev_area = current_pos['area']
        
        return {'vx': vx, 'vy': vy, 'vz': vz}

    def draw_tracking_info(self, frame, detection, position, velocity, all_detections):
        """Draw tracking information on frame"""
        # Draw center crosshair
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                (self.center_x + 20, self.center_y), (0, 255, 0), 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                (self.center_x, self.center_y + 20), (0, 255, 0), 2)
        cv2.circle(frame, (self.center_x, self.center_y), 5, (0, 255, 0), -1)
        
        # Draw all detected objects (lighter color) only if not locked
        if not self.tracking_locked:
            for det in all_detections:
                x1, y1, x2, y2 = det['bbox']
                cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
                cv2.putText(frame, f"{det['class_name']} {det['confidence']:.2f}", 
                           (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
        
        # Draw tracked object (highlighted)
        x1, y1, x2, y2 = detection['bbox']
        box_color = (0, 255, 0) if self.tracking_locked else (0, 255, 255)
        box_thickness = 4 if self.tracking_locked else 3
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, box_thickness)
        cv2.circle(frame, (position['x'], position['y']), 7, (0, 0, 255), -1)
        
        # Draw line from center to object
        cv2.line(frame, (self.center_x, self.center_y), 
                (position['x'], position['y']), (255, 0, 0), 2)
        
        # Display information
        info_y = 30
        lock_status = " [LOCKED]" if self.tracking_locked else ""
        cv2.putText(frame, f"Tracking: {detection['class_name']} ({detection['confidence']:.2f}){lock_status}", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if self.tracking_locked else (0, 255, 255), 2)
        
        info_y += 30
        cv2.putText(frame, f"Position: ({position['x']}, {position['y']})", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"Offset X: {position['dx']:+.0f}px  Y: {position['dy']:+.0f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"Distance from center: {position['distance_2d']:.1f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity X: {velocity['vx']:+.1f} px/s", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity Y: {velocity['vy']:+.1f} px/s", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity Z: {velocity['vz']:+.3f} (depth)", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Direction indicators
        if abs(position['dx']) > 50:
            direction = "RIGHT" if position['dx'] > 0 else "LEFT"
            cv2.putText(frame, direction, (self.frame_width - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        if abs(position['dy']) > 50:
            direction = "DOWN" if position['dy'] > 0 else "UP"
            cv2.putText(frame, direction, (self.frame_width - 150, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        
        return frame

    def run(self, confidence_threshold=0.5):
        """Main tracking loop"""
        print("YOLO Object Tracker Started")
        print("Press 'q' to quit")
        print("Press 'r' to reset/unlock tracking")
        print("Press 'l' to toggle tracking mode (largest/closest to center)")
        print("Press '0-9' to filter by class ID")
        print("Press 'a' to track all classes")
        print("CLICK on any object to lock tracking to it")
        print(f"\nAvailable classes: {self.model.names}")
        
        self.click_detected = False
        self.click_x = 0
        self.click_y = 0
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            current_time = time.time()
            
            # Detect all objects with YOLO
            detections = self.detect_objects_yolo(frame, confidence_threshold)
            
            # Handle mouse click to select object
            if self.click_detected:
                clicked_obj = self.find_clicked_object(detections, self.click_x, self.click_y)
                if clicked_obj:
                    self.selected_bbox = clicked_obj['bbox']
                    self.tracking_locked = True
                    print(f"Locked tracking to: {clicked_obj['class_name']}")
                else:
                    print("No object clicked")
                self.click_detected = False
            
            # Select target object to track
            target = self.select_target_object(detections)
            
            # Process tracked object
            if target is not None:
                position = self.calculate_position_and_distance(target)
                velocity = self.calculate_velocity(position, current_time)
                frame = self.draw_tracking_info(frame, target, position, velocity, detections)
            else:
                # Reset tracking when object is lost
                self.prev_position = None
                self.prev_time = None
                self.prev_area = None
                
                # Draw center crosshair only
                cv2.line(frame, (self.center_x - 20, self.center_y), 
                        (self.center_x + 20, self.center_y), (0, 255, 0), 2)
                cv2.line(frame, (self.center_x, self.center_y - 20), 
                        (self.center_x, self.center_y + 20), (0, 255, 0), 2)
                
                # Draw all detections
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
                    cv2.putText(frame, f"{det['class_name']} {det['confidence']:.2f}", 
                               (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                
                cv2.putText(frame, "No target object - Click to select", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display tracking mode and filter info
            if not self.tracking_locked:
                mode_text = "Largest" if self.track_largest else "Closest"
                filter_text = f"Class: {self.model.names[self.selected_class]}" if self.selected_class is not None else "All"
                cv2.putText(frame, f"Mode: {mode_text} | Filter: {filter_text}", 
                           (10, self.frame_height - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(frame, "LOCKED - Press 'r' to unlock", 
                           (10, self.frame_height - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            cv2.imshow('YOLO Object Tracker', frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):
                self.tracking_locked = False
                self.selected_bbox = None
                self.prev_position = None
                self.prev_time = None
                self.prev_area = None
                print("Tracking unlocked")
            elif key == ord('l'):
                if not self.tracking_locked:
                    self.track_largest = not self.track_largest
                    print(f"Tracking mode: {'Largest' if self.track_largest else 'Closest to center'}")
            elif key == ord('a'):
                if not self.tracking_locked:
                    self.selected_class = None
                    print("Tracking all classes")
            elif ord('0') <= key <= ord('9'):
                if not self.tracking_locked:
                    class_id = key - ord('0')
                    if class_id in self.model.names:
                        self.selected_class = class_id
                        print(f"Filtering class: {self.model.names[class_id]}")
                    else:
                        print(f"Class ID {class_id} not found")
        
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Use yolov8n.pt (nano) for speed, or yolov8s.pt, yolov8m.pt for better accuracy
    tracker = ObjectTracker(video_source=0, model_path='yolov8n.pt')
    tracker.run(confidence_threshold=0.5)
