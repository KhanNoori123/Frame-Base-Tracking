"""
UI rendering module
Handles all visual display and user interface
"""

import cv2
from config import UIConfig


class UIRenderer:
    """Professional UI renderer"""
    
    def __init__(self, frame_width, frame_height):
        """
        Initialize UI renderer
        
        Args:
            frame_width: Frame width in pixels
            frame_height: Frame height in pixels
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2
        
        # Mouse callback state
        self.click_x = 0
        self.click_y = 0
        self.click_detected = False
        
        # Create window
        cv2.namedWindow(UIConfig.WINDOW_NAME)
        cv2.setMouseCallback(UIConfig.WINDOW_NAME, self._mouse_callback)
    
    def _mouse_callback(self, event, x, y, flags, param):
        """Handle mouse events"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_x = x
            self.click_y = y
            self.click_detected = True
    
    def get_click(self):
        """Get and reset click state"""
        if self.click_detected:
            self.click_detected = False
            return self.click_x, self.click_y
        return None
    
    def draw_frame(self, frame, target, position, velocity, all_detections, 
                   tracking_locked, control_enabled, fps=0):
        """
        Draw complete UI on frame
        
        Args:
            frame: Input frame
            target: Target detection dict
            position: Position metrics dict
            velocity: Velocity metrics dict
            all_detections: List of all detections
            tracking_locked: Boolean tracking lock state
            control_enabled: Boolean control state
            fps: Current FPS
            
        Returns:
            Rendered frame
        """
        # Draw center crosshair
        self._draw_crosshair(frame)
        
        if target is not None:
            # Draw all detections (if enabled)
            if UIConfig.SHOW_ALL_DETECTIONS and not tracking_locked:
                self._draw_all_detections(frame, all_detections)
            
            # Draw tracked object
            self._draw_tracked_object(frame, target, position, tracking_locked)
            
            # Draw tracking info
            if UIConfig.SHOW_TRACKING_INFO:
                self._draw_tracking_info(frame, target, position, velocity, tracking_locked)
        else:
            # No target
            if UIConfig.SHOW_ALL_DETECTIONS:
                self._draw_all_detections(frame, all_detections)
            self._draw_no_target_message(frame)
        
        # Draw status bar
        self._draw_status_bar(frame, control_enabled, tracking_locked, fps)
        
        return frame
    
    def _draw_crosshair(self, frame):
        """Draw center crosshair"""
        color = UIConfig.COLOR_CENTER_CROSSHAIR
        cv2.line(frame, (self.center_x - 20, self.center_y), 
                (self.center_x + 20, self.center_y), color, 2)
        cv2.line(frame, (self.center_x, self.center_y - 20), 
                (self.center_x, self.center_y + 20), color, 2)
        cv2.circle(frame, (self.center_x, self.center_y), 5, color, -1)
    
    def _draw_all_detections(self, frame, detections):
        """Draw all detected objects"""
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(frame, (x1, y1), (x2, y2), UIConfig.COLOR_DETECTED_OBJECT, 1)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(frame, label, (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, UIConfig.COLOR_DETECTED_OBJECT, 1)
    
    def _draw_tracked_object(self, frame, target, position, locked):
        """Draw tracked object"""
        x1, y1, x2, y2 = target['bbox']
        color = UIConfig.COLOR_LOCKED_OBJECT if locked else UIConfig.COLOR_TRACKED_OBJECT
        thickness = 4 if locked else 3
        
        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        
        # Draw center point
        cv2.circle(frame, (position['x'], position['y']), 7, (0, 0, 255), -1)
        
        # Draw line to center
        cv2.line(frame, (self.center_x, self.center_y), 
                (position['x'], position['y']), (255, 0, 0), 2)
    
    def _draw_tracking_info(self, frame, target, position, velocity, locked):
        """Draw tracking information"""
        info_y = 30
        lock_status = " [LOCKED]" if locked else ""
        
        # Object info
        text = f"Tracking: {target['class_name']} ({target['confidence']:.2f}){lock_status}"
        cv2.putText(frame, text, (10, info_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, UIConfig.COLOR_LOCKED_OBJECT if locked else (0, 255, 255), 2)
        
        info_y += 30
        cv2.putText(frame, f"Position: ({position['x']}, {position['y']})", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        info_y += 25
        cv2.putText(frame, f"Offset X: {position['dx']:+.0f}px  Y: {position['dy']:+.0f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        info_y += 25
        cv2.putText(frame, f"Distance: {position['distance_2d']:.1f}px", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity X: {velocity['vx']:+.1f} px/s", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity Y: {velocity['vy']:+.1f} px/s", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        info_y += 25
        cv2.putText(frame, f"Velocity Z: {velocity['vz']:+.3f} (depth)", 
                   (10, info_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, UIConfig.COLOR_TEXT, 2)
        
        # Direction indicators
        if abs(position['dx']) > 50:
            direction = "RIGHT" if position['dx'] > 0 else "LEFT"
            cv2.putText(frame, direction, (self.frame_width - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, UIConfig.COLOR_WARNING, 2)
        
        if abs(position['dy']) > 50:
            direction = "DOWN" if position['dy'] > 0 else "UP"
            cv2.putText(frame, direction, (self.frame_width - 150, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, UIConfig.COLOR_WARNING, 2)
    
    def _draw_no_target_message(self, frame):
        """Draw no target message"""
        cv2.putText(frame, "No target - Click to select", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, UIConfig.COLOR_WARNING, 2)
    
    def _draw_status_bar(self, frame, control_enabled, tracking_locked, fps):
        """Draw status bar at bottom"""
        y_pos = self.frame_height - 10
        
        # Control status
        if not tracking_locked:
            status = "ENABLED" if control_enabled else "DISABLED"
            color = (0, 255, 0) if control_enabled else (0, 0, 255)
            cv2.putText(frame, f"Control: {status}", 
                       (self.frame_width - 250, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        else:
            cv2.putText(frame, "LOCKED - Press 'r' to unlock", 
                       (10, y_pos), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # FPS
        if UIConfig.SHOW_FPS and fps > 0:
            cv2.putText(frame, f"FPS: {fps:.1f}", 
                       (self.frame_width - 120, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    
    def show(self, frame):
        """Display frame"""
        cv2.imshow(UIConfig.WINDOW_NAME, frame)
    
    def wait_key(self, delay=1):
        """Wait for key press"""
        return cv2.waitKey(delay) & 0xFF
    
    def destroy(self):
        """Cleanup UI"""
        cv2.destroyAllWindows()
