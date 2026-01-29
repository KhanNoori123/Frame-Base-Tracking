import time
import math
from pymavlink import mavutil
from object_tracker import ObjectTracker

class DroneController:
    def __init__(self, connection_string='udp:172.30.144.1:14551'):
        """Initialize drone connection"""
        print(f"Connecting to drone at {connection_string}...")
        self.master = mavutil.mavlink_connection(connection_string)
        
        # Wait for heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Heartbeat from system (system {self.master.target_system} component {self.master.target_component})")
        
        # Control parameters
        self.yaw_gain = 0.05  # Yaw control gain (increased for better response)
        self.z_gain = 0.005   # Z velocity gain (adjust for sensitivity)
        self.max_yaw_rate = 45  # Max yaw rate in degrees/sec (increased)
        self.max_z_velocity = 1.0  # Max vertical velocity in m/s
        
        # Deadzone to prevent jitter
        self.yaw_deadzone = 20  # pixels (reduced for more responsive)
        self.z_deadzone = 20    # pixels
        
        # Invert controls if needed
        self.invert_yaw = False  # Set to True if yaw direction is reversed
        self.invert_z = False    # Set to True if Z direction is reversed
        
        # Control mode
        self.control_enabled = False
        
    def set_velocity(self, vx=0, vy=0, vz=0, yaw_rate=0):
        """Send velocity and yaw commands to drone"""
        # Send Z velocity using velocity command
        type_mask = 0b0000_1111_1111_0111  # Use velocity only
        
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,  # positions (ignored)
            vx, vy, vz,  # velocities
            0, 0, 0,  # accelerations (ignored)
            0, 0  # yaw, yaw_rate (ignored)
        )
        
        # Send yaw using RC override (channel 4 only, leave throttle to velocity control)
        self.set_rc_yaw_only(yaw_rate)
    
    def set_rc_yaw_only(self, yaw_rate_deg):
        """Send yaw command via RC override, leave other channels alone"""
        # Yaw (channel 4)
        yaw_center = 1500
        yaw_range = 400
        
        if abs(yaw_rate_deg) < 0.1:
            yaw_pwm = yaw_center
        else:
            yaw_pwm = int(yaw_center + (yaw_rate_deg / self.max_yaw_rate) * yaw_range)
            yaw_pwm = max(1100, min(1900, yaw_pwm))
        
        # Send RC override - use 65535 for channels we don't want to override
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535,  # roll - don't override
            65535,  # pitch - don't override
            65535,  # throttle - don't override (let velocity control handle it)
            yaw_pwm,  # yaw - override this
            65535, 65535, 65535, 65535  # other channels - don't override
        )
    
    def send_ned_velocity(self, vx, vy, vz, yaw_rate):
        """Alternative velocity command using different message"""
        msg = self.master.mav.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED frame
            0b0000110111000111,  # type_mask (ignore position, use velocity and yaw_rate)
            0, 0, 0,  # x, y, z positions (not used)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not used)
            0, 0)    # yaw, yaw_rate (set separately)
        self.master.mav.send(msg)
    
    def calculate_control_commands(self, position_data):
        """Calculate yaw rate and Z velocity based on object position"""
        if position_data is None:
            return 0, 0
        
        dx = position_data['dx']  # Horizontal offset from center (positive = right)
        dy = position_data['dy']  # Vertical offset from center (positive = down)
        
        # Calculate yaw rate to center the object
        # If object is to the RIGHT (dx > 0), turn RIGHT (positive yaw)
        # If object is to the LEFT (dx < 0), turn LEFT (negative yaw)
        if abs(dx) > self.yaw_deadzone:
            yaw_rate = dx * self.yaw_gain
            if self.invert_yaw:
                yaw_rate = -yaw_rate
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        else:
            yaw_rate = 0
        
        # Calculate Z velocity (NED frame: positive = down, negative = up)
        # If object is BELOW center (dy > 0), descend (positive z)
        # If object is ABOVE center (dy < 0), ascend (negative z)
        if abs(dy) > self.z_deadzone:
            z_velocity = dy * self.z_gain
            if self.invert_z:
                z_velocity = -z_velocity
            z_velocity = max(-self.max_z_velocity, min(self.max_z_velocity, z_velocity))
        else:
            z_velocity = 0
        
        return yaw_rate, z_velocity
    
    def arm_drone(self):
        """Arm the drone"""
        print("Arming drone...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Drone armed!")
    
    def disarm_drone(self):
        """Disarm the drone"""
        print("Disarming drone...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("Drone disarmed!")
    
    def set_mode(self, mode):
        """Set flight mode"""
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mode set to {mode}")
        
        # Wait for mode change confirmation
        time.sleep(0.5)
        
    def get_mode(self):
        """Get current flight mode"""
        # Request heartbeat to get current mode
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            return mode
        return "UNKNOWN"

    def run_with_tracker(self, video_source=0, model_path='yolov8n.pt'):
        """Run object tracker with drone control"""
        tracker = ObjectTracker(video_source, model_path)
        
        print("\n=== Drone Object Tracking Controller ===")
        print("Controls:")
        print("  SPACE - Enable/Disable drone control")
        print("  q - Quit")
        print("  r - Reset tracking")
        print("  i - Invert yaw direction")
        print("  k - Invert Z direction")
        print("  g - Set GUIDED mode")
        print("  Click - Lock to specific object")
        print("\nDrone will control:")
        print("  YAW - Based on horizontal offset (X)")
        print("  Z VELOCITY - Based on vertical offset (Y)")
        
        # Check current mode
        current_mode = self.get_mode()
        print(f"\nCurrent flight mode: {current_mode}")
        if current_mode != "GUIDED":
            print("WARNING: Drone is not in GUIDED mode. Velocity commands may not work.")
            print("Press 'g' to set GUIDED mode or use MAVProxy: 'mode GUIDED'")
        
        # Check if armed
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            print(f"Drone armed: {'YES' if armed else 'NO'}")
            if not armed:
                print("WARNING: Drone is not armed. Commands will not execute.")
                print("Arm in MAVProxy with: 'arm throttle'")
        
        print("\nWaiting for object detection...")
        
        import cv2
        import numpy as np
        
        tracker.click_detected = False
        tracker.click_x = 0
        tracker.click_y = 0
        
        # Store last command for continuous sending
        last_yaw = 0
        last_z = 0
        last_command_time = time.time()
        command_rate = 0.1  # Send commands every 100ms
        
        # Lower confidence threshold for better detection
        confidence_threshold = 0.35
        
        while True:
            ret, frame = tracker.cap.read()
            if not ret:
                break
            
            current_time = time.time()
            
            # Detect objects
            detections = tracker.detect_objects_yolo(frame, confidence_threshold)
            
            # Handle mouse click
            if tracker.click_detected:
                clicked_obj = tracker.find_clicked_object(detections, tracker.click_x, tracker.click_y)
                if clicked_obj:
                    tracker.selected_bbox = clicked_obj['bbox']
                    tracker.tracking_locked = True
                    print(f"Locked tracking to: {clicked_obj['class_name']}")
                tracker.click_detected = False
            
            # Select target
            target = tracker.select_target_object(detections)
            
            yaw_rate = 0
            z_velocity = 0
            
            if target is not None:
                position = tracker.calculate_position_and_distance(target)
                velocity = tracker.calculate_velocity(position, current_time)
                frame = tracker.draw_tracking_info(frame, target, position, velocity, detections)
                
                # Calculate drone control commands
                yaw_rate, z_velocity = self.calculate_control_commands(position)
                last_yaw = yaw_rate
                last_z = z_velocity
                
                # Send commands if control is enabled
                if self.control_enabled:
                    # Use combined velocity command for both yaw and Z
                    self.set_velocity(vx=0, vy=0, vz=z_velocity, yaw_rate=yaw_rate)
                    last_command_time = current_time
                    
                    # Debug output
                    print(f"dx={position['dx']:+4.0f}px, dy={position['dy']:+4.0f}px | Yaw={yaw_rate:+5.1f}°/s ({math.radians(yaw_rate):+.3f}rad/s), Z={z_velocity:+.2f}m/s")
                    
                    # Display control commands on frame
                    cv2.putText(frame, f"Offset X: {position['dx']:+.0f}px", 
                               (10, tracker.frame_height - 100), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    cv2.putText(frame, f"YAW RATE: {yaw_rate:+.1f} deg/s", 
                               (10, tracker.frame_height - 70), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f"Z VEL: {z_velocity:+.2f} m/s", 
                               (10, tracker.frame_height - 40), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                # No target - reset tracking
                tracker.prev_position = None
                tracker.prev_time = None
                tracker.prev_area = None
                
                # Stop drone if control enabled
                if self.control_enabled:
                    self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)
                    last_yaw = 0
                    last_z = 0
                
                # Draw center crosshair
                cv2.line(frame, (tracker.center_x - 20, tracker.center_y), 
                        (tracker.center_x + 20, tracker.center_y), (0, 255, 0), 2)
                cv2.line(frame, (tracker.center_x, tracker.center_y - 20), 
                        (tracker.center_x, tracker.center_y + 20), (0, 255, 0), 2)
                
                # Draw all detections
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (100, 100, 100), 1)
                    cv2.putText(frame, f"{det['class_name']} {det['confidence']:.2f}", 
                               (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                
                cv2.putText(frame, "No target - Click to select", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Resend last command periodically to maintain control
            if self.control_enabled and (current_time - last_command_time) > command_rate:
                self.set_velocity(vx=0, vy=0, vz=last_z, yaw_rate=last_yaw)
                last_command_time = current_time
            
            # Display control status
            control_status = "ENABLED" if self.control_enabled else "DISABLED"
            control_color = (0, 255, 0) if self.control_enabled else (0, 0, 255)
            cv2.putText(frame, f"DRONE CONTROL: {control_status}", 
                       (tracker.frame_width - 350, tracker.frame_height - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, control_color, 2)
            
            cv2.imshow('YOLO Object Tracker', frame)
            
            # Handle keyboard
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # Stop drone before quitting
                if self.control_enabled:
                    self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)
                break
            elif key == ord(' '):  # Space bar
                self.control_enabled = not self.control_enabled
                if self.control_enabled:
                    print("Drone control ENABLED")
                else:
                    print("Drone control DISABLED")
                    self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)
            elif key == ord('r'):
                tracker.tracking_locked = False
                tracker.selected_bbox = None
                tracker.prev_position = None
                tracker.prev_time = None
                tracker.prev_area = None
                print("Tracking unlocked")
            elif key == ord('i'):
                self.invert_yaw = not self.invert_yaw
                print(f"Yaw direction: {'INVERTED' if self.invert_yaw else 'NORMAL'}")
            elif key == ord('k'):
                self.invert_z = not self.invert_z
                print(f"Z direction: {'INVERTED' if self.invert_z else 'NORMAL'}")
            elif key == ord('g'):
                print("Setting mode to GUIDED...")
                self.set_mode('GUIDED')
                current_mode = self.get_mode()
                print(f"Current mode: {current_mode}")
            elif key == ord('l'):
                if not tracker.tracking_locked:
                    tracker.track_largest = not tracker.track_largest
                    print(f"Tracking mode: {'Largest' if tracker.track_largest else 'Closest'}")
        
        # Cleanup
        self.set_velocity(vx=0, vy=0, vz=0, yaw_rate=0)
        tracker.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # Connection string for SITL via MAVProxy
    # Default: "udp:172.30.144.1:14551"
    # Alternative: "tcp:127.0.0.1:5760" for direct connection
    
    connection_string = "udp:172.30.144.1:14551"
    
    controller = DroneController(connection_string)
    
    # Set mode to GUIDED (required for velocity control)
    print("Setting mode to GUIDED...")
    controller.set_mode('GUIDED')
    time.sleep(1)
    
    # Optional: Arm the drone if needed
    # controller.arm_drone()
    
    # Run tracker with drone control
    controller.run_with_tracker(video_source=0, model_path='yolov8n.pt')
