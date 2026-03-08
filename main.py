"""
Main application module
Professional drone tracking system
"""

import time
import math
import sys
from camera_manager import CameraManager
from tracker import ObjectTracker
from drone_interface import DroneInterface
from ui_renderer import UIRenderer
from ibvs_controller import IBVSController
from config import DroneConfig, CameraConfig, IBVSConfig

# Import gazebo camera enabler
try:
    from gazebo_camera_on import enable_camera_stream
    GAZEBO_CAMERA_AVAILABLE = True
except ImportError:
    GAZEBO_CAMERA_AVAILABLE = False
    print("Warning: gazebo_camera_on.py not found")


class DroneTrackingSystem:
    """Professional drone tracking system"""
    
    def __init__(self):
        """Initialize the tracking system"""
        print("=" * 60)
        print("Professional Drone Tracking System")
        print("=" * 60)
        
        # Enable Gazebo camera stream if using UDP stream
        if CameraConfig.USE_UDP_STREAM and GAZEBO_CAMERA_AVAILABLE:
            print("\nEnabling Gazebo camera stream...")
            if enable_camera_stream():
                print("Gazebo camera enabled successfully")
                time.sleep(1)  # Give camera time to start
            else:
                print("Warning: Could not enable Gazebo camera")
                print("If using Gazebo, make sure it's running")
                response = input("Continue anyway? (y/n): ")
                if response.lower() != 'y':
                    print("Exiting...")
                    sys.exit(1)
        
        # Initialize components
        self.camera = CameraManager()
        self.tracker = ObjectTracker()
        self.drone = DroneInterface()
        self.ibvs = IBVSController(IBVSConfig)
        
        frame_width, frame_height = self.camera.get_frame_dimensions()
        self.ui = UIRenderer(frame_width, frame_height)
        
        # State
        self.running = False
        self.last_command_time = time.time()
        self.last_yaw = 0
        self.last_z = 0
        self.frame_counter = 0
        self.last_detections = []
        self.last_target = None
        self.debug_counter = 0  # For reducing debug output
        self.altitude_control_enabled = False  # Toggle for vertical tracking
        
        # Setup drone
        self._setup_drone()
    
    def _setup_drone(self):
        """Setup drone initial state"""
        print("\nSetting up drone...")
        self.drone.set_mode(DroneConfig.DEFAULT_MODE)
        
        current_mode = self.drone.get_mode()
        print(f"Flight mode: {current_mode}")
        
        armed = self.drone.is_armed()
        print(f"Armed: {'YES' if armed else 'NO'}")
        
        if not armed:
            print("WARNING: Drone not armed. Commands will not execute.")
            print("Arm in MAVProxy: 'arm throttle'")
        
        if DroneConfig.AUTO_ARM and not armed:
            self.drone.arm()
    
    def run(self):
        """Main application loop"""
        self.running = True
        self._print_controls()
        
        try:
            while self.running:
                self._process_frame()
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self._cleanup()
    
    def _process_frame(self):
        """Process single frame"""
        # Read frame
        ret, frame = self.camera.read_frame()
        if not ret:
            print("Failed to read frame")
            self.running = False
            return
        
        current_time = time.time()
        self.frame_counter += 1
        
        # Skip frames for performance (only run detection every Nth frame)
        run_detection = (self.frame_counter % CameraConfig.SKIP_FRAMES == 0)
        
        if run_detection:
            # Detect objects
            detections = self.tracker.detect(frame)
            self.last_detections = detections
            
            # Handle mouse click
            click = self.ui.get_click()
            if click:
                clicked_obj = self.tracker.find_clicked_object(detections, click[0], click[1])
                if clicked_obj:
                    self.tracker.lock_to_object(clicked_obj)
            
            # Handle manual selection (drag)
            manual_bbox = self.ui.get_manual_selection()
            if manual_bbox:
                # Create a manual detection object
                x1, y1, x2, y2 = manual_bbox
                manual_detection = {
                    'bbox': manual_bbox,
                    'confidence': 1.0,
                    'class': -1,  # Manual selection
                    'class_name': 'Manual',
                    'center_x': (x1 + x2) // 2,
                    'center_y': (y1 + y2) // 2,
                    'area': (x2 - x1) * (y2 - y1),
                    'track_id': None
                }
                self.tracker.lock_to_object(manual_detection)
                print("Locked to manual selection")
            
            # Select target
            frame_center = self.camera.get_frame_center()
            target = self.tracker.select_target(detections, frame_center, frame)
            self.last_target = target
        else:
            # Use cached detections and target
            detections = self.last_detections
            target = self.last_target
        
        # Process target
        position = None
        velocity = {'vx': 0, 'vy': 0, 'vz': 0}
        control_output = None
        
        if target is not None:
            position = self.tracker.calculate_position_metrics(target, self.camera.get_frame_center())
            velocity = self.tracker.calculate_velocity(position, current_time)
            
            # Calculate control commands using IBVS
            control_output = self.ibvs.calculate_control(position)
            
            vx = control_output['vx']
            vy = control_output['vy']
            vz = control_output['vz'] if self.altitude_control_enabled else 0  # Only use vz if enabled
            yaw_rate = control_output['yaw_rate']
            
            self.last_yaw = yaw_rate
            self.last_z = vz
            
            # Send commands
            if self.drone.control_enabled:
                self.drone.send_velocity_command(vx, vy, vz, yaw_rate)
                self.last_command_time = current_time
                
                # Debug output (every 5 frames to reduce spam)
                self.debug_counter += 1
                if self.debug_counter % 5 == 0:
                    area_info = f"Area: {position['area']:.0f}px² (target: {self.ibvs.target_area})"
                    print(f"IBVS: {self.ibvs.get_status_string(control_output)} | {area_info}")
        else:
            # No target - stop drone and reset IBVS state
            if self.drone.control_enabled:
                self.drone.stop()
                self.last_yaw = 0
                self.last_z = 0
            self.ibvs.reset_stopping_state()
        
        # Resend commands periodically
        if self.drone.control_enabled and (current_time - self.last_command_time) > DroneConfig.COMMAND_RATE and control_output:
            vx = control_output['vx']
            vy = control_output['vy']
            self.drone.send_velocity_command(vx, vy, self.last_z, self.last_yaw)
            self.last_command_time = current_time
        
        # Render UI
        fps = self.camera.get_fps()
        frame = self.ui.draw_frame(frame, target, position, velocity, detections,
                                   self.tracker.tracking_locked, self.drone.control_enabled, fps, 
                                   self.altitude_control_enabled)
        self.ui.show(frame)
        
        # Handle keyboard input
        self._handle_keyboard()
    
    def _handle_keyboard(self):
        """Handle keyboard input"""
        key = self.ui.wait_key(1)
        
        if key == ord('q'):
            self.running = False
        elif key == ord(' '):
            self.drone.toggle_control()
        elif key == ord('r'):
            self.tracker.reset_tracking()
            self.ibvs.reset_stopping_state()
            print("Tracking reset")
        elif key == ord('i'):
            self.drone.invert_yaw = not self.drone.invert_yaw
            print(f"Yaw: {'INVERTED' if self.drone.invert_yaw else 'NORMAL'}")
        elif key == ord('k'):
            self.drone.invert_z = not self.drone.invert_z
            print(f"Z: {'INVERTED' if self.drone.invert_z else 'NORMAL'}")
        elif key == ord('g'):
            print("Setting GUIDED mode...")
            self.drone.set_mode('GUIDED')
            print(f"Mode: {self.drone.get_mode()}")
        elif key == ord('l'):
            if not self.tracker.tracking_locked:
                self.tracker.track_largest = not self.tracker.track_largest
                mode = "Largest" if self.tracker.track_largest else "Closest"
                print(f"Tracking mode: {mode}")
        elif key == ord('z'):
            self.altitude_control_enabled = not self.altitude_control_enabled
            status = "ENABLED" if self.altitude_control_enabled else "DISABLED"
            print(f"Altitude control: {status}")
    
    def _print_controls(self):
        """Print control instructions"""
        print("\n" + "=" * 60)
        print("CONTROLS")
        print("=" * 60)
        print("SPACE  - Enable/Disable drone control")
        print("Click  - Lock tracking to object")
        print("r      - Reset tracking")
        print("z      - Toggle altitude control (vertical tracking)")
        print("i      - Invert yaw direction")
        print("k      - Invert Z direction")
        print("g      - Set GUIDED mode")
        print("l      - Toggle tracking mode (largest/closest)")
        print("q      - Quit")
        print("=" * 60)
        print("\nSystem ready. Waiting for detections...\n")
    
    def _cleanup(self):
        """Cleanup resources"""
        print("\nShutting down...")
        
        if self.drone.control_enabled:
            self.drone.stop()
        
        self.camera.release()
        self.ui.destroy()
        
        print("Shutdown complete")


def main():
    """Entry point"""
    try:
        system = DroneTrackingSystem()
        system.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
