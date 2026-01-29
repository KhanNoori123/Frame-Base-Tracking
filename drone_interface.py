"""
Drone interface module
Handles MAVLink communication and drone control
"""

import time
import math
from pymavlink import mavutil
from config import DroneConfig


class DroneInterface:
    """Professional drone control interface"""
    
    def __init__(self, connection_string=None):
        """
        Initialize drone interface
        
        Args:
            connection_string: MAVLink connection string
        """
        self.connection_string = connection_string or DroneConfig.CONNECTION_STRING
        self.master = None
        self.control_enabled = False
        
        # Control parameters
        self.yaw_gain = DroneConfig.YAW_GAIN
        self.z_gain = DroneConfig.Z_GAIN
        self.max_yaw_rate = DroneConfig.MAX_YAW_RATE
        self.max_z_velocity = DroneConfig.MAX_Z_VELOCITY
        self.yaw_deadzone = DroneConfig.YAW_DEADZONE
        self.z_deadzone = DroneConfig.Z_DEADZONE
        
        # Control inversion
        self.invert_yaw = False
        self.invert_z = False
        
        self._connect()
    
    def _connect(self):
        """Establish connection to drone"""
        print(f"Connecting to drone at {self.connection_string}...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(f"Connected to system {self.master.target_system} component {self.master.target_component}")
    
    def set_mode(self, mode):
        """Set flight mode"""
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mode set to {mode}")
        time.sleep(0.5)
    
    def get_mode(self):
        """Get current flight mode"""
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return mavutil.mode_string_v10(msg)
        return "UNKNOWN"
    
    def is_armed(self):
        """Check if drone is armed"""
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return False
    
    def arm(self):
        """Arm the drone"""
        print("Arming drone...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Drone armed")
    
    def disarm(self):
        """Disarm the drone"""
        print("Disarming drone...")
        self.master.arducopter_disarm()
        self.master.motors_disarmed_wait()
        print("Drone disarmed")
    
    def calculate_control_commands(self, position_data):
        """
        Calculate control commands from position data
        
        Args:
            position_data: Dictionary with dx, dy offsets
            
        Returns:
            tuple: (yaw_rate, z_velocity)
        """
        if position_data is None:
            return 0, 0
        
        dx = position_data['dx']
        dy = position_data['dy']
        
        # Calculate yaw rate
        if abs(dx) > self.yaw_deadzone:
            yaw_rate = dx * self.yaw_gain
            if self.invert_yaw:
                yaw_rate = -yaw_rate
            yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        else:
            yaw_rate = 0
        
        # Calculate Z velocity
        if abs(dy) > self.z_deadzone:
            z_velocity = dy * self.z_gain
            if self.invert_z:
                z_velocity = -z_velocity
            z_velocity = max(-self.max_z_velocity, min(self.max_z_velocity, z_velocity))
        else:
            z_velocity = 0
        
        return yaw_rate, z_velocity
    
    def send_velocity_command(self, vx=0, vy=0, vz=0, yaw_rate=0):
        """
        Send velocity command to drone
        
        Args:
            vx: X velocity (m/s)
            vy: Y velocity (m/s)
            vz: Z velocity (m/s)
            yaw_rate: Yaw rate (deg/s)
        """
        if not self.control_enabled:
            return
        
        # Send Z velocity
        type_mask = 0b0000_1111_1111_0111
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        
        # Send yaw via RC override
        self._send_rc_yaw(yaw_rate)
    
    def _send_rc_yaw(self, yaw_rate_deg):
        """Send yaw command via RC override"""
        yaw_center = 1500
        yaw_range = 400
        
        if abs(yaw_rate_deg) < 0.1:
            yaw_pwm = yaw_center
        else:
            yaw_pwm = int(yaw_center + (yaw_rate_deg / self.max_yaw_rate) * yaw_range)
            yaw_pwm = max(1100, min(1900, yaw_pwm))
        
        self.master.mav.rc_channels_override_send(
            self.master.target_system,
            self.master.target_component,
            65535, 65535, 65535,  # Don't override roll, pitch, throttle
            yaw_pwm,  # Yaw
            65535, 65535, 65535, 65535
        )
    
    def stop(self):
        """Stop all movement"""
        self.send_velocity_command(0, 0, 0, 0)
    
    def enable_control(self):
        """Enable drone control"""
        self.control_enabled = True
        print("Drone control ENABLED")
    
    def disable_control(self):
        """Disable drone control"""
        self.control_enabled = False
        self.stop()
        print("Drone control DISABLED")
    
    def toggle_control(self):
        """Toggle control state"""
        if self.control_enabled:
            self.disable_control()
        else:
            self.enable_control()
