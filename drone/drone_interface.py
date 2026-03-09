"""
Drone interface module
Handles MAVLink communication and drone control
"""

import time
import math
from pymavlink import mavutil
from config.settings import DroneConfig


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
            vx: X velocity (m/s) - forward/backward in body frame
            vy: Y velocity (m/s) - left/right in body frame
            vz: Z velocity (m/s) - up/down in NED frame (positive = down)
            yaw_rate: Yaw rate (deg/s)
        """
        if not self.control_enabled:
            return
        
        # Type mask: bit set to 1 means IGNORE that field
        # We want to USE velocity (vx, vy, vz) and yaw_rate
        # Bits: pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, force, yaw, yaw_rate
        # Binary: 0b0000_1111_1100_0111
        # Ignore: position (bits 0-2), acceleration (bits 6-8), force (bit 9), yaw (bit 10)
        # Use: velocity (bits 3-5), yaw_rate (bit 11)
        type_mask = (
            0b0000_0100_0000_0111  # Ignore pos, acc, yaw; Use velocity and yaw_rate
        )
        
        # Convert yaw rate from deg/s to rad/s
        yaw_rate_rad = math.radians(yaw_rate)
        
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use BODY frame for vx, vy
            type_mask,
            0, 0, 0,  # x, y, z positions (ignored)
            vx, vy, vz,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (ignored)
            0,  # yaw (ignored)
            yaw_rate_rad  # yaw_rate in rad/s
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
    
    def takeoff(self, altitude=5.0):
        """
        Command drone to takeoff to specified altitude
        
        Args:
            altitude: Target altitude in meters
        """
        print(f"Taking off to {altitude}m...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            0, 0,  # latitude, longitude (not used in GUIDED)
            altitude  # altitude
        )
        print(f"Takeoff command sent")
    
    def land(self):
        """Command drone to land"""
        print("Landing...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0, 0, 0, 0,  # params 1-4
            0, 0, 0  # latitude, longitude, altitude
        )
        print("Land command sent")
    
    def get_comprehensive_telemetry(self):
        """
        Get comprehensive telemetry data for UI display
        
        Returns:
            dict: Complete telemetry data including all flight parameters
        """
        telemetry = {
            'mode': self.get_mode(),
            'armed': self.is_armed(),
            'altitude': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'vx': 0.0,
            'vy': 0.0,
            'vz': 0.0,
            'groundspeed': 0.0,
            'lat': 0.0,
            'lon': 0.0,
            'battery_voltage': 0.0,
            'heading': 0.0
        }
        
        # Get attitude data (roll, pitch, yaw)
        msg = self.master.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            telemetry['roll'] = math.degrees(msg.roll)
            telemetry['pitch'] = math.degrees(msg.pitch)
            telemetry['yaw'] = math.degrees(msg.yaw)
        
        # Get position and velocity data
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            telemetry['altitude'] = msg.relative_alt / 1000.0  # Convert mm to m
            telemetry['vx'] = msg.vx / 100.0  # Convert cm/s to m/s
            telemetry['vy'] = msg.vy / 100.0
            telemetry['vz'] = msg.vz / 100.0
            telemetry['lat'] = msg.lat / 1e7
            telemetry['lon'] = msg.lon / 1e7
            telemetry['heading'] = msg.hdg / 100.0  # Convert centidegrees to degrees
        
        # Get VFR HUD data (ground speed, altitude)
        msg = self.master.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            telemetry['groundspeed'] = msg.groundspeed
            telemetry['altitude'] = msg.alt  # Barometric altitude
        
        # Get battery data
        msg = self.master.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            telemetry['battery_voltage'] = msg.voltage_battery / 1000.0  # Convert mV to V
        
        return telemetry