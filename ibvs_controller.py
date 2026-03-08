"""
Image-Based Visual Servoing (IBVS) Controller
Advanced drone motion control based on object size in image
"""

import math


class IBVSController:
    """
    IBVS controller that moves drone based on object appearance in image
    - Small object (far) -> Move forward fast
    - Large object (close) -> Move forward slow or stop
    - Object off-center -> Rotate to center
    """
    
    def __init__(self, config):
        """
        Initialize IBVS controller
        
        Args:
            config: IBVSConfig object with control parameters
        """
        self.config = config
        
        # Target area tracking
        self.target_area = config.TARGET_AREA
        self.min_area = config.MIN_AREA
        self.max_area = config.MAX_AREA
        
        # Control gains
        self.forward_gain = config.FORWARD_GAIN
        self.yaw_gain = config.YAW_GAIN
        self.altitude_gain = config.ALTITUDE_GAIN
        
        # Speed limits
        self.max_forward_speed = config.MAX_FORWARD_SPEED
        self.min_forward_speed = config.MIN_FORWARD_SPEED
        self.max_yaw_rate = config.MAX_YAW_RATE
        self.max_altitude_speed = config.MAX_ALTITUDE_SPEED
        
        # Deadzones
        self.yaw_deadzone = config.YAW_DEADZONE
        self.altitude_deadzone = config.ALTITUDE_DEADZONE
        self.area_deadzone = config.AREA_DEADZONE
        
        # Control mode
        self.use_altitude_control = config.USE_ALTITUDE_CONTROL
        
        # Stopping algorithm state
        self.prev_area = None
        self.prev_velocity = 0.0
        self.stopped_counter = 0  # Count frames at target to confirm stop
        self.STOP_CONFIRMATION_FRAMES = 5  # Frames to confirm we're at target
        
    def calculate_control(self, position_data):
        """
        Calculate control commands based on object position and size
        
        Args:
            position_data: Dict with 'dx', 'dy', 'area' keys
            
        Returns:
            dict: Control commands with 'vx', 'vy', 'vz', 'yaw_rate'
        """
        if position_data is None:
            return {'vx': 0, 'vy': 0, 'vz': 0, 'yaw_rate': 0}
        
        dx = position_data['dx']  # Horizontal offset from center
        dy = position_data['dy']  # Vertical offset from center
        area = position_data['area']  # Object area in pixels
        
        # Calculate yaw rate to center object horizontally
        yaw_rate = self._calculate_yaw_rate(dx)
        
        # Calculate forward velocity based on X centering AND object size
        # Only move forward when object is well-centered
        vx = self._calculate_forward_velocity_centered(dx, area)
        
        # Calculate altitude velocity to center object vertically
        if self.use_altitude_control:
            vz = self._calculate_altitude_velocity(dy)
        else:
            vz = 0
        
        # No lateral movement
        vy = 0
        
        return {
            'vx': vx,
            'vy': vy,
            'vz': vz,
            'yaw_rate': yaw_rate,
            'area_error': self.target_area - area,
            'distance_status': self._get_distance_status(area),
            'centering_status': self._get_centering_status(dx)
        }
    
    def _calculate_forward_velocity_centered(self, dx, current_area):
        """
        Calculate forward velocity with advanced stopping algorithms:
        1. Exponential deceleration as approaching target
        2. Predictive stopping based on area growth rate
        3. Hysteresis to prevent oscillation
        4. Safety buffer zone
        """
        # ALGORITHM 1: Safety check - too close, back up
        if current_area > self.max_area:
            self.stopped_counter = 0
            return -0.5  # Back up slowly
        
        # ALGORITHM 2: Centering requirement - must be centered to move forward
        centering_factor = self._calculate_centering_factor(dx)
        if centering_factor < 0.3:  # Less than 30% centered
            self.stopped_counter = 0
            return 0.0
        
        # Calculate area error (negative = too far, positive = too close)
        area_error = self.target_area - current_area
        
        # ALGORITHM 3: Hysteresis - confirm we're at target for multiple frames
        if abs(area_error) < self.area_deadzone:
            self.stopped_counter += 1
            if self.stopped_counter >= self.STOP_CONFIRMATION_FRAMES:
                # Confirmed at target, stay stopped
                return 0.0
            else:
                # Approaching target, slow down significantly
                return self.min_forward_speed * 0.2 * centering_factor
        else:
            self.stopped_counter = 0  # Reset counter if we leave deadzone
        
        # ALGORITHM 4: Predictive stopping - estimate if we'll overshoot
        if self.prev_area is not None and current_area > self.prev_area:
            # Calculate area growth rate (pixels per frame)
            area_growth_rate = current_area - self.prev_area
            
            # Estimate frames until target
            if area_growth_rate > 0:
                frames_to_target = area_error / area_growth_rate
                
                # If very close (less than 10 frames at current rate), apply braking
                if frames_to_target < 10 and frames_to_target > 0:
                    # Aggressive braking factor
                    braking_factor = frames_to_target / 10.0
                    braking_factor = max(0.1, braking_factor)  # Minimum 10% speed
                else:
                    braking_factor = 1.0
            else:
                braking_factor = 1.0
        else:
            braking_factor = 1.0
        
        # Store current area for next iteration
        self.prev_area = current_area
        
        # ALGORITHM 5: Exponential deceleration based on distance
        if area_error < 0:
            # Object is too close, back up slowly
            base_speed = -0.5
        else:
            # Object is too far, move forward
            if current_area < self.min_area:
                # Very far - use max speed
                base_speed = self.max_forward_speed
            else:
                # In range - smooth proportional control with exponential decay
                # Normalize area to 0-1 range (0 = far, 1 = at target)
                area_normalized = (current_area - self.min_area) / (self.target_area - self.min_area)
                area_normalized = max(0, min(1, area_normalized))
                
                # Use cubic function for very aggressive deceleration near target
                # This creates a smooth stop curve
                speed_factor = (1.0 - area_normalized) ** 3
                
                base_speed = self.min_forward_speed + (self.max_forward_speed - self.min_forward_speed) * speed_factor
                
                # ALGORITHM 6: Safety buffer zone - extra slow in final approach
                if current_area > self.target_area * 0.7:  # Within 70% of target
                    safety_factor = 1.0 - ((current_area - self.target_area * 0.7) / (self.target_area * 0.3))
                    safety_factor = max(0.2, safety_factor)  # Minimum 20% speed
                    base_speed = base_speed * safety_factor
        
        # Apply all factors: centering, braking, and safety
        vx = base_speed * centering_factor * braking_factor
        
        # Store velocity for next iteration
        self.prev_velocity = vx
        
        return vx
    
    def _calculate_centering_factor(self, dx):
        """
        Calculate how well-centered the object is
        Returns 0.0 (completely off-center) to 1.0 (perfectly centered)
        """
        # Use a smooth function based on dx
        # Within deadzone -> 1.0 (fully centered)
        # Beyond threshold -> 0.0 (not centered)
        
        abs_dx = abs(dx)
        
        if abs_dx <= self.yaw_deadzone:
            return 1.0  # Perfectly centered
        
        # Define a threshold beyond which we don't move forward at all
        max_offset = 150  # pixels
        
        if abs_dx >= max_offset:
            return 0.0  # Too off-center, don't move forward
        
        # Linear interpolation between deadzone and max_offset
        centering_factor = 1.0 - ((abs_dx - self.yaw_deadzone) / (max_offset - self.yaw_deadzone))
        centering_factor = max(0.0, min(1.0, centering_factor))
        
        return centering_factor
    
    def _calculate_yaw_rate(self, dx):
        """
        Calculate yaw rate to center object horizontally
        Positive dx = object on right -> turn right (positive yaw)
        """
        if abs(dx) < self.yaw_deadzone:
            return 0.0
        
        yaw_rate = dx * self.yaw_gain
        yaw_rate = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate))
        
        return yaw_rate
    
    def _calculate_altitude_velocity(self, dy):
        """
        Calculate altitude velocity to center object vertically
        Positive dy = object below center -> descend (positive vz in NED)
        """
        if abs(dy) < self.altitude_deadzone:
            return 0.0
        
        vz = dy * self.altitude_gain
        vz = max(-self.max_altitude_speed, min(self.max_altitude_speed, vz))
        
        return vz
    
    def _get_distance_status(self, current_area):
        """Get human-readable distance status"""
        if current_area < self.min_area:
            return "TOO_FAR"
        elif current_area > self.max_area:
            return "TOO_CLOSE"
        elif abs(self.target_area - current_area) < self.area_deadzone:
            return "OPTIMAL"
        elif current_area < self.target_area:
            return "FAR"
        else:
            return "CLOSE"
    
    def _get_centering_status(self, dx):
        """Get human-readable centering status"""
        abs_dx = abs(dx)
        if abs_dx <= self.yaw_deadzone:
            return "CENTERED"
        elif abs_dx < 50:
            return "NEARLY_CENTERED"
        elif abs_dx < 100:
            return "OFF_CENTER"
        else:
            return "FAR_OFF_CENTER"
    
    def get_status_string(self, control_output):
        """Get formatted status string for display"""
        if control_output is None:
            return "No control output"
        
        status = control_output.get('distance_status', 'UNKNOWN')
        centering = control_output.get('centering_status', 'UNKNOWN')
        vx = control_output.get('vx', 0)
        yaw = control_output.get('yaw_rate', 0)
        
        # Add stopping algorithm status
        stop_info = ""
        if self.stopped_counter > 0:
            stop_info = f" | Stopping: {self.stopped_counter}/{self.STOP_CONFIRMATION_FRAMES}"
        
        status_str = f"Distance: {status} | Centering: {centering}{stop_info} | "
        status_str += f"Forward: {vx:+.2f}m/s | "
        status_str += f"Yaw: {yaw:+.1f}°/s"
        
        return status_str
    
    def reset_stopping_state(self):
        """Reset stopping algorithm state (call when target is lost or changed)"""
        self.prev_area = None
        self.prev_velocity = 0.0
        self.stopped_counter = 0
