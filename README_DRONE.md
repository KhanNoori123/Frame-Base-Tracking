# Drone Controller with Object Tracking

Connects to SITL drone and controls yaw and Z velocity based on detected object position relative to frame center.

## How It Works

The drone automatically adjusts:
- **YAW (rotation)**: Based on horizontal offset (X-axis)
  - Object right of center → Drone turns right
  - Object left of center → Drone turns left
- **Z VELOCITY (altitude)**: Based on vertical offset (Y-axis)
  - Object below center → Drone descends
  - Object above center → Drone ascends

## Connection Strings

### Direct SITL Connection
```bash
python drone_controller.py
# Uses: tcp:127.0.0.1:5760
```

### Via MAVProxy
If using MAVProxy with these parameters:
```bash
mavproxy.py --master=udp:172.30.144.1:14550 --out=udp:172.30.144.1:14551 --map --console
```

Edit `drone_controller.py` line 186:
```python
connection_string = "udp:172.30.144.1:14551"
```

## Controls

- **SPACE**: Enable/Disable drone control (safety feature)
- **Click**: Lock tracking to specific object
- **r**: Reset/unlock tracking
- **q**: Quit (automatically stops drone)
- **l**: Toggle tracking mode (largest/closest)

## Setup SITL Drone

### 1. Install ArduPilot SITL
```bash
# Install dependencies
pip install pymavlink MAVProxy

# Clone ArduPilot (if not already installed)
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive
```

### 2. Start SITL
```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map
```

This starts SITL on `tcp:127.0.0.1:5760`

### 3. Run Drone Controller
```bash
python drone_controller.py
```

## Configuration

Edit `drone_controller.py` to adjust sensitivity:

```python
self.yaw_gain = 0.01      # Yaw sensitivity (higher = faster rotation)
self.z_gain = 0.005       # Z velocity sensitivity
self.max_yaw_rate = 30    # Max yaw rate (deg/s)
self.max_z_velocity = 1.0 # Max vertical speed (m/s)
self.yaw_deadzone = 30    # Deadzone in pixels (prevents jitter)
self.z_deadzone = 30      # Vertical deadzone
```

## Safety Features

1. **Control disabled by default**: Press SPACE to enable
2. **Automatic stop**: Drone stops when no object detected
3. **Deadzone**: Prevents jitter when object near center
4. **Max limits**: Prevents excessive yaw/velocity commands
5. **Clean shutdown**: Stops drone before exit

## Flight Modes

The drone should be in **GUIDED** mode for velocity control:

```python
controller.set_mode('GUIDED')
controller.arm_drone()  # If needed
```

## Troubleshooting

### Connection Failed
- Check SITL is running: `tcp:127.0.0.1:5760`
- Verify firewall settings
- Try different connection string

### Drone Not Responding
- Ensure GUIDED mode is active
- Check if drone is armed (if required)
- Verify control is enabled (press SPACE)

### Tracking Issues
- Click on object to lock tracking
- Adjust confidence threshold in code
- Check lighting conditions

## Example Workflow

1. Start SITL: `sim_vehicle.py -v ArduCopter`
2. Run controller: `python drone_controller.py`
3. Click on person/object to track
4. Press SPACE to enable drone control
5. Drone will automatically center the object
6. Press SPACE to disable control
7. Press 'q' to quit

## Advanced Usage

### Track Specific Object Class
```python
# In drone_controller.py, modify run_with_tracker():
tracker.selected_class = 0  # Track only persons
```

### Use Video File
```python
controller.run_with_tracker(video_source='test_video.mp4')
```

### Custom Control Logic
Modify `calculate_control_commands()` method to implement custom control algorithms (PID, etc.)
