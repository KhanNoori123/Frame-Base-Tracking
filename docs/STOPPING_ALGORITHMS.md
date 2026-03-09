# Drone Stopping Algorithms

## Overview
The drone now uses multiple advanced algorithms to prevent overshooting or losing the target when approaching. These algorithms work together to ensure smooth, precise stopping.

## Implemented Algorithms

### 1. **Safety Check (Too Close Detection)**
- **Purpose**: Prevent collision with target
- **How it works**: If object area exceeds MAX_AREA (150,000 pixels²), drone backs up slowly at -0.5 m/s
- **Trigger**: `current_area > MAX_AREA`

### 2. **Centering Requirement**
- **Purpose**: Ensure target is centered before moving forward
- **How it works**: Drone only moves forward when object is at least 30% centered horizontally
- **Benefit**: Prevents losing target by moving forward while off-center
- **Trigger**: `centering_factor < 0.3` → stop forward motion

### 3. **Hysteresis (Stop Confirmation)**
- **Purpose**: Prevent oscillation around target distance
- **How it works**: Requires drone to be within deadzone for 5 consecutive frames before confirming stop
- **Parameters**: 
  - `STOP_CONFIRMATION_FRAMES = 5`
  - `AREA_DEADZONE = 8000` pixels²
- **Benefit**: Smooth, stable stopping without jitter

### 4. **Predictive Stopping (Area Growth Rate)**
- **Purpose**: Anticipate arrival at target and brake early
- **How it works**: 
  - Calculates area growth rate (pixels per frame)
  - Estimates frames until target: `frames_to_target = area_error / area_growth_rate`
  - If less than 10 frames away, applies braking factor
  - Braking factor: `frames_to_target / 10.0` (minimum 10% speed)
- **Benefit**: Prevents overshooting by slowing down before reaching target

### 5. **Exponential Deceleration**
- **Purpose**: Smooth speed reduction as approaching target
- **How it works**: Uses cubic function for aggressive deceleration
  - Speed factor: `(1.0 - area_normalized)³`
  - When far (area_normalized = 0): full speed
  - When close (area_normalized = 1): stop
- **Benefit**: Natural, smooth approach curve

### 6. **Safety Buffer Zone**
- **Purpose**: Extra caution in final approach
- **How it works**: When within 70% of target area, applies additional safety factor
  - Safety factor reduces speed to minimum 20%
  - Creates a "slow zone" near target
- **Trigger**: `current_area > TARGET_AREA * 0.7`

## Configuration Parameters

### Speed Limits (config.py)
```python
MAX_FORWARD_SPEED = 2.5   # m/s - reduced for better control
MIN_FORWARD_SPEED = 0.3   # m/s - lower for final approach
```

### Target Areas
```python
TARGET_AREA = 80000       # Optimal distance (pixels²)
MIN_AREA = 5000          # Too far threshold
MAX_AREA = 150000        # Too close threshold
AREA_DEADZONE = 8000     # Tolerance for "at target"
```

### Stopping Algorithm Parameters
```python
STOP_CONFIRMATION_FRAMES = 5           # Hysteresis frames
PREDICTIVE_BRAKING_THRESHOLD = 10      # Frames ahead to brake
SAFETY_BUFFER_RATIO = 0.7              # Start extra slowing at 70%
```

## How They Work Together

1. **Far from target** (area < MIN_AREA):
   - Full speed forward (2.5 m/s)
   - Only if centered

2. **Approaching target** (MIN_AREA < area < TARGET_AREA * 0.7):
   - Exponential deceleration (cubic curve)
   - Predictive braking if close
   - Centering requirement enforced

3. **Safety buffer zone** (TARGET_AREA * 0.7 < area < TARGET_AREA):
   - Extra slow (20-100% of calculated speed)
   - All algorithms active
   - Hysteresis counting begins

4. **At target** (within AREA_DEADZONE):
   - Requires 5 consecutive frames to confirm
   - Stops completely when confirmed
   - Prevents oscillation

5. **Too close** (area > MAX_AREA):
   - Back up at -0.5 m/s
   - All counters reset

## Benefits

✓ **No overshooting**: Predictive braking prevents flying past target
✓ **No oscillation**: Hysteresis prevents back-and-forth jitter
✓ **Smooth approach**: Exponential deceleration creates natural motion
✓ **Safe operation**: Multiple safety checks and buffer zones
✓ **Won't lose target**: Centering requirement keeps target in view

## Debugging

The system displays stopping status in real-time:
- Distance status: TOO_FAR, FAR, CLOSE, OPTIMAL, TOO_CLOSE
- Centering status: CENTERED, NEARLY_CENTERED, OFF_CENTER, FAR_OFF_CENTER
- Stopping counter: Shows hysteresis progress (e.g., "Stopping: 3/5")
- Forward velocity: Current speed in m/s
- Yaw rate: Current rotation rate in deg/s

## Tuning Tips

If drone **overshoots**:
- Decrease `MAX_FORWARD_SPEED`
- Increase `PREDICTIVE_BRAKING_THRESHOLD`
- Increase `SAFETY_BUFFER_RATIO` (e.g., 0.8)

If drone **stops too early**:
- Increase `AREA_DEADZONE`
- Decrease `SAFETY_BUFFER_RATIO` (e.g., 0.6)
- Increase `MIN_FORWARD_SPEED`

If drone **oscillates**:
- Increase `STOP_CONFIRMATION_FRAMES`
- Increase `AREA_DEADZONE`

If drone **loses target while approaching**:
- Increase centering threshold (currently 0.3)
- Decrease `MAX_FORWARD_SPEED`
