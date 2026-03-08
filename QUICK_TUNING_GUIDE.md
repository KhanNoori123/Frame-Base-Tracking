# Quick Tuning Guide

## Problem: Detection is still weak

### Solution 1: Lower detection threshold
**File**: `config.py`
```python
CONFIDENCE_THRESHOLD = 0.15  # Even lower (was 0.20)
```

### Solution 2: Increase persistence
**File**: `config.py`
```python
MAX_LOST_FRAMES = 75  # Keep searching longer (was 50)
```

### Solution 3: More lenient matching
**File**: `config.py`
```python
MIN_MATCH_SCORE = 0.15  # Accept weaker matches (was 0.20)
```

---

## Problem: Too many false detections

### Solution 1: Raise detection threshold
**File**: `config.py`
```python
CONFIDENCE_THRESHOLD = 0.30  # Higher confidence required (was 0.20)
```

### Solution 2: Stricter matching
**File**: `config.py`
```python
MIN_MATCH_SCORE = 0.30  # Require better matches (was 0.20)
```

---

## Problem: Tracking is jittery/jumpy

### Solution 1: Increase Kalman smoothing
**File**: `tracker.py` → `_init_kalman()` method
```python
# Increase measurement noise (trust detections less, smooth more)
self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 20  # was 10
```

### Solution 2: Decrease process noise
**File**: `tracker.py` → `_init_kalman()` method
```python
# Decrease process noise (trust model more)
self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.01  # was 0.03
```

---

## Problem: Prediction is inaccurate

### Solution 1: Trust detections more
**File**: `tracker.py` → `_init_kalman()` method
```python
# Lower measurement noise (trust detections more)
self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 5  # was 10
```

### Solution 2: Faster adaptation
**File**: `tracker.py` → `_init_kalman()` method
```python
# Higher process noise (adapt faster to changes)
self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.05  # was 0.03
```

---

## Problem: Drone overshoots target

### Solution: Reduce speeds
**File**: `config.py` → `IBVSConfig`
```python
MAX_FORWARD_SPEED = 2.0   # Slower max speed (was 2.5)
MIN_FORWARD_SPEED = 0.2   # Slower min speed (was 0.3)
```

---

## Problem: Drone stops too early

### Solution: Adjust target area
**File**: `config.py` → `IBVSConfig`
```python
TARGET_AREA = 100000      # Larger target = closer approach (was 80000)
AREA_DEADZONE = 10000     # Wider tolerance (was 8000)
```

---

## Problem: Drone stops too late (too close)

### Solution: Adjust target area
**File**: `config.py` → `IBVSConfig`
```python
TARGET_AREA = 60000       # Smaller target = farther stop (was 80000)
AREA_DEADZONE = 6000      # Tighter tolerance (was 8000)
```

---

## Quick Test Commands

### Test detection only (no drone)
```bash
python tracker.py
```

### Test with drone control
```bash
python main.py
```

### Check if Kalman is working
Look for console output:
- "Using prediction... (X/50)" = Kalman active
- Yellow dashed box in UI = Prediction mode

---

## Recommended Starting Values

### For Gazebo simulation (stable environment):
```python
CONFIDENCE_THRESHOLD = 0.20
MAX_LOST_FRAMES = 50
MIN_MATCH_SCORE = 0.20
USE_KALMAN_FILTER = True
```

### For real drone (noisy environment):
```python
CONFIDENCE_THRESHOLD = 0.25
MAX_LOST_FRAMES = 75
MIN_MATCH_SCORE = 0.15
USE_KALMAN_FILTER = True
# Increase Kalman measurement noise to 20
```

### For fast-moving targets:
```python
MAX_TRACKING_DISTANCE = 400
MAX_LOST_FRAMES = 30  # Shorter timeout
# Increase Kalman process noise to 0.05
```

### For slow-moving targets:
```python
MAX_TRACKING_DISTANCE = 200
MAX_LOST_FRAMES = 75  # Longer timeout
# Decrease Kalman process noise to 0.01
```
