# Object Tracker with YOLO

A real-time object tracking system using YOLOv8 that detects objects, measures their position relative to the frame center, and calculates velocity along X, Y, and Z axes.

## Features

- **YOLO Detection**: Uses YOLOv8 for accurate object detection (80+ object classes)
- **Center-based tracking**: Displays frame center with crosshair
- **Smart object selection**: Track largest object or closest to center
- **Class filtering**: Filter and track specific object types (person, car, etc.)
- **Position measurement**: Tracks object position and offset from center
- **Distance calculation**: Measures 2D distance from frame center
- **Velocity tracking**: Calculates velocity on X, Y, Z axes
  - X-axis: Horizontal movement (left/right)
  - Y-axis: Vertical movement (up/down)
  - Z-axis: Depth estimation based on object size change
- **Real-time visualization**: Displays all metrics on video frame

## Installation

```bash
pip install -r requirements.txt
```

The first time you run the script, YOLOv8 will automatically download the model weights (~6MB for yolov8n.pt).

## Usage

Run the tracker:
```bash
python object_tracker.py
```

### Controls

- **Click on any object**: Lock tracking to that specific object (ignores all others)
- **r**: Reset/unlock tracking (return to automatic mode)
- **q**: Quit the application
- **l**: Toggle tracking mode (largest object / closest to center) - only when unlocked
- **a**: Track all object classes - only when unlocked
- **0-9**: Filter by specific class ID (e.g., 0 for person) - only when unlocked

### Common YOLO Classes

- 0: person
- 1: bicycle
- 2: car
- 3: motorcycle
- 5: bus
- 7: truck
- 16: dog
- 17: cat
- 39: bottle
- 41: cup
- 56: chair
- 67: cell phone

[Full list of 80 classes available in COCO dataset]

## Customization

### Use Different YOLO Model

```python
# Faster but less accurate
tracker = ObjectTracker(video_source=0, model_path='yolov8n.pt')

# More accurate but slower
tracker = ObjectTracker(video_source=0, model_path='yolov8m.pt')
tracker = ObjectTracker(video_source=0, model_path='yolov8l.pt')
```

### Use Video File Instead of Webcam

```python
tracker = ObjectTracker(video_source='path/to/video.mp4')
```

### Adjust Confidence Threshold

```python
tracker.run(confidence_threshold=0.7)  # Higher = fewer false positives
```

## How It Works

1. **YOLO Detection**: Identifies objects in each frame with bounding boxes
2. **Object Selection**: Chooses target based on size or proximity to center
3. **Position**: Calculates object center and offset from frame center
4. **Distance**: Measures 2D Euclidean distance from center
5. **Velocity**: Tracks movement speed in pixels/second for X and Y axes
6. **Depth (Z-axis)**: Estimates depth change based on bounding box area variation

## Output Information

The display shows:
- Tracked object class and confidence score
- Object position (x, y coordinates)
- Offset from center (dx, dy in pixels)
- Distance from center (in pixels)
- Velocity X, Y (pixels per second)
- Velocity Z (normalized depth change)
- Direction indicators (LEFT/RIGHT/UP/DOWN)
- All detected objects (gray boxes)
- Tracked object (yellow box)
