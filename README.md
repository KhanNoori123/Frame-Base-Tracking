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


## Gazebo and Ardupilot SITL Setup Guide

For using first time Gazebo and SITL ardupilot we should follow these steps.

1. first we should install the hormonic Gazebo with its own plugins .

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

2. clone the Ardupilot _gazeo and build it .

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

3. Set the Gazebo path (the simulation pluging,the models and the worlds) environment variables in your .bashrc .

3.1 this is for the simulation path it should be correct based on the arduiplilot gazebo build path .

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/Ard_Gaze/ardupilot_gazebo/build $GZ_SIM_SYSTEM_PLUGIN_PATH'
```

3.2 this is for the ardupilot models and worlds it should be also correct path or the worlds and models like the bellow .

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/Ard_Gaze/ardupilot_gazebo/models:$HOME/Ard_Gaze/ardupilot_gazebo/worlds'
source ~/.bashrc
```

3.4 for the checing of the correct path or not use this .

```bash
nano ~/.bashrc
```

4. after that we can run the Gazebo and Ardupilot SITL .

4.1 for the gazeo strat

```bash
gz sim -v4 -r iris_runway.sdf
```

4.2 for the Ardupliot SITL .

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --map --console
```

5. for the Removing and checking the gazebo cache we use this .

```bash
ps aux | grep gz
```

6. we can use this website for more reference and more features .

https://github.com/ArduPilot/ardupilot_gazebo

7. for the enabling the gazeo Camera .

7.1 we can enable that with the help of the Gestremer and we can configure the camera setting in the gazebo models gimble 3D modle.sdf file .

```xml
<plugin name="GstCameraPlugin"
        filename="GstCameraPlugin">
    <udp_host>127.0.0.1</udp_host>
    <udp_port>5602</udp_port> ---- the port we forward the display
    <use_basic_pipeline>true</use_basic_pipeline>
    <use_cuda>false</use_cuda>
</plugin>
```

7.2 for enabling or on the camera display before the camera.

```bash
gz topic -t /world/gimbal/model/mount/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

... or run the file of the gazebo_camera_on.py in the Frame-base-Tracking project .

7.3 for the checking the stream is working or not after enabling the the camera .

```bash
~$ gst-launch-1.0 -v udpsrc port=5602 caps='application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000' ! rtpjitterbuffer latency=100 ! tee name=t ! queue ! udpsink host=127.0.0.1 port=5604 t. ! queue ! udpsink host=127.0.0.1 port=5605 .
```

7.4 for camera diaplay use file of the test_camera_stream.py in the Frame-base-Tracking project .

7.5 for the yaw and pitch of the Gazebo camera use this .

```bash
cd ardupilot 
sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map
```
