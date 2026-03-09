# Camera Stream Troubleshooting Guide

## Problem: UI Hangs When Tracking Starts

The issue occurs when the GStreamer pipeline blocks waiting for camera frames. This has been fixed with the following improvements:

## Fixes Applied

### 1. GStreamer Pipeline Optimization (camera_manager.py)
- Added `sync=false` to prevent blocking on frame timing
- Increased `max-buffers=2` for better buffering
- Increased `latency=50` for more stable jitter handling
- Removed `skip-frame=default` which could cause frame drops
- Added `emit-signals=true` for better control

### 2. Non-Blocking Frame Reading (camera_manager.py)
- Implemented `grab()` + `retrieve()` pattern for UDP streams
- This clears the buffer and prevents blocking on old frames

### 3. Better Initialization (main.py)
- Increased wait time after enabling camera (3 seconds)
- Added frame validation loop to ensure camera is streaming
- Added timeout protection in frame reading
- Better error handling with retry logic

### 4. UI Improvements (ui_renderer.py)
- Added `WINDOW_KEEPRATIO` flag
- Force window refresh with `waitKey(1)` after creation

## Testing Steps

### Step 1: Test Camera Stream First
```bash
# Run the camera test script
python3 test_camera_stream.py
```

This will:
- Verify GStreamer pipeline works
- Check if frames are being received
- Show FPS and frame dimensions
- Display a test frame

### Step 2: Enable Camera in Gazebo
```bash
# If test fails, enable camera manually
python3 gazebo_camera_on.py
```

### Step 3: Run Main Application
```bash
python3 main.py
```

## Common Issues and Solutions

### Issue: "Failed to open video source"
**Solution:**
- Make sure Gazebo is running
- Run `gazebo_camera_on.py` to enable the camera
- Check if GStreamer is installed: `gst-inspect-1.0 --version`

### Issue: "Could not get valid frames from camera"
**Solution:**
- Camera may not be streaming yet
- Wait a few more seconds after enabling
- Check Gazebo logs for camera errors
- Verify UDP port 5602 is not blocked

### Issue: UI still hangs
**Solution:**
- Check if another process is using the camera
- Try restarting Gazebo
- Increase the wait time in main.py (line with `time.sleep(3)`)
- Run test_camera_stream.py to isolate the issue

### Issue: Low FPS or laggy video
**Solution:**
- Reduce YOLO model size (use yolov8n.pt instead of yolov8s.pt)
- Increase SKIP_FRAMES in config.py
- Check CPU/GPU usage
- Reduce jitter buffer latency (currently 50ms)

## GStreamer Pipeline Explained

```
udpsrc address=127.0.0.1 port=5602
  ↓ Receives UDP packets
caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264"
  ↓ Specifies RTP video format
rtpjitterbuffer latency=50 drop-on-latency=true
  ↓ Buffers packets, drops old ones
rtph264depay
  ↓ Extracts H264 from RTP
h264parse
  ↓ Parses H264 stream
avdec_h264 max-threads=2
  ↓ Decodes H264 (2 threads)
videoconvert
  ↓ Converts to OpenCV format
appsink drop=true max-buffers=2 sync=false
  ↓ Provides frames to OpenCV (non-blocking)
```

## Verification Commands

```bash
# Check if Gazebo is running
ps aux | grep gz

# List Gazebo topics
gz topic -l | grep camera

# Monitor camera topic
gz topic -e -t /camera/image

# Check if UDP port is listening
netstat -an | grep 5602

# Test GStreamer pipeline directly
gst-launch-1.0 udpsrc address=127.0.0.1 port=5602 \
  caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! \
  rtpjitterbuffer ! rtph264depay ! h264parse ! avdec_h264 ! \
  videoconvert ! autovideosink
```

## Performance Tips

1. **Use GPU acceleration**: Make sure CUDA is available for YOLO
2. **Reduce detection frequency**: Increase SKIP_FRAMES in config.py
3. **Lower resolution**: If possible, configure Gazebo camera for lower resolution
4. **Optimize pipeline**: Reduce latency parameter if network is stable
5. **Close other applications**: Free up CPU/GPU resources

## Still Having Issues?

If problems persist:
1. Check system logs: `dmesg | tail -50`
2. Check Gazebo logs in terminal
3. Verify OpenCV GStreamer support: `python3 -c "import cv2; print(cv2.getBuildInformation())" | grep GStreamer`
4. Try a different port or host configuration
5. Test with a simpler pipeline (remove jitter buffer, etc.)
