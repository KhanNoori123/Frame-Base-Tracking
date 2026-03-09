#!/usr/bin/env python3
"""
Test camera stream connectivity
Quick diagnostic tool to verify GStreamer pipeline is working
"""

import cv2
import time
import sys

def test_udp_stream(host="127.0.0.1", port=5602, timeout=10):
    """Test UDP camera stream"""
    print("=" * 60)
    print("Camera Stream Test")
    print("=" * 60)
    print(f"Testing UDP stream: {host}:{port}")
    print(f"Timeout: {timeout} seconds")
    print()
    
    # Build GStreamer pipeline
    pipeline = (
        f"udpsrc address={host} port={port} "
        "caps=\"application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264\" ! "
        "rtpjitterbuffer latency=50 drop-on-latency=true ! "
        "rtph264depay ! "
        "h264parse ! "
        "avdec_h264 max-threads=2 ! "
        "videoconvert ! "
        "appsink drop=true max-buffers=2 sync=false"
    )
    
    print("Opening video capture...")
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("❌ FAILED: Could not open video capture")
        print("\nTroubleshooting:")
        print("1. Make sure Gazebo is running")
        print("2. Run: python3 gazebo_camera_on.py")
        print("3. Check if camera is streaming: gz topic -e -t /camera/image")
        return False
    
    print("✓ Video capture opened")
    print("\nWaiting for frames...")
    
    start_time = time.time()
    frame_count = 0
    
    while (time.time() - start_time) < timeout:
        ret, frame = cap.read()
        
        if ret and frame is not None and frame.size > 0:
            frame_count += 1
            if frame_count == 1:
                print(f"✓ First frame received! Shape: {frame.shape}")
            
            if frame_count >= 10:
                elapsed = time.time() - start_time
                fps = frame_count / elapsed
                print(f"✓ SUCCESS: Received {frame_count} frames in {elapsed:.2f}s ({fps:.1f} FPS)")
                
                # Show a test frame
                cv2.imshow("Camera Test", frame)
                print("\nPress any key to close...")
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                
                cap.release()
                return True
        
        time.sleep(0.1)
    
    print(f"❌ TIMEOUT: Only received {frame_count} frames in {timeout} seconds")
    print("\nTroubleshooting:")
    print("1. Camera may not be enabled in Gazebo")
    print("2. Run: python3 gazebo_camera_on.py")
    print("3. Check Gazebo logs for camera errors")
    
    cap.release()
    return False


if __name__ == "__main__":
    success = test_udp_stream()
    sys.exit(0 if success else 1)
