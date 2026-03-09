#!/usr/bin/env python3
"""
Automatically enable Gazebo camera streaming
"""

import subprocess
import time

def find_camera_topic():
    """Find the camera enable_streaming topic."""
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            for topic in topics:
                if 'enable_streaming' in topic.lower() and 'camera' in topic.lower():
                    return topic
        return None
    except Exception as e:
        print(f"Error finding camera topic: {e}")
        return None

def enable_camera_stream():
    """Enable the camera stream in Gazebo."""
    print("🔍 Finding camera enable topic...")
    
    topic = find_camera_topic()
    
    if not topic:
        print("❌ No camera enable topic found!")
        print("   Is Gazebo running with a camera model?")
        return False
    
    print(f"✓ Found topic: {topic}")
    print("📹 Enabling camera stream...")
    
    try:
        result = subprocess.run(
            ["gz", "topic", "-t", topic, "-m", "gz.msgs.Boolean", "-p", "data: 1"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode == 0:
            print("✓ Camera stream enabled!")
            return True
        else:
            print(f"❌ Failed to enable camera: {result.stderr}")
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ Timeout while enabling camera")
        return False
    except FileNotFoundError:
        print("❌ 'gz' command not found. Is Gazebo installed?")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    # Can be run standalone for testing
    if enable_camera_stream():
        print("\n✓ Camera is ready!")
    else:
        print("\n❌ Failed to enable camera")
        exit(1)
