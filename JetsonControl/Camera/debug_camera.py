#!/usr/bin/env python3

"""
Debug camera capabilities
"""

import os
import subprocess
import sys

def check_video_devices():
    """Check available video devices"""
    print("=== Available Video Devices ===")
    
    # List /dev/video* devices
    video_devices = []
    for i in range(10):  # Check video0 to video9
        device = f"/dev/video{i}"
        if os.path.exists(device):
            video_devices.append(device)
            print(f"Found: {device}")
    
    if not video_devices:
        print("No video devices found!")
        return []
    
    return video_devices

def check_device_info(device):
    """Check device information using v4l2-ctl"""
    print(f"\n=== Device Info for {device} ===")
    
    try:
        # Check if device is readable
        result = subprocess.run(['v4l2-ctl', '-d', device, '--list-formats-ext'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("Supported formats:")
            print(result.stdout)
        else:
            print(f"Error getting formats: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("Timeout getting device info")
    except FileNotFoundError:
        print("v4l2-ctl not found. Install with: sudo apt install v4l-utils")
    except Exception as e:
        print(f"Error: {e}")

def test_gstreamer_formats(device):
    """Test different GStreamer formats"""
    print(f"\n=== Testing GStreamer formats for {device} ===")
    
    test_commands = [
        f"gst-launch-1.0 v4l2src device={device} ! video/x-raw ! fakesink",
        f"gst-launch-1.0 v4l2src device={device} ! image/jpeg ! fakesink",
        f"gst-launch-1.0 v4l2src device={device} ! video/x-raw,format=YUY2 ! fakesink",
        f"gst-launch-1.0 v4l2src device={device} ! video/x-raw,format=MJPG ! fakesink"
    ]
    
    for cmd in test_commands:
        print(f"Testing: {cmd}")
        try:
            result = subprocess.run(cmd.split(), capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                print("  ✓ SUCCESS")
            else:
                print(f"  ✗ FAILED: {result.stderr.strip()}")
        except subprocess.TimeoutExpired:
            print("  ✗ TIMEOUT")
        except Exception as e:
            print(f"  ✗ ERROR: {e}")

def main():
    print("Camera Debug Tool")
    print("================")
    
    # Check available devices
    devices = check_video_devices()
    
    if not devices:
        print("No video devices found. Check if camera is connected.")
        return
    
    # Check each device
    for device in devices:
        check_device_info(device)
        test_gstreamer_formats(device)
    
    print("\n=== USB Devices ===")
    try:
        result = subprocess.run(['lsusb'], capture_output=True, text=True)
        usb_lines = [line for line in result.stdout.split('\n') 
                    if any(keyword in line.lower() for keyword in ['camera', 'webcam', 'video', 'capture'])]
        if usb_lines:
            for line in usb_lines:
                print(line)
        else:
            print("No obvious camera devices found in USB list")
    except Exception as e:
        print(f"Error checking USB devices: {e}")

if __name__ == '__main__':
    main()
