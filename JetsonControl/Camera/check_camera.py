#!/usr/bin/env python3

"""
Check camera capabilities and available formats
"""

import sys
import gi
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

def check_camera_formats(device):
    """Check what formats the camera supports"""
    
    # Initialize GStreamer
    Gst.init(None)
    
    # Create a v4l2src element
    source = Gst.ElementFactory.make("v4l2src", "camera-source")
    if not source:
        print("Unable to create v4l2src")
        return
    
    source.set_property('device', device)
    
    # Get device capabilities
    pipeline = Gst.Pipeline()
    pipeline.add(source)
    
    # Set to READY state to query capabilities
    pipeline.set_state(Gst.State.READY)
    
    # Get the source pad and query capabilities
    src_pad = source.get_static_pad("src")
    if src_pad:
        caps = src_pad.query_caps(None)
        print(f"Camera {device} capabilities:")
        print(caps.to_string())
    else:
        print("Could not get source pad")
    
    pipeline.set_state(Gst.State.NULL)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 check_camera.py <camera_device_path>")
        print("Example: python3 check_camera.py /dev/video0")
        sys.exit(1)
    
    camera_device = sys.argv[1]
    check_camera_formats(camera_device)

if __name__ == '__main__':
    main()
