#!/usr/bin/env python3

"""
Simple DeepStream test without YOLO inference
"""

import sys
import gi
import traceback
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst

def bus_call(bus, message, loop):
    """Callback function for GStreamer bus messages"""
    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t == Gst.MessageType.WARNING:
        err, debug = message.parse_warning()
        sys.stderr.write("Warning: %s: %s\n" % (err, debug))
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        loop.quit()
    return True

def main():
    """Simple camera test without AI inference"""
    
    if len(sys.argv) < 2:
        print("Usage: python3 test_simple.py <camera_device_path>")
        print("Example: python3 test_simple.py /dev/video0")
        sys.exit(1)
    
    camera_device = sys.argv[1]
    
    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Try different pipeline configurations
    pipelines_to_try = [
        f"v4l2src device={camera_device} ! videoconvert ! autovideosink sync=false",
        f"v4l2src device={camera_device} ! video/x-raw,format=YUY2 ! videoconvert ! autovideosink sync=false",
        f"v4l2src device={camera_device} ! image/jpeg ! jpegdec ! videoconvert ! autovideosink sync=false",
        f"v4l2src device={camera_device} ! video/x-raw,format=MJPG ! jpegdec ! videoconvert ! autovideosink sync=false"
    ]
    
    for i, pipeline_str in enumerate(pipelines_to_try):
        print(f"Trying pipeline {i+1}: {pipeline_str}")
        
        try:
            pipeline = Gst.parse_launch(pipeline_str)
            if pipeline:
                # Test the pipeline
                ret = pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    print(f"Pipeline {i+1} failed to start")
                    pipeline.set_state(Gst.State.NULL)
                    continue
                else:
                    print(f"Pipeline {i+1} started successfully!")
                    break
        except Exception as e:
            print(f"Pipeline {i+1} exception: {e}")
            continue
    else:
        print("All pipeline configurations failed!")
        sys.exit(1)

    if not pipeline:
        sys.stderr.write("Unable to create pipeline\n")
        sys.exit(1)

    # Create an event loop
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    # Start playing
    print("Starting camera test...")
    
    print("Camera pipeline started. Press Ctrl+C to stop.")
    
    try:
        loop.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except:
        print("Error occurred")
        traceback.print_exc()

    # Cleanup
    print("Stopping pipeline")
    pipeline.set_state(Gst.State.NULL)
    print("Exiting")

if __name__ == '__main__':
    main()
