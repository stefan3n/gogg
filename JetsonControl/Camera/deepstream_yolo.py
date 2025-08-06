#!/usr/bin/env python3

"""
DeepStream YOLO implementation for ultra-low latency object detection
This implementation uses NVIDIA DeepStream SDK for hardware-accelerated inference
"""

import sys
import gi
import traceback
gi.require_version('Gst', '1.0')
from gi.repository import GObject, Gst
import pyds
import cv2
import numpy as np
from math import cos, sin
import time

# Global variables
frame_count = 0
saved_count = {"stream_0": 0}
fps_start_time = time.time()
fps_counter = 0
current_fps = 0.0

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

def osd_sink_pad_buffer_probe(pad, info, u_data):
    """Probe function to process frames and draw bounding boxes"""
    global frame_count, fps_counter, fps_start_time, current_fps, saved_count
    
    gst_buffer = info.get_buffer()
    if not gst_buffer:
        print("Unable to get GstBuffer ")
        return

    # Retrieve batch metadata from the gst_buffer
    batch_meta = pyds.gst_buffer_get_nvds_batch_meta(hash(gst_buffer))
    l_frame = batch_meta.frame_meta_list
    
    while l_frame is not None:
        try:
            frame_meta = pyds.NvDsFrameMeta.cast(l_frame.data)
        except StopIteration:
            break

        frame_count += 1
        fps_counter += 1
        
        # Calculate FPS
        if fps_counter >= 30:
            fps_end_time = time.time()
            current_fps = fps_counter / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
            fps_counter = 0

        # Get frame dimensions
        frame_width = frame_meta.source_frame_width
        frame_height = frame_meta.source_frame_height
        
        # Add FPS display metadata
        display_meta = pyds.nvds_acquire_display_meta_from_pool(batch_meta)
        py_nvosd_text_params = display_meta.text_params[0]
        py_nvosd_text_params.display_text = f"FPS: {current_fps:.1f}"
        py_nvosd_text_params.x_offset = 10
        py_nvosd_text_params.y_offset = 12
        py_nvosd_text_params.font_params.font_name = "Serif"
        py_nvosd_text_params.font_params.font_size = 20
        py_nvosd_text_params.font_params.font_color.red = 1.0
        py_nvosd_text_params.font_params.font_color.green = 1.0
        py_nvosd_text_params.font_params.font_color.blue = 0.0
        py_nvosd_text_params.font_params.font_color.alpha = 1.0
        py_nvosd_text_params.set_bg_clr = 1
        py_nvosd_text_params.text_bg_clr.red = 0.0
        py_nvosd_text_params.text_bg_clr.green = 0.0
        py_nvosd_text_params.text_bg_clr.blue = 0.0
        py_nvosd_text_params.text_bg_clr.alpha = 1.0
        pyds.nvds_add_display_meta_to_frame(frame_meta, display_meta)

        # Iterate through object metadata
        l_obj = frame_meta.obj_meta_list
        while l_obj is not None:
            try:
                obj_meta = pyds.NvDsObjectMeta.cast(l_obj.data)
            except StopIteration:
                break
                
            # Draw custom bounding boxes with better visibility
            rect_params = obj_meta.rect_params
            rect_params.border_width = 3
            rect_params.border_color.red = 0.0
            rect_params.border_color.green = 1.0
            rect_params.border_color.blue = 0.0
            rect_params.border_color.alpha = 1.0
            
            # Custom text parameters for better readability
            text_params = obj_meta.text_params
            text_params.font_params.font_size = 12
            text_params.font_params.font_color.red = 1.0
            text_params.font_params.font_color.green = 1.0
            text_params.font_params.font_color.blue = 1.0
            text_params.font_params.font_color.alpha = 1.0
            text_params.set_bg_clr = 1
            text_params.text_bg_clr.red = 0.0
            text_params.text_bg_clr.green = 0.0
            text_params.text_bg_clr.blue = 0.0
            text_params.text_bg_clr.alpha = 1.0

            try:
                l_obj = l_obj.next
            except StopIteration:
                break

        # Update saved count
        saved_count["stream_0"] += 1
        
        try:
            l_frame = l_frame.next
        except StopIteration:
            break

    return Gst.PadProbeReturn.OK

def main():
    """Main function to create and run the DeepStream pipeline"""
    
    # Check for required arguments
    if len(sys.argv) < 2:
        print("Usage: python3 deepstream_yolo.py <camera_device_path>")
        print("Example: python3 deepstream_yolo.py /dev/video0")
        sys.exit(1)
    
    camera_device = sys.argv[1]
    
    # Standard GStreamer initialization
    GObject.threads_init()
    Gst.init(None)

    # Create gstreamer elements
    print("Creating Pipeline")
    pipeline = Gst.Pipeline()

    if not pipeline:
        sys.stderr.write(" Unable to create Pipeline \n")
        sys.exit(1)

    # Source element for camera input
    print("Creating Source")
    source = Gst.ElementFactory.make("v4l2src", "camera-source")
    if not source:
        sys.stderr.write(" Unable to create Source \n")
        sys.exit(1)
    
    source.set_property('device', camera_device)
    
    # Caps filter for camera input
    caps_camera = Gst.ElementFactory.make("capsfilter", "camera-caps")
    caps_camera.set_property("caps", Gst.Caps.from_string("video/x-raw, width=640, height=480, framerate=30/1"))

    # Video converter
    videoconvert1 = Gst.ElementFactory.make("videoconvert", "videoconvert1")
    if not videoconvert1:
        sys.stderr.write(" Unable to create videoconvert1 \n")
        sys.exit(1)

    # nvvideoconvert to convert incoming raw buffers to NVMM Mem (NvBufSurface API)
    nvvidconv = Gst.ElementFactory.make("nvvideoconvert", "convertor")
    if not nvvidconv:
        sys.stderr.write(" Unable to create nvvidconv \n")
        sys.exit(1)

    # Create nvstreammux instance to form batches from one or more sources
    streammux = Gst.ElementFactory.make("nvstreammux", "stream-muxer")
    if not streammux:
        sys.stderr.write(" Unable to create NvStreamMux \n")
        sys.exit(1)
    
    # Set streammux properties
    streammux.set_property('width', 640)
    streammux.set_property('height', 480)
    streammux.set_property('batch-size', 1)
    streammux.set_property('batched-push-timeout', 4000000)

    # Create the nvinfer element for object detection
    print("Creating Primary GIE")
    pgie = Gst.ElementFactory.make("nvinfer", "primary-inference")
    if not pgie:
        sys.stderr.write(" Unable to create pgie \n")
        sys.exit(1)

    # Use nvinfer to run inferencing on decoder's output, behaviour of inferencing is set through config file
    pgie.set_property('config-file-path', "config_simple_test.txt")

    # Use convertor to convert from NV12 to RGBA as required by nvosd
    nvvidconv_postosd = Gst.ElementFactory.make("nvvideoconvert", "convertor_postosd")
    if not nvvidconv_postosd:
        sys.stderr.write(" Unable to create nvvidconv_postosd \n")
        sys.exit(1)

    # Create OSD to draw on the converted RGBA buffer
    nvosd = Gst.ElementFactory.make("nvdsosd", "onscreendisplay")
    if not nvosd:
        sys.stderr.write(" Unable to create nvosd \n")
        sys.exit(1)

    # Convert to format for display - REMOVE THIS ELEMENT
    # nvvidconv2 = Gst.ElementFactory.make("nvvideoconvert", "convertor2")
    
    # REMOVE caps_display element as well
    # caps_display = Gst.ElementFactory.make("capsfilter", "display-caps")

    # Video converter for final output
    videoconvert2 = Gst.ElementFactory.make("videoconvert", "videoconvert2")
    if not videoconvert2:
        sys.stderr.write(" Unable to create videoconvert2 \n")
        sys.exit(1)

    # Create a sink for display - Use autovideosink for better compatibility
    print("Creating Video Sink")
    sink = Gst.ElementFactory.make("autovideosink", "videosink")
    if not sink:
        print("Trying xvimagesink...")
        sink = Gst.ElementFactory.make("xvimagesink", "xvimagesink")
        if not sink:
            sys.stderr.write(" Unable to create video sink \n")
            sys.exit(1)

    sink.set_property("sync", False)

    print("Adding elements to Pipeline")
    pipeline.add(source)
    pipeline.add(caps_camera)
    pipeline.add(videoconvert1)
    pipeline.add(nvvidconv)
    pipeline.add(streammux)
    pipeline.add(pgie)
    pipeline.add(nvvidconv_postosd)
    pipeline.add(nvosd)
    pipeline.add(videoconvert2)
    pipeline.add(sink)

    # Link the elements together - UPDATED LINKING
    print("Linking elements in the Pipeline")
    if not source.link(caps_camera):
        sys.stderr.write("Unable to link source to caps_camera \n")
        sys.exit(1)
    if not caps_camera.link(videoconvert1):
        sys.stderr.write("Unable to link caps_camera to videoconvert1 \n")
        sys.exit(1)
    if not videoconvert1.link(nvvidconv):
        sys.stderr.write("Unable to link videoconvert1 to nvvidconv \n")
        sys.exit(1)
    
    # Get sink pad of streammux and link nvvidconv to it
    sinkpad = streammux.get_request_pad("sink_0")
    if not sinkpad:
        sys.stderr.write("Unable to get the sink pad of streammux \n")
        sys.exit(1)
    srcpad = nvvidconv.get_static_pad("src")
    if not srcpad:
        sys.stderr.write("Unable to get source pad of nvvidconv \n")
        sys.exit(1)
    if srcpad.link(sinkpad) != Gst.PadLinkReturn.OK:
        sys.stderr.write("Unable to link nvvidconv to streammux \n")
        sys.exit(1)
    
    if not streammux.link(pgie):
        sys.stderr.write("Unable to link streammux to pgie \n")
        sys.exit(1)
    if not pgie.link(nvvidconv_postosd):
        sys.stderr.write("Unable to link pgie to nvvidconv_postosd \n")
        sys.exit(1)
    if not nvvidconv_postosd.link(nvosd):
        sys.stderr.write("Unable to link nvvidconv_postosd to nvosd \n")
        sys.exit(1)
    if not nvosd.link(videoconvert2):
        sys.stderr.write("Unable to link nvosd to videoconvert2 \n")
        sys.exit(1)
    if not videoconvert2.link(sink):
        sys.stderr.write("Unable to link videoconvert2 to sink \n")
        sys.exit(1)

    # Create an event loop and feed gstreamer bus messages to it
    loop = GObject.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", bus_call, loop)

    # Add probe to get informed of the meta data generated, we add probe to the sink pad of the osd element
    osdsinkpad = nvosd.get_static_pad("sink")
    if not osdsinkpad:
        sys.stderr.write(" Unable to get sink pad of nvosd \n")
        sys.exit(1)

    osdsinkpad.add_probe(Gst.PadProbeType.BUFFER, osd_sink_pad_buffer_probe, 0)

    # Start playing
    print("Starting pipeline")
    pipeline.set_state(Gst.State.PLAYING)
    
    print("DeepStream pipeline started. Press Ctrl+C to stop.")
    
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
