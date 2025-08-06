#!/usr/bin/env python3
"""
YOLO Object Detection Camera Viewer with GStreamer Hardware Acceleration
Optimized for minimal latency on Jetson devices with bottle detection

Usage:
    python3 camera3.py

Requirements:
    - OpenCV with GStreamer support: pip install opencv-python
    - Ultralytics YOLO: pip install ultralytics
    - PyTorch: pip install torch torchvision
    - Camera device at /dev/video0
    - YOLO model: yolo10b_trained.pt
    - For Jetson devices: Hardware acceleration support

Controls:
    - Press 'q' or ESC to quit
    - Press 's' to toggle detection on/off
"""

import cv2
import sys
import os
import time
import numpy as np
from ultralytics import YOLO

def main():
    # Load YOLO model
    model_path = "yolo10b_trained.pt"
    if not os.path.exists(model_path):
        print(f"Error: YOLO model not found at {model_path}")
        print("Make sure yolo10b_trained.pt is in the same directory as this script")
        return -1
    
    print(f"Loading YOLO model from {model_path}...")
    try:
        model = YOLO(model_path)
        print("YOLO model loaded successfully!")
    except Exception as e:
        print(f"Error loading YOLO model: {e}")
        return -1
    
    # GStreamer pipeline with hardware acceleration for minimal latency
    # Optimized resolution for better performance while maintaining quality
    
    # Primary pipeline for USB camera (/dev/video0) with hardware acceleration
    gst_pipeline_hw = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw(memory:NVMM) ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1" 
    )
    
    # Fallback pipeline without hardware acceleration
    gst_pipeline_sw = (
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,width=640,height=480,framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )
    
    print("Opening camera with GStreamer pipeline...")
    
    # Try hardware accelerated pipeline first
    print("Trying hardware accelerated pipeline...")
    cap = cv2.VideoCapture(gst_pipeline_hw, cv2.CAP_GSTREAMER)
    
    if not cap.isOpened():
        print("Hardware acceleration failed, trying software pipeline...")
        cap = cv2.VideoCapture(gst_pipeline_sw, cv2.CAP_GSTREAMER)
        
        if not cap.isOpened():
            print("Error: Cannot open camera!")
            print("Make sure /dev/video0 exists and is accessible")
            return -1
    
    # Set buffer size to 1 for minimal latency
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    print("Camera opened successfully!")
    print("Controls:")
    print("  Press 'q' or ESC to quit")
    print("  Press 's' to toggle detection on/off")
    print("  Press 'c' to show confidence scores")
    
    window_name = "YOLO Detection - /dev/video0"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    
    # Performance tracking
    fps_counter = 0
    fps_start_time = time.time()
    detection_enabled = True
    show_confidence = True
    
    # Colors for different classes (bottles will be green)
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]
    
    try:
        while True:
            ret, frame = cap.read()
            
            if not ret:
                print("Error: Failed to capture frame")
                break
            
            # Run YOLO detection if enabled
            if detection_enabled:
                try:
                    # Run inference with optimized settings for speed
                    results = model(frame, verbose=False, conf=0.5, iou=0.45)
                    
                    # Draw detections
                    for result in results:
                        boxes = result.boxes
                        if boxes is not None:
                            for box in boxes:
                                # Get coordinates
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                                confidence = box.conf[0].cpu().numpy()
                                class_id = int(box.cls[0].cpu().numpy())
                                
                                # Get class name
                                class_name = model.names[class_id] if class_id < len(model.names) else f"Class_{class_id}"
                                
                                # Choose color
                                color = colors[class_id % len(colors)]
                                
                                # Draw bounding box
                                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                                
                                # Draw label with confidence
                                if show_confidence:
                                    label = f"{class_name}: {confidence:.2f}"
                                else:
                                    label = class_name
                                
                                # Calculate text size and position
                                (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                                
                                # Draw background rectangle for text
                                cv2.rectangle(frame, (x1, y1 - text_height - 10), (x1 + text_width, y1), color, -1)
                                
                                # Draw text
                                cv2.putText(frame, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                except Exception as e:
                    print(f"Detection error: {e}")
            
            # Calculate and display FPS
            fps_counter += 1
            if fps_counter % 30 == 0:  # Update FPS every 30 frames
                fps = 30 / (time.time() - fps_start_time)
                fps_start_time = time.time()
                print(f"FPS: {fps:.1f} | Detection: {'ON' if detection_enabled else 'OFF'}")
            
            # Display FPS on frame
            cv2.putText(frame, f"FPS: {fps_counter % 30}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if not detection_enabled:
                cv2.putText(frame, "DETECTION OFF", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display the frame
            cv2.imshow(window_name, frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:  # 'q' or ESC
                break
            elif key == ord('s'):  # Toggle detection
                detection_enabled = not detection_enabled
                print(f"Detection {'enabled' if detection_enabled else 'disabled'}")
            elif key == ord('c'):  # Toggle confidence display
                show_confidence = not show_confidence
                print(f"Confidence scores {'shown' if show_confidence else 'hidden'}")
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        print("Camera released and windows closed")
    
    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
