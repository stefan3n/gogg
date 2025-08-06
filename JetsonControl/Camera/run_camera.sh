#!/bin/bash
# Script to run YOLO camera detection
# Make sure all dependencies are installed

echo "YOLO Camera Detection with Hardware Acceleration"
echo "==============================================="

# Check if YOLO model exists
if [ ! -f "yolo10b_trained.pt" ]; then
    echo "Error: yolo10b_trained.pt not found!"
    echo "Please place the YOLO model file in this directory."
    exit 1
fi

# Check if requirements are installed
echo "Checking dependencies..."
python3 -c "import cv2, ultralytics, torch" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip3 install -r requirements.txt
fi

echo "Starting YOLO camera detection..."
echo "Controls:"
echo "  - Press 'q' or ESC to quit"
echo "  - Press 's' to toggle detection on/off"
echo "  - Press 'c' to toggle confidence scores"
echo ""

python3 camera3.py
