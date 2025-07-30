# Simple code that should open the camera and show what it's being seen.
# Press Q to quit.
# Change the name in the VideoCapture (ls /dev | grep video in terminal to see all connected cameras).
# For CSI camera uncomment the lines below and call gstreamer_pipeline instead of "/dev/video2" and replace cv2.CAP_V4L2 with
# cv2.CAP_GSTREAMER
# It should work out of the box.

# import os
# import sys
# sys.path.append(os.path.relpath("../"))
# from camera.help_camera import gstreamer_pipeline


import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

if not cap.isOpened():  # Check if camera opened successfully
    print("Unable to open camera")
    exit()

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            break
        
        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) == ord('q'):  # Press 'q' to quit
            break
finally:
    # Release the capture when everything is done
    cap.release()
    cv2.destroyAllWindows()