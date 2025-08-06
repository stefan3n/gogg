import cv2
import numpy as np
import time
import os

class Camera:
    def __init__(self, camera_name):
        self.camera_name = camera_name
        device_path = self.find_video_device_by_name(camera_name)
        if not device_path:
            raise Exception(f"Camera with name '{camera_name}' not found.")

        # Create GStreamer pipeline string
        self.gst_str = (
            f'v4l2src device={device_path} ! '
            'video/x-raw, width=640, height=480, framerate=30/1 ! '
            'nvvidconv ! '
            'video/x-raw(memory:NVMM), format=NV12 ! '
            'nvvidconv ! '
            'video/x-raw, format=BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=BGR ! '
            'appsink drop=1 max-buffers=1 sync=0'
        )

        print(f"[GStreamer] Initializing camera '{camera_name}' with: {self.gst_str}")
        try:
            self.cap = cv2.VideoCapture(self.gst_str, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                print(f"[Camera] '{camera_name}' opened with GStreamer pipeline.")
            else:
                raise Exception("Failed to open with GStreamer")
        except Exception as e:
            print(f"[GStreamer] ERROR: {e}")
            # Fallback to standard OpenCV capture
            print(f"[GStreamer] Falling back to standard capture for camera '{camera_name}'")
            self.cap = cv2.VideoCapture(device_path)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if self.cap.isOpened():
                print(f"[Camera] '{camera_name}' opened with standard OpenCV capture.")
            else:
                print(f"[Camera] ERROR: '{camera_name}' could not be opened with fallback either.")

    @staticmethod
    def find_video_device_by_name(name):
        by_id_path = "/dev/v4l/by-id/"
        if not os.path.exists(by_id_path):
            print("[Camera Utils] /dev/v4l/by-id/ does not exist. Falling back to default device paths.")
            return None

        for device in os.listdir(by_id_path):
            if name in device:  # 
                device_path = os.path.join(by_id_path, device)
                real_path = os.path.realpath(device_path) 
                print(f"[Camera Utils] Found device '{name}' at {real_path}")
                return real_path

        print(f"[Camera Utils] No device found with name '{name}'")
        return None

    def get_frame(self):
        """Capture a single frame from the camera."""
        if not self.cap.isOpened():
            print(f"[Camera] Camera '{self.camera_name}' is not opened.")
            return None

        ret, frame = self.cap.read()
        if not ret:
            print(f"[Camera] Failed to capture frame from '{self.camera_name}'.")
            return None

        # Encode frame as JPEG
        _, jpeg = cv2.imencode('.jpg', frame)
        return jpeg.tobytes()

# Cache for camera instances to avoid reopening
_camera_cache = {}

def get_camera(camera_id):
    """Get a camera instance from cache or create a new one"""
    if camera_id not in _camera_cache:
        _camera_cache[camera_id] = Camera(camera_id)
    return _camera_cache[camera_id]

def generate_frames(camera_id):
    """Frame generator for video streaming"""
    camera = get_camera(camera_id)
    while True:
        frame = camera.get_frame()
        if frame is None:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def cleanup():
    """Free resources for all cameras"""
    for camera_id, camera in _camera_cache.items():
        print(f"Releasing camera {camera_id}")
        camera.release()
    _camera_cache.clear()