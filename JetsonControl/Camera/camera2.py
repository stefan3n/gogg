import cv2
import os
import torch
import time
import threading
import queue
from ultralytics import YOLO
import numpy as np

# Clear CUDA cache to free memory
torch.cuda.empty_cache()

class CameraProcessor:
    def __init__(self, model_path, camera_name=None, camera_index=0):
        # Initialize queues for thread communication
        self.frame_queue = queue.Queue(maxsize=1)  # Only keep latest frame
        self.result_queue = queue.Queue(maxsize=1)  # Only keep latest result
        
        # Load YOLO model
        self.model = self.load_yolo_model(model_path)
        if self.model is None:
            raise RuntimeError("Failed to load YOLO model")
            
        # Optimize model
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.model.model.half()  # Use FP16
            print(f"Using CUDA with FP16 (device: {torch.cuda.get_device_name(0)})")
            
        # Camera setup
        self.device_path = self.find_video_device_by_name(camera_name) if camera_name else f"/dev/video{camera_index}"
        
        # FPS tracking
        self.fps_capture = 0
        self.fps_inference = 0
        self.fps_display = 0
        
        # Thread control
        self.running = False
        self.threads = []
        
    def start(self):
        self.running = True
        
        # Start threads
        capture_thread = threading.Thread(target=self.capture_loop)
        inference_thread = threading.Thread(target=self.inference_loop)
        display_thread = threading.Thread(target=self.display_loop)
        
        self.threads = [capture_thread, inference_thread, display_thread]
        for thread in self.threads:
            thread.daemon = True
            thread.start()
            
        print("Camera processor started with threaded architecture")
        
    def stop(self):
        self.running = False
        for thread in self.threads:
            thread.join(timeout=1.0)
        print("Camera processor stopped")
        
    def capture_loop(self):
        # Try GStreamer first, fallback to OpenCV
        pipeline = self.gstreamer_pipeline(self.device_path, 
                                         capture_width=320, 
                                         capture_height=240,
                                         framerate=60)
        
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            print("GStreamer failed, falling back to OpenCV")
            cap = cv2.VideoCapture(self.device_path)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FPS, 60)
            
        if not cap.isOpened():
            print("Failed to open camera")
            self.running = False
            return
            
        # Skip initial frames (camera warm-up)
        for _ in range(10):
            cap.read()
            
        # FPS measurement
        frame_count = 0
        fps_start = time.time()
        
        try:
            while self.running:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read frame")
                    time.sleep(0.01)
                    continue
                    
                # Update capture FPS
                frame_count += 1
                if frame_count >= 30:
                    now = time.time()
                    self.fps_capture = frame_count / (now - fps_start)
                    fps_start = now
                    frame_count = 0
                    
                # Put latest frame in queue, replacing any old frame
                try:
                    self.frame_queue.put(frame, block=False)
                except queue.Full:
                    # If queue is full, remove old frame and add new one
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put(frame, block=False)
                    except:
                        pass
        finally:
            cap.release()
            
    def inference_loop(self):
        last_frame = None
        last_result = None
        
        # FPS measurement
        frame_count = 0
        fps_start = time.time()
        
        while self.running:
            try:
                frame = self.frame_queue.get(timeout=0.1)
                last_frame = frame
            except queue.Empty:
                time.sleep(0.001)
                continue
                
            try:
                # Resize to tiny resolution for faster inference
                tiny_frame = cv2.resize(frame, (160, 120))
                
                # Run inference with optimized settings
                with torch.no_grad():
                    results = self.model(tiny_frame, verbose=False, imgsz=160,
                                       conf=0.5, iou=0.5)
                
                # Scale results back to original frame size
                if results and len(results) > 0 and results[0].boxes is not None:
                    scale_x = frame.shape[1] / 160
                    scale_y = frame.shape[0] / 120
                    
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    boxes[:, [0, 2]] *= scale_x
                    boxes[:, [1, 3]] *= scale_y
                    
                    last_result = {
                        'boxes': boxes,
                        'confidences': results[0].boxes.conf.cpu().numpy(),
                        'classes': results[0].boxes.cls.cpu().numpy(),
                        'names': results[0].names,
                        'frame': frame
                    }
                    
                    # Put result in queue for display thread
                    try:
                        self.result_queue.put(last_result, block=False)
                    except queue.Full:
                        try:
                            self.result_queue.get_nowait()
                            self.result_queue.put(last_result, block=False)
                        except:
                            pass
                
                # Update inference FPS
                frame_count += 1
                if frame_count >= 10:
                    now = time.time()
                    self.fps_inference = frame_count / (now - fps_start)
                    fps_start = now
                    frame_count = 0
                    
                # Free GPU memory
                torch.cuda.empty_cache()
                
            except Exception as e:
                print(f"Inference error: {e}")
                time.sleep(0.01)
                
    def display_loop(self):
        last_result = None
        
        # FPS measurement
        frame_count = 0
        fps_start = time.time()
        
        while self.running:
            try:
                result = self.result_queue.get(timeout=0.1)
                last_result = result
            except queue.Empty:
                if last_result is None:
                    time.sleep(0.01)
                    continue
            
            if last_result:
                frame = last_result['frame'].copy()
                
                # Draw detections
                frame = self.draw_scaled_detections(frame, last_result)
                
                # Add FPS information
                cv2.putText(frame, f"Capture: {self.fps_capture:.1f} fps", (10, 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame, f"Inference: {self.fps_inference:.1f} fps", (10, 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                cv2.putText(frame, f"Display: {self.fps_display:.1f} fps", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                
                # Display frame
                cv2.imshow("YOLO Detection - Optimized", frame)
                
                # Update display FPS
                frame_count += 1
                if frame_count >= 30:
                    now = time.time()
                    self.fps_display = frame_count / (now - fps_start)
                    fps_start = now
                    frame_count = 0
            
            # Check for exit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break
                
    # Helper methods (include your existing methods here)
    def load_yolo_model(self, model_path):
        """Load YOLO model with error handling"""
        try:
            print(f"Loading YOLO model from: {model_path}")
            model = YOLO(model_path)
            print("YOLO model loaded successfully!")
            return model
        except Exception as e:
            print(f"Error loading YOLO model: {e}")
            return None
            
    def find_video_device_by_name(self, name):
        """Find the video device path by name in /dev/v4l/by-id/."""
        # Your existing method
        # ...

    def gstreamer_pipeline(self, device_path, capture_width=640, capture_height=480, 
                         display_width=640, display_height=480, framerate=30, flip_method=0):
        """GStreamer pipeline for capturing video with hardware acceleration."""
        # Your existing method
        # ...
        
    def draw_scaled_detections(self, frame, detection_data):
        """Draw bounding boxes and labels from detection data"""
        # Your existing method
        # ...

def main():
    try:
        processor = CameraProcessor(
            model_path="../aiModels/yolov10n_trained.pt",
            camera_name="usb-046d_0825_5AA5590-video-index0"
        )
        processor.start()
        
        # Main thread just waits for keyboard interrupt
        while True:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'processor' in locals():
            processor.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()