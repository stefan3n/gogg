# We do this to be able to import our modules as
# from folder.file import XYZ
import sys
import os
sys.path.append(os.path.relpath("../"))


# pip3 install ultralytics
from ultralytics import YOLO
# pip3 install opencv-python
import cv2
import traceback

# Our modules
from help.help import sendSIGINT
from camera.help_camera import predict, predict_and_detect, track_and_detect


#   Both cameras:
#   30fps
#   640 x 480
#   0 = bottles on our trained model

# Class for the camera object

class Camera:
    '''
    Camera objects that handles camera devices
    '''
    def __init__(self,draw, pipe, logger, matrices):
        '''
        draw - True : opens a window on screen with the recorded material\n
        pipe - pipe connection with the main\n
        logger - the logger used\n
        matrices - class with a load() method and "static" members for undistorsion\n
        '''
        self.pipe = pipe
        self.draw = draw
        self.camera = None
        self.logger = logger
        self.model = None
        self.pidList = None
        self.matrices = matrices
        self.calc = None

        self.counter_with_bottle = 0
        self.counter_without_bottle = 0
        self.threshold_with_bottle = 10
        self.threshold_without_bottle = 4
        self.skip_rate =4
        self.skip = 0
    
    def setup(self, cv, name, buffersize=1):
        '''
        Method used for opening the camera device and loading the AI model\n
        cv - cv2.CAP_V4L2 or cv2.CAP_GSTREAMER\n
        name - gstreamer_pipeline() or /dev/videoX\n
        buffersize - 1 or a different value for the CSI camera\n

        '''
        self.matrices.load()
        # Try opening the yolo model
        try:
            self.logger.info("Starting YOLOv10n.")
            self.model = YOLO("../aiModels/yolov10b_trained.pt")
            #self.model.to('cuda')
            self.logger.info("Started YOLOV10n.")
        except KeyboardInterrupt:
            self.logger.error("Received SIGINT.")
            exit(1)
        except BaseException as e:
            self.logger.error("Unknown error.")
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            sendSIGINT(self.pidList)
            self.logger.error("Sent SIGINT")
            exit(1)

        # Try opening the camera device
        try:
            self.logger.info("Starting camera.")
            self.camera =  cv2.VideoCapture(name, cv)
            self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0) # turn off autofocus
            self.logger.info("Started camera.")
        except KeyboardInterrupt:
            self.logger.error("Received SIGINT.")
            try:
                self.camera.release()
            except BaseException:
                self.logger.error(traceback.format_exc())
            exit(1)
        except BaseException as e:
            self.logger.error("Error opening the camera.")
            self.logger(e)
            self.logger.error(traceback.format_exc())
            sendSIGINT(self.pidList)
            self.logger("Sent SIGINT.")
            exit(1)
        
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, buffersize)
        self.camera.set(cv2.CAP_PROP_AUTOFOCUS, 0)


    def close(self):
        '''
        Method for gracefully closing the camera
        '''
        self.camera.release()
        if(self.draw):
            cv2.destroyAllWindows()

    def loop(self):
        '''
        Method to be called in an infinite loop. Sends the coordinates of the object to main. Returns the frame read.\n
        '''
        # read the frame

        toBeSent = []
        
        ret_val, img = self.camera.read()
        if ret_val is False:
            return
        if self.skip%self.skip_rate != 0:
            self.skip = self.skip+1
            return
        self.skip = 0
        # undistort the frame
        #img = cv2.undistort(img, self.matrices.mtx, self.matrices.dist, None, self.matrices.camera_matrix)
        boxes = None


        img,boxes= track_and_detect(self.model, img, [0], conf=0.50)
        self.last_data_sent = None
        if boxes != []:
            toBeSent =[]
            # Empty list of tuples.
            # The first element of the tuple is data for motors.
            # The second element of the tuple is data for robot.

            self.counter_without_bottle = 0 
            self.counter_with_bottle += 1

            for box in boxes:
                
                x = box.xyxy[0][0].item() + box.xyxy[0][2].item()

                x = x/2 -320
                y = box.xyxy[0][1].item() + box.xyxy[0][3].item()
                y=-y/2 + 240
                middle = (x,y)
                h = box.xywh[0][3].item()
                motors =[middle, h]
                robot = [(box.xyxy[0][0].item(), box.xyxy[0][1].item()),(box.xyxy[0][2].item(),box.xyxy[0][3].item()) ]
                tuplu = (motors, robot)
                toBeSent.append(tuplu)

            self.last_data_sent = toBeSent
            if self.counter_with_bottle > self.threshold_with_bottle:
                self.pipe.send(toBeSent)
        else:
            self.counter_without_bottle +=1
            if self.counter_without_bottle < self.threshold_without_bottle:
                self.pipe.send(self.last_data_sent)
            else:
                self.counter_with_bottle = 0

        if self.draw:
            window_title = "camera"
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                cv2.imshow(window_title, img)
            else:
                self.close()
                exit(0)
            keyCode = cv2.waitKey(10) & 0xFF
            if keyCode == 27 or keyCode == ord('q'):
                self.close()
                exit(0)
             
        


        
        
        return img

    def getPidList(self):
        '''
        PidList getter.\n
        '''
        return self.pidList 
