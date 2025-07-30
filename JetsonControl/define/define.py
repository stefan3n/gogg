# pip3 install numpy
# should come with ultralytics
import numpy as np

# Module that contains enums and other constants that are being used by arduinos and main


TIMEOUT_SERVER = 5 * 60 * 1000  #ms (5 min)
ARDUINO_BAUDRATE = 9600
TIMEOUT_ARDUINO = 9999.0           #s

# Enum used on arduino. Needed here to be able to send commands that can be interpreted correctly.
class MotorCommands:

    START = 1
    SET_PWM = 2
    BREAK = 3
    SEND_WHEEL_SPEEDS = 6

# Common commands used by motors and robot
class ArduinoCommands:
    PING = 0
    EMERGENCY = 4
    END_EMERGENCY = 5


# Enum used on arduino. Needed here to be able to send commands that can be interpreted correctly.
class RobotCommands:

    STOP = 1
    MOVE_ONE = 2
    MOVE_ALL = 3
    ROTATE_RIGHT = 6
    ROTATE_LEFT = 7
    SEND_START_POSITION = 8
    INITIAL_POS = 9
    CALIBRATE_LEFT = 10
    CALIBRATE_RIGHT = 11
    CLAW_OPEN = 12
    CLAW_CLOSE = 13
    # ROTATE_CLAW = 14 | WE DON'T USE ROTATION ANYMORE
    FINAL_ARM = 15
    


# This is what the server sends to main. Needed here to recognize commands and send the right thing
# to the right arduino.
class ServerCommands:
    class Motors:
        TURN_LEFT = "turnLeft"
        TURN_RIGHT = "turnRight"
        FORWARD = "forward"
        BACKWARD = "backward"
        BREAK = "break"

        START = "start"
        WHEEL_SPEEDS ="wheelSpeeds"
    
    class Robot:
        MOVE_ONE = "moveOne"
        MOVE_ALL = "moveAll"
        INITIAL_POS = "initialPos"
        ROTATE_LEFT = "rotateLeft"
        ROTATE_RIGHT = "rotateRight"
        STOP_ROBOT = "stopRobot"

        CAL_LEFT = "calLeft"
        CAL_RIGHT = "calRight"

        # ROTATE_CLAW = "clawRotate" | WE DON'T USE ROTATION ANYMORE
        CLAW_OPEN = "clawOpen"
        CLAW_CLOSE = "clawClose"
    
    EXIT ="exit"
    AUTONOMOUS = "autonomous"
    MANUAL = "manual"
    EMERGENCY = "emergency"
    END_EMERGENCY = "endEmergency"
    KEEP_ALIVE = "keep_alive"
    GET_MAP = "get_map"

# Motor arduino states
class MotorStates:
    IDLE = 0
    RUNNING = 1
    EMERGENCY = 2

# The .npz files have been generated using the /tests/cameraCalib.py using np.savez("name", arr=arr to save)
# These matrices are needed to undistort the image from the webcam.
# Note before using any member of the USB/CSI class you NEED to call the load() method like this USBCamera.load() or CSICamera.load() (for each class)
# load() can be called multiple times.
# Not thread safe. We have used this only in the processes related to the cameras.
class RobotCamera:
    rvecs=None
    tvecs = None
    mtx = None
    dist = None
    camera_matrix = None
    def load():
        RobotCamera.rvecs= np.load('../matrix/rvecs_robot.npz')['arr']
        RobotCamera.tvecs= np.load('../matrix/tvecs_robot.npz')['arr']
        RobotCamera.mtx= np.load('../matrix/mtx_robot.npz')['arr']
        RobotCamera.dist= np.load('../matrix/dist_robot.npz')['arr']
        RobotCamera.camera_matrix=  np.load('../matrix/camera_matrix_robot.npz')['arr']

# Unused. Do not use.
class CarCamera:
    rvecs=None
    tvecs = None
    mtx = None
    dist = None
    camera_matrix = None
    def load():
        CarCamera.rvecs= np.load('../matrix/rvecs_csi.npz')['arr']
        CarCamera.tvecs= np.load('../matrix/tvecs_csi.npz')['arr']
        CarCamera.mtx= np.load('../matrix/mtx_csi.npz')['arr']
        CarCamera.dist= np.load('../matrix/dist_csi.npz')['arr']
        CarCamera.camera_matrix=  np.load('../matrix/camera_matrix_csi.npz')['arr']


hostName = "0.0.0.0"
commands_port = 8080
robot_port = 8081
car_port = 8082
map_port = 8083

# DEVICES IDS
PID_ROBOT = 29987
VID_ROBOT = 6790

PID_MOTORS = 67
VID_MOTORS = 10755




PID_LIDAR = 60000
VID_LIDAR = 4292

SUPPORTED_BUFFER = "[<V4l2BufferType.VIDEO_CAPTURE: 1>]"
CAR_CAMERA = "HD Webcam C525"
ROBOT_CAMERA ="UVC Camera (046d:0825)"
# These values were generated with /tests/findSerialDevices.py
# They should not change.
# You should make sure they match
