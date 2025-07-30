# pip3 install pyserial
import serial
import serial.tools
import serial.tools.list_ports

# pip3 install v4l2ctl
import v4l2ctl

from time import time
import logging
from os import listdir,kill, rename
from signal import SIGINT


from define.define import *
# This module contains simple functions that help the other processes


def exchangeFD(file_1, file_2):
    '''
    The files exchange names. Returns a tuple. Used in findDevices to ensure lidar is always /dev/ttyUSB0.\n
    The code must be ran as sudo for it to work.
    '''
    old_1 = file_1
    old_2 = file_2
    rename(file_2, "/dev/OS-ul nu va genera asa ceva niciodata sperrr")
    rename(file_1, old_2)
    rename("/dev/OS-ul nu va genera asa ceva niciodata sperrr", old_1)
    return file_2, file_1


# Returns ttyUSBx in order: robot, motors, lidar, robotCamera, motorsCamera
# Needed because devices change names sometimes (i.e ttyUSB0 will not always be the motors arduino)
def findDevices():
    '''
    Identifies the devices names (/dev/ttyUSBx /dev/videoX etc.) based on some constants.\n
    Returns a list: [robot, motors, lidar, motorsCamera,robotCamera]\n
    Lidar is always /dev/ttyUSB0
    '''
    robot = None
    motors = None
    lidar = None
    robotCamera = None
    motorsCamera = None

    for i in serial.tools.list_ports.comports():
        if i.pid == PID_ROBOT and i.vid == VID_ROBOT:
            robot = "/dev/" + i.name

        if i.pid == PID_MOTORS and i.vid == VID_MOTORS:
            motors = "/dev/" + i.name

        if i.pid == PID_LIDAR and i.vid == VID_LIDAR:
            lidar = "/dev/" + i.name
    devices = listdir("/dev")
    devices = [i for i in devices if i.startswith('video')]
    devices =['/dev/'+i for i in devices]


    for i in devices:
        device = v4l2ctl.V4l2Device(i)

        if str(device.supported_buffer_types) == SUPPORTED_BUFFER:
            if device.name == CAR_CAMERA:
                motorsCamera = i
            if device.name == ROBOT_CAMERA:
                robotCamera = i

    # Not the best way 
    # ROS needs the lidar to always be /dev/ttyUSB0
    # We just exchange the names of lidar and whoever is names /dev/ttyUSB0
    if robot == "/dev/ttyUSB0":
        robot=lidar
        lidar ="/dev/ttyUSB0"
  
    ret_list =[robot, motors, lidar, robotCamera, motorsCamera]
    # if lidar != "/dev/ttyUSB0":
    #     index = 0
    #     for i in range(0,5):
    #         if ret_list[i] == "/dev/ttyUSB0":
    #             index = i
    #             break
    #     ret_list[2], ret_list[index] = exchangeFD(ret_list[2], ret_list[index])

    print(ret_list)
    return ret_list


# Function to be called at the start of the main after identifying the ports.
# Will end execution of main without starting other processes.
# If one component is missing the car will not start.
def checkPorts(ports, logger):
    '''
    If one device is missing exit.
    '''
    for i, j in enumerate(ports):
        if j == None:
            if i == 0:
                logger.critical("Missing robot. Ending execution")
                exit(1)
            if i == 1:
                logger.critical("Missing motors. Ending execution")
                exit(1)
            if i == 2:
                logger.critical("Missing lidar. Ending execution")
                exit(1)
            if i == 3:
                logger.critical("Missing robot camera. Ending execution.")
                exit(1)
            if i == 4:
                logger.critical("Missing car camera. Ending execution.")
                exit(1)





# Time in milliseconds.
# Used to mimic the millis() from arduino.
# Needed for calculating timeouts.
def getTime():
    '''
    Returns time in milliseconds.
    '''
    return time() * 1000

# Used to make loggers for different purposes based on a name.
# Needed to make the code simpler and easier to read.
def setupLogger(name):
    '''
    Returns a logger that logs in the /logs/name.log.
    '''
    logger = logging.getLogger(name)
    logger.setLevel(logging.INFO)
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file = logging.FileHandler(f"../logs/{name}.log")
    file.setFormatter(formatter)
    file.mode = 'w'
    logger.addHandler(file)
    logger.propagate = False
    return logger

# Sends SIGINT to a list of pids
# If the pid is not valid, go on to the next pid in the list
# Needed by processes to tell the others that they should shut down.
def sendSIGINT(list):
    '''
    Send SIGINT to the pids in the list.
    '''
    for i in list:
        try:
            kill(i, SIGINT)
        except:
            pass
