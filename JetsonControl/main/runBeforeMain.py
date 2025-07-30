import serial
import serial.tools
import serial.tools.list_ports

from os import listdir, rename
import v4l2ctl

PID_ROBOT = 67
VID_ROBOT = 10755

PID_MOTORS = 29987
VID_MOTORS = 6790

PID_LIDAR = 60000
VID_LIDAR = 4292

SUPPORTED_BUFFER = "[<V4l2BufferType.VIDEO_CAPTURE: 1>]"
CAR_CAMERA = "HD Webcam C525"
ROBOT_CAMERA ="UVC Camera (046d:0825)"


def exchangeFD(file_1, file_2):
    old_1 = file_1
    old_2 = file_2
    rename(file_2, "/dev/OS-ul nu va genera asa ceva niciodata sperrr")
    rename(file_1, old_2)
    rename("/dev/OS-ul nu va genera asa ceva niciodata sperrr", old_1)
    return file_2, file_1

def findDevices():
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

    # Not the best way but ROS needs the lidar to always be /dev/ttyUSB0
    # We just exchange the names of lidar and whoever is names /dev/ttyUSB0
    ret_list =[robot, motors, lidar, robotCamera, motorsCamera]
    if lidar != "/dev/ttyUSB0":
        index = 0
        for i in range(0,5):
            if ret_list[i] == "/dev/ttyUSB0":
                index = i
                break
        ret_list[2], ret_list[index] = exchangeFD(ret_list[2], ret_list[index])

    return ret_list

if __name__ == "__main__":
    print(findDevices())
