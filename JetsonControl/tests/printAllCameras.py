import os
import v4l2ctl

# Each USB camera generates 2 /dev/videoX files: one for the data and one for the metadata
# When using VideoCapture you are interested in the /dev/videoX that sends the data
# It seems like the supported buffer can be used to distinguish from videoData and videoMetadata


CAR_CAMERA = "HD Webcam C525"
ROBOT_CAMERA ="UVC Camera (046d:0825)"
SUPPORTED_BUFFER_ROBOT = "[<V4l2BufferType.VIDEO_CAPTURE: 1>]"
noCameras = 2
devices = os.listdir('/dev')
devices = [i for i in devices if i.startswith('video')]
print(devices)
for i in devices:
    dev = v4l2ctl.V4l2Device('/dev/'+i)
    print(dev.physical_capabilities)






