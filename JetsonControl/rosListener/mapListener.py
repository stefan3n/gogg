import rospy
from nav_msgs.msg import OccupancyGrid
from functools import partial

import math

def onMsg(pipe, data):
    map = data.data
    metadata = data.info 

    width = metadata.width
    height = metadata.height
    poseOrigin = metadata.origin # geometry_msgs/Pose

    ret = {
        'width': width,
        'height': height,
        'resolution': metadata.resolution,

        'originX': poseOrigin.position.x,
        'originY': poseOrigin.position.y,
        'originHeading': 2*math.acos(poseOrigin.orientation.z),

        'map': map
    }

    pipe.send(ret)

def mapListenerProcess(pipe):
    pidList = pipe.recv()
    try:
        rospy.init_node('mapListener')    
        onMsg_partial = partial(onMsg, pipe)
        rospy.Subscriber("map", OccupancyGrid, onMsg_partial)
    except KeyboardInterrupt:
        # logger.error("Received SIGINT during.")
        # logger.error("Closed node.")
        exit(1)
    '''
    except BaseException as e:
        logger.error("Something went wrong.")
        logger.error(e)
        sendSIGINT(pidList)
        logger.error("Sent SIGINT")
        exit(1)
    '''

    pipe.send(1)
    pipe.recv()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        # logger.error("Received SIGINT during.")
        # logger.error("Closed node.")
        exit(1)

if __name__ == '__main__':
    mapListenerProcess(None)
