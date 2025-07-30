import sys
import os
sys.path.append(os.path.relpath("../"))

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from logs.loggers import loggerRos as logger
import actionlib
from actionlib_msgs.msg import GoalID
from help.help import sendSIGINT, getTime
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from math import sqrt, pi

from time import sleep

globalPipe=None
def fromRotToPWM(rot):
# 650 ROT PT 255 PWM
    
    return 255*rot/(650/60)


lastTimeSend = 0
def cmd_vel_callback(data):
    global lastTimeSend
    if getTime() - lastTimeSend < 100:
        return

    # '''
    # lastTimeSend = getTime()
    # msg=[data.linear.x,data.linear.y,data.angular.z]

    # if msg[2] > 0:
    #     pwmR = 25
    #     pwmL = -25
    # elif msg[2] < 0:
    #     pwmR = -25
    #     pwmL = 25
    # elif msg[0] < 0:
    #     pwmR = -15
    #     pwmL = -15
    # elif msg[0] > 0:
    #     pwmR = 15
    #     pwmL = 15

    # toBeSent = [pwmR, pwmL]
    # global globalPipe
    # globalPipe.send(toBeSent)
    # '''

    lastTimeSend = getTime()
    msg=[data.linear.x,data.linear.y,data.angular.z]
    v=sqrt(msg[0]**2+msg[1]**2)
    L=0.55
    vr=v+msg[2]*L/2
    
    vl=v-msg[2]*L/2
    r=0.08
    thetaR=vr/r
    thetaL=vl/r
    rotR=thetaR/(2*pi) # rot / s
    rotL=thetaL/(2*pi) # rot / s
    
    pwmR= fromRotToPWM(rotR)
    pwmL = fromRotToPWM(rotL)

    toBeSent = [pwmR, pwmL]
    global globalPipe
    if msg[0] < 0:
        toBeSent = [-25, -25]
    globalPipe.send(toBeSent)

    

def cmd_vel_listener(pipe):
    '''
    Function used by the lidar process that listens to /cmd_vel and sends through the pipe the PWM [rightWheel, leftWheel].
    '''
    pidList = pipe.recv()
    global globalPipe
    globalPipe = pipe

    pipe.send(1)
    pipe.recv()
    try:
        rospy.init_node('moveBaseListener', anonymous = True)
        rospy.Subscriber('cmd_vel',Twist, cmd_vel_callback)
        rospy.spin()
    except KeyboardInterrupt:
        logger.error("Received SIGINT.")
        exit(1)
    '''
    except BaseException as e:
        logger.error("Something went wrong.")
        logger.error(e)
        sendSIGINT(pidList)
        logger.error("Sent SIGINT")
        exit(1)
    '''




def get_position():
    '''
    Returns [xWorld, yWorld, zAngle] from /tf    
    '''
    listener = tf.TransformListener()
    parent_frame ="odom"
    child_frame ="base_link"

    try:
        listener.waitForTransform(parent_frame, child_frame, rospy.Time(), rospy.Duration(2))
        trans, rot = listener.lookupTransform(parent_frame, child_frame, rospy.Time())
        return [trans[0], trans[1], rot[2]]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
        
def send_goal(xWorld,yWorld,zAngle):
    '''
    Sends a goal to move_base to plan a trajectory.
    '''
    try:
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id ="map"
        goal.target_pose.header.stamp=rospy.Time.now()
        goal.target_pose.pose.position.x = xWorld
        goal.target_pose.pose.position.y = yWorld
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation.z = zAngle
        goal.target_pose.pose.orientation.w= 1
        client.send_goal(goal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


def cancel_goal():
    '''
    Cancels move_base goal.
    '''
    try:
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        client.send_goal(goal)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


def move_base_cmdProcess(pipe):
    pidList = pipe.recv()
    try:
        rospy.init_node('move_base_cmd')    
    except KeyboardInterrupt:
        logger.error("Received SIGINT during.")
        logger.error("Closed node.")
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
        while not rospy.is_shutdown():
            if not pipe.poll(0.1):
                continue

            cmd = pipe.recv()
            if cmd[0] == "send_goal":
                send_goal(cmd[1][0], cmd[1][1], cmd[1][2])
            elif cmd[0] == "cancel_goal":
                cancel_goal()
            elif cmd[0] == "get_position":
                pipe.send(get_position())
    except KeyboardInterrupt:
        logger.error("Received SIGINT during.")
        logger.error("Closed node.")
        exit(1)
    '''
    except BaseException as e:
        logger.error("Something went wrong.")
        logger.error(e)
        sendSIGINT(pidList)
        logger.error("Sent SIGINT")
        exit(1)
    '''
