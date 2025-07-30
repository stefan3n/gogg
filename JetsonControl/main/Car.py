import sys
import os
sys.path.append(os.path.relpath("../"))

from math import pi as PI, sqrt, atan
import math

from help.manager import Manager
from help.help import getTime
from define.define import *


class CarState:
    MANUAL = 0
    AUTONOMOUS = 1
class AutonomousState:
    EXPLORING = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    FORWARD = 3
    GRABBING = 4

class Car:
    AUTO_PWM = 20
    REVERSE_MILLIS = 1000
    TURNING_MILLIS = 1500

    CENTER_THRESH_X = 50
    CENTER_THRESH_Y = 50

    FORWARD_DIST = 1
    GRAB_TIMEOUT = 10000

    def __init__(self, processManager : Manager, motors, robot, move_base_cmd, server_robot):
        self.state = CarState.MANUAL
        self.autoState = AutonomousState.EXPLORING

        self._processManager = processManager
        self._motors = motors
        self._robot = robot
        self._move_base_cmd = move_base_cmd
        self._server_robot = server_robot

        # this forces the arduino into STATE_IDLE
        self.emergency()
        self.endEmergency()
        self.arduinoState = MotorStates.IDLE

        self.timerEnd = 0

        self.grabTimer = 0

    def start(self):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.START])
    def emergency(self):
        self._processManager.sendMessageTo(self._motors, [0, ArduinoCommands.EMERGENCY])
    def endEmergency(self):
        self._processManager.sendMessageTo(self._motors, [0, ArduinoCommands.END_EMERGENCY])
    def brake(self):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.BREAK])
    def forward(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 0, 0])
    def backward(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 1, 1])
    def turnLeft(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 0, 1])
    def turnRight(self, pwm):
        self._processManager.sendMessageTo(self._motors, [0, MotorCommands.SET_PWM, pwm, 1, 0])


    def cancel_goal(self):
        self._processManager.sendMessageTo(self._move_base_cmd, ["cancel_goal", []])
        #return self._processManager.getMessageFrom(self._move_base_cmd)
    
    def send_goal(self, args):
        self._processManager.sendMessageTo(self._move_base_cmd, ["send_goal", args])

    def get_position(self):
        self._processManager.sendMessageTo(self._move_base_cmd, ["get_position", []])
        return self._processManager.getMessageFrom(self._move_base_cmd, 5)

    # updates state and send commands
    def update(self, objData, secondCameraObj):
        if self.state == CarState.MANUAL:
            print("MANUAL")
            self.cancel_goal() # could cause performance issues maybe?
            return
        
        # if objData == None and self.autoState != AutonomousState.EXPLORING:
        #     self.autoState = AutonomousState.EXPLORING
        #     print("EXPLORING")
        #     self.cancel_goal()
        #     return


        # if objData != None:
        #     if secondCameraObj != None:
                #TODO: !!!! make it so that you can't exit GRABBING state if robot_camera is detecting
                    # also, enter GRABBING if only robot_camera is detecting (not just on both)
        #         print("GRABBING")
        #         self.autoState = AutonomousState.GRABBING
        #         self.cancel_goal()
        #         #TODO: send grab command
        #         return

        #     x, y = objData[0][0][0]
        #     print(x)
        #     if x < -Car.CENTER_THRESH_X and self.autoState != AutonomousState.TURN_LEFT:
        #         self.autoState = AutonomousState.TURN_LEFT
        #         print("TURN_LEFT")
        #         carPos = self.get_position()
        #         if carPos == None:
        #             print("no pose data")
        #             return
        #         self.send_goal([carPos[0], carPos[1], carPos[2] + PI / 2])
        #     elif x > Car.CENTER_THRESH_X and self.autoState != AutonomousState.TURN_RIGHT:
        #         self.autoState = AutonomousState.TURN_RIGHT
        #         print("TURN_RIGHT")
        #         carPos = self.get_position()
        #         if carPos == None:
        #             print("no pose data")
        #             return

        #         self.send_goal([carPos[0], carPos[1], carPos[2] - PI / 2])
        #     elif self.autoState != AutonomousState.FORWARD:
        #         self.autoState = AutonomousState.FORWARD
        #         print("FORWARD")
        #         carPos = self.get_position()
        #         if carPos == None:
        #             print("no pose data")
        #             return

        #         forwardVector = [math.cos(carPos[2]) * Car.FORWARD_DIST, math.sin(carPos[2]) * Car.FORWARD_DIST, carPos[2]]
        #         self.send_goal(forwardVector)

        # change this incase logic for car_camera is added
        objFound = secondCameraObj != None

        if self.autoState == AutonomousState.GRABBING:
            if getTime() - self.grabTimer > Car.GRAB_TIMEOUT:
                self.autoState = AutonomousState.EXPLORING
                print("EXPLORING")
            return

        if not objFound and self.autoState != AutonomousState.EXPLORING:
            self.autoState = AutonomousState.EXPLORING
            print("EXPLORING")
            self.cancel_goal()
            return

        # add logic for centering and forward if you use car_camera

        if secondCameraObj != None and self.autoState != AutonomousState.GRABBING:
            self.autoState = AutonomousState.GRABBING
            self.cancel_goal()
            self.grabTimer = getTime()
            print("GRABBING")
            self.armCoordsFromBox(secondCameraObj)
            #TODO: send grab command 
            return



    def setAutonomous(self):
        self.state = CarState.AUTONOMOUS
        self.autoState = AutonomousState.EXPLORING

        #self.endEmergency()
        #self.start()
        self.arduinoState = MotorStates.RUNNING

    def setManual(self):
        self.state = CarState.MANUAL

        self.emergency()
        self.arduinoState = MotorStates.EMERGENCY

    def armCoordsFromBox(self, response):
        x1,y1 = response[0][1][0]
        x2,y2 = response[0][1][1]
        # these are some constants that are mesured
        l1 = 67 * 100 # lenght of the vertical link
        l2 = 107 * 100 # lenght of the horizontal link
        pixelToMicroM = 6.875 # how much is a pixel in hundred micrometers
        heightCamera = 54 # the distance from the camera to the ground
        # determine the center of the bounding box
        xCameraPixel = (x1 + x2) / 2
        yCameraPixel = (y1 + y2) / 2
        # transform pixel coordinates to real world coordinates
        xCameraInMicroM=int(xCameraPixel*pixelToMicroM)
        yCameraInMicroM=int((480 - yCameraPixel) * pixelToMicroM)
        # because they are relative to center of the camera
        # I move the coordinates relative to bittom join
        xFinal = yCameraInMicroM + sqrt(l2 * l2 - (l1 - heightCamera) * (l1 - heightCamera))
        yFinal = 0 
        # deterime how much I need to rotate the arm00
        alpha_real = abs(atan(abs(((320 - yCameraPixel) * pixelToMicroM / xFinal))) * 180 / (PI))
        if alpha_real - int(alpha_real) >= 0.4:
            alpha = int(alpha_real) + 2
        else:
            alpha = int(alpha_real)

        #print(f"x:{xFinal} alpha:{alpha}")
        xFinal = xFinal + 200
        if xFinal > 10600:
            print("Obj detected but not in range of arm")
            return

        area = (x2 - x1) * (y2 - y1)
        # determie how much I need to rotate the claw
        beta = atan((x2 - x1) / (y2 - y1)) * 180 / PI
        if beta > 15 or beta < 75:
            # send command to rotate the claw
            receiver = self._robot

            self._processManager.sendMessageTo(self._robot, [0, ArduinoCommands.END_EMERGENCY])

            message = [0, RobotCommands.ROTATE_CLAW, int(beta)]
            self._processManager.sendMessageTo(receiver, message)
            t = getTime()
            while getTime() - t < 1000:
                pass
            #get new bounding box
            response = self._processManager.getMessageFrom(self._server_robot)
            t = getTime()
            while response == None and getTime() - t <= 2000:
                response = self._processManager.getMessageFrom(self._server_robot)
            if response != None:
                x1,y1 = response[0][1][0]
                x2,y2 = response[0][1][1]
            # if the bounding box area is not modified then 
            # the claw has rotated to the needed angle or if the respone is None
            # the the claw cover the bottle
            area2 = (x2 - x1) * (y2 - y1)
            print(f"1: {area}, 2:{area2}")
            if response == None or area2 < area * 0.85 or area2 > 1.15 * area:
                beta += 90
        elif beta <= 20:
            beta = 0
        else:
            beta = 90
        # I determine the direction I need to rotate the arm
        if xCameraPixel > 320:
            dir = 1
        else:
            dir = 0
        print(f"xFinal = {xFinal}, yFinal = {yFinal}, alpha = {alpha}, beta = {beta}, xCameraMicroM = {xCameraPixel}, yCamera = {yCameraInMicroM}, dir = {dir} alpha_real = {alpha_real}")
        #print(f"x1 = {x1}, y1 = {y1}, x2 = {x2}, y2 = {y2}")
        
        #return
        
        self._processManager.sendMessageTo(self._robot, [0, ArduinoCommands.END_EMERGENCY])

        message = [0, RobotCommands.FINAL_ARM, dir, alpha, int(xFinal), 0, int(beta)]
        self._processManager.sendMessageTo(self._robot, message)
        pass
