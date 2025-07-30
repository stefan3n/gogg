from warnings import warn
from math import sqrt, atan, pi
from cv2 import minAreaRect

import os
import sys
sys.path.append(os.path.relpath("../"))
# The above code make it possible to use: from folder.file import function/class/symbols/*
import traceback

# Our modules
## from server.jetson_server import serverProcess, handle_car_request, handle_video_stream
from multiprocessing import Process
from server.web_server import run_server
from arduino.processes import ArduinoProcess
from server.mapServer import mapServerProcess


from rosListener.cmd_vel import cmd_vel_listener, move_base_cmdProcess
from rosListener.mapListener import mapListenerProcess

from help.help import *
from define.define import *
from Car import Car
from help.process import Process
from help.manager import Manager
from logs.loggers import *
from logs.loggers import loggerMain as logger
# Main process that controls everything

#After a process is started it waits for main to send a list of pids

#Then the process initializes all the stuff it needs and sends 1 to let main know it is ready



def capPWMat(value, pwm):
    
    return abs(pwm)

    pwm = abs(pwm)
    if pwm>value:
        pwm = value
    if pwm < 20:
        pwm = 20
    
    return pwm

def main():

    mapData = None

    try:
        ports = findDevices()
        logger.info("Found devices:" + str(ports))
        print(ports)
        
        checkPorts([], logger)
        warn("/main/main.py: checkPorts([None], logger) - change None to ports.")
        # Process objects for all devices
        if True:
            robot = Process(target=ArduinoProcess, args=(ports[0],loggerRobot,))
            motors = Process(target=ArduinoProcess, args=(ports[1],loggerMotors,))

            mapServer = Process(target=mapServerProcess, args=(None,))

            rosListener = Process(target=cmd_vel_listener, args=())
            move_base_cmd = Process(target=move_base_cmdProcess, args=())
            mapListener = Process(target=mapListenerProcess, args=())

            # Starts the web server in a separate process
            web_server_proc = Process(target=run_server)
            web_server_proc.start()

            # robot             - process that controls the arm
            # motors            - process that controls the motors
            # mapServer         - TODO
            # rosListener       - TODO
            # move_base_cmd     - TODO
            # mapListener       - TODO

        # Object that handles the used processes that are passed as arguments
        # Only pass the processes that you want to start/test
        processManager = Manager([rosListener, move_base_cmd, motors, robot, mapServer, mapListener])

        logger.info("Starting devices.")
        processManager.start()
        logger.info("Devices ready.")

        # Change to pids to be able to compare processes (==)
        if True:
            motors = motors.getPid() if motors.is_alive() else None
            robot = robot.getPid() if robot.is_alive() else None
            mapServer = mapServer.getPid() if mapServer.is_alive() else None
            rosListener=rosListener.getPid() if rosListener.is_alive() else None
            move_base_cmd = move_base_cmd.getPid() if move_base_cmd.is_alive() else None
            mapListener = mapListener.getPid() if mapListener.is_alive() else None
        else:
            print("if False: ")


        processManager.exchangePids(os.getpid())
        processManager.waitForDevices()
        
        if os.path.exists("startPosition.txt"):
            with open("startPosition.txt", 'r') as file:
                startPosition = file.readline().strip()
                dir = file.readline().strip()
                receiver = robot
                message = [0, RobotCommands.SEND_START_POSITION, startPosition, dir]
                processManager.sendMessageTo(receiver, message)

       
        timerServer = getTime()
        lastSend = getTime()
        msgCounter = 0
    
    except KeyboardInterrupt as e:
        processManager.join()
        processManager.closePipes()
        logger.error("Received SIGINT.")
        exit(1)
   
        

    carObj = Car(processManager, motors, robot, move_base_cmd, None) 
    try:
        while True:
            if getTime() - timerServer > TIMEOUT_SERVER:
                logger.warning("Sent SIGINT")
                processManager.sendSIGINT()
                break

            if getTime() - lastSend > 100:
                processManager.sendMessageTo(motors, [msgCounter, ArduinoCommands.PING])
                processManager.sendMessageTo(robot, [msgCounter, ArduinoCommands.PING])
                msgCounter = msgCounter + 1
                lastSend = getTime()

            response = processManager.getMessageFrom(rosListener)
            if response!=None:
                pwmR = int(response[0])
                pwmL = int(response[1])

                pwmR = capPWMat(30, pwmR)
                pwmL = capPWMat(30, pwmL)

                processManager.sendMessageTo(motors, [0, MotorCommands.SET_PWM, pwmR, pwmL,int(response[0]<0), int(response[1]<0)])

            response = processManager.getMessageFrom(mapListener, 0.01)
            if response != None:
                mapData = response
            if processManager.getMessageFrom(mapServer) != None:
                processManager.sendMessageTo(move_base_cmd, ["get_position"])
                pose = processManager.getMessageFrom(move_base_cmd)

                processManager.sendMessageTo(mapServer, {"mapData": mapData, "pose": pose})
                mapData = None

            msgCounter = msgCounter % (65_000)

        processManager.join()
        processManager.closePipes()
        logger.info("Exiting...")
        
    except KeyboardInterrupt:
        processManager.join()
        processManager.closePipes()
        logger.error("Received SIGINT")
        exit(1)
        
    except BaseException as e:
        logger.error("Unknown error.")
        logger.error(e)
        logger.error(traceback.format_exc())
        processManager.sendSIGINT()
        logger.error("Sent SIGINT")
        exit(1)
        

if __name__ == "__main__":
    main()
