from warnings import warn
from math import sqrt, atan, pi
from cv2 import minAreaRect

import os
import sys
sys.path.append(os.path.relpath("../"))
# The above code make it possible to use: from folder.file import function/class/symbols/*
import traceback

# Our modules
from server.jetson_server import serverProcess, handle_car_request, handle_video_stream
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
            
            server_commands = Process(target=serverProcess, args=(commands_port,loggerServer,handle_car_request,[None]) )
            server_car = Process(target=serverProcess, args=(car_port, loggerCarCamera, handle_video_stream,[ports[3], RobotCamera]))
            server_robot = Process(target=serverProcess, args =(robot_port,loggerRobotCamera, handle_video_stream,[ports[4], RobotCamera]))
            mapServer = Process(target=mapServerProcess, args=(None,))

            rosListener = Process(target=cmd_vel_listener, args=())
            move_base_cmd = Process(target=move_base_cmdProcess, args=())
            mapListener = Process(target=mapListenerProcess, args=())

            # robot             - process that controls the arm
            # motors            - process that controls the motors
            # server_commands   - gets commands from the server and sends them to main
            # server_car        - opens the camera mounted on the car+ai model+displays it's content on the web
            # server_robot      - opens the camera mounted on the robot+ai model+displays it's content on the web
            # mapServer         - TODO
            # rosListener       - TODO
            # move_base_cmd     - TODO
            # mapListener       - TODO
      

        # Object that handles the used processes that are passed as arguments
        # Only pass the processes that you want to start/test
        processManager = Manager([server_commands, rosListener, move_base_cmd, motors, robot, mapServer, mapListener, server_car, server_robot])

        logger.info("Starting devices.")
        processManager.start()
        logger.info("Devices ready.")

        # Change to pids to be able to compare processes (==)
        if True:
            motors = motors.getPid() if motors.is_alive() else None
            robot = robot.getPid() if robot.is_alive() else None

            server_commands = server_commands.getPid() if server_commands.is_alive() else None
            server_robot = server_robot.getPid() if server_robot.is_alive() else None
            server_car = server_car.getPid() if server_car.is_alive() else None
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
   
        

    carObj = Car(processManager, motors, robot, move_base_cmd, server_robot) 
    try:
        while True:
            #response = processManager.getMessageFrom(lidar)
            # handle response from lidar

            if getTime() - timerServer > TIMEOUT_SERVER:
                logger.warning("Sent SIGINT")
                processManager.sendSIGINT()
                break
            response = processManager.getMessageFrom(server_commands)
            if response != None:
                print("MAIN POLL SERVER")
                timerServer = getTime()
                message = None
                receiver = None
                
                if response[0] == ServerCommands.Motors.TURN_LEFT:
                    pwm = int(response[1])
                    message =[msgCounter, MotorCommands.SET_PWM, pwm,pwm, 0,1]
                    receiver = motors
                    
                if response[0] == ServerCommands.Motors.TURN_RIGHT:
                    pwm = int(response[1])
                    message=[msgCounter, MotorCommands.SET_PWM, pwm,pwm,1,0]
                    receiver = motors
                    
                if response[0] == ServerCommands.Motors.BACKWARD:
                    pwm = int(response[1])
                    message = [msgCounter, MotorCommands.SET_PWM, pwm,pwm, 1, 1]
                    receiver = motors
                    
                if response[0] == ServerCommands.Motors.FORWARD:
                    pwm = int(response[1])
                    message = [msgCounter, MotorCommands.SET_PWM, pwm,pwm, 0, 0]
                    receiver = motors
                    
                if response[0] == ServerCommands.Motors.BREAK:
                    message = [msgCounter, MotorCommands.BREAK]
                    receiver = motors
                    
                if response[0] == ServerCommands.EMERGENCY:
                    message = [msgCounter, ArduinoCommands.EMERGENCY]
                    processManager.sendMessageTo(motors, message)
                    message = [msgCounter, ArduinoCommands.EMERGENCY]
                    receiver = robot
                    
                if response[0] == ServerCommands.END_EMERGENCY:
                    message = [msgCounter, ArduinoCommands.END_EMERGENCY]
                    processManager.sendMessageTo(robot, message)
                    message = [msgCounter, ArduinoCommands.END_EMERGENCY]
                    receiver = motors
                    
                if response[0] == ServerCommands.Motors.START:
                    message = [msgCounter, MotorCommands.START]
                    receiver = motors
                    
                if response[0] == ServerCommands.Robot.MOVE_ONE:
                    actuator = int(response[1])
                    distance = int(response[2])
                    message = [msgCounter, RobotCommands.MOVE_ONE, actuator, distance]
                    receiver = robot
                    
                if response[0]==ServerCommands.Robot.MOVE_BOTH:
                    distance1 = int(response[1])
                    distance2 = int(response[2])
                    message = [msgCounter, RobotCommands.MOVE_BOTH, distance1, distance2]
                    receiver = robot
                    
                if response[0] == ServerCommands.Robot.INITIAL_POS:
                    message = [msgCounter, RobotCommands.INITIAL_POS]
                    receiver = robot
                    
                if response[0] == ServerCommands.Robot.ROTATE_LEFT:
                    degrees = int(response[1])
                    message= [msgCounter, RobotCommands.ROTATE_LEFT, degrees]
                    receiver = robot
                    
                if response[0] ==ServerCommands.Robot.ROTATE_RIGHT:
                    degrees = int(response[1])
                    message = [msgCounter, RobotCommands.ROTATE_RIGHT, degrees]
                    receiver = robot
                    
                if response[0] ==  ServerCommands.Robot.STOP_ROBOT:
                    message = [msgCounter, RobotCommands.STOP]
                    receiver = robot

                if response[0] == ServerCommands.Robot.CAL_LEFT:
                    steps = int(response[1])
                    message = [msgCounter, RobotCommands.CALIBRATE_LEFT, steps]
                    receiver = robot
                    
                if response[0] == ServerCommands.Robot.CAL_RIGHT:
                    steps = int(response[1])
                    message = [msgCounter, RobotCommands.CALIBRATE_RIGHT, steps]
                    receiver = robot

                if response[0] == ServerCommands.Robot.ROTATE_CLAW:
                    degrees = int(response[1])
                    message = [msgCounter, RobotCommands.ROTATE_CLAW, degrees]
                    receiver = robot

                if response[0] == ServerCommands.Robot.CLAW_CLOSE:
                    message = [msgCounter, RobotCommands.CLAW_CLOSE]
                    receiver = robot

                if response[0] == ServerCommands.Robot.CLAW_OPEN:
                    message = [msgCounter, RobotCommands.CLAW_OPEN]
                    receiver = robot
                    
                if response[0]==ServerCommands.EXIT:
                    processManager.sendSIGINT()
                    break

                if response[0] == ServerCommands.Motors.WHEEL_SPEEDS:
                    message = [msgCounter, MotorCommands.SEND_WHEEL_SPEEDS]
                    processManager.sendMessageTo(motors, message)
                    r = processManager.getMessageFrom(motors, 0.1)
                    message = r
                    receiver = server_commands

                if response[0] == ServerCommands.AUTONOMOUS:
                    carObj.setAutonomous()

                if response[0] == ServerCommands.MANUAL:
                    carObj.setManual() 


                processManager.sendMessageTo(receiver, message)
                msgCounter = msgCounter+1

            else:
                
                if getTime() - lastSend > 100:
                    processManager.sendMessageTo(motors, [msgCounter, ArduinoCommands.PING])
                    processManager.sendMessageTo(robot, [msgCounter, ArduinoCommands.PING])
                    msgCounter = msgCounter + 1
                    lastSend = getTime()


            response = processManager.getMessageFrom(server_robot)
            
            responseCar = processManager.getMessageFrom(server_car)
            if responseCar!=None:
                # TO DO
                pass

            carObj.update(responseCar, response)

            response = processManager.getMessageFrom(rosListener)
            if response!=None:
                pwmR = int(response[0])
                pwmL = int(response[1])

                pwmR = capPWMat(30, pwmR)
                pwmL = capPWMat(30, pwmL)

                #print([0, MotorCommands.SET_PWM, pwmR, pwmL,int(response[0]<0), int(response[1]<0)])
                processManager.sendMessageTo(motors, [0, MotorCommands.SET_PWM, pwmR, pwmL,int(response[0]<0), int(response[1]<0)])
                # DE TRIMIS TO DO


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
