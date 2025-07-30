# Needed to be able to include our files
import sys
import os
sys.path.append(os.path.relpath("../"))

from warnings import warn
import traceback

# Our modules
from comm.comm import SerialConnection
from define.define import *
from help.help import *
from logs.loggers import loggerMotors





class Arduino:
    '''
    Class for the arduino devices.
    '''
    # pipeMain - the pipe that connects main and the arduino process
    # logger - the logger used
    # pidList - the list received from main in order to kill all the other processes in case of hardware/software failure
    # device - the serialConnection device for the arduino
    def __init__(self, pipeMain, logger):
        '''
        pipe - the pipe connection to main\n
        logger - logger used\n
        '''
        self.pipe = pipeMain
        self.pidList = []
        self.device = None
        self.logger = logger

    def setup(self, port):
        '''
        Method that opens the serial device for port=/dev/ttyUSBx
        '''
        self.pidList = self.pipe.recv()
        self.logger.info("Connecting to device.")
        try:
            # port - the device (/dev/ttyUSBx)
            self.device = SerialConnection(port, ARDUINO_BAUDRATE, TIMEOUT_ARDUINO)
            self.logger.info("Connected to device.")
        except KeyboardInterrupt:
            # if the process received sigint during serial port opening try to close it (try because it might not be ready)
            try:
                self.logger.error("Received SIGINT during setup.")
                self.device.close()
            except BaseException as e:
                self.logger.error("Error in setup during DEVICE.close() when I received SIGINT.")
                self.logger.error(e)
                self.logger.error(traceback.format_exc())
            finally:
                exit(1)
        except BaseException as e:
            # this process generated an error
            self.logger.error("Error in setup during the opening of serial connection.")
            self.logger.error(e) 
            self.logger.error(traceback.format_exc())
            # tell the other processes to stop
            sendSIGINT(self.pidList)
            self.logger.error("Sent SIGINT during setup.")
            exit(1)
        # Let main know the device is ready
        self.pipe.send(1)
        # Wait for main to say "start"
        self.pipe.recv()

    def loop(self):
        '''
        Method to be called in an infinite loop. Listens to main for commands and sends them to the arduino.
        '''
        command = None
        # wait 0.1s (100 ms) to receive a command from main
        if self.pipe.poll(0.1):
            command = self.pipe.recv()
        
        # if the command is None, the main did not send anything
        if command is None:
            return
        
        printBool = command[1] == ArduinoCommands.PING
        if not printBool:
            print("Sent to arduino: ",command)
            self.logger.info(f'Sent to arduino: {command}')

        
        

        response = self.device.send(command)
        if response == []:
            # send returns [] when there is an error during sending
            sendSIGINT(self.pidList)
            self.logger.error("Error in loop during DEVICE.send(). Timeout.")
            self.logger.error("Sent SIGINT during loop.")
            exit(1)

        response = self.device.receive()
        if response == []:
            # receive returns [] when there is an error
            # the arduino code must be written in such a way that it sends a response to the python code for each command
            # this is useful because the main will know exactly when the connection has ended
            sendSIGINT(self.pidList)
            self.logger.error("Error in loop during DEVICE.receive(). Timeout.")
            self.logger.error("Sent SIGINT during loop.")
            exit(1)


        if not printBool:    
            print("Arduino sent back: ",response)
            self.logger.info(f'Received: {response}')
        else:
            # this is the ping command, that is repurposed to send data. !! This is not good !! it's just a quick solution
            # The right way would be to have different commands for sending this data 

            # this msg is from motors (distance sensor reading)
            if len(response) == 3:
                #self.pipe.send(response[2])
                pass
            else:
                # this msg is from robot
                try:
                    if not os.path.exists("startPosition.txt"):
                        open("startPosition.txt", 'w')
                    with open("startPosition.txt", 'w') as file:
                        file.write(str(response[2]) + '\n' + str(response[3]))
                except BaseException as e:
                    # this process generated an error
                    self.logger.error("Error while saving the start position")
                    self.logger.error(e)
                    self.logger.error(traceback.format_exc())
                    # tell the other processes to stop
                    sendSIGINT(self.pidList)
                    self.logger.error("Sent SIGINT during setup.")
                    exit(1)
        
            
        
        # hmm
        warn("Razvan nu inteleg la ce ti trebuie vitezele.")
        if self.logger == loggerMotors:
            if command == MotorCommands.SEND_WHEEL_SPEEDS:
                self.pipe.send(response)

    def close(self):
        '''
        Try to gracefully shut down the arduino.
        '''
        try:
            # Try to send emergency to the device
            # Try to close it
            self.device.send([0,ArduinoCommands.EMERGENCY])
            self.device.close()
        except BaseException as e:
            self.logger.error("Error during DEVICE.close() in close().")
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            raise Exception
        else:
            self.logger.info("Disconnected normally from DEVICE.")
        
    def getPids(self):
        '''
        PidList getter.
        '''
        return self.pidList
