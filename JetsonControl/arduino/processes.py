# Needed to be able to import our modules
import sys
import os
sys.path.append(os.path.relpath("../"))

import traceback

# Our modules
from help.help import sendSIGINT
from arduino.arduino import Arduino


# this module contains the function for the arduino process

def ArduinoProcess(port,  logger,pipeMain):
    '''
    The arduino process function.
    '''
    arduinoDevice = Arduino(pipeMain, logger)
    arduinoDevice.setup(port)
    try:
        while True:
            arduinoDevice.loop()
    except KeyboardInterrupt:
        arduinoDevice.close()
        logger.error("Received SIGINT during update().")
        logger.error("Disconnected from device.")
        exit(1)

    except BaseException as e:
        logger.error("Something went wrong.")
        logger.error(e)
        logger.error(traceback.format_exc())
        sendSIGINT(arduinoDevice.getPids())
        logger.error("Sent SIGINT")
        exit(1)
