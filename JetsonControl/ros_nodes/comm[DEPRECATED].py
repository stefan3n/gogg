# NOT USED ANYMORE
# NOT USED ANYMORE
# NOT USED ANYMORE
# NOT USED ANYMORE
# NOT USED ANYMORE
# NOT USED ANYMORE

# pip3 install pyserial
import serial
import time

# Module for serial communication
class SerialConnection:
    '''
    Class for the serial device.
    '''
    def __init__(self, name, baudrate, timeout):
        '''
        name - port such as /dev/ttyUSBx\n
        baudrate - the speed\n
        timeout - timeout for receive and send\n
        '''
        self.ser = serial.Serial(name, baudrate, timeout=timeout, write_timeout=timeout)
        # No reason for this time.sleep(7). Doesn't work without it.
        time.sleep(7)
        print("Serial connection started")
        self.ser.flushInput()
        self.ser.flushOutput()

    def send(self, numbers):
        '''
        numbers - the list to be sent to the arduino
        '''
        try:
            send_vals = [str(i) for i in numbers]
            send = ','.join(send_vals)
            send = '<' + send + '>'
            # Returns in general a number greater than 0? Possible source of future errors.
            ret = self.ser.write(send.encode('utf-8'))
            if ret >0:
                return ret
            else:
                return []
        except BaseException as e:
            # If there's any exception just raise BaseException to be caught more easily.
            # Will enter here if there's a write timeout or if the serial disconnected
            raise e

    def close(self):
        '''
        Method for gracefully closing. Might raise exceptions.
        '''
        self.ser.close()

    def receive(self):
        '''
        Method for receiving data from arduino. List of integers. Might raise exceptions.
        '''
        buffer = ''
        msgStarted = False
        try:
            while True:
                c = self.ser.read(1)
                if len(c)==0:
                    return []
                c = chr(ord(c))
                if c == '<':
                    msgStarted = True
                    buffer = ''
                    continue

                if c == '>' and msgStarted:
                    msgStarted = False
                    break
                if msgStarted:
                    buffer = buffer + c
                
                

            buffer = buffer.split(',')
            ret = [int(i) for i in buffer]
            return ret
        except BaseException as e:
            # If there's any exception just raise BaseException to be caught more easily
            # Will enter here if there's a read timeout or if the serial disconnected
            raise e
