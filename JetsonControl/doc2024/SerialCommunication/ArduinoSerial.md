### [< back](../GuideForDocumentation.md)
# Arduino communication over serial

### This had a lot of issues for us until we got it running. The arduino serial is badly documented and misses important specifications. In this document we will detail things to be aware of when using serial.
### You should use our class/functions for the serial communication because it can be frustrating. There is not much information online about it and how it works in detail. We have spent ~2-3 weeks "perfecting" the functions used and even now there are a few this we do not understand. Also the serial communication will fail if the serial monitor from arduino IDE is open.

# How we did it
## The protocol
### We transfer messages by first sending ```<``` which denotes the beginning of a message. Then each number is sent as a string, one digit at a time, followed by ```,```. The message is ended by ```>```.
### The first value is supposed to be a message counter, used for safety (the idea was to enter emergency if too many messages are lost). This value is unused, but it could be used as a checksum, message counter, parity check (maybe even error correction?) etc.
### Second value is a command ID for the microcontroller.
### The remaining values (if there are any) are arguments for the command/additional data.
### The response has the same unused message counter, but the second value is an error/succes code.

## Python code for Jetson
### We have used the pyserial module for python. To install it run the command: ```pip3 install pyserial```. We have written out own class around the module with the methods that we believed to be important such as send, receive, close. The main code calls receive/send which are blocking methods. The arduino code calls its own receive/send.
### Receive and send are blocking, but receive has a timeout.

# Arduino code
### The files for the communication are *Communication.h* and *Communication.cpp*.
## Usage
### In loop, call receiveByte. If it returns true, that means that a message is ready in the global variable *msg* and *msgLen*. In our code, this message is directly sent to the *Commands.h* module, but it can be adapted.

# Issues
### • **<span style="color:red;">!!!Serial.write() doesn't always write only 1 byte!!!</span>** If u send a value greater than *127*, for some reason, it sends 2 bytes. A workaround could be to substract 128 from all values before sending them, but we haven't tested it. Idealy, this would be the desired way to communicate, as we send the least amount of data, so try to make it work if you can. But don't lose too much time with serial, as it's annoying to work with.
### • **<span style="color:red;">Arduino resets when serial is connected.</span>**