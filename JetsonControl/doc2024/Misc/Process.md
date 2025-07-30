### [< back](../GuideForDocumentation.md)
# Processes

## Process class
### This class serves as a means to abstract all the pipe creation, process initialization etc. Basically it's a wrapper for the *multiprocessing.Process*.
### The processes have some things to do before they are ready, such that *main* waits for all processes to start.
### The first thing, is that the procces must get the pidList from the pipe (it will always be sent when process is started). When ready, the process must send anything on the pipe and then wait to receive a message to start.
### Something like this:
```Python
def processfunction(pipe):
    pidList = pipe.recv()

    # initializing things and starting up

    pipe.send(1) # tell main we are ready
    pipe.recv() # wait for main to tell us all processes are ready

    # ...
    # the rest of the functionality
```

## Manager class
### This class's purpose is to further abstract inter-process communication while solving some issues that might occur. The advantage of this class is that it handles processes that are not started in such a way that doesn't break everything. It also helps with properly closing all processes.
### Once you have started all required processes, instantiate this class with the respective PID list.
### Everytime you want to communicate between processes, call the *sendMessageTo* and *getMessageFrom* functions. These functions don't do anything if given a process that hasn't started. 
### This makes it so that you can *getMessageFrom* a process and *sendMessageTo* and no data will be sent to any pipes, so as to not fill any pipes or raise exceptions.

## Issues
### **<span style="color:red;">!!!Do not send data to pipes without a process consuming the data at the other end!!!</span>** We found that after sending around 270 message to a pipe that wasn't consumed (number isn't relevant, it's probably that there is a size limit, but there is no documentation on this), the *pipe.send()* just blocks forever. This issue is solved by our *Manager* class.
### **<span style="color:skyblue;">Note: </span>** *pipe.recv()* is blocking until there is data available on the pipe. Call *pipe.poll(timeout)* to check before (or just use the *Manager* class).