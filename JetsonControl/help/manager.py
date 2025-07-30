from os import kill
from signal import SIGINT

# It expects a list of the Process wrapper class
# Helps with handling the processes such as exchanging pids,
# receiving/sending messages waiting for the processes to get ready.
# Very useful.

class Manager:
    '''
    Handles processes.
    '''
    def __init__(self, list=[]):
        self.processList = list
    
    def start(self):
        '''
        Starts the processes.
        '''
        for i in self.processList:
            i.start()

    def join(self):
        '''
        Joins all the processes.
        '''
        for i in self.processList:
            try:
                i.join()
            except BaseException:
                pass
    
    def closePipes(self):
        '''
        Closes the pipes of all processes.
        '''
        for i in self.processList:
            i.closePipes()

    def exchangePids(self, pidMain):
        '''
        Each process gets the pids of all the other processes + the pid of the main process.
        '''
        pidList = [pidMain]
        for i in self.processList:
            pidList.append(i.getPid())
        
        for process in self.processList:
            toBeSent = []
            pid = process.getPid()
            for i in pidList:
                if i == pid:
                    continue
                toBeSent.append(i)
            pipe = process.getPipe()
            pipe.send(toBeSent)

    def sendSIGINT(self):
        '''
        Sends SIGINT to all the processes.
        '''
        for i in self.processList:
            try:
                kill(i.getPid(), SIGINT)
            except BaseException:
                pass

    def getMessageFrom(self, sender, timeout=0.1):
        '''
        Receives a message from the process with pid sender.\n
        Returns the message or None.\n
        '''
        if sender == None:
            return None
        for i in self.processList:
            if i.getPid() == sender:
                return i.getMessage(timeout)

    def sendMessageTo(self, receiver, message):
        '''
        Sends the message to the process with pid receiver.
        '''
        if receiver == None:
            return
            
        for i in self.processList:
            if i.getPid() == receiver:
                i.sendMessage(message)
                return

    def waitForDevices(self):
        '''
        Wait for the devices to setup and let them know when they can start the loop execution.
        '''
        for i in self.processList:
            print(f"Process {i} has started")
            i.getMessage(5*60) # waits at most 5 minutes for a process.
        for i in self.processList:
            i.sendMessage(1)
