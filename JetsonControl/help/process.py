from multiprocessing import Pipe
from multiprocessing import Process as pr

# Wrapper class around the Process class
# Needed to handle the processes in a easier/simpler way
# If the process is not alive, the send function will not fill up
# the pipe buffer (making the main to stop and wait for the unstarted process to consume the pipe)
# If the process is not alive, the receive function will act as if there is nothing to receive
class Process:
    '''
    Wrapper class around the Process class from multiprocessing.
    '''
    def __init__(self, target, args):
        '''
        target - the function to be ran by the process\n
        args - the arguments (as a n-tuple)
        '''
        self.pipeMain, self.pipe = Pipe(True)
        self.process = pr(target=target, args=args+(self.pipe,))

    def start(self):
        '''
        Starts the process.
        '''
        self.process.start()
    
    def join(self):
        '''
        Joins the process. Blocks until process is finished.
        '''
        self.process.join()
    
    def getPipe(self):
        '''
        Returns the pipe connection to main.
        '''
        return self.pipeMain
    
    def closePipes(self):
        self.pipeMain.close()
        self.pipe.close()

    def getPid(self):
        '''
        Returns the pid if alive else None.
        '''
        if self.process.is_alive():
            return self.process.pid
        return None
    
    def is_alive(self):
        '''
        returns True if alive, else False
        '''
        return self.process.is_alive()
    
    def getMessage(self, timeout):
        '''
        Waits timeout seconds to get a message from main. Returns the message, else None.
        '''
        if self.process.is_alive() == False:
            return None
        if self.pipeMain.poll(timeout):
            return self.pipeMain.recv()
        return None
    
    def sendMessage(self, message):
        '''
        Sends the message through the pipe to the process.
        '''
        if self.process.is_alive() == False:
            return
        self.pipeMain.send(message)