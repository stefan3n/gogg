import sys
import os
sys.path.append(os.path.relpath("../"))


from http.server import BaseHTTPRequestHandler, HTTPServer
from functools import partial
import cv2
import traceback

# Our modules
from camera.camera import Camera
from define.define import *

from time import sleep

from multiprocessing import Pipe



def handle_car_request(self):
    '''
    Used for the buttons on the server.
    '''
    command = self.path.lstrip('/').split('/')

    if command[0] == 'commands.js':
        with open("commands.js", "r") as file:
            commandsFile = file.read()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(bytes(commandsFile, "utf-8"))
        return

    # not index
    if command[0] != '':
        self.send_error(404)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(bytes("404 Just '/' is a valid GET path", "utf-8"))
        return

    with open("index.html", "r") as file:
        index = file.read()
    self.send_response(200)
    self.send_header("Content-Type", "text/html")
    self.end_headers()
    self.wfile.write(bytes(index, "utf-8"))

def handle_video_stream(self):
    '''
    Used for the video feed on the server.
    '''
    if self.path == "/video_feed":
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        camera = Camera(False, self.pipe, self.logger, self.extra_args[1])
        camera.setup(cv2.CAP_V4L2, self.extra_args[0], 1)
        
        try:
            # We have merged the camera process and this server because we could not send images through the pipe for whatever reason.
            while True:
                command = None
                if self.pipe.poll(0.1):
                    command = self.pipe.recv()
        
                if command == "exit":
                    try:
                        self.logger.info("Closing camera.")
                        camera.close()
                        self.logger.info("Camera closed.")
                    except BaseException as e:
                        self.logger.info("Error while closing.")
                        self.logger.error(e)
                        self.logger.error(traceback.format_exc())
                    finally:
                        exit(0)

                frame = camera.loop()

                h, w = frame.shape[:2]
                aspect_ratio = w / h
                new_width = int(2 / 3 * h)
                resized_frame = cv2.resize(frame, (new_width, int(new_width / aspect_ratio)))
                
                ret, buffer = cv2.imencode('.jpg', resized_frame)
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Content-Length', len(buffer))
                self.end_headers()
                self.wfile.write(buffer.tobytes())
                self.wfile.write(b'\r\n')
        except BrokenPipeError:
            pass  # Client closed connection
        except KeyboardInterrupt:
            self.logger.error("Received SIGINT.")
            try:
                camera.close()
            except BaseException as e:
                self.logger.error("Error while closing camera.")
                self.logger.error(e)
                self.logger.error(traceback.format_exc())
            exit(1)
    
        except BaseException as e:
            self.logger.error("Unknown error.")
            self.logger.error(e)
            self.logger.error(traceback.format_exc())
            try:
                camera.close()
            except BaseException:
                self.logger.error(traceback.format_exc())
            finally:
                #sendSIGINT(camera.getPidList())
                self.logger.error("Sent SIGINT.")
                exit(1)

def handle_connection_drop(self):
    if self.path == "/video_feed":
        self.send_response(200)
        self.send_header('Content-Type', 'text/text')
        #self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        print("yaaas")
            # We have merged the camera process and this server because we could not send images through the pipe for whatever reason.
        try:
            while True:
                buffer = "\"Test\""
                self.send_header('Content-Type', 'text/json')
                self.send_header('Content-Length', len(buffer))
                self.end_headers()
                self.wfile.write(bytes(buffer, 'utf-8'))
                self.wfile.write(b'\r\n')
                sleep(0.1)
        except BaseException as e:
            print("heyy")
            print(e)
        


class RequestHandler(BaseHTTPRequestHandler):
    def __init__(self, pipe,logger,do_get_function, extra_args,  *args, **kwargs):
        self.pipe = pipe
        self.logger = logger
        self.handle_method = do_get_function
        self.extra_args = extra_args
        super().__init__(*args, **kwargs)

    def do_GET(self):
        self.handle_method(self)

    def do_POST(self):
        
        command = self.path.lstrip('/').split('/')
        self.pipe.send(command)

        if command[0] == ServerCommands.Motors.WHEEL_SPEEDS:
            speeds = self.pipe.recv()
            self.send_response(200)
            self.send_header("Content-Type", "text/json")
            self.end_headers()
            self.wfile.write(bytes('{' + f"speeds: {speeds[1:]}" + '}', 'utf-8'))
            return

        self.send_response(200)
        self.send_header("Content-Type", "text/json")
        self.end_headers()
        self.wfile.write(bytes('{status:"OK"}', 'utf-8'))


def serverProcess(port,logger, do_get_function, extra_args, pipe):
    '''
    Opens the server.
    port - 8080, 8081 etc\n
    logger - logger used\n
    do_get_function - handle video stream or car request\n
    extra_args - arguments for the do_get\n
    pipe - pipe connection with main\n
    '''
    try:
        os.chdir('../server/')
        pipe.recv()     #pidList
        handler = partial(RequestHandler, pipe,logger, do_get_function, extra_args)
        pipe.send(1)
        
        webServer = HTTPServer((hostName, port), handler)
        pipe.recv()
        print("Jetson Server started at http://%s:%s, device:%s." % (hostName, port,extra_args[0] ))

    except KeyboardInterrupt:
        exit(1)
    

    try:
        webServer.serve_forever()


    except KeyboardInterrupt:
        print("\n")
        webServer.server_close()
        
        print("Server closed")

if __name__ == "__main__":
    serverProcess(8084, None, handle_connection_drop, [None], None)
