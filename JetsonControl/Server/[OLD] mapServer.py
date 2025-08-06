import sys
import os
sys.path.append(os.path.relpath("../"))


from http.server import BaseHTTPRequestHandler, HTTPServer
from functools import partial

# Our modules
from define.define import *

import json


class RequestHandler(BaseHTTPRequestHandler):
    def __init__(self, pipe, logger,  *args, **kwargs):
        self.pipe = pipe
        self.logger = logger
        super().__init__(*args, **kwargs)

    def do_GET(self):
        with open("grid.html", "r") as file:
            index = file.read()
        self.send_response(200)
        self.send_header("Content-Type", "text/html")
        self.end_headers()
        self.wfile.write(bytes(index, "utf-8"))

    def do_POST(self):
        self.pipe.send(ServerCommands.GET_MAP)
        data = self.pipe.recv()

        self.send_response(200)
        self.send_header("Content-Type", "text/json")
        self.end_headers()
        self.wfile.write(bytes(json.dumps(data), 'utf-8'))


def mapServerProcess(logger, pipe):
    try:
        os.chdir('../server/')
        handler = partial(RequestHandler, pipe,logger)
        pipe.recv() #pidList
        pipe.send(1)
        
        webServer = HTTPServer((hostName, map_port), handler)
        print("Jetson Server started at http://%s:%s" % (hostName, map_port))
        pipe.recv()
    except KeyboardInterrupt:
        exit(1)

    try:
        webServer.serve_forever()


    except KeyboardInterrupt:
        print("\n")
        webServer.server_close()
        
        print("Server closed")
