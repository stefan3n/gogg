### [< back](./GuideForDocumentation.md)
# Before running ```python3 main.py```, run ```sudo python3 -E runBeforeMain.py```. The code is very simple, it is supposed to identify all the devices needed (lidar, arduinos, cameras) and their /dev file. The lidar must be /dev/ttyUSB0 for the ros nodes to work. Thus the code switches names between lidar and the /dev/ttyUSB0 device. There are better ways to do this but we did not have time (search udev rules).
# Run the ros nodes. You can just use ```roslaunch navigation launchWithViz.launch``` which runs: the imu node, the rplidar node, the cartographer node and the move_base node. Self exploration is done with explore_lite ```roslaunch explore_lite explore.launch```.
# Run main with: ```python3 main.py```.
# Setup
## 1. Main finds the devices (lidar, arduinos, cameras) and their /dev file (findDevices). If one device is missing the main exits (checkPorts).
## 2. All the needed processes are initialized (the Process class is our own class that contains an multiprocessing.Process and the pipes used, beware: the pipe should be passed as the last argument in the target function). There are comments describing each process. There are comments explaing each process.
## 3. Process Manager gets a list of the processes used (technically all but this was used mainly for testing as this allowed us to test individual parts) and then starts all of them. If a process is used, the process variable becomes its pid else None. This is done for an easier identification of the receiver/sender of messages. Each device sends 1 through the pipe and then waits for the main to send them 1. This is done such that all the processes finish the setup and can begin the "loop" at the same time. (waitForDevices()). Note: the process is responsible for: receiving the pid list, sending 1 after setup and receiving 1 from main.

# Loop
## 1. If the server "timed out", try to shut down everything.
## 2. Get message from server_commands, if received execute it (the gigantic if), else if 100ms have passed since the last ping, ping again (this is for safety, if the arduino does not respons everything should shut down).
## 3. Getting a message from server_car and server_robot means getting the bounding boxes of the detected objects. TODO NEXT