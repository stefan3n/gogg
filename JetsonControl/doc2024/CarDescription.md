### [< back](./GuideForDocumentation.md)
# Self Driving Toy Car Description

### The car is powered by a battery pack made from 30 Li-ION Panasonic NCR18650B cells configured in a 10s3p arrangement, delivering 36V and 10,050mAh. A voltage reducer is used to distribute power to all the components as follows, the Jetson and stepper driver are powered with 19V, the H-Bridge is powered with 12V, the servos are powered with 5V, while the motors are powered with 36V from battery.

### The Nvidia Jetson Nano serves as the central processing unit of the car, with cameras, IMU, Arduinos, and Lidar, all connected to it. 

### The system uses BLDC motors equipped with hall sensors, controlled by an XC-X11H driver, allowing precise speed control through PWM signals. Speed feedback is provided by monitoring changes in the hall sensors. In this project, the car operates relying on the rear wheels and a supporting wheel positioned centrally at the front.

### For navigation, the car uses RPLidar A1M8, IMU sensor and HC-SR04 ultrasonic distance sensor (not used anymore), running SLAM through ROS, utilizing the move_base package with exploration_lite for mapping and localization. 

### Two Arduinos are integrated in the system: one controls the motors, while another manages the robotic arm. The arm itself includes two servos (MG90S and MG996R) responsible for rotation and claw operation, alongside two linear actuators for motion, all controlled by the Arduino via an L298N H-Bridge. A DM422C driver paired with a stepper with gearbox with a 1/60 ratio ensures precision rotation of the whole arm, utilizing microstepping. 

### The vision system uses two USB cameras for real-time video capture through OpenCV. 

![car](./img/IMG_0089.JPG)



