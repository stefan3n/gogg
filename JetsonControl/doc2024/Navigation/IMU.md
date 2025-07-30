### [< back](../GuideForDocumentation.md)
# IMU in ROS

### We used the MPU6050 sensor which includes a 3-axis accelerometer and gyroscope. This data can be integrated to provide a pose estimate, but alone it might be innacurate. It can be used alongside other sensors to get a better estimate of the pose.

## Pinout
### The Jetson has some pins that are used as SDA and SCL pins for I2C Buses.
### **$$\color{red}!$$ $$\color{red}Bus$$ $$\color{red}number$$ $$\color{red}matters$$ $$\color{red}!$$**
### *Jetson Nano*:
- ### pin 3->SDA and 5->SCL are I2C Bus 1
- ### pin 27->SDA and 28->SCL are I2C Bus 0

### *Jetson Nano Orin*:
- ### pin 3->SDA and 5->SCL are I2C Bus 7
- ### pin 27->SDA and 28->SCL are I2C Bus 1

## Setting up
### The sensor was connected to the Jetson's I2C bus (pins 3 and 5 or pins 27 and 28). To check if sensor is connected run this command:
> *sudo i2cdetect -r -y 1*
### Replace the *1* with the bus number you want to check.
### In the grid displayed you should be able to to see your device's I2C addres. For our sensor it's 0x68 (the address *UU* means it's a kernel device, not any device that you connected).

## ROS package
### [This package](https://github.com/Brazilian-Institute-of-Robotics/mpu6050_driver) allows us to also calibrate the sensor. You simply have to run the launch files as shown in the repo's README.
### **<span style="color:red">! Very important !</span>**: this package expects the device to be connected to I2C Bus 1. Because for some reason *Nano* and *Nano Orin* have different pins for Bus 1, connect the IMU on the correct pins (3&5 for Nano and 27&28 for Nano Orin).

## Calibration
### Calibration should be done with the sensor in the orientation and position it is mounted on the car. Run the calibration sequence and wait for it to finish. If it is taking longer than a couple of minutes, try rerunning. When it's done, copy the displayed offsets into the config file.

### After this, you should be able to run the normal driver and see the sensor data in the */imu/data_raw* topic. The package also provides a filtered output, if you feel like the data is too noisy (altough we haven't tested it).
