### [< back](../GuideForDocumentation.md)
# Motor control

## The hardware
### The wheels are BLDC motors with hall effect sensors. These motors are 3 phase motors and are a bit more complicated than regular DC motors.
### To run them we used the *XC-X11H* driver, which also makes use of the hall effect sensors.
### Simply put, the hall effect sensors detect the presence of a magnetic field. Thus, as the motor rotates some magnets pass around the sensors and they change their state: 100, 110, 010, 011 etc.

## The driver
- ### A B C - motor phase wires
- ### Vcc Gnd - power (36V battery)
- ### STOP - disables the driver (active low)
- ### BREAK - breaks motor; it will resist motion; isn't regenerative break (active high)
- ### DIR - reverses the motor's direction (active low)
- ### GND 0-5V 5V - analog input for power control (a potentiometer should be connected here for manual control)
- ### G - logic level ground
- ### S - changes value anytime there is a change in hall sensor state (used to determine wheel speed/number of revolutions)
- ### V - 5V provided by the driver
- ### P - PWM input for power control (the pads near the sensor wires need to be short circuited to enable pwm)
- ### G - labled as external ground (don't know it's purpose; it's shorted to the other grounds)
- ### The on-board potentiometer is used for manual control (works the same as the analog input pin)

## Wiring the motor correctly
### The phase wires whould correspond to the right hall sensor wires, otherwise the motor won't work or it will work poorly (it will vibrate and make noise).
### If you don't know which wires are which, a way to do this is by trail and error.
### 1. Wire the hall sensors in any order.
### 2. Connect the phases in any order.
### 3. Run the motor (with the on-board potentiometer for example). If it doesn't spin, vibrates or makes noise, then swap two of the phase wires. Repeat 2. and 3. until it runs smoothly and silently.
### More info [here](https://electronics.stackexchange.com/questions/624932/how-do-you-know-which-pin-corresponds-hall-sensor-1-2-and-3-so-that-i-can-pair#:~:text=Rotating%20the%20motor%20by%20hand%20should%20give%20the,low%20voltages%20helps%20identify%20the%20corresponding%20others%20%28power%29.).

## Controlling from a microcontroller
### With a microcontroller we can provide a PWM signal. To enable PWM control via the P pin, the pads between the sensor port and capacitor needs to be short circuited (apply a blob of solder or add a jumper connector if you wish to sometimes disable this functionality manually).
### **<span style="color:red;">!Very important!</span>**: when controlling with an external signal, make sure that the on-board potentiometer is turned completely counter-clockwise. Otherwise the motor will turn even if the PWM command is at 0% duty cycle!
### **<span style="color:skyblue;">Note</span>**: it might be possible to command the driver with PWM through the **0-5V** analog pin. We haven't extensively tested this, but a small trial we ran showed that it worked. But again, it's not properly tested at multiple PWM values to check if it is reliable. It seems to work because the analog input is filtered. Altough, if this works, it could save the hassle of soldering stuff directly to the board and instead only use the screw terminals.

## Pinout
### 2 – DIR right motors =>Active LOW (reverse e LOW)
### 4 – DIR left motors =>Active LOW (reverse e LOW)
### 5 – PWM speed for right back motor
### 6 – PWM speed for left back motor
### 7 – BREAK (all motors) => Active HIGH
### 8 – TRIG distance sensor
### 10 – SPEED right back
### 11 – SPEED left back
### 12 - ECHO distance sensor


## Reading sensor data
### The driver changes the value on the S pin every time there is a state change of the hall sensors (basically the motor is spinning).
### You need to count the number of changes for a full rotation so that you can use the data to compute wheel speed/revolutions. Do this by manually counting the changes: spin the wheel for a full revolution and analize the signal with an oscilloscope.
### You could also count programatically with a microcontroller, but there is a chance that it misses some changes.
### To read the pin with a microcontroller there are 2 approaches:

### 1. Inside the main loop read the state of that pin and increment a counter if the value changes.
### 2. With the S signal connected to an interrupt pin, increment a counter every time there is a change interrupt on that pin.
### We setteled with the first option, because the Arduino only had 2 interrupt pins, but also the interrupt method didn't work for us. Altough, in theory the interrupt method is more precise as it won't jump over state changes if they happen faster then the loop iterations.

## Possible problems 
### **<span style="color:red">!Very important!</span>**: Even if the *STOP* pin is active low, **DO NOT** provide 5V to that pin, just leave it unconnected. If you provide a voltage to that pin, something weird happens to the internal logic levels and the analog pin is at a non-zero voltage, even when unconnected, making the wheel spin all the time. Just leave the *STOP* pin hanging.

### When setting low PWM to the wheels such that they don't have enough power to spin, the controller will stop working. We haven't found an exact solution to restart it, but it seems to start working when:
- ### resetting the arduino (by button or reconnecting serial)
- ### sending a setPwm command to motors with 0 on all arguments.
### To prevent this just make sure it doesn't get low pwms on both wheels. The best way would be to implement PID on either wheel speed or the whole car speed.

## Other sources
- ### https://mad-ee.com/easy-inexpensive-hoverboard-motor-controller/
- ### https://forum.arduino.cc/t/zs-x11h-control-exact-rpm-with-pwm-using-an-arduino/1062746/5
- ### https://www.youtube.com/watch?v=2KuY2Y5zcM8