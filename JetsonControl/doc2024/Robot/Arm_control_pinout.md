### [< back](../GuideForDocumentation.md)
# Arm control  pinout
## In H bridge:
  1. I connected 12V to a source of 12V which can handle an electrical current of 2-3 amperes.
  2. GND to the source ground and also to Arduino ground
  3. OUT1 is + for first actuator (red cable)
  4. OUT2 is ground for first actuator (black cable)
  5. OUT3 is + for second actuator (red cable)
  6. OUT4 is ground for second actuator (black cable)
  7. IN1 connected with 6 pin in Arduino
  8. IN2 connected with 5 pin in Arduino
  9. IN3 connected with 10 pin in Arduino
  10. IN4 connected with 9 pin in Arduino
  11. ENA is connected with 7 pin in Arduino => Active on HIGH
  12. ENB is connected with 8 pin in Arduino => Active on HIGH
## In Arduino:
  1. EXC+, from both actuator (yellow cable) and opto pin from DM422C.
  2. EXC- from both actuators(white cable) and a H Bridge ground (for a common ground).
  3. pin A0 is the feedback cable (blue one) for the first actuator
  4. pin A1 is the feedback cable (blue one) for the second actuator
  5. pin 2 is connected to direction pin from stepper's driver (DIR)
  6. pin 3 is connected to pulse pin from stepper's driver (PUL)
  7. pin 4 is connected to enable pin from stepper's driver (ENA) => Active on HIGH
  8. pin 5 is connected with IN2 from H bridge
  9. pin 6 is connected with IN1 form H bridge
  10. pin 7 is the eneable for first actuator => Active on HIGH
  11. pin 8 is the enabable for second actuator => Active on HIGH
  12. pin 9 is connected with IN4 form H bridge
  13. pin 10 is connected with IN3 form H bridge
  14. pin 11 is connected to the command wire for the second servo.
  15. pin 13 is connected with command cable for claw. 
### For the stepper driver I used a 19V source. In domcumentation for this driver is told that it should have 20V, but we don't have enough ports and the NVidia Jetson Orin Nano needs 19V and we choose this solution for power supply.
##### A recommendation is to connect the Arduino in the source, not to charge using the USB cable from where it is connected. Also connect the servos engines directly to the power source not into Arduino. For H Bridge be sure that the source is capable of high electrical current (an actuator can consume even 2.5-2.6 amperes). H Bridge is a component with overheating problems, so if the actuators are not moving then it is very possible to overheat.
##### Util informations for linear actuators pinout you will find in this [link](https://arduinogetstarted.com/images/tutorial/arduino-feedback-linear-actuator-wiring-diagram.jpg), and for servos is this [link](https://content.instructables.com/FYG/SWN3/IBXMMLB3/FYGSWN3IBXMMLB3.png?auto=webp&frame=1&width=1024&fit=bounds&md=82e53a3443fd67343967a3199d2aeca0).
##### $$\color{red}WARNING!$$ Do not use as output the pins 0, 1 because they are used for serial communication.
