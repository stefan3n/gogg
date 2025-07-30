#ifndef HardwareConfig
#define HardwareConfig

/*
    Any defines related to pinouts and other hardware related things should go here.
*/

#include "Arduino.h"

#define MOTOR_LEFT_DIR 13
#define MOTOR_RIGHT_DIR 12

#define MOTOR_LEFT_BREAK 10
#define MOTOR_RIGHT_BREAK 11

#define MOTOR_RF_PWM 6
#define MOTOR_RB_PWM 9
#define MOTOR_LB_PWM 3
#define MOTOR_LF_PWM 5
const unsigned int MOTORS_PWM[] = {MOTOR_RF_PWM, MOTOR_RB_PWM, MOTOR_LB_PWM, MOTOR_LF_PWM};

#define MOTOR_LEFT_SPEED ???
#define MOTOR_RIGHT_SPEED ???

#define RIGHT_DIR_DEFAULT HIGH
#define LEFT_DIR_DEFAULT LOW

#define TICKS_PER_REVOLUTION 98
#define WHEEL_RADIUS 8 //cm

#endif
