#ifndef HardwareConfig
#define HardwareConfig

/*
    Any defines related to pinouts and other hardware related things should go here.
*/

#include "Arduino.h"

#define MOTOR_LF_DIR A0
#define MOTOR_RF_DIR A1
#define MOTOR_RB_DIR 2
#define MOTOR_LB_DIR 4

#define MOTOR_BREAK 7

#define MOTOR_RF_PWM 3
#define MOTOR_RB_PWM 5
#define MOTOR_LB_PWM 6
#define MOTOR_LF_PWM 9
const unsigned int MOTORS_PWM[] = {MOTOR_RF_PWM, MOTOR_RB_PWM, MOTOR_LB_PWM, MOTOR_LF_PWM};

#define MOTOR_RF_SPEED 13
#define MOTOR_RB_SPEED 10
#define MOTOR_LB_SPEED 11
#define MOTOR_LF_SPEED 13

#define RIGHT_DIR_DEFAULT HIGH
#define LEFT_DIR_DEFAULT LOW

#define TICKS_PER_REVOLUTION 98
#define WHEEL_RADIUS 8 //cm

#endif
