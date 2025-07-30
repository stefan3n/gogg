#ifndef MOTOR_H
#define MOTOR_H

enum MotorDirection
{
  FORWARD = 0, REVERSE = 1
};

/*
  defaultDir - should be LOW if the motor normally spins in the forward direction and HIGH to reverse it.
*/
struct Motor
{
  unsigned int pwmPin;
  unsigned int dirPin;
  unsigned int speedPin;
  int stateChanges;
  int lastSpeedValue;
  bool defaultDir;

  float speed;

  long timeVelocityUpdate;
  int targetVelocity;
  int currVelocityCmd;

  bool dir;
  };

/*
  These variables are used by Commands.cpp to set signals on pins. 'setGlobalMotors' should be called after initializing motors for any module that uses these variables.
*/
extern Motor* MOTOR_RF;
extern Motor* MOTOR_RB;
extern Motor* MOTOR_LB;
extern Motor* MOTOR_LF;
extern Motor* ALL_MOTORS[4];
void setGlobalMotors(Motor& motorRF, Motor& motorRB, Motor& motorLB, Motor& motorLF);



/*
  Initialized the structure and enables the required pin. Break pin is set, even if it is common for every motor.
  'defaultDir' means the signal that needs to be on pin 'dirPin' for the motor to run in the forward direction.
*/
void initMotor(Motor& motor, unsigned int pwmPin, unsigned int dirPin, unsigned int speedPin, bool defaultDir = FORWARD);

/*
  Sets the pwm on the motor's pwm pin and changes the direction if 'pwm' is negative.
  !This function will change the direction of both left/right, because their direction shares the same pin.
*/
void setMotorPwm(Motor& motor, int pwm);

/*
  Break on ALL motors.
  It will also set the pwm on all motors to 0.
*/
void motorBreak();

/*
  Disables the break on ALL motors.
*/
void disableMotorBreak();

void updateStateChanges(Motor& motor);

void rampUpVelocity(Motor& motor);

#endif
