#include "Motor.h"
#include "HardwareConfig.h"

Motor* MOTOR_RF = NULL;
Motor* MOTOR_RB = NULL;
Motor* MOTOR_LB = NULL;
Motor* MOTOR_LF = NULL;
Motor* ALL_MOTORS[4];
void setGlobalMotors(Motor& motorRF, Motor& motorRB, Motor& motorLB, Motor& motorLF)
{
  MOTOR_RF = &motorRF;
  MOTOR_RB = &motorRB;
  MOTOR_LB = &motorLB;
  MOTOR_LF = &motorLF;
  ALL_MOTORS[0] = MOTOR_RF;
  ALL_MOTORS[1] = MOTOR_RB;
  ALL_MOTORS[2] = MOTOR_LB;
  ALL_MOTORS[3] = MOTOR_LF;
}

void initMotor(Motor& motor, unsigned int pwmPin, unsigned int dirPin, unsigned int speedPin, bool defaultDir)
{
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(speedPin, INPUT);
  pinMode(MOTOR_BREAK, OUTPUT);
  
  motor.pwmPin = pwmPin;
  motor.dirPin = dirPin;
  motor.speedPin = speedPin;
  motor.defaultDir = defaultDir;

  motor.stateChanges = 0;
  motor.lastSpeedValue = digitalRead(speedPin);
  motor.speed = 0;

  motor.targetVelocity = motor.currVelocityCmd = 0;
  motor.timeVelocityUpdate = millis();
  
  digitalWrite(motor.dirPin, motor.defaultDir);
  motor.dir = true;
  //true == forward
  //false == backward
}

void setMotorPwm(Motor& motor, int pwm)
{
  bool dir = (pwm < 0) ^ motor.defaultDir; // this line reverses the direction if pwm is negative
  digitalWrite(motor.dirPin, dir);
  analogWrite(motor.pwmPin, abs(pwm));
  motor.dir = dir;
}

void motorBreak()
{
  for(int i = 0; i < 4; ++i)
  {
    setMotorPwm(*ALL_MOTORS[i], 0);
    ALL_MOTORS[i]->targetVelocity = 0;
    ALL_MOTORS[i]->currVelocityCmd = 0;
  }
  digitalWrite(MOTOR_BREAK, HIGH); 
}

void disableMotorBreak()
{
 digitalWrite(MOTOR_BREAK, LOW); 
}

void updateStateChanges(Motor& motor)
{
  int currVal = digitalRead(motor.speedPin);
  if(motor.dir == motor.defaultDir)
  {
    if(currVal != motor.lastSpeedValue)
    {
      ++motor.stateChanges;
      motor.lastSpeedValue = currVal;
    }
  }
  else
  {
    if(currVal != motor.lastSpeedValue)
    {
      --motor.stateChanges;
      motor.lastSpeedValue = currVal;
    }
  }
  
}

#define RAMP_UP_TIME 200
#define RAMP_UP_FRACTION 0.05
void rampUpVelocity(Motor& motor)
{
  if(motor.targetVelocity == 0)
  {
    motor.currVelocityCmd = 0; 
  }
  else if(motor.targetVelocity != motor.currVelocityCmd
    && millis() - motor.timeVelocityUpdate > RAMP_UP_TIME)
  {
    int sgn = motor.targetVelocity < motor.currVelocityCmd ? -1 : 1;
    motor.currVelocityCmd += sgn * abs(motor.targetVelocity) * RAMP_UP_FRACTION;

    motor.timeVelocityUpdate = millis();

    if(abs(motor.targetVelocity - motor.currVelocityCmd) < 5)
    {
      motor.currVelocityCmd = motor.targetVelocity;
    }
    motor.currVelocityCmd = constrain(motor.currVelocityCmd, -255, 255);
  }

  setMotorPwm(motor, motor.currVelocityCmd);
}