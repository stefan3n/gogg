#include "Commands.h"

#include "Motor.h"
#include "Arduino.h"

Command scheduledCommand = NULL;
int carState = STATE_IDLE;

static unsigned long timeSendCounts = 0;

float distanceReading = 0.0f;

Response pingCommand(int len, const int* args)
{
  Response response;

  response.data[1] = SUCCES_OK;
  response.data[2] = distanceReading;
  //Serial.println(distanceReading);
  response.len = 3;
  return response;
}

Response startCommand(int len, const int* args)
{
  Response response;

  if(carState != STATE_IDLE)
  {
    response.data[1] = ERROR_INCORRECT_STATE;
    response.len = 2;
    return response;
  }

  carState = STATE_RUNNING;

  response.data[1] = SUCCES_OK;
  response.len = 2;
  return response;
}

Response setPwmCommand(int len, const int* args)
{
  Response response;

  if(carState != STATE_RUNNING)
  {
    response.data[1] = ERROR_INCORRECT_STATE;
    response.len = 2;
    return response;
  }
   
  if(len != 4)
  {
    response.data[1] = ERROR_ARGUMENTS_LENGTH;
    response.len = 2;
    return response;  
  }


  disableMotorBreak();
  int pwmR = args[0];
  int pwmL = args[1];
  int reverseR = args[2] ? -1 : 1;
  int reverseL = args[3] ? -1 : 1;

  // MOTOR_RF->targetVelocity = pwmR * reverseR;
  // MOTOR_RB->targetVelocity = pwmR * reverseR;
  // MOTOR_LB->targetVelocity = pwmL * reverseL;
  // MOTOR_LF->targetVelocity = pwmL * reverseL;

  // MOTOR_RF->timeVelocityUpdate = millis();
  // MOTOR_RB->timeVelocityUpdate = millis();
  // MOTOR_LB->timeVelocityUpdate = millis();
  // MOTOR_LF->timeVelocityUpdate = millis();

  setMotorPwm(*MOTOR_RF, pwmR * reverseR);
  setMotorPwm(*MOTOR_RB, pwmR * reverseR);
  setMotorPwm(*MOTOR_LB, pwmL * reverseL);
  setMotorPwm(*MOTOR_LF, pwmL * reverseL);
  
  response.data[1] = SUCCES_OK;
  response.len = 2;
  return response;
}

Response breakEffect(Response response) {
  motorBreak();

  for(int i = 0; i < 4; ++i)
  {
    ALL_MOTORS[i]->currVelocityCmd = 0;
    ALL_MOTORS[i]->targetVelocity = 0;
  }
  
  response.data[1] = SUCCES_OK;
  response.len = 2;
  return response;
}

Response breakCommand(int len, const int* args)
{
  Response response;

  if(carState == STATE_IDLE)
  {
    response.data[1] = ERROR_INCORRECT_STATE;
    response.len = 2;
    return response;
  }

  response = breakEffect(response);
}

Response emergencyCommand(int len, const int* args)
{
  Response response;

  carState = STATE_EMERGENCY;
  
  response = breakEffect(response);
}

Response endEmergencyCommand(int len, const int* args)
{
  Response response;

  if(carState != STATE_EMERGENCY)
  {
    response.data[1] = ERROR_INCORRECT_STATE;
    response.len = 2;
    return response;
  }
  carState = STATE_IDLE;
  
  response.data[1] = SUCCES_OK;
  response.len = 2;
  return response;
}

Response sendCountsCommand(int len, const int* args) // unused
{
  Response response;

  unsigned long currTime = millis();

  if(carState != STATE_RUNNING)
  {
    response.data[1] = ERROR_INCORRECT_STATE;
    response.len = 2;
    return response;
  }

  response.data[1] = SUCCES_OK;
  response.data[2] = MOTOR_RF->stateChanges;
  response.data[3] = MOTOR_RB->stateChanges;
  response.data[4] = MOTOR_LB->stateChanges;
  response.data[5] = MOTOR_LF->stateChanges;

  unsigned long t = currTime - timeSendCounts;
  response.data[6] = t >> 16;
  response.data[7] = t & 0x0000FFFF;

  timeSendCounts = currTime;

  MOTOR_RF->stateChanges = 0;
  MOTOR_RB->stateChanges = 0;
  MOTOR_LB->stateChanges = 0;
  MOTOR_LF->stateChanges = 0;

  response.len = 8;
  return response;
}

Response sendwheelSpeedsCommand(int len, const int* args)
{
  Response response;

  response.data[1] = SUCCES_OK;
  response.data[2] = abs(MOTOR_RF->speed);
  response.data[3] = abs(MOTOR_RB->speed);
  response.data[4] = abs(MOTOR_LB->speed);
  response.data[5] = abs(MOTOR_LF->speed);

  response.data[6] = MOTOR_RF->speed < 0;
  response.data[7] = MOTOR_RB->speed < 0;
  response.data[8] = MOTOR_LB->speed < 0;
  response.data[9] = MOTOR_LF->speed < 0;

  response.len = 10;
  return response;
}