#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"

struct Response
{
  int data[16];
  int len;
};

/*
  Any commands should be added here. When adding a command, always update 'COMMAND_MAX_VALUE' with the last value in the enum.
  Add commands only at the end of the list to avoid changing the order/value of the constants.
*/
enum CommandID
{
  CMD_PING = 0, CMD_START, CMD_SET_PWM, CMD_BREAK, CMD_EMERGENCY, CMD_END_EMERGENCY, CMD_SEND_WHEEL_SPEEDS
};
const int COMMAND_MAX_VALUE = CMD_SEND_WHEEL_SPEEDS;

enum CommandError
{
  SUCCES_OK,
  ERROR_INVALID_CMD, ERROR_BAD_MSG, ERROR_INCORRECT_STATE,
  ERROR_ARGUMENTS_LENGTH
};

/*
  STATE_IDLE - starting state. Car doesn't respond to any move commands.
  STATE_RUNNING - car responds to commands
  STATE_EMERGENCY - imediately stops all motors and stops responding to move commands.
*/
enum CarState
{
  STATE_IDLE, STATE_RUNNING, STATE_EMERGENCY
};
extern int carState;

extern float distanceReading;

/*
  return - command action error/succes code
  first argument - number of elements in the array passed
  second argument - array of arguments for the function. Can be NULL if there are no arguments.
*/
typedef Response(*Command)(int, const int*);
 
/*
  Currently this global variable isn't necesary, any local variable could work.
*/
extern Command scheduledCommand;


/*
  Expects no arguments.
  Errors: none
  Returns distance sensor reading. (this should be a different command, but it works for now)
*/
Response pingCommand(int len, const int* args);

/*
  Expects no arguments.
  Errors:
    * carState isn't STATE_IDLE
  Sets the state to STATE_RUNNING
*/
Response startCommand(int len, const int* args);

/*
  Arguments:
    * 1st - pwm value
    * 2nd - reverse right side motors
    * 3rd - reverse left side motors
  Errors:
    * carState isn't STATE_RUNNING
    * 3 arguments not passed
  Sets the target pwm of all motors according to the arguments.(will be controller by rampUpVelocity)
*/
Response setPwmCommand(int len, const int* args);

/*
  Excepts: response
  Errors: None.
  Brakes all motors
*/
Response brakeEffect(Response response);

/*
  Expects no arguments.
  Errors:
    * carState is STATE_IDLE
  Calls brakeEffect.
*/
Response breakCommand(int len, const int* args);

/*
  Expects no arguments.
  Errors: None.
  Enters STATE_EMERGENCY and calls brakeEffect.
*/
Response emergencyCommand(int len, const int* args);

/*
  Expects no arguments.
  Errors:
    * carState isn't STATE_EMERGENCY
  Enters STATE_IDLE.
*/
Response endEmergencyCommand(int len, const int* args);

/*
@Unused
*/
Response sendCountsCommand(int len, const int* args);

/*
  Expects no arguments.
  Errors: none.
  Sends motor speeds and corresponding signs.
  (speedRF, speedRB, speedLB, speed LF, signRF, signRB, signLB, signLF)
*/
Response sendwheelSpeedsCommand(int len, const int* args);

/*
  New command functions should be added here. The value of the CommandID should match the position in this array.
*/
const Command COMMAND_ARRAY[] = {
  pingCommand,               // CMD_PING
  startCommand,       // CMD_START
  setPwmCommand,      // CMD_SET_PWM
  breakCommand,       // CMD_BREAK
  emergencyCommand,   // CMD_EMERGENCY
  endEmergencyCommand, // CMD_END_EMERGENCY
  sendwheelSpeedsCommand // CMD_SEND_WHEEL_SPEEDS
};

#endif
