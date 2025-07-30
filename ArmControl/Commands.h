#ifndef COMMANDS_H
#define COMMANDS_H

#include "Arduino.h"

/*
  Any commands should be added here. When adding a command, always update 'COMMAND_MAX_VALUE' with the last value in the enum.
  Add commands only at the end of the list to avoid changing the order/value of the constants.
*/
struct Response
{
  int data[16];
  int len;
};
// More information about this in Commands.h from package motorControl 

enum CommandID
{
  CMD_PING = 0, CMD_STOP, CMD_MOVE_ONE, CMD_MOVE_ALL, CMD_EMERGENCY, CMD_END_EMERGENCY, CMD_ROTATE_RIGHT, CMD_ROTATE_LEFT, 
  CMD_START_POSITION_JETSON, CMD_INITIAL_POS, CMD_CALIBRATE_LEFT, CMD_CALIBRATE_RIGHT, CMD_CLAW_OPEN, CMD_CLAW_CLOSE,
  CMD_ROTATE_CLAW, CMD_FINAL_ARM
};

const int COMMAND_MAX_VALUE = CMD_FINAL_ARM;

enum CommandError
{
  SUCCES_OK,
  ERROR_INVALID_CMD, ERROR_BAD_MSG, ERROR_ARGUMENTS_LENGTH
};

/*
  return - command action error/succes code
  first argument - number of elements in the array passed
  second argument - array of arguments for the function. Can be NULL if there are no arguments.
*/
typedef int(*Command)(int, const int*);
 
/*
  Currently this global variable isn't necesary, any local variable could work.
*/
extern Command scheduledCommand;

/*
  Description: Responds to a ping command by sending back the current base position and rotation direction status.
  Input: N/A
  Errors: N/A
*/
Response pingCommand(int len, const int* args);


/*
  Description: Stops the stepper and the actuators.
  Input: N/A
  Errors: 
    - ERROR_BAD_MSG if the car is in emergency
*/
int stop(int len, const int* args);


/*
  Description: Processes a command to move a specified actuator to a target position within allowed limits, enabling the actuator if conditions are met.
  Input: 
    - args[0] is the selected Actuator
    - args[1] is the new position 
  Errors:
    - ERROR_BAD_MSG if the command is wrong
    - ERROR_ARGUMENTS_LENGTH if the command has to few arguments
*/
int moveOne(int len, const int* args);


/*  
  Description:
  Input:
    - args[0] is the new position for (1) first actuator
    - args[1] is the new position for (2) second actuator
    - args[2] is the new position for (3) third actuator
  Errors:
    ERROR_BAD_MSG if the command is wrong
    ERROR_ARGUMENTS_LENGTH if the command has to few arguments
*/
int move_all(int len, const int* args);


/*
  Description: Disables the stepper and the actuators and enters emergency state
  Input: N/A
  Errors: N/A
*/
int emergency_cmd(int len, const int* args);


/*
  Description: Enables the stepper and the actuators and exits emergency state
  Input: N/A
  Errors: N/A
*/
int end_emergency(int len, const int* args);


/*
  Description: Calculates and sets the number of steps needed to rotate the stepper motor to the left
  Input:
    - Rotation angle (in degrees)
  Errors:
    - ERROR_BAD_MSG if the nuber of steps is negative
      or the arm is in emergency or CMD_FINAL_COMMAND 
      is running
    - ERROR_ARGUMENT_LENGHT if the angle is not send
*/
int rotateLeft(int len, const int* args);


/*
  Description: Calculates and sets the number of steps needed to rotate the stepper motor to the right
  Input:
    - Rotation angle (in degrees)
  Errors:
    - ERROR_BAD_MSG if the nuber of steps is negative
      or the arm is in emergency or CMD_FINAL_COMMAND 
      is running
    - ERROR_ARGUMENT_LENGHT if the angle is not send
*/
int rotateRight(int len, const int* args);


/*
  Description: Sets the initial stepper position based on the value received from Jetson
  Input:
    - the startPosition from the jetson
    - the direction
  Errors:
    - ERROR_BAD_MSG if the arm is in emergency or CMD_FINAL_COMMAND
    - ERROR_ARGUMENT_LENGHT if the sartPos and direction are not send
*/
int startPositionJetson(int len, const int* args);


/*
  Description: Resets stepper, actuators, claw
  Input: N/A
  Errors: 
    - ERROR_BAD_MSG if it is send while emergency state or
      the CMD_FINAL_ARM is executing
*/
int backToInitialPos(int len, const int* args);


/*
  Description: Moves the stepper motor a specified number of steps to the left for calibration and resets position tracking
  Input:
    - numbers of steps to rotate left
  Errors: 
    - ERROR_BAD_MSG if the command is sent while emergency state
      or the CMD_FINAL_COMMAND is running
    - ERROR_ARGUMENT_LENGHT if the number of steps is not send
*/
int calibrate_left(int len, const int* args);


/*
  Description: Moves the stepper motor a specified number of steps to the right for calibration and resets position tracking
  Input:
    - numbers of steps to rotate right
  Errors: 
    - ERROR_BAD_MSG if the command is sent while emergency state
      or the CMD_FINAL_COMMAND is running
    - ERROR_ARGUMENT_LENGHT if the number of steps is not send
*/
int calibrate_right(int len, const int* args);


/*
  Description: Fully opens the robotic claw by setting its servo angle to 180 degrees
  Input: N/A
  Errors:
    - ERROR_BAD_MSG if it is send while emergency state or 
    the CMD_FINAL_COMMAND is running
*/
int openClaw(int len, const int* args);


/*
  Description: Fully closes the robotic claw by setting its servo angle to 45 degrees
  Input: N/A
  Errors:
   - ERROR_BAD_MSG if it is send while emergency state or 
     the CMD_FINAL_COMMAND is running
*/
int closeClaw(int len, const int* args);


/* ============== ISN'T USED ANYMORE ============== 
int rotate_claw(int len, const int* args);
/*
  inputs:
    - a value between 0 and 180 that is an angle
  errors:
    - ERROR_ARGUMENTS_LENGTH if the angle is not send
    - ERROR_BAD_MSG if the command is send while emergency state 
      is active or CMD_FINAL_ARM is running
  ===============================================*/


/*
  DESCRIPTION:
  INPUT:
    - direction
    - angle in degrees
    - xPosition
    - yPosition
    - claw angle
  ERRORS:
    - ERROR_BAD_MSG if the command is send while another CMD_FINAL_ARM
      is running or the emergency state is active
    - ERROR_ARGUMENT_LENGHT if the command hasn't all needed
      arguments: direction, angle for stepper, xPosition, yPosition,
      angle for claw
*/ 
int final_command(int len, const int* args);



/*
  New command functions should be added here. The value of the CommandID should match the position in this array.
*/



const Command COMMAND_ARRAY[] = {
  pingCommand,          // CMD_PING
  stop,                 // CMD_STOP
  moveOne,              // CMD_MOVE_ONE
  moveBoth,             // CMD_MOVE_BOTH
  emergency_cmd,        // CMD_EMERGENCY
  end_emergency,        // CMD_END_EMERGENCY
  rotateLeft,           // CMD_ROTATE_LEFT
  rotateRight,          // CMD_ROTATE_RIGHT
  startPositionJetson,  // CMD_START_POSITION_JETSON
  backToInitialPos,     // CMD_INITIAL_POS
  calibrate_left,       // CMD_CALIBRATE_LEFT
  calibrate_right,      // CMD_CALIBRATE_RIGHT
  openClaw,             // CMD_CLAW_OPEN
  closeClaw,            // CMD_CLAW_CLOSE
  rotate_claw,          // CMD_ROTATE_CLAW
  final_command         // CMD_FINAL_ARM
};

#endif