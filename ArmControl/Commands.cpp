#include "Commands.h"
#include "HardwareConfiguration.h"
#include "kinematics.h"
#include <Arduino.h>

Response pingCommand(int len, const int* args)
{
  Response response;

  response.data[1] = SUCCES_OK;
  // set the first argument for message
  // I use abs becuase I cannot send negative numbers
  response.data[2] = abs(startPosition);
  // set the direction left/right

  if(direction == -1)
  {
    response.data[3] = 0;
  }
  else
  {
    response.data[3] = direction;
  }
  // set the lenght of the message
  response.len = 4;
  return response;
}

int stop(int len, const int*  args)
{
  // Not in emergency state
  if(!emergency)
  {
    // Disable stepper
    digitalWrite(ENA_STEPPER, LOW);

    command = true;
    close = true;

    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int moveOne(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    if(len < 2)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }

    // Set the target position of the actuator
    if(args[0] == ACTUATOR1)
    {
      target_position1 = args[1];
    }
    else if (args[0] == ACTUATOR2)
    {
      target_position2 = args[1];
    }
    else if(args[0] == ACTUATOR3)
    {
      target_position3 = args[1];
    }
    else
    {
      return ERROR_BAD_MSG;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int move_all(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    if(len < 3)
    {
      return ERROR_ARGUMENTS_LENGTH; 
    }
    
    // Set the target positions of the actuators
    target_position1 = args[0];
    target_position2 = args[1];
    target_position3 = args[2];

    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}


int emergency_cmd(int len, const int* args)
{
  // Disable stepper
  digitalWrite(ENA_STEPPER, LOW);

  // Mark emergency state
  emergency = true;

  return SUCCES_OK;
}

int end_emergency(int len, const int* args)
{
  // Mark emergency state exit
  emergency = false;

  // Enable stepper
  digitalWrite(ENA_STEPPER, HIGH);

  return SUCCES_OK;
}

int rotateLeft(int len, const int* args) 
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    no_calibrate = true;

    // Input verify
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = (int)((double)args[0] / 0.00375); // Angle divided by the length of one step

      digitalWrite(DIRECTION_STEPPER, LOW); // Set the direction
      digitalWrite(ENA_STEPPER, HIGH); // Enable stepper

      // Set the target
      if(direction > 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + (stepTarget - stepCount));
      }

      // Set the direction
      direction = -1;
      
      // Reset the step count
      stepCount = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int rotateRight(int len, const int* args) 
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    no_calibrate = true;
    
    // Input verify
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = (int)((double)args[0] / 0.00375); // Angle divided by the length of one step
      
      digitalWrite(DIRECTION_STEPPER, HIGH); // Set the direction
      digitalWrite(ENA_STEPPER, HIGH); // Enable stepper

      // Set the target
      if(direction < 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + ( stepTarget - stepCount));
      }
      
      // Reset the step counter
      stepCount = 0; 
      
      // Set the direction
      direction = 1;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int startPositionJetson(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    if(len < 2)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }

    // Position set based on the last sign it had (when it was pinged)
    if(args[1])
    {
      startPosition = (-1) * args[0];
    }
    else
    {
      startPosition = args[0];
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int backToInitialPos(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    no_calibrate = true;
   
    // Enable the stepper
    digitalWrite(ENA_STEPPER, HIGH);

    // Set the target position
    if(startPosition < 0)
    {
      // Set stepper direction
      digitalWrite(DIRECTION_STEPPER, HIGH);

      stepTarget = (-1) * startPosition;
      stepCount = 0;
      direction = 1;
    }
    else
    {
        // Set stepper direction
        digitalWrite(DIRECTION_STEPPER, LOW);


        stepTarget = startPosition;
        stepCount = 0;
        direction = -1;
    }

    // Set the new targets for actuators
    target_position1 = pos1;
    target_position2 = pos2;
    target_position3 = pos3;
    
    // Open the claw
    angle.write(180);
    // Rotate the claw to it's initial position
    angle_claw.write(0);

    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int calibrate_left(int len, const int* args) 
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    // Verify input
    if(len < 1) 
    {
      return ERROR_BAD_MSG;
    }
    else if(args[0] < 0)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else 
    {
      int steps = args[0];

      // Set the direction
      digitalWrite(DIRECTION_STEPPER, HIGH);
    
      // Enable stepper
      digitalWrite(ENA_STEPPER, HIGH);

      // Set the target
      if(direction > 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + (stepTarget - stepCount));
      }
      
      no_calibrate = false;
      
      // Reset the step count
      stepCount = 0;
      
      //Set the direction
      direction = -1;
      
      startPosition = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int calibrate_right(int len, const int* args) 
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    // Verify input
    if(len < 1) 
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else if(args[0] < 0)
    {
      return ERROR_BAD_MSG;
    }
    else 
    {
      int steps = args[0];

      // Enable stepper
      digitalWrite(ENA_STEPPER, HIGH);

      // Set the direction
      digitalWrite(DIRECTION_STEPPER, LOW);

      // Set the target
      if(direction < 0)
      {
        stepTarget = (steps - ( stepTarget - stepCount));
      }
      else
      {
        stepTarget = (steps + ( stepTarget - stepCount));
      }
      
      // Reset the step counter
      stepCount = 0;
      
      no_calibrate = false;
      
      // Set the direction
      direction = 1;
      
      startPosition = 0;
    }
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int openClaw(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    // Set the maximum angle for claw to open
    angle.write(180);
    
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

int closeClaw(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    // Set the minimum angle for claw to close
    angle.write(45);

    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}

/* ======================= ISN'T USED ANYMORE =======================
int rotate_claw(int len, const int* args)
{
  if(command && !emergency)
  {
    //check if the number of arguments is right
    if(len < 1)
    {
      // the number of arguments is wrong so I send an error message
      return ERROR_ARGUMENTS_LENGTH;
    }
    else
    {
      // now I check if the angle value is right
      if(args[0] < 0 || args[0] > 180)
      {
        // it is not right so I senf an error message
        return ERROR_BAD_MSG;
      }
      else
      {
        // now I rotate the servo
        angle_claw.write(args[0]);
      }
    }
    // I send that the message that the operation is a succes
    return SUCCES_OK;
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}
  ===================================================================== */

int final_command(int len, const int* args)
{
  // Not in emergency state & not CMD_FINAL_ARM
  if(command && !emergency)
  {
    command = false;
    close = true;

    // Verify input
    if(len < 5)
    {
      return ERROR_ARGUMENTS_LENGTH;
    }
    else
    {
      int dir = args[0];
      
      // Stepper movement before actuators move (actuators are faster)
      if(dir == 0)
      {
        // Set stepper direction to rotate left
        digitalWrite(DIRECTION_STEPPER, HIGH);

        // Enable stepper
        digitalWrite(ENA_STEPPER, HIGH);

        int steps = (int)((double)args[1] / 0.00375); // Angle divided by the length of one step
        int stepCount = 0;

        // Step by step movement
        while(steps != stepCount)
        {
          digitalWrite(PULSE_STEPPER, HIGH); 
          delayMicroseconds(500); 

          digitalWrite(PULSE_STEPPER, LOW); 
          delayMicroseconds(500);

          stepCount++;
          startPosition++;
        }
      }
      else
      {
        // Set stepper direction to rotate right
        digitalWrite(DIRECTION_STEPPER, LOW);

        // Enable stepper
        digitalWrite(ENA_STEPPER, HIGH);

        int steps = (int)((double)args[1] / 0.00375); // Angle divided by the length of one step
        int stepCount = 0;

        // Step by step movement
        while(steps != stepCount)
        {
          digitalWrite(PULSE_STEPPER, HIGH); 
          delayMicroseconds(500); 

          digitalWrite(PULSE_STEPPER, LOW); 
          delayMicroseconds(500);

          stepCount++;
          startPosition--;
        }
      }

      // Set actuators position based on x and y of the object
      double x = args[2];
      double y = args[3];

      /*double r2 = sqrt(x*x + y*y);

      // Adjustments to avoid hitting the object
      if(x > 10000) { x += 800; } else { x += 400; }
      if(x-400 >= 11700)      { y += 3000; }
      else if(x >= 11600)     { y += 2500; }
      else if(x >= 11300)     { y += 2000; }
      else if(x >= 11200)     { y += 1500; }
      else if(x >= 11000)     { y += 1200; }
      else if(x >= 10200)     { y += heigh; }
      else if(x >= 10100)     { y += heigh / 2; }
      
      if(r2 > L1 + L2 || r2 < fabs(L1 - L2)) {
      return ERROR_BAD_MSG;*/
    }

    double cos_angle2 = (L1*L1 + L2*L2 - r2*r2) / (2 * L1 * L2);
    double angle2 = acos(cos_angle2); 

    double cos_angle1 = (L1*L1 + r2*r2 - L2*L2) / (2 * L1 * r2);
    double angle1 = acos(cos_angle1);

    double theta1 = atan2(y, x) - angle1;
    double theta3 = phi - theta1 - angle2;

    int a1 = (int)degrees(theta1);
    int a2 = (int)degrees(angle2);
    int a3 = (int)degrees(theta3);

    target_position1 = map(a1, 0, 180, 28, 983);
    target_position1 = constrain(target_position1, 28, 983);

    target_position2 = map(a2, 0, 180, 28, 983);
    target_position2 = constrain(target_position2, 28, 983);

    target_position3 = map(a3, 0, 180, 28, 983);
    target_position3 = constrain(target_position3, 28, 983);
      
      /* ====== ISN'T USED ANYMORE ====== 
      // rotate the claw
      angle_claw.write(args[4]);
      */

      return SUCCES_OK;
    }
  }
  else
  {
    return ERROR_BAD_MSG;
  }
}