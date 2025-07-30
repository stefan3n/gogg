#include "HardwareConfiguration.h"
#include "Commands.h"
#include "Communication.h"

// Global variables from HardwareConfiguration.h
int feedback_value1 = 0;
int feedback_value2 = 0;
int feedback_value3 = 0;

int ret1 = SUCCES_OK;
int ret2 = SUCCES_OK;
int ret3 = SUCCES_OK;

int target_position1 = 0;
int target_position2 = 0;
int target_position3 = 0;

int default_position1 = 37;
int default_position2 = 952;
int default_position3 = 957;

int stepCount = 0;
int direction = 1;
int stepTarget = 0;

int startPosition = 0;
Servo angle;
Servo angle_claw;

const int currentPosition = 0;

//this variable is a treshold for the actuator
double heigh = 1000;

//this will keep the y position of the claw
double y = 0;

bool no_calibrate = true;
bool command = true;
long time = 0;
int time_claw = 0;
bool close = false;
bool emergency = false;

int pos1 = 0;
int pos2 = 1000;

Command scheduledCommand = NULL;


void setup() 
{
  Serial.begin(9600);
 

  // Actuators
  pinMode(IN1_A1, OUTPUT);
  pinMode(IN2_A1, OUTPUT);
  pinMode(ENA_A1, OUTPUT);
  
  pinMode(IN1_A2, OUTPUT);
  pinMode(IN2_A2, OUTPUT);
  pinMode(ENA_A2, OUTPUT);

  pinMode(IN1_A3, OUTPUT);
  pinMode(IN2_A3, OUTPUT);
  pinMode(ENA_A3, OUTPUT);

  // Open claw
  angle.attach(CLAW); 
  
  /* ================ ISN'T USED ANYMORE ================
  angle_claw.attach(ROTATE_CLAW); // rotate to 0 degree
     ==================================================== */

  // Stepper
  pinMode(DIRECTION_STEPPER, OUTPUT);
  pinMode(PULSE_STEPPER, OUTPUT);
  pinMode(ENA_STEPPER, OUTPUT);

  // Default values for positions
  feedback_value1 = analogRead(FEEDBACK_A1);
  feedback_value2 = analogRead(FEEDBACK_A2);
  feedback_value3 = analogRead(FEEDBACK_A3);

  target_position1 = feedback_value1;
  target_position2 = feedback_value2;
  target_position3 = feedback_value3;

  backToInitialPos(msgLen, msg);

  // Default claw position
  angle.write(180);

  // Timeout
  time = millis()/1000;
}

void loop() 
{
  if(!emergency)
  {
    if(ret1 == SUCCES_OK)
    {
      moveActuatorGivenPosition(ACTUATOR1, target_position1);
      moveActuatorGivenPosition(ACTUATOR2, target_position2);
      moveActuatorGivenPosition(ACTUATOR3, target_position3);
    }

    if(ret2 == SUCCES_OK)
    {
      // Continue rotation if steps are not enough
      if(stepCount != stepTarget && digitalRead(ENA_STEPPER) == HIGH)
      {
        // Move one step
        digitalWrite(PULSE_STEPPER,HIGH); 
        delayMicroseconds(500); 

        digitalWrite(PULSE_STEPPER,LOW); 
        delayMicroseconds(500);
        
        stepCount++;
        if(no_calibrate)
        {
          startPosition += direction;
        }
      }
      else
      {
        // Disable stepper
        digitalWrite(ENA_STEPPER, LOW);
      }

      // Finish the CMD_FINAL_ARM command
      if(millis()/1000 - time_claw >= 4 && !command && close)
      {
        // Close the claw
        angle.write(45);
        
        // Mark that the claw is closed
        close = false;

        // Delay such that the claw has enough time to close
        delay(1000);

        // Rise the object
        moveActuatorGivenPosition(ACTUATOR1, default_position1);
        moveActuatorGivenPosition(ACTUATOR2, default_position2);
        moveActuatorGivenPosition(ACTUATOR3, default_position3);
      }
    }
  }

  int msgRecieve = receiveByte();

  // Check if msg received
  if(msgRecieve)
  {
    if(msg[1] == CMD_PING)
    {
        Response response = pingCommand(msgLen-2, msg+2);
        writeMessage(response.len, response.data);
    }
    else if(msg[1] <= CMD_MOVE_ALL) // STOP / MOVE_ONE / MOVE_ALL
    {
      scheduledCommand = COMMAND_ARRAY[msg[1]];
      if(scheduledCommand != NULL)
      {
        ret1  = scheduledCommand(msgLen - 2, msg + 2);
      }
      int response[] = {0, ret1};
      writeMessage(2, response);
    }
    else if(msg[1] < CMD_FINAL_ARM) // EMERGENCY / END_EMERGENCY / ROTATE_RIGHT / ROTATE_LEFT / START_POSITION_JETSON / INITIAL_POS / CALIBRATE_RIGHT / CLAW_OPEN / CLAW_CLOSE / ROTATE_CLAW 
    {
      Serial.print(msg[1]);
      scheduledCommand = COMMAND_ARRAY[msg[1]];
      if(scheduledCommand != NULL)
      {
        ret2  = scheduledCommand(msgLen - 2, msg + 2);
        int response[] = {0, ret2};
        writeMessage(2, response);
      }
    }
    else if(msg[1] == CMD_FINAL_ARM && command == true && !emergency)
    {
      if(millis()/1000 - time >= BlockingTime || time == 0)
      {
        time = millis()/1000;
        time_claw = millis()/1000;
        ret2 = COMMAND_ARRAY[msg[1]](msgLen - 2, msg + 2);
        int response[] = {0, ret2};
        writeMessage(2, response);
      }
    }
    else
    {
      ret2 = ERROR_BAD_MSG;
      int response[] = {0, ret2};
      writeMessage(2, response);
    }
  }

  // Timeout for CMD_FINAL
  if(millis()/1000 - time >=  BlockingTime || emergency)
  {
    command = true;
  }
}

void stopActuator(int ENA, int IN1, int IN2) {
  digitalWrite(ENA, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void moveActuator(int ENA, int IN1, int IN2, int dir) {
  digitalWrite(ENA, HIGH);
  if(dir == 0) { // extend
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {         // retract
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

int getFeedback(int FEEDBACK_PIN) {
  int suma = 0;
  const int nr_masuratori = 20;

  for (int i = 0; i < nr_masuratori; i++) {
    suma += analogRead(FEEDBACK_PIN);
    delay(2); 
  }

  return suma / nr_masuratori;
}

void moveActuatorGivenPosition(int actuator, int position) {
  int ENA;
  int IN1;
  int IN2;
  int potentimeter_pin;

  // Set the values from above
  if(actuator == ACTUATOR1)
  {
    ENA = ENA_A1;
    IN1 = IN1_A1;
    IN2 = IN2_A1;
    potentimeter_pin = FEEDBACK_A1;
  }
  else if(actuator == ACTUATOR2)
  {
    ENA = ENA_A2;
    IN1 = IN1_A2;
    IN2 = IN2_A2;
    potentimeter_pin = FEEDBACK_A2;
  }
  else if(actuator == ACTUATOR3) 
  {
    ENA = ENA_A3;
    IN1 = IN1_A3;
    IN2 = IN2_A3;
    potentimeter_pin = FEEDBACK_A3;
  }
 
  int currentPosition = getFeedback(potentimeter_pin);

  if (abs(currentPosition - targetPosition) <= ERROR_MARGIN) {
    stopActuator(ENA, IN1, IN2);
    return;
  }

  int direction = (currentPosition < targetPosition) ? 0 : 1;
  moveActuator(ENA, IN1, IN2, direction);

  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {  // timeout 3 seconds
    currentPosition = getFeedback(FEEDBACK_PIN);
    if (abs(currentPosition - targetPosition) <= ERROR_MARGIN) {
      break;
    }
    delay(10);
  }

  stopActuator(ENA, IN1, IN2);
}

/* ========================== OLD MOVEMENT ==========================
void moveActuator(int actuator, int position)
{
  int pin1;
  int pin2;
  int potentimeter_pin;
  int feedback_value;

  // Set the values from above
  if(actuator == ACTUATOR1)
  {
    feedback_value = feedback_value1 = analogRead(FEEDBACK_A1);
    pin1 = IN1_A1;
    pin2 = IN2_A1;
    potentimeter_pin = FEEDBACK_A1;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value = feedback_value2 = analogRead(FEEDBACK_A2);
    pin1 = IN1_A2;
    pin2 = IN2_A2;
    potentimeter_pin = FEEDBACK_A2;
  }
  else if(actuator == ACTUATOR3) 
  {
    feedback_value = feedback_value3 = analogRead(FEEDBACK_A3);
    pin1 = IN1_A3;
    pin2 = IN2_A3;
    potentimeter_pin = FEEDBACK_A3;
  }

  // Establish the move    
  if(feedback_value < position)
  {
    // Extend
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else
  {
    // Retract
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  feedback_value = analogRead(potentimeter_pin);

  // Set the new position for feedback value for the moved actuator
  if(actuator == ACTUATOR1)
  {
    feedback_value1 = feedback_value;
  }
  else if(actuator == ACTUATOR2)
  {
    feedback_value2 = feedback_value;
  }
  else if(actuator == ACTUATOR3)
  {
    feedback_value3 = feedback_value;
  }
}
===================================================================== */

/* =================================== ISN'T USED ANYMORE ===================================
// Check using forward kinematics if is to close to the ground
bool check_y(int position1, int position2)
{
  // Calculate the length of actuators
  double a1 = Al1 + map(position1, 28, 985, 0, 1000);
  double a2 = Al2 + map(position2, 28, 985, 0, 1000);
  // this terms are obtained by using math and will help me to calculate the y position
  double r1 = sqrt(1 - ((d1 * d1 + d2 * d2 - a1 * a1) / (2 * d1 * d2)));
  double r2 = sqrt(1 - ((d3 * d3 + d4 * d4 - a2 * a2) / (2 * d3 * d4)));
  double t1 = (d1 * d1 + d2 * d2 - a1 * a1) / (2 * d1 * d2);
  double t2 = (d3 * d3 + d4 * d4 - a2 * a2) / (2 * d3 * d4);
  // now I calculate the y by founded formula
  y = l1 * r1 + l2 * (t1 * r2 - r1 * t2);
  // now I send the message to move the arm or not
  if(y < heigh)
  {
    return false;
  }
  else
  {
    return true;
  }
}
 ============================================================================================ */