#include "Communication.h"

#include "Motor.h"
#include "HardwareConfig.h"

#include "Commands.h"

#define INTERVAL_SPEED_READ 50

unsigned long int timeLastCmd = 0;
unsigned long timeSpeedBegin = 0;
unsigned long timeLastPrint = 0;

bool isTimeout = false;

float x, y, th;
float x2, y2, th2;
const float dW = 27.75; //cm

Motor motorRF;
Motor motorRB;
Motor motorLB;
Motor motorLF;

float encoderTicksToCm(unsigned int ticks);

const int trigPin = 8;
const int echoPin = 12;

float duration;

void setup()
{
  Serial.begin(9600);
  
  initMotor(motorRF, MOTOR_RF_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_SPEED, RIGHT_DIR_DEFAULT);
  initMotor(motorRB, MOTOR_RB_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_SPEED, RIGHT_DIR_DEFAULT);
  initMotor(motorLB, MOTOR_LB_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_SPEED, LEFT_DIR_DEFAULT);
  initMotor(motorLF, MOTOR_LF_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_SPEED, LEFT_DIR_DEFAULT);
  
  motorBreak();
  
  setGlobalMotors(motorRF, motorRB, motorLB, motorLF);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop()
{
  bool msgReceived = receiveByte();
  if(timeout())
  {
    carState = STATE_EMERGENCY;
    motorBreak();
    //Serial.println("timeout");
  }

  if(msgReceived)
  { 
    Response response;

    int ret = 0;
    if(msgLen < 2)
    {
       ret = ERROR_BAD_MSG;
    }
     
    if(msg[1] > COMMAND_MAX_VALUE)
    {
      ret = ERROR_INVALID_CMD;
    }
    
    response.data[0] = msg[0];
    response.data[1] = ret;
    response.len = 2;

    if(ret == 0)
    {
      // restart timeout
      timeLastCmd = millis();

      scheduledCommand = COMMAND_ARRAY[msg[1]];
      if(scheduledCommand != NULL)
      {
        response = scheduledCommand(msgLen-2, msg+2);
      }
    }

    /*   
    int errorCode = response.data[1];
    for(int i = 0; i < msgLen; ++i)
    {
      response.data[i] = msg[i];
    } 
    response.data[msgLen] = errorCode;
    response.len = msgLen + 1;*/

    writeMessage(response.len, response.data);
  }

  // Motor updates
  for(int i = 0; i < 4; ++i)
  {
    updateStateChanges(*ALL_MOTORS[i]);
    //rampUpVelocity(*ALL_MOTORS[i]);
  }

  if(millis() - timeSpeedBegin > INTERVAL_SPEED_READ)
  {
    float dRF = encoderTicksToCm(motorRF.stateChanges);
    float dRB = encoderTicksToCm(motorRB.stateChanges);
    float dLB = encoderTicksToCm(motorLB.stateChanges);
    float dLF = encoderTicksToCm(motorLF.stateChanges);

    float d = (dRF+dLF) / 2;
    float deltaTh = (dRF - dLF) / (2 * dW);

    float d2 = (dRB+dLB) / 2;
    float deltaTh2 = (dRB - dLB) / (2 * dW);

    th += deltaTh / 2;
    x += d * cos(th);
    y += d * sin(th);
    th += deltaTh / 2;

    th2 += deltaTh2 / 2;
    x2 += d2 * cos(th2);
    y2 += d2 * sin(th2);
    th2 += deltaTh2 / 2;

    motorRF.speed = (float)motorRF.stateChanges / TICKS_PER_REVOLUTION / INTERVAL_SPEED_READ * 60000;
    motorRB.speed = (float)motorRB.stateChanges / TICKS_PER_REVOLUTION / INTERVAL_SPEED_READ * 60000;
    motorLB.speed = (float)motorLB.stateChanges / TICKS_PER_REVOLUTION / INTERVAL_SPEED_READ * 60000;
    motorLF.speed = (float)motorLF.stateChanges / TICKS_PER_REVOLUTION / INTERVAL_SPEED_READ * 60000;

    motorRF.stateChanges = 0;
    motorRB.stateChanges = 0;
    motorLB.stateChanges = 0;
    motorLF.stateChanges = 0;

    timeSpeedBegin = millis();
  }

  // if(millis() - timeLastPrint > 1000)
  // {
  //   Serial.print("x: ");
  //   Serial.println(x);
  //   Serial.print("y: ");
  //   Serial.println(y);
  //   Serial.print("th: ");
  //   Serial.println(th);
  //   Serial.println();

  //   Serial.print("x2: ");
  //   Serial.println(x2);
  //   Serial.print("y2: ");
  //   Serial.println(y2);
  //   Serial.print("th2: ");
  //   Serial.println(th2);
  //   Serial.println();

  //   timeLastPrint = millis();
  // }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH, 100000);
  distanceReading = (duration*.0343)/2;

  if(distanceReading < 45.0f)
  {
    //carState = STATE_EMERGENCY;
    //motorBreak();
  }
  // Serial.print("Distance: ");
  //Serial.println(distanceReading);
  //delay(100);
}

float encoderTicksToCm(unsigned int ticks)
{
  return (float)ticks / TICKS_PER_REVOLUTION * TWO_PI * WHEEL_RADIUS;
}