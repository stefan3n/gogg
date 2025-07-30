#include "Communication.h"
#include "Arduino.h"

static bool msgStart = false;
static bool escaped = false;

int msg[32];
int msgLen;
unsigned long prevCMDtime{0};


bool receiveByte()
{
  static int numar{0};
  if(Serial.available())
  {
    
    byte c = Serial.read();
    if((char)c == '<')
    {
     msgStart = true;
     msgLen = 0;
     numar=0;
     return false; 
    }
    
    if((char)c == '>' && msgStart)
    {
      msgStart = false;
      msg[msgLen] = numar;
      ++msgLen;
      prevCMDtime = millis();
      return true;
    }
    
    if((char)c ==',' && msgStart)
    {
      msg[msgLen] = numar;
      numar = 0;
      ++msgLen;
      return false;
    }
    
    if(msgStart)
    {
      numar=10*numar+((char)c-'0');
      msgLen = msgLen % 32;
    }
    return false;
  } 

  return false;
}

void writeMessage(int len, int* vec)
{
  Serial.print('<');
  for(int i = 0; i < len-1; ++i)
  {
    Serial.print(vec[i]);
    Serial.print(',');
  }
  Serial.print(vec[len-1]);
  Serial.print('>');
}

bool timeout()
{
  if(millis()-prevCMDtime > TIMEOUT_MILLIS)
  {
    return true;
  }
  return false;
}