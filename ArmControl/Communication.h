#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "Arduino.h"

#define MAX_MSG_LENGTH 32
#define TIMEOUT_MILLIS 3000

extern int msg[MAX_MSG_LENGTH];
extern int msgLen;

/*
  Tries to read a byte from Serial(non-blocking).
  Returns true if a message is ready, false otherwise.
  Expects messages to start with '<' and end with '>'.
*/
bool receiveByte();
void writeMessage(int, int*);

#endif