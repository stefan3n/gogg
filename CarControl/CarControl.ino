#include "HardwareConfig.h"

unsigned long lastCmdTime = 0;
const unsigned long TIMEOUT_MS = 500;

void setup() {
  Serial.begin(9600);
  // Set pinMode for all 4 motors
  pinMode(MOTOR_RF_PWM, OUTPUT);
  pinMode(MOTOR_RF_DIR, OUTPUT);
  pinMode(MOTOR_RB_PWM, OUTPUT);
  pinMode(MOTOR_RB_DIR, OUTPUT);
  pinMode(MOTOR_LF_PWM, OUTPUT);
  pinMode(MOTOR_LF_DIR, OUTPUT);
  pinMode(MOTOR_LB_PWM, OUTPUT);
  pinMode(MOTOR_LB_DIR, OUTPUT);
  stopMotors();
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('>');
    if (cmd.startsWith("<")) {
      cmd.remove(0, 1);
      if (cmd == "E") {
        stopMotors();
      } else {
        int vals[4];
        int idx = 0;
        char *token = strtok((char*)cmd.c_str(), ",");
        while (token != NULL && idx < 4) {
          vals[idx++] = atoi(token);
          token = strtok(NULL, ",");
        }
        if (idx == 4) {
          // Dreapta: RF + RB
          digitalWrite(MOTOR_RF_DIR, vals[0]);
          digitalWrite(MOTOR_RB_DIR, vals[0]);
          analogWrite(MOTOR_RF_PWM, vals[1]);
          analogWrite(MOTOR_RB_PWM, vals[1]);
          // Stanga: LF + LB
          digitalWrite(MOTOR_LF_DIR, vals[2]);
          digitalWrite(MOTOR_LB_DIR, vals[2]);
          analogWrite(MOTOR_LF_PWM, vals[3]);
          analogWrite(MOTOR_LB_PWM, vals[3]);
          lastCmdTime = millis();
        }
      }
    }
  }
  if (millis() - lastCmdTime > TIMEOUT_MS) {
    stopMotors();
  }
}

void stopMotors() {
  analogWrite(MOTOR_RF_PWM, 0);
  analogWrite(MOTOR_RB_PWM, 0);
  analogWrite(MOTOR_LF_PWM, 0);
  analogWrite(MOTOR_LB_PWM, 0);
}