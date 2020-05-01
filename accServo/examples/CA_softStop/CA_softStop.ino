/*
  Servo sweep in Constant acceleration mode
  Use serial monitor to control RC servo motor on pin 9
  by M Lab
  1 May 2020
*/
#define SERVO_PIN 9
#include "accServo.h"
accServo ser;
void setup() {
  ser.attach(SERVO_PIN);
  ser.setMaxVel(80);//deg/sec
  ser.setMaxAcc(100);//deg/sec^2
  Serial.begin(115200);
  ser.setPos(0);
}
bool run_flag = true;
void loop() {
  if (run_flag && ser.isFinish()) {
    if (ser.readPos() == 0) {
      ser.setPos(180);
    }

    else {
      ser.setPos(0);
    }

  }
  if (Serial.available() > 0) {
    char ch = Serial.read();
    if (ch == 'r') { //send 'r' for run motor
      run_flag = true;
    }
    else if (ch == 'b') { //send 'b' for smooth stop motor
      run_flag = false;
      ser.softStop();
    }
     else if (ch == 's') { //send 's' for immediately stop motor.
      run_flag = false;
      ser.stop();
    }
  }
  ser.update();
}
