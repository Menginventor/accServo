/*
  Servo sweep in normal mode
  by M Lab
  1 May 2020
*/
#define SERVO_PIN 9
#include "accServo.h"
accServo ser;
void setup() {
  ser.attach(SERVO_PIN);
}

void loop() {
  for (int i = 0; i < 180; i++) {
    ser.setPos(i);
    delay(20);
  }
  for (int i = 180; i > 0; i--) {
    ser.setPos(i);
    delay(20);
  }

}
