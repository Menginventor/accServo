/*
  Servo sweep in Constant velocity mode
  by M Lab
  1 May 2020
*/
#define SERVO_PIN 9
#include "accServo.h"
accServo ser;
void setup() {
  ser.attach(SERVO_PIN);
  ser.setMaxVel(100);// deg/sec
  Serial.begin(115200);
}

void loop() {

  ser.setPos(0);
  while (! ser.isFinish()) {
    ser.update();
    Serial.println(ser.readPos());
    delay(10);
  }
  ser.setPos(180);
  while (! ser.isFinish()) {
    ser.update();
    Serial.println(ser.readPos());
    delay(10);
  }


}
