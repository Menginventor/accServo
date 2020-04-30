#include "accServo.h"
#include <disp.h>
accServo ser;
void setup() {
  Serial.begin(115200);
  ser.attach(9);
  ser.setPosition(0);
  delay(1000);
  //ser.setMaxVel(120);
  //ser.setMaxAcc(60);

  ser.setPosition(180);
  while (!ser.isFinish()) {
    ser.update();
    Serial.println(ser.readPos());
    if (ser.readPos() >= 90.0) {
      ser.softStop();
      break;
    }
    delay(10);
  }
}

void loop() {
  ser.update();
  Serial.println(ser.readPos());
  delay(10);
  /*
    ser.setPosition(180);
    while (!ser.isFinish()) {
      ser.update();
      Serial.println(ser.readPos());
      delay(20);
    }
    ser.setPosition(0);
    while (!ser.isFinish()) {
      ser.update();
      Serial.println(ser.readPos());
      delay(20);
    }
  */
}
