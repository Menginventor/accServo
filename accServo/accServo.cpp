#include "accServo.h"


int accServo::deg2us(float deg) {
  return int(deg * (max_us - min_us) / 180.0f + min_us);
}
void accServo::attach(int pin) {

  ser.attach(pin);
}

void accServo::attach(int pin, int _min_us, int _max_us) {

  ser.attach(pin, _min_us, _max_us);
  min_us = _min_us;
  max_us = _max_us;
}

void accServo::detach() {

  ser.detach();
}

float accServo::write_deg(float deg) {
  int us = deg2us(deg);
  ser.writeMicroseconds(us);
  crr_pos = deg;

}
void accServo::setMaxVel(float vel) {
  v_max = vel;
}
void accServo::setMaxAcc(float acc) {
  acc_max = acc;
}
void accServo::CV_setPos(float pos) {
  mode = CV_MODE;
  s_crr = 0;
  s_goal = pos - crr_pos;
  if (s_goal > 0) {
    crr_vel = v_max;
  }
  else {
    crr_vel -= v_max;
  }
  tf = s_goal / crr_vel;
  is_active = true;
  time_init = millis();
  init_pos = crr_pos;
  goal_pos = pos;
  s_crr = 0;
}
void accServo::CA_setPos(float pos) {
  if (!is_active) {
    mode = CA_MODE;
    s_crr = 0;
    s_goal = pos - crr_pos;
    if (s_goal > 0) {
      crr_acc = acc_max ;
      v_peak = v_max;
    }
    else {
      crr_acc = -acc_max ;
      v_peak = -v_max;
    }
    /*Segment calculation*/
    /*Check for triangle profile*/
    if (abs(s_goal) < abs(pow(v_max, 2) / crr_acc) ) { //S < minimum, go triangle profile
      //Serial.println("Triangular mode");
      if (s_goal > 0) {
        v_peak = sqrt(acc_max  * abs(s_goal));
      } else {
        v_peak = -sqrt(acc_max  * abs(s_goal));
      }
      t1 = v_peak / crr_acc;
      t2 = t1;
      tf = 2 * t1;
      s_acc = 0.5 * crr_acc * pow(t1, 2);
      s_cv = 0;
    }
    else {//Trapezoidal Profile

      t1 = v_peak / crr_acc;
      s_acc = 0.5 * crr_acc * pow(t1, 2);

      s_cv = s_goal - 2 * s_acc;
      float t_cv = s_cv / v_peak;
      t2 = t1 + t_cv;
      tf = t2 + t1;
    }
    /*initialize motion*/
    is_active = true;
    time_init = millis();
    init_pos = crr_pos;
    goal_pos = pos;
    s_crr = 0;
  }
}
void accServo::setPos(float pos) {
  if (pos == crr_pos) {
    /*Do nothing*/
  } else {
    if (v_max == 0) { // No vel. control
      write_deg(pos);
      mode = CP_MODE;
    }
    else { // const vel control
      if (acc_max == 0) {    //const vel control, No acc control
        CV_setPos(pos);
      }
      else { //cons acc
        CA_setPos(pos);
      }//CA_MODE
    }

  }
}

void accServo::update() {
  if (mode == CV_MODE) {
    CV_update();
  }
  else if (mode == CA_MODE) {
    CA_update();
  }
}
void accServo::CV_update() {
  if (is_active) {
    float t = float(millis() - time_init) / 1000.0;
    if (t >= tf) {//Finish
      is_active = false;
      crr_vel = 0;
      write_deg(goal_pos);

    }
    else {
      s_crr = crr_vel * t;
      write_deg(init_pos + s_crr);

    }
  }
}
void accServo::CA_update() {
  if (is_active) {
    float t = float(millis() - time_init) / 1000.0;
    if (t < t1) {//Accelerate phase
      s_crr = 0.5 * crr_acc * pow(t, 2);
      crr_vel = crr_acc * t;
      write_deg(init_pos + s_crr);
    }
    else if (t < t2) {// Constant vel. phase
      s_crr =  s_acc + v_peak * (t - t1);
      crr_vel = v_peak;
      write_deg(init_pos + s_crr);
    }
    else if (t < tf) { // deaccelerate phase
      s_crr =  s_acc + s_cv + v_peak * (t - t2) - 0.5 * crr_acc * pow(t - t2, 2);
      crr_vel = v_peak - crr_acc * (t - t2);
      write_deg(init_pos + s_crr);
    }
    else {//Finish motion
      is_active = false;
      crr_vel = 0;
      write_deg(goal_pos);

    }

  }
}
bool accServo::isFinish() {
  return !is_active;
}
float accServo::readPos() {
  return crr_pos;
}
float accServo::readVel() {
  return crr_vel;
}

void accServo::stop() {//immediately stop
  is_active = false;
  crr_vel = 0;
}

void accServo::softStop() {//Smoothly stop motor
  if (is_active && mode == CV_MODE) {
    stop();
  }
  else if (is_active && mode == CA_MODE) {
    float t = float(millis() - time_init) / 1000.0;
    if (t < t1) {// Rising

      /*Create new profile*/
      time_init = millis();//restart timer
      t1 = 0;
      t2 = 0;
      tf = (crr_vel / crr_acc);
      v_peak = crr_vel;
      init_pos = crr_pos;
      goal_pos = init_pos + v_peak * (tf - t2) - 0.5 * crr_acc * pow(tf - t2, 2);
      s_acc = 0;
      s_cv = 0;
    }
    else if (t < t2) {// Constant Speed
      init_pos = crr_pos;
      t2 = t;
      tf = t + t1;
      goal_pos = init_pos + s_acc;
      s_acc = 0;
      s_cv = 0;
      //Create softStoping point
    }
  }
}
