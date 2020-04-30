#include <Servo.h>
#include <Arduino.h>

#define CP_MODE 0// constant position
#define CV_MODE 1// constant velocity
#define CA_MODE 2// Constant accelleration

class accServo {
  private:
    /*For remember init and goal prosition before doing something*/
    float init_pos;
    float goal_pos;
    /*For profile*/
    float s_crr = 0;
    float s_goal = 0;
    float t1;
    float t2;
    float tf;
    float s_acc;//distance of first and last segment
    float s_cv;
    /*Actual servo*/
    float crr_pos = 90.0;
    float crr_vel = 0;
    int min_us = 544;
    int max_us = 2400;

    int deg2us(float deg);
    bool is_active = false;
    float crr_acc;// +- on movement direction
    float v_peak = 0;
    int mode = -1;
    unsigned long time_init = 0;

  public:
    bool debug_mode = true;
    float acc = 0;// Alway positive
    float v_max = 0;
    float write_deg(float deg);
    Servo ser;
    /*Servo wrap up*/
    void attach(int pin);
    void attach(int pin, int min_us, int max_us);

    /*Motion*/
    void update();
    void setPosition(float pos);
    void setMaxVel(float vel);
    void setMaxAcc(float _acc);
    bool isFinish();
    float readPos();
    float readVel();
    void stop();
    void softStop();

};
