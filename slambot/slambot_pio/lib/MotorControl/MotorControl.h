 #ifndef MOTORCONTROL_H
 #define MOTORCONTROL_H

 #include <Arduino.h>

 class MotorControl {
 public:
MotorControl(int motorA_pwm = 19, int motorA_in1 = 20, int motorA_in2 = 21,
    int motorB_pwm = 47, int motorB_in3 = 48, int motorB_in4 = 45,
    int pwm_channelA = 0, int pwm_channelB = 1,
    int pwm_freq = 5000, int pwm_res = 8);

void begin();
void stop();
void setMotorSpeeds(int pwmLeft,int pwmRight);


 private:


int _motorA_pwm, _motorA_in1, _motorA_in2;
int _motorB_pwm, _motorB_in3, _motorB_in4;
int _pwm_channelA, _pwm_channelB;
int _pwm_frequency, _pwm_resolution;


 };

 #endif