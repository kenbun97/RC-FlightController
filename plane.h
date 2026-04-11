
#ifndef PLANE_H

#define PLANE_H
float pwmToThrottle(unsigned long pwm);
float pwmToRudder(unsigned long pwm);
float pwmToElevator(unsigned long pwm);
void setThrottle(float percent);
void setRudder(float angle);
void setElevator(float angle);
void setupPlane();
void debugInputPWM();
#endif