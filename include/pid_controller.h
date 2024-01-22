#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>
#include <Wire.h>

float err(float &A, float &B);
void pid(float *pid_return, float &Error, float &Kp, float &Ki, float &Kd, float &Prev_Error, float &Prev_Ki);
void pid_reset();

#endif