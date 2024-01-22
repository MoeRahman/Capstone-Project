#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include <Servo.h>

void motor_init(int motorOnePin, int motorTwoPin, int motorThreePin, int motorFourPin);
void motor_test(float &Thrust);
void motor_mixing(float &Thrust, float &Roll, float &Pitch, float &Yaw);

#endif

