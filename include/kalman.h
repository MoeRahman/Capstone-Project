#ifndef KALMAN_H
#define KALMAN_H
#include <Arduino.h>
#include <Wire.h>

void kalman1D(float *KalmanReturn, float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement);

#endif