#ifndef IMU_FUNCTION_H
#define IMU_FUNCTION_H
#include <Arduino.h>
#include <Wire.h>

void gyro_setup(void);
void gyro_signals(float &ROLLRATE, float &PITCHRATE, float &YAWRATE);
void gyro_calibration(float &ROLLCAL, float &PITCHCAL, float &YAWCAL);
void gyro_angle(float &g_roll, float &g_pitch, float &g_yaw,
                float &ROLLRATE, float &PITCHRATE, float &YAWRATE);

void accel_setup(void);
void accel_signals(float &AccX, float &AccY, float &AccZ);
void accel_calibration(float &AccX_Cal, float &AccY_Cal, float &AccZ_Cal);
void accel_angle(float &a_roll, float &a_pitch, float &a_yaw,
                 float &AccX, float &AccY, float &AccZ);

void mag_setup(void);
void mag_signals(float &magX, float &magY, float &magZ);
void mag_calibration(float &magA_Cal, float &magB_Cal);
void mag_angle(float &mag_yaw);

#endif