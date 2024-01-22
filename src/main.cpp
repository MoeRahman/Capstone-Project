#include <Arduino.h>
#include "imu_functions.h"
#include "receiver.h"
#include "motor.h"
#include "pid_controller.h"
#include "kalman.h"

//Global Variables
float Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate;
float Sensor_AccX, Sensor_AccY, Sensor_AccZ;
float RollRate_Cal, PitchRate_Cal, YawRate_Cal;
float AccX_Cal, AccY_Cal, AccZ_Cal;
float Roll_Cal, Pitch_Cal;

float G_Roll = 0, G_Pitch = 0, G_Yaw = 0,
      A_Roll = 0, A_Pitch = 0, A_Yaw = 0;

float pid_out[] = {0, 0, 0};

//PID RATE VARIABLES
float rollRate_err = 0.0;
float pitchRate_err = 0.0;
float yawRate_err = 0.0;  

float prev_rollRate_err = 0.0;
float prev_pitchRate_err = 0.0;
float prev_yawRate_err = 0.0;

float prev_rollRate_Ki = 0.0;
float prev_pitchRate_Ki = 0.0;
float prev_yawRate_Ki = 0.0;

//PID ANGLE VARIABLES
float roll_err = 0.0;
float pitch_err = 0.0;
float yaw_err = 0.0;  

float prev_roll_err = 0.0;
float prev_pitch_err = 0.0;
float prev_yaw_err = 0.0;

float prev_roll_Ki = 0.0;
float prev_pitch_Ki = 0.0;
float prev_yaw_Ki = 0.0;

// ADD KNOBS TO PID CONSTANTS
// RATE CONSTANTS
float rollRate_Kp = 0.2, rollRate_Ki = 2, rollRate_Kd = 0.02;
float pitchRate_Kp = 0.2, pitchRate_Ki = 2, pitchRate_Kd = 0.02;
float yawRate_Kp = 2, yawRate_Ki = 12, yawRate_Kd = 0;

// ATTITUDE CONSTANTS
float roll_Kp = 1, roll_Ki = 0, roll_Kd = 0;
float pitch_Kp = 1, pitch_Ki = 0, pitch_Kd = 0;
//float yaw_Kp = 2, yaw_Ki = 12, yaw_Kd = 0;

// PID OUTPUTS
float pid_yawRate = 0, pid_pitchRate = 0, pid_rollRate = 0;
float pid_yaw = 0, pid_pitch = 0, pid_roll = 0;

//KALMAN VARIABLES
float KalmanRoll = 0, KalmanUncertaintyRoll = 5;
float KalmanPitch = 0, KalmanUncertaintyPitch = 5;
float Kalman1D[] = {0,0,0};
float rollKG = 0;
float pitchKG = 0;

float ReceiverValues[] = {0, 0, 0, 0, 0, 0, 0, 0};


void setup() {

  Serial.begin(115200);

  pinMode(2, OUTPUT);//RED
  pinMode(3, OUTPUT);//GREEN
  pinMode(4, OUTPUT);//BLUE
  pinMode(5, OUTPUT);//BUZZER

  digitalWrite(3, LOW );
  digitalWrite(2, HIGH);

  gyro_setup();
  accel_setup();
  gyro_calibration(RollRate_Cal, PitchRate_Cal, YawRate_Cal); 

  Serial.println("GYROCAL: "+ String(RollRate_Cal) +"|"+ String(Pitch_Cal));

  for(int cal_samp = 0; cal_samp < 2000; cal_samp ++){

        gyro_signals(Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate);
        accel_signals(Sensor_AccX, Sensor_AccY, Sensor_AccZ);
        gyro_signals(Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate);
        accel_signals(Sensor_AccX, Sensor_AccY, Sensor_AccZ);

        //Gyro Rate Calibration
        Sensor_RollRate -= RollRate_Cal;
        Sensor_PitchRate -= PitchRate_Cal;
        Sensor_YawRate -= YawRate_Cal;

        //Acceleration Calibration (these are hard set values that are empirically obtained tilt and measure)
        AccX_Cal = 0.05;
        AccY_Cal = 0;
        AccZ_Cal = -0.13;

        Sensor_AccX -= AccX_Cal;
        Sensor_AccY -= AccY_Cal;
        Sensor_AccZ -= AccZ_Cal;

        //Gyro Integration
        gyro_angle(G_Roll, G_Pitch, G_Yaw, Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate);
        accel_angle(A_Roll, A_Pitch, A_Yaw, Sensor_AccX, Sensor_AccY, Sensor_AccZ);

          //KALMAN FILTER 1D
        //ROLL
        kalman1D(Kalman1D, KalmanRoll, KalmanUncertaintyRoll, Sensor_RollRate, A_Roll);
        KalmanRoll = Kalman1D[0];
        KalmanUncertaintyRoll = Kalman1D[1];
        //rollKG = Kalman1D[2];

        //PITCH
        kalman1D(Kalman1D, KalmanPitch, KalmanUncertaintyPitch, Sensor_PitchRate, A_Pitch);
        KalmanPitch = Kalman1D[0];
        KalmanUncertaintyPitch = Kalman1D[1];
        //pitchKG = Kalman1D[2];


        Roll_Cal += KalmanRoll;
        Pitch_Cal += KalmanPitch;
  }

  Roll_Cal /= 2000;
  Pitch_Cal /= 2000;

  Serial.println("ROLL/PITCH CAL: " + String(Roll_Cal) + "|" + String(Pitch_Cal)); 


  receiver_setup();
  motor_init(6,7,8,9);

  delay(1000);
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);

}

void loop() {

  unsigned long start_time = micros();

  gyro_signals(Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate);
  accel_signals(Sensor_AccX, Sensor_AccY, Sensor_AccZ);

  //Gyro Rate Calibration
  Sensor_RollRate -= RollRate_Cal;
  Sensor_PitchRate -= PitchRate_Cal;
  Sensor_YawRate -= YawRate_Cal;

  //Acceleration Calibration (these are hard set values that are empirically obtained tilt and measure)
  AccX_Cal = 0.05;
  AccY_Cal = 0;
  AccZ_Cal = -0.13;

  Sensor_AccX -= AccX_Cal;
  Sensor_AccY -= AccY_Cal;
  Sensor_AccZ -= AccZ_Cal;

  //Gyro Integration
  gyro_angle(G_Roll, G_Pitch, G_Yaw, Sensor_RollRate, Sensor_PitchRate, Sensor_YawRate);
  accel_angle(A_Roll, A_Pitch, A_Yaw, Sensor_AccX, Sensor_AccY, Sensor_AccZ);

  receiver_read(ReceiverValues);

  //KALMAN FILTER 1D
  //ROLL
  kalman1D(Kalman1D, KalmanRoll, KalmanUncertaintyRoll, Sensor_RollRate, A_Roll);
  KalmanRoll = Kalman1D[0];
  KalmanUncertaintyRoll = Kalman1D[1];
  //rollKG = Kalman1D[2];

  //PITCH
  kalman1D(Kalman1D, KalmanPitch, KalmanUncertaintyPitch, Sensor_PitchRate, A_Pitch);
  KalmanPitch = Kalman1D[0];
  KalmanUncertaintyPitch = Kalman1D[1];
  //pitchKG = Kalman1D[2];

  float desired_roll = -0.07*(ReceiverValues[0] - 1500);
  float desired_pitch = -0.07*(ReceiverValues[1] - 1500);
  float thrust = ReceiverValues[2];
  float desired_yaw = 0.15*(ReceiverValues[3] - 1500);
  float K_Value = ReceiverValues[4];
  float Select = ReceiverValues[5];


  // PID TUNING CODE
  int K_select = map(Select, 990, 1990, 1, 3);
  K_Value = map(K_Value, 1000, 1990, 0, 200);

  switch(K_select){
    case 1:
      //Serial.print(String(K_select)); Serial.print("\tcurrKP = " + String(roll_Kp)); 
      //Serial.println("\tKP = " + String(K_Value));
      roll_Kp = K_Value;
      break;
    case 2: 
      //Serial.print(String(K_select)); Serial.print("\tcurrKI = " + String(roll_Ki)); 
      //Serial.println("\tKI = " + String(K_Value));
      roll_Ki = K_Value;
      break;
    case 3:
      //Serial.print(String(K_select)); Serial.print("\tcurrKD = " + String(roll_Kd)); 
      //Serial.println("\tKD = " + String(K_Value));
      roll_Kd = K_Value;
      break;
  }




  // EVERTHING AFTER THIS POINT IS EXPERIMENTAL AND SUBJECT TO CHANGE
  // motor_test(THRUST);

  if (thrust > 1800) thrust = 1800; //Thrust Limit

  // TEST LOGIC (EXPERIMENTAL IDK)
  if(desired_yaw <= 2 && desired_yaw >= -2) G_Yaw = 0;

  //====================PID Attitude Controller====================
  roll_err = err(KalmanRoll, desired_roll) + Roll_Cal;
  
  // ADDING DEADBAND HERE
  if(abs(roll_err) < 5)  roll_err = 0;
  // DEADBAND CODE

  pid(pid_out, roll_err, roll_Kp, roll_Ki, roll_Kd, prev_roll_err, prev_roll_Ki);
  pid_roll = pid_out[0];
  prev_roll_err = pid_out[1];
  prev_roll_Ki = pid_out[2];

  pitch_err = err(KalmanPitch, desired_pitch) + Pitch_Cal;

  // ADDING DEADBAND HERE
  if(abs(pitch_err) < 5)  pitch_err = 0;
  // DEADBAND CODE

  pid(pid_out, pitch_err, pitch_Kp, pitch_Ki, pitch_Kd, prev_pitch_err, prev_pitch_Ki);
  pid_pitch = pid_out[0];
  prev_pitch_err = pid_out[1];
  prev_pitch_Ki = pid_out[2];

  //yaw_err = err(Sensor_YawRate, desired_yaw);
  //pid(pid_out, yaw_err, yaw_Kp, yaw_Ki, yaw_Kd, prev_yaw_err, prev_yaw_Ki);
  //pid_yaw = pid_out[0];
  //prev_yaw_err = pid_out[1];
  //prev_yaw_Ki = pid_out[2];

  //====================PID Attitude Controller====================

  //====================PID Rate Controller====================
  // PID Roll Rate
  rollRate_err = err(Sensor_RollRate, pid_roll);
  pid(pid_out, rollRate_err, rollRate_Kp, rollRate_Ki, rollRate_Kd, prev_rollRate_err, prev_rollRate_Ki);
  pid_rollRate = pid_out[0];
  prev_rollRate_err = pid_out[1];
  prev_rollRate_Ki = pid_out[2];

  //PID Pitch Rate
  pitchRate_err = err(Sensor_PitchRate, pid_pitch);
  pid(pid_out, pitchRate_err, pitchRate_Kp, pitchRate_Ki, pitchRate_Kd, prev_pitchRate_err, prev_pitchRate_Ki);
  pid_pitchRate = pid_out[0];
  prev_pitchRate_err = pid_out[1];
  prev_pitchRate_Ki = pid_out[2];

  //PID Yaw Rate
  yawRate_err = err(Sensor_YawRate, desired_yaw);
  pid(pid_out, yawRate_err, yawRate_Kp, yawRate_Ki, yawRate_Kd, prev_yawRate_err, prev_yawRate_Ki);
  pid_yawRate = pid_out[0];
  prev_yawRate_err = pid_out[1];
  prev_yawRate_Ki = pid_out[2];
  //====================PID Rate Controller====================

// CUT all motor signals if thrust is less than 1050 
// Should add condition to cut motor signals if another button is toggles (KILL SWITCH)
  if(thrust < 1110){
    // PID TERMS TO ZERO
    pid_rollRate = 0;
    pid_pitchRate = 0;
    pid_yawRate = 0;
    pid_pitch = 0;
    pid_roll = 0;
    pid_yaw = 0;
    // RESET PREV RATE VALUES
    prev_rollRate_err = 0;
    prev_pitchRate_err = 0;
    prev_yawRate_err = 0;
    prev_rollRate_Ki = 0;
    prev_pitchRate_Ki = 0;
    prev_yawRate_Ki = 0;
    // RESET PREV 0 VALUES
    prev_roll_err = 0.0;
    prev_pitch_err = 0.0;
    prev_yaw_err = 0.0;
    prev_roll_Ki = 0.0;
    prev_pitch_Ki = 0.0;
    prev_yaw_Ki = 0.0;
  }

  //Send Motor Signals
  motor_mixing(thrust, pid_rollRate, pid_pitchRate, pid_yawRate);

  unsigned long end_time = micros();
  unsigned long duration = end_time - start_time;
  if(duration < 2500) delayMicroseconds(2500 - duration);

  //Serial.print("LOOP TIME: "); Serial.println(end_time - start_time);
  //Serial.print("thrust_cmd: "); Serial.print(thrust); Serial.print("  \t");
  //Serial.print("roll_cmd: "); Serial.print(pid_roll); Serial.print("  \t");
  //Serial.print("pitch_cmd: "); Serial.print(pid_pitch); Serial.print("  \t");
  //Serial.print("yaw_cmd: "); Serial.print(pid_yaw); Serial.println("  \t");
  
  //Serial.print("thrust_mtr:\t"); Serial.print(thrust); Serial.print("\t");
  //Serial.print("roll_mtr:\t"); Serial.print(desired_roll); Serial.print("\t");
  //Serial.print("pitch_mtr:\t"); Serial.print(desired_pitch); Serial.print("\t");
  //Serial.print("yaw_mtr:\t"); Serial.print(desired_yaw); Serial.println("\t");
  
  //Serial.print(desired_roll); Serial.print("|"); Serial.print(desired_pitch); Serial.print("|"); Serial.println(desired_yaw);
  //Serial.println(String(A_Roll) + "|" + String(A_Pitch) + "|" + String(G_Yaw) + "|" + String(KalmanRoll) + "|" + String(KalmanPitch));
  //float Roll = KalmanRoll - Roll_Cal;
  //float Pitch = KalmanPitch - Pitch_Cal;
  //Serial.println("," + String(Roll) + "|" + String(Pitch) + "|" + String(G_Yaw)); // the one for GUI
  Serial.println("," + String(KalmanRoll - Roll_Cal) + "|" + String(KalmanPitch - Pitch_Cal) + "|" + String(G_Yaw)); //+"|"+ String(desired_roll) +"|"+ String(desired_pitch) +"|"+ String(desired_yaw));
  
 // Serial.print("\t M_0: " + String(KalmanRoll - Roll_Cal) + "|" + String(KalmanPitch - Pitch_Cal));
 // Serial.print("\t D_0: " + String(desired_roll) +"|"+ String(desired_pitch));
 // Serial.print("\t error_0: " + String(roll_err) +"|"+ String(pitch_err));
 // Serial.print("\t pid_0: " + String(pid_roll) +"|"+ String(pid_pitch));

  //Rates
  //Serial.print("\t\t pid_rates: " + String(pid_rollRate) +"|"+ String(pid_pitchRate));
  //Serial.print("\t\t gyrorates: " + String(Sensor_RollRate) +"|"+ String(Sensor_RollRate));
  //Serial.println("\t\t rate_errors: " + String(rollRate_err) +"|"+ String(pitchRate_err));

};