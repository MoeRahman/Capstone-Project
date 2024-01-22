
#include "imu_functions.h"

void gyro_setup(void){
    // Start the gyro in power mode

    digitalWrite(2, LOW);
    digitalWrite(2, HIGH);
    Wire.setClock(400000);                                  //Set the clock speed of I2C
    Wire.begin();
    delay(250);
    //digitalWrite(2, LOW);

    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    //digitalWrite(3, HIGH);

}

void gyro_signals(float &ROLLRATE, float &PITCHRATE, float &YAWRATE){
    Wire.beginTransmission(0x68);                           //Start I2C communiaction with the gyro

    Wire.write(0x1A);                                       //Switch on Low-Pass Filter
    Wire.write(0x05);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);                           //Set the sensitivity scale factor
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();

    Wire.beginTransmission(0x68);                           //Access registers storing gyro measurements
    Wire.write(0x43);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);

    int16_t GyroX = Wire.read()<<8|Wire.read(); //  Read the Gyro measurements around the x axis
    int16_t GyroY = Wire.read()<<8|Wire.read(); //  Read the Gyro measurements around the y axis
    int16_t GyroZ = Wire.read()<<8|Wire.read(); //  Read the Gyro measurements around the z axis

    //  Convert data from lsb to deg/sec
    ROLLRATE = (float)GyroX/65.5;
    PITCHRATE = (float)GyroY/65.5;
    YAWRATE = (float)GyroZ/65.5;
}

void gyro_calibration(float &Roll_Cal, float &Pitch_Cal, float &Yaw_Cal){
    float Roll, Pitch, Yaw;
    
    for(int Calibration_Samples = 0; Calibration_Samples < 2000; Calibration_Samples ++){

        gyro_signals(Roll, Pitch, Yaw);
        Roll_Cal += Roll;
        Pitch_Cal += Pitch;
        Yaw_Cal += Yaw;
        delay(1);
    }


    Roll_Cal /= 2000;
    Pitch_Cal /= 2000;
    Yaw_Cal /= 2000;

    digitalWrite(4, HIGH);
}

void gyro_angle(float &g_roll, float &g_pitch, float &g_yaw,
                float &ROLLRATE, float &PITCHRATE, float &YAWRATE){
    
    g_roll = g_roll + ROLLRATE*0.002;
    g_pitch = g_pitch + PITCHRATE*0.002;
    g_yaw = g_yaw + YAWRATE*0.002;

}

void accel_setup(void){
    // Must be called after gyro_setup()
    Wire.beginTransmission(0x68);   // Configure the accelerometer output
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();
}

void accel_signals(float &AccX, float &AccY, float &AccZ){
    
    Wire.beginTransmission(0x68);   //Read Sensor Acceleration Data 
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    
    int16_t AccX_int16 = Wire.read() << 8 | Wire.read();
    int16_t AccY_int16 = Wire.read() << 8 | Wire.read();
    int16_t AccZ_int16 = Wire.read() << 8 | Wire.read();

    AccX = (float)AccX_int16/4096;
    AccY = (float)AccY_int16/4096;
    AccZ = (float)AccZ_int16/4096;

}

void accel_calibration(float &AccX_Cal, float &AccY_Cal, float &AccZ_Cal){

    digitalWrite(4, LOW);

    float AccX, AccY, AccZ;
    
    for(int Calibration_Samples = 0; Calibration_Samples < 2000; Calibration_Samples ++){

        gyro_signals(AccX, AccY, AccZ);
        AccX_Cal += AccX;
        AccY_Cal += AccY;
        AccZ_Cal += AccZ;
        delay(1);
    }


    AccX_Cal /= 2000;
    AccY_Cal /= 2000;
    AccZ_Cal /= 2000;

    digitalWrite(4, HIGH);
}

void accel_angle(float &a_roll, float &a_pitch, float &a_yaw,
                 float &AccX, float &AccY, float &AccZ){
    
    // Usign Accelerometer Trig we are able to calcualte Roll and Pitch
    // But not yaw since it z-axis does not experince a chagne in accelation with changes in yaw
    // This method is shit, huge vibrations 
    a_roll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
    a_pitch = atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
}


