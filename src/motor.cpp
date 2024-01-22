
#include "motor.h"
Servo MOTOR_ONE;
Servo MOTOR_TWO;
Servo MOTOR_THREE;
Servo MOTOR_FOUR;

void motor_init(int motorOnePin, int motorTwoPin, int motorThreePin, int motorFourPin){

    MOTOR_ONE.attach(motorOnePin,1000,2000);
    MOTOR_TWO.attach(motorTwoPin,1000,2000);
    MOTOR_THREE.attach(motorThreePin,1000,2000);
    MOTOR_FOUR.attach(motorFourPin,1000,2000);

    digitalWrite(4, LOW);
    MOTOR_ONE.writeMicroseconds(2000);
    MOTOR_TWO.writeMicroseconds(2000);
    MOTOR_THREE.writeMicroseconds(2000);
    MOTOR_FOUR.writeMicroseconds(2000);
    delay(5000);
    digitalWrite(4, HIGH);


    MOTOR_ONE.writeMicroseconds(1000);
    MOTOR_TWO.writeMicroseconds(1000);
    MOTOR_THREE.writeMicroseconds(1000);
    MOTOR_FOUR.writeMicroseconds(1000);
    delay(5000);
    digitalWrite(4, LOW);
}

void motor_test(float &Thrust){

    // Safety Parameters to ensure motors never uses 100% Throttle (avoid overcurrent)
    // Set a Throttle IDLE STATE and SHUTOFF STATE to differentiate motor on and off
    // MOTOR MAX THROTTLE
    if (Thrust > 1950) Thrust = 1950;

    // MOTOR IDLE STATE
    if (Thrust < 1180) Thrust = 1180;

    // MOTOR SHUTOFF STATE
    if (Thrust < 1050) Thrust = 1000;

    MOTOR_ONE.writeMicroseconds(Thrust);
    MOTOR_TWO.writeMicroseconds(Thrust);
    MOTOR_THREE.writeMicroseconds(Thrust);
    MOTOR_FOUR.writeMicroseconds(Thrust);
}

void motor_mixing(float &Thrust, float &Roll, float &Pitch, float &Yaw){

    // DRONE X CONFIGURATION
    float M1 = 1.024*(Thrust + Roll); // + Pitch);
    float M2 = 1.024*(Thrust - Roll); // + Pitch);
    float M3 = 1.024*(Thrust - Roll); // - Pitch);
    float M4 = 1.024*(Thrust + Roll); // - Pitch);

    // Safety Parameters to ensure motors never uses 100% Throttle (avoid overcurrent)
    // Set a Throttle IDLE STATE and SHUTOFF STATE to differentiate motor on and off
    // MOTOR MAX THROTTLE
    if (M1 > 1950) M1 = 1950;
    if (M2 > 1950) M2 = 1950;
    if (M3 > 1950) M3 = 1950;
    if (M4 > 1950) M4 = 1950;

    // MOTOR SHUTOFF STATE
    if (Thrust < 1100){
        M1 = 1000;
        M2 = 1000;
        M3 = 1000;
        M4 = 1000;
    } 

    if (M1 < 1100) M1 = 1000;
    if (M2 < 1100) M2 = 1000;
    if (M3 < 1100) M3 = 1000;
    if (M4 < 1100) M4 = 1000;

    // MOTOR IDLE STATE
    if (((1100 < M1) && (M1 < 1180)) || ((Thrust < 1180) && (Thrust > 1100))) M1 = 1180;
    if (((1100 < M2) && (M2 < 1180)) || ((Thrust < 1180) && (Thrust > 1100))) M2 = 1180;
    if (((1100 < M3) && (M3 < 1180)) || ((Thrust < 1180) && (Thrust > 1100))) M3 = 1180;
    if (((1100 < M4) && (M4 < 1180)) || ((Thrust < 1180) && (Thrust > 1100))) M4 = 1180;

    // IDLE LOGIC


    //Serial.print("Motors: "); 
    Serial.print(String(M1) + "|" + String(M2) + "|" + String(M3) + "|" + String(M4));
    MOTOR_ONE.writeMicroseconds(M1);
    MOTOR_TWO.writeMicroseconds(M2);
    MOTOR_THREE.writeMicroseconds(M3);
    MOTOR_FOUR.writeMicroseconds(M4);
    
}