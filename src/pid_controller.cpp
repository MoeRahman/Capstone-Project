#include "pid_controller.h"


float err(float &A, float &B){
    return B - A;
}

void pid(float *pid_return, float &Error, float &Kp, float &Ki, float &Kd, float &Prev_Error, float &Prev_Ki){
    
    float P = Kp*Error;
    float I = Prev_Ki + Ki*(Error + Prev_Error)*0.0025/2;
    
    // Mitigate Integal Windup
    if (I > 400){
        I = 400;
    }
    else if (I < -400){
        I = -400;
    }
    
    float D = Kd*(Error - Prev_Error)/0.0025; // Ts = 2000us (500 Hz cycle) - 0.002
    float pid_output = P + I + D;
    
    if (pid_output > 400){
        pid_output = 400;
    }
    else if (pid_output < -400){
        pid_output = -400;
    }
    
    pid_return[0] = pid_output;
    pid_return[1] = Error; 
    pid_return[2] = I;
    

}

void pid_reset(){
    
}

