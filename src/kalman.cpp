#include "kalman.h"



void kalman1D(float *KalmanReturn, 
              float KalmanState, 
              float KalmanUncertainty, 
              float KalmanInput, 
              float KalmanMeasurement){

                KalmanState = KalmanState + 0.004*KalmanInput;
                KalmanUncertainty = KalmanUncertainty + 0.004*0.004*16;
                float KalmanGain = KalmanUncertainty *1/(KalmanUncertainty + 9);
                KalmanState = KalmanState + KalmanGain*(KalmanMeasurement-KalmanState);
                KalmanUncertainty =(1-KalmanGain)*KalmanUncertainty;

                KalmanReturn[0] = KalmanState;
                KalmanReturn[1] = KalmanUncertainty;
                KalmanReturn[2] = KalmanGain;

}