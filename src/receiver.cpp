#include "receiver.h"
PulsePositionInput ReceiverInput(RISING);

void receiver_setup(){
    //digitalWrite(3, LOW);
    ReceiverInput.begin(14);
    //digitalWrite(3, HIGH);

}

void receiver_read(float *ReceiverValue){
    
    int ChannelNum = ReceiverInput.available();

    if(ChannelNum > 0){
        for(int i = 1; i < ChannelNum; i++){
            ReceiverValue[i-1] = ReceiverInput.read(i);
            }
    }
    
}