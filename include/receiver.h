#ifndef RECEIVER_H
#define RECEIVER_H
#include <Arduino.h>
#include <PulsePosition.h>

void receiver_setup(void);
void receiver_read(float *ReceiverValue);


#endif