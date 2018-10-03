/*
 * Function to compute the motor speed using IC2 interrupts
 * January, 2018
 * Author: Rodrigo A. Romano
 */

#include "mcc_generated_files/mcc.h"
#include "myglobal_defs.h"

uint16_t pastCapture;
uint16_t actualCapture;
int16_t motorSpeed;

void ComputePeriod(void)
{
    pastCapture = actualCapture;
    actualCapture = IC2_CaptureDataRead();  // Read IC2 FIFO buffer
    
    while(!IC2_IsCaptureBufferEmpty())  // Empty IC2 buffer
        IC2_CaptureDataRead();   
}