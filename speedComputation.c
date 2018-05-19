/*
 * File:   speedComputation.c
 * Author: gchinellato
 *
 * Created on May 14, 2018, 8:03 PM
 */


#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/ic2.h"
#include "i2c_comm.h"
#include "speedComputation.h"

uint16_t pastCapture;
uint16_t actualCapture;
int16_t motorSpeed;

void GetPeriodData(void){
    pastCapture = actualCapture;
    actualCapture = IC2_CaptureDataRead(); //read IC2 FIFO buffer
    
    //empty IC2 buffer
    while(!IC2_IsCaptureBufferEmpty()){ 
        IC2_CaptureDataRead();
    }
}

void ComputeSpeed(void){
    uint16_t period = actualCapture - pastCapture;
    
    // Wmotor = 60 * 1/2 * Fcy /Npp * 1/Tmeasure
    motorSpeed = (int16_t)((uint32_t) (motorSpeed + PERIOD2RPM/period) >> 1);
    i2c1Buffer[0x1E] = (uint8_t)((motorSpeed >> 8) & 0x00FF);
    i2c1Buffer[0x1F] = (uint8_t)(motorSpeed & 0x00FF);
}
