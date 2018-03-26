/*
 * File:   phase_control.c
 * Author: gchinellato
 *
 * Created on March 19, 2018, 7:34 PM
 */


#include "mcc_generated_files/mcc.h"
#include "phase_control.h"

#define EN1 (LATBbits.LATB15) //PWM1L
#define EN2 (LATBbits.LATB13) //PWM2L
#define EN3 (LATBbits.LATB11) //PWM3L

uint8_t HallValue ;

/*
 * Hall         |
 * 101b (5dec)  | State 3
 * 001b (1dec)  | State 2
 * 011b (3dec)  | State 1
 * 010b (2dec)  | State 6
 * 110b (6dec)  | State 5
 * 100b (4dec)  | State 4
 * 
    Hall = 001b = 1
    EN1=1 EN2=0 EN3=1
    IN1=1 IN3=0
    State=2

    Hall = 010b = 2
    EN1=1 EN2=1 EN3=0
    IN1=0 IN2=1
    State=6

    Hall = 011b = 3
    EN1=0 EN2=1 EN3=1
    IN2=1 IN3=0
    State=1

    Hall = 100b = 4
    EN1=0 EN2=1 EN3=1
    IN2=0 IN3=1
    State=4

    Hall = 101b = 5
    EN1=1 EN2=1 EN3=0
    IN1=1 IN2=0
    State=3

    Hall = 110b = 6
    EN1=1 EN2=0 EN3=1
    IN1=0 IN3=1
    State=5
 */
uint8_t CommutTable[] = {0, 2, 6, 1, 4, 3, 5};

void Commut_Phase(void){
    LATB &= 0x57FF; // Turn off all motor phases
    IOCON1bits.OVRDAT1 = 0; //PWM1H Pin override output
    IOCON2bits.OVRDAT1 = 0; //PWM2H Pin override output
    IOCON3bits.OVRDAT1 = 0; //PWM3H Pin override output
    
    // Get Hall sensor state
    HallValue = (uint8_t) (PORTB & 0x03);
    if (PORTBbits.RB7)
        HallValue += 4;
    
    switch (CommutTable[HallValue]){
        case 1: // DRV8313 State : 1
            EN2 = 1;
            IOCON2bits.OVRENH = 0;
            EN3 = 1;
            IOCON3bits.OVRENH = 1;
            break;
        case 2:
            EN1 = 1;
            IOCON1bits.OVRENH = 0;
            EN3 = 1;
            IOCON3bits.OVRENH = 1; 
            break;
        case 3: 
            EN1 = 1;
            IOCON1bits.OVRENH = 0;
            EN2 = 1;
            IOCON2bits.OVRENH = 1; 
            break;
        case 4: 
            EN2 = 1;
            IOCON2bits.OVRENH = 1;
            EN3 = 1;
            IOCON3bits.OVRENH = 0; 
            break;
        case 5: 
            EN1 = 1;
            IOCON1bits.OVRENH = 1; 
            EN3 = 1;
            IOCON3bits.OVRENH = 0; 
            break;
        case 6: 
            EN1 = 1;
            IOCON1bits.OVRENH = 1; 
            EN2 = 1;
            IOCON2bits.OVRENH = 0; 
            break;
        default:
            break;
    }
}

