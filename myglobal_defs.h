/* 
 * File:   myglobal_defs.h
 * Author: Professores.IMT
 *
 * Created on January 30, 2018, 6:56 PM
 */

#ifndef MYGLOBAL_DEFS_H
#define	MYGLOBAL_DEFS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <stdint.h>
  
// Constants
#define PERIOD2RPM  7031250
    
// Macros    
#define M_BYTES_TO_WORD(byteH, byteL) ((uint16_t)(((uint16_t)byteL & 0x00FF)|(((uint16_t)byteH << 8) & 0xFF00)))    
#define M_SAT(input, min, max) ((input < min) ? min : ((input > max) ? max : input))

// Variables
extern uint8_t i2c1Buffer[64];
extern uint16_t pastCapture;
extern uint16_t actualCapture;
extern int16_t motorSpeed;
extern int16_t setPoint;
    
#ifdef	__cplusplus
}
#endif

#endif	/* MYGLOBALDEFS_H */
