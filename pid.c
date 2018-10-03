/*
 * File:   pid.c
 * Author: gchinellato
 *
 * Created on 1 de Outubro de 2018, 19:53
 */

#include "xc.h"
#include "myglobal_defs.h"
  
#define T   0.005
#define Kc  0.17778 //nossa Kp= 0.1288 Ki=0.0825
#define Ti  3
#define Td  0
#define N   20
#define Tt  Ti/20

#define umin    -749
#define umax    750

int16_t setPoint = 0;
int16_t yold = 0;
float I = 0;
float D = 0;

//const float bi = Kc*T/Ti ; /* Integral gain */
//const float ad = Td/(T*N+Td); /* derivative coefficients term */
//const float bd = N*Kc*ad;
//const float a0 = T/Tt ; /* wind -up reset constant */
#define bi (Kc*T/Ti)
#define ad (Td/(T*N+Td))
#define bd (N*Kc*ad)
#define a0 (T/Tt)

/* PID algorithm
Inputs : samples of the set - point and the process output signals
(r and y, respectively ).
Output : Saturated control signal ( usat ). */

float computePID(int16_t y) {
    
    if (setPoint == 0)
        return 0;
    
    float e = setPoint - y ; /* calculate the error signal */
    float P = Kc * e; /* compute proportional term */
    D = ad * D - bd * ( y - yold ) ; /* update derivative term */
    float u = P+I+D; /* compute temporary output */
    
    /* compute bounded output */
    float usat = M_SAT(u, umin, umax);
    
    I = I + bi * e + a0 *(usat - u) ; /* update integral term */
    yold = y; /* update old process output */
    
    return usat;
}
