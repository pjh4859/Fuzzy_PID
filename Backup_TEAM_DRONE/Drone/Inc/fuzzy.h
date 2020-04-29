#ifndef __FUZZY_H__
#define __FUZZY_H__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define min(X,Y) ((X) < (Y) ? (X) : (Y))  
#define max(X,Y) ((X) > (Y) ? (X) : (Y))

typedef struct __fuzzy_stru
{
    float Fuzzy_Val[3];
    float Selected_P[3];
    float Selected_I[3];
    float Selected_D[3];
}_Fuzzy_stru;

void fuzzy_init();
void Fuzzification(float setting_angle, float Euler_angle, float *prev_err);
void Create_Fuzzy_Matrix(uint8_t flag);
void Defuzzification(float *Kp, float *Ki, float *Kd, uint8_t flag);
void Find_Maxarr(_Fuzzy_stru flagarr[], int flag, int flag2, uint8_t flag3);

#endif
