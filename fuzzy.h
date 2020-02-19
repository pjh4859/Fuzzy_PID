#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<stdint.h>

#define min(X,Y) ((X) < (Y) ? (X) : (Y))  
#define max(X,Y) ((X) > (Y) ? (X) : (Y))

typedef struct __fuzzy_stru
{
    float Fuzzy_Val;
    uint8_t index;
    float Selected_PID[3];

}_Fuzzy_stru;

void fuzzy_init();
void Fuzzification(float setting_angle, float Euler_angle, float *prev_err);
void Create_Fuzzy_Matrix();
void Defuzzification(float *Kp, float *Ki, float *Kd);
void Find_Maxarr(_Fuzzy_stru flagarr[], int flag, int flag2);