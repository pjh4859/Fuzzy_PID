#ifndef __PID_H__
#define __PID_H__

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

typedef struct PID
{
	float err[3]; 
        float rateError[3];
	float err_last[3]; 
        float preRateError[3];
	float Kp[3],Ki[3],Kd[3];
        float iKp[3],iKi[3],iKd[3];
	float output[3];
	float integral[3];
}__PID;

void pid_init(__PID * pid, float pid_val[][3], float inpid_val[][3]);
void pid_gain_update(__PID * pid, float pid_val[][3], float inpid_val[][3]);
void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float * angular_velocity, float deltat);
void pid_update(__PID * pid, float set, float actual, float angular_velocity, int axis, float deltat);
//=========================Parsing part=============================
void Parsing_PID_val(uint8_t* arr, float pid_val[][3]);
void Parsing_inPID_val(uint8_t* arr, float pid_val[][3]);
void Parsing_Throttle_val(uint8_t* arr, int *Controller_1);
void Parsing_SettingPoint_val(uint8_t* arr, float* setting_angle);
//=======================Parsing part END===========================

#endif

