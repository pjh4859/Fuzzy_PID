#ifndef __PID_H__
#define __PID_H__

#include <math.h>

typedef struct PID
{
	float err[3]; 
        float Angularerr[3];
	float err_last[3]; 
        float inner_last[3];
	float Kp[3],Ki[3],Kd[3];
        float iKp[3],iKi[3],iKd[3];
	float output[3];
	float integral[3];
}__PID;


//void pid_init(struct PID * pid, float p, float i, float d);
void pid_init(struct PID * pid, float pid_val[][3], float inpid_val[][3]);
void pid_gain_update(struct PID * pid, float pid_val[][3], float inpid_val[][3]);
void __pid_update(__PID * pid, float * setting_angle, float * Euler_angle, float * angular_velocity);
void pid_update(__PID * pid, float set, float actual, float angular_velocity, int axis);


#endif

