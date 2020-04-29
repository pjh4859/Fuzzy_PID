#ifndef __LPF_H__
#define __LPF_H__

#include "tm_stm32_mpu9250.h"

void __LPF(float * LPF_Euler_angle, float * Euler_angle, float * preEuler_angle, float deltat);
void __LPFGyro(float * LPF_Gyro, TM_MPU9250_t* MPU9250, float * preGyro, float deltat);
//void __HPF(float * y, float* x, float * pre_y, float pre_x, float deltat);


#endif
