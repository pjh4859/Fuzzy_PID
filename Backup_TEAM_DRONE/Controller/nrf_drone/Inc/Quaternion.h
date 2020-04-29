#ifndef __Quaternion_H__
#define __Quaternion_H__

#include <math.h>

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q, float deltat);
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz, float* q, float deltat);
void Quternion2Euler(float *q, float *Euler_angle);
float invSqrt(float x);
void MahonyQuaternionUpdate2(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q, float deltat);

#endif
