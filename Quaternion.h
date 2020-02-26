#ifndef __Quaternion_H__
#define __Quaternion_H__

#include <math.h>

extern float twoKp;											// 2 * proportional gain (Kp)
extern float twoKi;											// 2 * integral gain (Ki)
extern float q0, q1, q2, q3;					// quaternion of sensor frame relative to auxiliary frame
extern float integralFBx,  integralFBy, integralFBz;	// integral error terms scaled by Ki

void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MahonyAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void Quternion2Euler(float *q);
float invSqrt(float x);


extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);


#endif
