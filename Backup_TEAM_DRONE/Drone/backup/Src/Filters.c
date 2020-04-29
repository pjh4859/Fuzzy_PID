#include "Filters.h"

#define tau        0.001f
#define tau2       0.001f
    
void __LPF(float * LPF_Euler_angle, float * Euler_angle, float * preEuler_angle, float deltat)
{
    LPF_Euler_angle[0] = (tau * preEuler_angle[0] + deltat * Euler_angle[0]) / (tau + deltat);
    LPF_Euler_angle[1] = (tau * preEuler_angle[1] + deltat * Euler_angle[1]) / (tau + deltat);
    LPF_Euler_angle[2] = (tau * preEuler_angle[2] + deltat * Euler_angle[2]) / (tau + deltat);
   
    preEuler_angle[0] = LPF_Euler_angle[0];
    preEuler_angle[1] = LPF_Euler_angle[1];
    preEuler_angle[2] = LPF_Euler_angle[2];
}

void __LPFGyro(float * LPF_Gyro, TM_MPU9250_t* MPU9250, float * preGyro, float deltat)
{
    LPF_Gyro[0] = (tau2 * preGyro[0] + deltat * MPU9250->Gx) / (tau2 + deltat);
    LPF_Gyro[1] = (tau2 * preGyro[1] + deltat * MPU9250->Gy) / (tau2 + deltat);
    LPF_Gyro[2] = (tau2 * preGyro[2] + deltat * MPU9250->Gz) / (tau2 + deltat);
    
    preGyro[0] = LPF_Gyro[0];
    preGyro[1] = LPF_Gyro[1];
    preGyro[2] = LPF_Gyro[2];
}

//void __HPF(float * y, float* x, float * pre_y, float pre_x, float deltat)
//{
//    y = tau/(tau + deltat) * pre_y + tau/(tau + deltat) * (x - pre_x);
//    pre_y = y;
//    pre_x = x;
//}
