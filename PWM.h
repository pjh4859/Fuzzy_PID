#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f4xx_hal.h"

//¸ðÅÍ°¡ È¸ÀüÇÏ´Â ÆÞ½ºÆø : 8600 ~ 15500
#define MIN_PULSE 8000
#define MAX_PULSE 15500

//#define MOTOR_V1 TIM2->CCR1
#define MOTOR_V1 TIM5->CCR1
#define MOTOR_V2 TIM2->CCR2
//#define MOTOR_V3 TIM3->CCR1
#define MOTOR_V3 TIM2->CCR1
//#define MOTOR_V4 TIM3->CCR2
#define MOTOR_V4 TIM3->CCR1

#define MoterGain_roll       1.0f
#define MoterGain_pitch      1.0f
#define MoterGain_yaw        1.0f

#define MOTER_SAFTY             (1000)

void ESC_Calibration(void);
void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(int count, uint32_t before_while);

void Motor_Drive(int Throttle, float *PID);
//void Motor_Drive(int Throttle, float Roll_PID, Pitch_PID, Yaw_PID);

#endif