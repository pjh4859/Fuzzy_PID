#include "PWM.h"

void ESC_Calibration(void)
{
  //  TIM2->CCR1 = 15500;
  TIM5->CCR1 = MAX_PULSE;
  TIM2->CCR2 = MAX_PULSE;
  TIM2->CCR1 = MAX_PULSE;
  TIM3->CCR1 = MAX_PULSE;
  HAL_Delay(5000);
  //  TIM2->CCR1 = 8500;
  TIM5->CCR1 = MIN_PULSE;
  TIM2->CCR2 = MIN_PULSE;
  TIM2->CCR1 = MIN_PULSE;
  TIM3->CCR1 = MIN_PULSE;
  HAL_Delay(4000);
}


void Motor_Init(void)
{
  MOTOR_V1 = MIN_PULSE;
  MOTOR_V2 = MIN_PULSE;
  MOTOR_V3 = MIN_PULSE;
  MOTOR_V4 = MIN_PULSE;
  //HAL_Delay(5000);
}

void Motor_Start(void)
{
  //MOTOR_V1 = MIN_PULSE + 200;
  MOTOR_V1 = MIN_PULSE + 700;//700
  MOTOR_V2 = MIN_PULSE + 700;
  MOTOR_V3 = MIN_PULSE + 700;
  MOTOR_V4 = MIN_PULSE + 700;
}

void Motor_Stop(int count, uint32_t before_while)
{
  if (HAL_GetTick() - before_while > count)
  {
     MOTOR_V1 = MIN_PULSE;
     MOTOR_V2 = MIN_PULSE;
     MOTOR_V3 = MIN_PULSE;
     MOTOR_V4 = MIN_PULSE;
  }
}

void Motor_Drive(int Throttle, float *PID)
{
  if (Throttle <= 5)
  {
    MOTOR_V1 = MIN_PULSE;
    MOTOR_V2 = MIN_PULSE;
    MOTOR_V3 = MIN_PULSE;
    MOTOR_V4 = MIN_PULSE;
  }
  
  else if (Throttle > 5)    //Controller_1
  {     
//    if (fabs(Euler_angle[0]) > 15.0f || fabs(Euler_angle[1]) > 15.0f)    //Restrict yaw acting Euler angle.
//    {           
//      pid.output[2] = 0.0f;
//    }         
    
    //MOTOR_V1 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * pid.output[0]) - (int)(MoterGain_pitch * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]);
    
    //MOTOR_V1 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[0]) - (int)(MoterGain_roll * PID[1]) + (int)(MoterGain_roll * PID[2]);
    MOTOR_V1 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[0]);
    //MOTOR_V1 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[1]);
    //MOTOR_V1 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[2]);
  
    //MOTOR_V2 = MIN_PULSE + (Throttle * 65) - (int)((MoterGain_roll) * pid.output[0]) - (int)((MoterGain_pitch) * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]);
    
    //MOTOR_V2 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[0]) - (int)(MoterGain_roll * PID[1]) - (int)(MoterGain_roll * PID[2]);
    MOTOR_V2 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[0]);
    //MOTOR_V2 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[1]);
    //MOTOR_V2 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[2]);
  
    //MOTOR_V3 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * pid.output[0]) + (int)(MoterGain_pitch * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]);
    
    //MOTOR_V3 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[0]) + (int)(MoterGain_roll * PID[1]) - (int)(MoterGain_roll * PID[2]);
    MOTOR_V3 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[0]);
    //MOTOR_V3 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[1]);
    //MOTOR_V3 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[2]);
    
    //MOTOR_V4 = MIN_PULSE + (Throttle * 65) - (int)((MoterGain_roll) * pid.output[0]) + (int)((MoterGain_pitch) * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]); 
    
    //MOTOR_V4 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[0]) + (int)(MoterGain_roll * PID[1]) + (int)(MoterGain_roll * PID[2]);
    MOTOR_V4 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[0]);
    //MOTOR_V4 = MIN_PULSE + (Throttle * 65) + (int)(MoterGain_roll * PID[1]);
    //MOTOR_V4 = MIN_PULSE + (Throttle * 65) - (int)(MoterGain_roll * PID[2]);
    
    
    if (MOTOR_V1 >= MAX_PULSE - MOTER_SAFTY)
      MOTOR_V1 = MAX_PULSE -  MOTER_SAFTY;
    else if (MOTOR_V1 <= MIN_PULSE + 700)
      MOTOR_V1 = MIN_PULSE + 700;
    
    if (MOTOR_V2 >= MAX_PULSE - MOTER_SAFTY)
      MOTOR_V2 = MAX_PULSE - MOTER_SAFTY;
    else if (MOTOR_V2 <= MIN_PULSE + 700)
      MOTOR_V2 = MIN_PULSE + 700;
    
    if (MOTOR_V3 >= MAX_PULSE - MOTER_SAFTY)
      MOTOR_V3 = MAX_PULSE - MOTER_SAFTY;
    else if (MOTOR_V3 <= MIN_PULSE + 700)
      MOTOR_V3 = MIN_PULSE + 700;
    
    if (MOTOR_V4 >= MAX_PULSE - MOTER_SAFTY)
      MOTOR_V4 = MAX_PULSE - MOTER_SAFTY;
    else if (MOTOR_V4 <= MIN_PULSE + 700)
      MOTOR_V4 = MIN_PULSE + 700;
   }
}
