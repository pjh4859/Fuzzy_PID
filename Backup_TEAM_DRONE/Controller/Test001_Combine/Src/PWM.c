#include "PWM.h"

void ESC_Calibration(void)
{
  //  TIM2->CCR1 = 15500;
  TIM2->CCR1 = MAX_PULSE;
  TIM2->CCR2 = MAX_PULSE;
  TIM3->CCR1 = MAX_PULSE;
  TIM3->CCR2 = MAX_PULSE;
  HAL_Delay(5000);
  //  TIM2->CCR1 = 8500;
  TIM2->CCR1 = MIN_PULSE;
  TIM2->CCR2 = MIN_PULSE;
  TIM3->CCR1 = MIN_PULSE;
  TIM3->CCR2 = MIN_PULSE;
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
