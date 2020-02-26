#include "PWM.h"



void Motor_Test(void)
{
  int i = 0;
  for (i = MIN_PULSE; i < MAX_PULSE;)
  {
          //  TIM2->CCR1 = i;
          MOTOR_V1 = i;
          MOTOR_V2 = i;
          MOTOR_V3 = i;
          MOTOR_V4 = i;
          HAL_Delay(100);
          i += 50;
  }
}

void Motor_Test_2(void)
{
  int i = 0;
  int a = 0;
  int b = 9700;
  for (i = MIN_PULSE; i < 9700;)
  {
          //  TIM2->CCR1 = i;
          MOTOR_V1 = i;
          MOTOR_V2 = i;
          MOTOR_V3 = i;
          MOTOR_V4 = i;
          HAL_Delay(100);
          i += 10;
  }
  for (i = 9700; i < 10500;)
  {
          //  TIM2->CCR1 = i;
          MOTOR_V1 = i;
          MOTOR_V2 = i;
          MOTOR_V3 = b - a;
          MOTOR_V4 = i;
          HAL_Delay(100);
          i += 10;
          a += 10;
  }
}

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
  //HAL_Delay(100);
  
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
  //
  //MOTOR_V1 += 100;
  //MOTOR_V2 += 100;
  //MOTOR_V3 += 100;
  //MOTOR_V4 += 100;
  ////HAL_Delay(100);
}

void Motor_Stop(int count)
{
  if (HAL_GetTick() - before_while > count)
  {
     MOTOR_V1 = MIN_PULSE;
     MOTOR_V2 = MIN_PULSE;
     MOTOR_V3 = MIN_PULSE;
     MOTOR_V4 = MIN_PULSE;
  }
}
