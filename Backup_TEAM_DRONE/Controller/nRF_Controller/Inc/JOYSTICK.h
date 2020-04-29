#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/*=========DEFINE=========*/
#define Key_Mode_Flag        2
#define Debug_Mode_Flag      1
#define Drive_Mode_Flag      0

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

extern UART_HandleTypeDef huart2;

/*=========FUNCTION==========*/
char INPUT_THROTTLE();
char INPUT_CONTORLLER();
void Print_Mode(uint8_t* Mode_Flag);
void Print_ADC_DEBUG();

/*=========VAL ADC=========*/
extern int8_t ADC_DATA[4];  
//extern uint8_t buffer_ADC[50];