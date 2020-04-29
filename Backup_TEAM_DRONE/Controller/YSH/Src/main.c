/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MY_NRF24.h"
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

    
#define Roll_Modify    '1'
#define Pitch_Modify    '2'
#define Yaw_Modify      '3'

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void INPUT_KEYBOARD();
void Modify_R_PID();
void Modify_P_PID();
void Modify_Y_PID();
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/****NRF VAL******/
uint8_t buffer[255];
uint8_t data ;// UART IT 데이터
uint64_t TxpipeAddrs = 0x11223344AA;
char myTxData[32] = "Hello World!";
char AckPayload[32];

/**UART IT VAL**/
uint8_t C_buff[7];// 소숫점 저장용 변수
static uint8_t save_flag=0; 
static uint8_t C_count=0;
static char input_flag=0;

int fputc(int ch ,FILE *f)
{
 HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
 return ch;
}
 
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
   float P_buffer[3];
   float R_buffer[3];
   float Y_buffer[3];
   char c_buff[3]={'P','I','D'};
   char i;
   char mode;
  // uint8_t interface_status=0x00;
  
   P_buffer[0]=12.345;
   P_buffer[1]=23.345;
   P_buffer[2]=42.345;
  
   R_buffer[0]=.345;
   R_buffer[1]=23.345;
   R_buffer[2]=42.345;
   
   Y_buffer[0]=12.345;
   Y_buffer[1]=3.345;
   Y_buffer[2]=.345;
   
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 HAL_UART_Receive_IT(&huart2,&data,1);
 printf("start\r\n");

        printf("\r\n-----PITCH PID------\r\n");
 /**************ROLL****************/     
       for(i=0;i<3;i++)
       printf("R-%c : %6.3f ",c_buff[i],R_buffer[i]);
       printf("\n\r");
/**************PITCH****************/     
       for(i=0;i<3;i++)
       printf("P-%c : %6.3f ",c_buff[i],P_buffer[i]);
       printf("\n\r");
/**************YAW****************/ 
       for(i=0;i<3;i++)
       printf("Y-%c : %6.3f ",c_buff[i],Y_buffer[i]);
       printf("\n\r");
       
       printf("1. Roll_Modify \n\r\
2. Pitch_Modify \n\r\
3. Yaw_Modify \n\r");
       

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  { 
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
        // HAL_UART_Transmit(&huart2,buffer, sizeof(buffer),10);
   
      mode = data;
 
       switch(mode){
         
       case Roll_Modify://Roll
         data = 0;
         mode = 0;
         Modify_R_PID();
                printf("\r\n1. Roll_Modify \n\r\
2. Pitch_Modify \n\r\
3. Yaw_Modify \n\r");
         break;
         
       case Pitch_Modify://Pitch
          printf("P-PID\r\n");
          data = 0;
          mode = 0;
                printf("\r\n1. Roll_Modify \n\r\
2. Pitch_Modify \n\r\
3. Yaw_Modify \n\r");
         break;
         
       case Yaw_Modify://YAWW
          printf("Y-PID\r\n");
          data = 0;
          mode = 0;
                          printf("\r\n1. Roll_Modify \n\r\
2. Pitch_Modify \n\r\
3. Yaw_Modify \n\r");
         break;
       default://이외의 입력;
         break;
        }
  }
    
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSN_Pin|CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Modify_R_PID()
{
  printf("\r\n******************* R-PID ********************** X == ESC\r\n ");
  printf("Input R-P : ");
  data = '\0';//입력 버퍼 비우기
  input_flag=1;// 인터럽트  검출
  while(!(data=='x'))
  {
   // INPUT_KEYBOARD();
  }
  
  
}

void INPUT_KEYBOARD()
{
  static char i;//for문 제어용 변수


/*****엔터 입력****/
    if(data==0x0D)// 엔터 입력 받으면 저장
    {
      input_flag=0;
      C_buff[6]='\0';
      printf("\r\n save : %s\r\n",C_buff);
      
      for( i=0;i<C_count;i++)//문자열 저장
      {
        printf("[%d]%c ",i,C_buff[i]);
        C_buff[i]='\0';//
      }    
      C_count=0;//문자 카운터 초기화
     // data='x';//키보드 입력 탈출용
    }
    
    /*******백스페이스 입력*******/
    else if(data==0x7f)//백스페이스
    {
      data ='\0';
      --C_count;
      if(C_count==255)
        C_count=0;
      
      C_buff[C_count]='\0';
     // printf("\r\n[%d)%s",C_count,C_buff);
    }
    
    /****숫자 입력****/
    else if(data <= '9' && data >= '0' ||data == '.' )
    {
       printf("[%d] %c \r\n",C_count,C_buff[C_count]);
     if(C_count<6)// 6글자 이하만 입력 받음
        {
        C_buff[C_count]=data;
         printf("\r\n ok");
        ++C_count;
        }
        data ='\0';
    }


    else if(data=='x')
    {
      printf("\r\nbye\r\n");
      return ;
    }
    else
    {
    //    data = '\0';//입력 버퍼 비우기
    }
    
    if(C_count<6)// 6글자 이하만 입력 받음
        {
        C_buff[C_count]=data;
        ++C_count;
        }
        data ='\0';
  
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
  if(huart->Instance == huart2.Instance)
  {
    HAL_UART_Transmit(&huart2,&data, sizeof(data),10);
    INPUT_KEYBOARD();
     

  }
   HAL_UART_Receive_IT(&huart2,&data,1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
