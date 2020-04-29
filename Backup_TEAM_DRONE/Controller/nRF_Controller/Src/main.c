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
#include "stm32f4xx.h"
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"
#include "nRF_Transmit.h"
#include "key_Debug.h"
#include "JOYSTICK.h"
#include "i2c-lcd.h"
//#include "tm_stm32_usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Roll_Set_Point                       'r'
#define THROTTLE                            't'
#define Pitch_Set_Point                      'p'
#define Yaw_Set_Point                       'y'
    
#define _Pitch                                    0    
#define _Roll                                      1
#define _Yaw                                      3
#define _Throttle                                  2
#define EXTI_PIN_PC12                        4096                       // External Interrupt Pin 12
#define EXTI_PIN_PC10                        1024                       // External Interrupt Pin 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* My address */
uint8_t MyAddress[] = {
	0xE7,
	0xE7,
	0xE7,
	0xE7,
	0xE7
};

/* Receiver address */
uint8_t TxAddress[] = {
	0x7E,
	0x7E,
	0x7E,
	0x7E,
	0x7E
};

// nRF 통신 변수//
uint8_t dataIn[8];            //  TRANSMIT DATA BUFFER

//디버깅 모드 변수//
uint8_t key_input=0;
uint8_t key_tr=0;

// ADC  변수
int8_t ADC_DATA[4];   // ADC DATA BUFFER

// Interrupt variable
uint8_t button_flag=0;          // pc13 button flag
uint8_t button_flag2=0;        // pc14 button flag
uint8_t button_flag3=0;       //  pc15 button flag

// in pid값
float In_Roll_P=0, In_Roll_I=0, In_Roll_D=0;
float In_Pitch_P=0, In_Pitch_I=0, In_Pitch_D=0;
float In_Yaw_P=0, In_Yaw_I=0, In_Yaw_D=0;

// out pid값
float Out_Roll_P=0, Out_Roll_I=0, Out_Roll_D=0;
float Out_Pitch_P=0, Out_Pitch_I=0, Out_Pitch_D=0;
float Out_Yaw_P=0, Out_Yaw_I=0, Out_Yaw_D=0;

// throttle 변수
float Throttle=0;

//SettingPoint
int Set_Point[3];       // Roll_SetPoint, Pitch_SetPoint, Yaw_SetPoint


int count=1;


int fputc(int ch ,FILE *f)
{
 HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
 return ch;
}

void Display_UI()
{
   printf("\r\n  --------------Enter Debugging Mode---------------\r\n");
   printf("  ------------------------------------------------------\r\n");
   printf("     Roll             Pitch                Yaw\r\n");
   printf("  ------------------------------------------------------\r\n");
   printf(" [1].IN_R_P : %.4f     [2].IN_P_P : %.4f      [3].IN_Y_P : %.4f\r\n",In_Roll_P,In_Pitch_P,In_Yaw_P);
   printf(" [4].IN_R_I : %.4f     [5].IN_P_I : %.4f      [6].IN_Y_I : %.4f\r\n",In_Roll_I,In_Pitch_I,In_Yaw_I);
   printf(" [7].IN_R_D : %.4f     [8].IN_P_D : %.4f      [9].IN_Y_D : %.4f\r\n",In_Roll_D,In_Pitch_D,In_Yaw_D);
   printf("  ------------------------------------------------------\r\n");
   printf(" [A].OUT_R_P : %.4f     [B].OUT_P_P : %.4f      [C].OUT_Y_P : %.4f\r\n",Out_Roll_P, Out_Pitch_P, Out_Yaw_P);
   printf(" [D].OUT_R_I : %.4f     [E].OUT_P_I : %.4f      [F].OUT_Y_I : %.4f\r\n",Out_Roll_I, Out_Pitch_I, Out_Yaw_I);
   printf(" [G].OUT_R_D : %.4f     [H].OUT_P_D : %.4f      [I].OUT_Y_D : %.4f\r\n",Out_Roll_D, Out_Pitch_D, Out_Yaw_D);
   printf("  ------------------------------------------------------\r\n");
   
   printf(" [T].Throttle : %.0f                         [x].ESCAPE\r\n",Throttle);
   printf(" [R].Set_Roll : %d    [P].Set_Pitch : %d    [Y].Set_Yaw : %d\r\n",Set_Point[0],Set_Point[1],Set_Point[2]);

}

void PID_THROTTLE_Transmit(uint8_t key_input, uint8_t key_tr, float* data)
{
  key_tr = key_input;
  key_input='\0';
  printf("[%c]: ",key_tr);
  
  Keyboard_Debug(data,key_tr);
  
  nRF24_Transmit(data,key_tr);
  nRF24_Transmit_Status();
  key_tr = '\0';
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t Mode_Flag = Debug_Mode_Flag;   // START DEBUG MODE
  uint8_t test_buf[8];
  uint8_t exit_flag=0;
  uint8_t Lcd_buffer[15];

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1);
  HAL_ADC_Start_IT(&hadc1); 
  lcd_init();
  TM_NRF24L01_Init(120,8);
  TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k, TM_NRF24L01_OutputPower_0dBm);

  /* Set my address, 5 bytes */
  TM_NRF24L01_SetMyAddress(MyAddress);

  /* Set TX address, 5 bytes */
  TM_NRF24L01_SetTxAddress(TxAddress);

  /* Reset counter */
  TM_DELAY_SetTime(2001);
  
  
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
    /*========================DRIVE MODE===================*/
    
   if(button_flag == Drive_Mode_Flag)
   {
     
     if(Mode_Flag==Debug_Mode_Flag){
       Print_Mode(&Mode_Flag);
     }
     //Print_ADC_DEBUG();
     
     
     // ======== Roll Set Point Transmit ======== //
       printf("Roll : %d",ADC_DATA[1]);
       sprintf((char*)Lcd_buffer,"R:%3d",ADC_DATA[1]);
       lcd_put_cur(0,0);
       lcd_send_string((char*)Lcd_buffer);
       
       memset(Lcd_buffer, '\0', 15);
       HAL_Delay(1);
       nRF24_Transmit_ADC(&ADC_DATA[1],Roll_Set_Point);
       
     //======================================//
       
//      // ======== Roll Set Point Transmit ======== //
//       //printf("Roll : %d",ADC_DATA[1]);
//       sprintf((char*)Lcd_buffer,"R:%3d",ADC_DATA[1]);
//       lcd_put_cur(0,0);
//       lcd_send_string((char*)Lcd_buffer);
//       
//       memset(Lcd_buffer, '\0', 15);
//       nRF24_Transmit_ADC(&ADC_DATA[1],Roll_Set_Point);
//       TM_NRF24L01_PowerUpRx();
//             
//      // HAL_Delay(1000);
//       TM_NRF24L01_GetData(test_buf);
//       printf("R : %s",test_buf);
//       memset(test_buf,'\0',8);
//       
//     //======================================//
      
     
     // ======== Throttle Transmit ======== //
       printf("   Th : %d",ADC_DATA[2]);
       sprintf((char*)Lcd_buffer,"T:%3d",ADC_DATA[2]);
       lcd_put_cur(1,6);
       lcd_send_string((char*)Lcd_buffer);
       
       memset(Lcd_buffer, '\0', 15);
       HAL_Delay(1);
       nRF24_Transmit_ADC(&ADC_DATA[2],THROTTLE);                       
     //======================================//
     
     // ======== Pitch Set Point Transmit ======== //
       printf("   Pitch : %d",ADC_DATA[0]);
       sprintf((char*)Lcd_buffer,"P:%3d",ADC_DATA[0]);
       lcd_put_cur(0,6);
       lcd_send_string((char*)Lcd_buffer);
       
       memset(Lcd_buffer, '\0', 15);
       HAL_Delay(1);
       nRF24_Transmit_ADC(&ADC_DATA[0],Pitch_Set_Point);                 
     //======================================//
     
     // ======== Yaw Set Point Transmit ========== //
       printf("   Yaw :  %d",ADC_DATA[3]);
       sprintf((char*)Lcd_buffer,"Y:%3d",ADC_DATA[3]);
       lcd_put_cur(1,0);
       lcd_send_string((char*)Lcd_buffer);
       
       memset(Lcd_buffer, '\0', 15);
       HAL_Delay(1);
       nRF24_Transmit_ADC(&ADC_DATA[3],Yaw_Set_Point);                    
     //======================================//
     
     printf("\r\n");
     

    }
   
   /*========================DEBUG MODE===================*/
    
  else if(button_flag == Debug_Mode_Flag)
  {
    lcd_put_cur(0,0);
    lcd_send_string("DEBUG MODE");
    
    memset(Lcd_buffer, '\0', 15);
     if(Mode_Flag==Drive_Mode_Flag){
         Print_Mode(&Mode_Flag);    
     }
     lcd_clear();
     INPUT_CONTORLLER();
     INPUT_THROTTLE();
     HAL_Delay(100);
  }
   
   //==============KEYBOARD INPUT MODE==============//
   
   else if(button_flag == Key_Mode_Flag)
   { 
          lcd_put_cur(0,0);
          lcd_send_string("KEYBOARD INPUT");
     
          memset(Lcd_buffer, '\0', 15);
          if(Mode_Flag==Key_Mode_Flag){
             Print_Mode(&Mode_Flag);    
          }
          HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10);
          if(key_input == 'd')      //디버깅 모드로 진입
          {
            exit_flag=0;
            while(exit_flag ==0)
            {
             memset(test_buf,'\0',8);
             nRF24_Transmit_Mode_Change(key_input);
//             TM_NRF24L01_PowerUpRx();
//             
//             HAL_Delay(1000);
//             TM_NRF24L01_GetData(test_buf);
//             printf("Receive data : %s",test_buf);
             Display_UI();
             

             
            while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
            
            switch(key_input)
            {
                    
              case 't':
                 PID_THROTTLE_Transmit(key_input, key_tr, &Throttle);            // Throttle Input and Transmit
                 break;
                 
               case 'T':
                 PID_THROTTLE_Transmit(key_input, key_tr, &Throttle);            // Throttle Input and Transmit
                 break;
                 
             // ============================== IN PID DATA ============================== //          
               case '1':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_P);            //  IN_Roll_P Input and Transmit
                  break;                 
               
               case '2':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_P);           // IN_Pitch_P Input and Transmit
                  break;
                 
               case '3':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_P);             //  IN_Yaw_P Input and Transmit
                  break;
                 
               case '4':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_I);               //  IN_Roll_I Input and Transmit
                  break;
                 
               case '5':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_I);             //  IN_Pitch_I Input and Transmit
                  break;
                 
               case '6':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_I);              //  IN_Yaw_I Input and Transmit
                  break;
                 
               case '7':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Roll_D);             //  IN_Roll_D Input and Transmit
                  break;
                 
               case '8':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Pitch_D);           //  IN_Pitch_D Input and Transmit
                  break;
                 
               case '9':
                  PID_THROTTLE_Transmit(key_input, key_tr, &In_Yaw_D);             //  IN_Yaw_D Input and Transmit
                  break;
                  
              // ========================================================================= // 
                  
                  
              // ============================== OUT PID DATA ============================== // 
                  
              case 'A':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_P);            //  OUT_Roll_P Input and Transmit
                  break;
                  
              case 'a':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_P);            //  OUT_Roll_P Input and Transmit
                  break;
                  
               case 'B':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_P);           // OUT_Pitch_P Input and Transmit
                  break;
                  
               case 'b':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_P);           // OUT_Pitch_P Input and Transmit
                  break;   
                 
               case 'C':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_P);             //  OUT_Yaw_P Input and Transmit
                  break;
               
               case 'c':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_P);             //  OUT_Yaw_P Input and Transmit
                  break;
                  
                  
               case 'D':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_I);              //  OUT_Roll_I Input and Transmit
                  break;
                 
               case 'E':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_I);            //  OUT_Pitch_I Input and Transmit
                  break;
                  
               case 'e':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_I);            //  OUT_Pitch_I Input and Transmit
                  break;
                 
               case 'F':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_I);              //  OUT_Yaw_I Input and Transmit
                  break;
                  
                case 'f':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_I);              //  OUT_Yaw_I Input and Transmit
                  break;
                 
               case 'G':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_D);             //  OUT_Roll_D Input and Transmit
                  break;
               
               case 'g':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Roll_D);             //  OUT_Roll_D Input and Transmit
                  break;
                 
               case 'H':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_D);           //  OUT_Pitch_D Input and Transmit
                  break;
                  
               case 'h':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Pitch_D);           //  OUT_Pitch_D Input and Transmit
                  break;
                 
               case 'I':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_D);             //  OUT_Yaw_D Input and Transmit
                  break;

               case 'i':
                  PID_THROTTLE_Transmit(key_input, key_tr, &Out_Yaw_D);             //  OUT_Yaw_D Input and Transmit
                  break;
                  
               // ========================================================================= //
               
                case 'q':
                  key_tr = key_input;
                  key_input='\0';
                    
                   while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
                   {
                      if(count <= 1000)
                      {
                        nRF24_Transmit_ASCII(key_tr);
                        nRF24_Transmit_Status();
                        count++;
                      }
                    }
                    count=1;
                   key_tr='\0';
                   break;
                 
                // ============================== SET POINT DATA ============================== // 
               
               // Roll Set Point Input and Transmit  
                case 'r':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Roll_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[0]);
                  nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
                 case 'R':                                                                              
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Roll_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[0]);
                  nRF24_Transmit_Set_Point(&Set_Point[0],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
               // Pitch Set Point Input and Transmit
                case 'p':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Pitch_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[1]);
                  nRF24_Transmit_Set_Point(&Set_Point[1],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
                case 'P':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Pitch_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[1]);
                  nRF24_Transmit_Set_Point(&Set_Point[1],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
                // Yaw Set Point Input and Transmit 
                case 'y':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Yaw_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[2]);
                  nRF24_Transmit_Set_Point(&Set_Point[2],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
                case 'Y':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  printf("Yaw_SetPoint : ");
                  
                  Keyboard_Debug_Set_Point(&Set_Point[2]);
                  nRF24_Transmit_Set_Point(&Set_Point[2],key_tr);
                  nRF24_Transmit_Status();
                  
                  key_tr ='\0';
                  break;
                  
                // ========================================================================= //
                
                // Exit Keyboard Debug     
                case 'x':                                                                               
                  key_tr = key_input;
                  key_input = '\0';
                  
                  nRF24_Transmit_Mode_Change(key_tr);
                  //nRF24_Transmit_Status();
                  key_tr='\0';
                  Throttle=0;
                  button_flag=0;
                  exit_flag = 1;
                  lcd_clear();
                  break;
         
                 default:
                  printf("\r\nError Number\r\n");
                  break;
              }  // switch
            key_input = '\0';
            }   // while
  
          }     //   if     
        }       // else if
     else 
     {
       button_flag = Drive_Mode_Flag;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : button_Pin */
  GPIO_InitStruct.Pin = button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : button2_Pin button3_Pin */
  GPIO_InitStruct.Pin = button2_Pin|button3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNpin_Pin CEpin_Pin */
  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/*==================ADC CALLBACK===================*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   static uint8_t adc_count;
   int temp;
   if(adc_count == _Throttle) //   
   {
      ADC_DATA[adc_count] =(( HAL_ADC_GetValue(hadc)/41)-100)*(-1);
      //ADC_DATA[adc_count] =HAL_ADC_GetValue(hadc);
   }
   else if(adc_count == _Roll)
   {
     
     //temp = (( HAL_ADC_GetValue(hadc)/58)-35);                // 30degree
     //temp = (( HAL_ADC_GetValue(hadc)/102)-20);              // 20degree
     temp = (( HAL_ADC_GetValue(hadc)/455)-4);                     // 5degree
     if(temp <= 1 && temp >= -1)
     {
       temp=0;
     }
     
      ADC_DATA[adc_count] = temp;
   }
   
   else if(adc_count == _Yaw)
   {
     temp = (( HAL_ADC_GetValue(hadc)/58)-35)*(-1);
     if(temp < 2  && temp > -2)
     {
       temp=0;
     }
     
      ADC_DATA[adc_count] = temp;
      //ADC_DATA[adc_count] =HAL_ADC_GetValue(hadc);
   }
   
   else                                                                                 // Pitch
   {
     //temp = (( HAL_ADC_GetValue(hadc)/58)-35)*(-1);
     //temp = (( HAL_ADC_GetValue(hadc)/102)-20)*(-1);
     temp = (( HAL_ADC_GetValue(hadc)/455)-4)*(-1);
     if(temp <= 1 && temp >= -1)
     {
       temp=0;
     }
     
      ADC_DATA[adc_count] = temp;
      //ADC_DATA[adc_count] =HAL_ADC_GetValue(hadc);
   }
   adc_count++;

   if(adc_count >3)
   {
     adc_count=0;
   }

    HAL_ADC_Start_IT(&hadc1);
}
///*=================GPIO CALLBACK===================*/
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {
    static uint32_t temp;// debounce time
   if(GPIO_Pin==button_Pin)
   {     
     if((HAL_GetTick()-temp)>500)
     {
      // button_flag = button_flag;
       button_flag++;
       if(button_flag >= 3)
       {
         button_flag = 0;
       }
     }
   }
   
//   else if(GPIO_Pin == EXTI_PIN_PC10)
//   {
//     if((HAL_GetTick()-temp)>500)
//     {
//       button_flag2 = 1;
//     }
//   }
//   
//   else if(GPIO_Pin == EXTI_PIN_PC12)
//   {
//     if((HAL_GetTick()-temp)>500)
//     {
//       button_flag3 = 1;
//     }  
//   }
      while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)==GPIO_PIN_RESET);
      temp = HAL_GetTick();
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
