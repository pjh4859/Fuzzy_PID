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
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "tm_stm32_mpu9250.h"
#include "Quaternion.h"
#include "pid.h"
#include "PWM.h"
#include "Filters.h"
#include "fuzzy.h"
#include "STM32F4_FLASH_MEMORY.h"
#include "nRF24_Receive.h"
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI                      (3.141592f)                                     //the ratio of the circumference of a circle to its diameter.
#define dt                      (2.0f)                                          //Least dt milliseconds (>1/dt mHz)Update term (milliseconds).
#define init_angle_average      (40)                                            //Initiate_Setting_angle.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void UART1_TX_string(char* str);
void UART2_TX_string(char* str);
void uart_recv_val(uint8_t* arr);
//void STM32f4_USART2_Init(void);
//void System_information(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//==============================GLOBAL VARIABLES================================
//==============================================================================
//==============================UART variables==================================
uint8_t Tx_buffer[50];                                                          //Transmit buffer.
        
uint8_t uart1_rx_irq_buffer[8];                                                 //Receive buffer.
uint8_t uart1_tx_to_MFC[16];                                                    //Transmit buffer.
uint8_t uart2_tx_data[255];
uint8_t data;
int count = 0;

//int num = 0;
//ADD YSH
//uint8_t TIM_INT =0;s

volatile __IO uint8_t  DMA_Rx_Flag = 0;
volatile __IO uint8_t  DMA_Tx_Flag = 0;
//==============================INIT Variables==================================
//=============================MPU9250 variables================================
//==============================================================================
//=======================nRF24L01 GLOBAL VARIABLES==============================
//uint8_t TxAddress[] = {                                                       // Controller
//  0xE7,
//  0xE7,
//  0xE7,
//  0xE7,
//  0xE7
//};
uint8_t MyAddress[] = {                                                         // Controller 
  0x7E,
  0x7E,
  0x7E,
  0x7E,
  0x7E
};

//int value=0;                                             

//==============================================================================

//================================printf FUNCTION===============================
int fputc(int ch ,FILE *f)
{
  //UART2_TX_string((char *)ch);
  //HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF);
  return ch;
}
//==============================================================================

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //==============================INIT Variables================================
  TM_MPU9250_t    MPU9250;                                                      //MPU9250 Sensor structure.
  __PID           pid;                                                          //PID Controll structure.
  //========================Flags and Time_flags================================    
  uint32_t Now = 0;                                                             //Used to calculate integration interval.
  uint32_t lastUpdate = 0;                                                      //Used to calculate integration interval.
  uint32_t before_while = 0;                                                    //Time of Before entering while loop.
  uint16_t wait = 0;                                                            //getting initiate setting_angle waiting time.
  uint8_t wait_flag = 0;                                                        //Time waiting flag.
 
  uint32_t UART_Now = 0;     
  uint32_t UART_Pre = 0;
  uint32_t UART_deltat = 0;
  
  uint8_t UART_flag=0;
  uint8_t UART_sytic_flag=0;
  //=============================UART Variables=================================
  //========================Quaternion VARIABLES================================
  float deltat = 0.0f;                                                          //integration interval for filter schemes.
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                                        //vector to hold quaternion.
  float Euler_angle[3] = {0.0f, 0.0f, 0.0f};                                    //roll pitch yaw.  
  //=============================Fuzzy Variables================================  
  float prev_err[3];                                                            //Prev_Setting_point - Euler_angle.
  //==============================PWM Variables=================================
  int Controller_1 = 15;                                                        //Moter Throttle.
  //==============================FILTER's Variables============================
  float preEuler_angle[3] = {0.0f, 0.0f, 0.0f};                                 //Used in LPF.
  float LPF_Euler_angle[3] = {0.0f, 0.0f, 0.0f};                                //Used in LPF.
  float preGyro[3] = {0.0f, 0.0f, 0.0f};                                        //Used in GyroLPF.
  float LPF_Gyro[3] = {0.0f, 0.0f, 0.0f};                                       //Used in GyroLPF.
  //=============================MPU9250 Variables==============================
  float Self_Test[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};                    //MPU9250 Accell and Gyro Self_Test.
  float Self_Test_Mag[3] = {0.0f, 0.0f, 0.0f};                                  //MPU9250 Magnetometer Self_Test.
  //========================Drone Calibration Mode Variables====================
  uint8_t CaliFlag = 0;
  //============================nRF24L01 VARIABLES==============================
  int temp_int;                                                                 //uint8_t
  float temp;                                                                   //uint8_t
  //====================Hanging Variables from external controll================
  float setting_angle[3] = {0.0f, 0.0f, 0.0f};                                  //roll pitch yaw.
  float init_setting_angle[3] = {0.0f, 0.0f, 0.0f};
  float pid_val[3][3] = {{1.32f, 0.3f, 0.0f}, {1.32f, 0.3f, 0.0f}, {3.0f, 0.0f, 0.0f}};            //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
  float inpid_val[3][3] = {{12.0f, 1.0f, 1.7f}, {12.0f, 1.0f, 1.7f}, {5.0f, 0.0f, 0.5f}};          //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
  float angular_velocity[3];                                                    //For double loop PID.
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __INIT__MPU9250(&MPU9250);                                                    //Init MPU9250 variables.
  MPU9250SelfTest(&MPU9250, &Self_Test[0],TM_MPU9250_Device_0);                 //Selftest MPU9250.
  //calibrateMPU9250(&MPU9250);                                                 //Calibrate MPU9250 Accelometer and Gyroscope.
  TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);                               //Init MPU9250 and setting.
  TM_MPU9250_ReadMagASA(&MPU9250);                                              //Get MPU9250 Magnetic ASA data.
  pid_init(&pid, pid_val, inpid_val);                                           //Init pid values.
  fuzzy_init();                                                                 //Init fuzzy values.
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */  
//System_information();
//STM32f4_USART2_Init();
  //=============================Calibration Part===============================
  if(CaliFlag == 1)    //1 is unable.
  {    
    HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    
    MPU9250SelfTest(&MPU9250, &Self_Test[0],TM_MPU9250_Device_0);  
    //calibrateMPU9250(&MPU9250);   
    Delayms(500);     
    TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
    TM_MPU9250_ReadMagASA(&MPU9250);                                            //Get MPU9250 Magnetic ASA data.
    AK8963SelfTest(&MPU9250, &Self_Test_Mag[0]);
    MagCalibration(&MPU9250);
    //while(1){}                                                                //Unlimited loop.
  }
  //==========================Calibration Part END==============================  
  //==============================PWM START=====================================
  if (1)
  {
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  }
  
    //ESC_Calibration();  
    Motor_Init();  
    //Motor_Start(); 
  
  TM_NRF24L01_Init(120,8);  
  TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_250k, TM_NRF24L01_OutputPower_0dBm);
  TM_NRF24L01_SetMyAddress(MyAddress);
  //======================Get Biases From Flash Memory==========================
  if(1){
    Get_biases(&MPU9250);
  }
  //=====================Get Biases From Flash Memory END=======================  
  before_while = HAL_GetTick();                                                 //Get time of before while loop.
  lastUpdate = before_while;                                                    //First time of lastUpdate using for gain the deltat.  
  
//  LL_USART_EnableIT_RXNE(USART1);
//  LL_USART_EnableIT_RXNE(USART2);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {        
    //int aa = HAL_GetTick();     //Get time.    
  //============================Get MPU9250 data================================
    TM_MPU9250_ReadAcce(&MPU9250);                                              //get Accel data.
    TM_MPU9250_ReadGyro(&MPU9250);                                              //get Gyro data.
    TM_MPU9250_ReadMag(&MPU9250);                                               //get Magnetic data.
  //==========================Get MPU9250 data END==============================
  //===Subtract Automatic Accelometer Gyroscope and Magnetic filed bias===

     MPU9250.Ax -= MPU9250.Accbiasx;                                            //callibrate Accel values.
     MPU9250.Ay -= MPU9250.Accbiasy;                            
     MPU9250.Az -= MPU9250.Accbiasz;                            
        
     MPU9250.Gx -= MPU9250.Gybiasx;                                             //callibrate Gyro values.
     MPU9250.Gy -= MPU9250.Gybiasy;                             
     MPU9250.Gz -= MPU9250.Gybiasz;                             
            
     MPU9250.Mx -= MPU9250.Magbiasx;                                            //callibrate Magnetic values.
     MPU9250.My -= MPU9250.Magbiasy;                            
     MPU9250.Mz -= MPU9250.Magbiasz;                            
            
     MPU9250.Mx *= MPU9250.Magscalex;                                           //callibrate Magnetic values.
     MPU9250.My *= MPU9250.Magscaley;                           
     MPU9250.Mz *= MPU9250.Magscalez;

  //====Subtract Automatic Accelometer Gyroscope and Magnetic filed bias END====
  //===========================Init settiing angle==============================
    if (wait_flag < init_angle_average)
    {
      wait = HAL_GetTick() - before_while;
      if (wait >= 5000)
      {
        init_setting_angle[0] += Euler_angle[0];                                //roll.
        init_setting_angle[1] += Euler_angle[1];                                //pitch.
        init_setting_angle[2] += Euler_angle[2];                                //yaw.
        wait_flag ++;
      }
    }
    if (wait_flag == init_angle_average)
    {
//      setting_angle[0] = init_setting_angle[0] / init_angle_average;          //init roll.
//      setting_angle[1] = init_setting_angle[1] / init_angle_average;          //init pitch.
      setting_angle[2] = init_setting_angle[2] / init_angle_average;            //init yaw.
      wait_flag ++;
    }
  //============================Init settiing angle END=========================
  //============================Get delta_t=====================================
    Now = HAL_GetTick();                                                        //Get current time.
    deltat += (Now - lastUpdate);                                               //Set integration time by time elapsed since+ last filter update (milliseconds).
    lastUpdate = Now;                                                           //Update lastupdate time to current time.
  //============================Get delta T END=================================
  //============================================================================
    angular_velocity[0] = MPU9250.Gx / 500.0f * dt;                             //angular velocity (degree/2ms(*2))
    angular_velocity[1] = MPU9250.Gy / 500.0f * dt;
    angular_velocity[2] = MPU9250.Gz / 500.0f * dt;    
    
    if (deltat >= dt)                                                           //Update term (500Hz.dt=2).
    {
      deltat /= 1000.0f;                                                        //Make millisecond to second.
      //__LPFGyro(LPF_Gyro, &MPU9250, preGyro, deltat);
      MahonyQuaternionUpdate(MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Gx*PI/180.0f, MPU9250.Gy*PI/180.0f, MPU9250.Gz*PI/180.0f, MPU9250.My, MPU9250.Mx, -MPU9250.Mz, q, deltat);
      //MahonyQuaternionUpdate(MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Gx*PI/180.0f, MPU9250.Gy*PI/180.0f, MPU9250.Gz*PI/180.0f, MPU9250.My, MPU9250.Mx, -MPU9250.Mz, q, deltat);
      Quternion2Euler(q, Euler_angle);                                          //Get Euler angles (roll, pitch, yaw) from Quaternions.
      //__LPF(LPF_Euler_angle, Euler_angle, preEuler_angle, deltat);
  //===================================Fuzzy part===============================
      Fuzzification(setting_angle[0], Euler_angle[0], &prev_err[0]);            //Fuzzy roll part.
      Create_Fuzzy_Matrix(0);
      Defuzzification(&inpid_val[0][0],&inpid_val[0][1],&inpid_val[0][2], 0);   //Fuzzy roll end.
      
      Fuzzification(setting_angle[1], Euler_angle[1], &prev_err[1]);            //Fuyzzy pitch part.
      Create_Fuzzy_Matrix(1);
      Defuzzification(&inpid_val[1][0],&inpid_val[1][1],&inpid_val[1][2], 1);   //Fuzzy pitch end.
      
      Fuzzification(setting_angle[2], Euler_angle[2], &prev_err[2]);            //Fuzzy yaw part.
      Create_Fuzzy_Matrix(2);
      Defuzzification(&inpid_val[2][0],&inpid_val[2][1],&inpid_val[2][2], 2);   //Fuzzy yaw end.     
      pid_gain_update(&pid, pid_val, inpid_val);                                //From Fuzzy the PID gain value is changed.
  //================================Fuzzy part END==============================
      if (HAL_GetTick() - before_while >= 5500)
      {
        __pid_update(&pid, setting_angle, Euler_angle, angular_velocity, deltat);         //PID value update.       
        //__pid_update(&pid, setting_angle, LPF_Euler_angle, angular_velocity, deltat);   //PID value update.
      }    
      //if(Euler_angle[2] < 0) Euler_angle[2] += 360.0f;                        // Ensure yaw stays between 0 and 360     
      deltat = 0.0f;                                                            //reset deltat.
    }
/*-----------------------------------------------------------------------------------------------*/
//============================Data print transmit UART part=====================        
    //sprintf((char*)uart2_tx_data,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f \r\n",  \
      MPU9250.Ax, MPU9250.Ay ,MPU9250.Az, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Mx, MPU9250.My, MPU9250.Mz);    
    //sprintf((char*)uart2_tx_data,"%10.4f %10.4f %10.4f \r\n", MPU9250.Mx, MPU9250.My, MPU9250.Mz);
    //sprintf((char*)uart2_tx_data,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f \r\n",  \
      MPU9250.Ax, MPU9250.Ay ,MPU9250.Az, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, LPF_Gyro[0],LPF_Gyro[1], LPF_Gyro[2]);
    //sprintf((char*)uart2_tx_data,"%f  %f  %f  %f  %f  %f\r\n", Self_Test[0], Self_Test[1], Self_Test[2], Self_Test[3], Self_Test[4], Self_Test[5]);
    //sprintf((char*)uart2_tx_data,"%f  %f  %f\r\n", Self_Test_Mag[0], Self_Test_Mag[1], Self_Test_Mag[2]);
    //sprintf((char*)uart2_tx_data,"%f %f %f %f %f %f %f %f %f\r\n", MPU9250.Accbiasx, MPU9250.Accbiasy, MPU9250.Accbiasz, MPU9250.Gybiasx, MPU9250.Gybiasy, MPU9250.Gybiasz, MPU9250.Magbiasx, MPU9250.Magbiasy, MPU9250.Magbiasz);
    //sprintf((char*)uart2_tx_data,"%f %f %f %f %f %f\r\n", MPU9250.Magbiasx, MPU9250.Magbiasy, MPU9250.Magbiasz, MPU9250.Magscalex, MPU9250.Magscaley, MPU9250.Magscalez);
    //sprintf((char*)uart2_tx_data,"%10.4f %10.4f %10.4f %10.4f\r\n", q[0], q[1], q[2], q[3]);   
    //sprintf((char*)uart2_tx_data,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\r\n", pid.iKp[0], pid.iKi[0], pid.iKd[0], pid.iKp[1], pid.iKi[1], pid.iKd[1],pid.iKp[2], pid.iKi[2], pid.iKd[2]);   

    //sprintf((char*)uart2_tx_data,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);
    //sprintf((char*)uart2_tx_data,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);   
    //sprintf((char*)uart2_tx_data,"%4d %4d %4d\r\n", (int)LPF_Euler_angle[0], (int)LPF_Euler_angle[1], (int)LPF_Euler_angle[2]);
    //sprintf((char*)uart2_tx_data,"%4d %4d %4d\r\n", (int)Euler_angle[0], (int)Euler_angle[1], (int)Euler_angle[2]);
    sprintf((char*)uart2_tx_data,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[2]);   
    //sprintf((char*)uart2_tx_data,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2]);   
    //sprintf((char*)uart2_tx_data,"%4d,%4d,%4d,\r\n",  (int)Euler_angle[0], (int)Euler_angle[1], (int)Euler_angle[2]);   

    //UART2_TX_string((char *)uart2_tx_data);
    
 //=========================Data print transmit UART part END===================
    
 //===========================BLDC Motor Part===================================
   
    if(HAL_GetTick() - before_while >= 5000 && HAL_GetTick() - before_while < 6000)
    {
       Motor_Start();
    }    
    else if (HAL_GetTick() - before_while >= 6000)
    { 
      //Motor_Drive(Controller_1, pid.output);

      if (Controller_1 <= 5)
      {
        MOTOR_V1 = MIN_PULSE;
        MOTOR_V2 = MIN_PULSE;
        MOTOR_V3 = MIN_PULSE;
        MOTOR_V4 = MIN_PULSE;
      }
      
      else if (Controller_1 > 5)    //Controller_1
      {     
        if (fabs(Euler_angle[0]) > 15.0f || fabs(Euler_angle[1]) > 15.0f)       //Restrict yaw acting Euler angle.
        {           
          pid.output[2] = 0.0f;
        }
        
        MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) + (int)(0.7 * (MoterGain_roll) * pid.output[0]) - (int)(0.7 * (MoterGain_pitch) * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) + (int)(0.7 * (MoterGain_roll) * pid.output[0]) - (int)(0.7 * (MoterGain_pitch) * pid.output[1]);
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) - (int)(MoterGain_pitch * pid.output[1]);
        //MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_roll * pid.output[0]);
       // MOTOR_V1 = MIN_PULSE + (Controller_1 * 70);
        if (MOTOR_V1 >= MAX_PULSE)// - MOTER_SAFTY)
          MOTOR_V1 = MAX_PULSE;// -  MOTER_SAFTY;
        else if (MOTOR_V1 <= MIN_PULSE + 700)
          MOTOR_V1 = MIN_PULSE + 700;
        
        MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)(0.7 * (MoterGain_roll) * pid.output[0]) - (int)(0.7 * (MoterGain_pitch) * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)(0.7 * (MoterGain_roll) * pid.output[0]) - (int)(0.7 * (MoterGain_pitch) * pid.output[1]);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)((MoterGain_pitch) * pid.output[1]);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)(MoterGain_roll * pid.output[0]);
        //MOTOR_V2 = MIN_PULSE + (Controller_1 * 70);
        if (MOTOR_V2 >= MAX_PULSE)// - MOTER_SAFTY)
          MOTOR_V2 = MAX_PULSE;// - MOTER_SAFTY;
        else if (MOTOR_V2 <= MIN_PULSE + 700)
          MOTOR_V2 = MIN_PULSE + 700;
        
        MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) + (int)(0.7 * (MoterGain_roll) * pid.output[0]) + (int)(0.7 * (MoterGain_pitch) * pid.output[1]) - (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) - (int)(MoterGain_yaw * pid.output[2]);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) + (int)(0.7 * (MoterGain_roll) * pid.output[0]) + (int)(0.7 * (MoterGain_pitch) * pid.output[1]);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_pitch * pid.output[1]);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_roll * pid.output[0]);
        //MOTOR_V3 = MIN_PULSE + (Controller_1 * 70);
        if (MOTOR_V3 >= MAX_PULSE)// - MOTER_SAFTY)
          MOTOR_V3 = MAX_PULSE;// - MOTER_SAFTY;
        else if (MOTOR_V3 <= MIN_PULSE + 700)
          MOTOR_V3 = MIN_PULSE + 700;
        
        MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) - (int)(0.7 * (MoterGain_roll) * pid.output[0]) + (int)(0.7 * (MoterGain_pitch) * pid.output[1]) + (int)(MoterGain_yaw * pid.output[2]); 
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_yaw * pid.output[2]); 
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) - (int)(0.7 * (MoterGain_roll) * pid.output[0]) + (int)(0.7 * (MoterGain_pitch) * pid.output[1]); 
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) + (int)((MoterGain_pitch) * pid.output[1]); 
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) - (int)(MoterGain_roll * pid.output[0]);
        //MOTOR_V4 = MIN_PULSE + (Controller_1 * 70);
        if (MOTOR_V4 >= MAX_PULSE)// - MOTER_SAFTY)
          MOTOR_V4 = MAX_PULSE;// - MOTER_SAFTY;
        else if (MOTOR_V4 <= MIN_PULSE + 700)
          MOTOR_V4 = MIN_PULSE + 700;
       }
    }    
//========================BLDC Motor Part END===================================
    
//========================NRF24L01 Receive Part=================================    
  NRF24_Receive(&Controller_1,temp,temp_int,&pid,setting_angle,Euler_angle[2]);          
//=======================NRF24L01 Receive Part END==============================  
    
//==========================Data transmit part==================================
//==========================Euler_angle_chart_part==============================  
    sprintf((char*)uart1_tx_to_MFC,"%d,%d,%d,",  (int)Euler_angle[0], (int)Euler_angle[1], (int)Euler_angle[2]);   
    //UART1_TX_string((char *)uart1_tx_to_MFC);
//===========================outPID inPID change part===========================  
//     if(count == 8)
//    {
//      count = 0;
//      uart_recv_val(uart1_rx_irq_buffer); 
//    }     
//============================Data transmit part END============================    
//================================TIme Check====================================
//================================TIme Check END================================
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 16000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 16000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 9-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 16000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  /**USART1 GPIO Configuration  
  PA10   ------> USART1_RX
  PB6   ------> USART1_TX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_RX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_2, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_2, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_2, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_2, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_2, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_2, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_2);

  /* USART1_TX Init */
  LL_DMA_SetChannelSelection(DMA2, LL_DMA_STREAM_7, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA2, LL_DMA_STREAM_7, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA2, LL_DMA_STREAM_7, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA2, LL_DMA_STREAM_7, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA2, LL_DMA_STREAM_7, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA2, LL_DMA_STREAM_7, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA2, LL_DMA_STREAM_7);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration  
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2|LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */
  
  /* USART2_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* USART2_TX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_6, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_6, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_6, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_6, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_6, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_6);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream6_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  NVIC_SetPriority(DMA2_Stream7_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CEpin_Pin|CSNpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CEpin_Pin CSNpin_Pin */
  GPIO_InitStruct.Pin = CEpin_Pin|CSNpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void USART_Rx_Callback(USART_TypeDef *USARTx){
//  data = (USARTx->DR & 0x1ff);
//  uart1_rx_irq_buffer[count++] = data;
  uart1_rx_irq_buffer[count++] = (USARTx->DR & 0x1ff);;
//  data = LL_USART_ReceiveData8(USARTx);
}

void UART1_TX_string(char* str){
  while(*str){
    while(!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, *str);
    str++;
  }
}

void UART2_TX_string(char* str){
  while(*str){
    while(!LL_USART_IsActiveFlag_TXE(USART2));
    LL_USART_TransmitData8(USART2, *str);
    str++;
  }
}

void uart_recv_val(uint8_t* arr)
{
    char pid_buf[8]={0,};
    char debuging_buf[3] = "\r\n";
    memset(pid_buf,'\0',sizeof(pid_buf));
    UART2_TX_string(debuging_buf);
                          
    switch(arr[6])
    {
      case '1' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[0][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '2' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[0][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '3' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[0][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
        case '4' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[1][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '5' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[1][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '6' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[1][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
        case '7' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[2][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '8' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[2][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case '9' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //inpid_val[2][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
        
      case 'a' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[0][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case 'b' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[0][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case 'c' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[0][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
        case 'd' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[1][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case 'e' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[1][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case 'f' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[1][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      case 'g' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[2][0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
     case 'h' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[2][1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
     case 'i' :
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //pid_val[2][2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
    case 'T':
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //*Controller_1 = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
    case 'R':
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //setting_angle[0] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
    case 'P':
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //setting_angle[1] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
    case 'Y':
        memcpy(pid_buf,arr,6);
        pid_buf[6] = '\0';
        //setting_angle[2] = atof(pid_buf);
        UART2_TX_string(pid_buf);
        break;
      
      default: break;
  }
}

//void STM32f4_USART2_Init(void)
//{
//	UartHandle.Instance        = USART2;
//
//	UartHandle.Init.BaudRate     = 115200;
//	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
//	UartHandle.Init.StopBits     = UART_STOPBITS_1;
//	UartHandle.Init.Parity       = UART_PARITY_NONE;
//	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
//	UartHandle.Init.Mode         = UART_MODE_TX_RX;
//	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
//
//	HAL_UART_MspInit(&UartHandle);
//
//	if(HAL_UART_DeInit(&UartHandle) != HAL_OK)
//	{
//		Error_Handler();
//	}  
//	if(HAL_UART_Init(&UartHandle) != HAL_OK)
//	{
//		Error_Handler();
//	}
//
//	//if(HAL_UART_Transmit(&UartHandle, (uint8_t*)aTxBufferUart, sizeof(aTxBufferUart), 5000)!= HAL_OK)
//	//{
//	//	Error_Handler();   
//	//}
//
//	Dprintf("STM32f4_USART2_Init \n\r");
//
//}

//void System_information(void)
//{
//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//#if 0
//	char buff[200];
//
//	Dprintf("DEVICE ID:%ld[0x%x], DEVICE REVISION ID:%ld[0x%x], flash size:0x%x\n\r", HAL_GetDEVID(), HAL_GetDEVID(), HAL_GetREVID(), HAL_GetREVID(), ID_GetFlashSize());
//
//	/* Format unique ID in 32-bit read mode */
//	sprintf(buff, "Unique ID in 32-bit read mode: 0x%08X 0x%08X 0x%08X\n\r",
//		ID_GetUnique32(0),	/* LSB */
//		ID_GetUnique32(1),
//		ID_GetUnique32(2)	/* MSB */
//		);
//	Dprintf("%s",buff);
//	/* Format unique ID in 16-bit read mode */
//	sprintf(buff, "Unique ID in 16-bit read mode: 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X 0x%04X\n\r",
//		ID_GetUnique16(0),	/* LSB */
//		ID_GetUnique16(1),
//		ID_GetUnique16(2),
//		ID_GetUnique16(3),
//		ID_GetUnique16(4),
//		ID_GetUnique16(5)	/* MSB */
//		);
//	Dprintf("%s",buff);
//	/* Format unique ID in 8-bit read mode */
//	sprintf(buff, "Unique ID in 8-bit read mode: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n\r",
//		ID_GetUnique8(0),	/* LSB */
//		ID_GetUnique8(1),
//		ID_GetUnique8(2),
//		ID_GetUnique8(3),
//		ID_GetUnique8(4),
//		ID_GetUnique8(5),
//		ID_GetUnique8(6),
//		ID_GetUnique8(7),
//		ID_GetUnique8(8),
//		ID_GetUnique8(9),
//		ID_GetUnique8(10),
//		ID_GetUnique8(11)	/* MSB */
//		);
//	Dprintf("%s",buff);
//#endif
//
//	/* Get the Oscillators configuration according to the internal RCC registers */
//	HAL_RCC_GetOscConfig(&RCC_OscInitStruct);
//
//	//Dprintf("StartUpCounter:%d", StartUpCounter);
//	/* Get frequency value */
//	Dprintf("SYSCLK:%d, HCLK:%d, PCLK1:%d, PCLK2:%d\n\r",
//			HAL_RCC_GetSysClockFreq(), HAL_RCC_GetHCLKFreq(), HAL_RCC_GetPCLK1Freq(), HAL_RCC_GetPCLK2Freq());
//
//}
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
