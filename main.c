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
#include <string.h>
//#include <stdio.h>
#include <math.h>
#include "tm_stm32_mpu9250.h"
#include "Quaternion.h"
#include "pid.h"
#include "PWM.h"
#include "fuzzy.h"
#include "LPF.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI                      (3.141592f)             //the ratio of the circumference of a circle to its diameter.
#define dt                      (2.0f)                  //Least dt milliseconds (>1/dt mHz)Update term (milliseconds).
#define init_angle_average      (40)                    //Initiate_Setting_angle.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//========================GLOBAL VARIABLES=========================
//========================USER VARIABLES===========================
uint8_t uart1_tx_to_MFC[100];                             //Trandmit variable.
uint8_t uart1_tx_to_MFC2[100];                           //Trandmit variable.
uint8_t uart2_tx_data2[255];
//uint8_t uart2_tx_data3[255];
//======================UART variables============================
uint8_t pid_buffer[70];
int num = 0;
uint8_t data;

//=======================INIT Variables=============================
TM_MPU9250_t    MPU9250;                                //MPU9250 Sensor structure.
__PID           pid;                                    //PID Controll structure.

//=======================MPU9250 variables==========================
float ax, ay, az, gx, gy, gz, mx, my, mz;               // variables to hold latest sensor data values.

//=======================Changing Variable from external controll========
float setting_angle[3] = {0.0f, 0.0f, 0.0f};            //roll pitch yaw.
//float init_setting_angle[3] = {0.0f, 0.0f, 0.0f};
float pid_val[3][3] = {{4.0f, 0.00f, 0.0f}, {3.5f, 0.00f, 0.0f}, {3.5f, 0.00f, 0.0f}};       //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
float inpid_val[3][3] = {{1.6f, 0.9f, 0.6f}, {2.0f, 1.1f, 0.66f}, {2.0f, 1.1f, 0.66f}};        //P I D gain controll (Roll PID, Pitch PID, Yaw PID sequences).
//float angular_velocity[3];                              //For double loop PID.
//float Magbias[3] = {0.0f, 0.0f, 0.0f};                  //Magnetic data bias.

int Controller_1 = 30;                                  //Moter Throttle.(40이면 뜰듯)
int Controller_2 = 0;                                   //Moter Throttle. 
//====================Quaternion VARIABLES===============================
float Euler_angle[3] = {0.0f, 0.0f, 0.0f};              //roll pitch yaw.
float delta_angle[3] = {0.0f, 0.0f, 0.0f};              //d_roll d_pitch d_yaw.
float preEuler_angle[3] = {0.0f, 0.0f, 0.0f};           //Used in LPF.
float LPF_Euler_angle[3] = {0.0f, 0.0f, 0.0f};           //Used in LPF.
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};                  // vector to hold quaternion.
//float eInt[3] = {0.0f, 0.0f, 0.0f};                     // vector to hold integral error for Mahony method.
float deltat = 0.0f;                                    //integration interval for filter schemes.

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  float angular_velocity[3];                              //For double loop PID.
  float Magbias[3] = {0.0f, 0.0f, 0.0f};                  //Magnetic data bias.
  
  //=====================Flag and Time flag=================================    
uint32_t Now = 0;                                       //Used to calculate integration interval.
uint32_t lastUpdate = 0;                                //Used to calculate integration interval.
uint32_t before_while = 0;                              //Time of Before entering while loop.
uint16_t wait = 0;                                      //getting initiate setting_angle waiting time(3secs) 
uint8_t wait_flag = 0;                                  //Time waiting flag.
uint8_t Mcal_flag = 0;                                  //User Magnetic bias flag.
//====================Fuzzy Variables====================================
//float prev_err[3];                                      //Prev_Setting_point - Euler_angle.

//uint32_t UART_Now = 0;     
//uint32_t UART_lastUpdate = 0;
//uint32_t UART_deltat = 0;

//float dt2 = 0.0f;                                       //임시 테스트용.
//========================================================================

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  TM_MPU9250_Init(&MPU9250, TM_MPU9250_Device_0);
  pid_init(&pid, pid_val, inpid_val);
  fuzzy_init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  TM_MPU9250_ReadMagASA(&MPU9250);      //Get MPU9250 Magnetic ASA data.
  //=========Automatic calculation of Magnetic filed bias=======
  if (Mcal_flag == 1)
  { 
    magcal(Magbias);
    Magbias[0] = Magbias[0] * MPU9250.MMult * MPU9250.ASAX;
    Magbias[1] = Magbias[1] * MPU9250.MMult * MPU9250.ASAY;
    Magbias[2] = Magbias[2] * MPU9250.MMult * MPU9250.ASAZ;
    Mcal_flag ++;
  }    
  //======Automatic calculation of Magnetic filed bias END======
  //================PWM START===================================
  if (0)
  {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  }
  
 
  
//  ESC_Calibration();  
  Motor_Init();  
//  Motor_Start();
  
//*********************************************
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);          //인터럽프 활성화.
  memset(pid_buffer,'\0',sizeof(pid_buffer));           //버퍼비우기
//********************************************
  

  before_while = HAL_GetTick(); //Get time of before while loop.
  lastUpdate = before_while;    //First time of lastUpdate using for gain the deltat.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {         
    //int aa = HAL_GetTick();     //Get time for 
    
    HAL_UART_Receive_IT(&huart1, &data, 1);             //PID PPID 받을 때.

    TM_MPU9250_ReadAcce(&MPU9250);      //get Accel data.
    TM_MPU9250_ReadGyro(&MPU9250);      //get Gyro data.
    TM_MPU9250_ReadMag(&MPU9250);       //get Magnetic data.
    
    MPU9250.Gx -= 2.55f;                //callibration values.
    MPU9250.Gy -= 0.15f;
    MPU9250.Gz -= 0.24f;
    
    MPU9250.Mx -= 67.5f;                //callibration values.
    MPU9250.My -= 49.0f;
    MPU9250.Mz -= -24.5f;    
    
    //MPU9250.Mx -= 25.;                  //callibration values.(SH's)
    //MPU9250.My -= 13.5;
    //MPU9250.Mz -= -32.;    
  //=========Subtract Automatic Magnetic filed bias============
    if (Mcal_flag == 2)
    {
       MPU9250.Mx -= Magbias[0];        //Automatic User callibration values.
       MPU9250.My -= Magbias[1];
       MPU9250.Mz -= Magbias[2];    
    }
  //=========Subtract Automatic Magnetic filed bias END========

    Now = HAL_GetTick();                //Get current time.
    deltat += (Now - lastUpdate);       //Set integration time by time elapsed since last filter update (milliseconds).
    lastUpdate = Now;                   //Update lastupdate time to current time.
  //---------------must do functionization!!!----------    
    if (wait_flag < init_angle_average)
    {
      wait = HAL_GetTick() - before_while;
      if (wait >= 3500)
      {            
//        init_setting_angle[0] += Euler_angle[0];         //roll
//        init_setting_angle[1] += Euler_angle[1];         //pitch
//        init_setting_angle[2] += Euler_angle[2];         //yaw
        setting_angle[0] += Euler_angle[0];         //roll
        setting_angle[1] += Euler_angle[1];         //pitch
        setting_angle[2] += Euler_angle[2];         //yaw
        wait_flag ++;
      }
    }    
    if (wait_flag == init_angle_average)
    {
      //setting_angle[0] = init_setting_angle[0] / init_angle_average;    //init roll
      //setting_angle[1] = init_setting_angle[1] / init_angle_average;    //init pitch
      //setting_angle[2] = init_setting_angle[2] / init_angle_average;    //init yaw
      setting_angle[0] = setting_angle[0] / init_angle_average;    //init roll
      setting_angle[1] = setting_angle[1] / init_angle_average;    //init pitch
      setting_angle[2] = setting_angle[2] / init_angle_average;    //init yaw
    }
  //---------------must do functionization!!!-------------------
    
    angular_velocity[0] = MPU9250.Gx / 1000.0f * dt;           
    angular_velocity[1] = MPU9250.Gy / 1000.0f * dt;
    angular_velocity[2] = MPU9250.Gz / 1000.0f * dt;
    
    if (deltat >= dt)                           //Update term.(500Hz.dt=2)
    {
      deltat /= 1000.0f;                        //Make millisecond to second.
      //dt2 = deltat;
      MahonyQuaternionUpdate(MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Gx*PI/180.0f, MPU9250.Gy*PI/180.0f, MPU9250.Gz*PI/180.0f, MPU9250.My, MPU9250.Mx, -MPU9250.Mz);
      //MadgwickAHRSupdate(MPU9250.Ax, MPU9250.Ay, MPU9250.Az, MPU9250.Gx*PI/180.0f, MPU9250.Gy*PI/180.0f, MPU9250.Gz*PI/180.0f, MPU9250.My, MPU9250.Mx, -MPU9250.Mz);
      Quternion2Euler(q); //Get Euler angles (roll, pitch, yaw) from Quaternions.
      __LPF(LPF_Euler_angle, Euler_angle, preEuler_angle, deltat);

      //=========================Fuzzy part============================
//      
//      Fuzzification(setting_angle[0], Euler_angle[0], &prev_err[0]);                  //roll
//      Create_Fuzzy_Matrix();
//      Defuzzification(&pid_val[0][0],&pid_val[0][1],&pid_val[0][2]);                  //Fuzzy roll end.
//      
//      Fuzzification(setting_angle[1], Euler_angle[1], &prev_err[1]);                  //pitch
//      Create_Fuzzy_Matrix();
//      Defuzzification(&pid_val[1][0],&pid_val[1][1],&pid_val[1][2]);                  //Fuzzy pitch end.
//      
//      Fuzzification(setting_angle[2], Euler_angle[2], &prev_err[2]);                  //yaw
//      Create_Fuzzy_Matrix();
//      Defuzzification(&pid_val[2][0],&pid_val[2][1],&pid_val[2][2]);                  //Fuzzy yaw end.     
//      pid_gain_update(&pid, pid_val, inpid_val);                                      //From Fuzzy the PID gain value is changed.
      //=========================Fuzzy part END============================
      if (HAL_GetTick() - before_while >= 5000)
      {
        __pid_update(&pid, setting_angle, LPF_Euler_angle, angular_velocity);         //PID value update.
        //__pid_update(&pid, setting_angle, Euler_angle, angular_velocity);         //PID value update.
      }    
      //Euler_angle[1] -= 2.5f;                                  // pich bias.
      //Euler_angle[2] -= 8.2f;                                  // Declination at Seoul korea on 2020-02-04(yaw bias)            
      //if(Euler_angle[2] < 0) Euler_angle[2]   += 360.0f;       // Ensure yaw stays between 0 and 360
      
      deltat = 0.0f;                                             //reset deltat.
    }
    //TM_MPU9250_DataReady(&MPU9250);                            //?????
//=====================Data print transmit UART part===============        
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f \r\n",  \
      MPU9250.Ax, MPU9250.Ay ,MPU9250.Az, MPU9250.Gx, MPU9250.Gy, MPU9250.Gz, MPU9250.Mx, MPU9250.My, MPU9250.Mz);
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f \r\n", MPU9250.Mx, MPU9250.My ,MPU9250.Mz);
    //sprintf((char*)uart2_tx_data2," ASAx = %.2f \t ASAy = %.2f \t ASAz = %.2f\r\n",MPU9250.ASAX, MPU9250.ASAY, MPU9250.ASAZ);
    //sprintf((char*)uart2_tx_data2,"%10.4f %10.4f %10.4f %10.4f\r\n", q[0], q[1], q[2], q[3]);
    
    //sprintf((char*)uart2_tx_data2,"%10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f\r\n", pid_val[0][0],pid_val[0][1],pid_val[0][2],pid_val[1][0],pid_val[1][1],pid_val[1][2],pid_val[2][0],pid_val[2][1],pid_val[2][2]);
    //sprintf((char*)uart2_tx_data2,"%10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f\r\n", pid.Kp[0],pid.Ki[0],pid.Kd[0],pid.Kp[1],pid.Ki[1],pid.Kd[1],pid.Kp[2],pid.Ki[2],pid.Kd[2]);

    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2]);
    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2]);

    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);
    //sprintf((char*)uart2_tx_data2,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2], setting_angle[0], setting_angle[1], setting_angle[2], pid.output[0],pid.output[1], pid.output[2]);
    sprintf((char*)uart2_tx_data2,"%4d  %4d  %4d\r\n", (int)LPF_Euler_angle[0], (int)LPF_Euler_angle[1], (int)LPF_Euler_angle[2]);

    //sprintf((char*)uart2_tx_data3,"%10.2f  %10.2f  %10.2f  %10.2f  %10.2f  %10.2f\r\n",  Euler_angle[0], Euler_angle[1], Euler_angle[2], q[1], q[2], q[3]);
    //sprintf((char*)uart2_tx_data2,"%10.5f  %10.5f  %10.5f\r\n",  angular_velocity[0], angular_velocity[1], angular_velocity[2]);

    //sprintf((char*)uart2_tx_data2,"%10.5f\r\n",dt2);
    
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
    //HAL_UART_Transmit(&huart2,uart2_tx_data3 ,sizeof(uart2_tx_data3), 10);        

 //======================BLDC Moter Part===============================================  
   
    if(HAL_GetTick() - before_while >= 5000 && HAL_GetTick() - before_while < 6000)
    {
       Motor_Start();
    }
 
   if (HAL_GetTick() - before_while >= 6000 && HAL_GetTick() - before_while <= 100000)
   {
     
     if (Controller_1 <= 5)
         {
           MOTOR_V1 = MIN_PULSE;
           MOTOR_V2 = MIN_PULSE;
           MOTOR_V3 = MIN_PULSE;
           MOTOR_V4 = MIN_PULSE;
         }
     
      if (Controller_1 > 5)    //Controller_1은 신호를 주고 있고, Controller_2의 조이스틱이 가운데 위치할 때 (고도만 제어할때)
       {     
         if (fabs(LPF_Euler_angle[0]) <= 15.0f && fabs(LPF_Euler_angle[1]) <= 15.0f)    //Restrict yaw acting Euler angle.
         {           
           pid.output[2] = 0.0f;
         }         
         MOTOR_V1 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_roll * pid.output[0]);// - (int)(MoterGain_pitch * pid.output[1]);
         if (MOTOR_V1 >= MAX_PULSE - MOTER_SAFTY)
           MOTOR_V1 = MAX_PULSE -  MOTER_SAFTY;
         else if (MOTOR_V1 <= MIN_PULSE + 700)
           MOTOR_V1 = MIN_PULSE + 700;
   
         MOTOR_V2 = MIN_PULSE + (Controller_1 * 70) - (int)((MoterGain_roll)  * pid.output[0]);// - (int)((MoterGain_pitch) * pid.output[1]);
         if (MOTOR_V2 >= MAX_PULSE - MOTER_SAFTY)
           MOTOR_V2 = MAX_PULSE - MOTER_SAFTY;
         else if (MOTOR_V2 <= MIN_PULSE + 700)
           MOTOR_V2 = MIN_PULSE + 700;
   
         MOTOR_V3 = MIN_PULSE + (Controller_1 * 70) + (int)(MoterGain_roll * pid.output[0]);// + (int)(MoterGain_pitch * pid.output[1]);
         if (MOTOR_V3 >= MAX_PULSE - MOTER_SAFTY)
           MOTOR_V3 = MAX_PULSE - MOTER_SAFTY;
         else if (MOTOR_V3 <= MIN_PULSE + 700)
           MOTOR_V3 = MIN_PULSE + 700;
   
         MOTOR_V4 = MIN_PULSE + (Controller_1 * 70) - (int)((MoterGain_roll) * pid.output[0]);// + (int)((MoterGain_pitch) * pid.output[1]); 
         if (MOTOR_V4 >= MAX_PULSE - MOTER_SAFTY)
           MOTOR_V4 = MAX_PULSE - MOTER_SAFTY;
         else if (MOTOR_V4 <= MIN_PULSE + 700)
           MOTOR_V4 = MIN_PULSE + 700;          
       }
    }
    
    Motor_Stop(100000, before_while);
    
    //sprintf((char*)uart2_tx_data2,"%10d  %10d  %10d  %10d\r\n",  MOTOR_V1, MOTOR_V2, MOTOR_V3, MOTOR_V4);
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);

//======================BLDC Moter Part END======================================

    
//==============Data transmit part==============================================
    //UART_Now = HAL_GetTick();               //Get current time.
    //UART_deltat += (UART_Now - UART_lastUpdate);       //Set integration time by time elapsed since last filter update (milliseconds).
    //UART_lastUpdate = UART_Now;                   //Update lastupdate time to current time.
//    if (UART_deltat >= 10)
//    {
      //sprintf((char*)uart1_tx_to_MFC,"%.2f,%.2f,%.2f", Euler_angle[0], Euler_angle[1], Euler_angle[2]);
      //sprintf((char*)uart1_tx_to_MFC,"%.2f,%.2f,%.2f", LPF_Euler_angle[0], LPF_Euler_angle[1], LPF_Euler_angle[2]);
      sprintf((char*)uart1_tx_to_MFC,"%d,%d,%d", (int)LPF_Euler_angle[0], (int)LPF_Euler_angle[1], (int)LPF_Euler_angle[2]);

      HAL_UART_Transmit(&huart1,uart1_tx_to_MFC ,sizeof(uart1_tx_to_MFC), 5);
      //HAL_Delay(3);
      HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 4);
      
      //UART_deltat = 0;
//    }
//*********************************************************************************
  
    if(num>=70)
    {
      //HAL_UART_Transmit(&huart2,pid_buffer,sizeof(pid_buffer), 10); //출력테스트용.
      if (strstr((char*)pid_buffer,"B") != NULL)               //Outer PID.
      {
        Parsing_PID_val(pid_buffer, pid_val);
        sprintf((char*)uart1_tx_to_MFC2,"PPP%6.3fPPI%6.3fPPD%6.3fPRRP%6.3fRRI%6.3fRRD%6.3fRYYP%6.3fYYI%6.3fYYD%6.3fY\r\n", pid_val[1][0], pid_val[1][1], pid_val[1][2], pid_val[0][0], pid_val[0][1], pid_val[0][2], pid_val[2][0], pid_val[2][1], pid_val[2][2]);
        pid_gain_update(&pid, pid_val, inpid_val);
        HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        memset(pid_buffer,'\0',sizeof(pid_buffer));
        memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
        num = 0;
      }
      else if (strstr((char*)pid_buffer,"A") != NULL)          //Inner PID.
      {
        Parsing_inPID_val(pid_buffer, inpid_val);
        sprintf((char*)uart1_tx_to_MFC2,"PP%6.3fPI%6.3fPD%6.3fPRP%6.3fRI%6.3fRD%6.3fRYP%6.3fYI%6.3fYD%6.3fY\r\n", inpid_val[1][0], inpid_val[1][1], inpid_val[1][2], inpid_val[0][0], inpid_val[0][1], inpid_val[0][2], inpid_val[2][0], inpid_val[2][1], inpid_val[2][2]);
        pid_gain_update(&pid, pid_val, inpid_val);
        HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        memset(pid_buffer,'\0',sizeof(pid_buffer));
        memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
        num = 0;
      }
      else if (strstr((char*)pid_buffer,"C") != NULL)          //Throttle.
      {
        Parsing_Throttle_val(pid_buffer, &Controller_1);
        sprintf((char*)uart1_tx_to_MFC2,"T%6.3f\r\n", (float)Controller_1);
        HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        memset(pid_buffer,'\0',sizeof(pid_buffer));
        memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
        num = 0;
      }
      else if (strstr((char*)pid_buffer,"D") != NULL)          //Setting Point.
      {
        Parsing_SettingPoint_val(pid_buffer, setting_angle);
        sprintf((char*)uart1_tx_to_MFC2,"S%6.3fP%6.3fR%6.3fY\r\n", setting_angle[0], setting_angle[1], setting_angle[2]);
        HAL_UART_Transmit(&huart1,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        //HAL_UART_Transmit(&huart2,uart1_tx_to_MFC2,sizeof(uart1_tx_to_MFC2), 10);
        memset(pid_buffer,'\0',sizeof(pid_buffer));
        memset(uart1_tx_to_MFC2,'\0',sizeof(uart1_tx_to_MFC2));
        num = 0;
      }
    }
//*********************************************************************************
//-------------------TIme Check--------------------
    //int bb = HAL_GetTick();
    //sprintf((char*)uart2_tx_data2,"%d  %d\r\n",  aa, bb);    
    //HAL_UART_Transmit(&huart2,uart2_tx_data2 ,sizeof(uart2_tx_data2), 10);
//-------------------TIme Check END--------------------

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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CEpin_Pin|CSpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CEpin_Pin CSpin_Pin */
  GPIO_InitStruct.Pin = CEpin_Pin|CSpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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
