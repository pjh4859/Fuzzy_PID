#include "main.h"
#include "nRF24_Receive.h"


//========================nRF24L01 FUNCTION==============================
void NRF24_Data_save(int* Throttle,float temp, int temp_int, int value, __PID*  pid, float* setting_angle, float Euler_angle_yaw)
{
  
    static float temp_yaw_angle ;
   temp_yaw_angle = Euler_angle_yaw;   
   static float temp_set_angle;
   static uint8_t flag=0;
   if (flag == 0 && HAL_GetTick() > 5000)
   {
     temp_set_angle = Euler_angle_yaw;
     flag = 1;     
   }
  switch(value)
  {
    case 't':
      *Throttle = temp_int;
      //printf("Throttle : %d\r\n",temp_int);                                      // 쓰로틀 값 저장
      value = '\0';
      temp_int = '\0';
      break;
      
    case 1:
      pid->iKp[0] = temp;
      //inpid_val[0][0] = temp;

      //printf("Roll_P[0][0] : %.4f\r\n",temp);                                 // Roll_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 2:
       pid->iKp[1] = temp;
      //inpid_val[1][0] = temp;
     // printf("Pitch_P[1][0] : %.3f\r\n",temp);                               // Pitch_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 3:
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
     // printf("Yaw_P[2][0] : %.3f\r\n",temp);                                // Yaw_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 4:
       pid->iKi[0] = temp;
      //inpid_val[0][1] = temp;
     // printf("Roll_I[0][1] : %.3f\r\n",temp);                                 // Roll_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 5:
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      //printf("Pitch_I[1][1] : %.3f\r\n",temp);                               // Pitch_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 6:
       pid->iKi[2] = temp;
      //inpid_val[2][1] = temp;
     // printf("Yaw_I[2][1] : %.3f\r\n",temp);                                 // Yaw_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 7:
       pid->iKd[0] = temp;
      //inpid_val[0][2] = temp;
      //printf("Roll_D[0][2] : %.3f\r\n",temp);                                // Roll_D 값 저장
      value = '\0';
      break;
      
     case 8:
       pid->iKd[1] = temp;
      //inpid_val[1][2] = temp;
      //printf("Pitch_D[1][2] : %.3f\r\n",temp);                              // Pitch_D 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 9:
       pid->iKd[2] = temp;
      //inpid_val[2][2] = temp;
     // printf("Yaw_D[2][2] : %.3f\r\n",temp);                               // Yaw_D 값 저장
      value = '\0';
      temp = '\0';
      break;
      
      // ============================== OUT PID DATA ============================== //    
    case 'A':
      pid->Kp[0] = temp;
      //inpid_val[0][0] = temp;
     // printf("OUT_Roll_P : %.3f\r\n",temp);                                 // Roll_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'B':
       pid->Kp[1] = temp;
      //inpid_val[1][0] = temp;
     // printf("OUT_Pitch_P : %.3f\r\n",temp);                               // Pitch_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'C':
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
     // printf("OUT_Yaw_P : %.3f\r\n",temp);                                // Yaw_P 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'D':
       pid->Ki[0] = temp;
      //inpid_val[0][1] = temp;
      //printf("OUT_Roll_I : %.3f\r\n",temp);                                 // Roll_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'E':
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      //printf("OUT_Pitch_I : %.3f\r\n",temp);                               // Pitch_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'F':
       pid->Ki[2] = temp;
      //inpid_val[2][1] = temp;
     // printf("OUT_Yaw_I : %.3f\r\n",temp);                                 // Yaw_I 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'G':
       pid->Kd[0] = temp;
      //inpid_val[0][2] = temp;
      //printf("OUT_Roll_D : %.3f\r\n",temp);                                // Roll_D 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'H':
       pid->Kd[1] = temp;
      //inpid_val[1][2] = temp;
      //printf("OUT_Pitch_D : %.3f\r\n",temp);                              // Pitch_D 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'I':
       pid->Kd[2] = temp;
      //inpid_val[2][2] = temp;
     // printf("OUT_Yaw_D : %.3f\r\n",temp);                               // Yaw_D 값 저장
      value = '\0';
      temp = '\0';
      break;
      
   // ========================================================================= //  
      
     case 'r':
      setting_angle[0] = (float)((int8_t)temp);
      //printf("Roll_Set_Point : %.0f\r\n",setting_angle[0]);            // Roll_SetPoint 값 저장
      value = '\0';
      temp = '\0';
      break;
       
     case 'p':
      setting_angle[1] = (float)((int8_t)temp);
      //printf("Pitch_Set_Point : %.0f\r\n",setting_angle[1]);           // Pitch_SetPoint 값 저장
      value = '\0';
      temp = '\0';
      break;
      
     case 'y':       
       if (temp <= 5 && temp >= -5)
       {
         setting_angle[2] = temp_set_angle;                                      // Yaw_SetPoint 값 저장
         
          if (setting_angle[2] < -180)                                                   //correct angle jump ( ex: 180 -> -180)
          setting_angle[2] = setting_angle[2] + 360;
        else if (setting_angle[2] > 180)
          setting_angle[2] = setting_angle[2] - 360;
       }
      else
      {
        setting_angle[2] = temp_yaw_angle + (float)((int8_t)temp);
        temp_set_angle = temp_yaw_angle;
        
         if (setting_angle[2] < -180)                                                       //correct angle jump ( ex: 180 -> -180)
          setting_angle[2] = setting_angle[2] + 360;
        else if (setting_angle[2] > 180)
          setting_angle[2] = setting_angle[2] - 360;
      }
      value = '\0';
      temp = '\0';
      break;
      
    case 'x':
      *Throttle = temp_int;
     // printf("Throttle_Reset : %d\r\n",temp_int);                                     // 쓰로틀 값 저장(Throttle = MIN_PULSE)
      value = '\0';
      temp = '\0';
      break;
  }
}

void NRF24_Receive(int* Throttle,float temp, int temp_int,__PID*  pid,float* setting_angle, float Euler_angle_yaw)    // Controller에서 PID값 수신
{
  
  int value='\0';     // 컨트롤러에서 받은 key_input 값 저장 변수
  uint8_t dataIn[8]={0};                                     // Controller Data Receive Buffer

  //printf("Receive_Data_Ready : %d\r\n",TM_NRF24L01_DataReady());
  
  if(TM_NRF24L01_DataReady())
  {
   // printf("Receive_Data_Ok : %d\r\n",TM_NRF24L01_DataReady());
       
      TM_NRF24L01_GetData(dataIn);
      
        if(dataIn[7]=='q'||dataIn[7]=='Q')      // q 눌렀을 때 퀵 테스트
        {
          value = dataIn[7];
          dataIn[7]='\0';
          //printf("\r\n[6] : %c\r\n",value);
        }
        
        else if(dataIn[7] == 'd'||dataIn[7]=='D')       // d 수신 되었을 때 디버그 모드
        {
          value = dataIn[7];
          dataIn[7] = '\0'; 
          temp_int=atoi((char*)dataIn);
          //temp_int = 0;
          //printf("\r\n dataIn_test : %d\r\n",temp_int);
        }
        
        else if(dataIn[7] == 's')                     // s 수신 되었을 때 주행 모드 (컨트롤러에서 s 누르면 드론의 현재 Roll, Pitch, Yaw 값을 컨트롤러 로 송신하게 구현할 예정)
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          //printf("key_input : %c", value);
        }
        
        else if(dataIn[7] == 'x')                                             // x 수신 되었을 때  Throttle값 초기화
        {
          value = dataIn[7];
          dataIn[7] = '\0'; 
          temp_int=atoi((char*)dataIn);
          temp_int = 0;
        }
        
        else if(dataIn[7] == 'r'||dataIn[7]=='R')                       // r, R 수신 되었을 때 Roll SetPoint 값 수신
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atof((char*)dataIn);                                       // uint8_t 형으로 수신된 Roll SetPoint 값을 float형으로 변환하여 temp 에 저장
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[7] == 'p'||dataIn[7]=='P')                       // p, P 수신 되었을 때 Pitch SetPoint 값 수신
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atof((char*)dataIn);                                        // uint8_t 형으로 수신된 Pitch SetPoint 값을 float형으로 변환하여 temp 에 저장
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[7] == 'y'||dataIn[7]=='Y')                        // y, Y 수신 되었을 때 Yaw SetPoint 값 수신
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atoi((char*)dataIn);                                       // uint8_t 형으로 수신된 Yaw SetPoint 값을 float형으로 변환하여 temp 에 저장
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
 
        else if(dataIn[7]<='9' && dataIn[7]>='1')                     // 1~9번 수신 되었을 때 pid값 수신
        {
          value = dataIn[7]-48;
          dataIn[7]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t 형으로 수신된 PID 값을 float형으로 변환하여 temp 에 저장
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
        }
        
        else if(dataIn[7]<='I' && dataIn[7]>='A')
        {
          value = dataIn[7];
          dataIn[7]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t 형으로 수신된 PID 값을 float형으로 변환하여 temp 에 저장
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
        }
        
        
        else if(dataIn[7]=='t')                                               // 0번 눌렀을 때 Throttle값 수신
        {
          value = dataIn[7];  
          dataIn[7]='\0';
          temp_int=atoi((char*)dataIn);                                     //  uint8_t 형으로 수신된 Throttle 값을 int형으로 변환하여 temp_init에 저장
          //printf("\r\n dataIn_test : %d\r\n",temp_int);
//          if(temp_int>5)
//          {
//            MOTOR_V1 = 8000+(Controller_1*70);
//            MOTOR_V2 = 8000+(Controller_1*70);
//            MOTOR_V3 = 8000+(Controller_1*70);
//            MOTOR_V4 = 8000+(Controller_1*70);
//          }
        }
//        else
//        {
//          
//          
//        }
        
      NRF24_Data_save(Throttle,temp,temp_int,value,pid,setting_angle,Euler_angle_yaw);        //  수신된 데이터 저장 함수
      TM_NRF24L01_PowerUpRx();
  }
}
//===================================================