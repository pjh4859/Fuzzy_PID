#include "main.h"
#include "nRF24_Receive.h"
//#include "Transmit_To_Controller.h"

TM_NRF24L01_Transmit_Status_t transmissionStatus;

//========================nRF24L01 FUNCTION==============================
void NRF24_Data_save(int* Throttle,float temp, int temp_int, int value, __PID*  pid, float* setting_angle)
{
   int No_Connection1=0;
  switch(value)
  {
    case 't':
      *Throttle = temp_int;
      printf("Throttle : %d\r\n",temp_int);                                      // ����Ʋ �� ����
      value = '\0';
      temp_int = '\0';
      break;
      
    case 'T':
      *Throttle = temp_int;
      printf("Throttle : %d\r\n",temp_int);                                      // ����Ʋ �� ����
      value = '\0';
      temp_int = '\0';
      break;
    
   // ============================== IN PID DATA ============================== //  
     
    case 1:
      pid->iKp[0] = temp;
      //inpid_val[0][0] = temp;
      printf("IN_Roll_P : %.4f\r\n",temp);                                 // IN Roll_P �� ����
      
      value = '\0';
      temp = '\0';
      break;
      
     case 2:
       pid->iKp[1] = temp;
      //inpid_val[1][0] = temp;
      printf("IN_Pitch_P : %.4f\r\n",temp);                               // IN Pitch_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 3:
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
      printf("IN_Yaw_P : %.4f\r\n",temp);                                // IN Yaw_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 4:
       pid->iKi[0] = temp;
      //inpid_val[0][1] = temp;
      printf("IN_Roll_I : %.4f\r\n",temp);                                 // IN Roll_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 5:
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      printf("IN_Pitch_I : %.4f\r\n",temp);                               // IN Pitch_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 6:
       pid->iKi[2] = temp;
      //inpid_val[2][1] = temp;
      printf("IN_Yaw_I : %.4f\r\n",temp);                                 // IN Yaw_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 7:
       pid->iKd[0] = temp;
      //inpid_val[0][2] = temp;
      printf("IN_Roll_D : %.4f\r\n",temp);                                // IN Roll_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 8:
       pid->iKd[1] = temp;
      //inpid_val[1][2] = temp;
      printf("IN_Pitch_D : %.4f\r\n",temp);                              // Pitch_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 9:
       pid->iKd[2] = temp;
      //inpid_val[2][2] = temp;
      printf("IN_Yaw_D : %.4f\r\n",temp);                               // IN Yaw_D �� ����
      value = '\0';
      temp = '\0';
      break;
   //==============================================================//
      
   // ============================== OUT PID DATA ============================== //    
    case 'A':
      pid->Kp[0] = temp;
      //inpid_val[0][0] = temp;
      printf("OUT_Roll_P : %.4f\r\n",temp);                                 // OUT Roll_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'B':
       pid->Kp[1] = temp;
      //inpid_val[1][0] = temp;
      printf("OUT_Pitch_P : %.4f\r\n",temp);                               // OUT Pitch_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'C':
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
      printf("OUT_Yaw_P : %.4f\r\n",temp);                                // OUT Yaw_P �� ����
      value = '\0';
      break;
      
     case 'D':
       pid->Ki[0] = temp;
      //inpid_val[0][1] = temp;
      printf("OUT_Roll_I : %.4f\r\n",temp);                                 // OUT Roll_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'E':
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      printf("OUT_Pitch_I : %.4f\r\n",temp);                               // Pitch_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'F':
       pid->Ki[2] = temp;
      //inpid_val[2][1] = temp;
      printf("OUT_Yaw_I : %.4f\r\n",temp);                                 // Yaw_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'G':
       pid->Kd[0] = temp;
      //inpid_val[0][2] = temp;
      printf("OUT_Roll_D : %.4f\r\n",temp);                                // Roll_D �� ����
      value = '\0';
      break;
      
     case 'H':
       pid->Kd[1] = temp;
      //inpid_val[1][2] = temp;
      printf("OUT_Pitch_D : %.4f\r\n",temp);                              // Pitch_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'I':
       pid->Kd[2] = temp;
      //inpid_val[2][2] = temp;
      printf("OUT_Yaw_D : %.4f\r\n",temp);                               // Yaw_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'a':
      pid->Kp[0] = temp;
      //inpid_val[0][0] = temp;
      printf("OUT_Roll_P : %.4f\r\n",temp);                                 // OUT Roll_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'b':
       pid->Kp[1] = temp;
      //inpid_val[1][0] = temp;
      printf("OUT_Pitch_P : %.4f\r\n",temp);                               // OUT Pitch_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'c':
       pid->iKp[2] = temp;
      //inpid_val[2][0] = temp;
      printf("OUT_Yaw_P : %.4f\r\n",temp);                                // OUT Yaw_P �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'e':
       pid->iKi[1] = temp;
      //inpid_val[1][1] = temp;
      printf("OUT_Pitch_I : %.4f\r\n",temp);                               // Pitch_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'f':
       pid->Ki[2] = temp;
      //inpid_val[2][1] = temp;
      printf("OUT_Yaw_I : %.4f\r\n",temp);                                 // Yaw_I �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'g':
       pid->Kd[0] = temp;
      //inpid_val[0][2] = temp;
      printf("OUT_Roll_D : %.4f\r\n",temp);                                // Roll_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'h':
       pid->Kd[1] = temp;
      //inpid_val[1][2] = temp;
      printf("OUT_Pitch_D : %.4f\r\n",temp);                              // Pitch_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'i':
       pid->Kd[2] = temp;
      //inpid_val[2][2] = temp;
      printf("OUT_Yaw_D : %.4f\r\n",temp);                               // Yaw_D �� ����
      value = '\0';
      temp = '\0';
      break;
      
   // ========================================================================= //  

     case 'r':
      setting_angle[0] = (float)((int8_t)temp);
      printf("Roll_Set_Point : %.0f\r\n",setting_angle[0]);            // Roll_SetPoint �� ����
      value = '\0';
      temp = '\0';
      TM_NRF24L01_Transmit((uint8_t*)setting_angle);
          do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
            
            if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
            {
              
              TM_NRF24L01_Transmit((uint8_t*)setting_angle);
              nRF24_Transmit_Status();
              if(No_Connection1 >= 5)
              {
                //printf("No Connection!\r\n");
                break;
              }
              else
                No_Connection1++;
            }
        }while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
          memset(setting_angle,'\0',8); // C_buff �޸� �ʱ�ȭ
      
      break;
     
     case 'R':
      setting_angle[0] = (float)((int8_t)temp);
      printf("Roll_Set_Point : %.0f\r\n",setting_angle[0]);            // Roll_SetPoint �� ����
      value = '\0';
      temp = '\0';
      break; 
      
     case 'p':
      setting_angle[1] = (float)((int8_t)temp);
      printf("Pitch_Set_Point : %.0f\r\n",setting_angle[1]);           // Pitch_SetPoint �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'P':
      setting_angle[1] = (float)((int8_t)temp);
      printf("Pitch_Set_Point : %.0f\r\n",setting_angle[1]);           // Pitch_SetPoint �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'y':
      setting_angle[2] = (float)((int8_t)temp);
      printf("Yaw_Set_Point : %.0f\r\n",setting_angle[2]);           // Yaw_SetPoint �� ����
      value = '\0';
      temp = '\0';
      break;
      
     case 'Y':
      setting_angle[2] = (float)((int8_t)temp);
      printf("Yaw_Set_Point : %.0f\r\n",setting_angle[2]);           // Yaw_SetPoint �� ����
      value = '\0';
      temp = '\0';
      break;
      
      
    case 'x':
      *Throttle = temp_int;
      printf("Throttle_Reset : %d\r\n",temp_int);                         // ����Ʋ �� ����(Throttle = MIN_PULSE)
      value = '\0';
      temp_int = '\0';
      break;
      
      
  case 'd':
    //printf("debug mode\r\n");
    break;

  }
}



void NRF24_Receive(int* Throttle,float temp, int temp_int,__PID*  pid,float* setting_angle)    // Controller���� PID�� ����
{
  int value='\0';     // ��Ʈ�ѷ����� ���� key_input �� ���� ����
  
  uint8_t dataIn[8]={'\0'};                                     // Controller Data Receive Buffer
  uint8_t dataOut[8]="hi";
   int No_Connection=0;
  
  if(TM_NRF24L01_DataReady())
  {      
      
      TM_NRF24L01_GetData(dataIn);
      
      //printf("%s",dataIn);
        if(dataIn[7]=='q'||dataIn[7]=='Q')      // q ������ �� �� �׽�Ʈ
        {
          value = dataIn[7];
          dataIn[7]='\0';
          printf("\r\n[7] : %c\r\n",value);
        }
        
//        else if(dataIn[7] == 'd')       // d ���� �Ǿ��� �� ����� ���
//        {
//          
//          value = dataIn[7];
//          //dataIn[7] = '\0';

//         
//         //Transmit_to_Controller(pid);
//         
//         }
                
        
        else if(dataIn[7] == 'x')                                             // x ���� �Ǿ��� ��  Throttle�� �ʱ�ȭ
        {
          value = dataIn[7];
          dataIn[7] = '\0'; 
          temp_int=atoi((char*)dataIn);
          temp_int = 0;
        }
        
        else if(dataIn[7] == 'r' || dataIn[7] == 'R')                       // r, R ���� �Ǿ��� �� Roll SetPoint �� ����
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atof((char*)dataIn);                                       // uint8_t ������ ���ŵ� Roll SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[7] == 'p' || dataIn[7] == 'P')                       // p, P ���� �Ǿ��� �� Pitch SetPoint �� ����
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atof((char*)dataIn);                                        // uint8_t ������ ���ŵ� Pitch SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
        
        else if(dataIn[7] == 'y' || dataIn[7] == 'Y')                        // y, Y ���� �Ǿ��� �� Yaw SetPoint �� ����
        {
          value = dataIn[7];
          dataIn[7] = '\0';
          temp = atof((char*)dataIn);                                       // uint8_t ������ ���ŵ� Yaw SetPoint ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.0f\r\n",temp);
        }
 
        else if(dataIn[7]<='9' && dataIn[7]>='1')                     // 1~9�� ���� �Ǿ��� �� pid�� ����
        {
          value = dataIn[7]-48;
          dataIn[7]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t ������ ���ŵ� PID ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %s\r\n",dataIn);
        }
        
        else if(dataIn[7]<='I' && dataIn[7]>='A')
        {
          value = dataIn[7];
          dataIn[7]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t ������ ���ŵ� PID ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
          printf("Receive data : %s\r\n",dataIn);
          TM_NRF24L01_Transmit(dataIn);
          do {
        /* Get transmission status */
            transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
            
            if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
            {
              
              TM_NRF24L01_Transmit(dataIn);
              nRF24_Transmit_Status();
              if(No_Connection >= 5)
              {
                //printf("No Connection!\r\n");
                break;
              }
              else
                No_Connection++;
            }
        }while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
          memset(dataIn,'\0',8); // C_buff �޸� �ʱ�ȭ
        }
        
        else if((dataIn[7]<='i' && dataIn[7]>='a'))
        {
          value = dataIn[7];
          dataIn[7]='\0';
          temp=atof((char*)dataIn);                                         // uint8_t ������ ���ŵ� PID ���� float������ ��ȯ�Ͽ� temp �� ����
          //printf("\r\n dataIn_test : %.3f\r\n",temp);
           printf("[value] : %c\r\n",value);
           
        }
        
        else if(dataIn[7]=='t' || dataIn[7] == 'T')                                               // t ������ �� Throttle�� ����
        {
          value = dataIn[7];  
          dataIn[7]='\0';
          temp_int=atoi((char*)dataIn);                                     //  uint8_t ������ ���ŵ� Throttle ���� int������ ��ȯ�Ͽ� temp_init�� ����

        }
      NRF24_Data_save(Throttle,temp,temp_int,value,pid,setting_angle);        //  ���ŵ� ������ ���� �Լ�
      TM_NRF24L01_PowerUpRx();
  }
}
void nRF24_Transmit_Status()
{
  /* Check transmit status */
      if (transmissionStatus == TM_NRF24L01_Transmit_Status_Ok) 
      {
        /* Transmit went OK */
        printf("\r\nOK   \r\n");      
      }
      else if (transmissionStatus == TM_NRF24L01_Transmit_Status_Lost) 
      {
         /* Message was LOST */
           printf("\r\nLOST \r\n");
      }
      else {
	/* This should never happen */
         printf("\r\nSEND \r\n");
      } 
}

