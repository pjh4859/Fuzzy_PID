#include "key_Debug.h"
#include "main.h"
#include <stdio.h>

uint8_t C_buff[7];
uint8_t tempbuf[7];
uint16_t C_count;
extern uint8_t key_input;
extern UART_HandleTypeDef huart2;

void Keyboard_Debug(float* temp,uint8_t key_flag)
{
  char save_flag = 0; 
  int j=0;

  while(!save_flag)
  {
    while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
    {
      if(key_input==0x0D)// 엔터 입력 받으면 저장
      {
        key_input ='\0';                 //키 버퍼 초기화
        C_buff[6]='\0';
        sprintf((char*)tempbuf,"%s", C_buff);
        printf("\r\n save : %s\r\n",tempbuf);
        
        memset(C_buff,'\0',7);      // C_buff 메모리 초기화

        C_count=0;//문자 카운터 초기화
        save_flag=1;
        break;
      }
      else if(key_flag<='9' || key_flag>='1')           // key_input값이 1~9를 받았을 때
      {
        while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
        if(key_input <= '9' && key_input >= '0' ||key_input == '.')
        {
          if(C_count<6)// 6글자 이하만 입력 받음
          {
            C_buff[C_count]=key_input;
            ++C_count;
            printf("%c",C_buff[j++]);
            if(C_count>=6)
            {
              C_count=0;
            }
            
            
          }
           *temp = atof((char*)C_buff);
            key_input ='\0';
            break;
         }
        
        key_flag = '\0';
      }
      
     else if(key_flag<='I' || key_flag>='A')           // key_input값이 a~i를 받았을 때
     {
       while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
       if(key_input <= '9' && key_input >= '0' ||key_input == '.')
       {
         if(C_count<6)// 6글자 이하만 입력 받음
         {
           C_buff[C_count]=key_input;
           ++C_count;
           printf("%c",C_buff[j++]);
           if(C_count>=6)
           {
             C_count=0;
           }
         }
          *temp = atof((char*)C_buff);
           key_input ='\0';
           break;
        }
       key_flag = '\0';
     }
      
      else if(key_flag=='t' || key_flag == 'T')                            // key_input값이 t 를 받았을때
      {
        while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK));
        if(key_input == 't' || key_flag == 'T')
       {
         
         if(C_count<2)   // 2글자 이하만 입력 받음
         {
           C_buff[C_count]=key_input;
           ++C_count;
           printf("%c",C_buff[j++]);
           if(C_count>=2)
           {
             printf("\r\nModify data : %s",C_buff);
           }
         }
         *temp = atoi((char*)C_buff);
           key_input ='\0';
           key_flag = '\0';
       }
      }
    }
  }
 }

void Keyboard_Debug_Set_Point(int* temp)
{
  char save_flag = 0;
  int j=0;

  while(!save_flag)
  {
    while(!(HAL_UART_Receive(&huart2,&key_input,sizeof(key_input),10)==HAL_OK))
    {
      if(key_input==0x0D)// 엔터 입력 받으면 저장
      {
        key_input='\0';                 //키 버퍼 초기화
        C_buff[6]='\0';
        sprintf((char*)tempbuf,"%s", C_buff);
        printf("\r\n save : %s",tempbuf);
        
        memset(C_buff,'\0',7); // C_buff 메모리 초기화

        C_count=0;//문자 카운터 초기화
        save_flag=1;
        break;
      }
      
        else if(key_input <= '9' && key_input >= '0' || key_input == '-')
        {
          
          if(C_count<5)// 5글자 이하만 입력 받음
          {
            C_buff[C_count]=key_input;
            ++C_count;
            printf("%c",C_buff[j++]);
            if(C_count==5)
            {
              printf("\r\n");
              printf("Modify data : %s",C_buff);
            }
          }
         
          *temp = atoi((char*)C_buff);
            key_input ='\0';
         }
    }
  }
 }