#include <WiFi.h>
#include <WiFiUdp.h>
#include <pthread.h>
#include <string.h>

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems

const char* ssid = "room201";
const char* password =  "room201201";

const char * serverAddress = "192.168.0.21";
const int serverPort = 9000;

WiFiUDP udp;
 
WiFiServer wifiServer(9000);
WiFiClient wifiClient;

static int socket_flag = 1;
static int thread_flag = 1;
char buff[255]={0,};
char tcp_buff[100]={0,};
char ack_buff[100];
//vTaskDelay(10);

void parsing_pid(char* pb);

void parsing_thro(char* pb);

void *t_function(void *arg);

void *u_function(void *arg);

pthread_mutex_t t_mutex;//= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  t_cond;//= PTHREAD_COND_INITIALIZER;

pthread_mutex_t u_mutex;//= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  u_cond;//= PTHREAD_COND_INITIALIZER;

void *u_function(void *arg)
{
  int i;
  //char* ret;
  while(1)
  {
    pthread_mutex_lock(&u_mutex);
    if(!(thread_flag ==1))
      pthread_cond_wait(&u_cond,&u_mutex);
    else
      pthread_cond_signal(&u_cond);
    if(Serial.available()>0)
    {
      char buff[255]={0,};  
      for(i=0;Serial.available()>0;i++)
      {
        buff[i] = Serial.read();
      }
      if(!(buff[0] == '\0'))
      {
        
        if(strstr(buff,"PP")!=NULL)
        {
          memcpy(tcp_buff,buff,100);
          memset(buff,'\0',sizeof(buff));
        }
        else if(strstr(buff,"T")!=NULL)
        {
          tcp_buff[0] = '1';
          memset(buff,'\0',sizeof(buff));
        }
        else
        {
        udp.beginPacket(serverAddress,serverPort);    
        udp.printf("%s         \0",buff);
        udp.endPacket();
        }
        memset(buff,'\0',sizeof(buff));
      }
    }
    delay(10);
    pthread_mutex_unlock(&u_mutex);
  }
} 

char ch[255]={0,};
void *t_function(void *arg)
{
  int i = 0;
  while(1)
  { 
     wifiClient = wifiServer.available(); // accept();
     while(wifiClient.connected()) // client 연결확인
     {
        if(wifiClient.available()>0);
          {
            socket_flag = 2;
            while(wifiClient.available()>0) // 데이터 통신확인 연결되있을때부터 비교
            {
              ch[i++] = wifiClient.read();
            }
            //wifiClient.write(ch,100);
            if(strstr(ch,"PPP")!=NULL)
            {
              parsing_pid(ch);
              Serial.print('B'); // double
              memset(ch,'\0',sizeof(ch));  
            }
            else if(strstr(ch,"PP")!=NULL)
            {
              parsing_pid(ch);
              Serial.print('A'); // solo
              memset(ch,'\0',sizeof(ch));  
            }
            else if(strstr(ch,"T")!=NULL)
            {
              parsing_thro(ch);
              Serial.print('C');
              memset(ch,'\0',sizeof(ch));
              memset(ack_buff,'\0',sizeof(ack_buff));
              ack_buff[0]='1';
              wifiClient.write(ack_buff,100);
            }
            else
            {
              //wifiClient.write(ch,100);
              //memset(ch,'\0',sizeof(ch));  
            }
            i=0;
          }
          socket_flag = 1;
          if(!(tcp_buff[0]=='\0'))
          {
            wifiClient.write(tcp_buff,100);
            memset(tcp_buff,'\0',100);
          }  
      }
      wifiClient.stop();  
   }
}   

void setup() 
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  delay(1000);
// We start by connecting to a WiFi network
reboot:
    WiFi.begin(ssid, password);
    delay(2000);
    if(WiFi.status() != WL_CONNECTED)  {
          goto reboot;
        }
  wifiServer.begin(); 

  pthread_t u_thread; 
  pthread_t t_thread;
  pthread_attr_t attr;   //쓰레드 속성값
 
  int status;
  int a = 100;

  // '쓰레드 분리' 속성값 지정 : PTHREAD_CREATE_DETACHED
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  pthread_mutex_init(&u_mutex, NULL);
  pthread_mutex_init(&t_mutex, NULL);
  pthread_cond_init(&u_cond, NULL);
  pthread_cond_init(&t_cond, NULL);
  
  pthread_create(&u_thread, &attr, u_function, (void *)&a);
  pthread_create(&t_thread, &attr, t_function, (void *)&a);
  
  pthread_attr_destroy(&attr); 
}
void loop() 
{ 
  if(socket_flag == 1)
  {
    pthread_cond_signal(&u_cond);
  }
}

void parsing_pid(char* pb)
{
  int i;
  int Blink;
  char* R_PP, * R_PI, * R_PD, * R_RP, * R_RI, * R_RD, * R_YP, * R_YI, * R_YD;
  
  R_PP = strtok((char*)pb, "PP");
  R_PI = strtok(NULL, "PI");
  R_PD = strtok(NULL, "PD");
  
  R_RP = strtok(NULL, "R");
  R_RP = strtok(NULL, "RP");
  R_RI = strtok(NULL, "RI");
  R_RD = strtok(NULL, "RD");
  
  R_YP = strtok(NULL, "YP");
  R_YI = strtok(NULL, "YI");
  R_YD = strtok(NULL, "YD");
  
  for(i = 0; i<strlen(R_RP); i++)
  {
    Serial.print(R_RP[i]);
  }
  Serial.print(',');
  delay(10);
  
  for(i = 0; i<strlen(R_RI); i++)
  {
    Serial.print(R_RI[i]);
  }
  Serial.print(',');
  delay(10);
  
  for(i = 0; i<strlen(R_RD); i++)
  {
    Serial.print(R_RD[i]);
  }
  Serial.print(',');
  delay(10);
/**************************************************/

  for(i = 0; i<strlen(R_PP); i++)
  {
    Serial.print(R_PP[i]);
  }
  Serial.print(',');
  delay(10);
  for(i = 0; i<strlen(R_PI); i++)
  {
    Serial.print(R_PI[i]);
  }
  Serial.print(',');
  delay(10);
  
  for(i = 0; i<strlen(R_PD); i++)
  {
    Serial.print(R_PD[i]);
  }
  Serial.print(',');
  delay(10);
/**************************************************/

  for(i = 0; i<strlen(R_YP); i++)
  {
    Serial.print(R_YP[i]);
  }
  Serial.print(',');
  delay(10);
  
  for(i = 0; i<strlen(R_YI); i++)
  {
    Serial.print(R_YI[i]);
  }
  Serial.print(',');
  delay(10);
  
  for(i = 0; i<strlen(R_YD); i++)
  {
    Serial.print(R_YD[i]);
  }
  Serial.print(',');
  delay(10);
  
  Blink = 100 - strlen(R_PP) - strlen(R_PI) - strlen(R_PD) \
              - strlen(R_RP) - strlen(R_RI) - strlen(R_RD) \
              - strlen(R_YP) - strlen(R_YI) - strlen(R_YD) - 10;
  char buf = '0';

  for(i = 0; i< Blink; i++)
  {
    Serial.print(buf);
    if(i % 10 == 0)
      delay(10);
  }
  
}

void parsing_thro(char* pb)
{

  int i;
  int Blink;
  
  char* T;
  T = strtok((char*)ch,"T");

  for(i = 0; i<strlen(T); i++)
  {
    Serial.print(T[i]);
  }
  Serial.print(',');
  delay(10);

  Blink = 100 - strlen(T) - 2;
  char buf = '0';

  for(i = 0; i< Blink; i++)
  {
    Serial.print(buf);
    if(i % 10 == 0)
      delay(10);
  }
}
