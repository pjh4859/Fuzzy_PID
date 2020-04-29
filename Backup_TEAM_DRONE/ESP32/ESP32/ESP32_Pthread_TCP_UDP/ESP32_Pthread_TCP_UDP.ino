#include <WiFi.h>
#include <WiFiUdp.h>
#include <pthread.h>
#include <string.h>

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems

const char* ssid = "room201";
const char* password =  "room201201";

const char * serverAddress = "192.168.0.23";
const int serverPort = 9000;

WiFiUDP udp;
 
WiFiServer wifiServer(9000);
WiFiClient wifiClient;

static int socket_flag = 1;
static int thread_flag = 1;
char buff[255]={0,};
char tcp_buff[100]={0,};
//vTaskDelay(10);

void *t_function(void *arg);

void *u_function(void *arg);

pthread_mutex_t t_mutex;//= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  t_cond;//= PTHREAD_COND_INITIALIZER;

pthread_mutex_t u_mutex;//= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  u_cond;//= PTHREAD_COND_INITIALIZER;

void *u_function(void *arg)
{
  int i;
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
          memcpy(buff,tcp_buff,100);
        }
        else
        {
        udp.beginPacket(serverAddress,serverPort);    
        udp.printf("%s",buff);
        memset(buff,'\0',sizeof(buff));
        udp.endPacket();
        }
      }
    }
    delay(10);
    pthread_mutex_unlock(&u_mutex);
  }
} 

void *t_function(void *arg)
{
  int i = 0;
  while(1)
  { 
     wifiClient = wifiServer.available(); // accept();
     while(wifiClient.connected()) 
     {
        if(wifiClient.available()>0);
          {
            socket_flag = 2;
            while(wifiClient.available()>0) 
            {
              char c = wifiClient.read();
              Serial.print(c);
              if(wifiClient.available()%9 ==0)
                delay(10);
            }
            
            //Serial.print(ch);
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
