#include "Arduino.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <pthread.h>
#include <string.h>

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems

#include "driver/rtc_io.h"
#include "esp_camera.h"

          

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


//연결할 Wifi 이름 및 비밀번호
const char* ssid = "room201";
const char* password =  "room201201";

//Roll, Pitch, Yaw UDP 서버 주소 및 포트
const char * serverAddress = "192.168.0.21";
const int serverPort = 9000;

//Camera Data UDP 서버 주소 및 포트
const char * cameraAddress = "192.168.0.21";
const int cameraPort = 8000;

WiFiUDP sudp;  // UDP Client Roll, Pitch, Yaw 전송 
WiFiUDP cudp; // UDP Client Camera Data 전송


//TCP/IP 서버 생성
WiFiServer wifiServer(9000);  
WiFiClient wifiClient;

camera_fb_t * fb = NULL;

#define BUFSIZE 65507

char message[BUFSIZE];
size_t fsize = 0;

static int socket_flag = 1; 
static int thread_flag = 1;

char udp_buff[200]={0,};
char tcp_buff[100]={0,};
char ack_buff[100];

unsigned long pre_sec = 0;
unsigned long sec = 0;
unsigned long dt = 0;

void parsing_pid(char* pb);

void parsing_thro(char* pb);

void parsing_setpoint(char* pb);

void *t_function(void *arg);

void *u_function(void *arg);

void *c_function(void *arg);

pthread_mutex_t u_mutex;//= PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  u_cond;//= PTHREAD_COND_INITIALIZER;

void *u_function(void *arg)                           //UDP 클라이언트 함수
{
  int i;
  pre_sec = millis();
  while(1)
  {
    sec = millis();
    dt+=sec - pre_sec;
    pre_sec = sec;
    if(dt>=2000)                                     
    {
      pthread_mutex_lock(&u_mutex);
      if(!(socket_flag ==1))
        pthread_cond_wait(&u_cond,&u_mutex);
      else
        pthread_cond_signal(&u_cond);
      if(Serial.available()>0)                        // 데이터가 들어오면 들어온 데이터(Byte) 길이 return
      {
        char udp_buff[200]={0,};                      // 버퍼 초기화
        //memset(udp_buff,0,sizeof(udp_buff)); 
        for(i=0;Serial.available()>0;i++)
        {
          udp_buff[i] = Serial.read();                // 1Byte씩 데이터 버퍼에 저장
        }
        if(!(udp_buff[0] == '\0'))                    // 데이터가 있는지 확인
       {
          sudp.beginPacket(serverAddress,serverPort);    
          sudp.printf("%s         \0",udp_buff);
          sudp.endPacket();
       }
      }
      delay(9);
      pthread_mutex_unlock(&u_mutex);
    }
  }
} 

char ch[255]={0,};
void *t_function(void *arg) // TCP/IP 서버 함수
{
  int i = 0;
  
  while(1)
  { 
     wifiClient = wifiServer.available();     // accept();
     while(wifiClient.connected())            // client 연결확인
     {
        if(wifiClient.available()>0);         // 데이터가 들어오면 들어온 데이터(Byte) 길이 return
          {
            socket_flag = 2;
            while(wifiClient.available()>0)
            {
              ch[i++] = wifiClient.read();    // 1Byte씩 데이터 버퍼에 저장
            }
            if(strstr(ch,"T")!=NULL)          // Throttle
            {                     
              parsing_thro(ch);
              Serial.print('C');
              
              memset(ch,'\0',sizeof(ch));
              memset(ack_buff,'\0',sizeof(ack_buff));
              ack_buff[0]='1';
              wifiClient.write(ack_buff,100);
            }
            else if(strstr(ch,"S")!=NULL)     //Set-Point
            {
              parsing_setpoint(ch);
              Serial.print('D');
            
              memset(ch,'\0',sizeof(ch));
              memset(ack_buff,'\0',sizeof(ack_buff));
              ack_buff[0]='1';
              wifiClient.write(ack_buff,10);
            }
            else                              //ACK용
            {
              wifiClient.write(ch,100);
            }
            if(strstr(ch,"PPP")!=NULL)        //PPID
            {
              parsing_pid(ch);
              Serial.print('B');
              memset(ch,'\0',sizeof(ch));  
            }
            else if(strstr(ch,"PP")!=NULL)    //PID
            {
              parsing_pid(ch);
              Serial.print('A'); 
              memset(ch,'\0',sizeof(ch));  
            }
            i=0;
          }
          socket_flag = 1;
      }
      wifiClient.stop();  
   }
}   

void *c_function(void *arg)                   //카메라 클라이언트 함수
{
  
  while(1)
  {  
    cudp.begin(cameraPort);    
Camera:     
   fb = esp_camera_fb_get();                  //카메라 데이터 읽어오기
   if(!fb) {
    goto Camera;
  }

  fsize = fb->len;                            //이미지 데이터 크기 저장

  cudp.beginSize(cameraAddress,cameraPort);   //이미지 데이터 크기 전송  
  cudp.endSize(fsize);
  
  static uint8_t cbuf[1460]={0,};
  int i = 0;
  while(fsize)
  {
    size_t toRead = fsize;
    if(toRead > 1460)
    {
      toRead = 1460;
    }
    memset(cbuf,'\0',sizeof(cbuf));
    for(int j=0;j<toRead;j++)
    {
      cbuf[j] = (fb->buf[i++]);
    }
    cudp.beginData(cameraAddress,cameraPort);
    cudp.image_write((char*)cbuf,toRead);
    cudp.endData();
    fsize -= toRead;
  }
  cudp.endPacket();
  i=0;
  esp_camera_fb_return(fb);
  delay(10); 
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

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  config.frame_size = FRAMESIZE_CIF; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // Camera reboot fix. Powers down the camera for 10 ms to reset it
  // since there is no reset pin on the OV2640
  gpio_set_level((gpio_num_t)config.pin_pwdn, 1);
  delay(10);
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  wifiServer.begin(); 
  
  pthread_t u_thread; 
  pthread_t t_thread;
  pthread_t c_thread;
  pthread_attr_t attr;   //쓰레드 속성값
 
  int status;
  int a = 100;

  // '쓰레드 분리' 속성값 지정 : PTHREAD_CREATE_DETACHED
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  pthread_mutex_init(&u_mutex, NULL);
  //pthread_mutex_init(&t_mutex, NULL);
  //pthread_mutex_init(&c_mutex, NULL);
  
  pthread_cond_init(&u_cond, NULL);
  //pthread_cond_init(&t_cond, NULL);
  //pthread_cond_init(&c_cond, NULL);
  
  pthread_create(&u_thread, &attr, u_function, (void *)&a);
  pthread_create(&t_thread, &attr, t_function, (void *)&a);
  pthread_create(&c_thread, &attr, c_function, (void *)&a);
  
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
  
  Blink = 70 - strlen(R_PP) - strlen(R_PI) - strlen(R_PD) \
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

  Blink = 70 - strlen(T) - 2;
  char buf = '0';

  for(i = 0; i< Blink; i++)
  {
    Serial.print(buf);
    if(i % 10 == 0)
      delay(10);
  }
}

void parsing_setpoint(char* pb)
{
  int i;
  int Blink;
  
  char* P, *R, *Y, *temp;
  temp = strtok((char*)ch,"S");
  P = strtok(temp, "P");
  R = strtok(NULL, "R");
  Y = strtok(NULL, "Y");

  for(i = 0; i<strlen(R); i++)
  {
    Serial.print(R[i]);
  }
  Serial.print(',');
  delay(10);

  for(i = 0; i<strlen(P); i++)
  {
    Serial.print(P[i]);
  }
  Serial.print(',');
  delay(10);

  for(i = 0; i<strlen(Y); i++)
  {
    Serial.print(Y[i]);
  }
  Serial.print(',');
  delay(10);
  
  Blink = 70 - strlen(P)- strlen(R)- strlen(Y)  - 4;
  char buf = '0';

  for(i = 0; i< Blink; i++)
  {
    Serial.print(buf);
    if(i % 10 == 0)
      delay(10);
  }
}
