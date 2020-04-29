#include "Arduino.h"
#include <WiFi.h>
#include <WiFiUdp.h>

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

#define Debuging_mode

const char* ssid = "room201";
const char* password = "room201201";



const char * serverAddress = "192.168.0.23";
const int serverPort = 8000;

const char * okAddress = "192.168.0.23";
const int okPort = 8008;

const int sizePort = 8001;

/*
const char * serverAddress = "192.168.0.21";
const int serverPort = 8000;

const char * okAddress = "192.168.0.21";
const int okPort = 8008;

const int sizePort = 8001;
*/

WiFiUDP udp;
WiFiUDP okudp;
WiFiUDP szudp;


static int camera_flag = 0;
static int size_flag = 1;
static int data_flag = 0;
static int ReSe_flag = 0;
static int send_count = 0;

static int ok_flag = 1;

camera_fb_t* fb = NULL;
FILE* fp = NULL;

static int flag = 0;
static int Reflag = 0;
static int oneflag = 0;
static int once_flag =0;

#define BUFSIZE 65507
char message[BUFSIZE];
size_t fsize = 0;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  delay(1000);
reboot:
  WiFi.begin(ssid, password);
  delay(2000);
  if (WiFi.status() != WL_CONNECTED)
  {
    goto reboot;
  }

  Serial.println(WiFi.localIP());


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
  config.frame_size = FRAMESIZE_CIF;
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

  udp.begin(serverPort);
  okudp.begin(okPort);
  szudp.begin(sizePort);
}

int k = 0;
char cbuff[50];
int sendcount=0;

void loop() {

  if (flag == 0)              // START 받는 부분
  {
END:
    flag = okudp.StartPacket();
    delay(9);
  }
  else if (flag == 1)         // OK 받는 부분
  {
   // Serial.println("START");  // START 확인용
    flag = okudp.receviePacket(); 
    //delay(1);
  }
  else if (flag == 2 || flag == 3)  // 사진 데이터 받는 부분
  {
//if(once_flag ==0)
//{   
//  if(sendcount==2)
//        once_flag = 1;

//     sendcount++;
      if(once_flag == 1) 
      {
                                          // ReSendSize 받는 부분
       do{
        size_flag = okudp.SizePacket();       
//#ifdef Debuging_mode          
        //Serial.println("RSS Wait..");
        if(size_flag == 1)
          Serial.println("ReSendSize");        
//#endif          
       }while(size_flag !=1);

                                          // OK 받는 부분
        do{
        size_flag = okudp.OKPacket();       
//#ifdef Debuging_mode
        //Serial.println("RSS OK Wait..");          
        if(size_flag == 3)
          Serial.println("RSS OK");
//#endif          
       }while(size_flag !=3);
       size_flag = 1;
      }
if(size_flag ==1)                         
{   
    once_flag = 1;
    //if(oneflag == 0)                    // OK 확인용
    //{
    //  Serial.println("OK");
    //  oneflag = 1;
    //}
    ReSend:
    //if (ReSe_flag == 1)                 // ReSend 확인용
    //{
    //  Serial.println("ReSend OK");
    //  ReSe_flag == 0;
    //}
    
    fb = esp_camera_fb_get();             // 카메라 데이터 얻는 부분
    if (!fb) {
      goto ReSend;
    }
    char data_check[5];
    for (int k = 6; k < 10; k++)
    {
      data_check[k - 6] = fb->buf[k];
    }

    if (strstr(data_check, "JFIF"))       // 깨진 파일 검출
    {
      //사이즈 전송부분
      szudp.begin(sizePort);          
      fsize = fb->len;
      szudp.beginSize(serverAddress, sizePort);             
      szudp.endSize(fsize);               // 사이즈 전송

      
     // Serial.println(fsize);
      
             
      do                                  // STOP 받는 부분
      {
        size_flag = okudp.StopPacket();
//#ifdef Debuging_mode        
        //Serial.println("STOP Wait..");          
        //if(size_flag == 2)
        //  Serial.println("STOPSIZE");  
        //else
        //  Serial.println(size_flag);    
//#endif           
      }while(size_flag!=2);

      do                                  // OK 받는 부분
      {
        size_flag = okudp.OKPacket();
//#ifdef Debuging_mode
        //Serial.println("STOP OK Wait..");                  
        //if(size_flag == 3)
        //  Serial.println("STOP OK");          
//#endif          
      }while(size_flag!=3);
      size_flag = 2;
                                          // ReSend 랑 END 받는부분 현재 죽여놓음
      if (flag == 3)
      {
        flag = 2;
        ReSe_flag = 1;
        esp_camera_fb_return(fb);
        delay(9);
        goto ReSend;
      }
      else if(flag == 0)
      {
        esp_camera_fb_return(fb);
        delay(9);
        goto END;
      } 
      

    static uint8_t buf[1460];
    int i = 0;
                                          //이미지 데이터 보내는 부분 
    if(fsize>1460)
    {
      send_count = 0;
      while(1)
      {
        size_t toRead = fsize;
        if (toRead > 1460)          
        {
          toRead = 1460;            
        }
        memset(buf, '\0', sizeof(buf));
        for (int j = 0; j < toRead; j++)
        {
          buf[j] = (fb->buf[i++]);
        }
        udp.begin(serverPort);
        udp.beginData(serverAddress, serverPort); 
        //Serial.println(udp.image_write((char*)buf,toRead));
        udp.image_write((char*)buf, toRead);                  // 버퍼에 담고
        udp.endData();                                        // Sendto 로 보냄
      
        fsize -= toRead;      
        //flag = okudp.resendPacket();                        //ReSend 받는 부분
        if(fsize == 0)
          break;
        if (flag == 3)
        {
          flag = 2;
          i = 0;
          ReSe_flag = 1;
          esp_camera_fb_return(fb);
          delay(9);
          goto ReSend;
        }
        else if(flag == 0)
        {
          esp_camera_fb_return(fb);
          delay(9);
          goto END;
        }
      }
    }
      udp.endPacket();                    // 0바이트 전송  // 이미지 전송 끝
      esp_camera_fb_return(fb);
      delay(9);
    }
    else                  
    {
      esp_camera_fb_return(fb);
      delay(9);
    }
}
  
  }

}
