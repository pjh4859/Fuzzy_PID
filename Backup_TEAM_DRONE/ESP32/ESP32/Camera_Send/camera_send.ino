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

//const char* ssid = "room201";
//const char* password =  "room201201";

const char* ssid = "KT_WLAN_D073";
const char* password =  "0000004733";

const char * serverAddress = "***.***.***.***";
const int serverPort = 8000;

WiFiUDP udp;

camera_fb_t * fb = NULL;
FILE* fp = NULL;


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
  if(WiFi.status() != WL_CONNECTED)  
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
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

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
  
  

  
}

void loop() {
  udp.begin(serverPort);
  
   fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.printf("fsize: %d\n",fb->len);
  fsize = fb->len;

  udp.beginSize(serverAddress,serverPort);    
  udp.endSize(fsize);
  
  static uint8_t buf[1460];
  int i = 0;
  while(fsize)
  {
    size_t toRead = fsize;
    if(toRead > 1460)
    {
      toRead = 1460;
    }
    memset(buf,'\0',sizeof(buf));
    for(int j=0;j<toRead;j++)
    {
      buf[j] = (fb->buf[i++]);
    }
    udp.beginData(serverAddress,serverPort);
    Serial.println(udp.image_write((char*)buf,toRead));
    udp.endData();
    fsize -= toRead;
  }
  udp.endPacket();
  i=0;
  esp_camera_fb_return(fb);
  delay(10); 
 
}
