#include <Arduino.h>
#include <Udp.h>
#include <cbuf.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <errno.h>
#include <string.h>
#include "Arduino.h"
#include "WiFiUdp.h"

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

#include "esp_camera.h"

#include <WiFi.h>



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
const char* password = "room201201";

//Roll, Pitch, Yaw UDP 서버 주소 및 포트
const char* serverAddress = "192.168.0.21";
const int serverPort = 9000;

//Camera Data UDP 서버 포트
const int cameraPort = 8000;

//check Data UDP 서버 포트
const int signalPort = 8008;

//
const int sizePort = 8001;

const int endPort = 8004;

WiFiUDP sudp;  // UDP Client Roll, Pitch, Yaw 송신

WiFiUDP cudp; // UDP Client Camera Data 송신
WiFiUDP oudp; // UDP Client Check Data 수신
WiFiUDP szudp; // UDP Client Size Data 송신
//WiFiUDP eudp; // UDP Client END Signal 수신

//TCP/IP 서버 생성
WiFiServer wifiServer(9000);
WiFiClient wifiClient;

camera_fb_t* fb = NULL;

#define BUFSIZE 65507

char message[BUFSIZE];
size_t fsize = 0;

static int socket_flag = 1;
static int thread_flag = 1;
static int camera_flag = 0;
static int size_flag = 1;
//static int data_flag = 0;
static int once_flag = 0;
static int flag = 0;
static int endflag = 0;


static int Roll = 0;
static int Pitch = 0;
static int Yaw = 0;

char udp_buff[200] = { 0, };
char tcp_buff[100] = { 0, };
char ack_buff[100];

unsigned long pre_sec = 0;
unsigned long sec = 0;
unsigned long dt = 0;

unsigned long pre_now = 0;
unsigned long now = 0;
unsigned long now_dt = 0;

void parsing_pid(char* pb);
void parsing_ppid(char* pb);

void parsing_thro(char* pb);

void parsing_setpoint(char* pb);

void* t_function(void* arg); // pid, setpoint, throtle

void* u_function(void* arg); // roll, pitch, yaw

void* c_function(void* arg); // Camera

void* e_function(void* arg); // End Signal

void* u_function(void* arg)                           //UDP 클라이언트 함수
{
  int i;
  pre_now = millis();
  while (1)
  {
    now = millis();
    now_dt += now - pre_now;
    pre_now = now;
    if (now_dt >= 2000)
    {
     if(socket_flag == 1)
     {
      if (Serial.available() > 0)                        // 데이터가 들어오면 들어온 데이터(Byte) 길이 return
      {
        char udp_buff[200] = { 0, };                      // 버퍼 초기화
        //memset(udp_buff,0,sizeof(udp_buff)); 
        for (i = 0; Serial.available() > 0; i++)
        {
          udp_buff[i] = Serial.read();                // 1Byte씩 데이터 버퍼에 저장
        }
        if (!(udp_buff[0] == '\0'))                    // 데이터가 있는지 확인
        {
          sudp.beginPacket(serverAddress, serverPort);
          sudp.printf("%s         \0", udp_buff);
          sudp.endPacket();
        }
      }
     }
     delay(10);
    }
  }
}

//char ch[255]={0,};
void* t_function(void* arg) // TCP/IP 서버 함수
{
  char ch[255] = { 0, };
  int i = 0;

  wifiServer.begin();

  while (1)
  {
    wifiClient = wifiServer.available(); // accept();
    while (wifiClient.connected()) // client 연결확인
    {

      if (wifiClient.available() > 0);
      {
        socket_flag = 2;
        delay(5);
        while (wifiClient.available() > 0) // 데이터 통신확인 연결되있을때부터 비교
        {
          ch[i++] = wifiClient.read();
        }
        if (strstr(ch, "T") != NULL)
        {
          parsing_thro(ch);
          memset(ch, '\0', sizeof(ch));
          memset(ack_buff, '\0', sizeof(ack_buff));
          ack_buff[0] = '1';
          wifiClient.write(ack_buff, 100);
        }
        else if (strstr(ch, "S") != NULL)
        {
          parsing_setpoint(ch);

          memset(ch, '\0', sizeof(ch));
          memset(ack_buff, '\0', sizeof(ack_buff));
          ack_buff[0] = '1';
          wifiClient.write(ack_buff, 10);
        }
        else
        {
          wifiClient.write(ch, 100);
        }

        if (strstr(ch, "PPP") != NULL)
        {
          parsing_ppid(ch);
          memset(ch, '\0', sizeof(ch));
        }
        else if (strstr(ch, "PP") != NULL)
        {
          parsing_pid(ch);
          memset(ch, '\0', sizeof(ch));
        }
        i = 0;
      }
      socket_flag = 1;
    }
    delay(10);
    wifiClient.stop();
  }
}

char flag_buff[20];

void* c_function(void* arg)                   //카메라 클라이언트 함수
{
END:  
  int signal_socket = -1;
  char buf[20] = { 0, };
  char c = '1';
  
  pinMode(33,LOW);
  
  struct sockaddr_in server_addr, client_addr;

  int fromlen = sizeof(client_addr);

  IPAddress remote_ip;
  uint16_t remote_port;

  signal_socket = socket(AF_INET, SOCK_DGRAM, 0);

  int yes = 1;
  setsockopt(signal_socket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  memset((char*)&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);//(in_addr_t)serverip;
  server_addr.sin_port = htons(signalPort);

  bind(signal_socket, (struct sockaddr*) & server_addr, sizeof(server_addr));
  fcntl(signal_socket, F_SETFL, O_NONBLOCK);

  cudp.begin(cameraPort);
  szudp.begin(sizePort);

  while (1)
  {
    if (flag == 0)
    {
      memset((char*)&client_addr, 0, sizeof(client_addr));
      memset(buf, 0, sizeof(buf));
      do
      {
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
      } while (!(strcmp(buf, "START") == 0));

      Serial.println(buf);
      pinMode(33,HIGH);
      remote_ip = IPAddress(client_addr.sin_addr.s_addr);
      remote_port = ntohs(client_addr.sin_port);

      memset(buf, 0, sizeof(buf));
      //dt = 0;
      //pre_sec = millis();
      do
      {
        oudp.beginPacket(remote_ip, remote_port);
        oudp.write(c);
        oudp.endPacket();
        //sec = millis();
        //dt = sec - pre_sec;
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
        //if (dt >= 5000)
        //{
        //  close(signal_socket);
        //  goto END;
        //}
      } while (!(strcmp(buf, "OK") == 0));
      flag = 1;
    }
    if (once_flag == 1)
    {
      memset(buf, 0, sizeof(buf));
      //dt = 0;
      //pre_sec = millis();
      do
      {
        //sec = millis();
        //dt = sec - pre_sec;
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
        if (flag == 0)
        {
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        /*
        else if (dt >= 5000)
        {
          endflag = 1;
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        */
      } while (!(strcmp(buf, "ReSendSize") == 0));
    
      remote_ip = IPAddress(client_addr.sin_addr.s_addr);
      remote_port = ntohs(client_addr.sin_port);

      oudp.beginPacket(remote_ip, remote_port);
      oudp.write(c);
      oudp.endPacket();

      memset(buf, 0, sizeof(buf));
      //dt = 0;
      //pre_sec = millis();
      do
      {
        //sec = millis();
        //dt = sec - pre_sec;
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
        if (flag == 0)
        {
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        /*
        else if (dt >= 5000)
        {
          endflag = 1;
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        */
      } while (!(strcmp(buf, "OK") == 0));
     
      remote_ip = IPAddress(client_addr.sin_addr.s_addr);
      remote_port = ntohs(client_addr.sin_port);

      oudp.beginPacket(remote_ip, remote_port);
      oudp.write(c);
      oudp.endPacket();
    }

    once_flag = 1;
    fb = esp_camera_fb_get();

    char data_check[5] = { 0, };
    for (int k = 6; k < 10; k++)
    {
      data_check[k - 6] = fb->buf[k];
    }

    if (strstr(data_check, "JFIF"))       // 깨진 파일 검출
    {
    ReSend:
      //사이즈 전송부분
      szudp.begin(sizePort);
      fsize = fb->len;
      szudp.beginSize(serverAddress, sizePort);
      szudp.endSize(fsize);

      recvfrom(signal_socket, buf, 20, 0, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
      if (strcmp(buf, "ReSend") == 0)
        goto ReSend;

      memset(buf, 0, sizeof(buf));
      //dt = 0;
      //pre_sec = millis();
      do
      {
        //sec = millis();
        //dt = sec - pre_sec;
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
        if (flag == 0)
        {
          esp_camera_fb_return(fb);
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        /*
        else if (dt >= 5000)
        {
          endflag = 1;
          esp_camera_fb_return(fb);
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        */
      } while (!(strcmp(buf, "STOPSIZE") == 0));

      remote_ip = IPAddress(client_addr.sin_addr.s_addr);
      remote_port = ntohs(client_addr.sin_port);

      oudp.beginPacket(remote_ip, remote_port);
      oudp.write(c);
      oudp.endPacket();
      memset(buf, 0, sizeof(buf));
      //dt = 0;
      //pre_sec = millis();
      do
      {
        //sec = millis();
        //dt = sec - pre_sec;
        recvfrom(signal_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
        if (flag == 0)
        {
          esp_camera_fb_return(fb);
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        /*
        else if (dt >= 5000)
        {
          endflag = 1;
          esp_camera_fb_return(fb);
          once_flag = 0;
          close(signal_socket);
          delay(9);
          goto END;
        }
        */
      } while (!(strcmp(buf, "OK") == 0));
    
      remote_ip = IPAddress(client_addr.sin_addr.s_addr);
      remote_port = ntohs(client_addr.sin_port);

      oudp.beginPacket(remote_ip, remote_port);
      oudp.write(c);
      oudp.endPacket();

      static uint8_t buf[1460];
      int i = 0;
      //이미지 데이터 보내는 부분 
      if (fsize > 1460)
      {
        while (1)
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
          cudp.begin(cameraPort);
          cudp.beginData(serverAddress, cameraPort);
          cudp.image_write((char*)buf, toRead);                  // 버퍼에 담고
          cudp.endData();

          fsize -= toRead;
          if (fsize == 0)
            break;
        }
        cudp.endPacket();                    // 0바이트 전송  // 이미지 전송 끝
        esp_camera_fb_return(fb);
        delay(9);
      }
    }
    else
    {
      esp_camera_fb_return(fb);
      delay(9);
    }
  }
}


void* e_function(void* arg)
{
end_thread:
  int End_socket = -1;
  char buf[20] = { 0, };
  
  struct sockaddr_in server_addr, client_addr;

  int fromlen = sizeof(client_addr);

  End_socket = socket(AF_INET, SOCK_DGRAM, 0);


  int yes = 1;
  setsockopt(End_socket, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));


  struct timeval timeout;
  timeout.tv_sec = 10;
  timeout.tv_usec = 0;
  int optlen = sizeof(timeout);

  setsockopt(End_socket, SOL_SOCKET, SO_RCVTIMEO, (void*)&timeout, (socklen_t)optlen);


  memset((char*)&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);//(in_addr_t)serverip;
  server_addr.sin_port = htons(endPort);

  bind(End_socket, (struct sockaddr*) & server_addr, sizeof(server_addr));
  fcntl(End_socket, F_SETFL, O_NONBLOCK);

  while (1)
  {
    //recvfrom(End_socket, buf, 20, MSG_DONTWAIT, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
    memset(buf,'\0',sizeof(buf));
    recvfrom(End_socket, buf, 20, 0, (struct sockaddr*) & client_addr, (socklen_t*)&fromlen);
    if (strcmp(buf, "END") == 0)
    {
      //endflag = 1;
    //if (endflag == 1)
    //{
      flag = 0;
      //endflag = 0;
      close(End_socket);
      delay(9);
      goto end_thread;
    //}
    }
    delay(9);
  }
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  delay(1000);
  pinMode(33,OUTPUT);
  // We start by connecting to a WiFi network
reboot:
  WiFi.begin(ssid, password);
  delay(2000);
  if (WiFi.status() != WL_CONNECTED) {
    goto reboot;
  }
  pinMode(33,LOW);
  //Serial.println(WiFi.localIP());

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

  pthread_t u_thread;
  pthread_t t_thread;
  pthread_t c_thread;
  pthread_t e_thread;
  pthread_attr_t attr;   //쓰레드 속성값

  int status;
  int a = 100;

  // '쓰레드 분리' 속성값 지정 : PTHREAD_CREATE_DETACHED
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

  pthread_create(&u_thread, &attr, u_function, (void*)&a);
  pthread_create(&t_thread, &attr, t_function, (void*)&a);
  pthread_create(&c_thread, &attr, c_function, (void*)&a);
  pthread_create(&e_thread, &attr, e_function, (void*)&a);

  pthread_attr_destroy(&attr);
}
void loop()
{
  delay(10000);
}

void parsing_pid(char* pb)
{
  int i;
  int Blink;
  char* R_PP, * R_PI, * R_PD, * R_RP, * R_RI, * R_RD, * R_YP, * R_YI, * R_YD;
  char buf = '0';

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

  //Serial.println("Start");
  if (strstr(R_RP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RP); i++)
    {
      Serial.print(R_RP[i]);
    }
    Blink = 6 - strlen(R_RP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('1');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_RI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RI); i++)
    {
      Serial.print(R_RI[i]);
    }
    Blink = 6 - strlen(R_RI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('2');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_RD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RD); i++)
    {
      Serial.print(R_RD[i]);
    }
    Blink = 6 - strlen(R_RD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('3');
    Serial.print('\0');
    delay(5);
  }

  /**************************************************/

  if (strstr(R_PP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PP); i++)
    {
      Serial.print(R_PP[i]);
    }
    Blink = 6 - strlen(R_PP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('4');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_PI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PI); i++)
    {
      Serial.print(R_PI[i]);
    }
    Blink = 6 - strlen(R_PI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('5');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_PD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PD); i++)
    {
      Serial.print(R_PD[i]);
    }
    Blink = 6 - strlen(R_PD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('6');
    Serial.print('\0');
    delay(5);
  }

  /**************************************************/

  if (strstr(R_YP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YP); i++)
    {
      Serial.print(R_YP[i]);
    }
    Blink = 6 - strlen(R_YP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('7');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_YI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YI); i++)
    {
      Serial.print(R_YI[i]);
    }
    Blink = 6 - strlen(R_YI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('8');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_YD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YD); i++)
    {
      Serial.print(R_YD[i]);
    }
    Blink = 6 - strlen(R_YD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('9');
    Serial.print('\0');
    delay(5);
  }
}

void parsing_ppid(char* pb)
{
  int i;
  int Blink;
  char* R_PP, * R_PI, * R_PD, * R_RP, * R_RI, * R_RD, * R_YP, * R_YI, * R_YD;
  char buf = '0';

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

  if (strstr(R_RP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RP); i++)
    {
      Serial.print(R_RP[i]);
    }
    Blink = 6 - strlen(R_RP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('a');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_RI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RI); i++)
    {
      Serial.print(R_RI[i]);
    }
    Blink = 6 - strlen(R_RI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('b');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_RD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_RD); i++)
    {
      Serial.print(R_RD[i]);
    }
    Blink = 6 - strlen(R_RD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('c');
    Serial.print('\0');
    delay(5);
  }

  /**************************************************/

  if (strstr(R_PP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PP); i++)
    {
      Serial.print(R_PP[i]);
    }
    Blink = 6 - strlen(R_PP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('d');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_PI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PI); i++)
    {
      Serial.print(R_PI[i]);
    }
    Blink = 6 - strlen(R_PI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('e');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_PD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_PD); i++)
    {
      Serial.print(R_PD[i]);
    }
    Blink = 6 - strlen(R_PD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('f');
    Serial.print('\0');
    delay(5);
  }

  /**************************************************/

  if (strstr(R_YP, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YP); i++)
    {
      Serial.print(R_YP[i]);
    }
    Blink = 6 - strlen(R_YP);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('g');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_YI, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YI); i++)
    {
      Serial.print(R_YI[i]);
    }
    Blink = 6 - strlen(R_YI);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('h');
    Serial.print('\0');
    delay(5);
  }

  if (strstr(R_YD, "0.000") == NULL)
  {
    for (i = 0; i < strlen(R_YD); i++)
    {
      Serial.print(R_YD[i]);
    }
    Blink = 6 - strlen(R_YD);

    for (i = 0; i < Blink; i++)
    {
      Serial.print(buf);
    }
    Serial.print('i');
    Serial.print('\0');
    delay(5);
  }
}

void parsing_thro(char* pb)
{

  int i;
  int Blink;
  char buf = '0';
  char* T, * temp;
  temp = strtok((char*)pb, "T");
  T = strtok(temp, ".");
  for (i = 0; i < strlen(T); i++)
  {
    Serial.print(T[i]);
  }
  Blink = 6 - strlen(T) - 1;
  Serial.print('.');
  for (i = 0; i < Blink; i++)
  {
    Serial.print(buf);
  }
  Serial.print('T');
  Serial.print('\0');
  delay(5);
}

void parsing_setpoint(char* pb)
{
  int i;
  int Blink;
  char buf = '0';
  char* P, * R, * Y, * temp, * ptemp, * rtemp, * ytemp;
  temp = strtok((char*)pb, "S");
  ptemp = strtok(temp, "P");
  rtemp = strtok(NULL, "R");
  ytemp = strtok(NULL, "Y");

  P = strtok(ptemp, ".");
  R = strtok(rtemp, ".");
  Y = strtok(ytemp, ".");

  for (i = 0; i < strlen(R); i++)
  {
    Serial.print(R[i]);
  }
  Blink = 6 - strlen(R) - 1;
  Serial.print('.');
  for (i = 0; i < Blink; i++)
  {
    Serial.print(buf);
  }
  Serial.print('R');
  Serial.print('\0');
  delay(5);

  for (i = 0; i < strlen(P); i++)
  {
    Serial.print(P[i]);
  }
  Blink = 6 - strlen(P) - 1;
  Serial.print('.');
  for (i = 0; i < Blink; i++)
  {
    Serial.print(buf);
  }
  Serial.print('P');
  Serial.print('\0');
  delay(5);

  for (i = 0; i < strlen(Y); i++)
  {
    Serial.print(Y[i]);
  }
  Blink = 6 - strlen(Y) - 1;
  Serial.print('.');
  for (i = 0; i < Blink; i++)
  {
    Serial.print(buf);
  }
  Serial.print('Y');
  Serial.print('\0');
  delay(5);
}
