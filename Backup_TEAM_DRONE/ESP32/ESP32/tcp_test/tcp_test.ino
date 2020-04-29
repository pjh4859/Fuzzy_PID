#include <WiFi.h>
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems

const char* ssid = "room201";
const char* password =  "room201201";
 
WiFiServer wifiServer(9000);
WiFiClient wifiClient;

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
}
int i = 0;
void loop() 
{
 wifiClient = wifiServer.available(); // accept(); 연결요청 대기
 
   while(wifiClient.connected()) // client 연결확인 if 도전?
    {
     char ch[100];
     memset(ch,'\0',sizeof(ch));
     while(wifiClient.available()>0) // 데이터 통신확인 연결되있을때부터 비교
     {
     ch[i++] = wifiClient.read();
     }
     Serial.printf("%s",ch);
    }
    wifiClient.stop();
/*
    while(wifiClient.connected()) // client 연결확인 if 도전?
    {
     while(wifiClient.available()>0) // 데이터 통신확인 연결되있을때부터 비교
     {
     char c = wifiClient.read();
     Serial.print(c);
     }
     delay(1000);
    }
    wifiClient.stop();
*/
}
