#include <WiFi.h>
#include <WiFiUdp.h>
#include <stdio.h>
#include <string.h>
#pragma warning(disable:4996)
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems


// WiFi network name and password:
const char * networkName = "room201";
const char * networkPswd = "room201201";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * serverAddress = "192.168.0.23";
const int serverPort = 9000;

WiFiUDP udp;

void setup() 
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
    Serial.begin(115200);
    delay(10);

// We start by connecting to a WiFi network
reboot:

    WiFi.begin(networkName, networkPswd);

    while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        if(WiFi.status() != WL_CONNECTED)  {
          goto reboot;
        }
    }
}
int i;
void loop()
{
   char buff[255]={0,};
   for(i=0;Serial.available()>0;i++)
   {
     buff[i] = Serial.read();
     if(i==254)
       memset(buff,0,sizeof(buff));     
   }
  if(!(buff[0] == '\0'))
  {
  udp.beginPacket(serverAddress,serverPort);    
  udp.printf("%s       \0",buff);
  udp.endPacket();
  } 
}
