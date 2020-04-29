#include <WiFi.h>

#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems

const char* ssid = "room201";
const char* password =  "room201201";

void setup() {
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
     Serial.println("");       
     Serial.print("localIP : ");
     Serial.print(WiFi.localIP());
     Serial.println("");    
        
}


void loop() {
  delay(1000);
}
