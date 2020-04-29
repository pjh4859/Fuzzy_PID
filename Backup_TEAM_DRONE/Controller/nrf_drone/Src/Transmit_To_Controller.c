//#include "main.h"
//#include "Transmit_To_Controller.h"
//#include <stdio.h>
//TM_NRF24L01_Transmit_Status_t transmissionStatus;
//void Transmit_to_Controller(__PID* pid)
//{
//  uint8_t dataOut[8]="hi";
//  //uint8_t test_buf[8]={'\0'};
//  
//  int No_Connection=0;
//  
// 
// // sprintf((char*)test_buf,"%.3f",*(pid->iKp));
////  sprintf((char*)test_buf[1],"%.3f",pid->iKp[1]);
////  sprintf((char*)test_buf[2],"%.3f",pid->iKp[2]);
//
//  //dataOut[0] = (uint8_t)test_buf[0];
//  TM_NRF24L01_Transmit(dataOut);
//   do {
//  /* Get transmission status */
//      transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
//      if(!(transmissionStatus == TM_NRF24L01_Transmit_Status_Ok))
//      {
//        No_Connection++;
//        TM_NRF24L01_Transmit(dataOut);
//        nRF24_Transmit_Status();
//        if(No_Connection >= 1000)
//        {
//          //printf("No Connection!\r\n");
//          break;
//        }
//      }
//  }while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
//  memset(dataOut,'\0',8); // dataOut 메모리 초기화
//  
//  
//
////   dataOut[0] = pid->iKi[0];
////   dataOut[1] = pid->iKi[1];
////   dataOut[2] = pid->iKi[2];
////   
////   TM_NRF24L01_Transmit(dataOut);
////   do {
////   /* Get transmission status */
////       transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
////       No_Connection++;
////       if(No_Connection >= 1000)
////       {
////         printf("No Connection!\r\n");
////         break;
////       }
////   } 
////   while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
////   memset(dataOut,'\0',32); // C_buff 메모리 초기화
////  
////  
////   dataOut[0] = pid->iKd[0];
////   dataOut[1] = pid->iKd[1];
////   dataOut[2] = pid->iKd[2];
////   
////   TM_NRF24L01_Transmit(dataOut);
////   do {
////   /* Get transmission status */
////       transmissionStatus = TM_NRF24L01_GetTransmissionStatus();
////       No_Connection++;
////       if(No_Connection >= 1000)
////       {
////         printf("No Connection!\r\n");
////         break;
////       }
////   } 
////   while (transmissionStatus == TM_NRF24L01_Transmit_Status_Sending);
////   memset(dataOut,'\0',32); // C_buff 메모리 초기화
//
//  
//}
