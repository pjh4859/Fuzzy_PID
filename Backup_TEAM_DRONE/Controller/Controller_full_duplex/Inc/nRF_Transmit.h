#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"


void nRF24_Transmit(float* temp, uint8_t adr_value);            // PID �� ����
void nRF24_Transmit_ADC(int8_t* temp,uint8_t adr_value);
void nRF24_Transmit_Set_Point(int* temp, uint8_t adr_value);    // Roll, Pitch, Yaw SetPoint ����
void nRF24_Transmit_ASCII(uint8_t adr_value);                   // Qick Test ����
void nRF24_Transmit_Mode_Change(uint8_t adr_value);             // Debugging Drive mode ����
void nRF24_Transmit_Status();                                   // Transmit status Check