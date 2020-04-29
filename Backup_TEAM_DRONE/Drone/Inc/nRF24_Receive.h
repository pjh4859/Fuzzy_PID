#ifndef __NRF_H__
#define __NRF_H__


#include "tm_stm32_nrf24l01.h"
#include "tm_stm32_delay.h"
#include "pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


void NRF24_Data_save(int* Throttle,float temp, int temp_int, int value, __PID* pid, float* setting_angle,float Euler_angle_yaw);
void NRF24_Receive(int* Throttle,float temp, int temp_int,__PID*  pid,float* setting_angle, float Euler_angle_yaw);


#endif