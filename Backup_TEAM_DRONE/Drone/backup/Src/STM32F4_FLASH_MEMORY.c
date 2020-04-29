#include "STM32F4_FLASH_MEMORY.h"

void Get_biases(TM_MPU9250_t* MPU9250)
{
    __IO uint32_t data32 = 0;
    uint32_t Address_sector5 = 0x08020000;
    uint32_t Address_sector6 = 0x08040000;

    data32 = *(__IO uint32_t*)(Address_sector5);  
    MPU9250->Accbiasx = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector5+4);  
    MPU9250->Accbiasy = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector5+8);  
    MPU9250->Accbiasz = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector5+12);  
    MPU9250->Gybiasx = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector5+16);  
    MPU9250->Gybiasy = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector5+20);  
    MPU9250->Gybiasz = (float)((int)(data32) / 1000.0f);
    
    data32 = *(__IO uint32_t*)(Address_sector6);  
    MPU9250->Magbiasx = (float)((int)(data32) / 10.0f);
    data32 = *(__IO uint32_t*)(Address_sector6+4);  
    MPU9250->Magbiasy = (float)((int)(data32) / 10.0f);
    data32 = *(__IO uint32_t*)(Address_sector6+8);  
    MPU9250->Magbiasz = (float)((int)(data32) / 10.0f);
    data32 = *(__IO uint32_t*)(Address_sector6+12);  
    MPU9250->Magscalex = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector6+16);  
    MPU9250->Magscaley = (float)((int)(data32) / 1000.0f);
    data32 = *(__IO uint32_t*)(Address_sector6+20);  
    MPU9250->Magscalez = (float)((int)(data32) / 1000.0f); 
}