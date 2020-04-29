#include "lcd_1602.h"

void lcd1602_init(void)
{    
	LCD1602_sendbyte(LCD_CMD, LCD_mode_8_2_58);	// function mode setting 
	while(LCD1602_readBF());
	LCD1602_sendbyte(LCD_CMD, DISPLAY_ON);		// open the show 
	while(LCD1602_readBF());
	LCD1602_clear();			        // clear the screen 
	while(LCD1602_readBF());
	LCD1602_sendbyte(LCD_CMD, CMD_ENTRY);	// input mode setting 
}
void LCD1602_print(uint8_t Row, uint8_t Col, uint8_t *ptString)
{
	switch (Row)
	{
		case 2:
			LCD1602_sendbyte(LCD_CMD, LCD_L2 + Col); break;	// write line 2 of the specified column 
		default:
			LCD1602_sendbyte(LCD_CMD, LCD_L1 + Col); break;	// write the first line of the specified column 
	}
	while((*ptString)!='\0')		 // string is not the end of 
	{
		LCD1602_sendbyte(LCD_DATA, *ptString++);
	}
}

void LCD1602_sendbyte(uint8_t DatCmd, uint8_t du8)
{
	if (DatCmd == LCD_CMD)		// Command operation 
            LCD1602_RS_LOW();
	else
            LCD1602_RS_HIGH();
		
	LCD1602_RW_LOW();		// write
	LCD1602_EN_HIGH();
	LCD_DO(du8);		// write data
	HAL_Delay(2);
	LCD1602_EN_LOW();	
}

uint8_t LCD1602_readbyte(uint8_t DatCmd)
{
 	GPIO_InitTypeDef GPIO_InitStruct;
        
        uint8_t du8;

	if (DatCmd == LCD_CMD)		 // Command operation 
            LCD1602_RS_LOW();
	else
            LCD1602_RS_HIGH();
	
        LCD1602_RW_HIGH();               //read
        LCD1602_EN_HIGH(); 

	/* Change Port Direction As Input */
	GPIO_InitStruct.Pin = LCD_DATA_D7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(LCD_DATA_PORT, &GPIO_InitStruct);
	HAL_Delay(1);
        
	du8=HAL_GPIO_ReadPin(LCD_DATA_PORT, LCD_DATA_D7);          		 // read data or instructions 
		
	LCD1602_EN_LOW();	

	// restore the output data bus 
        GPIO_InitStruct.Pin = LCD_DATA_D7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(LCD_DATA_PORT, &GPIO_InitStruct);
	HAL_Delay(1);
        
	return du8;
}

uint8_t LCD1602_readBF(void)
{	  
	uint8_t busy;
	busy=LCD1602_readbyte(LCD_CMD);		// read back to the BF flag and address 

	return busy;
}

void LCD1602_clear(void)
{
	LCD1602_sendbyte(LCD_CMD,CMD_CLR);
	HAL_Delay(4); // clear the screen after the command is written, 2ms delay is necessary!!! 
}
