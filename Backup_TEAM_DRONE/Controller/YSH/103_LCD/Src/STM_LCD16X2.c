/*
	Library: 				LCD 16X2 - 
	Written by:  		Mohamed Yaqoob
	Date Written:		04/12/2017
	Description:		This is a library for the standard 16X2 LCD display, for the STM32 MCUs based on HAL libraries. 
									It perfroms the basic Text/Number printing to your 16X2 LCD, in 8 bits and 4 bits modes of operation.

	References**:
									This was written based on the open source Arduino LiquidCrystal library 
									and by referring to the DATASHEET of the LCD16X2, also with the help of 
									the following YouTube tutorials on LCD 16X2:
									(1): 'RC Tractor Guy' YouTube tutorial on the following link:
											 https://www.youtube.com/watch?v=efi2nlsvbCI 
									(2): 'Explore Embedded' YouTube tutorial on the following link:
											 https://www.youtube.com/watch?v=YDJISiPUdA8
*/


//***** Header files *****//
#include "STM_LCD16X2.h"

//***** Variables *****//
static const uint32_t writeTimeConstant = 10;
static uint8_t mode_8_4_I2C = 1;
static GPIO_TypeDef* PORT_RS_and_B;							  // RS and E PORT
static uint16_t PIN_RS, PIN_B;									  // RS and E pins
static GPIO_TypeDef* PORT_LSB;										// LSBs D0, D1, D2 and D3 PORT
static uint16_t D0_PIN, D1_PIN, D2_PIN, D3_PIN;	// LSBs D0, D1, D2 and D3 pins
static GPIO_TypeDef* PORT_MSB;										// MSBs D5, D6, D7 and D8 PORT
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;	// MSBs D5, D6, D7 and D8 pins

static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;

//***** Functions definitions *****//
//Private functions
//1) Enable EN pulse
static void LCD1602_EnablePulse(void)
{
	HAL_GPIO_WritePin(PORT_RS_and_B, PIN_B, GPIO_PIN_SET);
	LCD1602_TIM_MicorSecDelay(writeTimeConstant);
	HAL_GPIO_WritePin(PORT_RS_and_B, PIN_B, GPIO_PIN_RESET);
	LCD1602_TIM_MicorSecDelay(60);
}
//2) RS control
static void LCD1602_RS(bool state)
{
	if(state) HAL_GPIO_WritePin(PORT_RS_and_B, PIN_RS, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(PORT_RS_and_B, PIN_RS, GPIO_PIN_RESET);
}

//3) Write Parallel interface
static void LCD1602_write(uint8_t byte)
{
	uint8_t LSB_nibble = byte&0xF, MSB_nibble = (byte>>4)&0xF;
	 
	if(mode_8_4_I2C == 1)		//8bits mode
	{
		//write data to output pins
		//LSB data
		HAL_GPIO_WritePin(PORT_LSB, D0_PIN, (GPIO_PinState)(LSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_LSB, D1_PIN, (GPIO_PinState)(LSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_LSB, D2_PIN, (GPIO_PinState)(LSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_LSB, D3_PIN, (GPIO_PinState)(LSB_nibble&0x8));
		//MSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
		//Write the Enable pulse
		LCD1602_EnablePulse();
	}
	else if(mode_8_4_I2C == 2)	//4 bits mode
	{
		//write data to output pins
		//MSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(MSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(MSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(MSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(MSB_nibble&0x8));
		//Write the Enable pulse
		LCD1602_EnablePulse();
		
		//LSB data
		HAL_GPIO_WritePin(PORT_MSB, D4_PIN, (GPIO_PinState)(LSB_nibble&0x1));
		HAL_GPIO_WritePin(PORT_MSB, D5_PIN, (GPIO_PinState)(LSB_nibble&0x2));
		HAL_GPIO_WritePin(PORT_MSB, D6_PIN, (GPIO_PinState)(LSB_nibble&0x4));
		HAL_GPIO_WritePin(PORT_MSB, D7_PIN, (GPIO_PinState)(LSB_nibble&0x8));
		//Write the Enable pulse
		LCD1602_EnablePulse();
	}
}
//4) Microsecond delay functions
static void LCD1602_TIM_Config(void)
{
	RCC_ClkInitTypeDef myCLKtypeDef;
	uint32_t clockSpeed;
	uint32_t flashLatencyVar;
	HAL_RCC_GetClockConfig(&myCLKtypeDef, &flashLatencyVar);
	if(myCLKtypeDef.APB1CLKDivider == RCC_HCLK_DIV1)
	{
		clockSpeed = HAL_RCC_GetPCLK1Freq();
	}
	else
	{
		clockSpeed = HAL_RCC_GetPCLK1Freq()*2;
	}
	clockSpeed *= 0.000001;
	
	//Enable clock for TIM2 timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // 0x1
	//Set the mode to Count up
	TIM3->CR1 &= ~(0x0010);
	//Enable Update Event
	TIM3->CR1 &= ~(0x0001);
	//Update request source 
	TIM3->CR1 &= ~(1UL << 2);
	// Set bit 3 High to enable One-Pulse mode
	TIM3->CR1 |= (1UL << 3);				  
	//Set the Prescalar
	TIM3->PSC = clockSpeed-1;
	//Set and Auto-Reload Value to delay the timer 1 sec
	TIM3->ARR = 10-1; 								// The Flag sets when overflows
	//Event generation handling to reset the counter
	TIM3->EGR = 1; 					//Update generate auto
	TIM3->SR &= ~(0x0001);	//Clear Update interrupt flag
}
static void LCD1602_TIM_MicorSecDelay(uint32_t uSecDelay)
{
	TIM3->ARR = uSecDelay-1;
	TIM3->SR &= ~(0x0001);  // Clear UEV flag
	TIM3->CR1 |= 1UL;
	while((TIM3->SR&0x0001) != 1);
}
//5) Write command
static void LCD1602_writeCommand(uint8_t command)
{
	//Set RS to 0
	LCD1602_RS(false);
	//Call low level write parallel function
	LCD1602_write(command);
}
//6) Write 8 bits data
static void LCD1602_writeData(uint8_t data)
{
	//Set RS to 1
	LCD1602_RS(true);
	//Call low level write parallel function
	LCD1602_write(data);
}
//7) Write 4 bits command, *FOR 4 BITS MODE ONLY*
static void LCD1602_write4bitCommand(uint8_t nibble)
{
	uint8_t LSB_nibble = nibble&0xF;
	//Set RS to 0
	LCD1602_RS(false);
	//LSB data
	HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (GPIO_PinState)(LSB_nibble&0x1));
	HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (GPIO_PinState)(LSB_nibble&0x2));
	HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (GPIO_PinState)(LSB_nibble&0x4));
	HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (GPIO_PinState)(LSB_nibble&0x8));
	//Write the Enable pulse
	LCD1602_EnablePulse();
}

//Public functions
//1) LCD begin 8 bits function
void LCD1602_Begin8BIT(GPIO_TypeDef* PORT_RS_B, uint16_t RS, uint16_t B, GPIO_TypeDef* PORT_LSBs0to3, uint16_t D0, uint16_t D1, uint16_t D2, uint16_t D3, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7)
{
	//Set GPIO Ports and Pins data
	PORT_RS_and_B = PORT_RS_B;
	PIN_RS = RS;
	PIN_B = B;
	PORT_LSB = PORT_LSBs0to3;
	D0_PIN = D0;
	D1_PIN = D1;
	D2_PIN = D2;
	D3_PIN = D3;
	PORT_MSB = PORT_MSBs4to7;
	D4_PIN = D4;
	D5_PIN = D5;
	D6_PIN = D6;
	D7_PIN = D7;
	//Initialise microsecond timer
	LCD1602_TIM_Config();
	//Set the mode to 8 bits
	mode_8_4_I2C = 1;
	//Function set variable to 8 bits mode
	FunctionSet = 0x38;
	
	//Initialise LCD
	//1. Wait at least 15ms
	HAL_Delay(20);
	//2. Attentions sequence
	LCD1602_writeCommand(0x30);
	HAL_Delay(5);
	LCD1602_writeCommand(0x30);
	HAL_Delay(1);
	LCD1602_writeCommand(0x30);
	HAL_Delay(1);
	//3. Function set; Enable 2 lines, Data length to 8 bits
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N | LCD_FUNCTION_DL);
	//4. Display control (Display ON, Cursor ON, blink cursor)
	LCD1602_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
	//5. Clear LCD and return home
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	HAL_Delay(2);
}
//2) LCD begin 4 bits function
void LCD1602_Begin4BIT(GPIO_TypeDef* PORT_RS_B, uint16_t RS, uint16_t B, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7)
{
	//Set GPIO Ports and Pins data
	PORT_RS_and_B = PORT_RS_B;
	PIN_RS = RS;
	PIN_B = B;
	PORT_MSB = PORT_MSBs4to7;
	D4_PIN = D4;
	D5_PIN = D5;
	D6_PIN = D6;
	D7_PIN = D7;
	//Initialise microsecond timer
	LCD1602_TIM_Config();
	//Set the mode to 4 bits
	mode_8_4_I2C = 2;
	//Function set variable to 4 bits mode
	FunctionSet = 0x28;
	
	//Initialise LCD
	//1. Wait at least 15ms
	HAL_Delay(20);
	//2. Attentions sequence
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(5);
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(1);
	LCD1602_write4bitCommand(0x3);
	HAL_Delay(1);
	LCD1602_write4bitCommand(0x2);  //4 bit mode
	HAL_Delay(1);
	//3. Display control (Display ON, Cursor ON, blink cursor)
	LCD1602_writeCommand(LCD_DISPLAYCONTROL | LCD_DISPLAY_B | LCD_DISPLAY_C | LCD_DISPLAY_D);
	//4. Clear LCD and return home
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	HAL_Delay(3);
	//4. Function set; Enable 2 lines, Data length to 8 bits
	LCD1602_writeCommand(LCD_FUNCTIONSET | LCD_FUNCTION_N);
	HAL_Delay(3);
}
//3) LCD print string
void LCD1602_print(char string[])
{
	for(uint8_t i=0;  i< 16 && string[i]!=NULL; i++)
	{
		LCD1602_writeData((uint8_t)string[i]);
	}
}
//4) set cursor position
void LCD1602_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		LCD1602_writeCommand(maskData);
	}
	else
	{
		maskData |= (0xc0);
		LCD1602_writeCommand(maskData);
	}
}	
void LCD1602_1stLine(void)
{
	LCD1602_setCursor(1,1);
}
void LCD1602_2ndLine(void)
{
	LCD1602_setCursor(2,1);
}
//5) Enable two lines
void LCD1602_TwoLines(void)
{
	FunctionSet |= (0x08);
	LCD1602_writeCommand(FunctionSet);
}
void LCD1602_OneLine(void)
{
	FunctionSet &= ~(0x08);
	LCD1602_writeCommand(FunctionSet);
}
//6) Cursor ON/OFF
void LCD1602_noCursor(void)
{
	DisplayControl &= ~(0x02);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_cursor(void)
{
	DisplayControl |= (0x02);
	LCD1602_writeCommand(DisplayControl);
}
//7) Clear display
void LCD1602_clear(void)
{
	LCD1602_writeCommand(LCD_CLEARDISPLAY);
	HAL_Delay(3);
}
//8) Blinking cursor
void LCD1602_noBlink(void)
{
	DisplayControl &= ~(0x01);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_blink(void)
{
	DisplayControl |= 0x01;
	LCD1602_writeCommand(DisplayControl);
}
//9) Display ON/OFF
void LCD1602_noDisplay(void)
{
	DisplayControl &= ~(0x04);
	LCD1602_writeCommand(DisplayControl);
}
void LCD1602_display(void)
{
	DisplayControl |= (0x04);
	LCD1602_writeCommand(DisplayControl);
}
//10) Shift Display, right or left
void LCD1602_shiftToRight(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x1c);
	}
}
void LCD1602_shiftToLeft(uint8_t num)
{
	for(uint8_t i=0; i<num;i++)
	{
		LCD1602_writeCommand(0x18);
	}
}

//********** Print numbers to LCD **********//
//1. Integer
void LCD1602_PrintInt(int number)
{
	char numStr[16];
	sprintf(numStr,"%d", number);
	LCD1602_print(numStr);
}
//2. Float
void LCD1602_PrintFloat(float number, int decimalPoints)
{
	char numStr[16];
	sprintf(numStr,"%.*f",decimalPoints, number);
	LCD1602_print(numStr);
}
