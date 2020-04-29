#ifndef  __lcd_1602_H
#define  __lcd_1602_H

#include "include.h"

#define	LCD_CMD		0
#define	LCD_DATA	1
/*
 * LCD Command Set
 */
#define CMD_CLR     0X01
#define CMD_HOME    0X02
#define CMD_ENTRY   0x06	
#define CMD_ONOFF   0X08
#define CMD_SHIFT   0X10
#define CMD_FUNC    0X20
#define CGRAM_ADDR  0X40
#define DDRAM_ADDR  0X80

/*
 * Function set
 */
#define LCD_mode_8_2_58     0x38 //8bit 2line 5x8font
#define LCD_L1		    0x80		 
#define LCD_L2	            0xC0
/*
 * Entry Mode Set
 */
#define DISP_SHIFT  0X01
#define CURSOR_INC  0X02

/*
 * Display On/Off Control
 */
#define CURSOR_ON   0X0E
#define DISPLAY_ON  0X0C
#define DISPLAY_OFF 0X08


#define LCD_CH_LEN      16
#define LCD_START_LINE1 0x00
#define LCD_START_LINE2 0x40

/*
 * Cursor or Display Shift
 */
#define CURSOR_R    0X04
#define DISPLAY_SHIFT   0X08

///////////////////////RS,RW,EN Setup/////////////////////////////////////
#define LCD1602_RS_PORT GPIOB
#define LCD1602_RW_PORT GPIOC
#define LCD1602_EN_PORT GPIOB 
   
#define LCD1602_RS	GPIO_PIN_2	/* Low -> CMD & High -> Data */
#define LCD1602_RW	GPIO_PIN_5	/* Low -> Write & High -> Read */
#define LCD1602_EN	GPIO_PIN_1	/* Hight To Low */

#define LCD1602_RS_HIGH()     	HAL_GPIO_WritePin(LCD1602_RS_PORT, LCD1602_RS,GPIO_PIN_SET)  
#define LCD1602_RS_LOW()     	HAL_GPIO_WritePin(LCD1602_RS_PORT, LCD1602_RS,GPIO_PIN_RESET)  
#define LCD1602_RW_HIGH()     	HAL_GPIO_WritePin(LCD1602_RW_PORT, LCD1602_RW,GPIO_PIN_SET)  
#define LCD1602_RW_LOW()     	HAL_GPIO_WritePin(LCD1602_RW_PORT, LCD1602_RW,GPIO_PIN_RESET)  
#define LCD1602_EN_HIGH()     	HAL_GPIO_WritePin(LCD1602_EN_PORT, LCD1602_EN,GPIO_PIN_SET)  
#define LCD1602_EN_LOW()     	HAL_GPIO_WritePin(LCD1602_EN_PORT, LCD1602_EN,GPIO_PIN_RESET)  
///////////////////////RS,RW,EN Setup/////////////////////////////////////

#define LCD_DATA_D0_num 8
#define LCD_DATA_PORT   GPIOB
#define LCD_DATA_D0     GPIO_PIN_10    
#define LCD_DATA_D1     GPIO_PIN_4
#define LCD_DATA_D2     GPIO_PIN_5
#define LCD_DATA_D3     GPIO_PIN_3
#define LCD_DATA_D4     GPIO_PIN_15
#define LCD_DATA_D5     GPIO_PIN_14
#define LCD_DATA_D6     GPIO_PIN_13
#define LCD_DATA_D7     GPIO_PIN_11

#define LCD_DO(x)	LCD_DATA_PORT->BSRR = 0xFF000000 | ( x<<LCD_DATA_D0_num)

//void Init_LCD_Port(void);

void lcd1602_init(void);
void LCD1602_print(uint8_t Row, uint8_t  Col,uint8_t *ptString);
uint8_t LCD1602_readbyte(uint8_t  DatCmd);
void LCD1602_sendbyte(uint8_t DatCmd, uint8_t dByte);
void LCD1602_clear(void);
uint8_t LCD1602_readBF(void);

#endif 