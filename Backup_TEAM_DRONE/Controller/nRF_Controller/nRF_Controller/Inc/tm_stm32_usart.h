/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.net
 * @link    http://stm32f4-discovery.net/2015/07/hal-library-07-usart-for-stm32fxxx
 * @version v1.2.1
 * @ide     Keil uVision
 * @license MIT
 * @brief   USART Library for STM32Fxxx with receive interrupt
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef TM_USART_H
#define TM_USART_H 121

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32Fxxx_HAL_Libraries
 * @{
 */

/**
 * @defgroup TM_USART
 * @brief    TM USART Library for STM32Fxxx - http://stm32f4-discovery.net/2015/07/hal-library-07-usart-for-stm32fxxx
 * @{
 *
 * <b>Library works for all 8 U(S)ARTs which are supported on STM32Fxxx devices.</b>
 *
 * \par Supported devices
 * 
 * Library supported STM32F4xx and STM32F7xx devices as well as STM32F0xx.
 *
 * - STM32F4xx and STM32F7xx has the same U(S)ART names: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8 where possible.
 * - STM32F0xx has a little bit different names: USART1, USART2, USART3, USART4, USART5, USART6, USART7, USART8. ALl of them are <b>USART</b> modules.
 *
 * @note  All parameters in this library uses the same names as USART name in STM32 series.
 *
 * \par USART receive interrupt handlers
 *
 * Every USART channel has it's own receive interrupt which stores incoming data into cyclic buffer.
 * If you want to use your own receive handler, then you have to open defines.h files and set a define.
\code
//Use custom IRQ Receive handler
//Change X with possible U(S)ARTs: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8
#define TM_X_USE_CUSTOM_IRQ
\endcode
 *
 * After you set define, you have to create a function, which will handle custom request
 *
\code
//Change X with possible U(S)ARTs: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8
//Parameter c is a received character
void TM_X_ReceiveHandler(uint8_t c) {
   //Do your stuff here when byte is received
}
\endcode
 *
 * @note If you use custom receive interrupt handler, then incoming data is not stored in internal buffer
 *
 * \par USART Internal cyclic buffer
 *
 * In your project you can set internal cyclic buffer length, default is 32Bytes, with:
 *
\code
//Set buffer sie for all buffers
#define TM_USART_BUFFER_SIZE number_of_bytes
\endcode
 *
 * in your project's defines.h file. This will set default length for each buffer.
 * So if you are working with F429 (it has 8 U(S)ARTs) then you will use 8kB RAM if 
 * you set define above to 1024.
 *
 * If you don't change anything, then all USART's have buffer length of value, stored in
 * <code>TM_USART_BUFFER_SIZE</code> define. If you want let's say just for USART1 to be 1kB, but others default value,
 * you can add define below in defines.h file:
 *
\code
//Buffer length for USART1 is 1kB, for others is still TM_USART_BUFFER_SIZE
#define TM_USART1_BUFFER_SIZE 1024
\endcode
 *
 * Other possible settings are (for other U(S)ARTs):
 *   - TM_USART1_BUFFER_SIZE
 *   - TM_USART2_BUFFER_SIZE
 *   - TM_USART3_BUFFER_SIZE
 *   - TM_UART4_BUFFER_SIZE
 *   - TM_UART5_BUFFER_SIZE
 *   - TM_USART6_BUFFER_SIZE
 *   - TM_UART7_BUFFER_SIZE
 *   - TM_UART8_BUFFER_SIZE
 *
 *   - STM32F0xx related:
 *   - TM_USART4_BUFFER_SIZE
 *   - TM_USART5_BUFFER_SIZE
 *   - TM_USART7_BUFFER_SIZE
 *   - TM_USART8_BUFFER_SIZE
 *	
 * \par Custom string delimiter for @ref TM_USART_Gets() function
 * 
 * By default, LF (Line Feed) character was used, but now you can select custom character using @ref TM_USART_SetCustomStringEndCharacter() function.
 *
 * \par Pinout
 *
\verbatim
             |PINSPACK 1     |PINSPACK 2     |PINSPACK 3	
U(S)ARTX     |TX     RX      |TX     RX      |TX     RX

//STM32F4xx and STM32F7xx
USART1       |PA9    PA10    |PB6    PB7     |PA9    PB7
USART2       |PA2    PA3     |PD5    PD6     |-      -
USART3       |PB10   PB11    |PC10   PC11    |PD8    PD9
UART4        |PA0    PA1     |PC10   PC11    |-      -
UART5        |PC12   PD2     |-      -       |-      -
USART6       |PC6    PC7     |PG14   PG9     |-      -
UART7        |PE8    PE7     |PF7    PF6     |-      -
UART8        |PE1    PE0     |-      -       |-      -

//Additions for STM32F0xx
USART4       |PA0    PA1     |PC10   PC11    |PE8    PE9
USART5       |PB3    PB4     |PC12   PD2     |PE10   PE11
USART7       |PC0    PC1     |PC6    PC7     |PF2    PF3
USART8       |PC2    PC3     |PC8    PC9     |PD13   PD14
\endverbatim
 *
 * In case these pins are not good for you, you can use
 * TM_USART_PinsPack_Custom in function and callback function will be called,
 * where you can initialize your custom pinout for your USART peripheral
 *
 * \par Change USART default operation modes
 * 
 * In this section, you can change USART functionality.
 * Do this only in case you know what are you doing!
 * 
 * Open \ref defines.h file, copy define you want to change and fill settings
\code
//Change X with possible U(S)ARTs: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8, for STM32F0xx additions: USART4, USART5, USART7, USART8
//Set flow control
#define TM_X_HARDWARE_FLOW_CONTROL    TM_USART_HardwareFlowControl_None
//Set mode
#define TM_X_MODE                     USART_MODE_TX_RX
//Set parity
#define TM_X_PARITY                   USART_PARITY_NONE
//Set stopbits
#define TM_X_STOP_BITS                USART_STOPBITS_1
//Set USART datasize
#define TM_X_WORD_LENGTH              UART_WORDLENGTH_8B
\endcode
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
   - First release
   
 Version 1.1
  - September 03, 2015
  - Added support for buffers which now requires separate library for USART
   
 Version 1.2
  - December 26, 2015
  - On reinitialization USART with other baudrate, USART didn't work properly and needs some time to start.
  - With forcing register reset this has been fixed
\endverbatim
 *
 * \b Dependencies
 *
\verbatim
 - STM32Fxxx HAL
 - STM32Fxxx HAL UART
 - attributes.h
 - defines.h
 - TM GPIO
 - TM BUFFER
\endverbatim
 */
#include "stm32f4xx_hal.h"
#include "attributes.h"
//#include "defines.h"
#include "tm_stm32_gpio.h"
#include "tm_stm32_buffer.h"

/**
 * @defgroup TM_USART_Typedefs
 * @brief    USART Typedefs
 * @{
 */
 
/**
 * @brief  USART PinsPack enumeration to select pins combination for USART
 */
typedef enum {
	TM_USART_PinsPack_1 = 0x00, /*!< Select PinsPack1 from Pinout table for specific USART */
	TM_USART_PinsPack_2,        /*!< Select PinsPack2 from Pinout table for specific USART */
	TM_USART_PinsPack_3,        /*!< Select PinsPack3 from Pinout table for specific USART */
	TM_USART_PinsPack_Custom    /*!< Select custom pins for specific USART, callback will be called, look @ref TM_USART_InitCustomPinsCallback */
} TM_USART_PinsPack_t;

/**
 * @brief  USART Hardware flow control selection
 * @note   Corresponsing pins must be initialized in case you don't use "None" options
 */
typedef enum {
	TM_USART_HardwareFlowControl_None = UART_HWCONTROL_NONE,      /*!< No flow control */
	TM_USART_HardwareFlowControl_RTS = UART_HWCONTROL_RTS,        /*!< RTS flow control */
	TM_USART_HardwareFlowControl_CTS = UART_HWCONTROL_CTS,        /*!< CTS flow control */
	TM_USART_HardwareFlowControl_RTS_CTS = UART_HWCONTROL_RTS_CTS /*!< RTS and CTS flow control */
} TM_USART_HardwareFlowControl_t;

/**
 * @}
 */

/**
 * @defgroup TM_USART_Macros
 * @brief    USART default values for defines
 * @{
 *
 * All values can be overwritten in your project's defines.h file.
 * 
 * Do this only in case you know what are you doing.
 */

/* Default buffer size for each USART */
#ifndef TM_USART_BUFFER_SIZE
#define TM_USART_BUFFER_SIZE 				32
#endif

/* Set default buffer size for specific USART if not set by user */
#ifndef TM_USART1_BUFFER_SIZE
#define TM_USART1_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART2_BUFFER_SIZE
#define TM_USART2_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART3_BUFFER_SIZE
#define TM_USART3_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_UART4_BUFFER_SIZE
#define TM_UART4_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_UART5_BUFFER_SIZE
#define TM_UART5_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART6_BUFFER_SIZE
#define TM_USART6_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_UART7_BUFFER_SIZE
#define TM_UART7_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_UART8_BUFFER_SIZE
#define TM_UART8_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif

/* STM32F0xx related */
#ifndef TM_USART4_BUFFER_SIZE
#define TM_USART4_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART5_BUFFER_SIZE
#define TM_USART5_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART7_BUFFER_SIZE
#define TM_USART7_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif
#ifndef TM_USART8_BUFFER_SIZE
#define TM_USART8_BUFFER_SIZE				TM_USART_BUFFER_SIZE
#endif

/* NVIC Global Priority */
#ifndef USART_NVIC_PRIORITY
#define USART_NVIC_PRIORITY					0x06
#endif

/* U(S)ART settings, can be changed in your defines.h project file */
/* USART1 default settings */
#ifndef TM_USART1_HARDWARE_FLOW_CONTROL
#define TM_USART1_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART1_MODE
#define TM_USART1_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_USART1_PARITY
#define TM_USART1_PARITY					USART_PARITY_NONE
#endif
#ifndef TM_USART1_STOP_BITS
#define TM_USART1_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_USART1_WORD_LENGTH
#define TM_USART1_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART2 default settings */
#ifndef TM_USART2_HARDWARE_FLOW_CONTROL
#define TM_USART2_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART2_MODE
#define TM_USART2_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_USART2_PARITY
#define TM_USART2_PARITY					USART_PARITY_NONE
#endif
#ifndef TM_USART2_STOP_BITS
#define TM_USART2_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_USART2_WORD_LENGTH
#define TM_USART2_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART3 default settings */
#ifndef TM_USART3_HARDWARE_FLOW_CONTROL
#define TM_USART3_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART3_MODE
#define TM_USART3_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_USART3_PARITY
#define TM_USART3_PARITY					USART_PARITY_NONE
#endif
#ifndef TM_USART3_STOP_BITS
#define TM_USART3_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_USART3_WORD_LENGTH
#define TM_USART3_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART4 default settings */
#ifndef TM_UART4_HARDWARE_FLOW_CONTROL
#define TM_UART4_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_UART4_MODE
#define TM_UART4_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_UART4_PARITY
#define TM_UART4_PARITY						USART_PARITY_NONE
#endif
#ifndef TM_UART4_STOP_BITS
#define TM_UART4_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_UART4_WORD_LENGTH
#define TM_UART4_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART5 default settings */
#ifndef TM_UART5_HARDWARE_FLOW_CONTROL
#define TM_UART5_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_UART5_MODE
#define TM_UART5_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_UART5_PARITY
#define TM_UART5_PARITY						USART_PARITY_NONE
#endif
#ifndef TM_UART5_STOP_BITS
#define TM_UART5_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_UART5_WORD_LENGTH
#define TM_UART5_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* USART6 default settings */
#ifndef TM_USART6_HARDWARE_FLOW_CONTROL
#define TM_USART6_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART6_MODE
#define TM_USART6_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_USART6_PARITY
#define TM_USART6_PARITY					USART_PARITY_NONE
#endif
#ifndef TM_USART6_STOP_BITS
#define TM_USART6_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_USART6_WORD_LENGTH
#define TM_USART6_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART7 default settings */
#ifndef TM_UART7_HARDWARE_FLOW_CONTROL
#define TM_UART7_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_UART7_MODE
#define TM_UART7_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_UART7_PARITY
#define TM_UART7_PARITY						USART_PARITY_NONE
#endif
#ifndef TM_UART7_STOP_BITS
#define TM_UART7_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_UART7_WORD_LENGTH
#define TM_UART7_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* UART8 default settings */
#ifndef TM_UART8_HARDWARE_FLOW_CONTROL
#define TM_UART8_HARDWARE_FLOW_CONTROL		TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_UART8_MODE
#define TM_UART8_MODE						USART_MODE_TX_RX
#endif
#ifndef TM_UART8_PARITY
#define TM_UART8_PARITY						USART_PARITY_NONE
#endif
#ifndef TM_UART8_STOP_BITS
#define TM_UART8_STOP_BITS					USART_STOPBITS_1
#endif
#ifndef TM_UART8_WORD_LENGTH
#define TM_UART8_WORD_LENGTH				UART_WORDLENGTH_8B
#endif

/* STM32F0xx related */
/* USART4 default settings */
#ifndef TM_USART4_HARDWARE_FLOW_CONTROL
#define TM_USART4_HARDWARE_FLOW_CONTROL   TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART4_MODE
#define TM_USART4_MODE            USART_MODE_TX_RX
#endif
#ifndef TM_USART4_PARITY
#define TM_USART4_PARITY          USART_PARITY_NONE
#endif
#ifndef TM_USART4_STOP_BITS
#define TM_USART4_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef TM_USART4_WORD_LENGTH
#define TM_USART4_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART5 default settings */
#ifndef TM_USART5_HARDWARE_FLOW_CONTROL
#define TM_USART5_HARDWARE_FLOW_CONTROL   TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART5_MODE
#define TM_USART5_MODE            USART_MODE_TX_RX
#endif
#ifndef TM_USART5_PARITY
#define TM_USART5_PARITY          USART_PARITY_NONE
#endif
#ifndef TM_USART5_STOP_BITS
#define TM_USART5_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef TM_USART5_WORD_LENGTH
#define TM_USART5_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART7 default settings */
#ifndef TM_USART7_HARDWARE_FLOW_CONTROL
#define TM_USART7_HARDWARE_FLOW_CONTROL   TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART7_MODE
#define TM_USART7_MODE            USART_MODE_TX_RX
#endif
#ifndef TM_USART7_PARITY
#define TM_USART7_PARITY          USART_PARITY_NONE
#endif
#ifndef TM_USART7_STOP_BITS
#define TM_USART7_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef TM_USART7_WORD_LENGTH
#define TM_USART7_WORD_LENGTH       UART_WORDLENGTH_8B
#endif

/* USART8 default settings */
#ifndef TM_USART8_HARDWARE_FLOW_CONTROL
#define TM_USART8_HARDWARE_FLOW_CONTROL   TM_USART_HardwareFlowControl_None
#endif
#ifndef TM_USART8_MODE
#define TM_USART8_MODE            USART_MODE_TX_RX
#endif
#ifndef TM_USART8_PARITY
#define TM_USART8_PARITY          USART_PARITY_NONE
#endif
#ifndef TM_USART8_STOP_BITS
#define TM_USART8_STOP_BITS         USART_STOPBITS_1
#endif
#ifndef TM_USART8_WORD_LENGTH
#define TM_USART8_WORD_LENGTH       UART_WORDLENGTH_8B
#endif


/* Define ISR if not already */
#if !defined(USART_ISR_RXNE)
#define USART_ISR_RXNE                      USART_SR_RXNE
#endif

/**
 * @brief  Default string delimiter for USART
 */
#define USART_STRING_DELIMITER              '\n'

/* Configuration */
#if defined(STM32F4XX)
#define USART_TX_REG(USARTx)                ((USARTx)->DR)
#define USART_WRITE_DATA(USARTx, data)      ((USARTx)->DR = (data))
#define USART_READ_DATA(USARTx)             ((USARTx)->DR)
#define GPIO_AF_UART5                       (GPIO_AF8_UART5)
#define USART_STATUS_REG                    SR
#else
#define USART_TX_REG(USARTx)                ((USARTx)->TDR)
#define USART_WRITE_DATA(USARTx, data)      ((USARTx)->TDR = (data))
#define USART_READ_DATA(USARTx)             ((USARTx)->RDR)
#if defined(STM32F7xx)
#define GPIO_AF_UART5                       (GPIO_AF8_UART5)
#else
#define GPIO_AF_UART5                       (GPIO_AF7_UART5)
#endif /* STM32F7xx */
#define USART_STATUS_REG                    ISR
#endif /* STM32F4XX */

/* Wait for TX empty */
#define USART_TXEMPTY(USARTx)               ((USARTx)->USART_STATUS_REG & USART_FLAG_TXE)
#define USART_WAIT(USARTx)                  while (!USART_TXEMPTY(USARTx))

 /**
 * @}
 */

/**
 * @defgroup TM_USART_Functions
 * @brief    USART Functions
 * @{
 */

/**
 * @brief  Initializes USARTx peripheral and corresponding pins
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  pinspack: This parameter can be a value of @ref TM_USART_PinsPack_t enumeration
 * @param  baudrate: Baudrate number for USART communication
 * @retval None
 */
void TM_USART_Init(USART_TypeDef* USARTx, TM_USART_PinsPack_t pinspack, uint32_t baudrate);

/**
 * @brief  Initializes USARTx peripheral and corresponding pins with custom hardware flow control mode
 * @note   Hardware flow control pins are not initialized. Easy solution is to use @arg TM_USART_PinsPack_Custom pinspack option 
 *         when you call @ref TM_USART_Init() function and initialize all USART pins at a time inside @ref TM_USART_InitCustomPinsCallback() 
 *         callback function, which will be called from my library
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  pinspack: This parameter can be a value of @ref TM_USART_PinsPack_t enumeration
 * @param  baudrate: Baudrate number for USART communication
 * @param  FlowControl: Flow control mode you will use. This parameter can be a value of @ref TM_USART_HardwareFlowControl_t enumeration
 * @retval None
 */
void TM_USART_InitWithFlowControl(USART_TypeDef* USARTx, TM_USART_PinsPack_t pinspack, uint32_t baudrate, TM_USART_HardwareFlowControl_t FlowControl);

/**
 * @brief  Puts character to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  c: character to be send over USART
 * @retval None
 */
static __INLINE void TM_USART_Putc(USART_TypeDef* USARTx, volatile char c) {
	/* Check USART */
	if ((USARTx->CR1 & USART_CR1_UE)) {	
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
		/* Send data */
		USART_WRITE_DATA(USARTx, (uint16_t)(c & 0x01FF));
		/* Wait to be ready, buffer empty */
		USART_WAIT(USARTx);
	}
}

/**
 * @brief  Puts string to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *str: Pointer to string to send over USART
 * @retval None
 */
void TM_USART_Puts(USART_TypeDef* USARTx, char* str);

/**
 * @brief  Sends data array to USART port
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *DataArray: Pointer to data array to be sent over USART
 * @param  count: Number of elements in data array to be send over USART
 * @retval None
 */
void TM_USART_Send(USART_TypeDef* USARTx, uint8_t* DataArray, uint16_t count);

/**
 * @brief  Gets character from internal USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Character from buffer, or 0 if nothing in buffer
 */
uint8_t TM_USART_Getc(USART_TypeDef* USARTx);

/**
 * @brief  Get string from USART
 *
 *         This function can create a string from USART received data.
 *
 *         It generates string until "\n" is not recognized or buffer length is full.
 * 
 * @note   As of version 1.5, this function automatically adds 0x0A (Line feed) at the end of string.
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *buffer: Pointer to buffer where data will be stored from buffer
 * @param  bufsize: maximal number of characters we can add to your buffer, including leading zero
 * @retval Number of characters in buffer
 */
uint16_t TM_USART_Gets(USART_TypeDef* USARTx, char* buffer, uint16_t bufsize);

/**
 * @brief  Check if character c is available in internal buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  c: character to check if it is in USARTx's buffer
 * @retval Character status:
 *            -  < 0: Character was not found
 *            - >= 0: Character has been found in buffer
 */
int16_t TM_USART_FindCharacter(USART_TypeDef* USARTx, uint8_t c);

/**
 * @brief  Checks if internal USARTx buffer is empty
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Buffer empty status:
 *            - 0: Buffer is not empty
 *            - > 0: Buffer is empty
 */
uint8_t TM_USART_BufferEmpty(USART_TypeDef* USARTx);

/**
 * @brief  Checks if internal USARTx buffer is full
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Buffer full status:
 *            - 0: Buffer is not full
 *            - > 0: Buffer is full
 */
uint8_t TM_USART_BufferFull(USART_TypeDef* USARTx);

/**
 * @brief  Gets number of bytes in USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval Number of elements in buffer
 */
uint16_t TM_USART_BufferCount(USART_TypeDef* USARTx);

/**
 * @brief  Clears internal USART buffer
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @retval None
 */
void TM_USART_ClearBuffer(USART_TypeDef* USARTx);

/**
 * @brief  Sets custom character for @ref TM_USART_Gets() function to detect when string ends
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  Character: Character value to be used as string end
 * @note   Character will also be added at the end for your buffer when calling @ref TM_USART_Gets() function
 * @retval None
 */
void TM_USART_SetCustomStringEndCharacter(USART_TypeDef* USARTx, uint8_t Character);

/**
 * @brief  Search for string in USART buffer if exists
 * @param  *USARTx: Pointer to USARTx peripheral you will use
 * @param  *str: String to be searched
 * @retval Search status:
 *            -  < 0: String is not in buffer
 *            - >= 0: String is in buffer
 */
int16_t TM_USART_FindString(USART_TypeDef* USARTx, char* str);

/**
 * @brief  Callback for custom pins initialization for USARTx.
 *
 *         When you call @ef TM_USART_Init() function, and if you pass @arg TM_USART_PinsPack_Custom to function,
 *         then this function will be called where you can initialize custom pins for USART peripheral.
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  *USARTx: Pointer to USARTx peripheral you will use for initialization
 * @param  AlternateFunction: Alternate function number which should be used for GPIO pins
 * @retval None
 */
void TM_USART_InitCustomPinsCallback(USART_TypeDef* USARTx, uint16_t AlternateFunction);

/**
 * @brief  Callback function for receive interrupt on USART1 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_USART1_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART2 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_USART2_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART3 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_USART3_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART4 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_UART4_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART5 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_UART5_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on USART6 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_USART6_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART7 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_UART7_ReceiveHandler(uint8_t c);

/**
 * @brief  Callback function for receive interrupt on UART8 in case you have enabled custom USART handler mode 
 * @note   With __weak parameter to prevent link errors if not defined by user
 * @param  c: character received via USART
 * @retval None
 */
void TM_UART8_ReceiveHandler(uint8_t c);

/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
