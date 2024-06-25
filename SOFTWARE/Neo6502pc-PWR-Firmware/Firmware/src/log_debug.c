#include <ch32v00x.h>

#include "log_debug.h"

void LOG_DEBUG_Configure(uint32_t baudrate, uint16_t stop_bits, uint16_t parity) {
	#ifdef LOD_DEBUG_ENABLE
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    /* CH32_UART TX-->D5   RX-->D6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
    USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_StopBits = stop_bits;
	USART_InitStructure.USART_Parity = parity;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
	#endif
}
