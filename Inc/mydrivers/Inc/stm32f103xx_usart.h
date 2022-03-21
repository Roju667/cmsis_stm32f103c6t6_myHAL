/*
 * stm32f103xx_usart.h
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_USART_H_
#define MYDRIVERS_INC_STM32F103XX_USART_H_

// @usart_word_lenght
typedef enum
{
	USART_WORD_LENGHT_8BIT,
	USART_WORD_LENGHT_9BIT
}usart_word_lenght_t;

// @usart_stop_bits
typedef enum
{
	USART_STOP_BITS_1,
	USART_STOP_BITS_05,
	USART_STOP_BITS_2,
	USART_STOP_BITS_15
}usart_stop_bits_t;

#endif /* MYDRIVERS_INC_STM32F103XX_USART_H_ */
