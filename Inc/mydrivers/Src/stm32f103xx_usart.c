/*
 * stm32f103xx_usart.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */


#include "stm32f103xx_usart.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"

/*
 * Starts clock for USART and resets the peripheral
 * @param[*pUSARTx] - usartx base address
 * @return - void
 */
void md_usart_init_clock(USART_TypeDef *pUSARTx)
{
	if(pUSARTx == USART1)
	{
          SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
          CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
        }else if(pUSARTx == USART2)
	{
          SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
          CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
        }
	return;
}

/*
 * Init gpio pins for usart
 * @param[*pUSARTx] - usartx base address
 * @return - void
 */
void md_usart_init_gpio(USART_TypeDef *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		// tx - PA9 / REMAP : PB6
		md_gpio_configure_output(GPIOA, GPIO_PIN_9, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// rx - PA10 / REMAP : PB7
		md_gpio_configure_input(GPIOA, GPIO_PIN_10, GPIO_INPUT_PULLUP);

		// ck - clock for synchornous mode
		// md_gpio_configure_output(GPIOA, GPIO_PIN_1, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// rts - hardware flow control
		// md_gpio_configure_output(GPIOA, GPIO_PIN_1, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// cts - hardware flow control
		// md_gpio_configure_input(GPIOA, GPIO_PIN_1, GPIO_INPUT_PULLUP);
	}else if (pUSARTx == USART2)
	{
		// tx - PA2
		md_gpio_configure_output(GPIOA, GPIO_PIN_2, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// rx - PA3
		md_gpio_configure_input(GPIOA, GPIO_PIN_3, GPIO_INPUT_PULLUP);

		// ck - clock for synchornous mode PA4
		// md_gpio_configure_output(GPIOA, GPIO_PIN_1, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// rts - hardware flow control PA1
		// md_gpio_configure_output(GPIOA, GPIO_PIN_1, GPIO_SPEED_50MHZ, GPIO_OUTPUT_AF_PP);

		// cts - hardware flow control PA0
		// md_gpio_configure_input(GPIOA, GPIO_PIN_1, GPIO_INPUT_PULLUP);
	}

}


/*
 * Init uart parameters
 * @param[*pUSARTx] - usartx base address
 * @param[word_lenght] - word lenght enum @usart_word_lenght
 * @param[stop_bits] - stop bits enum @usart_stop_bits
 * @return - void
 */
void md_usart_init(USART_TypeDef *pUSARTx,usart_word_lenght_t word_lenght,usart_stop_bits_t stop_bits, uint32_t baud_rate)
{
	// Enable Usart
	pUSARTx->CR1 |= USART_CR1_UE;

	// Define word lenght
	pUSARTx->CR1 |= (word_lenght << USART_CR1_M);

	// Program the number of stop bits
	pUSARTx->CR2 &= ~(USART_CR2_STOP_Msk);
	pUSARTx->CR2 |= (stop_bits << USART_CR2_STOP_Pos);
}


/*
 * Calculate parameters for BRR registers
 * @param[*pUSARTx] - usartx base address
 * @param[baud_rate] - baud rate
 * @return - void
 */
static void usart_set_baud_rate(USART_TypeDef *pUSARTx,uint32_t baud_rate)
{

}
