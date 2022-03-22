/*
 * stm32f103xx_usart.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */


#include "stm32f103xx_usart.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"
#include "math.h"

#ifdef MD_ENABLE_USART

#if MD_USING_USART1
usart_handle_t husart1;
#endif // MD_USING_USART1

#if MD_USING_USART2
usart_handle_t husart2;
#endif // MD_USING_USART2

/*
 * Init handler adresses
 * @param[void]
 * @return - void
 */
void md_usart_init_handlers(void)
{
#if MD_USING_USART1
	husart1.p_USARTx = USART1;
#endif // MD_USING_USART1

#if MD_USING_USART2
	husart2.p_USARTx = USART2;
#endif // MD_USING_USART2

}

/*
 * Starts clock for USART and resets the peripheral
 * @param[*pUSARTx] - usartx base address
 * @return - void
 */
void md_usart_init_clock(usart_handle_t *p_hUSARTx)
{
	if(p_hUSARTx->p_USARTx == USART1)
	{
		  RCC_CLOCK_ENABLE_USART1();
          SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
          CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
        }else if(p_hUSARTx->p_USARTx == USART2)
	{
        	RCC_CLOCK_ENABLE_USART2();
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
void md_usart_init_gpio(usart_handle_t *p_hUSARTx)
{
	if(p_hUSARTx->p_USARTx == USART1)
	{
		// init clock
		md_gpio_init_clock(GPIOA);

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
	}else if (p_hUSARTx->p_USARTx == USART2)
	{
		// init clock
		md_gpio_init_clock(GPIOA);

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
 * Calculate parameters for BRR registers
 * @param[*pUSARTx] - usartx base address
 * @param[baud_rate] - baud rate
 * @return - void
 */
void md_usart_set_baud_rate(usart_handle_t *p_hUSARTx,uint32_t baud_rate)
{
	uint32_t pclk_freq;
	uint32_t div_mantissa, div_fraction;
	float f_div_fraction;

	// get clock frequency
	if(p_hUSARTx->p_USARTx == USART1)
	{
		pclk_freq = md_rcc_get_pclk(2);
	}else if(p_hUSARTx->p_USARTx == USART2)
	{
		pclk_freq = md_rcc_get_pclk(1);
	}


	// calculate mantissa/fraction
	div_mantissa =  (pclk_freq / (baud_rate * 16));

	// **_START_SHAME_FLOAT_ZONE
	f_div_fraction = ((float) pclk_freq / (baud_rate * 16));
	f_div_fraction = round((f_div_fraction - div_mantissa) / 0.0625);

	// if fraction is more than 0,9375 then round it up to 1 and add to mantissa
	if(f_div_fraction > 15)
	{
		f_div_fraction = 0;
		div_mantissa++;
	}

	div_fraction = f_div_fraction;

	// **_STOP_SHAME_FLOAT_ZONE

	// clear register
	p_hUSARTx->p_USARTx->BRR = 0;

	// write new values
	p_hUSARTx->p_USARTx->BRR |= (div_mantissa << USART_BRR_DIV_Mantissa_Pos);
	p_hUSARTx->p_USARTx->BRR |= (div_fraction << USART_BRR_DIV_Fraction_Pos);

	return;
}


/*
 * Init uart parameters
 * @param[*pUSARTx] - usartx base address
 * @param[word_lenght] - word lenght enum @usart_word_lenght
 * @param[stop_bits] - stop bits enum @usart_stop_bits
 * @return - void
 */
void md_usart_init_basic(usart_handle_t *p_hUSARTx,usart_word_lenght_t word_lenght,usart_stop_bits_t stop_bits, uint32_t baud_rate)
{
	// Enable Usart
	p_hUSARTx->p_USARTx->CR1 |= USART_CR1_UE;

	// Define word lenght
	p_hUSARTx->p_USARTx->CR1 |= (word_lenght << USART_CR1_M_Pos);

	// Program the number of stop bits
	p_hUSARTx->p_USARTx->CR2 &= ~(USART_CR2_STOP_Msk);
	p_hUSARTx->p_USARTx->CR2 |= (stop_bits << USART_CR2_STOP_Pos);

	md_usart_set_baud_rate(p_hUSARTx, baud_rate);

	return;
}

/*
 * transmit message by polling
 * @param[*pUSARTx] - usartx base address
 * @param[data_buffer] - pointer to data buffer
 * @param[lenght] - message lenght
 * @param[timeout_ms] - time in miliseconds
 * @return - status @usart_status
 */
usart_error_t md_usart_tx_polling(usart_handle_t *p_hUSARTx, uint8_t* p_data_buffer, uint16_t lenght, uint32_t timeout_ms)
{
	uint16_t bytes_to_send = lenght;
	uint32_t time_tick;
	// enable transmitter
	p_hUSARTx->p_USARTx->CR1 |= USART_CR1_TE;

	while(bytes_to_send != 0)
	{

		// wait until TDR register is empty
		time_tick = md_systick_get_tick();
		while(!(p_hUSARTx->p_USARTx->SR & USART_SR_TXE))
		{
			if((time_tick - md_systick_get_tick())  > timeout_ms)
			{
				return USART_ERR_TIMEOUT_TXE;
			}
		}

		// put data in DR register
		p_hUSARTx->p_USARTx->DR = p_data_buffer[lenght - bytes_to_send];

		if(bytes_to_send == 1)
		{
			//wait until TC is set
			time_tick = md_systick_get_tick();
			while(!(p_hUSARTx->p_USARTx->SR & USART_SR_TC))
			{
				if((time_tick - md_systick_get_tick())  > timeout_ms)
				{
					return USART_ERR_TIMEOUT_TC;
				}
			}

		}

		//decrement no bytes
		bytes_to_send--;
	}

	return USART_ERR_NOERR;
}

usart_error_t md_usart_tx_irq(usart_handle_t *p_hUSARTx, uint8_t* p_data_buffer, uint16_t lenght, uint32_t timeout_ms)
{

}

__weak void md_usart_tx_single_callback(void)
{
	md_usart_tx_single_callback();
}

__weak void md_usart_tx_done_callback(void)
{
	md_usart_tx_done_callback();
}


static void md_usart_main_callback(usart_handle_t *p_hUSARTx)
{

}

// Vector table handlers for usart
#if MD_USING_USART1
void USART1_IRQHandler(void)
{
	md_usart_main_callback(&husart1);
}
#endif // MD_USING_USART1

#if MD_USING_USART2
void USART2_IRQHandler(void)
{
	md_usart_main_callback(&husart2);
}
#endif // MD_USING_USART1

#endif // MD_ENABLE_USART
