/*
 * usart_example.c
 *
 *  Created on: 22 mar 2022
 *      Author: ROJEK
 */
#include "gpio_example.h"
#include "usart_example.h"
#include "stm32f103xx_usart.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"

void example_usart_configure_baud(void)
{
	// configure pll baud rate
	  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_DIV2,
	                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);

	  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
	                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);

	  //init usart

	  md_usart_init_handlers();

	  md_usart_init_clock(&husart1);

	  md_usart_init_gpio(&husart1);

          md_usart_init_basic(&husart1, USART_WORD_LENGHT_8BIT,
                              USART_STOP_BITS_1, 9600);

          uint8_t databuffer[16] = "Test 123123 \n\r";

	while(1)
	{
          example_heart_beat();
          md_usart_tx_polling(&husart1, databuffer, 16, 10000);
        }

}
