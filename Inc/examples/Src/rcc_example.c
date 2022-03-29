/*
 * rcc_example.c
 *
 *  Created on: 21 mar 2022
 *      Author: pawel
 */

#include "gpio_example.h"
#include "stm32f103xx_rcc.h"

#define LED1_GPIO_PORT GPIOC
#define LED1_GPIO_PIN GPIO_PIN_13

void example_rcc_configure_pll_32Mhz(void)
{

  rcc_clock_freqs_t frequencies = {0};

  // configure frequencies - no prescaler on buses + pll on mul 4x
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_NODIV,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);
  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);
  md_rcc_get_frequencies(&frequencies);
}
