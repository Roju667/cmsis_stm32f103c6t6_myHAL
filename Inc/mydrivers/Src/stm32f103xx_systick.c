/*
 * stm32f103xx_systick.c
 *
 *  Created on: 22 mar 2022
 *      Author: ROJEK
 */


#include "stm32f1xx.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

static volatile uint32_t systick;

void md_systick_configure_ms(void)
{
	rcc_clock_freqs_t freqs;

	md_rcc_get_frequencies(&freqs);

	SysTick_Config(freqs.hclk/1000);

	return;
}

uint32_t md_systick_get_tick(void)
{
	return systick;
}

void md_systick_delay(uint32_t miliseconds)
{
	uint32_t delay = md_systick_get_tick();
	while(md_systick_get_tick() - delay < miliseconds)
		;
	return;
}

void SysTick_Handler (void)
{
	systick++;
}

