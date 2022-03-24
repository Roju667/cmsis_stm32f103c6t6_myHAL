/*
 * stm32f103xx_systick.c
 *
 *  Created on: 22 mar 2022
 *      Author: ROJEK
 */
#include "stm32f1xx.h"
#include "stm32f103xx_systick.h"
#include "stm32f103xx_rcc.h"

static volatile uint32_t systick;

/*
 * Configure systick value to 1 ms and start it
 * @param[void]
 * @return - void
 */
void md_systick_configure_ms(void)
{
  rcc_clock_freqs_t freqs;

  md_rcc_get_frequencies(&freqs);

  SysTick_Config(freqs.hclk / 1000);

  return;
}

/*
 * Get current sys tick value
 * @param[void]
 * @return - systick value
 */
uint32_t md_systick_get_tick(void) { return systick; }

/*
 * calssic delay
 * @param[miliseconds] - time in ms
 * @return - void
 */
void md_systick_delay(uint32_t miliseconds)
{
  uint32_t delay = md_systick_get_tick();
  while (md_systick_get_tick() - delay < miliseconds)
    ;
  return;
}

/*
 * Inc systick counter
 * @param[void]
 * @return - void
 */
void SysTick_Handler(void) { systick++; }
