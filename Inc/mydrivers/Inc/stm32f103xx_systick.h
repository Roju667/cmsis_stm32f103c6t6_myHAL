/*
 * stm32f103xx_systick.h
 *
 *  Created on: 22 mar 2022
 *      Author: ROJEK
 */

#ifndef MYDRIVERS_INC_STM32F103XX_SYSTICK_H_
#define MYDRIVERS_INC_STM32F103XX_SYSTICK_H_

void md_systick_configure_ms(void);
uint32_t md_systick_get_tick(void);
void md_systick_delay(uint32_t miliseconds);

#endif /* MYDRIVERS_INC_STM32F103XX_SYSTICK_H_ */
