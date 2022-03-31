/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include <stdint.h>

#include "can_example.h"
#include "gpio_example.h"
#include "rcc_example.h"
#include "spi_example.h"
#include "usart_example.h"
#include "adc_example.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning                                                                       \
    "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
  /* Loop forever */
  // example_gpio_toggle_led();
  // example_rcc_configure_pll();

  /* USART EXAMPLES */
  // example_usart_configure_baud()
  // example_usart_send_polling_2usarts();
  // example_usart_send_irq_2usarts();

  /* CAN EXAMPLES */
  //	example_can_init();

  /* SPI EXAMPLES */
  //  example_spi_transfer_message();
  //  example_spi_transfer_tft();

	/* ADC EXAMPLES */
	example_adc_init();

  for (;;)
    ;
}
