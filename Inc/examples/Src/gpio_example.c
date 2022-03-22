/*
 * gpio_example.c
 *
 *  Created on: 21 mar 2022
 *      Author: pawel
 */

#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

#define LED1_GPIO_PORT GPIOC
#define LED1_GPIO_PIN GPIO_PIN_13

void example_gpio_toggle_led(void) {

	// init LED
  md_gpio_init_clock(GPIOC);
  md_gpio_configure_output(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  // configure systick as 1ms
  md_systick_configure_ms();

  // toggle led every 1s
  while (1) {
    md_gpio_write_pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET);

    md_systick_delay(1000);

    md_gpio_write_pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);

    md_systick_delay(1000);
  }
}
