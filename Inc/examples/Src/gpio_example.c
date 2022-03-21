/*
 * gpio_example.c
 *
 *  Created on: 21 mar 2022
 *      Author: pawel
 */

#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"

#define LED1_GPIO_PORT GPIOC
#define LED1_GPIO_PIN GPIO_PIN_13

void example_gpio_toggle_led(void) {
  md_gpio_init_clock(GPIOC);
  md_gpio_configure_output(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  while (1) {
    md_gpio_write_pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_SET);

    for (uint32_t i = 0; i < 100000; i++)
      ;

    md_gpio_write_pin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_PIN_RESET);

    for (uint32_t i = 0; i < 100000; i++)
      ;
  }
}
