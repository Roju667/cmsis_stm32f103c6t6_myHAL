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

void example_heart_beat_no_delay(uint32_t *tick_buffer, uint32_t freq)
{
  // init LED
  md_gpio_configure_output(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  if (md_systick_get_tick() - (*tick_buffer) > 500)
    {
      md_gpio_toggle_pin(LED1_GPIO_PORT, LED1_GPIO_PIN);
      *tick_buffer = md_systick_get_tick();
    }

  return;
}
