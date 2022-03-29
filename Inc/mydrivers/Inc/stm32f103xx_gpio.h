/*
 * stm32f103xx_gpio.h
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#ifndef STM32F103XX_GPIO_H_
#define STM32F103XX_GPIO_H_

#include "stm32f1xx.h"

// @gpio_pin_number
typedef enum
{
  GPIO_PIN_0,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_3,
  GPIO_PIN_4,
  GPIO_PIN_5,
  GPIO_PIN_6,
  GPIO_PIN_7,
  GPIO_PIN_8,
  GPIO_PIN_9,
  GPIO_PIN_10,
  GPIO_PIN_11,
  GPIO_PIN_12,
  GPIO_PIN_13,
  GPIO_PIN_14,
  GPIO_PIN_15
} gpio_pin_number_t;

// @gpio_speed
typedef enum
{
  GPIO_SPEED_10MHZ = 1,
  GPIO_SPEED_2MHZ,
  GPIO_SPEED_50MHZ
} gpio_speed_t;

// @gpio_output_config
typedef enum
{
  GPIO_OUTPUT_PP,
  GPIO_OUTPUT_OD,
  GPIO_OUTPUT_AF_PP,
  GPIO_OUTPUT_AF_OD
} gpio_ouput_config_t;

// @gpio_input_config
typedef enum
{
  GPIO_INPUT_ANALOG,
  GPIO_INPUT_FLOATING,
  GPIO_INPUT_PULLUP,
  GPIO_INPUT_PULLDOWN
} gpio_input_config;

// @gpio_exti_trigger
typedef enum
{
  GPIO_EXTI_RISING,
  GPIO_EXTI_FALLING,
  GPIO_EXTI_RISINGFALLING
} gpio_exti_trigger;

#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U

#define GPIO_BASEADDR_TO_CODE(x)                                               \
  ((x == GPIOA)   ? 0                                                          \
   : (x == GPIOB) ? 1                                                          \
   : (x == GPIOC) ? 2                                                          \
   : (x == GPIOD) ? 3                                                          \
                  : 0)

// functions prototypes

// clock functions
void md_gpio_init_clock(GPIO_TypeDef *p_GPIOx);
void md_gpio_init_af_clock(void);

// pin configuration functions
void md_gpio_configure_output(GPIO_TypeDef *p_GPIOx,
                              gpio_pin_number_t pin_number,
                              gpio_speed_t output_speed,
                              gpio_ouput_config_t output_conifg);
void md_gpio_configure_input(GPIO_TypeDef *p_GPIOx,
                             gpio_pin_number_t pin_number,
                             gpio_input_config input_config);
void md_gpio_configure_exti(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,
                            gpio_input_config input_config,
                            gpio_exti_trigger trigger, uint8_t irq_prio);

// pin status handling cuntions
uint8_t md_gpio_read_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number);
void md_gpio_write_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,
                       uint8_t value);
void md_gpio_toggle_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number);

// exti callbacks
void md_gpio_exti15_10_callback(gpio_pin_number_t pin_number);
void md_gpio_exti9_5_callback(gpio_pin_number_t pin_number);
void md_gpio_exti4_callback(void);
void md_gpio_exti3_callback(void);
void md_gpio_exti2_callback(void);
void md_gpio_exti1_callback(void);
void md_gpio_exti0_callback(void);

#endif /* STM32F103XX_GPIO_H_ */
