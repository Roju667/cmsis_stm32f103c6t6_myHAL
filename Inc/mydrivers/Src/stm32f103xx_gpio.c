/*
 * stm32f103xx_gpio.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#include "stm32f103xx_gpio.h"

#include "stm32f103xx_mydriver.h"
#include "stm32f103xx_rcc.h"

/*
 * Starts clock for GPIO and resets the peripheral
 * @param[*p_GPIOx] - gpiox base address
 * @return - void
 */
void md_gpio_init_clock(GPIO_TypeDef *p_GPIOx)
{
  if (p_GPIOx == GPIOA)
    {
      if (RCC->APB2ENR & RCC_APB2ENR_IOPAEN)
        return;

      RCC_CLOCK_ENABLE_IOPA();
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPARST);
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPARST);
    }
  else if (p_GPIOx == GPIOB)
    {
      if (RCC->APB2ENR & RCC_APB2ENR_IOPBEN)
        return;

      RCC_CLOCK_ENABLE_IOPB();
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPBRST);
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPBRST);
    }
  else if (p_GPIOx == GPIOC)
    {
      if (RCC->APB2ENR & RCC_APB2ENR_IOPCEN)
        return;

      RCC_CLOCK_ENABLE_IOPC();
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPCRST);
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPCRST);
    }
  else if (p_GPIOx)
    {
      if (RCC->APB2ENR & RCC_APB2ENR_IOPDEN)
        return;

      RCC_CLOCK_ENABLE_IOPD();
      SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPDRST);
      CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_IOPDRST);
    }

  return;
}

/*
 * Starts clock for AFIO and resets the peripheral
 * @param[void]
 * @return - void
 */
void md_gpio_init_af_clock(void)
{
  if (RCC->APB2ENR & RCC_APB2ENR_AFIOEN)
    return;

  RCC_CLOCK_ENABLE_AFIO();
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_AFIORST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_AFIORST);
  return;
}

/*
 * Config output pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @param[output_speed] - output speed enum @gpio_speed
 * @param[output_conifg] - output config enum @gpio_output_config
 * @return - void
 */
void md_gpio_configure_output(GPIO_TypeDef *p_GPIOx,
                              gpio_pin_number_t pin_number,
                              gpio_speed_t output_speed,
                              gpio_ouput_config_t output_conifg)
{
  md_gpio_init_clock(p_GPIOx);

  // choose between CRL/CRH
  if (pin_number < 8)
    {
      // clear config and mode bits
      p_GPIOx->CRL &= ~(0x0F << (pin_number * 4));

      // set mode and config
      p_GPIOx->CRL |= (output_speed << (pin_number * 4));
      p_GPIOx->CRL |= (output_conifg << ((pin_number * 4) + 2));
    }
  else
    {
      // clear config and mode bits
      p_GPIOx->CRH &= ~(0x0F << ((pin_number - 8) * 4));

      // set mode and config
      p_GPIOx->CRH |= (output_speed << ((pin_number - 8) * 4));
      p_GPIOx->CRH |= (output_conifg << (((pin_number - 8) * 4) + 2));
    }

  return;
}

/*
 * Config input pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @param[input_config] - input config enum @gpio_input_config
 * @return - void
 */
void md_gpio_configure_input(GPIO_TypeDef *p_GPIOx,
                             gpio_pin_number_t pin_number,
                             gpio_input_config input_config)
{
  md_gpio_init_clock(p_GPIOx);

  // choose between CRL/CRH
  if (pin_number < 8)
    {
      // clear config and mode bits
      p_GPIOx->CRL &= ~(0x0F << (pin_number * 4));

      // set config
      if (input_config > GPIO_INPUT_FLOATING)
        {
          p_GPIOx->CRL |= (GPIO_INPUT_PULLUP << ((pin_number * 4) + 2));
        }
      else
        {
          p_GPIOx->CRL |= (input_config << ((pin_number * 4) + 2));
        }
    }
  else
    {
      // clear config and mode bits
      p_GPIOx->CRH &= ~(0x0F << ((pin_number - 8) * 4));

      // set config
      if (input_config > GPIO_INPUT_FLOATING)
        {
          p_GPIOx->CRH |= (GPIO_INPUT_PULLUP << (((pin_number - 8) * 4) + 2));
        }
      else
        {
          p_GPIOx->CRH |= (input_config << (((pin_number - 8) * 4) + 2));
        }
    }

  // set or reset ouput register - pullup must have 1 in ODR
  if (input_config == GPIO_INPUT_PULLUP)
    {
      p_GPIOx->BSRR = (0x01 << pin_number);
    }
  else
    {
      p_GPIOx->BRR = (0x01 << pin_number);
    }

  return;
}

/*
 * Config exti input
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @param[input_config] - input config enum @gpio_input_config
 * @param[irq_prio] - priority of interrupt
 * @return - void
 */
void md_gpio_configure_exti(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,
                            gpio_input_config input_config,
                            gpio_exti_trigger trigger, uint8_t irq_prio)
{

  md_gpio_init_af_clock();
  md_gpio_configure_input(p_GPIOx, pin_number, input_config);

  uint8_t code = GPIO_BASEADDR_TO_CODE(p_GPIOx);

  AFIO->EXTICR[pin_number / 4] &= ~(0x0F << (pin_number % 4) * 4);
  AFIO->EXTICR[pin_number / 4] |= code << (pin_number % 4) * 4;

  // mask irq in exti
  EXTI->IMR |= (0x01 << pin_number);

  if (trigger == GPIO_EXTI_RISING || trigger == GPIO_EXTI_RISINGFALLING)
    {
      EXTI->RTSR |= (0x01 << pin_number);
    }

  if (trigger == GPIO_EXTI_FALLING || trigger == GPIO_EXTI_RISINGFALLING)
    {
      EXTI->FTSR |= (0x01 << pin_number);
    }

  if (pin_number == GPIO_PIN_0)
    {
      NVIC_EnableIRQ(EXTI0_IRQn);
      NVIC_SetPriority(EXTI0_IRQn, irq_prio);
    }
  else if (pin_number == GPIO_PIN_1)
    {
      NVIC_EnableIRQ(EXTI1_IRQn);
      NVIC_SetPriority(EXTI1_IRQn, irq_prio);
    }
  else if (pin_number == GPIO_PIN_2)
    {
      NVIC_EnableIRQ(EXTI2_IRQn);
      NVIC_SetPriority(EXTI2_IRQn, irq_prio);
    }
  else if (pin_number == GPIO_PIN_3)
    {
      NVIC_EnableIRQ(EXTI3_IRQn);
      NVIC_SetPriority(EXTI3_IRQn, irq_prio);
    }
  else if (pin_number == GPIO_PIN_4)
    {
      NVIC_EnableIRQ(EXTI4_IRQn);
      NVIC_SetPriority(EXTI4_IRQn, irq_prio);
    }
  else if (pin_number < GPIO_PIN_10)
    {
      NVIC_EnableIRQ(EXTI9_5_IRQn);
      NVIC_SetPriority(EXTI9_5_IRQn, irq_prio);
    }
  else
    {
      NVIC_SetPriority(EXTI15_10_IRQn, irq_prio);
      NVIC_EnableIRQ(EXTI15_10_IRQn);
    }

  return;
}

/*
 * Reads value from pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @return - pin value
 */
uint8_t md_gpio_read_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number)
{
  return (p_GPIOx->IDR >> pin_number & 0x01);
}

/*
 * Write value to pin
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @param[value] - GPIO_PIN_SET/GPIO_PIN_RESET
 * @return - void
 */
void md_gpio_write_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,
                       uint8_t value)
{
  if (value == GPIO_PIN_SET)
    {
      p_GPIOx->BSRR = (0x01 << pin_number);
    }
  else if (value == GPIO_PIN_RESET)
    {
      p_GPIOx->BRR = (0x01 << pin_number);
    }

  return;
}

/*
 * toggle output value
 * @param[*p_GPIOx] - gpiox base address
 * @param[pin_number] - pin number enum @gpio_pin_number
 * @return - void
 */
void md_gpio_toggle_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number)
{
  p_GPIOx->ODR ^= (0x01 << pin_number);
  return;
}

/*
 * callback for exti 15-10 lines - clearing irq flags not required
 * @param[pin_number] - first pending register from 15-10 lines
 * @return - void
 */
__weak void md_gpio_exti15_10_callback(gpio_pin_number_t pin_number) {}

/*
 * callback for exti 9-5 lines - clearing irq flags not required
 * @param[pin_number] - first pending register from 15-10 lines
 * @return - void
 */
__weak void md_gpio_exti9_5_callback(gpio_pin_number_t pin_number) {}

/*
 * callback for exti 4 line - clearing irq flags not required
 * @param[void]
 * @return - void
 */
__weak void md_gpio_exti4_callback(void) {}
/*
 * callback for exti 3 line - clearing irq flags not required
 * @param[void]
 * @return - void
 */
__weak void md_gpio_exti3_callback(void) {}
/*
 * callback for exti 2 line - clearing irq flags not required
 * @param[void]
 * @return - void
 */
__weak void md_gpio_exti2_callback(void) {}
/*
 * callback for exti 1 line - clearing irq flags not required
 * @param[void]
 * @return - void
 */
__weak void md_gpio_exti1_callback(void) {}
/*
 * callback for exti 0 line - clearing irq flags not required
 * @param[void]
 * @return - void
 */
__weak void md_gpio_exti0_callback(void) {}

/*
 * find first exti pending number from registers 15-10/9-5
 * @return - pin number;
 */
static uint8_t gpio_exti15_10_get_pin(IRQn_Type exti_irq)
{
  uint32_t temp = EXTI->PR;

  if (exti_irq == EXTI9_5_IRQn)
    {
      temp >>= 5;
      for (uint8_t i = 0; i < 5; i++)
        {
          if (temp & 0x01)
            return (i + 5);

          temp >>= 1;
        }
    }
  else if (exti_irq == EXTI15_10_IRQn)
    {
      temp >>= 10;
      for (uint8_t i = 0; i < 6; i++)
        {
          if (temp & 0x01)
            return (i + 10);

          temp >>= 1;
        }
    }

  return 0;
}

// Vector table handlers for exti

void EXTI15_10_IRQHandler(void)
{
  // get pending irq number from exti
  uint8_t pending_irq_no = gpio_exti15_10_get_pin(EXTI15_10_IRQn);
  // clear exti flag
  EXTI->PR |= (0x01 << pending_irq_no);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  // user defined irq action
  md_gpio_exti15_10_callback(pending_irq_no);
}

void EXTI9_5_IRQHandler(void)
{
  // get pending irq number from exti
  uint8_t pending_irq_no = gpio_exti15_10_get_pin(EXTI9_5_IRQn);
  // clear exti flag
  EXTI->PR |= (0x01 << pending_irq_no);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
  // user defined irq action
  md_gpio_exti15_10_callback(pending_irq_no);
}

void EXTI4_IRQHandler(void)
{
  // clear exti flag
  EXTI->PR |= (0x01 << 4);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI4_IRQn);
  // user defined irq action
  md_gpio_exti4_callback();
}

void EXTI3_IRQHandler(void)
{
  // clear exti flag
  EXTI->PR |= (0x01 << 3);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI3_IRQn);
  // user defined irq action
  md_gpio_exti3_callback();
}

void EXTI2_IRQHandler(void)
{
  // clear exti flag
  EXTI->PR |= (0x01 << 2);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI2_IRQn);
  // user defined irq action
  md_gpio_exti2_callback();
}

void EXTI1_IRQHandler(void)
{
  // clear exti flag
  EXTI->PR |= (0x01 << 1);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI1_IRQn);
  // user defined irq action
  md_gpio_exti1_callback();
}

void EXTI0_IRQHandler(void)
{
  // clear exti flag
  EXTI->PR |= (0x01 << 0);
  // clear nvic flag
  NVIC_ClearPendingIRQ(EXTI0_IRQn);
  // user defined irq action
  md_gpio_exti0_callback();
}
