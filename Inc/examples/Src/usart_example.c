/*
 * usart_example.c
 *
 *  Created on: 22 mar 2022
 *      Author: ROJEK
 */
#include "usart_example.h"
#include "gpio_example.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_usart.h"

volatile uint32_t transmission_counter_usart1, transmission_counter_usart2;
volatile uint8_t start_new_transmit1, start_new_transmit2;
/*
 * Example : Basic usart configuration
 * @return - void
 */
void example_usart_configure_baud(void)
{
  // Configure RCC
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_DIV2,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);

  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);

  // Start GPIO clock for USART pins
  md_gpio_init_clock(GPIOA);

  // Init Usart
  md_usart_init_handlers();

  md_usart_init_clock(&husart1);

  md_usart_init_gpio(&husart1);

  md_usart_init_basic(&husart1, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  // Loop toggle led
  while (1)
    {
      example_heart_beat();
    }
}

/*
 * Example : Send message on uart in IRQ mode
 * @return - void
 */
void example_usart_send_irq(void)
{

  transmission_counter_usart1 = 0;
  transmission_counter_usart2 = 0;

  // Configure RCC
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_DIV2,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);

  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);

  // Start GPIO clock for USART pins
  md_gpio_init_clock(GPIOA);

  // Init Usart
  md_usart_init_handlers();

  md_usart_init_clock(&husart1);

  md_usart_init_gpio(&husart1);

  md_usart_init_basic(&husart1, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  md_usart_enable_irq(&husart1, 5);

  uint8_t databuffer[16] = "Test 123123 \n\r";

  // Send messages by IRQ
  while (1)
    {
      example_heart_beat();
      md_usart_tx_irq(&husart1, databuffer, 16, 1000);
    }
}

/*
 * Example : Send messages on 2 uarts in IRQ mode at the same time
 * @return - void
 */
void example_usart_send_irq_2usarts(void)
{

  transmission_counter_usart1 = 0;
  transmission_counter_usart2 = 0;

  // Configure RCC
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_DIV2,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);

  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);

  // Start GPIO clock for USART pins
  md_gpio_init_clock(GPIOA);

  md_usart_init_handlers();

  // Init Usart 1
  md_usart_init_handlers();

  md_usart_init_clock(&husart1);

  md_usart_init_gpio(&husart1);

  md_usart_init_basic(&husart1, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  md_usart_enable_irq(&husart1, 5);

  // Init Usart 2

  md_usart_init_clock(&husart2);

  md_usart_init_gpio(&husart2);

  md_usart_init_basic(&husart2, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  md_usart_enable_irq(&husart2, 5);

  uint8_t databuffer1[16] = "Test 123123 \n\r";
  uint8_t databuffer2[64] =
      "very very long message that has to go on usart\n\r";

  md_usart_tx_irq(&husart1, databuffer1, 16, 1000);
  md_usart_tx_irq(&husart2, databuffer2, 64, 1000);

  // Send messages in loop in irq mode
  while (1)
    {
      if (start_new_transmit1 == 1)
        {
          start_new_transmit1 = 0;
          md_usart_tx_irq(&husart1, databuffer1, 16, 1000);
        }

      if (start_new_transmit2 == 1)
        {
          start_new_transmit2 = 0;
          md_usart_tx_irq(&husart2, databuffer2, 64, 1000);
        }
    }
}

/*
 * Example : Send usart message on 2 usarts in polling mode
 * @return - void
 */
void example_usart_send_polling_2usarts(void)
{

  // Configure RCC
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_DIV2,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);

  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);

  // Start GPIO clock for USART pins
  md_gpio_init_clock(GPIOA);

  md_usart_init_handlers();

  // Init Usart 1

  md_usart_init_clock(&husart1);

  md_usart_init_gpio(&husart1);

  md_usart_init_basic(&husart1, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  // Init Usart 2

  md_usart_init_clock(&husart2);

  md_usart_init_gpio(&husart2);

  md_usart_init_basic(&husart2, USART_WORD_LENGHT_8BIT, USART_STOP_BITS_1,
                      9600);

  uint8_t databuffer1[16] = "Test 123123 \n\r";
  uint8_t databuffer2[64] =
      "very very long message that has to go on usart\n\r";

  // Send messages in loop in polling mode
  while (1)
    {
      md_usart_tx_polling(&husart1, databuffer1, 16, 1000);
      md_usart_tx_polling(&husart2, databuffer2, 64, 1000);
    }
}

void md_usart_tc_callback(usart_handle_t *p_hUSARTx)
{
  if (p_hUSARTx->p_USARTx == USART1)
    {
      transmission_counter_usart1++;
      start_new_transmit1 = 1;
    }

  if (p_hUSARTx->p_USARTx == USART2)
    {
      transmission_counter_usart2++;
      start_new_transmit2 = 1;
    }
}
