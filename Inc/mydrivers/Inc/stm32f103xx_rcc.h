/*
 * stm32f103xx_rcc.h
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_RCC_H_
#define MYDRIVERS_INC_STM32F103XX_RCC_H_

#include "stm32f1xx.h"

// struct to save all the clock frequencies
typedef struct rcc_clock_freqs_t
{
  uint32_t sysclk;

  uint32_t hclk;

  uint32_t pclk1;

  uint32_t pclk2;

  uint32_t adcclk;

} rcc_clock_freqs_t;

// @rcc_sysclk_source
typedef enum rcc_sysclk_source_t
{
  RCC_SYSCLK_SOURCE_HSI = 0,
  RCC_SYSCLK_SOURCE_HSE,
  RCC_SYSCLK_SOURCE_PLL
} rcc_sysclk_source_t;

// @rcc_pll_source
typedef enum rcc_pll_source_t
{
  RCC_PLL_SOURCE_HSI = 0,
  RCC_PLL_SOURCE_HSE
} rcc_pll_source_t;

// @rcc_pll1_mul
typedef enum rcc_pll1_mul_t
{
  RCC_PLL1_MUL_X4 = 2,
  RCC_PLL1_MUL_X5,
  RCC_PLL1_MUL_X6,
  RCC_PLL1_MUL_X7,
  RCC_PLL1_MUL_X8,
  RCC_PLL1_MUL_X9,
  RCC_PLL1_MUL_X65 = 13
} rcc_pll1_mul_t;

// @rcc_hse_div
typedef enum rcc_hse_div_t
{
  RCC_HSE_DIV_NODIV = 0,
  RCC_HSE_DIV_DIV2,
} rcc_hse_div_t;

// HSI/HSE speed @rcc_oscillators_frequencies
#define RCC_HSI_FREQUENCY 8000000U
#define RCC_HSE_FREQUENCY 8000000U

// @rcc_ahb_prescaler
typedef enum rcc_ahb_prescaler_t
{
  RCC_AHB_PRESCALER_NODIV = 0,
  RCC_AHB_PRESCALER_DIV2 = 8,
  RCC_AHB_PRESCALER_DIV4,
  RCC_AHB_PRESCALER_DIV8,
  RCC_AHB_PRESCALER_DIV16,
  RCC_AHB_PRESCALER_DIV64,
  RCC_AHB_PRESCALER_DIV128,
  RCC_AHB_PRESCALER_DIV256,
  RCC_AHB_PRESCALER_DIV512,
} rcc_ahb_prescaler_t;

// @rcc_apb_prescaler
typedef enum rcc_apb_prescaler_t
{
  RCC_APB_PRESCALER_NODIV = 0,
  RCC_APB_PRESCALER_DIV2 = 4,
  RCC_APB_PRESCALER_DIV4,
  RCC_APB_PRESCALER_DIV8,
  RCC_APB_PRESCALER_DIV16,
} rcc_apb_prescaler_t;

// @rcc_adc_prescaler
typedef enum rcc_adc_prescaler_t
{
  RCC_ADC_PRESCALER_DIV2 = 0U,
  RCC_ADC_PRESCALER_DIV4,
  RCC_ADC_PRESCALER_DIV6,
  RCC_ADC_PRESCALER_DIV8,
} rcc_adc_prescaler_t;

/*
 * RCC MACROS
 */

// AHB peripherals enable
#define RCC_CLOCK_ENABLE_DMA1() RCC->AHBENR |= RCC_AHBENR_DMA1EN
#define RCC_CLOCK_ENABLE_SRAM() RCC->AHBENR |= RCC_AHBENR_SRAMEN
#define RCC_CLOCK_ENABLE_CRC() RCC->AHBENR |= RCC_AHBENR_CRCEN

// APB2 peripherals enable
#define RCC_CLOCK_ENABLE_AFIO() RCC->APB2ENR |= RCC_APB2ENR_AFIOEN
#define RCC_CLOCK_ENABLE_IOPA() RCC->APB2ENR |= RCC_APB2ENR_IOPAEN
#define RCC_CLOCK_ENABLE_IOPB() RCC->APB2ENR |= RCC_APB2ENR_IOPBEN
#define RCC_CLOCK_ENABLE_IOPC() RCC->APB2ENR |= RCC_APB2ENR_IOPCEN
#define RCC_CLOCK_ENABLE_IOPD() RCC->APB2ENR |= RCC_APB2ENR_IOPDEN
#define RCC_CLOCK_ENABLE_ADC1() RCC->APB2ENR |= RCC_APB2ENR_ADC1EN
#define RCC_CLOCK_ENABLE_ADC2() RCC->APB2ENR |= RCC_APB2ENR_ADC2EN
#define RCC_CLOCK_ENABLE_TIM1() RCC->APB2ENR |= RCC_APB2ENR_TIM1EN
#define RCC_CLOCK_ENABLE_SPI1() RCC->APB2ENR |= RCC_APB2ENR_SPI1EN
#define RCC_CLOCK_ENABLE_USART1() RCC->APB2ENR |= RCC_APB2ENR_USART1EN

// APB1 peripherals enable
#define RCC_CLOCK_ENABLE_TIM2() RCC->APB1ENR |= RCC_APB1ENR_TIM2EN
#define RCC_CLOCK_ENABLE_TIM3() RCC->APB1ENR |= RCC_APB1ENR_TIM3EN
#define RCC_CLOCK_ENABLE_WWDG() RCC->APB1ENR |= RCC_APB1ENR_WWDGEN
#define RCC_CLOCK_ENABLE_USART2() RCC->APB1ENR |= RCC_APB1ENR_USART2EN
#define RCC_CLOCK_ENABLE_I2C1() RCC->APB1ENR |= RCC_APB1ENR_I2C1EN
#define RCC_CLOCK_ENABLE_USB() RCC->APB1ENR |= RCC_APB1ENR_USBEN
#define RCC_CLOCK_ENABLE_CAN() RCC->APB1ENR |= RCC_APB1ENR_CAN1EN
#define RCC_CLOCK_ENABLE_BKP() RCC->APB1ENR |= RCC_APB1ENR_BKPEN
#define RCC_CLOCK_ENABLE_PWR() RCC->APB1ENR |= RCC_APB1ENR_PWREN

void md_rcc_configure_sysclk(rcc_sysclk_source_t sysclk_source,
                             rcc_pll_source_t pll_source,
                             rcc_pll1_mul_t pll1_mul, rcc_hse_div_t hse_div);

void md_rcc_configure_prescalers(rcc_ahb_prescaler_t ahb_prescaler,
                                 rcc_apb_prescaler_t apb1_prescaler,
                                 rcc_apb_prescaler_t apb2_prescaler,
                                 rcc_adc_prescaler_t adc_prescaler);

uint32_t md_rcc_get_sysclk(void);
uint32_t md_rcc_get_hclk(void);
uint32_t md_rcc_get_pclk(uint32_t pclk_x);
uint32_t md_rcc_get_adcclk(void);
void md_rcc_get_frequencies(rcc_clock_freqs_t *p_clock_freqs);

#endif /* MYDRIVERS_INC_STM32F103XX_RCC_H_ */
