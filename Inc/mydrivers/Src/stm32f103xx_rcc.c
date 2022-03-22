/*
 * stm32f103xx_rcc.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#include "stm32f103xx_rcc.h"

static uint16_t rcc_get_ahb_prescaler(void);
static uint16_t rcc_get_apb_prescaler(uint8_t pclk);
static uint8_t rcc_get_pll_multiplier(void);
static uint32_t rcc_calculate_pll_sysclk(void);
static uint8_t rcc_get_adc_prescaler(void);

/*
 * Configure sysclk source and frequency (in case of using PLL)
 * @param[sysclk_source] - sysclk source enum @rcc_sysclk_source
 * @param[pll_source] - pll source enum @rcc_pll_source
 * @param[pll1_mul] - pll1 multiplier enum @rcc_pll1_mul
 * @param[hse_div] - pll1 hse division factor @rcc_hse_div
 * @return - void
 */
void md_rcc_configure_sysclk(rcc_sysclk_source_t sysclk_source,
                             rcc_pll_source_t pll_source,
                             rcc_pll1_mul_t pll1_mul, rcc_hse_div_t hse_div) {
  // enable HSI
  if (sysclk_source == RCC_SYSCLK_SOURCE_HSI ||
      pll_source == RCC_PLL_SOURCE_HSI) {
    SET_BIT(RCC->CR, RCC_CR_HSION);
    // wait until HSI is ready
    while (!(RCC->CR & RCC_CR_HSIRDY))
      ;
  }

  // enable HSE
  if (sysclk_source == RCC_SYSCLK_SOURCE_HSE ||
      pll_source == RCC_PLL_SOURCE_HSE) {
    SET_BIT(RCC->CR, RCC_CR_HSEON);
    // wait until HSE is ready
    while (!(RCC->CR & RCC_CR_HSERDY))
      ;
  }

  // configure PLL
  if (sysclk_source == RCC_SYSCLK_SOURCE_PLL) {
    // choose PLL source and multiplier
    RCC->CFGR &= ~(RCC_CFGR_PLLSRC);
    RCC->CFGR |= (pll_source << RCC_CFGR_PLLSRC_Pos);

    RCC->CFGR &= ~(RCC_CFGR_PLLMULL);
    RCC->CFGR |= (pll1_mul << RCC_CFGR_PLLMULL_Pos);

    // if source is HSE choose prediv
    if (pll_source == RCC_PLL_SOURCE_HSE) {
      RCC->CFGR &= ~(RCC_CFGR_PLLXTPRE);
      RCC->CFGR |= (hse_div << RCC_CFGR_PLLXTPRE_Pos);
    }

    // enable PLL
    SET_BIT(RCC->CR, RCC_CR_PLLON);
    // wait until PLL is ready
    while (!(RCC->CR & RCC_CR_PLLRDY))
      ;
  }

  // change sysclk source
  RCC->CFGR |= (sysclk_source << RCC_CFGR_SW_Pos);

   //wait until sys clock is switched
   while (!(RCC->CFGR & (sysclk_source << 2U)))
      ;

  return;
}

/*
 * Configure prescaler for all the peripheral clocks
 * @param[ahb_prescaler] - ahb prescaler enum @rcc_ahb_prescaler
 * @param[apb1_prescaler] - apb prescaler enum @rcc_apb_prescaler
 * @param[apb2_prescaler] - apb prescaler enum @rcc_apb_prescaler
 * @param[adc_prescaler] - adc prescaler enum @rcc_adc_prescaler
 * @return - void
 */
void md_rcc_configure_prescalers(rcc_ahb_prescaler_t ahb_prescaler,
                                 rcc_apb_prescaler_t apb1_prescaler,
                                 rcc_apb_prescaler_t apb2_prescaler,
                                 rcc_adc_prescaler_t adc_prescaler) {
  // configure ahb prescaler
  RCC->CFGR |= (ahb_prescaler << RCC_CFGR_HPRE_Pos);

  // configure apb1/apb2 prescalers
  RCC->CFGR |= (apb1_prescaler << RCC_CFGR_PPRE1_Pos);
  RCC->CFGR |= (apb2_prescaler << RCC_CFGR_PPRE2_Pos);

  // configure adc prescaler
  RCC->CFGR |= (adc_prescaler << RCC_CFGR_ADCPRE_Pos);

  return;
}
/*
 * Get system clock frequency depending on selected source
 * HSI/HSE speed must be predefined @rcc_oscillators_frequencies
 * @param[void]
 * @return - void
 */
uint32_t md_rcc_get_sysclk(void) {
  // Calculate sysclk depending on source
  switch (RCC->CFGR & RCC_CFGR_SWS) {
    case (RCC_CFGR_SWS_HSI):
      return RCC_HSI_FREQUENCY;
      break;

    case (RCC_CFGR_SWS_HSE):
      return RCC_HSE_FREQUENCY;
      break;

    case (RCC_CFGR_SWS_PLL):
      return rcc_calculate_pll_sysclk();
      break;
  }

  return 0;
}

/*
 * Calculate ahb clock frequency
 * @param[void]
 * @return - hclk frequency
 */
uint32_t md_rcc_get_hclk(void) {
  uint32_t sysclk = md_rcc_get_sysclk();
  uint16_t ahb_prescaler = rcc_get_ahb_prescaler();
  return sysclk / ahb_prescaler;
}

/*
 * Calculate apb clock frequency
 * @param[pclk_x] - 1/2 pclk number
 * @return - pclk frequency
 */
uint32_t md_rcc_get_pclk(uint8_t pclk_x) {
  uint32_t hclk = md_rcc_get_hclk();
  uint8_t apb_prescaler = rcc_get_apb_prescaler(pclk_x);
  return hclk / apb_prescaler;
}

/*
 * Calculate adc clock frequency
 * @param[void]
 * @return - adcclk frequency
 */
uint32_t md_rcc_get_adcclk(void)
{
	uint32_t pclk2 = md_rcc_get_pclk(2);
	uint8_t adc_prescaler = rcc_get_adc_prescaler();
	return pclk2/adc_prescaler;
}

/*
 * Save all the clock frequencies in clock_freqs struct
 * @param[*p_clock_freqs] - pointer to frequencies struct
 * @return - void
 */
void md_rcc_get_frequencies(rcc_clock_freqs_t *p_clock_freqs) {
  p_clock_freqs->sysclk = md_rcc_get_sysclk();
  p_clock_freqs->hclk = md_rcc_get_hclk();
  p_clock_freqs->pclk1 = md_rcc_get_pclk(1);
  p_clock_freqs->pclk2 = md_rcc_get_pclk(2);
  p_clock_freqs->adcclk = md_rcc_get_adcclk();

  return;
}


/*
 * Change bit value from ahb prescaler register to uint number
 * @param[void]
 * @return - ahb_prescaler value
 */
static uint16_t rcc_get_ahb_prescaler(void) {
  uint32_t ahb_prescaler;
  uint8_t bitvalue = (RCC->CFGR >> RCC_CFGR_HPRE_Pos) & 0x0F;

  // convert bit code to prescaler value
  switch (bitvalue) {
    case (RCC_AHB_PRESCALER_NODIV):
      ahb_prescaler = 1;
      break;
    case (RCC_AHB_PRESCALER_DIV2):
      ahb_prescaler = 2;
      break;
    case (RCC_AHB_PRESCALER_DIV4):
      ahb_prescaler = 4;
      break;
    case (RCC_AHB_PRESCALER_DIV8):
      ahb_prescaler = 8;
      break;
    case (RCC_AHB_PRESCALER_DIV16):
      ahb_prescaler = 16;
      break;
    case (RCC_AHB_PRESCALER_DIV64):
      ahb_prescaler = 64;
      break;
    case (RCC_AHB_PRESCALER_DIV128):
      ahb_prescaler = 128;
      break;
    case (RCC_AHB_PRESCALER_DIV256):
      ahb_prescaler = 256;
      break;
    case (RCC_AHB_PRESCALER_DIV512):
      ahb_prescaler = 512;
      break;
  }

  return ahb_prescaler;
}

/*
 * Change bit value from apb prescaler register to uint number
 * @param[void]
 * @return - apb_prescaler value
 */
static uint16_t rcc_get_apb_prescaler(uint8_t pclk) {
  uint32_t apb_prescaler;
  uint8_t bitvalue;

  // get bit value from register
  switch (pclk) {
    case (1):
      bitvalue = (RCC->CFGR >> RCC_CFGR_PPRE1_Pos) & 0x07;
      break;
    case (2):
      bitvalue = (RCC->CFGR >> RCC_CFGR_PPRE2_Pos) & 0x07;
      break;
  }

  // convert bit code to prescaler value
  switch (bitvalue) {
    case (RCC_APB_PRESCALER_NODIV):
      apb_prescaler = 1;
      break;
    case (RCC_APB_PRESCALER_DIV2):
      apb_prescaler = 2;
      break;
    case (RCC_APB_PRESCALER_DIV4):
      apb_prescaler = 4;
      break;
    case (RCC_APB_PRESCALER_DIV8):
      apb_prescaler = 8;
      break;
    case (RCC_APB_PRESCALER_DIV16):
      apb_prescaler = 16;
      break;
  }

  return apb_prescaler;
}

/*
 * Change bit value from pll multiplier to uint value
 * @param[void]
 * @return - pll multiplier value
 */
static uint8_t rcc_get_pll_multiplier(void) {
  uint8_t bitvalue = (RCC->CFGR >> RCC_CFGR_PLLMULL_Pos) & 0x0F;

  switch (bitvalue) {
    case (RCC_PLL1_MUL_X4):
      return 4;

    case (RCC_PLL1_MUL_X5):
      return 5;

    case (RCC_PLL1_MUL_X6):
      return 6;

    case (RCC_PLL1_MUL_X7):
      return 7;

    case (RCC_PLL1_MUL_X8):
      return 8;

    case (RCC_PLL1_MUL_X9):
      return 9;

    case (RCC_PLL1_MUL_X65):
      return 13;
  }

  return 0;
}

/*
 * Calculate sysclk from PLL parameters
 * @param[void]
 * @return - pll multiplier value
 */
static uint32_t rcc_calculate_pll_sysclk(void) {
  uint8_t pll_multiplier = rcc_get_pll_multiplier();
  uint8_t hse_divider = 1;
  uint32_t sysclk_value = 0;

  // check HSE divider
  if (RCC->CFGR & RCC_CFGR_PLLXTPRE) {
    hse_divider = 2;
  }

  switch (RCC->CFGR & RCC_CFGR_PLLSRC) {
      // source HSI
    case (0):
      sysclk_value = (RCC_HSI_FREQUENCY / 2) * pll_multiplier;
      break;
      // source HSE
    case (RCC_CFGR_PLLSRC):
      sysclk_value = (RCC_HSE_FREQUENCY / hse_divider) * pll_multiplier;
      break;
  }

  // if multiplier is 6,5
  if (pll_multiplier == 13) {
    sysclk_value = sysclk_value / 2;
  }

  return sysclk_value;
}

/*
 * Change bit value of adc prescaler to uint value
 * @param[void]
 * @return - adc prescaler value
 */
static uint8_t rcc_get_adc_prescaler(void)
{
	uint8_t bitvalue = (RCC->CFGR >> RCC_CFGR_ADCPRE_Pos) & 0x03;

	  switch (bitvalue) {
	    case (RCC_ADC_PRESCALER_DIV2):
	      return 2;

	    case (RCC_ADC_PRESCALER_DIV4):
	      return 4;

	    case (RCC_ADC_PRESCALER_DIV6):
	      return 6;

	    case (RCC_ADC_PRESCALER_DIV8):
	      return 8;

	  }

	  return 0;

}
