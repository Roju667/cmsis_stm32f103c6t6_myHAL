/*
 * stm32f103xx.adc.c
 *
 *  Created on: Mar 25, 2022
 *      Author: ROJEK
 */

#include "stm32f103xx_adc.h"
#include "math.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_mydriver.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

#ifdef MD_ENABLE_ADC

#if MD_USING_ADC1
adc_handle_t hadc1;
#endif // MD_USING_ADC1

static void adc_init_handlers(void);
static void adc_init_clock(adc_handle_t *p_hADCx);
static void adc_init_gpio(adc_handle_t *p_hADCx);

/*
 * Init handlers gpio and clock
 * @param[*p_hADCx] - can struct handler @adc_handler
 * @return - void
 */
void md_adc_init(adc_handle_t *p_hADCx) {}
/*
 * ADC basic configuration
 * @param[*p_hADCx] - adcx base address
 * @return - void
 */
void md_adc_init_basic(adc_handle_t *p_hADCx) { return; }

/*
 * Init handler structures
 * @param[void]
 * @return - void
 */
static void adc_init_handlers(void)
{
#if MD_USING_ADC1
  hadc1.p_ADCx = ADC1;
#endif // MD_USING_ADC1
}

/*
 * Starts clock for ADC and resets the peripheral
 * @param[*p_hADCx] - adcx base address
 * @return - void
 */
static void adc_init_clock(adc_handle_t *p_hADCx)
{
  if (RCC->APB2ENR & RCC_APB2ENR_ADC1EN)
    return;

  RCC_CLOCK_ENABLE_ADC1();
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_ADC1RST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_ADC1RST);

  return;
}

/*
 * Init gpio pins for adc - make sure that GPIO clock is enabled before
 * @param[*p_hADCx] - adcx base address
 * @return - void
 */
static void adc_init_gpio(adc_handle_t *p_hADCx) {}

#endif // MD_ENABLE_ADC
