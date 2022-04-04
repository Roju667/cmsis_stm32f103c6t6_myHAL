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
adc_handle_t hadc1 ={.p_ADCx = ADC1};
#endif // MD_USING_ADC1

static void adc_init_clock(adc_handle_t *p_hADCx);
static void adc_init_gpio(adc_handle_t *p_hADCx, uint8_t adc_channel);

/*
 * Init handlers gpio and clock
 * @param[*p_hADCx] - can struct handler @adc_handler
 * @param[*p_adc_config] - pointer to adc config structure @adc_config
 * @return - void
 */
void md_adc_init(adc_handle_t *p_hADCx, adc_config_t *p_adc_config)
{

  adc_init_clock(p_hADCx);

  // scan mode
  md_set_if_condition(p_adc_config->scan_mode_en, &(p_hADCx->p_ADCx->CR1),
                      ADC_CR1_SCAN);
  // continious
  md_set_if_condition(p_adc_config->cont_mode_en, &(p_hADCx->p_ADCx->CR2),
                      ADC_CR2_CONT);
  // data allignment
  md_set_if_condition(p_adc_config->data_alignment_left,
                      &(p_hADCx->p_ADCx->CR2), ADC_CR2_ALIGN);
  // dma
  md_set_if_condition(p_adc_config->dma_en, &(p_hADCx->p_ADCx->CR2),
                      ADC_CR2_DMA);

  // trigger mode reg
  if (p_adc_config->ext_trig_reg < ADC_TRIG_REG_NO_TRIGGER)
    {
      SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_EXTTRIG);
      p_hADCx->p_ADCx->CR2 &= ~(ADC_CR2_EXTSEL);
      p_hADCx->p_ADCx->CR2 |=
          (p_adc_config->ext_trig_reg << ADC_CR2_EXTSEL_Pos);
    }

  // trigger mode inj
  if (p_adc_config->ext_trig_inj < ADC_TRIG_IN_NO_TRIGGER)
    {
      SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_JEXTTRIG);
      p_hADCx->p_ADCx->CR2 &= ~(ADC_CR2_JEXTSEL);
      p_hADCx->p_ADCx->CR2 |=
          (p_adc_config->ext_trig_inj << ADC_CR2_JEXTSEL_Pos);
    }

  // number of conversions
  p_hADCx->p_ADCx->SQR1 &= ~(ADC_SQR1_L);
  p_hADCx->p_ADCx->SQR1 |=
      ((p_adc_config->number_of_conv - 1) << ADC_SQR1_L_Pos);

  // start adc
  SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_ADON);
  md_systick_delay(1);
  // calibration
  SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_RSTCAL);

  // wait until register init is finished
  while (p_hADCx->p_ADCx->CR2 & ADC_CR2_RSTCAL)
    ;

  // enable calibration
  SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_CAL);

  // wait until calibration is finished
  while (p_hADCx->p_ADCx->CR2 & ADC_CR2_CAL)
    ;

  return;
}
/*
 * ADC channel configuration
 * @param[*p_hADCx] - adcx base address @adc_handler
 * @param[*p_config] - pointer to adc channel configuration struct
 * @adc_channel_config
 * @return - void
 */
void md_adc_init_channel(adc_handle_t *p_hADCx, adc_channel_config_t *p_config)
{
  adc_init_gpio(p_hADCx, p_config->channel_number);

  // sample time
  if (p_config->channel_number > 10)
    {
      p_hADCx->p_ADCx->SMPR1 &=
          ~(ADC_SMPR1_SMP10 << ((p_config->channel_number % 10) * 3));
      p_hADCx->p_ADCx->SMPR1 |=
          (p_config->sample_time << ((p_config->channel_number % 10) * 3));
    }
  else
    {
      p_hADCx->p_ADCx->SMPR2 &=
          ~(ADC_SMPR2_SMP0 << ((p_config->channel_number) * 3));
      p_hADCx->p_ADCx->SMPR2 |=
          (p_config->sample_time << ((p_config->channel_number) * 3));
    }

  // sequence position
  if (p_config->channel_number < 7)
    {
      p_hADCx->p_ADCx->SQR3 &=
          ~(ADC_SQR3_SQ1 << (p_config->sequence_place * 5));
      p_hADCx->p_ADCx->SQR3 |=
          (p_config->channel_number << (p_config->sequence_place * 5));
    }
  else if (p_config->channel_number < 13)
    {
      p_hADCx->p_ADCx->SQR2 &=
          ~(ADC_SQR2_SQ7 << ((p_config->sequence_place % 7) * 5));
      p_hADCx->p_ADCx->SQR2 |=
          (p_config->channel_number << ((p_config->sequence_place % 7) * 5));
    }
  else
    {
      p_hADCx->p_ADCx->SQR1 &=
          ~(ADC_SQR1_SQ13 << ((p_config->sequence_place % 13) * 5));
      p_hADCx->p_ADCx->SQR1 |=
          (p_config->channel_number << ((p_config->sequence_place % 13) * 5));
    }

  return;
}

/*
 * Init analog watchdog
 * @param[*p_hADCx] - adcx base address	@adc_handle
 * @param[*p_wdg_config] - pointer to config struct for watchdog @adc_analog_wdg
 * @return - void
 */
void md_adc_init_awdg(adc_handle_t *p_hADCx, adc_analog_wdg *p_wdg_config) {}

/*
 * ADC single conversion in polling mode
 * @param[*p_hADCx] - adcx base address	@adc_handle
 * @return - void
 */
uint16_t md_adc_single_conversion_polling(adc_handle_t *p_hADCx)
{
  uint16_t adc_value = 0;

  SET_BIT(p_hADCx->p_ADCx->CR2, ADC_CR2_ADON);
  md_systick_delay(1);
  // wait until data is ready
  while (!(p_hADCx->p_ADCx->SR & ADC_SR_EOC))
    ;

  adc_value = (p_hADCx->p_ADCx->DR & 0x0FFF);
  return adc_value;
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
 * @param[adc_channel] - channel number
 * @return - void
 */
static void adc_init_gpio(adc_handle_t *p_hADCx, uint8_t adc_channel)
{
  GPIO_TypeDef *adc_pin_port = NULL;
  gpio_pin_number_t adc_pin_number = 0;

  if (adc_channel < 8)
    {
      adc_pin_port = GPIOA;
      adc_pin_number = adc_channel;
    }
  else if (adc_channel < 10)
    {
      adc_pin_port = GPIOB;
      adc_pin_number = adc_channel - 8;
    }
  else
    {
      adc_pin_port = GPIOC;
      adc_pin_number = adc_channel - 10;
    }

  md_gpio_configure_input(adc_pin_port, adc_pin_number, GPIO_INPUT_ANALOG);

  return;
}

#endif // MD_ENABLE_ADC
