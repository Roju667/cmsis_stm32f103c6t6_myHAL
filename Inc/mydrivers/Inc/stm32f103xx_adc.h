/*
 * stm32f103xx.adc.h
 *
 *  Created on: Mar 25, 2022
 *      Author: ROJEK
 */

#ifndef MYDRIVERS_INC_STM32F103XX_ADC_H_
#define MYDRIVERS_INC_STM32F103XX_ADC_H_

#include "stdbool.h"
#include "stm32f1xx.h"

// defining those macros will allocate static data
// for purpose of using DMA/IRQ
#define MD_USING_ADC1 1U

#if MD_USING_ADC1
#define MD_ENABLE_ADC
#endif // MD_USING_SPI1

#ifdef MD_ENABLE_ADC

// @adc_error
typedef enum adc_error_t
{
  ADC_ERR_NOERR = 0

} adc_error_t;

// @adc_sample_time
typedef enum adc_sample_time_t
{
  ADC_SAMPLE_TIME_15 = 0,
  ADC_SAMPLE_TIME_75,
  ADC_SAMPLE_TIME_135,
  ADC_SAMPLE_TIME_285,
  ADC_SAMPLE_TIME_415,
  ADC_SAMPLE_TIME_555,
  ADC_SAMPLE_TIME_715,
  ADC_SAMPLE_TIME_2395
} adc_sample_time_t;

// @adc_trigger_injected
typedef enum adc_trigger_in_t
{
  ADC_TRIG_IN_TIM1TRGO = 0,
  ADC_TRIG_IN_TIM1CC4,
  ADC_TRIG_IN_TIM2TRGO,
  ADC_TRIG_IN_TIM2CC1,
  ADC_TRIG_IN_TIM3CC4,
  ADC_TRIG_IN_TIM4TRGO,
  ADC_TRIG_IN_EXTI15,
  ADC_TRIG_IN_JSWSTART,
  ADC_TRIG_IN_NO_TRIGGER
} adc_trigger_in_t;

// @adc_trigger_regular
typedef enum adc_trigger_reg_t
{
  ADC_TRIG_REG_TIM1CC1 = 0,
  ADC_TRIG_REG_TIM1CC2,
  ADC_TRIG_REG_TIM1CC3,
  ADC_TRIG_REG_TIM2CC2,
  ADC_TRIG_REG_TIM3TRGO,
  ADC_TRIG_REG_TIM4CC4,
  ADC_TRIG_REG_EXTI11,
  ADC_TRIG_REG_SWSTART,
  ADC_TRIG_REG_NO_TRIGGER
} adc_trigger_reg_t;

// @adc_analog_wdg
typedef struct adc_analog_wdg
{
  bool watch_single_channel;
  bool watch_regular;
  bool watch_injected;
  uint32_t channel_number;
  uint32_t high_threshold;
  uint32_t low_threshold;
} adc_analog_wdg;

// @adc_channel_config
typedef struct adc_channel_config_t
{
  uint32_t channel_number;
  uint32_t sequence_place;
  adc_sample_time_t sample_time;
} adc_channel_config_t;

// @adc_config
typedef struct adc_config_t
{
  bool scan_mode_en;
  bool cont_mode_en;
  uint32_t number_of_conv;
  adc_trigger_reg_t ext_trig_reg;
  adc_trigger_in_t ext_trig_inj;
  bool data_alignment_left;
  bool dma_en;
} adc_config_t;

// @adc_handle
typedef struct adc_handle_t
{
  ADC_TypeDef *p_ADCx;
} adc_handle_t;

void md_adc_init_channel(adc_handle_t *p_hADCx, adc_channel_config_t *p_config);
void md_adc_init(adc_handle_t *p_hADCx, adc_config_t *p_adc_config);
uint16_t md_adc_single_conversion_polling(adc_handle_t *p_hADCx);

#if MD_USING_ADC1
extern adc_handle_t hadc1;
#endif // MD_USING_ADC1

#endif // MD_ENABLE_ADC

#endif /* MYDRIVERS_INC_STM32F103XX_ADC_H_ */
