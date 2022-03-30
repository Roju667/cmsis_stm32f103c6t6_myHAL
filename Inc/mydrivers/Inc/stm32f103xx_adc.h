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
typedef enum
{
  ADC_ERR_NOERR

} adc_error_t;

// @adc_config
typedef struct
{
  bool master_mode;

} adc_config_t;

// @adc_sample_time
typedef enum
{
  ADC_SAMPLE_TIME_15,
  ADC_SAMPLE_TIME_75,
  ADC_SAMPLE_TIME_135,
  ADC_SAMPLE_TIME_285,
  ADC_SAMPLE_TIME_415,
  ADC_SAMPLE_TIME_555,
  ADC_SAMPLE_TIME_715,
  ADC_SAMPLE_TIME_2395
} adc_sample_time_t;

// @adc_trigger_injected
typedef enum
{
  ADC_TRIG_IN_TIM1TRGO,
  ADC_TRIG_IN_TIM1CC4,
  ADC_TRIG_IN_TIM2TRGO,
  ADC_TRIG_IN_TIM2CC1,
  ADC_TRIG_IN_TIM3CC4,
  ADC_TRIG_IN_TIM4TRGO,
  ADC_TRIG_IN_EXTI15,
  ADC_TRIG_IN_JSWSTART
} adc_trigger_in_t;

// @adc_trigger_regular
typedef enum
{
  ADC_TRIG_REG_TIM1CC1,
  ADC_TRIG_REG_TIM1CC2,
  ADC_TRIG_REG_TIM1CC3,
  ADC_TRIG_REG_TIM2CC2,
  ADC_TRIG_REG_TIM3TRGO,
  ADC_TRIG_REG_TIM4CC4,
  ADC_TRIG_REG_EXTI11,
  ADC_TRIG_REG_SWSTART
} adc_trigger_reg_t;

// @adc_handle
typedef struct
{
  ADC_TypeDef *p_ADCx;
} adc_handle_t;

#if MD_USING_ADC1
extern adc_handle_t hadc1;
#endif // MD_USING_ADC1

#endif // MD_ENABLE_ADC

#endif /* MYDRIVERS_INC_STM32F103XX_ADC_H_ */
