/*
 * adc_example.c
 *
 *  Created on: 30 mar 2022
 *      Author: pawel
 */

#include "stm32f103xx_adc.h"
#include "stm32f103xx_systick.h"

#include "rcc_example.h"
#include "gpio_example.h"

void example_adc_init(void)
{

	example_rcc_configure_adc_clock_8Mhz();

	md_systick_configure_ms();

	uint16_t adc_value;

	adc_config_t adc_config;
	adc_config.cont_mode_en = true;
	adc_config.data_alignment_left = false;
	adc_config.scan_mode_en = false;
	adc_config.dma_en = false;
	adc_config.ext_trig_inj = ADC_TRIG_IN_NO_TRIGGER;
	adc_config.ext_trig_reg = ADC_TRIG_REG_NO_TRIGGER;
	adc_config.number_of_conv = 1;

	adc_channel_config_t channel_config;
	channel_config.channel_number = 1;
	channel_config.sample_time = ADC_SAMPLE_TIME_2395;
	channel_config.sequence_place = 0;

	md_adc_init(&hadc1, adc_config);
	md_adc_init_channel(&hadc1, channel_config);



	while(1)
	{
		example_heart_beat();
		adc_value = md_adc_single_conversion_polling(&hadc1);
	}
}
