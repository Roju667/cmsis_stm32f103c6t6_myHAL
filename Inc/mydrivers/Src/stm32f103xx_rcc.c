/*
 * stm32f103xx_rcc.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#include "stm32f103xx_rcc.h"



static uint16_t rcc_get_ahb_prescaler(void);
static uint16_t rcc_get_apb_prescaler(uint8_t pclk);

void md_rcc_configure_clock_source(void)
{

}

/*
 * Get system clock frequency depending on selected source
 * HSI/HSE speed must be predefined @rcc_oscillators_frequencies
 * @param[void]
 * @return - void
 */
uint32_t md_rcc_get_sysclk(void)
{
	// Calculate sysclk depending on source
	switch (RCC->CFGR & RCC_CFGR_SWS)
	{
	case (RCC_CFGR_SWS_HSI):
		return RCC_HSI_FREQUENCY;
		break;

	case (RCC_CFGR_SWS_HSE):
		return RCC_HSE_FREQUENCY;
		break;

//	case (RCC_CFGR_SWS_PLL):
//		switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC)
//		{
//		case (RCC_PLLCFGR_PLLSRC_HSI):
//			return RCC_CalculatePllclk(RCC_HSI_FREQUENCY);
//			break;
//
//		case (RCC_PLLCFGR_PLLSRC_HSE):
//			return RCC_CalculatePllclk(RCC_HSE_FREQUENCY);
//			break;
//		}
//		break;
	}

	return 0;
}

/*
 * Calculate ahb clock frequency
 * @param[void]
 * @return - void
 */
uint32_t md_rcc_get_hclk(void)
{
	uint32_t sysclk = md_rcc_get_sysclk();
	uint16_t ahb_prescaler = rcc_get_ahb_prescaler();
	return sysclk / ahb_prescaler;
}


uint32_t md_rcc_get_pclk(uint8_t plck_x)
{
	uint32_t hclk = md_rcc_get_hclk();
	uint8_t apb_prescaler = rcc_get_apb_prescaler(plck_x);
	return hclk / apb_prescaler;
}

/*
 * Save all the clock frequencies in clock_freqs struct
 * @param[*p_clock_freqs] - pointer to frequencies struct
 * @return - void
 */
void md_rcc_get_frequencies(rcc_clock_freqs_t *p_clock_freqs)
{
	p_clock_freqs->sysclk = md_rcc_get_sysclk();
	p_clock_freqs->hclk = md_rcc_get_hclk();
	p_clock_freqs->pclk1 = md_rcc_get_pclk(1);
	p_clock_freqs->pclk2 = md_rcc_get_pclk(2);

	return;
}


/*
 * Change bit value from ahb prescaler register to uint number
 * @param[void]
 * @return - void
 */
static uint16_t rcc_get_ahb_prescaler(void)
{
	uint32_t ahb_prescaler;
	uint8_t bitvalue = (RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos;

	// convert bit code to prescaler value
	switch (bitvalue)
	{
	case (RCC_HPRE_PRESCALER_NODIV):
		ahb_prescaler = 1;
		break;
	case (RCC_HPRE_PRESCALER_DIV2):
		ahb_prescaler = 2;
		break;
	case (RCC_HPRE_PRESCALER_DIV4):
		ahb_prescaler = 4;
		break;
	case (RCC_HPRE_PRESCALER_DIV8):
		ahb_prescaler = 8;
		break;
	case (RCC_HPRE_PRESCALER_DIV16):
		ahb_prescaler = 16;
		break;
	case (RCC_HPRE_PRESCALER_DIV64):
		ahb_prescaler = 64;
		break;
	case (RCC_HPRE_PRESCALER_DIV128):
		ahb_prescaler = 128;
		break;
	case (RCC_HPRE_PRESCALER_DIV256):
		ahb_prescaler = 256;
		break;
	case (RCC_HPRE_PRESCALER_DIV512):
		ahb_prescaler = 512;
		break;
	}

	return ahb_prescaler;
}

/*
 * Change bit value from apb prescaler register to uint number
 * @param[void]
 * @return - void
 */
static uint16_t rcc_get_apb_prescaler(uint8_t pclk)
{
	uint32_t apb_prescaler;
	uint8_t bitvalue;

	// get bit value from register
	switch(pclk)
	{
	case(1):
		bitvalue = ((RCC->CFGR & RCC_CFGR_PPRE1) >>
		RCC_CFGR_PPRE1_Pos);
			break;
			case(2):
		bitvalue = ((RCC->CFGR & RCC_CFGR_PPRE2) >>
			RCC_CFGR_PPRE2_Pos);
			break;
	}


	// convert bit code to prescaler value
	switch (bitvalue)
	{
	case (RCC_ABP_PRESCALER_NODIV):
		apb_prescaler = 1;
		break;
	case (RCC_ABP_PRESCALER_DIV2):
		apb_prescaler = 2;
		break;
	case (RCC_ABP_PRESCALER_DIV4):
		apb_prescaler = 4;
		break;
	case (RCC_ABP_PRESCALER_DIV8):
		apb_prescaler = 8;
		break;
	case (RCC_ABP_PRESCALER_DIV16):
		apb_prescaler = 16;
		break;
	}

	return apb_prescaler;
}
