/*
 * stm32f103xx_gpio.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */


#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"

/*
 * Starts clock for GPIO and resets the peripheral
 * @param[*p_GPIOx] - gpiox base address
 * @return - void
 */
void md_gpio_init_clock(GPIO_TypeDef *p_GPIOx)
{
	if(p_GPIOx == GPIOA)
	{
		RCC_CLOCK_ENABLE_IOPA();
		RCC_RESET_IOPA();
	}else if(p_GPIOx == GPIOB)
	{
		RCC_CLOCK_ENABLE_IOPB();
		RCC_RESET_IOPB();
	}else if(p_GPIOx == GPIOC)
	{
		RCC_CLOCK_ENABLE_IOPC();
		RCC_RESET_IOPC();
	}else if(p_GPIOx)
	{
		RCC_CLOCK_ENABLE_IOPD();
		RCC_RESET_IOPD();
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
	RCC_CLOCK_ENABLE_AFIO();
	RCC_RESET_AFIO();

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
void md_gpio_configure_output(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,gpio_speed_t output_speed,gpio_ouput_config_t output_conifg)
{
	// choose between CRL/CRH
	if(pin_number < 8)
	{
		// clear config and mode bits
		p_GPIOx->CRL &= ~(0x0F << (pin_number * 4));

		// set mode and config
		p_GPIOx->CRL |= (output_speed << (pin_number * 4));
		p_GPIOx->CRL |= (output_conifg << ((pin_number *4) + 2));
	}else
	{
		// clear config and mode bits
		p_GPIOx->CRH &= ~(0x0F << ((pin_number - 8) * 4));

		// set mode and config
		p_GPIOx->CRH |= (output_speed << ((pin_number - 8) * 4));
		p_GPIOx->CRH |= (output_conifg << (((pin_number - 8) *4) + 2));
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
void md_gpio_configure_input(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number,gpio_input_config input_config)
{
	// choose between CRL/CRH
	if(pin_number < 8)
	{
		// clear config and mode bits
		p_GPIOx->CRL &= ~(0x0F << (pin_number * 4));

		// set config
		if(input_config > GPIO_INPUT_FLOATING)
		{
			p_GPIOx->CRL |= (GPIO_INPUT_PULLUP << ((pin_number *4) + 2));
		}else
		{
			p_GPIOx->CRL |= (input_config << ((pin_number *4) + 2));
		}


	}else
	{
		// clear config and mode bits
		p_GPIOx->CRH &= ~(0x0F << ((pin_number - 8) * 4));

		// set config
		if(input_config > GPIO_INPUT_FLOATING)
		{
			p_GPIOx->CRH |= (GPIO_INPUT_PULLUP << (((pin_number - 8) *4) + 2));
		}else
		{
			p_GPIOx->CRH |= (input_config << (((pin_number - 8) *4) + 2));
		}

	}

	// set or reset ouput register - pullup must have 1 in ODR
	if(input_config == GPIO_INPUT_PULLUP)
	{
		p_GPIOx->BSRR = (0x01 << pin_number);
	}else
	{
		p_GPIOx->BRR = (0x01 << pin_number);
	}

	return;
}

void md_gpio_configure_exti(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number)
{
	uint8_t code = GPIO_BASEADDR_TO_CODE(p_GPIOx);

	if(pin_number < 4)
	{

	}else if(pin_number > 3 && pin_number < 8)
	{

	}else if(pin_number > 7 && pin_number < 12)
	{

	}else if(pin_number >11)
	{

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
void md_gpio_write_pin(GPIO_TypeDef *p_GPIOx, gpio_pin_number_t pin_number, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		p_GPIOx->BSRR = (0x01 << pin_number);
	}else if(value == GPIO_PIN_RESET)
	{
		p_GPIOx->BRR = (0x01 << pin_number);
	}

	return;
}

