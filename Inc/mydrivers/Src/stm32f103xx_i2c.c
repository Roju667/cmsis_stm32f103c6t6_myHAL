/*
 * stm32f103xx_i2c.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */


#include "stm32f103xx_i2c.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"

/*
 * Starts clock for I2C and resets the peripheral
 * @param[*pI2Cx] - i2cx base address
 * @return - void
 */
void md_i2c_init_clock(I2C_TypeDef *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		RCC_CLOCK_ENABLE_I2C1();
		RCC_RESET_I2C1();
	}

	return;
}

void md_i2c_init(I2C_TypeDef *pI2Cx)
{

}
