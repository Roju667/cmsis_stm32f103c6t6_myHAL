/*
 * stm32f103xx_mydriver.h
 *
 *  Created on: 21 mar 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_MYDRIVER_H_
#define MYDRIVERS_INC_STM32F103XX_MYDRIVER_H_

#include "stdbool.h"
#include "stm32f1xx.h"

#define __weak __attribute__((weak))

static inline void md_set_if_condition(bool condition,volatile uint32_t *reg,
                                     uint32_t bit)
{
  if (condition == true)
    {
      // set
      *reg |= bit;
    }
  else
    {
      // reset
      *reg &= ~(bit);
    }
}

static inline void md_clear_if_condition(bool condition,volatile uint32_t *reg,
                                     uint32_t bit)
{
  if (condition == false)
    {
      // set
      *reg |= bit;
    }
  else
    {
      // reset
      *reg &= ~(bit);
    }
}

#endif /* MYDRIVERS_INC_STM32F103XX_MYDRIVER_H_ */
