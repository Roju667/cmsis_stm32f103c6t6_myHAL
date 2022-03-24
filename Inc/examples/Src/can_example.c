/*
 * can_example.c
 *
 *  Created on: 24 mar 2022
 *      Author: ROJEK
 */

#include "stm32f103xx_can.h"
#include "stm32f103xx_systick.h"
#include "stm32f103xx_gpio.h"

#include "gpio_example.h"

void example_can_init(void)
{
	md_can_init_handlers();
	md_can_init_clock(&hcan1);
	md_can_change_op_mode(&hcan1, CAN_OPMODE_INIT, 1000);
	md_can_init_time_quanta(&hcan1, 2, CAN_TIME_QUANTA12, CAN_TIME_QUANTA2, CAN_TIME_QUANTA1);
	md_can_change_op_mode(&hcan1, CAN_OPMODE_NORMAL, 1000);

	while(1)
	{
		example_heart_beat();
	}
}
