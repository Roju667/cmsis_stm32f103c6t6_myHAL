/*
 * can_example.c
 *
 *  Created on: 24 mar 2022
 *      Author: ROJEK
 */

#include "stm32f103xx_can.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

#include "gpio_example.h"

void example_can_init(void)
{
  // configure systick
  md_systick_configure_ms();
  // configure rcc to get 32 Mhz on APB1 (CAN bus)

  rcc_clock_freqs_t frequencies = {0};

  // configure frequencies - no prescaler on buses + pll on mul 4x
  md_rcc_configure_prescalers(RCC_AHB_PRESCALER_NODIV, RCC_APB_PRESCALER_NODIV,
                              RCC_APB_PRESCALER_NODIV, RCC_ADC_PRESCALER_DIV6);
  md_rcc_configure_sysclk(RCC_SYSCLK_SOURCE_PLL, RCC_PLL_SOURCE_HSE,
                          RCC_PLL1_MUL_X4, RCC_HSE_DIV_NODIV);
  md_rcc_get_frequencies(&frequencies);

  // init low level - handler/clock/gpio
  md_can_init_handlers();
  md_can_init_clock(&hcan1);
  md_gpio_init_clock(GPIOA);
  md_can_init_gpio(&hcan1);

  // can time quanta configuartion
  can_quanta_init_t time_quanta;
  time_quanta.prescaler = 2;
  time_quanta.quanta_ts1 = CAN_TIME_QUANTA13;
  time_quanta.quanta_ts2 = CAN_TIME_QUANTA2;
  time_quanta.quanta_sjw = CAN_TIME_QUANTA1;

  // can basic configuration
  can_basic_init_t basic_init;
  basic_init.auto_bus_off = false;
  basic_init.debug_freeze = false;
  basic_init.time_triggered_comm = false;
  basic_init.auto_wake_up = true;
  basic_init.auto_retransmit = true;
  basic_init.rx_fifo_lock = false;
  basic_init.tx_fifo_prio = false;

  // enter init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_INIT, 1000);
  md_can_init_basic(&hcan1, basic_init);
  md_can_init_time_quanta(&hcan1, time_quanta);
  md_can_enter_test_mode(&hcan1, CAN_TESTMODE_SILENTLOOPBACK);
  // exit init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_NORMAL, 1000);

  // send message
  can_frame_t frame;
  frame.data_lenght = 8;
  frame.id = 35;
  frame.id_extended = false;
  frame.remote = false;

  uint8_t mailbox_number;
  uint8_t databuffer[8] = "cantest1";

  md_can_write_mailbox(&hcan1, frame, databuffer,&mailbox_number);

  while (1)
    {
      example_heart_beat();
;
    }
}
