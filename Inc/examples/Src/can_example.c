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
#include "rcc_example.h"

volatile bool send_can_msg;

void example_can_init(void)
{
  // configure systick
  md_systick_configure_ms();
  // configure rcc to get 32 Mhz on APB1 (CAN bus)
  example_rcc_configure_pll_32Mhz();

  // configure gpio button input
  md_gpio_configure_exti(GPIOB, GPIO_PIN_11, GPIO_INPUT_FLOATING,
                         GPIO_EXTI_FALLING, 40);

  // configure gpio led output
  md_gpio_configure_output(GPIOC, GPIO_PIN_13, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  // init low level - handler/clock/gpio
  md_can_init(&hcan1);

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
  basic_init.auto_retransmit = false;
  basic_init.rx_fifo_lock = false;
  basic_init.tx_fifo_prio = false;

  // can configure filter
  can_filter_t can_filter;
  can_filter.assign_to_fifo1 = false;
  can_filter.filter_id0 = 0;
  can_filter.filter_mask_or_id1 = 0xFFFF;
  can_filter.filter_number = 0;
  can_filter.list_mode = false;
  can_filter.scale_32bit = false;

  // enter init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_INIT, 1000);
  md_can_init_basic(&hcan1, basic_init);
  md_can_init_time_quanta(&hcan1, time_quanta);
  md_can_activate_irq(&hcan1, CAN_IRQ_GROUP_TX, CAN_IER_TMEIE, 30);
  md_can_activate_irq(&hcan1, CAN_IRQ_GROUP_RX0,
                      CAN_IER_FMPIE0 | CAN_IER_FFIE0 | CAN_IER_FOVIE0, 20);
  md_can_activate_irq(&hcan1, CAN_IRQ_GROUP_RX1,
                      CAN_IER_FMPIE1 | CAN_IER_FFIE1 | CAN_IER_FOVIE1, 20);
  md_can_init_filter(&hcan1, can_filter);
  //  md_can_enter_test_mode(&hcan1, CAN_TESTMODE_LOOPBACK);
  //   exit init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_NORMAL, 1000);

  // send message
  uint8_t data_buffer[8] = "cantest1";
  can_frame_t frame, recieve_frame;
  frame.data_lenght = 8;
  frame.id = 0;
  frame.id_extended = false;
  frame.remote = false;
  frame.p_data_buffer = data_buffer;

  uint8_t recieve_data_buffer[8];
  uint8_t mailbox_number;

  while (1)
    {
      example_heart_beat();

      if (send_can_msg)
        {
          send_can_msg = 0;
          md_can_write_mailbox(&hcan1, frame, &mailbox_number);
        }

      if (hcan1.msg_pending_fifo0 == 1)
        {
          md_can_read_fifo(&hcan1, &recieve_frame, recieve_data_buffer, 0);
        }
    }
}

void md_gpio_exti15_10_callback(gpio_pin_number_t pin_number)
{
  send_can_msg = 1;
}

void md_can_mailbox_empty_callback(void) {}
