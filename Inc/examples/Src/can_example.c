/*
 * can_example.c
 *
 *  Created on: 24 mar 2022
 *      Author: ROJEK
 */

#include "stm32f103xx_adc.h"
#include "stm32f103xx_can.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_spi.h"
#include "stm32f103xx_systick.h"

#include "adc_example.h"
#include "gpio_example.h"
#include "rcc_example.h"

#include "GFX_COLOR.h"
#include "ILI9341.h"
#include "fonts.h"
#include "string.h"
#include "temt6000.h"

volatile bool send_can_msg;
volatile uint8_t can_data_read = 1;

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
  can_frame_t frame;
  frame.data_lenght = 8;
  frame.id = 0;
  frame.id_extended = false;
  frame.remote = false;
  frame.p_data_buffer = data_buffer;
  uint32_t time_tick_led = md_systick_get_tick();

  while (1)
    {
      example_heart_beat_no_delay(&time_tick_led, 500);
    }
}

void example_can_send_data(void)
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
  md_can_init_filter(&hcan1, can_filter);
  //   exit init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_NORMAL, 1000);

  // config ADC
  example_adc_init();
  temt6000_data_t temt_data;
  uint16_t adc_value;

  // send message
  can_frame_t frame;
  frame.data_lenght = 8;
  frame.id = 0;
  frame.id_extended = false;
  frame.remote = false;
  uint8_t data_buffer[8] = {0};

  uint8_t mailbox_number;

  uint32_t time_tick_adc = md_systick_get_tick();
  uint32_t time_tick_led = md_systick_get_tick();
  uint8_t data_ready;

  while (1)
    {

      // read adc every 10 ms

      if (md_systick_get_tick() - time_tick_adc > 10)
        {
          adc_value = md_adc_single_conversion_polling(&hadc1);
          temt6000_get_data(adc_value, &temt_data);
          memcpy(&data_buffer[0], &temt_data.lux, 2);
          memcpy(&data_buffer[2], &temt_data.microamps, 2);
          memcpy(&data_buffer[4], &temt_data.milivolts, 2);
          frame.p_data_buffer = data_buffer;
          data_ready = 1;
          time_tick_adc = md_systick_get_tick();
        }

      example_heart_beat_no_delay(&time_tick_led, 500);

      if (data_ready)
        {
          md_can_write_mailbox(&hcan1, frame, &mailbox_number);
        }
    }
}

void example_can_recieve_data(void)
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
  md_can_init_filter(&hcan1, can_filter);
  //   exit init mode
  md_can_change_op_mode(&hcan1, CAN_OPMODE_NORMAL, 1000);

  // init frame
  can_frame_t frame;

  // INIT SPI AND TFT
  md_spi_init(&hspi1);

  spi_config_t spi_config;
  spi_config.clock_1_when_idle = false;
  spi_config.clock_second_edge_capture = false;
  spi_config.data_format_16bit = false;
  spi_config.full_duplex = true;
  spi_config.lsb_first = false;
  spi_config.master_mode = true;
  spi_config.prescaler = SPI_PRESCALER_2; // 100kHz
  spi_config.software_nss_management = true;

  md_spi_init_basic(&hspi1, spi_config);

  // init gpio for tft
  //  DC
  md_gpio_configure_output(GPIOB, GPIO_PIN_1, GPIO_SPEED_10MHZ, GPIO_OUTPUT_PP);
  // RESET
  md_gpio_configure_output(GPIOB, GPIO_PIN_10, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  ILI9341_Init(&hspi1);
  ILI9341_ClearDisplay(ILI9341_BLACK);

  GFX_SetFont(font_8x5);
  GFX_SetFontSize(2);
  GFX_DrawString(10, 10, "Can message recieved", ILI9341_YELLOW);

  md_can_activate_irq(&hcan1, CAN_IRQ_GROUP_RX0, CAN_IER_FMPIE0, 20);
  md_can_activate_irq(&hcan1, CAN_IRQ_GROUP_RX1, CAN_IER_FMPIE1, 20);


  uint32_t time_tick_led = md_systick_get_tick();
  uint8_t data_buffer[8];
  uint8_t message[16];
  uint16_t milivolts, microamps, lux;

  while (1)
    {
      example_heart_beat_no_delay(&time_tick_led, 500);



          md_can_read_fifo(&hcan1, &frame, data_buffer, 0);
            lux = (data_buffer[0] | (data_buffer[1] << 8));
            microamps = (data_buffer[2] | (data_buffer[3] << 8));
            milivolts = (data_buffer[4] | (data_buffer[5] << 8));
            sprintf((char *)message, "Lux : %d", lux);
            ILI9341_ClearArea(ILI9341_BLACK, 10, 10, 220, 20);
            GFX_DrawString(10, 10, (char*)message, ILI9341_YELLOW);
            sprintf((char *)message, "Microamps : %d", microamps);
            ILI9341_ClearArea(ILI9341_BLACK, 10, 50, 220, 20);
            GFX_DrawString(10, 50, (char*)message, ILI9341_YELLOW);
            sprintf((char *)message, "Milivolts : %d", milivolts);
            ILI9341_ClearArea(ILI9341_BLACK, 10, 90, 220, 20);
            GFX_DrawString(10, 90, (char*)message, ILI9341_YELLOW);
            SET_BIT(CAN1->IER, CAN_IER_FMPIE0);


    }
}

void md_gpio_exti15_10_callback(gpio_pin_number_t pin_number)
{
  send_can_msg = 1;
}

void md_can_mailbox_empty_callback(void) {}

void md_can_msg_pending_fifo0_callback(void) {}
