/*
 * spi_example.c
 *
 *  Created on: 29 mar 2022
 *      Author: pawel
 */

#ifndef EXAMPLES_INC_SPI_EXAMPLE_C_
#define EXAMPLES_INC_SPI_EXAMPLE_C_

#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_spi.h"
#include "stm32f103xx_systick.h"

#include "GFX_COLOR.h"
#include "ILI9341.h"
//#include "fonts.h"

#include "gpio_example.h"
#include "rcc_example.h"

void example_spi_init_master_fullduplex(void)
{
  example_rcc_configure_pll_32Mhz();

  md_systick_configure_ms();

  md_spi_init(&hspi1);

  spi_config_t spi_config;
  spi_config.clock_1_when_idle = true;
  spi_config.clock_second_edge_capture = false;
  spi_config.data_format_16bit = false;
  spi_config.full_duplex = true;
  spi_config.lsb_first = true;
  spi_config.master_mode = true;
  spi_config.prescaler = SPI_PRESCALER_32; // 100kHz
  spi_config.software_nss_management = true;

  md_spi_init_basic(&hspi1, &spi_config);
}

void example_spi_transfer_message(void)
{
  example_spi_init_master_fullduplex();

  uint8_t data_buffer[32] = "0123456789ABCDEFspitest2022";

  md_spi_tx_polling(&hspi1, data_buffer, 32, 1000);

  uint32_t time_tick_led = md_systick_get_tick();

  while (1)
    {
      example_heart_beat_no_delay(&time_tick_led, 500);
    }
}

void example_spi_transfer_tft(void)
{
  example_rcc_configure_pll_32Mhz();
  // init spi and sys tick
  md_systick_configure_ms();

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

  md_spi_init_basic(&hspi1, &spi_config);

  // init gpio for tft
  //  DC
  md_gpio_configure_output(GPIOB, GPIO_PIN_1, GPIO_SPEED_10MHZ, GPIO_OUTPUT_PP);
  // CS
  // RESET
  md_gpio_configure_output(GPIOB, GPIO_PIN_10, GPIO_SPEED_10MHZ,
                           GPIO_OUTPUT_PP);

  ILI9341_Init(&hspi1);
  ILI9341_ClearDisplay(ILI9341_BLACK);

//  GFX_SetFont(font_8x5);
  GFX_SetFontSize(2);
  GFX_DrawString(10, 10, "Can message recieved", ILI9341_YELLOW);
  uint32_t time_tick_led = md_systick_get_tick();

  while (1)
    {
      example_heart_beat_no_delay(&time_tick_led, 500);
    }
}

#endif /* EXAMPLES_INC_SPI_EXAMPLE_C_ */
