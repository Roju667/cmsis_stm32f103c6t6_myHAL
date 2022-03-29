/*
 * stm32f103xx_spi.h
 *
 *  Created on: 29 mar 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_SPI_H_
#define MYDRIVERS_INC_STM32F103XX_SPI_H_

#include "stdbool.h"
#include "stm32f1xx.h"

// defining those macros will allocate static data
// for purpose of using DMA/IRQ
#define MD_USING_SPI1 1U

#if MD_USING_SPI1
#define MD_ENABLE_SPI
#endif // MD_USING_SPI1

#ifdef MD_ENABLE_SPI

// @spi_error
typedef enum
{
  SPI_ERR_NOERR,
  SPI_ERR_TX_COLLISION,
  SPI_ERR_TIMEOUT_TXE

} spi_error_t;

// @spi_tx_status
typedef enum
{
  SPI_TX_IDLE,
  SPI_TX_POLLING,
  SPI_TX_IRQ,
  SPI_TX_DMA
} spi_tx_status_t;

// @spi_rx_status
typedef enum
{
  SPI_RX_IDLE,
  SPIRX_POLLING,
  SPI_RX_IRQ,
  SPI_RX_DMA
} spi_rx_status_t;

// @spi_prescaler
typedef enum
{
  SPI_PRESCALER_2,
  SPI_PRESCALER_4,
  SPI_PRESCALER_8,
  SPI_PRESCALER_16,
  SPI_PRESCALER_32,
  SPI_PRESCALER_64,
  SPI_PRESCALER_128,
  SPI_PRESCALER_256
} spi_prescaler_t;

typedef struct
{
  bool master_mode;
  bool full_duplex;
  bool software_nss_management;
  bool data_format_16bit;
  bool lsb_first;
  bool clock_1_when_idle;
  bool clock_second_edge_capture;
  spi_prescaler_t prescaler;

} spi_config_t;

// @spi_handle
typedef struct
{
  SPI_TypeDef *p_SPIx;
  uint16_t tx_buffer_len;
  uint16_t tx_buffer_count;
  uint8_t *p_tx_buffer;
  spi_error_t spi_error;
  spi_tx_status_t spi_tx_status;
  spi_rx_status_t spi_rx_status;

} spi_handle_t;

// init functions
void md_spi_init(spi_handle_t *p_hSPIx);
void md_spi_init_basic(spi_handle_t *p_hSPIx, spi_config_t spi_config);

// transmit
spi_error_t md_spi_tx_polling(spi_handle_t *p_hSPIx, uint8_t *p_data_buffer,
                              uint16_t data_lenght, uint32_t timeout_ms);

#if MD_USING_SPI1
extern spi_handle_t hspi1;
#endif // MD_USING_SPI1

#endif // MD_ENABLE_SPI

#endif /* MYDRIVERS_INC_STM32F103XX_SPI_H_ */
