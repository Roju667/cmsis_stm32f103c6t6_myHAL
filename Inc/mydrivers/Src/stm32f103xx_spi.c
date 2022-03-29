/*
 * stm32f103xx_spi.c
 *
 *  Created on: 29 mar 2022
 *      Author: pawel
 */

#include "stm32f103xx_spi.h"
#include "math.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_mydriver.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

#ifdef MD_ENABLE_SPI

#if MD_USING_SPI1
spi_handle_t hspi1;
#endif // MD_USING_SPI1

static void spi_init_handlers(void);
static void spi_init_clock(spi_handle_t *p_hSPIx);
static void spi_init_gpio(spi_handle_t *p_hSPIx, spi_config_t spi_config);

/*
 * Init handlers gpio and clock
 * @param[*p_hSPIx] - can struct handler @spi_handler
 * @return - void
 */
void md_spi_init(spi_handle_t *p_hSPIx)
{
  spi_init_handlers();
  spi_init_clock(p_hSPIx);
}
/*
 * Spi uart parameters
 * @param[*p_hSPIx] - spix base address
 * @param[spi_config] - basic configuration to work as master/slave full duplex
 * mode
 * @return - void
 */
void md_spi_init_basic(spi_handle_t *p_hSPIx, spi_config_t spi_config)
{
  // init pins
  spi_init_gpio(p_hSPIx, spi_config);

  // clock phase
  if (spi_config.clock_second_edge_capture == true)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_CPHA);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_CPHA);
    }

  // clock polarity
  if (spi_config.clock_1_when_idle == true)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_CPOL);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_CPOL);
    }

  // data format
  if (spi_config.data_format_16bit == true)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_DFF);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_DFF);
    }

  // full duplex
  if (spi_config.full_duplex == true)
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_RXONLY);
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_BIDIMODE);
    }
  else
    {
      //!!! ADD SIMPELX !!!
    }

  // lsb/msb first
  if (spi_config.lsb_first == true)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_LSBFIRST);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_LSBFIRST);
    }

  // nss manangement
  if (spi_config.software_nss_management)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_SSM);
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_SSI);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_SSM);
    }

  // master mode
  if (spi_config.master_mode == true)
    {
      SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_MSTR);
    }
  else
    {
      CLEAR_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_MSTR);
    }

  // prescaler
  p_hSPIx->p_SPIx->CR1 &= ~(SPI_CR1_BR_Msk);
  p_hSPIx->p_SPIx->CR1 |= (spi_config.prescaler << SPI_CR1_BR_Pos);

  SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_SPE);

  return;
}

/*
 * Transmit data in polling mode on spi line
 * @param[*p_hSPIx] - spix base address
 * @param[p_data_buffer] - pointer to data buffer that has to be transmitted
 * @param[data_lenght] - how many bytes to transmit
 * @param[timeout_ms] - timeout in miliseconds
 * @return - void
 */
spi_error_t md_spi_tx_polling(spi_handle_t *p_hSPIx, uint8_t *p_data_buffer,
                              uint16_t data_lenght, uint32_t timeout_ms)
{
  uint32_t time_tick;
  uint16_t data_counter = data_lenght;

  // enable SPI
  SET_BIT(p_hSPIx->p_SPIx->CR1, SPI_CR1_SPE);

  // check if other transfer is not ongoing
  if (p_hSPIx->spi_tx_status != SPI_TX_IDLE)
    {
      p_hSPIx->spi_error = SPI_ERR_TX_COLLISION;
      return SPI_ERR_TX_COLLISION;
    }

  // change status of tx line
  p_hSPIx->spi_tx_status = SPI_TX_POLLING;

  while (data_counter > 0)
    {
      // waint until transmit buffer is empty
      time_tick = md_systick_get_tick();
      while (!(p_hSPIx->p_SPIx->SR & SPI_SR_TXE))
        {
          if ((md_systick_get_tick() - time_tick) > timeout_ms)
            {
              p_hSPIx->spi_error = SPI_ERR_TIMEOUT_TXE;
              p_hSPIx->spi_tx_status = SPI_TX_IDLE;
              return SPI_ERR_TIMEOUT_TXE;
            }
        }

      // put data in data register
      p_hSPIx->p_SPIx->DR = p_data_buffer[data_lenght - data_counter];
      data_counter--;
    }

  p_hSPIx->spi_error = SPI_ERR_TIMEOUT_TXE;
  p_hSPIx->spi_tx_status = SPI_TX_IDLE;
  return SPI_ERR_NOERR;
}
/*
 * Init handler structures
 * @param[void]
 * @return - void
 */
static void spi_init_handlers(void)
{
#if MD_USING_SPI1
  hspi1.p_SPIx = SPI1;
  hspi1.p_tx_buffer = NULL;
  hspi1.tx_buffer_count = 0;
  hspi1.spi_error = SPI_ERR_NOERR;
  hspi1.spi_rx_status = SPI_RX_IDLE;
  hspi1.spi_tx_status = SPI_TX_IDLE;
#endif // MD_USING_USART1
}

/*
 * Starts clock for SPI and resets the peripheral
 * @param[*p_hSPIx] - spix base address
 * @return - void
 */
static void spi_init_clock(spi_handle_t *p_hSPIx)
{
  if (RCC->APB2ENR & RCC_APB2ENR_SPI1EN)
    return;

  RCC_CLOCK_ENABLE_SPI1();
  SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
  CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);

  return;
}

/*
 * Init gpio pins for spi - make sure that GPIO clock is enabled before
 * @param[*pUSARTx] - spix base address
 * @return - void
 */
static void spi_init_gpio(spi_handle_t *p_hSPIx, spi_config_t spi_config)
{
  // master mode
  if (spi_config.master_mode == true)
    {
      // SCK - PA5
      md_gpio_configure_output(GPIOA, GPIO_PIN_5, GPIO_SPEED_10MHZ,
                               GPIO_OUTPUT_AF_PP);

      if (spi_config.full_duplex == true)
        {
          // MISO - PA6 // REMAP PB4
          md_gpio_configure_input(GPIOA, GPIO_PIN_6, GPIO_INPUT_PULLUP);
        }

      // MOSI PA7 // REMAP PB5
      md_gpio_configure_output(GPIOA, GPIO_PIN_7, GPIO_SPEED_10MHZ,
                               GPIO_OUTPUT_AF_PP);

      if (spi_config.software_nss_management == false)
        {
          // NSS - PA4
          md_gpio_configure_output(GPIOA, GPIO_PIN_4, GPIO_SPEED_50MHZ,
                                   GPIO_OUTPUT_AF_PP);
        }
    }
  else
    {
      // slave mode
      // SCK - PA5
      md_gpio_configure_input(GPIOA, GPIO_PIN_5, GPIO_INPUT_FLOATING);

      if (spi_config.full_duplex == true)
        {
          // MOSI PA7 // REMAP PB5
          md_gpio_configure_input(GPIOA, GPIO_PIN_7, GPIO_INPUT_PULLUP);
        }

      // MISO - PA6 // REMAP PB4
      md_gpio_configure_output(GPIOA, GPIO_PIN_6, GPIO_SPEED_10MHZ,
                               GPIO_OUTPUT_AF_PP);

      if (spi_config.software_nss_management == false)
        {
          // NSS - PA4
          md_gpio_configure_input(GPIOA, GPIO_PIN_4, GPIO_INPUT_PULLUP);
        }
    }
}

#endif // MD_ENABLE_SPI
