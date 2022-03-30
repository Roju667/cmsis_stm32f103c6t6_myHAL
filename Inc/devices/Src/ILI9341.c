/*
 * ILI9341.c
 *
 *  Created on: Oct 11, 2021
 *      Author: ROJEK
 */


#include "stm32f103xx_spi.h"
#include "stm32f103xx_systick.h"
#include "ILI9341.h"

spi_handle_t *Tft_hspi;

// Delay for the functions
static void ILI9341_Delay(uint32_t ms) { md_systick_delay(ms); }

// Transmit data to ILI controller
static void ILI9341_SendTFT(uint8_t *Data, uint8_t Lenght)
{

  md_spi_tx_polling(Tft_hspi, Data, Lenght, 1000);
}
// Send single command
static void ILI9341_SendCommand(uint8_t Command)
{
  // CS LOW
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_LOW;
#endif

  // DC LOW
  ILI9341_DC_LOW;

  // SEND COMMAND
  ILI9341_SendTFT(&Command, 1);

  // CS HIGH
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_HIGH;
#endif
}

// Send 16 bit data
static void ILI9341_SendData16(uint16_t Data)
{
  // CS LOW
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_LOW;
#endif

  // DC HIGH
  ILI9341_DC_HIGH;

  // Send 2 8 bits, first MSB (ILI9341 datasheet)
  uint8_t tmp[2];
  tmp[0] = (Data >> 8);
  tmp[1] = Data & 0xFF;

  // SEND COMMAND
  ILI9341_SendTFT(tmp, 2);

  // CS HIGH
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_HIGH;
#endif
}

// Send command then data
static void ILI9341_SendCommandAndData(uint8_t Command, uint8_t *Data,
                                       uint16_t Lenght)
{
  // CS LOW
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_LOW;
#endif

  // DC LOW
  ILI9341_DC_LOW;

  // SEND COMMAND
  ILI9341_SendTFT(&Command, 1);

  // DC HIGH
  ILI9341_DC_HIGH;

  // SEND DATA
  ILI9341_SendTFT(Data, Lenght);

  // CS HIGH
#if (ILI9341_USE_CS == 1)
  ILI9341_CS_HIGH;
#endif
}

void ILI9341_SetRotation(uint8_t Rotation)
{
  if (Rotation > 3)
    return;

  switch (Rotation)
    {
    case 0:
      Rotation = (MADCTL_MX | MADCTL_BGR);
      break;
    case 1:
      Rotation = (MADCTL_MV | MADCTL_BGR);
      break;
    case 2:
      Rotation = (MADCTL_MY | MADCTL_BGR);
      break;
    case 3:
      Rotation = (MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      break;
    }

  ILI9341_SendCommandAndData(ILI9341_MADCTL, &Rotation, 1);
}

// Set adress range window
static void ILI9341_SetAddrWindow(uint16_t x1, uint16_t y1, uint16_t w,
                                  uint16_t h)
{
  // prepare buffer for data
  uint8_t DataToTransfer[4];

  // calculate ranges
  uint16_t x2 = (x1 + w - 1), y2 = (y1 + h - 1);

  //	put data into buffer
  DataToTransfer[0] = (x1 >> 8);
  DataToTransfer[1] = x1 & 0xFF;
  DataToTransfer[2] = (x2 >> 8);
  DataToTransfer[3] = x2 & 0xFF;

  // send command and data about x
  ILI9341_SendCommandAndData(ILI9341_CASET, DataToTransfer, 4);

  //	put data into buffer
  DataToTransfer[0] = (y1 >> 8);
  DataToTransfer[1] = y1 & 0xFF;
  DataToTransfer[2] = (y2 >> 8);
  DataToTransfer[3] = y2 & 0xFF;

  // send command and data about y
  ILI9341_SendCommandAndData(ILI9341_PASET, DataToTransfer, 4);

  ILI9341_SendCommand(ILI9341_RAMWR); // Write to RAM
}

// Write single pixel
void ILI9341_WritePixel(int16_t x, int16_t y, uint16_t color)
{

  // prepare buffer for data
  uint8_t DataToTransfer[2];

  // check TFT range to not overwrite something else
  if ((x >= 0) && (x < ILI9341_TFTWIDTH) && (y >= 0) && (y < ILI9341_TFTHEIGHT))
    {
      //	put data into buffer
      DataToTransfer[0] = (color >> 8);
      DataToTransfer[1] = color & 0xFF;

      // Set window range the single pixel in tft
      // x,y positions 1,1 ranges
      ILI9341_SetAddrWindow(x, y, 1, 1);

      // send command that we are writing to RAM, and also color data
      ILI9341_SendCommandAndData(ILI9341_RAMWR, DataToTransfer, 2);
      // Send 16 bit color to that range
    }
}

void ILI9341_DrawImage(int x, int y, const uint8_t *img, uint16_t w, uint16_t h)
{
  // check if the image is inisde tft boundaries
  if ((x >= 0) && ((x + w) <= ILI9341_TFTWIDTH) && (y >= 0) &&
      ((y + h) <= ILI9341_TFTHEIGHT))
    {
      ILI9341_SetAddrWindow(x, y, w, h);
      ILI9341_SendCommandAndData(ILI9341_RAMWR, (uint8_t *)img, (w * h * 2));
    }
}

// Clear whole dipslay with a color
void ILI9341_ClearArea(uint16_t color, uint16_t start_x, uint16_t start_y,
                       uint16_t width, uint16_t height)
{
  uint32_t Lenght = width * height;

  // set window for whole screen
  ILI9341_SetAddrWindow(start_x, start_y, width, height);

  // send command that we are writing to RAM
  ILI9341_SendCommand(ILI9341_RAMWR);

  // without HAL optimizing
  for (uint32_t i = 0; i < Lenght; i++)
    {
      ILI9341_SendData16(color);
    }
}

// Clear whole dipslay with a color
void ILI9341_ClearDisplay(uint16_t color)
{
  uint32_t Lenght = ILI9341_TFTWIDTH * ILI9341_TFTHEIGHT;

  // set window for whole screen
  ILI9341_SetAddrWindow(0, 0, ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT);

  // send command that we are writing to RAM
  ILI9341_SendCommand(ILI9341_RAMWR);

  // without HAL optimizing
  for (uint32_t i = 0; i < Lenght; i++)
    {
      ILI9341_SendData16(color);
    }

}

// values for init
static const uint8_t initcmd[] = {
    0xEF,
    3,
    0x03,
    0x80,
    0x02,
    0xCF,
    3,
    0x00,
    0xC1,
    0x30,
    0xED,
    4,
    0x64,
    0x03,
    0x12,
    0x81,
    0xE8,
    3,
    0x85,
    0x00,
    0x78,
    0xCB,
    5,
    0x39,
    0x2C,
    0x00,
    0x34,
    0x02,
    0xF7,
    1,
    0x20,
    0xEA,
    2,
    0x00,
    0x00,
    ILI9341_PWCTR1,
    1,
    0x23, // Power control VRH[5:0]
    ILI9341_PWCTR2,
    1,
    0x10, // Power control SAP[2:0];BT[3:0]
    ILI9341_VMCTR1,
    2,
    0x3e,
    0x28, // VCM control
    ILI9341_VMCTR2,
    1,
    0x86, // VCM control2
    ILI9341_MADCTL,
    1,
    0x48, // Memory Access Control
    ILI9341_VSCRSADD,
    1,
    0x00, // Vertical scroll zero
    ILI9341_PIXFMT,
    1,
    0x55,
    ILI9341_FRMCTR1,
    2,
    0x00,
    0x18,
    ILI9341_DFUNCTR,
    3,
    0x08,
    0x82,
    0x27, // Display Function Control
    0xF2,
    1,
    0x00, // 3Gamma Function Disable
    ILI9341_GAMMASET,
    1,
    0x01, // Gamma curve selected
    ILI9341_GMCTRP1,
    15,
    0x0F,
    0x31,
    0x2B,
    0x0C,
    0x0E,
    0x08, // Set Gamma
    0x4E,
    0xF1,
    0x37,
    0x07,
    0x10,
    0x03,
    0x0E,
    0x09,
    0x00,
    ILI9341_GMCTRN1,
    15,
    0x00,
    0x0E,
    0x14,
    0x03,
    0x11,
    0x07, // Set Gamma
    0x31,
    0xC1,
    0x48,
    0x08,
    0x0F,
    0x0C,
    0x31,
    0x36,
    0x0F,
    ILI9341_SLPOUT,
    0x80, // Exit Sleep
    ILI9341_DISPON,
    0x80, // Display on
    0x00  // End of list
};

void ILI9341_Init(spi_handle_t *hspi)
{

  // assign correct spi
  Tft_hspi = hspi;

  // prepare data

  uint8_t cmd, x, numArgs;
  const uint8_t *addr = initcmd;
  ILI9341_CS_HIGH;

// if hardware reset is defined
#if (ILI9341_USE_HW_RESET == 1)
  ILI9341_RST_LOW;
  ILI9341_Delay(10);
  ILI9341_RST_HIGH;
  ILI9341_Delay(10);
#else
  ILI9341_SendCommand(ILI9341_SWRESET); // Engage software reset
  ILI9341_Delay(150);
#endif

  // As long as value under address is not 0 loop
  while ((cmd = *(addr++)) > 0)
    {
      // assign value form address to x (second value that is number of data to
      // be send)
      x = *(addr++);

      // mask this value to maximum of 127
      // 0x7F	0111 1111
      // so if we send 0x80 as second argument then we just send command ->
      // without data
      numArgs = x & 0x7F;

      // send command then array of data
      ILI9341_SendCommandAndData(cmd, (uint8_t *)addr, numArgs);

      // move adress to next command
      addr += numArgs;

      // if only command is sent then make a delay
      if (x & 0x80)
        {
          ILI9341_Delay(150);
        }
    }

  ILI9341_SetRotation(ILI9341_ROTATION);
}
