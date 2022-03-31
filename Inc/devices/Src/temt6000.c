/*
 * temt6000.c
 *
 *  Created on: 31 mar 2022
 *      Author: ROJEK
 */

#include "temt6000.h"

void temt6000_get_data(uint16_t analog_signal, temt6000_data_t *data)
{
  // 10k resistor
  // 5V reference voltage

  data->milivolts = analog_signal * ((TEMT6000_REF_VOLTAGE * 1000) / 4096);

  // I = U/R
  data->microamps = ((1000 * data->milivolts) / TEMT6000_RESISTANCE);

  // sensor characteristic
  data->lux = data->microamps * 2;

  return;
}

uint16_t temt6000_get_lux(uint16_t analog_signal)
{
  uint32_t milivolts = analog_signal * ((TEMT6000_REF_VOLTAGE * 1000) / 4096);

  uint32_t microamps = ((1000 * milivolts) / TEMT6000_RESISTANCE);

  return 2 * microamps;
}
