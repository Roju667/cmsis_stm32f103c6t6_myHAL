/*
 * temt6000.h
 *
 *  Created on: 31 mar 2022
 *      Author: ROJEK
 */

#ifndef DEVICES_INC_TEMT6000_H_
#define DEVICES_INC_TEMT6000_H_

#include "stdint.h"

#define TEMT6000_REF_VOLTAGE 5
#define TEMT6000_RESISTANCE 10000

typedef struct
{
  uint16_t milivolts;
  uint32_t microamps;
  uint16_t lux;
} temt6000_data_t;

void temt6000_get_data(uint16_t analog_signal,temt6000_data_t *data);
uint16_t temt6000_get_lux(uint16_t analog_signal);

#endif /* DEVICES_INC_TEMT6000_H_ */
