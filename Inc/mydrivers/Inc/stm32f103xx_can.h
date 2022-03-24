/*
 * stm32f103xx_can.h
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_CAN_H_
#define MYDRIVERS_INC_STM32F103XX_CAN_H_

#include "stm32f1xx.h"

// defining those macros will allocate static data
// for purpose of using DMA/IRQ
#define MD_USING_CAN1 1U

#if MD_USING_CAN1
#define MD_ENABLE_CAN
#endif // MD_USING_CAN1

#ifdef MD_ENABLE_CAN

// @can_mode
typedef enum
{
  CAN_OPMODE_INIT,
  CAN_OPMODE_NORMAL,
  CAN_OPMODE_SLEEP,
  CAN_OPMODE_TEST
} can_op_mode_t;

// @can_error
typedef enum
{
  CAN_ERR_NOERR,
  CAN_ERR_TIMEOUT_INAK,
  CAN_ERR_TIMEOUT_SLAK,
  CAN_ERR_SWITCH_MODE,
  CAN_ERR_WRONG_MDOE,
  CAN_ERR_INIT_QUANTA,
  CAN_ERR_INIT_BAUD
} can_error_t;

// @can_time_quanta
typedef enum
{
  CAN_TIME_QUANTA1,
  CAN_TIME_QUANTA2,
  CAN_TIME_QUANTA3,
  CAN_TIME_QUANTA4,
  CAN_TIME_QUANTA5,
  CAN_TIME_QUANTA6,
  CAN_TIME_QUANTA7,
  CAN_TIME_QUANTA8,
  CAN_TIME_QUANTA9,
  CAN_TIME_QUANTA10,
  CAN_TIME_QUANTA11,
  CAN_TIME_QUANTA12,
  CAN_TIME_QUANTA13,
  CAN_TIME_QUANTA14,
  CAN_TIME_QUANTA15,
  CAN_TIME_QUANTA16
} can_time_quanta_t;

// @can_test_mode
typedef enum
{
  CAN_TESTMODE_SLEEP,
  CAN_TESTMODE_LOOPBACK,
  CAM_TESTMODE_SLEEPLOOPBACK
} can_test_mode_t;

// @can_mailbox
typedef enum
{
  CAN_MAILBOX0,
  CAN_MAILBOX1,
  CAN_MAILBOX2
} can_mailbox_t;

// @can_id_extension
typedef enum
{
  CAN_ID_EXTENSION_STD,
  CAN_ID_EXTENSIO_EXT
} can_id_extension_t;

// @can_transmit type
typedef enum
{
  CAN_TRANSMIT_DATA,
  CAN_TRANSMIT_REMOTE
} can_transmit_t;

// @can_

// @can_handler
typedef struct
{
  CAN_TypeDef *p_CANx;
  can_error_t can_error;
  can_op_mode_t op_mode;
} can_handle_t;

#if MD_USING_CAN1
extern can_handle_t hcan1;
#endif // MD_USING_CAN1

void md_can_init_handlers(void);
void md_can_init_clock(can_handle_t *p_hCANx);
void md_can_init_gpio(can_handle_t *p_hCANx);

can_error_t md_can_change_op_mode(can_handle_t *p_hCANx, can_op_mode_t op_mode,
                                  uint32_t timeout_ms);

can_error_t md_can_init_time_quanta(can_handle_t *p_hCANx, uint16_t prescaler,
                                    can_time_quanta_t quanta_ts1,
                                    can_time_quanta_t quanta_ts2,
                                    can_time_quanta_t quanta_sjw);

can_error_t md_can_enter_test_mode(can_handle_t *p_hCANx,
                                   can_test_mode_t test_mode);

#endif // MD_ENABLE_CAN CAN
#endif /* MYDRIVERS_INC_STM32F103XX_CAN_H_ */
