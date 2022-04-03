/*
 * stm32f103xx_can.h
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#ifndef MYDRIVERS_INC_STM32F103XX_CAN_H_
#define MYDRIVERS_INC_STM32F103XX_CAN_H_

#include "stdbool.h"
#include "stm32f1xx.h"

// defining those macros will allocate static data
// for purpose of using DMA/IRQ
#define MD_USING_CAN1 1U

#if MD_USING_CAN1
#define MD_ENABLE_CAN
#endif // MD_USING_CAN1

#ifdef MD_ENABLE_CAN

// @can_mode
typedef enum can_op_mode_t
{
  CAN_OPMODE_INIT = 0,
  CAN_OPMODE_NORMAL,
  CAN_OPMODE_SLEEP,
  CAN_OPMODE_TEST
} can_op_mode_t;

// @can_error
typedef enum can_error_t
{
  CAN_ERR_NOERR = 0,
  CAN_ERR_TIMEOUT_INAK = -100,
  CAN_ERR_TIMEOUT_SLAK,
  CAN_ERR_SWITCH_MODE,
  CAN_ERR_WRONG_MDOE,
  CAN_ERR_INIT_QUANTA,
  CAN_ERR_INIT_BAUD,
  CAN_ERR_TX_DATA_EXCEEDED,
  CAN_ERR_TX_NO_MAILBOX,
  CAN_ERR_FILTER_NO_TOO_HIGH,
  CAN_ERR_FIFO_EMPTY
} can_error_t;

// @can_time_quanta
typedef enum can_time_quanta_t
{
  CAN_TIME_QUANTA1 = 0,
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
typedef enum can_test_mode_t
{
  CAN_TESTMODE_SILENT = 0,
  CAN_TESTMODE_LOOPBACK,
  CAN_TESTMODE_SILENTLOOPBACK
} can_test_mode_t;

// @can_mailbox
typedef enum can_mailbox_t
{
  CAN_MAILBOX0 = 0,
  CAN_MAILBOX1,
  CAN_MAILBOX2,
  CAN_MAILBOX_NOMAILBOX
} can_mailbox_t;

// @can_id_extension
typedef enum can_id_extension_t
{
  CAN_ID_EXTENSION_STD = 0,
  CAN_ID_EXTENSION_EXT
} can_id_extension_t;

// @can_quanta_init
typedef struct can_quanta_init_t
{
  uint32_t prescaler;
  can_time_quanta_t quanta_ts1;
  can_time_quanta_t quanta_ts2;
  can_time_quanta_t quanta_sjw;
} can_quanta_init_t;

// @can_basic_init
typedef struct can_basic_init_t
{
  bool debug_freeze;
  bool time_triggered_comm;
  bool auto_bus_off;
  bool auto_wake_up;
  bool auto_retransmit;
  bool rx_fifo_lock;
  bool tx_fifo_prio;
} can_basic_init_t;

// @can_irq_group
typedef enum can_irq_group_t
{
  CAN_IRQ_GROUP_TX,
  CAN_IRQ_GROUP_RX0,
  CAN_IRQ_GROUP_RX1,
  CAN_IRQ_GROUP_SCE
} can_irq_group_t;

// @can_frame
typedef struct can_frame_t
{
  uint32_t id;
  uint32_t data_lenght;
  bool id_extended;
  bool remote;
  uint8_t *p_data_buffer;
} can_frame_t;

// @can_filter
typedef struct can_filter_t
{
  uint32_t filter_number;
  bool list_mode;
  bool scale_32bit;
  bool assign_to_fifo1;
  uint32_t filter_id0;
  uint32_t filter_mask_or_id1;

} can_filter_t;

// @can_handler
typedef struct can_handle_t
{
  CAN_TypeDef *p_CANx;
  can_error_t can_error;
  can_op_mode_t op_mode;
  bool msg_pending_fifo0;
  bool msg_pending_fifo1;

} can_handle_t;

#if MD_USING_CAN1
extern can_handle_t hcan1;
#endif // MD_USING_CAN1

// init functions
void md_can_init(can_handle_t *p_hCANx);
can_error_t md_can_init_time_quanta(can_handle_t *p_hCANx,
                                    can_quanta_init_t *p_quanta_init);
can_error_t md_can_init_basic(can_handle_t *p_hCANx,
                              can_basic_init_t *p_basic_init);
// mode functions
can_error_t md_can_change_op_mode(can_handle_t *p_hCANx, can_op_mode_t op_mode,
                                  uint32_t timeout_ms);
can_error_t md_can_enter_test_mode(can_handle_t *p_hCANx,
                                   can_test_mode_t test_mode);

// write to mailbox
can_error_t md_can_write_mailbox(can_handle_t *p_hCANx, can_frame_t *p_frame,
                                 uint32_t *p_mailbox_number);

// read from fifo
can_error_t md_can_read_fifo(can_handle_t *p_hCANx, can_frame_t *p_frame_buffer,
                             uint8_t *p_data_buffer, uint8_t fifo_number);

// filter configuration
can_error_t md_can_init_filter(can_handle_t *p_hCANx, can_filter_t *p_filter);

// irq functions
void md_can_activate_irq(can_handle_t *p_hCANx, can_irq_group_t irq_group,
                         uint32_t irq_flags, uint8_t irq_prio);

// callbacks
void md_can_mailbox_empty_callback(void);
void md_can_msg_pending_fifo0_callback(void);
void md_can_msg_pending_fifo1_callback(void);

#endif // MD_ENABLE_CAN CAN
#endif /* MYDRIVERS_INC_STM32F103XX_CAN_H_ */
