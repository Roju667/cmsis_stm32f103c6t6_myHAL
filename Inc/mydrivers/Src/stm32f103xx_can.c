/*
 * stm32f103xx_can.c
 *
 *  Created on: Mar 20, 2022
 *      Author: pawel
 */

#include "stm32f103xx_can.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_rcc.h"
#include "stm32f103xx_systick.h"

#ifdef MD_ENABLE_CAN

#if MD_USING_CAN1
can_handle_t hcan1;
#endif // MD_USING_CAN1

static can_error_t can_enter_init_mode(can_handle_t *p_hCANx,
                                       uint32_t timeout_ms);
static can_error_t can_enter_normal_mode(can_handle_t *p_hCANx,
                                         uint32_t timeout_ms);
static can_error_t can_enter_sleep_mode(can_handle_t *p_hCANx,
                                        uint32_t timeout_ms);
static can_mailbox_t can_get_empty_mailbox(can_handle_t *p_hCANx);

/*
 * Init handler structures
 * @param[void]
 * @return - void
 */
void md_can_init_handlers(void)
{
  hcan1.p_CANx = CAN1;
  hcan1.can_error = CAN_ERR_NOERR;
  hcan1.op_mode = CAN_OPMODE_SLEEP;
}

/*
 * Starts clock for CAN and resets the peripheral
 * @param[*p_hCANx] - can struct handler @can_handler
 * @return - void
 */
void md_can_init_clock(can_handle_t *p_hCANx)
{
  if (p_hCANx->p_CANx == CAN1)
    {
      RCC_CLOCK_ENABLE_CAN();
      SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);
      CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_CAN1RST);
    }
  return;
}

/*
 * Init gpio pins for can bus
 * @param[*p_hCANx] - can struct handler @can_handler
 * @return - void
 */
void md_can_init_gpio(can_handle_t *p_hCANx)
{
  if (p_hCANx->p_CANx == CAN1)
    {
      // CAN TX PA12, REMAP : PB8
      md_gpio_configure_output(GPIOA, GPIO_PIN_12, GPIO_SPEED_50MHZ,
                               GPIO_OUTPUT_AF_PP);

      // CAN RX PA11, REMAP : PB9
      md_gpio_configure_input(GPIOA, GPIO_PIN_11, GPIO_INPUT_FLOATING);
    }
  return;
}

/*
 * Switch operation mode
 * @param[*p_hCANx] - can struct handler @can_handler
 * @param[op_mode] - op mode that should be entered @can_mode
 * @param[timeout_ms] - timeout in miliseconds
 * @return - can_error_t - can error status
 */
can_error_t md_can_change_op_mode(can_handle_t *p_hCANx, can_op_mode_t op_mode,
                                  uint32_t timeout_ms)
{
  switch (op_mode)
    {
    case (CAN_OPMODE_INIT):
      {
        return can_enter_init_mode(p_hCANx, timeout_ms);
      }
    case (CAN_OPMODE_NORMAL):
      {
        return can_enter_normal_mode(p_hCANx, timeout_ms);
      }
    case (CAN_OPMODE_SLEEP):
      {
        return can_enter_sleep_mode(p_hCANx, timeout_ms);
      }
    default:
      return CAN_ERR_SWITCH_MODE;
    }
}

/*
 * init time quanta for can - function must be used in init mode
 * @param[*p_hCANx] - @can_handler
 * @param[quanta init] - init with values for time configuration
 * presacler clock prescaler - 0-512
 * quanta_ts1 - number of quanta for time segement 1 MAX 16 quanta
 * @can_time_quanta quanta_ts2 - number of quanta for time segement 2 MAX 8
 * quanta quanta_sjw - number of quanta for synchronization MAX 4 quanta
 * @return - can_error_t - can error status
 */
can_error_t md_can_init_time_quanta(can_handle_t *p_hCANx,
                                    can_quanta_init_t quanta_init)
{
  // check if init mode
  if (p_hCANx->op_mode != CAN_OPMODE_INIT)
    {
      return CAN_ERR_WRONG_MDOE;
    }

  // check if values are not exceeded
  if (quanta_init.quanta_ts2 > CAN_TIME_QUANTA8 ||
      quanta_init.quanta_sjw > CAN_TIME_QUANTA4)
    {
      p_hCANx->can_error = CAN_ERR_INIT_QUANTA;
      return CAN_ERR_INIT_QUANTA;
    }

  if (quanta_init.prescaler > 511 || quanta_init.prescaler < 1)
    {
      p_hCANx->can_error = CAN_ERR_INIT_BAUD;
      return CAN_ERR_INIT_BAUD;
    }

  p_hCANx->p_CANx->BTR &= ~(CAN_BTR_BRP_Msk);
  p_hCANx->p_CANx->BTR &= ~(CAN_BTR_TS1_Msk);
  p_hCANx->p_CANx->BTR &= ~(CAN_BTR_TS2_Msk);
  p_hCANx->p_CANx->BTR &= ~(CAN_BTR_SJW_Msk);

  // init prescaler and quantas
  p_hCANx->p_CANx->BTR |= ((quanta_init.prescaler - 1) << CAN_BTR_BRP_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_init.quanta_ts1 << CAN_BTR_TS1_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_init.quanta_ts2 << CAN_BTR_TS2_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_init.quanta_sjw << CAN_BTR_SJW_Pos);

  p_hCANx->can_error = CAN_ERR_NOERR;
  return CAN_ERR_NOERR;
}

/*
 * init basic configuration for can bus
 * @param[*p_hCANx] - @can_handler
 * @param[basic_init] - @can_basic_init
 * @return - can_error_t - can error status
 */
can_error_t md_can_init_basic(can_handle_t *p_hCANx,
                              can_basic_init_t basic_init)
{
  // check if init mode
  if (p_hCANx->op_mode != CAN_OPMODE_INIT)
    {
      return CAN_ERR_WRONG_MDOE;
    }

  // set or reset all the configuration flags
  if (basic_init.debug_freeze == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_DBF);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_DBF);
    }

  if (basic_init.time_triggered_comm == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_TTCM);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_TTCM);
    }

  if (basic_init.auto_bus_off == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_ABOM);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_ABOM);
    }

  if (basic_init.auto_wake_up == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_AWUM);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_AWUM);
    }

  if (basic_init.auto_retransmit == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_NART);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_NART);
    }

  if (basic_init.rx_fifo_lock == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_RFLM);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_RFLM);
    }

  if (basic_init.tx_fifo_prio == true)
    {
      SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_TXFP);
    }
  else
    {
      CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_TXFP);
    }

  return CAN_ERR_NOERR;
}

/*
 * init time quanta for can - function must be used in init mode
 * @param[*p_hCANx] - @can_handler
 * @param[test_mode] - @can_test_mode
 * @return - can_error_t - can error status
 */
can_error_t md_can_enter_test_mode(can_handle_t *p_hCANx,
                                   can_test_mode_t test_mode)
{
  // check if init mode
  if (p_hCANx->op_mode != CAN_OPMODE_INIT)
    {
      return CAN_ERR_WRONG_MDOE;
    }

  switch (test_mode)
    {
    case (CAN_TESTMODE_SILENT):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      CLEAR_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      break;

    case (CAN_TESTMODE_LOOPBACK):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      CLEAR_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      break;

    case (CAN_TESTMODE_SILENTLOOPBACK):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      break;

    default:
      return CAN_ERR_SWITCH_MODE;
    }

  p_hCANx->op_mode = CAN_OPMODE_TEST;
  return CAN_ERR_NOERR;
}

/*
 * write mailbox
 * @param[*p_hCANx] - @can_handler
 * @param[frame] - frame structure @can_frame
 * @param[p_databuffer] - pointer to data buffer
 * @param[p_mailbox_number] - pointer to variable that will hold mailbox number that were used
 * @return - can_error_t - can error status
 */
can_error_t md_can_write_mailbox(can_handle_t *p_hCANx, can_frame_t frame,
                                 uint8_t *p_databuffer, uint8_t *p_mailbox_number)
{
  uint8_t mailbox = can_get_empty_mailbox(p_hCANx);

  // check if there is an empty mailbox
  if (mailbox == CAN_MAILBOX_NOMAILBOX)
    {
	  *p_mailbox_number = CAN_MAILBOX_NOMAILBOX;
	  p_hCANx->can_error = CAN_ERR_TX_NO_MAILBOX;
      return CAN_ERR_TX_NO_MAILBOX;
    }

  // check if maximum lenghts is not exceeded
  if (frame.data_lenght > 8)
    {
	  p_hCANx->can_error = CAN_ERR_TX_DATA_EXCEEDED;
      return CAN_ERR_TX_DATA_EXCEEDED;
    }

  // write id and choose standard/extended
  if (frame.id_extended == false)
    {
      CLEAR_BIT(p_hCANx->p_CANx->sTxMailBox[mailbox].TIR, CAN_TI0R_IDE);
      p_hCANx->p_CANx->sTxMailBox[mailbox].TIR &= ~(CAN_TI0R_STID_Msk);
      p_hCANx->p_CANx->sTxMailBox[mailbox].TIR |=
          ((frame.id & 0x07FF) << CAN_TI0R_STID_Pos);
    }
  else if (frame.id_extended == true)
    {
      SET_BIT(p_hCANx->p_CANx->sTxMailBox[mailbox].TIR, CAN_TI0R_IDE);
      p_hCANx->p_CANx->sTxMailBox[mailbox].TIR &= ~(CAN_TI0R_EXID_Msk);
      p_hCANx->p_CANx->sTxMailBox[mailbox].TIR |=
          ((frame.id & 0x1FFFFFFF) << CAN_TI0R_EXID_Pos);
    }

  // set data lenght
  p_hCANx->p_CANx->sTxMailBox[mailbox].TDTR &= ~(CAN_TDT0R_DLC_Msk);
  p_hCANx->p_CANx->sTxMailBox[mailbox].TDTR |=
      (frame.data_lenght << CAN_TDT0R_DLC_Pos);

  // prepare remote or data msg
  if(frame.remote == true)
  {
	  SET_BIT(p_hCANx->p_CANx->sTxMailBox[mailbox].TIR, CAN_TI0R_RTR);
  }else
  {
	  CLEAR_BIT(p_hCANx->p_CANx->sTxMailBox[mailbox].TIR, CAN_TI0R_RTR);
	  // clear data registers
	  p_hCANx->p_CANx->sTxMailBox[mailbox].TDLR = 0;
	  p_hCANx->p_CANx->sTxMailBox[mailbox].TDHR = 0;

	  // write data to registers
	  for (uint8_t i = 0; i < frame.data_lenght; i++)
	    {
	      if (i < 4)
	        {
	          p_hCANx->p_CANx->sTxMailBox[mailbox].TDLR |=
	              (p_databuffer[i] << (i * 8));
	        }
	      else
	        {
	          p_hCANx->p_CANx->sTxMailBox[mailbox].TDHR |=
	              (p_databuffer[i] << ((i % 4) * 8));
	        }
	    }
  }

  // request transmission
  *p_mailbox_number = mailbox;
  SET_BIT(p_hCANx->p_CANx->sTxMailBox[mailbox].TIR, CAN_TI0R_TXRQ);
  p_hCANx->can_error = CAN_ERR_NOERR;
  return CAN_ERR_NOERR;
}

/*
 * read from fifo
 * @param[*p_hCANx] - @can_handler
 * @param[timeout_ms] - timeout in miliseonds
 * @return - can_error_t - can error status
 */
can_error_t md_can_read_fifo(can_handle_t *p_hCANx) {}

/*
 * enter init mode
 * @param[*p_hCANx] - @can_handler
 * @param[timeout_ms] - timeout in miliseonds
 * @return - can_error_t - can error status
 */
static can_error_t can_enter_init_mode(can_handle_t *p_hCANx,
                                       uint32_t timeout_ms)
{
  uint32_t time_tick = 0;
  // request entering init mode
  SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_INRQ);
  CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_SLEEP);

  // wait init mode enter is acknowledged
  while (!(p_hCANx->p_CANx->MSR & CAN_MSR_INAK))
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_INAK;
          return CAN_ERR_TIMEOUT_INAK;
        }
    }

  // wait sleep mode exit is acknowledged
  while (p_hCANx->p_CANx->MSR & CAN_MSR_SLAK)
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_SLAK;
          return CAN_ERR_TIMEOUT_SLAK;
        }
    }

  // change operation mode status
  p_hCANx->op_mode = CAN_OPMODE_INIT;

  p_hCANx->can_error = CAN_ERR_NOERR;
  return CAN_ERR_NOERR;
}

/*
 * enter normal mode
 * @param[*p_hCANx] - can struct handler @can_handler
 * @param[timeout_ms] - timeout in miliseonds
 * @return - can_error_t - can error status
 */
static can_error_t can_enter_normal_mode(can_handle_t *p_hCANx,
                                         uint32_t timeout_ms)
{
  uint32_t time_tick = 0;
  // request entering init mode
  CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_INRQ);
  CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_SLEEP);

  // wait init mode enter is acknowledged
  while (p_hCANx->p_CANx->MSR & CAN_MSR_INAK)
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_INAK;
          return CAN_ERR_TIMEOUT_INAK;
        }
    }

  // wait sleep mode exit is acknowledged
  while (p_hCANx->p_CANx->MSR & CAN_MSR_SLAK)
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_SLAK;
          return CAN_ERR_TIMEOUT_SLAK;
        }
    }

  // change operation mode status
  p_hCANx->op_mode = CAN_OPMODE_NORMAL;

  p_hCANx->can_error = CAN_ERR_NOERR;
  return CAN_ERR_NOERR;
}

/*
 * enter sleep mode
 * @param[*p_hCANx] - can struct handler @can_handler
 * @param[timeout_ms] - timeout in miliseonds
 * @return - can_error_t - can error status
 */
static can_error_t can_enter_sleep_mode(can_handle_t *p_hCANx,
                                        uint32_t timeout_ms)
{
  uint32_t time_tick = 0;
  // request entering init mode
  CLEAR_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_INRQ);
  SET_BIT(p_hCANx->p_CANx->MCR, CAN_MCR_SLEEP);

  // wait init mode enter is acknowledged
  while (p_hCANx->p_CANx->MSR & CAN_MSR_INAK)
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_INAK;
          return CAN_ERR_TIMEOUT_INAK;
        }
    }

  // wait sleep mode exit is acknowledged
  while (!(p_hCANx->p_CANx->MSR & CAN_MSR_SLAK))
    {
      if ((md_systick_get_tick() - time_tick) > timeout_ms)
        {
          p_hCANx->can_error = CAN_ERR_TIMEOUT_SLAK;
          return CAN_ERR_TIMEOUT_SLAK;
        }
    }

  // change operation mode status
  p_hCANx->op_mode = CAN_OPMODE_SLEEP;

  p_hCANx->can_error = CAN_ERR_NOERR;
  return CAN_ERR_NOERR;
}

/*
 * return mailbox number that can be used to send a new message
 * @param[*p_hCANx] - @can_handler
 * @return - mailbox number
 */
can_mailbox_t can_get_empty_mailbox(can_handle_t *p_hCANx)
{
  if (p_hCANx->p_CANx->TSR & CAN_TSR_TME0)
    {
      return CAN_MAILBOX0;
    }
  else if (p_hCANx->p_CANx->TSR & CAN_TSR_TME1)
    {
      return CAN_MAILBOX1;
    }
  else if (p_hCANx->p_CANx->TSR & CAN_TSR_TME2)
    {
      return CAN_MAILBOX2;
    }
  else
    {
      return CAN_MAILBOX_NOMAILBOX;
    }
}

#endif // MD_ENABLE_CAN
