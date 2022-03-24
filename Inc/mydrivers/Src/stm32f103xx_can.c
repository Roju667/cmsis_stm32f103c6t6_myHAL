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
      // CAN RX PA11, REMAP : PB8
      md_gpio_configure_output(GPIOA, GPIO_PIN_11, GPIO_SPEED_10MHZ,
                               GPIO_OUTPUT_AF_PP);

      // CAN RX PA12, REMAP : PB9
      md_gpio_configure_input(GPIOA, GPIO_PIN_12, GPIO_INPUT_PULLUP);
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
 * @param[prescaler] - clock prescaler - 0-512
 * @param[quanta_ts1] - number of quanta for time segement 1 MAX 16 quanta
 * @can_time_quanta
 * @param[quanta_ts2] - number of quanta for time segement 2 MAX 8 quanta
 * @can_time_quanta
 * @param[quanta_sjw] - number of quanta for synchronization MAX 4 quanta
 * @can_time_quanta
 * @return - can_error_t - can error status
 */
can_error_t md_can_init_time_quanta(can_handle_t *p_hCANx, uint16_t prescaler,
                                    can_time_quanta_t quanta_ts1,
                                    can_time_quanta_t quanta_ts2,
                                    can_time_quanta_t quanta_sjw)
{
  // check if init mode
  if (p_hCANx->op_mode != CAN_OPMODE_INIT)
    {
      return CAN_ERR_WRONG_MDOE;
    }

  // check if values are not exceeded
  if (quanta_ts2 > CAN_TIME_QUANTA8 || quanta_sjw > CAN_TIME_QUANTA4)
    {
      p_hCANx->can_error = CAN_ERR_INIT_QUANTA;
      return CAN_ERR_INIT_QUANTA;
    }

  if (prescaler > 511)
    {
      p_hCANx->can_error = CAN_ERR_INIT_BAUD;
      return CAN_ERR_INIT_BAUD;
    }

  // init prescaler and quantas
  p_hCANx->p_CANx->BTR |= (prescaler << CAN_BTR_BRP_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_ts1 << CAN_BTR_TS1_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_ts2 << CAN_BTR_TS2_Pos);
  p_hCANx->p_CANx->BTR |= (quanta_sjw << CAN_BTR_SJW_Pos);

  p_hCANx->can_error = CAN_ERR_NOERR;
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
    case (CAN_TESTMODE_SLEEP):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      CLEAR_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      break;

    case (CAN_TESTMODE_LOOPBACK):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      CLEAR_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      break;

    case (CAM_TESTMODE_SLEEPLOOPBACK):
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_LBKM);
      SET_BIT(p_hCANx->p_CANx->BTR, CAN_BTR_SILM);
      break;

    default:
      return CAN_ERR_SWITCH_MODE;
    }

  p_hCANx->op_mode = p_hCANx->op_mode = CAN_OPMODE_TEST;
  return CAN_ERR_NOERR;
}

/*
 * configure mailbox before sending can message
 * @param[*p_hCANx] - @can_handler
 * @param[mailbox] - mailbox number @can_mailbox
 * @param[id] - 11 bit for stnd, 29 bits for extended
 * @param[can_id_extension_t] - standard/extended ID @can_id_extension
 * @return - can_error_t - can error status
 */
can_error_t md_can_configure_mailbox_id(can_handle_t *p_hCANx,
                                        can_mailbox_t mailbox, uin32_t id,
                                        can_id_extension_t id_extension){

    p_hCANx->p_CANx->sTxMailBox[mailbox]->TIR

}

/*
 * request message send on can bus
 * @param[*p_hCANx] - @can_handler
 * @param[mailbox] - mailbox number @can_mailbox
 * @param[p_databuffer] - message buffer - max 8 bytes
 * @param[data_lenght] - max 8 bytes
 * @param[transmit] - message type - data/remote
 * @return - can_error_t - can error status
 */
can_error_t
    md_can_transmit_mailbox(can_handle_t *p_hCANx, can_mailbox_t mailbox,
                            uint8_t *p_databuffer, uint8_t data_lenght,
                            can_transmit_t transmit)
{
}

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

#endif // MD_ENABLE_CAN
