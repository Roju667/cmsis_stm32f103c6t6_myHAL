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
 * @param[*p_hCANx] - can struct handler
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
 * Switch operation mode
 * @param[*p_hCANx] - can struct handler
 * @param[op_mode] - op mode that should be entered
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
 * @param[*p_hCANx] - can struct handler
 * @param[prescaler] - clock prescaler - 0-512
 * @param[quanta_ts1] - number of quanta for time segement 1 MAX 16 quanta
 * @param[quanta_ts2] - number of quanta for time segement 2 MAX 8 quanta
 * @param[quanta_sjw] - number of quanta for synchronization MAX 4 quanta
 * @return - can_error_t - can error status
 */
can_error_t md_can_init_time_quanta(can_handle_t *p_hCANx, uint16_t prescaler,
                                    can_time_quanta_t quanta_ts1,
                                    can_time_quanta_t quanta_ts2,
                                    can_time_quanta_t quanta_sjw)
{
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
 * enter init mode
 * @param[*p_hCANx] - can struct handler
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
 * @param[*p_hCANx] - can struct handler
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
 * @param[*p_hCANx] - can struct handler
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
