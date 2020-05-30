/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */

/** @file 
 *
 * @defgroup hal_gpiote
 * @{
 * @ingroup nrfready_hal
 * @brief GPIOTE hardware abstraction layer.
 *
 * @note Due to PAN anomaly in silicon revision x. PIN events are too power consuming. PORT event is not.
 * @details 
 */
#ifndef __HAL_GPIOTE_H__
#define __HAL_GPIOTE_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf51.h"
#include "nrf51_bitfields.h"

/**@brief Channel number type */
typedef enum
{
    hal_gpiote_chn_0 = 0,
    hal_gpiote_chn_1 = 1,
    hal_gpiote_chn_2 = 2,
    hal_gpiote_chn_3 = 3
} hal_gpiote_chn_num_t;

typedef enum
{
    hal_gpiote_evt_0    = 0,
    hal_gpiote_evt_1    = 1,
    hal_gpiote_evt_2    = 2,
    hal_gpiote_evt_3    = 3,
    hal_gpiote_evt_port = 4,
} hal_gpiote_evt_num_t;

/**@brief Mode type */
typedef enum
{
    hal_gpiote_mode_disabled = GPIOTE_CONFIG_MODE_Disabled,
    hal_gpiote_mode_event    = GPIOTE_CONFIG_MODE_Event,
    hal_gpiote_mode_task     = GPIOTE_CONFIG_MODE_Task
} hal_gpiote_mode_t;

/**@brief Polarity type */
typedef enum
{
    hal_gpiote_pol_lotohi = GPIOTE_CONFIG_POLARITY_LoToHi,
    hal_gpiote_pol_hitolo = GPIOTE_CONFIG_POLARITY_HiToLo,
    hal_gpiote_pol_toggle = GPIOTE_CONFIG_POLARITY_Toggle
} hal_gpiote_polarity_t;

/**@brief Outinit type */
typedef enum
{
    hal_gpiote_outinit_none = 0,
    hal_gpiote_outinit_low  =  GPIOTE_CONFIG_OUTINIT_Low,
    hal_gpiote_outinit_high =  GPIOTE_CONFIG_OUTINIT_High
} hal_gpiote_outinit_t;

/**@brief GPIOTE configuration struct. */
typedef struct
{
    hal_gpiote_chn_num_t  chn;
    hal_gpiote_mode_t     mode;
    hal_gpiote_polarity_t polarity;
    hal_gpiote_outinit_t  outinit;
    uint32_t              psel;
} hal_gpiote_cfg_t;

/**@brief Callback type. */
typedef void (*hal_gpiote_callback_t)(void);

/**@brief GPIOTE initialization.
 *
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t hal_gpiote_init(void);

/**@brief GPIOTE configuration.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t hal_gpiote_cfg(const hal_gpiote_cfg_t* p_cfg);

/**@brief GPIOTE callbacks.
 *
 * @note Callbacks are called from interrupt context
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_NO_MEM 
 */
uint32_t hal_gpiote_cb_set(hal_gpiote_evt_num_t p_evt_num, hal_gpiote_callback_t p_cb);

/**@brief GPIOTE callbacks clear.
 * 
 * @param[in] p_evt_num Which event to disable
 * @param[in] p_cb      Which callback function to disable
 * @return
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t hal_gpiote_cb_clr(hal_gpiote_evt_num_t p_evt_num, hal_gpiote_callback_t p_cb);

#endif /* __HAL_GPIOTE_H__ */

/** @} */
