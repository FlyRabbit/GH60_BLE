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
 * @defgroup modules_coms_ble_adv
 * @{
 * @ingroup nrfready_modules
 * @brief BLE advertising sub-module.
 *
 * @details This sub-module includes all advertising logic: when to start advertising, and of which type.
 * @{
 */
#ifndef __M_COMS_BLE_ADV_H__
#define __M_COMS_BLE_ADV_H__

#include <stdbool.h>
#include <stdint.h>

#include "ble.h"
#include "m_coms_ble.h"

/**@brief State parameter struct to keep track of which advertising to use */
typedef struct
{
    bool     bond_initiate;       /** Indicate if a new bond should be made */
    bool     advertising_running; /** Indicate if advertising is running */
    uint8_t  directed_adv_count;  /** Count to keep track of directed advertising vs different masters */
    uint8_t  adv_type;            /** See @ref BLE_GAP_ADV_TYPES */
    uint8_t  adv_flags;           /** See @ref BLE_GAP_DISC_MODES */
} m_coms_ble_adv_params_t;

/**@brief State struct to keep track of advertising scheme */
typedef enum
{
    ADV_STATE_NONE,                 /** No advertising running */
    ADV_STATE_DIRECTED_SINGLE,      /** Single directed advertising */
    ADV_STATE_DIRECTED_MULTIPLE,    /** Multiple directed advertisings (in sequence), one targeted at each bonded Central */
    ADV_STATE_UNDIRECTED_BONDABLE,  /** Undirected advertising */
    ADV_STATE_UNDIRECTED_WHITELIST, /** Undirected advertising with whitelist (when directed cannot be used, not bondable) */
} m_coms_ble_adv_type_t;

/**@brief Struct to keep track of advertising type and dynamic parameters */
typedef struct
{
    m_coms_ble_adv_type_t   type;
    m_coms_ble_adv_params_t params;
} m_coms_ble_adv_state_t;

/**@brief Initialize advertising logic
 *
 * @note @ref p_bond_params is expected to be persistent, and is not copied locally.
 * @param[in] m_coms_ble_bond_params_t Pointer to bond parameters. 
 * @param[in] p_evt_handler            Event handler
 * @param[in] p_encryption_state       Pointer to encryption state
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_adv_init(const m_coms_ble_bond_params_t * p_bond_params,
                             app_sched_event_handler_t        p_evt_handler,
                             bool *                           p_encryption_state,
                             dm_handle_t                      p_dm_handle);

void m_coms_ble_adv_on_evt(ble_evt_t * p_ble_evt);

void m_coms_ble_adv_on_sys_evt(uint32_t sys_evt);

/**@brief Determine if advertising should be started or not.
 *
 * @param[in] p_state     State struct. Will be updated to reflect appropriate advertising type
 * @param[in] p_ble_evt   Event to respond to. May be 0.
 * @param[in] p_encrypted Encryption state
 * @return true if advertising should be started
 */
bool m_coms_ble_adv_determine(ble_evt_t * p_ble_evt);

/**@brief Start advertising according to state struct.
 *
 * @param[in] p_state       State struct. This determines type of advertising.
 * @param[in] p_evt_handler Event handler used to notify of address changes and bonding advertising
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_x
 */
uint32_t m_coms_ble_adv_start(void);

/**@brief Start bondable advertising. 
 *
 * @note When connected, a disconnect will be triggered. Use @ref m_coms_ble_adv_determine() and @ref m_coms_ble_adv_start when disconnected.
 * 
 * @params[in] p_conn_handle Connection handle
 * @params[in] p_encrypted   Encryption state
 * @return 
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_x
 */
uint32_t m_coms_ble_adv_bond_adv_start(uint16_t p_conn_handle);
                                  
/**@brief See if any type of advertising is currently running
 *
 * @return true when any advertising is running
 */                                  
bool m_coms_ble_adv_running(void);

/**@brief See if bondable advertising is currently running
 * 
 * @return true when bondable advertising is running
 */
bool m_coms_ble_adv_bondable_running(void);

bool m_coms_ble_adv_bond_stored(void);

#endif /* __M_COMS_BLE_ADV_H__ */

/** @} */
