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
 * @defgroup modules_coms_ble_addr
 * @{
 * @ingroup nrfready_modules
 * @brief 
 *
 * @brief Flash address storage utility module.
 *
 * @details This module deals with storing and maintaining locla address information in flash
 *
 */
#ifndef __M_COMS_BLE_ADDR_H__
#define __M_COMS_BLE_ADDR_H__

#include <stdbool.h>
#include <stdint.h>

#include "device_manager.h"

//typedef enum
//{
//    M_COMS_BLE_ADDR_EVT_SET,
//    M_COMS_BLE_ADDR_EVT_GET,
//    M_COMS_BLE_ADDR_EVT_DELETE,
//    M_COMS_BLE_ADDR_EVT_NOTIFY
//}m_coms_ble_addr_evt_t;

uint32_t m_coms_ble_addr_init(dm_application_instance_t *p_app_id);

/**@brief Write currently used ble address to the application context flash area of the device manager.
 *
 * @params[in] dm_handle  Device manager device handle. 
 * @params[in] p_ble_addr BLE address used with current bond
 */
uint32_t m_coms_ble_addr_set(dm_handle_t *dm_handle, ble_gap_addr_t *p_ble_addr);

/**@brief Get the ble address from the application context flash area of the device manager.
 *
 * @params[in]  app_id      Application instance.
 * @params[in]  addr_idx    Index of the address read.
 * @params[out] p_addr      BlE GAP address.
 * @params[out] p_device_id The device instance.
 */
uint32_t m_coms_ble_addr_addr_get(dm_application_instance_t app_id,
                                  uint32_t                  addr_idx,
                                  ble_gap_addr_t            *p_addr,
                                  dm_device_instance_t      *p_device_id);

/**@brief Delete a specific application context flash area of the device manager.
 *
 * @params[in]  app_id      Application instance.
 * @params[in]  addr_idx    Index of the address read.
 * @params[out] p_device_id The device instance of the deleted device.
 */
uint32_t m_coms_ble_addr_delete(dm_application_instance_t app_id,
                                uint32_t                  addr_idx,
                                dm_device_instance_t      *p_device_id);

/**@brief Notify usage of a specific bonded host.
 *
 * @params[in]  dm_handle      Device manager handler indicating used bonded host.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 */
uint32_t m_coms_ble_addr_notify_usage(dm_handle_t *dm_handle, ble_gap_addr_t *p_ble_addr);

/**@brief Get device ids and number of bonded hosts.
 *
 * @params[in]   app_id      Device manager application id.
 * @params[out]  device_ids  Pointer to array of device ids.
 * @params[out]  device_num  Number of device ids found.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 */
uint32_t m_coms_ble_addr_device_ids_get(dm_application_instance_t app_id, 
                                        dm_device_instance_t      *p_device_ids, 
                                        uint8_t                   *p_device_num);

/**@brief Get a new ble addr used for new bonds.
 *
 * @params[in]   app_id      Device manager application id.
 * @params[out]  addr        Pointer to new address structure.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 */
uint32_t m_coms_ble_addr_new_get(dm_application_instance_t app_id, 
                                 ble_gap_addr_t *addr);


#endif /* __M_COMS_BLE_ADDR_H__ */

/** @} */
