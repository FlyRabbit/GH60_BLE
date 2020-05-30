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
 * @defgroup modules_coms_ble_hid
 * @{
 * @ingroup nrfready_modules
 * @brief BLE HID (HID over GATT) sub-module.
 *
 * @details 
 * @{
 */
 
#ifndef __M_COMS_BLE_HID_CONF_H__
#define __M_COMS_BLE_HID_CONF_H__

#define BLE_HID_MAX_INTERFACE_NUM 1  /**@brief Max number of HID Services that can be created */
#define BLE_HID_MAX_REPORT_NUM    3 /**@brief Max number of HID Reports (in total for all services) that can be created */
#define BLE_HID_MAX_EXT_MAPPINGS  1  /**@brief Max number of External Reports that can be mapped to HID Services */

#endif /* __M_COMS_BLE_HID_CONF_H__ */
