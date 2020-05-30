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
 */

#include "ble_sps.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"

#define SCAN_INT_WINDOW_LEN 4 /**< Length of Scan Interval Window Characteristic. */


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_sps       Scan Parameters Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{
    p_sps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_sps       Scan Parameters Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sps->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_sps       Scan Parameters Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    ble_hids_char_id_t      char_id;

    if (p_evt_write->handle == p_sps->scan_interval_window_handles.value_handle)
    {
        on_scan_int_window_write(p_sps, p_evt_write);
    }
    else if (p_evt_write->handle == p_sps->scan_refresh_handles.cccd_handle)
    {
        on_cccd_write(p_sps, p_evt_write);
    }
}


void ble_bas_on_ble_evt(ble_sps_t * p_sps, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sps, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sps, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sps, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for encoding a Scan Interval Window.
 *
 * @param[in]   scan_interval_window   Measurement to be encoded.
 * @param[out]  p_encoded_buffer       Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t scan_int_win_encode(scan_interval_window_t scan_interval_window,
                                   uint8_t              * p_encoded_buffer)
{
    uint8_t len = 0;

    len += uint16_encode(scan_interval_window.scan_interval, p_encoded_buffer);
    len += uint16_encode(scan_interval_window.scan_window, &p_encoded_buffer[len]);

    return len;
}


/**@brief Function for adding Scan Interval Window characteristics.
 *
 * @param[in]   p_sps        Scan Parameters Service structure.
 * @param[in]   p_sps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t scan_int_window_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_scan_int_win_encoded[SCAN_INT_WINDOW_LEN];

    if (p_sps_init->p_initial_scan_int_window == NULL)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SCAN_INTERVAL_WINDOW_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sps_init->scan_interval_window_attr_md.read_perm;
    attr_md.write_perm = p_sps_init->scan_interval_window_attr_md.write_perm;

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = scan_int_win_encode(*(p_sps_init->p_initial_scan_int_window),
                                                    initial_scan_int_win_encoded);
    attr_char_value.p_value = initial_scan_int_win_encoded;

    return sd_ble_gatts_characteristic_add(p_sps->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sps->scan_interval_window_handles);
}


/**@brief Function for adding Scan Refresh characteristics.
 *
 * @param[in]   p_sps        Scan Parameters Service structure.
 * @param[in]   p_sps_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t scan_refresh_char_add(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_scan_refresh;
    uint8_t             init_len;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.write_perm = p_sps_init->scan_refresh_attr_md.cccd_write_perm;
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SCAN_REFRESH_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_sps_init->scan_refresh_attr_md.read_perm;
    attr_md.write_perm = p_sps_init->scan_refresh_attr_md.write_perm;

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    initial_scan_refresh = p_bas_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof (attr_char_value));
    initial_scan_refresh = p_sps_init->initial_scan_refresh;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof (uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof (uint8_t);
    attr_char_value.p_value   = &initial_scan_refresh;

    err_code = sd_ble_gatts_characteristic_add(p_sps->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_sps->scan_refresh_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_sps_init(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_sps->evt_handler             = p_sps_init->evt_handler;
    p_sps->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_sps->is_notification_enabled = false;
    // START HERE
    p_sps->battery_level_last = INVALID_BATTERY_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_BATTERY_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_sps->service_handle);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add battery level characteristic
    err_code = battery_level_characteristic_add(p_sps, p_sps_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}


uint32_t ble_bas_battery_level_update(ble_sps_t * p_sps, uint8_t battery_level)
{
    uint32_t err_code;

    if (battery_level != p_sps->battery_level_last)
    {
        uint16_t len = sizeof(uint8_t);

        // Save new battery value
        p_sps->battery_level_last = battery_level;

        // Update database
        err_code = sd_ble_gatts_value_set(p_sps->battery_level_handles.value_handle,
                                          0,
                                          &len,
                                          &battery_level);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Send value if connected and notifying
        if ((p_sps->conn_handle != BLE_CONN_HANDLE_INVALID) && p_sps->is_notifying)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));
            len = sizeof(uint8_t);

            hvx_params.handle = p_sps->battery_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = 0;
            hvx_params.p_len  = &len;
            hvx_params.p_data = &battery_level;

            err_code = sd_ble_gatts_hvx(p_sps->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
