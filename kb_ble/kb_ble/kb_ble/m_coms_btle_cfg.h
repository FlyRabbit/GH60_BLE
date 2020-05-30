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
 
/** @file Communications module bluetooth configuration.
 *
 * @defgroup modules_coms_btle_cfg_keyboard
 * @{
 * @ingroup nrfready_modules
 * @brief 
 *
 * @details Each HID over GATT-device differs quite a bit in their configuration.
 *          This file is used to deal with device-specifics and keep @ref modules_coms_btle generic.
 */

#ifndef __M_COMS_BTLE_CONFIG_H__
#define __M_COMS_BTLE_CONFIG_H__

#include "common_params.h"

#define DEVICE_NAME                      "Desktop 2 Keyboard"               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemi"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define PNP_ID_VENDOR_ID_SOURCE          0x02                               /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                 0x1915                             /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID                0x0040                             /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION           0x0001                             /**< Product Version. */

#define BLE_APPEARANCE                  BLE_APPEARANCE_HID_KEYBOARD    

#define APP_ADV_INTERVAL_FAST            64                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_INTERVAL_SLOW            64                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 60 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                     /**< The advertising timeout in units of seconds. */
#define APP_FAST_ADV_TIMEOUT             180                     /**< The duration of the fast advertising period (in seconds). */
#define APP_DIRECTED_ADV_TIMEOUT         5                       /**< number of direct advertisement (each lasting 1.28seconds). */

#define MIN_CONN_INTERVAL                16                                 /**< Minimum connection interval (15 ms) */
#define MAX_CONN_INTERVAL                16                                /**< Maximum connection interval (30 ms). */
#define SLAVE_LATENCY                    100                               /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 (6 * 100)                          /**< Connection supervisory timeout (6000 ms). */  
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5 * 1000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30 * 1000, APP_TIMER_PRESCALER)    /**< Time between each call to ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                                  /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT                30                                 /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                   1                                  /**< Perform bonding. */
#define SEC_PARAM_MITM                   1                                  /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_KEYBOARD_ONLY               /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    1                                  /**< Out Of Band data available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                  /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                 /**< Maximum encryption key size. */
   
#define OUTPUT_REPORT_INDEX              0                                  /**< Index of Output Report. */
#define OUTPUT_REPORT_MAX_LEN            1                                  /**< Maximum length of Output Report. */
#define INPUT_REPORT_KEYS_INDEX          0                                  /**< Index of Input Report. */
#define OUTPUT_REPORT_BIT_MASK_CAPS_LOCK 0x02                               /**< CAPS LOCK bit in Output Report (based on 'LED Page (0x08)' of the Universal Serial Bus HID Usage Tables). */
#define INPUT_REP_REF_ID                 0                                  /**< Id of reference to Keyboard Input Report. */
#define OUTPUT_REP_REF_ID                1                                  /**< Id of reference to Keyboard Output Report. */

#define INPUT_REP_BUTTONS_LEN           3                                   /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                   /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                   /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                   /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                   /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                   /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                   /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                   /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                   /**< Id of reference to Mouse Input Report containing media player data. */

#define FLASH_PAGE_SYS_ATTR              253                                            /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                  255 

#define INPUT_REPORT_KEYS_MAX_LEN 8
     
#define BASE_USB_HID_SPEC_VERSION       0x0101                              /**< Version number of base USB HID Specification implemented by this application. */

#define PASSKEY_TIMEOUT_MS              30000
#define OOB_KEY_LEN                     16
#define MTU_SIZE                        BLE_L2CAP_MTU_DEF

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t *p_evt);
static void service_error_handler(uint32_t nrf_error);

/**@brief Project-specific HID configuration function
 * 
 * @param[out] hids Pointer to ble_hids_t struct
 * @return
 * @retval NRF_SUCCESS
 */
static __inline uint32_t user_hids_init(ble_hids_t *hids)
{
    ble_hids_init_t            hids_init_obj;
    ble_hids_inp_rep_init_t    input_report_array[1];
    ble_hids_inp_rep_init_t  * p_input_report;
    ble_hids_outp_rep_init_t   output_report_array[1];
    ble_hids_outp_rep_init_t * p_output_report;
    uint8_t                    hid_info_flags;

    static uint8_t report_map_data[] =
    {
        0x05, 0x01,                 // Usage Page (Generic Desktop)
        0x09, 0x06,                 // Usage (Keyboard)
        0xA1, 0x01,                 // Collection (Application)
        0x05, 0x07,                 //     Usage Page (Key Codes)
        0x19, 0xe0,                 //     Usage Minimum (224)
        0x29, 0xe7,                 //     Usage Maximum (231)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x01,                 //     Logical Maximum (1)
        0x75, 0x01,                 //     Report Size (1)
        0x95, 0x08,                 //     Report Count (8)
        0x81, 0x02,                 //     Input (Data, Variable, Absolute)
    
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x08,                 //     Report Size (8)
        0x81, 0x01,                 //     Input (Constant) reserved byte(1)
    
        0x95, 0x05,                 //     Report Count (5)
        0x75, 0x01,                 //     Report Size (1)
        0x05, 0x08,                 //     Usage Page (Page# for LEDs)
        0x19, 0x01,                 //     Usage Minimum (1)
        0x29, 0x05,                 //     Usage Maximum (5)
        0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x03,                 //     Report Size (3)
        0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding
    
        0x95, 0x06,                 //     Report Count (6)
        0x75, 0x08,                 //     Report Size (8)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x65,                 //     Logical Maximum (101)
        0x05, 0x07,                 //     Usage Page (Key codes)
        0x19, 0x00,                 //     Usage Minimum (0)
        0x29, 0x65,                 //     Usage Maximum (101)
        0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)
    
        0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)

        0xC0                        // End Collection (Application)    
    };

    // Initialize HID Service
    p_input_report                      = &input_report_array[INPUT_REPORT_KEYS_INDEX];
    p_input_report->max_len             = INPUT_REPORT_KEYS_MAX_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&p_input_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

    p_output_report                      = &output_report_array[OUTPUT_REPORT_INDEX];
    p_output_report->max_len             = OUTPUT_REPORT_MAX_LEN;
    p_output_report->rep_ref.report_id   = OUTPUT_REP_REF_ID;
    p_output_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_OUTPUT;
    
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_output_report->security_mode.write_perm);

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = true;
    hids_init_obj.is_mouse                       = false;
    hids_init_obj.inp_rep_count                  = 1;
    hids_init_obj.p_inp_rep_array                = input_report_array;
    hids_init_obj.outp_rep_count                 = 1;
    hids_init_obj.p_outp_rep_array               = output_report_array;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(report_map_data);
    hids_init_obj.rep_map.p_data                 = report_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_inp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_boot_kb_inp_rep.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_kb_outp_rep.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

    return ble_hids_init(hids, &hids_init_obj);
}

/**@brief Project-specific check of supported packet types
 * 
 * @param[in] type Packet type to check
 * @return True if packet is supported by this device
 */
static __inline bool user_is_packet_supported(packet_type_t type)
{
    switch (type)
    {
        case packet_type_keyboard:
            /* Fall through */
        case packet_type_battery_level:
            return true;
        default:
            return false;
    }
}

#endif
