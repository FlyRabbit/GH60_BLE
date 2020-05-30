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
 *  @brief Project-specific parameters and helper functions.
 *
 */
#ifndef __PROJECT_PARAMS_H__
#define __PROJECT_PARAMS_H__

#include <string.h>


#include "ble_types.h"
#include "m_coms.h"
#include "ble_dis.h"
#include "nrf51.h"


static const uint8_t s_keyboard_hid_descriptor[] = 
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
    
        0x95, 0x06,                 //     Report Count (6)
        0x75, 0x08,                 //     Report Size (8)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x25, 0x65,                 //     Logical Maximum (101)
        0x05, 0x07,                 //     Usage Page (Key codes)
        0x19, 0x00,                 //     Usage Minimum (0)
        0x29, 0x65,                 //     Usage Maximum (101)
        0x81, 0x00,                 //     Input (Data, Array) Key array(6 bytes)
        
        0x95, 0x05,                 //     Report Count (5)
        0x75, 0x01,                 //     Report Size (1)
        0x05, 0x08,                 //     Usage Page (Page# for LEDs)
        0x19, 0x01,                 //     Usage Minimum (1)
        0x29, 0x05,                 //     Usage Maximum (5)
        0x91, 0x02,                 //     Output (Data, Variable, Absolute), Led report
        0x95, 0x01,                 //     Report Count (1)
        0x75, 0x03,                 //     Report Size (3)
        0x91, 0x01,                 //     Output (Data, Variable, Absolute), Led report padding
    
        0x09, 0x05,                 //     Usage (Vendor Defined)
        0x15, 0x00,                 //     Logical Minimum (0)
        0x26, 0xFF, 0x00,           //     Logical Maximum (255)
        0x75, 0x08,                 //     Report Count (2)
        0x95, 0x02,                 //     Report Size (8 bit)
        0xB1, 0x02,                 //     Feature (Data, Variable, Absolute)

        0xC0                        // End Collection (Application)    
    };

/**@brief Convenience function to fill parameter struct.
 *
 * @param[in] ble_params Parameters to 
 */
static __inline void M_COMS_BLE_PARAMS_FILL(m_coms_ble_params_t * ble_params)
{
    memset(ble_params, 0, sizeof(m_coms_ble_params_t));
    
    // Device information
    ble_params->device_info.device_name       = "Super Keyboard";   
    ble_params->device_info.manufacturer_name = "Nordic Semiconductor";  
    ble_params->device_info.fw_revision       = "3.0.0";                
    ble_params->device_info.hw_revision       = 0;      // Don't have a specific hardware revision yet
    ble_params->device_info.serial_number     = 0;      // TODO: Use NRF_FICR->DEVICEID
    ble_params->device_info.pnp_product_id    = 0x0040; // PNP ID product ID
    ble_params->device_info.pnp_product_version = 0x0001; // PNP ID product version
    ble_params->device_info.pnp_vendor_id     = 0x1915; // Nordic Semiconductor vendor ID
    ble_params->device_info.pnp_vendor_id_src = BLE_DIS_VENDOR_ID_SRC_USB_IMPL_FORUM;


    
    
    // Bond behavior parameters:
    ble_params->bond_params.directed_adv       = true; // Use directed advertising vs bonded central devices.
    ble_params->bond_params.change_address     = true; // Change device address for each new bond. 
                                                       // This prevents previously connected masters from attempting reconnect.
    ble_params->bond_params.bond_reconnect_all = true; // When pairing button is pressed: 
                                                       // Attempt reconnection to other bonded masters before advertising as bondable.
    ble_params->bond_params.reconnect_all      = false;// Do not normally try to reconnect to every master, only the latest used one.
    
    // Preferred connection parameters
    ble_params->conn_params.min_conn_interval = 6;  // Minimum connection interval. In units of 1.25 ms. 
    ble_params->conn_params.max_conn_interval = 16;  // Maximum connection interval. In units of 1.25 ms.
    ble_params->conn_params.slave_latency     = 100; // Number of connection events the Peripheral can skip. 
                                                     // Note that this introduces delay in Central -> Peripheral communication
    ble_params->conn_params.conn_sup_timeout  = 610; // Connection timeout in units of 100 ms. If link members i.e. loses power it takes this long
                                                     // before the connection is considered dead.
    // Security parameters
    //ble_params->sec_params.io_capabilities = BLE_GAP_IO_CAPS_KEYBOARD_ONLY; // See @ref BLE_GAP_IO_CAPS for valid numbers.
    ble_params->sec_params.io_capabilities = BLE_GAP_IO_CAPS_NONE; // See @ref BLE_GAP_IO_CAPS for valid numbers.
    ble_params->sec_params.oob_data_availible = false;
    // Misc parameters
    ble_params->appearance       = BLE_APPEARANCE_HID_KEYBOARD;
    ble_params->base_hid_version = 0x0101; // Version number of base USB HID Specification implemented by this device
    ble_params->hid_country_code = 0;      // Country code can be used to specify which country the hardware is localized for. Most hardware is not localized (value 0).
    ble_params->max_tx_buf_cnt   = 2;      // Only allow 2 packets to be in SoftDevice TX buffer at a time to reduce max interrupt latency
    // HID Service flags: 
    // Remote wakeup indicates that this device considers itself capable of wakeing up the Central device
    // Normally connectable flag indicates that this device is normally advertising (and can be connected to) when not already connected.
    ble_params->hids_flags       =  HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;
    
    
    //
    // Parameters not initialized: THESE MUST BE INITIALIZED IN APPLICATION
    //
    ble_params->boot_mode_callback = 0; // Callback function used to translate HID reports into boot format when needed
}

/**@brief Convenience function to fill parameter struct.
 *
 * @param[in] ble_params Parameters to 
 */
static __inline void M_COMS_GZLL_PARAMS_FILL(m_coms_gzll_params_t * gzll_params)
{
    memset(gzll_params, 0, sizeof(m_coms_gzll_params_t));
    
    // Device information
    gzll_params->encrypt=true;  /** Make encrypted pipe available */
    gzll_params->tx_attempts = 400;
    gzll_params->timeslot_period = 504; //Should be the the LU1 dongle RX_PERIOD / 2
    gzll_params->sync_lifetime=1000;

    
}

#endif /* __PROJECT_PARAMS_H__ */
