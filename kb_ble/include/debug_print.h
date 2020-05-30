 /* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __DEBUG_PRINT_H__
#define __DEBUG_PRINT_H__

#ifdef DEBUG_PRINT
#include "hal_debug_uart.h"
#include "io_cfg.h"

#define MAX_DEBUG_DAT_LEN 32

// Maingroups
#define dbg_start_up            0x10   // Power on!
#define dbg_sys_off             0x20   // System Off
#define dbg_assert              0x30   // Assert
#define dbg_idle                0x40   // Idle
#define dbg_jump_to_bootloader  0x41   //
#define dbg_dfu_update_crc      0x43   
#define dbg_dfu_evt             0x44   // -
                                       // 0x00 BLE_DFU_START
                                       // 0x01 BLE_DFU_RECEIVE_INIT_DATA
                                       // 0x02 BLE_DFU_RECEIVE_APP_DATA
                                       // 0x03 BLE_DFU_VALIDATE
                                       // 0x04 BLE_DFU_ACTIVATE_N_RESET
                                       // 0x05 BLE_DFU_SYS_RESET
                                       // 0x06 BLE_DFU_PKT_RCPT_NOTIF_ENABLED
                                       // 0x07 BLE_DFU_PKT_RCPT_NOTIF_DISABLED
                                       // 0x08 BLE_DFU_PACKET_WRITE
                                       // 0x09 BLE_DFU_BYTES_RECEIVED_SEND      
#define dbg_ble_hids            0x45
#define dbg_ble                 0x50   // -
                                       // 0x10 BLE_GAP_EVT_CONNECTED
                                       // 0x11 BLE_GAP_EVT_DISCONNECTED
                                       // 0x12 BLE_GAP_EVT_CONN_PARAM_UPDATE
                                       // 0x13 BLE_GAP_EVT_SEC_PARAMS_REQUEST
                                       // 0x14 BLE_GAP_EVT_SEC_INFO_REQUEST
                                       // 0x15 BLE_GAP_EVT_PASSKEY_DISPLAY
                                       // 0x16 BLE_GAP_EVT_AUTH_KEY_REQUEST
                                       // 0x17 BLE_GAP_EVT_AUTH_STATUS
                                       // 0x18 BLE_GAP_EVT_CONN_SEC_UPDATE
                                       // 0x19 BLE_GAP_EVT_TIMEOUT
                                       // 0x1A BLE_GAP_EVT_RSSI_CHANGED
                                       // 0x50 BLE_GATTS_EVT_WRITE
                                       // 0x51 BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST
                                       // 0x52 BLE_GATTS_EVT_SYS_ATTR_MISSING
                                       // 0x53 BLE_GATTS_EVT_HVC
                                       // 0x54 BLE_GATTS_EVT_SC_CONFIRM
                                       // 0x55 BLE_GATTS_EVT_TIMEOUT
#define dbg_ble_oob_key         0x51   // OOB_KEY :
#define dbg_ble_bonding_start   0x52   // 
#define dbg_ble_enable          0x53   
#define dbg_ble_disable         0x54   
#define dbg_ble_addr            0x53   //
#define dbg_gzll                0x60   // -
                                       // 0x01 gzll_event_sysaddr_exchanged
                                       // 0x02 gzll_event_sysaddr_timeout
                                       // 0x03 gzll_event_hostid_exchanged
                                       // 0x04 gzll_event_hostid_rejected
                                       // 0x05 gzll_event_tx_failed
#define dbg_coms                0x70   //
#define dbg_coms_ble            0x74
#define dbg_coms_ble_wakeup_prepare 0x75
#define dbg_coms_switch         0x71   //
#define dbg_coms_enable         0x72   
#define dbg_coms_disable        0x73   
#define dbg_keyboard            0x80   //
#define dbg_kbrd_matrix_enable  0x81   //
#define dbg_kbrd_matrix_disable 0x82   //
#define dbg_kbrd_matrix_sense   0x83   
#define dbg_kbrd_polling_start  0x84   
#define dbg_kbrd_polling_stop   0x85   
#define dbg_mouse               0x90   //
#define dbg_mouse_enable        0x91   
#define dbg_mouse_disable       0x92   
#define dbg_nfc_enable          0xA0   //
#define dbg_nfc_disable         0xA1   //
#define dbg_nfc_response        0xA2   
#define dbg_nfc_drv_power_on    0xA3   
#define dbg_nfc_drv_power_off   0xA4   
#define dbg_nfc_drv_resp_init   0xA5   // -
                                       // 0x01 drv_nfc_init prev_IDN
                                       // 0x02 drv_nfc_init prev_PROTSEL
                                       // 0x03 drv_nfc_init prev_POLLFIELD
                                       // 0x05 drv_nfc_init prev_LISTEN
                                       // 0x06 drv_nfc_init prev_SEND
                                       // 0x07 drv_nfc_init prev_IDLE
                                       // 0x0D drv_nfc_init prev_ACFILTER
                                       // 0x55 drv_nfc_init prev_ECHO
                                       // 0xFF drv_nfc_init prev_NONE
#define dbg_nfc_drv_resp_listen 0xA6   // -
                                       // 0x01 drv_nfc_listen prev_IDN
                                       // 0x02 drv_nfc_listen prev_PROTSEL
                                       // 0x03 drv_nfc_listen prev_POLLFIELD
                                       // 0x05 drv_nfc_listen prev_LISTEN
                                       // 0x06 drv_nfc_listen prev_SEND
                                       // 0x07 drv_nfc_listen prev_IDLE
                                       // 0x0D drv_nfc_listen prev_ACFILTER
                                       // 0x55 drv_nfc_listen prev_ECHO
                                       // 0xFF drv_nfc_listen prev_NONE
#define dbg_nfc_drv_timeout     0xA7   
#define dbg_nfc_drv_hibernate   0xA8   
#define dbg_main                0xB0   
#define dbg_pairing_button      0xB1   // 
#define dbg_batt_meas_handler   0xB2   
#define dbg_buffer_keys         0xB3   
#define dbg_buffer_send         0xB4   
#define dbg_pwr_and_clk_mgmt    0xC0   
#define dbg_lfclk_cal_start     0xC1   
#define dbg_lfclk_cal_done      0xC2   
#define dbg_hfclk_xtal_start_fix 0xC3
#define dbg_on_sys_evt_acces_cnt 0xC4
#define dbg_conn_params         0xD0   //
                                       
/* Device Manager */                   
#define dbg_dev_mngr            0xD1   // -
                                       // 0x00 DM_EVT_RFU                     
                                       // 0x01 DM_EVT_ERROR                   
                                       // 0x11 DM_EVT_CONNECTION              
                                       // 0x12 DM_EVT_DISCONNECTION           
                                       // 0x13 DM_EVT_SECURITY_SETUP          
                                       // 0x14 DM_EVT_SECURITY_SETUP_COMPLETE 
                                       // 0x15 DM_EVT_LINK_SECURED            
                                       // 0x16 DM_EVT_SECURITY_SETUP_REFRESH  
                                       // 0x21 DM_EVT_DEVICE_CONTEXT_LOADED   
                                       // 0x22 DM_EVT_DEVICE_CONTEXT_STORED   
                                       // 0x23 DM_EVT_DEVICE_CONTEXT_DELETED  
                                       // 0x31 DM_EVT_SERVICE_CONTEXT_LOADED  
                                       // 0x32 DM_EVT_SERVICE_CONTEXT_STORED  
                                       // 0x33 DM_EVT_SERVICE_CONTEXT_DELETED 
                                       // 0x41 DM_EVT_APPL_CONTEXT_LOADED     
                                       // 0x42 DM_EVT_APPL_CONTEXT_STORED     
                                       // 0x43 DM_EVT_APPL_CONTEXT_DELETED  
#define dbg_hid                 0xD2   // -
                                       // 0x00 BLE_HIDS_EVT_HOST_SUSP
                                       // 0x01 BLE_HIDS_EVT_HOST_EXIT_SUSP
                                       // 0x02 BLE_HIDS_EVT_NOTIF_ENABLED
                                       // 0x03 BLE_HIDS_EVT_NOTIF_DISABLED
                                       // 0x04 BLE_HIDS_EVT_REP_CHAR_WRITE
                                       // 0x05 BLE_HIDS_EVT_BOOT_MODE_ENTERED
                                       // 0x06 BLE_HIDS_EVT_REPORT_MODE_ENTERED
                                       // 0x07 BLE_HIDS_EVT_REPORT_READ
#define dbg_main_coms_handler   0xD3   // -
                                       // 0x00 com_event_none
                                       // 0x01 com_event_init_finished
                                       // 0x02 com_event_connected
                                       // 0x03 com_event_disconnected
                                       // 0x04 com_event_data_received
                                       // 0x05 com_event_data_read
                                       // 0x06 com_event_timing_update
                                       // 0x07 com_event_advertising_timeout
                                       // 0x08 com_event_advertising_bondable
                                       // 0x09 com_event_address_changed
                                       // 0x0A com_event_passkey_req
                                       // 0x0B com_event_oobkey_req
                                       // 0x0C com_event_key_sent
#define dbg_m_coms_ble_evt_handle 0xD4 // -
                                       // 0x00 M_COMS_BLE_EVT_INIT_FINISHED
                                       // 0x01 M_COMS_BLE_EVT_CONNECTED
                                       // 0x02 M_COMS_BLE_EVT_DISCONNECTED
                                       // 0x03 M_COMS_BLE_EVT_DATA_RECEIVED
                                       // 0x04 M_COMS_BLE_EVT_READ_REQ
                                       // 0x05 M_COMS_BLE_EVT_CONN_UPDATE
                                       // 0x06 M_COMS_BLE_EVT_ADV_TIMEOUT
                                       // 0x07 M_COMS_BLE_EVT_ADV_BONDABLE
                                       // 0x08 M_COMS_BLE_EVT_ADDR_CHANGED
                                       // 0x09 M_COMS_BLE_EVT_PASSKEY_REQ
                                       // 0x0A M_COMS_BLE_EVT_OOBKEY_REQ
                                       // 0x0B M_COMS_BLE_EVT_KEY_SENT
                                       // 0x0C M_COMS_BLE_EVT_DISABLED
#define dbg_on_sys_evt            0xD5 // -
                                       // 0x00 NRF_EVT_HFCLKSTARTED
                                       // 0x01 NRF_EVT_POWER_FAILURE_WARNING
                                       // 0x02 NRF_EVT_FLASH_OPERATION_SUCCESS
                                       // 0x03 NRF_EVT_FLASH_OPERATION_ERROR
                                       // 0x04 NRF_EVT_RADIO_BLOCKED
                                       // 0x05 NRF_EVT_RADIO_CANCELED
                                       // 0x06 NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN
                                       // 0x07 NRF_EVT_RADIO_SESSION_IDLE
                                       // 0x08 NRF_EVT_RADIO_SESSION_CLOSED
                                       // 0x09 NRF_EVT_NUMBER_OF_EVTS
#define dbg_peer_addr             0xD6 //
#define dbg_device_addr           0xD7 //
#define dbg_m_coms_ble_addr_usage_idx_get 0xD8
#define dbg_BLE_HIDS_EVT_NOTIF_ENABLED    0xD9
#define dbg_print_addr_list               0xDB
#define dbg_m_coms_ble_addr_update        0xDC
#define dbg_usage_idx_set                 0xE5
#define dbg_addr_set                      0xE6
#define dbg_usage_idx_update              0xE7
#define dbg_m_coms_ble_addr_notify_usage  0xE8
#define dbg_m_coms_ble_addr_get           0xEB
#define dbg_m_coms_ble_addr_set           0xEA
#define dbg_usage_idx_print               0xEC
#define dbg_m_coms_ble_addr_delete        0xED
#define dbg_audio_frame_recvd             0xF0
#define dbg_M_COMS_BLE_EVT_DATA_RECEIVED  0xF1
#define dbg_flash_access_timeout          0xF2
#define dbg_buffer_send_handler           0xF3
#define dbg_BLE_HID_MAX_REPORT_NUM        0xF4
#define dbg_service_context_set           0xF5
#define dbg_get_device_ids                0xF7
#define dbg_whitelist_addr_count          0xF8
#define dbg_whitelist_irk_count           0xF9
#define dbg_whitelist_addr                0xFA
#define dbg_whitelist_irk                 0xFB
#define dbg_device_manager                0xFC
#define dbg_usage_idx_new_get             0xFD
#define dbg_usage_idx_get                 0xFE
#define dbg_m_coms_ble_addr_get_dev_id    0xFF

#define dbg_cmd_bonding_start     0x01 // <-- dbg_cmd_bonding_start
#define dbg_cmd_m_coms_disable    0x02 // <-- dbg_cmd_m_coms_disable
#define dbg_cmd_m_coms_enable     0x03 // <-- dbg_cmd_m_coms_enable 
#define dbg_cmd_bonds_delete      0x04 // <-- dbg_cmd_bonds_delete
#define dbg_cmd_sys_reset         0x05 // <-- dbg_cmd_sys_reset
#define dbg_cmd_usage_idx_print   0x06 // <-- dbg_cmd_usage_idx_print

#define debug_print(dbg, dat, dat_len)                                            \
{                                                                                 \
    uint8_t msg[MAX_DEBUG_DAT_LEN + 1];                                           \
    uint32_t len = (dat_len > MAX_DEBUG_DAT_LEN) ? MAX_DEBUG_DAT_LEN : dat_len;   \
    uint8_t *p_dat = dat;                                                         \
                                                                                  \
    msg[0] = dbg;                                                                 \
                                                                                  \
    for (uint32_t i = 0; i < len; i++)                                            \
    {                                                                             \
        msg[i + 1] = p_dat[i];                                                    \
    }                                                                             \
                                                                                  \
    hal_uart_put_stuffed(msg, len + 1);                                           \
}

#define debug_print_init()                                                                   \
{                                                                                            \
    hal_uart_config(IO_DBG_RTS, IO_DBG_TX, IO_DBG_CTS, IO_DBG_RX, IO_DBG_HWFC, 0, true);  \
}

#endif

#ifndef DEBUG_PRINT
#define debug_print(...)
#define debug_print_init(...)
#endif                                                                              

#endif
