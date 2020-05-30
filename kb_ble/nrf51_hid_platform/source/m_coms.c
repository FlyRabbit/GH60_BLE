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
 
#include "m_coms.h"

#include <string.h>

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "m_coms_ble.h"
#include "m_coms_gzll.h"
#include "nrf_assert.h"
#include "nrf_error.h"
//#include "hal_rng.h"
#include "debug_print.h"
#include "app_util_platform.h"

#define M_COMS_ADV_TIMEOUTS     2

typedef enum
{
    M_COMS_BONDING_STATE_IDLE,
    M_COMS_BONDING_STATE_GZLL,
    M_COMS_BONDING_STATE_BLE
}m_coms_bonding_state_t;

static protocol_mode_t        m_coms_protocol_scheme; /** Which protocol(s) to use. */
static protocol_mode_t        m_coms_protocol_used;   /** Currently used protocol. */
static uint32_t               m_coms_timeouts;        /** How many advertising timeouts has occured. Used in protocol_mode_auto. */
static m_coms_bonding_state_t m_coms_bonding_state = M_COMS_BONDING_STATE_IDLE;
//static protocol_mode_t        m_coms_protocol_switch_pending = protocol_mode_void;
app_sched_event_handler_t     m_coms_event_callback;  /** Event handler used to notify application of events. */

/* Static function prototypes */
//static void m_coms_switch_protocol(void);
static void m_coms_ble_evt_handler(void * p_event_data, uint16_t event_size);
//static void m_coms_gzll_evt_handler(void * p_event_data, uint16_t event_size);
static void m_coms_bonding_handler(void * p_event_data, uint16_t event_size);

static void m_coms_regret_write(protocol_mode_t protocol)
{
     uint32_t err_code;
     uint8_t  softdevice_enabled;
     
     err_code = sd_softdevice_is_enabled(&softdevice_enabled);
     APP_ERROR_CHECK(err_code);
     if (softdevice_enabled != 0)
     {   
         err_code = sd_power_gpregret_clr(0x0000000E);
         APP_ERROR_CHECK(err_code);
         err_code = sd_power_gpregret_set(((uint32_t)protocol) << 1);
         APP_ERROR_CHECK(err_code);
     }
     else
     {
         NRF_POWER->GPREGRET &= ~0x0000000E;
         NRF_POWER->GPREGRET |= ((uint32_t)protocol) << 1;
     }
}

//static protocol_mode_t m_coms_regret_read(void)
//{
//    uint32_t err_code;
//    uint8_t  softdevice_enabled;
//    uint32_t regret;

//    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
//    APP_ERROR_CHECK(err_code);
//    if (softdevice_enabled != 0)
//    {   
//        err_code = sd_power_gpregret_get(&regret);
//        APP_ERROR_CHECK(err_code);
//    }
//    else
//    {
//        regret = NRF_POWER->GPREGRET;
//    }

//    return (protocol_mode_t)((regret & 0x0000000E) >> 1);
//}

//static void m_coms_switch_protocol(void)
//{
//    uint32_t       err_code;
//    
//    switch (m_coms_protocol_used)
//    {
//        case protocol_mode_gzll:
//            m_coms_protocol_switch_pending = protocol_mode_ble; 

//            err_code = m_coms_gzll_disable();
//            APP_ERROR_CHECK(err_code);
//            break;
//        //
//        case protocol_mode_ble:
//            m_coms_protocol_switch_pending = protocol_mode_gzll;

//            err_code = m_coms_ble_disable();
//            APP_ERROR_CHECK(err_code);
//            break;
//        //
//        default:
//            APP_ERROR_CHECK_BOOL(false);
//            break;
//    }
//}

//static void m_coms_switch_protocol_handler(void * p_event_data, uint16_t event_size)
//{
//    uint32_t err_code;
//    
//    switch(m_coms_protocol_switch_pending)
//    {
//        case protocol_mode_ble:
//            err_code = m_coms_ble_enable(true);
//            if (err_code != NRF_ERROR_TIMEOUT)
//            {   // TODO: Verify that NRF_ERROR_TIMEOUT should not cause an ASSERT.
//                APP_ERROR_CHECK(err_code);
//            }

//            m_coms_protocol_used           = protocol_mode_ble;
//            m_coms_protocol_switch_pending = protocol_mode_void;     
//            break;
//        //
//        case protocol_mode_gzll:
//            err_code = m_coms_gzll_enable(true);
//            APP_ERROR_CHECK(err_code);

//            m_coms_protocol_used           = protocol_mode_gzll;
//            m_coms_protocol_switch_pending = protocol_mode_void;
//            break;
//        //
//        case protocol_mode_void:
//        // Fall through
//        default:
//            APP_ERROR_CHECK_BOOL(false);
//            break;
//    }
//}

static void m_coms_ble_evt_handler(void * p_event_data, uint16_t event_size)
{
    m_coms_ble_evt_t * p_evt = (m_coms_ble_evt_t *) p_event_data;;
    m_coms_evt_t       evt;
    uint32_t           err_code;
    
     evt.type = com_event_none;
    
    APP_ERROR_CHECK_BOOL(event_size == sizeof(m_coms_ble_evt_t));
    
//    debug_print(dbg_m_coms_ble_evt_handle, (uint8_t *)&p_evt->type, 1);
    
    switch (p_evt->type)
    {
        case M_COMS_BLE_EVT_INIT_FINISHED:
            evt.type = com_event_init_finished;
            break;
        
        case M_COMS_BLE_EVT_CONNECTED:
            m_coms_regret_write(protocol_mode_ble);
        
            if (m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE)
            {
                m_coms_bonding_state = M_COMS_BONDING_STATE_IDLE;
            }
            evt.type = com_event_connected;
            break;

        case M_COMS_BLE_EVT_DISCONNECTED:
            if(m_coms_bonding_state == M_COMS_BONDING_STATE_IDLE)
            {
                evt.type = com_event_disconnected;
            }
            break;
        
        case M_COMS_BLE_EVT_DATA_RECEIVED:
            evt.type                             = com_event_data_received;
            evt.data.data_received.data          = p_evt->data.data_received.data;
            evt.data.data_received.len           = p_evt->data.data_received.len;
            evt.data.data_received.interface_idx = p_evt->data.data_received.interface_idx;
            evt.data.data_received.report_type   = p_evt->data.data_received.report_type;
            evt.data.data_received.report_idx    = p_evt->data.data_received.report_idx;
            break;
        
        case M_COMS_BLE_EVT_CONN_UPDATE:
            evt.type                                   = com_event_timing_update;
            evt.data.timing_update.max_conn_interval   = p_evt->data.conn_update.max_conn_interval;
            evt.data.timing_update.min_conn_interval   = p_evt->data.conn_update.min_conn_interval;
            evt.data.timing_update.slave_latency       = p_evt->data.conn_update.slave_latency;
            evt.data.timing_update.supervision_timeout = p_evt->data.conn_update.supervision_timeout;
            break;
        
        case M_COMS_BLE_EVT_ADV_TIMEOUT:
            ++m_coms_timeouts;
            if ((m_coms_timeouts == M_COMS_ADV_TIMEOUTS) || (m_coms_protocol_scheme != protocol_mode_auto) || (m_coms_ble_bond_stored()))
            {
                if (m_coms_bonding_state == M_COMS_BONDING_STATE_IDLE)
                {
                    // Notify the application of the advertising timeout
                    evt.type             = com_event_advertising_timeout;
                }
                m_coms_timeouts = 0;
            }                
//            else if ((m_coms_protocol_scheme == protocol_mode_auto) && (m_coms_gzll_keys_stored()))
//            {
//                // We're in auto mode and no BTLE bond is stored: switch protocol
//                m_coms_switch_protocol();
//            }
            break;
        
        case M_COMS_BLE_EVT_ADDR_CHANGED:
            evt.type = com_event_address_changed;
            break;
        
        case M_COMS_BLE_EVT_PASSKEY_REQ:
            evt.type = com_event_passkey_req;
            break;
        
        case M_COMS_BLE_EVT_OOBKEY_REQ:
            evt.type = com_event_oobkey_req;
            break;        
        
        case M_COMS_BLE_EVT_KEY_SENT:
            evt.type = com_event_key_sent;
            break;
        
        case M_COMS_BLE_EVT_READ_REQ:
            evt.type = com_event_data_read;
            evt.data.data_read.interface_idx = p_evt->data.read_req.interface_idx;
            evt.data.data_read.report_idx    = p_evt->data.read_req.report_idx;
            break;
        
        case M_COMS_BLE_EVT_ADV_BONDABLE:
            if (m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE)
            {
                m_coms_bonding_state = M_COMS_BONDING_STATE_IDLE;
            }        
            evt.type = com_event_advertising_bondable;
            break;
        
        case M_COMS_BLE_EVT_DISABLED:
            if(m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE)
            {
                err_code = app_sched_event_put(0, 0, m_coms_bonding_handler);
                APP_ERROR_CHECK(err_code);
            }
//            else if( m_coms_protocol_switch_pending == protocol_mode_gzll)
//            {
//                err_code = app_sched_event_put(0, 0, m_coms_switch_protocol_handler);
//                APP_ERROR_CHECK(err_code); 
//            }
            else
            {
                evt.type = com_event_disabled;
            }
            
            break;
        
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    
    if (evt.type != com_event_none)
    {
        uint32_t err_code;
        
        // Relay event to the application
        err_code = app_sched_event_put(&evt, sizeof(evt), m_coms_event_callback);
        APP_ERROR_CHECK(err_code);
    }
}

//static void m_coms_gzll_evt_handler(void * p_event_data, uint16_t event_size)
//{
//    gzll_event_t * p_evt = (gzll_event_t *) p_event_data;
//    m_coms_evt_t   evt;
//    uint32_t       err_code;
//    
//    evt.type = com_event_none; 
//    
//    APP_ERROR_CHECK_BOOL(event_size == sizeof(gzll_event_t));
//    
//    switch (*p_evt)
//    {
//        case gzll_event_sysaddr_exchanged:
//            m_coms_regret_write(protocol_mode_gzll);
//        
//            if (m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE)
//            {
//                m_coms_bonding_state = M_COMS_BONDING_STATE_IDLE;
//            }
//            evt.type = com_event_connected;
//            dfu_update_crc();
//            break;
//        
//        case gzll_event_hostid_exchanged:
//            dfu_update_crc();
//            break;
//        
//        case gzll_event_tx_failed:
//            evt.type = com_event_disconnected;
//            break;
//        
//        case gzll_event_hostid_rejected:
//            /* Fall through */
//        case gzll_event_sysaddr_timeout: 
//            ++m_coms_timeouts;
//            if (((m_coms_timeouts == M_COMS_ADV_TIMEOUTS) || (m_coms_gzll_keys_stored())) && (m_coms_protocol_scheme != protocol_mode_auto))
//            {
//                // Notify the application that no Gazell host is found
//                evt.type        = com_event_advertising_timeout;
//                m_coms_timeouts = 0;
//            }                                
//            else if ((m_coms_protocol_scheme == protocol_mode_auto) && ((m_coms_regret_read()==protocol_mode_void) || m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE))
//            {
//                // We're in auto mode and no gzp keys are stored: switch protocol
//                m_coms_switch_protocol();
//            }
//            else
//            {
//                evt.type = com_event_disconnected;
//            }
//            break;
//        
//        case gzll_event_disabled:
//            if (m_coms_bonding_state != M_COMS_BONDING_STATE_IDLE)
//            {
//                err_code = app_sched_event_put(0, 0, m_coms_bonding_handler);
//                APP_ERROR_CHECK(err_code);                
//            }
//            else if (m_coms_protocol_switch_pending == protocol_mode_ble)
//            {
//                err_code = app_sched_event_put(0, 0, m_coms_switch_protocol_handler);
//                APP_ERROR_CHECK(err_code);                  
//            }
//            else
//            {
//                evt.type = com_event_disabled;
//            }
//        break;

//        default:
//            APP_ERROR_CHECK_BOOL(false);
//            break;
//    }
//    
//    if (evt.type != com_event_none)
//    {
//        uint32_t err_code;
//        
//        // Relay event to the application
//        err_code = app_sched_event_put(&evt, sizeof(evt), m_coms_event_callback);
//        APP_ERROR_CHECK(err_code);
//    }
//}
 
uint32_t m_coms_init(const m_coms_init_t* params)
{
    uint32_t                  err_code = NRF_SUCCESS;
    
    // Checking parameters
    if ((params->event_callback == 0) || 
        (params->protocol > protocol_mode_auto))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_coms_protocol_scheme    = params->protocol;
    m_coms_event_callback     = params->event_callback;
    m_coms_timeouts           = 0;  
    
    err_code = m_coms_ble_init(m_coms_ble_evt_handler, &params->ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
//    err_code = m_coms_gzll_init(m_coms_gzll_evt_handler, &params->gzll_params);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
    
    return err_code;
}

uint32_t m_coms_enable(void)
{
    uint32_t err_code;
	
	  err_code = m_coms_ble_enable(true);
    m_coms_protocol_used = protocol_mode_ble;
    
//    switch (m_coms_protocol_scheme)
//    {
//        case protocol_mode_gzll:
//            err_code = m_coms_gzll_enable(true);
//            m_coms_protocol_used = protocol_mode_gzll;
//            break;
//        
//        case protocol_mode_ble:
//            err_code = m_coms_ble_enable(true);
//            m_coms_protocol_used = protocol_mode_ble;
//            break;
//        
//        case protocol_mode_auto:
//            // Automatic protocol decision:
//            // If Gazell or BTLE has been paired/bonded before: go with that
//            // If it's the first startup or pairing button has been pressed: Try Gazell first, then BTLE, then Gazell, etc. etc.
//            // (It's faster to detect a Gazell host than a BTLE Central Device, so Gazell is tried before BTLE.)                     
//            if (m_coms_regret_read() == protocol_mode_ble)
//            {
//                err_code = m_coms_ble_enable(true);
//                m_coms_protocol_used = protocol_mode_ble;
//            }
//            else
//            {
//                err_code = m_coms_gzll_enable(true);
//                m_coms_protocol_used = protocol_mode_gzll;
//            }
//            break;
//        
//        default:
//            return NRF_ERROR_INVALID_STATE;
//    }
    
    return err_code;
}

uint32_t m_coms_disable(void)
{
//    switch (m_coms_protocol_used)
//    {
//        case protocol_mode_gzll:
//            return m_coms_gzll_disable();
//        
//        case protocol_mode_ble:
//            return m_coms_ble_disable();
//        
//        default:
//            break;
//    }
//    return NRF_ERROR_INTERNAL;
		
		return m_coms_ble_disable();
}

uint32_t m_coms_bonding_start(void)
{
    uint32_t       err_code = NRF_SUCCESS;
    if(m_coms_bonding_state == M_COMS_BONDING_STATE_IDLE)
    {
//        switch(m_coms_protocol_scheme)
//        {
//            case protocol_mode_gzll:
//                err_code = m_coms_gzll_bonding_start();
//                break;
//            case protocol_mode_ble:
//                err_code = m_coms_ble_bonding_start();
//                break;
//            case protocol_mode_auto:
//                // Always try gazell first.
//                m_coms_bonding_state = M_COMS_BONDING_STATE_GZLL;

//                if (m_coms_protocol_used == protocol_mode_ble)
//                {
//                        
//                    // Disable ble and wait for M_COMS_BLE_EVT_DISABLED event.
//                    err_code = m_coms_ble_disable();
//                }
//                else
//                {
//                    // Disable gzll and wait for gzll_event_disabled event.
//                    err_code = m_coms_gzll_disable();

//                }        
//            
//                break;
//            default:
//                break;
//            
//        }
				err_code = m_coms_ble_bonding_start();
				m_coms_bonding_state = M_COMS_BONDING_STATE_BLE;
    }
    
    return err_code;
}

static void m_coms_bonding_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t err_code;
    
    switch(m_coms_bonding_state)
    {       
//        case M_COMS_BONDING_STATE_GZLL:
//        {
//            m_coms_protocol_used = protocol_mode_gzll;

//            err_code = m_coms_gzll_enable(false);
//            APP_ERROR_CHECK(err_code);

//            if (m_coms_gzll_bonding_start() != NRF_SUCCESS)
//            {
//                // Bonding with gazell dongle failed. Try ble!
//                m_coms_bonding_state = M_COMS_BONDING_STATE_BLE;

//                err_code = m_coms_gzll_disable();
//                APP_ERROR_CHECK(err_code);
//            }
//            else
//            {   // TODO: verify that we should not set M_COMS_BONDING_STATE_IDLE here.
//                // Bonding succeded, wait for gzll_event_sysaddr_exchanged event to set bonding state idle.
//            }
//        }
//        break;
        //
        case M_COMS_BONDING_STATE_BLE:
        {
            m_coms_protocol_used = protocol_mode_ble;

            err_code = m_coms_ble_enable(false);
            APP_ERROR_CHECK(err_code);
            
            err_code = m_coms_ble_bonding_start();
            APP_ERROR_CHECK(err_code);
        }
        break;
        //
        case M_COMS_BONDING_STATE_IDLE:
        // Fall through
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
        
    //m_coms_bonding = false;
}

uint32_t m_coms_passkey_set(uint8_t * p_key, uint8_t p_key_len)
{
    return NRF_SUCCESS;    
}

uint32_t m_coms_hid_report_descriptor_add(const uint8_t * p_descriptor, 
                                          uint16_t        p_descriptor_len, 
                                          uint8_t         p_boot_type_bitmask, 
                                          uint8_t *       p_interface_idx)
{
    if (m_coms_protocol_scheme == protocol_mode_ble ||
        m_coms_protocol_scheme == protocol_mode_auto)
    {
        return m_coms_ble_report_descriptor_add(p_descriptor,
                                                p_descriptor_len,
                                                p_boot_type_bitmask,
                                                p_interface_idx);
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t m_coms_hid_report_id_map(uint8_t                  p_interface_id, 
                                  m_coms_hid_report_type_t p_report_type, 
                                  bool                     p_read_resp,
                                  uint8_t                  p_report_id, 
                                  uint8_t                  p_report_len,
                                  uint8_t *                p_report_idx)
{
    if (m_coms_protocol_scheme == protocol_mode_ble ||
        m_coms_protocol_scheme == protocol_mode_auto)
    {
        return m_coms_ble_report_id_map(p_interface_id, 
                                        p_report_type,
                                        p_read_resp,
                                        p_report_id, 
                                        p_report_len, 
                                        p_report_idx);
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t m_coms_hid_report_id_map_external(uint8_t                  p_interface_id, 
                                           m_coms_hid_report_type_t p_report_type, 
                                           uint8_t                  p_report_id, 
                                           uint16_t                 p_external_char_uuid, 
                                           uint16_t                 p_external_service_uuid)
{
    if (m_coms_protocol_scheme == protocol_mode_ble ||
        m_coms_protocol_scheme == protocol_mode_auto)
    {    
       return m_coms_ble_report_id_map_external(p_interface_id, 
                                                p_report_type, 
                                                p_report_id, 
                                                p_external_char_uuid, 
                                                p_external_service_uuid);
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t m_coms_hid_report_send(uint8_t * p_data,
                                uint8_t   p_len,
                                uint8_t   p_hid_interface,
                                uint8_t   p_report_idx)
{
//    if (m_coms_bonding)
//    {
//        // Hackish way of discarding packets during bondable advertising
//        // Reasoning: bondable advertising times out after 3 minutes. It's not ideal to buffer and send 
//        // 3 minute old packets when connected
//        return NRF_SUCCESS;
//    }
    
    uint32_t err_code = NRF_SUCCESS;
    switch (m_coms_protocol_used)
    {
//        case protocol_mode_gzll:
//            err_code =  m_coms_gzll_send(p_hid_interface, p_data, p_len);
//            break;
        
        case protocol_mode_ble:
            err_code =  m_coms_ble_hid_report_send(p_data, p_len, p_hid_interface, p_report_idx);
            break;
        
        default:
            err_code = NRF_ERROR_INTERNAL;
            break;
    }
    if (err_code == NRF_ERROR_INVALID_PARAM)
    {
        // As a specific protocol might not deal with all packet types we're ignoring this error
        err_code = NRF_SUCCESS;
    }
    
    return err_code;
}

#ifdef USE_MULTIPLE_HID_REPS
uint32_t m_coms_multiple_hid_reports_send(const uint8_t * p_data,
                                         uint8_t          p_len,
                                         uint8_t          p_hid_interface,
                                         uint8_t          p_report_idx,
                                         bool           * p_status)
{
    uint32_t err_code;
    
    switch (m_coms_protocol_used)
    {
//        case protocol_mode_gzll:
//            err_code = NRF_ERROR_NOT_SUPPORTED;
//            break;
        
        case protocol_mode_ble:
            err_code = m_coms_ble_multiple_hid_reports_send(p_data,
                                                            p_len,
                                                            p_hid_interface,
                                                            p_report_idx,
                                                            p_status);
            break;
        
        default:
            err_code = NRF_ERROR_INTERNAL;
            break;
    }
    
    return err_code;
}
#endif /* USE_MULTIPLE_HID_REPS */

uint32_t m_coms_read_respond(uint8_t * p_data, uint8_t p_len)
{
//    if (m_coms_protocol_used == protocol_mode_gzll)
//    {
//        return NRF_ERROR_INVALID_STATE;
//    }
    
    return m_coms_ble_read_respond(p_data, p_len);
}

uint32_t m_coms_battery_level_update(uint8_t p_battery_level)
{
    return m_coms_ble_battery_level_update(p_battery_level);
}

__inline protocol_mode_t m_coms_protocol_mode_get(void)
{
    return m_coms_protocol_used;
}

bool m_coms_wakeup_prepare(bool wakeup)
{
    bool retval = true;
    
    m_coms_regret_write(m_coms_protocol_used);
    
    switch (m_coms_protocol_used)
    {
//        case protocol_mode_gzll:
//            retval = m_coms_gzll_wakeup_prepare();
//            break;
        
        case protocol_mode_ble:
            retval = m_coms_ble_wakeup_prepare(true);
            break;
        
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    return retval;
}
