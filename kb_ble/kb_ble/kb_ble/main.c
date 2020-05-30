
#include <stdint.h>
#include <string.h>
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_timer_appsh.h"
#include "device_manager.h"
#include "common_params.h"
#include "io_cfg.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_error.h"
#include "nrf_gzp_config.h"
#include "nrf_nvmc.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

#include "debug_print.h"

#include "m_batt_meas.h"
#include "m_coms.h"
#include "m_coms_ble.h"

#include "m_keyboard.h"
#include "m_pwr_and_clk_mgmt.h"
#include "project_params.h"

#include "hal_gpiote.h"
#include "hal_rng.h"

#define SCHED_QUEUE_SIZE           20
#define SCHED_MAX_EVENT_DATA_SIZE    MAX(sizeof(ble_evt_t), MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(m_coms_evt_t)))      /**< Maximum size of scheduler events. */


#define APP_TIMER_MAX_TIMERS    11 // One for each module + one for ble_conn_params + a few extra
#define APP_TIMER_OP_QUEUE_SIZE 9 // Maximum number of timeout handlers pending execution

#define PACKET_BUFFER_SIZE SCHED_QUEUE_SIZE + 1 // Include empty element
    
static enum
{
    state_disconnected,
    state_connected
} s_connection_state = state_disconnected;

static struct
{
    m_keyboard_data_t buffer[PACKET_BUFFER_SIZE]; 
    uint32_t start_idx;
    uint32_t end_idx;
} s_packet_buffer;

static bool            s_buffer_timer_running = false;
static app_timer_id_t  s_packet_buffer_id = 0;
bool            				pair_btn_pressed = false;

// Static function declarations
static void modules_enable(void);
static void m_keyboard_handler(void * p_event_data, uint16_t event_size);
static void m_coms_handler(void * p_event_data, uint16_t event_size);
static void m_batt_meas_handler(void * p_event_data, uint16_t event_size);
static void m_pwr_and_clk_mgmt_handler(void * p_event_data, uint16_t event_size);
static void hid_boot_mode_handler(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                  m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                  uint8_t *                    p_data, 
                                  uint8_t                      p_len, 
                                  uint8_t                      p_hid_interface, 
                                  uint8_t                      p_report_idx);

static uint8_t            s_passkey_idx = 0;
static uint8_t            s_passkey_buf[6];
static bool               s_waiting_for_passkey = false;
static void buffer_timer_handler(void* p_context);
void buffer_init(void);
static uint8_t s_hid_interface_idx;
static struct
{
    uint8_t keyboard_rep_idx;
    uint8_t output_rep_idx;
    uint8_t feature_rep_idx;
} s_hid_reports;

#pragma push
#pragma O0
/**@brief Error handler function, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // Copying parameters to static variables because parameters are not accessible in debugger.
    static volatile uint8_t  s_file_name[128];
    static volatile uint16_t s_line_num;
    static volatile uint32_t s_error_code;

    strcpy((char *)s_file_name, (const char *)p_file_name);
    s_line_num   = line_num;
    s_error_code = error_code;
    UNUSED_VARIABLE(s_file_name);
    UNUSED_VARIABLE(s_line_num);
    UNUSED_VARIABLE(s_error_code);  

    debug_print(dbg_assert, 0 , 0); 
    
#ifndef DEBUG_NRF_USER    
    NVIC_SystemReset();
#endif /* DEBUG_NRF_USER */

    for (;;)
    {
        // Loop forever. On assert, the system can only recover on reset.
    }
}
#pragma pop
/**@brief Assert macro callback function.
 *
 * @details This function will be called if the ASSERT macro fails.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    app_error_handler(0, line_num, file_name);
}

/**@brief Module initialization.
 */
static void modules_init(void)
{
    m_prw_and_clk_mgmt_init_t pwr_and_clk_params;
    m_coms_init_t             m_coms_params;
    uint32_t                  err_code;
    
    // Battery measurement module
    err_code =  m_batt_meas_init(m_batt_meas_handler, 1, hal_adc_pin_ain4);
    APP_ERROR_CHECK(err_code);   
    
    // Communication module init
    m_coms_params.protocol = protocol_mode_ble;
    M_COMS_BLE_PARAMS_FILL(&m_coms_params.ble_params);
    m_coms_params.event_callback = m_coms_handler;
    m_coms_params.ble_params.boot_mode_callback = hid_boot_mode_handler;
    m_coms_params.ble_params.bond_params.delete_bonds = false;
    
    err_code = m_coms_init(&m_coms_params);
    APP_ERROR_CHECK(err_code);
    

    // Power management module
    pwr_and_clk_params.sysoff.callbacks[0] = m_keyboard_wakeup_prepare;
    pwr_and_clk_params.sysoff.callbacks[1] = m_coms_wakeup_prepare;
    pwr_and_clk_params.sysoff.callbacks[2] = m_batt_meas_wakeup_prepare;
    pwr_and_clk_params.sysoff.num          = 3;
    pwr_and_clk_params.idle.num            = 0;
    pwr_and_clk_params.sysoff_timeout      = DEFAULT_BTLE_INACTIVITY_DISCONNECT_PERIOD;
    pwr_and_clk_params.idle_timeout        = DEFAULT_IDLE_TIMEOT; 
    pwr_and_clk_params.lfclk_cal_interval  = (8000/250); // 8000ms LFCLK calibration interval. 250ms pr tick.    
    
    err_code = m_pwr_and_clk_mgmt_init(&pwr_and_clk_params, m_pwr_and_clk_mgmt_handler);
    APP_ERROR_CHECK(err_code);
}

static void modules_enable(void)
{
    uint32_t err_code;

    err_code = m_coms_ble_report_descriptor_add(s_keyboard_hid_descriptor, sizeof(s_keyboard_hid_descriptor), ble_boot_pkt_keyboard, &s_hid_interface_idx);
    APP_ERROR_CHECK(err_code);
    
    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_input, false, 0, 8, &s_hid_reports.keyboard_rep_idx);
    APP_ERROR_CHECK(err_code);
    
    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_output, false, 0, 1, &s_hid_reports.output_rep_idx);
    APP_ERROR_CHECK(err_code);
    
    err_code = m_coms_ble_report_id_map(s_hid_interface_idx, hid_report_type_feature, false, 0, 2, &s_hid_reports.feature_rep_idx);
    APP_ERROR_CHECK(err_code);
        // Keyboard module. 
    err_code = m_keyboard_init(keyboard_format_usbhid, m_keyboard_handler);    
    APP_ERROR_CHECK(err_code); 
    m_keyboard_matrix_enable();
		if(!pair_btn_pressed)
    {
        err_code = m_coms_enable();
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Packet buffering initialization
 */
static void buffer_init(void)
{
    uint32_t err_code;

    memset(&s_packet_buffer, 0, sizeof(s_packet_buffer));
    s_packet_buffer.start_idx = 0;
    s_packet_buffer.end_idx = 0;

    err_code =  app_timer_create(&s_packet_buffer_id,
                               APP_TIMER_MODE_REPEATED,
                               buffer_timer_handler);   
    APP_ERROR_CHECK(err_code);
}


/**@brief Timeout function used when packet buffer is not empty
 */
static void buffer_timer_handler(void* p_context)
 {    
     uint32_t err_code;
     uint8_t report[8] = {0};
     
     if (s_connection_state == state_disconnected)
     {
         // No point in trying to send if we are disconnected.
         return;
     }
     
     if (s_packet_buffer.end_idx == s_packet_buffer.start_idx)
     {
         // Buffer is empty
         if (s_buffer_timer_running)
         {
             err_code = app_timer_stop(s_packet_buffer_id);
             APP_ERROR_CHECK(err_code);
             s_buffer_timer_running = false;
         }
         
         return;
     }
     
    report[0] = s_packet_buffer.buffer[s_packet_buffer.start_idx].modifier_keys;
    
    for (uint32_t i = 0; i < s_packet_buffer.buffer[s_packet_buffer.start_idx].num_keys; i++)
    {
        report[i + 2] = s_packet_buffer.buffer[s_packet_buffer.start_idx].keys[i];
    }
    
    err_code = m_coms_hid_report_send(report, 
                                      sizeof(report), 
                                      s_hid_interface_idx,
                                      s_hid_reports.keyboard_rep_idx);     
     
     if (err_code == NRF_SUCCESS)
     {
         s_packet_buffer.start_idx = (s_packet_buffer.start_idx + 1) % PACKET_BUFFER_SIZE;
         
         if (s_packet_buffer.end_idx == s_packet_buffer.start_idx)
         {
             if (s_buffer_timer_running)
             {
                 err_code = app_timer_stop(s_packet_buffer_id);
                 APP_ERROR_CHECK(err_code);
                 s_buffer_timer_running = false;
             }
         }
     }
 }

/**@brief Utility function to buffer packets that fails to be sent
 */
static void buffer_keys(const m_keyboard_data_t* packet)
{
    if ((s_packet_buffer.end_idx + 1) % PACKET_BUFFER_SIZE == s_packet_buffer.start_idx) 
    {
        // Buffer is full. Won't overwrite.
        return;
    }
    else
    {
        uint32_t err_code;

        memcpy(&s_packet_buffer.buffer[s_packet_buffer.end_idx], packet, sizeof(m_keyboard_data_t));
        s_packet_buffer.end_idx = (s_packet_buffer.end_idx + 1) % PACKET_BUFFER_SIZE;
        
        if (s_buffer_timer_running == false)
        {
            // Start the timeout handler
            err_code = app_timer_start(s_packet_buffer_id, APP_TIMER_TICKS(15 /* [ms] */, APP_TIMER_PRESCALER), 0);
            APP_ERROR_CHECK(err_code);
            s_buffer_timer_running = true;
        }
    }
}


/**@brief Callback function used to re-assemble packets in HID boot mode.
 *
 * @details When in HID boot mode, a non-configurable packet format is used for mouse and keyboard reports.
 *          Applicable reports (in this case keyboard reports) needs to be re-formatted to fit the Boot format.
 * @note Only Boot keyboard and Boot mouse HID reports can be sent when in Boot mode.
 *
 * @param[in\out] p_packet      Packet which needs re-assembly
 * @param[out]    p_packet_type Type of packet. If type is @ref ble_boot_pkt_none the packet is discarded.
 */
static void hid_boot_mode_handler(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                  m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                  uint8_t *                    p_data, 
                                  uint8_t                      p_len, 
                                  uint8_t                      p_hid_interface, 
                                  uint8_t                      p_report_idx)
{
    if (p_hid_interface == s_hid_interface_idx && p_report_idx == s_hid_reports.keyboard_rep_idx)
    {
        // Keyboard report: boot packet and standard keyboard packet has the same format
        memcpy(p_boot_pkt->keyboard_data.keys, p_data, p_len);
        *p_pkt_type = ble_boot_pkt_keyboard;
    }
    else
    {
        // Unknown report
    }
}



/**@brief Handler function for keyboard module events
 */
static void m_keyboard_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t            err_code;
    m_keyboard_data_t * keyboard_pkt;
    
    ASSERT(event_size == sizeof(m_keyboard_data_t));
    
    keyboard_pkt = (m_keyboard_data_t *) p_event_data;
    
    if (keyboard_pkt->pairing_button)
    {
        uint8_t release_report[8] = {0};
        
        // Stop buffer timer and flush
        if (s_buffer_timer_running)
        {
            err_code = app_timer_stop(s_packet_buffer_id);
            APP_ERROR_CHECK(err_code);
            s_buffer_timer_running = false;
        }
        
        s_packet_buffer.start_idx = 0;
        s_packet_buffer.end_idx = 0;
        
        if(s_connection_state == state_connected)
        {
            // Send release packet
            m_coms_hid_report_send(release_report, 
                                   sizeof(release_report), 
                                   0,
                                   s_hid_reports.keyboard_rep_idx);     
       }                               
        // Start bonding
        m_coms_bonding_start();
    }
    else
    {
        if (s_waiting_for_passkey)
        {
            if (keyboard_pkt->num_keys > 0)
            {
                memcpy(&s_passkey_buf[s_passkey_idx], keyboard_pkt->keys, keyboard_pkt->num_keys);
                s_passkey_idx += keyboard_pkt->num_keys;
                
                if (s_passkey_idx >= 6)
                {
                    err_code = m_coms_ble_passkey_set(s_passkey_buf);
                    APP_ERROR_CHECK(err_code);
                    s_passkey_idx = 0;
                }
            }
        }
        else
        {
            buffer_keys(keyboard_pkt);
        }
    }
    // Notifying the power manager of activity
    m_pwr_mgmt_feed();
}

/**@brief Handler function for communication module events
 */
static void m_coms_handler(void * p_event_data, uint16_t event_size)
{
    m_coms_evt_t * evt;
    
    ASSERT(event_size == sizeof(m_coms_evt_t));
    
    evt = (m_coms_evt_t *) p_event_data;

    // Events coming from the communications module.
    switch (evt->type)
    {
        case com_event_init_finished:
            modules_enable();
            break;
        
        case com_event_connected:
            s_connection_state = state_connected;
        
            m_prw_mgmt_set_sysoff_timeout(DEFAULT_BTLE_INACTIVITY_DISCONNECT_PERIOD);
            break;

        case com_event_data_received:
            if(evt->data.data_received.report_type==BLE_HIDS_REP_TYPE_FEATURE)
            {
          
            }
            else if(evt->data.data_received.report_type==BLE_HIDS_REP_TYPE_OUTPUT)
            {
                //led etc.
            }
      

            break;

        case com_event_timing_update:
            break;

        case com_event_advertising_bondable:
            
            break;
        
        case com_event_address_changed:
            break;
        
        case com_event_passkey_req:
            m_keyboard_format_set(keyboard_format_ascii);
            s_waiting_for_passkey = true;
            break;
        
        case com_event_oobkey_req:
            break;        

        case com_event_key_sent:
            m_keyboard_format_set(keyboard_format_usbhid);
            s_waiting_for_passkey = false;            
            break;
        case com_event_disconnected:
            s_connection_state = state_disconnected;
//            if (m_keyboard_pairing_btn_pressed())
						if (pair_btn_pressed)
            {
                m_coms_bonding_start();
            }
            else
            {
                m_coms_disable();
            }
            break;
        case com_event_disabled:
         /* Fall through */
        case com_event_advertising_timeout:
            m_pwr_mgmt_goto_sysoff(0, 0);
            break;

        default:
         break;
    }
 }

/**@brief Handler function for battery module events
 */
static void m_batt_meas_handler(void * p_event_data, uint16_t event_size)
{
     uint32_t err_code;
     
     ASSERT(event_size == 1);
     
     err_code = m_coms_ble_battery_level_update(*(uint8_t*) p_event_data);
     if (err_code != NRF_SUCCESS)
     {
         // Don't care
     }
}

/**@brief Handler function for power and clock manager module events
 */
static void m_pwr_and_clk_mgmt_handler(void * p_event_data, uint16_t event_size)
{
    uint32_t            err_code;
    m_pwr_and_clk_mgmt_event_t * event;
    
    ASSERT(event_size == sizeof(m_pwr_and_clk_mgmt_event_t));
    
    event = (m_pwr_and_clk_mgmt_event_t *) p_event_data;
    
    if(event->idle_timeout)
    {
        err_code = app_sched_event_put(0,0, m_pwr_mgmt_goto_idle);
        APP_ERROR_CHECK(err_code);
    }
    else if (event->sysoff_timeout)
    {
        err_code = app_sched_event_put(0,0, m_pwr_mgmt_goto_sysoff);
        APP_ERROR_CHECK(err_code);
    }  
}

/**@brief Application main function.
 */
int main(void)
{   
    NRF_POWER->DCDCEN = (POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos); 
    
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);
    buffer_init();
    modules_init();
    while(1)
    {
        app_sched_execute();
        m_pwr_mgmt_run();
    }
}

