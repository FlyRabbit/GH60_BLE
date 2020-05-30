#include "m_coms_ble.h"
#include "nordic_common.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_bas.h"
#include "device_manager.h"
#include "ble_conn_params.h"
#include "ble_dis.h"
#include "ble_flash.h"
#include "ble_hci.h"
#include "ble_hids.h"
#include "ble_srv_common.h"
#include "dfu_app_handler.h"
#include "common_params.h"
#include "m_coms_ble_addr.h"
#include "m_coms_ble_adv.h"
#include "m_coms_ble_hid.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "pstorage.h"
#include "nrf_delay.h"
#include "bootloader_settings.h"
#include "crc16.h"
#include "dfu_types.h"

#ifdef RTT
    #include "SEGGER_RTT.h"
#else
    #define SEGGER_RTT_printf(...)
    #define SEGGER_RTT_WriteString(...)
#endif
    
#define IS_SRVC_CHANGED_CHARACT_PRESENT  1   /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

/**@brief Only 0's can be written to flash. The target flash area must be erased (set to 1's) to ensure valid data can be written to it once */
#define QUARTER_PAGE_FLASH_ERASE_VALUES \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \
    0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF, \

//
// These defines are considered very unlikely to be changed between HID applications
//
#define MTU_SIZE BLE_L2CAP_MTU_DEF

// Connection parameter handling
#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)     /**< Time from initiating event (connect or start of notification) to first time ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)    /**< Time between each call to ble_gap_conn_param_update after the first (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                                  /**< Number of attempts before giving up the connection parameter negotiation. */

// Security parameters
#define SEC_PARAM_BOND                   1   /**< Perform bonding (always true for HID applications). */
#define SEC_PARAM_TIMEOUT                30  /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_MIN_KEY_SIZE           7   /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16  /**< Maximum encryption key size. */
#define ENC_REQ_DELAY                   500/**< Time[ms] to wait when connected to a bonded master before requesting encryption */

// Multiple reports send buffer size
#define MULTIPLE_REPORTS_BUFFER_SIZE 259

// Flash databases: These needs to be initialized to the flash erased state (0xFF) so they can be written once without erasing first
static const uint32_t s_ble_params_db[] = {QUARTER_PAGE_FLASH_ERASE_VALUES}; // 1/4 flash page
static const uint32_t s_hid_params_db[] = {QUARTER_PAGE_FLASH_ERASE_VALUES QUARTER_PAGE_FLASH_ERASE_VALUES 
                                           QUARTER_PAGE_FLASH_ERASE_VALUES QUARTER_PAGE_FLASH_ERASE_VALUES}; // 1 flash page

typedef struct
{
    uint8_t  data[MULTIPLE_REPORTS_BUFFER_SIZE];
    uint32_t idx;
    bool     valid;
} mult_rep_buf_t;
                                           
// Static variables
static m_coms_ble_params_t *         s_ble_params     = 0; /** BLE parameters */
static app_sched_event_handler_t     s_event_callback = 0; /** Event callback used to notify m_coms */
static app_timer_id_t                s_encryption_timer;   /** Timer used to request encryption if Central doesnt enable it in a timely manner */
static app_timer_id_t                s_flash_access_timer; /** Timer used while waiting for flash access. */
static app_timer_id_t                s_notify_usage_timer;  /** Timer used to delay m_coms_ble_addr_notify_usage function */
static ble_gap_sec_params_t          s_sec_params;         /** Security requirements for this application. */
static ble_bas_t                     s_bas;                /** Structure used to identify the battery service. */
static ble_dfu_t                     s_dfu;                /** Structure used to identify the dfu service. */
static uint8_t                       s_num_inp_reports;
static uint8_t                       s_hid_notif_enabled_count = 0;
static int8_t                        s_tx_buf_cnt;         /** Number of packets in SoftDevice TX buffer. Used to limit amount of buffered packets */
static bool                          s_encrypted;          /** Link encryption status */
static bool                          s_sec_params_requested; /** Auth key requested */
static bool                          s_boot_mode_active;   /** In Boot or Report mode? */
static ble_srv_report_ref_t          s_bas_report_ref;     /** Battery service report reference mapping to HID service */
static uint16_t                      s_conn_handle;        /** Handle value of current connection */
static m_coms_ble_boot_mode_callback s_boot_mode_callback; /** Callback used to handle Boot mode reports */

static dm_application_instance_t     m_app_instance_id;         /**< Application identifier allocated by device manager. */
static dm_handle_t                   m_bonded_peer_handle;      /**< Device reference handle to the current bonded central. */

#ifdef USE_MULTIPLE_HID_REPS
    static mult_rep_buf_t                s_mult_rep_buf[2];
    static mult_rep_buf_t*               s_mult_rep_buf_in_use = 0;
    static bool*                         s_mult_rep_status;
    static uint8_t                       s_mult_rep_interface_idx;
    static uint8_t                       s_mult_rep_report_idx;
#endif /* USE_MULTIPLE_HID_REPS*/

#define WAITING_REASON_INIT                     (1UL << 0)
#define WAITING_REASON_INIT_BONDS_DELETE        (1UL << 1)
#define WAITING_REASON_DISABLE                  (1UL << 2)
#define WAITING_REASON_APP_CONTEXT_WRITE        (1UL << 3)
#define WAITING_REASON_SERVICE_CONTEXT_WRITE    (1UL << 4)
#define WAITING_REASON_APP_CONTEXT_NOTIFY       (1UL << 5)
#define WAITING_REASON_ADVERTISING_START        (1UL << 6)

static bool s_waiting_for_flash      = false;
static bool s_waiting_for_disconnect = false;
static bool s_wait_to_notify_ble_addr = false;
static uint32_t s_waiting_reason     = 0;

static void ble_stack_disable(void * p_evt, uint16_t evt_size);

#ifdef USE_MULTIPLE_HID_REPS
static __inline void multiple_hid_report_send(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    uint8_t  packet_size;
    
#if MULTIPLE_REPORTS_BUFFER_SIZE != 259
#error MULTIPLE_REPORTS_BUFFER_SIZE are hardcoded to support 259-byte frames
#endif /* AUDIO_FRAME_SIZE != 259 */
    
    while (true)
    {
        if (s_mult_rep_buf_in_use->idx < 240)
        {
            packet_size = 20;
        }
        else
        {
            packet_size = 19;
        }
        
        err_code = m_coms_ble_hid_input_report_send(s_mult_rep_interface_idx, 
                                                    s_mult_rep_report_idx, 
                                                    &(s_mult_rep_buf_in_use->data[s_mult_rep_buf_in_use->idx]),
                                                    packet_size);
        
        if (err_code != NRF_SUCCESS)
        {
            return;
        }
        else if (packet_size == 19)
        {
            int i;
            
            // Packet size 16 is the last packet in a frame
            s_mult_rep_buf_in_use->valid = false; // No valid data left in this buffer
            s_mult_rep_buf_in_use        = 0;     // Zero pointer to indicate no frames are sent right now
            *s_mult_rep_status           = true;

            for (i = 0; i < (sizeof(s_mult_rep_buf) / sizeof(s_mult_rep_buf[0])); ++i)
            {
                if (s_mult_rep_buf[i].valid)
                {
                    s_mult_rep_buf_in_use = &s_mult_rep_buf[i];
                    break;
                }
            }
            
            if (s_mult_rep_buf_in_use == 0)
            {
                // No more valid buffers to send
                return;
            }
        }
        else
        {
            // Still more packets in the frame to send
            s_mult_rep_buf_in_use->idx += 20;
        }
    }
}
#endif /* USE_MULTIPLE_HID_REPS */

static __inline uint32_t service_context_set(void)
{
    uint32_t flash_access_cnt = 0;
    uint32_t err_code;
    
    err_code = pstorage_access_status_get(&flash_access_cnt);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    //debug_print(dbg_service_context_set, (uint8_t *)&flash_access_cnt, 1)

    if (flash_access_cnt != 0)
    {
        s_waiting_for_flash = true;
        s_waiting_reason |= WAITING_REASON_SERVICE_CONTEXT_WRITE;
    }
    else
    {
        dm_service_context_t   service_context;
        service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
        service_context.context_data.len = 0;
        service_context.context_data.p_data = NULL;
        
        // The notification of boot keyboard input report has been enabled.
        // Save the system attribute (CCCD) information into the flash.                
        err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            // The system attributes could not be written to the flash because
            // the connected central is not a new central. The system attributes
            // will only be written to flash only when disconnected from this central.
            // Do nothing now.
        }                    
    }
    
    return err_code;
}

/**@brief Flash access wait timeout handler.
 *
 * @param[in] p_context
 */
static void on_flash_access_timeout(void *p_context)
{
    SEGGER_RTT_printf(0, "on_flash_access_timeout\r\n");
    
    if (!s_waiting_for_flash)
    {
        // Flash access waited for finished
        return;
    }    
    
    NVIC_SystemReset();
}

static void on_addr_notify_usage_timeout(void *p_context)
{
    SEGGER_RTT_printf(0, "on_addr_notify_usage_timeout\r\n");
    
    if ((s_wait_to_notify_ble_addr) && 
        (s_encrypted) &&
        (s_conn_handle != BLE_CONN_HANDLE_INVALID) &&
        (m_bonded_peer_handle.device_id < DEVICE_MANAGER_MAX_BONDS))
    {
        uint32_t flash_access_cnt;
        uint32_t err_code;
        
        s_wait_to_notify_ble_addr = false;
        
        err_code = pstorage_access_status_get(&flash_access_cnt);
        APP_ERROR_CHECK(err_code);            
        
        if (flash_access_cnt == 0)
        {
            ble_gap_addr_t ble_addr;
            err_code = sd_ble_gap_address_get(&ble_addr);
            APP_ERROR_CHECK(err_code);
            err_code = m_coms_ble_addr_notify_usage(&m_bonded_peer_handle, &ble_addr);
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            s_waiting_for_flash = true;
            s_waiting_reason |= WAITING_REASON_APP_CONTEXT_NOTIFY;
        }
        
        return;
    }
    else
    {
        SEGGER_RTT_printf(0, "on_addr_notify_usage_timeout - NOTIFY FAILED\r\n");
        //APP_ERROR_CHECK_BOOL(false);
    }
}

/**@brief Application's BLE Stack event handler.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t         err_code = NRF_SUCCESS;
    m_coms_ble_evt_t coms_evt;
    
   
    SEGGER_RTT_printf(0, "on_ble_evt - %d\r\n", p_ble_evt->header.evt_id);
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            s_conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
            s_encrypted   = false;
            s_hid_notif_enabled_count = 0;
            
            SEGGER_RTT_printf(0, "BLE_GAP_EVT_CONNECTED - %02X%02X%02X%02X%02X%02X\r\n", p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[5],
                                                                                         p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[4],
                                                                                         p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[3],
                                                                                         p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[2],
                                                                                         p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[1],
                                                                                         p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr[0]);
            
    
    SEGGER_RTT_printf(0, "connected conn params - %d - %d - %d \r\n", p_ble_evt->evt.gap_evt.params.connected.conn_params.min_conn_interval,p_ble_evt->evt.gap_evt.params.connected.conn_params.max_conn_interval, p_ble_evt->evt.gap_evt.params.connected.conn_params.slave_latency);
   
            // Notify m_coms of event
            coms_evt.type = M_COMS_BLE_EVT_CONNECTED;
            s_event_callback(&coms_evt, sizeof(coms_evt));
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            s_conn_handle        = BLE_CONN_HANDLE_INVALID;
            s_sec_params_requested = false;
        
            if (s_waiting_for_disconnect)
            {
                err_code = m_coms_ble_disable();
                APP_ERROR_CHECK(err_code);
            }

            if (m_coms_ble_adv_running())
            {
                // Advertising has started again: don't do anything
            }
            else
            {
                coms_evt.type = M_COMS_BLE_EVT_DISCONNECTED;
                // Disconnect was not caused by bonding procedure: Notify m_coms of event
                s_event_callback(&coms_evt, sizeof(coms_evt));
            }
            break;
            
        case BLE_GAP_EVT_TIMEOUT:
            if (!m_coms_ble_adv_running())
            {
                coms_evt.type = M_COMS_BLE_EVT_ADV_TIMEOUT;
                // Notify m_coms of event
                s_event_callback(&coms_evt, sizeof(coms_evt));
            }
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = app_timer_stop(s_encryption_timer);  
            s_sec_params_requested = true;
            break;
        case BLE_GAP_EVT_SEC_INFO_REQUEST:   
            err_code = app_timer_stop(s_encryption_timer);  
            break;

        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            switch (p_ble_evt->evt.gap_evt.params.auth_key_request.key_type)
            {
                case BLE_GAP_AUTH_KEY_TYPE_PASSKEY:
                    // Notify m_coms of event
                    coms_evt.type = M_COMS_BLE_EVT_PASSKEY_REQ;
                    s_event_callback(&coms_evt, sizeof(coms_evt));
                    break;
                
                case BLE_GAP_AUTH_KEY_TYPE_OOB:
                    // Notify m_coms of event
                    coms_evt.type = M_COMS_BLE_EVT_OOBKEY_REQ;
                    s_event_callback(&coms_evt, sizeof(coms_evt));
                    break;
                
                case BLE_GAP_AUTH_KEY_TYPE_NONE:
                    break;
                
                default:
                    break;
            }
            
            // Don't want encryption timer running during passkey entry
            err_code = app_timer_stop(s_encryption_timer);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GAP_EVT_AUTH_STATUS:
            // Notifying m_coms of event
            coms_evt.type = M_COMS_BLE_EVT_KEY_SENT;
            s_event_callback(&coms_evt, sizeof(coms_evt));
            break;
        
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            coms_evt.type = M_COMS_BLE_EVT_CONN_UPDATE;
            coms_evt.data.conn_update.min_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
            coms_evt.data.conn_update.max_conn_interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
            coms_evt.data.conn_update.slave_latency = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency;
            coms_evt.data.conn_update.supervision_timeout = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout;
          
        
            SEGGER_RTT_printf(0, "conn params update- %d - %d - %d \r\n", p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval, p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency);
   
            s_event_callback(&coms_evt, sizeof(coms_evt));
            break;        
        
        default:
            break;
    }
}

/**@brief Encrypting timeout handler.
 *
 * @param[in] p_context
 */
static void on_encrypt_timeout(void* p_context)
{
    uint32_t err_code;
    
    if (s_encrypted)
    {
        // Already encrypted
        return;
    }
    
    // Request link to be authenticated
    err_code = sd_ble_gap_authenticate(s_conn_handle, &s_sec_params);                
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_handle     Device manager handle.
 * @param[in]   p_event      Data associated to the device manager event.
 * @param[in]   event_result Event result.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t           event_result)
{
    uint32_t err_code;
    
    SEGGER_RTT_printf(0, "device_manager_evt_handler - %d\r\n", p_event->event_id);
    
    if ((event_result != NRF_SUCCESS) && (p_event->event_id == DM_EVT_SERVICE_CONTEXT_STORED))
    {
        err_code = service_context_set();
        APP_ERROR_CHECK(err_code);
    }
    else if (p_event->event_id == DM_EVT_SECURITY_SETUP_COMPLETE) 
    {
        if((event_result != BLE_GAP_SEC_STATUS_CONFIRM_VALUE) && (event_result != BLE_GAP_SEC_STATUS_PASSKEY_ENTRY_FAILED))
        {
             APP_ERROR_CHECK(event_result);
        }
        
    }
    else if (p_event->event_id == DM_EVT_APPL_CONTEXT_STORED) //catching unhandeled pstorage timeout on app context update.       
    {
        if(event_result == NRF_ERROR_TIMEOUT)
        {    
            SEGGER_RTT_WriteString(0, "Pstorage timeout exposed to m_coms.\r\n");
        }
        APP_ERROR_CHECK(event_result);
    }
    else
    {
         APP_ERROR_CHECK(event_result);
    }
    
    switch(p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_STORED:
        {
            uint32_t flash_access_cnt;
            
            err_code = pstorage_access_status_get(&flash_access_cnt);
            APP_ERROR_CHECK(err_code);            
            
            if (flash_access_cnt == 0)
            {
                ble_gap_addr_t ble_addr;
                err_code = sd_ble_gap_address_get(&ble_addr);
                APP_ERROR_CHECK(err_code);                
                err_code = m_coms_ble_addr_set(&m_bonded_peer_handle, &ble_addr);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                s_waiting_for_flash = true;
                s_waiting_reason |= WAITING_REASON_APP_CONTEXT_WRITE;
            }
        }
        break;
        
        case DM_EVT_CONNECTION:
            if(p_handle->device_id != DM_INVALID_ID)
            {
                err_code = app_timer_start(s_encryption_timer, APP_TIMER_TICKS(ENC_REQ_DELAY, APP_TIMER_PRESCALER), 0);
                APP_ERROR_CHECK(err_code);
            }
        break;
        
        case DM_EVT_DEVICE_CONTEXT_LOADED:
        case DM_EVT_SECURITY_SETUP_COMPLETE:
            m_bonded_peer_handle = (*p_handle);
//            SEGGER_RTT_printf(0, "Device ID: %d\r\n", m_bonded_peer_handle.device_id);
  //          SEGGER_RTT_printf(0, "Connec ID: %d\r\n", m_bonded_peer_handle.connection_id);           
            
            break;
        
        case DM_EVT_LINK_SECURED:
        {   
            s_encrypted = true;
            
            if (!s_sec_params_requested)
            {
                ble_gap_conn_params_t conn_params;
                
                // Check connection parameters
                conn_params.conn_sup_timeout  = s_ble_params->conn_params.conn_sup_timeout;
                conn_params.max_conn_interval = s_ble_params->conn_params.max_conn_interval;
                conn_params.min_conn_interval = s_ble_params->conn_params.min_conn_interval;
                conn_params.slave_latency     = s_ble_params->conn_params.slave_latency;
                
                err_code = ble_conn_params_change_conn_params(&conn_params);
                if(err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            
            if ((m_bonded_peer_handle.device_id < DEVICE_MANAGER_MAX_BONDS))
            {
                s_wait_to_notify_ble_addr = true;
                on_addr_notify_usage_timeout(NULL);
            }
        }
        break;
        
        case DM_EVT_APPL_CONTEXT_LOADED:
        {

        }
        break;
				
        case DM_EVT_APPL_CONTEXT_STORED:
        {
        
       
        }
        break;
      			
				
        default:
            break;
    }

    return NRF_SUCCESS;
}

/**@brief Function for handling the HID events.
 *
 * @param[in]   p_evt     HID event structure.
 */
static void ble_hids_evt_handler(m_coms_ble_hid_evt_t * p_evt)
{
    uint32_t         err_code;
    m_coms_ble_evt_t evt;
    
    SEGGER_RTT_printf(0, "ble_hids_evt_handler - %d\r\n", p_evt->hids_evt->evt_type); 
    
    switch (p_evt->hids_evt->evt_type)
    {
        case BLE_HIDS_EVT_HOST_SUSP:
            break;
        
        case BLE_HIDS_EVT_HOST_EXIT_SUSP:
            break;
        
        case BLE_HIDS_EVT_NOTIF_ENABLED:
            s_hid_notif_enabled_count++;
            //debug_print(dbg_BLE_HIDS_EVT_NOTIF_ENABLED, (uint8_t *)&s_hid_notif_enabled_count, 1);
            if (s_hid_notif_enabled_count >= s_num_inp_reports)
            {   
                ble_gap_conn_params_t conn_params;
                
                // All input reports enabled: update connection parameters
                conn_params.conn_sup_timeout  = s_ble_params->conn_params.conn_sup_timeout;
                conn_params.max_conn_interval = s_ble_params->conn_params.max_conn_interval;
                conn_params.min_conn_interval = s_ble_params->conn_params.min_conn_interval;
                conn_params.slave_latency     = s_ble_params->conn_params.slave_latency;
                
                err_code = ble_conn_params_change_conn_params(&conn_params);
                if(err_code != NRF_ERROR_BUSY)
                {
                    APP_ERROR_CHECK(err_code);
                }                
                
                err_code = service_context_set();
                APP_ERROR_CHECK(err_code);
            }
            break;
        
        case BLE_HIDS_EVT_NOTIF_DISABLED:
            break;
        
        case BLE_HIDS_EVT_REP_CHAR_WRITE:
            APP_ERROR_CHECK_BOOL(p_evt->data != 0);
        
            evt.type                             = M_COMS_BLE_EVT_DATA_RECEIVED;
            evt.data.data_received.interface_idx = p_evt->interface_idx;
            evt.data.data_received.report_type   = p_evt->report_type;
            evt.data.data_received.report_idx    = p_evt->report_idx;
            evt.data.data_received.len           = p_evt->len;
            evt.data.data_received.data          = p_evt->data;
            s_event_callback(&evt, sizeof(evt));
            break;
        
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            s_boot_mode_active = true;
            break;
        
        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            s_boot_mode_active = false;
            break;
        
        case BLE_HIDS_EVT_REPORT_READ:
            evt.type                        = M_COMS_BLE_EVT_READ_REQ;
            evt.data.read_req.interface_idx = p_evt->interface_idx;
            evt.data.read_req.report_idx    = p_evt->report_idx;
            s_event_callback(&evt, sizeof(evt));
            break;
        
        default:
            // Unknown event
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

/**@brief Function for handling the HID errors.
 *
 * @param[in]   nrf_error Error code.
 */
static void ble_hids_error_handler(uint32_t nrf_error)
{
    if (nrf_error != NRF_ERROR_INVALID_PARAM)
    {
        APP_ERROR_CHECK(nrf_error);
    }
    else
    {
        //
    }
}

/**@brief Function for handling the DFU errors.
 *
 * @param[in]   nrf_error Error code.
 */
static void ble_dfu_error_handler(uint32_t nrf_error)
{
    APP_ERROR_CHECK(nrf_error);
}

/**@brief Function for handling the Application's system events.
 *
 * @param[in]   sys_evt   system event.
 */
static void on_sys_evt(uint32_t sys_evt)
{
    SEGGER_RTT_printf(0, "on_sys_evt - %d\r\n", sys_evt);    
    
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_ERROR:
             SEGGER_RTT_WriteString(0, "NRF_EVT_FLASH_OPERATION_ERROR\r\n");
            //APP_ERROR_CHECK_BOOL(false);
            break;
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            SEGGER_RTT_WriteString(0, "NRF_EVT_FLASH_OPERATION_SUCCESS\r\n");
            if (s_waiting_for_flash)
            {
                uint32_t err_code;
                uint32_t flash_access_cnt = 0;
                
                err_code = pstorage_access_status_get(&flash_access_cnt);
                APP_ERROR_CHECK(err_code);
                //debug_print(dbg_on_sys_evt_acces_cnt, (uint8_t *)&flash_access_cnt, 1);
                if (flash_access_cnt <= 0)
                {
                    
                    if ((s_waiting_reason & WAITING_REASON_INIT) == WAITING_REASON_INIT)
                    {
                        m_coms_ble_evt_t coms_evt;

                        s_waiting_reason &= ~(WAITING_REASON_INIT);
                        
                        err_code = app_timer_stop(s_flash_access_timer);
                        APP_ERROR_CHECK(err_code);
                        
                        coms_evt.type = M_COMS_BLE_EVT_INIT_FINISHED;
                        err_code = app_sched_event_put(&coms_evt, sizeof(coms_evt), ble_stack_disable);
                        APP_ERROR_CHECK(err_code);
                    }
                    
                                       
                    
                    if ((s_waiting_reason & WAITING_REASON_APP_CONTEXT_WRITE) == WAITING_REASON_APP_CONTEXT_WRITE)
                    {
                        s_waiting_reason &= ~(WAITING_REASON_APP_CONTEXT_WRITE);
                        
                        ble_gap_addr_t ble_addr;
                        err_code = sd_ble_gap_address_get(&ble_addr);
                        APP_ERROR_CHECK(err_code);                           
                        err_code = m_coms_ble_addr_set(&m_bonded_peer_handle, &ble_addr);
                        APP_ERROR_CHECK(err_code);
                    }
                    else
                    if ((s_waiting_reason & WAITING_REASON_APP_CONTEXT_NOTIFY) == WAITING_REASON_APP_CONTEXT_NOTIFY)
                    {
                        s_waiting_reason &= ~(WAITING_REASON_APP_CONTEXT_NOTIFY);
                        
                        ble_gap_addr_t ble_addr;
                        err_code = sd_ble_gap_address_get(&ble_addr);
                        APP_ERROR_CHECK(err_code);
                        err_code = m_coms_ble_addr_notify_usage(&m_bonded_peer_handle, &ble_addr);
                        APP_ERROR_CHECK(err_code);
                    }                    
                    else
                    if ((s_waiting_reason & WAITING_REASON_SERVICE_CONTEXT_WRITE) == WAITING_REASON_SERVICE_CONTEXT_WRITE)
                    {
                        dm_service_context_t   service_context;
                        
                        s_waiting_reason &= ~(WAITING_REASON_SERVICE_CONTEXT_WRITE);
                        
                        service_context.service_type = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
                        service_context.context_data.len = 0;
                        service_context.context_data.p_data = NULL;
                        
                        // The notification of boot keyboard input report has been enabled.
                        // Save the system attribute (CCCD) information into the flash.                
                        err_code = dm_service_context_set(&m_bonded_peer_handle, &service_context);
                        if (err_code != NRF_ERROR_INVALID_STATE)
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                        else
                        {
                            // The system attributes could not be written to the flash because
                            // the connected central is not a new central. The system attributes
                            // will only be written to flash only when disconnected from this central.
                            // Do nothing now.
                        }
                    }
                    else
                    if ((s_waiting_reason & WAITING_REASON_DISABLE) == WAITING_REASON_DISABLE)
                    {
                        m_coms_ble_evt_t coms_evt;

                        s_waiting_reason &= ~(WAITING_REASON_DISABLE);                        
                        
                        coms_evt.type = M_COMS_BLE_EVT_DISABLED;
                        m_coms_ble_disable();
                        err_code = app_sched_event_put(&coms_evt, sizeof(coms_evt), ble_stack_disable);
                        APP_ERROR_CHECK(err_code);
                    } 
                    
                    if (s_waiting_reason == 0)
                    {
                        s_waiting_for_flash = false;
                    }
                    
                }
            }
            
            break;
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Connection Parameters module error handler.
 *
 * @param[in] nrf_error Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_CHECK_BOOL(false);
}

/**@brief Dispatches a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received or from interrupt context depending on the event.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(void *p_evt_data, uint16_t evt_size)
{   
    APP_ERROR_CHECK_BOOL(evt_size == sizeof(ble_evt_t));
    
    ble_evt_t *p_ble_evt = (ble_evt_t *)p_evt_data;
    
    if (p_ble_evt->header.evt_id == BLE_EVT_TX_COMPLETE)
    {
        #ifdef USE_MULTIPLE_HID_REPS
            // If multiple report buffer is being sent: Stuff available TX buffers with audio packets
            if (s_mult_rep_buf_in_use != 0)
            {
                multiple_hid_report_send(p_ble_evt);
            }
        #endif /* USE_MULTIPLE_HID_REPS */
            
        // Updating TX buffer count variable
        s_tx_buf_cnt -= p_ble_evt->evt.common_evt.params.tx_complete.count;
        if (s_tx_buf_cnt < 0)
        {
            // This can happen whenever there are L2CAP packet transmissions
            s_tx_buf_cnt = 0;
        }
        
        return;
    }
    
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    m_coms_ble_hid_on_evt(p_ble_evt);
    m_coms_ble_adv_on_evt(p_ble_evt);
    ble_bas_on_ble_evt(&s_bas, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_dfu_on_ble_evt(&s_dfu, p_ble_evt);
}

/**@brief Manage both interrupt and sheduled BLE stack events.
 *
 * @details This function is called in interrupt context when a new BLE Stack event has occured.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_manager(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    if ( (p_ble_evt->header.evt_id == BLE_EVT_TX_COMPLETE) &&
         (s_ble_params->tx_complete_callback != NULL))
    {
            // Call BLE_EVT_TX_COMPLETE callback if it's configured.
            s_ble_params->tx_complete_callback(p_ble_evt);
    }
    
    // Dispatch event through scheduler.
    err_code = app_sched_event_put(p_ble_evt, sizeof(ble_evt_t), ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(void *p_sys_evt, uint16_t size)
{
    APP_ERROR_CHECK_BOOL(size == sizeof(uint32_t));
    uint32_t sys_evt = *(uint32_t *)p_sys_evt;
    
    pstorage_sys_event_handler(sys_evt);
    on_sys_evt(sys_evt);
    m_coms_ble_adv_on_sys_evt(sys_evt);
}

/**@brief Manage both interrupt and sheduled BLE stack events.
 *
 * @details This function is called in interrupt context when a new system event has occured.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_manager(uint32_t sys_evt)
{
    uint32_t err_code;
    
    // Dispatch event through scheduler.
    err_code = app_sched_event_put(&sys_evt, sizeof(uint32_t), sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Device Manager initialization.
 */
static uint32_t device_manager_init(bool p_bonds_delete)
{
    uint32_t                 err_code;
    dm_init_param_t          init_data;
    dm_application_param_t   register_param;

    // Initialize peer device handle.
    err_code = dm_handle_initialize(&m_bonded_peer_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Clear all bonded centrals if the "delete all bonds" button is pushed.
    init_data.clear_persistent_data = p_bonds_delete;
    err_code = dm_init(&init_data);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    //register_param.sec_param.timeout      = s_sec_params.timeout; TODO: Find out where this timeout should be set.
    register_param.sec_param.bond         = s_sec_params.bond;
    register_param.sec_param.mitm         = s_sec_params.mitm;
    register_param.sec_param.io_caps      = s_sec_params.io_caps;
    register_param.sec_param.oob          = s_sec_params.oob;
    register_param.sec_param.min_key_size = s_sec_params.min_key_size;
    register_param.sec_param.max_key_size = s_sec_params.max_key_size;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;
    
    return dm_register(&m_app_instance_id, &register_param);
}

/**@brief BLE stack initialization.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static uint32_t ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_TEMP_8000MS_CALIBRATION, NULL);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;

    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_manager);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Register with the SoftDevice handler module for BLE events.
    return softdevice_sys_evt_handler_set(sys_evt_manager);
}

/**@brief BLE stack disable.
 *
 * @details Disables the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_disable(void * p_evt, uint16_t evt_size)
{
    uint32_t         err_code;
    
    err_code = softdevice_handler_sd_disable();
    APP_ERROR_CHECK(err_code);
    
    if (evt_size == sizeof(m_coms_ble_evt_t))
    {
        s_event_callback(p_evt, evt_size);
    }
}

/**@brief GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile) 
 *          parameters of the device. It also sets the permissions and appearance.
 */
static uint32_t gap_params_init(const m_coms_ble_params_t * p_params)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, 
                                          (const uint8_t *) p_params->device_info.device_name, 
                                          strlen(p_params->device_info.device_name));
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = sd_ble_gap_appearance_set(p_params->appearance);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    memcpy(&gap_conn_params, &p_params->conn_params, sizeof(p_params->conn_params));

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

/**@brief Initialize the Connection Parameters module.
 */
static uint32_t conn_params_init(const m_coms_ble_params_t * p_params)
{
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  conn_params;
    
    memset(&cp_init, 0, sizeof(cp_init));
    memcpy(&conn_params, &p_params->conn_params, sizeof(p_params->conn_params));

    cp_init.p_conn_params                  = &conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;
    
    // Don't know the CCCD handle yet. Connection parameter negotiation is triggered once _all_ HID Service CCCDs are set
    cp_init.start_on_notify_cccd_handle = 0xFFFF;
    
    return ble_conn_params_init(&cp_init);
}

/**@brief Initialize security parameters.
 */
static uint32_t sec_params_init(const m_coms_ble_params_t * p_params)
{
    // Checking parameters
    switch (p_params->sec_params.io_capabilities)
    {
        case BLE_GAP_IO_CAPS_DISPLAY_ONLY:
        case BLE_GAP_IO_CAPS_DISPLAY_YESNO:
        case BLE_GAP_IO_CAPS_KEYBOARD_ONLY:
        case BLE_GAP_IO_CAPS_NONE:
        case BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY:
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    s_sec_params.io_caps = p_params->sec_params.io_capabilities;
    
    // Sufficient IO capabilities to do MITM (man-in-the-middle) bonding?
    if (s_sec_params.io_caps == BLE_GAP_IO_CAPS_NONE)
    {
        s_sec_params.mitm = 0;
    }
    else
    {
        s_sec_params.mitm = 1;
    }
    
    s_sec_params.bond         = SEC_PARAM_BOND;
    s_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    s_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    //s_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    s_sec_params.oob          = p_params->sec_params.oob_data_availible ? 1 : 0; 
    
    return NRF_SUCCESS;
}

/**@brief Initialize Device Information Service.
 */
static uint32_t dis_init(const m_coms_ble_params_t * p_params)
{
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;
    
    pnp_id.vendor_id_source = p_params->device_info.pnp_vendor_id_src;
    pnp_id.vendor_id        = p_params->device_info.pnp_vendor_id;
    pnp_id.product_id       = p_params->device_info.pnp_product_id;
    pnp_id.product_version  = p_params->device_info.pnp_product_version;
    
    memset(&dis_init_obj, 0, sizeof(dis_init_obj));
    
    // Converting null-terminated strings to structs
    if (p_params->device_info.manufacturer_name != 0)
    {
        ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, (char *) p_params->device_info.manufacturer_name);
    }
    if (p_params->device_info.hw_revision != 0)
    {
        ble_srv_ascii_to_utf8(&dis_init_obj.hw_rev_str, (char *) p_params->device_info.hw_revision);
    }
    if (p_params->device_info.fw_revision != 0)
    {
        ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, (char *) p_params->device_info.fw_revision);
    }
    if (p_params->device_info.serial_number != 0)
    {
        ble_srv_ascii_to_utf8(&dis_init_obj.serial_num_str, (char *) p_params->device_info.serial_number);
    }
    
    dis_init_obj.p_pnp_id = &pnp_id;
    
    // Device Information Service can be read without encryption. 
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init_obj.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

    return ble_dis_init(&dis_init_obj);
}

/**@brief Initialize Battery Service.
 */
static uint32_t bas_init(const m_coms_ble_params_t * p_params)
{
    ble_bas_init_t bas_init_obj;
    
    memset(&bas_init_obj, 0, sizeof(bas_init_obj));
    
    bas_init_obj.evt_handler          = 0;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = s_bas_report_ref.report_type == 0 ? 0 : &s_bas_report_ref;
    bas_init_obj.initial_batt_level   = 100;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init_obj.battery_level_report_read_perm);
    
    return ble_bas_init(&s_bas, &bas_init_obj);
}

/**@brief Initialize DFU Service.
 */
static uint32_t dfu_init(void)
{
    ble_dfu_init_t dfu_init_params;
    
    dfu_init_params.evt_handler = dfu_app_on_dfu_evt;
    dfu_init_params.error_handler = ble_dfu_error_handler;
    
    return ble_dfu_init(&s_dfu, &dfu_init_params);
}

/**@brief Update DFU application crc.
 */
uint32_t dfu_update_crc(void)
{
    uint32_t err_code = NRF_SUCCESS;
    const bootloader_settings_t * p_bootloader_settings = (const bootloader_settings_t *)BOOTLOADER_SETTINGS_ADDRESS;
    const uint32_t *p_boot_start_address = (const uint32_t*)NRF_UICR_BOOT_START_ADDRESS;
    
    if (*p_boot_start_address == 0xFFFFFFFF)
    {
        // Bootloader not present no need to update crc
        return NRF_SUCCESS;
    }
    

    
    if ((p_bootloader_settings->bank_0 == BANK_VALID_APP) && (p_bootloader_settings->bank_0_crc != 0))
    {
        bootloader_settings_t new_settings;
        
        memcpy(&new_settings, p_bootloader_settings, sizeof(bootloader_settings_t));
        
        new_settings.bank_0_crc = crc16_compute((uint8_t *)DFU_BANK_0_REGION_START,
                                                 p_bootloader_settings->bank_0_size,
                                                 NULL);
        if (new_settings.bank_0_crc != p_bootloader_settings->bank_0_crc)
        {
            uint8_t flash_page = (uint8_t)((uint32_t)p_bootloader_settings / BLE_FLASH_PAGE_SIZE);
            
            err_code = ble_flash_page_erase(flash_page);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            err_code = ble_flash_block_write((uint32_t *)p_bootloader_settings, (uint32_t *)&new_settings, sizeof(bootloader_settings_t));
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }            
        }
    }
    
    return err_code;
}

uint32_t m_coms_ble_init(const app_sched_event_handler_t p_event_callback, const m_coms_ble_params_t * p_ble_params)
{
    uint32_t                err_code;
    m_coms_ble_hid_init_t   hid_params;
    uint32_t                flash_access_cnt = 0;
    
    if (p_event_callback == 0 || p_event_callback == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (sizeof(m_coms_ble_params_t) > sizeof(s_ble_params_db))
    {
        // m_coms_ble_params_t has grown beyond flash storage size
        return NRF_ERROR_NO_MEM;
    }
    
    s_ble_params = (m_coms_ble_params_t *) s_ble_params_db;
     
    if (memcmp(s_ble_params, p_ble_params, sizeof(m_coms_ble_params_t)) != 0)
    {
        // If this is the first time the program has run, the parameters are written to flash. 
        // This is to avoid putting parameter persistance requirements on the above application layers.
        // Assert triggers if the database exceeds flash memory size
        APP_ERROR_CHECK_BOOL((((uint32_t) s_ble_params_db) + CEIL_DIV(sizeof(m_coms_ble_params_t),4)) <= (NRF_FICR->CODEPAGESIZE * BLE_FLASH_PAGE_END));
        ble_flash_block_write((uint32_t *)s_ble_params_db, (uint32_t *)p_ble_params, CEIL_DIV(sizeof(m_coms_ble_params_t),4));
    }
    
    // Initializing static variables
    s_event_callback     = p_event_callback != 0 ? p_event_callback : s_event_callback;
    s_boot_mode_callback = s_ble_params->boot_mode_callback;
    s_encrypted          = false;
    s_sec_params_requested = false;
    s_boot_mode_active   = false;
    s_conn_handle        = BLE_CONN_HANDLE_INVALID;
    s_tx_buf_cnt         = 0;
    s_num_inp_reports    = 0;
    memset(&s_sec_params, 0, sizeof(s_sec_params));
    memset(&s_bas, 0, sizeof(s_bas));
    memset(&s_bas_report_ref, 0, sizeof(s_bas_report_ref));
    
    s_waiting_for_flash      = false;
    s_waiting_for_disconnect = false;
    s_waiting_reason     = 0;
    
    err_code = app_timer_create(&s_encryption_timer, APP_TIMER_MODE_SINGLE_SHOT, on_encrypt_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // HID module initialization
    hid_params.base_hid_version = s_ble_params->base_hid_version;
    hid_params.b_country_code   = s_ble_params->hid_country_code;
    hid_params.flags            = s_ble_params->hids_flags;
    hid_params.io_capabilities  = s_ble_params->sec_params.io_capabilities;
    hid_params.db_loc           = (uint32_t *) s_hid_params_db;
    hid_params.db_size          = sizeof(s_hid_params_db);
    hid_params.evt_handler      = ble_hids_evt_handler;
    hid_params.error_handler    = ble_hids_error_handler;
    
    err_code = m_coms_ble_hid_init(&hid_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
    
    // SoftDevice and event handling initialization
    err_code = ble_stack_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }      
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = sec_params_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
    
    // Device manager initialization
    err_code = device_manager_init(p_ble_params->bond_params.delete_bonds);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = m_coms_ble_addr_init(&m_app_instance_id);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = app_timer_create(&s_flash_access_timer, APP_TIMER_MODE_SINGLE_SHOT, on_flash_access_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = app_timer_create(&s_notify_usage_timer, APP_TIMER_MODE_SINGLE_SHOT, on_addr_notify_usage_timeout);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = pstorage_access_status_get(&flash_access_cnt);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    SEGGER_RTT_printf(0, "flash_access_cnt: %d\r\n", flash_access_cnt);

    if (flash_access_cnt > 0)
    {
        // Wait for flash access to finish before sending init finished event.
        s_waiting_for_flash = true;
        s_waiting_reason |= WAITING_REASON_INIT;
    }
    else
    {
        m_coms_ble_evt_t coms_evt;
        
        coms_evt.type = M_COMS_BLE_EVT_INIT_FINISHED;
        err_code = app_sched_event_put(&coms_evt, sizeof(coms_evt), ble_stack_disable);
        APP_ERROR_CHECK(err_code);        
    }
    
    return err_code;
}

uint32_t m_coms_ble_passkey_set(uint8_t * p_key)
{    
    return sd_ble_gap_auth_key_reply(s_conn_handle, BLE_GAP_AUTH_KEY_TYPE_PASSKEY, p_key);
}

uint32_t m_coms_ble_oobkey_set(uint8_t * p_key)
{
    return sd_ble_gap_auth_key_reply(s_conn_handle, BLE_GAP_AUTH_KEY_TYPE_OOB, p_key); 
}

uint32_t m_coms_ble_addr_get(ble_gap_addr_t * p_ble_addr)
{
    return sd_ble_gap_address_get(p_ble_addr);
}

uint32_t m_coms_ble_hid_report_send(uint8_t * p_data,
                                    uint8_t   p_len,
                                    uint8_t   p_hid_interface,
                                    uint8_t   p_report_idx)
{
    uint32_t err_code;
    
    if (!s_encrypted)
    {
        // HoG Profile requires encryption
        return NRF_ERROR_INVALID_STATE;
    }
    if (s_ble_params->max_tx_buf_cnt != 0 && s_tx_buf_cnt >= s_ble_params->max_tx_buf_cnt)
    {
        // Wont stuff more into TX buffer 
        return BLE_ERROR_NO_TX_BUFFERS;
    }
    
    if (s_boot_mode_active)
    {
        static m_coms_hid_boot_pkt_t boot_report;
        m_coms_ble_hid_boot_type_t   boot_report_type = ble_boot_pkt_none;
        
        // If boot mode is active: use boot callback to allow main application to restructure the report to fit the boot format
        s_boot_mode_callback(&boot_report, &boot_report_type, p_data, p_len, p_hid_interface, p_report_idx);
        
        err_code = NRF_ERROR_INVALID_STATE;

        if (boot_report_type & ble_boot_pkt_keyboard)
        {
            err_code = m_coms_ble_hid_keyboard_boot_report_send(&boot_report);
        }
        if (boot_report_type & ble_boot_pkt_mouse)
        {
            err_code = m_coms_ble_hid_mouse_boot_report_send(&boot_report);
        }
    }
    else
    {
        err_code = m_coms_ble_hid_input_report_send(p_hid_interface, p_report_idx, p_data, p_len);
    }
    
    if (err_code == NRF_SUCCESS)
    {
        // Incrementing TX buffer count variable
        ++s_tx_buf_cnt;
    }
    
    return err_code;
}

#ifdef USE_MULTIPLE_HID_REPS
uint32_t m_coms_ble_multiple_hid_reports_send(const uint8_t * p_data,
                                              uint16_t  p_len,
                                              uint8_t   p_hid_interface,
                                              uint8_t   p_report_idx,
                                              bool*     p_status)
{
    uint32_t err_code     = NRF_SUCCESS;
    bool     free_buffers = false;
    int      free_buf_nb  = 0;
    int      free_buf_idx = 0;
    int      i;
    
    if (p_len != MULTIPLE_REPORTS_BUFFER_SIZE)
    {
        // Don't support other frame sizes now.
        return NRF_ERROR_INVALID_PARAM;
    }

    for (i = 0; i < (sizeof(s_mult_rep_buf) / sizeof(s_mult_rep_buf[0])); ++i)
    {
        if (!s_mult_rep_buf[i].valid && !free_buffers)
        {
            // This buffer does not have valid data, which means it's not in use.
            free_buffers = true;
            free_buf_idx = i;
        }
        else if (!s_mult_rep_buf[i].valid)
        {
            ++free_buf_nb;
        }
    }
    for (i = 0; i < (sizeof(s_mult_rep_buf) / sizeof(s_mult_rep_buf[0])); ++i)
    {
        if (!s_mult_rep_buf[i].valid)
        {
            ++free_buf_nb;
        }
    }
    
    if (!free_buffers)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    s_mult_rep_status        = p_status;
    if (free_buf_nb == 0)
    {
        // No additional buffers left for the application
        *p_status = false;
    }
    else
    {
        // One or more frame buffers left for the application to use
        *p_status = true;
    }
    
    // Filling frame buffer
    memcpy(s_mult_rep_buf[free_buf_idx].data, p_data, p_len);
    s_mult_rep_buf[free_buf_idx].idx       = 0;
    s_mult_rep_buf[free_buf_idx].valid     = true;
    s_mult_rep_interface_idx               = p_hid_interface;
    s_mult_rep_report_idx                  = p_report_idx;
    
    if (s_mult_rep_buf_in_use != 0)
    {
        // Another buffer is still being sent: don't try stuffing TX buffers yet
        return NRF_SUCCESS;
    }
    else
    {
        s_mult_rep_buf_in_use = &s_mult_rep_buf[free_buf_idx];
    }
    
    // Stuffing TX buffers full to get started
    while (err_code == NRF_SUCCESS)
    {
        err_code = m_coms_ble_hid_input_report_send(p_hid_interface, 
                                                    p_report_idx, 
                                                    &(s_mult_rep_buf_in_use->data[s_mult_rep_buf_in_use->idx]),
                                                    20);
        if (err_code == NRF_SUCCESS)
        {
            s_mult_rep_buf_in_use->idx += 20;
        }
        else
        {
            break;
        }
    }
    
    return NRF_SUCCESS;
}
#endif

uint32_t m_coms_ble_read_respond(uint8_t * p_data, uint8_t p_len)
{
    ble_gatts_rw_authorize_reply_params_t reply_params;
    
    if (p_data == 0 || p_len == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    reply_params.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;
    reply_params.params.read.len         = p_len;
    reply_params.params.read.offset      = 0;
    reply_params.params.read.p_data      = p_data;
    reply_params.params.read.update      = 1;
    reply_params.type                    = BLE_GATTS_AUTHORIZE_TYPE_READ;

    return sd_ble_gatts_rw_authorize_reply(s_conn_handle, &reply_params);
}
 
uint32_t m_coms_ble_battery_level_update(uint8_t p_batt_level)
{
    uint32_t err_code;
    
    err_code = ble_bas_battery_level_update(&s_bas, p_batt_level);
    
    if (err_code == NRF_SUCCESS)
    {
        s_tx_buf_cnt += 1;
    }
    
    return err_code;
}

uint32_t m_coms_ble_enable(bool p_advertise)
{
    uint32_t err_code;
    uint32_t num_cccds;
    
    // Check if CCCD define has been set correctly
    num_cccds  = 0;
    num_cccds += 1; // Battery level characteristic
    num_cccds += 1; // Service changed characteristic
    num_cccds += 1; // DFU characteristic
    num_cccds += m_coms_ble_hid_num_cccds_get(); // HID Reports
    
    if (num_cccds > DM_GATT_CCCD_COUNT)
    {
        // @ref BLE_BONDMNGR_CCCD_COUNT must be increased
        return NRF_ERROR_NO_MEM;
    }
    
    // Calculate and rewrite application checksum
    err_code = dfu_update_crc();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // SoftDevice and event handling initialization
    err_code = ble_stack_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
//    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_MODE_AUTOMATIC);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
    
    // GAP parameters initialization
    err_code = gap_params_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Device Information Service initialization
    err_code = dis_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Battery Service initialization
    err_code = bas_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // DFU Service initialization
    err_code = dfu_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Advertising initialization
    err_code = m_coms_ble_adv_init(&s_ble_params->bond_params, 
                                    s_event_callback, 
                                    &s_encrypted, 
                                    m_bonded_peer_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Connection parameter handling initialization
    err_code = conn_params_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Security params initialization
    // Note: The sec params are updated if a OOB key is sent from main at a later time
    err_code = sec_params_init(s_ble_params);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Enable HID module
    err_code = m_coms_ble_hid_enable();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    
    if (p_advertise)
    {    
        if (!m_coms_ble_adv_determine(0))
        {
            // m_coms_ble_adv_determine() should always return true in this case 
            return NRF_ERROR_INTERNAL;
        }    
        
        err_code = m_coms_ble_adv_start();
        APP_ERROR_CHECK(err_code);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
 
    return err_code;
}

uint32_t m_coms_ble_bonding_start(void)
{    
    
    if (s_ble_params == 0)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    //debug_print(dbg_ble_bonding_start, 0, 0);
    
    return m_coms_ble_adv_bond_adv_start(s_conn_handle);
}

uint32_t m_coms_ble_report_descriptor_add(const uint8_t * p_descriptor,
                                          uint16_t        p_descriptor_len,
                                          uint8_t         p_boot_type_bitmask,
                                          uint8_t *       p_interface_idx)
{
    uint32_t err_code;
    
    err_code =  m_coms_ble_hid_report_descriptor_add(p_descriptor,
                                                     p_descriptor_len,
                                                     p_boot_type_bitmask,
                                                     p_interface_idx);
    
    return err_code;
}

uint32_t m_coms_ble_report_id_map(uint8_t                  p_interface_idx, 
                                  m_coms_hid_report_type_t p_report_type, 
                                  bool                     p_read_resp,
                                  uint8_t                  p_report_id, 
                                  uint8_t                  p_report_len,
                                  uint8_t *                p_report_idx)
{
    uint32_t err_code;
    
    if (p_report_type == hid_report_type_input)
    {
        // Counting these so that we can check connection parameters once CCCDs has been set
        ++s_num_inp_reports;
    }
    
    err_code =  m_coms_ble_hid_report_id_map(p_interface_idx, 
                                             p_report_type,
                                             p_read_resp,
                                             p_report_id,
                                             p_report_len,
                                             p_report_idx);
    
    return err_code;    
}

uint32_t m_coms_ble_report_id_map_external(uint8_t                  p_interface_idx, 
                                           m_coms_hid_report_type_t p_report_type, 
                                           uint8_t                  p_report_id, 
                                           uint16_t                 p_external_char_uuid, 
                                           uint16_t                 p_external_service_uuid)
{
    uint32_t err_code;
    
    if (p_external_char_uuid    != BLE_UUID_BATTERY_LEVEL_CHAR || 
        p_external_service_uuid != BLE_UUID_BATTERY_SERVICE)
    {
        return NRF_ERROR_NOT_SUPPORTED;
    }
    
    s_bas_report_ref.report_id   = p_report_id;
    s_bas_report_ref.report_type = p_report_type; 
    
    err_code = m_coms_ble_hid_report_id_map_external(p_interface_idx, p_external_char_uuid);
    
    return err_code;        
}

uint32_t m_coms_ble_disconnect(void)
{
    uint32_t err_code;
    
    // Connected?
    if (s_conn_handle != BLE_CONN_HANDLE_INVALID)
    {     
        // Disconnect
        err_code = sd_ble_gap_disconnect(s_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        s_conn_handle = BLE_CONN_HANDLE_INVALID;
    }
    else
    {
        // We are disconnected, let application know
        m_coms_ble_evt_t coms_evt;
        coms_evt.type = M_COMS_BLE_EVT_DISCONNECTED;
        err_code = app_sched_event_put(&coms_evt, sizeof(coms_evt), s_event_callback);
        APP_ERROR_CHECK(err_code);        
    }
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_disable(void)
{
    uint32_t         err_code;
    uint32_t         flash_access_cnt;
    m_coms_ble_evt_t coms_evt;
    
    err_code = app_timer_stop(s_flash_access_timer);
    err_code = pstorage_access_status_get(&flash_access_cnt);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    if (flash_access_cnt != 0)
    {
        s_waiting_for_flash = true;
        s_waiting_reason |= WAITING_REASON_DISABLE;
        
        return NRF_SUCCESS;
    }    
    
    // Connected?
    if (s_conn_handle != BLE_CONN_HANDLE_INVALID)
    {     
        // Disconnect
        err_code = sd_ble_gap_disconnect(s_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        s_conn_handle = BLE_CONN_HANDLE_INVALID;
        
        s_waiting_for_disconnect = true;
        
        return NRF_SUCCESS;
    }
    
    // Stop any ongoing connection parameter updates
    err_code = ble_conn_params_stop();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Stop advertising.
    // sd_ble_gap_adv_stop will return "NRF_ERROR_INVALID_STATE" if we're not currently advertising.
    // Ignoring this error.
    err_code = sd_ble_gap_adv_stop();
    
    err_code = pstorage_access_status_get(&flash_access_cnt);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    if (flash_access_cnt != 0)
    {
        s_waiting_for_flash = true;
        s_waiting_reason |= WAITING_REASON_DISABLE;
        
        return NRF_SUCCESS;
    }
    
    coms_evt.type = M_COMS_BLE_EVT_DISABLED; 
    
    return app_sched_event_put(&coms_evt, sizeof(coms_evt), ble_stack_disable);
}

bool m_coms_ble_wakeup_prepare(bool wakeup)
{
    bool ready_for_sysoff = true;
    
    //debug_print(dbg_coms_ble_wakeup_prepare, 0, 0);
    
    if (s_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ready_for_sysoff = false;
        m_coms_ble_disconnect();
    }
    
    return ready_for_sysoff;
}

bool m_coms_ble_bond_stored(void)
{
    return m_coms_ble_adv_bond_stored();
}

uint32_t m_coms_ble_bond_clear(void)
{
    return dm_device_delete_all(&m_app_instance_id);
}
