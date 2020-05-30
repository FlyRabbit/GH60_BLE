#include "m_coms_ble_adv.h"

#include <string.h>

#include "ble_advdata.h"
#include "m_coms_ble_addr.h"
#include "ble_hci.h"
#include "m_coms_ble.h"
#include "device_manager.h"
#include "pstorage.h"
#include "nrf_soc.h"

// Advertisement parameters
#define APP_ADV_INTERVAL_FAST            BLE_GAP_ADV_INTERVAL_MIN /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 20 ms.). */
#define APP_ADV_INTERVAL_SLOW            160 /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 100 ms). */
#define APP_ADV_WHITELIST_RECONN_TIMEOUT 5   /**< The advertising timeout in units of seconds used when advertising vs a resolvable address for reconnect. */

static m_coms_ble_bond_params_t * s_params;
static m_coms_ble_adv_state_t     s_state;
static app_sched_event_handler_t  s_evt_handler;
static dm_application_instance_t  s_dm_app_id; 
static bool *                     s_encrypted;
static bool                       s_wait_for_flash = false;

/**@brief Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 * @param[in]  adv_flags  Indicates which type of advertisement to use,
 *                             see @ref BLE_GAP_DISC_MODES.
 */
static uint32_t advertising_init(uint8_t adv_flags)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = adv_flags;
    ble_gap_addr_t ble_addr;

    ble_uuid_t adv_uuids[] = { { BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE } };

    err_code = sd_ble_gap_address_get(&ble_addr);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &ble_addr);
    APP_ERROR_CHECK(err_code);    
    
    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);

    return err_code;
}

/**@brief Generate and set new random BLE device address.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 * @retval NRF_ERROR_INVALID_STATE
 * @retval NRF_ERROR_SOFTDEVICE_NOT_ENABLED
 */
static uint32_t ble_address_update(void)
{
    uint32_t                  err_code;
    uint32_t                  addr_offset;
    ble_gap_addr_t            addr;
    
    addr_offset = 1;
    while (true)
    {
        err_code = m_coms_ble_addr_new_get(s_dm_app_id, &addr);
        if (err_code == NRF_ERROR_INVALID_STATE)
        {
            // Does not have any addresses stored in db: don't change address
            return NRF_SUCCESS;
        }
        else if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
        if (err_code == NRF_SUCCESS)
        {
            break;
        }
        
        ++addr_offset;
    }
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_adv_init(const m_coms_ble_bond_params_t * p_bond_params,
                             app_sched_event_handler_t        p_evt_handler,
                             bool *                           p_encryption_state,
                             dm_handle_t                      p_dm_handle)
{
    if (p_bond_params == 0 || p_evt_handler == 0 || p_encryption_state == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    s_params      = (m_coms_ble_bond_params_t *) p_bond_params;
    s_dm_app_id   = 0;
    s_evt_handler = p_evt_handler;
    s_encrypted   = p_encryption_state;
    
    s_state.type = ADV_STATE_NONE;
    memset(&s_state.params, 0, sizeof(s_state.params));
    
    return NRF_SUCCESS;
}
static void adv_start_handler(void * dat, uint16_t size)
{
    uint32_t err_code;
    
    err_code = m_coms_ble_adv_start();
    APP_ERROR_CHECK(err_code);    
}
void m_coms_ble_adv_on_evt(ble_evt_t * p_ble_evt)
{
    uint32_t 	err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            s_state.params.advertising_running = false;
            s_state.params.bond_initiate       = false;
            s_state.type                       = ADV_STATE_NONE;
            break;
        
        case BLE_GAP_EVT_DISCONNECTED:
            if (m_coms_ble_adv_determine(p_ble_evt))
            {
                // Disconnection was caused by desire to advertise for bond
                err_code = app_sched_event_put(0, 0, adv_start_handler);
                APP_ERROR_CHECK(err_code);
            }
            break;
        
        case BLE_GAP_EVT_TIMEOUT:
            s_state.params.advertising_running = false;
        
            if (m_coms_ble_adv_determine(p_ble_evt))
            {
                // Continue advertising
                err_code = m_coms_ble_adv_start();
                if(err_code != NRF_SUCCESS)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        
        default:
            break;
    }
}

void m_coms_ble_adv_on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (s_wait_for_flash)
            {
                uint32_t err_code;
                uint32_t flash_access_cnt;
                
                err_code = pstorage_access_status_get(&flash_access_cnt);
                APP_ERROR_CHECK(err_code);
                
                if (flash_access_cnt == 0)
                {
                    s_wait_for_flash = false;
                    
                    err_code = m_coms_ble_adv_start();
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
    }
}

bool m_coms_ble_adv_determine(ble_evt_t * p_ble_evt)
{
    dm_device_instance_t      master_ids[DEVICE_MANAGER_MAX_BONDS];
    uint8_t                   master_ids_len;
    uint32_t err_code;
    
    err_code       = m_coms_ble_addr_device_ids_get(s_dm_app_id, master_ids, &master_ids_len);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Differentiate between bonding behavior and normal behavor
    if (s_state.params.bond_initiate)
    {
        switch (s_state.type)
        {
            case ADV_STATE_NONE:
                if (s_params->bond_reconnect_all && master_ids_len > 0)
                {
                    // Try advertising vs. all bonded masters
                    // Use undirected with whitelist if directed advertising is not to be used
                    s_state.type = s_params->directed_adv ?  
                        ADV_STATE_DIRECTED_MULTIPLE : ADV_STATE_UNDIRECTED_WHITELIST;
                    s_state.params.directed_adv_count = 0;
                }
                else
                {
                    // Go straight to undirected bondable
                    s_state.type = ADV_STATE_UNDIRECTED_BONDABLE;
                }
                return true;
            
            case ADV_STATE_DIRECTED_SINGLE:
                // Not used when bonding
                break;
            
            case ADV_STATE_UNDIRECTED_WHITELIST:
                /* Fall through: Both these types behaves the same */
            case ADV_STATE_DIRECTED_MULTIPLE:
                if (p_ble_evt && 
                    p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
                {
                    if (s_state.params.directed_adv_count < master_ids_len)
                    {
                        // Continue same advertising type
                    }
                    else
                    {
                        // Reconnection attempt completed: start undirected bondable
                        s_state.type = ADV_STATE_UNDIRECTED_BONDABLE;
                    }
                    return true;
                }
                break;
            
            case ADV_STATE_UNDIRECTED_BONDABLE:
                if (p_ble_evt && p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
                {
                    // Bondable advertising timed out
                    s_state.type                 = ADV_STATE_NONE;
                    s_state.params.bond_initiate = false;
                    return false;
                }
                break;
            
            default:
                APP_ERROR_CHECK_BOOL(false);
                break;
        }
    }
    else
    {
        switch (s_state.type)
        {
            case ADV_STATE_NONE:
                if (p_ble_evt && 
                    p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED &&
                    !*s_encrypted)
                {
                    if(p_ble_evt->evt.gap_evt.params.disconnected.reason==BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
                    {                        
                    // Disconnection probably caused by failed passkey entry: 
                    // start bondable advertising again
                        s_state.type = ADV_STATE_UNDIRECTED_BONDABLE;
                        return true;
                    }
                    else
                    {
                        //default
                        return false;
                    }
                }
                else if (p_ble_evt && 
                         p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
                {
                    // Normal disconnect: don't start advertising again
                    return false;
                }
                else if (s_params->reconnect_all && master_ids_len > 0)
                {
                    // Try advertising vs. all bonded masters
                    // Use undirected with whitelist if directed advertising is not to be used
                    s_state.type = s_params->directed_adv ?  
                        ADV_STATE_DIRECTED_MULTIPLE : ADV_STATE_UNDIRECTED_WHITELIST;
                    s_state.params.directed_adv_count = 0;
                    return true;
                }
                
                if (master_ids_len == 0)
                {
                    // Have no bonds: look for one
                    s_state.type = ADV_STATE_UNDIRECTED_BONDABLE;
                }
                else
                {
                    // Have one or more bonded master. Target latest used one for reconnect
                    s_state.type = s_params->directed_adv ?  
                        ADV_STATE_DIRECTED_SINGLE : ADV_STATE_UNDIRECTED_WHITELIST;

                }
                return true;
            
            case ADV_STATE_DIRECTED_SINGLE:
                if (p_ble_evt && 
                    p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
                {
                    s_state.type = ADV_STATE_NONE;
                    // Return false, don't continue advertising 
                    return false;
                }
                break;
            
            case ADV_STATE_UNDIRECTED_WHITELIST:
                /* Fall through: Both these types behaves the same */
            case ADV_STATE_DIRECTED_MULTIPLE:
                if (p_ble_evt && 
                    p_ble_evt->header.evt_id == BLE_GAP_EVT_TIMEOUT)
                {
                    if (s_state.params.directed_adv_count < master_ids_len)
                    {
                        // Continue same type
                        return true;
                    }
                    else
                    {
                        // Reconnect vs all bonded masters attempted
                        s_state.type = ADV_STATE_NONE;
                        return false;
                    }
                }
                break;
            
            case ADV_STATE_UNDIRECTED_BONDABLE:
                s_state.type = ADV_STATE_NONE;
                return false;
            
            default:
                APP_ERROR_CHECK_BOOL(false);
                break;
        }
    }
    
    return false;
}

uint32_t m_coms_ble_adv_start(void)
{
    uint32_t                  err_code;
    ble_gap_adv_params_t      adv_params;
    ble_gap_addr_t            peer_address;
    dm_device_instance_t      master_ids[DEVICE_MANAGER_MAX_BONDS];
    uint8_t                   master_ids_len;
    int8_t                    addr_idx       = -1;
    bool                      whitelist_get  = false;
    bool                      address_update = false;
    uint32_t                  count;
    ble_gap_whitelist_t       whitelist;
    ble_gap_addr_t            *p_whitelist_addr[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    ble_gap_irk_t             *p_whitelist_irk[BLE_GAP_WHITELIST_IRK_MAX_COUNT];    
    

    
    if (s_state.params.advertising_running)
    {
        // Advertising already running
        return NRF_SUCCESS;
    }

    err_code = pstorage_access_status_get(&count);
    APP_ERROR_CHECK(err_code);
    
    if (count != 0)
    {
        s_wait_for_flash = true;
        return NRF_SUCCESS;
    }        
    
    memset(&adv_params, 0, sizeof(adv_params));
    memset(&whitelist, 0, sizeof(whitelist));
    memset(&peer_address, 0, sizeof(peer_address));
    
    // Getting number of bonded masters
    err_code       = m_coms_ble_addr_device_ids_get(s_dm_app_id, master_ids, &master_ids_len);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    switch (s_state.type)
    {
        case ADV_STATE_NONE:
            APP_ERROR_CHECK_BOOL(false);
            break;
        
        case ADV_STATE_DIRECTED_SINGLE:
            addr_idx                = master_ids_len - 1;
            s_state.params.adv_type = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
            break;
        
        case ADV_STATE_DIRECTED_MULTIPLE:
            // Decide which bonded master to target
            if (s_state.params.bond_initiate)
            {
                // Least used one first
                addr_idx = s_state.params.directed_adv_count;
            }
            else
            {
                // Latest used one first
                addr_idx = master_ids_len - s_state.params.directed_adv_count - 1;
            }
            s_state.params.adv_type = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
            ++s_state.params.directed_adv_count;
            break;
        
        case ADV_STATE_UNDIRECTED_WHITELIST:
            // Decide which bonded master to "target"
            if (s_state.params.bond_initiate)
            {
                // Least used one first
                addr_idx = s_state.params.directed_adv_count;
            }
            else
            {
                // Latest used one first
                addr_idx = master_ids_len - s_state.params.directed_adv_count - 1;
            }
            s_state.params.adv_type = BLE_GAP_ADV_TYPE_ADV_IND;
            whitelist_get           = true;
            ++s_state.params.directed_adv_count;
            break;
        
        case ADV_STATE_UNDIRECTED_BONDABLE:
            s_state.params.adv_type = BLE_GAP_ADV_TYPE_ADV_IND;
            address_update          = s_params->change_address;
            break;
        
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
    
    if (addr_idx != -1 && addr_idx <= master_ids_len)
    {
        ble_gap_addr_t addr;
        dm_handle_t    dm_handle;
        
        dm_handle_initialize(&dm_handle);
        dm_handle.device_id = 0;
        dm_handle.appl_id = s_dm_app_id;
        
        err_code = m_coms_ble_addr_addr_get(s_dm_app_id, addr_idx, &addr, &dm_handle.device_id);
        if (err_code != NRF_SUCCESS)
        {
            // addr_idx not found, try first idx first and increment.
            for (uint32_t i = 0; i < master_ids_len; i++)
            {
                addr_idx = i;
                err_code = m_coms_ble_addr_addr_get(s_dm_app_id, addr_idx, &addr, &dm_handle.device_id);
                if (err_code == NRF_SUCCESS)
                {
                    break;
                }
            }
            
            if(err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
        
        // Set local address
        err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }        
        
        // Get peer address
        err_code = dm_peer_addr_get(&dm_handle, &peer_address);
        if (err_code != NRF_SUCCESS)
        {
            // Either flash storage is corrupted, or this master has a resolvable address
            // To figure out if address is resolvable: see if we have a corresponding IRK (Identity Resolving Key),
            // and then use undirected advertising with whitelist instead of directed advertising.
            s_state.params.adv_type = BLE_GAP_ADV_TYPE_ADV_IND;
            whitelist_get           = true;
        }
        
        //debug_print(dbg_device_addr, addr.addr, 6);
        //debug_print(dbg_device_addr, (uint8_t *)&addr_idx, 1);
        //debug_print(dbg_peer_addr, peer_address.addr, 6);
    }
    
    if (whitelist_get)
    {
        whitelist.addr_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
        whitelist.irk_count  = BLE_GAP_WHITELIST_IRK_MAX_COUNT;
        whitelist.pp_addrs   = p_whitelist_addr;
        whitelist.pp_irks    = p_whitelist_irk;
        
        err_code = dm_whitelist_create(&s_dm_app_id, &whitelist);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        //debug_print(dbg_whitelist_addr_count, &whitelist.addr_count, 1);
        for (uint8_t i = 0; i < whitelist.addr_count; i++)
        {
            //debug_print(dbg_whitelist_addr, whitelist.pp_addrs[i]->addr, BLE_GAP_ADDR_LEN);
        }
        
        //debug_print(dbg_whitelist_irk_count, &whitelist.irk_count, 1);
        for (uint8_t i = 0; i < whitelist.irk_count; i++)
        {
            //debug_print(dbg_whitelist_irk, whitelist.pp_irks[i]->irk, BLE_GAP_SEC_KEY_LEN);
        }        
    }
    
    if (address_update)
    {
        m_coms_ble_evt_t evt;

        // Set a new address. This prevents previously connected masters from reconnecting
        err_code = ble_address_update();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        // Notify application that address has changed
        evt.type = M_COMS_BLE_EVT_ADDR_CHANGED;
        err_code = app_sched_event_put(&evt, sizeof(evt), s_evt_handler);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    
    // Configure advertising parameters
    switch (s_state.params.adv_type)
    {
        case BLE_GAP_ADV_TYPE_ADV_IND:
            if (!whitelist_get)
            {
                // Check if we need to make room for the new bond.
                if (master_ids_len >= DEVICE_MANAGER_MAX_BONDS)
                {
                    dm_handle_t    dm_handle;
                    
                    dm_handle_initialize(&dm_handle);
                    dm_handle.appl_id = s_dm_app_id;
                    
                    err_code = m_coms_ble_addr_delete(s_dm_app_id, 0, &dm_handle.device_id);
                    if (err_code != NRF_SUCCESS)
                    {
                        return err_code;
                    }                
                    
                    err_code = dm_device_delete(&dm_handle);
                    if (err_code != NRF_SUCCESS)
                    {
                        return err_code;
                    }                   
                }                
                // Don't have a whitelist set up: This is generic "I want to bond" advertising
                APP_ERROR_CHECK_BOOL(whitelist.addr_count == 0 && whitelist.irk_count == 0);
                
                // Bondable
                s_state.params.adv_flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
                // Anyone can connect
                adv_params.fp               = BLE_GAP_ADV_FP_ANY;
                // Don't need fast advertising, because masters wanting to connect will do foreground scanning (intense scanning)
                adv_params.interval         = APP_ADV_INTERVAL_SLOW; 
                // Timeout should always be 180s when advertising as limited discoverable (according to Core spec)
                adv_params.timeout          = BLE_GAP_ADV_TIMEOUT_LIMITED_MAX;
            }
            else
            {
                // Whitelist is configured: This is advertising targeted towards masters without fixed addresses
                
                // Not bondable
                s_state.params.adv_flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
                // Only devices on whitelist can connect
                adv_params.fp            = BLE_GAP_ADV_FP_FILTER_CONNREQ;
                //adv_params.fp               = BLE_GAP_ADV_FP_ANY;
                // Advertising with shortest possible interval for a few seconds
                adv_params.interval      = APP_ADV_INTERVAL_FAST;
                adv_params.timeout       = APP_ADV_WHITELIST_RECONN_TIMEOUT; 
                adv_params.p_whitelist   = &whitelist;
            }
            // Undirected advertising
            adv_params.type = BLE_GAP_ADV_TYPE_ADV_IND;
            break;
        
        case BLE_GAP_ADV_TYPE_ADV_DIRECT_IND:
            // Undirected advertising
            adv_params.type          = BLE_GAP_ADV_TYPE_ADV_DIRECT_IND;
            // Not bondable
            s_state.params.adv_flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
            adv_params.p_peer_addr   = &peer_address;
            // Directed advertising is always fixed at 1.28 seconds
            adv_params.timeout       = 0;
            break;
        
        case BLE_GAP_ADV_TYPE_ADV_SCAN_IND:
            /* Fall through */ 
        case BLE_GAP_ADV_TYPE_ADV_NONCONN_IND:
            /* Fall through */ 
        default:
            // Not dealing with these advertising types
            return NRF_ERROR_INTERNAL;
    }
    

    err_code = advertising_init(s_state.params.adv_flags);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = sd_ble_gap_adv_start(&adv_params);
    if (err_code == NRF_SUCCESS)
    {
        s_state.params.advertising_running = true;
        
        if (s_state.type == ADV_STATE_UNDIRECTED_BONDABLE)
        {
            m_coms_ble_evt_t evt;
            
            // Notify m_coms
            evt.type = M_COMS_BLE_EVT_ADV_BONDABLE;
            err_code = app_sched_event_put(&evt, sizeof(evt), s_evt_handler);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
        }
    }
    
    return err_code;
}

uint32_t m_coms_ble_adv_bond_adv_start(uint16_t p_conn_handle)
{
    uint32_t err_code;
    
    // New bonds are accepted when advertising is started again
    s_state.params.bond_initiate      = true;
    s_state.params.directed_adv_count = 0;
    
    if (p_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect. Disconnected event will trigger advertising
        err_code = sd_ble_gap_disconnect(p_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code == NRF_ERROR_INVALID_STATE)
        {
            // Not connected, or have already started disconnection procedure
            err_code = NRF_SUCCESS;
        }
    }
    else
    {
        // Not connected; can start advertising right away
        err_code                               = sd_ble_gap_adv_stop();
        // Ignoring potential NRF_ERROR_INVALID_STATE error.
        
       s_state.params.advertising_running = false;
       s_state.type                       = ADV_STATE_NONE;
        
        if (!m_coms_ble_adv_determine(0))
        {
            // advertising_determine() should always return true in this case
            return NRF_ERROR_INTERNAL;
        }
        err_code = m_coms_ble_adv_start();
        APP_ERROR_CHECK(err_code);
    }
    
    return err_code;
}

bool m_coms_ble_adv_running(void)
{
    return s_state.params.advertising_running;
}

bool m_coms_ble_adv_bondable_running(void)
{
    return (s_state.params.bond_initiate && s_state.params.advertising_running);
}

bool m_coms_ble_adv_bond_stored(void)    
{
    dm_device_instance_t      master_ids[DEVICE_MANAGER_MAX_BONDS];
    uint8_t                   master_ids_len;
    uint32_t err_code;
    
    err_code       = m_coms_ble_addr_device_ids_get(s_dm_app_id, master_ids, &master_ids_len);
    APP_ERROR_CHECK(err_code);
    return (master_ids > 0);
}
