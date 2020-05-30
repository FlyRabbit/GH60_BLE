#include "m_coms_ble_addr.h"

#include "m_coms_ble.h"
#include <string.h>

#include "ble.h"
#include "ble_advdata.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"

#ifdef ENABLE_DEBUG
#include "SEGGER_RTT.h"
#define DEBUG_PRINTF(x) DEBUG_PRINTF(x)
#else
#define DEBUG_PRINTF(...)
#endif

/**@brief Macro for verifying NULL parameters are not passed to API.
 *
 * @param[in] PARAM Parameter checked for NULL.
 *
 * @retval (NRF_ERROR_NULL) when @ref PARAM is NULL.
 */
#define NULL_PARAM_CHECK(PARAM)                            \
    do                                                     \
    {                                                      \
        if ((PARAM) == NULL)                               \
        {                                                  \
            return NRF_ERROR_NULL;                         \
        }                                                  \
    } while (0)

/**@brief Macro for verifying if device instance is allocated.
 *
 * @param[in] X Device instance identifier.
 *
 * @retval (NRF_ERROR_INVALID_PARAM) when device instance is not allocated.
 */
#define VERIFY_DEVICE_ID(X)                                            \
    do                                                                 \
    {                                                                  \
        if ((X) >= DEVICE_MANAGER_MAX_BONDS)                           \
        {                                                              \
            return NRF_ERROR_INVALID_PARAM;                            \
        }                                                              \
    } while (0)

typedef enum
{
    m_coms_ble_addr_state_idle,
    m_coms_ble_addr_state_notify,
    m_coms_ble_addr_state_set,
    m_coms_ble_addr_state_get,
    m_coms_ble_addr_state_dev_ids_get,
    m_coms_ble_addr_state_delete
}m_coms_ble_addr_state_t;

typedef struct
{
    bool                          valid;
    dm_application_context_t      ac;
    dm_application_context_data_t ac_data;      
}app_context_ram_layout_t;

typedef struct
{
    m_coms_ble_addr_state_t       state;
    app_context_ram_layout_t      app_context[DEVICE_MANAGER_MAX_BONDS];
    
}m_coms_ble_addr_context_t;

static m_coms_ble_addr_context_t s_context;

static uint32_t usage_idx_get(dm_handle_t *dm_handle, uint32_t *usage_idx)
{
    uint32_t err_code = NRF_ERROR_NOT_FOUND;
    
    if (s_context.app_context[dm_handle->device_id].valid)
    {
        *usage_idx = s_context.app_context[dm_handle->device_id].ac_data.data.usage_idx;
        err_code = NRF_SUCCESS;
    }
    
    return err_code;
}

static uint32_t addr_set(dm_handle_t *p_dm_handle, uint32_t usage_idx, ble_gap_addr_t *p_ble_addr)
{
    uint32_t       err_code;
    
    if (memcmp(&s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr, p_ble_addr, sizeof(ble_gap_addr_t)))
    {
        memcpy(&s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr, p_ble_addr, sizeof(ble_gap_addr_t));
    }

    DEBUG_PRINTF(0, "addr_set - %02X%02X%02X%02X%02X%02X\r\n",      s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[5],
                                                                         s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[4],
                                                                         s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[3],
                                                                         s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[2],
                                                                         s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[1],
                                                                         s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[0]);     
    
    // Store current used address and increment used count
    s_context.app_context[p_dm_handle->device_id].ac_data.data.usage_idx = usage_idx;
    s_context.app_context[p_dm_handle->device_id].valid = true;

    err_code = dm_application_context_set(p_dm_handle, &s_context.app_context[p_dm_handle->device_id].ac);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;        
}

static uint32_t usage_idx_set(dm_handle_t *p_dm_handle, uint32_t usage_idx, ble_gap_addr_t *p_ble_addr)
{
    uint32_t                      err_code;
    
    if (memcmp(&s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr, p_ble_addr, sizeof(ble_gap_addr_t)))
    {
        // The address used and stored does not match, this is not the correct bond to update.
        DEBUG_PRINTF(0, "usage_idx_set - %02X%02X%02X%02X%02X%02X\r\n", s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[5],
                                                                             s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[4],
                                                                             s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[3],
                                                                             s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[2],
                                                                             s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[1],
                                                                             s_context.app_context[p_dm_handle->device_id].ac_data.data.ble_addr.addr[0]);
        DEBUG_PRINTF(0, "usage_idx_set - %02X%02X%02X%02X%02X%02X\r\n", current_addr.addr[5],
                                                                             current_addr.addr[4],
                                                                             current_addr.addr[3],
                                                                             current_addr.addr[2],
                                                                             current_addr.addr[1],
                                                                             current_addr.addr[0]);         
        DEBUG_PRINTF(0, "NBNB usage_idx_set address mismatch\r\n");
        
        return NRF_SUCCESS;
    }
    
    if (s_context.app_context[p_dm_handle->device_id].ac_data.data.usage_idx != usage_idx)
    {
        s_context.app_context[p_dm_handle->device_id].ac_data.data.usage_idx = usage_idx;
        
        err_code = dm_application_context_set(p_dm_handle, &s_context.app_context[p_dm_handle->device_id].ac);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }        
    }
    
    return NRF_SUCCESS;        
}

static uint32_t usage_idx_update(dm_handle_t *p_dm_handle, uint32_t usage_idx_current, uint32_t *p_usage_idx)
{
    uint32_t                      err_code;
    uint32_t                      max = 0; 
    dm_handle_t                   this_dm_handle;
    
    dm_handle_initialize(&this_dm_handle);
    this_dm_handle.appl_id = p_dm_handle->appl_id;
    this_dm_handle.connection_id = p_dm_handle->connection_id;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        this_dm_handle.device_id = dev_id;
        
        if (s_context.app_context[dev_id].valid)
        {
            if (s_context.app_context[dev_id].ac_data.data.usage_idx > max)
            {
                max = s_context.app_context[dev_id].ac_data.data.usage_idx;
            }
            
            if (s_context.app_context[dev_id].ac_data.data.usage_idx > usage_idx_current)
            {
                s_context.app_context[dev_id].ac_data.data.usage_idx -= 1;

                err_code = dm_application_context_set(&this_dm_handle, &s_context.app_context[dev_id].ac);
                if(err_code != NRF_SUCCESS)
                {
                    return err_code;
                }
            }
        }
    }
    
    *p_usage_idx = max;
    
    return NRF_SUCCESS;
}

static uint32_t usage_idx_new_get(uint32_t *p_usage_idx)
{
    uint32_t                      max = 0UL;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        if (s_context.app_context[dev_id].valid)
        {
            if (s_context.app_context[dev_id].ac_data.data.usage_idx > max)
            {
                max = s_context.app_context[dev_id].ac_data.data.usage_idx;
            }
        }
    }
    
    *p_usage_idx = max + 1UL;
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_addr_set(dm_handle_t *dm_handle, ble_gap_addr_t *p_ble_addr)
{
    uint32_t err_code;
    uint32_t current_usage_idx;
    uint32_t new_usage_idx;
    
    NULL_PARAM_CHECK(dm_handle);
    NULL_PARAM_CHECK(p_ble_addr);
    VERIFY_DEVICE_ID(dm_handle->device_id);    
    
    if (s_context.state != m_coms_ble_addr_state_idle)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    s_context.state = m_coms_ble_addr_state_set;
    
    err_code = usage_idx_get(dm_handle, &current_usage_idx);
    if (err_code == NRF_SUCCESS)
    {
        // Device already in list. UPDATE usage indexes.
        err_code = usage_idx_update(dm_handle, current_usage_idx, &new_usage_idx);
        if (err_code != NRF_SUCCESS)
        { 
            return err_code;
        }
    }
    else
    {
        // Device not in list. Increment the largest usage index already in the list.
        err_code = usage_idx_new_get(&new_usage_idx);
        if (err_code != NRF_SUCCESS) 
        {
            return err_code;
        }
    }
    
    err_code = addr_set(dm_handle, new_usage_idx, p_ble_addr);
    if (err_code != NRF_SUCCESS) 
    { 
        return err_code;
    }
    
    s_context.state = m_coms_ble_addr_state_idle;    
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_addr_notify_usage(dm_handle_t *dm_handle, ble_gap_addr_t *p_ble_addr)
{
    uint32_t err_code;
    uint32_t current_usage_idx;
    uint32_t new_usage_idx;
    
    NULL_PARAM_CHECK(dm_handle);
    NULL_PARAM_CHECK(p_ble_addr);
    VERIFY_DEVICE_ID(dm_handle->device_id);
    
    if (s_context.state != m_coms_ble_addr_state_idle)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
      
    
    s_context.state = m_coms_ble_addr_state_notify;
    
    err_code = usage_idx_get(dm_handle, &current_usage_idx);
    if (err_code == NRF_SUCCESS)
    {
        err_code = usage_idx_update(dm_handle, current_usage_idx, &new_usage_idx);
        if (err_code != NRF_SUCCESS)
        {
            s_context.state = m_coms_ble_addr_state_idle;  
            return err_code;
        }
    }
    else
    {
        DEBUG_PRINTF(0, "NBNB m_coms_ble_addr_notify_usage - %d\r\n", err_code);
        s_context.state = m_coms_ble_addr_state_idle;  
        return err_code;
    }
    
    if (current_usage_idx != new_usage_idx) {
        err_code = usage_idx_set(dm_handle, new_usage_idx, p_ble_addr);
        if (err_code != NRF_SUCCESS)
        {
            s_context.state = m_coms_ble_addr_state_idle;  
            return err_code;
        }
    }
    
    s_context.state = m_coms_ble_addr_state_idle;    
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_addr_addr_get(dm_application_instance_t app_id,
                                  uint32_t                  addr_idx,
                                  ble_gap_addr_t            *p_addr,
                                  dm_device_instance_t      *p_device_id)
{
    NULL_PARAM_CHECK(p_addr);
    NULL_PARAM_CHECK(p_device_id);
    
    if (s_context.state != m_coms_ble_addr_state_idle)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    s_context.state = m_coms_ble_addr_state_get;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        if (s_context.app_context[dev_id].valid)
        {
            if ((s_context.app_context[dev_id].ac_data.data.usage_idx - 1UL) == addr_idx)
            {
                memcpy(p_addr, &s_context.app_context[dev_id].ac_data.data.ble_addr, sizeof(ble_gap_addr_t));
                *p_device_id = dev_id;
                
                s_context.state = m_coms_ble_addr_state_idle; 
                return NRF_SUCCESS;
            }
        }
    }
    
    s_context.state = m_coms_ble_addr_state_idle;
    return NRF_ERROR_NOT_FOUND;
}

uint32_t m_coms_ble_addr_device_ids_get(dm_application_instance_t app_id, dm_device_instance_t *p_device_ids, uint8_t *p_device_num)
{
    uint8_t                       device_count = 0;
    
    NULL_PARAM_CHECK(p_device_ids);
    NULL_PARAM_CHECK(p_device_num);
    
    if (s_context.state != m_coms_ble_addr_state_idle)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    s_context.state = m_coms_ble_addr_state_dev_ids_get;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        if (s_context.app_context[dev_id].valid)
        {
            *p_device_ids++ = dev_id;
            device_count++;
        }
    }
    
    *p_device_num = device_count;
    
    s_context.state = m_coms_ble_addr_state_idle;   
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_addr_delete(dm_application_instance_t app_id,
                                uint32_t                  addr_idx,
                                dm_device_instance_t      *p_device_id)
{
    uint32_t     err_code;
    dm_handle_t  dm_handle;
    bool         deleted = false;    
    
    NULL_PARAM_CHECK(p_device_id);
    
    if (s_context.state != m_coms_ble_addr_state_idle)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    s_context.state = m_coms_ble_addr_state_delete;       
    
    // Read application context for stored bonds to determine largest used count.
    dm_handle_initialize(&dm_handle);
    dm_handle.appl_id = app_id;
    dm_handle.connection_id = 0;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        dm_handle.device_id = dev_id;
        
        if (s_context.app_context[dev_id].valid)
        {
            if ((s_context.app_context[dev_id].ac_data.data.usage_idx - 1) == addr_idx)
            {
                *p_device_id = dev_id;
                s_context.app_context[dev_id].valid = false;
                err_code = dm_application_context_delete(&dm_handle);
                if (err_code != NRF_SUCCESS)
                {
                    s_context.state = m_coms_ble_addr_state_idle; 
                    return err_code;
                }
                deleted = true;
            }
            else if ((s_context.app_context[dev_id].ac_data.data.usage_idx - 1) > addr_idx)
            {
                s_context.app_context[dev_id].ac_data.data.usage_idx -= 1;
                err_code = dm_application_context_set(&dm_handle, &s_context.app_context[dev_id].ac);
                if (err_code != NRF_SUCCESS)
                {
                    s_context.state = m_coms_ble_addr_state_idle;
                    return err_code;
                }
            }
        }
    }

    s_context.state = m_coms_ble_addr_state_idle; 

    return deleted ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;    
}

uint32_t m_coms_ble_addr_new_get(dm_application_instance_t app_id, ble_gap_addr_t *addr)
{
    dm_handle_t                   dm_handle;
    uint64_t                      largest_addr_val = 0;
    
    dm_handle_initialize(&dm_handle);
    dm_handle.appl_id = app_id;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        dm_handle.device_id = dev_id;
        
        if (s_context.app_context[dev_id].valid)
        {
            uint64_t this_addr_val = 0ULL;
            
            // Calculate address value
            this_addr_val =  (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[0];
            this_addr_val |= (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[1] << 8;
            this_addr_val |= (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[2] << 16;
            this_addr_val |= (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[3] << 24;
            this_addr_val |= (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[4] << 32;
            this_addr_val |= (uint64_t)s_context.app_context[dev_id].ac_data.data.ble_addr.addr[5] << 40;
            
            if (this_addr_val > largest_addr_val)
            {
                largest_addr_val = this_addr_val;
            }
        }
    }
    
    if(largest_addr_val == 0ULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    largest_addr_val += 1;
    
    addr->addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    addr->addr[0]   = largest_addr_val;
    addr->addr[1]   = largest_addr_val >> 8;
    addr->addr[2]   = largest_addr_val >> 16;
    addr->addr[3]   = largest_addr_val >> 24;
    addr->addr[4]   = largest_addr_val >> 32;
    addr->addr[5]   = largest_addr_val >> 40;
    addr->addr[5]  |= 0xC0; // Two upper bits must be "1"    
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_addr_init(dm_application_instance_t *p_app_id)
{
    dm_handle_t dm_handle;
    uint32_t    err_code;
    
    s_context.state = m_coms_ble_addr_state_idle;
    
    dm_handle_initialize(&dm_handle);
    
    dm_handle.appl_id = *p_app_id;
    
    for(dm_device_instance_t dev_id = 0; dev_id < DEVICE_MANAGER_MAX_BONDS; dev_id++)
    {
        dm_handle.device_id = dev_id;
        
        s_context.app_context[dev_id].ac.len    = sizeof(dm_application_context_data_t);
        s_context.app_context[dev_id].ac.p_data = (uint8_t *)&s_context.app_context[dev_id].ac_data;
      
        err_code = dm_application_context_get(&dm_handle, &s_context.app_context[dev_id].ac);
        if (err_code == NRF_SUCCESS)
        {
            s_context.app_context[dev_id].valid = true;
        }
        else
        {
            s_context.app_context[dev_id].valid = false;
        }

    }
    
    return NRF_SUCCESS;
}

