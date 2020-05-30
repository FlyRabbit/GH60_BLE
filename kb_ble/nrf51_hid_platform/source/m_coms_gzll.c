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
 
#include "m_coms_gzll.h"
#include "m_coms.h"
#include "nordic_common.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_nvmc.h"
#include "nrf_delay.h"
#include "nrf_gzll.h"
#include "nrf_gzp.h"
#include "nrf_gzllde_params.h"
#include "nrf51.h"

#define MAX_RTC_TASKS_DELAY     47                                          /**< Maximum delay until an RTC task is executed. */


static bool                      m_coms_gzll_have_system_address = false;
static bool                      m_coms_gzll_have_host_id        = false;
static bool                      m_coms_gzll_use_encrypted_pipe  = false;
static bool                      m_coms_gzll_tx_attempted        = false;
static uint32_t                  m_coms_gzll_max_tx_attempts     = 0;
static uint32_t                  m_coms_gzll_timeslot_period     = 0;
static uint32_t                  m_coms_gzll_sync_lifetime       = 0;
static app_sched_event_handler_t m_coms_event_callback           = 0;

static void m_coms_gzll_app_timer_suspend(void)
{
   NRF_RTC1->TASKS_STOP  = 1; 
   nrf_delay_us(MAX_RTC_TASKS_DELAY);
}
static void m_coms_gzll_app_timer_resume(void)
{
   NRF_RTC1->TASKS_START= 1; 
   nrf_delay_us(MAX_RTC_TASKS_DELAY);
}

static void m_coms_gzll_send_event(gzll_event_t p_event)
{
    uint32_t err_code;
    gzll_event_t evt;

    evt = p_event;
    
    err_code = app_sched_event_put(&evt, sizeof(evt), m_coms_event_callback);
    APP_ERROR_CHECK(err_code);
}


static gzp_id_req_res_t send_host_id_req(void)
{
    gzp_id_req_res_t id_resp;
    
    // gzp_id_req_send() could take several seconds. Have to disable RTC timer to prevent app_timer overflow
    m_coms_gzll_app_timer_suspend();
    id_resp = gzp_id_req_send(); 
    m_coms_gzll_app_timer_resume();

    switch(id_resp)
    {
        case GZP_ID_RESP_REJECTED:
        case GZP_ID_RESP_FAILED:
            // Reset variables so that pairing restarts from the beginning. 
            m_coms_gzll_have_host_id        = false;
            m_coms_gzll_have_system_address = false;                    
            m_coms_gzll_send_event(gzll_event_hostid_rejected);
            break;
        case GZP_ID_RESP_GRANTED:
            // Reset variables so that pairing restarts from the beginning. 
            m_coms_gzll_have_host_id        = true;
            m_coms_gzll_have_system_address = true;
            m_coms_gzll_send_event(gzll_event_hostid_exchanged);
            break;
        case GZP_ID_RESP_PENDING:
            /* Fall through */
        default:
            break; 
    }
    return id_resp;
}

static void send_addr_req(void)
{
    // gzp_address_req_send() could take several seconds. Have to disable RTC timer to prevent app_timer overflow
    m_coms_gzll_app_timer_suspend();
    m_coms_gzll_have_system_address = gzp_address_req_send();
    m_coms_gzll_app_timer_resume();  
    if (m_coms_gzll_have_system_address)
    {
        m_coms_gzll_send_event(gzll_event_sysaddr_exchanged);
    }
    else
    {
        m_coms_gzll_send_event(gzll_event_sysaddr_timeout);
    }    
}





uint32_t m_coms_gzll_send(uint8_t p_pipe, uint8_t * p_packet, uint8_t p_packet_size)
{    
    if(!m_coms_gzll_have_system_address)
    {
        send_addr_req();
    }
    
    if (m_coms_gzll_tx_attempted && !nrf_gzp_tx_success())
    {
        m_coms_gzll_send_event(gzll_event_tx_failed);
    }
    
    m_coms_gzll_tx_attempted = false;
    
    if (!m_coms_gzll_have_system_address)
    {
        // Still don't have system address?
        return NRF_ERROR_INVALID_STATE;
    }
    
    if(p_pipe < 2) //pipe 0 and 1 is used by Gazell pairing (gzp)
    {
        if(m_coms_gzll_use_encrypted_pipe)        
        {
            bool success;
            
            if(!m_coms_gzll_have_host_id)
            {
                send_host_id_req();
            }
            // Note that this function will block until transmission completes or fails
            m_coms_gzll_app_timer_suspend();
            success = gzp_crypt_data_send(p_packet, p_packet_size);     
            m_coms_gzll_app_timer_resume();
            if (!success)
            {
                m_coms_gzll_send_event(gzll_event_tx_failed);
                
                return NRF_ERROR_INVALID_STATE;
            }
            
         }
         else
         {
             m_coms_gzll_send_event(gzll_event_tx_failed);
             return NRF_ERROR_INVALID_STATE;                 
         }
    }
    else if(p_pipe < NRF_GZLL_CONST_PIPE_COUNT)
    {
          // We should not have any packets in our RX FIFO at this point
        if (nrf_gzll_get_rx_fifo_packet_count(p_pipe))
        {
            nrf_gzp_flush_rx_fifo(p_pipe);            
        }
        m_coms_gzll_tx_attempted = nrf_gzll_add_packet_to_tx_fifo(p_pipe, p_packet, p_packet_size);
    }        
    else
    {
           return NRF_ERROR_INVALID_PARAM;
    }  
   
    return NRF_SUCCESS;
}

 


uint32_t m_coms_gzll_init(const app_sched_event_handler_t event_callback, const m_coms_gzll_params_t * p_gzll_params)
{
    int8_t pairing_status;
    
    if (event_callback == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    m_coms_event_callback           = event_callback;
    m_coms_gzll_max_tx_attempts          = p_gzll_params->tx_attempts;
    m_coms_gzll_use_encrypted_pipe  = p_gzll_params->encrypt;
    m_coms_gzll_timeslot_period     = p_gzll_params->timeslot_period;
    m_coms_gzll_sync_lifetime       = p_gzll_params->sync_lifetime;
    m_coms_gzll_tx_attempted             = false;
    m_coms_gzll_have_system_address = false;
    m_coms_gzll_have_host_id        = false;    
    
    // Checking if we have pairing data stored in flash
    pairing_status =  gzp_get_pairing_status();
    
    if (pairing_status == -1)
    {
        m_coms_gzll_have_system_address = true;
    }
    else if (pairing_status >= 0)
    {
        m_coms_gzll_have_system_address = true;
        m_coms_gzll_have_host_id        = true;
    }
    
    return NRF_SUCCESS;
}

uint32_t m_coms_gzll_enable(bool search_for_host)
{
    bool init_ok = true;     
    
    // Initialize and enable Gazell
    init_ok &= nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    
    // Ensure Gazell parameters are configured.
    init_ok &= nrf_gzll_set_max_tx_attempts(m_coms_gzll_max_tx_attempts);
    init_ok &= nrf_gzll_set_device_channel_selection_policy(NRF_GZLLDE_DEVICE_CHANNEL_SELECTION_POLICY);
    init_ok &= nrf_gzll_set_timeslot_period(m_coms_gzll_timeslot_period);
    init_ok &= nrf_gzll_set_sync_lifetime(m_coms_gzll_sync_lifetime); 
    
    gzp_init();

    init_ok &= nrf_gzll_enable();

    if (!init_ok)
    {
        return NRF_ERROR_INTERNAL;
    }
    
    if (search_for_host && !m_coms_gzll_have_system_address)
    {
        // Start looking for a host
        send_addr_req();
    }
    
    if (search_for_host && m_coms_gzll_have_system_address)
    {
        bool    success;
        uint8_t dummy_pkt[4] = {0x02,0,0,0};
        
        // Transmission could take a long time, need to stop timers in the meantime
        NRF_CLOCK->TASKS_LFCLKSTOP = 1;  
        
        // Send a dummy packet to see if host is actually there
        // Dummy packet is an empty motion packet
        nrf_gzp_reset_tx_complete();
        success = nrf_gzll_add_packet_to_tx_fifo(2, dummy_pkt, sizeof(dummy_pkt));
        if (!success)
        {
            return NRF_ERROR_INTERNAL;
        }
        
        while (!nrf_gzp_tx_complete())
        {
            // Wait for transmission to complete (or fail)
            nrf_delay_us(1000);
        }
        
        if (nrf_gzp_tx_success())
        {
            m_coms_gzll_send_event(gzll_event_sysaddr_exchanged);
        }
        else
        {
            m_coms_gzll_send_event(gzll_event_sysaddr_timeout);
        }
    }
    
    // Make sure 32 kHz clock is running (needed by timer module)
    NRF_CLOCK->TASKS_LFCLKSTART = 1;   
    
    return NRF_SUCCESS;
}


uint32_t m_coms_gzll_disable(void)
{
    nrf_gzll_disable();
    
    while (nrf_gzll_is_enabled())
    {
        // Wait.
    }
    
    // Clean up after Gazell
    NVIC_DisableIRQ(RADIO_IRQn);
    NVIC_DisableIRQ(NRF_GZLL_TIMER_IRQn);
    NVIC_DisableIRQ(NRF_GZLL_SWI_IRQn);
    NVIC_DisableIRQ(POWER_CLOCK_IRQn); 
    
    NRF_PPI->CHENCLR = NRF_GZLL_PPI_CHEN_MSK_0_AND_1;
    NRF_PPI->CHENCLR = NRF_GZLL_PPI_CHEN_MSK_2;
    
    m_coms_gzll_send_event(gzll_event_disabled);
    
    return NRF_SUCCESS;
}

bool m_coms_gzll_keys_stored(void)
{
    return m_coms_gzll_have_system_address;
}

void m_coms_gzll_clear_keys(void)
{
    // Deleting gzp keys from flash
    nrf_nvmc_page_erase((uint32_t)GZP_PARAMS_STORAGE_ADR);
    m_coms_gzll_have_system_address = false;
    m_coms_gzll_have_host_id = false;    
}

uint32_t m_coms_gzll_bonding_start(void)
{
    bool new_system_address=false;
    
    m_coms_gzll_app_timer_suspend();
    
    new_system_address = gzp_address_req_send();
    if(new_system_address == true)
    {
        m_coms_gzll_have_system_address = true;
    }
    m_coms_gzll_app_timer_resume(); 
    
    if (new_system_address)
    {
        m_coms_gzll_send_event(gzll_event_sysaddr_exchanged);
    }
    else
    {
        return NRF_ERROR_TIMEOUT;
    }
    
    return NRF_SUCCESS;
}
bool m_coms_gzll_wakeup_prepare(void)
{
    return true;
  // Don't have to do anything
}
 
