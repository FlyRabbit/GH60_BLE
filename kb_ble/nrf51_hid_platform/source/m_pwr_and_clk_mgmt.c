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
 
#include "m_pwr_and_clk_mgmt.h"

#include <stdbool.h>
#include <string.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util.h"
#include "common_params.h"
#include "nrf.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_delay.h"
#include "m_coms.h"

#define LFCLK_CAL_INACTIVITY_TIMEOUT   10
#define LFCLK_CTIV_MAX 127
#define LFCLK_CTIV_MIN 1


static uint32_t                    m_clk_mgmt_lfclk_cal_interval = (8000 / 250); // NRF_CLOCK_LFCLKSRC_RC_250_PPM_8000MS_CALIBRATION
static bool                        m_clk_mgmt_lfclk_cal_running  = false; 
static uint32_t                    m_pwr_mgmt_counter            = 0;
static uint32_t                    m_pwr_mgmt_sysoff_timeout     = 0xFFFFFFFF;
static uint32_t                    m_pwr_mgmt_idle_timeout       = 0xFFFFFFFF;
static pwr_mgmt_idle_callbacks_t   m_pwr_mgmt_idle_callbacks;
static pwr_mgmt_sysoff_callbacks_t m_pwr_mgmt_sysoff_callbacks;
static app_timer_id_t              m_pwr_mgmt_timer_id;
static app_sched_event_handler_t   m_pwr_mgmt_event_cb         = 0;

static void hfclk_crystal_oscillator_start(bool p_wait)
{
    if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) == 
        (CLOCK_HFCLKSTAT_SRC_RC << CLOCK_HFCLKSTAT_SRC_Pos))
    {
        uint8_t softdevice_enabled = 0;
        
        // 16M RC oscillator is running, start 16M XOSC instead
        sd_softdevice_is_enabled(&softdevice_enabled);
        if (softdevice_enabled == 0)
        {
            NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
            NRF_CLOCK->TASKS_HFCLKSTART    = 1;
            
            if (p_wait)
            {
                while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
                {
                    nrf_delay_us(900);
                    if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) == 
                        (CLOCK_HFCLKSTAT_SRC_RC << CLOCK_HFCLKSTAT_SRC_Pos))
                    {
                        NRF_CLOCK->TASKS_HFCLKSTART    = 1;
                        //break;
                    }
                }
            }
        }
        else
        {
            sd_clock_hfclk_request();
            
            if (p_wait)
            {
                uint32_t hfclk_running = false;
                while (!hfclk_running)
                {
                    sd_clock_hfclk_is_running(&hfclk_running);
                }
            }
        }
    }
}

static void hfclk_crystal_oscillator_stop(void)
{
    uint8_t softdevice_enabled = 0;
    
    sd_softdevice_is_enabled(&softdevice_enabled);
    if (softdevice_enabled == 0)
    {
        // Stopping 16M XOSC. This conserves the 20 µA XOSC standby current
        NRF_CLOCK->TASKS_HFCLKSTOP = 1;
    }
    else
    {
        sd_clock_hfclk_release();
    }
}

void POWER_CLOCK_IRQHandler(void)
{       
    if (NRF_CLOCK->EVENTS_CTTO)
    {
        NRF_CLOCK->EVENTS_CTTO = 0;
        
        nrf_delay_us(32); /* XLR workaround */
        NRF_CLOCK->TASKS_CTSTART = 1;        
        
        hfclk_crystal_oscillator_start(true);
    
        NRF_POWER->TASKS_CONSTLAT = 1;
        NRF_CLOCK->TASKS_CAL = 1;
        
        m_clk_mgmt_lfclk_cal_running = true;
    }
    
    if (NRF_CLOCK->EVENTS_DONE)
    {
        /* XLR workaround */
        nrf_delay_us(32); 
        nrf_delay_us(32); 
        nrf_delay_us(32); 

        NRF_CLOCK->EVENTS_DONE = 0;

        hfclk_crystal_oscillator_stop();

        NRF_POWER->TASKS_LOWPWR = 1;
    }
}

static void lfclk_calibration_start(void)
{
    bool hfclk_started = false;
    
    NRF_CLOCK->INTENCLR             = CLOCK_INTENSET_DONE_Msk | CLOCK_INTENSET_CTTO_Msk;
    NRF_CLOCK->EVENTS_DONE          = 0;
    NRF_CLOCK->EVENTS_CTTO          = 0;     
    
    if ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) != (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos))
    {    
        hfclk_crystal_oscillator_start(true);
        hfclk_started = true;
    }

    /* Initial calibration for the LFCLK RCOSC */
    NRF_CLOCK->TASKS_CAL = 1;
    while (NRF_CLOCK->EVENTS_DONE == 0) ;
    NRF_CLOCK->EVENTS_DONE = 0;    

    if (hfclk_started)
    {
        hfclk_crystal_oscillator_stop();
    }    

    /* Start the calibration timer using specified interval */
    NRF_CLOCK->INTENSET = CLOCK_INTENSET_DONE_Msk | CLOCK_INTENSET_CTTO_Msk;
    NRF_CLOCK->CTIV = m_clk_mgmt_lfclk_cal_interval;
    nrf_delay_us(32); /* XLR PAN workaround */
    NRF_CLOCK->TASKS_CTSTOP = 1;
    nrf_delay_us(32); /* XLR PAN workaround */
    NRF_CLOCK->TASKS_CTSTART = 1;
    
    NVIC_SetPriority(POWER_CLOCK_IRQn, 3);
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);    
}

static void lfclk_start(bool calibrate)
{
    // Start LFCLK if not already running
    if ((NRF_CLOCK->LFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) !=
        (CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos))
    {
        NRF_CLOCK->LFCLKSRC = ((CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos) & CLOCK_LFCLKSRC_SRC_Msk);    
        NRF_CLOCK->TASKS_LFCLKSTART = 1;
        
        while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
        {
            // Wait for LFCLK to start
        }
        
        NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;        
    }
    
    if (calibrate)
    {
        lfclk_calibration_start();        
    }
}

static void lfclk_calibration_handle(void)
{
    uint8_t softdevice_enabled = 0;
    
    sd_softdevice_is_enabled(&softdevice_enabled); 

    if (softdevice_enabled)
    {
        // Softdevice handles calibration.
    }
    else
    {
        // Handle calibration of LFCLK when in gzll mode.
        static uint32_t lfclk_cal_timer = 0;
        lfclk_cal_timer++;
        
        if (m_clk_mgmt_lfclk_cal_running)
        {
            // New calibration finished, reset calibrhation timer.
            lfclk_cal_timer = 0;
            m_clk_mgmt_lfclk_cal_running = false;
        }    
        else 
        if (lfclk_cal_timer >= LFCLK_CAL_INACTIVITY_TIMEOUT)
        {
            // Start new calibration if its more than 20 sec since last calibration done event.
            lfclk_cal_timer = 0;
            
            lfclk_calibration_start();
        }        
    }
}

static void timeout_handler(void* s_context)
{
    ++m_pwr_mgmt_counter;
    uint32_t err_code = NRF_SUCCESS;
    if (m_pwr_mgmt_counter == m_pwr_mgmt_idle_timeout)
    {
        m_pwr_and_clk_mgmt_event_t event;
        event.idle_timeout=true;
        event.sysoff_timeout = false;
        err_code = app_sched_event_put(&event,sizeof(event), m_pwr_mgmt_event_cb);
    }
    if (m_pwr_mgmt_counter >= m_pwr_mgmt_sysoff_timeout)
    {
        m_pwr_and_clk_mgmt_event_t event;
        event.idle_timeout=false;
        event.sysoff_timeout = true;
        err_code = app_sched_event_put(&event,sizeof(event), m_pwr_mgmt_event_cb);
    }
    APP_ERROR_CHECK(err_code);
    lfclk_calibration_handle();
}

uint32_t m_pwr_and_clk_mgmt_init(const m_prw_and_clk_mgmt_init_t* p_params, app_sched_event_handler_t event_handler)
{
    uint32_t err_code;
    uint8_t softdevice_enabled = 0;
    
    if (p_params == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (event_handler == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    m_pwr_mgmt_event_cb  = event_handler; 
    
    m_pwr_mgmt_counter        = 0;
    m_pwr_mgmt_sysoff_timeout = p_params->sysoff_timeout;
    m_pwr_mgmt_idle_timeout   = p_params->idle_timeout;
    memcpy(&m_pwr_mgmt_idle_callbacks, &p_params->idle, sizeof(m_pwr_mgmt_idle_callbacks));
    memcpy(&m_pwr_mgmt_sysoff_callbacks, &p_params->sysoff, sizeof(m_pwr_mgmt_sysoff_callbacks));  
    
    if ((p_params->lfclk_cal_interval <= LFCLK_CTIV_MAX) && 
        (p_params->lfclk_cal_interval >= LFCLK_CTIV_MIN))
    {
        m_clk_mgmt_lfclk_cal_interval = p_params->lfclk_cal_interval;
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }
        
    // Start LFCLK and LFCLK calibration if softdevice is not enabled.
    sd_softdevice_is_enabled(&softdevice_enabled); 

    if (softdevice_enabled == 0)
    {
        lfclk_start(true);        
    }
    
    err_code =  app_timer_create(&m_pwr_mgmt_timer_id,
                                 APP_TIMER_MODE_REPEATED,
                                 timeout_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Enabling (and keeping enabled) 16MHz XOSC when active. A running XOSC consumes less power than running RCOSC.
    // However, XOSC has a 20 µA standby current which greatly impacts the sleep current of the MCU.
    // Thus HFCLKSRC is switched to RCOSC after a short period of idle time.
    hfclk_crystal_oscillator_start(true);    
    
    return app_timer_start(m_pwr_mgmt_timer_id, APP_TIMER_TICKS(PWR_MGMT_POLLRATE, APP_TIMER_PRESCALER), 0);
}

void m_prw_mgmt_set_sysoff_timeout(uint32_t p_sysoff_timeout)
{
    m_pwr_mgmt_counter        = 0;
    m_pwr_mgmt_sysoff_timeout = p_sysoff_timeout;
}

void m_prw_mgmt_set_idle_timeout(uint32_t p_idle_timeout)
{
    m_pwr_mgmt_counter        = 0;
    m_pwr_mgmt_idle_timeout = p_idle_timeout;
}
 
void m_pwr_mgmt_run(void)
{
    uint32_t err_code;
    #ifdef BLE_ONLY
        err_code = sd_app_evt_wait();
        if (err_code == NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
        {
                __WFE();
                __SEV();
                __WFE();
        }
        else if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK_BOOL(false);
        }
        else
        {
        }        
    #else
        switch (m_coms_protocol_mode_get())
        {
            case protocol_mode_ble:
                err_code = sd_app_evt_wait();
                if (err_code == NRF_ERROR_SOFTDEVICE_NOT_ENABLED)
                {

                }
                else if (err_code != NRF_SUCCESS)
                {
                    APP_ERROR_CHECK_BOOL(false);
                }
                else
                {
                    break;
                }
            /* Fall through */

            case protocol_mode_gzll:
                __WFE();

                __SEV();
                __WFE();
                break;

            default:
                break;
        }  
    #endif
}

void m_pwr_mgmt_goto_idle(void * p_event_data, uint16_t event_size)
{
    int i;
    
    // Executing all callbacks
    for (i = 0; i < m_pwr_mgmt_idle_callbacks.num; ++i)
    {
        m_pwr_mgmt_idle_callbacks.callbacks[i]();
    }
    
    hfclk_crystal_oscillator_stop();
}

void m_pwr_mgmt_goto_sysoff(void * p_event_data, uint16_t event_size)
{
    uint8_t softdevice_enabled = false;
    int i;
    bool go_to_sys_off = true;
    
    // Executing all callbacks
    for (i = 0; i < m_pwr_mgmt_sysoff_callbacks.num; ++i)
    {
        if (!m_pwr_mgmt_sysoff_callbacks.callbacks[i](true))
        {
            go_to_sys_off = false;
        }
    }
    
    if (go_to_sys_off)
    {
        //debug_print(dbg_sys_off, 0,0);
        sd_softdevice_is_enabled(&softdevice_enabled);
        
        // Entering System OFF
        if (softdevice_enabled)
        {
            sd_power_system_off();
        }
        else
        {
            NRF_POWER->SYSTEMOFF = 0x1;
        }
    }
    else
    {
        // One or more modules are not ready for sys_off
    }
}
 
void m_pwr_mgmt_stay_in_sysoff(void * p_event_data, uint16_t event_size)
{
    uint8_t softdevice_enabled = 0;
    int i;
    
    // Executing all callbacks
    for (i = 0; i < m_pwr_mgmt_sysoff_callbacks.num; ++i)
    {
        m_pwr_mgmt_sysoff_callbacks.callbacks[i](false);
    }
    
    sd_softdevice_is_enabled(&softdevice_enabled);
  
    // Entering System OFF
    if (softdevice_enabled)
    {        
        sd_power_system_off();
    }
    else
    {
        NRF_POWER->SYSTEMOFF = 0x1;
    }
       
}

void m_pwr_mgmt_feed(void)
{
    m_pwr_mgmt_counter = 0;
    
    hfclk_crystal_oscillator_start(false);
}
