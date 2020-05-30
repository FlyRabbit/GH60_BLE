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
 
#include "drv_mouse_scroll.h"

#include "app_timer.h"
#include "nrf_assert.h"
#include "common_params.h"
#include "hal_qdec.h"
#include "io_cfg.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_soc.h"

static void             (*s_scroll_event_cb)(int32_t) = 0;
static app_timer_id_t   s_timer_id                    = 0; 
static volatile bool    s_scroll_detected             = false;
static volatile bool    s_first_step                  = false; //flag to indicate start of a scroll

static void timeout_handler(void* p_context)
{
    // If no scroll motion has happened since last timer event: stop QDEC
    if (!s_scroll_detected)
    {
        drv_mscroll_disable();
        app_timer_stop(s_timer_id);
    }
    
    s_scroll_detected = false;
}

static void qdec_int_callback(hal_qdec_event_t p_evt_type)
{
    int32_t scroll;
    
    switch (p_evt_type)
    {
        case hal_qdec_event_samplerdy:
            break;
        
        case hal_qdec_event_reportrdy:
            scroll = hal_qdec_accread();
            
            if (scroll != 0)
            {
                if(s_first_step)
                {
                    if(scroll > 1)
                    {
                        scroll--;
                    }
                    if(scroll < -1)
                    {
                        scroll++;
                    }
                    s_first_step=false;
                }
                
                s_scroll_event_cb(scroll);
                s_scroll_detected = true;
            }
            break;
        
        case hal_qdec_event_accof:
            break;
    
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

bool drv_mscroll_int_set(void)
{
    uint32_t input;
    bool scroll1_high;
    bool scroll2_high;
    uint32_t scroll1_sense = (NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] & GPIO_PIN_CNF_SENSE_Msk) >> 
                                GPIO_PIN_CNF_SENSE_Pos;
    uint32_t scroll2_sense = (NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] & GPIO_PIN_CNF_SENSE_Msk) >> 
                                GPIO_PIN_CNF_SENSE_Pos;
    
    input        = NRF_GPIO->IN;
    scroll1_high = ((input & (1 << IO_QDEC_SCROLL1_PIN)) != 0);
    scroll2_high = ((input & (1 << IO_QDEC_SCROLL2_PIN)) != 0);
    
    if (((scroll1_sense == GPIO_PIN_CNF_SENSE_High) && scroll1_high)  || 
        ((scroll1_sense == GPIO_PIN_CNF_SENSE_Low) &&  !scroll1_high) || 
        ((scroll2_sense == GPIO_PIN_CNF_SENSE_High) &&  scroll2_high) || 
        ((scroll2_sense == GPIO_PIN_CNF_SENSE_Low) &&  !scroll2_high))
    {
        return true;
    }
    
    return false;
}

uint32_t drv_mscroll_init(void (*scroll_event_cb)(int32_t))
{
    uint32_t        err_code;
    uint32_t        scroll1_sense = GPIO_PIN_CNF_SENSE_High;
    uint32_t        scroll2_sense = GPIO_PIN_CNF_SENSE_High;
    uint32_t        input;
    hal_qdec_init_t qdec_params;

    if (scroll_event_cb == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    // Connecting pins
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);  
        
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL3_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos);
    
    s_scroll_event_cb = scroll_event_cb;     
    input             = NRF_GPIO->IN;
    
    // Configuring pin sense
    if (input & (1 << IO_QDEC_SCROLL1_PIN))
    {
        scroll1_sense = GPIO_PIN_CNF_SENSE_Low;
    }
    
    if (input & (1 << IO_QDEC_SCROLL2_PIN))
    {
        scroll2_sense = GPIO_PIN_CNF_SENSE_Low;
    }
    
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] = 
          (scroll1_sense               << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] = 
          (scroll2_sense               << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);  
    
        
    NRF_GPIO->OUTCLR = (1 << IO_QDEC_SCROLL3_PIN);
    
    err_code =  app_timer_create(&s_timer_id,
                                 APP_TIMER_MODE_REPEATED,
                                 timeout_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    qdec_params.qdec_int_callback = qdec_int_callback;
    qdec_params.reportper         = hal_qdec_reportper_280smpl;
    qdec_params.sampleper         = hal_qdec_sampleper_256us;
    qdec_params.psela             = IO_QDEC_SCROLL1_PIN;
    qdec_params.pselb             = IO_QDEC_SCROLL2_PIN;
    qdec_params.pselled           = 0xFFFFFFFF;
    qdec_params.ledpre            = 0;
    qdec_params.dbfen             = 1;
    
    return hal_qdec_init(&qdec_params);
}

uint32_t drv_mscroll_enable(void)
{
    uint32_t err_code;
    
    // Disabling pin sense and disconnecting pins
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled    << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1        << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled     << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input         << GPIO_PIN_CNF_DIR_Pos); 
    
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled    << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1        << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled     << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input         << GPIO_PIN_CNF_DIR_Pos);     
    
    s_first_step=true;
    err_code = hal_qdec_int_enable(QDEC_INTENSET_REPORTRDY_Msk | QDEC_INTENSET_ACCOF_Msk);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code =  app_timer_start(s_timer_id, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 0);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    hal_qdec_start();
    
    return err_code;
}


void drv_mscroll_disable(void)
{
    hal_qdec_int_disable(QDEC_INTENSET_REPORTRDY_Msk | QDEC_INTENSET_ACCOF_Msk);
    hal_qdec_stop();
    
    // Resetting pin configuration
    drv_mscroll_wakeup_prepare();
}


bool drv_mscroll_wakeup_prepare(void)
{
    uint32_t scroll1_sense = GPIO_PIN_CNF_SENSE_High;
    uint32_t scroll2_sense = GPIO_PIN_CNF_SENSE_High;
    uint32_t input;
    
    // Connecting pins
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);   

    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled  << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos);  
    
    input = NRF_GPIO->IN;
    
    // Setting pin sense to opposite of current state
    if (input & (1 << IO_QDEC_SCROLL1_PIN))
    {
        scroll1_sense = GPIO_PIN_CNF_SENSE_Low;
    }
    
    if (input & (1 << IO_QDEC_SCROLL2_PIN))
    {
        scroll2_sense = GPIO_PIN_CNF_SENSE_Low;
    }

    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL1_PIN] = 
          (scroll1_sense                 << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos); 

    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL2_PIN] = 
          (scroll2_sense                 << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)   
        | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos); 
   
        
    NRF_GPIO->PIN_CNF[IO_QDEC_SCROLL3_PIN] = 
          (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos)
        | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
        | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
        | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
        | (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos);

    return true;
}
