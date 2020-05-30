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
 
#include "m_mouse.h"

#include "app_timer.h"
#include "common_params.h"
#include "drv_mouse_btn.h"
#include "drv_mouse_scroll.h"
#include "drv_mouse_sensor.h"
#include "hal_gpiote.h"
#include "app_scheduler.h"
#include "io_cfg.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"

static app_sched_event_handler_t s_event_handler        = 0;
static app_timer_id_t            s_btn_timer_id         = 0;
static app_timer_id_t            s_sensor_timer_id      = 0;
static volatile bool             s_btn_timer_running    = false;
static volatile bool             s_sensor_timer_running = false;
static uint32_t                  s_sensor_pollrate      = M_MOUSE_SENSOR_POLLRATE;
static uint32_t                  s_button_pollrate      = M_MOUSE_BTN_POLLRATE;
static bool                      s_mouse_sensor_enabled = false;
static bool                      s_mouse_initialized = false;

static void sensor_handler(void*);
static void button_handler(void*);

static void gpio_int_handler(void)
{
    bool     interrupts_left = true; 
    uint32_t err_code;
    
    while (interrupts_left)
    {
        interrupts_left = false;
        
        // Has scroll wheel generated an interrupt?
        if (drv_mscroll_int_set())
        {
            // Enable QDEC
            drv_mscroll_enable();
            interrupts_left = true;
        }
        
        if (s_mouse_sensor_enabled)
        {
            // Has sensor generated an interrupt?
            if (drv_mouse_sensor_int_get())
            {
                if (!s_sensor_timer_running)
                {
                    err_code = app_timer_start(s_sensor_timer_id, APP_TIMER_TICKS(s_sensor_pollrate, APP_TIMER_PRESCALER), 0);
                    APP_ERROR_CHECK(err_code);
                    s_sensor_timer_running = true;
                    drv_mouse_sensor_sense_enable(false);
                    interrupts_left = true;
                }
            }
        }
        // Button poll timer already running?
        if (!s_btn_timer_running)
        {
            // Starting button poll timer regardless if buttons are pressed or not
            err_code = app_timer_start(s_btn_timer_id, APP_TIMER_TICKS(s_button_pollrate, APP_TIMER_PRESCALER), 0);            
            APP_ERROR_CHECK(err_code);
            s_btn_timer_running = true;
            drv_mouse_btn_sense_enable(false);
            interrupts_left = true;
        }
    }
}

static void sensor_handler(void* p_context)
{
    uint32_t err_code;
    int16_t  delta_x;
    int16_t  delta_y;    
    
    if (drv_mouse_sensor_int_get())
    {   
        drv_mouse_sensor_read(&delta_x, &delta_y);    

        if (delta_x || delta_y)
        {
            m_mouse_data_t data;

            data.type                        = mouse_packet_type_motion;
            data.data.motion.x_delta = delta_x;
            data.data.motion.y_delta = delta_y;
            
            app_sched_event_put(&data, sizeof(m_mouse_data_t), s_event_handler);
        }
        else
        {
            drv_mouse_sensor_sense_enable(true);
        }
    }
    else
    {
        if (s_sensor_timer_running)
        {
            // No mouse movement
            err_code = app_timer_stop(s_sensor_timer_id);
            APP_ERROR_CHECK(err_code);
            s_sensor_timer_running = false;
            drv_mouse_sensor_sense_enable(true);            
        }
        else
        {
            drv_mouse_sensor_sense_enable(true);
        }
    }
}

static void button_handler(void* p_context)
{
    static uint8_t prev_buttons     = 0;
    static uint8_t prev_adv_buttons = 0;
    uint8_t        buttons          = 0;
    uint8_t        adv_buttons      = 0;
    uint32_t       all_lines;
    
    // Read button state
    all_lines = drv_mouse_btn_read();    
   
    // Check which buttons are held and flip interrupt sense polarity of those who are
    if (all_lines & IO_BTN_LEFT_MSK)
    {
        buttons |= (1 << 0);
    }
    
    if (all_lines & IO_BTN_RIGHT_MSK)
    {
        buttons |= (1 << 1);
    }
    
    if (all_lines & IO_BTN_MIDDLE_MSK)
    {
        buttons |= (1 << 2);
    }  
    
    if (all_lines & IO_BTN_SIDE_LEFT_MSK)
    {
        buttons |= (1 << 3);
    }
    
    if (all_lines & IO_BTN_SIDE_RIGHT_MSK)
    {
        buttons |= (1 << 4);
    }
    
    if ((all_lines & IO_BTN_RESET_MSK) == IO_BTN_RESET_MSK)
    {    
        // Pairing button combo has been pressed.
        m_mouse_data_t data;
        data.type = mouse_packet_type_pairing_button;
        
        while(((drv_mouse_btn_read() & IO_BTN_RESET_MSK) == IO_BTN_RESET_MSK));
        

        app_sched_event_put(&data, sizeof(m_mouse_data_t), s_event_handler);
        return;
    }
    
    if (buttons != prev_buttons)
    {
        m_mouse_data_t data;       
        
        data.type                 = mouse_packet_type_buttons;
        data.data.buttons = buttons;
        
        app_sched_event_put(&data, sizeof(m_mouse_data_t), s_event_handler);
        
        prev_buttons = buttons;
    }
    
     if (adv_buttons != prev_adv_buttons)
     {
         m_mouse_data_t data;     
         
         data.type                     = mouse_packet_type_adv_buttons;
         data.data.adv_buttons = adv_buttons;

         app_sched_event_put(&data, sizeof(m_mouse_data_t), s_event_handler);
         
         prev_adv_buttons = adv_buttons;
     }
    
    if ((adv_buttons == 0) && (buttons == 0))
    {
        uint32_t err_code;
        if (s_btn_timer_running)
        {
            err_code = app_timer_stop(s_btn_timer_id);
            APP_ERROR_CHECK(err_code);
            s_btn_timer_running = false;
            drv_mouse_btn_sense_enable(true);
        }
        else
        {
            drv_mouse_btn_sense_enable(true);
        }
    }
}

static void scroll_wheel_handler(int32_t mwheel_delta)
{
    if (mwheel_delta != 0)
    {
        m_mouse_data_t data;

        data.type        = mouse_packet_type_scroll;
        data.data.scroll = mwheel_delta;

        app_sched_event_put(&data, sizeof(m_mouse_data_t), s_event_handler);
    }
}

uint32_t m_mouse_init(m_mouse_init_t *init)
{
    uint32_t         err_code;
    
    if ((init->event_handler   == 0) || 
        (init->sensor_pollrate == 0) || 
        (init->button_pollrate == 0) ||
        (s_mouse_initialized   == true))
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    s_sensor_pollrate = init->sensor_pollrate;
    s_button_pollrate = init->button_pollrate;
    s_event_handler   = init->event_handler;
    
    err_code = app_timer_create(&s_btn_timer_id, APP_TIMER_MODE_REPEATED, button_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    } 
    
    err_code = app_timer_create(&s_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensor_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }    
    
    err_code = drv_mscroll_init(scroll_wheel_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = drv_mouse_sensor_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code = drv_mouse_btn_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    drv_mouse_sensor_sense_enable(false);
    if (drv_mouse_sensor_int_get())
    {
        // Reading mouse sensor to clear Motion interrupt
        drv_mouse_sensor_dummy_read(); 
    }
    
    // GPIOTE callback
    err_code = hal_gpiote_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }       
    
    err_code = hal_gpiote_cb_set(hal_gpiote_evt_port, gpio_int_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    err_code =  app_timer_start(s_btn_timer_id, APP_TIMER_TICKS(s_button_pollrate, APP_TIMER_PRESCALER), 0);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    s_btn_timer_running = true;
    s_mouse_initialized = true;
    
    return err_code;
}

uint32_t m_mouse_sensor_enable(void)
{
    uint32_t err_code = NRF_SUCCESS;
    
    if (!s_mouse_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (!s_mouse_sensor_enabled)
    {
        s_mouse_sensor_enabled = true;
        
        drv_mouse_sensor_sense_enable(false);
        if (drv_mouse_sensor_int_get())
        {
            // Reading mouse sensor to clear Motion interrupt
            drv_mouse_sensor_dummy_read(); 
        }        
        
        if (!s_sensor_timer_running)
        {   
            err_code = app_timer_start(s_sensor_timer_id, APP_TIMER_TICKS(s_sensor_pollrate, APP_TIMER_PRESCALER), 0);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            s_sensor_timer_running = true;
        }
        else
        {
            // Timer already running.
        }
    }
    else
    {
        // No action needed, mouse sensor already enabled.
    }
    
    return err_code;
}

uint32_t m_mouse_sensor_disable(void)
{
    uint32_t err_code = NRF_SUCCESS;
 
    if (!s_mouse_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }    
    
    if (s_mouse_sensor_enabled)
    {
        s_mouse_sensor_enabled = false;
        drv_mouse_sensor_sense_enable(false);
        
        if (s_sensor_timer_running)
        {
            err_code = app_timer_stop(s_sensor_timer_id);
            if (err_code != NRF_SUCCESS)
            {
                return err_code;
            }
            s_sensor_timer_running = false;
        }
        else
        {
            // Timer already stopped.
        }
    }
    else
    {
        // No action needed, mouse sensor already disabled.        
    }
    
    return err_code;
}

void m_mouse_pollrate_set(uint32_t sensor_pollrate, uint32_t button_pollrate)
{    
    if (sensor_pollrate)
    {
        s_sensor_pollrate = sensor_pollrate;
    }
    
    if (button_pollrate)
    {
        s_button_pollrate = button_pollrate;
    }
}

bool m_mouse_is_pairing_btn_pressed(void)
{
    bool pairing_btn_pressed = false;
    
    drv_mouse_btn_sense_enable(false);
    
    if((drv_mouse_btn_read() & IO_BTN_RESET_MSK) == IO_BTN_RESET_MSK)
    {
        pairing_btn_pressed = true;
    }
    
    drv_mouse_btn_sense_enable(true);
    return pairing_btn_pressed;
}

bool m_mouse_wakeup_prepare(bool wakeup)
{
    
    if (wakeup)
    {
                uint8_t     softdevice_enabled     = false;
    uint32_t    err_code;
    #ifdef SOFTDEVICE_PRESENT
    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
    APP_ERROR_CHECK(err_code);
#else
#ifndef SOFTDEVICE_NOT_PRESENT
#error "Please define either SOFTDEVICE_PRESENT or SOFTDEVICE_NOT_PRESENT to state if a Softdevice is present or not."
#endif /* SOFTDEVICE_NOT_PRESENT */
#endif /* SOFTDEVICE_PRESENT */ 

    if (softdevice_enabled == 1)
    {   
        sd_nvic_DisableIRQ(GPIOTE_IRQn);        
        sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
    }
    else
    {       
        NVIC_DisableIRQ(GPIOTE_IRQn);
        NVIC_ClearPendingIRQ(GPIOTE_IRQn);
        
    }
        drv_mscroll_wakeup_prepare();
        drv_mouse_sensor_wakeup_prepare();
        drv_mouse_btn_wakeup_prepare();
    }
    else
    {
        // disable all sense
    }
    return true;
}
