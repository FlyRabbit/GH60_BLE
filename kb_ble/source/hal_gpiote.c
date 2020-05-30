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
 
#include "hal_gpiote.h"

#include <string.h>

#include "nrf_error.h"
#include "nrf_delay.h"
#include "nrf_sdm.h"

#define HAL_GPIOTE_MAX_CB_PER_EVENT 2

typedef struct
{
    hal_gpiote_callback_t callbacks[HAL_GPIOTE_MAX_CB_PER_EVENT];
    uint8_t               num_callbacks;
} hal_gpiote_cb_list_t;

static hal_gpiote_cb_list_t s_callbacks[5] = {0};

void GPIOTE_IRQHandler(void)
{
    int i;
    
    if (NRF_GPIOTE->EVENTS_IN[0])
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        for (i = 0; i < s_callbacks[0].num_callbacks; ++i)
        {
            (s_callbacks[0].callbacks[i])();
        }
    }
    if (NRF_GPIOTE->EVENTS_IN[1])
    {
        NRF_GPIOTE->EVENTS_IN[1] = 0;
        for (i = 0; i < s_callbacks[1].num_callbacks; ++i)
        {
            (s_callbacks[1].callbacks[i])();
        }
    }
    if (NRF_GPIOTE->EVENTS_IN[2])
    {
        NRF_GPIOTE->EVENTS_IN[2] = 0;
        for (i = 0; i < s_callbacks[2].num_callbacks; ++i)
        {
            (s_callbacks[2].callbacks[i])();
        }
    }
    if (NRF_GPIOTE->EVENTS_IN[3])
    {
        NRF_GPIOTE->EVENTS_IN[3] = 0;
        for (i = 0; i < s_callbacks[3].num_callbacks; ++i)
        {
            (s_callbacks[3].callbacks[i])();
        }
    }
    if (NRF_GPIOTE->EVENTS_PORT)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
        for (i = 0; i < s_callbacks[4].num_callbacks; ++i)
        {
            (s_callbacks[4].callbacks[i])();
        }
    }    
}

uint32_t hal_gpiote_init(void)
{
    static bool hal_gpiote_initialized = false;
    uint8_t     softdevice_enabled     = false;
    uint32_t    err_code;
    
    if (hal_gpiote_initialized)
    {
        return NRF_SUCCESS;
    }
    
#ifdef SOFTDEVICE_PRESENT
    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#else
#ifndef SOFTDEVICE_NOT_PRESENT
#error "Please define either SOFTDEVICE_PRESENT or SOFTDEVICE_NOT_PRESENT to state if a Softdevice is present or not."
#endif /* SOFTDEVICE_NOT_PRESENT */
#endif /* SOFTDEVICE_PRESENT */ 

    if (softdevice_enabled == 1)
    {
        sd_nvic_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
        sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
        sd_nvic_EnableIRQ(GPIOTE_IRQn);
    }
    else
    {
        NVIC_SetPriority(GPIOTE_IRQn, NRF_APP_PRIORITY_LOW);
        NVIC_ClearPendingIRQ(GPIOTE_IRQn);
        NVIC_EnableIRQ(GPIOTE_IRQn);
    }
    
    hal_gpiote_initialized = true;

    return NRF_SUCCESS;
}

uint32_t hal_gpiote_cfg(const hal_gpiote_cfg_t* p_cfg)
{
    switch (p_cfg->chn)
    {
        case hal_gpiote_chn_0:
            /* Fall through */
        case hal_gpiote_chn_1:
            /* Fall through */
        case hal_gpiote_chn_2:
            /* Fall through */
        case hal_gpiote_chn_3:
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    switch (p_cfg->mode)
    {
        case hal_gpiote_mode_disabled:
            /* Fall through */
        case hal_gpiote_mode_event:
            /* Fall through */
        case hal_gpiote_mode_task:
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    switch (p_cfg->polarity)
    {
        case hal_gpiote_pol_lotohi:
            /* Fall through */
        case hal_gpiote_pol_hitolo:
            /* Fall through */
        case hal_gpiote_pol_toggle:
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    switch (p_cfg->outinit)
    {
        case hal_gpiote_outinit_low:
            /* Fall through */
        case hal_gpiote_outinit_high:
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    if (p_cfg->psel > 31)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    NRF_GPIOTE->CONFIG[p_cfg->chn] = (p_cfg->mode     << GPIOTE_CONFIG_MODE_Pos)     | 
                                     (p_cfg->psel     << GPIOTE_CONFIG_PSEL_Pos)     | 
                                     (p_cfg->polarity << GPIOTE_CONFIG_POLARITY_Pos) | 
                                     (p_cfg->outinit  << GPIOTE_CONFIG_OUTINIT_Pos);
    
    return NRF_SUCCESS;
}

uint32_t hal_gpiote_cb_set(hal_gpiote_evt_num_t p_evt_num, hal_gpiote_callback_t p_cb)
{
    bool interrupt_set;
    
    if (p_evt_num > 4)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (s_callbacks[p_evt_num].num_callbacks >= HAL_GPIOTE_MAX_CB_PER_EVENT)
    {
        return NRF_ERROR_NO_MEM;
    }
    
    s_callbacks[p_evt_num].callbacks[s_callbacks[p_evt_num].num_callbacks] = p_cb;
    ++s_callbacks[p_evt_num].num_callbacks;
    
    interrupt_set = p_cb != 0 ? true : false;
    switch (p_evt_num)
    {
        case hal_gpiote_evt_0:
            NRF_GPIOTE->EVENTS_IN[0] = 0;
            NRF_GPIOTE->INTENSET = interrupt_set ? GPIOTE_INTENSET_IN0_Msk : 0;
            break;
        case hal_gpiote_evt_1:
            NRF_GPIOTE->EVENTS_IN[1] = 0;
            NRF_GPIOTE->INTENSET = interrupt_set ? GPIOTE_INTENSET_IN1_Msk : 0;
            break;
        case hal_gpiote_evt_2:
            NRF_GPIOTE->EVENTS_IN[2] = 0;
            NRF_GPIOTE->INTENSET = interrupt_set ? GPIOTE_INTENSET_IN2_Msk : 0;
            break;
        case hal_gpiote_evt_3:
            NRF_GPIOTE->EVENTS_IN[3] = 0;
            NRF_GPIOTE->INTENSET = interrupt_set ? GPIOTE_INTENSET_IN3_Msk : 0;
            break;
        case hal_gpiote_evt_port:
            NRF_GPIOTE->INTENSET = interrupt_set ? GPIOTE_INTENSET_PORT_Msk : 0;;
            NRF_GPIOTE->EVENTS_PORT = 0;
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    return NRF_SUCCESS;
}

uint32_t hal_gpiote_cb_clr(hal_gpiote_evt_num_t p_evt_num, hal_gpiote_callback_t p_cb)
{
    int  i;
    bool callbacks_left;
    
    if (p_cb == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    callbacks_left = (s_callbacks[p_evt_num].num_callbacks == 0 ? false : true);
    
    for (i = 0; i < HAL_GPIOTE_MAX_CB_PER_EVENT; ++i)
    {
        if (s_callbacks[p_evt_num].callbacks[i] == p_cb)
        {
            s_callbacks[p_evt_num].callbacks[i] = 0;
            s_callbacks[p_evt_num].num_callbacks--;
            
            callbacks_left = (s_callbacks[p_evt_num].num_callbacks == 0 ? false : true);
        }
    }
    
#if HAL_GPIOTE_MAX_CB_PER_EVENT > 2
#error Functionality for maintaning a callback list with length > 2 must be implemented
#endif /* HAL_GPIOTE_MAX_CB_PER_EVENT > 2 */
    // Cleaning up callback list: have to make sure valid callback is at front of the list
    if (s_callbacks[p_evt_num].num_callbacks > 0 && s_callbacks[p_evt_num].callbacks[0] == 0)
    {
        s_callbacks[p_evt_num].callbacks[0] = s_callbacks[p_evt_num].callbacks[1];
    }
    
    if (callbacks_left)
    {
        // Not disabling interrupts if there are still callbacks in list
        return NRF_SUCCESS;
    }

    // There are no callbacks left in this list: disable interrupts
    switch (p_evt_num)
    {
        case hal_gpiote_evt_0:
            NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_IN0_Msk;
            break;
        case hal_gpiote_evt_1:
            NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_IN1_Msk;
            break;
        case hal_gpiote_evt_2:
            NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_IN2_Msk;
            break;
        case hal_gpiote_evt_3:
            NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_IN3_Msk;
            break;
        case hal_gpiote_evt_port:
            NRF_GPIOTE->INTENCLR = GPIOTE_INTENSET_PORT_Msk;
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    return NRF_SUCCESS;
}
