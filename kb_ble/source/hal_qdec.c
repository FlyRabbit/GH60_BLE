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
 
#include "hal_qdec.h"

#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

static void (*s_qdec_int_callback)(hal_qdec_event_t event_type) = 0;

uint32_t hal_qdec_init(const hal_qdec_init_t* params)
{
    s_qdec_int_callback = params->qdec_int_callback;
     
    NRF_QDEC->DBFEN     = params->dbfen;
    NRF_QDEC->SAMPLEPER = (params->sampleper << QDEC_SAMPLEPER_SAMPLEPER_Pos);
    NRF_QDEC->REPORTPER = (params->reportper << QDEC_REPORTPER_REPORTPER_Pos);
    NRF_QDEC->LEDPRE    = params->ledpre;
    NRF_QDEC->PSELA     = params->psela;
    NRF_QDEC->PSELB     = params->pselb;     
    NRF_QDEC->PSELLED   = params->pselled;

    return NRF_SUCCESS;
}

void hal_qdec_start(void)
{
    NRF_QDEC->ENABLE      = (QDEC_ENABLE_ENABLE_Enabled << QDEC_ENABLE_ENABLE_Pos);
    NRF_QDEC->TASKS_START = 1;
}

void hal_qdec_stop(void)
{
    NRF_QDEC->TASKS_STOP = 1;
    NRF_QDEC->ENABLE     = (QDEC_ENABLE_ENABLE_Disabled << QDEC_ENABLE_ENABLE_Pos);
}

uint32_t hal_qdec_int_enable(uint32_t intenset)
{
    uint32_t err_code;
    uint8_t softdevice_enabled = false;
        
    if (s_qdec_int_callback == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
#ifdef SOFTDEVICE_PRESENT       
    err_code = sd_softdevice_is_enabled(&softdevice_enabled);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
#else
#ifndef SOFTDEVICE_NOT_PRESENT
#error Please define either SOFTDEVICE_PRESENT or SOFTDEVICE_NOT_PRESENT to state if a Softdevice is present or not.
#endif /* SOFTDEVICE_NOT_PRESENT */
#endif /* SOFTDEVICE_PRESENT */    
    
    if (softdevice_enabled)
    {
        err_code = sd_nvic_ClearPendingIRQ(QDEC_IRQn);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        err_code = sd_nvic_SetPriority(QDEC_IRQn, NRF_APP_PRIORITY_LOW);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
        
        err_code = sd_nvic_EnableIRQ(QDEC_IRQn);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {       
        NVIC_SetPriority(QDEC_IRQn, NRF_APP_PRIORITY_LOW);
        NVIC_ClearPendingIRQ(QDEC_IRQn);
        NVIC_EnableIRQ(QDEC_IRQn);
    }
    intenset &= (QDEC_INTENSET_SAMPLERDY_Msk | 
                 QDEC_INTENSET_REPORTRDY_Msk |
                 QDEC_INTENSET_ACCOF_Msk);
    
    NRF_QDEC->INTENSET         = intenset;
    NRF_QDEC->EVENTS_SAMPLERDY = 0;
    NRF_QDEC->EVENTS_REPORTRDY = 0;    
    NRF_QDEC->EVENTS_ACCOF     = 0; 
    
    return NRF_SUCCESS;
}

void hal_qdec_int_disable(uint32_t intenclr)
{
    intenclr &= (QDEC_INTENCLR_SAMPLERDY_Msk | 
                 QDEC_INTENCLR_REPORTRDY_Msk |
                 QDEC_INTENCLR_ACCOF_Msk);
    
    NRF_QDEC->INTENCLR = intenclr;
}

int32_t hal_qdec_accread(void)
{
    NRF_QDEC->TASKS_READCLRACC = 1;
    return (int32_t) NRF_QDEC->ACCREAD;   
}

void QDEC_IRQHandler(void)
{
    hal_qdec_event_t event = hal_qdec_event_void;
    
    if (NRF_QDEC->EVENTS_REPORTRDY)
    {                 
        NRF_QDEC->EVENTS_REPORTRDY = 0; 
        event = hal_qdec_event_reportrdy;
    }
    else if (NRF_QDEC->EVENTS_SAMPLERDY)
    {
        NRF_QDEC->EVENTS_SAMPLERDY = 0;
        event = hal_qdec_event_samplerdy;
    }
    else if (NRF_QDEC->EVENTS_ACCOF)
    {
        NRF_QDEC->EVENTS_ACCOF = 0;
        event = hal_qdec_event_accof;
    }
    
    s_qdec_int_callback(event);
}
