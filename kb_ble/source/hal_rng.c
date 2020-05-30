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
 
#include "hal_rng.h"

#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"

uint32_t hal_rng_vector_get(uint8_t* p_buffer, uint8_t p_count)
{
    int i;
    uint8_t softdevice_enabled = 0;
    
#ifdef SOFTDEVICE_PRESENT    
    {
        uint32_t err_code;
        
        err_code = sd_softdevice_is_enabled(&softdevice_enabled);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
#else
#ifndef SOFTDEVICE_NOT_PRESENT
#error "Please define either SOFTDEVICE_PRESENT or SOFTDEVICE_NOT_PRESENT to state if a Softdevice is present or not."
#endif /* SOFTDEVICE_NOT_PRESENT */
#endif /* SOFTDEVICE_PRESENT */
    
    if (softdevice_enabled != 0)
    {
        return sd_rand_application_vector_get(p_buffer, p_count);
    }
    
    
    NRF_RNG->TASKS_START = 1;
    for (i = 0; i < p_count; ++i)
    {
        NRF_RNG->EVENTS_VALRDY = 0;
        while (NRF_RNG->EVENTS_VALRDY == 0)
        {
            // Wait for value to become ready
        }
        
        p_buffer[i] = (uint8_t) NRF_RNG->VALUE;
    }
    NRF_RNG->TASKS_STOP = 1;
    
    return NRF_SUCCESS;
}
