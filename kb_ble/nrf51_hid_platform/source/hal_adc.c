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
 
#include "hal_adc.h"
#include "nrf_sdm.h"

uint32_t hal_adc_init(const hal_adc_init_t* params)
{
    NRF_ADC->ENABLE = 1;
    NRF_ADC->CONFIG = ((params->res       << ADC_CONFIG_RES_Pos)    |
                       (params->inpsel    << ADC_CONFIG_INPSEL_Pos) |
                       (params->refsel    << ADC_CONFIG_REFSEL_Pos) |
                       (params->psel      << ADC_CONFIG_PSEL_Pos)   | 
                       (params->extrefsel << ADC_CONFIG_EXTREFSEL_Pos));
    NRF_ADC->ENABLE = 0;
    return NRF_SUCCESS;
}

uint16_t hal_adc_convert_single(void)
{
    int16_t val;
    NRF_ADC->ENABLE = 1;
    NRF_ADC->TASKS_START = 1;
    while (NRF_ADC->EVENTS_END == 0)
    {
        
    }
		    NRF_ADC->EVENTS_END = 0;
    val = (uint16_t) NRF_ADC->RESULT;
    NRF_ADC->TASKS_STOP = 1;
    NRF_ADC->ENABLE = 0;
    return val;
}
