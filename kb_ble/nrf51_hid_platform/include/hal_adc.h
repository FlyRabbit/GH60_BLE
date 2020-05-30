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

/** @file 
 *
 * @defgroup hal_adc
 * @{
 * @ingroup nrfready_hal
 * @brief ADC hardware abstraction layer.
 *
 * @details Simple ADC HAL implementation. Only geared towards single conversions "now and again". 
 *          For more complex behaviour and continous conversions, expand using the PPI.
 */
#ifndef __HAL_ADC_H__
#define __HAL_ADC_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf51_bitfields.h"

typedef enum
{
    hal_adc_res_8bit  = ADC_CONFIG_RES_8bit,
    hal_adc_res_9bit  = ADC_CONFIG_RES_9bit,
    hal_adc_res_10bit = ADC_CONFIG_RES_10bit
} hal_adc_res_t;

typedef enum
{
    hal_adc_inp_analog_no_pre        = ADC_CONFIG_INPSEL_AnalogInputNoPrescaling,
    hal_adc_inp_analog_twothirds_pre = ADC_CONFIG_INPSEL_AnalogInputTwoThirdsPrescaling,
    hal_adc_inp_analog_onethird_pre  = ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling,
    hal_adc_inp_vdd_twothirds_pre    = ADC_CONFIG_INPSEL_SupplyTwoThirdsPrescaling,
    hal_adc_inp_vdd_onethird_pre     = ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling
} hal_adc_inp_sel_t;

typedef enum
{
    hal_adc_ref_vbg              = ADC_CONFIG_REFSEL_VBG,
    hal_adc_ref_ext              = ADC_CONFIG_REFSEL_External,
    hal_adc_ref_vdd_onehalf_pre  = ADC_CONFIG_REFSEL_SupplyOneHalfPrescaling,
    hal_adc_ref_vdd_onethird_pre = ADC_CONFIG_REFSEL_SupplyOneThirdPrescaling
} hal_adc_ref_sel_t;

typedef enum
{
    hal_adc_pin_disabled = ADC_CONFIG_PSEL_Disabled,
    hal_adc_pin_ain0     = ADC_CONFIG_PSEL_AnalogInput0, /*!< Use analog input 0 as analog input. */
    hal_adc_pin_ain1     = ADC_CONFIG_PSEL_AnalogInput1, /*!< Use analog input 1 as analog input. */
    hal_adc_pin_ain2     = ADC_CONFIG_PSEL_AnalogInput2, /*!< Use analog input 2 as analog input. */
    hal_adc_pin_ain3     = ADC_CONFIG_PSEL_AnalogInput3, /*!< Use analog input 3 as analog input. */
    hal_adc_pin_ain4     = ADC_CONFIG_PSEL_AnalogInput4, /*!< Use analog input 4 as analog input. */
    hal_adc_pin_ain5     = ADC_CONFIG_PSEL_AnalogInput5, /*!< Use analog input 5 as analog input. */
    hal_adc_pin_ain6     = ADC_CONFIG_PSEL_AnalogInput6, /*!< Use analog input 6 as analog input. */
    hal_adc_pin_ain7     = ADC_CONFIG_PSEL_AnalogInput7  /*!< Use analog input 7 as analog input. */
} hal_adc_pin_sel_t;

typedef enum
{    
    hal_adc_ext_none  = ADC_CONFIG_EXTREFSEL_None, /*!< Analog external reference inputs disabled. */
    hal_adc_ext_aref0 = ADC_CONFIG_EXTREFSEL_AnalogReference0, /*!< Use analog reference 0 as reference. */
    hal_adc_ext_aref1 = ADC_CONFIG_EXTREFSEL_AnalogReference1  /*!< Use analog reference 1 as reference. */
} hal_adc_extref_sel_t;

typedef struct
{
    hal_adc_res_t        res;
    hal_adc_inp_sel_t    inpsel;
    hal_adc_ref_sel_t    refsel;
    hal_adc_pin_sel_t    psel;
    hal_adc_extref_sel_t extrefsel;
} hal_adc_init_t;

/**@brief ADC initialization.
 * 
 * @param[in] params Configuration parameters
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t hal_adc_init(const hal_adc_init_t* params);

/**@brief Starts conversion and waits until conversion is completed.
 *
 * @return Converted analog value.
 */
uint16_t hal_adc_convert_single(void);

#endif /* __HAL_ADC_H__ */

/** @} */
