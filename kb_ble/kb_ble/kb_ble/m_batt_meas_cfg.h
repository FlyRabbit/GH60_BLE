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
 * @ingroup modules_batt_meas
 * @brief Battery module configuration.
 *
 * @details This file contains project-specific details related to battery measurement.
 */
#ifndef __M_BATT_MEAS_CFG_H__
#define __M_BATT_MEAS_CFG_H__

#define M_BATT_MEAS_DEFAULT_POLL_INTERVAL 1000 // [ms]

#define M_BATT_MEAS_TYPE_COINCELL     1
#define M_BATT_MEAS_TYPE_ALKALINE_AAA_2x_SERIES 2
#define M_BATT_MEAS_TYPE_ALKALINE_AAA 3

#define M_BATT_MEAS_BATTERY_TYPE M_BATT_MEAS_TYPE_ALKALINE_AAA_2x_SERIES /** The battery type used */

#if (M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_ALKALINE_AAA)
    #define M_BATT_MEAS_MIN_LEVEL 800  /** Which voltage is considered to be 0% battery level [mV] */
    #define M_BATT_MEAS_MAX_LEVEL 1600 /** Which voltage is considered to be 100% battery level [mV] */
#elif (M_BATT_MEAS_BATTERY_TYPE == M_BATT_MEAS_TYPE_ALKALINE_AAA_2x_SERIES)
    #define M_BATT_MEAS_MIN_LEVEL 3080  /** Which voltage is considered to be 0% battery level [mV] */
    #define M_BATT_MEAS_MAX_LEVEL 3450 /** Which voltage is considered to be 100% battery level [mV] */
#else
    #define M_BATT_MEAS_MIN_LEVEL 900  /** Which voltage is considered to be 0% battery level [mV] */
    #define M_BATT_MEAS_MAX_LEVEL 3100 /** Which voltage is considered to be 100% battery level [mV] */

#endif


#endif /* __M_BATT_MEAS_CFG_H__ */
