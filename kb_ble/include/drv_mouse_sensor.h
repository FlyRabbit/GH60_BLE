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
 * @defgroup drivers_mouse_sensor
 * @{
 * @ingroup nrfready_drivers
 * @brief Mouse sensor driver.
 *
 * @details This header defines prototypes for mouse sensor driver functions.
 */
#ifndef __DRV_MOUSE_H__
#define __DRV_MOUSE_H__

#include <stdbool.h>
#include <stdint.h>

/**@brief Mouse driver initialization.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_x
 */
uint32_t drv_mouse_sensor_init(void);

/**@brief Get interrupt line status
 *
 * @return true if sensor has set an interrupt line
 */
bool drv_mouse_sensor_int_get(void);

/**@brief Read movement deltas.
 *
 * @param[out] delta_x
 * @param[out] delta_y
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t drv_mouse_sensor_read(int16_t* delta_x, int16_t* delta_y);

/**@brief Enable pin sense for mouse sensor interrupt pin
 *
 * @param[in] enable if true, enable pin sense
 */
void drv_mouse_sensor_sense_enable(bool enable);

/**@brief Preform sensor readout, but discard data.
 */
void drv_mouse_sensor_dummy_read(void);

/**@brief Prepare for sleep and subsequent wakeup.
 */
bool drv_mouse_sensor_wakeup_prepare(void);

#endif /* __DRV_MOUSE_H__ */

/** @} */
