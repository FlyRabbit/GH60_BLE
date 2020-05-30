 /* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic
 * Semiconductor ASA.Terms and conditions of usage are described in detail
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef __DRV_MSCROLL_H__
#define __DRV_MSCROLL_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf51_bitfields.h"

#define DRV_MOUSE_SCROLL_POLL_INTERVAL 3

/** @file
* @brief Scroll wheel driver.
*
*
* @defgroup drivers_mouse_scroll
* @{
* @ingroup nrfready_drivers
* @brief 
*/

/**@brief Scroll wheel initialization.
 *
 * @param[in] scroll_event_cb Scroll event callback. NOTE: might be called from interrupt context!
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t drv_mscroll_init(void (*scroll_event_cb)(int32_t));

/**@brief Run this function when a pin change occurs on scroll pins */
void drv_mouse_scroll_run(void);

/**@brief Utility function to see if scroll pins has set off PORT GPIOTE interrupt
 *
 * @return true scroll pins are in a state to trigger PORT interrupt
 */
bool drv_mscroll_int_set(void);

/**@brief Enable scroll wheel.
 *
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t drv_mscroll_enable(void);

/**@brief Enable scroll wheel.
 */
void drv_mscroll_disable(void);

/**@brief Prepare for sleep and subsequent wakeup.
 */
bool drv_mscroll_wakeup_prepare(void);

#endif /* __DRV_MSCROLL_H__ */

/** @} */
