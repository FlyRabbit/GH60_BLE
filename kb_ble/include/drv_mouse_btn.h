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
 * @defgroup drivers_mouse_btn
 * @{
 * @ingroup nrfready_drivers
 * @brief Mouse buttons driver.
 *
 * @details This header defines prototypes for keyboard matrix driver functions.
 */
#ifndef __DRV_MOUSE_BTN_H__
#define __DRV_MOUSE_BTN_H__

#define DRV_MOUSE_BTN_LEFT_PIN       IO_BTN_LEFT_PIN
#define DRV_MOUSE_BTN_RIGHT_PIN      IO_BTN_RIGHT_PIN
#define DRV_MOUSE_BTN_MIDDLE_PIN     IO_BTN_MIDDLE_PIN
#define DRV_MOUSE_BTN_SIDE_LEFT_PIN  IO_BTN_SIDE_LEFT_PIN
#define DRV_MOUSE_BTN_SIDE_RIGHT_PIN IO_BTN_SIDE_RIGHT_PIN

#define DRV_MOUSE_BTN_LEFT       IO_BTN_LEFT_MSK
#define DRV_MOUSE_BTN_RIGHT      IO_BTN_RIGHT_MSK
#define DRV_MOUSE_BTN_MIDDLE     IO_BTN_MIDDLE_MSK
#define DRV_MOUSE_BTN_SIDE_LEFT  IO_BTN_SIDE_LEFT_MSK
#define DRV_MOUSE_BTN_SIDE_RIGHT IO_BTN_SIDE_RIGHT_MSK

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    drv_mouse_btn_hilevel = 0,
    drv_mouse_btn_lolevel = 1,
} btn_int_polarity_t;

/**@brief Mouse button initialization.
 *
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t drv_mouse_btn_init(void);

/**@brief Enable pin sense for button pins
 *
 * @param[in] p_enable if true, enable pin sense
 */
void drv_mouse_btn_sense_enable(bool p_enable);

/**@brief Read button state.
 *
 * @return State bitmask. 1 if pressed, 0 otherwise.
 * @retval DRV_MOUSE_BTN_LEFT
 * @retval DRV_MOUSE_BTN_RIGHT
 * @retval DRV_MOUSE_BTN_MIDDLE
 * @retval DRV_MOUSE_BTN_SIDE_LEFT
 * @retval DRV_MOUSE_BTN_SIDE_RIGHT
 */
uint32_t drv_mouse_btn_read(void);

/**@brief Prepare for sleep and subsequent wakeup.
 */
bool drv_mouse_btn_wakeup_prepare(void);

#endif /* __DRV_MOUSE_BTN_H__ */

/** @} */
