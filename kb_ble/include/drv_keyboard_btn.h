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
 * @defgroup drivers_keyboard_buttons
 * @{
 * @ingroup nrfready_drivers
 * @brief Keyboard button driver.
 *
 * @details This header defines prototypes for non-matrix keyboard button driver functions. I.e. pairing button
 */
#ifndef __DRV_KEYBOARD_BTN_H__
#define __DRV_KEYBOARD_BTN_H__

#include <stdbool.h>
#include <stdint.h>

/**@brief Button initialization.
 */
void drv_keyboard_btn_init(void);

/**@brief Configure button to generate sense interrupts on press
 *
 * @param[in] p_enable if true sense is enabled 
 */
void drv_keyboard_btn_sense_enable(bool p_enable);

/**@brief Read button state
 *
 * @return Button state bitmask. A set bit equals "button pressed".
 */
uint32_t drv_keyboard_btn_read_pin_state(void);

 /**@brief Prepares keyboard button to generate wakeup.
  */
void drv_keyboard_btn_wakeup_prepare(void);

#endif /* __DRV_KEYBOARD_BTN_H__ */

/** @} */
