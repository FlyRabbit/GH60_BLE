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
 * @defgroup drivers_keyboard_matrix
 * @{
 * @ingroup nrfready_drivers
 * @brief Keyboard matrix driver.
 *
 * @details This header defines prototypes for keyboard matrix driver functions.
 */
#ifndef __DRV_KEYBOARD_MATRIX_H__
#define __DRV_KEYBOARD_MATRIX_H__

#include <stdbool.h>
#include <stdint.h>

#define KEYBOARD_MAX_NUM_OF_PRESSED_KEYS 6 //!< Maximum number of pressed keys kept in buffers

/**@brief Keyboard driver initialization.
 */
void drv_keyboard_init(void);

/**@brief Configure matrix to generate sense interrupts on key press
 *
 * @note When enabled, columns are pulled high. If not disabled when pressed, power consumption will rise.
 * @param[in] p_enable if true sense is enabled 
 */
void drv_keyboard_sense_enable(bool p_enable);

/**@brief Check if any keys are currently being pressed
 *
 * @return true if any key is pressed, false otherwise
 */
bool drv_keyboard_keys_held(void);

/**@brief Reads keyboard matrix state and stores pressed keys to an array.
 *
 * This function resolves keys from the matrix and finds their corresponding HID usage codes
 * If there are any ghost key conditions the packet will be discarded
 * @param pressed_keys Array holding pressed keys. Must be at least CHERRY8x16_MAX_NUM_OF_PRESSED_KEYS in size.
 * @param number_of_pressed_keys Pointer to variable where number of pressed keys will be stored.
 * @return 
 * @retval true If no keys were blocking each other.
 * @retval false If some keys were blocking each other o rno key is pressed.
 */
bool drv_keyboard_keymatrix_read(uint8_t *pressed_keys, uint8_t *number_of_pressed_keys);

/**@brief Remapping of any special keys present on the keyboard.
 *
 * @param[in] keys           Array of keys
 * @param[in] number_of_keys Length of key array
 * @return 
 * @retval true If fn-key is pressed
 * @retval false If fn-key is not pressed. 
 */
bool drv_keyboard_remap_special_keys(uint8_t *keys, uint8_t modifier_keys, uint8_t number_of_keys);

 /**@brief Prepares keyboard matrix to generate wakeup.
  */
void drv_keyboard_wakeup_prepare(void);

#endif /* __DRV_KEYBOARD_MATRIX_H__ */

/** @} */
