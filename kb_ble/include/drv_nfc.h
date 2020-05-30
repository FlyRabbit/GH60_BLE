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
 * @defgroup drivers_nfc
 * @{
 * @ingroup nrfready_drivers
 * @brief NFC driver.
 *
 * @details This driver deals with the NFC hardware.
 */
#ifndef __DRV_NFC_H__
#define __DRV_NFC_H__

#include <stdint.h>

typedef enum {
    NFC_REPLY_ACK,
    NFC_REPLY_NACK,
    NFC_REPLY_DATA
} nfc_reply_type_t;

/**@brief NFC driver initalization
 *
 * @param[in] callback Function to handle NFC the protocol 
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t drv_nfc_init(uint32_t (*callback)(uint8_t *resp));

/**@brief NFC hibernate.
 *
 * @details Puts the NFC in a low power hibernation state.
 */
uint32_t drv_nfc_init_hibernate(void);

/**@brief Power on and start card emulation.
 */
uint32_t drv_nfc_start(void);

/**@brief Power off.
 */
uint32_t drv_nfc_stop(void);

/**@brief Power on NFC hardware.
 */
void drv_nfc_power_on(void);

/**@brief Power on NFC hardware.
 */
void drv_nfc_power_off(void);

/**@brief Schedule nfc hardware event
 *
 * @details This function must be called from NFC interrupt or polling mechanism.
 */
void drv_nfc_evt_schedule(void);

/**@brief Send NFC reply to the reader.
 *
 * @param[in] reply_type Reply type NFC_REPLY_ACK, NFC_REPLY_NACK or NFC_REPLY_DATA.
 * @param[in] dat        Pointer to the data.
 * @param[in] size       Size of data in bytes.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM 
 */
void drv_nfc_reply(nfc_reply_type_t reply_type, uint8_t *dat, uint8_t size);


#endif

/** @} */
