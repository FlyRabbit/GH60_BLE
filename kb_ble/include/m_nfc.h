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
 * @defgroup modules_nfc
 * @{
 * @ingroup nrfready_modules
 * @brief NFC module.
 *
 * @details This module deals with the NFC transceiver.
 */
 #ifndef __M_NFC_H__
 #define __M_NFC_H__
 
 #include <stdbool.h>
 #include <stdint.h>

/**@brief NFC initalization
 *
 * @param[in] ble_addr        Local BLE address
 * @param[in] ble_eir_oob_key OOB key 
 * @param[in] ble_local_name  NULL terminated name string
 * @param[in] appearance      BLE appearance
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_nfc_init(uint8_t *ble_addr, uint8_t *ble_eir_oob_key, uint8_t *ble_local_name, uint8_t name_size, uint16_t appearance);

/**@brief Put NFC-chip to low power hibernation mode.
 *
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t m_nfc_init_hibernate(void);
 
/**@brief Sets the OOB key stored in the tag.
 *
 * @param[in] key 16-byte OOB key value
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t m_nfc_set_oob_key(uint8_t *key);

/**@brief Sets the BLE addr stored in the tag.
 *
 * @param[in] addr 6-byte address array
 * @return
 * @retval NRF_SUCCESS
 */
void m_nfc_set_ble_addr(uint8_t *addr);

/**@brief Function to se if NFC is enabled.
 *
 * @return
 * @retval true If NFC is enabled
 * @retval false If NGC is disabled
 */
bool m_nfc_is_enabled(void);

/**@brief Power on the NFC chip and enable tag emulation.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL 
 */
uint32_t m_nfc_enable(void);

/**@brief Power off the NFC chip.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL 
 */
uint32_t m_nfc_disable(void);

/**@brief Prepare for wakeup after sleep
 */
void m_nfc_wakeup_prepare(void);

/**@brief Check if the NFC module is present
 * @return
 * @retval TRUE if module detected
 * @retval FALSE otherwise
 */
bool m_nfc_module_present(void);
 
#endif /* __M_NFC_H__ */

/** @} */
