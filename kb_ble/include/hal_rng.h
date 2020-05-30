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
 * @defgroup hal_rng
 * @{
 * @ingroup nrfready_hal
 * @brief Random number generator (RNG) hardware abstraction layer.
 *
 */
#ifndef __HAL_RNG_H__
#define __HAL_RNG_H__

#include <stdint.h>

/**@brief Fill an array with random numbers
 *
 * @param[out] p_buffer Buffer to store the random values
 * @param[in]  p_count  Number of random bytes to put buffer
 * @return 
 * @retval NRF_SUCCESS
 * @return NRF_ERROR_INVALID_PARAM
 */
uint32_t hal_rng_vector_get(uint8_t* p_buffer, uint8_t p_count);

#endif /* __HAL_RNG_H__ */

/** @} */
