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

#ifndef __HAL_QDEC_H__
#define __HAL_QDEC_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf51_bitfields.h"

/** @file
* @brief Quadrature decoder driver.
*
* @note To save power, only use the QDEC when needed. I.e. using a GPIOTE-triggered encoder line interrupt.
*
* @defgroup hal_qdec
* @{
* @ingroup nrfready_hal
* @brief 
*/

typedef enum
{
    hal_qdec_sampleper_128us   = QDEC_SAMPLEPER_SAMPLEPER_128us,   /*!< 128us sample period. */
    hal_qdec_sampleper_256us   = QDEC_SAMPLEPER_SAMPLEPER_256us,   /*!< 256us sample period. */
    hal_qdec_sampleper_512us   = QDEC_SAMPLEPER_SAMPLEPER_512us,   /*!< 512us sample period. */
    hal_qdec_sampleper_1024us  = QDEC_SAMPLEPER_SAMPLEPER_1024us,  /*!< 1024us sample period. */
    hal_qdec_sampleper_2048us  = QDEC_SAMPLEPER_SAMPLEPER_2048us,  /*!< 2048us sample period. */
    hal_qdec_sampleper_4096us  = QDEC_SAMPLEPER_SAMPLEPER_4096us,  /*!< 4096us sample period. */
    hal_qdec_sampleper_8192us  = QDEC_SAMPLEPER_SAMPLEPER_8192us,  /*!< 8192us sample period. */
    hal_qdec_sampleper_16384us = QDEC_SAMPLEPER_SAMPLEPER_16384us, /*!< 16384us sample period. */
} hal_qdec_sampleper_t;

typedef enum
{
    hal_qdec_reportper_10smpl  = QDEC_REPORTPER_REPORTPER_10Smpl, /*!< 10 samples per report. */
    hal_qdec_reportper_40smpl  = QDEC_REPORTPER_REPORTPER_40Smpl, /*!< 40 samples per report. */
    hal_qdec_reportper_80smpl  = QDEC_REPORTPER_REPORTPER_80Smpl, /*!< 80 samples per report. */
    hal_qdec_reportper_120smpl = QDEC_REPORTPER_REPORTPER_120Smpl, /*!< 120 samples per report. */
    hal_qdec_reportper_160smpl = QDEC_REPORTPER_REPORTPER_160Smpl, /*!< 160 samples per report. */
    hal_qdec_reportper_200smpl = QDEC_REPORTPER_REPORTPER_200Smpl, /*!< 200 samples per report. */
    hal_qdec_reportper_240smpl = QDEC_REPORTPER_REPORTPER_240Smpl, /*!< 240 samples per report. */
    hal_qdec_reportper_280smpl = QDEC_REPORTPER_REPORTPER_280Smpl, /*!< 280 samples per report. */
} hal_qdec_reportper_t;

typedef enum
{
    hal_qdec_led_pol_active_low = QDEC_LEDPOL_LEDPOL_ActiveLow,
    hal_qdec_led_pol_active_high = QDEC_LEDPOL_LEDPOL_ActiveHigh,
} hal_qdec_led_pol_t;

typedef enum
{
    hal_qdec_event_void,
    hal_qdec_event_samplerdy,
    hal_qdec_event_reportrdy,
    hal_qdec_event_accof
} hal_qdec_event_t;

typedef struct
{
  void (*qdec_int_callback)(hal_qdec_event_t event_type);
  hal_qdec_reportper_t reportper;
  hal_qdec_sampleper_t sampleper;      
  uint32_t             psela;
  uint32_t             pselb;
  uint32_t             pselled;
  uint32_t             ledpre;
  bool                 dbfen;    
} hal_qdec_init_t;

/**@brief QDEC initialization.
 *
 * @param[in] params
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t hal_qdec_init(const hal_qdec_init_t* params);

/**@brief Interrupt enable.
 *
 * @param[in] intenset Bitmask of interrupts to enable
 * @arg QDEC_INTENSET_ACCOF_Msk
 * @arg QDEC_INTENSET_REPORTRDY_Msk
 * @arg QDEC_INTENSET_SAMPLERDY_Msk
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM if no interrupt callback given in hal_qdec_init()
 */
uint32_t hal_qdec_int_enable(uint32_t intenset);

/**@brief Interrupt disable.
 *
 * @param[in] intenset Bitmask of interrupts to enable
 * @arg QDEC_INTENCLR_ACCOF_Msk
 * @arg QDEC_INTENCLR_REPORTRDY_Msk
 * @arg QDEC_INTENCLR_SAMPLERDY_Msk
 */
void hal_qdec_int_disable(uint32_t intenclr);

/**@brief Read accumulated samples.
 *
 * @return accread register contents
 */
int32_t hal_qdec_accread(void);

/**@brief Starts sampling. Note: started automatically by hal_qdec_init()
 */
void hal_qdec_start(void);

/**@brief Stops sampling.
 */
void hal_qdec_stop(void);

#endif /* __HAL_QDEC_H__ */

/** @} */
