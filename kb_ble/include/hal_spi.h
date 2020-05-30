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

#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#include <stdbool.h>
#include <stdint.h>

/** @file
* @brief Software controlled SPI Master driver.
*
*
* @defgroup hal_spi
* @{
* @ingroup nrfready_hal
* @brief Software controlled SPI Master driver.
*
* Supported features:
* - Operate two SPI masters independently or in parallel.
* - Transmit and Receive given size of data through SPI.
* - configure each SPI module separately through @ref spi_master_init.
*/

/**
 *  SPI master operating frequency
 */
typedef enum
{
    Freq_125Kbps = 0,        /*!< drive SClk with frequency 125Kbps */
    Freq_250Kbps,            /*!< drive SClk with frequency 250Kbps */
    Freq_500Kbps,            /*!< drive SClk with frequency 500Kbps */
    Freq_1Mbps,              /*!< drive SClk with frequency 1Mbps */
    Freq_2Mbps,              /*!< drive SClk with frequency 2Mbps */
    Freq_4Mbps,              /*!< drive SClk with frequency 4Mbps */
    Freq_8Mbps               /*!< drive SClk with frequency 8Mbps */
} SPIFrequency_t;

/**
 *  SPI master module number
 */
typedef enum
{
    SPI0 = 0,               /*!< SPI module 0 */
    SPI1                    /*!< SPI module 1 */
} SPIModuleNumber;

/**
 *  SPI mode
 */
typedef enum
{
    //------------------------Clock polarity 0, Clock starts with level 0-------------------------------------------
    SPI_MODE0 = 0,          /*!< Sample data at rising edge of clock and shift serial data at falling edge */
    SPI_MODE1,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    //------------------------Clock polarity 1, Clock starts with level 1-------------------------------------------
    SPI_MODE2,              /*!< sample data at falling edge of clock and shift serial data at rising edge */
    SPI_MODE3               /*!< Sample data at rising edge of clock and shift serial data at falling edge */
} SPIMode;

typedef struct
{
    SPIModuleNumber module;
    SPIMode         mode;
    SPIFrequency_t  frequency;
    bool            lsb_first;
    uint32_t        sck_pin;
    uint32_t        mosi_pin;
    uint32_t        miso_pin;
    uint32_t        ss_pin;
} hal_spi_init_t;


/**
 * Initializes given SPI master with given configuration.
 *
 * After initializing the given SPI master with given configuration, this function also test if the
 * SPI slave is responding with the configurations by transmitting few test bytes. If the slave did not
 * respond then error is returned and contents of the rx_data are invalid.
 *
 * @param[in] params 
 * @return
 * @retval pointer to direct physical address of the requested SPI module if init was successful
 * @retval 0, if either init failed or slave did not respond to the test transfer
 */
uint32_t* hal_spi_init(const hal_spi_init_t* params);

/**
 * Enables given SPI master. Note that spi_master_init automatically enables the SPI.
 * 
 * @param[in] module_number SPI master to enable.
 * @return
 * @retval NRF_SUCCESS upon successful enable.
 * @retval NRF_ERROR_INVALID_PARAM if invalid SPI number is used.
 */
uint32_t hal_spi_enable(SPIModuleNumber module_number);

/**
 * Disables given SPI master. No settings are altered.
 * 
 * @param[in] module_number SPI master to disable.
 * @return
 * @retval NRF_SUCCESS upon successful disable. 
 * @retval NRF_ERROR_INVALID_PARAM if invalid SPI number is used.
 */
uint32_t hal_spi_disable(SPIModuleNumber module_number);

/**
 * Transfer/Receive data over SPI bus.
 *
 * If TWI master detects even one NACK from the slave or timeout occurs, STOP condition is issued
 * and the function returns false.
 *
 * @note Make sure at least transfer_size number of bytes is allocated in tx_data/rx_data.
 *
 * @param spi_base_address  register base address of the selected SPI master module
 * @param transfer_size  number of bytes to transmit/receive over SPI master
 * @param tx_data pointer to the data that needs to be transmitted
 * @param rx_data pointer to the data that needs to be received
 * @return
 * @retval true if transmit/reveive of transfer_size were completed.
 * @retval false if transmit/reveive of transfer_size were not complete and tx_data/rx_data points to invalid data.
 */
bool hal_spi_tx_rx(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data);

/**
 * @brief Transfer/Receive variable length data over SPI bus.
 *
 * @param spi_base_address  register base address of the selected SPI master module
 * @param max_len  number of bytes to transmit/receive over SPI master
 * @param format  Specification of data to receive: Ex. "%c%c%d" transmit/receive 2 chars + x chars specified in the 3rd received byte
 * @param tx_data pointer to the data that needs to be transmitted
 * @param rx_data pointer to the data that needs to be received
 * @return
 * @retval NRF_SUCCESS 
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t hal_spi_tx_rx_var(uint32_t *spi_base_address, uint32_t max_len, const char* format, const uint8_t *tx_data, uint8_t *rx_data);

/**
 *@}
 **/
 
#endif /* __HAL_SPI_H__ */
