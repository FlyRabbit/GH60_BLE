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

#include <string.h>

#include "hal_spi.h"
#include "nrf_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"

#define TIMEOUT_COUNTER 0x3000UL  /*!< timeout for getting rx bytes from slave */

typedef struct
{
    uint32_t sck_pin;
    uint32_t mosi_pin;
    uint32_t miso_pin;
    uint32_t ss_pin;
} hal_spi_pins_t;

static hal_spi_pins_t spi0_pins = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
static hal_spi_pins_t spi1_pins = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

uint32_t* hal_spi_init(const hal_spi_init_t* params)
{
    uint32_t config_mode;
    uint32_t *spi_base_address_pointer = (uint32_t *)0;
    NRF_SPI_Type *spi_base_address = (SPI0 == params->module)? NRF_SPI0 : (NRF_SPI_Type *)NRF_SPI1;       

    if(params->module == SPI0)
    {   
        spi0_pins.ss_pin   = params->ss_pin;
        spi0_pins.sck_pin  = params->sck_pin;
        spi0_pins.mosi_pin = params->mosi_pin;
        spi0_pins.miso_pin = params->miso_pin;
    }
    else
    {
        spi1_pins.ss_pin   = params->ss_pin;
        spi1_pins.sck_pin  = params->sck_pin;
        spi1_pins.mosi_pin = params->mosi_pin;
        spi1_pins.miso_pin = params->miso_pin;
    }
    
    /* Configure GPIO pins used for pselsck, pselmosi, pselmiso and pselss for SPI0*/
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[params->sck_pin] = 
            (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[params->mosi_pin] = 
            (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[params->miso_pin] = 
            (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)   |
            (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) |
            (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  |
            (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) |
            (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    /* Configure GPIO on pin SPI_PSELSS to use as Slave select which needs to be controlled by software independently */
    /*lint -e{845} // A zero has been given as right argument to operator '|'" */
    NRF_GPIO->PIN_CNF[params->ss_pin] = 
                       (GPIO_PIN_CNF_DIR_Output       << GPIO_PIN_CNF_DIR_Pos)   |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled      << GPIO_PIN_CNF_PULL_Pos)  |
                       (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) |
                       (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos);

    /* Configure pins, frequency and mode */
    spi_base_address->PSELSCK  = params->sck_pin;
    spi_base_address->PSELMOSI = params->mosi_pin;
    spi_base_address->PSELMISO = params->miso_pin;
    nrf_gpio_pin_set(params->ss_pin); /* disable Set slave select (inactive high) */
    spi_base_address->FREQUENCY = (0x02000000UL << (uint32_t) params->frequency);

    switch (params->mode)
    {
    /*lint -e845 -save // A zero has been given as right argument to operator '!'" */
    case SPI_MODE0:
        config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
        break;
    case SPI_MODE1:
        config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
        break;
    case SPI_MODE2:
        config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
        break;
    case SPI_MODE3:
        config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
        break;
    default:
        config_mode = 0;
        break;
    /*lint -restore */
    }
    if (params->lsb_first)
    {
      /*lint -e{845} // A zero has been given as right argument to operator '|'" */
      spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos));
    }
    else
    {
      /*lint -e{845} // A zero has been given as right argument to operator '|'" */
      spi_base_address->CONFIG = (config_mode | (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos));
    }

    spi_base_address->EVENTS_READY = 0U;

    /* Enable */
    spi_base_address->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);

    spi_base_address_pointer = (uint32_t *)spi_base_address;

    return spi_base_address_pointer;
}

uint32_t hal_spi_enable(SPIModuleNumber module_number)
{
    switch (module_number)
    {
        case SPI0:
            NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
            break;
        case SPI1:
            NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    return NRF_SUCCESS;
}

uint32_t hal_spi_disable(SPIModuleNumber module_number)
{
    switch (module_number)
    {
        case SPI0:
            NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
            break;
        case SPI1:
            NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }
    
    return NRF_SUCCESS;
}

bool hal_spi_tx_rx(uint32_t *spi_base_address, uint16_t transfer_size, const uint8_t *tx_data, uint8_t *rx_data)
{
    uint32_t counter = 0;
    uint16_t number_of_txd_bytes = 0;
    uint32_t SEL_SS_PINOUT;
    /*lint -e{826} //Are too small pointer conversion */
    NRF_SPI_Type *spi_base = (NRF_SPI_Type *)spi_base_address;

    SEL_SS_PINOUT = spi_base == NRF_SPI0 ? spi0_pins.ss_pin : spi1_pins.ss_pin;

    /* enable slave (slave select active low) */
    nrf_gpio_pin_clear(SEL_SS_PINOUT);

    while(number_of_txd_bytes < transfer_size)
    {
        spi_base->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);

        /* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
        while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
        {
            counter++;
        }

        if (counter == TIMEOUT_COUNTER)
        {
            /* timed out, disable slave (slave select active low) and return with error */
            nrf_gpio_pin_set(SEL_SS_PINOUT);
            return false;
        }
        else
        {   /* clear the event to be ready to receive next messages */
            spi_base->EVENTS_READY = 0U;
        }

        rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
        number_of_txd_bytes++;
    };

    /* disable slave (slave select active low) */
    nrf_gpio_pin_set(SEL_SS_PINOUT);

    return true;
}

static void hal_spi_tx_rx_bytes(NRF_SPI_Type* spi_base, uint32_t transfer_size, const uint8_t* tx_data, uint8_t* rx_data)
{
	uint32_t number_of_txd_bytes = 0;
	uint32_t counter = 0;
	
	while(number_of_txd_bytes < transfer_size)
	{
			spi_base->TXD = (uint32_t)(tx_data[number_of_txd_bytes]);
			counter = 0;

			/* Wait for the transaction complete or timeout (about 10ms - 20 ms) */
			while ((spi_base->EVENTS_READY == 0U) && (counter < TIMEOUT_COUNTER))
			{
					counter++;
			}

			if (counter >= TIMEOUT_COUNTER)
			{
					/* timed out, return */
					return;
			}
			else
			{   /* clear the event to be ready to receive next messages */
					spi_base->EVENTS_READY = 0U;
			}

			rx_data[number_of_txd_bytes] = (uint8_t)spi_base->RXD;
			number_of_txd_bytes++;
	};
}

uint32_t hal_spi_tx_rx_var(uint32_t *spi_base_address, uint32_t max_len, const char* format, const uint8_t *tx_data, uint8_t *rx_data)
{
	uint32_t i;
	NRF_SPI_Type* spi_base;
	uint32_t SEL_SS_PINOUT;
	uint32_t ret = NRF_SUCCESS;
	uint32_t next_len;
	
	spi_base = (NRF_SPI_Type *)spi_base_address;
	SEL_SS_PINOUT = spi_base == NRF_SPI0 ? spi0_pins.ss_pin : spi1_pins.ss_pin;
	
	nrf_gpio_pin_clear(SEL_SS_PINOUT);
	
	for (i = 0; i < strlen(format); ++i)
	{
		switch (format[i])
		{
			case '%':				
				break;
			case 'd':			
				hal_spi_tx_rx_bytes(spi_base, 1, tx_data, rx_data);
				next_len = *rx_data;
				tx_data += 1;
				rx_data += 1;				
				if ((next_len > 0) && (next_len <= max_len))
				{
					hal_spi_tx_rx_bytes(spi_base, next_len, tx_data, rx_data);
                    max_len -= next_len;
				}
                else
                {
                    ret = NRF_ERROR_DATA_SIZE;
                }
				break;
			case 'c':
                if (max_len >= 1)
                {
                    hal_spi_tx_rx_bytes(spi_base, 1, tx_data, rx_data);
                    max_len -= 1;
                    tx_data += 1;
                    rx_data += 1;
                }
                else
                {
                    ret = NRF_ERROR_DATA_SIZE;
                }                    
				break;
			default:
				ret = NRF_ERROR_INVALID_PARAM;
				break;
		}
	}
	
	nrf_gpio_pin_set(SEL_SS_PINOUT);
	return ret;
}
