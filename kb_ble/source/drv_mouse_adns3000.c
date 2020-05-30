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
 
#include <stdbool.h>
#include <stdint.h>

#include "drv_mouse_sensor.h"

#include "io_cfg.h"
#include "hal_spi.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"


/*lint ++flb "Enter library region" */

#define ADNS3000_PRODUCT_ID (0x2AU) /*!< ADNS3000 product id */
#define ADNS3000_RESET_NUMBER (0x5AU) /*!< ADNS3000 reset code */

/* ADNS3000 register addresses */
#define REG_PROD_ID (0x00U) /*!< Product ID. Default value : 0x2A */
#define REG_REV_ID (0x01U) /*!< Revision ID. Default value : 0x00 */
#define REG_MOTION_ST (0x02U) /*!< Motion Status. Default value : 0x00 */
#define REG_DELTA_X (0x03U) /*!< Lower byte of Delta_X. Default value : 0x00 */
#define REG_DELTA_Y (0x04U) /*!< Lower byte of Delta_Y. Default value : 0x00 */
#define REG_SQUAL (0x05U) /*!< Squal Quality. Default value : 0x00 */
#define REG_SHUT_HI (0x06U) /*!< Shutter Open Time (Upper 8-bit). Default value : 0x00 */
#define REG_SHUT_LO (0x07U) /*!< Shutter Open Time (Lower 8-bit). Default value : 0x64 */
#define REG_PIX_MAX (0x08U) /*!< Maximum Pixel Value. Default value : 0xD0 */
#define REG_PIX_ACCUM (0x09U) /*!< Average Pixel Value. Default value : 0x80 */
#define REG_PIX_MIN (0x0AU) /*!< Minimum Pixel Value. Default value : 0x00 */
#define REG_PIX_GRAB (0x0BU) /*!< Pixel Grabber. Default value : 0x00 */
#define REG_DELTA_XY_HIGH (0x0CU) /*!< Upper 4 bits of Delta X and Y displacement. Default value : 0x00 */
#define REG_MOUSE_CTRL (0x0DU) /*!< Mouse Control. Default value : 0x01 */
#define REG_RUN_DOWNSHIFT (0x0EU) /*!< Run to Rest1 Time. Default value : 0x08 */
#define REG_REST1_PERIOD (0x0FU) /*!< Rest1 Period. Default value : 0x01 */
#define REG_REST1_DOWNSHIFT (0x10U) /*!< Rest1 to Rest2 Time. Default value : 0x1f */
#define REG_REST2_PERIOD (0x11U) /*!< Rest2 Period. Default value : 0x09 */
#define REG_REST2_DOWNSHIFT (0x12U) /*!< Rest2 to Rest3 Time. Default value : 0x2f */
#define REG_REST3_PERIOD (0x13U) /*!< Rest3 Period. Default value : 0x31 */
#define REG_PERFORMANCE (0x22U) /*!< Performance. Default value : 0x00 */
#define REG_RESET (0x3aU) /*!< Reset. Default value : 0x00 */
#define REG_NOT_REV_ID (0x3fU) /*!< Inverted Revision ID. Default value : 0xff */
#define REG_LED_CTRL (0x40U) /*!< LED Control. Default value : 0x00 */
#define REG_MOTION_CTRL (0x41U) /*!< Motion Control. Default value : 0x40 */
#define REG_BURST_READ_FIRST (0x42U) /*!< Burst Read Starting Register. Default value : 0x03 */
#define REG_BURST_READ_LAST (0x44U) /*!< Burst Read Ending Register. Default value : 0x09 */
#define REG_REST_MODE_CONFIG (0x45U) /*!< Rest Mode Confi guration. Default value : 0x00 */
#define REG_MOTION_BURST (0x63U) /*!< Burst Read. Default value : 0x00 */

/* ADNS3000 register bits */
#define REG_MOUSE_CTRL_POWERDOWN (0x02U) /*!< Mouse control register powerdown bit */
#define REG_MOTION_CTRL_MOT_A (0x80U) /*!< Motion control register polarity bit */
#define REG_MOTION_CTRL_MOT_S (0x40U) /*!< Motion control register edge sensitivity bit */
#define REG_MOUSE_CTRL_RES_EN (0x40U) /*!< Mouse control register resolution enable bit */
#define REG_MOUSE_CTRL_BIT_REPORTING (0x80U) /*!< Mouse control register "number of motion bits" bit*/

#define ADNS3000_SS_PIN   23
#define ADNS3000_SCK_PIN  29
#define ADNS3000_MOSI_PIN 12
#define ADNS3000_MISO_PIN 22

/**
 * adns3000 motion output pin polarity values.
 */
typedef enum
{
  ADNS3000_MOTION_OUTPUT_POLARITY_LOW = 0,  /*!< Motion output polarity active low */
  ADNS3000_MOTION_OUTPUT_POLARITY_HIGH = 1  /*!< Motion output polarity active high */
} motion_output_polarity_t;

/**
 * Motion output pin configuration.
 */
typedef enum
{
  ADNS3000_MOTION_OUTPUT_SENSITIVITY_LEVEL = 0,  /*!< Motion output pin will be driven low/high (depending on the polarity setting) as long as there is motion data in DELTA registers */
  ADNS3000_MOTION_OUTPUT_SENSITIVITY_EDGE = 1  /*!< Motion output pin will be driven low/high (depending on the polarity setting) for 380 ns when motion is detected during rest modes */
} motion_output_sensitivity_t;

/**
 * Mouse sensor resolution values.
 */
typedef enum
{
  ADNS3000_RESOLUTION_250DPI = 1, /*!< 250 dpi resolution */
  ADNS3000_RESOLUTION_500DPI = 2, /*!< 500 dpi resolution */
  ADNS3000_RESOLUTION_1000DPI = 0, /*!< 1000 dpi resolution */
  ADNS3000_RESOLUTION_1250DPI = 3, /*!< 1250 dpi resolution */
  ADNS3000_RESOLUTION_1500DPI = 4, /*!< 1500 dpi resolution */
  ADNS3000_RESOLUTION_1750DPI = 5, /*!< 1750 dpi resolution */
  ADNS3000_RESOLUTION_2000DPI = 6 /*!< 2000 dpi resolution */
} adns3000_resolution_t;

/**
 * Mouse sensor forced mode options.
 */
typedef enum
{
  ADNS3000_MODE_NORMAL = 0, /*!< Normal operation mode */
  ADNS3000_MODE_REST1 = 1, /*!< Rest1 operation mode */
  ADNS3000_MODE_REST2 = 2, /*!< Rest2 operation mode */
  ADNS3000_MODE_REST3 = 3, /*!< Rest3 operation mode */
  ADNS3000_MODE_RUN1 = 4, /*!< Run1 operation mode */
  ADNS3000_MODE_RUN2 = 5, /*!< Run2 operation mode */
  ADNS3000_MODE_IDLE = 6 /*!< Idle operation mode */
} adns3000_mode_t;

/**
 * Mouse sensor motion reporting bits.
 */
typedef enum
{
  ADNS3000_MOTION_BITS_8 = 0,  /*!< Motion reporting uses 8 bits */
  ADNS3000_MOTION_BITS_12 = 1  /*!< Motion reporting uses 12 bits */
} adns3000_motion_bits_t;

/* Register values according to Application Note 5544 from Avago */
static const uint8_t adns3000_config[][2] = 
{
	/*{register, value}*/
    {0x0D,0x81},
	{0x47,0x52},
	{0x48,0x68},
	{0x49,0x20},
	{0x6D,0x41},
	{0x6E,0xA0},
	{0x70,0x85},
	{0x71,0x64},
	{0x72,0x46},
	{0x73,0x37},
	{0x74,0x41},
	{0x75,0x28},
	{0x76,0x16},
	{0x77,0x0F},
	{0x64,0xF0},
	{0x03,0x03},
	{0x48,0x60},
};

static void adns3000_reset(void);

/* SPI Manipulation */
NRF_SPI_Type* spi_base_address = 0;

static uint8_t sdio_read_byte(uint8_t address)
{
	uint8_t tx_data[] = {0,0};
	uint8_t rx_data[] = {0,0};

	tx_data[0] = (address & ~(BIT_7));

  hal_spi_tx_rx((uint32_t*)spi_base_address, 2, tx_data, rx_data);

	return rx_data[1];
}

static uint8_t sdio_write_byte(uint8_t address, uint8_t data_byte)
{
    uint8_t tx_data[] = {0, 0};
    uint8_t rx_data[] = {0, 0};

    tx_data[0] = address | BIT_7;
    tx_data[1] = data_byte;

    hal_spi_tx_rx((uint32_t*)spi_base_address, 2, tx_data, rx_data);
    return rx_data[1];
}

static __inline bool adns3000_is_motion_detected(void)
{
    return ((sdio_read_byte(REG_MOTION_ST) & 0x80) != 0);
}

static __inline uint8_t adns3000_product_id_read(void)
{
  return sdio_read_byte(REG_PROD_ID);
}

static __inline void adns3000_reset(void)
{
  sdio_write_byte(REG_RESET, ADNS3000_RESET_NUMBER);
}

static uint32_t adns3000_motion_interrupt_set(motion_output_polarity_t polarity, motion_output_sensitivity_t sensitivity)
{
    uint8_t databyte = 0;
    uint32_t status = NRF_SUCCESS;
    
    databyte = sdio_read_byte(REG_MOTION_CTRL);
    
    switch (polarity)
    {
        case ADNS3000_MOTION_OUTPUT_POLARITY_LOW:
            databyte &= ~REG_MOTION_CTRL_MOT_A; // Clear REG_MOTION_CTRL_MOT_A bit
            break;
        
        case ADNS3000_MOTION_OUTPUT_POLARITY_HIGH:
            databyte |= REG_MOTION_CTRL_MOT_A;
            break;
        
        default:
            status = NRF_ERROR_INVALID_PARAM;
            break;
    }
    
    switch (sensitivity)
    {
        case ADNS3000_MOTION_OUTPUT_SENSITIVITY_LEVEL:
            databyte &= ~(REG_MOTION_CTRL_MOT_S);
            break;
        
        case ADNS3000_MOTION_OUTPUT_SENSITIVITY_EDGE:
            databyte |= (REG_MOTION_CTRL_MOT_S);
            break;
        
        default:
            status = NRF_ERROR_INVALID_PARAM;
            break;
    }
    
    if (status == NRF_SUCCESS)
    {
        sdio_write_byte(REG_MOTION_CTRL, databyte);		
    }

    return status;
}

uint32_t drv_mouse_sensor_read(int16_t* _delta_x, int16_t* _delta_y)
{
    uint8_t delta_x; /*!< Stores REG_DELTA_X contents */
    uint8_t delta_y; /*!< Stores REG_DELTA_Y contents */
    uint8_t delta_xy_high; /*!< Stores REG_DELTA_XY contents which contains upper 4 bits for both delta_x and delta_y when 12 bit mode is used */

    delta_x = sdio_read_byte(REG_DELTA_X);
    delta_y = sdio_read_byte(REG_DELTA_Y);
    delta_xy_high = sdio_read_byte(REG_DELTA_XY_HIGH);

    *_delta_x = ((int16_t) (((delta_xy_high & 0xF0) << 8)  | (delta_x << 4))) / 16;
    *_delta_y = ((int16_t) (((delta_xy_high & 0x0F) << 12) | (delta_y << 4))) / 16;	
    
    return NRF_SUCCESS;
}

void drv_mouse_sensor_dummy_read(void)
{
    if (drv_mouse_sensor_int_get())
    {
        int16_t dummy;
        drv_mouse_sensor_read(&dummy, &dummy);        
    }
}

uint32_t drv_mouse_sensor_init(void)
{
    uint32_t i;
    hal_spi_init_t spi_params;   
    
    spi_params.module    = SPI0;
    spi_params.mode      = SPI_MODE0;
    spi_params.frequency = Freq_1Mbps;
    spi_params.lsb_first = false;
    spi_params.sck_pin   = IO_ADNS3000_SCK_PIN;
    spi_params.mosi_pin  = IO_ADNS3000_MOSI_PIN;
    spi_params.miso_pin  = IO_ADNS3000_MISO_PIN;
    spi_params.ss_pin    = IO_ADNS3000_CSN_PIN;
    
    spi_base_address = (NRF_SPI_Type*)hal_spi_init(&spi_params);  
    if (spi_base_address == 0)
    {
        return NRF_ERROR_TIMEOUT;
    }

    NRF_GPIO->PIN_CNF[IO_ADNS3000_MOTION_PIN] = (GPIO_PIN_CNF_SENSE_High       << GPIO_PIN_CNF_SENSE_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)  |
                                                (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos);  

    adns3000_reset();
    if (adns3000_product_id_read() != ADNS3000_PRODUCT_ID)
    {
        return NRF_ERROR_TIMEOUT;
    }

    for (i = 0; i < (sizeof(adns3000_config) / 2); ++i)
    {
        sdio_write_byte(adns3000_config[i][0], adns3000_config[i][1]);
    }

    if (adns3000_motion_interrupt_set(ADNS3000_MOTION_OUTPUT_POLARITY_HIGH, ADNS3000_MOTION_OUTPUT_SENSITIVITY_LEVEL) != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }
    
    if (adns3000_is_motion_detected())
    {
        int16_t dummy;
        drv_mouse_sensor_read(&dummy, &dummy);
    }

    return NRF_SUCCESS;
}

bool drv_mouse_sensor_int_get(void)
{
    return ((NRF_GPIO->IN & (1 << IO_ADNS3000_MOTION_PIN)) != 0);
}

void drv_mouse_sensor_sense_enable(bool enable)
{
    uint32_t sense = enable ? GPIO_PIN_CNF_SENSE_High : GPIO_PIN_CNF_SENSE_Disabled;
    
    NRF_GPIO->PIN_CNF[IO_ADNS3000_MOTION_PIN] = (sense                         << GPIO_PIN_CNF_SENSE_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)  |
                                                (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos);      
}

bool drv_mouse_sensor_wakeup_prepare(void)
{
    NRF_GPIO->PIN_CNF[IO_ADNS3000_MOTION_PIN] = (GPIO_PIN_CNF_SENSE_High    << GPIO_PIN_CNF_SENSE_Pos) |
                                                (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos) |
                                                (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)  |
                                                (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
                                                (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos); 
    return true;
}
