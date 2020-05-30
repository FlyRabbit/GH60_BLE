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
 * @defgroup
 * @{
 * @ingroup 
 * @brief I/O configuration
 *
 * @details 
 */
#ifndef __IO_CFG_H__
#define __IO_CFG_H__

/** Keyboard matrix I/O definitions */
#define IO_CHERRY8x16_NUM_COLUMNS 5
#define IO_CHERRY8x16_ROWS        14
#define IO_CHERRY8x16_ROW_PORT    0x72A05927UL
#define IO_CHERRY8x16_COLUMN_PORT 0x010A0480UL

#define IO_PAIRING_BTN_PIN 13

/** NFC hardware I/O definitions */
//#define IO_STRFNFCA_IRQ_OUT_PIN 0
//#define IO_STRFNFCA_IRQ_IN_PIN  1
//#define IO_STRFNFCA_EN_NFC_PIN  20
//#define IO_STRFNFCA_OE_PIN      20
//#define IO_NFC_DETECT           26

//#define IO_STRFNFCA_SPI         SPI1
//#define IO_STRFNFCA_SCK         2
//#define IO_STRFNFCA_MOSI        3
//#define IO_STRFNFCA_MISO        4
//#define IO_STRFNFCA_SS          30

/** Misc. I/O */
//#define IO_MISC_TXD_PIN 16
//#define IO_MISC_RXD_PIN 18
//#define IO_MISC_LED_PIN 15

//#define IO_DBG_RX   18
//#define IO_DBG_TX   19
//#define IO_DBG_CTS  15
//#define IO_DBG_RTS  15
//#define IO_DBG_HWFC false


#endif /* __IO_CFG_H__ */
