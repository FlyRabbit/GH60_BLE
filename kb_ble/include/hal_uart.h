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

#ifndef __HAL_UART_H__
#define __HAL_UART_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf51.h"
#include "nrf51_bitfields.h"

/** @file
* @brief UART hardware abstraction library
*
*
* @defgroup 
* @{
* @ingroup 
* @brief 
*
*/

/**@brief Blocking UART0 read
 *
 * @return Read byte from UART0
 */
uint8_t hal_uart_get(void);

/**@brief Blocking UART0 read with timeout
 *
 * @param[in] timeout_ms Timeout in milliseconds.
 * @param[out] rx_data Byte read if return value is true.
 *
 * @return
 * @retval true
 * @retval false
 */
bool hal_uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data);

/**@brief Nonblocking UART0 read.
 *
 * @param[out] byte Byte read if return value is true.
 *
 * @return
 * @retval true
 * @retval false
 */
bool hal_uart_get_nonblock(uint8_t *byte);

/**@brief Blocking UART0 buffer read.
 *
 * @param[out] buffer Buffer to put the data received data.
 * @param[in] size Number of bytes to read.
 */
void hal_uart_get_buffer(uint8_t *buffer, uint16_t size);

/**@brief Write byte to UART0.
 *
 * @param[in] cr Byte to write.
 */
void hal_uart_put(uint8_t cr);

/**@brief Write null terminated string to UART0.
 *
 * @param[in] str null terminated string to write.
 */
void hal_uart_put_str(const uint8_t *str);

/**@brief Write buffer to UART0.
 *
 * @param[in] buffer Buffer to write.
 * @param[in] size Number of bytes to write.
 */
void hal_uart_put_buffer(const uint8_t *buffer, uint16_t size);

/**@brief Write a byte stuffed message to UART0.
 *
 * @note Sends start byte(STX), message, then end byte(ETX). 
 * If the message contains either STX, ETX or ESC bytes, 
 * it will be replaced by an escape byte (ESC) followed by the byte xor'ed with 0x20.
 * this is done to have a defined start and end of each message. 
 *
 * @param[in] msg Message to write.
 * @param[in] size Number of bytes in message.
 */
void hal_uart_put_stuffed(const uint8_t *msg, uint16_t size);

/**@brief Configure UART0.
 *
 * @param[in] rts_pin_number RTS pin number.
 * @param[in] txd_pin_number TXD pin number.
 * @param[in] cts_pin_number CTS pin number.
 * @param[in] rxd_pin_number RXD pin number.
 * @param[in] hwfc Use HW flow control.
 * @param[in] callback Callback on RX event, set to 0 if not used.
 * @param[in] unstuff_rx
 */
void hal_uart_config(uint8_t rts_pin_number,
                 uint8_t txd_pin_number,
                 uint8_t cts_pin_number,
                 uint8_t rxd_pin_number,
                 bool hwfc,
                 void (*callback)(uint8_t*, uint8_t),
                 bool unstuff_rx);

#endif /* __HAL_UART_H__ */

/** @} */
