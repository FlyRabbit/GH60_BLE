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
 * $LastChangedRevision:  $
 */

#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf51_bitfields.h"
#include "hal_uart.h"

#define STX 0x02
#define ETX 0x03
#define ESC 0x1F

#define UART_RX_BUF_SIZE 48

typedef enum
{
    rx_state_idle,
    rx_state_recv,
    rx_state_esc_recv
}uart_rx_state_t;

static uint8_t         s_rx_buf[UART_RX_BUF_SIZE];
static uint16_t        s_rx_buf_idx;
static uart_rx_state_t s_rx_state = rx_state_idle;
static bool            s_unstuff_rx;

static void (*rx_callback)(uint8_t*, uint8_t);

static void uart_put(uint8_t cr)
{
    NRF_UART0->TXD = cr;

    while(NRF_UART0->EVENTS_TXDRDY!=1)
    {
      // Wait for TXD data to be sent
    }

    NRF_UART0->EVENTS_TXDRDY=0;
}

static uint8_t uart_get(void)
{
    while (NRF_UART0->EVENTS_RXDRDY != 1)
    {
        // Wait for RXD data to be received
    }

    NRF_UART0->EVENTS_RXDRDY = 0;
    return (uint8_t)NRF_UART0->RXD;    
}

uint8_t hal_uart_get(void)
{
    uint8_t cr;
    
    NRF_UART0->TASKS_STARTRX = 1;
    
    cr = uart_get();
    
    NRF_UART0->TASKS_STOPRX = 1;
    
    return cr;
}

bool hal_uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data)
{
    bool ret = true;

    NRF_UART0->TASKS_STARTRX = 1;

    while (NRF_UART0->EVENTS_RXDRDY != 1)
    {
        if (timeout_ms-- >= 0)
        {
            // wait in 1ms chunk before checking for status
            nrf_delay_us(1000);
        }
        else
        {
            ret = false;
            break;
        }
    }  // Wait for RXD data to be received

    if (timeout_ms >= 0)
    {
        // clear the event and set rx_data with received byte
        NRF_UART0->EVENTS_RXDRDY = 0;
        *rx_data = (uint8_t)NRF_UART0->RXD;
    }

    NRF_UART0->TASKS_STOPRX = 1;

    return ret;
}

bool hal_uart_get_nonblock(uint8_t *byte)
{
    bool ret = false;
  
    NRF_UART0->TASKS_STARTRX = 1;
    
    if(NRF_UART0->EVENTS_RXDRDY == 1)
    {
        NRF_UART0->EVENTS_RXDRDY=0;
        *byte = (uint8_t)NRF_UART0->RXD;
        ret = true;
    }
  
    NRF_UART0->TASKS_STOPRX = 1;
    
    return ret;
}

void hal_uart_get_buffer(uint8_t *buffer, uint16_t size)
{
    uint16_t i;
    
    NRF_UART0->TASKS_STARTRX = 1;
    
    for(i = 0; i < size; i++)
    {
        buffer[i] = uart_get();
    }
    
    NRF_UART0->TASKS_STOPRX = 1;
}

void hal_uart_put(uint8_t cr)
{
    NRF_UART0->TASKS_STARTTX = 1;
    
    uart_put(cr);
    
    NRF_UART0->TASKS_STOPTX = 1;
}

void hal_uart_put_str(const uint8_t *str)
{
    uint_fast8_t i = 0;
    uint8_t ch = str[i++];
    
    NRF_UART0->TASKS_STARTTX = 1;
    
    while (ch != '\0')
    {
        uart_put(ch);
        ch = str[i++];
    }
    
    NRF_UART0->TASKS_STOPTX = 1;
}

void hal_uart_put_buffer(const uint8_t *buffer, uint16_t size)
{
    uint16_t i;

    NRF_UART0->TASKS_STARTTX = 1;
    
    for(i = 0; i < size; i++)
    {
        uart_put(buffer[i]);
    }
    
    NRF_UART0->TASKS_STOPTX = 1;
}

void hal_uart_put_stuffed(const uint8_t *msg, uint16_t size)
{
    uint16_t i;
    
    NRF_UART0->TASKS_STARTTX = 1;
    
    uart_put(STX);
    
    for (i = 0; i < size; i++)
    {
        if( (msg[i] == STX) || (msg[i] == ETX) || (msg[i] == ESC))
        {
            uart_put(ESC);
            uart_put(msg[i] ^ 0x20);
        }
        else
        {
            uart_put(msg[i]);
        }
    }
    
    uart_put(ETX);
    
    NRF_UART0->TASKS_STOPTX = 1;
}

void hal_uart_config(uint8_t rts_pin_number,
                 uint8_t txd_pin_number,
                 uint8_t cts_pin_number,
                 uint8_t rxd_pin_number,
                 bool hwfc,
                 void (*callback)(uint8_t*, uint8_t),
                 bool unstuff_rx)
{
    uint8_t  softdevice_enabled;
    uint32_t err_code;
    
    rx_callback = callback;
    s_unstuff_rx = unstuff_rx;
  
    nrf_gpio_cfg_output(txd_pin_number);
    nrf_gpio_cfg_input(rxd_pin_number, NRF_GPIO_PIN_NOPULL);  

    NRF_UART0->PSELTXD = txd_pin_number;
    NRF_UART0->PSELRXD = rxd_pin_number;

    if (hwfc)
    {
        nrf_gpio_cfg_output(rts_pin_number);
        nrf_gpio_cfg_input(cts_pin_number, NRF_GPIO_PIN_NOPULL);
        NRF_UART0->PSELCTS = cts_pin_number;
        NRF_UART0->PSELRTS = rts_pin_number;
        NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
    }

    NRF_UART0->BAUDRATE         = (UART_BAUDRATE_BAUDRATE_Baud250000 << UART_BAUDRATE_BAUDRATE_Pos);
    NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->EVENTS_RXDRDY    = 0;
    
    if (callback != 0)
    {
        NRF_UART0->INTENSET = (UART_INTENSET_RXDRDY_Enabled << UART_INTENSET_RXDRDY_Pos);
        NRF_UART0->TASKS_STARTRX = 1;

        err_code = sd_softdevice_is_enabled(&softdevice_enabled);
        APP_ERROR_CHECK(err_code);
    
        if (softdevice_enabled == 0)
        {
            NVIC_SetPriority(UART0_IRQn, NRF_APP_PRIORITY_LOW);
            NVIC_ClearPendingIRQ(UART0_IRQn);
            NVIC_EnableIRQ(UART0_IRQn);
        }
        else
        {
            err_code = sd_nvic_SetPriority(UART0_IRQn, NRF_APP_PRIORITY_LOW);
            APP_ERROR_CHECK(err_code);
            err_code = sd_nvic_ClearPendingIRQ(UART0_IRQn);
            APP_ERROR_CHECK(err_code);
            err_code = sd_nvic_EnableIRQ(UART0_IRQn);
            APP_ERROR_CHECK(err_code);
        }
    } 
}

void UART0_IRQHandler(void)
{
    uint8_t rxd;
    
    NRF_UART0->EVENTS_RXDRDY=0;

    rxd = NRF_UART0->RXD;
    
    if(rx_callback != 0)
    {
    
        if (s_unstuff_rx)
        {
            switch (s_rx_state)
            {
                case rx_state_idle:
                    if (rxd == STX)
                    {
                        s_rx_state = rx_state_recv;
                    }
                    break;
                
                case rx_state_recv:
                    if (rxd == ESC)
                    {
                        s_rx_state = rx_state_esc_recv;
                    }
                    else if (rxd == ETX)
                    {
                        rx_callback(s_rx_buf, s_rx_buf_idx);
                        s_rx_buf_idx = 0;
                        s_rx_state = rx_state_idle;
                    }
                    else
                    {
                        s_rx_buf[s_rx_buf_idx++] = rxd;       
                    }
                    break;
                
                case rx_state_esc_recv:

                    s_rx_buf[s_rx_buf_idx++] = rxd ^ 0x20;
                    s_rx_state = rx_state_recv;
                    break;
            }            
        }
        else
        {
            rx_callback(&rxd, 1);
        }
    }
}
