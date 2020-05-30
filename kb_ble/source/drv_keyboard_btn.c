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
#include "drv_keyboard_btn.h"
#include "io_cfg.h"
#include "nrf.h"

void drv_keyboard_btn_init(void)
{
    NRF_GPIO->PIN_CNF[IO_PAIRING_BTN_PIN] =
        (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void drv_keyboard_btn_sense_enable(bool p_enable)
{
    NRF_GPIO->PIN_CNF[IO_PAIRING_BTN_PIN] &= ~GPIO_PIN_CNF_SENSE_Msk;
    
    if (p_enable)
    {
        NRF_GPIO->PIN_CNF[IO_PAIRING_BTN_PIN] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
    }
    else
    {
        NRF_GPIO->PIN_CNF[IO_PAIRING_BTN_PIN] |= (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    }
}

uint32_t __inline drv_keyboard_btn_read_pin_state(void)
{
    return (NRF_GPIO->IN & (1 << IO_PAIRING_BTN_PIN)) == 0;
}

void drv_keyboard_btn_wakeup_prepare(void)
{
    NRF_GPIO->PIN_CNF[IO_PAIRING_BTN_PIN] =
        (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
        (GPIO_PIN_CNF_SENSE_Low      << GPIO_PIN_CNF_SENSE_Pos);
}
