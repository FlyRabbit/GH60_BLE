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
 
#include "drv_mouse_btn.h"

#include "io_cfg.h"
#include "nrf.h"
#include "nrf_error.h"

uint32_t drv_mouse_btn_init(void)
{    
    drv_mouse_btn_wakeup_prepare();
    return NRF_SUCCESS;
}

void drv_mouse_btn_sense_enable(bool p_enable)
{
    uint32_t sense = p_enable ? GPIO_PIN_CNF_SENSE_High : GPIO_PIN_CNF_SENSE_Disabled;
    
    NRF_GPIO->PIN_CNF[IO_BTN_LEFT_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
        (sense                      << GPIO_PIN_CNF_SENSE_Pos);
                                          
    NRF_GPIO->PIN_CNF[IO_BTN_RIGHT_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
        (sense                      << GPIO_PIN_CNF_SENSE_Pos);
                                          
    NRF_GPIO->PIN_CNF[IO_BTN_MIDDLE_PIN] =
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
        (sense                      << GPIO_PIN_CNF_SENSE_Pos);
                                          
    NRF_GPIO->PIN_CNF[IO_BTN_SIDE_LEFT_PIN] = 
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
        (sense                      << GPIO_PIN_CNF_SENSE_Pos);   
                                          
    NRF_GPIO->PIN_CNF[IO_BTN_SIDE_RIGHT_PIN] = 
        (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
        (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
        (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
        (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
        (sense                      << GPIO_PIN_CNF_SENSE_Pos);    
}

uint32_t drv_mouse_btn_read(void)
{    
  return (NRF_GPIO->IN & IO_BTNS_ALL_MSK);
}

bool drv_mouse_btn_wakeup_prepare(void)
{
    int      i;
    uint32_t sense;
    uint32_t input;
    uint32_t pins[] = {IO_BTN_LEFT_PIN, 
                       IO_BTN_RIGHT_PIN, 
                       IO_BTN_MIDDLE_PIN, 
                       IO_BTN_SIDE_LEFT_PIN, 
                       IO_BTN_SIDE_RIGHT_PIN};
    
    input = NRF_GPIO->IN;
                       
    for(i = 0; i < (sizeof(pins) / sizeof(pins[0])); ++i)
    {
        uint32_t pin;
        
        pin = pins[i];
        if ((input & (1 << pin)) != 0)
        {
            sense = GPIO_PIN_CNF_SENSE_Low;
        }
        else
        {
            sense = GPIO_PIN_CNF_SENSE_High;
        }
        NRF_GPIO->PIN_CNF[pin] =
            (GPIO_PIN_CNF_DIR_Input     << GPIO_PIN_CNF_DIR_Pos)    |
            (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)  |
            (GPIO_PIN_CNF_PULL_Pulldown << GPIO_PIN_CNF_PULL_Pos)   |
            (GPIO_PIN_CNF_DRIVE_S0S1    << GPIO_PIN_CNF_DRIVE_Pos)  |
            (sense                      << GPIO_PIN_CNF_SENSE_Pos);
    }
    return true;
}
