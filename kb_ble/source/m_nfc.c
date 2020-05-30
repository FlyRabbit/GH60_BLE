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
 
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "app_scheduler.h"
#include "app_timer.h"

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "drv_nfc.h"
#include "m_nfc.h"
#include "io_cfg.h"

static uint8_t card_data[0x2D*4] = {
    0x04, 0xDB, 0xE3, 0xB4,  // UID / Internal
    0xEA, 0x4B, 0x28, 0x80,  // Serial Number
    0x09, 0x48, 0x00, 0x00,  // Internal / LOCK
    0xE1, 0x11, 0x12, 0x00,  // CC read only
    
    0x03, 0x54, // TLV Header and length NBNB
    
    0xDA, // NDEF Record Header 
    0x20, // Record Type Length
    0x2F, // Payload Length  NBNB
    0x01, // Payload ID Length
    0x61, 0x70, 0x70, 0x6C, // -
    0x69, 0x63, 0x61, 0x74, //  |
    0x69, 0x6F, 0x6E, 0x2F, //  |
    0x76, 0x6E, 0x64, 0x2E, //   > Record Type Name: application/vnd.bluetooth.ep.oob
    0x62, 0x6C, 0x75, 0x65, //  |
    0x74, 0x6F, 0x6F, 0x74, //  |
    0x68, 0x2E, 0x65, 0x70, //  |
    0x2E, 0x6F, 0x6F, 0x62, // -
    0x30, // Payload ID: "0"
    0x2F, 0x00, //Bluetooth OOB Data Length: NBNB
    0x49, 0xF9, 0x69, 0x59, 0x3D, 0xEF, // Bluetooth Device Address: 00:1E:DE:C0:4B:73
    0x02, // EIR Data Length: 2
    0x11, // EIR Data Type: Security Manager Out of Band Flag
    //0x03, // Security Manager Out of Band Flag indicating OOB data present and LE Host supported.
    0x0B,
    0x11, // EIR Data Length: 
    0x10, // EIR Data Type: Security Manager TK Value
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // TK = OOB 
    0x03, // EIR Data Length: 
    0x19, // EIR Data Type: Appearance
    0x03, 0xC2,
    0x0D, // EIR Data Length: 13
    0x09, // EIR Data Type: Complete Local Name
    'N', 'o', 'r', 'd', 'i', 'c', ' ', 'T', 'e', 's', 't', '!',

    0xFE, // TLV terminator

    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00     
};

#define NFC_TLV_LEN_OFFSET         17
#define NFC_NDEF_LEN_OFFSET        20
#define NFC_OOB_DATA_LEN_OFFSET    55
#define NFC_BLE_ADDR_OFFSET        57
#define NFC_OOB_KEY_OFFSET         68
#define NFC_APPEARANCE_OFFSET      86
#define NFC_LOCAL_NAME_LEN_OFFSET  88
#define NFC_LOCAL_NAME_OFFSET      90

#define NFC_CARD_MEM_SIZE         144
#define NFC_LOCAL_NAME_MAX_SIZE   (NFC_CARD_MEM_SIZE - (NFC_LOCAL_NAME_OFFSET + 1))


#define NFC_TYPE2_CMD_OFFSET  0x00
#define NFC_TYPE2_WRITE       0xA2
#define NFC_TYPE2_READ        0x30
#define NFC_TYPE2_SECTOR_SEL  0xC2

#define NFC_TYPE2_BNO_OFFSET  0x01
#define NFC_TYPE2_DATA_OFFSET 0x02

static bool           s_nfc_enabled     = false;
static bool           s_nfc_initialized = false;

static uint32_t on_nfc_response(uint8_t *resp);

uint32_t m_nfc_set_oob_key(uint8_t *key)
{
    uint8_t i;
    
    for (i = 0; i < 16; i++)
    {
        card_data[NFC_OOB_KEY_OFFSET + (15 - i)] = key[i];
    }
    
    return NRF_SUCCESS;
}

static void m_nfc_set_appearance(uint16_t appearance)
{
    card_data[NFC_APPEARANCE_OFFSET] = (uint8_t)(appearance >> 8);
    card_data[NFC_APPEARANCE_OFFSET + 1] = (uint8_t)(appearance & 0x00FF);
}

static uint32_t m_nfc_set_local_name(uint8_t *local_name, uint8_t size)
{
    uint32_t status;
    
    if (size <= NFC_LOCAL_NAME_MAX_SIZE)
    {
        uint8_t i;
        int8_t size_diff = (size - card_data[NFC_LOCAL_NAME_LEN_OFFSET]) + 1;
        
        card_data[NFC_TLV_LEN_OFFSET] = card_data[NFC_TLV_LEN_OFFSET] + size_diff;
        card_data[NFC_NDEF_LEN_OFFSET] = card_data[NFC_NDEF_LEN_OFFSET] + size_diff;
        card_data[NFC_OOB_DATA_LEN_OFFSET] = card_data[NFC_OOB_DATA_LEN_OFFSET] + size_diff;
        card_data[NFC_LOCAL_NAME_LEN_OFFSET] = size + 1;
       
        for (i = 0; i < size; i++)
        {
            card_data[NFC_LOCAL_NAME_OFFSET + i] = local_name[i];
        }
        
        card_data[NFC_LOCAL_NAME_OFFSET + size] = 0xFE; // TLV Terminator
        
        status = NRF_SUCCESS;
    }
    else
    {
        status = NRF_ERROR_DATA_SIZE;
    }
    
    return status;
}

void m_nfc_set_ble_addr(uint8_t *addr)
{
    uint8_t i;
    
    for (i = 0; i < 6; i++)
    {     
        card_data[NFC_BLE_ADDR_OFFSET + i] = addr[i];
    }
}

uint32_t m_nfc_init(uint8_t *ble_addr, uint8_t *ble_eir_oob_key, uint8_t *ble_local_name, uint8_t name_size, uint16_t appearance)
{
    uint32_t err_code = NRF_SUCCESS;  
    
    s_nfc_initialized = true;
    
    if (ble_addr != NULL)
    {
        m_nfc_set_ble_addr(ble_addr);
    }
    
    if (ble_eir_oob_key != NULL)
    {
        m_nfc_set_oob_key(ble_eir_oob_key);
    }
    
    if (ble_local_name != NULL)
    {    
        err_code = m_nfc_set_local_name(ble_local_name, name_size);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    
    m_nfc_set_appearance(appearance);  
    
    return drv_nfc_init(on_nfc_response);
}

bool m_nfc_is_enabled(void)
{
    return s_nfc_enabled;
}

uint32_t m_nfc_enable(void)
{   
    uint32_t err_code = NRF_SUCCESS;
    
    if (s_nfc_initialized == false)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (s_nfc_enabled == false)
    {    
        err_code = drv_nfc_start();
        s_nfc_enabled = true;
    }     
    
    return err_code;
}

uint32_t m_nfc_disable(void)
{   
    uint32_t err_code = NRF_SUCCESS;
    
    if (s_nfc_enabled == true)
    {   
        err_code = drv_nfc_stop();
        s_nfc_enabled = false;
    }  
    
    return err_code;
}

static void nfc_handle_write(uint8_t *resp)
{
    drv_nfc_reply(NFC_REPLY_ACK, NULL, 0);
}

static void nfc_handle_read(uint8_t *resp)
{
    uint8_t card_data_bno;
    uint8_t *card_data_ptr;
    
    card_data_bno = 0x2A;
    card_data_ptr = card_data;          
    
    if (resp[NFC_TYPE2_BNO_OFFSET] < card_data_bno)
    {   
        drv_nfc_reply(NFC_REPLY_DATA, &card_data_ptr[resp[NFC_TYPE2_BNO_OFFSET] * 4], 16);
    }
    else
    {
        drv_nfc_reply(NFC_REPLY_NACK, NULL, 0);     
    }
}

static void nfc_handle_sector_sel(uint8_t *resp)
{
    drv_nfc_reply(NFC_REPLY_ACK, NULL, 0);
}

static uint32_t on_nfc_response(uint8_t *resp)
{
    uint32_t err_code = NRF_SUCCESS;
    
    switch (resp[NFC_TYPE2_CMD_OFFSET])
    {
        case NFC_TYPE2_WRITE:
            nfc_handle_write(resp);         
            break;
        
        case NFC_TYPE2_READ:
            nfc_handle_read(resp); 
            break;
            
        case NFC_TYPE2_SECTOR_SEL:
            nfc_handle_sector_sel(resp); 
            break;

        default:
            err_code = NRF_ERROR_INVALID_PARAM;
            break;
    }
    
    return err_code;
}

bool m_nfc_module_present(void)
{
    nrf_gpio_cfg_input(IO_NFC_DETECT, NRF_GPIO_PIN_PULLUP);
    if (!nrf_gpio_pin_read(IO_NFC_DETECT))
    {
        nrf_gpio_cfg_input(IO_NFC_DETECT, NRF_GPIO_PIN_NOPULL);
        return true;
    }
    else
    {
        nrf_gpio_cfg_input(IO_NFC_DETECT, NRF_GPIO_PIN_NOPULL);
        return false; 
    }    
}

