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
#include <string.h>

#include "app_scheduler.h"
#include "app_timer.h"
#include "app_util.h"
#include "common_params.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#include "nrf_assert.h"
#include "nrf_error.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "hal_spi.h"
#include "hal_gpiote.h"
#include "drv_nfc.h"
#include "io_cfg.h"

#define STRFNFC_IRQ_OUT     IO_STRFNFCA_IRQ_OUT_PIN
#define STRFNFC_IRQ_OUT_MSK (1UL << STRFNFC_IRQ_OUT)
#define STRFNFC_IRQ_IN     IO_STRFNFCA_IRQ_IN_PIN
#define STRFNFC_IRQ_IN_MSK  (1UL << IO_STRFNFCA_IRQ_IN_PIN)
#define STRFNFC_EN_NFC     IO_STRFNFCA_EN_NFC_PIN
#define STRFNFC_OE         IO_STRFNFCA_OE_PIN

#define STRFNFC_SELECT()    
#define STRFNFC_DESELECT()  

#define STRFNFC_CMD_SEND 0x00
#define STRFNFC_CMD_RECV 0x02
#define STRFNFC_CMD_POLL 0x03

#define STRFNFC_IDN       0x01
#define STRFNFC_PROTSEL   0x02
#define STRFNFC_POLLFIELD 0x03
#define STRFNFC_SENDRECV  0x04
#define STRFNFC_LISTEN    0x05
#define STRFNFC_SEND      0x06
#define STRFNFC_IDLE      0x07
#define STRFNFC_RDREG     0x08
#define STRFNFC_WRREG     0x09
#define STRFNFC_BAUDRATE  0x0A
#define STRFNFC_ACFILTER  0x0D
#define STRFNFC_ECHO      0x55
#define STRFNFC_NONE      0xFF

#define STRFNFC_CMD_OFFSET     0x00
#define STRFNFC_CMD_SUCCESS    0x00
#define STRFNFC_CMD_ECHO       0x55
#define STRFNFC_CMD_DATA       0x80
#define STRFNFC_CMD_NOFIELD    0x8F

#define STRFNFC_LEN_OFFSET     0x01

#define STRFNFC_DAT_OFFSET     0x02
#define STRFNFC_DAT_FIELD      0x01
#define STRFNFC_DAT_WRITE      0xA2
#define STRFNFC_DAT_READ       0x30
#define STRFNFC_DAT_SECTOR_SEL 0xC2 

#define STRFNFC_MAX_BUFFER_SIZE   48
#define STRFNFC_MAX_POLLFIELD_NUM 150

#define STRFNFC_IDN_CMD       {STRFNFC_IDN, 0x00}        
#define STRFNFC_PROTSEL_CMD   {STRFNFC_PROTSEL, 0x02, 0x12, 0x02} // ISO/IEC 14443
#define STRFNFC_POLLFIELD_CMD {STRFNFC_POLLFIELD, 0x00}
#define STRFNFC_LISTEN_CMD    {STRFNFC_LISTEN, 0x00}
#define STRFNFC_ECHO_CMD      {STRFNFC_ECHO, 0x00}
#define STRFNFC_ACFILTER_CMD  {STRFNFC_ACFILTER, 0x0B, 0x44, 0x00, 0x00, 0x88, 0x04, 0xDB, 0xE3, 0xEA, 0x4B, 0x28, 0x80}
#define STRFNFC_IDLE_CMD      {STRFNFC_IDLE, 0x0E, (0x04 | 0x08), 0x01, 0x42, 0x38, 0x00, 0x18, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00}
#define STRFNFC_ACK_CMD       {STRFNFC_SEND, 2, 0xA0, 0x04};
#define STRFNFC_NACK_CMD      {STRFNFC_SEND, 2, 0x00, 0x04};
#define STRFNFC_HIBERNATE_CMD {STRFNFC_IDLE, 0x0E, 0x08, 0x04, 0x00, 0x04, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define ELSE_ASSERT() else{APP_ERROR_CHECK_BOOL(false);}

#define APP_TIMER_POLLFIELD 0
#define APP_TIMER_LISTEN    1

typedef enum {
	NFC_STATE_VOID,
	NFC_STATE_INIT,
    NFC_STATE_LISTEN
}nfc_state_t;

typedef struct {
	nfc_state_t state;
    uint8_t response[STRFNFC_MAX_BUFFER_SIZE];
    uint8_t prev_cmd;
}nfc_context_t;

static nfc_context_t nfc_context;

static const uint8_t strfnfc_idn_cmd[]       = STRFNFC_IDN_CMD;
static const uint8_t strfnfc_protsel_cmd[]   = STRFNFC_PROTSEL_CMD;
static const uint8_t strfnfc_pollfield_cmd[] = STRFNFC_POLLFIELD_CMD;
static const uint8_t strfnfc_listen_cmd[]    = STRFNFC_LISTEN_CMD;
static const uint8_t strfnfc_echo_cmd[]      = STRFNFC_ECHO_CMD;
static const uint8_t strfnfc_acfilter_cmd[]  = STRFNFC_ACFILTER_CMD;
static const uint8_t strfnfc_idle_cmd[]      = STRFNFC_IDLE_CMD;
static const uint8_t strfnfc_ack_cmd[]       = STRFNFC_ACK_CMD;
static const uint8_t strfnfc_nack_cmd[]      = STRFNFC_NACK_CMD;
static const uint8_t strfnfc_hibernate_cmd[] = STRFNFC_HIBERNATE_CMD;

NRF_SPI_Type*         spi_address           = 0;
static uint32_t       nfc_pollfield_counter = 0;
static uint32_t       (*s_callback)(uint8_t *resp);
static app_timer_id_t s_timer_id[2];
static bool           listen_timer_running = false;
static bool           drv_nfc_sense_enabled = false;

static void send_pollfield_handler(void* p_context);
static void inactivity_listen_handler(void* p_context);
void drv_nfc_get_response(uint8_t *res);
static void drv_nfc_resp_handler(void * p_event_data, uint16_t event_size);

static void on_nfc_response_in_init(nfc_context_t *context);
static void on_nfc_response_in_listen(nfc_context_t *context);

__INLINE static uint32_t SPI_MASTER_INIT(void)
{
    hal_spi_init_t spi_init;
    
    spi_init.module    = IO_STRFNFCA_SPI;
    spi_init.mode      = SPI_MODE3;
    spi_init.frequency = Freq_2Mbps;
    spi_init.lsb_first = false;
    spi_init.sck_pin   = IO_STRFNFCA_SCK;
    spi_init.mosi_pin  = IO_STRFNFCA_MOSI;
    spi_init.miso_pin  = IO_STRFNFCA_MISO;
    spi_init.ss_pin    = IO_STRFNFCA_SS;
    
    spi_address = (NRF_SPI_Type*)hal_spi_init(&spi_init);
    if (spi_address == 0)
    {
        return NRF_ERROR_TIMEOUT;
    }
    
    return NRF_SUCCESS; 
}

__INLINE static bool SPI_TRANSMIT_BUFFER(uint16_t size, uint8_t* tx, uint8_t* rx)
{
	return hal_spi_tx_rx((uint32_t *)spi_address, size, tx, rx);
}

__INLINE static uint32_t SPI_RECV_CMD(uint32_t max_len, const uint8_t* tx, uint8_t* rx)
{
	return hal_spi_tx_rx_var((uint32_t *)spi_address, max_len, "%c%c%d", tx, rx);
}

__INLINE static void drv_nfc_pin_configure(void)
{
	NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)         |
										 (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)   |
										 (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos)      |
										 (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)      |
										 (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
	
	NRF_GPIO->PIN_CNF[STRFNFC_IRQ_IN] =  (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)         |
										 (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
										 (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
										 (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
										 (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);	
																			 
	NRF_GPIO->PIN_CNF[STRFNFC_EN_NFC] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)         |
										(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
										(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
										(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
										(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
																			
	NRF_GPIO->PIN_CNF[STRFNFC_OE] =     (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)         |
										(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
										(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
										(GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
										(GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);                                      
}

__INLINE static void drv_nfc_sense_enable(bool enable)
{
    
    NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] &= ~GPIO_PIN_CNF_SENSE_Msk;
    
    if (enable)
    {
        NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] |= (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);
        drv_nfc_sense_enabled = true;
    }
    else
    {
        NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] |= (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
        drv_nfc_sense_enabled = false;
    }
}

static void drv_nfc_gpio_int_handler(void)
{   
    if (drv_nfc_sense_enabled)
    {
        uint32_t timeout = 0;
        while ((NRF_GPIO->IN & STRFNFC_IRQ_OUT_MSK) == 0)
        {
            drv_nfc_resp_handler(0,0);
            
            timeout++;
            if (timeout > 10)
            {
                break;
            }
        }
    }
}

uint32_t drv_nfc_init(uint32_t (*callback)(uint8_t *resp))
{
    uint32_t err_code;
    
    nfc_context.state = NFC_STATE_INIT;
    nfc_context.prev_cmd = STRFNFC_NONE;    
    
    if (callback == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    s_callback = callback;
			 
    err_code = app_timer_create(&s_timer_id[APP_TIMER_POLLFIELD],
                                APP_TIMER_MODE_SINGLE_SHOT, 
                                send_pollfield_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
        
    err_code = app_timer_create(&s_timer_id[APP_TIMER_LISTEN],
                                APP_TIMER_MODE_SINGLE_SHOT,
                                inactivity_listen_handler);

    // GPIOTE callback
    err_code = hal_gpiote_init();
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }        
    
    err_code = hal_gpiote_cb_set(hal_gpiote_evt_port, drv_nfc_gpio_int_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }       
    
    drv_nfc_power_on();
    
    err_code = drv_nfc_init_hibernate();
    if (err_code != NRF_SUCCESS)
    {
        drv_nfc_power_off();
        return err_code;
    }      
    drv_nfc_power_off();
    
	return err_code;
}

static void drv_nfc_send_cmd(const uint8_t *cmd)
{
    uint8_t tmp[STRFNFC_MAX_BUFFER_SIZE];
    uint8_t tx_cmd[STRFNFC_MAX_BUFFER_SIZE];
    uint_fast8_t i;
    uint32_t sense = NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT];
    
    NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] &= ~GPIO_PIN_CNF_SENSE_Msk;
    NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] |= (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    nfc_context.prev_cmd = cmd[0];
    
    tx_cmd[0]                      = STRFNFC_CMD_SEND;
    tx_cmd[STRFNFC_CMD_OFFSET + 1] = cmd[STRFNFC_CMD_OFFSET];
    tx_cmd[STRFNFC_LEN_OFFSET + 1] = cmd[STRFNFC_LEN_OFFSET];
    for (i = 0; i < cmd[STRFNFC_LEN_OFFSET]; ++i)
    {
        tx_cmd[STRFNFC_DAT_OFFSET + 1 + i] = cmd[STRFNFC_DAT_OFFSET + i];
    }
    
    SPI_TRANSMIT_BUFFER(cmd[STRFNFC_LEN_OFFSET] + 3, tx_cmd, tmp);
        
    if (sense != NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT])
    {
        if ((NRF_GPIO->IN & STRFNFC_IRQ_OUT_MSK) == 0)
        {
            drv_nfc_resp_handler(0,0);
        }
        
        NRF_GPIO->PIN_CNF[STRFNFC_IRQ_OUT] = sense;        
    }
}

uint32_t drv_nfc_init_hibernate(void)
{
    uint32_t timeout = 0;   
    uint8_t response[STRFNFC_MAX_BUFFER_SIZE];
    
    while(!(NRF_GPIO->IN & (1<<STRFNFC_IRQ_OUT)))
    {
        // Dummy read.
        drv_nfc_get_response(response);
        ++timeout;
        
        if (timeout > 100)
        {
            // No response
            return NRF_ERROR_INTERNAL;
        }        
    }
    
    // Sending echo command to make sure it's alive
    drv_nfc_send_cmd(strfnfc_echo_cmd);
    
    while ((NRF_GPIO->IN & (1<<STRFNFC_IRQ_OUT)) != 0)
    {
        ++timeout;
        
        if (timeout > 20000)
        {
            // No response
            break;          
        }
    }
    
    drv_nfc_get_response(response);
    if (response[0] != STRFNFC_ECHO)
    {
        __NOP();
    }

    // Sending hibernate command. No response is generated
    drv_nfc_send_cmd(strfnfc_hibernate_cmd);
                      
    return NRF_SUCCESS;
}

uint32_t drv_nfc_start(void)
{
    uint32_t err_code = NRF_SUCCESS;
    uint32_t timeout = 0;   
    uint8_t response[STRFNFC_MAX_BUFFER_SIZE];    
    
    drv_nfc_power_on();
    
    while(!(NRF_GPIO->IN & (1<<STRFNFC_IRQ_OUT)))
    {
        // Dummy read.
        drv_nfc_get_response(response);
        ++timeout;
        
        if (timeout > 100)
        {
            // No response
            return NRF_ERROR_INTERNAL;
        }        
    }
    
    drv_nfc_sense_enable(true);      
    
    nfc_context.state = NFC_STATE_INIT;
    nfc_context.prev_cmd = STRFNFC_NONE;    
    
    drv_nfc_send_cmd(strfnfc_echo_cmd);

    return err_code;
}

uint32_t drv_nfc_stop(void)
{
    uint32_t err_code = NRF_SUCCESS;
    
    drv_nfc_sense_enable(false);
    
    err_code = drv_nfc_init_hibernate();
    
    drv_nfc_power_off();
    
    return err_code;
}

void drv_nfc_power_on(void)
{
    SPI_MASTER_INIT();    
    
	drv_nfc_pin_configure();

	NRF_GPIO->OUTSET = (1UL << STRFNFC_OE) | (1UL << STRFNFC_EN_NFC) | (1UL << STRFNFC_IRQ_IN);
    nrf_delay_us(2000);  //100us + pwr enable time and level translator on time. 
	NRF_GPIO->OUTCLR = (1UL << STRFNFC_IRQ_IN);
	nrf_delay_us(20);    // 10us min according to datasheet.
	NRF_GPIO->OUTSET = (1UL << STRFNFC_IRQ_IN);
	nrf_delay_us(11000); // 10ms max according to datasheet.
}

void drv_nfc_power_off(void)
{   
	NRF_GPIO->OUTCLR = (1UL << STRFNFC_OE);
	NRF_GPIO->OUTCLR = (1UL << STRFNFC_EN_NFC);

    NRF_SPI1->PSELSCK = 0xFFFFFFFF;
    NRF_SPI1->PSELMOSI = 0xFFFFFFFF;
    NRF_SPI1->PSELMISO = 0xFFFFFFFF;
    hal_spi_disable(IO_STRFNFCA_SPI);
    
    NRF_GPIO->PIN_CNF[IO_STRFNFCA_IRQ_OUT_PIN] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)         |
										      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
										      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
										      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
										      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    
    NRF_GPIO->PIN_CNF[IO_STRFNFCA_MISO] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)         |
										  (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
										  (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)     |
										  (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)       |
										  (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void drv_nfc_get_response(uint8_t *res)
{
    uint32_t len = 0;
    uint32_t i;
    uint8_t rx[STRFNFC_MAX_BUFFER_SIZE] = {0};
    uint8_t tx[STRFNFC_MAX_BUFFER_SIZE] = {0};   
    
    memset(tx, 0, STRFNFC_MAX_BUFFER_SIZE);
    memset(rx, 0, STRFNFC_MAX_BUFFER_SIZE);

    tx[0] = STRFNFC_CMD_RECV;  
    
    SPI_RECV_CMD(STRFNFC_MAX_BUFFER_SIZE, tx, rx);     
    
    len = rx[STRFNFC_LEN_OFFSET + 1] + 2;    
    
    if (len > STRFNFC_MAX_BUFFER_SIZE)
    {
        len = STRFNFC_MAX_BUFFER_SIZE;
    }
    
    for (i = 0; i < len; ++i)
    {
        res[i] = rx[i+1];
    }    
}

void io_strfnfc_reset(void)
{
    nfc_context.state = NFC_STATE_INIT;
    nfc_context.prev_cmd = STRFNFC_NONE;     
    drv_nfc_stop();
    drv_nfc_start();    
}
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------

void drv_nfc_reply(nfc_reply_type_t reply_type, uint8_t *dat, uint8_t size)
{
    uint8_t reply_buf[STRFNFC_MAX_BUFFER_SIZE];
    uint8_t i;
     
    switch (reply_type)
    {
        case NFC_REPLY_ACK:
            drv_nfc_send_cmd(strfnfc_ack_cmd);
            break;
        case NFC_REPLY_NACK:
            drv_nfc_send_cmd(strfnfc_nack_cmd);
            break;
        case NFC_REPLY_DATA:
            reply_buf[0]  = STRFNFC_SEND;
            reply_buf[1]  = size + 1;
            reply_buf[size + 2] = (0x08 | (1<<5));
            
            for(i = 0; i < size; i++)
            {
                reply_buf[i + 2] = dat[i];
            }
            
            drv_nfc_send_cmd(reply_buf);
            break;
        default:
            break;
    }
}

static void inactivity_listen_handler(void* p_context)
{
     drv_nfc_gpio_int_handler();
}

static void send_pollfield_handler(void* p_context)
{
    drv_nfc_send_cmd(strfnfc_pollfield_cmd);
}

static void on_nfc_response_in_init(nfc_context_t *context)
{

    if (listen_timer_running)
    {
        uint32_t err_code;
        err_code = app_timer_stop(s_timer_id[APP_TIMER_LISTEN]);
        APP_ERROR_CHECK(err_code);
        listen_timer_running = false;
    }
    
    switch (context->prev_cmd)
    {
        case 0:
        case STRFNFC_ECHO:
        case STRFNFC_NONE:
            if (nfc_context.response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_ECHO)
            {
                drv_nfc_send_cmd(strfnfc_protsel_cmd);
            }
            else
            {
                io_strfnfc_reset();
            }
            break;
            
        case STRFNFC_IDN:
            // Check IDN and send next cmd
            if ( (context->response[STRFNFC_CMD_OFFSET]   == STRFNFC_CMD_SUCCESS) && 
                 (context->response[STRFNFC_LEN_OFFSET] == 0x0F                 ) )
            {
                drv_nfc_send_cmd(strfnfc_protsel_cmd);
            }
            else
            {
                drv_nfc_send_cmd(strfnfc_idn_cmd);    
            }
            break; 

        case STRFNFC_PROTSEL:
            if ( context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_SUCCESS )
            {
                drv_nfc_send_cmd(strfnfc_acfilter_cmd);
            }
            ELSE_ASSERT()
            break;

        case STRFNFC_ACFILTER:
            if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_SUCCESS)
            {
                drv_nfc_send_cmd(strfnfc_pollfield_cmd);                
            }
            ELSE_ASSERT()
            break;       

        case STRFNFC_POLLFIELD:      
            if (context->response[STRFNFC_DAT_OFFSET] == STRFNFC_DAT_FIELD) 
            {
                // Field detected
                nfc_pollfield_counter = 0;
                drv_nfc_send_cmd(strfnfc_listen_cmd);
            }
            else
            {
                 if (nfc_pollfield_counter < STRFNFC_MAX_POLLFIELD_NUM)
                 {
                     app_timer_start(s_timer_id[APP_TIMER_POLLFIELD], 145/*163*/, 0);
                     nfc_pollfield_counter++;                
                 }
                 else
                 {                   
                    nfc_pollfield_counter = 0;
                    drv_nfc_send_cmd(strfnfc_idle_cmd);
                     
                 }
            }
            break;
            
        case STRFNFC_IDLE:
            if ( (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_SUCCESS) && (context->response[STRFNFC_LEN_OFFSET] == 0x01) )
            {
                drv_nfc_send_cmd(strfnfc_protsel_cmd);
            }
            ELSE_ASSERT()
            break;

        case STRFNFC_LISTEN:
            if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_SUCCESS)
            {
                uint32_t err_code;
                context->state = NFC_STATE_LISTEN;
                
                if(listen_timer_running == false)
                {
                    err_code = app_timer_start(s_timer_id[APP_TIMER_LISTEN], APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER), 0);
                    APP_ERROR_CHECK(err_code);
                    listen_timer_running = true;
                }
            }
            else if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_NOFIELD)
            {
                drv_nfc_send_cmd(strfnfc_pollfield_cmd);
            }
            else if (context->response[STRFNFC_CMD_OFFSET] == 0x80)
            {
                
            }
            ELSE_ASSERT()
            break;

        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

static void on_nfc_response_in_listen(nfc_context_t *context)
{
    
    if (listen_timer_running)
    {
        uint32_t err_code;
        err_code = app_timer_stop(s_timer_id[APP_TIMER_LISTEN]);
        APP_ERROR_CHECK(err_code);
        listen_timer_running = false;
    }
    
    if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_DATA)
    {
        if (s_callback(&context->response[STRFNFC_DAT_OFFSET]) != NRF_SUCCESS)
        {
            drv_nfc_send_cmd(strfnfc_listen_cmd);
            nfc_context.state = NFC_STATE_INIT;
        }
    }
    else if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_NOFIELD)
    {
        drv_nfc_send_cmd(strfnfc_pollfield_cmd);
        
        nfc_context.state = NFC_STATE_INIT;
    }
    else if (context->response[STRFNFC_CMD_OFFSET] == STRFNFC_CMD_SUCCESS)
    {
        if (context->prev_cmd == STRFNFC_SEND)
        {
            drv_nfc_send_cmd(strfnfc_listen_cmd);
            nfc_context.state = NFC_STATE_INIT;
        }
        else if (context->prev_cmd == STRFNFC_LISTEN)
        {     
            drv_nfc_send_cmd(strfnfc_echo_cmd);
        }
        else
        {         
            drv_nfc_send_cmd(strfnfc_listen_cmd);
            nfc_context.state = NFC_STATE_INIT;
        }
    }
    else
    {     
        drv_nfc_send_cmd(strfnfc_listen_cmd);
        nfc_context.state = NFC_STATE_INIT;           
    }
}

static void drv_nfc_resp_handler(void * p_event_data, uint16_t event_size)
{
    drv_nfc_get_response(nfc_context.response);    
    
	switch (nfc_context.state)
	{
		case NFC_STATE_INIT:
            on_nfc_response_in_init(&nfc_context);
            break;
 
        case NFC_STATE_LISTEN:
            on_nfc_response_in_listen(&nfc_context);
            break;
      
    	default:
            APP_ERROR_CHECK_BOOL(false);
    	    break;
	}  
}
