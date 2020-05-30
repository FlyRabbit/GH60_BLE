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
 * @defgroup modules_mouse
 * @{
 * @ingroup nrfready_modules
 * @brief Mouse module.
 *
 * @details This module deals with the mouse sensor, buttons and scroll wheel.
 *          The figure below depicts how this module operates:
 * @image html flow_m_mouse.png Mouse module flow  
 */
#ifndef __M_MOUSE_H__
#define __M_MOUSE_H__

#define M_MOUSE_SENSOR_POLLRATE 7 // [ms]
#define M_MOUSE_BTN_POLLRATE   15 // [ms]

#include <stdint.h>

#include "app_scheduler.h"

typedef enum {
    mouse_packet_type_motion,
    mouse_packet_type_buttons,
    mouse_packet_type_scroll,
    mouse_packet_type_adv_buttons,
    mouse_packet_type_pairing_button
}mouse_packet_type_t;

typedef struct {
    mouse_packet_type_t type;
    union {
        uint8_t buttons;
        uint8_t adv_buttons;
        int8_t scroll;
        struct {
            int16_t x_delta;
            int16_t y_delta;            
        } motion;
        
    } data;
}m_mouse_data_t;

typedef struct
{
    app_sched_event_handler_t event_handler;   /** Event handler used when polling is started with enable*/
    uint32_t                  sensor_pollrate; /** Pollrate of mouse sensor in ms */
    uint32_t                  button_pollrate; /** Pollrate of button presses in ms */
}m_mouse_init_t;

/**@brief Mouse module initialization.
 * 
 * @param[in] init Initialization parameters to m_mouse module.  
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t m_mouse_init(m_mouse_init_t *init);

/**@brief Enables polling of sensor. Data is dispatched using event handler.
 */
uint32_t m_mouse_sensor_enable(void);

/**@brief Disables polling of sensor.
 */
uint32_t m_mouse_sensor_disable(void);

/**@brief Set pollrate of both mouse sensor and buttons. If polling timer is
 *        running, the pollrate will not be updated until next time the timer starts.
 *
 * @param[in] sensor_pollrate Mouse sensor pollrate in milliseconds. No change if zero.
 * @param[in] button_pollrate Mouse buttons pollrate in milliseconds. No change if zero. 
 */
void m_mouse_pollrate_set(uint32_t sensor_pollrate, uint32_t button_pollrate);

/**@brief Returns true if pairing combo is pressed
 */
bool m_mouse_is_pairing_btn_pressed(void);

/**@brief Prepare for sensor wakeup after sleep
 */
bool m_mouse_wakeup_prepare(bool wakeup);

#endif /* __M_MOUSE_H__ */

/** @} */
