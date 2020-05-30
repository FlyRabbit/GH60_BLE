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
#ifndef __DESKTOP_2_KEYBOARD_CFG_H__
#define __DESKTOP_2_KEYBOARD_CFG_H__

#define APP_TIMER_PRESCALER                       0    // RTC prescaler value used by app_timer
#define DEFAULT_IDLE_TIMEOT                       3    // Go to idle state after 3 seconds of inactivity
#define DEFAULT_BTLE_INACTIVITY_DISCONNECT_PERIOD 300 //Disconnect after 5 minutes of inactivity
#define DEFAULT_GZLL_INACTIVITY_DISCONNECT_PERIOD 30   //Disconnect after 30 seconds of inactivity

#endif /* __DESKTOP_2_KEYBOARD_CFG_H__ */
