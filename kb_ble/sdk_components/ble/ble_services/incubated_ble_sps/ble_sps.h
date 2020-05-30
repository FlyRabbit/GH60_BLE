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
 */
 
/** @cond To make doxygen skip this file */

#ifndef BLE_SPS_H__
#define BLE_SPS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief Scan Parameters Service event type. */
typedef enum
{
    BLE_SPS_EVT_NOTIFICATION_ENABLED,                           /**< Scan Refresh notification has been enabled by the peer*/
    BLE_SPS_EVT_NOTIFICATION_DISABLED,                          /**< Scan Refresh notification has been disabled by the peer*/
    BLE_SPS_EVT_SCAN_INTERVAL_WINDOW_UPDATED                    /**< Scan Interval Window Characteristic has been written by the peer*/
} ble_sps_evt_type_t;

/**@brief Scan Interval Window type. */
typedef struct
{
    uint16_t scan_interval;                                     /**< LE Scan Interval */
    uint16_t scan_window;                                       /**< LE Scan Window */
} scan_interval_window_t;

/**@brief Scan Parameters Service event. */
typedef struct
{
    ble_sps_evt_type_t evt_type;
    union
    {
        scan_interval_window_t scan_interval_window;            /**< Scan Interval Window */
    } params;
} ble_sps_evt_t;

// Forward declaration of the ble_sps_t type for the sake of defining the ble_sps_evt_handler_t type.
typedef struct ble_sps_s ble_sps_t;

/**@brief Type definition for Scan Parameter Service event handler. */
typedef void (*ble_sps_evt_handler_t) (ble_sps_t* p_sps, ble_sps_evt_t *p_evt);

/**@brief Scan Parameters Service init structure. This contains all options and data needed for initialization of the service. */
typedef struct
{
    ble_sps_evt_handler_t        evt_handler;                   /**< Event handler to be called on events in the Scan Parameters Service */
    bool                         is_scan_refresh_supported;     /**< Must be set to TRUE if the Scan Refresh Characteristic is to be supported */
    scan_interval_window_t *     p_initial_scan_int_window;     /**< Pointer to the initial value of scan interval window. Will be used when creating this characteristic in the local database. */
    uint8_t                      initial_scan_refresh;          /**< Initial value of scan refresh characteristic. Will be used when creating this characteristic in the local database.  */
    ble_srv_cccd_security_mode_t scan_refresh_attr_md;          /**< Initial security level for Scan Refresh Characteristics attributes in Scan Parameters Service . */
    ble_srv_security_mode_t      scan_interval_window_attr_md;  /**< Initial security level for Scan Interval Window Characteristics attributes in Scan Parameters Service . */
} ble_sps_init_t;

/**@brief Scan Parameters Service structure. This contains information used by the service code to identify an instance of service. */
typedef struct ble_sps_s
{
    ble_sps_evt_handler_t        evt_handler;                   /**< Event handler to be called on events in the Scan Parameters Service */
    uint16_t                     service_handle;                /**< Handle of Scan Parameters Service (as provided by the BLE stack) */
    ble_gatts_char_handles_t     scan_interval_window_handles;  /**< Handles related to the Scan Interval Window characteristic (as provided by the BLE stack) */
    ble_gatts_char_handles_t     scan_refresh_handles;          /**< Handles related to the Scan Refresh characteristic (as provided by the BLE stack)*/
    uint16_t                     conn_handle;                   /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection) */
    bool                         is_notification_enabled;       /**< TRUE if Scan Refresh notification is currently enabled by the peer*/
} ble_sps_t;


/**@brief Function for initializing the Scan Parameters Service.
 *
 * @details This call allows the application to initialize the Scan Parameters service.
 *
 * @param[in, out]   p_sps       Scan Parameters Service structure used to refer to the instance of the service. The memory
 *                               for this structure should be allocated by the application before calling this function. This
 *                               function will fill in values into the structure.
 * @param[in]        p_sps_init  Information needed to initialize the service.
 *
 * @return           NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_sps_init(ble_sps_t * p_sps, const ble_sps_init_t * p_sps_init);

/**@brief Function for handling events from stack.
 *
 * @details Handles all events from the BLE stack of interest to the Scan Parameters Service.
 *
 * @param[in]   p_sps      Scan Parameters Service structure used to refer to the instance of the service
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_sps_on_ble_evt(ble_sps_t * p_sps, ble_evt_t * p_ble_evt);

/**@brief Function for notifying Scan Refresh characteristic to the peer.
 *
 * @details The application should call this function when it needs notify the scan refresh characteristic to the peer.
 *          If notification for Scan Refresh has been enabled by the peer, the value of Scan Refresh will be sent.
 *
 * @param[in]   p_sps          Scan Parameters Service structure used to refer to the instance of the service.
 * @param[in]   scan_refresh   Scan Refresh value.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_sps_scan_refresh_send(ble_sps_t * p_sps, uint8_t scan_refresh);

#endif // BLE_SPS_H__

/** @endcond */
