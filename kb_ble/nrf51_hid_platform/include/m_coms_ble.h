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
 * @defgroup modules_coms_ble
 * @{
 * @ingroup nrfready_modules
 * @brief BLE communications sub-module.
 *
 * @details This is a communication sub-module used to deal with BTLE and HID over GATT-specifics.
 *          
 */
#ifndef __M_COMS_BLE_H__
#define __M_COMS_BLE_H__
 
#include <stdbool.h>
#include <stdint.h>

#include "app_scheduler.h"
#include "app_util.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "ble_hids.h"
#include "device_manager.h"

#define BOOT_KEYBOARD_LEN 8 /** See HID 1.11 spec Appendix B.1 (Boot keyboard descriptor) */

/**@brief Events types generated by this module */
typedef enum
{
    M_COMS_BLE_EVT_INIT_FINISHED, /** Initialization finished, ready to enable */
    M_COMS_BLE_EVT_CONNECTED,     /** Connected */
    M_COMS_BLE_EVT_DISCONNECTED,  /** Disconnected */
    M_COMS_BLE_EVT_DATA_RECEIVED, /** Data received */
    M_COMS_BLE_EVT_READ_REQ,      /** Read request received */
    M_COMS_BLE_EVT_CONN_UPDATE,   /** Connection parameter update */
    M_COMS_BLE_EVT_ADV_TIMEOUT,   /** Advertising has timed out */
    M_COMS_BLE_EVT_ADV_BONDABLE,  /** Bondable advertising is running */
    M_COMS_BLE_EVT_ADDR_CHANGED,  /** Address has changed */
    M_COMS_BLE_EVT_PASSKEY_REQ,   /** Passkey requested */
    M_COMS_BLE_EVT_OOBKEY_REQ,    /** Passkey requested */    
    M_COMS_BLE_EVT_KEY_SENT,      /** Pass or OOB key sent */
    M_COMS_BLE_EVT_DISABLED
} m_coms_ble_evt_type_t;

/**@brief Data received event details */
typedef struct
{
    uint8_t   interface_idx; /** Which interface data was received on */
    uint8_t   report_type;   /** Which type of report is it. Input, output or feature */
    uint8_t   report_idx;    /** Which report index data was received on */ 
    uint8_t   len;           /** Length of received data */
    uint8_t * data;          /** Received data */
} m_coms_ble_evt_data_recv_t;

/**@brief Read request event details */
typedef struct
{
    uint8_t   interface_idx; /** Which interface read report is in */
    uint8_t   report_idx;    /** Which report index is read */ 
} m_coms_ble_evt_read_req_t;

/**@Brief Connection update event details */
typedef struct
{
    uint16_t min_conn_interval;   /** Min Connection interval [1.25 ms units] */
    uint16_t max_conn_interval;   /** Max Connection interval [1.25 ms units] */
    uint16_t slave_latency;       /** Slave latency */
    uint16_t supervision_timeout; /** Link timeout [10 ms units] */
} m_coms_ble_evt_conn_update_t;

/**@brief Event structs generated by this module */
typedef struct
{
    m_coms_ble_evt_type_t type;
    union
    {
        m_coms_ble_evt_data_recv_t    data_received;
        m_coms_ble_evt_read_req_t     read_req;
        m_coms_ble_evt_conn_update_t  conn_update;
    } data;
} m_coms_ble_evt_t;

/**@brief Data stored in the device manager's application context */
typedef struct
{
    uint32_t       usage_idx;
    ble_gap_addr_t ble_addr;
} dm_app_data_t;

typedef union
{
    dm_app_data_t data;
    uint32_t      padding[CEIL_DIV(sizeof(dm_app_data_t), 4)]; /** To make sure size is size of n words */
}dm_application_context_data_t;

/**@brief When in BLE HID (HID over GATT) boot mode, only keyboard and mouse packets of a specified format can be sent. 
 * 
 * @note Bitmask type enum: valid values are powers of 2
 */
typedef enum
{
    ble_boot_pkt_keyboard = 0x01, /** Keyboard boot report */
    ble_boot_pkt_mouse    = 0x02, /** Mouse boot report */
    ble_boot_pkt_none     = 0x80  /** Not keyboard/mouse boot report. Will not be sent in boot mode. */
} m_coms_ble_hid_boot_type_t;

/**@brief Packet definition used for boot reports. 
 * @details Format is specified in USB HID spec, Appendix B: Boot Interface Descriptors.
 */
typedef union
{
    struct
    {
        uint8_t keys[BOOT_KEYBOARD_LEN]; /** Keyboard keycodes */
    } keyboard_data;
    struct
    {
        uint8_t buttons; /** Mouse buttons */
        int8_t  x_delta; /** Mouse cursor X delta */
        int8_t  y_delta; /** Mouse cursor Y delta */
    } mouse_data;
} m_coms_hid_boot_pkt_t;

/**@brief HID Report types */
typedef enum
{
    hid_report_type_input   = BLE_HIDS_REP_TYPE_INPUT,
    hid_report_type_output  = BLE_HIDS_REP_TYPE_OUTPUT,
    hid_report_type_feature = BLE_HIDS_REP_TYPE_FEATURE
} m_coms_hid_report_type_t;

/**@brief When in boot mode (BLE HID) a specific format is used for mouse and keyboard packets regardless of HID descriptor.
 *        THe application will be tasked to re-assemble keyboard and mouse packets to fit the specified format 
 *        (http://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.human_interface_device.xml).
 *        Non-keyboard and mouse packets will be discarded in boot mode.
 *
 * @param[in\out] p_boot_pkt      Resulting boot packet
 * @param[in]     p_pkt_type      Boot packet type
 * @param[in]     p_data          Original packet
 * @param[in]     p_len           Length of original packet
 * @param[in]     p_hid_interface Interface index packet was sent on
 * @param[in]     p_report_idx    Report index used
 */
typedef void (*m_coms_ble_boot_mode_callback)(m_coms_hid_boot_pkt_t *      p_boot_pkt, 
                                              m_coms_ble_hid_boot_type_t * p_pkt_type, 
                                              uint8_t *                    p_data, 
                                              uint8_t                      p_len, 
                                              uint8_t                      p_hid_interface, 
                                              uint8_t                      p_report_idx);

/**@brief Callback to get the BLE_EVT_TX_COMPLETE event to the application.
 *
 * @param[in]     p_ble_evt    Pointer to the BLE event structure.
 */
typedef void (*m_coms_ble_tx_complete_cb_t)(ble_evt_t * p_ble_evt);

/**@brief BLE bond behaviour parameters. 
 *
 * @note Maximum number of bonds are defined by @ref BLE_BONDMNGR_MAX_BONDED_MASTERS.
 */
typedef struct
{
    bool change_address;        /** Use a new address for each new bond */
    bool directed_adv;          /** Use directed advertising vs bonded masters (unless peer has private resolvable device address).
                                *  If false, undirected advertising with whitelist is used: this is best suited with change_address = false. */
    bool reconnect_all;         /** Attempt reconnection to all bonded masters. If false, only the last connected master will be advertised against,
                                * unless bonding procedure is started. */
    bool bond_reconnect_all;    /** Attempt to reconnect to previously bonded masters before advertising as bondable. 
                                * If false, bondable advertising is started immediately when bonding procedure is started. */
    bool delete_bonds;
    
} m_coms_ble_bond_params_t;

/**@brief BLE security parameters. */
typedef struct
{
    uint8_t io_capabilities; /** See @ref BLE_GAP_IO_CAPS. */
    bool    oob_data_availible;
} m_coms_ble_sec_params_t;

/**@brief Flash pages available for BLE parameter storage. */
typedef struct
{
    uint8_t flash_page_system_attributes; /** Flash page number for persistent connection attributes */
    uint8_t flash_page_bond_attributes;   /** Flash page number keys and addresses */
    uint8_t flash_page_addr_storage;      /** Flash page number for local address storage */
} m_coms_ble_flash_params_t;

/**@brief Device Information Service parameters
 * (see http://developer.bluetooth.org/gatt/services/Pages/ServiceViewer.aspx?u=org.bluetooth.service.device_information.xml)
 * @note All strings should be of UTF-8 format.
 * @note NULL termination works for all UTF-8 strings, including Chinese, Korean, and Japanese.
 */
typedef struct
{
    char *   device_name;       /** Null-terminated device name */
    char *   manufacturer_name; /** Null-terminated manufacturer name */
    char *   hw_revision;       /** Null-terminated hardware revision string */
    char *   fw_revision;       /** Null-terminated firmware revision string */
    char *   serial_number;     /** Null-terminated serial number string */
    uint16_t pnp_vendor_id;     /** Vendor ID */
    uint8_t  pnp_vendor_id_src; /** Vendor ID source (@ref BLE_DIS_VENDOR_ID_SRC_BLUETOOTH_SIG or @ref BLE_DIS_VENDOR_ID_SRC_USB_IMPL_FORUM) */
    uint16_t pnp_product_id;    /** Product ID */
    uint16_t pnp_product_version;/** Product version */
} m_coms_ble_device_info_t;

/**@brief BLE parameters. See @ref M_COMS_BLE_PARAMS_FILL for to initialize default values. */
typedef struct
{
    m_coms_ble_device_info_t   device_info;  /** Device information parameters */
    m_coms_ble_sec_params_t    sec_params;   /** BLE security parameters */
    m_coms_ble_bond_params_t   bond_params;  /** Bond behavior parameters */
    uint16_t                   appearance;   /** Device appearance. See @ref BLE_APPEARANCES */
    ble_gap_conn_params_t      conn_params;  /** Preferred connection parameters */
    
    uint16_t base_hid_version; /** This states which HID usage table version that this USB descriptor uses */
    uint8_t  hid_country_code; /** HID country code. Set to 0 to indicate no specific country */
    uint8_t  hids_flags;       /** HID Service flags. See section 4.10 (HID Information Behavior) of HOGP spec. */
    uint8_t  max_tx_buf_cnt;   /** Max number of packets put in SoftDevice TX buffer at a time */
    
    m_coms_ble_boot_mode_callback boot_mode_callback; /** Callback function used in HID Boot mode */
    m_coms_ble_tx_complete_cb_t   tx_complete_callback; /** Callback function used to get BLE_EVT_TX_COMPLETE event to the application in interrupt context */
} m_coms_ble_params_t;

/**@brief BLE initialization.
 *
 * @details SoftDevice, Battery service, and Device Information Service will be initialized.
 *          p_ble_params may be 0 if m_coms_ble_init() is called multiple times, as
 *          the parameters are saved in flash. The first execution must always have non-zero parameters.
 * 
 * @note SoftDevice will be enabled.
 *
 * @param[in] p_event_callback Callback used to deliver events
 * @param[in] p_ble_params     BLE parameters
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_NO_MEM
 */
uint32_t m_coms_ble_init(const app_sched_event_handler_t p_event_callback, const m_coms_ble_params_t * p_ble_params);

/**@brief Set passkey for encrypted MITM connections.
 *
 * @note The key should be in ASCII format, <b>not</b> USB HID format.
 * 
 * @param[in] p_key 6-byte ASCII string (digit 0..9 only, no NULL termination)
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_passkey_set(uint8_t * p_key);

/**@brief Set OOB key for encrypted OOB connections.
 *
 * @note The key should be 128 bit in a 16 bytes array.
 * 
 * @param[in] p_key 16-byte byte-array
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_oobkey_set(uint8_t * p_key);

/**@brief Get BLE address currently used
 *
 * @param[out] p_ble_addr Pointer to address struct
 * @return NRF_SUCCESS
 * @return NRF_ERROR_INVALID_ADDR
 */
uint32_t m_coms_ble_addr_get(ble_gap_addr_t * p_ble_addr);

/**@brief Start bonding procedure.
 *
 * @details Advertising for a new bond will be started according to parameters (@ref m_coms_ble_init).
 * 
 * @note Will cause a disconnect if connected.
 *
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_STATE
 * @retval NRF_ERROR_INTERNAL
 */
uint32_t m_coms_ble_bonding_start(void);

/**@brief Add new BLE HID (HID over GATT) descriptor
 * e
 * @note USB Dongle interface is not affected
 *
 * @param[in]  p_descriptor         Pointer to USB HID report descriptor.
 * @param[in]  p_descriptor_len     Length of descriptor.
 * @param[in]  p_boot_type_bitmask  Boot type avilable (keyboard or mouse boot device). See @ref p_boot_type_bitmask for valid types.
 * @param[out] p_interface_idx      Reference to the generated HID interface. Used in all subsequent HID operations.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_report_descriptor_add(const uint8_t * p_descriptor,
                                          uint16_t        p_descriptor_len,
                                          uint8_t         p_boot_type_bitmask,
                                          uint8_t *       p_interface_idx);
                                          
/**@brief Map each USB HID report (speficied in report descriptor) to a BLE HID (HID over GATT) characteristic.
 *
 * @details When a HID report descriptor is added (@ref m_coms_hid_report_descriptor_add), 
 *          this functon needs to be called once for each report specified in the HID report descriptor.
 * 
 * @note Gazell USB Dongle interface is not configured by this.
 *
 * @param[in]  p_interface_id Reference to HID interface (@ref m_coms_hid_report_descriptor_add)
 * @param[in]  p_report_type  HID report type
 * @param[in]  p_read_resp    If true, application will get an event and be required to respond when this Report is read
 * @param[in]  p_report_id    HID report id
 * @param[in]  p_report_len   HID report length
 * @param[out] p_report_idx   Use when sending (see @ref m_coms_ble_hid_input_report_send). Used to identify the particular report characteristic.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_report_id_map(uint8_t                  p_interface_idx, 
                                  m_coms_hid_report_type_t p_report_type, 
                                  bool                     p_read_resp,
                                  uint8_t                  p_report_id, 
                                  uint8_t                  p_report_len,
                                  uint8_t *                p_report_idx);
                                  
/**@brief Maps an external BLE service characteristic to a HID report id.
 *
 * @details In the case that the HID Report Descriptor has for example a battery level report, the HID service can be mapped to the battery service.
 * 
 * @note USB Dongle interface is not configured by this.
 * @note Only Battery Service <-> HID Service mapping is supported at this point
 *
 * @param[in] p_interface_id          Reference to HID interface (@ref m_coms_hid_report_descriptor_add)
 * @param[in] p_report_type           HID report type
 * @param[in] p_report_id             HID report id
 * @param[in] p_external_char_uuid    UUID of the external BLE characteristic which will refer to this HID report
 * @param[in] p_external_service_uuid UUID of the external BLE characteristic which will refer to this HID report
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_NOT_SUPPORTED
 */
uint32_t m_coms_ble_report_id_map_external(uint8_t                  p_interface_idx, 
                                           m_coms_hid_report_type_t p_report_type, 
                                           uint8_t                  p_report_id, 
                                           uint16_t                 p_external_char_uuid, 
                                           uint16_t                 p_external_service_uuid);

/**@brief Send HID report.
 * 
 * @note Report length cannot exceed 20 bytes.
 *
 * @param[in] p_hid_packet    HID report data to send.
 * @param[in] p_len           Length of report data
 * @param[in] p_hid_interface Interface Id generated by @ref m_coms_add_hid_descriptor.
 * @param[in] p_report_idx    Report index generated by @ref m_coms_hid_report_id_map.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_INVALID_STATE
 * @retval BLE_ERROR_NO_TX_BUFFERS
 */
// uint32_t m_coms_hid_report_send(const m_coms_hid_pkt_t * p_hid_packet, uint8_t p_hid_interface);
uint32_t m_coms_ble_hid_report_send(uint8_t * p_data,
                                    uint8_t   p_len,
                                    uint8_t   p_hid_interface,
                                    uint8_t   p_report_idx);

#ifdef USE_MULTIPLE_HID_REPS
/**@brief Send multiple HID reports in a buffer.
 * 
 * @note Each report length cannot exceed 20 bytes.
 *
 * @param[in] p_data          HID report data buffer to send.
 * @param[in] p_len           Length of buffer data
 * @param[in] p_hid_interface Interface Id generated by @ref m_coms_add_hid_descriptor.
 * @param[in] p_report_idx    Report index generated by @ref m_coms_hid_report_id_map.
 * @param[out] p_status       Status of transmission.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_INVALID_STATE
 * @retval BLE_ERROR_NO_TX_BUFFERS
 */
uint32_t m_coms_ble_multiple_hid_reports_send(const uint8_t * p_data,
                                              uint16_t  p_len,
                                              uint8_t   p_hid_interface,
                                              uint8_t   p_report_idx,
                                              bool*     p_status);
#endif /* USE_MULTIPLE_HID_REPS */

/**@brief Respond to a read request (@ref M_COMS_BLE_EVT_READ_REQ)
 *
 * 
 * @param[in] p_data Data to respond with
 * @param[in] p_len  Length of data.
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 * @retval NRF_ERROR_INVALID_STATE
 */
uint32_t m_coms_ble_read_respond(uint8_t * p_data, uint8_t p_len);

/**@brief Update battery level.
 *
 * @param[in] type        Type of data
 * @param[in] packet      The packet to send
 * @param[in] packet_size Size of the packet
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INVALID_PARAM
 */
uint32_t m_coms_ble_battery_level_update(uint8_t p_batt_level);

/**@brief Enable Services and (optionally) start advertising. Must have been initialized prior to this.
 *
 * @param[in] p_advertise If true, start advertising right away. 
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
uint32_t m_coms_ble_enable(bool p_advertise);

/**@brief Disconnect.
 *
 * @note The time it takes to disconnect properly depends on connection parameters.
 * 
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_x
 */
uint32_t m_coms_ble_disconnect(void);

/**@brief Disconnect and stop advertising.
 * 
 * @details All activity is stopped. SoftDevice is <b>not</b> disabled. Necessary bond management is also done when this happens.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_INTERNAL
 */
uint32_t m_coms_ble_disable(void);

/**@brief Prepares for wakeup from sleep.
 */
bool m_coms_ble_wakeup_prepare(bool wakeup);

/**@brief Check if bonding data is kept in flash. 
 *       
 * @note Module MUST be initialized prior to calling function
 *
 * @return True if keys from previous connection is stored in flash
 */
bool m_coms_ble_bond_stored(void);

/**@brief Delete bonding data stored in flash.
 *
 * @return
 * @retval NRF_SUCCESS
 */
uint32_t m_coms_ble_bond_clear(void);

/**@brief Update DFU application crc.
 * @return
 * @retval NRF_SUCCESS
 * @retval NRF_ERROR_x
 */
uint32_t dfu_update_crc(void);
#endif /*  __M_COMS_BLE_H__ */

/** @} */