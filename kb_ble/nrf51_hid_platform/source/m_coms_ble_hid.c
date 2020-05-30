#include "m_coms_ble_hid.h"
#include "m_coms_ble_hid_cfg.h"

#include <string.h>

#include "ble_flash.h"
#include "ble_hids.h"
#include "nrf_assert.h"
#include "nrf_error.h"

/**@brief HID report map (HID USB descriptor) */
typedef struct
{
    uint8_t * report_map;     /** Pointer to the actual descriptor */
    uint16_t  report_map_len; /** Length of descriptor */
    uint8_t   interface_idx;  /** ID to distinguish between multiple HID services */
    uint8_t   boot_type;      /** Indicates if descriptor contains boot reports */
} report_map_data_t;

/**@brief Database record containing HID report map info */
typedef union
{
    report_map_data_t rec_data; /** HID report map */
    uint32_t          padding[CEIL_DIV(sizeof(report_map_data_t), 4)]; /** To make sure size is a multiple of 32 bits */
} ble_hid_report_map_record_t;

/**@brief Information describing a single report in a HID report map */
typedef struct
{
    uint8_t                  interface_idx;/** ID to distinguish between multiple HID services */
    m_coms_hid_report_type_t report_type;  /** Type (input/output/feature) */
    uint8_t                  report_idx;   /** Index among similar type of reports in same interface */
    bool                     read_resp;    /** Should application respond to read operations */
    uint8_t                  report_id;    /** Report ID as stated in associated HID report map */
    uint8_t                  report_len;   /** Length of report */
} report_data_t;

/**@brief Database record containing HID report information. */
typedef union
{
    report_data_t rec_data; /** HID report description */
    uint32_t      padding[CEIL_DIV(sizeof(report_data_t), 4)]; /** To make sure size is a multiple of 32 bits */
} ble_hid_report_record_t;

/**@brief Mapping information between a HID report and another BLE service */
typedef struct
{
    uint8_t                  interface_idx; /** ID to distinguish between multiple HID services */
    uint16_t                 external_char_uuid;    /** UUID of characteristic in the other BLE service */
} external_map_t;

/**@brief Database record containing external service mapping information */
typedef union
{
    external_map_t rec_data; /** External mapping information */
    uint32_t       padding[CEIL_DIV(sizeof(external_map_t), 4)]; /** To make sure size is a multiple of 32 bits */
} ble_hid_ext_map_record_t;

/**@brief Database layout */
typedef struct
{
    ble_hid_report_map_record_t report_maps[BLE_HID_MAX_INTERFACE_NUM];
    ble_hid_report_record_t     reports[BLE_HID_MAX_REPORT_NUM];
    ble_hid_ext_map_record_t    ext_mappings[BLE_HID_MAX_EXT_MAPPINGS];
} ble_hid_db_t;

STATIC_ASSERT(sizeof(ble_hid_report_map_record_t) % 4 == 0);
STATIC_ASSERT(sizeof(ble_hid_report_record_t) % 4 == 0);
STATIC_ASSERT(sizeof(ble_hid_ext_map_record_t) % 4 == 0);

//
// FLASH DB LAYOUT: 
//
// <Beginning of db>
// ble_hid_db_t 
// Report map #1 
// Report map #2
// ...
// Report map #BLE_HID_MAX_INTERFACE_NUM
// <End of db>
//
// NOTE: ble_hid_db_t and the report maps will require a factor of sizeof(uint32_t) flash space, even though the maps themselves are smaller
//



static m_coms_hid_evt_handler_t   s_evt_handler;
static ble_srv_error_handler_t    s_error_handler;
static ble_hid_db_t *             s_hid_db;
static uint16_t                   s_hid_db_size; // Size of db in bytes
static uint8_t                    s_num_interfaces;
static uint8_t                    s_num_reports;
static uint8_t                    s_num_ext_mappings;
static ble_hids_t                 s_interfaces[BLE_HID_MAX_INTERFACE_NUM];
static ble_hids_hid_information_t s_hid_info;
static bool                       s_with_mitm;

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt/*, ble_evt_t * p_ble_evt*/)
{
    m_coms_ble_hid_evt_t evt;
    
    evt.interface_idx = 0xFF;
    evt.report_idx    = 0xFF;
    evt.report_type   = 0xFF;
    
    // Translating from ble_hids_t* to interface index
    for (int_fast8_t i = 0; i < s_num_interfaces; ++i)
    {
        if (p_hids == &s_interfaces[i])
        {
            evt.interface_idx = i;
            break;
        }
    }
    
    APP_ERROR_CHECK_BOOL(evt.interface_idx != 0xFF);
    
    
    evt.hids_evt = p_evt;
    evt.len      = 0;    
    
    if (p_evt->evt_type == BLE_HIDS_EVT_REP_CHAR_WRITE)
    {
        evt.report_idx  = p_evt->params.char_write.char_id.rep_index;
        evt.report_type =  p_evt->params.char_write.char_id.rep_type;
        evt.data        = p_evt->params.char_write.data;
        evt.len         = p_evt->params.char_write.len;
    }
    else if (p_evt->evt_type == BLE_HIDS_EVT_REPORT_READ)
    {
        evt.report_idx = p_evt->params.char_auth_read.char_id.rep_index;
        evt.report_type = p_evt->params.char_auth_read.char_id.rep_type;
    }
    
    s_evt_handler(&evt);
}

static void security_mode_cccd_set(ble_srv_cccd_security_mode_t * p_sec_mode, bool p_mitm_use)
{
    if (p_mitm_use)
    {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_sec_mode->cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_sec_mode->read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_sec_mode->write_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_sec_mode->cccd_write_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_sec_mode->read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_sec_mode->write_perm);
    }
}

static void security_mode_set(ble_srv_security_mode_t * p_sec_mode, bool p_read, bool p_write, bool p_mitm_use)
{
    if (p_read)
    {
        if (p_mitm_use)
        {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_sec_mode->read_perm);
        }
        else
        {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_sec_mode->read_perm);
        }
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&p_sec_mode->read_perm);
    }
    
    if (p_write)
    {
        if (p_mitm_use)
        {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&p_sec_mode->write_perm);
        }
        else
        {
            BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_sec_mode->write_perm);
        }
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&p_sec_mode->write_perm);
    }    
}

static uint8_t report_idx_determine(uint8_t p_interface_idx, m_coms_hid_report_type_t p_rep_type)
{
    uint8_t rep_count = 0;
    
    // Find out how many reports are belonging to the same interface and is of the same type.
    // This will tell which index the new report will have.
    for (int i = 0; i < s_num_reports; ++i)
    {
        if (s_hid_db->reports[i].rec_data.interface_idx != p_interface_idx || 
            s_hid_db->reports[i].rec_data.report_type   != p_rep_type)
        {
            continue;
        }
        
        ++rep_count;
    }
    
    return rep_count;
}

static int8_t find_in_array(uint8_t * p_elem, uint8_t * p_arr, uint8_t p_size, uint8_t p_len)
{
    for (int i = 0; i < p_len; ++i)
    {
        if (memcmp(p_elem, &p_arr[i * p_size], p_size) == 0)
        {
            return i;
        }
    }
    
    return -1;
}


static int8_t find_report_record(ble_hid_report_record_t *new_record)
{
    for (int i = 0; i < s_num_reports; ++i)
    {
        if ( (new_record->rec_data.interface_idx==s_hid_db->reports[i].rec_data.interface_idx) && \
             (new_record->rec_data.read_resp==s_hid_db->reports[i].rec_data.read_resp) && \
             (new_record->rec_data.report_id==s_hid_db->reports[i].rec_data.report_id) && \
             (new_record->rec_data.report_len==s_hid_db->reports[i].rec_data.report_len) && \
             (new_record->rec_data.report_type==s_hid_db->reports[i].rec_data.report_type) )
        {
            return s_hid_db->reports[i].rec_data.report_idx;
        }
    }
    
    return -1;
}

uint32_t m_coms_ble_hid_init(const m_coms_ble_hid_init_t * p_params)
{
    ble_hid_report_map_record_t empty_map_record;
    ble_hid_report_record_t     empty_report_record;
    ble_hid_ext_map_record_t    empty_ext_map_record;
    
    if (p_params->evt_handler    == 0 ||
        p_params->error_handler  == 0 ||
        p_params->base_hid_version == 0 || 
        p_params->db_loc         == 0 ||
        p_params->db_size        == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    if (sizeof(ble_hid_db_t) > p_params->db_size)
    {
        // HID database has grown beyond flash storage 
        return NRF_ERROR_NO_MEM;
    }
    
    
    s_evt_handler             = p_params->evt_handler;
    s_error_handler           = p_params->error_handler;
    s_hid_db                  = (ble_hid_db_t *) p_params->db_loc;
    s_hid_db_size             = p_params->db_size;
    s_num_interfaces          = 0;
    s_num_reports             = 0;
    s_num_ext_mappings        = 0;
    s_hid_info.bcd_hid        = p_params->base_hid_version;
    s_hid_info.b_country_code = p_params->b_country_code;
    s_hid_info.flags          = p_params->flags;
    s_with_mitm               = p_params->io_capabilities == BLE_GAP_IO_CAPS_NONE ? false : true;
    
    if (s_with_mitm)
    {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&s_hid_info.security_mode.read_perm);
    }
    else
    {
        BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&s_hid_info.security_mode.read_perm);
    }
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&s_hid_info.security_mode.write_perm);
    
    // Find number of interfaces already in database
    memset(&empty_map_record, 0xFF, sizeof(empty_map_record));
    for (int i = 0; i < BLE_HID_MAX_INTERFACE_NUM; ++i)
    {
        if (memcmp(&s_hid_db->report_maps[i], &empty_map_record, sizeof(empty_map_record)) != 0)
        {
            // Report map is not flash erase value (0xff) = it is valid
            ++s_num_interfaces;
        }
    }
    
    // Find number of reports already in database
    memset(&empty_report_record, 0xFF, sizeof(empty_report_record));
    for (int i = 0; i < BLE_HID_MAX_REPORT_NUM; ++i)
    {
        if (memcmp(&s_hid_db->reports[i], &empty_report_record, sizeof(empty_report_record)) != 0)
        {
            ++s_num_reports;
        }
    }
    
    // Find number of external mappings in database
    memset(&empty_ext_map_record, 0xFF, sizeof(empty_ext_map_record));
    for (int i = 0; i < BLE_HID_MAX_EXT_MAPPINGS; ++i)
    {
        if (memcmp(&s_hid_db->ext_mappings[i], &empty_ext_map_record, sizeof(empty_ext_map_record)) != 0)
        {
            ++s_num_ext_mappings;
        }
    }
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_hid_report_descriptor_add(const uint8_t * p_descriptor, 
                                              uint16_t        p_descriptor_len, 
                                              uint8_t         p_boot_type_bitmask, 
                                              uint8_t *       p_interface_idx)
{
    uint8_t *                   map_dst;
    ble_hid_report_map_record_t new_map_record;
    
    if (p_descriptor == 0 || p_descriptor_len == 0 || p_interface_idx == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    // Check that buffer is correctly aligned
    if (!is_word_aligned((uint8_t *)p_descriptor))
    {
        return NRF_ERROR_INVALID_PARAM;
    }        
    
    // See if descriptor is in db already
    for (int i = 0; i < s_num_interfaces; ++i)
    {
        if (p_descriptor_len != s_hid_db->report_maps[i].rec_data.report_map_len)
        {
            continue;
        }
        
        if (memcmp(p_descriptor, s_hid_db->report_maps[i].rec_data.report_map, p_descriptor_len) == 0)
        {
            // Already have this interface in database: no need to do anything
            *p_interface_idx = i;
            return NRF_SUCCESS;
        }
    }
    
    
    // Store the report map in the db
    // Need to calculate where to store it, and if we have enough room left
    map_dst  = (uint8_t *) (s_hid_db); // Start of db
    map_dst += CEIL_DIV(sizeof(ble_hid_db_t), 4) * 4; // Size of db layout struct 
    for (int i = 0; i < s_num_interfaces; ++i)
    {
        map_dst += CEIL_DIV(s_hid_db->report_maps[i].rec_data.report_map_len, 4) * 4; // Size of HID descriptor
    }

    if (((uint32_t) (map_dst + p_descriptor_len)) > (s_hid_db_size + (uint32_t) s_hid_db))
    {
        // Not enough room for descriptor: Increase the BLE_HID_MAX_INTERFACE_NUM define
        return NRF_ERROR_NO_MEM;
    }
    
    // Write map record to flash
    new_map_record.rec_data.boot_type      = p_boot_type_bitmask;
    new_map_record.rec_data.interface_idx  = s_num_interfaces;
    new_map_record.rec_data.report_map     = map_dst;
    new_map_record.rec_data.report_map_len = p_descriptor_len;
    
    *p_interface_idx = s_num_interfaces;
    
    ble_flash_block_write((uint32_t *) &s_hid_db->report_maps[s_num_interfaces], 
                          (uint32_t *) &new_map_record, 
                           CEIL_DIV(sizeof(new_map_record), 4));
    ++s_num_interfaces;
    
    // Write HID descriptor to flash.
    // Extra bytes are written for descriptors with length not a factor of sizeof(uint32_t)
    APP_ERROR_CHECK_BOOL((CEIL_DIV(p_descriptor_len, 4) * 4 + (uint32_t) p_descriptor) <= (NRF_FICR->CODEPAGESIZE * NRF_FICR->CODESIZE));
    ble_flash_block_write((uint32_t *) map_dst, 
                          (uint32_t *) p_descriptor, 
                           CEIL_DIV(p_descriptor_len, 4));
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_hid_report_id_map(uint8_t                  p_interface_idx, 
                                      m_coms_hid_report_type_t p_report_type, 
                                      bool                     p_read_resp,
                                      uint8_t                  p_report_id, 
                                      uint8_t                  p_report_len,
                                      uint8_t *                p_report_idx)
{
    ble_hid_report_record_t new_report_record;
    uint8_t                 tmp_report_idx;
    int8_t                  rec_idx;
    
    if (p_interface_idx >= s_num_interfaces || p_report_idx == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    *p_report_idx = report_idx_determine(p_interface_idx, p_report_type);
    
    memset(&new_report_record, 0, sizeof(new_report_record));
    new_report_record.rec_data.interface_idx = p_interface_idx;
    new_report_record.rec_data.read_resp     = p_read_resp;
    new_report_record.rec_data.report_type   = p_report_type;
    tmp_report_idx=*p_report_idx;
   
    new_report_record.rec_data.report_idx    =tmp_report_idx;
    new_report_record.rec_data.report_id     = p_report_id;
    new_report_record.rec_data.report_len    = p_report_len;
    
    //look for match with previously stored report record
    rec_idx = find_report_record(&new_report_record);
    
    if (rec_idx != -1)
    {
        // Already have this mapping in db
        *p_report_idx=rec_idx;
        return NRF_SUCCESS;
    }
    
    // See if there is room for this record
    //debug_print(dbg_BLE_HID_MAX_REPORT_NUM, (uint8_t *)&s_num_reports, 1);
    if (s_num_reports >= BLE_HID_MAX_REPORT_NUM)
    {
        // Not enough room: Increase the BLE_HID_MAX_REPORT_NUM define
        return NRF_ERROR_NO_MEM;
    }
    
    
    // Write record to flash
    ble_flash_block_write((uint32_t *) &s_hid_db->reports[s_num_reports],
                          (uint32_t *) &new_report_record,
                          CEIL_DIV(sizeof(new_report_record), 4));
    ++s_num_reports;
    
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_hid_report_id_map_external(uint8_t  p_interface_idx, 
                                               uint16_t p_external_char_uuid)
{
    ble_hid_ext_map_record_t ext_map;
    int8_t                   rec_idx;
    
    memset(&ext_map, 0, sizeof(ext_map));
    ext_map.rec_data.external_char_uuid    = p_external_char_uuid;
    ext_map.rec_data.interface_idx         = p_interface_idx;
    
    rec_idx = find_in_array((uint8_t *)&ext_map, 
                            (uint8_t *)s_hid_db->ext_mappings, 
                            sizeof(ext_map), 
                            s_num_ext_mappings);
    
    if (rec_idx != -1)
    {
        return NRF_SUCCESS;
    }
    
    if (s_num_ext_mappings >= BLE_HID_MAX_EXT_MAPPINGS)
    {
        // Not enough room in db: Increase the BLE_HID_MAX_EXT_MAPPINGS define
        return NRF_ERROR_NO_MEM;
    }
    
    // Write db record to flash
    ble_flash_block_write((uint32_t *) &s_hid_db->ext_mappings[s_num_ext_mappings],
                          (uint32_t *) &ext_map,
                          CEIL_DIV(sizeof(ext_map), 4));
    
    ++s_num_ext_mappings;
    
    return NRF_SUCCESS;
}

uint32_t m_coms_ble_hid_enable(void)
{
    uint32_t                    err_code;
    ble_hids_init_t             hids_init_obj;
    ble_hids_inp_rep_init_t     inp_reps[BLE_HID_MAX_REPORT_NUM];    
    ble_hids_outp_rep_init_t    outp_reps[BLE_HID_MAX_REPORT_NUM];
    ble_hids_feature_rep_init_t feature_reps[BLE_HID_MAX_REPORT_NUM];
    ble_uuid_t                  ext_rep_refs[BLE_HID_MAX_REPORT_NUM];
    
    memset(inp_reps, sizeof(inp_reps), 0);
    memset(outp_reps, sizeof(outp_reps), 0);
    memset(feature_reps, sizeof(feature_reps), 0);
    
    // Warning: If the order in which reports are created is changed, make sure to update
    // report_idx_determine() to match the new behavior. 
    // This function estimates which index a report will have once the service is created.
    
    for (int interface_idx = 0; interface_idx < s_num_interfaces; ++interface_idx)
    {
        uint8_t inp_rep_count     = 0;
        uint8_t outp_rep_count    = 0;
        uint8_t feat_rep_count    = 0;
        uint8_t ext_rep_ref_count = 0;
        bool    keyboard_boot     = false;
        bool    mouse_boot        = false;
        
        // Are boot modes defined in HID report map?
        if (s_hid_db->report_maps[interface_idx].rec_data.boot_type & ble_boot_pkt_keyboard)
        {
            keyboard_boot = true;
        }
        if (s_hid_db->report_maps[interface_idx].rec_data.boot_type & ble_boot_pkt_mouse)
        {
            mouse_boot = true;
        }
        
        // Filling out report structs
        for (int rep_idx = 0; rep_idx < s_num_reports; ++rep_idx)
        {
            report_data_t * report;
            
            report = &s_hid_db->reports[rep_idx].rec_data;
            
            if (report->interface_idx != interface_idx)
            {
                // This report doesn't belong to this service
                continue;
            }
            
            if (report->report_type == hid_report_type_input)
            {
                inp_reps[inp_rep_count].max_len             = report->report_len;
                inp_reps[inp_rep_count].rep_ref.report_id   = report->report_id;
                inp_reps[inp_rep_count].rep_ref.report_type = report->report_type;
                inp_reps[inp_rep_count].read_resp           = report->read_resp;
                
                security_mode_cccd_set(&inp_reps[inp_rep_count].security_mode, s_with_mitm);
                
                ++inp_rep_count;
            }
            else if (report->report_type == hid_report_type_output)
            {
                outp_reps[outp_rep_count].max_len             = report->report_len;
                outp_reps[outp_rep_count].rep_ref.report_id   = report->report_id;
                outp_reps[outp_rep_count].rep_ref.report_type = report->report_type;
                outp_reps[outp_rep_count].read_resp           = report->read_resp;
                
                security_mode_cccd_set(&outp_reps[outp_rep_count].security_mode, s_with_mitm);
                
                ++outp_rep_count;
            }
            else if (report->report_type == hid_report_type_feature)
            {
                feature_reps[feat_rep_count].max_len             = report->report_len;
                feature_reps[feat_rep_count].rep_ref.report_id   = report->report_id;
                feature_reps[feat_rep_count].rep_ref.report_type = report->report_type;
                feature_reps[feat_rep_count].read_resp           = report->read_resp;
                
                security_mode_cccd_set(&feature_reps[feat_rep_count].security_mode, s_with_mitm);
                
                ++feat_rep_count;
            }
            else
            {
                return NRF_ERROR_INTERNAL;
            }
        }
        
        // Filling out external service references
        for (int ext_ref_idx = 0; ext_ref_idx < s_num_ext_mappings; ++ext_ref_idx)
        {
            external_map_t * ext_rep_ref;
            
            ext_rep_ref = &s_hid_db->ext_mappings[ext_ref_idx].rec_data;
            
            if (ext_rep_ref->interface_idx != interface_idx)
            {
                // This external report reference does not belong to this service
                continue;
            }
            
            ext_rep_refs[ext_rep_ref_count].type = BLE_UUID_TYPE_BLE;
            ext_rep_refs[ext_rep_ref_count].uuid = ext_rep_ref->external_char_uuid;
            
            ++ext_rep_ref_count;
        }
        
        memset(&hids_init_obj, 0, sizeof(hids_init_obj));
        hids_init_obj.evt_handler                    = on_hids_evt;
        hids_init_obj.error_handler                  = s_error_handler;
        hids_init_obj.is_kb                          = keyboard_boot;
        hids_init_obj.is_mouse                       = mouse_boot;
        hids_init_obj.inp_rep_count                  = inp_rep_count;
        hids_init_obj.p_inp_rep_array                = inp_reps;
        hids_init_obj.outp_rep_count                 = outp_rep_count;
        hids_init_obj.p_outp_rep_array               = outp_reps;
        hids_init_obj.feature_rep_count              = feat_rep_count;
        hids_init_obj.p_feature_rep_array            = feature_reps;
        hids_init_obj.rep_map.data_len               = s_hid_db->report_maps[interface_idx].rec_data.report_map_len;
        hids_init_obj.rep_map.p_data                 = s_hid_db->report_maps[interface_idx].rec_data.report_map;
        hids_init_obj.rep_map.p_ext_rep_ref          = ext_rep_refs;
        hids_init_obj.rep_map.ext_rep_ref_num        = ext_rep_ref_count;
        hids_init_obj.included_services_count        = 0;
        hids_init_obj.p_included_services_array      = NULL;
        memcpy(&hids_init_obj.hid_information, &s_hid_info, sizeof(s_hid_info));
        
        // Configuring security modes
        security_mode_set(&hids_init_obj.security_mode_protocol, true, true, s_with_mitm);
        security_mode_set(&hids_init_obj.security_mode_ctrl_point, false, true, s_with_mitm);
        security_mode_set(&hids_init_obj.rep_map.security_mode, true, false, s_with_mitm);
        
        if (keyboard_boot)
        {
            security_mode_set(&hids_init_obj.security_mode_boot_kb_outp_rep, true, true, s_with_mitm);
            security_mode_cccd_set(&hids_init_obj.security_mode_boot_kb_inp_rep, s_with_mitm);
        }
        
        if (mouse_boot)
        {
            security_mode_cccd_set(&hids_init_obj.security_mode_boot_mouse_inp_rep, s_with_mitm);
        }
        
        err_code = ble_hids_init(&s_interfaces[interface_idx], &hids_init_obj);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    
    return NRF_SUCCESS;
}

void m_coms_ble_hid_on_evt(ble_evt_t * p_ble_evt)
{
    for (uint_fast8_t i = 0; i < s_num_interfaces; ++i)
    {     
        ble_hids_on_ble_evt(&s_interfaces[i], p_ble_evt);
    }
}


uint32_t m_coms_ble_hid_input_report_send(uint8_t   p_interface_idx, 
                                          uint8_t   p_report_idx, 
                                          uint8_t * p_data, 
                                          uint8_t   p_len)
{
    return ble_hids_inp_rep_send(&s_interfaces[p_interface_idx], p_report_idx, p_len, p_data);
}

uint32_t m_coms_ble_hid_keyboard_boot_report_send(const m_coms_hid_boot_pkt_t * p_pkt)
{
    for (uint_fast8_t i = 0; i < s_num_interfaces; ++i)
    {
        if (s_interfaces[i].boot_kb_inp_rep_handles.value_handle != 0)
        {
            return ble_hids_boot_kb_inp_rep_send(&s_interfaces[i], 
                                                 BOOT_KEYBOARD_LEN, 
                                                 (uint8_t *)p_pkt->keyboard_data.keys);
        }
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t m_coms_ble_hid_mouse_boot_report_send(const m_coms_hid_boot_pkt_t * p_pkt)
{
    for (uint_fast8_t i = 0; i < s_num_interfaces; ++i)
    {
        if (s_interfaces[i].boot_mouse_inp_rep_handles.value_handle != 0)
        {
            return ble_hids_boot_mouse_inp_rep_send(&s_interfaces[i], 
                                                    p_pkt->mouse_data.buttons, 
                                                    p_pkt->mouse_data.x_delta, 
                                                    p_pkt->mouse_data.y_delta, 
                                                    0, 
                                                    0);
        }
    }
    
    return NRF_ERROR_INVALID_STATE;
}

uint32_t m_coms_ble_hid_num_cccds_get(void)
{
    bool    keyboard_boot = false;
    bool    mouse_boot    = false;
    int32_t cccd_count    = 0;
    
    
    // Find number of CCCDs
    // Each Input Report has a CCCD, as does boot reports
    
    for (int i = 0; i < s_num_interfaces; ++i)
    {
        if (s_hid_db->report_maps[i].rec_data.boot_type & ble_boot_pkt_keyboard)
        {
            keyboard_boot = true;
        }
        if (s_hid_db->report_maps[i].rec_data.boot_type & ble_boot_pkt_mouse)
        {
            mouse_boot = true;
        }
    }
    
    for (int i = 0; i < s_num_reports; ++i)
    {
        if (s_hid_db->reports[i].rec_data.report_type == hid_report_type_input)
        {
            cccd_count += 1;
        }
    }
    
    if (keyboard_boot)
    {
        cccd_count += 1;
    }
    if (mouse_boot)
    {
        cccd_count += 1;
    }
    
    return cccd_count;
}

uint32_t m_coms_ble_hid_feature_report_set(uint8_t   p_interface_idx, 
                                          uint8_t   p_report_idx, 
                                          uint8_t * p_data, 
                                          uint16_t   p_len)
{
    return NRF_ERROR_NOT_SUPPORTED; //TODO: Add support for calling sd_ble_gatts_value_set.
//        return sd_ble_gatts_value_set(s_interfaces[p_interface_idx].feature_rep_array[p_report_idx].char_handles.value_handle,
//                                  0,
//                                  &p_len,
//                                  p_data);
}
