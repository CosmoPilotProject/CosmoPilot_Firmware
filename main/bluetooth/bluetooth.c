/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* CosmoPilot BLE GATT server, adapted from the ESP-IDF GATT server example. */



#include "bluetooth.h"
#include "mobile_app/headers/mobile_app_commands.h"
#include "config.h"
#include "display/display.h"
#include "esp_timer.h"
#include "telescope/headers/telescope_common.h"

#define TRACE_NAME BLUETOOTH
#define TRACE_LEVEL BLUETOOTH_TRACE_LEVEL
#include "trace_conf.h"


#define GATTS_SERVICE_UUID_TEST_A   0x00FF  // 16-bit UUID for Service A (like a unique ID card for this service)
#define GATTS_CHAR_UUID_TEST_A      0xFF01  // 16-bit UUID for Characteristic in Service A (specific feature in the service)
#define GATTS_DESCR_UUID_TEST_A     0x3333  // 16-bit UUID for Descriptor in Service A (metadata about the characteristic)
#define GATTS_NUM_HANDLE_TEST_A     4       // Number of handles needed for Service A (BLE uses handles like index numbers)

#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define BLE_DEVICE_NAME             "CosmoPilot Telescope"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40    // Max allowed length for characteristic value (64 bytes)

#define BLE_INVALID_CONN_ID UINT16_MAX

/**
 * @brief   Connection subscribed to notifications, or BLE_INVALID_CONN_ID when disabled.
 */
static uint16_t s_notify_conn_id = BLE_INVALID_CONN_ID;

/**
 * @brief   Send a GATT response and trace submission failures.
 * @param   gatts_if GATT server interface.
 * @param   conn_id Connection receiving the response.
 * @param   trans_id Request transaction identifier.
 * @param   status GATT result code.
 * @param   response Optional response value.
 * @return  None.
 */
static void send_gatt_response(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id, esp_gatt_status_t status,
                               esp_gatt_rsp_t *response);

// Debounce connection-parameter updates
static bool s_conn_upd_inflight = false;
static esp_bd_addr_t s_last_peer = {0};
static esp_timer_handle_t s_conn_upd_timer = NULL;
static void conn_update_cb(void *arg);  // forward decl


static uint8_t char1_str[] = {0x11,0x22,0x33};  // Default value for characteristic (hex values 11, 22, 33)
static esp_gatt_char_prop_t a_property = 0;     // Will store properties for Service A's characteristic (write/notify)


static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),  // Actual current data length (3 bytes)
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)



static uint8_t adv_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0x0e, 0x01, 0x33, 0x21, 0x2b, 0xcc, 0xa8, 0x88, 0x63, 0x47, 0xe7, 0xff, 0x67, 0x7f, 0xe2, 0xb2
    
};




// The length of adv data must be less than 31 bytes
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,      // Not using this as scan response
    .include_name = true,       // Device name included in advertisement
    .include_txpower = false,   // Don't include TX power in advertisement
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec (6 * 1.25 ms = 7.5ms)
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec (16 * 1.25 ms = 20ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,   // This is scan response data
    .include_name = true,   // Send device name when central scans
    .include_txpower = true,    
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};



static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20, // 32 * 0.625ms = 20ms
    .adv_int_max        = 0x40, // 64 * 0.625ms = 40ms
    .adv_type           = ADV_TYPE_IND, // General connectable undirected advertisement
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,  // Use all 3 BLE adv channels
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY, // Accept any scan or connect request
};





/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst telescope_profile[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = BLE_INVALID_CONN_ID,
    }
};



//This is the callback registered in app_main() for all GAP events — it handles advertising config, start/stop events, and connection parameter updates.
void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params); // Start advertising when both adv + scan rsp are ready
        }
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            TRACE_ERR("Advertising start failed: status %d", param->adv_start_cmpl.status);
        }
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            TRACE_ERR("Advertising stop failed: status %d", param->adv_stop_cmpl.status);
        } else {
            TRACE_INF("Advertising stopped");
        }
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         s_conn_upd_inflight = false;  // Connection-parameter update completed.
        if (param->update_conn_params.status != ESP_BT_STATUS_SUCCESS)
        {
            TRACE_WAR("Connection parameter update failed: status %d", param->update_conn_params.status);
        }
         TRACE_DEB("Connection params: status %d, min %d, max %d, interval %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;

    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        if (param->pkt_data_length_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            TRACE_WAR("Packet length update failed: status %d", param->pkt_data_length_cmpl.status);
        }
        TRACE_DEB("Packet length: rx %d, tx %d, status %d",
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len,
                  param->pkt_data_length_cmpl.status);
        break;
    default:
        break;
    }
}

static void send_gatt_response(esp_gatt_if_t gatts_if, uint16_t conn_id, uint32_t trans_id, esp_gatt_status_t status,
                               esp_gatt_rsp_t *response)
{
    esp_err_t err = esp_ble_gatts_send_response(gatts_if, conn_id, trans_id, status, response);
    if (err != ESP_OK)
    {
        TRACE_ERR("GATT response failed: %s", esp_err_to_name(err));
    }
}

void ble_notify_app(uint8_t *data, size_t len)
{
    // One atomic snapshot binds subscription permission to its connection.
    uint16_t conn_id = __atomic_load_n(&s_notify_conn_id, __ATOMIC_ACQUIRE);
    if (conn_id == BLE_INVALID_CONN_ID || telescope_profile[PROFILE_A_APP_ID].gatts_if == ESP_GATT_IF_NONE)
    {
        TRACE_VER("Notification skipped: no subscribed connection");
        return;
    }

    // The link can still close after the snapshot. Do not retry on a new connection.
    esp_err_t err = esp_ble_gatts_send_indicate(telescope_profile[PROFILE_A_APP_ID].gatts_if, conn_id, telescope_profile[PROFILE_A_APP_ID].char_handle, len, data, false);
    if (err == ESP_ERR_INVALID_STATE)
    {
        TRACE_VER("Notification not sent: Bluetooth connection unavailable");
    }
    else if (err != ESP_OK)
    {
        TRACE_ERR("Notification failed: %s", esp_err_to_name(err));
    }
}





void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        TRACE_DEB("REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        telescope_profile[PROFILE_A_APP_ID].service_id.is_primary = true;
        telescope_profile[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        telescope_profile[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        telescope_profile[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        if (set_dev_name_ret){
            TRACE_ERR("Setting device name failed: %s", esp_err_to_name(set_dev_name_ret));
        }

        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            TRACE_ERR("Advertising data configuration failed: %s", esp_err_to_name(ret));
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            TRACE_ERR("Scan response configuration failed: %s", esp_err_to_name(ret));
        }
        adv_config_done |= scan_rsp_config_flag;

        esp_ble_gatts_create_service(gatts_if, &telescope_profile[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t response = {0};
        esp_gatt_status_t status = ESP_GATT_OK;
        response.attr_value.handle = param->read.handle;
        response.attr_value.offset = param->read.offset;
        if (param->read.handle == telescope_profile[PROFILE_A_APP_ID].char_handle)
        {
            status = ESP_GATT_READ_NOT_PERMIT;
        }
        else if (param->read.handle != telescope_profile[PROFILE_A_APP_ID].descr_handle)
        {
            status = ESP_GATT_INVALID_HANDLE;
        }
        else if (param->read.is_long)
        {
            // The fixed-size CCCD is not a long attribute (also enforced by the stack).
            status = ESP_GATT_NOT_LONG;
        }
        else if (param->read.offset != 0)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else
        {
            uint16_t subscribed_conn_id = __atomic_load_n(&s_notify_conn_id, __ATOMIC_ACQUIRE);
            response.attr_value.len = 2;
            response.attr_value.value[0] =
                subscribed_conn_id != BLE_INVALID_CONN_ID && subscribed_conn_id == param->read.conn_id ? 1 : 0;
            response.attr_value.value[1] = 0;
        }
        if (param->read.need_rsp)
        {
            send_gatt_response(gatts_if, param->read.conn_id, param->read.trans_id, status,
                               status == ESP_GATT_OK ? &response : NULL);
        }
        break;
    }

    case ESP_GATTS_WRITE_EVT: {
        TRACE_VER("GATT write: conn %d, transaction %" PRIu32 ", handle %d, length %d", param->write.conn_id,
                  param->write.trans_id, param->write.handle, param->write.len);

        esp_gatt_status_t status = ESP_GATT_OK;
        if (param->write.is_prep)
        {
            // Commands are single ordinary writes; never acknowledge or store fragments.
            status = ESP_GATT_REQ_NOT_SUPPORTED;
        }
        else
        {

            // Check if it's the CCCD descriptor (notifications enable/disable)
            if (telescope_profile[PROFILE_A_APP_ID].descr_handle == param->write.handle)
            {
                uint16_t active_conn_id =
                    __atomic_load_n(&telescope_profile[PROFILE_A_APP_ID].conn_id, __ATOMIC_ACQUIRE);
                if (active_conn_id == BLE_INVALID_CONN_ID || param->write.conn_id != active_conn_id)
                {
                    status = ESP_GATT_REQ_NOT_SUPPORTED;
                }
                else if (param->write.offset != 0)
                {
                    status = ESP_GATT_INVALID_OFFSET;
                }
                else if (param->write.len != 2 || param->write.value == NULL)
                {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
                else
                {
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    if (descr_value == 0x0001 && (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                    {
                        __atomic_store_n(&s_notify_conn_id, active_conn_id, __ATOMIC_RELEASE);
                        TRACE_INF("Notifications enabled by client");
                    }
                    else if (descr_value == 0x0000)
                    {
                        __atomic_store_n(&s_notify_conn_id, BLE_INVALID_CONN_ID, __ATOMIC_RELEASE);
                        TRACE_INF("Notifications disabled by client");
                    }
                    else
                    {
                        status = ESP_GATT_CCC_CFG_ERR;
                        TRACE_WAR("Unsupported notification descriptor value: 0x%04x", descr_value);
                    }
                }
            }

            // Command written to the characteristic value.
            else if (telescope_profile[PROFILE_A_APP_ID].char_handle == param->write.handle)
            {
                esp_err_t command_err = mobile_app_command_receive(param->write.value, param->write.len);
                if (command_err != ESP_OK)
                {
                    TRACE_WAR("Command rejected: %s", esp_err_to_name(command_err));
                }
                else if (param->write.len > MAX_COMMAND_LENGTH ||
                         (param->write.len == MAX_COMMAND_LENGTH && param->write.value[MAX_COMMAND_LENGTH - 1] != '\0'))
                {
                    TRACE_WAR("Command truncated to %u bytes", (unsigned) (MAX_COMMAND_LENGTH - 1));
                }
            }
            else
            {
                status = ESP_GATT_INVALID_HANDLE;
            }
        }

        if (param->write.need_rsp)
        {
            send_gatt_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT: {
        // No fragments are accepted. Cancellation is a harmless empty operation.
        esp_gatt_status_t status =
            param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_CANCEL ? ESP_GATT_OK : ESP_GATT_REQ_NOT_SUPPORTED;
        send_gatt_response(gatts_if, param->exec_write.conn_id, param->exec_write.trans_id, status, NULL);
        break;
    }

    case ESP_GATTS_MTU_EVT:
        TRACE_DEB("MTU updated: %d", param->mtu.mtu);
        break;


    case ESP_GATTS_UNREG_EVT:
        break;


    case ESP_GATTS_CREATE_EVT:
        if (param->create.status != ESP_GATT_OK)
        {
            TRACE_ERR("Service creation failed: status %d", param->create.status);
        }
        TRACE_DEB("CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        telescope_profile[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        telescope_profile[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        telescope_profile[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(telescope_profile[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(telescope_profile[PROFILE_A_APP_ID].service_handle, &telescope_profile[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            TRACE_ERR("Adding characteristic failed: %s", esp_err_to_name(add_char_ret));
        }
        break;


    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;


    case ESP_GATTS_ADD_CHAR_EVT: {
        if (param->add_char.status != ESP_GATT_OK)
        {
            TRACE_ERR("Characteristic creation failed: status %d", param->add_char.status);
        }
        uint16_t length = 0;
        const uint8_t *prf_char;

        TRACE_DEB("ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        telescope_profile[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        telescope_profile[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        telescope_profile[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
        if (get_attr_ret == ESP_FAIL){
            TRACE_ERR("Reading characteristic value failed: %s", esp_err_to_name(get_attr_ret));
        }

        TRACE_VER("Initial characteristic length: %u", (unsigned)length);
        for(int i = 0; i < length; i++){
            TRACE_VER("Initial characteristic byte %d: 0x%02x", i, prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(telescope_profile[PROFILE_A_APP_ID].service_handle, &telescope_profile[PROFILE_A_APP_ID].descr_uuid,
                                                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret){
            TRACE_ERR("Adding descriptor failed: %s", esp_err_to_name(add_descr_ret));
        }
        break;
    }


    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        telescope_profile[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        if (param->add_char_descr.status != ESP_GATT_OK)
        {
            TRACE_ERR("Descriptor creation failed: status %d", param->add_char_descr.status);
        }
        TRACE_DEB("ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;


    case ESP_GATTS_START_EVT:
        if (param->start.status != ESP_GATT_OK)
        {
            TRACE_ERR("Service start failed: status %d", param->start.status);
        }
        TRACE_DEB("SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;


    case ESP_GATTS_STOP_EVT:
        break;


    case ESP_GATTS_CONNECT_EVT: {
    esp_ble_conn_update_params_t conn_params = {0};
    memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    conn_params.latency = 0;
    conn_params.max_int = 0x20;    // 40 ms
    conn_params.min_int = 0x10;    // 20 ms
    conn_params.timeout = 400;     // 4 s

    TRACE_INF("Connected: conn %d", param->connect.conn_id);

    __atomic_store_n(&s_notify_conn_id, BLE_INVALID_CONN_ID, __ATOMIC_RELEASE);
    __atomic_store_n(&telescope_profile[PROFILE_A_APP_ID].conn_id, param->connect.conn_id, __ATOMIC_RELEASE);

    // remember peer for delayed update
    memcpy(s_last_peer, param->connect.remote_bda, sizeof(esp_bd_addr_t));

    // Schedule one delayed connection-parameter update to avoid racing the phone.
    if (!s_conn_upd_inflight) {
        s_conn_upd_inflight = true;

        if (s_conn_upd_timer == NULL) {
            const esp_timer_create_args_t args = {
                .callback = &conn_update_cb,
                .arg = NULL,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "conn_upd"
            };
            esp_timer_create(&args, &s_conn_upd_timer);
        }
        esp_timer_start_once(s_conn_upd_timer, 1200 * 1000); // 1.2 s
    }

    global_status_bar_screen_bluetooth = true;

    if (STATUS_TRACKING_ON != global_status_message) {
        global_status_message = STATUS_IDLE;
    } else {
        esp_err_t command_err = mobile_app_command_receive((const uint8_t *)APP_RX_NONE, strlen(APP_RX_NONE));
        if (command_err != ESP_OK)
        {
            TRACE_ERR("Tracking stop handoff failed: %s", esp_err_to_name(command_err));
        }
    }
    update_display_screen();

    global_standalone_mode = false;
    global_ble_is_connected = true;
    mobile_app_command_connected();
    global_ble_connected_start_time = esp_timer_get_time();
    break;
}



    case ESP_GATTS_DISCONNECT_EVT:
        __atomic_store_n(&s_notify_conn_id, BLE_INVALID_CONN_ID, __ATOMIC_RELEASE);
        __atomic_store_n(&telescope_profile[PROFILE_A_APP_ID].conn_id, BLE_INVALID_CONN_ID, __ATOMIC_RELEASE);
        mobile_app_command_disconnected();
        global_ble_is_connected = false;
        if (s_conn_upd_timer) esp_timer_stop(s_conn_upd_timer);
            s_conn_upd_inflight = false;
        TRACE_INF("Disconnected: reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);

        global_status_bar_screen_bluetooth = false;
        if(STATUS_TRACKING_ON != global_status_message) global_status_message = STATUS_BT_NOT_CONNECTED;
        update_display_screen();
       
        break;


    case ESP_GATTS_CONF_EVT:
        TRACE_VER("GATT confirmation: status %d, handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            TRACE_WAR("GATT confirmation failed: status %d, handle %d", param->conf.status, param->conf.handle);
        }
        break;


    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // If registering an app, save the interface returned by the stack
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            telescope_profile[param->reg.app_id].gatts_if = gatts_if;
        } else {
            TRACE_ERR("App registration failed: app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    // Dispatch the event to the correct profile(s)
    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == telescope_profile[idx].gatts_if) {
                if (telescope_profile[idx].gatts_cb) {
                    telescope_profile[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void update_global_transmit_status() {
    global_transmit_status = 0; // Clear previous value

    // Bit 0: Tracking active
    if (global_status_is_tracking_active) {
        global_transmit_status |= (1 << APP_STATUS_TRACKING_BIT);
    }

    // Bit 1: Telescope aligned
    if (global_status_bar_screen_telescope) {
        global_transmit_status |= (1 << APP_STATUS_ALIGNED_BIT);
    }

    // Bits 2–4: Date/Time/Location sync status (3 bits)
    global_transmit_status |= ((global_status_bar_screen_loc_date & APP_STATUS_SYNC_MASK) << APP_STATUS_SYNC_SHIFT);
}

void send_status_to_app() {
    update_global_transmit_status();

    uint8_t payload[APP_TX_STATUS_LENGTH];
    payload[0] = APP_TX_STATUS_TAG; // ASCII 'S' = 0x53
    payload[1] = global_transmit_status;

    ble_notify_app(payload, sizeof(payload));

    TRACE_VER("Status notification submitted: 0x%02x", (unsigned)global_transmit_status);
}

static void conn_update_cb(void *arg)
{
    if (!global_ble_is_connected) {  // Skip the update if the link has already closed.
        s_conn_upd_inflight = false;
        return;
    }

    esp_ble_conn_update_params_t p = {0};
    memcpy(p.bda, s_last_peer, sizeof(esp_bd_addr_t));
    p.latency = 0;
    p.min_int = 0x10;   // 20 ms
    p.max_int = 0x20;   // 40 ms
    p.timeout = 400;    // 4 s

    esp_err_t err = esp_ble_gap_update_conn_params(&p);
    if (err != ESP_OK) {
        TRACE_WAR("Connection parameter update request failed: %s", esp_err_to_name(err));
        // allow future retries
        s_conn_upd_inflight = false;
    }
}
