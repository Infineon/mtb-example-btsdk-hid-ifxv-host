/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*/

#include "wiced_bt_trace.h"
#include "app.h"

#if BT_TRACE >= 1
 #define     IFXV_BT_TRACE      WICED_BT_TRACE
#else
 #define     IFXV_BT_TRACE(...)
#endif

typedef struct
{
    app_init_t app_init_cb;
    wiced_bt_device_address_t  loc_bda;
    wiced_bt_ble_address_type_t loc_bda_type;
} bt_t;

static bt_t bt = {0};

#if BT_TRACE

#define STR(x) #x
static const char* eventStr[] =
{
    STR(BTM_ENABLED_EVT),
    STR(BTM_DISABLED_EVT),
    STR(BTM_POWER_MANAGEMENT_STATUS_EVT),
 #ifdef WICED_X
    STR(BTM_RE_START_EVT),
 #endif
    STR(BTM_PIN_REQUEST_EVT),
    STR(BTM_USER_CONFIRMATION_REQUEST_EVT),
    STR(BTM_PASSKEY_NOTIFICATION_EVT),
    STR(BTM_PASSKEY_REQUEST_EVT),
    STR(BTM_KEYPRESS_NOTIFICATION_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT),
    STR(BTM_PAIRING_COMPLETE_EVT),
    STR(BTM_ENCRYPTION_STATUS_EVT),
    STR(BTM_SECURITY_REQUEST_EVT),
    STR(BTM_SECURITY_FAILED_EVT),
    STR(BTM_SECURITY_ABORTED_EVT),
    STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT),
    STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT),
    STR(BTM_BLE_SCAN_STATE_CHANGED_EVT),
    STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT),
    STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT),
    STR(BTM_SCO_CONNECTED_EVT),
    STR(BTM_SCO_DISCONNECTED_EVT),
    STR(BTM_SCO_CONNECTION_REQUEST_EVT),
    STR(BTM_SCO_CONNECTION_CHANGE_EVT),
    STR(BTM_BLE_CONNECTION_PARAM_UPDATE),
    STR(BTM_BLE_PHY_UPDATE_EVT),
    STR(BTM_LPM_STATE_LOW_POWER),                        /**< Bluetooth device wake has been deasserted */
    STR(BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT),        /**< LE remote connection parameter request. Reply using wiced_bt_l2cap_reply_ble_remote_conn_params_req and return WICED_BT_CMD_STORED to denote this event was handled. Event data: wiced_bt_ble_rc_connection_param_req_t */
};

/****************************************************/
const char* getStackEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(uint8_t*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}
#endif // BT_TRACE

/*
 *
 */
static wiced_result_t bt_management_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                    result = WICED_BT_SUCCESS;
    wiced_bt_dev_encryption_status_t *p_status;
    wiced_bt_dev_ble_pairing_info_t  *p_info;
    wiced_bt_ble_advert_mode_t       *p_mode;
    wiced_bt_device_link_keys_t       paired_device_link_keys_request;
    uint8_t                           bytes_written, bytes_read;
    uint8_t                          *p_keys;

    IFXV_BT_TRACE("=== %s (%d)\n", getStackEventStr(event), event);    // hidd default handler

    switch(event)
    {
    /* Bluetooth stack enabled */
    case BTM_ENABLED_EVT:
        if (wiced_bt_cfg_settings.rpa_refresh_timeout == WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE)
        {
            wiced_bt_dev_read_local_addr(bt.loc_bda);
            WICED_BT_TRACE("bda:%B\n", bt.loc_bda);
            bt.loc_bda_type = BLE_ADDR_PUBLIC;
        }
        else
        {
            memcpy(bt.loc_bda, wiced_btm_get_private_bda(), BD_ADDR_LEN);
            WICED_BT_TRACE("rpa:%B\n", bt.loc_bda);
            bt.loc_bda_type = BLE_ADDR_RANDOM;
        }

        if (bt.app_init_cb)
        {
            bt.app_init_cb();
        }
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC | BTM_LE_KEY_PID | BTM_LE_KEY_PCSRK | BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if(p_event_data->pairing_complete.transport == BT_TRANSPORT_LE)
        {
            IFXV_BT_TRACE("LE pairing complete %d\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
            hci_send_pairing_complete_evt(p_event_data->pairing_complete.pairing_complete_info.ble.reason, p_event_data->pairing_complete.pairing_complete_info.ble.resolved_bd_addr, BT_DEVICE_TYPE_BLE );
        }
        else
        {
            IFXV_BT_TRACE("Transport %d BTM_PAIRING_COMPLETE_EVT: %d -- ignored\n", p_event_data->pairing_complete.transport, p_event_data->pairing_complete.pairing_complete_info.ble.reason);
        }
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        IFXV_BT_TRACE("encryption: bd (%B) ressult %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;

    case BTM_SECURITY_REQUEST_EVT:
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        app_save_link_keys(&p_event_data->paired_device_link_keys_update);
        if (p_event_data->paired_device_link_keys_update.key_data.ble_addr_type != BLE_ADDR_PUBLIC)
        {
            wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update);
        }
         wiced_bt_ble_update_background_connection_device(TRUE, p_event_data->paired_device_link_keys_update.bd_addr);
         wiced_bt_ble_set_background_connection_type(BTM_BLE_CONN_AUTO, NULL);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (app_read_link_keys(&p_event_data->paired_device_link_keys_request))
        {
            IFXV_BT_TRACE("Key retrieval success\n");
        }
        else
        {
            result = WICED_BT_ERROR;
            IFXV_BT_TRACE("Link key request: retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* save keys to NVRAM */
        p_keys = (uint8_t*)&p_event_data->local_identity_keys_update;
        wiced_bt_ble_set_local_identity_key_data(p_keys);
        wiced_hal_write_nvram ( APP_LOCAL_KEYS_VS_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
        IFXV_BT_TRACE("Save local keys to NVRAM, result: %d \n", result);
        break;


    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /* read keys from NVRAM */
        p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
        wiced_hal_read_nvram( APP_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
        if (result==WICED_SUCCESS)
        {
            IFXV_BT_TRACE("local keys found in NVRAM\n");
            wiced_bt_ble_set_local_identity_key_data(p_keys);
        }
        else
        {
            IFXV_BT_TRACE("no local keys in NVRAM\n");
        }
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        IFXV_BT_TRACE( "Scan state changed to %d\n", p_event_data->ble_scan_state_changed );
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        IFXV_BT_TRACE( "Link parameter changed to: status:%d, %B: Interval:%d Latency:%d TO:%d\n", p_event_data->ble_connection_param_update.status,
                                                                                                   p_event_data->ble_connection_param_update.bd_addr,
                                                                                                   p_event_data->ble_connection_param_update.conn_interval,
                                                                                                   p_event_data->ble_connection_param_update.conn_latency,
                                                                                                   p_event_data->ble_connection_param_update.supervision_timeout);
        break;

    case BTM_BLE_REMOTE_CONNECTION_PARAM_REQ_EVT:
        IFXV_BT_TRACE( "Link parameter req: %B: Min:%d Max:%d Latency:%d TO:%d\n", p_event_data->ble_rc_connection_param_req.bd_addr,
                                                                                    p_event_data->ble_rc_connection_param_req.min_int,
                                                                                    p_event_data->ble_rc_connection_param_req.max_int,
                                                                                    p_event_data->ble_rc_connection_param_req.latency,
                                                                                    p_event_data->ble_rc_connection_param_req.timeout);
        break;

    default:
        IFXV_BT_TRACE( "*** Unhandled BTM event: %d\n", event );
        break;
    }
    return result;
}

wiced_bt_device_address_t * bt_loc_addr()
{
    return &bt.loc_bda;
}

wiced_bt_ble_address_type_t bt_loc_addr_type()
{
    return bt.loc_bda_type;
}

/*
 *
 */
wiced_result_t bt_init(app_init_t cb)
{
    bt.app_init_cb = cb;
    return bt_stack_init(bt_management_cback);
}
