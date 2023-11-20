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

/* This file is gatts functions for all devices with BTSTACK version lower than 3.0, i.e. 20xxx and 43012C0 */

#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"
#include "wiced_memory.h"
#include "app.h"

#if GATT_TRACE
 #define     APP_GATT_TRACE   WICED_BT_TRACE
 #if GATT_TRACE > 1
  #define    APP_GATT_TRACE2   WICED_BT_TRACE
 #else
  #define    APP_GATT_TRACE2(...)
 #endif
#else
 #define     APP_GATT_TRACE(...)
 #define     APP_GATT_TRACE2(...)
#endif

#if GATT_TRACE > 1
#define STR(x) #x
static const char* eventStr[] =
{
    STR(GATT_CONNECTION_STATUS_EVT),        /**< GATT connection status change. Event data: #wiced_bt_gatt_connection_status_t */
    STR(GATT_OPERATION_CPLT_EVT),           /**< GATT client events. Event data: #wiced_bt_gatt_event_data_t */
    STR(GATT_DISCOVERY_RESULT_EVT),         /**< GATT attribute discovery result. Event data: #wiced_bt_gatt_discovery_result_t */
    STR(GATT_DISCOVERY_CPLT_EVT),           /**< GATT attribute discovery complete. Event data: #wiced_bt_gatt_event_data_t */
    STR(GATT_ATTRIBUTE_REQUEST_EVT),        /**< GATT attribute request (from remote client). Event data: #wiced_bt_gatt_attribute_request_t */
    STR(GATT_CONGESTION_EVT),               /**< GATT congestion (running low in tx buffers). Event data: #wiced_bt_gatt_congestion_event_t TODO: add more details regarding congestion */
    STR(GATT_GET_RESPONSE_BUFFER_EVT),      /**< GATT buffer request, typically sized to max of bearer mtu - 1 */
    STR(GATT_APP_BUFFER_TRANSMITTED_EVT),   /**< GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
};

/****************************************************/
const char* getGattEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(uint8_t*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}
#endif // GATT_TRACE

/*
 * This function sends write to the peer GATT server
 */
wiced_bt_gatt_status_t app_gatt_send_write( uint16_t conn_id, uint16_t handle, void *p_data, uint16_t len, wiced_bt_gatt_write_type_t type )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_INSUF_RESOURCE;

    // Allocating a buffer to send the write request
    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( GATT_RESPONSE_SIZE( len ) );

    if ( p_write )
    {
        p_write->handle   = handle;
        p_write->offset   = 0;
        p_write->len      = len;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        memcpy( p_write->value, p_data, len );

        // Register with the server to receive notification
        status = wiced_bt_gatt_send_write ( conn_id, type, p_write );

        APP_GATT_TRACE( "wiced_bt_gatt_send_write status:%d\n", status );

        wiced_bt_free_buffer( p_write );
    }
    return ( status );
}

/*
 * app_gatt_send_read_by_handle -- send a read-by-handle request to server
 */
wiced_bt_gatt_status_t app_gatt_send_read_by_handle(uint16_t conn_id, uint16_t handle)
{
    static wiced_bt_gatt_read_param_t read_req;

    memset( &read_req, 0, sizeof( wiced_bt_gatt_read_param_t ) );
    read_req.by_handle.auth_req = GATT_AUTH_REQ_NONE;
    read_req.by_handle.handle = handle;
    return wiced_bt_gatt_send_read(conn_id, GATT_READ_BY_HANDLE, &read_req);
}

/*
 * GATT operation started by the client has been completed
 */
void app_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch ( p_data->op )
    {
        case GATTC_OPTYPE_READ:
            APP_GATT_TRACE( "read_rsp status:%d\n", p_data->status );
            app_process_read_rsp(p_data);
            break;

        case GATTC_OPTYPE_WRITE:
            APP_GATT_TRACE( "write_rsp status:%d desc_handle:%x \n", p_data->status,p_data->response_data.handle );
            break;

        case GATTC_OPTYPE_CONFIG:
            APP_GATT_TRACE( "peer mtu:%d\n", p_data->response_data.mtu );
            break;

        case GATTC_OPTYPE_NOTIFICATION:
            APP_GATT_TRACE( "notification status:%d\n", p_data->status );
            app_notification_handler( p_data );
            break;

        case GATTC_OPTYPE_INDICATION:
            APP_GATT_TRACE( "indication status:%d\n", p_data->status );
            app_indication_handler( p_data );
            break;
    }
}

wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_INVALID_PDU;

    APP_GATT_TRACE2("*** %s (%d)\n", getGattEventStr(event), event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            app_connection_evt( p_data );
            break;

        case GATT_DISCOVERY_RESULT_EVT:
            result = app_gatt_discovery_result(&p_data->discovery_result);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            result = app_gatt_discovery_complete(&p_data->discovery_complete);
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = app_gatt_operation_complete(&p_data->operation_complete);
            break;

        default:
            break;
    }

    return result;
}
