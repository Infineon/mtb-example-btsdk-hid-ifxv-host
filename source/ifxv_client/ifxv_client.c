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
 *
 * IFXV Client Profile library.
 *
 */

#include "app.h"

#if CLIENT_TRACE == 1
 #define     IFXV_CLIENT_TRACE   WICED_BT_TRACE
 #define     IFXV_CLIENT_TRACE2(...)
#elif CLIENT_TRACE == 2
 #define     IFXV_CLIENT_TRACE   WICED_BT_TRACE
 #define     IFXV_CLIENT_TRACE2  WICED_BT_TRACE
#else
 #define     IFXV_CLIENT_TRACE(...)
 #define     IFXV_CLIENT_TRACE2(...)
#endif

/* Peer Info */
typedef struct
{
    uint16_t    conn_id;                   // Connection Identifier
    uint8_t     role;                      // central or peripheral in the current connection
    uint8_t     addr_type;                 // peer address type
    uint8_t     transport;                 // peer connected transport
    uint8_t     peer_addr[BD_ADDR_LEN];    // Peer BD Address
} peer_info_t;

typedef struct
{
    peer_info_t     peer;
    ifxv_callback_t *p_callback;    /* Application BAC Callback */
    uint8_t         state;
} ifxv_t;

/******************************************************
 *                Variables Definitions
 ******************************************************/

static ifxv_t ifxv = {0};

/******************************************************
 *  Private Functions
 ******************************************************/

/*
 * With given event, call user callback functon .
 * Application passes it here if handle belongs to our service.
 */
static void ifxv_client_callback(ifxv_event_t event, wiced_bt_gatt_operation_complete_t *p_data)
{
    ifxv_event_data_t data_event;

    if (ifxv.p_callback)
    {
        data_event.data.conn_id = p_data->conn_id;
        data_event.data.status = p_data->status;
//        data_event.data.uuid = ifxv.characteristics[i].uuid;
        data_event.data.handle = p_data->response_data.att_value.handle;
        data_event.data.len = p_data->response_data.att_value.len;
        data_event.data.offset = p_data->response_data.att_value.offset;
        data_event.data.p_data = p_data->response_data.att_value.p_data;
        ifxv.p_callback(event, &data_event);
    }
}

/******************************************************
 * Public Functions
 ******************************************************/

/*
 * ifxv_conn_id
 */
uint16_t ifxv_conn_id()
{
    return ifxv.peer.conn_id;
}

/*
 * ifxv_peer_addr
 */
uint8_t * ifxv_peer_addr()
{
    return ifxv.peer.peer_addr;
}

/*
 * ifxv_peer_transport
 */
uint8_t ifxv_peer_transport()
{
    return ifxv.peer.transport;
}

/*
 * ifxv_peer_addr_type
 */
uint8_t ifxv_peer_addr_type()
{
    return ifxv.peer.addr_type;
}

/*
 * ifxv_conn_role
 */
uint8_t ifxv_conn_role()
{
    return ifxv.peer.role;
}

/*
 *  ifxv_client_read_response
 */
void  ifxv_client_read_response(wiced_bt_gatt_operation_complete_t *p_data)
{
    IFXV_CLIENT_TRACE("ifxv_client_read_response\n");
    ifxv_client_callback(IFXV_EVENT_RSP, p_data);
}

/*
 * ifxv_client_notification
 */
void ifxv_client_notification(wiced_bt_gatt_operation_complete_t *p_data)
{
    IFXV_CLIENT_TRACE2("ifxv_client_notification\n");

    // If it is not custom service that handles by the service, we pass to application
    if (!discovery_custom_notification(p_data))
    {
        ifxv_client_callback(IFXV_EVENT_NOTIFICATION, p_data);
    }
}

/*
 * ifxv_client_indication
 */
void ifxv_client_indication(wiced_bt_gatt_operation_complete_t *p_data)
{
    WICED_BT_TRACE( "Send indication confirm id:%d handle:%04x\n", p_data->conn_id,  p_data->response_data.handle);
    wiced_bt_gatt_client_send_indication_confirm( p_data->conn_id, p_data->response_data.handle );
    ifxv_client_callback(IFXV_EVENT_INDICATION, p_data);
}

/*
 * ifxv_client_link_up
 */
void ifxv_client_link_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    uint8_t dev_role;

    wiced_bt_dev_get_role( p_conn_status->bd_addr, &dev_role, BT_TRANSPORT_LE );

    // Save the peer info
    ifxv.peer.conn_id       = p_conn_status->conn_id;
    ifxv.peer.addr_type     = p_conn_status->addr_type;
    ifxv.peer.conn_id       = p_conn_status->conn_id;
    ifxv.peer.role          = dev_role;
    ifxv.peer.transport     = p_conn_status->transport;
    memcpy(ifxv.peer.peer_addr, p_conn_status->bd_addr, BD_ADDR_LEN);

    // Start to discover primary services
    discovery_link_up(p_conn_status->conn_id);
}

/*
 * ifxv_client_link_down
 */
void ifxv_client_link_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    memset(&ifxv.peer, 0, sizeof(peer_info_t));
    discovery_link_down();
}

/*
 * ifxv_client_init
 */
wiced_result_t ifxv_client_init(ifxv_callback_t *p_callback)
{
    IFXV_CLIENT_TRACE("ifxv_init\n");

    discovery_init((ifxv.p_callback = p_callback));
    ifxv_op_init();
    audio_init();

    return WICED_SUCCESS;
}
