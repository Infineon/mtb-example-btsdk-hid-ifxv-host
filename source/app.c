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

#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "hci_control_api.h"
#include "app.h"
#include "cycfg_pins.h"

#if APP_TRACE == 1
 #define     IFXV_APP_TRACE      WICED_BT_TRACE
 #define     IFXV_APP_TRACE2(...)
#elif APP_TRACE == 2
 #define     IFXV_APP_TRACE      WICED_BT_TRACE
 #define     IFXV_APP_TRACE2     WICED_BT_TRACE
#else
 #define     IFXV_APP_TRACE(...)
 #define     IFXV_APP_TRACE2(...)
#endif

/******************************************************************************
 *   Defines
 ******************************************************************************/
#define LED_SCAN_BLINK_SPEED    500
#define TRANS_UART_BUFFER_SIZE  1024

/******************************************************************************
 *   External Definitions
 ******************************************************************************/

/******************************************************************************
 *   Data
 ******************************************************************************/

const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
#if BTSTACK_VER >= 0x03000001
    .heap_config =
    {
        .data_heap_size = 1024 * 4 + 1500 * 2,
        .hci_trace_heap_size = 1024 * 2,
        .debug_trace_heap_size = 1024,
    },
#else
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
#endif

    .p_status_handler = hci_transport_status,
    .p_data_handler = hci_rx_cmd,

    .p_tx_complete_cback = NULL
};

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 * ifxv client callback
 */
static void app_callback(ifxv_event_t event, ifxv_event_data_t *p_data)
{
    switch (event)
    {
    case IFXV_EVENT_DISCOVERY_COMPLETE:
        IFXV_APP_TRACE("IFXV_EVENT_DISCOVERY_COMPLETE\n");
        /* If ifxv Service successfully discovered */
        if (p_data->discovery.status == WICED_BT_GATT_SUCCESS)
        {
//            ifxv_read(p_data->discovery.conn_id);
        }
        break;

    case IFXV_EVENT_RSP:
        IFXV_APP_TRACE("IFXV_EVENT_RSP\n");
        break;

    case IFXV_EVENT_NOTIFICATION:
//        WICED_BT_TRACE("IFXV_EVENT_NOTIFICATION\n");
        break;

    case IFXV_EVENT_INDICATION:
        IFXV_APP_TRACE("IFXV_EVENT_INDICATION\n");
        break;

    default:
        WICED_BT_TRACE("Unknown IFXV Event:%d\n", event);
        break;
    }
}

/*
 * Link Up
 */
static wiced_bt_gatt_status_t app_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE( "Link up, Conn Id:%d Addr: < %B>\n", p_status->conn_id, p_status->bd_addr);

    led_on(LINK_LED);

    // need to notify ifxv that the connection is up
    ifxv_client_link_up( p_status );
    hci_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Link down
 */
static wiced_bt_gatt_status_t app_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("Link down, reason %d (0x%02x)\n", p_status->reason, p_status->reason);

    led_off(LINK_LED);

    ifxv_client_link_down( p_status );
    hci_send_disconnect_evt( p_status->reason, p_status->conn_id );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Load the address resolution database
 */
static void app_load_keys_to_addr_resolution_db()
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;
    uint16_t                    i;

    for ( i = APP_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram(i, sizeof(keys), (uint8_t *)&keys, &result);

        // if failed to read NVRAM, there is nothing saved at that location
        if ((result == WICED_SUCCESS) && (bytes_read == sizeof(wiced_bt_device_link_keys_t)))
        {
            IFXV_APP_TRACE("[%s] Added to address resolution database\n");
            result = wiced_bt_dev_add_device_to_address_resolution_db(&keys);
        }
        else
        {
            break;
        }
    }
}

/*
 * Check for device entry exists in NVRAM list
 */
wiced_bool_t app_is_device_bonded(wiced_bt_device_address_t bd_address)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = APP_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, bd_address, BD_ADDR_LEN ) == 0 )
            {
                IFXV_APP_TRACE2("[%s] return TRUE\n");
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    IFXV_APP_TRACE("[%s] return FALSE\n");
    return WICED_FALSE;
}

/*
 * This function handles the scan results and attempt to connect to IFXV Server.
 */
static void app_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    wiced_result_t          status;
    wiced_bool_t            ret_status;
    uint8_t                 length = 0;
    uint8_t *               p_data;
    uint8_t                 *device_voice = (uint8_t *) "IFX-Voice";
    uint8_t                 *device_remote = (uint8_t *) "IFXV-Remote";
    uint8_t                 *device_rcu = (uint8_t *) "LE RCU";
    uint16_t                service_uuid16=0;

    if ( p_scan_result )
    {
        // Search for Device Name in the Advertisement data received.
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &length );
        if ( p_data != NULL && (!memcmp(p_data, device_voice, strlen((char *) device_voice)) ||
                                !memcmp(p_data, device_remote, strlen((char *) device_remote)) ||
                                !memcmp(p_data, device_rcu, strlen((char *) device_rcu))))
        {
            WICED_BT_TRACE("Found the peer device %s, BD Addr: %B\n", p_data, p_scan_result->remote_bd_addr);

            led_blink_stop(LINK_LED);

            /* Device found. Stop scan. */
            if((ret_status = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, app_scan_result_cback )) != WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("Failed to stop scan, status %d\n", ret_status);
            }

            WICED_BT_TRACE("Initiating connection...\n");
            /* Initiate the connection */
            ret_status = wiced_bt_gatt_le_connect( p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, TRUE );
            IFXV_APP_TRACE( "wiced_bt_gatt_connect status %d\n", ret_status );
        }
    }
    else
    {
        IFXV_APP_TRACE("End of scan\n");
        led_blink_stop(LINK_LED);
        hci_send_status(HCI_CONTROL_IFXV_STATUS_IDLE);
    }
}

/*
 * Save link key to nvram
 */
wiced_bool_t app_save_link_keys( wiced_bt_device_link_keys_t *p_keys )
{
    uint8_t                     bytes_written, bytes_read;
    wiced_bt_device_link_keys_t temp_keys;
    uint16_t                    id = 0;
    uint32_t i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = APP_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        IFXV_APP_TRACE( "Read NVRAM at:%d bytes:%d result:%d\n", i, bytes_read, result );

        // if failed to read NVRAM, there is nothing saved at that location
        if ( ( result != WICED_SUCCESS ) || ( bytes_read != sizeof( temp_keys ) ) )
        {
            id = i;
            break;
        }
        else
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved, reuse the ID
                id = i;
                break;
            }
        }
    }
    if ( id == 0 )
    {
        // all NVRAM locations are already occupied.  Cann't save anything.
        WICED_BT_TRACE( "Failed to save NVRAM\n" );
        return WICED_FALSE;
    }
    IFXV_APP_TRACE( "writing to id:%d\n", id );
    bytes_written = wiced_hal_write_nvram( id, sizeof( wiced_bt_device_link_keys_t ), (uint8_t *)p_keys, &result );
    WICED_BT_TRACE( "Saved %d bytes at id:%d %d\n", bytes_written, id );
    return WICED_TRUE;
}

/*
 * Read link key from nvram
 */
wiced_bool_t app_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    wiced_bt_device_link_keys_t temp_keys;
    uint8_t                     bytes_read;
    uint16_t                    i;
    wiced_result_t              result;

    // search through all available NVRAM IDs.
    for ( i = APP_PAIRED_KEYS_START_VS_ID; i < WICED_NVRAM_VSID_END; i++ )
    {
        bytes_read = wiced_hal_read_nvram( i, sizeof( temp_keys ), (uint8_t *)&temp_keys, &result );

        IFXV_APP_TRACE("[%s] read status %d bytes read %d \n", __FUNCTION__, result, bytes_read);
        UNUSED_VARIABLE(bytes_read);

        // if failed to read NVRAM, there is nothing saved at that location
        if ( result == WICED_SUCCESS )
        {
            if ( memcmp( temp_keys.bd_addr, p_keys->bd_addr, BD_ADDR_LEN ) == 0 )
            {
                // keys for this device have been saved
                memcpy( &p_keys->key_data, &temp_keys.key_data, sizeof( temp_keys.key_data ) );
                return WICED_TRUE;
            }
        }
        else
        {
            break;
        }
    }
    return WICED_FALSE;
}

/*
 * Find IFXV server and pair
 */
void app_enter_pairing()
{
    wiced_result_t result;

    WICED_BT_TRACE("Searching for IFX-Voice device...\n" );
    led_blink(LINK_LED, 0, LED_SCAN_BLINK_SPEED);
    /*start scan if not connected and no scan in progress*/
    if (!ifxv_is_connected() && (wiced_bt_ble_get_current_scan_state() == BTM_BLE_SCAN_TYPE_NONE))
    {
        result = wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, app_scan_result_cback );
        hci_send_status(result == WICED_BT_PENDING ? HCI_CONTROL_IFXV_STATUS_SCANNING : HCI_CONTROL_IFXV_STATUS_IDLE );
        IFXV_APP_TRACE("wiced_bt_ble_scan: %d \n", result);
    }
    else
    {
         WICED_BT_TRACE("Scanning not initiated, scan in progress ??  \n");
    }
}

/*
 * link status change event
 */
void app_connection_evt( wiced_bt_gatt_event_data_t * p_data )
{
    if (p_data->connection_status.connected)
    {
        app_connection_up(&p_data->connection_status);
    }
    else
    {
        app_connection_down(&p_data->connection_status);
    }
}

/*
 * Pass read response to appropriate client based on the attribute handle
 */
void app_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data)
{
    IFXV_APP_TRACE("read response\n");
    ifxv_client_read_response(p_data);
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void app_indication_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    IFXV_APP_TRACE("indication handle:%04x\n", p_data->response_data.att_value.handle);

    ifxv_client_indication(p_data);
}

/*
 * Pass notification to appropriate client based on the attribute handle
 */
void app_notification_handler(wiced_bt_gatt_operation_complete_t *p_data)
{
    IFXV_APP_TRACE("notification handle:%04x\n", p_data->response_data.att_value.handle);

    ifxv_client_notification(p_data);
}

/*
 * Application init
 */
static void app_init()
{
    wiced_bt_gatt_status_t gatt_status;
    UNUSED_VARIABLE(gatt_status);

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_gatt_callback);

    IFXV_APP_TRACE("wiced_bt_gatt_register: %d\n", gatt_status);

#if defined(ENABLE_HCI_TRACE)
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hci_trace_cback );
#endif

    /* Load the address resolution DB with the keys stored in the NVRAM */
    app_load_keys_to_addr_resolution_db();

    button_init();
    ifxv_client_init(app_callback);

    WICED_BT_TRACE("Free RAM bytes=%d bytes\n", wiced_memory_get_free_bytes());

    app_enter_pairing();
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that Bluetooth device is ready.
 */
APPLICATION_START( )
{
    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE
 #ifdef ENABLE_HCI_TRACE
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_trbasort_data_handler_t callback present
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
 #else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
 #endif
#endif

    led_init(led_count, platform_led);

    WICED_BT_TRACE( "\n<< IFX Voice Host v0.2 Start >>\n" );

    // Register call back and configuration with stack
    bt_init(app_init);
}
