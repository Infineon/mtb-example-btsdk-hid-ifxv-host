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

#include "app.h"
#include "wiced_bt_types.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_trace.h"

#if TRACE_GATT == 1
 #define     APP_GATT_TRACE   WICED_BT_TRACE
#else
 #define     APP_GATT_TRACE(...)
#endif

wiced_bt_gatt_status_t app_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    APP_GATT_TRACE("[%s] conn %d, discovery type %d\n", __FUNCTION__, p_data->conn_id, p_data->discovery_type);
    return discovery_result(p_data);
}

/*
 * GATT discovery has been completed
 */
wiced_bt_gatt_status_t app_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_data)
{
    APP_GATT_TRACE("[%s] conn %d type %d\n", __FUNCTION__, p_data->conn_id, p_data->disc_type);
    return discovery_complete(p_data);
}

/*
 * GATT operation started by the client has been completed
 */
wiced_bt_gatt_status_t app_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    wiced_result_t                  status;
    wiced_bt_ble_sec_action_type_t  encryption_type = BTM_BLE_SEC_ENCRYPT;

    APP_GATT_TRACE("ifxv_client_gatt_operation_complete conn %d op %d st %d\n", p_data->conn_id, p_data->op, p_data->status );

    app_handle_op_complete(p_data);

    /* server puts authentication requirement. Encrypt the link */
    if ( p_data->status == WICED_BT_GATT_INSUF_AUTHENTICATION )
    {
        APP_GATT_TRACE("Insufficient Authentication\n");
        if ( app_is_device_bonded(ifxv_peer_addr()) )
        {
            APP_GATT_TRACE("Authentified. Start Encryption\n");
            status = wiced_bt_dev_set_encryption( ifxv_peer_addr(), ifxv_peer_transport(), &encryption_type );
            APP_GATT_TRACE( "wiced_bt_dev_set_encryption %d \n", status );
        }
        else
        {
            APP_GATT_TRACE("Start Authentification/Pairing\n");
            status = wiced_bt_dev_sec_bond( ifxv_peer_addr(), ifxv_peer_addr_type(), ifxv_peer_transport(), 0, NULL );
            APP_GATT_TRACE( "wiced_bt_dev_sec_bond %d \n", status );
        }
    }
    UNUSED_VARIABLE(status);

    return WICED_BT_GATT_SUCCESS;
}
