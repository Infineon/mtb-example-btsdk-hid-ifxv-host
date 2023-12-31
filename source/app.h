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
 */
#ifndef APP_H__
#define APP_H__

#include "wiced_bt_types.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "hci_control_api.h"
#include "button.h"
#include "led.h"
#include "hci.h"
#include "ifxv_client.h"
#include "bt.h"
#include "findme.h"
#include "battery.h"
#include "hid.h"
#include "op_queue.h"

/******************************************************************************
 *  defines
 ******************************************************************************/

/* App Timer Timeout in seconds  */
#ifndef WICED_BUTTON_PRESSED_VALUE
#define WICED_BUTTON_PRESSED_VALUE 1
#endif

#define BUTTON_PRESSED              WICED_BUTTON_PRESSED_VALUE

#define LINK_LED        0
#define RED_LED         1

/******************************************************************************
 *                                Structures
 ******************************************************************************/
/******************************************************************************
 * extern data
 ******************************************************************************/
extern const wiced_transport_cfg_t          transport_cfg;
extern const wiced_bt_cfg_settings_t        wiced_bt_cfg_settings;

/******************************************************************************
 *                          Function Protoyping
 ******************************************************************************/
void app_handle_op_complete(wiced_bt_gatt_operation_complete_t *p_data);
void app_connection_evt( wiced_bt_gatt_event_data_t * p_data );
void app_process_read_rsp(wiced_bt_gatt_operation_complete_t *p_data);
void app_indication_handler(wiced_bt_gatt_operation_complete_t *p_data);
void app_notification_handler(wiced_bt_gatt_operation_complete_t *p_data);
void app_enter_pairing();
void app_erase_bonding();
wiced_bool_t app_save_link_keys( wiced_bt_device_link_keys_t *p_keys );
wiced_bool_t app_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
wiced_bool_t app_is_device_bonded(wiced_bt_device_address_t bd_address);

#endif // APP_H__
