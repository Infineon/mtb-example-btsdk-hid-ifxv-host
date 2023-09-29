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
 * Battery Status Client Profile library.
 * This file is for backward compatiblity
 *
 */
#pragma once

#include "discovery.h"

#define ifxv_is_connected() ifxv_conn_id()

/******************************************************
 *                  typedef
 ******************************************************/

/*
 * Events received by the applicaton's ifxv callback
 */
typedef enum
{
    IFXV_EVENT_DISCOVERY_COMPLETE = 1,      /**< GATT Discovery Complete */
    IFXV_EVENT_RSP,                         /**< Read Response */
    IFXV_EVENT_NOTIFICATION,                /**< Notification received */
    IFXV_EVENT_INDICATION,                  /**< Notification received */
} ifxv_event_t;

/**
 * \brief Data associated with \ref WICED_BT_BAC_EVENT_DISCOVERY_COMPLETE.
 *
 * This event is received when a GATT Discovery Operation is complete
 *
 */
typedef struct
{
    uint16_t conn_id;                                   /**< Connection Id */
    wiced_bt_gatt_status_t status;                      /**< Discovery Status */
} discovery_complete_t;

/**
 * \brief event Data
 *
 * The data of read response, notification, or indication.
 *
 */
typedef struct
{
    uint16_t conn_id;
    wiced_bt_gatt_status_t status;
    uint16_t uuid;       /**< uuid */
    uint16_t handle;     /**< handle */
    uint16_t len;        /**< length of response data */
    uint16_t offset;     /**< offset */
    uint8_t  *p_data;    /**< attribute data */
} ifxv_data_t;

/**
 * \brief Union of data associated with BAC events
 *
 */
typedef union
{
    discovery_complete_t           discovery;
    ifxv_data_t                         data;
} ifxv_event_data_t;


typedef void (ifxv_callback_t)(ifxv_event_t event, ifxv_event_data_t * p_data);

/******************************************************
 *                  Function prototyping
 ******************************************************/

uint16_t ifxv_conn_id();
uint8_t * ifxv_peer_addr();
uint8_t ifxv_peer_transport();
uint8_t ifxv_peer_addr_type();
uint8_t ifxv_conn_role();

wiced_result_t ifxv_client_init(ifxv_callback_t * p_callback);
void ifxv_client_link_up(wiced_bt_gatt_connection_status_t * p_conn_status);
void ifxv_client_link_down(wiced_bt_gatt_connection_status_t * p_conn_status);
void ifxv_client_read_response(wiced_bt_gatt_operation_complete_t * p_data);
void ifxv_client_notification(wiced_bt_gatt_operation_complete_t * p_data);
void ifxv_client_indication(wiced_bt_gatt_operation_complete_t * p_data);
