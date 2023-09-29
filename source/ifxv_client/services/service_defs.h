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
#pragma once

#include "wiced_bt_gatt.h"

typedef wiced_bool_t (*char_cback_t)(wiced_bt_gatt_data_t * gatt_data_p);

typedef struct
{
    wiced_bt_uuid_t  uuid;
    char *           name;
    uint8_t          index;
    char_cback_t     cb;
} char_t;

typedef struct
{
    wiced_bt_gatt_group_value_t id;
    char *           name;
    uint8_t          char_cnt;
    char_t *         char_ptr;
} service_t;

#pragma pack(1)
typedef struct
{
    struct {
        uint8_t     major;
        uint8_t     minor;
    } version;
    struct {
        uint8_t     codec;
        uint8_t     data_unit;
        uint8_t     sampling_rate;
        uint8_t     resp_time_limit_in_sec;
    } cap;
} server_info_t;
#pragma pack()

#define UNDEFINED_CHAR_INDEX 0xff
