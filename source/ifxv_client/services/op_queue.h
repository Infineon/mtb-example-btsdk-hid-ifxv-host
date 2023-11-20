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
#ifndef OP_QUEUE_H__
#define OP_QUEUE_H__

#include "wiced_bt_types.h"

typedef enum
{
    IFXV_OP_WRITE,
    IFXV_OP_READ,
    IFXV_OP_SET_CONFIG,
} ifxv_op_e;

void ifxv_op_init();
wiced_bt_gatt_status_t ifxv_op(uint8_t op, uint16_t handle, uint16_t data16, void * p_data, uint8_t len);
#define ifxv_op_set_cfg(handle, data16)     ifxv_op(IFXV_OP_SET_CONFIG, handle, data16, NULL, 0)
#define ifxv_op_write(handle, p_data, len)  ifxv_op(IFXV_OP_WRITE, handle, 0, p_data, (uint8_t) len)
#define ifxv_op_read(handle)                ifxv_op(IFXV_OP_READ, handle, 0, NULL, 0)

#endif // OP_QUEUE_H__
