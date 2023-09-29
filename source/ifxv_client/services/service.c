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
 * IFXV operation.
 *
 */

#include "app.h"
#include "wiced_bt_trace.h"

#if SERVICE_TRACE==1
 #define IFXV_SERVICE_TRACE      WICED_BT_TRACE
#else
 #define IFXV_SERVICE_TRACE(...)
#endif

/*************************************
 * Functions
 ************************************/

wiced_bool_t service_found(service_t * service)
{
    return service->id.s_handle && service->id.e_handle;
}

void service_down(service_t * service)
{
    service->id.s_handle = service->id.e_handle = 0;
}

wiced_bool_t service_char_found(service_t * service, uint8_t index)
{
    if (index<service->char_cnt)
    {
        return discovery_characteristic(service->char_ptr[index].index) != NULL;
    }
    return FALSE;
}

wiced_bool_t service_all_char_found(service_t * service)
{
    for (int i=0; i<service->char_cnt; i++)
    {
        if (!service_char_found(service, i))
        {
            return FALSE;
        }
    }
    return TRUE;
}

wiced_bool_t service_cback(service_t * service, wiced_bt_gatt_data_t * gatt_data_p)
{
    // Go through the service's custom charactristics list
    for (int i=0; i<service->char_cnt; i++)
    {
        charstc_t * char_p = discovery_characteristic(service->char_ptr[i].index);

        // check if this charactristic exists and the handle matches, then, call the callback function.
        if ((char_p != NULL) && (char_p->chr.val_handle == gatt_data_p->handle) && service->char_ptr[i].cb)
        {
            return service->char_ptr[i].cb(gatt_data_p);
        }
    }
    return FALSE;
}
