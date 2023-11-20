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

#include "app.h"
#include "gatt_utils_lib.h"

#if DISCOVERY_TRACE == 1
 #define     IFXV_DISC_TRACE      WICED_BT_TRACE
 #define     IFXV_DISC_TRACE2(...)
#elif DISCOVERY_TRACE == 2
 #define     IFXV_DISC_TRACE      WICED_BT_TRACE
 #define     IFXV_DISC_TRACE2     WICED_BT_TRACE
#else
 #define     IFXV_DISC_TRACE(...)
 #define     IFXV_DISC_TRACE2(...)
#endif

#ifdef INCLUDE_ATV
 #define NUMBER_OF_SUPPORTED_SERVER 5
#else
 #define NUMBER_OF_SUPPORTED_SERVER 4
#endif
#define MAX_SUPPORTED_CHAR 32
#define FIRST_HANDLE 1
#define LAST_HANDLE  0xffff
#define found_service(s) (s->s_handle && s->e_handle)
#define handle_in_service_range(s, h) ((h >= s->s_handle) && (h <= s->e_handle))
#define characteristic_in_service_range(s, i) ((i<server.char_cnt) && handle_in_service_range(s, server.characteristics[i].chr.handle))
#define LED_DISCOVERY_BLINK_SPEED 100

typedef struct
{
    service_t *         services[NUMBER_OF_SUPPORTED_SERVER];
    charstc_t           characteristics[MAX_SUPPORTED_CHAR];
    uint16_t            conn_id;
    uint16_t            server_characterstic_handle;
    uint8_t             index;
    uint8_t             char_cnt;
    uint8_t             char_index;
    ifxv_callback_t *   p_callback;
} discovery_t;

/******************************************************
 *                Variables
 ******************************************************/
#ifdef INCLUDE_ATV
static discovery_t server = {{&audio_ifxv, &audio_atv, &hid, &battery, &findme}};
#else
static discovery_t server = {{&audio_ifxv, &hid, &battery, &findme}};
#endif
/******************************************************
 *                Functions
 ******************************************************/

/*
 * Set or unset the flag for the given charactristics.
 */
static void ifxv_set_config_descriptor(uint8_t i,  wiced_bool_t enable )
{
    if ((server.characteristics[i].chr.characteristic_properties & (LEGATTDB_CHAR_PROP_INDICATE|LEGATTDB_CHAR_PROP_NOTIFY)) && server.characteristics[i].cccd_handle)
    {
        uint16_t type = GATT_CLIENT_CONFIG_NONE;

        if (enable)
        {
            type = server.characteristics[i].chr.characteristic_properties & LEGATTDB_CHAR_PROP_INDICATE ? GATT_CLIENT_CONFIG_INDICATION : GATT_CLIENT_CONFIG_NOTIFICATION;
        }
        WICED_BT_TRACE( type == GATT_CLIENT_CONFIG_INDICATION ? "Eanbling indication for handle 0x%04x\n" :
                         type == GATT_CLIENT_CONFIG_NOTIFICATION ? "Enabling notification for handle 0x%04x\n" : "Disabling for handle 0x%04x\n", server.characteristics[i].cccd_handle);

        wiced_bt_gatt_status_t status = ifxv_op_set_cfg( server.characteristics[i].cccd_handle, type );
    }
}

/*
 * ifxv_set_all_config_descriptors
 *   This function go though the discovered charactristics table and to enable or disable the descriptor notification/indication flags.
 */
static void ifxv_set_all_config_descriptors(wiced_bool_t enable )
{
    for (int i=0; i<server.char_cnt; i++)
    {
        ifxv_set_config_descriptor(i, enable);
    }
}

/*
 *  discovery_handle_service_result
 *   This function is called when a service is found.
 */
static wiced_bt_gatt_status_t discovery_handle_service_result(wiced_bt_gatt_group_value_t * grp)
{
    WICED_BT_TRACE("Found %s service, handle: 0x%04X-0x%04X\n", server.services[server.index]->name, grp->s_handle, grp->e_handle);

    server.services[server.index]->id.s_handle = grp->s_handle;
    server.services[server.index]->id.e_handle = grp->e_handle;
    return WICED_BT_GATT_SUCCESS;
}

/*
 * return TRUE if this is a custom characteristic
 */
wiced_bool_t discovery_custom_characteristics(uint8_t char_index, wiced_bt_gatt_char_declaration_t *p_char)
{
    service_t * service = server.services[server.index];
    char_t * char_ptr = service->char_ptr;

    for (int i=0; i< service->char_cnt; i++, char_ptr++)
    {
        // Only check for not yet found charactristic
        if (char_ptr->index == UNDEFINED_CHAR_INDEX)
        {
            if (((p_char->char_uuid.len == LEN_UUID_128) && (memcmp(char_ptr->uuid.uu.uuid128, p_char->char_uuid.uu.uuid128, LEN_UUID_128) == 0)) ||
                ((p_char->char_uuid.len == LEN_UUID_16) && (char_ptr->uuid.uu.uuid16 == p_char->char_uuid.uu.uuid16)) ||
                ((p_char->char_uuid.len == LEN_UUID_32) && (char_ptr->uuid.uu.uuid32 == p_char->char_uuid.uu.uuid32)))
            {
                IFXV_DISC_TRACE("%s characterictics\n", char_ptr->name);
                char_ptr->index = char_index;
                return TRUE;
            }
        }
    }
    return FALSE;
}

/*
 * discovery_handle_characteristics_result
 *   This function is called when a charactristristics is found.
 */
static wiced_bt_gatt_status_t discovery_handle_characteristics_result(wiced_bt_gatt_char_declaration_t *p_char)
{
    uint8_t index = server.char_cnt;

    if (server.char_cnt<MAX_SUPPORTED_CHAR)
    {
        IFXV_DISC_TRACE("  *** Characteristics idx:%d, handle:%04x, Val:%04x, UUID_len=%d, ", server.char_cnt, p_char->handle, p_char->val_handle, p_char->char_uuid.len);
        memcpy(&server.characteristics[server.char_cnt++].chr, p_char, sizeof(wiced_bt_gatt_char_declaration_t));
    }
    else
    {
        IFXV_DISC_TRACE("Error! No more space to save characteristics result\n");
        return WICED_BT_GATT_ERROR;
    }

    if (discovery_custom_characteristics(index, p_char))
    {
//        IFXV_DISC_TRACE("Found custom characteristic\n");
    }
    else if (p_char->char_uuid.len == LEN_UUID_16)
    {
        switch (p_char->char_uuid.uu.uuid16) {
        case UUID_CHARACTERISTIC_HID_CONTROL_POINT:
            IFXV_DISC_TRACE("HID control point\n");
            break;

        case GATT_UUID_HID_INFORMATION:
            IFXV_DISC_TRACE("HID Info\n");
            break;

        case UUID_CHARACTERISTIC_BATTERY_LEVEL:
            IFXV_DISC_TRACE("Battery Level\n");
            break;

        case UUID_CHARACTERISTIC_HID_REPORT:
            IFXV_DISC_TRACE("HID Report\n");
//                    app_add_hid_report(p_data);
            break;
        case UUID_CHARACTERISTIC_REPORT_MAP:
            IFXV_DISC_TRACE("HID Report Map\n");
//                    app_add_hid_report_map(p_data);
            break;
        default:
            IFXV_DISC_TRACE("uuid16:%04X\n", p_char->char_uuid.uu.uuid16);
            break;
        }
    }
    else if (p_char->char_uuid.len == LEN_UUID_32)
    {
        IFXV_DISC_TRACE("uuid32:%08x\n", p_char->char_uuid.uu.uuid32);
    }
    else if (p_char->char_uuid.len == LEN_UUID_128)
    {
        IFXV_DISC_TRACE("uuid128:%A\n", p_char->char_uuid.uu.uuid128, LEN_UUID_128);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * discovery_found_descriptor
 *   To print out the found descriptor information.
 */
static void discovery_found_descriptor(uint16_t handle, char * str)
{
    IFXV_DISC_TRACE("    Found handle %04x, %s, for index %d (handle:%04x)\n", handle, str, server.char_index, server.characteristics[server.char_index].chr.handle);
}

/*
 * discovery_handle_descriptor_result
 *   This is the callback function for when the descriptor is found.
 */
static wiced_bt_gatt_status_t discovery_handle_descriptor_result(wiced_bt_gatt_char_descr_info_t *p_info)
{
    if (p_info->type.len == LEN_UUID_16)
    {
        switch (p_info->type.uu.uuid16) {
        case UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
            server.characteristics[server.char_index].cccd_handle = p_info->handle;
            discovery_found_descriptor(p_info->handle, "CCCD");
            break;
        case UUID_DESCRIPTOR_EXTERNAL_REPORT_REFERENCE:
            server.characteristics[server.char_index].rpt_ref_handle = p_info->handle;
            discovery_found_descriptor(p_info->handle, "External Rpt Reference");
            break;
        case UUID_DESCRIPTOR_REPORT_REFERENCE:
            server.characteristics[server.char_index].rpt_ref_handle = p_info->handle;
            discovery_found_descriptor(p_info->handle, "Rpt Reference");
            break;
        case UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
            server.server_characterstic_handle = p_info->handle;
            discovery_found_descriptor(p_info->handle, "Server Characteristic Descriptor");
            break;
        default:
            IFXV_DISC_TRACE("    Found handle %04x, uuid16:%04X, for index %d (handle:%04x)\n", p_info->handle, p_info->type.uu.uuid16, server.char_index, server.characteristics[server.char_index].chr.handle);
            break;
        }
    }
    else if (p_info->type.len == LEN_UUID_32)
    {
        IFXV_DISC_TRACE("    Found handle %04x, uuid32:%08X, for index %d (handle:%04x)\n", p_info->handle, p_info->type.uu.uuid32, server.char_index, server.characteristics[server.char_index].chr.handle);
    }
    else if (p_info->type.len == LEN_UUID_128)
    {
        IFXV_DISC_TRACE("    Found handle:%04x, uuid128:%A, for index %d (handle:%04x)\n", p_info->handle, p_info->type.uu.uuid128, LEN_UUID_128, server.char_index, server.characteristics[server.char_index].chr.handle);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * return TRUE when discovery is done
 */
static wiced_bool_t discovery_characteristics_detail(wiced_bt_gatt_group_value_t * service, wiced_bt_gatt_discovery_type_t type)
{
    IFXV_DISC_TRACE2("**** Discover detail, type=%d, handle: 0x%04X-0x%04X\n", type, service->s_handle, service->e_handle);
    switch (type) {
    case GATT_DISCOVER_SERVICES_BY_UUID:
        IFXV_DISC_TRACE2("GATT_DISCOVER_SERVICES_BY_UUID\n");
        if (service->s_handle && service->e_handle)
        {
            IFXV_DISC_TRACE("**** Discover characteristics handle: 0x%04X-0x%04X\n", service->s_handle, service->e_handle);
            wiced_bt_util_send_gatt_discover(server.conn_id, GATT_DISCOVER_CHARACTERISTICS, 0, service->s_handle, service->e_handle);
            return FALSE;
        }
        return TRUE;

    case GATT_DISCOVER_CHARACTERISTICS:
        IFXV_DISC_TRACE2("GATT_DISCOVER_CHARACTERISTICS\n");
        {
            IFXV_DISC_TRACE("Discover characteristics complete\n");
            // get ready to find descriptors, first, we find out the first characteristic for this service.
            int i;
            for (i=0; i<server.char_cnt; i++)
            {
                // if characteristics[i] belongs to this service
                if (characteristic_in_service_range(service, i))
                {
                    IFXV_DISC_TRACE2("Start to find descriptor from index %d, range:0x%04X-0x%04X\n", i, service->s_handle, service->e_handle);
                    server.char_index = i;
                    break;
                }
            }
            // if we don't have any charactericstis for this service, no need to find descriptor
            if (i >= server.char_cnt)
            {
                return TRUE;
            }
        }
        // after fall through, the index will be incremented by 1, declement it first so it won't be changed.
        server.char_index--;
        // fallthrough
        IFXV_DISC_TRACE2("Fall Through!!\n");

    case GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
        IFXV_DISC_TRACE2("GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS\n");
        while (TRUE)
        {
            // find descriptor for the next characteristics
            server.char_index++;

            // server.cur_index should be the characteristic to find descriptor
            // check if the current index is valid
            if (server.char_index >= server.char_cnt)
            {
                IFXV_DISC_TRACE2("No more characteristic\n", server.char_index);
                return TRUE;
            }

            uint16_t s_handle = server.characteristics[server.char_index].chr.handle;

            // if the handle is out of range, we stop search for descriptor
            if (!handle_in_service_range(service, s_handle))
            {
                IFXV_DISC_TRACE2("The current characteristic %d is not in range\n", server.char_index);
                return TRUE;
            }

            // The characteristic must have value to look for cccd
            if (!server.characteristics[server.char_index].chr.val_handle)
            {
                IFXV_DISC_TRACE2("No descriptor for characteristic %d, check next\n", server.char_index);
                continue;
            }

            uint16_t e_handle = service->e_handle;
            s_handle = server.characteristics[server.char_index].chr.val_handle + 1; // search from val_handle + 1

            IFXV_DISC_TRACE2("  The current characteristic %d, val_handle:0x%04x, range:0x%04x-0x%04x\n", server.char_index, server.characteristics[server.char_index].chr.val_handle, s_handle, e_handle);

            // if this is not the last characteristic in database, we only need to search until the next characteristic.
            // Do we have the next characteristic?
            if ((server.char_index + 1) < server.char_cnt)
            {
                IFXV_DISC_TRACE2("  Has next characteristic, handle:0x%04x, val_handle:0x%04x\n", server.characteristics[server.char_index+1].chr.handle,server.characteristics[server.char_index+1].chr.val_handle);

                // if the next characteristic belongs to this service, we set the ending handle before the next characteristics handle
                if (server.characteristics[server.char_index+1].chr.handle < e_handle)
                {
                    IFXV_DISC_TRACE2("  next characteristic belongs to this service\n");
                    e_handle = server.characteristics[server.char_index+1].chr.handle - 1;
                }
            }

            IFXV_DISC_TRACE2("  The descriptor search range for characteristic %d is 0x%04x-0x%04x%s\n",
                                server.char_index, s_handle, e_handle,
                                s_handle > e_handle ? " -- not valid" : "");

            if (s_handle <= e_handle)
            {
                IFXV_DISC_TRACE("  ** Discover descriptor for characteristic index %d, handle:%04x, val_handle:%04x, range:0x%04x-0x%04x\n",
                                server.char_index,
                                server.characteristics[server.char_index].chr.handle, server.characteristics[server.char_index].chr.val_handle,
                                s_handle, e_handle);

                wiced_bt_util_send_gatt_discover(server.conn_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, 0, s_handle, e_handle);
                return FALSE;
            }
            else
            {
                IFXV_DISC_TRACE("  ** No descriptor found for characteristic index %d, handle:%04x, val_handle:%04x\n",
                                server.char_index,
                                server.characteristics[server.char_index].chr.handle, server.characteristics[server.char_index].chr.val_handle);
            }
        }

    default:
        IFXV_DISC_TRACE("not handled\n");
        return TRUE;
    }
}

/*
 * Start to discover the service by index
 */
static void ifxv_discovery(uint8_t index)
{
    if (index < NUMBER_OF_SUPPORTED_SERVER)
    {
        server.index = index;
        wiced_bt_uuid_t * uuid = &server.services[index]->id.service_type;

        if (uuid->len == LEN_UUID_16)
        {
            IFXV_DISC_TRACE("Finding GATT database for %s, uuid16 = 0x%04x\n", server.services[index]->name, uuid->uu.uuid16);
            wiced_bt_util_send_gatt_discover( server.conn_id, GATT_DISCOVER_SERVICES_BY_UUID, uuid->uu.uuid16, FIRST_HANDLE, LAST_HANDLE);
        }
#ifdef INCLUDE_ATV
        else if (uuid->len == LEN_UUID_128)
        {
            IFXV_DISC_TRACE("Finding GATT database for %s, uuid128 = %A\n", server.services[index]->name, uuid->uu.uuid128, 16);
            wiced_bt_util_send_gatt_discover_by_uuid128( server.conn_id, uuid->uu.uuid128, FIRST_HANDLE, LAST_HANDLE);
        }
#endif
    }
    else
    {
        ifxv_event_data_t data_event;

        led_blink_stop(LINK_LED);

        WICED_BT_TRACE("Discovery Complete\n");
        /* Tell the application that the GATT Discovery failed */
        data_event.discovery.conn_id = server.conn_id;
        data_event.discovery.status = WICED_BT_GATT_SUCCESS;
        server.p_callback(IFXV_EVENT_DISCOVERY_COMPLETE, &data_event);

        hci_send_status(HCI_CONTROL_IFXV_STATUS_CONNECTED);

        // enable call CCCD flags
        ifxv_set_all_config_descriptors(TRUE);

        hci_send_pairing_complete_evt( WICED_BT_GATT_SUCCESS, ifxv_peer_addr(), BT_DEVICE_TYPE_BLE );

        // Notifiy audio that link is up and ready.
        audio_ready();
    }
}

/*
 ******* Public *************************************************************
 */

/*
 * Return TRUE if this notification is handled by service internally. Otherwise,
 * returning FALSE will continue pass to application for further process.
 */
wiced_bool_t discovery_custom_notification(wiced_bt_gatt_operation_complete_t * p_data)
{
    wiced_bt_gatt_data_t * att_value = &p_data->response_data.att_value;

    for (int i=0; i< NUMBER_OF_SUPPORTED_SERVER; i++)
    {
        if (service_cback(server.services[i], &p_data->response_data.att_value))
        {
            return TRUE;
        }
    }
    return FALSE;
}

charstc_t * discovery_characteristic(uint8_t index)
{
    return index < server.char_cnt ? &server.characteristics[index] : NULL;
}

/*
 * While application performs GATT discovery it shall pass discovery results for
 * for the handles that belong to BAC service, to this function.
 * Process discovery results from the stack.
 */
wiced_bt_gatt_status_t discovery_result(wiced_bt_gatt_discovery_result_t *p_data)
{
    switch (p_data->discovery_type) {
    case GATT_DISCOVER_SERVICES_ALL:
    case GATT_DISCOVER_SERVICES_BY_UUID:
        discovery_handle_service_result(&p_data->discovery_data.group_value);
        break;
    case GATT_DISCOVER_INCLUDED_SERVICES:
        break;
    case GATT_DISCOVER_CHARACTERISTICS:
        discovery_handle_characteristics_result(&p_data->discovery_data.characteristic_declaration);
        break;
    case GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
        discovery_handle_descriptor_result(&p_data->discovery_data.char_descr_info);
        break;
    default:
        WICED_BT_TRACE("%s unhandled type:%d\n", __FUNCTION__, p_data->discovery_type);
        break;
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * Handle discovery complete event
 */
wiced_bt_gatt_status_t discovery_complete(wiced_bt_gatt_discovery_complete_t* p_data)
{
    wiced_bt_gatt_discovery_type_t disc_type = app_gatt_discovery_complete_type(p_data);

    // Check to see characteristics & descriptors are all found
    if ( discovery_characteristics_detail(&server.services[server.index]->id, disc_type) )
    {
        // find the next service
        ifxv_discovery(server.index+1);
    }
    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is call when the link is up
 */
void discovery_link_up(uint16_t conn_id)
{
    server.conn_id = conn_id;
    led_blink(LINK_LED, 0, LED_DISCOVERY_BLINK_SPEED);
    hci_send_status(HCI_CONTROL_IFXV_STATUS_DISCOVERY);
    WICED_BT_TRACE("Start service discovery...\n");
    ifxv_discovery(0);
}

/*
 * This function is call when the link is down
 */
void discovery_link_down()
{
    server.conn_id = 0;
    server.char_cnt = 0;
    audio_down();
}

/*
 * Start up init for discovery. It registers the callback function.
 */
void discovery_init(ifxv_callback_t *p_callback)
{
    server.p_callback = p_callback;
}
