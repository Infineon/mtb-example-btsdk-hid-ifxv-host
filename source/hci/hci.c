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

#if HCI_TRACE == 1
 #define     IFXV_HCI_TRACE      WICED_BT_TRACE
#else
 #define     IFXV_HCI_TRACE(...)
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/
typedef struct
{
    uint8_t         profile;
    server_info_t   cap;
} hci_cap_t;

/******************************************************************************
 *                           Variables Definitions
 ******************************************************************************/

/******************************************************************************
 *                             External Definitions
 ******************************************************************************/

/*
 *  Pass protocol traces up through the UART
 */
#if defined(ENABLE_HCI_TRACE)
void hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    transport_send_hci_trace( type, p_data, length );
}
#endif

static void hci_send_capability(uint16_t code, uint8_t profile, server_info_t * cap)
{
    if (profile)
    {
        hci_cap_t hci_cap;

        hci_cap.profile = profile;
        memcpy(&hci_cap.cap, cap, sizeof(server_info_t));

        wiced_transport_send_data(code, (uint8_t *) &hci_cap, sizeof(hci_cap_t));
    }
    else
    {
        wiced_transport_send_data(code, NULL, 0);
    }
}

static void hci_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_IFXVH;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);

    hci_send_capability(HCI_CONTROL_IFXVH_EVENT_HOST_CAPABILITIES, HOST_PROFILE_CAPABILITY, ifxv_audio_host_cap());
    hci_send_device_capability();
}

void hci_send_audio_cfg(uint16_t code, audio_cfg_t * cfg)
{
    wiced_transport_send_data(code, (uint8_t *)cfg, cfg==NULL ? 0 : sizeof(audio_cfg_t));
}

void hci_send_disconnect_event()
{
    hci_send_capability(HCI_CONTROL_IFXVH_EVENT_DEVICE_CAPABILITIES, 0, NULL);
}

void hci_send_device_capability()
{
    if (ifxv_is_connected())
    {
        hci_send_capability(HCI_CONTROL_IFXVH_EVENT_DEVICE_CAPABILITIES, audio_profile(), ifxv_audio_device_cap());
    }
    else
    {
        hci_send_disconnect_event();
    }
}

/* transport status */
void hci_transport_status( wiced_transport_type_t type )
{
    IFXV_HCI_TRACE("ifxv_client_transport connected type: %d ", type);
}

void hci_send_start(audio_cfg_t * cfg)
{
    hci_send_audio_cfg(HCI_CONTROL_IFXVH_EVENT_AUDIO_START, cfg);
}

void hci_send_stop(uint8_t reason)
{
    wiced_transport_send_data(HCI_CONTROL_IFXVH_EVENT_AUDIO_STOP, &reason, 1);
}

void hci_send_audio_data(uint8_t * ptr, uint16_t len)
{
    IFXV_HCI_TRACE("HCI send audio data: %d bytes\n", len);
    while (len)
    {
        uint16_t size = len >= MAX_SEND_HCI_BYTE_CNT ? MAX_SEND_HCI_BYTE_CNT : len;

        wiced_transport_send_data(HCI_CONTROL_IFXVH_EVENT_AUDIO_DATA, ptr, size);
        len -= size;
        ptr += size;
    }
}

void hci_send_msg(uint8_t * str)
{
    wiced_transport_send_data(HCI_CONTROL_IFXVH_EVENT_MSG, str, strlen((const  char *)str)+1);
}

void hci_send_status(uint8_t status)
{
    wiced_transport_send_data(HCI_CONTROL_IFXVH_EVENT_STATUS_CHANGED, &status, 1);
}

uint32_t hci_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t                opcode;
    uint8_t*                p_data = p_buffer;
    uint16_t                payload_len;
    uint8_t                 status = HCI_CONTROL_STATUS_SUCCESS;
    wiced_bt_gatt_status_t  gatt_status = WICED_BT_GATT_SUCCESS;
    uint8_t  addr[]         = {0x0A, 0X0B, 0xC, 0x0D, 0x0E, 0X0F};

    IFXV_HCI_TRACE("hci_control_proc_rx_cmd:%d\n", length);

    if ( !p_data )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if (length < 4)
    {
        WICED_BT_TRACE( "[%s] invalid params\n" , __FUNCTION__);
        wiced_transport_free_buffer( p_data );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        STREAM_TO_UINT16(opcode, p_data);  // Get OpCod
        STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length
        IFXV_HCI_TRACE("cmd_opcode 0x%04X payload_len %d \n", opcode, payload_len);

        switch ( opcode )
        {
            case HCI_CONTROL_IFXVH_COMMAND_GET_HOST_CAPABILITIES:
                IFXV_HCI_TRACE("HCI_CONTROL_IFXVH_COMMAND_GET_HOST_CAPABILITIES\n");
                hci_send_capability(HCI_CONTROL_IFXVH_EVENT_HOST_CAPABILITIES, HOST_PROFILE_CAPABILITY, ifxv_audio_host_cap());
                break;

            case HCI_CONTROL_IFXVH_COMMAND_GET_DEVICE_CAPABILITIES:
                IFXV_HCI_TRACE("HCI_CONTROL_IFXVH_COMMAND_GET_DEVICE_CAPABILITIES\n");
                hci_send_device_capability();
                break;

            case HCI_CONTROL_IFXVH_COMMAND_GET_AUDIO_CFG:
                IFXV_HCI_TRACE("HCI_CONTROL_IFXVH_COMMAND_GET_AUDIO_CFG\n");
                hci_send_audio_cfg(HCI_CONTROL_IFXVH_EVENT_AUDIO_CFG, audio_get_cfg());
                break;

            case HCI_CONTROL_IFXVH_COMMAND_AUDIO_START:
                if (payload_len == sizeof(audio_cfg_t))
                {
                    WICED_BT_TRACE("Start audio request from HCI\n");
                    audio_start_req((audio_cfg_t *) p_data);
                }
                else
                {
                    WICED_BT_TRACE("Invalid cfg to start audio\n");
                }
                break;

            case HCI_CONTROL_IFXVH_COMMAND_AUDIO_STOP:
                WICED_BT_TRACE("Stop audio request from HCI\n");
                audio_stop_req(IFX_AUDIO_STOP_REASON_USER_REQUEST);
                break;

            case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
                IFXV_HCI_TRACE("HCI_CONTROL_MISC_COMMAND_GET_VERSION\n");
                hci_handle_get_version();
                break;

            case HCI_CONTROL_IFXVH_COMMAND_CONNECT:
                IFXV_HCI_TRACE("HCI_CONTROL_IFXVH_COMMAND_CONNECT\n");
                app_enter_pairing();
                break;

            default:
                IFXV_HCI_TRACE("ignored:%02X payload_len:%d \n", opcode, payload_len);
                status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
                break;
        }
    }

    wiced_transport_send_data(HCI_CONTROL_BATT_CLIENT_EVENT_STATUS, &status, 1);

    // Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_buffer );
    return HCI_CONTROL_STATUS_SUCCESS;
}

void hci_send_disconnect_evt( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    IFXV_HCI_TRACE("Send HCI disconnect event, reason=%d\n", reason);

    wiced_transport_send_data( HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

void hci_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    IFXV_HCI_TRACE("Send HCI connect event %B, type:%d, role:%d\n", addr, addr_type, role);

    wiced_transport_send_data( HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 * Send notification to the host that pairing has been completed
 */
void hci_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type )
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;
    int i;

    *p++ = result;

    for ( i = 0; i < 6; i++ )
        *p++ = p_bda[5 - i];

    *p++ = type;

    IFXV_HCI_TRACE("Send HCI pairing complete event, result=%d, addr=%B, type:%d\n", result, p_bda, type);

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, tx_buf, ( int ) ( p - tx_buf ) );
}
