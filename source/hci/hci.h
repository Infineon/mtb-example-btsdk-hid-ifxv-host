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

#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "audio.h"

#define MAX_SEND_HCI_SAMPLE_CNT 126    // max only can send 126 samples each transaction
#define MAX_SEND_HCI_BYTE_CNT (MAX_SEND_HCI_SAMPLE_CNT*2)

void hci_send_stop(uint8_t reason);
void hci_send_start(audio_cfg_t * cfg);
void hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
uint32_t  hci_rx_cmd(uint8_t *p_data, uint32_t length);
void hci_transport_status( wiced_transport_type_t type );
void hci_send_disconnect_event();
void hci_send_device_capability();
void hci_send_audio_data(uint8_t * ptr, uint16_t len);
void hci_send_status(uint8_t status);
void hci_send_audio_cfg(uint16_t code, audio_cfg_t * cfg);
void hci_send_disconnect_evt( uint8_t reason, uint16_t con_handle );
void hci_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
void hci_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type );

