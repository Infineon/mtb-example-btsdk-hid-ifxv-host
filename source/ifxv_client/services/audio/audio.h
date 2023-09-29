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

#include "ifxv_audio.h"
#ifdef INCLUDE_ATV
 #include "atv_audio.h"
#else
 #define atv_audio_send_capabilities()
 #define atv_audio_init()
 #define atv_audio_down()
 #define atv_audio_stop_req(reason)
 #define atv_audio_start_req(cfg)
#endif

typedef struct
{
    wiced_timer_t  duration_timer;  /* timer */
    uint8_t        started:1;
    audio_cfg_t    cfg;
} audio_data_t;

extern audio_data_t audio;

void audio_data(uint8_t ch, int16_t * pcm, uint16_t cnt);
void audio_started(audio_cfg_t * cfg);
void audio_stopped(audio_stop_reason_e reason);
void audio_start_req(audio_cfg_t * cfg);
void audio_stop_req(audio_stop_reason_e reason);
void audio_ready();
void audio_down();
void audio_init();
audio_cfg_t * audio_get_cfg();
uint8_t audio_profile();
void audio_default_cfg(server_info_t * host, server_info_t * dev);
