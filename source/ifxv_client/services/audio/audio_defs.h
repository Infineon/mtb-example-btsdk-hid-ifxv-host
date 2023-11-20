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
#ifndef AUDIO_DEFS_H__
#define AUDIO_DEFS_H__

#include "wiced_bt_types.h"

#define STREAMING_TIME_LIMIT_IN_SEC  10         // use 0 for no limit
#define AUDIO_LED                    RED_LED

#define AUDIO_FRAME_SEQ  0
#define AUDIO_FRAME_CH   1
#define AUDIO_FRAME_DATA 2

#define BIT(n) (1<<n)

typedef enum
{
    IFX_CAP_PROFILE_IFXV  = BIT(0),
    IFX_CAP_PROFILE_ATV = BIT(1),
} ifxv_cap_profile_e;

#ifdef INCLUDE_ATV
 #define HOST_PROFILE_CAPABILITY (IFX_CAP_PROFILE_IFXV|IFX_CAP_PROFILE_ATV)
#else
 #define HOST_PROFILE_CAPABILITY (IFX_CAP_PROFILE_IFXV)
#endif

typedef enum
{
    IFX_AUDIO_STOP_REASON_TIMEOUT,
    IFX_AUDIO_STOP_REASON_USER_REQUEST,
    IFX_AUDIO_STOP_REASON_OTHER,
} audio_stop_reason_e;

typedef enum
{
    IFX_AUDIO_CODEC_PCM       = 0,
    IFX_AUDIO_CODEC_MSBC      = 1,
    IFX_AUDIO_CODEC_ADPCM     = 2,
    IFX_AUDIO_CODEC_OPUS      = 3,
    IFX_AUDIO_CODEC_MAX
} ifxv_audio_codec_e;

typedef enum
{
    IFX_CAP_CODEC_MSBC  = BIT(0),
    IFX_CAP_CODEC_ADPCM = BIT(1),
    IFX_CAP_CODEC_OPUS  = BIT(2),
} ifxv_cap_codec_e;

typedef enum
{
    IFX_CAP_8_BIT_DATA  = BIT(0),
    IFX_CAP_16_BIT_DATA = BIT(1),
} ifxv_cap_data_unit_e;

typedef enum
{
    IFX_CAP_8KHZ        = BIT(0),
    IFX_CAP_16KHZ       = BIT(1),
} ifxv_cap_sampling_rate_e;

typedef enum
{
    IFX_8_BIT_DATA      = 0,
    IFX_16_BIT_DATA     = 1,
    IFX_SUPPORTED_DATA
} ifxv_data_unit_e;

typedef enum
{
    IFX_8KHZ            = 0,
    IFX_16KHZ           = 1,
    IFX_SUPPORTED_SAMPLE_RATE
} ifxv_sample_rate_e;

#pragma pack(1)
typedef struct
{
    uint8_t         codec;
    uint8_t         data_unit;
    uint8_t         sampling_rate;
    uint8_t         streaming_time_limit_in_sec;
    uint16_t        len;
} audio_cfg_t;
#pragma pack()

#endif // AUDIO_DEFS_H__
