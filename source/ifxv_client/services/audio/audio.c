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
 * IFXV audio.
 *
 */

#include "app.h"

#if AUDIO_TRACE == 1
 #define     IFXV_AUDIO_TRACE       WICED_BT_TRACE
 #define     IFXV_AUDIO_TRACE2(...)
#elif AUDIO_TRACE == 2
 #define     IFXV_AUDIO_TRACE       WICED_BT_TRACE
 #define     IFXV_AUDIO_TRACE2      WICED_BT_TRACE
#else
 #define     IFXV_AUDIO_TRACE(...)
 #define     IFXV_AUDIO_TRACE2(...)
#endif

/******************************************************
 *  Data
 ******************************************************/
audio_data_t audio = {.cfg={IFX_AUDIO_CODEC_ADPCM, IFX_16_BIT_DATA, IFX_16KHZ, 10, 134}};

/******************************************************
 *  Private functions
 ******************************************************/

/*
 *   Private fnction
 */

static void audio_timeout(TIMER_PARAM_TYPE count)
{
    WICED_BT_TRACE("audio duration timeout\n");
    audio_stop_req(IFX_AUDIO_STOP_REASON_TIMEOUT);
}

/******************************************************
 *   Public fnction
 ******************************************************/

/*
 * Reciving PCM raw data
 */
void audio_data(uint8_t ch, int16_t * p_data, uint16_t sample_cnt)
{
    IFXV_AUDIO_TRACE2("audio data: %d samples\n", sample_cnt);
    hci_send_audio_data((uint8_t *) p_data, sample_cnt * 2);
}

/*
 *
 */
void audio_started(audio_cfg_t * cfg)
{
    led_on(AUDIO_LED);

    // Notify HCI audio has started
    hci_send_start(cfg);

    audio.started = TRUE;

    if ( !wiced_is_timer_in_use(&audio.duration_timer) && audio.cfg.streaming_time_limit_in_sec && (audio.cfg.codec !=IFX_AUDIO_CODEC_PCM) )
    {
        wiced_start_timer (&audio.duration_timer, audio.cfg.streaming_time_limit_in_sec);
    }
}

/*
 * audio stopped, notify application
 */
void audio_stopped(audio_stop_reason_e reason)
{
    led_off(AUDIO_LED);

    if (audio.started)
    {
        audio.started = FALSE;
        wiced_stop_timer (&audio.duration_timer);
        // Notify HCI audio has started
        hci_send_stop(reason);
    }
}

/*
 * This start request is called by HCI Control interface.
 */
void audio_start_req(audio_cfg_t * cfg)
{
    // save the configuration
    memcpy(&audio.cfg, cfg, sizeof(audio_cfg_t));

    // propagate to each audio service
    atv_audio_start_req(cfg);
    ifxv_audio_start_req(cfg);
}

void audio_stop_req(audio_stop_reason_e reason)
{
    wiced_stop_timer (&audio.duration_timer);
    atv_audio_stop_req(reason);
    ifxv_audio_stop_req(reason);
}

/*
 *
 */
void audio_ready()
{
    atv_audio_send_capabilities();
    ifxv_audio_send_capabilities();
}

/*
 *
 */
void audio_down()
{
    atv_audio_down();
    ifxv_audio_down();
    hci_send_disconnect_event();
    led_off(AUDIO_LED);
}

/*
 *
 */
void audio_init()
{
    wiced_init_timer (&audio.duration_timer, &audio_timeout, 0, WICED_SECONDS_TIMER );
    ifxv_audio_init();
    atv_audio_init();
}

/*
 *
 */
audio_cfg_t * audio_get_cfg()
{
    return &audio.cfg;
}

/*
 *
 */
uint8_t audio_profile()
{
    uint8_t cap = service_found(&audio_ifxv) ? IFX_CAP_PROFILE_IFXV : 0;

#ifdef INCLUDE_ATV
    if (service_found(&audio_atv))
    {
        cap |=  IFX_CAP_PROFILE_ATV;
    }
#endif
    return cap;
}

/*
 *
 */
void audio_default_cfg(server_info_t * host, server_info_t * dev)
{
    uint8_t codec = host->cap.codec & dev->cap.codec;
    if (codec & IFX_CAP_CODEC_OPUS)
    {
        audio.cfg.codec = IFX_AUDIO_CODEC_OPUS;
        audio.cfg.len = OPUS_ENCODE_BUF_SIZE;
    }
    else if (codec & IFX_CAP_CODEC_MSBC)
    {
        audio.cfg.codec = IFX_AUDIO_CODEC_MSBC;
        audio.cfg.len = 180; // to be checked
    }
    else if (codec & IFX_CAP_CODEC_ADPCM)
    {
        audio.cfg.codec = IFX_AUDIO_CODEC_ADPCM;
        audio.cfg.len = APDCM_ENCODE_BUF_SIZE;
    }
    else
    {
        audio.cfg.codec = IFX_AUDIO_CODEC_PCM;
    }
    audio.cfg.data_unit = IFX_16_BIT_DATA;
    audio.cfg.sampling_rate = host->cap.sampling_rate & host->cap.sampling_rate & IFX_CAP_16KHZ ? IFX_16KHZ : IFX_8KHZ;
    audio.cfg.streaming_time_limit_in_sec = 10;
    hci_send_audio_cfg(HCI_CONTROL_IFXVH_EVENT_AUDIO_CFG, &audio.cfg);
}
