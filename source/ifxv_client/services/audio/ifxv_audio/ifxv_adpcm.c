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

#if CODEC_TRACE == 1
 #define     IFXV_CODEC_TRACE       WICED_BT_TRACE
 #define     IFXV_CODEC_TRACE2(...)
#elif CODEC_TRACE == 2
 #define     IFXV_CODEC_TRACE       WICED_BT_TRACE
 #define     IFXV_CODEC_TRACE2      WICED_BT_TRACE
#else
 #define     IFXV_CODEC_TRACE(...)
 #define     IFXV_CODEC_TRACE2(...)
#endif

#define ADPCM_FRAME_VALPREV_L       2
#define ADPCM_FRAME_VALPREV_H       3
#define ADPCM_FRAME_INDEX           4
#define APPCM_FRAME_NOT_USED        5
#define ADPCM_FRAME_HEADER_SIZE     6

#define ifxv_adpcm_set_state(v,i)              {adpcm.codec_state.valprev = v; adpcm.codec_state.index = i;}

/******************************************************
 *                  typedef
 ******************************************************/
typedef struct
{
    uint8_t     seq;
    uint8_t     ch;
    CodecState  codec_state;
    uint8_t     header_processed:1;
    int16_t     pcm[IFXV_SAMPLE_CNT_PER_ADPCM_FRAME];
    int16_t     sample_cnt;
} ifxv_adpcm_t;

/******************************************************
 *                  Data
 ******************************************************/
static ifxv_adpcm_t adpcm;

/******************************************************
 *              Functions
 ******************************************************/

void ifxv_adpcm_init()
{
    adpcm.header_processed = FALSE;
    adpcm.seq = 0;
    memset(&adpcm.codec_state, 0, sizeof(CodecState));
}

/*
 *
 */
void ifxv_adpcm_data( uint8_t * p_frame,  uint16_t len )
{
    if (!audio.started)
    {
        // ignore any audio data if audio is not active.
        return;
    }

    // If this fragment contains header, process header info
    if (!adpcm.header_processed)
    {
        adpcm.sample_cnt = 0;
        adpcm.ch = p_frame[AUDIO_FRAME_CH];

        if (adpcm.seq != p_frame[AUDIO_FRAME_SEQ])
        {
            WICED_BT_TRACE("Error: audio frame out of sequence, expected %d, received %d\n", adpcm.seq, p_frame[AUDIO_FRAME_SEQ]);

            int16_t valprev = (int16_t) (( p_frame[ADPCM_FRAME_VALPREV_L] & 0xff) | ((p_frame[ADPCM_FRAME_VALPREV_H] & 0xff)<<8));
            adpcm.seq = p_frame[AUDIO_FRAME_SEQ];
            ifxv_adpcm_set_state(valprev, p_frame[ADPCM_FRAME_INDEX]);
        }
        adpcm.seq++;
        p_frame += ADPCM_FRAME_HEADER_SIZE;
        len -= ADPCM_FRAME_HEADER_SIZE;
        adpcm.header_processed = TRUE;
    }
    int numSamples = len * 2;  // compress ratio 4:1, out data = len * 4 / 2 (2-bytes, 16-data)

    // make sure the numSamples is within the buffer size limit
    if ((adpcm.sample_cnt + numSamples) > IFXV_SAMPLE_CNT_PER_ADPCM_FRAME)
    {
        numSamples = IFXV_SAMPLE_CNT_PER_ADPCM_FRAME - adpcm.sample_cnt;
    }

    IFXV_CODEC_TRACE2("IFXV: sample_cnt %d, numSamples %d\n", adpcm.sample_cnt, numSamples);
    decode(&adpcm.codec_state, p_frame, numSamples, &adpcm.pcm[adpcm.sample_cnt]);

    // if pcm buffer is full, report the data
    adpcm.sample_cnt += numSamples;
    if (adpcm.sample_cnt >= IFXV_SAMPLE_CNT_PER_ADPCM_FRAME)
    {
        IFXV_CODEC_TRACE2("Notify data, cnt=%d\n", adpcm.sample_cnt);
        audio_data(adpcm.ch, adpcm.pcm, adpcm.sample_cnt);
        adpcm.header_processed = FALSE;
    }
}
