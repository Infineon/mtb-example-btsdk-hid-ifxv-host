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

/******************************************************
 *                  typedef
 ******************************************************/
typedef struct
{
    uint8_t     seq;
    int16_t     pcm[OPUS_SAMPLE_CNT];
} ifxv_opus_t;

/******************************************************
 *                  Data
 ******************************************************/
static ifxv_opus_t opus;


/******************************************************
 *              Functions
 ******************************************************/
void ifxv_opus_init()
{
    opus.seq = 0;
    opus_init(opus.pcm);
}

/*
 *
 */
void ifxv_opus_data( uint8_t * p_frame,  uint16_t len )
{
    if (p_frame[AUDIO_FRAME_SEQ] != opus.seq)
    {
        IFXV_CODEC_TRACE("OPUS: frame %d is out of sequence, expected %d\n", p_frame[AUDIO_FRAME_SEQ], opus.seq);
        opus.seq = p_frame[AUDIO_FRAME_SEQ];
    }
    opus.seq++;

    uint16_t sample_cnt = opus_decode(&p_frame[OPUS_ENC_OFFSET], len-OPUS_ENC_OFFSET, opus.pcm);

    if (sample_cnt != OPUS_SAMPLE_CNT)
    {
        WICED_BT_TRACE("OPUS: sample count %d is not %d\n", sample_cnt, OPUS_SAMPLE_CNT);
    }

    IFXV_CODEC_TRACE2("Notify data, cnt=%d\n", sample_cnt);
    audio_data(p_frame[AUDIO_FRAME_CH], opus.pcm, sample_cnt);
}
