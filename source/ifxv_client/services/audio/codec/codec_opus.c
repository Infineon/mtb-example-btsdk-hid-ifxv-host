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
#ifdef ENABLE_OPUS_ENCODER
 #include "celt_encoder_api.h"
#endif
#include "celt_decoder_api.h"
#include "opus.h"

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

#ifdef ENABLE_OPUS_ENCODER
CELT_ENC_PARAMS celt_enc_params = {
    .sampling_rate  = OPUS_ENC_SAMPLING_RATE,
    .channels       = 1,
    .bitrate        = OPUS_ENC_BITRATE,
    .complexity     = 3,
    .use_vbr        = 0,                    // must be 0 (CBR only)
    .use_cvbr       = 0,                    // must be 0 (CBR only)
    .frame_size     = OPUS_ENC_FRAME_SIZE,  // frame_size default to (sampling_rate / 50) = 16000/50=320 ;
    .enc_handler = NULL,
};
#endif

CELT_DEC_PARAMS celt_dec_params = {
    .sampling_rate  = OPUS_ENC_SAMPLING_RATE,    /* 8k, 16k, 24k or 48k */
    .channels       = 1,                         /* mono or streo */
    .pkt_len        = OPUS_ENC_PACKET_SIZE,      /* Input packet length (bytes) */
    .frame_status   = 0,                         /* Frame status: 0:GOOD, 1:BAD (lost)  */
    .frame_size     = OPUS_ENC_FRAME_SIZE,       /* PCM samples per frame per channel, needed for PLC init*/
    .enable_plc     = 0,                         /* Enable PLC: 1:enable, 0:disable */
};

void opus_init()
{
#ifdef ENABLE_OPUS_ENCODER
    CELT_Encoder_Init(&celt_enc_params);
#endif
    CELT_Decoder_Init(&celt_dec_params);
}

#ifdef ENABLE_OPUS_ENCODER
// return encode data size
uint16_t opus_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_frame)
{
    uint16_t len;

    if (sample_cnt != OPUS_SAMPLE_CNT)
    {
        WICED_BT_TRACE("OPUS encode error! sample cnt %d is not %d\n", sample_cnt, OPUS_ENC_FRAME_SIZE);
        return 0;
    }

    //followed by CELT encoded audio data
    celt_enc_params.pcmBuffer = p_pcm;
    celt_enc_params.packet = p_frame;
    celt_opus_packet_get_samples_per_frame(data, OPUS_ENC_SAMPLING_RATE);
    return CELT_Encoder(&celt_enc_params); //encode one frame
}
#endif

// returns decoded sample count
uint16_t opus_decode(uint8_t * p_opus_enc_data, uint16_t opus_enc_data_len, int16_t * p_pcm)
{
    if ( opus_enc_data_len > OPUS_ENC_SIZE )
    {
        WICED_BT_TRACE("OPUS decoder error! Encoded packet size %d is over %d\n", opus_enc_data_len, OPUS_ENC_SIZE);
        return 0;
    }

    IFXV_CODEC_TRACE("OPUS packeet len %d, frame len = %d, %A \n", opus_enc_data_len, celt_opus_packet_get_samples_per_frame(p_opus_enc_data+OPUS_ENC_HEADER_SIZE, OPUS_ENC_SAMPLING_RATE), p_opus_enc_data, 8);

    celt_dec_params.pkt_len = OPUS_ENC_PACKET_SIZE;
    celt_dec_params.packet = p_opus_enc_data+OPUS_ENC_HEADER_SIZE;
    celt_dec_params.pcmBuffer = p_pcm;

    uint16_t sample_cnt = CELT_Decoder(&celt_dec_params);  // returns number of samples

    if (celt_dec_params.frame_status != 0) // Not Good?
    {
        WICED_BT_TRACE("Error.... OPUS decoder returned status %d\n", celt_dec_params.frame_status);
    }
    else if (sample_cnt != OPUS_SAMPLE_CNT)
    {
        WICED_BT_TRACE("Warning.... sample cnt %d is not %d after decode\n", sample_cnt, OPUS_SAMPLE_CNT);
    }
    else
    {
        IFXV_CODEC_TRACE2("OPUS decoded sample cnt = %d\n", sample_cnt);
    }
    return sample_cnt;
}
