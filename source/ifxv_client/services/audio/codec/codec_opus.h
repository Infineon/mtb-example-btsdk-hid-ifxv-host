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
#ifndef CODEC_OPUS_H__
#define CODEC_OPUS_H__

#include "wiced_bt_types.h"

#define OPUS_SAMPLE_CNT         OPUS_ENC_FRAME_SIZE
#define OPUS_ENC_MS_PER_FRAME   20                                                  // 20 ms per frame
#define OPUS_ENC_SAMPLING_RATE  16000               // 16k samples @ 160 samples per frame, each frame is 10 ms. 16000/320 = 50 frames per sec. et. 20 ms / frame
#define OPUS_ENC_BITRATE        32000
#define OPUS_ENC_FRAME_SIZE     (OPUS_ENC_SAMPLING_RATE/50) // frame_size must be equal to (sampling_rate / N), (sampling_rate / 50) = 16000/50=320
                                                    // where N = 400, 200, 100, 50, 25, 50/3, ie how mnay frames per second.
                                                    // We use 50 frames per second, 16000 samples / 50 frames = 320 samples per frame.
#define OPUS_ENC_PACKET_SIZE    (OPUS_ENC_BITRATE*OPUS_ENC_MS_PER_FRAME/1000/8) // 20 ms per frame, 32000 * 20 / 1000 / 8 = 80
#define OPUS_ENC_OFFSET         2
#define OPUS_ENC_HEADER_SIZE    8
#define OPUS_ENC_SIZE           (OPUS_ENC_PACKET_SIZE + OPUS_ENC_HEADER_SIZE)       // 88
#define ENCODE_BUF_SIZE         (OPUS_ENC_OFFSET + OPUS_ENC_SIZE)                   // 90

typedef enum
{
    OPUS_STATUS_SUCCESS             = 0,
    OPUS_STATUS_PENDING             = 1,
    OPUS_STATUS_ERROR               = 2,
    OPUS_STATUS_INVALID_PARAMETERS  = 3,
} opus_status_t;

void     opus_init();
uint16_t opus_encode(int16_t * p_pcm, uint16_t sample_cnt, uint8_t * p_opus);                   // returns opus encoded data length
uint16_t opus_decode(uint8_t * p_opus_enc_data, uint16_t opus_enc_data_len, int16_t * p_pcm);   // returns sample cnt

#endif // CODEC_OPUS_H__
