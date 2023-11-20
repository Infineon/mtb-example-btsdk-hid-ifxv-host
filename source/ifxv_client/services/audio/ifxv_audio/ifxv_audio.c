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

#define RESPONSE_TIMEOUT_DEFAULT     2

#define SAMPLE_CNT_PER_PCM_FRAME     64
#define SAMPLE_CNT_PER_MSBC_FRAME    360
#define MAX_SAMPLE_CNT_PER_FRAME     SAMPLE_CNT_PER_MSBC_FRAME  // largest of all

#define PCM_DATA_SIZE_PER_FRAME      128
#define OPUS_DATA_SIZE_PER_FRAME     80
#define MSBC_DATA_SIZE_PER_FRAME     180

#define PCM_HEADER_SIZE              1

#define VER_MAJOR 0
#define VER_MINOR 2

/******************************************************
 * WARNING:  The below UUID is not a valid UUID and
 *           cannot be used in shipping products
 *           A valid 16-bit UUID must be purchased, see
 *           README.md for details
 ******************************************************/
#define IFXV_UUID      0x0000

#define IFXV_DATA_UUID {0xd2, 0x65, 0xf8, 0x72, 0x30, 0x8a, 0x43, 0xa7, 0x7c, 0x48, 0x5e, 0xec, 0xea, 0x05, 0x96, 0xec}
#define IFXV_HOST_UUID {0x3d, 0x3a, 0x46, 0x15, 0x14, 0x4f, 0x9d, 0xa9, 0xaa, 0x48, 0xbb, 0x8d, 0x37, 0xaf, 0x34, 0x0e}
#define IFXV_DEV_UUID  {0x5a, 0x64, 0x60, 0xd0, 0x8e, 0x75, 0x0b, 0x97, 0xef, 0x44, 0xc5, 0x67, 0x3e, 0xb5, 0x97, 0xb5}

#define BIT(n) (1<<n)

/******************************************************
 *                  typedef
 ******************************************************/
typedef enum
{
    IFXV_DATA_CHAR,
    IFXV_HOST_CHAR,
    IFXV_DEV_CHAR,
    SUPPORTED_CHARACTERISTIC_CNT,
} ifxv_char_e;

typedef enum
{
    IFXV_STATE_SERVER_NOT_AVAILABLE,
    IFXV_STATE_SENT_CAP,
    IFXV_STATE_IDLE,
    IFXV_STATE_STARTED,
    IFXV_STATE_ACTIVE,
    IFXV_STATE_STOPPED,
    IFXV_STATE_MAX,
} ifxv_state_e;

typedef enum
{
    IFX_AUDIO_CAP        = 1,
    IFX_AUDIO_CAP_RESP   = 2,
    IFX_AUDIO_RESP       = 3,
    IFX_AUDIO_START_REQ  = 4,
    IFX_AUDIO_START      = 5,
    IFX_AUDIO_STOP_REQ   = 6,
    IFX_AUDIO_STOP       = 7,
} ifxv_msg_e;

typedef enum
{
    IFX_REASON_USER_REQUEST     = 0,
    IFX_REASON_NOT_CAPABLE      = 1,
    IFX_REASON_TIMEOUT          = 2,
    IFX_REASON_INVALID_CFG      = 3,
    IFX_REASON_CAP_TOO_LONG     = 4,
    IFX_REASON_CAP_TOO_SHORT    = 5,
    IFX_REASON_AUDIO_NOT_ACTIVE = 6,
    IFX_REASON_BUSY             = 7,
    IFX_REASON_AUDIO_STOPPED    = 8,
    IFX_REASON_INVALID_DATA_LEN = 9,
} ifxv_reason_e;

#pragma pack(1)
typedef struct
{
    uint8_t         msg_type;
    server_info_t   info;
} ifxv_cap_msg_t;

typedef struct
{
    uint8_t          msg_type;
    audio_cfg_t cfg;
} ifxv_start_msg_t;

typedef struct
{
    uint8_t         msg_type;
    uint8_t         reason;
} ifxv_rsp_msg_t;

typedef ifxv_rsp_msg_t ifxv_stop_req_msg_t;

typedef struct
{
    uint8_t         sequence;
} ifxv_pcm_header_t;

typedef struct
{
    uint8_t         sequence;
} ifxv_msbc_t;

typedef struct
{
    uint8_t         msg_type;
    uint32_t        len;
} ifxv_opus_header_t;

#pragma pack()

typedef struct
{
    server_info_t   server;
    audio_cfg_t     cfg;
    wiced_timer_t   duration_timer; /* timer */
    uint8_t         state;
    uint8_t         seq;
    uint8_t         time_out_in_sec;
} ifxv_audio_t;

/******************************************************
 *                  Data
 ******************************************************/
static ifxv_cap_msg_t host_cap_msg = {
.msg_type = IFX_AUDIO_CAP,
.info.version  = {VER_MAJOR, VER_MINOR},
#ifdef INCLUDE_MSBC
.info.cap      = {IFX_CAP_CODEC_MSBC | IFX_CAP_CODEC_ADPCM | IFX_CAP_CODEC_OPUS, // mSBC, ADPCM, OPUS
#else
.info.cap      = {IFX_CAP_CODEC_ADPCM | IFX_CAP_CODEC_OPUS, // mSBC, ADPCM, OPUS
#endif
                 IFX_CAP_16_BIT_DATA,                   // can do 16-bit data only
                 IFX_CAP_8KHZ | IFX_CAP_16KHZ,          // 8kHz or 16 kHz
                 STREAMING_TIME_LIMIT_IN_SEC }
};

static ifxv_audio_t ifxv = {0};

/******************************************************
 *               Private Functions
 ******************************************************/

/*
 *
 */
static void stop_timer(uint8_t state)
{
    if (&ifxv.duration_timer)
    {
        wiced_stop_timer(&ifxv.duration_timer);
    }
    ifxv.state = state;
}

/*
 *
 */
static void start_timer(uint8_t state)
{
    stop_timer(state);
    wiced_start_timer (&ifxv.duration_timer, ifxv.server.cap.resp_time_limit_in_sec);
}

/*
 *
 */
static wiced_bt_gatt_status_t send_msg(void * ptr, uint16_t len)
{
    charstc_t * char_p = discovery_characteristic(audio_ifxv.char_ptr[IFXV_HOST_CHAR].index);
    if (char_p!=NULL)
    {
        wiced_bt_gatt_status_t status = ifxv_op_write( char_p->chr.val_handle, ptr, len );
        IFXV_AUDIO_TRACE("IFXV: Sent msg, status:%d (%04x) -- %A\n", status, status, ptr, len);
        return status;
    }
    return WICED_BT_GATT_INSUF_RESOURCE;
}

/*
 *
 */
static wiced_bt_gatt_status_t send_response(uint8_t type, uint8_t reason)
{
    ifxv_rsp_msg_t msg;

    msg.msg_type = type;
    msg.reason   = reason;

    return send_msg(&msg, sizeof(ifxv_rsp_msg_t));
}

/*
 *
 */
static void ifxv_audio_stop(uint8_t reason)
{
    send_response(IFX_AUDIO_STOP, reason);
    ifxv.state = IFXV_STATE_IDLE;

    // notify app audio is stopped, translate IFXV reason to common audio reason
    switch (reason) {
    case IFX_REASON_TIMEOUT:
        reason = IFX_AUDIO_STOP_REASON_TIMEOUT;
        break;
    case IFX_REASON_USER_REQUEST:
        reason = IFX_AUDIO_STOP_REASON_USER_REQUEST;
        break;
    default:
        reason = IFX_AUDIO_STOP_REASON_OTHER;
        break;
    }

    // Notify Application audio is stopped
    audio_stopped(reason);
}

/*
 *
 */
static void ifxv_audio_timeout(TIMER_PARAM_TYPE count)
{
    switch (ifxv.state) {
    case IFXV_STATE_STOPPED:
    case IFXV_STATE_STARTED:
        // no response from server after sending start/stop command
        // we send the stop
        ifxv_audio_stop(IFX_REASON_TIMEOUT);
        break;

    case IFXV_STATE_ACTIVE:
        break;

    case IFXV_STATE_SENT_CAP:
        ifxv.state = IFXV_STATE_IDLE;
        break;
    }
}

/*************************************************************
 * handle_data_cback -- received audio data.
 */
static wiced_bool_t handle_data_cback( wiced_bt_gatt_data_t * gatt_data_p )
{
    IFXV_AUDIO_TRACE2("ifxv audio data: len=%d\n", gatt_data_p->len);

    if (ifxv.state == IFXV_STATE_STARTED)
    {
        stop_timer(IFXV_STATE_ACTIVE);
    }

    switch (ifxv.cfg.codec) {
    case IFX_AUDIO_CODEC_PCM:
        audio_data(gatt_data_p->p_data[AUDIO_FRAME_CH], (int16_t *) &gatt_data_p->p_data[AUDIO_FRAME_DATA], (gatt_data_p->len-2)/2);
        break;
    case IFX_AUDIO_CODEC_MSBC:
        ifxv_msbc_data(gatt_data_p->p_data, gatt_data_p->len);
        break;
    case IFX_AUDIO_CODEC_ADPCM:
        ifxv_adpcm_data(gatt_data_p->p_data, gatt_data_p->len);
        break;
    case IFX_AUDIO_CODEC_OPUS:
        ifxv_opus_data(gatt_data_p->p_data, gatt_data_p->len);
        break;
    }
    return FALSE;
}

/*
 *
 */
static wiced_bool_t handle_host_cback( wiced_bt_gatt_data_t * gatt_data_p)
{
    WICED_BT_TRACE("ifxv handle host cback -- hm? This shouldn't happen");
    return FALSE;
}

/*
 *
 */
static void handle_cap_resp(ifxv_cap_msg_t * msg, uint16_t len)
{
    if (len > sizeof(ifxv_cap_msg_t))
    {
        WICED_BT_TRACE("capability data length too long\n");
        // will only copy valid length.
        len = sizeof(ifxv_cap_msg_t);
        send_response(IFX_AUDIO_RESP,IFX_REASON_CAP_TOO_LONG);
    }
    else if (len < sizeof(ifxv_cap_msg_t))
    {
        WICED_BT_TRACE("Invalid capability data length %d, expected %d\n", len, sizeof(ifxv_cap_msg_t));
        send_response(IFX_AUDIO_RESP, IFX_REASON_CAP_TOO_SHORT);
        // we copy whatever is given anyway...
    }

    // The response timeout value 0 is not allowed. If so, use RESPONSE_TIMEOUT_DEFAULT
    if (!msg->info.cap.resp_time_limit_in_sec)
    {
        msg->info.cap.resp_time_limit_in_sec = RESPONSE_TIMEOUT_DEFAULT;
    }

    memcpy(&ifxv.server, &msg->info, len-1);

    IFXV_AUDIO_TRACE("CAP RESP:  Ver:%d.%d, Codec:%02X, DataUnit:%02X, SRate:%02X, RespTO:%d\n",
                     ifxv.server.version.major, ifxv.server.version.minor,
                     ifxv.server.cap.codec, ifxv.server.cap.data_unit,
                     ifxv.server.cap.sampling_rate, ifxv.server.cap.resp_time_limit_in_sec);

    // notify hci
    hci_send_device_capability();
    audio_default_cfg( ifxv_audio_host_cap(), ifxv_audio_device_cap());
}

/*
 *
 */
static void handle_resp(ifxv_rsp_msg_t * msg, uint16_t len)
{
    switch (ifxv.state) {
    case IFXV_STATE_STOPPED:
        if (msg->reason == IFX_REASON_AUDIO_STOPPED)
        {
            ifxv.state = IFXV_STATE_IDLE;
        }
        break;
    }
}

/*
 *
 */
static void handle_start_req(ifxv_start_msg_t * msg, uint16_t len)
{
    uint8_t reason = IFX_REASON_BUSY;
    audio_cfg_t * cfg = &msg->cfg;

    IFXV_AUDIO_TRACE("Received start request: codec=%d, dataUnit:%d, sampleRate:%d\n", cfg->codec, cfg->data_unit, cfg->sampling_rate);
    if (ifxv.state == IFXV_STATE_IDLE)
    {
        reason = IFX_REASON_INVALID_CFG;
        if (  (len == sizeof(ifxv_start_msg_t))                 // valid config length
           && (cfg->codec < IFX_AUDIO_CODEC_MAX)                // valid supported codec
           && (cfg->data_unit == IFX_16_BIT_DATA)               // only support 16-bit data size
           && (cfg->sampling_rate < IFX_SUPPORTED_SAMPLE_RATE)) // valid supported sample rate
        {

            // find out what value we should use for duration timeout
            // Server   Client                                Result
            // ------   ------                                ------
            //   0      STREAMING_TIME_LIMIT_IN_SEC==0    --> 0
            //   0      STREAMING_TIME_LIMIT_IN_SEC!=0    --> STREAMING_TIME_LIMIT_IN_SEC
            //   n      STREAMING_TIME_LIMIT_IN_SEC==0    --> n
            //   n      STREAMING_TIME_LIMIT_IN_SEC!=0    --> MIN(n, STREAMING_TIME_LIMIT_IN_SEC)
            //
            if (!cfg->streaming_time_limit_in_sec)
            {
                // result is whatever defined in STREAMING_TIME_LIMIT_IN_SEC
                cfg->streaming_time_limit_in_sec = STREAMING_TIME_LIMIT_IN_SEC;
            }
            else
            {
                #if STREAMING_TIME_LIMIT_IN_SEC
                // STREAMING_TIME_LIMIT_IN_SEC is non-zero, use the smaller number
                // MIN(msg->audio_streaming_limit_in_sec, STREAMING_TIME_LIMIT_IN_SEC)
                if (cfg->streaming_time_limit_in_sec > STREAMING_TIME_LIMIT_IN_SEC)
                {
                    cfg->streaming_time_limit_in_sec = STREAMING_TIME_LIMIT_IN_SEC;
                }
                #endif
            }

            memcpy(&ifxv.cfg, cfg, sizeof(audio_cfg_t));

            switch (cfg->codec) {
            case IFX_AUDIO_CODEC_ADPCM:
                ifxv_adpcm_init();
                break;
            case IFX_AUDIO_CODEC_OPUS:
                ifxv_opus_init();
                break;
            case IFX_AUDIO_CODEC_MSBC:
                ifxv_msbc_reset();
                break;
            }

            audio_started(cfg);

            msg->msg_type = IFX_AUDIO_START;
            IFXV_AUDIO_TRACE("Sending start cmd: codec=%d, dataUnit:%d, sampleRate:%d, AudioTO:%d\n", cfg->codec, cfg->data_unit, cfg->sampling_rate, cfg->streaming_time_limit_in_sec);
            send_msg(msg, sizeof(ifxv_start_msg_t));
            start_timer(IFXV_STATE_STARTED);

            return;
        }
    }
    send_response(IFX_AUDIO_RESP,reason);
}

/*
 *
 */
static void handle_stop_request(ifxv_stop_req_msg_t * msg, uint16_t len)
{
    IFXV_AUDIO_TRACE("Received stopped request, reason %d\n", msg->reason);
    switch (ifxv.state) {
    case IFXV_STATE_STARTED:
    case IFXV_STATE_ACTIVE:
    case IFXV_STATE_STOPPED:
        ifxv_audio_stop(msg->reason);
        break;
    }
}

/*************************************************************
 * handle_dev_cback - Received control message from device (server)
 */
static wiced_bool_t handle_dev_cback( wiced_bt_gatt_data_t * gatt_data_p)
{
    uint8_t * msg = gatt_data_p->p_data;
    uint16_t  len = gatt_data_p->len;

    IFXV_AUDIO_TRACE("handle_dev_cback, msg=%d, len=%d\n", msg[0], len);
    // The first byte of message is message type
    switch (msg[0]) {
    case IFX_AUDIO_CAP_RESP:
        handle_cap_resp((ifxv_cap_msg_t *) msg, len);
        break;

    case IFX_AUDIO_RESP:
        handle_resp((ifxv_rsp_msg_t *) msg, len);
        break;

    case IFX_AUDIO_START_REQ:
        WICED_BT_TRACE("Start audio request from device\n");
        handle_start_req((ifxv_start_msg_t *) msg, len);
        break;

    case IFX_AUDIO_STOP_REQ:
        WICED_BT_TRACE("Stop request from device, reason %d\n", ((ifxv_stop_req_msg_t *)msg)->reason);
        handle_stop_request((ifxv_stop_req_msg_t *) msg, len);
        break;

    default:
        WICED_BT_TRACE("Not handled msg %02x\n", *gatt_data_p->p_data);
        break;
    }
    return TRUE;
}

/******************************************************
 *               ATV service database
 ******************************************************/
static char_t char_data[SUPPORTED_CHARACTERISTIC_CNT] =
{
    {{LEN_UUID_128, {.uuid128=IFXV_DATA_UUID}}, .name="IFXV Data",     .index=UNDEFINED_CHAR_INDEX, .cb=handle_data_cback},
    {{LEN_UUID_128, {.uuid128=IFXV_HOST_UUID}}, .name="IFXV Host Cmd", .index=UNDEFINED_CHAR_INDEX, .cb=handle_host_cback},
    {{LEN_UUID_128, {.uuid128=IFXV_DEV_UUID}},  .name="IFXV Dev Msg",  .index=UNDEFINED_CHAR_INDEX, .cb=handle_dev_cback}
};

service_t audio_ifxv =
{
    .id =
    {{
        .len = LEN_UUID_16,
        .uu.uuid16 = IFXV_UUID,
    }},
    .name = "IFX Voice",
    .char_cnt = SUPPORTED_CHARACTERISTIC_CNT,
    .char_ptr = char_data,
};


/******************************************************
 *   Public functions
 ******************************************************/

/*
 * This only gets called from HCI
 */
void ifxv_audio_start_req(audio_cfg_t * cfg)
{
    if (ifxv.state >= IFXV_STATE_IDLE)
    {
        ifxv_start_msg_t msg;
        memcpy(&msg.cfg, cfg, sizeof(audio_cfg_t));
        handle_start_req(&msg, sizeof(ifxv_start_msg_t));
    }
}

/*
 * This only gets called from HCI
 * We translate the stop reason to IFXV reason.
 */
void ifxv_audio_stop_req(audio_stop_reason_e reason)
{
    if (ifxv.state >= IFXV_STATE_IDLE)
    {
        ifxv_stop_req_msg_t msg;

        switch (ifxv.state) {
        case IFXV_STATE_STARTED:
        case IFXV_STATE_ACTIVE:
        case IFXV_STATE_STOPPED:
            switch (reason) {
            case IFX_AUDIO_STOP_REASON_TIMEOUT:
                msg.reason = IFX_REASON_TIMEOUT;
                break;
            case IFX_AUDIO_STOP_REASON_USER_REQUEST:
                msg.reason = IFX_REASON_USER_REQUEST;
                break;
            default:
                msg.reason = IFX_REASON_AUDIO_NOT_ACTIVE;
                break;
            }
            handle_stop_request(&msg, sizeof(ifxv_stop_req_msg_t));
            break;
        }
    }
}

void ifxv_audio_send_capabilities()
{
    if (service_found(&audio_ifxv) && service_all_char_found(&audio_ifxv))
    {
        IFXV_AUDIO_TRACE("Found IFXV Service: Send host audio capabilities\n");
        send_msg(&host_cap_msg, sizeof(host_cap_msg));
        ifxv.server.cap.resp_time_limit_in_sec = RESPONSE_TIMEOUT_DEFAULT;
        start_timer(IFXV_STATE_SENT_CAP);
        return;
    }
    IFXV_AUDIO_TRACE2("IFXV server NOT found\n");
}

/*
 *
 */
void ifxv_audio_down()
{
    service_down(&audio_ifxv);
    stop_timer(IFXV_STATE_SERVER_NOT_AVAILABLE);
}

/*
 *
 */
void ifxv_audio_init()
{
    wiced_init_timer (&ifxv.duration_timer, &ifxv_audio_timeout, 0, WICED_SECONDS_TIMER );
    ifxv_msbc_init();
}

/*
 *
 */
server_info_t * ifxv_audio_device_cap()
{
    return ifxv_is_connected() ? &ifxv.server : NULL;
}

/*
 *
 */
server_info_t * ifxv_audio_host_cap()
{
    return &host_cap_msg.info;
}
