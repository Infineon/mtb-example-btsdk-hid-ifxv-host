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
 * IFXV operation.
 *
 */

#include "app.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_gatt.h"

#if OP_TRACE==1
 #define IFXV_OP_TRACE           WICED_BT_TRACE
 #define IFXV_OP_TRACE2(...)
#elif OP_TRACE==2
 #define IFXV_OP_TRACE           WICED_BT_TRACE
 #define IFXV_OP_TRACE2          WICED_BT_TRACE
#else
 #define IFXV_OP_TRACE(...)
 #define IFXV_OP_TRACE2(...)
#endif

#define TX_TIMER_PERIOD_IN_MS   50
#define OP_QUEUE_SIZE           10
#define OP_MAX_WR_DATA          20

#define op_queue_empty()        (op.head == op.tail)
#define op_queue_full()         (op_queue_cnt() >= (OP_QUEUE_SIZE-1))

typedef struct
{
    uint16_t handle;
    uint8_t  op;        // defined by ifxv_op_e
    uint8_t  len;
    union {
        uint16_t u16;
        uint8_t  d[OP_MAX_WR_DATA];
    } data;
} ifxv_op_element_t;

typedef struct
{
    ifxv_op_element_t elm[OP_QUEUE_SIZE];
    uint8_t           head, tail;
    wiced_timer_t     tx_timer;
} ifxv_op_t;

static ifxv_op_t op = {0};

/*************************************
 * Functions
 ************************************/

/*
 * returns TRUE when success
 */
static wiced_bool_t op_queue_cnt()
{
    return op.head >= op.tail ? op.head - op.tail : OP_QUEUE_SIZE + op.head - op.tail;
}

/*
 * ifxv_operation -- Do GATT operation. Returns TRUE when success
 */
static wiced_bool_t ifxv_operation()
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_ERROR;
    ifxv_op_element_t * ptr = &op.elm[op.tail];

    switch (ptr->op) {
    case IFXV_OP_WRITE:
        status = app_gatt_send_write( ifxv_conn_id(), ptr->handle, ptr->data.d, ptr->len, GATT_WRITE_NO_RSP );
        break;
    case IFXV_OP_READ:
        status = app_gatt_send_read_by_handle(ifxv_conn_id(), ptr->handle);
        break;
    case IFXV_OP_SET_CONFIG:
        status = app_gatt_set_config_descriptor(ifxv_conn_id(), ptr->handle, ptr->data.u16);
        break;
    }

#if TRACE_IFXV_OP_ENABLE != 0
 #if TRACE_IFXV_OP_ENABLE==1
    if (status == WICED_BT_GATT_SUCCESS)
 #endif
    {
        switch (ptr->op) {
        case IFXV_OP_WRITE:
            IFXV_OP_TRACE("Called app_gatt_send_write, handle:%04x, len:%d, qCnt:%d, qIdx:%d --- %s\n", ptr->handle, ptr->len, op_queue_cnt(), op.tail, status ? "failed": "success");
            break;
        case IFXV_OP_READ:
            IFXV_OP_TRACE("Called app_gatt_send_read_by_handle, handle:%04x, qCnt:%d, qIdx:%d --- %s\n", ptr->handle, op_queue_cnt(), op.tail, status ? "failed": "success");
            break;
        case IFXV_OP_SET_CONFIG:
            IFXV_OP_TRACE("Called app_gatt_set_config_descriptor, handle:%04x, data:%04x, qCnt:%d, qIdx:%d -- %s\n", ptr->handle, ptr->data.u16, op_queue_cnt(), op.tail, status ? "failed": "success");
            break;
        }
    }
#endif
    return status;
}

/*
 * tx_timer_handler - Periodic timer handler. When queue is not empty, the timer should be running to call this function to empty the queue.
 *
 *      1. Do operation. If the operation succseeds, remove the item from queue.
 *      3. If the queue is empty, stop the timer.
 */
static void tx_timer_handler( TIMER_PARAM_TYPE arg )
{
    IFXV_OP_TRACE2("OP Timer Callback tail:%d, head:%d, queue cnt:%d empty:%d, full:%d\n", op.tail, op.head, op_queue_cnt(), op_queue_empty(), op_queue_full());

    // 1. Do the operation. If the operation succseeds, remove the item from queue.
    if (ifxv_operation() == WICED_BT_GATT_SUCCESS)
    {
        // remove the item from queue.
        if (++op.tail == OP_QUEUE_SIZE)
        {
            op.tail = 0;
        }
        IFXV_OP_TRACE2("OP Success, after removing data, cnt:%d empty:%d, full:%d\n", op_queue_cnt(), op_queue_empty(), op_queue_full());
    }

    // 2. stop the timer if no more item in queue
    if (op_queue_empty() && wiced_is_timer_in_use(&op.tx_timer))
    {
        IFXV_OP_TRACE("OP timer stopped\n");
        wiced_stop_timer(&op.tx_timer);
    }
}

/*
 ******** Publicc Functions ********************************************************************
 */

/*
 * ifxv_op - submit an operation.
 *      1. Add the operation to queue first.
 *      2. If the timer was not running (queue was empty before submit), perform the operation immeidately (otherwise, wait for timer to expire)
 *      3. After the operation, if the queue still have pending operation and timer is not running, start timer.
 */
wiced_bt_gatt_status_t ifxv_op(uint8_t opr, uint16_t handle, uint16_t data16, void * p_data, uint8_t len)
{
    ifxv_op_element_t * ptr = &op.elm[op.head];
    IFXV_OP_TRACE("IFXV_OP, BEFORE SUBMIT OP:%d, handle:%04x: tail:%d, head:%d, Queue cnt:%d, empty:%d, full:%d\n",
                   opr, handle, op.tail, op.head, op_queue_cnt(), op_queue_empty(), op_queue_full());

    // 1. Submit the data queue. First, check if there if queue is full.
    if (op_queue_full())
    {
        WICED_BT_TRACE("OP queue FULL!!\n", TX_TIMER_PERIOD_IN_MS);
        return WICED_BT_GATT_PREPARE_Q_FULL; // no more room
    }

    ptr->op      = opr;
    ptr->handle  = handle;

    if (opr == IFXV_OP_WRITE)
    {
        if (len > OP_MAX_WR_DATA)
        {
            WICED_BT_TRACE("Error!! OP write data length > %d\n", OP_MAX_WR_DATA);
            return WICED_BT_GATT_ERROR;
        }
        ptr->len = len;
        memcpy(&ptr->data, p_data, len);
    }
    else
    {
        ptr->data.u16 = data16;
    }

    if (++op.head == OP_QUEUE_SIZE)
    {
        op.head = 0;
    }

    // 2. If timer is not runnig, perform the operation immediately
    if (!wiced_is_timer_in_use(&op.tx_timer))
    {
        tx_timer_handler(0);

        // 3. After the operation, start the timer if necessary.
        if (!op_queue_empty())
        {
            IFXV_OP_TRACE("OP periodic timer started for %d ms, qCnt:%d\n", TX_TIMER_PERIOD_IN_MS, op_queue_cnt());
            wiced_start_timer(&op.tx_timer, TX_TIMER_PERIOD_IN_MS);
        }
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 * ifxv_op_init - need to call function at startup to initialize timer.
 */
void ifxv_op_init()
{
    wiced_init_timer(&op.tx_timer, tx_timer_handler, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
}
