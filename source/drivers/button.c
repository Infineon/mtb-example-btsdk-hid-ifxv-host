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
 * button functions
 *
 */

#include "app.h"
#include "cycfg_pins.h"
#include "wiced_bt_trace.h"

#define BUTTON_TIMER_TICK               1
#define BUTTON_HOLDING_TIME_FOR_ERASE   5
#define USER_BUTTON                     WICED_PLATFORM_BUTTON_4

static wiced_timer_t button_timer;
static int button_timer_cnt = 0;

static void button_timer_handler( TIMER_PARAM_TYPE arg )
{
    if (++button_timer_cnt >= BUTTON_HOLDING_TIME_FOR_ERASE)
    {
        wiced_stop_timer(&button_timer);
        if (ifxv_conn_id())
        {
            wiced_bt_gatt_disconnect(ifxv_conn_id());
        }
        WICED_BT_TRACE("Erase bonding\n");
        app_erase_bonding();
        app_enter_pairing();
    }
    else
    {
        WICED_BT_TRACE("%d\n", button_timer_cnt);
    }
}

static void button_interrupt_handler( void *user_data, uint8_t value )
{
    int down = button_down();

    if (down)
    {
        if (ifxv_conn_id())
        {
            wiced_stop_timer(&button_timer);
            WICED_BT_TRACE("User button start audio\n");
            audio_start_req(audio_get_cfg());
        }
        else
        {
            WICED_BT_TRACE( "Hold Button 5 sec to erase pairing\n" );
            button_timer_cnt = 0;
            wiced_start_timer(&button_timer,BUTTON_TIMER_TICK);
        }
    }
    else
    {
        wiced_stop_timer(&button_timer);
        if (ifxv_conn_id())
        {
            WICED_BT_TRACE("Stop audio\n");
            audio_stop_req(IFX_AUDIO_STOP_REASON_USER_REQUEST);
        }
        else
        {
            WICED_BT_TRACE( "Button released\n" );
        }
    }
}

wiced_bool_t button_down()
{
    return wiced_hal_gpio_get_pin_input_status( WICED_GET_PIN_FOR_BUTTON(USER_BUTTON) ) == wiced_platform_get_button_pressed_value(USER_BUTTON);
}

void button_init()
{
    wiced_platform_register_button_callback( USER_BUTTON, button_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
    wiced_init_timer(&button_timer, button_timer_handler, 0, WICED_SECONDS_PERIODIC_TIMER);
}

