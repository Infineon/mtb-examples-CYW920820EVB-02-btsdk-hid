/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 * Remote control
 *
 * This file provides definitions and function prototypes for remote control
 * device
 *
 */

#ifndef __REMOTE_H__
#define __REMOTE_H__
#include "wiced.h"
#include "wiced_hidd_lib.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "hidd_lib.h"

#if 0
 #define STATIC
#else
 #define STATIC static
#endif

/********************************************************************************
* Types and Defines
*******************************************************************************/
#if defined(BLE_SUPPORT) && defined(BR_EDR_SUPPORT)
 #define LOCAL_NAME "CY DUAL KEYBOARD"
 #define BLE_LOCAL_NAME "CY BLE KEYBOARD"
 #define BT_LOCAL_NAME "CY BT KEYBOARD"
#elif defined(BLE_SUPPORT)
 #define LOCAL_NAME "CY BLE KEYBOARD"
 #define BLE_LOCAL_NAME LOCAL_NAME
#else
 #define LOCAL_NAME "CY BT KEYBOARD"
 #define BT_LOCAL_NAME LOCAL_NAME
#endif

/********************************************************************************
* Types and Defines
*******************************************************************************/
#ifdef KEYBOARD_PLATFORM
 #define NUM_KEYSCAN_ROWS    8    // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    18   // Num of Cols in keyscan matrix

 #define LED_WHITE           WICED_PLATFORM_LED_1
 #define LED_BLUE            WICED_PLATFORM_LED_2
 #define LED_GREEN           WICED_PLATFORM_LED_3
 #define LED_RED             WICED_PLATFORM_LED_4

 #define LED_ERROR           LED_RED
 #define LED_LE_LINK         LED_BLUE
 #define LED_BREDR_LINK      LED_GREEN
 #define LED_CAPS            LED_WHITE

 #define CONNECT_KEY_INDEX   94
#else
 #define NUM_KEYSCAN_ROWS    7    // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    8    // Num of Cols in keyscan matrix

 #define LED_GREEN           WICED_PLATFORM_LED_1
 #define LED_RED             WICED_PLATFORM_LED_2

 #define LED_ERROR           LED_RED
 #define LED_LE_LINK         LED_GREEN
 #define LED_BREDR_LINK      LED_GREEN
 #define LED_CAPS            LED_RED

 #define CONNECT_KEY_INDEX   8
#endif

typedef void (app_poll_callback_t)(void);

/********************************************************************************
 * Report ID defines
 ********************************************************************************/
// Input report id
typedef enum {
    RPT_ID_IN_STD_KEY    =0x01,
    RPT_ID_IN_BIT_MAPPED =0x02,
    RPT_ID_IN_BATTERY    =0x03,
    RPT_ID_IN_SLEEP      =0x04,
    RPT_ID_IN_FUNC_LOCK  =0x05,
    RPT_ID_IN_SCROLL     =0x06,
    RPT_ID_IN_PIN        =0x07,
    RPT_ID_IN_CNT_CTL    =0xcc,
    RPT_ID_NOT_USED      =0xff,
} rpt_id_in_e;

// Output report id
typedef enum {
    RPT_ID_OUT_KB_LED   =0x01,
} rpt_id_out_e;

// Feature report id
typedef enum {
    RPT_ID_FEATURE_CNT_CTL   =0xcc,
} rpt_id_feature_e;

// BIT mapped defines
enum
{
    BIT_MAPPED_POWER,           // 0    USAGE_POWER
    BIT_MAPPED_LOCK,            // 1    USAGE_LOCK_SCRSVR
    BIT_MAPPED_LIGHT,           // 2    USAGE_LIGHT_ENABLE
    BIT_MAPPED_RGB,             // 3    USAGE_RESERVED0
    BIT_MAPPED_WORLD,           // 4    USAGE_INTERNET
    BIT_MAPPED_EDIT,            // 5    USAGE_AC_EDIT
    BIT_MAPPED_COPY,            // 6    USAGE_AC_COPY
    BIT_MAPPED_SEARCH,          // 7    USAGE_AC_SEARCH

    BIT_MAPPED_BACKLIGHT,       // 8    USAGE_RESERVED1
    BIT_MAPPED_LIGHT_UP,        // 9    USAGE_RESERVED2
    BIT_MAPPED_LIGHT_DOWN,      // 10   USAGE_RESERVED3
    BIT_MAPPED_REWIND,          // 11   USAGE_REWIND
    BIT_MAPPED_FAST_FORWARD,    // 12   USAGE_FAST_FORWRD
    BIT_MAPPED_FUNCTION,        // 13   USAGE_FUNCTION
    BIT_MAPPED_PLAY_PAUSE,      // 14   USAGE_PLAY_PAUSE
    BIT_MAPPED_MAX
};

/********************************************************************************
 * App queue defines
 ********************************************************************************/
typedef union {
    uint8_t                     type;
    HidEvent                    info;
    HidEventMotionXY            mouse;
    HidEventButtonStateChange   button;
    HidEventKey                 key;
    HidEventAny                 any;
    HidEventUserDefine          user;
} app_queue_t;

#define APP_QUEUE_SIZE sizeof(app_queue_t)
#define APP_QUEUE_MAX  44                         // max number of event in queue

/********************************************************************************
 * Include all components
 *******************************************************************************/
#include "battery/battery.h"
#include "ota/ota.h"
#include "bt/bt.h"
#include "key/key.h"
#include "key/key_entry.h"

typedef struct {
    wiced_hidd_app_event_queue_t eventQueue;
    app_queue_t events[APP_QUEUE_MAX];
    uint8_t pollSeqn;
    uint8_t recoveryInProgress;
    uint8_t protocol;
    uint8_t setReport_status;
    uint8_t connection_ctrl_rpt;
    uint8_t  idleRate;                           // Save the idle rate in units of 4 ms
    uint32_t idleRateInBtClocks;                 // Convert to BT clocks for later use. Formula is ((Rate in 4 ms)*192)/15

    uint8_t transportStateChangeNotification:1;
    uint8_t pollStarted:1;

} app_t;

extern app_t app;
/********************************************************************************
 * Function Name: app_setReport
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  reportId -- report id
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);

/********************************************************************************
 * Function Name: app_queueEvent
 ********************************************************************************
 * Summary:
 *  Queue an event to event queue
 *
 * Parameters:
 *  event -- event to queue
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_queueEvent(app_queue_t * event);

/********************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *  This function is called when the state of a link is changed.
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint8_t transport, uint8_t newState);

/********************************************************************************
 * Function Name: app_setReport
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  reportId -- report id
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize);

/********************************************************************************
 * Function Name: app_start
 ********************************************************************************
 * Summary: This is application start function. After system initialization is done, when the
 *          bt management calls with BTM_ENABLED_EVT, this function is called to
 *          start application
 *
 * Parameters:
 *  none
 *
 * Return:
 *  WICED_BT_SUCCESS -- if application initialization is okay and ready to start;
 *                      otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start();

#endif // __BLEREMOTE_H__
