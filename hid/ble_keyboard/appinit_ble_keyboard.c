/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * Entry point to LE keyboard application.
 *
 */
#include "ble_keyboard.h"
#include "wiced_bt_trace.h"
#include "hidd_lib.h"

#ifdef TESTING_USING_HCI
static hci_rpt_db_t hci_rpt_db[] =
{
   // rpt_buf,             rpt_type,                    rpt_id,              length (exclude rpt_id)
   {blekb_key_std_rpt,     WICED_HID_REPORT_TYPE_INPUT, STD_KB_REPORT_ID,    KEYRPT_MAX_KEYS_IN_STD_REPORT+2},
   {blekb_bitmap_rpt,      WICED_HID_REPORT_TYPE_INPUT, BITMAPPED_REPORT_ID, KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT},
   {&blekb_func_lock_rpt,  WICED_HID_REPORT_TYPE_INPUT, FUNC_LOCK_REPORT_ID, 1},
};
#define HCI_CONTROL_RPT_CNT (sizeof(hci_rpt_db)/sizeof(hci_rpt_db_t))
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void app_LED_init(void)
{
    kb_LED_init(LED_CAPS, LED_OFF_LEVEL);
    kb_LED_init(LED_BLUE, LED_OFF_LEVEL);
#ifdef KEYBOARD_PLATFORM
    kb_LED_init(LED_GREEN, LED_OFF_LEVEL);
    kb_LED_init(LED_RED, LED_OFF_LEVEL);
#endif
}
/******************************************************************************
 *                          Function Definitions
******************************************************************************/

wiced_result_t blehid_app_init(void)
{
    wiced_result_t result;
    /*  GATT DB Initialization  */
    if ( (result=wiced_hidd_gatts_init( blehid_db_data, blehid_db_size, blehid_gattAttributes, blehid_gattAttributes_size, NULL, NULL )) != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("\nwiced_hidd_gatts_init failed, result=%d",result);
        wiced_hidd_led_blink_error(KB_LED_ERROR, LED_ERROR_CODE_GATTS);
        return WICED_BT_ERROR;
    }

    /* general hid app init */
    wiced_hidd_app_init(BT_DEVICE_TYPE_BLE);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    //start blekb app
    blekbapp_create();

    return WICED_BT_SUCCESS;
}

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    //restore content from AON memory when wake up from shutdown sleep (SDS)
    kbapp_aon_restore();

    wiced_hidd_start(blehid_app_init, NULL, &wiced_bt_hid_cfg_settings, wiced_bt_hid_cfg_buf_pools);
    hci_control_init(HCI_CONTROL_RPT_CNT, hci_rpt_db);

    WICED_BT_TRACE("\nSLEEP_ALLOWED=%d",SLEEP_ALLOWED);
    WICED_BT_TRACE("\nLED SUPPORT=%d", LED_SUPPORT);

#ifdef OTA_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_FW_UPGRADE");
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_SEC_FW_UPGRADE");
 #endif
#endif

#ifdef ASSYM_SLAVE_LATENCY
    WICED_BT_TRACE("\nASSYMETRIC_SLAVE_LATENCY");
#endif

#ifdef LE_LOCAL_PRIVACY_SUPPORT
    WICED_BT_TRACE("\nLE_LOCAL_PRIVACY");
#endif

#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    WICED_BT_TRACE("\nSKIP_PARAM_UPDATE");
#endif

#ifdef AUTO_RECONNECT
    WICED_BT_TRACE("\nAUTO_RECONNECT");
#endif

    wiced_hidd_led_blink(LED_BLUE, 5, 100);  // fast 5 blinks to indicate firmware is up and running
}
