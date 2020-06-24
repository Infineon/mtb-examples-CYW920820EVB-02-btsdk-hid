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
 * Entry point to LE remote control application.
 *
 */
#include "wiced_bt_trace.h"
#include "wiced_hidd_lib.h"
#include "ble_remote.h"
#include "android_voice.h"
#include "hidd_lib.h"

extern void sfi_allow_deep_sleep(void);

#ifdef TESTING_USING_HCI
static hci_rpt_db_t hci_rpt_db[] =
{
   // rpt_buf,             rpt_type,                    rpt_id,              length (exclude rpt_id)
   {bleremote_key_std_rpt, WICED_HID_REPORT_TYPE_INPUT, STD_KB_REPORT_ID,    KEYRPT_LEN},
   {bleremote_bitmap_rpt,  WICED_HID_REPORT_TYPE_INPUT, BITMAPPED_REPORT_ID, KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT},
};
#define HCI_CONTROL_RPT_CNT (sizeof(hci_rpt_db)/sizeof(hci_rpt_db_t))
#endif

#ifdef ANDROID_AUDIO
wiced_bt_gatt_status_t blehid_app_gatts_req_read_callback( uint16_t conn_id, wiced_bt_gatt_read_t * p_data )
{
    // Check if the write request is for the Android TV Voice service
    if ((p_data->handle >= HANDLE_ATV_VOICE_SERVICE) && (p_data->handle <= HANDLE_ATV_VOICE_SERVICE_END))
    {
        WICED_BT_TRACE("\nAndroid gatts read, handle:%04x offset:%d len=%d, ", p_data->handle, p_data->offset, p_data->p_val_len);
        return android_voice_read_handler(conn_id, p_data);
    }
    return WICED_BT_GATT_NOT_FOUND;
}

wiced_bt_gatt_status_t blehid_app_gatts_req_write_callback( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    // Check if the write request is for the Android TV Voice service
    if ((p_data->handle >= HANDLE_ATV_VOICE_SERVICE) && (p_data->handle <= HANDLE_ATV_VOICE_SERVICE_END))
    {
#if 0
        WICED_BT_TRACE("\nAndroid gatts write, handle:%04x offset:%d len=%d", p_data->handle, p_data->offset, p_data->val_len);
        if (p_data->val_len)
        {
            STRACE_ARRAY(", ",p_data->p_val + p_data->offset, p_data->val_len);
        }
#endif
        return android_voice_write_handler(conn_id, p_data);
    }
    return WICED_BT_GATT_NOT_FOUND;
}
#else
 #define blehid_app_gatts_req_read_callback     NULL
 #define blehid_app_gatts_req_write_callback    NULL
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void app_LED_init(void)
{
    wiced_hidd_led_init(LED_RED, LED_OFF);
    wiced_hidd_led_init(LED_BLUE, LED_OFF);
}

/******************************************************************************
 *                          Function Definitions
******************************************************************************/
wiced_result_t app_init(void)
{
    /*  GATT DB Initialization  */
    if ( wiced_hidd_gatts_init( blehid_db_data,
                                blehid_db_size,
                                blehid_gattAttributes,
                                blehid_gattAttributes_size,
                                blehid_app_gatts_req_read_callback,
                                blehid_app_gatts_req_write_callback ) != WICED_BT_SUCCESS )
    {
        return WICED_BT_ERROR;
    }

    /* general hid app init */
    wiced_hidd_app_init(BT_DEVICE_TYPE_BLE);

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    //start bleremote app
    bleremoteapp_create();

    return WICED_BT_SUCCESS;
}

#ifdef ANDROID_AUDIO
/*
 * bleremote ble link management callbacks
 */
wiced_result_t bleremote_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_RESUME_HIDD_LIB_HANDLER;

    switch( event )
    {
        case BTM_ENCRYPTION_STATUS_EVT:
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                //configure ATT MTU size with peer device
                wiced_bt_gatt_configure_mtu(ble_hidd_link.gatts_conn_id, wiced_bt_hid_cfg_settings.gatt_cfg.max_mtu_size);
            }
            // let default library to handle the reset
            break;
    }
    return result;
}
#else
 #define bleremote_management_cback NULL
#endif

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    sfi_allow_deep_sleep();
    app_LED_init();

    //restore content from AON memory
    bleremoteapp_aon_restore();

    wiced_hidd_start(app_init, bleremote_management_cback, &wiced_bt_hid_cfg_settings, wiced_bt_hid_cfg_buf_pools);
    hci_control_init(HCI_CONTROL_RPT_CNT, hci_rpt_db);

    WICED_BT_TRACE("\nDEV=%d",wiced_hidd_chip());
    WICED_BT_TRACE("\nSLEEP_ALLOWED=%d",SLEEP_ALLOWED);

#ifdef SUPPORT_SCROLL
    WICED_BT_TRACE("\nENABLE_SCROLL");
#endif

#ifdef ASSYM_SLAVE_LATENCY
    WICED_BT_TRACE("\nASSYMETRIC_SLAVE_LATENCY");
#endif

#ifdef SUPPORT_AUDIO
 #ifdef CELT_ENCODER
    WICED_BT_TRACE("\nENABLE_AUDIO(CELT)");
 #elif defined(ADPCM_ENCODER)
    WICED_BT_TRACE("\nENABLE_AUDIO(ADPCM)");
 #else
    WICED_BT_TRACE("\nENABLE_AUDIO(mSBC)");
 #endif
 #ifdef ANDROID_AUDIO
    WICED_BT_TRACE("--ANDROID");
 #endif
#endif

#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    WICED_BT_TRACE("\nSKIP_PARAM_UPDATE");
#endif

#ifdef AUTO_RECONNECT
    WICED_BT_TRACE("\nAUTO_RECONNECT");
#endif

#ifdef EASY_PAIR
    WICED_BT_TRACE("\nENABLE_EASY_PAIR");
#endif

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
    WICED_BT_TRACE("\nSTART_ADV_ON_POWERUP");
#endif

#ifdef CONNECTED_ADVERTISING_SUPPORTED
    WICED_BT_TRACE("\nENABLE_CONNECTED_ADV");
#endif

#ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
    WICED_BT_TRACE("\nDISCONNECTED_ENDLESS_ADV");
#endif

#ifdef SUPPORTING_FINDME
    WICED_BT_TRACE("\nENABLE_FINDME");
#endif

#ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("\nENABLE_DIGITAL_MIC");
#endif

#ifdef OTA_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_FW_UPGRADE");
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_SEC_FW_UPGRADE");
 #endif
#endif

    wiced_hidd_led_blink(LED_BLUE, 5, 100);  // fast 5 blinks to indicate firmware is up and running
}
