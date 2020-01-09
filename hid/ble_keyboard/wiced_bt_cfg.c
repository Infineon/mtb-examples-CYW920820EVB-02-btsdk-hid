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
 * Runtime Bluetooth stack and keyboard configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "ble_keyboard.h"
#include "ble_keyboard_gatts.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
#endif

const char  dev_local_name[]             = "CY BLE KB";
const uint8_t dev_appearance_name[2]     = {BIT16_TO_8(APPEARANCE_HID_KEYBOARD)};
const uint8_t dev_pnp_id[]               ={0x01, 0x31, 0x01, 0xB4, 0x04, 0x01, 0x00}; //BT SIG, cypress semiconductor, 0x04B4, 0x0001
const char dev_char_mfr_name_value[]     = "Cypress Semiconductor";

const uint8_t   rpt_ref_battery[]             = {BATTERY_REPORT_ID ,0x01};
const uint8_t   rpt_ref_std_key_input[]       = {STD_KB_REPORT_ID,0x01};
const uint8_t   rpt_ref_std_key_output[]      = {STD_KB_REPORT_ID,0x02};
const uint8_t   rpt_ref_bitmap[]              = {BITMAPPED_REPORT_ID,0x01};
const uint8_t   rpt_ref_sleep[]               = {SLEEP_REPORT_ID,0x01};
const uint8_t   rpt_ref_func_lock[]           = {FUNC_LOCK_REPORT_ID,0x01};
const uint8_t   rpt_ref_scroll[]              = {SCROLL_REPORT_ID, 0x01};
const uint8_t   rpt_ref_connection_ctrl[]     = {0xCC,0x03}; //feature rpt

const uint8_t dev_hid_information[] = {0x00, 0x01, 0x00, 0x00};      // Verison 1.00, Not localized, Cannot remote wake, not normally connectable
const uint16_t dev_battery_service_uuid = UUID_CHARACTERISTIC_BATTERY_LEVEL;
#define INCLUDE_GATT_SERVICE_CHANGED 1
#if INCLUDE_GATT_SERVICE_CHANGED
uint8_t hid_service_changed[]            = {0x00,0x00,0x00,0x00};
#endif

extern uint16_t characteristic_client_configuration[];
extern uint8_t kbapp_protocol;
extern uint8_t battery_level;

extern uint8_t blekb_kb_output_rpt;
extern uint8_t blekb_sleep_rpt;
extern uint8_t blekb_scroll_rpt;
extern uint8_t blekb_func_lock_rpt;
extern uint8_t blekb_connection_ctrl_rpt;

extern uint8_t blekb_key_std_rpt[];       //map to (&(kbAppState->stdRpt.modifierKeys))[kbAppState->stdRptSize]
extern uint8_t blekb_bitmap_rpt[];  //map to kbAppState->bitMappedReport.bitMappedKeys[]


/*****************************************************************************
 * This is the GATT database for the BLE HID KB application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if application is allowed to read or write
 * into it.  Handles do not need to be sequential, but need to be in order.
 ****************************************************************************/
const uint8_t blehid_db_data[]=
{
    // Declare gatt service
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

#if INCLUDE_GATT_SERVICE_CHANGED
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_GATT_SERVICE_CHANGED,
        HANDLE_BLEKB_GATT_SERVICE_CHANGED_VAL,
        GATT_UUID_GATT_SRV_CHGD,
        LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_NONE
    ),
#endif

    // Declare GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        GATT_UUID_GAP_DEVICE_NAME,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        GATT_UUID_GAP_ICON,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare GAP service characteristic: Peripheral Prefered Connection Parameter
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM,
        HANDLE_BLEKB_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        GATT_UUID_GAP_PREF_CONN_PARAM,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

    // Handle 0x29: characteristic PnP ID, handle 0x2A characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_PNP_ID,
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        GATT_UUID_PNP_ID,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Handle 0x2B: characteristic Manufacturer Name, handle 0x2C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_MFR_NAME,
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        GATT_UUID_MANU_NAME,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),

    // Declare Battery service
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY),

    // Handle 0x31: characteristic Battery Level, handle 0x32 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_BATTERY_SERVICE_CHAR_LEVEL,       // attribute handle
        HANDLE_BLEKB_BATTERY_SERVICE_CHAR_LEVEL_VAL, // attribute value handle
        GATT_UUID_BATTERY_LEVEL,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_BATTERY_SERVICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x34: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_BATTERY_SERVICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // Declare Scan Parameters service
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_SCAN_PARAM_SERVICE, UUID_SERVCLASS_SCAN_PARAM),

    // Handle 0x41: characteristic Battery Level, handle 0x42 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW,
        HANDLE_BLEKB_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL,
        GATT_UUID_SCAN_INT_WINDOW,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ
    ),

    // Declare HID over LE
    PRIMARY_SERVICE_UUID16
    ( HANDLE_BLEKB_LE_HID_SERVICE, UUID_SERVCLASS_LE_HID),


    // Include BSA SERVICE
    INCLUDE_SERVICE_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_INC_BAS_SERVICE,
        HANDLE_BLEKB_BATTERY_SERVICE,
        HANDLE_BLEKB_BATTERY_SERVICE_RPT_REF_DESCR,
        UUID_SERVCLASS_BATTERY
    ),

    // Handle 0x51: characteristic Protocol Mode, handle 0x52 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_PROTO_MODE,
        HANDLE_BLEKB_LE_HID_SERVICE_PROTO_MODE_VAL,
        GATT_UUID_HID_PROTO_MODE,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD
    ),

    // Handle 0x53: characteristic HID information, handle 0x54 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_INFO,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_INFO_VAL,
        GATT_UUID_HID_INFORMATION,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


    // Handle 0x55: characteristic Boot Keyboard Input report, handle 0x56 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_VAL,
        GATT_UUID_HID_BT_KB_INPUT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),


    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),


    // Handle 0x58: characteristic Boot Keyboard Output report, handle 0x59 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_OUTPUT,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_OUTPUT_VAL,
        GATT_UUID_HID_BT_KB_OUTPUT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE|LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x5A: characteristic HID Report MAP, handle 0x5B characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_MAP,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        GATT_UUID_HID_REPORT_MAP,
        LEGATTDB_CHAR_PROP_READ,
        LEGATTDB_PERM_READABLE
    ),


    // include Battery Service
    // Handle 0x5C: external report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        GATT_UUID_EXT_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


    // STD Input report
    // Handle 0x5D: characteristic HID Report, handle 0x5E characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x60: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // STD Output report
    // Handle 0x61: characteristic HID Report, handle 0x62 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE|LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x63: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // Bit mapped report, Report ID=2
    // Handle 0x64: characteristic HID Report, handle 0x65 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x67: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // sleep report
    // Handle 0x68: characteristic HID Report, handle 0x69 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x6B: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // Func lock report
    // Handle 0x6C: characteristic HID Report, handle 0x6D characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x6F: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    //Scroll report
    // Handle 0x70: characteristic HID Report, handle 0x71 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_NOTIFY,
        LEGATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_CMD|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x73: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),


    // Connection control feature
    // Handle 0x74: characteristic HID Report, handle 0x75 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        GATT_UUID_HID_REPORT,
        LEGATTDB_CHAR_PROP_READ|LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_READABLE|LEGATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x76: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        LEGATTDB_PERM_READABLE
    ),

    // HID control point
    // Handle 0x77: characteristic HID Report, handle 0x78 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        GATT_UUID_HID_CONTROL_POINT,
        LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        LEGATTDB_PERM_WRITE_CMD
    ),

#ifdef OTA_FIRMWARE_UPGRADE
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
    // Handle 0xff00: Cypress vendor specific WICED Secure OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
    ( HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE ),
#else
    // Handle 0xff00: Cypress vendor specific WICED OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
    ( HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE ),
#endif

    // Handles 0xff03: characteristic WS Control Point, handle 0xff04 characteristic value.
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
        LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ  | LEGATTDB_PERM_AUTH_WRITABLE
    ),

    // Declare client characteristic configuration descriptor
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
    // for bonded devices.  Setting value to 1 tells this application to send notification
    // when value of the characteristic changes.  Value 2 is to allow indications.
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE
    ),

    // Handle 0xff07: characteristic WS Data, handle 0xff08 characteristic value. This
    // characteristic is used to send next portion of the FW Similar to the control point
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        HANDLE_OTA_FW_UPGRADE_DATA,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        LEGATTDB_CHAR_PROP_WRITE,
        LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ | LEGATTDB_PERM_AUTH_WRITABLE
    ),
#endif
};
const uint16_t blehid_db_size = sizeof(blehid_db_data);

/*****************************************************************************
 * This is the report map for HID Service
  ****************************************************************************/
const uint8_t blehid_rpt_map[] =
{
            // STD_KB_REPORT_ID
            // Input Report, 8 bytes
            // 1st byte:Keyboard LeftControl/Keyboard Right GUI
            // 2nd byte:Constant, 3rd ~ 6th: keycode
            // Output Report, 1 byte: LED control
            0x05 , 0x01,                    // USAGE_PAGE (Generic Desktop)
            0x09 , 0x06,                    // USAGE (Keyboard)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , STD_KB_REPORT_ID,        //    REPORT_ID (1)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x95 , 0x08,                    //    REPORT_COUNT (8)
            0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
            0x19 , 0xE0,                    //    USAGE_MINIMUM (Keyboard LeftControl)
            0x29 , 0xE7,                    //    USAGE_MAXIMUM (Keyboard Right GUI)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
            0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
            0x95 , 0x05,                    //    REPORT_COUNT (5)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x05 , 0x08,                    //    USAGE_PAGE (LEDs)
            0x19 , 0x01,                    //    USAGE_MINIMUM (Num Lock)
            0x29 , 0x05,                    //    USAGE_MAXIMUM (Kana)
            0x91 , 0x02,                    //    OUTPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x03,                    //    REPORT_SIZE (3)
            0x91 , 0x03,                    //    OUTPUT (Cnst,Var,Abs)
            0x95 , 0x06,                    //    REPORT_COUNT (6)
            0x75 , 0x08,                    //    REPORT_SIZE (8)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x26 , 0xFF , 0x00,             //    LOGICAL_MAXIMUM (255)
            0x05 , 0x07,                    //    USAGE_PAGE (Keyboard)
            0x19 , 0x00,                    //    USAGE_MINIMUM (Reserved (no event indicated))
            0x29 , 0xFF,                    //    USAGE_MAXIMUM (Reserved (no event indicated))
            0x81 , 0x00,                    //    INPUT (Data,Ary,Abs)
            0xC0,                           // END_COLLECTION

            //Bit mapped report, BITMAPPED_REPORT_ID
            0x05 , 0x0C,                    // USAGE_PAGE (Consumer Devices)
            0x09 , 0x01,                    // USAGE (Consumer Control)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , BITMAPPED_REPORT_ID,     //    REPORT_ID (2)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x95 , 0x12,                    //    REPORT_COUNT (18)
            //byte 0
            0x09 , 0x30,                    //    USAGE (Power)
            0x0A , 0x92 , 0x01,             //    USAGE (AL Calculator)
            0x0A , 0x83 , 0x01,             //    USAGE (Media)
            0x0A , 0x23 , 0x02,             //    USAGE (WWW Home)
            0x0A , 0x8A , 0x01,             //    USAGE (AL Email)
            0x0A , 0x82 , 0x01,             //    USAGE (Programmable Button Control)
            0x0A , 0x21 , 0x02,             //    USAGE (AC Search)
            0x0A , 0x24 , 0x02,             //    USAGE (AC Back)
            // byte 1
            0x0A , 0xA6 , 0x01,             //    USAGE (AL Integrated Help center)
            0x0A , 0xAE , 0x01,             //    USAGE (AL Keyboard Layout)
            0x0A , 0x27 , 0x02,             //    USAGE (AC Refresh)
            0x09 , 0xB6,                    //    USAGE (Scan Previous Track)
            0x09 , 0xB5,                    //    USAGE (Scan Next Track)
            0x09 , 0xB8,                    //    USAGE (Eject)
            0x09 , 0xCD,                    //    USAGE (Play/pause)
            0x09 , 0xE9,                    //    USAGE (Volume Up)
            //byte 2
            0x09 , 0xEA,                    //    USAGE (Volume Down)
            0x09 , 0xE2,                    //    USAGE (Mute)
            0x81 , 0x02,                    //    INPUT (Data,Var,Abs)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x06,                    //    REPORT_SIZE (6)
            0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
            0xC0,                           // END_COLLECTION

            0x05 , 0x01,                    // USAGE_PAGE (Generic Desktop)
            0x09 , 0x80,                    // Usage (System Control)
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , SLEEP_REPORT_ID,         //    REPORT_ID (4)
            0x15 , 0x00,                    //    LOGICAL_MINIMUM (0)
            0x25 , 0x01,                    //    LOGICAL_MAXIMUM (1)
            0x75 , 0x01,                    //    REPORT_SIZE (1)
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x09 , 0x82,                    //    USAGE (System Sleep)
            0x81 , 0x02,                    //    Input (Data, Variable, Absolute),
            0x95 , 0x01,                    //    REPORT_COUNT (1)
            0x75 , 0x07,                    //    REPORT_SIZE (7)
            0x81 , 0x03,                    //    INPUT (Cnst,Var,Abs)
            0xC0,                           // END_COLLECTION

            //Func Lock, FUNC_LOCK_REPORT_ID,
            0x05 , 0x0C,                    // Usage Page (Consumer Devices)
            0x09 , 0x01,                    //    Usage (Consumer Control)
            0xA1 , FUNC_LOCK_REPORT_ID,     //    Collection (Application)
            0x85 , 0x05,                    //      Report ID=05
            0x05 , 0x01,                    //      Usage Page (Generic Desktop),
            0x09 , 0x06,                    //      Usage (Keyboard)
            0xA1 , 0x02,                    //      Collection: (Logical),
            0x06 , 0x00 , 0xFF,             //        Usage Page (Vendor Specific)
            0x25 , 0x01,                    //        LOGICAL_MAXIMUM (1)
            0x75 , 0x01,                    //        REPORT_SIZE (1)
            0x95 , 0x02,                    //        REPORT_COUNT (2)
            0x0A , 0x03 , 0xFE,             //        USAGE (Func Lock State)
            0x0A , 0x04 , 0xFE,             //        USAGE (Func Lock Event)
            0x81 , 0x02,                    //        Input (Data, Variable, Absolute),
            0x95 , 0x06,                    //        REPORT_COUNT (6)
            0x81 , 0x03,                    //        INPUT (Cnst,Var,Abs)
            0xC0,                           //    END_COLLECTION (Logical)
            0xC0,                           // END_COLLECTION

            //SCROLL_REPORT_ID.Created by GID Descriptor Tool
            //char ReportDescriptor[29] = {
            0x05, 0x0c,                    // USAGE_PAGE (Consumer Devices)
            0x09, 0x01,                    // USAGE (Consumer Control)
            0xa1, 0x01,                    // COLLECTION (Application)
            0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
            0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
            0x85, SCROLL_REPORT_ID,        //   REPORT_ID (6)
            0x09, 0xe9,                    //   USAGE (Volume Up)
            0x09, 0xea,                    //   USAGE (Volume Down)
            0x75, 0x01,                    //   REPORT_SIZE (1)
            0x95, 0x02,                    //   REPORT_COUNT (2)
            0x81, 0x06,                    //   INPUT (Data,Var,Abs)
            0x75, 0x01,                    //   REPORT_SIZE (1)
            0x95, 0x06,                    //   REPORT_COUNT (6)
            0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
            0xc0,                          // END_COLLECTION

            //Battery report
            0x05 , 0x0C,                    // Usage Page (Consumer Devices),
            0x09 , 0x01,                    // Usage (Consumer Control),
            0xA1 , 0x01,                    // COLLECTION (Application)
            0x85 , BATTERY_REPORT_ID,       //    REPORT_ID (3)
            0x05 , 0x01,                    //    Usage Page (Generic Desktop),
            0x09 , 0x06,                    //    Usage (Keyboard)
            0xA1 , 0x02,                    //    Collection: (Logical),
            0x05 , 0x06,                    //        USAGE PAGE (Generic Device Control),
            0x09 , 0x20,                    //        USAGE (Battery Strength),
            0x15 , 0x00,                    //        Log Min (0),
            0x26 , 0x64 , 0x00,             //        Log Max (255),
            0x75 , 0x08,                    //        Report Size (8),
            0x95 , 0x01,                    //        Report Count (1),
            0x81 , 0x02,                    //        Input (Data, Variable, Absolute),
            0xC0,                           //    END_COLLECTION (Logical)
            0xC0,                           // END_COLLECTION
            // END of Battery report
};

/*****************************************************************************
 * This is the attribute table containing LEGATTDB_PERM_READABLE attributes
  ****************************************************************************/
const attribute_t blehid_gattAttributes[] =
{
#if INCLUDE_GATT_SERVICE_CHANGED
    {
        HANDLE_BLEKB_GATT_SERVICE_CHANGED_VAL,
        sizeof(hid_service_changed),
        hid_service_changed
    },
#endif

    {
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        sizeof(dev_local_name),
        dev_local_name  //fixed
    },

    {
        HANDLE_BLEKB_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        sizeof(dev_appearance_name),
        dev_appearance_name //fixed
    },

    {
        HANDLE_BLEKB_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        8,
        ble_hidd_link.prefered_conn_params //fixed
    },

    {
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        sizeof(dev_pnp_id),
        dev_pnp_id  //fixed
    },

    {
        HANDLE_BLEKB_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        sizeof(dev_char_mfr_name_value),
        dev_char_mfr_name_value //fixed
    },

    {
        HANDLE_BLEKB_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        1,
        &battery_level //get the battery reading from ADC
    },

    {
        HANDLE_BLEKB_BATTERY_SERVICE_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[5]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT   (0x20)
    },

    {
        HANDLE_BLEKB_BATTERY_SERVICE_RPT_REF_DESCR,
        2,
        rpt_ref_battery     //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_PROTO_MODE_VAL,
        1,
        &kbapp_protocol   //Report Protocol mode
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_INFO_VAL,
        4,
        dev_hid_information   //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_VAL,
        KEYRPT_MAX_KEYS_IN_STD_REPORT+2,
        blekb_key_std_rpt   //updated before a std key input report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[0]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BOOT_RPT          (0x01)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_BT_KB_OUTPUT_VAL,
        1,
        &blekb_kb_output_rpt   //updated after a std key output report received
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        sizeof(blehid_rpt_map),
        blehid_rpt_map  //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        2,
        &dev_battery_service_uuid   //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        KEYRPT_MAX_KEYS_IN_STD_REPORT+2,
        blekb_key_std_rpt   //updated before a std key input report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[1]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT           (0x02)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_input   //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        1,
        &blekb_kb_output_rpt    //updated after a std key output report received
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_output  //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT,
        blekb_bitmap_rpt    //updated before a bitmap report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[2]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT    (0x04)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        2,
        rpt_ref_bitmap  //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_VAL,
        1,
        &blekb_sleep_rpt    //updated before a sleep report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[3]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_SLP_RPT           (0x08)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SLEEP_RPT_REF_DESCR,
        2,
        rpt_ref_sleep   //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_VAL,
        1,
        &blekb_func_lock_rpt    //updated before a func lock report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[4]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_FUNC_LOCK_RPT     (0x10)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_FUNC_LOCK_RPT_REF_DESCR,
        2,
        rpt_ref_func_lock   //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_VAL,
        1,
        &blekb_scroll_rpt   //updated before a scroll report sent
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_CHAR_CFG_DESCR,
        2,
        &characteristic_client_configuration[6]  //bit mask: KBAPP_CLIENT_CONFIG_NOTIF_SCROLL_RPT        (0x40)
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_SCROLL_RPT_REF_DESCR,
        2,
        rpt_ref_scroll  //fixed
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        1,
        &blekb_connection_ctrl_rpt  //even though it is defined. But no usage. ignore now.
    },

    {
        HANDLE_BLEKB_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        2,
        rpt_ref_connection_ctrl //fixed
    },
};
const uint16_t blehid_gattAttributes_size = sizeof(blehid_gattAttributes)/sizeof(attribute_t);


/*****************************************************************************
* Keyboard application configuration. Defines behavior of the keyboard
* application
*****************************************************************************/
#define USE_KEYSCAN_KEY_TO_PAIR 1
KbAppConfig kbAppConfig =
{
    // Standard report ID
    STD_KB_REPORT_ID,

    // Maximum number of keys in standard report
    KEYRPT_MAX_KEYS_IN_STD_REPORT,

    // Report ID of bit mapped report
    BITMAPPED_REPORT_ID,

    // Number of bit mapped keys
    BIT_MAPPED_MAX,

    // Sleep report ID
    SLEEP_REPORT_ID,

    // Pin report ID
    NOT_USED_REPORT_ID,

    // LED (output) reportID
    LED_OUTPUT_REPORT_ID,

    // Default LED state is all off
    0,
#if USE_KEYSCAN_KEY_TO_PAIR
    94, // index 94 -- lock key
#else
    // Connect button scan index: for using one key_matrix key. Use 0xFF since we are using GPIO button.
    0xff, // 69
#endif

    // Recovery poll count
    3,

    // HW fifo threshold for idle rate report generation
    3,

    // Repeat the rollover report after every half a second, i.e. 1600 BT clocks
    1600,

    // Only repeat the rollover report if the HW fifo has fewer than this number of packets
    3,

    // Func lock report ID
    FUNC_LOCK_REPORT_ID,

    // Default state of the func-lock key
    0,

    // Scroll report ID
    SCROLL_REPORT_ID,

    // Scroll report length
    1,

    // No negation of scroll values
    FALSE,

    // Scroll scaling of 2 (2^1) for normal mechanical scroll
    1,

    // Keep fractional scroll data around for 50 polls
    50,

    // Enable scroll combining
    TRUE,

    // Size of each event
    6,

    // Maximum number of events
    44
};

/*****************************************************************************
/// Key translation table. It maps keyscan matrix position to key types and
/// specific usage within the type. For example, row 5, column 6 may be
/// mapped as a standard key with usage "ESCAPE". This means that the key
/// will be reported in the standard report with a USB usage of "ESCAPE"
/// See config documentation for details and the keyboard config for an example.
/// By default this table is initialized for the BCM keyboard
*****************************************************************************/
KbKeyConfig kbKeyConfig[] =
{
// Column 0:  row0 ->row7
/*   0 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*   1 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*   2 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*   3 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*   4 */ {KEY_TYPE_BIT_MAPPED, BIT_MAPPED_RGB},
/*   5 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*   6 */ {KEY_TYPE_BIT_MAPPED, BIT_MAPPED_FUNCTION},
/*   7 */ {FN_KEY_TYPE,         FN5_KEYCODE},

// Column 1: order is row0 ->row7
/*   8 */ {KEY_TYPE_STD,        USB_USAGE_Q},
/*   9 */ {KEY_TYPE_STD,        USB_USAGE_TAB},
/*  10 */ {KEY_TYPE_STD,        USB_USAGE_A},
/*  11 */ {KEY_TYPE_STD,        USB_USAGE_ESCAPE},
/*  12 */ {KEY_TYPE_STD,        USB_USAGE_Z},
/*  13 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  14 */ {KEY_TYPE_STD,        USB_USAGE_ACCENT},
/*  15 */ {KEY_TYPE_STD,        USB_USAGE_1},

// Column 2: order is row0 ->row7
/*  16 */ {KEY_TYPE_STD,        USB_USAGE_W},
/*  17 */ {KEY_TYPE_STD,        USB_USAGE_CAPS_LOCK},
/*  18 */ {KEY_TYPE_STD,        USB_USAGE_S},
/*  19 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  20 */ {KEY_TYPE_STD,        USB_USAGE_X},
/*  21 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  22 */ {FN_KEY_TYPE,         FN1_KEYCODE},
/*  23 */ {KEY_TYPE_STD,        USB_USAGE_2},

// Column 3: order is row0 ->row7
/*  24 */ {KEY_TYPE_STD,        USB_USAGE_E},
/*  25 */ {FN_KEY_TYPE,         FN3_KEYCODE},
/*  26 */ {KEY_TYPE_STD,        USB_USAGE_D},
/*  27 */ {FN_KEY_TYPE,         FN4_KEYCODE},
/*  28 */ {KEY_TYPE_STD,        USB_USAGE_C},
/*  29 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  30 */ {FN_KEY_TYPE,         FN2_KEYCODE},
/*  31 */ {KEY_TYPE_STD,        USB_USAGE_3},

// Column 4: order is row0 ->row7
/*  32 */ {KEY_TYPE_STD,        USB_USAGE_R},
/*  33 */ {KEY_TYPE_STD,        USB_USAGE_T},
/*  34 */ {KEY_TYPE_STD,        USB_USAGE_F},
/*  35 */ {KEY_TYPE_STD,        USB_USAGE_G},
/*  36 */ {KEY_TYPE_STD,        USB_USAGE_V},
/*  37 */ {KEY_TYPE_STD,        USB_USAGE_B},
/*  38 */ {KEY_TYPE_STD,        USB_USAGE_5},
/*  39 */ {KEY_TYPE_STD,        USB_USAGE_4},

// Column 5: order is row0 ->row7
/*  40 */ {KEY_TYPE_STD,        USB_USAGE_U},
/*  41 */ {KEY_TYPE_STD,        USB_USAGE_Y},
/*  42 */ {KEY_TYPE_STD,        USB_USAGE_J},
/*  43 */ {KEY_TYPE_STD,        USB_USAGE_H},
/*  44 */ {KEY_TYPE_STD,        USB_USAGE_M},
/*  45 */ {KEY_TYPE_STD,        USB_USAGE_N},
/*  46 */ {KEY_TYPE_STD,        USB_USAGE_6},
/*  47 */ {KEY_TYPE_STD,        USB_USAGE_7},

// Column 6: order is row0 ->row7
/*  48 */ {KEY_TYPE_STD,        USB_USAGE_I},
/*  49 */ {KEY_TYPE_STD,        USB_USAGE_RIGHT_BRACKET},
/*  50 */ {KEY_TYPE_STD,        USB_USAGE_K},
/*  51 */ {FN_KEY_TYPE,         FN6_KEYCODE},
/*  52 */ {KEY_TYPE_STD,        USB_USAGE_COMMA},
/*  53 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  54 */ {KEY_TYPE_STD,        USB_USAGE_EQUAL},
/*  55 */ {KEY_TYPE_STD,        USB_USAGE_8},

// Column 7: order is row0 ->row7
/*  56 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  57 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  58 */ {MD_KEY_TYPE,         MD_LEFT_CTL},
/*  59 */ {MD_KEY_TYPE,         MD_LEFT_GUI},
/*  60 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  61 */ {KEY_TYPE_BIT_MAPPED, BIT_MAPPED_LIGHT},
/*  62 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  63 */ {KEY_TYPE_STD,        USB_USAGE_VOL_UP},

// Column 8: order is row0 ->row7
/*  64 */ {KEY_TYPE_STD,        USB_USAGE_P},
/*  65 */ {KEY_TYPE_STD,        USB_USAGE_LEFT_BRACKET},
/*  66 */ {KEY_TYPE_STD,        USB_USAGE_SEMICOLON},
/*  67 */ {KEY_TYPE_STD,        USB_USAGE_QUOTE},
/*  68 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  69 */ {KEY_TYPE_STD,        USB_USAGE_SLASH},
/*  70 */ {KEY_TYPE_STD,        USB_USAGE_MINUS},
/*  71 */ {KEY_TYPE_STD,        USB_USAGE_0},

// Column 9: order is row0 ->row7
/*  72 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  73 */ {KEY_TYPE_STD,        USB_USAGE_BACKSPACE},
/*  74 */ {KEY_TYPE_STD,        USB_USAGE_BACK_SLASH},
/*  75 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  76 */ {KEY_TYPE_STD,        USB_USAGE_ENTER},
/*  77 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  78 */ {FN_KEY_TYPE,         FN9_KEYCODE},
/*  79 */ {KEY_TYPE_STD,        FN10_KEYCODE},  // both F10 or MUTE keys are STD KEY

// Column 10: order is row0 ->row7
/*  80 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  81 */ {MD_KEY_TYPE,         MD_LEFT_SHIFT},
/*  82 */ {MD_KEY_TYPE,         MD_RIGHT_SHIFT},
/*  83 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  84 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  85 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  86 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  87 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 11: order is row0 ->row7
/*  88 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  89 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  90 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  91 */ {KEY_TYPE_STD,        USB_USAGE_SPACEBAR},
/*  92 */ {KEY_TYPE_STD,        USB_USAGE_VOL_DOWN},
/*  93 */ {KEY_TYPE_STD,        USB_USAGE_DOWN_ARROW},
/*  94 */ {KEY_TYPE_STD,        USB_USAGE_SCROLL_LOCK},
/*  95 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 12: order is row0 ->row7
/*  96 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  97 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  98 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/*  99 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 100 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 101 */ {KEY_TYPE_STD,        USB_USAGE_RIGHT_ARROW},
/* 102 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 103 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 13: order is row0 ->row7
/* 104 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 105 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 106 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 107 */ {KEY_TYPE_STD,        USB_USAGE_UP_ARROW},
/* 108 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 109 */ {KEY_TYPE_STD,        USB_USAGE_LEFT_ARROW},
/* 110 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 111 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 14: order is row0 ->row7
/* 112 */ {KEY_TYPE_STD,        USB_USAGE_O},
/* 113 */ {FN_KEY_TYPE,         FN7_KEYCODE},
/* 114 */ {KEY_TYPE_STD,        USB_USAGE_L},
/* 115 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 116 */ {KEY_TYPE_STD,        USB_USAGE_STOP_AND_GREATER},
/* 117 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 118 */ {FN_KEY_TYPE,         FN8_KEYCODE},
/* 119 */ {KEY_TYPE_STD,        USB_USAGE_9},

// Column 15: order is row0 ->row7
/* 120 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 121 */ {MD_KEY_TYPE,         MD_LEFT_ALT},
/* 122 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 123 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 124 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 125 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 126 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 127 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 16: order is row0 ->row7
/* 128 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 129 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 130 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 131 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 132 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 133 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 134 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 135 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

// Column 17: order is row0 ->row7
/* 136 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 137 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 138 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 139 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 140 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 141 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 142 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},
/* 143 */ {KEY_TYPE_NONE,       USB_USAGE_NO_EVENT},

};

const uint8_t kbKeyConfig_size = sizeof(kbKeyConfig)/sizeof(KbKeyConfig);

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings =
{
    .device_name                         = (uint8_t*)dev_local_name,                                   /**< Local device name (NULL terminated) */
    .device_class                        = {0x00, 0x05, 0xc0},                                         /**< Local device class */
    .security_requirement_mask           = BTM_SEC_ENCRYPT,                                            /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 1,                                                          /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg =                                              /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg =                                                 /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_PASSIVE,                                  /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,               /**< High duty scan interval */
        .high_duty_scan_window           = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,                 /**< High duty scan window */
        .high_duty_scan_duration         = 5,                                                          /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,                /**< Low duty scan interval  */
        .low_duty_scan_window            = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,                  /**< Low duty scan window */
        .low_duty_scan_duration          = 5,                                                          /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,          /**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,            /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                                         /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL,           /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,             /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                                         /**< Low duty cycle connection duration in seconds (0 for infinite) */

        /* Connection configuration */
        .conn_min_interval               = 18,                                                         /**< Minimum connection interval */
        .conn_max_interval               = 18,                                                         /**< Maximum connection interval */
        .conn_latency                    = 21,                          /**< Connection latency */
        .conn_supervision_timeout        = 600,                                                        /**< Connection link supervision timeout */
    },

    .ble_advert_cfg =                                               /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map (mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39) */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = 32,                                                         /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = 32,                                                         /**< High duty undirected connectable maximum advertising interval */
#ifdef ALLOW_SDS_IN_DISCOVERABLE
        .high_duty_duration              = 0,                                                          /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        .high_duty_duration              = 30,                                                         /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif
        .low_duty_min_interval           = 48,             /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = 48,             /**< Low duty undirected connectable maximum advertising interval */
#ifdef ALLOW_SDS_IN_DISCOVERABLE
        .low_duty_duration               = 0,                                                          /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        .low_duty_duration               = 180,                                                        /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif
        .high_duty_directed_min_interval = 32,                                                         /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = 32,                                                         /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = 2048,                                                       /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = 2048,                                                       /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 0,                                                          /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                         /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                           /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg =                                                     /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                                      /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 1,                                                           /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 1,                                                           /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 180,                                                         /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 185                                                           /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                   /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = 0                                                            /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                            /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                           /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                           /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                           /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                           /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0                                                            /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 0,                                                           /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = 0                                                            /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                     /* Audio/Video Remote Control configuration */
    {
        .roles                          = 0,                                                           /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 0                                                            /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                           /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 185,                                                         /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                           /**< Max. power level of the device */
#else
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 4,                                                           /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
#ifdef LE_LOCAL_PRIVACY_SUPPORT
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,          /**< Interval of  random address refreshing - secs */
#else
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,            /**< Interval of  random address refreshing - secs */
#endif
    /* BLE white list size */
    .ble_white_list_size                = 0,                                                           /**< Maximum number of white list devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    .default_ble_power_level            = 0                                                            /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

/*****************************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       20        }, /* Small Buffer Pool */
    { 100,      30        }, /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 330,      8         }, /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,     2         }, /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};
