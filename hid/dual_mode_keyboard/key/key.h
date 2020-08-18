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
 * Keyscan Interface definitions
 *
 */
#ifndef __KEY_H__
#define __KEY_H__

#include "wiced.h"
#include "hidevent.h"
#include "keyscan.h"

/// Maximum number of keys in a standard key report. Technically the report is
/// limited to 6 keys. A BLE ATT can hold 23 bytes. We'll
/// only use 6. The length of a non-boot mode report will be set through the config
/// record
#define KEY_MAX_KEYS_IN_STD_REPORT    6

/// Maximum number of bytes in the bit-mapped key report structure.
/// A BLE ATT can hold 23 bytes.
#define KEY_NUM_BYTES_IN_BIT_MAPPED_REPORT   11
#define KEY_NUM_BYTES_IN_USER_DEFINED_REPORT   8

/// Func lock key state
typedef enum
{
    /// Func lock key is up
    FUNC_LOCK_KEY_UP,

    /// Func lock key is down
    FUNC_LOCK_KEY_DOWN
}FuncLockKeyPosition;

enum KeyType
{
    /// Represents no key. This should not occur normally
    KEY_TYPE_NONE,

    /// Represents a standard key. The associated translation code represents the reported value
    /// of this key
    KEY_TYPE_STD,

    /// Represents a modifier key. The associated translation value indicates which bit
    /// in the modifier key mask is controlled by this key
    KEY_TYPE_MODIFIER,

    /// Represents a bit mapped key in the bit mapped report. The associated translation value
    /// provides the row col of the bit which represents this key
    KEY_TYPE_BIT_MAPPED,

    /// The sleep key
    KEY_TYPE_SLEEP,

    /// The function lock key
    KEY_TYPE_FUNC_LOCK,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_0,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_MAX
};

///
/// Report structures must be packed
///
#pragma pack(1)
/// Standard key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record. 1 is recommended for boot-mode support
    uint8_t    reportID;

    /// Modifier keys
    uint8_t    modifierKeys;

    /// Reserved (OEM). Normally set to 0 unless changed by application code.
    uint8_t    reserved;

    /// Key array.
    uint8_t    keyCodes[KEY_MAX_KEYS_IN_STD_REPORT];
}KeyboardStandardReport;

/// Bit mapped key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Bit mapped keys
    uint8_t    bitMappedKeys[KEY_NUM_BYTES_IN_BIT_MAPPED_REPORT];
}KeyboardBitMappedReport;

/// Pin entry report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Set to the value specified in the config record.
    uint8_t    reportCode;
}KeyboardPinEntryReport;

/// Sleep report structure. Sent when sleep key press is detected.
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Set to the value specified in the config record.
    uint8_t    sleepVal;
}KeyboardSleepReport;

/// Func lock report struct. Sent when the func lock key is pressed
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Set to the value specified in the config record.
    uint8_t    status;
}KeyboardFuncLockReport;

/// Keyboard output report. Sets the LED state
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// State of various LEDs
    uint8_t    ledStates;
}KeyboardLedReport;

/// Scroll report structure. This can scroll wheels, track balls, and volume knobs.
/// This report allows for 3 axis, but most devices will only use one or 2
#define KEYRPT_NUM_RESERVED_BYTES_IN_MOTION_REPORT  5
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Motion along axis 0
    int16_t    motionAxis0;

    /// Motion along axis 1
    int16_t    motionAxis1;

    /// Motion along axis 2
    int16_t    motionAxis2;

    /// Reserved bytes. Can be used by application to extend motion report
    uint8_t    reserved[KEYRPT_NUM_RESERVED_BYTES_IN_MOTION_REPORT];
}KeyboardMotionReport;

typedef struct {
    /// Standard key report
    KeyboardStandardReport  stdRpt;

    /// Bit mapped key report
    KeyboardBitMappedReport bitMappedReport;

    /// Function lock report
    KeyboardFuncLockReport  funcLockReport;

    /// Sleep report
    KeyboardSleepReport     sleepReport;

    /// scroll report
    KeyboardMotionReport    scrollReport;

#ifdef SUPPORT_CODE_ENTRY
    /// pin report
    KeyboardPinEntryReport  pinReport;
#endif

    KeyboardLedReport       ledReport;

} key_input_rpt_t;
#pragma pack()

typedef PACKED struct
{
    /// Type of key, e.g. std key, modifier key, etc.
    uint8_t    type;

    /// Translation code. The actual value depend on the key type.
    ///     - For modifier keys, it  is a bit mask for the reported key
    ///     - For std key, it is the usage provided in the std key report
    ///     - For bit mapped keys, it is the row/col of the associated bit in the bit mapped report
    uint8_t    translationValue;
}KbKeyConfig;

extern key_input_rpt_t key_rpts;
extern KbKeyConfig kbKeyConfig[];

#ifdef SUPPORT_KEY_REPORT
/********************************************************************************
 * Function Name: void key_procEvtKey(uint8_t keyCode, uint8_t keyDown)
 ********************************************************************************
 * Summary: process key event
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  TRUE -- handled correctly
 *  FALSE -- error detected
 *
 *******************************************************************************/
wiced_bool_t key_procEvtKey(uint8_t keyCode, uint8_t keyDown);

/********************************************************************************
 * Function Name: void key_init
 ********************************************************************************
 * Summary: initialize key report
 *
 * Parameters:
 *  row, col -- key matrix row & col dimention
 *  poll_callback_t * pcb -- application poll function pointer to poll user activities
 *  keyPressDetected_callback_t * cb -- application callback function pointer to handle key event
 *
 * Return:
 *  none
 *
 *******************************************************************************/
#define key_init(r,c,pcb,cb) kscan_init(r,c,pcb,cb)

/********************************************************************************
 * Function Name: void key_send(void)
 ********************************************************************************
 * Summary: Send any pending key reports.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_send();

/********************************************************************************
 * Function Name: void key_pinReport(uint8_t code)
 ********************************************************************************
 * Summary: Prepare to pin report to be sent
 *
 * Parameters:
 *  code -- event code
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_pinReport(uint8_t code);

/********************************************************************************
 * Function Name: void key_clear(void)
 ********************************************************************************
 * Summary: clear all key reports
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_clear(wiced_bool_t sendRpt);

/********************************************************************************
 * Function Name: void key_sendRollover()
 ********************************************************************************
 * Summary: Send a rollover packet
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_sendRollover();

/********************************************************************************
 * Function Name: void key_setReport()
 ********************************************************************************
 * Summary: handle HID setReport for keyboard
 *
 * Parameters:
 *  reportType -- report type (defined in wiced_hidd_report_type_e)
 *  reportId   -- Report ID
 *  payload    -- data pointer
 *  payloadSize -- data length
 *
 * Return:
 *  TRUE -- if handled
 *
 *******************************************************************************/
wiced_bool_t key_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);

#else
 #define key_procEvtKey(c,d) TRUE
 #define key_init()
 #define key_send()
 #define key_clear(s)
 #define key_sendRollover();
 #define key_setReport(t,r,p,s) FALSE

#endif // SUPPORT_KEY_REPORT
#endif // __KEY_H__
