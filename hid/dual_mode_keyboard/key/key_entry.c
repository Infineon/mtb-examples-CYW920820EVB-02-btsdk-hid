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
 * code_entry (for pin_code/pass_code entry)
 *
 */

#ifdef SUPPORT_CODE_ENTRY

#include "app.h"
#include "usb_usage.h"
#include "hidd_lib.h"

enum
{
    CODE_ENTRY_NONE,
    CODE_ENTRY_PINCODE,
    CODE_ENTRY_PASSCODE,
};

enum
{
    ENTRY_EVENT_START,
    ENTRY_EVENT_CHAR,
    ENTRY_EVENT_BACKSPACE,
    ENTRY_EVENT_RESTART,
    ENTRY_EVENT_STOP,
};
#define PIN_ENTRY_CODE(n) (n-1)
#define INVALID_ASCII 0xff

enum
{
    /// Maximum pin code size
    MAX_PIN_SIZE = 16,

    /// Maximum number of digits in pass code
    MAX_PASS_SIZE = 6
};

typedef struct {
    uint8_t mode;
    uint8_t codeSize;
    uint8_t codeBuffer[MAX_PIN_SIZE];
    uint8_t maxCodeSize;
} code_entry_t;

code_entry_t codeEntry = {};

/********************************************************************************
 * Function Name: Key_entry_reset()
 ********************************************************************************
 * Summary: Reset enter mode by clearing input buffer.
 *
 * Parameters:
 *  none  -- When called, the key entry mode should be in CODE_ENTRY_PINCODE or CODE_ENTRY_PASSCODE
 *           The maxCodeSize also should be already initialized
 *
 * Return:
 *  none
 *
 *******************************************************************************/
STATIC void Key_entry_reset()
{
    codeEntry.codeSize = 0;
    memset(codeEntry.codeBuffer, 0, MAX_PIN_SIZE);        // Clear any previous pin code
}

/********************************************************************************
 * Function Name: Key_entry_event()
 ********************************************************************************
 * Summary: Enter new entry mode.
 *
 * Parameters:
 *  newMode  -- should be either CODE_ENTRY_PINCODE or CODE_ENTRY_PASSCODE
 *
 * Return:
 *  none
 *
 *******************************************************************************/
STATIC void Key_entry_event(uint8_t event)
{
    switch (codeEntry.mode) {
    case CODE_ENTRY_PINCODE:
        switch (event) {
        case ENTRY_EVENT_CHAR:
        case ENTRY_EVENT_BACKSPACE:
        case ENTRY_EVENT_RESTART:
            key_pinReport(PIN_ENTRY_CODE(event));
            break;
        case ENTRY_EVENT_STOP:
            // Pass the pin code on to the authenticating transport
            hidd_link_pinCode(codeEntry.codeSize, codeEntry.codeBuffer);
            break;
        }
        break;

    case CODE_ENTRY_PASSCODE:
        hidd_link_passCodeKeyPressReport(event);
        if (event == ENTRY_EVENT_STOP)
        {
            // Null terminate the buffer
            codeEntry.codeBuffer[codeEntry.codeSize] = 0;
            // Pass the pass key on to the authenticating transport
            hidd_link_passCode(codeEntry.codeSize, codeEntry.codeBuffer);
        }
        break;
    }
}

/********************************************************************************
 * Function Name: Key_entry_enterMode()
 ********************************************************************************
 * Summary: Enter new entry mode.
 *
 * Parameters:
 *  newMode  -- should be either CODE_ENTRY_PINCODE or CODE_ENTRY_PASSCODE
 *
 * Return:
 *  none
 *
 *******************************************************************************/
STATIC void Key_entry_enterMode(uint8_t newMode)
{
    // If we are not already in some pin entry mode
    if(!key_entry_idle())
    {
        // Some pin code request pending, disconnect
        hidd_link_disconnect();
    }
    else
    {
        // Flag that pin code entry is in progress
        codeEntry.mode = newMode;
        codeEntry.maxCodeSize = (newMode==CODE_ENTRY_PASSCODE) ? MAX_PASS_SIZE : MAX_PIN_SIZE;

        Key_entry_reset();
    }
}

/********************************************************************************
 * Function Name: Key_entry_usb2numericAscii()
 ********************************************************************************
 * Summary: Convert USB Usage Code to ASCII numeric char
 *          if the code is not numeric key, then return INVLIAD_ASCII
 *
 * Parameters:
 *  usbUsageCode  -- the key code defined by USB Usage
 *
 * Return:
 *  numeric ascii or INVALID_ASCII
 *
 *******************************************************************************/
STATIC BYTE Key_entry_usb2numericAscii(BYTE usbUsageCode)
{
    // 0 is special; handle it seperately
    if (usbUsageCode == USB_USAGE_0 || usbUsageCode == USB_USAGE_KP_0)
    {
        return '0';
    }

    // Check for keyboard 1-9
    if (usbUsageCode >= USB_USAGE_1 && usbUsageCode <= USB_USAGE_9)
    {
        return usbUsageCode - USB_USAGE_1 + '1';
    }

    // Check for numpad 1-9
    if (usbUsageCode >= USB_USAGE_KP_1 && usbUsageCode <= USB_USAGE_KP_9)
    {
        return usbUsageCode - USB_USAGE_KP_1 + '1';
    }

    return INVALID_ASCII;
}

/********************************************************************************
 * Function Name: key_entry_idle()
 ********************************************************************************
 * Summary: return true if the key_entry mode is idle
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
wiced_bool_t key_entry_idle()
{
    return codeEntry.mode == CODE_ENTRY_NONE;
}

////////////////////////////////////////////////////////////////////////////////
/// This function provides pin code entry functionality on the keyboard.
/// It processes all pending events in the event fifo and uses them to construct
/// the pin code. All non-key events will be thrown away, as well as any unrecognized keys
/// This function uses the translation code of each key and assumes that the translation
/// code will match the USB usage. The following USB usage codes are understood:
///     0-9, Enter, Key Pad Enter, Backspace, Delete (works like backspace),
///     Escape (resets pin entry)
////////////////////////////////////////////////////////////////////////////////
void key_entry_handleCode(wiced_hidd_app_event_queue_t * pEventQueue)
{
    HidEventKey *curEvent;

    // Process events until the FIFO is empty
    while ((curEvent = (HidEventKey *)wiced_hidd_event_queue_get_current_element(pEventQueue)))
    {
        // We only process key events here
        if ((curEvent->eventInfo.eventType == HID_EVENT_KEY_STATE_CHANGE) && (curEvent->keyEvent.upDownFlag==KEY_DOWN))
        {
            BYTE keyCode = curEvent->keyEvent.keyCode;

            // Only process it if it is a standard key
            if (kbKeyConfig[keyCode].type == KEY_TYPE_STD)
            {
                BYTE usbUsageCode = kbKeyConfig[keyCode].translationValue;

                switch (usbUsageCode) {
                // Backspace and delete are handled the same way
                case USB_USAGE_BACKSPACE:
                case USB_USAGE_DELETE:
                    // Check if we have any accumulated digits
                    if (codeEntry.codeSize)
                    {
                        // Kill the previous character
                        codeEntry.codeSize--;

                        // Update the pin code report
                        Key_entry_event(ENTRY_EVENT_BACKSPACE);
                    }
                    break;

                case USB_USAGE_ESCAPE:
                    Key_entry_reset();
                    Key_entry_event(ENTRY_EVENT_RESTART);
                    break;

                case USB_USAGE_ENTER:
                case USB_USAGE_KP_ENTER:
                    Key_entry_event(ENTRY_EVENT_STOP);
                    key_entry_exitCodeEntryMode();
                    break;

                default:
                    // Add it to the existing code if there is room
                    if (codeEntry.codeSize < codeEntry.maxCodeSize)
                    {
                        BYTE ascii = Key_entry_usb2numericAscii(usbUsageCode);
                        if (ascii != INVALID_ASCII)
                        {
                            codeEntry.codeBuffer[codeEntry.codeSize++] = ascii;
                            Key_entry_event(ENTRY_EVENT_CHAR);
                        }
                    }
                } // switch
            } // if
        } // if

        // We have consumed the current event
        wiced_hidd_event_queue_remove_current_element(pEventQueue);
    } // while
}

/********************************************************************************
 * Function Name: key_entry_exitCodeEntryMode()
 ********************************************************************************
 * Summary: exit key entry mode
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
 void key_entry_exitCodeEntryMode(void)
 {
     codeEntry.mode = CODE_ENTRY_NONE;
 }

///////////////////////////////////////////////////////////////////////////////
/// The keyboard application responds to a pin code entry request as follows:
/// - If user needs does not need to be prompted to enter pincode or user needs
///   a prompt and app is capable of prompting (through dosplay), flush any pending user input
///   and enter pin code entry mode. This is done by setting the flag
///   codeEntryInProgress to LEGACY_PIN_ENTRY_IN_PROGRESS.
/// - Else it rejects the request and tells the BT transport to disconnect.
////////////////////////////////////////////////////////////////////////////////
void key_entry_enterPinCodeEntryMode(void)
{
    Key_entry_enterMode(CODE_ENTRY_PINCODE);
}

////////////////////////////////////////////////////////////////////////////////
/// The KB app responds to pass code request as follows:
/// - if no other pin/pass code request is pending, flush any pending user input
///   and enter pin code entry mode. This is done by setting the flag
///   codeEntryInProgress to PASS_KEY_ENTRY_IN_PROGRESS
/// - Else it rejects the request and tells the BT transport to disconnect.
////////////////////////////////////////////////////////////////////////////////
void key_entry_enterPassCodeEntryMode(void)
{
    Key_entry_enterMode(CODE_ENTRY_PASSCODE);
    // Indicate pin entry start to the peer
    Key_entry_event(ENTRY_EVENT_START);
}

#endif // SUPPORT_CODE_ENTRY
