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
 * Keyscan Interface
 *
 */

#ifdef SUPPORT_KEY_REPORT

#include "app.h"
#include "usb_usage.h"

#define CODE_ROLLOVER 1

#define USE_MODIFIER 1
#if USE_MODIFIER
 #define MD_KEY_TYPE    KEY_TYPE_MODIFIER
 #define MD_LEFT_SHIFT  USB_MODKEY_MASK_LEFT_SHIFT
 #define MD_LEFT_ALT    USB_MODKEY_MASK_LEFT_ALT
 #define MD_LEFT_CTL    USB_MODKEY_MASK_LEFT_CTL
 #define MD_LEFT_GUI    USB_MODKEY_MASK_LEFT_GUI
 #define MD_RIGHT_SHIFT USB_MODKEY_MASK_RIGHT_SHIFT
#else
 #define MD_KEY_TYPE    KEY_TYPE_STD
 #define MD_LEFT_SHIFT  USB_USAGE_LEFT_SHIFT
 #define MD_LEFT_ALT    USB_USAGE_LEFT_ALT
 #define MD_LEFT_CTL    USB_USAGE_LEFT_CTL
 #define MD_LEFT_GUI    USB_USAGE_LEFT_GUI
 #define MD_RIGHT_SHIFT USB_USAGE_RIGHT_SHIFT
#endif

#define USE_FUNCTION_KEYS 0
#if USE_FUNCTION_KEYS
 #define FN_KEY_TYPE KEY_TYPE_STD
 #define FN1_KEYCODE USB_USAGE_F1
 #define FN2_KEYCODE USB_USAGE_F2
 #define FN3_KEYCODE USB_USAGE_F3
 #define FN4_KEYCODE USB_USAGE_F4
 #define FN5_KEYCODE USB_USAGE_F5
 #define FN6_KEYCODE USB_USAGE_F6
 #define FN7_KEYCODE USB_USAGE_F7
 #define FN8_KEYCODE USB_USAGE_F8
 #define FN9_KEYCODE USB_USAGE_F9
 #define FN10_KEYCODE USB_USAGE_F10
#else
 #define FN_KEY_TYPE KEY_TYPE_BIT_MAPPED
 #define FN1_KEYCODE BIT_MAPPED_LIGHT_DOWN
 #define FN2_KEYCODE BIT_MAPPED_LIGHT_UP
 #define FN3_KEYCODE BIT_MAPPED_EDIT
 #define FN4_KEYCODE BIT_MAPPED_SEARCH
 #define FN5_KEYCODE BIT_MAPPED_COPY
 #define FN6_KEYCODE BIT_MAPPED_EDIT
 #define FN7_KEYCODE BIT_MAPPED_REWIND
 #define FN8_KEYCODE BIT_MAPPED_PLAY_PAUSE
 #define FN9_KEYCODE BIT_MAPPED_FAST_FORWARD
 #define FN10_KEYCODE USB_USAGE_MUTE
#endif

enum
{
    USB_MODKEY_MASK_LEFT_CTL=0x01,
    USB_MODKEY_MASK_LEFT_SHIFT=0x02,
    USB_MODKEY_MASK_LEFT_ALT=0x04,
    USB_MODKEY_MASK_LEFT_GUI=0x08,
    USB_MODKEY_MASK_RIGHT_CTL=0x10,
    USB_MODKEY_MASK_RIGHT_SHIFT=0x20,
    USB_MODKEY_MASK_RIGHT_ALT=0x40,
    USB_MODKEY_MASK_RIGHT_GUI=0x80
};

/// Keyboard Key Config

/// Key types. Used to direct key codes to the relevant key processing function
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

#define KEY_TABLE_SIZE (sizeof(kbKeyConfig)/sizeof(KbKeyConfig))

//////////////////////////////////////////////////////////////////////////////
typedef struct {

    uint8_t                 stdRpt_changed:1;
    uint8_t                 bitMapped_changed:1;
    uint8_t                 funcLock_changed:1;
    uint8_t                 sleep_changed:1;
    uint8_t                 scroll_changed:1;
#ifdef SUPPORT_CODE_ENTRY
    uint8_t                 pin_changed:1;
#endif
} kbrpt_t;
static kbrpt_t keyRpt;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


key_input_rpt_t key_rpts = {
    .stdRpt          = {RPT_ID_IN_STD_KEY},
    .bitMappedReport = {RPT_ID_IN_BIT_MAPPED},
    .funcLockReport  = {RPT_ID_IN_FUNC_LOCK},
    .sleepReport     = {RPT_ID_IN_SLEEP},
    .scrollReport    = {RPT_ID_IN_SCROLL},
#ifdef SUPPORT_CODE_ENTRY
    .pinReport       = {RPT_ID_IN_PIN},
#endif
    .ledReport       = {RPT_ID_OUT_KB_LED},
};

/////////////////////////////////////////////////////////////////////////////////
/// This function transmits the remote report over the interrupt channel and
/********************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyDown(uint8_t translationCode)
 ********************************************************************************
 * Summary: add the key to standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyDown(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Check if the key is already in the report
    for (i=0; keyCodes[i] && (i < KEY_MAX_KEYS_IN_STD_REPORT); i++)
    {
        if (keyCodes[i] == key)
        {
            // Already in the report. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < KEY_MAX_KEYS_IN_STD_REPORT)
    {
        // Add the new key to the report
        keyCodes[i] = key;

        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/********************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyUp(uint8_t translationCode)
 ********************************************************************************
 * Summary: remove the code from standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyUp(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Find the key in the current standard report
    for (i=0; keyCodes[i] && i < KEY_MAX_KEYS_IN_STD_REPORT; i++)
    {
        if (keyCodes[i] == key)
        {
            // Found it. Remove it by shifting it!
            do
            {
                keyCodes[i] = keyCodes[i+1];
                // over sized? if so, we copied junk
                if (++i == KEY_MAX_KEYS_IN_STD_REPORT)
                {
                    keyCodes[--i] = 0; // replace junk data with 0
                }
            }
            while (keyCodes[i]);
            keyRpt.stdRpt_changed = TRUE;
        }
    }
}

/********************************************************************************
 * Function Name: wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
 ********************************************************************************
 * Summary: Set or Clear the bit identified in bitMask within the byte
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  TRUE -- bit is changed
 *  FALSE -- no change
 *
 *******************************************************************************/
static wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
{
    uint8_t bits = *buf;

    if (set)
    {
        *buf |= bitMask;
    }
    else
    {
        *buf &= ~bitMask;
    }
    return bits != *buf;
}

/********************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtModKey(uint8_t down, uint8_t translationCode)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtModKey(uint8_t set, uint8_t translationCode)
{
    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.stdRpt.modifierKeys, set, translationCode))
    {
        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/********************************************************************************
 * Function Name: void KeyRpt_bitRptProcEvtKey(uint8_t down, uint8_t bitPos)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  set -- TRUE then set the bit, otherwise, clear the bit
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_bitRptProcEvtKey(uint8_t set, uint8_t bitPos)
{
    uint8_t idx = bitPos / 8;
    uint8_t bitMask = (1<< (bitPos % 8));

    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.bitMappedReport.bitMappedKeys[idx], set, bitMask))
    {
        // Flag that the standard key report has changed
        keyRpt.bitMapped_changed = TRUE;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles func lock key events. Func-lock events are ignored
/// during recovery and in boot mode. On func-lock down, it performs
/// the following actions:
///     - It toggles the func lock state and clears the toggleStateOnKeyUp flag
///       By default, func lock state will not be toggled when the key goes up
///       unless this flag is cleared. Typically, this flags is set if
///       a func-lock dependent key is detected while func-lock is down
///     - It updates the func lock report with the current func-lock state
///       but does not send it
/// On func-lock up, it performs the following actions:
///     - If the toggleStateOnKeyUp flag is set, it toggles func-lock state
///       and updates the func lock report with the new state and event flag.
///       It does not send the report
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param translationCode associated with the func-lock key. Unused
/////////////////////////////////////////////////////////////////////////////////
static void KeyRpt_funcLockProcEvtKey(uint8_t upDown)
{
    // Process the event only if we are not in recovery
    if (!app.recoveryInProgress && app.protocol == PROTOCOL_REPORT)
    {
        // Check if this is a down key or up key
        if (upDown == KEY_DOWN)
        {
            key_rpts.funcLockReport.status = key_rpts.funcLockReport.status ? FUNC_LOCK_KEY_UP : FUNC_LOCK_KEY_DOWN;
            keyRpt.funcLock_changed = TRUE;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// This function handles sleep key events. It updates the sleep report with
/// the new value of the sleep bit.
///
/// \param upDownFlag indicates whether the key went up or down
/// \param keyCode scan code of this key
/// \param slpBitMask location of the sleep bit in the sleep report
/////////////////////////////////////////////////////////////////////////////////
static void KeyRpt_slpRptProcEvtKey(uint8_t upDown, uint8_t slpBitMask)
{
    // Check if this is a down key or up key
    if (upDown == KEY_DOWN)
    {
        key_rpts.sleepReport.sleepVal ^= slpBitMask;
        keyRpt.sleep_changed = TRUE;
    }
}


/********************************************************************************
 * Function Name: void KeyRpt_procEvtUserDefinedKey(void)
 ********************************************************************************
 * Summary: User defined key event handling
 *          Not used.
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_procEvtUserDefinedKey(uint8_t down, uint8_t translationCode)
{
}

/********************************************************************************
 * Function Name: void key_keyEvent(void)
 ********************************************************************************
 * Summary: key event handling
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
wiced_bool_t key_procEvtKey(uint8_t keyCode, uint8_t keyDown)
{
    // Check if we have a valid key
    if (keyCode < KEY_TABLE_SIZE)
    {
        uint8_t keyValue = kbKeyConfig[keyCode].translationValue;

        // Depending on the key type, call the appropriate function for handling
        // Pass unknown key types to user function
        switch(kbKeyConfig[keyCode].type)
        {
            case KEY_TYPE_STD:
                // Processing depends on whether the event is an up or down event
                keyDown ? KeyRpt_stdRptProcEvtKeyDown(keyValue) : KeyRpt_stdRptProcEvtKeyUp(keyValue);
                break;
            case KEY_TYPE_MODIFIER:
                KeyRpt_stdRptProcEvtModKey(keyDown, keyValue);
                break;
            case KEY_TYPE_BIT_MAPPED:
                KeyRpt_bitRptProcEvtKey(keyDown, keyValue);
                break;
            case KEY_TYPE_SLEEP:
                KeyRpt_slpRptProcEvtKey(keyDown, keyValue);
                break;
            case KEY_TYPE_FUNC_LOCK:
                KeyRpt_funcLockProcEvtKey(keyDown);
                break;
            case KEY_TYPE_NONE:
                // do nothing
                break;
            default:
                KeyRpt_procEvtUserDefinedKey(keyDown, keyValue);
                break;
        }
    }
    // Check if we have an end of scan cycle event
    else if (keyCode == END_OF_SCAN_CYCLE)
    {
        key_send();
    }
    else
    {
        // key index is out of range
        return FALSE;
    }
    return TRUE;
}

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
void key_send()
{
    if (keyRpt.stdRpt_changed)
    {
        hidd_link_send_report(&key_rpts.stdRpt, sizeof(KeyboardStandardReport));
        keyRpt.stdRpt_changed = FALSE;
    }
    if (keyRpt.bitMapped_changed)
    {
        hidd_link_send_report(&key_rpts.bitMappedReport, sizeof(KeyboardBitMappedReport));
        keyRpt.bitMapped_changed = FALSE;
    }
    if (keyRpt.funcLock_changed)
    {
        hidd_link_send_report(&key_rpts.funcLockReport, sizeof(KeyboardFuncLockReport));
        keyRpt.funcLock_changed = FALSE;
    }
    if (keyRpt.sleep_changed)
    {
        hidd_link_send_report(&key_rpts.sleepReport, sizeof(KeyboardSleepReport));
        keyRpt.sleep_changed = FALSE;
    }
    if (keyRpt.bitMapped_changed)
    {
        hidd_link_send_report(&key_rpts.scrollReport, sizeof(KeyboardMotionReport));
        keyRpt.bitMapped_changed = FALSE;
    }
#ifdef SUPPORT_CODE_ENTRY
    if (keyRpt.pin_changed)
    {
        hidd_link_send_report(&key_rpts.pinReport, sizeof(KeyboardPinEntryReport));
        keyRpt.pin_changed = FALSE;
    }
#endif
}

#ifdef SUPPORT_CODE_ENTRY
void key_pinReport(uint8_t code)
{
    key_rpts.pinReport.reportCode = code;
    keyRpt.pin_changed = TRUE;
}
#endif
/********************************************************************************
 * Function Name: void key_clear(wiced_bool_t sendRpt)
 ********************************************************************************
 * Summary: Clear all reports to default value
 *
 * Parameters:
 *  sendRpt -- When TRUE, after clear, send standard report (to indicate all keys are up)
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_clear(wiced_bool_t sendRpt)
{
    // clear report data
    memset(&key_rpts.stdRpt.modifierKeys, 0, sizeof(KeyboardStandardReport)-1);
    memset(&key_rpts.bitMappedReport.bitMappedKeys, 0, sizeof(KeyboardBitMappedReport)-1);
    memset(&key_rpts.scrollReport.motionAxis0, 0, sizeof(KeyboardMotionReport)-1);
#ifdef SUPPORT_CODE_ENTRY
    memset(&key_rpts.pinReport.reportCode, 0, sizeof(KeyboardPinEntryReport)-1);
#endif
    // mark if we need to generate report
    keyRpt.stdRpt_changed = keyRpt.bitMapped_changed = sendRpt;

    key_send();
}

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
void key_sendRollover()
{
    KeyboardStandardReport  rolloverRpt = {RPT_ID_IN_STD_KEY, 0, 0, {CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER}};
    // Tx rollover report
    WICED_BT_TRACE("\nRollOverRpt");
    hidd_link_send_report(&rolloverRpt, sizeof(KeyboardStandardReport));
}

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
                     uint16_t payloadSize)
{
    if (reportType == WICED_HID_REPORT_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID is in the payload
        if (payloadSize >= 1)
        {
            // Demux on report ID
            if(reportId == RPT_ID_OUT_KB_LED)
            {
                key_rpts.ledReport.ledStates = *(uint8_t *) payload;
//                WICED_BT_TRACE("\nKB LED report %d", key_rpts.ledReport.ledStates);
#if LED_SUPPORT
                key_rpts.ledReport.ledStates & 0x2 ? hidd_led_on(LED_CAPS) : hidd_led_off(LED_CAPS);
#endif
                return TRUE;
            }
        }
    }
    return FALSE;
}
#endif // SUPPORT_KEY_REPORT
