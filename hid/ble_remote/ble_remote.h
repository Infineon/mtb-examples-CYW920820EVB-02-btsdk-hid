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
 * BLE Remote control
 *
 * This file provides definitions and function prototypes for BLE remote control
 * device
 *
 */

#ifndef __BLEREMOTE_H__
#define __BLEREMOTE_H__
#ifdef SUPPORT_IR
#include "ir/bleapp_appirtx.h"
#endif // SUPPORT_IR
#ifdef ENABLE_ADC_AUDIO_ENHANCEMENTS
#include "adc.h"
#endif // ENABLE_ADC_AUDIO_ENHANCEMENTS
#include "interrupt.h"
#include "wiced_hidd_micaudio.h"
#include "wiced_hidd_lib.h"
#include "hidd_lib.h"
#include "wiced_platform.h"

extern const uint8_t blehid_db_data[];
extern const uint16_t blehid_db_size;
extern const attribute_t blehid_gattAttributes[];
extern const uint16_t blehid_gattAttributes_size;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[];
extern const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
extern wiced_bt_device_link_keys_t  blehostlist_link_keys;
extern uint16_t blehostlist_flags;
extern wiced_platform_led_config_t platform_led[];
extern wiced_platform_gpio_t platform_gpio_pins[];
extern uint8_t bleremote_key_std_rpt[];
extern uint8_t bleremote_bitmap_rpt[];

/////////////////////////////////////////////////////////////////////////////////
#ifdef ATT_MTU_SIZE_180
 #define AUDIO_MTU_SIZE 180
#else
 #define AUDIO_MTU_SIZE 20
#endif

/*******************************************************************************
* Types and Defines
*******************************************************************************/

#if is_20735Family
 #define NUM_KEYSCAN_ROWS    5  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    4  // Num of Cols in keyscan matrix
#else
 #define NUM_KEYSCAN_ROWS    7  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    7  // Num of Cols in keyscan matrix
#endif

#define STD_KB_REPORT_ID            1
#define BITMAPPED_REPORT_ID         2
#define BATTERY_REPORT_ID           3
#define RPT_ID_MOUSE                8
#define MEDIA_REPORT_ID           0xa
#define RPT_ID_IN_ABS_XY         0x20
#define RPT_ID_VOICE_DATA        0xF7
#define RPT_ID_VOICE_CTL         0xF8
#define GPIO_TOUCHPAD_OFF           0
#define GPIO_TOUCHPAD_ON            1

/// Maximum number of keys supported by our HID
#define KB_MAX_KEYS 160

/// Maximum number of remote reports supported by our HID
#define REMOTE_MAX_USER_DEFINED_RPT_TYPE 8

/// Maximum size of remote translation code
#define REMOTE_MAX_TRANSLATION_CODE_SIZE 2

/// Maximum number of keys in a standard key report. Technically the report is
/// limited to 6 keys. A BLE ATT can hold 23 bytes. We'll
/// only use 6. The length of a non-boot mode report will be set through the config
/// record
#define KEYRPT_MAX_KEYS_IN_STD_REPORT    6
#define KEYRPT_BIT_MODIFIER_LEN          2
#define KEYRPT_LEN (KEYRPT_MAX_KEYS_IN_STD_REPORT+KEYRPT_BIT_MODIFIER_LEN)

/// Maximum number of bytes in the bit-mapped key report structure.
/// A BLE ATT can hold 23 bytes.
#define KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT   11

/// 11 bytes allow 88 keys in the bit mapped key report
#define KEYRPT_NUM_KEYS_IN_BIT_MAPPED_REPORT    ((KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT)*8)


#define REMOTERPT_MAX_BYTES_IN_REPORT 8

/// Rollover code
#define KEYRPT_CODE_ROLLOVER        0x01

//defined the maximum number of different client configuration notifications
#define MAX_NUM_CLIENT_CONFIG_NOTIF     16
//bit mask for different client configuration notifications
#define KBAPP_CLIENT_CONFIG_NOTIF_STD_RPT               (0x02)
#define KBAPP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT        (0x04)
#define KBAPP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT  (0x10)
#define KBAPP_CLIENT_CONFIG_NOTIF_BATTERY_RPT           (0x20)
#define KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT             (0x40)
#define KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT        (0x80)
#ifdef ANDROID_AUDIO
 #define KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_RX_BIT_OFFSET    6
 #define KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_CTL_BIT_OFFSET   7
 #define KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_RX     KBAPP_CLIENT_CONFIG_NOTIF_VOICE_RPT
 #define KBAPP_CLIENT_CONFIG_NOTIF_ATV_VOICE_CTL    KBAPP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT
#endif // ANDROID_AUDIO
#ifdef SUPPORT_TOUCHPAD
 #define KBAPP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT       (0x0100)
#endif // SUPPORT_TOUCHPAD

#define KBAPP_CLIENT_CONFIG_NOTIF_NONE              (0)

#pragma pack(1)
/// Keyboard application configuration
typedef PACKED struct
{
    /// ID of the standard report
    uint8_t stdRptID;

    /// Maximum number of keys in standard key report. Should be set to 6
    uint8_t maxKeysInStdRpt;

    /// Report ID for the bit mapped report
    uint8_t bitReportID;

    /// Number of bit mapped keys. Size of the bit report is automatically calculated from this value
    /// according to the following formula:
    ///     report size = ((num bit mapped keys) + 7)/8
    uint8_t numBitMappedKeys;

    /// Report ID for the bit mapped report
    uint8_t sleepReportID;

    /// Report ID of the pin entry report
    uint8_t pinReportID;

    /// Report ID of the LED (output) report
    uint8_t ledReportID;

    /// Default LED state. Note that the default implementation does not tie the LED value to physical LEDs
    uint8_t defaultLedState;

    /// Scan code of the connect button
    uint8_t connectButtonScanIndex;

    /// After an error has occurred, events from multiple poll cycles are combined to ensure that transient
    /// events are not generated. The count below specifies the recovery period in poll cycles.
    uint8_t recoveryPollCount;

    /// HW fifo threshold to stop generating idle rate reports. Idle rate report will be generated
    /// as long as the number of packets in the HW fifo is below this number
    uint8_t hwFifoThresholdForIdleRateReports;

    /// This parameter defines the rate at which a rollover report is generated when an error state (ghost or overflow) is
    /// maintained for long periods of time. The rate is in BT clock periods. If set to 0, it disables regeneration of
    /// the rollover report.
    uint16_t repeatRateInBTClocksForRolloverRpt;

    /// Rollover reports will only be repeated as long as the number of packets in the HW fifo is less than this threshold
    uint8_t hwFifoThresholdForRolloverRepeats;

    /// Report ID for func-lock reports
    uint8_t funcLockReportID;

    /// Default func lock state
    uint8_t defaultFuncLockState;

    /// Scroll report ID
    uint8_t scrollReportID;

    /// Length of scroll report
    uint8_t scrollReportLen;

    /// Negate scroll data.
    uint8_t negateScroll;

    /// Scale values for scroll wheel data. Should be set to zero if no scaling is desired
    /// Scaling always divides the input data, i.e. its a shift right.
    uint8_t scrollScale;

    /// Maximum number of ticks for which fractional scroll wheel motion data is kept,
    /// i.e. if no additional motion is detected, remaining fractional data is discarded.
    /// If set to 0, data is never discarded. If scroll scaling is not used, should be set to
    /// 0 to improve execution efficiency.
    uint8_t pollsToKeepFracScrollData;

    /// Flag indicating whether multiple scroll events should be combined into a single report
    /// Note that this will not combine any other type of event with scroll info
    uint8_t scrollCombining;

    /// Size of each element in the event queue. Note: This has to be at least as large as the
    /// largest event that the app will handle
    uint8_t maxEventSize;

    /// Maximum number of events that the event queue can hold.
    uint8_t maxEventNum;
}KbAppConfig;

/// Keyboard Key Config
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

/// Remote user defined report configuration
typedef PACKED struct
{
    /// Remote user defined report ID
    uint8_t rptID;

    /// Size of the key translation code
    uint8_t translationCodeSize;
}RemoteUserDefinedReportConfig;

/// Remote Key Translation Code
typedef PACKED struct
{
    uint8_t translationValue[REMOTE_MAX_TRANSLATION_CODE_SIZE];
}RemoteKeyTranslationCode;

/// Remote application configuration
typedef PACKED struct
{
    /// the default lpm index for mode "HIGH"
    uint8_t default_lpm_idx;

    /// the lpm index for motion
    uint8_t motion_lpm_idx;

    /// the lpm index for voice
    uint8_t voice_lpm_idx;

    /// report ID for motion data
    uint8_t motionRptID;

    /// Scan code of the IR button
    uint8_t IR_ButtonScanIndex;

    /// Scan code of the Motion START button
    uint8_t MotionStart_ButtonScanIndex;

    /// Scan code of the Motion STOP button
    uint8_t MotionStop_ButtonScanIndex;

    /// Scan code of the Voice button
    uint8_t Voice_ButtonScanIndex;

    /// delay sending audio time period in ms
    uint16_t  audio_delay;

    /// audio mode
    uint8_t    audio_mode;

    /// gain of the audio codec
    uint8_t    audio_gain;

    /// boost of the audio codec
    uint8_t    audio_boost;

    /// Maximum number of data bytes in remote report
    uint8_t maxBytesInRemoteRpt;

    /// Number of different typpes of remote reports
    uint8_t numOfRemoteRpt;

    /// Maximum sample number read in one slot callback
    uint8_t maxSampleInOneSlot;

    /// Remote user defined report configuration
    RemoteUserDefinedReportConfig remoteUserDefinedReportConfig[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Remote Key Translation Code
    RemoteKeyTranslationCode remoteKeyTranslationCode[KB_MAX_KEYS];
}RemoteAppConfig;
#pragma pack()

/// KB USB usages
enum UsbUsage
{
    USB_USAGE_NO_EVENT=0,
    USB_USAGE_ROLLOVER=1,
    USB_USAGE_POST_FAIL=2,
    USB_USAGE_UNDEFINED_ERROR=3,

    USB_USAGE_A=4,
    USB_USAGE_B=5,
    USB_USAGE_C=6,
    USB_USAGE_D=7,
    USB_USAGE_E=8,
    USB_USAGE_F=9,
    USB_USAGE_G=10,
    USB_USAGE_H=11,
    USB_USAGE_I=12,
    USB_USAGE_J=13,
    USB_USAGE_K=14,
    USB_USAGE_L=15,
    USB_USAGE_M=16,
    USB_USAGE_N=17,
    USB_USAGE_O=18,
    USB_USAGE_P=19,
    USB_USAGE_Q=20,
    USB_USAGE_R=21,
    USB_USAGE_S=22,
    USB_USAGE_T=23,
    USB_USAGE_U=24,
    USB_USAGE_V=25,
    USB_USAGE_W=26,
    USB_USAGE_X=27,
    USB_USAGE_Y=28,
    USB_USAGE_Z=29,

    USB_USAGE_1=30,
    USB_USAGE_2=31,
    USB_USAGE_3=32,
    USB_USAGE_4=33,
    USB_USAGE_5=34,
    USB_USAGE_6=35,
    USB_USAGE_7=36,
    USB_USAGE_8=37,
    USB_USAGE_9=38,
    USB_USAGE_0=39,

    USB_USAGE_ENTER=40,
    USB_USAGE_ESCAPE=41,
    USB_USAGE_BACKSPACE=42,
    USB_USAGE_TAB=43,
    USB_USAGE_SPACEBAR=44,
    USB_USAGE_MINUS=45,
    USB_USAGE_EQUAL=46,
    USB_USAGE_LEFT_BRACKET=47,
    USB_USAGE_RIGHT_BRACKET=48,
    USB_USAGE_BACK_SLASH=49,

    USB_USAGE_NON_US_HASH=50,
    USB_USAGE_SEMICOLON=51,
    USB_USAGE_QUOTE=52,
    USB_USAGE_ACCENT=53,
    USB_USAGE_COMMA=54,
    USB_USAGE_STOP_AND_GREATER=55,
    USB_USAGE_SLASH=56,
    USB_USAGE_CAPS_LOCK=57,
    USB_USAGE_F1=58,
    USB_USAGE_F2=59,

    USB_USAGE_F3=60,
    USB_USAGE_F4=61,
    USB_USAGE_F5=62,
    USB_USAGE_F6=63,
    USB_USAGE_F7=64,
    USB_USAGE_F8=65,
    USB_USAGE_F9=66,
    USB_USAGE_F10=67,
    USB_USAGE_F11=68,
    USB_USAGE_F12=69,

    USB_USAGE_PRINT_SCREEN=70,
    USB_USAGE_SCROLL_LOCK=71,
    USB_USAGE_PAUSE=72,
    USB_USAGE_INSERT=73,
    USB_USAGE_HOME=74,
    USB_USAGE_PAGE_UP=75,
    USB_USAGE_DELETE=76,
    USB_USAGE_END=77,
    USB_USAGE_PAGE_DOWN=78,
    USB_USAGE_RIGHT_ARROW=79,

    USB_USAGE_LEFT_ARROW=80,
    USB_USAGE_DOWN_ARROW=81,
    USB_USAGE_UP_ARROW=82,
    USB_USAGE_NUM_LOCK=83,
    USB_USAGE_KP_SLASH=84,
    USB_USAGE_KP_ASTERISK=85,
    USB_USAGE_KP_MINUS=86,
    USB_USAGE_KP_PLUS=87,
    USB_USAGE_KP_ENTER=88,
    USB_USAGE_KP_1=89,

    USB_USAGE_KP_2=90,
    USB_USAGE_KP_3=91,
    USB_USAGE_KP_4=92,
    USB_USAGE_KP_5=93,
    USB_USAGE_KP_6=94,
    USB_USAGE_KP_7=95,
    USB_USAGE_KP_8=96,
    USB_USAGE_KP_9=97,
    USB_USAGE_KP_0=98,
    USB_USAGE_KP_DOT=99,

    USB_USAGE_NON_US_BACK_SLASH=100,
    USB_USAGE_APPLICATION=101,
    USB_USAGE_POWER=102,
    USB_USAGE_KP_EQUAL=103,
    USB_USAGE_F13=104,
    USB_USAGE_F14=105,
    USB_USAGE_F15=106,
    USB_USAGE_F16=107,
    USB_USAGE_F17=108,
    USB_USAGE_F18=109,

    USB_USAGE_F19=110,
    USB_USAGE_F20=111,
    USB_USAGE_F21=112,
    USB_USAGE_F22=113,
    USB_USAGE_F23=114,
    USB_USAGE_F24=115,
    USB_USAGE_EXECUTE=116,
    USB_USAGE_HELP=117,
    USB_USAGE_MENU=118,
    USB_USAGE_SELECT=119,

    USB_USAGE_STOP=120,
    USB_USAGE_AGAIN=121,
    USB_USAGE_UNDO=122,
    USB_USAGE_CUT=123,
    USB_USAGE_COPY=124,
    USB_USAGE_PASTE=125,
    USB_USAGE_FIND=126,
    USB_USAGE_MUTE=127,
    USB_USAGE_VOL_UP=128,
    USB_USAGE_VOL_DOWN=129,

    USB_USAGE_LOCKING_CAPS_LOCK=130,
    USB_USAGE_LOCKING_NUM_LOCK=131,
    USB_USAGE_LOCKING_SCROLL_LOCK=132,
    USB_USAGE_KP_COMMA=133,
    USB_USAGE_KP_EQUAL_AS400=134,
    USB_USAGE_INTL_1=135,
    USB_USAGE_INTL_2=136,
    USB_USAGE_INTL_3=137,
    USB_USAGE_INTL_4=138,
    USB_USAGE_INTL_5=139,

    USB_USAGE_INTL_6=140,
    USB_USAGE_INTL_7=141,
    USB_USAGE_INTL_8=142,
    USB_USAGE_INTL_9=143,
    USB_USAGE_LANG_1=144,
    USB_USAGE_LANG_2=145,
    USB_USAGE_LANG_3=146,
    USB_USAGE_LANG_4=147,
    USB_USAGE_LANG_5=148,
    USB_USAGE_LANG_6=149,

    USB_USAGE_LANG_7=150,
    USB_USAGE_LANG_8=151,
    USB_USAGE_LANG_9=152,
    USB_USAGE_ALT_ERASE=153,
    USB_USAGE_SYS_REQ=154,
    USB_USAGE_CANCEL=155,
    USB_USAGE_CLEAR=156,
    USB_USAGE_PRIOR=157,
    USB_USAGE_RETURN=158,
    USB_USAGE_SEPARATOR=159,

    USB_USAGE_OUT=160,
    USB_USAGE_OPER=161,
    USB_USAGE_CLEAR_AGAIN=162,
    USB_USAGE_CRSEL=163,
    USB_USAGE_EXSEL=164,

    // Reserved 165-175

    USB_USAGE_KP_00=176,
    USB_USAGE_KP_000=177,
    USB_USAGE_THOUSANDS_SEPERATOR=178,
    USB_USAGE_DECIMAL_SEPERATOR=179,

    USB_USAGE_CURRENCY_UNIT=180,
    USB_USAGE_CURRENCY_SUB_UNIT=181,
    USB_USAGE_KP_LEFT_PAREN=182,
    USB_USAGE_KP_RIGHT_PAREN=183,
    USB_USAGE_KP_LEFT_CURLY_BRACE=184,
    USB_USAGE_KP_RIGHT_CURLY_BRACE=185,
    USB_USAGE_KP_TAB=186,
    USB_USAGE_KP_BACKSPACE=187,
    USB_USAGE_KP_A=188,
    USB_USAGE_KP_B=189,

    USB_USAGE_KP_C=190,
    USB_USAGE_KP_D=191,
    USB_USAGE_KP_E=192,
    USB_USAGE_KP_F=193,
    USB_USAGE_KP_XOR=194,
    USB_USAGE_KP_CARET=195,
    USB_USAGE_KP_PERCENT=196,
    USB_USAGE_KP_LESS_THAN=197,
    USB_USAGE_KP_GREATER_THAN=198,
    USB_USAGE_KP_AMPERSAND=199,

    USB_USAGE_KP_DOUBLE_AMPERSAND=200,
    USB_USAGE_KP_VERTICAL_BAR=201,
    USB_USAGE_KP_DOUBLE_VERTICAL_BAR=202,
    USB_USAGE_KP_COLON=203,
    USB_USAGE_KP_HASH=204,
    USB_USAGE_KP_SPACE=205,
    USB_USAGE_KP_AT=206,
    USB_USAGE_KP_EXCLAMATION=207,
    USB_USAGE_KP_MEM_STORE=208,
    USB_USAGE_KP_MEM_RECALL=209,

    USB_USAGE_KP_MEM_CLEAR=210,
    USB_USAGE_KP_MEM_ADD=211,
    USB_USAGE_KP_MEM_SUBTRACT=212,
    USB_USAGE_KP_MEM_MULTIPLY=213,
    USB_USAGE_KP_MEM_DIVIDE=214,
    USB_USAGE_KP_PLUS_MINUS=215,
    USB_USAGE_KP_CLEAR=216,
    USB_USAGE_KP_CLEAR_ENTRY=217,
    USB_USAGE_KP_BINARY=218,
    USB_USAGE_KP_OCTAL=219,

    USB_USAGE_KP_DECIMAL=220,
    USB_USAGE_KP_HEX=221,
    // 222-223 reserved
    USB_USAGE_LEFT_CTL=224,
    USB_USAGE_LEFT_SHIFT=225,
    USB_USAGE_LEFT_ALT=226,
    USB_USAGE_LEFT_GUI=227,
    USB_USAGE_RIGHT_CTL=228,
    USB_USAGE_RIGHT_SHIFT=229,

    USB_USAGE_RIGHT_ALT=230,
    USB_USAGE_RIGHT_GUI=231
};

/// Modifier keys bit masks
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

#if !is_20735Family
enum bitmapped_key
{
    BITMAP_AC_BACK,   // bit 0
    BITMAP_AC_HOME,   // bit 1
    BITMAP_AC_SEARCH, // bit 2
    BITMAP_MENU_LEFT, // bit 3
    BITMAP_MENU_UP,   // bit 4
    BITMAP_MENU_DOWN, // bit 5
    BITMAP_VOL_DOWN,  // bit 6
    BITMAP_VOL_UP,    // bit 7

    BITMAP_MENU_RIGHT,// bit 8
    BITMAP_MENU_PICK, // bit 9
    BITMAP_PLAY_PAUSE,// bit 10
    BITMAP_MUTE,      // bit 11
    BITMAP_FAST_FORWD,// bit 12
    BITMAP_REWIND,    // bit 13
    BITMAP_CH_UP,     // bit 14
    BITMAP_CH_DOWN,   // bit 15

    BITMAP_CH,        // bit 16
    BITMAP_ODR_MOVIE, // bit 17
    BITMAP_PREVIOUS,  // bit 18 (Recall Last)
    BITMAP_POWER,     // bit 19
    BITMAP_SHOPPING,  // bit 20
    BITMAP_VIEW_TGL,  // bit 21
    BITMAP_NUMBER,    // bit 22
    BITMAP_NEXT_TRACK,// bit 23

    BITMAP_PREV_TRACK,// bit 24
    BITMAP_REVIEW,    // bit 25
    BITMAP_MENU,      // bit 26

};
#endif

/*** ALL EXTERNAL REPORTS MUST BE PACKED!
 *** (and don't forget the pack() afterwards) **/
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
    uint8_t    keyCodes[KEYRPT_MAX_KEYS_IN_STD_REPORT];
}KeyboardStandardReport;

/// Battery key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    uint8_t    level[1];
}KeyboardBatteryReport;

/// Bit mapped key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Bit mapped keys
    uint8_t    bitMappedKeys[KEYRPT_NUM_BYTES_IN_BIT_MAPPED_REPORT];
}KeyboardBitMappedReport;


/// Keyboard output report. Sets the LED state
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// State of various LEDs
    uint8_t    ledStates;
}KeyboardLedReport;


/// Remote report structure
typedef struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Key array.
    uint8_t    keyCodes[REMOTERPT_MAX_BYTES_IN_REPORT];
}RemoteReport;
#pragma pack()

#define GPIO_TOUCHPAD_INT_GPIO     WICED_PLATFORM_GPIO_5                           // interrupt pin used for touchpad interrupt
#define GPIO_TOUCHPAD_INT          (platform_gpio_pins[GPIO_TOUCHPAD_INT_GPIO].gpio_pin)
#define GPIO_RSTN_TP_GPIO          WICED_PLATFORM_GPIO_4                           // touchpad reset_N pin
#define GPIO_RSTN_TP               (platform_gpio_pins[GPIO_RSTN_TP_GPIO].gpio_pin)
#ifdef SUPPORT_TOUCHPAD
 #include "touchpad/touchPad.h"
 #define NUM_ROWS                   5
 #define NUM_COLS                   5  // 4 actual keyscan column + 1 extra column for touch pad virtual keys = 5 columns
 #define TOUCHPAD_BUTTON_KEYINDEX   4 // ENTER
 #define VKEY_INDEX_CENTER          TOUCHPAD_BUTTON_KEYINDEX
 #define VKEY_INDEX_RIGHT           (NUM_ROWS * (NUM_COLS-1))
 #define VKEY_INDEX_LEFT            (VKEY_INDEX_RIGHT+1)
 #define VKEY_INDEX_DOWN            (VKEY_INDEX_RIGHT+2)
 #define VKEY_INDEX_UP              (VKEY_INDEX_RIGHT+3)
#endif /* SUPPORT_TOUCHPAD */

#define LED_ON  0
#define LED_OFF 1
#if 0
 #define LED_RED  (*platform_led[0].gpio)
 #define LED_BLUE (*platform_led[1].gpio)
#else
  #define LED_RED  26
  #define LED_BLUE 27
#endif
#define LED_LE_LINK LED_BLUE

#ifdef SUPPORTING_FINDME
#include "pwm.h"
#define FINDME_ALERT_TYPE        ALERT_BUZ_LED  // ALERT_LED, ALERT_BUZ

#define FINDME_LED LED_RED

// findMe BUZ alert config
typedef struct
{
    uint8_t freq;         // pwm freq
    uint16_t init_value;  // pwm init value
    uint16_t toggle_val;  // pwm toggle value
    uint16_t buz_on_ms;   // buz on duration
    uint16_t buz_off_ms;  // buz on duration
    uint16_t repeat_num;  // repeat num
}AppBuzAlertConfig;

// findMe LED alert config
typedef struct
{
    uint16_t led_on_ms;    // led on duration
    uint16_t led_off_ms;   // led on duration
    uint16_t repeat_num;   // repeat num
}AppLedAlertConfig;


//Find me Alert mode
enum ble_findme_alert_level
{
    NO_ALERT                        = 0,
    MILD_ALERT                      = 1,
    HIGH_ALERT                      = 2,
    UNDIRECTED_DISCOVERABLE_ALERT   = 3,
};

// Find me Alert type
#define ALERT_NONE    0x00
#define ALERT_BUZ     0x01
#define ALERT_LED     0x02

#define ALERT_BUZ_LED (ALERT_BUZ | ALERT_LED)
// valid id's are 0 thru 5, corresponding to P26 thru p31
typedef enum {
    BUZ_ID0 = PWM0,
    BUZ_ID1 = PWM1,
    BUZ_ID2 = PWM2,
    BUZ_ID3 = PWM3,
    BUZ_ID4 = PWM4,
    BUZ_ID5 = PWM5,
} tBuzId;
#define FINDME_BUZ_PWM_ID        BUZ_ID2
// app Alert ID
enum
{
    APP_ALERT_PATTERN_MILD_ID     = 0x00,
    APP_ALERT_PATTERN_HIGH_ID     = 0x01,
    APP_ALERT_PATTERN_MAX_ID      = 0x02,
};

typedef struct
{
    uint8_t activeAlterLevel;   // active immediate alert level.
    uint8_t alertType;          // alert type(LED or BUZ or both)

    // buz alert state
    tBuzId buz_id;             // id for buz
    uint8_t buz_alert_active;   // buz alert is on playing
    uint8_t buz_on;             // app buz on state
    uint8_t buz_pattern_id;     // active buz pattern ID (mild or High)
    uint16_t buz_repeat;        // app buz repeat num
    uint16_t buz_timeout_sec;   // buz timeout in sec part
    uint16_t buz_timeout_ms;    // buz timeout in ms part
    uint16_t buz_timer_call_per_sec; // buz tick divider for ms timer

    // led alert state
    uint8_t led_alert_active;   // app led alert  is on playing
    uint8_t led_on;             // app led on state
    uint8_t led_pattern_id;     // active pattern ID (mild or High)
    uint16_t led_repeat;        // app led repeat num
    uint16_t led_timeout_sec;   // led timeout in sec part
    uint16_t led_timeout_ms;    // led timeout in ms part
    uint16_t led_timer_call_per_sec; // led tick divider for ms timer
}tAppFindmeState;

typedef struct
{
    // Alert Buz config
    AppBuzAlertConfig alertBuzCfg[APP_ALERT_PATTERN_MAX_ID];

    // Alert Led config
    AppLedAlertConfig alertLedCfg[APP_ALERT_PATTERN_MAX_ID];
}tAppAlertConfig;

#endif // SUPPORTING_FINDME

/// Key types. Used to direct key codes to the relevant key processing function
enum KeyType
{
    /// Represents no key. This should not occur normally
    KEY_TYPE_NONE=0,

    /// Represents a standard key. The associated translation code represents the reported value
    /// of this key
    KEY_TYPE_STD=1,

    /// Represents a modifier key. The associated translation value indicates which bit
    /// in the modifier key mask is controlled by this key
    KEY_TYPE_MODIFIER=2,

    /// Represents a bit mapped key in the bit mapped report. The associated translation value
    /// provides the row col of the bit which represents this key
    KEY_TYPE_BIT_MAPPED=3,

    /// The sleep key
    KEY_TYPE_SLEEP=4,

    /// The function lock key
    KEY_TYPE_FUNC_LOCK=5,

    /// Function lock dependent keys. These keys act like bit mapped keys when function lock is on
    /// and standard keys when function lock is off or the keyboard is in boot mode.
    KEY_TYPE_FUNC_LOCK_DEP=6,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_0=16,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_1=17,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_2=18,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_3=19,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_4=20,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_5=21,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_6=22,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_DEF_7=23
};


typedef struct
{
    /// Standard key report
    KeyboardStandardReport  stdRpt;

    /// Battery level report
    KeyboardBatteryReport     batRpt;

    /// Standard rollover report
    KeyboardStandardReport rolloverRpt;

    /// Output LED report. Maintained for GET_REPORT
    KeyboardLedReport ledReport;

    /// Bit mapped key report
    KeyboardBitMappedReport bitMappedReport;

    /// Remote reports
    RemoteReport remoteRpt[REMOTE_MAX_USER_DEFINED_RPT_TYPE];


    // Report change flags

    /// Flag indicating that the std report has been changed since it was last sent
    uint8_t stdRptChanged;

    /// Flag indicating that the bit mapped report has been changed since it was last sent
    uint8_t bitRptChanged;

    /// Flag indicating that the remote reports have been changed since they were last sent
    uint8_t remoteRptChanged[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    // Additional report related attributes

    /// Number of keys in the current standard report
    uint8_t keysInStdRpt;

    /// Number of down modifier keys in the standard report
    uint8_t modKeysInStdRpt;

    /// Size of the standard report. Arrived at by adding 4 bytes (report header, report ID,
    /// modifier byte, and reserved byte) to the maximum number of keys in a standard report
    uint8_t stdRptSize;

    /// Size of the Battery report.
    uint8_t batRptSize;

    /// Number of keys in the current bit mapped report
    uint8_t keysInBitRpt;

    /// Size of the bit mapped report. Includes header and report ID
    uint8_t bitReportSize;

    /// Number of keys in the current remote reports
    uint8_t bytesInRemoteRpt[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Size of the remote reports
    uint8_t remoteRptSize[REMOTE_MAX_USER_DEFINED_RPT_TYPE];

    /// Number of polls cycles left in the recovery period
    uint8_t recoveryInProgress;

    // Event structures

    /// Temporary used for creating key events
    HidEventKey kbKeyEvent;

    /// null Event.
    HidEventAny eventNULL;

    /// Temporary used for events
    HidEventUserDefine  voiceEvent;
    HidEventUserDefine  voiceCtrlEvent;
#ifdef SUPPORT_TOUCHPAD
    HidEventTouchpad    touchpadEvent;
#endif // SUPPORT_TOUCHPAD
/// The event queue for use by app.
    wiced_hidd_app_event_queue_t appEventQueue;

#ifdef SUPPORT_AUDIO
    uint8_t audioStopEventInQueue;  //indicate if WICED_HIDD_RC_MIC_STOP_REQ event is in the event queue
    uint8_t micStopEventInQueue;    //indicate if WICED_HIDD_MIC_STOP event is in the event queue
    uint8_t audioPacketInQueue;
    uint8_t audiobutton_pressed;
#endif // SUPPORT_AUDIO

    uint8_t codecSettingMsg_type;
    uint8_t codecSettingMsg_dataCnt;
    uint8_t codecSettingMsg_dataBuffer[6];

    uint8_t pollSeqn;
    uint8_t keyInterrupt_On;
    uint8_t allowSDS;

} tRemoteAppState;

/// Connect button state
typedef enum
{
    CONNECT_BUTTON_UP,
    CONNECT_BUTTON_DOWN
}ConnectButtonPosition;


void bleremoteapp_create(void);
void bleremoteapp_pre_init(void);
void bleremoteapp_init(void);
void bleremoteapp_shutdown(void);
void bleremoteapp_pollReportUserActivity(void);
uint8_t bleremoteapp_pollActivityUser(void);
void bleremoteapp_pollActivityKey(void);

void bleremoteapp_flushUserInput(void);
void bleremoteapp_stdErrResp(void);
void bleremoteapp_procErrKeyscan(void);
void bleremoteapp_procErrEvtQueue(void);

void bleremoteapp_stdRptRolloverSend(void);
void bleremoteapp_bitRptSend(void);
void bleremoteapp_batRptSend(void);
void bleremoteapp_stdRptSend(void);
void bleremoteapp_remoteRptSend(uint8_t rptIndex);

void bleremoteapp_procEvtKey(void);
void bleremoteapp_stdRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_stdRptProcOverflow(void);
void bleremoteapp_stdRptProcEvtModKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);
void bleremoteapp_bitRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rowCol);

uint8_t bleremoteapp_findKeyInRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
uint8_t bleremoteapp_addKeytoRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
void bleremoteapp_removeKeyfromRemoteRpt(uint8_t rptIndex, uint8_t keyCode, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKeyDown(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKeyUp(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_remoteRptProcEvtKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t rptID, uint8_t translationCodeSize);
void bleremoteapp_procEvtUserDefinedKey(uint8_t upDownFlag, uint8_t keyCode, uint8_t translationCode);

void bleremoteapp_appActivityDetected(void *remApp);
void bleremoteapp_userKeyPressDetected(void* unused);


void bleremoteapp_batLevelChangeNotification(uint32_t newLevel);

void bleremoteapp_connectButtonHandler(ConnectButtonPosition connectButtonPosition);

void bleremoteapp_ledRptInit(void);
void bleremoteapp_stdRptRolloverInit(void);
void bleremoteapp_stdRptClear(void);
void bleremoteapp_bitRptClear(void);
void bleremoteapp_remoteRptClear(void);
void bleremoteapp_clearAllReports(void);

//audio
void bleremoteapp_pollActivityVoice(void);
void bleremoteApp_procEvtVoice(void);
void bleremoteapp_procEvtVoiceCtrl(uint8_t eventType);
void bleremoteapp_voiceModeSend(void);
void bleremoteapp_voiceReadCodecSetting(void);
void bleremoteapp_voiceWriteCodecSetting(void);


uint8_t bleremoteapp_pollActivitySensor(void);
void bleremoteapp_procEvtMotion(void);
#ifdef SUPPORT_TOUCHPAD
void bleremoteapp_procEvtTouchpad(void);
void   gpioActivityDetected(void * appData, uint8_t portPin);
uint8_t   pollTouchpadActivity(void);
uint8_t  handleTouchpadVirtualKey(HidEventKey * ke);
#endif // SUPPORT_TOUCHPAD


void bleremoteapp_txModifiedKeyReports(void);
void bleremoteapp_procEvtUserDefined(void);
void bleremoteapp_transportStateChangeNotification(uint32_t newState);

void bleremoteapp_setReport(wiced_hidd_report_type_t reportType,uint8_t reportId,void *payload,uint16_t payloadSize);


//gatt callback of client write (HID READ)
void bleremoteapp_ctrlPointWrite(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptStd(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptUserDefinedKey(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptVoice(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
void bleremoteapp_clientConfWriteRptVoiceCtrl(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
#ifdef SUPPORT_TOUCHPAD
void bleremoteapp_clientConfWriteRptTouchpad(wiced_hidd_report_type_t reportType, uint8_t reportId, void *payload, uint16_t payloadSize);
#endif // SUPPORT_TOUCHPAD
void bleremoteapp_updateClientConfFlags(uint16_t enable, uint16_t featureBit);
void bleremoteapp_updateGattMapWithNotifications(uint16_t flags);

uint32_t bleremoteapp_sleep_handler(wiced_sleep_poll_type_t type );
void bleremoteapp_aon_restore(void);

#ifdef SUPPORTING_FINDME
void bleremoteapp_findme_init(void);
uint8_t bleremoteapp_isAlertIdle(void);
void bleremoteapp_alertBuz_timeout(uint32_t unused);
void bleremoteapp_StartAlertBuzTimer(uint16_t timeout_ms);
void bleremoteapp_StopAlertBuzTimer(void);
void bleremoteapp_alertBuzFreq(uint8_t freq, uint16_t init_value, uint16_t toggle_val);
void bleremoteapp_alertBuzOn(uint8_t pwm_id);
void bleremoteapp_alertBuzOff(uint8_t pwm_id);
void bleremoteapp_alertBuzPlay(uint8_t pattern_id);
void bleremoteapp_alertBuzStop(void);
void bleremoteapp_alertLed_timeout(uint32_t unused);
void bleremoteapp_StartAlertLedTimer(uint16_t timeout_ms);
void bleremoteapp_StopAlertLedTimer(void);
void bleremoteapp_alertLedOn(void);
void bleremoteapp_alertLedOff(void);
void bleremoteapp_alertLedPlay(uint8_t pattern_id);
void bleremoteapp_alertLedStop(void);
int bleremoteapp_findme_writeCb(void *p);
#endif // SUPPORTING_FINDME

#endif // __BLEREMOTE_H__
