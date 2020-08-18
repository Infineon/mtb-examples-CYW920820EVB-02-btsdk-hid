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
* Keyboard Application
*
*
* Keyboard
*
* The exmaple can be compile to BT keyboard, LE keyboard, or Dual Mode (BT classic and LE) Keyboard
* application to run on a single chip SoC.  It provides a turnkey solution
* using on-chip keyscan HW component and is compliant with HID over GATT Profile (HOGP) and HID Profile.
*
* During initialization the app registers with LE and BT stack, WICED HID Device Library and
* keyscan HW to receive various notifications including bonding/pairing complete, (HIDD) connection
* status change, peer GATT request/commands, HIDD events and interrupts for key pressed/released.
* If not paired before, pressing any key will start LE advertising and enter discoverable, i.e. inquiry scan and page scan enabled.
* When device is successfully bonded, the app saves bonded host's information in the NVRAM and stops LE advertising and stops
* inquiry scan and page scan.
* If the bonded peer device is using BT classic, the dual mode keyboard now acts as a BT classic keyboard.
* If the bonded peer device is using LE, the dual mode keyboard now acts as a LE keyboard.
* When user presses/releases key, a key report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is below shutdown voltage, device will critical shutdown.
* Host can send LED report to the device to control LED.
*
* Features demonstrated
*  - GATT database, SDP database and Device configuration initialization
*  - Registration with LE and BT stack for various events
*  - Sending HID reports to the host
*  - Processing write requests from the host
*  - Low power management
*  - Over the air firmware update (OTAFWU) via LE
*
* To demonstrate the app, walk through the following steps.
* 1. Plug the CYW920739FCBGA120 board or 20739B1 Keyboard HW into your computer
* 2. Build and download the application (to the EVAL board or Keyboard HW) as below:
*    demo.hid.dual_mode_keyboard-CYW920719Q40EVB_01 download UART=COMxx
* 3. Unplug the EVAL board or Keyboard HW from your computer and power cycle the EVAL board or keyboard HW
* 4. Press any key to start LE advertising, enable inquiry scan and page scan, then pair with a PC or Tablet
*     If using the CYW920739FCBGA120 board, use a fly wire to connect GPIO P0 and P11 to simulate key 'r' press,
*     and remove the wire to simulate key release.
* 5. Once connected, it becomes the keyboard of the PC or Tablet.
*
* !!! In case you don't have the right board, i.e. CYW920739FCBGA120, which is required to support the 8*15
* key matrix used in the keyboard application. And you only have CYW920719Q40EVB_01 board.
* There is a ClientControl tool in the apps\host\client_control that you can use to test the basic BLE functions.
* NOTE!!!Make sure you include "TESTING_USING_HCI=1" in make target:
*     demo.hid.dual_mode_keyboard-CYW920719Q40EVB_01 download UART=COMxx TESTING_USING_HCI=1
*
* 1. Plug the WICED EVAL board into your computer
* 2. Build and download the application (to the WICED board) as below:
*    demo.hid.dual_mode_keyboard-CYW920719Q40EVB_01 download UART=COMxx TESTING_USING_HCI=1
* 3. Run ClientControl.exe
* 4. Choose 115200 baudrate and select the "COM Port" in ClientControl tool window.
* 5. Press "Enter Pairing Mode"or "Connect" to start LE advertising and enable inquiry scan and page scan, then pair with a PC or Tablet
* 6. Once connected, it becomes the keyboard of the PC or Tablet.
*  - Select Interrupt channel, Input report, enter the contents of the report
*    and click on the Send button, to send the report.  For example to send
*    key down event when key '1' is pushed, report should be
*    01 00 00 1e 00 00 00 00 00.  All keys up 01 00 00 00 00 00 00 00 00.
*    Please make sure you always send a key up report following key down report.
*/
#include "wiced_hal_mia.h"
#include "wiced_memory.h"
#include "gki_target.h"
#include "app.h"

#define RECOVERY_COUNT 3
#define keyscanActive() (wiced_hal_keyscan_is_any_key_pressed() || wiced_hal_keyscan_events_pending())

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
app_t app = {
.protocol = PROTOCOL_REPORT,
};

/********************************************************************************
 * Function Name: APP_getIdleRate
 ********************************************************************************
 * Summary:
 *   Process get idle rate. Generates an idle rate report on the control channel
 *   of the given transport.
 *
 * Parameters:
 *   none
 *
 * Return:
 *   The current idle rate in 4ms unit (0 means infinity idle rate)
 *
 *******************************************************************************/
STATIC uint8_t APP_getIdleRate(void)
{
    return app.idleRate;
}

/********************************************************************************
 * Function Name: APP_setIdleRate
 ********************************************************************************
 * Summary:
 *   Set the idle rate. Converts the idle rate to BT clocks and saves the value
 *   for later use.
 *
 * Parameters:
 *   idleRateIn4msUnits -- 0 means infinite idle rate
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC uint8_t APP_setIdleRate(uint8_t idleRateIn4msUnits)
{
    // Save the idle rate in units of 4 ms
    app.idleRate = idleRateIn4msUnits;

    // Convert to BT clocks for later use. Formula is ((Rate in 4 ms)*192)/15
    app.idleRateInBtClocks = idleRateIn4msUnits*192;
    app.idleRateInBtClocks /= 15;

    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/********************************************************************************
 * Function Name: APP_getProtocol
 ********************************************************************************
 * Summary:
 *   Process get current protocol request. Sends a data transaction over
 *   the control channel of the given transport with the current protocol.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   The current protocol
 *
 *******************************************************************************/
STATIC uint8_t APP_getProtocol(void)
{
    return app.protocol;
}

/********************************************************************************
 * Function Name: APP_getReport
 ********************************************************************************
 * Summary:
 *   This function implements the rxGetReport() function defined by
 *   the HID application used to handle "Get Report" requests.
 *   When the report type and id is recognized, it will send out the report to host.
 *
 * Parameters:
 *   reportType  -- report type of the requested report, e.g. feature
 *   reportId -- report id being requested
 *
 * Return:
 *   HID_PAR_HANDSHAKE_RSP_SUCCESS on success, and a DATA message will be sent out
 *   HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM on failure.  It is assumed that the
 *   caller will generate an error response.
 *
 *******************************************************************************/
STATIC uint8_t APP_getReport( uint8_t reportType, uint8_t reportId)
{
    uint8_t size;
    void *reportPtr = 0;

    // We only handle input/output reports.
    if (reportType == HID_PAR_REP_TYPE_INPUT)
    {
        switch (reportId) {
        case RPT_ID_IN_STD_KEY:
            size = sizeof(KeyboardStandardReport);
            reportPtr = &key_rpts.stdRpt;
            break;

        case RPT_ID_IN_BIT_MAPPED:
            size = sizeof(KeyboardBitMappedReport);
            reportPtr = &key_rpts.bitMappedReport;
            break;

        case RPT_ID_IN_BATTERY:
            size = sizeof(BatteryReport);
            reportPtr = &batRpt;
            break;

        case RPT_ID_IN_SLEEP:
            size = sizeof(KeyboardSleepReport);
            reportPtr = &key_rpts.sleepReport;
            break;

        case RPT_ID_IN_FUNC_LOCK:
            size = sizeof(KeyboardFuncLockReport);
            reportPtr = &key_rpts.funcLockReport;
            break;

        }
    }
    else if (reportType == HID_PAR_REP_TYPE_OUTPUT)
    {
        // Ensure that one of the valid keyboard output reports is being requested
        // Also grab its length and pointer
        if (reportId == RPT_ID_OUT_KB_LED)
        {
            size = sizeof(KeyboardLedReport);
            reportPtr = &key_rpts.ledReport;
        }
    }

    // We do not understand this, pass this to the base class.
    if (!reportPtr)
    {
        return HID_PAR_HANDSHAKE_RSP_ERR_INVALID_PARAM;
    }

    hidd_link_send_data(HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL, reportType , reportPtr, size);

    // Done!
    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/********************************************************************************
 * Function Name: APP_connect_button
 ********************************************************************************
 * Summary:
 *   Checks for if the current key is connet button.
 *   If it is, handle the action accordingly and return TRUE to indicate it is taken care of.
 *   Otherwise, it returns FALSE.
 *
 * Parameters:
 *   keyCode  -- key index
 *   Down -- TRUE to indicate the key is pressed down.
 *
 * Return:
 *   TRUE -- keyCode is connect button and it is handled.
 *   FALSE -- keyCode is not connect button.
 *
 *******************************************************************************/
STATIC wiced_bool_t APP_connect_button(uint8_t keyCode, wiced_bool_t down)
{
    if (keyCode == CONNECT_KEY_INDEX)
    {
        if (down)
        {
            WICED_BT_TRACE("\nConnect Btn Pressed");
            hidd_link_virtual_cable_unplug();
            hidd_enter_pairing();
        }
        return TRUE;
    }
    return FALSE;
}

#ifdef SUPPORT_KEYSCAN
/********************************************************************************
 * Function Name: APP_keyDetected
 ********************************************************************************
 * Summary:
 *   This is a callback function from keyscan when key action is detected
 *
 * Parameters:
 *   kbKeyEvent  -- key event structure to contain key info.
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_keyDetected(HidEventKey* kbKeyEvent)
{
    static uint8_t suppressEndScanCycleAfterConnectButton = TRUE;
    uint8_t keyDown = kbKeyEvent->keyEvent.upDownFlag == KEY_DOWN;
    uint8_t keyCode = kbKeyEvent->keyEvent.keyCode;

    // Check for buttons
    if (!APP_connect_button(keyCode, keyDown))
    {
        // Check if this is an end-of-scan cycle event
        if (keyCode == END_OF_SCAN_CYCLE)
        {
            // Yes. Queue it if it need not be suppressed
            if (!suppressEndScanCycleAfterConnectButton)
            {
                wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &kbKeyEvent->eventInfo, sizeof(HidEventKey), app.pollSeqn);
            }

            // Enable end-of-scan cycle suppression since this is the start of a new cycle
            suppressEndScanCycleAfterConnectButton = TRUE;
        }
        else if (keyCode != ROLLOVER)
        {
            WICED_BT_TRACE("\nkc:%d %c", kbKeyEvent->keyEvent.keyCode, keyDown ? 'D':'U');

            // No. Queue the key event
            wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &kbKeyEvent->eventInfo, sizeof(HidEventKey), app.pollSeqn);

            // Disable end-of-scan cycle suppression
            suppressEndScanCycleAfterConnectButton = FALSE;
        }
    }
}
#endif

/********************************************************************************
 * Function Name: APP_stdErrResp
 ********************************************************************************
 * Summary:
 *  This function provides a standard response for errors. The response is:
 *      - A rollover report is sent to the host if we are not already recovering from an error
 *      - All reports are cleared and marked as modified. They will be sent once
 *        we have recovered from the error.
 *      - Marks the func-lock key as up but dow not toggle its state even if
 *        the associated toggle flag is set. This allows for proper reconstruction
 *        of the keyboard state including func-lock dependent keys after the recovery
 *      - The recovery poll count is also set to the configured value.
 *      - Connect button state is cleared since we don't know if the connect button press
 *        is valid This is a callback function from keyscan when key action is detected
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_stdErrResp(void)
{
    // Send a rollover report
    if (!app.recoveryInProgress)
    {
        // Send rollover report
        key_sendRollover();
    }

    // Reset recovery timeout
    app.recoveryInProgress = RECOVERY_COUNT;

    key_clear(FALSE);
}

/********************************************************************************
 * Function Name: APP_stdErrRespWithFwHwReset
 ********************************************************************************
 * Summary:
 *   This function provides a standard response identical to kbAppStdErrorResponse.
 *   In addition, it also performs the following actions:
 *       - All pending events are flushed
 *       - The keyscan HW is reset
 *   This function is typically used when the FW itself is (or involved in) in error.
 *   In such cases the FW no longer has the correct state of anything and we
 *   must resort to a total reset
 *   This function is called when battery voltage drops below the configured threshold.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_stdErrRespWithFwHwReset(void)
{
    // Provide standard error response
    APP_stdErrResp();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&app.eventQueue);

    // reset keyscan
    kscan_reset();
}

/********************************************************************************
 * Function Name: APP_procEvtUserDefined
 ********************************************************************************
 * Summary:
 *   Process a user defined event. By default the keyboard application
 *   define key and scroll events. If an application needs additional types of
 *   events it should define them and override this function to process them.
 *   This function should remove the user defined event from the event queue
 *   after processing it. This function can consume additional events after
 *   the user defined event.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_procEvtUserDefined(void)
{
    WICED_BT_TRACE("\nAPP_procEvtUserDefined -- ignored");
    // no one claims for it, remove it
    if (wiced_hidd_event_queue_get_current_element(&app.eventQueue)!= NULL)
    {
        wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
    }
}

/********************************************************************************
 * Function Name: APP_flushInput
 ********************************************************************************
 * Summary:
 *   Flush out all events in queue
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_flushInput()
{
    // Flush all user inputs.
    wiced_hidd_event_queue_flush(&app.eventQueue);
    key_clear(TRUE);
}

/********************************************************************************
 * Function Name: APP_connectFailedNotification
 ********************************************************************************
 * Summary:
 *   This function is called when connection is failured
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_connectFailedNotification(void)
{
    WICED_BT_TRACE("\nconnectFailedNotification");
    APP_flushInput();
}

/********************************************************************************
 * Function Name: APP_connectFailedNotification
 ********************************************************************************
 * Summary:
 *   This function is called when system is shutting down. Typically caused by
 *   battery voltage drops below the configured threshold.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_shutdown(void)
{
    WICED_BT_TRACE("\napp_shutdown");

    // Flush the event queue
    wiced_hidd_event_queue_flush(&app.eventQueue);

    kscan_shutdown();

    if(hidd_link_is_connected())
    {
        hidd_link_disconnect();
    }
    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);
}

/********************************************************************************
 * Function Name: APP_pollActivityUser
 ********************************************************************************
 * Summary:
 *   This function is called when system is shutting down. Typically caused by
 *   battery voltage drops below the configured threshold.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   Bit mapped value indicating
 *        - HID_APP_ACTIVITY_NON_REPORTABLE - if any key (excluding connect button) is down. Always
 *          set in pin code entry state
 *        - HID_APP_ACTIVITY_REPORTABLE - if any event is queued. Always
 *          set in pin code entry state
 *        - HID_APP_ACTIVITY_NONE otherwise
 *
 *   As long as it is not ACTIVITY_NONE, the btlpm will be notified for low power management.
 *
 *******************************************************************************/
static uint8_t APP_pollActivityUser(void)
{
    uint8_t status;

    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    // Poll and queue key activity
    kscan_pollActivity();

#ifdef SUPPORT_CODE_ENTRY
    // Check if we are in pin code entry mode. If so, call the pin code entry processing function
    if (!key_entry_idle())
    {
        key_entry_handleCode(&app.eventQueue);

        // Always indicate reportable and non-reportable activity when doing pin code entry
        return HIDLINK_ACTIVITY_REPORTABLE | HIDLINK_ACTIVITY_NON_REPORTABLE;
    }
    else
#endif
    {
        // For all other cases, return value indicating whether any event is pending or
        status = wiced_hidd_event_queue_get_num_elements(&app.eventQueue) || kscan_is_any_key_pressed() ? HIDLINK_ACTIVITY_REPORTABLE : HIDLINK_ACTIVITY_NONE;

#if (SLEEP_ALLOWED == 3)
        if (!app.pollStarted)
        {
            app.pollStarted = 1;
            // if this is first poll waking up from HIDOFF, we want to reconnect
            // This is a work around for not able detect the first key done waking up from HIDOFF. The detected key
            // is support initite a connection and send the key report, but since there is no key, at least we work around
            // to make connection.
            if (hidd_is_paired() && !wiced_hal_mia_is_reset_reason_por() && !hidd_link_is_connected())
            {
                WICED_BT_TRACE("\nHIDOFF wake up reconnect");
                status = HIDLINK_ACTIVITY_REPORTABLE;
            }
        }
#endif
    }
    return status;
}

/********************************************************************************
 * Function Name: APP_sleep_handler
 ********************************************************************************
 * Summary:
 *   Sleep permit query to check if sleep is allowed and sleep time
 *
 * Parameters:
 *   type -- quary type. It can be WICED_SLEEP_POLL_TIME_TO_SLEEP or WICED_SLEEP_POLL_SLEEP_PERMISSION
 *
 * Return:
 *   WICED_SLEEP_NOT_ALLOWED -- not allow to sleep
 *   When WICED_SLEEP_POLL_TIME_TO_SLEEP:
 *      WICED_SLEEP_MAX_TIME_TO_SLEEP or the time to sleep
 *   When WICED_SLEEP_POLL_SLEEP_PERMISSION:
 *      WICED_SLEEP_ALLOWED_WITH_SHUTDOWN -- allowed to sleep, but no SDS nor ePDS
 *      WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN -- allowed to enter SDS/ePDS
 *
 *******************************************************************************/
uint32_t APP_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if (!(app.recoveryInProgress || keyscanActive()))
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
 #if SLEEP_ALLOWED > 1
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
            // a key is down, no deep sleep
            if (keyscanActive())
 #endif
            ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            break;
    }
#endif

    return ret;
}

/********************************************************************************
 * Function Name: APP_procErrEvtQueue
 ********************************************************************************
 * Summary:
 *   This function handles event queue errors. This includes event queue overflow
 *   unexpected events, missing expected events, and events in unexpected order.
 *   This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
 *   to address the problem. A user defined implementation should at least
 *   remove the first element in the queue if this event is an overflow event
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_procErrEvtQueue(void)
{
    WICED_BT_TRACE("\nKSQerr");
    APP_stdErrRespWithFwHwReset();
}

/********************************************************************************
 * Function Name: APP_procErrEvtQueue
 ********************************************************************************
 * Summary:
 *   This function handles error events reported by the keyscan HW. Typically
 *   these would be ghost events.
 *   This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
 *   to address the problem.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_procErrKeyscan(void)
{
    WICED_BT_TRACE("\nAPP_procErrKeyscan");

    //call base class handling
    APP_stdErrRespWithFwHwReset();

    key_clear(TRUE);
}

/********************************************************************************
 * Function Name: APP_procErrEvtQueue
 ********************************************************************************
 * Summary:
 *   This function provides an implementation for the generateAndTxReports() function
 *   defined by the HID application. This function is only called when the active transport
 *   is connected. This function performs the following actions:
 *    - When pin code entry is in progress, the behavior of this function is changed.
 *      It only checks and transmits the pin code report; normal event processing is
 *      suspended.
 *    - If the number of packets in the hardware fifo is less than the report generation
 *      threshold and the event queue is not empty, this function will process events
 *      by calling the event processing functions, e.g. procEvtKey() etc
 *    - This function also tracks the recovery period after an error. If
 *      the recovery count is non-zero, it is decremented as long as there is room
 *      for one report in the transport
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_generateAndTxReports(void)
{
    app_queue_t *curEvent;

    // If we are recovering from an error, decrement the recovery count as long as the transport
    // has room. Avoid the case where no event processing is done during recovery because
    // transport is full, as the failure might be a non-responding transport.
    if (app.recoveryInProgress)
    {
        // If recovery is complete, transmit any modified reports that we have been hoarding
        if (!--app.recoveryInProgress)
        {
            key_send();
        }
    }
    // Continue report generation as long as the transport has room and we have events to process
    while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) &&
           ((curEvent = (app_queue_t *)wiced_hidd_event_queue_get_current_element(&app.eventQueue)) != NULL))
    {
        // Further processing depends on the event type
        switch (curEvent->type)
        {
            case HID_EVENT_KEY_STATE_CHANGE:
                // process the event key. If fails, resets keys
                if (!key_procEvtKey(curEvent->key.keyEvent.keyCode, curEvent->key.keyEvent.upDownFlag == KEY_DOWN))
                {
                    APP_procErrKeyscan();
                }
                break;

            case HID_EVENT_EVENT_FIFO_OVERFLOW:
                // Call event queue error handler
                APP_procErrEvtQueue();
                break;

            default:
                APP_procEvtUserDefined();
                break;
        }
        wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
    }
}

/********************************************************************************
 * Function Name: APP_pollReportUserActivity
 ********************************************************************************
 * Summary:
 *  This function should be called by the transport when it wants the application
 *  to poll for user activity. This function performs the following actions:
 *   - Polls for activity. If user activity is detected, events should be
 *     queued up for processing
 *   - If an unmasked user activity is detected, it passes the activity type to the
 *     transports
 *   - If the active transport is connected, requests generation of reports via
 *     generateAndTransmitReports()
 *   - Does connect button polling and informs the BT transport once the connect
 *     button has been held for the configured amount of time.
 *  Note: transport may be NULL if no transport context is required - like when
 *  none
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  None
 *
 *******************************************************************************/
STATIC void APP_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    // Increment polling sequence number.
    app.pollSeqn++;

    if((app.pollSeqn % 64) == 0)
    {
        WICED_BT_TRACE(".");
    }

    // Check for activity. This should queue events if any user activity is detected
    activitiesDetectedInLastPoll = APP_pollActivityUser();

    // Check if the active transport is connected
    if(hidd_link_is_connected())
    {
        // Generate a report
        if(!bt_cfg.security_requirement_mask || hidd_link_is_encrypted())
        {

            APP_generateAndTxReports();
        }

#ifdef BATTERY_REPORT_SUPPORT
        if (!ota_is_active())
        {
            // poll for battery monitor
            wiced_hal_batmon_poll_monitor();
        }
#endif
    }
    else
    {
        // Check if we have any user activity. If it is paired and not connected, we reconnect.
        if (activitiesDetectedInLastPoll != HIDLINK_ACTIVITY_NONE && hidd_is_paired())
        {
            // ask the transport to connect.
            hidd_link_connect();
        }
    }
}

/********************************************************************************
 * Function Name: APP_setProtocol
 ********************************************************************************
 * Summary:
 *  Handles set protocol from the host. Uses the default hid application function
 *  for setting the protocol. In addition, if the protocol changes and the new protocol
 *  is report, it:
 *      - clears the bit mapped report
 *      - clears the sleep report
 *      - sets the func-lock key as up regardless of its current state
 *
 * Parameters:
 *  newProtocol -- protocol
 *
 * Return:
 *  HID_PAR_HANDSHAKE_RSP_SUCCESS
 *
 *******************************************************************************/
uint8_t APP_setProtocol(uint8_t newProtocol)
{
    ble_setProtocol(newProtocol);   // for BLE table selection

    // Check if the protocol was changed and the new protocol is report
    if ((app.protocol != newProtocol) && (newProtocol == HID_PAR_PROTOCOL_REPORT))
    {
        key_clear(FALSE);

        // clear sleep bits
        key_rpts.sleepReport.sleepVal = 0;
        // Mark the func-lock key as up.
        key_rpts.funcLockReport.status = FUNC_LOCK_KEY_UP;

        app.protocol = newProtocol;
    }

    return HID_PAR_HANDSHAKE_RSP_SUCCESS;
}

/********************************************************************************
 * Function Name: APP_setReport
 ********************************************************************************
 * Summary:
 *   This function implements the rxSetReport function defined by
 *   the HID application to handle "Set Report" messages.
 *   This function looks at the report ID and passes the message to the
 *   appropriate handler.
 *
 * Parameters:
 *   reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *   payload -- pointer to data that came along with the set report request after the report ID
 *   payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *   None
 *
 *******************************************************************************/
uint8_t APP_setReport(wiced_hidd_report_type_t reportType, uint8_t *payload, uint16_t payloadSize)
{
    uint8_t reportId = *payload++;
    app_setReport(reportType, reportId, payload, --payloadSize);
    return app.setReport_status;
}

/********************************************************************************
 * Function Name: APP_rxData
 ********************************************************************************
 * Summary:
 *   This function implements the rxData function defined by
 *   HID applicathetion used to handle the "Data" message.
 *   The data messages are output reports.
 *   This function looks at the report ID and passes the message to the
 *   appropriate handler.
 *
 * Parameters:
 *   reportType -- reportType extracted from the header
 *   payload -- pointer to the data message
 *   payloadSize -- size of the data message
 *
 * Return:
 *   None
 *
 *******************************************************************************/
STATIC void APP_rxData(uint8_t reportType, uint8_t *payload,  uint16_t payloadSize)
{
    APP_setReport(reportType, payload, payloadSize);
}

/********************************************************************************
 * Function Name: app_setProtocol
 *******************************************************************************
 *   This function implements the setProtocol function defined by
 *   the HID application to handle "Set Protocol" messages.
 *
 * Parameters:
 *   reportType -- not used
 *   reportId -- not used
 *   payload -- pointer to new protocol
 *   payloadSize -- not used, assumed to be 1
 *
 * Return:
 *   None
 *
 *******************************************************************************/
void app_setProtocol(wiced_hidd_report_type_t reportType,
                                   uint8_t reportId,
                                   void *payload,
                                   uint16_t payloadSize)
{
    uint8_t protocol = *((uint8_t*)payload);

//    WICED_BT_TRACE("\nNew Protocol = %d", protocol);

    APP_setProtocol(protocol);
}

/********************************************************************************
 * Function Name: app_setReport
 ********************************************************************************
 * Summary:
 *   This function implements the rxSetReport function defined by
 *   the HID application to handle "Set Report" messages.
 *   This function looks at the report ID and passes the message to the
 *   appropriate handler.
 *
 * Parameters:
 *   reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *   reportId -- report id
 *   payload -- pointer to data that came along with the set report request after the report ID
 *   payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *   None
 *
 *******************************************************************************/
void app_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    WICED_BT_TRACE("\napp_setReport: %d", payloadSize);
    app.setReport_status = HID_PAR_HANDSHAKE_RSP_SUCCESS;

    if (!key_setReport(reportType, reportId, payload, payloadSize))
    {
        if ((reportType == WICED_HID_REPORT_TYPE_FEATURE) && (reportId == RPT_ID_IN_CNT_CTL))
        {
            app.connection_ctrl_rpt = *((uint8_t*)payload);
            WICED_BT_TRACE("\nPTS_HIDS_CONFORMANCE_TC_CW_BV_03_C write val: %d ", app.connection_ctrl_rpt);
        }
        else
        {
            app.setReport_status = HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ;
        }
    }
}

/********************************************************************************
 * Function Name: app_queueEvent
 ********************************************************************************
 * Summary:
 *   Queue an event to event queue
 *
 * Parameters:
 *   event -- event to queue
 *
 * Return:
 *   None
 *
 *******************************************************************************/
void app_queueEvent(app_queue_t * event)
{
    wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &event->info, APP_QUEUE_SIZE, app.pollSeqn);
}

/********************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *   This function informs the application that the state of a link changed.
 *
 * Parameters:
 *   newState new state of the link
 *
 * Return:
 *   None
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint8_t transport, uint8_t newState)
{
    int16_t flags;
//    WICED_BT_TRACE("\n%s %s (%d)", transport==BT_TRANSPORT_LE ? "LE":"BR/EDR", hidd_link_state_str(newState), newState & HIDLINK_MASK);
    uint8_t led = transport==BT_TRANSPORT_LE ? LED_LE_LINK : LED_BREDR_LINK;

    hidd_led_blink_stop(led);
    hidd_set_deep_sleep_allowed(WICED_FALSE);

    switch (newState & HIDLINK_MASK) {
    case HIDLINK_CONNECTED:
        hidd_led_on(led);

        // enable ghost detection
        kscan_enable_ghost_detection(TRUE);

        hidd_link_enable_poll_callback(transport,WICED_TRUE);

        if(app.transportStateChangeNotification)
        {
            //We connected after power on reset or HID off recovery.
            //Start 20 second timer to allow time to setup connection encryption
            //before allowing HID Off/Micro-BCS.
            hidd_deep_sleep_not_allowed(20000); //20 seconds. timeout in ms
        }
        else
        {
            //Wake up from HID Off and already have a connection then allow HID Off in 1 second
            //This will allow time to send a key press.
            //To do need to check if key event is in the queue at lpm query
            hidd_deep_sleep_not_allowed(1000); // 1 second. timeout in ms
        }
        break;

    case HIDLINK_DISCONNECTED:
        hidd_led_off(led);
        hidd_led_off(LED_CAPS);

        // disable Ghost detection
        kscan_enable_ghost_detection(FALSE);

        // Tell the transport to stop polling
        hidd_link_enable_poll_callback(transport,WICED_FALSE);
        hidd_deep_sleep_not_allowed(2000); //2 seconds. timeout in ms
        break;

    case HIDLINK_DISCOVERABLE:
        hidd_led_blink(led, 0, 500);
        break;

    case HIDLINK_RECONNECTING:
        hidd_led_blink(led, 0, 200);     // faster blink LINK line to indicate reconnecting
        break;

    case HIDLINK_ADVERTISING_IN_uBCS_DIRECTED:
        hidd_set_deep_sleep_allowed(WICED_TRUE);
        break;
    }

    app.transportStateChangeNotification = 1;
}

/********************************************************************************
 * sleep configuration
 *******************************************************************************/
wiced_sleep_config_t    hidd_link_sleep_config = {
    . sleep_mode            = WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    .host_wake_mode         = 0,                              //host_wake_mode
    .device_wake_mode       = 0,                              //device_wake_mode
    .device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    .device_wake_gpio_num   = 255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    .sleep_permit_handler   = APP_sleep_handler,              //sleep_permit_handler
#if defined(CYW20819A1) || defined(CYW20820A1)
    .post_sleep_cback_handler=NULL,                           //post_sleep_handler
#endif
};

/********************************************************************************
 * Callback pointer structure
 *******************************************************************************/
hidd_link_callback_t appCallbacks =
{
    .p_app_poll_user_activities                 = APP_pollReportUserActivity,       //   *p_app_poll_user_activities;
    .p_app_connection_failed_notification       = APP_connectFailedNotification,    //   *p_app_connection_failed_notification;

#ifdef SUPPORT_CODE_ENTRY
    .p_app_enter_pincode_entry_mode             = key_entry_enterPinCodeEntryMode,  //   *p_app_enter_pincode_entry_mode;
    .p_app_enter_passcode_entry_mode            = key_entry_enterPassCodeEntryMode, //   *p_app_enter_passcode_entry_mode;
    .p_app_exit_pin_and_passcode_entry_mode     = key_entry_exitCodeEntryMode,      //   *p_app_exit_pin_and_passcode_entry_mode;
#endif
    .p_app_get_idle                             = APP_getIdleRate,                  //  *p_app_get_idle;
    .p_app_set_idle                             = APP_setIdleRate,                  //  *p_app_set_idle;
    .p_app_get_protocol                         = APP_getProtocol,                  //  *p_app_get_protocol;
    .p_app_set_protocol                         = APP_setProtocol,                  //  *p_app_set_protocol;
    .p_app_get_report                           = APP_getReport,                    //  *p_app_get_report;
    .p_app_set_report                           = APP_setReport,                    //  *p_app_set_report;
    .p_app_rx_data                              = APP_rxData,                       //  *p_app_rx_data;
};

/********************************************************************************
 * Function Name: app_start()
 ********************************************************************************
 * Summary:
 *   This is application start function. After system is up, when the
 *   bt management calls with BTM_ENABLED_EVT, this function is called to
 *   start application
 *
 * Parameters:
 *   none
 *
 * Return:
 *   WICED_BT_SUCCESS -- if initialization is okay and ready to start;
 *   otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start(void)
{
    WICED_BT_TRACE("\napp_start");

    // allocate necessary memory and initialize event queue
    wiced_hidd_event_queue_init(&app.eventQueue, (uint8_t *)&app.events, APP_QUEUE_SIZE, APP_QUEUE_MAX);

    // register applicaton callbacks
    hidd_register_app_callback(&appCallbacks);

    /* transport init */
    bt_init();
    hidd_sleep_configure(&hidd_link_sleep_config);

    /* component/peripheral init */
    bat_init(APP_shutdown);
    hidd_link_init();
    key_init(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS, APP_pollReportUserActivity, APP_keyDetected);

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt

    // poll for any activities
    APP_pollReportUserActivity();

    WICED_BT_TRACE("\nFree RAM bytes=%d bytes", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}
