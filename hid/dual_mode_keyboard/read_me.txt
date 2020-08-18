-------------------------------------------------------------------------------
Keyboard application
-------------------------------------------------------------------------------

Overview
--------
The Keyboard application is a single chip SoC. It can be built to BLE and/or classic BT keyboard application.
It provides a turnkey solution using on-chip keyscan HW component. It can operate in both BR/EDR Bluetooth mode and
LE, HID over GATT Profile (HOGP).

When start pairing, it operates in BR/EDR mode first. During pairing, if user re-start
pairing again, it switches to LE mode pairing. If user re-start again during LE mode
pairing, it stops pairing. The keyboard will operate in either BR/EDR or LE mode based
on last paired host.

During initialization the app registers with both LE and BR/EDR stack, WICED HID Device
Library and keyscan HW to receive various notifications including bonding complete, connection
status change, peer GATT request/commands, SDP protocol, and interrupts for key pressed/released.

If the device is not paired, press any key will start pairing. When device is successfully bonded,
the app saves bonded host's information in the NVRAM.

When user presses/releases key, a key report will be sent to the host.
On connection up or battery level changed, a battery report will be sent to the host.
When battery level is below shutdown voltage, device will do critical shutdown.
Host can send LED report to the device to control LED.

Features demonstrated
---------------------
 - BR/EDR Bluetooth operation
 - SDP protocol support
 - GATT database and Device configuration initialization
 - Registration with LE stack for various events
 - Sending HID reports to the host
 - Processing write requests from the host
 - Low power management
 - Over the air firmware update (OTAFWU)

Instructions
------------
To demonstrate the app, walk through the following steps.
1. Plug the keyboard HW into your computer
2. Build and download the application
3. Unplug the keyboard HW from your computer and power cycle the keyboard HW
4. Press 'Lock' key to start BR/EDR pairing, then pair with a PC or Tablet. The Lock key is located at right top most cornor.
5. To pair with LE host, during BR/EDR pairing, press Lock key again to stop BR/EDR pairing and start LE advertisment.
6. Once connected, it becomes the keyboard of the PC or Tablet.

In case you don't have the right hardware, reference keyboard, platform CYW920819REF-KB-01, which is required to support the 8x18
key matrix used, you can build evaluation board version in conjunction to work with ClientControl. You can choose CYW920819EVB-02,
CYW920820EVB-02, or CYW920735Q60EVB-01 platfrom. In this case, the key-matrix will not be functioning correctly due to the lack of
key-matrix pin assignment and eval board perpheral pins conflicts. However, you can test the basic Bluetooth functions and to mimic
to send key report by using ClientControl.

NOTE: To use Client Control, make sure you use "TESTING_USING_HCI=1" in application settings. 208xx device and 20735 device firmware
behaves differently. The following steps shows how to establish communication between ClientControl and the device.

For 20819/20820 devices
-----------------
1. Plug the hardware into your computer
2. Build and download the application
3. Run ClientControl.exe.
4. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
5. Open the port and then reset the device. Close and re-open the port so the HID tab gets activated.

For 20735/20835 devices
-----------------
1. Plug the hardware into your computer
2. Build and download the application
3. Run ClientControl.exe.
4. Choose 3M as Baudrate and select the serial port in ClientControl tool window.
5. Reset the device. (Press reset button or unplug/plug the USB cable)
   Within 2 seconds, before the device enters deep sleep, open the port.
   If HIDD tab is not activated, close the port and repeat step 5.

Once the HID tab is activated, the HID buttons will become available.
6. Click on "Enter Pairing Mode" to start BR/EDR pairing, then pair with a PC or Tablet.
   The "Enter/Exit Pairing Mode" click button is acting as 'Lock' key on the keyboard. Click on "Exit Pairing Mode" while it is
   in BR/EDR pairing, it switches to LE advertising for pairing. Click one more time while it is in LE advertising, it will exit
   pairing mode.
7. Once connected, it becomes the keyboard of the PC or Tablet.
8  Click on the key buttons, to send the key reports.  For example to send
   key down event when key '1' is pushed, report should be 01 00 00 1e 00 00 00 00 00.
   When key is released, it should send all keys up 01 00 00 00 00 00 00 00 00.

Notes
-----
The application GATT is located in bt/ble.h and SDP databases is located in bt/bredr.h
If you create a GATT or SDP database using Bluetooth Configurator, update the
database in the location mentioned above.

Application Settings
--------------------
LE
    Use this option to enable LE Bluetooth compliant with HID over GATT Profile (HOGP).
    The following options are available only when LE is enabled.

  DISCONNECTED_ENDLESS_ADV
    Use this option to enable disconnected endless advertisement. When this option
    is used, the device will do advertising forever until it is connected. To
    conserve power, it allows SDS/ePDS and do the advertising in a long interval.
    If AUTO_RECONNECT option is not set, then pressing a key will try to reconnect
    and stays in adv forever until it is connected.

  SKIP_PARAM_UPDATE
    Use this option to skip to send link parameter update request.
    When this option is disabled, if the peer device (master) assigned link parameter
    is not within the device's preferred range, the device will send a request for
    the desired link parameter change. This option can be enabled to stop the device
    from sending the reuqest and accept the given link parameter as is.

    Background:
    In some OS (peer host), after link is up, it continuously sends different
    parameter of LINK_PARAM_CHANGE over and over for some time. When the parameter
    is not in our device preferred range, the firmware was rejecting and renegotiating
    for new preferred parameter. It can lead up to endless and unnecessary overhead
    in link parameter change. Instead of keep rejecting the link parameter, by using
    this option, we accept peer requested link parameter as it and starts a timer to
    send the final link parameter change request later when the peer host settles down
    in link parameter change.

  ASSYMETRIC_SLAVE_LATENCY
    Use this option to enable assymetric slave latency.

    Background:
    In early days, some HID host devices will always reject HID slave's link
    parameter update request. Because of this, HID device will end up consuming
    high power when slave latency was short. To work around this issue, we use
    Asymmetric Slave Latency method to save power by waking up only at multiple
    time of the communication anchor point. When this option is enabled,

    1.  We do not send LL_CONNECTION_PARAM_REQ.
    2.  We simply start Asymmetric Slave Latency by waking up at multiple times
        of given slave latency.

    Since this is not a standard protocol, we do not recommend enabling this
    option unless if it is necessary to save power to work around some HID hosts.

  LE_LOCAL_PRIVACY
    When enabled, the device uses RPA (Random Private Address).
    When disabled, the device uses Public static address.


BREDR
    Use this option to enable classic BR/EDR.
    Since 20735 device does not support BREDR. If 20735 TARGET is used, this option will
    forced to turn off.

TESTING_USING_HCI
    Use this option for testing with Bluetooth Profile Client Control. The Client
    Control UI can be used to provide input. When this option is enabled, the
    device will not enter SDS/ePDS for power saving.

OTA_FW_UPGRADE
    Use this option for enabling firmware upgrade over the Air (OTA) capability.

OTA_SEC_FW_UPGRADE
    Use this option for secure OTA firmware upgrade. OTA_FW_UPGRADE option must be
    enabled for this option to take effect.

AUTO_RECONNECT
    Use this option to enable auto reconnect. By enabling this option, the device
    will always stay connected. If it is disconnected, it try to reconnect until
    it is connected.

    This option should be used together with DISCONNECTED_ENDLESS_ADV. When this
    option is enabled, the HID device will always try to maintain connection with
    the paired HID host; therefore, if the link is down, it will continuously try
    to reconnect. To conserve power, it should allow entering SDS/ePDS while
    advertising; thus, the DISCONNECTED_ENDLESS_ADV option should be enabled;
    otherwise, it may drain battery quickly if host was not available to reconnect.

SLEEP_ALLOWED
    Use this to set sleep option

LED
    Use this option to turn on/off LED function (Useful when turned off for power measurement)

-------------------------------------------------------------------------------

Note:
When testing with Client Control on CYW920819EVB-02, please do not pair to a host
that runs Client Control. You will not see the key presses. The reason is because
with CYW920819EVB-02 platform, since the key matrix is disabled, the only way to
send a character is by using simulated Client Control buttons. The moment you click
on the key button, the Windows focus on Client Control application. The text editor
loses focus. The character sent to host will be delivered to the focused application,
Client Control. Client Control ignores key '1', '2', '3', or 'a', 'b', 'c'. To send
the character to text editor, you will need another PC (host) to pair to CYW920819EVB-02
device so when you click on button in Client Control, the other PC (host) focused
on text editor application does not lose focus and can receive the key.
