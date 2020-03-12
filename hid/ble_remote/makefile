#
# Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
# Cypress Semiconductor Corporation. All Rights Reserved.
#
# This software, including source code, documentation and related
# materials ("Software"), is owned by Cypress Semiconductor Corporation
# or one of its subsidiaries ("Cypress") and is protected by and subject to
# worldwide patent protection (United States and foreign),
# United States copyright laws and international treaty provisions.
# Therefore, you may use this Software only as provided in the license
# agreement accompanying the software package from which you
# obtained this Software ("EULA").
# If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
# non-transferable license to copy, modify, and compile the Software
# source code solely for use in connection with Cypress's
# integrated circuit products. Any reproduction, modification, translation,
# compilation, or representation of this Software except as specified
# above is prohibited without the express written permission of Cypress.
#
# Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
# reserves the right to make changes to the Software without notice. Cypress
# does not assume any liability arising out of the application or use of the
# Software or any product or circuit described in the Software. Cypress does
# not authorize its products for use in any products where a malfunction or
# failure of the Cypress product may reasonably be expected to result in
# significant property damage, injury or death ("High Risk Product"). By
# including Cypress's product in a High Risk Product, the manufacturer
# of such system or application assumes all risk of such use and in doing
# so agrees to indemnify Cypress against all liability.
#

ifeq ($(WHICHFILE),true)
$(info Processing $(lastword $(MAKEFILE_LIST)))
endif

#
# Basic Configuration
#
APPNAME=BLE_HID_Remote
TOOLCHAIN=GCC_ARM
CONFIG=Debug
VERBOSE=

# default target
TARGET=CYW920820EVB-02
#TARGET=CYW920735Q60EVB-01

SUPPORTED_TARGETS = \
  CYW920819EVB-02 \
  CYW920820EVB-02 \
  CYW920735Q60EVB-01

TARGET_DEVICE_MAP = \
  CYW920819EVB-02/20819A1 \
  CYW920820EVB-02/20820A1 \
  CYW920735Q60EVB-01/20735B1

CY_TARGET_DEVICE = $(patsubst $(TARGET)/%,%,$(filter $(TARGET)%,$(TARGET_DEVICE_MAP)))

ifeq ($(filter $(TARGET),$(SUPPORTED_TARGETS)),)
 $(error TARGET $(TARGET) not supported for this code example)
endif

#
# Advanced Configuration
#
SOURCES=
INCLUDES=\
    $(CY_BASELIB_PATH)/include \
    $(CY_BASELIB_PATH)/include/hal \
    $(CY_BASELIB_PATH)/include/internal \
    $(CY_BASELIB_PATH)/include/stack \
    $(CY_BASELIB_PATH)/internal/$(CY_TARGET_DEVICE) \
    $(CY_SHARED_PATH)/dev-kit/btsdk-include \
    $(CY_BSP_PATH)
DEFINES=
VFP_SELECT=
CFLAGS=
CXXFLAGS=
ASFLAGS=
LDFLAGS=
LDLIBS=
LINKER_SCRIPT=
PREBUILD=
POSTBUILD=
FEATURES=

#
# Define basic library COMPONENTS
#
COMPONENTS +=bsp_design_modus
COMPONENTS += hidd_lib
CY_APP_PATCH_LIBS += wiced_hidd_lib.a

#
# App features/defaults
#
# Note: TESTING_USING_HCI only enabled for testing via HCI UART using ClientControl tool.
# It is only for the case that the WICED BOARD you get doesn't have any GPIO pins
# exposed, you can't use any fly wire to connect to GPIOs to simulate key press
# and key release to test out the application. ClientControl tool can help you
# simulate key press and release by sending report via HCI to the WICED BOARD
#
# Use OTA_FW_UPGRADE=1 to enable Over-the-air firmware upgrade functionality
# Use OTA_SEC_FW_UPGRADE=1 in the make target to use secure OTA procedure.
# See instructions in ota_firmware_upgrade.c file how to use secure upgrade.
# Use ASSYMETRIC_SLAVE_LATENCY=1 if master won't accept slave connection parameter update
# request.
# Use ENABLE_TOUCHPAD=1 to enable touchpad support
# Use ENABLE_AUDIO=1 to enable audio microphone
# Use ENABLE_DIGITAL_MIC=1 to enable digital microphone, ENABLE_DIGITAL_MIC=0 to use analog microphone
# Use ENABLE_IR=1 to enable IR support
# Use OPUS_CELT_ENCODER=1 to use OPUS CELT encoder, or
# Use ADPCM_ENCODER=1 to use ACPCM encoder, else default is mSBC
# Use AUTO_RECONNECT=1 to automatically reconnect when connection drops
# Use SKIP_PARAM_UPDATE=1 to not request connection parameter update immediately when
# received LE conn param update complete event with non-preferred values
# Use ENABLE_EASY_PAIR=1 to enable Easy Pair feature
# Use START_ADV_ON_POWERUP=1 to start advertising on power up if not connected
# Use ENABLE_CONNECTED_ADV=1 to enable advertising while connected
# Use DISCONNECTED_ENDLESS_ADV=1 to enable endless advertising while disconnected and bonded
# Use ENABLE_FINDME=1 to enable Find Me profile support
# SLEEP_ALLOWED
#  SLEEP_ALLOWED=0  Disable sleep function
#  SLEEP_ALLOWED=1  Allow sleep without shutdown
#  SLEEP_ALLOWED=2  Allow sleep with shutdown
# LED
#  LED=0  Disable LED functions (Good for power consumption measurement)
#  LED=1  Use LED for status indication

# defaults if not overridden on the command line:

ifeq ($(TARGET), CYW920735Q60EVB-01)
 SLEEP_ALLOWED_DEFAULT=1
 ENABLE_DIGITAL_MIC_DEFAULT=0
 TESTING_USING_HCI_DEFAULT=1
else
 SLEEP_ALLOWED_DEFAULT=2
 ENABLE_DIGITAL_MIC_DEFAULT=1
 TESTING_USING_HCI_DEFAULT=0
endif
XIP?=xip
OTA_FW_UPGRADE_DEFAULT=1
OTA_SEC_FW_UPGRADE_DEFAULT=0
ENABLE_TOUCHPAD_DEFAULT=0
ENABLE_AUDIO_DEFAULT=1
ENABLE_ANDROID_AUDIO_DEFAULT=0
ENABLE_IR_DEFAULT=0
OPUS_CELT_ENCODER_DEFAULT=0
ADPCM_ENCODER_DEFAULT=0
AUTO_RECONNECT_DEFAULT=0
SKIP_PARAM_UPDATE_DEFAULT=1
ENABLE_EASY_PAIR_DEFAULT=0
START_ADV_ON_POWERUP_DEFAULT=0
ENABLE_CONNECTED_ADV_DEFAULT=0
DISCONNECTED_ENDLESS_ADV_DEFAULT=0
ENABLE_FINDME_DEFAULT=0
ASSYMETRIC_SLAVE_LATENCY_DEFAULT=0
LED_SUPPORT_DEFAULT=1

XIP?=xip
BT_DEVICE_ADDRESS?=default
UART?=AUTO
TESTING_USING_HCI ?= $(TESTING_USING_HCI_DEFAULT)
OTA_FW_UPGRADE ?= $(OTA_FW_UPGRADE_DEFAULT)
OTA_SEC_FW_UPGRADE ?= $(OTA_SEC_FW_UPGRADE_DEFAULT)
ENABLE_TOUCHPAD ?= $(ENABLE_TOUCHPAD_DEFAULT)
ENABLE_ANDROID_AUDIO ?= $(ENABLE_ANDROID_AUDIO_DEFAULT)
ifeq ($(ENABLE_ANDROID_AUDIO),1)
 ENABLE_AUDIO=1
 ADPCM_ENCODER=1
 OPUS_CELT_ENCODER=0
else
 ENABLE_AUDIO ?= $(ENABLE_AUDIO_DEFAULT)
 OPUS_CELT_ENCODER ?= $(OPUS_CELT_ENCODER_DEFAULT)
 ifeq ($(OPUS_CELT_ENCODER),1)
  ADPCM_ENCODER = 0
 else
  ADPCM_ENCODER ?= $(ADPCM_ENCODER_DEFAULT)
 endif
endif
ENABLE_DIGITAL_MIC ?= $(ENABLE_DIGITAL_MIC_DEFAULT)
ENABLE_IR ?= $(ENABLE_IR_DEFAULT)
AUTO_RECONNECT ?= $(AUTO_RECONNECT_DEFAULT)
SKIP_PARAM_UPDATE ?= $(SKIP_PARAM_UPDATE_DEFAULT)
ENABLE_EASY_PAIR ?= $(ENABLE_EASY_PAIR_DEFAULT)
START_ADV_ON_POWERUP ?= $(START_ADV_ON_POWERUP_DEFAULT)
ENABLE_CONNECTED_ADV ?= $(ENABLE_CONNECTED_ADV_DEFAULT)
DISCONNECTED_ENDLESS_ADV ?= $(DISCONNECTED_ENDLESS_ADV_DEFAULT)
ENABLE_FINDME ?= $(ENABLE_FINDME_DEFAULT)
ASSYMETRIC_SLAVE_LATENCY ?= $(ASSYMETRIC_SLAVE_LATENCY_DEFAULT)
SLEEP_ALLOWED ?= $(SLEEP_ALLOWED_DEFAULT)
LED ?= $(LED_SUPPORT_DEFAULT)

#
# App defines
#
CY_APP_DEFINES = \
  -DWICED_BT_TRACE_ENABLE \
  -DBLE_SUPPORT \
  -DLED_SUPPORT=$(LED) \
  -DSLEEP_ALLOWED=$(SLEEP_ALLOWED)

ifneq ($(filter CYW920819EVB-02 CYW920820EVB-02 CYW920820REF-RM-01,$(TARGET)),)
 CY_APP_DEFINES += -DSUPPORT_EPDS
endif

ifeq ($(TARGET), CYW920820REF-RM-01)
 CY_APP_DEFINES += -DREMOTE_PLATFORM
endif

ifeq ($(hci_dump), 1)
 CY_APP_DEFINES += -DHCI_TRACES_ENABLED
else
 ifeq ($(TESTING_USING_HCI),1)
  CY_APP_DEFINES += -DTESTING_USING_HCI
 endif
endif

ifeq ($(ENABLE_SCROLL),1)
 CY_APP_DEFINES += -DSUPPORT_SCROLL
endif

ifeq ($(ASSYMETRIC_SLAVE_LATENCY),1)
 CY_APP_DEFINES += -DASSYM_SLAVE_LATENCY
endif

ifeq ($(ENABLE_AUDIO),1)
 CY_APP_DEFINES += -DSUPPORT_AUDIO

 ifeq ($(ENABLE_ANDROID_AUDIO),1)
  CY_APP_DEFINES += -DANDROID_AUDIO
 else
  CY_APP_DEFINES += -DHID_AUDIO
  # send audio data as 1 big gatt packet
  CY_APP_DEFINES += -DATT_MTU_SIZE_180
 endif

 #enabled audio enhancement
 CY_APP_DEFINES += -DENABLE_ADC_AUDIO_ENHANCEMENTS

 ifneq ($(filter CYW920819EVB-02 CYW920820EVB-02 CYW989820EVB-01 CYW920820REF-RM-01 CYBT-213043-EVAL,$(PLATFORM)),)
  CY_APP_PATCH_LIBS += adc_audio_lib.a
 endif

 ifeq ($(OPUS_CELT_ENCODER), 1)
  #use OPUS CELT encoder
  CY_APP_DEFINES += -DCELT_ENCODER
  ifeq (A_20819A1,$(BLD))
   CY_APP_PATCH_LIBS += celt_lib.a
  endif
 else
  ifeq ($(ADPCM_ENCODER), 1)
   CY_APP_DEFINES += -DADPCM_ENCODER
   CY_APP_PATCH_LIBS += adpcm_lib.a
  else
   #use mSBC encoder
   CY_APP_DEFINES += -DSBC_ENCODER
   #enabled only if 256 Kbyte/2M Bit Sflash is used; default is 512 Kbyte/4M Bit Sflash
   #CY_APP_DEFINES += -DSFLASH_SIZE_2M_BITS
  endif # ADPCM_ENCODER
 endif # OPUS_CELT_ENCODER

 ifeq ($(ENABLE_DIGITAL_MIC),1)
  CY_APP_DEFINES += -DSUPPORT_DIGITAL_MIC
 endif
endif # ENABLE_AUDIO

ifeq ($(SKIP_PARAM_UPDATE),1)
 CY_APP_DEFINES += -DSKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
endif

ifeq ($(AUTO_RECONNECT),1)
 CY_APP_DEFINES += -DAUTO_RECONNECT
endif

ifeq ($(ENABLE_EASY_PAIR),1)
 CY_APP_DEFINES += -DEASY_PAIR
endif

ifeq ($(START_ADV_ON_POWERUP),1)
 CY_APP_DEFINES += -DSTART_ADV_WHEN_POWERUP_NO_CONNECTED
endif

ifeq ($(ENABLE_CONNECTED_ADV),1)
 CY_APP_DEFINES += -DCONNECTED_ADVERTISING_SUPPORTED
endif

ifeq ($(DISCONNECTED_ENDLESS_ADV),1)
 CY_APP_DEFINES += -DENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
endif

ifeq ($(ENABLE_FINDME),1)
 CY_APP_DEFINES += -DSUPPORTING_FINDME
endif

ifeq ($(OTA_FW_UPGRADE),1)
 # DEFINES
 CY_APP_DEFINES += -DOTA_FIRMWARE_UPGRADE
 CY_APP_DEFINES += -DDISABLED_SLAVE_LATENCY_ONLY
 CY_APP_DEFINES += -DOTA_SKIP_CONN_PARAM_UPDATE
 ifeq ($(OTA_SEC_FW_UPGRADE), 1)
  CY_APP_DEFINES += -DOTA_SECURE_FIRMWARE_UPGRADE
 endif # OTA_SEC_FW_UPGRADE
 # COMPONENTS
 COMPONENTS += fw_upgrade_lib
else
 ifeq ($(OTA_SEC_FW_UPGRADE),1)
  $(error setting OTA_SEC_FW_UPGRADE=1 requires OTA_FW_UPGRADE also set to 1)
 endif # OTA_SEC_FW_UPGRADE
endif # OTA_FW_UPGRADE

################################################################################
# Paths
################################################################################

# Path (absolute or relative) to the project
CY_APP_PATH=.

# Path (absolute or relative) to the bt-sdk folder (at repo root)
CY_SHARED_PATH=$(CY_APP_PATH)/../../../wiced_btsdk
# absolute path to the root of the BTSDK
CY_SHARED_PATH_ABS=$(CURDIR)/../../../wiced_btsdk

# Path (absolute or relative) to the base library
CY_BASELIB_PATH=$(CY_SHARED_PATH)/dev-kit/baselib/$(CY_TARGET_DEVICE)

# Path to the bsp library
CY_BSP_PATH=$(CY_SHARED_PATH)/dev-kit/bsp/TARGET_$(TARGET)

INCLUDES+=\
    $(CY_BASELIB_PATH)/WICED/common

CY_DEVICESUPPORT_PATH=$(CY_BASELIB_PATH)

# Absolute path to the compiler (Default: GCC in the tools)
CY_COMPILER_PATH=

# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_* \
    $(CY_IDE_TOOLS_DIR))

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder).
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
 $(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS))
endif

# path to WICED tools root folder
CY_WICED_TOOLS_ROOT?=$(CY_SHARED_PATH)/dev-kit/btsdk-tools

# tools that can be launched with "make open CY_OPEN_TYPE=<tool>
CY_BT_APP_TOOLS=BTSpy ClientControl

# getlibs path
CY_GETLIBS_PATH=.

# paths to shared_libs targets
SEARCH_LIBS_AND_INCLUDES=$(CY_BSP_PATH) $(CY_BASELIB_PATH)
SEARCH_LIBS_AND_INCLUDES+=$(CY_SHARED_PATH)/dev-kit/libraries/btsdk-ble
SEARCH_LIBS_AND_INCLUDES+=$(CY_SHARED_PATH)/dev-kit/libraries/btsdk-hid
ifeq ($(OTA_FW_UPGRADE),1)
SEARCH_LIBS_AND_INCLUDES+=$(CY_SHARED_PATH)/dev-kit/libraries/btsdk-ota
endif

# common make arguments for lib builds
MAKE_OVERRIDES=OTA_FW_UPGRADE=$(OTA_FW_UPGRADE)
export CONFIG
export COMPONENTS
export DISABLE_COMPONENTS
CY_SHARED_LIB_ARGS:=$(MAKECMDGOALS) $(MAKE_OVERRIDES) CY_APP_DEFINES+="$(CY_APP_DEFINES)" CY_TARGET_DEVICE=$(CY_TARGET_DEVICE) TARGET=$(TARGET)

.PHONY: $(SEARCH_LIBS_AND_INCLUDES) shared_libs

# recursive make on each shared_lib project
shared_libs: $(SEARCH_LIBS_AND_INCLUDES)
$(SEARCH_LIBS_AND_INCLUDES):
	$(MAKE) -C $@ $(CY_SHARED_LIB_ARGS)

CY_APP_LOCATION=$(lastword $(MAKEFILE_LIST))

CY_APP_BUILD_GOALS:=build qbuild clean program qprogram debug qdebug all
-include internal.mk
ifeq ($(filter setup teardown,$(MAKECMDGOALS)),)

# ensure pre-requisite wiced_btsdk project is present
ifeq ($(wildcard $(CY_SHARED_PATH)),)
# if not present, IDE will be trying to run make eclipse to get launch configuration
# need to store in that case and re-run later after the wiced_btsdk project exists
ifeq ($(MAKECMDGOALS),eclipse)
$(shell echo $(CY_IDE_PRJNAME) > ./.cy_ide_prjname)
endif
ifneq ($(filter eclipse $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
$(warning BTSDK application projects require the wiced_btsdk project as a prerequisite.)
$(warning Please create the wiced_btsdk project via New Application wizard in the IDE or via git clone for CLI.)
$(error Missing prerequisite wiced_btsdk project)
endif
endif

# ensure that the wiced_btsdk project has performed getlibs to populate the assets (CLI only)
ifneq ($(filter $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
ifeq ($(wildcard $(CY_BASELIB_PATH)),)
$(warning Prerequisite baselib path $(CY_BASELIB_PATH) not found under wiced_btsdk project.)
$(warning Please perform 'make getlibs' in the $(CY_SHARED_PATH) folder.)
$(error Missing prerequisite $(CY_BASELIB_PATH))
endif
endif

# all prerequisites satisfied, check for .cy_ide_prjname if stored earlier
ifeq ($(CY_MAKE_IDE),eclipse)
ifneq ($(filter $(CY_APP_BUILD_GOALS),$(MAKECMDGOALS)),)
ifneq ($(wildcard ./.cy_ide_prjname),)
CY_IDE_PRJNAME=`cat ./.cy_ide_prjname`
$(MAKECMDGOALS): regenerate_launches
regenerate_launches:
	@make -C . eclipse CY_IDE_PRJNAME=$(CY_IDE_PRJNAME)
	@rm ./.cy_ide_prjname
endif
endif
endif
include $(CY_TOOLS_DIR)/make/start.mk
endif
