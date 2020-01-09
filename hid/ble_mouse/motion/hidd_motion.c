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
 * This file implements the PAW3805EK-CJV1: Track-On-Glass Mouse Sensor.
 *
 */

#ifdef SUPPORT_MOTION

#include "hidd_motion.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_trace.h"
#include "wiced_hidd_lib.h"
#include "wiced_timer.h"
#include "wiced_gki.h"
#include "interrupt.h"
#include "wiced_platform.h"

void _trigger(int p, int n);
#define trigger(n) _trigger(WICED_P05, n)

struct {
    Intr_State    Intr;
    wiced_timer_t timer;
    uint8_t       state;
} motion = {};

#ifdef MOUSE_PLATFORM
 #define is_PAW       1
 #if defined(CYW20819A1)
  #define is_PAW3220  1
  #define DATA_BIT    8
 #else
  #define is_PAW3805  1
  #define DATA_BIT   16
 #endif
 #define is8BitMode()  (DATA_BIT==8)
 #define is12BitMode() (DATA_BIT==12)
 #define is16BitMode() (DATA_BIT==16)
#else
 #define is_ST_LSM9D1 1
#endif

typedef struct
{
    uint8_t       regoffset;   // register offset
    uint8_t       value;       // write value
}SensorRegSeq;

#ifdef is_PAW
 #include "wiced_hal_pspi.h"       // PAW uses SPI

 #if is_PAW3805
  #define ID1  0x31
  #define ID2  0x61
 #elif is_PAW3220
  #define ID1  0x30
  #define ID2  0x02
 SensorRegSeq HighVoltageSegmentConfig[] = {
                 {0x09,0x5A}, //to disable Write Protect
                 {0x0D,0x23}, //to set X-axis resolution to 1000 cpi
                 {0x0E,0x24}, //to set Y-axis resolution to 1000 cpi
                 {0x19,0x04}, //to use 12-bit motion data for delta_x and delta_y
                 {0x2B,0x6D},
                 {0x30,0x2E},
                 {0x5C,0xD4}, //to set current source mode, 4mA
                 {0x7F,0x01}, //to switch to Register Bank1
                 {0x06,0x14},
                 {0x31,0x25},
                 {0x34,0xC4},
                 {0x36,0xCC},
                 {0x37,0x42},
                 {0x38,0x01},
                 {0x3A,0x76},
                 {0x3B,0x34},
                 {0x42,0x39},
                 {0x43,0xF2},
                 {0x44,0x39},
                 {0x45,0xF0},
                 {0x46,0x12},
                 {0x47,0x39},
                 {0x48,0xE3},
                 {0x49,0x48},
                 {0x4A,0xD3},
                 {0x4B,0x98},
                 {0x64,0x46},
                 {0x71,0x28},
                 {0x72,0x28},
                 {0x7F,0x00}, //to switch to Register Bank0
                 {0x09,0x00}}; //to enable Write Protect
  #define INIT_CONFIG_CNT (sizeof(HighVoltageSegmentConfig)/sizeof(SensorRegSeq))
 #endif

 // register defines
 #define PRODUCT_ID1         0
 #define PRODUCT_ID2         1
 #define MOTION_STATUS       2
  #define MOTION                0x80
 #define DELTA_X             3
 #define DELTA_Y             4
 #define OPERATION_MODE      5
 #define CONFIGURATION       6
  #define PAW_CFG_RESET         0x80
  #define PAW_CFG_SLEEP3        0x20
  #define PAW_CFG_POWER_DOWN    0x08
  #define PAW_CFG_NO_SLEEP3     0x00
 #define DELTA_X_Hi       0x11
 #define DELTA_Y_Hi       0x12
 #define DELTA_XY         0x12

 #define WRITE_CMD_BIT    0x80
 #define SPI_CLK_SPEED    1000000

 #define RESET_REG CONFIGURATION
 #define RESET_VAL PAW_CFG_RESET

 #define _motion_DataAvailable()    (paw_readReg(MOTION_STATUS) & MOTION)
 #define _motion_hwInit(c)           wiced_hal_pspi_init(SPI1, SPI_MASTER, GPIO_PULL_UP, c,	SPI_CLK_SPEED, SPI_MSB_FIRST, SPI_SS_ACTIVE_LOW, SPI_MODE_3, c>>24)
 #define _motion_readMotion(x,y)     paw_readMotion(x,y)
 #define _motion_writeReg(x,y)       paw_writeReg(x,y)
 #define _motion_readReg(x)          paw_readReg(x)
 #define _motion_hwPowerDown()       paw_writeReg(CONFIGURATION, PAW_CFG_POWER_DOWN)
 #define _motion_hwEnterSleepMode()  paw_writeReg(CONFIGURATION , PAW_CFG_SLEEP3)
 #define _motion_hwExitSleepMode()   paw_writeReg(CONFIGURATION , PAW_CFG_NO_SLEEP3)
 #define _motion_hwReset()           paw_writeReg(CONFIGURATION, PAW_CFG_RESET)
 #if isPAW3220
  #define _motion_rstInit()          _motion_WriteSequence(HighVoltageSegmentConfig,INIT_CONFIG_CNT)
 #else
  #define _motion_rstInit()
 #endif
 #define _motion_idMatches()         (paw_readReg(PRODUCT_ID1)==ID1) && (paw_readReg(PRODUCT_ID2)==ID2)

#else // ST LSM
 #include "wiced_hal_i2c.h"       // LSM uses I2C
 #define LSM_I2C_ADDR               0x6a

 #define WHO_AM_I                   0x0f
  #define WHO_AM_I_ID                0x68
 #define CTRL_REG6_XL               0x20
  #define CTRL_REG6_XL_SCALE2        0x00
  #define CTRL_REG6_XL_SCALE16       0x08
  #define CTRL_REG6_XL_SCALE4        0x10
  #define CTRL_REG6_XL_SCALE8        0x18
  #define CTRL_REG6_XL_SCALE         CTRL_REG6_XL_SCALE2 // use scale 2g
  #define CTRL_REG6_XL_POWERDOWN     0x00 // power down
  #define CTRL_REG6_XL_SLEEP         0x20 // sleep, use 10Hz for sleep
  #define CTRL_REG6_XL_ACTIVE        0xd8 // active, use 952Hz for active
  #define SLM_POWERDOWN     CTRL_REG6_XL_POWERDOWN
  #define SLM_SLEEP         (CTRL_REG6_XL_SLEEP|CTRL_REG6_XL_SCALE)
  #define SLM_ACTIVE        (CTRL_REG6_XL_ACTIVE|CTRL_REG6_XL_SCALE)
 #define CTRL_REG7_XL               0x21
  #define SLM_NO_FILTER                 0
  #define SLM_FILTER_ORG_50          0x85
  #define SLM_FILTER_ORG_100         0xa5
  #define SLM_FILTER_ORG_9           0xc5
  #define SLM_FILTER_ORG_400         0xe5
 #define CTRL_REG8      0x22
  #define CTRL_REG8_RST                 1
 #define CTRL_REG9      0x23
  #define CTRL_REG0_FIFO_EN          0x02
 #define INT_GEN_SRC_XL 0x26                  // Leaner accelerationsensor interrupt source register
 #define STATUS_REG     0x27
  #define IG_XL                      0x40     // Accelerometer interrupt signal
 #define OUT_X_XL       0x28
 #define OUT_Y_XL       0x2a
 #define FIFO_CTRL      0x2e
  #define FIFO_STOPS_WHEN_FULL          1
 #define FIFO_SRC       0x2f
  #define FIFO_UNREAD_CNT_MASK       0x3f

 SensorRegSeq LSM_initConfig[] = {
                 {0x06, 0x4f},    // INT_GEN_CFG_XL, Linear acceleration sensor interrupt generator configuration register
                 {0x0c, 0x41},    // INT1_A/G pin control register
                 {0x1f, 0xd8},    // Control Reg5, Update every 8 samples for X and Y axis only
                 {0x20, 0xd8},    // Control Reg6, SLM_ACTIVE
                 {0x23, 0x02},    // Control Reg9, FIFO_EN
                 };
 #define INIT_CONFIG_CNT (sizeof(LSM_initConfig)/sizeof(SensorRegSeq))

 SensorRegSeq LSM_sleepConfig[] = {
                 {CTRL_REG6_XL,   SLM_SLEEP},
                 {CTRL_REG7_XL,   SLM_FILTER_ORG_400},
                 };
 #define SLP_CONFIG_CNT (sizeof(LSM_sleepConfig)/sizeof(SensorRegSeq))

 SensorRegSeq LSM_actConfig[] = {
                 {CTRL_REG6_XL,   SLM_ACTIVE},
                 {CTRL_REG7_XL,   SLM_NO_FILTER},
                 };
 #define ACT_CONFIG_CNT (sizeof(LSM_actConfig)/sizeof(SensorRegSeq))

 #define _motion_DataAvailable()    (lsm_readReg(STATUS_REG) & IG_XL)
 #define _motion_hwInit(c)           lsm_init()
 #define _motion_readMotion(x,y)     lsm_readMotion(x,y)
 #define _motion_readReg(x)          lsm_readReg(x)
 #define _motion_writeReg(x,y)       lsm_writeReg(x,y)
 #define _motion_hwPowerDown()       lsm_writeReg(CTRL_REG6_XL, SLM_POWERDOWN)
 #define _motion_hwEnterSleepMode()  _motion_WriteSequence(LSM_sleepConfig , SLP_CONFIG_CNT)
 #define _motion_hwExitSleepMode()   _motion_WriteSequence(LSM_actConfig , ACT_CONFIG_CNT)
 #define _motion_hwReset()           {lsm_writeReg(CTRL_REG8, CTRL_REG8_RST); while (lsm_readReg(CTRL_REG8)&CTRL_REG8_RST);}
 #define _motion_rstInit()           _motion_WriteSequence(LSM_initConfig, INIT_CONFIG_CNT)
 #define _motion_idMatches()         (lsm_readReg(WHO_AM_I) == WHO_AM_I_ID)

#endif // is_PAW

#define deviceFound (motion.state != MOTION_NOT_FOUND)
#define RESET_TIMEOUT              4
#define ACTIVE_TIMEOUT             1000           // idle for 1 second to enter sleep mode

#ifdef is_PAW
////////////////////////////////////////////////////////////////////////////////////////
/// This function reads the specified register over the SPI interface and returns
/// its value
/// \param
///    regAddress - address to read
/// \return
///    register value read from sensor
////////////////////////////////////////////////////////////////////////////////////////
uint8_t paw_readReg(uint8_t regAddress)
{
    uint8_t tx[2], rx[2];

    tx[0]=regAddress;

    wiced_hal_pspi_exchange_data( SPI1, 2, tx, rx);
    return rx[1];
}

///////////////////////////////////////////////////////////////////////////
/// This function writes the given value to the specified sensor register over the
/// SPI interface
/// \param
///    regAddress - address to write to
/// \param
///    val - value to write
///////////////////////////////////////////////////////////////////////////
void paw_writeReg(uint8_t regAddress, uint8_t val)
{
    uint8_t buf[2];

    // Create spi command. Ensure write bit is set in the address byte
    buf[0] = regAddress | WRITE_CMD_BIT;
    buf[1] = val;

    wiced_hal_pspi_tx_data(SPI1, 2,(uint8_t *)buf);
}

////////////////////////////////////////////////////////////////////////////////
/// This function reads data
/// \param
///    x,y    pointer to data storage
/// \return
///    TRUE   data is valid
///    FALSE  no data to read
////////////////////////////////////////////////////////////////////////////////
uint8_t paw_readMotion(int16_t *x, int16_t *y)
{
#pragma pack(push, 1)
    typedef struct {
        int8_t  y:4;
        int8_t  x:4;
    } xy_t;

    union {
        uint8_t  u8[4];
        int8_t   s8[4];
        xy_t     xy[4];
        int16_t  s16[2];
    } data;
#pragma pack(pop)

    if (_motion_DataAvailable())
    {
#if is8BitMode()
        data.u8[0] = paw_readReg(DELTA_X);
        data.u8[1] = paw_readReg(DELTA_Y);
        *x = data.s8[0];   // extend to 16 bit
        *y = data.s8[1];   // extend to 16 bit
//        WICED_BT_TRACE("\ndata:0x%02x 0x%02x", data.u8[0], data.u8[1]);
#elif is12BitMode()
        data.u8[0] = paw_readReg(DELTA_X);
        data.u8[2] = paw_readReg(DELTA_Y);
        data.u8[3] = paw_readReg(DELTA_XY);
//        WICED_BT_TRACE("\ndata:0x%02x 0x%02x 0x%02x", data.u8[0], data.u8[2], data.u8[3]);
        data.s8[1] = data.xy[3].x;   // sign extend
        data.s8[3] = data.xy[3].y;   // sign extend
        *x = data.s16[0];
        *y = data.s16[1];
#elif is16BitMode()
        data.u8[0] = paw_readReg(DELTA_X);
        data.u8[1] = paw_readReg(DELTA_Y);
        data.u8[2] = paw_readReg(DELTA_X_hi);
        data.u8[3] = paw_readReg(DELTA_Y_hi);
//        WICED_BT_TRACE("\ndata:0x%02x 0x%02x 0x%02x 0x%02x", data.u8[0], data.u8[1], data.u8[2], data.u8[3]);
        *x = data.s16[0];
        *y = data.s16[1];
#endif
//        WICED_BT_TRACE("\ndata:%d %d", *x, *y);
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

#else  // !is_PAW, use EVB-2 ST motion sensor

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
uint8_t lsm_writeReg(uint8_t regAddress, uint8_t val)
{
    uint8_t data_array[2];

    data_array[0] = regAddress;
    data_array[1] = val;
    return wiced_hal_i2c_write(data_array, 2, LSM_I2C_ADDR);
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
uint8_t lsm_readReg(uint8_t regAddress)
{
    uint8_t data;
    wiced_hal_i2c_combined_read(&data, 1, &regAddress, 1, LSM_I2C_ADDR);
    return data;
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
uint32_t lsm_readReg4(uint8_t regAddress)
{
    uint32_t data;
    wiced_hal_i2c_combined_read((uint8_t *) &data, 4, &regAddress, 1, LSM_I2C_ADDR);
    return data;
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
uint8_t lsm_readMotion(int16_t *x, int16_t *y)
{
    uint32_t data;
    if (_motion_DataAvailable())
    {
        data = lsm_readReg4(OUT_X_XL);
        *x = (int16_t) data & 0xffff;
        *y = (int16_t) data >> 16;
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void lsm_init()
{
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
}

#endif // is_PAW

/////////////////////////////////////////////////////////////////////
/// walk the register settings list point by regSegList
///
/// \param *regSeqList - point to the PAWSensorRegSeq structure list.
/// \param regSeqNo - size of the regSeglist
/// \return WICED_TRUE if sucess, WICED_FALSE otherwise
///
/////////////////////////////////////////////////////////////////////
void _motion_WriteSequence(const SensorRegSeq *regSeqList, uint8_t regSeqNo)
{
    uint8_t i;

    while(regSeqNo--)
    {
        _motion_writeReg(regSeqList->regoffset , regSeqList->value );
        regSeqList++;
    }
}

///////////////////////////////////////////////////////////////////////////
/// Device is reset/waking up, need some time to settle down
///////////////////////////////////////////////////////////////////////////
void _motion_warmUp(void)
{
    motion.state = MOTION_WARMUP;
    wiced_start_timer(&motion.timer, RESET_TIMEOUT);
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void _motion_exitSleepMode()
{
    WICED_BT_TRACE("\nMotion Active");
    _motion_hwExitSleepMode();
    _motion_warmUp();
}

///////////////////////////////////////////////////////////////////////////
/// Reset sensor and wait 4ms
///////////////////////////////////////////////////////////////////////////
void _motion_reset(void)
{
    if (deviceFound)
    {
        WICED_BT_TRACE("\nLSM reset");
        _motion_hwReset();
        _motion_rstInit();
        _motion_warmUp();
    }
}

///////////////////////////////////////////////////////////////////////////
/// Device is reset/waking up, need some time to settle down
///////////////////////////////////////////////////////////////////////////
void _motion_startRunTimer(void)
{
    if (wiced_is_timer_in_use(&motion.timer))
        wiced_stop_timer(&motion.timer);

    motion.state = MOTION_ACTIVE;
    wiced_start_timer(&motion.timer, ACTIVE_TIMEOUT);
}

///////////////////////////////////////////////////////////////////////////
/// Device is reset/waking up, need some time to settle down
///////////////////////////////////////////////////////////////////////////
void _motion_enterRunMode()
{
    switch (motion.state) {
    case MOTION_NOT_FOUND:
        break;
    case MOTION_SLEEP:
        _motion_exitSleepMode();
        break;
    case MOTION_SHUTDOWN:
        _motion_reset();
        break;
    default:
        _motion_startRunTimer();
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// flush motion data so that MOTION interrupt will be clear
////////////////////////////////////////////////////////////////////////////////
void _motion_flushMotion(void)
{
    int16_t delta_x, delta_y;

    trigger(3);
    WICED_BT_TRACE("\ndeviceFound%d %d", deviceFound, motion.state);
    WICED_BT_TRACE("\npin %d", Interrupt_isInterruptPinActive(&motion.Intr));

    // If we have pending motion data, keep reading until there is no data
    if (motion_isActive())
    {
        while (_motion_readMotion(&delta_x, &delta_y));
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void _motion_enterSleepMode()
{
    WICED_BT_TRACE("\nMoton Sleep");
    _motion_hwEnterSleepMode();
    motion.state = MOTION_SLEEP;
}

#if 0
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
UINT16 _abs(int16_t x)
{
    return (x<0) ? -x : x;
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void _motion_checkTolerance(int16_t *x, int16_t *y)
{
	#define XY_TOLERANCE 1
	if ((_abs(*x) <= XY_TOLERANCE) && (_abs(*y) <= XY_TOLERANCE))
	{
		*x = *y = 0;
	}
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for reset timer
////////////////////////////////////////////////////////////////////////////////
void _motion_timer_timeout( uint32_t arg )
{
    switch (motion.state) {
    case MOTION_WARMUP:
        _motion_enterRunMode();
        break;
    case MOTION_ACTIVE:
        _motion_enterSleepMode();
        break;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// public functions
//////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t motion_init(void (*userfn)(void*, uint8_t), void* userdata, uint32_t pinCfg, uint8_t pinIntr)
{
    // initialize reset timer
    wiced_init_timer( &motion.timer, _motion_timer_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    // Initialize peripheral bus
    _motion_hwInit(pinCfg);

    motion.state = _motion_idMatches() ? MOTION_ACTIVE : MOTION_NOT_FOUND;

    if (deviceFound)
    {
        if (wiced_hal_mia_is_reset_reason_por())
        {
            _motion_rstInit();
        }

        // initialize for motion interrupt
        Interrupt_init(&motion.Intr, userfn, userdata, pinIntr, INTR_LVL_HIGH, GPIO_EN_INT_RISING_EDGE);
//        Interrupt_init(&motion.Intr, userfn, userdata, pinIntr, INTR_LVL_LOW, GPIO_EN_INT_FALLING_EDGE);

        Interrupt_enableInterrupt(&motion.Intr);
        _motion_flushMotion();

        // start motion detection
        _motion_enterRunMode();
        trigger(1);
    }

    return deviceFound;
}

//////////////////////////////////////////////////////////////////////////////////////////
/// enable/disable motion interrupt
///////////////////////////////////////////////////////////////////////////////////////
void motion_enableIntr(wiced_bool_t enabled)
{
    if (deviceFound)
    {
        Interrupt_setInterruptEnable(&motion.Intr, enabled ? 1:0);
    }
}

///////////////////////////////////////////////////////////////////////////
/// Check if PAW3805 is active
///////////////////////////////////////////////////////////////////////////
wiced_bool_t motion_isActive(void)
{
    // If we have a sensor connected and MOTION interrupt is active
    return deviceFound && Interrupt_isInterruptPinActive(&motion.Intr);
}

////////////////////////////////////////////////////////////////////////////////
/// Reads motion data.
///
/// \param
///    x - pointer to location where x movement is returned
/// \param
///    y - pointer to location where y movement is returned
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t motion_getMotion(int16_t *x, int16_t *y, uint8_t MaxPoll)
{
    int16_t delta_x, delta_y;
    wiced_bool_t hasData = WICED_FALSE;

    trigger(2);
    if (motion_isActive())
    {
        trigger(1);
        _motion_startRunTimer();     // has data, reset timer

        while (_motion_readMotion(&delta_x, &delta_y) && MaxPoll--)
        {
            hasData = WICED_TRUE;
            *x += delta_x;
            *y += delta_y;
        }

        // if no more data and interrupt is pending, clear interrupt status
        if (!Interrupt_isInterruptPinActive(&motion.Intr) && Interrupt_isInterruptPending(&motion.Intr))
        {
            trigger(3);
            Interrupt_clearInterrupt(&motion.Intr);
        }

#if 0
        _motion_checkTolerance(x, y);
#endif
    }
    return hasData;
}

///////////////////////////////////////////////////////////////////////////
/// power down sensor
///////////////////////////////////////////////////////////////////////////
void motion_powerDown(void)
{
    if (deviceFound)
    {
        _motion_flushMotion();

        //disable interrupt
        Interrupt_setInterruptEnable(&motion.Intr, 0);

        _motion_hwPowerDown();
        motion.state = MOTION_SHUTDOWN;
    }
}

///////////////////////////////////////////////////////////////////////////
/// return current motion_state
///////////////////////////////////////////////////////////////////////////
motion_state_t motion_getState(void)
{
    return motion.state;
}

///////////////////////////////////////////////////////////////////////////
/// return if PAW hardware is found
///////////////////////////////////////////////////////////////////////////
wiced_bool_t motion_isFound(void)
{
    return deviceFound;
}

///////////////////////////////////////////////////////////////////////////
/// return if PAW is enabled
///////////////////////////////////////////////////////////////////////////
wiced_bool_t motion_isEnabled(void)
{
    return deviceFound && (motion.state!=MOTION_SHUTDOWN);
}

#endif // SUPPORT_MOTION
