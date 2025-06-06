/***************************************************************************//**
 *   @file   adxl372.h
 *   @brief  Header file for adxl372 Driver.
 *   @author SPopa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef ADXL372_H_
#define ADXL372_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/*
 * ADXL372 registers definition
 */
#define ADXL372_DEVID	0x00u   /* Analog Devices, Inc., accelerometer ID */
#define ADXL372_DEVID_MST	0x01u   /* Analog Devices MEMS device ID */
#define ADXL372_PARTID		0x02u   /* Device ID */
#define ADXL372_REVID		0x03u   /* product revision ID*/
#define ADXL372_STATUS_1	0x04u   /* Status register 1 */
#define ADXL372_STATUS_2	0x05u   /* Status register 2 */
#define ADXL372_FIFO_ENTRIES_2	0x06u   /* Valid data samples in the FIFO */
#define ADXL372_FIFO_ENTRIES_1	0x07u   /* Valid data samples in the FIFO */
#define ADXL372_X_DATA_H	0x08u   /* X-axis acceleration data [11:4] */
#define ADXL372_X_DATA_L	0x09u   /* X-axis acceleration data [3:0] */
#define ADXL372_Y_DATA_H	0x0Au   /* Y-axis acceleration data [11:4] */
#define ADXL372_Y_DATA_L	0x0Bu   /* Y-axis acceleration data [3:0] */
#define ADXL372_Z_DATA_H	0x0Cu   /* Z-axis acceleration data [11:4] */
#define ADXL372_Z_DATA_L	0x0Du   /* Z-axis acceleration data [3:0] */
#define ADXL372_X_MAXPEAK_H	0x15u   /* X-axis MaxPeak acceleration data [15:8] */
#define ADXL372_X_MAXPEAK_L	0x16u   /* X-axis MaxPeak acceleration data [7:0] */
#define ADXL372_Y_MAXPEAK_H	0x17u   /* Y-axis MaxPeak acceleration data [15:8] */
#define ADXL372_Y_MAXPEAK_L	0x18u   /* Y-axis MaxPeak acceleration data [7:0] */
#define ADXL372_Z_MAXPEAK_H	0x19u   /* Z-axis MaxPeak acceleration data [15:8] */
#define ADXL372_Z_MAXPEAK_L	0x1Au   /* Z-axis MaxPeak acceleration data [7:0] */
#define ADXL372_OFFSET_X	0x20u   /* X axis offset */
#define ADXL372_OFFSET_Y	0x21u   /* Y axis offset */
#define ADXL372_OFFSET_Z	0x22u   /* Z axis offset */
#define ADXL372_X_THRESH_ACT_H	0x23u   /* X axis Activity Threshold [15:8] */
#define ADXL372_X_THRESH_ACT_L	0x24u   /* X axis Activity Threshold [7:0] */
#define ADXL372_Y_THRESH_ACT_H	0x25u   /* Y axis Activity Threshold [15:8] */
#define ADXL372_Y_THRESH_ACT_L 	0x26u   /* Y axis Activity Threshold [7:0] */
#define ADXL372_Z_THRESH_ACT_H	0x27u   /* Z axis Activity Threshold [15:8] */
#define ADXL372_Z_THRESH_ACT_L	0x28u   /* Z axis Activity Threshold [7:0] */
#define ADXL372_TIME_ACT	0x29u   /* Activity Time */
#define ADXL372_X_THRESH_INACT_H	0x2Au   /* X axis Inactivity Threshold [15:8] */
#define ADXL372_X_THRESH_INACT_L	0x2Bu   /* X axis Inactivity Threshold [7:0] */
#define ADXL372_Y_THRESH_INACT_H    	0x2Cu   /* Y axis Inactivity Threshold [15:8] */
#define ADXL372_Y_THRESH_INACT_L    	0x2Du   /* Y axis Inactivity Threshold [7:0] */
#define ADXL372_Z_THRESH_INACT_H	0x2Eu   /* Z axis Inactivity Threshold [15:8] */
#define ADXL372_Z_THRESH_INACT_L	0x2Fu   /* Z axis Inactivity Threshold [7:0] */
#define ADXL372_TIME_INACT_H       	0x30u   /* Inactivity Time [15:8] */
#define ADXL372_TIME_INACT_L        	0x31u   /* Inactivity Time [7:0] */
#define ADXL372_X_THRESH_ACT2_H     	0x32u   /* X axis Activity2 Threshold [15:8] */
#define ADXL372_X_THRESH_ACT2_L		0x33u   /* X axis Activity2 Threshold [7:0] */
#define ADXL372_Y_THRESH_ACT2_H     	0x34u   /* Y axis Activity2 Threshold [15:8] */
#define ADXL372_Y_THRESH_ACT2_L     	0x35u   /* Y axis Activity2 Threshold [7:0] */
#define ADXL372_Z_THRESH_ACT2_H		0x36u   /* Z axis Activity2 Threshold [15:8] */
#define ADXL372_Z_THRESH_ACT2_L		0x37u   /* Z axis Activity2 Threshold [7:0] */
#define ADXL372_HPF			0x38u   /* High Pass Filter */
#define ADXL372_FIFO_SAMPLES        	0x39u   /* FIFO Samples */
#define ADXL372_FIFO_CTL	    	0x3Au   /* FIFO Control */
#define ADXL372_INT1_MAP	0x3Bu   /* Interrupt 1 mapping control */
#define ADXL372_INT2_MAP        0x3Cu   /* Interrupt 2 mapping control */
#define ADXL372_TIMING	       	0x3Du   /* Timing */
#define ADXL372_MEASURE		0x3Eu   /* Measure */
#define ADXL372_POWER_CTL       0x3Fu   /* Power control */
#define ADXL372_SELF_TEST       0x40u   /* Self Test */
#define ADXL372_RESET           0x41u   /* Reset */
#define ADXL372_FIFO_DATA	0x42u   /* FIFO Data */

#define ADXL372_DEVID_VAL       0xADu   /* Analog Devices, Inc., accelerometer ID */
#define ADXL372_MST_DEVID_VAL   0x1Du   /* Analog Devices MEMS device ID */
#define ADXL372_PARTID_VAL      0xFAu   /* Device ID */
#define ADXL372_REVID_VAL       0x02u   /* product revision ID*/
#define ADXL372_RESET_CODE	0x52u	/* Writing code 0x52 resets the device */

#define ADXL372_REG_READ(x)	(((x & 0xFF) << 1) | 0x01)
#define ADXL372_REG_WRITE(x)	((x & 0xFF) << 1)

/* ADXL372_POWER_CTL */
#define ADXL372_POWER_CTL_INSTANT_ON_TH_MSK	NO_OS_BIT(5)
#define ADXL372_POWER_CTL_INSTANT_ON_TH_MODE(x)	(((x) & 0x1) << 5)
#define ADXL372_POWER_CTL_FIL_SETTLE_MSK	NO_OS_BIT(4)
#define ADXL372_POWER_CTL_FIL_SETTLE_MODE(x)	(((x) & 0x1) << 4)
#define ADXL372_POWER_CTL_LPF_DIS_MSK		NO_OS_BIT(3)
#define ADXL372_POWER_CTL_LPF_DIS_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_POWER_CTL_HPF_DIS_MSK		NO_OS_BIT(2)
#define ADXL372_POWER_CTL_HPF_DIS_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_POWER_CTL_MODE_MSK		NO_OS_GENMASK(1, 0)
#define ADXL372_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL372_MEASURE */
#define ADXL372_MEASURE_AUTOSLEEP_MSK		NO_OS_BIT(6)
#define ADXL372_MEASURE_AUTOSLEEP_MODE(x)	(((x) & 0x1) << 6)
#define ADXL372_MEASURE_LINKLOOP_MSK		NO_OS_GENMASK(5, 4)
#define ADXL372_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL372_MEASURE_LOW_NOISE_MSK		NO_OS_BIT(3)
#define ADXL372_MEASURE_LOW_NOISE_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_MEASURE_BANDWIDTH_MSK		NO_OS_GENMASK(2, 0)
#define ADXL372_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL372_TIMING */
#define ADXL372_TIMING_ODR_MSK			NO_OS_GENMASK(7, 5)
#define ADXL372_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)
#define ADXL372_TIMING_WAKE_UP_RATE_MSK		NO_OS_GENMASK(4, 2)
#define ADXL372_TIMING_WAKE_UP_RATE_MODE(x)	(((x) & 0x7) << 2)
#define ADXL372_TIMING_EXT_CLK_MSK		NO_OS_BIT(1)
#define ADXL372_TIMING_EXT_CLK_MODE(x)		(((x) & 0x1) << 1)
#define ADXL372_TIMING_EXT_SYNC_MSK		NO_OS_BIT(0)
#define ADXL372_TIMING_EXT_SYNC_MODE(x)		(((x) & 0x1) << 0)

/* ADXL372_FIFO_CTL */
#define ADXL372_FIFO_CTL_FORMAT_MSK		NO_OS_GENMASK(5, 3)
#define ADXL372_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL372_FIFO_CTL_MODE_MSK		NO_OS_GENMASK(2, 1)
#define ADXL372_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL372_FIFO_CTL_SAMPLES_MSK		NO_OS_BIT(0)
#define ADXL372_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL372_STATUS_1 */
#define ADXL372_STATUS_1_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL372_STATUS_1_FIFO_RDY(x)		(((x) >> 1) & 0x1)
#define ADXL372_STATUS_1_FIFO_FULL(x)		(((x) >> 2) & 0x1)
#define ADXL372_STATUS_1_FIFO_OVR(x)		(((x) >> 3) & 0x1)
#define ADXL372_STATUS_1_USR_NVM_BUSY(x)	(((x) >> 5) & 0x1)
#define ADXL372_STATUS_1_AWAKE(x)		(((x) >> 6) & 0x1)
#define ADXL372_STATUS_1_ERR_USR_REGS(x)	(((x) >> 7) & 0x1)

/* ADXL372_INT1_MAP */
#define ADXL372_INT1_MAP_DATA_RDY_MSK		NO_OS_BIT(0)
#define ADXL372_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL372_INT1_MAP_FIFO_RDY_MSK		NO_OS_BIT(1)
#define ADXL372_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL372_INT1_MAP_FIFO_FULL_MSK		NO_OS_BIT(2)
#define ADXL372_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_INT1_MAP_FIFO_OVR_MSK		NO_OS_BIT(3)
#define ADXL372_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_INT1_MAP_INACT_MSK		NO_OS_BIT(4)
#define ADXL372_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL372_INT1_MAP_ACT_MSK		NO_OS_BIT(5)
#define ADXL372_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL372_INT1_MAP_AWAKE_MSK		NO_OS_BIT(6)
#define ADXL372_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL372_INT1_MAP_LOW_MSK		NO_OS_BIT(7)
#define ADXL372_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL372_INT2_MAP */
#define ADXL372_INT2_MAP_DATA_RDY_MSK		NO_OS_BIT(0)
#define ADXL372_INT2_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL372_INT2_MAP_FIFO_RDY_MSK		NO_OS_BIT(1)
#define ADXL372_INT2_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL372_INT2_MAP_FIFO_FULL_MSK		NO_OS_BIT(2)
#define ADXL372_INT2_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL372_INT2_MAP_FIFO_OVR_MSK		NO_OS_BIT(3)
#define ADXL372_INT2_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL372_INT2_MAP_INACT_MSK		NO_OS_BIT(4)
#define ADXL372_INT2_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL372_INT2_MAP_ACT_MSK		NO_OS_BIT(5)
#define ADXL372_INT2_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL372_INT2_MAP_AWAKE_MSK		NO_OS_BIT(6)
#define ADXL372_INT2_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL372_INT2_MAP_LOW_MSK		NO_OS_BIT(7)
#define ADXL372_INT2_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/***************************************************************************//**
 * @brief The `adxl372_th_reg_addr_h` is a 2D array of integers that holds the
 * high byte register addresses for the activity and inactivity
 * thresholds of the ADXL372 accelerometer for the X, Y, and Z axes. It
 * is organized into three sets of three addresses, corresponding to
 * activity, activity2, and inactivity thresholds for each axis.
 *
 * @details This variable is used to access the high byte register addresses for
 * configuring the activity and inactivity thresholds of the ADXL372
 * accelerometer.
 ******************************************************************************/
static const int adxl372_th_reg_addr_h[3][3] = {
	{
		ADXL372_X_THRESH_ACT_H,
		ADXL372_Y_THRESH_ACT_H,
		ADXL372_Z_THRESH_ACT_H,
	}, {
		ADXL372_X_THRESH_ACT2_H,
		ADXL372_Y_THRESH_ACT2_H,
		ADXL372_Z_THRESH_ACT2_H,
	}, {
		ADXL372_X_THRESH_INACT_H,
		ADXL372_Y_THRESH_INACT_H,
		ADXL372_Z_THRESH_INACT_H,
	}
};

/***************************************************************************//**
 * @brief The `adxl372_th_reg_addr_l` is a 3x3 static constant integer array
 * that holds the lower byte register addresses for the activity and
 * inactivity threshold settings of the ADXL372 accelerometer for the X,
 * Y, and Z axes. Each row corresponds to a different threshold type:
 * activity, activity2, and inactivity.
 *
 * @details This array is used to access the lower byte register addresses for
 * configuring the activity and inactivity thresholds of the ADXL372
 * accelerometer.
 ******************************************************************************/
static const int adxl372_th_reg_addr_l[3][3] = {
	{
		ADXL372_X_THRESH_ACT_L,
		ADXL372_Y_THRESH_ACT_L,
		ADXL372_Z_THRESH_ACT_L,
	}, {
		ADXL372_X_THRESH_ACT2_L,
		ADXL372_Y_THRESH_ACT2_L,
		ADXL372_Z_THRESH_ACT2_L,
	}, {
		ADXL372_X_THRESH_INACT_L,
		ADXL372_Y_THRESH_INACT_L,
		ADXL372_Z_THRESH_INACT_L,
	}
};

/***************************************************************************//**
 * @brief The `adxl372_axis` enumeration defines the three possible axes (X, Y,
 * and Z) for the ADXL372 accelerometer, which is used to specify the
 * axis of interest when interacting with the device's data or
 * configuration settings.
 *
 * @param ADXL372_X_AXIS Represents the X-axis of the ADXL372 accelerometer.
 * @param ADXL372_Y_AXIS Represents the Y-axis of the ADXL372 accelerometer.
 * @param ADXL372_Z_AXIS Represents the Z-axis of the ADXL372 accelerometer.
 ******************************************************************************/
enum adxl372_axis {
	ADXL372_X_AXIS,
	ADXL372_Y_AXIS,
	ADXL372_Z_AXIS
};

/***************************************************************************//**
 * @brief The `adxl372_op_mode` enumeration defines the various operational
 * modes available for the ADXL372 accelerometer device. These modes
 * control the power and performance characteristics of the device,
 * allowing it to operate in standby, wake-up, instant-on, or full
 * bandwidth measurement modes. Each mode is tailored for specific use
 * cases, balancing between power consumption and data acquisition needs.
 *
 * @param ADXL372_STANDBY Represents the standby mode of the ADXL372 device.
 * @param ADXL372_WAKE_UP Represents the wake-up mode of the ADXL372 device.
 * @param ADXL372_INSTANT_ON Represents the instant-on mode of the ADXL372
 * device.
 * @param ADXL372_FULL_BW_MEASUREMENT Represents the full bandwidth measurement
 * mode of the ADXL372 device.
 ******************************************************************************/
enum adxl372_op_mode {
	ADXL372_STANDBY,
	ADXL372_WAKE_UP,
	ADXL372_INSTANT_ON,
	ADXL372_FULL_BW_MEASUREMENT
};

/***************************************************************************//**
 * @brief The `adxl372_bandwidth` enumeration defines the possible bandwidth
 * settings for the ADXL372 accelerometer device. Each enumerator
 * corresponds to a specific bandwidth frequency, allowing the user to
 * configure the device's data rate and filtering characteristics
 * according to the application's requirements. This configuration is
 * crucial for optimizing the device's performance in different
 * operational environments.
 *
 * @param ADXL372_BW_200HZ Represents a bandwidth setting of 200 Hz for the
 * ADXL372 device.
 * @param ADXL372_BW_400HZ Represents a bandwidth setting of 400 Hz for the
 * ADXL372 device.
 * @param ADXL372_BW_800HZ Represents a bandwidth setting of 800 Hz for the
 * ADXL372 device.
 * @param ADXL372_BW_1600HZ Represents a bandwidth setting of 1600 Hz for the
 * ADXL372 device.
 * @param ADXL372_BW_3200HZ Represents a bandwidth setting of 3200 Hz for the
 * ADXL372 device.
 ******************************************************************************/
enum adxl372_bandwidth {
	ADXL372_BW_200HZ,
	ADXL372_BW_400HZ,
	ADXL372_BW_800HZ,
	ADXL372_BW_1600HZ,
	ADXL372_BW_3200HZ
};

/***************************************************************************//**
 * @brief The `adxl372_act_proc_mode` is an enumeration that defines the
 * different activity processing modes available for the ADXL372
 * accelerometer. These modes determine how the device processes activity
 * detection, with options for default, linked, and looped processing.
 * This allows for flexible configuration of the accelerometer's behavior
 * in response to detected motion.
 *
 * @param ADXL372_DEFAULT Represents the default activity processing mode.
 * @param ADXL372_LINKED Represents a linked activity processing mode.
 * @param ADXL372_LOOPED Represents a looped activity processing mode.
 ******************************************************************************/
enum adxl372_act_proc_mode {
	ADXL372_DEFAULT,
	ADXL372_LINKED,
	ADXL372_LOOPED
};

/***************************************************************************//**
 * @brief The `adxl372_odr` enumeration defines the possible output data rates
 * (ODR) for the ADXL372 accelerometer. Each enumerator corresponds to a
 * specific frequency at which the accelerometer can output data, ranging
 * from 400 Hz to 6400 Hz. This allows users to configure the device to
 * operate at different data rates depending on the application's
 * requirements for data resolution and power consumption.
 *
 * @param ADXL372_ODR_400HZ Represents an output data rate of 400 Hz.
 * @param ADXL372_ODR_800HZ Represents an output data rate of 800 Hz.
 * @param ADXL372_ODR_1600HZ Represents an output data rate of 1600 Hz.
 * @param ADXL372_ODR_3200HZ Represents an output data rate of 3200 Hz.
 * @param ADXL372_ODR_6400HZ Represents an output data rate of 6400 Hz.
 ******************************************************************************/
enum adxl372_odr {
	ADXL372_ODR_400HZ,
	ADXL372_ODR_800HZ,
	ADXL372_ODR_1600HZ,
	ADXL372_ODR_3200HZ,
	ADXL372_ODR_6400HZ
};

/***************************************************************************//**
 * @brief The `adxl372_instant_on_th_mode` enumeration defines the threshold
 * modes for the instant-on feature of the ADXL372 accelerometer. This
 * feature allows the device to quickly transition from a low-power state
 * to full operation based on the detected acceleration exceeding a
 * predefined threshold. The enumeration provides two modes: low
 * threshold and high threshold, which determine the sensitivity of the
 * instant-on trigger.
 *
 * @param ADXL372_INSTANT_ON_LOW_TH Represents the low threshold mode for
 * instant-on operation.
 * @param ADXL372_INSTANT_ON_HIGH_TH Represents the high threshold mode for
 * instant-on operation.
 ******************************************************************************/
enum adxl372_instant_on_th_mode {
	ADXL372_INSTANT_ON_LOW_TH,
	ADXL372_INSTANT_ON_HIGH_TH
};

/***************************************************************************//**
 * @brief The `adxl372_wakeup_rate` enumeration defines various wake-up rates
 * for the ADXL372 accelerometer, specifying the time intervals at which
 * the device can be configured to wake up from a low-power state. These
 * rates range from 52 milliseconds to 24576 milliseconds, allowing for
 * flexible power management and responsiveness in applications where the
 * accelerometer is used.
 *
 * @param ADXL372_WUR_52ms Represents a wake-up rate of 52 milliseconds.
 * @param ADXL372_WUR_104ms Represents a wake-up rate of 104 milliseconds.
 * @param ADXL372_WUR_208ms Represents a wake-up rate of 208 milliseconds.
 * @param ADXL372_WUR_512ms Represents a wake-up rate of 512 milliseconds.
 * @param ADXL372_WUR_2048ms Represents a wake-up rate of 2048 milliseconds.
 * @param ADXL372_WUR_4096ms Represents a wake-up rate of 4096 milliseconds.
 * @param ADXL372_WUR_8192ms Represents a wake-up rate of 8192 milliseconds.
 * @param ADXL372_WUR_24576ms Represents a wake-up rate of 24576 milliseconds.
 ******************************************************************************/
enum adxl372_wakeup_rate {
	ADXL372_WUR_52ms,
	ADXL372_WUR_104ms,
	ADXL372_WUR_208ms,
	ADXL372_WUR_512ms,
	ADXL372_WUR_2048ms,
	ADXL372_WUR_4096ms,
	ADXL372_WUR_8192ms,
	ADXL372_WUR_24576ms
};

/***************************************************************************//**
 * @brief The `adxl372_th_activity` enumeration defines the possible threshold
 * activity states for the ADXL372 accelerometer, which include activity,
 * a second activity threshold, and inactivity. These states are used to
 * configure and interpret the accelerometer's response to different
 * levels of motion or lack thereof, allowing for precise control over
 * how the device detects and reacts to movement.
 *
 * @param ADXL372_ACTIVITY Represents the activity threshold state.
 * @param ADXL372_ACTIVITY2 Represents the second activity threshold state.
 * @param ADXL372_INACTIVITY Represents the inactivity threshold state.
 ******************************************************************************/
enum adxl372_th_activity {
	ADXL372_ACTIVITY,
	ADXL372_ACTIVITY2,
	ADXL372_INACTIVITY
};

/***************************************************************************//**
 * @brief The `adxl372_filter_settle` enumeration defines two possible filter
 * settling times for the ADXL372 accelerometer: 370 microseconds and 16
 * microseconds. This setting is used to configure the time it takes for
 * the filter to stabilize after a change in input, which can affect the
 * responsiveness and accuracy of the accelerometer's measurements.
 *
 * @param ADXL372_FILTER_SETTLE_370 Represents a filter settling time of 370
 * microseconds.
 * @param ADXL372_FILTER_SETTLE_16 Represents a filter settling time of 16
 * microseconds.
 ******************************************************************************/
enum adxl372_filter_settle {
	ADXL372_FILTER_SETTLE_370,
	ADXL372_FILTER_SETTLE_16
};

/***************************************************************************//**
 * @brief The `adxl372_fifo_format` enumeration defines the various formats in
 * which data can be stored in the FIFO (First In, First Out) buffer of
 * the ADXL372 accelerometer. Each enumerator specifies a different
 * combination of axis data (X, Y, Z) that can be captured and stored,
 * allowing for flexible data acquisition configurations depending on the
 * application's requirements.
 *
 * @param ADXL372_XYZ_FIFO Represents a FIFO format that includes X, Y, and Z
 * axis data.
 * @param ADXL372_X_FIFO Represents a FIFO format that includes only X axis
 * data.
 * @param ADXL372_Y_FIFO Represents a FIFO format that includes only Y axis
 * data.
 * @param ADXL372_XY_FIFO Represents a FIFO format that includes both X and Y
 * axis data.
 * @param ADXL372_Z_FIFO Represents a FIFO format that includes only Z axis
 * data.
 * @param ADXL372_XZ_FIFO Represents a FIFO format that includes both X and Z
 * axis data.
 * @param ADXL372_YZ_FIFO Represents a FIFO format that includes both Y and Z
 * axis data.
 * @param ADXL372_XYZ_PEAK_FIFO Represents a FIFO format that includes peak data
 * for X, Y, and Z axes.
 ******************************************************************************/
enum adxl372_fifo_format {
	ADXL372_XYZ_FIFO,
	ADXL372_X_FIFO,
	ADXL372_Y_FIFO,
	ADXL372_XY_FIFO,
	ADXL372_Z_FIFO,
	ADXL372_XZ_FIFO,
	ADXL372_YZ_FIFO,
	ADXL372_XYZ_PEAK_FIFO,
};

/***************************************************************************//**
 * @brief The `adxl372_fifo_mode` enumeration defines the different modes of
 * operation for the FIFO (First In, First Out) buffer in the ADXL372
 * accelerometer. These modes determine how data is managed and stored in
 * the FIFO, allowing for different data handling strategies such as
 * bypassing the FIFO, streaming data continuously, capturing data upon a
 * trigger, or saving old data. This flexibility is crucial for adapting
 * the device's data management to various application requirements.
 *
 * @param ADXL372_FIFO_BYPASSED Represents a FIFO mode where the FIFO is
 * bypassed and not used.
 * @param ADXL372_FIFO_STREAMED Represents a FIFO mode where data is
 * continuously streamed into the FIFO.
 * @param ADXL372_FIFO_TRIGGERED Represents a FIFO mode where data is captured
 * in the FIFO upon a trigger event.
 * @param ADXL372_FIFO_OLD_SAVED Represents a FIFO mode where old data is saved
 * in the FIFO.
 ******************************************************************************/
enum adxl372_fifo_mode {
	ADXL372_FIFO_BYPASSED,
	ADXL372_FIFO_STREAMED,
	ADXL372_FIFO_TRIGGERED,
	ADXL372_FIFO_OLD_SAVED
};

/***************************************************************************//**
 * @brief The `adxl372_comm_type` is an enumeration that defines the types of
 * communication protocols available for interfacing with the ADXL372
 * device. It includes two options: SPI (Serial Peripheral Interface) and
 * I2C (Inter-Integrated Circuit), which are common communication
 * protocols used in embedded systems for connecting microcontrollers to
 * peripheral devices.
 *
 * @param SPI Represents the Serial Peripheral Interface communication type.
 * @param I2C Represents the Inter-Integrated Circuit communication type.
 ******************************************************************************/
enum adxl372_comm_type {
	SPI,
	I2C,
};

/***************************************************************************//**
 * @brief The `adxl372_fifo_config` structure is used to configure the FIFO
 * (First-In-First-Out) buffer settings for the ADXL372 accelerometer. It
 * allows the user to specify the operational mode of the FIFO, the
 * format in which data is stored, and the number of samples that the
 * FIFO can hold. This configuration is crucial for managing how
 * acceleration data is buffered and retrieved from the device, enabling
 * efficient data handling and processing in applications that require
 * precise motion detection and analysis.
 *
 * @param fifo_mode Specifies the mode of the FIFO buffer, using the
 * adxl372_fifo_mode enumeration.
 * @param fifo_format Defines the data format of the FIFO buffer, using the
 * adxl372_fifo_format enumeration.
 * @param fifo_samples Indicates the number of samples to be stored in the FIFO
 * buffer, represented as a 16-bit unsigned integer.
 ******************************************************************************/
struct adxl372_fifo_config {
	enum adxl372_fifo_mode fifo_mode;
	enum adxl372_fifo_format fifo_format;
	uint16_t fifo_samples;
};

/***************************************************************************//**
 * @brief The `adxl372_activity_threshold` structure is used to define the
 * parameters for activity threshold detection in the ADXL372
 * accelerometer. It includes a threshold value (`thresh`) that
 * determines the level of acceleration required to trigger an activity
 * event, a `referenced` flag to indicate if the threshold is relative to
 * a baseline, and an `enable` flag to activate or deactivate the
 * threshold detection.
 *
 * @param thresh A 16-bit unsigned integer representing the activity threshold
 * value.
 * @param referenced A boolean indicating if the threshold is referenced to a
 * baseline.
 * @param enable A boolean indicating if the activity threshold is enabled.
 ******************************************************************************/
struct adxl372_activity_threshold {
	uint16_t thresh;
	bool referenced;
	bool enable;
};

/***************************************************************************//**
 * @brief The `adxl372_xyz_accel_data` structure is designed to store
 * acceleration data for the ADXL372 accelerometer sensor along three
 * axes: X, Y, and Z. Each axis is represented by a 16-bit unsigned
 * integer, which allows for capturing the sensor's output data in a
 * compact form. This structure is typically used to hold the raw
 * acceleration data read from the sensor, which can then be processed or
 * analyzed for various applications such as motion detection, vibration
 * analysis, or orientation sensing.
 *
 * @param x Represents the acceleration data along the X-axis.
 * @param y Represents the acceleration data along the Y-axis.
 * @param z Represents the acceleration data along the Z-axis.
 ******************************************************************************/
struct adxl372_xyz_accel_data {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} ;

/***************************************************************************//**
 * @brief The `adxl372_irq_config` structure is used to configure and manage the
 * interrupt settings for the ADXL372 accelerometer. It contains boolean
 * flags that represent various interrupt conditions such as data
 * readiness, FIFO status, activity detection, and power state. This
 * structure allows the user to easily configure and check the status of
 * different interrupt sources on the ADXL372 device.
 *
 * @param data_rdy Indicates if new data is ready.
 * @param fifo_rdy Indicates if the FIFO is ready.
 * @param fifo_full Indicates if the FIFO is full.
 * @param fifo_ovr Indicates if the FIFO has overflowed.
 * @param inactivity Indicates if inactivity is detected.
 * @param activity Indicates if activity is detected.
 * @param awake Indicates if the device is awake.
 * @param low_operation Indicates if the device is in low operation mode.
 ******************************************************************************/
struct adxl372_irq_config {
	bool data_rdy;
	bool fifo_rdy;
	bool fifo_full;
	bool fifo_ovr;
	bool inactivity;
	bool activity;
	bool awake;
	bool low_operation;
};

struct adxl372_dev;

typedef int32_t (*adxl372_reg_read_func)(struct adxl372_dev *dev,
		uint8_t reg_addr,
		uint8_t *reg_data);
typedef int32_t (*adxl372_reg_write_func)(struct adxl372_dev *dev,
		uint8_t reg_addr,
		uint8_t reg_data);
typedef int32_t (*adxl372_reg_read_multi_func)(struct adxl372_dev *dev,
		uint8_t reg_addr,
		uint8_t *reg_data,
		uint16_t count);

/***************************************************************************//**
 * @brief The `adxl372_dev` structure is a comprehensive representation of the
 * ADXL372 accelerometer device, encapsulating all necessary components
 * for its operation. It includes descriptors for SPI and I2C
 * communication, GPIO descriptors for interrupt handling, and function
 * pointers for register read and write operations. Additionally, it
 * holds configuration settings such as bandwidth, output data rate,
 * wake-up rate, activity processing mode, instant-on threshold mode, and
 * FIFO configuration, along with the communication type used. This
 * structure is essential for managing the device's settings and
 * interfacing with it through various communication protocols.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param i2c_desc Pointer to an I2C descriptor for I2C communication.
 * @param gpio_int1 Pointer to a GPIO descriptor for the first interrupt pin.
 * @param gpio_int2 Pointer to a GPIO descriptor for the second interrupt pin.
 * @param reg_read Function pointer for reading a register from the device.
 * @param reg_write Function pointer for writing to a register on the device.
 * @param reg_read_multiple Function pointer for reading multiple registers from
 * the device.
 * @param bw Enum specifying the bandwidth setting of the device.
 * @param odr Enum specifying the output data rate of the device.
 * @param wur Enum specifying the wake-up rate of the device.
 * @param act_proc_mode Enum specifying the activity processing mode of the
 * device.
 * @param th_mode Enum specifying the instant-on threshold mode of the device.
 * @param fifo_config Structure containing FIFO configuration settings.
 * @param comm_type Enum specifying the communication type (SPI or I2C).
 ******************************************************************************/
struct adxl372_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* I2C */
	struct no_os_i2c_desc			*i2c_desc;
	/* GPIO */
	struct no_os_gpio_desc		*gpio_int1;
	struct no_os_gpio_desc		*gpio_int2;
	/* Device Settings */
	adxl372_reg_read_func		reg_read;
	adxl372_reg_write_func		reg_write;
	adxl372_reg_read_multi_func	reg_read_multiple;
	enum adxl372_bandwidth		bw;
	enum adxl372_odr		odr;
	enum adxl372_wakeup_rate	wur;
	enum adxl372_act_proc_mode	act_proc_mode;
	enum adxl372_instant_on_th_mode	th_mode;
	struct adxl372_fifo_config	fifo_config;
	enum adxl372_comm_type		comm_type;
};

/***************************************************************************//**
 * @brief The `adxl372_init_param` structure is used to initialize and configure
 * the ADXL372 accelerometer device. It includes parameters for setting
 * up communication interfaces (SPI and I2C), GPIO configurations for
 * interrupts, and various device settings such as bandwidth, output data
 * rate, wake-up rate, and operational modes. Additionally, it allows for
 * the configuration of activity and inactivity thresholds, filter
 * settling, FIFO settings, and interrupt configurations, providing a
 * comprehensive setup for the device's operation.
 *
 * @param spi_init Initializes SPI communication parameters.
 * @param i2c_init Initializes I2C communication parameters.
 * @param gpio_int1 Configures GPIO for interrupt 1.
 * @param gpio_int2 Configures GPIO for interrupt 2.
 * @param bw Sets the bandwidth for the device.
 * @param odr Specifies the output data rate.
 * @param wur Defines the wake-up rate for the device.
 * @param act_proc_mode Determines the activity processing mode.
 * @param th_mode Sets the threshold mode for instant-on.
 * @param activity_th Configures the activity threshold settings.
 * @param activity2_th Configures the secondary activity threshold settings.
 * @param inactivity_th Configures the inactivity threshold settings.
 * @param activity_time Specifies the duration for activity detection.
 * @param inactivity_time Specifies the duration for inactivity detection.
 * @param filter_settle Sets the filter settling time.
 * @param fifo_config Configures the FIFO settings.
 * @param int1_config Configures interrupt 1 settings.
 * @param int2_config Configures interrupt 2 settings.
 * @param op_mode Defines the operational mode of the device.
 * @param comm_type Specifies the communication type (SPI or I2C).
 ******************************************************************************/
struct adxl372_init_param {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* I2C */
	struct no_os_i2c_init_param			i2c_init;
	/* GPIO */
	struct no_os_gpio_init_param			gpio_int1;
	struct no_os_gpio_init_param			gpio_int2;
	/* Device Settings */
	enum adxl372_bandwidth			bw;
	enum adxl372_odr			odr;
	enum adxl372_wakeup_rate		wur;
	enum adxl372_act_proc_mode		act_proc_mode;
	enum adxl372_instant_on_th_mode		th_mode;
	struct adxl372_activity_threshold	activity_th;
	struct adxl372_activity_threshold	activity2_th;
	struct adxl372_activity_threshold	inactivity_th;
	uint8_t					activity_time;
	uint16_t				inactivity_time;
	enum adxl372_filter_settle		filter_settle;
	struct adxl372_fifo_config		fifo_config;
	struct adxl372_irq_config		int1_config;
	struct adxl372_irq_config		int2_config;
	enum adxl372_op_mode			op_mode;
	enum adxl372_comm_type			comm_type;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief Use this function to read a single byte from a specified register of
 * the ADXL372 accelerometer using SPI communication. It is essential to
 * ensure that the device has been properly initialized and configured
 * for SPI communication before calling this function. The function will
 * attempt to read the register and store the result in the provided
 * memory location. If the read operation fails, the function will return
 * an error code.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the ADXL372.
 * @param reg_data A pointer to a uint8_t where the read data will be stored.
 * Must not be null.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_spi_reg_read(struct adxl372_dev *dev,
			     uint8_t reg_addr,
			     uint8_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to read a sequence of registers from the ADXL372
 * accelerometer using SPI communication. It is useful when you need to
 * retrieve multiple consecutive register values in a single operation.
 * The function requires a valid device structure and a starting register
 * address. The number of registers to read is specified by the count
 * parameter, which must not exceed 512. The function will populate the
 * provided buffer with the read data. It is important to ensure that the
 * device has been properly initialized and configured for SPI
 * communication before calling this function.
 *
 * @param dev A pointer to an adxl372_dev structure representing the device.
 * Must not be null and should be properly initialized for SPI
 * communication.
 * @param reg_addr The starting register address from which to begin reading.
 * Must be a valid register address within the device's
 * addressable range.
 * @param reg_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should have enough space to hold 'count'
 * bytes of data.
 * @param count The number of registers to read. Must be between 1 and 512
 * inclusive. If greater than 512, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t adxl372_spi_reg_read_multiple(struct adxl372_dev *dev,
				      uint8_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count);
/***************************************************************************//**
 * @brief Use this function to write a single byte of data to a specific
 * register on the ADXL372 device via SPI communication. This function
 * requires a valid device structure that has been properly initialized
 * with SPI settings. It is typically used to configure device settings
 * or control registers. Ensure that the device is in a state where it
 * can accept SPI commands before calling this function.
 *
 * @param dev A pointer to an adxl372_dev structure representing the device.
 * Must not be null and should be properly initialized with SPI
 * settings.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the ADXL372 device.
 * @param reg_data The data byte to write to the specified register. It is
 * masked to ensure only the lower 8 bits are used.
 * @return Returns an int32_t status code from the SPI write operation, where 0
 * indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t adxl372_spi_reg_write(struct adxl372_dev *dev,
			      uint8_t reg_addr,
			      uint8_t reg_data);
/***************************************************************************//**
 * @brief This function reads a single byte from a specified register of the
 * ADXL372 accelerometer using the I2C communication protocol. It is
 * typically used to retrieve configuration or status information from
 * the device. The function requires a valid device structure that has
 * been properly initialized with I2C settings. The register address must
 * be within the valid range of the device's register map. The function
 * writes the read value into the provided memory location pointed to by
 * `reg_data`. If the I2C communication fails, the function returns a
 * negative error code.
 *
 * @param dev A pointer to an initialized `adxl372_dev` structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the ADXL372's register map.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_i2c_reg_read(struct adxl372_dev *dev,
			     uint8_t reg_addr,
			     uint8_t *reg_data);
/***************************************************************************//**
 * @brief Use this function to write a single byte of data to a specific
 * register of the ADXL372 device via the I2C interface. This function is
 * typically called when configuring the device or updating its settings.
 * Ensure that the device has been properly initialized and that the I2C
 * communication is set up before calling this function. The function
 * returns an error code if the write operation fails.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the ADXL372.
 * @param reg_data The data byte to write to the specified register. The value
 * is masked to 8 bits.
 * @return Returns an int32_t error code indicating the success or failure of
 * the I2C write operation.
 ******************************************************************************/
int32_t adxl372_i2c_reg_write(struct adxl372_dev *dev,
			      uint8_t reg_addr,
			      uint8_t reg_data);
/***************************************************************************//**
 * @brief This function is used to read a sequence of registers from the ADXL372
 * accelerometer using the I2C communication protocol. It is typically
 * called when multiple consecutive register values are needed, such as
 * when reading sensor data or configuration settings. The function
 * requires a valid device structure and a starting register address. The
 * caller must ensure that the buffer provided for storing the read data
 * is large enough to hold the specified number of bytes. The function
 * will return an error if the requested byte count exceeds 512, which is
 * the maximum buffer size. It is important to check the return value to
 * handle any communication errors.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The starting register address from which to begin reading.
 * Must be a valid register address for the ADXL372.
 * @param reg_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold 'count'
 * bytes.
 * @param count The number of bytes to read from the device. Must be less than
 * or equal to 512. If greater, the function returns an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_i2c_reg_read_multiple(struct adxl372_dev *dev,
				      uint8_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * ADXL372 device by applying a mask. It first reads the current value of
 * the register, applies the mask to clear specific bits, and then writes
 * the new data to those bits. This function is typically used when only
 * certain bits of a register need to be updated without affecting the
 * other bits. It must be called with a valid device structure and
 * register address. The function handles errors by returning a negative
 * value if the read or write operation fails.
 *
 * @param dev A pointer to an adxl372_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the ADXL372 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The data to be written to the register, after applying the mask.
 * Only the bits specified by the mask will be updated.
 * @return Returns 0 on success, or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int32_t adxl372_write_mask(struct adxl372_dev *dev,
			   uint8_t reg_addr,
			   uint32_t mask,
			   uint8_t data);
/***************************************************************************//**
 * @brief This function configures the activity threshold for a specified
 * activity type on the ADXL372 accelerometer device. It should be used
 * to set the threshold value that determines when an activity event is
 * detected. The function allows specifying whether the threshold is
 * referenced and whether it should be enabled. It must be called with a
 * valid device structure and appropriate threshold parameters. The
 * function returns an error code if the operation fails, which can occur
 * if the device is not properly initialized or if communication with the
 * device fails.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param act An enum value of type adxl372_th_activity indicating the type of
 * activity threshold to set (e.g., ADXL372_ACTIVITY,
 * ADXL372_ACTIVITY2, ADXL372_INACTIVITY).
 * @param thresh A 16-bit unsigned integer representing the threshold value. The
 * valid range is device-specific and should be set according to
 * the desired sensitivity.
 * @param referenced A boolean indicating whether the threshold is referenced.
 * True if referenced, false otherwise.
 * @param enable A boolean indicating whether to enable the threshold. True to
 * enable, false to disable.
 * @return Returns an int32_t error code: 0 on success, or a negative value if
 * an error occurs.
 ******************************************************************************/
int32_t adxl372_set_activity_threshold(struct adxl372_dev *dev,
				       enum adxl372_th_activity act,
				       uint16_t thresh,
				       bool referenced,
				       bool enable);
/***************************************************************************//**
 * @brief Use this function to configure the ADXL372 accelerometer to operate in
 * a specific mode, such as standby, wake-up, instant-on, or full
 * bandwidth measurement. This function should be called after the device
 * has been initialized and before starting any measurement operations.
 * It modifies the device's power control settings to reflect the desired
 * operational mode.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param op_mode An enum value of type adxl372_op_mode specifying the desired
 * operational mode. Valid values are ADXL372_STANDBY,
 * ADXL372_WAKE_UP, ADXL372_INSTANT_ON, and
 * ADXL372_FULL_BW_MEASUREMENT.
 * @return Returns an int32_t indicating success (0) or a negative error code if
 * the operation fails.
 ******************************************************************************/
int32_t adxl372_set_op_mode(struct adxl372_dev *dev,
			    enum adxl372_op_mode op_mode);
/***************************************************************************//**
 * @brief This function is used to control the autosleep feature of the ADXL372
 * accelerometer device. Autosleep mode allows the device to
 * automatically enter a low-power state when inactivity is detected,
 * conserving energy. This function should be called when the device is
 * already initialized and configured. It modifies the device's
 * measurement settings to either enable or disable the autosleep
 * functionality based on the provided parameter. Proper error handling
 * should be implemented by the caller to manage any issues that arise
 * during the operation.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null, and the device should be properly
 * configured before calling this function.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the autosleep feature.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t adxl372_set_autosleep(struct adxl372_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function configures the bandwidth setting of the ADXL372
 * accelerometer device. It should be called when you need to adjust the
 * bandwidth to match the desired data rate and noise performance for
 * your application. The function requires a valid device structure and a
 * bandwidth enumeration value. It updates the device's bandwidth setting
 * and returns an error code if the operation fails. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param bw An enumeration value of type adxl372_bandwidth specifying the
 * desired bandwidth setting. Valid values are defined in the
 * adxl372_bandwidth enum.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_set_bandwidth(struct adxl372_dev *dev,
			      enum adxl372_bandwidth bw);
/***************************************************************************//**
 * @brief This function configures the activity processing mode of the ADXL372
 * accelerometer device. It should be called when the user needs to
 * change the mode of activity processing, which can affect how the
 * device handles activity detection. The function must be called with a
 * valid device structure and a valid mode enumeration. It returns an
 * error code if the operation fails, which can be used to diagnose
 * issues with the device communication or configuration.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param mode An enumeration value of type adxl372_act_proc_mode specifying the
 * desired activity processing mode. Valid values are
 * ADXL372_DEFAULT, ADXL372_LINKED, and ADXL372_LOOPED.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_set_act_proc_mode(struct adxl372_dev *dev,
				  enum adxl372_act_proc_mode mode);
/***************************************************************************//**
 * @brief This function configures the output data rate (ODR) of the ADXL372
 * accelerometer device. It should be called when you need to change the
 * data sampling rate of the device, which can be useful for optimizing
 * power consumption or data resolution based on application needs. The
 * function requires a valid device structure and a specified ODR value.
 * It updates the device's internal settings and returns an error code if
 * the operation fails. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param odr An enum value of type adxl372_odr representing the desired output
 * data rate. Must be a valid ODR value defined in the adxl372_odr
 * enumeration.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_set_odr(struct adxl372_dev *dev,
			enum adxl372_odr odr);
/***************************************************************************//**
 * @brief This function configures the instant-on threshold mode of the ADXL372
 * accelerometer device. It should be called when the device is in a
 * state where configuration changes are allowed, typically during
 * initialization or when the device is not actively measuring. The
 * function updates the device's internal configuration to reflect the
 * specified threshold mode. It is important to ensure that the device
 * pointer is valid and properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an adxl372_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param mode An enum value of type adxl372_instant_on_th_mode specifying the
 * desired instant-on threshold mode. Valid values are
 * ADXL372_INSTANT_ON_LOW_TH and ADXL372_INSTANT_ON_HIGH_TH.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_set_instant_on_th(struct adxl372_dev *dev,
				  enum adxl372_instant_on_th_mode mode);
/***************************************************************************//**
 * @brief This function configures the wake-up rate of the ADXL372 accelerometer
 * device, which determines how frequently the device wakes up to check
 * for activity. It should be called when the device is in a
 * configuration state, typically during initialization or when changing
 * operational parameters. The function updates the device's internal
 * settings to reflect the new wake-up rate. It is important to ensure
 * that the device structure is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null, and the device must be properly
 * configured before use.
 * @param wur An enum value of type adxl372_wakeup_rate representing the desired
 * wake-up rate. Valid values are defined by the adxl372_wakeup_rate
 * enumeration.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_set_wakeup_rate(struct adxl372_dev *dev,
				enum adxl372_wakeup_rate wur);
/***************************************************************************//**
 * @brief This function sets the activity time for the ADXL372 accelerometer
 * device, which determines the duration for which activity must be
 * detected before it is considered valid. It should be called when
 * configuring the device for activity detection. Ensure that the device
 * structure is properly initialized before calling this function. The
 * function writes the specified time value to the appropriate register
 * in the device.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param time An 8-bit unsigned integer representing the activity time to be
 * set. Valid values depend on the device's specifications.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adxl372_set_activity_time(struct adxl372_dev *dev, uint8_t time);
/***************************************************************************//**
 * @brief This function configures the inactivity time for the ADXL372
 * accelerometer device, which determines the duration the device must
 * remain inactive before triggering an inactivity event. It should be
 * called when the device is initialized and configured for operation.
 * The function writes the specified time value to the appropriate
 * registers in the device. Ensure that the device structure is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param time A 16-bit unsigned integer representing the inactivity time to be
 * set. The value is split into two bytes and written to the
 * device's registers.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_set_inactivity_time(struct adxl372_dev *dev, uint16_t time);
/***************************************************************************//**
 * @brief Use this function to configure the interrupt settings for the ADXL372
 * accelerometer device. It allows you to specify the interrupt
 * conditions for two interrupt pins, INT1 and INT2, by providing
 * configurations for each. This function should be called after the
 * device has been initialized and before enabling interrupts. It writes
 * the specified configurations to the device registers and returns an
 * error code if the operation fails.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param int1 A struct adxl372_irq_config specifying the interrupt
 * configuration for the INT1 pin. Each field is a boolean
 * indicating whether a specific interrupt condition is enabled.
 * @param int2 A struct adxl372_irq_config specifying the interrupt
 * configuration for the INT2 pin. Each field is a boolean
 * indicating whether a specific interrupt condition is enabled.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_interrupt_config(struct adxl372_dev *dev,
				 struct adxl372_irq_config int1,
				 struct adxl372_irq_config int2);
/***************************************************************************//**
 * @brief This function configures the filter settling mode of the ADXL372
 * accelerometer device. It should be called when you need to adjust the
 * filter settling behavior, typically during device initialization or
 * when changing operational modes. Ensure that the device structure is
 * properly initialized before calling this function. The function
 * modifies the device's power control register to set the desired filter
 * settling mode.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param mode An enum value of type adxl372_filter_settle indicating the
 * desired filter settling mode. Valid values are
 * ADXL372_FILTER_SETTLE_370 and ADXL372_FILTER_SETTLE_16.
 * @return Returns an int32_t status code. A value of 0 indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int32_t adxl372_set_filter_settle(struct adxl372_dev *dev,
				  enum adxl372_filter_settle mode);
/***************************************************************************//**
 * @brief Use this function to obtain the current status and FIFO entry count
 * from an ADXL372 device. It is essential to call this function after
 * the device has been properly initialized and configured. The function
 * reads multiple registers from the device to gather status information
 * and the number of valid data samples in the FIFO. Ensure that the
 * device pointer is valid and that the status and FIFO entry pointers
 * are not null to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param status1 A pointer to a uint8_t variable where the first status byte
 * will be stored. Must not be null.
 * @param status2 A pointer to a uint8_t variable where the second status byte
 * will be stored. Must not be null.
 * @param fifo_entries A pointer to a uint16_t variable where the number of FIFO
 * entries will be stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t adxl372_get_status(struct adxl372_dev *dev,
			   uint8_t *status1,
			   uint8_t *status2,
			   uint16_t *fifo_entries);
/***************************************************************************//**
 * @brief Use this function to reset the ADXL372 device, which is typically
 * necessary when initializing the device or recovering from an error
 * state. The function sets the device to standby mode before issuing a
 * reset command. It is important to ensure that the device structure is
 * properly initialized before calling this function. The function will
 * return an error code if the operation fails, which should be checked
 * by the caller to ensure the reset was successful.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * structure.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t adxl372_reset(struct adxl372_dev *dev);
/***************************************************************************//**
 * @brief This function sets up the FIFO mode, format, and sample count for the
 * ADXL372 accelerometer. It must be called when the device is in standby
 * mode to ensure proper configuration. The function checks if the number
 * of FIFO samples is within the valid range and returns an error if it
 * exceeds the maximum allowed value. It updates the device's internal
 * configuration to reflect the new FIFO settings.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param mode The desired FIFO mode, specified as an enum adxl372_fifo_mode.
 * Valid modes are defined in the adxl372_fifo_mode enumeration.
 * @param format The desired FIFO format, specified as an enum
 * adxl372_fifo_format. Valid formats are defined in the
 * adxl372_fifo_format enumeration.
 * @param fifo_samples The number of samples to store in the FIFO, as a
 * uint16_t. Must be 512 or less; otherwise, the function
 * returns an error.
 * @return Returns 0 on success, or a negative error code if the configuration
 * fails or if the input parameters are invalid.
 ******************************************************************************/
int32_t adxl372_configure_fifo(struct adxl372_dev *dev,
			       enum adxl372_fifo_mode mode,
			       enum adxl372_fifo_format format,
			       uint16_t fifo_samples);
/***************************************************************************//**
 * @brief This function reads a specified number of XYZ acceleration data
 * samples from the FIFO buffer of an ADXL372 device. It is essential to
 * ensure that the device is properly initialized and configured to use
 * the FIFO before calling this function. The function reads up to 512
 * samples, and each sample consists of XYZ data. If the requested sample
 * count exceeds the FIFO capacity, the function returns an error. This
 * function is useful for applications that require batch processing of
 * acceleration data.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param samples A pointer to an array of adxl372_xyz_accel_data structures
 * where the retrieved samples will be stored. Must not be null
 * and should have enough space to hold 'cnt' samples.
 * @param cnt The number of XYZ samples to read from the FIFO. Must be between 1
 * and 512, inclusive. If greater than 512, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if 'cnt' is greater than 512.
 ******************************************************************************/
int32_t adxl372_get_fifo_xyz_data(struct adxl372_dev *dev,
				  struct adxl372_xyz_accel_data *fifo_data,
				  uint16_t cnt);
/***************************************************************************//**
 * @brief This function is used to handle FIFO events for the ADXL372
 * accelerometer device. It should be called when the FIFO is ready or
 * full, as indicated by the device status. The function checks for FIFO
 * overrun conditions and reads available FIFO data if the FIFO is not
 * bypassed. It is important to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * modifies the number of FIFO entries to reflect the number of samples
 * read.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null.
 * @param fifo_data A pointer to a buffer where the function will store the read
 * FIFO data. The buffer must be large enough to hold the data
 * for the specified number of entries. Must not be null.
 * @param fifo_entries A pointer to a uint16_t variable that initially contains
 * the number of entries to read from the FIFO. The function
 * updates this value to reflect the actual number of
 * entries read. Must not be null.
 * @return Returns an int32_t indicating success (0) or an error code. If a FIFO
 * overrun is detected, it returns -1.
 ******************************************************************************/
int32_t adxl372_service_fifo_ev(struct adxl372_dev *dev,
				struct adxl372_xyz_accel_data *fifo_data,
				uint16_t *fifo_entries);
/***************************************************************************//**
 * @brief Use this function to obtain the highest peak acceleration data
 * recorded by the ADXL372 accelerometer. It should be called when the
 * device is ready to provide data, typically after initialization and
 * configuration. The function waits for the data to be ready and then
 * reads the peak values for the x, y, and z axes. Ensure that the device
 * structure is properly initialized before calling this function. The
 * function returns an error code if the data cannot be retrieved.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param max_peak A pointer to an adxl372_xyz_accel_data structure where the
 * function will store the highest peak acceleration data. Must
 * not be null. The caller provides the memory for this
 * structure.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t adxl372_get_highest_peak_data(struct adxl372_dev *dev,
				      struct adxl372_xyz_accel_data *max_peak);
/***************************************************************************//**
 * @brief Use this function to obtain the current acceleration data from an
 * ADXL372 device. It should be called when new data is ready, as
 * indicated by the device's status. The function requires a valid device
 * structure and a pre-allocated structure to store the acceleration
 * data. It returns an error code if the data cannot be retrieved, which
 * should be handled by the caller.
 *
 * @param dev A pointer to an initialized adxl372_dev structure representing the
 * device. Must not be null. The device should be properly configured
 * and operational.
 * @param accel_data A pointer to an adxl372_xyz_accel_data structure where the
 * acceleration data will be stored. Must not be null. The
 * caller is responsible for allocating this structure.
 * @return Returns an int32_t value indicating success (0) or an error code (<0)
 * if the operation fails.
 ******************************************************************************/
int32_t adxl372_get_accel_data(struct adxl372_dev *dev,
			       struct adxl372_xyz_accel_data *accel_data);
/***************************************************************************//**
 * @brief This function sets up the ADXL372 accelerometer device using the
 * provided initialization parameters, configuring communication
 * interfaces, GPIOs, and device settings. It must be called before any
 * other operations on the device. The function handles both SPI and I2C
 * communication types and configures the device according to the
 * specified operational mode, bandwidth, output data rate, and other
 * settings. If initialization fails, the function returns an error code
 * and the device pointer is not valid.
 *
 * @param device A pointer to a pointer of type struct adxl372_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A struct adxl372_init_param containing initialization
 * parameters such as communication type, GPIO configurations,
 * and device settings. Must be properly configured before
 * calling the function.
 * @return Returns an int32_t value. A return value of 0 indicates success,
 * while a negative value indicates an error during initialization.
 ******************************************************************************/
int32_t adxl372_init(struct adxl372_dev **device,
		     struct adxl372_init_param init_param);

#endif // ADXL372_H_
