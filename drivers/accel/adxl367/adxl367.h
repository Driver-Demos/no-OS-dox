/***************************************************************************//**
 *   @file   ADXL367.h
 *   @brief  Header file of ADXL367 Driver.
 *   @author Andrei Porumb (andrei.porumb@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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

#ifndef __ADXL367_H__
#define __ADXL367_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_i2c.h"

/******************************************************************************/
/********************************* ADXL367 ************************************/
/******************************************************************************/

/* ADXL367 communication commands */
#define ADXL367_WRITE_REG               0x0A
#define ADXL367_READ_REG                0x0B
#define ADXL367_READ_FIFO               0x0D
#define ADXL367_I2C_READ		0x01
#define ADXL367_I2C_WRITE		0x00

/* Registers */
#define ADXL367_REG_DEVID_AD            0x00
#define ADXL367_REG_DEVID_MST           0x01
#define ADXL367_REG_PARTID              0x02
#define ADXL367_REG_REVID               0x03
#define ADXL367_REG_SERIAL_NUMBER_3	0x04
#define ADXL367_REG_SERIAL_NUMBER_2	0x05
#define ADXL367_REG_SERIAL_NUMBER_1	0x06
#define ADXL367_REG_SERIAL_NUMBER_0	0x07
#define ADXL367_REG_XDATA               0x08
#define ADXL367_REG_YDATA               0x09
#define ADXL367_REG_ZDATA               0x0A
#define ADXL367_REG_STATUS              0x0B
#define ADXL367_REG_FIFO_ENTRIES_L      0x0C
#define ADXL367_REG_FIFO_ENTRIES_H      0x0D
#define ADXL367_REG_XDATA_H             0x0E
#define ADXL367_REG_XDATA_L             0x0F
#define ADXL367_REG_YDATA_H             0x10
#define ADXL367_REG_YDATA_L             0x11
#define ADXL367_REG_ZDATA_H             0x12
#define ADXL367_REG_ZDATA_L             0x13
#define ADXL367_REG_TEMP_H              0x14
#define ADXL367_REG_TEMP_L              0x15
#define ADXL367_REG_EX_ADC_H		0x16
#define ADXL367_REG_EX_ADC_L		0x17
#define ADXL367_REG_I2C_FIFO_DATA	0x18
#define ADXL367_REG_SOFT_RESET          0x1F
#define ADXL367_REG_THRESH_ACT_H        0x20
#define ADXL367_REG_THRESH_ACT_L        0x21
#define ADXL367_REG_TIME_ACT            0x22
#define ADXL367_REG_THRESH_INACT_H      0x23
#define ADXL367_REG_THRESH_INACT_L      0x24
#define ADXL367_REG_TIME_INACT_H        0x25
#define ADXL367_REG_TIME_INACT_L        0x26
#define ADXL367_REG_ACT_INACT_CTL       0x27
#define ADXL367_REG_FIFO_CONTROL	0x28
#define ADXL367_REG_FIFO_SAMPLES        0x29
#define ADXL367_REG_INTMAP1_LWR		0x2A
#define ADXL367_REG_INTMAP2_LWR		0x2B
#define ADXL367_REG_FILTER_CTL          0x2C
#define ADXL367_REG_POWER_CTL           0x2D
#define ADXL367_REG_SELF_TEST           0x2E
#define ADXL367_REG_TAP_THRESH          0x2F
#define ADXL367_REG_TAP_DUR		0x30
#define ADXL367_REG_TAP_LATENT		0x31
#define ADXL367_REG_TAP_WINDOW		0x32
#define ADXL367_REG_X_OFFSET		0x33
#define ADXL367_REG_Y_OFFSET		0x34
#define ADXL367_REG_Z_OFFSET		0x35
#define ADXL367_REG_X_SENS		0x36
#define ADXL367_REG_Y_SENS		0x37
#define ADXL367_REG_Z_SENS		0x38
#define ADXL367_REG_TIMER_CTL           0x39
#define ADXL367_REG_INTMAP1_UPPER       0x3A
#define ADXL367_REG_INTMAP2_UPPER	0x3B
#define ADXL367_REG_ADC_CTL		0x3C
#define ADXL367_REG_TEMP_CTL		0x3D
#define ADXL367_REG_TEMP_ADC_OV_TH_H	0x3E
#define ADXL367_REG_TEMP_ADC_OV_TH_L    0x3F
#define ADXL367_REG_TEMP_ADC_UN_TH_H    0x40
#define ADXL367_REG_TEMP_ADC_UN_TH_L	0x41
#define ADXL367_REG_TEMP_ADC_TIMER      0x42
#define ADXL367_REG_AXIS_MASK		0x43
#define ADXL367_REG_STATUS_COPY         0x44
#define ADXL367_REG_STATUS_2            0x45

/* ADXL367_REG_STATUS definitions */
#define ADXL367_STATUS_ERR_USER_REGS    NO_OS_BIT(7)
#define ADXL367_STATUS_AWAKE            NO_OS_BIT(6)
#define ADXL367_STATUS_INACT            NO_OS_BIT(5)
#define ADXL367_STATUS_ACT              NO_OS_BIT(4)
#define ADXL367_STATUS_FIFO_OVERRUN     NO_OS_BIT(3)
#define ADXL367_STATUS_FIFO_WATERMARK   NO_OS_BIT(2)
#define ADXL367_STATUS_FIFO_RDY         NO_OS_BIT(1)
#define ADXL367_STATUS_DATA_RDY         NO_OS_BIT(0)

/* ADXL367_REG_THRESH_H mask */
#define ADXL367_THRESH_H		0x7F

/* ADXL367_REG_THRESH_L mask */
#define ADXL367_THRESH_L		0xFC

/* ADXL367_REG_ACT_INACT_CTL definitions */
#define ADXL367_ACT_INACT_CTL_LINKLOOP_MSK	NO_OS_GENMASK(5, 4)
#define ADXL367_ACT_INACT_CTL_INACT_EN_MSK	NO_OS_GENMASK(3, 2)
#define ADXL367_ACT_INACT_CTL_ACT_EN_MSK	NO_OS_GENMASK(1, 0)

/* ADXL367_ACT_INACT_CTL_INACT_EN(x) options */
#define ADXL367_NO_INACTIVITY_DETECTION_ENABLED		0x0
#define ADXL367_INACTIVITY_ENABLE			0x1
#define ADXL367_NO_INACTIVITY_DETECTION_ENABLED_2	0x2
#define ADXL367_REFERENCED_INACTIVITY_ENABLE		0x3

/* ADXL367_ACT_INACT_CTL_ACT_EN(x) options */
#define ADXL367_NO_ACTIVITY_DETECTION		0x0
#define ADXL367_ACTIVITY_ENABLE			0x1
#define ADXL367_NO_ACTIVITY_DETECTION_2		0x2
#define ADXL367_REFERENCED_ACTIVITY_ENABLE	0x3

/* ADXL367_REG_FIFO_CONTROL */
#define ADXL367_FIFO_CONTROL_FIFO_CHANNEL_MSK	NO_OS_GENMASK(6, 3)
#define ADXL367_FIFO_CONTROL_FIFO_SAMPLES	NO_OS_BIT(2)
#define ADXL367_FIFO_CONTROL_FIFO_MODE_MSK	NO_OS_GENMASK(1, 0)

/* ADXL367_FIFO_CONTROL_FIFO_CHANNEL(x) options */
#define ADXL367_ALL_AXIS		0x0
#define ADXL367_X_AXIS			0x1
#define ADXL367_Y_AXIS			0x2
#define ADXL367_X_AXIS_2		0x3
#define ADXL367_ALL_AXIS_TEMP		0x4
#define ADXL367_X_AXIS_TEMP		0x5
#define ADXL367_Y_AXIS_TEMP		0x6
#define ADXL367_Z_AXIS_TEMP		0x7
#define ADXL367_ALL_AXIS_EXT_ADC	0x8
#define ADXL367_X_AXIS_EXT_ADC		0x9
#define ADXL367_Y_AXIS_EXT_ADC		0xA
#define ADXL367_Z_AXIS_EXT_ADC		0xB

/* ADXL367_FIFO_CONTROL_FIFO_MODE(x) options */
#define ADXL367_FIFO_DISABLE            0
#define ADXL367_FIFO_OLDEST_SAVED       1
#define ADXL367_FIFO_STREAM             2
#define ADXL367_FIFO_TRIGGERED          3

/* ADXL367_REG_INTMAP1_LOWER */
#define ADXL367_INTMAP1_INT_LOW_INT1		NO_OS_BIT(7)
#define ADXL367_INTMAP1_AWAKE_INT1		NO_OS_BIT(6)
#define ADXL367_INTMAP1_INACT_INT1		NO_OS_BIT(5)
#define ADXL367_INTMAP1_ACT_INT1		NO_OS_BIT(4)
#define ADXL367_INTMAP1_FIFO_OVERRUN_INT1	NO_OS_BIT(3)
#define ADXL367_INTMAP1_FIFO_WATERMARK_INT1	NO_OS_BIT(2)
#define ADXL367_INTMAP1_FIFO_RDY_INT1		NO_OS_BIT(1)
#define ADXL367_INTMAP1_DATA_RDY_INT1		NO_OS_BIT(0)

/* ADXL367_REG_INTMAP2_LOWER definitions */
#define ADXL367_INTMAP2_INT_LOW_INT2		NO_OS_BIT(7)
#define ADXL367_INTMAP2_AWAKE_INT2		NO_OS_BIT(6)
#define ADXL367_INTMAP2_INACT_INT2		NO_OS_BIT(5)
#define ADXL367_INTMAP2_ACT_INT2		NO_OS_BIT(4)
#define ADXL367_INTMAP2_FIFO_OVERRUN_INT2	NO_OS_BIT(3)
#define ADXL367_INTMAP2_FIFO_WATERMARK_INT2	NO_OS_BIT(2)
#define ADXL367_INTMAP2_FIFO_RDY_INT2		NO_OS_BIT(1)
#define ADXL367_INTMAP2_DATA_RDY_INT2		NO_OS_BIT(0)

/* ADXL367_REG_FILTER_CTL definitions */
#define ADXL367_FILTER_CTL_RANGE_MSK	NO_OS_GENMASK(7, 6)
#define ADXL367_FILTER_I2C_HS		NO_OS_BIT(5)
#define ADXL367_FILTER_CTL_RES		NO_OS_BIT(4)
#define ADXL367_FILTER_CTL_EXT_SAMPLE   NO_OS_BIT(3)
#define ADXL367_FILTER_CTL_ODR_MSK		NO_OS_GENMASK(2, 0)

/* ADXL367_FILTER_CTL_RANGE(x) options */
#define ADXL367_RANGE_2G                0 /* +/-2 g */
#define ADXL367_RANGE_4G                1 /* +/-4 g */
#define ADXL367_RANGE_8G                2 /* +/-8 g */

/* ADXL367_REG_POWER_CTL definitions */
#define ADXL367_POWER_CTL_RES           NO_OS_BIT(7)
#define ADXL367_POWER_CTL_EXT_CLK       NO_OS_BIT(6)
#define ADXL367_POWER_CTL_LOW_NOISE_MSK	NO_OS_GENMASK(5, 4)
#define ADXL367_POWER_CTL_WAKEUP        NO_OS_BIT(3)
#define ADXL367_POWER_CTL_AUTOSLEEP     NO_OS_BIT(2)
#define ADXL367_POWER_CTL_MEASURE_MSK	NO_OS_GENMASK(1, 0)

/* ADXL367_POWER_CTL_NOISE(x) options */
#define ADXL367_NOISE_MODE_NORMAL       0
#define ADXL367_NOISE_MODE_LOW          1
#define ADXL367_NOISE_MODE_ULTRALOW     2

/* ADXL367_REG_SELF_TEST */
#define ADXL367_SELF_TEST_ST_FORCE      NO_OS_BIT(1)
#define ADXL367_SELF_TEST_ST            NO_OS_BIT(0)

/* XYZ_AXIS_OFFSET MASK */
#define ADXL367_XYZ_AXIS_OFFSET_MASK	0x1F

/* ADXL367_REG_INTMAPX_UPPER MASK */
#define ADXL367_INTMAPX_UPPER_MASK	0xDF

/* ADXL367_REG_ADC_CTL definitions. */
#define ADXL367_FIFO_8_12BIT_MSK		NO_OS_GENMASK(7,6)
#define ADXL367_ADC_INACT_EN			NO_OS_BIT(3)
#define ADXL367_ADC_ACT_EN			NO_OS_BIT(1)
#define ADXL367_ADC_EN				NO_OS_BIT(0)

/* ADXL367_REG_TEMP_CTL definitions. */
#define ADXL367_TEMP_INACT_EN			NO_OS_BIT(3)
#define ADXL367_TEMP_ACT_EN			NO_OS_BIT(1)
#define ADXL367_TEMP_EN				NO_OS_BIT(0)

/* ADXL367 device information */
#define ADXL367_DEVICE_AD               0xAD
#define ADXL367_DEVICE_MST              0x1D
#define ADXL367_PART_ID                 0xF7

/* ADXL367 Reset settings */
#define ADXL367_RESET_KEY               0x52

/* Channel ID for FIFO read */
#define ADXL367_FIFO_X_ID		0x00
#define ADXL367_FIFO_Y_ID		0x01
#define ADXL367_FIFO_Z_ID		0x02
#define ADXL367_FIFO_TEMP_ADC_ID	0x03

#define ADXL367_ABSOLUTE		0x00
#define ADXL367_REFERENCED 		0x01

/*
 * At +/- 2g with 14-bit resolution, scale is given in datasheet as
 * 250ug/LSB = 0.0002500 * 9.80665 = 0.0024516625 m/s^2.
 * For +/- 4g range a multiplier with value 2 is used.
 * For +/-8g range, a multiplier with value 4 is used.
 */
#define ADXL367_ACC_SCALE_FACTOR_MUL  245166ULL
#define ADXL367_ACC_SCALE_FACTOR_DIV  1000000000

/*
 * At 25C, raw value is equal to 165 LSB. Raw value varies with 54LSB/C.
 * Offset = 25 * ADXL367_TEMP_PER_C - ADXL367_TEMP_25C = 1185.
 * Temp = (RAW + OFFSET) * SCALE
 *  */
#define ADXL367_TEMP_OFFSET		1185
#define ADXL367_TEMP_25C		165
#define ADXL367_TEMP_SCALE		18518518
#define ADXL367_TEMP_SCALE_DIV		1000000000

/* Min change = 90mg. Sensitivity = 4LSB / mg */
#define ADXL367_SELF_TEST_MIN	90 * 100 / 25
/* Max change = 270mg. Sensitivity = 4LSB / mg */
#define ADXL367_SELF_TEST_MAX	270 * 100 / 25

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxl367_comm_type` is an enumeration that defines the
 * communication types available for the ADXL367 device, specifically SPI
 * and I2C. This enum is used to specify the communication protocol that
 * the device will use to interface with other components or systems.
 *
 * @param ADXL367_SPI_COMM Represents the SPI communication type for the ADXL367
 * device.
 * @param ADXL367_I2C_COMM Represents the I2C communication type for the ADXL367
 * device.
 ******************************************************************************/
enum adxl367_comm_type {
	ADXL367_SPI_COMM,
	ADXL367_I2C_COMM
};

/***************************************************************************//**
 * @brief The `adxl367_op_mode` enumeration defines the operating modes for the
 * ADXL367 accelerometer device. It includes two modes: standby and
 * measure, which are used to control the power state and functionality
 * of the device. The standby mode is used when the device is not
 * actively measuring, while the measure mode is used when the device is
 * actively collecting data.
 *
 * @param ADXL367_OP_STANDBY Represents the standby mode of the ADXL367 device,
 * with a value of 0.
 * @param ADXL367_OP_MEASURE Represents the measurement mode of the ADXL367
 * device, with a value of 2.
 ******************************************************************************/
enum adxl367_op_mode {
	ADXL367_OP_STANDBY = 0,
	ADXL367_OP_MEASURE = 2
};

/***************************************************************************//**
 * @brief The `adxl367_range` enumeration defines the possible measurement
 * ranges for the ADXL367 accelerometer, allowing the user to select
 * between +/-2g, +/-4g, and +/-8g ranges. This selection impacts the
 * sensitivity and dynamic range of the accelerometer readings, enabling
 * the device to be configured for different levels of acceleration
 * measurement precision and range.
 *
 * @param ADXL367_2G_RANGE Represents a measurement range of +/-2g.
 * @param ADXL367_4G_RANGE Represents a measurement range of +/-4g.
 * @param ADXL367_8G_RANGE Represents a measurement range of +/-8g.
 ******************************************************************************/
enum adxl367_range {
	ADXL367_2G_RANGE,
	ADXL367_4G_RANGE,
	ADXL367_8G_RANGE,
};

/***************************************************************************//**
 * @brief The `adxl367_odr` enumeration defines the possible output data rates
 * for the ADXL367 accelerometer. Each enumerator corresponds to a
 * specific frequency at which the device can output data, ranging from
 * 12.5 Hz to 400 Hz. This allows users to select the appropriate data
 * rate for their application needs, balancing between power consumption
 * and data update frequency.
 *
 * @param ADXL367_ODR_12P5HZ Represents an output data rate of 12.5 Hz.
 * @param ADXL367_ODR_25HZ Represents an output data rate of 25 Hz.
 * @param ADXL367_ODR_50HZ Represents an output data rate of 50 Hz.
 * @param ADXL367_ODR_100HZ Represents an output data rate of 100 Hz.
 * @param ADXL367_ODR_200HZ Represents an output data rate of 200 Hz.
 * @param ADXL367_ODR_400HZ Represents an output data rate of 400 Hz.
 ******************************************************************************/
enum adxl367_odr {
	ADXL367_ODR_12P5HZ,
	ADXL367_ODR_25HZ,
	ADXL367_ODR_50HZ,
	ADXL367_ODR_100HZ,
	ADXL367_ODR_200HZ,
	ADXL367_ODR_400HZ,
};

/***************************************************************************//**
 * @brief The `adxl367_fifo_mode` enumeration defines the different modes of
 * operation for the FIFO (First In, First Out) buffer in the ADXL367
 * accelerometer. It allows the user to select between disabling the
 * FIFO, saving the oldest data, streaming data continuously, or using a
 * triggered mode to store data based on specific events. This
 * flexibility in FIFO operation is crucial for managing data storage and
 * retrieval in various application scenarios.
 *
 * @param ADXL367_FIFO_DISABLED Represents the state where the FIFO is disabled.
 * @param ADXL367_OLDEST_SAVED Indicates the mode where the oldest data is saved
 * in the FIFO.
 * @param ADXL367_STREAM_MODE Represents the streaming mode where data is
 * continuously written to the FIFO.
 * @param ADXL367_TRIGGERED_MODE Indicates the triggered mode where data is
 * written to the FIFO based on a trigger event.
 ******************************************************************************/
enum adxl367_fifo_mode {
	ADXL367_FIFO_DISABLED,
	ADXL367_OLDEST_SAVED,
	ADXL367_STREAM_MODE,
	ADXL367_TRIGGERED_MODE
};

/***************************************************************************//**
 * @brief The `adxl367_fifo_format` enum defines various modes for storing data
 * in the FIFO of the ADXL367 device. Each mode specifies which
 * combination of axis data, temperature, and ADC data is stored,
 * allowing for flexible configuration based on the application's data
 * requirements. This enum is crucial for setting up the FIFO storage
 * format to match the specific needs of the data acquisition process.
 *
 * @param ADXL367_FIFO_FORMAT_XYZ Represents the default mode where all axis
 * data is stored in the FIFO.
 * @param ADXL367_FIFO_FORMAT_X Stores only the X axis data in the FIFO.
 * @param ADXL367_FIFO_FORMAT_Y Stores only the Y axis data in the FIFO.
 * @param ADXL367_FIFO_FORMAT_Z Stores only the Z axis data in the FIFO.
 * @param ADXL367_FIFO_FORMAT_XYZT Stores all axis data plus temperature in the
 * FIFO.
 * @param ADXL367_FIFO_FORMAT_XT Stores X axis data plus temperature in the
 * FIFO.
 * @param ADXL367_FIFO_FORMAT_YT Stores Y axis data plus temperature in the
 * FIFO.
 * @param ADXL367_FIFO_FORMAT_ZT Stores Z axis data plus temperature in the
 * FIFO.
 * @param ADXL367_FIFO_FORMAT_XYZA Stores all axis data plus ADC in the FIFO.
 * @param ADXL367_FIFO_FORMAT_XA Stores X axis data plus ADC in the FIFO.
 * @param ADXL367_FIFO_FORMAT_YA Stores Y axis data plus ADC in the FIFO.
 * @param ADXL367_FIFO_FORMAT_ZA Stores Z axis data plus ADC in the FIFO.
 ******************************************************************************/
enum adxl367_fifo_format {
	/* All axis. Default mode. */
	ADXL367_FIFO_FORMAT_XYZ,
	/* X axis. */
	ADXL367_FIFO_FORMAT_X,
	/* Y axis. */
	ADXL367_FIFO_FORMAT_Y,
	/* Z axis. */
	ADXL367_FIFO_FORMAT_Z,
	/* All axis + temperature. */
	ADXL367_FIFO_FORMAT_XYZT,
	/* X axis + temperature. */
	ADXL367_FIFO_FORMAT_XT,
	/* Y axis + temperature. */
	ADXL367_FIFO_FORMAT_YT,
	/* Z axis + temperature. */
	ADXL367_FIFO_FORMAT_ZT,
	/* All axis + ADC. */
	ADXL367_FIFO_FORMAT_XYZA,
	/* X axis + ADC. */
	ADXL367_FIFO_FORMAT_XA,
	/* Y axis + ADC. */
	ADXL367_FIFO_FORMAT_YA,
	/* Z axis + ADC. */
	ADXL367_FIFO_FORMAT_ZA
};

/***************************************************************************//**
 * @brief The `adxl367_fifo_read_mode` enumeration defines the different modes
 * available for reading data from the FIFO of the ADXL367 device. Each
 * mode specifies the bit resolution and whether a channel ID is included
 * in the data read from the FIFO. This allows for flexibility in how
 * data is retrieved and processed, depending on the application's
 * requirements for precision and data identification.
 *
 * @param ADXL367_12B_CHID Represents a FIFO read mode with upper 12 bits plus
 * channel ID.
 * @param ADXL367_8B Represents a FIFO read mode with upper 8 bits, no channel
 * ID.
 * @param ADXL367_12B Represents a FIFO read mode with upper 12 bits, no channel
 * ID.
 * @param ADXL367_14B_CHID Represents a FIFO read mode with 14 bits plus channel
 * ID, and is the default mode.
 ******************************************************************************/
enum adxl367_fifo_read_mode {
	/* Upper 12 bits plus channel ID.*/
	ADXL367_12B_CHID,
	/* Upper 8 bits, no channel ID. */
	ADXL367_8B,
	/* Upper 12 bits, no channel ID. */
	ADXL367_12B,
	/* 14 bits plus channel ID. Default mode. */
	ADXL367_14B_CHID
};

/***************************************************************************//**
 * @brief The `adxl367_int_map` structure is a bit-field representation of
 * various interrupt and status flags for the ADXL367 accelerometer. Each
 * member of the structure is a single bit that represents a specific
 * condition or event, such as errors, activity/inactivity status, tap
 * events, and FIFO conditions. This structure is used to map and manage
 * the interrupt signals generated by the device, allowing for efficient
 * monitoring and handling of the accelerometer's operational states.
 *
 * @param err_fuse Indicates an error in the fuse settings.
 * @param err_user_regs Indicates an error in the user registers.
 * @param kpalv_timer Represents the keep-alive timer status.
 * @param temp_adc_hi Indicates if the temperature ADC is high.
 * @param temp_adc_low Indicates if the temperature ADC is low.
 * @param tap_two Indicates a double tap event.
 * @param tap_one Indicates a single tap event.
 * @param int_low Indicates a low interrupt signal.
 * @param awake Indicates if the device is awake.
 * @param inact Indicates inactivity status.
 * @param act Indicates activity status.
 * @param fifo_overrun Indicates a FIFO overrun condition.
 * @param fifo_watermark Indicates if the FIFO watermark level is reached.
 * @param fifo_ready Indicates if the FIFO is ready for reading.
 * @param data_ready Indicates if new data is ready to be read.
 ******************************************************************************/
struct adxl367_int_map {
	uint8_t err_fuse 	: 1;
	uint8_t err_user_regs 	: 1;
	uint8_t kpalv_timer	: 1;
	uint8_t temp_adc_hi 	: 1;
	uint8_t temp_adc_low	: 1;
	uint8_t tap_two		: 1;
	uint8_t tap_one		: 1;
	uint8_t int_low 	: 1;
	uint8_t awake 		: 1;
	uint8_t inact		: 1;
	uint8_t act		: 1;
	uint8_t fifo_overrun 	: 1;
	uint8_t fifo_watermark	: 1;
	uint8_t fifo_ready	: 1;
	uint8_t data_ready	: 1;
};

/***************************************************************************//**
 * @brief The `adxl367_fractional_val` structure is used to represent a value
 * with both integer and fractional components, allowing for precise
 * representation of measurements such as acceleration or temperature in
 * the ADXL367 driver. The integer part is stored in a 64-bit integer,
 * while the fractional part is stored in a 32-bit integer, providing a
 * way to handle values that require more precision than a simple integer
 * can provide.
 *
 * @param integer Stores the integer part of the fractional value.
 * @param fractional Stores the fractional part of the value as a 32-bit
 * integer.
 ******************************************************************************/
struct adxl367_fractional_val {
	int64_t integer;
	int32_t fractional;
};

/***************************************************************************//**
 * @brief The `adxl367_dev` structure represents a device configuration for the
 * ADXL367 accelerometer, encapsulating communication settings,
 * measurement parameters, and FIFO configurations. It includes
 * descriptors for both SPI and I2C communication, allowing flexibility
 * in interfacing with the device. The structure also holds configuration
 * settings for measurement range, operating mode, output data rate, and
 * FIFO operation, as well as user-defined offsets for each axis to
 * calibrate the accelerometer readings. The FIFO buffer is used to store
 * data from the device, supporting various read and storage formats.
 *
 * @param comm_type Specifies the communication type, either I2C or SPI.
 * @param spi_desc Pointer to the SPI descriptor for SPI communication.
 * @param i2c_desc Pointer to the I2C descriptor for I2C communication.
 * @param i2c_slave_address I2C slave address, which can be 0x53 or 0x1D
 * depending on the ASEL pin.
 * @param range Specifies the measurement range of the accelerometer.
 * @param op_mode Defines the operating mode of the device.
 * @param odr Specifies the output data rate of the device.
 * @param fifo_mode Defines the FIFO operating mode.
 * @param fifo_format Specifies the FIFO storage format.
 * @param fifo_read_mode Defines the FIFO reading mode.
 * @param fifo_buffer Buffer for storing FIFO data, with a size of 1027 bytes.
 * @param x_offset User-defined offset for the X-axis.
 * @param y_offset User-defined offset for the Y-axis.
 * @param z_offset User-defined offset for the Z-axis.
 ******************************************************************************/
struct adxl367_dev {
	/** Communication type - I2C or SPI. */
	enum adxl367_comm_type		comm_type;
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** I2C Descriptor */
	struct no_os_i2c_desc  		*i2c_desc;
	/** Depending on ASEL pin, can be 0x53 or 0x1D. Only for I2C Comm. */
	uint8_t i2c_slave_address;
	/** Measurement Range: */
	enum adxl367_range		range;
	enum adxl367_op_mode 		op_mode;
	enum adxl367_odr 		odr;
	enum adxl367_fifo_mode		fifo_mode;
	enum adxl367_fifo_format 	fifo_format;
	enum adxl367_fifo_read_mode 	fifo_read_mode;
	/** FIFO Buffer 513 * 2 + 1 cmd byte */
	uint8_t 			fifo_buffer[1027];
	uint16_t 			x_offset;
	uint16_t 			y_offset;
	uint16_t 			z_offset;
};

/***************************************************************************//**
 * @brief The `adxl367_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the ADXL367 device.
 * It includes fields for specifying the communication type (I2C or SPI),
 * as well as the necessary initialization parameters for each
 * communication protocol. Additionally, it contains the I2C slave
 * address, which is relevant only when I2C communication is used. This
 * structure is essential for configuring the device before it can be
 * used for data acquisition.
 *
 * @param comm_type Specifies the communication type, either I2C or SPI.
 * @param spi_init Holds the SPI initialization parameters.
 * @param i2c_init Holds the I2C initialization parameters.
 * @param i2c_slave_address Specifies the I2C slave address, which can be 0x53
 * or 0x1D depending on the ASEL pin.
 ******************************************************************************/
struct adxl367_init_param {
	/** Communication type - I2C or SPI. */
	enum adxl367_comm_type 		comm_type;
	/** SPI Initialization structure. */
	struct no_os_spi_init_param	spi_init;
	/** I2C Initialization structure. */
	struct no_os_i2c_init_param    	i2c_init;
	/** Depending on ASEL pin, can be 0x53 or 0x1D. Only for I2C Comm. */
	uint8_t 			i2c_slave_address;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initializes the device. */
/***************************************************************************//**
 * @brief This function sets up the ADXL367 accelerometer device for operation
 * by initializing communication interfaces and verifying device
 * identity. It should be called before any other operations on the
 * device. The function requires a valid communication type (SPI or I2C)
 * and corresponding initialization parameters. It performs a soft reset
 * and checks the device ID to ensure proper initialization. If
 * initialization fails, resources are cleaned up and an error is
 * returned.
 *
 * @param device A pointer to a pointer of type `struct adxl367_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adxl367_init_param` containing
 * initialization parameters such as communication type and
 * specific settings for SPI or I2C. The communication type
 * must be either `ADXL367_SPI_COMM` or `ADXL367_I2C_COMM`.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during initialization.
 ******************************************************************************/
int adxl367_init(struct adxl367_dev **device,
		 struct adxl367_init_param init_param);

/* Free the resources allocated by adxl367_init(). */
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * allocated for an ADXL367 device when it is no longer needed. It
 * handles the cleanup of communication resources based on the
 * communication type (SPI or I2C) used by the device. This function must
 * be called after the device has been initialized and used, to ensure
 * that all associated resources are freed and no memory leaks occur. The
 * function returns an integer status code indicating the success or
 * failure of the resource release process.
 *
 * @param dev A pointer to an adxl367_dev structure representing the device to
 * be removed. Must not be null. The function will handle the cleanup
 * of resources associated with this device.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the removal of resources fails.
 ******************************************************************************/
int adxl367_remove(struct adxl367_dev *dev);

/* Performs device self-test. */
/***************************************************************************//**
 * @brief This function is used to perform a self-test on the ADXL367
 * accelerometer device to ensure it is functioning correctly. It should
 * be called when the device is in a known good state, typically after
 * initialization and before any critical measurements are taken. The
 * function temporarily changes the device's power mode and self-test
 * settings, and it requires the device to be configured with a valid
 * output data rate (ODR). If the ODR is not set to a supported value,
 * the function will return an error. The self-test involves measuring
 * the device's response to a known stimulus and comparing it against
 * expected limits. If the response is within the specified range, the
 * test passes; otherwise, it fails. The function returns an integer
 * indicating the result of the self-test.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The device should be properly configured
 * with a valid ODR before calling this function. If the ODR is
 * invalid, the function returns an error.
 * @return Returns 0 if the self-test passes, -1 if it fails, or a negative
 * error code if an invalid parameter is provided or an error occurs
 * during execution.
 ******************************************************************************/
int adxl367_self_test(struct adxl367_dev *dev);

/* Writes data into a register. */
/***************************************************************************//**
 * @brief This function is used to write a specific value to a designated
 * register on the ADXL367 device, which can be communicated with via
 * either SPI or I2C. It is essential to ensure that the device has been
 * properly initialized and that the communication type is correctly set
 * in the device structure before calling this function. The function
 * handles both SPI and I2C communication protocols, selecting the
 * appropriate method based on the device's configuration. It returns an
 * integer status code indicating the success or failure of the write
 * operation.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null, and the communication type must be
 * correctly set.
 * @param register_value The value to be written to the specified register. It
 * is an 8-bit unsigned integer.
 * @param register_address The address of the register to which the value will
 * be written. It is an 8-bit unsigned integer.
 * @return Returns an integer status code, where 0 typically indicates success
 * and a negative value indicates an error.
 ******************************************************************************/
int adxl367_set_register_value(struct adxl367_dev *dev,
			       uint8_t register_value,
			       uint8_t register_address);

/* Performs a burst read of a specified number of registers. */
/***************************************************************************//**
 * @brief This function is used to read a sequence of registers from the ADXL367
 * device, either via SPI or I2C communication, depending on the device
 * configuration. It requires a valid device structure and a buffer to
 * store the read data. The function checks if the register address is
 * within a valid range and returns an error if it is not. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function will return an error code
 * if the communication fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param read_data A pointer to a buffer where the read register data will be
 * stored. Must not be null and should be large enough to hold
 * 'register_nb' bytes.
 * @param register_address The starting address of the register to read. Must be
 * less than or equal to ADXL367_REG_STATUS_2.
 * @param register_nb The number of registers to read. Must be a positive
 * integer.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl367_get_register_value(struct adxl367_dev *dev,
			       uint8_t *read_data,
			       uint8_t  register_address,
			       uint8_t  bytes_number);

/* Performs a masked write to a register. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * ADXL367 device by applying a mask. It first reads the current value of
 * the register, applies the mask to clear the bits that need to be
 * changed, and then writes the new data to those bits. This function is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. It should be called when the device is
 * properly initialized and communication is established. The function
 * returns an error code if the read or write operation fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the ADXL367.
 * @param data The data to be written to the register, after applying the mask.
 * Only the bits corresponding to the mask will be affected.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be updated with the
 * corresponding bits from the data parameter.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_reg_write_msk(struct adxl367_dev *dev,
			  uint8_t reg_addr,
			  uint8_t data,
			  uint8_t mask);

/* Resets the device via comm. */
/***************************************************************************//**
 * @brief This function resets the ADXL367 device to its default settings,
 * including standby mode, measurement range, output data rate, and FIFO
 * configuration. It should be used when a full reset of the device is
 * required to ensure it is in a known state. The function must be called
 * with a valid device structure that has been initialized. After the
 * reset, a delay is introduced to allow the device to stabilize. This
 * function returns an error code if the reset command fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The function will modify the device's
 * internal state to reflect the reset defaults.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int adxl367_software_reset(struct adxl367_dev *dev);

/* Places the device into standby/measure mode. */
/***************************************************************************//**
 * @brief This function configures the ADXL367 device to operate in a specified
 * power mode, either standby or measurement. It should be called when
 * there is a need to change the device's power state, such as before
 * starting or stopping data acquisition. If the measurement mode is
 * selected, the function introduces a delay to ensure the device is
 * ready to provide accurate data. The function must be called with a
 * valid device structure and a valid operating mode. It returns an error
 * code if the operation fails, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param mode An enum value of type adxl367_op_mode indicating the desired
 * power mode. Valid values are ADXL367_OP_STANDBY and
 * ADXL367_OP_MEASURE. Invalid values may result in undefined
 * behavior.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_set_power_mode(struct adxl367_dev *dev,
			   enum adxl367_op_mode mode);

/* Selects the measurement range. */
/***************************************************************************//**
 * @brief This function configures the measurement range of the ADXL367
 * accelerometer to one of the predefined ranges. It should be called
 * when the user needs to change the sensitivity of the device to
 * different g-force levels. The function must be called with a valid
 * device structure and a valid range enumeration value. It updates the
 * device's internal state to reflect the new range setting. If the
 * operation fails, an error code is returned.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param range An enumeration value of type adxl367_range specifying the
 * desired measurement range. Valid values are ADXL367_2G_RANGE,
 * ADXL367_4G_RANGE, and ADXL367_8G_RANGE.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_set_range(struct adxl367_dev *dev,
		      enum adxl367_range range);

/* Selects the Output Data Rate of the device. */
/***************************************************************************//**
 * @brief This function configures the output data rate (ODR) of the ADXL367
 * accelerometer device. It should be called when you need to change the
 * rate at which the device outputs data, which can be important for
 * balancing power consumption and data update frequency. The function
 * must be called with a valid device structure that has been properly
 * initialized. It updates both the device's internal register and the
 * device structure to reflect the new ODR. If the operation fails, an
 * error code is returned.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param odr An enum value of type adxl367_odr representing the desired output
 * data rate. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_set_output_rate(struct adxl367_dev *dev,
			    enum adxl367_odr odr);

/* Sets user defined offset for each axis. */
/***************************************************************************//**
 * @brief This function allows the user to set custom offset values for the X,
 * Y, and Z axes of the ADXL367 accelerometer device. It should be called
 * when there is a need to calibrate the device by adjusting the zero-g
 * offset for each axis. The function requires a valid device structure
 * and offset values for each axis. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function will return an error code if the operation fails, which can
 * occur if the device is not properly configured or if communication
 * with the device fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param x_offset A 16-bit unsigned integer representing the offset for the X
 * axis. The value is masked with ADXL367_XYZ_AXIS_OFFSET_MASK
 * before being applied.
 * @param y_offset A 16-bit unsigned integer representing the offset for the Y
 * axis. The value is masked with ADXL367_XYZ_AXIS_OFFSET_MASK
 * before being applied.
 * @param z_offset A 16-bit unsigned integer representing the offset for the Z
 * axis. The value is masked with ADXL367_XYZ_AXIS_OFFSET_MASK
 * before being applied.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_set_offset(struct adxl367_dev *dev, uint16_t x_offset,
		       uint16_t y_offset, uint16_t z_offset);

/* Reads the 3-axis raw data from the accelerometer. */
/***************************************************************************//**
 * @brief This function retrieves the raw acceleration data for the X, Y, and Z
 * axes from the ADXL367 accelerometer. It should be called when the
 * device is ready to provide new data, typically after initialization
 * and configuration. The function waits for the data to be ready before
 * reading, ensuring that the most recent measurements are obtained. It
 * is important to ensure that the device is in a proper operational mode
 * and that the provided pointers for the axis data are valid and non-
 * null.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to an int16_t where the raw X-axis data will be stored.
 * Must not be null.
 * @param y A pointer to an int16_t where the raw Y-axis data will be stored.
 * Must not be null.
 * @param z A pointer to an int16_t where the raw Z-axis data will be stored.
 * Must not be null.
 * @return Returns 0 on success, or -1 if an error occurs while reading the
 * data.
 ******************************************************************************/
int adxl367_get_raw_xyz(struct adxl367_dev *dev,
			int16_t* x,
			int16_t* y,
			int16_t* z);

/* Reads the 3-axis raw data from the accelerometer and converts it to g. */
/***************************************************************************//**
 * @brief Use this function to obtain the current acceleration values in g-units
 * for the x, y, and z axes from the ADXL367 device. This function should
 * be called when you need precise acceleration measurements in a
 * fractional format. Ensure that the device is properly initialized and
 * configured before calling this function. The function will return an
 * error code if it fails to read or convert the data.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to an adxl367_fractional_val structure where the x-axis
 * acceleration value will be stored. Must not be null.
 * @param y A pointer to an adxl367_fractional_val structure where the y-axis
 * acceleration value will be stored. Must not be null.
 * @param z A pointer to an adxl367_fractional_val structure where the z-axis
 * acceleration value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl367_get_g_xyz(struct adxl367_dev *dev,
		      struct adxl367_fractional_val* x,
		      struct adxl367_fractional_val* y,
		      struct adxl367_fractional_val* z);

/* Enables temperature reading. */
/***************************************************************************//**
 * @brief Use this function to control the temperature reading feature of the
 * ADXL367 device. It should be called when you need to enable or disable
 * the temperature sensor, typically as part of device configuration or
 * power management. Ensure that the device is properly initialized
 * before calling this function. The function modifies the device's
 * internal register to reflect the desired state of the temperature
 * reading feature.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param enable A boolean value where true enables temperature reading and
 * false disables it.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_temp_read_en(struct adxl367_dev *dev, bool enable);

/* Enables adc reading and disables temperature reading. */
/***************************************************************************//**
 * @brief Use this function to control the ADC reading capability of the ADXL367
 * device. It disables temperature reading and enables or disables ADC
 * reading based on the provided parameter. This function should be
 * called when you need to switch between temperature and ADC data
 * acquisition modes. Ensure that the device is properly initialized
 * before calling this function. The function returns an error code if
 * the operation fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) ADC reading.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_adc_read_en(struct adxl367_dev *dev, bool enable);

/* Reads the raw temperature of the device. */
/***************************************************************************//**
 * @brief This function retrieves the raw temperature data from the ADXL367
 * sensor. It should be called when the raw temperature reading is needed
 * for further processing or conversion. The function waits until the
 * data is ready before reading it, ensuring that the temperature value
 * is current. It is important to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * returns an error code if the reading process fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param raw_temp A pointer to an int16_t variable where the raw temperature
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl367_read_raw_temp(struct adxl367_dev *dev, int16_t *raw_temp);

/* Reads the temperature of the device. */
/***************************************************************************//**
 * @brief This function retrieves the current temperature reading from the
 * ADXL367 device and converts it into a fractional value format. It
 * should be called when a temperature measurement is needed from the
 * device. The function requires a valid device structure and a pointer
 * to a fractional value structure where the temperature will be stored.
 * It is important to ensure that the device has been properly
 * initialized before calling this function. The function will return an
 * error code if the temperature reading or conversion fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param temp A pointer to an adxl367_fractional_val structure where the
 * converted temperature value will be stored. Must not be null.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_read_temperature(struct adxl367_dev *dev,
			     struct adxl367_fractional_val *temp);

/* Reads ADC data. */
/***************************************************************************//**
 * @brief Use this function to read the ADC data from an ADXL367 device. It
 * should be called when ADC data is needed from the device, and it
 * requires the device to be properly initialized and configured. The
 * function waits for the ADC data to be ready before reading it,
 * ensuring that the data returned is valid. It is important to handle
 * the return value to check for any errors during the read operation.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param data A pointer to an int16_t variable where the ADC data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl367_read_adc(struct adxl367_dev *dev, int16_t *data);

/* Reads the number of FIFO entries register value. */
/***************************************************************************//**
 * @brief Use this function to obtain the current number of entries in the FIFO
 * buffer of an ADXL367 device. It is essential to ensure that the device
 * has been properly initialized before calling this function. The
 * function reads the FIFO entry count from the device and stores it in
 * the provided memory location. This function is useful for applications
 * that need to monitor or manage the FIFO buffer's state. It returns an
 * error code if the operation fails, which should be checked to ensure
 * successful execution.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param entr_nb A pointer to a uint16_t variable where the number of FIFO
 * entries will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl367_get_nb_of_fifo_entries(struct adxl367_dev *dev,
				   uint16_t *entr_nb);

/* Sets the number of FIFO samples register value. */
/***************************************************************************//**
 * @brief This function configures the ADXL367 device to store a specified
 * number of sample sets in its FIFO buffer. It should be called when the
 * device is in a state where FIFO configuration is allowed, typically
 * after initialization and before starting measurements. The function
 * takes a 16-bit integer representing the number of sample sets to be
 * stored. It handles the configuration by writing to the appropriate
 * registers of the device. If the operation fails, an error code is
 * returned.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param sets_nb A 16-bit unsigned integer specifying the number of sample sets
 * to store in the FIFO. The value is split across two registers,
 * with the 9th bit affecting the FIFO control register.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adxl367_set_fifo_sample_sets_nb(struct adxl367_dev *dev,
				    uint16_t sets_nb);

/* Sets FIFO mode. */
/***************************************************************************//**
 * @brief This function configures the FIFO mode of the ADXL367 accelerometer
 * device. It should be called when you need to change the FIFO operating
 * mode, such as disabling it or setting it to stream or triggered mode.
 * The function requires a valid device structure and a mode from the
 * predefined FIFO modes. It updates the device's internal state to
 * reflect the new mode. Ensure the device is properly initialized before
 * calling this function. If the operation fails, an error code is
 * returned.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param mode An enum value of type adxl367_fifo_mode specifying the desired
 * FIFO mode. Valid values are ADXL367_FIFO_DISABLED,
 * ADXL367_OLDEST_SAVED, ADXL367_STREAM_MODE, and
 * ADXL367_TRIGGERED_MODE.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_set_fifo_mode(struct adxl367_dev *dev,
			  enum adxl367_fifo_mode mode);

/* Sets FIFO read mode. */
/***************************************************************************//**
 * @brief This function configures the FIFO read mode of the ADXL367 device,
 * which determines how data is read from the FIFO buffer. It should be
 * called after the device has been initialized and before reading data
 * from the FIFO. The function updates the device's internal state to
 * reflect the selected read mode. It is important to ensure that the
 * device pointer is valid and that the read mode is a valid enumeration
 * value. If the operation fails, an error code is returned.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param read_mode An enumeration value of type adxl367_fifo_read_mode
 * specifying the desired FIFO read mode. Must be a valid enum
 * value; otherwise, the behavior is undefined.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_set_fifo_read_mode(struct adxl367_dev *dev,
			       enum adxl367_fifo_read_mode read_mode);

/* Sets FIFO data storage format. */
/***************************************************************************//**
 * @brief This function configures the FIFO data storage format for the ADXL367
 * accelerometer device. It should be called when you need to change how
 * data is stored in the FIFO, such as selecting which axes or additional
 * data (like temperature or ADC) are included. The function must be
 * called with a valid device structure and a valid format enumeration
 * value. It updates the device's internal state to reflect the new
 * format and returns an error code if the operation fails or if an
 * invalid format is provided.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param format An enumeration value of type adxl367_fifo_format specifying the
 * desired FIFO data storage format. Must be a valid format;
 * otherwise, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if an invalid format is provided.
 ******************************************************************************/
int adxl367_set_fifo_format(struct adxl367_dev *dev,
			    enum adxl367_fifo_format format);

/* Configures FIFO feature. */
/***************************************************************************//**
 * @brief This function sets up the FIFO configuration for the ADXL367 device,
 * including the mode, format, and number of sample sets. It should be
 * called after the device has been initialized and before starting data
 * acquisition if FIFO functionality is required. The function configures
 * the FIFO mode, format, and the number of sample sets to be stored in
 * the FIFO. It returns an error code if any of the configuration steps
 * fail, allowing the caller to handle configuration errors
 * appropriately.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param mode An enum value of type adxl367_fifo_mode specifying the desired
 * FIFO mode. Valid values are ADXL367_FIFO_DISABLED,
 * ADXL367_OLDEST_SAVED, ADXL367_STREAM_MODE, and
 * ADXL367_TRIGGERED_MODE.
 * @param format An enum value of type adxl367_fifo_format specifying the
 * desired FIFO data storage format. Valid values include
 * ADXL367_FIFO_FORMAT_XYZ, ADXL367_FIFO_FORMAT_X,
 * ADXL367_FIFO_FORMAT_Y, ADXL367_FIFO_FORMAT_Z, and others as
 * defined in the enum.
 * @param sets_nb A uint8_t value specifying the number of sample sets to store
 * in the FIFO. The valid range is device-specific and should be
 * within the limits supported by the ADXL367.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * which configuration step failed.
 ******************************************************************************/
int adxl367_fifo_setup(struct adxl367_dev *dev,
		       enum adxl367_fifo_mode mode,
		       enum adxl367_fifo_format format,
		       uint8_t sets_nb);

/* Reads raw values from FIFO. */
/***************************************************************************//**
 * @brief This function retrieves raw data from the FIFO buffer of an ADXL367
 * device, including acceleration data for the X, Y, and Z axes, as well
 * as temperature ADC values. It should be called when the FIFO buffer is
 * expected to contain data, and the device has been properly initialized
 * and configured to use the FIFO. The function updates the provided
 * pointers with the retrieved data and the number of entries read. It
 * returns an error code if any of the pointers for the requested data
 * types are null or if an invalid channel ID is encountered.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to an array of int16_t where X-axis data will be stored.
 * Can be null if X-axis data is not needed.
 * @param y A pointer to an array of int16_t where Y-axis data will be stored.
 * Can be null if Y-axis data is not needed.
 * @param z A pointer to an array of int16_t where Z-axis data will be stored.
 * Can be null if Z-axis data is not needed.
 * @param temp_adc A pointer to an array of int16_t where temperature ADC data
 * will be stored. Can be null if temperature data is not
 * needed.
 * @param entries A pointer to a uint16_t where the number of entries read from
 * the FIFO will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs,
 * such as null pointers for required data or invalid channel IDs.
 ******************************************************************************/
int adxl367_read_raw_fifo(struct adxl367_dev *dev, int16_t *x, int16_t *y,
			  int16_t *z, int16_t *temp_adc, uint16_t *entries);

/* Reads converted values from FIFO. */
/***************************************************************************//**
 * @brief This function retrieves data from the FIFO of an ADXL367 device,
 * converts the raw accelerometer and temperature data into a fractional
 * format, and stores the results in the provided output structures. It
 * should be called when you need to process the latest FIFO data from
 * the device. The function requires a valid device structure and pre-
 * allocated output structures for the x, y, z axes, and temperature
 * data. The number of entries read is returned through the entries
 * parameter. Ensure the device is properly initialized and configured to
 * use the FIFO before calling this function.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param x A pointer to an adxl367_fractional_val structure where the converted
 * x-axis data will be stored. Must not be null.
 * @param y A pointer to an adxl367_fractional_val structure where the converted
 * y-axis data will be stored. Must not be null.
 * @param z A pointer to an adxl367_fractional_val structure where the converted
 * z-axis data will be stored. Must not be null.
 * @param temp_adc A pointer to an adxl367_fractional_val structure where the
 * converted temperature data will be stored. Must not be null.
 * @param entries A pointer to a uint16_t where the number of FIFO entries read
 * will be stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_read_converted_fifo(struct adxl367_dev *dev,
				struct adxl367_fractional_val *x, struct adxl367_fractional_val *y,
				struct adxl367_fractional_val *z, struct adxl367_fractional_val *temp_adc,
				uint16_t *entries);

/* Enables specified events to interrupt pin. */
int adxl367_int_map(struct adxl367_dev *dev, struct adxl367_int_map *map,
		    uint8_t pin);

/* Configures activity detection. */
/***************************************************************************//**
 * @brief This function sets up activity detection on the ADXL367 accelerometer
 * by configuring the activity threshold and time parameters. It must be
 * called after the device has been initialized and is typically used to
 * enable the device to detect motion or changes in acceleration. The
 * function allows the user to specify whether the activity detection
 * should be based on absolute or referenced values. Proper configuration
 * of the threshold and time parameters is crucial for accurate activity
 * detection. The function returns an error code if the configuration
 * fails.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param ref_or_abs A uint8_t value indicating the type of activity detection:
 * ADXL367_ABSOLUTE for absolute detection or
 * ADXL367_REFERENCED for referenced detection. Invalid values
 * result in an error.
 * @param threshold A uint16_t value representing the activity threshold. The
 * value is split across two registers, and must be within the
 * valid range for the device.
 * @param time A uint8_t value specifying the time parameter for activity
 * detection. Must be within the valid range for the device.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_setup_activity_detection(struct adxl367_dev *dev,
				     uint8_t  ref_or_abs,
				     uint16_t threshold,
				     uint8_t  time);

/* Configures inactivity detection. */
/***************************************************************************//**
 * @brief This function sets up the inactivity detection feature on the ADXL367
 * accelerometer device. It should be called when you want the device to
 * monitor for periods of inactivity based on a specified threshold and
 * time duration. The function requires the device to be initialized and
 * configured properly before calling. It allows you to choose between
 * absolute or referenced inactivity detection modes. The function will
 * return an error code if the configuration fails, which can occur if
 * invalid parameters are provided.
 *
 * @param dev A pointer to an initialized adxl367_dev structure representing the
 * device. Must not be null.
 * @param ref_or_abs A uint8_t value indicating the type of inactivity
 * detection: ADXL367_ABSOLUTE for absolute mode or
 * ADXL367_REFERENCED for referenced mode. Invalid values will
 * result in an error.
 * @param threshold A uint16_t value representing the inactivity threshold. The
 * valid range depends on the device's configuration and should
 * be set according to the desired sensitivity.
 * @param time A uint16_t value specifying the duration of inactivity in a
 * device-specific time unit. Must be a valid time value for the
 * device.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl367_setup_inactivity_detection(struct adxl367_dev *dev,
				       uint8_t  ref_or_abs,
				       uint16_t threshold,
				       uint16_t  time);

#endif /* __ADXL367_H__ */
