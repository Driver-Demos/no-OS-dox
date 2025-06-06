/***************************************************************************//**
 *   @file   adxl313.h
 *   @brief  Header file of ADXL313 Driver.
 *   @author GMois (george.mois@analog.com)
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
#ifndef __ADXL313_H__
#define __ADXL313_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* SPI commands */
#define ADXL313_SPI_READ					(0x80)
#define ADXL313_SPI_WRITE					(0x00)
#define ADXL313_MULTIBIT					NO_OS_BIT(6)

/* ADXL313 register map */
#define ADXL313_REG_DEVID_AD					0x00
#define ADXL313_REG_DEVID1_AD					0x01
#define ADXL313_REG_PARTID					0x02
#define ADXL313_REG_REVID					0x03
#define ADXL313_REG_XID						0x04
#define ADXL313_REG_SOFT_RESET					0x18
#define ADXL313_REG_OFS_AXIS(index)				(0x1E + (index))
#define ADXL313_REG_THRESH_ACT					0x24
#define ADXL313_REG_THRESH_INACT				0x25
#define ADXL313_REG_TIME_INACT					0x26
#define ADXL313_REG_ACT_INACT_CTL				0x27
#define ADXL313_REG_BW_RATE					0x2C
#define ADXL313_REG_POWER_CTL					0x2D
#define ADXL313_REG_INT_EN					0x2E
#define ADXL313_REG_INT_MAP					0x2F
#define ADXL313_REG_INT_SRC					0x30
#define ADXL313_REG_DATA_FORMAT					0x31
#define ADXL313_REG_DATA_AXIS(index)				(0x32 + ((index) * 2))
#define ADXL313_REG_FIFO_CTL					0x38
#define ADXL313_REG_FIFO_STATUS					0x39

/* ADXL313_REG_DEVID_AD value */
#define ADXL313_DEVID						0xAD
/* ADXL313_REG_DEVID1_AD value */
#define ADXL313_DEVID1						0x1D
/* ADXL313_REG_PARTID value, 0xCB, 313 in octal */
#define ADXL313_PARTID						0xCB
/* ADXL314_DEVID_AD value, same as ADXL312 */
#define ADXL314_DEVID						0xE5

/* ADXL313 reset key */
#define ADXL313_RESET_KEY					0x52

/* Maximum number of FIFO samples */
#define ADXL313_MAX_FIFO_SAMPLES_VAL  				0x60

/* Device bandwidth settings, ADXL313_REG_BW_RATE definitions */
#define ADXL313_RATE_MSK					NO_OS_GENMASK(3, 0)
#define ADXL313_LOW_POWER_OP					NO_OS_BIT(4)

/* ADXL313_REG_POWER_CTL definitions */
#define ADXL313_POWER_CTL_I2C_DISABLE				NO_OS_BIT(6)
#define ADXL313_POWER_CTL_LINK					NO_OS_BIT(5)
#define ADXL313_POWER_CTL_AUTO_SLEEP				NO_OS_BIT(4)
#define ADXL313_POWER_CTL_MEASURE				NO_OS_BIT(3)
#define ADXL313_POWER_CTL_SLEEP					NO_OS_BIT(2)
#define ADXL313_POWER_CTL_WAKEUP(x)				((x) & 0x3)

/* ADXL313_REG_INT_EN definitions */
#define ADXL313_REG_INT_EN_DATA_READY				NO_OS_BIT(7)
#define ADXL313_REG_INT_EN_ACTIVITY				NO_OS_BIT(4)
#define ADXL313_REG_INT_EN_INACTIVITY				NO_OS_BIT(3)
#define ADXL313_REG_INT_EN_WATERMARK				NO_OS_BIT(1)
#define ADXL313_REG_INT_EN_OVERRUN				NO_OS_BIT(0)

/* ADXL313_REG_INT_MAP definitions */
#define ADXL313_REG_INT_MAP_DR_INT2				NO_OS_BIT(7)
#define ADXL313_REG_INT_MAP_ACT_INT2				NO_OS_BIT(4)
#define ADXL313_REG_INT_MAP_INACT_INT2				NO_OS_BIT(3)
#define ADXL313_REG_INT_MAP_WM_INT2				NO_OS_BIT(1)
#define ADXL313_REG_INT_MAP_OVER_INT2				NO_OS_BIT(0)

/* ADXL313_REG_INT_SRC definitions */
#define ADXL313_REG_INT_SRC_DATA_READY				NO_OS_BIT(7)
#define ADXL313_REG_INT_SRC_ACTIVITY				NO_OS_BIT(4)
#define ADXL313_REG_INT_SRC_INACTIVITY				NO_OS_BIT(3)
#define ADXL313_REG_INT_SRC_WATERMARK				NO_OS_BIT(1)
#define ADXL313_REG_INT_SRC_OVERRUN				NO_OS_BIT(0)

/* ADXL313_REG_DATA_FORMAT definitions */
#define ADXL313_REG_DATA_FORMAT_SELF_TEST			NO_OS_BIT(7)
#define ADXL313_REG_DATA_FORMAT_SPI_3WIRE			NO_OS_BIT(6)
#define ADXL313_REG_DATA_FORMAT_INT_INV				NO_OS_BIT(5)
#define ADXL313_REG_DATA_FORMAT_FULL_RES			NO_OS_BIT(3)
#define ADXL313_REG_DATA_FORMAT_JUSTIFY_BIT			NO_OS_BIT(2)
#define ADXL313_REG_DATA_FORMAT_RANGE				NO_OS_GENMASK(1, 0)

/* ADXL313_REG_FIFO_CTL definitions */
#define ADXL313_REG_FIFO_CTL_MODE_MSK				NO_OS_GENMASK(7, 6)
#define ADXL313_REG_FIFO_CTL_TRIGGER_BIT			NO_OS_BIT(5)
#define ADXL313_REG_FIFO_CTL_SAMPLES_MSK			NO_OS_GENMASK(4, 0)

/* ADXL313_REG_FIFO_STATUS definitions */
#define ADXL313_REG_FIFO_FIFO_TRIGGER_BIT			NO_OS_BIT(7)
#define ADXL313_REG_FIFO_STS_ENTRIES_MSK			NO_OS_GENMASK(5, 0)

/*
 * ADXL312
 * At all g ranges with full bit resolution, sensitivity is given in datasheet as
 * 2.9 mg/LSB = 0.0029000 * 9.80665 = 0.0284392850 m/s^2.
 */
#define ADXL312_ACC_SCALE_FACTOR_MUL_FULL_RES			((int64_t)28439285ULL)

/*
 * ADXL313
 * At all g ranges with full bit resolution, sensitivity is given in datasheet as
 * 1024 LSB/g = 0.0009765625 * 9.80665 = 0.00957680664 m/s^2.
 *
 * For +/- 2g range, a multiplier with value 2 is used.
 * For +/- 2g range, a multiplier with value 4 is used.
 * For +/- 4g range, a multiplier with value 8 is used.
 *
 */
#define ADXL313_ACC_SCALE_FACTOR_MUL_FULL_RES			((int64_t)957680664ULL)
/* Divider used for dividing of multiplier.
 * Multiplication with 100 was used for preserving precision. */
#define ADXL313_ACC_SCALE_FACTOR_MUL_DIVIDER			((int32_t)100)

/*
 * ADXL314
 * At +/-200g with 13-bit resolution, scale factor is given in datasheet as
 * 48.83 mg/LSB = 0.0488300 * 9.80665 = 0.4788587195 m/s^2.
 */
#define ADXL314_ACC_SCALE_FACTOR_MUL				((int64_t)478858719ULL)
#define ADXL313_ACC_SCALE_FACTOR_DIV				((int32_t)1000000000)

/* ADXL312
 * The scale factor of the offset adjustment, in g / 1000 000 (ug) */
#define ADXL312_OFFSET_SCALE_FACTOR				11600

/* ADXL313
 * The scale factor of the offset adjustment, in g / 1000 000 (ug) */
#define ADXL313_OFFSET_SCALE_FACTOR				3900

/* ADXL314
 * The scale factor of the offset adjustment, in g / 1000 000 (ug) */
#define ADXL314_OFFSET_SCALE_FACTOR				195000

/* ADXL312
 * The scale factor of the activity/inactivity thresholds, in g / 1000 000 (ug) */
#define ADXL312_THRESH_SCALE_FACTOR				46400

/* ADXL313
 * The scale factor of the activity/inactivity thresholds, in g / 1000 000 (ug) */
#define ADXL313_THRESH_SCALE_FACTOR				15625

/* ADXL314
 * The scale factor of the activity/inactivity thresholds, in g / 1000 000 (ug) */
#define ADXL314_THRESH_SCALE_FACTOR				784000

/* Offset added to the ODR enum for obtaining corresponding register setting */
#define ADXL313_ODR_OFFSET_VAL					6

/* Number of registers read for all axes */
#define ADXL313_REGS_PER_ENTRY					6

/* Max number of FIFO entries */
#define ADXL313_MAX_FIFO_ENTRIES				32

/* Number of Self-test samples */
#define ADXL313_SELF_TEST_SAMPLES				10

/* Self-test multiplication factor */
#define ADXL312_SELF_TEST_MULT					2900
#define ADXL313_SELF_TEST_MULT					976
#define ADXL314_SELF_TEST_MULT					48830
#define ADXL313_SELF_TEST_DIV					1000

/* Self-test minimum deviation for ADXL312, mg */
#define ADXL312_SELF_TEST_MIN_DEVIATION				300
/* Self-test maximum deviation for ADXL312, mg */
#define ADXL312_SELF_TEST_MAX_DEVIATION				3400

/* Self-test minimum deviation for ADXL313, mg */
#define ADXL313_SELF_TEST_MIN_DEVIATION				300
/* Self-test maximum deviation for ADXL313, mg */
#define ADXL313_SELF_TEST_MAX_DEVIATION				3700

/* Self-test minimum deviation for ADXL314, mg */
#define ADXL314_SELF_TEST_MIN_DEVIATION				100
/* Self-test maximum deviation for ADXL314, mg */
#define ADXL314_SELF_TEST_MAX_DEVIATION				8000

/* Factor used for distinguishing between ADXL312 and ADXL313 ranges */
#define ADXL313_RANGE_FACTOR					4

/* Activity/inactivity detection defines */
/* Enable axis for participating in act/inact detection */
#define ADXL313_X_EN        					NO_OS_BIT(1)
#define ADXL313_Y_EN        					NO_OS_BIT(2)
#define ADXL313_Z_EN        					NO_OS_BIT(3)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxl313_type` is an enumeration that defines the different device
 * types for the ADXL series accelerometers, specifically identifying the
 * ADXL312, ADXL313, and ADXL314 models. This enum is used to distinguish
 * between these models in the driver code, allowing for specific
 * handling and configuration based on the device type.
 *
 * @param ID_ADXL312 Represents the device ID for ADXL312.
 * @param ID_ADXL313 Represents the device ID for ADXL313.
 * @param ID_ADXL314 Represents the device ID for ADXL314.
 ******************************************************************************/
enum adxl313_type {
	ID_ADXL312 = 0,
	ID_ADXL313 = 1,
	ID_ADXL314 = 2
};

/***************************************************************************//**
 * @brief The `adxl313_comm_type` is an enumeration that defines the
 * communication types available for the ADXL313 device, specifically SPI
 * and I2C. This enum is used to specify how the device will communicate
 * with the host system, allowing for flexibility in interfacing with
 * different hardware configurations.
 *
 * @param ADXL313_SPI_COMM Represents the SPI communication type for the ADXL313
 * device.
 * @param ADXL313_I2C_COMM Represents the I2C communication type for the ADXL313
 * device.
 ******************************************************************************/
enum adxl313_comm_type {
	ADXL313_SPI_COMM = 0,
	ADXL313_I2C_COMM = 1
};

/***************************************************************************//**
 * @brief The `adxl313_op_mode` enumeration defines the operating modes for the
 * ADXL313 accelerometer device. It includes two modes: `ADXL313_STDBY`,
 * which is used to put the device in standby mode, and `ADXL313_MEAS`,
 * which is used to set the device to measurement mode. These modes are
 * essential for controlling the power consumption and functionality of
 * the device, allowing it to either conserve energy or actively measure
 * acceleration data.
 *
 * @param ADXL313_STDBY Represents the standby mode of the ADXL313 device.
 * @param ADXL313_MEAS Represents the measurement mode of the ADXL313 device.
 ******************************************************************************/
enum adxl313_op_mode {
	ADXL313_STDBY = 0,
	ADXL313_MEAS = 1
};

/***************************************************************************//**
 * @brief The `adxl313_lp_mode` enumeration defines the different low power
 * operating modes available for the ADXL313 accelerometer. It includes
 * options for no low power mode, enabling low power mode, and an
 * automatic sleep mode, which helps in managing power consumption based
 * on the device's operational requirements.
 *
 * @param ADXL313_LP_MODE_NONE Represents the absence of low power mode.
 * @param ADXL313_LP_MODE_ON Indicates that the low power mode is enabled.
 * @param ADXL313_LP_MODE_AUTOSLEEP Represents the automatic sleep mode in low
 * power operation.
 ******************************************************************************/
enum adxl313_lp_mode {
	ADXL313_LP_MODE_NONE = 0,
	ADXL313_LP_MODE_ON = 1,
	ADXL313_LP_MODE_AUTOSLEEP = 2
};

/***************************************************************************//**
 * @brief The `adxl313_wake_up_f` enumeration defines the possible wake-up
 * frequencies for the ADXL313 accelerometer when it is in sleep mode.
 * These frequencies determine how often the device will wake up to check
 * for activity, allowing for power-saving while still monitoring for
 * events. The enumeration provides four options, ranging from 1 Hz to 8
 * Hz, giving flexibility in balancing power consumption and
 * responsiveness.
 *
 * @param ADXL313_WU_8_HZ Represents a wake-up frequency of 8 Hz.
 * @param ADXL313_WU_4_HZ Represents a wake-up frequency of 4 Hz.
 * @param ADXL313_WU_2_HZ Represents a wake-up frequency of 2 Hz.
 * @param ADXL313_WU_1_HZ Represents a wake-up frequency of 1 Hz.
 ******************************************************************************/
enum adxl313_wake_up_f {
	ADXL313_WU_8_HZ = 0,
	ADXL313_WU_4_HZ = 1,
	ADXL313_WU_2_HZ = 2,
	ADXL313_WU_1_HZ = 3
};

/***************************************************************************//**
 * @brief The `adxl313_range` enum defines the possible measurement ranges for
 * the ADXL313 accelerometer, allowing the user to select the desired
 * sensitivity and range for acceleration measurements. Each enumerator
 * corresponds to a specific range in g-forces, providing flexibility in
 * configuring the device for different applications and ensuring
 * accurate readings within the selected range.
 *
 * @param ADXL313_0_5G_RANGE Represents a measurement range of ±0.5g.
 * @param ADXL313_1G_RANGE Represents a measurement range of ±1g.
 * @param ADXL313_2G_RANGE Represents a measurement range of ±2g.
 * @param ADXL313_4G_RANGE Represents a measurement range of ±4g.
 * @param ADXL313_1_5G_RANGE Represents a measurement range of ±1.5g.
 * @param ADXL313_3G_RANGE Represents a measurement range of ±3g.
 * @param ADXL313_6G_RANGE Represents a measurement range of ±6g.
 * @param ADXL313_12G_RANGE Represents a measurement range of ±12g.
 * @param ADXL313_200G_RANGE Represents a measurement range of ±200g.
 ******************************************************************************/
enum adxl313_range {
	ADXL313_0_5G_RANGE = 0,
	ADXL313_1G_RANGE = 1,
	ADXL313_2G_RANGE = 2,
	ADXL313_4G_RANGE = 3,
	ADXL313_1_5G_RANGE = 4,
	ADXL313_3G_RANGE = 5,
	ADXL313_6G_RANGE = 6,
	ADXL313_12G_RANGE = 7,
	ADXL313_200G_RANGE = 8
};

/***************************************************************************//**
 * @brief The `adxl313_resolution` enum defines the possible resolution settings
 * for the ADXL313 accelerometer device, allowing the user to select
 * between 10-bit, 11-bit, 12-bit, and 13-bit resolutions. This setting
 * affects the precision of the accelerometer's output data, with higher
 * bit resolutions providing more precise measurements.
 *
 * @param ADXL313_10_BIT_RES Represents a 10-bit resolution setting for the
 * ADXL313 device.
 * @param ADXL313_11_BIT_RES Represents an 11-bit resolution setting for the
 * ADXL313 device.
 * @param ADXL313_12_BIT_RES Represents a 12-bit resolution setting for the
 * ADXL313 device.
 * @param ADXL313_13_BIT_RES Represents a 13-bit resolution setting for the
 * ADXL313 device.
 ******************************************************************************/
enum adxl313_resolution {
	ADXL313_10_BIT_RES = 0,
	ADXL313_11_BIT_RES = 1,
	ADXL313_12_BIT_RES = 2,
	ADXL313_13_BIT_RES = 3
};

/***************************************************************************//**
 * @brief The `adxl313_odr` enumeration defines the possible output data rates
 * (ODR) for the ADXL313 accelerometer. Each enumerator corresponds to a
 * specific frequency at which the device can output data, ranging from
 * 6.25 Hz to 3200 Hz. This allows users to select the appropriate data
 * rate for their application needs, balancing between power consumption
 * and data update frequency.
 *
 * @param ADXL313_ODR_6_25HZ Represents an output data rate of 6.25 Hz.
 * @param ADXL313_ODR_12_5HZ Represents an output data rate of 12.5 Hz.
 * @param ADXL313_ODR_25HZ Represents an output data rate of 25 Hz.
 * @param ADXL313_ODR_50HZ Represents an output data rate of 50 Hz.
 * @param ADXL313_ODR_100HZ Represents an output data rate of 100 Hz.
 * @param ADXL313_ODR_200HZ Represents an output data rate of 200 Hz.
 * @param ADXL313_ODR_400HZ Represents an output data rate of 400 Hz.
 * @param ADXL313_ODR_800HZ Represents an output data rate of 800 Hz.
 * @param ADXL313_ODR_1600HZ Represents an output data rate of 1600 Hz.
 * @param ADXL313_ODR_3200HZ Represents an output data rate of 3200 Hz.
 ******************************************************************************/
enum adxl313_odr {
	ADXL313_ODR_6_25HZ = 0,
	ADXL313_ODR_12_5HZ = 1,
	ADXL313_ODR_25HZ = 2,
	ADXL313_ODR_50HZ = 3,
	ADXL313_ODR_100HZ = 4,
	ADXL313_ODR_200HZ = 5,
	ADXL313_ODR_400HZ = 6,
	ADXL313_ODR_800HZ = 7,
	ADXL313_ODR_1600HZ = 8,
	ADXL313_ODR_3200HZ = 9
};

/***************************************************************************//**
 * @brief The `adxl313_axis` enumeration defines constants for the three axes
 * (X, Y, and Z) of the ADXL313 accelerometer. Each constant is
 * associated with a specific integer value, allowing for easy reference
 * and manipulation of axis-specific data within the accelerometer's
 * operations.
 *
 * @param ADXL313_X_AXIS Represents the X-axis of the accelerometer.
 * @param ADXL313_Y_AXIS Represents the Y-axis of the accelerometer.
 * @param ADXL313_Z_AXIS Represents the Z-axis of the accelerometer.
 ******************************************************************************/
enum adxl313_axis {
	ADXL313_X_AXIS = 0,
	ADXL313_Y_AXIS = 1,
	ADXL313_Z_AXIS = 2
};

/***************************************************************************//**
 * @brief The `adxl313_fifo_mode` enumeration defines the different FIFO
 * operating modes available for the ADXL313 accelerometer. Each mode
 * corresponds to a specific way the device handles data buffering and
 * retrieval, allowing for flexibility in how data is managed and
 * processed. The modes include bypass, FIFO, stream, and triggered, each
 * serving different use cases depending on the application's
 * requirements.
 *
 * @param ADXL313_BYPAS_MODE Represents the bypass mode with a value of 0.
 * @param ADXL313_FIFO_MODE Represents the FIFO mode with a value of 1.
 * @param ADXL313_STREAM_MODE Represents the stream mode with a value of 2.
 * @param ADXL313_TRIGGERED_MODE Represents the triggered mode with a value of
 * 3.
 ******************************************************************************/
enum adxl313_fifo_mode {
	ADXL313_BYPAS_MODE = 0,
	ADXL313_FIFO_MODE = 1,
	ADXL313_STREAM_MODE = 2,
	ADXL313_TRIGGERED_MODE = 3
};

/***************************************************************************//**
 * @brief The `adxl313_int_pol` enumeration defines the possible states for the
 * interrupt polarity of the ADXL313 device. It allows the user to
 * specify whether the interrupt signal should be active high or active
 * low, which is crucial for configuring the device to work correctly
 * with different hardware setups that may require specific interrupt
 * signal levels.
 *
 * @param ADXL313_INT_ACTIVE_HIGH Represents the active high state for the
 * interrupt polarity.
 * @param ADXL313_INT_ACTIVE_LOW Represents the active low state for the
 * interrupt polarity.
 ******************************************************************************/
enum adxl313_int_pol {
	ADXL313_INT_ACTIVE_HIGH = 0,
	ADXL313_INT_ACTIVE_LOW = 1
};

/***************************************************************************//**
 * @brief The `bit_action` enum is a simple enumeration used to define two
 * possible actions for a bit: enabling or disabling it. It provides a
 * clear and concise way to represent these actions with the constants
 * `DISABLE_E` and `ENABLE_E`, corresponding to the integer values 0 and
 * 1, respectively. This enum is typically used in contexts where a
 * binary decision is needed to either activate or deactivate a feature
 * or setting.
 *
 * @param DISABLE_E Represents the action to disable a bit, with a value of 0.
 * @param ENABLE_E Represents the action to enable a bit, with a value of 1.
 ******************************************************************************/
enum bit_action {
	DISABLE_E = 0,
	ENABLE_E = 1
};

/***************************************************************************//**
 * @brief The `adxl313_comm_init_param` is a union data structure designed to
 * hold initialization parameters for communication with the ADXL313
 * device. It provides two options for communication initialization: one
 * for I2C and another for SPI, allowing the user to select the
 * appropriate communication protocol for their application. This union
 * ensures that only one of the communication protocols is initialized at
 * a time, optimizing memory usage and simplifying the initialization
 * process for the ADXL313 accelerometer.
 *
 * @param i2c_init I2C Initialization structure.
 * @param spi_init SPI Initialization structure.
 ******************************************************************************/
union adxl313_comm_init_param {
	/** I2C Initialization structure. */
	struct no_os_i2c_init_param i2c_init;
	/** SPI Initialization structure. */
	struct no_os_spi_init_param spi_init;
} ;

/***************************************************************************//**
 * @brief The `adxl313_frac_repr` structure is used to represent acceleration
 * values in a fractional format, where the `integer` field holds the
 * whole number part and the `fractional` field holds the fractional
 * part. This structure is particularly useful for converting raw
 * accelerometer data into a more precise and human-readable format,
 * allowing for detailed analysis and processing of acceleration
 * measurements.
 *
 * @param integer Stores the integer part of the fractional representation.
 * @param fractional Stores the fractional part of the representation as a
 * 32-bit integer.
 ******************************************************************************/
struct adxl313_frac_repr {
	int64_t integer;
	int32_t fractional;
};

/***************************************************************************//**
 * @brief The `adxl313_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up an ADXL313 device.
 * It includes a union for communication initialization parameters,
 * allowing for flexibility between SPI and I2C interfaces. The structure
 * also specifies the communication type and device type through
 * enumerations, ensuring that the correct settings are applied for the
 * specific device and communication protocol being used. This structure
 * is essential for initializing the device correctly before any
 * operations can be performed.
 *
 * @param comm_init A union that holds the initialization parameters for device
 * communication, either SPI or I2C.
 * @param comm_type An enum that specifies the type of communication used,
 * either ADXL355_SPI_COMM or ADXL355_I2C_COMM.
 * @param dev_type An enum that indicates the type of device, which can be
 * ADXL312, ADXL313, or ADXL314.
 ******************************************************************************/
struct adxl313_init_param {
	/** Device Communication initialization structure: either SPI or I2C */
	union adxl313_comm_init_param comm_init;
	/** Device Communication type: ADXL355_SPI_COMM, ADXL355_I2C_COMM */
	enum adxl313_comm_type comm_type;
	/** Device type: ADXL312, 313, or 314 */
	enum adxl313_type dev_type;
};

/***************************************************************************//**
 * @brief The `_adxl313_int_reg_flags` structure is a bit-field representation
 * used to hold various interrupt flags for the ADXL313 accelerometer.
 * Each field in the structure corresponds to a specific interrupt
 * condition, such as overrun, watermark, inactivity, activity, and data
 * readiness. The reserved fields are included for alignment purposes or
 * potential future use. This structure is crucial for managing and
 * interpreting the interrupt status of the ADXL313 device.
 *
 * @param OVERRUN A 1-bit flag indicating an overrun condition.
 * @param WATERMARK A 1-bit flag indicating a watermark condition.
 * @param reserved A 1-bit reserved field for future use or alignment.
 * @param INACTIVITY A 1-bit flag indicating inactivity.
 * @param ACTIVITY A 1-bit flag indicating activity.
 * @param reserved1 A 2-bit reserved field for future use or alignment.
 * @param DATA_READY A 1-bit flag indicating that data is ready.
 ******************************************************************************/
struct _adxl313_int_reg_flags {
	uint8_t OVERRUN    : 1;
	uint8_t WATERMARK  : 1;
	uint8_t reserved   : 1;
	uint8_t INACTIVITY : 1;
	uint8_t ACTIVITY   : 1;
	uint8_t reserved1  : 2;
	uint8_t DATA_READY : 1;
};

/***************************************************************************//**
 * @brief The `adxl313_int_en_reg_flags` is a union designed to manage the
 * interrupt enable flags for the ADXL313 accelerometer. It provides two
 * ways to access the interrupt enable settings: as a structured set of
 * individual flags through the `fields` member, or as a single byte
 * value through the `value` member. This dual access allows for both
 * fine-grained control and efficient bulk operations on the interrupt
 * enable settings.
 *
 * @param fields A structure of type _adxl313_int_reg_flags that holds
 * individual interrupt flags.
 * @param value A uint8_t value representing the combined state of all interrupt
 * flags.
 ******************************************************************************/
union adxl313_int_en_reg_flags {
	struct _adxl313_int_reg_flags fields;
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `adxl313_int_map_reg_flags` is a union designed to manage the
 * interrupt mapping flags for the ADXL313 accelerometer. It provides a
 * way to access the interrupt flags either as individual bits through
 * the `fields` structure or as a single byte through the `value` member.
 * This dual access method allows for flexible manipulation and reading
 * of the interrupt mapping configuration, which is crucial for setting
 * up the device's interrupt behavior.
 *
 * @param fields A structure of type _adxl313_int_reg_flags that holds
 * individual interrupt flag fields.
 * @param value A uint8_t value representing the combined interrupt flags as a
 * single byte.
 ******************************************************************************/
union adxl313_int_map_reg_flags {
	struct _adxl313_int_reg_flags fields;
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `adxl313_int_src_reg_flags` is a union that encapsulates the
 * interrupt source flags for the ADXL313 accelerometer. It provides two
 * ways to access the interrupt source information: as a structured set
 * of individual flags through the `fields` member, or as a single byte
 * value through the `value` member. This design allows for flexible
 * manipulation and interpretation of the interrupt source data,
 * facilitating both bitwise operations and structured access to specific
 * interrupt flags.
 *
 * @param fields A structure holding the flags for ADXL313 interrupt sources.
 * @param value A uint8_t representing the interrupt source register value as a
 * whole.
 ******************************************************************************/
union adxl313_int_src_reg_flags {
	struct _adxl313_int_reg_flags fields;
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `_adxl313_act_inact_ctl_flags` structure is used to configure the
 * activity and inactivity detection features of the ADXL313
 * accelerometer. It consists of eight 1-bit fields that enable or
 * disable detection on the X, Y, and Z axes, as well as AC coupling for
 * both activity and inactivity detection. This structure allows for
 * fine-grained control over which axes are monitored for motion and
 * whether AC or DC coupling is used in the detection process.
 *
 * @param INACT_Z_EN A 1-bit flag to enable inactivity detection on the Z-axis.
 * @param INACT_Y_EN A 1-bit flag to enable inactivity detection on the Y-axis.
 * @param INACT_X_EN A 1-bit flag to enable inactivity detection on the X-axis.
 * @param INACT_AC_EN A 1-bit flag to enable AC coupling for inactivity
 * detection.
 * @param ACT_Z_EN A 1-bit flag to enable activity detection on the Z-axis.
 * @param ACT_Y_EN A 1-bit flag to enable activity detection on the Y-axis.
 * @param ACT_X_EN A 1-bit flag to enable activity detection on the X-axis.
 * @param ACT_AC_EN A 1-bit flag to enable AC coupling for activity detection.
 ******************************************************************************/
struct _adxl313_act_inact_ctl_flags {
	uint8_t INACT_Z_EN     : 1;
	uint8_t INACT_Y_EN     : 1;
	uint8_t INACT_X_EN     : 1;
	uint8_t INACT_AC_EN    : 1;
	uint8_t ACT_Z_EN       : 1;
	uint8_t ACT_Y_EN       : 1;
	uint8_t ACT_X_EN       : 1;
	uint8_t ACT_AC_EN      : 1;
};

/***************************************************************************//**
 * @brief The `adxl313_act_inact_ctl_flags` is a union that provides two ways to
 * access the activity and inactivity control flags for the ADXL313
 * device. It allows for both bit-level manipulation through the `fields`
 * structure, which breaks down the control flags into individual bits
 * for each axis and AC/DC mode, and byte-level manipulation through the
 * `value` member, which treats the flags as a single 8-bit value. This
 * design facilitates flexible and efficient configuration of the
 * device's activity and inactivity detection features.
 *
 * @param fields A structure containing individual bit fields for activity and
 * inactivity control flags.
 * @param value A single byte representation of the activity and inactivity
 * control flags.
 ******************************************************************************/
union adxl313_act_inact_ctl_flags {
	struct _adxl313_act_inact_ctl_flags fields;
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `adxl313_comm_desc` is a union that encapsulates the communication
 * descriptors for the ADXL313 device, allowing it to interface with
 * either I2C or SPI communication protocols. This union provides
 * flexibility in selecting the communication method by holding a pointer
 * to either an I2C descriptor or an SPI descriptor, depending on the
 * communication type used for the device.
 *
 * @param i2c_desc Pointer to an I2C descriptor structure for I2C communication.
 * @param spi_desc Pointer to an SPI descriptor structure for SPI communication.
 ******************************************************************************/
union adxl313_comm_desc {
	/** I2C Descriptor */
	struct no_os_i2c_desc *i2c_desc;
	/** SPI Descriptor */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `adxl313_dev` structure is a comprehensive representation of an
 * ADXL313 device, encapsulating all necessary parameters for its
 * operation and configuration. It includes descriptors for communication
 * (either I2C or SPI), device type, measurement range, resolution, and
 * operational modes. Additionally, it manages user-defined offset
 * adjustments for each axis, a scale factor for data conversion, and
 * settings for FIFO operation and activity/inactivity detection. This
 * structure is essential for initializing and controlling the ADXL313
 * accelerometer, providing a detailed interface for interacting with the
 * device's features and capabilities.
 *
 * @param com_desc Device communication descriptor, either I2C or SPI.
 * @param comm_type Specifies the communication type, either SPI or I2C.
 * @param dev_type Indicates the type of ADXL device (e.g., ADXL312, ADXL313,
 * ADXL314).
 * @param range Defines the measurement range of the ADXL313.
 * @param resolution Specifies the resolution of the ADXL313.
 * @param op_mode Indicates the operation mode, either standby or measure.
 * @param lp_mode Specifies the low power operation mode (none, on, or
 * autosleep).
 * @param odr Defines the output data rate of the ADXL313.
 * @param x_offset_raw User-set offset adjustment for the X axis in twos
 * complement format.
 * @param y_offset_raw User-set offset adjustment for the Y axis in twos
 * complement format.
 * @param z_offset_raw User-set offset adjustment for the Z axis in twos
 * complement format.
 * @param scale_factor_mult Multiplier for converting data to m/s^2.
 * @param fifo_mode Specifies the FIFO mode selection.
 * @param fifo_samples Number of FIFO samples before event signaling.
 * @param act_thr Threshold value for detecting activity in raw data.
 * @param inact_thr Threshold value for detecting inactivity in raw data.
 * @param time_inact Time duration for declaring inactivity.
 * @param act_inact_ctl Flags for activity and inactivity control settings.
 * @param comm_buff Buffer used for communication with the ADXL313.
 ******************************************************************************/
struct adxl313_dev {
	/** Device communication descriptor */
	union adxl313_comm_desc com_desc;
	/** Device Communication type: ADXL313_SPI_COMM, ADXL313_I2C_COMM */
	enum adxl313_comm_type comm_type;
	/** Device type */
	enum adxl313_type dev_type;
	/** ADXL313 measurement range */
	enum adxl313_range range;
	/** ADXL313 resolution */
	enum adxl313_resolution resolution;
	/** ADXL313 operation mode, STDBY or MEASURE */
	enum adxl313_op_mode op_mode;
	/** ADXL313 LOW POOWER operation mode (NONE, ON, or AUTOSLEEP) */
	enum adxl313_lp_mode lp_mode;
	/** ADXL313 output data rate */
	enum adxl313_odr odr;
	/** User-set offset adjustments  on the X axis in twos complement
	 * format with a scale factor depending on device */
	uint8_t x_offset_raw;
	/** User-set offset adjustments  on the Y axis in twos complement
	 * format with a scale factor depending on device */
	uint8_t y_offset_raw;
	/** User-set offset adjustments  on the Z axis in twos complement
	 * format with a scale factor depending on device */
	uint8_t z_offset_raw;
	/** Scale factor multiplier for data conversion to m/s^2 */
	int64_t scale_factor_mult;
	/** ADXL313 FIFO mode selection */
	enum adxl313_fifo_mode fifo_mode;
	/** Number of FISO samples before FIFO event signalling */
	uint8_t fifo_samples;
	/** Threshold value for detecting activity, raw data */
	uint8_t act_thr;
	/** Threshold value for detecting inactivity, raw data */
	uint8_t inact_thr;
	/** Amount of time for inactivity to be declared */
	uint8_t time_inact;
	/** The contents of the ACT_INACT_CTL register */
	union adxl313_act_inact_ctl_flags act_inact_ctl;
	/** Buffer used for communication with ADXL313 */
	uint8_t comm_buff[24];
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function is used to initialize the ADXL313 accelerometer device
 * by allocating necessary resources and configuring communication
 * settings. It must be called before any other operations on the device.
 * The function checks for the presence of the device by verifying its ID
 * and sets the device to standby mode initially. If the initialization
 * fails at any point, appropriate cleanup is performed, and an error
 * code is returned. Ensure that the `init_param` structure is properly
 * populated with valid communication parameters and device type before
 * calling this function.
 *
 * @param device A pointer to a pointer of `struct adxl313_dev` where the
 * initialized device structure will be stored. The caller retains
 * ownership of this pointer.
 * @param init_param A structure of type `struct adxl313_init_param` that
 * contains the initialization parameters, including
 * communication type and device type. This structure must be
 * properly initialized before passing it to the function.
 * @return Returns 0 on successful initialization, or a negative error code
 * indicating the type of failure (e.g., -ENOMEM for memory allocation
 * failure, -ENODEV for device not found, -EPIPE for communication
 * error).
 ******************************************************************************/
int adxl313_init(struct adxl313_dev **device,
		 struct adxl313_init_param init_param);

/***************************************************************************//**
 * @brief This function should be called to release resources associated with an
 * `adxl313_dev` instance that was previously initialized with
 * `adxl313_init()`. It is important to ensure that this function is
 * called only after the device is no longer needed, as it will free the
 * memory allocated for the device structure. If the device was not
 * properly initialized or if it has already been removed, the behavior
 * is undefined. The function will handle both SPI and I2C communication
 * types appropriately.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid device that was initialized. If the pointer is invalid or
 * the device was not initialized, the function may return an error.
 * @return Returns 0 on success, indicating that the device was successfully
 * removed and resources were freed. A non-zero value indicates an error
 * occurred during the removal process.
 ******************************************************************************/
int adxl313_remove(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to perform a software reset on the ADXL313
 * device, which restores the device to its default settings. It should
 * be called when the device needs to be reinitialized or when it is not
 * functioning as expected. Before calling this function, ensure that the
 * device has been properly initialized. If the device type is
 * `ID_ADXL314`, the function will return an error without performing the
 * reset. After a successful reset, the device's configuration parameters
 * will be set to their default values.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null. If the device type is `ID_ADXL314`, the function
 * will return an error code without performing any operation.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int adxl313_software_reset(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to set the operational mode of the ADXL313
 * device, which can either be in standby or measurement mode. It should
 * be called after the device has been initialized. If an unsupported
 * mode is provided, the function will return an error. The operation
 * mode affects how the device behaves and responds to data requests, so
 * it is important to set it appropriately before performing
 * measurements.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param op_mode The desired operation mode, which must be either
 * `ADXL313_STDBY` or `ADXL313_MEAS`. If an invalid mode is
 * provided, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation mode
 * is not supported or if there is a communication error.
 ******************************************************************************/
int adxl313_set_op_mode(struct adxl313_dev *dev,
			enum adxl313_op_mode op_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current operational mode of the
 * ADXL313 device, which can either be in measurement mode or standby
 * mode. It should be called after the device has been initialized and
 * powered on. The function reads the power control register to determine
 * the mode and updates the provided `op_mode` pointer with the result.
 * If the device is not in a valid state or if there is an error reading
 * the register, the function will return an error code.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device instance.
 * @param op_mode A pointer to an `enum adxl313_op_mode` variable where the
 * current operation mode will be stored. This must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_get_op_mode(struct adxl313_dev *dev,
			enum adxl313_op_mode *op_mode);

/***************************************************************************//**
 * @brief This function is used to configure the offset for a specific axis of
 * the accelerometer device, which is essential for accurate
 * measurements. It should be called after the device has been
 * initialized and is ready for configuration. The offset value is
 * provided in micro-g (ug) and is scaled according to the device type
 * and resolution settings. If the specified axis is invalid or if the
 * device type is not recognized, the function will return an error code.
 * It is important to ensure that the device is in the correct
 * operational state before calling this function.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param offset_ug The offset value in micro-g to be set for the specified
 * axis. This value should be within the acceptable range for
 * the device, and the function will handle scaling based on
 * the device type.
 * @param axis An enumerated value of type `adxl313_axis` indicating which axis
 * to set the offset for. Valid values are `ADXL313_X_AXIS`,
 * `ADXL313_Y_AXIS`, and `ADXL313_Z_AXIS`. If an invalid axis is
 * provided, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_set_offset(struct adxl313_dev *dev,
		       int32_t offset_ug,
		       enum adxl313_axis axis);

/***************************************************************************//**
 * @brief This function is used to obtain the offset value for a specified axis
 * of the ADXL313 accelerometer. It must be called after the device has
 * been initialized and configured. The function takes a pointer to an
 * `int32_t` variable where the offset value will be stored. The `axis`
 * parameter must be one of the defined axis enumerations (X, Y, or Z).
 * If the specified axis is invalid, the function will return an error
 * code. Additionally, if the device type is not recognized, an error
 * will also be returned.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param offset_ug A pointer to an `int32_t` where the offset value in micro-g
 * will be stored. Must not be null.
 * @param axis An enumeration value representing the axis for which the offset
 * is requested. Valid values are `ADXL313_X_AXIS`,
 * `ADXL313_Y_AXIS`, or `ADXL313_Z_AXIS`. If an invalid value is
 * provided, the function will return an error.
 * @return Returns 0 on success, or a negative error code if an error occurs,
 * such as invalid axis or unrecognized device type.
 ******************************************************************************/
int adxl313_get_offset(struct adxl313_dev *dev,
		       int32_t *offset_ug,
		       enum adxl313_axis axis);

/***************************************************************************//**
 * @brief This function is used to set the raw offset value for a specified axis
 * of the ADXL313 accelerometer. It should be called after the device has
 * been initialized and configured. The function clamps the provided
 * offset value to ensure it falls within the valid range based on the
 * device type and resolution. If the provided axis is invalid or the
 * offset value is out of range, the function will return an error. The
 * function modifies the internal state of the device by updating the
 * offset for the specified axis.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param offset_raw The raw offset value to be set, which should be within the
 * valid range determined by the device type and resolution.
 * If the value is out of range, the function will return an
 * error.
 * @param axis The axis for which the offset is being set, specified as an
 * `adxl313_axis` enum. Valid values are `ADXL313_X_AXIS`,
 * `ADXL313_Y_AXIS`, and `ADXL313_Z_AXIS`. If an invalid axis is
 * provided, the function will return an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_set_raw_offset(struct adxl313_dev *dev,
			   int32_t offset_raw,
			   enum adxl313_axis axis);

/***************************************************************************//**
 * @brief This function is used to obtain the raw offset value for a specified
 * axis of the ADXL313 accelerometer. It should be called after the
 * device has been properly initialized and configured. The function
 * checks if the provided axis is valid; if not, it returns an error. The
 * retrieved offset value is stored in the location pointed to by the
 * `offset_raw` parameter. The function may return an error code if the
 * device type is not recognized or if the read operation fails.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param offset_raw Pointer to an `int32_t` where the raw offset value will be
 * stored. Must not be null.
 * @param axis Specifies the axis for which the offset is to be retrieved. Valid
 * values are `ADXL313_X_AXIS`, `ADXL313_Y_AXIS`, and
 * `ADXL313_Z_AXIS`. If an invalid value is provided, the function
 * will return an error.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int adxl313_get_raw_offset(struct adxl313_dev *dev,
			   int32_t *offset_raw,
			   enum adxl313_axis axis);

/***************************************************************************//**
 * @brief This function retrieves the raw acceleration data from the ADXL313
 * accelerometer for the X, Y, and Z axes. It should be called after the
 * device has been properly initialized and configured. The function
 * expects valid pointers for the output parameters to store the raw
 * data. If the device is not ready or an error occurs during the read
 * operation, the function will return an error code, allowing the caller
 * to handle it appropriately.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param x_raw Pointer to an `int16_t` where the raw X-axis data will be
 * stored. Must not be null.
 * @param y_raw Pointer to an `int16_t` where the raw Y-axis data will be
 * stored. Must not be null.
 * @param z_raw Pointer to an `int16_t` where the raw Z-axis data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adxl313_get_raw_xyz(struct adxl313_dev *dev,
			int16_t *x_raw,
			int16_t *y_raw,
			int16_t *z_raw);

/***************************************************************************//**
 * @brief This function is used to retrieve the acceleration values along the X,
 * Y, and Z axes from the ADXL313 accelerometer and convert them into a
 * fractional representation in meters per second squared (m/s^2). It
 * should be called after the device has been properly initialized and
 * configured. The function expects valid pointers to `struct
 * adxl313_frac_repr` for each axis, which will be populated with the
 * converted acceleration values. If the device is not ready or an error
 * occurs while reading the raw data, the function will return an error
 * code, and the output parameters will remain unchanged.
 *
 * @param dev Pointer to the `struct adxl313_dev` representing the device. Must
 * not be null and should point to a valid initialized device.
 * @param x_m_s2 Pointer to a `struct adxl313_frac_repr` where the converted
 * X-axis acceleration will be stored. Must not be null.
 * @param y_m_s2 Pointer to a `struct adxl313_frac_repr` where the converted
 * Y-axis acceleration will be stored. Must not be null.
 * @param z_m_s2 Pointer to a `struct adxl313_frac_repr` where the converted
 * Z-axis acceleration will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The output parameters will contain the converted acceleration
 * values if successful.
 ******************************************************************************/
int adxl313_get_xyz(struct adxl313_dev *dev,
		    struct adxl313_frac_repr *x_m_s2,
		    struct adxl313_frac_repr *y_m_s2,
		    struct adxl313_frac_repr *z_m_s2);

/***************************************************************************//**
 * @brief This function retrieves the number of entries currently stored in the
 * FIFO of the ADXL313 device. It should be called after the device has
 * been initialized and configured properly. The function reads the FIFO
 * status register and extracts the number of entries. If the device is
 * not ready or if there is an error in reading the register, the
 * function will return an error code. The caller must ensure that the
 * `entries_no` pointer is valid and points to a location where the
 * result can be stored.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param entries_no Pointer to a `uint8_t` variable where the number of FIFO
 * entries will be stored. Must not be null; the function will
 * write the number of entries to this location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adxl313_get_no_of_fifo_entries(struct adxl313_dev *dev,
				   uint8_t *entries_no);

/***************************************************************************//**
 * @brief This function configures the ADXL313 device to use a specified number
 * of samples in its FIFO buffer. It should be called after initializing
 * the device and before starting data collection. The `samples_no`
 * parameter must be within the valid range, which is from 0 to
 * `ADXL313_MAX_FIFO_SAMPLES_VAL`. If an invalid value is provided, the
 * function will return an error code without modifying the device state.
 * Successful execution updates the internal state of the device to
 * reflect the new FIFO sample count.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param samples_no An 8-bit unsigned integer specifying the number of samples
 * to be stored in the FIFO. Valid values range from 0 to
 * `ADXL313_MAX_FIFO_SAMPLES_VAL`. If the value exceeds this
 * range, the function will return an error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if the operation fails.
 ******************************************************************************/
int adxl313_set_fifo_samples(struct adxl313_dev *dev,
			     uint8_t samples_no);

/***************************************************************************//**
 * @brief This function configures the FIFO mode of the ADXL313 device, which
 * determines how data is stored in the FIFO buffer. It should be called
 * after initializing the device and before starting data acquisition.
 * The function accepts a mode parameter that specifies the desired FIFO
 * operation mode. If an invalid mode is provided, the function will
 * return an error. The FIFO mode can affect how data is processed and
 * retrieved, so it is important to set it according to the application's
 * requirements.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param mode The desired FIFO mode, which must be one of the values defined in
 * the `adxl313_fifo_mode` enum. Valid values are
 * `ADXL313_BYPAS_MODE`, `ADXL313_FIFO_MODE`, `ADXL313_STREAM_MODE`,
 * and `ADXL313_TRIGGERED_MODE`. If an invalid value is provided,
 * the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_set_fifo_mode(struct adxl313_dev *dev,
			  enum adxl313_fifo_mode mode);

/***************************************************************************//**
 * @brief This function retrieves raw accelerometer data from the FIFO buffer of
 * the ADXL313 device. It should be called after ensuring that the FIFO
 * is populated with data, which can be checked using the
 * `adxl313_get_no_of_fifo_entries` function. The function will fill the
 * provided arrays with the raw X, Y, and Z axis data for the specified
 * number of entries. If the number of entries is greater than zero, the
 * function will read the corresponding data from the FIFO. It is
 * important to note that the function may introduce a small delay while
 * reading the data.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param entries Pointer to a `uint8_t` variable that will hold the number of
 * FIFO entries read. The caller retains ownership and must
 * ensure it is initialized before calling.
 * @param x_raw Pointer to an array of `int16_t` where the raw X-axis data will
 * be stored. Must not be null and should have enough space to hold
 * the number of entries.
 * @param y_raw Pointer to an array of `int16_t` where the raw Y-axis data will
 * be stored. Must not be null and should have enough space to hold
 * the number of entries.
 * @param z_raw Pointer to an array of `int16_t` where the raw Z-axis data will
 * be stored. Must not be null and should have enough space to hold
 * the number of entries.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operation.
 ******************************************************************************/
int adxl313_get_raw_fifo_data(struct adxl313_dev *dev, uint8_t *entries,
			      int16_t *x_raw, int16_t *y_raw, int16_t *z_raw);

/***************************************************************************//**
 * @brief This function retrieves acceleration data stored in the FIFO of the
 * ADXL313 device and converts it into a more usable format. It should be
 * called after ensuring that the FIFO contains valid data, which can be
 * checked using the `adxl313_get_no_of_fifo_entries` function. The
 * function will populate the provided arrays with the converted
 * acceleration values for the x, y, and z axes. If the number of entries
 * is zero, the output arrays will remain unchanged. It is important to
 * ensure that the `entries` parameter points to a valid memory location
 * and that the `x`, `y`, and `z` pointers are also valid and allocated
 * to hold the expected number of entries.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param entries Pointer to a `uint8_t` that holds the number of entries to
 * read from the FIFO. The value must be less than or equal to
 * `ADXL313_MAX_FIFO_ENTRIES`.
 * @param x Pointer to an array of `adxl313_frac_repr` structures for storing
 * converted x-axis values. Must be allocated to hold at least the
 * number of entries specified.
 * @param y Pointer to an array of `adxl313_frac_repr` structures for storing
 * converted y-axis values. Must be allocated to hold at least the
 * number of entries specified.
 * @param z Pointer to an array of `adxl313_frac_repr` structures for storing
 * converted z-axis values. Must be allocated to hold at least the
 * number of entries specified.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_get_fifo_data(struct adxl313_dev *dev,
			  uint8_t *entries,
			  struct adxl313_frac_repr *x,
			  struct adxl313_frac_repr *y,
			  struct adxl313_frac_repr *z);

/***************************************************************************//**
 * @brief This function is used to set the activity threshold for the ADXL313
 * device, which determines the sensitivity for detecting motion. It
 * should be called after the device has been initialized and configured.
 * The threshold value is specified in micro-g (ug) and is scaled
 * according to the device type. If an invalid device type is provided,
 * the function will return an error. It is important to ensure that the
 * threshold value is within the acceptable range for the specific device
 * type.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param act_thr_ug The activity threshold value in micro-g (ug). Valid values
 * depend on the device type and should be within the range
 * defined by the device specifications.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_conf_act_thr(struct adxl313_dev *dev, uint32_t act_thr_ug);

/***************************************************************************//**
 * @brief This function is used to obtain the current activity threshold value
 * configured in the ADXL313 device. It should be called after the device
 * has been properly initialized and configured. The function reads the
 * threshold value from the device's register and converts it to micro-g
 * (ug) based on the device type. If the device type is not recognized,
 * an error will be returned. Ensure that the pointer for the output
 * parameter is valid and points to a memory location that can store the
 * resulting threshold value.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param act_thr_ug Pointer to a `uint32_t` where the activity threshold value
 * in micro-g will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_get_act_thr(struct adxl313_dev *dev, uint32_t *act_thr_ug);

/***************************************************************************//**
 * @brief This function is used to set the inactivity threshold for the ADXL313
 * accelerometer, which determines the level of acceleration below which
 * the device will consider itself inactive. It should be called after
 * the device has been initialized and configured. The threshold value is
 * specified in micro-g (ug) and must be within the valid range defined
 * by the device specifications. If an invalid device type is provided,
 * the function will return an error code. Additionally, if the
 * configuration is successful, the device's internal state will be
 * updated to reflect the new threshold.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param inact_thr_ug The inactivity threshold value in micro-g (ug). This
 * value should be within the acceptable range for the
 * device. If the value is out of range, the function will
 * handle it appropriately by clamping or returning an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as when an invalid device type is specified.
 ******************************************************************************/
int adxl313_conf_inact_thr(struct adxl313_dev *dev, uint32_t inact_thr_ug);

/***************************************************************************//**
 * @brief This function is used to obtain the current inactivity threshold
 * setting from the ADXL313 device. It should be called after the device
 * has been initialized and configured. The function reads the threshold
 * value from the device's register and scales it according to the
 * specific device type (ADXL312, ADXL313, or ADXL314). If the device
 * type is not recognized, an error is returned. The caller must ensure
 * that the pointer for the inactivity threshold is valid and can store
 * the resulting value.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param inact_thr_ug Pointer to a `uint32_t` where the inactivity threshold
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_get_inact_thr(struct adxl313_dev *dev, uint32_t *inact_thr_ug);

/***************************************************************************//**
 * @brief This function is used to set the time duration for which the device
 * must remain inactive before it triggers an inactivity event. It should
 * be called after the device has been initialized and configured. The
 * `time_inact_s` parameter specifies the inactivity time in seconds, and
 * it is important to ensure that this value is within the valid range.
 * If the provided value is invalid, the function will return an error
 * code without modifying the device's state.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param time_inact_s The time duration in seconds for inactivity detection.
 * Valid values are typically in the range of 0 to 255
 * seconds. The function will return an error if the value
 * is outside this range.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_conf_time_inact(struct adxl313_dev *dev, uint8_t time_inact_s);

/***************************************************************************//**
 * @brief This function is used to obtain the time inactivity setting from the
 * ADXL313 device, which determines how long the device must remain
 * inactive before it triggers an inactivity event. It should be called
 * after the device has been properly initialized. The function will
 * write the retrieved value into the provided pointer. If the device is
 * not responding or an error occurs during the read operation, the
 * function will return an error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param time_inact_s Pointer to a `uint8_t` where the time inactivity value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adxl313_get_time_inact(struct adxl313_dev *dev, uint8_t *time_inact_s);

/***************************************************************************//**
 * @brief This function is used to configure the activity and inactivity
 * detection settings of the ADXL313 device. It should be called after
 * the device has been initialized and is ready for configuration. The
 * `config` parameter allows the user to specify which axes are enabled
 * for activity and inactivity detection, as well as whether the
 * detection is based on AC or DC signals. If the provided `config` is
 * invalid, the function will not apply any changes, and an error code
 * will be returned.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param config A union of type `adxl313_act_inact_ctl_flags` that contains the
 * configuration settings for activity and inactivity detection.
 * The fields within this union specify which axes are enabled and
 * the type of detection (AC/DC). Caller retains ownership.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_conf_act_inact_ctl(struct adxl313_dev *dev,
			       union adxl313_act_inact_ctl_flags config);

/***************************************************************************//**
 * @brief This function is used to read the current configuration settings for
 * activity and inactivity detection from the ADXL313 device. It should
 * be called after the device has been initialized and is ready for
 * communication. The function will populate the provided `config`
 * parameter with the current settings. If the read operation fails, the
 * function will return an error code, and the contents of `config` will
 * remain unchanged.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * This must not be null and should point to a valid initialized
 * device instance.
 * @param config A pointer to a union of type `adxl313_act_inact_ctl_flags`
 * where the current configuration will be stored. This must not
 * be null; otherwise, the function will not be able to write the
 * configuration.
 * @return Returns 0 on success, indicating that the configuration was
 * successfully read and stored in `config`. On failure, it returns a
 * negative error code.
 ******************************************************************************/
int adxl313_get_conf_act_inact_ctl(struct adxl313_dev *dev,
				   union adxl313_act_inact_ctl_flags *config);

/***************************************************************************//**
 * @brief This function configures the ADXL313 accelerometer to detect activity
 * based on specified axes, threshold, and interrupt pin. It must be
 * called after initializing the device and before starting measurements.
 * The function allows the user to specify which axes (X, Y, Z) should be
 * monitored for activity, whether the detection should be based on AC or
 * DC signals, and the threshold for activity detection in micro-g. The
 * specified interrupt pin will be used to signal when activity is
 * detected. If any of the parameters are invalid, the function will
 * return an error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param act_axes Bitmask indicating which axes to enable for activity
 * detection. Valid values are combinations of `ADXL313_X_EN`,
 * `ADXL313_Y_EN`, and `ADXL313_Z_EN`.
 * @param act_ac_dc Specifies whether to use AC or DC coupling for activity
 * detection. Valid values are defined in the `bit_action`
 * enum.
 * @param act_thresh_ug Threshold for activity detection in micro-g. Must be a
 * non-negative value.
 * @param int_pin Interrupt pin number to be used for signaling activity
 * detection. Must be a valid pin number.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_set_activity_detection(struct adxl313_dev *dev,
				   uint8_t act_axes,
				   uint8_t act_ac_dc,
				   uint32_t act_thresh_ug,
				   uint8_t int_pin);

/***************************************************************************//**
 * @brief This function configures the inactivity detection feature of the
 * ADXL313 accelerometer. It should be called after initializing the
 * device and before starting measurements. The function allows the user
 * to specify which axes to monitor for inactivity, whether to use AC or
 * DC coupling for the detection, the threshold for inactivity in
 * micro-g, the time duration for which inactivity must be detected, and
 * the interrupt pin to be used for signaling inactivity events. It is
 * important to ensure that the specified threshold and time values are
 * within valid ranges to avoid unexpected behavior.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param inact_axes Bitmask indicating which axes (X, Y, Z) to monitor for
 * inactivity. Valid values are combinations of
 * `ADXL313_X_EN`, `ADXL313_Y_EN`, and `ADXL313_Z_EN`.
 * @param inact_ac_dc Specifies whether to use AC or DC coupling for inactivity
 * detection. Valid values are defined in the `bit_action`
 * enum.
 * @param inact_thresh_ug Threshold for inactivity detection in micro-g. Must be
 * a positive value.
 * @param inact_time Duration in seconds for which inactivity must be detected.
 * Must be a positive value.
 * @param int_pin Interrupt pin to be used for signaling inactivity events. Must
 * be a valid pin number.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_set_inactivity_detection(struct adxl313_dev *dev,
				     uint8_t inact_axes,
				     uint8_t inact_ac_dc,
				     uint32_t inact_thresh_ug,
				     uint8_t inact_time,
				     uint8_t int_pin);

/***************************************************************************//**
 * @brief This function configures the output data rate (ODR) of the ADXL313
 * accelerometer. It should be called after the device has been
 * initialized and before starting measurements. The ODR can be set to
 * various predefined rates, and the function will ensure that the
 * provided rate is valid. If an invalid rate is specified, the function
 * will return an error. Additionally, the function updates the internal
 * state of the device to reflect the new ODR.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param odr The desired output data rate, specified as an enum value from
 * `adxl313_odr`. Valid values range from `ADXL313_ODR_6_25HZ` to
 * `ADXL313_ODR_3200HZ`. If the value exceeds `ADXL313_ODR_3200HZ`,
 * the function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_set_odr(struct adxl313_dev *dev, enum adxl313_odr odr);

/***************************************************************************//**
 * @brief This function is used to obtain the current output data rate (ODR)
 * setting of the ADXL313 device. It should be called after the device
 * has been initialized and configured. The function reads the relevant
 * register and updates the provided pointer with the current ODR value.
 * If the read operation fails, the function will return an error code,
 * and the value pointed to by `odr` will remain unchanged.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param odr Pointer to an `enum adxl313_odr` variable where the ODR value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adxl313_get_odr(struct adxl313_dev *dev, enum adxl313_odr *odr);

/***************************************************************************//**
 * @brief This function is used to disable I2C communication for the ADXL313
 * device. It should be called when the device is initialized and the
 * user wants to switch from I2C to another communication mode, or when
 * I2C communication is no longer needed. The function checks the device
 * type to ensure that it is an ADXL313; if the device type is incorrect,
 * it will return an error code. It is important to ensure that the
 * device is properly initialized before calling this function.
 *
 * @param dev Pointer to a `struct adxl313_dev` that represents the device. This
 * pointer must not be null and should point to a valid device
 * structure that has been initialized. If the device type is not
 * `ID_ADXL313`, the function will return an error code.
 * @return Returns 0 on success, or a negative error code if the device type is
 * incorrect.
 ******************************************************************************/
int adxl313_disable_i2c(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function should be called to enable I2C communication for the
 * ADXL313 device. It is essential to ensure that the device type is
 * correctly set to `ID_ADXL313` before invoking this function. If the
 * device type is not `ID_ADXL313`, the function will return an error
 * code indicating access is denied. This function is typically used
 * after initializing the device to configure its communication mode.
 *
 * @param dev Pointer to a `struct adxl313_dev` that represents the device. This
 * pointer must not be null and should point to a valid device
 * structure that has been properly initialized.
 * @return Returns 0 on success if I2C is enabled; otherwise, it returns an
 * error code, specifically -EACCES if the device type is not
 * `ID_ADXL313`.
 ******************************************************************************/
int adxl313_enable_i2c(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to enable or disable low power mode for the
 * ADXL313 device. It must be called after the device has been
 * initialized and is in standby mode. When enabling low power mode, the
 * function adjusts the output data rate (ODR) if it is set above 400 Hz
 * or below 12.5 Hz to ensure compatibility with low power operation. If
 * the input parameter is invalid, the function will return an error code
 * without changing the device state.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param enable An enum value of type `bit_action` that specifies whether to
 * enable (ENABLE_E) or disable (DISABLE_E) low power mode. Must
 * be a valid enum value.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_set_low_power_mode(struct adxl313_dev *dev,
			       enum bit_action enable);

/***************************************************************************//**
 * @brief This function is used to configure the autosleep mode of the ADXL313
 * device, which allows the device to enter a low-power state after a
 * period of inactivity. It should be called after the device has been
 * initialized and is in standby mode. When enabling autosleep, the
 * inactivity threshold and the time of inactivity must be specified. If
 * the function is called with `ENABLE_E`, it will set the inactivity
 * threshold and time; if called with `DISABLE_E`, it will reset these
 * values to zero. The function also changes the device's low power mode
 * accordingly.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param enable An enum value of type `bit_action` that specifies whether to
 * enable or disable autosleep mode. Valid values are `ENABLE_E`
 * to enable and `DISABLE_E` to disable.
 * @param inact_thr A `uint8_t` value representing the inactivity threshold.
 * This value is used only when enabling autosleep mode.
 * @param time_inact_s A `uint8_t` value representing the time in seconds for
 * which inactivity is detected before entering autosleep.
 * This value is used only when enabling autosleep mode.
 * @return Returns 0 on success, or a negative error code on failure. No output
 * is produced via pointers.
 ******************************************************************************/
int adxl313_set_autosleep_mode(struct adxl313_dev *dev, enum bit_action enable,
			       uint8_t inact_thr, uint8_t time_inact_s);

/***************************************************************************//**
 * @brief This function should be called when you want to enable Link Mode,
 * which allows the device to perform activity and inactivity detection.
 * It is typically used after initializing the device and setting up the
 * necessary configurations for activity monitoring. Ensure that the
 * device is properly initialized before calling this function. If the
 * device is not initialized or if the provided `dev` pointer is null,
 * the function may not behave as expected.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * This pointer must not be null and should point to a valid device
 * structure that has been initialized. Passing a null pointer or an
 * uninitialized device may lead to undefined behavior.
 * @return Returns 0 on success, indicating that Link Mode has been successfully
 * enabled. A negative value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int adxl313_link_mode_enable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to disable Link Mode in the ADXL313 device,
 * which is necessary when the device is not required to perform activity
 * and inactivity detection functions. It should be called when the
 * application no longer needs to monitor these conditions, ensuring that
 * the device operates in a standard mode. There are no specific
 * preconditions for calling this function, but it is important to ensure
 * that the `dev` parameter points to a valid and initialized
 * `adxl313_dev` structure.
 *
 * @param dev A pointer to an `adxl313_dev` structure representing the device.
 * This parameter must not be null and should point to a valid device
 * that has been initialized. If the pointer is null or invalid, the
 * function may not behave as expected.
 * @return Returns 0 on success, indicating that Link Mode has been successfully
 * disabled. A negative value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int adxl313_link_mode_disable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to place the device into sleep mode, which
 * reduces power consumption. It must be called after the device has been
 * initialized and configured. The function allows the user to specify
 * the frequency of wake-up events while in sleep mode, which can be set
 * to 8 Hz, 4 Hz, 2 Hz, or 1 Hz. If an invalid frequency is provided, the
 * function will return an error. It is important to ensure that the
 * device is not already in sleep mode before calling this function, as
 * it may lead to unexpected behavior.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param wake_up_f_hz An enumeration value representing the frequency of wake-
 * up events during sleep. Valid values are
 * `ADXL313_WU_8_HZ`, `ADXL313_WU_4_HZ`, `ADXL313_WU_2_HZ`,
 * and `ADXL313_WU_1_HZ`. If the value exceeds
 * `ADXL313_WU_1_HZ`, the function will return an error.
 * @return Returns 0 on success, or a negative error code if an invalid
 * frequency is specified or if the operation fails.
 ******************************************************************************/
int adxl313_sleep(struct adxl313_dev *dev, enum adxl313_wake_up_f wake_up_f_hz);

/***************************************************************************//**
 * @brief This function is used to wake the ADXL313 device from sleep mode,
 * allowing it to resume normal operation. It should be called after the
 * device has been initialized and is currently in sleep mode. The
 * function first sets the operation mode to standby, disables the sleep
 * mode, and then sets the operation mode to measurement. If any of these
 * operations fail, the function will return an error code, indicating
 * the specific failure.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null. If the pointer is invalid or the device is not
 * in sleep mode, the function will return an error code.
 * @return Returns 0 on success, or a negative error code if any operation
 * fails.
 ******************************************************************************/
int adxl313_exit_sleep(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to configure the interrupt enable settings for
 * the ADXL313 device. It should be called after initializing the device
 * and before starting any operations that rely on interrupts. The
 * `en_ctl` parameter specifies which interrupts to enable, and it is
 * important to ensure that the device is properly initialized before
 * invoking this function. If the provided `dev` pointer is null, the
 * function will return an error.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param en_ctl Union containing the interrupt enable flags. The `value` field
 * should be set to indicate which interrupts to enable.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_conf_int_enable(struct adxl313_dev *dev,
			    union adxl313_int_en_reg_flags en_ctl);

/***************************************************************************//**
 * @brief This function is used to enable the activity detection interrupt for
 * the ADXL313 device. It should be called after the device has been
 * properly initialized and configured. The function reads the current
 * interrupt enable settings, modifies the relevant bit to enable
 * activity detection, and then applies the updated settings. If the
 * device is not initialized or if the read operation fails, the function
 * will return an error code.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device.
 * Must not be null. The caller retains ownership of this structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_activity_int_enable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to disable the activity interrupt feature of the
 * ADXL313 device. It should be called when the application no longer
 * requires activity detection, such as during power-saving modes or when
 * the device is not in use. Before calling this function, the device
 * must be properly initialized and configured. If the device is not
 * initialized or if there is an error in communication, the function
 * will return an error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null. The caller retains ownership of this structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_activity_int_disable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to enable the inactivity interrupt feature of
 * the ADXL313 device. It should be called after the device has been
 * properly initialized and configured. The function reads the current
 * interrupt enable settings, modifies the relevant bit to enable
 * inactivity detection, and then applies the updated settings. If the
 * device is not initialized or if there is an error during the read
 * operation, the function will return an error code.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device.
 * Must not be null. The caller retains ownership of this structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_inactivity_int_enable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to disable the inactivity interrupt feature of
 * the ADXL313 device. It should be called when the inactivity detection
 * is no longer needed, such as when the device is being reconfigured or
 * powered down. Before calling this function, the device must be
 * properly initialized and configured. If the device is not initialized
 * or if there is an error during the operation, the function will return
 * an error code.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device.
 * Must not be null. The caller retains ownership of this structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_inactivity_int_disable(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to set the mapping of interrupt sources to
 * specific pins on the ADXL313 device. It should be called after the
 * device has been initialized and before enabling interrupts. The
 * `int_map` parameter specifies which interrupt sources are routed to
 * which pins. If the provided `int_map` value is invalid, the function
 * will handle it gracefully, ensuring that the device remains in a
 * stable state.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_map Union containing the interrupt mapping flags. The value should
 * be a valid configuration of interrupt sources. Invalid values
 * will be handled by the function.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_conf_int_map(struct adxl313_dev *dev,
			 union adxl313_int_map_reg_flags int_map);

/***************************************************************************//**
 * @brief This function configures which pin will be used for the DATA_READY
 * interrupt of the ADXL313 device. It should be called after
 * initializing the device and before enabling interrupts to ensure that
 * the correct pin is mapped for the DATA_READY signal. The `int_pin`
 * parameter specifies the pin to be used, and it is important to ensure
 * that the pin is valid for the device's configuration. If the function
 * encounters an error while reading the current interrupt mapping, it
 * will return an error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_pin The pin number to be configured for the DATA_READY interrupt.
 * Valid values depend on the specific hardware configuration and
 * should be checked against the device's documentation. If an
 * invalid pin is specified, the function may not behave as
 * expected.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_data_ready_int_map(struct adxl313_dev *dev, uint8_t int_pin);

/***************************************************************************//**
 * @brief This function is used to configure which pin will be used for the
 * activity interrupt signal of the ADXL313 device. It should be called
 * after the device has been initialized and before starting any
 * measurements. The `int_pin` parameter specifies the pin to which the
 * activity interrupt will be routed. If the specified pin is invalid,
 * the function will handle it gracefully, ensuring that the device
 * remains in a stable state.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_pin The pin number to which the activity interrupt will be mapped.
 * Valid values are typically defined by the hardware
 * specifications. The function will handle invalid pin numbers
 * appropriately.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_activity_int_map(struct adxl313_dev *dev, uint8_t int_pin);

/***************************************************************************//**
 * @brief This function is used to configure the pin for the inactivity
 * interrupt of the ADXL313 device. It should be called after the device
 * has been initialized and is ready for configuration. The `int_pin`
 * parameter specifies which pin will be used for the inactivity
 * interrupt. It is important to ensure that the specified pin is valid
 * and not already in use for another interrupt. If the function
 * encounters an error while reading the current interrupt mapping, it
 * will return an error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_pin The pin number to which the inactivity interrupt will be
 * mapped. Valid values are typically defined by the hardware
 * specifications. The function will handle invalid pin numbers
 * by returning an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_inactivity_int_map(struct adxl313_dev *dev, uint8_t int_pin);

/***************************************************************************//**
 * @brief This function is used to configure which pin will be used for the
 * watermark interrupt on the ADXL313 device. It should be called after
 * the device has been initialized and before any data collection begins.
 * The `int_pin` parameter specifies the pin to which the watermark
 * interrupt will be mapped. If the function encounters an error while
 * reading the current interrupt mapping, it will return an error code.
 * It is important to ensure that the `dev` parameter is valid and
 * properly initialized before calling this function.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should be properly initialized.
 * @param int_pin An 8-bit unsigned integer representing the pin to which the
 * watermark interrupt will be mapped. Valid values depend on the
 * specific hardware configuration, and invalid values may lead
 * to undefined behavior.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_watermark_int_map(struct adxl313_dev *dev, uint8_t int_pin);

/***************************************************************************//**
 * @brief This function is used to configure which pin will be used for the
 * overrun interrupt of the ADXL313 device. It should be called after the
 * device has been initialized and before any data collection or
 * interrupt handling is performed. The function reads the current
 * interrupt mapping settings, modifies the mapping for the overrun
 * interrupt, and then applies the updated settings. If the specified pin
 * is invalid, the function will handle it gracefully by returning an
 * error code.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_pin The pin number to which the overrun interrupt will be mapped.
 * Valid values are typically defined by the hardware
 * specifications. The function will return an error if an
 * invalid pin number is provided.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_overrun_int_map(struct adxl313_dev *dev, uint8_t int_pin);

/***************************************************************************//**
 * @brief This function retrieves the current interrupt source status from the
 * ADXL313 device. It should be called after the device has been
 * initialized and configured. The function reads the interrupt source
 * register and populates the provided `int_status_flags` union with the
 * current interrupt status flags. If the read operation is successful,
 * the flags will reflect the state of various interrupt sources, such as
 * data ready, activity, inactivity, watermark, and overrun. If the
 * device is not properly initialized or if there is a communication
 * error, the function will return an error code.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * This parameter must not be null and should point to a valid
 * initialized device instance.
 * @param int_status_flags A pointer to a `union adxl313_int_src_reg_flags`
 * where the interrupt status flags will be stored. This
 * parameter must not be null.
 * @return Returns 0 on success, indicating that the interrupt source register
 * was read successfully. A non-zero value indicates an error occurred
 * during the read operation.
 ******************************************************************************/
int adxl313_get_int_source_reg(struct adxl313_dev *dev,
			       union adxl313_int_src_reg_flags *int_status_flags);

/***************************************************************************//**
 * @brief This function is used to set the polarity of the interrupt signal for
 * the ADXL313 device. It should be called after the device has been
 * initialized and before enabling interrupts. The function accepts an
 * enumeration value that specifies whether the interrupt should be
 * active high or active low. If an invalid value is provided, the
 * function will handle it gracefully, ensuring that the device remains
 * in a safe state.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param int_pol An enumeration value of type `adxl313_int_pol` that specifies
 * the desired interrupt polarity. Valid values are
 * `ADXL313_INT_ACTIVE_HIGH` and `ADXL313_INT_ACTIVE_LOW`.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int adxl313_set_int_pol(struct adxl313_dev *dev, enum adxl313_int_pol int_pol);

/***************************************************************************//**
 * @brief This function should be called to enable full resolution mode for the
 * ADXL312 or ADXL313 accelerometer device. It is important to ensure
 * that the device type is not ADXL314, as this mode is not supported for
 * that device. If the device is of the correct type, the function will
 * attempt to write the necessary configuration to the device's
 * registers. If the operation is successful, it will also set the
 * device's range according to its current configuration. Calling this
 * function when the device is not properly initialized or if the device
 * type is incorrect will result in an error.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device
 * structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as if the device type is not supported.
 ******************************************************************************/
int adxl313_enable_full_res(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function is used to disable the full resolution mode of the
 * ADXL312 or ADXL313 device. It should be called when the device is
 * initialized and ready for configuration. If the device type is
 * `ADXL314`, the function will return an error without making any
 * changes. The function also updates the device's range setting after
 * disabling full resolution, ensuring that the device operates correctly
 * in its new configuration.
 *
 * @param dev Pointer to an `adxl313_dev` structure representing the device.
 * Must not be null. If the device type is `ADXL314`, the function
 * will return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_disable_full_res(struct adxl313_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves the full resolution setting of the ADXL312 or
 * ADXL313 device. It should be called after the device has been
 * initialized and configured. If the device type is `ADXL314`, the
 * function will return an error without modifying the output parameter.
 * The function reads the relevant register and updates the `full_res`
 * pointer with the full resolution setting, which indicates whether full
 * resolution is enabled or not.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @param full_res A pointer to a `uint8_t` where the full resolution setting
 * will be stored. Caller retains ownership and must ensure it
 * points to a valid memory location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as if the device type is `ADXL314`.
 ******************************************************************************/
int adxl313_get_full_res_setting(struct adxl313_dev *dev, uint8_t *full_res);

/***************************************************************************//**
 * @brief This function configures the measurement range of the ADXL313
 * accelerometer device. It should be called after the device has been
 * initialized and before starting measurements. The function checks the
 * device type and validates the specified range against the supported
 * ranges for that device type. If the specified range is invalid or if
 * the device type does not support range setting, an error code is
 * returned. The function also updates the device's internal state to
 * reflect the new range setting.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param range An enumeration value of type `adxl313_range` that specifies the
 * desired measurement range. Valid values depend on the device
 * type: for ADXL312, valid ranges are from 1.5G to 12G; for
 * ADXL313, valid ranges are up to 4G. If an invalid range is
 * specified, the function will return an error.
 * @return Returns 0 on success, or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int adxl313_set_range(struct adxl313_dev *dev, enum adxl313_range range);

/***************************************************************************//**
 * @brief This function is used to obtain the measurement range setting of the
 * ADXL313 device. It should be called after the device has been
 * initialized and configured. The function checks the device type and
 * reads the appropriate register to determine the range. If the device
 * type is `ID_ADXL314`, it directly sets the range to
 * `ADXL313_200G_RANGE`. For other device types, it reads the data format
 * register to extract the range value. The caller must ensure that the
 * `dev` pointer is valid and that the `range` pointer is not null.
 *
 * @param dev A pointer to the `adxl313_dev` structure representing the device.
 * Must not be null.
 * @param range A pointer to an `enum adxl313_range` where the function will
 * store the retrieved range value. Caller retains ownership and
 * must ensure it is not null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl313_get_range(struct adxl313_dev *dev, enum adxl313_range *range);

/* Perform device self-test. */
/***************************************************************************//**
 * @brief This function is used to execute a self-test on the ADXL313
 * accelerometer device. It must be called after the device has been
 * properly initialized and configured. The function sets the device to
 * standby mode, configures the output data rate, and enables full
 * resolution before performing the self-test. It reads samples from the
 * FIFO buffer both before and after enabling the self-test feature to
 * compute the deviation in output. The results of the self-test are
 * logged, indicating whether the test passed or failed based on
 * predefined deviation thresholds. It is important to ensure that the
 * device is in the correct operational state before invoking this
 * function.
 *
 * @param dev Pointer to the `adxl313_dev` structure representing the device.
 * Must not be null and should point to a valid initialized device.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adxl313_self_test(struct adxl313_dev *dev);

#endif /* __ADXL313_H__ */
