/***************************************************************************//**
 *   @file   adxl38x.h
 *   @brief  Header file of ADXL38X Driver.
 *   @author BRajendran (balarupini.rajendran@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef __ADXL38X_H__
#define __ADXL38X_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/******************************** ADXL38X *************************************/
/******************************************************************************/
/* Constants and Macros */

/* SPI */
#define ADXL38X_SPI_READ          		0x01
#define ADXL38X_SPI_WRITE         		0x00

/* Register Map fo ADXL38X (Rev H DS)*/
#define ADXL38X_DEVID_AD			0x00
#define ADXL38X_DEVID_MST			0x01
#define ADXL38X_PART_ID				0x02
#define ADXL38X_PART_ID_REV_ID			0x03
#define ADXL38X_SERIAL_NUMBER_0			0x04
#define ADXL38X_SERIAL_NUMBER_1			0x05
#define ADXL38X_SERIAL_NUMBER_2			0x06
#define ADXL38X_SERIAL_NUMBER_3			0x07
#define ADXL38X_SERIAL_NUMBER_4			0x08
#define ADXL38X_SERIAL_NUMBER_5			0x09
#define ADXL38X_SERIAL_NUMBER_6			0x0A
#define ADXL38X_DEV_DELTA_Q_X			0x0B
#define ADXL38X_DEV_DELTA_Q_Y			0x0C
#define ADXL38X_DEV_DELTA_Q_Z			0x0D
#define ADXL38X_DEV_DELTA_F0_X			0x0E
#define ADXL38X_DEV_DELTA_F0_Y			0x0F
#define ADXL38X_DEV_DELTA_F0_Z			0x10
#define ADXL38X_STATUS0				0x11
#define ADXL38X_STATUS1				0x12
#define ADXL38X_STATUS2				0x13
#define ADXL38X_STATUS3				0x14
#define ADXL38X_XDATA_H				0x15
#define ADXL38X_XDATA_L				0x16
#define ADXL38X_YDATA_H				0x17
#define ADXL38X_YDATA_L				0x18
#define ADXL38X_ZDATA_H				0x19
#define ADXL38X_ZDATA_L				0x1A
#define ADXL38X_TDATA_H				0x1B
#define ADXL38X_TDATA_L				0x1C
#define ADXL38X_FIFO_DATA			0x1D
#define ADXL38X_FIFO_STATUS0			0x1E
#define ADXL38X_FIFO_STATUS1			0x1F
#define ADXL38X_MISC0				0x20
#define ADXL38X_MISC1				0x21
#define ADXL38X_SENS_DSM			0x24
#define ADXL38X_CLK_CTRL			0x25
#define ADXL38X_OP_MODE				0x26
#define ADXL38X_DIG_EN				0x27
#define ADXL38X_SAR_I2C				0x28
#define ADXL38X_NVM_CTL				0x29
#define ADXL38X_REG_RESET			0x2A
#define ADXL38X_INT0_MAP0			0x2B
#define ADXL38X_INT0_MAP1			0x2C
#define ADXL38X_INT1_MAP0			0x2D
#define ADXL38X_INT1_MAP1			0x2E
#define ADXL38X_TEST_EN				0x2F
#define ADXL38X_FIFO_CFG0			0x30
#define ADXL38X_FIFO_CFG1			0x31
#define ADXL38X_SPT_CFG0			0x32
#define ADXL38X_SPT_CFG1			0x33
#define ADXL38X_SPT_CFG2			0x34
#define ADXL38X_SYNC_CFG			0x35
#define ADXL38X_PDM_CFG				0x36
#define ADXL38X_ACT_INACT_CTL			0x37
#define ADXL38X_SNSR_AXIS_EN			0x38
#define ADXL38X_THRESH_ACT_H			0x39
#define ADXL38X_THRESH_ACT_L			0x3A
#define ADXL38X_TIME_ACT_H			0x3B
#define ADXL38X_TIME_ACT_M			0x3C
#define ADXL38X_TIME_ACT_L			0x3D
#define ADXL38X_THRESH_INACT_H			0x3E
#define ADXL38X_THRESH_INACT_L			0x3F
#define ADXL38X_TIME_INACT_H			0x40
#define ADXL38X_TIME_INACT_M			0x41
#define ADXL38X_TIME_INACT_L			0x42
#define ADXL38X_TAP_THRESH			0x43
#define ADXL38X_TAP_DUR				0x44
#define ADXL38X_TAP_LATENT			0x45
#define ADXL38X_TAP_WINDOW			0x46
#define ADXL38X_TAP_CFG				0x47
#define ADXL38X_OR_CFG				0x48
#define ADXL38X_TRIG_CFG			0x49
#define ADXL38X_X_SAR_OFFSET			0x4A
#define ADXL38X_Y_SAR_OFFSET			0x4B
#define ADXL38X_Z_SAR_OFFSET			0x4C
#define ADXL38X_X_DSM_OFFSET			0x4D
#define ADXL38X_Y_DSM_OFFSET			0x4E
#define ADXL38X_Z_DSM_OFFSET			0x4F
#define ADXL38X_FILTER				0x50
#define ADXL38X_USER_TEMP_SENS_0		0x55
#define ADXL38X_USER_TEMP_SENS_1		0x56
#define ADXL38X_MISO				0x58
#define ADXL38X_SOUT0				0x59
#define ADXL38X_MCLK				0x5A
#define ADXL38X_BCLK				0x5B
#define ADXL38X_FSYNC				0x5C
#define ADXL38X_INT0				0x5D
#define ADXL38X_INT1				0x5E

/* Register reset value for ADXL38X */
#define ADXL38X_RESET_ZERO			0x00
#define ADXL38X_RESET_DEVID_AD 			0xAD
#define ADXL38X_RESET_DEVID_MST 		0x1D
#define ADXL38X_RESET_PART_ID 			0x17
#define ADXL38X_RESET_PART_ID_REV_ID 		0xC1
#define ADXL38X_RESET_STATUS0 			0x80
#define ADXL38X_RESET_STATUS2 			0x04
#define ADXL38X_RESET_INT0_MAP0 		0x80
#define ADXL38X_RESET_INT1_MAP1 		0x80
#define ADXL38X_RESET_SPT_CFG1 			0x08
#define ADXL38X_RESET_SPT_CFG2 			0x1A

/* Register masks */
#define ADXL38X_MASK_RANGE			0xC0
#define ADXL38X_MASK_OP_MODE			0x0F
#define ADXL38X_MASK_CHEN_DIG_EN		0xF0
#define ADXL38X_NEG_ACC_MSK			NO_OS_GENMASK(31, 16)
#define ADXL38X_SLF_TST_CTRL_MSK		0xE0
#define ADXL38X_FIFOCFG_FIFOMODE_MSK		0x30

/* Pre-defined codes */
#define ADXL38X_RESET_CODE            		0x52
#define ADXL38X_RESET_STATUS			0x80000400

/* Scale Factors */
/*
 * ADXL380
 * At +/- 4g with 16-bit resolution, scale from datasheet = 133.3ug/LSB.
 * 1333 * (10^-7)
 * For +/- 8g range a multiplier with value 2 is used.
 * For +/-16g range, a multiplier with value 4 is used.
 */
#define ADXL380_ACC_SCALE_FACTOR_GEE_MUL  (int64_t) 	1333
/*
 * ADXL382
 * At +/- 15g with 16-bit resolution, scale from datasheet = 500ug/LSB.
 * 5000 * (10^-7)
 * For +/- 30g range a multiplier with value 2 is used.
 * For +/-60g range, a multiplier with value 4 is used.
 */
#define ADXL382_ACC_SCALE_FACTOR_GEE_MUL  (int64_t) 	5000
/* Common denominator for ADXL380 and ADXL382 */
#define ADXL38X_ACC_SCALE_FACTOR_GEE_DIV  (int32_t)	10000000
/*
 * ADXL380/382 Temperature
 *
 * scale is 10.2 LSB/C.
 * Offset = 550LSB at 25C
 * offset at 0C = 550 - (10.2 * 25) LSB
 * offset at 0C = 295 LSB
 */
#define ADXL38X_TEMP_OFFSET (int32_t) 		295
#define ADXL38X_TEMP_SCALE_NUM (int32_t) 	102
#define ADXL38X_TEMP_SCALE_DEN (int32_t) 	10

/* Sensitivity */
/*
 * ADXL380
 * At +/- 4g with 16-bit resolution, sensitivity from datasheet = 7500LSB/g.
 * For +/- 8g range a divisor with value 2 is used.
 * For +/-16g range, a divisor with value 4 is used.
 */
#define ADXL380_ACC_SENSITIVITY (int32_t) 	7500
/*
 * ADXL382
 * At +/- 15g with 16-bit resolution, sensitivity from datasheet = 2000LSB/g.
 * For +/- 30g range a divisor with value 2 is used.
 * For +/-60g range, a divisor with value 4 is used.
 */
#define ADXL382_ACC_SENSITIVITY (int32_t) 	2000

/* Self Test Limits */
/* ADXL380 */
#define ADXL380_XY_ST_LIMIT_MIN	 	29
#define ADXL380_Z_ST_LIMIT_MIN 		27
#define ADXL380_XY_ST_LIMIT_MAX 	50
#define ADXL380_Z_ST_LIMIT_MAX 		32
/* ADXL382 */
#define ADXL382_XY_ST_LIMIT_MIN 	31
#define ADXL382_Z_ST_LIMIT_MIN 		26
#define ADXL382_XY_ST_LIMIT_MAX 	42
#define ADXL382_Z_ST_LIMIT_MAX 		34

#define ADXL38X_ST_LIMIT_DENOMINATOR 	10

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `adxl38x_id` enumeration defines identifiers for different
 * variants of the ADXL38X series accelerometers, specifically the
 * ADXL380 and ADXL382 models. This enumeration is used to distinguish
 * between these device types within the driver code, allowing for
 * device-specific configurations and operations.
 *
 * @param ID_ADXL380 Represents the identifier for the ADXL380 device.
 * @param ID_ADXL382 Represents the identifier for the ADXL382 device.
 ******************************************************************************/
enum adxl38x_id {
	ID_ADXL380,
	ID_ADXL382,
};

/***************************************************************************//**
 * @brief The `adxl38x_comm_type` is an enumeration that defines the
 * communication protocols available for the ADXL38X device, specifically
 * SPI and I2C. This enumeration is used to specify the type of
 * communication interface that the device will use to interact with
 * other components or systems, allowing for flexible integration
 * depending on the available hardware interfaces.
 *
 * @param ADXL38X_SPI_COMM Represents the SPI communication type for the ADXL38X
 * device.
 * @param ADXL38X_I2C_COMM Represents the I2C communication type for the ADXL38X
 * device.
 ******************************************************************************/
enum adxl38x_comm_type {
	ADXL38X_SPI_COMM,
	ADXL38X_I2C_COMM
};

/***************************************************************************//**
 * @brief The `adxl38x_range` enum defines the range settings for the ADXL38X
 * series accelerometers, specifically the ADXL380 and ADXL382 models.
 * Each range setting corresponds to a specific measurement range in
 * terms of gravitational force (g), allowing the device to measure
 * different levels of acceleration. The enum values are shared between
 * the two models, with each value representing a different range for
 * each model, such as ±4g for ADXL380 and ±15g for ADXL382, and so on.
 * This allows for flexible configuration of the accelerometer's
 * sensitivity based on the application requirements.
 *
 * @param ADXL380_RANGE_4G Represents a range of ±4g for the ADXL380 model.
 * @param ADXL382_RANGE_15G Represents a range of ±15g for the ADXL382 model.
 * @param ADXL380_RANGE_8G Represents a range of ±8g for the ADXL380 model.
 * @param ADXL382_RANGE_30G Represents a range of ±30g for the ADXL382 model.
 * @param ADXL380_RANGE_16G Represents a range of ±16g for the ADXL380 model.
 * @param ADXL382_RANGE_60G Represents a range of ±60g for the ADXL382 model.
 ******************************************************************************/
enum adxl38x_range {
	ADXL380_RANGE_4G = 0,
	ADXL382_RANGE_15G = 0,
	ADXL380_RANGE_8G = 1,
	ADXL382_RANGE_30G = 1,
	ADXL380_RANGE_16G = 2,
	ADXL382_RANGE_60G = 2
};

/***************************************************************************//**
 * @brief The `adxl38x_op_mode` enumeration defines various operating modes for
 * the ADXL38X accelerometer series, each represented by a unique integer
 * value. These modes include standby, heart sound, ultra-low power, very
 * low power, low power, reduced bandwidth, and high power, with
 * additional serial operation variants for some modes. This enumeration
 * is used to configure the device's power consumption and performance
 * characteristics based on the application's requirements.
 *
 * @param ADXL38X_MODE_STDBY Represents the standby mode with a value of 0.
 * @param ADXL38X_MODE_HRT_SND Represents the heart sound mode with a value of
 * 1.
 * @param ADXL38X_MODE_ULP Represents the ultra-low power mode with a value of
 * 2.
 * @param ADXL38X_MODE_VLP Represents the very low power mode with a value of 3.
 * @param ADXL38X_MODE_LP Represents the low power mode with a value of 4.
 * @param ADXL38X_MODE_LP_SERIAL_ULP_OP Represents the low power serial ultra-
 * low power operation mode with a value of
 * 6.
 * @param ADXL38X_MODE_LP_SERIAL_VLP_OP Represents the low power serial very low
 * power operation mode with a value of 7.
 * @param ADXL38X_MODE_RBW Represents the reduced bandwidth mode with a value of
 * 8.
 * @param ADXL38X_MODE_RBW_SERIAL_ULP_OP Represents the reduced bandwidth serial
 * ultra-low power operation mode with a
 * value of 10.
 * @param ADXL38X_MODE_RBW_SERIAL_VLP_OP Represents the reduced bandwidth serial
 * very low power operation mode with a
 * value of 11.
 * @param ADXL38X_MODE_HP Represents the high power mode with a value of 12.
 * @param ADXL38X_MODE_HP_SERIAL_ULP_OP Represents the high power serial ultra-
 * low power operation mode with a value of
 * 14.
 * @param ADXL38X_MODE_HP_SERIAL_VLP_OP Represents the high power serial very
 * low power operation mode with a value of
 * 15.
 ******************************************************************************/
enum adxl38x_op_mode {
	ADXL38X_MODE_STDBY = 0,
	ADXL38X_MODE_HRT_SND = 1,
	ADXL38X_MODE_ULP = 2,
	ADXL38X_MODE_VLP = 3,
	ADXL38X_MODE_LP = 4,
	ADXL38X_MODE_LP_SERIAL_ULP_OP = 6,
	ADXL38X_MODE_LP_SERIAL_VLP_OP = 7,
	ADXL38X_MODE_RBW = 8,
	ADXL38X_MODE_RBW_SERIAL_ULP_OP = 10,
	ADXL38X_MODE_RBW_SERIAL_VLP_OP = 11,
	ADXL38X_MODE_HP = 12,
	ADXL38X_MODE_HP_SERIAL_ULP_OP = 14,
	ADXL38X_MODE_HP_SERIAL_VLP_OP = 15,
};

/***************************************************************************//**
 * @brief The `adxl38x_ch_select` enumeration defines the possible channel
 * configurations for the ADXL38X accelerometer device. Each enumerator
 * represents a specific combination of enabled channels, including the
 * X, Y, Z axes, and the temperature sensor. This allows for flexible
 * configuration of the device to monitor specific axes or combinations
 * thereof, depending on the application requirements.
 *
 * @param ADXL38X_CH_DSB_ALL Disables all channels.
 * @param ADXL38X_CH_EN_X Enables the X channel.
 * @param ADXL38X_CH_EN_Y Enables the Y channel.
 * @param ADXL38X_CH_EN_XY Enables both X and Y channels.
 * @param ADXL38X_CH_EN_Z Enables the Z channel.
 * @param ADXL38X_CH_EN_YZ Enables both Y and Z channels.
 * @param ADXL38X_CH_EN_XYZ Enables X, Y, and Z channels.
 * @param ADXL38X_CH_EN_T Enables the temperature channel.
 * @param ADXL38X_CH_EN_ZT Enables both Z and temperature channels.
 * @param ADXL38X_CH_EN_YZT Enables Y, Z, and temperature channels.
 * @param ADXL38X_CH_EN_XYZT Enables X, Y, Z, and temperature channels.
 ******************************************************************************/
enum adxl38x_ch_select {
	ADXL38X_CH_DSB_ALL = 0,
	ADXL38X_CH_EN_X = 1,
	ADXL38X_CH_EN_Y = 2,
	ADXL38X_CH_EN_XY = 3,
	ADXL38X_CH_EN_Z = 4,
	ADXL38X_CH_EN_YZ = 6,
	ADXL38X_CH_EN_XYZ = 7,
	ADXL38X_CH_EN_T = 8,
	ADXL38X_CH_EN_ZT = 12,
	ADXL38X_CH_EN_YZT = 14,
	ADXL38X_CH_EN_XYZT = 15
};

/***************************************************************************//**
 * @brief The `adxl38x_fifo_mode` enumeration defines the different modes of
 * operation for the FIFO (First In, First Out) buffer in the ADXL38X
 * accelerometer series. These modes control how data is buffered and
 * output, allowing for different data handling strategies such as
 * disabling the FIFO, normal buffering, continuous streaming, or event-
 * triggered buffering.
 *
 * @param ADXL38X_FIFO_DISABLE Represents the disabled state of the FIFO mode.
 * @param ADXL38X_FIFO_NORMAL Represents the normal FIFO mode where data is
 * stored until full.
 * @param ADXL38X_FIFO_STREAM Represents the streaming FIFO mode where data is
 * continuously output.
 * @param ADXL38X_FIFO_TRIGGER Represents the trigger FIFO mode where data is
 * stored based on a trigger event.
 ******************************************************************************/
enum adxl38x_fifo_mode {
	ADXL38X_FIFO_DISABLE = 0,
	ADXL38X_FIFO_NORMAL = 1,
	ADXL38X_FIFO_STREAM = 2,
	ADXL38X_FIFO_TRIGGER = 3
};

/***************************************************************************//**
 * @brief The `adxl38x_fractional_val` structure is designed to represent a
 * value with both integer and fractional components, allowing for
 * precise representation of measurements such as acceleration or
 * temperature in the ADXL38X sensor family. The `integer` field holds
 * the whole number part of the value, while the `fractional` field
 * captures the fractional part, enabling high-resolution data
 * representation.
 *
 * @param integer Stores the integer part of the fractional value.
 * @param fractional Stores the fractional part of the value as a 32-bit
 * integer.
 ******************************************************************************/
struct adxl38x_fractional_val {
	int64_t integer;
	int32_t fractional;
};

/***************************************************************************//**
 * @brief The `adxl38x_comm_desc` is a union that provides a flexible
 * communication descriptor for the ADXL38X device, allowing it to
 * interface with either I2C or SPI communication protocols. This union
 * contains pointers to either an I2C descriptor or an SPI descriptor,
 * enabling the selection of the appropriate communication method based
 * on the device's configuration and requirements.
 *
 * @param i2c_desc Pointer to an I2C descriptor structure.
 * @param spi_desc Pointer to an SPI descriptor structure.
 ******************************************************************************/
union adxl38x_comm_desc {
	/** I2C Descriptor */
	struct no_os_i2c_desc *i2c_desc;
	/** SPI Descriptor */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `adxl38x_comm_init_param` is a union data structure designed to
 * initialize communication parameters for the ADXL38X device, supporting
 * both I2C and SPI communication protocols. It contains two members,
 * `i2c_init` and `spi_init`, which are structures that hold the
 * initialization parameters specific to I2C and SPI communication,
 * respectively. This union allows for flexible initialization of the
 * device depending on the chosen communication protocol.
 *
 * @param i2c_init I2C Initialization structure.
 * @param spi_init SPI Initialization structure.
 ******************************************************************************/
union adxl38x_comm_init_param {
	/** I2C Initialization structure. */
	struct no_os_i2c_init_param i2c_init;
	/** SPI Initialization structure. */
	struct no_os_spi_init_param spi_init;
} ;

/***************************************************************************//**
 * @brief The `adxl38x_dev` structure represents a device configuration for the
 * ADXL38X series accelerometers. It encapsulates essential parameters
 * such as the device type, communication protocol descriptor,
 * communication type, range, and operational mode. Additionally, it
 * includes a communication buffer to facilitate data exchange. This
 * structure is crucial for initializing and managing the device's
 * settings and operations, ensuring proper communication and
 * functionality according to the specified parameters.
 *
 * @param dev_type Specifies the type of ADXL38X device.
 * @param com_desc Holds the communication descriptor for selecting the
 * communication protocol.
 * @param comm_type Indicates the communication protocol type used by the
 * device.
 * @param range Defines the range setting of the device.
 * @param op_mode Specifies the operational mode of the device, such as
 * measurement or standby.
 * @param comm_buff A buffer used for communication, with a size of 320 bytes.
 ******************************************************************************/
struct adxl38x_dev {
	/** Device type*/
	enum adxl38x_id dev_type;
	/** Device communication descriptor (Selecting communication protocol descriptor) */
	union adxl38x_comm_desc com_desc;
	/** Device Communication type (Selecting protocol)*/
	enum adxl38x_comm_type comm_type;
	/** Range */
	enum adxl38x_range range;
	/** Modes - Measurement, Standby */
	enum adxl38x_op_mode op_mode;

	uint8_t comm_buff[320];
};

/***************************************************************************//**
 * @brief The `adxl38x_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up an ADXL38X device.
 * It includes a union for communication initialization, allowing for
 * flexibility between SPI and I2C protocols, and enumerations to specify
 * the communication type and the specific device model being used. This
 * structure is essential for configuring the device's communication
 * interface and identifying the device type during initialization.
 *
 * @param comm_init A union that holds the initialization parameters for either
 * SPI or I2C communication.
 * @param comm_type An enumeration that specifies the type of communication,
 * either SPI or I2C.
 * @param dev_type An enumeration that identifies the device type, either
 * ADXL380 or ADXL382.
 ******************************************************************************/
struct adxl38x_init_param {
	/** Device Communication initialization structure: either SPI or I2C */
	union adxl38x_comm_init_param comm_init;
	/** Device Communication type: ADXL38X_SPI_COMM, ADXL38X_I2C_COMM */
	enum adxl38x_comm_type comm_type;
	/** Device type: ADXL380 or 382 */
	enum adxl38x_id dev_type;
};

/***************************************************************************//**
 * @brief The `_adxl38x_sts_reg_flags` structure is a bit-field representation
 * of various status flags for the ADXL38X device, organized into four
 * status registers. Each field within the structure represents a
 * specific status or error condition, such as data readiness, tap
 * detection, memory errors, and FIFO buffer states. The structure uses
 * bit-fields to efficiently pack multiple status indicators into a
 * compact form, allowing for easy monitoring and handling of the
 * device's operational state.
 *
 * @param DATA_READY Indicates if new data is ready to be read.
 * @param RESERVED2 Reserved bits for future use or alignment.
 * @param UV_FLAG Indicates an under-voltage condition.
 * @param UV_FLAG_STICKY Indicates a persistent under-voltage condition.
 * @param RESERVED1 Reserved bit for future use or alignment.
 * @param NVM_CRC_ERR Indicates a CRC error in non-volatile memory.
 * @param NVM_CRC_DONE Indicates completion of a CRC check in non-volatile
 * memory.
 * @param NVM_ECC_DET Indicates detection of an ECC error in non-volatile
 * memory.
 * @param RESERVED Reserved bit for future use or alignment.
 * @param NVM_ECC_DONE Indicates completion of an ECC check in non-volatile
 * memory.
 * @param SINGLE_TAP Indicates detection of a single tap event.
 * @param DOUBLE_TAP Indicates detection of a double tap event.
 * @param TRIPLE_TAP Indicates detection of a triple tap event.
 * @param OVER_RANGE Indicates an over-range condition.
 * @param OVER_RANGE_STICKY Indicates a persistent over-range condition.
 * @param ACT Indicates an activity event.
 * @param INACT Indicates an inactivity event.
 * @param NVM_IRQ Indicates a non-volatile memory interrupt request.
 * @param PARITY_ERR_STICKY Indicates a persistent parity error.
 * @param FIFO_FULL Indicates that the FIFO buffer is full.
 * @param FIFO_OVR Indicates an overflow condition in the FIFO buffer.
 * @param FIFO_WATERMARK Indicates that the FIFO buffer has reached a predefined
 * watermark level.
 * @param FIFO_READY Indicates that the FIFO buffer is ready for data.
 * @param EFUSE_BUSY_REGERR_STICKY Indicates a persistent error in the EFUSE
 * busy register.
 * @param NVM_DONE Indicates completion of a non-volatile memory operation.
 * @param NVM_BUSY_STATUS Indicates that a non-volatile memory operation is in
 * progress.
 ******************************************************************************/
struct _adxl38x_sts_reg_flags {
	//Status 3
	uint8_t DATA_READY : 1;
	uint8_t RESERVED2 : 7;
	//Status 2
	uint8_t UV_FLAG : 1;
	uint8_t UV_FLAG_STICKY : 1;
	uint8_t RESERVED1 : 1;
	uint8_t NVM_CRC_ERR : 1;
	uint8_t NVM_CRC_DONE : 1;
	uint8_t NVM_ECC_DET : 1;
	uint8_t RESERVED : 1;
	uint8_t NVM_ECC_DONE : 1;
	//Status 1
	uint8_t SINGLE_TAP : 1;
	uint8_t DOUBLE_TAP : 1;
	uint8_t TRIPLE_TAP : 1;
	uint8_t OVER_RANGE : 1;
	uint8_t OVER_RANGE_STICKY : 1;
	uint8_t ACT : 1;
	uint8_t INACT : 1;
	uint8_t NVM_IRQ : 1;
	//Status 0
	uint8_t PARITY_ERR_STICKY : 1;
	uint8_t FIFO_FULL : 1;
	uint8_t FIFO_OVR : 1;
	uint8_t FIFO_WATERMARK : 1;
	uint8_t FIFO_READY : 1;
	uint8_t EFUSE_BUSY_REGERR_STICKY : 1;
	uint8_t NVM_DONE : 1;
	uint8_t NVM_BUSY_STATUS : 1;
};

/***************************************************************************//**
 * @brief The `adxl38x_sts_reg_flags` union is designed to encapsulate the
 * status register flags of the ADXL38X device, providing both a
 * structured view through the `fields` member and a raw 32-bit integer
 * view through the `value` member. This allows for flexible manipulation
 * and interpretation of the device's status, enabling both bitwise
 * operations and field-specific access to individual status flags.
 *
 * @param fields A structure containing individual status flags for the ADXL38X
 * device.
 * @param value A 32-bit unsigned integer representing the combined status flags
 * as a single value.
 ******************************************************************************/
union adxl38x_sts_reg_flags {
	struct _adxl38x_sts_reg_flags fields;
	uint32_t value;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function retrieves a block of data from the ADXL38X device,
 * starting at the specified base register address and reading a
 * specified number of bytes. It supports both SPI and I2C communication
 * protocols, determined by the device's communication type. The function
 * must be called with a properly initialized device structure. The read
 * data is stored in the provided buffer. If the communication fails, the
 * function returns an error code.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param base_address The starting register address from which data will be
 * read. Must be a valid register address for the device.
 * @param size The number of bytes to read from the device. Must be greater than
 * zero and should not exceed the buffer size.
 * @param read_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold the
 * requested data size.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl38x_read_device_data(struct adxl38x_dev *dev, uint8_t base_address,
			     uint16_t size, uint8_t *read_data);
/***************************************************************************//**
 * @brief This function is used to write a block of data to the ADXL38X device,
 * starting from a specified base address. It supports both SPI and I2C
 * communication protocols, determined by the device's communication
 * type. The function must be called with a valid device structure that
 * has been properly initialized. The size of the data to be written
 * should not exceed the buffer limits, and the write operation will be
 * performed according to the communication protocol specified in the
 * device structure. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param base_address The starting address in the device's register map where
 * data writing begins. Must be a valid register address.
 * @param size The number of bytes to write. Must be within the limits of the
 * device's buffer capacity.
 * @param write_data A pointer to the data buffer containing the bytes to be
 * written. Must not be null and should have at least 'size'
 * bytes available.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int adxl38x_write_device_data(struct adxl38x_dev *dev, uint8_t base_address,
			      uint16_t size, uint8_t *write_data);

/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * ADXL38X device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets them to the new
 * value provided. If the register being modified is the operating mode
 * register, the device is first set to standby mode to ensure a safe
 * transition. This function should be called when specific bits in a
 * register need to be updated without affecting other bits.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADXL38X device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param update_val The new values for the bits specified by the mask. Only the
 * bits corresponding to the mask will be updated.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int adxl38x_register_update_bits(struct adxl38x_dev *dev, uint8_t reg_addr,
				 uint8_t mask,
				 uint8_t update_val);

/***************************************************************************//**
 * @brief This function sets up an ADXL38X device for operation by allocating
 * necessary resources and configuring the device based on the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function checks the device type and
 * communication protocol, initializes the communication interface, and
 * verifies the device identity. If initialization fails at any step,
 * resources are cleaned up and an error code is returned. Successful
 * initialization results in a device structure being populated and
 * returned to the caller.
 *
 * @param device A pointer to a pointer of type `struct adxl38x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct adxl38x_init_param` containing
 * initialization parameters such as device type and
 * communication settings. The device type must be either
 * `ID_ADXL380` or `ID_ADXL382`. Invalid values result in an
 * error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error, such as memory allocation failure or
 * invalid parameters.
 ******************************************************************************/
int adxl38x_init(struct adxl38x_dev **device,
		 struct adxl38x_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to properly remove an ADXL38X device instance when
 * it is no longer needed. It handles the cleanup of communication
 * resources based on the communication type (SPI or I2C) and frees the
 * memory allocated for the device structure. This function should be
 * called to avoid memory leaks and ensure that the communication
 * resources are released correctly. Ensure that the device pointer is
 * valid and initialized before calling this function.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device to be removed. Must not be null. The function will handle
 * the cleanup of resources associated with this device.
 * @return Returns an integer status code indicating the success or failure of
 * the operation. A non-zero value indicates an error occurred during
 * the removal process.
 ******************************************************************************/
int adxl38x_remove(struct adxl38x_dev *dev);

/***************************************************************************//**
 * @brief This function is used to perform a soft reset on an ADXL38X device,
 * which is necessary to reset the device to its default state. It should
 * be called when the device needs to be reinitialized or when a known
 * state is required. The function requires a valid device structure and
 * will return an error code if the reset fails. It is important to
 * ensure that the device is properly initialized before calling this
 * function. After the reset, a delay is introduced to allow the device
 * to stabilize before further operations.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device to be reset. Must not be null. The function will return an
 * error if the device is not properly initialized.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int adxl38x_soft_reset(struct adxl38x_dev *dev);

int adxl38x_get_sts_reg(struct adxl38x_dev *dev,
			union adxl38x_sts_reg_flags *status_flags);

/***************************************************************************//**
 * @brief This function configures the ADXL38X device to operate in a specified
 * mode, which is essential for controlling the device's power
 * consumption and performance characteristics. It should be called after
 * the device has been initialized and before any data acquisition
 * operations. The function waits for 2 milliseconds to ensure the mode
 * change is settled. It returns an error code if the operation fails,
 * allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param op_mode An enum value of type adxl38x_op_mode representing the desired
 * operating mode. Valid modes are defined in the adxl38x_op_mode
 * enumeration.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation did not complete successfully.
 ******************************************************************************/
int adxl38x_set_op_mode(struct adxl38x_dev *dev, enum adxl38x_op_mode op_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current operating mode of an
 * ADXL38X device. It should be called when the user needs to verify or
 * log the device's mode of operation. The function requires a valid
 * device structure that has been properly initialized. It reads the
 * operating mode from the device and updates the provided pointer with
 * the current mode. If the operation is successful, the function returns
 * 0; otherwise, it returns a non-zero error code indicating the failure.
 *
 * @param dev A pointer to an initialized 'adxl38x_dev' structure representing
 * the device. Must not be null.
 * @param op_mode A pointer to an 'adxl38x_op_mode' variable where the current
 * operating mode will be stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int adxl38x_get_op_mode(struct adxl38x_dev *dev, enum adxl38x_op_mode *op_mode);

/***************************************************************************//**
 * @brief This function configures the measurement range of the ADXL38X
 * accelerometer device to a specified value. It should be called when
 * the user needs to change the sensitivity of the device to different
 * acceleration ranges, such as 4g, 8g, or 16g for ADXL380, and 15g, 30g,
 * or 60g for ADXL382. The function must be called with a valid device
 * structure and a valid range enumeration value. It updates the device's
 * internal range setting and returns a status code indicating success or
 * failure of the operation.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param range_val An enumeration value of type adxl38x_range specifying the
 * desired measurement range. Must be a valid range value for
 * the specific device model.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int adxl38x_set_range(struct adxl38x_dev *dev, enum adxl38x_range range_val);

/***************************************************************************//**
 * @brief This function is used to obtain the current range setting of an
 * ADXL38X device, which determines the measurement range of the
 * accelerometer. It should be called when the user needs to know the
 * current range configuration of the device. The function requires a
 * valid device structure and a pointer to an enum where the range value
 * will be stored. It returns an error code if the operation fails,
 * otherwise it updates the provided range value and the device's
 * internal range state.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null. The function will read from this device.
 * @param range_val A pointer to an enum adxl38x_range where the current range
 * setting will be stored. Must not be null. The function
 * writes the range value here.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl38x_get_range(struct adxl38x_dev *dev, enum adxl38x_range *range_val);

/***************************************************************************//**
 * @brief This function is used to obtain the device ID of an ADXL38X device,
 * which helps in identifying the specific model of the device being
 * used. It should be called when the device is properly initialized and
 * ready for communication. The function reads the device's internal
 * register to determine the device type and updates the provided device
 * ID variable accordingly. It returns an integer status code indicating
 * the success or failure of the operation.
 *
 * @param dev A pointer to an initialized 'adxl38x_dev' structure representing
 * the device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param devID A pointer to an 'adxl38x_id' enum where the device ID will be
 * stored. Must not be null, and the caller is responsible for
 * providing a valid memory location.
 * @return Returns an integer status code, where 0 indicates success and a non-
 * zero value indicates an error occurred during the operation.
 ******************************************************************************/
int adxl38x_get_deviceID(struct adxl38x_dev *dev,
			 enum adxl38x_id *devID);

/***************************************************************************//**
 * @brief This function is used to configure the self-test registers of an
 * ADXL38X device, allowing the user to enable or disable self-test mode,
 * force self-test, and set the self-test direction. It should be called
 * when the device is initialized and ready for configuration. The
 * function modifies the self-test control bits in the device's registers
 * based on the provided parameters. It is important to ensure that the
 * device pointer is valid and properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param st_mode A boolean value indicating whether to enable (true) or disable
 * (false) the self-test mode.
 * @param st_force A boolean value indicating whether to force the self-test
 * (true) or not (false).
 * @param st_dir A boolean value indicating the direction of the self-test. True
 * for one direction, false for the opposite.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int adxl38x_set_self_test_registers(struct adxl38x_dev *dev, bool st_mode,
				    bool st_force, bool st_dir);

/***************************************************************************//**
 * @brief This function is used to obtain the current status register flags from
 * an ADXL38X device. It should be called when you need to check the
 * status of the device, such as data readiness or error conditions. The
 * function requires a valid device structure that has been properly
 * initialized. It reads the status register and updates the provided
 * status_flags structure with the current status. Ensure that the device
 * is in a state where it can be read from, and handle any errors
 * returned by the function appropriately.
 *
 * @param dev A pointer to an adxl38x_dev structure representing the device.
 * This must be a valid, initialized device structure. The function
 * will not modify this structure.
 * @param status_flags A pointer to a union adxl38x_sts_reg_flags structure
 * where the status register flags will be stored. This must
 * not be null, and the caller is responsible for managing
 * the memory of this structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int adxl38x_get_sts_reg(struct adxl38x_dev *dev,
			union adxl38x_sts_reg_flags *status_flags);

/***************************************************************************//**
 * @brief This function is used to obtain the raw acceleration data from the
 * ADXL38X device for the X, Y, and Z axes. It should be called when the
 * device is properly initialized and ready for data acquisition. The
 * function ensures that the device is in the correct measurement mode
 * before reading the data. It is important to provide valid pointers for
 * the raw data outputs, as the function will store the retrieved data in
 * these locations. The function returns an error code if the operation
 * fails, which should be checked to ensure successful data retrieval.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param raw_x A pointer to a uint16_t where the raw X-axis data will be
 * stored. Must not be null.
 * @param raw_y A pointer to a uint16_t where the raw Y-axis data will be
 * stored. Must not be null.
 * @param raw_z A pointer to a uint16_t where the raw Z-axis data will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl38x_get_raw_xyz(struct adxl38x_dev *dev, uint16_t *raw_x,
			uint16_t *raw_y, uint16_t *raw_z);

/***************************************************************************//**
 * @brief This function reads the temperature data from the ADXL38X device and
 * converts it into a fractional value representing degrees Celsius. It
 * should be called when temperature data is needed from the device. The
 * function requires the device to be in a valid operational state and
 * will configure the device to enable the temperature channel if
 * necessary. It is important to ensure that the device has been properly
 * initialized before calling this function. The function will return an
 * error code if the operation fails at any step.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param temp_degC A pointer to an adxl38x_fractional_val structure where the
 * temperature in degrees Celsius will be stored. Must not be
 * null. The structure will be populated with the temperature
 * data.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl38x_get_temp(struct adxl38x_dev *dev,
		     struct adxl38x_fractional_val *raw_temp);

/***************************************************************************//**
 * @brief This function is used to obtain raw data from the specified channels
 * of an ADXL38X device, including accelerometer data for the X, Y, and Z
 * axes, as well as temperature data. It must be called with a properly
 * initialized device structure. The function enables the specified
 * channels, ensures the device is in measurement mode, and reads the
 * data into the provided pointers. If a pointer is null, the
 * corresponding data is not retrieved. The function returns an error
 * code if the operation fails, which can occur if the device is not
 * properly configured or if communication errors occur.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param channels An enum value of type adxl38x_ch_select indicating which
 * channels to read. Valid values are defined in the
 * adxl38x_ch_select enum.
 * @param raw_x A pointer to a uint16_t where the raw X-axis data will be
 * stored. Can be null if X data is not needed.
 * @param raw_y A pointer to a uint16_t where the raw Y-axis data will be
 * stored. Can be null if Y data is not needed.
 * @param raw_z A pointer to a uint16_t where the raw Z-axis data will be
 * stored. Can be null if Z data is not needed.
 * @param raw_temp A pointer to a uint16_t where the raw temperature data will
 * be stored. Can be null if temperature data is not needed.
 * @return Returns an integer error code, with 0 indicating success and non-zero
 * indicating an error.
 ******************************************************************************/
int adxl38x_get_raw_data(struct adxl38x_dev *dev,
			 enum adxl38x_ch_select channels,
			 uint16_t *raw_x, uint16_t *raw_y,
			 uint16_t *raw_z, uint16_t *raw_temp);

/***************************************************************************//**
 * @brief Use this function to obtain the acceleration data from the ADXL38X
 * device in g-force units for the specified channels. This function
 * should be called after the device has been properly initialized and
 * configured. It reads the raw acceleration data, converts it to
 * g-force, and stores the results in the provided structures for the x,
 * y, and z axes. Ensure that the device is in a suitable operational
 * mode and that the channels parameter correctly specifies the desired
 * axes. The function returns an error code if the data retrieval fails.
 *
 * @param dev Pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param channels Specifies which channels (axes) to read data from, using the
 * adxl38x_ch_select enumeration. Must be a valid channel
 * selection.
 * @param x Pointer to an adxl38x_fractional_val structure where the x-axis data
 * will be stored. Must not be null.
 * @param y Pointer to an adxl38x_fractional_val structure where the y-axis data
 * will be stored. Must not be null.
 * @param z Pointer to an adxl38x_fractional_val structure where the z-axis data
 * will be stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int adxl38x_get_xyz_gees(struct adxl38x_dev *dev,
			 enum adxl38x_ch_select channels,
			 struct adxl38x_fractional_val *x, struct adxl38x_fractional_val *y,
			 struct adxl38x_fractional_val *z);

/***************************************************************************//**
 * @brief This function is used to perform a self-test on the ADXL38X
 * accelerometer to verify its functionality. It should be called when
 * the device is in a known operational state, typically after
 * initialization. The function tests the X, Y, and Z axes and returns
 * the results through the provided boolean pointers. It requires the
 * device to be in standby mode initially and will configure the device
 * to the specified operational mode for testing. The function will
 * return an error code if any step in the process fails, and the results
 * for each axis are set to false by default.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param op_mode The operational mode to set the device to during the self-
 * test. Must be a valid adxl38x_op_mode value.
 * @param st_x A pointer to a boolean where the result of the X-axis self-test
 * will be stored. Must not be null.
 * @param st_y A pointer to a boolean where the result of the Y-axis self-test
 * will be stored. Must not be null.
 * @param st_z A pointer to a boolean where the result of the Z-axis self-test
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure. The
 * boolean pointers st_x, st_y, and st_z are updated to indicate the
 * success of the self-test for each axis.
 ******************************************************************************/
int adxl38x_selftest(struct adxl38x_dev *dev, enum adxl38x_op_mode op_mode,
		     bool *st_x, bool *st_y, bool *st_z);

/***************************************************************************//**
 * @brief This function sets up the FIFO (First In, First Out) buffer for the
 * ADXL38X accelerometer device, allowing for the configuration of the
 * number of samples, trigger mode, FIFO mode, channel ID, and read reset
 * options. It should be called after the device has been properly
 * initialized and configured. The function checks for valid sample
 * numbers and channel configurations, returning an error if the
 * conditions are not met. It is important to ensure that the device is
 * in a suitable state for configuration changes before calling this
 * function.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param num_samples The number of samples to store in the FIFO, ranging from 0
 * to 320. Values outside this range or incompatible with the
 * current channel configuration will result in an error.
 * @param external_trigger A boolean indicating whether an external trigger is
 * used. If true, the FIFO will be configured to use an
 * external trigger if the mode is set to
 * ADXL38X_FIFO_TRIGGER.
 * @param fifo_mode An enum value of type adxl38x_fifo_mode specifying the FIFO
 * mode. Must be a valid mode defined in the adxl38x_fifo_mode
 * enumeration.
 * @param ch_ID_enable A boolean indicating whether channel ID is enabled. If
 * true, the channel ID will be included in the FIFO
 * configuration.
 * @param read_reset A boolean indicating whether to reset the read pointer
 * after reading. If true, the read pointer will be reset.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int adxl38x_accel_set_FIFO(struct adxl38x_dev *dev, uint16_t numSamples,
			   bool externalTrigger, enum adxl38x_fifo_mode fifo_mode,
			   bool chIDEnable, bool readReset);

/***************************************************************************//**
 * @brief This function is used to convert raw accelerometer data from an
 * ADXL38X device into a fractional representation of gravitational units
 * (gees). It should be called when you have raw accelerometer data that
 * needs to be interpreted in terms of gravitational force. The function
 * requires a valid device structure and expects the raw data to be in a
 * specific format. The result is stored in a provided structure that
 * separates the integer and fractional parts of the gravitational value.
 * Ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized adxl38x_dev structure representing the
 * device. Must not be null.
 * @param raw_accel_data A pointer to a buffer containing raw accelerometer
 * data. Must not be null and should be properly aligned.
 * @param data_frac A pointer to an adxl38x_fractional_val structure where the
 * converted gravitational data will be stored. Must not be
 * null.
 * @return Returns 0 on success. The data_frac structure is populated with the
 * converted gravitational data.
 ******************************************************************************/
int adxl38x_data_raw_to_gees(struct adxl38x_dev *dev, uint8_t *raw_accel_data,
			     struct adxl38x_fractional_val *data_frac);

#endif /* __ADXL38X_H__ */

