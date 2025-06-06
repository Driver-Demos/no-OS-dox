/**************************************************************************//**
*   @file   ad3552r.h
*   @brief  Header file of ad3552r Driver
*   @author Mihail Chindris (Mihail.Chindris@analog.com)
*
*******************************************************************************
* Copyright 2021(c) Analog Devices, Inc.
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
******************************************************************************/

#ifndef _AD3552R_H_
#define _AD3552R_H_

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_crc8.h"

/*****************************************************************************/
/******************** Macros and Constants Definitions ***********************/
/*****************************************************************************/

/* Register addresses */
/* Primary address space */
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_A		0x00
#define   AD3552R_MASK_SOFTWARE_RESET			(NO_OS_BIT(7) | NO_OS_BIT(0))
#define   AD3552R_MASK_ADDR_ASCENSION			NO_OS_BIT(5)
#define   AD3552R_MASK_SDO_ACTIVE			NO_OS_BIT(4)
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_B		0x01
#define   AD3552R_MASK_SINGLE_INST			NO_OS_BIT(7)
#define   AD3552R_MASK_SHORT_INSTRUCTION		NO_OS_BIT(3)
#define AD3552R_REG_ADDR_DEVICE_CONFIG			0x02
#define   AD3552R_MASK_DEVICE_STATUS(n)			NO_OS_BIT(4 + (n))
#define   AD3552R_MASK_CUSTOM_MODES			(NO_OS_BIT(3) | NO_OS_BIT(2))
#define   AD3552R_MASK_OPERATING_MODES			NO_OS_GENMASK(1, 0)
#define AD3552R_REG_ADDR_CHIP_TYPE			0x03
#define   AD3552R_MASK_CLASS				NO_OS_GENMASK(7, 0)
#define AD3552R_REG_ADDR_PRODUCT_ID_L			0x04
#define AD3552R_REG_ADDR_PRODUCT_ID_H			0x05
#define AD3552R_REG_ADDR_CHIP_GRADE			0x06
#define   AD3552R_MASK_GRADE				NO_OS_GENMASK(7, 4)
#define   AD3552R_MASK_DEVICE_REVISION			NO_OS_GENMASK(3, 0)
#define AD3552R_REG_ADDR_SCRATCH_PAD			0x0A
#define AD3552R_REG_ADDR_SPI_REVISION			0x0B
#define AD3552R_REG_ADDR_VENDOR_L			0x0C
#define AD3552R_REG_ADDR_VENDOR_H			0x0D
#define AD3552R_REG_ADDR_STREAM_MODE			0x0E
#define   AD3552R_MASK_LENGTH				0xFF
#define AD3552R_REG_ADDR_TRANSFER_REGISTER		0x0F
#define   AD3552R_MASK_MULTI_IO_MODE			(NO_OS_BIT(7) | NO_OS_BIT(6))
#define   AD3552R_MASK_STREAM_LENGTH_KEEP_VALUE		NO_OS_BIT(2)
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_C		0x10
#define   AD3552R_MASK_CRC_ENABLE			(NO_OS_BIT(7) | NO_OS_BIT(6) | NO_OS_BIT(1) | NO_OS_BIT(0))
#define   AD3552R_MASK_STRICT_REGISTER_ACCESS		NO_OS_BIT(5)
#define AD3552R_REG_ADDR_INTERFACE_STATUS_A		0x11
#define   AD3552R_MASK_INTERFACE_NOT_READY		NO_OS_BIT(7)
#define   AD3552R_MASK_CLOCK_COUNTING_ERROR		NO_OS_BIT(5)
#define   AD3552R_MASK_INVALID_OR_NO_CRC		NO_OS_BIT(3)
#define   AD3552R_MASK_WRITE_TO_READ_ONLY_REGISTER	NO_OS_BIT(2)
#define   AD3552R_MASK_PARTIAL_REGISTER_ACCESS		NO_OS_BIT(1)
#define   AD3552R_MASK_REGISTER_ADDRESS_INVALID		NO_OS_BIT(0)
#define AD3552R_REG_ADDR_INTERFACE_CONFIG_D		0x14
#define   AD3552R_MASK_ALERT_ENABLE_PULLUP		NO_OS_BIT(6)
#define   AD3552R_MASK_MEM_CRC_EN			NO_OS_BIT(4)
#define   AD3552R_MASK_SDO_DRIVE_STRENGTH		(NO_OS_BIT(3) | NO_OS_BIT(2))
#define   AD3552R_MASK_DUAL_SPI_SYNCHROUNOUS_EN		NO_OS_BIT(1)
#define   AD3552R_MASK_SPI_CONFIG_DDR			NO_OS_BIT(0)
#define AD3552R_REG_ADDR_SH_REFERENCE_CONFIG		0x15
#define   AD3552R_MASK_IDUMP_FAST_MODE			NO_OS_BIT(6)
#define   AD3552R_MASK_SAMPLE_HOLD_DIFFERENTIAL_USER_EN	NO_OS_BIT(5)
#define   AD3552R_MASK_SAMPLE_HOLD_USER_TRIM		(NO_OS_BIT(4) | NO_OS_BIT(3))
#define   AD3552R_MASK_SAMPLE_HOLD_USER_ENABLE		NO_OS_BIT(2)
#define   AD3552R_MASK_REFERENCE_VOLTAGE_SEL		(NO_OS_BIT(1) | NO_OS_BIT(0))
#define AD3552R_REG_ADDR_ERR_ALARM_MASK			0x16
#define   AD3552R_MASK_REF_RANGE_ALARM			NO_OS_BIT(6)
#define   AD3552R_MASK_CLOCK_COUNT_ERR_ALARM		NO_OS_BIT(5)
#define   AD3552R_MASK_MEM_CRC_ERR_ALARM		NO_OS_BIT(4)
#define   AD3552R_MASK_SPI_CRC_ERR_ALARM		NO_OS_BIT(3)
#define   AD3552R_MASK_WRITE_TO_READ_ONLY_ALARM		NO_OS_BIT(2)
#define   AD3552R_MASK_PARTIAL_REGISTER_ACCESS_ALARM	NO_OS_BIT(1)
#define   AD3552R_MASK_REGISTER_ADDRESS_INVALID_ALARM	NO_OS_BIT(0)
#define AD3552R_REG_ADDR_ERR_STATUS			0x17
#define   AD3552R_MASK_REF_RANGE_ERR_STATUS			NO_OS_BIT(6)
#define   AD3552R_MASK_DUAL_SPI_STREAM_EXCEEDS_DAC_ERR_STATUS	NO_OS_BIT(5)
#define   AD3552R_MASK_MEM_CRC_ERR_STATUS			NO_OS_BIT(4)
#define   AD3552R_MASK_RESET_STATUS				NO_OS_BIT(0)
#define AD3552R_REG_ADDR_POWERDOWN_CONFIG		0x18
#define   AD3552R_MASK_CH_DAC_POWERDOWN(ch)		NO_OS_BIT(4 + (ch))
#define   AD3552R_MASK_CH_AMPLIFIER_POWERDOWN(ch)	NO_OS_BIT(ch)
#define AD3552R_REG_ADDR_CH0_CH1_OUTPUT_RANGE		0x19
#define   AD3552R_MASK_CH_OUTPUT_RANGE_SEL(ch)		((ch) ? 0xF0 : 0xF)
#define AD3552R_REG_ADDR_CH_OFFSET(ch)			(0x1B + (ch) * 2)
#define   AD3552R_MASK_CH_OFFSET_BITS_0_7		0xFF
#define AD3552R_REG_ADDR_CH_GAIN(ch)			(0x1C + (ch) * 2)
#define   AD3552R_MASK_CH_RANGE_OVERRIDE		NO_OS_BIT(7)
#define   AD3552R_MASK_CH_GAIN_SCALING_N		(NO_OS_BIT(6) | NO_OS_BIT(5))
#define   AD3552R_MASK_CH_GAIN_SCALING_P		(NO_OS_BIT(4) | NO_OS_BIT(3))
#define   AD3552R_MASK_CH_OFFSET_POLARITY		NO_OS_BIT(2)
#define   AD3552R_MASK_CH_OFFSET_BIT_8			NO_OS_BIT(0)

/*
 * Secondary region
 * For multibyte registers specify the highest address because the access is
 * done in descending order
 */
#define AD3552R_SECONDARY_REGION_START			0x28
#define AD3552R_REG_ADDR_HW_LDAC_16B			0x28
#define AD3552R_REG_ADDR_CH_DAC_16B(ch)			(0x2C - (1 - ch) * 2)
#define AD3552R_REG_ADDR_DAC_PAGE_MASK_16B		0x2E
#define AD3552R_REG_ADDR_CH_SELECT_16B			0x2F
#define AD3552R_REG_ADDR_INPUT_PAGE_MASK_16B		0x31
#define AD3552R_REG_ADDR_SW_LDAC_16B			0x32
#define AD3552R_REG_ADDR_CH_INPUT_16B(ch)		(0x36 - (1 - ch) * 2)
/* 3 bytes registers */
#define AD3552R_REG_START_24B				0x37
#define AD3552R_REG_ADDR_HW_LDAC_24B			0x37
#define AD3552R_REG_ADDR_CH_DAC_24B(ch)			(0x3D - (1 - ch) * 3)
#define AD3552R_REG_ADDR_DAC_PAGE_MASK_24B		0x40
#define AD3552R_REG_ADDR_CH_SELECT_24B			0x41
#define AD3552R_REG_ADDR_INPUT_PAGE_MASK_24B		0x44
#define AD3552R_REG_ADDR_SW_LDAC_24B			0x45
#define AD3552R_REG_ADDR_CH_INPUT_24B(ch)		(0x4B - (1 - ch) * 3)

#define AD3552R_REG_ADDR_MAX		0x4B

/* Useful defines */
#define AD3552R_MASK_CH(ch)				NO_OS_BIT(ch)
#define AD3552R_MASK_ALL_CH				(NO_OS_BIT(0) | NO_OS_BIT(1))
#define AD3552R_MASK_DAC_12B				0xFFF0
#define AD3552R_REAL_BITS_PREC_MODE			16
#define AD3552R_STORAGE_BITS_PREC_MODE			24
#define AD3552R_REAL_BITS_FAST_MODE			12
#define AD3552R_STORAGE_BITS_FAST_MODE			16
#define AD3552R_MAX_OFFSET				511
#define AD3552R_LDAC_PULSE_US				1
#define AD3552R_BOTH_CH_SELECT			(NO_OS_BIT(0) | NO_OS_BIT(1))
#define AD3552R_BOTH_CH_DESELECT		0x0

/* Maximum number of channels in this family of devices */
#define AD3552R_MAX_NUM_CH		2

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad3552r_id` enumeration defines a set of constants that represent
 * different device identifiers for the AD3541R, AD3542R, AD3551R, and
 * AD3552R devices. These identifiers are used to distinguish between
 * different models within the AD3552R family of devices, allowing for
 * specific handling and configuration based on the device type.
 *
 * @param AD3541R_ID Represents the identifier for the AD3541R device.
 * @param AD3542R_ID Represents the identifier for the AD3542R device.
 * @param AD3551R_ID Represents the identifier for the AD3551R device.
 * @param AD3552R_ID Represents the identifier for the AD3552R device.
 ******************************************************************************/
enum ad3552r_id {
	AD3541R_ID,
	AD3542R_ID,
	AD3551R_ID,
	AD3552R_ID
};

/***************************************************************************//**
 * @brief The `ad3552r_io_mode` enumeration defines the different SPI
 * communication modes available for the AD3552R device. It includes
 * standard SPI, dual SPI, and quad SPI modes, allowing for varying
 * levels of data throughput and communication complexity. This
 * enumeration is used to configure the device's communication interface
 * according to the desired SPI mode.
 *
 * @param AD3552R_SPI Represents the standard SPI mode.
 * @param AD3552R_DUAL_SPI Represents the dual SPI mode.
 * @param AD3552R_QUAD_SPI Represents the quad SPI mode.
 ******************************************************************************/
enum ad3552r_io_mode {
	AD3552R_SPI,
	AD3552R_DUAL_SPI,
	AD3552R_QUAD_SPI,
};

/***************************************************************************//**
 * @brief The `ad3552r_ch_vref_select` enumeration defines the possible
 * configurations for selecting the voltage reference source for the
 * AD3552R device. It provides options for using either an internal
 * voltage reference with different pin configurations or an external
 * voltage reference. This allows for flexibility in how the device's
 * reference voltage is managed, depending on the specific application
 * requirements.
 *
 * @param AD3552R_INTERNAL_VREF_PIN_FLOATING Represents an internal voltage
 * reference source with the Vref I/O
 * pin floating.
 * @param AD3552R_INTERNAL_VREF_PIN_2P5V Represents an internal voltage
 * reference source with the Vref I/O pin
 * set to 2.5V.
 * @param AD3552R_EXTERNAL_VREF_PIN_INPUT Represents an external voltage
 * reference source with the Vref I/O pin
 * used as input.
 ******************************************************************************/
enum ad3552r_ch_vref_select {
	/* Internal source with Vref I/O floating */
	AD3552R_INTERNAL_VREF_PIN_FLOATING,
	/* Internal source with Vref I/O at 2.5V */
	AD3552R_INTERNAL_VREF_PIN_2P5V,
	/* External source with Vref I/O as input */
	AD3552R_EXTERNAL_VREF_PIN_INPUT
};

/***************************************************************************//**
 * @brief The `ad3552r_status` is an enumeration that defines various status and
 * error codes for the AD3552R device. It includes status bits such as
 * reset status and interface readiness, as well as error codes for
 * issues like clock counting errors, invalid CRC, and attempts to write
 * to read-only registers. Each enumerator is associated with a specific
 * bitmask value, allowing for bitwise operations to check or set
 * specific status or error conditions.
 *
 * @param AD3552R_RESET_STATUS Indicates a reset status with a value of 0x0001.
 * @param AD3552R_INTERFACE_NOT_READY Indicates the interface is not ready with
 * a value of 0x0002.
 * @param AD3552R_CLOCK_COUNTING_ERROR Represents a clock counting error with a
 * value of 0x0004.
 * @param AD3552R_INVALID_OR_NO_CRC Indicates an invalid or missing CRC with a
 * value of 0x0008.
 * @param AD3552R_WRITE_TO_READ_ONLY_REGISTER Indicates an attempt to write to a
 * read-only register with a value of
 * 0x0010.
 * @param AD3552R_PARTIAL_REGISTER_ACCESS Indicates a partial register access
 * with a value of 0x0020.
 * @param AD3552R_REGISTER_ADDRESS_INVALID Indicates an invalid register address
 * with a value of 0x0040.
 * @param AD3552R_REF_RANGE_ERR_STATUS Indicates a reference range error status
 * with a value of 0x0080.
 * @param AD3552R_DUAL_SPI_STREAM_EXCEEDS_DAC_ERR_STATUS Indicates a dual SPI
 * stream exceeds DAC
 * error status with a
 * value of 0x0100.
 * @param AD3552R_MEM_CRC_ERR_STATUS Indicates a memory CRC error status with a
 * value of 0x0200.
 ******************************************************************************/
enum ad3552r_status {
	/* Status bits */
	AD3552R_RESET_STATUS				= 0x0001,
	AD3552R_INTERFACE_NOT_READY			= 0x0002,

	/* Errors */
	AD3552R_CLOCK_COUNTING_ERROR			= 0x0004,
	AD3552R_INVALID_OR_NO_CRC			= 0x0008,
	AD3552R_WRITE_TO_READ_ONLY_REGISTER		= 0x0010,
	AD3552R_PARTIAL_REGISTER_ACCESS			= 0x0020,
	AD3552R_REGISTER_ADDRESS_INVALID		= 0x0040,
	AD3552R_REF_RANGE_ERR_STATUS			= 0x0080,
	AD3552R_DUAL_SPI_STREAM_EXCEEDS_DAC_ERR_STATUS	= 0x0100,
	AD3552R_MEM_CRC_ERR_STATUS			= 0x0200
};

/***************************************************************************//**
 * @brief The `ad3552r_ch_output_range` enumeration defines various output
 * voltage ranges for a channel in the AD3552R device, each associated
 * with specific feedback resistor connections (Rfb). These ranges allow
 * the device to output voltages from a minimum of 0 V to a maximum of 10
 * V, or from negative voltages up to positive voltages, depending on the
 * selected range. The enumeration is used to configure the output range
 * of the device's channels, ensuring compatibility with different
 * application requirements.
 *
 * @param AD3552R_CH_OUTPUT_RANGE_0__2P5V Represents an output range from 0 V to
 * 2.5 V, requiring an Rfb1x connection.
 * @param AD3552R_CH_OUTPUT_RANGE_0__5V Represents an output range from 0 V to 5
 * V, requiring an Rfb1x connection.
 * @param AD3552R_CH_OUTPUT_RANGE_0__10V Represents an output range from 0 V to
 * 10 V, requiring an Rfb2x connection.
 * @param AD3552R_CH_OUTPUT_RANGE_NEG_5__5V Represents an output range from -5 V
 * to 5 V, requiring an Rfb2x
 * connection.
 * @param AD3552R_CH_OUTPUT_RANGE_NEG_10__10V Represents an output range from
 * -10 V to 10 V, requiring an Rfb4x
 * connection.
 ******************************************************************************/
enum ad3552r_ch_output_range {
	/* Range from 0 V to 2.5 V. Requires Rfb1x connection */
	AD3552R_CH_OUTPUT_RANGE_0__2P5V,
	/* Range from 0 V to 5 V. Requires Rfb1x connection  */
	AD3552R_CH_OUTPUT_RANGE_0__5V,
	/* Range from 0 V to 10 V. Requires Rfb2x connection  */
	AD3552R_CH_OUTPUT_RANGE_0__10V,
	/* Range from -5 V to 5 V. Requires Rfb2x connection  */
	AD3552R_CH_OUTPUT_RANGE_NEG_5__5V,
	/* Range from -10 V to 10 V. Requires Rfb4x connection  */
	AD3552R_CH_OUTPUT_RANGE_NEG_10__10V,
};

/***************************************************************************//**
 * @brief The `ad3542r_ch_output_range` is an enumeration that defines various
 * output voltage ranges for a channel in the AD3542R device. Each
 * enumerated value specifies a distinct voltage range that the channel
 * can output, along with the required feedback resistor connection
 * (Rfb1x or Rfb2x) to achieve that range. This enumeration is used to
 * configure the output range of the device's channels, allowing for
 * flexibility in voltage output based on application needs.
 *
 * @param AD3542R_CH_OUTPUT_RANGE_0__2P5V Represents an output range from 0 V to
 * 2.5 V, requiring an Rfb1x connection.
 * @param AD3542R_CH_OUTPUT_RANGE_0__5V Represents an output range from 0 V to 5
 * V, requiring an Rfb1x connection.
 * @param AD3542R_CH_OUTPUT_RANGE_0__10V Represents an output range from 0 V to
 * 10 V, requiring an Rfb2x connection.
 * @param AD3542R_CH_OUTPUT_RANGE_NEG_5__5V Represents an output range from -5 V
 * to 5 V, requiring an Rfb2x
 * connection.
 * @param AD3542R_CH_OUTPUT_RANGE_NEG_2P5__7P5V Represents an output range from
 * -2.5 V to 7.5 V, requiring an
 * Rfb2x connection.
 ******************************************************************************/
enum ad3542r_ch_output_range {
	/* Range from 0 V to 2.5 V. Requires Rfb1x connection */
	AD3542R_CH_OUTPUT_RANGE_0__2P5V,
	/* Range from 0 V to 5 V. Requires Rfb1x connection  */
	AD3542R_CH_OUTPUT_RANGE_0__5V,
	/* Range from 0 V to 10 V. Requires Rfb2x connection  */
	AD3542R_CH_OUTPUT_RANGE_0__10V,
	/* Range from -5 V to 5 V. Requires Rfb2x connection  */
	AD3542R_CH_OUTPUT_RANGE_NEG_5__5V,
	/* Range from -2.5 V to 7.5 V. Requires Rfb2x connection  */
	AD3542R_CH_OUTPUT_RANGE_NEG_2P5__7P5V,
};

/***************************************************************************//**
 * @brief The `ad3552r_sdio_drive_strength` enumeration defines different levels
 * of drive strength for the SDIO (Serial Data Input/Output) interface in
 * the AD3552R device. This allows the user to configure the electrical
 * drive capability of the SDIO lines, which can be important for
 * ensuring signal integrity and performance in different system
 * configurations and load conditions.
 *
 * @param AD3552R_LOW_SDIO_DRIVE_STRENGTH Represents a low drive strength
 * setting for the SDIO interface.
 * @param AD3552R_MEDIUM_LOW_SDIO_DRIVE_STRENGTH Represents a medium-low drive
 * strength setting for the SDIO
 * interface.
 * @param AD3552R_MEDIUM_HIGH_SDIO_DRIVE_STRENGTH Represents a medium-high drive
 * strength setting for the SDIO
 * interface.
 * @param AD3552R_HIGH_SDIO_DRIVE_STRENGTH Represents a high drive strength
 * setting for the SDIO interface.
 ******************************************************************************/
enum ad3552r_sdio_drive_strength {
	AD3552R_LOW_SDIO_DRIVE_STRENGTH,
	AD3552R_MEDIUM_LOW_SDIO_DRIVE_STRENGTH,
	AD3552R_MEDIUM_HIGH_SDIO_DRIVE_STRENGTH,
	AD3552R_HIGH_SDIO_DRIVE_STRENGTH
};

/***************************************************************************//**
 * @brief The `num_channels` enumeration defines the number of channels
 * available for different models of the AD35xx series devices. Each
 * enumerator corresponds to a specific device model and indicates the
 * number of channels that the device supports, which is either 1 or 2.
 * This enumeration is useful for configuring and managing device-
 * specific operations based on the number of channels available.
 *
 * @param AD3541R_NUM_CHANNELS Represents the number of channels for the AD3541R
 * device, which is 1.
 * @param AD3542R_NUM_CHANNELS Represents the number of channels for the AD3542R
 * device, which is 2.
 * @param AD3551R_NUM_CHANNELS Represents the number of channels for the AD3551R
 * device, which is 1.
 * @param AD3552R_NUM_CHANNELS Represents the number of channels for the AD3552R
 * device, which is 2.
 ******************************************************************************/
enum num_channels {
	AD3541R_NUM_CHANNELS = 1,
	AD3542R_NUM_CHANNELS = 2,
	AD3551R_NUM_CHANNELS = 1,
	AD3552R_NUM_CHANNELS = 2
};

#define AD3552R_CH_OUTPUT_RANGE_CUSTOM 100

/***************************************************************************//**
 * @brief The `ad3552r_ch_gain_scaling` enumeration defines a set of constants
 * representing different gain scaling factors for a channel in the
 * AD3552R device. Each enumerator corresponds to a specific gain scaling
 * value, allowing for easy selection and configuration of the desired
 * gain scaling in the device's operation.
 *
 * @param AD3552R_CH_GAIN_SCALING_1 Represents a gain scaling factor of 1.
 * @param AD3552R_CH_GAIN_SCALING_0_5 Represents a gain scaling factor of 0.5.
 * @param AD3552R_CH_GAIN_SCALING_0_25 Represents a gain scaling factor of 0.25.
 * @param AD3552R_CH_GAIN_SCALING_0_125 Represents a gain scaling factor of
 * 0.125.
 ******************************************************************************/
enum ad3552r_ch_gain_scaling {
	/* Gain scaling of 1 */
	AD3552R_CH_GAIN_SCALING_1,
	/* Gain scaling of 0.5 */
	AD3552R_CH_GAIN_SCALING_0_5,
	/* Gain scaling of 0.25 */
	AD3552R_CH_GAIN_SCALING_0_25,
	/* Gain scaling of 0.125 */
	AD3552R_CH_GAIN_SCALING_0_125,
};

/***************************************************************************//**
 * @brief The `ad3552r_offset_polarity` is an enumeration that defines the
 * possible polarities for an offset in the AD3552R driver. It includes
 * two possible values: `AD3552R_OFFSET_POLARITY_POSITIVE` for positive
 * offset polarity and `AD3552R_OFFSET_POLARITY_NEGATIVE` for negative
 * offset polarity. This enumeration is used to specify the direction of
 * the offset applied in the device's configuration.
 *
 * @param AD3552R_OFFSET_POLARITY_POSITIVE Represents a positive offset
 * polarity.
 * @param AD3552R_OFFSET_POLARITY_NEGATIVE Represents a negative offset
 * polarity.
 ******************************************************************************/
enum ad3552r_offset_polarity {
	/* Positive offset */
	AD3552R_OFFSET_POLARITY_POSITIVE,
	/* Negative offset */
	AD3552R_OFFSET_POLARITY_NEGATIVE,
};

/***************************************************************************//**
 * @brief The `ad3552r_dev_attributes` enumeration defines various device
 * attributes for the AD3552R, including settings for SDO drive strength,
 * reference voltage selection, CRC enablement, and SPI configurations.
 * These attributes allow for customization of the device's operation,
 * particularly in terms of communication and reference voltage handling.
 * The enumeration is conditionally compiled to include SPI-related
 * attributes if QSPI is implemented, providing flexibility for different
 * hardware configurations.
 *
 * @param AD3552R_SDO_DRIVE_STRENGTH Represents the drive strength of the SDO
 * pin.
 * @param AD3552R_VREF_SELECT Selects the reference voltage source, either
 * internal or external.
 * @param AD3552R_CRC_ENABLE Enables or disables CRC for data integrity.
 * @param AD3552R_SPI_MULTI_IO_MODE Defines the SPI mode as standard, dual, or
 * quad.
 * @param AD3552R_SPI_DATA_RATE Specifies the SPI data rate as single or dual.
 * @param AD3552R_SPI_SYNCHRONOUS_ENABLE Enables dual SPI synchronous mode.
 ******************************************************************************/
enum ad3552r_dev_attributes {
	/* Direct register values */
	/* From 0-3 */
	AD3552R_SDO_DRIVE_STRENGTH,
	/*
	 * 0 -> Internal Vref, vref_io pin floating (default)
	 * 1 -> Internal Vref, vref_io driven by internal vref
	 * 2 or 3 -> External Vref
	 */
	AD3552R_VREF_SELECT,
	/* Enable / Disable CRC */
	AD3552R_CRC_ENABLE,
#ifdef AD3552R_QSPI_IMPLEMENTED
	/* Spi mode: Strandard, Dual or Quad */
	AD3552R_SPI_MULTI_IO_MODE,
	/* Spi data rate: Single or dual */
	AD3552R_SPI_DATA_RATE,
	/* Dual spi synchronous mode */
	AD3552R_SPI_SYNCHRONOUS_ENABLE,
#endif
};

/***************************************************************************//**
 * @brief The `ad3552r_ch_attributes` enum defines a set of attributes for
 * configuring and controlling the channels of the AD3552R device. These
 * attributes include powerdown states, output range selection, gain and
 * offset settings, and control flags for both software and hardware
 * operations. Each enumerator in this enum corresponds to a specific
 * channel attribute that can be used to manipulate the behavior and
 * configuration of the DAC channels, providing flexibility in managing
 * the output characteristics and operational modes of the device.
 *
 * @param AD3552R_CH_DAC_POWERDOWN Represents the DAC powerdown state.
 * @param AD3552R_CH_AMPLIFIER_POWERDOWN Represents the DAC amplifier powerdown
 * state.
 * @param AD3552R_CH_OUTPUT_RANGE_SEL Selects the output range from predefined
 * enums.
 * @param AD3552R_CH_RANGE_OVERRIDE Overrides the range selector to manually set
 * the output voltage range.
 * @param AD3552R_CH_GAIN_OFFSET Manually sets the offset voltage.
 * @param AD3552R_CH_GAIN_OFFSET_POLARITY Sets the polarity of the offset.
 * @param AD3552R_CH_GAIN_SCALING_P Represents PDAC gain scaling.
 * @param AD3552R_CH_GAIN_SCALING_N Represents NDAC gain scaling.
 * @param AD3552R_CH_TRIGGER_SOFTWARE_LDAC Triggers a software LDAC.
 * @param AD3552R_CH_HW_LDAC_MASK Represents the hardware LDAC mask.
 * @param AD3552R_CH_RFB Represents the Rfb value.
 * @param AD3552R_CH_FAST_EN Enables writing to fast registers with only 16 bits
 * of data.
 * @param AD3552R_CH_SELECT Selects the channel, allowing Input -> DAC and Mask
 * -> DAC.
 * @param AD3552R_CH_CODE Represents the raw value to be set to the DAC.
 ******************************************************************************/
enum ad3552r_ch_attributes {
	/* DAC powerdown */
	AD3552R_CH_DAC_POWERDOWN,
	/* DAC amplifier powerdown */
	AD3552R_CH_AMPLIFIER_POWERDOWN,
	/* Select from enum ad3552r_ch_output_range or ad3542r_ch_output_range */
	AD3552R_CH_OUTPUT_RANGE_SEL,
	/*
	 * Over-rider the range selector in order to manually set the output
	 * voltage range
	 */
	AD3552R_CH_RANGE_OVERRIDE,
	/* Manually set the offset voltage */
	AD3552R_CH_GAIN_OFFSET,
	/* Sets the polarity of the offset. */
	AD3552R_CH_GAIN_OFFSET_POLARITY,
	/* PDAC gain scaling */
	AD3552R_CH_GAIN_SCALING_P,
	/* NDAC gain scaling */
	AD3552R_CH_GAIN_SCALING_N,
	/* Trigger a software LDAC */
	AD3552R_CH_TRIGGER_SOFTWARE_LDAC,
	/* Hardware LDAC Mask */
	AD3552R_CH_HW_LDAC_MASK,
	/* Rfb value */
	AD3552R_CH_RFB,
	/* Write to fast regs (only 16 bits of data) */
	AD3552R_CH_FAST_EN,
	/* Channel select. When set allow Input -> DAC and Mask -> DAC */
	AD3552R_CH_SELECT,
	/* Raw value to be set to dac */
	AD3552R_CH_CODE
};

/***************************************************************************//**
 * @brief The `ad3552r_write_mode` enumeration defines the modes of writing data
 * to the AD3552R device, specifying whether the data is written directly
 * to the DAC registers or to input registers, and whether the LDAC (Load
 * DAC) signal needs to be manually triggered by the user or is
 * automatically handled by the driver. This allows for flexible control
 * over how and when the DAC outputs are updated.
 *
 * @param AD3552R_WRITE_DAC_REGS Write to DAC registers without needing to
 * trigger LDAC.
 * @param AD3552R_WRITE_INPUT_REGS Write to input registers, requiring the user
 * to trigger LDAC.
 * @param AD3552R_WRITE_INPUT_REGS_AND_TRIGGER_LDAC Write to input registers
 * with LDAC triggered by the
 * driver.
 ******************************************************************************/
enum ad3552r_write_mode {
	/* Write to DAC registers. No need to trigger LDAC */
	AD3552R_WRITE_DAC_REGS,
	/* Write to input registers. User needs to trigger LDAC */
	AD3552R_WRITE_INPUT_REGS,
	/* Write to input registers. LDAC is triggered by the driver */
	AD3552R_WRITE_INPUT_REGS_AND_TRIGGER_LDAC
};

/* By default all values are set to 0 */
/***************************************************************************//**
 * @brief The `ad3552_transfer_config` structure is used to configure the data
 * transfer settings for the AD3552R device. It includes fields to define
 * the streaming mode length, address sequencing, and instruction mode.
 * Additionally, if QSPI is implemented, it allows configuration of
 * multi-IO modes, DDR settings, and synchronous SPI configurations. This
 * structure is essential for managing how data is transferred to and
 * from the device, ensuring that the correct modes and behaviors are
 * applied during operation.
 *
 * @param stream_mode_length Defines the length of the loop when streaming data.
 * @param addr_asc Determines sequential addressing behavior.
 * @param single_instr Selects streaming or single instruction mode.
 * @param stream_length_keep_value Prevents the STREAM_MODE LENGTH value from
 * automatically resetting to zero.
 * @param multi_io_mode Controls the SPI mode: Single (0), Dual (1), Quad (2)
 * (only if QSPI is implemented).
 * @param ddr Indicates if the DAC word is expected in Double Data Rate (DDR)
 * configuration (only if QSPI is implemented).
 * @param synchronous Indicates if the SPI interface is expected as a dual
 * synchronous configuration (only if QSPI is implemented).
 ******************************************************************************/
struct ad3552_transfer_config {
	/* Defines the length of the loop when streaming data */
	uint8_t		stream_mode_length;
	/* Determines Sequential Addressing Behavior */
	uint8_t		addr_asc : 1;
	/* Select Streaming or Single Instruction Mode */
	uint8_t		single_instr: 1;
	/*
	 * Set this bit to prevent the STREAM_MODE LENGTH value from
	 * automatically resetting to zero
	 */
	uint8_t		stream_length_keep_value : 1;
#ifdef AD3552R_QSPI_IMPLEMENTED
	/* Controls the SPI. Single (0), Dual (1), Quad (2)*/
	uint8_t		multi_io_mode : 2;
	/*
	 * When this bIt is set, the DAC word is expected in
	 * Double Data Rate(DDR) configuration
	 */
	uint8_t		ddr : 1;
	/*
	 * When this bit is set the SPI interface is expected as a dual
	 * synchronous configuration
	 */
	uint8_t		synchronous : 1;
#endif
};

/***************************************************************************//**
 * @brief The `ad3552_transfer_data` structure is designed to encapsulate the
 * necessary information for a data transfer operation in a SPI
 * communication context. It includes the starting address, a pointer to
 * the data buffer, the length of the data, a flag to indicate whether
 * the operation is a read or write, and an optional pointer to a
 * specific SPI configuration. This structure is essential for managing
 * data transactions with the AD3552R device, allowing for flexible and
 * configurable data transfers.
 *
 * @param addr Starting address for the data transfer.
 * @param data Pointer to the data to be transferred.
 * @param len Size of the data to be transferred.
 * @param is_read Flag indicating if the transaction is a read (true) or write
 * (false).
 * @param spi_cfg Pointer to the SPI configuration, default or last used if
 * NULL.
 ******************************************************************************/
struct ad3552_transfer_data {
	/* Starting address for transfer */
	uint8_t		addr;
	/* Data to transfer */
	uint8_t		*data;
	/* Size of data to transfer */
	uint32_t	len;
	/* Read transaction if true, write transfer otherwise */
	uint8_t		is_read : 1;
	/* If NULL will be default or last configured will be used */
	struct ad3552_transfer_config *spi_cfg;
};

/***************************************************************************//**
 * @brief The `ad3552r_ch_data` structure is used to encapsulate various
 * configuration parameters for a channel in the AD3552R device. It
 * includes fields for scale and offset values, both integer and decimal
 * parts, as well as gain and offset adjustments. The structure also
 * contains fields for feedback resistor value, gain scaling parameters,
 * output range settings, and a flag to enable fast mode. This structure
 * is essential for configuring the channel's behavior and output
 * characteristics in the AD3552R device.
 *
 * @param scale_int Represents the integer part of the scale factor for the
 * channel.
 * @param scale_dec Represents the decimal part of the scale factor for the
 * channel.
 * @param offset_int Represents the integer part of the offset for the channel.
 * @param offset_dec Represents the decimal part of the offset for the channel.
 * @param gain_offset Specifies the gain offset for the channel.
 * @param offset Defines the offset value for the channel.
 * @param offset_polarity Indicates the polarity of the offset, either positive
 * or negative.
 * @param rfb Specifies the feedback resistor value for the channel.
 * @param n Represents a parameter related to gain scaling.
 * @param p Represents another parameter related to gain scaling.
 * @param range Defines the output range for the channel.
 * @param range_override Indicates if the range is manually overridden.
 * @param fast_en Enables fast mode for the channel.
 ******************************************************************************/
struct ad3552r_ch_data {
	int32_t scale_int;
	int32_t scale_dec;
	int32_t offset_int;
	int32_t offset_dec;
	int16_t gain_offset;
	uint16_t offset;
	uint8_t offset_polarity;
	uint16_t rfb;
	uint8_t n;
	uint8_t p;
	uint8_t range;
	uint8_t range_override;
	uint8_t fast_en;
};

/***************************************************************************//**
 * @brief The `ad3552r_desc` structure is a comprehensive descriptor for
 * managing the AD3552R DAC device, encapsulating configuration and state
 * information necessary for its operation. It includes SPI
 * configuration, pointers to various hardware interface descriptors such
 * as SPI, GPIO for LDAC and reset, and AXI components for clock
 * generation and data transfer. The structure also maintains channel-
 * specific data, transfer settings, and operational flags, enabling
 * efficient control and communication with the DAC device.
 *
 * @param spi_cfg Configuration for SPI transfer settings.
 * @param spi Pointer to the SPI descriptor for communication.
 * @param ldac Pointer to the GPIO descriptor for the LDAC pin.
 * @param reset Pointer to the GPIO descriptor for the reset pin.
 * @param clkgen Pointer to the AXI clock generator structure.
 * @param ad3552r_core_ip Pointer to the AXI DAC core IP structure.
 * @param dmac_ip Pointer to the AXI DMAC structure for data transfer.
 * @param ch_data Array of channel data structures for each channel.
 * @param axi_xfer_size Size of the AXI transfer.
 * @param crc_table Table for CRC8 calculations.
 * @param chip_id Identifier for the chip.
 * @param num_spi_data_lanes Number of SPI data lanes used.
 * @param crc_en Flag indicating if CRC is enabled.
 * @param is_simultaneous Flag indicating if simultaneous updates are enabled.
 * @param single_transfer Flag indicating if single transfer mode is enabled.
 * @param axi Flag indicating if AXI interface is used.
 ******************************************************************************/
struct ad3552r_desc {
	struct ad3552_transfer_config spi_cfg;
	struct no_os_spi_desc *spi;
	struct no_os_gpio_desc *ldac;
	struct no_os_gpio_desc *reset;
	struct axi_clkgen *clkgen;
	struct axi_dac  *ad3552r_core_ip;
	struct axi_dmac *dmac_ip;
	struct ad3552r_ch_data ch_data[AD3552R_MAX_NUM_CH];
	uint8_t axi_xfer_size;
	uint8_t crc_table[NO_OS_CRC8_TABLE_SIZE];
	uint8_t chip_id;
	uint8_t num_spi_data_lanes;
	uint8_t crc_en : 1;
	uint8_t is_simultaneous : 1;
	uint8_t single_transfer : 1;
	uint8_t axi: 1;
};

/***************************************************************************//**
 * @brief The `ad3552r_custom_output_range_cfg` structure is used to define a
 * custom output range configuration for the AD3552R device. It includes
 * parameters for gain offset and scaling, both positive and negative, as
 * well as a resistance feedback value. This configuration allows for
 * precise control over the output range of the device, enabling
 * customization beyond standard predefined ranges.
 *
 * @param gain_offset An integer representing the gain offset for the custom
 * output range configuration.
 * @param gain_scaling_p_inv_log2 A 3-bit unsigned integer representing the
 * inverse logarithm base 2 of the positive gain
 * scaling factor.
 * @param gain_scaling_n_inv_log2 A 3-bit unsigned integer representing the
 * inverse logarithm base 2 of the negative gain
 * scaling factor.
 * @param rfb_ohms An unsigned integer representing the resistance feedback
 * value in ohms.
 ******************************************************************************/
struct ad3552r_custom_output_range_cfg {
	int16_t gain_offset;
	/* GainP = 1 / ( 2 ^ gain_scaling_p_inv_log2)
	   From 0 to 3 */
	uint8_t gain_scaling_p_inv_log2;
	/* GainP = 1 / ( 2 ^ gain_scaling_n_inv_log2)
	   From 0 to 3 */
	uint8_t gain_scaling_n_inv_log2;
	/* RFB value */
	uint16_t rfb_ohms;
};

/***************************************************************************//**
 * @brief The `ad3552r_channel_init` structure is used to initialize and
 * configure a channel for the AD3552R device. It includes options to
 * enable the channel, select between fast and normal precision modes,
 * and set the output range either through predefined options or a custom
 * configuration. The `custom_range` member allows for detailed
 * customization of the output range, providing flexibility in channel
 * setup.
 *
 * @param en A boolean flag to enable or disable the channel.
 * @param fast_en A boolean flag to enable fast mode, using only 12 bits
 * precision instead of 16.
 * @param range An 8-bit unsigned integer to specify the output range using
 * predefined enums or a custom range.
 * @param custom_range A structure to configure custom output range settings.
 ******************************************************************************/
struct ad3552r_channel_init {
	bool en;
	/* Use only 12 bits precision instead of 16 for data. */
	bool fast_en;
	/*
	 * Use enum ad3552r_ch_ouput_range or ad3542r_ch_output_range
	 * (Depending on id), or AD3552R_CH_OUTPUT_RANGE_CUSTOM to configure
	 * using custom_output_range.
	 */
	uint8_t range;
	struct ad3552r_custom_output_range_cfg custom_range;
};

/***************************************************************************//**
 * @brief The `ad3552r_init_param` structure is used to initialize the AD3552R
 * device, encapsulating various configuration parameters such as chip
 * identification, SPI and GPIO settings, voltage reference options, and
 * channel-specific settings. It also includes options for enabling CRC,
 * simultaneous updates, and single transfer mode, as well as
 * configurations for using an AXI QSPI controller and setting the AXI
 * clock rate. This structure is essential for setting up the device's
 * operational parameters before use.
 *
 * @param chip_id Specifies the chip identifier from the ad3552r_id enumeration.
 * @param spi_param Holds the SPI initialization parameters.
 * @param reset_gpio_param_optional Optional GPIO parameters for the reset pin,
 * if hardware reset is used.
 * @param ldac_gpio_param_optional Optional GPIO parameters for the LDAC pin, if
 * input registers and LDAC pulse are used.
 * @param use_external_vref Indicates whether an external voltage reference is
 * used.
 * @param vref_out_enable Enables output of the internal voltage reference on
 * the Vref pin.
 * @param sdo_drive_strength Sets the drive strength for the SDO pin, ranging
 * from 0 to 3.
 * @param channels Array of channel initialization parameters, with a size
 * defined by AD3552R_MAX_NUM_CH.
 * @param crc_en Enables CRC error checking if set.
 * @param is_simultaneous Indicates if simultaneous updates are enabled.
 * @param single_transfer Specifies if single transfer mode is enabled.
 * @param axi_qspi_controller Indicates if the AXI QSPI controller is in use.
 * @param axi_clkgen_rate Sets the clock rate for the AXI clock generator.
 * @param clkgen_ip Pointer to the AXI clock generator initialization
 * parameters.
 * @param ad3552r_core_ip Pointer to the AXI DAC core initialization parameters.
 * @param dmac_ip Pointer to the AXI DMAC initialization parameters.
 ******************************************************************************/
struct ad3552r_init_param {
	enum ad3552r_id	chip_id;
	struct no_os_spi_init_param spi_param;
	/* If set, reset is done with RESET pin, otherwise it will be soft */
	struct no_os_gpio_init_param	*reset_gpio_param_optional;
	/* If set, input register are used and LDAC pulse is sent */
	struct no_os_gpio_init_param	*ldac_gpio_param_optional;
	/* If set, use external Vref */
	bool use_external_vref;
	/* If set, output internal Vref on Vref pin */
	bool vref_out_enable;
	/* From 0 to 3 */
	uint8_t sdo_drive_strength;
	struct ad3552r_channel_init channels[AD3552R_MAX_NUM_CH];
	/* Set to enable CRC */
	bool crc_en;
	bool is_simultaneous;
	bool single_transfer;
	/* Set for AXI qspi controller in use */
	bool axi_qspi_controller;
	/* Set AXI clock rate */
	int axi_clkgen_rate;
	/* Points to struct axi_clkgen_init for clkgen ip init params */
	struct axi_clkgen_init *clkgen_ip;
	/* Points to struct axi_dac_init for AXI ip init params */
	struct axi_dac_init *ad3552r_core_ip;
	/* Points to struct axi_dmac_init for AXI DMAC init params */
	struct axi_dmac_init *dmac_ip;
};

/*****************************************************************************/
/************************* Functions Declarations ****************************/
/*****************************************************************************/

/***************************************************************************//**
 * @brief Use this function to find out the number of bytes required to access a
 * specific register in the AD3552R device, based on its address. This is
 * useful when preparing data for SPI communication with the device. The
 * function expects a valid register address and returns the
 * corresponding register length. It handles specific cases for 16-bit
 * and 24-bit register addresses and defaults to a length of 1 byte for
 * other addresses.
 *
 * @param addr The address of the register for which the length is to be
 * determined. It must be a valid register address as defined by the
 * AD3552R device specifications. Invalid or out-of-range addresses
 * may lead to undefined behavior.
 * @return Returns the length of the register in bytes as a uint8_t value, which
 * can be 1, 2, or 3 depending on the address.
 ******************************************************************************/
uint8_t ad3552r_reg_len(uint8_t addr);

/***************************************************************************//**
 * @brief Use this function to obtain the register address for a specific
 * channel based on whether it is a DAC or input register and whether it
 * is in fast mode. This function is useful when you need to access or
 * modify the register values for a particular channel in the AD3552R
 * device. Ensure that the channel number is valid and corresponds to the
 * device's supported channels.
 *
 * @param ch The channel number for which the register address is needed. It
 * should be within the valid range of channels supported by the
 * device.
 * @param is_dac A flag indicating whether the register is a DAC register (non-
 * zero value) or an input register (zero value).
 * @param is_fast A flag indicating whether the register is in fast mode (non-
 * zero value) or not (zero value).
 * @return Returns the register address as an 8-bit unsigned integer for the
 * specified channel and mode.
 ******************************************************************************/
uint8_t ad3552r_get_code_reg_addr(uint8_t ch, uint8_t is_dac, uint8_t is_fast);

/***************************************************************************//**
 * @brief This function sets up and initializes the AD3552R device descriptor
 * based on the provided initialization parameters. It must be called
 * before any other operations on the AD3552R device. The function
 * configures the device for either AXI or SPI communication, initializes
 * necessary GPIOs, and verifies the device ID. It handles memory
 * allocation for the descriptor and ensures the device is reset and
 * configured correctly. If initialization fails at any step, it returns
 * an error code and cleans up any allocated resources.
 *
 * @param desc A pointer to a pointer of type `struct ad3552r_desc`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param param A pointer to a `struct ad3552r_init_param` containing
 * initialization parameters for the device. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int32_t ad3552r_init(struct ad3552r_desc **desc,
		     struct ad3552r_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources and memory
 * associated with an AD3552R device descriptor when it is no longer
 * needed. This function should be called to clean up after using the
 * device to prevent memory leaks. It handles the removal of GPIO and SPI
 * resources if they were initialized, and then frees the descriptor
 * itself. Ensure that the descriptor is valid and has been initialized
 * before calling this function.
 *
 * @param desc A pointer to an `ad3552r_desc` structure representing the device
 * descriptor. This must be a valid, non-null pointer that was
 * previously initialized. The function will handle null checks
 * internally for its members, but the pointer itself must not be
 * null.
 * @return Returns 0 to indicate successful removal of resources. The descriptor
 * is freed and should not be used after this call.
 ******************************************************************************/
int32_t ad3552r_remove(struct ad3552r_desc *desc);

/***************************************************************************//**
 * @brief This function performs a reset of the AD3552R device, which can be
 * either a hardware reset using a GPIO pin or a software reset via
 * register manipulation, depending on the configuration of the device
 * descriptor. It should be called when a reset of the device is
 * required, such as during initialization or to recover from an error
 * state. The function handles both AXI and non-AXI modes, with specific
 * behavior for each. It returns an error code if the reset process
 * fails, ensuring that the device is ready for further operations.
 *
 * @param desc A pointer to an ad3552r_desc structure that describes the device
 * configuration and state. Must not be null. The function expects
 * this structure to be properly initialized before calling.
 * @return Returns 0 on success, or a negative error code if the reset fails.
 ******************************************************************************/
int32_t ad3552r_reset(struct ad3552r_desc *desc);

/* Get status and error bits. If clear_errors is set, errors will be cleared */
/***************************************************************************//**
 * @brief Use this function to obtain the current status and error bits from the
 * AD3552R device. It is useful for monitoring the device's operational
 * state and diagnosing issues. The function can optionally clear error
 * bits after reading them, depending on the `clr_err` parameter. Ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function returns an error code if the operation fails,
 * which should be checked to ensure successful execution.
 *
 * @param desc A pointer to an `ad3552r_desc` structure representing the device
 * descriptor. This must be a valid, initialized descriptor and must
 * not be null.
 * @param status A pointer to a `uint32_t` where the function will store the
 * retrieved status and error bits. This pointer must not be null,
 * and the caller is responsible for managing the memory it points
 * to.
 * @param clr_err A `uint8_t` flag indicating whether to clear error bits after
 * reading them. A non-zero value will clear the errors, while
 * zero will leave them unchanged.
 * @return Returns an `int32_t` error code. A value of 0 indicates success,
 * while a negative value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int32_t ad3552r_get_status(struct ad3552r_desc *desc, uint32_t *status,
			   uint8_t clr_err);

/***************************************************************************//**
 * @brief This function facilitates SPI data transfer to or from an AD3552R
 * device, supporting both read and write operations. It should be called
 * with a valid device descriptor and transfer data structure. The
 * function handles optional SPI configuration updates and supports CRC-
 * enabled transfers if configured. It is essential to ensure that both
 * the descriptor and data parameters are non-null before calling this
 * function.
 *
 * @param desc A pointer to an ad3552r_desc structure representing the device
 * descriptor. Must not be null.
 * @param data A pointer to an ad3552_transfer_data structure containing the
 * transfer details, including address, data buffer, length, and
 * read/write flag. Must not be null.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code for failure, such as -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad3552r_transfer(struct ad3552r_desc *desc,
			 struct ad3552_transfer_data *data);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * address on the AD3552R device. It requires a valid device descriptor
 * and a register address to perform the operation. The function handles
 * both single and multiple byte register writes, depending on the
 * register length. It must be called with a properly initialized
 * descriptor and a valid register address. If the descriptor is null or
 * the address is invalid, the function returns an error code. This
 * function is essential for configuring the device and updating its
 * settings.
 *
 * @param desc A pointer to an ad3552r_desc structure representing the device
 * descriptor. Must not be null. The caller retains ownership.
 * @param addr The register address to which the value will be written. Must be
 * a valid address within the device's register map.
 * @param val The 16-bit value to write to the specified register. The function
 * handles the conversion to the appropriate byte format based on the
 * register length.
 * @return Returns an int32_t error code: 0 on success, -ENODEV if the
 * descriptor is null, or -EINVAL if the address is invalid.
 ******************************************************************************/
int32_t ad3552r_write_reg(struct ad3552r_desc *desc, uint8_t addr,
			  uint16_t val);

/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from the
 * AD3552R device. It requires a valid device descriptor and a pointer to
 * store the read value. The function supports both SPI and AXI
 * interfaces, automatically selecting the appropriate method based on
 * the descriptor configuration. It handles errors such as invalid
 * addresses and null pointers by returning specific error codes. This
 * function should be called when a register value needs to be read from
 * the device, ensuring that the descriptor is properly initialized and
 * the address is within the valid range.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure representing
 * the device. Must not be null.
 * @param addr The address of the register to read. Must be a valid register
 * address for the AD3552R device.
 * @param val A pointer to a uint16_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -ENODEV for null pointers or -EINVAL for invalid addresses.
 ******************************************************************************/
int32_t ad3552r_read_reg(struct ad3552r_desc *desc, uint8_t addr,
			 uint16_t *val);

/***************************************************************************//**
 * @brief This function is used to obtain the current value of a specified
 * device attribute from an AD3552R device. It requires a valid device
 * descriptor and a pointer to store the retrieved value. The function
 * must be called with a valid attribute identifier, and it will return
 * an error if the descriptor or value pointer is null. It is important
 * to ensure that the device is properly initialized before calling this
 * function. The function handles specific attributes differently, and
 * unsupported attributes may result in an error.
 *
 * @param desc A pointer to a valid ad3552r_desc structure representing the
 * device. Must not be null.
 * @param attr An enum value of type ad3552r_dev_attributes specifying the
 * device attribute to retrieve. Must be a valid attribute.
 * @param val A pointer to a uint16_t where the retrieved attribute value will
 * be stored. Must not be null.
 * @return Returns an int32_t indicating success (0) or an error code (negative
 * value) if the operation fails.
 ******************************************************************************/
int32_t ad3552r_get_dev_value(struct ad3552r_desc *desc,
			      enum ad3552r_dev_attributes attr,
			      uint16_t *val);

/***************************************************************************//**
 * @brief Use this function to configure a specific attribute of the AD3552R
 * device by setting it to a given value. This function requires a valid
 * device descriptor and an attribute to modify. It is essential to
 * ensure that the descriptor is properly initialized before calling this
 * function. The function handles specific attributes, and unsupported
 * attributes will result in an error. It is important to note that some
 * attributes may not be implemented, and attempting to set them will
 * also result in an error.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param attr An enum value of type ad3552r_dev_attributes representing the
 * device attribute to set. Only certain attributes are supported,
 * and unsupported ones will result in an error.
 * @param val A 16-bit unsigned integer representing the value to set for the
 * specified attribute. The valid range and effect depend on the
 * attribute being set.
 * @return Returns an int32_t indicating success or an error code. A negative
 * value indicates an error, such as an invalid descriptor or
 * unsupported attribute.
 ******************************************************************************/
int32_t ad3552r_set_dev_value(struct ad3552r_desc *desc,
			      enum ad3552r_dev_attributes attr,
			      uint16_t val);

/***************************************************************************//**
 * @brief Use this function to obtain the current value of a specified attribute
 * for a given channel on the AD3552R device. It is essential to ensure
 * that the descriptor and value pointer are valid before calling this
 * function. The function handles various channel attributes, including
 * fast enable, code, and RFB, among others. It returns an error if the
 * attribute is not supported or if the attribute corresponds to a write-
 * only register. This function is typically used in scenarios where the
 * current configuration or status of a channel needs to be read.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure representing
 * the device. Must not be null.
 * @param attr An enum value of type ad3552r_ch_attributes specifying the
 * channel attribute to retrieve. Must be a valid attribute for the
 * device.
 * @param ch A uint8_t representing the channel number. Must be within the valid
 * range of channels for the device.
 * @param val A pointer to a uint16_t where the retrieved attribute value will
 * be stored. Must not be null.
 * @return Returns an int32_t indicating success (0) or a negative error code if
 * the operation fails, such as when an invalid attribute is specified
 * or a write-only register is accessed.
 ******************************************************************************/
int32_t ad3552r_get_ch_value(struct ad3552r_desc *desc,
			     enum ad3552r_ch_attributes attr,
			     uint8_t ch,
			     uint16_t *val);

/***************************************************************************//**
 * @brief This function is used to set a specific attribute value for a given
 * channel on the AD3552R device. It should be called when you need to
 * configure or modify channel-specific settings such as enabling fast
 * mode, setting the output code, or adjusting the feedback resistor
 * value. The function requires a valid descriptor and channel index, and
 * it handles various attributes by updating the device's registers or
 * internal data structures accordingly. Ensure the descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure. Must not be
 * null. Represents the device descriptor.
 * @param attr An enum value of type ad3552r_ch_attributes indicating which
 * channel attribute to set. Must be a valid attribute defined in
 * the enum.
 * @param ch An unsigned 8-bit integer representing the channel index. Must be
 * within the valid range of channels for the device.
 * @param val A 16-bit unsigned integer representing the value to set for the
 * specified attribute. The interpretation of this value depends on
 * the attribute being set.
 * @return Returns an int32_t error code. Returns 0 on success, or a negative
 * error code if the operation fails or if invalid parameters are
 * provided.
 ******************************************************************************/
int32_t ad3552r_set_ch_value(struct ad3552r_desc *desc,
			     enum ad3552r_ch_attributes attr,
			     uint8_t ch,
			     uint16_t val);

/***************************************************************************//**
 * @brief Use this function to obtain the integer and decimal scale factors for
 * a specific channel of the AD3552R device. This function is typically
 * called when you need to understand or adjust the scaling applied to
 * the channel's output. Ensure that the descriptor is properly
 * initialized and that the channel index is within the valid range
 * before calling this function. The function will return an error if the
 * descriptor is null, the channel index is out of range, or if the
 * integer pointer is null.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure. Must not be
 * null.
 * @param ch The channel index for which to retrieve the scale factors. Must be
 * less than AD3552R_MAX_NUM_CH.
 * @param integer A pointer to an int32_t where the integer part of the scale
 * factor will be stored. Must not be null.
 * @param dec A pointer to an int32_t where the decimal part of the scale factor
 * will be stored. Can be null if the decimal part is not needed.
 * @return Returns 0 on success. Returns -EINVAL if any input validation fails.
 ******************************************************************************/
int32_t ad3552r_get_scale(struct ad3552r_desc *desc, uint8_t ch,
			  int32_t *integer, int32_t *dec);

/***************************************************************************//**
 * @brief Use this function to obtain the integer and decimal parts of the
 * offset for a specific channel in the AD3552R device. It is essential
 * to ensure that the descriptor is valid and that the channel number is
 * within the permissible range before calling this function. The
 * function will populate the provided pointers with the offset values if
 * successful. It is important to handle the case where the function
 * returns an error code, indicating invalid input parameters.
 *
 * @param desc A pointer to an ad3552r_desc structure representing the device
 * descriptor. Must not be null.
 * @param ch The channel number for which the offset is to be retrieved. Must be
 * less than AD3552R_MAX_NUM_CH.
 * @param integer A pointer to an int32_t where the integer part of the offset
 * will be stored. Must not be null.
 * @param dec A pointer to an int32_t where the decimal part of the offset will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid.
 ******************************************************************************/
int32_t ad3552r_get_offset(struct ad3552r_desc *desc, uint8_t ch,
			   int32_t *integer, int32_t *dec);

/***************************************************************************//**
 * @brief This function is used to trigger a Load DAC (LDAC) operation on the
 * AD3552R device, which updates the DAC outputs with the values from the
 * input registers. It can be used in both fast and normal modes,
 * depending on the `is_fast` parameter. The function requires a valid
 * device descriptor and a mask indicating which channels to update. If
 * the LDAC pin is not configured, the function performs a software LDAC
 * by writing to the appropriate register. Otherwise, it manipulates the
 * LDAC GPIO pin to trigger the update. This function should be called
 * when the input registers have been updated and the new values need to
 * be applied to the DAC outputs.
 *
 * @param desc A pointer to an `ad3552r_desc` structure representing the device
 * descriptor. Must not be null.
 * @param mask A 16-bit mask indicating which channels to update. Each bit
 * corresponds to a channel.
 * @param is_fast A boolean value (0 or 1) indicating whether to use fast mode.
 * Non-zero for fast mode, zero for normal mode.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad3552r_ldac_trigger(struct ad3552r_desc *desc, uint16_t mask,
			     uint8_t is_fast);

/***************************************************************************//**
 * @brief This function is used to control the asynchronous mode of the AD3552R
 * device by setting the LDAC pin to either low or high. It should be
 * called when you need to enable or disable asynchronous updates to the
 * DAC outputs. The function requires a valid device descriptor and a
 * boolean flag indicating whether to enable or disable the mode. It
 * returns an error code if the operation fails, which should be checked
 * by the caller to ensure successful execution.
 *
 * @param desc A pointer to an ad3552r_desc structure representing the device
 * descriptor. Must not be null, and should be properly initialized
 * before calling this function.
 * @param enable A uint8_t value where non-zero enables asynchronous mode and
 * zero disables it. The value is interpreted as a boolean flag.
 * @return Returns an int32_t error code. A return value of 0 indicates success,
 * while a negative value indicates an error occurred during the
 * operation.
 ******************************************************************************/
int32_t ad3552r_set_asynchronous(struct ad3552r_desc *desc, uint8_t enable);

/* Send one sample at a time, one after an other or at a LDAC_period interval.
 * If LDAC pin set, send LDAC signal. Otherwise software LDAC is used. */
/***************************************************************************//**
 * @brief This function is used to write a series of samples to the AD3552R
 * device, targeting specific channels as indicated by the channel mask.
 * It supports different write modes, including direct DAC register
 * writes and input register writes with optional LDAC triggering. The
 * function must be called with a valid device descriptor and a non-null
 * data buffer. It handles both single and multiple channel writes, but
 * requires that the fast mode setting is consistent across channels when
 * writing to all channels simultaneously. The function returns an error
 * code if any write operation fails.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure representing
 * the device. Must not be null.
 * @param data A pointer to an array of 16-bit samples to be written. Must not
 * be null and should contain at least 'samples' elements.
 * @param samples The number of samples to write. Must be a positive integer.
 * @param ch_mask A bitmask indicating which channels to write to. Valid values
 * depend on the device configuration, such as
 * AD3552R_MASK_ALL_CH for all channels.
 * @param mode An enum value of type ad3552r_write_mode indicating the write
 * mode. Determines whether LDAC is triggered automatically or
 * manually.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad3552r_write_samples(struct ad3552r_desc *desc, uint16_t *data,
			      uint32_t samples, uint32_t ch_mask,
			      enum ad3552r_write_mode mode);

/***************************************************************************//**
 * @brief This function configures the AD3552R device to enable or disable
 * simultaneous update mode, which affects how channel data is updated.
 * It should be called when the user needs to synchronize updates across
 * channels. The function requires that both channels have the same fast
 * mode setting; otherwise, it returns an error. The function must be
 * called with a valid descriptor that has been properly initialized.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure. This
 * parameter must not be null and should represent a valid device
 * descriptor. The function will return an error if the fast mode
 * settings of the channels are not identical.
 * @return Returns 0 on success, or a negative error code if the channels' fast
 * mode settings differ or if another error occurs.
 ******************************************************************************/
int32_t ad3552r_simulatneous_update_enable(struct ad3552r_desc *desc);

/* DMA buffering, fast mode, AXI QSPI */
/***************************************************************************//**
 * @brief This function is used to transfer a specified number of samples from a
 * buffer to the AD3552R device using AXI DMA. It supports both cyclic
 * and non-cyclic transfers. For cyclic transfers, the function can run
 * indefinitely or for a specified number of seconds. The function must
 * be called with a valid descriptor that has been properly initialized,
 * and it assumes that the AXI DMA is available. It handles pre-transfer
 * and post-transfer configurations and waits for the transfer to
 * complete in non-cyclic mode.
 *
 * @param desc A pointer to an initialized ad3552r_desc structure. This must not
 * be null and must have a valid AXI DMA configuration.
 * @param buf A pointer to the buffer containing the data to be transferred. The
 * buffer must not be null and should contain at least 'samples'
 * number of samples.
 * @param samples The number of samples to transfer. This value determines the
 * size of the data transfer.
 * @param cyclic A boolean indicating whether the transfer should be cyclic. If
 * true, the transfer will repeat indefinitely or for
 * 'cyclic_secs' seconds.
 * @param cyclic_secs The number of seconds to run the cyclic transfer. If set
 * to 0, the transfer will run indefinitely. This parameter
 * is ignored if 'cyclic' is false.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad3552r_axi_write_data(struct ad3552r_desc *desc, uint32_t *buf,
			       uint16_t samples, bool cyclic, int cyclic_secs);

#endif /* _AD3552R_H_ */
