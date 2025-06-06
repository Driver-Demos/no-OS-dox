/***************************************************************************//**
 *   @file   ltc2672.h
 *   @brief  Header file of ltc2672 Driver.
 *   @author JSanBuen (jose.sanbuenaventura@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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

#ifndef __LTC2672_H__
#define __LTC2672_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_error.h"
#include "no_os_units.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/* LTC2672 Masks */
#define LTC2672_16_DONT_CARE 	0xFFF0
#define LTC2672_MUX_DONT_CARE 	0xFFFE0
#define LTC2672_DUMMY		 	0xFFFF
#define LTC2672_FAULT_REG_MASK	0xFF0000

/* LTC2672 Constants */
#define LTC2672_BASE_CURRENT 			3.125 // base current in mA
#define LTC2672_VMINUS_FIXED_CURRENT 	-80	// Fixed V- Current as per data sheet
#define LTC2672_OFF_CURRENT 			0
#define LTC2672_300MA_CURRENT 			300
#define LTC2672_16BIT_RESO   			65535
#define LTC2672_12BIT_RESO   			4095
#define LTC2672_TOTAL_CHANNELS 			5
#define LTC2672_MAX_CONFIG_MASK  		15
#define LTC2672_MAX_TOGGLE_MASK  		31
#define LTC2672_BIT_SHIFT_12BIT			4
#define LTC2672_NUM_MUX_SELECTS			22
#define LTC2672_NUM_CURRENT_SPANS		10
#define LTC2672_NUM_FAULTS				7

/***************************************************************************//**
 * @brief The `ltc2672_commands` enumeration defines a set of command codes used
 * to control the LTC2672 device, a digital-to-analog converter (DAC).
 * Each enumerator represents a specific command that can be sent to the
 * device to perform operations such as setting the output code for a
 * channel, powering up or down channels, configuring spans, and
 * monitoring the multiplexer. These commands facilitate the management
 * of the DAC's channels and their respective configurations, enabling
 * precise control over the device's output behavior.
 *
 * @param LTC2672_CODE_TO_CHANNEL_X Command to set code to a specific channel.
 * @param LTC2672_PWRUP_UPD_CHANNEL_X Command to power up and update a specific
 * channel.
 * @param LTC2672_CODE_TO_CHANNEL_X_PWRUP_UPD_CHANNEL_ALL Command to set code to
 * a specific channel and
 * power up and update
 * all channels.
 * @param LTC2672_CODE_PWRUP_UPD_CHANNEL_X Command to set code, power up, and
 * update a specific channel.
 * @param LTC2672_PWRDWN_CHANNEL_X Command to power down a specific channel.
 * @param LTC2672_PWRDWN_DEV Command to power down the entire device.
 * @param LTC2672_SPAN_TO_CHANNEL_X Command to set span to a specific channel.
 * @param LTC2672_CNFG_CMD Command for configuration.
 * @param LTC2672_CODE_TO_CHANNEL_ALL Command to set code to all channels.
 * @param LTC2672_PWRUP_UPD_CHANNEL_ALL Command to power up and update all
 * channels.
 * @param LTC2672_CODE_PWRUP_UPD_CHANNEL_ALL Command to set code, power up, and
 * update all channels.
 * @param LTC2672_MON_MUX Command to monitor multiplexer.
 * @param LTC2672_TOGGLE_SEL Command to select toggle.
 * @param LTC2672_TOGGLE_GLBL Command to toggle globally.
 * @param LTC2672_SPAN_TO_CHANNEL_ALL Command to set span to all channels.
 * @param LTC2672_NO_OP No operation command.
 ******************************************************************************/
enum ltc2672_commands {
	LTC2672_CODE_TO_CHANNEL_X,
	LTC2672_PWRUP_UPD_CHANNEL_X,
	LTC2672_CODE_TO_CHANNEL_X_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_CODE_PWRUP_UPD_CHANNEL_X,
	LTC2672_PWRDWN_CHANNEL_X,
	LTC2672_PWRDWN_DEV,
	LTC2672_SPAN_TO_CHANNEL_X,
	LTC2672_CNFG_CMD,
	LTC2672_CODE_TO_CHANNEL_ALL,
	LTC2672_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_CODE_PWRUP_UPD_CHANNEL_ALL,
	LTC2672_MON_MUX,
	LTC2672_TOGGLE_SEL,
	LTC2672_TOGGLE_GLBL,
	LTC2672_SPAN_TO_CHANNEL_ALL,
	LTC2672_NO_OP
};

/* LTC2672 Command Generation */
#define LTC2672_COMMAND32_GENERATE(comm, add, dat) \
		(0xFF << 24) | (comm << 20) | (add << 16) | (dat)

#define LTC2672_COMMAND24_GENERATE(comm, add, dat) \
		(comm << 20) | (add << 16) | (dat)

#define LTC2672_SPAN_SET(span_code) 	 LTC2672_16_DONT_CARE | span_code

#define LTC2672_MUX_SET(mux_code) 	 	 LTC2672_MUX_DONT_CARE | mux_code

#define LTC2672_MUX32_GENERATE(comm, dat) \
		(0xFF << 24) | (comm << 20) | (dat)

#define LTC2672_MUX24_GENERATE(comm, dat) \
		(comm << 20) | (dat)

/* Device Family */
/***************************************************************************//**
 * @brief The `ltc2672_device_id` enumeration defines the possible device
 * variants for the LTC2672, a digital-to-analog converter (DAC) device.
 * It specifies two variants, one with a 12-bit resolution and another
 * with a 16-bit resolution, allowing the software to distinguish between
 * these two types of devices and configure them accordingly.
 *
 * @param LTC2672_12 Represents a 12-bit variant of the LTC2672 device.
 * @param LTC2672_16 Represents a 16-bit variant of the LTC2672 device.
 ******************************************************************************/
enum ltc2672_device_id {
	LTC2672_12,
	LTC2672_16
};

/* DAC Channels */
/***************************************************************************//**
 * @brief The `ltc2672_dac_ch` enumeration defines the available DAC channels
 * for the LTC2672 device, which is a digital-to-analog converter. Each
 * enumerator corresponds to a specific DAC channel, allowing for easy
 * reference and manipulation of the channels within the software. This
 * enumeration is used to specify which channel is being addressed or
 * configured in various operations involving the LTC2672.
 *
 * @param LTC2672_DAC0 Represents the first DAC channel.
 * @param LTC2672_DAC1 Represents the second DAC channel.
 * @param LTC2672_DAC2 Represents the third DAC channel.
 * @param LTC2672_DAC3 Represents the fourth DAC channel.
 * @param LTC2672_DAC4 Represents the fifth DAC channel.
 ******************************************************************************/
enum ltc2672_dac_ch {
	LTC2672_DAC0,
	LTC2672_DAC1,
	LTC2672_DAC2,
	LTC2672_DAC3,
	LTC2672_DAC4
};

/* Output Range */
/***************************************************************************//**
 * @brief The `ltc2672_out_range` enumeration defines various output current
 * ranges for the LTC2672 device, each associated with a specific current
 * value in milliamperes (mA). These ranges are used to configure the
 * output current of the device's digital-to-analog converter (DAC)
 * channels, allowing for precise control over the current output based
 * on the selected range. The enumeration includes options for turning
 * off the output, as well as several predefined current levels,
 * including a fixed negative current option.
 *
 * @param LTC2672_OFF Represents the off mode for the output range.
 * @param LTC2672_50VREF Represents an output range of 3.125mA.
 * @param LTC2672_100VREF Represents an output range of 6.25mA.
 * @param LTC2672_200VREF Represents an output range of 12.5mA.
 * @param LTC2672_400VREF Represents an output range of 25mA.
 * @param LTC2672_800VREF Represents an output range of 50mA.
 * @param LTC2672_1600VREF Represents an output range of 100mA.
 * @param LTC2672_3200VREF Represents an output range of 200mA.
 * @param LTC2672_VMINUS_VREF Represents a fixed output range of -80mA.
 * @param LTC2672_4800VREF Represents an output range of 300mA.
 ******************************************************************************/
enum ltc2672_out_range {
	LTC2672_OFF, // Off mode
	LTC2672_50VREF, // 3.125mA
	LTC2672_100VREF, // 6.25mA
	LTC2672_200VREF, //12.5mA
	LTC2672_400VREF, // 25mA
	LTC2672_800VREF, // 50mA
	LTC2672_1600VREF, // 100mA
	LTC2672_3200VREF,  // 200mA
	LTC2672_VMINUS_VREF, // Fixed, -80mA
	LTC2672_4800VREF = 0XF // 300mA
};

/* Multiplexer Command Codes */
/***************************************************************************//**
 * @brief The `ltc2672_mux_commands` enumeration defines a set of command codes
 * used to control the multiplexer settings of the LTC2672 device. Each
 * enumerator corresponds to a specific output or input channel that can
 * be selected for monitoring or measurement purposes. The enumeration
 * provides a convenient way to reference these command codes in the
 * software, facilitating the configuration of the multiplexer to route
 * different signals to the output.
 *
 * @param LTC2672_MUX_DISABLED Represents a disabled state for the multiplexer.
 * @param LTC2672_MUX_IOUT0 Selects the IOUT0 output for the multiplexer.
 * @param LTC2672_MUX_IOUT1 Selects the IOUT1 output for the multiplexer.
 * @param LTC2672_MUX_IOUT2 Selects the IOUT2 output for the multiplexer.
 * @param LTC2672_MUX_IOUT3 Selects the IOUT3 output for the multiplexer.
 * @param LTC2672_MUX_IOUT4 Selects the IOUT4 output for the multiplexer.
 * @param LTC2672_MUC_VCC Selects the VCC for the multiplexer.
 * @param LTC2672_MUX_VREF Selects the VREF for the multiplexer.
 * @param LTC2672_MUX_VREF_LO Selects the VREF_LO for the multiplexer.
 * @param LTC2672_MUX_DIE_TEMP Selects the die temperature for the multiplexer.
 * @param LTC2672_MUX_VDD0 Selects the VDD0 for the multiplexer.
 * @param LTC2672_MUX_VDD1 Selects the VDD1 for the multiplexer.
 * @param LTC2672_MUX_VDD2 Selects the VDD2 for the multiplexer.
 * @param LTC2672_MUX_VDD3 Selects the VDD3 for the multiplexer.
 * @param LTC2672_MUX_VDD4 Selects the VDD4 for the multiplexer.
 * @param LTC2672_MUX_VMINUS Selects the VMINUS for the multiplexer.
 * @param LTC2672_MUX_GND Selects the ground for the multiplexer.
 * @param LTC2672_MUX_VOUT0 Selects the VOUT0 for the multiplexer.
 * @param LTC2672_MUX_VOUT1 Selects the VOUT1 for the multiplexer.
 * @param LTC2672_MUX_VOUT2 Selects the VOUT2 for the multiplexer.
 * @param LTC2672_MUX_VOUT3 Selects the VOUT3 for the multiplexer.
 * @param LTC2672_MUX_VOUT4 Selects the VOUT4 for the multiplexer.
 ******************************************************************************/
enum ltc2672_mux_commands {
	LTC2672_MUX_DISABLED,
	LTC2672_MUX_IOUT0,
	LTC2672_MUX_IOUT1,
	LTC2672_MUX_IOUT2,
	LTC2672_MUX_IOUT3,
	LTC2672_MUX_IOUT4,
	LTC2672_MUC_VCC,
	LTC2672_MUX_VREF = 0x08,
	LTC2672_MUX_VREF_LO,
	LTC2672_MUX_DIE_TEMP,
	LTC2672_MUX_VDD0 = 0x10,
	LTC2672_MUX_VDD1,
	LTC2672_MUX_VDD2,
	LTC2672_MUX_VDD3,
	LTC2672_MUX_VDD4,
	LTC2672_MUX_VMINUS = 0X16,
	LTC2672_MUX_GND,
	LTC2672_MUX_VOUT0,
	LTC2672_MUX_VOUT1,
	LTC2672_MUX_VOUT2,
	LTC2672_MUX_VOUT3,
	LTC2672_MUX_VOUT4
};

/* Faults */
/***************************************************************************//**
 * @brief The `ltc2672_faults` enumeration defines a set of possible fault
 * conditions for the LTC2672 device, which is a digital-to-analog
 * converter (DAC). Each enumerator represents a specific fault type,
 * such as open circuit faults on individual channels, an over-
 * temperature condition, an unused fault register bit, and an invalid
 * SPI communication length. This enumeration is used to identify and
 * handle different fault conditions that may occur during the operation
 * of the LTC2672 device.
 *
 * @param LTC2672_OPEN_OUT0 Indicates an open circuit fault on channel 0.
 * @param LTC2672_OPEN_OUT1 Indicates an open circuit fault on channel 1.
 * @param LTC2672_OPEN_OUT2 Indicates an open circuit fault on channel 2.
 * @param LTC2672_OPEN_OUT3 Indicates an open circuit fault on channel 3.
 * @param LTC2672_OPEN_OUT4 Indicates an open circuit fault on channel 4.
 * @param LTC2672_OVER_TEMP Indicates an over-temperature fault when the
 * temperature exceeds 175 degrees Celsius.
 * @param LTC2672_UNUSED Represents an unused fault register bit.
 * @param LTC2672_INV_LENGTH Indicates an invalid SPI length fault when the
 * length is not 24 or 32 times n.
 ******************************************************************************/
enum ltc2672_faults {
	LTC2672_OPEN_OUT0, // Open circuit CH0
	LTC2672_OPEN_OUT1, // Open circuit CH0
	LTC2672_OPEN_OUT2, // Open circuit CH0
	LTC2672_OPEN_OUT3, // Open circuit CH0
	LTC2672_OPEN_OUT4, // Open circuit CH0
	LTC2672_OVER_TEMP, // Over-temperature (T > 175 deg C)
	LTC2672_UNUSED, // Unused fault register bit
	LTC2672_INV_LENGTH, // Invalid SPI Length (len != 24 or 32 * n)
};

/***************************************************************************//**
 * @brief The `ltc2672_dev` structure is a descriptor for the LTC2672 device,
 * which is a multi-channel digital-to-analog converter (DAC). It
 * includes a SPI descriptor for communication, a device ID to specify
 * the variant of the LTC2672, and arrays to manage the output spans and
 * maximum currents for each of the DAC channels. Additionally, it keeps
 * track of the last command sent to the device and includes a global
 * toggle bit for controlling the device's toggle functionality.
 *
 * @param comm_desc Pointer to the SPI descriptor used for communication.
 * @param id Indicates the device variant of the LTC2672.
 * @param out_spans Array defining the output range for each DAC channel.
 * @param max_currents Array specifying the maximum current for each DAC channel
 * in microamperes.
 * @param prev_command Tracks the previous command sent to the device.
 * @param global_toggle Boolean flag indicating the state of the global toggle
 * bit.
 ******************************************************************************/
struct ltc2672_dev {
	/* SPI descriptor */
	struct no_os_spi_desc *comm_desc;
	/* Device Variant indicator */
	enum ltc2672_device_id id;
	/* DAC Channel Spans */
	enum ltc2672_out_range out_spans[LTC2672_TOTAL_CHANNELS];
	/* Maximum Current Per Channel in uA */
	uint32_t max_currents[LTC2672_TOTAL_CHANNELS];
	/* Previous command tracker */
	uint32_t prev_command;
	/* Global toggle bit flag */
	bool global_toggle;
};

/***************************************************************************//**
 * @brief The `ltc2672_init_param` structure is used to hold the initialization
 * parameters required for setting up the LTC2672 device. It includes a
 * SPI descriptor for configuring the SPI communication interface and an
 * identifier for specifying the device variant, which is crucial for
 * ensuring compatibility and correct operation of the device.
 *
 * @param spi_init A structure containing the initialization parameters for the
 * SPI communication interface.
 * @param id An enumeration indicating the specific device variant of the
 * LTC2672.
 ******************************************************************************/
struct ltc2672_init_param {
	/* SPI descriptor */
	struct no_os_spi_init_param spi_init;
	/* Device Variant indicator */
	enum ltc2672_device_id id;
};

/***************************************************************************//**
 * @brief This function initializes the LTC2672 device by allocating necessary
 * resources and setting up the communication interface using SPI. It
 * should be called before any other operations on the LTC2672 device to
 * ensure that the device is properly configured and ready for
 * communication. The function requires a valid initialization parameter
 * structure and will return an error code if initialization fails, such
 * as when memory allocation is unsuccessful or SPI initialization
 * encounters an issue.
 *
 * @param device A pointer to a pointer of type `struct ltc2672_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct ltc2672_init_param` containing
 * initialization parameters, including SPI configuration and
 * device ID. This must not be null and should be properly
 * initialized before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -ENOMEM for
 * memory allocation failure.
 ******************************************************************************/
int ltc2672_init(struct ltc2672_dev **, struct ltc2672_init_param *);

/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources allocated
 * for an LTC2672 device when it is no longer needed. This function
 * should be called to prevent resource leaks after the device has been
 * initialized and used. It is important to ensure that the device
 * pointer is valid and not null before calling this function, as passing
 * a null pointer will result in an error. The function will handle the
 * deallocation of the SPI communication descriptor and the device
 * structure itself.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device
 * to be removed. Must not be null. If null, the function returns
 * an error code indicating the device is not found.
 * @return Returns 0 on successful removal of the device, or a negative error
 * code if the device pointer is null or if there is an error in
 * removing the SPI communication descriptor.
 ******************************************************************************/
int ltc2672_remove(struct ltc2672_dev *);

/***************************************************************************//**
 * @brief This function is used to send a command to the LTC2672 device and read
 * back the response using SPI communication. It should be called when a
 * specific command needs to be executed on the device, and the result of
 * that command needs to be captured. The function requires a valid
 * device descriptor and a command to be sent. It supports both 24-bit
 * and 32-bit command formats, determined by the `is_32` flag. The
 * function updates the `prev_command` field of the device structure with
 * the response received from the device. It is important to ensure that
 * the device has been properly initialized before calling this function.
 *
 * @param device A pointer to an `ltc2672_dev` structure representing the
 * device. Must not be null. The structure should be properly
 * initialized before use.
 * @param comm A 32-bit unsigned integer representing the command to be sent to
 * the device. The command should be formatted according to the
 * device's protocol.
 * @param is_32 A boolean flag indicating whether the command is 32 bits (true)
 * or 24 bits (false). Determines the number of bytes to be
 * transmitted.
 * @return Returns 0 on success, or a negative error code if the SPI transaction
 * fails. The `prev_command` field of the `device` structure is updated
 * with the response from the device.
 ******************************************************************************/
int ltc2672_transaction(struct ltc2672_dev *device, uint32_t, bool);

/***************************************************************************//**
 * @brief This function is used to convert a specified current value into a DAC
 * code for a given channel of the LTC2672 device. It is essential to
 * call this function when you need to set a specific current output on a
 * DAC channel, as it translates the current into a format that the DAC
 * can understand. The function requires a valid device structure and a
 * channel identifier, and it assumes that the device has been properly
 * initialized. The function handles different device variants by
 * adjusting the resolution accordingly.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param dac_current The desired current in microamperes to be converted to a
 * DAC code. It should be within the range supported by the
 * device and the specific channel.
 * @param out_ch An enum value of type ltc2672_dac_ch representing the DAC
 * channel for which the current is to be converted. Must be a
 * valid channel identifier.
 * @return Returns a uint32_t value representing the DAC code corresponding to
 * the specified current for the given channel.
 ******************************************************************************/
uint32_t ltc2672_current_to_code(struct ltc2672_dev *device, uint32_t,
				 enum ltc2672_dac_ch);

/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) code for a
 * specified channel on the LTC2672 device. It should be used when you
 * need to update the output of a specific DAC channel. The function
 * requires that the device has been properly initialized and that the
 * channel specified is within the valid range. It handles different
 * resolutions based on the device variant and ensures that the code is
 * within the permissible range for the device. If the output span for
 * the channel is set to a specific mode, the function will adjust the
 * command accordingly. The function returns an error code if the input
 * parameters are invalid.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null and should be initialized before calling this
 * function.
 * @param code A 16-bit unsigned integer representing the DAC code to set. For a
 * 12-bit device, the code must not exceed 4095, and for a 16-bit
 * device, it must not exceed 65535.
 * @param out_ch An enum value of type ltc2672_dac_ch representing the DAC
 * channel to set. Must be one of the defined channels
 * (LTC2672_DAC0 to LTC2672_DAC4).
 * @return Returns 0 on success or a negative error code if the input parameters
 * are invalid.
 ******************************************************************************/
int ltc2672_set_code_channel(struct ltc2672_dev *device, uint16_t code,
			     enum ltc2672_dac_ch out_ch);

/***************************************************************************//**
 * @brief This function sets the output current for a specified DAC channel on
 * the LTC2672 device. It should be used when you need to configure the
 * current output of a particular channel. The function requires that the
 * specified current is within the allowable range for the channel, and
 * the channel index is valid. If the current is outside the permissible
 * range or the channel index is invalid, the function returns an error
 * code. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and the device should be initialized.
 * @param current The desired current in microamperes to set for the specified
 * channel. Must be within the range defined by the device's
 * max_currents for the channel and not less than
 * LTC2672_OFF_CURRENT.
 * @param out_ch The DAC channel to configure, specified as an enum
 * ltc2672_dac_ch. Must be a valid channel index (LTC2672_DAC0 to
 * LTC2672_DAC4).
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or the operation fails.
 ******************************************************************************/
int ltc2672_set_current_channel(struct ltc2672_dev *, uint32_t,
				enum ltc2672_dac_ch);

/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) code for all
 * channels of the specified LTC2672 device. It should be used when you
 * want to apply the same DAC code across all channels simultaneously.
 * The function requires that the device has been properly initialized
 * and that the code value is within the valid range for the device's
 * resolution. If the code is out of range, the function returns an
 * error. This function does not alter the device state if an invalid
 * code is provided.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null and should be initialized before calling this
 * function. The caller retains ownership.
 * @param code A 16-bit unsigned integer representing the DAC code to set for
 * all channels. For a 12-bit device, the code must not exceed 4095;
 * for a 16-bit device, it must not exceed 65535. If the code is out
 * of range, the function returns an error.
 * @return Returns 0 on success or a negative error code if the code is out of
 * range or if there is a communication failure.
 ******************************************************************************/
int ltc2672_set_code_all_channels(struct ltc2672_dev *device, uint16_t code);

/***************************************************************************//**
 * @brief This function sets the specified current for all channels of the
 * LTC2672 device. It should be used when a uniform current setting is
 * required across all channels. Before calling this function, ensure
 * that all channels have the same output span; otherwise, the function
 * will return an error. This function is typically used in applications
 * where synchronized current output is necessary across multiple
 * channels.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param current The desired current to set for all channels, specified in
 * microamperes. The value should be within the allowable range
 * for the device and channels.
 * @return Returns 0 on success, or a negative error code if the channels do not
 * have the same output span or if another error occurs.
 ******************************************************************************/
int ltc2672_set_current_all_channels(struct ltc2672_dev *, uint32_t);

/***************************************************************************//**
 * @brief This function configures the output span for a specific DAC channel on
 * the LTC2672 device. It should be used when you need to adjust the
 * voltage range for a particular channel. The function requires a valid
 * device structure and valid span and channel parameters. It updates the
 * device's internal state to reflect the new span setting and ensures
 * that the channel's maximum current is adjusted accordingly. The
 * function must be called with valid span and channel values; otherwise,
 * it returns an error code.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param ch_span An enum value of type ltc2672_out_range representing the
 * desired output span. Must be a valid span value between
 * LTC2672_OFF and LTC2672_4800VREF.
 * @param out_ch An enum value of type ltc2672_dac_ch representing the DAC
 * channel to configure. Must be a valid channel value between
 * LTC2672_DAC0 and LTC2672_DAC4.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL if the span or channel is invalid.
 ******************************************************************************/
int ltc2672_set_span_channel(struct ltc2672_dev *, enum ltc2672_out_range,
			     enum ltc2672_dac_ch);

/***************************************************************************//**
 * @brief This function configures the output span for all channels of the
 * specified LTC2672 device. It should be called when you need to
 * uniformly set the output range across all DAC channels. The function
 * requires a valid device structure and a span value within the defined
 * range. If the span value is outside the valid range, the function
 * returns an error. The function updates the device's internal state to
 * reflect the new span settings and adjusts the maximum current for each
 * channel based on the selected span.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param ch_span An enum value of type ltc2672_out_range representing the
 * desired output span for all channels. Must be within the range
 * of defined output spans (LTC2672_OFF to LTC2672_4800VREF). If
 * the value is invalid, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the span is invalid
 * or if there is a communication failure with the device.
 ******************************************************************************/
int ltc2672_set_span_all_channels(struct ltc2672_dev *, enum ltc2672_out_range);

/***************************************************************************//**
 * @brief This function is used to power down the entire LTC2672 device, which
 * can be useful for reducing power consumption when the device is not in
 * use. It should be called when the device is no longer needed or before
 * shutting down the system to ensure that the device is in a low-power
 * state. The function requires a valid device descriptor that has been
 * initialized using the appropriate initialization function. It returns
 * an integer status code indicating the success or failure of the
 * operation.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device
 * to be powered down. This must be a valid, non-null pointer to a
 * properly initialized device descriptor. If the pointer is null
 * or invalid, the behavior is undefined.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ltc2672_chip_power_down(struct ltc2672_dev *);

/***************************************************************************//**
 * @brief Use this function to power down a specific DAC channel on an LTC2672
 * device. This is useful when you want to disable a channel to save
 * power or when the channel is not in use. The function should be called
 * with a valid device descriptor and a channel identifier. It is
 * important to ensure that the channel identifier is within the valid
 * range of available channels; otherwise, the function will return an
 * error. This function does not affect other channels or the device as a
 * whole.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param out_ch An enum value of type ltc2672_dac_ch representing the DAC
 * channel to power down. Valid values are LTC2672_DAC0 to
 * LTC2672_DAC4. If the value is outside this range, the function
 * returns an error.
 * @return Returns 0 on success. If the channel is invalid, returns -EINVAL.
 * Other error codes may be returned based on the underlying transaction
 * function.
 ******************************************************************************/
int ltc2672_power_down_channel(struct ltc2672_dev *, enum ltc2672_dac_ch);

/***************************************************************************//**
 * @brief This function is used to power down all digital-to-analog converter
 * (DAC) channels on a specified LTC2672 device. It should be called when
 * it is necessary to disable all channels, for instance, to save power
 * or reset the device state. The function iterates over each channel,
 * sending a power-down command. It is important to ensure that the
 * device has been properly initialized before calling this function. If
 * any channel fails to power down, the function will return an error
 * code corresponding to the failure.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device
 * to be controlled. This must not be null, and the device should
 * be properly initialized before calling this function.
 * @return Returns 0 on success, or a negative error code if any channel fails
 * to power down.
 ******************************************************************************/
int ltc2672_power_down_all_channels(struct ltc2672_dev *);

/***************************************************************************//**
 * @brief This function configures the MUX pin of the LTC2672 device to output a
 * selected measurement based on the provided multiplexer command. It
 * should be called when a specific measurement output is required from
 * the MUX pin. The function requires a valid device descriptor and a
 * multiplexer command within the defined range. If the command is
 * outside the valid range, the function returns an error. This function
 * is typically used in applications where monitoring of specific
 * voltages or currents is necessary.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param mux_comm An enum value of type ltc2672_mux_commands specifying the
 * desired MUX command. Valid values range from
 * LTC2672_MUX_DISABLED to LTC2672_MUX_VOUT4. If an invalid
 * value is provided, the function returns an error.
 * @return Returns 0 on success or a negative error code if the multiplexer
 * command is invalid or if the transaction fails.
 ******************************************************************************/
int ltc2672_monitor_mux(struct ltc2672_dev *, enum ltc2672_mux_commands);

/***************************************************************************//**
 * @brief This function sets up a specified DAC channel on the LTC2672 device to
 * toggle between two current settings, defined by `current_reg_a` and
 * `current_reg_b`. It is essential to ensure that the channel is not set
 * to an unsupported output range, such as `LTC2672_VMINUS_VREF` or
 * `LTC2672_OFF`, before calling this function. The function requires
 * valid current values that do not exceed the maximum allowable current
 * for the specified channel. It is typically used in applications where
 * dynamic current switching is needed. The function returns an error
 * code if the setup fails due to invalid parameters or communication
 * issues.
 *
 * @param device A pointer to an `ltc2672_dev` structure representing the
 * initialized LTC2672 device. Must not be null.
 * @param out_ch An enumerated value of type `ltc2672_dac_ch` specifying the DAC
 * channel to configure. Must be within the range of available
 * channels (LTC2672_DAC0 to LTC2672_DAC4).
 * @param current_reg_a A 32-bit unsigned integer representing the current
 * setting for Register A. Must be non-negative and within
 * the maximum current limit for the specified channel.
 * @param current_reg_b A 32-bit unsigned integer representing the current
 * setting for Register B. Must be non-negative and within
 * the maximum current limit for the specified channel.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * invalid parameters or communication errors.
 ******************************************************************************/
int ltc2672_setup_toggle_channel(struct ltc2672_dev *, enum ltc2672_dac_ch,
				 uint32_t, uint32_t);

/***************************************************************************//**
 * @brief This function is used to enable the toggle feature for specific DAC
 * channels on the LTC2672 device. It should be called when you want to
 * configure the device to allow toggling of the output state of the
 * specified channels. The function requires a valid device descriptor
 * and a mask indicating which channels to enable for toggling. The mask
 * must not exceed the maximum toggle mask value defined by the device.
 * If the mask is invalid, the function returns an error code.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param mask A 32-bit unsigned integer representing the bitmask of channels to
 * enable for toggling. Each bit corresponds to a channel, and the
 * mask must not exceed LTC2672_MAX_TOGGLE_MASK. If the mask is
 * invalid, the function returns an error.
 * @return Returns 0 on success or a negative error code if the mask is invalid
 * or if the transaction fails.
 ******************************************************************************/
int ltc2672_enable_toggle_channel(struct ltc2672_dev *, uint32_t);

/***************************************************************************//**
 * @brief This function is used to control the global toggle bit of an LTC2672
 * device, which can be used to enable or disable a global toggle feature
 * across all DAC channels. It should be called when there is a need to
 * change the global toggle state of the device. The function requires a
 * valid device structure that has been properly initialized. It returns
 * an integer status code indicating the success or failure of the
 * operation.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device.
 * Must not be null and should be initialized before calling this
 * function. The caller retains ownership.
 * @param is_enable A boolean value indicating whether to enable (true) or
 * disable (false) the global toggle feature.
 * @return Returns an integer status code from the ltc2672_transaction function,
 * indicating success or failure of the operation.
 ******************************************************************************/
int ltc2672_global_toggle(struct ltc2672_dev *, bool);

/***************************************************************************//**
 * @brief This function is used to configure the LTC2672 device by sending a
 * command with a specified mask. It should be called when a specific
 * configuration is needed for the device, using a mask that determines
 * the configuration settings. The function requires a valid device
 * structure and a mask value that does not exceed the maximum allowed
 * configuration mask. If the mask is invalid, the function returns an
 * error code. This function is typically used after the device has been
 * initialized and is ready for configuration.
 *
 * @param device A pointer to an ltc2672_dev structure representing the device
 * to be configured. Must not be null, and the device should be
 * properly initialized before calling this function.
 * @param mask An 8-bit unsigned integer representing the configuration mask.
 * Valid values range from 0 to LTC2672_MAX_CONFIG_MASK (15). If the
 * mask exceeds this range, the function returns an error.
 * @return Returns 0 on success or a negative error code if the mask is invalid
 * or if the transaction fails.
 ******************************************************************************/
int ltc2672_config_command(struct ltc2672_dev *, uint8_t);

#endif // __LTC2672_H__
