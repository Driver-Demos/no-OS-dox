/***************************************************************************//**
 *   @file   ad5766.h
 *   @brief  Header file of AD5766 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
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
#ifndef AD5766_H_
#define AD5766_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD5766_CMD_NOP_MUX_OUT		0x00
#define AD5766_CMD_SDO_CNTRL		0x01
#define AD5766_CMD_WR_IN_REG(x)		(0x10 | ((x) & 0xF))
#define AD5766_CMD_WR_DAC_REG(x)	(0x20 | ((x) & 0xF))
#define AD5766_CMD_SW_LDAC		0x30
#define AD5766_CMD_SPAN_REG		0x40
#define AD5766_CMD_WR_PWR_DAC		0x50
#define AD5766_CMD_WR_PWR_DITHER	0x51
#define AD5766_CMD_WR_DAC_REG_ALL	0x60
#define AD5766_CMD_SW_FULL_RESET	0x70
#define AD5766_CMD_READBACK_REG(x)	(0x80 | ((x) & 0xF))
#define AD5766_CMD_DITHER_SIG_1		0x90
#define AD5766_CMD_DITHER_SIG_2		0xA0
#define AD5766_CMD_INV_DITHER		0xB0
#define AD5766_CMD_DITHER_SCALE_1	0xC0
#define AD5766_CMD_DITHER_SCALE_2	0xD0

/* AD5766_CMD_SDO_CNTRL */
#define AD5766_SDO_EN			(1 << 0)

/* AD5766_CMD_SW_LDAC */
#define AD5766_LDAC(x)			(1 << ((x) & 0xF))

/* AD5766_CMD_SPAN_REG */
#define AD5766_CFG_CLR(x)		(((x) & 0x3) << 3)
#define AD5766_SPAN(x)			(((x) & 0x7) << 0)

/* AD5766_CMD_WR_PWR_DAC, AD5766_CMD_WR_PWR_DITHER */
#define AD5766_PWDN(x)			(1 << ((x) & 0xF))

/* AD5766_CMD_SW_FULL_RESET */
#define AD5766_RESET			0x1234

/* AD5766_CMD_DITHER_SIG_x */
#define AD5766_N0(x)			(1 << (2 * ((x) & 0xF)))
#define AD5766_N1(x)			(2 << (2 * ((x) & 0xF)))

/* AD5766_CMD_INV_DITHER */
#define AD5766_INV_D(x)			(1 << ((x) & 0xF))

/* AD5766_CMD_DITHER_SCALE_x */
#define AD5766_75(x)			(1 << (2 * ((x) & 0xF)))
#define AD5766_50(x)			(2 << (2 * ((x) & 0xF)))
#define AD5766_25(x)			(3 << (2 * ((x) & 0xF)))

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad5766_state` enumeration defines two possible states for the
 * AD5766 device: enabled and disabled. This enumeration is used to
 * manage the operational state of the device, allowing it to be turned
 * on or off as needed. It is a simple enumeration with two values,
 * providing a clear and straightforward way to control the device's
 * power state.
 *
 * @param AD5766_ENABLE Represents the enabled state of the AD5766 device.
 * @param AD5766_DISABLE Represents the disabled state of the AD5766 device.
 ******************************************************************************/
enum ad5766_state {
	AD5766_ENABLE,
	AD5766_DISABLE,
};

/***************************************************************************//**
 * @brief The `ad5766_dac` enumeration defines constants for each of the 16
 * digital-to-analog converter (DAC) channels available in the AD5766
 * device. Each enumerator corresponds to a specific DAC channel,
 * allowing for easy reference and manipulation of individual channels
 * within the device's driver code. This enumeration is crucial for
 * operations that involve selecting or configuring specific DAC channels
 * in the AD5766.
 *
 * @param AD5766_DAC_0 Represents the first DAC channel in the AD5766 device.
 * @param AD5766_DAC_1 Represents the second DAC channel in the AD5766 device.
 * @param AD5766_DAC_2 Represents the third DAC channel in the AD5766 device.
 * @param AD5766_DAC_3 Represents the fourth DAC channel in the AD5766 device.
 * @param AD5766_DAC_4 Represents the fifth DAC channel in the AD5766 device.
 * @param AD5766_DAC_5 Represents the sixth DAC channel in the AD5766 device.
 * @param AD5766_DAC_6 Represents the seventh DAC channel in the AD5766 device.
 * @param AD5766_DAC_7 Represents the eighth DAC channel in the AD5766 device.
 * @param AD5766_DAC_8 Represents the ninth DAC channel in the AD5766 device.
 * @param AD5766_DAC_9 Represents the tenth DAC channel in the AD5766 device.
 * @param AD5766_DAC_10 Represents the eleventh DAC channel in the AD5766
 * device.
 * @param AD5766_DAC_11 Represents the twelfth DAC channel in the AD5766 device.
 * @param AD5766_DAC_12 Represents the thirteenth DAC channel in the AD5766
 * device.
 * @param AD5766_DAC_13 Represents the fourteenth DAC channel in the AD5766
 * device.
 * @param AD5766_DAC_14 Represents the fifteenth DAC channel in the AD5766
 * device.
 * @param AD5766_DAC_15 Represents the sixteenth DAC channel in the AD5766
 * device.
 ******************************************************************************/
enum ad5766_dac {
	AD5766_DAC_0,
	AD5766_DAC_1,
	AD5766_DAC_2,
	AD5766_DAC_3,
	AD5766_DAC_4,
	AD5766_DAC_5,
	AD5766_DAC_6,
	AD5766_DAC_7,
	AD5766_DAC_8,
	AD5766_DAC_9,
	AD5766_DAC_10,
	AD5766_DAC_11,
	AD5766_DAC_12,
	AD5766_DAC_13,
	AD5766_DAC_14,
	AD5766_DAC_15,
};

/***************************************************************************//**
 * @brief The `ad5766_span` enumeration defines a set of voltage span
 * configurations for the AD5766 device, which is a digital-to-analog
 * converter (DAC). Each enumerator represents a specific voltage range
 * that the DAC can output, allowing for flexible configuration of the
 * output voltage range to suit different application requirements.
 *
 * @param AD5766_M_20V_TO_0V Represents a voltage span from -20V to 0V.
 * @param AD5766_M_16V_TO_0V Represents a voltage span from -16V to 0V.
 * @param AD5766_M_10V_TO_0V Represents a voltage span from -10V to 0V.
 * @param AD5766_M_12V_TO_P_14V Represents a voltage span from -12V to +14V.
 * @param AD5766_M_16V_TO_P_10V Represents a voltage span from -16V to +10V.
 * @param AD5766_M_5V_TO_P_6V Represents a voltage span from -5V to +6V.
 * @param AD5766_M_10V_TO_P_10V Represents a voltage span from -10V to +10V.
 ******************************************************************************/
enum ad5766_span {
	AD5766_M_20V_TO_0V,
	AD5766_M_16V_TO_0V,
	AD5766_M_10V_TO_0V,
	AD5766_M_12V_TO_P_14V,
	AD5766_M_16V_TO_P_10V,
	AD5766_M_5V_TO_P_6V,
	AD5766_M_10V_TO_P_10V,
};

/***************************************************************************//**
 * @brief The `ad5766_clr` enumeration defines the possible clear code settings
 * for the AD5766 device, which determine the output level when a clear
 * operation is performed. This enumeration is used to specify whether
 * the output should be set to zero, mid-scale, or full-scale, providing
 * flexibility in how the device handles clear operations.
 *
 * @param AD5766_ZERO Represents a clear code setting where the output is set to
 * zero.
 * @param AD5766_MID Represents a clear code setting where the output is set to
 * mid-scale.
 * @param AD5766_FULL Represents a clear code setting where the output is set to
 * full-scale.
 ******************************************************************************/
enum ad5766_clr {
	AD5766_ZERO,
	AD5766_MID,
	AD5766_FULL,
};

/***************************************************************************//**
 * @brief The `ad5766_dev` structure is a data structure used to represent and
 * manage the state of an AD5766 device, which is a digital-to-analog
 * converter (DAC). It contains pointers to SPI and GPIO descriptors for
 * communication and control, as well as a setting for enabling or
 * disabling daisy chaining of devices. This structure is essential for
 * interfacing with the AD5766 hardware, allowing for configuration and
 * operation of the DAC through software.
 *
 * @param spi_desc Pointer to a SPI descriptor used for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor used for resetting the device.
 * @param daisy_chain_en Enum indicating whether daisy chaining is enabled or
 * disabled.
 ******************************************************************************/
struct ad5766_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc		*gpio_reset;
	/* Device Settings */
	enum ad5766_state	daisy_chain_en;
};

/***************************************************************************//**
 * @brief The `ad5766_init_param` structure is used to initialize the AD5766
 * device, a high-performance digital-to-analog converter (DAC). It
 * contains parameters for SPI and GPIO initialization, as well as
 * various device settings such as daisy chain mode, clear code, output
 * voltage span, and power-down configurations for both DAC and dither
 * blocks. This structure is essential for setting up the device's
 * operational parameters before use.
 *
 * @param spi_init Defines the SPI initialization parameters.
 * @param gpio_reset Specifies the GPIO initialization parameters for the reset
 * pin.
 * @param daisy_chain_en Indicates whether the daisy chain mode is enabled or
 * disabled.
 * @param clr Specifies the clear code setting for the device.
 * @param span Defines the output voltage span setting for the device.
 * @param pwr_dac_setting Configures the power-down setting for the DAC
 * channels.
 * @param pwr_dither_setting Configures the power-down setting for the dither
 * block.
 * @param dither_signal_setting Sets the dither signal configuration for the
 * channels.
 * @param inv_dither_setting Configures the inversion setting for the dither
 * signal.
 * @param dither_scale_setting Sets the scaling factor for the dither signal.
 ******************************************************************************/
struct ad5766_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_reset;
	/* Device Settings */
	enum ad5766_state	daisy_chain_en;
	enum ad5766_clr		clr;
	enum ad5766_span	span;
	uint16_t		pwr_dac_setting;
	uint16_t		pwr_dither_setting;
	uint32_t		dither_signal_setting;
	uint16_t		inv_dither_setting;
	uint32_t		dither_scale_setting;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI command write to device. */
/***************************************************************************//**
 * @brief This function is used to send a command along with associated data to
 * the AD5766 device over the SPI interface. It is typically called when
 * a specific operation or configuration needs to be performed on the
 * device. The function requires a valid device structure that has been
 * properly initialized. The command and data are packed into a buffer
 * and transmitted using the SPI protocol. It is important to ensure that
 * the device is ready to receive commands before calling this function.
 *
 * @param dev A pointer to an ad5766_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param cmd A uint8_t value representing the command to be sent to the device.
 * This should be a valid command as defined by the device's
 * protocol.
 * @param data A uint16_t value representing the data to be sent along with the
 * command. The data should be formatted according to the command's
 * requirements.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * write operation. A non-negative value typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t ad5766_spi_cmd_write(struct ad5766_dev *dev,
			     uint8_t cmd,
			     uint16_t data);
/* SPI readback register from device. */
/***************************************************************************//**
 * @brief Use this function to read back a register value from a specific DAC
 * channel on the AD5766 device. This function should not be called when
 * the device is in Daisy-Chain mode, as it is not supported in that
 * configuration. Ensure that the device is properly initialized and not
 * in Daisy-Chain mode before calling this function. The function will
 * return an error if the operation is attempted in an unsupported mode.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null and the device must not be in Daisy-Chain
 * mode.
 * @param dac An enum value of type ad5766_dac specifying the DAC channel from
 * which to read. Must be a valid DAC channel as defined by the
 * ad5766_dac enumeration.
 * @param data A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A negative value indicates an error, such as attempting to
 * read in Daisy-Chain mode.
 ******************************************************************************/
int32_t ad5766_spi_readback_reg(struct ad5766_dev *dev,
				enum ad5766_dac dac,
				uint32_t *data);
/* Set software LDAC for the selected channels. */
/***************************************************************************//**
 * @brief This function is used to configure the software Load DAC (LDAC)
 * setting for the specified channels on the AD5766 device. It should be
 * called when you need to update the LDAC configuration, which controls
 * when the DAC registers are updated with new data. Ensure that the
 * device has been properly initialized before calling this function. The
 * function communicates with the device over SPI to apply the setting.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param setting A 16-bit value specifying the LDAC setting for the channels.
 * Each bit corresponds to a channel, and setting a bit enables
 * LDAC for that channel.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad5766_set_sw_ldac(struct ad5766_dev *dev,
			   uint16_t setting);
/* Set clear code and span settings. */
/***************************************************************************//**
 * @brief This function configures the clear code and span settings of the
 * AD5766 device, which are used to define the output voltage range and
 * behavior when a clear operation is triggered. It should be called
 * after the device has been initialized and before any operations that
 * depend on these settings. The function requires valid `clr` and `span`
 * enumerations to be provided, and it returns an error code if the
 * operation fails.
 *
 * @param dev A pointer to an initialized `ad5766_dev` structure representing
 * the device. Must not be null.
 * @param clr An enumeration value of type `ad5766_clr` specifying the clear
 * code setting. Valid values are `AD5766_ZERO`, `AD5766_MID`, and
 * `AD5766_FULL`.
 * @param span An enumeration value of type `ad5766_span` specifying the span
 * setting. Valid values range from `AD5766_M_20V_TO_0V` to
 * `AD5766_M_10V_TO_P_10V`.
 * @return Returns an `int32_t` error code, where 0 indicates success and a
 * negative value indicates failure.
 ******************************************************************************/
int32_t ad5766_set_clr_span(struct ad5766_dev *dev,
			    enum ad5766_clr clr,
			    enum ad5766_span span);
/* Power down the selected channels. */
/***************************************************************************//**
 * @brief This function is used to power down specific DAC channels on the
 * AD5766 device. It should be called when you need to reduce power
 * consumption by disabling unused channels. The function requires a
 * valid device structure and a setting that specifies which channels to
 * power down. Ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param setting A 16-bit value where each bit represents the power state of a
 * corresponding DAC channel. Valid values depend on the specific
 * channels you wish to power down.
 * @return Returns an int32_t indicating success or failure of the operation. A
 * non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5766_set_pwr_dac(struct ad5766_dev *dev,
			   uint16_t setting);
/* Power down the dither block for the selected channels. */
/***************************************************************************//**
 * @brief This function is used to power down the dither block for specific
 * channels on the AD5766 device. It should be called when you want to
 * disable the dither functionality for power saving or other operational
 * reasons. Ensure that the device has been properly initialized before
 * calling this function. The function communicates with the device over
 * SPI to apply the power-down setting.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param setting A 16-bit value specifying which channels' dither blocks to
 * power down. Each bit corresponds to a channel, where a set bit
 * indicates the channel's dither block should be powered down.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t ad5766_set_pwr_dither(struct ad5766_dev *dev,
			      uint16_t setting);
/* Enable the dither signal for the selected channels. */
/***************************************************************************//**
 * @brief This function is used to enable the dither signal on the AD5766 device
 * for the specified channels. It should be called when you need to apply
 * a dither signal to the output of the DAC channels to improve
 * performance or reduce quantization errors. The function requires a
 * valid device structure and a setting that specifies which channels to
 * apply the dither signal to. It is important to ensure that the device
 * has been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5766_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param setting A 32-bit unsigned integer specifying the channels to which the
 * dither signal should be applied. The lower 16 bits are used to
 * determine the channels, and invalid settings may result in
 * undefined behavior.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad5766_set_dither_signal(struct ad5766_dev *dev,
				 uint32_t setting);
/* Invert the dither signal for the selected channels. */
/***************************************************************************//**
 * @brief This function is used to invert the dither signal for the channels
 * specified by the setting parameter on an AD5766 device. It should be
 * called when you need to modify the dither signal behavior for specific
 * channels. Ensure that the device has been properly initialized before
 * calling this function. The function communicates with the device over
 * SPI to apply the specified setting.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param setting A 16-bit value specifying which channels' dither signals
 * should be inverted. Each bit corresponds to a channel, and the
 * function will invert the dither signal for channels where the
 * corresponding bit is set.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad5766_set_inv_dither(struct ad5766_dev *dev,
			      uint16_t setting);
/* Enable the dither scaling for the selected channels. */
/***************************************************************************//**
 * @brief This function is used to configure the dither scaling for the AD5766
 * device. It should be called when you need to adjust the dither scale
 * settings for specific channels on the device. The function requires a
 * valid device structure and a 32-bit setting value that specifies the
 * dither scale configuration. It is important to ensure that the device
 * has been properly initialized before calling this function. The
 * function returns an integer status code indicating success or failure
 * of the operation.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param setting A 32-bit unsigned integer representing the dither scale
 * settings. The lower 16 bits and the upper 16 bits are used to
 * configure different aspects of the dither scale. Invalid
 * settings may result in an error return value.
 * @return Returns an int32_t status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t ad5766_set_dither_scale(struct ad5766_dev *dev,
				uint32_t setting);
/* Do a software reset. */
/***************************************************************************//**
 * @brief Use this function to perform a software reset on the AD5766 device,
 * which resets the device to its default state. This function should be
 * called when a reset of the device is required, such as during
 * initialization or when recovering from an error state. Ensure that the
 * device structure is properly initialized before calling this function.
 * The function communicates with the device over SPI to issue the reset
 * command.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null. The caller retains ownership of the
 * structure.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad5766_do_soft_reset(struct ad5766_dev *dev);
/* Set the input register for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to set the input register of a specified DAC
 * channel on the AD5766 device. It should be called when you need to
 * update the input register with new data for a particular channel. The
 * function requires a valid device structure and a DAC channel
 * identifier. It is important to ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the operation fails.
 *
 * @param dev A pointer to an ad5766_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param dac An enum value of type ad5766_dac specifying the DAC channel to be
 * updated. Must be a valid channel identifier.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the input register. The valid range depends on the device's
 * configuration.
 * @return Returns an int32_t error code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad5766_set_in_reg(struct ad5766_dev *dev,
			  enum ad5766_dac dac,
			  uint16_t data);
/* Set the DAC register for the selected channel. */
/***************************************************************************//**
 * @brief This function is used to set the digital-to-analog converter (DAC)
 * register for a specific channel on the AD5766 device. It should be
 * called when you need to update the output value of a particular DAC
 * channel. Ensure that the device has been properly initialized before
 * calling this function. The function communicates with the device over
 * SPI to write the specified data to the DAC register of the selected
 * channel.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param dac An enum value of type ad5766_dac specifying the DAC channel to be
 * updated. Valid values are AD5766_DAC_0 to AD5766_DAC_15.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the DAC register. The valid range depends on the device's
 * configuration and the specific application requirements.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t ad5766_set_dac_reg(struct ad5766_dev *dev,
			   enum ad5766_dac dac,
			   uint16_t data);
/* Set the DAC register for all channels. */
/***************************************************************************//**
 * @brief This function sets the DAC register for all channels of the AD5766
 * device to the specified data value. It is typically used when a
 * uniform output is desired across all channels. The function must be
 * called with a valid device structure that has been properly
 * initialized. It communicates with the device over SPI to perform the
 * operation.
 *
 * @param dev A pointer to an initialized ad5766_dev structure representing the
 * device. Must not be null.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the DAC register of all channels. The valid range is 0 to 65535.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad5766_set_dac_reg_all(struct ad5766_dev *dev,
			       uint16_t data);
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD5766 device by setting up the
 * necessary SPI and GPIO configurations and applying the specified
 * device settings. It must be called before any other operations on the
 * AD5766 device. The function allocates memory for the device structure
 * and configures the device according to the provided initialization
 * parameters. If initialization is successful, the device pointer is
 * updated to point to the newly created device structure. The function
 * returns an error code if any step of the initialization fails, and the
 * caller is responsible for handling these errors appropriately.
 *
 * @param device A pointer to a pointer of type `struct ad5766_dev`. This will
 * be updated to point to the initialized device structure. Must
 * not be null.
 * @param init_param A structure of type `struct ad5766_init_param` containing
 * the initialization parameters for the device, including SPI
 * and GPIO settings, and various device-specific
 * configurations. All fields must be properly initialized
 * before calling this function.
 * @return Returns an `int32_t` error code: 0 on success, or a negative value if
 * an error occurs during initialization.
 ******************************************************************************/
int32_t ad5766_init(struct ad5766_dev **device,
		    struct ad5766_init_param init_param);

/* Free the resources allocated by ad5766_init(). */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD5766 device instance when it is no longer needed. This function
 * should be called to clean up after a successful call to ad5766_init(),
 * ensuring that all allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks in
 * your application.
 *
 * @param dev A pointer to an ad5766_dev structure representing the device
 * instance to be removed. Must not be null. The function will handle
 * null pointers gracefully by returning an error code.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource deallocation process. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during the removal
 * of resources.
 ******************************************************************************/
int32_t ad5766_remove(struct ad5766_dev *dev);
#endif // AD5766_H_
