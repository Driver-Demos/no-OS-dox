/***************************************************************************//**
 *   @file   LTC268X.h
 *   @brief  Header file of LTC2686/8 Driver
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
 ********************************************************************************
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
 *******************************************************************************/
#ifndef __LTC268X_H__
#define __LTC268X_H__

#include "no_os_spi.h"
#include "no_os_util.h"
#include "no_os_delay.h"
#include "errno.h"

/******************************************************************************/
/******************* Macros and Constants Definitions *************************/
/******************************************************************************/
#define LTC268X_CHANNEL_SEL(x, id)			(id ? x : (x << 1))

#define LTC268X_CMD_CH_CODE(x, id)			(0x00 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_SETTING(x, id)		(0x10 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_OFFSET(x, id)		(0X20 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_GAIN(x, id)			(0x30 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_CODE_UPDATE(x, id)		(0x40 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_CODE_UPDATE_ALL(x, id)	(0x50 + LTC268X_CHANNEL_SEL(x, id))
#define LTC268X_CMD_CH_UPDATE(x, id)		(0x60 + LTC268X_CHANNEL_SEL(x, id))

#define LTC268X_CMD_CONFIG_REG			0x70
#define LTC268X_CMD_POWERDOWN_REG		0x71
#define LTC268X_CMD_A_B_SELECT_REG		0x72
#define LTC268X_CMD_SW_TOGGLE_REG		0x73
#define LTC268X_CMD_TOGGLE_DITHER_EN_REG	0x74
#define LTC268X_CMD_MUX_CTRL_REG		0x75
#define LTC268X_CMD_FAULT_REG			0x76
#define LTC268X_CMD_CODE_ALL			0x78
#define LTC268X_CMD_CODE_UPDATE_ALL		0x79
#define LTC268X_CMD_SETTING_ALL			0x7A
#define LTC268X_CMD_SETTING_UPDATE_ALL		0x7B
#define LTC268X_CMD_UPDATE_ALL			0x7C
#define LTC268X_CMD_NOOP			0xFF

#define LTC268X_READ_OPERATION			0x80

/* Channel Settings */
#define LTC268X_CH_SPAN_MSK			NO_OS_GENMASK(3, 0)
#define LTC268X_CH_SPAN(x)			no_os_field_prep(LTC268X_CH_SPAN_MSK, x)
#define LTC268X_CH_TD_SEL_MSK			NO_OS_GENMASK(5, 4)
#define LTC268X_CH_TD_SEL(x)			no_os_field_prep(LTC268X_CH_TD_SEL_MSK, x)
#define LTC268X_CH_DIT_PER_MSK			NO_OS_GENMASK(8, 6)
#define LTC268X_CH_DIT_PER(x)			no_os_field_prep(LTC268X_CH_DIT_PER_MSK, x)
#define LTC268X_CH_DIT_PH_MSK			NO_OS_GENMASK(10, 9)
#define LTC268X_CH_DIT_PH(x)			no_os_field_prep(LTC268X_CH_DIT_PH_MSK, x)
#define LTC268X_CH_MODE				NO_OS_BIT(11)

/* Configuration register */
#define LTC268X_CONFIG_RST			NO_OS_BIT(15)

#define LTC268X_PWDN(x)				(1 << ((x) & 0xF))
#define LTC268X_DITH_EN(x)			(1 << ((x) & 0xF))

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ltc268x_voltage_range` enumeration defines a set of constants
 * representing different voltage ranges that can be used with the
 * LTC268X series of devices. Each enumerator corresponds to a specific
 * voltage range, allowing for easy selection and configuration of the
 * desired range in applications involving these devices.
 *
 * @param LTC268X_VOLTAGE_RANGE_0V_5V Represents a voltage range from 0V to 5V.
 * @param LTC268X_VOLTAGE_RANGE_0V_10V Represents a voltage range from 0V to
 * 10V.
 * @param LTC268X_VOLTAGE_RANGE_M5V_5V Represents a voltage range from -5V to
 * 5V.
 * @param LTC268X_VOLTAGE_RANGE_M10V_10V Represents a voltage range from -10V to
 * 10V.
 * @param LTC268X_VOLTAGE_RANGE_M15V_15V Represents a voltage range from -15V to
 * 15V.
 ******************************************************************************/
enum ltc268x_voltage_range {
	LTC268X_VOLTAGE_RANGE_0V_5V,
	LTC268X_VOLTAGE_RANGE_0V_10V,
	LTC268X_VOLTAGE_RANGE_M5V_5V,
	LTC268X_VOLTAGE_RANGE_M10V_10V,
	LTC268X_VOLTAGE_RANGE_M15V_15V,
};

/***************************************************************************//**
 * @brief The `ltc268x_span_tbl` structure is used to define a range or span
 * with a minimum and maximum value, likely for configuring or
 * representing voltage ranges or other parameters in the context of the
 * LTC268X series devices.
 *
 * @param min Represents the minimum value of the span.
 * @param max Represents the maximum value of the span.
 ******************************************************************************/
struct ltc268x_span_tbl {
	int min;
	int max;
};

/***************************************************************************//**
 * @brief The `ltc268x_dither_period` enumeration defines a set of constants
 * representing different dither periods for the LTC268X series of
 * devices. These constants are used to configure the dither period,
 * which is a parameter that affects the modulation of the output signal,
 * potentially improving the performance of digital-to-analog conversion
 * by spreading quantization noise over a wider frequency range.
 *
 * @param LTC268X_DITH_PERIOD_4 Represents a dither period of 4.
 * @param LTC268X_DITH_PERIOD_8 Represents a dither period of 8.
 * @param LTC268X_DITH_PERIOD_16 Represents a dither period of 16.
 * @param LTC268X_DITH_PERIOD_32 Represents a dither period of 32.
 * @param LTC268X_DITH_PERIOD_64 Represents a dither period of 64.
 ******************************************************************************/
enum ltc268x_dither_period {
	LTC268X_DITH_PERIOD_4,
	LTC268X_DITH_PERIOD_8,
	LTC268X_DITH_PERIOD_16,
	LTC268X_DITH_PERIOD_32,
	LTC268X_DITH_PERIOD_64
};

/***************************************************************************//**
 * @brief The `ltc268x_dither_phase` enumeration defines the possible phase
 * angles for dithering in the LTC268X series of devices. Dithering is a
 * technique used to reduce quantization error in digital-to-analog
 * conversion by adding a small amount of noise to the signal. This
 * enumeration allows the selection of one of four phase angles (0, 90,
 * 180, or 270 degrees) to be used in the dithering process, providing
 * flexibility in how the dithering is applied to the output signal.
 *
 * @param LTC268X_DITH_PHASE_0 Represents a dither phase of 0 degrees.
 * @param LTC268X_DITH_PHASE_90 Represents a dither phase of 90 degrees.
 * @param LTC268X_DITH_PHASE_180 Represents a dither phase of 180 degrees.
 * @param LTC268X_DITH_PHASE_270 Represents a dither phase of 270 degrees.
 ******************************************************************************/
enum ltc268x_dither_phase {
	LTC268X_DITH_PHASE_0,
	LTC268X_DITH_PHASE_90,
	LTC268X_DITH_PHASE_180,
	LTC268X_DITH_PHASE_270
};

/***************************************************************************//**
 * @brief The `ltc268x_a_b_register` is an enumeration that defines constants
 * for selecting between two registers, A and B, in the LTC268X series of
 * devices. This enumeration is used to specify which register to
 * interact with when configuring or controlling the device, allowing for
 * flexible register selection in the device's operation.
 *
 * @param LTC268X_SELECT_A_REG Represents the selection of register A in the
 * LTC268X device.
 * @param LTC268X_SELECT_B_REG Represents the selection of register B in the
 * LTC268X device.
 ******************************************************************************/
enum ltc268x_a_b_register {
	LTC268X_SELECT_A_REG,
	LTC268X_SELECT_B_REG
};

/***************************************************************************//**
 * @brief The `ltc268x_clk_input` enumeration defines the possible clock input
 * sources for the LTC268X device, which include a software toggle and
 * three hardware clock inputs. This enumeration is used to select the
 * clock input for toggling or dithering operations in the device,
 * allowing for flexible control over the timing and synchronization of
 * the device's operations.
 *
 * @param LTC268X_SOFT_TGL Represents a software toggle clock input.
 * @param LTC268X_TGP0 Represents the first hardware clock input.
 * @param LTC268X_TGP1 Represents the second hardware clock input.
 * @param LTC268X_TGP2 Represents the third hardware clock input.
 ******************************************************************************/
enum  ltc268x_clk_input {
	LTC268X_SOFT_TGL,
	LTC268X_TGP0,
	LTC268X_TGP1,
	LTC268X_TGP2
};

/***************************************************************************//**
 * @brief The `ltc268x_device_id` enumeration defines identifiers for different
 * models of the LTC268X series, specifically the LTC2686 and LTC2688.
 * These identifiers are used to distinguish between the two models in
 * the software, allowing for model-specific configurations and
 * operations.
 *
 * @param LTC2686 Represents the device ID for the LTC2686 model, assigned the
 * value 0.
 * @param LTC2688 Represents the device ID for the LTC2688 model, assigned the
 * value 1.
 ******************************************************************************/
enum ltc268x_device_id {
	LTC2686 = 0,
	LTC2688 = 1
};

/***************************************************************************//**
 * @brief The `ltc268x_dev` structure represents a device configuration for the
 * LTC268x series of digital-to-analog converters (DACs). It includes
 * fields for SPI communication, device identification, power-down
 * settings, dither configurations, DAC codes, and channel-specific
 * settings such as voltage range, dither phase, dither period, clock
 * input, and register selection. This structure is used to manage and
 * configure the operation of the DAC channels, allowing for precise
 * control over the output signals.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param dev_id Identifier for the specific LTC268x device variant.
 * @param pwd_dac_setting Power-down setting for the DAC.
 * @param dither_toggle_en Enables or disables dither toggling.
 * @param dither_mode Array indicating the dither mode for each channel.
 * @param dac_code Array holding the DAC code for each channel.
 * @param num_channels Number of channels available in the device.
 * @param crt_range Array specifying the voltage range for each channel.
 * @param dither_phase Array specifying the dither phase for each channel.
 * @param dither_period Array specifying the dither period for each channel.
 * @param clk_input Array specifying the clock input source for each channel.
 * @param reg_select Array specifying the register selection (A or B) for each
 * channel.
 ******************************************************************************/
struct ltc268x_dev {
	struct no_os_spi_desc			*spi_desc;
	enum ltc268x_device_id   dev_id;
	uint16_t			pwd_dac_setting;
	uint16_t			dither_toggle_en;
	bool				dither_mode[16];
	uint16_t			dac_code[16];
	uint8_t				num_channels;
	enum ltc268x_voltage_range 	crt_range[16];
	enum ltc268x_dither_phase	dither_phase[16];
	enum ltc268x_dither_period	dither_period[16];
	enum ltc268x_clk_input		clk_input[16];
	enum ltc268x_a_b_register	reg_select[16];
};

/***************************************************************************//**
 * @brief The `ltc268x_init_param` structure is used to initialize and configure
 * the LTC268x series of devices, which are digital-to-analog converters
 * (DACs). It includes parameters for SPI communication, device
 * identification, power-down settings, and various dither configurations
 * for up to 16 channels. Each channel can be individually configured
 * with specific voltage ranges, dither phases, periods, and clock
 * inputs, allowing for flexible and precise control of the DAC outputs.
 *
 * @param spi_init Initializes the SPI communication parameters.
 * @param dev_id Specifies the device ID for the LTC268x device.
 * @param pwd_dac_setting Defines the power-down setting for the DAC.
 * @param dither_toggle_en Enables or disables the dither toggle feature.
 * @param dither_mode Array indicating the dither mode for each channel.
 * @param crt_range Array specifying the voltage range for each channel.
 * @param dither_phase Array defining the dither phase for each channel.
 * @param dither_period Array specifying the dither period for each channel.
 * @param clk_input Array indicating the clock input source for each channel.
 * @param reg_select Array selecting the A or B register for each channel.
 ******************************************************************************/
struct ltc268x_init_param {
	/* SPI */
	struct no_os_spi_init_param 			spi_init;
	enum ltc268x_device_id          dev_id;
	uint16_t			pwd_dac_setting;
	uint16_t			dither_toggle_en;
	bool				dither_mode[16];
	enum ltc268x_voltage_range 	crt_range[16];
	enum ltc268x_dither_phase	dither_phase[16];
	enum ltc268x_dither_period	dither_period[16];
	enum ltc268x_clk_input		clk_input[16];
	enum ltc268x_a_b_register	reg_select[16];
};
/******************************************************************************/
/******************************** LTC268X *************************************/
/***************************************************************************//**
 * @brief Use this function to configure the power-down setting of the DAC on
 * the specified device. This function should be called when you need to
 * change the power state of the DAC, such as powering it down to save
 * energy when not in use. Ensure that the device structure is properly
 * initialized before calling this function. The function updates the
 * device's internal state to reflect the new power-down setting.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param setting A 16-bit unsigned integer representing the desired power-down
 * setting for the DAC. Valid values depend on the specific DAC
 * configuration and requirements.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ltc268x_set_pwr_dac(struct ltc268x_dev *dev, uint16_t setting);
/***************************************************************************//**
 * @brief This function configures the dither toggle setting on an LTC268x
 * device. It should be called when you need to enable or disable the
 * dither toggle feature, which is part of the device's configuration.
 * The function requires a valid device structure and a setting value,
 * and it updates the device's internal state to reflect the new dither
 * toggle configuration. Ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param setting A 16-bit unsigned integer representing the dither toggle
 * setting to be applied. Valid values depend on the device's
 * specifications.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ltc268x_set_dither_toggle(struct ltc268x_dev *dev, uint16_t setting);
/***************************************************************************//**
 * @brief Use this function to enable or disable the dither mode for a specific
 * channel on an LTC268x device. This function should be called after the
 * device has been properly initialized and configured. It updates the
 * internal state of the device to reflect the dither mode setting for
 * the specified channel. Ensure that the channel number is within the
 * valid range of available channels for the device. If the channel
 * number is invalid, the function returns an error code.
 *
 * @param dev A pointer to an initialized ltc268x_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure. Must be less than the number
 * of channels available on the device. If the channel is out of
 * range, the function returns -ENOENT.
 * @param en A boolean value indicating whether to enable (true) or disable
 * (false) the dither mode for the specified channel.
 * @return Returns 0 on success. If the channel is invalid, returns -ENOENT. If
 * an error occurs during SPI communication, returns a negative error
 * code.
 ******************************************************************************/
int32_t ltc268x_set_dither_mode(struct ltc268x_dev *dev, uint8_t channel,
				bool en);
/***************************************************************************//**
 * @brief Use this function to configure the voltage range for a specific
 * channel on an LTC268x device. This function should be called after the
 * device has been initialized and is typically used when setting up the
 * device for a specific application or adjusting its configuration. The
 * function checks if the specified channel is valid and updates the
 * device's settings accordingly. If the channel number is invalid, the
 * function returns an error code.
 *
 * @param dev A pointer to an initialized ltc268x_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure. Must be less than the number
 * of channels supported by the device. If invalid, the function
 * returns an error.
 * @param range An enum value of type ltc268x_voltage_range specifying the
 * desired voltage range for the channel.
 * @return Returns 0 on success, or a negative error code if the channel is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int32_t ltc268x_set_span(struct ltc268x_dev *dev, uint8_t channel,
			 enum ltc268x_voltage_range range);
/***************************************************************************//**
 * @brief Use this function to configure the dither phase of a specific channel
 * on an LTC268x device. This function is typically called after
 * initializing the device and when you need to adjust the dither phase
 * for signal processing purposes. Ensure that the channel number is
 * within the valid range of available channels on the device. The
 * function updates the device's internal settings and the local
 * configuration state. If the specified channel is invalid, the function
 * returns an error code.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param channel The channel number for which the dither phase is to be set.
 * Must be less than the number of channels available on the
 * device. If the channel is invalid, the function returns
 * -ENOENT.
 * @param phase An enum value of type ltc268x_dither_phase representing the
 * desired dither phase. Valid values are defined in the
 * ltc268x_dither_phase enumeration.
 * @return Returns 0 on success. If the channel is invalid, returns -ENOENT.
 * Other negative values indicate different errors.
 ******************************************************************************/
int32_t ltc268x_set_dither_phase(struct ltc268x_dev *dev, uint8_t channel,
				 enum  ltc268x_dither_phase phase);
/***************************************************************************//**
 * @brief This function configures the dither period for a specific channel on
 * an LTC268x device, which is useful for applications requiring precise
 * control over signal modulation. It should be called when the device is
 * initialized and the channel is within the valid range. The function
 * updates the device's internal settings and returns an error if the
 * channel number is invalid or if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ltc268x_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure, ranging from 0 to
 * dev->num_channels - 1. If the channel is out of range, the
 * function returns an error.
 * @param period An enum value of type ltc268x_dither_period specifying the
 * desired dither period. Valid values are defined in the
 * ltc268x_dither_period enumeration.
 * @return Returns 0 on success, a negative error code on failure, such as
 * -ENOENT if the channel is invalid.
 ******************************************************************************/
int32_t ltc268x_set_dither_period(struct ltc268x_dev *dev, uint8_t channel,
				  enum  ltc268x_dither_period period);
/***************************************************************************//**
 * @brief Use this function to configure the dither clock input for a specific
 * channel on the LTC268x device. This function should be called when you
 * need to change the clock input source for dither operations on a
 * particular channel. Ensure that the channel number is within the valid
 * range of available channels on the device. The function updates the
 * device configuration and stores the selected clock input in the device
 * structure. It returns an error if the channel number is invalid or if
 * the SPI update operation fails.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function.
 * @param channel The channel number for which the dither clock input is to be
 * selected. Must be less than the number of channels available
 * on the device. If the channel number is invalid, the function
 * returns -ENOENT.
 * @param clk_input An enum value of type ltc268x_clk_input representing the
 * desired clock input source. Valid values are
 * LTC268X_SOFT_TGL, LTC268X_TGP0, LTC268X_TGP1, and
 * LTC268X_TGP2.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -ENOENT for an invalid channel number or other negative values for
 * SPI update errors.
 ******************************************************************************/
int32_t ltc268x_select_tg_dith_clk(struct ltc268x_dev *dev, uint8_t channel,
				   enum  ltc268x_clk_input clk_input);
/***************************************************************************//**
 * @brief Use this function to select either the A or B register for a specific
 * channel on the LTC268x device. This function is typically called when
 * configuring the device to switch between different register settings
 * for a channel. It must be called with a valid channel number that is
 * within the range of available channels on the device. The function
 * updates the device's internal state to reflect the selected register.
 * If the channel number is invalid, the function returns an error code.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param channel The channel number for which the register selection is to be
 * made. Must be less than the number of channels available on
 * the device. If the channel number is invalid, the function
 * returns -ENOENT.
 * @param sel_reg An enum value of type ltc268x_a_b_register indicating whether
 * to select the A or B register for the specified channel.
 * @return Returns 0 on success. If the channel number is invalid, returns
 * -ENOENT. Other negative values may be returned to indicate different
 * errors.
 ******************************************************************************/
int32_t ltc268x_select_reg(struct ltc268x_dev *dev, uint8_t channel,
			   enum  ltc268x_a_b_register sel_reg);
/***************************************************************************//**
 * @brief Use this function to reset the LTC268x device to its default state via
 * a software command. This is typically used to ensure the device is in
 * a known state before configuration or after an error condition. The
 * function must be called with a valid device structure that has been
 * properly initialized. It is important to ensure that the device is not
 * in use by other operations when performing the reset to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device to
 * reset. This must not be null and should point to a properly
 * initialized device structure. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns an int32_t value indicating the success or failure of the
 * reset operation. A return value of 0 typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t ltc268x_software_reset(struct ltc268x_dev *dev);
/***************************************************************************//**
 * @brief Use this function to set the desired output voltage on a specific
 * channel of the LTC268x device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function calculates the appropriate DAC code based on the provided
 * voltage and updates the DAC's data register. It handles voltage values
 * that exceed the DAC's range by clamping them to the maximum allowable
 * value. This function is useful for applications requiring precise
 * voltage control on multiple channels.
 *
 * @param dev A pointer to an initialized ltc268x_dev structure representing the
 * device. Must not be null.
 * @param channel The DAC channel to set the voltage for. Must be a valid
 * channel number within the range supported by the device.
 * @param voltage The desired output voltage for the specified channel. The
 * function will clamp the voltage to the maximum allowable value
 * if it exceeds the DAC's range.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ltc268x_set_voltage(struct ltc268x_dev *dev, uint8_t channel,
			    float voltage);
/***************************************************************************//**
 * @brief Use this function to toggle the software state of a specific channel
 * on an LTC268x device. This function is typically called when you need
 * to change the state of a channel without altering the hardware
 * configuration. It is important to ensure that the channel number is
 * within the valid range of available channels on the device. If the
 * channel number is invalid, the function will return an error. This
 * function should be used in contexts where the device has been properly
 * initialized and is ready for operation.
 *
 * @param dev A pointer to an ltc268x_dev structure representing the device.
 * Must not be null, and the device must be initialized before
 * calling this function. The caller retains ownership.
 * @param channel An unsigned 8-bit integer representing the channel to toggle.
 * Must be less than the number of channels available on the
 * device. If the channel is out of range, the function returns
 * an error.
 * @return Returns 0 on success, or a negative error code if the channel is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int32_t ltc268x_software_toggle(struct ltc268x_dev *dev, uint8_t channel);
/***************************************************************************//**
 * @brief This function initializes an LTC268X device, setting up the SPI
 * communication and configuring the device according to the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure, initializes the SPI interface, performs a software reset,
 * and configures each channel based on the initialization parameters. If
 * any step fails, the function returns an error code and ensures that
 * allocated resources are freed. Successful initialization is indicated
 * by a non-negative return value.
 *
 * @param device A pointer to a pointer of type `struct ltc268x_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory upon successful initialization.
 * @param init_param A structure of type `struct ltc268x_init_param` containing
 * the initialization parameters for the device. This includes
 * SPI initialization parameters, device ID, power-down
 * settings, dither toggle settings, and channel-specific
 * configurations. The caller retains ownership of this
 * structure.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t ltc268x_init(struct ltc268x_dev **device,
		     struct ltc268x_init_param init_param);
/***************************************************************************//**
 * @brief This function is used to properly remove and clean up an instance of
 * an LTC268x device. It should be called when the device is no longer
 * needed to free associated resources. The function first checks if the
 * provided device pointer is valid. If the pointer is null, it returns
 * an error code indicating that the device is not found. Otherwise, it
 * proceeds to remove the SPI descriptor associated with the device and
 * frees the memory allocated for the device structure. This function
 * must be called to avoid memory leaks after the device is no longer in
 * use.
 *
 * @param dev A pointer to the ltc268x_dev structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -ENODEV indicating the device is not found. The caller is
 * responsible for ensuring the pointer is valid.
 * @return Returns an integer status code. A successful removal returns 0, while
 * a failure in removing the SPI descriptor returns a negative error
 * code.
 ******************************************************************************/
int32_t ltc268x_remove(struct ltc268x_dev *dev);

#endif // __LTC268X_H__
