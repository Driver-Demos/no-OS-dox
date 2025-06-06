/***************************************************************************//**
 *   @file   ltc6953.h
 *   @brief  Implementation of LTC6953 Driver.
 *   @author MTinaco (mariel.tinaco@analog.com)
********************************************************************************
 * Copyright 2023-2024(c) Analog Devices, Inc.
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

#ifndef __LTC6953_H__
#define __LTC6953_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_error.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define LTC6953_NUM_REGADDR		57

/* Register address macro */
#define LTC6953_REG_VCO_STATUS		0x00
#define LTC6953_REG_STAT		0x01
#define LTC6953_REG_PD_CTL		0x02
#define LTC6953_REG_CHAN_POWER_DOWN(x)	0x03 + (x / 4)
#define LTC6953_REG_TEMP		0x05
#define LTC6953_REG_SYNC_CONFIG		0x0B
#define LTC6953_REG_OUTPUT_DIVIDER(x)	0x0C + (x * 4)
#define LTC6953_REG_OUTPUT_CONFIG(x)	0x0D + (x * 4)
#define LTC6953_REG_DDEL_HIGH(x)	0x0D + (x * 4)
#define LTC6953_REG_DDEL_LOW(x)		0x0E + (x * 4)
#define LTC6953_REG_ADEL(x)		0x0F + (x * 4)
#define LTC6953_REG_CHIP_INFO		0x38

/* LTC6952_REG0 */
#define LTC6953_VCOOK_MSK		NO_OS_BIT(2)
#define LTC6953_NVCOOK_MSK		NO_OS_BIT(3)

/* LTC6952_REG1 */
#define LTC6953_INVSTAT_MSK		NO_OS_BIT(7)
#define LTC6953_STAT_OUT_MSK		NO_OS_GENMASK(6, 0)

/* LTC6952_REG2 */
#define LTC6953_PDALL_MSK		NO_OS_BIT(7)
#define LTC6953_PDVCOPK_MSK		NO_OS_BIT(5)
#define LTC6953_FILTV_MSK		NO_OS_BIT(1)
#define LTC6953_POR_MSK			NO_OS_BIT(0)

/* LTC6953 REG3, 4, 5 */
#define LTC6953_PD_MSK(ch)		NO_OS_GENMASK( \
						((ch) & NO_OS_GENMASK(1, 0)) * 2 + 1, \
						((ch) & NO_OS_GENMASK(1, 0)) * 2)
#define LTC6953_PD(ch, x)		no_os_field_prep(LTC6953_PD_MSK(ch), x)

/* LTC6953 REG5 */
#define LTC6953_TEMPO_MSK		NO_OS_BIT(7)

/* LTC6953 REG11 */
#define LTC6953_EZMD_MSK		NO_OS_BIT(4)
#define LTC6953_SRQMD_MSK		NO_OS_BIT(3)
#define LTC6953_SYSCT_MSK		NO_OS_GENMASK(2, 1)
#define LTC6953_SSRQ_MSK		NO_OS_BIT(0)

/* LTC6953 REG12,16,20,24,28,32,36,40,44,48,52 */
#define LTC6953_MP_MSK			NO_OS_GENMASK(7, 3)
#define LTC6953_MD_MSK			NO_OS_GENMASK(2, 0)

/* LTC6953_REG13,17,21,25,29,33,37,41,45,49,53 */
#define LTC6953_SRQEN_MSK		NO_OS_BIT(7)
#define LTC6953_MODE_MSK		NO_OS_GENMASK(6, 5)
#define LTC6953_OINV_MSK		NO_OS_BIT(4)
#define LTC6953_DDEL_HIGH_MSK		NO_OS_GENMASK(3, 0)

/* LTC6953_REG14,18,22,26,30,34,38,42,46,50,54 */
#define LTC6953_DDEL_LOW_MSK		NO_OS_GENMASK(7, 0)

/* LTC6953_REG15,19,23,27,31,35,39,43,47,51,55 */
#define LTC6953_ADEL_MSK		NO_OS_GENMASK(5, 0)

/* LTC6953_REG56 */
#define LTC6953_REV_MSK			NO_OS_GENMASK(7, 4)
#define LTC6953_PART_MSK		NO_OS_GENMASK(3, 0)

#define LTC6953_NUM_CHAN		11

#define LTC6953_OUT_DIV_MIN		1
#define LTC6953_OUT_DIV_MAX		1048576
#define LTC6953_X_MAX			127

#define LTC6953_ADDRX 			0
#define LTC6953_DXMSB 			1
#define LTC6953_NUMBITS 		2
#define LTC6953_R_ONLY 			3

/* Specifications */
#define LTC6953_SPI_WRITE_CMD		0x00
#define LTC6953_SPI_READ_CMD		0x01
#define LTC6953_SPI_ADDR_CMD(x)		((x) << 1)
#define LTC6953_BUFF_SIZE_BYTES		2
#define LTC6953_DUMMY_BYTES		0x00


/******************************************************************************/
/*************************** Types Definitions ********************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ltc6953_channel_spec` structure is used to define the
 * configuration for an output channel of the LTC6953 device. It includes
 * parameters such as the channel number, output divider, multiplier, and
 * divider settings, as well as digital and analog delay configurations.
 * Additionally, it specifies the SYSREF mode and power down mode for the
 * channel, and provides a field for an extended name, allowing for
 * descriptive labeling of each channel.
 *
 * @param num Specifies the channel number.
 * @param out_divider Defines the output divider value for the channel.
 * @param mp Represents the multiplier for the channel.
 * @param md Represents the divider for the channel.
 * @param digital_delay Specifies the digital delay setting for the channel.
 * @param analog_delay Specifies the analog delay setting for the channel.
 * @param sysref_mode Indicates the SYSREF mode configuration for the channel.
 * @param power_down_mode Specifies the power down mode for the channel.
 * @param extended_name Holds an extended name for the channel, up to 16
 * characters.
 ******************************************************************************/
struct ltc6953_channel_spec {
	uint8_t		num;
	uint8_t		out_divider;
	uint8_t		mp;
	uint8_t		md;
	uint8_t		digital_delay;
	uint8_t		analog_delay;
	uint8_t		sysref_mode;
	uint8_t		power_down_mode;
	int8_t   	extended_name[16];
};

/***************************************************************************//**
 * @brief The `ltc6953_init_param` structure is used to define the
 * initialization parameters for the LTC6953 device. It includes SPI
 * initialization parameters, an array of channel specifications for
 * configuring each channel of the LTC6953, and the VCO frequency
 * setting. This structure is essential for setting up the LTC6953 device
 * to operate according to the desired specifications and configurations.
 *
 * @param spi_init Contains the SPI initialization parameters.
 * @param chans An array of channel specifications for the LTC6953, with a size
 * defined by LTC6953_NUM_CHAN.
 * @param vco_frequency Specifies the VCO frequency for the LTC6953.
 ******************************************************************************/
struct ltc6953_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	spi_init;
	struct ltc6953_channel_spec chans[LTC6953_NUM_CHAN];
	float vco_frequency;
};

/***************************************************************************//**
 * @brief The `ltc6953_dev` structure is a device descriptor for the LTC6953, a
 * clock distribution IC. It encapsulates the necessary components for
 * interfacing with the device, including a SPI descriptor for
 * communication, an array of channel specifications to configure each of
 * the device's output channels, and a floating-point value representing
 * the VCO frequency. This structure is central to managing the
 * configuration and operation of the LTC6953 device within a system.
 *
 * @param spi_desc A pointer to the SPI descriptor used for communication with
 * the device.
 * @param chans An array of channel specifications for each of the LTC6953's
 * channels.
 * @param vco_frequency The frequency of the Voltage Controlled Oscillator (VCO)
 * in the device.
 ******************************************************************************/
struct ltc6953_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	struct ltc6953_channel_spec chans[LTC6953_NUM_CHAN];
	float vco_frequency;
};

/*****************************************************************************/
/************************* Functions Declarations ****************************/
/*****************************************************************************/

/***************************************************************************//**
 * @brief This function initializes the LTC6953 device using the provided
 * initialization parameters, setting up the necessary SPI communication
 * and configuring the device's VCO frequency. It must be called before
 * any other operations on the LTC6953 device. The function allocates
 * memory for the device structure and initializes the SPI interface. If
 * initialization fails at any step, it ensures proper cleanup of
 * resources. The caller is responsible for providing valid
 * initialization parameters and handling the device pointer upon
 * successful initialization.
 *
 * @param device A pointer to a pointer of type `struct ltc6953_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller takes ownership of the allocated memory and is
 * responsible for freeing it using `ltc6953_remove`.
 * @param init_param A pointer to a `struct ltc6953_init_param` containing the
 * initialization parameters for the LTC6953 device. Must not
 * be null. The structure should be properly populated with
 * valid SPI initialization parameters and VCO frequency
 * before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or SPI initialization failure.
 ******************************************************************************/
int ltc6953_init(struct ltc6953_dev **dev,
		 struct ltc6953_init_param *init_param);

/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an LTC6953 device when it is no longer needed. It
 * ensures that the SPI descriptor is removed and the memory allocated
 * for the device structure is freed. This function must be called after
 * the device is initialized and used, to prevent resource leaks. It is
 * important to check the return value to ensure that the resources were
 * successfully deallocated.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device to
 * be removed. This pointer must not be null, and it must point to a
 * valid, initialized device structure. The function will handle
 * invalid pointers by returning an error code.
 * @return Returns 0 on successful deallocation of resources, or a negative
 * error code if the SPI descriptor removal fails.
 ******************************************************************************/
int ltc6953_remove(struct ltc6953_dev *dev);

/***************************************************************************//**
 * @brief Use this function to reset the LTC6953 device to its default power-on
 * state. This is typically necessary when you want to ensure the device
 * is in a known state before starting configuration or operation. The
 * function should be called only after the device has been properly
 * initialized using the ltc6953_init function. It is important to handle
 * the return value to check for any errors during the reset process.
 *
 * @param device A pointer to an ltc6953_dev structure representing the device
 * to reset. Must not be null. The device should be initialized
 * before calling this function.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the reset process.
 ******************************************************************************/
int ltc6953_reset(struct ltc6953_dev *dev);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register address on the LTC6953 device via SPI communication. It is
 * essential to ensure that the device has been properly initialized
 * before calling this function. The function constructs the appropriate
 * command to write to the register and sends it over the SPI interface.
 * It is typically used when configuring or updating the settings of the
 * LTC6953 device. Proper error handling should be implemented by
 * checking the return value to ensure the write operation was
 * successful.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 8-bit address of the register to which data will be
 * written. Must be a valid register address for the LTC6953.
 * @param data The 16-bit data to be written to the specified register. The data
 * should be formatted according to the register's requirements.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the result of the SPI write operation.
 ******************************************************************************/
int ltc6953_write(struct ltc6953_dev *dev, uint8_t addr,
		  uint16_t data);

/***************************************************************************//**
 * @brief Use this function to read a single byte from a specific register of
 * the LTC6953 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * communicates with the device over SPI and retrieves the data from the
 * specified register address. It is important to handle the return value
 * to check for any communication errors that may occur during the SPI
 * transaction.
 *
 * @param dev A pointer to an initialized `ltc6953_dev` structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the device's address space.
 * @param data A pointer to a uint8_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int ltc6953_read(struct ltc6953_dev *dev, uint8_t addr,
		 uint8_t *data);

/***************************************************************************//**
 * @brief This function is used to update a specific register in the LTC6953
 * device by applying a mask to the current register value and then
 * setting the specified bits according to the provided data. It is
 * typically used when only certain bits of a register need to be
 * modified without affecting the other bits. The function first reads
 * the current value of the register, applies the mask to clear the bits
 * to be updated, and then sets the new data. It must be called with a
 * valid device descriptor and a register address within the valid range
 * for the LTC6953.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the LTC6953 device.
 * @param mask A bitmask indicating which bits in the register should be cleared
 * before setting the new data. Only the bits set in the mask will
 * be affected.
 * @param data The new data to be written to the register. Only the bits
 * specified by the mask will be updated with this data.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int ltc6953_update(struct ltc6953_dev *dev, uint8_t reg_addr,
		   uint8_t mask, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to power down or power up all channels of the
 * LTC6953 device. It should be called when there is a need to globally
 * control the power state of all channels, such as during device
 * initialization or shutdown procedures. The function requires a valid
 * device descriptor and a boolean flag indicating the desired power
 * state. It is important to ensure that the device has been properly
 * initialized before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * This must be a valid, initialized device descriptor and must not
 * be null.
 * @param is_pwdn A boolean value where 'true' indicates that all channels
 * should be powered down, and 'false' indicates that all
 * channels should be powered up.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation's result.
 ******************************************************************************/
int ltc6953_power_down_all(struct ltc6953_dev *dev, bool is_pwdn);

/***************************************************************************//**
 * @brief Use this function to control the state of the input filter on an
 * LTC6953 device. It is typically called when configuring the device to
 * ensure the input filter is in the desired state, either enabled or
 * disabled. This function should be called after the device has been
 * properly initialized. The function modifies the device's configuration
 * register to reflect the desired filter state.
 *
 * @param dev A pointer to an initialized `ltc6953_dev` structure representing
 * the device. Must not be null.
 * @param is_en A boolean value indicating whether to enable (`true`) or disable
 * (`false`) the input filter.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ltc6953_enable_filter(struct ltc6953_dev *dev, bool is_en);

/***************************************************************************//**
 * @brief Use this function to power down or power up the Voltage Controlled
 * Oscillator (VCO) of the LTC6953 device. This function is typically
 * called when you need to manage the power consumption of the device,
 * such as during low-power modes or when the VCO is not needed. Ensure
 * that the device has been properly initialized before calling this
 * function. The function modifies the power state based on the boolean
 * parameter provided, and it returns an integer status code indicating
 * the success or failure of the operation.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param is_pwdn A boolean value where 'true' powers down the VCO and 'false'
 * powers it up.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ltc6953_power_down_vco(struct ltc6953_dev *dev, bool is_pwdn);

/***************************************************************************//**
 * @brief Use this function to configure the output divider for a specific
 * channel on the LTC6953 device. This function is essential when you
 * need to adjust the frequency division for a particular output channel.
 * It must be called with a valid channel number and a desired divider
 * value. The function will return an error if the channel number exceeds
 * the maximum allowed value. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param channel The channel number for which the output divider is to be set.
 * Valid range is 0 to 10. Values outside this range will result
 * in an error.
 * @param divider The desired output divider value. Must be a valid divider
 * supported by the device. Invalid values will not set the
 * divider and may result in undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the channel number is invalid.
 ******************************************************************************/
int ltc6953_set_output_divider(struct ltc6953_dev *dev, uint32_t channel,
			       uint32_t divider);

/***************************************************************************//**
 * @brief This function configures the power mode for a specified channel on the
 * LTC6953 device. It should be used when you need to change the power
 * state of a particular channel, such as powering it down or setting it
 * to a specific operational mode. The function requires that the device
 * has been properly initialized before calling. It validates the mode
 * and channel parameters to ensure they are within acceptable ranges,
 * returning an error code if they are not. This function is useful for
 * managing power consumption and operational states of individual
 * channels on the device.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure, ranging from 0 to 10. Values
 * outside this range will result in an error.
 * @param mode The power mode to set for the channel, ranging from 0 to 3.
 * Values outside this range will result in an error.
 * @return Returns 0 on success or a negative error code if the input parameters
 * are invalid.
 ******************************************************************************/
int ltc6953_power_mode(struct ltc6953_dev *dev, uint32_t channel,
		       int32_t mode);

/***************************************************************************//**
 * @brief Use this function to enable or disable synchronization for a specific
 * channel on the LTC6953 device. This function is typically called when
 * configuring the device to ensure that the desired channels are
 * synchronized according to the application's requirements. It is
 * important to ensure that the channel number is within the valid range
 * before calling this function. If the channel number exceeds the
 * maximum allowed value, the function will return an error code.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param channel The channel number to configure for synchronization. Valid
 * range is 0 to 10. If the channel number is greater than 10,
 * the function returns an error.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) synchronization for the specified channel.
 * @return Returns 0 on success or a negative error code if the channel number
 * is invalid.
 ******************************************************************************/
int ltc6953_enable_sync(struct ltc6953_dev *dev, uint32_t channel,
			bool enable);

/***************************************************************************//**
 * @brief Use this function to configure the digital delay for a specific
 * channel on the LTC6953 device. This function is typically called when
 * you need to adjust the timing of the output signal for synchronization
 * purposes. Ensure that the device has been properly initialized before
 * calling this function. The function checks that the provided delay is
 * within the valid range of 0 to 4095 and that the channel number is
 * within the valid range of 0 to 10. If these conditions are not met,
 * the function returns an error code.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param channel The channel number for which the digital delay is to be set.
 * Valid range is 0 to 10. Values outside this range result in an
 * error.
 * @param delay The digital delay value to set, in the range 0 to 4095. Values
 * outside this range result in an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int ltc6953_set_digital_delay(struct ltc6953_dev *dev, uint32_t channel,
			      uint16_t delay);

/***************************************************************************//**
 * @brief Use this function to configure the analog delay for a specific channel
 * on the LTC6953 device. This function should be called when you need to
 * adjust the timing characteristics of a channel's output. Ensure that
 * the device has been properly initialized before calling this function.
 * The function validates the delay and channel parameters, returning an
 * error if they are out of range. It is important to handle these error
 * codes in your application to ensure reliable operation.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure, ranging from 0 to 10. Values
 * outside this range will result in an error.
 * @param delay The desired analog delay value, ranging from 0 to 63. Values
 * outside this range will result in an error.
 * @return Returns 0 on success or a negative error code (e.g., -EINVAL) if the
 * input parameters are invalid.
 ******************************************************************************/
int ltc6953_set_analog_delay(struct ltc6953_dev *dev, uint32_t channel,
			     uint16_t delay);

/***************************************************************************//**
 * @brief Use this function to configure the mode of a specific channel on the
 * LTC6953 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function checks
 * for valid mode and channel values, returning an error if they are out
 * of range. This function is useful when you need to change the
 * operational mode of a channel, such as during device configuration or
 * reconfiguration.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to configure, ranging from 0 to 10. Values
 * outside this range will result in an error.
 * @param mode The mode to set for the specified channel, with valid values
 * ranging from 0 to 3. Values outside this range will result in an
 * error.
 * @return Returns 0 on success or a negative error code (e.g., -EINVAL) if the
 * input parameters are invalid.
 ******************************************************************************/
int ltc6953_set_mode(struct ltc6953_dev *dev, uint32_t channel,
		     uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to control the inversion of the output signal for a
 * specific channel on the LTC6953 device. This function is typically
 * called when configuring the output characteristics of the device. It
 * is important to ensure that the channel number is within the valid
 * range of 0 to 10, as values outside this range will result in an
 * error. The function must be called with a valid device descriptor that
 * has been properly initialized.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * This must be a valid, initialized device descriptor. The caller
 * retains ownership and must ensure it is not null.
 * @param channel The channel number for which to set the output inversion.
 * Valid values are from 0 to 10. If the channel number is
 * greater than 10, the function returns an error.
 * @param is_invert A boolean value indicating whether to enable (true) or
 * disable (false) output inversion for the specified channel.
 * @return Returns 0 on success. If the channel number is invalid, returns
 * -EINVAL.
 ******************************************************************************/
int ltc6953_invert_output(struct ltc6953_dev *dev, uint32_t channel,
			  bool is_invert);

/***************************************************************************//**
 * @brief Use this function to enable or disable the EZSYNC mode on an LTC6953
 * device. This function should be called when you need to configure the
 * synchronization mode of the device, typically during initialization or
 * when changing operational modes. Ensure that the device has been
 * properly initialized before calling this function. The function
 * modifies the device's configuration register to reflect the desired
 * EZSYNC mode state.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param is_en A boolean value indicating whether to enable (true) or disable
 * (false) the EZSYNC mode.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int ltc6953_ezsync_mode(struct ltc6953_dev *dev, bool is_en);

/***************************************************************************//**
 * @brief Use this function to enable or disable the SRQ mode on an LTC6953
 * device. This function should be called when you need to configure the
 * synchronization request mode of the device. Ensure that the device has
 * been properly initialized before calling this function. The function
 * modifies the device's configuration based on the provided boolean
 * parameter, affecting how the device handles synchronization requests.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param is_en A boolean value indicating whether to enable (true) or disable
 * (false) the SRQ mode.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ltc6953_sync_mode(struct ltc6953_dev *dev, bool is_en);

/***************************************************************************//**
 * @brief This function configures the SSRQ (Synchronous Serial Request) mode of
 * the LTC6953 device. It should be called when you need to enable or
 * disable the SSRQ mode, which is part of the device's synchronization
 * configuration. Ensure that the device has been properly initialized
 * before calling this function. The function modifies the device's
 * configuration based on the boolean parameter provided, and it returns
 * an integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * This must not be null and should point to a valid, initialized
 * device descriptor.
 * @param is_en A boolean value indicating whether to enable (true) or disable
 * (false) the SSRQ mode. The function will update the device
 * configuration based on this value.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ltc6953_ssrq_mode(struct ltc6953_dev *dev, bool is_en);

/***************************************************************************//**
 * @brief This function configures the number of SYSCT pulses for the LTC6953
 * device, which is used to control synchronization behavior. It should
 * be called when the device is initialized and ready to accept
 * configuration changes. The function expects a valid pulse count within
 * a specific range and will return an error if the input is outside this
 * range.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param num_pulse An unsigned 8-bit integer representing the number of SYSCT
 * pulses. Valid values are 0 to 3. Values outside this range
 * will result in an error.
 * @return Returns 0 on success or a negative error code if the input is
 * invalid.
 ******************************************************************************/
int ltc6953_num_pulse(struct ltc6953_dev *dev, uint8_t num_pulse);

/***************************************************************************//**
 * @brief Use this function to control the temperature measurement feature of
 * the LTC6953 device, which is accessible via the STAT pin. This
 * function should be called when you need to enable or disable the
 * temperature status output, typically after the device has been
 * initialized. It modifies the device's configuration to reflect the
 * desired state of temperature measurement. Ensure that the device
 * pointer is valid before calling this function.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param is_en A boolean value where 'true' enables temperature measurement and
 * 'false' disables it.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ltc6953_enable_temp_stat(struct ltc6953_dev *dev, bool is_en);

/***************************************************************************//**
 * @brief Use this function to determine the status of the VCO input on an
 * LTC6953 device. It should be called when you need to verify if the VCO
 * input is functioning correctly. The function requires a valid device
 * descriptor and a pointer to a boolean variable where the status will
 * be stored. Ensure that the device has been properly initialized before
 * calling this function. The function will return an error code if the
 * read operation fails.
 *
 * @param dev A pointer to an initialized `ltc6953_dev` structure representing
 * the device. Must not be null.
 * @param is_ok A pointer to a boolean variable where the VCO status will be
 * stored. Must not be null. The variable will be set to true if
 * the VCO input is okay, or false otherwise.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc6953_vco_status(struct ltc6953_dev *dev, bool *is_ok);

/***************************************************************************//**
 * @brief This function is used to obtain the status of the INVSTAT bit from the
 * LTC6953 device, which indicates a specific condition or state of the
 * device. It should be called when the user needs to check this
 * particular status bit. The function requires a valid device descriptor
 * and a pointer to a boolean variable where the status will be stored.
 * It is important to ensure that the device has been properly
 * initialized before calling this function. The function will return an
 * error code if the read operation fails.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * This must be a valid, initialized device descriptor and must not
 * be null.
 * @param status A pointer to a boolean variable where the function will store
 * the status of the INVSTAT bit. This pointer must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc6953_get_invstat(struct ltc6953_dev *dev, bool *status);

/***************************************************************************//**
 * @brief Use this function to set the INVSTAT bit of the LTC6953 device, which
 * is typically used to control the inversion status of the device's
 * output. This function should be called when you need to change the
 * inversion status of the output signal. Ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the operation fails.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param status A boolean value indicating the desired state of the INVSTAT
 * bit. 'true' sets the bit, while 'false' clears it.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ltc6953_set_invstat(struct ltc6953_dev *dev, bool status);

/***************************************************************************//**
 * @brief This function sets the X value for the specified LTC6953 device. It
 * should be used when you need to configure the device with a specific X
 * value, which must be within the valid range. The function checks if
 * the provided X value is within the allowed range and returns an error
 * if it is not. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ltc6953_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param x An 8-bit unsigned integer representing the X value to set. Valid
 * range is 0 to LTC6953_X_MAX (127). Values outside this range will
 * result in an error.
 * @return Returns 0 on success or a negative error code if the X value is
 * invalid or if there is an issue updating the device.
 ******************************************************************************/
int ltc6953_set_x(struct ltc6953_dev *dev, uint8_t x);

/***************************************************************************//**
 * @brief Use this function to obtain the current value of the X register from
 * an initialized LTC6953 device. This function should be called after
 * the device has been properly initialized and configured. It reads the
 * status register of the device and extracts the relevant bits
 * corresponding to the X value. Ensure that the device pointer is valid
 * and that the output pointer is not null to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized `ltc6953_dev` structure representing
 * the device. Must not be null.
 * @param x A pointer to a uint8_t where the retrieved X value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc6953_get_x(struct ltc6953_dev *dev, uint8_t *x);

/***************************************************************************//**
 * @brief Use this function to obtain the revision number of an LTC6953 device.
 * It is typically called after the device has been successfully
 * initialized to verify or log the hardware revision. The function reads
 * the relevant register and extracts the revision information. Ensure
 * that the device pointer is valid and that the revision pointer is not
 * null before calling this function.
 *
 * @param dev A pointer to an initialized ltc6953_dev structure representing the
 * device. Must not be null.
 * @param rev A pointer to a uint8_t variable where the revision number will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc6953_read_rev(struct ltc6953_dev *dev, uint8_t *rev);

/***************************************************************************//**
 * @brief This function is used to obtain the part number of an LTC6953 device.
 * It should be called when the part number is needed for identification
 * or verification purposes. The function requires a valid device
 * descriptor and a pointer to store the part number. It reads the
 * relevant register and extracts the part number, returning it through
 * the provided pointer. The function must be called after the device has
 * been successfully initialized. If the read operation fails, the
 * function returns an error code.
 *
 * @param dev A pointer to an initialized `ltc6953_dev` structure representing
 * the device. Must not be null.
 * @param part A pointer to a uint8_t where the part number will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc6953_read_part(struct ltc6953_dev *dev, uint8_t *part);

#endif // __LTC6953_H__
