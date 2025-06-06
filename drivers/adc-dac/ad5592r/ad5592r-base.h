/***************************************************************************//**
 *   @file   ad5592r-base.h
 *   @brief  Header file of AD5592R Base Driver.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
 * Copyright 2018, 2020, 2025(c) Analog Devices, Inc.
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
#ifndef AD5592R_BASE_H_
#define AD5592R_BASE_H_

#include "stdint.h"
#include "no_os_delay.h"
#include "no_os_spi.h"
#include "no_os_i2c.h"
#include "no_os_util.h"
#include "no_os_alloc.h"
#include <stdbool.h>

#define CH_MODE_UNUSED			0
#define CH_MODE_ADC			1
#define CH_MODE_DAC			2
#define CH_MODE_DAC_AND_ADC		3
#define CH_MODE_GPI			4
#define CH_MODE_GPO			5

#define CH_OFFSTATE_PULLDOWN		0
#define CH_OFFSTATE_OUT_LOW		1
#define CH_OFFSTATE_OUT_HIGH		2
#define CH_OFFSTATE_OUT_TRISTATE	3

/***************************************************************************//**
 * @brief The `ad5592r_registers` enumeration defines a set of constants
 * representing the various registers available in the AD5592R device.
 * Each enumerator corresponds to a specific register used for different
 * functionalities such as DAC readback, ADC sequencing, control
 * operations, enabling ADC/DAC, GPIO configurations, and power
 * management. These constants are used to facilitate communication and
 * control of the AD5592R device by providing a clear and concise way to
 * reference each register by name rather than by its numeric value.
 *
 * @param AD5592R_REG_NOOP Represents a no-operation command with a value of
 * 0x0.
 * @param AD5592R_REG_DAC_READBACK Used for DAC readback operations with a value
 * of 0x1.
 * @param AD5592R_REG_ADC_SEQ Represents the ADC sequence register with a value
 * of 0x2.
 * @param AD5592R_REG_CTRL Control register with a value of 0x3.
 * @param AD5592R_REG_ADC_EN ADC enable register with a value of 0x4.
 * @param AD5592R_REG_DAC_EN DAC enable register with a value of 0x5.
 * @param AD5592R_REG_PULLDOWN Pulldown register with a value of 0x6.
 * @param AD5592R_REG_LDAC LDAC register with a value of 0x7.
 * @param AD5592R_REG_GPIO_OUT_EN GPIO output enable register with a value of
 * 0x8.
 * @param AD5592R_REG_GPIO_SET GPIO set register with a value of 0x9.
 * @param AD5592R_REG_GPIO_IN_EN GPIO input enable register with a value of 0xA.
 * @param AD5592R_REG_PD Power down register with a value of 0xB.
 * @param AD5592R_REG_OPEN_DRAIN Open drain register with a value of 0xC.
 * @param AD5592R_REG_TRISTATE Tristate register with a value of 0xD.
 * @param AD5592R_REG_RESET Reset register with a value of 0xF.
 ******************************************************************************/
enum ad5592r_registers {
	AD5592R_REG_NOOP		= 0x0,
	AD5592R_REG_DAC_READBACK	= 0x1,
	AD5592R_REG_ADC_SEQ		= 0x2,
	AD5592R_REG_CTRL		= 0x3,
	AD5592R_REG_ADC_EN		= 0x4,
	AD5592R_REG_DAC_EN		= 0x5,
	AD5592R_REG_PULLDOWN		= 0x6,
	AD5592R_REG_LDAC		= 0x7,
	AD5592R_REG_GPIO_OUT_EN		= 0x8,
	AD5592R_REG_GPIO_SET		= 0x9,
	AD5592R_REG_GPIO_IN_EN		= 0xA,
	AD5592R_REG_PD			= 0xB,
	AD5592R_REG_OPEN_DRAIN		= 0xC,
	AD5592R_REG_TRISTATE		= 0xD,
	AD5592R_REG_RESET		= 0xF,
};

#define AD5592R_REG_PD_PD_ALL			    NO_OS_BIT(10)
#define AD5592R_REG_PD_EN_REF			    NO_OS_BIT(9)

#define AD5592R_REG_CTRL_ADC_PC_BUFF		    NO_OS_BIT(9)
#define AD5592R_REG_CTRL_ADC_BUFF_EN		    NO_OS_BIT(8)
#define AD5592R_REG_CTRL_CONFIG_LOCK		    NO_OS_BIT(7)
#define AD5592R_REG_CTRL_W_ALL_DACS		    NO_OS_BIT(6)
#define AD5592R_REG_CTRL_ADC_RANGE		    NO_OS_BIT(5)
#define AD5592R_REG_CTRL_DAC_RANGE		    NO_OS_BIT(4)

#define AD5592R_REG_ADC_SEQ_REP			    NO_OS_BIT(9)
#define AD5592R_REG_ADC_SEQ_TEMP_READBACK	    NO_OS_BIT(8)
#define AD5592R_REG_ADC_SEQ_CODE_MSK(x)		    ((x) & 0x0FFF)

#define AD5592R_REG_GPIO_OUT_EN_ADC_NOT_BUSY	    NO_OS_BIT(8)

#define AD5592R_REG_LDAC_IMMEDIATE_OUT		    0x00
#define AD5592R_REG_LDAC_INPUT_REG_ONLY		    0x01
#define AD5592R_REG_LDAC_INPUT_REG_OUT		    0x02

#define INTERNAL_VREF_VOLTAGE			    2.5

#define NUM_OF_CHANNELS 8

struct ad5592r_dev;

/***************************************************************************//**
 * @brief The `ad5592r_rw_ops` structure defines a set of function pointers for
 * performing read and write operations on the AD5592R device. These
 * operations include writing to and reading from DAC and ADC channels,
 * as well as accessing device registers and GPIO pins. This structure is
 * essential for abstracting the hardware-specific operations, allowing
 * for flexible and reusable code when interfacing with the AD5592R
 * device.
 *
 * @param write_dac Function pointer for writing a value to a DAC channel.
 * @param read_adc Function pointer for reading a value from an ADC channel.
 * @param multi_read_adc Function pointer for reading values from multiple ADC
 * channels.
 * @param reg_write Function pointer for writing a value to a device register.
 * @param reg_read Function pointer for reading a value from a device register.
 * @param gpio_read Function pointer for reading GPIO values from the device.
 ******************************************************************************/
struct ad5592r_rw_ops {
	int32_t (*write_dac)(struct ad5592r_dev *dev, uint8_t chan,
			     uint16_t value);
	int32_t (*read_adc)(struct ad5592r_dev *dev, uint8_t chan,
			    uint16_t *value);
	int32_t(*multi_read_adc)(struct ad5592r_dev *dev,
				 uint16_t chans, uint16_t *value);
	int32_t (*reg_write)(struct ad5592r_dev *dev, uint8_t reg,
			     uint16_t value);
	int32_t (*reg_read)(struct ad5592r_dev *dev, uint8_t reg,
			    uint16_t *value);
	int32_t (*gpio_read)(struct ad5592r_dev *dev, uint8_t *value);
};

/***************************************************************************//**
 * @brief The `ad559xr_range` enumeration defines two possible voltage ranges
 * for the AD5592R device's ADC and DAC operations. These ranges are used
 * to configure the device to operate within a specified voltage range,
 * either from zero to the reference voltage (VREF) or from zero to twice
 * the reference voltage (2VREF). This configuration is crucial for
 * ensuring that the device operates correctly within the desired voltage
 * limits.
 *
 * @param ZERO_TO_VREF Represents a range from zero to the reference voltage
 * (VREF).
 * @param ZERO_TO_2VREF Represents a range from zero to twice the reference
 * voltage (2VREF).
 ******************************************************************************/
enum ad559xr_range {
	ZERO_TO_VREF,
	ZERO_TO_2VREF
};

/***************************************************************************//**
 * @brief The `ad5592r_init_param` structure is used to initialize the AD5592R
 * device, which is a configurable analog-to-digital and digital-to-
 * analog converter. It contains parameters for setting up the internal
 * reference, SPI and I2C communication interfaces, channel modes and
 * off-states, ADC and DAC ranges, ADC buffer status, and power-down
 * states for each channel. This structure allows for flexible
 * configuration of the device's operational parameters to suit various
 * application needs.
 *
 * @param int_ref Indicates whether the internal reference is used.
 * @param spi_init Pointer to SPI initialization parameters.
 * @param i2c_init Pointer to I2C initialization parameters.
 * @param channel_modes Array defining the mode for each of the 8 channels.
 * @param channel_offstate Array defining the off-state for each of the 8
 * channels.
 * @param adc_range Specifies the ADC range setting.
 * @param dac_range Specifies the DAC range setting.
 * @param adc_buf Indicates whether the ADC buffer is enabled.
 * @param power_down Array indicating the power-down state for each of the 8
 * channels.
 ******************************************************************************/
struct ad5592r_init_param {
	bool int_ref;
	struct no_os_spi_init_param *spi_init;
	struct no_os_i2c_init_param *i2c_init;
	uint8_t channel_modes[8];
	uint8_t channel_offstate[8];
	enum ad559xr_range adc_range;
	enum ad559xr_range dac_range;
	bool adc_buf;
	uint8_t power_down[8];
};

/***************************************************************************//**
 * @brief The `ad5592r_dev` structure represents a device instance for the
 * AD5592R, a versatile mixed-signal device with ADC, DAC, and GPIO
 * capabilities. It encapsulates configuration and state information
 * necessary for operating the device, including communication
 * descriptors (I2C and SPI), channel configurations, cached values for
 * DAC and control registers, and settings for ADC and DAC ranges. The
 * structure also manages GPIO states and modes, power-down
 * configurations, and the use of internal references, providing a
 * comprehensive interface for interacting with the AD5592R hardware.
 *
 * @param ops Pointer to a structure containing read/write operations for the
 * device.
 * @param i2c Pointer to an I2C descriptor for communication.
 * @param spi Pointer to an SPI descriptor for communication.
 * @param spi_msg 16-bit message buffer for SPI communication.
 * @param num_channels Number of channels available on the device.
 * @param cached_dac Array storing cached DAC values for each channel.
 * @param cached_gp_ctrl Cached general-purpose control register value.
 * @param channel_modes Array defining the mode of each channel (e.g., ADC,
 * DAC).
 * @param channel_offstate Array defining the off-state configuration for each
 * channel.
 * @param gpio_out Current GPIO output state.
 * @param gpio_in Current GPIO input state.
 * @param gpio_val Current GPIO value.
 * @param ldac_mode Mode for the LDAC pin operation.
 * @param adc_range ADC range setting for the device.
 * @param dac_range DAC range setting for the device.
 * @param int_ref Boolean indicating if the internal reference is used.
 * @param power_down Array indicating power-down state for each channel.
 * @param adc_buf Boolean indicating if the ADC buffer is enabled.
 ******************************************************************************/
struct ad5592r_dev {
	const struct ad5592r_rw_ops *ops;
	struct no_os_i2c_desc *i2c;
	struct no_os_spi_desc *spi;
	uint16_t spi_msg;
	uint8_t num_channels;
	uint16_t cached_dac[8];
	uint16_t cached_gp_ctrl;
	uint8_t channel_modes[8];
	uint8_t channel_offstate[8];
	uint8_t gpio_out;
	uint8_t gpio_in;
	uint8_t gpio_val;
	uint8_t ldac_mode;
	enum ad559xr_range adc_range;
	enum ad559xr_range dac_range;
	bool int_ref;
	uint8_t power_down[8];
	bool adc_buf;
};

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the AD5592R device. It is essential for configuring the device's
 * registers to achieve the desired operation. The function should be
 * called with a valid device structure that has been properly
 * initialized. It is important to ensure that the register address
 * provided is within the valid range of the device's registers. The
 * function returns an integer status code indicating the success or
 * failure of the write operation.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null.
 * @param reg The 8-bit register address to which the value will be written.
 * Must be a valid register address for the AD5592R device.
 * @param value The 16-bit value to write to the specified register.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad5592r_base_reg_write(struct ad5592r_dev *dev, uint8_t reg,
			       uint16_t value);
/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from an
 * AD5592R device. It is typically used to obtain configuration or status
 * information from the device. The function requires a valid device
 * structure and a register address to read from. The result is stored in
 * the provided memory location pointed to by the value parameter. The
 * function returns an error code if the read operation fails, so it is
 * important to check the return value to ensure successful execution.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param reg The address of the register to read from. Must be a valid register
 * address as defined in the ad5592r_registers enumeration.
 * @param value A pointer to a uint16_t where the read register value will be
 * stored. Must not be null, and the caller is responsible for
 * ensuring that the memory is valid for writing.
 * @return Returns an int32_t error code, with 0 indicating success and a
 * negative value indicating failure.
 ******************************************************************************/
int32_t ad5592r_base_reg_read(struct ad5592r_dev *dev, uint8_t reg,
			      uint16_t *value);
/***************************************************************************//**
 * @brief This function is used to obtain the current state of a specific GPIO
 * pin on the AD5592R device. It should be called when you need to check
 * whether a particular GPIO pin is set high or low. The function
 * requires a valid device structure and a valid pin offset. If the pin
 * is configured as an output, the function returns the cached output
 * value. If the pin is configured as an input, it reads the actual state
 * from the hardware. The function returns a negative error code if the
 * device structure is null or if there is an error reading the GPIO
 * state.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param offset An 8-bit unsigned integer representing the GPIO pin offset to
 * be queried. Valid values are typically within the range of
 * available GPIO pins on the device.
 * @return Returns 1 if the specified GPIO pin is high, 0 if it is low, or a
 * negative error code if an error occurs.
 ******************************************************************************/
int32_t ad5592r_gpio_get(struct ad5592r_dev *dev, uint8_t offset);
/***************************************************************************//**
 * @brief This function is used to set the state of a specific GPIO pin on the
 * AD5592R device. It should be called when you need to change the output
 * state of a GPIO pin, either setting it high or low. The function
 * requires a valid device structure and a valid pin offset. It modifies
 * the internal GPIO state of the device and writes the new state to the
 * device's GPIO register. Ensure that the device structure is properly
 * initialized before calling this function. If the device pointer is
 * null, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. The function will return -1 if this
 * parameter is null.
 * @param offset An 8-bit unsigned integer representing the GPIO pin offset to
 * be set. Valid values are typically within the range of
 * available GPIO pins on the device.
 * @param value A 32-bit integer representing the desired state of the GPIO pin.
 * A non-zero value sets the pin high, while a zero value sets it
 * low.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad5592r_gpio_set(struct ad5592r_dev *dev, uint8_t offset,
			 int32_t value);
/***************************************************************************//**
 * @brief Use this function to set a specific GPIO pin on the AD5592R device to
 * input mode. This is typically done when you need to read data from an
 * external source connected to the pin. The function must be called with
 * a valid device structure and a valid pin offset. It modifies the
 * internal state of the device to disable output and enable input for
 * the specified pin. Ensure that the device structure is properly
 * initialized before calling this function. If the device pointer is
 * null, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. The function returns -1 if this
 * parameter is null.
 * @param offset An 8-bit unsigned integer representing the GPIO pin number to
 * configure as input. Valid values are typically within the range
 * of available GPIO pins on the device.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5592r_gpio_direction_input(struct ad5592r_dev *dev, uint8_t offset);
/***************************************************************************//**
 * @brief This function is used to configure a specific GPIO pin on the AD5592R
 * device as an output and to set its initial logical value. It should be
 * called when you need to control a GPIO pin for output purposes. The
 * function requires a valid device structure pointer and a valid pin
 * offset within the range of available GPIO pins. The initial value of
 * the pin can be set to either high or low. If the device pointer is
 * null, the function returns an error. The function also returns an
 * error if any of the register write operations fail.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param offset The GPIO pin offset to configure as output. Must be within the
 * valid range of GPIO pins for the device.
 * @param value The initial logical value to set for the GPIO pin. Typically, 0
 * for low and non-zero for high.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if a register write operation fails.
 ******************************************************************************/
int32_t ad5592r_gpio_direction_output(struct ad5592r_dev *dev,
				      uint8_t offset, int32_t value);
/***************************************************************************//**
 * @brief This function is used to reset the AD5592R device to its default state
 * by performing a software reset. It should be called when the device
 * needs to be reinitialized or when a known starting state is required.
 * The function requires a valid device structure pointer and will return
 * an error if the pointer is null. After issuing the reset command, the
 * function introduces a delay to ensure the reset process is completed
 * before any further operations are performed on the device.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device to
 * be reset. Must not be null. If null, the function returns an error
 * code.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if the reset command fails.
 ******************************************************************************/
int32_t ad5592r_software_reset(struct ad5592r_dev *dev);
/***************************************************************************//**
 * @brief This function sets up the channel modes for the AD5592R device based
 * on the configuration specified in the device structure. It must be
 * called after the device has been initialized and the channel modes
 * have been set in the device structure. The function configures each
 * channel to operate as a DAC, ADC, GPIO input, GPIO output, or leaves
 * it unused, applying the specified off-state behavior for unused
 * channels. It writes the configuration to the device registers and
 * verifies the configuration by reading back a register. If the device
 * pointer is null or if any register write or read operation fails, the
 * function returns an error code.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure. Must not be
 * null. The structure should have the channel_modes and
 * channel_offstate arrays properly configured before calling this
 * function. If the pointer is null, the function returns -1.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if any register write or read operation fails.
 ******************************************************************************/
int32_t ad5592r_set_channel_modes(struct ad5592r_dev *dev);
/***************************************************************************//**
 * @brief This function is used to reset all channel modes of the AD5592R device
 * to an unused state. It should be called when you want to clear the
 * current configuration of the device's channels, typically as part of a
 * reinitialization process. The function requires a valid device
 * structure and will return an error if the device pointer is null.
 * After resetting the channel modes, it applies the changes by calling
 * another function to set the channel modes on the device.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null. If null, the function returns an error code.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if setting the channel modes fails.
 ******************************************************************************/
int32_t ad5592r_reset_channel_modes(struct ad5592r_dev *dev);
/***************************************************************************//**
 * @brief Use this function to configure the ADC range of an AD5592R device to
 * either ZERO_TO_VREF or ZERO_TO_2VREF. This function should be called
 * after the device has been properly initialized. It updates the
 * device's control register to reflect the desired ADC range setting. If
 * the device pointer is null, the function returns an error. Successful
 * execution results in the ADC range being updated in the device
 * structure.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param adc_range An enum value of type ad559xr_range indicating the desired
 * ADC range. Valid values are ZERO_TO_VREF and ZERO_TO_2VREF.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5592r_set_adc_range(struct ad5592r_dev *dev,
			      enum ad559xr_range adc_range);
/***************************************************************************//**
 * @brief This function configures the digital-to-analog converter (DAC) output
 * voltage range for the specified AD5592R device. It should be called
 * when there is a need to change the DAC range, either to the internal
 * reference voltage (VREF) or twice the VREF. The function must be
 * called with a valid device structure that has been properly
 * initialized. If the device pointer is null, the function returns an
 * error. The function updates the device's internal state to reflect the
 * new DAC range setting.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param dac_range An enum value of type ad559xr_range indicating the desired
 * DAC range. Valid values are ZERO_TO_VREF and ZERO_TO_2VREF.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5592r_set_dac_range(struct ad5592r_dev *dev,
			      enum ad559xr_range dac_range);
/***************************************************************************//**
 * @brief This function is used to enable or disable the power down mode for a
 * specific channel on the AD5592R device. It is typically called when
 * there is a need to conserve power by disabling unused channels. The
 * function requires a valid device structure and a channel number within
 * the valid range. It updates the device's internal state to reflect the
 * power down status of the specified channel. The function must be
 * called with a properly initialized device structure, and the channel
 * number must be within the range of available channels. If the device
 * structure is null, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param chan The channel number to be powered down or up. Must be within the
 * range of available channels (0 to NUM_OF_CHANNELS-1).
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) power down mode for the specified channel.
 * @return Returns 0 on success, or a negative error code if the device
 * structure is null or if an error occurs during the operation.
 ******************************************************************************/
int32_t ad5592r_power_down(struct ad5592r_dev *dev, uint8_t chan, bool enable);
/***************************************************************************//**
 * @brief This function is used to control the internal reference voltage of the
 * AD5592R device. It should be called when there is a need to enable or
 * disable the internal reference, which is typically required during
 * device configuration or reconfiguration. The function must be called
 * with a valid device structure pointer, and it will return an error if
 * the device pointer is null. The function updates the device's internal
 * state to reflect the change in the reference setting.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the internal reference.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5592r_set_int_ref(struct ad5592r_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function is used to control the ADC buffer state on an AD5592R
 * device. It should be called when there is a need to enable or disable
 * the ADC buffer, which can affect the performance of ADC operations.
 * The function requires a valid device structure and will return an
 * error if the device is not properly initialized. It is important to
 * ensure that the device is in a suitable state for configuration
 * changes before calling this function.
 *
 * @param dev A pointer to an initialized ad5592r_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the ADC buffer.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5592r_set_adc_buffer(struct ad5592r_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function is used to update a specific register in the AD5592R
 * device by applying a mask to the current register value and then
 * writing the new data. It is typically called when a specific bit or
 * set of bits in a register needs to be modified without affecting other
 * bits. The function requires a valid device structure and a register
 * address to operate. It reads the current value of the register,
 * applies the mask, and writes the updated value back to the register.
 * If the device structure is null or if any read or write operation
 * fails, the function returns an error code.
 *
 * @param dev A pointer to an ad5592r_dev structure representing the device.
 * Must not be null. The function returns -1 if this parameter is
 * null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the AD5592R device.
 * @param data The data to be written to the register after masking. It is
 * combined with the current register value using the mask.
 * @param mask A bitmask indicating which bits of the register should be
 * updated. Bits set to 1 in the mask will be updated with the
 * corresponding bits from the data parameter.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if a read/write operation fails.
 ******************************************************************************/
int32_t ad5592r_base_reg_update(struct ad5592r_dev* dev, uint16_t reg_addr,
				uint16_t data, uint16_t mask);

#endif /* AD5592R_BASE_H_ */
