/***************************************************************************//**
*   @file   AD5449.h
*   @brief  Header file of AD5449 Driver. This driver supporting the following
*              devices: AD5415, AD5443, AD5432, AD5426, AD5429, AD5439, AD5449
*
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
********************************************************************************
* Copyright 2013(c) Analog Devices, Inc.
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

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `bool_t` data structure is an enumeration that defines a boolean
 * type with two possible values: `false` and `true`. This enum is used
 * to represent binary states or conditions in the code, providing a
 * clear and concise way to handle boolean logic.
 *
 * @param false Represents the boolean value 'false'.
 * @param true Represents the boolean value 'true'.
 ******************************************************************************/
enum bool_t {
	false,
	true
};

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5449_type_t` is an enumeration that defines a set of constants
 * representing different types of AD54xx series devices supported by the
 * AD5449 driver. Each enumerator corresponds to a specific device model,
 * allowing the software to identify and handle different devices within
 * the same driver framework. This enumeration is crucial for ensuring
 * that the correct device-specific operations and configurations are
 * applied during the driver's operation.
 *
 * @param ID_AD5415 Represents the AD5415 device type.
 * @param ID_AD5426 Represents the AD5426 device type.
 * @param ID_AD5429 Represents the AD5429 device type.
 * @param ID_AD5432 Represents the AD5432 device type.
 * @param ID_AD5439 Represents the AD5439 device type.
 * @param ID_AD5443 Represents the AD5443 device type.
 * @param ID_AD5449 Represents the AD5449 device type.
 ******************************************************************************/
enum ad5449_type_t {
	ID_AD5415,
	ID_AD5426,
	ID_AD5429,
	ID_AD5432,
	ID_AD5439,
	ID_AD5443,
	ID_AD5449,
};

/***************************************************************************//**
 * @brief The `ad5449_chip_info` structure is used to encapsulate information
 * about a specific AD5449 chip, including the number of channels it
 * supports, its resolution, and whether it has a control feature. This
 * structure is essential for configuring and managing the chip's
 * capabilities in applications that utilize the AD5449 series of
 * digital-to-analog converters.
 *
 * @param num_channels Specifies the number of channels available in the chip.
 * @param resolution Indicates the resolution of the chip in bits.
 * @param has_ctrl A boolean indicating if the chip has a control feature.
 ******************************************************************************/
struct ad5449_chip_info {
	uint8_t num_channels;
	uint8_t resolution;
	enum bool_t has_ctrl;
};

/***************************************************************************//**
 * @brief The `ad5449_dev` structure is designed to encapsulate the necessary
 * components for interfacing with AD5449 series devices, which are
 * digital-to-analog converters (DACs). It includes pointers to SPI and
 * GPIO descriptors for communication and control, as well as fields for
 * specifying the active device type and maintaining control register
 * settings. This structure is essential for managing the hardware
 * interface and configuration of the AD5449 devices.
 *
 * @param spi_desc Pointer to a SPI descriptor for managing SPI communication.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin control.
 * @param gpio_clr Pointer to a GPIO descriptor for the CLR pin control.
 * @param act_device Enumeration indicating the active AD5449 device type.
 * @param control_reg 16-bit register for storing control settings of the
 * device.
 ******************************************************************************/
struct ad5449_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_ldac;
	struct no_os_gpio_desc	*gpio_clr;
	/* Device Settings */
	enum ad5449_type_t		act_device;
	uint16_t		control_reg;
};

/***************************************************************************//**
 * @brief The `ad5449_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up an AD5449 device. It
 * includes SPI initialization parameters, GPIO configurations for the
 * LDAC and CLR pins, and the specific device type being used. This
 * structure is essential for configuring the device's communication and
 * control interfaces before operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_ldac Holds the initialization parameters for the LDAC GPIO pin.
 * @param gpio_clr Holds the initialization parameters for the CLR GPIO pin.
 * @param act_device Specifies the active device type from the supported AD5449
 * series.
 ******************************************************************************/
struct ad5449_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_ldac;
	struct no_os_gpio_init_param	gpio_clr;
	/* Device Settings */
	enum ad5449_type_t	act_device;
};

/* Control Bits */
#define AD5449_CTRL_NOP             0
#define AD5449_CTRL_LOADUPDATE(x)   (1 + 3 * (x))
#define AD5449_CTRL_READBACK(x)     (2 + 3 * (x))
#define AD5449_CTRL_LOAD(x)         (3 + 3 * (x))
#define AD5449_CTRL_UPDATEALL       7
#define AD5449_CTRL_LOADALL         8
#define AD5449_CTRL_DAISY_CHAIN     9
#define AD5449_CTRL_CLK_EDGE        10
#define AD5449_CTRL_CLR_ZERO        11
#define AD5449_CTRL_CLR_MID         12
#define AD5449_CTRL_REG             13

/* AD5449 channels */
#define AD5449_CH_A                 0
#define AD5449_CH_B                 1

/* Clear target scales */
#define AD5449_ZERO_SCALE            0
#define AD5449_MID_SCALE             1

/* Active clock edge */
#define AD5449_CLOCK_NEGEDGE         0
#define AD5449_CLOCK_POSEDGE         1

/* Daisy-Chain Control */
#define AD5449_DAISY_CHAIN_DIS       0
#define AD5449_DAISY_CHAIN_EN        1

/* AD5449_CTRL_REG definition */
#define AD5449_SDO_MASK          (3 << 10)
#define AD5449_DSY_MASK          (1 << 9)
#define AD5449_HCLR_MASK         (1 << 8)
#define AD5449_SCLK_MASK         (1 << 7)
#define AD5449_SDO_BIT           10
#define AD5449_DSY_BIT           9
#define AD5449_HCLR_BIT          8
#define AD5449_SCLK_BIT          7

/* AD5449 GPIO */
#define AD5449_LDAC_OUT             no_os_gpio_direction_output(dev->gpio_ldac,   \
			            NO_OS_GPIO_HIGH)
#define AD5449_LDAC_LOW             no_os_gpio_set_value(dev->gpio_ldac,          \
			            NO_OS_GPIO_LOW)
#define AD5449_LDAC_HIGH            no_os_gpio_set_value(dev->gpio_ldac,          \
			            NO_OS_GPIO_HIGH)

#define AD5449_CLR_OUT              no_os_gpio_direction_output(dev->gpio_clr,   \
			            NO_OS_GPIO_HIGH)
#define AD5449_CLR_LOW              no_os_gpio_set_value(dev->gpio_clr,          \
			            NO_OS_GPIO_LOW)
#define AD5449_CLR_HIGH             no_os_gpio_set_value(dev->gpio_clr,          \
			            NO_OS_GPIO_HIGH)

/* SDO Control Bits */
#define AD5449_SDO_FULL             0
#define AD5449_SDO_WEAK             1
#define AD5449_SDO_OPEN_DRAIN       2
#define AD5449_SDO_DISABLE          3

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize SPI and Initial Values for AD5449 Board. */
/***************************************************************************//**
 * @brief This function sets up the AD5449 device by allocating necessary
 * resources and initializing SPI and GPIO interfaces based on the
 * provided initialization parameters. It must be called before any other
 * operations on the AD5449 device to ensure proper setup. The function
 * configures the device's control register and sets initial states for
 * GPIO pins. It returns a status code indicating success or failure of
 * the initialization process.
 *
 * @param device A pointer to a pointer of type `struct ad5449_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad5449_init_param` containing
 * initialization parameters for the SPI and GPIO interfaces,
 * as well as the specific AD5449 device type to be
 * initialized. All fields must be properly set before calling
 * the function.
 * @return Returns an `int8_t` status code: 0 for success, or a negative value
 * indicating an error during initialization.
 ******************************************************************************/
int8_t ad5449_init(struct ad5449_dev **device,
		   struct ad5449_init_param init_param);

/* Free the resources allocated by AD5449_Init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD5449
 * device when it is no longer needed. This function should be called to
 * clean up after a successful initialization with `ad5449_init`. It
 * ensures that all allocated memory and hardware resources, such as SPI
 * and GPIO descriptors, are properly freed. Failure to call this
 * function may result in resource leaks.
 *
 * @param dev A pointer to an `ad5449_dev` structure representing the device to
 * be removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during resource
 * deallocation.
 ******************************************************************************/
int32_t ad5449_remove(struct ad5449_dev *dev);

/* Write to shift register via SPI. */
/***************************************************************************//**
 * @brief This function writes a command and data to the input shift register of
 * an AD5449 device using SPI communication. It is typically used to
 * configure the device or update its settings. The function requires a
 * valid device structure that has been initialized and configured for
 * SPI communication. The command parameter determines the operation to
 * be performed, while the data parameter provides the necessary data for
 * the operation. The function returns the value read back from the
 * device, which can be used for verification purposes. It is important
 * to ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad5449_dev structure representing the device. Must
 * not be null and should be properly initialized with SPI settings.
 * @param command A 16-bit unsigned integer representing the command to be sent
 * to the device. Valid commands are defined by the device's
 * protocol.
 * @param data A 16-bit unsigned integer representing the data to be sent along
 * with the command. The data should be formatted according to the
 * command's requirements.
 * @return Returns a 16-bit unsigned integer representing the data read back
 * from the device after the write operation.
 ******************************************************************************/
uint16_t ad5449_set_input_shift_reg(struct ad5449_dev *dev,
				    uint16_t command,
				    uint16_t data);

/* Load and updates the selected DAC with a given value. */
/***************************************************************************//**
 * @brief This function is used to load a digital-to-analog converter (DAC)
 * channel with a specified value and update it immediately. It is
 * suitable for devices that support multiple channels, as well as those
 * with a single channel. The function should be called when you need to
 * set a specific output value on a DAC channel. Ensure that the device
 * has been properly initialized before calling this function. The
 * function handles the selection of the appropriate channel based on the
 * device's capabilities.
 *
 * @param dev A pointer to an initialized ad5449_dev structure representing the
 * device. Must not be null.
 * @param channel The DAC channel to be updated. For devices with a single
 * channel, this parameter is ignored and the default channel is
 * used.
 * @param dac_value The value to be loaded into the DAC. It should be within the
 * valid range for the device's resolution.
 * @return None
 ******************************************************************************/
void ad5449_load_update_channel(struct ad5449_dev *dev,
				uint8_t channel,
				uint16_t dac_value);

/* Load selected DAC input register with a given value. */
/***************************************************************************//**
 * @brief This function is used to load a specified DAC channel with a given
 * digital value on devices that support multiple channels. It should be
 * called when you need to set the input register of a specific DAC
 * channel to a desired value. If the device supports only one channel,
 * the function defaults to loading channel A. This function does not
 * update the DAC output; it only loads the input register. Ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5449_dev structure representing the
 * device. Must not be null.
 * @param channel The channel number to load the value into. Valid values depend
 * on the device's number of channels, typically 0 or 1 for
 * devices with two channels.
 * @param dac_value The digital value to load into the specified channel's input
 * register. This is a 16-bit value representing the desired
 * DAC input.
 * @return None
 ******************************************************************************/
void ad5449_load_channel(struct ad5449_dev *dev,
			 uint8_t channel,
			 uint16_t dac_value);

/* Read from the selected DAC register. */
/***************************************************************************//**
 * @brief Use this function to retrieve the current digital-to-analog converter
 * (DAC) value from a specified channel on an AD5449 device. This
 * function is useful for verifying the current output setting of a DAC
 * channel. It should be called only after the device has been properly
 * initialized using `ad5449_init`. The function handles devices with
 * multiple channels by selecting the appropriate channel for readback.
 * If the device supports only a single channel, it defaults to reading
 * from channel A. Ensure that the `dev` parameter is a valid pointer to
 * an initialized `ad5449_dev` structure.
 *
 * @param dev A pointer to an `ad5449_dev` structure representing the device.
 * Must not be null and should be initialized using `ad5449_init`
 * before calling this function.
 * @param channel The channel number to read from. Valid values depend on the
 * specific device configuration, typically 0 for channel A and 1
 * for channel B. If the device supports only one channel, this
 * parameter is ignored and channel A is read.
 * @return Returns a 16-bit unsigned integer representing the current DAC value
 * of the specified channel.
 ******************************************************************************/
uint16_t ad5449_readback_channel(struct ad5449_dev *dev,
				 uint8_t channel);

/* Update the DAC outputs (all channels). */
/***************************************************************************//**
 * @brief This function updates the outputs of all DAC channels for devices that
 * support more than one channel. It should be called when you need to
 * synchronize the output of all channels on a multi-channel device. The
 * function does nothing for devices with only one channel, so it is safe
 * to call regardless of the device type. Ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad5449_dev structure representing the
 * device. Must not be null. The function checks the number of
 * channels supported by the device and updates all channels if there
 * is more than one.
 * @return None
 ******************************************************************************/
void ad5449_update_all(struct ad5449_dev *dev);

/* Load the DAC input registers. */
/***************************************************************************//**
 * @brief This function is used to load a specified DAC value into the input
 * registers of all channels of the AD5449 device, but only if the device
 * supports multiple channels. It should be called when you need to set
 * the same DAC value across all channels simultaneously. Ensure that the
 * device is properly initialized and supports more than one channel
 * before calling this function. The function does not perform any
 * operation if the device has only one channel.
 *
 * @param dev A pointer to an initialized ad5449_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param dac_value An int16_t value representing the DAC value to be loaded
 * into the input registers. The valid range depends on the
 * device's resolution, and values outside this range may
 * result in undefined behavior.
 * @return None
 ******************************************************************************/
void ad5449_load_all(struct ad5449_dev *dev,
		     int16_t dac_value);

/* Set up the scale where to the output will be cleared on active CLR signal */
/***************************************************************************//**
 * @brief This function configures the scale to which the output will be cleared
 * when the active CLR signal is triggered. It should be used to set the
 * desired clear scale for devices that support control register
 * operations. The function must be called with a valid device structure
 * and a scale type, either zero scale or mid scale. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an initialized ad5449_dev structure representing the
 * device. Must not be null.
 * @param type An integer representing the scale type to set. Valid values are
 * AD5449_ZERO_SCALE (0) for zero scale and AD5449_MID_SCALE (1) for
 * mid scale. Invalid values are ignored.
 * @return None
 ******************************************************************************/
void ad5449_clear_scale_setup(struct ad5449_dev *dev,
			      int8_t type);

/* Enable/disable the Daisy-Chain mode */
/***************************************************************************//**
 * @brief This function configures the daisy-chain mode of an AD5449 device,
 * allowing multiple devices to be connected in series. It should be
 * called when setting up the device chain configuration, typically
 * during initialization or when changing the device setup. The function
 * requires a valid device structure and a value indicating whether to
 * enable or disable the daisy-chain mode. The function assumes the
 * device has been properly initialized and the provided device structure
 * is valid. It modifies the control register of the device based on the
 * input value.
 *
 * @param dev A pointer to an ad5449_dev structure representing the device to
 * configure. Must not be null and should point to a valid,
 * initialized device structure.
 * @param value An int8_t value indicating whether to enable
 * (AD5449_DAISY_CHAIN_EN) or disable (AD5449_DAISY_CHAIN_DIS) the
 * daisy-chain mode. Invalid values are ignored, and the function
 * will not alter the device state.
 * @return None
 ******************************************************************************/
void ad5449_daisy_chain_setup(struct ad5449_dev *dev,
			      int8_t value);

/* Control the SDO output driver strength */
/***************************************************************************//**
 * @brief This function configures the SDO (Serial Data Output) driver strength
 * for a given AD5449 device. It should be called only if the device
 * supports control register operations, as indicated by the device's
 * capabilities. The function modifies the control register of the device
 * to set the desired SDO driver strength based on the provided control
 * bits. It is important to ensure that the device has been properly
 * initialized before calling this function. If the device does not
 * support control operations, the function will have no effect.
 *
 * @param dev A pointer to an ad5449_dev structure representing the device to be
 * configured. Must not be null and should point to a properly
 * initialized device.
 * @param control_bits An int8_t value representing the desired SDO control
 * settings. Only the two least significant bits are used,
 * and they should be set according to the desired SDO
 * driver strength (e.g., full, weak, open drain, or
 * disable).
 * @return None
 ******************************************************************************/
void ad5449_sdocontrol(struct ad5449_dev *dev,
		       int8_t control_bits);

/* Set up the active clock edge of the SPI interface */
/***************************************************************************//**
 * @brief This function configures the active clock edge for the SPI interface
 * of the AD5449 device. It should be called to ensure the correct clock
 * edge is used for data transfer, which is essential for proper
 * communication with the device. The function must be called with a
 * valid device structure and a clock edge value. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an ad5449_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param value An int8_t representing the desired clock edge. Valid values are
 * AD5449_CLOCK_POSEDGE for clocking data on the rising edge and
 * AD5449_CLOCK_NEGEDGE for clocking data on the falling edge.
 * Invalid values will be ignored.
 * @return None
 ******************************************************************************/
void ad5449_sclksetup(struct ad5449_dev *dev,
		      int8_t value);
