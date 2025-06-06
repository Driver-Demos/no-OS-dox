/***************************************************************************//**
*   @file   ad5686.h
*   @brief  Header file of AD5686 Driver. This driver supporting the following
*              devices: AD5684R, AD5685R, AD5686R, AD5694R, AD5695R, AD5696R,
*
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
********************************************************************************
* Copyright 2013, 2020(c) Analog Devices, Inc.
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
*
*******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_i2c.h"

/* Control Bits */
#define AD5686_CTRL_NOP          0
#define AD5686_CTRL_WRITE        1
#define AD5686_CTRL_UPDATE       2
#define AD5686_CTRL_WRITEUPDATE  3
#define AD5686_CTRL_PWR          4
#define AD5686_CTRL_LDAC_MASK    5
#define AD5686_CTRL_SWRESET      6
#define AD5686_CTRL_IREF_REG     7
#define AD5686_CTRL_DCEN         8
#define AD5686_CTRL_RB_REG       9

#define AD5683_CMD_WR_CTRL_REG   4
#define AD5683_CTRL_RB_REG       5

/* Power-down operation modes masks */
#define AD5686_PWRM_NORMAL       0
#define AD5686_PWRM_1K           1
#define AD5686_PWRM_100K         2
#define AD5686_PWRM_THREESTATE   3

#define AD5686_PWRM_MASK         3

/* Enable/disable defines */
#define AD5686_INTREF_EN        1
#define AD5686_INTREF_DIS       0
#define AD5686_DC_EN            1
#define AD5686_DC_DIS           0
#define AD5686_RB_EN            1
#define AD5686_RB_DIS           0

#define MAX_RESOLUTION  16     // Maximum resolution of the supported devices

#define PKT_LENGTH               3      // SPI packet length in byte

#define ADDR_MASK                0xFF   // Mask for Address bits
#define CMD_OFFSET               4      // Offset for Command

#define AD5686_CMD_MASK          0xFF
#define AD5686_MSB_MASK          0xFF00 // Most significant byte of the data word
#define AD5686_MSB_OFFSET        8
#define AD5686_LSB_MASK          0x00FF // Least significant byte of the data word
#define AD5686_LSB_OFFSET        0

#define AD5683_MIDB_OFFSET       4	   // Offset for middle bits
#define AD5683_MIDB_MASK         0xFF
#define AD5683_MSB_OFFSET        12
#define AD5683_MSB_MASK          0xF
#define AD5683_CMD_MASK          0xF
#define AD5683_LSB_MASK          0xF
#define AD5683_LSB_OFFSET        4

#define AD5683_REG_MAP           0
#define AD5686_REG_MAP           1

/********************** AD5683 Write Control Register Bits ********************/

#define AD5683_CTRL_DCEN(x)      (((((x) & 0x1) << 0) << 10) & 0xFC00)
#define AD5683_CTRL_GM(x)        (((((x) & 0x1) << 1) << 10) & 0xFC00)
#define AD5683_CTRL_INT_REF(x)   (((((x) & 0x1) << 2) << 10) & 0xFC00)
#define AD5683_CTRL_PWRM(x)      (((((x) & 0x3) << 3) << 10) & 0xFC00)
#define AD5683_SW_RESET          ((((0x1) << 5) << 10) & 0xFC00)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad5686_type` enumeration defines a set of constants representing
 * different device IDs for a series of digital-to-analog converters
 * (DACs) from Analog Devices. Each enumerator corresponds to a specific
 * model in the AD56xx and AD57xx series, which are used to identify the
 * type of DAC being interfaced with in the driver code. This enumeration
 * is crucial for ensuring that the correct device-specific operations
 * and configurations are applied in the software.
 *
 * @param ID_AD5671R Represents the device ID for the AD5671R model.
 * @param ID_AD5672R Represents the device ID for the AD5672R model.
 * @param ID_AD5673R Represents the device ID for the AD5673R model.
 * @param ID_AD5674 Represents the device ID for the AD5674 model.
 * @param ID_AD5674R Represents the device ID for the AD5674R model.
 * @param ID_AD5675R Represents the device ID for the AD5675R model.
 * @param ID_AD5676 Represents the device ID for the AD5676 model.
 * @param ID_AD5676R Represents the device ID for the AD5676R model.
 * @param ID_AD5677R Represents the device ID for the AD5677R model.
 * @param ID_AD5679 Represents the device ID for the AD5679 model.
 * @param ID_AD5679R Represents the device ID for the AD5679R model.
 * @param ID_AD5686 Represents the device ID for the AD5686 model.
 * @param ID_AD5684R Represents the device ID for the AD5684R model.
 * @param ID_AD5685R Represents the device ID for the AD5685R model.
 * @param ID_AD5686R Represents the device ID for the AD5686R model.
 * @param ID_AD5687 Represents the device ID for the AD5687 model.
 * @param ID_AD5687R Represents the device ID for the AD5687R model.
 * @param ID_AD5689 Represents the device ID for the AD5689 model.
 * @param ID_AD5689R Represents the device ID for the AD5689R model.
 * @param ID_AD5697R Represents the device ID for the AD5697R model.
 * @param ID_AD5694 Represents the device ID for the AD5694 model.
 * @param ID_AD5694R Represents the device ID for the AD5694R model.
 * @param ID_AD5695R Represents the device ID for the AD5695R model.
 * @param ID_AD5696 Represents the device ID for the AD5696 model.
 * @param ID_AD5696R Represents the device ID for the AD5696R model.
 * @param ID_AD5681R Represents the device ID for the AD5681R model.
 * @param ID_AD5682R Represents the device ID for the AD5682R model.
 * @param ID_AD5683R Represents the device ID for the AD5683R model.
 * @param ID_AD5683 Represents the device ID for the AD5683 model.
 * @param ID_AD5691R Represents the device ID for the AD5691R model.
 * @param ID_AD5692R Represents the device ID for the AD5692R model.
 * @param ID_AD5693R Represents the device ID for the AD5693R model.
 * @param ID_AD5693 Represents the device ID for the AD5693 model.
 ******************************************************************************/
enum ad5686_type {
	ID_AD5671R,
	ID_AD5672R,
	ID_AD5673R,
	ID_AD5674,
	ID_AD5674R,
	ID_AD5675R,
	ID_AD5676,
	ID_AD5676R,
	ID_AD5677R,
	ID_AD5679,
	ID_AD5679R,
	ID_AD5686,
	ID_AD5684R,
	ID_AD5685R,
	ID_AD5686R,
	ID_AD5687,
	ID_AD5687R,
	ID_AD5689,
	ID_AD5689R,
	ID_AD5697R,
	ID_AD5694,
	ID_AD5694R,
	ID_AD5695R,
	ID_AD5696,
	ID_AD5696R,
	ID_AD5681R,
	ID_AD5682R,
	ID_AD5683R,
	ID_AD5683,
	ID_AD5691R,
	ID_AD5692R,
	ID_AD5693R,
	ID_AD5693
};

/***************************************************************************//**
 * @brief The `comm_type` enumeration defines two types of communication
 * protocols, SPI and I2C, which are used to interface with devices in
 * the AD5686 driver. This enum is used to specify the communication
 * method for data transfer between the microcontroller and the DAC
 * devices supported by the driver.
 *
 * @param SPI Represents the Serial Peripheral Interface communication type.
 * @param I2C Represents the Inter-Integrated Circuit communication type.
 ******************************************************************************/
enum comm_type {
	SPI,
	I2C,
};

/***************************************************************************//**
 * @brief The `ad5686_dac_channels` enumeration defines a set of constants
 * representing the individual channels of a digital-to-analog converter
 * (DAC) in the AD5686 series. Each enumerator corresponds to a specific
 * DAC channel, ranging from channel 0 to channel 15, allowing for easy
 * reference and manipulation of these channels in the driver code.
 *
 * @param AD5686_CH_0 Represents DAC channel 0.
 * @param AD5686_CH_1 Represents DAC channel 1.
 * @param AD5686_CH_2 Represents DAC channel 2.
 * @param AD5686_CH_3 Represents DAC channel 3.
 * @param AD5686_CH_4 Represents DAC channel 4.
 * @param AD5686_CH_5 Represents DAC channel 5.
 * @param AD5686_CH_6 Represents DAC channel 6.
 * @param AD5686_CH_7 Represents DAC channel 7.
 * @param AD5686_CH_8 Represents DAC channel 8.
 * @param AD5686_CH_9 Represents DAC channel 9.
 * @param AD5686_CH_10 Represents DAC channel 10.
 * @param AD5686_CH_11 Represents DAC channel 11.
 * @param AD5686_CH_12 Represents DAC channel 12.
 * @param AD5686_CH_13 Represents DAC channel 13.
 * @param AD5686_CH_14 Represents DAC channel 14.
 * @param AD5686_CH_15 Represents DAC channel 15.
 ******************************************************************************/
enum ad5686_dac_channels {
	AD5686_CH_0 = 0,
	AD5686_CH_1,
	AD5686_CH_2,
	AD5686_CH_3,
	AD5686_CH_4,
	AD5686_CH_5,
	AD5686_CH_6,
	AD5686_CH_7,
	AD5686_CH_8,
	AD5686_CH_9,
	AD5686_CH_10,
	AD5686_CH_11,
	AD5686_CH_12,
	AD5686_CH_13,
	AD5686_CH_14,
	AD5686_CH_15,
};

/***************************************************************************//**
 * @brief The `ad5686_chip_info` structure is used to encapsulate information
 * about a specific AD5686 DAC chip configuration. It includes the
 * resolution of the DAC, the register map configuration, the
 * communication protocol (SPI or I2C), and a pointer to the channel
 * addresses. This structure is essential for setting up and managing the
 * communication and operational parameters of the AD5686 series of DACs.
 *
 * @param resolution Specifies the resolution of the DAC in bits.
 * @param register_map Indicates the register map configuration for the device.
 * @param communication Defines the communication protocol used, either SPI or
 * I2C.
 * @param channel_addr Points to an array of channel addresses for the DAC.
 ******************************************************************************/
struct ad5686_chip_info {
	uint8_t		resolution;
	uint8_t		register_map;
	enum comm_type	communication;
	const uint32_t *channel_addr;
};

/***************************************************************************//**
 * @brief The `ad5686_dev` structure is a compound data type used to represent a
 * device instance for the AD5686 series of digital-to-analog converters
 * (DACs). It encapsulates descriptors for I2C and SPI communication
 * interfaces, as well as GPIO descriptors for controlling various pins
 * such as reset, LDAC, and gain. Additionally, it holds device-specific
 * settings including the active device type and configuration masks for
 * power-down and LDAC operations, facilitating the management and
 * control of the DAC's functionality.
 *
 * @param i2c_desc Pointer to an I2C descriptor for I2C communication.
 * @param spi_desc Pointer to an SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_ldac Pointer to a GPIO descriptor for the LDAC pin.
 * @param gpio_gain Pointer to a GPIO descriptor for the gain pin.
 * @param act_device Enumeration indicating the active device type.
 * @param power_down_mask 32-bit mask for power-down settings.
 * @param ldac_mask 32-bit mask for LDAC settings.
 ******************************************************************************/
struct ad5686_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_ldac;
	struct no_os_gpio_desc	*gpio_gain;
	/* Device Settings */
	enum ad5686_type	act_device;
	uint32_t power_down_mask;
	uint32_t ldac_mask;
};

/***************************************************************************//**
 * @brief The `ad5686_init_param` structure is used to initialize the AD5686
 * series of digital-to-analog converters (DACs) by specifying the
 * necessary communication interfaces and GPIO configurations. It
 * includes parameters for both I2C and SPI communication setups, as well
 * as GPIO settings for reset, LDAC, and gain control. Additionally, it
 * identifies the specific device type being configured, allowing for
 * flexible initialization of various models within the AD5686 family.
 *
 * @param i2c_init Initializes I2C communication parameters.
 * @param spi_init Initializes SPI communication parameters.
 * @param gpio_reset Configures the GPIO pin for resetting the device.
 * @param gpio_ldac Configures the GPIO pin for the LDAC function.
 * @param gpio_gain Configures the GPIO pin for gain control.
 * @param act_device Specifies the active device type from the supported AD5686
 * series.
 ******************************************************************************/
struct ad5686_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_reset;
	struct no_os_gpio_init_param	gpio_ldac;
	struct no_os_gpio_init_param	gpio_gain;
	/* Device Settings */
	enum ad5686_type	act_device;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize SPI and Initial Values for AD5686 Board. */
/***************************************************************************//**
 * @brief This function sets up the AD5686 device by allocating necessary
 * resources and configuring it based on the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function handles both SPI and I2C
 * communication types, initializing the appropriate interface as
 * specified. It also configures GPIO pins for reset, LDAC, and gain
 * control, setting their initial states. The caller is responsible for
 * managing the memory of the device structure, which is allocated within
 * this function.
 *
 * @param device A pointer to a pointer of type `struct ad5686_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will manage the allocated
 * memory.
 * @param init_param A structure of type `struct ad5686_init_param` containing
 * initialization parameters for the device, including
 * communication type, GPIO configurations, and the specific
 * device type. All fields must be properly initialized before
 * calling the function.
 * @return Returns an `int32_t` indicating success (0) or failure (non-zero). A
 * negative return value indicates a memory allocation failure, while
 * other non-zero values indicate errors in initializing communication
 * interfaces or GPIOs.
 ******************************************************************************/
int32_t ad5686_init(struct ad5686_dev **device,
		    struct ad5686_init_param init_param);

/* Free the resources allocated by ad5686_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD5686
 * device that were allocated during initialization. It should be called
 * when the device is no longer needed to ensure proper cleanup and avoid
 * resource leaks. This function handles both SPI and I2C communication
 * types and will also free any associated GPIO resources if they were
 * initialized. It is important to ensure that the device pointer is
 * valid and was previously initialized using the appropriate
 * initialization function.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure.
 * @return Returns an int32_t indicating the success or failure of the resource
 * deallocation process. A non-zero value indicates an error occurred
 * during the removal of resources.
 ******************************************************************************/
int32_t ad5686_remove(struct ad5686_dev *dev);

/* Write to input register */
/***************************************************************************//**
 * @brief This function is used to write a command, address, and data to the
 * shift register of an AD5686 device. It is typically called when there
 * is a need to configure or update the device's settings or output
 * values. The function requires a valid device structure that has been
 * initialized and configured for communication via SPI or I2C. The
 * command and address parameters determine the specific operation and
 * target within the device, while the data parameter contains the value
 * to be written. The function returns the data read back from the
 * device, which can be used to verify the write operation.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null and should be properly initialized.
 * @param command A uint8_t value representing the command to be sent to the
 * device. Valid commands are defined by the device's protocol.
 * @param address A uint8_t value representing the address within the device to
 * which the command applies. Must be within the valid range for
 * the device.
 * @param data A uint16_t value containing the data to be written to the device.
 * The data is split into bytes according to the device's protocol.
 * @return Returns a uint16_t value representing the data read back from the
 * device after the write operation.
 ******************************************************************************/
uint16_t ad5686_set_shift_reg(struct ad5686_dev *dev,
			      uint8_t command,
			      uint8_t address,
			      uint16_t data);

/* Write to Input Register n (dependent on LDAC) */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to the input
 * register of a specified DAC channel on the AD5686 device. It is
 * typically called when you need to update the input register with new
 * data, which can later be transferred to the DAC register for output.
 * The function requires a valid device structure and a channel
 * enumeration to specify which DAC channel to target. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null and should be initialized prior to calling this
 * function. The caller retains ownership.
 * @param channel An enumeration value of type ad5686_dac_channels specifying
 * the DAC channel to which the data will be written. Must be a
 * valid channel for the device.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the input register. The data is left-shifted according to the
 * device's resolution before being written.
 * @return None
 ******************************************************************************/
void ad5686_write_register(struct ad5686_dev *dev,
			   enum ad5686_dac_channels channel,
			   uint16_t data);

/* Update DAC Register n with contents of Input Register n */
/***************************************************************************//**
 * @brief This function is used to update the DAC register of a specified
 * channel with the current contents of its corresponding input register.
 * It is typically called after writing new data to the input register
 * when the user wants to apply the changes to the DAC output. The
 * function requires a valid device structure and a channel identifier.
 * It should be used when the DAC output needs to be updated to reflect
 * new input data.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null. The device should be properly initialized before
 * calling this function.
 * @param channel An enum value of type ad5686_dac_channels specifying the DAC
 * channel to update. Must be a valid channel for the device.
 * @return None
 ******************************************************************************/
void ad5686_update_register(struct ad5686_dev *dev,
			    enum ad5686_dac_channels channel);

/* Write to and update DAC channel n */
/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specified DAC
 * channel and immediately update the channel with this new value. It is
 * typically called when you need to set a new output value for a DAC
 * channel and want the change to take effect immediately. The function
 * requires a valid device structure and a channel enumeration to specify
 * which DAC channel to update. It is important to ensure that the device
 * has been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param channel An enumeration value of type ad5686_dac_channels specifying
 * the DAC channel to write to and update. Must be a valid
 * channel for the device.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the specified DAC channel. The data is shifted according to the
 * device's resolution.
 * @return None
 ******************************************************************************/
void ad5686_write_update_register(struct ad5686_dev *dev,
				  enum ad5686_dac_channels channel,
				  uint16_t data);

/* Read back Input Register n */
/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in the input
 * register of a specified DAC channel on the AD5686 device. This
 * function is useful for verifying the data that has been written to a
 * channel. It requires a valid device structure and a channel
 * identifier. The function supports both SPI and I2C communication
 * protocols, automatically handling the appropriate method based on the
 * device configuration. Ensure that the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5686_dev structure representing the
 * device. Must not be null.
 * @param channel An enum value of type ad5686_dac_channels specifying the DAC
 * channel to read from. Must be a valid channel for the device.
 * @return Returns a 16-bit unsigned integer representing the value read from
 * the specified channel's input register.
 ******************************************************************************/
uint16_t ad5686_read_back_register(struct ad5686_dev *dev,
				   enum ad5686_dac_channels channel);

/* Power down / power up DAC */
/***************************************************************************//**
 * @brief This function configures the power mode of a specified DAC channel on
 * the AD5686 device. It should be used when you need to change the power
 * state of a DAC channel, such as powering it down to save energy or
 * powering it up for operation. The function requires a valid device
 * structure and a channel enumeration to specify which DAC channel to
 * configure. The mode parameter determines the power state, which can be
 * normal operation, 1K ohm to ground, 100K ohm to ground, or three-
 * state. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null and should be initialized.
 * @param channel An enum value of type ad5686_dac_channels specifying the DAC
 * channel to configure. Must be a valid channel for the device.
 * @param mode A uint8_t value representing the power mode. Valid values are 0
 * (normal), 1 (1K ohm to ground), 2 (100K ohm to ground), and 3
 * (three-state).
 * @return None
 ******************************************************************************/
void ad5686_power_mode(struct ad5686_dev *dev,
		       enum ad5686_dac_channels channel,
		       uint8_t mode);

/* Set up LDAC mask register */
/***************************************************************************//**
 * @brief This function configures the Load DAC (LDAC) mask for a specific
 * channel on the AD5686 device, allowing selective updating of DAC
 * registers. It should be used when you need to control which DAC
 * channels are updated by the LDAC pin. The function must be called with
 * a valid device structure and is only applicable to devices with the
 * AD5686 register map. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad5686_dev structure representing the
 * device. Must not be null.
 * @param channel An enum value of type ad5686_dac_channels specifying the DAC
 * channel to configure. Must be a valid channel for the device.
 * @param enable A uint8_t value where 1 enables and 0 disables the LDAC mask
 * for the specified channel.
 * @return None
 ******************************************************************************/
void ad5686_ldac_mask(struct ad5686_dev *dev,
		      enum ad5686_dac_channels channel,
		      uint8_t enable);

/* Software reset (power-on reset) */
/***************************************************************************//**
 * @brief Use this function to perform a software reset on an AD5686 device,
 * which resets the device to its default power-on state. This function
 * should be called when a reset of the device is required, such as
 * during initialization or error recovery. The function requires a valid
 * device structure that has been properly initialized. It does not
 * return a value and does not provide feedback on the success of the
 * reset operation.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device to
 * reset. This must not be null and should point to a properly
 * initialized device structure. The function does not handle null
 * pointers and expects the caller to ensure the validity of this
 * parameter.
 * @return None
 ******************************************************************************/
void ad5686_software_reset(struct ad5686_dev *dev);

/* Write to Internal reference setup register */
/***************************************************************************//**
 * @brief This function is used to enable or disable the internal reference of
 * an AD5686 series device. It should be called when there is a need to
 * change the reference voltage source of the device, either enabling the
 * internal reference or disabling it to use an external reference. The
 * function must be called with a valid device structure that has been
 * properly initialized. The behavior of the function depends on the
 * specific device model being used, as it adjusts the internal settings
 * accordingly.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param value A uint8_t value indicating whether to enable or disable the
 * internal reference. Typically, 1 to enable and 0 to disable.
 * Invalid values may result in undefined behavior.
 * @return None
 ******************************************************************************/
void ad5686_internal_reference(struct ad5686_dev *dev,
			       uint8_t value);

/* Set up DCEN register (daisy-chain enable) */
/***************************************************************************//**
 * @brief This function configures the daisy-chain mode of the specified AD5686
 * device. It should be used when you need to enable or disable the
 * daisy-chain feature, which allows multiple devices to be connected in
 * series and controlled through a single interface. The function must be
 * called with a valid device structure that has been initialized, and it
 * is important to ensure that the device supports the daisy-chain
 * feature before calling this function. The function does not return a
 * value, and its behavior depends on the specific device type being
 * used.
 *
 * @param dev A pointer to an initialized ad5686_dev structure representing the
 * device to configure. Must not be null.
 * @param value A uint8_t value indicating whether to enable (1) or disable (0)
 * the daisy-chain mode. Values outside this range may lead to
 * undefined behavior.
 * @return None
 ******************************************************************************/
void ad5686_daisy_chain_en(struct ad5686_dev *dev,
			   uint8_t value);

/* Set up readback register (readback enable) */
/***************************************************************************//**
 * @brief This function is used to control the readback feature of the AD5686
 * device, allowing the user to enable or disable it as needed. It should
 * be called when the device is configured to use the AD5686 register
 * map. This function does not return a value and does not provide
 * feedback on the success of the operation, so it is important to ensure
 * that the device is properly initialized and configured before calling
 * this function.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. This
 * must not be null and should be properly initialized before calling
 * the function.
 * @param value A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) the readback feature. The function does not
 * validate this parameter, so it is the caller's responsibility to
 * provide a valid value.
 * @return None
 ******************************************************************************/
void ad5686_read_back_en(struct ad5686_dev *dev,
			 uint8_t value);

/* Set Gain mode */
/***************************************************************************//**
 * @brief This function configures the gain mode of the specified AD5686 device.
 * It should be called when you need to change the gain setting of the
 * device. The function requires a valid device structure and a gain mode
 * value. It is important to ensure that the device is properly
 * initialized before calling this function. The function will only
 * succeed if the device's register map is compatible with the AD5683
 * register map; otherwise, it will return an error.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function. The caller retains ownership.
 * @param value A uint8_t value representing the desired gain mode. Valid values
 * are typically 0 or 1, corresponding to the specific gain
 * settings supported by the device.
 * @return Returns 0 on success if the device's register map is AD5683_REG_MAP,
 * otherwise returns -1 indicating an unsupported operation for the
 * device.
 ******************************************************************************/
int32_t ad5686_gain_mode(struct ad5686_dev *dev, uint8_t value);
