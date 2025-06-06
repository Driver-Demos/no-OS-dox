/**************************************************************************//**
*   @file   ad525x.h
*   @brief  Header file of AD525X Driver for Microblaze processor. This driver
*           supporting the following devices : AD5232, AD5235, AD5252, AD5251,
*           AD5254, AD5253, ADN2850
*   @author Istvan Csomortani (istvan.csomortani@analog.com)
*
*******************************************************************************
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
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*****************************************************************************/
#ifndef __AD525X_H__
#define __AD525X_H__

/*****************************************************************************/
/****************************** Include Files ********************************/
/*****************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_i2c.h"

/*****************************************************************************/
/*  Device specific MACROs                                                   */
/*****************************************************************************/
/* GPIOs */
#define AD525X_RESET_OUT                    no_os_gpio_direction_output(dev->gpio_reset,    \
			                    NO_OS_GPIO_HIGH)
#define AD525X_RESET_LOW                    no_os_gpio_set_value(dev->gpio_reset,           \
			                    NO_OS_GPIO_LOW)
#define AD525X_RESET_HIGH                   no_os_gpio_set_value(dev->gpio_reset,           \
			                    NO_OS_GPIO_HIGH)
#define AD525X_SHUTDOWN_OUT                 no_os_gpio_direction_output(dev->gpio_shutdown, \
			                    NO_OS_GPIO_HIGH)
#define AD525X_SHUTDOWN_LOW                 no_os_gpio_set_value(dev->gpio_shutdown,        \
			                    NO_OS_GPIO_LOW)
#define AD525X_SHUTDOWN_HIGH                no_os_gpio_set_value(dev->gpio_shutdown,        \
			                    NO_OS_GPIO_HIGH)
#define AD525X_READY_IN                     no_os_gpio_direction_input(dev->gpio_ready)
#define AD525X_READY_LOW                    no_os_gpio_set_value(dev->gpio_ready,           \
			                    NO_OS_GPIO_LOW)
#define AD525X_READY_HIGH                   no_os_gpio_set_value(dev->gpio_ready,           \
			                    NO_OS_GPIO_HIGH)
#define AD525X_WP_BF_OUT                    no_os_gpio_direction_output(dev->gpio_wpbf,     \
			                    NO_OS_GPIO_HIGH)
#define AD525X_WP_BF_LOW                    no_os_gpio_set_value(dev->gpio_wpbf,            \
			                    NO_OS_GPIO_LOW)
#define AD525X_WP_BF_HIGH                   no_os_gpio_set_value(dev->gpio_wpbf,            \
			                    NO_OS_GPIO_HIGH)

/* Data word masks */
#define AD525X_MEM_ADDR_MASK                0xF
#define AD525X_RDAC_ADDR_MASK_1BIT          0x1
#define AD525X_RDAC_ADDR_MASK_3BIT          0x7
#define AD525X_DATA8_MASK                   0xFF
#define AD525X_DATA10_MASK                  0x3FF

/*********************** Command definitions *********************************/
#define AD525X_CMD_NOP                      0x0
#define AD525X_CMD_MEM2RDAC                 0x1
#define AD525X_CMD_RDAC2MEM                 0x2
#define AD525X_CMD_DECRDAC_6DB              0x3
#define AD525X_CMD_DECALLRDAC_6DB           0x4
#define AD525X_CMD_DECRDAC_ONE              0x5
#define AD525X_CMD_DECALLRDAC_ONE           0x6
#define AD525X_CMD_RESET                    0x7
#define AD525X_CMD_INCRDAC_6DB              0x8
#define AD525X_CMD_INCALLRDAC_6DB           0x9
#define AD525X_CMD_INCRDAC_ONE              0xA
#define AD525X_CMD_INCALLRDAC_ONE           0xB
/*****************************************************************************/

/**************************** SPI specific macros ****************************/
#define AD525X_CMD_SPI_OFFSET               0x4
#define AD525X_CMD_MASK                     0xF
/* SPI Read/Write commands */
#define AD525X_CMD_SPI_SREG2MEM             0x3
#define AD525X_CMD_SPI_MEM2SREG             0x9 // Result in the next frame
#define AD525X_CMD_SPI_RDAC2SREG            0xA // Result in the next frame
#define AD525X_CMD_SPI_SREG2RDAC            0xB

/*************************** I2C specific macros *****************************/
/* I2C device address */
#define AD525X_I2C_HARD_ADDR                0x2C
/* Package pin-programmable address bits */
#define AD525X_I2C_PIN_ADDR_MASK            0x03
/* Mask for I2C Control Bits */
#define AD525X_I2C_CNTR_MASK                0xBF
/* Access Types */
#define AD525X_I2C_CMD_OR_REG               0x80
#define AD525X_I2C_EE_OR_RDAC               0x20
/* Address mask for EEMEM addresses, one bit wider than at devices with SPI */
#define AD525X_I2C_MEM_ADDR_MASK            0x1F
/* The offset of the Command in the I2C word */
#define AD525X_CMD_I2C_OFFSET               0x3

/*************************** Reading tolerance addresses *********************/
/* Note: The valid tolerance addresses varies on different devices. Check the data sheet for further information */
#define AD525x_RDAC_TOLERANCE               0x0F
#define AD525X_RDAC0_SIGN_TOL               0x18
#define AD525X_RDAC0_DECIMAL_TOL            0x19
#define AD525X_RDAC1_SIGN_TOL               0x1A
#define AD525X_RDAC1_DECIMAL_TOL            0x1B
#define AD525X_RDAC2_SIGN_TOL               0x1C
#define AD525X_RDAC2_DECIMAL_TOL            0x1D
#define AD525X_RDAC3_SIGN_TOL               0x1E
#define AD525X_RDAC4_DECIMAL_TOL            0x1F

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Custom boolean type */
/***************************************************************************//**
 * @brief The `bool_t` data structure is an enumeration that defines a custom
 * boolean type with two possible values: `false` and `true`. This enum
 * is used to represent boolean logic in a more readable and type-safe
 * manner within the C programming language, particularly in the context
 * of the AD525X driver for the Microblaze processor.
 *
 * @param false Represents the boolean value false.
 * @param true Represents the boolean value true.
 ******************************************************************************/
enum bool_t {
	false,
	true
};

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad525_x_type_t` is an enumeration that defines a set of constants
 * representing different device types supported by the AD525X driver.
 * Each enumerator corresponds to a specific device model, allowing the
 * software to identify and handle various devices such as AD5232,
 * AD5235, ADN2850, AD5252, AD5251, AD5254, and AD5253. This enumeration
 * is used within the driver to manage device-specific operations and
 * configurations.
 *
 * @param ID_AD5232 Represents the AD5232 device type.
 * @param ID_AD5235 Represents the AD5235 device type.
 * @param ID_ADN2850 Represents the ADN2850 device type.
 * @param ID_AD5252 Represents the AD5252 device type.
 * @param ID_AD5251 Represents the AD5251 device type.
 * @param ID_AD5254 Represents the AD5254 device type.
 * @param ID_AD5253 Represents the AD5253 device type.
 ******************************************************************************/
enum ad525_x_type_t {
	ID_AD5232,
	ID_AD5235,
	ID_ADN2850,
	ID_AD5252,
	ID_AD5251,
	ID_AD5254,
	ID_AD5253,
};

/* Communication types */
/***************************************************************************//**
 * @brief The `comm_type_t` is an enumeration that defines the types of
 * communication protocols supported by the AD525X driver, specifically
 * SPI and I2C. This enumeration is used to specify the communication
 * method for interfacing with the supported devices, allowing the driver
 * to handle different communication protocols as required by the
 * specific device configuration.
 *
 * @param SPI Represents the Serial Peripheral Interface communication type.
 * @param I2C Represents the Inter-Integrated Circuit communication type.
 ******************************************************************************/
enum comm_type_t {
	SPI,
	I2C
};

/***************************************************************************//**
 * @brief The `ad525x_chip_info` structure is used to encapsulate information
 * about a specific AD525X chip, including the number of channels it
 * supports, the communication protocol it uses (SPI or I2C), and the
 * number of adjustable positions it offers. This structure is essential
 * for configuring and interfacing with the chip in various applications,
 * providing a clear definition of its capabilities and communication
 * requirements.
 *
 * @param num_channels Specifies the number of channels available in the chip.
 * @param comm_type Indicates the type of communication protocol used, either
 * SPI or I2C.
 * @param num_position Represents the number of positions or steps available for
 * adjustment in the chip.
 ******************************************************************************/
struct ad525x_chip_info {
	uint8_t num_channels;
	enum comm_type_t comm_type;
	uint16_t num_position;
};

/***************************************************************************//**
 * @brief The `ad525x_dev` structure is designed to encapsulate the necessary
 * descriptors and settings for interfacing with AD525X series devices,
 * which are digital potentiometers. It includes pointers to I2C and SPI
 * descriptors for communication, as well as GPIO descriptors for various
 * control pins such as reset, shutdown, ready, and write protect/buffer.
 * Additionally, it holds an enumeration to specify the exact type of
 * AD525X device being used, allowing for flexible and device-specific
 * operations.
 *
 * @param i2c_desc Pointer to an I2C descriptor for I2C communication.
 * @param spi_desc Pointer to an SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_shutdown Pointer to a GPIO descriptor for the shutdown pin.
 * @param gpio_ready Pointer to a GPIO descriptor for the ready pin.
 * @param gpio_wpbf Pointer to a GPIO descriptor for the write protect/buffer
 * pin.
 * @param this_device Enumeration indicating the specific AD525X device type.
 ******************************************************************************/
struct ad525x_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_shutdown;
	struct no_os_gpio_desc	*gpio_ready;
	struct no_os_gpio_desc	*gpio_wpbf;
	/* Device Settings */
	enum ad525_x_type_t	this_device;
};

/***************************************************************************//**
 * @brief The `ad525x_init_param` structure is used to encapsulate all the
 * necessary initialization parameters for setting up an AD525X device.
 * It includes configurations for both I2C and SPI communication
 * interfaces, as well as GPIO settings for various control pins such as
 * reset, shutdown, ready, and write protect/buffer. Additionally, it
 * specifies the particular type of AD525X device being used, allowing
 * for flexible initialization of different devices within the AD525X
 * family.
 *
 * @param i2c_init Holds the initialization parameters for I2C communication.
 * @param spi_init Holds the initialization parameters for SPI communication.
 * @param gpio_reset Defines the GPIO initialization parameters for the reset
 * pin.
 * @param gpio_shutdown Defines the GPIO initialization parameters for the
 * shutdown pin.
 * @param gpio_ready Defines the GPIO initialization parameters for the ready
 * pin.
 * @param gpio_wpbf Defines the GPIO initialization parameters for the write
 * protect/buffer pin.
 * @param this_device Specifies the type of AD525X device being initialized.
 ******************************************************************************/
struct ad525x_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_reset;
	struct no_os_gpio_init_param	gpio_shutdown;
	struct no_os_gpio_init_param	gpio_ready;
	struct no_os_gpio_init_param	gpio_wpbf;
	/* Device Settings */
	enum ad525_x_type_t	this_device;
};

/*****************************************************************************/
/*  Functions Prototypes                                                     */
/*****************************************************************************/
/* Initialize the communication with the device */
/***************************************************************************//**
 * @brief This function sets up the communication interface and GPIOs for an
 * AD525X device, preparing it for further operations. It must be called
 * before any other operations on the device. The function allocates
 * memory for the device structure and initializes the communication
 * interface based on the specified device type, either SPI or I2C. It
 * also configures several GPIO pins for device control. If memory
 * allocation fails or any initialization step encounters an error, the
 * function returns a negative status code.
 *
 * @param device A pointer to a pointer of type `struct ad525x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad525x_init_param` containing
 * initialization parameters for the device, including
 * communication and GPIO settings. The `this_device` field
 * specifies the type of AD525X device to initialize.
 * @return Returns 0 on success or a negative error code if initialization
 * fails.
 ******************************************************************************/
int8_t ad525x_init(struct ad525x_dev **device,
		   struct ad525x_init_param init_param);

/* Free the resources allocated by ad525x_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an AD525X
 * device when it is no longer needed. This includes removing any
 * associated SPI or I2C descriptors and GPIO configurations. It is
 * important to call this function to prevent resource leaks after the
 * device is no longer in use. Ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad525x_dev structure representing the device to be
 * removed. Must not be null. The function will handle invalid or
 * uninitialized devices gracefully by checking the communication
 * type and associated GPIOs.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource removal process. A return value of 0 typically indicates
 * success, while a non-zero value indicates an error occurred during
 * the removal of resources.
 ******************************************************************************/
int32_t ad525x_remove(struct ad525x_dev *dev);

/* Read data from the EEMEM */
/***************************************************************************//**
 * @brief This function retrieves data from the EEMEM of a specified AD525x
 * device at a given memory address. It supports both SPI and I2C
 * communication interfaces, automatically selecting the appropriate
 * protocol based on the device configuration. The function should be
 * called after the device has been properly initialized using
 * `ad525x_init`. It is important to ensure that the `address` parameter
 * is within the valid range for the specific device being used. The
 * function returns the data read from the specified memory location.
 *
 * @param dev A pointer to an `ad525x_dev` structure representing the device
 * from which to read. This must be a valid, initialized device
 * structure. The caller retains ownership and must ensure it is not
 * null.
 * @param address An 8-bit unsigned integer specifying the memory address to
 * read from. The valid range depends on the specific AD525x
 * device being used. Invalid addresses may result in undefined
 * behavior.
 * @return Returns a 16-bit unsigned integer containing the data read from the
 * specified EEMEM address.
 ******************************************************************************/
uint16_t ad525x_read_mem(struct ad525x_dev *dev,
			 uint8_t address);

/* Write data to EEMEM */
/***************************************************************************//**
 * @brief This function writes a specified data value to a given address in the
 * EEMEM of an AD525x device. It supports both SPI and I2C communication
 * interfaces, automatically selecting the appropriate protocol based on
 * the device configuration. The function should be called when the
 * device is properly initialized and ready for communication. It is
 * important to ensure that the address and data values are within the
 * valid range for the specific device model being used. The function
 * does not return a value, and any errors in communication are not
 * reported back to the caller.
 *
 * @param dev A pointer to an initialized ad525x_dev structure representing the
 * device. Must not be null.
 * @param address The memory address in the EEMEM where the data will be
 * written. Valid range depends on the specific device model.
 * @param data The data to be written to the specified EEMEM address. The valid
 * range depends on the specific device model.
 * @return None
 ******************************************************************************/
void ad525x_write_mem(struct ad525x_dev *dev,
		      uint8_t address,
		      uint16_t data);

/* Read the value of the RDAC register */
/***************************************************************************//**
 * @brief This function retrieves the current value stored in the RDAC register
 * of an AD525x device at a given address. It supports both SPI and I2C
 * communication interfaces, automatically selecting the appropriate
 * protocol based on the device configuration. The function should be
 * called when the current RDAC value is needed for further processing or
 * monitoring. Ensure that the device is properly initialized before
 * calling this function. The function handles different data word sizes
 * depending on the specific device model.
 *
 * @param dev A pointer to an initialized ad525x_dev structure representing the
 * device. Must not be null.
 * @param address The address of the RDAC register to read from. It should be
 * within the valid range defined by the device's memory address
 * mask.
 * @return Returns the 8-bit or 10-bit value read from the RDAC register,
 * depending on the device model.
 ******************************************************************************/
uint16_t ad525x_read_rdac(struct ad525x_dev *dev,
			  uint8_t address);

/* Write the value of the RDAC register */
/***************************************************************************//**
 * @brief This function is used to write a specified value to the RDAC register
 * of an AD525x device. It supports devices with both SPI and I2C
 * communication interfaces. The function must be called with a valid
 * device structure that has been initialized using `ad525x_init`. The
 * address parameter specifies the RDAC register to be written, and the
 * data parameter contains the value to be written. The function handles
 * communication protocol differences internally and includes a delay to
 * ensure the write operation is completed.
 *
 * @param dev A pointer to an `ad525x_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param address An 8-bit unsigned integer specifying the RDAC register
 * address. The valid range is determined by the device's
 * capabilities, typically masked by `AD525X_MEM_ADDR_MASK`.
 * @param data A 16-bit unsigned integer representing the data to be written to
 * the RDAC register. The valid range depends on the specific
 * device's data width.
 * @return None
 ******************************************************************************/
void ad525x_write_rdac(struct ad525x_dev *dev,
		       uint8_t address,
		       uint16_t data);

/* Write quick commands to the device */
/***************************************************************************//**
 * @brief Use this function to send a specific command to an AD525X device,
 * which can be communicated with via either SPI or I2C interfaces. This
 * function is typically used to perform quick operations on the device,
 * such as resetting or incrementing/decrementing the RDAC values. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function handles the necessary
 * adjustments for command representations and ensures the correct data
 * format is sent based on the communication protocol in use. A delay is
 * introduced after the command is sent to ensure proper execution.
 *
 * @param dev A pointer to an ad525x_dev structure representing the device. Must
 * not be null and should be properly initialized with the correct
 * communication descriptors.
 * @param command An 8-bit unsigned integer representing the command to be sent.
 * The command should be within the valid range defined by the
 * device's command set.
 * @param address An 8-bit unsigned integer representing the address to which
 * the command applies. The address should be valid according to
 * the device's addressing scheme.
 * @return None
 ******************************************************************************/
void ad525x_write_command(struct ad525x_dev *dev,
			  uint8_t command,
			  uint8_t address);

#endif // __AD525X_H__
