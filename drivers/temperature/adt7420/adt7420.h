/***************************************************************************//**
 *   @file   adt7420.h
 *   @brief  Header file of ADT7420 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012, 2019, 2021(c) Analog Devices, Inc.
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
#ifndef __ADT7420_H__
#define __ADT7420_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no_os_spi.h"
#include "no_os_i2c.h"
#include "no_os_util.h"
#include <stdbool.h>

/******************************************************************************/
/************************** ADT7420 Definitions *******************************/
/******************************************************************************/

/* ADT7420 address */
#define ADT7420_A0_PIN(x)		(((x) & 0x1) << 0) // I2C Serial Bus Address Selection Pin
#define ADT7420_A1_PIN(x)		(((x) & 0x1) << 1) // I2C Serial Bus Address Selection Pin
#define ADT7420_ADDRESS(x,y)		(0x48 + ADT7420_A1_PIN(x) + ADT7420_A0_PIN(y))
#define ADT7320_L16				NO_OS_BIT(8)	//indicator that register is 16-bit
#define ADT7320_ADDR_MSK		NO_OS_GENMASK(7,0)

/* ADT7320  (SPI) registers */
#define ADT7320_REG_STATUS		0x00 // Status
#define ADT7320_REG_CONFIG		0x01 // Configuration
#define ADT7320_REG_TEMP    	(ADT7320_L16 | 0x02) // Temperature value
#define ADT7320_REG_ID			0x03 // ID
#define ADT7320_REG_T_CRIT		(ADT7320_L16 | 0x04) // Temperature CRIT setpoint (147'C)
#define ADT7320_REG_HIST		0x05 // Temperature HYST setpoint (5'C)
#define ADT7320_REG_T_HIGH    		(ADT7320_L16 | 0x06) // Temperature HIGH setpoint (64'C)
#define ADT7320_REG_T_LOW    		(ADT7320_L16 | 0x07) // Temperature LOW setpoint (10'C)

/* ADT7320 SPI command byte */
#define ADT7320_WRITE_MASK_CMD		0b00111000 // SPI write command
#define ADT7320_READ_CMD		0b01000000 // SPI read command

/* ADT7420 (I2C) registers */
#define ADT7420_REG_TEMP_MSB		(ADT7320_L16 |0x00) // Temperature value MSB
#define ADT7420_REG_TEMP_LSB		0x01 // Temperature value LSB
#define ADT7420_REG_STATUS		0x02 // Status
#define ADT7420_REG_CONFIG		0x03 // Configuration
#define ADT7420_REG_T_HIGH_MSB		(ADT7320_L16 |0x04 )// Temperature HIGH setpoint MSB
#define ADT7420_REG_T_HIGH_LSB		0x05 // Temperature HIGH setpoint LSB
#define ADT7420_REG_T_LOW_MSB		(ADT7320_L16 |0x06) // Temperature LOW setpoint MSB
#define ADT7420_REG_T_LOW_LSB		0x07 // Temperature LOW setpoint LSB
#define ADT7420_REG_T_CRIT_MSB		(ADT7320_L16 |0x08) // Temperature CRIT setpoint MSB
#define ADT7420_REG_T_CRIT_LSB		0x09 // Temperature CRIT setpoint LSB
#define ADT7420_REG_HIST		0x0A // Temperature HYST setpoint
#define ADT7420_REG_ID			0x0B // ID
#define ADT7420_REG_RESET		0x2F // Software reset

/* ADT7420_REG_STATUS definition */
#define ADT7420_STATUS_T_LOW		NO_OS_BIT(4)
#define ADT7420_STATUS_T_HIGH		NO_OS_BIT(5)
#define ADT7420_STATUS_T_CRIT		NO_OS_BIT(6)
#define ADT7420_STATUS_RDY		NO_OS_BIT(7)

/* ADT7420_REG_CONFIG definition */
#define ADT7420_CONFIG_OP_MODE(x)	((x) << 5) & (NO_OS_GENMASK(6,5))
#define ADT7420_CONFIG_RESOLUTION	NO_OS_BIT(7)

/* ADT7420 temperature conversion definitions */
#define ADT7420_16BIT_NEG			NO_OS_BIT(16)
#define ADT7420_16BIT_SIGN			0x8000
#define ADT7420_16BIT_DIV			128
#define ADT7420_13BIT_NEG			NO_OS_BIT(13)
#define ADT7420_13BIT_SIGN			0x1000
#define ADT7420_13BIT_DIV			16

/* ADT7420_CONFIG_OP_MODE(x) options */
#define ADT7420_OP_MODE_CONT_CONV	0
#define ADT7420_OP_MODE_ONE_SHOT	1
#define ADT7420_OP_MODE_1_SPS		2
#define ADT7420_OP_MODE_SHUTDOWN	3

/* ADT7420 default ID */
#define ADT7xxx_ID			0xC

#define ADT7420_RESET_DELAY 		1


/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `dev_interface` enumeration defines the types of communication
 * interfaces available for a device, specifically SPI and I2C. This
 * enumeration is used to specify the communication protocol that a
 * device will use to interact with other components or systems, allowing
 * for flexibility in device configuration and communication setup.
 *
 * @param SPI Represents the Serial Peripheral Interface communication protocol.
 * @param I2C Represents the Inter-Integrated Circuit communication protocol.
 ******************************************************************************/
enum dev_interface {
	SPI,
	I2C,
};

/***************************************************************************//**
 * @brief The `adt7420_chip_info` structure is used to encapsulate information
 * about the ADT7420 temperature sensor chip, specifically its resolution
 * and the communication interface it uses. This structure is essential
 * for configuring and interfacing with the chip, allowing the software
 * to understand how to communicate with the device and interpret its
 * data correctly.
 *
 * @param resolution Specifies the resolution of the ADT7420 chip, typically in
 * bits.
 * @param communication Indicates the communication interface used by the
 * ADT7420 chip, either SPI or I2C.
 ******************************************************************************/
struct adt7420_chip_info {
	uint8_t			resolution;
	enum dev_interface  communication;
};

/***************************************************************************//**
 * @brief The `adt7420_type` enumeration defines a set of constants representing
 * different types of temperature sensor devices from the ADT74xx and
 * ADT73xx series. Each enumerator corresponds to a specific model of the
 * sensor, allowing the software to identify and handle different device
 * types appropriately within the driver code.
 *
 * @param ID_ADT7410 Represents the ADT7410 device type.
 * @param ID_ADT7420 Represents the ADT7420 device type.
 * @param ID_ADT7422 Represents the ADT7422 device type.
 * @param ID_ADT7310 Represents the ADT7310 device type.
 * @param ID_ADT7311 Represents the ADT7311 device type.
 * @param ID_ADT7312 Represents the ADT7312 device type.
 * @param ID_ADT7320 Represents the ADT7320 device type.
 ******************************************************************************/
enum adt7420_type {
	ID_ADT7410,
	ID_ADT7420,
	ID_ADT7422,
	ID_ADT7310,
	ID_ADT7311,
	ID_ADT7312,
	ID_ADT7320
};

/***************************************************************************//**
 * @brief The `adt7420_dev` structure is designed to encapsulate the necessary
 * components for interfacing with an ADT7420 temperature sensor device.
 * It includes descriptors for both I2C and SPI communication, allowing
 * flexibility in the communication protocol used. The structure also
 * holds information about the specific type of ADT7420 device being used
 * and its resolution setting, which are crucial for configuring and
 * operating the sensor correctly.
 *
 * @param i2c_desc Pointer to a descriptor for I2C communication.
 * @param spi_desc Pointer to a descriptor for SPI communication.
 * @param active_device Enumeration indicating the type of ADT7420 device in
 * use.
 * @param resolution_setting 8-bit unsigned integer representing the resolution
 * setting of the device.
 ******************************************************************************/
struct adt7420_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* Device Settings */
	enum adt7420_type active_device;
	/* Device Settings */
	uint8_t		resolution_setting;

};

/***************************************************************************//**
 * @brief The `adt7420_init_param` structure is used to initialize an ADT7420
 * device, specifying the communication interface (I2C or SPI) and
 * device-specific settings such as resolution and device type. This
 * structure facilitates the configuration of the device during
 * initialization, ensuring that the correct interface and settings are
 * applied for the desired operation.
 *
 * @param interface_init A union that holds initialization parameters for either
 * I2C or SPI interfaces.
 * @param resolution_setting A uint8_t value representing the resolution setting
 * of the device.
 * @param active_device An enum indicating the type of ADT7420 device being
 * used.
 ******************************************************************************/
struct adt7420_init_param {
/***************************************************************************//**
 * @brief The `interface_type` union is designed to encapsulate initialization
 * parameters for two different communication interfaces, I2C and SPI,
 * within a single data structure. This allows for flexible
 * initialization of a device that can operate over either interface,
 * depending on the specific requirements of the application. The union
 * provides a way to store the necessary parameters for either interface
 * without allocating separate memory for both, thus optimizing resource
 * usage.
 *
 * @param i2c_init Holds initialization parameters for I2C communication.
 * @param spi_init Holds initialization parameters for SPI communication.
 ******************************************************************************/
	union interface_type {
		/* I2C */
		struct no_os_i2c_init_param	i2c_init;
		/* SPI */
		struct no_os_spi_init_param	spi_init;
	} interface_init;
	/* Device Settings */
	uint8_t		resolution_setting;
	/* Device Settings */
	enum adt7420_type	active_device;
};

/***************************************************************************//**
 * @brief The `chip_info` variable is an external constant array of
 * `adt7420_chip_info` structures. Each element in this array contains
 * information about the resolution and communication interface (SPI or
 * I2C) for different ADT7420 chip variants.
 *
 * @details This variable is used to store and provide access to the
 * configuration details of various ADT7420 chip models, facilitating
 * their initialization and operation in the driver.
 ******************************************************************************/
extern const struct adt7420_chip_info chip_info[];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to read the value of a specific register from an
 * ADT7420 device. It determines the communication interface (SPI or I2C)
 * based on the device configuration and performs the read operation
 * accordingly. This function is typically called when you need to
 * retrieve configuration or status information from the device. Ensure
 * that the device has been properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an adt7420_dev structure representing the device.
 * Must not be null and should be properly initialized.
 * @param register_address The address of the register to read from. It should
 * be a valid register address for the ADT7420 device.
 * @param data A pointer to a uint16_t variable where the read register value
 * will be stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the read operation.
 ******************************************************************************/
int adt7420_reg_read(struct adt7420_dev *dev,
		     uint16_t register_address, uint16_t *data);

/* Read-modify-write operation*/
/***************************************************************************//**
 * @brief Use this function to update specific bits of a register in the ADT7420
 * device by applying a mask and a new value. This function is useful
 * when you need to change only certain bits of a register without
 * affecting the other bits. It first reads the current value of the
 * register, applies the mask to clear the bits to be updated, and then
 * sets the new value. This function should be called when the device is
 * properly initialized and communication is established. It returns an
 * error code if the read or write operation fails.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null.
 * @param register_address The address of the register to be updated. Must be a
 * valid register address for the ADT7420 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param value The new value to be written to the bits specified by the mask.
 * Only the bits corresponding to the mask will be updated.
 * @return Returns 0 on success, or -1 if the read or write operation fails.
 ******************************************************************************/
int adt7420_reg_update_bits(struct adt7420_dev *dev,
			    uint16_t register_address, uint8_t mask, uint8_t value);

/* Write to SPI register */
/***************************************************************************//**
 * @brief This function is used to write data to a specific register of the
 * ADT7420 device using the SPI interface. It is essential to ensure that
 * the device has been properly initialized and configured for SPI
 * communication before calling this function. The function determines
 * the number of bytes to write based on the register address, supporting
 * both 8-bit and 16-bit registers. It constructs the appropriate command
 * and data bytes for the SPI transaction. This function is typically
 * used when configuring the device or updating its settings.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null, and the device must be configured for
 * SPI communication.
 * @param register_address A 16-bit value specifying the address of the register
 * to write to. The address must be valid for the
 * ADT7420 device.
 * @param data A 32-bit value containing the data to be written to the specified
 * register. The relevant portion of this data is used based on the
 * register's size (8-bit or 16-bit).
 * @return Returns an integer status code from the SPI write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int adt7420_spi_reg_write(struct adt7420_dev *dev, uint16_t register_address,
			  uint32_t data);

/* Write to I2C register */
/***************************************************************************//**
 * @brief Use this function to write data to a specific register of an ADT7420
 * device over the I2C interface. It is essential to ensure that the
 * device has been properly initialized and is ready for communication
 * before calling this function. The function handles both 8-bit and
 * 16-bit register writes based on the register address provided. It is
 * important to pass valid register addresses and data values to avoid
 * unexpected behavior. The function returns an integer status code
 * indicating the success or failure of the write operation.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null.
 * @param register_address A 16-bit unsigned integer specifying the register
 * address to write to. Must be a valid register address
 * for the ADT7420 device.
 * @param data A 32-bit unsigned integer containing the data to be written to
 * the specified register. The relevant portion of the data is used
 * based on the register's expected size.
 * @return Returns an integer status code from the underlying I2C write
 * operation, where 0 typically indicates success and a negative value
 * indicates an error.
 ******************************************************************************/
int adt7420_i2c_reg_write(struct adt7420_dev *dev, uint16_t register_address,
			  uint32_t data);

/*Sets register value*/
/***************************************************************************//**
 * @brief This function writes a 32-bit data value to a specified register of
 * the ADT7420 device, using either SPI or I2C communication based on the
 * device configuration. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * determines the communication protocol by checking the device's
 * interface type and then delegates the write operation to the
 * appropriate protocol-specific function. This function is typically
 * used to configure device settings or update operational parameters.
 *
 * @param dev A pointer to an adt7420_dev structure representing the device.
 * Must not be null and should be properly initialized.
 * @param register_address A 16-bit unsigned integer specifying the address of
 * the register to write to. The address must be valid
 * for the device's register map.
 * @param data A 32-bit unsigned integer containing the data to be written to
 * the specified register.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the write operation.
 ******************************************************************************/
int adt7420_reg_write(struct adt7420_dev *dev,
		      uint16_t register_address,
		      uint32_t data);

/***************************************************************************//**
 * @brief This function sets up the ADT7420 device by initializing the specified
 * communication interface, either SPI or I2C, and verifying the device's
 * presence. It should be called before any other operations on the
 * device to ensure proper setup. The function allocates memory for the
 * device structure and configures it based on the provided
 * initialization parameters. If the initialization is successful, the
 * device pointer is updated to point to the newly created device
 * structure. The function handles errors by returning a negative status
 * code, and it is the caller's responsibility to handle these errors
 * appropriately.
 *
 * @param device A double pointer to an adt7420_dev structure. This must not be
 * null, and upon successful initialization, it will point to the
 * newly allocated and initialized device structure. The caller is
 * responsible for managing the memory of this structure.
 * @param init_param A structure of type adt7420_init_param containing the
 * initialization parameters for the device. This includes the
 * communication interface settings and device-specific
 * settings. The structure must be properly populated before
 * calling the function.
 * @return Returns 0 on successful initialization. If an error occurs, a
 * negative error code is returned, indicating the type of failure.
 ******************************************************************************/
int32_t adt7420_init(struct adt7420_dev **device,
		     struct adt7420_init_param init_param);

/* Free the resources allocated by adt7420_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an ADT7420
 * device after it is no longer needed. This function should be called to
 * clean up after a successful initialization with `adt7420_init`. It
 * handles both SPI and I2C interfaces, ensuring that the appropriate
 * communication descriptor is removed. The function returns an error
 * code if the removal of the communication descriptor fails.
 *
 * @param dev A pointer to an `adt7420_dev` structure representing the device to
 * be removed. Must not be null. The caller retains ownership of the
 * pointer, but the resources it points to will be freed.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the communication descriptor removal fails.
 ******************************************************************************/
int32_t adt7420_remove(struct adt7420_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure the operational mode of an ADT7420
 * device, which can be in continuous conversion, one-shot, 1 SPS, or
 * shutdown mode. This function should be called after the device has
 * been initialized and is ready for configuration. The function updates
 * the device's configuration register to reflect the desired mode. It is
 * important to ensure that the device structure is properly initialized
 * and that the mode parameter is within the valid range of operational
 * modes.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param mode A uint8_t value representing the desired operational mode. Valid
 * values are 0 (continuous conversion), 1 (one-shot), 2 (1 SPS),
 * and 3 (shutdown). Passing an invalid mode may result in undefined
 * behavior.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the operation.
 ******************************************************************************/
int adt7420_set_operation_mode(struct adt7420_dev *dev,
			       uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to configure the resolution setting of an ADT7420
 * device. This function should be called after the device has been
 * initialized and is ready for configuration. The resolution parameter
 * determines the precision of temperature measurements. The function
 * updates the device's configuration register to reflect the new
 * resolution setting and stores the setting in the device structure. It
 * returns a status code indicating the success or failure of the
 * operation.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null.
 * @param resolution A uint8_t value representing the desired resolution
 * setting. Valid values depend on the device's capabilities
 * and should be within the range supported by the device.
 * Invalid values may result in undefined behavior.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int adt7420_set_resolution(struct adt7420_dev *dev,
			   uint8_t resolution);

/***************************************************************************//**
 * @brief Use this function to perform a software reset on the ADT7420 device,
 * which can be connected via either SPI or I2C interface. This function
 * should be called when a reset of the device is required, such as after
 * initialization or when the device is not responding as expected. The
 * function will determine the communication interface in use and send
 * the appropriate reset command. It is important to ensure that the
 * device structure is properly initialized and that the communication
 * interface is correctly configured before calling this function. The
 * function will block for a short delay to allow the device to restart.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device to reset. Must not be null. The structure should have the
 * appropriate SPI or I2C descriptor initialized based on the
 * communication interface in use.
 * @return Returns 0 on success, or -1 if the reset command fails to be sent
 * over the communication interface.
 ******************************************************************************/
int32_t adt7420_reset(struct adt7420_dev *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from an
 * ADT7420 device, expressed in degrees Celsius. It is essential to
 * ensure that the device has been properly initialized and configured
 * before calling this function. The function automatically determines
 * whether the device is using SPI or I2C communication and reads the
 * temperature accordingly. It also handles different resolution settings
 * to provide an accurate temperature reading. This function is typically
 * called when a temperature measurement is needed from the device.
 *
 * @param dev A pointer to an initialized adt7420_dev structure representing the
 * device. Must not be null. The device should be properly configured
 * and initialized before calling this function.
 * @return Returns the temperature in degrees Celsius as a float. The value is
 * derived from the device's current temperature reading.
 ******************************************************************************/
float adt7420_get_temperature(struct adt7420_dev *dev);

/***************************************************************************//**
 * @brief This function reads the value of a specified register from an ADT7420
 * device using the I2C interface. It is intended for use with devices
 * that are configured to communicate over I2C. The function requires a
 * valid device structure and a register address to read from. It
 * determines the number of bytes to read based on the register address
 * and stores the result in the provided data pointer. The function must
 * be called with a properly initialized device structure, and the data
 * pointer must not be null. It returns an error code if the I2C
 * communication fails.
 *
 * @param dev A pointer to an adt7420_dev structure representing the device.
 * Must not be null and must be initialized for I2C communication.
 * @param register_address The address of the register to read from. It is a
 * 16-bit value, and the function determines the number
 * of bytes to read based on this address.
 * @param data A pointer to a 16-bit variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if an I2C communication error occurs.
 ******************************************************************************/
int adt7420_i2c_reg_read(struct adt7420_dev *dev,
			 uint16_t register_address, uint16_t *data);

/***************************************************************************//**
 * @brief This function reads the value of a specified register from an ADT7420
 * device using the SPI interface. It is intended for use with devices
 * that are configured to communicate over SPI. The function requires a
 * valid device structure and a register address to read from. The result
 * is stored in the provided data pointer. The function must be called
 * with a properly initialized device structure, and the data pointer
 * must not be null. It returns an error code if the read operation
 * fails.
 *
 * @param dev A pointer to an adt7420_dev structure representing the device.
 * Must be initialized and not null. The caller retains ownership.
 * @param register_address The address of the register to read from. Must be a
 * valid register address for the ADT7420 device.
 * @param data A pointer to a uint16_t where the read data will be stored. Must
 * not be null. The function writes the register value to this
 * location.
 * @return Returns 0 on success, or -1 if the SPI read operation fails.
 ******************************************************************************/
int adt7420_spi_reg_read(struct adt7420_dev *dev,
			 uint16_t register_address, uint16_t *data);

/***************************************************************************//**
 * @brief Use this function to determine whether the active device in the
 * provided `adt7420_dev` structure is configured to use the SPI
 * communication interface. This is useful when the device supports
 * multiple communication protocols and you need to ensure that the
 * correct interface is being used for subsequent operations. The
 * function should be called with a valid `adt7420_dev` structure that
 * has been properly initialized. It does not modify the input structure
 * or have any side effects.
 *
 * @param dev A pointer to an `adt7420_dev` structure representing the device.
 * This structure must be initialized and must not be null. If the
 * pointer is invalid or null, the behavior is undefined.
 * @return Returns `true` if the device's communication interface is SPI,
 * otherwise returns `false`.
 ******************************************************************************/
bool adt7420_is_spi(struct adt7420_dev *dev);

#endif	/* __ADT7420_H__ */
