/***************************************************************************//**
 *   @file   max31875.h
 *   @brief  Header file of MAX31875 Driver.
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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
#ifndef __MAX31875_H__
#define __MAX31875_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "no_os_i2c.h"
#include "no_os_util.h"

/******************************************************************************/
/************************** MAX31875 Definitions ******************************/
/******************************************************************************/

/** Register addresses */
#define MAX31875_TEMPERATURE_REG	0x0
#define MAX31875_CONFIGURATION_REG	0x1
#define MAX31875_THYST_REG		0x2
#define MAX31875_TOS_REG		0x3

/** MAX31875_TEMPERATURE_REG */
#define MAX31875_OVER_TEMP_STATUS_MASK	NO_OS_BIT(15)
#define MAX31875_FAULT_QUEUE_MASK	NO_OS_GENMASK(12, 11)
#define MAX31875_COMP_INT_MASK		NO_OS_BIT(9)
#define MAX31875_SHUTDOWN_MASK		NO_OS_BIT(8)
#define MAX31875_DATA_FORMAT_MASK	NO_OS_BIT(7)
#define MAX31875_RESOLUTION_MASK	NO_OS_GENMASK(6, 5)
#define MAX31875_TIMEOUT_MASK		NO_OS_BIT(4)
#define MAX31875_PEC_MASK		NO_OS_BIT(3)
#define MAX31875_CONVERSION_RATE_MASK	NO_OS_GENMASK(2, 1)
#define MAX31875_ONESHOT_MASK		NO_OS_BIT(0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `max31875_i2c_addr` enumeration defines a set of constants
 * representing the possible I2C addresses for the MAX31875 temperature
 * sensor device. Each enumerator corresponds to a unique I2C address
 * that the device can be configured to use, allowing for multiple
 * devices to be connected on the same I2C bus without address conflicts.
 *
 * @param MAX31875R0 Represents the I2C address 0x48 for the MAX31875 device.
 * @param MAX31875R1 Represents the I2C address 0x49 for the MAX31875 device.
 * @param MAX31875R2 Represents the I2C address 0x4A for the MAX31875 device.
 * @param MAX31875R3 Represents the I2C address 0x4B for the MAX31875 device.
 * @param MAX31875R4 Represents the I2C address 0x4C for the MAX31875 device.
 * @param MAX31875R5 Represents the I2C address 0x4D for the MAX31875 device.
 * @param MAX31875R6 Represents the I2C address 0x4E for the MAX31875 device.
 * @param MAX31875R7 Represents the I2C address 0x4F for the MAX31875 device.
 ******************************************************************************/
enum max31875_i2c_addr {
	MAX31875R0 = 0x48,
	MAX31875R1 = 0x49,
	MAX31875R2 = 0x4A,
	MAX31875R3 = 0x4B,
	MAX31875R4 = 0x4C,
	MAX31875R5 = 0x4D,
	MAX31875R6 = 0x4E,
	MAX31875R7 = 0x4F
};

/***************************************************************************//**
 * @brief The `max31875_dev` structure is a handler for the MAX31875 temperature
 * sensor device, encapsulating the necessary I2C descriptor required for
 * communication with the device. This structure is used to manage the
 * state and operations of the MAX31875 sensor, facilitating interactions
 * such as reading and writing to the device's registers over the I2C
 * interface.
 *
 * @param i2c_desc A pointer to a no_os_i2c_desc structure, representing the I2C
 * descriptor for communication.
 ******************************************************************************/
struct max31875_dev {
	/* I2C */
	struct no_os_i2c_desc *i2c_desc;
};

/***************************************************************************//**
 * @brief The `max31875_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the MAX31875
 * temperature sensor driver. It primarily focuses on the I2C
 * communication setup, as indicated by its sole member, `i2c_init`,
 * which holds the necessary I2C initialization parameters. This
 * structure is essential for configuring the I2C interface before the
 * MAX31875 device can be used for temperature measurements.
 *
 * @param i2c_init This member is a structure of type no_os_i2c_init_param, used
 * to initialize I2C communication parameters.
 ******************************************************************************/
struct max31875_init_param {
	/* I2C */
	struct no_os_i2c_init_param i2c_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function reads a 16-bit value from a specified register of the
 * MAX31875 device using the I2C interface. It should be called when you
 * need to retrieve data from a specific register of the device. The
 * function requires a valid device structure and a pointer to store the
 * read value. It returns an error code if the device structure or the
 * read value pointer is null, or if the I2C communication fails.
 *
 * @param dev A pointer to a max31875_dev structure representing the device.
 * Must not be null. The structure should be properly initialized
 * before calling this function.
 * @param reg The register address to read from. It should be a valid register
 * address defined for the MAX31875 device.
 * @param readval A pointer to a uint32_t where the read value will be stored.
 * Must not be null. The function writes the 16-bit register
 * value to this location.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., invalid parameters or I2C
 * communication failure).
 ******************************************************************************/
int32_t max31875_reg_read(struct max31875_dev *dev,
			  uint32_t reg,
			  uint32_t *readval);

/***************************************************************************//**
 * @brief This function writes a 16-bit value to a specified register of the
 * MAX31875 temperature sensor using the I2C interface. It should be
 * called when you need to configure or update the settings of the
 * device. The function requires a valid device structure, which must be
 * initialized prior to calling this function. If the device structure is
 * null, the function will return an error. This function is typically
 * used in conjunction with other device operations to manage the
 * sensor's configuration.
 *
 * @param dev A pointer to a max31875_dev structure representing the device.
 * This must not be null, and the device must be properly initialized
 * before use. If null, the function returns an error.
 * @param reg The register address to which the value will be written. It should
 * be a valid register address defined for the MAX31875 device.
 * @param writeval The 16-bit value to be written to the specified register. The
 * value is split into two bytes for transmission over I2C.
 * @return Returns 0 on success or a negative error code if the device structure
 * is null or if the I2C write operation fails.
 ******************************************************************************/
int32_t max31875_reg_write(struct max31875_dev *dev,
			   uint32_t reg,
			   uint32_t writeval);

/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the
 * MAX31875 device by applying a mask to isolate the field of interest.
 * This function is useful when only a portion of a register needs to be
 * updated without affecting other bits. It must be called with a valid
 * device structure that has been initialized. The function reads the
 * current register value, applies the mask to clear the targeted bits,
 * and then sets them to the new value before writing back to the
 * register. If the device structure is null, the function returns an
 * error.
 *
 * @param dev Pointer to a max31875_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg The register address to be accessed. Must be a valid register
 * address for the MAX31875 device.
 * @param val The new value to be written to the specified field within the
 * register. It is masked and applied to the register.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Only the bits set in this mask will be affected by the
 * operation.
 * @return Returns 0 on success or a negative error code on failure, such as if
 * the device structure is null or if the read/write operations fail.
 ******************************************************************************/
int32_t max31875_reg_write_mask(struct max31875_dev *dev,
				uint32_t reg,
				uint32_t val,
				uint32_t mask);

/***************************************************************************//**
 * @brief This function sets up the MAX31875 device driver by allocating
 * necessary resources and initializing the I2C interface. It must be
 * called before any other operations on the MAX31875 device. The
 * function requires valid pointers for both the device structure and
 * initialization parameters. If initialization fails, it returns an
 * error code and ensures that allocated resources are freed.
 *
 * @param device A pointer to a pointer of type `struct max31875_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct max31875_init_param` containing
 * initialization parameters for the device. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -EINVAL for
 * invalid arguments or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int32_t max31875_init(struct max31875_dev **device,
		      struct max31875_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release resources allocated for a
 * MAX31875 device instance when it is no longer needed. This function
 * should be called to clean up after a successful initialization with
 * `max31875_init`. It ensures that the I2C descriptor associated with
 * the device is removed and the memory allocated for the device
 * structure is freed. Calling this function with a null pointer will
 * result in an error.
 *
 * @param dev A pointer to a `max31875_dev` structure representing the device
 * instance to be removed. Must not be null. If null, the function
 * returns an error code.
 * @return Returns 0 on successful removal of the device resources. If the input
 * is null or an error occurs during the I2C descriptor removal, a
 * negative error code is returned.
 ******************************************************************************/
int32_t max31875_remove(struct max31875_dev *dev);

#endif	/* __MAX31875_H__ */
