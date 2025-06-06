/***************************************************************************//**
 *   @file   adt75.h
 *   @brief  Header file of ADT75 Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
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
#ifndef _ADT75_H
#define _ADT75_H

#include <stdint.h>
#include "no_os_i2c.h"

/** x is set based on the value of A2, A1, A0 pins */
#define ADT75_ADDR(x)		(0x48 + (x))
#define ADT75_CONV_DELAY_MS     40
#define ADT75_FRAME_SIZE	3

#define ADT75_TEMP_VALUE_REG    0x0
#define ADT75_CONF_REG          0x1
#define ADT75_HYST_REG          0x2
#define ADT75_OS_REG            0x3
#define ADT75_ONE_SHOT_REG      0x4

#define ADT75_DATA_REG_MASK     	NO_OS_GENMASK(2, 0)
#define ADT75_CONFIG_REG_MASK   	NO_OS_GENMASK(7, 0)

#define ADT75_TEMP_MASK			NO_OS_GENMASK(15, 4)
#define ADT75_SIGN_BIT			11

/** Configuration register fields */
#define ADT75_SHUTDOWN_MASK		NO_OS_BIT(0)
#define ADT75_COMP_INT_MASK		NO_OS_BIT(1)
#define ADT75_ALERT_POL_MASK		NO_OS_BIT(2)
#define ADT75_FAULT_QUEUE_MASK		NO_OS_GENMASK(4, 3)
#define ADT75_ONESHOT_MASK		NO_OS_BIT(5)

#define ADT75_HYST_MASK			NO_OS_GENMASK(15, 4)
#define ADT75_OVER_TEMP_MASK		NO_OS_GENMASK(15, 4)

/** 1 LSB = 0.0625 C */
#define ADT75_TEMP_DIV			16

/***************************************************************************//**
 * @brief The `adt75_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the ADT75 device,
 * specifically focusing on the I2C communication parameters. It contains
 * a single member, `comm_param`, which is a structure that holds the
 * necessary details for initializing I2C communication, ensuring that
 * the ADT75 device can be properly interfaced with over an I2C bus.
 *
 * @param comm_param This member is a structure of type no_os_i2c_init_param,
 * used for I2C communication initialization parameters.
 ******************************************************************************/
struct adt75_init_param {
	struct no_os_i2c_init_param comm_param;
};

/***************************************************************************//**
 * @brief The `adt75_desc` structure serves as a device descriptor for the ADT75
 * temperature sensor, encapsulating the necessary I2C communication
 * descriptor to facilitate interactions with the sensor hardware. This
 * structure is integral to the initialization, configuration, and
 * operation of the ADT75 device within the software, providing a means
 * to manage and access the sensor's functionalities through I2C
 * protocol.
 *
 * @param comm_desc A pointer to a no_os_i2c_desc structure, used for I2C
 * communication.
 ******************************************************************************/
struct adt75_desc {
	struct no_os_i2c_desc *comm_desc;
};

/***************************************************************************//**
 * @brief This function is used to obtain a single temperature reading from the
 * ADT75 temperature sensor and convert the result into milli-degrees
 * Celsius. It should be called when a temperature measurement is needed.
 * The function requires a valid device descriptor and a pointer to an
 * integer where the temperature value will be stored. It waits for the
 * conversion to complete before returning the result. Ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized 'adt75_desc' structure representing
 * the device. Must not be null. The function will fail if the
 * descriptor is invalid.
 * @param val A pointer to an int32_t where the temperature value will be stored
 * in milli-degrees Celsius. Must not be null. The function writes
 * the converted temperature value to this location.
 * @return Returns 0 on success, or a negative error code if the temperature
 * reading fails.
 ******************************************************************************/
int adt75_get_single_temp(struct adt75_desc *, int32_t *);

/***************************************************************************//**
 * @brief This function reads the value from a specified register of the ADT75
 * temperature sensor device. It is typically used to retrieve
 * configuration or status information from the device. The function
 * requires a valid device descriptor and a register address to read
 * from. The result is stored in the provided output parameter. It is
 * important to ensure that the device descriptor is properly initialized
 * before calling this function. The function handles both single-byte
 * and two-byte register reads, depending on the register address
 * provided.
 *
 * @param desc A pointer to an initialized `adt75_desc` structure representing
 * the device. Must not be null.
 * @param addr The address of the register to read from. Valid addresses are
 * defined by the device's register map.
 * @param val A pointer to a `uint16_t` where the read value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int adt75_reg_read(struct adt75_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function writes a 16-bit value to a specified register of the
 * ADT75 temperature sensor device. It is typically used to configure the
 * device or set specific parameters by writing to its registers. The
 * function requires a valid device descriptor, a register address, and
 * the value to be written. It is important to ensure that the device has
 * been properly initialized before calling this function. The function
 * handles different register sizes by adjusting the number of bytes
 * written based on the register address. It returns an error code if the
 * write operation fails.
 *
 * @param desc A pointer to an `adt75_desc` structure representing the device
 * descriptor. Must not be null. The descriptor should be properly
 * initialized before use.
 * @param addr A 32-bit unsigned integer representing the register address to
 * which the value will be written. Valid register addresses are
 * defined by the device, such as configuration or temperature
 * registers.
 * @param val A 16-bit unsigned integer representing the value to be written to
 * the specified register. The value should be formatted according to
 * the register's expected data format.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the write operation fails.
 ******************************************************************************/
int adt75_reg_write(struct adt75_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function sets up the ADT75 device by allocating and initializing
 * a device descriptor. It must be called before any other operations on
 * the ADT75 device. The function requires valid initialization
 * parameters and will return an error if memory allocation fails or if
 * the I2C communication setup is unsuccessful. The caller is responsible
 * for managing the memory of the descriptor, which should be freed using
 * the appropriate function when no longer needed.
 *
 * @param desc A pointer to a pointer of type `struct adt75_desc`. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct adt75_init_param` containing the
 * initialization parameters for the device. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -ENOMEM for memory allocation
 * failure).
 ******************************************************************************/
int adt75_init(struct adt75_desc **, struct adt75_init_param *);

/***************************************************************************//**
 * @brief This function is used to release resources associated with an ADT75
 * device descriptor. It should be called when the device is no longer
 * needed to ensure proper cleanup of resources. The function expects a
 * valid descriptor and will return an error if the descriptor is null.
 * It also handles the removal of the associated I2C communication
 * descriptor.
 *
 * @param desc A pointer to the ADT75 device descriptor to be freed. Must not be
 * null. If null, the function returns an error code.
 * @return Returns 0 on success, or a negative error code if the descriptor is
 * null or if there is an error in removing the I2C communication
 * descriptor.
 ******************************************************************************/
int adt75_remove(struct adt75_desc *);

#endif // _ADT75_H
