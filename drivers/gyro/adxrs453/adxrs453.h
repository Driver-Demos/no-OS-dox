/***************************************************************************//**
 *   @file   ADXRS453.h
 *   @brief  Header file of ADXRS453 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
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
#ifndef __ADXRS453_H__
#define __ADXRS453_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/************************** ADXRS453 Definitions ******************************/
/******************************************************************************/

#define ADXRS453_READ           (1 << 7)
#define ADXRS453_WRITE          (1 << 6)
#define ADXRS453_SENSOR_DATA    (1 << 5)

#define ADXRS453_REG_RATE       0x00
#define ADXRS453_REG_TEM        0x02
#define ADXRS453_REG_LOCST      0x04
#define ADXRS453_REG_HICST      0x06
#define ADXRS453_REG_QUAD       0x08
#define ADXRS453_REG_FAULT      0x0A
#define ADXRS453_REG_PID        0x0C
#define ADXRS453_REG_SN_HIGH    0x0E
#define ADXRS453_REG_SN_LOW     0x10

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adxrs453_dev` structure is a data structure used to represent a
 * device instance of the ADXRS453 gyroscope sensor. It contains a single
 * member, `spi_desc`, which is a pointer to a SPI descriptor structure.
 * This member is used to facilitate communication with the ADXRS453
 * sensor over the SPI interface, allowing for operations such as reading
 * sensor data and configuring the device.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct adxrs453_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/***************************************************************************//**
 * @brief The `adxrs453_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the ADXRS453 device,
 * specifically focusing on the SPI interface configuration. This
 * structure is essential for initializing the device and ensuring proper
 * communication over the SPI bus.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct adxrs453_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes the ADXRS453 device by setting up the
 * necessary SPI communication and verifying the device's presence
 * through its ID register. It should be called before any other
 * operations on the ADXRS453 device to ensure that the device is
 * correctly set up and ready for use. The function allocates memory for
 * the device structure and initializes the SPI interface using the
 * provided initialization parameters. If the device is not present or
 * memory allocation fails, the function returns an error code.
 *
 * @param device A pointer to a pointer of type `struct adxrs453_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory
 * using `adxrs453_remove`.
 * @param init_param A structure of type `struct adxrs453_init_param` containing
 * the initialization parameters for the SPI interface. Must
 * be properly initialized before calling this function.
 * @return Returns 0 on successful initialization and device verification, or -1
 * if an error occurs (e.g., memory allocation failure or device not
 * present).
 ******************************************************************************/
int32_t adxrs453_init(struct adxrs453_dev **device,
		      struct adxrs453_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release all resources associated with an
 * ADXRS453 device that were previously allocated during initialization.
 * It should be called when the device is no longer needed to ensure
 * proper cleanup and to prevent resource leaks. The function must be
 * called with a valid device pointer that was successfully initialized
 * using `adxrs453_init`. After calling this function, the device pointer
 * should not be used unless it is re-initialized.
 *
 * @param dev A pointer to an `adxrs453_dev` structure representing the device
 * to be removed. This pointer must not be null and should point to a
 * valid device structure that was initialized using `adxrs453_init`.
 * Passing an invalid or null pointer results in undefined behavior.
 * @return Returns an integer status code from the underlying SPI removal
 * operation, where 0 typically indicates success and a negative value
 * indicates an error.
 ******************************************************************************/
int32_t adxrs453_remove(struct adxrs453_dev *dev);

/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in a specific
 * register of the ADXRS453 device. It is essential to ensure that the
 * device has been properly initialized using `adxrs453_init` before
 * calling this function. The function communicates with the device over
 * SPI to read the register value. This function is useful for obtaining
 * configuration or status information from the device. Ensure that the
 * register address provided is valid and corresponds to one of the
 * defined registers for the ADXRS453.
 *
 * @param dev A pointer to an `adxrs453_dev` structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null.
 * @param register_address The address of the register to read from. It should
 * be a valid register address as defined by the
 * ADXRS453 device specifications.
 * @return Returns a 16-bit unsigned integer representing the value read from
 * the specified register.
 ******************************************************************************/
uint16_t adxrs453_get_register_value(struct adxrs453_dev *dev,
				     uint8_t register_address);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the ADXRS453 device. It is typically called when there is a need to
 * configure or update the settings of the device. The function requires
 * a valid device structure that has been initialized and a valid
 * register address. The caller must ensure that the register address is
 * within the valid range defined by the device's register map. The
 * function does not return a value, and any errors during the SPI
 * communication are not reported back to the caller.
 *
 * @param dev A pointer to an initialized adxrs453_dev structure. This must not
 * be null and should be properly initialized using adxrs453_init
 * before calling this function. The caller retains ownership.
 * @param register_address A uint8_t representing the address of the register to
 * write to. It must be a valid register address as
 * defined by the ADXRS453 device's register map.
 * @param register_value A uint16_t value to be written to the specified
 * register. The value should be within the range that the
 * register can accept.
 * @return None
 ******************************************************************************/
void adxrs453_set_register_value(struct adxrs453_dev *dev,
				 uint8_t register_address,
				 uint16_t register_value);

/***************************************************************************//**
 * @brief This function retrieves the current sensor data from the ADXRS453
 * device. It should be called when the user needs to obtain the latest
 * sensor reading. The function requires a valid device structure that
 * has been initialized using `adxrs453_init`. It communicates with the
 * device over SPI to fetch the data, and the caller is responsible for
 * ensuring that the device is properly configured and operational before
 * calling this function.
 *
 * @param dev A pointer to an `adxrs453_dev` structure representing the device.
 * This must be a valid, non-null pointer to a device that has been
 * initialized with `adxrs453_init`. The function will not perform
 * any operation if this parameter is invalid.
 * @return Returns a 32-bit unsigned integer representing the sensor data read
 * from the device.
 ******************************************************************************/
uint32_t adxrs453_get_sensor_data(struct adxrs453_dev *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the current rotational rate from the
 * ADXRS453 gyroscope sensor, expressed in degrees per second. This
 * function should be called after the device has been successfully
 * initialized using `adxrs453_init`. It reads the rate register from the
 * device and converts the raw data into a floating-point value
 * representing the rate of rotation. The function handles both positive
 * and negative rate values, ensuring accurate representation of the
 * sensor's output.
 *
 * @param dev A pointer to an `adxrs453_dev` structure representing the device
 * instance. This must be a valid, initialized device structure, and
 * must not be null. The function will not perform any operation if
 * this parameter is invalid.
 * @return Returns a float representing the rotational rate in degrees per
 * second. The value can be positive or negative, depending on the
 * direction of rotation.
 ******************************************************************************/
float adxrs453_get_rate(struct adxrs453_dev *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from the
 * ADXRS453 device, expressed in degrees Celsius. This function should be
 * called when a temperature measurement is needed from the device.
 * Ensure that the device has been properly initialized using
 * `adxrs453_init` before calling this function. The function reads the
 * temperature register, processes the raw data, and returns the
 * temperature in a human-readable format. It is important to handle the
 * returned value appropriately, especially in systems where temperature
 * monitoring is critical.
 *
 * @param dev A pointer to an `adxrs453_dev` structure representing the device
 * instance. This must be a valid, initialized device structure
 * obtained from a successful call to `adxrs453_init`. The pointer
 * must not be null, and the function does not modify the device
 * structure.
 * @return Returns the temperature as a float representing degrees Celsius.
 ******************************************************************************/
float adxrs453_get_temperature(struct adxrs453_dev *dev);

#endif // __ADXRS453_H__
