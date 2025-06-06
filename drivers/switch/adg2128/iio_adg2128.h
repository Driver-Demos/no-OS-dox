/***************************************************************************//**
*   @file   iio_adg2128.h
*   @brief  Header file of iio_adg2128
*   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
* Copyright 2020(c) Analog Devices, Inc.
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

#ifndef IIO_ADG2128_H
#define IIO_ADG2128_H

#include "iio.h"

/***************************************************************************//**
 * @brief The `adg2128_iio_dev` structure is designed to encapsulate the
 * necessary components for interfacing with an ADG2128 device through
 * the IIO framework. It contains pointers to an IIO device and an I2C
 * descriptor, facilitating communication and control of the device over
 * an I2C bus.
 *
 * @param iio_dev A pointer to an IIO (Industrial I/O) device structure.
 * @param i2c_desc A pointer to a descriptor for an I2C (Inter-Integrated
 * Circuit) communication interface.
 ******************************************************************************/
struct adg2128_iio_dev {
	struct iio_device *iio_dev;
	struct no_os_i2c_desc *i2c_desc;
};

/***************************************************************************//**
 * @brief This function initializes an ADG2128 IIO device structure, setting up
 * the necessary IIO device and associating it with the provided I2C
 * descriptor. It must be called before any operations are performed on
 * the ADG2128 device through the IIO interface. The function allocates
 * memory for the device structure, and the caller is responsible for
 * ensuring that the I2C descriptor is valid and properly configured. If
 * the initialization is successful, the function returns 0 and provides
 * a pointer to the initialized device structure. In case of a memory
 * allocation failure, it returns a negative error code.
 *
 * @param iio_dev A pointer to a pointer where the initialized ADG2128 IIO
 * device structure will be stored. Must not be null. The caller
 * receives ownership of the allocated structure and is
 * responsible for its deallocation.
 * @param i2c_desc A pointer to a valid I2C descriptor that will be associated
 * with the ADG2128 IIO device. Must be properly initialized
 * before calling this function. The caller retains ownership of
 * this descriptor.
 * @return Returns 0 on successful initialization. Returns a negative error code
 * if memory allocation fails.
 ******************************************************************************/
int32_t adg2128_iio_init(struct adg2128_iio_dev **iio_dev,
			 struct no_os_i2c_desc *i2c_desc);
/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources
 * associated with an ADG2128 IIO device when it is no longer needed.
 * This function should be called to prevent memory leaks after the
 * device has been initialized and used. It is important to ensure that
 * the `desc` parameter is a valid pointer to an initialized
 * `adg2128_iio_dev` structure before calling this function. The function
 * does not perform any error checking on the input and assumes that the
 * provided descriptor is valid.
 *
 * @param desc A pointer to an `adg2128_iio_dev` structure that represents the
 * device to be removed. This pointer must not be null and should
 * point to a valid, initialized device structure. The function does
 * not handle null or invalid pointers, so the caller must ensure
 * the validity of this parameter.
 * @return Returns 0 to indicate successful execution. The function does not
 * return error codes or perform any other output operations.
 ******************************************************************************/
int32_t adg2128_iio_remove(struct adg2128_iio_dev *desc);

#endif /** IIO_ADG2128_H */
