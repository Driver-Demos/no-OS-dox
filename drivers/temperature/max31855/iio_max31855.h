/***************************************************************************//**
 *   @file   iio_max31855.h
 *   @brief  Header file of IIO MAX31855 driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
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

#ifndef IIO_MAX31855_H
#define IIO_MAX31855_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `max31855_iio_dev` structure is designed to interface with the
 * MAX31855 thermocouple-to-digital converter through the Industrial I/O
 * (IIO) subsystem. It encapsulates a device descriptor for the MAX31855,
 * an IIO device interface, and manages the active channels and their
 * count, facilitating the integration of the MAX31855 into systems that
 * utilize the IIO framework for sensor data acquisition.
 *
 * @param max31855_desc A pointer to a max31855_dev structure, representing the
 * MAX31855 device descriptor.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 * @param active_channels A 32-bit unsigned integer indicating the active
 * channels in the device.
 * @param no_of_active_channels An 8-bit unsigned integer representing the
 * number of active channels.
 ******************************************************************************/
struct max31855_iio_dev {
	struct max31855_dev *max31855_desc;
	struct iio_device *iio_dev;
	uint32_t active_channels;
	uint8_t no_of_active_channels;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `max31855_iio_dev_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up a MAX31855 IIO
 * (Industrial Input/Output) device. It contains a single member,
 * `max31855_dev_init`, which is a pointer to another structure that
 * holds the specific initialization settings for the MAX31855 device.
 * This structure is essential for configuring the device before it is
 * used in an application.
 *
 * @param max31855_dev_init A pointer to a max31855_init_param structure, which
 * holds initialization parameters for the MAX31855
 * device.
 ******************************************************************************/
struct max31855_iio_dev_init_param {
	struct max31855_init_param *max31855_dev_init;
};

/***************************************************************************//**
 * @brief This function initializes a MAX31855 IIO device structure, preparing
 * it for use with the specified initialization parameters. It must be
 * called before any operations are performed on the device. The function
 * allocates memory for the device structure and initializes it using the
 * provided parameters. If the initialization is successful, the function
 * returns 0 and provides a pointer to the initialized device structure.
 * If the initialization fails, it returns a negative error code and does
 * not modify the output pointer.
 *
 * @param iio_dev A pointer to a pointer where the initialized MAX31855 IIO
 * device structure will be stored. Must not be null. The caller
 * takes ownership of the allocated structure and is responsible
 * for freeing it.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the MAX31855 device. Must not be null. If
 * null, the function returns -EINVAL.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -ENOMEM if
 * memory allocation fails).
 ******************************************************************************/
int max31855_iio_init(struct max31855_iio_dev **,
		      struct max31855_iio_dev_init_param *);
/***************************************************************************//**
 * @brief Use this function to properly remove and deallocate resources
 * associated with a MAX31855 IIO device instance. It should be called
 * when the device is no longer needed to ensure that all associated
 * resources are freed. This function must be called only after the
 * device has been successfully initialized and used. Failure to call
 * this function may result in resource leaks.
 *
 * @param desc A pointer to the MAX31855 IIO device instance to be removed. Must
 * not be null. The function will handle deallocation, so the caller
 * should not use the pointer after this function is called.
 * @return Returns 0 on successful removal and deallocation. If an error occurs
 * during the removal process, a negative error code is returned.
 ******************************************************************************/
int max31855_iio_remove(struct max31855_iio_dev *);

#endif
