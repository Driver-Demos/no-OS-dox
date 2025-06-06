/***************************************************************************//**
 *   @file   iio_max11205.h
 *   @brief  Header file of iio max11205 driver.
 *   @author RBolboac (ramona.bolboaca@analog.com)
 *******************************************************************************
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
 ******************************************************************************/
#ifndef __IIO_MAX11205_H__
#define __IIO_MAX11205_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `max11205_dev_id` enumeration defines identifiers for different
 * variants of the MAX11205 device, specifically MAX11205A and MAX11205B.
 * This enumeration is used to distinguish between these device variants
 * in the code, allowing for specific handling or configuration based on
 * the device type.
 *
 * @param MAX11205A Represents the device ID for the MAX11205A variant.
 * @param MAX11205B Represents the device ID for the MAX11205B variant.
 ******************************************************************************/
enum max11205_dev_id {
	MAX11205A,
	MAX11205B,
};

/***************************************************************************//**
 * @brief The `max11205_iio_dev` structure is a compound data type that
 * encapsulates the necessary descriptors and configuration for
 * interfacing with a MAX11205 device through an IIO (Industrial I/O)
 * framework. It includes a pointer to the MAX11205 device descriptor, a
 * pointer to the IIO device descriptor, and an integer to specify the
 * sampling frequency of the MAX11205 device. This structure is essential
 * for managing the device's operations and integrating it into the IIO
 * subsystem.
 *
 * @param max11205_dev Pointer to the MAX11205 device descriptor.
 * @param iio_dev Pointer to the IIO device descriptor.
 * @param sampling_frequency Integer representing the sampling frequency of the
 * MAX11205 device.
 ******************************************************************************/
struct max11205_iio_dev {
	/** MAX11205 device descriptor */
	struct max11205_dev 	*max11205_dev;
	/** IIO device descriptor */
	struct iio_device 	*iio_dev;
	/** MAX11205 device sampling frequency */
	int32_t 		sampling_frequency;
};

/***************************************************************************//**
 * @brief The `max11205_iio_dev_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up a MAX11205 device in
 * an IIO (Industrial Input/Output) context. It includes a pointer to a
 * `max11205_init_param` structure, which holds the device-specific
 * initialization data, and an enumeration `dev_id` that specifies the
 * particular device variant being used. This structure is essential for
 * initializing the device with the correct configuration and ensuring
 * compatibility with the IIO framework.
 *
 * @param max11205_dev_init Pointer to the initialization parameters for the
 * MAX11205 device.
 * @param dev_id Enumeration value representing the specific MAX11205 device ID.
 ******************************************************************************/
struct max11205_iio_dev_init_param {
	/** MAX11205 device initialization data */
	struct max11205_init_param *max11205_dev_init;
	/** MAX11205 device id */
	enum max11205_dev_id dev_id;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function initializes a MAX11205 IIO device using the provided
 * initialization parameters. It allocates memory for the device
 * descriptor and sets up the device based on the specified device ID.
 * The function must be called before any operations are performed on the
 * device, and it is expected that the caller provides valid
 * initialization parameters. If the initialization is successful, the
 * function returns 0 and provides a pointer to the initialized device
 * descriptor. In case of failure, a negative error code is returned, and
 * the device descriptor is not valid.
 *
 * @param iio_dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership
 * of the allocated memory upon successful initialization.
 * @param init_param A pointer to a structure containing the initialization
 * parameters for the MAX11205 device. Must not be null and
 * must contain valid data for the initialization to succeed.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error.
 ******************************************************************************/
int max11205_iio_init(struct max11205_iio_dev **iio_dev,
		      struct max11205_iio_dev_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with a MAX11205 IIO device. It should be called when the
 * device is no longer needed to ensure that all resources are freed and
 * any necessary cleanup is performed. This function must be called only
 * after the device has been successfully initialized and used. Failure
 * to call this function may result in resource leaks.
 *
 * @param desc A pointer to a max11205_iio_dev structure representing the device
 * to be removed. Must not be null. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns 0 on successful removal and deallocation, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int max11205_iio_remove(struct max11205_iio_dev *desc);

#endif /* __IIO_MAX11205_H__ */
