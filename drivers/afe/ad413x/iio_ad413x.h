/***************************************************************************//**
*   @file   iio_ad413x.h
*   @brief  Header file of iio_ad413x
*   @author Andrei Porumb (andrei.porumb@analog.com)
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

#ifndef IIO_AD413X_H
#define IIO_AD413X_H

#include "iio.h"
#include "ad413x.h"

/***************************************************************************//**
 * @brief The `ad413x_iio_dev` structure is a compound data type that
 * encapsulates both an AD413x device instance and its corresponding IIO
 * device interface. It serves as a bridge between the AD413x hardware
 * and the IIO subsystem, allowing for the integration and management of
 * the device within an IIO framework. This structure is essential for
 * initializing and managing the AD413x device in applications that
 * utilize the IIO interface for data acquisition and processing.
 *
 * @param ad413x_dev A pointer to an ad413x_dev structure, representing the
 * AD413x device instance.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct ad413x_iio_dev {
	struct ad413x_dev *ad413x_dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `ad413x_iio_init_param` structure is designed to encapsulate
 * initialization parameters specifically for the AD413x device within an
 * IIO (Industrial Input/Output) context. It contains a single member,
 * `ad413x_ip`, which is itself a structure that holds the necessary
 * parameters to initialize the AD413x device, facilitating the setup and
 * configuration process for interfacing with the device in an IIO
 * framework.
 *
 * @param ad413x_ip This member is an instance of the ad413x_init_param
 * structure, which holds initialization parameters for the
 * AD413x device.
 ******************************************************************************/
struct ad413x_iio_init_param {
	struct ad413x_init_param ad413x_ip;
};

/***************************************************************************//**
 * @brief This function sets up and initializes an AD413x IIO device structure,
 * preparing it for use. It must be called before any operations are
 * performed on the device. The function allocates memory for the device
 * structure and initializes the underlying AD413x device using the
 * provided initialization parameters. If the initialization fails, the
 * function ensures that allocated resources are properly freed. It is
 * essential to check the return value to ensure successful
 * initialization before proceeding with further operations.
 *
 * @param iio_dev A pointer to a pointer of type struct ad413x_iio_dev. This
 * will be allocated and initialized by the function. The caller
 * must ensure that this pointer is valid and will receive
 * ownership of the allocated structure upon successful
 * initialization.
 * @param init_param A struct ad413x_iio_init_param containing the
 * initialization parameters for the AD413x device. This
 * structure must be properly populated with valid parameters
 * before calling the function.
 * @return Returns 0 on successful initialization. If an error occurs, a
 * negative error code is returned, and the iio_dev pointer will not be
 * valid.
 ******************************************************************************/
int32_t ad413x_iio_init(struct ad413x_iio_dev **iio_dev,
			struct ad413x_iio_init_param init_param);
/***************************************************************************//**
 * @brief This function is used to properly release and clean up resources
 * associated with an AD413x IIO device. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * freed and any necessary cleanup is performed. This function must be
 * called after the device has been initialized and used, and it is
 * important to ensure that the `desc` parameter is valid and not null
 * before calling this function. Failure to do so may result in undefined
 * behavior.
 *
 * @param desc A pointer to an `ad413x_iio_dev` structure representing the
 * device to be removed. This pointer must not be null and should
 * point to a valid, initialized device structure. The function will
 * handle freeing the resources associated with this device.
 * @return Returns 0 on successful removal and cleanup of the device. If an
 * error occurs during the removal process, a negative error code is
 * returned.
 ******************************************************************************/
int32_t ad413x_iio_remove(struct ad413x_iio_dev *desc);

#endif /** IIO_AD413X_H */
