// /***************************************************************************//**
//  *   @file   iio_ltc7871.h
//  *   @brief  Header file for the LTC7871 IIO Driver
//  *   @author Aldrin Abacan (aldrin.abacan@analog.com)
//  *******************************************************************************
//  * Copyright 2024(c) Analog Devices, Inc.
//  *
//  * Redistribution and use in source and binary forms, with or without
//  * modification, are permitted provided that the following conditions are met:
//  *
//  * 1. Redistributions of source code must retain the above copyright notice,
//  *    this list of conditions and the following disclaimer.
//  *
//  * 2. Redistributions in binary form must reproduce the above copyright notice,
//  *    this list of conditions and the following disclaimer in the documentation
//  *    and/or other materials provided with the distribution.
//  *
//  * 3. Neither the name of Analog Devices, Inc. nor the names of its
//  *    contributors may be used to endorse or promote products derived from this
//  *    software without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
//  * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
//  * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
//  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
//  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
//  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
//  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  ******************************************************************************/

#ifndef __IIO_LTC7871_H__
#define __IIO_LTC7871_H__

#include <stdbool.h>
// #include "iio/iio.h"
#include "iio.h"
#include "ltc7871.h"

/***************************************************************************//**
 * @brief The `ltc7871_iio_dev` structure is a compound data type that
 * encapsulates the necessary components for interfacing with an LTC7871
 * device through the Industrial I/O (IIO) subsystem. It contains
 * pointers to both the specific LTC7871 device structure and the generic
 * IIO device structure, allowing for the integration and management of
 * the LTC7871 within an IIO framework. This structure is essential for
 * initializing and managing the device's operations in a standardized
 * manner.
 *
 * @param ltc7871_dev A pointer to an ltc7871_dev structure, representing the
 * LTC7871 device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct ltc7871_iio_dev {
	struct ltc7871_dev *ltc7871_dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `ltc7871_iio_dev_init_param` structure is designed to encapsulate
 * the initialization parameters required for setting up an LTC7871 IIO
 * device. It contains a single member, which is a pointer to another
 * structure, `ltc7871_init_param`, that holds the specific
 * initialization settings for the LTC7871 device. This structure is used
 * during the initialization process to configure the device with the
 * necessary parameters.
 *
 * @param ltc7871_init_param A pointer to a structure containing initialization
 * parameters for the LTC7871 device.
 ******************************************************************************/
struct ltc7871_iio_dev_init_param {
	struct ltc7871_init_param *ltc7871_init_param;
};

/***************************************************************************//**
 * @brief This function sets up the LTC7871 IIO device descriptor using the
 * provided initialization parameters. It must be called before any
 * operations are performed on the LTC7871 IIO device. The function
 * allocates memory for the device descriptor and initializes it with the
 * given parameters. If the initialization is successful, the function
 * returns 0 and the iio_dev pointer is updated to point to the newly
 * created descriptor. In case of failure, an error code is returned and
 * no memory is allocated to the iio_dev pointer.
 *
 * @param iio_dev A pointer to a pointer of ltc7871_iio_dev structure. This must
 * not be null. On successful initialization, it will point to
 * the newly allocated and initialized device descriptor.
 * @param init_param A pointer to a ltc7871_iio_dev_init_param structure
 * containing the initialization parameters. This must not be
 * null and must contain a valid ltc7871_init_param pointer.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error, such as -EINVAL for invalid parameters
 * or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int ltc7871_iio_init(struct ltc7871_iio_dev **iio_dev,
		     struct ltc7871_iio_dev_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to release resources associated with an LTC7871 IIO
 * device descriptor when it is no longer needed. This function should be
 * called to clean up after a successful initialization with
 * `ltc7871_iio_init`. It ensures that all allocated memory and resources
 * are properly freed, preventing memory leaks. The function handles null
 * pointers gracefully by returning an error code, making it safe to call
 * even if the descriptor might be null.
 *
 * @param desc A pointer to the `ltc7871_iio_dev` structure representing the IIO
 * device descriptor. Must not be null; if it is null, the function
 * returns an error code (-ENODEV). The caller retains ownership and
 * is responsible for ensuring the descriptor was previously
 * initialized.
 * @return Returns 0 on successful resource deallocation. If the input
 * descriptor is null, returns -ENODEV.
 ******************************************************************************/
int ltc7871_iio_remove(struct ltc7871_iio_dev *desc);

#endif /* __IIO_LTC7871_H__ */
