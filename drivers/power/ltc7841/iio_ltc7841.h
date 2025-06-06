/***************************************************************************//**
 *   @file   iio_ltc7841.h
 *   @brief  Header file for the LTC7841 IIO Driver
 *   @author Marvin Cabuenas (marvinneil.cabuenas@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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

#ifndef IIO_LTC7841_H
#define IIO_LTC7841_H

#include <stdbool.h>
#include "iio.h"
#include "ltc7841.h"

#define CHANNEL_COUNT                            2
#define SCALED_VOLTAGE                           50 /* in units of uV/bit */
#define DECIMAL_POINT_SCALE_VOLTAGE              10000

/***************************************************************************//**
 * @brief The `ltc7841_iio_desc` structure is a compound data type that
 * encapsulates the necessary descriptors for interfacing with an LTC7841
 * device through the Industrial I/O (IIO) framework. It contains
 * pointers to both the LTC7841 device descriptor and the IIO device
 * interface, facilitating the integration and management of the LTC7841
 * within an IIO context.
 *
 * @param ltc7841_desc A pointer to an ltc7841_desc structure, representing the
 * LTC7841 device descriptor.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct ltc7841_iio_desc {
	struct ltc7841_desc *ltc7841_desc;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `ltc7841_iio_desc_init_param` structure is used to initialize the
 * LTC7841 IIO device descriptor. It contains a pointer to the
 * `ltc7841_init_param` structure, which holds the necessary
 * initialization parameters for the LTC7841 device, and an 8-bit
 * unsigned integer representing the Rsense value in milliOhms, which is
 * crucial for configuring the device's sensing capabilities.
 *
 * @param ltc7841_init_param A pointer to a structure containing initialization
 * parameters for the LTC7841.
 * @param LTC7841_rsense_value An 8-bit unsigned integer representing the Rsense
 * value in milliOhms.
 ******************************************************************************/
struct ltc7841_iio_desc_init_param {
	struct ltc7841_init_param *ltc7841_init_param;
	/* Rsense value used in units of milliOhm*/
	uint8_t LTC7841_rsense_value;
};

/***************************************************************************//**
 * @brief This function initializes an LTC7841 IIO descriptor, which is
 * necessary for interfacing with the LTC7841 device through the IIO
 * framework. It must be called before any operations are performed on
 * the LTC7841 device. The function allocates memory for the descriptor
 * and initializes it using the provided parameters. If the
 * initialization parameters are invalid or memory allocation fails, the
 * function returns an error code. Ensure that the `ltc7841_iio_remove`
 * function is called to free resources when the descriptor is no longer
 * needed.
 *
 * @param iio_desc A pointer to a pointer where the initialized LTC7841 IIO
 * descriptor will be stored. Must not be null. The caller takes
 * ownership of the allocated descriptor and is responsible for
 * freeing it using `ltc7841_iio_remove`.
 * @param init_param A pointer to an `ltc7841_iio_desc_init_param` structure
 * containing initialization parameters. Must not be null and
 * must contain a valid `ltc7841_init_param`. If invalid, the
 * function returns -EINVAL.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL for invalid parameters or -ENOMEM if memory allocation
 * fails.
 ******************************************************************************/
int ltc7841_iio_init(struct ltc7841_iio_desc ** iio_desc,
		     struct ltc7841_iio_desc_init_param * init_param);

/***************************************************************************//**
 * @brief Use this function to release resources associated with an LTC7841 IIO
 * descriptor when it is no longer needed. It should be called to clean
 * up after the descriptor has been initialized and used, ensuring that
 * all allocated memory is properly freed. This function must be called
 * with a valid descriptor that was previously initialized; passing a
 * null pointer will result in an error.
 *
 * @param desc A pointer to an ltc7841_iio_desc structure that was previously
 * initialized. Must not be null. If null, the function returns an
 * error code (-ENODEV). The caller retains ownership of the
 * pointer.
 * @return Returns 0 on successful resource deallocation. Returns -ENODEV if the
 * provided descriptor is null.
 ******************************************************************************/
int ltc7841_iio_remove(struct ltc7841_iio_desc * desc);

#endif /* IIO_LTC7841_H */
