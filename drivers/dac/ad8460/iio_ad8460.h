/***************************************************************************//**
 *   @file   iio_ad8460.h
 *   @brief  Implementation of IIO AD8460 Driver.
 *   @author John Erasmus Mari Geronimo (johnerasmusmari.geronimo@analog.com)
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

#ifndef __IIO_AD8460_H__
#define __IIO_AD8460_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad8460_iio_device` structure is designed to encapsulate the
 * necessary components for interfacing with an AD8460 device through the
 * Industrial I/O (IIO) framework. It includes pointers to both the
 * specific AD8460 device and the generic IIO device interface, as well
 * as fields to track the active channels and their count, facilitating
 * the management and operation of the device within an IIO context.
 *
 * @param dev A pointer to an ad8460_device structure, representing the
 * underlying AD8460 device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 * @param active_channels A 32-bit unsigned integer indicating the currently
 * active channels.
 * @param no_active_channels An 8-bit unsigned integer representing the number
 * of active channels.
 ******************************************************************************/
struct ad8460_iio_device {
	struct ad8460_device *dev;
	struct iio_device *iio_dev;
	uint32_t active_channels;
	uint8_t no_active_channels;
};

/***************************************************************************//**
 * @brief The `ad8460_iio_init_param` structure is designed to encapsulate
 * initialization parameters specifically for the AD8460 IIO (Industrial
 * Input/Output) device. It contains a single member, `init_param`, which
 * is a pointer to another structure, `ad8460_init_param`, that holds the
 * necessary configuration settings required to initialize the AD8460
 * device. This structure is used during the setup phase of the device to
 * ensure that all necessary parameters are correctly configured before
 * the device becomes operational.
 *
 * @param init_param A pointer to an ad8460_init_param structure, which holds
 * initialization parameters for the AD8460 device.
 ******************************************************************************/
struct ad8460_iio_init_param {
	struct ad8460_init_param *init_param;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes an AD8460 IIO device using the provided
 * initialization parameters. It must be called before any operations are
 * performed on the device. The function allocates memory for the device
 * structure and sets up the device with default settings. It is
 * essential to ensure that the `iio_init_param` is properly configured
 * and not null before calling this function. If initialization fails,
 * the function returns an error code and does not modify the output
 * parameter.
 *
 * @param iio_device A pointer to a pointer where the initialized AD8460 IIO
 * device structure will be stored. The caller must ensure
 * this is a valid, non-null pointer. On success, the function
 * allocates memory for the device and assigns it to this
 * pointer.
 * @param iio_init_param A pointer to an `ad8460_iio_init_param` structure
 * containing initialization parameters. This must not be
 * null and must contain a valid `init_param` field. If
 * this parameter is invalid, the function returns an
 * error code.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid parameters,
 * -ENOMEM for memory allocation failure).
 ******************************************************************************/
int ad8460_iio_init(struct ad8460_iio_device **iio_device,
		    struct ad8460_iio_init_param *iio_init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with an AD8460 IIO device. It should be called when the
 * device is no longer needed to ensure that all resources are freed and
 * any associated hardware is properly shut down. The function first
 * attempts to remove the underlying device and, if successful, frees the
 * memory allocated for the IIO device structure. It is important to
 * ensure that the `iio_device` parameter is valid and properly
 * initialized before calling this function.
 *
 * @param iio_device A pointer to the `ad8460_iio_device` structure representing
 * the IIO device to be removed. This pointer must not be null
 * and should point to a valid, initialized device structure.
 * The function will handle invalid pointers by returning an
 * error code.
 * @return Returns 0 on successful removal and deallocation of the device. If an
 * error occurs during the removal of the underlying device, a non-zero
 * error code is returned.
 ******************************************************************************/
int ad8460_iio_remove(struct ad8460_iio_device *iio_device);

#endif	/* __IIO_AD8460_H__ */
