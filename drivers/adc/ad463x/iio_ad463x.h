/***************************************************************************//**
*   @file   iio_ad463x.h
*   @brief  Header file of iio_ad463x
*   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
* Copyright 2021(c) Analog Devices, Inc.
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
#ifndef IIO_AD463x
#define IIO_AD463x

#ifdef IIO_SUPPORT

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include "iio_types.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `iio_ad463x` structure is designed to encapsulate the necessary
 * components for interfacing with an AD463x device in an IIO (Industrial
 * I/O) context. It includes a mask to specify active channels, an IIO
 * device descriptor for managing IIO operations, and a pointer to the
 * AD463x device descriptor, which provides the specific details and
 * operations for the AD463x device. This structure is essential for
 * initializing and managing the AD463x device within an IIO framework.
 *
 * @param mask A 32-bit unsigned integer representing the mask of active
 * channels.
 * @param iio_dev_desc An instance of the iio_device structure, serving as the
 * IIO device descriptor.
 * @param ad463x_desc A pointer to an ad463x_dev structure, representing the
 * device descriptor.
 ******************************************************************************/
struct iio_ad463x {
	/* Mask of active ch */
	uint32_t mask;
	/** iio device descriptor */
	struct iio_device iio_dev_desc;
	/** Device Descriptor */
	struct ad463x_dev *ad463x_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Init function. */
/***************************************************************************//**
 * @brief This function sets up an IIO AD463x device descriptor, associating it
 * with a given AD463x device. It allocates memory for the descriptor and
 * configures it based on the device's characteristics. This function
 * must be called before any operations are performed on the IIO AD463x
 * device, and the caller is responsible for ensuring that the provided
 * AD463x device is valid and properly initialized. The function returns
 * an error code if memory allocation fails, and the caller should handle
 * this scenario appropriately.
 *
 * @param desc A pointer to a pointer where the initialized IIO AD463x
 * descriptor will be stored. Must not be null. The caller takes
 * ownership of the allocated memory and is responsible for freeing
 * it using iio_ad463x_remove().
 * @param dev A pointer to an initialized AD463x device structure. Must not be
 * null. The function uses this to configure the IIO AD463x
 * descriptor.
 * @return Returns 0 on success, or -1 if memory allocation fails.
 ******************************************************************************/
int32_t iio_ad463x_init(struct iio_ad463x **desc,
			struct ad463x_dev *dev);

/* Free the resources allocated by iio_ad463x_init(). */
/***************************************************************************//**
 * @brief This function is used to release the resources associated with an
 * iio_ad463x instance that were previously allocated by
 * iio_ad463x_init(). It should be called when the iio_ad463x instance is
 * no longer needed to prevent memory leaks. The function checks if the
 * provided descriptor is valid before attempting to free it. If the
 * descriptor is null, the function returns an error code.
 *
 * @param desc A pointer to the iio_ad463x structure to be freed. Must not be
 * null. If null, the function returns an error code and no action
 * is taken.
 * @return Returns 0 on successful resource deallocation, or -1 if the provided
 * descriptor is null.
 ******************************************************************************/
int32_t iio_ad463x_remove(struct iio_ad463x *desc);

#endif /* IIO_SUPPORT */

#endif /* IIO_AD463x */
