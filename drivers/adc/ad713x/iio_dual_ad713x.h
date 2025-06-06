/***************************************************************************//**
*   @file   iio_ad713x.h
*   @brief  Header file of iio_dual_ad713x
*   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
* Copyright 2019(c) Analog Devices, Inc.
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
#ifndef IIO_DUAL_AD713X
#define IIO_DUAL_AD713X

#ifdef IIO_SUPPORT

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include "ad713x.h"
#include "iio_types.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `iio_ad713x_init_par` structure is used to configure the
 * initialization parameters for the IIO interface of the AD713x device.
 * It includes the number of channels, a pointer to the device instance,
 * SPI engine descriptors, and a function pointer for cache invalidation,
 * facilitating the setup and management of the device's data acquisition
 * capabilities.
 *
 * @param num_channels Specifies the number of channels.
 * @param dev Pointer to the device instance of type ad713x_dev.
 * @param spi_eng_desc Pointer to the SPI engine descriptor of type
 * no_os_spi_desc.
 * @param spi_engine_offload_message Pointer to the SPI engine message
 * descriptor of type
 * spi_engine_offload_message.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a given address range.
 ******************************************************************************/
struct iio_ad713x_init_par {
	/** Number of channels */
	uint8_t	num_channels;
	/* Device instance */
	struct ad713x_dev *dev;
	/** Spi engine descriptor */
	struct no_os_spi_desc *spi_eng_desc;
	/** Spi engine message descriptor */
	struct spi_engine_offload_message *spi_engine_offload_message;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/***************************************************************************//**
 * @brief The `iio_ad713x` structure is designed to encapsulate the
 * configuration and state of an AD713x device within an IIO (Industrial
 * I/O) context. It includes a mask for active channels, descriptors for
 * the IIO device and SPI engine, and a function pointer for cache
 * management, facilitating efficient data handling and communication
 * with the AD713x device.
 *
 * @param mask A 32-bit integer representing the mask of active channels.
 * @param iio_dev_desc An instance of the iio_device structure, serving as the
 * IIO device descriptor.
 * @param spi_eng_desc A pointer to a no_os_spi_desc structure, which describes
 * the SPI engine.
 * @param spi_engine_offload_message A pointer to a spi_engine_offload_message
 * structure, which describes the SPI engine
 * message.
 * @param dcache_invalidate_range A function pointer for invalidating the data
 * cache over a specified address range.
 ******************************************************************************/
struct iio_ad713x {
	/* Mask of active ch */
	uint32_t mask;
	/** iio device descriptor */
	struct iio_device iio_dev_desc;
	/** Spi engine descriptor */
	struct no_os_spi_desc *spi_eng_desc;
	/** Spi engine message descriptor */
	struct spi_engine_offload_message *spi_engine_offload_message;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Init function. */
/***************************************************************************//**
 * @brief This function initializes an IIO AD713x device structure based on the
 * provided initialization parameters. It allocates memory for the device
 * descriptor and sets up the device configuration using the parameters
 * specified in the `iio_ad713x_init_par` structure. This function must
 * be called before any operations are performed on the AD713x device. If
 * the initialization is successful, the function returns 0 and provides
 * a pointer to the initialized device structure. In case of a memory
 * allocation failure, it returns -1, indicating an error.
 *
 * @param desc A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller is responsible for
 * managing the memory of the descriptor after initialization.
 * @param param A pointer to an `iio_ad713x_init_par` structure containing the
 * initialization parameters. Must not be null. The structure
 * should be properly populated with valid configuration data
 * before calling this function.
 * @return Returns 0 on successful initialization, or -1 if memory allocation
 * fails.
 ******************************************************************************/
int32_t iio_dual_ad713x_init(struct iio_ad713x **desc,
			     struct iio_ad713x_init_par *param);
/* Get desciptor. */
/***************************************************************************//**
 * @brief This function is used to obtain the IIO device descriptor associated
 * with a given AD713x device structure. It is typically called after the
 * device has been initialized using the appropriate initialization
 * function. The function does not perform any validation on the input
 * parameters, so it is the caller's responsibility to ensure that the
 * provided device structure is valid and properly initialized. The
 * function directly assigns the internal IIO device descriptor to the
 * provided pointer, allowing the caller to access the descriptor for
 * further operations.
 *
 * @param desc A pointer to a valid and initialized `iio_ad713x` structure. This
 * parameter must not be null, as it is used to access the internal
 * IIO device descriptor.
 * @param dev_descriptor A pointer to a pointer of type `iio_device`. This
 * parameter must not be null, as the function will assign
 * the address of the internal IIO device descriptor to
 * it.
 * @return None
 ******************************************************************************/
void iio_dual_ad713x_get_dev_descriptor(struct iio_ad713x *desc,
					struct iio_device **dev_descriptor);
/* Free the resources allocated by iio_ad713x_init(). */
/***************************************************************************//**
 * @brief This function is used to release the resources associated with an
 * iio_ad713x instance that were previously allocated by
 * iio_dual_ad713x_init(). It should be called when the iio_ad713x
 * instance is no longer needed to ensure proper cleanup and avoid memory
 * leaks. The function checks if the provided descriptor is valid before
 * attempting to free it. If the descriptor is null, the function returns
 * an error code.
 *
 * @param desc A pointer to the iio_ad713x instance to be freed. Must not be
 * null. If null, the function returns an error code and no action
 * is taken.
 * @return Returns 0 on successful resource deallocation, or -1 if the provided
 * descriptor is null.
 ******************************************************************************/
int32_t iio_dual_ad713x_remove(struct iio_ad713x *desc);

#endif /* IIO_SUPPORT */

#endif /* IIO_AD713X */
