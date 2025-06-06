/***************************************************************************//**
*   @file   iio_ad713x.h
*   @brief  Header file of iio_axi_adc
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
#ifndef IIO_AD713X
#define IIO_AD713X

#ifdef IIO_SUPPORT

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_types.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

struct ad713x_iio;

/***************************************************************************//**
 * @brief The `ad713x_iio_init_param` structure is used to initialize the AD713x
 * IIO interface, containing pointers to device handlers and
 * configuration parameters such as the reference voltage and a function
 * for cache management. It facilitates the setup of the AD713x device
 * within an IIO framework, ensuring proper communication and data
 * handling through SPI and cache management.
 *
 * @param drv_dev Pointer to the AD713x driver handler.
 * @param iio_dev Pointer to a generic IIO device handler.
 * @param vref_int Stores the integer part of the reference voltage (VREF).
 * @param vref_micro Stores the decimal part of the reference voltage (VREF).
 * @param spi_eng_desc Pointer to the SPI Engine driver handler.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a specified address range.
 ******************************************************************************/
struct ad713x_iio_init_param {
	/** AD713x driver handler */
	struct ad713x_dev *drv_dev;
	/** Generic IIO device handler */
	struct iio_device *iio_dev;
	/** Integer part of the VREF */
	uint32_t vref_int;
	/** Decimal part of the VREF */
	uint32_t vref_micro;
	/** SPI Engine driver handler */
	struct no_os_spi_desc *spi_eng_desc;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
};

/***************************************************************************//**
 * @brief The `ad713x_iio_desc` is an external global variable of type `struct
 * iio_device`. It is intended to represent an IIO (Industrial I/O)
 * device descriptor for the AD713x series of devices, which are likely
 * analog-to-digital converters (ADCs) or similar components.
 *
 * @details This variable is used to interface with the AD713x devices through
 * the IIO framework, providing a standardized way to interact with the
 * device's data and configuration.
 ******************************************************************************/
extern struct iio_device ad713x_iio_desc;

/***************************************************************************//**
 * @brief This function sets up and initializes an AD713x IIO device using the
 * provided initialization parameters. It must be called before any
 * operations are performed on the device. The function allocates memory
 * for the device structure and configures it based on the parameters
 * provided. If the initialization is successful, a pointer to the
 * initialized device structure is returned through the `desc` parameter.
 * The function expects valid initialization parameters and will return
 * an error if the parameters are null or if memory allocation fails.
 *
 * @param desc A pointer to a pointer where the initialized AD713x IIO device
 * structure will be stored. Must not be null. The caller is
 * responsible for managing the memory of the structure after
 * initialization.
 * @param param A pointer to an `ad713x_iio_init_param` structure containing the
 * initialization parameters for the AD713x device. Must not be
 * null. The structure should be properly populated with valid
 * device handlers and configuration values before calling this
 * function.
 * @return Returns 0 on successful initialization. Returns a negative error code
 * if initialization fails, such as when parameters are invalid or
 * memory allocation fails.
 ******************************************************************************/
int iio_ad713x_init(struct ad713x_iio **desc,
		    struct ad713x_iio_init_param *param);

/***************************************************************************//**
 * @brief Use this function to release resources associated with an AD713x IIO
 * handler that were previously allocated using iio_ad713x_init. It is
 * important to call this function to prevent memory leaks when the
 * AD713x IIO handler is no longer needed. Ensure that the provided
 * descriptor is not null before calling this function, as passing a null
 * pointer will result in an error.
 *
 * @param desc A pointer to the AD713x IIO handler to be freed. Must not be
 * null. If null, the function returns an error code.
 * @return Returns 0 on successful deallocation, or a negative error code if the
 * input is invalid (e.g., -EINVAL if desc is null).
 ******************************************************************************/
int iio_ad713x_remove(struct ad713x_iio *desc);

#endif /* IIO_SUPPORT */

#endif /* IIO_AD713X */
