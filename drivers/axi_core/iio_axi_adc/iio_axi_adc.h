/***************************************************************************//**
 *   @file   iio_axi_adc.h
 *   @brief  Header file of iio_axi_adc.
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

#ifndef IIO_AXI_ADC_H_
#define IIO_AXI_ADC_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_types.h"
#include "axi_adc_core.h"
#include "axi_dmac.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `iio_axi_adc_desc` structure is a descriptor for an IIO AXI ADC
 * device, encapsulating various components and functionalities required
 * for ADC operations. It includes pointers to ADC and DMA device
 * structures, a mask for ADC operations, and function pointers for cache
 * invalidation and sampling frequency retrieval. Additionally, it holds
 * an IIO device descriptor, an array for channel names, and a pointer to
 * a custom data format structure, facilitating comprehensive management
 * and configuration of ADC functionalities.
 *
 * @param adc Pointer to an ADC device structure.
 * @param mask 32-bit mask for ADC operations.
 * @param dmac Pointer to a DMA device structure.
 * @param dcache_invalidate_range Function pointer to invalidate cache memory
 * for a given address range.
 * @param get_sampling_frequency Function pointer to get the sampling frequency
 * of a specified channel.
 * @param dev_descriptor IIO device descriptor structure.
 * @param ch_names Array of channel names, each with a maximum length of 20
 * characters.
 * @param scan_type_common Pointer to a structure defining the custom data
 * format for scanning.
 ******************************************************************************/
struct iio_axi_adc_desc {
	/** ADC device */
	struct axi_adc *adc;
	/** ADC mask */
	uint32_t mask;
	/** dma device */
	struct axi_dmac *dmac;
	/** Invalidate cache memory function pointer */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	/** Custom implementation for get sampling frequency */
	int (*get_sampling_frequency)(struct axi_adc *dev, uint32_t chan,
				      uint64_t *sampling_freq_hz);
	/** iio device descriptor */
	struct iio_device dev_descriptor;
	/** Channel names */
	char (*ch_names)[20];
	/** Custom data format */
	struct scan_type *scan_type_common;
};

/***************************************************************************//**
 * @brief The `iio_axi_adc_init_param` structure is used to initialize and
 * configure an IIO (Industrial I/O) ADC (Analog-to-Digital Converter)
 * device in a system. It contains pointers to the ADC and DMA devices,
 * function pointers for cache invalidation and custom sampling frequency
 * retrieval, and a pointer to a structure defining the data format
 * common to all channels. This structure is essential for setting up the
 * ADC device with specific configurations and custom behaviors.
 *
 * @param rx_adc Pointer to the ADC device structure.
 * @param rx_dmac Pointer to the DMA device structure for receiving data.
 * @param dcache_invalidate_range Function pointer to invalidate the data cache
 * for a specified address range.
 * @param get_sampling_frequency Function pointer to a custom implementation for
 * retrieving the sampling frequency.
 * @param scan_type_common Pointer to a structure defining the custom data
 * format common to all channels.
 ******************************************************************************/
struct iio_axi_adc_init_param {
	/** ADC device */
	struct axi_adc *rx_adc;
	/** Receive DMA device */
	struct axi_dmac *rx_dmac;
	/** Invalidate the Data cache for the given address range */
	void (*dcache_invalidate_range)(uint32_t address, uint32_t bytes_count);
	/** Custom sampling frequency getter */
	int (*get_sampling_frequency)(struct axi_adc *dev, uint32_t chan,
				      uint64_t *sampling_freq_hz);
	/** Custom data format (unpopulated if not used, set to default)
	    Common to all channels */
	struct scan_type *scan_type_common;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Init iio. */
/***************************************************************************//**
 * @brief This function sets up an IIO AXI ADC descriptor based on the provided
 * initialization parameters. It must be called before using the ADC for
 * any operations. The function allocates memory for the descriptor and
 * configures it using the provided ADC and optional DMA parameters. If
 * the initialization parameters are invalid or memory allocation fails,
 * the function returns an error code. Ensure that the `init` parameter
 * is properly populated before calling this function.
 *
 * @param desc A pointer to a pointer where the initialized descriptor will be
 * stored. Must not be null. The caller is responsible for managing
 * the memory of the descriptor after initialization.
 * @param init A pointer to an `iio_axi_adc_init_param` structure containing
 * initialization parameters. Must not be null and must have a valid
 * `rx_adc` field. If `rx_dmac` is provided, it will be used for DMA
 * operations.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. On success, `*desc` is set to point to the newly allocated
 * descriptor.
 ******************************************************************************/
int32_t iio_axi_adc_init(struct iio_axi_adc_desc **desc,
			 struct iio_axi_adc_init_param *param);

/***************************************************************************//**
 * @brief This function is used to obtain a pointer to the IIO device descriptor
 * contained within a given IIO AXI ADC descriptor. It is typically
 * called after the ADC descriptor has been properly initialized,
 * allowing the caller to access the IIO device descriptor for further
 * operations. The function does not perform any validation on the input
 * parameters, so it is the caller's responsibility to ensure that the
 * provided ADC descriptor is valid and properly initialized.
 *
 * @param desc A pointer to a valid and initialized `iio_axi_adc_desc`
 * structure. This parameter must not be null, as it is used to
 * access the internal IIO device descriptor.
 * @param dev_descriptor A pointer to a pointer of type `struct iio_device`.
 * This parameter must not be null, as the function will
 * set it to point to the IIO device descriptor within the
 * provided ADC descriptor.
 * @return None
 ******************************************************************************/
void iio_axi_adc_get_dev_descriptor(struct iio_axi_adc_desc *desc,
				    struct iio_device **dev_descriptor);

/* Free the resources allocated by iio_axi_adc_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an IIO AXI
 * ADC descriptor that was previously initialized using
 * iio_axi_adc_init(). It is essential to call this function to prevent
 * memory leaks when the descriptor is no longer needed. The function
 * must be called with a valid descriptor pointer; otherwise, it will
 * return an error. Ensure that no other operations are performed on the
 * descriptor after calling this function.
 *
 * @param desc A pointer to the iio_axi_adc_desc structure to be removed. Must
 * not be null. If the pointer is null, the function returns an
 * error code (-1). The caller retains ownership of the pointer, but
 * the memory it points to will be freed.
 * @return Returns 0 on successful removal of the descriptor and freeing of
 * resources. Returns a negative error code if the descriptor is null or
 * if an error occurs during the removal process.
 ******************************************************************************/
int32_t iio_axi_adc_remove(struct iio_axi_adc_desc *desc);

#endif // IIO_AXI_ADC_H_
