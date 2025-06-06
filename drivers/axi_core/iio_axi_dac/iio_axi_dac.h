/***************************************************************************//**
 *   @file   iio_axi_dac.h
 *   @brief  Header file of iio_axi_dac.
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

#ifndef IIO_AXI_DAC_H_
#define IIO_AXI_DAC_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_types.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `iio_axi_dac_desc` structure is a descriptor for an IIO AXI DAC
 * device, encapsulating essential components such as the DAC and DMAC
 * device pointers, an ADC mask, a function pointer for cache management,
 * an IIO device descriptor, and an array for channel names. This
 * structure is used to manage and interface with AXI DAC devices in an
 * IIO context, providing a comprehensive representation of the device's
 * configuration and operational parameters.
 *
 * @param dac Pointer to an AXI DAC device structure.
 * @param dmac Pointer to an AXI DMAC device structure.
 * @param mask 32-bit unsigned integer representing the ADC mask.
 * @param dcache_flush_range Function pointer for flushing the instruction
 * and/or data cache over a specified address range.
 * @param dev_descriptor IIO device descriptor structure.
 * @param ch_names Array of channel names, each with a maximum length of 20
 * characters.
 ******************************************************************************/
struct iio_axi_dac_desc {
	struct axi_dac *dac;
	/** dma device */
	struct axi_dmac *dmac;
	/** ADC mask */
	uint32_t mask;
	/** flush contents of instruction and/or data cache */
	void (*dcache_flush_range)(uint32_t address, uint32_t bytes_count);
	/** iio device descriptor */
	struct iio_device dev_descriptor;
	/** Channel names */
	char (*ch_names)[20];
};

/***************************************************************************//**
 * @brief The `iio_axi_dac_init_param` structure is used to initialize an AXI
 * DAC device in an IIO (Industrial I/O) context. It contains pointers to
 * the DAC and DMA devices, as well as a function pointer for flushing
 * the data cache over a specified address range, which is crucial for
 * ensuring data consistency and performance in embedded systems.
 *
 * @param tx_dac A pointer to an axi_dac structure representing the DAC device.
 * @param tx_dmac A pointer to an axi_dmac structure representing the transmit
 * DMA device.
 * @param dcache_flush_range A function pointer to flush the data cache for a
 * specified address range.
 ******************************************************************************/
struct iio_axi_dac_init_param {
	/** DAC device */
	struct axi_dac *tx_dac;
	/** Transmit DMA device */
	struct axi_dmac *tx_dmac;
	/** Function pointer to flush the data cache for the given address range */
	void (*dcache_flush_range)(uint32_t address, uint32_t bytes_count);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Init application. */
/***************************************************************************//**
 * @brief This function initializes a descriptor for an IIO AXI DAC device,
 * which is necessary for subsequent operations on the DAC. It must be
 * called before any other operations on the DAC are performed. The
 * function requires valid initialization parameters, including a DAC
 * device, and optionally a DMA device and a cache flush function. If the
 * initialization is successful, the descriptor is allocated and
 * populated with the provided parameters. The caller is responsible for
 * managing the memory of the descriptor, including freeing it when no
 * longer needed. The function returns an error code if initialization
 * fails due to invalid parameters or memory allocation issues.
 *
 * @param desc A pointer to a pointer where the initialized descriptor will be
 * stored. Must not be null. The caller takes ownership of the
 * allocated descriptor and is responsible for freeing it.
 * @param init A pointer to an iio_axi_dac_init_param structure containing
 * initialization parameters. Must not be null. The structure must
 * include a valid DAC device. A DMA device and cache flush function
 * are optional.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * invalid parameters or memory allocation failure.
 ******************************************************************************/
int32_t iio_axi_dac_init(struct iio_axi_dac_desc **desc,
			 struct iio_axi_dac_init_param *param);
/***************************************************************************//**
 * @brief This function is used to obtain a pointer to the IIO device descriptor
 * contained within a given DAC descriptor. It is typically called after
 * the DAC descriptor has been initialized, allowing the caller to access
 * the IIO device descriptor for further operations. The function does
 * not perform any validation on the input parameters, so it is the
 * caller's responsibility to ensure that the provided DAC descriptor is
 * valid and properly initialized.
 *
 * @param desc A pointer to a valid and initialized `iio_axi_dac_desc`
 * structure. This parameter must not be null, as it is used to
 * access the internal IIO device descriptor.
 * @param dev_descriptor A pointer to a pointer of type `struct iio_device`.
 * This parameter must not be null, as the function will
 * set it to point to the IIO device descriptor within the
 * provided DAC descriptor.
 * @return None
 ******************************************************************************/
void iio_axi_dac_get_dev_descriptor(struct iio_axi_dac_desc *desc,
				    struct iio_device **dev_descriptor);
/* Free the resources allocated by iio_axi_dac_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a
 * previously initialized IIO AXI DAC descriptor. It should be called
 * when the DAC descriptor is no longer needed to ensure proper cleanup
 * and avoid memory leaks. The function must be called with a valid
 * descriptor that was successfully initialized using iio_axi_dac_init().
 * If the descriptor is null, the function will return an error code.
 *
 * @param desc A pointer to the iio_axi_dac_desc structure to be removed. Must
 * not be null. The caller retains ownership of the pointer, but the
 * memory it points to will be freed by this function. If the
 * pointer is null, the function returns -1.
 * @return Returns 0 on successful removal and cleanup of resources. If the
 * descriptor is null or an error occurs during cleanup, a negative
 * error code is returned.
 ******************************************************************************/
int32_t iio_axi_dac_remove(struct iio_axi_dac_desc *desc);

#endif // IIO_AXI_DAC_H_
