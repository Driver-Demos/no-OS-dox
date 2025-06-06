/***************************************************************************//**
 *   @file   ltc4332.h
 *   @brief  Header file of SPI ltc4332 Interface
 *   @author Paul Benoit (paul.benoit@analog.com)
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

#ifndef SRC_LTC4332_SPI_H_
#define SRC_LTC4332_SPI_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ltc4332_spi_platform_ops` is a constant structure of type
 * `no_os_spi_platform_ops` that provides platform-specific operations
 * for SPI communication. It is defined externally, indicating that its
 * actual definition and initialization occur in another source file.
 * This structure is crucial for abstracting the hardware-specific
 * details of SPI operations, allowing the same high-level code to be
 * used across different platforms.
 *
 * @details This variable is used to define the platform-specific SPI operations
 * for the LTC4332 interface.
 ******************************************************************************/
extern const struct no_os_spi_platform_ops ltc4332_spi_platform_ops;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize the SPI communication peripheral. */
/***************************************************************************//**
 * @brief This function sets up the SPI communication interface using the
 * provided initialization parameters. It must be called before any SPI
 * communication can occur. The function requires a valid parent platform
 * operations structure within the initialization parameters. If the
 * parent is not provided, the function will return an error. Upon
 * successful initialization, the SPI descriptor is populated and can be
 * used for subsequent SPI operations.
 *
 * @param desc A pointer to a pointer of type `struct no_os_spi_desc`. This will
 * be allocated and initialized by the function. The caller must
 * ensure that this pointer is valid and can be modified.
 * @param param A pointer to a `struct no_os_spi_init_param` containing the
 * initialization parameters. This must include a valid `parent`
 * field with platform-specific operations. The caller retains
 * ownership of this structure, and it must not be null.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails.
 ******************************************************************************/
int32_t ltc4332_spi_init(struct no_os_spi_desc **desc,
			 const struct no_os_spi_init_param *param);

/* Free the resources allocated by no_os_spi_init(). */
/***************************************************************************//**
 * @brief Use this function to release the resources associated with an SPI
 * descriptor that was previously initialized. It should be called when
 * the SPI communication is no longer needed to ensure that all allocated
 * memory is properly freed. This function must be called with a valid
 * descriptor that was successfully initialized; passing a null pointer
 * will result in an error.
 *
 * @param desc A pointer to the SPI descriptor to be removed. Must not be null.
 * If null, the function returns an error code.
 * @return Returns 0 on success, or a negative error code if the input is
 * invalid.
 ******************************************************************************/
int32_t ltc4332_spi_remove(struct no_os_spi_desc *desc);

/* Write and read data to/from SPI. */
/***************************************************************************//**
 * @brief This function performs a simultaneous write and read operation over
 * SPI using the provided descriptor. It is typically used when
 * communication with an SPI device requires sending data and receiving a
 * response in a single transaction. The function must be called with a
 * valid SPI descriptor that has been initialized and is associated with
 * a parent descriptor. It handles memory allocation internally and
 * ensures that the original data buffer is updated with the received
 * data. The function returns an error code if the descriptor is invalid
 * or if memory allocation fails.
 *
 * @param desc A pointer to a valid `no_os_spi_desc` structure. This must not be
 * null and must have a valid parent descriptor. The caller retains
 * ownership.
 * @param data A pointer to a buffer containing the data to be written. This
 * buffer will be updated with the data read from the SPI device.
 * Must not be null.
 * @param bytes_number The number of bytes to write and read. Must be a positive
 * integer.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid input or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int32_t ltc4332_spi_write_and_read(struct no_os_spi_desc *desc, uint8_t *data,
				   uint16_t bytes_number);

#endif /* SRC_LTC4332_SPI_H_ */
