/***************************************************************************//**
 *   @file   demux_spi.h
 *   @brief  Header file of SPI Demux Interface
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

#ifndef SRC_DEMUX_SPI_H_
#define SRC_DEMUX_SPI_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define CS_OFFSET 0x80

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `demux_spi_platform_ops` is a constant structure of type
 * `no_os_spi_platform_ops` that is declared as an external variable.
 * This structure is intended to hold platform-specific operations for
 * SPI communication, allowing for abstraction and flexibility in
 * handling SPI operations across different hardware platforms.
 *
 * @details This variable is used to define and access the platform-specific SPI
 * operations required for initializing, removing, and performing
 * read/write operations on SPI devices.
 ******************************************************************************/
extern const struct no_os_spi_platform_ops demux_spi_platform_ops;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Initialize the SPI communication peripheral. */
/***************************************************************************//**
 * @brief This function sets up the SPI communication peripheral using the
 * provided initialization parameters. It must be called before any SPI
 * communication can occur. The function allocates memory for the SPI
 * descriptor and initializes it with the specified parameters. If the
 * initialization is successful, the descriptor is returned through the
 * provided pointer. The function should be called with valid
 * initialization parameters, and the caller is responsible for managing
 * the memory of the descriptor after initialization. If the
 * initialization fails, the function returns an error code and no memory
 * is allocated.
 *
 * @param desc A pointer to a pointer where the initialized SPI descriptor will
 * be stored. Must not be null. The caller is responsible for
 * freeing the allocated memory.
 * @param param A pointer to a structure containing the initialization
 * parameters for the SPI peripheral. Must not be null. If null,
 * the function returns an error code.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails.
 ******************************************************************************/
int32_t demux_spi_init(struct no_os_spi_desc **desc,
		       const struct no_os_spi_init_param *param);

/* Free the resources allocated by no_os_spi_init(). */
/***************************************************************************//**
 * @brief Use this function to release resources associated with an SPI
 * descriptor that was previously initialized. It should be called when
 * the SPI communication is no longer needed to ensure proper cleanup and
 * avoid memory leaks. The function must be called with a valid
 * descriptor that was successfully initialized; passing a null pointer
 * will result in an error. This function also handles the removal of any
 * additional resources associated with the descriptor.
 *
 * @param desc A pointer to a struct no_os_spi_desc that represents the SPI
 * descriptor to be removed. Must not be null. If the pointer is
 * null, the function returns an error code.
 * @return Returns 0 on successful removal of the SPI descriptor and its
 * resources. Returns -1 if the descriptor is null or if an error occurs
 * during the removal process.
 ******************************************************************************/
int32_t demux_spi_remove(struct no_os_spi_desc *desc);

/* Write and read data to/from SPI. */
/***************************************************************************//**
 * @brief This function facilitates SPI communication by writing data to and
 * reading data from an SPI device through a demultiplexer. It requires a
 * valid SPI descriptor that has been initialized with the demux SPI
 * platform operations. The function modifies the provided data buffer by
 * writing to the SPI device and then updating it with the data read
 * back. It is essential to ensure that the descriptor is not null before
 * calling this function, as a null descriptor will result in an error.
 * The function handles memory allocation internally and returns an error
 * code if memory allocation fails or if the SPI operation is
 * unsuccessful.
 *
 * @param desc A pointer to a no_os_spi_desc structure representing the SPI
 * device. Must not be null. The descriptor should be initialized
 * with the demux SPI platform operations.
 * @param data A pointer to a buffer containing the data to be written. The
 * buffer will be updated with the data read from the SPI device.
 * The caller retains ownership of the buffer.
 * @param bytes_number The number of bytes to write and read. Must be a positive
 * integer.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure, such as when memory allocation fails or the SPI
 * operation is unsuccessful.
 ******************************************************************************/
int32_t demux_spi_write_and_read(struct no_os_spi_desc *desc, uint8_t *data,
				 uint16_t bytes_number);

#endif /* SRC_DEMUX_SPI_H_ */
