/***************************************************************************//**
*   @file   sd.h
*   @brief  Header file for SD card interface over SPI.
*   @author Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
* @copyright
*
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
********************************************************************************
*
* @section details Library description
* This interface supports byte read and write operations for SD cards that
* that meet the following conditions:
*   	- Version 2.00 or later
*   	- High capacity or extended capacity (SDHX or SDXC)
*   	- Supply voltage of 3.3V
*
*******************************************************************************/

#ifndef __SD_H__
#define __SD_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define DATA_BLOCK_LEN			(512u)
#define MAX_RESPONSE_LEN		(18u)

#ifdef SD_DEBUG
#include <stdio.h>
#define DEBUG_MSG(X)			printf((X))
#else
#define DEBUG_MSG(X)
#endif //SD_DEBUG

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `sd_init_param` structure is used to configure the initialization
 * parameters for an SD card interface over SPI. It contains a single
 * member, `spi_desc`, which is a pointer to a `no_os_spi_desc` structure
 * that describes an initialized SPI channel. This structure is typically
 * used as an argument in functions that initialize SD card operations,
 * ensuring that the SPI channel is properly set up for communication
 * with the SD card.
 *
 * @param spi_desc A pointer to a descriptor of an initialized SPI channel.
 ******************************************************************************/
struct sd_init_param {
	/** Descriptor of an initialized SPI channel */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `sd_desc` structure is designed to store configuration and state
 * information for an SD card interface over SPI. It includes a pointer
 * to an SPI descriptor, which is used to manage the communication
 * channel, the total memory size of the SD card, a flag indicating
 * whether the card is high capacity or extended capacity, and a buffer
 * for internal driver operations. This structure is essential for
 * managing and interacting with SD cards in embedded systems.
 *
 * @param spi_desc Descriptor of an initialized SPI channel.
 * @param memory_size Memory size of the SD card in bytes.
 * @param high_capacity Indicates if the SD card is high capacity (HC) or
 * extended capacity (XC).
 * @param buff Buffer used for the driver implementation.
 ******************************************************************************/
struct sd_desc {
	/** Descriptor of an initialized SPI channel */
	struct no_os_spi_desc	*spi_desc;
	/** Memory size of the SD card in bytes */
	uint64_t	memory_size;
	/** 1 if SD card is HC or XC, 0 otherwise */
	uint8_t		high_capacity;
	/** Buffer used for the driver implementation */
	uint8_t		buff[18];
};

/***************************************************************************//**
 * @brief The `cmd_desc` structure is designed to encapsulate the necessary
 * components for constructing and handling a command in the context of
 * an SD card interface over SPI. It includes a command code, an argument
 * for the command, a buffer to store the response, and the expected
 * length of the response. This structure is essential for managing
 * command transactions with the SD card, ensuring that commands are
 * properly formatted and responses are correctly handled.
 *
 * @param cmd Command code represented as an 8-bit unsigned integer.
 * @param arg 32-bit unsigned integer representing the argument for the command.
 * @param response Array to store the response, with a maximum length defined by
 * MAX_RESPONSE_LEN.
 * @param response_len 32-bit unsigned integer indicating the expected length of
 * the response.
 ******************************************************************************/
struct cmd_desc {
	/** Command code */
	uint8_t		cmd;
	/** Argument for the command */
	uint32_t	arg;
	/** Response with the size response_len will be written here */
	uint8_t		response[MAX_RESPONSE_LEN];
	/** Expected length for the response */
	uint32_t	response_len;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes an SD card for communication over an SPI
 * interface. It must be called before any other operations on the SD
 * card, such as reading or writing. The function configures the SD card
 * to operate in SPI mode and checks for compatibility with version 2.0
 * or later, as well as high capacity (SDHX) or extended capacity (SDXC)
 * cards. It requires a valid SPI descriptor and initialization
 * parameters. If initialization fails, the function returns an error
 * code and does not modify the provided descriptor.
 *
 * @param sd_desc A pointer to a pointer of type struct sd_desc. This must not
 * be null. On successful initialization, it will point to a
 * newly allocated and initialized SD card descriptor. The caller
 * is responsible for managing the memory of this descriptor.
 * @param param A pointer to a struct sd_init_param containing the
 * initialization parameters, including a valid SPI descriptor.
 * This must not be null. The function uses this to configure the
 * SD card.
 * @return Returns 0 on successful initialization. Returns -1 if initialization
 * fails due to invalid parameters or unsupported SD card features.
 ******************************************************************************/
int32_t sd_init(struct sd_desc **sd_desc,
		const struct sd_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources
 * associated with an SD card descriptor when it is no longer needed.
 * This function should be called to prevent memory leaks after the SD
 * card operations are complete. It is important to ensure that the
 * descriptor is valid and initialized before calling this function. If
 * the descriptor is null, the function will return an error code.
 *
 * @param desc A pointer to an `sd_desc` structure representing the SD card
 * descriptor to be removed. Must not be null. If null, the function
 * returns an error code.
 * @return Returns 0 on successful removal of the descriptor, or -1 if the
 * descriptor is null.
 ******************************************************************************/
int32_t sd_remove(struct sd_desc *desc);
/***************************************************************************//**
 * @brief Use this function to read a specified number of bytes from an SD card
 * starting at a given address into a provided buffer. It is essential to
 * ensure that the buffer is large enough to hold the data being read and
 * that the address and length do not exceed the SD card's memory size.
 * The function must be called with a valid SD card descriptor, and it
 * will return an error if any parameter is invalid or if the read
 * operation fails.
 *
 * @param sd_desc A pointer to an initialized 'sd_desc' structure representing
 * the SD card. Must not be null and should be properly
 * initialized before calling this function.
 * @param data A pointer to a buffer where the read data will be stored. Must
 * not be null and should have enough space to store 'len' bytes.
 * @param address The starting address on the SD card from which to read data.
 * Must be within the bounds of the SD card's memory size.
 * @param len The number of bytes to read from the SD card. The sum of 'address'
 * and 'len' must not exceed the SD card's memory size.
 * @return Returns 0 on success, or -1 if an error occurs (e.g., invalid
 * parameters or read failure).
 ******************************************************************************/
int32_t sd_read(struct sd_desc *desc,
		uint8_t *data,
		uint64_t address,
		uint64_t len);
/***************************************************************************//**
 * @brief Use this function to write a specified number of bytes to an SD card
 * starting at a given address. It is essential to ensure that the
 * address and length of data do not exceed the memory size of the SD
 * card. The function handles partial block writes by reading and
 * updating the necessary blocks before writing. It must be called with a
 * valid SD card descriptor obtained from a successful initialization.
 * The function returns an error if the input parameters are invalid or
 * if the write operation fails.
 *
 * @param sd_desc A pointer to an initialized 'sd_desc' structure representing
 * the SD card. Must not be null and should be properly
 * initialized using 'sd_init'.
 * @param data A pointer to the data buffer containing the bytes to be written.
 * Must not be null.
 * @param address The starting address on the SD card where the data should be
 * written. Must be within the bounds of the SD card's memory
 * size.
 * @param len The number of bytes to write. The sum of 'address' and 'len' must
 * not exceed the SD card's memory size.
 * @return Returns 0 on success, or -1 if an error occurs (e.g., invalid
 * parameters or write failure).
 ******************************************************************************/
int32_t sd_write(struct sd_desc *desc,
		 uint8_t *data,
		 uint64_t address,
		 uint64_t len);

#endif /* __SD_H__ */

