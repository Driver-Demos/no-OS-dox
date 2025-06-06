/***************************************************************************//**
 *   @file   ltc2358.h
 *   @brief  Implementation of LTC2358 Driver.
 *   @author Kim Seer Paller (kimseer.paller@analog.com)
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

#ifndef LTC2358_H
#define LTC2358_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define LTC2358_BYTES_PER_CH    3
#define LTC2358_CHANNEL_MSK     NO_OS_GENMASK(2, 0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ltc2358_init_param` structure is designed to hold the
 * initialization parameters necessary for setting up the LTC2358 device,
 * specifically focusing on the SPI (Serial Peripheral Interface)
 * configuration. It contains a single member, `spi_init`, which is a
 * pointer to a `no_os_spi_init_param` structure, encapsulating all the
 * necessary details for initializing the SPI communication required by
 * the LTC2358 device.
 *
 * @param spi_init A pointer to a structure containing SPI initialization
 * parameters.
 ******************************************************************************/
struct ltc2358_init_param {
	/* SPI Initialization structure. */
	struct no_os_spi_init_param *spi_init;
};

/***************************************************************************//**
 * @brief The `ltc2358_dev` structure is a descriptor for the LTC2358 device,
 * primarily used to manage and facilitate SPI communication with the
 * device. It contains a single member, `spi_desc`, which is a pointer to
 * a `no_os_spi_desc` structure, representing the SPI handler necessary
 * for interfacing with the LTC2358 ADC (Analog-to-Digital Converter).
 * This structure is essential for initializing, configuring, and
 * managing data transactions with the LTC2358 through SPI.
 *
 * @param spi_desc A pointer to a SPI descriptor used for handling SPI
 * communication.
 ******************************************************************************/
struct ltc2358_dev {
	/* SPI handler */
	struct no_os_spi_desc *spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Creates 24-bit configuration word for the 8 channels. */
/***************************************************************************//**
 * @brief This function is used to generate a 24-bit configuration word for one
 * of the eight channels of the LTC2358 device. It is typically called
 * when setting up the device configuration for data acquisition. The
 * function modifies the provided configuration word by setting the bits
 * corresponding to the specified channel and configuration number. It is
 * important to ensure that the `config_word` pointer is valid and points
 * to a pre-allocated 32-bit integer, as the function will modify the
 * value it points to. The function does not perform any validation on
 * the input parameters, so the caller must ensure that the channel and
 * configuration number are within valid ranges.
 *
 * @param channel Specifies the channel number for which the configuration word
 * is being created. Valid values are typically 0 to 7,
 * corresponding to the eight channels of the LTC2358.
 * @param config_number Specifies the configuration number to be applied to the
 * specified channel. It is masked with
 * `LTC2358_CHANNEL_MSK` to ensure it fits within the
 * expected bit range.
 * @param config_word A pointer to a 32-bit unsigned integer where the
 * configuration word will be stored. Must not be null, and
 * the caller is responsible for ensuring it points to a
 * valid memory location.
 * @return None
 ******************************************************************************/
void ltc2358_create_config_word(uint8_t channel, uint8_t config_number,
				uint32_t *config_word);

/* Parse single channel data. */
/***************************************************************************//**
 * @brief This function retrieves and parses the data for a specific channel
 * from the LTC2358 device. It should be called after the device has been
 * properly initialized and configured. The function writes the parsed
 * 24-bit data of the specified channel into the provided output
 * parameter. It is important to ensure that the `data_array` contains
 * valid data for all channels, and the `channel` parameter is within the
 * valid range of 0 to 7. The function returns an error code if the data
 * retrieval fails.
 *
 * @param dev A pointer to an `ltc2358_dev` structure representing the device.
 * Must not be null.
 * @param config_word A 24-bit configuration word used for the device operation.
 * Must be correctly set up before calling.
 * @param data_array An array of 24 bytes containing the data for all channels.
 * Must be populated with valid data before calling.
 * @param channel An unsigned 8-bit integer specifying the channel number to
 * read from. Valid range is 0 to 7.
 * @param readval A pointer to a 32-bit unsigned integer where the parsed
 * channel data will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if data retrieval
 * fails.
 ******************************************************************************/
int32_t ltc2358_channel_data(struct ltc2358_dev *dev, uint32_t config_word,
			     uint8_t data_array[24], uint8_t channel, uint32_t *readval);

/* Initializes the LTC2358. */
/***************************************************************************//**
 * @brief This function sets up the LTC2358 device by allocating memory for the
 * device structure and initializing the SPI interface using the provided
 * parameters. It must be called before any other operations on the
 * LTC2358 device to ensure that the device is properly configured. The
 * function expects valid initialization parameters and will return an
 * error code if memory allocation fails or if the SPI initialization is
 * unsuccessful. The caller is responsible for managing the memory of the
 * device structure, which should be freed using `ltc2358_remove()` when
 * no longer needed.
 *
 * @param device A double pointer to an ltc2358_dev structure where the
 * initialized device descriptor will be stored. Must not be null.
 * The caller takes ownership of the allocated memory.
 * @param init_param A pointer to an ltc2358_init_param structure containing the
 * initialization parameters, including SPI configuration.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @return Returns 0 on successful initialization, or a negative error code if
 * memory allocation or SPI initialization fails.
 ******************************************************************************/
int32_t ltc2358_init(struct ltc2358_dev **device,
		     struct ltc2358_init_param *init_param);

/* Free the resources allocated by ltc2358_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an LTC2358
 * device that were previously allocated during initialization. It should
 * be called when the device is no longer needed to ensure proper cleanup
 * and avoid memory leaks. The function must be called with a valid
 * device descriptor obtained from a successful call to ltc2358_init().
 * If the provided device descriptor is null, the function will return an
 * error code indicating that the device is not available.
 *
 * @param dev A pointer to an ltc2358_dev structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * -ENODEV.
 * @return Returns 0 on successful removal of the device. If the device
 * descriptor is null, returns -ENODEV. If an error occurs during the
 * SPI removal process, the function returns the corresponding error
 * code.
 ******************************************************************************/
int32_t ltc2358_remove(struct ltc2358_dev *dev);

#endif /* LTC2358_H */
