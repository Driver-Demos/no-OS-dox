/***************************************************************************//**
*   @file   iio_ad7746.h
*   @brief  Header file of iio_ad7746
*   @author Darius Berghe (darius.berghe@analog.com)
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

#ifndef IIO_AD7746_H
#define IIO_AD7746_H

#include "iio.h"

/***************************************************************************//**
 * @brief The `ad7746_iio_dev` structure is a compound data type that
 * encapsulates the necessary components for interfacing with an AD7746
 * capacitive sensor through the Industrial I/O (IIO) framework. It
 * includes pointers to both the AD7746 device and the IIO device
 * structures, allowing for integration and communication between the
 * hardware and software layers. The structure also contains a 2x2 array
 * for configuring capacitance DAC settings for two channels, supporting
 * both single-ended and differential modes, and an integer to track the
 * current DAC setting.
 *
 * @param ad7746_dev A pointer to an ad7746_dev structure, representing the
 * AD7746 device instance.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device instance.
 * @param capdac A 2x2 array of uint8_t, representing capacitance DAC settings
 * for two channels, with single-ended and differential
 * configurations.
 * @param capdac_set An int8_t indicating the current setting of the capacitance
 * DAC.
 ******************************************************************************/
struct ad7746_iio_dev {
	struct ad7746_dev *ad7746_dev;
	struct iio_device *iio_dev;
	// capdac[0] - first channel, capdac[1] - second channel
	// capdac[_][0] - single-ended, capdac[_][1] - differential
	uint8_t capdac[2][2];
	int8_t capdac_set;
};

/***************************************************************************//**
 * @brief The `ad7746_iio_init_param` structure is designed to hold
 * initialization parameters for setting up an AD7746 IIO device. It
 * contains a single member, `ad7746_initial`, which is a pointer to
 * another structure, `ad7746_init_param`, that presumably contains the
 * necessary configuration settings for initializing the AD7746 device.
 * This structure is used in the initialization function
 * `ad7746_iio_init` to properly configure the device before use.
 *
 * @param ad7746_initial A pointer to an ad7746_init_param structure, used for
 * initialization parameters.
 ******************************************************************************/
struct ad7746_iio_init_param {
	struct ad7746_init_param *ad7746_initial;
};

/***************************************************************************//**
 * @brief This function initializes an AD7746 IIO device structure, preparing it
 * for use with the specified initialization parameters. It allocates
 * memory for the device structure and sets up the internal device
 * configuration based on the provided initialization parameters. This
 * function must be called before any operations are performed on the
 * AD7746 IIO device. If the initialization fails, the function returns a
 * negative error code and no device structure is allocated. The caller
 * is responsible for managing the memory of the initialized device
 * structure, including freeing it when no longer needed.
 *
 * @param iio_dev A pointer to a pointer where the initialized AD7746 IIO device
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure and is responsible for
 * freeing it.
 * @param init_param A pointer to an initialization parameter structure
 * containing the initial settings for the AD7746 device. Must
 * not be null. The structure should be properly populated
 * before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and the iio_dev pointer is not modified.
 ******************************************************************************/
int32_t ad7746_iio_init(struct ad7746_iio_dev **iio_dev,
			struct ad7746_iio_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to properly release and clean up resources
 * associated with an AD7746 IIO device. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * freed and any associated device operations are terminated. This
 * function must be called only after a successful initialization of the
 * device using `ad7746_iio_init`. Failure to call this function may
 * result in resource leaks. The function returns an error code if the
 * removal of the underlying device fails, otherwise it returns zero.
 *
 * @param desc A pointer to an `ad7746_iio_dev` structure representing the
 * device to be removed. This pointer must not be null and should
 * point to a valid, initialized device structure. The function will
 * handle freeing the resources associated with this structure.
 * @return Returns 0 on successful removal and cleanup of the device. If an
 * error occurs during the removal process, a negative error code is
 * returned.
 ******************************************************************************/
int32_t ad7746_iio_remove(struct ad7746_iio_dev *desc);

#endif /** IIO_AD7746_H */
