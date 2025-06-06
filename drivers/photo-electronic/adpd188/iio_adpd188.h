/***************************************************************************//**
*   @file   iio_adpd188.h
*   @brief  Header of the ADPD188 IIO driver.
*   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
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

#ifndef IIO_ADPD188_H
#define IIO_ADPD188_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "adpd188.h"
#include "iio.h"

#define ADPD1080_CHANNEL_NO		8
#define ADPD1080_WORD_BIT_SIZE		16
#define ADPD1080_BITS_PER_SAMPLE	32
#define ADPD1080_WORDS_PER_SAMPLE	\
	(ADPD1080_BITS_PER_SAMPLE / ADPD1080_WORD_BIT_SIZE)

/***************************************************************************//**
 * @brief The `adpd188_iio_init_param` structure is used to encapsulate
 * initialization parameters specifically for the ADPD188 IIO driver. It
 * contains a single member, `drv_init_param`, which is a structure that
 * holds the necessary parameters to initialize the ADPD188 device
 * driver. This structure is essential for setting up the device in a
 * consistent and organized manner, ensuring that all required
 * initialization data is passed correctly to the driver.
 *
 * @param drv_init_param A structure of type adpd188_init_param that holds
 * initialization parameters for the ADPD188 driver.
 ******************************************************************************/
struct adpd188_iio_init_param {
	struct adpd188_init_param drv_init_param;
};

/***************************************************************************//**
 * @brief The `adpd188_iio_desc` structure is used to describe an instance of
 * the ADPD188 IIO device driver. It contains a pointer to the device
 * driver structure (`drv_dev`) and a channel mask (`ch_mask`) that
 * specifies which channels are active or configured for use. This
 * structure is essential for managing the state and configuration of the
 * ADPD188 device within the IIO framework.
 *
 * @param drv_dev A pointer to an adpd188_dev structure, representing the device
 * driver instance.
 * @param ch_mask An 8-bit unsigned integer representing the channel mask for
 * the device.
 ******************************************************************************/
struct adpd188_iio_desc {
	struct adpd188_dev *drv_dev;
	uint8_t ch_mask;
};

/***************************************************************************//**
 * @brief The `iio_adpd188_device` is an external global variable of type
 * `struct iio_device`. It represents an Industrial I/O (IIO) device
 * interface for the ADPD188 sensor, which is a photometric front end for
 * optical measurement applications. This variable is likely used to
 * facilitate communication and data exchange between the ADPD188 sensor
 * and the IIO subsystem in a Linux environment.
 *
 * @details This variable is used to define the IIO device interface for the
 * ADPD188 sensor, enabling its integration with the IIO framework.
 ******************************************************************************/
extern struct iio_device iio_adpd188_device;

/***************************************************************************//**
 * @brief This function sets up and initializes the ADPD188 IIO device using the
 * provided initialization parameters. It must be called before any
 * operations are performed on the device. The function allocates memory
 * for the device descriptor and configures the device's slots and
 * registers. If initialization fails at any step, the function ensures
 * that resources are properly freed and returns an error code. The
 * caller is responsible for managing the memory of the device descriptor
 * and must call the corresponding removal function to free resources
 * when the device is no longer needed.
 *
 * @param device A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated memory and is responsible for freeing it.
 * @param init_param A pointer to a structure containing the initialization
 * parameters for the device. Must not be null. The structure
 * should be properly populated with valid initialization data
 * before calling this function.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during the process.
 ******************************************************************************/
int32_t adpd188_iio_init(struct adpd188_iio_desc **device,
			 struct adpd188_iio_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to properly remove an ADPD188 IIO device
 * descriptor and free associated resources. It should be called when the
 * device is no longer needed to ensure that all resources are released.
 * The function first attempts to remove the underlying driver device
 * and, if successful, deallocates the memory associated with the IIO
 * descriptor. It is important to ensure that the descriptor is valid and
 * initialized before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to the ADPD188 IIO device descriptor to be removed. Must
 * not be null and should point to a valid, initialized descriptor.
 * If the descriptor is invalid or null, the behavior is undefined.
 * @return Returns 0 on successful removal and deallocation, or a negative error
 * code if the underlying driver device removal fails.
 ******************************************************************************/
int32_t adpd188_iio_remove(struct adpd188_iio_desc *dev);

#endif /* IIO_ADPD188_H */

