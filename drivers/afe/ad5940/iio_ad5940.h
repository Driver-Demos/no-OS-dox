/***************************************************************************//**
 *   @file   iio_ad5940.h
 *   @brief  Header file of ad5940 iio driver.
 *   @author Darius Berghe (darius.berghe@analog.com)
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
#ifndef IIO_AD5940_H
#define IIO_AD5940_H

#include "iio.h"
#include "ad5940.h"

/***************************************************************************//**
 * @brief The `ad5940_iio_attr` enumeration defines a set of attributes for
 * configuring the AD5940 IIO device, which is used in interfacing with
 * the AD5940 chip. Each enumerator corresponds to a specific
 * configuration parameter, such as excitation frequency, amplitude,
 * impedance mode, magnitude mode, and GPIO1 toggle, allowing for
 * flexible control over the device's operational settings.
 *
 * @param AD5940_IIO_EXCITATION_FREQUENCY Represents the excitation frequency
 * attribute for the AD5940 IIO device.
 * @param AD5940_IIO_EXCITATION_AMPLITUDE Represents the excitation amplitude
 * attribute for the AD5940 IIO device.
 * @param AD5940_IIO_IMPEDANCE_MODE Represents the impedance mode attribute for
 * the AD5940 IIO device.
 * @param AD5940_IIO_MAGNITUDE_MODE Represents the magnitude mode attribute for
 * the AD5940 IIO device.
 * @param AD5940_IIO_GPIO1_TOGGLE Represents the GPIO1 toggle attribute for the
 * AD5940 IIO device.
 ******************************************************************************/
enum ad5940_iio_attr {
	AD5940_IIO_EXCITATION_FREQUENCY,
	AD5940_IIO_EXCITATION_AMPLITUDE,
	AD5940_IIO_IMPEDANCE_MODE,
	AD5940_IIO_MAGNITUDE_MODE,
	AD5940_IIO_GPIO1_TOGGLE,
};

/***************************************************************************//**
 * @brief The `ad5940_iio_dev` structure is a compound data type that
 * encapsulates the necessary components for interfacing with an AD5940
 * device through the IIO framework. It includes pointers to the AD5940
 * device and the IIO device interface, as well as configuration flags
 * for operational modes and a buffer for application-specific data
 * handling. This structure is essential for managing the state and
 * configuration of the AD5940 device in an IIO context.
 *
 * @param ad5940 A pointer to an ad5940_dev structure, representing the AD5940
 * device.
 * @param iio A pointer to an iio_device structure, representing the IIO device
 * interface.
 * @param magnitude_mode A boolean flag indicating if the magnitude mode is
 * enabled.
 * @param gpio1 A boolean flag indicating the state of GPIO1.
 * @param AppBuff An array of 512 uint32_t elements used as an application
 * buffer.
 ******************************************************************************/
struct ad5940_iio_dev {
	struct ad5940_dev *ad5940;
	struct iio_device *iio;
	bool magnitude_mode;
	bool gpio1;
	uint32_t AppBuff[512];
};

/***************************************************************************//**
 * @brief The `ad5940_iio_init_param` structure is a simple data structure used
 * to encapsulate initialization parameters for the AD5940 IIO driver. It
 * contains a single member, which is a pointer to another structure,
 * `ad5940_init_param`, that holds the specific initialization settings
 * required to configure the AD5940 device. This structure is typically
 * used when setting up the AD5940 device in an IIO (Industrial I/O)
 * context, facilitating the transfer of initialization parameters to the
 * device initialization function.
 *
 * @param ad5940_init A pointer to an ad5940_init_param structure, used for
 * initializing the AD5940 device.
 ******************************************************************************/
struct ad5940_iio_init_param {
	struct ad5940_init_param *ad5940_init;
};

/***************************************************************************//**
 * @brief This function sets up and initializes an AD5940 IIO device, preparing
 * it for use with the specified initialization parameters. It must be
 * called before any operations are performed on the device. The function
 * allocates memory for the device structure and its components, and
 * configures the device according to the provided initialization
 * parameters. If the initialization is successful, the function returns
 * a pointer to the initialized device structure. If any error occurs
 * during initialization, such as invalid parameters or memory allocation
 * failure, the function returns a negative error code.
 *
 * @param iio_dev A double pointer to an AD5940 IIO device structure. This must
 * not be null. On successful initialization, it will point to
 * the newly allocated and initialized device structure. The
 * caller is responsible for managing the memory of this
 * structure after initialization.
 * @param init_param A pointer to an AD5940 IIO initialization parameter
 * structure. This must not be null and should be properly
 * initialized with the necessary parameters for the AD5940
 * device. Invalid or null input will result in an error.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -EINVAL for
 * invalid input or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int32_t ad5940_iio_init(struct ad5940_iio_dev **iio_dev,
			struct ad5940_iio_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly remove an AD5940 IIO device instance
 * when it is no longer needed. This function ensures that all associated
 * resources are freed, preventing memory leaks. It should be called only
 * after the device has been initialized and is no longer in use. The
 * function will return an error code if the underlying AD5940 device
 * removal fails, otherwise it will return 0 upon successful deallocation
 * of resources.
 *
 * @param desc A pointer to the ad5940_iio_dev structure representing the device
 * instance to be removed. Must not be null. The caller retains
 * ownership of the pointer, but the memory it points to will be
 * freed by this function.
 * @return Returns 0 on successful removal and deallocation of the device
 * instance. If the underlying AD5940 device removal fails, it returns a
 * non-zero error code.
 ******************************************************************************/
int32_t ad5940_iio_remove(struct ad5940_iio_dev *desc);

#endif /** IIO_AD5940_H */

