/***************************************************************************//**
 *   @file   iio_admfm2000.h
 *   @brief  Header file for admfm2000 IIO Driver.
 *   @author Ramona Nechita (ramona.nechita@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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

#ifndef SRC_IIO_ADMFM2000_H_
#define SRC_IIO_ADMFM2000_H_

#include <stdint.h>
#include "admfm2000.h"
#include "iio.h"

/***************************************************************************//**
 * @brief The `admfm2000_iio_dev` structure is a compound data type that
 * encapsulates both the device-specific data and the IIO device
 * interface for the ADMFM2000 device. It serves as a bridge between the
 * device's hardware-specific operations and the IIO subsystem, allowing
 * for seamless integration and communication between the two. This
 * structure is essential for initializing and managing the ADMFM2000
 * device within the IIO framework.
 *
 * @param admfm2000_dev A pointer to an admfm2000_dev structure, representing
 * the underlying device-specific data.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct admfm2000_iio_dev {
	struct admfm2000_dev *admfm2000_dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `admfm2000_iio_dev_init_param` structure is designed to
 * encapsulate initialization parameters for the ADMFM2000 IIO device. It
 * contains a single member, which is a pointer to an
 * `admfm2000_init_param` structure, providing the necessary
 * configuration details required to initialize the ADMFM2000 device
 * within an IIO context. This structure is typically used in conjunction
 * with the `admfm2000_iio_init` function to set up the device for
 * operation.
 *
 * @param admfm2000_dev_init A pointer to an admfm2000_init_param structure,
 * used for initializing the admfm2000 device.
 ******************************************************************************/
struct admfm2000_iio_dev_init_param {
	struct admfm2000_init_param *admfm2000_dev_init;
};

/***************************************************************************//**
 * @brief This function sets up an IIO device for the ADMFM2000 by allocating
 * necessary resources and initializing the device with the provided
 * parameters. It must be called before any operations are performed on
 * the ADMFM2000 IIO device. The function expects valid initialization
 * parameters and will return an error code if initialization fails or if
 * memory allocation is unsuccessful. The caller is responsible for
 * managing the memory of the initialized device structure.
 *
 * @param iio_dev A pointer to a pointer where the initialized IIO device
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure and is responsible for
 * freeing it.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the ADMFM2000 device. Must not be null and
 * should be properly initialized before calling this
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -ENOMEM for
 * memory allocation failure).
 ******************************************************************************/
int admfm2000_iio_init(struct admfm2000_iio_dev **iio_dev,
		       struct admfm2000_iio_dev_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly remove and deallocate resources
 * associated with an admfm2000 IIO device. It should be called when the
 * device is no longer needed to ensure that all associated resources are
 * freed. This function must be called only after the device has been
 * successfully initialized and used. It handles any necessary cleanup
 * and deallocation, ensuring that memory is not leaked. If the removal
 * of the underlying device fails, the function will return an error
 * code.
 *
 * @param desc A pointer to an admfm2000_iio_dev structure representing the
 * device to be removed. Must not be null. The caller retains
 * ownership of the pointer, but the function will deallocate the
 * memory associated with the device.
 * @return Returns 0 on successful removal and deallocation of the device. If an
 * error occurs during the removal process, a non-zero error code is
 * returned.
 ******************************************************************************/
int admfm2000_iio_remove(struct admfm2000_iio_dev *desc);

#endif /* SRC_IIO_ADMFM2000_H_ */
