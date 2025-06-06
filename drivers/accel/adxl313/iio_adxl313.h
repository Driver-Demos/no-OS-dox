/***************************************************************************//**
 *   @file   iio_adxl313.h
 *   @brief  Header file of IIO ADXL355 Driver.
 *   @author GMois (george.mois@analog.com)
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
#ifndef IIO_ADXL313_H
#define IIO_ADXL313_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `adxl313_iio_dev` structure is designed to encapsulate the
 * necessary components for interfacing with an ADXL313 device through
 * the Industrial I/O (IIO) subsystem. It includes pointers to both the
 * ADXL313 device and the IIO device structures, allowing for integration
 * and communication between the hardware and the IIO framework.
 * Additionally, it maintains information about the active channels and
 * their count, which is crucial for managing data acquisition and
 * processing tasks.
 *
 * @param adxl313_dev A pointer to an ADXL313 device structure.
 * @param iio_dev A pointer to an IIO device structure.
 * @param active_channels A 32-bit integer representing the active channels.
 * @param no_of_active_channels An 8-bit integer representing the number of
 * active channels.
 ******************************************************************************/
struct adxl313_iio_dev {
	struct adxl313_dev *adxl313_dev;
	struct iio_device *iio_dev;
	uint32_t active_channels;
	uint8_t no_of_active_channels;
};

/***************************************************************************//**
 * @brief The `adxl313_iio_dev_init_param` structure is used to encapsulate
 * initialization parameters for setting up an ADXL313 IIO device. It
 * contains a single member, `adxl313_dev_init`, which is a pointer to
 * another structure that holds the specific initialization parameters
 * required for the ADXL313 device. This structure is typically used in
 * the context of initializing the device within the IIO (Industrial I/O)
 * framework.
 *
 * @param adxl313_dev_init A pointer to an adxl313_init_param structure, used to
 * initialize the ADXL313 device.
 ******************************************************************************/
struct adxl313_iio_dev_init_param {
	struct adxl313_init_param *adxl313_dev_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function sets up an IIO device for the ADXL313 accelerometer by
 * allocating necessary resources and configuring the device with default
 * settings. It must be called before any operations are performed on the
 * ADXL313 device through the IIO interface. The function expects valid
 * initialization parameters and will return an error code if
 * initialization fails, ensuring that resources are properly cleaned up
 * in such cases. The caller is responsible for managing the memory of
 * the created device structure.
 *
 * @param iio_dev A pointer to a pointer where the initialized IIO device
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure and is responsible for
 * freeing it using the appropriate remove function.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the ADXL313 device. Must not be null and
 * should be properly initialized before calling this
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered.
 ******************************************************************************/
int adxl313_iio_init(struct adxl313_iio_dev **iio_dev,
		     struct adxl313_iio_dev_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and free resources associated
 * with an ADXL313 IIO device instance. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * released. The function first attempts to remove the underlying ADXL313
 * device and, if successful, proceeds to free the memory associated with
 * the IIO device descriptor. It is important to ensure that the
 * descriptor provided is valid and was previously initialized by
 * `adxl313_iio_init`. The function returns an error code if the removal
 * of the underlying device fails.
 *
 * @param desc A pointer to an `adxl313_iio_dev` structure representing the IIO
 * device instance to be removed. Must not be null and should point
 * to a valid, initialized device descriptor. The caller retains
 * ownership of the pointer, but the memory it points to will be
 * freed by this function.
 * @return Returns 0 on successful removal and deallocation, or a negative error
 * code if the underlying device removal fails.
 ******************************************************************************/
int adxl313_iio_remove(struct adxl313_iio_dev *desc);

#endif /** IIO_ADXL355_H */
