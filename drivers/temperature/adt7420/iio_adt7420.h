/***************************************************************************//**
 *   @file   iio_adt7420.h
 *   @brief  Header file of IIO ADT7420 Driver.
 *   @author RNechita (ramona.nechita@analog.com)
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
#ifndef IIO_ADT7420_H
#define IIO_ADT7420_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "iio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `adt7420_iio_dev` structure is a descriptor for an IIO (Industrial
 * Input/Output) device that interfaces with the ADT7420 temperature
 * sensor. It contains pointers to both the ADT7420 device and the IIO
 * device structures, allowing for integration and communication between
 * the sensor and the IIO subsystem. Additionally, it maintains
 * information about the active channels and the number of active
 * channels, which are crucial for managing data acquisition and
 * processing in applications that utilize the ADT7420 sensor.
 *
 * @param adt7420_dev Pointer to an ADT7420 device structure.
 * @param iio_dev Pointer to an IIO device structure.
 * @param active_channels 32-bit integer representing the active channels.
 * @param no_active_channels 8-bit integer representing the number of active
 * channels.
 ******************************************************************************/
struct adt7420_iio_dev {
	struct adt7420_dev *adt7420_dev;
	struct iio_dev *iio_dev;
	uint32_t active_channels;
	uint8_t no_active_channels;
};

/***************************************************************************//**
 * @brief The `adt7420_iio_init_param` structure is a configuration structure
 * used to initialize an ADT7420 device in the context of an IIO
 * (Industrial I/O) driver. It contains a single member,
 * `adt7420_dev_init`, which is a pointer to another structure,
 * `adt7420_init_param`, that holds the initialization parameters
 * specific to the ADT7420 device. This structure is essential for
 * setting up the device with the correct parameters before it is used in
 * an IIO context.
 *
 * @param adt7420_dev_init A pointer to an adt7420_init_param structure, used
 * for initializing the ADT7420 device.
 ******************************************************************************/
struct adt7420_iio_init_param {
	struct adt7420_init_param *adt7420_dev_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function sets up an IIO device for the ADT7420 temperature sensor
 * by allocating necessary resources and initializing the sensor with the
 * provided parameters. It must be called before any operations are
 * performed on the ADT7420 sensor through the IIO interface. The
 * function expects valid initialization parameters and will return an
 * error code if initialization fails or if memory allocation is
 * unsuccessful. The caller is responsible for managing the memory of the
 * created device descriptor, including its eventual deallocation using
 * the appropriate removal function.
 *
 * @param iio_dev A pointer to a pointer where the initialized IIO device
 * descriptor will be stored. Must not be null. The caller takes
 * ownership of the allocated memory and is responsible for
 * freeing it.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the ADT7420 device. Must not be null and
 * should be properly initialized before calling this
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -ENOMEM for
 * memory allocation failure).
 ******************************************************************************/
int adt7420_iio_init(struct adt7420_iio_dev **iio_dev,
		     struct adt7420_iio_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly remove and free resources associated
 * with an ADT7420 IIO device. It should be called when the device is no
 * longer needed to ensure that all allocated resources are released.
 * This function must be called with a valid device descriptor that was
 * previously initialized. Failure to do so may result in undefined
 * behavior or memory leaks.
 *
 * @param desc A pointer to an adt7420_iio_dev structure representing the device
 * to be removed. This must not be null and should point to a valid,
 * initialized device descriptor. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns 0 on successful removal and deallocation of the device. If an
 * error occurs during the removal process, a non-zero error code is
 * returned.
 ******************************************************************************/
int adt7420_iio_remove(struct adt7420_iio_dev *desc);

#endif /** IIO_ADT7420_H **/
