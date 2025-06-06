/***************************************************************************//**
 *   @file   iio_adxl367.h
 *   @brief  Header file of IIO ADXL367 Driver.
 *   @author Andrei Porumb (andrei.porumb@analog.com)
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
#ifndef IIO_ADXL367_H
#define IIO_ADXL367_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `adxl367_iio_dev` structure is designed to encapsulate the
 * necessary components for interfacing an ADXL367 accelerometer device
 * with the Industrial I/O (IIO) subsystem. It includes pointers to both
 * the specific ADXL367 device and the IIO device interface, as well as
 * fields to track the active channels and their count, facilitating the
 * management and operation of the device within an IIO context.
 *
 * @param adxl367_dev A pointer to an ADXL367 device structure, representing the
 * specific device instance.
 * @param iio_dev A pointer to an IIO device structure, representing the IIO
 * interface for the device.
 * @param active_channels A 32-bit unsigned integer indicating which channels
 * are currently active.
 * @param no_of_active_channels An 8-bit unsigned integer representing the
 * number of active channels.
 ******************************************************************************/
struct adxl367_iio_dev {
	struct adxl367_dev *adxl367_dev;
	struct iio_device *iio_dev;
	uint32_t active_channels;
	uint8_t no_of_active_channels;
};

/***************************************************************************//**
 * @brief The `adxl367_iio_init_param` structure is used to encapsulate
 * initialization parameters specifically for setting up an ADXL367
 * device in an IIO (Industrial I/O) context. It contains a single
 * member, which is a pointer to another structure, `adxl367_init_param`,
 * that holds the necessary configuration details required to initialize
 * the ADXL367 device. This structure is typically used when initializing
 * the device to ensure that all necessary parameters are correctly set
 * up before the device is used in an application.
 *
 * @param adxl367_initial_param A pointer to an adxl367_init_param structure,
 * which holds initialization parameters for the
 * ADXL367 device.
 ******************************************************************************/
struct adxl367_iio_init_param {
	struct adxl367_init_param *adxl367_initial_param;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function sets up and initializes an IIO device for the ADXL367
 * sensor, preparing it for data acquisition. It must be called before
 * any operations are performed on the ADXL367 sensor through the IIO
 * interface. The function configures the sensor to enable temperature
 * reading, sets the output data rate to 400Hz, and switches the sensor
 * to measurement mode. It is essential to provide valid initialization
 * parameters, and the function will allocate necessary resources. If
 * initialization fails, it returns an error code and ensures that any
 * allocated resources are freed.
 *
 * @param iio_dev A pointer to a pointer where the initialized IIO device
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure upon successful
 * initialization.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the ADXL367 sensor. Must not be null and
 * should be properly initialized with valid parameters.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no resources are leaked.
 ******************************************************************************/
int adxl367_iio_init(struct adxl367_iio_dev **iio_dev,
		     struct adxl367_iio_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and free resources associated
 * with an ADXL367 IIO device instance. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * released. The function expects a valid device descriptor and will
 * return an error code if the removal process fails. It is important to
 * ensure that the descriptor is not used after this function is called,
 * as it will be deallocated.
 *
 * @param desc A pointer to an adxl367_iio_dev structure representing the device
 * instance to be removed. Must not be null. The function will
 * deallocate the memory associated with this descriptor.
 * @return Returns 0 on successful removal and deallocation, or a negative error
 * code if the removal process fails.
 ******************************************************************************/
int adxl367_iio_remove(struct adxl367_iio_dev *desc);

#endif /** IIO_ADXL367_H */
