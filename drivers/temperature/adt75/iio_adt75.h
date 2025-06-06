/***************************************************************************//**
 *   @file   iio_adt75.h
 *   @brief  Header file of the IIO ADT75 Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
#ifndef IIO_ADT75_H
#define IIO_ADT75_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "iio.h"
#include "adt75.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `adt75_iio_desc` structure is designed to encapsulate the state
 * specific to the IIO (Industrial I/O) interface for an ADT75
 * temperature sensor device. It includes a reference to the ADT75 device
 * descriptor, a reference to the IIO device interface, and a field to
 * track the number of active channels, facilitating the integration of
 * the ADT75 sensor with the IIO subsystem.
 *
 * @param adt75_desc A pointer to an adt75_desc structure, representing the
 * ADT75 device descriptor.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 * @param active_channels A 32-bit unsigned integer indicating the number of
 * active channels.
 ******************************************************************************/
struct adt75_iio_desc {
	struct adt75_desc *adt75_desc;
	struct iio_device *iio_dev;
	uint32_t active_channels;
};

/***************************************************************************//**
 * @brief The `adt75_iio_init_param` structure is designed to encapsulate
 * initialization parameters specifically for the IIO (Industrial
 * Input/Output) interface of the ADT75 temperature sensor. It contains a
 * single member, which is a pointer to another structure,
 * `adt75_init_param`, that holds the necessary configuration settings
 * required to initialize the ADT75 device. This structure is used during
 * the initialization process to ensure that the ADT75 device is set up
 * correctly for IIO operations.
 *
 * @param adt75_init_param A pointer to a structure of type adt75_init_param,
 * which holds initialization parameters for the ADT75
 * device.
 ******************************************************************************/
struct adt75_iio_init_param {
	struct adt75_init_param *adt75_init_param;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up an IIO descriptor for the ADT75 device,
 * preparing it for use in IIO operations. It must be called before any
 * IIO operations are performed on the ADT75 device. The function
 * allocates memory for the descriptor and initializes it using the
 * provided initialization parameters. If the initialization fails, the
 * function ensures that any allocated resources are properly freed. It
 * is important to check the return value to ensure that the
 * initialization was successful before proceeding with further
 * operations.
 *
 * @param desc A pointer to a pointer where the initialized descriptor will be
 * stored. Must not be null. The caller takes ownership of the
 * allocated descriptor and is responsible for freeing it using
 * `adt75_iio_remove`.
 * @param init_param A pointer to an `adt75_iio_init_param` structure containing
 * the initialization parameters for the ADT75 device. Must
 * not be null. The structure should be properly populated
 * before calling this function.
 * @return Returns 0 on successful initialization. On failure, a negative error
 * code is returned, and the descriptor is not allocated.
 ******************************************************************************/
int adt75_iio_init(struct adt75_iio_desc **, struct adt75_iio_init_param *);

/***************************************************************************//**
 * @brief Use this function to release resources associated with an ADT75 IIO
 * descriptor when it is no longer needed. This function should be called
 * to clean up after the descriptor has been initialized and used,
 * ensuring that all associated resources are properly freed. It is
 * important to ensure that the descriptor is valid and has been
 * previously initialized before calling this function to avoid undefined
 * behavior.
 *
 * @param desc A pointer to the adt75_iio_desc structure to be freed. This must
 * be a valid, non-null pointer to a descriptor that was previously
 * initialized. The function will handle invalid or null pointers by
 * returning an error code.
 * @return Returns 0 on success, or a negative error code if the removal process
 * fails.
 ******************************************************************************/
int adt75_iio_remove(struct adt75_iio_desc *);

#endif /** IIO_ADT75_H */
