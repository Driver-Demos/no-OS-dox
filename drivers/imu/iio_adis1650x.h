/***************************************************************************//**
 *   @file   iio_adis1650x.h
 *   @brief  Implementation of iio_adis1650x.h
 *   @author RBolboac (ramona.bolboaca@analog.com)
 *******************************************************************************
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
 ******************************************************************************/

#ifndef IIO_ADIS1650X_H
#define IIO_ADIS1650X_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_adis_internals.h"
#include "adis1650x.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes an ADIS1650x IIO device and prepares it for
 * use. It must be called before any operations are performed on the
 * device. The function allocates memory for the device descriptor and
 * initializes the device using the provided initialization parameters.
 * If the initialization is successful, the function returns 0 and
 * provides a pointer to the initialized device descriptor. If an error
 * occurs during initialization, the function returns a negative error
 * code and ensures that no resources are leaked.
 *
 * @param iio_dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller gains ownership
 * of the allocated descriptor upon successful initialization.
 * @param init_param A pointer to an adis_init_param structure containing the
 * initialization parameters for the device. Must not be null.
 * The structure should be properly populated with valid
 * device-specific parameters before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -ENOMEM if
 * memory allocation fails.
 ******************************************************************************/
int adis1650x_iio_init(struct adis_iio_dev **iio_dev,
		       struct adis_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly remove and free resources associated
 * with an ADIS1650x IIO device. It should be called when the device is
 * no longer needed to ensure that all allocated resources are released.
 * This function must be called with a valid device descriptor that was
 * previously initialized. If the provided descriptor is null, the
 * function will safely return without performing any operations.
 *
 * @param desc A pointer to the adis_iio_dev structure representing the device
 * to be removed. Must not be null if the device is to be removed;
 * otherwise, the function will return immediately without any
 * effect. The caller retains ownership of the pointer.
 * @return None
 ******************************************************************************/
void adis1650x_iio_remove(struct adis_iio_dev *desc);

#endif
