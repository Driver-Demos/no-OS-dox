/***************************************************************************//**
 *   @file   iio_adis1655x.h
 *   @brief  Implementation of iio_adis1655x.h
 *   @author RBolboac (ramona.gradinariu@analog.com)
 *******************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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

#ifndef IIO_ADIS1655X_H
#define IIO_ADIS1655X_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_adis_internals.h"
#include "adis1655x.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function initializes an ADIS1655x IIO device and prepares it for
 * use. It must be called before any operations are performed on the
 * device. The function allocates memory for the device descriptor and
 * initializes the device using the provided initialization parameters.
 * If the initialization is successful, the function returns a pointer to
 * the initialized device descriptor. If the initialization fails, it
 * returns an error code and ensures that no resources are leaked. This
 * function should be called once per device and the device must be
 * removed using the corresponding removal function when it is no longer
 * needed.
 *
 * @param iio_dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership
 * of the allocated descriptor and is responsible for freeing it
 * using the appropriate removal function.
 * @param init_param A pointer to a structure containing the initialization
 * parameters for the device. Must not be null. The structure
 * should be properly populated with valid device-specific
 * parameters before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure or device initialization failure.
 ******************************************************************************/
int adis1655x_iio_init(struct adis_iio_dev **iio_dev,
		       struct adis_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with an ADIS1655x IIO device instance. It should be called
 * when the device is no longer needed to ensure that all allocated
 * resources are freed. The function checks if the provided descriptor is
 * non-null before proceeding with the removal, making it safe to call
 * with a null pointer, in which case it will have no effect.
 *
 * @param desc A pointer to the adis_iio_dev structure representing the device
 * instance to be removed. This pointer must be valid and non-null
 * for the function to perform any action. If the pointer is null,
 * the function will return immediately without performing any
 * operations. The caller is responsible for ensuring that the
 * pointer is valid and that the device is no longer in use before
 * calling this function.
 * @return None
 ******************************************************************************/
void adis1655x_iio_remove(struct adis_iio_dev *desc);

#endif
