/***************************************************************************//**
 *   @file   iio_adis1657x.h
 *   @brief  Implementation of iio_adis1657x.h
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

#ifndef IIO_ADIS1657X_H
#define IIO_ADIS1657X_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "iio_adis_internals.h"
#include "adis1657x.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up an ADIS1657x IIO device for use, allocating
 * necessary resources and initializing the device with the provided
 * parameters. It must be called before any operations are performed on
 * the device. The function requires valid initialization parameters and
 * a hardware trigger descriptor. If initialization fails, it returns an
 * error code and ensures no resources are leaked.
 *
 * @param iio_dev A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership
 * of the allocated descriptor upon successful initialization.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the device. Must not be null and should be
 * properly populated with valid data.
 * @param adis1657x_trig_desc A pointer to a hardware trigger descriptor. Can be
 * null if no hardware trigger is used.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or device initialization error.
 ******************************************************************************/
int adis1657x_iio_init(struct adis_iio_dev **iio_dev,
		       struct adis_init_param *init_param,
		       struct iio_hw_trig *adis1657x_trig_desc);

/***************************************************************************//**
 * @brief This function is used to properly remove and free resources associated
 * with an ADIS1657x IIO device. It should be called when the device is
 * no longer needed to ensure that all allocated resources are released.
 * The function checks if the provided device descriptor is non-null
 * before proceeding with the removal and deallocation process, making it
 * safe to call with a null pointer.
 *
 * @param desc A pointer to the adis_iio_dev structure representing the device
 * to be removed. Must not be null; if it is null, the function will
 * return immediately without performing any action. The caller
 * retains ownership of the pointer, but the resources it points to
 * will be deallocated.
 * @return None
 ******************************************************************************/
void adis1657x_iio_remove(struct adis_iio_dev *desc);

#endif
