/**************************************************************************//**
*   @file   ad3552r.h
*   @brief  IIO Header file of ad3552r Driver
*   @author Mihail Chindris (Mihail.Chindris@analog.com)
*
*******************************************************************************
* Copyright 2021(c) Analog Devices, Inc.
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

#ifndef IIO_AD3552R_H
#define IIO_AD3552R_H

#include "ad3552r.h"
#include "iio_types.h"

struct iio_ad3552r_desc;

/***************************************************************************//**
 * @brief This function initializes an IIO AD3552R descriptor using the provided
 * initialization parameters. It must be called before any operations are
 * performed on the AD3552R device through the IIO interface. The
 * function allocates memory for the descriptor and sets up the necessary
 * configuration based on the provided parameters. It is essential to
 * ensure that both the descriptor pointer and the initialization
 * parameters are valid and non-null before calling this function. If the
 * initialization fails, an error code is returned, and the descriptor is
 * not allocated.
 *
 * @param iio_dac A pointer to a pointer where the initialized IIO AD3552R
 * descriptor will be stored. Must not be null. The caller is
 * responsible for managing the memory of the descriptor after
 * initialization.
 * @param param A pointer to an ad3552r_init_param structure containing the
 * initialization parameters for the AD3552R device. Must not be
 * null. The structure should be properly populated with valid
 * configuration data before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid arguments, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int32_t iio_ad3552r_init(struct iio_ad3552r_desc **iio_dac,
			 struct ad3552r_init_param *param);

/***************************************************************************//**
 * @brief Use this function to properly remove and deallocate resources
 * associated with an IIO AD3552R descriptor when it is no longer needed.
 * This function should be called to clean up after using the descriptor
 * to prevent memory leaks. It is important to ensure that the descriptor
 * is valid and initialized before calling this function. If the provided
 * descriptor is null, the function will return an error code indicating
 * invalid input.
 *
 * @param iio_dac A pointer to the IIO AD3552R descriptor to be removed. Must
 * not be null. The caller retains ownership of the pointer, but
 * the function will deallocate the resources associated with the
 * descriptor. If null, the function returns -EINVAL.
 * @return Returns 0 on successful removal and deallocation of the descriptor,
 * or -EINVAL if the input descriptor is null.
 ******************************************************************************/
int32_t iio_ad3552r_remove(struct iio_ad3552r_desc *iio_dac);

/***************************************************************************//**
 * @brief Use this function to obtain the IIO device descriptor associated with
 * a given AD3552R descriptor. This function is typically called after
 * the AD3552R descriptor has been initialized. It requires valid
 * pointers for both the AD3552R descriptor and the output IIO device
 * descriptor. If either pointer is null, the function will return
 * without performing any action.
 *
 * @param iio_dac A pointer to an initialized `iio_ad3552r_desc` structure. Must
 * not be null. The caller retains ownership.
 * @param desc A pointer to a pointer of type `iio_device`. Must not be null. On
 * successful execution, this will point to the IIO device
 * descriptor associated with the provided AD3552R descriptor.
 * @return None
 ******************************************************************************/
void iio_ad3552r_get_descriptor(struct iio_ad3552r_desc *iio_dac,
				struct iio_device **desc);

#endif /* IIO_AD3552R_H */
