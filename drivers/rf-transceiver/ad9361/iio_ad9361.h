/***************************************************************************//**
*   @file   iio_ad9361.h
*   @brief  Header file of iio_ad9361
*   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
* Copyright 2019(c) Analog Devices, Inc.
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
#ifndef IIO_AD9361_H_
#define IIO_AD9361_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "iio.h"
#include "iio_types.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `iio_ad9361_init_param` structure is used to configure and
 * initialize an AD9361 device instance within the IIO framework. It
 * contains a single member, `ad9361_phy`, which is a pointer to an
 * `ad9361_rf_phy` structure. This pointer is essential for referencing
 * the specific AD9361 device instance that is being configured or
 * manipulated, allowing for the initialization and setup of the device's
 * parameters and operational state.
 *
 * @param ad9361_phy A pointer to an ad9361_rf_phy structure, representing the
 * AD9361 device instance.
 ******************************************************************************/
struct iio_ad9361_init_param {
	/** ad9361 device instance pointer */
	struct ad9361_rf_phy *ad9361_phy;
};

/***************************************************************************//**
 * @brief The `iio_ad9361_desc` structure is designed to encapsulate the IIO
 * device descriptor for the AD9361 device, which is a highly integrated
 * RF transceiver. This structure serves as a container for the
 * `iio_device` descriptor, facilitating the management and interaction
 * with the AD9361 device through the Industrial I/O (IIO) framework. It
 * is primarily used in conjunction with initialization and management
 * functions to handle the device's operations and configurations.
 *
 * @param dev_descriptor This member is an instance of the iio_device structure,
 * representing the IIO device descriptor.
 ******************************************************************************/
struct iio_ad9361_desc {
	/** iio device descriptor */
	struct iio_device dev_descriptor;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Init ad9361 iio. */
/***************************************************************************//**
 * @brief This function initializes an IIO device descriptor for the AD9361
 * device, setting up necessary structures and configurations. It must be
 * called before any operations are performed on the AD9361 device
 * through the IIO interface. The function allocates memory for the
 * descriptor and populates it with channel and attribute information. It
 * is essential to ensure that the `init` parameter is properly
 * configured before calling this function. If the initialization is
 * successful, the descriptor is returned through the `desc` parameter;
 * otherwise, an error code is returned.
 *
 * @param desc A pointer to a pointer of type `struct iio_ad9361_desc`. This is
 * an output parameter where the initialized descriptor will be
 * stored. The caller must ensure that this pointer is valid and
 * non-null.
 * @param init A pointer to an `iio_ad9361_init_param` structure containing
 * initialization parameters. This must be properly configured
 * before calling the function, and the caller retains ownership of
 * this structure.
 * @return Returns 0 on successful initialization, or -1 if memory allocation
 * fails.
 ******************************************************************************/
int32_t iio_ad9361_init(struct iio_ad9361_desc **desc,
			struct iio_ad9361_init_param *init);
/* Get desciptor. */
/***************************************************************************//**
 * @brief This function is used to obtain a reference to the IIO device
 * descriptor contained within an AD9361 descriptor. It is typically
 * called after the AD9361 descriptor has been initialized using
 * `iio_ad9361_init`. The function provides a way to access the
 * underlying IIO device descriptor for further operations. The caller
 * must ensure that the `desc` parameter is a valid, non-null pointer to
 * an initialized `iio_ad9361_desc` structure. The function does not
 * perform any validation on the input parameters.
 *
 * @param desc A pointer to an `iio_ad9361_desc` structure. This must be a
 * valid, non-null pointer to an initialized descriptor from which
 * the IIO device descriptor will be retrieved.
 * @param dev_descriptor A pointer to a pointer of type `struct iio_device`.
 * This must be a valid, non-null pointer where the
 * address of the IIO device descriptor will be stored.
 * @return None
 ******************************************************************************/
void iio_ad9361_get_dev_descriptor(struct iio_ad9361_desc *desc,
				   struct iio_device **dev_descriptor);
/* Free the resources allocated by iio_ad9361_init(). */
/***************************************************************************//**
 * @brief This function is used to release resources that were allocated during
 * the initialization of an AD9361 IIO device using iio_ad9361_init(). It
 * should be called when the device is no longer needed to ensure that
 * all allocated memory is properly freed. The function must be provided
 * with a valid descriptor that was previously initialized. If the
 * descriptor is null, the function will return an error code.
 *
 * @param desc A pointer to an iio_ad9361_desc structure that was previously
 * initialized by iio_ad9361_init(). Must not be null. If null, the
 * function returns -1 indicating an error.
 * @return Returns 0 on success, indicating that resources were successfully
 * freed. Returns -1 if the provided descriptor is null.
 ******************************************************************************/
int32_t iio_ad9361_remove(struct iio_ad9361_desc *desc);

#endif /* IIO_AD9361_H_ */
