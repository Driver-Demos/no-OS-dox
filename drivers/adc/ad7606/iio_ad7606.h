/***************************************************************************//**
* @file   iio_ad7606.h
* @brief  Header file for AD7606 IIO interface
********************************************************************************
* Copyright (c) 2024 Analog Devices, Inc.
* Copyright (c) 2024 BayLibre, SAS.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*******************************************************************************/
#ifndef _AD7606_IIO_H_
#define _AD7606_IIO_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

#include "iio.h"
#include "iio_types.h"
#include "ad7606.h"

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7606_iio_dev` structure is a compound data type that
 * encapsulates the necessary components for interfacing an AD7606 device
 * with the Industrial I/O (IIO) subsystem. It includes a pointer to the
 * AD7606 device structure, a pointer to the IIO device structure, and an
 * integer to store the sign bit, which is crucial for interpreting the
 * data correctly. This structure is used to manage the interaction
 * between the AD7606 hardware and the IIO framework, facilitating data
 * acquisition and processing.
 *
 * @param ad7606_dev A pointer to an ad7606_dev structure, representing the
 * AD7606 device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 * @param sign_bit An integer representing the sign bit for the device data.
 ******************************************************************************/
struct ad7606_iio_dev {
	struct ad7606_dev *ad7606_dev;
	struct iio_device *iio_dev;
	int sign_bit;
};

/* Init the IIO interface */
/***************************************************************************//**
 * @brief This function initializes an AD7606 IIO interface device using the
 * provided initialization parameters. It allocates memory for the device
 * structure and sets up the device based on the resolution of the AD7606
 * device. This function must be called before any operations are
 * performed on the AD7606 IIO device. If the initialization fails, the
 * function returns an error code and no memory is allocated to the
 * device pointer.
 *
 * @param dev A pointer to a pointer of type `struct ad7606_iio_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure that this pointer is valid and will receive ownership of
 * the allocated memory upon successful initialization.
 * @param init_param A pointer to a `struct ad7606_init_param` containing the
 * initialization parameters for the AD7606 device. This must
 * be properly configured before calling the function. The
 * caller retains ownership of this parameter.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or device setup failure.
 ******************************************************************************/
int ad7606_iio_init(struct ad7606_iio_dev **dev,
		    struct ad7606_init_param *init_param);
/***************************************************************************//**
 * @brief This function is used to properly remove and deallocate resources
 * associated with an AD7606 IIO device. It should be called when the
 * device is no longer needed to ensure that all resources are freed and
 * no memory leaks occur. The function must be called with a valid device
 * pointer that was previously initialized using `ad7606_iio_init`. If
 * the provided device pointer is null, the function will return an error
 * code indicating that the device is not available.
 *
 * @param dev A pointer to an `ad7606_iio_dev` structure representing the device
 * to be removed. This pointer must not be null, and it should point
 * to a valid device initialized by `ad7606_iio_init`. If the pointer
 * is null, the function returns -ENODEV.
 * @return Returns 0 on successful removal and deallocation of the device.
 * Returns -ENODEV if the provided device pointer is null.
 ******************************************************************************/
int ad7606_iio_remove(struct ad7606_iio_dev *dev);

#endif /* __AD7606_IIO_H__ */
