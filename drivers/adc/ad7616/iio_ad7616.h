/***************************************************************************//**
* @file   iio_ad7616.h
* @brief  Header file for AD7616 IIO interface
********************************************************************************
* Copyright (c) 2024 Analog Devices, Inc.
* Copyright (c) 2024 BayLibre, SAS.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*******************************************************************************/
#ifndef _AD7616_IIO_H_
#define _AD7616_IIO_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

#include "iio.h"
#include "iio_types.h"
#include "ad7616.h"

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7616_iio_dev` structure is a compound data type that
 * encapsulates the relationship between an AD7616 device and its
 * corresponding IIO (Industrial I/O) device interface. It contains
 * pointers to both the AD7616 device structure and the IIO device
 * structure, facilitating the integration and management of the AD7616
 * device within an IIO framework. This structure is essential for
 * initializing and managing the IIO interface for the AD7616 device,
 * enabling data acquisition and processing in industrial applications.
 *
 * @param ad7616_dev A pointer to an instance of the ad7616_dev structure,
 * representing the AD7616 device.
 * @param iio_dev A pointer to an instance of the iio_device structure,
 * representing the IIO device interface.
 ******************************************************************************/
struct ad7616_iio_dev {
	struct ad7616_dev *ad7616_dev;
	struct iio_device *iio_dev;
};

/* Init the IIO interface */
/***************************************************************************//**
 * @brief This function initializes the IIO interface for the AD7616 device,
 * setting up necessary configurations and resources. It must be called
 * before any other operations on the AD7616 IIO interface. The function
 * allocates memory for the device structure and configures the device
 * according to the provided initialization parameters. If initialization
 * fails, it returns an error code and ensures no resources are leaked.
 *
 * @param dev A pointer to a pointer of type `struct ad7616_iio_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated structure upon successful initialization.
 * @param init_param A pointer to a `struct ad7616_init_param` containing the
 * initialization parameters for the AD7616 device. This must
 * be properly configured before calling the function and must
 * not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, and the `dev`
 * pointer will not be valid.
 ******************************************************************************/
int ad7616_iio_init(struct ad7616_iio_dev **dev,
		    struct ad7616_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly remove and free resources associated
 * with an AD7616 IIO device instance. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * released. This function must be called with a valid device instance
 * that was previously initialized using `ad7616_iio_init`. If the
 * provided device pointer is null, the function will return an error
 * code indicating that the device is not available.
 *
 * @param dev A pointer to the `ad7616_iio_dev` structure representing the
 * device instance to be removed. Must not be null. If null, the
 * function returns -ENODEV.
 * @return Returns 0 on successful removal and deallocation of the device.
 * Returns -ENODEV if the provided device pointer is null.
 ******************************************************************************/
int ad7616_iio_remove(struct ad7616_iio_dev *dev);

void iio_ad7616_event_handler(struct iio_desc *desc);

#endif /* __AD7616_IIO_H__ */
