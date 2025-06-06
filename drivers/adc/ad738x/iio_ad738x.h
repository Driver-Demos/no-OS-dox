/***************************************************************************//**
* @file   iio_ad738x.h
* @brief  Header file for AD738X IIO interface
********************************************************************************
* Copyright (c) 2024 Analog Devices, Inc.
* Copyright (c) 2024 BayLibre, SAS.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
*******************************************************************************/
#ifndef _AD738X_IIO_H_
#define _AD738X_IIO_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>

#include "iio.h"
#include "iio_types.h"
#include "ad738x.h"

/******************************************************************************/
/********************** Public/Extern Declarations ****************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad738x_iio_dev` structure is a compound data type that
 * encapsulates the necessary components for interfacing an AD738X device
 * with the Industrial I/O (IIO) subsystem. It contains pointers to both
 * the AD738X device instance and the IIO device interface, facilitating
 * the integration and management of the device within an IIO context.
 *
 * @param ad738x_dev A pointer to an ad738x_dev structure, representing the
 * AD738X device instance.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct ad738x_iio_dev {
	struct ad738x_dev *ad738x_dev;
	struct iio_device *iio_dev;
};

/* Init the IIO interface */
/***************************************************************************//**
 * @brief This function sets up the IIO interface for the AD738X device by
 * allocating necessary resources and initializing the device with the
 * provided parameters. It must be called before any other operations on
 * the AD738X IIO interface. The function expects valid initialization
 * parameters and will return an error code if initialization fails,
 * ensuring that resources are properly cleaned up in such cases.
 *
 * @param dev A pointer to a pointer where the initialized AD738X IIO device
 * structure will be stored. Must not be null. The caller takes
 * ownership of the allocated structure upon successful
 * initialization.
 * @param init_param A pointer to an ad738x_init_param structure containing the
 * initialization parameters for the AD738X device. Must not
 * be null and should be properly configured before calling
 * this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or device setup error.
 ******************************************************************************/
int ad738x_iio_init(struct ad738x_iio_dev **dev,
		    struct ad738x_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources
 * associated with an AD738X IIO device when it is no longer needed. It
 * is important to call this function to prevent resource leaks. The
 * function must be called with a valid device pointer that was
 * previously initialized using `ad738x_iio_init`. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to an `ad738x_iio_dev` structure representing the device
 * to be removed. Must not be null. If null, the function returns an
 * error code `-ENODEV`.
 * @return Returns 0 on successful removal of the device, or `-ENODEV` if the
 * device pointer is null.
 ******************************************************************************/
int ad738x_iio_remove(struct ad738x_iio_dev *dev);
#endif /* __AD738X_IIO_H__ */
