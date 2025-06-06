/***************************************************************************//**
 *   @file   iio_adf5611.h
 *   @brief  Implementation of IIO ADF5611 Driver.
 *   @author josemene (jude.osemene@analog.com)
********************************************************************************
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
*******************************************************************************/
#ifndef IIO_ADF5611_H
#define IIO_ADF5611_H

#include "iio_types.h"
#include "iio.h"

/***************************************************************************//**
 * @brief The `adf5611_iio_dev` structure is a compound data type that
 * encapsulates the relationship between an ADF5611 device and its
 * corresponding IIO (Industrial I/O) device interface. It contains
 * pointers to both an `adf5611_dev` structure, which represents the
 * specific ADF5611 device, and an `iio_device` structure, which provides
 * the necessary interface for interacting with the device through the
 * IIO subsystem. This structure is essential for managing the
 * integration of the ADF5611 device within an IIO framework,
 * facilitating communication and control operations.
 *
 * @param adf5611_dev A pointer to an adf5611_dev structure, representing the
 * ADF5611 device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct adf5611_iio_dev {
	struct adf5611_dev *adf5611_dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `adf5611_iio_dev_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up an ADF5611 device in
 * an IIO (Industrial I/O) context. It contains a single member,
 * `adf5611_dev_init`, which is a pointer to another structure that holds
 * the specific initialization settings for the ADF5611 device. This
 * structure is essential for configuring the device before it is used in
 * an application.
 *
 * @param adf5611_dev_init A pointer to an adf5611_init_param structure, which
 * holds initialization parameters for the ADF5611
 * device.
 ******************************************************************************/
struct adf5611_iio_dev_init_param {
	struct adf5611_init_param *adf5611_dev_init;
};

/***************************************************************************//**
 * @brief The `adf5611_iio_ch_attr_id` is an enumeration that defines
 * identifiers for channel attributes specific to the ADF5611 IIO driver.
 * It includes attributes such as frequency and output power, which are
 * essential for configuring and managing the channel's operational
 * parameters.
 *
 * @param ADF5611_IIO_CH_ATTR_FREQ Represents the frequency attribute of the IIO
 * channel.
 * @param ADF5611_IIO_CH_ATTR_OPWR Represents the output power attribute of the
 * IIO channel.
 ******************************************************************************/
enum adf5611_iio_ch_attr_id {
	ADF5611_IIO_CH_ATTR_FREQ,
	ADF5611_IIO_CH_ATTR_OPWR,
};

/***************************************************************************//**
 * @brief The `adf5611_iio_dev_attr_id` is an enumeration that defines a set of
 * device attribute identifiers for the ADF5611 device within the IIO
 * (Industrial I/O) framework. These identifiers are used to specify
 * various configuration and status attributes related to the ADF5611
 * device, such as reference clock, reference divider, charge pump
 * current, and RF output divider settings. This enumeration facilitates
 * the management and manipulation of these attributes in a structured
 * and consistent manner within the driver implementation.
 *
 * @param ADF5611_IIO_DEV_ATTR_REF_CLK Represents the reference clock attribute
 * for the ADF5611 device.
 * @param ADF5611_IIO_DEV_ATTR_REF_DIV Represents the reference divider
 * attribute for the ADF5611 device.
 * @param ADF5611_IIO_DEV_ATTR_CP_I Represents the charge pump current attribute
 * for the ADF5611 device.
 * @param ADF5611_IIO_DEV_ATTR_CP_AVAIL Indicates the availability of the charge
 * pump attribute for the ADF5611 device.
 * @param ADF5611_IIO_DEV_ATTR_RFOUTDIV_PWR Represents the RF output divider
 * power attribute for the ADF5611
 * device.
 * @param ADF5611_IIO_DEV_ATTR_RFOUTDIV_DIV Represents the RF output divider
 * division attribute for the ADF5611
 * device.
 * @param ADF5611_IIO_DEV_ATTR_EN_RFOUTDIV Indicates whether the RF output
 * divider is enabled for the ADF5611
 * device.
 * @param ADF5611_IIO_DEV_ATTR_RFOUTDIV_DIV_AVAIL Indicates the availability of
 * the RF output divider division
 * attribute for the ADF5611
 * device.
 ******************************************************************************/
enum adf5611_iio_dev_attr_id {
	ADF5611_IIO_DEV_ATTR_REF_CLK,
	ADF5611_IIO_DEV_ATTR_REF_DIV,
	ADF5611_IIO_DEV_ATTR_CP_I,
	ADF5611_IIO_DEV_ATTR_CP_AVAIL,
	ADF5611_IIO_DEV_ATTR_RFOUTDIV_PWR,
	ADF5611_IIO_DEV_ATTR_RFOUTDIV_DIV,
	ADF5611_IIO_DEV_ATTR_EN_RFOUTDIV,
	ADF5611_IIO_DEV_ATTR_RFOUTDIV_DIV_AVAIL,
};

/***************************************************************************//**
 * @brief This function initializes an ADF5611 IIO device structure, preparing
 * it for use with the IIO subsystem. It must be called before any
 * operations are performed on the device. The function allocates memory
 * for the device structure and initializes it using the provided
 * initialization parameters. If the initialization fails, the function
 * returns an error code and ensures that no resources are leaked. It is
 * important to check the return value to ensure that the initialization
 * was successful before proceeding with further operations.
 *
 * @param iio_dev A pointer to a pointer where the initialized device structure
 * will be stored. Must not be null. The caller is responsible
 * for managing the memory of the structure after initialization.
 * @param init_param A pointer to a structure containing the initialization
 * parameters for the ADF5611 device. Must not be null. The
 * structure should be properly populated with valid
 * initialization data before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure.
 ******************************************************************************/
int adf5611_iio_init(struct adf5611_iio_dev **iio_dev,
		     struct adf5611_iio_dev_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly remove and free resources associated
 * with an ADF5611 IIO device instance. It should be called when the
 * device is no longer needed to ensure that all allocated resources are
 * released. The function first attempts to remove the underlying ADF5611
 * device and, if successful, proceeds to free the memory associated with
 * the IIO device structure. It is important to ensure that the device
 * instance is valid and has been initialized before calling this
 * function to avoid undefined behavior.
 *
 * @param desc A pointer to the adf5611_iio_dev structure representing the
 * device instance to be removed. Must not be null and should point
 * to a valid, initialized device instance. If the pointer is
 * invalid, the behavior is undefined.
 * @return Returns 0 on successful removal and deallocation of the device
 * instance. If an error occurs during the removal of the underlying
 * ADF5611 device, a non-zero error code is returned.
 ******************************************************************************/
int adf5611_iio_remove(struct adf5611_iio_dev *desc);

#endif
