/***************************************************************************//**
*   @file   iio_hmc630x.h
*   @brief  Header file of hmc6300 and hmc6301 driver extension for IIOD.
*   @author Darius Berghe (darius.berghe@analog.com)
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
#ifndef IIO_HMC630X_H
#define IIO_HMC630X_H

#include "iio.h"
#include "hmc630x.h"

/***************************************************************************//**
 * @brief The `hmc630x_iio_dev` structure is a compound data type that
 * encapsulates the relationship between an HMC630x device and its
 * corresponding IIO (Industrial I/O) device interface. It contains
 * pointers to both the underlying device structure (`hmc630x_dev`) and
 * the IIO device structure (`iio_device`), facilitating the integration
 * of the HMC630x device with the IIO subsystem for data acquisition and
 * control.
 *
 * @param dev A pointer to an hmc630x_dev structure, representing the underlying
 * device.
 * @param iio_dev A pointer to an iio_device structure, representing the IIO
 * device interface.
 ******************************************************************************/
struct hmc630x_iio_dev {
	struct hmc630x_dev *dev;
	struct iio_device *iio_dev;
};

/***************************************************************************//**
 * @brief The `hmc630x_iio_init_param` structure is used to encapsulate
 * initialization parameters for the HMC630x device within the IIO
 * (Industrial I/O) framework. It contains a single member, `ip`, which
 * is a pointer to another structure, `hmc630x_init_param`, that holds
 * the specific initialization settings required to configure the HMC630x
 * device. This structure is typically used during the initialization
 * process of the HMC630x IIO device to ensure that all necessary
 * parameters are correctly set up.
 *
 * @param ip A pointer to a `hmc630x_init_param` structure, which holds
 * initialization parameters for the HMC630x device.
 ******************************************************************************/
struct hmc630x_iio_init_param {
	struct hmc630x_init_param *ip;
};

/***************************************************************************//**
 * @brief The `hmc630x_iio_attr_id` is an enumeration that defines a set of
 * attribute identifiers for the HMC630x series devices, specifically the
 * HMC6300 and HMC6301. These identifiers are used to manage and
 * configure various aspects of the devices, such as enabling features,
 * controlling the VCO, and adjusting RF and baseband settings. Each
 * enumerator corresponds to a specific attribute that can be accessed or
 * modified through the IIO (Industrial I/O) interface, facilitating the
 * integration and control of these devices in a system.
 *
 * @param HMC630X_IIO_ATTR_ENABLED Represents the enabled state of the HMC630x
 * device.
 * @param HMC630X_IIO_ATTR_TEMP_EN Indicates whether temperature monitoring is
 * enabled.
 * @param HMC630X_IIO_ATTR_VCO Represents the Voltage Controlled Oscillator
 * (VCO) attribute.
 * @param HMC630X_IIO_ATTR_VCO_AVAILABLE Indicates the availability of the VCO.
 * @param HMC630X_IIO_ATTR_VCO_BAND Represents the VCO band setting.
 * @param HMC630X_IIO_ATTR_VCO_LOCK Indicates the lock status of the VCO.
 * @param HMC630X_IIO_ATTR_IF_ATTN Represents the Intermediate Frequency (IF)
 * attenuation setting.
 * @param HMC6300_IIO_ATTR_RF_ATTN Represents the RF attenuation setting for the
 * HMC6300 device.
 * @param HMC6301_IIO_ATTR_RF_LNA_GAIN Represents the RF Low Noise Amplifier
 * (LNA) gain setting for the HMC6301
 * device.
 * @param HMC6301_IIO_ATTR_BB_ATTN1 Represents the first baseband attenuation
 * setting for the HMC6301 device.
 * @param HMC6301_IIO_ATTR_BB_ATTN2 Represents the second baseband attenuation
 * setting for the HMC6301 device.
 * @param HMC6301_IIO_ATTR_BB_ATTNI_FINE Represents the fine adjustment for the
 * in-phase baseband attenuation for the
 * HMC6301 device.
 * @param HMC6301_IIO_ATTR_BB_ATTNQ_FINE Represents the fine adjustment for the
 * quadrature baseband attenuation for the
 * HMC6301 device.
 * @param HMC6301_IIO_ATTR_BB_LPC Represents the baseband low-pass filter
 * control for the HMC6301 device.
 * @param HMC6301_IIO_ATTR_BB_HPC Represents the baseband high-pass filter
 * control for the HMC6301 device.
 ******************************************************************************/
enum hmc630x_iio_attr_id {
	HMC630X_IIO_ATTR_ENABLED,
	HMC630X_IIO_ATTR_TEMP_EN,
	HMC630X_IIO_ATTR_VCO,
	HMC630X_IIO_ATTR_VCO_AVAILABLE,
	HMC630X_IIO_ATTR_VCO_BAND,
	HMC630X_IIO_ATTR_VCO_LOCK,
	HMC630X_IIO_ATTR_IF_ATTN,
	HMC6300_IIO_ATTR_RF_ATTN,
	HMC6301_IIO_ATTR_RF_LNA_GAIN,
	HMC6301_IIO_ATTR_BB_ATTN1,
	HMC6301_IIO_ATTR_BB_ATTN2,
	HMC6301_IIO_ATTR_BB_ATTNI_FINE,
	HMC6301_IIO_ATTR_BB_ATTNQ_FINE,
	HMC6301_IIO_ATTR_BB_LPC,
	HMC6301_IIO_ATTR_BB_HPC,
};

/***************************************************************************//**
 * @brief This function initializes an HMC630x IIO device structure, setting up
 * the necessary resources for interfacing with HMC6300 or HMC6301
 * devices through the Industrial I/O (IIO) subsystem. It must be called
 * before any operations are performed on the device. The function
 * allocates memory for the device structures and configures them based
 * on the initialization parameters provided. If the initialization
 * fails, it ensures that allocated resources are properly freed. This
 * function should be called once for each device instance that needs to
 * be managed.
 *
 * @param iiodev A pointer to a pointer where the initialized device structure
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated structure and is responsible for freeing it using
 * hmc630x_iio_remove.
 * @param init_param A pointer to an initialization parameter structure
 * containing the necessary configuration for the device. Must
 * not be null. The structure should be properly populated
 * before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures no memory leaks by freeing any
 * allocated resources.
 ******************************************************************************/
int32_t hmc630x_iio_init(struct hmc630x_iio_dev **iio_dev,
			 struct hmc630x_iio_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly remove and deallocate resources
 * associated with an HMC630x IIO device. It should be called when the
 * device is no longer needed to ensure that all associated resources are
 * freed. The function checks if the provided device descriptor is non-
 * null before proceeding with the removal. If the descriptor is null,
 * the function returns immediately with a success code. This function
 * also handles any errors that occur during the removal process by
 * returning the error code.
 *
 * @param iiodev A pointer to the hmc630x_iio_dev structure representing the
 * device to be removed. Must not be null unless the function is
 * expected to return immediately with success. The caller retains
 * ownership of the pointer, but the function will free the memory
 * associated with the device.
 * @return Returns 0 on successful removal and deallocation of the device. If an
 * error occurs during the removal process, a non-zero error code is
 * returned.
 ******************************************************************************/
int32_t hmc630x_iio_remove(struct hmc630x_iio_dev *desc);

#endif /** IIO_HMC630X_H */
