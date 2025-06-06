/***************************************************************************//**
 *   @file   AD799x.h
 *   @brief  Header file of AD799x Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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

#ifndef __AD799X_H__
#define __AD799X_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_i2c.h"

/******************************************************************************/
/******************************** AD799x **************************************/
/******************************************************************************/

/* Configuration Register definition. */
#define AD799X_CHANNEL(ch)        ((1 << ch) << 4)
#define AD799X_REF_SEL	          (1 << 3)
#define AD799X_FLTR		  (1 << 2)
#define AD799X_BIT_TRIAL_DELAY	  (1 << 1)
#define AD799X_SAMPLE_DELAY	  (1 << 0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad799x_type` enumeration defines the supported device types for
 * the AD799x series of analog-to-digital converters. Each enumerator
 * corresponds to a specific model within the series, allowing the
 * software to handle different device configurations and capabilities
 * appropriately.
 *
 * @param AD7991 Represents the AD7991 device type.
 * @param AD7995 Represents the AD7995 device type.
 * @param AD7999 Represents the AD7999 device type.
 ******************************************************************************/
enum ad799x_type {
	AD7991,
	AD7995,
	AD7999
};

/***************************************************************************//**
 * @brief The `ad799x_dev` structure is designed to encapsulate the necessary
 * components for interfacing with AD799x series devices over I2C. It
 * includes a pointer to an I2C descriptor for managing communication and
 * a configuration setting that specifies the number of bits used in the
 * device's operation. This structure is essential for initializing and
 * managing the AD799x devices, allowing for configuration and data
 * retrieval through I2C communication.
 *
 * @param i2c_desc A pointer to a no_os_i2c_desc structure, representing the I2C
 * descriptor for communication.
 * @param bits_number An 8-bit unsigned integer representing the number of bits
 * used in the device settings.
 ******************************************************************************/
struct ad799x_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* Device Settings */
	uint8_t         bits_number;
};

/***************************************************************************//**
 * @brief The `ad799x_init_param` structure is used to encapsulate the
 * initialization parameters required to set up an AD799x device. It
 * includes the I2C initialization parameters and the specific part
 * number of the AD799x device, allowing for flexible configuration and
 * initialization of different models within the AD799x series.
 *
 * @param i2c_init This member is a structure that holds the initialization
 * parameters for the I2C interface.
 * @param part_number This member is an enumeration that specifies the type of
 * AD799x device being initialized.
 ******************************************************************************/
struct ad799x_init_param {
	/* I2C */
	struct no_os_i2c_init_param		i2c_init;
	/* Device Settings */
	enum ad799x_type	part_number;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD799x device by allocating necessary
 * resources and initializing the I2C interface based on the provided
 * initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * determines the number of bits available for conversion based on the
 * specified part number. If memory allocation fails or the I2C
 * initialization is unsuccessful, the function returns an error code.
 *
 * @param device A pointer to a pointer of type `struct ad799x_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory
 * using `ad799x_remove`.
 * @param init_param A structure of type `struct ad799x_init_param` containing
 * initialization parameters. This includes I2C initialization
 * parameters and the part number of the AD799x device. The
 * part number determines the conversion bit width.
 * @return Returns 0 on successful initialization, or -1 if memory allocation
 * fails or I2C initialization is unsuccessful.
 ******************************************************************************/
int8_t ad799x_init(struct ad799x_dev **device,
		   struct ad799x_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD799x
 * device instance. It should be called when the device is no longer
 * needed, typically at the end of its lifecycle, to ensure proper
 * cleanup and avoid memory leaks. This function must be called only
 * after a successful call to ad799x_init().
 *
 * @param dev A pointer to an ad799x_dev structure representing the device
 * instance to be removed. Must not be null. The function will handle
 * invalid input by freeing the resources and returning the result of
 * the I2C removal operation.
 * @return Returns an int32_t indicating the success or failure of the I2C
 * removal operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad799x_remove(struct ad799x_dev *dev);

/***************************************************************************//**
 * @brief Use this function to configure the AD799x device by writing a
 * specified value to its configuration register. This function should be
 * called after the device has been initialized using `ad799x_init`. It
 * is essential for setting up the device's operational parameters, such
 * as channel selection and reference voltage settings. Ensure that the
 * `dev` parameter is a valid, initialized device structure before
 * calling this function.
 *
 * @param dev A pointer to an `ad799x_dev` structure representing the device.
 * This must be a valid, non-null pointer to a device that has been
 * initialized with `ad799x_init`. The caller retains ownership.
 * @param register_value A `uint8_t` value representing the configuration
 * settings to be written to the device's configuration
 * register. This value should be constructed using the
 * appropriate configuration macros (e.g.,
 * `AD799X_CHANNEL`, `AD799X_REF_SEL`).
 * @return None
 ******************************************************************************/
void ad799x_set_configuration_reg(struct ad799x_dev *dev,
				  uint8_t register_value);

/***************************************************************************//**
 * @brief Use this function to obtain the latest conversion result from an
 * AD799x device. It reads the conversion data over I2C and extracts both
 * the conversion value and the channel number. This function should be
 * called after the device has been properly initialized and configured.
 * The conversion result is provided as a 12-bit value, adjusted
 * according to the device's bit resolution setting. Ensure that the
 * pointers provided for the conversion value and channel are valid and
 * non-null.
 *
 * @param dev A pointer to an initialized ad799x_dev structure representing the
 * device. Must not be null.
 * @param conv_value A pointer to an int16_t where the conversion result will be
 * stored. Must not be null.
 * @param channel A pointer to an int8_t where the channel number will be
 * stored. Must not be null.
 * @return None
 ******************************************************************************/
void ad799x_get_conversion_result(struct ad799x_dev *dev,
				  int16_t* conv_value,
				  int8_t* channel);

/***************************************************************************//**
 * @brief This function is used to convert a raw sample obtained from an AD799x
 * ADC device into a voltage value based on the provided reference
 * voltage. It is typically called after obtaining a raw sample from the
 * ADC to interpret the result in terms of voltage. The function requires
 * a valid device structure that specifies the resolution of the ADC in
 * bits. Ensure that the reference voltage is correctly specified to
 * match the ADC's configuration for accurate conversion.
 *
 * @param dev A pointer to an ad799x_dev structure representing the ADC device.
 * This must be initialized and must not be null. The structure
 * should contain the correct number of bits for the ADC resolution.
 * @param raw_sample An int16_t representing the raw sample value obtained from
 * the ADC. It should be within the range that the ADC can
 * produce, based on its resolution.
 * @param v_ref A float representing the reference voltage used by the ADC. This
 * should be a positive value and should match the reference
 * voltage used during the ADC's operation.
 * @return Returns a float representing the converted voltage value.
 ******************************************************************************/
float ad799x_convert_to_volts(struct ad799x_dev *dev,
			      int16_t raw_sample,
			      float v_ref);

#endif /* __AD799X_H__ */
