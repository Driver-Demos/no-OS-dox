/***************************************************************************//**
 *   @file   AD7091R.h
 *   @brief  Header file of AD7091R Driver.
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

#ifndef __AD7091R_H__
#define __AD7091R_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7091r_dev` structure is a simple data structure used to
 * encapsulate the SPI communication descriptor for the AD7091R device.
 * It contains a single member, `spi_desc`, which is a pointer to a
 * `no_os_spi_desc` structure, facilitating the management of SPI
 * communication settings and operations for the AD7091R analog-to-
 * digital converter.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for communication.
 ******************************************************************************/
struct ad7091r_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/***************************************************************************//**
 * @brief The `ad7091r_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the SPI
 * communication interface for the AD7091R device. It contains a single
 * member, `spi_init`, which is a structure of type
 * `no_os_spi_init_param` that holds the necessary configuration details
 * for initializing the SPI peripheral. This structure is essential for
 * establishing communication with the AD7091R device, allowing for its
 * configuration and operation.
 *
 * @param spi_init Holds the initialization parameters for the SPI communication
 * interface.
 ******************************************************************************/
struct ad7091r_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD7091R device by initializing the SPI
 * communication interface and performing a software reset to ensure the
 * device is in a known state. It must be called before any other
 * operations on the AD7091R device to ensure proper communication and
 * functionality. The function allocates memory for the device structure
 * and initializes the SPI interface using the provided parameters. If
 * the initialization is successful, the device pointer is updated to
 * point to the newly allocated device structure. The function returns a
 * status code indicating the success or failure of the initialization
 * process.
 *
 * @param device A double pointer to an ad7091r_dev structure. This must not be
 * null. On successful initialization, it will point to the newly
 * allocated and initialized device structure. The caller is
 * responsible for managing the memory of this structure,
 * including freeing it when no longer needed.
 * @param init_param A structure containing the initialization parameters for
 * the SPI interface. This includes the necessary
 * configuration for setting up the SPI communication. The
 * structure must be properly populated before calling this
 * function.
 * @return Returns an int8_t status code: 0 for success, or a negative value
 * indicating an error (e.g., memory allocation failure or SPI
 * initialization failure).
 ******************************************************************************/
int8_t ad7091r_init(struct ad7091r_dev **device,
		    struct ad7091r_init_param init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD5686
 * device that were allocated during initialization. It should be called
 * when the device is no longer needed to ensure proper cleanup and avoid
 * resource leaks. This function handles both SPI and I2C communication
 * peripherals, as well as any associated GPIOs. Ensure that the device
 * pointer provided is valid and was previously initialized.
 *
 * @param dev A pointer to an ad5686_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The function will handle invalid pointers
 * gracefully by not performing any operations.
 * @return Returns an int32_t indicating the success or failure of the resource
 * deallocation process. A return value of 0 indicates success, while a
 * non-zero value indicates an error occurred during the removal of one
 * or more resources.
 ******************************************************************************/
int32_t ad5686_remove(struct ad7091r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to perform a software reset on the AD7091R
 * device, which is necessary to restore the device to its default state.
 * It should be called when the device needs to be reinitialized or after
 * a configuration change that requires a reset. The function must be
 * called with a valid device structure that has been properly
 * initialized using the ad7091r_init function. This operation involves
 * communication over SPI, so the SPI interface must be correctly
 * configured and operational.
 *
 * @param dev A pointer to an ad7091r_dev structure representing the device.
 * This structure must be initialized and must not be null. The
 * function will not perform any action if the pointer is invalid.
 * @return None
 ******************************************************************************/
void ad7091r_software_reset(struct ad7091r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to perform a single conversion on the AD7091R
 * device and retrieve the resulting sample. It should be called when a
 * new sample is needed from the device. The function requires a valid
 * device structure that has been properly initialized with SPI
 * communication settings. It initiates a conversion and reads the
 * conversion result over SPI, returning the 12-bit sample. Ensure that
 * the device is powered up and not in power-down mode before calling
 * this function.
 *
 * @param dev A pointer to an ad7091r_dev structure representing the device.
 * This must be a valid, non-null pointer to a device that has been
 * initialized with ad7091r_init. The function will not perform any
 * checks on the validity of this pointer, so passing an invalid
 * pointer may result in undefined behavior.
 * @return Returns a 12-bit unsigned integer representing the conversion result
 * from the AD7091R device.
 ******************************************************************************/
uint16_t ad7091r_read_sample(struct ad7091r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to transition the AD7091R device into a low-
 * power state, effectively reducing its power consumption when it is not
 * actively being used. It should be called when the device is not needed
 * for conversions to conserve energy. The function must be called with a
 * valid device structure that has been properly initialized. It is
 * important to ensure that the device is not in the middle of a critical
 * operation before invoking this function to avoid unintended behavior.
 *
 * @param dev A pointer to an ad7091r_dev structure representing the device.
 * This structure must be initialized and must not be null. The
 * function assumes that the SPI descriptor within this structure is
 * valid and ready for communication.
 * @return None
 ******************************************************************************/
void ad7091r_power_down(struct ad7091r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to power up the AD7091R device by pulling the
 * CONVST signal high. It should be called when the device needs to be
 * brought out of a power-down state and made ready for operation. This
 * function assumes that the device has been properly initialized and
 * that the SPI communication is correctly set up. It is important to
 * ensure that the device is in a state where it can be powered up
 * safely, and that any necessary preconditions for powering up are met.
 *
 * @param dev A pointer to an ad7091r_dev structure representing the device to
 * be powered up. This pointer must not be null, and the structure
 * should be properly initialized with a valid SPI descriptor. If the
 * pointer is null or the structure is not properly initialized, the
 * behavior is undefined.
 * @return None
 ******************************************************************************/
void ad7091r_power_up(struct ad7091r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to convert a raw 12-bit ADC sample into a
 * voltage value based on a given reference voltage. It is typically
 * called after obtaining a raw sample from the ADC to interpret the
 * sample in terms of voltage. If the provided reference voltage is zero,
 * a default value of 2.5 volts is used. This function assumes the raw
 * sample is a signed 16-bit integer, but only the lower 12 bits are used
 * for conversion.
 *
 * @param raw_sample A signed 16-bit integer representing the raw ADC sample.
 * Only the lower 12 bits are used for conversion.
 * @param v_ref A floating-point number representing the reference voltage in
 * volts. If set to zero, a default of 2.5 volts is used.
 * @return Returns a floating-point number representing the converted voltage
 * value.
 ******************************************************************************/
float ad7091r_convert_to_volts(int16_t raw_sample, float v_ref);

#endif /* __AD7091R_H__ */
