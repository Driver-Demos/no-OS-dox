/***************************************************************************//**
 *   @file   AD7980.h
 *   @brief  Header file of AD7980 Driver.
 *   @author Bancisor Mihai(Bancisor.Mihai@analog.com)
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
#ifndef __AD7980_H__
#define __AD7980_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/******************************** AD7980 **************************************/
/******************************************************************************/
/* AD74XX Chip Select Pin declaration */
#define AD7980_CS_LOW           no_os_gpio_set_value(dev->gpio_cs,  \
			        NO_OS_GPIO_LOW)
#define AD7980_CS_HIGH          no_os_gpio_set_value(dev->gpio_cs,  \
			        NO_OS_GPIO_HIGH)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad7980_dev` structure is designed to encapsulate the necessary
 * components for interfacing with the AD7980 device, specifically
 * managing the SPI communication and GPIO control for chip select. It
 * contains pointers to SPI and GPIO descriptors, which are essential for
 * the initialization and operation of the AD7980, a precision analog-to-
 * digital converter. This structure is used in conjunction with
 * initialization parameters and functions to facilitate communication
 * and data conversion processes.
 *
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 * @param gpio_cs A pointer to a GPIO descriptor used for chip select control.
 ******************************************************************************/
struct ad7980_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_cs;
};

/***************************************************************************//**
 * @brief The `ad7980_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD7980 device.
 * It includes configuration details for both the SPI interface and the
 * GPIO chip select, which are essential for establishing communication
 * with the AD7980 analog-to-digital converter. This structure is
 * typically used during the initialization process to ensure that the
 * device is correctly configured before any data conversion operations
 * are performed.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_cs Holds the initialization parameters for the GPIO chip select.
 ******************************************************************************/
struct ad7980_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_cs;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD7980 device by allocating necessary
 * resources and initializing the SPI and GPIO communication peripherals
 * based on the provided initialization parameters. It must be called
 * before any other operations on the AD7980 device to ensure that the
 * device is properly configured and ready for use. The function returns
 * a status code indicating success or failure of the initialization
 * process. If the function fails, the device pointer will not be valid,
 * and the caller should not attempt to use it.
 *
 * @param device A pointer to a pointer of type `struct ad7980_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure upon successful initialization.
 * @param init_param A structure of type `struct ad7980_init_param` containing
 * the initialization parameters for the SPI and GPIO
 * peripherals. This structure must be properly populated with
 * valid initialization data before calling the function.
 * @return Returns an int8_t status code: 0 for success, or a negative value
 * indicating an error during initialization. On success, the `device`
 * pointer is set to point to a newly allocated and initialized
 * `ad7980_dev` structure.
 ******************************************************************************/
int8_t ad7980_init(struct ad7980_dev **device,
		   struct ad7980_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to release all resources associated with an
 * AD7980 device that were previously allocated during initialization. It
 * should be called when the device is no longer needed to ensure proper
 * cleanup and to prevent resource leaks. The function handles the
 * deallocation of SPI and GPIO resources and frees the memory allocated
 * for the device structure. It is important to ensure that the device
 * pointer is valid and was successfully initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad7980_dev structure representing the device to be
 * removed. This pointer must not be null and should point to a
 * valid, initialized device structure. If the pointer is invalid,
 * the behavior is undefined.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource deallocation. A return value of 0 indicates success, while a
 * non-zero value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t ad7980_remove(struct ad7980_dev *dev);

/***************************************************************************//**
 * @brief This function is used to start an analog-to-digital conversion on the
 * AD7980 device and retrieve the resulting digital value. It must be
 * called after the device has been properly initialized using
 * `ad7980_init`. The function handles the necessary SPI communication to
 * trigger the conversion and read the result. It is important to ensure
 * that the device structure is valid and properly configured before
 * calling this function. The function returns the conversion result as a
 * 16-bit unsigned integer.
 *
 * @param dev A pointer to an `ad7980_dev` structure representing the device.
 * This structure must be initialized and configured before calling
 * this function. The pointer must not be null, and the function
 * assumes ownership of the SPI and GPIO descriptors within the
 * structure.
 * @return Returns a 16-bit unsigned integer representing the conversion result
 * from the AD7980 device.
 ******************************************************************************/
uint16_t ad7980_conversion(struct ad7980_dev *dev);

/***************************************************************************//**
 * @brief This function is used to convert a raw 16-bit sample from the AD7980
 * ADC into a voltage value based on a given reference voltage. It is
 * typically called after obtaining a raw sample from the ADC to
 * interpret the sample in terms of actual voltage. The function assumes
 * a linear relationship between the raw sample and the voltage, scaled
 * by the reference voltage. It is important to ensure that the reference
 * voltage provided is accurate and corresponds to the voltage used
 * during the ADC conversion process.
 *
 * @param raw_sample A 16-bit unsigned integer representing the raw sample
 * obtained from the AD7980 ADC. It should be within the range
 * of 0 to 65535, as it represents the full scale of the ADC
 * output.
 * @param v_ref A floating-point number representing the reference voltage used
 * during the ADC conversion. This value should be positive and
 * accurately reflect the reference voltage applied to the ADC to
 * ensure correct conversion to volts.
 * @return Returns a floating-point number representing the voltage
 * corresponding to the raw ADC sample, scaled by the reference voltage.
 ******************************************************************************/
float ad7980_convert_to_volts(uint16_t raw_sample, float v_ref);

#endif /* __AD7980_H__ */
