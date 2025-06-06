/***************************************************************************//**
 *   @file   AD74xx.h
 *   @brief  Header file of AD74xx Driver.
 *   @author DNechita(Dan.Nechita@analog.com)
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

#ifndef __AD74XX_H__
#define __AD74XX_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/******************************** AD74XX **************************************/
/******************************************************************************/

/* AD74XX Chip Select Pin declaration */
#define AD74XX_CS_LOW           no_os_gpio_set_value(dev->gpio_cs,  \
			        NO_OS_GPIO_LOW)
#define AD74XX_CS_HIGH          no_os_gpio_set_value(dev->gpio_cs,  \
			        NO_OS_GPIO_HIGH)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Supported devices */
/***************************************************************************//**
 * @brief The `ad74xx_type` enumeration defines a set of constants representing
 * different models of the AD74xx series of devices, each associated with
 * a specific integer value that likely corresponds to the number of bits
 * or another characteristic of the device. This enumeration is used to
 * identify and differentiate between various supported devices in the
 * AD74xx series, facilitating device-specific operations in the driver
 * code.
 *
 * @param ID_AD7466 Represents the AD7466 device with a value of 12.
 * @param ID_AD7467 Represents the AD7467 device with a value of 10.
 * @param ID_AD7468 Represents the AD7468 device with a value of 8.
 * @param ID_AD7475 Represents the AD7475 device with a value of 12.
 * @param ID_AD7476 Represents the AD7476 device with a value of 12.
 * @param ID_AD7476A Represents the AD7476A device with a value of 12.
 * @param ID_AD7477 Represents the AD7477 device with a value of 10.
 * @param ID_AD7477A Represents the AD7477A device with a value of 10.
 * @param ID_AD7478 Represents the AD7478 device with a value of 8.
 * @param ID_AD7478A Represents the AD7478A device with a value of 8.
 * @param ID_AD7495 Represents the AD7495 device with a value of 12.
 ******************************************************************************/
enum ad74xx_type {
	ID_AD7466  = 12,
	ID_AD7467  = 10,
	ID_AD7468  = 8,
	ID_AD7475  = 12,
	ID_AD7476  = 12,
	ID_AD7476A = 12,
	ID_AD7477  = 10,
	ID_AD7477A = 10,
	ID_AD7478  = 8,
	ID_AD7478A = 8,
	ID_AD7495  = 12
};

/***************************************************************************//**
 * @brief The `ad74xx_dev` structure is designed to encapsulate the necessary
 * components for interfacing with AD74xx series devices, which are
 * analog-to-digital converters. It includes pointers to SPI and GPIO
 * descriptors for communication and control, as well as fields for
 * device-specific settings such as the number of bits in the data output
 * and the specific device model being used. This structure is essential
 * for initializing and managing the device's operation within a software
 * application.
 *
 * @param spi_desc Pointer to a SPI descriptor for managing SPI communication.
 * @param gpio_cs Pointer to a GPIO descriptor for managing the chip select
 * line.
 * @param device_bits_number Specifies the number of bits for the device's data
 * output.
 * @param part_number Enumerated type indicating the specific AD74xx device
 * model.
 ******************************************************************************/
struct ad74xx_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_cs;
	/* Device Settings */
	int8_t			device_bits_number;
	enum ad74xx_type	part_number;
};

/***************************************************************************//**
 * @brief The `ad74xx_init_param` structure is used to encapsulate the
 * initialization parameters required to set up an AD74xx device. It
 * includes configuration details for the SPI interface and GPIO chip
 * select, as well as device-specific settings such as the number of bits
 * and the part number. This structure is essential for initializing the
 * device correctly before use.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param gpio_cs Contains the initialization parameters for the GPIO chip
 * select.
 * @param device_bits_number Specifies the number of bits for the device.
 * @param part_number Indicates the specific part number of the AD74xx device.
 ******************************************************************************/
struct ad74xx_init_param {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* GPIO */
	struct no_os_gpio_init_param	gpio_cs;
	/* Device Settings */
	int8_t			device_bits_number;
	enum ad74xx_type	part_number;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up an AD74xx device by allocating necessary
 * resources and configuring SPI and GPIO interfaces based on the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure proper setup. The function returns
 * a status code indicating success or failure of the initialization
 * process. If the function fails, the device pointer will not be valid,
 * and the caller should not attempt to use it.
 *
 * @param device A pointer to a pointer of type `struct ad74xx_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A structure of type `struct ad74xx_init_param` containing
 * initialization parameters for the SPI and GPIO interfaces,
 * as well as device-specific settings like the number of bits
 * and part number. All fields must be properly initialized
 * before calling this function.
 * @return Returns an int8_t status code: 0 for success, or a negative value
 * indicating an error during initialization.
 ******************************************************************************/
int8_t ad74xx_init(struct ad74xx_dev **device,
		   struct ad74xx_init_param init_param);

/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD74xx device after it is no longer needed. It
 * ensures that the SPI and GPIO resources are freed and the memory
 * allocated for the device structure is deallocated. This function must
 * be called after the device has been initialized and used, to prevent
 * resource leaks. It is important to ensure that the device pointer is
 * valid and was previously initialized by `ad74xx_init`.
 *
 * @param dev A pointer to an `ad74xx_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid device structure that was initialized using `ad74xx_init`.
 * If the pointer is invalid, the behavior is undefined.
 * @return Returns an integer status code indicating the success or failure of
 * the resource deallocation process. A non-zero value indicates an
 * error occurred during the removal of resources.
 ******************************************************************************/
int32_t ad74xx_remove(struct ad74xx_dev *dev);

/***************************************************************************//**
 * @brief Use this function to power down the AD74xx device when it is not in
 * use to conserve power. This function should be called when the device
 * is initialized and operational, and it is safe to power it down. It
 * performs a sequence of SPI operations to ensure the device enters a
 * low-power state. Ensure that the device is not in the middle of a
 * critical operation before calling this function.
 *
 * @param dev A pointer to an initialized ad74xx_dev structure representing the
 * device to be powered down. Must not be null. The caller retains
 * ownership of the structure.
 * @return None
 ******************************************************************************/
void ad74xx_power_down(struct ad74xx_dev *dev);

/***************************************************************************//**
 * @brief This function is used to power up an AD74xx device by executing a
 * dummy SPI read operation. It should be called when the device needs to
 * be brought out of a low-power state or initialized for operation. The
 * function requires a valid device structure that has been properly
 * initialized. It does not return any value or modify the input
 * structure, but it is essential for ensuring the device is ready for
 * subsequent operations.
 *
 * @param dev A pointer to an ad74xx_dev structure representing the device to be
 * powered up. This structure must be initialized and must not be
 * null. The function assumes that the SPI descriptor within the
 * structure is valid and ready for communication.
 * @return None
 ******************************************************************************/
void ad74xx_power_up(struct ad74xx_dev *dev);

/***************************************************************************//**
 * @brief This function retrieves the conversion result from an AD74xx device by
 * performing an SPI read operation. It should be called when a
 * conversion result is needed from the device. The function requires a
 * valid device structure that has been properly initialized. The
 * conversion result is adjusted based on the device's bit resolution,
 * which is specified in the device structure. Ensure that the device is
 * powered up before calling this function to obtain valid data.
 *
 * @param dev A pointer to an ad74xx_dev structure representing the device. This
 * structure must be initialized and must not be null. The
 * device_bits_number field within this structure determines the bit
 * resolution of the conversion result.
 * @return Returns a 16-bit unsigned integer representing the conversion result
 * from the device, adjusted according to the device's bit resolution.
 ******************************************************************************/
uint16_t ad74xx_get_register_value(struct ad74xx_dev *dev);

/***************************************************************************//**
 * @brief This function is used to convert a raw analog-to-digital converter
 * (ADC) sample into a corresponding voltage value based on the reference
 * voltage and the resolution of the ADC device. It is typically called
 * after obtaining a raw sample from the ADC to interpret the sample in
 * terms of voltage. The function requires a valid device structure that
 * contains the ADC's resolution information. Ensure that the device has
 * been properly initialized before calling this function.
 *
 * @param dev A pointer to an ad74xx_dev structure representing the ADC device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param raw_value The raw ADC sample value to be converted. It should be
 * within the range supported by the ADC's resolution.
 * @param v_ref The reference voltage used by the ADC for conversion. This
 * should be a positive floating-point value representing the
 * voltage reference.
 * @return Returns the calculated voltage as a floating-point value,
 * representing the equivalent voltage of the raw ADC sample.
 ******************************************************************************/
float ad74xx_convert_to_volts(struct ad74xx_dev *dev,
			      uint16_t raw_value,
			      float v_ref);

#endif /* __AD74XX_H__ */
