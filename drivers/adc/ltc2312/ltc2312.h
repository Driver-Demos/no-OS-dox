/*!
REVISION HISTORY
$Revision: 1807 $
$Date: 2013-07-29 13:06:06 -0700 (Mon, 29 Jul 2013) $

Copyright (c) 2013, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.
*/

/*! @file
    @ingroup LTC2315
    Header for LTC2315: 12/14-bit 1Msps ADC
*/

#ifndef LTC231X_H
#define LTC231X_H

/*! Define the SPI CS pin */
#ifndef LTC2315_CS
#define LTC2315_CS QUIKEVAL_CS
#endif

/*! @name LTC2315 Channel Address */
//! @{
// Channel Address
#define LTC2315_ADDRESS 0x00
//!@}

#define LTC2312_READ_BYTES_NUMBER 2
#define LTC2312_READ_VALUES_NUMBER 100

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `device_type` enumeration defines two constants, `LTC2312_12` and
 * `LTC2312_14`, which are used to specify the resolution of the LTC2312
 * analog-to-digital converter (ADC) device, either 12-bit or 14-bit.
 * This enumeration is used within the `ltc2312_dev` and
 * `ltc2312_init_param` structures to indicate the specific type of
 * device being initialized or operated upon.
 *
 * @param LTC2312_12 Represents a 12-bit version of the LTC2312 device.
 * @param LTC2312_14 Represents a 14-bit version of the LTC2312 device.
 ******************************************************************************/
enum device_type {
	LTC2312_12,
	LTC2312_14
};

/***************************************************************************//**
 * @brief The `ltc2312_dev` structure is designed to represent a device
 * configuration for the LTC2312 ADC, encapsulating both the device type
 * and the SPI communication descriptor. This structure is essential for
 * initializing and managing the communication with the ADC, allowing for
 * operations such as reading data and converting it to voltage. It
 * serves as a central data structure for handling the device's
 * characteristics and communication interface.
 *
 * @param type Specifies the type of the device, which can be either LTC2312_12
 * or LTC2312_14.
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 ******************************************************************************/
struct ltc2312_dev {
	/* Device characteristics */
	enum device_type type;
	/* SPI */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `ltc2312_init_param` structure is used to initialize an LTC2312
 * device, specifying both the device type and the SPI interface
 * parameters. It is essential for setting up the device correctly before
 * any operations can be performed, ensuring that the device
 * characteristics and communication settings are properly configured.
 *
 * @param type Specifies the type of the device, either LTC2312_12 or
 * LTC2312_14.
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ltc2312_init_param {
	/* Device characteristics */
	enum device_type type;
	/* SPI */
	struct no_os_spi_init_param spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initializes the ltc231x. */
/***************************************************************************//**
 * @brief This function sets up an LTC2312 device by allocating memory for the
 * device structure and initializing the SPI interface based on the
 * provided initialization parameters. It must be called before any other
 * operations on the LTC2312 device to ensure that the device is properly
 * configured. The function expects valid initialization parameters and
 * will return an error if memory allocation fails or if the SPI
 * initialization is unsuccessful. The caller is responsible for managing
 * the memory of the device structure, including freeing it when no
 * longer needed.
 *
 * @param device A pointer to a pointer of type `struct ltc2312_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ltc2312_init_param` containing the
 * initialization parameters for the device, including SPI
 * settings and device type. Must not be null and should be
 * properly initialized before calling this function.
 * @return Returns 0 on success, or a negative error code if memory allocation
 * or SPI initialization fails.
 ******************************************************************************/
int32_t ltc2312_setup(struct ltc2312_dev **device,
		      struct ltc2312_init_param *init_param);

/* Free the resources allocated by ltc231x_setup(). */
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an LTC2312
 * device after it is no longer needed. This function should be called to
 * clean up after a successful setup with `ltc2312_setup`. It ensures
 * that the SPI descriptor associated with the device is properly removed
 * and the memory allocated for the device structure is freed. The
 * function must be called with a valid device pointer; otherwise, it
 * returns an error code.
 *
 * @param dev A pointer to an `ltc2312_dev` structure representing the device to
 * be removed. Must not be null. If null, the function returns -1.
 * @return Returns 0 on successful removal of the device. If the device pointer
 * is null, returns -1. If an error occurs during SPI removal, the
 * function returns the error code from the SPI removal operation.
 ******************************************************************************/
int32_t ltc2312_remove(struct ltc2312_dev *dev);

/* Reads the LTC2315 and returns 32-bit data in offset binary format. */
/***************************************************************************//**
 * @brief This function reads multiple values from the LTC2312 ADC, averages
 * them, and stores the result in the provided pointer. It is typically
 * used to obtain a stable ADC reading by averaging out noise. The
 * function must be called with a valid device structure that has been
 * properly initialized using `ltc2312_setup`. The pointer to store the
 * ADC code must not be null, as the function will write the averaged ADC
 * value to this location. If the SPI communication fails at any point,
 * the function will return a non-zero error code.
 *
 * @param dev A pointer to an initialized `ltc2312_dev` structure representing
 * the ADC device. Must not be null.
 * @param ptr_adc_code A pointer to a `uint16_t` where the averaged ADC code
 * will be stored. Must not be null.
 * @return Returns 0 on success or a non-zero error code if SPI communication
 * fails.
 ******************************************************************************/
int32_t ltc2312_read(struct ltc2312_dev *dev, uint16_t *ptr_adc_code);

/* Calculates the LTC2315 input voltage given the binary data and LSB weight. */
/***************************************************************************//**
 * @brief This function is used to convert a raw ADC code obtained from the
 * LTC2312 device into a corresponding input voltage, based on a given
 * reference voltage. It should be called after successfully reading an
 * ADC code from the device. The function requires a valid device
 * structure and a reference voltage to perform the conversion. The
 * result is stored in the provided voltage pointer. Ensure that the
 * voltage pointer is not null before calling this function.
 *
 * @param dev A pointer to an ltc2312_dev structure representing the initialized
 * device. Must not be null.
 * @param adc_code A 16-bit unsigned integer representing the raw ADC code to be
 * converted.
 * @param vref A floating-point value representing the reference voltage used
 * for conversion. Should be a positive, non-zero value.
 * @param voltage A pointer to a float where the calculated input voltage will
 * be stored. Must not be null.
 * @return None
 ******************************************************************************/
void ltc2312_code_to_voltage(struct ltc2312_dev *dev, uint16_t adc_code,
			     float vref, float *voltage);

#endif  //  LTC2315_H
