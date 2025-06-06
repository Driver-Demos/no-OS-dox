/***************************************************************************//**
 *   @file   max31855.c
 *   @brief  Implementation of MAX31855 Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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

#ifndef __MAX31855_H__
#define __MAX31855_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define MAX31855_GET_INTERNAL_TEMP(x)		(no_os_field_get(NO_OS_GENMASK(15, 4), x))
#define MAX31855_GET_FAULT_BIT(x)		(no_os_field_get(NO_OS_BIT(16), x))
#define MAX31855_GET_THERMOCOUPLE_TEMP(x)	(no_os_field_get(NO_OS_GENMASK(31, 18), x))
#define MAX31855_GET_FAULTS(x)			(no_os_field_get(NO_OS_GENMASK(2, 0), x))

/**
 * Decimal resolution for the 2 temperatures (0.25 C and 0.0625 C / LSB).
 * integer_temp = raw_temp / dec_div.
 */
#define MAX31855_THERMOCOUPLE_TEMP_DEC_DIV	4
#define MAX31855_INTERNAL_TEMP_DEC_DIV	 	16

/** Sign bit position */
#define MAX31855_THERMOCOUPLE_TEMP_SIGN_POS	13
#define MAX31855_INTERNAL_TEMP_SIGN_POS		11

/***************************************************************************//**
 * @brief The `max31855_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the MAX31855
 * device's SPI communication interface. It contains a single member,
 * `spi_init`, which is a structure of type `no_os_spi_init_param` that
 * holds the necessary configuration details for initializing the SPI
 * interface used by the MAX31855 thermocouple-to-digital converter.
 *
 * @param spi_init A structure containing the initialization parameters for the
 * SPI interface.
 ******************************************************************************/
struct max31855_init_param {
	struct no_os_spi_init_param spi_init;
};

/***************************************************************************//**
 * @brief The `max31855_decimal` structure is used to represent temperature
 * values in a split integer and decimal format, allowing for precise
 * temperature readings. This structure is particularly useful in the
 * context of the MAX31855 thermocouple-to-digital converter, where
 * temperature values need to be represented with both integer and
 * fractional components for accurate measurement and processing.
 *
 * @param integer Stores the integer part of the temperature value.
 * @param decimal Stores the decimal part of the temperature value.
 ******************************************************************************/
struct max31855_decimal {
	int32_t integer;
	int32_t decimal;
};

/***************************************************************************//**
 * @brief The `_max31855_fault_sts_mask` structure is a bit-field data structure
 * used to represent the fault status of a thermocouple connected to a
 * MAX31855 device. It contains three 1-bit fields: `SCV`, `SCG`, and
 * `OC`, each representing a specific fault condition such as short
 * circuit to VCC, short circuit to GND, and open circuit, respectively.
 * This structure is typically used within a union to facilitate easy
 * access and manipulation of the fault status as a whole or as
 * individual fault indicators.
 *
 * @param SCV Indicates if the thermocouple is short circuited to VCC.
 * @param SCG Indicates if the thermocouple is short circuited to GND.
 * @param OC Indicates if the thermocouple probe is not connected.
 ******************************************************************************/
struct _max31855_fault_sts_mask {
	/** Thermocouple is short circuited to VCC */
	uint8_t SCV: 1;
	/** Thermocouple is short circuited to GND */
	uint8_t SCG: 1;
	/** Thermocouple probe not connected */
	uint8_t OC: 1;
};

/***************************************************************************//**
 * @brief The `max31855_fault_sts` union is designed to encapsulate the fault
 * status of the MAX31855 thermocouple-to-digital converter. It provides
 * two ways to access the fault information: as a structured set of bit
 * fields (`fields`) that indicate specific fault conditions such as
 * short circuits to VCC or GND, and open circuit detection, or as a raw
 * 8-bit value (`value`). This dual representation allows for flexible
 * handling of fault data, either by examining individual fault
 * conditions or by processing the entire fault status byte at once.
 *
 * @param fields A structure containing bit fields representing specific fault
 * conditions of the MAX31855 device.
 * @param value A single byte representing the fault status as an 8-bit unsigned
 * integer.
 ******************************************************************************/
union max31855_fault_sts {
	struct _max31855_fault_sts_mask fields;
	uint8_t value;
};

/***************************************************************************//**
 * @brief The `max31855_dev` structure is a descriptor for the MAX31855
 * thermocouple-to-digital converter device, encapsulating the necessary
 * components for communication and fault status monitoring. It includes
 * a pointer to an SPI descriptor for handling communication with the
 * device and a union to represent various fault conditions that may be
 * detected by the MAX31855, such as short circuits or open connections.
 *
 * @param comm_desc A pointer to a no_os_spi_desc structure used for SPI
 * communication.
 * @param fault A union representing the fault status of the MAX31855 device.
 ******************************************************************************/
struct max31855_dev {
	struct no_os_spi_desc *comm_desc;
	union max31855_fault_sts fault;
};

/***************************************************************************//**
 * @brief This function sets up a MAX31855 device by allocating necessary
 * resources and initializing communication parameters. It should be
 * called before any other operations on the MAX31855 device. The
 * function requires valid initialization parameters and will return an
 * error if memory allocation fails or if the communication
 * initialization encounters an issue. Ensure that the device pointer is
 * not null and points to a valid memory location for storing the device
 * descriptor.
 *
 * @param device A pointer to a pointer of type `struct max31855_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct max31855_init_param` containing
 * initialization parameters for the SPI communication. Must
 * not be null and should be properly initialized before
 * calling this function.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., memory allocation failure or SPI
 * initialization failure).
 ******************************************************************************/
int max31855_init(struct max31855_dev **, struct max31855_init_param *);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for a
 * MAX31855 device when it is no longer needed. This function should be
 * called to clean up after a successful initialization with
 * `max31855_init`. It ensures that the communication descriptor is
 * removed and the device memory is freed. The function must be called
 * with a valid device pointer; otherwise, it returns an error. It is
 * important to handle the return value to ensure that resources are
 * released correctly.
 *
 * @param device A pointer to a `max31855_dev` structure representing the device
 * to be removed. Must not be null. If null, the function returns
 * -ENODEV.
 * @return Returns 0 on success, -EINVAL if the communication descriptor removal
 * fails, or -ENODEV if the device pointer is null.
 ******************************************************************************/
int max31855_remove(struct max31855_dev *);

/***************************************************************************//**
 * @brief This function retrieves the raw 32-bit register value from a MAX31855
 * thermocouple-to-digital converter device. It should be called when raw
 * data is needed for further processing or analysis. The function
 * requires a valid device descriptor and a pointer to store the
 * retrieved value. It performs a SPI communication to read the data and
 * checks for any fault conditions in the device. If a fault is detected,
 * the function returns an error code and updates the device's fault
 * status. This function must be called after the device has been
 * properly initialized.
 *
 * @param device A pointer to a max31855_dev structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param val A pointer to a uint32_t variable where the raw register value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if a fault is detected
 * or if the SPI communication fails. The raw register value is stored
 * in the location pointed to by 'val' on success.
 ******************************************************************************/
int max31855_read_raw(struct max31855_dev *, uint32_t *);

/***************************************************************************//**
 * @brief Use this function to obtain the current thermocouple and internal
 * temperatures from a MAX31855 device, expressed in degrees Celsius. It
 * requires a valid device descriptor and outputs the temperatures in a
 * structured format. Ensure the device has been properly initialized
 * before calling this function. The function will return an error code
 * if the device is not available or if reading the raw data fails.
 *
 * @param device A pointer to a max31855_dev structure representing the device.
 * Must not be null. If null, the function returns -ENODEV.
 * @param thermocouple_temp A pointer to a max31855_decimal structure where the
 * thermocouple temperature will be stored. Must not be
 * null. The function populates this structure with the
 * temperature data.
 * @param internal_temp A pointer to a max31855_decimal structure where the
 * internal temperature will be stored. Must not be null.
 * The function populates this structure with the
 * temperature data.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error.
 ******************************************************************************/
int max31855_read_temp(struct max31855_dev *, struct max31855_decimal *,
		       struct max31855_decimal *);

#endif
