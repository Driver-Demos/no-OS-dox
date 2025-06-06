/***************************************************************************//**
 *   @file   max538x.h
 *   @brief  Header file of max538x Family Driver.
 *   @author JSanbuen (jose.sanbuenaventura@analog.com)
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
#ifndef __MAX538x_H__
#define __MAX538x_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_i2c.h"
#include "no_os_util.h"

/******************************************************************************/
/************************** max538x Definitions *******************************/
/******************************************************************************/
/* MAX5380 default resolution */
#define MAX538X_RESOLUTION 		0xFF

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `max538x_type` enumeration defines a set of constants representing
 * different variants of the MAX538x family of devices. Each constant
 * corresponds to a specific model within the family, which may have
 * different features or specifications. This enumeration is used to
 * identify and differentiate between the various models when configuring
 * or interacting with the devices in software.
 *
 * @param MAX5380L Represents a specific variant of the MAX538x device family.
 * @param MAX5380M Represents a specific variant of the MAX538x device family.
 * @param MAX5380N Represents a specific variant of the MAX538x device family.
 * @param MAX5380K Represents a specific variant of the MAX538x device family.
 * @param MAX5381L Represents a specific variant of the MAX538x device family.
 * @param MAX5381M Represents a specific variant of the MAX538x device family.
 * @param MAX5381N Represents a specific variant of the MAX538x device family.
 * @param MAX5381K Represents a specific variant of the MAX538x device family.
 * @param MAX5382L Represents a specific variant of the MAX538x device family.
 * @param MAX5382M Represents a specific variant of the MAX538x device family.
 * @param MAX5382N Represents a specific variant of the MAX538x device family.
 * @param MAX5382K Represents a specific variant of the MAX538x device family.
 ******************************************************************************/
enum max538x_type {
	MAX5380L,
	MAX5380M,
	MAX5380N,
	MAX5380K,
	MAX5381L,
	MAX5381M,
	MAX5381N,
	MAX5381K,
	MAX5382L,
	MAX5382M,
	MAX5382N,
	MAX5382K,
};

/***************************************************************************//**
 * @brief The `max538x_chip_info` structure is used to store essential
 * information about a MAX538x chip, specifically its voltage factor and
 * I2C address. This structure is likely used to configure and
 * communicate with the chip over an I2C interface, allowing for the
 * adjustment of voltage outputs or other settings specific to the
 * MAX538x family of devices.
 *
 * @param vfactor A uint8_t representing a voltage factor for the chip.
 * @param addr A uint8_t representing the I2C address of the chip.
 ******************************************************************************/
struct max538x_chip_info {
	uint8_t vfactor;
	uint8_t addr;
};

/***************************************************************************//**
 * @brief The `max538x_dev` structure is designed to represent a device from the
 * MAX538x family, encapsulating essential configuration and
 * communication details. It includes a pointer to an I2C descriptor for
 * managing I2C communication, an enumeration to specify the active
 * device type, and fields for voltage supply and reference levels,
 * particularly for the MAX5382 variant. This structure is crucial for
 * initializing and managing the device's settings and operations within
 * the driver.
 *
 * @param i2c_desc Pointer to an I2C descriptor for communication.
 * @param active_device Specifies the active device type from the max538x
 * family.
 * @param max538x_vdd Voltage supply level for the MAX5382 variant.
 * @param max538x_vref Reference voltage level for the device.
 ******************************************************************************/
struct max538x_dev {
	/* I2C */
	struct no_os_i2c_desc	*i2c_desc;
	/* Device Settings*/
	enum max538x_type active_device;
	/* Device VDD (for MAX5382 variant) */
	uint8_t max538x_vdd ;
	/* Device VREF */
	uint8_t max538x_vref ;
};

/***************************************************************************//**
 * @brief The `max538x_init_param` structure is used to define the
 * initialization parameters for a MAX538x device, which is part of a
 * family of digital potentiometers. It includes settings for the I2C
 * communication interface, the specific type of MAX538x device in use,
 * and voltage parameters such as the supply voltage (VDD) and reference
 * voltage (VREF). This structure is essential for configuring the device
 * before it is used in an application.
 *
 * @param i2c_init Holds the initialization parameters for the I2C communication
 * interface.
 * @param active_device Specifies the type of MAX538x device being used.
 * @param max538x_vdd Represents the supply voltage for the MAX5382 variant.
 * @param max538x_vref Represents the reference voltage for the device.
 ******************************************************************************/
struct max538x_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
	/* Device Settings*/
	enum max538x_type active_device;
	/* Device VDD (for MAX5382 variant) */
	uint8_t max538x_vdd ;
	/* Device VREF */
	uint8_t max538x_vref ;
};

/***************************************************************************//**
 * @brief The `chip_info` variable is an external constant array of
 * `max538x_chip_info` structures. Each element in this array contains
 * information about a specific MAX538x chip, including its voltage
 * factor (`vfactor`) and I2C address (`addr`).
 *
 * @details This variable is used to store and provide access to the
 * configuration details of different MAX538x chip variants.
 ******************************************************************************/
extern const struct max538x_chip_info chip_info[];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initializes the communications peripheral and checks if the device is present. */
/***************************************************************************//**
 * @brief This function initializes a MAX538x device by setting up the necessary
 * I2C communication and configuring the device based on the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure and configures the I2C interface using the parameters
 * specified in `init_param`. If the initialization is successful, a
 * pointer to the device structure is returned through the `device`
 * parameter. The function handles invalid initialization parameters by
 * returning an error code, ensuring that the device is not initialized
 * with incorrect settings.
 *
 * @param device A pointer to a pointer of `struct max538x_dev` where the
 * initialized device structure will be stored. Must not be null.
 * The caller is responsible for freeing the allocated memory
 * using `max538x_remove()`.
 * @param init_param A `struct max538x_init_param` containing the initialization
 * parameters for the device. This includes the I2C
 * initialization parameters, the active device type, and the
 * VDD value for MAX5382 variants. The `max538x_vdd` must be
 * non-null and non-zero for devices beyond MAX5381K.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid parameters, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int max538x_init(struct max538x_dev **device,
		 struct max538x_init_param init_param);

/* Free the resources allocated by max538x_init(). */
/***************************************************************************//**
 * @brief Use this function to release all resources associated with a MAX538x
 * device when it is no longer needed. This function should be called
 * after you are done using the device to ensure that all allocated
 * memory and resources are properly freed. It is important to call this
 * function to prevent memory leaks. The function returns an integer
 * status code indicating the success or failure of the operation.
 *
 * @param dev A pointer to a max538x_dev structure representing the device to be
 * removed. This pointer must not be null, and it must point to a
 * valid device structure that was previously initialized using
 * max538x_init(). The function will handle invalid pointers by
 * returning an error code.
 * @return Returns an integer status code from the underlying I2C removal
 * operation, which indicates success or failure of the resource
 * deallocation process.
 ******************************************************************************/
int max538x_remove(struct max538x_dev *dev);

/* Sets the DAC output using VOUT */
/***************************************************************************//**
 * @brief Use this function to set the output voltage of a MAX538x digital-to-
 * analog converter (DAC) device. The function requires a valid device
 * structure and a desired output voltage within the permissible range.
 * It is essential to ensure that the voltage is non-negative and does
 * not exceed the device's reference voltage. If the voltage is outside
 * this range, the function will return an error. This function should be
 * called after the device has been successfully initialized.
 *
 * @param dev A pointer to a max538x_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership and must ensure it is not null.
 * @param vout The desired output voltage as a float. It must be within the
 * range of 0 to the device's reference voltage (max538x_vref).
 * Values outside this range will result in an error.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL if the voltage is out of range.
 ******************************************************************************/
int max538x_set_voutput(struct max538x_dev *dev, float vout);

#endif	/* __MAX538x_H__ */
