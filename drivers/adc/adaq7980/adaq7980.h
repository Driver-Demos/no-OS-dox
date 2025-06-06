/***************************************************************************//**
 * @file adaq7980.c
 * @brief Header file for adaq7980 Driver.
 * @author ADI
 ********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
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
#ifndef ADAQ7980_H_
#define ADAQ7980_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "spi_engine.h"
#include "no_os_pwm.h"
#include "no_os_gpio.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `adaq7980_init_param` structure is used to encapsulate all the
 * necessary initialization parameters required to set up the ADAQ7980
 * device. It includes pointers to initialization structures for SPI
 * communication, SPI offload module, PWM generator for triggering, and
 * GPIO for power down control. This structure is essential for
 * configuring the device's hardware interfaces before operation.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param offload_init_param Pointer to SPI module offload initialization
 * parameters.
 * @param trigger_pwm_init Pointer to PWM generator initialization parameters.
 * @param gpio_pd_ldo Pointer to power down GPIO initialization parameters.
 ******************************************************************************/
struct adaq7980_init_param {
	/* SPI */
	struct no_os_spi_init_param		*spi_init;
	/* SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	/* PWM generator init structure */
	struct no_os_pwm_init_param	*trigger_pwm_init;
	/** Power down GPIO initialization structure. */
	struct no_os_gpio_init_param	*gpio_pd_ldo;
};

/***************************************************************************//**
 * @brief The `adaq7980_dev` structure represents a device instance for the
 * ADAQ7980, an analog-to-digital converter. It encapsulates the
 * necessary descriptors and parameters for managing SPI communication,
 * PWM-based conversion triggering, SPI offloading, and GPIO-based power
 * management. This structure is essential for initializing and operating
 * the ADAQ7980 device within a system, providing a cohesive interface
 * for hardware interaction.
 *
 * @param spi_desc A pointer to the SPI descriptor used for SPI communication.
 * @param trigger_pwm_desc A pointer to the PWM descriptor used for triggering
 * conversions.
 * @param offload_init_param A pointer to the SPI engine offload initialization
 * parameters.
 * @param gpio_pd_ldo A pointer to the GPIO descriptor for handling power down
 * operations.
 ******************************************************************************/
struct adaq7980_dev {
	/* SPI descriptor */
	struct no_os_spi_desc		*spi_desc;
	/* Trigger conversion PWM generator descriptor */
	struct no_os_pwm_desc		*trigger_pwm_desc;
	/* SPI module offload init */
	struct spi_engine_offload_init_param *offload_init_param;
	/** Power down GPIO handler. */
	struct no_os_gpio_desc	*gpio_pd_ldo;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the ADAQ7980 device by allocating necessary
 * resources and initializing its components, such as SPI, GPIO, and PWM.
 * It must be called before any other operations on the device. The
 * function configures the power-down GPIO if provided, and initializes
 * the SPI and PWM interfaces based on the parameters supplied. If any
 * initialization step fails, the function will clean up and return an
 * error code. Ensure that the `init_param` structure is properly
 * populated before calling this function.
 *
 * @param device A pointer to a pointer of type `struct adaq7980_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory.
 * @param init_param A pointer to a `struct adaq7980_init_param` containing
 * initialization parameters for the device. Must not be null.
 * The structure should be fully populated with valid SPI,
 * GPIO, and PWM initialization parameters before calling this
 * function.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during setup.
 ******************************************************************************/
int32_t adaq7980_setup(struct adaq7980_dev **device,
		       struct adaq7980_init_param *init_param);

/* Read data from device */
/***************************************************************************//**
 * @brief This function reads a specified number of samples from the ADAQ7980
 * device and stores them in the provided buffer. It should be called
 * after the device has been properly initialized using the appropriate
 * setup function. The function handles the SPI communication required to
 * retrieve the data. If the SPI engine offload initialization or
 * transfer fails, the function returns an error code. Ensure that the
 * buffer provided is large enough to hold the requested number of
 * samples.
 *
 * @param dev A pointer to an initialized adaq7980_dev structure representing
 * the device. Must not be null.
 * @param buf A pointer to a buffer where the read data will be stored. Must not
 * be null and should be large enough to hold 'samples' number of
 * 16-bit values.
 * @param samples The number of 16-bit samples to read from the device. Must be
 * a positive integer.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad7980_read_data(struct adaq7980_dev *dev,
			 uint16_t *buf,
			 uint16_t samples);

#endif
