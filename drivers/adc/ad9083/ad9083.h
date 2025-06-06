/***************************************************************************//**
 *   @file   ad9083.h
 *   @brief  Header file of ad9083 Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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
#ifndef __AD9083_H__
#define __AD9083_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "adi_ad9083.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9083_init_param` structure is used to hold initialization
 * parameters for the AD9083 device. It includes pointers to
 * initialization parameters for SPI and GPIO interfaces, as well as a
 * clock structure for the JESD receive clock. Additionally, it contains
 * a byte for selecting specific settings, making it a comprehensive
 * configuration structure for setting up the AD9083 device.
 *
 * @param spi_init Pointer to a structure for SPI initialization parameters.
 * @param gpio_reset Pointer to a structure for GPIO reset parameters.
 * @param gpio_pd Pointer to a structure for GPIO power down parameters.
 * @param uc Unsigned 8-bit integer for settings selection.
 * @param jesd_rx_clk Pointer to a structure for the JESD receive clock.
 ******************************************************************************/
struct ad9083_init_param {
	/* SPI */
	struct no_os_spi_init_param	*spi_init;
	/* GPIO reset */
	struct no_os_gpio_init_param	*gpio_reset;
	/* GPIO power down */
	struct no_os_gpio_init_param	*gpio_pd;
	/* Settings selection */
	uint8_t uc;
	/* jesd receive clock */
	struct no_os_clk *jesd_rx_clk;
};

/***************************************************************************//**
 * @brief The `ad9083_phy` structure is a descriptor for the AD9083 device,
 * encapsulating the necessary SPI and GPIO interfaces required for
 * device communication and control. It includes pointers to SPI and GPIO
 * descriptors for managing the device's reset, power down, and reference
 * selection functionalities, as well as an instance of the AD9083 device
 * type, facilitating interaction with the AD9083 hardware.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param gpio_reset Pointer to a GPIO descriptor for reset control.
 * @param gpio_pd Pointer to a GPIO descriptor for power down control.
 * @param gpio_ref_sel Pointer to a GPIO descriptor for reference selection.
 * @param adi_ad9083 Instance of the adi_ad9083_device_t representing the AD9083
 * device.
 ******************************************************************************/
struct ad9083_phy {
	/* SPI */
	struct no_os_spi_desc 	*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	/* GPIO power down */
	struct no_os_gpio_desc	*gpio_pd;
	/* GPIO reference selection */
	struct no_os_gpio_desc	*gpio_ref_sel;
	/* adi ad9083 device*/
	adi_ad9083_device_t	adi_ad9083;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD9083 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the necessary GPIO and
 * SPI interfaces, verifies the device ID, and configures the device
 * according to the specified settings. If the JESD204 link clock is
 * provided, it will be enabled. The function returns an error code if
 * initialization fails at any step, ensuring that resources are properly
 * cleaned up in case of failure.
 *
 * @param device A pointer to a pointer of type `struct ad9083_phy`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated structure upon successful initialization.
 * @param init_param A pointer to a `struct ad9083_init_param` containing the
 * initialization parameters. This includes SPI and GPIO
 * configurations, as well as optional JESD204 link clock. The
 * caller retains ownership of this structure, which must be
 * valid and properly initialized before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures that any allocated resources are
 * freed.
 ******************************************************************************/
int32_t ad9083_init(struct ad9083_phy **device,
		    struct ad9083_init_param *init_param);

/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly shut down and free all resources
 * associated with an AD9083 device instance. It should be called when
 * the device is no longer needed, ensuring that all allocated resources,
 * such as SPI and GPIO descriptors, are released. This function must be
 * called only after the device has been successfully initialized. If the
 * provided device pointer is null, the function will return an error
 * code. Proper error handling should be implemented to manage any
 * failures during the resource deallocation process.
 *
 * @param device A pointer to an ad9083_phy structure representing the device to
 * be removed. Must not be null. The caller retains ownership of
 * the pointer, but the resources it points to will be
 * deallocated. If null, the function returns an error code.
 * @return Returns 0 on successful removal and deallocation of resources. If the
 * device pointer is null or if any resource deallocation fails, a
 * negative error code is returned.
 ******************************************************************************/
int32_t ad9083_remove(struct ad9083_phy *device);

/* Read device register. */
/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from the
 * AD9083 device. It should be called when you need to read a register
 * value for configuration or status checking purposes. The function
 * requires a valid device structure and a register address within the
 * allowable range. It writes the read value to the provided pointer.
 * Ensure that the device has been properly initialized before calling
 * this function. If the register address is invalid, the function will
 * not perform the read operation.
 *
 * @param device A pointer to an initialized ad9083_phy structure representing
 * the device. Must not be null.
 * @param reg The address of the register to read. Must be less than
 * MAX_REG_ADDR.
 * @param readval A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t ad9083_reg_get(struct ad9083_phy *device, uint32_t reg,
		       uint8_t *readval);

/* Write device register. */
/***************************************************************************//**
 * @brief This function writes a given value to a specified register on the
 * AD9083 device. It should be used when you need to configure or modify
 * the settings of the device by writing to its registers. The function
 * requires that the device has been properly initialized and that the
 * register address is within the valid range. If the register address is
 * invalid, the function will not perform any write operation. It returns
 * an error code if the SPI communication fails, otherwise it returns 0
 * indicating success.
 *
 * @param device A pointer to an initialized ad9083_phy structure representing
 * the device. Must not be null.
 * @param reg The register address to write to. Must be less than MAX_REG_ADDR
 * to be valid.
 * @param writeval The value to write to the specified register. It is an 8-bit
 * unsigned integer.
 * @return Returns 0 on success, or a negative error code if the SPI write
 * operation fails.
 ******************************************************************************/
int32_t ad9083_reg_set(struct ad9083_phy *device, uint32_t reg,
		       uint8_t writeval);

#endif // __AD9083_H__
