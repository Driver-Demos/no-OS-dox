/***************************************************************************//**
 * @file ad9553.h
 * @brief Header file of AD9553 Driver.
 * @author DHotolea (dan.hotoleanu@analog.com)
 ********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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
#ifndef AD9553_H_
#define AD9553_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD9553_SPI_CONFIG				        0x000
#define AD9553_PLL_CHARGE_PUMP_PFD_CTRL	        		0x00B
#define AD9553_PLL_CTRL	        				0x00D
#define AD9553_P1_DIV_HIGH     					0x015
#define AD9553_P1_DIV_LOW_P2_DIV_HIGH   			0x016
#define AD9553_P2_DIV_LOW				   	0x017
#define AD9553_P0_DIV					   	0x018
#define AD9553_N_DIV_HIGH				   	0x012
#define AD9553_N_DIV_MEDIUM				   	0x013
#define AD9553_N_DIV_LOW				   	0x014
#define AD9553_REFA_DIV_HIGH				   	0x01F
#define AD9553_REFA_DIV_LOW					0x020
#define AD9553_K_VALUE					   	0x021
#define AD9553_REFA_DIFF				   	0x029
#define AD9553_OUT1_DRIVER_CTRL			   		0x032
#define AD9553_OUT2_DRIVER_CTRL			   		0x034
#define AD9553_IO_UPDATE				   	0x005

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9553_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD9553 device,
 * specifically focusing on the SPI interface configuration. It contains
 * a single member, `spi_init`, which is a structure itself that holds
 * the necessary parameters to initialize the SPI communication needed by
 * the AD9553 device.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad9553_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/***************************************************************************//**
 * @brief The `ad9553_dev` structure is a simple data structure used to
 * represent a device instance of the AD9553, a clock generator IC. It
 * contains a single member, `spi_desc`, which is a pointer to a SPI
 * descriptor structure. This member is used to manage the SPI
 * communication interface required for interacting with the AD9553
 * device. The structure is part of a driver that facilitates the
 * initialization, configuration, and control of the AD9553 through SPI
 * communication.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for the device.
 ******************************************************************************/
struct ad9553_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9553 device. It requires a valid device structure and a register
 * address to read from. The function will store the read value in the
 * provided memory location pointed to by reg_data. It is essential to
 * ensure that the device has been properly initialized before calling
 * this function. The function returns an error code if the read
 * operation fails, which should be checked to ensure successful
 * execution.
 *
 * @param dev A pointer to an ad9553_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer, and the valid range depends on the device's
 * register map.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9553_reg_read(struct ad9553_dev *dev, uint16_t reg_addr,
			uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register of the AD9553 device. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address within the valid range of the device's register map.
 * It performs the write operation using the SPI interface associated
 * with the device. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to an initialized ad9553_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. Must be a valid register address within the AD9553's
 * register map.
 * @param reg_data The 8-bit data to be written to the specified register.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9553_reg_write(struct ad9553_dev *dev, uint16_t reg_addr,
			 uint8_t reg_data);

/***************************************************************************//**
 * @brief This function initializes the AD9553 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * AD9553 device. The function sets up the SPI interface and configures
 * various registers to prepare the device for operation. If the
 * initialization is successful, the function returns a pointer to the
 * device structure through the provided pointer. The caller is
 * responsible for ensuring that the `init_param` is properly populated
 * before calling this function. In case of failure, the function returns
 * a negative error code and the device pointer is not valid.
 *
 * @param device A double pointer to a `struct ad9553_dev` where the initialized
 * device structure will be stored. Must not be null. The caller
 * takes ownership of the allocated structure upon successful
 * initialization.
 * @param init_param A pointer to a `struct ad9553_init_param` containing the
 * initialization parameters, including SPI configuration.
 * Must not be null and must be properly initialized before
 * calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, and the device
 * pointer is not valid.
 ******************************************************************************/
int32_t ad9553_setup(struct ad9553_dev **device,
		     const struct ad9553_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources allocated
 * for an AD9553 device when it is no longer needed. This function should
 * be called to ensure that the SPI descriptor associated with the device
 * is removed and that the memory allocated for the device structure is
 * freed. It is important to call this function to prevent resource
 * leaks. The function must be called only after the device has been
 * successfully initialized and used.
 *
 * @param dev A pointer to an ad9553_dev structure representing the device to be
 * removed. Must not be null. The function will handle freeing the
 * memory associated with this structure.
 * @return Returns 0 on success or a negative error code if the SPI descriptor
 * removal fails.
 ******************************************************************************/
int32_t ad9553_remove(struct ad9553_dev *dev);

#endif
