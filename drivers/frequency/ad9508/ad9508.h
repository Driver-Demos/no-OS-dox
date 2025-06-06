/***************************************************************************//**
 * @file ad9508.h
 * @brief Header file of AD9508 Driver.
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
#ifndef AD9508_H_
#define AD9508_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD9508_SPI_CONFIG				    0x000
#define AD9508_PART_ID_LOW				    0x00C
#define AD9508_PART_ID_HIGH				    0x00D
#define AD9508_OUT1_DIV_RATIO_LOW			    0x01B
#define AD9508_OUT1_DIV_RATIO_HIGH			    0x01C
#define AD9508_OUT1_PHASE_LOW			   	    0x01D
#define AD9508_OUT1_PHASE_HIGH			   	    0x01E
#define AD9508_PART_ID_VALUE			   	    0x005
#define AD9508_DIVIDE_RATIO_ONE			   	    0x000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9508_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD9508 device,
 * specifically focusing on the SPI interface configuration. It contains
 * a single member, `spi_init`, which is a structure itself that holds
 * the necessary parameters to initialize the SPI communication needed by
 * the AD9508 device.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad9508_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/***************************************************************************//**
 * @brief The `ad9508_dev` structure is a simple data structure used to
 * represent a device instance for the AD9508 driver. It contains a
 * single member, `spi_desc`, which is a pointer to a SPI descriptor.
 * This structure is essential for managing the SPI communication
 * required to interface with the AD9508 device, allowing for operations
 * such as reading from and writing to the device's registers.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for the device.
 ******************************************************************************/
struct ad9508_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to read a specific register from the AD9508 device.
 * It requires a valid device structure and a register address to read
 * from. The function will store the read value in the provided memory
 * location. Ensure that the device has been properly initialized before
 * calling this function. The function returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to an initialized ad9508_dev structure. Must not be
 * null. The structure should be properly set up using ad9508_setup
 * before calling this function.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer representing the register address within the
 * device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null. The caller is responsible for
 * providing a valid memory location.
 * @return Returns an int32_t error code. A non-negative value indicates
 * success, while a negative value indicates an error during the read
 * operation.
 ******************************************************************************/
int32_t ad9508_reg_read(struct ad9508_dev *dev, uint16_t reg_addr,
			uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register of the AD9508 device. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address within the valid range of the device. It is typically
 * used when configuring the device or updating its settings. The
 * function returns an integer status code indicating the success or
 * failure of the write operation.
 *
 * @param dev A pointer to an initialized ad9508_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. Must be within the valid address range of the AD9508
 * device.
 * @param reg_data The 8-bit data to be written to the specified register.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error during the write operation.
 ******************************************************************************/
int32_t ad9508_reg_write(struct ad9508_dev *dev, uint16_t reg_addr,
			 uint8_t reg_data);

/***************************************************************************//**
 * @brief This function sets up the AD9508 device by initializing the necessary
 * SPI communication and configuring the device to pass a 125MHz input
 * clock unmodified. It must be called before any other operations on the
 * AD9508 device to ensure proper initialization. The function allocates
 * memory for the device structure and performs a series of register
 * writes to configure the device. If any step fails, the function
 * returns an error code and the device is not initialized. The caller is
 * responsible for managing the memory of the device structure after
 * successful initialization.
 *
 * @param device A pointer to a pointer of type `struct ad9508_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad9508_init_param` containing
 * initialization parameters, specifically for SPI
 * configuration. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered.
 ******************************************************************************/
int32_t ad9508_setup(struct ad9508_dev **device,
		     const struct ad9508_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources
 * associated with an AD9508 device when it is no longer needed. This
 * function should be called to ensure that the SPI descriptor and any
 * allocated memory are freed, preventing resource leaks. It is important
 * to call this function only after the device has been successfully
 * initialized and used, and when no further operations on the device are
 * required.
 *
 * @param dev A pointer to an ad9508_dev structure representing the device to be
 * removed. Must not be null. The function will handle freeing the
 * associated resources, so the caller should not attempt to use or
 * free this pointer after calling this function.
 * @return Returns 0 on success or a negative error code if the SPI descriptor
 * could not be removed.
 ******************************************************************************/
int32_t ad9508_remove(struct ad9508_dev *dev);

#endif
