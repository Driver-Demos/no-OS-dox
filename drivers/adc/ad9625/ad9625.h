/***************************************************************************//**
 * @file ad9625.h
 * @brief Header file of AD9625 Driver.
 * @author DBogdan (dragos.bogdan@analog.com)
 *******************************************************************************
 * Copyright 2014(c) Analog Devices, Inc.
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
 ******************************************************************************/
#ifndef AD9625_H_
#define AD9625_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9625_REG_CHIP_PORT_CONF				0x000
#define AD9625_REG_CHIP_ID					0x001
#define AD9625_REG_POWER_MODE					0x008
#define AD9625_REG_PLL_STATUS					0x00A
#define AD9625_REG_TEST_CNTRL					0x00D
#define AD9625_REG_OUTPUT_MODE					0x014
#define AD9625_REG_OUTPUT_ADJUST				0x015
#define AD9625_REG_SYSREF_CONTROL				0x03A
#define AD9625_REG_JESD204B_LINK_CNTRL_1			0x05F
#define AD9625_REG_JESD204B_CONFIGURATION			0x072
#define AD9625_REG_JESD204B_LANE_POWER_MODE			0x080
#define AD9625_REG_TRANSFER					0x0FF
#define AD9625_REG_IRQ_STATUS					0x100
#define AD9625_REG_DIVCLK_OUT_CNTRL				0x120
#define AD9625_REG_SYSREF_SETUP_TIME_GUARDBAND			0x13C

#define AD9625_CHIP_ID						0x041

#define AD9625_TEST_OFF						0x000
#define AD9625_TEST_MID_SCALE					0x001
#define AD9625_TEST_POS_FSCALE					0x002
#define AD9625_TEST_NEG_FSCALE					0x003
#define AD9625_TEST_CHECKBOARD					0x004
#define AD9625_TEST_PNLONG					0x005
#define AD9625_TEST_ONE2ZERO					0x007
#define AD9625_TEST_PATTERN					0x008
#define AD9625_TEST_RAMP					0x00F

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9625_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD9625 device.
 * It includes SPI initialization parameters, the lane rate for data
 * transmission, and an array for test samples, which are essential for
 * configuring the device's operation and testing its functionality.
 *
 * @param spi_init A structure containing the initialization parameters for the
 * SPI interface.
 * @param lane_rate_kbps An unsigned 32-bit integer representing the lane rate
 * in kilobits per second.
 * @param test_samples An array of four unsigned 32-bit integers used for test
 * samples.
 ******************************************************************************/
struct ad9625_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	uint32_t	lane_rate_kbps;
	uint32_t	test_samples[4];
};

/***************************************************************************//**
 * @brief The `ad9625_dev` structure is a simple data structure used to
 * represent an instance of the AD9625 device in the software. It
 * contains a single member, `spi_desc`, which is a pointer to a SPI
 * descriptor. This descriptor is used to manage the SPI communication
 * interface for the AD9625 device, allowing the software to perform read
 * and write operations to the device's registers over SPI.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for the device.
 ******************************************************************************/
struct ad9625_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9625 device using the SPI interface. It is essential to ensure that
 * the device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. The read value is stored in the
 * provided memory location pointed to by reg_data. This function is
 * typically used when you need to retrieve configuration or status
 * information from the AD9625 device.
 *
 * @param dev A pointer to an ad9625_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer representing the register's address within
 * the device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null, and the caller is responsible for
 * providing a valid memory location.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9625_spi_read(struct ad9625_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * of the AD9625 device using SPI communication. It is essential to
 * ensure that the device has been properly initialized and configured
 * before calling this function. The function requires a valid device
 * structure and a register address to which the data will be written. It
 * is typically used when configuring or updating the settings of the
 * AD9625 device. Proper error handling should be implemented to handle
 * any communication issues that may arise during the SPI transaction.
 *
 * @param dev A pointer to an initialized ad9625_dev structure representing the
 * device. Must not be null, and the SPI descriptor within must be
 * valid.
 * @param reg_addr The 16-bit address of the register to write to. Must be a
 * valid register address for the AD9625 device.
 * @param reg_data The 8-bit data to write to the specified register. Represents
 * the new value for the register.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * write operation. A non-negative value typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t ad9625_spi_write(struct ad9625_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);
/***************************************************************************//**
 * @brief This function sets up the AD9625 device by initializing it with the
 * provided parameters and configuring its registers. It must be called
 * before any other operations on the AD9625 device. The function
 * allocates memory for the device structure and initializes the SPI
 * interface. It checks the device's chip ID and PLL status to ensure
 * proper initialization. If the initialization fails at any step, the
 * function returns an error code and does not modify the device pointer.
 *
 * @param device A pointer to a pointer of type struct ad9625_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A struct ad9625_init_param containing initialization
 * parameters for the device, including SPI settings and
 * device-specific configurations. The caller retains
 * ownership of this structure.
 * @return Returns 0 on successful initialization. Returns a negative error code
 * if initialization fails, such as memory allocation failure or invalid
 * chip ID.
 ******************************************************************************/
int32_t ad9625_setup(struct ad9625_dev **device,
		     struct ad9625_init_param init_param);
/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources allocated
 * for an AD9625 device when it is no longer needed. This function should
 * be called to prevent resource leaks after the device has been
 * initialized and used. It handles the deallocation of memory and the
 * removal of the SPI descriptor associated with the device. Ensure that
 * the device pointer is valid and initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad9625_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The function will handle invalid pointers by
 * potentially causing undefined behavior.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * removal operation. A return value of 0 typically indicates success,
 * while a negative value indicates an error occurred during the SPI
 * removal process.
 ******************************************************************************/
int32_t ad9625_remove(struct ad9625_dev *dev);
/***************************************************************************//**
 * @brief This function sets the AD9625 device into a specified test mode by
 * writing to its control registers. It should be used when the device
 * needs to be tested in different modes for validation or diagnostic
 * purposes. The function assumes that the device has been properly
 * initialized and is ready for configuration. It does not perform any
 * validation on the test mode value, so the caller must ensure that the
 * provided test mode is valid according to the device's specifications.
 *
 * @param dev A pointer to an initialized ad9625_dev structure representing the
 * device. Must not be null, and the device should be properly set up
 * before calling this function.
 * @param test_mode An unsigned 32-bit integer representing the test mode to
 * set. Valid values are defined by the device's
 * specifications, such as AD9625_TEST_OFF,
 * AD9625_TEST_MID_SCALE, etc. The function does not validate
 * this parameter, so invalid values may lead to undefined
 * behavior.
 * @return Returns 0 on success, indicating that the test mode was set without
 * errors.
 ******************************************************************************/
int32_t ad9625_test(struct ad9625_dev *dev, uint32_t test_mode);

#endif
