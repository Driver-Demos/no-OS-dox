/***************************************************************************//**
 * @file ad9656.h
 * @brief Header file of AD9656 Driver.
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
#ifndef AD9656_H_
#define AD9656_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9656_SPI_CONFIG				        0x000
#define AD9656_REG_CHIP_ID					0x001
#define AD9656_REG_DEVICE_INDEX					0x005
#define AD9656_REG_ADC_TEST_MODE				0x00D
#define AD9656_REG_OUTPUT_MODE					0x014
#define AD9656_REG_USER_TEST_PATTERN_1_LSB			0x19
#define AD9656_REG_USER_TEST_PATTERN_1_MSB      		0x1A
#define AD9656_REG_USER_TEST_PATTERN_2_LSB 			0x1B
#define AD9656_REG_USER_TEST_PATTERN_2_MSB			0x1C
#define AD9656_REG_LINK_CONTROL					0x05F
#define AD9656_REG_JESD204B_LANE_RATE_CTRL			0x021
#define AD9656_REG_JESD204B_PLL_LOCK_STATUS			0x00A
#define AD9656_REG_JESD204B_QUICK_CONFIG			0x05E
#define AD9656_REG_JESD204B_SCR_L                               0x06E
#define AD9656_REG_JESD204B_MF_CTRL				0x070
#define AD9656_REG_JESD204B_M_CTRL                              0x071
#define AD9656_REG_JESD204B_CSN_CONFIG				0x072
#define AD9656_REG_JESD204B_SUBCLASS_CONFIG		        0x073
#define AD9656_REG_JESD204B_LANE_SERD_OUT1_OUT0_ASSIGN		0x082
#define AD9656_REG_JESD204B_LANE_SERD_OUT3_OUT2_ASSIGN		0x083

#define AD9656_CHIP_ID						0x0C0
#define AD9656_TEST_OFF						0x000
#define AD9656_TEST_PN9						0x006
#define AD9656_TEST_PN23					0x005
#define AD9656_TEST_USER_INPUT					0x048
#define AD9656_FORMAT_2S_COMPLEMENT				0x001
#define AD9656_FORMAT_OFFSET_BINARY				0x000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad9656_dev` structure is a simple data structure used to
 * represent an instance of the AD9656 device, specifically focusing on
 * its SPI communication interface. It contains a single member,
 * `spi_desc`, which is a pointer to a `no_os_spi_desc` structure,
 * facilitating the management and configuration of SPI communication for
 * the AD9656 device.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, representing the SPI
 * descriptor for the device.
 ******************************************************************************/
struct ad9656_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
};

/***************************************************************************//**
 * @brief The `ad9656_init_param` structure is used to initialize the AD9656
 * device, encapsulating the necessary parameters for setting up the SPI
 * interface and configuring the device's lane rate. This structure is
 * essential for the proper configuration and operation of the AD9656,
 * ensuring that the device communicates correctly over the SPI bus and
 * operates at the desired data rate.
 *
 * @param spi_init Contains the initialization parameters for the SPI interface.
 * @param lane_rate_kbps Specifies the lane rate in kilobits per second for the
 * device.
 ******************************************************************************/
struct ad9656_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	uint32_t	lane_rate_kbps;
};

/***************************************************************************//**
 * @brief The `ad9656_user_input_test_pattern` structure is designed to hold two
 * user-defined test patterns, each represented as a 16-bit unsigned
 * integer. These patterns are used in testing the AD9656 device,
 * allowing users to input specific patterns for validation or diagnostic
 * purposes. The structure provides a simple way to encapsulate these two
 * test patterns for use in functions that require user-defined test
 * data.
 *
 * @param user_test_pattern1 A 16-bit unsigned integer representing the first
 * user input test pattern.
 * @param user_test_pattern2 A 16-bit unsigned integer representing the second
 * user input test pattern.
 ******************************************************************************/
struct ad9656_user_input_test_pattern {
	/* User input test pattern 1 */
	uint16_t user_test_pattern1;
	/* User input test pattern 2 */
	uint16_t user_test_pattern2;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function retrieves the value of a specified register from the
 * AD9656 device using SPI communication. It is essential to call this
 * function with a valid device structure that has been properly
 * initialized. The function is useful for obtaining configuration or
 * status information from the device. It is important to ensure that the
 * provided register address is within the valid range of the device's
 * register map. The function will return an error code if the SPI
 * communication fails.
 *
 * @param dev A pointer to an initialized ad9656_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address within the device's register map.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int32_t ad9656_reg_read(struct ad9656_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register of the AD9656 device via SPI communication. It is typically
 * called when configuring the device or updating its settings. The
 * function requires a valid device structure that has been properly
 * initialized and a valid register address within the device's
 * addressable range. The caller must ensure that the device is ready for
 * communication and that the register address is correct. The function
 * returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized ad9656_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to write to. Must be a
 * valid register address for the AD9656.
 * @param reg_data The 8-bit data to write to the specified register.
 * @return Returns an int32_t error code, with 0 indicating success and a
 * negative value indicating an error in SPI communication.
 ******************************************************************************/
int32_t ad9656_reg_write(struct ad9656_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);

/***************************************************************************//**
 * @brief This function initializes the AD9656 device and configures it
 * according to the specified initialization parameters. It must be
 * called before any other operations on the device. The function sets up
 * the SPI communication, performs a device reset, and configures various
 * JESD204B settings. It also checks the device's chip ID and PLL lock
 * status to ensure proper initialization. If any step fails, the
 * function returns an error code and ensures that resources are properly
 * released.
 *
 * @param device A pointer to a pointer of type `struct ad9656_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure upon successful initialization.
 * @param init_param A pointer to a constant `struct ad9656_init_param`
 * containing the initialization parameters, including SPI
 * settings and lane rate. This must not be null, and the
 * structure must be properly initialized before calling the
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -ENOMEM for
 * memory allocation failure, -ENXIO for invalid chip ID, -ENOLCK for
 * PLL lock failure).
 ******************************************************************************/
int32_t ad9656_setup(struct ad9656_dev **device,
		     const struct ad9656_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to properly release and clean up resources
 * associated with an AD9656 device instance. It should be called when
 * the device is no longer needed to ensure that all allocated resources
 * are freed and any open connections are closed. This function must be
 * called after the device has been initialized and used, and it is
 * important to ensure that the `dev` parameter is valid and not null
 * before calling this function. Failure to call this function may result
 * in resource leaks.
 *
 * @param dev A pointer to an `ad9656_dev` structure representing the device to
 * be removed. This pointer must not be null, and it is expected that
 * the device has been previously initialized. The function will
 * handle invalid pointers by not performing any operations, but it
 * is the caller's responsibility to ensure the pointer is valid.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a non-zero
 * value indicates an error occurred during the removal process.
 ******************************************************************************/
int32_t ad9656_remove(struct ad9656_dev *dev);

/***************************************************************************//**
 * @brief Use this function to set the AD9656 device into a specific JESD204
 * test mode, which is useful for testing and validation purposes. The
 * function writes the desired test mode to the device and adjusts the
 * output data format accordingly. It must be called with a valid device
 * structure and a supported test mode. The function returns an error
 * code if the operation fails, ensuring that the caller can handle such
 * cases appropriately.
 *
 * @param dev A pointer to an initialized ad9656_dev structure representing the
 * device. Must not be null, and the device should be properly set up
 * before calling this function.
 * @param test_mode A uint32_t value representing the desired test mode. Valid
 * values include AD9656_TEST_OFF, AD9656_TEST_PN9,
 * AD9656_TEST_PN23, and AD9656_TEST_USER_INPUT. Invalid values
 * may result in undefined behavior.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered during the operation.
 ******************************************************************************/
int32_t ad9656_JESD204_test(struct ad9656_dev *dev,
			    uint32_t test_mode);

/***************************************************************************//**
 * @brief This function sets the AD9656 device to output a user-defined test
 * pattern by writing the specified test pattern values to the
 * appropriate registers. It should be used when a custom test pattern is
 * needed for testing or validation purposes. The function requires a
 * valid device structure and a test mode to be specified. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function will return an error code if any
 * of the register writes fail.
 *
 * @param dev A pointer to an initialized ad9656_dev structure representing the
 * device. Must not be null.
 * @param test_mode A 32-bit unsigned integer specifying the test mode to be
 * set. Must be a valid test mode supported by the device.
 * @param user_input_test_pattern A structure containing two 16-bit unsigned
 * integers representing the user-defined test
 * patterns. These values will be written to the
 * device's registers.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating a failure in writing to the device registers.
 ******************************************************************************/
int32_t ad9656_user_input_test(struct ad9656_dev *dev, uint32_t test_mode,
			       struct ad9656_user_input_test_pattern user_input_test_pattern);

#endif
