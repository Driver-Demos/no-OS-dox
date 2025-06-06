/***************************************************************************//**
* @file ad9434.h
* @brief Header file of AD9434 Driver.
* @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
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
#ifndef AD9434_H_
#define AD9434_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9434_REG_CHIP_PORT_CONF	0x00
#define AD9434_REG_CHIP_ID		0x01
#define AD9434_REG_CHIP_GRADE		0x02
#define AD9434_REG_CHAN_INDEX		0x05
#define AD9434_REG_TRANSFER		0xFF
#define AD9434_REG_MODES		0x08
#define AD9434_REG_TEST_IO		0x0D
#define AD9434_REG_ADC_INPUT		0x0F
#define AD9434_REG_OFFSET		0x10
#define AD9434_REG_OUTPUT_MODE		0x14
#define AD9434_REG_OUTPUT_ADJUST	0x15
#define AD9434_REG_OUTPUT_PHASE		0x16
#define AD9434_REG_OUTPUT_DELAY		0x17
#define AD9434_REG_VREF			0x18
#define AD9434_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* AD9434_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ		0x6
#define TESTMODE_ONE_ZERO_TOGGLE	0x7
#define TESTMODE_USER_DEFINED		0x8

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

#define AD9434_CHIP_ID			0x6A
#define AD9434_DEF_OUTPUT_MODE		0x00

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad9434_dev` structure is a data structure used to represent an
 * instance of the AD9434 device, specifically managing its SPI
 * communication interface. It contains a single member, `spi_desc`,
 * which is a pointer to a `no_os_spi_desc` structure, encapsulating the
 * details necessary for SPI communication. This structure is essential
 * for interfacing with the AD9434 device, allowing for operations such
 * as reading and writing registers via SPI.
 *
 * @param spi_desc A pointer to a no_os_spi_desc structure, which describes the
 * SPI interface for the device.
 ******************************************************************************/
struct ad9434_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
};

/***************************************************************************//**
 * @brief The `ad9434_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the AD9434 device,
 * specifically focusing on the SPI interface configuration. This
 * structure is essential for initializing the device with the correct
 * SPI settings, ensuring proper communication and operation of the
 * AD9434 ADC.
 *
 * @param spi_init Contains the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad9434_init_param {
	/* SPI */
	struct no_os_spi_init_param spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9434 device using SPI communication. It requires a valid device
 * structure that has been properly initialized and configured for SPI
 * operations. The function reads the register value and stores it in the
 * provided memory location. It is essential to ensure that the device is
 * correctly set up before calling this function to avoid communication
 * errors. The function returns an integer status code indicating the
 * success or failure of the operation.
 *
 * @param dev A pointer to an ad9434_dev structure representing the device. This
 * must be a valid, initialized device structure with a configured
 * SPI descriptor. Must not be null.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer representing the register address within the
 * device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. This must be a valid memory location provided by the
 * caller. Must not be null.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error occurred during
 * the SPI read operation.
 ******************************************************************************/
int32_t ad9434_spi_read(struct ad9434_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);
/***************************************************************************//**
 * @brief Use this function to write a single byte of data to a specific
 * register on the AD9434 device. This function requires a valid device
 * structure that has been properly initialized and configured for SPI
 * communication. It is typically used to configure or modify the
 * settings of the AD9434 by writing to its control registers. Ensure
 * that the device is correctly set up before calling this function to
 * avoid communication errors.
 *
 * @param dev A pointer to an ad9434_dev structure representing the device. This
 * must be initialized and must not be null. The caller retains
 * ownership.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. Valid register addresses are defined by the AD9434
 * device specifications.
 * @param reg_data The 8-bit data to be written to the specified register. This
 * is the value that will be stored in the register.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * write operation. A non-negative value typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t ad9434_spi_write(struct ad9434_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);
/***************************************************************************//**
 * @brief Use this function to configure the AD9434 device into a specific test
 * mode, which is useful for diagnostic and testing purposes. The
 * function must be called with a valid device structure and a test mode
 * value. It is important to ensure that the device has been properly
 * initialized before calling this function. The function writes to the
 * device's registers to set the desired test mode and synchronizes the
 * transfer. If an invalid mode is provided, the behavior is undefined.
 *
 * @param dev A pointer to an initialized ad9434_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param mode An 8-bit unsigned integer representing the test mode to set.
 * Valid values are defined as macros (e.g., TESTMODE_OFF,
 * TESTMODE_MIDSCALE_SHORT, etc.). Providing an invalid mode results
 * in undefined behavior.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9434_testmode_set(struct ad9434_dev *dev,
			    uint8_t mode);
/***************************************************************************//**
 * @brief This function sets up the AD9434 device by initializing the necessary
 * SPI communication and configuring the device with default settings. It
 * must be called before any other operations on the AD9434 device. The
 * function allocates memory for the device structure and checks the
 * device's chip ID to ensure it is correct. If the initialization is
 * successful, the device pointer is updated to point to the newly
 * created device structure. If any step fails, the function returns an
 * error code and the device pointer is not updated.
 *
 * @param device A pointer to a pointer of type `struct ad9434_dev`. This will
 * be updated to point to the initialized device structure if the
 * function succeeds. Must not be null.
 * @param init_param A structure of type `struct ad9434_init_param` containing
 * the initialization parameters for the SPI interface. The
 * structure must be properly initialized before calling this
 * function.
 * @return Returns 0 on success, or a negative error code if initialization
 * fails. The device pointer is only updated on success.
 ******************************************************************************/
int32_t ad9434_setup(struct ad9434_dev **device,
		     struct ad9434_init_param init_param);
/***************************************************************************//**
 * @brief This function configures the output mode of the AD9434 device by
 * writing the specified mode to the appropriate register. It should be
 * called when the output mode needs to be changed, typically after
 * device initialization. The function ensures that the test mode is
 * turned off and synchronizes the transfer. It is important to ensure
 * that the device is properly initialized before calling this function
 * to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized ad9434_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the desired output mode. Valid
 * values are typically defined as macros, such as
 * OUTPUT_MODE_OFFSET_BINARY, OUTPUT_MODE_TWOS_COMPLEMENT, or
 * OUTPUT_MODE_GRAY_CODE. Invalid values may result in undefined
 * behavior.
 * @return Returns an int32_t indicating success (0) or a negative error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad9434_outputmode_set(struct ad9434_dev *dev,
			      uint8_t mode);
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD9434 device when it is no longer needed. It
 * ensures that the SPI descriptor associated with the device is removed
 * and the memory allocated for the device structure is freed. This
 * function must be called after the device is no longer in use to
 * prevent resource leaks. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to an ad9434_dev structure representing the device to be
 * removed. This pointer must not be null, and the structure should
 * have been previously initialized and used with the AD9434 driver
 * functions. Passing a null pointer will result in undefined
 * behavior.
 * @return Returns an int32_t status code from the SPI removal operation, where
 * 0 typically indicates success and a negative value indicates an
 * error.
 ******************************************************************************/
int32_t ad9434_remove(struct ad9434_dev *dev);
#endif
