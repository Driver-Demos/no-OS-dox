/***************************************************************************//**
* @file ad9265.h
* @brief Header file of AD9265 Driver.
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
#ifndef AD9265_H_
#define AD9265_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9265_REG_CHIP_PORT_CONF	0x00
#define AD9265_REG_CHIP_ID		0x01
#define AD9265_REG_CHIP_GRADE		0x02
#define AD9265_REG_CHAN_INDEX		0x05
#define AD9265_REG_TRANSFER		0xFF
#define AD9265_REG_MODES		0x08
#define AD9265_REG_TEST_IO		0x0D
#define AD9265_REG_ADC_INPUT		0x0F
#define AD9265_REG_OFFSET		0x10
#define AD9265_REG_OUTPUT_MODE		0x14
#define AD9265_REG_OUTPUT_ADJUST	0x15
#define AD9265_REG_OUTPUT_PHASE		0x16
#define AD9265_REG_OUTPUT_DELAY		0x17
#define AD9265_REG_VREF			0x18
#define AD9265_REG_ANALOG_INPUT		0x2C

/* ADC_REG_TRANSFER */
#define TRANSFER_SYNC			0x1

/* AD9265_REG_TEST_IO */
#define TESTMODE_OFF			0x0
#define TESTMODE_MIDSCALE_SHORT		0x1
#define TESTMODE_POS_FULLSCALE		0x2
#define TESTMODE_NEG_FULLSCALE		0x3
#define TESTMODE_ALT_CHECKERBOARD	0x4
#define TESTMODE_PN23_SEQ		0x5
#define TESTMODE_PN9_SEQ		0x6
#define TESTMODE_ONE_ZERO_TOGGLE	0x7

/* ADC_REG_OUTPUT_MODE */
#define OUTPUT_MODE_OFFSET_BINARY	0x0
#define OUTPUT_MODE_TWOS_COMPLEMENT	0x1
#define OUTPUT_MODE_GRAY_CODE		0x2

/* ADC_REG_OUTPUT_PHASE */
#define OUTPUT_EVEN_ODD_MODE_EN		0x20
#define INVERT_DCO_CLK			0x80

#define AD9265_CHIP_ID			0x64
#define AD9265_DEF_OUTPUT_MODE		0x40

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad9265_init_param` structure is used to initialize the AD9265
 * device, encapsulating both SPI interface parameters and specific
 * device settings. It includes fields for configuring the output mode,
 * data clock output, enabling the data clock, and setting the number of
 * lanes, which are essential for the proper setup and operation of the
 * AD9265 analog-to-digital converter.
 *
 * @param spi_init A structure containing the initialization parameters for the
 * SPI interface.
 * @param output_mode A uint8_t value representing the output mode of the
 * device.
 * @param dco A uint8_t value indicating the data clock output setting.
 * @param dco_en A uint8_t value that enables or disables the data clock output.
 * @param nb_lanes A uint8_t value specifying the number of lanes used.
 ******************************************************************************/
struct ad9265_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	uint8_t		output_mode;
	uint8_t		dco;			// data clock output
	uint8_t		dco_en;			// dco enable
	uint8_t		nb_lanes;		// number of lanes
};

/***************************************************************************//**
 * @brief The `ad9265_dev` structure is a simple data structure designed to
 * encapsulate the SPI communication descriptor for the AD9265 device. It
 * contains a single member, `spi_desc`, which is a pointer to a
 * `no_os_spi_desc` structure, facilitating the management of SPI
 * operations necessary for interacting with the AD9265 ADC device. This
 * structure is essential for initializing and managing the communication
 * interface with the AD9265, enabling various operations such as reading
 * and writing to the device's registers.
 *
 * @param spi_desc A pointer to a SPI descriptor used for SPI communication.
 ******************************************************************************/
struct ad9265_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief Use this function to configure the AD9265 device to operate in a
 * specific test mode. This is typically used for diagnostic or
 * calibration purposes. The function must be called with a valid device
 * structure and a test mode value. It writes the specified mode to the
 * device's test I/O register and synchronizes the transfer. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad9265_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param mode A uint8_t value representing the desired test mode. Valid values
 * are defined as macros (e.g., TESTMODE_OFF,
 * TESTMODE_MIDSCALE_SHORT). Invalid values may result in undefined
 * behavior.
 * @return Always returns 0, indicating success.
 ******************************************************************************/
int32_t ad9265_testmode_set(struct ad9265_dev *dev,
			    uint8_t mode);
/***************************************************************************//**
 * @brief This function sets up the AD9265 device by initializing the necessary
 * SPI communication and configuring the device with the specified
 * initialization parameters. It must be called before any other
 * operations on the AD9265 device to ensure proper setup. The function
 * checks the device's chip ID to verify correct communication and
 * configuration. If the setup is successful, the function returns a non-
 * negative value and provides a pointer to the initialized device
 * structure. If an error occurs, such as a memory allocation failure or
 * an incorrect chip ID, the function returns a negative error code.
 *
 * @param device A pointer to a pointer of type `struct ad9265_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A structure of type `struct ad9265_init_param` containing
 * the initialization parameters for the AD9265 device. This
 * includes SPI initialization parameters and device-specific
 * settings. The caller must provide valid initialization
 * data.
 * @param core A structure of type `struct axi_adc` representing the ADC core
 * configuration. This is used during the calibration process. The
 * caller must ensure this structure is properly initialized before
 * calling the function.
 * @return Returns 0 on successful initialization, or a negative error code if
 * an error occurs during setup.
 ******************************************************************************/
int32_t ad9265_setup(struct ad9265_dev **device,
		     struct ad9265_init_param init_param,
		     struct axi_adc core);
/***************************************************************************//**
 * @brief This function should be called to properly release all resources
 * associated with an AD9265 device when it is no longer needed. It
 * ensures that the SPI descriptor associated with the device is removed
 * and the memory allocated for the device structure is freed. This
 * function must be called after the device is no longer in use to
 * prevent resource leaks. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to an ad9265_dev structure representing the device to be
 * removed. This pointer must not be null, and the structure should
 * have been previously initialized and used with the AD9265 driver
 * functions. The function will handle freeing the memory associated
 * with this structure.
 * @return Returns an int32_t status code from the SPI removal operation, where
 * 0 typically indicates success and a negative value indicates an
 * error.
 ******************************************************************************/
int32_t ad9265_remove(struct ad9265_dev *dev);
/***************************************************************************//**
 * @brief This function is used to calibrate the AD9265 device, ensuring it
 * operates with optimal performance by adjusting its output delay and
 * phase settings. It should be called after the device has been properly
 * initialized and configured. The function performs a series of tests to
 * determine the best delay and phase settings, which are then applied to
 * the device. It handles both DCO (Data Clock Output) and non-DCO modes,
 * adjusting settings accordingly. The function returns an error code if
 * the calibration process encounters any issues, allowing the caller to
 * handle such cases appropriately.
 *
 * @param dev A pointer to an ad9265_dev structure representing the device to be
 * calibrated. Must not be null, and the device should be initialized
 * before calling this function.
 * @param init_param An ad9265_init_param structure containing initialization
 * parameters for the device, including output mode, DCO
 * settings, and the number of lanes. These parameters guide
 * the calibration process.
 * @param core An axi_adc structure representing the ADC core associated with
 * the device. This is used for reading and writing ADC registers
 * during calibration.
 * @return Returns 0 on successful calibration, or a negative error code if the
 * calibration fails.
 ******************************************************************************/
int32_t ad9265_calibrate(struct ad9265_dev *dev,
			 struct ad9265_init_param init_param,
			 struct axi_adc core);
/***************************************************************************//**
 * @brief This function configures the output mode of the AD9265 device by
 * writing the specified mode to the appropriate register. It should be
 * called when the output mode needs to be changed, typically after
 * device initialization. The function ensures that the test mode is
 * turned off and synchronizes the transfer. It is important to ensure
 * that the device structure is properly initialized before calling this
 * function. The function returns an error code if the SPI write
 * operations fail.
 *
 * @param dev A pointer to an initialized ad9265_dev structure representing the
 * device. Must not be null.
 * @param mode A uint8_t value representing the desired output mode. Valid
 * values are typically defined as macros, such as
 * OUTPUT_MODE_OFFSET_BINARY, OUTPUT_MODE_TWOS_COMPLEMENT, and
 * OUTPUT_MODE_GRAY_CODE.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad9265_outputmode_set(struct ad9265_dev *dev,
			      uint8_t mode);
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * of the AD9265 device using the SPI interface. It is typically called
 * when there is a need to configure or modify the settings of the AD9265
 * by writing to its registers. The function requires a valid device
 * structure that has been properly initialized and configured for SPI
 * communication. It is important to ensure that the register address and
 * data are within valid ranges as per the device's specifications. The
 * function returns an integer status code indicating the success or
 * failure of the SPI write operation.
 *
 * @param dev A pointer to an ad9265_dev structure representing the device. This
 * must be a valid, initialized device structure with an SPI
 * descriptor. The caller retains ownership and must ensure it is not
 * null.
 * @param reg_addr A 16-bit unsigned integer representing the register address
 * to which data will be written. The address must be within the
 * valid range of the device's register map.
 * @param reg_data An 8-bit unsigned integer representing the data to be written
 * to the specified register. The data must be formatted
 * according to the register's requirements.
 * @return Returns an int32_t status code from the SPI write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t ad9265_spi_write(struct ad9265_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);
/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9265 device using SPI communication. It is essential to ensure that
 * the device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. The read value is stored in the
 * provided memory location pointed to by reg_data. This function returns
 * an integer status code indicating the success or failure of the SPI
 * read operation.
 *
 * @param dev A pointer to an ad9265_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer representing the register address within the
 * device.
 * @param reg_data A pointer to an 8-bit unsigned integer where the read
 * register value will be stored. Must not be null.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error occurred during
 * the SPI read operation.
 ******************************************************************************/
int32_t ad9265_spi_read(struct ad9265_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);
#endif
