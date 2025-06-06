/***************************************************************************//**
 * @file ad9680.h
 * @brief Header file of AD9680 Driver.
 * @author DBogdan (dragos.bogdan@analog.com)
 ********************************************************************************
 * Copyright 2014-2016(c) Analog Devices, Inc.
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
#ifndef AD9680_H_
#define AD9680_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"
#include "jesd204.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9680_REG_INTERFACE_CONF_A				0x000
#define AD9680_REG_INTERFACE_CONF_B				0x001
#define AD9680_REG_CHIP_ID_LOW					0x004
#define AD9680_REG_CHIP_ID_HIGH					0x005
#define AD9680_REG_DEVICE_INDEX					0x008
#define AD9680_REG_CHIP_DEC_RATIO				0x201
#define AD9680_REG_ADC_TEST_MODE				0x550
#define AD9680_REG_OUTPUT_MODE					0x561
#define AD9680_REG_LINK_CONTROL					0x571
#define AD9680_REG_JESD204B_LANE_RATE_CTRL			0x56e
#define AD9680_REG_JESD204B_PLL_LOCK_STATUS			0x56f
#define AD9680_REG_JESD204B_QUICK_CONFIG			0x570
#define AD9680_REG_JESD_LINK_CTRL1_REG    			0x571
#define AD9680_REG_JESD204B_MF_CTRL				0x58d
#define AD9680_REG_JESD204B_CSN_CONFIG				0x58f
#define AD9680_REG_JESD204B_SUBCLASS_CONFIG			0x590
#define AD9680_REG_JESD204B_LANE_SERD_OUT0_ASSIGN		0x5b2
#define AD9680_REG_JESD204B_LANE_SERD_OUT1_ASSIGN		0x5b3
#define AD9680_REG_JESD204B_LANE_SERD_OUT2_ASSIGN		0x5b5
#define AD9680_REG_JESD204B_LANE_SERD_OUT3_ASSIGN		0x5b6

#define AD9680_CHIP_ID						0x0C5
#define AD9680_TEST_OFF						0x000
#define AD9680_TEST_PN9						0x006
#define AD9680_TEST_PN23					0x005
#define AD9680_TEST_RAMP					0x00f
#define AD9680_FORMAT_2S_COMPLEMENT				0x001
#define AD9680_FORMAT_OFFSET_BINARY				0x000
#define AD9680_JESD_LINK_PDN     	     NO_OS_BIT(0)

#define AD9680_SYSREF_NONE 0	/* No SYSREF Support */
#define AD9680_SYSREF_ONESHOT 2	/* ONE-SHOT SYSREF */
#define AD9680_SYSREF_CONT 1	/* Continuous Sysref Synchronisation */
#define AD9680_SYSREF_MON 3	/* SYSREF monitor Mode */

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad9680_dev` structure is a compound data type used to represent
 * the state and configuration of an AD9680 device, which is a high-speed
 * analog-to-digital converter. It includes members for SPI
 * communication, JESD204 link configuration, and operational parameters
 * such as sampling frequency, decimation factor, and SYSREF mode. This
 * structure is essential for managing the device's setup and operation
 * in a digital signal processing system.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param jdev Pointer to a JESD204 device structure.
 * @param jesd204_link Structure representing the JESD204 link configuration.
 * @param sampling_frequency_hz Sampling frequency in hertz.
 * @param dcm Decimation factor for the device.
 * @param sysref_mode Mode of SYSREF operation for synchronization.
 ******************************************************************************/
struct ad9680_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;

	struct jesd204_dev *jdev;
	struct jesd204_link jesd204_link;

	unsigned long long sampling_frequency_hz;
	unsigned long dcm;

	unsigned long sysref_mode;
};

/***************************************************************************//**
 * @brief The `ad9680_init_param` structure is used to initialize the AD9680
 * device, encapsulating all necessary parameters for setting up the
 * device's SPI interface, JESD204 link, and operational settings such as
 * lane rate, sampling frequency, decimation factor, and SYSREF mode.
 * This structure is crucial for configuring the device to operate
 * correctly within a system, ensuring proper communication and data
 * processing.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param lane_rate_kbps Specifies the lane rate in kilobits per second for the
 * JESD204 link.
 * @param jesd204_link Contains the configuration parameters for the JESD204
 * link.
 * @param sampling_frequency_hz Defines the sampling frequency in hertz.
 * @param dcm Represents the decimation factor for the device.
 * @param sysref_mode Indicates the SYSREF mode of operation for
 * synchronization.
 ******************************************************************************/
struct ad9680_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	uint32_t	lane_rate_kbps;

	struct jesd204_link jesd204_link;
	unsigned long long sampling_frequency_hz;
	unsigned long dcm;
	unsigned long sysref_mode;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to read a specific register from the AD9680
 * device using the SPI interface. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. The read data is returned through a
 * pointer provided by the caller. This function is typically used when
 * there is a need to verify or retrieve the current configuration or
 * status of the AD9680 device.
 *
 * @param dev A pointer to an initialized ad9680_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address for the AD9680.
 * @param reg_data A pointer to a uint8_t where the read register data will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9680_spi_read(struct ad9680_dev *dev,
			uint16_t reg_addr,
			uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register on the AD9680 device using the SPI interface. It is typically
 * called when configuring the device or changing its settings. The
 * function requires a valid device structure that has been properly
 * initialized, a register address within the valid range, and the data
 * to be written. It returns an integer status code indicating the
 * success or failure of the operation.
 *
 * @param dev A pointer to an ad9680_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param reg_addr The 16-bit address of the register to which data will be
 * written. It should be within the valid range of register
 * addresses for the AD9680.
 * @param reg_data The 8-bit data value to be written to the specified register.
 * Any value within the range of an unsigned 8-bit integer is
 * valid.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9680_spi_write(struct ad9680_dev *dev,
			 uint16_t reg_addr,
			 uint8_t reg_data);

/***************************************************************************//**
 * @brief This function sets up the AD9680 device by initializing it with the
 * provided parameters and configuring its SPI interface. It must be
 * called before any other operations on the AD9680 device. The function
 * checks the device's chip ID to ensure it is correct and configures the
 * JESD204B interface based on the initialization parameters. It also
 * verifies the PLL lock status to ensure proper operation. If the setup
 * is successful, a device handle is returned through the provided
 * pointer. If any step fails, the function returns an error code and the
 * device handle is not valid.
 *
 * @param device A pointer to a pointer of type struct ad9680_dev. This is used
 * to return the initialized device handle. Must not be null.
 * @param init_param A pointer to a struct ad9680_init_param containing the
 * initialization parameters for the device. Must not be null
 * and should be properly initialized with valid SPI and
 * JESD204B settings.
 * @return Returns 0 on success, or -1 if an error occurs during initialization,
 * such as a memory allocation failure, incorrect chip ID, or PLL lock
 * failure.
 ******************************************************************************/
int32_t ad9680_setup(struct ad9680_dev **device,
		     const struct ad9680_init_param *init_param);

/* Initialize ad9680_dev, JESD FSM ON */
/***************************************************************************//**
 * @brief This function sets up the AD9680 device by initializing its SPI
 * interface and configuring the JESD204B interface according to the
 * provided initialization parameters. It must be called before any other
 * operations on the AD9680 device. The function checks the device's chip
 * ID to ensure it is correct and configures the JESD204B link settings,
 * including the lane rate and subclass configuration. It also verifies
 * the PLL lock status to ensure proper operation. The function allocates
 * memory for the device structure, which the caller must manage and
 * eventually free. If the initialization fails at any step, an error
 * code is returned, and the device pointer is not valid.
 *
 * @param device A pointer to a pointer of type struct ad9680_dev. This will be
 * allocated and initialized by the function. The caller is
 * responsible for freeing this memory. Must not be null.
 * @param init_param A pointer to a struct ad9680_init_param containing the
 * initialization parameters for the device, including SPI
 * settings and JESD204B configuration. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int32_t ad9680_setup_jesd_fsm(struct ad9680_dev **device,
			      const struct ad9680_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release and clean up resources allocated
 * for an AD9680 device when it is no longer needed. This function should
 * be called to prevent resource leaks after the device has been
 * initialized and used. It is important to ensure that the device
 * pointer is valid and that the device has been properly initialized
 * before calling this function. Failure to do so may result in undefined
 * behavior.
 *
 * @param dev A pointer to an ad9680_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The function will handle invalid pointers by
 * potentially causing undefined behavior.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t ad9680_remove(struct ad9680_dev *dev);

/***************************************************************************//**
 * @brief This function sets the AD9680 device to operate in a specified test
 * mode, which is useful for diagnostic and verification purposes. It
 * must be called with a valid device structure and a test mode constant
 * defined for the AD9680. The function writes the test mode to the
 * device and adjusts the output data format accordingly. It is important
 * to ensure that the device is properly initialized before calling this
 * function. The function does not handle invalid test mode values, so
 * the caller must ensure that the test mode is valid.
 *
 * @param dev A pointer to an initialized ad9680_dev structure representing the
 * device. Must not be null.
 * @param test_mode A uint32_t value representing the desired test mode. Valid
 * values are defined as constants, such as AD9680_TEST_OFF,
 * AD9680_TEST_PN9, etc. Invalid values may lead to undefined
 * behavior.
 * @return Returns 0 on success. No error handling is performed for invalid
 * input values.
 ******************************************************************************/
int32_t ad9680_test(struct ad9680_dev *dev,
		    uint32_t test_mode);

#endif
