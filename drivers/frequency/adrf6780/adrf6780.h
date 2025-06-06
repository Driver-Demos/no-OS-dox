/***************************************************************************//**
 *   @file   adrf6780.h
 *   @brief  Header file for adrf6780 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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

#ifndef ADRF6780_H_
#define ADRF6780_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* ADRF6780 Register Map */
#define ADRF6780_REG_CONTROL			0x00
#define ADRF6780_REG_ALARM_READBACK		0x01
#define ADRF6780_REG_ALARM_MASKS		0x02
#define ADRF6780_REG_ENABLE			0x03
#define ADRF6780_REG_LINEARIZE			0x04
#define ADRF6780_REG_LO_PATH			0x05
#define ADRF6780_REG_ADC_CONTROL		0x06
#define ADRF6780_REG_ADC_OUTPUT			0x0C

/* ADRF6780_REG_CONTROL Map */
#define ADRF6780_PARITY_EN_MSK			NO_OS_BIT(15)
#define ADRF6780_SOFT_RESET_MSK			NO_OS_BIT(14)
#define ADRF6780_CHIP_ID_MSK			NO_OS_GENMASK(11, 4)
#define ADRF6780_CHIP_ID			0xA
#define ADRF6780_CHIP_REVISION_MSK		NO_OS_GENMASK(3, 0)

/* ADRF6780_REG_ALARM_READBACK Map */
#define ADRF6780_PARITY_ERROR_MSK		NO_OS_BIT(15)
#define ADRF6780_TOO_FEW_ERRORS_MSK		NO_OS_BIT(14)
#define ADRF6780_TOO_MANY_ERRORS_MSK		NO_OS_BIT(13)
#define ADRF6780_ADDRESS_RANGE_ERROR_MSK	NO_OS_BIT(12)

/* ADRF6780_REG_ENABLE Map */
#define ADRF6780_VGA_BUFFER_EN_MSK		NO_OS_BIT(8)
#define ADRF6780_DETECTOR_EN_MSK		NO_OS_BIT(7)
#define ADRF6780_LO_BUFFER_EN_MSK		NO_OS_BIT(6)
#define ADRF6780_IF_MODE_EN_MSK			NO_OS_BIT(5)
#define ADRF6780_IQ_MODE_EN_MSK			NO_OS_BIT(4)
#define ADRF6780_LO_X2_EN_MSK			NO_OS_BIT(3)
#define ADRF6780_LO_PPF_EN_MSK			NO_OS_BIT(2)
#define ADRF6780_LO_EN_MSK			NO_OS_BIT(1)
#define ADRF6780_UC_BIAS_EN_MSK			NO_OS_BIT(0)

/* ADRF6780_REG_LINEARIZE Map */
#define ADRF6780_RDAC_LINEARIZE_MSK		NO_OS_GENMASK(7, 0)

/* ADRF6780_REG_LO_PATH Map */
#define ADRF6780_LO_SIDEBAND_MSK		NO_OS_BIT(10)
#define ADRF6780_Q_PATH_PHASE_ACCURACY_MSK	NO_OS_GENMASK(7, 4)
#define ADRF6780_I_PATH_PHASE_ACCURACY_MSK	NO_OS_GENMASK(3, 0)

/* ADRF6780_REG_ADC_CONTROL Map */
#define ADRF6780_VDET_OUTPUT_SELECT_MSK		NO_OS_BIT(3)
#define ADRF6780_ADC_START_MSK			NO_OS_BIT(2)
#define ADRF6780_ADC_EN_MSK			NO_OS_BIT(1)
#define ADRF6780_ADC_CLOCK_EN_MSK		NO_OS_BIT(0)

/* ADRF6780_REG_ADC_OUTPUT Map */
#define ADRF6780_ADC_STATUS_MSK			NO_OS_BIT(8)
#define ADRF6780_ADC_VALUE_MSK			NO_OS_GENMASK(7, 0)

/* Specifications */
#define ADRF6780_BUFF_SIZE_BYTES		3
#define ADRF6780_SPI_READ_CMD			NO_OS_BIT(7)
#define ADRF6780_SPI_WRITE_CMD			(0 << 7)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adrf6780_init_param` structure is used to define the
 * initialization parameters for the ADRF6780 device. It includes various
 * configuration options such as SPI initialization parameters, LO input
 * frequency, and several enable flags for different features like VGA
 * buffer, LO buffer, IF mode, LO x2, LO, UC bias, LO sideband, and VDET
 * output select. This structure is essential for setting up the ADRF6780
 * device with the desired operational parameters.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param lo_in Specifies the LO input frequency.
 * @param vga_buff_en Enables or disables the VGA buffer.
 * @param lo_buff_en Enables or disables the LO buffer.
 * @param if_mode_en Enables or disables the IF mode.
 * @param lo_x2_en Enables or disables the LO x2 feature.
 * @param lo_en Enables or disables the LO.
 * @param uc_bias_en Enables or disables the UC bias.
 * @param lo_sideband Specifies the LO sideband setting.
 * @param vdet_out_en Enables or disables the VDET output select.
 ******************************************************************************/
struct adrf6780_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** VGA Buffer Enable */
	bool				vga_buff_en;
	/** LO Buffer Enable */
	bool				lo_buff_en;
	/** IF Mode Enable */
	bool				if_mode_en;
	/** LO x2 Enable */
	bool				lo_x2_en;
	/** LO Enable */
	bool				lo_en;
	/** UC Bias Enable */
	bool				uc_bias_en;
	/** LO Sideband */
	bool				lo_sideband;
	/** VDET Output Select Enable */
	bool				vdet_out_en;
};

/***************************************************************************//**
 * @brief The `adrf6780_dev` structure is a device descriptor for the ADRF6780,
 * a wideband microwave upconverter. It contains configuration parameters
 * and state information necessary for controlling the device, including
 * SPI communication details, local oscillator settings, and various
 * enable flags for different operational modes and features. This
 * structure is essential for initializing and managing the ADRF6780
 * device in a software application.
 *
 * @param spi_desc Pointer to the SPI descriptor used for communication.
 * @param lo_in Specifies the local oscillator input frequency.
 * @param vga_buff_en Boolean flag to enable or disable the VGA buffer.
 * @param lo_buff_en Boolean flag to enable or disable the LO buffer.
 * @param if_mode_en Boolean flag to enable or disable the IF mode.
 * @param lo_x2_en Boolean flag to enable or disable the LO x2 feature.
 * @param lo_en Boolean flag to enable or disable the LO.
 * @param uc_bias_en Boolean flag to enable or disable the UC bias.
 * @param lo_sideband Boolean flag to select the LO sideband.
 * @param vdet_out_en Boolean flag to enable or disable the VDET output select.
 ******************************************************************************/
struct adrf6780_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** VGA Buffer Enable */
	bool				vga_buff_en;
	/** LO Buffer Enable */
	bool				lo_buff_en;
	/** IF Mode Enable */
	bool				if_mode_en;
	/** LO x2 Enable */
	bool				lo_x2_en;
	/** LO Enable */
	bool				lo_en;
	/** UC Bias Enable */
	bool				uc_bias_en;
	/** LO Sideband */
	bool				lo_sideband;
	/** VDET Output Select Enable */
	bool				vdet_out_en;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to write a 16-bit data value to a specific register
 * of the ADRF6780 device using SPI communication. This function is
 * typically called when you need to configure or update the settings of
 * the ADRF6780 device. Ensure that the device has been properly
 * initialized and that the SPI interface is correctly set up before
 * calling this function. The function returns an integer status code
 * indicating the success or failure of the write operation.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null, and the SPI descriptor within must
 * be valid.
 * @param reg_addr The 8-bit address of the register to which data will be
 * written. Valid register addresses are defined by the device's
 * register map.
 * @param data The 16-bit data to be written to the specified register. The data
 * is formatted and sent over SPI according to the device's
 * protocol.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int adrf6780_spi_write(struct adrf6780_dev *dev, uint8_t reg_addr,
		       uint16_t data);

/* ADRF6780 Register Update */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * ADRF6780 device by performing a read-modify-write operation over SPI.
 * It is useful when only certain bits of a register need to be changed
 * without affecting the other bits. The function first reads the current
 * value of the register, applies the mask to clear the bits to be
 * updated, and then sets the new bits as specified by the data
 * parameter. It should be called when the device is properly initialized
 * and the SPI interface is configured. The function returns an error
 * code if the read or write operation fails.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADRF6780 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param data The new bit values to be written to the register, aligned with
 * the mask. Only the bits corresponding to the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code if the SPI read or
 * write operation fails.
 ******************************************************************************/
int adrf6780_spi_update_bits(struct adrf6780_dev *dev, uint8_t reg_addr,
			     uint16_t mask, uint16_t data);

/***************************************************************************//**
 * @brief Use this function to read a 16-bit data value from a specific register
 * of the ADRF6780 device using SPI communication. It is essential to
 * ensure that the device has been properly initialized and that the SPI
 * interface is correctly configured before calling this function. The
 * function requires a valid device descriptor and a register address to
 * read from. The read data is returned through a pointer provided by the
 * caller. This function returns an error code if the SPI communication
 * fails.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The 8-bit address of the register to read from. Valid
 * register addresses are defined in the ADRF6780 register map.
 * @param data A pointer to a uint16_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adrf6780_spi_read(struct adrf6780_dev *dev, uint8_t reg_addr,
		      uint16_t *data);

/***************************************************************************//**
 * @brief This function configures the RDAC linearization setting of the
 * ADRF6780 device, which is used to improve intermodulation distortion
 * (IMD) performance. It should be called when the device is initialized
 * and ready for configuration. The function writes the specified
 * linearization value to the appropriate register on the device. Ensure
 * that the device pointer is valid and the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param rdac_lin An 8-bit unsigned integer representing the RDAC linearization
 * value to be set. Valid range is 0 to 255.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * that the write operation to the device failed.
 ******************************************************************************/
int adrf6780_set_rdac_linearize(struct adrf6780_dev *dev, uint8_t rdac_lin);

/***************************************************************************//**
 * @brief This function is used to obtain the current RDAC linearization setting
 * from an ADRF6780 device. It should be called when you need to read the
 * linearization value for purposes such as diagnostics or configuration
 * verification. Ensure that the device has been properly initialized
 * before calling this function. The function reads the relevant register
 * and extracts the linearization value, which is then stored in the
 * provided output parameter. If the read operation fails, an error code
 * is returned.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param rdac_lin A pointer to a uint8_t where the RDAC linearization value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adrf6780_get_rdac_linearize(struct adrf6780_dev *dev, uint8_t *rdac_lin);

/***************************************************************************//**
 * @brief This function configures the I and Q path phase accuracy settings for
 * the ADRF6780 device. It should be called when you need to adjust the
 * phase accuracy of the I/Q paths, typically after initializing the
 * device. The function updates the relevant registers with the provided
 * I and Q data values. Ensure that the device is properly initialized
 * before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param i_data A uint8_t value representing the desired phase accuracy for the
 * I path. Valid range is 0 to 15.
 * @param q_data A uint8_t value representing the desired phase accuracy for the
 * Q path. Valid range is 0 to 15.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int adrf6780_set_cdac_iq_phase_accuracy(struct adrf6780_dev *dev,
					uint8_t i_data, uint8_t q_data);

/***************************************************************************//**
 * @brief Use this function to obtain the current I and Q path phase accuracy
 * values from the ADRF6780 device. This function is typically called
 * when you need to monitor or verify the phase accuracy settings of the
 * device. Ensure that the device has been properly initialized and is in
 * a state where it can communicate over SPI before calling this
 * function. The function will return an error code if the SPI read
 * operation fails.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param i_data A pointer to a uint8_t variable where the I path phase accuracy
 * value will be stored. Must not be null.
 * @param q_data A pointer to a uint8_t variable where the Q path phase accuracy
 * value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int adrf6780_get_cdac_iq_phase_accuracy(struct adrf6780_dev *dev,
					uint8_t *i_data, uint8_t *q_data);

/***************************************************************************//**
 * @brief Use this function to read the raw ADC data from an ADRF6780 device. It
 * should be called when you need to obtain the current ADC output value
 * from the device. The function requires a valid device descriptor and a
 * pointer to store the ADC data. It handles enabling the ADC, starting
 * the conversion, and reading the result. Ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the ADC data is not ready or if any SPI
 * communication fails.
 *
 * @param dev A pointer to an initialized adrf6780_dev structure representing
 * the device. Must not be null.
 * @param data A pointer to a uint16_t variable where the ADC data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the ADC data is not
 * ready or if an SPI communication error occurs.
 ******************************************************************************/
int adrf6780_read_adc_raw(struct adrf6780_dev *dev, uint16_t *data);

/* Software Reset */
/***************************************************************************//**
 * @brief Use this function to reset the ADRF6780 device to its default state
 * via a software command. This function is typically called when the
 * device needs to be reinitialized or when a known starting state is
 * required. It must be called with a valid device descriptor that has
 * been properly initialized. The function will return an error code if
 * the reset operation fails, which can occur due to communication issues
 * with the device.
 *
 * @param dev A pointer to an initialized `adrf6780_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly configured before calling this function.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int adrf6780_soft_reset(struct adrf6780_dev *dev);

/***************************************************************************//**
 * @brief This function initializes the ADRF6780 device using the provided
 * initialization parameters. It sets up the device's SPI communication,
 * performs a software reset, and configures various operational settings
 * based on the input parameters. This function must be called before any
 * other operations on the ADRF6780 device. It allocates memory for the
 * device structure and initializes it with the given parameters. If
 * initialization fails at any step, it cleans up and returns an error
 * code.
 *
 * @param device A pointer to a pointer of type `struct adrf6780_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct adrf6780_init_param` containing the
 * initialization parameters. This includes SPI initialization
 * parameters and various enable flags. The pointer must not
 * be null, and the structure must be properly populated
 * before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error. The `device`
 * pointer is set to a valid device structure on success, or remains
 * unchanged on failure.
 ******************************************************************************/
int adrf6780_init(struct adrf6780_dev **device,
		  struct adrf6780_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADRF6780 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated SPI resources are also released. It is
 * important to call this function to prevent resource leaks in your
 * application.
 *
 * @param dev A pointer to an adrf6780_dev structure representing the device to
 * be removed. Must not be null. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns 0 on successful deallocation of resources. If an error occurs
 * during the SPI resource removal, a non-zero error code is returned.
 ******************************************************************************/
int adrf6780_remove(struct adrf6780_dev *dev);

#endif /* ADRF6780_H_ */
