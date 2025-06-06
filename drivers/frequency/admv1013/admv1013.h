/***************************************************************************//**
 *   @file   admv1013.h
 *   @brief  Header file for admv1013 Driver.
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

#ifndef ADMV1013_H_
#define ADMV1013_H_

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

/* ADMV1013 Register Map */
#define ADMV1013_REG_SPI_CONTROL		0x00
#define ADMV1013_REG_ALARM			0x01
#define ADMV1013_REG_ALARM_MASKS		0x02
#define ADMV1013_REG_ENABLE			0x03
#define ADMV1013_REG_LO_AMP_I			0x05
#define ADMV1013_REG_LO_AMP_Q			0x06
#define ADMV1013_REG_OFFSET_ADJUST_I		0x07
#define ADMV1013_REG_OFFSET_ADJUST_Q		0x08
#define ADMV1013_REG_QUAD			0x09
#define ADMV1013_REG_VVA_TEMP_COMP		0x0A

/* ADMV1013_REG_SPI_CONTROL Map */
#define ADMV1013_PARITY_EN_MSK			NO_OS_BIT(15)
#define ADMV1013_SPI_SOFT_RESET_MSK		NO_OS_BIT(14)
#define ADMV1013_CHIP_ID_MSK			NO_OS_GENMASK(11, 4)
#define ADMV1013_CHIP_ID			0xA
#define ADMV1013_REVISION_ID_MSK		NO_OS_GENMASK(3, 0)

/* ADMV1013_REG_ALARM Map */
#define ADMV1013_PARITY_ERROR_MSK		NO_OS_BIT(15)
#define ADMV1013_TOO_FEW_ERRORS_MSK		NO_OS_BIT(14)
#define ADMV1013_TOO_MANY_ERRORS_MSK		NO_OS_BIT(13)
#define ADMV1013_ADDRESS_RANGE_ERROR_MSK	NO_OS_BIT(12)

/* ADMV1013_REG_ENABLE Map */
#define ADMV1013_VGA_PD_MSK			NO_OS_BIT(15)
#define ADMV1013_MIXER_PD_MSK			NO_OS_BIT(14)
#define ADMV1013_QUAD_PD_MSK			NO_OS_GENMASK(13, 11)
#define ADMV1013_BG_PD_MSK			NO_OS_BIT(10)
#define ADMV1013_MIXER_IF_EN_MSK		NO_OS_BIT(7)
#define ADMV1013_DET_EN_MSK			NO_OS_BIT(5)

/* ADMV1013_REG_LO_AMP Map */
#define ADMV1013_LOAMP_PH_ADJ_FINE_MSK		NO_OS_GENMASK(13, 7)
#define ADMV1013_MIXER_VGATE_MSK		NO_OS_GENMASK(6, 0)

/* ADMV1013_REG_OFFSET_ADJUST Map */
#define ADMV1013_MIXER_OFF_ADJ_P_MSK		NO_OS_GENMASK(15, 9)
#define ADMV1013_MIXER_OFF_ADJ_N_MSK		NO_OS_GENMASK(8, 2)

/* ADMV1013_REG_QUAD Map */
#define ADMV1013_QUAD_SE_MODE_MSK		NO_OS_GENMASK(9, 6)
#define ADMV1013_QUAD_FILTERS_MSK		NO_OS_GENMASK(3, 0)

/* ADMV1013_REG_VVA_TEMP_COMP Map */
#define ADMV1013_VVA_TEMP_COMP_MSK		NO_OS_GENMASK(15, 0)

/* Specifications */
#define ADMV1013_BUFF_SIZE_BYTES		3
#define ADMV1013_SPI_READ_CMD			NO_OS_BIT(7)
#define ADMV1013_SPI_WRITE_CMD			(0 << 7)

#define MIXER_GATE_0_to_1_8_V(x)		((2389 * x/ 1000000 + 8100) / 100)
#define MIXER_GATE_1_8_to_2_6_V(x)		((2375 * x/ 1000000 + 125) / 100)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `admv1013_input_mode` enumeration defines the possible input modes
 * for the ADMV1013 device, which can operate in either I/Q mode or
 * Intermediate Frequency (IF) mode. This enumeration is used to
 * configure the device's input mode, allowing it to switch between
 * processing I/Q signals or IF signals, depending on the application
 * requirements.
 *
 * @param ADMV1013_IQ_MODE Represents the I/Q mode for the ADMV1013 device.
 * @param ADMV1013_IF_MODE Represents the Intermediate Frequency (IF) mode for
 * the ADMV1013 device.
 ******************************************************************************/
enum admv1013_input_mode {
	ADMV1013_IQ_MODE,
	ADMV1013_IF_MODE
};

/***************************************************************************//**
 * @brief The `admv1013_quad_se_mode` enumeration defines the different modes
 * for switching between differential and single-ended configurations in
 * the ADMV1013 device. It includes three modes: positive single-ended,
 * negative single-ended, and differential, each represented by a
 * specific integer value. This enumeration is used to configure the mode
 * of operation for the device's quadrature section.
 *
 * @param ADMV1013_SE_MODE_POS Represents the positive single-ended mode with a
 * value of 6.
 * @param ADMV1013_SE_MODE_NEG Represents the negative single-ended mode with a
 * value of 9.
 * @param ADMV1013_SE_MODE_DIFF Represents the differential mode with a value of
 * 12.
 ******************************************************************************/
enum admv1013_quad_se_mode {
	ADMV1013_SE_MODE_POS = 6,
	ADMV1013_SE_MODE_NEG = 9,
	ADMV1013_SE_MODE_DIFF = 12
};

/***************************************************************************//**
 * @brief The `admv1013_quad_filters` enumeration defines a set of constants
 * representing different local oscillator (LO) frequency bands for the
 * ADMV1013 device. Each enumerator corresponds to a specific frequency
 * range, allowing the selection of the appropriate LO filter bandwidth
 * for the device's operation. This enumeration is used to configure the
 * LO filter settings in the ADMV1013 driver.
 *
 * @param LO_BAND_8_62_TO_10_25_GHZ Represents a local oscillator band with a
 * frequency range from 8.62 GHz to 10.25 GHz,
 * assigned the value 0.
 * @param LO_BAND_6_6_TO_9_2_GHZ Represents a local oscillator band with a
 * frequency range from 6.6 GHz to 9.2 GHz,
 * assigned the value 5.
 * @param LO_BAND_5_4_TO_8_GHZ Represents a local oscillator band with a
 * frequency range from 5.4 GHz to 8 GHz, assigned
 * the value 10.
 * @param LO_BAND_5_4_TO_7_GHZ Represents a local oscillator band with a
 * frequency range from 5.4 GHz to 7 GHz, assigned
 * the value 15.
 ******************************************************************************/
enum admv1013_quad_filters {
	LO_BAND_8_62_TO_10_25_GHZ = 0,
	LO_BAND_6_6_TO_9_2_GHZ = 5,
	LO_BAND_5_4_TO_8_GHZ = 10,
	LO_BAND_5_4_TO_7_GHZ = 15
};

/***************************************************************************//**
 * @brief The `admv1013_init_param` structure is used to define the
 * initialization parameters for the ADMV1013 device. It includes
 * settings for SPI communication, local oscillator frequency, input
 * mode, quad single-ended mode, detector enablement, and common-mode
 * voltage. This structure is essential for configuring the ADMV1013
 * device to operate according to specific application requirements.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param lo_in Specifies the local oscillator input frequency.
 * @param input_mode Defines the input mode, either I/Q or IF.
 * @param quad_se_mode Specifies the quad single-ended mode.
 * @param det_en Boolean flag to enable or disable the detector.
 * @param vcm_uv Specifies the common-mode voltage in microvolts.
 ******************************************************************************/
struct admv1013_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** Input Mode */
	enum admv1013_input_mode	input_mode;
	/** Quad SE Mode */
	enum admv1013_quad_se_mode	quad_se_mode;
	/** Detector Enable */
	bool				det_en;
	/** Common-Mode Voltage (uV) */
	unsigned int			vcm_uv;
};

/***************************************************************************//**
 * @brief The `admv1013_dev` structure is a device descriptor for the ADMV1013,
 * a microwave upconverter. It encapsulates the necessary parameters for
 * configuring and operating the device, including SPI communication
 * details, local oscillator frequency, input mode, quadrature mode,
 * detector enablement, and common-mode voltage. This structure is
 * essential for initializing and managing the ADMV1013 device in a
 * software application.
 *
 * @param spi_desc Pointer to the SPI descriptor used for communication.
 * @param lo_in Specifies the local oscillator input frequency.
 * @param input_mode Defines the input mode, either I/Q or IF mode.
 * @param quad_se_mode Specifies the quadrature single-ended mode.
 * @param det_en Boolean flag to enable or disable the detector.
 * @param vcm_uv Specifies the common-mode voltage in microvolts.
 ******************************************************************************/
struct admv1013_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** Input Mode */
	enum admv1013_input_mode	input_mode;
	/** Quad SE Mode */
	enum admv1013_quad_se_mode	quad_se_mode;
	/** Detector Enable */
	bool				det_en;
	/** Common-Mode Voltage (uV) */
	unsigned int			vcm_uv;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to write a 16-bit data value to a specific register
 * of the ADMV1013 device over the SPI interface. This function is
 * typically called when configuring or updating the device's settings.
 * Ensure that the device has been properly initialized and that the SPI
 * interface is correctly configured before calling this function. The
 * function returns an integer status code indicating the success or
 * failure of the SPI write operation.
 *
 * @param dev A pointer to an initialized 'admv1013_dev' structure representing
 * the device. Must not be null.
 * @param reg_addr The 8-bit address of the register to which data will be
 * written. Valid register addresses are defined by the device's
 * register map.
 * @param data The 16-bit data to be written to the specified register. The data
 * is formatted and sent over SPI according to the device's
 * protocol.
 * @return Returns an integer status code from the underlying SPI write
 * operation, where 0 typically indicates success and a negative value
 * indicates an error.
 ******************************************************************************/
int admv1013_spi_write(struct admv1013_dev *dev, uint8_t reg_addr,
		       uint16_t data);

/* ADMV1013 Register Update */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * ADMV1013 device by performing a read-modify-write operation over SPI.
 * It is useful when only certain bits of a register need to be changed
 * without affecting the others. The function first reads the current
 * value of the register, applies a mask to clear the bits to be updated,
 * and then sets them to the new value provided. It should be called when
 * the device is properly initialized and the SPI interface is
 * configured. The function returns an error code if the read or write
 * operation fails.
 *
 * @param dev A pointer to an initialized 'admv1013_dev' structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADMV1013 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits specified by the mask will be updated.
 * @return Returns 0 on success or a negative error code if the SPI read or
 * write operation fails.
 ******************************************************************************/
int admv1013_spi_update_bits(struct admv1013_dev *dev, uint8_t reg_addr,
			     uint16_t mask, uint16_t data);

/***************************************************************************//**
 * @brief Use this function to read data from a specific register of the
 * ADMV1013 device. It requires a valid device descriptor and a register
 * address to perform the read operation. The function will store the
 * read 16-bit data into the provided data pointer. Ensure that the
 * device has been properly initialized before calling this function. The
 * function returns an error code if the SPI communication fails,
 * otherwise it returns 0 on success.
 *
 * @param dev A pointer to an initialized admv1013_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The 8-bit address of the register to read from. Valid
 * register addresses are defined by the device's register map.
 * @param data A pointer to a uint16_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int admv1013_spi_read(struct admv1013_dev *dev, uint8_t reg_addr,
		      uint16_t *data);

/***************************************************************************//**
 * @brief This function configures the I and Q phase adjustments for the
 * ADMV1013 device, which is essential for fine-tuning the phase accuracy
 * in applications requiring precise signal processing. It must be called
 * with a valid device descriptor after the device has been initialized.
 * The function updates the phase settings by writing to the appropriate
 * registers via SPI. It returns an error code if the SPI communication
 * fails, ensuring that the caller can handle such cases appropriately.
 *
 * @param dev A pointer to an initialized `admv1013_dev` structure representing
 * the device. Must not be null.
 * @param i_phase A uint8_t value representing the desired I phase adjustment.
 * The value is masked and prepared for register writing.
 * @param q_phase A uint8_t value representing the desired Q phase adjustment.
 * The value is masked and prepared for register writing.
 * @return Returns 0 on success or a negative error code if the SPI update
 * operation fails.
 ******************************************************************************/
int admv1013_set_iq_phase(struct admv1013_dev *dev, uint8_t i_phase,
			  uint8_t q_phase);

/***************************************************************************//**
 * @brief Use this function to obtain the current I and Q phase values from the
 * ADMV1013 device. This function should be called when you need to read
 * the phase settings for diagnostic or configuration purposes. Ensure
 * that the device has been properly initialized before calling this
 * function. The function will attempt to read the phase values and store
 * them in the provided pointers. If the read operation fails, an error
 * code will be returned.
 *
 * @param dev A pointer to an initialized `admv1013_dev` structure representing
 * the device. Must not be null.
 * @param i_phase A pointer to a uint8_t where the I phase value will be stored.
 * Must not be null.
 * @param q_phase A pointer to a uint8_t where the Q phase value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int admv1013_get_iq_phase(struct admv1013_dev *dev, uint8_t *i_phase,
			  uint8_t *q_phase);

/***************************************************************************//**
 * @brief This function configures the I and Q offset values for the ADMV1013
 * device, which is essential for calibrating the device to correct any
 * DC offset imbalances in the I/Q signal paths. It should be called when
 * the device is initialized and ready for configuration. The function
 * requires valid offset values for both the positive and negative paths
 * of the I and Q channels. If the operation is successful, it returns 0;
 * otherwise, it returns a negative error code indicating the failure.
 *
 * @param dev A pointer to an initialized admv1013_dev structure representing
 * the device. Must not be null.
 * @param i_offset_p The positive offset value for the I channel. Valid range is
 * 0 to 127.
 * @param i_offset_n The negative offset value for the I channel. Valid range is
 * 0 to 127.
 * @param q_offset_p The positive offset value for the Q channel. Valid range is
 * 0 to 127.
 * @param q_offset_n The negative offset value for the Q channel. Valid range is
 * 0 to 127.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int admv1013_set_iq_offset(struct admv1013_dev *dev, uint8_t i_offset_p,
			   uint8_t i_offset_n, uint8_t q_offset_p,
			   uint8_t q_offset_n);

/***************************************************************************//**
 * @brief This function is used to obtain the I and Q offset values from an
 * ADMV1013 device. It should be called when the user needs to read the
 * current offset settings for both the I and Q channels. The function
 * requires a valid device descriptor and pointers to store the retrieved
 * offset values. It is important to ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized `admv1013_dev` structure representing
 * the device. Must not be null.
 * @param i_offset_p A pointer to a uint8_t where the positive I offset value
 * will be stored. Must not be null.
 * @param i_offset_n A pointer to a uint8_t where the negative I offset value
 * will be stored. Must not be null.
 * @param q_offset_p A pointer to a uint8_t where the positive Q offset value
 * will be stored. Must not be null.
 * @param q_offset_n A pointer to a uint8_t where the negative Q offset value
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int admv1013_get_iq_offset(struct admv1013_dev *dev, uint8_t *i_offset_p,
			   uint8_t *i_offset_n, uint8_t *q_offset_p,
			   uint8_t *q_offset_n);

/***************************************************************************//**
 * @brief This function initializes an ADMV1013 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function sets up the SPI communication,
 * configures the device registers, and performs a software reset. It
 * also verifies the device ID to ensure the correct device is being
 * initialized. If initialization fails at any step, the function will
 * return an error code and ensure that any allocated resources are
 * properly freed.
 *
 * @param device A pointer to a pointer of type struct admv1013_dev. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a struct admv1013_init_param containing the
 * initialization parameters. Must not be null and should be
 * properly populated with valid configuration data before
 * calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure or SPI communication error.
 ******************************************************************************/
int admv1013_init(struct admv1013_dev **device,
		  struct admv1013_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADMV1013 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that any allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks in
 * your application.
 *
 * @param dev A pointer to an initialized `admv1013_dev` structure representing
 * the device to be removed. Must not be null. The function will
 * handle invalid pointers by returning an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int admv1013_remove(struct admv1013_dev *dev);

#endif /* ADMV1013_H_ */
