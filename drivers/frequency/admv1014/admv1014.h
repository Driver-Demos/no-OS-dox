/***************************************************************************//**
 *   @file   admv1014.h
 *   @brief  Header file for admv1014 Driver.
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

#ifndef ADMV1014_H_
#define ADMV1014_H_

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
/* ADMV1014 Register Map */
#define ADMV1014_REG_SPI_CONTROL		0x00
#define ADMV1014_REG_ALARM			0x01
#define ADMV1014_REG_ALARM_MASKS		0x02
#define ADMV1014_REG_ENABLE			0x03
#define ADMV1014_REG_QUAD			0x04
#define ADMV1014_REG_LO_AMP_PHASE_ADJUST1	0x05
#define ADMV1014_REG_MIXER			0x07
#define ADMV1014_REG_IF_AMP			0x08
#define ADMV1014_REG_IF_AMP_BB_AMP		0x09
#define ADMV1014_REG_BB_AMP_AGC			0x0A
#define ADMV1014_REG_VVA_TEMP_COMP		0x0B

/* ADMV1014_REG_SPI_CONTROL Map */
#define ADMV1014_PARITY_EN_MSK			NO_OS_BIT(15)
#define ADMV1014_SPI_SOFT_RESET_MSK		NO_OS_BIT(14)
#define ADMV1014_CHIP_ID_MSK			NO_OS_GENMASK(11, 4)
#define ADMV1014_CHIP_ID			0x9
#define ADMV1014_REVISION_ID_MSK		NO_OS_GENMASK(3, 0)

/* ADMV1014_REG_ALARM Map */
#define ADMV1014_PARITY_ERROR_MSK		NO_OS_BIT(15)
#define ADMV1014_TOO_FEW_ERRORS_MSK		NO_OS_BIT(14)
#define ADMV1014_TOO_MANY_ERRORS_MSK		NO_OS_BIT(13)
#define ADMV1014_ADDRESS_RANGE_ERROR_MSK	NO_OS_BIT(12)

/* ADMV1014_REG_ENABLE Map */
#define ADMV1014_IBIAS_PD_MSK			NO_OS_BIT(14)
#define ADMV1014_P1DB_COMPENSATION_MSK		NO_OS_GENMASK(13, 12)
#define ADMV1014_IF_AMP_PD_MSK			NO_OS_BIT(11)
#define ADMV1014_QUAD_BG_PD_MSK			NO_OS_BIT(9)
#define ADMV1014_BB_AMP_PD_MSK			NO_OS_BIT(8)
#define ADMV1014_QUAD_IBIAS_PD_MSK		NO_OS_BIT(7)
#define ADMV1014_DET_EN_MSK			NO_OS_BIT(6)
#define ADMV1014_BG_PD_MSK			NO_OS_BIT(5)

/* ADMV1014_REG_QUAD Map */
#define ADMV1014_QUAD_SE_MODE_MSK		NO_OS_GENMASK(9, 6)
#define ADMV1014_QUAD_FILTERS_MSK		NO_OS_GENMASK(3, 0)

/* ADMV1014_REG_LO_AMP_PHASE_ADJUST1 Map */
#define ADMV1014_LOAMP_PH_ADJ_I_FINE_MSK	NO_OS_GENMASK(15, 9)
#define ADMV1014_LOAMP_PH_ADJ_Q_FINE_MSK	NO_OS_GENMASK(8, 2)

/* ADMV1014_REG_MIXER Map */
#define ADMV1014_MIXER_VGATE_MSK		NO_OS_GENMASK(15, 9)
#define ADMV1014_DET_PROG_MSK			NO_OS_GENMASK(6, 0)

/* ADMV1014_REG_IF_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_I_MSK	NO_OS_GENMASK(11, 8)
#define ADMV1014_IF_AMP_FINE_GAIN_Q_MSK		NO_OS_GENMASK(7, 4)
#define ADMV1014_IF_AMP_FINE_GAIN_I_MSK		NO_OS_GENMASK(3, 0)

/* ADMV1014_REG_IF_AMP_BB_AMP Map */
#define ADMV1014_IF_AMP_COARSE_GAIN_Q_MSK	NO_OS_GENMASK(15, 12)
#define ADMV1014_BB_AMP_OFFSET_Q_MSK		NO_OS_GENMASK(9, 5)
#define ADMV1014_BB_AMP_OFFSET_I_MSK		NO_OS_GENMASK(4, 0)

/* ADMV1014_REG_BB_AMP_AGC Map */
#define ADMV1014_BB_AMP_REF_GEN_MSK		NO_OS_GENMASK(6, 3)
#define ADMV1014_BB_AMP_GAIN_CTRL_MSK		NO_OS_GENMASK(2, 1)
#define ADMV1014_BB_SWITCH_HIGH_LOW_CM_MSK	NO_OS_BIT(0)

/* ADMV1014_REG_VVA_TEMP_COMP Map */
#define ADMV1014_VVA_TEMP_COMP_MSK		NO_OS_GENMASK(15, 0)

/* ADMV1014 Miscellaneous Defines */
#define ADMV1014_READ				NO_OS_BIT(7)
#define ADMV1014_REG_ADDR_READ_MSK		NO_OS_GENMASK(6, 1)
#define ADMV1014_REG_ADDR_WRITE_MSK		NO_OS_GENMASK(22, 17)
#define ADMV1014_REG_DATA_MSK			NO_OS_GENMASK(16, 1)
#define ADMV1014_NUM_REGULATORS			9

/* Specifications */
#define ADMV1014_BUFF_SIZE_BYTES		3
#define ADMV1014_SPI_READ_CMD			NO_OS_BIT(7)
#define ADMV1014_SPI_WRITE_CMD			(0 << 7)

/***************************************************************************//**
 * @brief The `admv1014_input_mode` enumeration defines the possible input modes
 * for the ADMV1014 device, which can operate in either I/Q mode or
 * Intermediate Frequency (IF) mode. This enumeration is used to
 * configure the device's input mode, allowing it to switch between
 * processing I/Q signals or IF signals, depending on the application
 * requirements.
 *
 * @param ADMV1014_IQ_MODE Represents the I/Q mode for the ADMV1014 device.
 * @param ADMV1014_IF_MODE Represents the Intermediate Frequency (IF) mode for
 * the ADMV1014 device.
 ******************************************************************************/
enum admv1014_input_mode {
	ADMV1014_IQ_MODE,
	ADMV1014_IF_MODE,
};

/***************************************************************************//**
 * @brief The `admv1014_quad_se_mode` enumeration defines the different modes
 * for switching between differential and single-ended configurations in
 * the ADMV1014 device. It provides three distinct modes: positive
 * single-ended, negative single-ended, and differential, each associated
 * with a specific integer value. This enumeration is used to configure
 * the device's input mode to suit different application requirements.
 *
 * @param ADMV1014_SE_MODE_POS Represents the positive single-ended mode with a
 * value of 6.
 * @param ADMV1014_SE_MODE_NEG Represents the negative single-ended mode with a
 * value of 9.
 * @param ADMV1014_SE_MODE_DIFF Represents the differential mode with a value of
 * 12.
 ******************************************************************************/
enum admv1014_quad_se_mode {
	ADMV1014_SE_MODE_POS = 6,
	ADMV1014_SE_MODE_NEG = 9,
	ADMV1014_SE_MODE_DIFF = 12,
};

/***************************************************************************//**
 * @brief The `admv1014_quad_filters` enumeration defines a set of constants
 * representing different local oscillator (LO) frequency bands for the
 * ADMV1014 device. Each enumerator corresponds to a specific frequency
 * range, allowing the selection of the appropriate LO band for the
 * device's operation. This enumeration is used to configure the LO
 * filters' bandwidth selection in the ADMV1014 driver.
 *
 * @param LO_BAND_8_62_TO_10_25_GHZ Represents a local oscillator band with a
 * frequency range from 8.62 GHz to 10.25 GHz.
 * @param LO_BAND_6_6_TO_9_2_GHZ Represents a local oscillator band with a
 * frequency range from 6.6 GHz to 9.2 GHz.
 * @param LO_BAND_5_4_TO_8_GHZ Represents a local oscillator band with a
 * frequency range from 5.4 GHz to 8 GHz.
 * @param LO_BAND_5_4_TO_7_GHZ Represents a local oscillator band with a
 * frequency range from 5.4 GHz to 7 GHz.
 ******************************************************************************/
enum admv1014_quad_filters {
	LO_BAND_8_62_TO_10_25_GHZ = 0,
	LO_BAND_6_6_TO_9_2_GHZ = 5,
	LO_BAND_5_4_TO_8_GHZ = 10,
	LO_BAND_5_4_TO_7_GHZ = 15
};

/***************************************************************************//**
 * @brief The `admv1014_det_prog` enumeration defines various digital receiver
 * detector program ranges for the ADMV1014 device, each corresponding to
 * a specific range of power levels in dBm. These enumerated values are
 * used to configure the detector's sensitivity and operational range,
 * allowing the device to handle different signal strengths effectively.
 *
 * @param DET_PROG_NEG_12_DBM_TO_POS_4DBM Represents a detector program range
 * from -12 dBm to +4 dBm.
 * @param DET_PROG_NEG_13_DBM_TO_POS_3DBM Represents a detector program range
 * from -13 dBm to +3 dBm.
 * @param DET_PROG_NEG_14_DBM_TO_POS_2DBM Represents a detector program range
 * from -14 dBm to +2 dBm.
 * @param DET_PROG_NEG_15_DBM_TO_POS_1DBM Represents a detector program range
 * from -15 dBm to +1 dBm.
 * @param DET_PROG_NEG_15_5_DBM_TO_POS_0_5_DBM Represents a detector program
 * range from -15.5 dBm to +0.5 dBm.
 * @param DET_PROG_NEG_16_25_DBM_TO_NEG_0_25_DBM Represents a detector program
 * range from -16.25 dBm to -0.25
 * dBm.
 * @param DET_PROG_NEG_17_DBM_TO_NEG_1DBM Represents a detector program range
 * from -17 dBm to -1 dBm.
 * @param DET_PROG_NEG_18_DBM_TO_NEG_2DBM Represents a detector program range
 * from -18 dBm to -2 dBm.
 ******************************************************************************/
enum admv1014_det_prog {
	DET_PROG_NEG_12_DBM_TO_POS_4DBM = 0,
	DET_PROG_NEG_13_DBM_TO_POS_3DBM = 1,
	DET_PROG_NEG_14_DBM_TO_POS_2DBM = 2,
	DET_PROG_NEG_15_DBM_TO_POS_1DBM = 4,
	DET_PROG_NEG_15_5_DBM_TO_POS_0_5_DBM = 8,
	DET_PROG_NEG_16_25_DBM_TO_NEG_0_25_DBM = 16,
	DET_PROG_NEG_17_DBM_TO_NEG_1DBM = 32,
	DET_PROG_NEG_18_DBM_TO_NEG_2DBM = 64,
};
/***************************************************************************//**
 * @brief The `admv1014_init_param` structure is used to define the
 * initialization parameters for the ADMV1014 device. It includes
 * settings for SPI communication, local oscillator frequency, input and
 * quad SE modes, and various enable flags for detector and P1DB
 * compensation. This structure is essential for configuring the ADMV1014
 * device to operate according to specific application requirements.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param lo_in Specifies the local oscillator input frequency.
 * @param input_mode Defines the input mode using the admv1014_input_mode
 * enumeration.
 * @param quad_se_mode Specifies the quad single-ended mode using the
 * admv1014_quad_se_mode enumeration.
 * @param det_en Boolean flag to enable or disable the detector.
 * @param vcm_mv Specifies the common-mode voltage in millivolts.
 * @param p1db_comp_en Boolean flag to enable or disable P1DB compensation.
 ******************************************************************************/
struct admv1014_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** Input Mode */
	enum admv1014_input_mode	input_mode;
	/** Quad SE Mode */
	enum admv1014_quad_se_mode	quad_se_mode;
	/** Detector Enable */
	bool				det_en;
	/** Common-Mode Voltage (mV) */
	unsigned int			vcm_mv;
	/** P1DB Compensation Enable */
	bool				p1db_comp_en;
};

/***************************************************************************//**
 * @brief The `admv1014_dev` structure is a device descriptor for the ADMV1014,
 * a microwave downconverter. It contains configuration parameters such
 * as the SPI descriptor for communication, local oscillator input
 * frequency, input mode, quadrature single-ended mode, and various
 * enable flags for features like the detector and P1DB compensation.
 * This structure is essential for initializing and managing the ADMV1014
 * device's operational settings.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param lo_in Specifies the local oscillator input frequency.
 * @param input_mode Defines the input mode, either intermediate frequency or
 * I/Q mode.
 * @param quad_se_mode Specifies the quadrature single-ended mode.
 * @param det_en Boolean flag to enable or disable the detector.
 * @param vcm_mv Specifies the common-mode voltage in millivolts.
 * @param p1db_comp_en Boolean flag to enable or disable P1DB compensation.
 ******************************************************************************/
struct admv1014_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** LO Input Frequency */
	unsigned long long		lo_in;
	/** Input Mode */
	enum admv1014_input_mode	input_mode;
	/** Quad SE Mode */
	enum admv1014_quad_se_mode	quad_se_mode;
	/** Detector Enable */
	bool				det_en;
	/** Common-Mode Voltage (mV) */
	unsigned int			vcm_mv;
	/** P1DB Compensation Enable */
	bool				p1db_comp_en;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to write a 16-bit data value to a specific
 * register of the ADMV1014 device using SPI communication. It is
 * essential to ensure that the device has been properly initialized
 * before calling this function. The function constructs a 3-byte buffer
 * according to the ADMV1014 SPI protocol and sends it over the SPI
 * interface. This function is typically used when configuring or
 * updating the settings of the ADMV1014 device. Proper error handling
 * should be implemented by checking the return value to ensure the SPI
 * transaction was successful.
 *
 * @param dev A pointer to an initialized 'admv1014_dev' structure representing
 * the device. Must not be null.
 * @param reg_addr The 8-bit address of the register to which data will be
 * written. Valid register addresses are defined by the device's
 * register map.
 * @param data A 16-bit data value to be written to the specified register. The
 * data is formatted according to the device's protocol before
 * transmission.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the result of the SPI write operation.
 ******************************************************************************/
int admv1014_spi_write(struct admv1014_dev *dev, uint8_t reg_addr,
		       uint16_t data);

/* ADMV1014 Register Update */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * ADMV1014 device by performing a read-modify-write operation over SPI.
 * It is useful when only certain bits of a register need to be changed
 * without affecting the other bits. The function first reads the current
 * value of the register, applies the mask to clear the bits to be
 * updated, and then sets the new data. It should be called when the
 * device is properly initialized and the SPI interface is configured.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an initialized 'admv1014_dev' structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADMV1014 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits specified by the mask will be updated.
 * @return Returns 0 on success, or a negative error code if the SPI read or
 * write operation fails.
 ******************************************************************************/
int admv1014_spi_update_bits(struct admv1014_dev *dev, uint8_t reg_addr,
			     uint16_t mask, uint16_t data);

/***************************************************************************//**
 * @brief Use this function to read data from a specific register of the
 * ADMV1014 device. It requires a valid device descriptor and a register
 * address to perform the read operation. The function communicates with
 * the device over SPI and retrieves a 16-bit value from the specified
 * register, storing it in the provided data pointer. Ensure that the
 * device has been properly initialized before calling this function. The
 * function returns an error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized admv1014_dev structure representing
 * the device. Must not be null.
 * @param reg_addr The 8-bit address of the register to read from. Valid
 * register addresses are defined by the device's register map.
 * @param data A pointer to a uint16_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int admv1014_spi_read(struct admv1014_dev *dev, uint8_t reg_addr,
		      uint16_t *data);

/***************************************************************************//**
 * @brief This function configures the digital Rx detector program of the
 * ADMV1014 device to a specified setting. It should be called when you
 * need to adjust the detector's sensitivity range according to your
 * application requirements. Ensure that the device has been properly
 * initialized before calling this function. The function updates the
 * relevant register with the provided detector program setting and
 * returns an integer status code indicating success or failure.
 *
 * @param dev A pointer to an initialized 'admv1014_dev' structure representing
 * the device. Must not be null.
 * @param det_prog An enumerated value of type 'admv1014_det_prog' specifying
 * the desired detector program setting. Valid values are
 * defined in the 'admv1014_det_prog' enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int admv1014_set_det_prog(struct admv1014_dev *dev,
			  enum admv1014_det_prog det_prog);

/***************************************************************************//**
 * @brief Use this function to obtain the current setting of the digital Rx
 * detector program from the ADMV1014 device. It is essential to ensure
 * that the device has been properly initialized before calling this
 * function. The function reads the relevant register from the device and
 * extracts the detector program setting, which is then stored in the
 * provided output parameter. This function is useful for verifying the
 * current configuration of the detector program.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device. Must not be null.
 * @param det_prog A pointer to an `admv1014_det_prog` enum where the current
 * detector program setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int admv1014_get_det_prog(struct admv1014_dev *dev,
			  enum admv1014_det_prog *det_prog);

/***************************************************************************//**
 * @brief This function configures the baseband amplifier gain of the ADMV1014
 * device by updating the relevant register through SPI communication. It
 * should be called when the user needs to adjust the gain settings of
 * the baseband amplifier. The function requires a valid device
 * descriptor and a gain value within the acceptable range. It returns an
 * integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an initialized 'admv1014_dev' structure representing
 * the device. Must not be null.
 * @param gain An 8-bit unsigned integer representing the desired gain setting.
 * The valid range is determined by the device's specifications, and
 * invalid values may result in undefined behavior.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int admv1014_set_bb_amp_gain(struct admv1014_dev *dev, uint8_t gain);

/***************************************************************************//**
 * @brief Use this function to obtain the current gain setting of the baseband
 * amplifier in the ADMV1014 device. It is essential to call this
 * function only after the device has been properly initialized. The
 * function reads the gain setting from the device and stores it in the
 * provided memory location. If the read operation fails, the function
 * returns an error code, and the gain value is not updated.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device. Must not be null.
 * @param gain A pointer to a uint8_t variable where the gain value will be
 * stored. Must not be null. The caller is responsible for providing
 * a valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int admv1014_get_bb_amp_gain(struct admv1014_dev *dev, uint8_t *gain);

/***************************************************************************//**
 * @brief This function configures the I and Q phase adjustments for the
 * ADMV1014 device by updating the relevant register bits. It should be
 * called when you need to set or modify the phase settings of the
 * device. Ensure that the device has been properly initialized before
 * calling this function. The function returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to an initialized admv1014_dev structure representing
 * the device. Must not be null.
 * @param i_phase The desired I phase adjustment value. It is an 8-bit unsigned
 * integer.
 * @param q_phase The desired Q phase adjustment value. It is an 8-bit unsigned
 * integer.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int admv1014_set_phase(struct admv1014_dev *dev, uint8_t i_phase,
		       uint8_t q_phase);

/***************************************************************************//**
 * @brief Use this function to obtain the current I and Q phase adjustment
 * values from the ADMV1014 device. This function is typically called
 * when you need to read the phase settings for diagnostic or
 * configuration purposes. Ensure that the device has been properly
 * initialized before calling this function. The function reads the phase
 * adjustment values from the device's registers and stores them in the
 * provided pointers. It returns an error code if the read operation
 * fails.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device. Must not be null.
 * @param i_phase A pointer to a uint8_t where the I phase adjustment value will
 * be stored. Must not be null.
 * @param q_phase A pointer to a uint8_t where the Q phase adjustment value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int admv1014_get_phase(struct admv1014_dev *dev, uint8_t *i_phase,
		       uint8_t *q_phase);

/***************************************************************************//**
 * @brief Use this function to configure the intermediate frequency (IF)
 * amplifier gain settings for the ADMV1014 device. This function allows
 * you to set both coarse and fine gain values for the I and Q channels.
 * It is essential to ensure that the device is properly initialized
 * before calling this function. The function updates the device's
 * registers to reflect the specified gain settings. If the operation
 * fails, an error code is returned.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device. Must not be null.
 * @param i_coarse_gain The coarse gain setting for the I channel. Valid range
 * is determined by the device's specifications.
 * @param q_coarse_gain The coarse gain setting for the Q channel. Valid range
 * is determined by the device's specifications.
 * @param i_fine_gain The fine gain setting for the I channel. Valid range is
 * determined by the device's specifications.
 * @param q_fine_gain The fine gain setting for the Q channel. Valid range is
 * determined by the device's specifications.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int admv1014_set_if_amp_gain(struct admv1014_dev *dev, uint8_t i_coarse_gain,
			     uint8_t q_coarse_gain, uint8_t i_fine_gain, uint8_t q_fine_gain);

/***************************************************************************//**
 * @brief Use this function to obtain the current gain settings for the I and Q
 * channels of the IF amplifier in the ADMV1014 device. This function
 * should be called when you need to read the current gain configuration,
 * which includes both coarse and fine gain settings for the I and Q
 * channels. Ensure that the device has been properly initialized before
 * calling this function. The function will populate the provided
 * pointers with the gain values, and it returns an error code if the
 * read operation fails.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device. Must not be null.
 * @param i_coarse_gain A pointer to a uint8_t where the I channel coarse gain
 * will be stored. Must not be null.
 * @param q_coarse_gain A pointer to a uint8_t where the Q channel coarse gain
 * will be stored. Must not be null.
 * @param i_fine_gain A pointer to a uint8_t where the I channel fine gain will
 * be stored. Must not be null.
 * @param q_fine_gain A pointer to a uint8_t where the Q channel fine gain will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int admv1014_get_if_amp_gain(struct admv1014_dev *dev, uint8_t *i_coarse_gain,
			     uint8_t *q_coarse_gain, uint8_t *i_fine_gain, uint8_t *q_fine_gain);

/***************************************************************************//**
 * @brief This function initializes the ADMV1014 device using the provided
 * initialization parameters. It sets up the SPI communication,
 * configures the device registers, and performs a software reset. The
 * function must be called before any other operations on the ADMV1014
 * device. It returns an error code if initialization fails, ensuring
 * that the device is not left in an undefined state. The caller is
 * responsible for providing valid initialization parameters and handling
 * the device pointer.
 *
 * @param device A pointer to a pointer of type `struct admv1014_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct admv1014_init_param` containing the
 * initialization parameters. This must not be null and should
 * be properly initialized with valid values before calling
 * the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered. The
 * `device` pointer is set to a valid device descriptor on success, or
 * remains unchanged on failure.
 ******************************************************************************/
int admv1014_init(struct admv1014_dev **device,
		  struct admv1014_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADMV1014 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that any allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks in
 * your application.
 *
 * @param dev A pointer to an initialized `admv1014_dev` structure representing
 * the device to be removed. Must not be null. The function will
 * handle invalid pointers by returning an error code.
 * @return Returns 0 on successful removal of the device resources. If an error
 * occurs during the removal process, a negative error code is returned.
 ******************************************************************************/
int admv1014_remove(struct admv1014_dev *dev);

#endif /* ADMV1014_H_ */
