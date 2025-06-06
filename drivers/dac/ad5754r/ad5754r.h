/***************************************************************************//**
 *   @file   ad5754r.h
 *   @brief  Header file of AD5754R Driver.
 *   @author Ribhu DasPurkayastha (Ribhu.DasPurkayastha@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
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
 *
*******************************************************************************/
#ifndef __AD5754R_H__
#define __AD5754R_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"
#include "no_os_error.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Maximum resolution */
#define AD5754R_MAX_RESOLUTION          	16

/* DAC Channel Count */
#define AD5754R_NUM_CHANNELS			4

/* Register Map */
#define AD5754R_REG_DAC				0
#define AD5754R_REG_OUTPUT_RANGE_SEL    	1
#define AD5754R_REG_PWR_CTRL			2
#define AD5754R_REG_CONTROL			3

/* DAC Channel Address */
#define AD5754R_DAC_CH_A_ADDR			0
#define AD5754R_DAC_CH_B_ADDR			1
#define AD5754R_DAC_CH_C_ADDR			2
#define AD5754R_DAC_CH_D_ADDR			3
#define AD5754R_DAC_CH_ALL_ADDR			4

/* Input Shift Register Masks */
#define AD5754R_ADDR_REG_MASK			NO_OS_GENMASK(5, 3)
#define AD5754R_ADDR_REG(x)			no_os_field_prep(AD5754R_ADDR_REG_MASK, x)
#define AD5754R_ADDR_DAC_CH_MASK		NO_OS_GENMASK(2, 0)
#define AD5754R_ADDR_DAC_CH(x)			no_os_field_prep(AD5754R_ADDR_DAC_CH_MASK, x)
#define AD5754R_PREP_INSTR_ADDR(reg, dac_ch)	(AD5754R_ADDR_REG(reg) | \
						AD5754R_ADDR_DAC_CH(dac_ch))

/* Output Range Select Mask */
#define AD5754R_OUTPUT_RANGE_SEL_MASK		NO_OS_GENMASK(2,0)

/* Control Register bit Definition */
#define AD5754R_CTRL_NOP			0
#define AD5754R_CTRL_TSD_EN(x)			no_os_field_prep(NO_OS_BIT(3), x)
#define AD5754R_CTRL_TSD_EN_MASK		NO_OS_BIT(3)
#define AD5754R_CTRL_CLAMP_EN(X)		no_os_field_prep(NO_OS_BIT(2), x)
#define AD5754R_CTRL_CLAMP_EN_MASK		NO_OS_BIT(2)
#define AD5754R_CTRL_CLR_SEL(x)			no_os_field_prep(NO_OS_BIT(1), x)
#define AD5754R_CTRL_CLR_SEL_MASK		NO_OS_BIT(1)
#define AD5754R_CTRL_SDO_DISABLE(x)		(x)
#define AD5754R_CTRL_SDO_DISABLE_MASK	  	NO_OS_BIT(0)
#define AD5754R_CTRL_CLEAR			0
#define AD5754R_CTRL_LOAD			0

/* Power Control Register bit definition */
#define AD5754R_PWR_UP_DAC_CH_MASK(x)		NO_OS_BIT(x)
#define AD5754R_PWR_UP_INT_REF_MASK		NO_OS_BIT(4)
#define AD5754R_PWR_OC_ALERT_MASK		NO_OS_GENMASK(10,7)
#define AD5754R_PWR_OC_ALERT_CH_MASK(x)		no_os_field_prep(AD5754R_PWR_OC_ALERT_MASK, \
						NO_OS_BIT(x))
#define AD5754R_PWR_TSD_ALERT_MASK		NO_OS_BIT(5)

#define AD5754R_BYTE_H				NO_OS_GENMASK(15, 8)
#define AD5754R_BYTE_L				NO_OS_GENMASK(7, 0)

#define AD5754R_READ				NO_OS_BIT(7)
#define AD5754R_WRITE				0

#define AD5754R_INSTR_NOP			0x18
#define AD5754R_INSTR_CLEAR			0x1C
#define AD5754R_INSTR_LOAD			0x1D

#define AD5754R_GAIN_SCALE      		1000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5754r_dac_channels` enumeration defines the available DAC
 * channels for the AD5754R device, which includes four distinct channels
 * labeled A through D. This enumeration is used to specify and manage
 * the individual DAC channels within the AD5754R driver, allowing for
 * operations such as setting output ranges, power states, and other
 * channel-specific configurations.
 *
 * @param AD5754R_DAC_CH_A Represents DAC channel A.
 * @param AD5754R_DAC_CH_B Represents DAC channel B.
 * @param AD5754R_DAC_CH_C Represents DAC channel C.
 * @param AD5754R_DAC_CH_D Represents DAC channel D.
 ******************************************************************************/
enum ad5754r_dac_channels {
	AD5754R_DAC_CH_A,
	AD5754R_DAC_CH_B,
	AD5754R_DAC_CH_C,
	AD5754R_DAC_CH_D,
};

/***************************************************************************//**
 * @brief The `ad5754r_dac_ch_range` enumeration defines the possible output
 * voltage ranges for the DAC channels in the AD5754R device. Each
 * enumerator corresponds to a specific voltage span that the DAC can
 * output, allowing for both unipolar and bipolar configurations. This
 * enumeration is used to configure the output range of each DAC channel,
 * enabling the device to meet various application requirements.
 *
 * @param AD5754R_SPAN_0V_TO_5V Represents a DAC output range from 0V to 5V.
 * @param AD5754R_SPAN_0V_TO_10V Represents a DAC output range from 0V to 10V.
 * @param AD5754R_SPAN_0V_TO_10V8 Represents a DAC output range from 0V to
 * 10.8V.
 * @param AD5754R_SPAN_M5V_TO_5V Represents a DAC output range from -5V to 5V.
 * @param AD5754R_SPAN_M10V_TO_10V Represents a DAC output range from -10V to
 * 10V.
 * @param AD5754R_SPAN_M10V8_TO_10V8 Represents a DAC output range from -10.8V
 * to 10.8V.
 ******************************************************************************/
enum ad5754r_dac_ch_range {
	AD5754R_SPAN_0V_TO_5V,
	AD5754R_SPAN_0V_TO_10V,
	AD5754R_SPAN_0V_TO_10V8,
	AD5754R_SPAN_M5V_TO_5V,
	AD5754R_SPAN_M10V_TO_10V,
	AD5754R_SPAN_M10V8_TO_10V8,
};

/***************************************************************************//**
 * @brief The `ad5754r_dac_addr_for_ctrl_settings` enumeration defines specific
 * addresses used for controlling various settings of the AD5754R DAC,
 * such as no-operation, thermal shutdown, clamp, clear, and load
 * operations. Each enumerator corresponds to a unique address that can
 * be used to configure the DAC's control register, facilitating the
 * management of its operational modes and features.
 *
 * @param AD5754R_NOP_DAC_ADDR Represents a no-operation address for the DAC
 * control settings.
 * @param AD5754R_TSD_CLAMP_CLR_SDO_DAC_ADDR Represents the address for enabling
 * thermal shutdown, clamp, clear, and
 * SDO control settings.
 * @param AD5754R_CLEAR_DAC_ADDR Represents the address for clearing the DAC
 * settings.
 * @param AD5754R_LOAD_DAC_ADDR Represents the address for loading the DAC
 * settings.
 ******************************************************************************/
enum ad5754r_dac_addr_for_ctrl_settings {
	AD5754R_NOP_DAC_ADDR = 0,
	AD5754R_TSD_CLAMP_CLR_SDO_DAC_ADDR = 1,
	AD5754R_CLEAR_DAC_ADDR = 4,
	AD5754R_LOAD_DAC_ADDR = 5
};

/***************************************************************************//**
 * @brief The `ad5754r_ctrl_current_clamp_en` is an enumeration that defines the
 * modes for enabling or disabling the current clamp feature in the
 * AD5754R device. It provides two options: `AD5754R_CTRL_CLAMP_DIS` to
 * disable the current clamp and `AD5754R_CTRL_CLAMP_EN` to enable it.
 * This enumeration is used to control the current clamp functionality,
 * which is a safety feature to prevent excessive current flow in the
 * device.
 *
 * @param AD5754R_CTRL_CLAMP_DIS Represents the mode where the current clamp is
 * disabled.
 * @param AD5754R_CTRL_CLAMP_EN Represents the mode where the current clamp is
 * enabled.
 ******************************************************************************/
enum ad5754r_ctrl_current_clamp_en {
	AD5754R_CTRL_CLAMP_DIS,
	AD5754R_CTRL_CLAMP_EN
};

/***************************************************************************//**
 * @brief The `ad5754r_ctrl_tsd_en` enumeration defines the possible states for
 * enabling or disabling the thermal shutdown (TSD) feature in the
 * AD5754R device. This feature is crucial for protecting the device from
 * overheating by shutting down operations when a certain temperature
 * threshold is exceeded. The enumeration provides two states:
 * `AD5754R_CTRL_TSD_DIS` for disabling the TSD feature and
 * `AD5754R_CTRL_TSD_EN` for enabling it.
 *
 * @param AD5754R_CTRL_TSD_DIS Represents the disabled state for the thermal
 * shutdown (TSD) control.
 * @param AD5754R_CTRL_TSD_EN Represents the enabled state for the thermal
 * shutdown (TSD) control.
 ******************************************************************************/
enum ad5754r_ctrl_tsd_en {
	AD5754R_CTRL_TSD_DIS,
	AD5754R_CTRL_TSD_EN
};

/***************************************************************************//**
 * @brief The `ad5754r_ctrl_clear_sel` enumeration defines the clear mode
 * options for the AD5754R device, which is a digital-to-analog converter
 * (DAC). This enum provides two options for clearing the DAC output:
 * setting it to 0 volts or to a midscale code, allowing for flexible
 * control over the DAC's behavior during a clear operation.
 *
 * @param AD5754R_CTRL_CLEAR_0V Represents a clear mode option where the output
 * is set to 0 volts.
 * @param AD5754R_CTRL_CLEAR_MIDSCALE_CODE Represents a clear mode option where
 * the output is set to a midscale code.
 ******************************************************************************/
enum ad5754r_ctrl_clear_sel {
	AD5754R_CTRL_CLEAR_0V,
	AD5754R_CTRL_CLEAR_MIDSCALE_CODE
};

/***************************************************************************//**
 * @brief The `ad5754r_ctrl_sdo_dis` enumeration defines the possible states for
 * enabling or disabling the Serial Data Out (SDO) line in the AD5754R
 * device. This is used to control whether the SDO line is active or
 * inactive, which can be important for managing data output in SPI
 * communication scenarios.
 *
 * @param AD5754R_CTRL_SDO_EN Represents the enabled state of the SDO (Serial
 * Data Out) line.
 * @param AD5754R_CTRL_SDO_DIS Represents the disabled state of the SDO (Serial
 * Data Out) line.
 ******************************************************************************/
enum ad5754r_ctrl_sdo_dis {
	AD5754R_CTRL_SDO_EN,
	AD5754R_CTRL_SDO_DIS
};

/***************************************************************************//**
 * @brief The `ad5754r_pwr_dac_ch_pwrup` enumeration defines the power states
 * for DAC channels in the AD5754R device, allowing for the selection
 * between powering down or powering up a specific DAC channel. This is
 * crucial for managing power consumption and operational states of the
 * DAC channels in the device.
 *
 * @param AD5754R_PWR_DAC_CH_POWERDOWN Represents the power-down state for a DAC
 * channel.
 * @param AD5754R_PWR_DAC_CH_POWERUP Represents the power-up state for a DAC
 * channel.
 ******************************************************************************/
enum ad5754r_pwr_dac_ch_pwrup {
	AD5754R_PWR_DAC_CH_POWERDOWN,
	AD5754R_PWR_DAC_CH_POWERUP
};

/***************************************************************************//**
 * @brief The `ad5754r_pwr_int_ref_pwrup` enumeration defines the power states
 * for the internal reference of the AD5754R device. It includes two
 * states: `AD5754R_PWR_INT_REF_POWERDOWN` for powering down the internal
 * reference, and `AD5754R_PWR_INT_REF_POWERUP` for powering it up. This
 * enumeration is used to control the power state of the internal
 * reference, which is crucial for the operation of the device.
 *
 * @param AD5754R_PWR_INT_REF_POWERDOWN Represents the power-down state for the
 * internal reference.
 * @param AD5754R_PWR_INT_REF_POWERUP Represents the power-up state for the
 * internal reference.
 ******************************************************************************/
enum ad5754r_pwr_int_ref_pwrup {
	AD5754R_PWR_INT_REF_POWERDOWN,
	AD5754R_PWR_INT_REF_POWERUP
};

/***************************************************************************//**
 * @brief The `ad5754r_pwr_tsd_alert` enumeration defines the possible states of
 * a thermal shutdown alert for the AD5754R device. It is used to
 * represent whether a thermal shutdown condition has been detected or
 * not, which is crucial for monitoring the device's thermal status and
 * ensuring its safe operation.
 *
 * @param AD5754R_PWR_TSD_NOT_DETECTED Indicates that a thermal shutdown
 * condition has not been detected.
 * @param AD5754R_PWR_TSD_DETECTED Indicates that a thermal shutdown condition
 * has been detected.
 ******************************************************************************/
enum ad5754r_pwr_tsd_alert {
	AD5754R_PWR_TSD_NOT_DETECTED,
	AD5754R_PWR_TSD_DETECTED
};

/***************************************************************************//**
 * @brief The `ad5754r_pwr_oc_ch_alert` enumeration defines the possible states
 * of overcurrent alerts for DAC channels in the AD5754R device. It is
 * used to represent whether an overcurrent condition has been detected
 * or not on a specific DAC channel, which is crucial for monitoring and
 * managing the power status of the device to prevent damage or
 * malfunction.
 *
 * @param AD5754R_PWR_OC_CH_NOT_DETECTED Indicates that no overcurrent condition
 * is detected on the DAC channel.
 * @param AD5754R_PWR_OC_CH_DETECTED Indicates that an overcurrent condition is
 * detected on the DAC channel.
 ******************************************************************************/
enum ad5754r_pwr_oc_ch_alert {
	AD5754R_PWR_OC_CH_NOT_DETECTED,
	AD5754R_PWR_OC_CH_DETECTED
};

/***************************************************************************//**
 * @brief The `ad5754r_encoding_scheme` is an enumeration that defines the
 * encoding schemes available for the AD5754R DAC device. It provides two
 * options: `AD5754R_ENCODING_TWOSCOMPLEMENT` for two's complement
 * encoding and `AD5754R_ENCODING_BINARY` for binary encoding. This
 * enumeration is used to specify how the digital input data is
 * interpreted by the DAC, affecting how the output voltage is generated
 * based on the input code.
 *
 * @param AD5754R_ENCODING_TWOSCOMPLEMENT Represents the two's complement
 * encoding scheme for the DAC.
 * @param AD5754R_ENCODING_BINARY Represents the binary encoding scheme for the
 * DAC.
 ******************************************************************************/
enum ad5754r_encoding_scheme {
	AD5754R_ENCODING_TWOSCOMPLEMENT,
	AD5754R_ENCODING_BINARY
};

/***************************************************************************//**
 * @brief The `ad5754r_init_param` structure is used to initialize the AD5754R
 * device, containing parameters for SPI and GPIO initialization, DAC
 * channel configurations, and control settings such as current clamp,
 * thermal shutdown, and SDO disable. It also includes arrays for
 * specifying the output range and power-up states of the DAC channels,
 * as well as the power-up state of the internal reference and the
 * encoding scheme. The reference voltage is specified in millivolts.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_clear_init Pointer to GPIO initialization parameters for the
 * clear function.
 * @param gpio_ldac_init Pointer to GPIO initialization parameters for the load
 * DAC function.
 * @param dac_ch_range Array specifying the output range for each DAC channel.
 * @param clamp_en Enum indicating whether the current clamp is enabled or
 * disabled.
 * @param tsd_en Enum indicating whether thermal shutdown is enabled or
 * disabled.
 * @param clear_sel Enum specifying the clear mode setting.
 * @param sdo_dis Enum indicating whether the SDO is disabled.
 * @param dac_ch_pwr_states Array specifying the power-up states of each DAC
 * channel.
 * @param int_ref_pwrup Enum indicating the power-up state of the internal
 * reference.
 * @param encoding Enum specifying the encoding scheme used by the DAC.
 * @param vref_mv Reference voltage in millivolts.
 ******************************************************************************/
struct ad5754r_init_param {
	/* SPI Init Parameters */
	struct no_os_spi_init_param	*spi_init;
	/* Clear GPIO Init Params*/
	struct no_os_gpio_init_param *gpio_clear_init;
	/* Load DAC Init Params*/
	struct no_os_gpio_init_param *gpio_ldac_init;
	/* Output Range Select */
	enum ad5754r_dac_ch_range dac_ch_range[AD5754R_NUM_CHANNELS];
	/* Current Clamp Enable */
	enum ad5754r_ctrl_current_clamp_en clamp_en;
	/* TSD Enable */
	enum ad5754r_ctrl_tsd_en tsd_en;
	/* Clear Setting */
	enum ad5754r_ctrl_clear_sel clear_sel;
	/* SDO Disable */
	enum ad5754r_ctrl_sdo_dis sdo_dis;
	/* Powerup States of DAC Channels */
	enum ad5754r_pwr_dac_ch_pwrup dac_ch_pwr_states[AD5754R_NUM_CHANNELS];
	/* Powerup State of Internal Reference */
	enum ad5754r_pwr_int_ref_pwrup int_ref_pwrup;
	/* Encoding scheme */
	enum ad5754r_encoding_scheme encoding;
	/* Reference voltage in millivolts */
	uint16_t vref_mv;
};

/***************************************************************************//**
 * @brief The `ad5754r_dev` structure represents the configuration and state of
 * an AD5754R device, which is a digital-to-analog converter (DAC). It
 * includes descriptors for SPI and GPIO interfaces, settings for DAC
 * channel output ranges, and various control options such as current
 * clamp, thermal shutdown, and clear settings. The structure also holds
 * power-up states for DAC channels and the internal reference, as well
 * as the encoding scheme and reference voltage used by the device. This
 * structure is essential for managing the device's operation and
 * interfacing with it programmatically.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_clear Pointer to the GPIO descriptor for the clear function.
 * @param gpio_ldac Pointer to the GPIO descriptor for the load DAC function.
 * @param dac_ch_range Array specifying the output range for each DAC channel.
 * @param clamp_en Enum indicating if the current clamp is enabled or disabled.
 * @param tsd_en Enum indicating if the thermal shutdown is enabled or disabled.
 * @param clear_sel Enum specifying the clear setting mode.
 * @param sdo_dis Enum indicating if the SDO is disabled or enabled.
 * @param dac_ch_pwr_states Array specifying the power-up states of each DAC
 * channel.
 * @param int_ref_pwrup Enum indicating the power-up state of the internal
 * reference.
 * @param encoding Enum specifying the encoding scheme used by the DAC.
 * @param vref_mv Reference voltage in millivolts.
 ******************************************************************************/
struct ad5754r_dev {
	/* SPI Descriptor */
	struct no_os_spi_desc	*spi_desc;
	/* Clear GPIO */
	struct no_os_gpio_desc *gpio_clear;
	/* Load DAC */
	struct no_os_gpio_desc *gpio_ldac;
	/* Output Range Select */
	enum ad5754r_dac_ch_range dac_ch_range[AD5754R_NUM_CHANNELS];
	/* Current Clamp Enable */
	enum ad5754r_ctrl_current_clamp_en clamp_en;
	/* TSD Enable */
	enum ad5754r_ctrl_tsd_en tsd_en;
	/* Clear Setting */
	enum ad5754r_ctrl_clear_sel clear_sel;
	/* SDO Disable */
	enum ad5754r_ctrl_sdo_dis sdo_dis;
	/* Powerup States of DAC Channels */
	enum ad5754r_pwr_dac_ch_pwrup dac_ch_pwr_states[AD5754R_NUM_CHANNELS];
	/* Powerup State of Internal Reference */
	enum ad5754r_pwr_int_ref_pwrup int_ref_pwrup;
	/* Encoding scheme */
	enum ad5754r_encoding_scheme encoding;
	/* Reference voltage in millivolts */
	uint16_t vref_mv;
};

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* AD5754R gain values for different output ranges */
/***************************************************************************//**
 * @brief The `ad5754r_gain_values_scaled` is a constant array of unsigned
 * integers that holds the gain values for the AD5754R DAC for different
 * output ranges. The size of the array is determined by the macro
 * `AD5754R_SPAN_M10V8_TO_10V8 + 1`, which suggests it covers all
 * possible output range configurations defined in the
 * `ad5754r_dac_ch_range` enumeration.
 *
 * @details This array is used to store pre-calculated gain values that are
 * applied to the DAC output ranges to ensure accurate voltage scaling.
 ******************************************************************************/
extern const unsigned int ad5754r_gain_values_scaled[AD5754R_SPAN_M10V8_TO_10V8
		+ 1];

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * of the AD5754R device, identified by the instruction address. It is
 * essential to ensure that the device structure is properly initialized
 * and not null before calling this function. The function constructs a
 * command frame and sends it over SPI to the device. It should be used
 * when there is a need to configure or update the settings of the
 * AD5754R device. If the device pointer is null, the function returns an
 * error code.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param instr_addr A uint8_t value representing the instruction address of the
 * register to be written. It specifies which register or DAC
 * channel to target.
 * @param reg_val A uint16_t value containing the data to be written to the
 * specified register. It represents the new value or
 * configuration to be set.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device pointer is null.
 ******************************************************************************/
int ad5754r_write(struct ad5754r_dev *dev, uint8_t instr_addr,
		  uint16_t reg_val);

/***************************************************************************//**
 * @brief This function is used to read data from a specific register of the
 * AD5754R device. It requires a valid device structure and a pointer to
 * store the read value. The function should be called when the user
 * needs to retrieve the current value of a register. It is important to
 * ensure that the device has been properly initialized before calling
 * this function. The function will return an error if the device
 * structure or the output pointer is null.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param instr_addr A uint8_t value representing the address of the register to
 * be read. It should be a valid register address as defined
 * by the device's register map.
 * @param reg_val A pointer to a uint16_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if input parameters are invalid.
 ******************************************************************************/
int ad5754r_read(struct ad5754r_dev *dev, uint8_t instr_addr,
		 uint16_t *reg_val);

/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * AD5754R device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets the new values for
 * those bits. This function is typically used when only certain bits of
 * a register need to be changed without affecting the other bits. It
 * must be called with a valid device structure and appropriate
 * instruction address, mask, and register value. The function returns an
 * error code if the read or write operation fails.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param instr_addr The address of the register to be updated. Must be a valid
 * register address for the device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Only bits set in this mask will be modified.
 * @param reg_val The new values for the bits specified by the mask. These
 * values will be written to the register after applying the
 * mask.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int ad5754r_update_bits(struct ad5754r_dev *dev,
			uint8_t instr_addr,
			uint16_t mask,
			uint16_t reg_val);

/***************************************************************************//**
 * @brief This function updates the digital-to-analog converter (DAC) register
 * for a specified channel on the AD5754R device. It should be used when
 * you need to set a new output value for a particular DAC channel. The
 * function requires a valid device structure and a channel number within
 * the allowed range. It is important to ensure that the device is
 * properly initialized before calling this function. The function will
 * return an error code if the device pointer is null or if the channel
 * number is out of range.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param chn The channel number to update, which must be between 0 and
 * AD5754R_NUM_CHANNELS (inclusive). Invalid values will result in an
 * error.
 * @param value The 16-bit value to write to the DAC register for the specified
 * channel. The value should be within the valid range for the DAC.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input parameters.
 ******************************************************************************/
int ad5754r_update_dac_ch_register(struct ad5754r_dev *dev, uint8_t chn,
				   uint16_t value);

/***************************************************************************//**
 * @brief This function updates the digital-to-analog converter (DAC) register
 * value for all channels of the AD5754R device. It should be used when a
 * uniform value needs to be set across all DAC channels. The function
 * requires a valid device structure and a 16-bit value to be written to
 * the DAC registers. It is important to ensure that the device structure
 * is properly initialized before calling this function. The function
 * handles the necessary GPIO operations to trigger the update and
 * returns an error code if any operation fails.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The function returns -EINVAL if this parameter
 * is null.
 * @param value A 16-bit unsigned integer representing the value to be written
 * to all DAC channels. The value should be within the valid range
 * for the DAC resolution.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad5754r_update_dac_all_ch_registers(struct ad5754r_dev *dev,
					uint16_t value);

/***************************************************************************//**
 * @brief This function is used to trigger the Load DAC (LDAC) operation on the
 * AD5754R device, which updates the DAC outputs with the values in the
 * input registers. It should be called when the user wants to apply new
 * DAC settings. If a GPIO is configured for the LDAC pin, the function
 * will toggle this pin to perform the load operation. If no GPIO is
 * assigned, it will use a software command to load the DAC. The function
 * must be called with a valid device structure that has been properly
 * initialized. It returns an error code if the device structure is null
 * or if any GPIO operation fails.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The function will return -EINVAL if this
 * parameter is null. The caller retains ownership of the structure.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if a GPIO operation fails.
 ******************************************************************************/
int ad5754r_ldac_trigger(struct ad5754r_dev *dev);

/***************************************************************************//**
 * @brief This function is used to clear the output of all DAC channels on the
 * AD5754R device. It should be called when a reset of the DAC outputs is
 * required. The function first checks if a valid device structure is
 * provided. If a GPIO pin is configured for clearing, it toggles the pin
 * to perform the clear operation. If no GPIO is assigned, it uses a
 * software command to clear the outputs. This function must be called
 * with a properly initialized device structure and handles invalid
 * device pointers by returning an error code.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. If null, the function returns -EINVAL. The
 * caller retains ownership of the structure.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input is invalid.
 ******************************************************************************/
int ad5754r_clear_async(struct ad5754r_dev *dev);

/***************************************************************************//**
 * @brief Use this function to retrieve the current value stored in the DAC
 * register for a specified channel on the AD5754R device. This function
 * is typically called when you need to verify or monitor the output
 * setting of a particular DAC channel. Ensure that the device has been
 * properly initialized before calling this function. The function will
 * return an error if the channel number is out of range or if the
 * provided pointer for the value is null.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param chn The channel number to read from, ranging from 0 to
 * AD5754R_NUM_CHANNELS - 1. If the channel number is invalid, the
 * function returns an error.
 * @param value A pointer to a uint16_t where the read DAC value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the channel is
 * invalid or the value pointer is null.
 ******************************************************************************/
int ad5754r_read_dac_ch_register(struct ad5754r_dev *dev, uint8_t chn,
				 uint16_t *value);

/***************************************************************************//**
 * @brief This function is used to convert a digital-to-analog converter (DAC)
 * code into a millivolt value for a specific channel of the AD5754R
 * device. It requires a valid device structure and a channel number
 * within the range of available channels. The function calculates the
 * millivolt value based on the channel's configured output range and
 * encoding scheme. It must be called with a properly initialized device
 * structure and a non-null pointer for the output value. If the input
 * parameters are invalid, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param chn The channel number for which the conversion is to be performed.
 * Must be less than AD5754R_NUM_CHANNELS.
 * @param code The DAC code to be converted to millivolts. It is a 16-bit
 * unsigned integer.
 * @param value A pointer to a 16-bit unsigned integer where the resulting
 * millivolt value will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid.
 ******************************************************************************/
int ad5754r_dac_code_to_mvolts(struct ad5754r_dev *dev, uint8_t chn,
			       uint16_t code,
			       uint16_t *value);

/***************************************************************************//**
 * @brief This function is used to convert a given millivolt value to a
 * corresponding digital code for a specified channel of the AD5754R DAC
 * device. It is essential to ensure that the device structure and the
 * code pointer are valid and that the channel number is within the valid
 * range before calling this function. The function handles both unipolar
 * and bipolar output ranges, adjusting the encoding as necessary. It
 * returns an error code if any input validation fails.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * DAC device. Must not be null.
 * @param chn The channel number for which the conversion is to be performed.
 * Must be less than AD5754R_NUM_CHANNELS.
 * @param mvolts The input voltage in millivolts to be converted. The valid
 * range depends on the DAC's configuration.
 * @param code A pointer to a uint16_t where the resulting digital code will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the inputs are
 * invalid.
 ******************************************************************************/
int ad5754r_dac_mvolts_to_code(struct ad5754r_dev *dev, uint8_t chn,
			       uint16_t mvolts, uint16_t *code);

/***************************************************************************//**
 * @brief This function configures the output voltage range for a specified DAC
 * channel on the AD5754R device. It should be used when you need to
 * change the voltage span of a particular channel to match the
 * requirements of your application. The function must be called with a
 * valid device structure and channel number, and the channel range must
 * be within the defined enumeration. If the channel number or range is
 * invalid, the function returns an error code. Successful execution
 * updates the device's internal state to reflect the new range.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param chn The channel number to configure, ranging from 0 to
 * AD5754R_NUM_CHANNELS - 1. Values outside this range result in an
 * error.
 * @param ch_range The desired output range for the channel, specified as an
 * enum ad5754r_dac_ch_range. Must be a valid range within the
 * enum; otherwise, an error is returned.
 * @return Returns 0 on success, or a negative error code if the channel number
 * or range is invalid.
 ******************************************************************************/
int ad5754r_set_ch_range(struct ad5754r_dev *dev, uint8_t chn,
			 enum ad5754r_dac_ch_range ch_range);

/***************************************************************************//**
 * @brief This function is used to configure the power-up state of a specified
 * DAC channel on the AD5754R device. It should be called when you need
 * to change the power state of a channel, either powering it up or down.
 * The function requires a valid device structure and channel index, and
 * it will return an error if the channel index is out of range or if the
 * power-up mode is invalid. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param chn The index of the DAC channel to configure. Valid values are 0 to
 * AD5754R_NUM_CHANNELS - 1. If the value is out of range, the
 * function returns an error.
 * @param ch_pwrup The desired power-up state for the channel, specified as an
 * enum ad5754r_pwr_dac_ch_pwrup. Valid values are
 * AD5754R_PWR_DAC_CH_POWERDOWN and AD5754R_PWR_DAC_CH_POWERUP.
 * If the value is invalid, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the input
 * parameters are invalid or if an error occurs during the operation.
 ******************************************************************************/
int ad5754r_set_ch_pwrup(struct ad5754r_dev *dev, uint8_t chn,
			 enum ad5754r_pwr_dac_ch_pwrup ch_pwrup);

/***************************************************************************//**
 * @brief This function configures the current clamp setting on the AD5754R
 * device, allowing the user to enable or disable the current clamp
 * feature. It should be called when the user needs to control the
 * current clamp state of the device. The function requires a valid
 * device structure and a clamp enable setting. It returns an error code
 * if the clamp enable setting is invalid or if the operation fails. The
 * function updates the device's internal state to reflect the new clamp
 * setting.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param clamp_en An enum value of type ad5754r_ctrl_current_clamp_en
 * indicating whether to enable or disable the current clamp.
 * Must be a valid value within the enum range; otherwise, the
 * function returns an error.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid input.
 ******************************************************************************/
int ad5754r_set_current_clamp_en(struct ad5754r_dev *dev,
				 enum ad5754r_ctrl_current_clamp_en clamp_en);

/***************************************************************************//**
 * @brief This function is used to control the thermal shutdown (TSD) feature of
 * the AD5754R device. It should be called when there is a need to enable
 * or disable the TSD functionality, which is a safety feature to prevent
 * overheating. The function requires a valid device structure and a
 * control parameter indicating the desired TSD state. It must be called
 * after the device has been properly initialized. If the control
 * parameter is invalid, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param tsd_en An enum value of type ad5754r_ctrl_tsd_en indicating whether to
 * enable or disable the thermal shutdown feature. Valid values
 * are AD5754R_CTRL_TSD_DIS and AD5754R_CTRL_TSD_EN. If an invalid
 * value is provided, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the input is
 * invalid or if there is a failure in updating the device settings.
 ******************************************************************************/
int ad5754r_set_tsd_en(struct ad5754r_dev *dev,
		       enum ad5754r_ctrl_tsd_en tsd_en);

/***************************************************************************//**
 * @brief This function configures the clear mode of the AD5754R device, which
 * determines the output state when a clear command is issued. It should
 * be called when you need to change the clear mode setting of the
 * device. The function requires a valid device structure and a clear
 * mode selection. It returns an error if the clear mode selection is
 * invalid or if there is a failure in updating the device's control
 * register.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param clear_sel An enum value of type ad5754r_ctrl_clear_sel indicating the
 * desired clear mode. Valid values are AD5754R_CTRL_CLEAR_0V
 * and AD5754R_CTRL_CLEAR_MIDSCALE_CODE. If an invalid value is
 * provided, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the clear mode
 * selection is invalid or if there is a failure in updating the
 * device's control register.
 ******************************************************************************/
int ad5754r_set_clear_mode(struct ad5754r_dev *dev,
			   enum ad5754r_ctrl_clear_sel clear_sel);

/***************************************************************************//**
 * @brief Use this function to configure the Serial Data Output (SDO) line of
 * the AD5754R device to be either enabled or disabled. This function
 * should be called when you need to change the SDO state, typically
 * during device initialization or configuration. Ensure that the `dev`
 * parameter is a valid, initialized device structure. The function will
 * return an error if the `sdo_dis` parameter is not a valid value from
 * the `ad5754r_ctrl_sdo_dis` enumeration.
 *
 * @param dev A pointer to an `ad5754r_dev` structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function. The caller retains ownership.
 * @param sdo_dis An enumeration value of type `ad5754r_ctrl_sdo_dis` indicating
 * whether to enable or disable the SDO line. Valid values are
 * `AD5754R_CTRL_SDO_EN` and `AD5754R_CTRL_SDO_DIS`. If an
 * invalid value is provided, the function returns an error.
 * @return Returns 0 on success. On failure, returns a negative error code, such
 * as -EINVAL if `sdo_dis` is invalid.
 ******************************************************************************/
int ad5754r_set_sdo_disable(struct ad5754r_dev *dev,
			    enum ad5754r_ctrl_sdo_dis sdo_dis);

/***************************************************************************//**
 * @brief This function configures the internal reference power-up state of the
 * AD5754R device. It should be used when you need to enable or disable
 * the internal reference of the device. The function must be called with
 * a valid device structure and a valid power-up state enumeration. It
 * returns an error code if the power-up state is invalid or if there is
 * a failure in updating the device configuration.
 *
 * @param dev A pointer to an initialized ad5754r_dev structure representing the
 * device. Must not be null.
 * @param int_ref_pwrup An enumeration value of type ad5754r_pwr_int_ref_pwrup
 * indicating the desired power-up state of the internal
 * reference. Valid values are
 * AD5754R_PWR_INT_REF_POWERDOWN and
 * AD5754R_PWR_INT_REF_POWERUP. If an invalid value is
 * provided, the function returns an error.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ad5754r_set_int_ref_pwrup(struct ad5754r_dev *dev,
			      enum ad5754r_pwr_int_ref_pwrup int_ref_pwrup);

/***************************************************************************//**
 * @brief Use this function to check if an overcurrent condition has been
 * detected on a specific DAC channel of the AD5754R device. It is
 * essential to ensure that the `oc_ch_alert` pointer is valid and that
 * the channel number is within the valid range before calling this
 * function. The function will return an error if these conditions are
 * not met. This function is useful for monitoring the health and status
 * of the DAC channels in applications where overcurrent conditions might
 * occur.
 *
 * @param dev A pointer to an initialized `ad5754r_dev` structure representing
 * the device. The caller retains ownership and must ensure it is not
 * null.
 * @param chn The channel number to check for overcurrent alerts. Must be less
 * than `AD5754R_NUM_CHANNELS` (4). Invalid values will result in an
 * error.
 * @param oc_ch_alert A pointer to an `ad5754r_pwr_oc_ch_alert` enum where the
 * overcurrent alert status will be stored. Must not be null.
 * @return Returns 0 on success, with the overcurrent alert status stored in
 * `oc_ch_alert`. Returns a negative error code on failure, such as
 * invalid parameters.
 ******************************************************************************/
int ad5754r_get_oc_ch_alert(struct ad5754r_dev *dev, uint8_t chn,
			    enum ad5754r_pwr_oc_ch_alert *oc_ch_alert);

/***************************************************************************//**
 * @brief Use this function to check the thermal shutdown alert status of the
 * AD5754R device. It is essential to ensure that the `tsd_alert`
 * parameter is not null before calling this function, as a null pointer
 * will result in an error. This function should be called when you need
 * to monitor the thermal status of the device to prevent overheating
 * issues.
 *
 * @param dev A pointer to an `ad5754r_dev` structure representing the device.
 * The device must be properly initialized before calling this
 * function.
 * @param tsd_alert A pointer to an `enum ad5754r_pwr_tsd_alert` where the
 * thermal shutdown alert status will be stored. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The `tsd_alert` parameter is updated with the current thermal
 * shutdown alert status.
 ******************************************************************************/
int ad5754r_get_tsd_alert(struct ad5754r_dev *dev,
			  enum ad5754r_pwr_tsd_alert *tsd_alert);

/***************************************************************************//**
 * @brief This function is used to verify the SPI communication with the AD5754R
 * device by writing to and reading from the Output Range Select register
 * for DAC Channel A. It should be called when there is a need to confirm
 * that the SPI interface is functioning correctly. The function requires
 * a valid device structure and will return an error code if the
 * verification fails. It temporarily changes the output range of Channel
 * A to 0-10V for the verification process and restores it to 0-5V after
 * completion.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @return Returns 0 on successful verification, -EINVAL if the device pointer
 * is null, or other negative error codes if the SPI communication fails
 * or the readback value does not match the expected value.
 ******************************************************************************/
int ad5754r_spi_verify(struct ad5754r_dev* dev);

/***************************************************************************//**
 * @brief This function sets up the GPIOs required for the AD5754R device,
 * specifically the clear and load DAC GPIOs, based on the provided
 * initialization parameters. It should be called after the device
 * structure is allocated and before any operations that require GPIO
 * interaction. The function ensures that the GPIOs are configured as
 * outputs and set to a high state if they are available. It returns an
 * error code if any of the GPIO initializations fail or if the input
 * parameters are invalid.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @param init_param A pointer to an ad5754r_init_param structure containing the
 * initialization parameters for the device. Must not be null.
 * The function will return -EINVAL if this parameter is null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other codes for GPIO initialization
 * failures.
 ******************************************************************************/
int ad5754r_gpio_init(struct ad5754r_dev* dev,
		      struct ad5754r_init_param *init_param);

/***************************************************************************//**
 * @brief This function configures the AD5754R device registers according to the
 * provided initialization parameters. It should be called after the
 * device has been initialized and before any other operations are
 * performed on the device. The function sets the output range, power-up
 * states, and various control settings for each DAC channel. It also
 * configures global settings such as current clamp, thermal shutdown,
 * clear mode, SDO state, and internal reference power-up. If any
 * parameter is invalid or an error occurs during configuration, the
 * function returns an error code.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param init_param A pointer to an ad5754r_init_param structure containing the
 * initialization parameters. Must not be null. The function
 * returns -EINVAL if this parameter is null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * which step failed.
 ******************************************************************************/
int ad5754r_reg_init(struct ad5754r_dev* dev,
		     struct ad5754r_init_param *init_param);

/***************************************************************************//**
 * @brief This function sets up the AD5754R device by initializing its GPIO and
 * SPI interfaces, verifying SPI communication, and configuring device
 * registers based on the provided initialization parameters. It must be
 * called before any other operations on the device. The function
 * allocates memory for the device structure and assigns it to the
 * provided pointer. If initialization fails at any step, it cleans up
 * allocated resources and returns an error code.
 *
 * @param device A pointer to a pointer of type `struct ad5754r_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad5754r_init_param` containing
 * initialization parameters for the device. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and does not modify the `device` pointer.
 ******************************************************************************/
int ad5754r_init(struct ad5754r_dev **device,
		 struct ad5754r_init_param *init_param);

/***************************************************************************//**
 * @brief This function is used to release the GPIO resources that were
 * allocated for the AD5754R device. It should be called when the GPIOs
 * are no longer needed, typically during the cleanup phase of the device
 * lifecycle. The function checks if the device structure is valid and if
 * the GPIOs were initialized, removing them if they exist. It returns an
 * error code if the device is null or if any GPIO removal operation
 * fails.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device.
 * Must not be null. The function will return an error if this
 * parameter is null.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if any GPIO removal operation fails.
 ******************************************************************************/
int ad5754r_remove_gpios(struct ad5754r_dev *dev);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD5754R device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that all allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks in
 * your application.
 *
 * @param dev A pointer to an ad5754r_dev structure representing the device to
 * be removed. This pointer must not be null, and it should point to
 * a valid, initialized device structure. The function will handle
 * invalid pointers by returning an error code.
 * @return Returns 0 on success or a negative error code if the removal process
 * fails at any step.
 ******************************************************************************/
int ad5754r_remove(struct ad5754r_dev *dev);

#endif /* __AD5754R_H__ */
