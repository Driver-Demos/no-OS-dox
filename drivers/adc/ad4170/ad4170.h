/***************************************************************************//**
 *   @file   ad4170.h
 *   @brief  Header file for the ad4170 driver.
 *   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
 * Copyright 2020, 2023(c) Analog Devices, Inc.
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
#ifndef AD4170_H_
#define AD4170_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_util.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

#ifndef ECOMM
#define ECOMM 70
#endif

#define AD4170_R1B				(1ul << 16)
#define AD4170_R2B				(2ul << 16)
#define AD4170_R3B				(3ul << 16)
#define AD4170_R4B				(4ul << 16)
#define AD4170_TRANSF_LEN(x)			((x) >> 16)
#define AD4170_ADDR(x)				((x) & 0xFFFF)

#define AD4170_SPI_SYNC_PATTERN			0x2645

#define AD4170_REG_READ_6(x)			(((x) & 0x3F) | 0x40)
#define AD4170_REG_WRITE_6(x)			((x) & 0x3F)
#define AD4170_REG_READ_14(x)			(((x) & 0x3FFF) | 0x4000)
#define AD4170_REG_WRITE_14(x)			((x) & 0x3FFF)
#define AD4170_REG_INTERFACE_CONFIG_A		(AD4170_R1B | 0x00)
#define AD4170_REG_INTERFACE_CONFIG_B		(AD4170_R1B | 0x01)
#define AD4170_REG_DEVICE_CONFIG		(AD4170_R1B | 0x02)
#define AD4170_REG_CHIP_TYPE			(AD4170_R1B | 0x03)
#define AD4170_REG_PRODUCT_ID_L			(AD4170_R1B | 0x04)
#define AD4170_REG_PRODUCT_ID_H			(AD4170_R1B | 0x05)
#define AD4170_REG_CHIP_GRADE			(AD4170_R1B | 0x06)
#define AD4170_REG_SCRATCH_PAD			(AD4170_R1B | 0x0a)
#define AD4170_REG_SPI_REVISION			(AD4170_R1B | 0x0b)
#define AD4170_REG_VENDOR_L			(AD4170_R1B | 0x0c)
#define AD4170_REG_VENDOR_H			(AD4170_R1B | 0x0d)
#define AD4170_REG_INTERFACE_CONFIG_C		(AD4170_R1B | 0x10)
#define AD4170_REG_INTERFACE_STATUS_A		(AD4170_R1B | 0x11)
#define AD4170_REG_DATA_STATUS			(AD4170_R2B | 0x14)
#define AD4170_REG_DATA_16b			(AD4170_R2B | 0x16)
#define AD4170_REG_DATA_16b_STATUS		(AD4170_R3B | 0x18)
#define AD4170_REG_DATA_24b			(AD4170_R3B | 0x1c)
#define AD4170_REG_DATA_24b_STATUS		(AD4170_R4B | 0x20)
#define AD4170_REG_DATA_32b			(AD4170_R4B | 0x24)
#define AD4170_REG_DATA_PER_CHANNEL(ch)		(AD4170_R3B | (0x28 + 4 * (ch)))
#define AD4170_REG_PIN_MUXING			(AD4170_R2B | 0x68)
#define AD4170_REG_CLOCK_CTRL			(AD4170_R2B | 0x6a)
#define AD4170_REG_STANDBY_CTRL			(AD4170_R2B | 0x6c)
#define AD4170_REG_POWER_DOWN_SW		(AD4170_R2B | 0x6e)
#define AD4170_REG_ADC_CTRL			(AD4170_R2B | 0x70)
#define AD4170_REG_ERROR_EN			(AD4170_R2B | 0x72)
#define AD4170_REG_ERROR			(AD4170_R2B | 0x74)
#define AD4170_REG_CHANNEL_EN			(AD4170_R2B | 0x78)
#define AD4170_REG_ADC_CHANNEL_SETUP(ch)	(AD4170_R2B | (0x80 + 4 * (ch)))
#define AD4170_REG_ADC_CHANNEL_MAP(ch)		(AD4170_R2B | (0x82 + 4 * (ch)))
#define AD4170_REG_ADC_SETUPS_MISC(n)		(AD4170_R2B | (0xc0 + 14 * (n)))
#define AD4170_REG_ADC_SETUPS_AFE(n)		(AD4170_R2B | (0xc2 + 14 * (n)))
#define AD4170_REG_ADC_SETUPS_FILTER(n)		(AD4170_R2B | (0xc4 + 14 * (n)))
#define AD4170_REG_ADC_SETUPS_FILTER_FS(n)	(AD4170_R2B | (0xc6 + 14 * (n)))
#define AD4170_REG_ADC_SETUPS_OFFSET(n)		(AD4170_R3B | (0xc8 + 14 * (n)))
#define AD4170_REG_ADC_SETUPS_GAIN(n)		(AD4170_R3B | (0xcb + 14 * (n)))
#define AD4170_REG_REF_CONTROL			(AD4170_R2B | 0x130)
#define AD4170_REG_V_BIAS			(AD4170_R2B | 0x134)
#define AD4170_REG_I_PULLUP			(AD4170_R2B | 0x136)
#define AD4170_REG_CURRENT_SOURCE(n)		(AD4170_R2B | (0x138 + 2 * (n)))
#define AD4170_REG_FIR_CONTROL			(AD4170_R2B | 0x140)
#define AD4170_REG_COEFF_WRITE_DATA		(AD4170_R4B | 0x143)
#define AD4170_REG_COEFF_READ_DATA		(AD4170_R4B | 0x147)
#define AD4170_REG_COEFF_ADDRESS		(AD4170_R2B | 0x14b)
#define AD4170_REG_COEFF_WRRD_STB		(AD4170_R2B | 0x14d)
#define AD4170_REG_DAC_SPAN			(AD4170_R2B | 0x150)
#define AD4170_REG_DAC_CHANNEL_EN		(AD4170_R2B | 0x152)
#define AD4170_REG_DAC_HW_TOGGLE_MASK		(AD4170_R2B | 0x154)
#define AD4170_REG_DAC_HW_LDAC_MASK		(AD4170_R2B | 0x156)
#define AD4170_REG_DAC_DATA(ch)			(AD4170_R2B | (0x158 + 2 * (ch)))
#define AD4170_REG_DAC_SW_TOGGLE_TRIGGERS	(AD4170_R2B | 0x168)
#define AD4170_REG_DAC_SW_LDAC_TRIGGERS		(AD4170_R2B | 0x16a)
#define AD4170_REG_DAC_INPUTA(ch)		(AD4170_R2B | (0x16c + 2 * (ch)))
#define AD4170_REG_DAC_INPUTB(ch)		(AD4170_R2B | (0x17c + 2 * (ch)))
#define AD4170_REG_GPIO_MODE			(AD4170_R2B | 0x190)
#define AD4170_REG_OUTPUT_DATA			(AD4170_R2B | 0x192)
#define AD4170_REG_INPUT_DATA			(AD4170_R2B | 0x194)

/* AD4170_REG_INTERFACE_CONFIG_A */
#define AD4170_SW_RESET_MSK					NO_OS_BIT(7)
#define AD4170_ADDR_ASCENSION_MSK				NO_OS_BIT(5)
#define AD4170_SDO_ENABLE_MSK					NO_OS_BIT(4)
#define AD4170_SW_RESETX_MSK					NO_OS_BIT(0)

/* AD4170_REG_INTERFACE_CONFIG_B */
#define AD4170_INTERFACE_CONFIG_B_SINGLE_INST_MSK		NO_OS_BIT(7)
#define AD4170_INTERFACE_CONFIG_B_SHORT_INSTRUCTION_MSK		NO_OS_BIT(3)
#define AD4170_INTERFACE_CONFIG_B_SHORT_INSTRUCTION(x)		(((x) & 0x1) << 3)

/* AD4170_REG_INTERFACE_CONFIG_C */
#define AD4170_INTERFACE_CONFIG_C_CRC_MSK			(NO_OS_BIT(7) | NO_OS_BIT(6) | NO_OS_BIT(1) | NO_OS_BIT(0))
#define AD4170_INTERFACE_CONFIG_C_CRC(x)			(((~x) & 0x3) | (((x) << 6) & 0xC0))
#define AD4170_INTERFACE_CONFIG_C_STRICT_REG_ACCESS_MSK		NO_OS_BIT(5)
#define AD4170_INTERFACE_CONFIG_C_STRICT_REG_ACCESS(x)		(((x) & 0x1) << 5)
#define AD4170_CRC8_POLYNOMIAL					0x7
#define AD4170_CRC8_INITIAL_VALUE				0xA5

/* AD4170_REG_INTERFACE_STATUS_A */
#define AD4170_INTERFACE_STATUS_A_NOT_READY_ERR_MSK		NO_OS_BIT(7)
#define AD4170_INTERFACE_STATUS_A_CLOCK_COUNT_ERR_MSK		NO_OS_BIT(4)
#define AD4170_INTERFACE_STATUS_A_CRC_ERR_MSK			NO_OS_BIT(3)
#define AD4170_INTERFACE_STATUS_A_INVALID_ACCESS_ERR_MSK	NO_OS_BIT(2)
#define AD4170_INTERFACE_STATUS_A_PARTIAL_ACCESS_ERR_MSK	NO_OS_BIT(1)
#define AD4170_INTERFACE_STATUS_A_ADDR_INVALID_ERR_MSK		NO_OS_BIT(0)

/* AD4170_REG_PRODUCT_ID */
#define AD4170_PRODUCT_ID_L_VALUE				0x40
#define AD4170_PRODUCT_ID_H_VALUE				0x0

/* AD4190_REG_PRODUCT_ID */
#define AD4190_PRODUCT_ID_L_VALUE				0x48
#define AD4190_PRODUCT_ID_H_VALUE				0x0

/* AD4170_REG_DATA_STATUS */
#define AD4170_DATA_STATUS_MASTER_ERR_S_MSK			NO_OS_BIT(7)
#define AD4170_DATA_STATUS_POR_FLAG_S_MSK			NO_OS_BIT(6)
#define AD4170_DATA_STATUS_RDYB_MSK				NO_OS_BIT(5)
#define AD4170_DATA_STATUS_SETTLED_FIR_MSK			NO_OS_BIT(4)
#define AD4170_DATA_STATUS_CH_ACTIVE_MSK			NO_OS_GENMASK(3,0)

/* AD4170_REG_PIN_MUXING */
#define AD4170_PIN_MUXING_CHAN_TO_GPIO_MSK			NO_OS_BIT(14)
#define AD4170_PIN_MUXING_DIG_AUX2_CTRL_MSK			NO_OS_GENMASK(7,6)
#define AD4170_PIN_MUXING_DIG_AUX1_CTRL_MSK			NO_OS_GENMASK(5,4)
#define AD4170_PIN_MUXING_SYNC_CTRL_MSK				NO_OS_GENMASK(3,2)
#define AD4170_PIN_MUXING_DIG_OUT_STR_MSK			NO_OS_BIT(1)
#define AD4170_PIN_MUXING_SDO_RDBY_DLY_MSK			NO_OS_BIT(0)

/* AD4170_REG_CLOCK_CTRL */
#define AD4170_CLOCK_CTRL_DCLK_DIVIDE_MSK			NO_OS_GENMASK(7,6)
#define AD4170_CLOCK_CTRL_CLOCKDIV_MSK				NO_OS_GENMASK(5,4)
#define AD4170_CLOCK_CTRL_CLOCKSEL_MSK				NO_OS_GENMASK(1,0)

/* AD4170_REG_STANDBY_CTRL */
#define AD4170_STANDBY_CTRL_STB_EN_CLOCK_MSK			NO_OS_BIT(8)
#define AD4170_STANDBY_CTRL_STB_EN_IPULLUP_MSK			NO_OS_BIT(7)
#define AD4170_STANDBY_CTRL_STB_EN_DIAGNOSTICS_MSK		NO_OS_BIT(6)
#define AD4170_STANDBY_CTRL_STB_EN_DAC_MSK			NO_OS_BIT(5)
#define AD4170_STANDBY_CTRL_STB_EN_PDSW1_MSK			NO_OS_BIT(4)
#define AD4170_STANDBY_CTRL_STB_EN_PDSW0_MSK			NO_OS_BIT(3)
#define AD4170_STANDBY_CTRL_STB_EN_VBIAS_MSK			NO_OS_BIT(2)
#define AD4170_STANDBY_CTRL_STB_EN_IEXC_MSK			NO_OS_BIT(1)
#define AD4170_STANDBY_CTRL_STB_EN_REFERENCE_MSK		NO_OS_BIT(0)

/* AD4170_REG_POWER_DOWN_SW */
#define AD4170_POWER_DOWN_SW_PDSW1_MSK				NO_OS_BIT(1)
#define AD4170_POWER_DOWN_SW_PDSW0_MSK				NO_OS_BIT(0)

/* AD4170_REG_ADC_CTRL */
#define AD4170_ADC_CTRL_PARALLEL_FILT_EN_MSK			NO_OS_BIT(8)
#define AD4170_ADC_CTRL_MULTI_DATA_REG_SEL_MSK			NO_OS_BIT(7)
#define AD4170_ADC_CTRL_CONT_READ_STATUS_EN_MSK			NO_OS_BIT(6)
#define AD4170_REG_CTRL_CONT_READ_MSK				NO_OS_GENMASK(5,4)
#define AD4170_REG_CTRL_MODE_MSK				NO_OS_GENMASK(3,0)

/* AD4170_REG_ERROR_EN and AD4170_REG_ERROR */
#define AD4170_ERROR_DEVICE_ERROR_MSK				NO_OS_BIT(15)
#define AD4170_ERROR_DLDO_PSM_ERR_MSK				NO_OS_BIT(13)
#define AD4170_ERROR_ALDO_PSM_ERR_MSK				NO_OS_BIT(12)
#define AD4170_ERROR_IOUT3_COMP_ERR_MSK				NO_OS_BIT(11)
#define AD4170_ERROR_IOUT2_COMP_ERR_MSK				NO_OS_BIT(10)
#define AD4170_ERROR_IOUT1_COMP_ERR_MSK				NO_OS_BIT(9)
#define AD4170_ERROR_IOUT0_COMP_ERR_MSK				NO_OS_BIT(8)
#define AD4170_ERROR_REF_DIFF_MIN_ERR_MSK			NO_OS_BIT(7)
#define AD4170_ERROR_REF_OV_UV_ERR_MSK				NO_OS_BIT(6)
#define AD4170_ERROR_AINM_OV_UV_ERR_MSK				NO_OS_BIT(5)
#define AD4170_ERROR_AINP_OV_UV_ERR_MSK				NO_OS_BIT(4)
#define AD4170_ERROR_ADC_CONV_ERR_MSK				NO_OS_BIT(3)
#define AD4170_ERROR_MM_CRC_ERR_MSK				NO_OS_BIT(1)
#define AD4170_ERROR_ROM_CRC_ERR_MSK				NO_OS_BIT(0)

/* AD4170_REG_CHANNEL_EN */
#define AD4170_CHANNEL(ch)					NO_OS_BIT(ch)

/* AD4170_REG_ADC_CHANNEL_SETUP */
#define AD4170_CHANNEL_SETUPN_REPEAT_N_MSK			NO_OS_GENMASK(15,8)
#define AD4170_CHANNEL_SETUPN_DELAY_N_MSK			NO_OS_GENMASK(6,4)
#define AD4170_CHANNEL_SETUPN_SETUP_N_MSK			NO_OS_GENMASK(2,0)

/* AD4170_REG_ADC_CHANNEL_MAP */
#define AD4170_CHANNEL_MAPN_AINP_MSK				NO_OS_GENMASK(12,8)
#define AD4170_CHANNEL_MAPN_AINM_MSK				NO_OS_GENMASK(4,0)

/* AD4170_REG_ADC_SETUPS_MISC */
#define AD4170_ADC_SETUPS_MISC_CHOP_IEXC_MSK			NO_OS_GENMASK(15,14)
#define AD4170_ADC_SETUPS_MISC_CHOP_ADC_MSK			NO_OS_GENMASK(9,8)
#define AD4170_ADC_SETUPS_MISC_BURNOUT_MSK			NO_OS_GENMASK(1,0)

/* AD4170_REG_ADC_SETUPS_AFE */
#define AD4170_ADC_SETUPS_AFE_REF_BUF_M_MSK			NO_OS_GENMASK(11,10)
#define AD4170_ADC_SETUPS_AFE_REF_BUF_P_MSK			NO_OS_GENMASK(9,8)
#define AD4170_ADC_SETUPS_AFE_REF_SELECT_MSK			NO_OS_GENMASK(6,5)
#define AD4170_ADC_SETUPS_AFE_BIPOLAR_MSK			NO_OS_BIT(4)
#define AD4170_ADC_SETUPS_AFE_PGA_GAIN_MSK			NO_OS_GENMASK(3,0)

/* AD4170_REG_ADC_SETUPS_FILTER */
#define AD4170_ADC_SETUPS_POST_FILTER_SEL_MSK			NO_OS_GENMASK(7,4)
#define AD4170_ADC_SETUPS_FILTER_TYPE_MSK			NO_OS_GENMASK(3,0)

/* AD4170_REG_CURRENT_SOURCE */
#define AD4170_CURRENT_SOURCE_I_OUT_PIN_MSK			NO_OS_GENMASK(12,8)
#define AD4170_CURRENT_SOURCE_I_OUT_VAL_MSK			NO_OS_GENMASK(2,0)

/* AD4170_REG_REF_CONTROL */
#define AD4170_REF_CONTROL_REF_EN_MSK				NO_OS_BIT(0)

/* AD4170_REG_FIR_CONTROL */
#define AD4170_FIR_CONTROL_IIR_MODE_MSK				NO_OS_BIT(15)
#define AD4170_FIR_CONTROL_FIR_MODE_MSK				NO_OS_GENMASK(14,12)
#define AD4170_FIR_CONTROL_COEFF_SET_MSK			NO_OS_BIT(10)
#define AD4170_FIR_CONTROL_FIR_LENGTH_MSK			NO_OS_GENMASK(6,0)

/* AD4170_REG_DAC_SPAN */
#define AD4170_REG_DAC_SPAN_DAC_GAIN_MSK			NO_OS_BIT(0)

/* AD4170_REG_DAC_CHANNEL_EN */
#define AD4170_REG_DAC_CHANNEL_EN_DAC_EN_MSK			NO_OS_BIT(0)

/* AD4170_REG_DAC_HW_TOGGLE_MASK */
#define AD4170_REG_DAC_HW_TOGGLE_MASK_HW_TOGGLE_EN_MSK		NO_OS_BIT(0)

/* AD4170_REG_DAC_HW_LDAC_MASK */
#define AD4170_REG_DAC_HW_LDAC_MASK_HW_LDAC_EN_MSK		NO_OS_BIT(0)

/* AD4170_REG_DAC_DATA */
#define AD4170_REG_DAC_DATA_MSK					NO_OS_GENMASK(11,0)

/* AD4170_REG_DAC_SW_TOGGLE_TRIGGERS */
#define AD4170_REG_DAC_SW_TOGGLE_TRIGGERS_SW_TOGGLE_MSK		NO_OS_BIT(0)

/* AD4170_REG_DAC_SW_LDAC_TRIGGERS */
#define AD4170_REG_DAC_SW_LDAC_TRIGGERS_SW_LDAC_EN_MSK		NO_OS_BIT(0)

#define AD4170_NUM_CHANNELS					16
#define AD4170_NUM_SETUPS					8
#define AD4170_NUM_CURRENT_SOURCE				4
#define AD4170_FIR_COEFF_MAX_LENGTH				72

/***************************************************************************//**
 * @brief The `ad4170_chan_to_gpio` is an enumeration that defines the
 * configuration for whether the active channel of the AD4170 device is
 * output to the GPIO pins or not. It provides two possible states: one
 * where the active channel is not output to the GPIO pins, and another
 * where it is. This configuration is useful for controlling the output
 * behavior of the device in relation to its GPIO interface.
 *
 * @param AD4170_CHANNEL_NOT_TO_GPIO Indicates that the active channel is not
 * output to GPIO pins.
 * @param AD4170_CHANNEL_TO_GPIO Indicates that the active channel is output to
 * GPIO pins.
 ******************************************************************************/
enum ad4170_chan_to_gpio {
	/** Active Channel is Not Output to GPIO Pins. */
	AD4170_CHANNEL_NOT_TO_GPIO,
	/** Active Channel is Output to GPIO Pins. */
	AD4170_CHANNEL_TO_GPIO
};

/***************************************************************************//**
 * @brief The `ad4170_dig_aux2_ctrl` enumeration defines the possible
 * configurations for the DIG_AUX2 pin on the AD4170 device. This pin can
 * be set to different modes, including being disabled, acting as a DAC
 * LDAC input, serving as a START input, or functioning as a modulator
 * data output. This flexibility allows the pin to be used in various
 * roles depending on the specific application requirements.
 *
 * @param AD4170_DIG_AUX2_DISABLED Represents the state where the Dig_Aux2 pin
 * is disabled and in high impedance.
 * @param AD4170_DIG_AUX2_LDAC Configures the Dig_Aux2 pin as a DAC LDAC input.
 * @param AD4170_DIG_AUX2_SYNC Configures the Dig_Aux2 pin as a START input.
 * @param AD4170_DIG_AUX2_MODOUT Configures the Dig_Aux2 pin as a modulator data
 * output.
 ******************************************************************************/
enum ad4170_dig_aux2_ctrl {
	/** Dig_Aux2 Pin Disabled. High Impedance */
	AD4170_DIG_AUX2_DISABLED,
	/** Dig_Aux2 Pin Configured as DAC LDAC Input. */
	AD4170_DIG_AUX2_LDAC,
	/** Dig_Aux2 Pin Configured as START Input. */
	AD4170_DIG_AUX2_SYNC,
	/** Dig_Aux2 Pin Configured as Modulator Data Output. */
	AD4170_DIG_AUX2_MODOUT
};

/***************************************************************************//**
 * @brief The `ad4170_dig_aux1_ctrl` enumeration defines the possible
 * configurations for the DIG_AUX1 pin on the AD4170 device. This pin can
 * be set to different modes, including being disabled (high impedance),
 * or configured as an output for ADC Data Ready, SYNC_OUT, or Modulator
 * Data. This allows for flexible use of the DIG_AUX1 pin depending on
 * the specific application requirements.
 *
 * @param AD4170_DIG_AUX1_DISABLED Represents the state where the Dig_Aux1 pin
 * is disabled and in high impedance.
 * @param AD4170_DIG_AUX1_RDY Configures the Dig_Aux1 pin as an ADC Data Ready
 * output.
 * @param AD4170_DIG_AUX1_SYNC Configures the Dig_Aux1 pin as a SYNC_OUT output.
 * @param AD4170_DIG_AUX1_MODOUT Configures the Dig_Aux1 pin as a Modulator Data
 * output.
 ******************************************************************************/
enum ad4170_dig_aux1_ctrl {
	/** Dig_Aux1 Pin Disabled. High Impedance */
	AD4170_DIG_AUX1_DISABLED,
	/** Dig_Aux1 Pin Configured as ADC Data Ready Output. */
	AD4170_DIG_AUX1_RDY,
	/** Dig_Aux1 Pin Configured as SYNC_OUT Output. */
	AD4170_DIG_AUX1_SYNC,
	/** Dig_Aux1 Pin Configured Modulator Data Output. */
	AD4170_DIG_AUX1_MODOUT
};

/***************************************************************************//**
 * @brief The `ad4170_sync_ctrl` is an enumeration that defines the
 * configuration options for the SYNC_IN pin used for ADC synchronization
 * in the AD4170 device. It provides three possible states: disabling the
 * SYNC_IN pin, enabling it with standard synchronization functionality,
 * or enabling it with an alternative synchronization functionality. This
 * allows for flexible control over the synchronization behavior of the
 * ADC, depending on the specific application requirements.
 *
 * @param AD4170_SYNC_DISABLED SYNC_IN Pin Disabled.
 * @param AD4170_SYNC_STANDARD SYNC_IN Has Default SYNC Functionality.
 * @param AD4170_SYNC_ALTERNATE SYNC_IN Has Alternative SYNC Functionality.
 ******************************************************************************/
enum ad4170_sync_ctrl {
	/** SYNC_IN Pin Disabled. */
	AD4170_SYNC_DISABLED,
	/** SYNC_IN Has Default SYNC Functionality. */
	AD4170_SYNC_STANDARD,
	/** SYNC_IN Has Alternative SYNC Functionality. */
	AD4170_SYNC_ALTERNATE
};

/***************************************************************************//**
 * @brief The `ad4170_dig_out_str` is an enumeration that defines the drive
 * strength settings for the digital outputs of the AD4170 device. It
 * provides two options: a default drive strength suitable for higher
 * IOVDD voltages and a high drive strength for increased output
 * capability. This configuration is crucial for optimizing the
 * performance of digital signals in various voltage environments.
 *
 * @param AD4170_DIG_STR_DEFAULT Represents the default drive strength,
 * recommended for higher IOVDD voltages.
 * @param AD4170_DIG_STR_HIGH Represents an increased drive strength.
 ******************************************************************************/
enum ad4170_dig_out_str {
	/** Default Drive Strength. Recommended for higher IOVDD voltages. */
	AD4170_DIG_STR_DEFAULT,
	/** Increased Drive Strength. */
	AD4170_DIG_STR_HIGH
};

/***************************************************************************//**
 * @brief The `ad4170_sdo_rdby_dly` enumeration defines two possible reset
 * conditions for the AD4170 interface: one triggered by the last SCLK
 * and the other by the /CS edge. This allows for flexibility in how the
 * interface reset is managed, depending on the specific requirements of
 * the application.
 *
 * @param AD4170_SDO_RDY_SCLK Represents the option to reset on the last SCLK.
 * @param AD4170_SDO_RDY_CSB Represents the option to reset on the /CS edge.
 ******************************************************************************/
enum ad4170_sdo_rdby_dly {
	/** Reset on Last SCLK. */
	AD4170_SDO_RDY_SCLK,
	/** Reset on /CS Edge. */
	AD4170_SDO_RDY_CSB
};

/***************************************************************************//**
 * @brief The `ad4170_pin_muxing` structure is used to configure various pin
 * functionalities and settings for the AD4170 device. It includes
 * settings for outputting the current channel number to GPIO pins,
 * configuring auxiliary digital pins (DIG_AUX1 and DIG_AUX2),
 * synchronizing the ADC with the SYNC_IN pin, adjusting the drive
 * strength of digital outputs, and resetting the interface based on CS
 * or SCLK signals. This structure is essential for managing the pin
 * multiplexing and ensuring proper communication and functionality of
 * the AD4170 device.
 *
 * @param chan_to_gpio Enables the current channel number to be output to GPIO
 * pins.
 * @param dig_aux2_ctrl Configures the functionality of the DIG_AUX2 pin.
 * @param dig_aux1_ctrl Configures the functionality of the DIG_AUX1 pin.
 * @param sync_ctrl Configures the SYNC_IN pin for ADC synchronization.
 * @param dig_out_str Configures the drive strength of the digital outputs.
 * @param sdo_rdby_dly Resets the interface on CS or SCLK.
 ******************************************************************************/
struct ad4170_pin_muxing {
	/** Enables Current Channel Number Be Output to GPIO Pins. */
	enum ad4170_chan_to_gpio chan_to_gpio;
	/** Configures Functionality of DIG_AUX2 Pin. */
	enum ad4170_dig_aux2_ctrl dig_aux2_ctrl;
	/** Configures Functionality of DIG_AUX1 Pin. */
	enum ad4170_dig_aux1_ctrl dig_aux1_ctrl;
	/** Configures SYNC_IN Pin for ADC Synchronization. */
	enum ad4170_sync_ctrl sync_ctrl;
	/** Configures the drive strength of the Digital Outputs. */
	enum ad4170_dig_out_str dig_out_str;
	/** Reset Interface on CS or SCLK. */
	enum ad4170_sdo_rdby_dly sdo_rdby_dly;
};

/***************************************************************************//**
 * @brief The `ad4170_dclk_div` enumeration defines the possible division ratios
 * for the data clock (DCLK) relative to the master clock in the AD4170
 * device. This allows for configuring the speed of the data clock by
 * selecting one of the predefined division factors, which can be 1, 2,
 * 4, or 8. This configuration is crucial for synchronizing data
 * transmission with the master clock, ensuring proper timing and data
 * integrity in communication with the device.
 *
 * @param AD4170_DCLKDIVBY1 DCLK equals master clock divided by 1.
 * @param AD4170_DCLKDIVBY2 DCLK equals master clock divided by 2.
 * @param AD4170_DCLKDIVBY4 DCLK equals master clock divided by 4.
 * @param AD4170_DCLKDIVBY8 DCLK equals master clock divided by 8.
 ******************************************************************************/
enum ad4170_dclk_div {
	/** DCLK Equals Master Clock Divide by 1. */
	AD4170_DCLKDIVBY1,
	/** DCLK Equals Master Clock Divide by 2. */
	AD4170_DCLKDIVBY2,
	/** DCLK Equals Master Clock Divide by 4. */
	AD4170_DCLKDIVBY4,
	/** DCLK Equals Master Clock Divide by 8. */
	AD4170_DCLKDIVBY8
};

/***************************************************************************//**
 * @brief The `ad4170_mclk_div` enumeration defines the possible division
 * factors for the master clock in the AD4170 device. It allows the
 * selection of different clock division ratios, which can be used to
 * adjust the clock speed for various operational requirements. This
 * enumeration is part of the AD4170 driver, which is used to configure
 * and control the AD4170 device, a high-performance analog-to-digital
 * converter.
 *
 * @param AD4170_CLKDIVBY1 Represents a clock division factor of 1.
 * @param AD4170_CLKDIVBY2 Represents a clock division factor of 2.
 * @param AD4170_CLKDIVBY4 Represents a clock division factor of 4.
 * @param AD4170_CLKDIVBY8 Represents a clock division factor of 8.
 ******************************************************************************/
enum ad4170_mclk_div {
	/** Divide by 1. */
	AD4170_CLKDIVBY1,
	/** Divide by 2. */
	AD4170_CLKDIVBY2,
	/** Divide by 4. */
	AD4170_CLKDIVBY4,
	/** Divide by 8. */
	AD4170_CLKDIVBY8
};

/***************************************************************************//**
 * @brief The `ad4170_clocksel` enumeration defines the possible clock source
 * selections for the AD4170 ADC. It allows the user to choose between
 * using the internal oscillator, the internal oscillator with output, an
 * external clock input, or an external crystal. This selection is
 * crucial for configuring the clocking mechanism of the ADC, which can
 * impact the performance and integration of the device in various
 * applications.
 *
 * @param AD4170_INTERNAL_OSC Represents the internal oscillator option for the
 * ADC clock.
 * @param AD4170_INTERNAL_OSC_OUTPUT Represents the internal oscillator with
 * output to the XTAL2/CLKIO pin.
 * @param AD4170_EXTERNAL_OSC Represents the external clock input on the
 * XTAL2/CLKIO pin.
 * @param AD4170_EXTERNAL_XTAL Represents the external crystal on XTAL1 and
 * XTAL2/CLKIO pins.
 ******************************************************************************/
enum ad4170_clocksel {
	/** Internal Oscillator. */
	AD4170_INTERNAL_OSC,
	/** Internal Oscillator, Output to XTAL2/CLKIO Pin. */
	AD4170_INTERNAL_OSC_OUTPUT,
	/** External Clock Input on XTAL2/CLKIO Pin. */
	AD4170_EXTERNAL_OSC,
	/** External Crystal on XTAL1 and XTAL2/CLKIO Pins. */
	AD4170_EXTERNAL_XTAL
};

/***************************************************************************//**
 * @brief The `ad4170_clock_ctrl` structure is used to configure the clock
 * control settings for the AD4170 device. It includes settings for
 * dividing the continuous transmit data clock, the master clock, and
 * selecting the ADC clock source. These settings are crucial for
 * managing the timing and synchronization of data transmission and
 * processing within the device, ensuring that the ADC operates correctly
 * with the desired clock configurations.
 *
 * @param dclk_divide Specifies the continuous transmit data clock divider using
 * the ad4170_dclk_div enumeration.
 * @param clockdiv Defines the master clock divider using the ad4170_mclk_div
 * enumeration.
 * @param clocksel Selects the ADC clock source using the ad4170_clocksel
 * enumeration.
 ******************************************************************************/
struct ad4170_clock_ctrl {
	/** Continuous Transmit Data Clock Divider. */
	enum ad4170_dclk_div dclk_divide;
	/** Master Clock Divider. */
	enum ad4170_mclk_div clockdiv;
	/** ADC Clock Select. */
	enum ad4170_clocksel clocksel;
};

/***************************************************************************//**
 * @brief The `ad4170_cont_read` enumeration defines the modes for configuring
 * continuous data register read or transmit operations for the AD4170
 * device. It provides options to either disable continuous operations,
 * enable continuous reading over SPI, or enable continuous transmission
 * over TDM, allowing for flexible data handling based on the
 * communication protocol used.
 *
 * @param AD4170_CONT_READ_OFF Disables continuous read/transmit of the ADC Data
 * register.
 * @param AD4170_CONT_READ_ON Enables continuous read of the ADC Data register
 * over SPI.
 * @param AD4170_CONT_TRANSMIT_ON Enables continuous transmit of the ADC Data
 * register over TDM.
 ******************************************************************************/
enum ad4170_cont_read {
	/** Disable Continuous Read/Transmit. */
	AD4170_CONT_READ_OFF,
	/** Enable Continuous Read. This enables continuous read of the ADC Data register over SPI. */
	AD4170_CONT_READ_ON,
	/** Enable Continuous Transmit. This enables continuous transmit of the ADC Data register over TDM. */
	AD4170_CONT_TRANSMIT_ON
};

/***************************************************************************//**
 * @brief The `ad4170_mode` enumeration defines various operating modes for the
 * AD4170 ADC, including continuous conversion modes with different
 * filters (Sinc, FIR, IIR), single conversion mode, standby, power-down,
 * idle, and calibration modes. Each mode specifies a distinct
 * operational state or function of the ADC, allowing for flexible
 * configuration based on application requirements.
 *
 * @param AD4170_MODE_CONT Continuous Conversion Mode using Sinc-based filters.
 * @param AD4170_MODE_CONT_FIR Continuous Conversion Mode using the FIR Filter.
 * @param AD4170_MODE_CONT_IIR Continuous Conversion Mode using the IIR Filter.
 * @param AD4170_MODE_SINGLE Single conversion mode using Sinc based filters.
 * @param AD4170_MODE_STANDBY Part enters Standby Mode.
 * @param AD4170_MODE_POWER_DOWN All blocks are disabled, including LDO
 * regulators and serial interface.
 * @param AD4170_MODE_IDLE ADC enters idle mode, part remains powered on.
 * @param AD4170_MODE_SYS_OFFSET_CAL System Offset Calibration Mode.
 * @param AD4170_MODE_SYS_GAIN_CAL System Gain Calibration Mode.
 * @param AD4170_MODE_SELF_OFFSET_CAL Self Offset Calibration Mode.
 * @param AD4170_MODE_SELF_GAIN_CAL Self Gain Calibration Mode.
 ******************************************************************************/
enum ad4170_mode {
	/** Continuous Conversion Mode. ADC converts continuously on the enabled channel(s) using Sinc-based filters. */
	AD4170_MODE_CONT,
	/** Continuous Conversion Mode with FIR filter. ADC converts continuously on one channel using the FIR Filter. */
	AD4170_MODE_CONT_FIR,
	/** Continuous Conversion Mode with IIR filter. ADC converts continuously on one channel using the IIR Filter. */
	AD4170_MODE_CONT_IIR,
	/** Single conversion mode. ADC performs a single conversion (possibly repeated) on each enabled channel(s) using Sinc based filters. */
	AD4170_MODE_SINGLE = 0x4,
	/** Standby Mode. Part enters Standby Mode. */
	AD4170_MODE_STANDBY,
	/** Power-Down Mode. All blocks are disabled in Power-Down Mode, including the LDO regulators and the serial interface. */
	AD4170_MODE_POWER_DOWN,
	/** Idle Mode. ADC enters idle mode, part remains powered on. */
	AD4170_MODE_IDLE,
	/** System Offset Calibration Mode.  */
	AD4170_MODE_SYS_OFFSET_CAL,
	/** System Gain Calibration Mode. */
	AD4170_MODE_SYS_GAIN_CAL,
	/** Self Offset Calibration Mode. */
	AD4170_MODE_SELF_OFFSET_CAL,
	/** Self Gain Calibration Mode. */
	AD4170_MODE_SELF_GAIN_CAL
};

/***************************************************************************//**
 * @brief The `ad4170_adc_ctrl` structure is used to configure the control
 * settings of the AD4170 ADC. It includes options for enabling parallel
 * filtering across multiple channels, selecting between single or
 * multiple data registers, and enabling status output during continuous
 * read or transmit operations. Additionally, it allows for the
 * configuration of continuous data register read or transmit operations
 * and sets the ADC's operating mode, which can include various
 * conversion and calibration modes.
 *
 * @param parallel_filt_en Enables multiple channels to be converted in
 * parallel.
 * @param multi_data_reg_sel Selects between one or multiple data registers.
 * @param cont_read_status_en Enables status output in continuous read/transmit.
 * @param cont_read Continuous data register read/transmit enable.
 * @param mode ADC operating mode.
 ******************************************************************************/
struct ad4170_adc_ctrl {
	/** Enables Multiple Channels to Be Converted in Parallel. */
	bool parallel_filt_en;
	/** Selects Between One or Multiple Data Registers. */
	bool multi_data_reg_sel;
	/** Enables Status Output in Continuous Read/Transmit. */
	bool cont_read_status_en;
	/** Continuous Data Register Read/Transmit Enable. */
	enum ad4170_cont_read cont_read;
	/** ADC Operating Mode. */
	enum ad4170_mode mode;
};

/***************************************************************************//**
 * @brief The `ad4170_delay_n` enumeration defines a set of constants
 * representing different delay intervals, measured in multiples of the
 * Mod_Clk cycle, to be applied after a channel switch in the AD4170
 * device. These delay values are used to configure the timing behavior
 * of the device, allowing for precise control over the delay period
 * between channel switches.
 *
 * @param AD4170_DLY_0 Represents a delay of 0 Mod_Clk cycles.
 * @param AD4170_DLY_16 Represents a delay of 16 Mod_Clk cycles.
 * @param AD4170_DLY_256 Represents a delay of 256 Mod_Clk cycles.
 * @param AD4170_DLY_1024 Represents a delay of 1024 Mod_Clk cycles.
 * @param AD4170_DLY_2048 Represents a delay of 2048 Mod_Clk cycles.
 * @param AD4170_DLY_4096 Represents a delay of 4096 Mod_Clk cycles.
 * @param AD4170_DLY_8192 Represents a delay of 8192 Mod_Clk cycles.
 * @param AD4170_DLY_16384 Represents a delay of 16384 Mod_Clk cycles.
 ******************************************************************************/
enum ad4170_delay_n {
	/** 0 Delay. */
	AD4170_DLY_0,
	/** Delay 16 * Mod_Clk. */
	AD4170_DLY_16,
	/** Delay 256 * Mod_Clk. */
	AD4170_DLY_256,
	/** Delay 1024 * Mod_Clk. */
	AD4170_DLY_1024,
	/** Delay 2048 * Mod_Clk. */
	AD4170_DLY_2048,
	/** Delay 4096 * Mod_Clk. */
	AD4170_DLY_4096,
	/** Delay 8192 * Mod_Clk. */
	AD4170_DLY_8192,
	/** Delay 16384 * Mod_Clk. */
	AD4170_DLY_16384
};

/***************************************************************************//**
 * @brief The `ad4170_ain` enumeration defines a set of constants representing
 * various analog input channels and special inputs for the AD4170
 * device. These constants are used to specify the positive and negative
 * inputs for the multiplexer channels, including standard analog inputs,
 * temperature sensor inputs, power supply inputs, and reference inputs.
 * The enumeration provides a convenient way to refer to these inputs by
 * name in the code, facilitating the configuration and control of the
 * AD4170 device's input channels.
 *
 * @param AD4170_AIN0 Represents the analog input channel 0.
 * @param AD4170_AIN1 Represents the analog input channel 1.
 * @param AD4170_AIN2 Represents the analog input channel 2.
 * @param AD4170_AIN3 Represents the analog input channel 3.
 * @param AD4170_AIN4 Represents the analog input channel 4.
 * @param AD4170_AIN5 Represents the analog input channel 5.
 * @param AD4170_AIN6 Represents the analog input channel 6.
 * @param AD4170_AIN7 Represents the analog input channel 7.
 * @param AD4170_AIN8 Represents the analog input channel 8.
 * @param AD4170_AIN9 Represents the analog input channel 9.
 * @param AD4170_AIN10 Represents the analog input channel 10.
 * @param AD4170_AIN11 Represents the analog input channel 11.
 * @param AD4170_AIN12 Represents the analog input channel 12.
 * @param AD4170_AIN13 Represents the analog input channel 13.
 * @param AD4170_AIN14 Represents the analog input channel 14.
 * @param AD4170_AIN15 Represents the analog input channel 15.
 * @param AD4170_AIN16 Represents the analog input channel 16.
 * @param AD4170_TEMP_SENSOR_P Represents the positive input for the temperature
 * sensor, with a value of 17.
 * @param AD4170_TEMP_SENSOR_N Represents the negative input for the temperature
 * sensor, with a value of 17.
 * @param AD4170_AVDD_AVSS_P Represents the positive input for AVDD to AVSS,
 * with a value of 18.
 * @param AD4170_AVDD_AVSS_N Represents the negative input for AVDD to AVSS,
 * with a value of 18.
 * @param AD4170_IOVDD_DGND_P Represents the positive input for IOVDD to DGND,
 * with a value of 19.
 * @param AD4170_IOVDD_DGND_N Represents the negative input for IOVDD to DGND,
 * with a value of 19.
 * @param AD4170_DAC Represents the DAC input.
 * @param AD4170_ALDO Represents the ALDO input.
 * @param AD4170_DLDO Represents the DLDO input.
 * @param AD4170_AVSS Represents the AVSS input.
 * @param AD4170_DGND Represents the DGND input.
 * @param AD4170_REFIN1_P Represents the positive input for REFIN1.
 * @param AD4170_REFIN1_N Represents the negative input for REFIN1.
 * @param AD4170_REFIN2_P Represents the positive input for REFIN2.
 * @param AD4170_REFIN2_N Represents the negative input for REFIN2.
 * @param AD4170_REFOUT Represents the REFOUT input.
 * @param AD4170_OPEN Represents an open input, with a value of 31.
 ******************************************************************************/
enum ad4170_ain {
	AD4170_AIN0,
	AD4170_AIN1,
	AD4170_AIN2,
	AD4170_AIN3,
	AD4170_AIN4,
	AD4170_AIN5,
	AD4170_AIN6,
	AD4170_AIN7,
	AD4170_AIN8,
	AD4170_AIN9,
	AD4170_AIN10,
	AD4170_AIN11,
	AD4170_AIN12,
	AD4170_AIN13,
	AD4170_AIN14,
	AD4170_AIN15,
	AD4170_AIN16,
	AD4170_TEMP_SENSOR_P = 17,
	AD4170_TEMP_SENSOR_N = 17,
	AD4170_AVDD_AVSS_P = 18,
	AD4170_AVDD_AVSS_N = 18,
	AD4170_IOVDD_DGND_P = 19,
	AD4170_IOVDD_DGND_N = 19,
	AD4170_DAC,
	AD4170_ALDO,
	AD4170_DLDO,
	AD4170_AVSS,
	AD4170_DGND,
	AD4170_REFIN1_P,
	AD4170_REFIN1_N,
	AD4170_REFIN2_P,
	AD4170_REFIN2_N,
	AD4170_REFOUT,
	AD4170_OPEN = 31
};

/***************************************************************************//**
 * @brief The `ad4170_channel_setup` structure is used to define the
 * configuration for a specific channel in the AD4170 device. It includes
 * parameters for repeating the channel, adding a delay after switching
 * channels, and selecting a setup configuration that encompasses both
 * the Analog Front End (AFE) and the Digital Filter settings. This
 * structure is essential for managing how each channel operates within
 * the device, allowing for precise control over channel behavior and
 * timing.
 *
 * @param repeat_n Specifies the number of times to repeat this channel.
 * @param delay_n Specifies the delay to add after channel switch, using the
 * ad4170_delay_n enumeration.
 * @param setup_n Specifies the setup to use for this channel, which includes
 * the configuration for the AFE and the Digital Filter.
 ******************************************************************************/
struct ad4170_channel_setup {
	/** Number of Times to Repeat This Channel. */
	uint8_t repeat_n;
	/** Delay to Add After Channel Switch. */
	enum ad4170_delay_n delay_n;
	/** Setup to Use for This Channel. A "Setup" includes the configuration for the AFE and the Digital Filter. */
	uint8_t setup_n;
};

/***************************************************************************//**
 * @brief The `ad4170_channel_map` structure is used to define the channel
 * mapping for the AD4170 device, specifically selecting the analog
 * inputs for a sequencer channel. It consists of two members, `ainp` and
 * `ainm`, which represent the positive and negative inputs of the
 * multiplexer, respectively. These inputs are defined using the
 * `ad4170_ain` enumeration, allowing for a flexible configuration of the
 * analog input channels.
 *
 * @param ainp Multiplexer Positive Input.
 * @param ainm Multiplexer Negative Input.
 ******************************************************************************/
struct ad4170_channel_map {
	/** Multiplexer Positive Input. */
	enum ad4170_ain ainp;
	/** Multiplexer Negative Input. */
	enum ad4170_ain ainm;
};

/***************************************************************************//**
 * @brief The `ad4170_chop_iexc` enumeration defines the control settings for
 * chopping excitation currents in the AD4170 device. It provides options
 * to either disable chopping or enable it for specific pairs of
 * excitation currents, namely Iout_A and Iout_B, Iout_C and Iout_D, or
 * both pairs. This configuration is crucial for applications requiring
 * precise control over excitation currents to minimize errors and
 * improve measurement accuracy.
 *
 * @param AD4170_CHOP_IEXC_OFF No Chopping of Excitation Currents.
 * @param AD4170_CHOP_IEXC_AB Chopping of Iout_A and Iout_B Excitation Currents.
 * @param AD4170_CHOP_IEXC_CD Chopping of Iout_C and Iout_D Excitation Currents.
 * @param AD4170_CHOP_IEXC_ABCD Chopping of Both Pairs of Excitation Currents.
 ******************************************************************************/
enum ad4170_chop_iexc {
	/** No Chopping of Excitation Currents. */
	AD4170_CHOP_IEXC_OFF,
	/** Chopping of Iout_A and Iout_B Excitation Currents. */
	AD4170_CHOP_IEXC_AB,
	/** Chopping of Iout_C and Iout_D Excitation Currents. */
	AD4170_CHOP_IEXC_CD,
	/** Chopping of Both Pairs of Excitation Currents. */
	AD4170_CHOP_IEXC_ABCD
};

/***************************************************************************//**
 * @brief The `ad4170_chop_adc` is an enumeration that defines the different
 * chopping modes available for the AD4170 ADC/Mux. Chopping is a
 * technique used to reduce offset and drift errors in analog-to-digital
 * converters. This enumeration provides four options: no chopping,
 * chopping of the internal multiplexer, and two modes of AC excitation
 * chopping using either 4 or 2 GPIO pins. These modes allow for
 * flexibility in configuring the ADC to optimize performance based on
 * the specific application requirements.
 *
 * @param AD4170_CHOP_OFF No Chopping.
 * @param AD4170_CHOP_MUX Chops Internal Mux.
 * @param AD4170_CHOP_ACX_4PIN Chops AC Excitation Using 4 GPIO Pins.
 * @param AD4170_CHOP_ACX_2PIN Chops AC Excitation Using 2 GPIO Pins.
 ******************************************************************************/
enum ad4170_chop_adc {
	/** No Chopping. */
	AD4170_CHOP_OFF,
	/** Chops Internal Mux. */
	AD4170_CHOP_MUX,
	/** Chops AC Excitation Using 4 GPIO Pins. */
	AD4170_CHOP_ACX_4PIN,
	/** Chops AC Excitation Using 2 GPIO Pins. */
	AD4170_CHOP_ACX_2PIN
};

/***************************************************************************//**
 * @brief The `ad4170_burnout` enumeration defines the possible burnout current
 * settings for the AD4170 device, which are used to detect open-circuit
 * conditions by applying a small current to the input. The settings
 * include turning the burnout current off, or setting it to 100
 * nanoamperes, 2 microamperes, or 100 microamperes, allowing for
 * flexibility in detecting different levels of circuit integrity.
 *
 * @param AD4170_BURNOUT_OFF Represents the burnout current setting being turned
 * off.
 * @param AD4170_BURNOUT_100N Represents a burnout current setting of 100
 * nanoamperes.
 * @param AD4170_BURNOUT_2U Represents a burnout current setting of 2
 * microamperes.
 * @param AD4170_BURNOUT_10U Represents a burnout current setting of 100
 * microamperes.
 ******************************************************************************/
enum ad4170_burnout {
	/** Off */
	AD4170_BURNOUT_OFF,
	/** 100nA */
	AD4170_BURNOUT_100N,
	/** 2uA */
	AD4170_BURNOUT_2U,
	/** 100uA */
	AD4170_BURNOUT_10U
};

/***************************************************************************//**
 * @brief The `ad4170_misc` structure is used to configure miscellaneous
 * settings for the AD4170 device, specifically related to chopping and
 * burnout current control. It includes fields to manage the excitation
 * current chopping, ADC/Mux chopping, and burnout current values, which
 * are essential for fine-tuning the device's performance in various
 * applications.
 *
 * @param chop_iexc Controls the chopping of excitation currents.
 * @param chop_adc Controls the ADC/Mux chopping.
 * @param burnout Specifies the burnout current values.
 ******************************************************************************/
struct ad4170_misc {
	/** Excitation Current Chopping Control. */
	enum ad4170_chop_iexc chop_iexc;
	/** ADC/Mux Chopping Control. */
	enum ad4170_chop_adc chop_adc;
	/** Burnout Current Values. */
	enum ad4170_burnout burnout;
};

/***************************************************************************//**
 * @brief The `ad4170_ref_buf` enumeration defines the possible states for the
 * reference buffer configuration in the AD4170 device. It includes
 * options for pre-charging the buffer, fully buffering, or bypassing the
 * buffer entirely, allowing for flexible configuration of the reference
 * input buffer based on application requirements.
 *
 * @param AD4170_REF_BUF_PRE Represents the pre-charge buffer state.
 * @param AD4170_REF_BUF_FULL Represents the full buffer state.
 * @param AD4170_REF_BUF_BYPASS Represents the bypass state.
 ******************************************************************************/
enum ad4170_ref_buf {
	/** Pre-charge Buffer. */
	AD4170_REF_BUF_PRE,
	/** Full Buffer.*/
	AD4170_REF_BUF_FULL,
	/** Bypass */
	AD4170_REF_BUF_BYPASS
};

/***************************************************************************//**
 * @brief The `ad4170_ref_select` enumeration defines the possible reference
 * voltage selections for the AD4170 device. Each enumerator corresponds
 * to a specific pair of reference inputs or outputs that can be used by
 * the ADC, allowing for flexibility in configuring the reference voltage
 * source for the device's operation.
 *
 * @param AD4170_REFIN_REFIN1 Represents the reference selection of Refin1+ and
 * Refin1-.
 * @param AD4170_REFIN_REFIN2 Represents the reference selection of Refin2+ and
 * Refin2-.
 * @param AD4170_REFIN_REFOUT Represents the reference selection of Refout and
 * AVSS, with the 2.5V Refout needing separate
 * enablement in the ref_control register.
 * @param AD4170_REFIN_AVDD Represents the reference selection of AVDD and AVSS.
 ******************************************************************************/
enum ad4170_ref_select {
	/** Refin1+, Refin1-. */
	AD4170_REFIN_REFIN1,
	/** Refin2+, Refin2-. */
	AD4170_REFIN_REFIN2,
	/** Refout, AVSS. The 2.5V Refout must be enabled separately in the ref_control register. */
	AD4170_REFIN_REFOUT,
	/** AVDD, AVSS. */
	AD4170_REFIN_AVDD
};

/***************************************************************************//**
 * @brief The `ad4170_pga_gain` enumeration defines various programmable gain
 * amplifier (PGA) gain settings for the AD4170 device. These settings
 * allow the user to select different gain levels, ranging from 0.5 to
 * 128, to amplify input signals. Additionally, a special gain setting
 * with a pre-charge buffer is available, which can affect input
 * currents. This enumeration is used to configure the gain of the PGA in
 * the AD4170 device, which is crucial for adapting the device to
 * different signal levels and applications.
 *
 * @param AD4170_PGA_GAIN_1 PGA Gain = 1.
 * @param AD4170_PGA_GAIN_2 PGA Gain = 2.
 * @param AD4170_PGA_GAIN_4 PGA Gain = 4.
 * @param AD4170_PGA_GAIN_8 PGA Gain = 8.
 * @param AD4170_PGA_GAIN_16 PGA Gain = 16.
 * @param AD4170_PGA_GAIN_32 PGA Gain = 32.
 * @param AD4170_PGA_GAIN_64 PGA Gain = 64.
 * @param AD4170_PGA_GAIN_128 PGA Gain = 128.
 * @param AD4170_PGA_GAIN_0P5 PGA Gain = 0.5.
 * @param AD4170_PGA_GAIN_1_PRECHARGE PGA Gain = 1 with Pre-charge Buffer, which
 * may increase input currents.
 ******************************************************************************/
enum ad4170_pga_gain {
	/** PGA Gain = 1 */
	AD4170_PGA_GAIN_1,
	/** PGA Gain = 2 */
	AD4170_PGA_GAIN_2,
	/** PGA Gain = 4 */
	AD4170_PGA_GAIN_4,
	/** PGA Gain = 8 */
	AD4170_PGA_GAIN_8,
	/** PGA Gain = 16 */
	AD4170_PGA_GAIN_16,
	/** PGA Gain = 32 */
	AD4170_PGA_GAIN_32,
	/** PGA Gain = 64 */
	AD4170_PGA_GAIN_64,
	/** PGA Gain = 128 */
	AD4170_PGA_GAIN_128,
	/** PGA Gain = 0.5 */
	AD4170_PGA_GAIN_0P5,
	/** PGA Gain = 1 Pre-charge Buffer. Input currents may increase when the pre-charge-buffer is used. */
	AD4170_PGA_GAIN_1_PRECHARGE
};

/***************************************************************************//**
 * @brief The `ad4170_afe` structure is used to configure the Analog Front End
 * (AFE) settings for the AD4170 device. It includes settings for
 * enabling the REFIN buffers, selecting the ADC reference, choosing
 * between bipolar or unipolar ADC operation, and setting the gain for
 * the Programmable Gain Amplifier (PGA). This structure is crucial for
 * defining how the ADC interacts with its input signals and reference
 * voltages, impacting the accuracy and range of the analog-to-digital
 * conversion process.
 *
 * @param ref_buf_m Enables the negative REFIN buffer.
 * @param ref_buf_p Enables the positive REFIN buffer.
 * @param ref_select Selects the ADC reference source.
 * @param bipolar Determines if the ADC operates in bipolar or unipolar mode.
 * @param pga_gain Sets the gain for the Programmable Gain Amplifier (PGA).
 ******************************************************************************/
struct ad4170_afe {
	/** REFIN Buffer- Enable. */
	enum ad4170_ref_buf ref_buf_m;
	/** REFIN Buffer+ Enable. */
	enum ad4170_ref_buf ref_buf_p;
	/** ADC Reference Selection. */
	enum ad4170_ref_select ref_select;
	/** Select Bipolar or Unipolar ADC Span. */
	bool bipolar;
	/** PGA Gain Selection. */
	enum ad4170_pga_gain pga_gain;
};

/***************************************************************************//**
 * @brief The `ad4170_post_filter` is an enumeration that defines various post-
 * filter configurations for the AD4170 device. These configurations are
 * used to apply specific filtering techniques to the signal, such as
 * rejecting 50/60Hz noise or averaging, with different settling times.
 * Each enumerator represents a distinct filter setting that can be
 * applied to the device to optimize signal processing for different
 * applications.
 *
 * @param AD4170_POST_FILTER_NONE No Post Filter.
 * @param AD4170_POST_FILTER_40MS Post Filter for 50/60Hz Rejection with 40ms
 * Settling.
 * @param AD4170_POST_FILTER_50MS Post Filter for 50/60Hz Rejection with 50ms
 * Settling.
 * @param AD4170_POST_FILTER_60MS Post Filter for 50/60Hz Rejection with 60ms
 * Settling.
 * @param AD4170_POST_FILTER_FAST_AC Post Filter for AC Excitation with 5ms
 * Settling.
 * @param AD4170_POST_FILTER_AVG16 Post Filter for Average-By-16.
 * @param AD4170_POST_FILTER_AVG20 Post Filter for 60Hz Rejection with 16.7ms
 * Settling.
 * @param AD4170_POST_FILTER_AVG24 Post Filter for 50Hz Rejection with 20ms
 * Settling.
 ******************************************************************************/
enum ad4170_post_filter {
	/** No Post Filter. */
	AD4170_POST_FILTER_NONE,
	/** Post Filter for 50/60Hz Rejection with 40ms Settling. */
	AD4170_POST_FILTER_40MS,
	/** Post Filter for 50/60Hz Rejection with 50ms Settling. */
	AD4170_POST_FILTER_50MS,
	/** Post Filter for 50/60Hz Rejection with 60ms Settling. */
	AD4170_POST_FILTER_60MS,
	/** Post Filter for AC Excitation with 5ms Settling. */
	AD4170_POST_FILTER_FAST_AC,
	/** Post Filter for Average-By-16. */
	AD4170_POST_FILTER_AVG16,
	/** Post Filter for 60Hz Rejection with 16.7ms Settling. */
	AD4170_POST_FILTER_AVG20,
	/** Post Filter for 50Hz Rejection with 20ms Settling. */
	AD4170_POST_FILTER_AVG24
};

/***************************************************************************//**
 * @brief The `ad4170_filter_type` is an enumeration that defines the different
 * filter modes available for the AD4170 device, specifically focusing on
 * Sinc-based filters. It provides three distinct filter options: Sinc5
 * Plus Average, Sinc5, and Sinc3, each represented by a unique
 * hexadecimal value. This enumeration is used to configure the filtering
 * behavior of the AD4170, allowing users to select the appropriate
 * filter type for their specific application needs.
 *
 * @param AD4170_FILT_SINC5_AVG Represents the Sinc5 Plus Average filter mode
 * with a value of 0x0.
 * @param AD4170_FILT_SINC5 Represents the Sinc5 filter mode with a value of
 * 0x4.
 * @param AD4170_FILT_SINC3 Represents the Sinc3 filter mode with a value of
 * 0x6.
 ******************************************************************************/
enum ad4170_filter_type {
	/** Sinc5 Plus Average. */
	AD4170_FILT_SINC5_AVG = 0x0,
	/** Sinc5 */
	AD4170_FILT_SINC5 = 0x4,
	/** Sinc3 */
	AD4170_FILT_SINC3 = 0x6
};

/***************************************************************************//**
 * @brief The `ad4170_filter` structure is used to configure the filtering
 * options for the AD4170 device. It includes settings for an optional
 * post-filter, which can be used to further refine the signal
 * processing, and a filter type selection for Sinc-based filters,
 * allowing for customization of the filtering process to suit specific
 * application needs.
 *
 * @param post_filter_sel Optional Post-Filter configuration.
 * @param filter_type Filter Mode for Sinc-Based Filters.
 ******************************************************************************/
struct ad4170_filter {
	/** Optional Post-Filter configuration. */
	enum ad4170_post_filter post_filter_sel;
	/** Filter Mode for Sinc-Based Filters. */
	enum ad4170_filter_type filter_type;
};

/***************************************************************************//**
 * @brief The `ad4170_setup` structure is used to configure various settings for
 * the AD4170 device, which is a precision analog-to-digital converter.
 * It includes configurations for miscellaneous settings, the analog
 * front end (AFE), and digital filters. The structure also allows for
 * the adjustment of digital offset and gain, providing flexibility in
 * tuning the device's performance to meet specific application
 * requirements.
 *
 * @param misc Holds miscellaneous register settings for the AD4170 device.
 * @param afe Configures the Analog Front End, including PGA, reference, and
 * buffers.
 * @param filter Selects the digital filter type for the AD4170 device.
 * @param filter_fs Specifies the filter select word for digital filters.
 * @param offset Defines the digital offset adjustment value.
 * @param gain Defines the digital gain adjustment value.
 ******************************************************************************/
struct ad4170_setup {
	struct ad4170_misc misc;
	/** Configures Analog Front End - PGA, Reference, Buffers. */
	struct ad4170_afe afe;
	/** Selects Digital Filter Type. */
	struct ad4170_filter filter;
	/** Filter select word for Digital Filters. */
	uint16_t filter_fs;
	/** Digital Offset Adjustment Value. */
	uint32_t offset;
	/** Digital Gain Adjustment Value. */
	uint32_t gain;
};

/***************************************************************************//**
 * @brief The `ad4170_ref_control` structure is a simple data structure used to
 * manage the internal reference settings of the AD4170 device. It
 * contains a single boolean member, `ref_en`, which is used to enable or
 * disable the internal reference. This structure is part of the
 * configuration settings for the AD4170, a precision analog-to-digital
 * converter, and plays a role in controlling the reference voltage
 * source used by the device.
 *
 * @param ref_en A boolean flag indicating whether the internal reference is
 * enabled.
 ******************************************************************************/
struct ad4170_ref_control {
	/** Internal Reference Enable. */
	bool ref_en;
};

/***************************************************************************//**
 * @brief The `ad4170_i_out_pin` enumeration defines the possible destinations
 * for the current source output (I_OUT) in the AD4170 device. It
 * specifies various analog input pins (AIN0 to AIN15, AINCOM) and
 * general-purpose input/output pins (GPIO0 to GPIO3) where the current
 * source can be directed. This enumeration is used to configure the
 * hardware to route the current source to the desired pin, facilitating
 * flexible hardware configurations and applications.
 *
 * @param AD4170_I_OUT_AIN0 I_OUT is available on AIN0.
 * @param AD4170_I_OUT_AIN1 I_OUT is available on AIN1.
 * @param AD4170_I_OUT_AIN2 I_OUT is available on AIN2.
 * @param AD4170_I_OUT_AIN3 I_OUT is available on AIN3.
 * @param AD4170_I_OUT_AIN4 I_OUT is available on AIN4.
 * @param AD4170_I_OUT_AIN5 I_OUT is available on AIN5.
 * @param AD4170_I_OUT_AIN6 I_OUT is available on AIN6.
 * @param AD4170_I_OUT_AIN7 I_OUT is available on AIN7.
 * @param AD4170_I_OUT_AIN8 I_OUT is available on AIN8.
 * @param AD4170_I_OUT_AIN9 I_OUT is available on AIN9.
 * @param AD4170_I_OUT_AIN10 I_OUT is available on AIN10.
 * @param AD4170_I_OUT_AIN11 I_OUT is available on AIN11.
 * @param AD4170_I_OUT_AIN12 I_OUT is available on AIN12.
 * @param AD4170_I_OUT_AIN13 I_OUT is available on AIN13.
 * @param AD4170_I_OUT_AIN14 I_OUT is available on AIN14.
 * @param AD4170_I_OUT_AIN15 I_OUT is available on AIN15.
 * @param AD4170_I_OUT_AINCOM I_OUT is available on AINCOM.
 * @param AD4170_I_OUT_GPIO0 I_OUT is available on GPIO0.
 * @param AD4170_I_OUT_GPIO1 I_OUT is available on GPIO1.
 * @param AD4170_I_OUT_GPIO2 I_OUT is available on GPIO2.
 * @param AD4170_I_OUT_GPIO3 I_OUT is available on GPIO3.
 ******************************************************************************/
enum ad4170_i_out_pin {
	/** I_OUT is Available on AIN0. */
	AD4170_I_OUT_AIN0,
	/** I_OUT is Available on AIN1. */
	AD4170_I_OUT_AIN1,
	/** I_OUT is Available on AIN2. */
	AD4170_I_OUT_AIN2,
	/** I_OUT is Available on AIN3. */
	AD4170_I_OUT_AIN3,
	/** I_OUT is Available on AIN4. */
	AD4170_I_OUT_AIN4,
	/** I_OUT is Available on AIN5. */
	AD4170_I_OUT_AIN5,
	/** I_OUT is Available on AIN6. */
	AD4170_I_OUT_AIN6,
	/** I_OUT is Available on AIN7. */
	AD4170_I_OUT_AIN7,
	/** I_OUT is Available on AIN8. */
	AD4170_I_OUT_AIN8,
	/** I_OUT is Available on AIN9. */
	AD4170_I_OUT_AIN9,
	/** I_OUT is Available on AIN10. */
	AD4170_I_OUT_AIN10,
	/** I_OUT is Available on AIN11. */
	AD4170_I_OUT_AIN11,
	/** I_OUT is Available on AIN12. */
	AD4170_I_OUT_AIN12,
	/** I_OUT is Available on AIN13. */
	AD4170_I_OUT_AIN13,
	/** I_OUT is Available on AIN14. */
	AD4170_I_OUT_AIN14,
	/** I_OUT is Available on AIN15. */
	AD4170_I_OUT_AIN15,
	/** I_OUT is Available on AINCOM. */
	AD4170_I_OUT_AINCOM,
	/** I_OUT is Available on GPIO0. */
	AD4170_I_OUT_GPIO0,
	/** I_OUT is Available on GPIO1. */
	AD4170_I_OUT_GPIO1,
	/** I_OUT is Available on GPIO2. */
	AD4170_I_OUT_GPIO2,
	/** I_OUT is Available on GPIO3. */
	AD4170_I_OUT_GPIO3
};

/***************************************************************************//**
 * @brief The `ad4170_i_out_val` enumeration defines a set of constants
 * representing different current output values in microamperes for the
 * AD4170 device. These values are used to configure the current source
 * output of the device, allowing for precise control over the current
 * output level.
 *
 * @param AD4170_I_OUT_0UA Represents a current output value of 0 microamperes.
 * @param AD4170_I_OUT_10UA Represents a current output value of 10
 * microamperes.
 * @param AD4170_I_OUT_50UA Represents a current output value of 50
 * microamperes.
 * @param AD4170_I_OUT_100UA Represents a current output value of 100
 * microamperes.
 * @param AD4170_I_OUT_250UA Represents a current output value of 250
 * microamperes.
 * @param AD4170_I_OUT_500UA Represents a current output value of 500
 * microamperes.
 * @param AD4170_I_OUT_1000UA Represents a current output value of 1000
 * microamperes.
 * @param AD4170_I_OUT_1500UA Represents a current output value of 1500
 * microamperes.
 ******************************************************************************/
enum ad4170_i_out_val {
	/** 0uA */
	AD4170_I_OUT_0UA,
	/** 10uA */
	AD4170_I_OUT_10UA,
	/** 50uA */
	AD4170_I_OUT_50UA,
	/** 100uA */
	AD4170_I_OUT_100UA,
	/** 250uA */
	AD4170_I_OUT_250UA,
	/** 500uA */
	AD4170_I_OUT_500UA,
	/** 1000uA */
	AD4170_I_OUT_1000UA,
	/** 1500uA */
	AD4170_I_OUT_1500UA
};

/***************************************************************************//**
 * @brief The `ad4170_current_source` structure is used to configure the current
 * source settings for the AD4170 device. It includes two members:
 * `i_out_pin`, which determines the destination pin for the current
 * output, and `i_out_val`, which specifies the magnitude of the current
 * to be output. This structure is essential for managing the current
 * source functionality of the AD4170, allowing for precise control over
 * current output in various applications.
 *
 * @param i_out_pin Specifies the pin where the current source output is
 * directed.
 * @param i_out_val Defines the value of the current output from the source.
 ******************************************************************************/
struct ad4170_current_source {
	enum ad4170_i_out_pin i_out_pin;
	enum ad4170_i_out_val i_out_val;
};

/***************************************************************************//**
 * @brief The `ad4170_fir_mode` is an enumeration that defines the different
 * types of Finite Impulse Response (FIR) filters available for
 * configuration in the AD4170 device. It includes options for default
 * FIR filtering, as well as programmable filters with various symmetric
 * and asymmetric coefficient configurations. This allows for flexible
 * digital signal processing tailored to specific application needs.
 *
 * @param AD4170_FIR_DEFAULT Selects the default FIR filter and ignores any
 * programmed FIR_Length and FIR Coefficient values.
 * @param AD4170_FIR_SYM_ODD FIR Programmable with Odd Symmetric Coefficients.
 * @param AD4170_FIR_SYM_EVEN FIR Programmable with Even Symmetric Coefficients.
 * @param AD4170_FIR_ANTISYM_ODD FIR Programmable with Odd Anti-Symmetric
 * Coefficients.
 * @param AD4170_FIR_ANTISYM_EVEN FIR Programmable with Even Anti-Symmetric
 * Coefficients.
 * @param AD4170_FIR_ASYM FIR Programmable with Asymmetric Coefficients.
 ******************************************************************************/
enum ad4170_fir_mode {
	/** FIR Default. Selects the default FIR filter and ignores any programmed FIR_Length and FIR Coefficient values. */
	AD4170_FIR_DEFAULT,
	/** FIR Programmable with Odd Symmetric Coefficients. */
	AD4170_FIR_SYM_ODD,
	/** FIR Programmable with Even Symmetric Coefficients. */
	AD4170_FIR_SYM_EVEN,
	/** FIR Programmable with Odd Anti-Symmetric Coefficients. */
	AD4170_FIR_ANTISYM_ODD,
	/** FIR Programmable with Even Anti-Symmetric Coefficients. */
	AD4170_FIR_ANTISYM_EVEN,
	/** FIR Programmable with Asymmetric Coefficients. */
	AD4170_FIR_ASYM
};

/***************************************************************************//**
 * @brief The `ad4170_fir_coeff_set` is an enumeration that defines two sets of
 * FIR (Finite Impulse Response) filter coefficients for the AD4170
 * device. Each set specifies a range of coefficient addresses that can
 * be used for configuring the FIR filter, allowing for different
 * filtering configurations based on the selected set. This enumeration
 * is used to select which set of coefficients to apply in the FIR filter
 * configuration.
 *
 * @param AD4170_FIR_COEFF_SET0 FIR set 0 uses coefficient addresses from 0 to
 * FIR_LENGTH-1.
 * @param AD4170_FIR_COEFF_SET1 FIR set 1 uses coefficient addresses from 72 to
 * 72 + FIR_LENGTH-1.
 ******************************************************************************/
enum ad4170_fir_coeff_set {
	/* FIR set 0. Use coefficient addresses 0 to FIR_LENGTH-1 */
	AD4170_FIR_COEFF_SET0,
	/* FIR set 1. Use coefficient addresses 72 to 72 + FIR_LENGTH-1 */
	AD4170_FIR_COEFF_SET1
};

/***************************************************************************//**
 * @brief The `ad4170_fir_control` structure is used to configure the FIR
 * (Finite Impulse Response) filter settings for the AD4170 device. It
 * allows the selection of the FIR filter type, the set of coefficients
 * to be used, and the number of coefficients. Additionally, it provides
 * a pointer to the array containing the FIR coefficients, enabling
 * flexible and programmable filter configurations tailored to specific
 * application needs.
 *
 * @param fir_mode Selects the type of FIR filter to be used.
 * @param coeff_set Determines which set of FIR coefficients to use.
 * @param fir_length Specifies the number of programmed FIR coefficients, with a
 * maximum of 72.
 * @param fir_coefficients Pointer to an array that holds the FIR coefficients.
 ******************************************************************************/
struct ad4170_fir_control {
	/** Selects FIR Type. */
	enum ad4170_fir_mode fir_mode;
	/** Selects Which Set of FIR Coefficients to Use. */
	enum ad4170_fir_coeff_set coeff_set;
	/** Number of Programmed Coefficients, Max: 72. */
	uint8_t fir_length;
	/** Pointer to array holding the FIR coefficients */
	int32_t *fir_coefficients;
};

/***************************************************************************//**
 * @brief The `ad4170_dac_gain` enumeration defines the possible output voltage
 * ranges for the DAC in the AD4170 device. It provides two options: a
 * gain of 1, where the output range is from 0V to the reference output
 * (REFOUT), and a gain of 2, where the output range is from 0V to twice
 * the reference output. This allows for flexibility in configuring the
 * DAC output span based on application requirements.
 *
 * @param AD4170_DAC_GAIN_1 DAC Output Range is 0V to REFOUT.
 * @param AD4170_DAC_GAIN_2 DAC Output Range is 0V to 2*REFOUT.
 ******************************************************************************/
enum ad4170_dac_gain {
	/** DAC Output Range is 0V to REFOUT. */
	AD4170_DAC_GAIN_1,
	/** DAC Output Range is 0V to 2*REFOUT. */
	AD4170_DAC_GAIN_2
};

/***************************************************************************//**
 * @brief The `ad4170_dac_config` structure is used to configure the settings
 * for DAC0 in the AD4170 device. It includes options to enable or
 * disable the DAC, set the gain, and configure hardware toggling
 * features for both the DAC and LDAC signals. This structure is
 * essential for managing the DAC's operational parameters and ensuring
 * it functions as intended within the broader AD4170 configuration.
 *
 * @param enabled Indicates whether DAC0 is enabled or disabled.
 * @param gain Specifies the gain setting for DAC0.
 * @param hw_toggle Determines if hardware toggle is enabled for DAC0.
 * @param hw_ldac Indicates if hardware LDAC toggle is enabled for DAC0.
 ******************************************************************************/
struct ad4170_dac_config {
	/** Selects DAC0 Enabled/Disabled. */
	bool enabled;
	/** Select DAC0 Gain. */
	enum ad4170_dac_gain gain;
	/** Select DAC0 HW Toggle. */
	bool hw_toggle;
	/** Select DAC0 LDAC Toggle. */
	bool hw_ldac;
};

/***************************************************************************//**
 * @brief The `ad4170_config` structure is a comprehensive configuration data
 * structure for the AD4170 device, encapsulating various register
 * settings required for its operation. It includes settings for pin
 * muxing, clock control, standby and power down modes, error handling,
 * ADC control, channel enablement, and channel setup and mapping.
 * Additionally, it manages reference control, voltage bias, current
 * pull-up, current source configurations, FIR filter control, and DAC
 * settings. This structure is essential for initializing and configuring
 * the AD4170 device to perform its intended functions in data
 * acquisition and processing applications.
 *
 * @param pin_muxing Holds the pin muxing register settings for the AD4170.
 * @param clock_ctrl Contains the clock control register settings for the
 * AD4170.
 * @param standby_ctrl 16-bit value for standby control register settings.
 * @param powerdown_sw 16-bit value for power down switch register settings.
 * @param error_en 16-bit value for error enable register settings.
 * @param adc_ctrl Holds the ADC control register settings for the AD4170.
 * @param channel_en 16-bit value for channel enable register settings.
 * @param setup Array of channel setup register settings for each channel.
 * @param map Array of channel map register settings for each channel.
 * @param setups Array of setup configurations for the AD4170.
 * @param ref_control Holds the reference control register settings for the
 * AD4170.
 * @param v_bias 16-bit value for voltage bias register settings.
 * @param i_pullup 16-bit value for current pull-up register settings.
 * @param current_source Array of current source register settings.
 * @param fir_control Holds the FIR control register settings for the AD4170.
 * @param dac Contains the DAC configuration settings including HW_LDAC_Mask,
 * HW_Toggle_Mask, Channel_En, and DAC_Span.
 ******************************************************************************/
struct ad4170_config {
	/** Pin_Muxing register settings. */
	struct ad4170_pin_muxing pin_muxing;
	/** Clock_Ctrl register settings. */
	struct ad4170_clock_ctrl clock_ctrl;
	/** Standby_Ctrl register settings. */
	uint16_t standby_ctrl;
	/** Power_Down_Sw register settings. */
	uint16_t powerdown_sw;
	/** Error_En register settings. */
	uint16_t error_en;
	/** ADC_Ctrl register settings. */
	struct ad4170_adc_ctrl adc_ctrl;
	/** Channel_En register settings. */
	uint16_t channel_en;
	/** Channel_Setup register settings. */
	struct ad4170_channel_setup setup[AD4170_NUM_CHANNELS];
	/** Channel_Map register settings. */
	struct ad4170_channel_map map[AD4170_NUM_CHANNELS];
	/** Setups settings. */
	struct ad4170_setup setups[AD4170_NUM_SETUPS];
	/** Ref_Control register settings. */
	struct ad4170_ref_control ref_control;
	/** V_Bias register settings. */
	uint16_t v_bias;
	/** I_Pullup register settings. */
	uint16_t i_pullup;
	/** Current_Source register settings */
	struct ad4170_current_source current_source[AD4170_NUM_CURRENT_SOURCE];
	/** FIR_Control register settings. */
	struct ad4170_fir_control fir_control;
	/** DAC settings (registers HW_LDAC_Mask, HW_Toggle_Mask, Channel_En and DAC_Span). */
	struct ad4170_dac_config dac;
};

/***************************************************************************//**
 * @brief The `ad4170_spi_settings` structure is used to configure the SPI
 * communication settings for the AD4170 device. It includes options for
 * selecting short instruction addressing, enabling CRC for communication
 * error checking, and detecting and recovering from sync pattern loss.
 * This structure is essential for ensuring reliable and efficient
 * communication with the AD4170 device over SPI.
 *
 * @param short_instruction Selects short instruction, 7-bit Addressing, instead
 * of the default 15-bit Addressing.
 * @param crc_enabled Enables communication CRC generation and checking.
 * @param sync_loss_detect Enables sync pattern loss detection and recovery.
 ******************************************************************************/
struct ad4170_spi_settings {
	/** Select short instruction, 7-bit Addressing, instead of the default 15-bit Addressing. */
	bool short_instruction;
	/** Enable communication CRC generation and checking. */
	bool crc_enabled;
	/** Enable sync pattern loss detection and recovery. */
	bool sync_loss_detect;
};

/***************************************************************************//**
 * @brief The `ad4170_id` enumeration defines a set of constants that represent
 * different device IDs for the AD4170 series of devices. Each enumerator
 * corresponds to a specific model within the series, allowing for easy
 * identification and selection of the device type in the software. This
 * enumeration is typically used in device initialization and
 * configuration processes to specify which model of the AD4170 series is
 * being interfaced with.
 *
 * @param ID_AD4170 Represents the device ID for AD4170.
 * @param ID_AD4171 Represents the device ID for AD4171.
 * @param ID_AD4172 Represents the device ID for AD4172.
 * @param ID_AD4190 Represents the device ID for AD4190.
 * @param ID_AD4195 Represents the device ID for AD4195.
 ******************************************************************************/
enum ad4170_id {
	ID_AD4170,
	ID_AD4171,
	ID_AD4172,
	ID_AD4190,
	ID_AD4195
};

/***************************************************************************//**
 * @brief The `ad4170_init_param` structure is used to initialize the AD4170
 * device, encapsulating all necessary parameters for device setup. It
 * includes the device ID, SPI configuration for both the host processor
 * and the AD4170, a timeout setting for monitoring the Ready GPIO, and
 * various GPIO configurations for synchronization and auxiliary digital
 * pins. This structure is essential for setting up the AD4170 device in
 * a system, ensuring proper communication and operation.
 *
 * @param id Specifies the active device using an enumeration of device IDs.
 * @param spi_init Holds the SPI configuration parameters for the host
 * processor.
 * @param spi_settings Contains the SPI communication settings specific to the
 * AD4170.
 * @param rdy_conv_timeout Defines the timeout loop counter for the Ready GPIO
 * (EOC) monitor.
 * @param config Holds the configuration settings for the AD4170 device.
 * @param gpio_sync_inb Pointer to the GPIO configuration for the SYNC_IN pin.
 * @param gpio_dig_aux1 Pointer to the GPIO configuration for the DIG_AUX1 pin.
 * @param gpio_dig_aux2 Pointer to the GPIO configuration for the DIG_AUX2 pin.
 ******************************************************************************/
struct ad4170_init_param {
	/** Active Device */
	enum ad4170_id id;
	/** Host processor SPI configuration. */
	struct no_os_spi_init_param spi_init;
	/** AD4170 SPI communication configuration. */
	struct ad4170_spi_settings spi_settings;
	/* Rdy GPIO (EOC) monitor timeout loop counter */
	uint32_t rdy_conv_timeout;
	/** AD4170 device configuration. */
	struct ad4170_config config;
	/** SYNC_IN GPIO configuration. */
	struct no_os_gpio_init_param *gpio_sync_inb;
	/** DIG_AUX1 GPIO configuration. */
	struct no_os_gpio_init_param *gpio_dig_aux1;
	/** DIG_AUX2 GPIO configuration. */
	struct no_os_gpio_init_param *gpio_dig_aux2;
};

/***************************************************************************//**
 * @brief The `ad4170_dev` structure is a comprehensive descriptor for the
 * AD4170 device, encapsulating all necessary configurations and states
 * required for its operation. It includes identifiers for the device and
 * SPI settings, GPIO configurations for synchronization and auxiliary
 * functions, and flags for endianess and interface initialization. This
 * structure is essential for managing the communication and control of
 * the AD4170 device within a host system, ensuring proper setup and
 * operation of the device's SPI and GPIO interfaces.
 *
 * @param id Device ID of type enum ad4170_id.
 * @param spi_init Copy of the Host processor SPI configuration.
 * @param spi_desc Pointer to the SPI descriptor.
 * @param spi_settings Copy of the AD4170 SPI communication configuration.
 * @param rdy_conv_timeout Rdy GPIO (EOC) monitor timeout loop counter.
 * @param big_endian Specifies whether CPU is big endian.
 * @param config Copy of the AD4170 device configuration.
 * @param gpio_sync_inb_init Copy of the SYNC_IN GPIO configuration.
 * @param gpio_sync_inb SYNC_IN GPIO descriptor.
 * @param gpio_dig_aux1_init Copy of the DIG_AUX1 GPIO configuration.
 * @param gpio_dig_aux1 DIG_AUX1 GPIO descriptor.
 * @param gpio_dig_aux2_init Copy of the DIG_AUX2 GPIO configuration.
 * @param dig_aux2_output Specifies whether DIG_AUX2 is configured as an output.
 * @param gpio_dig_aux2 DIG_AUX2 GPIO descriptor.
 * @param digif Specifies whether the SPI+GPIO interface is initialized.
 ******************************************************************************/
struct ad4170_dev {
	/** Device ID */
	enum ad4170_id id;
	/** Copy of the Host processor SPI configuration. */
	struct no_os_spi_init_param spi_init;
	/** SPI descriptor. */
	struct no_os_spi_desc *spi_desc;
	/** Copy of the AD4170 SPI communication configuration. */
	struct ad4170_spi_settings spi_settings;
	/* Rdy GPIO (EOC) monitor timeout loop counter */
	uint32_t rdy_conv_timeout;
	/** Specifies whether CPU is big endian. */
	bool big_endian;
	/** Copy of the AD4170 device configuration. */
	struct ad4170_config config;
	/** Copy of the SYNC_IN GPIO configuration. */
	struct no_os_gpio_init_param *gpio_sync_inb_init;
	/** SYNC_IN GPIO descriptor. */
	struct no_os_gpio_desc *gpio_sync_inb;
	/** Copy of the DIG_AUX1 GPIO configuration. */
	struct no_os_gpio_init_param *gpio_dig_aux1_init;
	/** DIG_AUX1 GPIO descriptor. */
	struct no_os_gpio_desc *gpio_dig_aux1;
	/** Copy of the DIG_AUX2 GPIO configuration. */
	struct no_os_gpio_init_param *gpio_dig_aux2_init;
	/** DIG_AUX2 may be configured as input or output, this specifies whether it is an output. */
	bool dig_aux2_output;
	/** DIG_AUX2 GPIO descriptor. */
	struct no_os_gpio_desc *gpio_dig_aux2;
	/** Specifies whether the SPI+GPIO is interface is initialized. */
	bool digif;
};

/***************************************************************************//**
 * @brief The `ad4170_config_reset` is a global variable of type `struct
 * ad4170_config` that represents the default configuration state of the
 * AD4170 device when it undergoes a hardware or software reset. This
 * structure contains various configuration settings for the AD4170,
 * including pin muxing, clock control, ADC control, channel setup, and
 * more.
 *
 * @details This variable is used to reset the AD4170 device to its default
 * configuration state.
 ******************************************************************************/
extern struct ad4170_config ad4170_config_reset;

/***************************************************************************//**
 * @brief This function reads the value of a specified register from an AD4170
 * device using SPI communication. It requires a valid device descriptor
 * and a register address to be provided. The function will store the
 * read data in the location pointed to by the reg_data parameter. It
 * must not be called when the device is in continuous read mode, as this
 * will result in an access error. The function handles invalid device
 * pointers, invalid register addresses, and null data pointers by
 * returning an error code.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the device.
 * @param reg_data A pointer to a uint32_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or -EACCES if the device is in
 * continuous read mode.
 ******************************************************************************/
int ad4170_spi_reg_read(struct ad4170_dev *dev,
			uint32_t reg_addr,
			uint32_t *reg_data);
/***************************************************************************//**
 * @brief This function is used to write a 32-bit data value to a specified
 * register address on the AD4170 device using SPI communication. It
 * should be called when the device is properly initialized and
 * configured. The function checks for valid device and register address
 * inputs, and it handles specific conditions when the device is in
 * continuous read or transmit modes, allowing only certain register
 * writes. It also supports optional CRC checking if enabled in the
 * device settings. The function returns an error code if the operation
 * fails due to invalid inputs, access restrictions, or communication
 * errors.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param reg_addr The 32-bit address of the register to write to. The address
 * must be valid for the device; otherwise, the function returns
 * -EINVAL.
 * @param reg_data The 32-bit data to be written to the specified register.
 * There are no specific constraints on the data value itself.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters, -EACCES for access violations, or
 * -EBADMSG for CRC errors.
 ******************************************************************************/
int ad4170_spi_reg_write(struct ad4170_dev *dev,
			 uint32_t reg_addr,
			 uint32_t reg_data);
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register on the AD4170
 * device by applying a mask and writing new data. This function is
 * useful when only certain bits of a register need to be updated without
 * affecting the other bits. It first reads the current value of the
 * register, applies the mask to clear the bits to be modified, and then
 * writes the new data. Ensure that the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD4170 device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, after applying
 * the mask. Only the bits specified by the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as if the register read or write fails.
 ******************************************************************************/
int ad4170_spi_reg_write_mask(struct ad4170_dev *dev,
			      uint32_t reg_addr,
			      uint8_t mask,
			      uint32_t reg_data);
/***************************************************************************//**
 * @brief Use this function to reset the SPI interface of the AD4170 device.
 * This is typically necessary when the SPI communication needs to be
 * reinitialized or if there are communication errors. The function must
 * be called with a valid device descriptor, and it will perform a
 * specific sequence of SPI writes to achieve the reset. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. If null, the function returns an error code.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the input is invalid.
 ******************************************************************************/
int ad4170_reset_spi_interface(struct ad4170_dev *dev);
/***************************************************************************//**
 * @brief This function is used to read a 16-bit data value from the AD4170
 * device. It should be called when you need to obtain the latest 16-bit
 * data from the device. Ensure that the device has been properly
 * initialized and configured before calling this function. The function
 * requires a valid pointer to a `uint16_t` variable where the data will
 * be stored. If the pointer is null, the function will return an error.
 * This function does not modify the device state or configuration.
 *
 * @param dev A pointer to an `ad4170_dev` structure representing the device.
 * This must be a valid, initialized device descriptor.
 * @param data A pointer to a `uint16_t` variable where the retrieved data will
 * be stored. Must not be null. If null, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * (e.g., if `data` is null).
 ******************************************************************************/
int ad4170_get_data16(struct ad4170_dev *dev, uint16_t *data);
/***************************************************************************//**
 * @brief Use this function to read a 16-bit data value and its associated
 * status from the AD4170 device. It is essential to ensure that both the
 * data and status pointers are valid and non-null before calling this
 * function. The function will return an error code if the pointers are
 * invalid or if the SPI read operation fails. This function is typically
 * used in applications where both the data and status information are
 * required for further processing or decision-making.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. The caller must ensure this is a valid and properly
 * configured device instance.
 * @param data A pointer to a uint16_t variable where the 16-bit data will be
 * stored. Must not be null.
 * @param status A pointer to a uint8_t variable where the status byte will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if any input pointers are null.
 ******************************************************************************/
int ad4170_get_data16s(struct ad4170_dev *dev, uint16_t *data,
		       uint8_t *status);
/***************************************************************************//**
 * @brief This function reads a 24-bit data value from the AD4170 device and
 * stores it in the provided memory location. It should be called when a
 * 24-bit data read is required from the device. The function requires a
 * valid device descriptor and a non-null pointer to store the data. If
 * the data pointer is null, the function returns an error. Ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. The device must be properly initialized before use.
 * @param data A pointer to a uint32_t variable where the 24-bit data will be
 * stored. Must not be null. If null, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the data pointer is null.
 ******************************************************************************/
int ad4170_get_data24(struct ad4170_dev *dev, uint32_t *data);
/***************************************************************************//**
 * @brief This function is used to read 24-bit data along with a status byte
 * from the AD4170 device. It should be called when the user needs to
 * obtain the latest data and status from the device. The function
 * requires valid pointers for both data and status parameters, and it
 * will return an error if either is null. The function must be called
 * with a properly initialized device structure.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. The device must be properly configured and initialized
 * before calling this function.
 * @param data A pointer to a uint32_t variable where the 24-bit data will be
 * stored. Must not be null. If null, the function returns -EINVAL.
 * @param status A pointer to a uint8_t variable where the status byte will be
 * stored. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the data and status pointers are populated with the
 * retrieved values.
 ******************************************************************************/
int ad4170_get_data24s(struct ad4170_dev *dev, uint32_t *data,
		       uint8_t *status);
/***************************************************************************//**
 * @brief This function is used to obtain a 32-bit data value from the AD4170
 * device. It should be called when a 32-bit data read is required from
 * the device. The function requires a valid device descriptor and a non-
 * null pointer to store the retrieved data. If the data pointer is null,
 * the function will return an error. This function is typically used in
 * applications where precise data acquisition from the AD4170 is
 * necessary.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. The device must be properly configured and initialized
 * before calling this function.
 * @param data A pointer to a uint32_t variable where the retrieved 32-bit data
 * will be stored. This pointer must not be null, otherwise the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. If successful, the 32-bit data is stored in the location
 * pointed to by the data parameter.
 ******************************************************************************/
int ad4170_get_data32(struct ad4170_dev *dev, uint32_t *data);
/***************************************************************************//**
 * @brief Use this function to obtain the data from a specific channel of the
 * AD4170 device. It is essential to ensure that the channel number is
 * within the valid range and that the data pointer is not null before
 * calling this function. This function should be called when you need to
 * read the data from a particular channel after the device has been
 * properly initialized and configured. If the channel number is invalid
 * or the data pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. The caller must ensure this is a valid, non-null pointer.
 * @param ch The channel number from which to read data. Must be less than
 * AD4170_NUM_CHANNELS, which is 16.
 * @param data A pointer to a uint32_t variable where the channel data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the channel number
 * is invalid or the data pointer is null.
 ******************************************************************************/
int ad4170_get_ch_data(struct ad4170_dev *dev, uint8_t ch, uint32_t *data);
/***************************************************************************//**
 * @brief Use this function to read a specified number of 16-bit data samples
 * from the AD4170 device into a provided buffer. This function is
 * typically called after the device has been properly initialized and
 * configured for data acquisition. Ensure that the buffer provided is
 * large enough to hold the number of samples requested. The function
 * returns an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param pbuf A pointer to a buffer where the read data will be stored. Must
 * not be null and should have enough space to store 'nb_samples'
 * 16-bit values.
 * @param nb_samples The number of 16-bit samples to read. Must be a positive
 * integer.
 * @return Returns an integer error code, with 0 indicating success and a non-
 * zero value indicating an error.
 ******************************************************************************/
int ad4170_read16(struct ad4170_dev *dev, uint32_t *pbuf,
		  uint16_t nb_samples);
/***************************************************************************//**
 * @brief Use this function to read a specified number of 16-bit samples, each
 * accompanied by a status byte, from the AD4170 device. This function is
 * typically called when you need to retrieve multiple samples in a
 * single operation. Ensure that the device is properly initialized and
 * configured before calling this function. The function will fill the
 * provided buffer with the read data, and it returns an integer status
 * code indicating success or failure.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param pbuf A pointer to a buffer where the read samples will be stored. The
 * buffer must be large enough to hold nb_samples * sizeof(uint32_t)
 * bytes. Must not be null.
 * @param nb_samples The number of 16-bit samples to read. Must be a positive
 * integer.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad4170_read16s(struct ad4170_dev *dev, uint32_t *pbuf,
		   uint16_t nb_samples);
/***************************************************************************//**
 * @brief This function is used to read a specified number of 24-bit data
 * samples from an AD4170 device into a provided buffer. It is typically
 * called when data acquisition from the device is required. The function
 * requires a valid device descriptor and a pre-allocated buffer to store
 * the read samples. The number of samples to read must be specified and
 * should not exceed the buffer's capacity. Proper initialization of the
 * device must be ensured before calling this function to avoid undefined
 * behavior.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device from
 * which data is to be read. Must not be null.
 * @param pbuf A pointer to a pre-allocated buffer where the read 24-bit data
 * samples will be stored. Must not be null and should have enough
 * space to hold 'nb_samples' 24-bit values.
 * @param nb_samples The number of 24-bit samples to read from the device. Must
 * be a positive integer and should not exceed the capacity of
 * 'pbuf'.
 * @return Returns an integer status code indicating success or failure of the
 * read operation. A non-zero value indicates an error.
 ******************************************************************************/
int ad4170_read24(struct ad4170_dev *dev, uint32_t *pbuf,
		  uint16_t nb_samples);
/***************************************************************************//**
 * @brief Use this function to read a specified number of 24-bit samples, along
 * with their status, from an AD4170 device. This function is typically
 * called after the device has been properly initialized and configured.
 * It is important to ensure that the device pointer is valid and that
 * the buffer provided is large enough to hold the requested number of
 * samples. The function will return an error code if the read operation
 * fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param pbuf A pointer to a buffer where the read samples will be stored. The
 * buffer must be large enough to hold nb_samples 24-bit samples.
 * Must not be null.
 * @param nb_samples The number of 24-bit samples to read. Must be a positive
 * integer.
 * @return Returns an integer status code indicating success or failure of the
 * read operation.
 ******************************************************************************/
int ad4170_read24s(struct ad4170_dev *dev, uint32_t *pbuf,
		   uint16_t nb_samples);
/***************************************************************************//**
 * @brief This function retrieves a specified number of 32-bit samples from the
 * AD4170 device and stores them in the provided buffer. It is typically
 * used when continuous or batch data acquisition is required from the
 * device. The function must be called with a valid device descriptor and
 * a pre-allocated buffer capable of holding the requested number of
 * samples. Ensure that the device is properly initialized and configured
 * before calling this function. The function returns an error code if
 * the read operation fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param pbuf A pointer to a pre-allocated buffer where the read samples will
 * be stored. Must not be null and should have enough space to hold
 * 'nb_samples' 32-bit values.
 * @param nb_samples The number of 32-bit samples to read from the device. Must
 * be a positive integer.
 * @return Returns an integer error code: 0 on success, or a negative error code
 * on failure.
 ******************************************************************************/
int ad4170_read32(struct ad4170_dev *dev, uint32_t *pbuf,
		  uint16_t nb_samples);
/***************************************************************************//**
 * @brief This function is used to continuously read a specified number of
 * samples from the ADC device, storing the results in the provided
 * output buffers. It is essential to ensure that the device is properly
 * initialized and configured before calling this function. The function
 * will return immediately if the number of samples requested is zero. It
 * is important to provide valid, non-null pointers for both data and
 * status output buffers, as the function will return an error if these
 * are not provided. The function processes each enabled channel,
 * repeating reads as configured, and stops once the requested number of
 * samples is read.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * ADC device. Must not be null.
 * @param data_out A pointer to a buffer where the read ADC data will be stored.
 * Must not be null.
 * @param status_out A pointer to a buffer where the status of each read will be
 * stored. Must not be null.
 * @param nb_samples The number of samples to read. If zero, the function
 * returns immediately without reading any samples.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if data_out or status_out is null.
 ******************************************************************************/
int ad4170_continuous_read(struct ad4170_dev *dev, uint32_t *data_out,
			   uint8_t *status_out, uint16_t nb_samples);
/***************************************************************************//**
 * @brief Use this function to exit the continuous read mode of the AD4170
 * device. It should be called when continuous reading is no longer
 * required. The function checks if the device is in continuous read mode
 * before attempting to exit. If the device is not in continuous read
 * mode, the function returns an error. Ensure that the device pointer is
 * valid before calling this function.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, or
 * -EACCES if the device is not in continuous read mode.
 ******************************************************************************/
int ad4170_continuous_read_exit(struct ad4170_dev *dev);
/***************************************************************************//**
 * @brief This function is used to exit the continuous transmit mode of an
 * AD4170 device. It should be called when the device is currently in
 * continuous transmit mode, as indicated by the `cont_read` field of the
 * `adc_ctrl` configuration being set to `AD4170_CONT_TRANSMIT_ON`. The
 * function will disable this mode and update the device configuration
 * accordingly. It is important to ensure that the device pointer is
 * valid and that the device is in the correct mode before calling this
 * function. If the device is not in continuous transmit mode, the
 * function will return an error. Additionally, if the digital interface
 * is not initialized, it will attempt to initialize it.
 *
 * @param dev A pointer to an `ad4170_dev` structure representing the device.
 * Must not be null. The device must be in continuous transmit mode,
 * otherwise, the function will return an error.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, -EACCES
 * if the device is not in continuous transmit mode, or other error
 * codes if the operation fails.
 ******************************************************************************/
int ad4170_continuous_transmit_exit(struct ad4170_dev *dev);
/***************************************************************************//**
 * @brief Use this function to reset the AD4170 device, ensuring it returns to
 * its default configuration. This is typically necessary when the device
 * needs to be reinitialized or when a known starting state is required.
 * The function must be called with a valid device descriptor that has
 * been previously initialized. It performs a software reset and waits
 * for the device to settle before updating the device's configuration to
 * match the default state. If the device descriptor is null, the
 * function returns an error.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device to reset. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad4170_reset(struct ad4170_dev *dev);
/***************************************************************************//**
 * @brief This function is used to obtain the current status from the AD4170
 * device by reading its status register. It should be called when the
 * status information is needed, such as checking for errors or
 * operational states. The function requires a valid device descriptor
 * and a pointer to store the status. It returns an error code if the
 * device descriptor or status pointer is null, or if the SPI read
 * operation fails.
 *
 * @param dev A pointer to an initialized `ad4170_dev` structure representing
 * the device. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @param status A pointer to a `uint16_t` where the status register value will
 * be stored. Must not be null. The function will return an error
 * if this parameter is invalid.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * `-EINVAL` for invalid parameters or other error codes for SPI read
 * failures.
 ******************************************************************************/
int ad4170_get_status(struct ad4170_dev *dev, uint16_t *status);
/***************************************************************************//**
 * @brief Use this function to set the pin muxing configuration for an AD4170
 * device. This function should be called when you need to change the pin
 * functionality settings, such as GPIO output, digital auxiliary
 * control, synchronization control, digital output strength, and SDO
 * readback delay. Ensure that the device structure is properly
 * initialized before calling this function. The function will return an
 * error code if the device pointer is null or if the SPI write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param pin_muxing A structure of type ad4170_pin_muxing containing the
 * desired pin muxing settings. The structure should be
 * properly populated with valid configuration values.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the SPI write operation fails.
 ******************************************************************************/
int ad4170_set_pin_muxing(struct ad4170_dev *dev,
			  struct ad4170_pin_muxing pin_muxing);
/***************************************************************************//**
 * @brief Use this function to configure the data clock divider of an AD4170
 * device. This function should be called when you need to change the
 * data clock division setting to one of the predefined values. It is
 * important to ensure that the device pointer is valid before calling
 * this function. If the current clock divider setting is already the
 * desired value, the function will return immediately without making any
 * changes. This function returns an error code if the device pointer is
 * null or if there is an issue writing to the device's register.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param div An enum value of type ad4170_dclk_div representing the desired
 * data clock divider setting. Valid values are defined by the
 * ad4170_dclk_div enumeration.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, or a
 * negative error code if there is a failure in writing to the device's
 * register.
 ******************************************************************************/
int ad4170_set_dclk_div(struct ad4170_dev *dev,
			enum ad4170_dclk_div clk_div);
/***************************************************************************//**
 * @brief This function configures the master clock divider for the AD4170
 * device, which determines the division factor applied to the master
 * clock. It should be called when a change in the clock division is
 * required. The function checks if the device pointer is valid and if
 * the current clock divider setting is different from the desired
 * setting before applying the change. It must be called with a valid
 * device structure that has been properly initialized.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param div An enum value of type ad4170_mclk_div representing the desired
 * master clock division factor. It must be a valid enum value.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error writing to the device register.
 ******************************************************************************/
int ad4170_set_mclk_div(struct ad4170_dev *dev,
			enum ad4170_mclk_div clk_div);
/***************************************************************************//**
 * @brief This function configures the clock source for the AD4170 device by
 * selecting one of the available clock options. It should be called when
 * there is a need to change the clock source of the device. The function
 * requires a valid device descriptor and a clock selection enumeration
 * value. If the current clock source is already set to the desired
 * value, the function returns immediately without making any changes. It
 * is important to ensure that the device descriptor is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param sel An enumeration value of type ad4170_clocksel representing the
 * desired clock source. Valid values are defined in the
 * ad4170_clocksel enum.
 * @return Returns 0 on success, a negative error code on failure, or -EINVAL if
 * the device pointer is null.
 ******************************************************************************/
int ad4170_set_clocksel(struct ad4170_dev *dev, enum ad4170_clocksel sel);
/***************************************************************************//**
 * @brief This function configures the standby control settings of the AD4170
 * device by writing to its standby control register. It should be called
 * when you need to change the standby behavior of the device. The
 * function requires a valid device structure and a standby control
 * value. It returns an error code if the device structure is null or if
 * the SPI write operation fails. Successful execution updates the
 * device's configuration with the new standby control value.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param standby_ctrl A 16-bit unsigned integer representing the standby
 * control settings to be written to the device. The valid
 * range is determined by the device's specifications.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for a null device pointer or an error code from the SPI write
 * operation.
 ******************************************************************************/
int ad4170_set_standby_ctrl(struct ad4170_dev *dev, uint16_t standby_ctrl);
/***************************************************************************//**
 * @brief This function configures the power-down switch settings of an AD4170
 * device. It should be called when you need to change the power
 * management settings of the device. The function requires a valid
 * device descriptor and a power-down switch configuration value. It
 * updates the device's configuration register and the internal
 * configuration state. Ensure that the device descriptor is properly
 * initialized before calling this function. If the device descriptor is
 * null, the function returns an error code.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param powerdown_sw A 16-bit unsigned integer representing the power-down
 * switch configuration. The valid range is determined by
 * the device's specifications.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4170_set_powerdown_sw(struct ad4170_dev *dev, uint16_t powerdown_sw);
/***************************************************************************//**
 * @brief This function sets the error enable register of the AD4170 device,
 * allowing the user to specify which errors should be monitored. It must
 * be called with a valid device structure that has been properly
 * initialized. The function will return an error code if the device
 * pointer is null or if the SPI write operation fails. This function is
 * typically used during device configuration to enable specific error
 * monitoring features.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param error_en A 16-bit unsigned integer specifying the error enable
 * settings to be written to the device's error enable register.
 * The valid range is 0 to 65535, representing different error
 * conditions to be enabled.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for a null device pointer or other error codes from the SPI
 * write operation.
 ******************************************************************************/
int ad4170_set_error_en(struct ad4170_dev *dev, uint16_t error_en);
/***************************************************************************//**
 * @brief Use this function to set the error register of the AD4170 device to a
 * specified value. This function is typically used to configure or
 * simulate error conditions for testing or diagnostic purposes. It
 * requires a valid device descriptor and an error code to be written to
 * the device's error register. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param error A 16-bit unsigned integer representing the error code to be set
 * in the device's error register. Valid error codes depend on the
 * device's specifications.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error occurred during the operation.
 ******************************************************************************/
int ad4170_set_error(struct ad4170_dev *dev, uint16_t error);
/***************************************************************************//**
 * @brief Use this function to obtain the current error status from an AD4170
 * device. It is essential to ensure that the device has been properly
 * initialized before calling this function. The function requires a
 * valid device descriptor and a pointer to a uint16_t variable where the
 * error status will be stored. If either the device descriptor or the
 * error pointer is null, the function will return an error code. This
 * function is useful for diagnosing issues with the device by checking
 * the error status.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param error A pointer to a uint16_t variable where the error status will be
 * stored. Must not be null. The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the device
 * descriptor or error pointer is null, or if there is an issue reading
 * the error status from the device.
 ******************************************************************************/
int ad4170_get_error(struct ad4170_dev *dev, uint16_t *error);
/***************************************************************************//**
 * @brief This function sets the ADC control parameters for the AD4170 device,
 * allowing configuration of features such as parallel filtering, data
 * register selection, continuous read status, and operating mode. It
 * must be called with a valid device structure that has been properly
 * initialized. The function updates the device's configuration and
 * writes the new settings to the ADC control register. If the continuous
 * read setting changes to enable continuous transmission, additional
 * internal adjustments are made. The function returns an error code if
 * the device pointer is null or if the register write operation fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. The function returns -EINVAL if this
 * parameter is null.
 * @param adc_ctrl A structure of type ad4170_adc_ctrl containing the desired
 * ADC control settings. The caller retains ownership of this
 * structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the register write operation fails.
 ******************************************************************************/
int ad4170_set_adc_ctrl(struct ad4170_dev *dev,
			struct ad4170_adc_ctrl adc_ctrl);
/***************************************************************************//**
 * @brief Use this function to configure which channels on the AD4170 device are
 * enabled or disabled. This function should be called after the device
 * has been initialized. It updates the channel enable register on the
 * device and also updates the local configuration state. Ensure that the
 * `dev` parameter is a valid, non-null pointer to an initialized
 * `ad4170_dev` structure. If the `dev` parameter is null, the function
 * will return an error code.
 *
 * @param dev A pointer to an `ad4170_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param channel_en A 16-bit value where each bit represents the enable state
 * of a corresponding channel. Valid values depend on the
 * specific channel configuration requirements of the AD4170
 * device.
 * @return Returns 0 on success. If the `dev` parameter is null, returns
 * -EINVAL. If there is an error writing to the device, returns a non-
 * zero error code.
 ******************************************************************************/
int ad4170_set_channel_en(struct ad4170_dev *dev, uint16_t channel_en);
/***************************************************************************//**
 * @brief Use this function to set up a specific channel on the AD4170 device
 * with the desired configuration. This function should be called after
 * the device has been initialized and before starting any data
 * acquisition. It configures the channel with the specified setup
 * parameters, including the number of repeats, delay, and setup number.
 * Ensure that the channel index is within the valid range and that the
 * device pointer is not null before calling this function.
 *
 * @param dev Pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param ch The channel index to configure. Must be less than
 * AD4170_NUM_CHANNELS.
 * @param setup An ad4170_channel_setup structure containing the configuration
 * parameters for the channel.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or the channel index is out of range.
 ******************************************************************************/
int ad4170_set_channel_setup(struct ad4170_dev *dev, uint8_t ch,
			     struct ad4170_channel_setup setup);
/***************************************************************************//**
 * @brief Use this function to set the analog input mapping for a specific
 * channel on the AD4170 device. It is essential to ensure that the
 * device pointer is valid and that the channel number is within the
 * valid range of available channels. This function should be called when
 * you need to update the channel configuration, and it will modify the
 * device's internal configuration to reflect the new mapping. If the
 * device pointer is null or the channel number is out of range, the
 * function will return an error code.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null.
 * @param ch The channel number to configure. Must be less than
 * AD4170_NUM_CHANNELS.
 * @param map An ad4170_channel_map structure specifying the new analog input
 * mapping for the channel. The structure should be properly
 * initialized with valid input selections.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or the channel number is invalid.
 ******************************************************************************/
int ad4170_set_channel_map(struct ad4170_dev *dev, uint8_t ch,
			   struct ad4170_channel_map map);
/***************************************************************************//**
 * @brief This function is used to configure a specific setup on the AD4170
 * device, identified by the setup index. It writes various configuration
 * parameters to the device's registers, including miscellaneous
 * settings, analog front end (AFE) settings, filter settings, filter
 * frequency select, offset, and gain. The function should be called when
 * a specific setup needs to be applied to the device. It is important to
 * ensure that the device pointer is valid and that the setup index is
 * within the valid range before calling this function. If the device
 * pointer is null or the setup index is out of range, the function will
 * return an error code.
 *
 * @param dev Pointer to the AD4170 device structure. Must not be null. The
 * caller retains ownership.
 * @param n Index of the setup to configure. Must be less than
 * AD4170_NUM_SETUPS. If out of range, the function returns an error.
 * @param setup Structure containing the setup configuration parameters. The
 * caller retains ownership.
 * @return Returns 0 on success or a negative error code on failure, such as
 * invalid parameters or communication errors.
 ******************************************************************************/
int ad4170_set_setup(struct ad4170_dev *dev, uint8_t n,
		     struct ad4170_setup setup);
/***************************************************************************//**
 * @brief Use this function to control the internal reference of the AD4170
 * device by enabling or disabling it. This function should be called
 * when you need to change the reference control state of the device.
 * Ensure that the device structure is properly initialized before
 * calling this function. If the device pointer is null, the function
 * will return an error code.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the internal reference.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4170_set_ref_control(struct ad4170_dev *dev, bool enable);
/***************************************************************************//**
 * @brief Use this function to configure the voltage bias for specific channels
 * on an AD4170 device. It is essential to ensure that the device pointer
 * is valid before calling this function, as passing a null pointer will
 * result in an error. This function should be called when you need to
 * update the voltage bias settings for the device's channels. The
 * function updates both the device's hardware register and its internal
 * configuration state.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param ch_mask A 16-bit mask specifying which channels to set the voltage
 * bias for. Each bit corresponds to a channel, and valid values
 * depend on the device's channel configuration.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in writing to the device's register.
 ******************************************************************************/
int ad4170_set_v_bias(struct ad4170_dev *dev, uint16_t ch_mask);
/***************************************************************************//**
 * @brief This function sets the pull-up current configuration for the specified
 * channels on the AD4170 device. It should be called when you need to
 * enable or modify the pull-up current settings for specific channels.
 * The function requires a valid device structure and a channel mask
 * indicating which channels to configure. It updates the device's
 * configuration with the new settings. Ensure the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. If null, the function returns an error.
 * @param ch_mask A 16-bit mask specifying the channels to configure for pull-up
 * current. Each bit corresponds to a channel, and the mask
 * determines which channels are affected.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in writing to the device.
 ******************************************************************************/
int ad4170_set_i_pullup(struct ad4170_dev *dev, uint16_t ch_mask);
/***************************************************************************//**
 * @brief This function sets the current source configuration for a specified
 * index on the AD4170 device. It should be used when you need to
 * configure the current source settings, such as the output pin and
 * current value, for one of the available current sources on the device.
 * The function must be called with a valid device structure and a valid
 * index within the range of available current sources. If the device
 * pointer is null or the index is out of range, the function returns an
 * error code. Successful execution updates the device's current source
 * configuration.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param n An index specifying which current source to configure. Must be less
 * than AD4170_NUM_CURRENT_SOURCE. If out of range, the function
 * returns an error.
 * @param current_source A structure of type ad4170_current_source specifying
 * the desired current source configuration, including the
 * output pin and current value.
 * @return Returns 0 on success or a negative error code on failure, such as
 * invalid parameters or communication errors.
 ******************************************************************************/
int ad4170_set_current_source(struct ad4170_dev *dev, uint8_t n,
			      struct ad4170_current_source current_source);
/***************************************************************************//**
 * @brief This function sets the FIR filter control parameters for the AD4170
 * device, including the FIR mode, coefficient set, and length. It must
 * be called with a valid device structure and a properly initialized FIR
 * control structure. The function writes the FIR control settings to the
 * device and, if applicable, writes the FIR coefficients to the device's
 * memory. It returns an error code if the device pointer is null or if
 * the FIR control parameters are invalid.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param fir_control A structure of type ad4170_fir_control containing the FIR
 * filter settings, including mode, coefficient set, length,
 * and coefficients. The FIR length must be between 0 and
 * AD4170_FIR_COEFF_MAX_LENGTH, and the mode must not be
 * AD4170_FIR_DEFAULT if coefficients are to be written.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other errors from underlying SPI
 * operations.
 ******************************************************************************/
int ad4170_set_fir_control(struct ad4170_dev *dev,
			   struct ad4170_fir_control fir_control);
/***************************************************************************//**
 * @brief This function is used to configure the DAC settings of an AD4170
 * device, including gain, hardware LDAC, hardware toggle, and enable
 * state. It should be called when the DAC configuration needs to be
 * updated. The function requires a valid device structure and a
 * configuration structure specifying the desired DAC settings. It
 * returns an error code if the device is not initialized or if any
 * configuration step fails.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param config An ad4170_dac_config structure containing the desired DAC
 * configuration settings, including gain, hardware LDAC, hardware
 * toggle, and enable state. The caller retains ownership of this
 * structure.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad4170_set_dac_config(struct ad4170_dev *dev,
			  struct ad4170_dac_config config);
/***************************************************************************//**
 * @brief Use this function to write a 12-bit code to the DAC data register of
 * the AD4170 device. This function is typically called when you need to
 * update the DAC output with a new value. Ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails, which can be used for
 * error handling.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param code A 12-bit unsigned integer representing the DAC code to be set.
 * Values outside the 12-bit range may lead to undefined behavior.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad4170_set_dac_data(struct ad4170_dev *dev, uint16_t code);
/***************************************************************************//**
 * @brief Use this function to set the DAC input register A of the AD4170 device
 * with a specific code value. This function is typically called when you
 * need to update the DAC input value for channel A. Ensure that the
 * device has been properly initialized before calling this function. The
 * function will return an error code if the operation fails, which
 * should be checked to ensure successful execution.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param code A 16-bit unsigned integer representing the code to be set in the
 * DAC input register A. The valid range is determined by the DAC's
 * resolution and configuration.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad4170_set_dac_inputa(struct ad4170_dev *dev, uint16_t code);
/***************************************************************************//**
 * @brief Use this function to set the DAC input register B of the AD4170 device
 * with a specific code value. This function is typically called when you
 * need to update the DAC input for channel B. Ensure that the device has
 * been properly initialized before calling this function. The function
 * will return an error code if the operation fails, which can be used
 * for error handling.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param code A 16-bit unsigned integer representing the code to be written to
 * the DAC input register B. The valid range is determined by the
 * DAC's resolution and configuration.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad4170_set_dac_inputb(struct ad4170_dev *dev, uint16_t code);
/***************************************************************************//**
 * @brief Use this function to trigger a software load DAC (LDAC) operation on
 * the AD4170 device, specifying the desired polarity. This function is
 * typically called when you need to update the DAC output with new data.
 * Ensure that the device is properly initialized before calling this
 * function. The function writes to a specific register to perform the
 * operation, and it returns an integer status code indicating success or
 * failure.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param polarity A boolean value indicating the desired polarity for the LDAC
 * operation. True or false are valid values.
 * @return Returns an integer status code, where 0 indicates success and a non-
 * zero value indicates an error.
 ******************************************************************************/
int ad4170_dac_sw_ldac(struct ad4170_dev *dev, bool polarity);
/***************************************************************************//**
 * @brief This function is used to toggle the DAC output of the AD4170 device
 * using software control. It is typically called when a change in the
 * DAC output is required without using hardware triggers. The function
 * requires a valid device structure and a polarity value to determine
 * the toggle state. It should be used when the device is properly
 * initialized and configured for DAC operations.
 *
 * @param dev A pointer to an initialized ad4170_dev structure representing the
 * device. Must not be null.
 * @param polarity A boolean value indicating the desired toggle state. True for
 * one state and false for the opposite.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int ad4170_dac_sw_toggle(struct ad4170_dev *dev, bool polarity);
/***************************************************************************//**
 * @brief This function is used to toggle the hardware DAC output of the AD4170
 * device based on the specified polarity. It requires a valid device
 * descriptor and the DIG_AUX2 pin must be configured as a DAC LDAC
 * input. If the device descriptor is null, the function returns an
 * error. Additionally, if the DIG_AUX2 pin is not configured correctly,
 * the function will not support the operation and return an error. This
 * function is typically used in applications where hardware control of
 * the DAC output is necessary.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device. Must
 * not be null. The function returns an error if this parameter is
 * null.
 * @param polarity A boolean value indicating the desired polarity for the DAC
 * output. True sets the output high, and false sets it low.
 * @return Returns 0 on success, -EINVAL if the device descriptor is null, or
 * -ENOTSUP if the DIG_AUX2 pin is not configured as a DAC LDAC input.
 ******************************************************************************/
int ad4170_dac_hw_toggle(struct ad4170_dev *dev, bool polarity);
/***************************************************************************//**
 * @brief This function sets up and initializes an AD4170 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure, configures the device according to the initialization
 * parameters, and performs necessary hardware checks and configurations.
 * If initialization fails, it returns an error code and cleans up any
 * allocated resources. The function requires valid pointers for both the
 * device and initialization parameters.
 *
 * @param device A pointer to a pointer of type `struct ad4170_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad4170_init_param` containing the
 * initialization parameters for the device. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or invalid device ID.
 ******************************************************************************/
int ad4170_init(struct ad4170_dev **device,
		struct ad4170_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD4170 device when it is no longer needed. This function should be
 * called to prevent memory leaks after the device has been initialized
 * and used. It is important to ensure that the device pointer is valid
 * and has been previously initialized by a successful call to the
 * initialization function. The function does not perform any error
 * checking on the input parameter, so passing a null or invalid pointer
 * will result in undefined behavior.
 *
 * @param dev A pointer to an ad4170_dev structure representing the device to be
 * removed. Must not be null and should point to a valid, initialized
 * device structure. The caller relinquishes ownership of the memory,
 * which will be freed by this function.
 * @return Always returns 0, indicating successful removal of the device.
 ******************************************************************************/
int ad4170_remove(struct ad4170_dev *dev);
/***************************************************************************//**
 * @brief Use this function to read and display the values of all registers in
 * the AD4170 device. It is useful for debugging and verifying the
 * current configuration of the device. The function must be called with
 * a valid device descriptor that has been properly initialized. If any
 * register read operation fails, the function will print an error
 * message and return the error code. Ensure that the device is in a
 * state where SPI communication is possible before calling this
 * function.
 *
 * @param dev A pointer to an initialized `ad4170_dev` structure representing
 * the device. Must not be null. The function will attempt to read
 * from this device's registers.
 * @return Returns 0 on success, or a negative error code if a register read
 * fails.
 ******************************************************************************/
int ad4170_regmap(struct ad4170_dev *dev);
#endif

