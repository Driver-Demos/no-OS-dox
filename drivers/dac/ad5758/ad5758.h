/***************************************************************************//**
 *   @file   ad5758.h
 *   @brief  Header file for ad5758 Driver.
 *   @author SPopa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
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
#ifndef AD5758_H_
#define AD5758_H_

#include "no_os_gpio.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h.
 */
#define NO_OS_GENMASK(h, l) \
		(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (31 - (h))))
#define NO_OS_BIT(x)	(1UL << (x))

/* AD5758 registers definition */
#define AD5758_REG_NOP						0x00
#define AD5758_REG_DAC_INPUT					0x01
#define AD5758_REG_DAC_OUTPUT					0x02
#define AD5758_REG_CLEAR_CODE					0x03
#define AD5758_REG_USER_GAIN					0x04
#define AD5758_REG_USER_OFFSET					0x05
#define AD5758_REG_DAC_CONFIG					0x06
#define AD5758_REG_SW_LDAC					0x07
#define AD5758_REG_KEY						0x08
#define AD5758_REG_GP_CONFIG1					0x09
#define AD5758_REG_GP_CONFIG2					0x0A
#define AD5758_REG_DCDC_CONFIG1					0x0B
#define AD5758_REG_DCDC_CONFIG2					0x0C
#define AD5758_REG_WDT_CONFIG					0x0F
#define AD5758_REG_DIGITAL_DIAG_CONFIG				0x10
#define AD5758_REG_ADC_CONFIG					0x11
#define AD5758_REG_FAULT_PIN_CONFIG				0x12
#define AD5758_REG_TWO_STAGE_READBACK_SELECT			0x13
#define AD5758_REG_DIGITAL_DIAG_RESULTS				0x14
#define AD5758_REG_ANALOG_DIAG_RESULTS				0x15
#define AD5758_REG_STATUS					0x16
#define AD5758_REG_CHIP_ID					0x17
#define AD5758_REG_FREQ_MONITOR					0x18
#define AD5758_REG_DEVICE_ID_0					0x19
#define AD5758_REG_DEVICE_ID_1					0x1A
#define AD5758_REG_DEVICE_ID_2					0x1B
#define AD5758_REG_DEVICE_ID_3					0x1C

/* AD5758_REG_DAC_CONFIG */
#define AD5758_DAC_CONFIG_RANGE_MSK			NO_OS_GENMASK(3, 0)
#define AD5758_DAC_CONFIG_RANGE_MODE(x)			(((x) & 0xF) << 0)
#define AD5758_DAC_CONFIG_OVRNG_EN_MSK			NO_OS_BIT(4)
#define AD5758_DAC_CONFIG_OVRNG_EN_MODE(x)		(((x) & 0x1) << 4)
#define AD5758_DAC_CONFIG_INT_EN_MSK			NO_OS_BIT(5)
#define AD5758_DAC_CONFIG_INT_EN_MODE(x)		(((x) & 0x1) << 5)
#define AD5758_DAC_CONFIG_OUT_EN_MSK			NO_OS_BIT(6)
#define AD5758_DAC_CONFIG_OUT_EN_MODE(x)		(((x) & 0x1) << 6)
#define AD5758_DAC_CONFIG_RSET_EXT_EN_MSK		NO_OS_BIT(7)
#define AD5758_DAC_CONFIG_RSET_EXT_EN_MODE(x)		(((x) & 0x1) << 7)
#define AD5758_DAC_CONFIG_SR_EN_MSK			NO_OS_BIT(8)
#define AD5758_DAC_CONFIG_SR_EN_MODE(x)			(((x) & 0x1) << 8)
#define AD5758_DAC_CONFIG_SR_CLOCK_MSK			NO_OS_GENMASK(12, 9)
#define AD5758_DAC_CONFIG_SR_CLOCK_MODE(x)		(((x) & 0xF) << 9)
#define AD5758_DAC_CONFIG_SR_STEP_MSK			NO_OS_GENMASK(15, 13)
#define AD5758_DAC_CONFIG_SR_STEP_MODE(x)		(((x) & 0x7) << 13)

/* AD5758_REG_SW_LDAC */
#define AD5758_SW_LDAC_COMMAND				0x1DAC

/* AD5758_REG_KEY */
#define AD5758_KEY_CODE_RESET_1				0x15FA
#define AD5758_KEY_CODE_RESET_2				0xAF51
#define AD5758_KEY_CODE_SINGLE_ADC_CONV			0x1ADC
#define AD5758_KEY_CODE_RESET_WDT			0x0D06
#define AD5758_KEY_CODE_CALIB_MEM_REFRESH		0xFCBA

/* AD5758_REG_GP_CONFIG1 */
#define AD5758_GP_CONFIG1_OSC_STOP_DETECT_EN_MSK	NO_OS_BIT(2)
#define AD5758_GP_CONFIG1_OSC_STOP_DETECT_EN_MODE(x)	(((x) & 0x1) << 2)
#define AD5758_GP_CONFIG1_SPI_DIAG_QUIET_EN_MSK		NO_OS_BIT(3)
#define AD5758_GP_CONFIG1_SPI_DIAG_QUIET_EN_MODE(x)	(((x) & 0x1) << 3)
#define AD5758_GP_CONFIG1_CLEAR_NOW_EN_MSK		NO_OS_BIT(4)
#define AD5758_GP_CONFIG1_CLEAR_NOW_EN_MODE(x)		(((x) & 0x1) << 4)
#define AD5758_GP_CONFIG1_NEG_OFFSET_EN_MSK		NO_OS_BIT(5)
#define AD5758_GP_CONFIG1_NEG_OFFSET_EN_MODE(x)		(((x) & 0x1) << 5)
#define AD5758_GP_CONFIG1_HART_EN_MSK			NO_OS_BIT(6)
#define AD5758_GP_CONFIG1_HART_EN_MODE(x)		(((x) & 0x1) << 6)
#define AD5758_GP_CONFIG1_CLKOUT_FREQ_MSK		NO_OS_GENMASK(9, 7)
#define AD5758_GP_CONFIG1_CLKOUT_FREQ_MODE(x)		(((x) & 0x7) << 7)
#define AD5758_GP_CONFIG1_CLKOUT_CONFIG_MSK		NO_OS_GENMASK(11, 10)
#define AD5758_GP_CONFIG1_CLKOUT_CONFIG_MODE(x)		(((x) & 0x3) << 10)
#define AD5758_GP_CONFIG1_SET_TEMP_THRESHOLD_MSK	NO_OS_GENMASK(13, 12)
#define AD5758_GP_CONFIG1_SET_TEMP_THRESHOLD_MODE(x)	(((x) & 0x3) << 12)

/* AD5758_REG_GP_CONFIG2 */
#define AD5758_GP_CONFIG2_FAULT_TIMEOUT_MSK		NO_OS_BIT(9)
#define AD5758_GP_CONFIG2_FAULT_TIMEOUT_MODE(x)		(((x) & 0x1) << 9)
#define AD5758_GP_CONFIG2_GLOBAL_SW_LDAC_MSK		NO_OS_BIT(10)
#define AD5758_GP_CONFIG2_GLOBAL_SW_LDAC_MODE(x)	(((x) & 0x1) << 10)
#define AD5758_GP_CONFIG2_INT_I_MONITOR_EN_MSK		NO_OS_BIT(11)
#define AD5758_GP_CONFIG2_INT_I_MONITOR_EN_MODE(x)	(((x) & 0x1) << 11)
#define AD5758_GP_CONFIG2_COMPARATOR_CONFIG_MSK		NO_OS_GENMASK(14, 13)
#define AD5758_GP_CONFIG2_COMPARATOR_CONFIG_MODE(x)	(((x) & 0x3) << 13)

/* AD5758_REG_DCDC_CONFIG1 */
#define AD5758_DCDC_CONFIG1_DCDC_VPROG_MSK		NO_OS_GENMASK(4, 0)
#define AD5758_DCDC_CONFIG1_DCDC_VPROG_MODE(x)		(((x) & 0x1F) << 0)
#define AD5758_DCDC_CONFIG1_DCDC_MODE_MSK		NO_OS_GENMASK(6, 5)
#define AD5758_DCDC_CONFIG1_DCDC_MODE_MODE(x)		(((x) & 0x3) << 5)

/* AD5758_REG_DCDC_CONFIG2 */
#define AD5758_DCDC_CONFIG2_ILIMIT_MSK			NO_OS_GENMASK(3, 1)
#define AD5758_DCDC_CONFIG2_ILIMIT_MODE(x)		(((x) & 0x7) << 1)
#define AD5758_DCDC_CONFIG2_ADC_CONTROL_DIAG_MSK	NO_OS_GENMASK(5, 4)
#define AD5758_DCDC_CONFIG2_ADC_CONTROL_DIAG_MODE(x)	(((x) & 0x3) << 4)
#define AD5758_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MSK	NO_OS_BIT(6)
#define AD5758_DCDC_CONFIG2_VIOUT_PULLDOWN_EN_MODE(x)	(((x) & 0x1) << 6)
#define AD5758_DCDC_CONFIG2_SHORT_DEGLITCH_MSK		NO_OS_BIT(7)
#define AD5758_DCDC_CONFIG2_SHORT_DEGLITCH_MODE(x)	(((x) & 0x1) << 7)
#define AD5758_DCDC_CONFIG2_READ_COMP_DIS_MSK		NO_OS_BIT(10)
#define AD5758_DCDC_CONFIG2_READ_COMP_DIS_MODE(x)	(((x) & 0x1) << 10)
#define AD5758_DCDC_CONFIG2_INTR_SAT_3WI_MSK		NO_OS_BIT(11)
#define AD5758_DCDC_CONFIG2_BUSY_3WI_MSK		NO_OS_BIT(12)

/* AD5758_REG_WDT_CONFIG */
#define AD5758_WDT_CONFIG_WDT_TIMEOUT_MSK			NO_OS_GENMASK(3, 0)
#define AD5758_WDT_CONFIG_WDT_TIMEOUT_MODE(x)			(((x) & 0xF) << 0)
#define AD5758_WDT_CONFIG_WDT_EN_MSK				NO_OS_BIT(6)
#define AD5758_WDT_CONFIG_WDT_EN_MODE(x)			(((x) & 0x1) << 6)
#define AD5758_WDT_CONFIG_KICK_ON_VALID_WRITE_MSK		NO_OS_BIT(8)
#define AD5758_WDT_CONFIG_KICK_ON_VALID_WRITE_MODE(x)		(((x) & 0x1) << 8)
#define AD5758_WDT_CONFIG_RESET_ON_WDT_FAIL_MSK			NO_OS_BIT(9)
#define AD5758_WDT_CONFIG_RESET_ON_WDT_FAIL_MODE(x)		(((x) & 0x1) << 9)
#define AD5758_WDT_CONFIG_CLEAR_ON_WDT_FAIL_MSK			NO_OS_BIT(10)
#define AD5758_WDT_CONFIG_CLEAR_ON_WDT_FAIL_MODE(x)		(((x) & 0x1) << 10)

/* AD5758_REG_DIGITAL_DIAG_CONFIG */
#define AD5758_DIG_DIAG_CONFIG_SPI_CRC_EN_MSK			NO_OS_BIT(0)
#define AD5758_DIG_DIAG_CONFIG_SPI_CRC_EN_MODE(x)		(((x) & 0x1) << 0)
#define AD5758_DIG_DIAG_CONFIG_FREQ_MON_EN_MSK			NO_OS_BIT(2)
#define AD5758_DIG_DIAG_CONFIG_FREQ_MON_EN_MODE(x)		(((x) & 0x1) << 2)
#define AD5758_DIG_DIAG_CONFIG_CAL_MEM_CRC_EN_MSK		NO_OS_BIT(3)
#define AD5758_DIG_DIAG_CONFIG_CAL_MEM_CRC_EN_MODE(x)		(((x) & 0x1) << 3)
#define AD5758_DIG_DIAG_CONFIG_INV_DAC_CHECK_EN_MSK		NO_OS_BIT(4)
#define AD5758_DIG_DIAG_CONFIG_INV_DAC_CHECK_EN_MODE(x)		(((x) & 0x1) << 4)
#define AD5758_DIG_DIAG_CONFIG_DAC_LATCH_MON_EN_MSK		NO_OS_BIT(6)
#define AD5758_DIG_DIAG_CONFIG_DAC_LATCH_MON_EN_MODE(x)		(((x) & 0x1) << 6)

/* AD5758_REG_ADC_CONFIG */
#define AD5758_ADC_CONFIG_ADC_IP_SELECT_MSK			NO_OS_GENMASK(4, 0)
#define AD5758_ADC_CONFIG_ADC_IP_SELECT_MODE(x)			(((x) & 0x1F) << 0)
#define AD5758_ADC_CONFIG_SEQUENCE_DATA_MSK			NO_OS_GENMASK(7, 5)
#define AD5758_ADC_CONFIG_SEQUENCE_DATA_MODE(x)			(((x) & 0x7) << 5)
#define AD5758_ADC_CONFIG_SEQUENCE_COMMAND_MSK			NO_OS_GENMASK(10, 8)
#define AD5758_ADC_CONFIG_SEQUENCE_COMMAND_MODE(x)		(((x) & 0x7) << 8)
#define AD5758_ADC_CONFIG_PPC_BUF_MSK				NO_OS_BIT(11)
#define AD5758_ADC_CONFIG_PPC_BUF_EN(x)				(((x) & 0x1) << 11)

/* AD5758_REG_FAULT_PIN_CONFIG */
#define AD5758_FAULT_PIN_CONFIG_MAIN_DIE_TEMP_ERR_MSK		NO_OS_BIT(0)
#define AD5758_FAULT_PIN_CONFIG_MAIN_DIE_TEMP_ERR_MODE(x) 	(((x) & 0x1) << 0)
#define AD5758_FAULT_PIN_CONFIG_DCDC_DIE_TEMP_ERR_MSK		NO_OS_BIT(1)
#define AD5758_FAULT_PIN_CONFIG_DCDC_DIE_TEMP_ERR_MODE(x)	(((x) & 0x1) << 1)
#define AD5758_FAULT_PIN_CONFIG_VOUT_SC_ERR_MSK			NO_OS_BIT(2)
#define AD5758_FAULT_PIN_CONFIG_VOUT_SC_ERR_MODE(x)		(((x) & 0x1) << 2)
#define AD5758_FAULT_PIN_CONFIG_IOUT_OC_ERR_MSK			NO_OS_BIT(3)
#define AD5758_FAULT_PIN_CONFIG_IOUT_OC_ERR_MODE(x)		(((x) & 0x1) << 3)
#define AD5758_FAULT_PIN_CONFIG_DCDC_P_SC_ERR_MSK		NO_OS_BIT(4)
#define AD5758_FAULT_PIN_CONFIG_DCDC_P_SC_ERR_MODE(x)		(((x) & 0x1) << 4)
#define AD5758_FAULT_PIN_CONFIG_SPI_CRC_ERR_MSK			NO_OS_BIT(6)
#define AD5758_FAULT_PIN_CONFIG_SPI_CRC_ERR_MODE(x)		(((x) & 0x1) << 6)
#define AD5758_FAULT_PIN_CONFIG_SLIPBIT_ERR_MSK			NO_OS_BIT(7)
#define AD5758_FAULT_PIN_CONFIG_SLIPBIT_ERR_MODE(x)		(((x) & 0x1) << 7)
#define AD5758_FAULT_PIN_CONFIG_WDT_ERR_MSK			NO_OS_BIT(8)
#define AD5758_FAULT_PIN_CONFIG_WDT_ERR_MODE(x)			(((x) & 0x1) << 8)
#define AD5758_FAULT_PIN_CONFIG_DAC_LATCH_MON_ERR_MSK		NO_OS_BIT(9)
#define AD5758_FAULT_PIN_CONFIG_DAC_LATCH_MON_ERR_MODE(x) 	(((x) & 0x1) << 9)
#define AD5758_FAULT_PIN_CONFIG_OSC_STOP_DETECT_MSK		NO_OS_BIT(10)
#define AD5758_FAULT_PIN_CONFIG_OSC_STOP_DETECT_MODE(x)		(((x) & 0x1) << 10)
#define AD5758_FAULT_PIN_CONFIG_INV_DAC_CHECK_ERR_MSK		NO_OS_BIT(12)
#define AD5758_FAULT_PIN_CONFIG_INV_DAC_CHECK_ERR_MODE(x) 	(((x) & 0x1) << 12)
#define AD5758_FAULT_PIN_CONFIG_PROT_SW_ERR_MSK			NO_OS_BIT(14)
#define AD5758_FAULT_PIN_CONFIG_PROT_SW_ERR_MODE(x)		(((x) & 0x1) << 14)
#define AD5758_FAULT_PIN_CONFIG_SPI_ACC_ERR_MSK			NO_OS_BIT(15)
#define AD5758_FAULT_PIN_CONFIG_SPI_ACC_ERR_MODE(x)		(((x) & 0x1) << 15)

/* AD5758_REG_TWO_STAGE_READBACK_SELECT */
#define AD5758_TWO_STAGE_READBACK_SELECT_MSK			NO_OS_GENMASK(4, 0)
#define AD5758_TWO_STAGE_READBACK_SELECT_MODE(x)		(((x) & 0x1F) << 0)
#define AD5758_TWO_STAGE_READBACK_SELECT_MODE_MSK		NO_OS_GENMASK(6, 5)
#define AD5758_TWO_STAGE_READBACK_SELECT_MODE_MODE(x)		(((x) & 0x3) << 5)

/* AD5758_REG_DIGITAL_DIAG_RESULTS */
#define AD5758_DIG_DIAG_RESULTS_SPI_CRC_ERR			NO_OS_BIT(0)
#define AD5758_DIG_DIAG_RESULTS_SLIPBIT_ERR_MSK			NO_OS_BIT(1)
#define AD5758_DIG_DIAG_RESULTS_SCLK_COUNT_ERR_MSK		NO_OS_BIT(2)
#define AD5758_DIG_DIAG_RESULTS_INVALID_SPI_ACC_ERR_MSK		NO_OS_BIT(4)
#define AD5758_DIG_DIAG_RESULTS_CAL_MEM_CRC_ERR_MSK		NO_OS_BIT(5)
#define AD5758_DIG_DIAG_RESULTS_INV_DAC_CHECK_ERR_MSK		NO_OS_BIT(6)
#define AD5758_DIG_DIAG_RESULTS_DAC_LATCH_MON_ERR_MSK		NO_OS_BIT(8)
#define AD5758_DIG_DIAG_RESULTS_3WI_RC_ERR_MSK			NO_OS_BIT(9)
#define AD5758_DIG_DIAG_RESULTS_WDT_ERR_MSK			NO_OS_BIT(11)
#define AD5758_DIG_DIAG_RESULTS_ERR_3WI_MSK			NO_OS_BIT(12)
#define AD5758_DIG_DIAG_RESULTS_RES_OCCURRED_MSK		NO_OS_BIT(13)
#define AD5758_DIG_DIAG_RESULTS_SLEW_BUSY_MSK			NO_OS_BIT(14)
#define AD5758_DIG_DIAG_RESULTS_CAL_MEM_UNREFRESHED_MSK		NO_OS_BIT(15)

/* AD5758_REG_ANALOG_DIAG_RESULTS */
#define AD5758_ANA_DIAG_RESULTS_REGOUT_ERR_MSK			NO_OS_BIT(0)
#define AD5758_ANA_DIAG_RESULTS_INT_AVCC_ERR_MSK		NO_OS_BIT(1)
#define AD5758_ANA_DIAG_RESULTS_REFIN_ERR_MSK			NO_OS_BIT(2)
#define AD5758_ANA_DIAG_RESULTS_REFOUT_ERR_MSK			NO_OS_BIT(3)
#define AD5758_ANA_DIAG_RESULTS_MAIN_DIE_TEMP_ERR_MSK		NO_OS_BIT(4)
#define AD5758_ANA_DIAG_RESULTS_DCDC_DIE_TEMP_ERR_MSK		NO_OS_BIT(5)
#define AD5758_ANA_DIAG_RESULTS_VOUT_SC_ERR_MSK			NO_OS_BIT(6)
#define AD5758_ANA_DIAG_RESULTS_IOUT_OC_ERR_MSK			NO_OS_BIT(7)
#define AD5758_ANA_DIAG_RESULTS_DCDC_P_PWR_ERR_MSK		NO_OS_BIT(9)
#define AD5758_ANA_DIAG_RESULTS_DCDC_P_SC_ERR_MSK		NO_OS_BIT(11)
#define AD5758_ANA_DIAG_RESULTS_FAULT_PROT_SW_ERR_MSK		NO_OS_BIT(13)

/* AD5758_REG_STATUS */
#define AD5758_STATUS_ADC_DATA_MSK			NO_OS_GENMASK(11, 0)
#define AD5758_STATUS_ADC_CH_MSK			NO_OS_GENMASK(16, 12)
#define AD5758_STATUS_ADC_BUSY_MSK			NO_OS_BIT(17)
#define AD5758_STATUS_WDT_STATUS_MSK			NO_OS_BIT(18)
#define AD5758_STATUS_ANA_DIAG_STATUS_MSK		NO_OS_BIT(19)
#define AD5758_STATUS_DIG_DIAG_STATUS_MSK		NO_OS_BIT(20)

#define AD5758_REG_WRITE(x) 		((0x80) | (x & 0x1F))
#define AD5758_CRC8_POLY		0x07 // x^8 + x^2 + x^1 + x^0

/*****************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad5758_dc_dc_mode` enumeration defines the various operational
 * modes for the DC-DC converter in the AD5758 device. It includes modes
 * for powering off the converter, as well as specific modes for current
 * and voltage operations in both DPC and PPC configurations. This
 * enumeration is used to configure the DC-DC converter's behavior in the
 * device.
 *
 * @param DC_DC_POWER_OFF Represents the mode where the DC-DC converter is
 * powered off.
 * @param DPC_CURRENT_MODE Represents the mode where the DC-DC converter
 * operates in DPC current mode.
 * @param DPC_VOLTAGE_MODE Represents the mode where the DC-DC converter
 * operates in DPC voltage mode.
 * @param PPC_CURRENT_MODE Represents the mode where the DC-DC converter
 * operates in PPC current mode.
 ******************************************************************************/
enum ad5758_dc_dc_mode {
	DC_DC_POWER_OFF,
	DPC_CURRENT_MODE,
	DPC_VOLTAGE_MODE,
	PPC_CURRENT_MODE,
};

/***************************************************************************//**
 * @brief The `ad5758_dig_diag_flags` is an enumeration that defines various
 * digital diagnostic flags for the AD5758 device. Each flag represents a
 * specific type of error or diagnostic condition that can be detected by
 * the device, such as SPI communication errors, calibration memory
 * errors, and watchdog timer errors. These flags are used to monitor the
 * health and status of the device, allowing for error detection and
 * troubleshooting.
 *
 * @param DIAG_SPI_CRC_ERR Indicates an error in SPI CRC.
 * @param DIAG_SLIPBIT_ERR Indicates a slip bit error.
 * @param DIAG_SCLK_COUNT_ERR Indicates an error in SCLK count.
 * @param DIAG_INVALID_SPI_ACCESS_ERR Indicates an invalid SPI access error.
 * @param DIAG_CAL_MEM_CRC_ERR Indicates a CRC error in calibration memory.
 * @param DIAG_INVERSE_DAC_CHECK_ERR Indicates an error in inverse DAC check.
 * @param DIAG_DAC_LATCH_MON_ERR Indicates an error in DAC latch monitoring.
 * @param DIAG_THREE_WI_RC_ERR Indicates a three-wire RC error.
 * @param DIAG_WDT_ERR Indicates a watchdog timer error.
 * @param DIAG_ERR_3WI Indicates a three-wire error.
 * @param DIAG_RESET_OCCURRED Indicates that a reset has occurred.
 ******************************************************************************/
enum ad5758_dig_diag_flags {
	DIAG_SPI_CRC_ERR = 0,
	DIAG_SLIPBIT_ERR = 1,
	DIAG_SCLK_COUNT_ERR = 2,
	DIAG_INVALID_SPI_ACCESS_ERR = 4,
	DIAG_CAL_MEM_CRC_ERR = 5,
	DIAG_INVERSE_DAC_CHECK_ERR = 6,
	DIAG_DAC_LATCH_MON_ERR = 8,
	DIAG_THREE_WI_RC_ERR = 9,
	DIAG_WDT_ERR = 11,
	DIAG_ERR_3WI = 12,
	DIAG_RESET_OCCURRED = 13,
};

/***************************************************************************//**
 * @brief The `ad5758_clkout_config` enumeration defines the possible
 * configurations for the clock output of the AD5758 device. It provides
 * two states: `CLKOUT_DISABLE` to turn off the clock output and
 * `CLKOUT_ENABLE` to activate it. This configuration is used to control
 * the clock output feature of the AD5758, which is a precision digital-
 * to-analog converter (DAC) with integrated dynamic power control.
 *
 * @param CLKOUT_DISABLE Represents the state where the clock output is
 * disabled.
 * @param CLKOUT_ENABLE Represents the state where the clock output is enabled.
 ******************************************************************************/
enum ad5758_clkout_config {
	CLKOUT_DISABLE,
	CLKOUT_ENABLE,
};

/***************************************************************************//**
 * @brief The `ad5758_clkout_freq` enumeration defines a set of constants
 * representing different clock output frequencies for the AD5758 device.
 * These frequencies range from 416 kHz to 588 kHz and are used to
 * configure the clock output settings of the device, allowing for
 * precise control over the clock signal generated by the AD5758.
 *
 * @param CLKOUT_FREQ_416_KHZ Represents a clock output frequency of 416 kHz.
 * @param CLKOUT_FREQ_435_KHZ Represents a clock output frequency of 435 kHz.
 * @param CLKOUT_FREQ_454_KHZ Represents a clock output frequency of 454 kHz.
 * @param CLKOUT_FREQ_476_KHZ Represents a clock output frequency of 476 kHz.
 * @param CLKOUT_FREQ_500_KHZ Represents a clock output frequency of 500 kHz.
 * @param CLKOUT_FREQ_526_KHZ Represents a clock output frequency of 526 kHz.
 * @param CLKOUT_FREQ_555_KHZ Represents a clock output frequency of 555 kHz.
 * @param CLKOUT_FREQ_588_KHZ Represents a clock output frequency of 588 kHz.
 ******************************************************************************/
enum ad5758_clkout_freq {
	CLKOUT_FREQ_416_KHZ,
	CLKOUT_FREQ_435_KHZ,
	CLKOUT_FREQ_454_KHZ,
	CLKOUT_FREQ_476_KHZ,
	CLKOUT_FREQ_500_KHZ,
	CLKOUT_FREQ_526_KHZ,
	CLKOUT_FREQ_555_KHZ,
	CLKOUT_FREQ_588_KHZ,
};

/***************************************************************************//**
 * @brief The `ad5758_slew_rate_clk` enumeration defines a set of constants
 * representing different clock frequencies for controlling the slew rate
 * in the AD5758 device. These constants are used to configure the
 * device's slew rate clock, allowing for precise control over the rate
 * at which the output voltage or current changes. The available
 * frequencies range from 240 kHz to 16 Hz, providing flexibility for
 * various application requirements.
 *
 * @param SR_CLOCK_240_KHZ Represents a slew rate clock frequency of 240 kHz.
 * @param SR_CLOCK_200_KHZ Represents a slew rate clock frequency of 200 kHz.
 * @param SR_CLOCK_150_KHZ Represents a slew rate clock frequency of 150 kHz.
 * @param SR_CLOCK_128_KHZ Represents a slew rate clock frequency of 128 kHz.
 * @param SR_CLOCK_64_KHZ Represents a slew rate clock frequency of 64 kHz.
 * @param SR_CLOCK_32_KHZ Represents a slew rate clock frequency of 32 kHz.
 * @param SR_CLOCK_16_KHZ Represents a slew rate clock frequency of 16 kHz.
 * @param SR_CLOCK_8_KHZ Represents a slew rate clock frequency of 8 kHz.
 * @param SR_CLOCK_4_KHZ Represents a slew rate clock frequency of 4 kHz.
 * @param SR_CLOCK_2_KHZ Represents a slew rate clock frequency of 2 kHz.
 * @param SR_CLOCK_1_KHZ Represents a slew rate clock frequency of 1 kHz.
 * @param SR_CLOCK_512_HZ Represents a slew rate clock frequency of 512 Hz.
 * @param SR_CLOCK_256_HZ Represents a slew rate clock frequency of 256 Hz.
 * @param SR_CLOCK_128_HZ Represents a slew rate clock frequency of 128 Hz.
 * @param SR_CLOCK_64_HZ Represents a slew rate clock frequency of 64 Hz.
 * @param SR_CLOCK_16_HZ Represents a slew rate clock frequency of 16 Hz.
 ******************************************************************************/
enum ad5758_slew_rate_clk {
	SR_CLOCK_240_KHZ,
	SR_CLOCK_200_KHZ,
	SR_CLOCK_150_KHZ,
	SR_CLOCK_128_KHZ,
	SR_CLOCK_64_KHZ,
	SR_CLOCK_32_KHZ,
	SR_CLOCK_16_KHZ,
	SR_CLOCK_8_KHZ,
	SR_CLOCK_4_KHZ,
	SR_CLOCK_2_KHZ,
	SR_CLOCK_1_KHZ,
	SR_CLOCK_512_HZ,
	SR_CLOCK_256_HZ,
	SR_CLOCK_128_HZ,
	SR_CLOCK_64_HZ,
	SR_CLOCK_16_HZ,
};

/***************************************************************************//**
 * @brief The `ad5758_dc_dc_ilimt` enumeration defines a set of constants
 * representing different current limit settings for the AD5758 device's
 * DC-DC converter. Each constant corresponds to a specific current limit
 * value, ranging from 150 mA to 400 mA, which can be used to configure
 * the device's current limiting feature.
 *
 * @param ILIMIT_150_mA Represents a current limit of 150 milliamperes.
 * @param ILIMIT_200_mA Represents a current limit of 200 milliamperes.
 * @param ILIMIT_250_mA Represents a current limit of 250 milliamperes.
 * @param ILIMIT_300_mA Represents a current limit of 300 milliamperes.
 * @param ILIMIT_350_mA Represents a current limit of 350 milliamperes.
 * @param ILIMIT_400_mA Represents a current limit of 400 milliamperes.
 ******************************************************************************/
enum ad5758_dc_dc_ilimt {
	ILIMIT_150_mA,
	ILIMIT_200_mA,
	ILIMIT_250_mA,
	ILIMIT_300_mA,
	ILIMIT_350_mA,
	ILIMIT_400_mA,
};

/***************************************************************************//**
 * @brief The `ad5758_output_range` enumeration defines various output ranges
 * for the AD5758 device, which can be configured to output either
 * voltage or current within specified limits. Each enumerator
 * corresponds to a specific range, allowing the device to be set to
 * output voltages from 0V to 10V or currents from 0mA to 24mA, among
 * other configurations. This flexibility is crucial for applications
 * requiring precise control over output signals.
 *
 * @param RANGE_0V_5V Represents an output range of 0 to 5 volts.
 * @param RANGE_0V_10V Represents an output range of 0 to 10 volts.
 * @param RANGE_M5V_5V Represents an output range of -5 to 5 volts.
 * @param RANGE_M10V_10V Represents an output range of -10 to 10 volts.
 * @param RANGE_0mA_20mA Represents an output range of 0 to 20 milliamps.
 * @param RANGE_0mA_24mA Represents an output range of 0 to 24 milliamps.
 * @param RANGE_4mA_24mA Represents an output range of 4 to 24 milliamps.
 * @param RANGE_M20mA_20mA Represents an output range of -20 to 20 milliamps.
 * @param RANGE_M24mA_24mA Represents an output range of -24 to 24 milliamps.
 * @param RANGE_M1mA_22mA Represents an output range of -1 to 22 milliamps.
 ******************************************************************************/
enum ad5758_output_range {
	RANGE_0V_5V = 0,
	RANGE_0V_10V = 1,
	RANGE_M5V_5V = 2,
	RANGE_M10V_10V = 3,
	RANGE_0mA_20mA = 8,
	RANGE_0mA_24mA = 9,
	RANGE_4mA_24mA = 10,
	RANGE_M20mA_20mA = 11,
	RANGE_M24mA_24mA = 12,
	RANGE_M1mA_22mA = 13,
};

/***************************************************************************//**
 * @brief The `ad5758_adc_ip` enum defines a set of constants representing
 * various analog-to-digital converter (ADC) input channels for the
 * AD5758 device. Each enumerator corresponds to a specific input
 * channel, such as temperature sensors, reference voltages, and ground
 * connections, which are used for monitoring and diagnostic purposes
 * within the device.
 *
 * @param ADC_IP_MAIN_DIE_TEMP Represents the main die temperature input.
 * @param ADC_IP_DCDC_DIE_TEMP Represents the DCDC die temperature input.
 * @param ADC_IP_REFIN Represents the reference input.
 * @param ADC_IP_REF2 Represents the second reference input.
 * @param ADC_IP_VSENSE Represents the voltage sense input.
 * @param ADC_IP_MVSENSE Represents the millivolt sense input.
 * @param ADC_IP_INT_AVCC Represents the internal AVCC input.
 * @param ADC_IP_REGOUT Represents the regulator output input.
 * @param ADC_IP_VLOGIC Represents the logic voltage input.
 * @param ADC_IP_INT_CURR_MON_VOUT Represents the internal current monitor
 * output input.
 * @param ADC_IP_REFGND Represents the reference ground input.
 * @param ADC_IP_AGND Represents the analog ground input.
 * @param ADC_IP_DGND Represents the digital ground input.
 * @param ADC_IP_VDPC Represents the VDPC input.
 * @param ADC_IP_AVDD2 Represents the AVDD2 input.
 * @param ADC_IP_AVSS Represents the AVSS input.
 * @param ADC_IP_DCDC_DIE_NODE Represents the DCDC die node input.
 * @param ADC_IP_REFOUT Represents the reference output input.
 ******************************************************************************/
enum ad5758_adc_ip {
	ADC_IP_MAIN_DIE_TEMP = 0,
	ADC_IP_DCDC_DIE_TEMP = 1,
	ADC_IP_REFIN = 3,
	ADC_IP_REF2 = 4,
	ADC_IP_VSENSE = 13,
	ADC_IP_MVSENSE = 14,
	ADC_IP_INT_AVCC = 20,
	ADC_IP_REGOUT = 21,
	ADC_IP_VLOGIC = 22,
	ADC_IP_INT_CURR_MON_VOUT = 23,
	ADC_IP_REFGND = 24,
	ADC_IP_AGND = 25,
	ADC_IP_DGND = 26,
	ADC_IP_VDPC = 27,
	ADC_IP_AVDD2 = 28,
	ADC_IP_AVSS = 29,
	ADC_IP_DCDC_DIE_NODE = 30,
	ADC_IP_REFOUT = 31,
};

/***************************************************************************//**
 * @brief The `ad5758_adc_mode` enumeration defines various modes of operation
 * for the ADC (Analog-to-Digital Converter) in the AD5758 device. Each
 * mode is associated with a specific integer value, which is used to
 * configure the ADC's behavior, such as performing key sequences,
 * automatic sequences, single conversions, or single key conversions.
 * This enumeration is crucial for setting the ADC to the desired
 * operational mode in applications involving the AD5758 device.
 *
 * @param ADC_MODE_KEY_SEQ Represents the ADC mode for key sequence with a value
 * of 2.
 * @param ADC_MODE_AUTO_SEQ Represents the ADC mode for automatic sequence with
 * a value of 3.
 * @param ADC_MODE_SINGLE_CONV Represents the ADC mode for single conversion
 * with a value of 4.
 * @param ADC_MODE_SINGLE_KEY_CONV Represents the ADC mode for single key
 * conversion with a value of 5.
 ******************************************************************************/
enum ad5758_adc_mode {
	ADC_MODE_KEY_SEQ = 2,
	ADC_MODE_AUTO_SEQ = 3,
	ADC_MODE_SINGLE_CONV = 4,
	ADC_MODE_SINGLE_KEY_CONV = 5,
};

/***************************************************************************//**
 * @brief The `ad5758_dev` structure is a comprehensive data structure used to
 * represent the configuration and state of an AD5758 device, which is a
 * precision digital-to-analog converter (DAC) with integrated DC-DC
 * converter. It includes pointers to SPI and GPIO descriptors for
 * communication and control, as well as various configuration settings
 * such as CRC enablement, DC-DC converter mode, clock output
 * configuration and frequency, current limit, output range, and slew
 * rate clock. This structure is essential for managing the device's
 * operation and interfacing with it programmatically.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param reset_n Pointer to a GPIO descriptor for the reset pin.
 * @param ldac_n Pointer to a GPIO descriptor for the LDAC pin.
 * @param crc_en 8-bit unsigned integer to enable or disable CRC.
 * @param dc_dc_mode Enum to specify the DC-DC converter mode.
 * @param clkout_config Enum to configure the clock output.
 * @param clkout_freq Enum to set the clock output frequency.
 * @param dc_dc_ilimit Enum to set the DC-DC converter current limit.
 * @param output_range Enum to define the output range of the device.
 * @param slew_rate_clk Enum to configure the slew rate clock.
 ******************************************************************************/
struct ad5758_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
	/* GPIO */
	struct no_os_gpio_desc *reset_n;
	struct no_os_gpio_desc *ldac_n;
	/* Device Settings */
	uint8_t crc_en;
	enum ad5758_dc_dc_mode dc_dc_mode;
	enum ad5758_clkout_config clkout_config;
	enum ad5758_clkout_freq clkout_freq;
	enum ad5758_dc_dc_ilimt dc_dc_ilimit;
	enum ad5758_output_range output_range;
	enum ad5758_slew_rate_clk slew_rate_clk;
};

/***************************************************************************//**
 * @brief The `ad5758_init_param` structure is used to initialize and configure
 * the AD5758 device, a precision digital-to-analog converter (DAC) with
 * integrated DC-DC converter. It includes parameters for SPI and GPIO
 * initialization, as well as various device settings such as CRC error
 * checking, DC-DC converter mode, clock output configuration, current
 * limit, output range, and slew rate control. This structure is
 * essential for setting up the device's operational parameters before
 * use.
 *
 * @param spi_init Initializes the SPI interface parameters for the device.
 * @param reset_n Configures the GPIO parameters for the reset pin.
 * @param ldac_n Configures the GPIO parameters for the LDAC pin.
 * @param crc_en Enables or disables CRC error checking.
 * @param dc_dc_mode Specifies the DC-DC converter mode.
 * @param clkout_config Configures the clock output settings.
 * @param clkout_freq Sets the frequency for the clock output.
 * @param dc_dc_ilimit Defines the current limit for the DC-DC converter.
 * @param output_range Specifies the output range of the device.
 * @param slew_rate_clk Sets the clock frequency for the slew rate control.
 ******************************************************************************/
struct ad5758_init_param {
	/* SPI */
	struct no_os_spi_init_param spi_init;
	/* GPIO */
	struct no_os_gpio_init_param reset_n;
	struct no_os_gpio_init_param ldac_n;
	/* Device Settings */
	uint8_t crc_en;
	enum ad5758_dc_dc_mode dc_dc_mode;
	enum ad5758_clkout_config clkout_config;
	enum ad5758_clkout_freq clkout_freq;
	enum ad5758_dc_dc_ilimt dc_dc_ilimit;
	enum ad5758_output_range output_range;
	enum ad5758_slew_rate_clk slew_rate_clk;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function is used to enable or disable the CRC error checking
 * feature for SPI communication on the AD5758 device. It should be
 * called when configuring the device to ensure data integrity during SPI
 * transactions. The function requires a valid device structure and a
 * flag indicating whether CRC should be enabled or disabled. It returns
 * an error code if the operation fails, otherwise it updates the
 * device's CRC setting.
 *
 * @param dev A pointer to an ad5758_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param crc_en A uint8_t value where 1 enables CRC error checking and 0
 * disables it. Values outside this range are not valid and may
 * result in undefined behavior.
 * @return Returns 0 on success, or -1 if an error occurs during the operation.
 ******************************************************************************/
int32_t ad5758_set_crc(struct ad5758_dev *dev, uint8_t crc_en);
/***************************************************************************//**
 * @brief This function is used to ensure that the calibration memory refresh
 * cycle of the AD5758 device has completed before proceeding with
 * further operations. It should be called when a refresh cycle is
 * expected or required, such as after certain configuration changes. The
 * function continuously checks the status of the calibration memory
 * until it is refreshed, ensuring that subsequent operations are based
 * on the latest calibration data. It is important to call this function
 * to avoid using stale calibration data, which could lead to incorrect
 * device behavior.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null. The function will read from the device
 * to check the refresh status.
 * @return Returns 0 on successful completion of the refresh cycle wait. No
 * other values are returned.
 ******************************************************************************/
int32_t ad5758_wait_for_refresh_cycle(struct ad5758_dev *dev);
/***************************************************************************//**
 * @brief This function is used to reset the AD5758 device via software, which
 * is useful for reinitializing the device to a known state without
 * requiring a hardware reset. It should be called when a reset of the
 * device is necessary, such as after a configuration change or error
 * condition. The function writes specific reset codes to the device and
 * waits for a short delay to ensure the reset is processed. It is
 * important to ensure that the device structure is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if communication with the
 * device fails.
 * @return Returns 0 on success, or -1 if an error occurs during the reset
 * process.
 ******************************************************************************/
int32_t ad5758_soft_reset(struct ad5758_dev *dev);
/***************************************************************************//**
 * @brief Use this function to refresh the calibration memory of an AD5758
 * device, which is necessary to ensure the device operates with the most
 * current calibration data. This function should be called when a
 * calibration update is required. It must be called with a valid device
 * structure that has been properly initialized. The function will return
 * an error code if the refresh operation fails, and it waits for the
 * refresh cycle to complete before returning.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad5758_calib_mem_refresh(struct ad5758_dev *dev);
/***************************************************************************//**
 * @brief This function configures the DC-DC converter mode of the AD5758 device
 * to the specified mode. It must be called with a valid device structure
 * and a mode from the predefined enumeration. If the mode is set to
 * PPC_CURRENT_MODE, additional configuration is performed to enable PPC
 * buffers. The function ensures that the configuration is complete by
 * polling the device until the communication is finished. It should be
 * used when the DC-DC converter mode needs to be changed, and it returns
 * an error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param mode An enum value of type ad5758_dc_dc_mode specifying the desired
 * DC-DC converter mode. Valid values are defined in the
 * ad5758_dc_dc_mode enumeration.
 * @return Returns 0 on success or -1 on failure, indicating an error occurred
 * during the operation.
 ******************************************************************************/
int32_t ad5758_set_dc_dc_conv_mode(struct ad5758_dev *dev,
				   enum ad5758_dc_dc_mode mode);
/***************************************************************************//**
 * @brief This function configures the current limit for the DC-DC converter in
 * the AD5758 device. It should be called when the user needs to set or
 * change the current limit to one of the predefined values. The function
 * requires a valid device structure and a current limit enumeration
 * value. It writes the configuration to the device and waits for the
 * operation to complete. If the operation fails, it returns an error
 * code.
 *
 * @param dev A pointer to an ad5758_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param ilimit An enumeration value of type ad5758_dc_dc_ilimt representing
 * the desired current limit. Must be a valid enumeration value.
 * @return Returns 0 on success or -1 on failure.
 ******************************************************************************/
int32_t ad5758_set_dc_dc_ilimit(struct ad5758_dev *dev,
				enum ad5758_dc_dc_ilimt ilimit);
int32_t ad5758_fault_prot_switch_en(struct ad5758_dev *dev, uint8_t enable);
/***************************************************************************//**
 * @brief This function is used to control the internal buffers of the AD5758
 * device, which may be necessary for certain operational modes or
 * configurations. It should be called when there is a need to enable or
 * disable these buffers, typically as part of the device setup or
 * configuration process. The function requires a valid device structure
 * and a flag indicating whether to enable or disable the buffers. It
 * returns an error code if the operation fails, ensuring that the caller
 * can handle such cases appropriately.
 *
 * @param dev A pointer to an ad5758_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param enable A uint8_t value where non-zero enables the internal buffers and
 * zero disables them. The value is used to set the internal
 * buffer state accordingly.
 * @return Returns an int32_t value indicating success (0) or failure (-1).
 ******************************************************************************/
int32_t ad5758_internal_buffers_en(struct ad5758_dev *dev, uint8_t enable);
/***************************************************************************//**
 * @brief This function configures the output range of the AD5758 device to the
 * specified range. It should be called when the output range needs to be
 * changed, and it is important to ensure that the device is properly
 * initialized before calling this function. The function will initiate a
 * calibration memory refresh, and it is crucial to avoid any subsequent
 * SPI writes until the refresh cycle is complete. This function returns
 * an error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param range An enum value of type ad5758_output_range specifying the desired
 * output range. Valid values are defined in the
 * ad5758_output_range enumeration.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad5758_set_out_range(struct ad5758_dev *dev,
			     enum ad5758_output_range range);
/***************************************************************************//**
 * @brief This function configures the slew rate settings of the AD5758 device,
 * allowing the user to enable or disable the slew rate feature and set
 * the desired clock frequency for the slew rate operation. It should be
 * called when the user needs to adjust the output signal's transition
 * rate to meet specific application requirements. The function must be
 * called with a valid device structure that has been properly
 * initialized. It returns an error code if the configuration fails,
 * ensuring that the user can handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param clk An enum value of type ad5758_slew_rate_clk specifying the desired
 * clock frequency for the slew rate. Must be a valid enum value.
 * @param enable A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) the slew rate feature.
 * @return Returns an int32_t value: 0 on success, or a negative error code on
 * failure.
 ******************************************************************************/
int32_t ad5758_slew_rate_config(struct ad5758_dev *dev,
				enum ad5758_slew_rate_clk clk,
				uint8_t enable);
/***************************************************************************//**
 * @brief This function is used to write a specified digital input code to the
 * DAC input register of the AD5758 device. It should be called when you
 * need to update the DAC input value. The function requires a valid
 * device structure that has been properly initialized. It returns an
 * error code if the write operation fails, which can be used to handle
 * communication errors.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param code A 16-bit unsigned integer representing the digital input code to
 * be written to the DAC input register. Valid range is 0 to 65535.
 * @return Returns 0 on success, or -1 if the write operation fails.
 ******************************************************************************/
int32_t ad5758_dac_input_write(struct ad5758_dev *dev, uint16_t code);
/***************************************************************************//**
 * @brief This function is used to control the output state of the DAC on the
 * AD5758 device. It should be called when you need to enable or disable
 * the DAC output, typically after the device has been initialized and
 * configured. The function writes to the DAC configuration register to
 * set the output enable state. It is important to ensure that the device
 * pointer is valid and properly initialized before calling this
 * function. A short delay is introduced after the operation to ensure
 * the setting takes effect.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param enable A uint8_t value where 1 enables the DAC output and 0 disables
 * it. Any non-zero value is treated as enable.
 * @return Returns 0 on success, or -1 if an error occurs during the SPI write
 * operation.
 ******************************************************************************/
int32_t ad5758_dac_output_en(struct ad5758_dev *dev, uint8_t enable);
/***************************************************************************//**
 * @brief Use this function to clear a specific digital diagnostic flag on the
 * AD5758 device. This is typically done to acknowledge and reset the
 * status of a diagnostic condition that has been previously detected.
 * The function should be called when a diagnostic flag needs to be
 * cleared to ensure the device's diagnostic status is up-to-date. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param flag An enumerated value of type ad5758_dig_diag_flags representing
 * the specific diagnostic flag to clear. Must be a valid flag from
 * the ad5758_dig_diag_flags enumeration.
 * @return Returns 0 on success, or -1 if an error occurs during the operation.
 ******************************************************************************/
int32_t ad5758_clear_dig_diag_flag(struct ad5758_dev *dev,
				   enum ad5758_dig_diag_flags flag);
/***************************************************************************//**
 * @brief Use this function to configure the AD5758 device to read from a
 * specific ADC input channel. This function should be called when you
 * need to change the ADC input selection for the device. Ensure that the
 * device is properly initialized before calling this function. The
 * function will return an error code if the operation fails, which can
 * be used for error handling in the application.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param adc_ip_sel An enum value of type ad5758_adc_ip specifying the ADC
 * input channel to select. Must be a valid enum value.
 * @return Returns 0 on success or -1 on failure, indicating an error occurred
 * during the operation.
 ******************************************************************************/
int32_t ad5758_select_adc_ip(struct ad5758_dev *dev,
			     enum ad5758_adc_ip adc_ip_sel);
/***************************************************************************//**
 * @brief This function sets the clock output configuration and frequency for
 * the AD5758 device. It should be called when you need to enable or
 * disable the clock output or change its frequency. The function
 * requires a valid device structure and appropriate configuration and
 * frequency values. It returns an error code if the operation fails,
 * ensuring that the device's clock output settings are correctly
 * updated.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param config An enum value of type ad5758_clkout_config specifying whether
 * the clock output should be enabled or disabled.
 * @param freq An enum value of type ad5758_clkout_freq specifying the desired
 * clock output frequency.
 * @return Returns 0 on success or -1 on failure, indicating whether the clock
 * output configuration was successfully applied.
 ******************************************************************************/
int32_t ad5758_set_clkout_config(struct ad5758_dev *dev,
				 enum ad5758_clkout_config config,
				 enum ad5758_clkout_freq freq);
/***************************************************************************//**
 * @brief Use this function to set the ADC depth by specifying the number of
 * channels to be used in the sequence. This function should be called
 * when configuring the ADC settings on the AD5758 device. It is
 * important to ensure that the number of channels is within the valid
 * range of 1 to 8, inclusive. If the number of channels is outside this
 * range, the function will return an error code and no changes will be
 * made. This function requires a valid device structure that has been
 * properly initialized.
 *
 * @param dev A pointer to an ad5758_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param num_of_channels The number of channels to configure for ADC depth.
 * Valid values are from 1 to 8. Values outside this
 * range will result in an error.
 * @return Returns 0 on success, or a negative error code if the number of
 * channels is invalid or if there is a failure in writing to the
 * device.
 ******************************************************************************/
int32_t ad5758_select_adc_depth(struct ad5758_dev *dev,
				uint8_t num_of_channels);
/***************************************************************************//**
 * @brief Use this function to set the input channel and the ADC input selection
 * for the AD5758 device. This function must be called after the device
 * has been properly initialized. It configures the ADC channel by
 * writing to the ADC configuration register. If the operation fails, the
 * function returns an error code. Ensure that the device pointer is
 * valid and that the channel and adc_ip_sel parameters are within their
 * respective valid ranges.
 *
 * @param dev A pointer to an initialized ad5758_dev structure. Must not be
 * null. The caller retains ownership.
 * @param channel The ADC channel to configure. Must be a valid channel number
 * as per the device's specifications.
 * @param adc_ip_sel An enum value of type ad5758_adc_ip representing the ADC
 * input selection. Must be a valid selection as defined in
 * the ad5758_adc_ip enumeration.
 * @return Returns 0 on success or -1 on failure, indicating an error occurred
 * during the SPI register write operation.
 ******************************************************************************/
int32_t ad5758_set_adc_channel_input(struct ad5758_dev *dev,
				     uint8_t channel,
				     enum ad5758_adc_ip adc_ip_sel);
/***************************************************************************//**
 * @brief This function sets the mode of the ADC on the AD5758 device and
 * enables or disables it based on the provided parameters. It should be
 * called when you need to configure the ADC for specific operations,
 * such as switching between different modes or enabling/disabling the
 * ADC. Ensure that the device is properly initialized before calling
 * this function. The function communicates with the device over SPI to
 * apply the configuration. If the operation fails, it returns an error
 * code.
 *
 * @param dev A pointer to an initialized ad5758_dev structure representing the
 * device. Must not be null.
 * @param adc_mode An enum value of type ad5758_adc_mode specifying the desired
 * ADC mode. Valid modes are defined in the ad5758_adc_mode
 * enumeration.
 * @param enable A uint8_t value where non-zero enables the ADC and zero
 * disables it. Any non-zero value is treated as enable.
 * @return Returns 0 on success or -1 on failure, indicating an error in writing
 * to the device.
 ******************************************************************************/
int32_t ad5758_set_adc_mode(struct ad5758_dev *dev,
			    enum ad5758_adc_mode adc_mode,
			    uint8_t enable);
/***************************************************************************//**
 * @brief This function initializes the AD5758 device by setting up the
 * necessary SPI and GPIO interfaces, configuring device settings, and
 * enabling various features such as the DAC output and slew rate
 * control. It must be called before any other operations on the AD5758
 * device. The function allocates memory for the device structure and
 * performs a series of configuration steps, including a software reset
 * and calibration memory refresh. If any step fails, the function will
 * clean up and return an error code. Successful initialization is
 * indicated by a return value of 0.
 *
 * @param device A pointer to a pointer of type `struct ad5758_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad5758_init_param` containing
 * initialization parameters such as SPI and GPIO
 * configurations, and device settings. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered.
 ******************************************************************************/
int32_t ad5758_init(struct ad5758_dev **device,
		    struct ad5758_init_param *init_param);
#endif /* AD5758_H_ */
