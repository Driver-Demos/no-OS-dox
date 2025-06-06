/***************************************************************************//**
 *   @file   ad5460.h
 *   @brief  Header file of AD5460 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef _AD5460_H
#define _AD5460_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "stdint.h"
#include "stdbool.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define AD5460_NOP				0x00
#define AD5460_CH_FUNC_SETUP(x)               (0x01 + (x * 6))
#define AD5460_OUTPUT_CONFIG(x)               (0x02 + (x * 6))
#define AD5460_DAC_SLEW_CONFIG(x)		(0x03 + (x * 6))
#define AD5460_DAC_CODE(x)			(0x04 + (x * 6))
#define AD5460_DAC_CLR_CODE(x)                (0x05 + (x * 6))
#define AD5460_DAC_ACTIVE(x)			(0x06 + (x * 6))
#define AD5460_GPIO_CONFIG(x)			(0x19 + x)
#define AD5460_WTD_CONFIG			0x1D
#define AD5460_DIAG_CONFIG			0x1E
#define AD5460_PWR_CONFIG			0x1F
#define AD5460_LIVE_STATUS			0x20
#define AD5460_DIGITAL_ALERT_STATUS		0x21
#define AD5460_ANALOG_ALERT_STATUS		0x22
#define AD5460_DIGITAL_ALERT_MSK		0x23
#define AD5460_ANALOG_ALERT_MSK		0x24
#define AD5460_READ_SELECT			0x73
#define AD5460_THERM_RST			0x74
#define AD5460_CMD_KEY			0x75
#define AD5460_BROADCAST_CMD_KEY		0x76
#define AD5460_SCRATCH(x)			(0x77 + x)
#define AD5460_GENERIC_ID			0x7B
#define AD5460_SILICON_REV			0x7C
#define AD5460_SILICON_ID0			0x7D
#define AD5460_SILICON_ID1			0x7E
#define AD5460_SILICON_ID2			0x7F

/** Software reset sequence */
#define AD5460_CMD_KEY_RESET_1                0x9A17
#define AD5460_CMD_KEY_RESET_2                0xE9C1

/* AD5460_CH_FUNC_SETUP */
#define AD5460_CH_FUNC_SETUP_MSK		NO_OS_GENMASK(3, 0)

/** OUTPUT_CONFIGx Register */
#define AD5460_AVDD_SELECT_MSK		NO_OS_GENMASK(13, 12)
#define AD5460_CLR_EN_MSK			NO_OS_BIT(11)
#define AD5460_WAIT_LDAC_CMD_MSK		NO_OS_BIT(10)
#define AD5460_IOUT_RANGE_MSK			NO_OS_GENMASK(9, 8)
#define AD5460_I_LIMIT_MSK			NO_OS_BIT(7)
#define AD5460_ALARM_DEG_PERIOD_MSK		NO_OS_BIT(6)
#define AD5460_VOUT_RANGE_MSK			NO_OS_BIT(5)
#define AD5460_VIOUT_DRV_EN_DLY_MSK		NO_OS_GENMASK(4, 3)
#define AD5460_VOUT_4W_EN_MSK			NO_OS_BIT(2)
#define AD5460_VIOUT_GLITCH_MSK		NO_OS_GENMASK(1, 0)

/** DAC_SLEW_CONFIG Register */
#define AD5460_SLEW_EN_MSK			NO_OS_GENMASK(5, 4)
#define AD5460_SLEW_LIN_STEP_MSK		NO_OS_GENMASK(3, 2)
#define AD5460_SLEW_LIN_RATE_MSK		NO_OS_GENMASK(1, 0)

/** DAC_CODE Register*/
#define AD5460_DAC_CODE_MSK			NO_OS_GENMASK(15, 0)

/** DAC_CLR_CODE Register*/
#define AD5460_DAC_CLR_CODE_MSK		NO_OS_GENMASK(15, 0)

/** DAC_ACTIVE Register*/
#define AD5460_DAC_ACTIVE_MSK			NO_OS_GENMASK(15, 0)

/** GPIO_CONFIG Register */
#define AD5460_GPI_DATA_MSK			NO_OS_BIT(5)
#define AD5460_GPO_DATA_MSK			NO_OS_BIT(4)
#define AD5460_GP_WK_PD_EN_MSK		NO_OS_BIT(3)
#define AD5460_GPIO_SELECT_MSK		NO_OS_GENMASK(2, 0)

/** DIAG_SELECT Register */
#define AD5460_DIAG_SELECT_MSK		NO_OS_GENMASK(4, 0)

/** PWR_CONFIG Register */
#define AD5460_REF_EN_MSK			NO_OS_BIT(0)

/** LIVE_STATUS Register */
#define AD5460_TEMP_ALERT_STATUS_MSK		NO_OS_BIT(9)
#define AD5460_SUPPLY_STATUS_MSK		NO_OS_BIT(8)
#define AD5460_ANALOG_IO_SC_STATUS_D_MSK	NO_OS_BIT(7)
#define AD5460_ANALOG_IO_SC_STATUS_C_MSK	NO_OS_BIT(6)
#define AD5460_ANALOG_IO_SC_STATUS_B_MSK	NO_OS_BIT(5)
#define AD5460_ANALOG_IO_SC_STATUS_A_MSK	NO_OS_BIT(4)
#define AD5460_ANALOG_IO_OC_STATUS_D_MSK	NO_OS_BIT(3)
#define AD5460_ANALOG_IO_OC_STATUS_C_MSK	NO_OS_BIT(2)
#define AD5460_ANALOG_IO_OC_STATUS_B_MSK	NO_OS_BIT(1)
#define AD5460_ANALOG_IO_OC_STATUS_A_MSK	NO_OS_BIT(0)

/** DIGITAL_ALERT_STATUS Register */
#define AD5460_TEMP_ALERT_MSK			NO_OS_BIT(4)
#define AD5460_CAL_MEM_ERR_MSK		NO_OS_BIT(3)
#define AD5460_SPI_ERR_MSK			NO_OS_BIT(2)
#define AD5460_WDT_RESET_OCCURRED_MSK		NO_OS_BIT(1)
#define AD5460_RESET_OCCURRED_MSK		NO_OS_BIT(0)

/* ANALOG_ALERT_STATUS Register */
#define AD5460_AVDD_HI_ERR_MSK			NO_OS_BIT(12)
#define AD5460_AVDD_LO_ERR_MSK			NO_OS_BIT(11)
#define AD5460_AVCC_ERR_MSK			NO_OS_BIT(10)
#define AD5460_IODVCC_ERR_MSK			NO_OS_BIT(9)
#define AD5460_AVSS_ERR_MSK			NO_OS_BIT(8)
#define AD5460_ANALOG_IO_SC_D_MSK		NO_OS_BIT(7)
#define AD5460_ANALOG_IO_SC_C_MSK		NO_OS_BIT(6)
#define AD5460_ANALOG_IO_SC_B_MSK		NO_OS_BIT(5)
#define AD5460_ANALOG_IO_SC_A_MSK		NO_OS_BIT(4)
#define AD5460_ANALOG_IO_OC_D_MSK		NO_OS_BIT(3)
#define AD5460_ANALOG_IO_OC_C_MSK		NO_OS_BIT(2)
#define AD5460_ANALOG_IO_OC_B_MSK		NO_OS_BIT(1)
#define AD5460_ANALOG_IO_OC_A_MSK		NO_OS_BIT(0)

/* AD5460 READ_SELECT Register */
#define AD5460_READBACK_SELECT_MODE_MSK	NO_OS_GENMASK(8, 0)
#define AD5460_READBACK_ADDR_MSK		NO_OS_GENMASK(8, 0)

/* THERM_RST Register */
#define AD5460_THERM_RST_MSK			NO_OS_GENMASK(8, 0)

/* CMD_KEY Register */
#define AD5460_CMD_KEY_MSK			NO_OS_GENMASK(15, 0)

/* BROADCAST_CMD_KEY Register */
#define AD5460_BROADCAST_CMD_KEY_MSK		NO_OS_GENMASK(15, 0)

/* SCRATCH Register */
#define AD5460_SCRATCH_BITS_MSK		NO_OS_GENMASK(15, 0)

/* GENERIC_ID Register */
#define AD5460_GENERIC_ID_MSK			NO_OS_GENMASK(2, 0)

/* SILICON_REV Register */
#define AD5460_SILICON_REV_ID_MSK		NO_OS_GENMASK(7, 0)

/* SILICON_ID0 Register */
#define AD5460_LOT_MSK			NO_OS_GENMASK(6, 0)

/* SILICON_ID1 Register */
#define AD5460_Y_COORDS_MSK			NO_OS_GENMASK(13, 7)
#define AD5460_X_COORDS_MSK			NO_OS_GENMASK(6, 0)

/* SILICON_ID2 Register */
#define AD5460_WAFER_NUM_MSK			NO_OS_GENMASK(4, 0)

#define AD5460_FRAME_SIZE			5
#define AD5460_N_CHANNELS			4
#define AD5460_CH_A				0
#define AD5460_CH_B				1
#define AD5460_CH_C				2
#define AD5460_CH_D				3
#define AD5460_DAC_RANGE			12000
#define AD5460_DAC_CURRENT_RANGE_25MA		25000
#define AD5460_DAC_CURRENT_RANGE_20MA		20000
#define AD5460_DAC_CURRENT_RANGE_4MA		4000
#define AD5460_DAC_RESOLUTION			16

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad5460_op_mode` enumeration defines the various operational modes
 * available for the AD5460 device, which include high impedance, voltage
 * output, current output, and current output with HART communication.
 * Each mode is associated with a specific hexadecimal value that is used
 * to configure the device's operation.
 *
 * @param AD5460_HIGH_Z Represents a high impedance state with a value of 0x0.
 * @param AD5460_VOLTAGE_OUT Represents a voltage output mode with a value of
 * 0x1.
 * @param AD5460_CURRENT_OUT Represents a current output mode with a value of
 * 0x2.
 * @param AD5460_CURRENT_OUT_HART Represents a current output mode with HART
 * communication with a value of 0x10.
 ******************************************************************************/
enum ad5460_op_mode {
	AD5460_HIGH_Z = 0x0,
	AD5460_VOLTAGE_OUT = 0x1,
	AD5460_CURRENT_OUT = 0x2,
	AD5460_CURRENT_OUT_HART = 0x10,

};

/***************************************************************************//**
 * @brief The `ad5460_vout_range` enumeration defines the possible voltage
 * output ranges for the AD5460 digital-to-analog converter (DAC). It
 * provides two options: a unipolar range from 0 to 12 volts and a
 * bipolar range from -12 to 12 volts, allowing the DAC to be configured
 * for different voltage output requirements.
 *
 * @param AD5460_VOUT_RANGE_0_12V Represents a voltage output range from 0 to 12
 * volts.
 * @param AD5460_VOUT_RANGE_NEG12_12V Represents a voltage output range from -12
 * to 12 volts.
 ******************************************************************************/
enum ad5460_vout_range {
	AD5460_VOUT_RANGE_0_12V,
	AD5460_VOUT_RANGE_NEG12_12V,
};

/***************************************************************************//**
 * @brief The `ad5460_iout_range` enumeration defines the possible current
 * output ranges for the AD5460 DAC device. It provides three distinct
 * current ranges that can be selected for the device's output, allowing
 * for flexibility in configuring the DAC to meet specific application
 * requirements. This enumeration is used to set the current output range
 * for a specific channel of the AD5460 device.
 *
 * @param AD5460_IOUT_RANGE_0_25MA Represents a current output range of 0 to 25
 * mA.
 * @param AD5460_IOUT_RANGE_0_20MA Represents a current output range of 0 to 20
 * mA.
 * @param AD5460_IOUT_RANGE_4_20MA Represents a current output range of 4 to 20
 * mA.
 ******************************************************************************/
enum ad5460_iout_range {
	AD5460_IOUT_RANGE_0_25MA,
	AD5460_IOUT_RANGE_0_20MA,
	AD5460_IOUT_RANGE_4_20MA,
};

/***************************************************************************//**
 * @brief The `ad5460_i_limit` enumeration defines the possible current limit
 * settings for the AD5460 DAC when operating in voltage output mode. It
 * provides two distinct current limit options, `AD5460_I_LIMIT0` and
 * `AD5460_I_LIMIT1`, which can be used to configure the device's current
 * limiting behavior to suit specific application requirements.
 *
 * @param AD5460_I_LIMIT0 Represents the first current limit setting for the DAC
 * in Vout mode.
 * @param AD5460_I_LIMIT1 Represents the second current limit setting for the
 * DAC in Vout mode.
 ******************************************************************************/
enum ad5460_i_limit {
	AD5460_I_LIMIT0,
	AD5460_I_LIMIT1,
};

/***************************************************************************//**
 * @brief The `ad5460_slew_lin_step` enumeration defines the possible linear
 * step sizes for the Digital-to-Analog Converter (DAC) slew rate control
 * in the AD5460 device. Each enumerator corresponds to a specific
 * percentage of the full-scale voltage, allowing for precise control
 * over the rate at which the DAC output changes. This is particularly
 * useful in applications requiring smooth transitions in output voltage
 * or current.
 *
 * @param AD5460_STEP_0_8_PERCENT Represents a linear step size of 0.8% for the
 * DAC slew rate.
 * @param AD5460_STEP_1_5_PERCENT Represents a linear step size of 1.5% for the
 * DAC slew rate.
 * @param AD5460_STEP_6_1_PERCENT Represents a linear step size of 6.1% for the
 * DAC slew rate.
 * @param AD5460_STEP_22_2_PERCENT Represents a linear step size of 22.2% for
 * the DAC slew rate.
 ******************************************************************************/
enum ad5460_slew_lin_step {
	AD5460_STEP_0_8_PERCENT,
	AD5460_STEP_1_5_PERCENT,
	AD5460_STEP_6_1_PERCENT,
	AD5460_STEP_22_2_PERCENT,
};

/***************************************************************************//**
 * @brief The `ad5460_lin_rate` enumeration defines the possible linear update
 * rates for the AD5460 DAC when slew control is enabled. Each enumerator
 * corresponds to a specific frequency at which the DAC can update its
 * output, allowing for precise control over the rate of change in the
 * output signal. This is particularly useful in applications requiring
 * smooth transitions or specific timing constraints in signal
 * generation.
 *
 * @param AD5460_LIN_RATE_4KHZ8 Represents a linear update rate of 4.8 kHz for
 * the DAC.
 * @param AD5460_LIN_RATE_76KHZ8 Represents a linear update rate of 76.8 kHz for
 * the DAC.
 * @param AD5460_LIN_RATE_153KHZ6 Represents a linear update rate of 153.6 kHz
 * for the DAC.
 * @param AD5460_LIN_RATE_230KHZ4 Represents a linear update rate of 230.4 kHz
 * for the DAC.
 ******************************************************************************/
enum ad5460_lin_rate {
	AD5460_LIN_RATE_4KHZ8,
	AD5460_LIN_RATE_76KHZ8,
	AD5460_LIN_RATE_153KHZ6,
	AD5460_LIN_RATE_230KHZ4,
};

/***************************************************************************//**
 * @brief The `ad5460_gpio_select` enumeration defines the possible
 * configurations for the GPIO pins on the AD5460 device, allowing them
 * to be set to high impedance, general-purpose input/output, or general-
 * purpose input modes. This enumeration is used to control the behavior
 * of the GPIO pins, which can be crucial for interfacing with other
 * components or systems.
 *
 * @param AD5460_GPIO_SEL_HIGH_Z Represents a high impedance state for the GPIO.
 * @param AD5460_GPIO_SEL_GPIO Configures the pin as a general-purpose
 * input/output.
 * @param AD5460_GPIO_SEL_GPI Configures the pin as a general-purpose input.
 ******************************************************************************/
enum ad5460_gpio_select {
	AD5460_GPIO_SEL_HIGH_Z,
	AD5460_GPIO_SEL_GPIO,
	AD5460_GPIO_SEL_GPI,
};

/***************************************************************************//**
 * @brief The `ad5460_diag_mode` enumeration defines various diagnostic modes
 * for the AD5460 device, each corresponding to a specific diagnostic
 * function or sensor reading. These modes allow the device to perform
 * self-checks and monitor different internal and external parameters,
 * such as temperature, reference voltages, and power supply levels, to
 * ensure proper operation and facilitate troubleshooting.
 *
 * @param AD5460_DIAG_NO_DIAG Represents no diagnostic mode.
 * @param AD5460_DIAG_PTAT Represents the PTAT (Proportional To Absolute
 * Temperature) diagnostic mode.
 * @param AD5460_DIAG_REFIN Represents the reference input diagnostic mode.
 * @param AD5460_DIAG_REFOUT Represents the reference output diagnostic mode.
 * @param AD5460_DIAG_INT_BG_V Represents the internal bandgap voltage
 * diagnostic mode.
 * @param AD5460_DIAG_SENSE_P_CH_A Represents the positive sense diagnostic mode
 * for channel A.
 * @param AD5460_DIAG_SENSE_CH_B Represents the sense diagnostic mode for
 * channel B.
 * @param AD5460_DIAG_SENSE_P_CH_C Represents the positive sense diagnostic mode
 * for channel C.
 * @param AD5460_DIAG_SENSE_P_CH_D Represents the positive sense diagnostic mode
 * for channel D.
 * @param AD5460_DIAG_SENSE_N_CH_A Represents the negative sense diagnostic mode
 * for channel A.
 * @param AD5460_DIAG_SENSE_N_CH_B Represents the negative sense diagnostic mode
 * for channel B.
 * @param AD5460_DIAG_SENSE_N_CH_C Represents the negative sense diagnostic mode
 * for channel C.
 * @param AD5460_DIAG_SENSE_N_CH_D Represents the negative sense diagnostic mode
 * for channel D.
 * @param AD5460_DIAG_SENSE_AVCC Represents the AVCC sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_LDO Represents the LDO sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_DVCC Represents the DVCC sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_AGND1 Represents the AGND1 sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_AGND2 Represents the AGND2 sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_AGND3 Represents the AGND3 sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_DGND Represents the DGND sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_AVDD_HI Represents the high AVDD sense diagnostic
 * mode.
 * @param AD5460_DIAG_SENSE_AVDD_LO Represents the low AVDD sense diagnostic
 * mode.
 * @param AD5460_DIAG_SENSE_AVSS Represents the AVSS sense diagnostic mode.
 * @param AD5460_DIAG_SENSE_AVDD_CH_A Represents the AVDD sense diagnostic mode
 * for channel A.
 * @param AD5460_DIAG_SENSE_AVDD_CH_B Represents the AVDD sense diagnostic mode
 * for channel B.
 * @param AD5460_DIAG_SENSE_AVDD_CH_C Represents the AVDD sense diagnostic mode
 * for channel C.
 * @param AD5460_DIAG_SENSE_AVDD_CH_D Represents the AVDD sense diagnostic mode
 * for channel D.
 ******************************************************************************/
enum ad5460_diag_mode {
	AD5460_DIAG_NO_DIAG,
	AD5460_DIAG_PTAT,
	AD5460_DIAG_REFIN,
	AD5460_DIAG_REFOUT,
	AD5460_DIAG_INT_BG_V,
	AD5460_DIAG_SENSE_P_CH_A,
	AD5460_DIAG_SENSE_CH_B,
	AD5460_DIAG_SENSE_P_CH_C,
	AD5460_DIAG_SENSE_P_CH_D,
	AD5460_DIAG_SENSE_N_CH_A,
	AD5460_DIAG_SENSE_N_CH_B,
	AD5460_DIAG_SENSE_N_CH_C,
	AD5460_DIAG_SENSE_N_CH_D,
	AD5460_DIAG_SENSE_AVCC,
	AD5460_DIAG_SENSE_LDO,
	AD5460_DIAG_SENSE_DVCC,
	AD5460_DIAG_SENSE_AGND1,
	AD5460_DIAG_SENSE_AGND2,
	AD5460_DIAG_SENSE_AGND3,
	AD5460_DIAG_SENSE_DGND,
	AD5460_DIAG_SENSE_AVDD_HI,
	AD5460_DIAG_SENSE_AVDD_LO,
	AD5460_DIAG_SENSE_AVSS,
	AD5460_DIAG_SENSE_AVDD_CH_A,
	AD5460_DIAG_SENSE_AVDD_CH_B,
	AD5460_DIAG_SENSE_AVDD_CH_C,
	AD5460_DIAG_SENSE_AVDD_CH_D,
};

/***************************************************************************//**
 * @brief The `_ad5460_live_status` structure is a bitfield representation used
 * to monitor the live status of various operational parameters of the
 * AD5460 device. It includes flags for overcurrent and short-circuit
 * conditions on four analog I/O channels, as well as indicators for
 * power supply status and temperature alerts. Each field is a single
 * bit, reflecting the real-time status of the corresponding condition,
 * which is crucial for diagnostics and ensuring the device operates
 * within safe parameters.
 *
 * @param ANALOG_IO_OC_STATUS_A Indicates overcurrent status for analog I/O
 * channel A.
 * @param ANALOG_IO_OC_STATUS_B Indicates overcurrent status for analog I/O
 * channel B.
 * @param ANALOG_IO_OC_STATUS_C Indicates overcurrent status for analog I/O
 * channel C.
 * @param ANALOG_IO_OC_STATUS_D Indicates overcurrent status for analog I/O
 * channel D.
 * @param ANALOG_IO_SC_STATUS_A Indicates short-circuit status for analog I/O
 * channel A.
 * @param ANALOG_IO_SC_STATUS_B Indicates short-circuit status for analog I/O
 * channel B.
 * @param ANALOG_IO_SC_STATUS_C Indicates short-circuit status for analog I/O
 * channel C.
 * @param ANALOG_IO_SC_STATUS_D Indicates short-circuit status for analog I/O
 * channel D.
 * @param SUPPLY_STATUS Indicates the status of the power supply.
 * @param TEMP_ALERT_STATUS Indicates if there is a temperature alert.
 ******************************************************************************/
struct _ad5460_live_status {
	uint8_t ANALOG_IO_OC_STATUS_A: 1;
	uint8_t ANALOG_IO_OC_STATUS_B: 1;
	uint8_t ANALOG_IO_OC_STATUS_C: 1;
	uint8_t ANALOG_IO_OC_STATUS_D: 1;
	uint8_t ANALOG_IO_SC_STATUS_A: 1;
	uint8_t ANALOG_IO_SC_STATUS_B: 1;
	uint8_t ANALOG_IO_SC_STATUS_C: 1;
	uint8_t ANALOG_IO_SC_STATUS_D: 1;
	uint8_t SUPPLY_STATUS: 1;
	uint8_t TEMP_ALERT_STATUS: 1;
};

/***************************************************************************//**
 * @brief The `ad5460_live_status` union is designed to store the live status of
 * the AD5460 device, providing a flexible way to access status
 * information. It contains a struct, `_ad5460_live_status`, which breaks
 * down the status into individual bit fields for specific conditions
 * such as overcurrent and short-circuit statuses for analog I/O, supply
 * status, and temperature alert status. Alternatively, the entire status
 * can be accessed as a single 16-bit value, allowing for efficient
 * reading and writing of the status register.
 *
 * @param status_bits A struct containing individual status bit fields for
 * various analog and supply statuses.
 * @param value A 16-bit unsigned integer representing the combined status bits
 * as a single value.
 ******************************************************************************/
union ad5460_live_status {
	struct _ad5460_live_status status_bits;
	uint16_t value;
};

/***************************************************************************//**
 * @brief The `ad5460_init_param` structure is used to initialize the AD5460
 * device, containing essential parameters such as the device address,
 * SPI initialization parameters, and GPIO reset parameters. This
 * structure is crucial for setting up the communication and control
 * interfaces required for the AD5460 device operation.
 *
 * @param dev_addr A uint8_t representing the device address.
 * @param spi_ip A struct of type no_os_spi_init_param for SPI initialization
 * parameters.
 * @param reset_gpio_param A pointer to a struct of type no_os_gpio_init_param
 * for GPIO reset parameters.
 ******************************************************************************/
struct ad5460_init_param {
	uint8_t dev_addr;
	struct no_os_spi_init_param spi_ip;
	struct no_os_gpio_init_param *reset_gpio_param;
};

/**
 * @brief Device channel state.
 */

/***************************************************************************//**
 * @brief The `ad5460_channel_config` structure is used to configure individual
 * channels of the AD5460 device, specifying their operational parameters
 * such as whether the channel is enabled, its function mode, voltage and
 * current output ranges, and current limit settings. This configuration
 * is crucial for tailoring the device's behavior to specific application
 * requirements, ensuring that each channel operates within the desired
 * electrical parameters.
 *
 * @param enabled Indicates whether the channel is enabled or not.
 * @param function Specifies the operation mode of the channel using the
 * ad5460_op_mode enum.
 * @param vout_range Defines the voltage output range for the channel using the
 * ad5460_vout_range enum.
 * @param iout_range Specifies the current output range for the channel using
 * the ad5460_iout_range enum.
 * @param i_limit Sets the current limit for the channel in voltage output mode
 * using the ad5460_i_limit enum.
 ******************************************************************************/
struct ad5460_channel_config {
	bool enabled;
	enum ad5460_op_mode function;
	enum ad5460_vout_range vout_range;
	enum ad5460_iout_range iout_range;
	enum ad5460_i_limit i_limit;
};

/***************************************************************************//**
 * @brief The `ad5460_desc` structure is a descriptor for the AD5460 device,
 * encapsulating all necessary information for its operation and
 * communication. It includes the device address, SPI communication
 * descriptor, a buffer for communication frames, configurations for each
 * channel, and a GPIO descriptor for the reset functionality. This
 * structure is essential for managing the device's state and interfacing
 * with its hardware components.
 *
 * @param dev_addr Stores the device address as an 8-bit unsigned integer.
 * @param spi_desc Pointer to a SPI descriptor structure for SPI communication.
 * @param comm_buff Array buffer for communication frames, sized by
 * AD5460_FRAME_SIZE.
 * @param channel_configs Array of channel configuration structures, one for
 * each channel defined by AD5460_N_CHANNELS.
 * @param reset_gpio Pointer to a GPIO descriptor structure for the reset pin.
 ******************************************************************************/
struct ad5460_desc {
	uint8_t dev_addr;
	struct no_os_spi_desc *spi_desc;
	uint8_t comm_buff[AD5460_FRAME_SIZE];
	struct ad5460_channel_config channel_configs[AD5460_N_CHANNELS];
	struct no_os_gpio_desc *reset_gpio;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function converts a specified millivolt value into a
 * corresponding DAC code for a given channel of the AD5460 device. It
 * should be used when you need to translate a voltage level into a
 * digital code that the DAC can use to produce the desired output
 * voltage. The function requires a valid device descriptor and channel
 * index, and it checks that the millivolt value is within the acceptable
 * range for the channel's configured voltage output range. If the input
 * voltage is out of range or the channel configuration is invalid, the
 * function returns an error code.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param mvolts The millivolt value to convert. Must be within the range
 * supported by the channel's voltage output configuration;
 * otherwise, an error is returned.
 * @param code A pointer to a uint16_t where the resulting DAC code will be
 * stored. Must not be null.
 * @param ch The channel index for which the conversion is to be performed. Must
 * be a valid channel index as per the device's configuration.
 * @return Returns 0 on success, or a negative error code if the input is
 * invalid or out of range.
 ******************************************************************************/
int ad5460_dac_voltage_to_code(struct ad5460_desc *, int32_t,
			       uint16_t *, uint32_t);

/***************************************************************************//**
 * @brief This function converts a given current value in microamps to a
 * corresponding 16-bit DAC code for a specified channel of the AD5460
 * device. It should be used when you need to set the DAC output current
 * based on a desired microamp value. The function requires a valid
 * device descriptor and channel index, and it checks if the provided
 * current value is within the allowable range for the channel's
 * configured output range. If the current value is out of range or the
 * channel configuration is invalid, the function returns an error code.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before calling this function.
 * @param uamps The desired current value in microamps. Must be within the valid
 * range for the specified channel's output configuration;
 * otherwise, an error is returned.
 * @param code A pointer to a uint16_t where the resulting DAC code will be
 * stored. Must not be null.
 * @param ch The index of the channel for which the conversion is to be
 * performed. Must be a valid channel index as per the device's
 * configuration.
 * @return Returns 0 on success, or a negative error code if the input current
 * is out of range or if the channel configuration is invalid.
 ******************************************************************************/
int ad5460_dac_current_to_code(struct ad5460_desc *, uint32_t, uint16_t *,
			       uint32_t);

/***************************************************************************//**
 * @brief This function writes a 16-bit value to a specified register address on
 * the AD5460 device. It is typically used to configure the device or
 * update its settings. The function requires a valid device descriptor,
 * a register address, and the value to be written. It must be called
 * after the device has been properly initialized. The function
 * communicates with the device over SPI and returns an error code if the
 * write operation fails.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function. The caller retains ownership.
 * @param addr A 32-bit unsigned integer representing the register address to
 * which the value will be written. The address must be valid for
 * the AD5460 device.
 * @param val A 16-bit unsigned integer representing the value to be written to
 * the specified register. The value should be within the valid range
 * for the target register.
 * @return Returns an integer status code. A non-zero value indicates an error
 * during the SPI write operation.
 ******************************************************************************/
int ad5460_reg_write(struct ad5460_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function reads a raw communication frame from a specified
 * register address on the AD5460 device. It is typically used when low-
 * level access to the device's registers is required. The function must
 * be called with a valid device descriptor and a non-null pointer for
 * the output buffer. The function performs a two-step SPI communication
 * process to retrieve the register value. It is important to ensure that
 * the device is properly initialized before calling this function. The
 * function returns an error code if the SPI communication fails.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param addr The address of the register to read from. Must be a valid
 * register address for the AD5460 device.
 * @param val A pointer to a buffer where the read value will be stored. Must
 * not be null. The buffer should be at least AD5460_FRAME_SIZE bytes
 * in size.
 * @return Returns 0 on success or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int ad5460_reg_read_raw(struct ad5460_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to read a 16-bit value from a specific register of
 * the AD5460 device. It is essential to ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function performs a CRC check to validate the integrity of the data
 * read from the device. If the CRC check fails, the function returns an
 * error code. This function is useful for retrieving configuration or
 * status information from the device.
 *
 * @param desc A pointer to an initialized `ad5460_desc` structure representing
 * the device. Must not be null.
 * @param addr The 32-bit address of the register to read from. Must be a valid
 * register address for the AD5460 device.
 * @param val A pointer to a 16-bit variable where the read value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or if the CRC check does not match.
 ******************************************************************************/
int ad5460_reg_read(struct ad5460_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function is used to modify a specific field within a register of
 * the AD5460 device. It reads the current value of the register, applies
 * a mask to clear the bits of interest, and then sets these bits to the
 * new value provided. This function should be called when you need to
 * update specific settings or configurations in the device's registers
 * without affecting other bits. Ensure that the device descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the AD5460 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Only these bits will be modified.
 * @param val The new value to be written to the bits specified by the mask. The
 * value is prepared and applied to the register using the mask.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as if the register read or write fails.
 ******************************************************************************/
int ad5460_reg_update(struct ad5460_desc *, uint32_t, uint16_t,
		      uint16_t);

/***************************************************************************//**
 * @brief This function configures the operation mode of a specified channel on
 * the AD5460 device. It should be called when you need to change the
 * channel's function, such as switching between high impedance, voltage
 * output, or current output modes. The function requires a delay to
 * ensure proper transition between modes, which is handled internally.
 * It is important to ensure that the device descriptor is properly
 * initialized before calling this function. The function returns an
 * error code if the operation fails, allowing the caller to handle such
 * cases appropriately.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null, and the device must be properly
 * initialized before use.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D. Invalid channel numbers may result in undefined
 * behavior.
 * @param ch_func The desired operation mode for the channel, specified as an
 * enum ad5460_op_mode value. Valid modes include AD5460_HIGH_Z,
 * AD5460_VOLTAGE_OUT, AD5460_CURRENT_OUT, and
 * AD5460_CURRENT_OUT_HART.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad5460_set_channel_function(struct ad5460_desc *,
				uint32_t, enum ad5460_op_mode);

/***************************************************************************//**
 * @brief This function configures the voltage output range for a specified
 * channel on the AD5460 device. It should be used when you need to
 * change the voltage output characteristics of a channel. Ensure that
 * the device descriptor is properly initialized before calling this
 * function. The function updates the device's configuration and returns
 * an error code if the operation fails.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to configure, typically ranging from 0 to
 * AD5460_N_CHANNELS - 1.
 * @param vout_range An enum value of type ad5460_vout_range specifying the
 * desired voltage output range for the channel.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad5460_set_channel_vout_range(struct ad5460_desc *, uint32_t,
				  enum ad5460_vout_range);

/***************************************************************************//**
 * @brief This function configures the current output range for a specific
 * channel on the AD5460 device. It should be used when you need to
 * adjust the current output characteristics of a channel to match
 * application requirements. The function must be called with a valid
 * device descriptor and a channel index within the supported range. It
 * updates the device's configuration and the internal descriptor state.
 * Ensure the device is properly initialized before calling this
 * function.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The index of the channel to configure. Must be within the range of
 * available channels (0 to AD5460_N_CHANNELS - 1).
 * @param iout_range The desired current output range, specified as a value from
 * the ad5460_iout_range enumeration. Determines the current
 * output characteristics for the channel.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad5460_set_channel_iout_range(struct ad5460_desc *, uint32_t,
				  enum ad5460_iout_range);

/***************************************************************************//**
 * @brief This function configures the current limit for a specified channel of
 * the AD5460 device when operating in voltage output mode. It should be
 * used to ensure that the current does not exceed a predefined limit,
 * which can help protect the device and connected components. The
 * function must be called with a valid device descriptor and channel
 * number. It updates the device's configuration and returns an error
 * code if the operation fails. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number to configure. Valid values are 0 to 3,
 * corresponding to channels A to D.
 * @param i_limit An enum value of type ad5460_i_limit specifying the current
 * limit to set. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad5460_set_channel_i_limit(struct ad5460_desc *, uint32_t,
			       enum ad5460_i_limit);

/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) code for a
 * specified channel on the AD5460 device. It is used to update the
 * output value of the DAC for a given channel, which must be specified
 * by the user. The function should be called when the user needs to
 * change the output voltage or current of a specific channel. It is
 * important to ensure that the device descriptor is properly initialized
 * before calling this function. The function returns an integer status
 * code indicating success or failure of the operation.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the DAC code is to be set. Valid
 * values are typically within the range of available channels for the
 * device.
 * @param dac_code The DAC code to be set for the specified channel. This is a
 * 16-bit value representing the desired output level.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad5460_set_channel_dac_code(struct ad5460_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief Use this function to configure the diagnostic mode for a specific
 * channel on the AD5460 device. This function is typically called when
 * you need to perform diagnostic operations on the device, such as
 * checking internal voltages or currents. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will return an error code if the operation fails.
 *
 * @param desc A pointer to an initialized `ad5460_desc` structure representing
 * the device. Must not be null.
 * @param ch The channel number to configure. Valid values are typically within
 * the range of available channels on the device.
 * @param diag_code An `ad5460_diag_mode` enumeration value specifying the
 * diagnostic mode to set. Must be a valid mode defined in the
 * `ad5460_diag_mode` enum.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad5460_set_diag(struct ad5460_desc *, uint32_t,
		    enum ad5460_diag_mode);

/***************************************************************************//**
 * @brief Use this function to obtain the current GPIO value for a specific
 * channel on the AD5460 device. It is essential to ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function reads the GPIO configuration register for the
 * specified channel and extracts the GPIO input data. This function is
 * useful for monitoring the state of GPIO pins configured as inputs. It
 * returns an error code if the read operation fails, otherwise it
 * returns 0 indicating success.
 *
 * @param desc A pointer to an initialized `ad5460_desc` structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the GPIO value is to be retrieved.
 * Valid values are typically within the range of available channels
 * for the device.
 * @param val A pointer to a uint8_t where the GPIO value will be stored. Must
 * not be null. The function writes the GPIO input value to this
 * location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ad5460_gpio_get(struct ad5460_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to set the GPIO operation mode for a specific
 * channel on the AD5460 device. This function is typically called when
 * initializing or reconfiguring the device's GPIO settings. Ensure that
 * the device descriptor is properly initialized before calling this
 * function. The function will return an error code if the operation
 * fails, which can be used for error handling.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to configure. Valid values are typically within
 * the range of available channels for the device.
 * @param config An enum value of type ad5460_gpio_select specifying the desired
 * GPIO operation mode. Valid values are defined by the
 * ad5460_gpio_select enumeration.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int ad5460_set_gpio_config(struct ad5460_desc *, uint32_t,
			   enum ad5460_gpio_select);

/***************************************************************************//**
 * @brief This function is used to set the logic value of a General Purpose
 * Output (GPO) pin on a specified channel of the AD5460 device. It
 * should be called when you need to control the output state of a GPO
 * pin. Before calling this function, ensure that the device has been
 * properly initialized and the channel is configured for GPIO operation.
 * The function first configures the channel for GPIO operation and then
 * updates the GPO pin with the specified logic value. If the
 * configuration step fails, the function will return an error code.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the GPO pin is to be set. Valid values
 * are typically within the range of available channels for the
 * device.
 * @param val The logic value to set on the GPO pin. Typically, 0 represents a
 * low state and 1 represents a high state.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad5460_gpio_set(struct ad5460_desc *, uint32_t, uint8_t);

/***************************************************************************//**
 * @brief Use this function to retrieve the current live status of the AD5460
 * device, which includes various status indicators such as temperature
 * alerts and supply status. This function should be called when you need
 * to monitor the device's operational status. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will populate the provided status union with the live status
 * data.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param status A pointer to an ad5460_live_status union where the live status
 * bits will be stored. Must not be null.
 * @return Returns an integer status code indicating success or failure of the
 * read operation.
 ******************************************************************************/
int ad5460_get_live(struct ad5460_desc *,
		    union ad5460_live_status *);

/***************************************************************************//**
 * @brief This function is used to configure and enable the slew rate control
 * for a DAC on a specified channel of the AD5460 device. It should be
 * called when you need to control the rate of change of the DAC output
 * to prevent abrupt changes that could affect the connected circuitry.
 * The function requires a valid device descriptor and channel number,
 * and it allows you to specify the linear step size and update rate for
 * the slew control. Ensure that the device is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null, and the device should be initialized.
 * @param ch The channel number for which to enable slew rate control. Valid
 * values are typically within the range of available channels for the
 * device.
 * @param step An enum value of type ad5460_slew_lin_step specifying the voltage
 * step size of the full-scale DAC voltage. Must be a valid enum
 * value.
 * @param rate An enum value of type ad5460_lin_rate specifying the update rate
 * for the DAC when slew control is enabled. Must be a valid enum
 * value.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad5460_dac_slew_enable(struct ad5460_desc *, uint32_t,
			   enum ad5460_slew_lin_step,
			   enum ad5460_lin_rate);

/***************************************************************************//**
 * @brief Use this function to disable the slew rate control for a specific
 * channel of the AD5460 DAC. This function is typically called when
 * precise control over the DAC output is required without the influence
 * of slew rate limiting. It is important to ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will return an error code if the operation fails, which
 * should be checked to ensure successful execution.
 *
 * @param desc A pointer to an initialized ad5460_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which to disable slew rate control. Valid
 * values are typically within the range of available channels for the
 * device.
 * @return Returns an integer status code: 0 for success or a negative error
 * code for failure.
 ******************************************************************************/
int ad5460_dac_slew_disable(struct ad5460_desc *, uint32_t);

/***************************************************************************//**
 * @brief This function is used to control the thermal reset feature of the
 * AD5460 device. It should be called when there is a need to enable or
 * disable the thermal reset functionality, which is typically used to
 * protect the device from overheating. The function requires a valid
 * device descriptor and a boolean value indicating whether to enable or
 * disable the feature. It returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param enable A boolean value where true enables the thermal reset feature
 * and false disables it.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int ad5460_set_therm_rst(struct ad5460_desc *, bool);

/***************************************************************************//**
 * @brief This function resets the AD5460 device, either by toggling a GPIO pin
 * if available or by sending a software reset command sequence. It
 * should be used to ensure the device is in a known state before
 * starting operations. The function requires a valid device descriptor
 * and assumes that the device has been properly initialized. If a reset
 * GPIO is configured, it will be used to perform a hardware reset;
 * otherwise, a software reset sequence is executed. The function waits
 * for the reset process to complete before returning.
 *
 * @param desc A pointer to an initialized ad5460_desc structure. This must not
 * be null and should represent a valid device instance. The
 * function will use this descriptor to determine the reset method
 * and perform the reset operation.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad5460_reset(struct ad5460_desc *);

/***************************************************************************//**
 * @brief This function sets up and initializes the AD5460 device descriptor
 * using the provided initialization parameters. It must be called before
 * any other operations on the AD5460 device. The function allocates
 * memory for the device descriptor, initializes the SPI interface, and
 * configures the device's reset GPIO if specified. It also performs a
 * device reset and a scratch test to ensure proper communication. If any
 * step fails, the function cleans up allocated resources and returns an
 * error code.
 *
 * @param desc A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller is responsible for
 * freeing the descriptor using `ad5460_remove`.
 * @param init_param A pointer to an `ad5460_init_param` structure containing
 * initialization parameters such as device address, SPI
 * initialization parameters, and optional reset GPIO
 * parameters. Must not be null. If null, the function returns
 * `-EINVAL`.
 * @return Returns 0 on success. On failure, returns a negative error code and
 * does not modify `*desc`.
 ******************************************************************************/
int ad5460_init(struct ad5460_desc **, struct ad5460_init_param *);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD5460 device descriptor when it is no longer needed. This includes
 * deallocating any associated GPIO and SPI resources. It is important to
 * ensure that the descriptor is not null before calling this function,
 * as passing a null descriptor will result in an error. This function
 * should be called to prevent resource leaks in applications that
 * utilize the AD5460 driver.
 *
 * @param desc A pointer to an ad5460_desc structure representing the device
 * descriptor to be removed. Must not be null. If the descriptor is
 * null, the function returns an error code.
 * @return Returns 0 on successful removal of the descriptor and associated
 * resources. Returns a negative error code if the descriptor is null or
 * if any resource deallocation fails.
 ******************************************************************************/
int ad5460_remove(struct ad5460_desc *desc);

#endif // _AD5460_H
