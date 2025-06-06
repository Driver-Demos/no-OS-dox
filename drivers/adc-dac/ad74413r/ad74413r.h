/***************************************************************************//**
 *   @file   ad74413r.h
 *   @brief  Header file of AD74413r Driver.
 *   @author Ciprian Regus (ciprian.regus@analog.com)
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
#ifndef _AD74413R_H
#define _AD74413R_H

#include "stdint.h"
#include "stdbool.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

#define AD74413R_N_CHANNELS             4
#define AD74413R_N_DIAG_CHANNELS	4

#define AD74413R_CH_A                   0
#define AD74413R_CH_B                   1
#define AD74413R_CH_C                   2
#define AD74413R_CH_D                   3

/** The value of the sense resistor in ohms */
#define AD74413R_RSENSE                 100
/** 16 bit ADC */
#define AD74413R_ADC_MAX_VALUE		65535

/** Register map */

#define AD74413R_NOP                            0x00
#define AD74413R_CH_FUNC_SETUP(x)               (0x01 + x)
#define AD74413R_ADC_CONFIG(x)                  (0x05 + x)
#define AD74413R_DIN_CONFIG(x)                  (0x09 + x)
#define AD74413R_GPO_PARALLEL                   0x0D
#define AD74413R_GPO_CONFIG(x)                  (0x0E + x)
#define AD74413R_OUTPUT_CONFIG(x)               (0x12 + x)
#define AD74413R_DAC_CODE(x)                    (0x16 + x)
#define AD74413R_DAC_CLR_CODE(x)                (0x1A + x)
#define AD74413R_DAC_ACTIVE(x)                  (0x1E + x)
#define AD74413R_DIN_THRESH                     0x22
#define AD74413R_ADC_CONV_CTRL                  0x23
#define AD74413R_DIAG_ASSIGN                    0x24
#define AD74413R_DIN_COMP_OUT                   0x25
#define AD74413R_ADC_RESULT(x)                  (0x26 + x)
#define AD74413R_DIAG_RESULT(x)                 (0x2A + x)
#define AD74413R_ALERT_STATUS                   0x2E
#define AD74413R_LIVE_STATUS                    0x2F
#define AD74413R_ALERT_MASK                     0x3C
#define AD74413R_DIN_COUNTER(x)                 (0x3D + x)
#define AD74413R_READ_SELECT                    0x41
#define AD74413R_THERM_RST                      0x43
#define AD74413R_CMD_KEY                        0x44
#define AD74413R_SCRATCH                        0x45
#define AD74413R_SILICON_REV                    0x46
#define AD74413R_ALERT_STATUS_RESET             NO_OS_GENMASK(15, 0)

/** Software reset sequence */
#define AD74413R_CMD_KEY_RESET_1                0x15FA
#define AD74413R_CMD_KEY_RESET_2                0xAF51

/** Command to load code into the DAC from DAC_CODEx */
#define AD74413R_CMD_KEY_LDAC	                0x953A

/** Command to load code into the DAC from DAC_CLR_CODEx */
#define AD74413R_CMD_KEY_DAC_CLEAR		0x73D1

#define AD74413R_SPI_RD_RET_INFO_MASK		NO_OS_BIT(8)
#define AD74413R_ERR_CLR_MASK			NO_OS_GENMASK(15, 0)
#define AD74413R_SPI_CRC_ERR_MASK		NO_OS_BIT(13)
#define AD74413R_CH_FUNC_SETUP_MASK             NO_OS_GENMASK(3, 0)
#define AD74413R_ADC_RANGE_MASK                 NO_OS_GENMASK(7, 5)
#define AD74413R_ADC_REJECTION_MASK             NO_OS_GENMASK(4, 3)
#define AD74413R_DEBOUNCE_TIME_MASK             NO_OS_GENMASK(4, 0)
#define AD74413R_DEBOUNCE_MODE_MASK             NO_OS_BIT(5)
#define AD74413R_DIAG_RESULT_MASK		NO_OS_GENMASK(15, 0)
#define AD74413R_REV_ID				NO_OS_GENMASK(7, 0)
#define AD74413R_CH_200K_TO_GND_MASK		NO_OS_BIT(2)

/** GPO_PARALLEL register */
#define AD74413R_GPO_PAR_DATA_MASK(x)		NO_OS_BIT(x)

/** GPO_CONFIGx register */
#define AD74413R_GPO_SELECT_MASK		NO_OS_GENMASK(2, 0)
#define AD74413R_GPO_DATA_MASK			NO_OS_BIT(3)

/** OUTPUT_CONFIGx register */
#define AD74413R_SLEW_EN_MASK			NO_OS_GENMASK(7, 6)
#define AD74413R_SLEW_LIN_STEP_MASK		NO_OS_GENMASK(5, 4)
#define AD74413R_SLEW_LIN_RATE_MASK		NO_OS_GENMASK(3, 2)
#define AD74413R_CLR_EN_MASK			NO_OS_BIT(1)
#define AD74413R_I_LIMIT_MASK			NO_OS_BIT(0)

/** DAC_CODEx register */
#define AD74413R_DAC_CODE_MASK			NO_OS_GENMASK(12, 0)

/** DAC_CLR_CODEx register */
#define AD74413R_CLR_CODE_MASK			NO_OS_GENMASK(12, 0)

/** DAC_ACTIVEx register */
#define AD74413R_DAC_ACTIVE_CODE_MASK		NO_OS_GENMASK(12, 0)

/** DIN_THRESH */
#define AD74413R_COMP_THRESH_MASK		NO_OS_GENMASK(5, 1)
#define AD74413R_DIN_THRESH_MODE_MASK		NO_OS_BIT(0)

/** DIN_COMP_OUT register */
#define AD74413R_DIN_COMP_CH(x)			NO_OS_BIT(x)

/** ADC_CONV_CTRL register */
#define AD74413R_EN_REJ_DIAG_MASK		NO_OS_BIT(10)
#define AD74413R_CONV_SEQ_MASK                  NO_OS_GENMASK(9, 8)
#define AD74413R_DIAG_EN_MASK(x)		(NO_OS_BIT(x) << 4)
#define AD74413R_CH_EN_MASK(x)                  NO_OS_BIT(x)

/** DIAG_ASSIGN register */
#define AD74413R_DIAG_ASSIGN_MASK(x)		(NO_OS_GENMASK(3, 0) << (x * 4))

/** The maximum voltage output of the DAC is 11V */
#define AD74413R_DAC_RANGE			11000
/** 13 bit DAC */
#define AD74413R_DAC_RESOLUTION			13
#define AD74413R_DAC_CODE_MAX			8191
#define AD74413R_ADC_RESOLUTION			16
#define AD74413R_ADC_CODE_MAX			65536

/** The number of possible DAC values */
#define AD74413R_THRESHOLD_DAC_RANGE		29
/** The comparator's value can be set betwen 0 - 16 V*/
#define AD74413R_THRESHOLD_RANGE		16000

#define AD74413R_TEMP_OFFSET			-2392
#define AD74413R_TEMP_SCALE			8950
#define AD74413R_TEMP_SCALE_DIV			1000

#define AD74413R_RANGE_10V_SCALE		15259ULL
#define AD74413R_RANGE_10V_SCALE_DIV		100000ULL
#define AD74413R_RANGE_2V5_SCALE		38147ULL
#define AD74413R_RANGE_2V5_SCALE_DIV		1000000ULL
#define AD74413R_RANGE_5V_SCALE			76294ULL
#define AD74413R_RANGE_5V_SCALE_DIV		1000000ULL
#define AD74413R_RANGE_5V_OFFSET		-(AD74413R_ADC_MAX_VALUE / 2)
#define AD74413R_RTD_PULL_UP			2100000ULL
#define AD74413R_SENSE_RESISTOR_OHMS		100

/***************************************************************************//**
 * @brief The `ad74413r_chip_id` enumeration defines identifiers for different
 * chip models supported by the AD74413R driver. It includes two possible
 * values, `AD74413R` and `AD74412R`, which are used to specify the
 * particular chip variant being interfaced with. This enumeration is
 * crucial for ensuring that the driver operates correctly with the
 * specific hardware model.
 *
 * @param AD74413R Represents the AD74413R chip identifier.
 * @param AD74412R Represents the AD74412R chip identifier.
 ******************************************************************************/
enum ad74413r_chip_id {
	AD74413R,
	AD74412R
};

/***************************************************************************//**
 * @brief The `ad74413r_rejection` enumeration defines various rejection
 * configurations for the AD74413R device, which are used to filter out
 * specific frequency interferences or to support HART communication.
 * These configurations are crucial for ensuring accurate signal
 * processing by minimizing the impact of unwanted frequency noise.
 *
 * @param AD74413R_REJECTION_50_60 Represents a rejection configuration for both
 * 50 Hz and 60 Hz frequencies.
 * @param AD74413R_REJECTION_NONE Indicates no rejection configuration is
 * applied.
 * @param AD74413R_REJECTION_50_60_HART Represents a rejection configuration for
 * 50 Hz and 60 Hz frequencies with HART
 * communication.
 * @param AD74413R_REJECTION_HART Indicates a rejection configuration
 * specifically for HART communication.
 ******************************************************************************/
enum ad74413r_rejection {
	AD74413R_REJECTION_50_60,
	AD74413R_REJECTION_NONE,
	AD74413R_REJECTION_50_60_HART,
	AD74413R_REJECTION_HART
};

/***************************************************************************//**
 * @brief The `ad74413r_op_mode` enumeration defines the various operational
 * modes available for the AD74413R device, which is a versatile analog
 * front-end device. Each enumerator represents a specific mode of
 * operation, such as high impedance, voltage or current output, and
 * various input modes, including those with HART communication
 * capabilities. This enumeration is crucial for configuring the device
 * to perform specific tasks in different application scenarios.
 *
 * @param AD74413R_HIGH_Z Represents a high impedance state.
 * @param AD74413R_VOLTAGE_OUT Represents a voltage output mode.
 * @param AD74413R_CURRENT_OUT Represents a current output mode.
 * @param AD74413R_VOLTAGE_IN Represents a voltage input mode.
 * @param AD74413R_CURRENT_IN_EXT Represents an external current input mode.
 * @param AD74413R_CURRENT_IN_LOOP Represents a loop current input mode.
 * @param AD74413R_RESISTANCE Represents a resistance measurement mode.
 * @param AD74413R_DIGITAL_INPUT Represents a digital input mode.
 * @param AD74413R_DIGITAL_INPUT_LOOP Represents a loop digital input mode.
 * @param AD74413R_CURRENT_IN_EXT_HART Represents an external current input mode
 * with HART communication.
 * @param AD74413R_CURRENT_IN_LOOP_HART Represents a loop current input mode
 * with HART communication.
 ******************************************************************************/
enum ad74413r_op_mode {
	AD74413R_HIGH_Z,
	AD74413R_VOLTAGE_OUT,
	AD74413R_CURRENT_OUT,
	AD74413R_VOLTAGE_IN,
	AD74413R_CURRENT_IN_EXT,
	AD74413R_CURRENT_IN_LOOP,
	AD74413R_RESISTANCE,
	AD74413R_DIGITAL_INPUT,
	AD74413R_DIGITAL_INPUT_LOOP,
	AD74413R_CURRENT_IN_EXT_HART,
	AD74413R_CURRENT_IN_LOOP_HART,
};

/***************************************************************************//**
 * @brief The `ad74413r_adc_range` enumeration defines the possible ADC voltage
 * ranges that can be configured for the AD74413R device. Each enumerator
 * corresponds to a specific voltage range setting, which is used to
 * configure the ADC for different measurement scenarios, such as 10V,
 * 2.5V with external or internal power, and 5V bi-directional. This
 * configuration is crucial for adapting the ADC to various input signal
 * conditions and ensuring accurate measurements.
 *
 * @param AD74413R_ADC_RANGE_10V Represents a 10V ADC range.
 * @param AD74413R_ADC_RANGE_2P5V_EXT_POW Represents a 2.5V ADC range with
 * external power.
 * @param AD74413R_ADC_RANGE_2P5V_INT_POW Represents a 2.5V ADC range with
 * internal power.
 * @param AD74413R_ADC_RANGE_5V_BI_DIR Represents a 5V bi-directional ADC range.
 ******************************************************************************/
enum ad74413r_adc_range {
	AD74413R_ADC_RANGE_10V,
	AD74413R_ADC_RANGE_2P5V_EXT_POW,
	AD74413R_ADC_RANGE_2P5V_INT_POW,
	AD74413R_ADC_RANGE_5V_BI_DIR
};

/***************************************************************************//**
 * @brief The `ad74413r_conv_seq` enumeration defines the possible commands for
 * controlling the ADC conversion sequence in the AD74413R device. It
 * includes commands to start and stop the conversion process, both in
 * single and continuous modes, as well as to manage the power state of
 * the ADC during these operations.
 *
 * @param AD74413R_STOP_PWR_UP Represents the command to stop and power up the
 * ADC conversion sequence.
 * @param AD74413R_START_SINGLE Represents the command to start a single ADC
 * conversion.
 * @param AD74413R_START_CONT Represents the command to start continuous ADC
 * conversions.
 * @param AD74413R_STOP_PWR_DOWN Represents the command to stop and power down
 * the ADC conversion sequence.
 ******************************************************************************/
enum ad74413r_conv_seq {
	AD74413R_STOP_PWR_UP,
	AD74413R_START_SINGLE,
	AD74413R_START_CONT,
	AD74413R_STOP_PWR_DOWN,
};

/***************************************************************************//**
 * @brief The `ad74413r_gpo_select` enumeration defines the possible
 * configurations for the General Purpose Output (GPO) of the AD74413R
 * device. Each enumerator specifies a different mode of operation for
 * the GPO, such as a pull-down resistor, data output, parallel data
 * output, comparator output, or a high impedance state. This allows for
 * flexible control of the GPO based on the specific application
 * requirements.
 *
 * @param AD74413R_GPO_CONFIG_100K_PD Represents a GPO configuration with a 100K
 * pull-down.
 * @param AD74413R_GPO_CONFIG_DATA Represents a GPO configuration for data
 * output.
 * @param AD74413R_GPO_CONFIG_PAR_DATA Represents a GPO configuration for
 * parallel data output.
 * @param AD74413R_GPO_CONFIG_COMP Represents a GPO configuration for comparator
 * output.
 * @param AD74413R_GPO_CONFIG_HIGH_Z Represents a GPO configuration for high
 * impedance state.
 ******************************************************************************/
enum ad74413r_gpo_select {
	AD74413R_GPO_CONFIG_100K_PD,
	AD74413R_GPO_CONFIG_DATA,
	AD74413R_GPO_CONFIG_PAR_DATA,
	AD74413R_GPO_CONFIG_COMP,
	AD74413R_GPO_CONFIG_HIGH_Z
};

/***************************************************************************//**
 * @brief The `ad74413r_diag_mode` enumeration defines various diagnostic modes
 * for the AD74413R device, each corresponding to a specific component or
 * function of the device that can be monitored or tested. These modes
 * allow the device to perform self-checks and report on the status of
 * its internal components, such as power supplies, temperature, and
 * sense lines, which are crucial for ensuring the device operates
 * correctly and reliably.
 *
 * @param AD74413R_DIAG_AGND Represents the diagnostic mode for analog ground.
 * @param AD74413R_DIAG_TEMP Represents the diagnostic mode for temperature.
 * @param AD74413R_DIAG_AVDD Represents the diagnostic mode for analog VDD.
 * @param AD74413R_DIAG_AVSS Represents the diagnostic mode for analog VSS.
 * @param AD74413R_DIAG_REFOUT Represents the diagnostic mode for reference
 * output.
 * @param AD74413R_DIAG_ALDO_5V Represents the diagnostic mode for analog LDO
 * 5V.
 * @param AD74413R_DIAG_ALDO_1V8 Represents the diagnostic mode for analog LDO
 * 1.8V.
 * @param AD74413R_DIAG_DLDO_1V8 Represents the diagnostic mode for digital LDO
 * 1.8V.
 * @param AD74413R_DIAG_DVCC Represents the diagnostic mode for digital VCC.
 * @param AD74413R_DIAG_IOVDD Represents the diagnostic mode for IO VDD.
 * @param AD74413R_SENSEL_A Represents the diagnostic mode for sense line A.
 * @param AD74413R_SENSEL_B Represents the diagnostic mode for sense line B.
 * @param AD74413R_SENSEL_C Represents the diagnostic mode for sense line C.
 * @param AD74413R_SENSEL_D Represents the diagnostic mode for sense line D.
 ******************************************************************************/
enum ad74413r_diag_mode {
	AD74413R_DIAG_AGND,
	AD74413R_DIAG_TEMP,
	AD74413R_DIAG_AVDD,
	AD74413R_DIAG_AVSS,
	AD74413R_DIAG_REFOUT,
	AD74413R_DIAG_ALDO_5V,
	AD74413R_DIAG_ALDO_1V8,
	AD74413R_DIAG_DLDO_1V8,
	AD74413R_DIAG_DVCC,
	AD74413R_DIAG_IOVDD,
	AD74413R_SENSEL_A,
	AD74413R_SENSEL_B,
	AD74413R_SENSEL_C,
	AD74413R_SENSEL_D
};

/***************************************************************************//**
 * @brief The `ad74413r_debounce_mode` is an enumeration that defines the
 * possible debounce modes for the IOx inputs when the device is
 * operating in digital input mode. It provides two distinct modes,
 * `AD74413R_DEBOUNCE_MODE_0` and `AD74413R_DEBOUNCE_MODE_1`, which can
 * be used to configure the debounce behavior of the digital inputs,
 * potentially affecting how input signals are filtered or stabilized
 * before being processed by the device.
 *
 * @param AD74413R_DEBOUNCE_MODE_0 Represents the first debounce mode option.
 * @param AD74413R_DEBOUNCE_MODE_1 Represents the second debounce mode option.
 ******************************************************************************/
enum ad74413r_debounce_mode {
	AD74413R_DEBOUNCE_MODE_0,
	AD74413R_DEBOUNCE_MODE_1
};

/***************************************************************************//**
 * @brief The `ad74413r_slew_lin_step` enumeration defines the possible step
 * increments for the Digital-to-Analog Converter (DAC) when slew control
 * is enabled in the AD74413R device. These steps determine the
 * granularity of the DAC's output changes, allowing for smoother
 * transitions between output levels. The enumeration provides four
 * distinct step sizes, offering flexibility in controlling the rate of
 * change in the DAC output.
 *
 * @param AD74413R_STEP_64 Represents a step increment of 64 for the DAC when
 * slew control is enabled.
 * @param AD74413R_STEP_120 Represents a step increment of 120 for the DAC when
 * slew control is enabled.
 * @param AD74413R_STEP_500 Represents a step increment of 500 for the DAC when
 * slew control is enabled.
 * @param AD74413R_STEP_1820 Represents a step increment of 1820 for the DAC
 * when slew control is enabled.
 ******************************************************************************/
enum ad74413r_slew_lin_step {
	AD74413R_STEP_64,
	AD74413R_STEP_120,
	AD74413R_STEP_500,
	AD74413R_STEP_1820,
};

/***************************************************************************//**
 * @brief The `ad74413r_lin_rate` enumeration defines the possible linear update
 * rates for the Digital-to-Analog Converter (DAC) when slew control is
 * enabled in the AD74413R device. These rates determine how quickly the
 * DAC output can change, with options ranging from 4 kHz to 240 kHz,
 * allowing for flexibility in applications requiring different speed and
 * precision levels.
 *
 * @param AD74413R_LIN_RATE_4KHZ Represents a linear update rate of 4 kHz for
 * the DAC.
 * @param AD74413R_LIN_RATE_64KHZ Represents a linear update rate of 64 kHz for
 * the DAC.
 * @param AD74413R_LIN_RATE_150KHZ Represents a linear update rate of 150 kHz
 * for the DAC.
 * @param AD74413R_LIN_RATE_240KHZ Represents a linear update rate of 240 kHz
 * for the DAC.
 ******************************************************************************/
enum ad74413r_lin_rate {
	AD74413R_LIN_RATE_4KHZ,
	AD74413R_LIN_RATE_64KHZ,
	AD74413R_LIN_RATE_150KHZ,
	AD74413R_LIN_RATE_240KHZ,
};

/***************************************************************************//**
 * @brief The `ad74413r_adc_sample` enumeration defines various sample rates for
 * the ADC (Analog-to-Digital Converter) in the AD74413R device. Each
 * enumerator corresponds to a specific frequency in Hertz, allowing the
 * user to select the desired sampling rate for ADC operations. This is
 * crucial for configuring the ADC to match the application's
 * requirements for data acquisition speed and resolution.
 *
 * @param AD74413R_ADC_SAMPLE_20HZ Represents an ADC sample rate of 20 Hz.
 * @param AD74413R_ADC_SAMPLE_4800HZ Represents an ADC sample rate of 4800 Hz.
 * @param AD74413R_ADC_SAMPLE_10HZ Represents an ADC sample rate of 10 Hz.
 * @param AD74413R_ADC_SAMPLE_1200HZ Represents an ADC sample rate of 1200 Hz.
 ******************************************************************************/
enum ad74413r_adc_sample {
	AD74413R_ADC_SAMPLE_20HZ = 20,
	AD74413R_ADC_SAMPLE_4800HZ = 4800,
	AD74413R_ADC_SAMPLE_10HZ = 10,
	AD74413R_ADC_SAMPLE_1200HZ = 1200,
};

/***************************************************************************//**
 * @brief The `ad74413r_init_param` structure is used to define the
 * initialization parameters for the AD74413R device. It includes the
 * chip ID to identify the specific device variant, SPI communication
 * parameters for setting up the communication interface, and a pointer
 * to GPIO parameters for handling the reset functionality. This
 * structure is essential for configuring the device before it is used in
 * an application.
 *
 * @param chip_id Specifies the chip ID of the AD74413R device.
 * @param comm_param Holds the SPI communication parameters for initializing the
 * device.
 * @param reset_gpio_param Pointer to the GPIO initialization parameters for the
 * reset pin.
 ******************************************************************************/
struct ad74413r_init_param {
	enum ad74413r_chip_id chip_id;
	struct no_os_spi_init_param comm_param;
	struct no_os_gpio_init_param *reset_gpio_param;
};

/***************************************************************************//**
 * @brief The `ad74413r_decimal` structure is designed to represent a decimal
 * number by separating it into two components: an integer part and a
 * decimal part. This allows for precise representation and manipulation
 * of decimal values, which is particularly useful in applications
 * requiring high precision, such as ADC (Analog-to-Digital Converter)
 * operations. The use of 64-bit and 32-bit integers ensures that a wide
 * range of values can be accurately represented.
 *
 * @param integer Stores the integer part of a decimal value as a 64-bit signed
 * integer.
 * @param decimal Stores the fractional part of a decimal value as a 32-bit
 * signed integer.
 ******************************************************************************/
struct ad74413r_decimal {
	int64_t integer;
	int32_t decimal;
};

/***************************************************************************//**
 * @brief The `ad74413r_channel_config` structure is used to define the
 * configuration of a channel in the AD74413R device. It includes a
 * boolean flag to indicate if the channel is enabled and an enumeration
 * to specify the operational mode of the channel, such as voltage
 * output, current input, or digital input, among others. This structure
 * is essential for managing the state and functionality of each channel
 * in the device.
 *
 * @param enabled A boolean flag indicating whether the channel is enabled.
 * @param function An enumeration specifying the operational mode of the
 * channel.
 ******************************************************************************/
struct ad74413r_channel_config {
	bool enabled;
	enum ad74413r_op_mode function;
};

/***************************************************************************//**
 * @brief The `_ad74413r_live_status` structure is a bitfield representation of
 * the live status register for the AD74413R device, capturing various
 * error states and operational statuses. Each field in the structure
 * corresponds to a specific error or status indicator, such as
 * voltage/current errors on different channels, temperature errors, and
 * ADC operation states. This structure is used to monitor the real-time
 * operational health and status of the device, providing a compact and
 * efficient way to track multiple conditions simultaneously.
 *
 * @param VI_ERR_A Indicates a voltage/current error on channel A.
 * @param VI_ERR_B Indicates a voltage/current error on channel B.
 * @param VI_ERR_C Indicates a voltage/current error on channel C.
 * @param VI_ERR_D Indicates a voltage/current error on channel D.
 * @param HI_TEMP_ERR Indicates a high temperature error.
 * @param CHARGE_PUMP_ERR Indicates a charge pump error.
 * @param ALDO5V_ERR Indicates an error with the 5V analog low dropout
 * regulator.
 * @param AVDD_ERR Indicates an error with the analog supply voltage.
 * @param DVCC_ERR Indicates an error with the digital supply voltage.
 * @param ALDO1V8_ERR Indicates an error with the 1.8V analog low dropout
 * regulator.
 * @param ADC_CH_CURR Represents the current ADC channel being processed, using
 * 3 bits.
 * @param ADC_BUSY Indicates if the ADC is currently busy.
 * @param ADC_DATA_RDY Indicates if the ADC data is ready.
 * @param _RESERVED Reserved bit for future use or alignment.
 ******************************************************************************/
struct _ad74413r_live_status {
	uint8_t VI_ERR_A: 1;
	uint8_t VI_ERR_B: 1;
	uint8_t VI_ERR_C: 1;
	uint8_t VI_ERR_D: 1;
	uint8_t HI_TEMP_ERR: 1;
	uint8_t CHARGE_PUMP_ERR: 1;
	uint8_t ALDO5V_ERR: 1;
	uint8_t AVDD_ERR: 1;
	uint8_t DVCC_ERR: 1;
	uint8_t ALDO1V8_ERR: 1;
	uint8_t ADC_CH_CURR: 3;
	uint8_t ADC_BUSY: 1;
	uint8_t ADC_DATA_RDY: 1;
	uint8_t _RESERVED: 1;
};

/***************************************************************************//**
 * @brief The `ad74413r_live_status` union is designed to store the live status
 * of the AD74413R device, providing a flexible way to access status
 * information. It contains a struct, `_ad74413r_live_status`, which
 * breaks down the status into individual bit fields for specific error
 * and status indicators, such as voltage input errors and temperature
 * errors. Alternatively, the entire status can be accessed as a single
 * 16-bit value, allowing for efficient reading and writing of the status
 * register.
 *
 * @param status_bits A struct that holds individual status bit fields for
 * various error and status indicators.
 * @param value A 16-bit unsigned integer representing the combined status bits
 * as a single value.
 ******************************************************************************/
union ad74413r_live_status {
	struct _ad74413r_live_status status_bits;
	uint16_t value;
};

/***************************************************************************//**
 * @brief The `ad74413r_desc` structure is a device descriptor for the AD74413R,
 * a multi-channel analog input/output device. It encapsulates the
 * necessary information for managing the device, including the chip ID,
 * communication interface, and channel configurations. The structure
 * also includes a buffer for communication and a GPIO descriptor for
 * handling device resets. This descriptor is essential for initializing
 * and controlling the AD74413R device in various operational modes.
 *
 * @param chip_id Specifies the chip ID of the AD74413R device.
 * @param comm_desc Pointer to the SPI communication descriptor used for device
 * communication.
 * @param comm_buff Buffer used for communication, with a fixed size of 4 bytes.
 * @param channel_configs Array of channel configurations, one for each channel
 * of the device.
 * @param reset_gpio Pointer to the GPIO descriptor used for resetting the
 * device.
 ******************************************************************************/
struct ad74413r_desc {
	enum ad74413r_chip_id chip_id;
	struct no_os_spi_desc *comm_desc;
	uint8_t comm_buff[4];
	struct ad74413r_channel_config channel_configs[AD74413R_N_CHANNELS];
	struct no_os_gpio_desc *reset_gpio;
};

/***************************************************************************//**
 * @brief This function is used to convert a given voltage in millivolts to a
 * corresponding 13-bit DAC code for the AD74413R device. It is essential
 * to ensure that the input voltage does not exceed the maximum DAC range
 * of 11000 millivolts. If the input voltage is within the valid range,
 * the function calculates the DAC code and stores it in the provided
 * output parameter. This function should be used when setting up the DAC
 * output voltage for the device.
 *
 * @param mvolts The voltage in millivolts to be converted. It must be a non-
 * negative value not exceeding 11000 millivolts. If the value
 * exceeds this range, the function returns an error.
 * @param code A pointer to a uint32_t where the resulting DAC code will be
 * stored. This pointer must not be null, and the caller is
 * responsible for providing a valid memory location.
 * @return Returns 0 on success, indicating the conversion was successful.
 * Returns a negative error code if the input voltage is out of range.
 ******************************************************************************/
int ad74413r_dac_voltage_to_code(uint32_t, uint32_t *);

/***************************************************************************//**
 * @brief Use this function to translate an ADC range, specified by an
 * enumeration, into a voltage value in millivolts. This is useful when
 * you need to understand the voltage range that corresponds to a
 * specific ADC configuration. Ensure that the `range` parameter is a
 * valid `ad74413r_adc_range` enumeration value. The function will store
 * the resulting voltage in the provided `val` pointer. If an invalid
 * range is provided, the function returns an error code.
 *
 * @param range An enumeration value of type `ad74413r_adc_range` representing
 * the ADC range. Must be a valid enumeration value; otherwise, the
 * function returns an error.
 * @param val A pointer to a `uint32_t` where the function will store the
 * corresponding voltage in millivolts. Must not be null, and the
 * caller is responsible for ensuring the pointer is valid.
 * @return Returns 0 on success, with the voltage value stored in `val`. Returns
 * a negative error code if the `range` is invalid.
 ******************************************************************************/
int ad74413r_range_to_voltage_range(enum ad74413r_adc_range, uint32_t *);

/***************************************************************************//**
 * @brief This function is used to determine the voltage offset associated with
 * a specific ADC range setting. It is useful when configuring or
 * interpreting ADC readings for the AD74413R device. The function must
 * be called with a valid ADC range enumeration value, and it will output
 * the corresponding voltage offset through the provided pointer. If an
 * invalid range is provided, the function returns an error code.
 *
 * @param range Specifies the ADC range using the ad74413r_adc_range
 * enumeration. Must be a valid range value; otherwise, the
 * function returns an error.
 * @param val A pointer to an int32_t where the voltage offset will be stored.
 * Must not be null, as the function writes the result to this
 * location.
 * @return Returns 0 on success, or a negative error code if the range is
 * invalid.
 ******************************************************************************/
int ad74413r_range_to_voltage_offset(enum ad74413r_adc_range, int32_t *);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * address on the AD74413R device. It is typically called when there is a
 * need to configure or update the settings of the device by writing to
 * its registers. The function requires a valid device descriptor, a
 * register address, and the value to be written. It is important to
 * ensure that the device descriptor is properly initialized before
 * calling this function. The function returns an integer status code
 * indicating the success or failure of the write operation.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before use.
 * @param addr A 32-bit unsigned integer representing the register address to
 * which the value will be written. The address should be within the
 * valid range of the device's register map.
 * @param val A 16-bit unsigned integer representing the value to be written to
 * the specified register. The value should be appropriate for the
 * register being accessed.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error during the write operation.
 ******************************************************************************/
int ad74413r_reg_write(struct ad74413r_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief Use this function to read a raw value from a specified register of the
 * AD74413R device. It is essential to ensure that the device descriptor
 * is properly initialized before calling this function. The function
 * performs a two-step SPI communication to retrieve the register value,
 * and it is crucial to provide a valid address and a non-null pointer
 * for storing the result. This function is typically used when low-level
 * access to the device's registers is required.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param addr The address of the register to read from. Must be a valid
 * register address for the AD74413R device.
 * @param val A pointer to a uint8_t variable where the read value will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the SPI communication.
 ******************************************************************************/
int ad74413r_reg_read_raw(struct ad74413r_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief This function is used to read a 16-bit value from a specified register
 * address of the AD74413R device. It should be called when you need to
 * retrieve data from the device's registers. The function requires a
 * valid device descriptor and a pointer to store the read value. It
 * performs a CRC check to ensure data integrity and returns an error if
 * the CRC does not match. Ensure that the device descriptor is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param addr The 32-bit address of the register to read from. Must be a valid
 * register address for the AD74413R device.
 * @param val A pointer to a uint16_t where the read value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or if the CRC check fails.
 ******************************************************************************/
int ad74413r_reg_read(struct ad74413r_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function is used to update a specific field within a register of
 * the AD74413R device. It reads the current value of the register,
 * applies a mask to clear the bits of interest, and then sets them to
 * the new value provided. This function is typically used when only a
 * portion of a register needs to be modified without affecting other
 * bits. It must be called with a valid device descriptor and register
 * address. The function handles errors by returning a non-zero value if
 * the read or write operation fails.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the AD74413R device.
 * @param mask A 16-bit mask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param val A 16-bit value to be written to the register, after being masked
 * and shifted into position. Only the bits corresponding to the mask
 * will be used.
 * @return Returns 0 on success, or a non-zero error code if the read or write
 * operation fails.
 ******************************************************************************/
int ad74413r_reg_update(struct ad74413r_desc *, uint32_t, uint16_t,
			uint16_t);

/***************************************************************************//**
 * @brief This function is used to determine how many channels are currently
 * active on the AD74413R device. It should be called when you need to
 * know the active channel count for configuration or monitoring
 * purposes. The function requires a valid device descriptor and a
 * pointer to store the result. It reads the necessary register to
 * compute the number of active channels and returns this count through
 * the provided pointer. Ensure that the device descriptor is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param nb_channels A pointer to a uint8_t where the number of active channels
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the register read
 * operation fails.
 ******************************************************************************/
int ad74413r_nb_active_channels(struct ad74413r_desc *, uint8_t *);

/***************************************************************************//**
 * @brief Use this function to clear any error flags that may have been set in
 * the AD74413R device's ALERT_STATUS register. This is typically done
 * after handling or logging errors to reset the device's error state. It
 * is important to ensure that the device descriptor is properly
 * initialized before calling this function. This function does not
 * modify the input descriptor or any other state beyond clearing the
 * error flags.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null. The function will not modify this
 * structure.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_clear_errors(struct ad74413r_desc *);

/***************************************************************************//**
 * @brief This function configures the AD74413R device to determine what
 * information it will return during a read operation. It should be used
 * when you need to specify or change the type of data the device
 * provides in response to read commands. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function does not perform any validation on the mode parameter, so it
 * is the caller's responsibility to provide a valid mode value.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param mode A 16-bit unsigned integer specifying the mode for the readback
 * information. The value should be within the valid range for the
 * device's mode settings.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad74413r_set_info(struct ad74413r_desc *desc, uint16_t mode);

/***************************************************************************//**
 * @brief This function resets the AD74413R device using either a hardware or
 * software reset method, depending on the availability of a reset GPIO.
 * It should be called when a reset of the device is required, such as
 * during initialization or error recovery. The function ensures the
 * device is reset and ready for operation by waiting for the required
 * reset time. It is important to ensure that the `desc` parameter is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an `ad74413r_desc` structure that must be
 * initialized before use. This structure contains the device
 * configuration and state, including the reset GPIO if available.
 * The function will return an error if this parameter is null or
 * improperly initialized.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad74413r_reset(struct ad74413r_desc *);

/***************************************************************************//**
 * @brief This function configures a specified channel on the AD74413R device to
 * operate in a given mode. It should be called when you need to change
 * the operational behavior of a channel, such as switching between
 * voltage output and current input modes. The function must be called
 * with a valid device descriptor and channel number. It ensures that the
 * channel is set to a high-impedance state before applying the new mode
 * and enforces necessary delays to ensure proper configuration. The
 * function returns an error code if the operation fails at any step.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null, and should be properly initialized before
 * calling this function.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to channels A to D. Invalid channel numbers will result in
 * undefined behavior.
 * @param ch_func An enum value of type ad74413r_op_mode representing the
 * desired operation mode for the channel. Must be a valid mode
 * defined in the ad74413r_op_mode enumeration.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_set_channel_function(struct ad74413r_desc *,
				  uint32_t, enum ad74413r_op_mode);

/***************************************************************************//**
 * @brief Use this function to obtain the raw ADC conversion result from a
 * specified channel of the AD74413R device. It is essential to ensure
 * that the device descriptor is properly initialized before calling this
 * function. The function reads the ADC result for the specified channel
 * and stores it in the provided memory location. This function is useful
 * when you need to access the raw ADC data for further processing or
 * analysis. Ensure that the channel number is within the valid range,
 * and the pointer to store the result is not null.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number from which to read the ADC result. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a uint16_t variable where the raw ADC result will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ad74413r_get_raw_adc_result(struct ad74413r_desc *, uint32_t,
				uint16_t *);

/***************************************************************************//**
 * @brief This function is used to control the enable state of a specific ADC
 * channel on the AD74413R device. It should be called when you need to
 * start or stop ADC conversions on a particular channel. The function
 * requires a valid device descriptor and a channel index within the
 * supported range. It updates the device's internal configuration to
 * reflect the new state of the channel. Ensure that the device
 * descriptor is properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The index of the ADC channel to be enabled or disabled. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param status A boolean value indicating whether to enable (true) or disable
 * (false) the specified ADC channel.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_set_adc_channel_enable(struct ad74413r_desc *, uint32_t,
				    bool);

/***************************************************************************//**
 * @brief This function is used to control the diagnostic channel conversions on
 * the AD74413R device. It allows enabling or disabling the diagnostic
 * channel for a specified channel index. This function should be called
 * when you need to start or stop diagnostic data collection for a
 * specific channel. Ensure that the device descriptor is properly
 * initialized before calling this function. The function does not
 * perform any action if the channel index is out of range.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel index to configure, ranging from 0 to 3. Values outside
 * this range are invalid and will not affect the device.
 * @param status A boolean value where true enables the diagnostic channel and
 * false disables it.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_diag_channel_enable(struct ad74413r_desc *, uint32_t, bool);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC measurement range
 * setting for a specific channel on the AD74413R device. It is typically
 * called when you need to verify or log the ADC configuration for a
 * channel. The function requires a valid device descriptor and a channel
 * number within the supported range. The result is stored in the
 * provided output parameter. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the ADC range is being queried. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a uint16_t where the ADC range value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The ADC range value is written to the location pointed to by
 * 'val' on success.
 ******************************************************************************/
int ad74413r_get_adc_range(struct ad74413r_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC rejection
 * configuration for a specific channel on the AD74413R device. It is
 * typically called when you need to verify or log the rejection settings
 * of a channel. Ensure that the device descriptor is properly
 * initialized before calling this function. The function will return an
 * error code if the read operation fails.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the ADC rejection setting is to be
 * retrieved. Valid values are 0 to 3, corresponding to channels A to
 * D.
 * @param val A pointer to an enum ad74413r_rejection variable where the
 * rejection setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_get_adc_rejection(struct ad74413r_desc *, uint32_t,
			       enum ad74413r_rejection *);

/***************************************************************************//**
 * @brief This function retrieves the current diagnostic ADC rejection setting
 * from the specified AD74413R device descriptor. It is used to determine
 * whether the diagnostic ADC is configured to reject 50/60 Hz
 * interference or not. This function should be called when the user
 * needs to verify or log the current rejection setting for diagnostic
 * purposes. The function requires a valid device descriptor and a
 * pointer to store the rejection setting. It returns an error code if
 * the operation fails, otherwise it returns 0 on success.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param val A pointer to an enum ad74413r_rejection variable where the
 * rejection setting will be stored. Must not be null. The function
 * writes the current rejection setting to this location.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_get_adc_diag_rejection(struct ad74413r_desc *,
				    enum ad74413r_rejection *);

/***************************************************************************//**
 * @brief This function configures the ADC rejection setting for a specified
 * channel on the AD74413R device. It should be used when you need to
 * adjust the rejection characteristics of the ADC to suit specific
 * application requirements, such as noise reduction. The function
 * requires a valid device descriptor and channel number, and it must be
 * called with a valid rejection configuration value. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null, and the device should be initialized before
 * use.
 * @param ch The channel number to configure, ranging from 0 to 3, corresponding
 * to AD74413R_CH_A to AD74413R_CH_D.
 * @param val An enum value of type ad74413r_rejection specifying the desired
 * rejection configuration. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_adc_rejection(struct ad74413r_desc *, uint32_t,
			       enum ad74413r_rejection);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC sample rate for a
 * specific channel on the AD74413R device. It should be called when you
 * need to know the sample rate setting for a channel, which is
 * determined by the rejection configuration. Ensure that the device
 * descriptor is properly initialized before calling this function. The
 * function will return an error code if the operation fails, such as
 * when the channel number is invalid or the device is not properly
 * configured.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the ADC sample rate is to be
 * retrieved. Valid values are 0 to 3, corresponding to channels A to
 * D.
 * @param val A pointer to an ad74413r_adc_sample enum where the sample rate
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_get_adc_rate(struct ad74413r_desc *, uint32_t,
			  enum ad74413r_adc_sample *);

/***************************************************************************//**
 * @brief This function configures the ADC sample rate for a specific channel on
 * the AD74413R device. It should be used when you need to adjust the
 * sampling frequency of the ADC to match the requirements of your
 * application. The function must be called with a valid device
 * descriptor and a channel number within the supported range. The sample
 * rate is specified using an enumeration value, which determines the
 * conversion rate of the ADC. If the provided sample rate is not
 * supported, the function will return an error code.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param ch The channel number for which the ADC sample rate is to be set.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param val An enumeration value of type ad74413r_adc_sample representing the
 * desired ADC sample rate. Must be a valid enumeration value;
 * otherwise, the function will return an error.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_set_adc_rate(struct ad74413r_desc *, uint32_t,
			  enum ad74413r_adc_sample);

/***************************************************************************//**
 * @brief This function is used to obtain the current ADC diagnostic sample rate
 * for a specified channel on the AD74413R device. It is typically called
 * when you need to verify or log the diagnostic sample rate settings of
 * the ADC. The function requires a valid device descriptor and a channel
 * identifier. It returns the sample rate through a pointer to an enum,
 * which must not be null. Ensure that the device descriptor is properly
 * initialized before calling this function.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device
 * descriptor. Must not be null and should be properly initialized
 * before use.
 * @param ch A uint32_t representing the channel number for which the diagnostic
 * sample rate is to be retrieved. Valid values are typically within
 * the range of available channels on the device.
 * @param val A pointer to an enum ad74413r_adc_sample where the retrieved
 * sample rate will be stored. Must not be null.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int ad74413r_get_adc_diag_rate(struct ad74413r_desc *, uint32_t,
			       enum ad74413r_adc_sample *);

/***************************************************************************//**
 * @brief This function configures the sample rate for the ADC diagnostic
 * channel on the specified device channel. It should be used when you
 * need to adjust the diagnostic sampling frequency for specific
 * monitoring or testing purposes. The function requires a valid device
 * descriptor and a channel number within the supported range. It only
 * accepts specific sample rate values defined in the
 * `ad74413r_adc_sample` enumeration. If an invalid sample rate is
 * provided, the function returns an error code.
 *
 * @param desc A pointer to an `ad74413r_desc` structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param ch The channel number for which the diagnostic sample rate is to be
 * set. Valid values are 0 to 3, corresponding to channels A to D.
 * @param val The desired ADC diagnostic sample rate, specified as an
 * `ad74413r_adc_sample` enumeration value. Only
 * `AD74413R_ADC_SAMPLE_20HZ` and `AD74413R_ADC_SAMPLE_4800HZ` are
 * valid.
 * @return Returns 0 on success. If an invalid sample rate is provided, returns
 * a negative error code, such as -EINVAL.
 ******************************************************************************/
int ad74413r_set_adc_diag_rate(struct ad74413r_desc *, uint32_t,
			       enum ad74413r_adc_sample);

/***************************************************************************//**
 * @brief This function is used to control the ADC conversion sequence of the
 * AD74413R device. It can start, stop, or configure the ADC to perform
 * single or continuous conversions. The function must be called with a
 * valid device descriptor and a conversion sequence command. After
 * setting the conversion sequence, the ADC is powered up, and a delay of
 * 100 microseconds is required before conversions begin. This function
 * is typically used when configuring the ADC for different operational
 * modes or when changing the conversion sequence during runtime.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param status An enum value of type ad74413r_conv_seq indicating the desired
 * ADC conversion sequence. Valid values are AD74413R_STOP_PWR_UP,
 * AD74413R_START_SINGLE, AD74413R_START_CONT, and
 * AD74413R_STOP_PWR_DOWN.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_set_adc_conv_seq(struct ad74413r_desc *, enum ad74413r_conv_seq);

/***************************************************************************//**
 * @brief This function retrieves a single raw ADC conversion result for a
 * specified channel on the AD74413R device. It can be used to obtain
 * either a regular ADC measurement or a diagnostic measurement,
 * depending on the `is_diag` parameter. The function must be called with
 * a valid device descriptor and a channel number within the supported
 * range. It temporarily enables the specified channel for conversion,
 * waits for the conversion to complete, retrieves the result, and then
 * disables the channel. This function is useful for applications
 * requiring single-shot ADC measurements.
 *
 * @param desc A pointer to an `ad74413r_desc` structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param ch The channel number for which the ADC conversion is requested. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a `uint16_t` where the ADC conversion result will be
 * stored. Must not be null.
 * @param is_diag A boolean indicating whether to perform a diagnostic
 * measurement (`true`) or a regular ADC measurement (`false`).
 * @return Returns 0 on success, or a negative error code on failure. The ADC
 * conversion result is stored in the location pointed to by `val`.
 ******************************************************************************/
int ad74413r_get_adc_single(struct ad74413r_desc *, uint32_t, uint16_t *, bool);

/***************************************************************************//**
 * @brief This function retrieves the real ADC value for a specified channel on
 * the AD74413R device, converting it into a decimal format based on the
 * channel's configured operation mode. It should be called when an
 * accurate measurement of the ADC value is required, and the channel
 * must be properly configured beforehand. The function handles different
 * operation modes and returns an error if the mode is unsupported or
 * invalid for the given chip.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to read from, ranging from 0 to 3. Invalid
 * channel numbers will result in an error.
 * @param val A pointer to an ad74413r_decimal structure where the result will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or the channel mode is unsupported.
 ******************************************************************************/
int ad74413r_adc_get_value(struct ad74413r_desc *, uint32_t,
			   struct ad74413r_decimal *);

/***************************************************************************//**
 * @brief This function retrieves the temperature of the AD74413R device's die
 * for a specified channel and stores it in a provided variable. It
 * should be used when temperature monitoring of the device is required.
 * The function must be called with a valid device descriptor and a
 * channel number within the supported range. The temperature value is
 * adjusted using predefined constants to provide a scaled result. Ensure
 * that the `temp` pointer is not null to avoid undefined behavior.
 *
 * @param desc A pointer to an initialized `ad74413r_desc` structure
 * representing the device. Must not be null.
 * @param ch The channel number from which to read the temperature. Valid values
 * are 0 to 3, corresponding to channels A to D.
 * @param temp A pointer to a `uint16_t` where the temperature value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_get_temp(struct ad74413r_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function sets a digital-to-analog converter (DAC) code for a
 * specified channel on the AD74413R device and immediately loads it into
 * the DAC. It should be used when you need to update the output voltage
 * or current of a specific channel. The function requires a valid device
 * descriptor and channel number, and the DAC code must be within the
 * permissible range. It is important to ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the DAC code is to be set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param dac_code The DAC code to be set for the specified channel. It must be
 * a 13-bit value, ranging from 0 to 8191.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad74413r_set_channel_dac_code(struct ad74413r_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function is used to set a diagnostic mode for a specific channel
 * on the AD74413R device. It should be called when you need to configure
 * the diagnostic behavior of a channel, allowing the device to perform
 * specific diagnostic checks. Ensure that the device descriptor is
 * properly initialized before calling this function. The function does
 * not perform any validation on the channel number or diagnostic mode,
 * so it is the caller's responsibility to provide valid inputs.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number to which the diagnostic mode will be assigned.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param diag_code An enum value of type ad74413r_diag_mode representing the
 * diagnostic mode to be set. Must be a valid diagnostic mode
 * as defined in the enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_diag(struct ad74413r_desc *, uint32_t,
		      enum ad74413r_diag_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the diagnostic code from a specified
 * channel of the AD74413R device. It should be called when diagnostic
 * information is needed for a particular channel. The function requires
 * a valid device descriptor and a channel number within the supported
 * range. The diagnostic code is returned through a pointer, which must
 * not be null. If the function encounters an error during the read
 * operation, it will return a non-zero error code.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null and should be properly initialized before
 * calling this function.
 * @param ch The channel number from which to retrieve the diagnostic code.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @param diag_code A pointer to a uint16_t where the diagnostic code will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a non-zero error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_get_diag(struct ad74413r_desc *, uint32_t, uint16_t *);

/***************************************************************************//**
 * @brief This function configures the debounce mode for the IOx inputs of a
 * specified channel when the ADC is operating in digital input mode. It
 * is essential to ensure that the device descriptor is properly
 * initialized before calling this function. The function is useful in
 * applications where input signal stability is critical, and debounce
 * logic is required to filter out noise or spurious signals. The
 * function does not perform any validation on the channel number or
 * debounce mode, so it is the caller's responsibility to ensure these
 * parameters are within valid ranges.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the debounce mode is to be set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param mode The debounce mode to be set, specified as an enum
 * ad74413r_debounce_mode. Valid values are AD74413R_DEBOUNCE_MODE_0
 * and AD74413R_DEBOUNCE_MODE_1.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_debounce_mode(struct ad74413r_desc *, uint32_t,
			       enum ad74413r_debounce_mode);

/***************************************************************************//**
 * @brief This function configures the debounce settle time for the digital
 * input mode of a specified channel on the AD74413R device. It should be
 * used when the ADC is operating in digital input mode and a specific
 * debounce time is required to filter out noise or unwanted signals. The
 * function maps the provided time to the closest available debounce time
 * setting. Ensure that the device descriptor is properly initialized
 * before calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the debounce time is being set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param time The desired debounce time in microseconds. The function will map
 * this to the closest available setting.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_debounce_time(struct ad74413r_desc *, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief This function is used to obtain the General Purpose Output (GPO) value
 * for a specific channel on the AD74413R device. It should be called
 * when you need to read the current state of a GPO pin. The function
 * requires a valid device descriptor and a channel number within the
 * supported range. The result is stored in the provided output
 * parameter. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which to retrieve the GPO value. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param val A pointer to a uint8_t where the GPO value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad74413r_gpo_get(struct ad74413r_desc *, uint32_t, uint8_t *);

/***************************************************************************//**
 * @brief This function configures the General Purpose Output (GPO) operation
 * mode for a specified channel on the AD74413R device. It should be used
 * when you need to change the GPO settings for a particular channel,
 * which can be one of the predefined modes in the `ad74413r_gpo_select`
 * enumeration. Ensure that the device descriptor is properly initialized
 * before calling this function. The function returns an integer status
 * code indicating success or failure of the operation.
 *
 * @param desc A pointer to an `ad74413r_desc` structure representing the device
 * descriptor. Must not be null and should be initialized before
 * use. The caller retains ownership.
 * @param ch An unsigned 32-bit integer representing the channel number to
 * configure. Valid values are 0 to 3, corresponding to channels A to
 * D.
 * @param config An `enum ad74413r_gpo_select` value specifying the desired GPO
 * operation mode. Must be a valid enumeration value.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad74413r_set_gpo_config(struct ad74413r_desc *, uint32_t,
			    enum ad74413r_gpo_select);

/***************************************************************************//**
 * @brief This function sets the threshold voltage for a specified channel on
 * the AD74413R device when operating in digital input mode. It is used
 * to define the voltage level at which a digital input signal is
 * considered high. The function must be called with a valid device
 * descriptor and a channel number within the supported range. The
 * threshold value must not exceed the maximum allowable threshold range,
 * otherwise, the function will return an error. This function modifies
 * the device's internal register settings to apply the threshold.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the threshold is being set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param threshold The desired threshold voltage in millivolts. Must be between
 * 0 and AD74413R_THRESHOLD_RANGE (16000 mV). Values outside
 * this range will result in an error.
 * @return Returns 0 on success, or a negative error code if the threshold is
 * out of range or if there is a failure in updating the device
 * registers.
 ******************************************************************************/
int ad74413r_set_threshold(struct ad74413r_desc *, uint32_t, uint32_t);

/***************************************************************************//**
 * @brief This function is used to set the logic value of a General Purpose
 * Output (GPO) pin on a specified channel of the AD74413R device. It
 * should be called when you need to control the output state of a GPO
 * pin. The function requires a valid device descriptor and channel
 * number, and it expects the channel to be configured for GPO operation.
 * The logic value to be set must be provided as a parameter. If the
 * channel is not configured correctly or if invalid parameters are
 * provided, the function will return an error code.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number on which to set the GPO value. Must be a valid
 * channel number (0 to 3). Invalid channel numbers will result in an
 * error.
 * @param val The logic value to set on the GPO pin. Typically, 0 represents a
 * low logic level and 1 represents a high logic level. Other values
 * may be considered invalid and result in an error.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int ad74413r_gpo_set(struct ad74413r_desc *, uint32_t, uint8_t);

/***************************************************************************//**
 * @brief This function is used to set multiple General Purpose Output (GPO)
 * values on the AD74413R device at once. It should be called when you
 * need to update the GPO states for multiple channels simultaneously.
 * The function requires a valid device descriptor and a bitmask
 * indicating which GPOs to set. Each bit in the mask corresponds to a
 * GPO channel, and a set bit indicates that the corresponding GPO should
 * be configured. The function returns an error code if the operation
 * fails, which can occur if the device descriptor is invalid or if there
 * is a communication error.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param mask A 32-bit unsigned integer where each bit represents a GPO
 * channel. A set bit indicates the corresponding GPO should be
 * configured. Valid values depend on the number of GPO channels
 * supported by the device.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_gpo_set_multiple(struct ad74413r_desc *, uint32_t);

/***************************************************************************//**
 * @brief Use this function to retrieve the current live status of the AD74413R
 * device, which includes various error and status flags. This function
 * should be called when you need to monitor the device's operational
 * status or diagnose issues. Ensure that the device descriptor is
 * properly initialized before calling this function.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param status A pointer to an ad74413r_live_status union where the live
 * status bits will be stored. Must not be null.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int ad74413r_get_live(struct ad74413r_desc *,
		      union ad74413r_live_status *);

/***************************************************************************//**
 * @brief This function sets the clear code for the DAC on a specified channel
 * of the AD74413R device. It should be used when you need to define the
 * code that the DAC will revert to when a clear operation is triggered.
 * Ensure that the device descriptor is properly initialized before
 * calling this function. The function does not perform any validation on
 * the channel or code values, so it is the caller's responsibility to
 * ensure they are within valid ranges.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which the DAC clear code is being set. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param code The 16-bit code to set as the DAC clear code. The caller must
 * ensure this is within the valid range for the DAC.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_set_dac_clear_code(struct ad74413r_desc *, uint32_t, uint16_t);

/***************************************************************************//**
 * @brief This function is used to clear the DAC on a specified channel of the
 * AD74413R device to a predefined code stored in the DAC_CLR_CODE
 * register. It should be called when there is a need to reset the DAC
 * output to a known state. The function requires a valid device
 * descriptor and a channel number. It is important to ensure that the
 * device is properly initialized before calling this function. The
 * function will return an error code if the operation fails, which can
 * occur if the device descriptor is invalid or if there is a
 * communication error with the device.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param ch The channel number to clear the DAC for. Valid values are 0 to 3,
 * corresponding to channels A to D. If an invalid channel is
 * specified, the behavior is undefined.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_clear_dac(struct ad74413r_desc *, uint32_t);

/***************************************************************************//**
 * @brief This function configures and enables the slew rate control for a DAC
 * on a specified channel of the AD74413R device. It should be used when
 * precise control over the rate of change of the DAC output is required.
 * The function must be called with a valid device descriptor and
 * appropriate channel, step, and rate parameters. It is important to
 * ensure that the device is properly initialized before calling this
 * function. The function returns an error code if the operation fails,
 * which can occur if the parameters are invalid or if there is a
 * communication issue with the device.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number on which to enable slew rate control. Valid
 * values are 0 to 3, corresponding to channels A to D.
 * @param step An enum value of type ad74413r_slew_lin_step specifying the
 * number of increments per step for the DAC. Must be a valid enum
 * value.
 * @param rate An enum value of type ad74413r_lin_rate specifying the update
 * rate for the DAC. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_dac_slew_enable(struct ad74413r_desc *, uint32_t,
			     enum ad74413r_slew_lin_step,
			     enum ad74413r_lin_rate);

/***************************************************************************//**
 * @brief Use this function to disable the slew rate control for a DAC on a
 * specified channel of the AD74413R device. This function is typically
 * called when precise control over the DAC output is required without
 * the gradual changes imposed by slew rate control. Ensure that the
 * device descriptor is properly initialized before calling this
 * function. The function will return an error code if the operation
 * fails.
 *
 * @param desc A pointer to an initialized ad74413r_desc structure representing
 * the device. Must not be null.
 * @param ch The channel number for which to disable the DAC slew rate control.
 * Valid values are 0 to 3, corresponding to channels A to D.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad74413r_dac_slew_disable(struct ad74413r_desc *, uint32_t);

/***************************************************************************//**
 * @brief This function is used to control the thermal reset feature of the
 * AD74413R device. It should be called when there is a need to enable or
 * disable the thermal reset functionality, which is typically used to
 * manage thermal conditions in the device. The function requires a valid
 * device descriptor and a boolean flag indicating whether to enable or
 * disable the feature. It is important to ensure that the device
 * descriptor is properly initialized before calling this function.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param enable A boolean value where 'true' enables the thermal reset feature
 * and 'false' disables it.
 * @return Returns an integer status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int ad74413r_set_therm_rst(struct ad74413r_desc *, bool);

/***************************************************************************//**
 * @brief This function sets up the AD74413R device by allocating and
 * initializing a device descriptor based on the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function handles the initialization
 * of communication interfaces and performs a series of tests to ensure
 * the device is ready for use. If initialization fails at any step,
 * resources are cleaned up and an error code is returned. The caller is
 * responsible for providing valid initialization parameters and managing
 * the lifecycle of the descriptor.
 *
 * @param desc A pointer to a pointer where the initialized device descriptor
 * will be stored. Must not be null. The caller takes ownership of
 * the allocated descriptor and is responsible for freeing it using
 * `ad74413r_remove`.
 * @param init_param A pointer to a structure containing initialization
 * parameters for the device. Must not be null. The structure
 * should be properly populated with valid communication and
 * reset GPIO parameters.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -EINVAL for
 * invalid parameters or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int ad74413r_init(struct ad74413r_desc **, struct ad74413r_init_param *);

/***************************************************************************//**
 * @brief This function is used to properly release all resources associated
 * with an AD74413R device descriptor, ensuring that the device is reset
 * to its default state and that any allocated resources are freed. It
 * should be called when the device is no longer needed to prevent
 * resource leaks. The function must be called with a valid descriptor
 * that was previously initialized. If the descriptor is null, the
 * function returns an error code. It handles any errors encountered
 * during the resource release process and returns appropriate error
 * codes.
 *
 * @param desc A pointer to an ad74413r_desc structure representing the device
 * descriptor. Must not be null. The function returns -EINVAL if
 * this parameter is null. The caller retains ownership of the
 * pointer, but the resources it points to will be freed.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the resource release process.
 ******************************************************************************/
int ad74413r_remove(struct ad74413r_desc *desc);

#endif // _AD74413R_H
