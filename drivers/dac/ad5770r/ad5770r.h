/***************************************************************************//**
 *   @file   ad5770r.h
 *   @brief  Header file for AD5770R Driver.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
********************************************************************************
 * Copyright 2018, 2020(c) Analog Devices, Inc.
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

#ifndef AD5770R_H_
#define AD5770R_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#ifndef BIT
#define NO_OS_BIT(x)					(1UL << (x))
#endif
/*
 * Create a contiguous bitmask starting at bit position @l and ending at
 * position @h.
 */
#ifdef GENMASK
#define NO_OS_GENMASK(h, l) \
		(((~0UL) - (1UL << (l)) + 1) & (~0UL >> (31 - (h))))
#endif

/*SPI configuration registers*/
#define AD5770R_INTERFACE_CONFIG_A 	0x00
#define AD5770R_INTERFACE_CONFIG_B 	0x01
#define AD5770R_CHIP_TYPE		0x03
#define AD5770R_PRODUCT_ID_L		0x04
#define AD5770R_PRODUCT_ID_H		0x05
#define AD5770R_CHIP_GRADE		0x06
#define AD5770R_SCRATCH_PAD		0x0A
#define AD5770R_SPI_REVISION		0x0B
#define AD5770R_VENDOR_L		0x0C
#define AD5770R_VENDOR_H		0x0D
#define AD5770R_STREAM_MODE		0x0E
#define AD5770R_INTERFACE_CONFIG_C	0x10
#define AD5770R_INTERFACE_STATUS_A	0x11

/*AD5770R configuration registers*/
#define AD5770R_CHANNEL_CONFIG		0x14
#define AD5770R_OUTPUT_RANGE_CH0	0x15
#define AD5770R_OUTPUT_RANGE_CH1	0x16
#define AD5770R_OUTPUT_RANGE_CH2	0x17
#define AD5770R_OUTPUT_RANGE_CH3	0x18
#define AD5770R_OUTPUT_RANGE_CH4	0x19
#define AD5770R_OUTPUT_RANGE_CH5	0x1A
#define AD5770R_REFERENCE		0x1B
#define AD5770R_ALARM_CONFIG		0x1C
#define AD5770R_OUTPUT_FILTER_CH0	0x1D
#define AD5770R_OUTPUT_FILTER_CH1	0x1E
#define AD5770R_OUTPUT_FILTER_CH2	0x1F
#define AD5770R_OUTPUT_FILTER_CH3	0x20
#define AD5770R_OUTPUT_FILTER_CH4	0x21
#define AD5770R_OUTPUT_FILTER_CH5	0x22
#define AD5770R_MONITOR_SETUP		0x23
#define AD5770R_STATUS			0x24
#define AD5770R_HW_LDAC			0x25
#define AD5770R_CH0_DAC_LSB		0x26
#define AD5770R_CH0_DAC_MSB		0x27
#define AD5770R_CH1_DAC_LSB		0x28
#define AD5770R_CH1_DAC_MSB		0x29
#define AD5770R_CH2_DAC_LSB		0x2A
#define AD5770R_CH2_DAC_MSB		0x2B
#define AD5770R_CH3_DAC_LSB		0x2C
#define AD5770R_CH3_DAC_MSB		0x2D
#define AD5770R_CH4_DAC_LSB		0x2E
#define AD5770R_CH4_DAC_MSB		0x2F
#define AD5770R_CH5_DAC_LSB		0x30
#define AD5770R_CH5_DAC_MSB		0x31
#define AD5770R_DAC_PAGE_MASK_LSB	0x32
#define AD5770R_DAC_PAGE_MASK_MSB	0x33
#define AD5770R_CH_SELECT		0x34
#define AD5770R_INPUT_PAGE_MASK_LSB	0x35
#define AD5770R_INPUT_PAGE_MASK_MSB	0x36
#define AD5770R_SW_LDAC			0x37
#define AD5770R_CH0_INPUT_LSB		0x38
#define AD5770R_CH0_INPUT_MSB		0x39
#define AD5770R_CH1_INPUT_LSB		0x3A
#define AD5770R_CH1_INPUT_MSB		0x3B
#define AD5770R_CH2_INPUT_LSB		0x3C
#define AD5770R_CH2_INPUT_MSB		0x3D
#define AD5770R_CH3_INPUT_LSB		0x3E
#define AD5770R_CH3_INPUT_MSB		0x3F
#define AD5770R_CH4_INPUT_LSB		0x40
#define AD5770R_CH4_INPUT_MSB		0x41
#define AD5770R_CH5_INPUT_LSB		0x42
#define AD5770R_CH5_INPUT_MSB		0x43

/* AD5770R_INTERFACE_CONFIG_A */
#define AD5770R_INTERFACE_CONFIG_A_SW_RESET_MSK			NO_OS_BIT(7) | NO_OS_BIT(0)
#define AD5770R_INTERFACE_CONFIG_A_SW_RESET(x)			(((x) & 0x1) | 0x80)
#define AD5770R_INTERFACE_CONFIG_A_ADDR_ASCENSION_MSB_MSK	NO_OS_BIT(5)
#define AD5770R_INTERFACE_CONFIG_A_ADDR_ASCENSION_MSB(x)	(((x) & 0x1) << 5)
#define AD5770R_INTERFACE_CONFIG_A_SDO_ACTIVE_MSK		NO_OS_BIT(4) | NO_OS_BIT(3)

/* AD5770R_INTERFACE_CONFIG_B */
#define AD5770R_INTERFACE_CONFIG_B_SINGLE_INST_MSK		NO_OS_BIT(7)
#define AD5770R_INTERFACE_CONFIG_B_SINGLE_INST(x)		(((x) & 0x1) << 7)
#define AD5770R_INTERFACE_CONFIG_B_SHORT_INST_MSK		NO_OS_BIT(3)
#define AD5770R_INTERFACE_CONFIG_B_SHORT_INST(x)		(((x) & 0x1) << 3)

/* AD5770R_INTERFACE_CONFIG_C */
#define AD5770R_INTERFACE_CONFIG_C_STRUCT_REGISTER_ACCESS_MSK	NO_OS_BIT(5)
#define AD5770R_INTERFACE_CONFIG_C_STRUCT_REGISTER_ACCESS(x)	(((x) & 0x1) << 5)

/* AD5770R_CHANNEL_CONFIG */
#define AD5770R_CHANNEL_CONFIG_CH0_SINK_EN_MSK			NO_OS_BIT(7)
#define AD5770R_CHANNEL_CONFIG_CH0_SINK_EN(x)			(((x) & 0x1) << 7)
#define AD5770R_CHANNEL_CONFIG_CH5_SHUTDOWN_B_MSK		NO_OS_BIT(5)
#define AD5770R_CHANNEL_CONFIG_CH5_SHUTDOWN_B(x)		(((x) & 0x1) << 5)
#define AD5770R_CHANNEL_CONFIG_CH4_SHUTDOWN_B_MSK		NO_OS_BIT(4)
#define AD5770R_CHANNEL_CONFIG_CH4_SHUTDOWN_B(x)		(((x) & 0x1) << 4)
#define AD5770R_CHANNEL_CONFIG_CH3_SHUTDOWN_B_MSK		NO_OS_BIT(3)
#define AD5770R_CHANNEL_CONFIG_CH3_SHUTDOWN_B(x)		(((x) & 0x1) << 3)
#define AD5770R_CHANNEL_CONFIG_CH2_SHUTDOWN_B_MSK		NO_OS_BIT(2)
#define AD5770R_CHANNEL_CONFIG_CH2_SHUTDOWN_B(x)		(((x) & 0x1) << 2)
#define AD5770R_CHANNEL_CONFIG_CH1_SHUTDOWN_B_MSK		NO_OS_BIT(1)
#define AD5770R_CHANNEL_CONFIG_CH1_SHUTDOWN_B(x)		(((x) & 0x1) << 1)
#define AD5770R_CHANNEL_CONFIG_CH0_SHUTDOWN_B_MSK		NO_OS_BIT(0)
#define AD5770R_CHANNEL_CONFIG_CH0_SHUTDOWN_B(x)		(((x) & 0x1) << 0)

/* AD5770R_OUTPUT_RANGE */
#define AD5770R_OUTPUT_RANGE_OUTPUT_SCALING_MSK			NO_OS_GENMASK(7, 2)
#define AD5770R_OUTPUT_RANGE_OUTPUT_SCALING(x)			(((x) & 0x3F) << 2)
#define AD5770R_OUTPUT_RANGE_MODE_MSK				NO_OS_GENMASK(1, 0)
#define AD5770R_OUTPUT_RANGE_MODE(x)				((x) & 0x03)

/* AD5770R_REFERENCE */
#define AD5770R_REFERENCE_RESISTOR_SEL_MSK			NO_OS_BIT(2)
#define AD5770R_REFERENCE_RESISTOR_SEL(x)			(((x) & 0x1) << 2)
#define AD5770R_REFERENCE_VOLTATE_SEL_MSK			NO_OS_GENMASK(1, 0)
#define AD5770R_REFERENCE_VOLTATE_SEL(x)			(((x) & 0x3) << 0)

/* AD5770R_ALARM_CONFIG */
#define AD5770R_ALARM_CONFIG_BACKGROUND_CRC_ALARM_MASK(x)	(((x) & 0x1) << 7)
#define AD5770R_ALARM_CONFIG_IREF_FAULT_ALARM_MASK(x)		(((x) & 0x1) << 6)
#define AD5770R_ALARM_CONFIG_NEGATIVE_CHANNEL0_ALARM_MASK(x)	(((x) & 0x1) << 5)
#define AD5770R_ALARM_CONFIG_OVER_TEMP_ALARM_MASK(x)		(((x) & 0x1) << 4)
#define AD5770R_ALARM_CONFIG_TEMP_WARNING_ALARM_MASK(x)		(((x) & 0x1) << 3)
#define AD5770R_ALARM_CONFIG_BACKGROUND_CRC_EN(x)		(((x) & 0x1) << 2)
#define AD5770R_ALARM_CONFIG_THERMAL_SHUTDOWN_EN(x)		(((x) & 0x1) << 1)
#define AD5770R_ALARM_CONFIG_OPEN_DRAIN_EN(x)			(((x) & 0x1) << 0)

/* AD5770R_OUTPUT_FILTER_CH */
#define AD5770R_OUTPUT_FILTER_CH_MSK				NO_OS_GENMASK(3, 0)
#define AD5770R_OUTPUT_FILTER_CH(x)				(((x) & 0xF) << 0)

/* AD5770R_MONITOR_SETUP */
#define AD5770R_MONITOR_SETUP_MON_FUNCTION_MSK			NO_OS_GENMASK(7, 6)
#define AD5770R_MONITOR_SETUP_MON_FUNCTION(x)			(((x) & 0x3) << 6)
#define AD5770R_MONITOR_SETUP_MUX_BUFFER_MSK			NO_OS_BIT(5)
#define AD5770R_MONITOR_SETUP_MUX_BUFFER(x)			(((x) & 0x1) << 5)
#define AD5770R_MONITOR_SETUP_IB_EXT_EN_MSK			NO_OS_BIT(4)
#define AD5770R_MONITOR_SETUP_IB_EXT_EN(x)			(((x) & 0x1) << 4)
#define AD5770R_MONITOR_SETUP_MON_CH_MSK			NO_OS_GENMASK(3, 0)
#define AD5770R_MONITOR_SETUP_MON_CH(x)				(((x) & 0x7) << 0)

/* AD5770R_STATUS */
#define AD5770R_STATUS_BACKGROUND_CRC_STATUS_MSK		NO_OS_BIT(7)
#define AD5770R_STATUS_IREF_FAULT_MSK				NO_OS_BIT(3)
#define AD5770R_STATUS_NEGATIVE_CHANNEL0_MSK			NO_OS_BIT(2)
#define AD5770R_STATUS_OVER_TEMP_MSK				NO_OS_BIT(1)
#define AD5770R_STATUS_TEMP_WARNING_MSK				NO_OS_BIT(0)

/* AD5770R_HW_LDAC */
#define AD5770R_HW_LDAC_MASK_CH(x, channel)			(((x) & 0x1) << (channel))

/* AD5770R_CH_DAC */
#define AD5770R_CH_DAC_DATA_LSB(x)				(((x) & 0x3F) << 2)
#define AD5770R_CH_DAC_DATA_MSB(x)				(((x) & 0xFF) << 0)

/* AD5770R_CH_SELECT */
#define AD5770R_CH_SELECT_SEL_CH(x, channel)			(((x) & 0x1) << (channel))

/* AD5770R_CH_INPUT */
#define AD5770R_CH_DAC_INPUT_DATA_LSB(x)			(((x) & 0x3F) << 2)
#define AD5770R_CH_DAC_INPUT_DATA_MSB(x)			(((x) & 0xFF) << 0)

/* AD5770R_SW_LDAC */
#define AD5770R_SW_LDAC_CH(x, channel)				(((x) & 0x1) << (channel))


#define AD5770R_REG_READ(x)					(((x) & 0x7F) | 0x80)
#define AD5770R_REG_WRITE(x)					((x) & 0x7F)

/***************************************************************************//**
 * @brief The `ad5770r_output_filter_resistor` enumeration defines a set of
 * constants representing different resistance values for output filter
 * resistors in the AD5770R device. These resistors are used to configure
 * the output filtering characteristics of the device, allowing for
 * different levels of signal conditioning based on the selected
 * resistance value. Each enumerator corresponds to a specific resistance
 * value, ranging from 60 Ohms to 104 kOhms, which can be used to tailor
 * the output response of the device to specific application
 * requirements.
 *
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_60_OHM Represents a 60 Ohm output
 * filter resistor.
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_5_6_KOHM Represents a 5.6 kOhm output
 * filter resistor.
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_11_2_KOHM Represents an 11.2 kOhm
 * output filter resistor.
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_22_2_KOHM Represents a 22.2 kOhm output
 * filter resistor.
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_44_4_KOHM Represents a 44.4 kOhm output
 * filter resistor.
 * @param AD5770R_OUTPUT_FILTER_RESISTOR_104_KOHM Represents a 104 kOhm output
 * filter resistor.
 ******************************************************************************/
enum ad5770r_output_filter_resistor {
	AD5770R_OUTPUT_FILTER_RESISTOR_60_OHM = 0x0,
	AD5770R_OUTPUT_FILTER_RESISTOR_5_6_KOHM = 0x5,
	AD5770R_OUTPUT_FILTER_RESISTOR_11_2_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_22_2_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_44_4_KOHM,
	AD5770R_OUTPUT_FILTER_RESISTOR_104_KOHM,
};

/***************************************************************************//**
 * @brief The `ad5770r_monitor_function` enumeration defines the possible
 * monitoring functions available for the AD5770R device, including
 * options to disable monitoring or to enable monitoring of voltage,
 * current, or temperature. This enumeration is used to configure the
 * monitoring setup of the device, allowing for flexible monitoring
 * capabilities depending on the application requirements.
 *
 * @param AD5770R_DISABLE Represents the disabled state for the monitor
 * function.
 * @param AD5770R_VOLTAGE_MONITORING Enables voltage monitoring functionality.
 * @param AD5770R_CURRENT_MONITORING Enables current monitoring functionality.
 * @param AD5770R_TEMPERATURE_MONITORING Enables temperature monitoring
 * functionality.
 ******************************************************************************/
enum ad5770r_monitor_function {
	AD5770R_DISABLE = 0,
	AD5770R_VOLTAGE_MONITORING,
	AD5770R_CURRENT_MONITORING,
	AD5770R_TEMPERATURE_MONITORING
};

/***************************************************************************//**
 * @brief The `ad5770r_channels` enumeration defines a set of constants
 * representing the six available channels (CH0 to CH5) in the AD5770R
 * device, which is a digital-to-analog converter (DAC) from Analog
 * Devices. Each enumerator corresponds to a specific channel, allowing
 * for easy reference and manipulation of the channels within the
 * device's driver code.
 *
 * @param AD5770R_CH0 Represents channel 0 in the AD5770R device.
 * @param AD5770R_CH1 Represents channel 1 in the AD5770R device.
 * @param AD5770R_CH2 Represents channel 2 in the AD5770R device.
 * @param AD5770R_CH3 Represents channel 3 in the AD5770R device.
 * @param AD5770R_CH4 Represents channel 4 in the AD5770R device.
 * @param AD5770R_CH5 Represents channel 5 in the AD5770R device.
 ******************************************************************************/
enum ad5770r_channels {
	AD5770R_CH0 = 0,
	AD5770R_CH1,
	AD5770R_CH2,
	AD5770R_CH3,
	AD5770R_CH4,
	AD5770R_CH5
};

/***************************************************************************//**
 * @brief The `ad5770r_reference_voltage` enumeration defines the possible
 * reference voltage configurations for the AD5770R device, allowing
 * selection between internal and external reference voltages at
 * different levels and states of output enablement.
 *
 * @param AD5770R_EXT_REF_2_5_V Represents an external reference voltage of 2.5
 * volts.
 * @param AD5770R_INT_REF_1_25_V_OUT_ON Represents an internal reference voltage
 * of 1.25 volts with output enabled.
 * @param AD5770R_EXT_REF_1_25_V Represents an external reference voltage of
 * 1.25 volts.
 * @param AD5770R_INT_REF_1_25_V_OUT_OFF Represents an internal reference
 * voltage of 1.25 volts with output
 * disabled.
 ******************************************************************************/
enum ad5770r_reference_voltage {
	AD5770R_EXT_REF_2_5_V = 0,
	AD5770R_INT_REF_1_25_V_OUT_ON,
	AD5770R_EXT_REF_1_25_V,
	AD5770R_INT_REF_1_25_V_OUT_OFF
};

/***************************************************************************//**
 * @brief The `ad5770r_monitor_setup` structure is used to configure the
 * monitoring setup for the AD5770R device. It allows the user to specify
 * the type of monitoring function (e.g., voltage, current, or
 * temperature), whether to enable the multiplexer buffer, whether to
 * enable external bias current, and which channel to monitor. This
 * structure is essential for setting up the monitoring capabilities of
 * the AD5770R, which is a precision digital-to-analog converter (DAC)
 * used in various applications requiring precise control and monitoring
 * of analog signals.
 *
 * @param monitor_function Specifies the monitoring function to be used, such as
 * voltage, current, or temperature monitoring.
 * @param mux_buffer Indicates whether the multiplexer buffer is enabled.
 * @param ib_ext_en Indicates whether the external bias current is enabled.
 * @param monitor_channel Specifies the channel to be monitored.
 ******************************************************************************/
struct ad5770r_monitor_setup {
	enum ad5770r_monitor_function		monitor_function;
	bool 					mux_buffer;
	bool					ib_ext_en;
	enum ad5770r_channels			monitor_channel;
};

/***************************************************************************//**
 * @brief The `ad5770r_dac_page_mask` structure is used to define page masks for
 * the AD5770R device, specifically for the DAC data and input pages.
 * This structure contains two 16-bit fields, `dac_data_page_mask` and
 * `input_page_mask`, which are used to specify which pages are active or
 * selected for operations involving DAC data and input data,
 * respectively. This allows for flexible configuration and control over
 * the data handling in the AD5770R device.
 *
 * @param dac_data_page_mask A 16-bit mask for the DAC data page.
 * @param input_page_mask A 16-bit mask for the input page.
 ******************************************************************************/
struct ad5770r_dac_page_mask {
	uint16_t 		dac_data_page_mask;
	uint16_t		input_page_mask;
};

/***************************************************************************//**
 * @brief The `ad5770r_output_range` structure is used to define the output
 * range settings for the AD5770R device. It contains two members:
 * `output_scale`, which specifies the scaling factor applied to the
 * output, and `output_range_mode`, which determines the mode of the
 * output range. This structure is essential for configuring the output
 * characteristics of the device, allowing for precise control over the
 * output signal parameters.
 *
 * @param output_scale Represents the scaling factor for the output.
 * @param output_range_mode Indicates the mode of the output range.
 ******************************************************************************/
struct ad5770r_output_range {
	uint8_t		output_scale;
	uint8_t		output_range_mode;
};

/***************************************************************************//**
 * @brief The `ad5770r_device_spi_settings` structure is used to configure the
 * SPI communication settings for the AD5770R device. It includes options
 * for address ascension, single instruction mode for efficient multibyte
 * operations, and the length of the stream mode, allowing for flexible
 * and efficient data transfer configurations.
 *
 * @param addr_ascension Indicates whether address ascension is enabled for SPI
 * communication.
 * @param single_instruction Specifies if single instruction mode is used for
 * multibyte read/write operations.
 * @param stream_mode_length Defines the length of the stream mode for SPI
 * communication.
 ******************************************************************************/
struct ad5770r_device_spi_settings {
	bool			addr_ascension;
	bool			single_instruction; // for multibyte read/write
	uint8_t		stream_mode_length;
};

/***************************************************************************//**
 * @brief The `ad5770r_channel_switches` structure is used to manage the enable
 * states of six channels (en0 to en5) and the sink mode of channel 0
 * (sink0) in the AD5770R device. Each member is a boolean value that
 * indicates whether a particular channel is enabled or if channel 0 is
 * set to sink mode, providing a simple interface for controlling the
 * operational state of the device's channels.
 *
 * @param en0 Represents the enable state of channel 0.
 * @param en1 Represents the enable state of channel 1.
 * @param en2 Represents the enable state of channel 2.
 * @param en3 Represents the enable state of channel 3.
 * @param en4 Represents the enable state of channel 4.
 * @param en5 Represents the enable state of channel 5.
 * @param sink0 Indicates whether channel 0 is in sink mode.
 ******************************************************************************/
struct ad5770r_channel_switches {
	bool en0, en1, en2, en3, en4, en5, sink0;
};

/***************************************************************************//**
 * @brief The `ad5770r_alarm_cfg` structure is used to configure various alarm
 * settings for the AD5770R device. It includes boolean flags to enable
 * or disable specific alarm features such as open-drain configuration,
 * thermal shutdown, and background CRC checking. Additionally, it
 * provides masking options for different types of alarms, including
 * temperature warnings, over-temperature conditions, negative channel 0,
 * IREF faults, and background CRC errors. This structure allows for
 * fine-grained control over the alarm system of the AD5770R, enabling
 * users to tailor the alarm behavior to their specific application
 * needs.
 *
 * @param open_drain_en Enables or disables open-drain configuration for alarms.
 * @param thermal_shutdown_en Enables or disables thermal shutdown feature.
 * @param background_crc_en Enables or disables background CRC checking.
 * @param temp_warning_msk Masks or unmasks temperature warning alarms.
 * @param over_temp_msk Masks or unmasks over-temperature alarms.
 * @param neg_ch0_msk Masks or unmasks negative channel 0 alarms.
 * @param iref_fault_msk Masks or unmasks IREF fault alarms.
 * @param background_crc_msk Masks or unmasks background CRC alarms.
 ******************************************************************************/
struct ad5770r_alarm_cfg {
	bool open_drain_en;
	bool thermal_shutdown_en;
	bool background_crc_en;
	bool temp_warning_msk;
	bool over_temp_msk;
	bool neg_ch0_msk;
	bool iref_fault_msk;
	bool background_crc_msk;
};

/***************************************************************************//**
 * @brief The `ad5770r_dev` structure is a comprehensive representation of the
 * AD5770R device, encapsulating all necessary configurations and
 * settings for its operation. It includes SPI and optional GPIO
 * descriptors for communication and control, device-specific SPI
 * settings, and various configurations for channel switching, output
 * modes, and alarms. The structure also manages settings for external
 * references, output filters, and monitoring setups, along with
 * maintaining DAC and input values for six channels. This structure is
 * essential for initializing and controlling the AD5770R device in a
 * system.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_alarm_n Pointer to the GPIO descriptor for the alarm pin,
 * optional.
 * @param gpio_reset_n Pointer to the GPIO descriptor for the reset pin,
 * optional.
 * @param gpio_ldac_n Pointer to the GPIO descriptor for the LDAC pin, optional.
 * @param dev_spi_settings Structure containing SPI settings specific to the
 * device.
 * @param channel_config Configuration for channel switches.
 * @param output_mode Array of output range settings for six channels.
 * @param external_reference Boolean indicating if an external reference is
 * used.
 * @param reference_selector Enum to select the reference voltage.
 * @param alarm_config Configuration for alarm settings.
 * @param output_filter Array of output filter resistor settings for six
 * channels.
 * @param mon_setup Setup configuration for monitoring functions.
 * @param mask_hw_ldac Channel switches mask for hardware LDAC.
 * @param dac_value Array of DAC values for six channels.
 * @param page_mask Page mask settings for DAC and input data.
 * @param mask_channel_sel Channel switches mask for channel selection.
 * @param sw_ldac Channel switches for software LDAC.
 * @param input_value Array of input values for six channels.
 ******************************************************************************/
struct ad5770r_dev {
	/* SPI */
	struct no_os_spi_desc			*spi_desc;
	/* GPIO */
	/** note: the GPIOs are optional */
	struct no_os_gpio_desc			*gpio_alarm_n;
	struct no_os_gpio_desc			*gpio_reset_n;
	struct no_os_gpio_desc			*gpio_ldac_n;
	/* Device SPI Settings */
	struct ad5770r_device_spi_settings	dev_spi_settings;
	/* Device Settings */
	struct ad5770r_channel_switches		channel_config;
	struct ad5770r_output_range		output_mode[6];
	bool					external_reference;
	enum ad5770r_reference_voltage		reference_selector;
	struct ad5770r_alarm_cfg		alarm_config;
	enum ad5770r_output_filter_resistor	output_filter[6];
	struct ad5770r_monitor_setup		mon_setup;
	struct ad5770r_channel_switches		mask_hw_ldac;
	uint16_t				dac_value[6];
	struct ad5770r_dac_page_mask		page_mask;
	struct ad5770r_channel_switches		mask_channel_sel;
	struct ad5770r_channel_switches		sw_ldac;
	uint16_t				input_value[6];
};

/***************************************************************************//**
 * @brief The `ad5770r_init_param` structure is used to initialize and configure
 * the AD5770R device, a multi-channel DAC, with specific settings for
 * SPI communication, GPIO pins, device-specific SPI settings, channel
 * configurations, output modes, reference voltages, alarm
 * configurations, output filters, monitoring setups, and DAC/input
 * values. It provides a comprehensive setup for the device's operation,
 * allowing for detailed customization of its behavior and interaction
 * with external components.
 *
 * @param spi_init Initializes the SPI interface for communication.
 * @param gpio_alarm_n Pointer to GPIO initialization parameters for the alarm
 * pin.
 * @param gpio_reset_n Pointer to GPIO initialization parameters for the reset
 * pin.
 * @param gpio_ldac_n Pointer to GPIO initialization parameters for the LDAC
 * pin.
 * @param dev_spi_settings Holds the SPI settings specific to the AD5770R
 * device.
 * @param channel_config Configures the channel switches for the device.
 * @param output_mode Defines the output range and scaling for each of the 6
 * channels.
 * @param external_reference Indicates whether an external reference voltage is
 * used.
 * @param reference_selector Selects the reference voltage configuration.
 * @param alarm_config Configures the alarm settings for the device.
 * @param output_filter Specifies the output filter resistor for each of the 6
 * channels.
 * @param mon_setup Sets up the monitoring function of the device.
 * @param mask_hw_ldac Configures the hardware LDAC mask for the channels.
 * @param dac_value Stores the DAC values for each of the 6 channels.
 * @param page_mask Defines the page mask for DAC and input data.
 * @param mask_channel_sel Configures the channel selection mask.
 * @param sw_ldac Configures the software LDAC for the channels.
 * @param input_value Stores the input values for each of the 6 channels.
 ******************************************************************************/
struct ad5770r_init_param {
	/* SPI */
	struct no_os_spi_init_param			spi_init;
	/* GPIO */
	struct no_os_gpio_init_param			*gpio_alarm_n;
	struct no_os_gpio_init_param			*gpio_reset_n;
	struct no_os_gpio_init_param			*gpio_ldac_n;
	/* Device SPI Settings */
	struct ad5770r_device_spi_settings	dev_spi_settings;
	/* Device Settings */
	struct ad5770r_channel_switches		channel_config;
	struct ad5770r_output_range		output_mode[6];
	bool					external_reference;
	enum ad5770r_reference_voltage		reference_selector;
	struct ad5770r_alarm_cfg		alarm_config;
	enum ad5770r_output_filter_resistor	output_filter[6];
	struct ad5770r_monitor_setup		mon_setup;
	struct ad5770r_channel_switches		mask_hw_ldac;
	uint16_t				dac_value[6];
	struct ad5770r_dac_page_mask		page_mask;
	struct ad5770r_channel_switches		mask_channel_sel;
	struct ad5770r_channel_switches		sw_ldac;
	uint16_t				input_value[6];
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief Use this function to read a single byte from a specified register of
 * the AD5770R device using SPI communication. It is essential to ensure
 * that the device has been properly initialized and that the provided
 * pointers are valid before calling this function. The function will
 * return an error code if the device or data pointer is null, or if the
 * SPI communication fails. This function is typically used when you need
 * to retrieve configuration or status information from the device.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD5770R.
 * @param reg_data A pointer to a uint8_t where the read data will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null, the reg_data pointer is null, or if the SPI communication
 * fails.
 ******************************************************************************/
int32_t ad5770r_spi_reg_read(struct ad5770r_dev *dev,
			     uint8_t reg_addr,
			     uint8_t *reg_data);
/***************************************************************************//**
 * @brief Use this function to read a sequence of registers from the AD5770R
 * device over SPI. It is essential to ensure that the device structure
 * and the data buffer are valid and properly initialized before calling
 * this function. The function reads the specified number of bytes
 * starting from the given register address and stores the result in the
 * provided buffer. It is important to handle the return value to check
 * for any errors during the SPI communication. This function should be
 * called only after the device has been successfully initialized.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The starting register address from which to begin reading.
 * Must be a valid register address for the AD5770R device.
 * @param reg_data A pointer to a buffer where the read data will be stored. The
 * buffer must be large enough to hold 'count' bytes. Must not
 * be null.
 * @param count The number of bytes to read from the device. Must be greater
 * than zero.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * an issue with SPI communication or invalid input parameters.
 ******************************************************************************/
int32_t ad5770r_spi_reg_read_multiple(struct ad5770r_dev *dev,
				      uint8_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count);
/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specified
 * register on the AD5770R device via SPI communication. It is essential
 * to ensure that the device structure is properly initialized and not
 * null before calling this function. This function is typically used
 * when configuring the device or updating its settings. If the device
 * pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null. The function returns -1 if this
 * parameter is null.
 * @param reg_addr The address of the register to which data will be written. It
 * is an 8-bit unsigned integer.
 * @param reg_data The data to be written to the specified register. It is an
 * 8-bit unsigned integer.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad5770r_spi_reg_write(struct ad5770r_dev *dev,
			      uint8_t reg_addr,
			      uint8_t reg_data);
/***************************************************************************//**
 * @brief This function is used to write a sequence of bytes to a specific
 * register on the AD5770R device using SPI communication. It is
 * essential to ensure that the device structure and the data buffer are
 * valid before calling this function. The function allocates memory to
 * prepare the data for transmission, writes the data to the device, and
 * then frees the allocated memory. It is important to handle the return
 * value to check for successful execution or errors.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to which data will be written.
 * Must be a valid register address for the AD5770R device.
 * @param reg_data A pointer to the data buffer containing the bytes to be
 * written. Must not be null and should contain at least 'count'
 * bytes.
 * @param count The number of bytes to write from the reg_data buffer to the
 * device. Must be greater than zero.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A negative value indicates an error.
 ******************************************************************************/
int32_t ad5770r_spi_reg_write_multiple(struct ad5770r_dev *dev,
				       uint8_t reg_addr,
				       uint8_t *reg_data,
				       uint16_t count);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * AD5770R device by applying a mask and writing new data to those bits.
 * It is useful when only certain bits of a register need to be updated
 * without affecting the other bits. The function first reads the current
 * value of the register, applies the mask to clear the bits to be
 * modified, and then writes the new data to those bits. It must be
 * called with a valid device structure pointer, and the device should be
 * properly initialized before calling this function. If the device
 * pointer is null, the function returns an error.
 *
 * @param dev Pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD5770R device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the masked bits of the register.
 * Only the bits specified by the mask will be updated.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad5770r_spi_write_mask(struct ad5770r_dev *dev,
			       uint8_t reg_addr,
			       uint32_t mask,
			       uint8_t data);
/***************************************************************************//**
 * @brief Use this function to set the SPI communication parameters for an
 * AD5770R device. It must be called with valid device and SPI settings
 * structures. The function updates the device's internal SPI
 * configuration registers and stores the new settings in the device
 * structure. It returns an error code if the device or settings are
 * null, or if any SPI write operation fails.
 *
 * @param dev A pointer to an ad5770r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param spi_settings A pointer to an ad5770r_device_spi_settings structure
 * containing the desired SPI settings. Must not be null.
 * The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the device or
 * settings are null, or if any SPI write operation fails.
 ******************************************************************************/
int32_t ad5770r_set_device_spi(struct ad5770r_dev *dev,
			       const struct ad5770r_device_spi_settings *spi_settings);
/***************************************************************************//**
 * @brief This function is used to configure the channel settings of an AD5770R
 * device by enabling or disabling specific channels and setting the sink
 * mode for channel 0. It should be called when the channel configuration
 * needs to be updated. The function requires a valid device structure
 * and a channel configuration structure. If either parameter is null,
 * the function returns an error code. Upon successful execution, the
 * device's channel configuration is updated to reflect the new settings.
 *
 * @param dev A pointer to an ad5770r_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param channel_config A pointer to an ad5770r_channel_switches structure
 * containing the desired channel settings. Must not be
 * null. The caller retains ownership.
 * @return Returns 0 on success or a negative error code if the operation fails,
 * such as when input parameters are invalid.
 ******************************************************************************/
int32_t ad5770r_channel_config(struct ad5770r_dev *dev,
			       const struct ad5770r_channel_switches *channel_config);
/***************************************************************************//**
 * @brief This function configures the output mode for a specific channel on the
 * AD5770R device. It should be called when you need to set or change the
 * output range and scaling for a particular channel. Ensure that the
 * device has been properly initialized before calling this function. The
 * function requires valid pointers for the device and output mode
 * structures, and it will return an error if these are null. The channel
 * parameter must be a valid channel enumeration value.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param output_mode A pointer to an ad5770r_output_range structure specifying
 * the desired output scale and range mode. Must not be null.
 * @param channel An enumeration value of type ad5770r_channels indicating the
 * channel to configure. Must be a valid channel.
 * @return Returns 0 on success or -1 if the device or output_mode pointers are
 * null.
 ******************************************************************************/
int32_t ad5770r_set_output_mode(struct ad5770r_dev *dev,
				const struct ad5770r_output_range *output_mode,
				enum ad5770r_channels channel);
/***************************************************************************//**
 * @brief This function sets the reference voltage source for the AD5770R
 * device, allowing the user to select between an internal or external
 * reference. It must be called with a valid device structure that has
 * been properly initialized. The function updates the device's
 * configuration to use the specified reference voltage and records the
 * selection in the device structure. It returns an error code if the
 * device structure is null or if the SPI write operation fails.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param external_reference A boolean value indicating whether to use an
 * external reference (true) or an internal reference
 * (false).
 * @param reference_selector An enum value of type ad5770r_reference_voltage
 * specifying the desired reference voltage
 * configuration. Valid values are defined in the
 * ad5770r_reference_voltage enumeration.
 * @return Returns an int32_t error code: 0 for success, -1 if the device
 * structure is null, or a non-zero error code if the SPI write
 * operation fails.
 ******************************************************************************/
int32_t ad5770r_set_reference(struct ad5770r_dev *dev,
			      bool external_reference,
			      enum ad5770r_reference_voltage reference_selector);
/***************************************************************************//**
 * @brief Use this function to set the alarm configuration for an AD5770R
 * device. It must be called with a valid device structure and a properly
 * initialized alarm configuration structure. This function writes the
 * alarm settings to the device and updates the device's internal
 * configuration. It is essential to ensure that both the device and the
 * alarm configuration pointers are not null before calling this
 * function, as null pointers will result in an error. The function
 * returns an error code if the operation fails, allowing the caller to
 * handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param alarm_config A pointer to a const ad5770r_alarm_cfg structure
 * containing the desired alarm settings. Must not be null.
 * The caller retains ownership.
 * @return Returns an int32_t value: 0 on success, or a negative error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad5770r_set_alarm(struct ad5770r_dev *dev,
			  const struct ad5770r_alarm_cfg *const alarm_config);
/***************************************************************************//**
 * @brief This function configures the output filter resistor for a specific
 * channel on the AD5770R device. It should be called when you need to
 * adjust the output filtering characteristics of a channel. Ensure that
 * the device structure is properly initialized before calling this
 * function. If the device pointer is null, the function will return an
 * error. This function is useful for applications requiring specific
 * output filtering to match the load or signal requirements.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure. Must not be
 * null. The function returns -1 if this parameter is null.
 * @param output_filter An enum value of type ad5770r_output_filter_resistor
 * specifying the desired output filter resistor setting.
 * Valid values are defined in the
 * ad5770r_output_filter_resistor enumeration.
 * @param channel An enum value of type ad5770r_channels specifying the channel
 * to configure. Valid values are AD5770R_CH0 through
 * AD5770R_CH5.
 * @return Returns 0 on success or -1 if the device pointer is null.
 ******************************************************************************/
int32_t ad5770r_set_output_filter(struct ad5770r_dev *dev,
				  enum ad5770r_output_filter_resistor output_filter,
				  enum ad5770r_channels channel);
/***************************************************************************//**
 * @brief This function sets the hardware LDAC (Load DAC) mask for the AD5770R
 * device, which controls the loading of DAC registers for specific
 * channels. It should be called when you need to update the LDAC
 * configuration to control which channels are affected by hardware LDAC
 * signals. Ensure that the device has been properly initialized before
 * calling this function. The function will return an error if either the
 * device or the mask_hw_ldac parameter is null.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param mask_hw_ldac A pointer to an ad5770r_channel_switches structure
 * specifying the LDAC mask for each channel. Must not be
 * null.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad5770r_set_hw_ldac(struct ad5770r_dev *dev,
			    const struct ad5770r_channel_switches *mask_hw_ldac);
/***************************************************************************//**
 * @brief This function sets the digital-to-analog converter (DAC) value for a
 * specified channel on the AD5770R device. It should be called when you
 * need to update the output voltage of a specific channel. The function
 * requires a valid device structure and a channel identifier. It updates
 * the internal state of the device to reflect the new DAC value. Ensure
 * the device is properly initialized before calling this function. If
 * the device pointer is null, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure. Must not be
 * null. The function returns -1 if this parameter is null.
 * @param dac_value A 16-bit unsigned integer representing the DAC value to set.
 * The valid range is determined by the device's
 * specifications.
 * @param channel An enum value of type ad5770r_channels indicating the channel
 * to set. Must be a valid channel identifier for the device.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad5770r_set_dac_value(struct ad5770r_dev *dev,
			      uint16_t dac_value, enum ad5770r_channels channel);
/***************************************************************************//**
 * @brief This function sets the input value for a specified channel on the
 * AD5770R device. It should be called when you need to update the DAC
 * input value for a particular channel. The function requires a valid
 * device structure and a channel enumeration to specify which channel's
 * input value to set. It is important to ensure that the device
 * structure is properly initialized before calling this function. If the
 * device structure is null, the function will return an error. The
 * function updates the internal state of the device to reflect the new
 * input value for the specified channel.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure. Must not be
 * null. The function will return an error if this parameter is null.
 * @param dac_input A 16-bit unsigned integer representing the input value to
 * set for the specified DAC channel. The valid range is
 * determined by the DAC's resolution and configuration.
 * @param channel An enumeration value of type ad5770r_channels specifying the
 * DAC channel to set the input value for. Must be a valid
 * channel enumeration value.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad5770r_set_dac_input(struct ad5770r_dev *dev,
			      uint16_t dac_input, enum ad5770r_channels channel);
/***************************************************************************//**
 * @brief This function configures the DAC and input page masks for the AD5770R
 * device, which are used to control which channels are affected by
 * subsequent operations. It should be called when you need to update the
 * page masks for the device. The function requires a valid device
 * structure and a page mask structure as inputs. If either input is
 * null, the function returns an error. The function communicates with
 * the device via SPI to set the page masks.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param page_mask A pointer to an ad5770r_dac_page_mask structure containing
 * the DAC and input page masks to be set. Must not be null.
 * The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if any input is invalid.
 ******************************************************************************/
int32_t ad5770r_set_page_mask(struct ad5770r_dev *dev,
			      const struct ad5770r_dac_page_mask *page_mask);
/***************************************************************************//**
 * @brief Use this function to configure which channels of the AD5770R device
 * are enabled or disabled by setting a mask. This function should be
 * called after the device has been initialized and when you need to
 * change the channel configuration. It updates the device's internal
 * state to reflect the new channel mask. Ensure that both the device and
 * the mask_channel_sel parameters are valid and not null before calling
 * this function.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param mask_channel_sel A pointer to an ad5770r_channel_switches structure
 * specifying the channel enable states. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the device or
 * mask_channel_sel is null, or if there is a failure in writing to the
 * device.
 ******************************************************************************/
int32_t ad5770r_set_mask_channel(struct ad5770r_dev *dev,
				 const struct ad5770r_channel_switches *mask_channel_sel);
/***************************************************************************//**
 * @brief This function sets the software load DAC (LDAC) configuration for the
 * AD5770R device, allowing selective updating of DAC channels. It should
 * be called when you need to update the LDAC settings to control which
 * channels are updated with new input values. The function requires a
 * valid device structure and a channel switches structure that specifies
 * the enable state for each channel. If either parameter is null, the
 * function returns an error code. Successful execution updates the
 * device's internal state to reflect the new LDAC settings.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param sw_ldac A pointer to an ad5770r_channel_switches structure specifying
 * which channels' LDAC settings to enable. Must not be null.
 * @return Returns 0 on success or a negative error code if the device or
 * channel switches are null, or if the SPI write operation fails.
 ******************************************************************************/
int32_t ad5770r_set_sw_ldac(struct ad5770r_dev *dev,
			    const struct ad5770r_channel_switches *sw_ldac);
/***************************************************************************//**
 * @brief Use this function to read the current status register value from an
 * AD5770R device. This function is typically called to check the
 * device's status for any alarms or operational conditions. Ensure that
 * the device has been properly initialized before calling this function.
 * The function will store the status register value in the provided
 * memory location. It is important to handle the return value to check
 * for any communication errors during the SPI read operation.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param status A pointer to a uint8_t variable where the status register value
 * will be stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad5770r_get_status(struct ad5770r_dev *dev,
			   uint8_t *status);
/***************************************************************************//**
 * @brief Use this function to obtain the current interface status of the
 * AD5770R device. It is typically called when you need to check the
 * status of the device's interface for diagnostic or monitoring
 * purposes. Ensure that the device has been properly initialized before
 * calling this function. The function reads the status from the device
 * and stores it in the provided status variable.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null.
 * @param status A pointer to a uint8_t variable where the interface status will
 * be stored. Must not be null.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad5770r_get_interface_status(struct ad5770r_dev *dev,
				     uint8_t *status);
/***************************************************************************//**
 * @brief This function sets up the monitoring configuration for the AD5770R
 * device, allowing the user to specify the monitoring function, channel,
 * and other related settings. It should be called after the device has
 * been initialized and before any monitoring operations are performed.
 * The function updates the device's internal state with the new monitor
 * setup configuration. If either the device or the monitor setup
 * structure is null, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad5770r_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param mon_setup A pointer to a const ad5770r_monitor_setup structure
 * containing the desired monitor configuration. Must not be
 * null. The caller retains ownership.
 * @return Returns 0 on success, or a negative error code if the device or
 * monitor setup is null, or if there is a failure in writing to the
 * device.
 ******************************************************************************/
int32_t ad5770r_set_monitor_setup(struct ad5770r_dev *dev,
				  const struct ad5770r_monitor_setup *mon_setup);
/***************************************************************************//**
 * @brief This function initializes the AD5770R device using the provided
 * initialization parameters, setting up the necessary SPI and GPIO
 * configurations. It must be called before any other operations on the
 * device. The function checks for the presence of the device by reading
 * its product ID and configures various device settings such as SPI,
 * channel configurations, reference settings, and alarms. If
 * initialization fails at any step, it performs cleanup and returns an
 * error code.
 *
 * @param device A pointer to a pointer of type `struct ad5770r_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ad5770r_init_param` containing the
 * initialization parameters for the device. Must not be null.
 * The structure should be properly populated with valid
 * configuration settings before calling this function.
 * @return Returns 0 on successful initialization, or a negative error code if
 * initialization fails at any step.
 ******************************************************************************/
int32_t ad5770r_init(struct ad5770r_dev **device,
		     const struct ad5770r_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly clean up and release all resources
 * associated with an AD5770R device instance when it is no longer
 * needed. This function should be called to prevent resource leaks after
 * the device has been initialized and used. It handles the removal of
 * SPI and GPIO resources and frees the memory allocated for the device
 * structure. Ensure that the device pointer is valid and initialized
 * before calling this function.
 *
 * @param dev A pointer to the ad5770r_dev structure representing the device
 * instance to be removed. Must not be null. If the pointer is null,
 * the function returns an error code.
 * @return Returns 0 on successful removal of the device resources, or -1 if an
 * error occurs, such as when the device pointer is null or if there is
 * an error in removing associated resources.
 ******************************************************************************/
int32_t ad5770r_remove(struct ad5770r_dev *dev);

#endif /* AD5770R_H_ */
