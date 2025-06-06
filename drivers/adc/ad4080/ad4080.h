/***************************************************************************//**
 *   @file   ad4080.h
 *   @brief  Header file of AD4080 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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
*******************************************************************************/

#ifndef __AD4080_H__
#define __AD4080_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/** Register Definition */
#define AD4080_REG_INTERFACE_CONFIG_A		0x0000
#define AD4080_REG_INTERFACE_CONFIG_B		0x0001
#define AD4080_REG_DEVICE_CONFIG		0x0002
#define AD4080_REG_CHIP_TYPE			0x0003
#define AD4080_REG_PRODUCT_ID_L			0x0004
#define AD4080_REG_PRODUCT_ID_H			0x0005
#define AD4080_REG_CHIP_GRADE			0x0006
#define AD4080_REG_SCRATCH_PAD			0x000A
#define AD4080_REG_SPI_REVISION			0x000B
#define AD4080_REG_VENDOR_L			0x000C
#define AD4080_REG_VENDOR_H			0x000D
#define AD4080_REG_STREAM_MODE			0x000E
#define AD4080_REG_TRANSFER_CONFIG		0x000F
#define AD4080_REG_INTERFACE_CONFIG_C		0x0010
#define AD4080_REG_INTERFACE_STATUS_A		0x0011
#define AD4080_REG_DEVICE_STATUS		0x0014
#define AD4080_REG_DATA_INTF_CONFIG_A		0x0015
#define AD4080_REG_DATA_INTF_CONFIG_B		0x0016
#define AD4080_REG_DATA_INTF_CONFIG_C		0x0017
#define AD4080_REG_PWR_CTRL			0x0018
#define AD4080_REG_GPIO_CONFIG_A		0x0019
#define AD4080_REG_GPIO_CONFIG_B		0x001A
#define AD4080_REG_GPIO_CONFIG_C		0x001B
#define AD4080_REG_GENERAL_CONFIG		0x001C
#define AD4080_REG_FIFO_WATERMARK		0x001D
#define AD4080_REG_EVENT_HYSTERESIS		0x001F
#define AD4080_REG_EVENT_DETECTION_HI		0x0021
#define AD4080_REG_EVENT_DETECTION_LO		0x0023
#define AD4080_REG_OFFSET			0x0025
#define AD4080_REG_GAIN				0x0027
#define AD4080_REG_FILTER_CONFIG		0x0029

/** AD4080_REG_INTERFACE_CONFIG_A Bit Definition */
#define AD4080_SW_RESET_MSK			NO_OS_BIT(7) | NO_OS_BIT(0)
#define AD4080_ADDR_ASC_MSK			NO_OS_BIT(5)
#define AD4080_SDO_ENABLE_MSK			NO_OS_BIT(4)

/** AD4080_REG_INTERFACE_CONFIG_B Bit Definition */
#define AD4080_SINGLE_INST_MSK			NO_OS_BIT(7)
#define AD4080_SHORT_INST_MSK			NO_OS_BIT(3)

/** AD4080_REG_DEVICE_CONFIG Bit Definition */
#define AD4080_OP_MODE_MSK			NO_OS_GENMASK(1, 0)

/** AD4080_REG_TRANSFER_CONFIG Bit Definition */
#define AD4080_KEEP_STREAM_LEN_VAL_MSK		NO_OS_BIT(2)

/** AD4080_REG_INTERFACE_CONFIG_C Bit Definition */
#define AD4080_STRICT_REG_ACCESS_MSK		NO_OS_BIT(5)

/** AD4080_REG_DATA_INTF_CONFIG_A Bit Definition */
#define AD4080_INTF_CHK_EN_MSK			NO_OS_BIT(4)
#define AD4080_SPI_LVDS_LANES_MSK		NO_OS_BIT(2)
#define AD4080_DATA_INTF_MODE_MSK		NO_OS_BIT(0)

/** AD4080_REG_DATA_INTF_CONFIG_B Bit Definition */
#define AD4080_LVDS_CNV_CLK_CNT_MSK		NO_OS_GENMASK(7, 4)
#define AD4080_LVDS_SELF_CLK_MODE_MSK		NO_OS_BIT(3)
#define AD4080_LVDS_CNV_EN			NO_OS_BIT(0)

/** AD4080_REG_DATA_INTF_CONFIG_C Bit Definition */
#define AD4080_LVDS_VOD_MSK			NO_OS_GENMASK(6, 4)

/** AD4080_REG_PWR_CTRL Bit Definition */
#define AD4080_ANA_DIG_LDO_PD_MSK		NO_OS_BIT(1)
#define AD4080_INTF_LDO_PD_MSK			NO_OS_BIT(0)

/** AD4080_GPIO_CONFIG Bit Defintion  */
#define AD4080_GPIO_EN_MSK(x)			NO_OS_BIT(x)
#define AD4080_GPIO_SEL_MSK(x)			(NO_OS_GENMASK(3, 0) << (4 * ((x)%2)))
#define AD4080_GPIO_DATA_MSK(x)			(NO_OS_BIT(x) << 4)

/** AD4080_REG_GENERAL_CONFIG Bit Definition */
#define AD4080_FIFO_MODE_MSK			NO_OS_GENMASK(1, 0)

/** Miscellaneous Definitions */
#define AD4080_SW_RESET				NO_OS_BIT(7) | NO_OS_BIT(0)
#define AD4080_SPI_READ          		NO_OS_BIT(7)
#define BYTE_ADDR_H				NO_OS_GENMASK(15, 8)
#define BYTE_ADDR_L				NO_OS_GENMASK(7, 0)
#define AD4080_CHIP_ID				NO_OS_GENMASK(2, 0)
#define AD4080_FIFO_SIZE		  NO_OS_BIT(14)

/******************************************************************************/
/************************ Types Declarations **********************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad4080_addr_asc` is an enumeration that defines the sequential
 * addressing behavior for the AD4080 device. It specifies whether the
 * address should increment or decrement during operations, which is
 * crucial for configuring how the device accesses its registers in a
 * sequence. This enum is used to control the address ascension mode in
 * the device's configuration.
 *
 * @param AD4080_ADDR_DECR Represents the decrementing address sequence.
 * @param AD4080_ADDR_INCR Represents the incrementing address sequence.
 ******************************************************************************/
enum ad4080_addr_asc {
	AD4080_ADDR_DECR,
	AD4080_ADDR_INCR,
};

/***************************************************************************//**
 * @brief The `ad4080_single_instr` enumeration defines the modes of operation
 * for the AD4080 device, specifically distinguishing between streaming
 * mode and single instruction mode. This enumeration is used to
 * configure the device's operational behavior in terms of data handling
 * and processing.
 *
 * @param AD4080_STREAM_MODE Represents the streaming mode of operation for the
 * AD4080 device.
 * @param AD4080_SINGLE_INST Represents the single instruction mode of operation
 * for the AD4080 device.
 ******************************************************************************/
enum ad4080_single_instr {
	AD4080_STREAM_MODE,
	AD4080_SINGLE_INST,
};

/***************************************************************************//**
 * @brief The `ad4080_short_instr` is an enumeration that defines the possible
 * short instruction address modes for the AD4080 device. It provides two
 * options: a 15-bit address mode and a 7-bit address mode, which are
 * used to specify the addressing scheme for communication with the
 * device. This enumeration is part of the configuration settings for the
 * AD4080, allowing users to select the appropriate address length for
 * their application.
 *
 * @param AD4080_15_BIT_ADDR Represents a 15-bit address instruction for the
 * AD4080 device.
 * @param AD4080_7_BIT_ADDR Represents a 7-bit address instruction for the
 * AD4080 device.
 ******************************************************************************/
enum ad4080_short_instr {
	AD4080_15_BIT_ADDR,
	AD4080_7_BIT_ADDR,
};

/***************************************************************************//**
 * @brief The `ad4080_op_mode` enumeration defines the different operational
 * modes available for the AD4080 device, which include normal, standby,
 * and low power modes. These modes allow the device to operate under
 * different power and performance conditions, enabling flexibility in
 * power management and operational efficiency.
 *
 * @param AD4080_OP_NORMAL Represents the normal operation mode of the AD4080
 * device.
 * @param AD4080_OP_STANDBY Represents the standby operation mode of the AD4080
 * device.
 * @param AD4080_OP_LOW_POWER Represents the low power operation mode of the
 * AD4080 device.
 ******************************************************************************/
enum ad4080_op_mode {
	AD4080_OP_NORMAL,
	AD4080_OP_STANDBY,
	AD4080_OP_LOW_POWER,
};

/***************************************************************************//**
 * @brief The `ad4080_strict_reg_access` enumeration defines two modes of
 * register access for the AD4080 device: normal mode and strict mode.
 * This enumeration is used to configure how the device's registers are
 * accessed, potentially affecting the security and integrity of the
 * register operations.
 *
 * @param AD4080_REG_NORMAL_MODE Represents the normal mode of register access.
 * @param AD4080_REG_STRICT_MODE Represents the strict mode of register access.
 ******************************************************************************/
enum ad4080_strict_reg_access {
	AD4080_REG_NORMAL_MODE,
	AD4080_REG_STRICT_MODE,
};

/***************************************************************************//**
 * @brief The `ad4080_intf_chk_en` enumeration defines the possible output
 * patterns for the AD4080 interface, allowing the selection between
 * normal data output and a fixed pattern output. This is useful for
 * configuring the device's interface behavior, particularly in testing
 * or diagnostic scenarios where a known output pattern is required.
 *
 * @param AD4080_DATA Represents the data output pattern for the AD4080
 * interface.
 * @param AD4080_FIXED_PATTERN Represents a fixed pattern output for the AD4080
 * interface.
 ******************************************************************************/
enum ad4080_intf_chk_en {
	AD4080_DATA,
	AD4080_FIXED_PATTERN,
};

/***************************************************************************//**
 * @brief The `ad4080_cnv_spi_lvds_lanes` enumeration defines the possible
 * configurations for the number of LVDS/SPI lanes used in the AD4080
 * device. It allows the user to specify whether the device should
 * operate with a single lane or multiple lanes, which can affect the
 * data transfer capabilities and performance of the device.
 *
 * @param AD4080_ONE_LANE Represents a configuration with a single LVDS/SPI
 * lane.
 * @param AD4080_MULTIPLE_LANES Represents a configuration with multiple
 * LVDS/SPI lanes.
 ******************************************************************************/
enum ad4080_cnv_spi_lvds_lanes {
	AD4080_ONE_LANE,
	AD4080_MULTIPLE_LANES
};

/***************************************************************************//**
 * @brief The `ad4080_conv_data_spi_lvds` enumeration defines the possible data
 * interface configurations for the AD4080 device, specifically
 * distinguishing between LVDS and SPI modes. This enumeration is used to
 * configure how the device communicates data, allowing for flexibility
 * in interfacing with different systems or components that may require
 * either LVDS or SPI communication protocols.
 *
 * @param AD4080_CONV_DATA_LVDS Represents the LVDS (Low Voltage Differential
 * Signaling) data interface configuration.
 * @param AD4080_CONV_DATA_SPI Represents the SPI (Serial Peripheral Interface)
 * data interface configuration.
 ******************************************************************************/
enum ad4080_conv_data_spi_lvds {
	AD4080_CONV_DATA_LVDS,
	AD4080_CONV_DATA_SPI
};

/***************************************************************************//**
 * @brief The `ad4080_lvds_self_clk_mode` is an enumeration that defines the
 * possible clock modes for the AD4080's LVDS self clock configuration.
 * It includes two modes: `AD4080_ECHO_CLK_MODE`, which uses an echo
 * clock, and `AD4080_SELF_CLK_MODE`, which uses a self-generated clock.
 * This enumeration is used to configure the clocking behavior of the
 * AD4080 device in applications where LVDS signaling is employed.
 *
 * @param AD4080_ECHO_CLK_MODE Represents the echo clock mode for the AD4080
 * LVDS self clock.
 * @param AD4080_SELF_CLK_MODE Represents the self clock mode for the AD4080
 * LVDS self clock.
 ******************************************************************************/
enum ad4080_lvds_self_clk_mode {
	AD4080_ECHO_CLK_MODE,
	AD4080_SELF_CLK_MODE,
};

/***************************************************************************//**
 * @brief The `ad4080_lvds_cnv_clk_mode` is an enumeration that defines the
 * clock modes for the AD4080 device's LVDS CNV (conversion) clock. It
 * provides two modes: CMOS and LVDS, allowing the user to select the
 * appropriate clock mode for their application. This enum is part of the
 * configuration settings for the AD4080 device, which is a high-speed
 * data converter.
 *
 * @param AD4080_CNV_CMOS_MODE Represents the CMOS mode for the AD4080 LVDS CNV
 * clock.
 * @param AD4080_CNV_LVDS_MODE Represents the LVDS mode for the AD4080 LVDS CNV
 * clock.
 ******************************************************************************/
enum ad4080_lvds_cnv_clk_mode {
	AD4080_CNV_CMOS_MODE,
	AD4080_CNV_LVDS_MODE,
};

/***************************************************************************//**
 * @brief The `ad4080_lvds_vod` enumeration defines the possible values for the
 * LVDS (Low Voltage Differential Signaling) differential output voltage
 * levels in the AD4080 device. Each enumerator corresponds to a specific
 * voltage level, which can be used to configure the device's output
 * characteristics for different signaling requirements.
 *
 * @param AD4080_185mVPP Represents a differential output voltage of 185 mV
 * peak-to-peak.
 * @param AD4080_240mVPP Represents a differential output voltage of 240 mV
 * peak-to-peak.
 * @param AD4080_325mVPP Represents a differential output voltage of 325 mV
 * peak-to-peak.
 ******************************************************************************/
enum ad4080_lvds_vod {
	AD4080_185mVPP = 1,
	AD4080_240mVPP = 2,
	AD4080_325mVPP = 4,
};

/***************************************************************************//**
 * @brief The `ad4080_ana_dig_ldo_pd` enumeration defines the power-down states
 * for the Analog/Digital Low Dropout (LDO) regulator in the AD4080
 * device. It provides two states: `AD4080_AD_LDO_EN` for enabling the
 * LDO and `AD4080_AD_LDO_DISABLE` for disabling it, allowing control
 * over the power management of the device's analog and digital sections.
 *
 * @param AD4080_AD_LDO_EN Represents the enabled state of the Analog/Digital
 * LDO.
 * @param AD4080_AD_LDO_DISABLE Represents the disabled state of the
 * Analog/Digital LDO.
 ******************************************************************************/
enum ad4080_ana_dig_ldo_pd {
	AD4080_AD_LDO_EN,
	AD4080_AD_LDO_DISABLE,
};

/***************************************************************************//**
 * @brief The `ad4080_intf_ldo_pd` enumeration defines the possible states for
 * the interface Low Dropout Regulator (LDO) in the AD4080 device. It
 * provides two states: `AD4080_INTF_LDO_EN` for enabling the LDO and
 * `AD4080_INTF_LDO_DISABLE` for disabling it. This enumeration is used
 * to control the power state of the interface LDO, which is crucial for
 * managing power consumption and operational modes of the device.
 *
 * @param AD4080_INTF_LDO_EN Represents the enabled state of the interface LDO.
 * @param AD4080_INTF_LDO_DISABLE Represents the disabled state of the interface
 * LDO.
 ******************************************************************************/
enum ad4080_intf_ldo_pd {
	AD4080_INTF_LDO_EN,
	AD4080_INTF_LDO_DISABLE,
};

/* AD4080 GPIOs */
/***************************************************************************//**
 * @brief The `ad4080_gpio` enumeration defines the available General Purpose
 * Input/Output (GPIO) pins for the AD4080 device, which includes four
 * distinct GPIO pins labeled from `AD4080_GPIO_0` to `AD4080_GPIO_3`.
 * Additionally, it includes a member `NUM_AD4080_GPIO` to represent the
 * total count of GPIO pins, facilitating iteration or validation
 * processes involving these pins.
 *
 * @param AD4080_GPIO_0 Represents the first GPIO pin in the AD4080 device.
 * @param AD4080_GPIO_1 Represents the second GPIO pin in the AD4080 device.
 * @param AD4080_GPIO_2 Represents the third GPIO pin in the AD4080 device.
 * @param AD4080_GPIO_3 Represents the fourth GPIO pin in the AD4080 device.
 * @param NUM_AD4080_GPIO Defines the total number of GPIO pins available in the
 * AD4080 device.
 ******************************************************************************/
enum ad4080_gpio {
	AD4080_GPIO_0,
	AD4080_GPIO_1,
	AD4080_GPIO_2,
	AD4080_GPIO_3,
	NUM_AD4080_GPIO
};

/* AD4080 GPIO Output Enable Selection */
/***************************************************************************//**
 * @brief The `ad4080_gpio_op_enable` enumeration defines the possible
 * operational states for GPIO pins on the AD4080 device, specifically
 * whether a GPIO pin is configured as an input or an output. This
 * enumeration is used to control the direction of data flow through the
 * GPIO pins, allowing for flexible configuration depending on the
 * application requirements.
 *
 * @param AD4080_GPIO_INPUT Represents the GPIO input mode.
 * @param AD4080_GPIO_OUTPUT Represents the GPIO output mode.
 ******************************************************************************/
enum ad4080_gpio_op_enable {
	AD4080_GPIO_INPUT,
	AD4080_GPIO_OUTPUT
};

/***************************************************************************//**
 * @brief The `ad4080_gpio_op_func_sel` is an enumeration that defines various
 * operational functions that can be assigned to the GPIO pins of the
 * AD4080 device. Each enumerator represents a specific function or
 * status indication that the GPIO can be configured to handle, such as
 * data output, FIFO status, filter results, threshold detection, and
 * external event triggers. This enumeration is used to configure the
 * GPIO pins to perform specific roles in the operation of the AD4080
 * device.
 *
 * @param AD4080_GPIO_ADI_NSPI_SDO_DATA Represents the ADI NSPI SDO data
 * function for the GPIO.
 * @param AD4080_GPIO_FIFO_FULL Indicates the FIFO full status for the GPIO.
 * @param AD4080_GPIO_FIFO_READ_DONE Indicates the FIFO read done status for the
 * GPIO.
 * @param AD4080_GPIO_FILTER_RESULT_READY Indicates the filter result ready
 * status for the GPIO.
 * @param AD4080_GPIO_HT_DETECT Represents the high threshold detect function
 * for the GPIO.
 * @param AD4080_GPIO_LT_DETECT Represents the low threshold detect function for
 * the GPIO.
 * @param AD4080_GPIO_STATUS_ALERT Indicates a status alert for the GPIO.
 * @param AD4080_GPIO_GPO_DATA Represents general-purpose output data for the
 * GPIO.
 * @param AD4080_GPIO_FILTER_SYNC_INPUT Represents the filter sync input
 * function for the GPIO.
 * @param AD4080_GPIO_EXT_EVENT_TRIGGER_FIFO Represents the external event
 * trigger for the FIFO function for
 * the GPIO.
 * @param AD4080_GPIO_CNV_INHIBIT_INPUT Represents the conversion inhibit input
 * function for the GPIO.
 ******************************************************************************/
enum ad4080_gpio_op_func_sel {
	AD4080_GPIO_ADI_NSPI_SDO_DATA,
	AD4080_GPIO_FIFO_FULL,
	AD4080_GPIO_FIFO_READ_DONE,
	AD4080_GPIO_FILTER_RESULT_READY,
	AD4080_GPIO_HT_DETECT,
	AD4080_GPIO_LT_DETECT,
	AD4080_GPIO_STATUS_ALERT,
	AD4080_GPIO_GPO_DATA,
	AD4080_GPIO_FILTER_SYNC_INPUT,
	AD4080_GPIO_EXT_EVENT_TRIGGER_FIFO,
	AD4080_GPIO_CNV_INHIBIT_INPUT
};

/***************************************************************************//**
 * @brief The `ad4080_fifo_mode` is an enumeration that defines the different
 * modes of operation for the FIFO (First In, First Out) buffer in the
 * AD4080 device. It provides options to disable the FIFO, enable it with
 * immediate triggering, or use event-based triggering with or without a
 * last watermark. This allows for flexible data handling and buffering
 * strategies depending on the application requirements.
 *
 * @param AD4080_FIFO_DISABLE Disables the FIFO mode.
 * @param AD4080_IMMEDIATE_TRIGGER Enables FIFO mode with immediate trigger.
 * @param AD4080_EVENT_TRIGGER_LAST_WM Enables FIFO mode with event trigger and
 * last watermark.
 * @param AD4080_EVENT_TRIGGER Enables FIFO mode with event trigger.
 ******************************************************************************/
enum ad4080_fifo_mode {
	AD4080_FIFO_DISABLE,
	AD4080_IMMEDIATE_TRIGGER,
	AD4080_EVENT_TRIGGER_LAST_WM,
	AD4080_EVENT_TRIGGER,
};

/***************************************************************************//**
 * @brief The `ad4080_dev` structure represents the configuration and state of
 * an AD4080 device, which is a high-speed ADC with various operational
 * modes and interface configurations. It includes members for SPI
 * communication, operational modes, interface configurations, and GPIO
 * settings, allowing for detailed control over the device's behavior and
 * data handling capabilities. The structure is designed to facilitate
 * the initialization, configuration, and operation of the AD4080 device
 * in various application scenarios.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param spi3wire Boolean indicating if SPI 3-wire connection is used.
 * @param addr_asc Enum for address ascension behavior.
 * @param single_instr Enum for single or streaming instruction mode.
 * @param short_instr Enum for short instruction addressing mode.
 * @param op_mode Enum for the operation mode of the device.
 * @param strict_reg Enum for strict register access mode.
 * @param intf_chk_en Enum for the output pattern of the device.
 * @param cnv_spi_lvds_lanes Enum for LVDS/SPI lane control.
 * @param conv_data_spi_lvds Enum for data interface configuration.
 * @param lvds_cnv_clk_cnt 8-bit integer for LVDS interface clock periods from
 * CNV rising edge.
 * @param lvds_self_clk_mode Enum for LVDS self clock mode.
 * @param cnv_clk_mode Enum for LVDS CNV clock mode.
 * @param lvds_vod Enum for LVDS differential output voltage.
 * @param ana_dig_ldo_pd Enum for analog/digital LDO power down.
 * @param intf_ldo_pd Enum for interface LDO power down.
 * @param fifo_mode Enum for conversion data FIFO mode.
 * @param gpio_op_enable Array of enums for GPIO output enable state.
 * @param gpio_op_func_sel Array of enums for GPIO output function selection.
 ******************************************************************************/
struct ad4080_dev {
	/* SPI */
	struct no_os_spi_desc	*spi_desc;
	/* SPI 3-Wire Connection */
	bool spi3wire;
	/** Address Ascension */
	enum ad4080_addr_asc addr_asc;
	/** Single/Streaming Mode */
	enum ad4080_single_instr single_instr;
	/** Short Instruction */
	enum ad4080_short_instr	short_instr;
	/** Operation Mode */
	enum ad4080_op_mode op_mode;
	/** Strict Register Access */
	enum ad4080_strict_reg_access strict_reg;
	/** AD4080 Output Pattern */
	enum ad4080_intf_chk_en intf_chk_en;
	/** AD4080 LVDS/SPI Lane Control. */
	enum ad4080_cnv_spi_lvds_lanes cnv_spi_lvds_lanes;
	/** AD4080 Data Interface Configuration */
	enum ad4080_conv_data_spi_lvds conv_data_spi_lvds;
	/** AD4080 Interface clock periods from CNV rising edge */
	uint8_t lvds_cnv_clk_cnt;
	/** AD4080 LVDS Self Clock Mode */
	enum ad4080_lvds_self_clk_mode lvds_self_clk_mode;
	/** AD4080 LVDS CNV Clock Mode */
	enum ad4080_lvds_cnv_clk_mode cnv_clk_mode;
	/** AD4080 LVDS Differential Output Voltage */
	enum ad4080_lvds_vod lvds_vod;
	/** AD4080 Analog/Digital LDO */
	enum ad4080_ana_dig_ldo_pd ana_dig_ldo_pd;
	/** AD4080 Interface LDO */
	enum ad4080_intf_ldo_pd intf_ldo_pd;
	/** AD4080 Conversion Data FIFO Mode */
	enum ad4080_fifo_mode fifo_mode;
	/** AD4080 GPIO Output Enable state */
	enum ad4080_gpio_op_enable gpio_op_enable[NUM_AD4080_GPIO];
	/** AD4080 GPIO Output Function Selection */
	enum ad4080_gpio_op_func_sel gpio_op_func_sel[NUM_AD4080_GPIO];
};

/***************************************************************************//**
 * @brief The `ad4080_init_param` structure is used to define the initialization
 * parameters for the AD4080 device. It includes various configuration
 * settings such as SPI initialization parameters, operation modes,
 * interface configurations, and GPIO settings. This structure allows for
 * detailed customization of the device's behavior and interface,
 * enabling it to be tailored to specific application requirements. The
 * structure's members cover a wide range of functionalities, from basic
 * SPI settings to advanced LVDS configurations and GPIO management.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param spi3wire Boolean indicating if SPI 3-wire connection is used.
 * @param addr_asc Enum for address ascension behavior.
 * @param single_instr Enum for single or streaming mode selection.
 * @param short_instr Enum for short instruction mode.
 * @param op_mode Enum for operation mode of the device.
 * @param strict_reg Enum for strict register access mode.
 * @param intf_chk_en Enum for AD4080 output pattern.
 * @param cnv_spi_lvds_lanes Enum for LVDS/SPI lane control.
 * @param conv_data_spi_lvds Enum for data interface configuration.
 * @param lvds_cnv_clk_cnt 8-bit integer for interface clock periods from CNV
 * rising edge.
 * @param lvds_self_clk_mode Enum for LVDS self clock mode.
 * @param cnv_clk_mode Enum for LVDS CNV clock mode.
 * @param lvds_vod Enum for LVDS differential output voltage.
 * @param ana_dig_ldo_pd Enum for analog/digital LDO power down.
 * @param intf_ldo_pd Enum for interface LDO power down.
 * @param fifo_mode Enum for conversion data FIFO mode.
 * @param gpio_op_enable Array of enums for GPIO output enable state.
 * @param gpio_op_func_sel Array of enums for GPIO output function selection.
 ******************************************************************************/
struct ad4080_init_param {
	/* SPI */
	struct no_os_spi_init_param	*spi_init;
	/* SPI 3-Wire Connection */
	bool spi3wire;
	/** Address Ascension */
	enum ad4080_addr_asc addr_asc;
	/** Single/Streaming Mode */
	enum ad4080_single_instr single_instr;
	/** Short Instruction */
	enum ad4080_short_instr	short_instr;
	/** Operation Mode */
	enum ad4080_op_mode op_mode;
	/** Strict Register Access */
	enum ad4080_strict_reg_access strict_reg;
	/** AD4080 Output Pattern */
	enum ad4080_intf_chk_en intf_chk_en;
	/** AD4080 LVDS/SPI Lane Control. */
	enum ad4080_cnv_spi_lvds_lanes cnv_spi_lvds_lanes;
	/** AD4080 Data Interface Configuration */
	enum ad4080_conv_data_spi_lvds conv_data_spi_lvds;
	/** AD4080 Interface clock periods from CNV rising edge */
	uint8_t lvds_cnv_clk_cnt;
	/** AD4080 LVDS Self Clock Mode */
	enum ad4080_lvds_self_clk_mode lvds_self_clk_mode;
	/** AD4080 LVDS CNV Clock Mode */
	enum ad4080_lvds_cnv_clk_mode cnv_clk_mode;
	/** AD4080 LVDS Differential Output Voltage */
	enum ad4080_lvds_vod lvds_vod;
	/** AD4080 Analog/Digital LDO */
	enum ad4080_ana_dig_ldo_pd ana_dig_ldo_pd;
	/** AD4080 Interface LDO */
	enum ad4080_intf_ldo_pd intf_ldo_pd;
	/** AD4080 Conversion Data FIFO Mode */
	enum ad4080_fifo_mode fifo_mode;
	/** AD4080 GPIO Output Enable state */
	enum ad4080_gpio_op_enable gpio_op_enable[NUM_AD4080_GPIO];
	/** AD4080 GPIO Output Function Selection */
	enum ad4080_gpio_op_func_sel gpio_op_func_sel[NUM_AD4080_GPIO];
};
/***************************************************************************//**
 * @brief This function is used to write a value to a specific register of the
 * AD4080 device via SPI communication. It requires a valid device
 * structure that has been properly initialized. The function constructs
 * a buffer containing the register address and the value to be written,
 * and then sends this buffer over SPI. It is essential to ensure that
 * the device structure is not null before calling this function, as a
 * null device structure will result in an error. This function is
 * typically used when configuring the device or updating its settings.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param reg_addr The 16-bit address of the register to which the value will be
 * written. The address should be within the valid range of the
 * device's register map.
 * @param reg_val The 8-bit value to write to the specified register. This value
 * will be sent to the device as part of the SPI transaction.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL if the device structure is null.
 ******************************************************************************/
int ad4080_write(struct ad4080_dev *dev, uint16_t reg_addr, uint8_t reg_val);

/***************************************************************************//**
 * @brief This function is used to read a byte of data from a specified register
 * address of the AD4080 device. It requires a valid device structure
 * pointer, a register address, and a pointer to store the read value.
 * The function must be called with a properly initialized device
 * structure. If the device pointer is null, the function returns an
 * error. The function communicates with the device over SPI and stores
 * the read value in the provided pointer. It is important to ensure that
 * the register address is valid and that the pointer for storing the
 * register value is not null.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address for the AD4080 device.
 * @param reg_val A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the SPI communication fails.
 ******************************************************************************/
int ad4080_read(struct ad4080_dev *dev, uint16_t reg_addr, uint8_t *reg_val);

/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * AD4080 device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets the new bits as
 * specified by the reg_val parameter. This function should be called
 * when there is a need to change specific settings in a register without
 * affecting other bits. It requires a valid device structure and a valid
 * register address. If the device structure is null, the function
 * returns an error. The function also handles any errors that occur
 * during the read or write operations.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the AD4080 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_val The new values for the bits specified by the mask. Only the
 * bits corresponding to the mask will be updated in the
 * register.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read or write operations.
 ******************************************************************************/
int ad4080_update_bits(struct ad4080_dev *dev, uint16_t reg_addr, uint8_t mask,
		       uint8_t reg_val);

/***************************************************************************//**
 * @brief Use this function to reset the AD4080 device to its default state via
 * a software command. This function should be called when a reset of the
 * device is required, such as after initialization or when the device is
 * not responding as expected. It is important to ensure that the device
 * structure is properly initialized and not null before calling this
 * function. The function will return an error code if the device pointer
 * is null or if the reset operation fails.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad4080_soft_reset(struct ad4080_dev *dev);

/***************************************************************************//**
 * @brief This function configures the address ascension mode of the AD4080
 * device, which determines how the device's internal address pointer
 * behaves during register access. It should be called when you need to
 * change the sequential addressing behavior of the device. The function
 * requires a valid device structure pointer and an address ascension
 * mode. If the device pointer is null, the function returns an error.
 * Successful execution updates the device's internal state to reflect
 * the new address ascension mode.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param addr_asc An enum value of type ad4080_addr_asc specifying the desired
 * address ascension mode. Valid values are AD4080_ADDR_DECR and
 * AD4080_ADDR_INCR.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if updating the device configuration fails.
 ******************************************************************************/
int ad4080_set_addr_asc(struct ad4080_dev *dev, enum ad4080_addr_asc addr_asc);

/***************************************************************************//**
 * @brief This function is used to obtain the current address ascension setting
 * of the AD4080 device. It should be called when you need to verify or
 * utilize the device's address increment or decrement configuration. The
 * function requires a valid device structure pointer and a pointer to an
 * enum where the result will be stored. It is important to ensure that
 * the device has been properly initialized before calling this function.
 * If the device pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param addr_asc A pointer to an enum ad4080_addr_asc where the current
 * address ascension setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error reading from the device.
 ******************************************************************************/
int ad4080_get_addr_asc(struct ad4080_dev *dev, enum ad4080_addr_asc *addr_asc);

/***************************************************************************//**
 * @brief This function configures the AD4080 device to operate in either single
 * instruction or streaming mode, as specified by the `single_instr`
 * parameter. It must be called with a valid device structure that has
 * been properly initialized. The function updates the device's
 * configuration register to reflect the desired mode and stores the mode
 * in the device structure for future reference. If the device pointer is
 * null, the function returns an error. This function is typically used
 * when setting up the device for specific operational requirements.
 *
 * @param dev A pointer to an initialized `ad4080_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error.
 * @param single_instr An enumeration value of type `enum ad4080_single_instr`
 * indicating the desired mode (either `AD4080_STREAM_MODE`
 * or `AD4080_SINGLE_INST`).
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if updating the device configuration fails.
 ******************************************************************************/
int ad4080_set_single_instr(struct ad4080_dev *dev,
			    enum ad4080_single_instr single_instr);

/***************************************************************************//**
 * @brief This function is used to obtain the current single instruction mode
 * setting of the AD4080 device. It should be called when you need to
 * verify or utilize the current mode setting of the device. The function
 * requires a valid device structure pointer and a pointer to an enum
 * where the result will be stored. It is important to ensure that the
 * device has been properly initialized before calling this function. If
 * the device pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function will return -EINVAL if this parameter is
 * null.
 * @param single_instr A pointer to an enum ad4080_single_instr where the
 * current single instruction mode will be stored. Must not
 * be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in reading the device register.
 ******************************************************************************/
int ad4080_get_single_instr(struct ad4080_dev *dev,
			    enum ad4080_single_instr *single_instr);

/***************************************************************************//**
 * @brief This function configures the short instruction mode of the AD4080
 * device by updating the relevant register. It should be called when you
 * need to change the addressing mode of the device to either 15-bit or
 * 7-bit. The function requires a valid device structure and a short
 * instruction mode to be specified. It returns an error if the device
 * structure is null or if the register update fails. The function also
 * updates the device structure to reflect the new short instruction
 * mode.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param short_instr An enum value of type ad4080_short_instr specifying the
 * desired short instruction mode. Valid values are
 * AD4080_15_BIT_ADDR and AD4080_7_BIT_ADDR.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if the register update fails.
 ******************************************************************************/
int ad4080_set_short_instr(struct ad4080_dev *dev,
			   enum ad4080_short_instr short_instr);

/***************************************************************************//**
 * @brief This function is used to obtain the current short instruction mode
 * setting from an AD4080 device. It should be called when you need to
 * verify or utilize the current short instruction configuration of the
 * device. The function requires a valid device structure pointer and a
 * pointer to an enum where the result will be stored. It returns an
 * error code if the device pointer is null or if there is a failure in
 * reading the device register.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function will return -EINVAL if this parameter is
 * null.
 * @param short_instr A pointer to an enum ad4080_short_instr where the current
 * short instruction mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error reading from the device.
 ******************************************************************************/
int ad4080_get_short_instr(struct ad4080_dev *dev,
			   enum ad4080_short_instr *short_instr);

/***************************************************************************//**
 * @brief This function configures the operation mode of the AD4080 device by
 * updating the relevant register with the specified mode. It should be
 * called when a change in the device's operation mode is required, such
 * as switching between normal, standby, or low power modes. The function
 * requires a valid device structure and an operation mode enumeration
 * value. It returns an error code if the device structure is null or if
 * the register update fails. The function must be called after the
 * device has been properly initialized.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param op_mode An enumeration value of type ad4080_op_mode specifying the
 * desired operation mode. Valid values are AD4080_OP_NORMAL,
 * AD4080_OP_STANDBY, and AD4080_OP_LOW_POWER.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4080_set_op_mode(struct ad4080_dev *dev,
		       enum ad4080_op_mode op_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current operation mode of an
 * AD4080 device. It should be called when the user needs to verify or
 * log the device's operational state. The function requires a valid
 * device structure pointer and a pointer to an enum where the operation
 * mode will be stored. It is important to ensure that the device has
 * been properly initialized before calling this function. If the device
 * pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param op_mode A pointer to an enum ad4080_op_mode where the current
 * operation mode will be stored. Must not be null. The caller
 * retains ownership.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in reading the device configuration.
 ******************************************************************************/
int ad4080_get_op_mode(struct ad4080_dev *dev,
		       enum ad4080_op_mode *op_mode);

/***************************************************************************//**
 * @brief This function configures the strict register access mode of the AD4080
 * device, which determines how registers are accessed. It should be
 * called when you need to change the register access mode to either
 * normal or strict. The function requires a valid device structure and a
 * strict register access mode to be specified. It returns an error code
 * if the device structure is null or if the register update operation
 * fails.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param strict_reg An enum value of type ad4080_strict_reg_access specifying
 * the desired strict register access mode. Valid values are
 * AD4080_REG_NORMAL_MODE and AD4080_REG_STRICT_MODE.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the register update operation fails.
 ******************************************************************************/
int ad4080_set_strict_reg_access(struct ad4080_dev *dev,
				 enum ad4080_strict_reg_access strict_reg);

/***************************************************************************//**
 * @brief This function is used to obtain the current strict register access
 * mode setting from the AD4080 device. It should be called when you need
 * to verify or utilize the device's strict register access
 * configuration. The function requires a valid device structure and a
 * pointer to store the retrieved mode. It returns an error code if the
 * device structure is null or if the read operation fails.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param strict_reg A pointer to an enum ad4080_strict_reg_access variable
 * where the current strict register access mode will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or the read operation fails.
 ******************************************************************************/
int ad4080_get_strict_reg_access(struct ad4080_dev *dev,
				 enum ad4080_strict_reg_access *strict_reg);

/***************************************************************************//**
 * @brief This function configures the interface check enable state of the
 * AD4080 device, which determines the output pattern of the device. It
 * should be called when the user needs to change the output pattern to
 * either normal data or a fixed pattern. The function requires a valid
 * device structure and an interface check enable state. It returns an
 * error code if the device structure is null or if the update operation
 * fails.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param intf_chk_en An enum value of type ad4080_intf_chk_en indicating the
 * desired interface check enable state. Valid values are
 * AD4080_DATA or AD4080_FIXED_PATTERN.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the update operation fails.
 ******************************************************************************/
int ad4080_set_intf_chk_en(struct ad4080_dev *dev,
			   enum ad4080_intf_chk_en intf_chk_en);

/***************************************************************************//**
 * @brief This function is used to obtain the current setting of the interface
 * check enable feature from an AD4080 device. It should be called when
 * you need to verify or log the current interface check configuration.
 * The function requires a valid device structure and a pointer to an
 * enum where the result will be stored. It returns an error code if the
 * device structure is null or if the read operation fails.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param intf_chk_en A pointer to an enum ad4080_intf_chk_en where the current
 * interface check enable setting will be stored. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or the read operation fails.
 ******************************************************************************/
int ad4080_get_intf_chk_en(struct ad4080_dev *dev,
			   enum ad4080_intf_chk_en *intf_chk_en);

/***************************************************************************//**
 * @brief This function sets the LVDS/SPI lane configuration for the AD4080
 * device, which determines how data is transmitted over the interface.
 * It should be called when the device is initialized and before starting
 * data transmission to ensure the correct lane configuration is used.
 * The function requires a valid device structure and a lane
 * configuration value. If the device structure is null, the function
 * returns an error. Successful execution updates the device's internal
 * configuration to reflect the new lane setting.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param cnv_spi_lvds_lanes An enum value of type ad4080_cnv_spi_lvds_lanes
 * specifying the desired lane configuration. Valid
 * values are AD4080_ONE_LANE and
 * AD4080_MULTIPLE_LANES.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4080_set_cnv_spi_lvds_lanes(struct ad4080_dev *dev,
				  enum ad4080_cnv_spi_lvds_lanes cnv_spi_lvds_lanes);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the
 * LVDS/SPI lanes for the AD4080 device. It should be called when you
 * need to verify or utilize the current lane configuration in your
 * application. The function requires a valid device structure and a
 * pointer to store the retrieved lane configuration. It returns an error
 * code if the device structure is null or if there is an issue reading
 * the configuration.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param cnv_spi_lvds_lanes A pointer to an enum ad4080_cnv_spi_lvds_lanes
 * variable where the current lane configuration will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if reading the configuration fails.
 ******************************************************************************/
int ad4080_get_cnv_spi_lvds_lanes(struct ad4080_dev *dev,
				  enum ad4080_cnv_spi_lvds_lanes *cnv_spi_lvds_lanes);

/***************************************************************************//**
 * @brief This function sets the data interface mode of the AD4080 device to
 * either SPI or LVDS, based on the provided configuration parameter. It
 * should be called when there is a need to change the data interface
 * mode of the device, typically during initialization or
 * reconfiguration. The function requires a valid device structure
 * pointer and a valid mode enumeration value. If the device pointer is
 * null, the function returns an error. Successful execution updates the
 * device's internal configuration to reflect the new mode.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param conv_data_spi_lvds An enumeration value of type
 * ad4080_conv_data_spi_lvds specifying the desired
 * data interface mode. Valid values are
 * AD4080_CONV_DATA_LVDS and AD4080_CONV_DATA_SPI.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if updating the device configuration fails.
 ******************************************************************************/
int ad4080_set_conv_data_spi_lvds(struct ad4080_dev *dev,
				  enum ad4080_conv_data_spi_lvds conv_data_spi_lvds);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the data
 * interface mode for the AD4080 device, which determines whether the
 * device is operating in SPI or LVDS mode. It should be called when the
 * user needs to verify or log the current data interface setting. The
 * function requires a valid device structure and a pointer to store the
 * retrieved configuration. It returns an error code if the device
 * structure is null or if the read operation fails.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param conv_data_spi_lvds A pointer to an enum ad4080_conv_data_spi_lvds
 * variable where the current data interface
 * configuration will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if the read operation fails.
 ******************************************************************************/
int ad4080_get_conv_data_spi_lvds(struct ad4080_dev *dev,
				  enum ad4080_conv_data_spi_lvds *conv_data_spi_lvds);

/***************************************************************************//**
 * @brief This function configures the number of interface clock periods from
 * the CNV rising edge for the AD4080 device. It should be used when you
 * need to adjust the timing of the LVDS interface clock relative to the
 * conversion signal. The function must be called with a valid device
 * structure that has been properly initialized. If the device pointer is
 * null, the function will return an error. The function updates the
 * device's internal state to reflect the new clock count.
 *
 * @param dev A pointer to an initialized ad4080_dev structure. Must not be
 * null. The function will return -EINVAL if this parameter is null.
 * @param lvds_cnv_clk_cnt A uint8_t value representing the desired number of
 * interface clock periods from the CNV rising edge. The
 * valid range is determined by the device's
 * specifications, and invalid values may result in
 * undefined behavior.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ad4080_set_lvds_cnv_clk_cnt(struct ad4080_dev *dev,
				uint8_t lvds_cnv_clk_cnt);

/***************************************************************************//**
 * @brief This function is used to obtain the LVDS conversion clock count from
 * the AD4080 device, which is essential for configuring the device's
 * data interface. It should be called when the user needs to verify or
 * utilize the current LVDS conversion clock count setting. The function
 * requires a valid device structure and a pointer to store the retrieved
 * clock count. It returns an error code if the device structure is null
 * or if the read operation fails.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param lvds_cnv_clk_cnt A pointer to a uint8_t variable where the LVDS
 * conversion clock count will be stored. Must not be
 * null. The function writes the retrieved clock count
 * to this location.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if the read operation fails.
 ******************************************************************************/
int ad4080_get_lvds_cnv_clk_cnt(struct ad4080_dev *dev,
				uint8_t *lvds_cnv_clk_cnt);

/***************************************************************************//**
 * @brief This function configures the LVDS self-clock mode of the AD4080
 * device, which is essential for determining the clocking behavior of
 * the LVDS interface. It should be called when the device is initialized
 * and before any operations that depend on the LVDS clock mode. The
 * function requires a valid device structure and a specified clock mode.
 * If the device pointer is null, the function returns an error.
 * Successful execution updates the device's configuration and returns
 * zero.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns -EINVAL.
 * @param lvds_self_clk_mode An enum value of type ad4080_lvds_self_clk_mode
 * specifying the desired LVDS self-clock mode. Must
 * be a valid enum value.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if updating the device configuration fails.
 ******************************************************************************/
int ad4080_set_lvds_self_clk_mode(struct ad4080_dev *dev,
				  enum ad4080_lvds_self_clk_mode lvds_self_clk_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current LVDS self clock mode from
 * the AD4080 device. It should be called when you need to verify or
 * utilize the current LVDS self clock mode setting in your application.
 * The function requires a valid device structure pointer and a pointer
 * to an enum where the mode will be stored. It returns an error code if
 * the device pointer is null or if there is a failure in reading the
 * register.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function will return -EINVAL if this parameter is
 * null.
 * @param lvds_self_clk_mode A pointer to an enum ad4080_lvds_self_clk_mode
 * where the current LVDS self clock mode will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in reading the register.
 ******************************************************************************/
int ad4080_get_lvds_self_clk_mode(struct ad4080_dev *dev,
				  enum ad4080_lvds_self_clk_mode *lvds_self_clk_mode);


/***************************************************************************//**
 * @brief This function configures the LVDS CNV clock mode of the AD4080 device
 * to the specified mode. It should be called when the user needs to
 * change the clock mode of the device, typically during initialization
 * or when reconfiguring the device for different operational
 * requirements. The function requires a valid device structure and a
 * valid clock mode enumeration value. It returns an error code if the
 * device structure is null or if the operation fails.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param cnv_clk_mode An enumeration value of type ad4080_lvds_cnv_clk_mode
 * specifying the desired LVDS CNV clock mode. Valid values
 * are AD4080_CNV_CMOS_MODE and AD4080_CNV_LVDS_MODE.
 * @return Returns 0 on success, or a negative error code if the device
 * structure is null or if the operation fails.
 ******************************************************************************/
int ad4080_set_lvds_cnv_clk_mode(struct ad4080_dev *dev,
				 enum ad4080_lvds_cnv_clk_mode cnv_clk_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current LVDS CNV clock mode from
 * the AD4080 device. It should be called when you need to verify or
 * utilize the current clock mode setting of the device. The function
 * requires a valid device structure pointer and a pointer to an enum
 * where the clock mode will be stored. It returns an error code if the
 * device pointer is null or if there is an issue reading the device
 * register.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param cnv_clk_mode A pointer to an enum ad4080_lvds_cnv_clk_mode where the
 * current clock mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error reading the register.
 ******************************************************************************/
int ad4080_get_lvds_cnv_clk_mode(struct ad4080_dev *dev,
				 enum ad4080_lvds_cnv_clk_mode *cnv_clk_mode);

/***************************************************************************//**
 * @brief This function configures the LVDS differential output voltage level
 * for the specified AD4080 device. It should be called when you need to
 * adjust the LVDS output voltage to match system requirements or to
 * optimize signal integrity. The function requires a valid device
 * structure and a valid LVDS voltage level enumeration value. It returns
 * an error code if the device structure is null or if the update
 * operation fails. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param lvds_vod An enumeration value of type ad4080_lvds_vod specifying the
 * desired LVDS differential output voltage. Must be a valid
 * value from the ad4080_lvds_vod enum.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if the update operation fails.
 ******************************************************************************/
int ad4080_set_lvds_vod(struct ad4080_dev *dev,
			enum ad4080_lvds_vod lvds_vod);

/***************************************************************************//**
 * @brief This function is used to obtain the current LVDS differential output
 * voltage setting from an AD4080 device. It should be called when you
 * need to know the LVDS VOD configuration of the device. The function
 * requires a valid device structure pointer and a pointer to an enum
 * where the result will be stored. It returns an error code if the
 * device pointer is null or if there is a failure in reading the device
 * register.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. If null, the function returns an error code.
 * @param lvds_vod A pointer to an enum ad4080_lvds_vod where the LVDS VOD
 * setting will be stored. Must not be null. The caller is
 * responsible for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in reading the register.
 ******************************************************************************/
int ad4080_get_lvds_vod(struct ad4080_dev *dev, enum ad4080_lvds_vod *lvds_vod);

/***************************************************************************//**
 * @brief This function sets the power-down state of the analog/digital LDO for
 * the AD4080 device. It should be called when there is a need to enable
 * or disable the LDO, typically during power management operations. The
 * function requires a valid device structure and an enumeration value
 * indicating the desired LDO state. It returns an error code if the
 * device structure is null or if the operation fails. The function
 * updates the device's internal state to reflect the new LDO
 * configuration.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param ana_dig_ldo_pd An enum ad4080_ana_dig_ldo_pd value indicating the
 * desired power-down state of the analog/digital LDO.
 * Valid values are AD4080_AD_LDO_EN to enable and
 * AD4080_AD_LDO_DISABLE to disable the LDO.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ad4080_set_ana_dig_ldo_pd(struct ad4080_dev *dev,
			      enum ad4080_ana_dig_ldo_pd ana_dig_ldo_pd);

/***************************************************************************//**
 * @brief This function is used to obtain the current power-down state of the
 * analog/digital LDO in the AD4080 device. It should be called when the
 * user needs to verify or log the power state of the LDO. The function
 * requires a valid device structure pointer and a pointer to an enum
 * where the result will be stored. It returns an error code if the
 * device pointer is null or if there is a failure in reading the
 * register.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error.
 * @param ana_dig_ldo_pd A pointer to an enum ad4080_ana_dig_ldo_pd where the
 * current LDO power-down state will be stored. Must not
 * be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is a failure in reading the register.
 ******************************************************************************/
int ad4080_get_ana_dig_ldo_pd(struct ad4080_dev *dev,
			      enum ad4080_ana_dig_ldo_pd *ana_dig_ldo_pd);

/***************************************************************************//**
 * @brief This function configures the power-down mode of the interface LDO for
 * a given AD4080 device. It should be called when you need to enable or
 * disable the interface LDO, which can be useful for power management
 * purposes. The function requires a valid device structure and an
 * enumeration value indicating the desired power-down mode. If the
 * device structure is null, the function returns an error. It updates
 * the device's internal state to reflect the new power-down mode.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param intf_ldo_pd An enum ad4080_intf_ldo_pd value indicating the desired
 * interface LDO power-down mode. Valid values are
 * AD4080_INTF_LDO_EN and AD4080_INTF_LDO_DISABLE.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if updating the device register fails.
 ******************************************************************************/
int ad4080_set_intf_ldo_pd(struct ad4080_dev *dev,
			   enum ad4080_intf_ldo_pd intf_ldo_pd);

/***************************************************************************//**
 * @brief Use this function to obtain the current power-down status of the
 * interface LDO for the AD4080 device. It is essential to ensure that
 * the `dev` parameter is a valid, initialized device structure before
 * calling this function. The function reads the relevant register and
 * extracts the status, which is then stored in the provided output
 * parameter. This function should be called when you need to check the
 * power state of the interface LDO, especially in power management
 * scenarios.
 *
 * @param dev A pointer to an initialized `ad4080_dev` structure representing
 * the device. Must not be null. If null, the function returns an
 * error code.
 * @param intf_ldo_pd A pointer to an `enum ad4080_intf_ldo_pd` where the
 * interface LDO power-down status will be stored. Must not
 * be null.
 * @return Returns 0 on success, or a negative error code if the device is
 * invalid or if there is a failure in reading the register.
 ******************************************************************************/
int ad4080_get_intf_ldo_pd(struct ad4080_dev *dev,
			   enum ad4080_intf_ldo_pd *intf_ldo_pd);

/***************************************************************************//**
 * @brief This function configures the FIFO mode of the AD4080 device, which
 * determines how data is buffered and triggered. It should be called
 * when the device is initialized and ready for configuration. The
 * function requires a valid device structure pointer and a specified
 * FIFO mode. If the device pointer is null, the function returns an
 * error. The function updates the device's internal state to reflect the
 * new FIFO mode.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. If null, the function returns an error
 * code.
 * @param fifo_mode An enum value of type ad4080_fifo_mode specifying the
 * desired FIFO mode. Valid values are defined in the
 * ad4080_fifo_mode enumeration.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ad4080_set_fifo_mode(struct ad4080_dev *dev,
			 enum ad4080_fifo_mode fifo_mode);

/***************************************************************************//**
 * @brief This function is used to obtain the current FIFO mode configuration of
 * an AD4080 device. It should be called when the user needs to verify or
 * utilize the FIFO mode setting of the device. The function requires a
 * valid device structure pointer and a pointer to an enum where the FIFO
 * mode will be stored. It returns an error code if the device pointer is
 * null or if there is a failure in reading the device's configuration
 * register.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. If null, the function returns an error code.
 * @param fifo_mode A pointer to an enum ad4080_fifo_mode where the current FIFO
 * mode will be stored. Must not be null. The caller must
 * ensure this pointer is valid.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if reading the register fails.
 ******************************************************************************/
int ad4080_get_fifo_mode(struct ad4080_dev *dev,
			 enum ad4080_fifo_mode *fifo_mode);

/***************************************************************************//**
 * @brief This function configures the FIFO watermark level for the AD4080
 * device, which is used to trigger an event when the FIFO reaches a
 * certain fill level. It should be called after the device has been
 * properly initialized. The function checks if the provided device
 * structure is valid and if the specified watermark level is within the
 * allowable range. If these conditions are not met, it returns an error
 * code. This function is useful for managing data flow and ensuring
 * timely processing of data in applications where the AD4080 is used.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @param fifo_watermark A 16-bit unsigned integer specifying the desired FIFO
 * watermark level. Must be less than or equal to
 * AD4080_FIFO_SIZE. If the value exceeds this limit, the
 * function returns an error.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or the watermark level is invalid.
 ******************************************************************************/
int ad4080_set_fifo_watermark(struct ad4080_dev *dev,
			      uint16_t fifo_watermark);

/***************************************************************************//**
 * @brief This function is used to obtain the current FIFO watermark setting
 * from an AD4080 device. It should be called when the user needs to know
 * the FIFO watermark level for monitoring or configuration purposes. The
 * function requires a valid device structure and a pointer to store the
 * retrieved watermark value. It returns an error code if the device
 * structure or the pointer is null, or if there is a failure in reading
 * the watermark value from the device.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @param fifo_watermark A pointer to a uint16_t variable where the FIFO
 * watermark value will be stored. Must not be null. The
 * function will return -EINVAL if this parameter is null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters or other error codes for read
 * failures.
 ******************************************************************************/
int ad4080_get_fifo_watermark(struct ad4080_dev *dev,
			      uint16_t *fifo_watermark);

/***************************************************************************//**
 * @brief This function is used to set a specific GPIO pin on the AD4080 device
 * to operate either as an input or an output. It should be called when
 * configuring the GPIOs of the device, typically during initialization
 * or when changing the device's operational mode. The function requires
 * a valid device structure and valid GPIO and operation enable
 * parameters. It returns an error code if the device structure is null,
 * if the specified GPIO is out of range, or if the operation enable
 * parameter is invalid.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param gpio An enum value of type ad4080_gpio specifying which GPIO pin to
 * configure. Must be a valid GPIO identifier within the range
 * defined by the enum.
 * @param gpio_op_enable An enum value of type ad4080_gpio_op_enable indicating
 * whether the GPIO should be configured as input or
 * output. Must be either AD4080_GPIO_INPUT or
 * AD4080_GPIO_OUTPUT.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ad4080_set_gpio_output_enable(struct ad4080_dev *dev,
				  enum ad4080_gpio gpio,
				  enum ad4080_gpio_op_enable gpio_op_enable);

/***************************************************************************//**
 * @brief Use this function to set a specific GPIO pin on the AD4080 device to
 * perform a designated function. This function should be called after
 * the device has been initialized and the GPIO has been configured as an
 * output. It is important to ensure that the `dev` parameter is not null
 * and that the `gpio` and `gpio_func` parameters are within their valid
 * ranges. If any of these conditions are not met, the function will
 * return an error code. The function updates the device's internal state
 * to reflect the new GPIO function configuration.
 *
 * @param dev A pointer to an initialized `ad4080_dev` structure representing
 * the device. Must not be null.
 * @param gpio An enumerated value of type `enum ad4080_gpio` specifying which
 * GPIO pin to configure. Must be a valid GPIO identifier within the
 * range defined by the `enum`.
 * @param gpio_func An enumerated value of type `enum ad4080_gpio_op_func_sel`
 * specifying the function to assign to the GPIO pin. Must be a
 * valid function selection within the range defined by the
 * `enum`.
 * @return Returns 0 on success or a negative error code on failure, such as
 * invalid parameters.
 ******************************************************************************/
int ad4080_set_gpio_output_func(struct ad4080_dev *dev,
				enum ad4080_gpio gpio,
				enum ad4080_gpio_op_func_sel gpio_func);

/***************************************************************************//**
 * @brief Use this function to set the output state of a specific GPIO pin on
 * the AD4080 device. The function requires that the GPIO pin is
 * configured as an output; otherwise, it will return an error. It is
 * essential to ensure that the device structure is properly initialized
 * and that the specified GPIO pin is within the valid range. This
 * function should be called when you need to control the state of a GPIO
 * pin that has been configured for output.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null.
 * @param gpio An enum value of type ad4080_gpio specifying the GPIO pin to
 * write to. Must be a valid GPIO pin within the range defined by
 * the enum.
 * @param data A boolean value representing the data to write to the GPIO pin.
 * True sets the pin high, and false sets it low.
 * @return Returns 0 on success, or a negative error code if the device is null,
 * the GPIO pin is invalid, or the pin is not configured as an output.
 ******************************************************************************/
int ad4080_gpio_write_data(struct ad4080_dev *dev,
			   enum ad4080_gpio gpio,
			   bool data);

/***************************************************************************//**
 * @brief Use this function to read the current state of a specified GPIO pin on
 * the AD4080 device. It is essential to ensure that the GPIO is
 * configured as an input before calling this function, as attempting to
 * read from a GPIO configured as an output will result in an error. The
 * function requires a valid device structure and a pointer to store the
 * read data. It returns an error code if the device or data pointer is
 * null, or if the specified GPIO index is out of range.
 *
 * @param dev A pointer to an initialized ad4080_dev structure representing the
 * device. Must not be null.
 * @param gpio An enum value of type ad4080_gpio specifying the GPIO pin to
 * read. Must be a valid GPIO index less than NUM_AD4080_GPIO.
 * @param data A pointer to a bool where the read GPIO state will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL if inputs are invalid or the GPIO is configured as output.
 ******************************************************************************/
int ad4080_gpio_read_data(struct ad4080_dev *dev,
			  enum ad4080_gpio gpio,
			  bool *data);

/***************************************************************************//**
 * @brief This function sets up the configuration interface of the AD4080 device
 * using the provided initialization parameters. It must be called after
 * the device structure is properly allocated and before any other device
 * operations are performed. The function configures the device's single
 * instruction mode, short instruction mode, and strict register access
 * settings based on the values in the `init_param` structure. If the
 * `dev` parameter is null, the function returns an error. It also
 * returns an error if any of the configuration steps fail, ensuring that
 * the device is not left in a partially configured state.
 *
 * @param dev A pointer to an `ad4080_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param init_param An `ad4080_init_param` structure containing the
 * initialization parameters for the device. The structure
 * should be fully populated with valid configuration settings
 * before calling this function.
 * @return Returns 0 on success, or a negative error code if the device is null
 * or if any configuration step fails.
 ******************************************************************************/
int ad4080_configuration_intf_init(struct ad4080_dev *device,
				   struct ad4080_init_param init_param);

/***************************************************************************//**
 * @brief This function configures the data interface of an AD4080 device using
 * the specified initialization parameters. It must be called after the
 * device structure is properly allocated and initialized. The function
 * sets various data interface parameters such as LVDS/SPI lane control,
 * conversion data interface, LVDS clock count, self-clock mode,
 * differential output voltage, and FIFO mode. If any parameter setting
 * fails, the function returns an error code, and the initialization
 * process is halted. This function is essential for preparing the device
 * for data operations.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device. Must
 * not be null. The function returns -EINVAL if this parameter is
 * null.
 * @param init_param An ad4080_init_param structure containing the
 * initialization parameters for the data interface. The
 * caller retains ownership of this structure.
 * @return Returns 0 on success or a negative error code if any parameter
 * setting fails.
 ******************************************************************************/
int ad4080_data_intf_init(struct ad4080_dev *device,
			  struct ad4080_init_param init_param);

/***************************************************************************//**
 * @brief This function sets up the AD4080 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * device to ensure proper setup. The function performs SPI
 * initialization, verifies the device identity, applies a software
 * reset, and configures various operational modes and GPIO settings. If
 * any step fails, the function cleans up allocated resources and returns
 * an error code. The caller must ensure that the `device` pointer is
 * valid and that the `init_param` structure is correctly populated with
 * desired settings.
 *
 * @param device A pointer to a pointer of type `struct ad4080_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad4080_init_param` containing
 * initialization parameters for the device. Must be properly
 * initialized with valid settings before calling the
 * function.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `device` pointer is set to point to the initialized
 * device structure.
 ******************************************************************************/
int ad4080_init(struct ad4080_dev **device,
		struct ad4080_init_param init_param);

/***************************************************************************//**
 * @brief This function is used to properly release and clean up resources
 * associated with an AD4080 device instance. It should be called when
 * the device is no longer needed to ensure that all allocated resources
 * are freed and any associated SPI descriptors are removed. The function
 * must be called with a valid device pointer that was previously
 * initialized. If the provided device pointer is null, the function will
 * return an error code indicating invalid input. This function is
 * essential for preventing resource leaks in applications that utilize
 * the AD4080 device.
 *
 * @param dev A pointer to an ad4080_dev structure representing the device to be
 * removed. Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on successful removal and resource cleanup. If the device
 * pointer is null, returns -EINVAL. If an error occurs during SPI
 * descriptor removal, the function returns the corresponding error
 * code.
 ******************************************************************************/
int ad4080_remove(struct ad4080_dev *dev);

#endif /* __AD4080_H__ */
