/***************************************************************************//**
 *   @file   ad4110.h
 *   @brief  Header file of AD4110 Driver.
 *   @author Stefan Popa (stefan.popa@analog.com)
 * 	     Andrei Porumb (andrei.porumb@analog.com)
 * 	     Mihail Chindris (mihail.chindris@analog.com)
********************************************************************************
 * Copyright 2021(c) Analog Devices, Inc.
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

#ifndef AD4110_H_
#define AD4110_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_irq.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD4110_CMD_WR_COM_REG(x)   (0x00 | ((x) & 0xF)) // Write to Register x
#define AD4110_CMD_READ_COM_REG(x) (0x40 | ((x) & 0xF)) // Read from Register x
#define AD4110_DEV_ADDR_MASK	   (0x30) // Device address mask

/* Register map */
#define A4110_ADC 0x00
#define A4110_AFE 0x01

/****************************** AFE Register Map ******************************/
#define AD4110_REG_AFE_TOP_STATUS 	     0x0
#define AD4110_REG_AFE_CNTRL1		     0x1
#define AD4110_REG_AFE_CLK_CTRL		     0x3
#define AD4110_REG_AFE_CNTRL2 		     0x4
#define AD4110_REG_PGA_RTD_CTRL		     0x5
#define AD4110_REG_AFE_ERR_DISABLE	     0x6
#define AD4110_REG_AFE_DETAIL_STATUS	     0x7
#define AD4110_REG_AFE_CAL_DATA		     0xC
#define AD4110_REG_AFE_RSENSE_DATA	     0xD
#define AD4110_REG_AFE_NO_PWR_DEFAULT_SEL    0xE
#define AD4110_REG_AFE_NO_PWR_DEFAULT_STATUS 0xF

/***************************** ADC Register Map *******************************/
#define AD4110_REG_ADC_STATUS 			0x0
#define AD4110_REG_ADC_MODE			0x1
#define AD4110_REG_ADC_INTERFACE		0x2
#define AD4110_REG_ADC_CONFIG			0x3
#define AD4110_REG_DATA				0x4
#define AD4110_REG_FILTER			0x5
#define AD4110_REG_ADC_GPIO_CONFIG		0x6
#define AD4110_REG_ID				0x7
#define AD4110_ADC_OFFSET0			0x8
#define AD4110_ADC_OFFSET1			0x9
#define AD4110_ADC_OFFSET2			0xA
#define AD4110_ADC_OFFSET3			0xB
#define AD4110_ADC_GAIN0			0xC
#define AD4110_ADC_GAIN1			0xD
#define AD4110_ADC_GAIN2			0xE
#define AD4110_ADC_GAIN3			0xF

/* AFE_CNTRL1 Register */
#define AD4110_REG_AFE_CNTRL1_CRC_EN		(1 << 14)
#define AD4110_REG_AFE_CNTRL1_DISRTD		(1 << 9)

/* AFE_CLK_CTRL Register */
#define AD4110_REG_AFE_CLK_CTRL_CFG(x)		(((x) & 0x3) << 3)

/* AFE_CNTRL2 Register */
#define AD4110_REG_AFE_CNTRL2_IMODE_MSK 	(1 << 1)
#define AD4110_REG_AFE_CNTRL2_EXT_R_SEL_MSK 	(1 << 2)
#define AD4110_REG_AFE_CNTRL2_EN_FLD_PWR_MSK 	(1 << 3)
#define AD4110_AFE_VBIAS(x) 			(((x) & 0x3) << 6)
#define AD4110_AFE_VBIAS_ON 			0x1
#define AD4110_AFE_VBIAS_DEFAULT_OFF		0x2
#define AD4110_AFE_VBIAS_OFF		 	0x3
#define AD4110_REG_AFE_CNTRL2_AINP_UP1		(1 << 8)
#define AD4110_REG_AFE_CNTRL2_AINP_UP100	(1 << 9)
#define AD4110_REG_AFE_CNTRL2_AINP_DN1		(1 << 10)
#define AD4110_REG_AFE_CNTRL2_AINP_DN100	(1 << 11)
#define AD4110_REG_AFE_CNTRL2_AINN_UP1		(1 << 12)
#define AD4110_REG_AFE_CNTRL2_AINN_UP100	(1 << 13)
#define AD4110_REG_AFE_CNTRL2_AINN_DN1		(1 << 14)
#define AD4110_REG_AFE_CNTRL2_AINN_DN100	(1 << 15)

/* PGA_RTD_CTRL Register */
#define AD4110_REG_PGA_RTD_CTRL_23W_EN_MSK	(1 << 15)
#define AD4110_REG_PGA_RTD_CTRL_I_COM_SEL(x)	(((x) & 0x7) << 12)
#define AD4110_REG_PGA_RTD_CTRL_I_EXC_SEL(x)	(((x) & 0x7) << 9)
#define AD4110_REG_PGA_RTD_CTRL_EXT_RTD		(1 << 8)
#define AD4110_REG_PGA_RTD_CTRL_GAIN_CH(x)	(((x) & 0xF) << 4)
#define AD4110_REG_PGA_RTD_CTRL_GAIN_CH_MSK     0xF0

/* AFE_ERR_DISABLE Register */
#define AD4110_REG_AFE_ERR_DIS_AIN_OC		(1 << 1)
#define AD4110_REG_AFE_ERR_DIS_FLD_PWR_OC	(1 << 2)
#define AD4110_REG_AFE_ERR_DIS_I_COM		(1 << 6)
#define AD4110_REG_AFE_ERR_DIS_I_EXC		(1 << 7)
#define AD4110_REG_AFE_ERR_DIS_AINP_OV		(1 << 8)
#define AD4110_REG_AFE_ERR_DIS_AINN_OV		(1 << 9)
#define AD4110_REG_AFE_ERR_DIS_AINP_UV		(1 << 10)
#define AD4110_REG_AFE_ERR_DIS_AINN_UV		(1 << 11)

/* NO_PWR_DEFAULT_SEL Register */
#define AD4110_REG_NO_PWR_DEFAULT_SEL_MSK	0xFF

/* ADC status register */
#define AD4110_REG_ADC_STATUS_RDY			(1 << 7)

/* ADC_MODE Register */
#define AD4110_REG_ADC_MODE_MSK			0x70
#define AD4110_ADC_MODE(x)			(((x) & 0x7) << 4)
#define AD4110_REG_ADC_MODE_REF_EN		(1 << 15)
#define AD4110_REG_ADC_DELAY(x)			(((x) & 0x7) << 8)
#define AD4110_REG_ADC_CLK_SEL(x)		(((x) & 0x3) << 2)

/* ADC_INTERFACE Register */
#define AD4110_REG_ADC_INTERFACE_CRC_EN_MSK	0x0C
#define AD4110_ADC_CRC_EN(x)			(((x) & 0x3) << 2)
#define AD4110_REG_ADC_INTERFACE_WL16_MSK	0x01
#define AD4110_REG_ADC_INTERFACE_DS_MSK		0x40
#define AD4110_DATA_STAT_EN			(1 << 6)

/* ADC_CONFIG Register */
#define AD4110_REG_ADC_CONFIG_CHAN_EN_MSK	0xF
#define AD4110_REG_ADC_CONFIG_CHAN_EN_0		(1 << 0)
#define AD4110_REG_ADC_CONFIG_CHAN_EN_1		(1 << 1)
#define AD4110_REG_ADC_CONFIG_CHAN_EN_2		(1 << 2)
#define AD4110_REG_ADC_CONFIG_CHAN_EN_3		(1 << 3)
#define AD4110_REG_ADC_CONFIG_REF_SEL(x)	(((x) & 0x3) << 4)
#define AD4110_REG_ADC_CONFIG_BIT_6		(1 << 6)
#define AD4110_REG_ADC_CONFIG_AIN_BUFF(x)	((((x) & 0x3) << 8))
#define AD4110_REG_ADC_CONFIG_BI_UNIPOLAR	(1 << 12)

/* ADC_FILTER Register */
#define AD4110_REG_ADC_FILTER_ODR(x)		(((x) & 0x1F) << 0)
#define AD4110_REG_ADC_FILTER_ORDER(x)		(((x) & 0x3) << 5)
#define AD4110_REG_ADC_FILTER_SEL_ENH(x)	(((x) & 0x7) << 8)
#define AD4110_REG_ADC_FILTER_EN_ENH		(1 << 11)

/* ADC_GPIO_CONFIG Register */
#define AD4110_REG_GPIO_CONFIG_ERR_EN(x)	(((x) & 0x3) << 9)
#define AD4110_REG_GPIO_CONFIG_SYNC_EN(x)	(((x) & 0x1) << 11)

/* 8-bits wide checksum generated using the polynomial */
#define AD4110_CRC8_POLY	0x07 // x^8 + x^2 + x^1 + x^0

/* ADC conversion timeout */
#define AD4110_ADC_CONV_TIMEOUT	10000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/* If AD4110_AFE_ADC_CLOCKED selected, select AD4110_ADC_INT_CLK_CLKIO */
/***************************************************************************//**
 * @brief The `ad4110_adc_clk_sel` enumeration defines the possible clock source
 * selections for the AD4110 ADC. It allows the user to choose between
 * using the internal clock, the internal clock with clock I/O, or an
 * external clock. This selection is crucial for configuring the ADC's
 * timing and synchronization with other components in a system.
 *
 * @param AD4110_ADC_INT_CLK Represents the internal clock selection for the
 * ADC.
 * @param AD4110_ADC_INT_CLK_CLKIO Represents the internal clock with clock I/O
 * selection for the ADC.
 * @param AD4110_ADC_EXT_CLK Represents the external clock selection for the
 * ADC.
 ******************************************************************************/
enum ad4110_adc_clk_sel {
	AD4110_ADC_INT_CLK,
	AD4110_ADC_INT_CLK_CLKIO,
	AD4110_ADC_EXT_CLK,
};

/***************************************************************************//**
 * @brief The `ad4110_afe_clk_cfg` enumeration defines the clock configuration
 * options for the Analog Front End (AFE) of the AD4110 device. It
 * provides two options: using the internal clock of the AFE or having
 * the AFE clocked by the ADC. This configuration is crucial for setting
 * up the timing and synchronization of the AFE with the rest of the
 * system.
 *
 * @param AD4110_AFE_INT_CLOCK Represents the internal clock configuration for
 * the AFE.
 * @param AD4110_AFE_ADC_CLOCKED Represents the configuration where the AFE is
 * clocked by the ADC.
 ******************************************************************************/
enum ad4110_afe_clk_cfg {
	AD4110_AFE_INT_CLOCK = 0,
	AD4110_AFE_ADC_CLOCKED = 2
};

/***************************************************************************//**
 * @brief The `ad4110_sync_en` enumeration defines two possible states for
 * synchronization in the AD4110 device: disabled (`AD4110_SYNC_DIS`) and
 * enabled (`AD4110_SYNC_EN`). This enumeration is used to configure the
 * synchronization feature of the AD4110, which is a precision analog-to-
 * digital converter (ADC). The synchronization feature allows the ADC to
 * be synchronized with an external clock or signal, which can be crucial
 * for applications requiring precise timing and coordination with other
 * system components.
 *
 * @param AD4110_SYNC_DIS Represents the disabled state for synchronization.
 * @param AD4110_SYNC_EN Represents the enabled state for synchronization.
 ******************************************************************************/
enum ad4110_sync_en {
	AD4110_SYNC_DIS,
	AD4110_SYNC_EN
};

/***************************************************************************//**
 * @brief The `ad4110_voltage_reference` enumeration defines the possible
 * voltage reference sources for the AD4110 device. It includes options
 * for using an external reference, an internal 2.5V reference, or the
 * AVDD5 as the reference. This enumeration is used to configure the
 * voltage reference setting in the AD4110 device, which is crucial for
 * accurate analog-to-digital conversion.
 *
 * @param AD4110_EXT_REF Represents an external voltage reference with a value
 * of 0.
 * @param AD4110_INT_2_5V_REF Represents an internal 2.5V voltage reference with
 * a value of 2.
 * @param AD4110_AVDD5_REF Represents an AVDD5 voltage reference with a value of
 * 3.
 ******************************************************************************/
enum ad4110_voltage_reference {
	AD4110_EXT_REF = 0,
	AD4110_INT_2_5V_REF = 2,
	AD4110_AVDD5_REF = 3
};

/***************************************************************************//**
 * @brief The `ad4110_state` enumeration defines two possible states for the
 * AD4110 device: `AD4110_DISABLE` and `AD4110_ENABLE`. These states are
 * used to control whether the device is active or inactive, providing a
 * simple mechanism to manage the operational status of the AD4110 within
 * the driver.
 *
 * @param AD4110_DISABLE Represents the disabled state of the AD4110 device.
 * @param AD4110_ENABLE Represents the enabled state of the AD4110 device.
 ******************************************************************************/
enum ad4110_state {
	AD4110_DISABLE,
	AD4110_ENABLE
};

/***************************************************************************//**
 * @brief The `ad4110_data_word_length` enumeration defines the possible data
 * word lengths for the AD4110 device, allowing for either 24-bit or
 * 16-bit data words. This setting is crucial for configuring the
 * device's data output format, which can impact the resolution and data
 * handling in applications using the AD4110.
 *
 * @param AD4110_DATA_WL24 Represents a 24-bit data word length.
 * @param AD4110_DATA_WL16 Represents a 16-bit data word length.
 ******************************************************************************/
enum ad4110_data_word_length {
	AD4110_DATA_WL24,
	AD4110_DATA_WL16,
};

/***************************************************************************//**
 * @brief The `ad4110_adc_mode` enumeration defines various operational modes
 * for the AD4110 ADC, including continuous and single conversion modes,
 * standby and power-down modes, as well as system offset and gain
 * calibration modes. Each mode is associated with a specific integer
 * value, which is used to configure the ADC's behavior in different
 * scenarios.
 *
 * @param AD4110_CONTINOUS_CONV_MODE Represents the continuous conversion mode
 * with a value of 0.
 * @param AD4110_SINGLE_CONV_MODE Represents the single conversion mode with a
 * value of 1.
 * @param AD4110_STANDBY_MODE Represents the standby mode with a value of 2.
 * @param AD4110_PW_DOWN_MODE Represents the power-down mode with a value of 3.
 * @param AD4110_SYS_OFFSET_CAL Represents the system offset calibration mode
 * with a value of 6.
 * @param AD4110_SYS_GAIN_CAL Represents the system gain calibration mode with a
 * value of 7.
 ******************************************************************************/
enum ad4110_adc_mode {
	AD4110_CONTINOUS_CONV_MODE = 0,
	AD4110_SINGLE_CONV_MODE = 1,
	AD4110_STANDBY_MODE = 2,
	AD4110_PW_DOWN_MODE = 3,
	AD4110_SYS_OFFSET_CAL = 6,
	AD4110_SYS_GAIN_CAL = 7
};

/***************************************************************************//**
 * @brief The `ad4110_op_mode` enumeration defines various operational modes for
 * the AD4110 device, which is an analog-to-digital converter. Each mode
 * corresponds to a specific type of input or configuration, such as
 * voltage, current, thermocouple, or RTD (Resistance Temperature
 * Detector) with different wiring configurations. This allows the device
 * to be configured for different types of sensor inputs and measurement
 * scenarios.
 *
 * @param AD4110_VOLTAGE_MODE Represents the voltage operation mode.
 * @param AD4110_CURRENT_MODE Represents the current operation mode.
 * @param AD4110_CURRENT_MODE_EXT_R_SEL Represents the current mode with
 * external resistor selection.
 * @param AD4110_THERMOCOUPLE Represents the thermocouple operation mode.
 * @param AD4110_FLD_POWER_MODE Represents the field power operation mode.
 * @param AD4110_RTD_2W_MODE Represents the 2-wire RTD operation mode.
 * @param AD4110_RTD_3W_MODE Represents the 3-wire RTD operation mode.
 * @param AD4110_RTD_4W_MODE Represents the 4-wire RTD operation mode.
 ******************************************************************************/
enum ad4110_op_mode {
	AD4110_VOLTAGE_MODE,
	AD4110_CURRENT_MODE,
	AD4110_CURRENT_MODE_EXT_R_SEL,
	AD4110_THERMOCOUPLE,
	AD4110_FLD_POWER_MODE,
	AD4110_RTD_2W_MODE,
	AD4110_RTD_3W_MODE,
	AD4110_RTD_4W_MODE
};

/***************************************************************************//**
 * @brief The `ad4110_adc_crc_mode` enumeration defines the different modes of
 * cyclic redundancy check (CRC) that can be applied to the ADC
 * operations in the AD4110 device. It provides options to disable CRC,
 * use an XOR checksum for reads and CRC for writes, or use CRC for both
 * reads and writes, enhancing data integrity during communication.
 *
 * @param AD4110_ADC_CRC_DISABLE Disables CRC for ADC operations.
 * @param AD4110_ADC_XOR_CRC Enables 8-bit XOR checksum on reads and 8-bit CRC
 * on writes.
 * @param AD4110_ADC_CRC_CRC Enables 8-bit CRC on both reads and writes.
 ******************************************************************************/
enum ad4110_adc_crc_mode {
	AD4110_ADC_CRC_DISABLE,
	AD4110_ADC_XOR_CRC,  // 8-bit XOR checksum on reads, 8-bit CRC on writes
	AD4110_ADC_CRC_CRC // 8-bit CRC on reads and writes.
};

/***************************************************************************//**
 * @brief The `ad4110_afe_crc_mode` enumeration defines the modes for enabling
 * or disabling the Cyclic Redundancy Check (CRC) for the Analog Front
 * End (AFE) of the AD4110 device. It provides two options: disabling CRC
 * or enabling an 8-bit CRC for both read and write operations, which
 * helps in ensuring data integrity during communication.
 *
 * @param AD4110_AFE_CRC_DISABLE Disables CRC for the AFE.
 * @param AD4110_AFE_CRC Enables 8-bit CRC on reads and writes for the AFE.
 ******************************************************************************/
enum ad4110_afe_crc_mode {
	AD4110_AFE_CRC_DISABLE,
	AD4110_AFE_CRC 	// 8-bit CRC on reads and writes.
};

/***************************************************************************//**
 * @brief The `ad4110_gain` enumeration defines a set of constants representing
 * different gain settings for the AD4110 device. Each enumerator
 * corresponds to a specific gain value that can be used to configure the
 * device's gain setting, allowing for precise control over the
 * amplification of input signals. This enumeration is part of the AD4110
 * driver, which facilitates interaction with the AD4110 analog-to-
 * digital converter.
 *
 * @param AD4110_GAIN_0_2 Represents a gain setting of 0.2.
 * @param AD4110_GAIN_0_25 Represents a gain setting of 0.25.
 * @param AD4110_GAIN_0_3 Represents a gain setting of 0.3.
 * @param AD4110_GAIN_0_375 Represents a gain setting of 0.375.
 * @param AD4110_GAIN_0_5 Represents a gain setting of 0.5.
 * @param AD4110_GAIN_0_75 Represents a gain setting of 0.75.
 * @param AD4110_GAIN_1 Represents a gain setting of 1.
 * @param AD4110_GAIN_1_5 Represents a gain setting of 1.5.
 * @param AD4110_GAIN_2 Represents a gain setting of 2.
 * @param AD4110_GAIN_3 Represents a gain setting of 3.
 * @param AD4110_GAIN_4 Represents a gain setting of 4.
 * @param AD4110_GAIN_6 Represents a gain setting of 6.
 * @param AD4110_GAIN_8 Represents a gain setting of 8.
 * @param AD4110_GAIN_12 Represents a gain setting of 12.
 * @param AD4110_GAIN_16 Represents a gain setting of 16.
 * @param AD4110_GAIN_24 Represents a gain setting of 24.
 ******************************************************************************/
enum ad4110_gain {
	AD4110_GAIN_0_2,
	AD4110_GAIN_0_25,
	AD4110_GAIN_0_3,
	AD4110_GAIN_0_375,
	AD4110_GAIN_0_5,
	AD4110_GAIN_0_75,
	AD4110_GAIN_1,
	AD4110_GAIN_1_5,
	AD4110_GAIN_2,
	AD4110_GAIN_3,
	AD4110_GAIN_4,
	AD4110_GAIN_6,
	AD4110_GAIN_8,
	AD4110_GAIN_12,
	AD4110_GAIN_16,
	AD4110_GAIN_24
};

/***************************************************************************//**
 * @brief The `ad4110_ain_buffer` enumeration defines the possible states for
 * configuring the analog input buffer of the AD4110 device. It allows
 * for enabling or disabling the input buffers, which can be set to
 * handle either the negative, positive, or both input signals. This
 * configuration is crucial for optimizing the input signal conditioning
 * based on the specific application requirements.
 *
 * @param DISABLE_AIN_BUFFER Represents the state where the analog input buffer
 * is disabled.
 * @param ENABLE_NEG_BUFFER Enables the negative input buffer for the analog
 * input.
 * @param ENABLE_POS_BUFFER Enables the positive input buffer for the analog
 * input.
 * @param ENABLE_FULL_BUFFER Enables both positive and negative input buffers
 * for the analog input.
 ******************************************************************************/
enum ad4110_ain_buffer {
	DISABLE_AIN_BUFFER,
	ENABLE_NEG_BUFFER,
	ENABLE_POS_BUFFER,
	ENABLE_FULL_BUFFER
};

/***************************************************************************//**
 * @brief The `ad4110_odr` enumeration defines a set of constants representing
 * various output data rates for the AD4110 device. These constants are
 * used to configure the device's data rate settings, which determine how
 * frequently data is output from the device. The enumeration includes a
 * range of kilohertz and samples per second settings, allowing for
 * flexible configuration based on the application's requirements.
 *
 * @param KSPS_125_A Represents a specific output data rate setting for the
 * AD4110 device.
 * @param KSPS_125_B Represents another specific output data rate setting for
 * the AD4110 device.
 * @param KSPS_62P5_A Represents a specific output data rate setting for the
 * AD4110 device.
 * @param KSPS_62P5_B Represents another specific output data rate setting for
 * the AD4110 device.
 * @param KSPS_31P25 Represents a specific output data rate setting for the
 * AD4110 device.
 * @param KSPS_25 Represents a specific output data rate setting for the AD4110
 * device.
 * @param KSPS_15P625 Represents a specific output data rate setting for the
 * AD4110 device.
 * @param KSPS_10P417 Represents a specific output data rate setting for the
 * AD4110 device.
 * @param KSPS_5 Represents a specific output data rate setting for the AD4110
 * device.
 * @param KSPS_2P5 Represents a specific output data rate setting for the AD4110
 * device.
 * @param KSPS_1 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_500 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_400P6 Represents a specific output data rate setting for the
 * AD4110 device.
 * @param SPS_200 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_100P2 Represents a specific output data rate setting for the
 * AD4110 device.
 * @param SPS_60 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_50 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_20 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_16P7 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_10 Represents a specific output data rate setting for the AD4110
 * device.
 * @param SPS_5 Represents a specific output data rate setting for the AD4110
 * device.
 ******************************************************************************/
enum ad4110_odr {
	KSPS_125_A,
	KSPS_125_B,
	KSPS_62P5_A,
	KSPS_62P5_B,
	KSPS_31P25,
	KSPS_25,
	KSPS_15P625,
	KSPS_10P417,
	KSPS_5,
	KSPS_2P5,
	KSPS_1,
	SPS_500,
	SPS_400P6,
	SPS_200,
	SPS_100P2,
	SPS_60,
	SPS_50,
	SPS_20,
	SPS_16P7,
	SPS_10,
	SPS_5
};

/***************************************************************************//**
 * @brief The `ad4110_order` enumeration defines the filter order options
 * available for the AD4110 device. It provides two specific filter
 * configurations: `sinc5_sinc1` and `sinc3`, each represented by a
 * unique hexadecimal value. This enumeration is used to configure the
 * filter order in the AD4110's ADC settings, allowing users to select
 * the desired filtering characteristics for their application.
 *
 * @param sinc5_sinc1 Represents the sinc5-sinc1 filter order with a value of
 * 0x0.
 * @param sinc3 Represents the sinc3 filter order with a value of 0x3.
 ******************************************************************************/
enum ad4110_order {
	sinc5_sinc1 = 0x0,
	sinc3 = 0x3
};

/***************************************************************************//**
 * @brief The `ad4110_dev` structure is a comprehensive configuration and state
 * descriptor for the AD4110 device, encapsulating all necessary settings
 * for its operation. It includes SPI communication details, device
 * settings such as voltage reference, data status, and word length, as
 * well as operational parameters like gain, synchronization, and clock
 * configurations. Additionally, it manages GPIO settings for continuous
 * mode operation, making it a central component for interfacing with and
 * controlling the AD4110 device.
 *
 * @param spi_dev Pointer to the SPI descriptor for communication.
 * @param volt_ref Specifies the voltage reference setting.
 * @param data_stat Indicates the data status.
 * @param data_length Defines the data word length.
 * @param adc_crc_en Specifies the ADC CRC mode.
 * @param afe_crc_en Specifies the AFE CRC mode.
 * @param op_mode Defines the operation mode of the device.
 * @param gain Specifies the gain setting.
 * @param sync Indicates if synchronization is enabled.
 * @param afe_clk Specifies the AFE clock configuration.
 * @param adc_clk Specifies the ADC clock selection.
 * @param addr Device address.
 * @param bipolar Indicates if the device is in bipolar mode.
 * @param analog_input_buff Specifies the analog input buffer configuration.
 * @param odr Specifies the output data rate.
 * @param order Defines the filter order.
 * @param irq_desc Pointer to the IRQ controller descriptor for GPIO.
 * @param nready_pin Specifies the pin used for readiness indication.
 ******************************************************************************/
struct ad4110_dev {
	/* SPI */
	struct no_os_spi_desc			*spi_dev;
	/* Device Settings */
	enum ad4110_voltage_reference volt_ref;
	enum ad4110_state		data_stat;
	enum ad4110_data_word_length 	data_length;
	enum ad4110_adc_crc_mode 	adc_crc_en;
	enum ad4110_afe_crc_mode	afe_crc_en;
	enum ad4110_op_mode 		op_mode;
	enum ad4110_gain 		gain;
	enum ad4110_sync_en 		sync;
	enum ad4110_afe_clk_cfg		afe_clk;
	enum ad4110_adc_clk_sel		adc_clk;
	uint8_t				addr;
	bool				bipolar;
	enum ad4110_ain_buffer		analog_input_buff;
	enum ad4110_odr			odr;
	enum ad4110_order		order;
	/* GPIO - used only for continuous mode */
	struct no_os_irq_ctrl_desc *irq_desc;
	uint32_t nready_pin;
};

/***************************************************************************//**
 * @brief The `ad4110_init_param` structure is used to initialize and configure
 * the AD4110 device, which is an analog-to-digital converter. It
 * includes parameters for SPI communication, device settings such as
 * voltage reference, data status, data length, CRC modes for both AFE
 * and ADC, operational mode, gain, synchronization, clock
 * configurations, and other settings like address, input polarity, and
 * buffer configurations. Additionally, it includes GPIO settings for
 * continuous mode operation, making it a comprehensive configuration
 * structure for setting up the AD4110 device.
 *
 * @param spi_init Initializes the SPI communication parameters.
 * @param volt_ref Specifies the voltage reference setting for the device.
 * @param data_stat Indicates the data status configuration.
 * @param data_length Defines the data word length for communication.
 * @param afe_crc_en Enables or disables CRC for the AFE.
 * @param adc_crc_en Enables or disables CRC for the ADC.
 * @param op_mode Sets the operational mode of the device.
 * @param gain Specifies the gain setting for the device.
 * @param sync Configures the synchronization setting.
 * @param afe_clk Selects the AFE clock configuration.
 * @param adc_clk Selects the ADC clock source.
 * @param addr Holds the device address.
 * @param bipolar Indicates if the input is bipolar or unipolar.
 * @param analog_input_buff Configures the analog input buffer setting.
 * @param odr Sets the output data rate.
 * @param order Defines the filter order for data processing.
 * @param irq_desc Describes the IRQ controller used for continuous mode.
 * @param nready_pin Specifies the pin used for the nREADY signal.
 ******************************************************************************/
struct ad4110_init_param {
	/* SPI */
	struct no_os_spi_init_param		spi_init;
	/* Device Settings */
	enum ad4110_voltage_reference volt_ref;
	enum ad4110_state		data_stat;
	enum ad4110_data_word_length 	data_length;
	enum ad4110_afe_crc_mode	afe_crc_en;
	enum ad4110_adc_crc_mode	adc_crc_en;
	enum ad4110_op_mode 		op_mode;
	enum ad4110_gain 		gain;
	enum ad4110_sync_en 		sync;
	enum ad4110_afe_clk_cfg		afe_clk;
	enum ad4110_adc_clk_sel		adc_clk;
	uint8_t				addr;
	bool				bipolar;
	enum ad4110_ain_buffer		analog_input_buff;
	enum ad4110_odr			odr;
	enum ad4110_order		order;
	/* GPIO - used only for continuous mode */
	struct no_os_irq_ctrl_desc *irq_desc;
	uint32_t nready_pin;
};

/***************************************************************************//**
 * @brief The `ad4110_callback_ctx` structure is used to manage the context for
 * callback operations related to the AD4110 device. It holds a reference
 * to the device through a pointer to an `ad4110_dev` structure, a buffer
 * for data storage, and the size of this buffer. This structure is
 * essential for handling data read operations in a callback-driven
 * manner, allowing for efficient data processing and management in
 * applications using the AD4110 device.
 *
 * @param dev A pointer to an `ad4110_dev` structure representing the device
 * context.
 * @param buffer A pointer to a buffer of type `uint32_t` used for storing data.
 * @param buffer_size An unsigned 32-bit integer representing the size of the
 * buffer.
 ******************************************************************************/
struct ad4110_callback_ctx {
	struct ad4110_dev *dev;
	uint32_t *buffer;
	uint32_t buffer_size;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Compute CRC8 checksum. */
/***************************************************************************//**
 * @brief Use this function to calculate an 8-bit CRC checksum for a block of
 * data, which is useful for error-checking purposes in data transmission
 * or storage. The function processes each byte of the input data and
 * applies a polynomial to compute the checksum. It is important to
 * ensure that the data pointer is valid and that the data size is
 * correctly specified to avoid undefined behavior.
 *
 * @param data A pointer to the data array for which the CRC checksum is to be
 * computed. Must not be null, and the caller retains ownership.
 * @param data_size The number of bytes in the data array to process. Must be a
 * non-negative integer; if zero, the function will return a
 * CRC of zero.
 * @return Returns an 8-bit unsigned integer representing the computed CRC
 * checksum.
 ******************************************************************************/
uint8_t ad4110_compute_crc8(uint8_t *data,
			    uint8_t data_size);
/* Compute XOR checksum. */
/***************************************************************************//**
 * @brief This function calculates an XOR checksum for an array of bytes, which
 * can be used for simple error detection in data transmission or
 * storage. It processes each byte in the provided data array and
 * computes the XOR of all bytes. This function is useful when a
 * lightweight checksum is needed. Ensure that the data array is not null
 * and that the data size does not exceed the buffer limit of 3 bytes, as
 * the function will only process up to 3 bytes.
 *
 * @param data A pointer to the array of bytes for which the XOR checksum is to
 * be computed. Must not be null, and the caller retains ownership.
 * @param data_size The number of bytes in the data array to process. Valid
 * values are 0 to 3, as the function processes up to 3 bytes.
 * @return Returns the computed XOR checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t ad4110_compute_xor(uint8_t *data,
			   uint8_t data_size);

/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function writes data to a specified register of the AD4110
 * device, applying a mask to ensure only certain bits are modified. It
 * is useful when only specific bits of a register need to be updated
 * without affecting other bits. The function first reads the current
 * register value, applies the mask to clear the bits to be modified, and
 * then writes the new data. It should be called when the device is
 * properly initialized and communication with the device is established.
 * The function returns an error code if the read or write operation
 * fails.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param reg_map Specifies the register map (e.g., ADC or AFE) to access. Must
 * be a valid map identifier.
 * @param reg_addr The address of the register within the specified map. Must be
 * a valid register address.
 * @param data The data to be written to the register. Only the bits specified
 * by the mask will be affected.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be updated with the
 * corresponding bits from data.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad4110_spi_int_reg_write_msk(struct ad4110_dev *dev,
				     uint8_t reg_map,
				     uint8_t reg_addr,
				     uint32_t data,
				     uint16_t mask);
/* Set the mode of the ADC. */
/***************************************************************************//**
 * @brief Use this function to configure the AD4110 ADC to operate in a specific
 * mode, such as continuous conversion or system calibration modes. This
 * function should be called when you need to change the ADC's
 * operational mode, which can affect how the ADC processes input
 * signals. Ensure that the device is properly initialized before calling
 * this function. The function logs informational messages when setting
 * system offset or gain calibration modes, assuming specific conditions
 * about the applied analog input.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param mode An enum value of type ad4110_adc_mode specifying the desired ADC
 * mode. Valid modes include continuous conversion, single
 * conversion, standby, power-down, system offset calibration, and
 * system gain calibration.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad4110_set_adc_mode(struct ad4110_dev *dev, enum ad4110_adc_mode mode);

/* Set the gain. */
/***************************************************************************//**
 * @brief This function configures the gain setting of the AD4110 device by
 * writing to the appropriate register. It should be called when the user
 * needs to adjust the gain for different measurement requirements. The
 * function requires a valid device structure and a gain value from the
 * predefined gain enumeration. It is important to ensure that the device
 * has been properly initialized before calling this function to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before use.
 * @param gain An enumerated value of type ad4110_gain representing the desired
 * gain setting. Must be a valid value from the ad4110_gain
 * enumeration.
 * @return Returns an int32_t indicating success or failure of the operation. A
 * non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad4110_set_gain(struct ad4110_dev *dev, enum ad4110_gain gain);

/* Set ADC clock mode. */
/***************************************************************************//**
 * @brief Use this function to configure the clock source for the ADC in the
 * AD4110 device. This function should be called when you need to change
 * the ADC clock settings, typically during the initialization phase or
 * when reconfiguring the device. Ensure that the device structure is
 * properly initialized before calling this function. The function
 * modifies the ADC clock selection register and returns an error code if
 * the operation fails.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param clk An enum value of type ad4110_adc_clk_sel specifying the desired
 * ADC clock source. Valid values are AD4110_ADC_INT_CLK,
 * AD4110_ADC_INT_CLK_CLKIO, and AD4110_ADC_EXT_CLK.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating failure.
 ******************************************************************************/
int32_t ad4110_set_adc_clk(struct ad4110_dev *dev, enum ad4110_adc_clk_sel clk);

/* Set AFE clock mode. */
/***************************************************************************//**
 * @brief This function sets the clock source for the Analog Front End (AFE) of
 * the AD4110 device. It should be called when configuring the device to
 * ensure the correct clock source is used for the AFE. The function
 * requires a valid device structure and a clock configuration
 * enumeration value. It returns an error code if the operation fails,
 * which can be used to verify successful configuration.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param clk An enumeration value of type ad4110_afe_clk_cfg specifying the
 * desired AFE clock configuration. Valid values are
 * AD4110_AFE_INT_CLOCK and AD4110_AFE_ADC_CLOCKED.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad4110_set_afe_clk(struct ad4110_dev *dev, enum ad4110_afe_clk_cfg clk);

/* Set voltage reference. */
/***************************************************************************//**
 * @brief This function configures the voltage reference for the AD4110 device,
 * which is essential for accurate analog-to-digital conversion. It
 * should be called after the device has been initialized and before
 * starting any conversion operations. The function supports both
 * internal and external reference options, and it handles the enabling
 * of the internal reference if selected. Proper selection of the
 * reference is crucial for the desired measurement accuracy.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param ref An enum value of type ad4110_voltage_reference specifying the
 * desired voltage reference. Valid values are AD4110_EXT_REF,
 * AD4110_INT_2_5V_REF, and AD4110_AVDD5_REF. If an invalid value is
 * provided, the function will not perform any operation.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad4110_set_reference(struct ad4110_dev *dev,
			     enum ad4110_voltage_reference ref);

/* Set the operation mode. */
/***************************************************************************//**
 * @brief This function configures the AD4110 device to operate in a specified
 * mode, such as voltage, current, or thermocouple mode. It must be
 * called after the device has been properly initialized. The function
 * modifies the device's internal registers to set the desired mode, and
 * it returns an error code if the operation fails. This function is
 * essential for adapting the device to different measurement
 * requirements and should be used whenever a change in operational mode
 * is needed.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @param mode An enum value of type ad4110_op_mode specifying the desired
 * operational mode. Valid values are defined in the ad4110_op_mode
 * enumeration. If an invalid mode is provided, the function returns
 * an error code.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad4110_set_op_mode(struct ad4110_dev *dev, enum ad4110_op_mode mode);

/* Do a SPI software reset. */
/***************************************************************************//**
 * @brief This function is used to reset the AD4110 device by sending a specific
 * sequence of bits over the SPI interface. It is typically called when
 * the device needs to be reinitialized or when it is in an unknown
 * state. The function requires a valid device structure that has been
 * properly initialized with SPI settings. It returns an integer status
 * code indicating the success or failure of the operation.
 *
 * @param dev A pointer to an ad4110_dev structure representing the device. This
 * must be a valid, non-null pointer with an initialized SPI device
 * field. If the pointer is null, the behavior is undefined.
 * @return Returns an int32_t status code from the SPI write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t ad4110_spi_do_soft_reset(struct ad4110_dev *dev);

/* Get the data size of a specified register. */
/***************************************************************************//**
 * @brief Use this function to retrieve the data size associated with a specific
 * register in the AD4110 device. This is useful when preparing to read
 * or write data to the device, as it informs the caller of the expected
 * data length. The function requires a valid device structure and
 * register identifiers. It assumes the device has been properly
 * initialized and configured. The data size varies based on the register
 * map and address, with special handling for certain ADC registers.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param reg_map An 8-bit unsigned integer specifying the register map. Valid
 * values are typically defined as constants, such as A4110_ADC.
 * @param reg_addr An 8-bit unsigned integer specifying the register address
 * within the map. Valid values are typically defined as
 * constants, such as AD4110_REG_ADC_STATUS.
 * @return Returns an 8-bit unsigned integer representing the data size in bytes
 * for the specified register.
 ******************************************************************************/
uint8_t ad4110_get_data_size(struct ad4110_dev *dev,
			     uint8_t reg_map,
			     uint8_t reg_addr);

/* SPI internal register write to device. */
/***************************************************************************//**
 * @brief This function is used to write a 24-bit or 32-bit data value to a
 * specified internal register of the AD4110 device using SPI
 * communication. It should be called when you need to configure or
 * update the settings of the AD4110 device. The function requires a
 * valid device structure, register map, register address, and data to be
 * written. It handles CRC calculation if enabled and returns an error
 * code if the operation fails. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an ad4110_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg_map An 8-bit value specifying the register map (AFE or ADC) to
 * which the data will be written. Valid values are A4110_ADC or
 * A4110_AFE.
 * @param reg_addr An 8-bit value specifying the address of the register within
 * the selected map. Must be a valid register address as defined
 * in the AD4110 register map.
 * @param reg_data A 32-bit value containing the data to be written to the
 * specified register. The data size is determined by the
 * register address, with some addresses requiring 24 bits and
 * others 32 bits.
 * @return Returns an int32_t status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int32_t ad4110_spi_int_reg_write(struct ad4110_dev *dev,
				 uint8_t reg_map,
				 uint8_t reg_addr,
				 uint32_t reg_data);

/* SPI internal register read from device. */
/***************************************************************************//**
 * @brief Use this function to read data from a specified register of the AD4110
 * device over the SPI interface. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function handles CRC checks if enabled, and it
 * returns an error code if the read operation fails or if a CRC error is
 * detected. This function is useful for retrieving configuration or
 * status information from the device.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param reg_map Specifies the register map (e.g., A4110_ADC or A4110_AFE) from
 * which to read. Must be a valid register map identifier.
 * @param reg_addr The address of the register to read within the specified
 * register map. Must be a valid register address.
 * @param reg_data A pointer to a uint32_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, a negative error code on failure, or -1 if a
 * CRC error is detected. The read data is stored in the location
 * pointed to by reg_data.
 ******************************************************************************/
int32_t ad4110_spi_int_reg_read(struct ad4110_dev *dev,
				uint8_t reg_map,
				uint8_t reg_addr,
				uint32_t *reg_data);

/* Fills buffer with buffer_size number of samples using irq */
/***************************************************************************//**
 * @brief This function is used to perform continuous data acquisition from the
 * AD4110 device, filling the provided buffer with the specified number
 * of samples. It sets up the device for continuous conversion mode and
 * uses interrupts to manage data transfer. The function must be called
 * with a properly initialized device structure and a buffer large enough
 * to hold the desired number of samples. It is important to ensure that
 * the device is fully initialized before calling this function. The
 * function will block until the buffer is completely filled.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param buffer A pointer to a uint32_t array where the samples will be stored.
 * Must not be null and should have enough space to hold
 * buffer_size samples.
 * @param buffer_size The number of samples to read into the buffer. Must be
 * greater than zero.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad4110_continuous_read(struct ad4110_dev *dev, uint32_t *buffer,
			       uint32_t buffer_size);

/* SPI internal DATA register read from device. */
/***************************************************************************//**
 * @brief This function retrieves data from the internal data register of an
 * AD4110 device using SPI communication. It should be called when you
 * need to read the latest conversion result from the device. The
 * function requires a valid device structure and a pointer to store the
 * read data. It handles different data sizes and checks for CRC errors
 * if CRC is enabled. Ensure the device is properly initialized and
 * configured before calling this function.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param reg_data A pointer to a uint32_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or if a CRC error is detected.
 ******************************************************************************/
int32_t ad4110_spi_int_data_reg_read(struct ad4110_dev *dev,
				     uint32_t *reg_data);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the AD4110 device using the provided
 * initialization parameters, setting up the SPI communication and
 * configuring the device settings such as CRC modes, data length,
 * synchronization, and clock settings. It must be called before any
 * other operations on the AD4110 device. The function allocates memory
 * for the device structure and configures the device according to the
 * specified parameters. If initialization fails at any step, it cleans
 * up and returns an error code.
 *
 * @param device A pointer to a pointer of type `struct ad4110_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad4110_init_param` containing
 * initialization parameters for the device, such as SPI
 * settings, voltage reference, data length, and other
 * configuration options. All fields must be properly set
 * before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and the device pointer is not valid.
 ******************************************************************************/
int32_t ad4110_setup(struct ad4110_dev **device,
		     struct ad4110_init_param init_param);

/* Enable/Disable channel */
/***************************************************************************//**
 * @brief This function is used to enable or disable a specific channel on the
 * AD4110 device. It should be called when you need to change the status
 * of a channel, either to start or stop data acquisition on that
 * channel. The function requires a valid device structure and a channel
 * identifier. It is important to ensure that the channel identifier is
 * within the valid range for the device. The function returns an integer
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to an ad4110_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param chan_id An unsigned 8-bit integer representing the channel ID to be
 * enabled or disabled. The value should be within the valid
 * range of channel IDs supported by the device.
 * @param status A boolean value indicating the desired status of the channel.
 * 'true' to enable the channel, 'false' to disable it.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int ad4110_set_channel_status(struct ad4110_dev *dev, uint8_t chan_id,
			      bool status);

/* Set analog input buffer */
/***************************************************************************//**
 * @brief This function sets the analog input buffer configuration for the
 * AD4110 device, which is essential for adjusting the input signal
 * conditioning. It should be called when the device is initialized and
 * before starting any data acquisition to ensure the input buffer is
 * correctly configured. The function updates the device's internal state
 * to reflect the new buffer setting. It is important to handle the
 * return value to check for any errors during the configuration process.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param buffer An enum value of type ad4110_ain_buffer indicating the desired
 * buffer configuration. Valid values are DISABLE_AIN_BUFFER,
 * ENABLE_NEG_BUFFER, ENABLE_POS_BUFFER, and ENABLE_FULL_BUFFER.
 * Invalid values may result in undefined behavior.
 * @return Returns 0 on success or a negative error code if the configuration
 * fails.
 ******************************************************************************/
int ad4110_set_analog_input_buffer(struct ad4110_dev *dev,
				   enum ad4110_ain_buffer buffer);

/* Set polarity */
/***************************************************************************//**
 * @brief This function configures the AD4110 ADC to operate in either bipolar
 * or unipolar mode based on the provided parameter. It should be called
 * when there is a need to switch the ADC's input range configuration.
 * The function requires a valid device structure and will update the
 * device's configuration accordingly. It returns an error code if the
 * operation fails, otherwise it returns 0. Ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param bipolar A boolean value where true sets the ADC to bipolar mode and
 * false sets it to unipolar mode.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad4110_set_bipolar(struct ad4110_dev *dev, bool bipolar);

/* Set ODR */
/***************************************************************************//**
 * @brief This function configures the output data rate (ODR) of the AD4110
 * device. It should be called when you need to change the data rate at
 * which the device samples and outputs data. The function requires a
 * valid device structure and a valid ODR enumeration value. It updates
 * the device's internal configuration to reflect the new ODR setting. If
 * the operation is successful, the function returns 0; otherwise, it
 * returns an error code indicating the failure.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * device. Must not be null.
 * @param odr An enumeration value of type ad4110_odr representing the desired
 * output data rate. Must be a valid value from the ad4110_odr enum.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad4110_set_odr(struct ad4110_dev *dev, enum ad4110_odr odr);

/* Set filter order */
/***************************************************************************//**
 * @brief Use this function to configure the filter order of the AD4110 device,
 * which affects the digital filtering applied to the ADC data. This
 * function should be called after the device has been initialized and
 * before starting data acquisition. It updates the device's internal
 * configuration to reflect the specified filter order. Ensure that the
 * `dev` parameter is a valid, initialized device structure. The function
 * returns an error code if the operation fails, allowing the caller to
 * handle such cases appropriately.
 *
 * @param dev A pointer to an initialized `ad4110_dev` structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param order An `enum ad4110_order` value specifying the desired filter
 * order. Valid values are `sinc5_sinc1` and `sinc3`. Invalid
 * values may result in undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation did not complete successfully.
 ******************************************************************************/
int ad4110_set_order(struct ad4110_dev *dev, enum ad4110_order order);

/* Fills the buffer with a single sample */
/***************************************************************************//**
 * @brief Use this function to perform a single conversion with the AD4110 ADC
 * and retrieve the conversion result. This function sets the ADC to
 * single conversion mode, waits for the conversion to complete, and then
 * reads the result into the provided buffer. It must be called with a
 * properly initialized `ad4110_dev` structure. The function returns an
 * error code if the ADC mode cannot be set, if the conversion does not
 * complete within the timeout, or if the data cannot be read.
 *
 * @param dev A pointer to an initialized `ad4110_dev` structure representing
 * the ADC device. Must not be null.
 * @param buffer A pointer to a `uint32_t` where the conversion result will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during mode setting, conversion, or data reading.
 ******************************************************************************/
int ad4110_do_single_read(struct ad4110_dev *dev, uint32_t *buffer);

/* Wait for conversion completion  */
/***************************************************************************//**
 * @brief This function is used to wait until the ADC is ready to provide data,
 * indicated by the RDY bit being low. It should be called when a data
 * conversion is expected, and the caller needs to ensure that the ADC is
 * ready before attempting to read data. The function will poll the ADC
 * status register until the RDY bit is cleared or the specified timeout
 * is reached. It is important to provide a reasonable timeout value to
 * avoid indefinite blocking.
 *
 * @param dev A pointer to an initialized ad4110_dev structure representing the
 * ADC device. Must not be null.
 * @param timeout The maximum number of polling attempts before the function
 * gives up. Must be a positive integer. If the timeout is
 * reached before the ADC is ready, the function will return an
 * error.
 * @return Returns 0 if the ADC is ready before the timeout expires, or
 * -ETIMEDOUT if the timeout is reached without the ADC becoming ready.
 * Returns a negative error code if a read error occurs.
 ******************************************************************************/
int ad4110_wait_for_rdy_low(struct ad4110_dev *dev, uint32_t timeout);

#endif // AD4110_H_
