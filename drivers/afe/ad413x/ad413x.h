/***************************************************************************//**
 *   @file   ad413x.h
 *   @brief  Header file of AD413X Driver.
 *   @author Andrei Porumb (andrei.porumb@analog.com)
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

#ifndef AD413X_H_
#define AD413X_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_irq.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD413X_CMD_WR_COM_REG(x)	(0x00 | ((x) & 0x3F)) // Write to Register x
#define AD413X_CMD_RD_COM_REG(x) 	(0x40 | ((x) & 0x3F)) // Read from Register x

/*************************** ADC Register Lengths *****************************/
#define AD413X_R1B				(1 << 8)
#define AD413X_R2B				(2 << 8)
#define AD413X_R3B				(3 << 8)
#define AD413X_TRANSF_LEN(x)			(((x) >> 8) & 0xFF)

/***************************** ADC Register Map *******************************/
#define AD413X_REG_STATUS 			(AD413X_R1B | 0x0)
#define AD413X_REG_ADC_CTRL			(AD413X_R2B | 0x1)
#define AD413X_REG_DATA				(AD413X_R3B | 0x2)
#define AD413X_REG_IO_CTRL			(AD413X_R2B | 0x3)
#define AD413X_REG_VBIAS_CTRL			(AD413X_R2B | 0x4)
#define AD413X_REG_ID				(AD413X_R1B | 0x5)
#define AD413X_REG_ERROR			(AD413X_R2B | 0x6)
#define AD413X_REG_ERROR_EN			(AD413X_R2B | 0x7)
#define AD413X_REG_MCLK_CNT			(AD413X_R1B | 0x8)
#define AD413X_REG_CHN(x)			(AD413X_R3B | (0x09U + (x)))
#define AD413X_REG_CONFIG(x)			(AD413X_R2B | (0x19U + (x)))
#define AD413X_REG_FILTER(x)			(AD413X_R3B | (0x21U + (x)))
#define AD413X_REG_OFFSET(x)			(AD413X_R3B | (0x29U + (x)))
#define AD413X_REG_GAIN(x)			(AD413X_R3B | (0x31U + (x)))
#define AD413X_REG_MISC				(AD413X_R2B | 0x39)
#define AD413X_REG_FIFO_CTRL			(AD413X_R3B | 0x3A)
#define AD413X_REG_FIFO_STS			(AD413X_R1B | 0x3B)
#define AD413X_REG_FIFO_THRSHLD			(AD413X_R3B | 0x3C)
#define AD413X_REG_FIFO_DATA			(AD413X_R3B | 0x3D)


/* ADC_CONTROL Register */
#define AD413X_ADC_BIPOLAR				NO_OS_BIT(14)
#define AD413X_ADC_REF_VAL				NO_OS_BIT(13)
#define AD413X_ADC_DOUT_DIS_DEL				NO_OS_BIT(12)
#define AD413X_ADC_CONT_READ				NO_OS_BIT(11)
#define AD413X_ADC_DATA_STATUS				NO_OS_BIT(10)
#define AD413X_ADC_CSB_EN				NO_OS_BIT(9)
#define AD413X_ADC_REF_EN				NO_OS_BIT(8)
#define AD413X_ADC_DUTY_CYC_RATIO			NO_OS_BIT(6)
#define AD413X_ADC_MODE(x)				(((x) & 0xF) << 2)
#define AD413X_ADC_CNTRL_MCLK(x)			((x) & 0x3)

/* IO_CONTROL Register */
#define AD413X_SYNCB_CLEAR 				NO_OS_BIT(10)
#define AD413X_INT_PIN_SEL(x) 				(((x) & 0x3) << 8)
#define AD413X_GPO_DATA_P4 				NO_OS_BIT(7)
#define AD413X_GPO_DATA_P3 				NO_OS_BIT(6)
#define AD413X_GPO_DATA_P2 				NO_OS_BIT(5)
#define AD413X_GPO_DATA_P1 				NO_OS_BIT(4)
#define AD413X_GPO_CTRL_P4		 		NO_OS_BIT(3)
#define AD413X_GPO_CTRL_P3		 		NO_OS_BIT(2)
#define AD413X_GPO_CTRL_P2		 		NO_OS_BIT(1)
#define AD413X_GPO_CTRL_P1		 		0x01

/* VBIAS_CTRL Register */
#define AD413X_VBIAS_15					NO_OS_BIT(15)
#define AD413X_VBIAS_14					NO_OS_BIT(14)
#define AD413X_VBIAS_13					NO_OS_BIT(13)
#define AD413X_VBIAS_12					NO_OS_BIT(12)
#define AD413X_VBIAS_11					NO_OS_BIT(11)
#define AD413X_VBIAS_10					NO_OS_BIT(10)
#define AD413X_VBIAS_9					NO_OS_BIT(9)
#define AD413X_VBIAS_8					NO_OS_BIT(8)
#define AD413X_VBIAS_7					NO_OS_BIT(7)
#define AD413X_VBIAS_6					NO_OS_BIT(6)
#define AD413X_VBIAS_5					NO_OS_BIT(5)
#define AD413X_VBIAS_4					NO_OS_BIT(4)
#define AD413X_VBIAS_3					NO_OS_BIT(3)
#define AD413X_VBIAS_2					NO_OS_BIT(2)
#define AD413X_VBIAS_1					NO_OS_BIT(1)
#define AD413X_VBIAS_0					0x01

/* ERROR Register */
#define AD413X_AINP_OV_UV_ERR				NO_OS_BIT(11)
#define AD413X_AINM_OV_UV_ERR				NO_OS_BIT(10)
#define AD413X_REF_OV_UV_ERR				NO_OS_BIT(9)
#define AD413X_REF_DETECT_ERR				NO_OS_BIT(8)
#define AD413X_ADC_ERR					NO_OS_BIT(7)
#define AD413X_SPI_IGNORE_ERR				NO_OS_BIT(6)
#define AD413X_SPI_SCLK_CNT_ERR				NO_OS_BIT(5)
#define AD413X_SPI_READ_ERR				NO_OS_BIT(4)
#define AD413X_SPI_WRITE_ERR				NO_OS_BIT(3)
#define AD413X_SPI_CRC_ERR				NO_OS_BIT(2)
#define AD413X_MM_CRC_ERR				NO_OS_BIT(1)
#define AD413X_ROM_CRC_ERR				0x01

/* ERROR_EN Register */
#define AD413X_MCLK_CNT_EN				NO_OS_BIT(12)
#define AD413X_AINP_OV_UV_ERR_EN			NO_OS_BIT(11)
#define AD413X_AINM_OV_UV_ERR_EN			NO_OS_BIT(10)
#define AD413X_REF_OV_UV_ERR_EN				NO_OS_BIT(9)
#define AD413X_REF_DETECT_ERR_EN			NO_OS_BIT(8)
#define AD413X_ADC_ERR_EN				NO_OS_BIT(7)
#define AD413X_SPI_IGNORE_ERR_EN			NO_OS_BIT(6)
#define AD413X_SPI_SCLK_CNT_ERR_EN			NO_OS_BIT(5)
#define AD413X_SPI_READ_ERR_EN				NO_OS_BIT(4)
#define AD413X_SPI_WRITE_ERR_EN				NO_OS_BIT(3)
#define AD413X_SPI_CRC_ERR_EN				NO_OS_BIT(2)
#define AD413X_MM_CRC_ERR_EN				NO_OS_BIT(1)
#define AD413X_ROM_CRC_ERR_EN				0x01

/* CHANNEL_M Register */
#define AD413X_ENABLE_M					NO_OS_BIT(23)
#define AD413X_SETUP_M(x)				(((x) & 0x7) << 20)
#define AD413X_PDSW_M					NO_OS_BIT(19)
#define AD413X_THRES_EN_M				NO_OS_BIT(18)
#define AD413X_AINP_M(x)				(((x) & 0x1F) << 13)
#define AD413X_AINM_M(x)				(((x) & 0x1F) <<  8)
#define AD413X_I_OUT1_CH_M(x)				(((x) & 0xF) <<   4)
#define AD413X_I_OUT0_CH_M(x)				((x) & 0xF)
#define AD413X_I_OUT_CH_MSK				NO_OS_GENMASK(7,0)

/* CONFIG_N Register */
#define AD413X_I_OUT1_N(x)				(((x) & 0x7) << 13)
#define AD413X_I_OUT0_N(x)				(((x) & 0x7) << 10)
#define AD413X_I_OUT_MSK				NO_OS_GENMASK(15,10)
#define AD413X_BURNOUT_N(x)				(((x) & 0x3) <<  8)
#define AD413X_REF_BUF_MSK				NO_OS_GENMASK(7,6)
#define AD413X_REF_BUFP_N				NO_OS_BIT(7)
#define AD413X_REF_BUFM_N				NO_OS_BIT(6)
#define AD413X_REF_SEL_N(x)				(((x) & 0x3) <<  4)
#define AD413X_PGA_N(x)					(((x) & 0x7) <<  1)
#define AD413X_PGA_BYP_N				0x01

/* FILTER_N Register */
#define AD413X_SETTLE_N(x)				(((x) & 0x7) <<  21)
#define AD413X_REPEAT_N(x)				(((x) & 0x1F) << 16)
#define AD413X_FILTER_MODE_N(x)				(((x) & 0xF) <<  12)
#define AD413X_FS_N(x)					((x) & 0x7FF)

/* OFFSET_N Register */
#define AD413X_OFFSET_N(x)				((x) & 0xFFFFFF)

/* GAIN_N Register */
#define AD413X_GAIN_N(x)				((x) & 0xFFFFFF)

/* MISC Register */
#define AD413X_BYPASS_OSC				NO_OS_BIT(15)
#define AD413X_PD_ALDO					NO_OS_BIT(14)
#define AD413X_CAL_RANGE_X2				NO_OS_BIT(13)
#define AD413X_RESERVED(x)				(((x) & 0xF) << 9)
#define AD413X_STBY_CTRL_MSK				NO_OS_GENMASK(8,0)
#define AD413X_STBY_OUT_EN				NO_OS_BIT(8)
#define AD413X_STBY_DIAGNOSTICS_EN			NO_OS_BIT(7)
#define AD413X_STBY_GPO_EN				NO_OS_BIT(6)
#define AD413X_STBY_PDSW_EN				NO_OS_BIT(5)
#define AD413X_STBY_BURNOUT_EN				NO_OS_BIT(4)
#define AD413X_STBY_VBIAS_EN				NO_OS_BIT(3)
#define AD413X_STBY_IEXC_EN				NO_OS_BIT(2)
#define AD413X_STBY_REFHOL_EN				NO_OS_BIT(1)
#define AD413X_STBY_INTREF_EN				0x01

/* FIFO_CONTROL Register */
#define AD413X_ADD_FIFO_STATUS				NO_OS_BIT(19)
#define AD413X_ADD_FIFO_HEADER				NO_OS_BIT(18)
#define AD413X_FIFO_MODE(x) 				(((x) & 0x3) << 16)
#define AD413X_FIFO_WRITE_ERR_INT_EN 			NO_OS_BIT(14)
#define AD413X_FIFO_READ_ERR_INT_EN 			NO_OS_BIT(13)
#define AD413X_THRES_HIGH_INT_EN 			NO_OS_BIT(12)
#define AD413X_THRES_LOW_INT_EN  			NO_OS_BIT(11)
#define AD413X_OVERRUN_INT_EN 				NO_OS_BIT(10)
#define AD413X_WATERMARK_INT_EN 			NO_OS_BIT(9)
#define AD413X_EMPTY_INT_EN 				NO_OS_BIT(8)
#define AD413X_WATERMARK(x) 				((x) & 0xFF)

/* FIFO_STATUS Register */
#define AD413X_MASTER_ERR 				NO_OS_BIT(7)
#define AD413X_FIFO_WRITE_ERR 				NO_OS_BIT(6)
#define AD413X_FIFO_READ_ERR 				NO_OS_BIT(5)
#define AD413X_THRES_HIGH_FLAG 				NO_OS_BIT(4)
#define AD413X_THRES_LOW_FLAG 				NO_OS_BIT(3)
#define AD413X_OVERRUN_FLAG 				NO_OS_BIT(2)
#define AD413X_WATERMARK_FLAG 				NO_OS_BIT(1)
#define AD413X_EMPTY_FLAG 				0x01

/* FIFO_THRESHOLD Register */
#define AD413X_THRES_HIGH_VAL(x)		(((x) & 0xFFF) << 12)
#define AD413X_THRES_LOW_VAL(x)			((x) & 0xFFF)

/* 8-bits wide checksum generated using the polynomial */
#define AD413X_CRC8_POLY	0x07 // x^8 + x^2 + x^1 + x^0

#define ENABLE				1U
#define DISABLE				0U

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief The `ad413x_input` enumeration defines various input sources for the
 * AD413X ADC channels, including analog input channels, temperature
 * sensor input, internal and external reference voltages, and various
 * supply and ground references. This enumeration is used to specify the
 * input source for each channel in the AD413X ADC configuration.
 *
 * @param AD413X_AIN0 Analog input channel 0.
 * @param AD413X_AIN1 Analog input channel 1.
 * @param AD413X_AIN2 Analog input channel 2.
 * @param AD413X_AIN3 Analog input channel 3.
 * @param AD413X_AIN4 Analog input channel 4.
 * @param AD413X_AIN5 Analog input channel 5.
 * @param AD413X_AIN6 Analog input channel 6.
 * @param AD413X_AIN7 Analog input channel 7.
 * @param AD413X_AIN8 Analog input channel 8.
 * @param AD413X_AIN9 Analog input channel 9.
 * @param AD413X_AIN10 Analog input channel 10.
 * @param AD413X_AIN11 Analog input channel 11.
 * @param AD413X_AIN12 Analog input channel 12.
 * @param AD413X_AIN13 Analog input channel 13.
 * @param AD413X_AIN14 Analog input channel 14.
 * @param AD413X_AIN15 Analog input channel 15.
 * @param AD413X_TEMP Temperature sensor input.
 * @param AD413X_AVSS Analog ground reference.
 * @param AD413X_INT_REF Internal reference voltage.
 * @param AD413X_DGND Digital ground reference.
 * @param AD413X_AVDD_AVSS_6P Positive analog supply to ground reference.
 * @param AD413X_AVDD_AVSS_6M Negative analog supply to ground reference.
 * @param AD413X_IOVDD_DGND_6P Positive I/O supply to digital ground reference.
 * @param AD413X_IOVDD_DGND_6M Negative I/O supply to digital ground reference.
 * @param AD413X_ALDO_AVSS_6P Positive analog LDO to ground reference.
 * @param AD413X_ALDO_AVSS_6M Negative analog LDO to ground reference.
 * @param AD413X_DLDO_DGND_6P Positive digital LDO to ground reference.
 * @param AD413X_DLDO_DGND_6M Negative digital LDO to ground reference.
 * @param AD413X_V_MV_P Positive millivolt reference.
 * @param AD413X_V_MV_M Negative millivolt reference.
 ******************************************************************************/
enum ad413x_input {
	AD413X_AIN0,
	AD413X_AIN1,
	AD413X_AIN2,
	AD413X_AIN3,
	AD413X_AIN4,
	AD413X_AIN5,
	AD413X_AIN6,
	AD413X_AIN7,
	AD413X_AIN8,
	AD413X_AIN9,
	AD413X_AIN10,
	AD413X_AIN11,
	AD413X_AIN12,
	AD413X_AIN13,
	AD413X_AIN14,
	AD413X_AIN15,
	AD413X_TEMP,
	AD413X_AVSS,
	AD413X_INT_REF,
	AD413X_DGND,
	AD413X_AVDD_AVSS_6P,
	AD413X_AVDD_AVSS_6M,
	AD413X_IOVDD_DGND_6P,
	AD413X_IOVDD_DGND_6M,
	AD413X_ALDO_AVSS_6P,
	AD413X_ALDO_AVSS_6M,
	AD413X_DLDO_DGND_6P,
	AD413X_DLDO_DGND_6M,
	AD413X_V_MV_P,
	AD413X_V_MV_M,
};

/***************************************************************************//**
 * @brief The `ad413x_preset_nb` enumeration defines a set of constants
 * representing different preset configurations for the AD413X device.
 * Each constant corresponds to a specific preset number, ranging from 0
 * to 7, which can be used to select predefined settings for the device's
 * operation. This enumeration is useful for managing and switching
 * between different configurations in a structured and readable manner.
 *
 * @param AD413X_PRESET_0 Represents the first preset configuration.
 * @param AD413X_PRESET_1 Represents the second preset configuration.
 * @param AD413X_PRESET_2 Represents the third preset configuration.
 * @param AD413X_PRESET_3 Represents the fourth preset configuration.
 * @param AD413X_PRESET_4 Represents the fifth preset configuration.
 * @param AD413X_PRESET_5 Represents the sixth preset configuration.
 * @param AD413X_PRESET_6 Represents the seventh preset configuration.
 * @param AD413X_PRESET_7 Represents the eighth preset configuration.
 ******************************************************************************/
enum ad413x_preset_nb {
	AD413X_PRESET_0,
	AD413X_PRESET_1,
	AD413X_PRESET_2,
	AD413X_PRESET_3,
	AD413X_PRESET_4,
	AD413X_PRESET_5,
	AD413X_PRESET_6,
	AD413X_PRESET_7
};

/***************************************************************************//**
 * @brief The `ad413x_mclk_sel` enumeration defines the possible master clock
 * options for the AD413X device, allowing selection between internal and
 * external clock sources with specific frequencies and configurations.
 * This selection is crucial for setting the clock source that drives the
 * ADC operations, impacting the device's performance and compatibility
 * with other system components.
 *
 * @param AD413X_INT_76_8_KHZ_OUT_OFF Represents the internal 76.8 kHz clock
 * with output off.
 * @param AD413X_INT_76_8_KHZ_OUT_ON Represents the internal 76.8 kHz clock with
 * output on.
 * @param AD413X_EXT_76_8KHZ Represents an external 76.8 kHz clock.
 * @param AD413X_EXT_153_6_KHZ_DIV_2 Represents an external 153.6 kHz clock
 * divided by 2.
 ******************************************************************************/
enum ad413x_mclk_sel {
	AD413X_INT_76_8_KHZ_OUT_OFF,
	AD413X_INT_76_8_KHZ_OUT_ON,
	AD413X_EXT_76_8KHZ,
	AD413X_EXT_153_6_KHZ_DIV_2
};

/***************************************************************************//**
 * @brief The `ad413x_adc_mode` enumeration defines various operational modes
 * for the AD413X ADC, each represented by a unique integer value. These
 * modes include continuous and single conversion modes, standby and
 * power down modes, as well as several calibration modes for offset and
 * gain. Additionally, it includes modes for duty cycling and
 * synchronization, allowing for flexible control over the ADC's
 * operation to suit different application needs.
 *
 * @param AD413X_CONTINOUS_CONV_MODE Represents continuous conversion mode with
 * a value of 0.
 * @param AD413X_SINGLE_CONV_MODE Represents single conversion mode with a value
 * of 1.
 * @param AD413X_STANDBY_MODE Represents standby mode with a value of 2.
 * @param AD413X_PW_DOWN_MODE Represents power down mode with a value of 3.
 * @param AD413X_IDLE_MODE Represents idle mode with a value of 4.
 * @param AD413X_INT_OFFSET_CAL Represents internal offset calibration mode with
 * a value of 5.
 * @param AD413X_INT_GAIN_CAL Represents internal gain calibration mode with a
 * value of 6.
 * @param AD413X_SYS_OFFSET_CAL Represents system offset calibration mode with a
 * value of 7.
 * @param AD413X_SYS_GAIN_CAL Represents system gain calibration mode with a
 * value of 8.
 * @param AD413X_DUTY_CYCLING_MODE Represents duty cycling mode with a value of
 * 9.
 * @param AD413X_SINGLE_CONV_SYNC_IDLE_MODE Represents single conversion sync
 * idle mode with a value of 10.
 * @param AD413X_DUTY_CYCLING_SYNC_STBY_MODE Represents duty cycling sync
 * standby mode with a value of 11.
 ******************************************************************************/
enum ad413x_adc_mode {
	AD413X_CONTINOUS_CONV_MODE = 0,
	AD413X_SINGLE_CONV_MODE = 1,
	AD413X_STANDBY_MODE = 2,
	AD413X_PW_DOWN_MODE = 3,
	AD413X_IDLE_MODE = 4,
	AD413X_INT_OFFSET_CAL = 5,
	AD413X_INT_GAIN_CAL = 6,
	AD413X_SYS_OFFSET_CAL = 7,
	AD413X_SYS_GAIN_CAL = 8,
	AD413X_DUTY_CYCLING_MODE = 9,
	AD413X_SINGLE_CONV_SYNC_IDLE_MODE = 10,
	AD413X_DUTY_CYCLING_SYNC_STBY_MODE = 11
};

/***************************************************************************//**
 * @brief The `ad413x_ref_buf` structure is used to manage the enabling of
 * reference buffers in the AD413X ADC driver. It contains two boolean
 * fields that specify whether the positive and negative reference
 * buffers are enabled, allowing for configuration of the ADC's reference
 * voltage buffering capabilities.
 *
 * @param ref_buf_p_en Indicates if the positive reference buffer is enabled.
 * @param ref_buf_m_en Indicates if the negative reference buffer is enabled.
 ******************************************************************************/
struct ad413x_ref_buf {
	bool ref_buf_p_en;
	bool ref_buf_m_en;
};

/***************************************************************************//**
 * @brief The `ad413x_ref_sel` enumeration defines the possible reference
 * voltage selections for the AD413X ADC device. It includes options for
 * using external reference inputs, as well as internal references
 * between AVDD and AVSS. This selection is crucial for configuring the
 * ADC's reference voltage, which impacts the accuracy and range of the
 * analog-to-digital conversion.
 *
 * @param AD413X_REFIN1 Represents the first external reference input.
 * @param AD413X_REFIN2 Represents the second external reference input.
 * @param AD413X_REFOUT_AVSS Represents the reference output with AVSS.
 * @param AD413X_AVDD_AVSS Represents the reference between AVDD and AVSS.
 ******************************************************************************/
enum ad413x_ref_sel {
	AD413X_REFIN1,
	AD413X_REFIN2,
	AD413X_REFOUT_AVSS,
	AD413X_AVDD_AVSS
};

/***************************************************************************//**
 * @brief The `ad413x_int_ref` enumeration defines the possible states for the
 * internal reference voltage of the AD413X device. It allows the
 * selection between disabling the internal reference or setting it to
 * either 2.5V or 1.25V, providing flexibility in configuring the
 * device's reference voltage according to application requirements.
 *
 * @param AD413X_INTREF_DISABLED Represents the state where the internal
 * reference is disabled.
 * @param AD413X_INTREF_2_5V Represents the state where the internal reference
 * is set to 2.5 volts.
 * @param AD413X_INTREF_1_25V Represents the state where the internal reference
 * is set to 1.25 volts.
 ******************************************************************************/
enum ad413x_int_ref {
	AD413X_INTREF_DISABLED,
	AD413X_INTREF_2_5V,
	AD413X_INTREF_1_25V
};

/***************************************************************************//**
 * @brief The `ad413x_filter` enumeration defines various filter configurations
 * available for the AD413X series of devices. These filters are used to
 * process the input signals with different synchronization and rejection
 * characteristics, allowing for flexible signal conditioning tailored to
 * specific application needs. Each enumerator represents a distinct
 * filter mode, such as standalone, synchronized, or with specific
 * rejection or programmable filter settings.
 *
 * @param AD413X_SYNC4_STANDALONE Represents a standalone filter mode with SYNC4
 * configuration.
 * @param AD413X_SYNC4_SYNC1 Represents a SYNC4 filter mode synchronized with
 * SYNC1.
 * @param AD413X_SYNC3_STANDALONE Represents a standalone filter mode with SYNC3
 * configuration.
 * @param AD413X_SYNC3_REJ60 Represents a SYNC3 filter mode with 60Hz rejection.
 * @param AD413X_SYNC3_SYNC1 Represents a SYNC3 filter mode synchronized with
 * SYNC1.
 * @param AD413X_SYNC3_PF1 Represents a SYNC3 filter mode with programmable
 * filter 1.
 * @param AD413X_SYNC3_PF2 Represents a SYNC3 filter mode with programmable
 * filter 2.
 * @param AD413X_SYNC3_PF3 Represents a SYNC3 filter mode with programmable
 * filter 3.
 * @param AD413X_SYNC3_PF4 Represents a SYNC3 filter mode with programmable
 * filter 4.
 ******************************************************************************/
enum ad413x_filter {
	AD413X_SYNC4_STANDALONE,
	AD413X_SYNC4_SYNC1,
	AD413X_SYNC3_STANDALONE,
	AD413X_SYNC3_REJ60,
	AD413X_SYNC3_SYNC1,
	AD413X_SYNC3_PF1,
	AD413X_SYNC3_PF2,
	AD413X_SYNC3_PF3,
	AD413X_SYNC3_PF4
};

/***************************************************************************//**
 * @brief The `ad413x_gain` enumeration defines a set of constants representing
 * different gain settings for the AD413X series of analog-to-digital
 * converters. Each constant corresponds to a specific gain factor that
 * can be applied to the input signal, allowing for amplification of the
 * signal before conversion. This is useful in applications where the
 * input signal is weak and needs to be amplified to improve the
 * resolution and accuracy of the conversion process.
 *
 * @param AD413X_GAIN_1 Represents a gain factor of 1.
 * @param AD413X_GAIN_2 Represents a gain factor of 2.
 * @param AD413X_GAIN_4 Represents a gain factor of 4.
 * @param AD413X_GAIN_8 Represents a gain factor of 8.
 * @param AD413X_GAIN_16 Represents a gain factor of 16.
 * @param AD413X_GAIN_32 Represents a gain factor of 32.
 * @param AD413X_GAIN_64 Represents a gain factor of 64.
 * @param AD413X_GAIN_128 Represents a gain factor of 128.
 ******************************************************************************/
enum ad413x_gain {
	AD413X_GAIN_1,
	AD413X_GAIN_2,
	AD413X_GAIN_4,
	AD413X_GAIN_8,
	AD413X_GAIN_16,
	AD413X_GAIN_32,
	AD413X_GAIN_64,
	AD413X_GAIN_128,
};

/***************************************************************************//**
 * @brief The `ad413x_chip_id` enumeration defines identifiers for different
 * AD413X series chips. Currently, it includes a single member,
 * `AD4130_8`, which is used to identify the AD4130-8 chip variant. This
 * enumeration is useful for distinguishing between different chip models
 * within the AD413X series, allowing for specific handling or
 * configuration based on the chip type.
 *
 * @param AD4130_8 Represents the chip ID for the AD4130-8 device, with a value
 * of 0x04.
 ******************************************************************************/
enum ad413x_chip_id {
	AD4130_8 = 0x04
};

/***************************************************************************//**
 * @brief The `ad413x_settle_time` enumeration defines various settle times for
 * the AD413X device, expressed in terms of master clock cycles (MCLK).
 * These values are used to configure the time it takes for the ADC to
 * settle after a change in input or configuration, which is crucial for
 * ensuring accurate readings. The settle times range from 32 to 4096
 * MCLK cycles, allowing for flexibility in balancing speed and accuracy
 * in different application scenarios.
 *
 * @param AD413X_32_MCLK Represents a settle time of 32 master clock cycles.
 * @param AD413X_64_MCLK Represents a settle time of 64 master clock cycles.
 * @param AD413X_128_MCLK Represents a settle time of 128 master clock cycles.
 * @param AD413X_256_MCLK Represents a settle time of 256 master clock cycles.
 * @param AD413X_512_MCLK Represents a settle time of 512 master clock cycles.
 * @param AD413X_1024_MCLK Represents a settle time of 1024 master clock cycles.
 * @param AD413X_2048_MCLK Represents a settle time of 2048 master clock cycles.
 * @param AD413X_4096_MCLK Represents a settle time of 4096 master clock cycles.
 ******************************************************************************/
enum ad413x_settle_time {
	AD413X_32_MCLK,
	AD413X_64_MCLK,
	AD413X_128_MCLK,
	AD413X_256_MCLK,
	AD413X_512_MCLK,
	AD413X_1024_MCLK,
	AD413X_2048_MCLK,
	AD413X_4096_MCLK
};

/***************************************************************************//**
 * @brief The `ad413x_exc_current` enumeration defines various levels of
 * excitation current that can be used in the AD413X device. These levels
 * range from being completely off to specific microampere and nanoampere
 * values, allowing for precise control over the excitation current used
 * in the device's operations. This enumeration is crucial for
 * configuring the device to meet specific application requirements
 * regarding current excitation.
 *
 * @param AD413X_EXC_OFF Represents the state where excitation current is turned
 * off.
 * @param AD413X_EXC_10UA Represents an excitation current of 10 microamperes.
 * @param AD413X_EXC_20UA Represents an excitation current of 20 microamperes.
 * @param AD413X_EXC_50UA Represents an excitation current of 50 microamperes.
 * @param AD413X_EXC_100UA Represents an excitation current of 100 microamperes.
 * @param AD413X_EXC_150UA Represents an excitation current of 150 microamperes.
 * @param AD413X_EXC_200UA Represents an excitation current of 200 microamperes.
 * @param AD413X_EXC_100NA Represents an excitation current of 100 nanoamperes.
 ******************************************************************************/
enum ad413x_exc_current {
	AD413X_EXC_OFF,
	AD413X_EXC_10UA,
	AD413X_EXC_20UA,
	AD413X_EXC_50UA,
	AD413X_EXC_100UA,
	AD413X_EXC_150UA,
	AD413X_EXC_200UA,
	AD413X_EXC_100NA
};

/***************************************************************************//**
 * @brief The `ad413x_standby_ctrl` structure is used to manage various control
 * flags for enabling or disabling specific features of the AD413X device
 * when it is in standby mode. Each member of the structure is a boolean
 * flag that corresponds to a particular feature, such as internal
 * reference, excitation current, or diagnostics, allowing for fine-
 * grained control over the device's behavior in low-power standby
 * conditions.
 *
 * @param standby_int_ref_en Indicates if the internal reference is enabled in
 * standby mode.
 * @param standby_ref_holder_en Indicates if the reference holder is enabled in
 * standby mode.
 * @param standby_iexc_en Indicates if the excitation current is enabled in
 * standby mode.
 * @param standby_vbias_en Indicates if the voltage bias is enabled in standby
 * mode.
 * @param standby_burnout_en Indicates if the burnout current is enabled in
 * standby mode.
 * @param standby_pdsw_en Indicates if the power-down switch is enabled in
 * standby mode.
 * @param standby_gpio_en Indicates if the GPIO is enabled in standby mode.
 * @param standby_diagn_en Indicates if diagnostics are enabled in standby mode.
 * @param standby_output_en Indicates if the output is enabled in standby mode.
 ******************************************************************************/
struct ad413x_standby_ctrl {
	bool standby_int_ref_en;
	bool standby_ref_holder_en;
	bool standby_iexc_en;
	bool standby_vbias_en;
	bool standby_burnout_en;
	bool standby_pdsw_en;
	bool standby_gpio_en;
	bool standby_diagn_en;
	bool standby_output_en;
};

/***************************************************************************//**
 * @brief The `ad413x_preset` structure is used to define a set of configuration
 * parameters for the AD413X ADC device. It includes settings for the
 * reference buffer, reference selection, gain, filter type, settle time,
 * and excitation currents for two outputs. This structure allows for the
 * encapsulation of a complete configuration preset that can be applied
 * to the ADC to tailor its operation to specific requirements.
 *
 * @param ref_buf A structure representing the reference buffer configuration.
 * @param ref_sel An enumeration for selecting the ADC reference.
 * @param gain An enumeration for setting the gain of the ADC.
 * @param filter An enumeration for selecting the filter type.
 * @param s_time An enumeration for setting the channel settle time, defaulting
 * to 32MCLK.
 * @param iout0_exc_current An enumeration for setting the excitation current
 * for output 0.
 * @param iout1_exc_current An enumeration for setting the excitation current
 * for output 1.
 ******************************************************************************/
struct ad413x_preset {
	struct ad413x_ref_buf ref_buf;
	enum ad413x_ref_sel ref_sel;
	enum ad413x_gain gain;
	enum ad413x_filter filter;
	/* By default, settle time = 32MCLK */
	enum ad413x_settle_time s_time;
	enum ad413x_exc_current iout0_exc_current;
	enum ad413x_exc_current iout1_exc_current;
};

/***************************************************************************//**
 * @brief The `ad413x_channel` structure is used to configure individual
 * channels of the AD413X ADC device. It includes settings for selecting
 * input sources, enabling or disabling the channel, and configuring
 * excitation currents and power-down switches. This structure allows for
 * flexible channel configuration, enabling the user to tailor the ADC's
 * behavior to specific application requirements.
 *
 * @param preset Specifies the preset configuration number for the channel.
 * @param enable Indicates whether the channel is enabled (1) or disabled (0).
 * @param ain_p Defines the positive analog input source for the channel.
 * @param ain_m Defines the negative analog input source for the channel.
 * @param iout0_exc_input Specifies the input source for the first excitation
 * current.
 * @param iout1_exc_input Specifies the input source for the second excitation
 * current.
 * @param pdsw_en Indicates whether the power-down switch is enabled (true) or
 * disabled (false).
 ******************************************************************************/
struct ad413x_channel {
	enum ad413x_preset_nb preset;
	uint8_t enable;
	enum ad413x_input ain_p;
	enum ad413x_input ain_m;
	enum ad413x_input iout0_exc_input;
	enum ad413x_input iout1_exc_input;
	bool pdsw_en;
};

/***************************************************************************//**
 * @brief The `ad413x_dev` structure is a comprehensive representation of the
 * AD413X device configuration and state, encapsulating all necessary
 * settings and interfaces for operation. It includes pointers to SPI and
 * GPIO descriptors for communication and conversion readiness detection,
 * respectively. The structure also holds arrays for preset and channel
 * configurations, allowing for flexible device setup. Various enums and
 * flags within the structure manage the device's operational mode, clock
 * selection, reference settings, and other critical parameters, ensuring
 * precise control over the ADC's functionality.
 *
 * @param spi_dev Pointer to the SPI descriptor for communication.
 * @param rdy_pin_desc Pointer to the GPIO descriptor used to detect when
 * conversion is ready.
 * @param preset Array of 8 preset configurations for the device.
 * @param ch Array of 16 channel configurations for the device.
 * @param chip_id Enum representing the chip ID of the device.
 * @param mclk Enum representing the master clock selection.
 * @param op_mode Enum representing the operational mode of the ADC.
 * @param bipolar 8-bit value indicating if the ADC is in bipolar mode.
 * @param int_ref Enum representing the internal reference selection.
 * @param v_bias 16-bit value for the voltage bias setting.
 * @param standby_ctrl Structure containing standby control flags.
 * @param data_stat 8-bit value indicating if data status is enabled.
 * @param spi_crc_en 8-bit value indicating if SPI CRC is enabled.
 ******************************************************************************/
struct ad413x_dev {
	/* SPI */
	struct no_os_spi_desc			*spi_dev;
	/* GPIO - used to know when conversion is rdy */
	struct no_os_gpio_desc *rdy_pin_desc;
	/* Device Settings */
	struct ad413x_preset preset[8];
	struct ad413x_channel ch[16];
	enum ad413x_chip_id chip_id;
	enum ad413x_mclk_sel mclk;
	enum ad413x_adc_mode op_mode;
	uint8_t bipolar;
	enum ad413x_int_ref int_ref;
	uint16_t v_bias;
	struct ad413x_standby_ctrl standby_ctrl;
	uint8_t data_stat;
	uint8_t spi_crc_en;
};

/***************************************************************************//**
 * @brief The `ad413x_init_param` structure is used to initialize the AD413X
 * device with specific settings, including SPI and GPIO configurations,
 * device presets, channel settings, and operational parameters such as
 * chip ID, master clock selection, and reference voltage. It also
 * includes flags for enabling bipolar mode, data status, and SPI CRC
 * error checking, as well as settings for voltage bias and standby
 * control. This structure is essential for configuring the device before
 * use.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param rdy_pin_init Pointer to GPIO initialization parameters for the ready
 * pin.
 * @param preset Array of 8 preset configurations for the device.
 * @param ch Array of 16 channel configurations.
 * @param chip_id Identifier for the specific chip model.
 * @param mclk Master clock selection for the device.
 * @param bipolar Flag indicating if the ADC operates in bipolar mode.
 * @param int_ref Internal reference voltage selection.
 * @param v_bias Voltage bias setting for the device.
 * @param standby_ctrl Configuration for standby control settings.
 * @param data_stat Flag to enable or disable data status.
 * @param spi_crc_en Flag to enable or disable SPI CRC error checking.
 ******************************************************************************/
struct ad413x_init_param {
	/* SPI */
	struct no_os_spi_init_param	*spi_init;
	/* GPIO - used to know when conversion is rdy */
	struct no_os_gpio_init_param *rdy_pin_init;
	/* Device Settings */
	struct ad413x_preset preset[8];
	struct ad413x_channel ch[16];
	enum ad413x_chip_id chip_id;
	enum ad413x_mclk_sel mclk;
	uint8_t bipolar;
	enum ad413x_int_ref int_ref;
	uint16_t v_bias;
	struct ad413x_standby_ctrl standby_ctrl;
	uint8_t data_stat;
	uint8_t spi_crc_en;
};

/***************************************************************************//**
 * @brief The `ad413x_callback_ctx` structure is designed to hold context
 * information for callback operations related to the AD413X device. It
 * includes a pointer to the device structure, a buffer for data storage,
 * and the size of this buffer, facilitating data handling during device
 * operations.
 *
 * @param dev A pointer to an `ad413x_dev` structure representing the device
 * context.
 * @param buffer A pointer to a buffer of type `uint32_t` used for storing data.
 * @param buffer_size A `uint32_t` representing the size of the buffer.
 ******************************************************************************/
struct ad413x_callback_ctx {
	struct ad413x_dev *dev;
	uint32_t *buffer;
	uint32_t buffer_size;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* SPI write to device using a mask. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in an
 * AD413X device by applying a mask. It first reads the current value of
 * the register, applies the mask to clear the bits that need to be
 * updated, and then writes the new data to those bits. This function is
 * useful when only certain bits of a register need to be changed without
 * affecting the others. It must be called with a valid device structure
 * and register address. The function returns an error code if the read
 * or write operation fails.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD413X device.
 * @param data The data to be written to the register. Only the bits specified
 * by the mask will be affected.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be updated with the
 * corresponding bits from the data parameter.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered during the read or write
 * operation.
 ******************************************************************************/
int32_t ad413x_reg_write_msk(struct ad413x_dev *dev,
			     uint32_t reg_addr,
			     uint32_t data,
			     uint32_t mask);

/* Set the mode of the ADC. */
/***************************************************************************//**
 * @brief Use this function to configure the AD413X ADC device to operate in a
 * specific mode, such as continuous conversion or standby. This function
 * should be called when you need to change the operational mode of the
 * ADC, for example, to switch between different power-saving modes or to
 * start a specific type of conversion. Ensure that the device structure
 * is properly initialized before calling this function. The function
 * updates the device's operational mode and returns an error code if the
 * operation fails.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * ADC device. Must not be null.
 * @param mode An enum value of type ad413x_adc_mode representing the desired
 * ADC mode. Valid modes are defined in the ad413x_adc_mode
 * enumeration.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad413x_set_adc_mode(struct ad413x_dev *dev, enum ad413x_adc_mode mode);

/* Set the internal reference. */
/***************************************************************************//**
 * @brief This function configures the internal reference voltage of the AD413x
 * device to one of the specified options. It should be called when the
 * device is initialized and before starting any conversions that require
 * a specific reference voltage. The function updates the device's
 * internal state to reflect the selected reference. If an invalid
 * reference is provided, the function returns an error. It is important
 * to ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param int_ref An enum value of type ad413x_int_ref specifying the desired
 * internal reference voltage. Valid values are
 * AD413X_INTREF_DISABLED, AD413X_INTREF_2_5V, and
 * AD413X_INTREF_1_25V. Invalid values result in an error.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if an invalid reference is specified.
 ******************************************************************************/
int32_t ad413x_set_int_ref(struct ad413x_dev *dev, enum ad413x_int_ref int_ref);

/* Enable/disable DATA_STAT. */
/***************************************************************************//**
 * @brief This function is used to control the data status feature of an AD413X
 * device, which can be enabled or disabled based on the provided
 * parameter. It should be called when there is a need to toggle the data
 * status functionality, typically after the device has been initialized.
 * The function modifies the device's internal state to reflect the
 * change and returns an error code if the operation fails.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param enable A uint8_t value indicating whether to enable (non-zero) or
 * disable (zero) the data status feature. Any non-zero value will
 * enable the feature.
 * @return Returns an int32_t value, which is 0 on success or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad413x_data_stat_en(struct ad413x_dev *dev, uint8_t enable);

/* Set the gain. */
/***************************************************************************//**
 * @brief This function configures the gain setting for a specific preset
 * configuration on the AD413X device. It should be used when you need to
 * adjust the gain for a particular preset, which is identified by the
 * `reg_nb` parameter. The function must be called with a valid device
 * structure that has been properly initialized. It updates the device's
 * internal preset configuration with the specified gain. If the
 * operation fails, an error code is returned. Ensure that the `dev`
 * parameter is not null and that `reg_nb` corresponds to a valid preset
 * index.
 *
 * @param dev A pointer to an initialized `ad413x_dev` structure representing
 * the device. Must not be null.
 * @param gain An `enum ad413x_gain` value specifying the desired gain setting.
 * Must be a valid gain option.
 * @param reg_nb An `enum ad413x_preset_nb` value indicating which preset
 * configuration to modify. Must be a valid preset number.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_gain(struct ad413x_dev *dev, enum ad413x_gain gain,
			enum ad413x_preset_nb reg_nb);

/* Set the reference. */
/***************************************************************************//**
 * @brief This function configures the reference voltage selection for a given
 * preset in the AD413X device. It should be used when you need to change
 * the reference voltage setting for a specific preset configuration. The
 * function must be called with a valid device structure and appropriate
 * enumeration values for the reference selection and preset number. It
 * is important to ensure that the device is properly initialized before
 * calling this function. The function updates the device's internal
 * preset configuration and returns an error code if the operation fails.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param ref An enumeration value of type ad413x_ref_sel indicating the desired
 * reference voltage selection. Must be a valid enum value.
 * @param reg_nb An enumeration value of type ad413x_preset_nb specifying the
 * preset number to configure. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad413x_set_ref(struct ad413x_dev *dev, enum ad413x_ref_sel ref,
		       enum ad413x_preset_nb reg_nb);

/* Set the reference buffers */
/***************************************************************************//**
 * @brief This function is used to set the reference buffer configuration for a
 * specific preset in the AD413X device. It should be called when you
 * need to enable or disable the positive and negative reference buffers
 * for a given preset configuration. Ensure that the device structure is
 * properly initialized before calling this function. The function
 * updates the device's configuration register and modifies the internal
 * preset settings accordingly. It returns an error code if the operation
 * fails.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param ref_buf A structure of type ad413x_ref_buf specifying the reference
 * buffer settings. It contains boolean fields to enable or
 * disable the positive and negative reference buffers.
 * @param reg_nb An enum value of type ad413x_preset_nb indicating the preset
 * number to configure. Must be a valid preset number.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_ref_buf(struct ad413x_dev *dev,
			   struct ad413x_ref_buf ref_buf,
			   enum ad413x_preset_nb reg_nb);

/* Set the filter */
/***************************************************************************//**
 * @brief Use this function to configure the filter settings for a specific
 * preset on the AD413X device. This function should be called when you
 * need to change the filter type for a given preset number. Ensure that
 * the device structure is properly initialized before calling this
 * function. The function updates the device's internal preset
 * configuration with the specified filter type. It returns an error code
 * if the operation fails, which should be checked to ensure the filter
 * was set successfully.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param filter An enum value of type ad413x_filter specifying the filter type
 * to set. Must be a valid filter type defined in the
 * ad413x_filter enumeration.
 * @param reg_nb An enum value of type ad413x_preset_nb indicating the preset
 * number to configure. Must be a valid preset number defined in
 * the ad413x_preset_nb enumeration.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_filter(struct ad413x_dev *dev, enum ad413x_filter filter,
			  enum ad413x_preset_nb reg_nb);

/* Set settle time */
/***************************************************************************//**
 * @brief This function configures the settle time for a given preset on the
 * AD413X device. It should be used when you need to adjust the settle
 * time for specific measurement conditions or requirements. The function
 * must be called with a valid device structure and appropriate
 * enumeration values for the settle time and preset number. It updates
 * the device's internal configuration and returns an error code if the
 * operation fails.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param s_time An enumeration value of type ad413x_settle_time representing
 * the desired settle time. Must be a valid value from the
 * ad413x_settle_time enum.
 * @param reg_nb An enumeration value of type ad413x_preset_nb representing the
 * preset number to configure. Must be a valid value from the
 * ad413x_preset_nb enum.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad413x_set_settle_time(struct ad413x_dev *dev,
			       enum ad413x_settle_time s_time,
			       enum ad413x_preset_nb reg_nb);

/* Set excitation currents */
/***************************************************************************//**
 * @brief This function configures the excitation current for two output
 * channels (iout0 and iout1) within a specified preset configuration of
 * the AD413X device. It should be used when you need to adjust the
 * excitation current settings for a particular preset, which is
 * identified by the reg_nb parameter. The function must be called with a
 * valid device structure and appropriate enumeration values for the
 * excitation currents and preset number. It returns an error code if the
 * operation fails, otherwise it returns 0.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param iout0_exc An enum ad413x_exc_current value representing the desired
 * excitation current for the iout0 output. Must be a valid
 * enumeration value.
 * @param iout1_exc An enum ad413x_exc_current value representing the desired
 * excitation current for the iout1 output. Must be a valid
 * enumeration value.
 * @param reg_nb An enum ad413x_preset_nb value indicating which preset
 * configuration to modify. Must be a valid preset number.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_exc_current(struct ad413x_dev *dev,
			       enum ad413x_exc_current iout0_exc,
			       enum ad413x_exc_current iout1_exc,
			       enum ad413x_preset_nb reg_nb);

/* Set excitation source pins */
/***************************************************************************//**
 * @brief This function sets the excitation source pins for a specified channel
 * on the AD413X device. It should be used when configuring the
 * excitation current paths for a channel, typically during the device
 * setup phase. The function requires a valid device structure and
 * channel number, and it updates the device's internal configuration to
 * reflect the specified excitation inputs. It is important to ensure
 * that the channel number and input selections are within valid ranges
 * to avoid configuration errors.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param ch_nb The channel number to configure, ranging from 0 to 15. Values
 * outside this range may result in undefined behavior.
 * @param iout0_exc_inp An enum value of type ad413x_input specifying the input
 * source for IOUT0. Must be a valid input source as
 * defined in the ad413x_input enum.
 * @param iout1_exc_inp An enum value of type ad413x_input specifying the input
 * source for IOUT1. Must be a valid input source as
 * defined in the ad413x_input enum.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int32_t ad413x_ch_exc_input(struct ad413x_dev *dev, uint8_t ch_nb,
			    enum ad413x_input iout0_exc_inp,
			    enum ad413x_input iout1_exc_inp);

/* Set channel preset */
/***************************************************************************//**
 * @brief This function assigns a preset configuration to a specific channel on
 * the AD413X device. It should be used when you need to apply a
 * predefined set of settings to a channel, such as gain, filter, and
 * reference configurations. The function requires a valid device
 * structure and channel number, and it updates the device's internal
 * state to reflect the new preset. Ensure that the device is properly
 * initialized before calling this function. If the operation fails, an
 * error code is returned.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param ch_nb The channel number to which the preset will be applied. Valid
 * range is typically 0 to 15, depending on the device
 * configuration.
 * @param preset_nb An enum value of type ad413x_preset_nb representing the
 * preset configuration to apply. Must be a valid preset
 * number.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_ch_preset(struct ad413x_dev *dev, uint8_t ch_nb,
			     enum ad413x_preset_nb preset_nb);

/* Enable channel */
/***************************************************************************//**
 * @brief This function is used to enable or disable a specific channel on the
 * AD413X device. It should be called when you need to control the
 * activation state of a channel, typically during device configuration
 * or operation changes. The function requires a valid device structure
 * and a channel number within the supported range. It modifies the
 * channel's enable state based on the provided enable parameter. Ensure
 * that the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be initialized.
 * @param ch_nb The channel number to be enabled or disabled. Must be within the
 * valid range of channels supported by the device.
 * @param enable A uint8_t value where non-zero enables the channel and zero
 * disables it.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad413x_ch_en(struct ad413x_dev *dev, uint8_t ch_nb, uint8_t enable);

/* Enable PDSW */
/***************************************************************************//**
 * @brief This function is used to enable or disable the Power-Down Switch
 * (PDSW) for a specific channel on the AD413X device. It should be
 * called when you need to control the power state of a channel,
 * typically to save power when the channel is not in use. The function
 * requires a valid device structure and a channel number within the
 * supported range. It modifies the device's internal state to reflect
 * the PDSW setting for the specified channel.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param ch_nb The channel number for which to enable or disable the PDSW. Must
 * be within the valid range of channels supported by the device.
 * @param pdsw_en A boolean value indicating whether to enable (true) or disable
 * (false) the PDSW for the specified channel.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad413x_pdsw_en(struct ad413x_dev *dev, uint8_t ch_nb, bool pdsw_en);

/* Set output VBIAS */
/***************************************************************************//**
 * @brief This function configures the voltage bias setting for an AD413X device
 * by writing the specified voltage bias value to the appropriate control
 * register. It should be called when you need to adjust the voltage bias
 * for the device, typically during initialization or configuration
 * changes. The function requires a valid device structure and a voltage
 * bias value. It returns an error code if the operation fails, ensuring
 * that the caller can handle such cases appropriately.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param v_bias_val A 16-bit unsigned integer representing the voltage bias
 * value to be set. The valid range is determined by the
 * device's specifications, and invalid values may result in
 * an error return.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad413x_set_v_bias(struct ad413x_dev *dev, uint16_t v_bias_val);

/* Set standby control flags */
/***************************************************************************//**
 * @brief This function sets the standby control flags for the AD413X device,
 * allowing the user to enable or disable various standby features such
 * as internal reference, reference holder, excitation current, and more.
 * It should be called when the device is in a state where these settings
 * can be safely modified, typically during initialization or
 * configuration phases. The function updates the device's internal state
 * to reflect the new standby control settings. It is important to ensure
 * that the `dev` parameter is a valid, initialized device structure
 * before calling this function.
 *
 * @param dev A pointer to an initialized `ad413x_dev` structure representing
 * the device. Must not be null.
 * @param standby_ctrl A structure of type `ad413x_standby_ctrl` containing the
 * desired standby control settings. Each field in the
 * structure is a boolean flag indicating whether a specific
 * standby feature should be enabled or disabled.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad413x_set_standby_ctrl(struct ad413x_dev *dev,
				struct ad413x_standby_ctrl standby_ctrl);

/* Set ADC master clock mode. */
/***************************************************************************//**
 * @brief This function configures the master clock source for the AD413X
 * device. It should be called when you need to change the clock source,
 * which can be internal or external, depending on the application
 * requirements. Ensure that the device is properly initialized before
 * calling this function. The function updates the device's internal
 * state to reflect the new clock setting. If the operation fails, an
 * error code is returned.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param clk An enum value of type ad413x_mclk_sel specifying the desired
 * master clock source. Valid values are defined in the
 * ad413x_mclk_sel enumeration.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_set_mclk(struct ad413x_dev *dev, enum ad413x_mclk_sel clk);

/* Do a SPI software reset. */
/***************************************************************************//**
 * @brief Use this function to reset the AD413X device by sending a specific
 * sequence of bits through the SPI interface. This function should be
 * called when a reset of the device is required, such as during
 * initialization or to recover from an error state. Ensure that the
 * device structure is properly initialized and that the SPI interface is
 * configured before calling this function. The function will introduce a
 * brief delay to allow the reset to take effect.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device to reset. Must not be null. The function will return an
 * error if the SPI write operation fails.
 * @return Returns 0 on success, or a negative error code if the SPI write
 * operation fails.
 ******************************************************************************/
int32_t ad413x_do_soft_reset(struct ad413x_dev *dev);

/* SPI internal register write to device. */
/***************************************************************************//**
 * @brief This function is used to write a 1, 2, or 3-byte data value to a
 * specified register on an AD413X device via SPI communication. It
 * should be called when you need to configure or update the settings of
 * the device by writing to its registers. The function requires a valid
 * device structure and register address, and it handles data size based
 * on the register's expected length. If the device is configured to use
 * CRC, a CRC byte is appended to the data. The function returns an error
 * code if the data size is invalid or if the SPI write operation fails.
 *
 * @param dev A pointer to an ad413x_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function.
 * @param reg_addr The address of the register to write to. The address must be
 * valid and correspond to a register on the AD413X device.
 * @param reg_data The data to write to the register. The data size must match
 * the expected size for the specified register, which can be 1,
 * 2, or 3 bytes.
 * @return Returns 0 on success, or a negative error code if the data size is
 * invalid or the SPI write operation fails.
 ******************************************************************************/
int32_t ad413x_reg_write(struct ad413x_dev *dev,
			 uint32_t reg_addr,
			 uint32_t reg_data);

/* SPI internal register read from device. */
/***************************************************************************//**
 * @brief Use this function to read the value of a specified register from an
 * AD413X device. It requires a valid device structure and a register
 * address to read from. The function will store the read value in the
 * provided output parameter. Ensure that the device is properly
 * initialized before calling this function. The function handles SPI
 * communication and checks for CRC errors if enabled. It returns an
 * error code if the read operation fails or if invalid parameters are
 * provided.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address defined for the AD413X device.
 * @param reg_data A pointer to a uint32_t where the read register data will be
 * stored. Must not be null.
 * @return Returns 0 on success, a negative error code on failure, such as
 * -EBADMSG for CRC errors or -EINVAL for invalid parameters.
 ******************************************************************************/
int32_t ad413x_reg_read(struct ad413x_dev *dev,
			uint32_t reg_addr,
			uint32_t *reg_data);

/* Fills buffer with data read from each active channel */
/***************************************************************************//**
 * @brief This function initiates a single conversion mode on the AD413X ADC
 * device for the specified number of channels and stores the conversion
 * results in the provided buffer. It should be called when a single
 * conversion is required for a set of active channels. The function will
 * block until all conversions are complete or a timeout occurs. Ensure
 * the device is properly initialized and configured before calling this
 * function. The function returns an error code if the conversion fails
 * or times out.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * ADC device. Must not be null.
 * @param buffer A pointer to a uint32_t array where the conversion results will
 * be stored. Must not be null and should have space for at least
 * ch_nb elements.
 * @param ch_nb The number of channels to read. Must be greater than zero and
 * should not exceed the number of active channels configured on
 * the device.
 * @return Returns 0 on success, a negative error code on failure, or -ETIMEDOUT
 * if the operation times out.
 ******************************************************************************/
int32_t ad413x_single_conv(struct ad413x_dev *dev, uint32_t *buffer,
			   uint8_t ch_nb);

/* Fills buffer with data from continuous conv mode from each active channel */
/***************************************************************************//**
 * @brief This function is used to perform continuous analog-to-digital
 * conversions on the specified number of channels and samples, storing
 * the results in the provided buffer. It must be called with a properly
 * initialized device structure and a buffer large enough to hold the
 * data for all specified channels and samples. The function sets the ADC
 * to continuous conversion mode, reads data until the buffer is filled,
 * and then returns the ADC to standby mode. It handles timeouts and
 * returns an error if the conversion is not completed in a timely
 * manner.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * ADC device. Must not be null.
 * @param buffer A pointer to a uint32_t array where the conversion data will be
 * stored. Must not be null and should have enough space for ch_nb
 * * sample_nb entries.
 * @param ch_nb The number of channels to read from. Must be a valid channel
 * number configured on the device.
 * @param sample_nb The number of samples to read per channel. Must be a
 * positive integer.
 * @return Returns 0 on success, a negative error code on failure, or -ETIMEDOUT
 * if the operation times out.
 ******************************************************************************/
int32_t ad413x_continuous_conv(struct ad413x_dev *dev, uint32_t *buffer,
			       uint8_t ch_nb, uint32_t sample_nb);

/* Set adc bipolar/unipolar coding. */
/***************************************************************************//**
 * @brief Use this function to configure the ADC for either bipolar or unipolar
 * coding, depending on the application's requirements. This function
 * should be called after the device has been initialized and before
 * starting any conversions. The function modifies the device's internal
 * state to reflect the selected coding mode. It returns an error code if
 * the operation fails, which should be checked to ensure the
 * configuration was successful.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param enable A uint8_t value where non-zero enables bipolar coding and zero
 * enables unipolar coding. The caller retains ownership.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad413x_adc_bipolar(struct ad413x_dev *dev, uint8_t enable);

/* Store presets for adc channels */
/***************************************************************************//**
 * @brief Use this function to store a set of configuration parameters as a
 * preset for an AD413X device. This function should be called when you
 * need to save specific settings such as gain, reference selection,
 * reference buffer, settling time, excitation currents, and filter type
 * for later use. Ensure that the device is properly initialized before
 * calling this function. The function will return an error code if any
 * of the configuration steps fail, allowing you to handle errors
 * appropriately.
 *
 * @param dev A pointer to an initialized ad413x_dev structure representing the
 * device. Must not be null.
 * @param preset An ad413x_preset structure containing the configuration
 * settings to be stored. The structure must be properly populated
 * with valid settings.
 * @param preset_nb An enum value of type ad413x_preset_nb indicating which
 * preset slot to store the configuration in. Must be a valid
 * preset number.
 * @return Returns an int32_t error code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during the configuration
 * process.
 ******************************************************************************/
int32_t ad413x_preset_store(struct ad413x_dev *dev, struct ad413x_preset preset,
			    enum ad413x_preset_nb preset_nb);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes an AD413X device using the provided
 * initialization parameters, setting up the SPI interface, GPIO for the
 * ready pin, and configuring device settings such as presets, channels,
 * and operational modes. It must be called before any other operations
 * on the device. The function allocates memory for the device structure
 * and configures the device according to the specified parameters. If
 * initialization fails at any step, it cleans up and returns an error
 * code.
 *
 * @param device A pointer to a pointer of type `struct ad413x_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad413x_init_param` containing
 * the initialization parameters for the device. This includes
 * SPI and GPIO initialization parameters, as well as device-
 * specific settings like presets, channels, and operational
 * modes. The structure must be fully populated with valid
 * data before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and the device pointer is set to NULL.
 ******************************************************************************/
int32_t ad413x_init(struct ad413x_dev **device,
		    struct ad413x_init_param init_param);

/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly remove an AD413X device instance and
 * free any resources associated with it. This function should be called
 * when the device is no longer needed, ensuring that the SPI interface
 * is properly closed and the memory allocated for the device structure
 * is released. It is important to ensure that the device pointer is
 * valid and initialized before calling this function to avoid undefined
 * behavior.
 *
 * @param dev A pointer to the ad413x_dev structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. If the pointer is invalid, the
 * behavior is undefined.
 * @return Returns 0 on success, or a negative error code if the SPI removal
 * fails.
 ******************************************************************************/
int32_t ad413x_remove(struct ad413x_dev *dev);

#endif // AD413X_H_
