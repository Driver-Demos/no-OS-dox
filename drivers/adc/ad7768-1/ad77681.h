/***************************************************************************//**
 *   @file   ad77681.h
 *   @brief  Header file of the AD7768-1 Driver.
 *   @author SPopa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2017(c) Analog Devices, Inc.
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

#ifndef SRC_AD77681_H_
#define SRC_AD77681_H_

#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define	AD77681_REG_CHIP_TYPE					0x3
#define	AD77681_REG_PROD_ID_L					0x4
#define	AD77681_REG_PROD_ID_H					0x5
#define	AD77681_REG_CHIP_GRADE					0x6
#define	AD77681_REG_SCRATCH_PAD					0x0A
#define	AD77681_REG_VENDOR_L					0x0C
#define	AD77681_REG_VENDOR_H					0x0D
#define	AD77681_REG_INTERFACE_FORMAT			0x14
#define AD77681_REG_POWER_CLOCK					0x15
#define AD77681_REG_ANALOG						0x16
#define AD77681_REG_ANALOG2						0x17
#define AD77681_REG_CONVERSION					0x18
#define AD77681_REG_DIGITAL_FILTER				0x19
#define AD77681_REG_SINC3_DEC_RATE_MSB			0x1A
#define AD77681_REG_SINC3_DEC_RATE_LSB			0x1B
#define AD77681_REG_DUTY_CYCLE_RATIO			0x1C
#define AD77681_REG_SYNC_RESET					0x1D
#define AD77681_REG_GPIO_CONTROL				0x1E
#define AD77681_REG_GPIO_WRITE					0x1F
#define AD77681_REG_GPIO_READ					0x20
#define AD77681_REG_OFFSET_HI					0x21
#define AD77681_REG_OFFSET_MID					0x22
#define AD77681_REG_OFFSET_LO					0x23
#define AD77681_REG_GAIN_HI						0x24
#define AD77681_REG_GAIN_MID					0x25
#define AD77681_REG_GAIN_LO						0x26
#define AD77681_REG_SPI_DIAG_ENABLE				0x28
#define AD77681_REG_ADC_DIAG_ENABLE				0x29
#define AD77681_REG_DIG_DIAG_ENABLE				0x2A
#define	AD77681_REG_ADC_DATA					0x2C
#define	AD77681_REG_MASTER_STATUS				0x2D
#define	AD77681_REG_SPI_DIAG_STATUS				0x2E
#define	AD77681_REG_ADC_DIAG_STATUS				0x2F
#define	AD77681_REG_DIG_DIAG_STATUS				0x30
#define	AD77681_REG_MCLK_COUNTER				0x31

/* AD77681_REG_INTERFACE_FORMAT */
#define AD77681_INTERFACE_CRC_EN_MSK			(0x1 << 6)
#define AD77681_INTERFACE_CRC_EN(x)				(((x) & 0x1) << 6)
#define AD77681_INTERFACE_CRC_TYPE_MSK			(0x1 << 5)
#define AD77681_INTERFACE_CRC_TYPE(x)			(((x) & 0x1) << 5)
#define AD77681_INTERFACE_STATUS_EN_MSK			(0x1 << 4)
#define AD77681_INTERFACE_STATUS_EN(x)			(((x) & 0x1) << 4)
#define AD77681_INTERFACE_CONVLEN_MSK			(0x1 << 3)
#define AD77681_INTERFACE_CONVLEN(x)			(((x) & 0x1) << 3)
#define AD77681_INTERFACE_RDY_EN_MSK			(0x1 << 2)
#define AD77681_INTERFACE_RDY_EN(x)				(((x) & 0x1) << 3)
#define AD77681_INTERFACE_CONT_READ_MSK			(0x1 << 0)
#define AD77681_INTERFACE_CONT_READ_EN(x)		(((x) & 0x1) << 0)
#define AD77681_REG_COEFF_CONTROL				0x32
#define AD77681_REG_COEFF_DATA					0x33
#define AD77681_REG_ACCESS_KEY					0x34

/* AD77681_REG_SCRATCH_PAD*/
#define AD77681_SCRATCHPAD_MSK					(0xFF << 0)
#define AD77681_SCRATCHPAD(x)					(((x) & 0xFF) << 0)

/* AD77681_REG_POWER_CLOCK */
#define AD77681_POWER_CLK_PWRMODE_MSK			0x3
#define AD77681_POWER_CLK_PWRMODE(x)			(((x) & 0x3) << 0)
#define AD77681_POWER_CLK_MOD_OUT_MSK			(0x1 << 2)
#define AD77681_POWER_CLK_MOD_OUT(x)			(((x) & 0x1) << 2)
#define AD77681_POWER_CLK_POWER_DOWN			0x08
#define AD77681_POWER_CLK_MCLK_DIV_MSK			(0x3 << 4)
#define AD77681_POWER_CLK_MCLK_DIV(x)			(((x) & 0x3) << 4)
#define AD77681_POWER_CLK_CLOCK_SEL_MSK			(0x3 << 6)
#define AD77681_POWER_CLK_CLOCK_SEL(x)			(((x) & 0x3) << 6)

/* AD77681_CONVERSION_REG */
#define AD77681_CONVERSION_DIAG_MUX_MSK			(0xF << 4)
#define AD77681_CONVERSION_DIAG_MUX_SEL(x)		(((x) & 0xF) << 4)
#define AD77681_CONVERSION_DIAG_SEL_MSK			(0x1 << 3)
#define AD77681_CONVERSION_DIAG_SEL(x)			(((x) & 0x1) << 3)
#define AD77681_CONVERSION_MODE_MSK				(0x7 << 0)
#define AD77681_CONVERSION_MODE(x)				(((x) & 0x7) << 0)

/* AD77681_REG_ANALOG */
#define AD77681_ANALOG_REF_BUF_POS_MSK			(0x3 << 6)
#define AD77681_ANALOG_REF_BUF_POS(x)			(((x) & 0x3) << 6)
#define AD77681_ANALOG_REF_BUF_NEG_MSK			(0x3 << 4)
#define AD77681_ANALOG_REF_BUF_NEG(x)			(((x) & 0x3) << 4)
#define AD77681_ANALOG_AIN_BUF_POS_OFF_MSK		(0x1 << 1)
#define AD77681_ANALOG_AIN_BUF_POS_OFF(x)		(((x) & 0x1) << 1)
#define AD77681_ANALOG_AIN_BUF_NEG_OFF_MSK		(0x1 << 0)
#define AD77681_ANALOG_AIN_BUF_NEG_OFF(x)		(((x) & 0x1) << 0)

/* AD77681_REG_ANALOG2 */
#define AD77681_ANALOG2_VCM_MSK					(0x7 << 0)
#define AD77681_ANALOG2_VCM(x)					(((x) & 0x7) << 0)

/* AD77681_REG_DIGITAL_FILTER */
#define AD77681_DIGI_FILTER_60HZ_REJ_EN_MSK		(0x1 << 7)
#define AD77681_DIGI_FILTER_60HZ_REJ_EN(x)		(((x) & 0x1) << 7)
#define AD77681_DIGI_FILTER_FILTER_MSK			(0x7 << 4)
#define AD77681_DIGI_FILTER_FILTER(x)			(((x) & 0x7) << 4)
#define AD77681_DIGI_FILTER_DEC_RATE_MSK		(0x7 << 0)
#define AD77681_DIGI_FILTER_DEC_RATE(x)			(((x) & 0x7) << 0)

/* AD77681_REG_SINC3_DEC_RATE_MSB */
#define AD77681_SINC3_DEC_RATE_MSB_MSK			(0x0F << 0)
#define AD77681_SINC3_DEC_RATE_MSB(x)			(((x) & 0x0F) << 0)

/* AD77681_REG_SINC3_DEC_RATE_LSB */
#define AD77681_SINC3_DEC_RATE_LSB_MSK			(0xFF << 0)
#define AD77681_SINC3_DEC_RATE_LSB(x)			(((x) & 0xFF) << 0)

/* AD77681_REG_DUTY_CYCLE_RATIO */
#define AD77681_DC_RATIO_IDLE_TIME_MSK			(0xFF << 0)
#define AD77681_DC_RATIO_IDLE_TIME(x)			(((x) & 0xFF) << 0)

/* AD77681_REG_SYNC_RESET */
#define AD77681_SYNC_RST_SPI_STARTB_MSK			(0x1 << 7)
#define AD77681_SYNC_RST_SPI_STARTB(x)			(((x) & 0x1) << 7)
#define AD77681_SYNC_RST_SYNCOUT_EDGE_MSK		(0x1 << 6)
#define AD77681_SYNC_RST_SYNCOUT_EDGE(x)		(((x) & 0x1) << 6)
#define AD77681_SYNC_RST_GPIO_START_EN_MSK		(0x1 << 3)
#define AD77681_SYNC_RST_GPIO_START_EN(x)		(((x) & 0x1) << 3)
#define AD77681_SYNC_RST_SPI_RESET_MSK			(0x3 << 0)
#define AD77681_SYNC_RST_SPI_RESET(x)			(((x) & 0x3) << 0)

/* AD77681_REG_GPIO_CONTROL */
#define AD77681_GPIO_CNTRL_UGPIO_EN_MSK			(0x1 << 7)
#define AD77681_GPIO_CNTRL_UGPIO_EN(x)			(((x) & 0x1) << 7)
#define AD77681_GPIO_CNTRL_GPIO2_OD_EN_MSK		(0x1 << 6)
#define AD77681_GPIO_CNTRL_GPIO2_OD_EN(x)		(((x) & 0x1) << 6)
#define AD77681_GPIO_CNTRL_GPIO1_OD_EN_MSK		(0x1 << 5)
#define AD77681_GPIO_CNTRL_GPIO1_OD_EN(x)		(((x) & 0x1) << 5)
#define AD77681_GPIO_CNTRL_GPIO0_OD_EN_MSK		(0x1 << 4)
#define AD77681_GPIO_CNTRL_GPIO0_OD_EN(x)		(((x) & 0x1) << 4)
#define AD77681_GPIO_CNTRL_ALL_GPIOS_OD_EN_MSK	(0x7 << 4)
#define AD77681_GPIO_CNTRL_ALL_GPIOS_OD_EN(x)	(((x) & 0x7) << 4)
#define AD77681_GPIO_CNTRL_GPIO3_OP_EN_MSK		(0x1 << 3)
#define AD77681_GPIO_CNTRL_GPIO3_OP_EN(x)		(((x) & 0x1) << 3)
#define AD77681_GPIO_CNTRL_GPIO2_OP_EN_MSK		(0x1 << 2)
#define AD77681_GPIO_CNTRL_GPIO2_OP_EN(x)		(((x) & 0x1) << 2)
#define AD77681_GPIO_CNTRL_GPIO1_OP_EN_MSK		(0x1 << 1)
#define AD77681_GPIO_CNTRL_GPIO1_OP_EN(x)		(((x) & 0x1) << 1)
#define AD77681_GPIO_CNTRL_GPIO0_OP_EN_MSK		(0x1 << 0)
#define AD77681_GPIO_CNTRL_GPIO0_OP_EN(x)		(((x) & 0x1) << 0)
#define AD77681_GPIO_CNTRL_ALL_GPIOS_OP_EN_MSK	(0xF << 0)
#define AD77681_GPIO_CNTRL_ALL_GPIOS_OP_EN(x)	(((x) & 0xF) << 0)

/* AD77681_REG_GPIO_WRITE */
#define AD77681_GPIO_WRITE_3_MSK				(0x1 << 3)
#define AD77681_GPIO_WRITE_3(x)					(((x) & 0x1) << 3)
#define AD77681_GPIO_WRITE_2_MSK				(0x1 << 2)
#define AD77681_GPIO_WRITE_2(x)					(((x) & 0x1) << 2)
#define AD77681_GPIO_WRITE_1_MSK				(0x1 << 1)
#define AD77681_GPIO_WRITE_1(x)					(((x) & 0x1) << 1)
#define AD77681_GPIO_WRITE_0_MSK				(0x1 << 0)
#define AD77681_GPIO_WRITE_0(x)					(((x) & 0x1) << 0)
#define AD77681_GPIO_WRITE_ALL_MSK				(0xF << 0)
#define AD77681_GPIO_WRITE_ALL(x)				(((x) & 0xF))

/* AD77681_REG_GPIO_READ */
#define AD77681_GPIO_READ_3_MSK					(0x1 << 3)
#define AD77681_GPIO_READ_2_MSK					(0x1 << 2)
#define AD77681_GPIO_READ_1_MSK					(0x1 << 1)
#define AD77681_GPIO_READ_0_MSK					(0x1 << 0)
#define AD77681_GPIO_READ_ALL_MSK				(0xF << 0)

/* AD77681_REG_OFFSET_HI */
#define AD77681_OFFSET_HI_MSK					(0xFF << 0)
#define	AD77681_OFFSET_HI(x)					(((x) & 0xFF) << 0)

/* AD77681_REG_OFFSET_MID */
#define AD77681_OFFSET_MID_MSK					(0xFF << 0)
#define	AD77681_OFFSET_MID(x)					(((x) & 0xFF) << 0)

/* AD77681_REG_OFFSET_LO */
#define AD77681_OFFSET_LO_MSK					(0xFF << 0)
#define	AD77681_OFFSET_LO(x)					(((x) & 0xFF) << 0)

/* AD77681_REG_GAIN_HI */
#define AD77681_GAIN_HI_MSK						(0xFF << 0)
#define	AD77681_GAIN_HI(x)						(((x) & 0xFF) << 0)

/* AD77681_REG_GAIN_MID */
#define AD77681_GAIN_MID_MSK					(0xFF << 0)
#define	AD77681_GAIN_MID(x)						(((x) & 0xFF) << 0)

/* AD77681_REG_GAIN_HI */
#define AD77681_GAIN_LOW_MSK					(0xFF << 0)
#define	AD77681_GAIN_LOW(x)						(((x) & 0xFF) << 0)

/* AD77681_REG_SPI_DIAG_ENABLE */
#define AD77681_SPI_DIAG_ERR_SPI_IGNORE_MSK		(0x1 << 4)
#define AD77681_SPI_DIAG_ERR_SPI_IGNORE(x)		(((x) & 0x1) << 4)
#define AD77681_SPI_DIAG_ERR_SPI_CLK_CNT_MSK	(0x1 << 3)
#define AD77681_SPI_DIAG_ERR_SPI_CLK_CNT(x)		(((x) & 0x1) << 3)
#define AD77681_SPI_DIAG_ERR_SPI_RD_MSK			(0x1 << 2)
#define AD77681_SPI_DIAG_ERR_SPI_RD(x)			(((x) & 0x1) << 2)
#define AD77681_SPI_DIAG_ERR_SPI_WR_MSK			(0x1 << 1)
#define AD77681_SPI_DIAG_ERR_SPI_WR(x)			(((x) & 0x1) << 1)

/* AD77681_REG_ADC_DIAG_ENABLE */
#define AD77681_ADC_DIAG_ERR_DLDO_PSM_MSK		(0x1 << 5)
#define AD77681_ADC_DIAG_ERR_DLDO_PSM(x)		(((x) & 0x1) << 5)
#define AD77681_ADC_DIAG_ERR_ALDO_PSM_MSK		(0x1 << 4)
#define AD77681_ADC_DIAG_ERR_ALDO_PSM(x)		(((x) & 0x1) << 4)
#define AD77681_ADC_DIAG_ERR_FILT_SAT_MSK		(0x1 << 2)
#define AD77681_ADC_DIAG_ERR_FILT_SAT(x)			(((x) & 0x1) << 2)
#define AD77681_ADC_DIAG_ERR_FILT_NOT_SET_MSK	(0x1 << 1)
#define AD77681_ADC_DIAG_ERR_FILT_NOT_SET(x)		(((x) & 0x1) << 1)
#define AD77681_ADC_DIAG_ERR_EXT_CLK_QUAL_MSK	(0x1 << 0)
#define AD77681_ADC_DIAG_ERR_EXT_CLK_QUAL(x)	(((x) & 0x1) << 0)

/* AD77681_REG_DIG_DIAG_ENABLE */
#define AD77681_DIG_DIAG_ERR_MEMMAP_CRC_MSK		(0x1 << 4)
#define AD77681_DIG_DIAG_ERR_MEMMAP_CRC(x)		(((x) & 0x1) << 4)
#define AD77681_DIG_DIAG_ERR_RAM_CRC_MSK		(0x1 << 3)
#define AD77681_DIG_DIAG_ERR_RAM_CRC(x)			(((x) & 0x1) << 3)
#define AD77681_DIG_DIAG_ERR_FUSE_CRC_MSK		(0x1 << 2)
#define AD77681_DIG_DIAG_ERR_FUSE_CRC(x)		(((x) & 0x1) << 2)
#define AD77681_DIG_DIAG_FREQ_COUNT_EN_MSK		(0x1 << 0)
#define AD77681_DIG_DIAG_FREQ_COUNT_EN(x)		(((x) & 0x1) << 0)

/* AD77681_REG_MASTER_STATUS */
#define AD77681_MASTER_ERROR_MSK				(0x1 << 7)
#define AD77681_MASTER_ADC_ERROR_MSK			(0x1 << 6)
#define AD77681_MASTER_DIG_ERROR_MSK			(0x1 << 5)
#define AD77681_MASTER_DIG_ERR_EXT_CLK_MSK		(0x1 << 4)
#define AD77681_MASTER_FILT_SAT_MSK				(0x1 << 3)
#define AD77681_MASTER_FILT_NOT_SET_MSK			(0x1 << 2)
#define AD77681_MASTER_SPI_ERROR_MSK			(0x1 << 1)
#define AD77681_MASTER_POR_FLAG_MSK				(0x1 << 0)

/* AD77681_REG_SPI_DIAG_STATUS */
#define AD77681_SPI_IGNORE_ERROR_MSK			(0x1 << 4)
#define AD77681_SPI_IGNORE_ERROR_CLR(x)			(((x) & 0x1) << 4)
#define AD77681_SPI_CLK_CNT_ERROR_MSK			(0x1 << 3)
#define AD77681_SPI_READ_ERROR_MSK				(0x1 << 2)
#define AD77681_SPI_READ_ERROR_CLR(x)			(((x) & 0x1) << 2)
#define AD77681_SPI_WRITE_ERROR_MSK				(0x1 << 1)
#define AD77681_SPI_WRITE_ERROR_CLR(x)			(((x) & 0x1) << 1)
#define AD77681_SPI_CRC_ERROR_MSK				(0x1 << 0)
#define AD77681_SPI_CRC_ERROR_CLR(x)			(((x) & 0x1) << 0)

/* AD77681_REG_ADC_DIAG_STATUS */
#define AD77681_ADC_DLDO_PSM_ERROR_MSK			(0x1 << 5)
#define AD77681_ADC_ALDO_PSM_ERROR_MSK			(0x1 << 4)
#define AD77681_ADC_REF_DET_ERROR_MSK           (0x1 << 3)
#define AD77681_ADC_FILT_SAT_MSK				(0x1 << 2)
#define AD77681_ADC_FILT_NOT_SET_MSK			(0x1 << 1)
#define AD77681_ADC_DIG_ERR_EXT_CLK_MSK			(0x1 << 0)

/* AD77681_REG_DIG_DIAG_STATUS */
#define AD77681_DIG_MEMMAP_CRC_ERROR_MSK		(0x1 << 4)
#define AD77681_DIG_RAM_CRC_ERROR_MSK			(0x1 << 3)
#define AD77681_DIG_FUS_CRC_ERROR_MSK			(0x1 << 2)

/* AD77681_REG_MCLK_COUNTER */
#define AD77681_MCLK_COUNTER_MSK				(0xFF << 0)
#define	AD77681_MCLK_COUNTER(x)					(((x) & 0xFF) << 0)

/* AD77681_REG_COEFF_CONTROL */
#define AD77681_COEF_CONTROL_COEFFACCESSEN_MSK	(0x1 << 7)
#define AD77681_COEF_CONTROL_COEFFACCESSEN(x)	(((x) & 0x1) << 7)
#define AD77681_COEF_CONTROL_COEFFWRITEEN_MSK	(0x1 << 6)
#define AD77681_COEF_CONTROL_COEFFWRITEEN(x)	(((x) & 0x1) << 6)
#define AD77681_COEF_CONTROL_COEFFADDR_MSK		(0x3F << 5)
#define AD77681_COEF_CONTROL_COEFFADDR(x)		(((x) & 0x3F) << 5)

/* AD77681_REG_COEFF_DATA */
#define AD77681_COEFF_DATA_USERCOEFFEN_MSK		(0x1 << 23)
#define	AD77681_COEFF_DATA_USERCOEFFEN(x)		(((x) & 0x1) << 23)
#define AD77681_COEFF_DATA_COEFFDATA_MSK		(0x7FFFFF << 22)
#define	AD77681_COEFF_DATA_COEFFDATA(x)			(((x) & 0x7FFFFF) << 22)

/* AD77681_REG_ACCESS_KEY */
#define AD77681_ACCESS_KEY_MSK					(0xFF << 0)
#define	AD77681_ACCESS_KEY(x)					(((x) & 0xFF) << 0)
#define AD77681_ACCESS_KEY_CHECK_MSK			(0x1 << 0)

#define AD77681_REG_READ(x)						( (1 << 6) | (x & 0xFF) )		// Read from register x
#define AD77681_REG_WRITE(x)					( (~(1 << 6)) & (x & 0xFF) )  	// Write to register x

/* 8-bits wide checksum generated using the polynomial */
#define AD77681_CRC8_POLY	0x07 // x^8 + x^2 + x^1 + x^0

/* Initial CRC for continuous read mode */
#define INITIAL_CRC_CRC8						0x03
#define INITIAL_CRC_XOR							0x6C
#define INITIAL_CRC								0x00

#define CRC_DEBUG

/* AD7768-1 */
/* A special key for exit the contiuous read mode, taken from the AD7768-1 datasheet */
#define EXIT_CONT_READ							0x6C
/* Bit resolution of the AD7768-1 */
#define AD7768_N_BITS							24
/* Full scale of the AD7768-1 = 2^24 = 16777216 */
#define AD7768_FULL_SCALE						(1 << AD7768_N_BITS)
/* Half scale of the AD7768-1 = 2^23 = 8388608 */
#define AD7768_HALF_SCALE						(1 << (AD7768_N_BITS - 1))

#define ENABLE		1
#define DISABLE		0

/*****************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad77681_power_mode` enumeration defines the different power modes
 * available for the AD7768-1 device, which are used to control the power
 * consumption and performance characteristics of the device. The modes
 * include ECO, MEDIAN, and FAST, each represented by a specific integer
 * value, allowing for easy selection and configuration of the desired
 * power mode in the device's operation.
 *
 * @param AD77681_ECO Represents the ECO power mode with a value of 0.
 * @param AD77681_MEDIAN Represents the MEDIAN power mode with a value of 2.
 * @param AD77681_FAST Represents the FAST power mode with a value of 3.
 ******************************************************************************/
enum ad77681_power_mode {
	AD77681_ECO = 0,
	AD77681_MEDIAN = 2,
	AD77681_FAST = 3,
};

/***************************************************************************//**
 * @brief The `ad77681_mclk_div` enumeration defines the possible division
 * factors for the master clock (MCLK) in the AD7768-1 ADC driver. Each
 * enumerator corresponds to a specific division factor, allowing the
 * user to configure the clock speed by selecting one of the predefined
 * division values. This configuration is crucial for setting the
 * appropriate clock rate for the ADC's operation, impacting the sampling
 * rate and overall performance of the device.
 *
 * @param AD77681_MCLK_DIV_16 Represents a master clock division factor of 16.
 * @param AD77681_MCLK_DIV_8 Represents a master clock division factor of 8.
 * @param AD77681_MCLK_DIV_4 Represents a master clock division factor of 4.
 * @param AD77681_MCLK_DIV_2 Represents a master clock division factor of 2.
 ******************************************************************************/
enum ad77681_mclk_div {
	AD77681_MCLK_DIV_16 = 0,
	AD77681_MCLK_DIV_8 = 1,
	AD77681_MCLK_DIV_4 = 2,
	AD77681_MCLK_DIV_2 = 3
};

/***************************************************************************//**
 * @brief The `ad77681_conv_mode` enumeration defines the various conversion
 * modes available for the AD7768-1 analog-to-digital converter. Each
 * enumerator corresponds to a specific mode of operation, such as
 * continuous, one-shot, single, periodic, and standby, allowing the user
 * to configure the ADC's behavior according to the application's
 * requirements.
 *
 * @param AD77681_CONV_CONTINUOUS Represents continuous conversion mode with a
 * value of 0.
 * @param AD77681_CONV_ONE_SHOT Represents one-shot conversion mode with a value
 * of 1.
 * @param AD77681_CONV_SINGLE Represents single conversion mode with a value of
 * 2.
 * @param AD77681_CONV_PERIODIC Represents periodic conversion mode with a value
 * of 3.
 * @param AD77681_CONV_STANDBY Represents standby mode with a value of 4.
 ******************************************************************************/
enum ad77681_conv_mode {
	AD77681_CONV_CONTINUOUS = 0,
	AD77681_CONV_ONE_SHOT = 1,
	AD77681_CONV_SINGLE = 2,
	AD77681_CONV_PERIODIC = 3,
	AD77681_CONV_STANDBY = 4
};

/***************************************************************************//**
 * @brief The `ad77681_conv_len` enumeration defines the possible conversion
 * lengths for the AD7768-1 ADC, allowing selection between 24-bit and
 * 16-bit conversion modes. This is used to configure the ADC's output
 * data resolution, impacting the precision and size of the data produced
 * by the ADC.
 *
 * @param AD77681_CONV_24BIT Represents a 24-bit conversion length.
 * @param AD77681_CONV_16BIT Represents a 16-bit conversion length.
 ******************************************************************************/
enum ad77681_conv_len {
	AD77681_CONV_24BIT = 0,
	AD77681_CONV_16BIT = 1
};

/***************************************************************************//**
 * @brief The `ad77681_rdy_dout` enumeration defines two possible states for the
 * ready data output of the AD7768-1 device: enabled and disabled. This
 * enumeration is used to configure or check the status of the ready data
 * output, which is a part of the device's interface for indicating data
 * readiness.
 *
 * @param AD77681_RDY_DOUT_EN Represents the enabled state for the ready data
 * output.
 * @param AD77681_RDY_DOUT_DIS Represents the disabled state for the ready data
 * output.
 ******************************************************************************/
enum ad77681_rdy_dout {
	AD77681_RDY_DOUT_EN,
	AD77681_RDY_DOUT_DIS
};

/***************************************************************************//**
 * @brief The `ad77681_conv_diag_mux` enumeration defines various diagnostic
 * modes for the AD7768-1 ADC, allowing selection between temperature
 * sensor, analog input short, positive full-scale, and negative full-
 * scale diagnostic tests. Each mode is represented by a unique
 * hexadecimal value, facilitating easy configuration and testing of the
 * ADC's conversion diagnostics.
 *
 * @param AD77681_TEMP_SENSOR Represents the temperature sensor diagnostic mode
 * with a value of 0x0.
 * @param AD77681_AIN_SHORT Represents the analog input short diagnostic mode
 * with a value of 0x8.
 * @param AD77681_POSITIVE_FS Represents the positive full-scale diagnostic mode
 * with a value of 0x9.
 * @param AD77681_NEGATIVE_FS Represents the negative full-scale diagnostic mode
 * with a value of 0xA.
 ******************************************************************************/
enum ad77681_conv_diag_mux {
	AD77681_TEMP_SENSOR = 0x0,
	AD77681_AIN_SHORT = 0x8,
	AD77681_POSITIVE_FS = 0x9,
	AD77681_NEGATIVE_FS = 0xA
};

/***************************************************************************//**
 * @brief The `ad77681_crc_sel` enumeration defines the different error-checking
 * modes available for the AD7768-1 device. It provides options for using
 * CRC, XOR, or no error-checking, allowing users to select the
 * appropriate mode for their application to ensure data integrity during
 * communication.
 *
 * @param AD77681_CRC Represents the CRC error-checking mode.
 * @param AD77681_XOR Represents the XOR error-checking mode.
 * @param AD77681_NO_CRC Indicates that no error-checking is applied.
 ******************************************************************************/
enum ad77681_crc_sel {
	AD77681_CRC,
	AD77681_XOR,
	AD77681_NO_CRC
};

/* Filter tye FIR, SINC3, SINC5 */
/***************************************************************************//**
 * @brief The `ad77681_filter_type` enumeration defines the types of digital
 * filters available for the AD7768-1 ADC. It includes options for SINC5
 * filters with different decimation factors (x8 and x16), a SINC3
 * filter, and a FIR filter. These filter types are used to configure the
 * digital filtering characteristics of the ADC, affecting the signal
 * processing and data output.
 *
 * @param AD77681_SINC5 Represents the SINC5 filter type with a value of 0.
 * @param AD77681_SINC5_DECx8 Represents the SINC5 filter type with a decimation
 * factor of 8, with a value of 1.
 * @param AD77681_SINC5_DECx16 Represents the SINC5 filter type with a
 * decimation factor of 16, with a value of 2.
 * @param AD77681_SINC3 Represents the SINC3 filter type with a value of 3.
 * @param AD77681_FIR Represents the FIR filter type with a value of 4.
 ******************************************************************************/
enum ad77681_filter_type {
	AD77681_SINC5			= 0,
	AD77681_SINC5_DECx8		= 1,
	AD77681_SINC5_DECx16	= 2,
	AD77681_SINC3			= 3,
	AD77681_FIR				= 4
};

/* Dectimation ratios for SINC5 and FIR */
/***************************************************************************//**
 * @brief The `ad77681_sinc5_fir_decimate` enumeration defines various
 * decimation factors for the SINC5 FIR filter used in the AD7768-1 ADC
 * driver. Each enumerator corresponds to a specific decimation rate,
 * which determines how much the input signal is downsampled, affecting
 * the output data rate and resolution. This enumeration is crucial for
 * configuring the digital filter settings of the ADC to achieve the
 * desired balance between data rate and signal fidelity.
 *
 * @param AD77681_SINC5_FIR_DECx32 Represents a decimation factor of 32 for the
 * SINC5 FIR filter.
 * @param AD77681_SINC5_FIR_DECx64 Represents a decimation factor of 64 for the
 * SINC5 FIR filter.
 * @param AD77681_SINC5_FIR_DECx128 Represents a decimation factor of 128 for
 * the SINC5 FIR filter.
 * @param AD77681_SINC5_FIR_DECx256 Represents a decimation factor of 256 for
 * the SINC5 FIR filter.
 * @param AD77681_SINC5_FIR_DECx512 Represents a decimation factor of 512 for
 * the SINC5 FIR filter.
 * @param AD77681_SINC5_FIR_DECx1024 Represents a decimation factor of 1024 for
 * the SINC5 FIR filter.
 ******************************************************************************/
enum ad77681_sinc5_fir_decimate {
	AD77681_SINC5_FIR_DECx32	= 0,
	AD77681_SINC5_FIR_DECx64	= 1,
	AD77681_SINC5_FIR_DECx128	= 2,
	AD77681_SINC5_FIR_DECx256	= 3,
	AD77681_SINC5_FIR_DECx512	= 4,
	AD77681_SINC5_FIR_DECx1024	= 5
};

/* Sleep / Power up */
/***************************************************************************//**
 * @brief The `ad77681_sleep_wake` enumeration defines two states for the
 * AD7768-1 device: sleep and wake. The sleep state is represented by the
 * value 1, indicating that the device is in a low-power mode, while the
 * wake state is represented by the value 0, indicating that the device
 * is active and operational. This enumeration is used to control the
 * power state of the device, allowing it to conserve energy when not in
 * use and to be fully functional when needed.
 *
 * @param AD77681_SLEEP Represents the sleep state with a value of 1.
 * @param AD77681_WAKE Represents the wake state with a value of 0.
 ******************************************************************************/
enum ad77681_sleep_wake {
	AD77681_SLEEP   = 1,
	AD77681_WAKE = 0
};

/* Reset option */
/***************************************************************************//**
 * @brief The `ad7761_reset_option` is an enumeration that defines the reset
 * options available for the AD7768-1 device, specifically providing
 * options for either a software reset or a hardware reset. This
 * enumeration is used to control the reset behavior of the device,
 * allowing for different levels of reset based on the operational
 * requirements.
 *
 * @param AD77681_SOFT_RESET Represents a software reset option for the AD7768-1
 * device.
 * @param AD77681_HARD_RESET Represents a hardware reset option for the AD7768-1
 * device.
 ******************************************************************************/
enum ad7761_reset_option {
	AD77681_SOFT_RESET,
	AD77681_HARD_RESET
};
/* AIN- precharge */
/***************************************************************************//**
 * @brief The `ad77681_AINn_precharge` enumeration defines the possible states
 * for the AINn precharge configuration in the AD7768-1 device. It allows
 * the user to enable or disable the precharge feature for the AINn
 * input, which can be useful for optimizing the input signal
 * conditioning in certain applications.
 *
 * @param AD77681_AINn_ENABLED Represents the enabled state of the AINn
 * precharge.
 * @param AD77681_AINn_DISABLED Represents the disabled state of the AINn
 * precharge.
 ******************************************************************************/
enum ad77681_AINn_precharge {
	AD77681_AINn_ENABLED   = 0,
	AD77681_AINn_DISABLED  = 1
};

/* AIN+ precharge */
/***************************************************************************//**
 * @brief The `ad77681_AINp_precharge` enumeration defines the possible states
 * for the AIN+ precharge configuration in the AD7768-1 device. It allows
 * the user to enable or disable the precharge feature for the positive
 * analog input, which can be useful for optimizing the input signal
 * conditioning in certain applications.
 *
 * @param AD77681_AINp_ENABLED Represents the enabled state for the AIN+
 * precharge.
 * @param AD77681_AINp_DISABLED Represents the disabled state for the AIN+
 * precharge.
 ******************************************************************************/
enum ad77681_AINp_precharge {
	AD77681_AINp_ENABLED  = 0,
	AD77681_AINp_DISABLED = 1
};

/* REF- buffer */
/***************************************************************************//**
 * @brief The `ad77681_REFn_buffer` is an enumeration that defines the possible
 * states of the REF- buffer in the AD7768-1 device. It provides three
 * states: enabled, disabled, and fully on, which are used to configure
 * the buffer's operational mode. This enumeration is part of the
 * configuration settings for the AD7768-1, a high-performance analog-to-
 * digital converter, allowing users to control the buffer settings for
 * optimal performance in various applications.
 *
 * @param AD77681_BUFn_ENABLED Represents the state where the REF- buffer is
 * enabled.
 * @param AD77681_BUFn_DISABLED Represents the state where the REF- buffer is
 * disabled.
 * @param AD77681_BUFn_FULL_BUFFER_ON Represents the state where the REF- buffer
 * is fully on.
 ******************************************************************************/
enum ad77681_REFn_buffer {
	AD77681_BUFn_ENABLED        = 0,
	AD77681_BUFn_DISABLED       = 1,
	AD77681_BUFn_FULL_BUFFER_ON = 2
};

/* REF+ buffer */
/***************************************************************************//**
 * @brief The `ad77681_REFp_buffer` is an enumeration that defines the possible
 * states of the REF+ buffer in the AD7768-1 device. It includes states
 * for enabling, disabling, and fully turning on the buffer, which are
 * used to configure the reference buffer settings for the device's
 * operation.
 *
 * @param AD77681_BUFp_ENABLED Represents the enabled state of the REF+ buffer.
 * @param AD77681_BUFp_DISABLED Represents the disabled state of the REF+
 * buffer.
 * @param AD77681_BUFp_FULL_BUFFER_ON Represents the state where the full buffer
 * is on for the REF+ buffer.
 ******************************************************************************/
enum ad77681_REFp_buffer {
	AD77681_BUFp_ENABLED        = 0,
	AD77681_BUFp_DISABLED       = 1,
	AD77681_BUFp_FULL_BUFFER_ON = 2
};

/* VCM output voltage */
/***************************************************************************//**
 * @brief The `ad77681_VCM_out` enumeration defines various output voltage
 * levels for the VCM (Voltage Common Mode) of the AD7768-1 device. Each
 * enumerator corresponds to a specific voltage level that can be set for
 * the VCM output, ranging from half the supply voltage to specific fixed
 * voltages, or turning the VCM output off entirely. This enumeration is
 * used to configure the VCM output voltage in the AD7768-1 device, which
 * is crucial for setting the common mode voltage in differential signal
 * applications.
 *
 * @param AD77681_VCM_HALF_VCC Represents a VCM output voltage of half the
 * supply voltage (VCC).
 * @param AD77681_VCM_2_5V Represents a VCM output voltage of 2.5 volts.
 * @param AD77681_VCM_2_05V Represents a VCM output voltage of 2.05 volts.
 * @param AD77681_VCM_1_9V Represents a VCM output voltage of 1.9 volts.
 * @param AD77681_VCM_1_65V Represents a VCM output voltage of 1.65 volts.
 * @param AD77681_VCM_1_1V Represents a VCM output voltage of 1.1 volts.
 * @param AD77681_VCM_0_9V Represents a VCM output voltage of 0.9 volts.
 * @param AD77681_VCM_OFF Represents the VCM output being turned off.
 ******************************************************************************/
enum ad77681_VCM_out {
	AD77681_VCM_HALF_VCC	= 0,
	AD77681_VCM_2_5V		= 1,
	AD77681_VCM_2_05V   	= 2,
	AD77681_VCM_1_9V		= 3,
	AD77681_VCM_1_65V		= 4,
	AD77681_VCM_1_1V		= 5,
	AD77681_VCM_0_9V		= 6,
	AD77681_VCM_OFF  		= 7
};

/* Global GPIO enable/disable */
/***************************************************************************//**
 * @brief The `ad77681_gobal_gpio_enable` is an enumeration that defines the
 * possible states for enabling or disabling the global GPIO
 * functionality in the AD7768-1 device. It provides two states:
 * `AD77681_GLOBAL_GPIO_ENABLE` for enabling the GPIO and
 * `AD77681_GLOBAL_GPIO_DISABLE` for disabling it, allowing for
 * straightforward control over the GPIO's operational state.
 *
 * @param AD77681_GLOBAL_GPIO_ENABLE Represents the enabled state of the global
 * GPIO, with a value of 1.
 * @param AD77681_GLOBAL_GPIO_DISABLE Represents the disabled state of the
 * global GPIO, with a value of 0.
 ******************************************************************************/
enum ad77681_gobal_gpio_enable {
	AD77681_GLOBAL_GPIO_ENABLE		= 1,
	AD77681_GLOBAL_GPIO_DISABLE		= 0
};

/* ADCs GPIO numbering */
/***************************************************************************//**
 * @brief The `ad77681_gpios` enumeration defines constants for the GPIO pins
 * available on the AD7768-1 device, allowing for easy reference and
 * manipulation of these pins in the code. Each enumerator corresponds to
 * a specific GPIO pin or a combination of all GPIOs, facilitating the
 * configuration and control of the device's GPIO functionality.
 *
 * @param AD77681_GPIO0 Represents GPIO pin 0 with a value of 0.
 * @param AD77681_GPIO1 Represents GPIO pin 1 with a value of 1.
 * @param AD77681_GPIO2 Represents GPIO pin 2 with a value of 2.
 * @param AD77681_GPIO3 Represents GPIO pin 3 with a value of 3.
 * @param AD77681_ALL_GPIOS Represents all GPIO pins with a value of 4.
 ******************************************************************************/
enum ad77681_gpios {
	AD77681_GPIO0					= 0,
	AD77681_GPIO1					= 1,
	AD77681_GPIO2					= 2,
	AD77681_GPIO3					= 3,
	AD77681_ALL_GPIOS				= 4
};

/***************************************************************************//**
 * @brief The `ad77681_gpio_output_type` is an enumeration that defines the
 * types of output configurations available for the GPIO pins in the
 * AD7768-1 device. It provides two options: `AD77681_GPIO_STRONG_DRIVER`
 * for a strong driver configuration, and `AD77681_GPIO_OPEN_DRAIN` for
 * an open-drain configuration. This allows for flexible control over the
 * electrical characteristics of the GPIO outputs, which can be crucial
 * for interfacing with different types of external circuits.
 *
 * @param AD77681_GPIO_STRONG_DRIVER Represents a strong driver output type for
 * GPIO.
 * @param AD77681_GPIO_OPEN_DRAIN Represents an open-drain output type for GPIO.
 ******************************************************************************/
enum ad77681_gpio_output_type {
	AD77681_GPIO_STRONG_DRIVER		= 0,
	AD77681_GPIO_OPEN_DRAIN			= 1
};

/* Continuous ADC read */
/***************************************************************************//**
 * @brief The `ad77681_continuous_read` enumeration defines two states for
 * controlling the continuous read mode of the AD7768-1 ADC device. It
 * provides a simple mechanism to enable or disable continuous reading of
 * ADC data, which is crucial for applications requiring constant data
 * acquisition without manual intervention.
 *
 * @param AD77681_CONTINUOUS_READ_ENABLE Represents the enabled state for
 * continuous read mode, with a value of
 * 1.
 * @param AD77681_CONTINUOUS_READ_DISABLE Represents the disabled state for
 * continuous read mode, with a value of
 * 0.
 ******************************************************************************/
enum ad77681_continuous_read {
	AD77681_CONTINUOUS_READ_ENABLE = 1,
	AD77681_CONTINUOUS_READ_DISABLE = 0,
};

/* ADC data read mode */
/***************************************************************************//**
 * @brief The `ad77681_data_read_mode` is an enumeration that defines the modes
 * of data reading for the AD7768-1 device. It provides two options:
 * `AD77681_REGISTER_DATA_READ` for reading data from registers and
 * `AD77681_CONTINUOUS_DATA_READ` for continuous data reading. This
 * enumeration is used to configure how data is accessed from the device,
 * allowing for either discrete register reads or a continuous stream of
 * data, depending on the application requirements.
 *
 * @param AD77681_REGISTER_DATA_READ Represents the register data read mode with
 * a value of 0.
 * @param AD77681_CONTINUOUS_DATA_READ Represents the continuous data read mode
 * with a value of 1.
 ******************************************************************************/
enum ad77681_data_read_mode {
	AD77681_REGISTER_DATA_READ = 0,
	AD77681_CONTINUOUS_DATA_READ = 1,
};

/* ADC data structure */
/***************************************************************************//**
 * @brief The `adc_data` structure is designed to hold information related to
 * the data acquisition process of an Analog-to-Digital Converter (ADC).
 * It includes a flag to indicate the completion of data acquisition,
 * counters for the number of data points and samples, and a large buffer
 * to store the raw data collected from the ADC. This structure is
 * essential for managing and processing the data output from the ADC in
 * applications requiring high-resolution data collection.
 *
 * @param finish A boolean flag indicating whether the ADC data acquisition is
 * complete.
 * @param count A 16-bit unsigned integer representing the number of data points
 * collected.
 * @param samples A 16-bit unsigned integer indicating the number of samples
 * taken.
 * @param raw_data An array of 4096 32-bit unsigned integers storing the raw ADC
 * data.
 ******************************************************************************/
struct adc_data {
	bool		finish;
	uint16_t	count;
	uint16_t	samples;
	uint32_t	raw_data[4096];
};
/* ADC status registers structure */
/***************************************************************************//**
 * @brief The `ad77681_status_registers` structure is designed to hold various
 * status flags for the AD7768-1 device, which is an analog-to-digital
 * converter. Each member of the structure is a boolean flag that
 * indicates the presence of specific errors or conditions within the
 * device, such as errors related to the ADC, digital logic, SPI
 * communication, and power management. This structure is crucial for
 * diagnostics and error handling, allowing the system to monitor and
 * respond to different operational states and faults.
 *
 * @param master_error Indicates if there is a master error.
 * @param adc_error Indicates if there is an ADC error.
 * @param dig_error Indicates if there is a digital error.
 * @param adc_err_ext_clk_qual Indicates if there is an ADC error related to
 * external clock quality.
 * @param adc_filt_saturated Indicates if the ADC filter is saturated.
 * @param adc_filt_not_settled Indicates if the ADC filter has not settled.
 * @param spi_error Indicates if there is an SPI error.
 * @param por_flag Indicates if a power-on reset has occurred.
 * @param spi_ignore Indicates if SPI ignore error has occurred.
 * @param spi_clock_count Indicates if there is an SPI clock count error.
 * @param spi_read_error Indicates if there is an SPI read error.
 * @param spi_write_error Indicates if there is an SPI write error.
 * @param spi_crc_error Indicates if there is an SPI CRC error.
 * @param dldo_psm_error Indicates if there is a DLDO PSM error.
 * @param aldo_psm_error Indicates if there is an ALDO PSM error.
 * @param ref_det_error Indicates if there is a reference detection error.
 * @param filt_sat_error Indicates if there is a filter saturation error.
 * @param filt_not_set_error Indicates if there is a filter not set error.
 * @param ext_clk_qual_error Indicates if there is an external clock quality
 * error.
 * @param memoy_map_crc_error Indicates if there is a memory map CRC error.
 * @param ram_crc_error Indicates if there is a RAM CRC error.
 * @param fuse_crc_error Indicates if there is a fuse CRC error.
 ******************************************************************************/
struct ad77681_status_registers {
	bool							master_error;
	bool							adc_error;
	bool							dig_error;
	bool							adc_err_ext_clk_qual;
	bool							adc_filt_saturated;
	bool							adc_filt_not_settled;
	bool							spi_error;
	bool							por_flag;
	bool							spi_ignore;
	bool							spi_clock_count;
	bool							spi_read_error;
	bool							spi_write_error;
	bool							spi_crc_error;
	bool							dldo_psm_error;
	bool							aldo_psm_error;
	bool                            ref_det_error;
	bool                            filt_sat_error;
	bool                            filt_not_set_error;
	bool                            ext_clk_qual_error;
	bool							memoy_map_crc_error;
	bool							ram_crc_error;
	bool							fuse_crc_error;
};

/***************************************************************************//**
 * @brief The `ad77681_dev` structure is a comprehensive configuration and state
 * representation for the AD7768-1 ADC device. It includes members for
 * SPI communication, power and clock settings, conversion modes,
 * diagnostic settings, and various buffer and filter configurations.
 * This structure is essential for managing the ADC's operation, allowing
 * for detailed control over its sampling, data integrity, and signal
 * processing capabilities. Each member of the structure corresponds to a
 * specific aspect of the ADC's functionality, enabling precise
 * adjustments and monitoring of the device's performance.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param power_mode Specifies the power mode of the device.
 * @param mclk_div Specifies the master clock division factor.
 * @param conv_mode Specifies the conversion mode of the device.
 * @param diag_mux_sel Selects the diagnostic multiplexer.
 * @param conv_diag_sel Enables or disables conversion diagnostics.
 * @param conv_len Specifies the conversion length (bit resolution).
 * @param crc_sel Specifies the CRC selection for data integrity.
 * @param status_bit Holds the status bit value.
 * @param VCM_out Specifies the VCM output voltage setting.
 * @param AINn Specifies the AIN- precharge setting.
 * @param AINp Specifies the AIN+ precharge setting.
 * @param REFn Specifies the REF- buffer setting.
 * @param REFp Specifies the REF+ buffer setting.
 * @param filter Specifies the filter type used in the device.
 * @param decimate Specifies the decimation ratio for SINC5 and FIR filters.
 * @param sinc3_osr Specifies the oversampling ratio for the SINC3 filter.
 * @param vref Specifies the reference voltage value.
 * @param mclk Specifies the master clock frequency.
 * @param sample_rate Specifies the sample rate of the device.
 * @param data_frame_byte Specifies the number of bytes in the SPI data frame.
 ******************************************************************************/
struct ad77681_dev {
	/* SPI */
	struct no_os_spi_desc			*spi_desc;
	/* Configuration */
	enum ad77681_power_mode		power_mode;
	enum ad77681_mclk_div		mclk_div;
	enum ad77681_conv_mode 		conv_mode;
	enum ad77681_conv_diag_mux 	diag_mux_sel;
	bool 					conv_diag_sel;
	enum ad77681_conv_len		conv_len;
	enum ad77681_crc_sel 		crc_sel;
	uint8_t					status_bit;
	enum ad77681_VCM_out            VCM_out;
	enum ad77681_AINn_precharge     AINn;
	enum ad77681_AINp_precharge     AINp;
	enum ad77681_REFn_buffer        REFn;
	enum ad77681_REFp_buffer        REFp;
	enum ad77681_filter_type        filter;
	enum ad77681_sinc5_fir_decimate decimate;
	uint16_t                        sinc3_osr;
	uint16_t                        vref;               /* Reference voltage*/
	uint16_t                        mclk;               /* Mater clock*/
	uint32_t                        sample_rate;        /* Sample rate*/
	uint8_t                         data_frame_byte;    /* SPI 8bit frames*/
};

/***************************************************************************//**
 * @brief The `ad77681_init_param` structure is used to initialize and configure
 * the AD77681 ADC device. It includes parameters for SPI initialization,
 * power mode, clock division, conversion settings, diagnostic options,
 * and various buffer and filter configurations. This structure allows
 * for detailed customization of the ADC's operation, including setting
 * the sample rate, reference voltage, and data integrity checks through
 * CRC. It is essential for setting up the ADC to meet specific
 * application requirements.
 *
 * @param spi_eng_dev_init Initializes the SPI engine device.
 * @param power_mode Specifies the power mode of the AD77681.
 * @param mclk_div Sets the master clock division factor.
 * @param conv_mode Defines the conversion mode of the AD77681.
 * @param diag_mux_sel Selects the diagnostic multiplexer.
 * @param conv_diag_sel Enables or disables conversion diagnostics.
 * @param conv_len Specifies the conversion length (bit resolution).
 * @param crc_sel Selects the CRC mode for data integrity.
 * @param status_bit Indicates the status bit configuration.
 * @param VCM_out Sets the VCM output voltage level.
 * @param AINn Configures the AIN- precharge setting.
 * @param AINp Configures the AIN+ precharge setting.
 * @param REFn Configures the REF- buffer setting.
 * @param REFp Configures the REF+ buffer setting.
 * @param filter Specifies the filter type used in the ADC.
 * @param decimate Sets the decimation ratio for SINC5 and FIR filters.
 * @param sinc3_osr Defines the oversampling ratio for the SINC3 filter.
 * @param vref Specifies the reference voltage value.
 * @param mclk Specifies the master clock frequency.
 * @param sample_rate Defines the sample rate of the ADC.
 * @param data_frame_byte Specifies the number of bytes in the SPI data frame.
 ******************************************************************************/
struct ad77681_init_param {
	/* SPI */
	struct no_os_spi_init_param			spi_eng_dev_init;
	/* Configuration */
	enum ad77681_power_mode		power_mode;
	enum ad77681_mclk_div		mclk_div;
	enum ad77681_conv_mode 		conv_mode;
	enum ad77681_conv_diag_mux 	diag_mux_sel;
	bool 					conv_diag_sel;
	enum ad77681_conv_len		conv_len;
	enum ad77681_crc_sel 		crc_sel;
	uint8_t					status_bit;
	enum ad77681_VCM_out            VCM_out;
	enum ad77681_AINn_precharge     AINn;
	enum ad77681_AINp_precharge     AINp;
	enum ad77681_REFn_buffer        REFn;
	enum ad77681_REFp_buffer        REFp;
	enum ad77681_filter_type        filter;
	enum ad77681_sinc5_fir_decimate decimate;
	uint16_t                        sinc3_osr;
	uint16_t                        vref;
	uint16_t                        mclk;
	uint32_t                        sample_rate;
	uint8_t                         data_frame_byte;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/***************************************************************************//**
 * @brief This function computes an 8-bit cyclic redundancy check (CRC) for a
 * specified data buffer, which is useful for error-checking in data
 * transmission or storage. It should be used when you need to verify the
 * integrity of data by generating a CRC value that can be compared
 * against a known value. The function requires an initial CRC value,
 * which allows for CRC calculations to be continued from a previous
 * state. Ensure that the data pointer is valid and that the data size
 * accurately reflects the number of bytes to process.
 *
 * @param data A pointer to the data buffer for which the CRC is to be computed.
 * Must not be null, and the caller retains ownership.
 * @param data_size The number of bytes in the data buffer to process. Must be a
 * non-negative integer.
 * @param init_val The initial CRC value to start the computation. This allows
 * for continuation of a previous CRC calculation.
 * @return Returns the computed 8-bit CRC value as an unsigned integer.
 ******************************************************************************/
uint8_t ad77681_compute_crc8(uint8_t *data,
			     uint8_t data_size,
			     uint8_t init_val);
/***************************************************************************//**
 * @brief This function calculates the XOR checksum of a given array of bytes,
 * starting with an initial value. It is useful for simple error
 * detection in data transmission or storage. The function requires a
 * pointer to the data array, the size of the data, and an initial XOR
 * value. It processes each byte in the array, updating the checksum
 * iteratively. Ensure that the data pointer is valid and that the data
 * size does not exceed the buffer limits.
 *
 * @param data Pointer to the array of bytes to be processed. Must not be null,
 * and the caller retains ownership.
 * @param data_size The number of bytes in the data array to process. Must be
 * less than or equal to 3 to avoid buffer overflow.
 * @param init_val The initial value for the XOR computation. Can be any 8-bit
 * unsigned integer.
 * @return Returns the computed XOR checksum as an 8-bit unsigned integer.
 ******************************************************************************/
uint8_t ad77681_compute_xor(uint8_t *data,
			    uint8_t data_size,
			    uint8_t init_val);
/***************************************************************************//**
 * @brief This function sets up the AD7768-1 device by allocating necessary
 * resources, configuring it according to the provided initialization
 * parameters, and performing a series of checks and configurations to
 * ensure proper operation. It must be called before any other operations
 * on the device. The function also verifies the physical connection and
 * initializes the SPI interface. If any step fails, the function returns
 * an error code and cleans up allocated resources.
 *
 * @param device A pointer to a pointer of type `struct ad77681_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad77681_init_param` containing
 * the initialization parameters for the device. All fields
 * must be properly set before calling the function.
 * @param status A pointer to a pointer of type `struct
 * ad77681_status_registers`. This will be allocated and populated
 * with the device's status information. Must not be null.
 * @return Returns an `int32_t` value. A return value of 0 indicates success,
 * while a negative value indicates an error occurred during setup.
 ******************************************************************************/
int32_t ad77681_setup(struct ad77681_dev **device,
		      struct ad77681_init_param init_param,
		      struct ad77681_status_registers **status);
/***************************************************************************//**
 * @brief This function reads a specified register from the AD7768-1 device
 * using SPI communication. It should be called when you need to retrieve
 * the value of a register from the device. The function requires a valid
 * device structure and a register address to read from. It supports
 * error checking through CRC or XOR, depending on the device
 * configuration. Ensure the device is properly initialized and
 * configured before calling this function. The function will return an
 * error code if the SPI communication fails or if the CRC/XOR check
 * fails.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read. Must be a valid register
 * address for the AD7768-1.
 * @param reg_data A pointer to a buffer where the read register data will be
 * stored. Must not be null and should be large enough to hold
 * the register data.
 * @return Returns an int32_t error code: 0 on success, or a negative value if
 * an error occurs (e.g., SPI communication failure or CRC/XOR
 * mismatch).
 ******************************************************************************/
int32_t ad77681_spi_reg_read(struct ad77681_dev *dev,
			     uint8_t reg_addr,
			     uint8_t *reg_data);
/***************************************************************************//**
 * @brief This function reads a value from a specified register of the AD7768-1
 * device using SPI communication and applies a mask to extract specific
 * bits. It is useful for retrieving specific configuration or status
 * bits from a register. The function requires a valid device structure
 * and a non-null data pointer to store the result. It should be called
 * when the device is properly initialized and configured for SPI
 * communication.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD7768-1 device.
 * @param mask A bitmask to apply to the register value to extract specific
 * bits. Any 8-bit value is valid.
 * @param data A pointer to a uint8_t where the masked register value will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating the success or failure of the SPI read
 * operation. The masked register value is stored in the location
 * pointed to by the data parameter.
 ******************************************************************************/
int32_t ad77681_spi_read_mask(struct ad77681_dev *dev,
			      uint8_t reg_addr,
			      uint8_t mask,
			      uint8_t *data);
/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * on the AD7768-1 device using the SPI interface. It should be called
 * when there is a need to configure or modify the settings of the
 * AD7768-1 by writing to its registers. The function requires a valid
 * device structure that has been properly initialized and configured for
 * SPI communication. It handles the optional inclusion of a CRC checksum
 * based on the device's configuration. The function returns an error
 * code if the SPI write operation fails.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null and should be initialized with valid SPI
 * configuration.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address as defined by the device's register map.
 * @param reg_data The data byte to write to the specified register. Any 8-bit
 * value is valid.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in the SPI write operation.
 ******************************************************************************/
int32_t ad77681_spi_reg_write(struct ad77681_dev *dev,
			      uint8_t reg_addr,
			      uint8_t reg_data);
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * AD7768-1 device by applying a mask and writing new data. It is useful
 * when only certain bits of a register need to be updated without
 * affecting the other bits. The function first reads the current value
 * of the register, applies the mask to clear the bits to be modified,
 * and then writes the new data to those bits. It must be called with a
 * valid device structure and register address. The function returns an
 * error code if the read or write operation fails.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the device.
 * @param mask A bitmask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits specified by the mask will be updated.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in reading or writing the register.
 ******************************************************************************/
int32_t ad77681_spi_write_mask(struct ad77681_dev *dev,
			       uint8_t reg_addr,
			       uint8_t mask,
			       uint8_t data);
/***************************************************************************//**
 * @brief This function configures the power mode of the AD7768-1 device to one
 * of the predefined modes. It should be called when a change in power
 * consumption or performance is required. The function updates the
 * device's power mode setting and returns a status code indicating
 * success or failure. It is important to ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param mode An enum value of type ad77681_power_mode specifying the desired
 * power mode. Valid values are AD77681_ECO, AD77681_MEDIAN, and
 * AD77681_FAST.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t ad77681_set_power_mode(struct ad77681_dev *dev,
			       enum ad77681_power_mode mode);
/***************************************************************************//**
 * @brief This function configures the master clock division factor for the
 * AD7768-1 device, which affects the device's clocking and potentially
 * its performance characteristics. It should be called when the device
 * is initialized and before starting any data acquisition processes that
 * depend on the clock settings. The function updates the device's
 * internal state to reflect the new clock division setting if the
 * operation is successful.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param clk_div An enum value of type ad77681_mclk_div specifying the desired
 * master clock division factor. Valid values are
 * AD77681_MCLK_DIV_16, AD77681_MCLK_DIV_8, AD77681_MCLK_DIV_4,
 * and AD77681_MCLK_DIV_2.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t ad77681_set_mclk_div(struct ad77681_dev *dev,
			     enum ad77681_mclk_div clk_div);
/***************************************************************************//**
 * @brief This function retrieves ADC data from the AD7768-1 device using the
 * specified SPI interface. It supports two modes of data reading:
 * register data read and continuous data read. The function must be
 * called with a properly initialized `ad77681_dev` structure. The
 * `adc_data` buffer is filled with the read data, and the function
 * returns an error code if the SPI communication fails or if a CRC error
 * is detected when CRC is enabled. Ensure that the device is configured
 * correctly before calling this function.
 *
 * @param dev A pointer to an `ad77681_dev` structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param adc_data A pointer to a buffer where the ADC data will be stored. The
 * buffer must be large enough to hold the data, and the caller
 * retains ownership.
 * @param mode An `ad77681_data_read_mode` enum value indicating the data read
 * mode. Valid values are `AD77681_REGISTER_DATA_READ` and
 * `AD77681_CONTINUOUS_DATA_READ`.
 * @return Returns an `int32_t` error code: 0 on success, or a negative value if
 * an error occurs (e.g., SPI communication failure or CRC error).
 ******************************************************************************/
int32_t ad77681_spi_read_adc_data(struct ad77681_dev *dev,
				  uint8_t *adc_data,
				  enum ad77681_data_read_mode mode);
/***************************************************************************//**
 * @brief This function sets the conversion mode, diagnostic multiplexer
 * selection, and diagnostic selection for the AD7768-1 device. It should
 * be called when you need to change the conversion settings of the
 * device. The function requires a valid device structure and appropriate
 * enumeration values for the conversion mode and diagnostic settings. It
 * updates the device's internal state with the new settings if the
 * operation is successful. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param conv_mode An enumeration value of type ad77681_conv_mode specifying
 * the desired conversion mode. Valid values are defined in the
 * ad77681_conv_mode enum.
 * @param diag_mux_sel An enumeration value of type ad77681_conv_diag_mux
 * specifying the diagnostic multiplexer selection. Valid
 * values are defined in the ad77681_conv_diag_mux enum.
 * @param conv_diag_sel A boolean value indicating whether the conversion
 * diagnostic is selected. True enables the diagnostic,
 * false disables it.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad77681_set_conv_mode(struct ad77681_dev *dev,
			      enum ad77681_conv_mode conv_mode,
			      enum ad77681_conv_diag_mux diag_mux_sel,
			      bool conv_diag_sel);
/***************************************************************************//**
 * @brief This function configures the conversion length of the AD7768-1 device,
 * which determines the bit resolution of the conversion results. It
 * should be called when the user needs to change the conversion length
 * setting, typically during device initialization or when a different
 * resolution is required. The function updates the device's internal
 * state and writes the new setting to the appropriate register. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function returns an error code if the
 * operation fails, allowing the caller to handle any issues that arise.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * The caller retains ownership.
 * @param conv_len An enum value of type ad77681_conv_len specifying the desired
 * conversion length. Valid values are AD77681_CONV_24BIT and
 * AD77681_CONV_16BIT. Invalid values may result in undefined
 * behavior.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating an error.
 ******************************************************************************/
int32_t ad77681_set_convlen(struct ad77681_dev *dev,
			    enum ad77681_conv_len conv_len);
/***************************************************************************//**
 * @brief This function is used to perform a soft reset on the AD7768-1 device,
 * which is necessary to reinitialize the device's state without a full
 * power cycle. It should be called when the device needs to be reset to
 * its default state, such as after configuration changes or error
 * conditions. The function requires a valid device structure that has
 * been properly initialized. It returns an error code if the reset
 * operation fails, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device to be reset. Must not be null. The function will return an
 * error if the device is not properly initialized or if the pointer
 * is invalid.
 * @return Returns an int32_t error code: 0 for success, or a negative value
 * indicating the type of error encountered during the reset operation.
 ******************************************************************************/
int32_t ad77681_soft_reset(struct ad77681_dev *dev);
/***************************************************************************//**
 * @brief This function is used to initiate a synchronization sequence for the
 * AD7768-1 device, which is necessary for aligning the device's
 * operation with other components in a system. It should be called when
 * synchronization is required, such as after device initialization or
 * configuration changes. The function requires a valid device structure
 * and will return an error code if the synchronization cannot be
 * initiated.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null. The caller retains ownership of the structure.
 * @return Returns an int32_t error code indicating success or failure of the
 * synchronization initiation.
 ******************************************************************************/
int32_t ad77681_initiate_sync(struct ad77681_dev *dev);
/***************************************************************************//**
 * @brief This function is used to configure the programmable filter of the
 * AD7768-1 device with a set of user-defined coefficients. It must be
 * called after the device has been properly initialized and is ready to
 * accept configuration commands. The function writes the provided
 * coefficients to the device, padding with zeros if fewer than 56
 * coefficients are provided. It is important to ensure that the device
 * is not in a power-down state and that the SPI interface is correctly
 * set up before calling this function. The function returns an error
 * code if any step in the process fails, allowing the caller to handle
 * such errors appropriately.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param coeffs A pointer to an array of float values representing the filter
 * coefficients. The caller retains ownership and must ensure the
 * array is valid and accessible.
 * @param num_coeffs The number of coefficients in the coeffs array. Must be
 * less than or equal to 56.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int32_t ad77681_programmable_filter(struct ad77681_dev *dev,
				    const float *coeffs,
				    uint8_t num_coeffs);
/***************************************************************************//**
 * @brief This function retrieves the current state of a specified GPIO pin on
 * the AD7768-1 device. It is useful for monitoring the status of GPIO
 * pins configured as inputs. The function must be called with a valid
 * device structure and a pointer to store the read value. The GPIO pin
 * to be read is specified by the gpio_number parameter, which can be one
 * of the individual GPIO pins or all GPIOs. If an invalid GPIO number is
 * provided, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param value A pointer to a uint8_t where the read GPIO value will be stored.
 * Must not be null.
 * @param gpio_number An enum ad77681_gpios value specifying which GPIO pin to
 * read. Valid values are AD77681_GPIO0, AD77681_GPIO1,
 * AD77681_GPIO2, AD77681_GPIO3, or AD77681_ALL_GPIOS.
 * Invalid values result in an error.
 * @return Returns an int32_t indicating success (0) or an error code (-1 for
 * invalid GPIO number). The read value is stored in the location
 * pointed to by the value parameter.
 ******************************************************************************/
int32_t ad77681_gpio_read(struct ad77681_dev *dev,
			  uint8_t *value,
			  enum ad77681_gpios gpio_number);
/***************************************************************************//**
 * @brief This function sets an offset value for the AD7768-1 device by writing
 * the specified offset to the device's registers. It should be used when
 * an offset adjustment is needed for the device's operation. The
 * function requires a valid device structure and an offset value. It
 * returns an error code if the operation fails, which can be used to
 * diagnose issues with the SPI communication or device configuration.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param value A 32-bit unsigned integer representing the offset value to be
 * applied. The value is split into three bytes and written to the
 * device's registers.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad77681_apply_offset(struct ad77681_dev *dev,
			     uint32_t value);
/***************************************************************************//**
 * @brief This function sets the gain for the AD7768-1 device by writing a
 * 24-bit gain value to the device's gain registers. It should be called
 * when a specific gain configuration is required for the device's
 * operation. The function expects a valid device structure and a gain
 * value, which is split into three 8-bit segments and written to the
 * corresponding high, mid, and low gain registers. The function returns
 * an error code if any of the SPI write operations fail.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param value A 24-bit unsigned integer representing the gain value to be
 * applied. The value is split into three 8-bit segments for the
 * high, mid, and low gain registers.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in writing to the device.
 ******************************************************************************/
int32_t ad77681_apply_gain(struct ad77681_dev *dev,
			   uint32_t value);
/***************************************************************************//**
 * @brief This function sets the CRC selection mode for the AD7768-1 device,
 * which determines how data integrity is verified during SPI
 * communication. It should be called when configuring the device to
 * ensure the desired CRC mode is set. The function updates the device's
 * internal state to reflect the selected CRC mode and adjusts the SPI
 * interface settings accordingly. It must be called with a valid device
 * structure and a valid CRC selection enumeration value.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param crc_sel An enumeration value of type ad77681_crc_sel indicating the
 * desired CRC mode. Valid values are AD77681_CRC, AD77681_XOR,
 * and AD77681_NO_CRC. If an invalid value is provided, the
 * function may not behave as expected.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a non-zero
 * value indicates an error occurred during the SPI write operations.
 ******************************************************************************/
int32_t ad77681_set_crc_sel(struct ad77681_dev *dev,
			    enum ad77681_crc_sel crc_sel);
/***************************************************************************//**
 * @brief This function sets the output type for one or more GPIO pins on the
 * AD7768-1 device to either strong driver or open drain. It should be
 * called when the GPIO configuration needs to be changed, typically
 * during device initialization or reconfiguration. The function requires
 * a valid device structure and specific GPIO and output type
 * enumerations. If an invalid GPIO number is provided, the function
 * returns an error code. Ensure the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param gpio_number An enumeration value of type ad77681_gpios specifying
 * which GPIO(s) to configure. Valid values are
 * AD77681_GPIO0, AD77681_GPIO1, AD77681_GPIO2, or
 * AD77681_ALL_GPIOS.
 * @param output_type An enumeration value of type ad77681_gpio_output_type
 * specifying the desired output type. Valid values are
 * AD77681_GPIO_STRONG_DRIVER or AD77681_GPIO_OPEN_DRAIN.
 * @return Returns an int32_t indicating success (0) or an error code (-1) if an
 * invalid GPIO number is provided.
 ******************************************************************************/
int32_t ad77681_gpio_open_drain(struct ad77681_dev *dev,
				enum ad77681_gpios gpio_number,
				enum ad77681_gpio_output_type output_type);
/***************************************************************************//**
 * @brief This function enables or disables the continuous read mode on the
 * AD77681 device. It should be called when you need to switch the device
 * between continuous read mode and normal operation. The function
 * requires a valid device structure and a continuous read mode setting.
 * If enabling continuous read, the function writes the necessary
 * configuration to the device. If disabling, it sends a specific key to
 * exit the mode. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param continuous_enable An enum value of type ad77681_continuous_read
 * indicating whether to enable
 * (AD77681_CONTINUOUS_READ_ENABLE) or disable
 * (AD77681_CONTINUOUS_READ_DISABLE) continuous read
 * mode.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t ad77681_set_continuos_read(struct ad77681_dev *dev,
				   enum ad77681_continuous_read continuous_enable);
/***************************************************************************//**
 * @brief This function is used to clear various SPI-related error flags in the
 * AD7768-1 device, including ignore, read, write, and CRC errors. It
 * should be called when the device is in an error state to reset these
 * flags and ensure proper operation. The function requires a valid
 * device structure and returns an error code if any operation fails. It
 * is important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null. The caller retains ownership of the structure.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error occurred while clearing the flags.
 ******************************************************************************/
int32_t ad77681_clear_error_flags(struct ad77681_dev *dev);
/***************************************************************************//**
 * @brief This function is used to convert a raw ADC code into a corresponding
 * voltage value based on the reference voltage of the AD7768-1 device.
 * It should be called when you have a raw ADC code that needs to be
 * interpreted as a voltage. The function requires a valid device
 * structure and pointers to the raw code and voltage variables. The raw
 * code is interpreted as a 24-bit signed integer, and the resulting
 * voltage is calculated using the device's reference voltage. Ensure
 * that the device structure is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param raw_code A pointer to a 32-bit unsigned integer containing the raw ADC
 * code. Must not be null.
 * @param voltage A pointer to a double where the calculated voltage will be
 * stored. Must not be null.
 * @return Returns 0 on successful conversion. The voltage is written to the
 * location pointed to by the voltage parameter.
 ******************************************************************************/
int32_t ad77681_data_to_voltage(struct ad77681_dev *dev,
				uint32_t *raw_code,
				double *voltage);
/***************************************************************************//**
 * @brief This function is used to validate the integrity of ADC data by
 * checking the CRC and status byte, if enabled, based on the device
 * configuration. It should be called after reading data from the ADC to
 * ensure data integrity. The function requires a properly initialized
 * device structure and a data buffer containing the ADC data. It returns
 * an error code if the CRC check fails, indicating data corruption. This
 * function is essential for applications where data integrity is
 * critical.
 *
 * @param dev A pointer to an initialized ad77681_dev structure. This must not
 * be null and should be configured with the desired CRC and status
 * bit settings.
 * @param data_buffer A pointer to a buffer containing the ADC data to be
 * checked. The buffer must be properly populated with data
 * from the ADC and must not be null.
 * @return Returns 0 on success, or -1 if the CRC check fails, indicating data
 * corruption.
 ******************************************************************************/
int32_t ad77681_CRC_status_handling(struct ad77681_dev *dev,
				    uint16_t *data_buffer);
/***************************************************************************//**
 * @brief This function sets the precharge configuration for the AIN- buffer of
 * the AD7768-1 device. It should be used when you need to enable or
 * disable the precharge buffer for the AIN- input. The function must be
 * called with a valid device structure and a valid precharge setting. It
 * updates the device configuration and returns a status code indicating
 * success or failure. This function is typically used during the device
 * initialization or configuration phase.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param AINn An enum value of type ad77681_AINn_precharge indicating the
 * desired precharge setting for the AIN- buffer. Valid values are
 * AD77681_AINn_ENABLED or AD77681_AINn_DISABLED.
 * @return Returns an int32_t status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t ad77681_set_AINn_buffer(struct ad77681_dev *dev,
				enum ad77681_AINn_precharge AINn);
/***************************************************************************//**
 * @brief This function sets the precharge configuration for the AIN+ buffer of
 * the AD7768-1 device. It should be used when you need to enable or
 * disable the precharge buffer for the AIN+ input. This function must be
 * called with a valid device structure and a valid precharge setting. It
 * is typically used during the initialization or configuration phase of
 * the device setup. The function returns an error code if the operation
 * fails, which can be used to verify successful configuration.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param AINp An enum value of type ad77681_AINp_precharge indicating the
 * desired precharge setting for the AIN+ buffer. Valid values are
 * AD77681_AINp_ENABLED or AD77681_AINp_DISABLED.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad77681_set_AINp_buffer(struct ad77681_dev *dev,
				enum ad77681_AINp_precharge AINp);
/***************************************************************************//**
 * @brief This function sets the configuration for the negative reference buffer
 * of the AD7768-1 device. It should be called when you need to enable,
 * disable, or fully buffer the negative reference input. The function
 * requires a valid device structure and a reference buffer setting. It
 * communicates with the device over SPI to apply the setting. If the
 * operation is successful, the device structure is updated to reflect
 * the new setting. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param REFn An enum value of type ad77681_REFn_buffer indicating the desired
 * buffer setting. Valid values are AD77681_BUFn_ENABLED,
 * AD77681_BUFn_DISABLED, and AD77681_BUFn_FULL_BUFFER_ON.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad77681_set_REFn_buffer(struct ad77681_dev *dev,
				enum ad77681_REFn_buffer REFn);
/***************************************************************************//**
 * @brief This function sets the REF+ buffer configuration for the AD7768-1
 * device, which is essential for managing the reference voltage
 * buffering. It should be called when the device is initialized and
 * before starting any data acquisition to ensure the correct buffer
 * setting is applied. The function updates the device's internal state
 * to reflect the new REF+ buffer setting. It is important to handle the
 * return value to check for any errors during the configuration process.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null, and the device should be properly
 * initialized before calling this function.
 * @param REFp An enum value of type ad77681_REFp_buffer indicating the desired
 * REF+ buffer setting. Valid values are AD77681_BUFp_ENABLED,
 * AD77681_BUFp_DISABLED, and AD77681_BUFp_FULL_BUFFER_ON.
 * @return Returns 0 on success, or -1 if an error occurs during the SPI write
 * operation.
 ******************************************************************************/
int32_t ad77681_set_REFp_buffer(struct ad77681_dev *dev,
				enum ad77681_REFp_buffer REFp);
/***************************************************************************//**
 * @brief This function sets the digital filter type and decimation rate for the
 * AD7768-1 device. It should be called when you need to configure the
 * filter settings of the device, typically during initialization or when
 * changing the data processing configuration. The function requires a
 * valid device structure and appropriate filter and decimation settings.
 * It updates the device's internal configuration and initiates a
 * synchronization pulse after the filter change. Ensure that the device
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param decimate An enum value of type ad77681_sinc5_fir_decimate specifying
 * the decimation rate for SINC5 and FIR filters. Valid values
 * are defined in the enum.
 * @param filter An enum value of type ad77681_filter_type specifying the filter
 * type. Valid values include AD77681_SINC5, AD77681_SINC5_DECx8,
 * AD77681_SINC5_DECx16, AD77681_SINC3, and AD77681_FIR.
 * @param sinc3_osr A 16-bit unsigned integer specifying the oversampling ratio
 * for the SINC3 filter. Used only when the filter type is
 * AD77681_SINC3.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad77681_set_filter_type(struct ad77681_dev *dev,
				enum ad77681_sinc5_fir_decimate decimate,
				enum ad77681_filter_type filter,
				uint16_t sinc3_osr);
/***************************************************************************//**
 * @brief This function is used to configure the AD7768-1 device to either
 * enable or disable the 50Hz rejection feature, which is useful for
 * reducing interference from power line frequencies in certain regions.
 * It should be called when setting up the device's digital filter
 * settings. The function requires a valid device structure pointer and a
 * flag indicating whether to enable or disable the feature. It returns
 * an integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param enable A uint8_t value where 1 enables 50Hz rejection and 0 disables
 * it. Values outside this range may lead to undefined behavior.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad77681_set_50HZ_rejection(struct ad77681_dev *dev,
				   uint8_t enable);
/***************************************************************************//**
 * @brief This function is used to either put the AD77681 device into a low-
 * power sleep mode or wake it up from sleep mode. It is essential to
 * manage the power state of the device to conserve energy or prepare it
 * for operation. The function must be called with a valid device
 * structure and a specified power state. It is important to ensure that
 * the device is properly initialized before calling this function. The
 * function returns an integer status code indicating the success or
 * failure of the operation.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param sleep_wake An enum value of type ad77681_sleep_wake indicating the
 * desired power state. Valid values are AD77681_SLEEP to put
 * the device to sleep and AD77681_WAKE to wake it up.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad77681_power_down(struct ad77681_dev *dev,
			   enum ad77681_sleep_wake sleep_wake);
/***************************************************************************//**
 * @brief This function is used to set or clear the status bit in the AD7768-1
 * device's interface format register. It should be called when there is
 * a need to modify the status bit, which may affect the device's
 * communication or operational status. The function must be called with
 * a valid device structure that has been properly initialized. If the
 * operation is successful, the device's internal status bit is updated
 * accordingly. The function returns an error code if the operation
 * fails, allowing the caller to handle such cases appropriately.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param status_bit A boolean value indicating the desired state of the status
 * bit. True to set the bit, false to clear it.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad77681_set_status_bit(struct ad77681_dev *dev,
			       bool status_bit);
/***************************************************************************//**
 * @brief This function configures the VCM (common-mode voltage) output level of
 * the AD7768-1 device. It should be called when there is a need to
 * adjust the VCM output to a specific level as defined by the `VCM_out`
 * parameter. The function requires a valid device structure and a valid
 * VCM output level enumeration. It returns an error code if the
 * operation fails, otherwise it updates the device's VCM output setting.
 *
 * @param dev A pointer to an `ad77681_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param VCM_out An enumeration value of type `ad77681_VCM_out` specifying the
 * desired VCM output level. Must be a valid enumeration value.
 * @return Returns an `int32_t` indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad77681_set_VCM_output(struct ad77681_dev *dev,
			       enum ad77681_VCM_out VCM_out);
/***************************************************************************//**
 * @brief This function is used to set the output value of a specific GPIO pin
 * on the AD7768-1 device. It can be used to control individual GPIO pins
 * or all GPIOs simultaneously. The function requires a valid device
 * structure and a GPIO number to specify which pin to write to. If an
 * invalid GPIO number is provided, the function returns an error. This
 * function is typically called when the GPIOs need to be controlled for
 * specific hardware configurations or signaling purposes.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param value A uint8_t representing the value to write to the GPIO.
 * Typically, 0 or 1 for individual GPIOs.
 * @param gpio_number An enum ad77681_gpios specifying the GPIO pin to write to.
 * Valid values are AD77681_GPIO0, AD77681_GPIO1,
 * AD77681_GPIO2, AD77681_GPIO3, or AD77681_ALL_GPIOS.
 * Invalid values result in an error return.
 * @return Returns an int32_t indicating success (0) or an error code (-1 for
 * invalid GPIO number).
 ******************************************************************************/
int32_t ad77681_gpio_write(struct ad77681_dev *dev,
			   uint8_t value,
			   enum ad77681_gpios gpio_number);
/***************************************************************************//**
 * @brief This function sets the direction (input or output) of a specified GPIO
 * pin on the AD7768-1 device. It can be used to configure individual
 * GPIO pins or all GPIOs at once. The function must be called with a
 * valid device structure and appropriate GPIO number. If an invalid GPIO
 * number is provided, the function returns an error. This function is
 * typically used in the initialization phase or when reconfiguring GPIO
 * settings during operation.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param direction A uint8_t value indicating the desired direction: 0 for
 * input, 1 for output. Values outside this range may lead to
 * undefined behavior.
 * @param gpio_number An enum ad77681_gpios value specifying which GPIO pin(s)
 * to configure. Valid values are AD77681_GPIO0,
 * AD77681_GPIO1, AD77681_GPIO2, AD77681_GPIO3, or
 * AD77681_ALL_GPIOS. Invalid values result in an error
 * return.
 * @return Returns an int32_t status code: 0 on success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad77681_gpio_inout(struct ad77681_dev *dev,
			   uint8_t direction,
			   enum ad77681_gpios gpio_number);
/***************************************************************************//**
 * @brief This function is used to enable or disable the global GPIO
 * functionality on the AD7768-1 device. It should be called when there
 * is a need to change the GPIO configuration globally, affecting all
 * GPIOs on the device. The function requires a valid device structure
 * and a specified enable or disable state. It is important to ensure
 * that the device has been properly initialized before calling this
 * function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param gpio_enable An enum value of type ad77681_gobal_gpio_enable indicating
 * whether to enable or disable the global GPIO. Valid values
 * are AD77681_GLOBAL_GPIO_ENABLE or
 * AD77681_GLOBAL_GPIO_DISABLE.
 * @return Returns an int32_t indicating success or failure of the operation. A
 * non-zero value indicates an error.
 ******************************************************************************/
int32_t ad77681_global_gpio(struct ad77681_dev *devices,
			    enum ad77681_gobal_gpio_enable gpio_enable);
/***************************************************************************//**
 * @brief This function is used to verify the integrity of the scratchpad
 * register in the AD7768-1 device by writing a sequence to it and then
 * reading it back to ensure it matches the original. It should be called
 * when you need to confirm that the device's scratchpad register is
 * functioning correctly. The function requires a valid device structure
 * and a sequence to test. If the read sequence does not match the
 * written sequence, the function returns an error code.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param sequence A pointer to a uint8_t value representing the sequence to
 * write to the scratchpad. The caller retains ownership, and it
 * must not be null. The function will read back and compare
 * this sequence.
 * @return Returns 0 on success if the sequence matches, or -1 if the sequence
 * read back does not match the sequence written. Additionally, it
 * returns any error code from the underlying SPI operations.
 ******************************************************************************/
int32_t ad77681_scratchpad(struct ad77681_dev *dev,
			   uint8_t *sequence);
/***************************************************************************//**
 * @brief This function enables various error flag reporting mechanisms for the
 * AD7768-1 device, including SPI errors, ADC diagnostic errors, and
 * digital diagnostic errors. It should be called when error monitoring
 * is required to ensure proper operation and debugging of the device.
 * The function must be called with a valid device structure that has
 * been properly initialized. It returns an error code if any of the
 * enabling operations fail, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * any enabling operation fails.
 ******************************************************************************/
int32_t ad77681_error_flags_enabe(struct ad77681_dev *dev);
/***************************************************************************//**
 * @brief This function calculates and updates the sample rate of the AD7768-1
 * device based on its current configuration settings, including the
 * master clock divider and filter settings. It should be called whenever
 * the configuration of the device changes to ensure the sample rate is
 * correctly updated. The function requires a valid device structure with
 * properly initialized fields. If the configuration settings are
 * invalid, the function returns an error code.
 *
 * @param dev A pointer to an ad77681_dev structure representing the device.
 * This structure must be initialized and contain valid configuration
 * settings. The pointer must not be null.
 * @return Returns 0 on success, or -1 if the configuration settings are
 * invalid.
 ******************************************************************************/
int32_t ad77681_update_sample_rate(struct ad77681_dev *dev);
/***************************************************************************//**
 * @brief This function calculates the appropriate value for the SINC3
 * decimation register to achieve a specified output data rate (ODR) for
 * the AD7768-1 device. It should be called when configuring the device's
 * digital filter settings. The function requires a valid device
 * structure and a positive desired ODR. It returns an error if the ODR
 * is negative or if the calculated decimation value exceeds the
 * register's capacity.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param sinc3_dec_reg A pointer to a uint16_t where the calculated SINC3
 * decimation register value will be stored. Must not be
 * null.
 * @param sinc3_odr A float representing the desired output data rate in Hz.
 * Must be positive.
 * @return Returns 0 on success, or -1 if the input ODR is invalid or the
 * calculated decimation value is out of range.
 ******************************************************************************/
int32_t ad77681_SINC3_ODR(struct ad77681_dev *dev,
			  uint16_t *sinc3_dec_reg,
			  float sinc3_odr);
/***************************************************************************//**
 * @brief Use this function to read and update the status registers of an
 * AD7768-1 device, which provides diagnostic information about the
 * device's operation. This function should be called when you need to
 * check for errors or operational status of the device. Ensure that the
 * device has been properly initialized before calling this function. The
 * function reads multiple status registers and updates the provided
 * status structure with the current status flags. It returns an error
 * code if any of the SPI read operations fail.
 *
 * @param dev A pointer to an initialized ad77681_dev structure representing the
 * device. Must not be null.
 * @param status A pointer to an ad77681_status_registers structure where the
 * status information will be stored. Must not be null.
 * @return Returns an int32_t error code: 0 for success, or a negative error
 * code if any SPI read operation fails.
 ******************************************************************************/
int32_t ad77681_status(struct ad77681_dev *dev,
		       struct ad77681_status_registers *status);
#endif /* SRC_AD77681_H_ */
