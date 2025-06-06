/***************************************************************************//**
 *   @file   ad713x.h
 *   @brief  Header file for the ad713x Driver.
 *   @author SPopa (stefan.popa@analog.com)
 *   @author Andrei Drimbarean (andrei.drimbarean@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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

/*
 * Supported parts:
 *  - AD7134;
 *  - AD4134.
 */

#ifndef SRC_AD713X_H_
#define SRC_AD713X_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
/*
 * AD713X registers definition
 */
#define AD713X_REG_INTERFACE_CONFIG_A		0x00
#define AD713X_REG_INTERFACE_CONFIG_B		0x01
#define AD713X_REG_DEVICE_CONFIG		0x02
#define AD713X_REG_CHIP_TYPE			0x03
#define AD713X_REG_PRODUCT_ID_LSB		0x04
#define AD713X_REG_PRODUCT_ID_MSB		0x05
#define AD713X_REG_CHIP_GRADE			0x06
#define AD713X_REG_CHIP_INDEX			0x07
#define AD713X_REG_SCTATCH_PAD			0x0A
#define AD713X_REG_SPI_REVISION			0x0B
#define AD713X_REG_VENDOR_ID_LSB		0x0C
#define AD713X_REG_VENDOR_ID_MSB		0x0D
#define AD713X_REG_STREAM_MODE			0x0E
#define AD713X_REG_TRANSFER_REGISTER		0x0F
#define AD713X_REG_DEVICE_CONFIG1		0x10
#define AD713X_REG_DATA_PACKET_CONFIG		0x11
#define AD713X_REG_DIGITAL_INTERFACE_CONFIG	0x12
#define AD713X_REG_POWER_DOWN_CONTROL		0x13
#define AD713X_REG_AIN_RANGE_SELECT		0x14
#define AD713X_REG_DEVICE_STATUS		0x15
#define AD713X_REG_ODR_VAL_INT_LSB		0x16
#define AD713X_REG_ODR_VAL_INT_MID		0x17
#define AD713X_REG_ODR_VAL_INT_MSB		0x18
#define AD713X_REG_ODR_VAL_FLT_LSB		0x19
#define AD713X_REG_ODR_VAL_FLT_MID0		0x1A
#define AD713X_REG_ODR_VAL_FLT_MID1		0x1B
#define AD713X_REG_ODR_VAL_FLT_MSB		0x1C
#define AD713X_REG_CHANNEL_ODR_SELECT		0x1D
#define AD713X_REG_CHAN_DIG_FILTER_SEL		0x1E
#define AD713X_REG_FIR_BW_SEL			0x1F
#define AD713X_REG_GPIO_DIR_CTRL		0x20
#define AD713X_REG_GPIO_DATA			0x21
#define AD713X_REG_ERROR_PIN_SRC_CONTROL	0x22
#define AD713X_REG_ERROR_PIN_CONTROL		0x23
#define AD713X_REG_VCMBUF_CTRL			0x24
#define AD713X_REG_DIAGNOSTIC_CONTROL		0x25
#define AD713X_REG_MPC_CONFIG			0x26
#define AD713X_REG_CH0_GAIN_LSB			0x27
#define AD713X_REG_CH0_GAIN_MID			0x28
#define AD713X_REG_CH0_GAIN_MSB			0x29
#define AD713X_REG_CH0_OFFSET_LSB		0x2A
#define AD713X_REG_CH0_OFFSET_MID		0x2B
#define AD713X_REG_CH0_OFFSET_MSB		0x2C
#define AD713X_REG_CH1_GAIN_LSB			0x2D
#define AD713X_REG_CH1_GAIN_MID			0x2E
#define AD713X_REG_CH1_GAIN_MSB			0x2F
#define AD713X_REG_CH1_OFFSET_LSB		0x30
#define AD713X_REG_CH1_OFFSET_MID		0x31
#define AD713X_REG_CH1_OFFSET_MSB		0x32
#define AD713X_REG_CH2_GAIN_LSB			0x33
#define AD713X_REG_CH2_GAIN_MID			0x34
#define AD713X_REG_CH2_GAIN_MSB			0x35
#define AD713X_REG_CH2_OFFSET_LSB		0x36
#define AD713X_REG_CH2_OFFSET_MID		0x37
#define AD713X_REG_CH2_OFFSET_MSB		0x38
#define AD713X_REG_CH3_GAIN_LSB			0x39
#define AD713X_REG_CH3_GAIN_MID			0x3A
#define AD713X_REG_CH3_GAIN_MSB			0x3B
#define AD713X_REG_CH3_OFFSET_LSB		0x3C
#define AD713X_REG_CH3_OFFSET_MID		0x3D
#define AD713X_REG_CH3_OFFSET_MSB		0x3E
#define AD713X_REG_MCLK_COUNTER			0x3F
#define AD713X_REG_DIG_FILTER_OFUF		0x40
#define AD713X_REG_DIG_FILTER_SETTLED		0x41
#define AD713X_REG_INTERNAL_ERROR		0x42
#define AD713X_REG_POWER_OV_ERROR_1		0x43
#define AD713X_REG_POWER_UV_ERROR_1		0x44
#define AD713X_REG_POWER_OV_ERROR_2		0x45
#define AD713X_REG_POWER_UV_ERROR_2		0x46
#define AD713X_REG_SPI_ERROR			0x47
#define AD713X_REG_AIN_OR_ERROR			0x48
#define AD713X_REG_AVDD5_VALUE			0x49
#define AD713X_REG_DVDD5_VALUE			0x4A
#define AD713X_REG_VREF_VALUE			0x4B
#define AD713X_REG_LDOIN_VALUE			0x4C
#define AD713X_REG_AVDD1V8_VALUE		0x4D
#define AD713X_REG_DVDD1V8_VALUE		0x4E
#define AD713X_REG_CLKVDD_VALUE			0x4F
#define AD713X_REG_IOVDD_VALUE			0x50
#define AD713X_REG_TEMPERATURE_DATA		0x51

/*
 * AD713X_REG_INTERFACE_CONFIG_A
 */
#define AD713X_INT_CONFIG_A_SOFT_RESET_MSK		NO_OS_BIT(7)
#define AD713X_INT_CONFIG_A_ADDR_ASC_BIT_MSK		NO_OS_BIT(5)
#define AD713X_INT_CONFIG_A_SDO_ACTIVE_BIT_MSK		NO_OS_BIT(4)
#define AD713X_INT_CONFIG_A_SOFT_RESET_MIRR_MSK		NO_OS_BIT(0)
#define AD713X_INT_CONFIG_A_ADDR_ASC_MIRR_MSK		NO_OS_BIT(2)
#define AD713X_INT_CONFIG_A_SDO_ACTIVE_MIRR_MSK		NO_OS_BIT(3)

/*
 * AD713X_REG_INTERFACE_CONFIG_B
 */
#define AD713X_INT_CONFIG_B_SINGLE_INSTR_MSK		NO_OS_BIT(7)
#define AD713X_INT_CONFIG_B_M_S_RD_CTRL_MSK		NO_OS_BIT(5)
#define AD713X_INT_CONFIG_B_DIG_IF_RST_MSK		NO_OS_BIT(1)

/*
 * AD713X_REG_DEVICE_CONFIG
 */
#define AD713X_DEV_CONFIG_OP_IN_PROGRESS_MSK		NO_OS_BIT(5)
#define AD713X_DEV_CONFIG_NO_CHIP_ERR_MSK		NO_OS_BIT(4)
#define AD713X_DEV_CONFIG_PWR_MODE_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_CHIP_TYPE
 */
#define AD713X_CHIP_TYPE_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_CHIP_TYPE_BITS_MODE(x)			(((x) & 0xFF) << 0)
#define AD713X_CHIP_TYPE				0x07

/*
 * AD713X_REG_PRODUCT_ID_LSB
 */
#define AD713X_PRODUCT_ID_LSB_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_PRODUCT_ID_LSB_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_PRODUCT_ID_MSB
 */
#define AD713X_PRODUCT_ID_MSB_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_PRODUCT_ID_MSB_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHIP_GRADE
 */
#define AD713X_CHIP_GRADE_PROD_GRADE_BITS_MSK		NO_OS_GENMASK(7, 4)
#define AD713X_CHIP_GRADE_PROD_GRADE_BITS_MODE(x) 	(((x) & 0x0F) << 4)
#define AD713X_CHIP_GRADE_DEV_VERSION_BITS_MSK		NO_OS_GENMASK(3, 0)
#define AD713X_CHIP_GRADE_DEV_VERSION_BITS_MODE(x) 	(((x) & 0x0F) << 0)

/*
 * AD713X_REG_CHIP_INDEX
 */
#define AD713X_SILICON_REV_ID_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_SILICON_REV_ID_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_SCRATCH_PAD
 */
#define AD713X_SCRATCH_PAD_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_SCRATCH_PAD_BITS_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_SPI_REVISION
 */
#define AD713X_SPI_REVISION_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_SPI_REVISION_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_VENDOR_ID_LSB
 */
#define AD713X_VENDOR_ID_LSB_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_VENDOR_ID_LSB_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_VENDOR_ID_MSB
 */
#define AD713X_VENDOR_ID_MSB_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_VENDOR_ID_MSB_BITS_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_STREAM_MODE
 */
#define AD713X_STREAM_MODE_BITS_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_STREAM_MODE_BITS_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_TRANSFER_REGISTER
 */
#define AD713X_TRANSFER_MASTER_SLAVE_TX_BIT_MSK		NO_OS_BIT(0)

/*
 * AD713X_REG_DEVICE_CONFIG1
 */
#define AD713X_DEV_CONFIG1_MPC_MAGPHA_EN_MSK		NO_OS_BIT(6)
#define AD713X_DEV_CONFIG1_MPC_MAG_EN_MSK		NO_OS_BIT(5)
#define AD713X_DEV_CONFIG1_AA_MODE_MSK			NO_OS_BIT(4)
#define AD713X_DEV_CONFIG1_SDO_PIN_SRC_SEL_MSK		NO_OS_BIT(2)
#define AD713X_DEV_CONFIG1_REF_GAIN_CORR_EN_MSK		NO_OS_BIT(1)
#define AD713X_DEV_CONFIG1_CLKOUT_EN_MSK		NO_OS_BIT(0)

/*
 * AD713X_REG_DATA_PACKET_CONFIG
 */
#define AD713X_DATA_PACKET_CONFIG_CRC_POLY_RST_MSK	NO_OS_BIT(7)
#define AD713X_DATA_PACKET_CONFIG_FRAME_MSK		NO_OS_GENMASK(6, 4)
#define AD713X_DATA_PACKET_CONFIG_FRAME_MODE(x)		(((x) & 0x7) << 4)
#define AD713X_DATA_PACKET_CONFIG_DCLK_FREQ_MSK		NO_OS_GENMASK(3, 0)
#define AD713X_DATA_PACKET_CONFIG_DCLK_FREQ_MODE(x)	(((x) & 0xF) << 0)

/*
 * AD713X_REG_DIGITAL_INTERFACE_CONFIG
 */
#define AD713X_DIG_INT_CONFIG_DAISY_CHAIN_NUM_MSK	NO_OS_GENMASK(7, 4)
#define AD713X_DIG_INT_CONFIG_DAISY_CHAIN_NUM_MODE(x)	(((x) & 0xF) << 4)
#define AD713X_DIG_INT_CONFIG_AVG_SEL_MSK		NO_OS_GENMASK(3, 2)
#define AD713X_DIG_INT_CONFIG_AVG_SEL_MODE(x)		(((x) & 0x3) << 2)
#define AD713X_DIG_INT_CONFIG_FORMAT_MSK		NO_OS_GENMASK(1, 0)
#define AD713X_DIG_INT_CONFIG_FORMAT_MODE(x)		(((x) & 0x3) << 0)

/*
 * AD713X_REG_POWER_DOWN_CONTROL
 */
#define AD713X_PWRDN_CTRL_PWRDN_CH_MSK(ch)		NO_OS_BIT(ch)
#define AD713X_PWRDN_CTRL_PWRDN_AUXADC_MSK		NO_OS_BIT(2)
#define AD713X_PWRDN_CTRL_PWRDN_LDO_MSK			NO_OS_BIT(1)
#define AD713X_PWRDN_CTRL_PWRDN_SLEEP_MODE_EN_MSK	NO_OS_BIT(0)

/*
 * AD713X_REG_AIN_RANGE_SELECT
 */
#define AD713X_AIN_RANGE_SEL_CH_MSK(ch)			NO_OS_BIT(ch)

/*
 * AD713X_REG_DEVICE_STATUS
 */
#define AD713X_DEV_STAT_DCLKMODE_MSK			NO_OS_BIT(5)
#define AD713X_DEV_STAT_DCLKIO_MSK			NO_OS_BIT(4)
#define AD713X_DEV_STAT_MODE_MSK			NO_OS_BIT(3)
#define AD713X_DEV_STAT_CLKSEL_MSK			NO_OS_BIT(2)
#define AD713X_DEV_STAT_FUSE_ECC_MSK			NO_OS_BIT(1)
#define AD713X_DEV_STAT_PLL_LOCK_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_ODR_VAL_INT_LSB
 */
#define AD713X_ODR_VAL_INT_LSB_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_INT_LSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_INT_MID
 */
#define AD713X_ODR_VAL_INT_MID_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_INT_MID_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_INT_MSB
 */
#define AD713X_ODR_VAL_INT_MSB_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_INT_MSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_FLT_LSB
 */
#define AD713X_ODR_VAL_FLT_LSB_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_FLT_LSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_FLT_MID0
 */
#define AD713X_ODR_VAL_FLT_MID0_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_FLT_MID0_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_FLT_MID1
 */
#define AD713X_ODR_VAL_FLT_MID1_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_FLT_MID1_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ODR_VAL_FLT_MSB
 */
#define AD713X_ODR_VAL_FLT_MSB_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_ODR_VAL_FLT_MSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHANNEL_ODR_SELECT
 */
#define AD713X_ODR_RATE_SEL_CH_MSK(ch)			(NO_OS_GENMASK(1, 0) << (2 * ch))
#define AD713X_ODR_RATE_SEL_CH_MODE(x, ch)		(((x) & 0x3) << (2 * ch))

/*
 * AD713X_REG_CHAN_DIG_FILTER_SEL
 */
#define AD713X_DIGFILTER_SEL_CH_MSK(ch)			(NO_OS_GENMASK(1, 0) << (2 * ch))
#define AD713X_DIGFILTER_SEL_CH_MODE(x, ch)		(((x) & 0x3) << (2 * ch))

/*
 * AD713X_REG_FIR_BW_SEL
 */
#define AD713X_FIR_BW_SEL_CH_MSK(ch)			NO_OS_BIT(ch)

/*
 * AD713X_REG_GPIO_DIR_CTRL
 */
#define AD713X_GPIO_IO_CTRL_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_GPIO_IO_CTRL_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_GPIO_DATA
 */
#define AD713X_GPIO_DATA_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_GPIO_DATA_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_ERROR_PIN_SRC_CONTROL
 */
#define AD713X_ERR_PIN_EN_OR_AIN_MSK			NO_OS_BIT(5)
#define AD713X_ERR_PIN_EN_INTERNAL_MSK			NO_OS_BIT(4)
#define AD713X_ERR_PIN_EN_SPI_MSK			NO_OS_BIT(3)
#define AD713X_ERR_PIN_EN_LDO_XOSC_MSK			NO_OS_BIT(2)
#define AD713X_ERR_PIN_EN_TEMP_MSK			NO_OS_BIT(1)
#define AD713X_ERR_PIN_EN_PWR_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_ERROR_PIN_CONTROL
 */
#define AD713X_ERR_PIN_IN_STATUS_MSK			NO_OS_BIT(2)
#define AD713X_ERR_PIN_IN_EN_MSK			NO_OS_BIT(1)
#define AD713X_ERR_PIN_OUT_EN_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_VCMBUF_CTRL
 */
#define AD713X_VCMBUF_CTRL_PWRDN_MSK			NO_OS_BIT(6)
#define AD713X_VCMBUF_CTRL_REF_DIV_SEL_MSK		NO_OS_GENMASK(5, 1)
#define AD713X_VCMBUF_CTRL_REF_DIV_SEL_MODE(x)		(((x) & 0x1F) << 1)
#define AD713X_VCMBUF_CTRL_REF_SEL_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_DIAGNOSTIC_CONTROL
 */
#define AD713X_DIAGCTRL_ERR_OR_AIN_EN_MSK		NO_OS_BIT(5)
#define AD713X_DIAGCTRL_ERR_PWR_MON_EN_MSK		NO_OS_BIT(4)
#define AD713X_DIAGCTRL_MCLK_CNT_EN_MSK			NO_OS_BIT(3)
#define AD713X_DIAGCTRL_ERR_SPI_CRC_EN_MSK		NO_OS_BIT(2)
#define AD713X_DIAGCTRL_ERR_MM_CRC_EN_MSK		NO_OS_BIT(1)
#define AD713X_DIAGCTRL_FUSE_CRC_CHECK_MSK		NO_OS_BIT(0)

/*
 * AD713X_REG_MPC_CONFIG
 */
#define AD713X_MPC_CLKDEL_EN_CH_MSK(ch)			(NO_OS_GENMASK(1, 0) << (2 * ch))
#define AD713X_MPC_CLKDEL_EN_CH_MODE(x, ch)		(((x) & 0x3) << (2 * ch))

/*
 * AD713X_REG_CHx_GAIN_LSB
 */
#define AD713X_CH_GAIN_LSB_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_CH_GAIN_LSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHx_GAIN_MID
 */
#define AD713X_CH_GAIN_MID_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_CH_GAIN_MID_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHx_GAIN_MSB
 */
#define AD713X_CH_GAIN_CAL_SEL_MSK			NO_OS_BIT(4)
#define AD713X_CH_GAIN_MSB_MSK				NO_OS_GENMASK(3, 0)
#define AD713X_CH_GAIN_MSB_MODE(x)			(((x) & 0xF) << 0)

/*
 * AD713X_REG_CHx_OFFSET_LSB
 */
#define AD713X_CH_OFFSET_LSB_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_CH_OFFSET_LSB_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHx_OFFSET_MID
 */
#define AD713X_CH_OFFSET_MID_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_CH_OFFSET_MID_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CHx_OFFSET_MSB
 */
#define AD713X_CH_OFFSET_CAL_EN_MSK			NO_OS_BIT(7)
#define AD713X_CH_OFFSET_MSB_MSK			NO_OS_GENMASK(6, 0)
#define AD713X_CH_OFFSET_MSB_MODE(x)			(((x) & 0x7F) << 0)

/*
 * AD713X_REG_MCLK_COUNTER
 */
#define AD713X_MCLK_COUNT_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_MCLK_COUNT_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_DIG_FILTER_OFUF
 */
#define AD713X_DIGFILTER_ERR_OFUF_CH_MSK(ch)		NO_OS_BIT(ch)

/*
 * AD713X_REG_DIG_FILTER_SETTLED
 */
#define AD713X_DIGFILTER_CH_SETTLED_MSK(ch)		NO_OS_BIT(ch)

/*
 * AD713X_REG_INTERNAL_ERROR
 */
#define AD713X_INT_ERR_NO_CLOCK_MSK			NO_OS_BIT(5)
#define AD713X_INT_ERR_TEMP_MSK				NO_OS_BIT(4)
#define AD713X_INT_ERR_DCLK_MSK				NO_OS_BIT(3)
#define AD713X_INT_ERR_FUSE_CRC_MSK			NO_OS_BIT(2)
#define AD713X_INT_ERR_ASRC_MSK				NO_OS_BIT(1)
#define AD713X_INT_ERR_MM_CRC_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_POWER_OV_ERROR_1
 */
#define AD713X_POWER_ERR_OV_IOVDD_MSK			NO_OS_BIT(3)
#define AD713X_POWER_ERR_OV_CLKVDD_MSK			NO_OS_BIT(2)
#define AD713X_POWER_ERR_OV_DVDD1V8_MSK			NO_OS_BIT(1)
#define AD713X_POWER_ERR_OV_AVDD1V8_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_POWER_UV_ERROR_1
 */
#define AD713X_POWER_ERR_UV_IOVDD_MSK			NO_OS_BIT(3)
#define AD713X_POWER_ERR_UV_CLKVDD_MSK			NO_OS_BIT(2)
#define AD713X_POWER_ERR_UV_DVDD1V8_MSK			NO_OS_BIT(1)
#define AD713X_POWER_ERR_UV_AVDD1V8_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_POWER_OV_ERROR_2
 */
#define AD713X_POWER_ERR_OV_VREF_MSK			NO_OS_BIT(3)
#define AD713X_POWER_ERR_OV_LDOIN_MSK			NO_OS_BIT(2)
#define AD713X_POWER_ERR_OV_DVDD5_MSK			NO_OS_BIT(1)
#define AD713X_POWER_ERR_OV_AVDD5_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_POWER_UV_ERROR_2
 */
#define AD713X_POWER_ERR_UV_VREF_MSK			NO_OS_BIT(3)
#define AD713X_POWER_ERR_UV_LDOIN_MSK			NO_OS_BIT(2)
#define AD713X_POWER_ERR_UV_DVDD5_MSK			NO_OS_BIT(1)
#define AD713X_POWER_ERR_UV_AVDD5_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_SPI_ERROR
 */
#define AD713X_SPI_ERROR_CRC_MSK			NO_OS_BIT(3)
#define AD713X_SPI_ERROR_SCLK_CNT_MSK			NO_OS_BIT(2)
#define AD713X_SPI_ERROR_WRITE_MSK			NO_OS_BIT(1)
#define AD713X_SPI_ERROR_READ_MSK			NO_OS_BIT(0)

/*
 * AD713X_REG_AIN_OR_ERROR
 */
#define AD713X_ERR_OR_AIN_MSK(ch)			NO_OS_BIT(ch)

/*
 * AD713X_REG_AVDD5_VALUE
 */
#define AD713X_AVDD5_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_AVDD5_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_DVDD5_VALUE
 */
#define AD713X_DVDD5_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_DVDD5_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_VREF_VALUE
 */
#define AD713X_VREF_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_VREF_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_LDOIN_VALUE
 */
#define AD713X_LDOIN_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_LDOIN_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_AVDD1V8_VALUE
 */
#define AD713X_AVDD1V8_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_AVDD1V8_VALUE_PIN_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_DVDD1V8_VALUE
 */
#define AD713X_DVDD1V8_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_DVDD1V8_VALUE_PIN_MODE(x)		(((x) & 0xFF) << 0)

/*
 * AD713X_REG_CLKVDD_VALUE
 */
#define AD713X_CLKVDD_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_CLKVDD_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_IOVDD_VALUE
 */
#define AD713X_IOVDD_VALUE_PIN_MSK			NO_OS_GENMASK(7, 0)
#define AD713X_IOVDD_VALUE_PIN_MODE(x)			(((x) & 0xFF) << 0)

/*
 * AD713X_REG_TEMPERATURE_DATA
 */
#define AD713X_TEMP_DATA_MSK				NO_OS_GENMASK(7, 0)
#define AD713X_TEMP_DATA_MODE(x)			(((x) & 0xFF) << 0)

#define AD713X_REG_READ(x)				((1 << 7) | (x & 0x7F))

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ad713x_supported_dev_ids` enumeration defines a set of constants
 * representing the IDs of devices supported by the AD713x driver. Each
 * enumerator corresponds to a specific device model, allowing the driver
 * to identify and handle different devices within the AD713x series.
 * This enumeration is crucial for ensuring that the driver can correctly
 * interface with and manage the specific features and configurations of
 * each supported device.
 *
 * @param ID_AD7132 Represents the device ID for AD7132.
 * @param ID_AD7134 Represents the device ID for AD7134.
 * @param ID_AD7136 Represents the device ID for AD7136.
 * @param ID_AD4134 Represents the device ID for AD4134.
 ******************************************************************************/
enum ad713x_supported_dev_ids {
	ID_AD7132,
	ID_AD7134,
	ID_AD7136,
	ID_AD4134
};

/***************************************************************************//**
 * @brief The `ad713x_power_mode` enumeration defines the power modes available
 * for the AD713x series of devices. It provides two options: `LOW_POWER`
 * for reduced power consumption and `HIGH_POWER` for full performance
 * operation. This enumeration is used to configure the power mode of the
 * device, allowing users to optimize for either power efficiency or
 * maximum performance depending on their application needs.
 *
 * @param LOW_POWER Represents the low power mode option for the AD713x device.
 * @param HIGH_POWER Represents the full power mode option for the AD713x
 * device.
 ******************************************************************************/
enum ad713x_power_mode {
	/** Low power mode option */
	LOW_POWER,
	/** Full power mode option */
	HIGH_POWER
};

/***************************************************************************//**
 * @brief The `ad713x_adc_data_len` enumeration defines the possible bit lengths
 * for data samples in the AD713x series of devices. It includes options
 * for 16-bit, 24-bit, and 32-bit data samples, as well as an `INVALID`
 * option to denote the end of valid data length options. This
 * enumeration is used to configure and manage the data output format of
 * the AD713x devices.
 *
 * @param ADC_16_BIT_DATA Represents a 16-bit data sample.
 * @param ADC_24_BIT_DATA Represents a 24-bit data sample.
 * @param ADC_32_BIT_DATA Represents a 32-bit data sample.
 * @param INVALID Indicates an invalid data length, used to signal the end of
 * valid options.
 ******************************************************************************/
enum ad713x_adc_data_len {
	/** 16 bit data sample */
	ADC_16_BIT_DATA,
	/** 24 bit data sample */
	ADC_24_BIT_DATA,
	/** 32 bit data sample */
	ADC_32_BIT_DATA,
	/** To know when to stop when cycling between them */
	INVALID
};

/***************************************************************************//**
 * @brief The `ad713x_crc_header` is an enumeration that defines the possible
 * CRC (Cyclic Redundancy Check) header options for data samples in the
 * AD713x series of devices. It specifies whether a data sample is
 * transmitted without a CRC, or with a 6-bit or 8-bit CRC, providing
 * flexibility in data integrity verification.
 *
 * @param NO_CRC Data sample comes with no CRC attached.
 * @param CRC_6 Data sample comes with 6-bit CRC attached.
 * @param CRC_8 Data sample comes with 8-bit CRC attached.
 ******************************************************************************/
enum ad713x_crc_header {
	/** Data sample comes with no CRC attached */
	NO_CRC,
	/** Data sample comes with 6-bit CRC attached */
	CRC_6,
	/** Data sample comes with 8-bit CRC attached */
	CRC_8
};

/***************************************************************************//**
 * @brief The `ad713x_doutx_format` enumeration defines the possible output
 * modes for the AD713x series of devices. These modes determine how data
 * is output from the device, with options for single or dual channel
 * daisy-chain modes, a quad-channel parallel output mode, and a channel
 * average mode. This allows for flexible data handling and output
 * configurations depending on the specific application requirements.
 *
 * @param SINGLE_CH_DC Single channel Daisy-chain mode.
 * @param DUAL_CH_DC Dual channel Daisy-chain mode.
 * @param QUAD_CH_PO Quad-channel parallel output mode.
 * @param CH_AVG_MODE Channel average mode.
 ******************************************************************************/
enum ad713x_doutx_format {
	/** Single channel Daisy-chain mode */
	SINGLE_CH_DC,
	/** Dual channel Daisy-chain mode */
	DUAL_CH_DC,
	/** Quad-channel parallel output mode */
	QUAD_CH_PO,
	/** Channel average mode */
	CH_AVG_MODE
};

/***************************************************************************//**
 * @brief The `ad713x_dig_filter_sel` enumeration defines a set of digital
 * filter options available for the AD713x series of devices. These
 * filters include a wideband finite impulse response filter, a Sinc6
 * filter, a Sinc3 filter, and a Sinc3 filter with specific rejection
 * capabilities for 50Hz and 60Hz frequencies. This enumeration is used
 * to select the type of digital filtering applied to the input signals
 * in the AD713x devices, allowing for customization based on the
 * application's requirements.
 *
 * @param FIR Represents a wideband filter using finite impulse response.
 * @param SINC6 Represents a Sinc6 filter.
 * @param SINC3 Represents a Sinc3 filter.
 * @param SINC3_50_60_REJ Represents a Sinc3 filter with 50Hz and 60Hz
 * rejection.
 ******************************************************************************/
enum ad713x_dig_filter_sel {
	/** Wideband filter (Finite impulse response) */
	FIR,
	/** Sinc6 filter */
	SINC6,
	/** Sinc3 filter */
	SINC3,
	/** Sinc3 filter with 50Hz and 60Hz rejection */
	SINC3_50_60_REJ
};

/***************************************************************************//**
 * @brief The `ad713x_channels` enumeration defines a set of constants
 * representing the available channels (CH0 to CH3) for the AD713x series
 * of devices, as well as a constant (`AD713X_CH_MAX`) that specifies the
 * maximum number of channels supported. This enumeration is used to
 * identify and manage the different channels within the AD713x driver.
 *
 * @param CH0 Represents Channel 0.
 * @param CH1 Represents Channel 1.
 * @param CH2 Represents Channel 2.
 * @param CH3 Represents Channel 3.
 * @param AD713X_CH_MAX Indicates the maximum number of channels.
 ******************************************************************************/
enum ad713x_channels {
	/** Channel 0 */
	CH0,
	/** Channel 1 */
	CH1,
	/** Channel 2 */
	CH2,
	/** Channel 3 */
	CH3,
	/** Max number of channels */
	AD713X_CH_MAX
};

/***************************************************************************//**
 * @brief The `ad717x_mpc_clkdel` enumeration defines the possible clock delay
 * settings for the AD713x series of devices. It provides options for no
 * delay, a delay of one clock cycle, or a delay of two clock cycles,
 * allowing for precise control over the timing of operations in the
 * device.
 *
 * @param DELAY_NONE Represents no delay in clock cycles.
 * @param DELAY_1_CLOCKS Represents a delay of one clock cycle.
 * @param DELAY_2_CLOCKS Represents a delay of two clock cycles.
 ******************************************************************************/
enum ad717x_mpc_clkdel {
	/** No delay */
	DELAY_NONE,
	/** One clock cycle delay */
	DELAY_1_CLOCKS,
	/** Two clock cycles delay */
	DELAY_2_CLOCKS
};

/***************************************************************************//**
 * @brief The `ad713x_dev` structure is a comprehensive handler for the AD713x
 * driver, encapsulating all necessary components for interfacing with
 * the device. It includes pointers to SPI and GPIO descriptors for
 * managing communication and control signals, as well as enumerations
 * for device identification, data length, and CRC options. This
 * structure is essential for initializing and operating the AD713x
 * series of devices, providing a centralized configuration and control
 * point for the driver.
 *
 * @param spi_desc Pointer to the SPI layer handler.
 * @param gpio_mode Pointer to the MODE GPIO handler.
 * @param gpio_dclkmode Pointer to the DCLKMODE GPIO handler.
 * @param gpio_dclkio Pointer to the DCLKIO GPIO handler.
 * @param gpio_resetn Pointer to the RESET GPIO handler.
 * @param gpio_pnd Pointer to the PDN GPIO handler.
 * @param gpio_cs_sync Pointer to the CS Sync SPI descriptor.
 * @param dev_id ID of the supported device.
 * @param adc_data_len Length of data in bits.
 * @param crc_header CRC option for data integrity.
 * @param mode_master_nslave Boolean indicating the starting value of the MODE
 * GPIO.
 ******************************************************************************/
struct ad713x_dev {
	/** SPI layer handler. */
	struct no_os_spi_desc        	*spi_desc;
	/** MODE GPIO handler. */
	struct no_os_gpio_desc		*gpio_mode;
	/** DCLKMODE GPIO handler. */
	struct no_os_gpio_desc		*gpio_dclkmode;
	/** DCLKIO GPIO handler. */
	struct no_os_gpio_desc		*gpio_dclkio;
	/** RESET GPIO handler. */
	struct no_os_gpio_desc		*gpio_resetn;
	/** PDN GPIO handler. */
	struct no_os_gpio_desc		*gpio_pnd;
	/** CS Sync */
	struct no_os_spi_desc *gpio_cs_sync;
	/** ID of supported device. */
	enum ad713x_supported_dev_ids dev_id;
	/** Length of data in bits. */
	enum ad713x_adc_data_len	adc_data_len;
	/** CRC option. */
	enum ad713x_crc_header	crc_header;
	/** MODE GPIO starting value */
	bool mode_master_nslave;
};

/***************************************************************************//**
 * @brief The `ad713x_init_param` structure is used to initialize the AD713x
 * driver, containing various parameters for SPI and GPIO configurations,
 * device identification, data length, CRC options, and clock settings.
 * It includes initialization structures for SPI and multiple GPIOs,
 * starting values for GPIOs, and options for data format and clock
 * delay. This structure is essential for setting up the AD713x device,
 * ensuring proper communication and functionality according to the
 * specified configurations.
 *
 * @param spi_init_prm SPI layer initialization structure.
 * @param gpio_mode MODE GPIO initialization structure.
 * @param gpio_dclkmode DCLKMODE GPIO initialization structure.
 * @param gpio_dclkio DCLKIO GPIO initialization structure.
 * @param gpio_resetn RESET GPIO initialization structure.
 * @param gpio_pnd PDN GPIO initialization structure.
 * @param gpio_cs_sync CS_GPIO initialization structure.
 * @param mode_master_nslave MODE GPIO starting value.
 * @param dclkmode_free_ngated DCLKMODE GPIO starting value.
 * @param dclkio_out_nin DCLKIO GPIO starting value.
 * @param pnd PDN GPIO starting value.
 * @param dev_id ID of supported device.
 * @param adc_data_len Length of data in bits.
 * @param crc_header CRC option.
 * @param format DOUTx output format.
 * @param clk_delay_en Clock delay state.
 * @param spi_common_dev SPI layer handler if the SPI bus is shared with another
 * device.
 ******************************************************************************/
struct ad713x_init_param {
	/** SPI layer initialization structure. */
	struct no_os_spi_init_param spi_init_prm;
	/** MODE GPIO initialization structure. */
	struct no_os_gpio_init_param *gpio_mode;
	/** DCLKMODE GPIO initialization structure. */
	struct no_os_gpio_init_param *gpio_dclkmode;
	/** DCLKIO GPIO initialization structure. */
	struct no_os_gpio_init_param *gpio_dclkio;
	/** RESET GPIO initialization structure. */
	struct no_os_gpio_init_param *gpio_resetn;
	/** PDN GPIO initialization structure. */
	struct no_os_gpio_init_param *gpio_pnd;
	/** CS_GPIO */
	struct no_os_gpio_init_param *gpio_cs_sync;
	/** MODE GPIO starting value */
	bool mode_master_nslave;
	/** DCLKMODE GPIO starting value */
	bool dclkmode_free_ngated;
	/** DCLKIO GPIO starting value */
	bool dclkio_out_nin;
	/** PDN GPIO starting value */
	bool pnd;
	/** ID of supported device. */
	enum ad713x_supported_dev_ids dev_id;
	/** Length of data in bits. */
	enum ad713x_adc_data_len	adc_data_len;
	/** CRC option. */
	enum ad713x_crc_header	crc_header;
	enum ad713x_doutx_format	format;
	/** Clock delay state. */
	bool 			clk_delay_en;
	/** SPI layer handler if the SPI bus is shared with another device. In this
	 *  case the SPI should not be initialized again. */
	struct no_os_spi_desc *spi_common_dev;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to read a specific register from an AD713x
 * device using SPI communication. It is essential to ensure that the
 * device has been properly initialized and configured before calling
 * this function. The function requires a valid device structure and a
 * register address to read from. It will store the read value in the
 * provided data pointer. If the SPI communication fails, the function
 * returns an error code. This function is typically used when you need
 * to retrieve configuration or status information from the device.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address for the AD713x device.
 * @param reg_data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if an error occurs during SPI
 * communication.
 ******************************************************************************/
int32_t ad713x_spi_reg_read(struct ad713x_dev *dev, uint8_t reg_addr,
			    uint8_t *reg_data);

/***************************************************************************//**
 * @brief This function is used to write a byte of data to a specific register
 * on the AD713x device via the SPI interface. It requires a valid device
 * structure that has been properly initialized and configured for SPI
 * communication. The function is typically called when there is a need
 * to configure or modify the settings of the AD713x device by writing to
 * its registers. It is important to ensure that the device is ready for
 * communication and that the register address and data are within valid
 * ranges before calling this function.
 *
 * @param dev A pointer to an ad713x_dev structure representing the device. This
 * must be initialized and must not be null. The caller retains
 * ownership.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the AD713x device.
 * @param reg_data The data byte to write to the specified register. Must be a
 * valid data value for the register.
 * @return Returns an int32_t indicating the success or failure of the SPI write
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad713x_spi_reg_write(struct ad713x_dev *dev, uint8_t reg_addr,
			     uint8_t reg_data);

/***************************************************************************//**
 * @brief This function allows you to modify specific bits of a register on the
 * AD713x device by applying a mask. It is useful when you need to change
 * only certain bits of a register without affecting the others. The
 * function first reads the current value of the register, applies the
 * mask to clear the bits specified by the mask, and then sets the bits
 * according to the provided data. It should be called when the device is
 * properly initialized and ready for SPI communication. If the read
 * operation fails, the function returns an error code.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD713x device.
 * @param mask A 32-bit mask indicating which bits of the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the masked bits of the register.
 * Only the bits specified by the mask will be updated.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int32_t ad713x_spi_write_mask(struct ad713x_dev *dev, uint8_t reg_addr,
			      uint32_t mask, uint8_t data);

/***************************************************************************//**
 * @brief This function configures the power mode of an AD713x device to either
 * low power or high power. It should be called when there is a need to
 * change the power consumption characteristics of the device, such as
 * during different operational states or to conserve energy. The
 * function requires a valid device structure and a specified power mode.
 * If an invalid power mode is provided, the function will return an
 * error code. This function is typically used after the device has been
 * initialized and is ready for operation.
 *
 * @param dev A pointer to an ad713x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param mode An enum value of type ad713x_power_mode indicating the desired
 * power mode. Valid values are LOW_POWER and HIGH_POWER. If an
 * invalid mode is provided, the function returns an error.
 * @return Returns 0 on success, or -1 if an invalid power mode is specified.
 ******************************************************************************/
int32_t ad713x_set_power_mode(struct ad713x_dev *dev,
			      enum ad713x_power_mode mode);

/***************************************************************************//**
 * @brief This function sets the output data frame configuration for the AD713x
 * device, based on the specified data length and CRC header options. It
 * should be called after the device has been initialized and before
 * starting data acquisition. The function checks if the provided data
 * length and CRC header combination is valid for the device. If the
 * combination is valid, it updates the device configuration; otherwise,
 * it returns an error. This function is essential for ensuring that the
 * data output format matches the expected configuration for subsequent
 * data processing.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param adc_data_len Specifies the number of bits per data sample. Must be a
 * valid value from the ad713x_adc_data_len enumeration,
 * excluding INVALID.
 * @param crc_header Specifies the CRC header option for the data. Must be a
 * valid value from the ad713x_crc_header enumeration.
 * @return Returns 0 on success if the configuration is valid, or -1 if the
 * combination of adc_data_len and crc_header is invalid for the device.
 ******************************************************************************/
int32_t ad713x_set_out_data_frame(struct ad713x_dev *dev,
				  enum ad713x_adc_data_len adc_data_len,
				  enum ad713x_crc_header crc_header);

/***************************************************************************//**
 * @brief This function sets the digital output format for the AD713x device,
 * which determines how data is output from the device. It should be
 * called when you need to change the output format to match your data
 * processing requirements. Ensure that the device is properly
 * initialized before calling this function. The function modifies the
 * device's configuration register to set the desired output format, and
 * it returns an error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param format An enum value of type ad713x_doutx_format specifying the
 * desired output format. Valid values are SINGLE_CH_DC,
 * DUAL_CH_DC, QUAD_CH_PO, and CH_AVG_MODE.
 * @return Returns an int32_t error code: 0 for success, or a negative value for
 * failure.
 ******************************************************************************/
int32_t ad713x_dout_format_config(struct ad713x_dev *dev,
				  enum ad713x_doutx_format format);

/***************************************************************************//**
 * @brief This function is used to enable or disable a clock delay for magnitude
 * and phase matching calibration across all channels of the AD713x
 * device. It should be called when there is a need to adjust the clock
 * delay for calibration purposes. The function requires a valid device
 * structure and a boolean flag indicating whether the clock delay should
 * be enabled or disabled. It returns an error code if the operation
 * fails, which can occur if there is an issue with the SPI
 * communication.
 *
 * @param dev A pointer to an ad713x_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param clk_delay_en A boolean value indicating whether to enable (true) or
 * disable (false) the clock delay for calibration. Any non-
 * boolean value is considered invalid and may lead to
 * undefined behavior.
 * @return Returns 0 on success, or -1 if an error occurs during SPI
 * communication.
 ******************************************************************************/
int32_t ad713x_mag_phase_clk_delay(struct ad713x_dev *dev, bool clk_delay_en);

/***************************************************************************//**
 * @brief This function is used to configure the digital filter type for a
 * specific channel on the AD713x device. It should be called when you
 * need to change the filtering characteristics of a channel, which can
 * be useful for optimizing signal processing based on application
 * requirements. Ensure that the device is properly initialized before
 * calling this function. The function modifies the filter settings for
 * the specified channel, and it is important to pass valid filter and
 * channel enumerations to avoid unexpected behavior.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param filter An enumeration value of type ad713x_dig_filter_sel representing
 * the desired digital filter type. Valid values are FIR, SINC6,
 * SINC3, and SINC3_50_60_REJ.
 * @param ch An enumeration value of type ad713x_channels representing the
 * channel to configure. Valid values are CH0, CH1, CH2, and CH3.
 * @return Returns an int32_t indicating success (0) or a negative error code on
 * failure.
 ******************************************************************************/
int32_t ad713x_dig_filter_sel_ch(struct ad713x_dev *dev,
				 enum ad713x_dig_filter_sel filter, enum ad713x_channels ch);

/***************************************************************************//**
 * @brief This function is used to control the CLKOUT output of the AD713x
 * device. It can be called to enable or disable the CLKOUT signal, which
 * is useful for synchronizing with other devices or systems. The
 * function must be called with a valid device structure that has been
 * properly initialized. It is important to ensure that the device is in
 * a state where changing the CLKOUT output is safe and will not disrupt
 * ongoing operations.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param enable A boolean value where 'true' enables the CLKOUT output and
 * 'false' disables it.
 * @return Returns an int32_t indicating success (0) or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t ad713x_clkout_output_en(struct ad713x_dev *dev, bool enable);

/***************************************************************************//**
 * @brief This function is used to control the reference gain correction feature
 * of the AD713x device. It should be called when you need to enable or
 * disable this feature, which may be necessary depending on the specific
 * application requirements or calibration needs. The function requires a
 * valid device structure and a boolean flag indicating whether to enable
 * or disable the correction. It is important to ensure that the device
 * has been properly initialized before calling this function to avoid
 * undefined behavior.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param enable A boolean value where 'true' enables reference gain correction
 * and 'false' disables it.
 * @return Returns an int32_t indicating success or failure of the operation,
 * typically 0 for success and a negative value for failure.
 ******************************************************************************/
int32_t ad713x_ref_gain_correction_en(struct ad713x_dev *dev, bool enable);

/***************************************************************************//**
 * @brief This function is used to configure the wideband filter bandwidth for a
 * specific channel on the AD713x device. It should be called when you
 * need to adjust the filter settings for a particular channel, which can
 * be useful in optimizing the device's performance for different
 * applications. The function requires a valid device structure and
 * channel identifier, and it modifies the filter bandwidth based on the
 * provided option. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null.
 * @param ch An enum value of type ad713x_channels specifying the channel for
 * which the bandwidth is to be set. Must be a valid channel
 * identifier.
 * @param wb_opt A uint8_t value indicating the wideband option to be set. Non-
 * zero values enable the wideband filter, while zero disables it.
 * @return Returns an int32_t status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t ad713x_wideband_bw_sel(struct ad713x_dev *dev,
			       enum ad713x_channels ch, uint8_t wb_opt);

/***************************************************************************//**
 * @brief This function sets up the AD713x device by initializing the necessary
 * hardware interfaces and configuring the device according to the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function handles both the allocation of
 * resources and the configuration of the device registers. If the SPI
 * bus is shared, the function will use the provided SPI descriptor
 * instead of initializing a new one. The function returns an error code
 * if initialization fails, ensuring that the device is not left in an
 * undefined state.
 *
 * @param device A pointer to a pointer of type `struct ad713x_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A pointer to a `struct ad713x_init_param` containing the
 * initialization parameters. This includes SPI and GPIO
 * configurations, device ID, data length, CRC options, and
 * other settings. The pointer must not be null, and the
 * structure must be properly initialized before calling the
 * function.
 * @return Returns 0 on successful initialization, or -1 if an error occurs
 * during the process.
 ******************************************************************************/
int32_t ad713x_init(struct ad713x_dev **device,
		    struct ad713x_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD713x
 * device instance. It should be called when the device is no longer
 * needed, typically at the end of its lifecycle, to ensure proper
 * cleanup and avoid memory leaks. The function expects a valid device
 * structure pointer and will return an error if the pointer is null or
 * if any internal cleanup operation fails.
 *
 * @param dev A pointer to an ad713x_dev structure representing the device to be
 * removed. Must not be null. The function will return an error if
 * this parameter is invalid.
 * @return Returns 0 on successful removal of the device resources, or -1 if an
 * error occurs during the process.
 ******************************************************************************/
int32_t ad713x_remove(struct ad713x_dev *dev);

/***************************************************************************//**
 * @brief Use this function to print the values of specific registers from an
 * AD713x device to the standard output. It is useful for debugging and
 * verifying the current configuration of the device. The function reads
 * a predefined set of registers and outputs their values. It must be
 * called with a valid device structure that has been properly
 * initialized. If any register read operation fails, the function will
 * print an error message and return an error code.
 *
 * @param dev A pointer to an initialized ad713x_dev structure representing the
 * device. Must not be null. The function will attempt to read from
 * this device.
 * @return Returns 0 on success, or -1 if any register read operation fails.
 ******************************************************************************/
int32_t ad713x_spi_reg_dump(struct ad713x_dev *dev);

/***************************************************************************//**
 * @brief This function is used to synchronize the channels of an AD713x device
 * by toggling the CS_SYNC GPIO and configuring the interface. It should
 * be called when channel synchronization is required, such as after
 * initialization or configuration changes. The function expects a valid
 * device structure and handles errors by returning a negative value if
 * any GPIO or SPI operation fails.
 *
 * @param dev A pointer to an ad713x_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @return Returns 0 on success or -1 if an error occurs during GPIO or SPI
 * operations.
 ******************************************************************************/
int32_t ad713x_channel_sync(struct ad713x_dev *dev);

#endif /* SRC_AD713X_H_ */
