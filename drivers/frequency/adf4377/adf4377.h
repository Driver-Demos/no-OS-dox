/***************************************************************************//**
 *   @file   adf4377.h
 *   @brief  Header file for adf4377 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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

#ifndef ADF4377_H_
#define ADF4377_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_util.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Registers Control Bits */
#define ADF4377_REG(x)					(x)

/* ADF4377 REG0000 Map */
#define ADF4377_SOFT_RESET_R_MSK        NO_OS_BIT(7)
#define ADF4377_SOFT_RESET_R(x)			no_os_field_prep(ADF4377_SOFT_RESET_R_MSK, x)
#define ADF4377_SOFT_RESET_MSK         	NO_OS_BIT(0)
#define ADF4377_SOFT_RESET(x)			no_os_field_prep(ADF4377_SOFT_RESET_MSK, x)
#define ADF4377_LSB_FIRST_R_MSK			NO_OS_BIT(6)
#define ADF4377_LSB_FIRST_R(x)       	no_os_field_prep(ADF4377_LSB_FIRST_R_MSK, x)
#define ADF4377_LSB_FIRST_MSK			NO_OS_BIT(1)
#define ADF4377_LSB_FIRST(x)       		no_os_field_prep(ADF4377_LSB_FIRST_MSK, x)
#define ADF4377_ADDRESS_ASC_R_MSK		NO_OS_BIT(5)
#define ADF4377_ADDRESS_ASC_R(x)		no_os_field_prep(ADF4377_ADDRESS_ASC_R_MSK, x)
#define ADF4377_ADDRESS_ASC_MSK			NO_OS_BIT(2)
#define ADF4377_ADDRESS_ASC(x)			no_os_field_prep(ADF4377_ADDRESS_ASC_MSK, x)
#define ADF4377_SDO_ACTIVE_R_MSK       	NO_OS_BIT(4)
#define ADF4377_SDO_ACTIVE_R(x)			no_os_field_prep(ADF4377_SDO_ACTIVE_R_MSK, x)
#define ADF4377_SDO_ACTIVE_MSK         	NO_OS_BIT(3)
#define ADF4377_SDO_ACTIVE(x)			no_os_field_prep(ADF4377_SDO_ACTIVE_MSK, x)

/* ADF4377 REG0000 Bit Definition */
#define ADF4377_SDO_ACTIVE_SPI_3W       0x0
#define ADF4377_SDO_ACTIVE_SPI_4W		0x1

#define ADF4377_ADDR_ASC_AUTO_DECR      0x0
#define ADF4377_ADDR_ASC_AUTO_INCR      0x1

#define ADF4377_LSB_FIRST_MSB           0x0
#define ADF4377_LSB_FIRST_LSB           0x1

#define ADF4377_SOFT_RESET_N_OP         0x0
#define ADF4377_SOFT_RESET_EN           0x1

/* ADF4377 REG0001 Map */
#define ADF4377_SINGLE_INSTR_MSK		NO_OS_BIT(7)
#define ADF4377_SINGLE_INSTR(x)   		no_os_field_prep(ADF4377_SINGLE_INSTRUCTION_MSK, x)
#define ADF4377_MASTER_RB_CTRL_MSK		NO_OS_BIT(5)
#define ADF4377_MASTER_RB_CTRL(x)		no_os_field_prep(ADF4377_MASTER_RB_CTRL_MSK, x)

/* ADF4377 REG0001 Bit Definition */
#define ADF4377_SPI_STREAM_EN           0x0
#define ADF4377_SPI_STREAM_DIS          0x1

#define ADF4377_RB_SLAVE_REG            0x0
#define ADF4377_RB_MASTER_REG           0x1

/* ADF4377 REG0003 Bit Definition */
#define ADF4377_CHIP_TYPE               0x06

/* ADF4377 REG0004 Bit Definition */
#define ADF4377_PRODUCT_ID_LSB          0x0005

/* ADF4377 REG0005 Bit Definition */
#define ADF4377_PRODUCT_ID_MSB          0x0005

/* ADF4377 REG000A Map */
#define ADF4377_SCRATCHPAD_MSK			NO_OS_GENMASK(7, 0)
#define ADF4377_SCRATCHPAD(x)           no_os_field_prep(ADF4377_SCRATCHPAD_MSK, x)

/* ADF4377 REG000B Bit Definition */
#define ADF4377_SPI_REVISION            0x01

/* ADF4377 REG000C Bit Definition */
#define ADF4377_VENDOR_ID_LSB           0x456

/* ADF4377 REG000D Bit Definition */
#define ADF4377_VENDOR_ID_MSB			0x456

/* ADF4377 REG000F Bit Definition */
#define ADF4377_R00F_RSV1				0x14

/* ADF4377 REG0010 Map*/
#define ADF4377_N_INT_LSB_MSK			NO_OS_GENMASK(7, 0)
#define ADF4377_N_INT_LSB(x)            no_os_field_prep(ADF4377_N_INT_LSB_MSK, x)

/* ADF4377 REG0011 Map*/
#define ADF4377_EN_AUTOCAL_MSK			NO_OS_BIT(7)
#define ADF4377_EN_AUTOCAL(x)           no_os_field_prep(ADF4377_EN_AUTOCAL_MSK, x)
#define ADF4377_EN_RDBLR_MSK			NO_OS_BIT(6)
#define ADF4377_EN_RDBLR(x)             no_os_field_prep(ADF4377_EN_RDBLR_MSK, x)
#define ADF4377_DCLK_DIV2_MSK			NO_OS_GENMASK(5,4)
#define ADF4377_DCLK_DIV2(x)            no_os_field_prep(ADF4377_DCLK_DIV2_MSK, x)
#define ADF4377_N_INT_MSB_MSK			NO_OS_GENMASK(3,0)
#define ADF4377_N_INT_MSB(x)            no_os_field_prep(ADF4377_N_INT_MSB_MSK, x)

/* ADF4377 REG0011 Bit Definition */
#define ADF4377_VCO_CALIB_DIS           0x0
#define ADF4377_VCO_CALIB_EN            0x1

#define ADF4377_REF_DBLR_DIS            0x0
#define ADF4377_REF_DBLR_EN             0x1

#define ADF4377_DCLK_DIV2_1             0x0
#define ADF4377_DCLK_DIV2_2             0x1
#define ADF4377_DCLK_DIV2_4             0x2
#define ADF4377_DCLK_DIV2_8             0x3

/* ADF4377 REG0012 Map*/
#define ADF4377_CLKOUT_DIV_MSK			NO_OS_GENMASK(7, 6)
#define ADF4377_CLKOUT_DIV(x)           no_os_field_prep(ADF4377_CLKOUT_DIV_MSK, x)
#define ADF4377_R_DIV_MSK				NO_OS_GENMASK(5, 0)
#define ADF4377_R_DIV(x)                no_os_field_prep(ADF4377_R_DIV_MSK, x)

/* ADF4377 REG0012 Bit Definition */
#define ADF4377_CLKOUT_DIV_1            0x0
#define ADF4377_CLKOUT_DIV_2            0x1
#define ADF4377_CLKOUT_DIV_4            0x2
#define ADF4377_CLKOUT_DIV_8            0x3

#define ADF4377_MIN_R_DIV               0x00
#define ADF4378_MAX_R_DIV               0x3F

/* ADF4377 REG0013 Map */
#define ADF4377_M_VCO_CORE_MSK			NO_OS_GENMASK(5,4)
#define ADF4377_M_VCO_CORE(x)           no_os_field_prep(ADF4377_M_VCO_CORE_MSK, x)
#define ADF4377_M_VCO_BIAS_MSK          NO_OS_GENMASK(3,0)
#define ADF4377_M_VCO_BIAS(x)          	no_os_field_prep(ADF4377_M_VCO_BIAS_MSK, x)

/* ADF4377 REG0013 Bit Definition */
#define ADF4377_M_VCO_0                 0x0
#define ADF4377_M_VCO_1                 0x1
#define ADF4377_M_VCO_2                 0x2
#define ADF4377_M_VCO_3                 0x3

#define M_VCO_BIAS_MIN                  0xF
#define M_VCO_BIAS_MAX                  0x0

/* ADF4377 REG0014 Map */
#define ADF4377_M_VCO_BAND_MSK          NO_OS_GENMASK(7,0)
#define ADF4377_M_VCO_BAND(x)           no_os_field_prep(ADF4377_M_VCO_BAND_MSK, x)

/* ADF4377 REG0014 Bit Definition */
#define ADF4377_VCO_BAND_MIN            0xFF
#define ADF4377_VCO_BAND_MAX            0x00

/* ADF4377 REG0015 Map */
#define ADF4377_BLEED_I_LSB_MSK			NO_OS_GENMASK(7, 6)
#define ADF4377_BLEED_I_LSB(x)          no_os_field_prep(ADF4377_BLEED_I_LSB_MSK, x)
#define ADF4377_BLEED_POL_MSK			NO_OS_BIT(5)
#define ADF4377_BLEED_POL(x)            no_os_field_prep(ADF4377_BLEED_POL_MSK, x)
#define ADF4377_EN_BLEED_MSK			NO_OS_BIT(4)
#define ADF4377_EN_BLEED(x)             no_os_field_prep(ADF4377_EN_BLEED_MSK, x)
#define ADF4377_CP_I_MSK				NO_OS_GENMASK(3, 0)
#define ADF4377_CP_I(x)                 no_os_field_prep(ADF4377_CP_I_MSK, x)

/* ADF4377 REG0015 Bit Description */
#define ADF4377_CURRENT_SINK            0x0
#define ADF4377_CURRENT_SOURCE          0x1

#define ADF4377_CP_0MA7                 0x0
#define ADF4377_CP_0MA9                 0x1
#define ADF4377_CP_1MA1                 0x2
#define ADF4377_CP_1MA3                 0x3
#define ADF4377_CP_1MA4                 0x4
#define ADF4377_CP_1MA8                 0x5
#define ADF4377_CP_2MA2                 0x6
#define ADF4377_CP_2MA5                 0x7
#define ADF4377_CP_2MA9                 0x8
#define ADF4377_CP_3MA6                 0x9
#define ADF4377_CP_4MA3                 0xA
#define ADF4377_CP_5MA0                 0xB
#define ADF4377_CP_5MA7                 0xC
#define ADF4377_CP_7MA2                 0xD
#define ADF4377_CP_8MA6                 0xE
#define ADF4377_CP_10MA1                0xF

/* ADF4377 REG0016 Map */
#define ADF4377_BLEED_I_MSB_MSK			NO_OS_GENMASK(7, 0)
#define ADF4377_BLEED_I_MSB(x)          no_os_field_prep(ADF4377_BLEED_I_MSB_MSK, x)

/* ADF4377 REG0017 Map */
#define ADF4377_INV_CLKOUT_MSK			NO_OS_BIT(7)
#define ADF4377_INV_CLKOUT(x)           no_os_field_prep(ADF4377_INV_CLKOUT_MSK, x)
#define ADF4377_N_DEL_MSK				NO_OS_GENMASK(6, 0)
#define ADF4377_N_DEL(x)                no_os_field_prep(ADF4377_N_DEL_MSK, x)

/* ADF4377 REG0018 Map */
#define ADF4377_CMOS_OV_MSK				NO_OS_BIT(7)
#define ADF4377_CMOS_OV(x)              no_os_field_prep(ADF4377_CMOS_OV_MSK, x)
#define ADF4377_R_DEL_MSK				NO_OS_GENMASK(6, 0)
#define ADF4377_R_DEL(x)                no_os_field_prep(ADF4377_R_DEL_MSK, x)

/* ADF4377 REG0018 Bit Definition */
#define ADF4377_1V8_LOGIC               0x0
#define ADF4377_3V3_LOGIC               0x1

#define ADF4377_R_DEL_MIN               0x00
#define ADF4377_R_DEL_MAX               0x7F

/* ADF4377 REG0019 Map */
#define ADF4377_CLKOUT2_OP_MSK			NO_OS_GENMASK(7, 6)
#define ADF4377_CLKOUT2_OP(x)           no_os_field_prep(ADF4377_CLKOUT2_OP_MSK, x)
#define ADF4377_CLKOUT1_OP_MSK			NO_OS_GENMASK(5, 4)
#define ADF4377_CLKOUT1_OP(x)           no_os_field_prep(ADF4377_CLKOUT1_OP_MSK, x)
#define ADF4377_PD_CLK_MSK				NO_OS_BIT(3)
#define ADF4377_PD_CLK(x)               no_os_field_prep(ADF4377_PD_CLK_MSK, x)
#define ADF4377_PD_RDET_MSK				NO_OS_BIT(2)
#define ADF4377_PD_RDET(x)              no_os_field_prep(ADF4377_PD_RDET_MSK, x)
#define ADF4377_PD_ADC_MSK				NO_OS_BIT(1)
#define ADF4377_PD_ADC(x)               no_os_field_prep(ADF4377_PD_ADC_MSK, x)
#define ADF4377_PD_CALADC_MSK			NO_OS_BIT(0)
#define ADF4377_PD_CALADC(x)            no_os_field_prep(ADF4377_PD_CALADC_MSK, x)

/* ADF4377 REG0019 Bit Definition */
#define ADF4377_CLKOUT_320MV            0x0
#define ADF4377_CLKOUT_420MV            0x1
#define ADF4377_CLKOUT_530MV            0x2
#define ADF4377_CLKOUT_640MV            0x3

#define ADF4377_PD_CLK_N_OP             0x0
#define ADF4377_PD_CLK_PD               0x1

#define ADF4377_PD_RDET_N_OP            0x0
#define ADF4377_PD_RDET_PD              0x1

#define ADF4377_PD_ADC_N_OP             0x0
#define ADF4377_PD_ADC_PD               0x1

#define ADF4377_PD_CALADC_N_OP          0x0
#define ADF4377_PD_CALADC_PD            0x1

/* ADF4377 REG001A Map */
#define ADF4377_PD_ALL_MSK				NO_OS_BIT(7)
#define ADF4377_PD_ALL(x)               no_os_field_prep(ADF4377_PD_ALL_MSK, x)
#define ADF4377_PD_RDIV_MSK				NO_OS_BIT(6)
#define ADF4377_PD_RDIV(x)              no_os_field_prep(ADF4377_PD_RDIV_MSK, x)
#define ADF4377_PD_NDIV_MSK				NO_OS_BIT(5)
#define ADF4377_PD_NDIV(x)              no_os_field_prep(ADF4377_PD_NDIV_MSK, x)
#define ADF4377_PD_VCO_MSK				NO_OS_BIT(4)
#define ADF4377_PD_VCO(x)               no_os_field_prep(ADF4377_PD_VCO_MSK, x)
#define ADF4377_PD_LD_MSK				NO_OS_BIT(3)
#define ADF4377_PD_LD(x)                no_os_field_prep(ADF4377_PD_LD_MSK, x)
#define ADF4377_PD_PFDCP_MSK			NO_OS_BIT(2)
#define ADF4377_PD_PFDCP(x)             no_os_field_prep(ADF4377_PD_PFDCP_MSK, x)
#define ADF4377_PD_CLKOUT1_MSK			NO_OS_BIT(1)
#define ADF4377_PD_CLKOUT1(x)           no_os_field_prep(ADF4377_PD_CLKOUT1_MSK, x)
#define ADF4377_PD_CLKOUT2_MSK			NO_OS_BIT(0)
#define ADF4377_PD_CLKOUT2(x)           no_os_field_prep(ADF4377_PD_CLKOUT2_MSK, x)

/* ADF4377 REG001A Bit Definition */
#define ADF4377_PD_ALL_N_OP             0x0
#define ADF4377_PD_ALL_PD               0x1

#define ADF4377_PD_RDIV_N_OP            0x0
#define ADF4377_PD_RDIV_PD              0x1

#define ADF4377_PD_NDIV_N_OP            0x0
#define ADF4377_PD_NDIV_PD              0x1

#define ADF4377_PD_VCO_N_OP             0x0
#define ADF4377_PD_VCO_PD               0x1

#define ADF4377_PD_LD_N_OP              0x0
#define ADF4377_PD_LD_PD                0x1

#define ADF4377_PD_PFDCP_N_OP           0x0
#define ADF4377_PD_PFDCP_PD             0x1

#define ADF4377_PD_CLKOUT1_N_OP         0x0
#define ADF4377_PD_CLKOUT1_PD           0x1

#define ADF4377_PD_CLKOUT2_N_OP         0x0
#define ADF4377_PD_CLKOUT2_PD           0x1

/* ADF4377 REG001B Map */
#define ADF4377_EN_LOL_MSK				NO_OS_BIT(7)
#define ADF4377_EN_LOL(x)               no_os_field_prep(ADF4377_EN_LOL_MSK, x)
#define ADF4377_LDWIN_PW_MSK			NO_OS_BIT(6)
#define ADF4377_LDWIN_PW(x)             no_os_field_prep(ADF4377_LDWIN_PW_MSK, x)
#define ADF4377_EN_LDWIN_MSK			NO_OS_BIT(5)
#define ADF4377_EN_LDWIN(x)             no_os_field_prep(ADF4377_EN_LDWIN_MSK, x)
#define ADF4377_LD_COUNT_MSK			NO_OS_GENMASK(4, 0)
#define ADF4377_LD_COUNT(x)             no_os_field_prep(ADF4377_LD_COUNT_MSK, x)

/* ADF4377 REG001B Bit Definition */

#define ADF4377_LDWIN_PW_NARROW         0x0
#define ADF4377_LDWIN_PW_WIDE           0x1

/* ADF4377 REG001C Map */
#define ADF4377_EN_DNCLK_MSK			NO_OS_BIT(7)
#define ADF4377_EN_DNCLK(x)            	no_os_field_prep(ADF4377_EN_DNCLK_MSK, x)
#define ADF4377_EN_DRCLK_MSK			NO_OS_BIT(6)
#define ADF4377_EN_DRCLK(x)             no_os_field_prep(ADF4377_EN_DRCLK_MSK, x)
#define ADF4377_RST_LD_MSK				NO_OS_BIT(2)
#define ADF4377_RST_LD(x)               no_os_field_prep(ADF4377_RST_LD_MSK, x)
#define ADF4377_R01C_RSV1_MSK			NO_OS_BIT(0)
#define ADF4377_R01C_RSV1(x)			no_os_field_prep(ADF4377_R01C_RSV1_MSK, x)

/* ADF4377 REG001C Bit Definition */
#define ADF4377_EN_DNCLK_OFF            0x0
#define ADF4377_EN_DNCLK_ON             0x1

#define ADF4377_EN_DRCLK_OFF            0x0
#define ADF4377_EN_DRCLK_ON             0x1

#define ADF4377_RST_LD_INACTIVE         0x0
#define ADF4377_RST_LD_ACTIVE           0x1

/* ADF4377 REG001D Map */
#define ADF4377_MUXOUT_MSK				NO_OS_GENMASK(7, 4)
#define ADF4377_MUXOUT(x)               no_os_field_prep(ADF4377_MUXOUT_MSK, x)
#define ADF4377_EN_CPTEST_MSK			NO_OS_BIT(2)
#define ADF4377_EN_CPTEST(x)            no_os_field_prep(ADF4377_EN_CPTEST_MSK, x)
#define ADF4377_CP_DOWN_MSK				NO_OS_BIT(1)
#define ADF4377_CP_DOWN(x)              no_os_field_prep(ADF4377_CP_DOWN_MSK, x)
#define ADF4377_CP_UP_MSK				NO_OS_BIT(0)
#define ADF4377_CP_UP(x)                no_os_field_prep(ADF4377_CP_UP_MSK, x)

/* ADF4377 REG001D Bit Definitons */
#define ADF4377_MUXOUT_HIGH_Z           0x0
#define ADF4377_MUXOUT_LKDET            0x1
#define ADF4377_MUXOUT_LOW              0x2
#define ADF4377_MUXOUT_DIV_RCLK_2       0x4
#define ADF4377_MUXOUT_DIV_NCLK_2       0x5
#define ADF4377_MUXOUT_HIGH             0x8

#define ADF4377_EN_CPTEST_OFF           0x0
#define ADF4377_EN_CPTEST_ON            0x1

#define ADF4377_CP_DOWN_OFF             0x0
#define ADF4377_CP_DOWN_ON              0x1

#define ADF4377_CP_UP_OFF               0x0
#define ADF4377_CP_UP_ON                0x1

/* ADF4377 REG001F Map */
#define ADF4377_BST_REF_MSK				NO_OS_BIT(7)
#define ADF4377_BST_REF(x)              no_os_field_prep(ADF4377_BST_REF_MSK, x)
#define ADF4377_FILT_REF_MSK			NO_OS_BIT(6)
#define ADF4377_FILT_REF(x)             no_os_field_prep(ADF4377_FILT_REF_MSK, x)
#define ADF4377_REF_SEL_MSK				NO_OS_BIT(5)
#define ADF4377_REF_SEL(x)              no_os_field_prep(ADF4377_REF_SEL_MSK, x)
#define ADF4377_R01F_RSV1_MSK           NO_OS_GENMASK(2, 0)
#define ADF4377_R01F_RSV1(x) 			no_os_field_prep(ADF4377_R01F_RSV1_MSK, x)

/* ADF4377 REG001F Bit Description */
#define ADF4377_BST_LARGE_REF_IN        0x0
#define ADF4377_BST_SMALL_REF_IN        0x1

#define ADF4377_FILT_REF_OFF            0x0
#define ADF4377_FILT_REF_ON             0x1

#define ADF4377_REF_SEL_DMA             0x0
#define ADF4377_REF_SEL_LNA             0x1

/* ADF4377 REG0020 Map */
#define ADF4377_RST_SYS_MSK				NO_OS_BIT(4)
#define ADF4377_RST_SYS(x)              no_os_field_prep(ADF4377_RST_SYS_MSK, x)
#define ADF4377_EN_ADC_CLK_MSK			NO_OS_BIT(3)
#define ADF4377_EN_ADC_CLK(x)           no_os_field_prep(ADF4377_EN_ADC_CLK_MSK, x)
#define ADF4377_R020_RSV1_MSK           NO_OS_BIT(0)
#define ADF4377_R020_RSV1(x)			no_os_field_prep(ADF4377_R020_RSV1_MSK, x)

/* ADF4377 REG0020 Bit Description */
#define ADF4377_RST_SYS_INACTIVE        0x0
#define ADF4377_RST_SYS_ACTIVE          0x1

/* ADF4377 REG0021 Map */
#define ADF4377_R021_RSV1               0xD3

/* ADF4377 REG0022 Map */
#define ADF4377_R022_RSV1               0x32

/* ADF4377 REG0023 Map */
#define ADF4377_R023_RSV1               0x18

/* ADF4377 REG0024 Map */
#define ADF4377_DCLK_MODE_MSK			NO_OS_BIT(2)
#define ADF4377_DCLK_MODE(x)            no_os_field_prep(ADF4377_DCLK_MODE_MSK, x)

/* ADF4377 REG0025 Map */
#define ADF4377_CLKODIV_DB_MSK			NO_OS_BIT(7)
#define ADF4377_CLKODIV_DB(x)           no_os_field_prep(ADF4377_CLKODIV_DB_MSK, x)
#define ADF4377_DCLK_DB_MSK				NO_OS_BIT(6)
#define ADF4377_DCLK_DB(x)              no_os_field_prep(ADF4377_DCLK_DB_MSK, x)
#define ADF4377_R025_RSV1_MSK           NO_OS_BIT(4) | NO_OS_BIT(2) | NO_OS_BIT(1)
#define ADF4377_R025_RSV1(x)			no_os_field_prep(ADF4377_R025_RSV1_MSK, x)

/* ADF4377 REG0026 Map */
#define ADF4377_VCO_BAND_DIV_MSK    	NO_OS_GENMASK(7, 0)
#define ADF4377_VCO_BAND_DIV(x)         no_os_field_prep(ADF4377_VCO_BAND_DIV_MSK, x)

/* ADF4377 REG0026 Bit Definition */
#define ADF4377_VCO_BAND_DIV_MIN        0x00
#define ADF4377_VCO_BAND_DIV_MAX        0xFF

/* ADF4377 REG0027 Map */
#define ADF4377_SYNTH_LOCK_TO_LSB_MSK   NO_OS_GENMASK(7, 0)
#define ADF4377_SYNTH_LOCK_TO_LSB(x)   	no_os_field_prep(ADF4377_SYNTH_LOCK_TO_LSB_MSK, x)

/* ADF4377 REG0028 Map */
#define ADF4377_O_VCO_DB_MSK			NO_OS_BIT(7)
#define ADF4377_O_VCO_DB(x)             no_os_field_prep(ADF4377_O_VCO_DB_MSK, x)
#define ADF4377_SYNTH_LOCK_TO_MSB_MSK	NO_OS_GENMASK(6, 0)
#define ADF4377_SYNTH_LOCK_TO_MSB(x)   	no_os_field_prep(ADF4377_SYNTH_LOCK_TO_MSB_MSK, x)

/* ADF4377 REG0029 Map */
#define ADF4377_VCO_ALC_TO_LSB_MSK		NO_OS_GENMASK(7, 0)
#define ADF4377_VCO_ALC_TO_LSB(x)     	no_os_field_prep(ADF4377_VCO_ALC_TO_LSB_MSK, x)

/* ADF4377 REG002A Map */
#define ADF4377_DEL_CTRL_DB_MSK			NO_OS_BIT(7)
#define ADF4377_DEL_CTRL_DB(x)          no_os_field_prep(ADF4377_DEL_CTRL_DB_MSK, x)
#define ADF4377_VCO_ALC_TO_MSB_MSK		NO_OS_GENMASK(6, 0)
#define ADF4377_VCO_ALC_TO_MSB(x)      	no_os_field_prep(ADF4377_VCO_ALC_TO_MSB_MSK, x)

/* ADF4377 REG002C Map */
#define ADF4377_R02C_RSV1               0xC0

/* ADF4377 REG002D Map */
#define ADF4377_ADC_CLK_DIV_MSK			NO_OS_GENMASK(7, 0)
#define ADF4377_ADC_CLK_DIV(x)          no_os_field_prep(ADF4377_ADC_CLK_DIV_MSK, x)

/* ADF4377 REG002E Map */
#define ADF4377_EN_ADC_CNV_MSK			NO_OS_BIT(7)
#define ADF4377_EN_ADC_CNV(x)           no_os_field_prep(ADF4377_EN_ADC_CNV_MSK, x)
#define ADF4377_EN_ADC_MSK				NO_OS_BIT(1)
#define ADF4377_EN_ADC(x)               no_os_field_prep(ADF4377_EN_ADC_MSK, x)
#define ADF4377_ADC_A_CONV_MSK			NO_OS_BIT(0)
#define ADF4377_ADC_A_CONV(x)           no_os_field_prep(ADF4377_ADC_A_CONV_MSK, x)

/* ADF4377 REG002E Bit Definition */
#define ADF4377_ADC_A_CONV_ADC_ST_CNV   0x0
#define ADF4377_ADC_A_CONV_VCO_CALIB    0x1

/* ADF4377 REG002F Map */
#define ADF4377_DCLK_DIV1_MSK			NO_OS_GENMASK(1, 0)
#define ADF4377_DCLK_DIV1(x)            no_os_field_prep(ADF4377_DCLK_DIV1_MSK, x)

/* ADF4377 REG002F Bit Definition */
#define ADF4377_DCLK_DIV1_1             0x0
#define ADF4377_DCLK_DIV1_2             0x1
#define ADF4377_DCLK_DIV1_8             0x2
#define ADF4377_DCLK_DIV1_32            0x3

/* ADF4377 REG0031 Map */
#define ADF4377_R031_RSV1				0x09

/* ADF4377 REG0032 Map */
#define ADF4377_ADC_CLK_SEL_MSK			NO_OS_BIT(6)
#define ADF4377_ADC_CLK_SEL(x)          no_os_field_prep(ADF4377_ADC_CLK_SEL_MSK, x)
#define ADF4377_R032_RSV1_MSK           NO_OS_BIT(3) | NO_OS_BIT(0)
#define ADF4377_R032_RSV1(x)			no_os_field_prep(ADF4377_R032_RSV1_MSK, x)

/* ADF4377 REG0032 Bit Definition */
#define ADF4377_ADC_CLK_SEL_N_OP        0x0
#define ADF4377_ADC_CLK_SEL_SPI_CLK     0x1

/* ADF4377 REG0033 Map */
#define ADF4377_R033_RSV1               0x18

/* ADF4377 REG0034 Map */
#define ADF4377_R034_RSV1               0x08

/* ADF4377 REG003A Map */
#define ADF4377_R03A_RSV1               0x5C

/* ADF4377 REG003B Map */
#define ADF4377_R03B_RSV1               0x2B

/* ADF4377 REG003D Map */
#define ADF4377_O_VCO_BAND_MSK			NO_OS_BIT(3)
#define ADF4377_O_VCO_BAND(x)           no_os_field_prep(ADF4377_O_VCO_BAND_MSK, x)
#define ADF4377_O_VCO_CORE_MSK			NO_OS_BIT(2)
#define ADF4377_O_VCO_CORE(x)           no_os_field_prep(ADF4377_O_VCO_CORE_MSK, x)
#define ADF4377_O_VCO_BIAS_MSK			NO_OS_BIT(1)
#define ADF4377_O_VCO_BIAS(x)           no_os_field_prep(ADF4377_O_VCO_BIAS_MSK, x)

/* ADF4377 REG003D Bit Definition */
#define ADF4377_O_VCO_BAND_VCO_CALIB    0x0
#define ADF4377_O_VCO_BAND_M_VCO        0x1

#define ADF4377_O_VCO_CORE_VCO_CALIB    0x0
#define ADF4377_O_VCO_CORE_M_VCO        0x1

#define ADF4377_O_VCO_BIAS_VCO_CALIB    0x0
#define ADF4377_O_VCO_BIAS_M_VCO        0x1

/* ADF4377 REG0042 Map */
#define ADF4377_R042_RSV1               0x05

/* ADF4377 REG0045 Map */
#define ADF4377_ADC_ST_CNV_MSK			NO_OS_BIT(0)
#define ADF4377_ADC_ST_CNV(x)           no_os_field_prep(ADF4377_ADC_ST_CNV_MSK, x)

/* ADF4377 REG0049 Map */
#define ADF4377_EN_CLK2_MSK				NO_OS_BIT(7)
#define ADF4377_EN_CLK2(x)              no_os_field_prep(ADF4377_EN_CLK2_MSK, x)
#define ADF4377_EN_CLK1_MSK				NO_OS_BIT(6)
#define ADF4377_EN_CLK1(x)              no_os_field_prep(ADF4377_EN_CLK1_MSK, x)
#define ADF4377_REF_OK_MSK				NO_OS_BIT(3)
#define ADF4377_REF_OK(x)               no_os_field_prep(ADF4377_REF_OK_MSK, x)
#define ADF4377_ADC_BUSY_MSK			NO_OS_BIT(2)
#define ADF4377_ADC_BUSY(x)             no_os_field_prep(ADF4377_ADC_BUSY_MSK, x)
#define ADF4377_FSM_BUSY_MSK			NO_OS_BIT(1)
#define ADF4377_FSM_BUSY(x)             no_os_field_prep(ADF4377_FSM_BUSY_MSK, x)
#define ADF4377_LOCKED_MSK				NO_OS_BIT(0)
#define ADF4377_LOCKED(x)               no_os_field_prep(ADF4377_LOCKED_MSK, x)

/* ADF4377 REG004B Map */
#define ADF4377_VCO_CORE_MSK			NO_OS_GENMASK(1, 0)
#define ADF4377_VCO_CORE(x)             no_os_field_prep(ADF4377_VCO_CORE_MSK, x)

/* ADF4377 REG004C Map */
#define ADF4377_CHIP_TEMP_LSB_MSK		NO_OS_GENMASK(7, 0)
#define ADF4377_CHIP_TEMP_LSB(x)        no_os_field_prep(ADF4377_CHIP_TEMP_LSB_MSK, x)

/* ADF4377 REG004D Map */
#define ADF4377_CHIP_TEMP_MSB_MSK		NO_OS_BIT(0)
#define ADF4377_CHIP_TEMP_MSB(x)        no_os_field_prep(ADF4377_CHIP_TEMP_MSB_MSK, x)

/* ADF4377 REG004F Map */
#define ADF4377_VCO_BAND_MSK			NO_OS_GENMASK(7, 0)
#define ADF4377_VCO_BAND(x)             no_os_field_prep(ADF4377_VCO_BAND_MSK, x)

/* ADF4377 REG0054 Map */
#define ADF4377_CHIP_VERSION_MSK		NO_OS_GENMASK(7, 0)
#define ADF4377_CHIP_VERSION(x)         no_os_field_prep(ADF4377_CHIP_VERSION_MSK, x)

/* Specifications */
#define ADF4377_SPI_WRITE_CMD		    0x0
#define ADF4377_SPI_READ_CMD		    NO_OS_BIT(7)
#define ADF4377_BUFF_SIZE_BYTES		    3
#define ADF4377_MAX_VCO_FREQ		    12800000000ull /* Hz */
#define ADF4377_MIN_VCO_FREQ		    6400000000ull /* Hz */
#define ADF4377_MAX_REFIN_FREQ		    1000000000 /* Hz */
#define ADF4377_MIN_REFIN_FREQ		    10000000 /* Hz */
#define ADF4377_MAX_FREQ_PFD		    500000000 /* Hz */
#define ADF4377_MIN_FREQ_PFD		    3000000 /* Hz */
#define ADF4377_MAX_CLKPN_FREQ		    ADF4377_MAX_VCO_FREQ /* Hz */
#define ADF4377_MIN_CLKPN_FREQ		    (ADF4377_MIN_VCO_FREQ / 8) /* Hz */
#define ADF4377_FREQ_PFD_80MHZ		    80000000
#define ADF4377_FREQ_PFD_125MHZ		    125000000
#define ADF4377_FREQ_PFD_160MHZ		    160000000
#define ADF4377_FREQ_PFD_250MHZ		    250000000
#define ADF4377_FREQ_PFD_320MHZ		    320000000

/* ADF4377 Extra Definitions */
#define ADF4377_SPI_SCRATCHPAD_TEST_A	    0xA5u
#define ADF4377_SPI_SCRATCHPAD_TEST_B	    0x5Au
#define ADF4377_SPI_DUMMY_DATA		    0x00
#define ADF4377_CHECK_RANGE(freq, range) \
	((freq > ADF4377_MAX_ ## range) || (freq < ADF4377_MIN_ ## range))

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adf4377_dev_id` enumeration defines identifiers for devices
 * supported by the ADF4377 driver, specifically the ADF4377 and ADF4378
 * devices. Each enumerator is associated with a unique hexadecimal value
 * that serves as an identifier for the respective device, facilitating
 * device-specific operations within the driver.
 *
 * @param ADF4377 Represents the device ID for the ADF4377 device, with a value
 * of 0x05.
 * @param ADF4378 Represents the device ID for the ADF4378 device, with a value
 * of 0x06.
 ******************************************************************************/
enum adf4377_dev_id {
	ADF4377 = 0x05,
	ADF4378 = 0x06
};

/***************************************************************************//**
 * @brief The `adf4377_init_param` structure is used to initialize the ADF4377
 * device, containing various configuration parameters such as SPI and
 * GPIO initialization settings, device ID, and frequency settings. It
 * includes fields for setting the input reference clock frequency,
 * output frequency, charge pump current, and other operational
 * parameters necessary for configuring the ADF4377 synthesizer.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_ce_param Pointer to GPIO Chip Enable parameters.
 * @param gpio_enclk1_param Pointer to GPIO ENCLK1 parameters.
 * @param gpio_enclk2_param Pointer to GPIO ENCLK2 parameters.
 * @param dev_id Device ID of the ADF4377.
 * @param spi4wire Boolean indicating if SPI 3-wire mode is used.
 * @param clkin_freq Frequency of the input reference clock in Hz.
 * @param f_clk Output frequency in Hz.
 * @param cp_i Charge pump current setting.
 * @param muxout_select MUXOUT select setting.
 * @param ref_doubler_en Reference doubler enable flag.
 * @param clkout_op Output amplitude setting.
 ******************************************************************************/
struct adf4377_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/** GPIO Chip Enable */
	struct no_os_gpio_init_param	*gpio_ce_param;
	/** GPIO ENCLK1 */
	struct no_os_gpio_init_param	*gpio_enclk1_param;
	/** GPIO ENCLK2 */
	struct no_os_gpio_init_param	*gpio_enclk2_param;
	/** Device ID */
	enum adf4377_dev_id dev_id;
	/** SPI 3-Wire */
	bool spi4wire;
	/** Input Reference Clock */
	uint32_t clkin_freq;
	/** Output frequency */
	uint64_t f_clk;
	/** Charge Pump Current */
	uint8_t cp_i;
	/** MUXOUT Select */
	uint32_t muxout_select;
	/** Reference doubler enable */
	uint8_t ref_doubler_en;
	/** Output Amplitude */
	uint8_t	clkout_op;
};

/***************************************************************************//**
 * @brief The `adf4377_dev` structure is a comprehensive descriptor for the
 * ADF4377 device, encapsulating all necessary parameters and
 * configurations required for its operation. It includes pointers to SPI
 * and GPIO descriptors for communication and control, as well as various
 * configuration settings such as device ID, SPI mode, frequency
 * settings, and output configurations. This structure is essential for
 * initializing and managing the ADF4377 device, allowing for precise
 * control over its frequency synthesis capabilities.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_enclk1 Pointer to the GPIO descriptor for ENCLK1 control.
 * @param gpio_enclk2 Pointer to the GPIO descriptor for ENCLK2 control.
 * @param gpio_ce Pointer to the GPIO descriptor for chip enable control.
 * @param dev_id Enum representing the device ID.
 * @param spi4wire Boolean indicating if SPI is in 4-wire mode.
 * @param f_pfd Phase Frequency Detector frequency in Hz.
 * @param f_clk Output frequency in Hz.
 * @param f_vco Output frequency of the VCO in Hz.
 * @param clkin_freq Input reference clock frequency in Hz.
 * @param cp_i Charge pump current setting.
 * @param muxout_default Default MUXOUT setting.
 * @param ref_doubler_en Flag to enable reference doubler.
 * @param ref_div_factor Reference divider factor.
 * @param clkout_div_sel CLKOUT divider selection.
 * @param n_int Feedback divider integer value.
 * @param clkout_op Output amplitude setting.
 ******************************************************************************/
struct adf4377_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** GPIO ENCLK1 */
	struct no_os_gpio_desc	*gpio_enclk1;
	/** GPIO ENCLK2 */
	struct no_os_gpio_desc	*gpio_enclk2;
	/** GPIO Chip Enable */
	struct no_os_gpio_desc	*gpio_ce;
	/** Device ID */
	enum adf4377_dev_id dev_id;
	/** SPI 3-Wire */
	bool spi4wire;
	/** PFD Frequency */
	uint32_t f_pfd;
	/** Output frequency */
	uint64_t f_clk;
	/** Output frequency of the VCO */
	uint64_t f_vco;
	/** Input Reference Clock */
	uint32_t clkin_freq;
	/** Charge Pump Current */
	uint8_t cp_i;
	/** MUXOUT Default */
	uint8_t muxout_default;
	/** Reference doubler enable */
	uint8_t	ref_doubler_en;
	/** Reference Divider */
	uint32_t ref_div_factor;
	/** CLKOUT Divider */
	uint8_t clkout_div_sel;
	/** Feedback Divider (N) */
	uint16_t n_int;
	/** Output Amplitude */
	uint8_t	clkout_op;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to write a byte of data to a specific register of
 * the ADF4377 device. It requires a valid device descriptor and the
 * register address to which the data should be written. This function is
 * typically used to configure the device by setting various control
 * registers. Ensure that the device has been properly initialized before
 * calling this function. The function handles the bit order based on the
 * device's SPI configuration and returns an error code if the SPI
 * communication fails.
 *
 * @param dev A pointer to an adf4377_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to write to. It is an 8-bit value
 * representing the register address within the device.
 * @param data The data byte to be written to the specified register. It is an
 * 8-bit value.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * write operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t adf4377_spi_write(struct adf4377_dev *dev, uint8_t reg_addr,
			  uint8_t data);

/* ADF4377 Register Update */
/***************************************************************************//**
 * @brief Use this function to update specific bits in a register of the ADF4377
 * device by applying a mask and writing the modified data via SPI. This
 * function is useful when only certain bits of a register need to be
 * changed without affecting the other bits. It reads the current value
 * of the register, applies the mask to clear the bits, and then sets the
 * new data before writing it back. Ensure that the device is properly
 * initialized and the SPI interface is configured before calling this
 * function.
 *
 * @param dev Pointer to an initialized adf4377_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be written to. Must be a valid
 * register address for the ADF4377 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the register, after applying the
 * mask. Only the bits corresponding to the mask will be updated.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the SPI read or write
 * operations.
 ******************************************************************************/
int32_t adf4377_spi_write_mask(struct adf4377_dev *dev, uint8_t reg_addr,
			       uint8_t mask, uint8_t data);

/***************************************************************************//**
 * @brief Use this function to read a specific register from the ADF4377 device
 * using SPI communication. It requires a valid device descriptor and a
 * register address to read from. The function will store the read value
 * in the provided data pointer. Ensure that the device has been properly
 * initialized and the SPI interface is correctly configured before
 * calling this function. The function returns an error code if the SPI
 * communication fails.
 *
 * @param dev A pointer to an adf4377_dev structure representing the device.
 * Must not be null and should be properly initialized.
 * @param reg_addr The address of the register to read from. It is an 8-bit
 * unsigned integer.
 * @param data A pointer to an 8-bit unsigned integer where the read data will
 * be stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or a negative error
 * code if the SPI read operation fails.
 ******************************************************************************/
int32_t adf4377_spi_read(struct adf4377_dev *dev, uint8_t reg_addr,
			 uint8_t *data);

/* Software Reset */
/***************************************************************************//**
 * @brief This function is used to perform a software reset on the ADF4377
 * device, which is necessary to restore the device to its default state.
 * It should be called when a reset of the device is required, such as
 * after initialization or when the device is not functioning as
 * expected. The function attempts to reset the device by writing to the
 * appropriate register and then verifies the reset by reading back the
 * register value. If the reset is successful, the device is set to its
 * default configuration. The function returns an error code if the reset
 * fails or if there is a communication error with the device.
 *
 * @param dev A pointer to an adf4377_dev structure representing the device.
 * Must not be null. The caller retains ownership of the memory.
 * @return Returns 0 on success, a negative error code on failure, or -1 if the
 * reset operation times out.
 ******************************************************************************/
int32_t adf4377_soft_reset(struct adf4377_dev *dev);

/* ADF4377 Scratchpad check */
/***************************************************************************//**
 * @brief This function is used to verify the communication with the ADF4377
 * device by performing a scratchpad test. It writes a test value to a
 * specific register and reads it back to ensure the device is responding
 * correctly. This function should be called to confirm device
 * functionality after initialization. It returns an error code if the
 * test fails, indicating a potential communication issue.
 *
 * @param dev A pointer to an adf4377_dev structure representing the device.
 * Must not be null. The caller retains ownership and is responsible
 * for ensuring the device is properly initialized before calling
 * this function.
 * @return Returns 0 on success, a negative error code if a SPI communication
 * error occurs, or -1 if the scratchpad test fails.
 ******************************************************************************/
int32_t adf4377_check_scratchpad(struct adf4377_dev *dev);

/* Set Output frequency */
/***************************************************************************//**
 * @brief This function configures the ADF4377 device to output a specified
 * frequency. It should be called after the device has been properly
 * initialized. The function adjusts internal dividers to achieve the
 * desired frequency and waits for the VCO calibration to complete. If
 * the specified frequency is outside the allowable range, the function
 * will return an error. The function also updates the device's internal
 * state to reflect the new frequency setting.
 *
 * @param dev A pointer to an initialized adf4377_dev structure representing the
 * device. Must not be null.
 * @param freq The desired output frequency in Hz. Must be within the device's
 * supported frequency range.
 * @return Returns 0 on success, a negative error code on failure, or -ETIMEDOUT
 * if the VCO calibration times out.
 ******************************************************************************/
int32_t adf4377_set_freq(struct adf4377_dev *dev, uint64_t freq);

/***************************************************************************//**
 * @brief This function initializes an ADF4377 device using the provided
 * initialization parameters. It sets up the necessary SPI and GPIO
 * configurations, performs a software reset, and verifies the device
 * identity. This function must be called before any other operations on
 * the ADF4377 device. If initialization fails at any step, it cleans up
 * allocated resources and returns an error code. The caller must ensure
 * that the `init_param` structure is properly populated and that the
 * `device` pointer is valid.
 *
 * @param device A pointer to a pointer of type `struct adf4377_dev`. This will
 * be allocated and initialized by the function. The caller must
 * provide a valid pointer to a pointer, and the function will set
 * it to point to the initialized device structure.
 * @param init_param A pointer to a `struct adf4377_init_param` containing the
 * initialization parameters. This structure must be fully
 * populated with valid data before calling the function. The
 * caller retains ownership of this structure, and it must not
 * be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or device mismatch.
 ******************************************************************************/
int32_t adf4377_init(struct adf4377_dev **device,
		     struct adf4377_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to safely release all resources allocated for an
 * ADF4377 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that all associated SPI and GPIO resources are properly
 * freed. It is important to call this function to prevent resource leaks
 * in the system.
 *
 * @param dev A pointer to an adf4377_dev structure representing the device to
 * be removed. Must not be null. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns 0 on success or a negative error code if any of the resource
 * deallocation steps fail.
 ******************************************************************************/
int32_t adf4377_remove(struct adf4377_dev *dev);

#endif /* ADF4377_H_ */
