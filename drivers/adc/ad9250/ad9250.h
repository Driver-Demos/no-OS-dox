/***************************************************************************//**
 *   @file   ad9250.h
 *   @brief  Header file of AD9250 Driver.
 *   @author DNechita (Dan.Nechita@analog.com)
 ********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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
#ifndef __AD9250_H__
#define __AD9250_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/*********************************** AD9250 ***********************************/
/******************************************************************************/

/* Registers */

#define AD9250_READ				(1 << 15)
#define AD9250_WRITE				(0 << 15)
#define AD9250_CNT(x)				((((x) & 0x3) - 1) << 13)
#define AD9250_ADDR(x)				((x) & 0xFF)

#define AD9250_R1B				(1 << 8)
#define AD9250_R2B				(2 << 8)
#define AD9250_R3B				(3 << 8)
#define AD9250_TRANSF_LEN(x)			(((x) >> 8) & 0xFF)
#define SHADOW(x)				((x) << 16)

/* Chip configuration registers */
#define AD9250_REG_SPI_CFG			(AD9250_R1B | 0x00)
#define AD9250_REG_CHIP_ID			(AD9250_R1B | 0x01)
#define AD9250_REG_CHIP_INFO			(AD9250_R1B | 0x02)

/* Channel index and transfer registers */
#define AD9250_REG_CH_INDEX			(AD9250_R1B | 0x05)
#define AD9250_REG_DEVICE_UPDATE		(AD9250_R1B | 0xFF)

/* Program register map */
#define AD9250_REG_PDWN				(AD9250_R1B | 0x08)
#define AD9250_REG_CLOCK			(AD9250_R1B | 0x09 | SHADOW(1))
#define AD9250_REG_PLL_STAT			(AD9250_R1B | 0x0A)
#define AD9250_REG_CLOCK_DIV			(AD9250_R1B | 0x0B | SHADOW(2))
#define AD9250_REG_TEST				(AD9250_R1B | 0x0D | SHADOW(3))
#define AD9250_REG_BIST				(AD9250_R1B | 0x0E | SHADOW(4))
#define AD9250_REG_OFFSET			(AD9250_R1B | 0x10 | SHADOW(5))
#define AD9250_REG_OUT_MODE			(AD9250_R1B | 0x14 | SHADOW(6))
#define AD9250_REG_CML				(AD9250_R1B | 0x15)
#define AD9250_REG_VREF				(AD9250_R1B | 0x18 | SHADOW(7))
#define AD9250_REG_USER_TEST1			(AD9250_R2B | 0x1A)
#define AD9250_REG_USER_TEST2			(AD9250_R2B | 0x1C)
#define AD9250_REG_USER_TEST3			(AD9250_R2B | 0x1E)
#define AD9250_REG_USER_TEST4			(AD9250_R2B | 0x20)
#define AD9250_REG_PLL_ENCODE			(AD9250_R1B | 0x21)
#define AD9250_REG_BIST_MISR			(AD9250_R2B | 0x25)
#define AD9250_REG_SYS_CTRL			(AD9250_R1B | 0x3A | SHADOW(8))
#define AD9250_REG_DCC_CTRL			(AD9250_R1B | 0x40 | SHADOW(9))
#define AD9250_REG_DCC_VAL			(AD9250_R2B | 0x42 | SHADOW(10))
#define AD9250_REG_FAST_DETECT			(AD9250_R1B | 0x45 | SHADOW(11))
#define AD9250_REG_FD_UPPER_THD			(AD9250_R2B | 0x48 | SHADOW(12))
#define AD9250_REG_FD_LOWER_THD			(AD9250_R2B | 0x4A | SHADOW(13))
#define AD9250_REG_FD_DWELL_TIME		(AD9250_R2B | 0x4C | SHADOW(14))
#define AD9250_REG_204B_QUICK_CFG		(AD9250_R1B | 0x5E)
#define AD9250_REG_204B_CTRL1			(AD9250_R1B | 0x5F)
#define AD9250_REG_204B_CTRL2			(AD9250_R1B | 0x60)
#define AD9250_REG_204B_CTRL3			(AD9250_R1B | 0x61)
#define AD9250_REG_204B_DID_CFG			(AD9250_R1B | 0x64)
#define AD9250_REG_204B_BID_CFG			(AD9250_R1B | 0x65)
#define AD9250_REG_204B_LID_CFG0		(AD9250_R1B | 0x66)
#define AD9250_REG_204B_LID_CFG1		(AD9250_R1B | 0x67)
#define AD9250_REG_204B_PARAM_SCR_L		(AD9250_R1B | 0x6E)
#define AD9250_REG_204B_PARAM_F			(AD9250_R1B | 0x6F)
#define AD9250_REG_204B_PARAM_K			(AD9250_R1B | 0x70)
#define AD9250_REG_204B_PARAM_M			(AD9250_R1B | 0x71)
#define AD9250_REG_204B_PARAM_CS_N		(AD9250_R1B | 0x72)
#define AD9250_REG_204B_PARAM_NP		(AD9250_R1B | 0x73)
#define AD9250_REG_204B_PARAM_S			(AD9250_R1B | 0x74)
#define AD9250_REG_204B_PARAM_HD_CF		(AD9250_R1B | 0x75)
#define AD9250_REG_204B_RESV1			(AD9250_R1B | 0x76)
#define AD9250_REG_204B_RESV2			(AD9250_R1B | 0x77)
#define AD9250_REG_204B_CHKSUM0			(AD9250_R1B | 0x79)
#define AD9250_REG_204B_CHKSUM1			(AD9250_R1B | 0x7A)
#define AD9250_REG_204B_LANE_ASSGN1		(AD9250_R1B | 0x82)
#define AD9250_REG_204B_LANE_ASSGN2		(AD9250_R1B | 0x83)
#define AD9250_REG_204B_LMFC_OFFSET		(AD9250_R1B | 0x8B)
#define AD9250_REG_204B_PRE_EMPHASIS		(AD9250_R1B | 0xA8)

/* AD9250_REG_SPI_CFG */
#define AD9250_SPI_CFG_LSB_FIRST		((1 << 6) | (1 << 1))
#define AD9250_SPI_CFG_SOFT_RST			((1 << 5) | (1 << 2))

/* AD9250_REG_CH_INDEX */
#define AD9250_CH_INDEX_ADC_A			(1 << 0)
#define AD9250_CH_INDEX_ADC_B			(1 << 1)

/* AD9250_REG_DEVICE_UPDATE */
#define AD9250_DEVICE_UPDATE_SW			(1 << 0)

/* AD9250_REG_PDWN */
#define AD9250_PDWN_EXTERN			(1 << 5)
#define AD9250_PDWN_JTX				(1 << 4)
#define AD9250_PDWN_JESD204B(x)			(((x) & 0x3) << 2)
#define AD9250_PDWN_CHIP(x)			(((x) & 0x3) << 0)

/* AD9250_REG_CLOCK */
#define AD9250_CLOCK_SELECTION(x)		(((x) & 0x3) << 4)
#define AD9250_CLOCK_DUTY_CYCLE			(1 << 0)

/* AD9250_REG_PLL_STAT */
#define AD9250_PLL_STAT_LOCKED			(1 << 7)
#define AD9250_PLL_STAT_204B_LINK_RDY		(1 << 0)

/* AD9250_REG_CLOCK_DIV */
#define AD9250_CLOCK_DIV_PHASE(x)		(((x) & 0x7) << 3)
#define AD9250_CLOCK_DIV_RATIO(x)		(((x) & 0x7) << 0)

/* AD9250_REG_TEST */
#define AD9250_TEST_USER_TEST_MODE(x)		(((x) & 0x3) << 6)
#define AD9250_TEST_RST_PN_LONG			(1 << 5)
#define AD9250_TEST_RST_PN_SHOR			(1 << 4)
#define AD9250_TEST_OUTPUT_TEST(x)		(((x) & 0xF) << 0)

/* AD9250_TEST */
#define AD9250_TEST_OFF				0x00
#define AD9250_TEST_MID_SCALE			0x01
#define AD9250_TEST_POS_FSCALE			0x02
#define AD9250_TEST_NEG_FSCALE			0x03
#define AD9250_TEST_CHECKBOARD			0x04
#define AD9250_TEST_PNLONG			0x05
#define AD9250_TEST_ONE2ZERO			0x07
#define AD9250_TEST_PATTERN			0x08
#define AD9250_TEST_RAMP			0x0F

/* AD9250_REG_BIST */
#define AD9250_BIST_RESET			(1 << 2)
#define AD9250_BIST_ENABLE			(1 << 0)

/* AD9250_REG_OFFSET */
#define AD9250_REG_OFFSET_ADJUST(x)		(((x) & 0x3F) << 0)

/* AD9250_REG_OUT_MODE */
#define AD9250_OUT_MODE_JTX_BIT_ASSIGN(x)	(((x) & 0x7) << 5)
#define AD9250_OUT_MODE_DISABLE			(1 << 4)
#define AD9250_OUT_MODE_INVERT_DATA		(1 << 3)
#define AD9250_OUT_MODE_DATA_FORMAT(x)		(((x) & 0x1) << 0)

/* AD9250_OUT */
#define AD9250_OUT_OFFSET_BINARY		0x00
#define AD9250_OUT_2S_COMPLEMENT		0x01

/* AD9250_REG_CML */
#define AD9250_CML_DIFF_OUT_LEVEL(x)		(((x) & 0x7) << 0)

/* AD9250_REG_VREF */
#define AD9250_VREF_FS_ADJUST(x)		(((x) & 0x1F) << 0)

/* AD9250_REG_PLL_ENCODE */
#define AD9250_PLL_ENCODE(x)			(((x) & 0x3) << 3)

/* AD9250_REG_SYS_CTRL */
#define AD9250_SYS_CTRL_REALIGN_ON_SYNCINB	(1 << 4)
#define AD9250_SYS_CTRL_REALIGN_ON_SYSREF	(1 << 3)
#define AD9250_SYS_CTRL_SYSREF_MODE		(1 << 2)
#define AD9250_SYS_CTRL_SYSREF_EN		(1 << 1)
#define AD9250_SYS_CTRL_SYNCINB_EN		(1 << 0)

/* AD9250_REG_DCC_CTRL */
#define AD9250_DCC_CTRL_FREEZE_DCC		(1 << 6)
#define AD9250_DCC_CTRL_DCC_BW(x)		(((x) & 0xF) << 2)
#define AD9250_DCC_CTRL_DCC_EN			(1 << 1)

/* AD9250_REG_FAST_DETECT */
#define AD9250_FAST_DETECT_PIN_FCT		(1 << 4)
#define AD9250_FAST_DETECT_FORCE_FDA_FDB_PIN	(1 << 3)
#define AD9250_FAST_DETECT_FORCE_FDA_FDB_VAL	(1 << 2)
#define AD9250_FAST_DETECT_OUTPUT_ENABLE	(1 << 0)

/* AD9250_REG_204B_QUICK_CFG */
#define AD9250_204B_QUICK_CFG(x)		(((x) & 0xFF) << 0)

/* AD9250_REG_204B_CTRL1 */
#define AD9250_204B_CTRL1_TAIL_BITS		(1 << 6)
#define AD9250_204B_CTRL1_TEST_SAMPLE_EN	(1 << 5)
#define AD9250_204B_CTRL1_ILAS_MODE(x)		(((x) & 0x3) << 2)
#define AD9250_204B_CTRL1_POWER_DOWN		(1 << 0)

/* AD9250_REG_204B_CTRL2 */
#define AD9250_204B_CTRL2_INVERT_JESD_BITS	(1 << 1)

/* AD9250_REG_204B_CTRL3 */
#define AD9250_204B_CTRL3_TEST_DATA_INJ_PT(x)	(((x) & 0x3) << 4)
#define AD9250_204B_CTRL3_JESD_TEST_MODE(x)	(((x) & 0xF) << 0)

/* AD9250_REG_204B_PARAM_SCR_L */
#define AD9250_204B_PARAM_SCR_L_SCRAMBLING	(1 << 7)
#define AD9250_204B_PARAM_SCR_L_LANES		(1 << 0)

/* AD9250_REG_204B_PARAM_CS_N */
#define AD9250_204B_PARAM_CS_N_NR_CTRL_BITS(x)	(((x) & 0x3) << 6)
#define AD9250_204B_PARAM_CS_N_ADC_RESOLUTION(x)	(((x) & 0xF) << 0)

/* AD9250_REG_204B_PARAM_NP */
#define AD9250_204B_PARAM_NP_JESD_SUBCLASS(x)	(((x) & 0x3) << 5)
#define AD9250_204B_PARAM_NP_JESD_N_VAL(x)	(((x) & 0xF) << 0)

/* AD9250_REG_204B_PARAM_S */
#define AD9250_204B_PARAM_S(x)			(((x) << 0x1F) << 0)

/* AD9250_REG_204B_PARAM_HD_CF */
#define AD9250_204B_PARAM_HD_CF_HD_VAL		(1 << 7)
#define AD9250_204B_PARAM_HD_CF_CF_VAL(x)	(((x) & 0x1F) << 0)

/* AD9250_REG_204B_LANE_ASSGN1 */
#define AD9250_204B_LANE_ASSGN1(x)		(((x) & 0x3) << 4)

/* AD9250_REG_204B_LANE_ASSGN2 */
#define AD9250_204B_LANE_ASSGN2(x)		(((x) &0x3) << 0)

/* AD9250_REG_204B_LMFC_OFFSET */
#define AD9250_204B_LMFC_OFFSET(x)		(((x) & 0x1F) << 0)

/*****************************************************************************/
/************************** Types Declarations *******************************/
/*****************************************************************************/

/***************************************************************************//**
 * @brief The `ad9250_platform_data` structure is used to configure platform-
 * specific settings for the AD9250 device, including power-down modes,
 * clock settings, and voltage reference adjustments. It provides fields
 * to control the external power-down mode, enable or disable the clock
 * duty cycle stabilizer, select the clock source, and set the clock
 * divider ratio and phase. Additionally, it allows for the adjustment of
 * the ADC's full-scale voltage reference and specifies the PLL low
 * encode mode based on lane speeds. The structure also includes a field
 * for storing the device name.
 *
 * @param extrn_pdwnmode Specifies the external power-down mode of the device.
 * @param en_clk_dcs Enables or disables the clock duty cycle stabilizer.
 * @param clk_selection Determines the clock source selection for the device.
 * @param clk_div_ratio Sets the clock divider ratio relative to the encode
 * clock.
 * @param clk_div_phase Specifies the clock divide phase relative to the encode
 * clock.
 * @param adc_vref Adjusts the main reference full-scale voltage reference
 * (VREF).
 * @param pll_low_encode Indicates the PLL low encode mode for lane speeds.
 * @param name Stores the device name as a string of up to 16 characters.
 ******************************************************************************/
struct ad9250_platform_data {
	/**
	 * External PDWN mode.
	 * 0 = PDWN is full power down
	 * 1 = PDWN puts device in standby
	 */
	int8_t extrn_pdwnmode;
	/**
	 * Clock duty cycle stabilizer enable.
	 * 0 = disable
	 * 1 = enable
	 */
	int8_t en_clk_dcs;
	/**
	 * Clock selection.
	 * 0 = Nyquist clock
	 * 2 = RF clock divide by 4
	 * 3 = clock off
	 */
	int8_t clk_selection;
	/**
	 * Clock divider ratio relative to the encode clock.
	 * 0x00 = divide by 1
	 * 0x01 = divide by 2
	 * ...
	 * 0x07 = divide by 8
	 */
	int8_t clk_div_ratio;
	/**
	 * Clock divide phase relative to the encode clock.
	 * 0x0 = 0 input clock cycles delayed
	 * 0x1 = 1 input clock cycles delayed
	 * ...
	 * 0x7 = 7 input clock cycles delayed
	 */
	int8_t clk_div_phase;
	/**
	 * Main reference full-scale VREF adjustment.
	 * 0x0f = internal 2.087 V p-p
	 * ...
	 * 0x01 = internal 1.772 V p-p
	 * 0x00 = internal 1.75 V p-p [default]
	 * 0x1F = internal 1.727 V p-p
	 * ...
	 * 0x10 = internal 1.383 V p-p
	 */
	int8_t adc_vref;
	/**
	 * PLL low encode.
	 * 0 = for lane speeds > 2 Gbps
	 * 1 = for lane speeds < 2 Gbps
	 */
	int8_t pll_low_encode;
	/** Device name */
	int8_t	name[16];
};

/***************************************************************************//**
 * @brief The `ad9250_jesd204b_cfg` structure is used to configure the JESD204B
 * interface for the AD9250 device. It includes various settings such as
 * standby mode, CML differential output levels, quick configuration
 * options, subclass selection, control bit settings, and identification
 * values for device, bank, and lanes. Additionally, it manages frame
 * settings, scrambling, initial lane alignment sequences, and logic
 * inversion. The structure also provides options for enabling and
 * configuring SYSREF and SYNCINB signals, as well as remapping lane
 * assignments. This configuration is crucial for ensuring proper
 * communication and data transfer over the JESD204B interface.
 *
 * @param jtx_in_standby Indicates whether the JESD204B core is in standby mode.
 * @param cml_level Adjusts the JESD204B CML differential output drive level.
 * @param quick_cfg_option Specifies a quick configuration option for the
 * JESD204B interface.
 * @param subclass Defines the JESD204B subclass used.
 * @param ctrl_bits_no Specifies the number of control bits in the JESD204B
 * interface.
 * @param ctrl_bits_assign Determines the assignment of control bits in the
 * JESD204B interface.
 * @param tail_bits_mode Specifies the mode for tail bits when control bits are
 * not enabled.
 * @param did Device identification value for the JESD204B interface.
 * @param bid Bank identification value for the JESD204B interface.
 * @param lid0 Lane 0 identification value for the JESD204B interface.
 * @param lid1 Lane 1 identification value for the JESD204B interface.
 * @param k Number of frames per multiframe in the JESD204B interface.
 * @param scrambling Indicates whether scrambling is enabled in the JESD204B
 * interface.
 * @param ilas_mode Specifies the initial lane alignment sequence mode.
 * @param en_ilas_test Indicates whether the ILAS test sample is enabled.
 * @param invert_logic_bits Determines if the logic of JESD204B bits is
 * inverted.
 * @param en_sys_ref Indicates if SYSREF+- is enabled.
 * @param en_sync_in_b Indicates if the SYNCINB+- buffer is enabled.
 * @param sys_ref_mode Specifies the mode for SYSREF+-.
 * @param align_sync_in_b Options for interpreting signals on SYNCINB+-.
 * @param align_sys_ref Options for interpreting signals on SYSREF+-.
 * @param lane0_assign Option to remap converter and lane assignments for
 * Logical Lane 0.
 * @param lane1_assign Option to remap converter and lane assignments for
 * Logical Lane 1.
 ******************************************************************************/
struct ad9250_jesd204b_cfg {
	/**
	 * JTX in standby.
	 * 0 = 204B core is unaffected in standby
	 * 1 = 204B core is powered down except for PLL during standby
	 */
	int8_t jtx_in_standby;
	/**
	 * JESD204B CML differential output drive level adjustment.
	 * 0 = 81% of nominal (that is, 238 mV)
	 * 1 = 89% of nominal (that is, 262 mV)
	 * 2 = 98% of nominal (that is, 286 mV)
	 * 3 = nominal [default] (that is, 293 mV)
	 * 6 = 126% of nominal (that is, 368 mV)
	 */
	int8_t cml_level;
	/**
	 * Quick configuration register.
	 * 0x11 = M = 1, L = 1; one converter, one lane
	 * 0x12 = M = 1, L = 2; one converter, two lanes
	 * 0x21 = M = 2, L = 1; two converters, one lane
	 * 0x22 = M = 2, L = 2; two converters, two lanes
	 */
	int8_t quick_cfg_option;
	/**
	 * JESD204B subclass.
	 * 0 = Subclass 0
	 * 1 = Subclass 1
	 */
	int8_t subclass;
	/**
	 * Number of control bits (CS).
	 * 0 = no control bits(CS = 0)
	 * 1 = 1 control bit  (CS = 1)
	 * 2 = 2 control bits (CS = 2)
	 */
	int8_t ctrl_bits_no;
	/**
	 * JTX CS bits assignment.
	 * 0 = {overrange||underrange, valid}
	 * 1 = {overrange||underrange}
	 * 2 = {overrange||underrange, blank}
	 * 3 = {blank, valid}
	 * 4 = {blank, blank}
	 * All others = {overrange||underrange, valid}
	 */
	int8_t ctrl_bits_assign;
	/**
	 * Tail bits: If CS bits are not enabled.
	 * 0 = extra bits are 0
	 * 1 = extra bits are 9-bit PN
	 */
	int8_t tail_bits_mode;
	/** JESD204B device identification value: DID[7:0] */
	int8_t did;
	/** JESD204B bank identification value : BID[3:0] */
	int8_t bid;
	/** JESD204B lane0 identification value: LID[4:0] */
	int8_t lid0;
	/** JESD204B lane1 identification value: LID[4:0] */
	int8_t lid1;
	/**
	 * JESD204B number of frames per multiframe (K); set value of K per JESD204B
	 * specifications, but also must be a multiple of 4 octets.
	 */
	int8_t k;
	/**
	 * JESD204B scrambling (SCR).
	 * 0 = disabled
	 * 1 = enabled
	 */
	int8_t scrambling;
	/**
	 * Initial lane alignment sequence (ILAS) mode.
	 * 1 = ILAS normal mode enabled
	 * 3 = ILAS always on, test mode
	 */
	int8_t ilas_mode;
	/**
	 * JESD204B test sample.
	 * 0 = disabled
	 * 1 = enabled
	 */
	int8_t en_ilas_test;
	/**
	 * Invert logic of JESD204B bits.
	 * 0 = non-invert
	 * 1 = invert
	 */
	int8_t invert_logic_bits;
	/**
	 * SYSREF+- enable.
	 * 0 = disabled
	 * 1 = enabled
	 */
	int8_t en_sys_ref;
	/**
	 * Enable SYNCINB+- buffer.
	 * 0 = buffer disabled
	 * 1 = buffer enabled
	 */
	int8_t en_sync_in_b;
	/**
	 * SYSREF+- mode.
	 * 0 = continuous reset clock dividers
	 * 1 = sync on next SYSREF+- rising edge only
	 */
	int8_t sys_ref_mode;
	/**
	 * Options for interpreting single on SYNCINB+-.
	 * 0 = normal mode
	 * 1 = realign lanes on every active SYNCINB+-
	 */
	int8_t align_sync_in_b;
	/**
	 * Options for interpreting single on SYSREF+-.
	 * 0 = normal mode;
	 * 1 = realign lanes on every active SYSREF+-
	 */
	int8_t align_sys_ref;
	/**
	 * Option to remap converter and lane assignments.
	 * 0 = assign Logical Lane 0 to Physical Lane A [default]
	 * 1 = assign Logical Lane 0 to Physical Lane B
	 */
	int8_t lane0_assign;
	/* Option to remap converter and lane assignments.
	 * 0 = assign Logical Lane 1 to Physical Lane A
	 * 1 = assign Logical Lane 1 to Physical Lane B [default]
	*/
	int8_t lane1_assign;
};

/***************************************************************************//**
 * @brief The `ad9250_fast_detect_cfg` structure is used to configure the fast
 * detect module of the AD9250 device. It includes settings to enable or
 * disable fast detect output, configure pin functions, and force pin
 * values. Additionally, it allows setting upper and lower thresholds for
 * fast detection and specifies the dwell time for the detection process.
 * This configuration is crucial for applications requiring rapid
 * detection of signal conditions.
 *
 * @param en_fd Enables or disables the fast detect output.
 * @param pin_function Determines the function of the pin, either fast detect or
 * overrange.
 * @param force_pins Indicates whether to force the FDA/FDB pins to a specific
 * value.
 * @param pin_force_value Specifies the forced output value on the FD pins.
 * @param fd_upper_tresh Sets the upper threshold for fast detection.
 * @param fd_lower_tresh Sets the lower threshold for fast detection.
 * @param df_dwell_time Defines the dwell time for fast detection.
 ******************************************************************************/
struct ad9250_fast_detect_cfg {
	/**
	 * Enable fast detect output.
	 * 0 = disable
	 * 1 = enable
	 */
	int8_t	en_fd;
	/**
	 * Pin function.
	 * 0 = fast detect
	 * 1 = overrange
	 */
	int8_t	pin_function;
	/**
	 * Force FDA/FDB pins
	 * 0 = normal function
	 * 1 = force to value
	 */
	int8_t	force_pins;
	/**
	 * Force value of FDA/FDB pins.
	 * 0 = output on FD pins will be 0
	 * 1 = output on FD pins will be 1
	 */
	int8_t	pin_force_value;
	/** Fast Detect Upper Threshold[14:0]. */
	int16_t fd_upper_tresh;
	/** Fast Detect Lower Threshold[14:0]. */
	int16_t fd_lower_tresh;
	/** Fast Detect Dwell Time[15:0]. */
	int16_t df_dwell_time;
};

/***************************************************************************//**
 * @brief The `ad9250_state` structure is a composite data structure used to
 * encapsulate the state and configuration of an AD9250 device. It
 * contains pointers to three other structures: `ad9250_platform_data`,
 * which holds platform-specific settings; `ad9250_jesd204b_cfg`, which
 * contains configuration details for the JESD204B interface; and
 * `ad9250_fast_detect_cfg`, which manages the fast detect module
 * settings. This structure is essential for managing the device's
 * configuration and operation in a structured and organized manner.
 *
 * @param pdata Pointer to platform-specific configuration data.
 * @param p_jesd204b Pointer to JESD204B interface configuration data.
 * @param p_fd Pointer to fast detect module configuration data.
 ******************************************************************************/
struct ad9250_state {
	struct ad9250_platform_data   *pdata;
	struct ad9250_jesd204b_cfg    *p_jesd204b;
	struct ad9250_fast_detect_cfg *p_fd;
};

/***************************************************************************//**
 * @brief The `shadow_registers` enum defines a set of constants representing
 * various shadow registers used in the AD9250 device driver. Each
 * constant corresponds to a specific register that holds configuration
 * settings for different functionalities such as clock, test, BIST,
 * offset, output mode, voltage reference, system control, DC correction,
 * and fast detect. The `SHADOW_REGISTER_COUNT` is used to indicate the
 * total number of shadow registers available.
 *
 * @param AD9250_SHD_REG_CLOCK Represents the shadow register for clock
 * settings.
 * @param AD9250_SHD_REG_CLOCK_DIV Represents the shadow register for clock
 * division settings.
 * @param AD9250_SHD_REG_TEST Represents the shadow register for test settings.
 * @param AD9250_SHD_REG_BIST Represents the shadow register for Built-In Self-
 * Test settings.
 * @param AD9250_SHD_REG_OFFSET Represents the shadow register for offset
 * settings.
 * @param AD9250_SHD_REG_OUT_MODE Represents the shadow register for output mode
 * settings.
 * @param AD9250_SHD_REG_VREF Represents the shadow register for voltage
 * reference settings.
 * @param AD9250_SHD_REG_SYS_CTRL Represents the shadow register for system
 * control settings.
 * @param AD9250_SHD_REG_DCC_CTRL Represents the shadow register for DC
 * correction control settings.
 * @param AD9250_SHD_REG_DCC_VAL Represents the shadow register for DC
 * correction value settings.
 * @param AD9250_SHD_REG_FAST_DETECT Represents the shadow register for fast
 * detect settings.
 * @param AD9250_SHD_REG_FD_UPPER_THD Represents the shadow register for fast
 * detect upper threshold settings.
 * @param AD9250_SHD_REG_FD_LOWER_THD Represents the shadow register for fast
 * detect lower threshold settings.
 * @param AD9250_SHD_REG_FD_DWELL_TIME Represents the shadow register for fast
 * detect dwell time settings.
 * @param SHADOW_REGISTER_COUNT Represents the total count of shadow registers.
 ******************************************************************************/
enum shadow_registers {
	AD9250_SHD_REG_CLOCK = 1,
	AD9250_SHD_REG_CLOCK_DIV,
	AD9250_SHD_REG_TEST,
	AD9250_SHD_REG_BIST,
	AD9250_SHD_REG_OFFSET,
	AD9250_SHD_REG_OUT_MODE,
	AD9250_SHD_REG_VREF,
	AD9250_SHD_REG_SYS_CTRL,
	AD9250_SHD_REG_DCC_CTRL,
	AD9250_SHD_REG_DCC_VAL,
	AD9250_SHD_REG_FAST_DETECT,
	AD9250_SHD_REG_FD_UPPER_THD,
	AD9250_SHD_REG_FD_LOWER_THD,
	AD9250_SHD_REG_FD_DWELL_TIME,
	SHADOW_REGISTER_COUNT
};

/***************************************************************************//**
 * @brief The `ad9250_dev` structure is designed to encapsulate the necessary
 * components for managing and interfacing with an AD9250 device. It
 * includes a pointer to a SPI descriptor for handling SPI communication,
 * a state structure that holds the current configuration and state of
 * the AD9250, and an array for storing shadow register values, which are
 * used to maintain a copy of the device's register settings for quick
 * access and modification.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param ad9250_st Holds the state and configuration of the AD9250 device.
 * @param shadow_regs Array to store shadow register values for the device.
 ******************************************************************************/
struct ad9250_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* Device Settings */
	struct ad9250_state ad9250_st;
	int32_t shadow_regs[SHADOW_REGISTER_COUNT];
};

/***************************************************************************//**
 * @brief The `ad9250_init_param` structure is used to encapsulate the
 * initialization parameters required to set up the AD9250 device. It
 * includes SPI initialization parameters and the initial state
 * configuration for the device, allowing for a structured and organized
 * way to manage the setup process of the AD9250, which is a high-speed
 * analog-to-digital converter.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param ad9250_st_init Contains the initial state settings for the AD9250
 * device.
 ******************************************************************************/
struct ad9250_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	struct ad9250_state ad9250_st_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD9250 device by allocating necessary
 * resources, initializing the SPI interface, and configuring the device
 * registers according to the provided initialization parameters. It must
 * be called before any other operations on the AD9250 device. The
 * function handles the configuration of the JESD204B interface and the
 * Fast-detect circuit, ensuring the device is ready for operation. If
 * the setup fails at any point, the function returns an error code and
 * the device pointer is not valid.
 *
 * @param device A pointer to a pointer of type `struct ad9250_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A structure of type `struct ad9250_init_param` containing
 * initialization parameters for the AD9250 device. This
 * includes SPI initialization parameters and device-specific
 * settings. The caller retains ownership of this structure.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t ad9250_setup(struct ad9250_dev **device,
		     struct ad9250_init_param init_param);
/***************************************************************************//**
 * @brief Use this function to release all resources allocated for an AD9250
 * device when it is no longer needed. This function should be called to
 * clean up after a successful call to `ad9250_setup`. It ensures that
 * the SPI descriptor associated with the device is properly removed and
 * that the memory allocated for the device structure is freed. This
 * function returns an error code if the SPI removal fails, otherwise it
 * returns success.
 *
 * @param dev A pointer to an `ad9250_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid device structure initialized by `ad9250_setup`. The function
 * does not check for null pointers, so passing a null pointer will
 * result in undefined behavior.
 * @return Returns an integer status code from the SPI removal operation, where
 * 0 indicates success and a negative value indicates an error.
 ******************************************************************************/
int32_t ad9250_remove(struct ad9250_dev *dev);
/***************************************************************************//**
 * @brief Use this function to read the value of a specific register from the
 * AD9250 device. It is essential to ensure that the device has been
 * properly initialized and configured before calling this function. The
 * function communicates with the device over SPI to retrieve the
 * register value. If the SPI communication fails, the function returns
 * an error code. This function is typically used when you need to verify
 * or monitor the current settings of the device.
 *
 * @param dev A pointer to an initialized ad9250_dev structure representing the
 * device. Must not be null, and the device must be properly
 * configured before use.
 * @param register_address The address of the register to read. Must be a valid
 * register address for the AD9250 device. Invalid
 * addresses may result in undefined behavior or error
 * codes.
 * @return Returns the value of the specified register as a 32-bit integer. If
 * an error occurs during SPI communication, returns -1.
 ******************************************************************************/
int32_t ad9250_read(struct ad9250_dev *dev,
		    int32_t register_address);
/***************************************************************************//**
 * @brief This function is used to write a specified value to a register of the
 * AD9250 device. It should be called when there is a need to configure
 * or modify the settings of the device by writing to its registers. The
 * function handles shadow registers by synchronizing them with the on-
 * chip registers if necessary. It requires a valid device structure and
 * register address, and it returns an error code if the write operation
 * fails. This function is typically used in the initialization or
 * configuration phase of the device.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param register_address An integer representing the address of the register
 * to write to. Must be a valid register address for the
 * AD9250 device.
 * @param register_value An integer representing the value to write to the
 * specified register. The value should be within the
 * valid range for the target register.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9250_write(struct ad9250_dev *dev,
		     int32_t register_address,
		     int32_t register_value);
/***************************************************************************//**
 * @brief This function is used to initiate a transfer operation on the AD9250
 * device and waits for the operation to complete. It should be called
 * when a device update is required. The function writes to the device
 * update register and then repeatedly reads from it until the update is
 * complete or a timeout occurs. It is important to ensure that the
 * device is properly initialized before calling this function to avoid
 * unexpected behavior.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. This
 * must not be null, and the device should be properly initialized
 * before use. If the pointer is invalid, the function will return an
 * error.
 * @return Returns 0 on success, or -1 if an error occurs during the write or
 * read operations.
 ******************************************************************************/
int32_t ad9250_transfer(struct ad9250_dev *dev);
/***************************************************************************//**
 * @brief This function resets the AD9250 device to its default SPI
 * configuration values using a software reset. It should be called when
 * a reset of the device's configuration is required, such as during
 * initialization or to recover from an error state. The function
 * attempts to write a reset command to the device and then waits for the
 * reset to complete, with a timeout mechanism to prevent indefinite
 * blocking. It is important to ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device to be
 * reset. Must not be null. The caller retains ownership of the
 * memory.
 * @return Returns an int32_t value indicating the result of the operation. A
 * return value of -1 indicates an error occurred during the reset
 * process.
 ******************************************************************************/
int32_t ad9250_soft_reset(struct ad9250_dev *dev);
/***************************************************************************//**
 * @brief This function sets the power mode of the AD9250 chip to one of the
 * predefined modes or retrieves the current power mode if an invalid
 * mode is specified. It should be used to manage the power consumption
 * of the device, especially in applications where power efficiency is
 * critical. The function must be called with a valid device structure,
 * and the mode parameter should be within the acceptable range to set
 * the power mode. If the mode is outside the valid range, the function
 * returns the current power mode instead.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param mode An integer representing the desired power mode. Valid values are
 * 0, 1, or 2. If the value is outside this range, the function will
 * return the current power mode instead of setting a new one.
 * @return Returns 0 on success when setting a power mode, or the current power
 * mode if the input mode is invalid. Returns -1 if there is an error
 * reading the current power mode.
 ******************************************************************************/
int32_t ad9250_chip_pwr_mode(struct ad9250_dev *dev,
			     int32_t mode);
/***************************************************************************//**
 * @brief This function is used to select a specific channel on the AD9250
 * device for further configuration. It should be called when a
 * particular channel needs to be configured, such as setting test modes
 * or adjusting offsets. The function writes the channel index to the
 * device if the channel is valid (1 to 3). If the channel is invalid, it
 * reads the current channel configuration and returns the active
 * channels. This function must be called with a valid device structure,
 * and the channel parameter should be within the specified range to
 * avoid unexpected behavior.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param channel An integer representing the channel to select. Valid values
 * are 1, 2, or 3. If the value is outside this range, the
 * function will return the currently active channels instead.
 * @return Returns an int32_t value. If a valid channel is selected, the
 * function returns 0 on success. If the channel is invalid, it returns
 * the current active channels or -1 if an error occurs during reading.
 ******************************************************************************/
int32_t ad9250_select_channel_for_config(struct ad9250_dev *dev,
		int32_t channel);
/***************************************************************************//**
 * @brief This function is used to configure the test mode of the AD9250 ADC
 * device or to retrieve the current test mode setting. It should be
 * called when you need to either set a specific test mode or check the
 * current mode. The function requires a valid device structure and a
 * mode value. If the mode is within the valid range (0 to 15), it sets
 * the test mode; otherwise, it retrieves the current mode. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mode An integer representing the test mode to set. Valid values are
 * from 0 to 15. If the value is outside this range, the function
 * retrieves the current test mode instead.
 * @return Returns 0 on success when setting a mode, or the current mode value
 * if retrieving. Returns -1 on error.
 ******************************************************************************/
int32_t ad9250_test_mode(struct ad9250_dev *dev,
			 int32_t mode);
/***************************************************************************//**
 * @brief This function is used to adjust the offset of the AD9250 device by
 * writing a specified adjustment value to the offset register. It should
 * be called when an offset adjustment is needed within the valid range.
 * If the provided adjustment value is outside the valid range, the
 * function will instead return the current offset value from the device.
 * This function requires a valid device structure and should be used
 * after the device has been properly initialized.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device must be initialized before calling
 * this function.
 * @param adj An integer representing the desired offset adjustment. Valid
 * values are between -32 and 31 inclusive. If the value is outside
 * this range, the function will return the current offset value
 * instead of making an adjustment.
 * @return Returns 0 on successful adjustment within the valid range. If the
 * adjustment value is out of range, it returns the current offset
 * value.
 ******************************************************************************/
int32_t ad9250_offset_adj(struct ad9250_dev *dev,
			  int32_t adj);
/***************************************************************************//**
 * @brief This function is used to control the data output state of the AD9250
 * device. It can either enable or disable the data output based on the
 * provided parameter. This function should be called when there is a
 * need to control the data flow from the device, such as during
 * configuration changes or power management. The function expects a
 * valid device structure and a control flag to specify the desired
 * output state. If the control flag is neither 0 nor 1, the function
 * will return the current output disable state instead of modifying it.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param en An integer flag indicating the desired output state: 0 to enable
 * output, 1 to disable output. If the value is neither 0 nor 1, the
 * function returns the current output disable state without making
 * changes.
 * @return Returns 0 on success when enabling or disabling output, or -1 if an
 * error occurs during register read. If 'en' is neither 0 nor 1,
 * returns the current output disable state (0 or 1).
 ******************************************************************************/
int32_t ad9250_output_disable(struct ad9250_dev *dev,
			      int32_t en);
/***************************************************************************//**
 * @brief This function is used to set the output mode of the AD9250 device to
 * either inverted or normal. It should be called when there is a need to
 * change the data output mode of the device. The function requires a
 * valid device structure and an invert flag to specify the desired
 * output mode. If the invert parameter is not 0 or 1, the function will
 * return the current output mode state instead of setting a new mode.
 * This function must be called after the device has been properly
 * initialized.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param invert An integer flag indicating the desired output mode. Use 0 for
 * normal output and 1 for inverted output. If the value is
 * neither 0 nor 1, the function returns the current output mode
 * state.
 * @return Returns 0 on success when setting the mode, or -1 if an error occurs
 * during register read. If invert is neither 0 nor 1, returns the
 * current output mode state (0 or 1).
 ******************************************************************************/
int32_t ad9250_output_invert(struct ad9250_dev *dev,
			     int32_t invert);
/***************************************************************************//**
 * @brief This function is used to configure the output data format of the
 * AD9250 device to either offset binary or two's complement. It can also
 * be used to retrieve the current output format setting. The function
 * must be called with a valid device structure and a format value of
 * either 0 or 1 to set the format. If an invalid format is provided, the
 * function will return the current format instead. This function is
 * typically used after device initialization to ensure the output data
 * is in the desired format.
 *
 * @param dev A pointer to an initialized ad9250_dev structure representing the
 * device. Must not be null.
 * @param format An integer representing the desired output format: 0 for offset
 * binary, 1 for two's complement. If the value is not 0 or 1, the
 * function will return the current format instead of setting a
 * new one.
 * @return Returns 0 on success when setting the format, or the current format
 * (0 or 1) if an invalid format is provided. Returns -1 if there is an
 * error reading the current format.
 ******************************************************************************/
int32_t ad9250_output_format(struct ad9250_dev *dev,
			     int32_t format);
/*! Sets (1) or clears (0) the reset short PN sequence bit(PN9). */
int32_t ad9250_reset_PN29(struct ad9250_dev *dev,
			  int32_t rst);
/***************************************************************************//**
 * @brief This function is used to control the reset state of the long pseudo-
 * random noise (PN) sequence bit, known as PN23, in the AD9250 device.
 * It can be used to either set or clear this bit, depending on the value
 * of the `rst` parameter. The function must be called with a valid
 * device structure and is typically used in scenarios where the PN
 * sequence needs to be reset as part of a test or initialization
 * process. If the `rst` parameter is neither 0 nor 1, the function will
 * return the current state of the PN23 bit instead of modifying it.
 *
 * @param dev A pointer to an `ad9250_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param rst An integer value indicating the desired state of the PN23 bit.
 * Must be 0 to clear the bit or 1 to set the bit. If any other value
 * is provided, the function returns the current state of the PN23
 * bit.
 * @return Returns 0 on success when setting or clearing the bit, -1 on error,
 * or the current state of the PN23 bit if `rst` is neither 0 nor 1.
 ******************************************************************************/
int32_t ad9250_reset_pn23(struct ad9250_dev *dev,
			  int32_t rst);
/***************************************************************************//**
 * @brief This function is used to set a user-defined test pattern in the AD9250
 * device, which can be useful for testing and validation purposes. It
 * requires a valid device structure and allows specifying which user
 * pattern register to configure. The function should be called when the
 * device is properly initialized and ready for configuration. It returns
 * an integer status code indicating the success or failure of the
 * operation.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be initialized before calling
 * this function. The caller retains ownership.
 * @param pattern_no An integer specifying which user pattern register to
 * configure. Valid values depend on the number of user
 * pattern registers available in the device.
 * @param user_pattern An integer representing the user-defined pattern to be
 * set. The valid range depends on the device's capabilities
 * and the specific register being configured.
 * @return Returns an int32_t status code, where 0 indicates success and a
 * negative value indicates an error.
 ******************************************************************************/
int32_t ad9250_set_user_pattern(struct ad9250_dev *dev,
				int32_t pattern_no,
				int32_t user_pattern);
/***************************************************************************//**
 * @brief This function is used to enable or disable the Built-In Self-Test
 * (BIST) feature on the AD9250 device, or to check its current status.
 * It should be called when you need to verify the integrity of the
 * device's operation through self-testing. The function requires a valid
 * device structure and an enable flag. If the enable flag is set to 0 or
 * 1, the function will attempt to set the BIST state accordingly. If the
 * enable flag is any other value, the function will return the current
 * BIST status. Ensure the device is properly initialized before calling
 * this function.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param enable An integer flag indicating whether to enable (1) or disable (0)
 * the BIST. If set to any other value, the function returns the
 * current BIST status.
 * @return Returns 0 on success when enabling/disabling BIST, or the current
 * BIST status if the enable parameter is not 0 or 1. Returns -1 on
 * error.
 ******************************************************************************/
int32_t ad9250_bist_enable(struct ad9250_dev *dev,
			   int32_t enable);
/***************************************************************************//**
 * @brief This function is used to reset the Built-In-Self-Test (BIST) of the
 * AD9250 device or to check its current reset status. It should be
 * called when you need to either initiate a BIST reset or verify if the
 * BIST reset is active. The function requires a valid device structure
 * and a reset parameter, which determines the action to be taken. If the
 * reset parameter is neither 0 nor 1, the function will return the
 * current status of the BIST reset bit.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param reset An integer value indicating the action to be taken: 0 to clear
 * the BIST reset, 1 to set the BIST reset, or any other value to
 * read the current BIST reset status. Invalid values will result
 * in the function returning the current status of the BIST reset
 * bit.
 * @return Returns 0 on success when setting or clearing the reset. If checking
 * the status, returns the current state of the BIST reset bit. Returns
 * -1 if an error occurs during the read operation.
 ******************************************************************************/
int32_t ad9250_bist_reset(struct ad9250_dev *dev,
			  int32_t reset);
/***************************************************************************//**
 * @brief This function sets up the JESD204B interface on the AD9250 device,
 * configuring various parameters such as quick configuration options,
 * CML differential output drive levels, subclass selection, control
 * bits, lane identification, and synchronization options. It must be
 * called after the device has been initialized and before any data
 * transmission occurs. The function handles error conditions by
 * returning a negative value if any configuration step fails, ensuring
 * that the setup process is robust against invalid configurations.
 *
 * @param dev A pointer to an initialized ad9250_dev structure. This must not be
 * null, and the structure should be properly configured with
 * JESD204B settings before calling this function. The caller retains
 * ownership of the memory.
 * @return Returns 0 on success or a negative error code if any configuration
 * step fails.
 ******************************************************************************/
int32_t ad9250_jesd204b_setup(struct ad9250_dev *dev);
/***************************************************************************//**
 * @brief This function is used to set or retrieve the power mode of the
 * JESD204B data transmit block in the AD9250 device. It should be called
 * when you need to adjust the power settings of the JESD204B interface.
 * The function accepts a mode parameter to set the power mode, which
 * must be within a valid range. If the mode is outside the valid range,
 * the function returns the current power mode instead. This function
 * requires a valid device structure and should be used after the device
 * has been properly initialized.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mode An integer representing the desired power mode. Valid values are
 * 0, 1, or 2. If the value is outside this range, the function
 * returns the current power mode instead of setting a new one.
 * @return Returns 0 on success when setting the mode, or the current power mode
 * if the input mode is invalid. Returns -1 if there is an error reading
 * the current mode.
 ******************************************************************************/
int32_t ad9250_jesd204b_pwr_mode(struct ad9250_dev *dev,
				 int32_t mode);
/***************************************************************************//**
 * @brief This function is used to configure the point in the processing path of
 * a JESD204B lane where test data will be injected. It can be used to
 * set the injection point to either 1 or 2, or to read the current
 * injection point setting. The function must be called with a valid
 * device structure, and the injection point must be either 1 or 2 to set
 * a new value. If an invalid injection point is provided, the function
 * will return the current setting instead. This function is typically
 * used during testing or debugging of the JESD204B interface.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param inj_point An integer specifying the injection point. Valid values are
 * 1 and 2 for setting the injection point. If any other value
 * is provided, the function returns the current injection
 * point setting.
 * @return Returns 0 on success when setting the injection point, or the current
 * injection point setting if an invalid value is provided. Returns -1
 * if there is an error reading the current setting.
 ******************************************************************************/
int32_t ad9250_jesd204b_select_test_injection_point(struct ad9250_dev *dev,
		int32_t inj_point);
/***************************************************************************//**
 * @brief This function is used to configure the AD9250 device to operate in a
 * specific JESD204B test mode. It should be called when the device is
 * initialized and ready for configuration. The function accepts a test
 * mode value, which must be within the valid range of 0 to 13. If the
 * test mode is within this range, the function sets the device to the
 * specified test mode. If the test mode is outside this range, the
 * function returns the current test mode setting. This function is
 * useful for testing and validation purposes in systems using the
 * JESD204B interface.
 *
 * @param dev A pointer to an initialized ad9250_dev structure representing the
 * device. Must not be null.
 * @param test_mode An integer specifying the desired JESD204B test mode. Valid
 * values are from 0 to 13 inclusive. If the value is outside
 * this range, the function returns the current test mode.
 * @return Returns 0 on success when setting a valid test mode. If the test mode
 * is invalid, returns the current test mode setting. Returns -1 if
 * there is an error reading the current mode.
 ******************************************************************************/
int32_t ad9250_jesd204b_test_mode(struct ad9250_dev *dev,
				  int32_t test_mode);
/***************************************************************************//**
 * @brief This function is used to configure the inversion of the JESD204B bits
 * in the AD9250 device. It can either set the inversion state based on
 * the provided parameter or return the current inversion state if the
 * parameter is outside the expected range. This function should be
 * called when configuring the JESD204B interface to ensure the correct
 * logic level is set for the application. It is important to ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param invert An integer value indicating whether to invert the JESD204B bits
 * (1 for invert, 0 for non-invert). If the value is neither 0 nor
 * 1, the function will return the current inversion state instead
 * of setting it.
 * @return Returns 0 on success when setting the inversion state, or -1 if an
 * error occurs during a read operation. If the invert parameter is
 * neither 0 nor 1, it returns the current inversion state (1 if
 * inverted, 0 if not).
 ******************************************************************************/
int32_t ad9250_jesd204b_invert_logic(struct ad9250_dev *dev,
				     int32_t invert);
/***************************************************************************//**
 * @brief This function sets up the Fast-Detect module of the AD9250 device by
 * writing configuration values to specific registers. It should be
 * called when the Fast-Detect feature needs to be enabled or configured
 * according to the desired thresholds and dwell time. The function
 * requires a valid device structure that has been properly initialized.
 * It returns an error code if any of the register writes fail,
 * indicating that the setup was not successful.
 *
 * @param dev A pointer to an initialized ad9250_dev structure. This must not be
 * null, and the structure should be properly configured with the
 * desired Fast-Detect settings before calling this function.
 * @return Returns an int32_t value: 0 on success, or -1 if a register write
 * operation fails.
 ******************************************************************************/
int32_t ad9250_fast_detect_setup(struct ad9250_dev *dev);
/***************************************************************************//**
 * @brief Use this function to enable or disable the DC correction feature of
 * the AD9250 device. This function should be called when you need to
 * control the DC correction state, which is part of the output data
 * signal path. It is important to ensure that the device is properly
 * initialized before calling this function. The function can also be
 * used to check the current state of the DC correction if an invalid
 * value is passed for the enable parameter.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device must be initialized before use.
 * @param enable An integer value indicating whether to enable (1) or disable
 * (0) the DC correction. If a value other than 0 or 1 is
 * provided, the function returns the current state of the DC
 * correction.
 * @return Returns 0 on success when enabling or disabling, or -1 if an error
 * occurs during a read operation. If an invalid enable value is
 * provided, it returns the current state of the DC correction (0 or 1).
 ******************************************************************************/
int32_t ad9250_dcc_enable(struct ad9250_dev *dev,
			  int32_t enable);
/***************************************************************************//**
 * @brief This function is used to set or retrieve the bandwidth value for the
 * DC correction circuit of the AD9250 device. It should be called when
 * you need to configure the DC correction bandwidth or check its current
 * setting. The function requires a valid device structure and a
 * bandwidth value within the specified range. If the bandwidth value is
 * outside the valid range, the function returns the current bandwidth
 * setting instead.
 *
 * @param dev A pointer to an ad9250_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function.
 * @param bw An integer representing the desired bandwidth setting. Valid values
 * are between 0 and 13 inclusive. If the value is outside this range,
 * the function will return the current bandwidth setting instead of
 * setting a new one.
 * @return Returns 0 on success when setting a new bandwidth. If the bandwidth
 * value is out of range, it returns the current bandwidth setting.
 * Returns -1 if there is an error reading the current setting.
 ******************************************************************************/
int32_t ad9250_dcc_bandwidth(struct ad9250_dev *dev,
			     int32_t bw);
/***************************************************************************//**
 * @brief This function is used to control the freezing of the DC correction
 * value in the AD9250 device. It can be called to either freeze or
 * unfreeze the DC correction by passing the appropriate parameter. The
 * function must be called with a valid device structure that has been
 * properly initialized. If the `freeze` parameter is set to 0 or 1, the
 * function will set the corresponding freeze state. If the `freeze`
 * parameter is any other value, the function will return the current
 * freeze state of the DC correction. This function returns an error code
 * if the operation fails.
 *
 * @param dev A pointer to an initialized `ad9250_dev` structure representing
 * the device. Must not be null.
 * @param freeze An integer indicating the desired freeze state: 0 to unfreeze,
 * 1 to freeze. If set to any other value, the function returns
 * the current freeze state.
 * @return Returns 0 on success, -1 on error, or the current freeze state if
 * `freeze` is not 0 or 1.
 ******************************************************************************/
int32_t ad9250_dcc_freeze(struct ad9250_dev *dev,
			  int32_t freeze);

#endif /* __AD9250_H__ */
