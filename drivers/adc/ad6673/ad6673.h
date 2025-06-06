/***************************************************************************//**
 *   @file   AD6673.h
 *   @brief  Header file of AD6673 Driver.
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
#ifndef __AD6673_H__
#define __AD6673_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_error.h"

/******************************************************************************/
/*********************************** AD6673 ***********************************/
/******************************************************************************/

/* Registers */

#define AD6673_READ                         (1 << 15)
#define AD6673_WRITE                        (0 << 15)
#define AD6673_CNT(x)                       ((((x) & 0x3) - 1) << 13)
#define AD6673_ADDR(x)                      ((x) & 0xFF)

#define AD6673_R1B                          (1 << 8)
#define AD6673_R2B                          (2 << 8)
#define AD6673_R3B                          (3 << 8)
#define AD6673_TRANSF_LEN(x)                (((x) >> 8) & 0xFF)
#define SHADOW(x)                           ((x) << 16)

/* Chip configuration registers */
#define AD6673_REG_SPI_CFG                  (AD6673_R1B | 0x00)
#define AD6673_REG_CHIP_ID                  (AD6673_R1B | 0x01)
#define AD6673_REG_CHIP_INFO                (AD6673_R1B | 0x02)

/* Channel index and transfer registers */
#define AD6673_REG_CH_INDEX                 (AD6673_R1B | 0x05)
#define AD6673_REG_DEVICE_UPDATE            (AD6673_R1B | 0xFF)

/* Program register map */
#define AD6673_REG_PDWN                     (AD6673_R1B | 0x08)
#define AD6673_REG_CLOCK                    (AD6673_R1B | 0x09 | SHADOW(1))
#define AD6673_REG_PLL_STAT                 (AD6673_R1B | 0x0A)
#define AD6673_REG_CLOCK_DIV                (AD6673_R1B | 0x0B | SHADOW(2))
#define AD6673_REG_TEST                     (AD6673_R1B | 0x0D | SHADOW(3))
#define AD6673_REG_BIST                     (AD6673_R1B | 0x0E | SHADOW(4))
#define AD6673_REG_OFFSET                   (AD6673_R1B | 0x10 | SHADOW(5))
#define AD6673_REG_OUT_MODE                 (AD6673_R1B | 0x14 | SHADOW(6))
#define AD6673_REG_CML                      (AD6673_R1B | 0x15)
#define AD6673_REG_VREF                     (AD6673_R1B | 0x18 | SHADOW(7))
#define AD6673_REG_USER_TEST1               (AD6673_R2B | 0x1A)
#define AD6673_REG_USER_TEST2               (AD6673_R2B | 0x1C)
#define AD6673_REG_USER_TEST3               (AD6673_R2B | 0x1E)
#define AD6673_REG_USER_TEST4               (AD6673_R2B | 0x20)
#define AD6673_REG_PLL_ENCODE               (AD6673_R1B | 0x21)
#define AD6673_REG_BIST_MISR                (AD6673_R2B | 0x25)
#define AD6673_REG_SYS_CTRL                 (AD6673_R1B | 0x3A | SHADOW(8))
#define AD6673_REG_NSR_CTRL                 (AD6673_R1B | 0x3C | SHADOW(9))
#define AD6673_REG_NSR_TUNING               (AD6673_R1B | 0x3E | SHADOW(10))
#define AD6673_REG_DCC_CTRL                 (AD6673_R1B | 0x40 | SHADOW(11))
#define AD6673_REG_DCC_VAL                  (AD6673_R2B | 0x42 | SHADOW(12))
#define AD6673_REG_FAST_DETECT              (AD6673_R1B | 0x45 | SHADOW(13))
#define AD6673_REG_FD_UPPER_THD             (AD6673_R2B | 0x48 | SHADOW(14))
#define AD6673_REG_FD_LOWER_THD             (AD6673_R2B | 0x4A | SHADOW(15))
#define AD6673_REG_FD_DWELL_TIME            (AD6673_R2B | 0x4C | SHADOW(16))
#define AD6673_REG_204B_QUICK_CFG           (AD6673_R1B | 0x5E)
#define AD6673_REG_204B_CTRL1               (AD6673_R1B | 0x5F)
#define AD6673_REG_204B_CTRL2               (AD6673_R1B | 0x60)
#define AD6673_REG_204B_CTRL3               (AD6673_R1B | 0x61)
#define AD6673_REG_204B_DID_CFG             (AD6673_R1B | 0x64)
#define AD6673_REG_204B_BID_CFG             (AD6673_R1B | 0x65)
#define AD6673_REG_204B_LID_CFG1            (AD6673_R1B | 0x67)
#define AD6673_REG_204B_LID_CFG2            (AD6673_R1B | 0x68)
#define AD6673_REG_204B_PARAM_SCR_L         (AD6673_R1B | 0x6E)
#define AD6673_REG_204B_PARAM_F             (AD6673_R1B | 0x6F)
#define AD6673_REG_204B_PARAM_K             (AD6673_R1B | 0x70)
#define AD6673_REG_204B_PARAM_M             (AD6673_R1B | 0x71)
#define AD6673_REG_204B_PARAM_CS_N          (AD6673_R1B | 0x72)
#define AD6673_REG_204B_PARAM_NP            (AD6673_R1B | 0x73)
#define AD6673_REG_204B_PARAM_S             (AD6673_R1B | 0x74)
#define AD6673_REG_204B_PARAM_HD_CF         (AD6673_R1B | 0x75)
#define AD6673_REG_204B_RESV1               (AD6673_R1B | 0x76)
#define AD6673_REG_204B_RESV2               (AD6673_R1B | 0x77)
#define AD6673_REG_204B_CHKSUM0             (AD6673_R1B | 0x79)
#define AD6673_REG_204B_CHKSUM1             (AD6673_R1B | 0x7A)
#define AD6673_REG_204B_LANE_ASSGN1         (AD6673_R1B | 0x82)
#define AD6673_REG_204B_LANE_ASSGN2         (AD6673_R1B | 0x83)
#define AD6673_REG_204B_LMFC_OFFSET         (AD6673_R1B | 0x8B)
#define AD6673_REG_204B_PRE_EMPHASIS        (AD6673_R1B | 0xA8)

/* AD6673_REG_SPI_CFG */
#define AD6673_SPI_CFG_LSB_FIRST            ((1 << 6) | (1 << 1))
#define AD6673_SPI_CFG_SOFT_RST             ((1 << 5) | (1 << 2))

/* AD6673_REG_CH_INDEX */
#define AD6673_CH_INDEX_ADC_A               (1 << 0)
#define AD6673_CH_INDEX_ADC_B               (1 << 1)

/* AD6673_REG_DEVICE_UPDATE */
#define AD6673_DEVICE_UPDATE_SW             (1 << 0)

/* AD6673_REG_PDWN */
#define AD6673_PDWN_EXTERN                  (1 << 5)
#define AD6673_PDWN_JTX                     (1 << 4)
#define AD6673_PDWN_JESD204B(x)             (((x) & 0x3) << 2)
#define AD6673_PDWN_CHIP(x)                 (((x) & 0x3) << 0)

/* AD6673_REG_CLOCK */
#define AD6673_CLOCK_SELECTION(x)           (((x) & 0x3) << 4)
#define AD6673_CLOCK_DUTY_CYCLE             (1 << 0)

/* AD6673_REG_PLL_STAT */
#define AD6673_PLL_STAT_LOCKED              (1 << 7)
#define AD6673_PLL_STAT_204B_LINK_RDY       (1 << 0)

/* AD6673_REG_CLOCK_DIV */
#define AD6673_CLOCK_DIV_PHASE(x)           (((x) & 0x7) << 3)
#define AD6673_CLOCK_DIV_RATIO(x)           (((x) & 0x7) << 0)

/* AD6673_REG_TEST */
#define AD6673_TEST_USER_TEST_MODE(x)       (((x) & 0x3) << 6)
#define AD6673_TEST_RST_PN_LONG             (1 << 5)
#define AD6673_TEST_RST_PN_SHOR             (1 << 4)
#define AD6673_TEST_OUTPUT_TEST(x)          (((x) & 0xF) << 0)

/* AD6673_REG_BIST */
#define AD6673_BIST_RESET                   (1 << 2)
#define AD6673_BIST_ENABLE                  (1 << 0)

/* AD6673_REG_OFFSET */
#define AD6673_REG_OFFSET_ADJUST(x)             (((x) & 0x3F) << 0)

/* AD6673_REG_OUT_MODE */
#define AD6673_OUT_MODE_JTX_BIT_ASSIGN(x)       (((x) & 0x7) << 5)
#define AD6673_OUT_MODE_DISABLE                 (1 << 4)
#define AD6673_OUT_MODE_INVERT_DATA             (1 << 3)
#define AD6673_OUT_MODE_DATA_FORMAT(x)          (((x) & 0x1) << 0)

/* AD6673_REG_CML */
#define AD6673_CML_DIFF_OUT_LEVEL(x)            (((x) & 0x7) << 0)

/* AD6673_REG_VREF */
#define AD6673_VREF_FS_ADJUST(x)                (((x) & 0x1F) << 0)

/* AD6673_REG_PLL_ENCODE */
#define AD6673_PLL_ENCODE(x)                    (((x) & 0x3) << 3)

/* AD6673_REG_SYS_CTRL */
#define AD6673_SYS_CTRL_REALIGN_ON_SYNCINB      (1 << 4)
#define AD6673_SYS_CTRL_REALIGN_ON_SYSREF       (1 << 3)
#define AD6673_SYS_CTRL_SYSREF_MODE             (1 << 2)
#define AD6673_SYS_CTRL_SYSREF_EN               (1 << 1)
#define AD6673_SYS_CTRL_SYNCINB_EN              (1 << 0)

/* AD6673_REG_NSR_CTRL */
#define AD6673_NSR_CTRL_BW_MODE                 (1 << 1)
#define AD6673_NSR_CTRL_ENABLE                  (1 << 0)

/* AD6673_REG_NSR_TUNING */
#define AD6673_NSR_TUNING(x)                    (((x) & 0x3F) << 0)

/* AD6673_REG_DCC_CTRL */
#define AD6673_DCC_CTRL_FREEZE_DCC              (1 << 6)
#define AD6673_DCC_CTRL_DCC_BW(x)               (((x) & 0xF) << 2)
#define AD6673_DCC_CTRL_DCC_EN                  (1 << 1)

/* AD6673_REG_FAST_DETECT */
#define AD6673_FAST_DETECT_PIN_FCT              (1 << 4)
#define AD6673_FAST_DETECT_FORCE_FDA_FDB_PIN    (1 << 3)
#define AD6673_FAST_DETECT_FORCE_FDA_FDB_VAL    (1 << 2)
#define AD6673_FAST_DETECT_OUTPUT_ENABLE        (1 << 0)

/* AD6673_REG_204B_QUICK_CFG */
#define AD6673_204B_QUICK_CFG(x)                (((x) & 0xFF) << 0)

/* AD6673_REG_204B_CTRL1 */
#define AD6673_204B_CTRL1_TAIL_BITS             (1 << 6)
#define AD6673_204B_CTRL1_TEST_SAMPLE_EN        (1 << 5)
#define AD6673_204B_CTRL1_ILAS_MODE(x)          (((x) & 0x3) << 2)
#define AD6673_204B_CTRL1_POWER_DOWN            (1 << 0)

/* AD6673_REG_204B_CTRL2 */
#define AD6673_204B_CTRL2_INVERT_JESD_BITS      (1 << 1)

/* AD6673_REG_204B_CTRL3 */
#define AD6673_204B_CTRL3_TEST_DATA_INJ_PT(x)   (((x) & 0x3) << 4)
#define AD6673_204B_CTRL3_JESD_TEST_MODE(x)     (((x) & 0xF) << 0)

/* AD6673_REG_204B_PARAM_SCR_L */
#define AD6673_204B_PARAM_SCR_L_SCRAMBLING      (1 << 7)
#define AD6673_204B_PARAM_SCR_L_LANES           (1 << 0)

/* AD6673_REG_204B_PARAM_CS_N */
#define AD6673_204B_PARAM_CS_N_NR_CTRL_BITS(x)      (((x) & 0x3) << 6)
#define AD6673_204B_PARAM_CS_N_ADC_RESOLUTION(x)    (((x) & 0xF) << 0)

/* AD6673_REG_204B_PARAM_NP */
#define AD6673_204B_PARAM_NP_JESD_SUBCLASS(x)       (((x) & 0x3) << 5)
#define AD6673_204B_PARAM_NP_JESD_N_VAL(x)          (((x) & 0xF) << 0)

/* AD6673_REG_204B_PARAM_S */
#define AD6673_204B_PARAM_S(x)                  (((x) << 0x1F) << 0)

/* AD6673_REG_204B_PARAM_HD_CF */
#define AD6673_204B_PARAM_HD_CF_HD_VAL          (1 << 7)
#define AD6673_204B_PARAM_HD_CF_CF_VAL(x)       (((x) & 0x1F) << 0)

/* AD6673_REG_204B_LANE_ASSGN1 */
#define AD6673_204B_LANE_ASSGN1(x)              (((x) & 0x3) << 4)

/* AD6673_REG_204B_LANE_ASSGN2 */
#define AD6673_204B_LANE_ASSGN2(x)              (((x) &0x3) << 0)

/* AD6673_REG_204B_LMFC_OFFSET */
#define AD6673_204B_LMFC_OFFSET(x)              (((x) & 0x1F) << 0)

/*****************************************************************************/
/************************** Types Declarations *******************************/
/*****************************************************************************/

/***************************************************************************//**
 * @brief The `ad6673_platform_data` structure is used to configure platform-
 * specific settings for the AD6673 device, including power-down modes,
 * clock settings, and reference voltage adjustments. It provides fields
 * to control the external power-down mode, enable or disable the clock
 * duty cycle stabilizer, select the clock source, and set the clock
 * divider ratio and phase. Additionally, it allows for the adjustment of
 * the ADC's reference voltage and specifies the PLL encoding mode based
 * on lane speeds. The structure also includes a field for storing the
 * device name.
 *
 * @param extrn_pdwnmode Specifies the external power-down mode, where 0
 * indicates full power down and 1 indicates standby mode.
 * @param en_clk_dcs Enables or disables the clock duty cycle stabilizer, with 0
 * for disable and 1 for enable.
 * @param clk_selection Determines the clock source, with options for Nyquist
 * clock, RF clock divided by 4, or clock off.
 * @param clk_div_ratio Sets the clock divider ratio relative to the encode
 * clock, ranging from divide by 1 to divide by 8.
 * @param clk_div_phase Specifies the clock divide phase relative to the encode
 * clock, with delays from 0 to 7 input clock cycles.
 * @param adc_vref Adjusts the main reference full-scale VREF, with various
 * internal voltage peak-to-peak settings.
 * @param pll_low_encode Indicates the PLL low encode mode, with 0 for lane
 * speeds greater than 2 Gbps and 1 for less than 2 Gbps.
 * @param name Holds the device name as a string of up to 16 characters.
 ******************************************************************************/
struct ad6673_platform_data {
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
	int8_t  name[16];
};

/***************************************************************************//**
 * @brief The `ad6673_jesd204b_cfg` structure is used to configure the JESD204B
 * interface for the AD6673 device. It includes various settings such as
 * standby mode, output drive levels, quick configuration options,
 * subclass selection, control bit settings, and identification values
 * for device, bank, and lanes. Additionally, it manages frame
 * configurations, scrambling, alignment sequences, and signal
 * interpretation for synchronization and reference signals. This
 * structure allows for detailed customization of the JESD204B interface
 * to meet specific application requirements.
 *
 * @param jtx_in_standby Indicates whether the JESD204B core is in standby mode
 * or powered down except for PLL.
 * @param cml_level Adjusts the JESD204B CML differential output drive level.
 * @param quick_cfg_option Specifies a quick configuration option for converters
 * and lanes.
 * @param subclass Defines the JESD204B subclass (0 or 1).
 * @param ctrl_bits_no Specifies the number of control bits (CS) used.
 * @param ctrl_bits_assign Determines the assignment of JTX CS bits.
 * @param tail_bits_mode Specifies the mode for tail bits when CS bits are not
 * enabled.
 * @param did Device identification value for JESD204B.
 * @param bid Bank identification value for JESD204B.
 * @param lid0 Lane0 identification value for JESD204B.
 * @param lid1 Lane1 identification value for JESD204B.
 * @param k Number of frames per multiframe, must be a multiple of 4 octets.
 * @param scrambling Indicates whether JESD204B scrambling is enabled or
 * disabled.
 * @param ilas_mode Specifies the Initial Lane Alignment Sequence mode.
 * @param en_ilas_test Enables or disables the JESD204B test sample.
 * @param invert_logic_bits Indicates whether the logic of JESD204B bits is
 * inverted.
 * @param en_sys_ref Enables or disables SYSREF+-.
 * @param en_sync_in_b Enables or disables the SYNCINB+- buffer.
 * @param sys_ref_mode Specifies the SYSREF+- mode for clock dividers.
 * @param align_sync_in_b Options for interpreting signals on SYNCINB+-.
 * @param align_sys_ref Options for interpreting signals on SYSREF+-.
 * @param lane0_assign Option to remap converter and lane assignments for
 * Logical Lane 0.
 * @param lane1_assign Option to remap converter and lane assignments for
 * Logical Lane 1.
 ******************************************************************************/
struct ad6673_jesd204b_cfg {
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
 * @brief The `ad6673_fast_detect_cfg` structure is used to configure the fast
 * detect module of the AD6673 device. It includes settings to enable or
 * disable fast detect output, configure pin functions, and force pin
 * values. Additionally, it allows setting upper and lower thresholds for
 * fast detection and specifies the dwell time, which is the duration for
 * which the fast detect condition must be met before triggering an
 * output.
 *
 * @param en_fd Enables or disables the fast detect output.
 * @param pin_function Determines the function of the pin, either fast detect or
 * overrange.
 * @param force_pins Forces the FDA/FDB pins to a specific value or allows
 * normal function.
 * @param pin_force_value Specifies the forced output value of the FDA/FDB pins.
 * @param fd_upper_tresh Sets the upper threshold for fast detection.
 * @param fd_lower_tresh Sets the lower threshold for fast detection.
 * @param df_dwell_time Specifies the dwell time for fast detection.
 ******************************************************************************/
struct ad6673_fast_detect_cfg {
	/**
	 * Enable fast detect output.
	 * 0 = disable
	 * 1 = enable
	 */
	int8_t  en_fd;
	/**
	 * Pin function.
	 * 0 = fast detect
	 * 1 = overrange
	 */
	int8_t  pin_function;
	/**
	 * Force FDA/FDB pins
	 * 0 = normal function
	 * 1 = force to value
	 */
	int8_t  force_pins;
	/**
	 * Force value of FDA/FDB pins.
	 * 0 = output on FD pins will be 0
	 * 1 = output on FD pins will be 1
	 */
	int8_t  pin_force_value;
	/** Fast Detect Upper Threshold[14:0]. */
	int16_t fd_upper_tresh;
	/** Fast Detect Lower Threshold[14:0]. */
	int16_t fd_lower_tresh;
	/** Fast Detect Dwell Time[15:0]. */
	int16_t df_dwell_time;
};

/***************************************************************************//**
 * @brief The `ad6673_type_band` structure is used to define a frequency band
 * with three key parameters: the starting frequency (`f0`), the center
 * frequency (`f_center`), and the ending frequency (`f1`). This
 * structure is likely used in the context of configuring or analyzing
 * frequency bands in the AD6673 device, which is a high-speed analog-to-
 * digital converter. Each member of the structure is an integer
 * representing a frequency value, allowing for precise control and
 * specification of frequency bands within the device's operational
 * parameters.
 *
 * @param f0 Represents the starting frequency of the band.
 * @param f_center Represents the center frequency of the band.
 * @param f1 Represents the ending frequency of the band.
 ******************************************************************************/
struct ad6673_type_band {
	int32_t f0;
	int32_t f_center;
	int32_t f1;
};

/***************************************************************************//**
 * @brief The `ad6673_state` structure is a compound data type used to
 * encapsulate the state and configuration of the AD6673 device. It
 * contains pointers to three other structures: `ad6673_platform_data`,
 * which holds platform-specific settings; `ad6673_jesd204b_cfg`, which
 * manages the JESD204B interface configuration; and
 * `ad6673_fast_detect_cfg`, which configures the fast detect module.
 * This structure is essential for managing the device's operational
 * parameters and interfacing with its various subsystems.
 *
 * @param pdata Pointer to platform-specific configuration data.
 * @param p_jesd204b Pointer to JESD204B interface configuration data.
 * @param p_fd Pointer to fast detect module configuration data.
 ******************************************************************************/
struct ad6673_state {
	struct ad6673_platform_data   *pdata;
	struct ad6673_jesd204b_cfg    *p_jesd204b;
	struct ad6673_fast_detect_cfg *p_fd;
};

/***************************************************************************//**
 * @brief The `shadow_registers` enum defines a set of constants representing
 * various shadow registers used in the AD6673 device driver. These
 * registers are used to configure different aspects of the device, such
 * as clock settings, test modes, offset adjustments, output modes,
 * voltage reference, system control, noise shaped requantizer settings,
 * DC correction, and fast detect parameters. Each constant in the enum
 * corresponds to a specific register, allowing for organized and
 * readable code when accessing or modifying these settings.
 *
 * @param AD6673_SHD_REG_CLOCK Represents the shadow register for clock
 * settings.
 * @param AD6673_SHD_REG_CLOCK_DIV Represents the shadow register for clock
 * division settings.
 * @param AD6673_SHD_REG_TEST Represents the shadow register for test settings.
 * @param AD6673_SHD_REG_BIST Represents the shadow register for Built-In Self-
 * Test settings.
 * @param AD6673_SHD_REG_OFFSET Represents the shadow register for offset
 * settings.
 * @param AD6673_SHD_REG_OUT_MODE Represents the shadow register for output mode
 * settings.
 * @param AD6673_SHD_REG_VREF Represents the shadow register for voltage
 * reference settings.
 * @param AD6673_SHD_REG_SYS_CTRL Represents the shadow register for system
 * control settings.
 * @param AD6673_REG_SHD_NSR_CTRL Represents the shadow register for noise
 * shaped requantizer control settings.
 * @param AD6673_REG_SHD_NSR_TUNING Represents the shadow register for noise
 * shaped requantizer tuning settings.
 * @param AD6673_SHD_REG_DCC_CTRL Represents the shadow register for DC
 * correction control settings.
 * @param AD6673_SHD_REG_DCC_VAL Represents the shadow register for DC
 * correction value settings.
 * @param AD6673_SHD_REG_FAST_DETECT Represents the shadow register for fast
 * detect settings.
 * @param AD6673_SHD_REG_FD_UPPER_THD Represents the shadow register for fast
 * detect upper threshold settings.
 * @param AD6673_SHD_REG_FD_LOWER_THD Represents the shadow register for fast
 * detect lower threshold settings.
 * @param AD6673_SHD_REG_FD_DWELL_TIME Represents the shadow register for fast
 * detect dwell time settings.
 * @param SHADOW_REGISTER_COUNT Represents the total count of shadow registers.
 ******************************************************************************/
enum shadow_registers {
	AD6673_SHD_REG_CLOCK = 1,
	AD6673_SHD_REG_CLOCK_DIV,
	AD6673_SHD_REG_TEST,
	AD6673_SHD_REG_BIST,
	AD6673_SHD_REG_OFFSET,
	AD6673_SHD_REG_OUT_MODE,
	AD6673_SHD_REG_VREF,
	AD6673_SHD_REG_SYS_CTRL,
	AD6673_REG_SHD_NSR_CTRL,
	AD6673_REG_SHD_NSR_TUNING,
	AD6673_SHD_REG_DCC_CTRL,
	AD6673_SHD_REG_DCC_VAL,
	AD6673_SHD_REG_FAST_DETECT,
	AD6673_SHD_REG_FD_UPPER_THD,
	AD6673_SHD_REG_FD_LOWER_THD,
	AD6673_SHD_REG_FD_DWELL_TIME,
	SHADOW_REGISTER_COUNT
};

/***************************************************************************//**
 * @brief The `ad6673_dev` structure is designed to encapsulate the necessary
 * components for managing and interfacing with an AD6673 device. It
 * includes a pointer to a SPI descriptor (`spi_desc`) for handling SPI
 * communication, a `ad6673_state` structure (`ad6673_st`) that maintains
 * the current state and configuration settings of the device, and an
 * array (`shadow_regs`) to store the values of shadow registers, which
 * are used to keep track of register states that may not be directly
 * accessible or visible.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param ad6673_st Holds the state and configuration of the AD6673 device.
 * @param shadow_regs Array to store shadow register values for the device.
 ******************************************************************************/
struct ad6673_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* Device Settings */
	struct ad6673_state ad6673_st;
	int32_t shadow_regs[SHADOW_REGISTER_COUNT];
};

/***************************************************************************//**
 * @brief The `ad6673_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD6673 device,
 * specifically focusing on the SPI interface configuration. This
 * structure is essential for ensuring that the SPI communication is
 * correctly established with the device, allowing for further
 * configuration and operation of the AD6673.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 ******************************************************************************/
struct ad6673_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function sets up the AD6673 device by initializing the SPI
 * interface, resetting the device to its default state, and configuring
 * various device parameters such as power modes, clock settings, and the
 * JESD204B interface. It must be called before any other operations on
 * the AD6673 device to ensure proper initialization. The function
 * allocates memory for the device structure and returns a pointer to it
 * through the provided pointer parameter. If any step in the setup
 * process fails, the function returns an error code and the device
 * pointer is not valid.
 *
 * @param device A pointer to a pointer of type `struct ad6673_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A structure of type `struct ad6673_init_param` containing
 * initialization parameters for the SPI interface. Must be
 * properly initialized before calling this function.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t ad6673_setup(struct ad6673_dev **device,
		     struct ad6673_init_param init_param);
/***************************************************************************//**
 * @brief This function is used to release all resources allocated for an AD6673
 * device instance, including the SPI descriptor and any memory allocated
 * for the device structure. It should be called when the device is no
 * longer needed to ensure proper cleanup and avoid memory leaks. The
 * function must be called only after the device has been successfully
 * initialized and used.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device
 * instance to be removed. Must not be null. The function will handle
 * freeing the memory, so the caller should not attempt to free it
 * again.
 * @return Returns an int32_t indicating the success or failure of the SPI
 * removal operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad6673_remove(struct ad6673_dev *dev);
/***************************************************************************//**
 * @brief This function retrieves the value stored in a specified register of
 * the AD6673 device. It is typically used to access configuration or
 * status information from the device. The function requires a valid
 * device structure and a register address as inputs. It returns the
 * register value on success or an error code if the read operation
 * fails. Ensure that the device has been properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param register_address An integer representing the address of the register
 * to be read. Must be a valid register address for the
 * AD6673 device.
 * @return Returns the value of the specified register on success, or an error
 * code if the operation fails.
 ******************************************************************************/
int32_t ad6673_read(struct ad6673_dev *dev,
		    int32_t register_address);
/***************************************************************************//**
 * @brief This function is used to write a specified value to a register on the
 * AD6673 device. It should be called when a register needs to be updated
 * with a new value. The function checks if the register is shadowed and
 * updates the shadow register if necessary. It requires a valid device
 * structure and register address. The function handles multi-byte
 * transfers based on the register's transfer length and returns an error
 * code if the SPI write operation fails.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param register_address An integer specifying the address of the register to
 * write to. Must be a valid register address for the
 * AD6673.
 * @param register_value An integer specifying the value to write to the
 * register. The value is written as-is to the specified
 * register.
 * @return Returns an integer indicating success (0) or an error code if the
 * write operation fails.
 ******************************************************************************/
int32_t ad6673_write(struct ad6673_dev *dev,
		     int32_t register_address,
		     int32_t register_value);
/***************************************************************************//**
 * @brief This function is used to initiate a transfer operation on the AD6673
 * device and waits for the operation to complete. It should be called
 * when a device update is required. The function writes to the device
 * update register to trigger the transfer and then polls the register to
 * check for completion. It is important to ensure that the device is
 * properly initialized before calling this function. The function will
 * return immediately if an error occurs during the write or read
 * operations.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership of the structure.
 * @return Returns 0 on success, or a negative error code if a read or write
 * operation fails.
 ******************************************************************************/
int32_t ad6673_transfer(struct ad6673_dev *dev);
/***************************************************************************//**
 * @brief This function resets the AD6673 device to its default SPI
 * configuration values. It should be used when a reset of the device's
 * configuration is necessary, such as after initialization or when
 * recovering from an error state. The function requires a valid device
 * structure and will return an error code if the reset operation fails.
 * It is important to ensure that the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership and is responsible for
 * ensuring the device is initialized.
 * @return Returns 0 on success or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int32_t ad6673_soft_reset(struct ad6673_dev *dev);
/***************************************************************************//**
 * @brief This function sets the power mode of the AD6673 chip to one of the
 * predefined modes or retrieves the current power mode if an invalid
 * mode is specified. It should be used to manage the power consumption
 * of the device, particularly in applications where power efficiency is
 * critical. The function must be called with a valid device structure,
 * and it is expected that the device has been properly initialized
 * before calling this function. If the mode is outside the valid range,
 * the function returns the current power mode instead of setting a new
 * one.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device must be initialized before use.
 * @param mode An integer representing the desired power mode. Valid values are
 * 0, 1, or 2. If the value is outside this range, the function will
 * return the current power mode instead of setting a new one.
 * @return Returns 0 on success when setting a power mode, or the current power
 * mode if an invalid mode is specified. Returns a negative error code
 * on failure.
 ******************************************************************************/
int32_t ad6673_chip_pwr_mode(struct ad6673_dev *dev,
			     int32_t mode);
/***************************************************************************//**
 * @brief This function is used to select a specific channel on the AD6673
 * device for further configuration. It should be called when a
 * particular channel needs to be configured, and the channel number must
 * be within the valid range. If the channel number is valid, it writes
 * the channel index to the device; otherwise, it reads the current
 * channel configuration. This function is typically used in the setup or
 * reconfiguration phase of the device operation.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param channel An integer representing the channel to be selected. Valid
 * values are 1, 2, or 3. If the value is outside this range, the
 * function will read the current channel configuration instead
 * of writing.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad6673_select_channel_for_config(struct ad6673_dev *dev,
		int32_t channel);
/***************************************************************************//**
 * @brief This function is used to configure the test mode of the AD6673 device
 * or to retrieve the current test mode setting. It should be called with
 * a valid mode value when setting the test mode, or with an invalid mode
 * value to read the current mode. The function must be called with a
 * properly initialized device structure. If the mode is within the valid
 * range (0 to 15), the function sets the test mode. If the mode is
 * outside this range, it returns the current test mode. This function is
 * useful for testing and diagnostic purposes.
 *
 * @param dev A pointer to an initialized ad6673_dev structure. Must not be
 * null. The caller retains ownership.
 * @param mode An integer specifying the test mode to set, ranging from 0 to 15.
 * If the value is outside this range, the function will return the
 * current test mode instead of setting a new one.
 * @return Returns 0 on success when setting a mode, or the current test mode if
 * the input mode is invalid. Returns a negative error code on failure.
 ******************************************************************************/
int32_t ad6673_test_mode(struct ad6673_dev *dev,
			 int32_t mode);
/***************************************************************************//**
 * @brief This function sets the offset adjustment for the AD6673 device if the
 * provided adjustment value is within the valid range. It should be used
 * when there is a need to calibrate or fine-tune the offset of the
 * device. The function must be called with a valid device structure and
 * an adjustment value within the specified range. If the adjustment
 * value is outside the valid range, the function returns the current
 * offset value instead of setting a new one.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param adj An integer representing the desired offset adjustment. Valid
 * values are between -32 and 31 inclusive. If the value is outside
 * this range, the function will not set a new offset but will return
 * the current offset value.
 * @return Returns 0 on successful adjustment within the valid range. If the
 * adjustment value is out of range, it returns the current offset
 * value.
 ******************************************************************************/
int32_t ad6673_offset_adj(struct ad6673_dev *dev,
			  int32_t adj);
/***************************************************************************//**
 * @brief This function is used to control the data output state of the AD6673
 * device. It can either enable or disable the data output based on the
 * provided parameter. This function should be called when there is a
 * need to control the data flow from the device, such as during
 * initialization or when changing operational modes. The function
 * expects a valid device structure and a control flag to specify the
 * desired output state. If the control flag is neither 0 nor 1, the
 * function will return the current output disable state instead of
 * modifying it.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param en An integer flag indicating the desired output state: 0 to enable
 * output, 1 to disable output. If set to any other value, the
 * function returns the current output disable state without making
 * changes.
 * @return Returns 0 on success when setting the output state, or the current
 * output disable state if the input is invalid. Returns a negative
 * error code on failure.
 ******************************************************************************/
int32_t ad6673_output_disable(struct ad6673_dev *dev,
			      int32_t en);
/***************************************************************************//**
 * @brief This function is used to set the output mode of the AD6673 device to
 * either inverted or normal. It should be called when there is a need to
 * change the data output mode of the device. The function requires a
 * valid device structure and an invert flag indicating the desired
 * output mode. If the invert parameter is not 0 or 1, the function will
 * return the current output mode status instead of changing it. This
 * function is typically used in applications where the output data
 * format needs to be adjusted for compatibility with other system
 * components.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param invert An integer indicating the desired output mode: 0 for normal
 * output, 1 for inverted output. If the value is neither 0 nor 1,
 * the function returns the current output mode status.
 * @return Returns 0 on success when setting the mode, or the current mode
 * status if the invert parameter is invalid. Returns a negative error
 * code on failure.
 ******************************************************************************/
int32_t ad6673_output_invert(struct ad6673_dev *dev,
			     int32_t invert);
/***************************************************************************//**
 * @brief This function is used to set or retrieve the output data format of the
 * AD6673 device. It should be called when there is a need to configure
 * the data format for the device's output. The function accepts a format
 * parameter that determines the desired output format. If the format is
 * set to 0 or 1, the function configures the device accordingly. If the
 * format is any other value, the function returns the current output
 * format setting. This function must be called with a valid device
 * structure that has been properly initialized.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. This
 * must not be null and should be initialized before calling the
 * function. The caller retains ownership.
 * @param format An integer specifying the desired output format. Valid values
 * are 0 or 1. If an invalid value is provided, the function
 * returns the current format setting instead of configuring a new
 * format.
 * @return Returns 0 on success when setting the format, or the current format
 * setting if an invalid format is provided. Returns a negative error
 * code if an error occurs during the operation.
 ******************************************************************************/
int32_t ad6673_output_format(struct ad6673_dev *dev,
			     int32_t format);
/***************************************************************************//**
 * @brief This function is used to control the reset state of the short PN
 * sequence bit (PN9) in the AD6673 device. It can either set or clear
 * the reset bit based on the provided parameter. This function should be
 * called when there is a need to reset the PN9 sequence, typically
 * during initialization or testing phases. The function expects a valid
 * device structure and a reset command, and it returns the status of the
 * operation.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param rst An integer value indicating the reset command. Valid values are 0
 * to clear the reset bit and 1 to set the reset bit. If a value
 * other than 0 or 1 is provided, the function will return the
 * current state of the reset bit.
 * @return Returns an int32_t status code. If the input is valid, it returns 0
 * on success or a negative error code on failure. If the input is
 * invalid, it returns the current state of the reset bit.
 ******************************************************************************/
int32_t ad6673_reset_pn9(struct ad6673_dev *dev,
			 int32_t rst);
/***************************************************************************//**
 * @brief This function is used to control the reset state of the long pseudo-
 * random noise (PN) sequence bit, known as PN23, in the AD6673 device.
 * It can either set or clear the reset bit based on the provided
 * parameter. This function should be called when you need to initialize
 * or verify the state of the PN23 sequence. It is important to ensure
 * that the device is properly initialized before calling this function
 * to avoid unexpected behavior.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device
 * instance. Must not be null, and the device must be properly
 * initialized before use.
 * @param rst An integer value indicating the desired state of the PN23 reset
 * bit. Valid values are 0 to clear the reset bit and 1 to set the
 * reset bit. If a value other than 0 or 1 is provided, the function
 * will return the current state of the PN23 reset bit.
 * @return Returns 0 on success when setting or clearing the reset bit. If an
 * invalid rst value is provided, it returns the current state of the
 * PN23 reset bit, or an error code if the read operation fails.
 ******************************************************************************/
int32_t ad6673_reset_pn23(struct ad6673_dev *dev,
			  int32_t rst);
/***************************************************************************//**
 * @brief This function is used to set a user-defined test pattern in the AD6673
 * device, which can be useful for testing and validation purposes. It
 * requires a valid device structure and allows specifying the pattern
 * number and the pattern itself. The function should be called when the
 * device is in a state ready to accept configuration changes. It returns
 * an integer status code indicating the success or failure of the
 * operation.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param pattern_no An integer specifying the pattern number to set. Valid
 * values depend on the device's capabilities and should be
 * within the range supported by the device.
 * @param user_pattern An integer representing the user-defined pattern to be
 * set. The value should be within the range supported by
 * the device for user patterns.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int32_t ad6673_set_user_pattern(struct ad6673_dev *dev,
				int32_t pattern_no,
				int32_t user_pattern);
/***************************************************************************//**
 * @brief This function is used to enable or disable the Built-In Self-Test
 * (BIST) feature on the AD6673 device, or to check its current status.
 * It should be called with a valid device structure and an enable
 * parameter. If the enable parameter is set to 0 or 1, the function will
 * enable or disable the BIST, respectively. If the enable parameter is
 * any other value, the function will return the current status of the
 * BIST. This function is useful for diagnostic purposes to ensure the
 * device is functioning correctly.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param enable An integer indicating whether to enable (1) or disable (0) the
 * BIST. If set to any other value, the function returns the
 * current BIST status.
 * @return Returns 0 on success when enabling or disabling BIST, or the current
 * BIST status if the enable parameter is not 0 or 1. Returns a negative
 * error code on failure.
 ******************************************************************************/
int32_t ad6673_bist_enable(struct ad6673_dev *dev,
			   int32_t enable);
/***************************************************************************//**
 * @brief Use this function to reset the Built-In Self-Test (BIST) of the AD6673
 * device or to check its current reset status. This function should be
 * called when you need to either initiate a BIST reset or verify if the
 * BIST reset is active. The function requires a valid device structure
 * and a reset parameter to specify the desired action. It is important
 * to ensure that the device is properly initialized before calling this
 * function.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reset An integer specifying the action: 0 to clear the BIST reset, 1
 * to set the BIST reset, or any other value to check the current
 * BIST reset status. Invalid values are treated as a request to
 * check the status.
 * @return Returns 0 on success when setting or clearing the reset. When
 * checking the status, returns the current BIST reset status (0 or 1).
 * Returns a negative error code on failure.
 ******************************************************************************/
int32_t ad6673_bist_reset(struct ad6673_dev *dev,
			  int32_t reset);
/***************************************************************************//**
 * @brief This function sets up the JESD204B interface on the AD6673 device,
 * configuring various parameters such as quick configuration options,
 * CML differential output drive level, subclass, control bits, lane
 * identification, and synchronization options. It must be called after
 * the device has been initialized and before any data transmission
 * occurs. The function handles disabling and re-enabling lanes during
 * configuration and requires a valid device structure with pre-
 * configured JESD204B settings. It returns an error code if any
 * configuration step fails.
 *
 * @param dev A pointer to an initialized ad6673_dev structure. This structure
 * must contain valid JESD204B configuration settings. The pointer
 * must not be null, and the caller retains ownership.
 * @return Returns 0 on success or a negative error code if any configuration
 * step fails.
 ******************************************************************************/
int32_t ad6673_jesd204b_setup(struct ad6673_dev *dev);
/***************************************************************************//**
 * @brief This function is used to set or retrieve the power mode of the
 * JESD204B data transmit block in the AD6673 device. It should be called
 * when you need to change the power mode or check the current power mode
 * of the JESD204B interface. The function requires a valid device
 * structure and a mode value. If the mode is within the valid range, it
 * sets the power mode; otherwise, it retrieves the current mode. Ensure
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param mode An integer representing the desired power mode. Valid values are
 * 0, 1, or 2. If the value is outside this range, the function will
 * return the current power mode instead of setting a new one.
 * @return Returns 0 on success when setting the mode, or the current mode if
 * the input mode is invalid. Returns a negative error code on failure.
 ******************************************************************************/
int32_t ad6673_jesd204b_pwr_mode(struct ad6673_dev *dev,
				 int32_t mode);
/***************************************************************************//**
 * @brief This function is used to configure the point in the processing path of
 * a JESD204B lane where test data will be injected. It is typically used
 * during testing and debugging of the JESD204B interface. The function
 * must be called with a valid device structure and an appropriate
 * injection point value. If the injection point is set to 1 or 2, the
 * function configures the device accordingly. If an invalid injection
 * point is provided, the function returns the current configuration
 * without making changes. This function should be used after the device
 * has been properly initialized and configured.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param inj_point An integer specifying the injection point. Valid values are
 * 1 and 2. If an invalid value is provided, the function
 * returns the current injection point configuration.
 * @return Returns 0 on success or a negative error code on failure. If an
 * invalid injection point is provided, it returns the current injection
 * point configuration.
 ******************************************************************************/
int32_t ad6673_jesd204b_select_test_injection_point(struct ad6673_dev *dev,
		int32_t inj_point);
/***************************************************************************//**
 * @brief This function is used to configure the AD6673 device to operate in a
 * specific JESD204B test mode. It should be called when the device needs
 * to be set to a test mode for validation or diagnostic purposes. The
 * function accepts a test mode value and applies it if it falls within
 * the valid range. If the test mode is outside the valid range, the
 * function returns the current test mode setting. This function requires
 * a valid device structure and should be used after the device has been
 * properly initialized.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device must be initialized before calling
 * this function.
 * @param test_mode An integer specifying the desired JESD204B test mode. Valid
 * values are from 0 to 13 inclusive. If the value is outside
 * this range, the function returns the current test mode
 * setting.
 * @return Returns 0 on success when setting a valid test mode. If the test mode
 * is invalid, returns the current test mode setting. If an error occurs
 * during reading, returns a non-zero error code.
 ******************************************************************************/
int32_t ad6673_jesd204b_test_mode(struct ad6673_dev *dev,
				  int32_t test_mode);
/***************************************************************************//**
 * @brief This function is used to configure the inversion of the JESD204B bits
 * in the AD6673 device. It can either set the inversion state or
 * retrieve the current state. The function should be called with a valid
 * device structure. If the `invert` parameter is set to 0 or 1, it will
 * configure the inversion state accordingly. If the `invert` parameter
 * is any other value, the function will return the current inversion
 * state. This function is useful for applications that require specific
 * data bit inversion settings for JESD204B communication.
 *
 * @param dev A pointer to an `ad6673_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param invert An integer indicating the desired inversion state. Valid values
 * are 0 (non-inverted) and 1 (inverted). Any other value will
 * cause the function to return the current inversion state.
 * @return Returns 0 on success when setting the inversion state, or the current
 * inversion state (0 or 1) if querying. Returns a negative error code
 * on failure.
 ******************************************************************************/
int32_t ad6673_jesd204b_invert_logic(struct ad6673_dev *dev,
				     int32_t invert);
/***************************************************************************//**
 * @brief This function sets up the Fast-Detect module of the AD6673 device by
 * writing configuration values to specific registers. It should be
 * called after the device has been initialized and before using the
 * Fast-Detect feature. The function configures the output enable, pin
 * function, force pin settings, and threshold values for the Fast-Detect
 * module. It returns an error code if any of the register writes fail,
 * indicating that the setup was not successful.
 *
 * @param dev A pointer to an initialized ad6673_dev structure. Must not be
 * null. The function will use this structure to access device-
 * specific settings and perform register writes.
 * @return Returns 0 on success or a negative error code if a register write
 * fails.
 ******************************************************************************/
int32_t ad6673_fast_detect_setup(struct ad6673_dev *dev);
/***************************************************************************//**
 * @brief This function is used to control the DC correction feature of the
 * AD6673 device. It can enable or disable the DC correction based on the
 * provided parameter. The function must be called with a valid device
 * structure and an appropriate enable value. If the enable parameter is
 * neither 0 nor 1, the function will return the current state of the DC
 * correction feature. This function is typically used after the device
 * has been properly initialized and configured.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be initialized before calling
 * this function.
 * @param enable An integer value indicating whether to enable (1) or disable
 * (0) the DC correction. If a value other than 0 or 1 is
 * provided, the function returns the current state of the DC
 * correction.
 * @return Returns 0 on success or a negative error code on failure. If the
 * enable parameter is invalid, it returns the current state of the DC
 * correction (0 or 1).
 ******************************************************************************/
int32_t ad6673_dcc_enable(struct ad6673_dev *dev,
			  int32_t enable);
/***************************************************************************//**
 * @brief This function is used to configure the bandwidth of the DC correction
 * circuit in the AD6673 device. It should be called when you need to
 * adjust the DC correction bandwidth to a specific value. The function
 * accepts a bandwidth value within a specified range and applies it to
 * the device. If the provided bandwidth value is outside the valid
 * range, the function returns the current bandwidth setting instead.
 * This function must be called with a valid device structure that has
 * been properly initialized.
 *
 * @param dev A pointer to an initialized ad6673_dev structure representing the
 * device. Must not be null.
 * @param bw An integer representing the desired bandwidth value. Valid range is
 * 0 to 13 inclusive. If the value is outside this range, the function
 * returns the current bandwidth setting.
 * @return Returns 0 on success when setting a new bandwidth. If the bandwidth
 * value is out of range, returns the current bandwidth setting. Returns
 * a negative error code on failure.
 ******************************************************************************/
int32_t ad6673_dcc_bandwidth(struct ad6673_dev *dev,
			     int32_t bw);
/***************************************************************************//**
 * @brief This function is used to control the freezing of the DC correction
 * value in the AD6673 device. It can either freeze the current DC
 * correction value or unfreeze it, allowing it to be updated. This
 * function should be called when you need to maintain a stable DC
 * correction value or when you want to allow the device to adjust the
 * correction dynamically. It is important to ensure that the device is
 * properly initialized before calling this function. The function also
 * allows querying the current freeze status if an invalid freeze value
 * is provided.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device must be initialized before use.
 * @param freeze An integer indicating whether to freeze (1) or unfreeze (0) the
 * DC correction value. If a value other than 0 or 1 is provided,
 * the function returns the current freeze status.
 * @return Returns 0 on success, or a negative error code if an error occurs. If
 * an invalid freeze value is provided, it returns the current freeze
 * status.
 ******************************************************************************/
int32_t ad6673_dcc_freeze(struct ad6673_dev *dev,
			  int32_t freeze);
/***************************************************************************//**
 * @brief This function is used to enable or disable the Noise Shaped
 * Requantizer (NSR) feature on the AD6673 device. It should be called
 * when the user needs to control the NSR functionality, which can be
 * useful for optimizing the performance of the device in specific
 * applications. The function must be called with a valid device
 * structure and an appropriate enable flag. If the enable parameter is
 * neither 0 nor 1, the function will return the current state of the NSR
 * enable bit. This function requires a valid device structure and
 * assumes that the device has been properly initialized.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param enable An integer value indicating whether to enable (1) or disable
 * (0) the NSR feature. If the value is neither 0 nor 1, the
 * function returns the current state of the NSR enable bit.
 * @return Returns 0 on success or a negative error code on failure. If the
 * enable parameter is invalid, returns the current state of the NSR
 * enable bit.
 ******************************************************************************/
int32_t ad6673_nsr_enable(struct ad6673_dev *dev,
			  int32_t enable);
/***************************************************************************//**
 * @brief This function is used to configure the Noise Shaped Requantizer (NSR)
 * bandwidth mode of the AD6673 device. It can be used to set the
 * bandwidth mode to either 0 or 1, or to read the current mode if an
 * invalid mode is provided. This function should be called when the NSR
 * feature needs to be configured or queried. The device must be properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad6673_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param mode An integer representing the desired NSR bandwidth mode. Valid
 * values are 0 and 1. If an invalid value is provided, the function
 * returns the current mode.
 * @return Returns 0 on success when setting the mode, or the current mode if an
 * invalid mode is provided. Returns a negative error code on failure.
 ******************************************************************************/
int32_t ad6673_nsr_bandwidth_mode(struct ad6673_dev *dev,
				  int32_t mode);
/***************************************************************************//**
 * @brief This function is used to calculate the tuning word for the noise-
 * shaped requantizer (NSR) based on the desired tuning frequency and the
 * ADC clock frequency. It updates the provided `ad6673_type_band`
 * structure with the calculated frequency band values. The function
 * should be called when configuring the NSR frequency range, ensuring
 * that the `p_band` pointer is valid and points to a pre-allocated
 * structure. The function handles out-of-range tuning words by setting
 * all frequency values in the structure to zero.
 *
 * @param tune_freq The desired tuning frequency in Hz. It should be a non-
 * negative integer.
 * @param f_adc The ADC clock frequency in Hz. It should be a positive integer.
 * @param p_band A pointer to a `struct ad6673_type_band` where the calculated
 * frequency band values will be stored. Must not be null.
 * @return Returns the calculated tuning word as an int32_t. If the tuning word
 * is out of the valid range (0 to 56), the frequency values in `p_band`
 * are set to zero.
 ******************************************************************************/
int32_t ad6673_nsr_tuning_freq(int64_t tune_freq,
			       int64_t f_adc,
			       struct ad6673_type_band *p_band);

#endif /* __AD6673_H__ */
