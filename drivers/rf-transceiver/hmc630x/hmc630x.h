/***************************************************************************//**
 *   @file   hmc630x.h
 *   @brief  hmc6300 and hmc6301 device driver header.
 *   @author Darius Berghe (darius.berghe@analog.com)
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
#ifndef _HMC630X_H_
#define _HMC630X_H_

#include <stdbool.h>
#include <stdint.h>
#include "no_os_util.h"
#include "no_os_gpio.h"

/* Code the row on highest 8 (of 16) bits and the field mask on lowest 8 bits. */
#define HMC630X_PARAM(row, mask)	(((((uint16_t)row) << 8)) | (mask))
#define HMC630X_ROW(param)		((param) >> 8)
#define HMC630X_MASK(param)		((param) & 0xff)

/* ROW0 */
#define HMC6301_LNA_PWRDWN		HMC630X_PARAM(0, NO_OS_BIT(7))
#define HMC6301_BBAMP_PWRDN_I		HMC630X_PARAM(0, NO_OS_BIT(6))
#define HMC6301_BBAMP_PWRDN_Q		HMC630X_PARAM(0, NO_OS_BIT(5))
#define HMC6301_DIVIDER_PWRDN		HMC630X_PARAM(0, NO_OS_BIT(4))
#define HMC6301_MIXER_PWRDN		HMC630X_PARAM(0, NO_OS_BIT(3))
#define HMC6301_IFMIXER_PWRDN_I		HMC630X_PARAM(0, NO_OS_BIT(2))
#define HMC6301_TRIPLER_PWRDN		HMC630X_PARAM(0, NO_OS_BIT(1))
#define HMC6301_IFVGA_PWRDN		HMC630X_PARAM(0, NO_OS_BIT(0))

/* ROW1 */
#define HMC6300_PA_SEL_VGBS		HMC630X_PARAM(1, NO_OS_GENMASK(7, 4))
#define HMC6300_PA_SEL_VREF		HMC630X_PARAM(1, NO_OS_GENMASK(3, 0))

#define HMC6301_IPC_PWRDWN		HMC630X_PARAM(1, NO_OS_BIT(7))
#define HMC6301_IFMIX_PWRDN_Q		HMC630X_PARAM(1, NO_OS_BIT(6))
#define HMC6301_IF_BGMUX_PWRDN		HMC630X_PARAM(1, NO_OS_BIT(5))
#define HMC6301_ASK_PWRDN		HMC630X_PARAM(1, NO_OS_BIT(4))
#define HMC6301_BBAMP_ATTEN1		HMC630X_PARAM(1, NO_OS_GENMASK(3, 2))
#define HMC6301_BBAMP_SELL_ASK		HMC630X_PARAM(1, NO_OS_BIT(1))
#define HMC6301_BBAMP_SIGSHORT		HMC630X_PARAM(1, NO_OS_BIT(0))

/* ROW2 */
#define HMC6300_PA_SEL_ALC_DAC		HMC630X_PARAM(2, NO_OS_GENMASK(7, 4))
#define HMC6300_PA_SEP_PA_PWRDWN_FAST	HMC630X_PARAM(2, NO_OS_BIT(3))
#define HMC6300_PA_PWRDWN_FAST		HMC630X_PARAM(2, NO_OS_BIT(2))
#define HMC6300_PA_SE_SEL		HMC630X_PARAM(2, NO_OS_BIT(1))
#define HMC6300_POWER_DET_PWRDN		HMC630X_PARAM(2, NO_OS_BIT(0))

#define HMC6301_BBAMP_ATTENFI		HMC630X_PARAM(2, NO_OS_GENMASK(7, 5))
#define HMC6301_BBAMP_ATTENFQ		HMC630X_PARAM(2, NO_OS_GENMASK(4, 2))
#define HMC6301_BBAMP_ATTEN2		HMC630X_PARAM(2, NO_OS_GENMASK(1, 0))

/* ROW3 */
#define HMC6300_DRIVER_BIAS		HMC630X_PARAM(3, NO_OS_GENMASK(7, 5))
#define HMC6300_DRIVER_BIAS2		HMC630X_PARAM(3, NO_OS_GENMASK(4, 2))
#define HMC6300_EN_IFMIX_HICG		HMC630X_PARAM(3, NO_OS_BIT(1))
#define HMC6300_EN_TEMPFLASH		HMC630X_PARAM(3, NO_OS_BIT(0))

#define HMC6301_BBAMP_SELBW		HMC630X_PARAM(3, NO_OS_GENMASK(7, 6))
#define HMC6301_BBAMP_SELFASTREC	HMC630X_PARAM(3, NO_OS_GENMASK(5, 4))
#define HMC6301_BG_MONITOR_SEL		HMC630X_PARAM(3, NO_OS_GENMASK(3, 2))
#define HMC6301_IF_REFSEL		HMC630X_PARAM(3, NO_OS_BIT(1))
#define HMC6301_LNA_REFSEL		HMC630X_PARAM(3, NO_OS_BIT(0))

/* ROW4 */
#define HMC6300_DRIVER_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(7))
#define HMC6300_UPMIXER_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(6))
#define HMC6300_IFVGA_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(5))
#define HMC6300_DIVIDER_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(4))
#define HMC6300_PA_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(3))
#define HMC6300_RFVGA_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(2))
#define HMC6300_TRIPLER_PWRDN		HMC630X_PARAM(4, NO_OS_BIT(1))
#define HMC6300_IF_UPMIXER_PWRDN	HMC630X_PARAM(4, NO_OS_BIT(0))

#define HMC6301_IFVGA_BIAS		HMC630X_PARAM(4, NO_OS_GENMASK(7, 5))
#define HMC6301_IFVGA_TUNE		HMC630X_PARAM(4, NO_OS_GENMASK(4, 1))
#define HMC6301_ENDIGVGA		HMC630X_PARAM(4, NO_OS_BIT(0))

/* ROW5 */
#define HMC6300_TRIPLER_BIAS_HIGH	HMC630X_PARAM(5, NO_OS_GENMASK(7, 0))

#define HMC6301_IFVGA_VGA_ADJ		HMC630X_PARAM(5, NO_OS_GENMASK(7, 4))
#define HMC6301_RFMIX_TUNE		HMC630X_PARAM(5, NO_OS_GENMASK(3, 0))

/* ROW6 */
#define HMC6300_TRIPLER_BIAS_LOW	HMC630X_PARAM(6, NO_OS_GENMASK(7, 2))

#define HMC6301_TRIPLER_BIAS_HIGH	HMC630X_PARAM(6, NO_OS_GENMASK(7, 0))

/* ROW7 */
#define HMC6300_IFVGA_VGA_ADJ		HMC630X_PARAM(7, NO_OS_GENMASK(7, 4))
#define HMC6300_IFVGA_TUNE		HMC630X_PARAM(7, NO_OS_GENMASK(3, 0))

#define HMC6301_TRIPLER_BIAS_LOW	HMC630X_PARAM(7, NO_OS_GENMASK(7, 2))
#define HMC6301_BBAMP_SELFM		HMC630X_PARAM(7, NO_OS_BIT(1))
#define HMC6301_FM_PWRDN		HMC630X_PARAM(7, NO_OS_BIT(0))

/* ROW8 */
#define HMC6300_IFVGA_BIAS		HMC630X_PARAM(8, NO_OS_GENMASK(7, 4))
#define HMC6300_IF_UPMIXER_TUNE		HMC630X_PARAM(8, NO_OS_GENMASK(3, 0))

#define HMC6301_LNA_BIAS		HMC630X_PARAM(8, NO_OS_GENMASK(7, 5))
#define HMC6301_LNA_GAIN		HMC630X_PARAM(8, NO_OS_GENMASK(4, 3))
#define HMC6301_IFVGA_Q_CNTRL		HMC630X_PARAM(8, NO_OS_GENMASK(2, 0))

/* ROW9*/
#define HMC6300_IFVGA_Q_CNTRL		HMC630X_PARAM(9, NO_OS_GENMASK(7, 5))

#define HMC6301_ENANAV_LNA		HMC630X_PARAM(9, NO_OS_BIT(7))
#define HMC6301_ENBAR_TEMPS		HMC630X_PARAM(9, NO_OS_BIT(6))
#define HMC6301_EN_TEMPFLASH		HMC630X_PARAM(9, NO_OS_BIT(5))
#define HMC6301_EN_SEP_IFMIX_PWRDN_Q	HMC630X_PARAM(9, NO_OS_BIT(4))

/* ROW10 */
#define HMC6300_ENABLE_FM		HMC630X_PARAM(10, NO_OS_BIT(7))
#define HMC6300_IF_REFSEL		HMC630X_PARAM(10, NO_OS_BIT(6))
#define HMC6300_BG_MONITOR		HMC630X_PARAM(10, NO_OS_BIT(5))
#define HMC6300_ENDIG_IFVGA_GAIN_CONTROL HMC630X_PARAM(10, NO_OS_BIT(4))
#define HMC6300_IPC_PWRDN		HMC630X_PARAM(10, NO_OS_BIT(3))
#define HMC6300_IF_BGMUX_PWRDN		HMC630X_PARAM(10, NO_OS_BIT(2))
#define HMC6300_UPMIX_CAL_PWRDN		HMC630X_PARAM(10, NO_OS_BIT(1))
#define HMC6300_TEMPSENSOR_PWRDN	HMC630X_PARAM(10, NO_OS_BIT(0))

/* ROW11 */
#define HMC6300_RFVGAGAIN		HMC630X_PARAM(11, NO_OS_GENMASK(7, 4))
#define HMC6300_ENRFVGA_ANA		HMC630X_PARAM(11, NO_OS_BIT(3))
#define HMC6300_RFVGA_ICTRL		HMC630X_PARAM(11, NO_OS_GENMASK(2, 0))

/* ROW12 */
#define HMC6300_UPMIX_CAL		HMC630X_PARAM(12, NO_OS_GENMASK(7, 0))

/* ROW16 */
#define HMC630X_BYP_SYNTH_LDO		HMC630X_PARAM(16, NO_OS_BIT(7))
#define HMC630X_EN_CPSHORT		HMC630X_PARAM(16, NO_OS_BIT(6))
#define HMC630X_EN_CPCMFB		HMC630X_PARAM(16, NO_OS_BIT(5))
#define HMC630X_EN_CP_DUMP		HMC630X_PARAM(16, NO_OS_BIT(4))
#define HMC630X_EN_CPTRIST		HMC630X_PARAM(16, NO_OS_BIT(3))
#define HMC630X_EN_CP			HMC630X_PARAM(16, NO_OS_BIT(2))
#define HMC630X_EN_SYNTH_LDO		HMC630X_PARAM(16, NO_OS_BIT(1))
#define HMC630X_ENBAR_SYNTHBG		HMC630X_PARAM(16, NO_OS_BIT(0))

/* ROW17 */
#define HMC630X_EN_LOCKD_CLK		HMC630X_PARAM(17, NO_OS_BIT(7))
#define HMC630X_EN_TEST_DIVOUT		HMC630X_PARAM(17, NO_OS_BIT(6))
#define HMC630X_EN_VTUNE_FLASH		HMC630X_PARAM(17, NO_OS_BIT(5))
#define HMC630X_EN_REBUF_DC		HMC630X_PARAM(17, NO_OS_BIT(4))
#define HMC630X_EN_REFBUF		HMC630X_PARAM(17, NO_OS_BIT(3))
#define HMC630X_EN_STICK_DIV		HMC630X_PARAM(17, NO_OS_BIT(2))
#define HMC630X_EN_FBDIV_CML2CMOS	HMC630X_PARAM(17, NO_OS_BIT(1))
#define HMC630X_EN_FBDIV		HMC630X_PARAM(17, NO_OS_BIT(0))

/* ROW18 */
#define HMC630X_EN_NB250M		HMC630X_PARAM(18, NO_OS_BIT(6))
#define HMC630X_BYP_VCO_LDO		HMC630X_PARAM(18, NO_OS_BIT(5))
#define HMC630X_EN_EXTLO		HMC630X_PARAM(18, NO_OS_BIT(4))
#define HMC630X_EN_VCOPK		HMC630X_PARAM(18, NO_OS_BIT(3))
#define HMC630X_EN_VCO			HMC630X_PARAM(18, NO_OS_BIT(2))
#define HMC630X_EN_VCO_REG		HMC630X_PARAM(18, NO_OS_BIT(1))
#define HMC630X_ENBAR_VCOGB		HMC630X_PARAM(18, NO_OS_BIT(0))

/* ROW19 */
#define HMC630X_REFSEL_SYNTHBG		HMC630X_PARAM(19, NO_OS_BIT(1))
#define HMC630X_MUXREF			HMC630X_PARAM(19, NO_OS_BIT(0))

/* ROW20 */
#define HMC630X_FBDIV_CODE		HMC630X_PARAM(20, NO_OS_GENMASK(6, 0))

/* ROW21 */
#define HMC630X_REFSEL_VCOBG		HMC630X_PARAM(21, NO_OS_BIT(4))
#define HMC630X_VCO_BIASTRIM		HMC630X_PARAM(21, NO_OS_GENMASK(3, 0))

/* ROW22 */
#define HMC630X_VCO_BANDSEL		HMC630X_PARAM(22, NO_OS_GENMASK(6, 0))

/* ROW23 */
#define HMC630X_ICP_BIASTRIM		HMC630X_PARAM(23, NO_OS_GENMASK(7, 5))
#define HMC630X_VCO_OFFSET		HMC630X_PARAM(23, NO_OS_GENMASK(4, 0))

/* ROW24 */
#define HMC630X_LOCKDET			HMC630X_PARAM(24, NO_OS_BIT(3))
#define HMC630X_DN			HMC630X_PARAM(24, NO_OS_BIT(2))
#define HMC630X_UP			HMC630X_PARAM(24, NO_OS_BIT(1))
#define HMC630X_CENTER			HMC630X_PARAM(24, NO_OS_BIT(0))

/* ROW25 */
#define HMC630X_VTUNE_FLASHP		HMC630X_PARAM(25, NO_OS_GENMASK(7, 0))

/* ROW26 */
#define HMC630X_VTUNE_FLASHN		HMC630X_PARAM(26, NO_OS_GENMASK(7, 0))

/* ROW27 */
#define HMC630X_TEMPS			HMC630X_PARAM(27, NO_OS_GENMASK(4, 0))

/* Devices supported by this driver. */
/***************************************************************************//**
 * @brief The `hmc630x_type` enumeration defines the types of devices supported
 * by the driver, specifically the HMC6300 and HMC6301. These enumerators
 * are used to specify the device type when initializing or interacting
 * with the device driver, allowing the software to handle device-
 * specific configurations and operations.
 *
 * @param HMC6300 Represents the HMC6300 device type.
 * @param HMC6301 Represents the HMC6301 device type.
 ******************************************************************************/
enum hmc630x_type {
	HMC6300,
	HMC6301,
};

/* Possible values for receiver LNA attenuator. */
/***************************************************************************//**
 * @brief The `hmc6301_lna_attn` enumeration defines the possible attenuation
 * levels for the Low Noise Amplifier (LNA) in the HMC6301 device. It
 * provides four discrete attenuation settings: 0 dB, 6 dB, 12 dB, and 18
 * dB, which can be used to adjust the gain of the LNA to suit different
 * signal conditions.
 *
 * @param HMC6301_LNA_ATTN_0dB Represents 0 dB attenuation for the LNA.
 * @param HMC6301_LNA_ATTN_6dB Represents 6 dB attenuation for the LNA.
 * @param HMC6301_LNA_ATTN_12dB Represents 12 dB attenuation for the LNA.
 * @param HMC6301_LNA_ATTN_18dB Represents 18 dB attenuation for the LNA.
 ******************************************************************************/
enum hmc6301_lna_attn {
	HMC6301_LNA_ATTN_0dB,
	HMC6301_LNA_ATTN_6dB,
	HMC6301_LNA_ATTN_12dB,
	HMC6301_LNA_ATTN_18dB
};

/* Possible baseband attenuation values. */
/***************************************************************************//**
 * @brief The `hmc6301_bb_attn` enumeration defines possible baseband
 * attenuation levels for the HMC6301 device. These levels are used to
 * adjust the attenuation of the baseband signal, which can be crucial
 * for optimizing signal quality and performance in various applications.
 * The enumeration provides four distinct attenuation settings: 0 dB, 6
 * dB, 12 dB, and 18 dB, allowing for flexible control over the signal
 * attenuation.
 *
 * @param HMC6301_BB_ATTN_0dB Represents a baseband attenuation level of 0 dB.
 * @param HMC6301_BB_ATTN_12dB Represents a baseband attenuation level of 12 dB.
 * @param HMC6301_BB_ATTN_6dB Represents a baseband attenuation level of 6 dB.
 * @param HMC6301_BB_ATTN_18dB Represents a baseband attenuation level of 18 dB.
 ******************************************************************************/
enum hmc6301_bb_attn {
	HMC6301_BB_ATTN_0dB,
	HMC6301_BB_ATTN_12dB,
	HMC6301_BB_ATTN_6dB,
	HMC6301_BB_ATTN_18dB
};

/* Possible baseband fine attenuation adjustment values. */
/***************************************************************************//**
 * @brief The `hmc6301_bb_attn_fine` enumeration defines a set of constants
 * representing possible fine adjustments for baseband attenuation in the
 * HMC6301 device. These values allow for precise control over the
 * attenuation levels, ranging from 0 dB to 5 dB, with specific reserved
 * values that should not be used. This enumeration is part of the
 * configuration settings for the HMC6301, which is a device used in RF
 * applications.
 *
 * @param HMC6301_BB_ATTN_FINE_0dB Represents a fine baseband attenuation of 0
 * dB.
 * @param HMC6301_BB_ATTN_FINE_4dB Represents a fine baseband attenuation of 4
 * dB.
 * @param HMC6301_BB_ATTN_FINE_2dB Represents a fine baseband attenuation of 2
 * dB.
 * @param HMC6301_BB_ATTN_FINE_RESERVED1 Reserved value, not to be used.
 * @param HMC6301_BB_ATTN_FINE_1dB Represents a fine baseband attenuation of 1
 * dB.
 * @param HMC6301_BB_ATTN_FINE_5dB Represents a fine baseband attenuation of 5
 * dB.
 * @param HMC6301_BB_ATTN_FINE_3dB Represents a fine baseband attenuation of 3
 * dB.
 * @param HMC6301_BB_ATTN_FINE_RESERVED2 Reserved value, not to be used.
 ******************************************************************************/
enum hmc6301_bb_attn_fine {
	HMC6301_BB_ATTN_FINE_0dB,
	HMC6301_BB_ATTN_FINE_4dB,
	HMC6301_BB_ATTN_FINE_2dB,
	HMC6301_BB_ATTN_FINE_RESERVED1, /* Do not use. */
	HMC6301_BB_ATTN_FINE_1dB,
	HMC6301_BB_ATTN_FINE_5dB,
	HMC6301_BB_ATTN_FINE_3dB,
	HMC6301_BB_ATTN_FINE_RESERVED2 /* Do not use. */
};

/* Possible values for the low-pass corner of the baseband amplifiers. */
/***************************************************************************//**
 * @brief The `hmc6301_bb_lpc` enumeration defines possible low-pass corner
 * frequencies for the baseband amplifiers in the HMC6301 device. Each
 * enumerator corresponds to a specific frequency setting, allowing the
 * user to configure the baseband amplifier's low-pass filter to one of
 * the predefined frequencies: 1400 MHz, 300 MHz, 500 MHz, or 200 MHz.
 * This configuration is crucial for optimizing the performance of the
 * device in different operational scenarios.
 *
 * @param HMC6301_BB_LPC_1400MHz Represents a low-pass corner frequency of 1400
 * MHz for the baseband amplifiers.
 * @param HMC6301_BB_LPC_300MHz Represents a low-pass corner frequency of 300
 * MHz for the baseband amplifiers.
 * @param HMC6301_BB_LPC_500MHz Represents a low-pass corner frequency of 500
 * MHz for the baseband amplifiers.
 * @param HMC6301_BB_LPC_200MHz Represents a low-pass corner frequency of 200
 * MHz for the baseband amplifiers.
 ******************************************************************************/
enum hmc6301_bb_lpc {
	HMC6301_BB_LPC_1400MHz,
	HMC6301_BB_LPC_300MHz,
	HMC6301_BB_LPC_500MHz,
	HMC6301_BB_LPC_200MHz
};

/* Possible values for the high-pass corner of the baseband amplifiers. */
/***************************************************************************//**
 * @brief The `hmc6301_bb_hpc` enumeration defines possible high-pass corner
 * frequencies for the baseband amplifiers in the HMC6301 device. These
 * values are used to configure the high-pass filter characteristics of
 * the baseband amplifiers, allowing for different frequency responses
 * based on the application requirements. The enumeration includes
 * specific frequency options and a reserved value that is not intended
 * for use.
 *
 * @param HMC6301_BB_HPC_45kHz Represents a high-pass corner frequency of 45 kHz
 * for the baseband amplifiers.
 * @param HMC6301_BB_HPC_1600kHz Represents a high-pass corner frequency of 1600
 * kHz for the baseband amplifiers.
 * @param HMC6301_BB_HPC_350kHz Represents a high-pass corner frequency of 350
 * kHz for the baseband amplifiers.
 * @param HMC6301_BB_HPC_RESERVED A reserved value that should not be used.
 ******************************************************************************/
enum hmc6301_bb_hpc {
	HMC6301_BB_HPC_45kHz,
	HMC6301_BB_HPC_1600kHz,
	HMC6301_BB_HPC_350kHz,
	HMC6301_BB_HPC_RESERVED /* Do not use. */
};

/* Possible values for the external reference clock. */
/***************************************************************************//**
 * @brief The `hmc630x_ref_clk` enumeration defines a set of possible reference
 * clock frequencies for the HMC6300 and HMC6301 devices. These
 * frequencies are used to configure the reference clock input for the
 * devices, which is critical for their operation in various
 * applications. The enumeration provides a convenient way to select from
 * predefined clock frequencies, ensuring compatibility and ease of use
 * in the device's configuration.
 *
 * @param HMC6300_REF_CLK_71p42857MHz Represents a reference clock frequency of
 * 71.42857 MHz.
 * @param HMC6300_REF_CLK_75MHz Represents a reference clock frequency of 75
 * MHz.
 * @param HMC6300_REF_CLK_142p8571MHz Represents a reference clock frequency of
 * 142.8571 MHz.
 * @param HMC6300_REF_CLK_154p2857MHz Represents a reference clock frequency of
 * 154.2857 MHz.
 ******************************************************************************/
enum hmc630x_ref_clk {
	HMC6300_REF_CLK_71p42857MHz,
	HMC6300_REF_CLK_75MHz,
	HMC6300_REF_CLK_142p8571MHz,
	HMC6300_REF_CLK_154p2857MHz,
};

/***************************************************************************//**
 * @brief The `hmc6300_attr` structure is a simple data structure used to define
 * attributes specific to the HMC6300 device, particularly focusing on
 * the RF attenuation settings. It contains a single member, `rf_attn`,
 * which is an 8-bit unsigned integer that specifies the level of RF
 * attenuation to be applied. This structure is likely used in the
 * context of configuring or querying the RF attenuation settings of the
 * HMC6300 device within a larger driver or application.
 *
 * @param rf_attn This member represents the RF attenuation level as an 8-bit
 * unsigned integer.
 ******************************************************************************/
struct hmc6300_attr {
	uint8_t rf_attn;
};

/***************************************************************************//**
 * @brief The `hmc6301_attr` structure is used to configure various attenuation
 * and filter settings for the HMC6301 device. It includes fields for
 * setting baseband attenuation levels, both coarse and fine, for in-
 * phase and quadrature signals, as well as settings for the low-noise
 * amplifier attenuation and the low-pass and high-pass corner
 * frequencies of the baseband amplifiers. This structure is essential
 * for tailoring the device's performance to specific application
 * requirements.
 *
 * @param bb_attn1 Specifies the first baseband attenuation setting.
 * @param bb_attn2 Specifies the second baseband attenuation setting.
 * @param bb_attni_fine Specifies the fine adjustment for the in-phase baseband
 * attenuation.
 * @param bb_attnq_fine Specifies the fine adjustment for the quadrature
 * baseband attenuation.
 * @param lna_attn Specifies the low-noise amplifier attenuation setting.
 * @param bb_lpc Specifies the low-pass corner frequency for the baseband
 * amplifiers.
 * @param bb_hpc Specifies the high-pass corner frequency for the baseband
 * amplifiers.
 ******************************************************************************/
struct hmc6301_attr {
	enum hmc6301_bb_attn bb_attn1;
	enum hmc6301_bb_attn bb_attn2;
	enum hmc6301_bb_attn_fine bb_attni_fine;
	enum hmc6301_bb_attn_fine bb_attnq_fine;
	enum hmc6301_lna_attn lna_attn;
	enum hmc6301_bb_lpc bb_lpc;
	enum hmc6301_bb_hpc bb_hpc;
};

/* Initialization parameters for hmc6300_init(). */
/***************************************************************************//**
 * @brief The `hmc630x_init_param` structure is used to initialize the HMC630x
 * device, which can be either the HMC6300 or HMC6301. It includes
 * configuration parameters such as the device type, reference clock, and
 * various GPIO signals for the digital interface. Additionally, it holds
 * flags for enabling the device and its temperature sensor, as well as
 * settings for the VCO frequency and IF attenuation. The structure also
 * contains a union to store attributes specific to either the HMC6300
 * transmitter or the HMC6301 receiver, allowing for flexible
 * initialization based on the device type.
 *
 * @param type Specifies the type of HMC630x device, either HMC6300 or HMC6301.
 * @param ref_clk Defines the reference clock value for the device.
 * @param en Represents the EN GPIO signal of the digital interface.
 * @param clk Represents the CLK GPIO signal of the digital interface.
 * @param data Represents the DATA GPIO signal of the digital interface.
 * @param scanout Represents the SCANOUT GPIO signal of the digital interface.
 * @param enabled Indicates whether the device is enabled.
 * @param temp_en Indicates whether the temperature sensor is enabled.
 * @param vco Specifies the VCO frequency value.
 * @param if_attn Defines the IF attenuation value.
 * @param tx Contains attributes specific to the HMC6300 transmitter.
 * @param rx Contains attributes specific to the HMC6301 receiver.
 ******************************************************************************/
struct hmc630x_init_param {
	enum hmc630x_type type;
	enum hmc630x_ref_clk ref_clk; /* Reference clock value. */
	struct no_os_gpio_init_param en; /* EN GPIO signal of the digital interface. */
	struct no_os_gpio_init_param
		clk; /* CLK GPIO signal of the digital interface. */
	struct no_os_gpio_init_param
		data; /* DATA GPIO signal of the digital interface. */
	struct no_os_gpio_init_param
		scanout; /* SCANOUT GPIO signal of the digital interface. */
	bool enabled;
	bool temp_en;
	uint64_t vco;
	uint8_t if_attn;
	union {
		struct hmc6300_attr tx;
		struct hmc6301_attr rx;
	};
};

struct hmc630x_dev;
struct hmc630x_vco;

/* Device driver init/remove API. */
/***************************************************************************//**
 * @brief This function initializes an HMC630x device, either HMC6300 or
 * HMC6301, based on the provided initialization parameters. It must be
 * called before any other operations on the device. The function
 * configures the device's GPIOs, sets up the VCO, and applies the
 * initial configuration settings. It handles different device types and
 * reference clock settings, ensuring the device is ready for operation.
 * If initialization fails at any step, it cleans up allocated resources
 * and returns an error code.
 *
 * @param dev A pointer to a pointer of type `struct hmc630x_dev`. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init A pointer to a `struct hmc630x_init_param` containing
 * initialization parameters such as device type, reference clock,
 * and GPIO configurations. Must not be null. The structure should
 * be fully populated with valid values before calling this
 * function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as -EINVAL for
 * invalid parameters or -ENOMEM for memory allocation failure.
 ******************************************************************************/
int hmc630x_init(struct hmc630x_dev **dev, struct hmc630x_init_param *init);
/***************************************************************************//**
 * @brief Use this function to properly deallocate all resources associated with
 * an HMC630x device and remove it from the system. This function should
 * be called when the device is no longer needed to ensure that all
 * allocated memory and GPIO resources are freed. It is important to
 * ensure that the device pointer is valid and initialized before calling
 * this function. The function will return an error code if any of the
 * GPIO removals fail, otherwise it returns 0 indicating successful
 * removal.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. The function will handle invalid
 * pointers by potentially causing undefined behavior.
 * @return Returns 0 on successful removal of the device and its resources. If
 * any GPIO removal fails, it returns a non-zero error code.
 ******************************************************************************/
int hmc630x_remove(struct hmc630x_dev *dev);
enum hmc630x_type hmc630x_type(struct hmc630x_dev *dev);

/* Register access API. */
/***************************************************************************//**
 * @brief Use this function to write a specific value to a designated row in the
 * HMC630x device's register. This function is typically called when
 * configuring or updating the device's settings. It requires a valid
 * device structure pointer and specific row and value parameters. The
 * function must be called with a properly initialized device structure,
 * and it will return an error if the device pointer is null. Ensure that
 * the row and value parameters are within the valid range for the
 * device.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param row An 8-bit unsigned integer specifying the row in the device's
 * register to write to. Must be within the valid range of rows for
 * the device.
 * @param val An 8-bit unsigned integer representing the value to write to the
 * specified row. Must be within the valid range for the device.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int hmc630x_write_row(struct hmc630x_dev *dev, uint8_t row, uint8_t val);
/***************************************************************************//**
 * @brief This function is used to read a specific row from an HMC630x device
 * and store the retrieved value in the provided memory location. It is
 * essential to ensure that the device has been properly initialized
 * before calling this function. The function requires valid pointers for
 * both the device structure and the output value location; otherwise, it
 * will return an error. This function is typically used when accessing
 * configuration or status information from the device.
 *
 * @param dev A pointer to an initialized hmc630x_dev structure representing the
 * device. Must not be null.
 * @param row An 8-bit unsigned integer specifying the row to read from the
 * device. Valid row numbers depend on the device's configuration.
 * @param val A pointer to an 8-bit unsigned integer where the read value will
 * be stored. Must not be null.
 * @return Returns 0 on success. Returns -EINVAL if either 'dev' or 'val' is
 * null.
 ******************************************************************************/
int hmc630x_read_row(struct hmc630x_dev *dev, uint8_t row, uint8_t *val);
/***************************************************************************//**
 * @brief This function is used to write a specific value to a parameter of the
 * HMC630x device, identified by the parameter code. It should be called
 * when you need to configure or modify the settings of the device. The
 * function requires a valid device structure and a parameter code that
 * specifies the target register and field. It handles reading the
 * current register value, modifying the specified field, and writing the
 * updated value back. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param param A 16-bit code that specifies the target register and field
 * within the device. The high byte indicates the row, and the low
 * byte specifies the field mask.
 * @param value An 8-bit value to be written to the specified field of the
 * parameter. The value is masked and prepared according to the
 * field's mask.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int hmc630x_write(struct hmc630x_dev *dev, uint16_t param, uint8_t val);
/***************************************************************************//**
 * @brief Use this function to read a specific parameter from an HMC630x device,
 * identified by the `param` argument. This function is typically called
 * after the device has been properly initialized and configured. It
 * requires a valid device structure and a non-null pointer to store the
 * read value. If the `value` pointer is null, the function will return
 * an error. The function extracts the relevant field from the device's
 * register and stores it in the provided location.
 *
 * @param dev A pointer to an initialized `hmc630x_dev` structure representing
 * the device. The device must be properly initialized before calling
 * this function.
 * @param param A 16-bit unsigned integer representing the parameter to be read.
 * The parameter encodes both the row and the field mask within the
 * device's register map.
 * @param value A pointer to an 8-bit unsigned integer where the read value will
 * be stored. Must not be null. If null, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The read value is stored in the location pointed to by
 * `value`.
 ******************************************************************************/
int hmc630x_read(struct hmc630x_dev *dev, uint16_t param, uint8_t *val);
/***************************************************************************//**
 * @brief This function writes a complete register map to the specified HMC630x
 * device, skipping certain registers based on the device type. It is
 * typically used to configure the device with a predefined set of
 * register values. The function must be called with a valid device
 * structure and a non-null register map. It performs a write and read-
 * back verification for each register to ensure the write operation was
 * successful. If any write or verification fails, the function returns
 * an error code.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device to
 * which the register map will be written. Must not be null.
 * @param regmap A pointer to an array of uint8_t representing the register map
 * to be written. The array must contain at least 24 elements.
 * Must not be null.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error encountered during the write or
 * verification process.
 ******************************************************************************/
int hmc630x_write_regmap(struct hmc630x_dev *dev, const uint8_t *regmap);
/***************************************************************************//**
 * @brief This function retrieves the register map of a specified HMC630x device
 * and stores it in the provided buffer. It should be called when the
 * current state of the device's registers needs to be known. The
 * function iterates over the register rows, skipping certain rows based
 * on the device type, and reads the values into the buffer. It is
 * important to ensure that the device has been properly initialized
 * before calling this function. The function returns an error code if
 * any read operation fails, allowing the caller to handle such cases
 * appropriately.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device should be properly initialized before
 * use.
 * @param regmap A pointer to a buffer where the register map will be stored.
 * Must not be null. The buffer should be large enough to hold the
 * register map data.
 * @return Returns 0 on success or a negative error code if a read operation
 * fails.
 ******************************************************************************/
int hmc630x_read_regmap(struct hmc630x_dev *dev, uint8_t *regmap);

/* hmc6300/hmc6301 API. */
/***************************************************************************//**
 * @brief This function is used to control the temperature sensor feature of an
 * HMC630x device, which can be either an HMC6300 or HMC6301. It should
 * be called when there is a need to enable or disable the temperature
 * sensor functionality, depending on the operational requirements. The
 * function must be called with a valid device structure that has been
 * properly initialized. It returns an error code if the operation fails,
 * which should be checked by the caller to ensure successful execution.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * This must not be null and should be initialized before calling
 * this function. The caller retains ownership.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the temperature sensor.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation did not complete as expected.
 ******************************************************************************/
int hmc630x_set_temp_en(struct hmc630x_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function checks whether the temperature sensor is enabled on the
 * specified HMC630x device. It should be called when you need to verify
 * the current status of the temperature sensor. The function requires a
 * valid device structure and a pointer to a boolean variable where the
 * result will be stored. It returns an error code if the read operation
 * fails, otherwise it returns 0 indicating success.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device should be properly initialized before
 * calling this function.
 * @param enable A pointer to a boolean variable where the enable status will be
 * stored. Must not be null. The function writes true if the
 * temperature sensor is enabled, false otherwise.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int hmc630x_get_temp_en(struct hmc630x_dev *dev, bool *enable);
/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from an
 * HMC630x device. It is essential to ensure that the device has been
 * properly initialized and is operational before calling this function.
 * The function will store the temperature value in the provided memory
 * location. This function is useful for monitoring the device's thermal
 * state and ensuring it operates within safe temperature limits.
 *
 * @param dev A pointer to an initialized `hmc630x_dev` structure representing
 * the device. Must not be null. The device should be properly
 * configured and enabled before calling this function.
 * @param temp A pointer to a `uint8_t` where the temperature value will be
 * stored. Must not be null. The caller is responsible for providing
 * a valid memory location.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the operation.
 ******************************************************************************/
int hmc630x_get_temp(struct hmc630x_dev *dev, uint8_t *temp);
/***************************************************************************//**
 * @brief This function is used to enable or disable the HMC630x device, which
 * can be either an HMC6300 or HMC6301. It should be called when you need
 * to change the operational state of the device. The function requires a
 * valid device structure and a boolean indicating the desired state. It
 * returns an error code if the operation fails, which can occur if the
 * device type is not recognized or if there is a communication error.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be properly initialized before
 * calling this function.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the device.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered.
 ******************************************************************************/
int hmc630x_set_enable(struct hmc630x_dev *dev, bool enable);
/***************************************************************************//**
 * @brief This function checks the enable status of an HMC630x device and stores
 * the result in the provided boolean pointer. It should be called when
 * the enable status of the device needs to be verified. The function
 * requires a valid device structure and a non-null pointer to store the
 * enable status. It returns an error code if the operation fails,
 * ensuring that the enable status is only updated on success.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param enable A pointer to a boolean variable where the enable status will be
 * stored. Must not be null. The function will update this value
 * only if the operation is successful.
 * @return Returns 0 on success, or a negative error code on failure. The enable
 * status is stored in the provided boolean pointer if successful.
 ******************************************************************************/
int hmc630x_get_enable(struct hmc630x_dev *dev, bool *enable);
/***************************************************************************//**
 * @brief This function configures the intermediate frequency (IF) attenuation
 * level for a specified HMC630x device. It should be called when you
 * need to adjust the IF attenuation settings of the device, which can be
 * either an HMC6300 or HMC6301. The function requires a valid device
 * structure and an attenuation value. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function returns an integer status code indicating success or failure
 * of the operation.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device should be initialized before use.
 * @param attn A uint8_t value representing the desired attenuation level. The
 * valid range depends on the specific device and its configuration.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a negative value indicates an error.
 ******************************************************************************/
int hmc630x_set_if_attn(struct hmc630x_dev *dev, uint8_t attn);
/***************************************************************************//**
 * @brief This function is used to obtain the current intermediate frequency
 * (IF) attenuation setting from a specified HMC630x device. It should be
 * called when the user needs to know the current IF attenuation value
 * for either the HMC6300 or HMC6301 device. The function requires a
 * valid device structure that has been initialized and configured. The
 * attenuation value is returned through a pointer provided by the
 * caller. Ensure that the device is properly initialized before calling
 * this function to avoid undefined behavior.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param attn A pointer to a uint8_t where the attenuation value will be
 * stored. This pointer must not be null, and the caller is
 * responsible for allocating memory for it.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the operation.
 ******************************************************************************/
int hmc630x_get_if_attn(struct hmc630x_dev *dev, uint8_t *attn);
/***************************************************************************//**
 * @brief This function configures the Voltage Controlled Oscillator (VCO) of an
 * HMC630x device to operate at a specified frequency. It should be
 * called when a specific frequency operation is required, and the
 * frequency must be one of the available V-band frequencies supported by
 * the device. The function attempts to lock the Phase-Locked Loop (PLL)
 * to the desired frequency and returns an error if the frequency is not
 * supported or if the PLL fails to lock. It is essential to ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null, and the device must be initialized.
 * @param frequency The desired frequency in Hz to set the VCO to. Must be a
 * valid frequency from the device's available V-band
 * frequencies list.
 * @return Returns 0 on success, indicating the PLL locked to the desired
 * frequency. Returns a negative error code if the frequency is not
 * available or if the PLL fails to lock.
 ******************************************************************************/
int hmc630x_set_vco(struct hmc630x_dev *dev, uint64_t frequency);
/***************************************************************************//**
 * @brief This function is used to obtain the current Voltage Controlled
 * Oscillator (VCO) frequency from a specified HMC630x device. It should
 * be called when the user needs to know the current frequency setting of
 * the VCO. The function requires a valid device structure and a pointer
 * to store the frequency. If the device is not locked, or if an invalid
 * feedback divider is detected, the frequency will be set to zero. The
 * function must be called with a properly initialized device structure,
 * and the frequency pointer must not be null.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device should be properly initialized before
 * calling this function.
 * @param frequency A pointer to a uint64_t variable where the frequency will be
 * stored. Must not be null. If the device is not locked or an
 * invalid feedback divider is detected, the frequency will be
 * set to zero.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 * The frequency is written to the location pointed to by the frequency
 * parameter.
 ******************************************************************************/
int hmc630x_get_vco(struct hmc630x_dev *dev, uint64_t *frequency);
/***************************************************************************//**
 * @brief Use this function to obtain a list of available Voltage-Controlled
 * Oscillator (VCO) frequencies and the number of these frequencies for a
 * given HMC630x device. This function is essential when you need to know
 * the possible VCO frequencies that can be set on the device. It must be
 * called with valid pointers for the device, the frequency list, and the
 * count. If any of these pointers are null, the function will return an
 * error code.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null.
 * @param avail A pointer to a location where the function will store the
 * address of the array of available VCO frequencies. Must not be
 * null.
 * @param avail_num A pointer to a uint8_t where the function will store the
 * number of available VCO frequencies. Must not be null.
 * @return Returns 0 on success, or a negative error code if any input pointers
 * are null.
 ******************************************************************************/
int hmc630x_get_avail_vco(struct hmc630x_dev *dev, const uint64_t **avail,
			  uint8_t *avail_num);

/* hmc6300-only API. */
/***************************************************************************//**
 * @brief Use this function to enable or disable the FM functionality on a
 * device of type HMC6300. This function should be called only for
 * devices that are specifically of type HMC6300, as it is a transmitter-
 * specific feature. Ensure that the device pointer is valid and
 * initialized before calling this function. If the device is not of type
 * HMC6300, the function will return an error indicating that the
 * operation is not supported.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null and must point to a valid, initialized device of
 * type HMC6300. If null, the function returns an error.
 * @param enable A boolean value indicating whether to enable (true) or disable
 * (false) the FM functionality.
 * @return Returns 0 on success. If the device pointer is null, returns -EINVAL.
 * If the device is not of type HMC6300, returns -ENOSYS.
 ******************************************************************************/
int hmc6300_set_fm_en(struct hmc630x_dev *dev, bool enable);
/***************************************************************************//**
 * @brief Use this function to check if the FM feature is enabled on an HMC6300
 * device. It is specifically applicable to HMC6300 devices and will not
 * work with HMC6301 devices. Ensure that the device pointer is valid
 * before calling this function. The function will return an error if the
 * device type is not HMC6300 or if the device pointer is null.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The function will return -EINVAL if this
 * parameter is null.
 * @param enable A pointer to a boolean variable where the FM enable status will
 * be stored. The caller must provide a valid pointer, and the
 * function will write the result to this location.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, and
 * -ENOSYS if the device is not of type HMC6300.
 ******************************************************************************/
int hmc6300_get_fm_en(struct hmc630x_dev *dev, bool *enable);
/***************************************************************************//**
 * @brief This function sets the RF attenuation level for a device of type
 * HMC6300. It should be called when you need to adjust the RF output
 * power of the transmitter. The function requires a valid device pointer
 * and is only applicable to devices of type HMC6300. If the device
 * pointer is null or the device type is not HMC6300, the function will
 * return an error code. Ensure the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6300.
 * @param attn An 8-bit unsigned integer representing the desired RF attenuation
 * level. The valid range is determined by the device's
 * capabilities.
 * @return Returns 0 on success. Returns -EINVAL if the device pointer is null,
 * and -ENOSYS if the device type is not HMC6300.
 ******************************************************************************/
int hmc6300_set_rf_attn(struct hmc630x_dev *dev, uint8_t attn);
/***************************************************************************//**
 * @brief Use this function to obtain the current RF attenuation setting from an
 * HMC6300 device. It is specifically designed for the HMC6300
 * transmitter and will not work with other device types. Ensure that the
 * device is properly initialized and that the device type is HMC6300
 * before calling this function. The function will return an error if the
 * device pointer is null or if the device type is not HMC6300.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6300.
 * @param attn A pointer to a uint8_t where the RF attenuation value will be
 * stored. Must not be null.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, and
 * -ENOSYS if the device type is not HMC6300.
 ******************************************************************************/
int hmc6300_get_rf_attn(struct hmc630x_dev *dev, uint8_t *attn);

/* hmc6301-only API. */
/***************************************************************************//**
 * @brief Use this function to configure the Low Noise Amplifier (LNA) gain for
 * a device of type HMC6301. This function should be called only for
 * devices that are confirmed to be of type HMC6301, as it is specific to
 * this receiver type. Ensure that the device pointer is valid and not
 * null before calling this function. If the device type is not HMC6301,
 * the function will return an error indicating that the operation is not
 * supported. This function is useful for adjusting the gain settings to
 * optimize signal reception.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6301, otherwise
 * the function will return an error.
 * @param gain An enum value of type hmc6301_lna_attn representing the desired
 * LNA gain setting. Valid values are HMC6301_LNA_ATTN_0dB,
 * HMC6301_LNA_ATTN_6dB, HMC6301_LNA_ATTN_12dB, and
 * HMC6301_LNA_ATTN_18dB.
 * @return Returns 0 on success. Returns -EINVAL if the device pointer is null,
 * and -ENOSYS if the device type is not HMC6301.
 ******************************************************************************/
int hmc6301_set_lna_gain(struct hmc630x_dev *dev, enum hmc6301_lna_attn gain);
/***************************************************************************//**
 * @brief Use this function to obtain the current low-noise amplifier (LNA) gain
 * setting of an HMC6301 device. It is essential to ensure that the
 * device is of type HMC6301 before calling this function, as it is
 * specific to this device type. The function requires a valid device
 * structure and a pointer to store the gain value. If the device is not
 * initialized or is of an incorrect type, the function will return an
 * error code.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null and must be initialized for an HMC6301 device. If
 * null, the function returns -EINVAL.
 * @param gain A pointer to an enum hmc6301_lna_attn where the current LNA gain
 * setting will be stored. Must not be null. The function will write
 * the gain value to this location if successful.
 * @return Returns 0 on success, -EINVAL if the device pointer is null, and
 * -ENOSYS if the device is not of type HMC6301.
 ******************************************************************************/
int hmc6301_get_lna_gain(struct hmc630x_dev *dev, enum hmc6301_lna_attn *gain);
/***************************************************************************//**
 * @brief This function configures the baseband attenuation levels for the
 * HMC6301 device, which is a specific type of receiver. It should be
 * called only for devices of type HMC6301, and it requires a valid
 * device structure. The function sets two attenuation levels, and it is
 * important to ensure that the provided attenuation values are within
 * the allowed range. If the device is not of type HMC6301 or if invalid
 * attenuation values are provided, the function will return an error
 * code.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null and must point to a device of type HMC6301. If
 * null or not of type HMC6301, the function returns an error.
 * @param attn1 An enum hmc6301_bb_attn value representing the first baseband
 * attenuation level. Must be one of the defined values in the
 * enum, up to HMC6301_BB_ATTN_18dB. Invalid values result in an
 * error.
 * @param attn2 An enum hmc6301_bb_attn value representing the second baseband
 * attenuation level. Must be one of the defined values in the
 * enum, up to HMC6301_BB_ATTN_18dB. Invalid values result in an
 * error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error, such as -EINVAL for invalid parameters
 * or -ENOSYS if the device type is not HMC6301.
 ******************************************************************************/
int hmc6301_set_bb_attn(struct hmc630x_dev *dev, enum hmc6301_bb_attn attn1,
			enum hmc6301_bb_attn attn2);
/***************************************************************************//**
 * @brief Use this function to obtain the current baseband attenuation settings
 * for an HMC6301 device. It must be called with a valid device structure
 * that has been initialized and configured as an HMC6301 type. The
 * function will populate the provided pointers with the attenuation
 * values for two channels. If the device is not of type HMC6301, the
 * function will return an error. Ensure that the device pointer is not
 * null before calling this function.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null and must be initialized as an HMC6301 type.
 * @param attn1 A pointer to an enum hmc6301_bb_attn variable where the
 * attenuation value for the first channel will be stored. Must not
 * be null.
 * @param attn2 A pointer to an enum hmc6301_bb_attn variable where the
 * attenuation value for the second channel will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * of type HMC6301 or if any other error occurs during the operation.
 ******************************************************************************/
int hmc6301_get_bb_attn(struct hmc630x_dev *dev, enum hmc6301_bb_attn *attn1,
			enum hmc6301_bb_attn *attn2);
/***************************************************************************//**
 * @brief Use this function to adjust the fine baseband attenuation levels for
 * the I and Q channels of an HMC6301 device. This function should only
 * be called for devices of type HMC6301, as it is specific to this
 * receiver. Ensure that the device pointer is valid and that the
 * attenuation values provided are not reserved values, as these will
 * result in an error. The function returns an error code if the device
 * type is incorrect, the device pointer is null, or if invalid
 * attenuation values are provided.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6301.
 * @param attn_i An enum value of type hmc6301_bb_attn_fine representing the
 * fine attenuation level for the I channel. Must not be a
 * reserved value.
 * @param attn_q An enum value of type hmc6301_bb_attn_fine representing the
 * fine attenuation level for the Q channel. Must not be a
 * reserved value.
 * @return Returns 0 on success, -EINVAL if the device pointer is null or if
 * invalid attenuation values are provided, and -ENOSYS if the device
 * type is not HMC6301.
 ******************************************************************************/
int hmc6301_set_bb_attn_fine(struct hmc630x_dev *dev,
			     enum hmc6301_bb_attn_fine attn_i,
			     enum hmc6301_bb_attn_fine attn_q);
/***************************************************************************//**
 * @brief Use this function to obtain the current fine baseband attenuation
 * settings for the I and Q channels of an HMC6301 device. This function
 * should be called only for devices of type HMC6301, as it is specific
 * to this receiver. Ensure that the device has been properly initialized
 * before calling this function. If the device is not of type HMC6301,
 * the function will return an error indicating that the operation is not
 * supported. Additionally, the function will return an error if the
 * device pointer is null.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6301.
 * @param attn_i A pointer to an enum hmc6301_bb_attn_fine variable where the
 * fine attenuation setting for the I channel will be stored. Must
 * not be null.
 * @param attn_q A pointer to an enum hmc6301_bb_attn_fine variable where the
 * fine attenuation setting for the Q channel will be stored. Must
 * not be null.
 * @return Returns 0 on success. Returns -EINVAL if the device pointer is null,
 * and -ENOSYS if the device is not of type HMC6301. Other negative
 * error codes may be returned if reading the attenuation settings
 * fails.
 ******************************************************************************/
int hmc6301_get_bb_attn_fine(struct hmc630x_dev *dev,
			     enum hmc6301_bb_attn_fine *attn_i,
			     enum hmc6301_bb_attn_fine *attn_q);
/***************************************************************************//**
 * @brief This function configures the low-pass and high-pass corner frequencies
 * of the baseband amplifiers for an HMC6301 device. It should be called
 * only for devices of type HMC6301, as it is specific to this receiver.
 * The function requires a valid device structure and appropriate
 * enumerated values for the low-pass and high-pass corner frequencies.
 * If the device is not of type HMC6301, the function will return an
 * error indicating that the operation is not supported. Ensure the
 * device is properly initialized before calling this function.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6301, otherwise
 * the function will return an error.
 * @param lpc An enumerated value of type hmc6301_bb_lpc representing the
 * desired low-pass corner frequency. Must be a valid enum value.
 * @param hpc An enumerated value of type hmc6301_bb_hpc representing the
 * desired high-pass corner frequency. Must be a valid enum value.
 * @return Returns 0 on success. Returns -EINVAL if the device pointer is null,
 * and -ENOSYS if the device is not of type HMC6301. Other negative
 * error codes may be returned for write failures.
 ******************************************************************************/
int hmc6301_set_bb_lpc_hpc(struct hmc630x_dev *dev, enum hmc6301_bb_lpc lpc,
			   enum hmc6301_bb_hpc hpc);
/***************************************************************************//**
 * @brief Use this function to obtain the current low-pass and high-pass corner
 * frequency settings of the baseband amplifiers in an HMC6301 device.
 * This function should only be called for devices of type HMC6301, as it
 * is specific to this receiver. Ensure that the device has been properly
 * initialized before calling this function. If the device is not of type
 * HMC6301, the function will return an error indicating that the
 * operation is not supported.
 *
 * @param dev A pointer to an hmc630x_dev structure representing the device.
 * Must not be null. The device must be of type HMC6301, otherwise
 * the function will return an error.
 * @param lpc A pointer to an enum hmc6301_bb_lpc variable where the low-pass
 * corner frequency will be stored. Must not be null.
 * @param hpc A pointer to an enum hmc6301_bb_hpc variable where the high-pass
 * corner frequency will be stored. Must not be null.
 * @return Returns 0 on success. If the device is not of type HMC6301, returns
 * -ENOSYS. If the dev parameter is null, returns -EINVAL. Other
 * negative error codes may be returned in case of read failures.
 ******************************************************************************/
int hmc6301_get_bb_lpc_hpc(struct hmc630x_dev *dev, enum hmc6301_bb_lpc *lpc,
			   enum hmc6301_bb_hpc *hpc);

#endif
