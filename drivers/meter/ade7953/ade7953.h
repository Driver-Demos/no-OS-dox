/***************************************************************************//**
 *   @file   ade7953.h
 *   @brief  Header file of ADE7953 Driver.
 *   @author REtz (radu.etz@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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
#ifndef __ADE7953_H__
#define __ADE7953_H__

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"
#include "no_os_irq.h"
#include "no_os_delay.h"
#include "no_os_units.h"
#include "no_os_alloc.h"
#include "no_os_crc16.h"
#include "no_os_print_log.h"
#include <stdlib.h>
#include <errno.h>
#include <math.h>

/* SPI commands */
#define ADE7953_SPI_READ					NO_OS_BIT(7)

/* Version product */
#define ADE7953_VERSION						0x40
#define ADE7953_REG_VERSION_PRODUCT     			0x004

/* Miscellaneous Definitions */
#define IS_8BITS_REG(x) ((x >= ADE7953_REG_SAGCYC && x <= ADE7953_REG_LAST_RWDATA) \
        || (x == ADE7953_REG_VERSION) || (x == ADE7953_REG_EX_REF))
#define IS_16BITS_REG(x) ((x >= ADE7953_REG_ZXTOUT && x <= ADE7953_REG_LAST_RWDATA_16))

/* Key needed to unlock register 0x120 */
#define ADE7953_UNLOCK_KEY             				0xAD
/* Value that needs to be write to the 0x120 register */
#define ADE7953_OPT_SETT                			0x30

/* Reset Delay */
#define ADE7953_RESET_DEL					100

/* ENABLE and DISABLE */
#define ENABLE							1u
#define DISABLE							0u

/* ADE7953 Register Map */
/* 8 BIT REGISTERS */
#define ADE7953_REG_SAGCYC					0x000
#define ADE7953_REG_DISNOLOAD					0x001
#define ADE7953_REG_LCYCMODE					0x004
#define ADE7953_REG_PGA_V					0x007
#define ADE7953_REG_PGA_IA					0x008
#define ADE7953_REG_PGA_IB					0x009
#define ADE7953_REG_WRITE_PROTECT				0x040
#define ADE7953_REG_LAST_OP					0x0FD
#define ADE7953_REG_UNLOCK_120              			0x0FE
#define ADE7953_REG_LAST_RWDATA					0x0FF
#define ADE7953_REG_VERSION					0x702
#define ADE7953_REG_EX_REF					0x800
/* 16 BIT REGISTERS	*/
#define ADE7953_REG_ZXTOUT					0x100
#define ADE7953_REG_LINECYC					0x101
#define ADE7953_REG_CONFIG					0x102
#define ADE7953_REG_CF1DEN					0x103
#define ADE7953_REG_CF2DEN					0x104
#define ADE7953_REG_CFMODE					0x107
#define ADE7953_REG_PHCALA					0x108
#define ADE7953_REG_PHCALB					0x109
#define ADE7953_REG_PFA						0x10A
#define ADE7953_REG_PFB						0x10B
#define ADE7953_REG_ANGLE_A					0x10C
#define ADE7953_REG_ANGLE_B					0x10D
#define ADE7953_REG_PERIOD					0x10E
#define ADE7953_REG_ALT_OUTPUT					0x110
#define ADE7953_REG_LAST_ADD					0x1FE
#define ADE7953_REG_LAST_RWDATA_16				0x1FF
#define ADE7953_REG_RESERVED					0x120
/* 24 BIT REGISTERS */
#define ADE7953_REG_SAGLVL					0x200
#define ADE7953_REG_ACCMODE					0x201
#define ADE7953_REG_AP_NOLOAD					0x203
#define ADE7953_REG_VAR_NOLOAD					0x204
#define ADE7953_REG_VA_NOLOAD					0x205
#define ADE7953_REG_AVA						0x210
#define ADE7953_REG_BVA						0x211
#define ADE7953_REG_AWATT					0x212
#define ADE7953_REG_BWATT					0x213
#define ADE7953_REG_AVAR					0x214
#define ADE7953_REG_BVAR					0x215
#define ADE7953_REG_IA						0x216
#define ADE7953_REG_IB						0x217
#define ADE7953_REG_V						0x218
#define ADE7953_REG_IRMSA					0x21A
#define ADE7953_REG_IRMSB					0x21B
#define ADE7953_REG_VRMS					0x21C
#define ADE7953_REG_AENERGYA					0x21E
#define ADE7953_REG_AENERGYB					0x21F
#define ADE7953_REG_RENERGYA					0x220
#define ADE7953_REG_RENERGYB					0x221
#define ADE7953_REG_APENERGYA					0x222
#define ADE7953_REG_APENERGYB					0x223
#define ADE7953_REG_OVLVL					0x224
#define ADE7953_REG_OILVL					0x225
#define ADE7953_REG_VPEAK					0x226
#define ADE7953_REG_RSTVPEAK					0x227
#define ADE7953_REG_IAPEAK					0x228
#define ADE7953_REG_RSTIAPEAK					0x229
#define ADE7953_REG_IBPEAK					0x22A
#define ADE7953_REG_RSTIBPEAK					0x22B
#define ADE7953_REG_IRQENA					0x22C
#define ADE7953_REG_IRQSTATA					0x22D
#define ADE7953_REG_RSTIRQSTATA					0x22E
#define ADE7953_REG_IRQENB					0x22F
#define ADE7953_REG_IRQSTATB					0x230
#define ADE7953_REG_RSTIRQSTATB					0x231
#define ADE7953_REG_AIGAIN					0x280
#define ADE7953_REG_AVGAIN					0x281
#define ADE7953_REG_AWGAIN					0x282
#define ADE7953_REG_AVARGAIN					0x283
#define ADE7953_REG_AVAGAIN					0x284
#define ADE7953_REG_AIRMSOS					0x286
#define ADE7953_REG_VRMSOS					0x288
#define ADE7953_REG_AWATTOS					0x289
#define ADE7953_REG_AVAROS					0x28A
#define ADE7953_REG_AVAOS					0x28B
#define ADE7953_REG_BIGAIN					0x28C
#define ADE7953_REG_BVGAIN					0x28D
#define ADE7953_REG_BWGAIN					0x28E
#define ADE7953_REG_BVARGAIN					0x28F
#define ADE7953_REG_BVAGAIN					0x290
#define ADE7953_REG_BIRMSOS					0x292
#define ADE7953_REG_BWATTOS					0x295
#define ADE7953_REG_BVAROS					0x296
#define ADE7953_REG_BVAOS					0x297
#define ADE7953_REG_LAST_RWDATA_24				0x2FF
/* 32 BIT REGISTERS */
/* Same registers as 24 bit registers +0x100 address offset*/
#define ADE7953_REG_CRC						0x37F

/* ADE7953_REG_DISNOLOAD Bit Definition */
#define ADE7953_DIS_VANLOAD_MSK					NO_OS_BIT(2)
#define ADE7953_DIS_VARNLOAD_MSK				NO_OS_BIT(1)
#define ADE7953_DIS_APNLOAD_MSK					NO_OS_BIT(0)

/* ADE7953_REG_LCYCMODE Bit Definition */
#define ADE7953_RSTREAD_MSK					NO_OS_BIT(6)
#define ADE7953_BLVA_MSK					NO_OS_BIT(5)
#define ADE7953_ALVA_MSK					NO_OS_BIT(4)
#define ADE7953_BLVAR_MSK					NO_OS_BIT(3)
#define ADE7953_ALVAR_MSK					NO_OS_BIT(2)
#define ADE7953_BLWATT_MSK					NO_OS_BIT(1)
#define ADE7953_ALWATT_MSK					NO_OS_BIT(0)

/* ADE7953_REG_PGA_V Bit Definition */
#define ADE7953_PGA_V_MSK					NO_OS_GENMASK(2, 0)

/* ADE7953_REG_PGA_IA / ADE7953_REG_PGA_IB Bit Definition */
#define ADE7953_PGA_IA_MSK					NO_OS_GENMASK(2, 0)

/* ADE7953_REG_CONFIG Bit Definition */
#define ADE7953_COMM_LOCK_MSK					NO_OS_BIT(15)
#define ADE7953_ZX_EDGE_MSK					NO_OS_GENMASK(13, 12)
#define ADE7953_ZX_I_MSK					NO_OS_BIT(11)
#define ADE7953_CRC_ENABLE_MSK					NO_OS_BIT(8)
#define ADE7953_SWRST_MSK					NO_OS_BIT(7)
#define ADE7953_ZXLPF_MSK					NO_OS_BIT(6)
#define ADE7953_REV_PULSE_MSK					NO_OS_BIT(5)
#define ADE7953_REVP_CF_MSK					NO_OS_BIT(4)
#define ADE7953_PFMODE_MSK					NO_OS_BIT(3)
#define ADE7953_HPFEN_MSK					NO_OS_BIT(2)
#define ADE7953_INTENB_MSK					NO_OS_BIT(1)
#define ADE7953_INTENA_MSK					NO_OS_BIT(0)

/* ADE7953_REG_CFMODE Bit Definition */
#define ADE7953_CF2DIS_MSK					NO_OS_BIT(9)
#define ADE7953_CF1DIS_MSK					NO_OS_BIT(8)
#define ADE7953_CF2SEL_MSK					NO_OS_GENMASK(7, 4)
#define ADE7953_CF1SEL_MSK					NO_OS_GENMASK(3, 0)

/* ADE7953_REG_ALT_OUTPUT Bit Definition */
#define ADE7953_REVP_ALT_MSK					NO_OS_GENMASK(11, 8)
#define ADE7953_ZXI_ALT_MSK					NO_OS_GENMASK(7, 4)
#define ADE7953_ZX_ALT_MSK					NO_OS_GENMASK(3, 0)

/* ADE7953_REG_ACCMODE Bit Definition */
#define ADE7953_VARNLOAD_B_MSK					NO_OS_BIT(21)
#define ADE7953_VANLOAD_B_MSK					NO_OS_BIT(20)
#define ADE7953_ACTNLOAD_B_MSK					NO_OS_BIT(19)
#define ADE7953_VARNLOAD_A_MSK					NO_OS_BIT(18)
#define ADE7953_VANLOAD_A_MSK					NO_OS_BIT(17)
#define ADE7953_ACTNLOAD_A_MSK					NO_OS_BIT(16)
#define ADE7953_VARSIGN_B_MSK					NO_OS_BIT(13)
#define ADE7953_VARSIGN_A_MSK					NO_OS_BIT(12)
#define ADE7953_APSIGN_B_MSK					NO_OS_BIT(11)
#define ADE7953_APSIGN_A_MSK					NO_OS_BIT(10)
#define ADE7953_BVAACC_MSK					NO_OS_BIT(9)
#define ADE7953_AVAACC_MSK					NO_OS_BIT(8)
#define ADE7953_BVARACC_MSK					NO_OS_GENMASK(7, 6)
#define ADE7953_AVARACC_MSK					NO_OS_GENMASK(5, 4)
#define ADE7953_BWATTACC_MSK					NO_OS_GENMASK(3, 2)
#define ADE7953_AWATTACC_MSK					NO_OS_GENMASK(1, 0)

/* ADE7953_REG_IRQENA / ADE7953_REG_IRQSTATA Bit Definition */
#define ADE7953_CRC_MSK						NO_OS_BIT(21)
#define ADE7953_RESET_MSK					NO_OS_BIT(20)
#define ADE7953_SAG_MSK						NO_OS_BIT(19)
#define ADE7953_CYCEND_MSK					NO_OS_BIT(18)
#define ADE7953_WSMP_MSK					NO_OS_BIT(17)
#define ADE7953_OV_MSK						NO_OS_BIT(16)
#define ADE7953_ZXV_MSK						NO_OS_BIT(15)
#define ADE7953_ZXTO_MSK					NO_OS_BIT(14)
#define ADE7953_OIA_MSK						NO_OS_BIT(13)
#define ADE7953_ZXIA_MSK					NO_OS_BIT(12)
#define ADE7953_ZXTO_IA_MSK					NO_OS_BIT(11)
#define ADE7953_VARSIGN_A_MSK					NO_OS_BIT(10)
#define ADE7953_APSIGN_A_MSK					NO_OS_BIT(9)
#define ADE7953_VA_NOLOADA_MSK					NO_OS_BIT(8)
#define ADE7953_VAR_NOLOADA_MSK					NO_OS_BIT(7)
#define ADE7953_AP_NOLOADA_MSK					NO_OS_BIT(6)
#define ADE7953_VAEOFA_MSK					NO_OS_BIT(5)
#define ADE7953_VAREOFA_MSK					NO_OS_BIT(4)
#define ADE7953_AEOFA_MSK					NO_OS_BIT(3)
#define ADE7953_VAEHFA_MSK					NO_OS_BIT(2)
#define ADE7953_VAREHFA_MSK					NO_OS_BIT(1)
#define ADE7953_AEHFA_MSK					NO_OS_BIT(0)

/* ADE7953_REG_IRQENB / ADE7953_REG_IRQSTATB Bit Definition */
#define ADE7953_OIB_MSK						NO_OS_BIT(13)
#define ADE7953_ZXIB_MSK					NO_OS_BIT(12)
#define ADE7953_ZXTO_IB_MSK					NO_OS_BIT(11)
#define ADE7953_VARSIGN_B_MSK					NO_OS_BIT(10)
#define ADE7953_APSIGN_B_MSK					NO_OS_BIT(9)
#define ADE7953_VA_NOLOADB_MSK					NO_OS_BIT(8)
#define ADE7953_VAR_NOLOADB_MSK					NO_OS_BIT(7)
#define ADE7953_AP_NOLOADB_MSK					NO_OS_BIT(6)
#define ADE7953_VAEOFB_MSK					NO_OS_BIT(5)
#define ADE7953_VAREOFB_MSK					NO_OS_BIT(4)
#define ADE7953_AEOFB_MSK					NO_OS_BIT(3)
#define ADE7953_VAEHFB_MSK					NO_OS_BIT(2)
#define ADE7953_VAREHFB_MSK					NO_OS_BIT(1)
#define ADE7953_AEHFB_MSK					NO_OS_BIT(0)

/* ADE7953_REG_WRITE_PROTECT Bit Definition */
#define ADE7953_24_32_BITS_PROTECT            			NO_OS_BIT(2)
#define ADE7953_16_BITS_PROTECT                 		NO_OS_BIT(1)
#define ADE7953_8_BITS_PROTECT                			NO_OS_BIT(0)

/***************************************************************************//**
 * @brief The `ade7953_i_ch_e` enumeration defines the current channel selection
 * for the ADE7953 device, allowing the user to specify which current
 * channel (A or B) is being referenced or utilized in operations. This
 * is useful for distinguishing between the two current channels
 * available in the ADE7953 energy metering IC.
 *
 * @param ADE7953_I_CH1 Represents the current channel A.
 * @param ADE7953_I_CH2 Represents the current channel B.
 ******************************************************************************/
enum ade7953_i_ch_e {
	/* Current channel selection */
	/* current ch A */
	ADE7953_I_CH1,
	/* current ch B */
	ADE7953_I_CH2
};

/***************************************************************************//**
 * @brief The `ade7953_cf_pin_e` is an enumeration that defines the selection of
 * CF (Calibration Frequency) pins for the ADE7953 energy metering IC. It
 * provides two options, CF1 and CF2, which are used to select the
 * current channel for calibration frequency output. This enumeration is
 * part of the configuration settings for the ADE7953 device, allowing
 * users to specify which pin is used for outputting calibration
 * frequency signals.
 *
 * @param ADE7953_CF1_PIN Represents the CF1 pin for current channel selection.
 * @param ADE7953_CF2_PIN Represents the CF2 pin for current channel selection.
 ******************************************************************************/
enum ade7953_cf_pin_e {
	/* Current channel selection */
	/* CF 1 pin */
	ADE7953_CF1_PIN,
	/* CF 2 pin */
	ADE7953_CF2_PIN
};

/***************************************************************************//**
 * @brief The `ade7953_write_protect_e` is an enumeration that defines the
 * different levels of write protection available for the ADE7953
 * registers. It categorizes the registers into three groups based on
 * their bit-width: 8 bits, 16 bits, and 24/32 bits, allowing for
 * selective enabling of write protection on these registers to prevent
 * unauthorized modifications.
 *
 * @param ADE7953_8BITS_REGS Represents registers with 8-bit write protection.
 * @param ADE7953_16BITS_REGS Represents registers with 16-bit write protection.
 * @param ADE7953_24_32BITS_REGS Represents registers with 24/32-bit write
 * protection.
 ******************************************************************************/
enum ade7953_write_protect_e {
	/* Select the registers that have write protect enabled */
	/* 8 bits regs */
	ADE7953_8BITS_REGS,
	/* 16 bits regs */
	ADE7953_16BITS_REGS,
	/* 24/32 bits regs */
	ADE7953_24_32BITS_REGS
};

/***************************************************************************//**
 * @brief The `ade7953_zx_edge_e` is an enumeration that defines the different
 * edge selections for zero-crossing interrupts in the ADE7953 device. It
 * specifies the conditions under which an interrupt is triggered based
 * on the zero-crossing of the input signal, allowing for configuration
 * of interrupts on positive-going, negative-going, or both types of zero
 * crossings. This is useful for applications that need to detect and
 * respond to changes in the direction of current or voltage waveforms.
 *
 * @param ADE7953_ZX_BOTH_1 Interrupt is issued on both positive-going and
 * negative-going zero crossing.
 * @param ADE7953_ZX_NEG Interrupt is issued on negative-going zero crossing.
 * @param ADE7953_ZX_POS Interrupt is issued on positive-going zero crossing.
 * @param ADE7953_ZX_BOTH_2 Interrupt is issued on both positive-going and
 * negative-going zero crossing.
 ******************************************************************************/
enum ade7953_zx_edge_e {
	/* Zero-crossing interrupt edge selection */
	/* Interrupt is issued on both positive-going and
	negative-going zero crossing */
	ADE7953_ZX_BOTH_1,
	/* Interrupt is issued on negative-going zero crossing */
	ADE7953_ZX_NEG,
	/* Interrupt is issued on positive-going zero crossing */
	ADE7953_ZX_POS,
	/* Interrupt is issued on both positive-going and
	negative-going zero crossing */
	ADE7953_ZX_BOTH_2
};

/***************************************************************************//**
 * @brief The `ade7953_cfsel_e` enumeration defines the configuration options
 * for the output signal on the CF1/CF2 pins of the ADE7953 device. Each
 * enumerator represents a different proportional relationship between
 * the CF signal and various power or current measurements for either
 * Current Channel A or B, such as active power, reactive power, apparent
 * power, or IRMS. This allows for flexible configuration of the CF
 * output to match specific measurement needs in power monitoring
 * applications.
 *
 * @param CF_APA CF is proportional to active power (Current Channel A).
 * @param CF_RPA CF is proportional to reactive power (Current Channel A).
 * @param CF_APPA CF is proportional to apparent power (Current Channel A).
 * @param CF_IRMSA CF is proportional to IRMS (Current Channel A).
 * @param CF_BPB CF is proportional to active power (Current Channel B).
 * @param CF_RPB CF is proportional to reactive power (Current Channel B).
 * @param CF_APPB CF is proportional to apparent power (Current Channel B).
 * @param CF_IRMSB CF is proportional to IRMS (Current Channel B).
 * @param CF_IRMSA_IRMSB CF is proportional to IRMS (Current Channel A) + IRMS
 * (Current Channel B).
 * @param CF_APA_APB CF is proportional to active power (Current Channel A) +
 * active power (Current Channel B).
 ******************************************************************************/
enum ade7953_cfsel_e {
	/* CF is proportional to active power (Current Channel A). */
	CF_APA,
	/* CF is proportional to reactive power (Current Channel A). */
	CF_RPA,
	/* CF is proportional to apparent power (Current Channel A). */
	CF_APPA,
	/* CF is proportional to IRMS (Current Channel A). */
	CF_IRMSA,
	/* CF is proportional to active power (Current Channel B). */
	CF_BPB,
	/* CF is proportional to reactive power (Current Channel B). */
	CF_RPB,
	/* CF is proportional to apparent power (Current Channel B). */
	CF_APPB,
	/* CF is proportional to IRMS (Current Channel B). */
	CF_IRMSB,
	/* CF is proportional to
	IRMS (Current Channel A) + IRMS (Current Channel B). */
	CF_IRMSA_IRMSB,
	/* CF is proportional to
	active power (Current Channel A) + active power (Current Channel B). */
	CF_APA_APB
};

/***************************************************************************//**
 * @brief The `ade7953_zx_alt_e` enumeration defines the configuration options
 * for the ZX pin (Pin 1) on the ADE7953 device. Each enumerator
 * represents a different signal or detection type that can be output on
 * Pin 1, such as zero-crossing detection, sag detection, active and
 * reactive power no-load detection for different current channels,
 * waveform sampling, IRQ signals, and reverse power detection. This
 * enumeration allows for flexible configuration of the pin's output
 * based on the specific application requirements.
 *
 * @param ZX_ALT_ZX ZX detection is output on Pin 1 (default).
 * @param ZX_ALT_SAG Sag detection is output on Pin 1.
 * @param ZX_ALT_APNLOAD_A Active power no-load detection (Current Channel A) is
 * output on Pin 1.
 * @param ZX_ALT_APNLOAD_B Active power no-load detection (Current Channel B) is
 * output on Pin 1.
 * @param ZX_ALT_VARNLOAD_A Reactive power no-load detection (Current Channel A)
 * is output on Pin 1.
 * @param ZX_ALT_VARNLOAD_B Reactive power no-load detection (Current Channel B)
 * is output on Pin 1.
 * @param ZX_ALT_WSMP Unlatched waveform sampling signal is output on Pin 1.
 * @param ZX_ALT_IRQ IRQ signal is output on Pin 1.
 * @param ZX_ALT_ZX_I ZX_I detection is output on Pin 1.
 * @param ZX_ALT_REVP REVP detection is output on Pin 1.
 ******************************************************************************/
enum ade7953_zx_alt_e {
	/* ZX detection is output on Pin 1 (default) */
	ZX_ALT_ZX,
	/* Sag detection is output on Pin 1 */
	ZX_ALT_SAG,
	/* Active power no-load detection
	(Current Channel A) is output on Pin 1 */
	ZX_ALT_APNLOAD_A = 5,
	/* Active power no-load detection
	(Current Channel B) is output on Pin 1 */
	ZX_ALT_APNLOAD_B,
	/* Reactive power no-load detection
	(Current Channel A) is output on Pin 1 */
	ZX_ALT_VARNLOAD_A,
	/* Reactive power no-load detection
	(Current Channel B) is output on Pin 1 */
	ZX_ALT_VARNLOAD_B,
	/* Unlatched waveform sampling signal is output on Pin 1 */
	ZX_ALT_WSMP,
	/* IRQsignal is output on Pin 1 */
	ZX_ALT_IRQ,
	/* ZX_I detection is output on Pin 1 */
	ZX_ALT_ZX_I,
	/* REVP detection is output on Pin 1 */
	ZX_ALT_REVP
};

/***************************************************************************//**
 * @brief The `ade7953_zxi_alt_e` is an enumeration that defines the
 * configuration options for the ZX_I pin (Pin 21) on the ADE7953 device.
 * Each enumerator specifies a different signal or detection type that
 * can be output on Pin 21, such as ZXI detection, sag detection, active
 * or reactive power no-load detection for different current channels,
 * waveform sampling, IRQ signal, ZX detection, and REVP detection. This
 * allows for flexible configuration of the pin's functionality based on
 * the application's requirements.
 *
 * @param ZXI_ALT_ZX_I ZXI detection is output on Pin 21 (default).
 * @param ZXI_ALT_SAG Sag detection is output on Pin 21.
 * @param ZXI_ALT_APNLOAD_A Active power no-load detection (Current Channel A)
 * is output on Pin 21.
 * @param ZXI_ALT_APNLOAD_B Active power no-load detection (Current Channel B)
 * is output on Pin 21.
 * @param ZXI_ALT_VARNLOAD_A Reactive power no-load detection (Current Channel
 * A) is output on Pin 21.
 * @param ZXI_ALT_VARNLOAD_B Reactive power no-load detection (Current Channel
 * B) is output on Pin 21.
 * @param ZXI_ALT_WSMP Unlatched waveform sampling signal is output on Pin 21.
 * @param ZXI_ALT_IRQ IRQ signal is output on Pin 21.
 * @param ZXI_ALT_ZX ZX detection is output on Pin 21.
 * @param ZXI_ALT_REVP REVP detection is output on Pin 21.
 ******************************************************************************/
enum ade7953_zxi_alt_e {
	/* ZXI detection is output on Pin 21 (default) */
	ZXI_ALT_ZX_I,
	/* Sag detection is output on Pin 21 */
	ZXI_ALT_SAG,
	/* Active power no-load detection
	(Current Channel A) is output on Pin 21 */
	ZXI_ALT_APNLOAD_A = 5,
	/* Active power no-load detection
	(Current Channel B) is output on Pin 21 */
	ZXI_ALT_APNLOAD_B,
	/* Reactive power no-load detection
	(Current Channel A) is output on Pin 21 */
	ZXI_ALT_VARNLOAD_A,
	/* Reactive power no-load detection
	(Current Channel B) is output on Pin 21 */
	ZXI_ALT_VARNLOAD_B,
	/* Unlatched waveform sampling signal is output on Pin 21 */
	ZXI_ALT_WSMP,
	/* IRQsignal is output on Pin 21 */
	ZXI_ALT_IRQ,
	/* ZX detection is output on Pin 21 */
	ZXI_ALT_ZX,
	/* REVP detection is output on Pin 21 */
	ZXI_ALT_REVP
};

/***************************************************************************//**
 * @brief The `ade7953_revp_alt_e` is an enumeration that defines the
 * configuration options for the REVP pin (Pin 20) on the ADE7953 device.
 * Each enumerator specifies a different signal or detection type that
 * can be output on this pin, such as REVP detection, sag detection,
 * active or reactive power no-load detection for different current
 * channels, waveform sampling, IRQ signals, and zero-crossing
 * detections. This allows for flexible configuration of the pin's
 * functionality based on the application's requirements.
 *
 * @param REVP_ALT_REVP REVP detection is output on Pin 20 (default).
 * @param REVP_ALT_SAG Sag detection is output on Pin 20.
 * @param REVP_ALT_APNLOAD_A Active power no-load detection (Current Channel A)
 * is output on Pin 20.
 * @param REVP_ALT_APNLOAD_B Active power no-load detection (Current Channel B)
 * is output on Pin 20.
 * @param REVP_ALT_VARNLOAD_A Reactive power no-load detection (Current Channel
 * A) is output on Pin 20.
 * @param REVP_ALT_VARNLOAD_B Reactive power no-load detection (Current Channel
 * B) is output on Pin 20.
 * @param REVP_ALT_WSMP Unlatched waveform sampling signal is output on Pin 20.
 * @param REVP_ALT_IRQ IRQ signal is output on Pin 20.
 * @param REVP_ALT_ZX ZX detection is output on Pin 20.
 * @param REVP_ALT_ZX_I ZX_I detection is output on Pin 20.
 ******************************************************************************/
enum ade7953_revp_alt_e {
	/* REVP detection is output on Pin 20 (default) */
	REVP_ALT_REVP,
	/* Sag detection is output on Pin 20 */
	REVP_ALT_SAG,
	/* Active power no-load detection
	(Current Channel A) is output on Pin 20 */
	REVP_ALT_APNLOAD_A = 5,
	/* Active power no-load detection
	(Current Channel B) is output on Pin 20 */
	REVP_ALT_APNLOAD_B,
	/* Reactive power no-load detection
	(Current Channel A) is output on Pin 20 */
	REVP_ALT_VARNLOAD_A,
	/* Reactive power no-load detection
	(Current Channel B) is output on Pin 20 */
	REVP_ALT_VARNLOAD_B,
	/* Unlatched waveform sampling signal is output on Pin 20 */
	REVP_ALT_WSMP,
	/* IRQsignal is output on Pin 20 */
	REVP_ALT_IRQ,
	/* ZX detection is output on Pin 20 */
	REVP_ALT_ZX,
	/* ZX_I detection is output on Pin 20 */
	REVP_ALT_ZX_I
};

/***************************************************************************//**
 * @brief The `ade7953_awattacc_e` enumeration defines the different modes of
 * active energy accumulation for the ADE7953 device, specifically for
 * Current Channels A and B. It includes modes for normal accumulation,
 * positive-only accumulation, and absolute accumulation, allowing for
 * flexible energy measurement configurations based on the specific
 * requirements of the application.
 *
 * @param ADE7953_NORMAL_ACC_MODE_AWATT Represents the normal accumulation mode
 * for active energy.
 * @param ADE7953_POSITIVE_ACC_MODE Represents the positive-only accumulation
 * mode for active energy.
 * @param ADE7953_ABSOLUTE_ACC_MODE_AWATT Represents the absolute accumulation
 * mode for active energy.
 ******************************************************************************/
enum ade7953_awattacc_e {
	/* Normal mode. */
	ADE7953_NORMAL_ACC_MODE_AWATT,
	/* Positive-olny accumulation mode */
	ADE7953_POSITIVE_ACC_MODE,
	/* Absolute accumulation mode */
	ADE7953_ABSOLUTE_ACC_MODE_AWATT
};

/***************************************************************************//**
 * @brief The `ade7953_avaracc_e` enumeration defines the different modes of
 * reactive energy accumulation for the ADE7953 device, which is used for
 * energy metering. It includes modes for normal accumulation, antitamper
 * accumulation, and absolute accumulation, allowing for flexible energy
 * measurement configurations depending on the application requirements.
 *
 * @param ADE7953_NORMAL_ACC_MODE_AVAR Represents the normal accumulation mode
 * for reactive energy.
 * @param ADE7953_ANTITAMP_ACC_MODE Represents the antitamper accumulation mode
 * for reactive energy.
 * @param ADE7953_ABSOLUTE_ACC_MODE_AVAR Represents the absolute accumulation
 * mode for reactive energy.
 ******************************************************************************/
enum ade7953_avaracc_e {
	/* Normal mode. */
	ADE7953_NORMAL_ACC_MODE_AVAR,
	/* Antitamper accumulation mode */
	ADE7953_ANTITAMP_ACC_MODE,
	/* Absolute accumulation mode */
	ADE7953_ABSOLUTE_ACC_MODE_AVAR
};

/***************************************************************************//**
 * @brief The `ade7953_pga_gain_e` is an enumeration that defines the possible
 * programmable gain amplifier (PGA) gain settings for the ADE7953
 * device. These gain settings are used to adjust the amplification level
 * for the current and voltage channels, allowing for different levels of
 * signal amplification based on the application requirements. The gain
 * values range from 1 to 22, with the gain of 22 being specific to
 * Current Channel A.
 *
 * @param ADE7953_GAIN_1 Represents a gain of 1.
 * @param ADE7953_GAIN_2 Represents a gain of 2.
 * @param ADE7953_GAIN_4 Represents a gain of 4.
 * @param ADE7953_GAIN_8 Represents a gain of 8.
 * @param ADE7953_GAIN_16 Represents a gain of 16.
 * @param ADE7953_GAIN_22 Represents a gain of 22, applicable only for Current
 * Channel A.
 ******************************************************************************/
enum ade7953_pga_gain_e {
	/* Gain = 1 */
	ADE7953_GAIN_1,
	/* Gain = 2 */
	ADE7953_GAIN_2,
	/* Gain = 4 */
	ADE7953_GAIN_4,
	/* Gain = 8 */
	ADE7953_GAIN_8,
	/* Gain = 16 */
	ADE7953_GAIN_16,
	/* Gain = 22 */
	/* applicable only for Current Channel A */
	ADE7953_GAIN_22
};
/***************************************************************************//**
 * @brief The `ade7953_init_param` structure is used to define the
 * initialization parameters for the ADE7953 device. It includes
 * descriptors for SPI communication, GPIO reset, and IRQ control, as
 * well as a flag to enable 24-bit register access. This structure is
 * essential for setting up the device's communication and control
 * interfaces during initialization.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param gpio_reset Pointer to the GPIO initialization parameters for hardware
 * reset.
 * @param en_24_bit Flag to enable 24-bit register access.
 * @param irq_ctrl Pointer to the IRQ control descriptor for handling GPIO RDY
 * interrupts.
 ******************************************************************************/
struct ade7953_init_param {
	/** Device communication descriptor */
	struct no_os_spi_init_param 	*spi_init;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_init_param  	*gpio_reset;
	/** Enable 24 bits registers access */
	uint8_t				en_24_bit;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
};

/***************************************************************************//**
 * @brief The `ade7953_dev` structure is designed to encapsulate the necessary
 * descriptors and configurations for interfacing with the ADE7953 energy
 * metering IC. It includes pointers to SPI and GPIO descriptors for
 * communication and hardware reset, as well as IRQ controller
 * descriptors for managing interrupt routines. Additionally, it contains
 * a flag to enable access to 24-bit registers, facilitating advanced
 * configuration and data retrieval from the device.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param gpio_reset Pointer to the GPIO descriptor used for hardware reset of
 * the device.
 * @param irq_ctrl Pointer to the IRQ controller descriptor for handling IRQN
 * interrupts.
 * @param zx_ctrl Pointer to the IRQ controller descriptor for handling ZX
 * interrupts.
 * @param en_24_bit Flag to enable 24-bit register access.
 ******************************************************************************/
struct ade7953_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_desc  	*gpio_reset;
	/** IRQ device descriptor used to handle interrupt routine for IRQN */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** IRQ device descriptor used to handle interrupt routine for ZX */
	struct no_os_irq_ctrl_desc 	*zx_ctrl;
	/** Enable 24 bits registers access */
	uint8_t				en_24_bit;
};

/***************************************************************************//**
 * @brief The `ade7953_energy_values` structure is designed to hold energy
 * register values for the ADE7953 energy metering IC. It contains three
 * 32-bit integer fields that store the active, fundamental reactive, and
 * apparent energy values, respectively. This structure is used to
 * encapsulate energy data read from the device, facilitating the
 * management and processing of energy measurements in applications
 * utilizing the ADE7953.
 *
 * @param active_energy_reg_val Stores the active energy register value as a
 * 32-bit integer.
 * @param fundamental_reactive_energy_reg_val Stores the fundamental reactive
 * energy register value as a 32-bit
 * integer.
 * @param apparent_energy_reg_val Stores the apparent energy register value as a
 * 32-bit integer.
 ******************************************************************************/
struct ade7953_energy_values {
	/** Active energy register value */
	int32_t active_energy_reg_val;
	/** Fundamental reactive energy register value */
	int32_t fundamental_reactive_energy_reg_val;
	/** Apparent energy register value */
	int32_t apparent_energy_reg_val;
};

/***************************************************************************//**
 * @brief The `ade7953_power_values` structure is designed to hold power
 * measurement values from the ADE7953 energy metering IC. It contains
 * three 32-bit integer fields that store the active, reactive, and
 * apparent power register values, respectively. This structure is used
 * to encapsulate the power data retrieved from the device, allowing for
 * easy access and manipulation of these values in applications that
 * require energy monitoring and analysis.
 *
 * @param active_power_reg_val Stores the active power register value as a
 * 32-bit integer.
 * @param reactive_power_reg_val Stores the fundamental reactive power register
 * value as a 32-bit integer.
 * @param apparent_power_reg_val Stores the apparent power register value as a
 * 32-bit integer.
 ******************************************************************************/
struct ade7953_power_values {
	/** Active power register value */
	int32_t active_power_reg_val;
	/** Fundamental reactive power register value */
	int32_t reactive_power_reg_val;
	/** Apparent power register value */
	int32_t apparent_power_reg_val;
};

/***************************************************************************//**
 * @brief The `ade7953_rms_values` structure is designed to hold the root mean
 * square (RMS) values for the ADE7953 energy metering IC. It contains
 * three integer fields that store the RMS register values for two
 * current channels (A and B) and one voltage channel. This structure is
 * essential for applications that require monitoring and processing of
 * RMS values for accurate energy measurement and analysis.
 *
 * @param current_chA_rms_reg_val Stores the RMS register value for current
 * channel A.
 * @param current_chB_rms_reg_val Stores the RMS register value for current
 * channel B.
 * @param voltage_rms_reg_val Stores the RMS register value for voltage.
 ******************************************************************************/
struct ade7953_rms_values {
	/** Current chA rms register value */
	int32_t current_chA_rms_reg_val;
	/** Current chB rms register value */
	int32_t current_chB_rms_reg_val;
	/** Voltage rms register value */
	int32_t voltage_rms_reg_val;
};

/***************************************************************************//**
 * @brief The `ade7953_pq_values` structure is designed to hold power quality
 * register values for the ADE7953 energy metering IC. It contains two
 * 32-bit integer fields: `power_factor_reg_val` for storing the power
 * factor register value, and `period_reg_val` for storing the period
 * register value. This structure is used to encapsulate power quality
 * data, facilitating the management and retrieval of these specific
 * register values in applications utilizing the ADE7953 device.
 *
 * @param power_factor_reg_val Stores the power factor register value as a
 * 32-bit integer.
 * @param period_reg_val Stores the period register value as a 32-bit integer.
 ******************************************************************************/
struct ade7953_pq_values {
	/** Power factor register value */
	int32_t power_factor_reg_val;
	/** Period register value */
	int32_t period_reg_val;
};

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the ADE7953 device for operation by initializing
 * its communication interfaces and configuring it according to the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure and initializes the SPI interface. It also configures the
 * device for optimal operation and verifies the product version. If
 * initialization fails at any step, resources are cleaned up and an
 * error code is returned.
 *
 * @param device A pointer to a pointer of type `struct ade7953_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory upon successful initialization.
 * @param init_param A structure of type `struct ade7953_init_param` containing
 * initialization parameters such as SPI and GPIO
 * configurations. The structure must be properly populated
 * before calling the function.
 * @return Returns 0 on successful initialization. On failure, a negative error
 * code is returned, and no device is allocated.
 ******************************************************************************/
int ade7953_init(struct ade7953_dev **device,
		 struct ade7953_init_param init_param);

/* Read device register. */
/***************************************************************************//**
 * @brief Use this function to read the value of a specified register from an
 * ADE7953 device. It requires a valid device structure and a register
 * address to read from. The function supports reading from 8-bit,
 * 16-bit, 24-bit, and 32-bit registers, depending on the register
 * address provided. Ensure that the device has been properly initialized
 * before calling this function. The function will return an error code
 * if the device structure or the output parameter is null, or if the
 * read operation fails.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null.
 * @param reg_addr The address of the register to read from. It is a 16-bit
 * unsigned integer.
 * @param reg_data A pointer to an integer where the read register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure. The read
 * register value is stored in the location pointed to by `reg_data`.
 ******************************************************************************/
int ade7953_read(struct ade7953_dev *dev, uint16_t reg_addr,
		 int32_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief Use this function to write a 32-bit data value to a specified register
 * address on the ADE7953 device. The function supports writing to 8-bit,
 * 16-bit, 24-bit, and 32-bit registers, automatically determining the
 * register size based on the address. It must be called with a valid
 * device structure that has been initialized. If the device pointer is
 * null, the function returns an error. This function is typically used
 * to configure the device or update its settings.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null. The function returns -ENODEV if this
 * parameter is null.
 * @param reg_addr The address of the register to write to. The function
 * determines the register size based on this address.
 * @param reg_data The 32-bit data to be written to the register. The function
 * handles the data appropriately based on the register size.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -ENODEV if the device pointer is null.
 ******************************************************************************/
int ade7953_write(struct ade7953_dev *dev, uint16_t reg_addr,
		  uint32_t reg_data);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief Use this function to modify specific bits of a register in the ADE7953
 * device. It reads the current value of the register, applies a mask to
 * clear the bits to be updated, and then sets them according to the
 * provided data. This function should be called when you need to change
 * only certain bits of a register without affecting the others. Ensure
 * that the device is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the ADE7953 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, masked by the
 * provided mask. Only the bits corresponding to the mask will
 * be updated.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
static int ade7953_update_bits(struct ade7953_dev *dev, uint16_t reg_addr,
			       uint32_t mask, uint32_t reg_data);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * ADE7953 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that all allocated resources are freed and any associated
 * hardware interfaces are properly closed. It is important to pass a
 * valid device structure that was previously initialized; otherwise, the
 * function will return an error. This function handles the removal of
 * SPI, GPIO, and IRQ resources associated with the device.
 *
 * @param dev A pointer to an `ade7953_dev` structure representing the device to
 * be removed. Must not be null. If null, the function returns
 * `-ENODEV`. The caller retains ownership of the pointer, but the
 * resources it points to will be freed.
 * @return Returns 0 on success, or a negative error code if any of the resource
 * removals fail.
 ******************************************************************************/
int ade7953_remove(struct ade7953_dev *dev);

/* Reset the device using SW reset. */
/***************************************************************************//**
 * @brief Use this function to reset the ADE7953 device via software, which is
 * useful for reinitializing the device without requiring a hardware
 * reset. This function should be called when a reset of the device's
 * internal state is necessary, such as after configuration changes or
 * error recovery. Ensure that the device is properly initialized before
 * calling this function. The function will block for a short period to
 * allow the device to complete its reset process.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null. The function will return an error
 * code if the device is not properly initialized or if the reset
 * operation fails.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ade7953_sw_reset(struct ade7953_dev *dev);

/* Reset the device using HW reset. */
/***************************************************************************//**
 * @brief This function is used to perform a hardware reset on the ADE7953
 * device, which is necessary to reinitialize the device to its default
 * state. It should be called when a reset is required, such as after a
 * configuration change or to recover from an error state. The function
 * requires a valid device structure and will return an error if the
 * device is not properly initialized. It manipulates the GPIO reset line
 * if available, ensuring the device is reset correctly.
 *
 * @param dev A pointer to an initialized ade7953_dev structure. This must not
 * be null, and the structure should be properly set up with a valid
 * GPIO reset descriptor if hardware reset is to be performed. If the
 * pointer is null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails or if the device structure is invalid.
 ******************************************************************************/
int ade7953_hw_reset(struct ade7953_dev *dev);

/* Lock write to registers. */
/***************************************************************************//**
 * @brief Use this function to enable write protection on a specific set of
 * ADE7953 registers, preventing any modifications to them. This is
 * useful for ensuring the integrity of critical register values during
 * operation. The function must be called with a valid device structure
 * and a valid register selection enumeration. If an invalid register
 * selection is provided, the function will return an error code.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param regs_select An enumeration value of type ade7953_write_protect_e
 * indicating which set of registers to lock. Valid values
 * are ADE7953_8BITS_REGS, ADE7953_16BITS_REGS, and
 * ADE7953_24_32BITS_REGS. If an invalid value is provided,
 * the function returns an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input.
 ******************************************************************************/
int ade7953_wr_lock_8bit(struct ade7953_dev *dev,
			 enum ade7953_write_protect_e regs_select);

/* Version product */
/***************************************************************************//**
 * @brief Use this function to obtain the version product information from an
 * ADE7953 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function
 * requires a valid pointer to store the retrieved version data. If the
 * pointer is null, the function will return an error. This function is
 * useful for verifying the version of the device in use.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. The device must be properly initialized before use.
 * @param data_read A pointer to a uint32_t variable where the version product
 * information will be stored. Must not be null, or the
 * function will return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The version product information is stored in the location
 * pointed to by data_read on success.
 ******************************************************************************/
int ade7953_version_product(struct ade7953_dev *dev, uint32_t *data_read);

/* reseat IApeak val */
/***************************************************************************//**
 * @brief This function is used to read the current value of the IAPEAK register
 * from the ADE7953 device and reset it. It should be called when you
 * need to obtain the peak current value and simultaneously reset the
 * register for future measurements. Ensure that the device is properly
 * initialized before calling this function. The function requires a
 * valid pointer to store the read value and will return an error if the
 * pointer is null.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. The device must be properly initialized before use.
 * @param val A pointer to a uint32_t variable where the read IAPEAK value will
 * be stored. Must not be null, as the function will return an error
 * if it is.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * or if the input pointer is null.
 ******************************************************************************/
int ade7953_reset_iapk_val(struct ade7953_dev *dev, uint32_t *val);

/* reseat IBpeak val */
/***************************************************************************//**
 * @brief This function is used to read the current peak value of the IB channel
 * from the ADE7953 device and reset it. It should be called when the
 * user needs to obtain the current peak value for monitoring or logging
 * purposes. The function requires a valid device structure and a non-
 * null pointer to store the retrieved value. It returns an error code if
 * the operation fails, such as when the provided pointer is null.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. The device must be properly initialized before calling
 * this function.
 * @param val A pointer to a uint32_t variable where the retrieved IB peak value
 * will be stored. Must not be null, otherwise the function will
 * return an error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as -EINVAL if the val parameter is null.
 ******************************************************************************/
int ade7953_reset_ibpk_val(struct ade7953_dev *dev, uint32_t *val);

/* reseat Vpeak val */
/***************************************************************************//**
 * @brief This function reads the current value of the Vpeak register from the
 * ADE7953 device and stores it in the provided memory location. It is
 * typically used to obtain the peak voltage measurement from the device.
 * The function must be called with a valid device structure and a non-
 * null pointer for storing the result. If the pointer is null, the
 * function returns an error. This function should be used after the
 * device has been properly initialized.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. The device must be properly initialized before calling
 * this function.
 * @param val A pointer to a uint32_t variable where the Vpeak register value
 * will be stored. Must not be null. If null, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The Vpeak register value is stored in the location pointed to
 * by val on success.
 ******************************************************************************/
int ade7953_reset_vpk_val(struct ade7953_dev *dev, uint32_t *val);

/* Get interrupt indicator from STATA register. */
/***************************************************************************//**
 * @brief Use this function to check the interrupt status of the ADE7953 device
 * by reading the STATA register. It is essential to ensure that the
 * `status` pointer is not null before calling this function, as it will
 * store the result of the interrupt status check. The function should be
 * called when you need to determine if specific interrupts, as indicated
 * by the mask, are active. This function returns an error code if the
 * read operation fails or if the `status` pointer is null.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. The device must be properly initialized before calling
 * this function.
 * @param msk A 32-bit mask indicating which interrupt bits to check in the
 * STATA register. The mask should be set according to the specific
 * interrupts of interest.
 * @param status A pointer to a uint8_t where the function will store the
 * interrupt status. Must not be null. The caller is responsible
 * for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code on failure. The
 * `status` pointer is updated with the interrupt status if the function
 * succeeds.
 ******************************************************************************/
int ade7953_get_int_stata(struct ade7953_dev *dev, uint32_t msk,
			  uint8_t *status);

/* Get interrupt indicator from STATB register. */
/***************************************************************************//**
 * @brief This function is used to check the interrupt status of the ADE7953
 * device by reading the STATB register. It is typically called when you
 * need to determine if specific interrupts have been triggered. The
 * function requires a valid device structure and a non-null pointer for
 * the status output. It should be used after the device has been
 * properly initialized. If the status pointer is null, the function will
 * return an error.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. The device must be properly initialized before calling
 * this function.
 * @param msk A 32-bit mask used to specify which bits in the STATB register
 * should be checked. The mask determines which interrupt status bits
 * are of interest.
 * @param status A pointer to a uint8_t where the function will store the result
 * of the interrupt status check. Must not be null. The caller is
 * responsible for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * (e.g., if the status pointer is null).
 ******************************************************************************/
int ade7953_get_int_statb(struct ade7953_dev *dev, uint32_t msk,
			  uint8_t *status);

/* Clear irq STATA flags. */
/***************************************************************************//**
 * @brief Use this function to clear the interrupt status flags in the STATA
 * register of the ADE7953 device. This is typically done after handling
 * the interrupts to reset the status and prepare for future interrupts.
 * It is important to ensure that the device is properly initialized
 * before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. This parameter must not be null, and the device must
 * be properly initialized before use.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * that the interrupt status flags were not cleared.
 ******************************************************************************/
int ade7953_clear_irq_stata(struct ade7953_dev *dev);

/* Clear irq STATB flags. */
/***************************************************************************//**
 * @brief Use this function to clear the interrupt status flags associated with
 * the STATB register of the ADE7953 device. This is typically necessary
 * after handling an interrupt to reset the status flags and prepare for
 * future interrupts. Ensure that the device is properly initialized
 * before calling this function. This function does not modify any other
 * state or configuration of the device.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null. The function will not perform any
 * operation if this parameter is invalid.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ade7953_clear_irq_statb(struct ade7953_dev *dev);

/* Enable interrupt voltage ch and current ch A. */
/***************************************************************************//**
 * @brief Use this function to enable or disable specific interrupts for the
 * voltage and current channel A on the ADE7953 device. This function is
 * typically called when configuring the device to respond to certain
 * events or conditions. Ensure that the device is properly initialized
 * before calling this function. The function modifies the interrupt
 * enable register based on the provided mask and enable parameters.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param msk A 32-bit mask specifying which interrupts to modify. Each bit
 * corresponds to a specific interrupt.
 * @param en A 8-bit value indicating whether to enable (1) or disable (0) the
 * specified interrupts. Values outside this range may lead to
 * undefined behavior.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation's result.
 ******************************************************************************/
int ade7953_enable_int_a(struct ade7953_dev *dev, uint32_t msk, uint8_t en);

/* Enable/disable interrupt voltage ch and current ch B. */
/***************************************************************************//**
 * @brief This function is used to enable or disable specific interrupts for the
 * voltage and current channel B on the ADE7953 device. It should be
 * called when you need to configure the interrupt settings for channel
 * B, typically after initializing the device. The function modifies the
 * interrupt enable register based on the provided mask and enable
 * parameters. Ensure that the device is properly initialized before
 * calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param msk A 32-bit mask specifying which interrupts to modify. Each bit
 * corresponds to a specific interrupt.
 * @param en A 8-bit value indicating whether to enable (1) or disable (0) the
 * specified interrupts. Values other than 0 or 1 may lead to
 * unexpected behavior.
 * @return Returns an integer status code. A non-zero value indicates an error
 * occurred during the operation.
 ******************************************************************************/
int ade7953_enable_int_b(struct ade7953_dev *dev, uint32_t msk, uint8_t en);

/* Zero-crossing interrupt edge selection */
/***************************************************************************//**
 * @brief Use this function to configure the zero-crossing interrupt edge
 * detection for an ADE7953 device. This function should be called when
 * you need to specify which edge (positive, negative, or both) should
 * trigger an interrupt. It is essential to ensure that the device is
 * properly initialized before calling this function. The function
 * modifies the device's configuration register to reflect the selected
 * edge. If an invalid selection is provided, the function defaults to a
 * predefined setting.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null.
 * @param sel An `enum ade7953_zx_edge_e` value specifying the desired zero-
 * crossing edge for interrupt generation. Valid values are
 * `ADE7953_ZX_BOTH_1`, `ADE7953_ZX_NEG`, `ADE7953_ZX_POS`, and
 * `ADE7953_ZX_BOTH_2`. If an invalid value is provided, a default
 * setting is applied.
 * @return Returns an integer status code. A non-negative value indicates
 * success, while a negative value indicates an error.
 ******************************************************************************/
int ade7953_zx_int_edge_set(struct ade7953_dev *dev,
			    enum ade7953_zx_edge_e sel);

/* Configure output signal on CF1/CF2 pin */
/***************************************************************************//**
 * @brief Use this function to set the output signal configuration for a
 * specified CF pin on the ADE7953 device. This function is typically
 * called after initializing the device and when you need to change the
 * output signal characteristics of the CF1 or CF2 pin. Ensure that the
 * device structure is properly initialized and that the selected CF pin
 * is valid. The function will return an error if an invalid CF pin is
 * specified.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param sel An enum value of type ade7953_cfsel_e specifying the desired
 * configuration for the CF pin. Valid values are defined in the
 * enum.
 * @param cf_pin An enum value of type ade7953_cf_pin_e indicating which CF pin
 * to configure (ADE7953_CF1_PIN or ADE7953_CF2_PIN). If an
 * invalid value is provided, the function returns -EINVAL.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid input.
 ******************************************************************************/
int ade7953_cf_output_set(struct ade7953_dev *dev,
			  enum ade7953_cfsel_e sel, enum ade7953_cf_pin_e cf_pin);

/* Configure of ZX pin (Pin1) */
/***************************************************************************//**
 * @brief This function configures the output signal on the ZX pin of the
 * ADE7953 device based on the specified selection. It should be used
 * when you need to change the signal output on the ZX pin to match
 * specific application requirements. The function must be called with a
 * valid device structure and a valid selection from the
 * `ade7953_zx_alt_e` enumeration. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to an `ade7953_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param sel An enumeration value of type `ade7953_zx_alt_e` specifying the
 * desired configuration for the ZX pin. Must be a valid enumeration
 * value.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ade7953_zx_config_pin(struct ade7953_dev *dev,
			  enum ade7953_zx_alt_e sel);

/* Configure of ZXI pin (Pin21) */
/***************************************************************************//**
 * @brief This function configures the output signal on the ZXI pin (Pin 21) of
 * the ADE7953 device. It should be called when you need to change the
 * signal output configuration of the ZXI pin to one of the predefined
 * modes. Ensure that the device is properly initialized before calling
 * this function. The function returns an integer status code indicating
 * success or failure of the operation.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null.
 * @param sel An enumeration value of type `ade7953_zxi_alt_e` specifying the
 * desired output configuration for the ZXI pin. Valid values are
 * defined in the `ade7953_zxi_alt_e` enum.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ade7953_zxi_config_pin(struct ade7953_dev *dev,
			   enum ade7953_zxi_alt_e sel);

/* Configure of REVP pin (Pin20) */
/***************************************************************************//**
 * @brief Use this function to set the output signal configuration for the REVP
 * pin on the ADE7953 device. This function should be called when you
 * need to change the behavior of the REVP pin to match specific
 * application requirements. Ensure that the device is properly
 * initialized before calling this function. The function modifies the
 * device's register settings based on the selected configuration.
 *
 * @param dev A pointer to an initialized `ade7953_dev` structure representing
 * the device. Must not be null. The caller retains ownership.
 * @param sel An enumeration value of type `ade7953_revp_alt_e` specifying the
 * desired configuration for the REVP pin. Valid values are defined
 * in the `ade7953_revp_alt_e` enum.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the operation's result.
 ******************************************************************************/
int ade7953_revp_pin_config(struct ade7953_dev *dev,
			    enum ade7953_revp_alt_e sel);

/* ACC mode selection for active energy */
/***************************************************************************//**
 * @brief This function configures the active energy accumulation mode for a
 * specified current channel on the ADE7953 device. It should be used
 * when you need to change the accumulation mode to either normal,
 * positive-only, or absolute mode for a specific current channel. The
 * function must be called with a valid device structure and appropriate
 * enumeration values for the mode and channel. If an invalid channel is
 * provided, the function returns an error. Ensure the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param mode An enumeration value of type ade7953_awattacc_e specifying the
 * desired accumulation mode. Valid values are
 * ADE7953_NORMAL_ACC_MODE_AWATT, ADE7953_POSITIVE_ACC_MODE, and
 * ADE7953_ABSOLUTE_ACC_MODE_AWATT.
 * @param channel An enumeration value of type ade7953_i_ch_e specifying the
 * current channel to configure. Valid values are ADE7953_I_CH1
 * and ADE7953_I_CH2. If an invalid value is provided, the
 * function returns -EINVAL.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid channel input.
 ******************************************************************************/
int ade7953_acc_active_engy_mode(struct ade7953_dev *dev,
				 enum ade7953_awattacc_e mode, enum ade7953_i_ch_e channel);

/* ACC mode selection for reactive energy */
/***************************************************************************//**
 * @brief This function configures the reactive energy accumulation mode for a
 * specified current channel on the ADE7953 device. It should be called
 * when you need to change the accumulation mode for reactive energy
 * measurements. The function requires a valid device structure and
 * specific enumeration values for the mode and channel. If an invalid
 * channel is provided, the function returns an error. Ensure the device
 * is properly initialized before calling this function.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param mode An enumeration value of type ade7953_avaracc_e specifying the
 * desired reactive energy accumulation mode. Valid values are
 * defined in the enum.
 * @param channel An enumeration value of type ade7953_i_ch_e specifying the
 * current channel (ADE7953_I_CH1 or ADE7953_I_CH2). If an
 * invalid channel is provided, the function returns -EINVAL.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for an invalid channel.
 ******************************************************************************/
int ade7953_acc_reactive_engy_mode(struct ade7953_dev *dev,
				   enum ade7953_avaracc_e mode, enum ade7953_i_ch_e channel);

/* Read energy values */
/***************************************************************************//**
 * @brief Use this function to obtain the active, reactive, and apparent energy
 * values from a specified current channel of the ADE7953 device. It is
 * essential to ensure that the `data` parameter is not null before
 * calling this function, as it will store the retrieved energy values.
 * The function requires a valid `channel` parameter, which must be
 * either `ADE7953_I_CH1` or `ADE7953_I_CH2`, corresponding to the two
 * current channels available on the device. If the `channel` is invalid
 * or the `data` parameter is null, the function will return an error
 * code. This function should be called after the device has been
 * properly initialized and configured.
 *
 * @param dev A pointer to an `ade7953_dev` structure representing the device.
 * The device must be initialized before use. The caller retains
 * ownership.
 * @param data A pointer to an `ade7953_energy_values` structure where the
 * energy values will be stored. Must not be null.
 * @param channel An `ade7953_i_ch_e` enum value specifying the current channel
 * to read from. Must be either `ADE7953_I_CH1` or
 * `ADE7953_I_CH2`. Invalid values result in an error.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `data` structure is populated with the energy values.
 ******************************************************************************/
int ade7953_energy_vals(struct ade7953_dev *dev,
			struct ade7953_energy_values *data, enum ade7953_i_ch_e channel);

/* Read power values */
/***************************************************************************//**
 * @brief Use this function to obtain the active, reactive, and apparent power
 * values from the ADE7953 device for a specified current channel. It is
 * essential to ensure that the `data` parameter is not null before
 * calling this function. The function requires a valid `channel`
 * parameter, which must be either `ADE7953_I_CH1` or `ADE7953_I_CH2`. If
 * the `channel` is invalid or if `data` is null, the function will
 * return an error. This function should be called after the device has
 * been properly initialized and configured.
 *
 * @param dev A pointer to an `ade7953_dev` structure representing the device.
 * The device must be initialized before use. The caller retains
 * ownership.
 * @param data A pointer to an `ade7953_power_values` structure where the power
 * values will be stored. Must not be null. The function will
 * populate this structure with the retrieved power values.
 * @param channel An `ade7953_i_ch_e` enum value specifying the current channel
 * to read from. Must be either `ADE7953_I_CH1` or
 * `ADE7953_I_CH2`. Invalid values will result in an error.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the `data` structure is populated with the power values.
 ******************************************************************************/
int ade7953_power_vals(struct ade7953_dev *dev,
		       struct ade7953_power_values *data, enum ade7953_i_ch_e channel);

/* Read rms values */
/***************************************************************************//**
 * @brief Use this function to obtain the root mean square (RMS) values for
 * voltage and current from an ADE7953 device. It requires a valid device
 * structure and a data structure to store the results. The function
 * reads the voltage RMS value and the current RMS value for the
 * specified channel. Ensure that the data parameter is not null before
 * calling this function. The function will return an error code if the
 * data parameter is null or if an invalid channel is specified.
 *
 * @param dev A pointer to an initialized ade7953_dev structure representing the
 * device. Must not be null.
 * @param data A pointer to an ade7953_rms_values structure where the RMS values
 * will be stored. Must not be null.
 * @param channel An enum ade7953_i_ch_e value specifying the current channel to
 * read (ADE7953_I_CH1 or ADE7953_I_CH2). Invalid values will
 * result in an error.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -EINVAL for invalid parameters).
 ******************************************************************************/
int ade7953_rms_vals(struct ade7953_dev *dev,
		     struct ade7953_rms_values *data, enum ade7953_i_ch_e channel);

/* Read power quaility values */
/***************************************************************************//**
 * @brief Use this function to obtain power quality metrics, specifically the
 * power factor and period values, from an ADE7953 device. This function
 * should be called when you need to access these specific power quality
 * parameters. Ensure that the device has been properly initialized
 * before calling this function. The function requires a valid pointer to
 * a `struct ade7953_pq_values` where the results will be stored. If the
 * provided pointer is null, the function will return an error code.
 *
 * @param dev A pointer to an `ade7953_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership.
 * @param data A pointer to an `ade7953_pq_values` structure where the power
 * quality values will be stored. Must not be null. If null, the
 * function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The `data` structure is populated with the power factor and
 * period values on success.
 ******************************************************************************/
int ade7953_power_quality_vals(struct ade7953_dev *dev,
			       struct ade7953_pq_values *data);

#endif /* __ADE7953_H__ */
