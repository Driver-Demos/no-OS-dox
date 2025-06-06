/***************************************************************************//**
 *   @file   ade9153a.h
 *   @brief  Header file of ADE9153A Driver.
 *   @author Radu Etz (radu.etz@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef __ADE9153A_H__
#define __ADE9153A_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
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

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* SPI commands */
#define ADE9153A_SPI_READ					NO_OS_BIT(3)

/* Lock write key */
#define ADE9153A_LOCK_KEY					0x3C64

/* Unlock write key */
#define ADE9153A_UNLOCK_KEY					0x4AD1

/* COMPMODE val */
#define ADE9153A_COMPMODEVAL					0x0005

/* RUN */
#define ADE9153A_RUN						0x0001
#define ADE9153A_STOP						0x0000

/* Version product */
#define ADE9153A_VERSION					0x9153a

/* Reset Delay */
#define ADE9153A_RESET_DEL					0x1F4

/* Address range of 16 bit registers */
#define ADE9153A_START_16ADDR					0x473
#define ADE9153A_END_16ADDR					0x600

/* ADE9153A CRC constants */
#define ADE9153A_CRC16_POLY					0x1021
#define ADE9153A_CRC16_INIT_VAL					0xFFFF

#define ADE9153A_NO_BYTES_W_16					0x0004
#define ADE9153A_NO_BYTES_W_32					0x0006
#define ADE9153A_NO_BYTES_R_16					0x0006
#define ADE9153A_NO_BYTES_R_32					0x0008


/* ENABLE and DISABLE */
#define ENABLE							1u
#define DISABLE							0u

/* ADE9153A Register Map */
#define ADE9153A_REG_AIGAIN					0x000
#define ADE9153A_REG_APHASECAL					0x001
#define ADE9153A_REG_AVGAIN					0x002
#define ADE9153A_REG_AIRMS_OS					0x003
#define ADE9153A_REG_AVRMS_OS					0x004
#define ADE9153A_REG_APGAIN					0x005
#define ADE9153A_REG_AWATT_OS					0x006
#define ADE9153A_REG_AFVAR_OS					0x007
#define ADE9153A_REG_AVRMS_OC_OS				0x008
#define ADE9153A_REG_AIRMS_OC_OS				0x009
#define ADE9153A_REG_BIGAIN					0x010
#define ADE9153A_REG_BPHASECAL					0x011
#define ADE9153A_REG_BIRMS_OS					0x013
#define ADE9153A_REG_BIRMS_OC_OS				0x019
#define ADE9153A_REG_CONFIG0					0x020
#define ADE9153A_REG_VNOM					0x021
#define ADE9153A_REG_DICOEFF					0x022
#define ADE9153A_REG_BI_PGAGAIN					0x023
#define ADE9153A_REG_MS_ACAL_CFG				0x030
#define ADE9153A_REG_MS_AICC_USER				0x045
#define ADE9153A_REG_MS_BICC_USER				0x046
#define ADE9153A_REG_MS_AVCC_USER				0x047
#define ADE9153A_REG_CT_PHASE_DELAY				0x049
#define ADE9153A_REG_CT_CORNER					0x04A
#define ADE9153A_REG_VDIV_RSMALL				0x04C
#define ADE9153A_REG_AI_WAV					0x200
#define ADE9153A_REG_AV_WAV					0x201
#define ADE9153A_REG_AIRMS					0x202
#define ADE9153A_REG_AVRMS					0x203
#define ADE9153A_REG_AWATT					0x204
#define ADE9153A_REG_AVA					0x206
#define ADE9153A_REG_AFVAR					0x207
#define ADE9153A_REG_APF					0x208
#define ADE9153A_REG_AIRMS_OC					0x209
#define ADE9153A_REG_AVRMS_OC					0x20A
#define ADE9153A_REG_BI_WAV					0x210
#define ADE9153A_REG_BIRMS					0x212
#define ADE9153A_REG_BIRMS_OC					0x219
#define ADE9153A_REG_MS_ACAL_AICC				0x220
#define ADE9153A_REG_MS_ACAL_AICERT				0x221
#define ADE9153A_REG_MS_ACAL_BICC				0x222
#define ADE9153A_REG_MS_ACAL_BICERT				0x223
#define ADE9153A_REG_MS_ACAL_AVCC				0x224
#define ADE9153A_REG_MS_ACAL_AVCERT				0x225
#define ADE9153A_REG_MS_STATUS_CURRENT				0x240
#define ADE9153A_REG_VERSION_DSP				0x241
#define ADE9153A_REG_VERSION_PRODUCT				0x242
#define ADE9153A_REG_AWATT_ACC					0x39D
#define ADE9153A_REG_AWATTHR_LO					0x39E
#define ADE9153A_REG_AWATTHR_HI					0x39F
#define ADE9153A_REG_AVA_ACC					0x3B1
#define ADE9153A_REG_AVAHR_LO					0x3B2
#define ADE9153A_REG_AVAHR_HI					0x3B3
#define ADE9153A_REG_AFVAR_ACC					0x3BB
#define ADE9153A_REG_AFVARHR_LO					0x3BC
#define ADE9153A_REG_AFVARHR_HI					0x3BD
#define ADE9153A_REG_PWATT_ACC					0x3EB
#define ADE9153A_REG_NWATT_ACC					0x3EF
#define ADE9153A_REG_PFVAR_ACC					0x3F3
#define ADE9153A_REG_NFVAR_ACC					0x3F7
#define ADE9153A_REG_IPEAK					0x400
#define ADE9153A_REG_VPEAK					0x401
#define ADE9153A_REG_STATUS					0x402
#define ADE9153A_REG_MASK					0x405
#define ADE9153A_REG_OI_LVL					0x409
#define ADE9153A_REG_OIA					0x40A
#define ADE9153A_REG_OIB					0x40B
#define ADE9153A_REG_USER_PERIOD				0x40E
#define ADE9153A_REG_VLEVEL					0x40F
#define ADE9153A_REG_DIP_LVL					0x410
#define ADE9153A_REG_DIPA					0x411
#define ADE9153A_REG_SWELL_LVL					0x414
#define ADE9153A_REG_SWELLA					0x415
#define ADE9153A_REG_APERIOD					0x418
#define ADE9153A_REG_ACT_NL_LVL					0x41C
#define ADE9153A_REG_REACT_NL_LVL				0x41D
#define ADE9153A_REG_APP_NL_LVL					0x41E
#define ADE9153A_REG_PHNOLOAD					0x41F
#define ADE9153A_REG_WTHR					0x420
#define ADE9153A_REG_VARTHR					0x421
#define ADE9153A_REG_VATHR					0x422
#define ADE9153A_REG_LAST_DATA_32				0x423
#define ADE9153A_REG_CT_PHASE_MEAS				0x424
#define ADE9153A_REG_CF_LCFG					0x425
#define ADE9153A_REG_TEMP_TRIM					0x471
#define ADE9153A_REG_CHIP_ID_HI					0x472
#define ADE9153A_REG_CHIP_ID_LO					0x473
#define ADE9153A_REG_RUN					0x480
#define ADE9153A_REG_CONFIG1					0x481
#define ADE9153A_REG_ANGL_AV_AI					0x485
#define ADE9153A_REG_ANGL_AI_BI					0x488
#define ADE9153A_REG_DIP_CYC					0x48B
#define ADE9153A_REG_SWELL_CYC					0x48C
#define ADE9153A_REG_CFMODE					0x490
#define ADE9153A_REG_COMPMODE					0x491
#define ADE9153A_REG_ACCMODE					0x492
#define ADE9153A_REG_CONFIG3					0x493
#define ADE9153A_REG_CF1DEN					0x494
#define ADE9153A_REG_CF2DEN					0x495
#define ADE9153A_REG_ZXTOUT					0x498
#define ADE9153A_REG_ZXTHRSH					0x499
#define ADE9153A_REG_ZX_CFG					0x49A
#define ADE9153A_REG_PHSIGN					0x49D
#define ADE9153A_REG_CRC_RSLT					0x4A8
#define ADE9153A_REG_CRC_SPI					0x4A9
#define ADE9153A_REG_LAST_DATA_16				0x4AC
#define ADE9153A_REG_LAST_CMD					0x4AE
#define ADE9153A_REG_CONFIG2					0x4AF
#define ADE9153A_REG_EP_CFG					0x4B0
#define ADE9153A_REG_PWR_TIME					0x4B1
#define ADE9153A_REG_EGY_TIME					0x4B2
#define ADE9153A_REG_CRC_FORCE					0x4B4
#define ADE9153A_REG_TEMP_CFG					0x4B6
#define ADE9153A_REG_TEMP_RSLT					0x4B7
#define ADE9153A_REG_AI_PGAGAIN					0x4B9
#define ADE9153A_REG_WR_LOCK					0x4BF
#define ADE9153A_REG_MS_STATUS_IRQ				0x4C0
#define ADE9153A_REG_EVENT_STATUS				0x4C1
#define ADE9153A_REG_CHIP_STATUS				0x4C2
#define ADE9153A_REG_UART_BAUD_SWITCH				0x4DC
#define ADE9153A_REG_VERSION					0x4FE
#define ADE9153A_REG_AI_WAV_1					0x600
#define ADE9153A_REG_AV_WAV_1					0x601
#define ADE9153A_REG_BI_WAV_1					0x602
#define ADE9153A_REG_AIRMS_1					0x604
#define ADE9153A_REG_BIRMS_1					0x605
#define ADE9153A_REG_AVRMS_1					0x606
#define ADE9153A_REG_AWATT_1					0x608
#define ADE9153A_REG_AFVAR_1					0x60A
#define ADE9153A_REG_AVA_1					0x60C
#define ADE9153A_REG_APF_1					0x60E
#define ADE9153A_REG_ AI_WAV_2					0x610
#define ADE9153A_REG_AV_WAV_2					0x611
#define ADE9153A_REG_AIRMS_2					0x612
#define ADE9153A_REG_AVRMS_2					0x613
#define ADE9153A_REG_AWATT_2					0x614
#define ADE9153A_REG_AVA_2					0x615
#define ADE9153A_REG_AFVAR_2					0x616
#define ADE9153A_REG_APF_2					0x617
#define ADE9153A_REG_BI_WAV_2					0x618
#define ADE9153A_REG_BIRMS_2					0x61A

/* ADE9153A_REG_CONFIG0 Bit Definition */
#define ADE9153A_DISRPLPF_MSK					NO_OS_BIT(8)
#define ADE9153A_DISAPLPF_MSK					NO_OS_BIT(7)
#define ADE9153A_VNOMA_EN_MSK					NO_OS_BIT(5)
#define ADE9153A_RMS_OC_SRC_MSK					NO_OS_BIT(4)
#define ADE9153A_ZX_SRC_SEL_MSK					NO_OS_BIT(3)
#define ADE9153A_INTEN_BI_MSK					NO_OS_BIT(2)
#define ADE9153A_HPFDIS_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_BI_PGAGAIN Bit Definition */
#define ADE9153A_BI_PGAGAIN_MSK					NO_OS_GENMASK(31, 0)

/* ADE9153A_REG_MS_ACAL_CFG Bit Definition */
#define ADE9153A_AUTOCAL_AV_MSK					NO_OS_BIT(6)
#define ADE9153A_AUTOCAL_BI_MSK					NO_OS_BIT(5)
#define ADE9153A_AUTOCAL_AI_MSK					NO_OS_BIT(4)
#define ADE9153A_ACALMODE_BI_MSK				NO_OS_BIT(3)
#define ADE9153A_ACALMODE_AI_MSK				NO_OS_BIT(2)
#define ADE9153A_ACAL_RUN_MSK					NO_OS_BIT(1)
#define ADE9153A_ACAL_MODE_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_MS_STATUS_CURRENT Bit Definition */
#define ADE9153A_MS_SYSRDYP_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_IPEAK Bit Definition */
#define ADE9153A_IPPHASE_MSK					NO_OS_GENMASK(26, 24)
#define ADE9153A_IPEAKVAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_VPEAK Bit Definition */
#define ADE9153A_VPEAKVAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_STATUS / ADE9153A_REG_MASK Bit Definition */
#define ADE9153A_CHIP_STAT_MSK					NO_OS_BIT(31)
#define ADE9153A_EVENT_STAT_MSK					NO_OS_BIT(30)
#define ADE9153A_MS_STAT_MSK					NO_OS_BIT(29)
#define ADE9153A_PF_RDY_MSK					NO_OS_BIT(25)
#define ADE9153A_CRC_CHG_MSK					NO_OS_BIT(24)
#define ADE9153A_CRC_DONE_MSK					NO_OS_BIT(23)
#define ADE9153A_ZXTOAV_MSK					NO_OS_BIT(21)
#define ADE9153A_ZXBI_MSK					NO_OS_BIT(20)
#define ADE9153A_ZXAI_MSK					NO_OS_BIT(19)
#define ADE9153A_ZXAV_MSK					NO_OS_BIT(17)
#define ADE9153A_RSTDONE_MSK					NO_OS_BIT(16)
#define ADE9153A_FVARNL_MSK					NO_OS_BIT(15)
#define ADE9153A_VANL_MSK					NO_OS_BIT(14)
#define ADE9153A_WATTNL_MSK					NO_OS_BIT(13)
#define ADE9153A_TEMP_RDY_MSK					NO_OS_BIT(12)
#define ADE9153A_RMS_OC_RDY_MSK					NO_OS_BIT(11)
#define ADE9153A_PWRRDY_MSK					NO_OS_BIT(10)
#define ADE9153A_DREADY_MSK					NO_OS_BIT(9)
#define ADE9153A_EGYRDY_MSK					NO_OS_BIT(8)
#define ADE9153A_CF2_MSK					NO_OS_BIT(7)
#define ADE9153A_CF1_MSK					NO_OS_BIT(6)
#define ADE9153A_REVPCF2_MSK					NO_OS_BIT(5)
#define ADE9153A_REVPCF1_MSK					NO_OS_BIT(4)
#define ADE9153A_REVRPA_MSK					NO_OS_BIT(2)
#define ADE9153A_REVAPA_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_OI_LVL Bit Definition */
#define ADE9153A_OILVL_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_OIA Bit Definition */
#define ADE9153A_OIA_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_OIB Bit Definition */
#define ADE9153A_OIB_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_VLEVEL Bit Definition */
#define ADE9153A_VLEVEL_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_DIPA Bit Definition */
#define ADE9153A_DIPA_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_SWELLA Bit Definition */
#define ADE9153A_SWELLA_VAL_MSK					NO_OS_GENMASK(23, 0)

/* ADE9153A_REG_PHNOLOAD Bit Definition */
#define ADE9153A_AFVARNL_MSK					NO_OS_BIT(2)
#define ADE9153A_AVANL_MSK					NO_OS_BIT(1)
#define ADE9153A_AWATTNL_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_CF_LCFG Bit Definition */
#define ADE9153A_CF2_LT_MSK					NO_OS_BIT(20)
#define ADE9153A_CF1_LT_MSK					NO_OS_BIT(19)
#define ADE9153A_CF_LTMR_MSK					NO_OS_GENMASK(18, 0)

/* ADE9153A_REG_TEMP_TRIM Bit Definition */
#define ADE9153A_TEMP_OFFSET_MSK				NO_OS_GENMASK(31, 16)
#define ADE9153A_TEMP_GAIN_MSK					NO_OS_GENMASK(15, 0)

/* ADE9153A_REG_CONFIG1 Bit Definition */
#define ADE9153A_EXT_REF_MSK					NO_OS_BIT(15)
#define ADE9153A_DIP_SWELL_IRQ_MODE_MSK				NO_OS_BIT(14)
#define ADE9153A_BURST_EN_MSK					NO_OS_BIT(11)
#define ADE9153A_PWR_SETTLE_MSK					NO_OS_GENMASK(9, 8)
#define ADE9153A_CF_ACC_CLR_MSK					NO_OS_BIT(5)
#define ADE9153A_ZX_OUT_OE_MSK					NO_OS_BIT(2)
#define ADE9153A_DREADY_OE_MSK					NO_OS_BIT(1)
#define ADE9153A_SWRST_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_CFMODE Bit Definition */
#define ADE9153A_CF2DIS_MSK					NO_OS_BIT(7)
#define ADE9153A_CF1DIS_MSK					NO_OS_BIT(6)
#define ADE9153A_CF2SEL_MSK					NO_OS_GENMASK(5, 3)
#define ADE9153A_CF1SEL_MSK					NO_OS_GENMASK(2, 0)

/* ADE9153A_REG_ACCMODE Bit Definition */
#define ADE9153A_SELFREQ_MSK					NO_OS_BIT(4)
#define ADE9153A_VARACC_MSK					NO_OS_GENMASK(3, 2)
#define ADE9153A_WATTACC_MSK					NO_OS_GENMASK(1, 0)

/* ADE9153A_REG_CONFIG3 Bit Definition */
#define ADE9153A_PEAK_SEL_MSK					NO_OS_GENMASK(3, 2)
#define ADE9153A_OIB_EN_MSK					NO_OS_BIT(1)
#define ADE9153A_OIA_EN_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_ZX_CFG Bit Definition */
#define ADE9153A_DISZXLPF_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_PHSIGN Bit Definition */
#define ADE9153A_CF2SIGN_MSK					NO_OS_BIT(7)
#define ADE9153A_CF1SIGN_MSK					NO_OS_BIT(6)
#define ADE9153A_AVARSIGN_MSK					NO_OS_BIT(1)
#define ADE9153A_AWSIGN_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_CONFIG2 Bit Definition */
#define ADE9153A_UPERIOD_SEL_MSK				NO_OS_BIT(12)
#define ADE9153A_HPF_CRN_MSK					NO_OS_GENMASK(11, 9)

/* ADE9153A_REG_EP_CFG Bit Definition */
#define ADE9153A_NOLOAD_TMR_MSK					NO_OS_GENMASK(7, 5)
#define ADE9153A_RD_RST_EN_MSK					NO_OS_BIT(3)
#define ADE9153A_EGY_LD_ACCUM_MSK				NO_OS_BIT(2)
#define ADE9153A_EGY_TMR_MODE_MSK				NO_OS_BIT(1)
#define ADE9153A_EGY_PWR_EN_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_CRC_FORCE Bit Definition */
#define ADE9153A_FORCE_CRC_UPDATE_MSK				NO_OS_BIT(0)

/* ADE9153A_REG_TEMP_CFG Bit Definition */
#define ADE9153A_TEMP_START_MSK					NO_OS_BIT(3)
#define ADE9153A_TEMP_EN_MSK					NO_OS_BIT(2)
#define ADE9153A_TEMP_TIME_MSK					NO_OS_GENMASK(1, 0)

/* ADE9153A_REG_TEMP_RSLT Bit Definition */
#define ADE9153A_TEMP_RESULT_MSK				NO_OS_GENMASK(11, 0)

/* ADE9153A_REG_AI_PGAGAIN Bit Definition */
#define ADE9153A_AI_SWAP_MSK					NO_OS_BIT(4)
#define ADE9153A_AI_GAIN_MSK					NO_OS_GENMASK(2, 0)

/* ADE9153A_REG_MS_STATUS_IRQ Bit Definition */
#define ADE9153A_MS_SYSRDY_MSK					NO_OS_BIT(14)
#define ADE9153A_MS_CONFERR_MSK					NO_OS_BIT(13)
#define ADE9153A_MS_ABSENT_MSK					NO_OS_BIT(12)
#define ADE9153A_MS_TIMEOUT_MSK					NO_OS_BIT(3)
#define ADE9153A_MS_READY_MSK					NO_OS_BIT(1)
#define ADE9153A_MS_SHIFT_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_EVENT_STATUS Bit Definition */
#define ADE9153A_OIB_MSK					NO_OS_BIT(5)
#define ADE9153A_OIA_MSK					NO_OS_BIT(4)
#define ADE9153A_SWELLA_MSK					NO_OS_BIT(2)
#define ADE9153A_DIPA_MSK					NO_OS_BIT(0)

/* ADE9153A_REG_CHIP_STATUS Bit Definition */
#define ADE9153A_UART_RESET_MSK					NO_OS_BIT(7)
#define ADE9153A_UART_ERROR2_MSK				NO_OS_BIT(6)
#define ADE9153A_UART_ERROR1_MSK				NO_OS_BIT(5)
#define ADE9153A_UART_ERROR0_MSK				NO_OS_BIT(4)
#define ADE9153A_ERROR3_MSK					NO_OS_BIT(3)
#define ADE9153A_ERROR2_MSK					NO_OS_BIT(2)
#define ADE9153A_ERROR1_MSK					NO_OS_BIT(1)
#define ADE9153A_ERROR0_MSK					NO_OS_BIT(0)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ade9153a_bi_gain_e` enumeration defines the possible programmable
 * gain amplifier (PGA) gain settings for Current Channel B in the
 * ADE9153A device. This enumeration allows the user to select between
 * three different gain levels (1, 2, and 4) to adjust the sensitivity of
 * the current measurement on Channel B, which can be useful for
 * optimizing the measurement range and accuracy based on the specific
 * application requirements.
 *
 * @param ADE9153A_PGA_CHB_GAIN_1 Represents a gain of 1 for the PGA on Current
 * Channel B.
 * @param ADE9153A_PGA_CHB_GAIN_2 Represents a gain of 2 for the PGA on Current
 * Channel B.
 * @param ADE9153A_PGA_CHB_GAIN_4 Represents a gain of 4 for the PGA on Current
 * Channel B.
 ******************************************************************************/
enum ade9153a_bi_gain_e {
	/* PGA gain for Current Channel B */
	/* Gain 1 */
	ADE9153A_PGA_CHB_GAIN_1,
	/* Gain 2 */
	ADE9153A_PGA_CHB_GAIN_2,
	/* Gain 4 */
	ADE9153A_PGA_CHB_GAIN_4
};

/***************************************************************************//**
 * @brief The `ade9153a_acal_ch_e` is an enumeration that defines the channels
 * available for autocalibration in the ADE9153A device. It provides
 * options to enable autocalibration on the voltage channel, Current
 * Channel B, or Current Channel A, facilitating precise calibration of
 * these channels for accurate measurements.
 *
 * @param AUTOCAL_AV Enables autocalibration on the voltage channel.
 * @param AUTOCAL_BI Enables autocalibration on Current Channel B.
 * @param AUTOCAL_AI Enables autocalibration on Current Channel A.
 ******************************************************************************/
enum ade9153a_acal_ch_e {
	/* Enable autocalibration on the voltage channel. */
	AUTOCAL_AV,
	/* Enable autocalibration on Current Channel B. */
	AUTOCAL_BI,
	/* Enable autocalibration on Current Channel A. */
	AUTOCAL_AI
};

/***************************************************************************//**
 * @brief The `ade9153a_acalmode_e` is an enumeration that defines the power
 * modes available for channels A and B in the ADE9153A device. It
 * provides two modes: NORMAL and TURBO, allowing the user to select
 * between standard and enhanced performance settings for the device's
 * operation.
 *
 * @param NORMAL Represents the normal mode of operation.
 * @param TURBO Represents the turbo mode of operation.
 ******************************************************************************/
enum ade9153a_acalmode_e {
	/* Normal mode. */
	NORMAL,
	/* Turbo mode */
	TURBO
};

/***************************************************************************//**
 * @brief The `ade9153a_selfreq_e` is an enumeration that defines the frequency
 * settings for the ADE9153A device, allowing the selection between 50 Hz
 * and 60 Hz. This is used to configure the device to operate at the
 * desired frequency, which is crucial for accurate energy measurement
 * and analysis in different regions where the power line frequency may
 * vary.
 *
 * @param F_50_HZ Represents a frequency setting of 50 Hz.
 * @param F_60_HZ Represents a frequency setting of 60 Hz.
 ******************************************************************************/
enum ade9153a_selfreq_e {
	/* 50 Hz */
	F_50_HZ,
	/* 60 Hz */
	F_60_HZ
};

/***************************************************************************//**
 * @brief The `ade9153a_pwr_settle_e` enumeration defines the possible settling
 * times for power and filter-based RMS measurements in the ADE9153A
 * device. These settling times determine how long the device waits for
 * measurements to stabilize before starting power, energy, and CF
 * accumulations. The available options are 64 ms, 128 ms, 256 ms, and 0
 * ms, allowing for flexibility in measurement timing based on
 * application requirements.
 *
 * @param ADE9153A_SETTLE_64_MS Represents a 64 ms settling time for power and
 * filter-based RMS measurements.
 * @param ADE9153A_SETTLE_128_MS Represents a 128 ms settling time for power and
 * filter-based RMS measurements.
 * @param ADE9153A_SETTLE_256_MS Represents a 256 ms settling time for power and
 * filter-based RMS measurements.
 * @param ADE9153A_SETTLE_0_MS Represents a 0 ms settling time for power and
 * filter-based RMS measurements.
 ******************************************************************************/
enum ade9153a_pwr_settle_e {
	/* Configure the time for the power and filter-based */
	/* rms measurements to settle before starting the */
	/* power, energy, and CF accumulations */
	/* 64 ms settle */
	ADE9153A_SETTLE_64_MS,
	/* 128 ms settle */
	ADE9153A_SETTLE_128_MS,
	/* 256 ms settle */
	ADE9153A_SETTLE_256_MS,
	/* 0 ms settle */
	ADE9153A_SETTLE_0_MS
};

/***************************************************************************//**
 * @brief The `ade9153a_cf2sel_e` enumeration defines the types of energy
 * outputs that can be configured at the CF2 pin of the ADE9153A device.
 * It includes options for total active power, total apparent power, and
 * fundamental reactive power, allowing users to select the desired
 * energy measurement for their application. This configuration is
 * crucial for applications that require specific energy monitoring and
 * management.
 *
 * @param ADE9153A_TOTAL_ACTIVE_POWER Represents the total active power
 * measurement.
 * @param ADE9153A_TOTAL_APPARENT_POWER Represents the total apparent power
 * measurement.
 * @param ADE9153A_TOTAL_FUNDAMENTAL_REACTIVE_POWER Represents the fundamental
 * reactive power measurement.
 ******************************************************************************/
enum ade9153a_cf2sel_e {
	/* Configure the time for the power and filter-based */
	/* rms measurements to settle before starting the */
	/* power, energy, and CF accumulations */
	/* Total active power */
	ADE9153A_TOTAL_ACTIVE_POWER,
	/* Total apparent power */
	ADE9153A_TOTAL_APPARENT_POWER,
	/* Fundamental reactive power */
	ADE9153A_TOTAL_FUNDAMENTAL_REACTIVE_POWER
};

/***************************************************************************//**
 * @brief The `ade9153a_accmode_e` enumeration defines the different
 * accumulation modes for energy registers and CFx pulses in the ADE9153A
 * device. These modes determine how the device accumulates energy data,
 * allowing for signed, absolute, positive, or negative accumulation.
 * This flexibility is crucial for accurately measuring and processing
 * energy data in various applications, such as power monitoring and
 * energy management systems.
 *
 * @param ADE9153A_SIGNED_ACC_MODE Represents the signed accumulation mode for
 * energy registers and CFx pulses.
 * @param ADE9153A_ABSOLUTE_VAL_ACC_MODE Represents the absolute value
 * accumulation mode for energy registers
 * and CFx pulses.
 * @param ADE9153A_POSITIVE_ACC_MODE Represents the positive accumulation mode
 * for energy registers and CFx pulses.
 * @param ADE9153A_NEGATIVE_ACC_MODE Represents the negative accumulation mode
 * for energy registers and CFx pulses.
 ******************************************************************************/
enum ade9153a_accmode_e {
	/* Fundamental reactive power / total active power accumulation mode */
	/* for energy registers and CFx pulses */
	/* Signed accumulation mode */
	ADE9153A_SIGNED_ACC_MODE,
	/* Absolute value accumulation mode */
	ADE9153A_ABSOLUTE_VAL_ACC_MODE,
	/* Positive accumulation mode */
	ADE9153A_POSITIVE_ACC_MODE,
	/* Negative accumulation mode */
	ADE9153A_NEGATIVE_ACC_MODE
};

/***************************************************************************//**
 * @brief The `ade9153a_peak_sel_e` enumeration defines the configuration
 * options for peak detection in the ADE9153A device, specifically for
 * selecting which phases (A or B) have their voltage and current peak
 * detection enabled or disabled. This allows for flexible configuration
 * of the device's peak detection capabilities, enabling users to tailor
 * the detection to specific phases as needed.
 *
 * @param ADE9153A_PEAK_DETECTION_DISABLE_PHA_PHB Disables peak detection for
 * both Phase A and Phase B for
 * voltage and current.
 * @param ADE9153A_PEAK_DETECTION_ENABLE_V_I_PHA Enables voltage and current
 * peak detection for Phase A,
 * while disabling current peak
 * detection for Phase B.
 * @param ADE9153A_PEAK_DETECTION_ENABLE_I_PHB Enables current peak detection
 * for Phase B, while disabling
 * voltage and current peak
 * detection for Phase A.
 * @param ADE9153A_PEAK_DETECTION_ENABLE_V_I_PHA_PHB Enables peak detection for
 * both voltage and current in
 * Phase A and Phase B.
 ******************************************************************************/
enum ade9153a_peak_sel_e {
	/* Peak detection phase selection */
	/* Phase A and Phase B disabled from voltage and current peak detection */
	ADE9153A_PEAK_DETECTION_DISABLE_PHA_PHB,
	/* Phase A Voltage and current peak detection enabled, */
	/* Phase B current peak detection disabled. */
	ADE9153A_PEAK_DETECTION_ENABLE_V_I_PHA,
	/* Phase A Voltage and current peak detection disabled */
	/* Phase B current peak detection enabled. */
	ADE9153A_PEAK_DETECTION_ENABLE_I_PHB,
	/* Phase A & Phase B peak detection enabled. */
	ADE9153A_PEAK_DETECTION_ENABLE_V_I_PHA_PHB
};

/***************************************************************************//**
 * @brief The `ade9153a_hpf_crn_e` is an enumeration that defines various high-
 * pass filter corner frequencies for the ADE9153A device. These corner
 * frequencies are used to configure the high-pass filter settings in the
 * device, which can be enabled when the HPFDIS bit in the CONFIG0
 * register is set to zero. Each enumerator corresponds to a specific
 * frequency value, allowing the user to select the desired filter corner
 * frequency for their application.
 *
 * @param ADE9153A_HPF_CORNER_38_695_HZ Represents a high-pass filter corner
 * frequency of 38.695 Hz.
 * @param ADE9153A_HPF_CORNER_19_6375_HZ Represents a high-pass filter corner
 * frequency of 19.6375 Hz.
 * @param ADE9153A_HPF_CORNER_9_895_HZ Represents a high-pass filter corner
 * frequency of 9.895 Hz.
 * @param ADE9153A_HPF_CORNER_4_9675_HZ Represents a high-pass filter corner
 * frequency of 4.9675 Hz.
 * @param ADE9153A_HPF_CORNER_2_49_HZ Represents a high-pass filter corner
 * frequency of 2.49 Hz.
 * @param ADE9153A_HPF_CORNER_1_2475_HZ Represents a high-pass filter corner
 * frequency of 1.2475 Hz.
 * @param ADE9153A_HPF_CORNER_0_625_HZ Represents a high-pass filter corner
 * frequency of 0.625 Hz.
 * @param ADE9153A_HPF_CORNER_0_3125_HZ Represents a high-pass filter corner
 * frequency of 0.3125 Hz.
 ******************************************************************************/
enum ade9153a_hpf_crn_e {
	/* Enabled when the HPFDIS bit in the CONFIG0 register is equal to zero */
	/* 38.695 Hz */
	ADE9153A_HPF_CORNER_38_695_HZ,
	/* 19.6375 Hz */
	ADE9153A_HPF_CORNER_19_6375_HZ,
	/* 9.895 Hz */
	ADE9153A_HPF_CORNER_9_895_HZ,
	/* 4.9675 Hz */
	ADE9153A_HPF_CORNER_4_9675_HZ,
	/* 2.49 Hz */
	ADE9153A_HPF_CORNER_2_49_HZ,
	/* 1.2475 Hz */
	ADE9153A_HPF_CORNER_1_2475_HZ,
	/* 0.625 Hz */
	ADE9153A_HPF_CORNER_0_625_HZ,
	/* 0.3125 Hz */
	ADE9153A_HPF_CORNER_0_3125_HZ
};

/***************************************************************************//**
 * @brief The `ade9153a_noload_tmr_e` is an enumeration that defines various
 * configurations for evaluating the no load condition in the ADE9153A
 * device. Each enumerator specifies the number of 4 kSPS samples over
 * which the no load condition is assessed, ranging from 64 to 4096
 * samples, with an option to disable the no load threshold evaluation
 * entirely. This allows for flexible configuration of the no load
 * detection mechanism based on the specific requirements of the
 * application.
 *
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_64 Represents a configuration for
 * evaluating the no load condition over
 * 64 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_128 Represents a configuration for
 * evaluating the no load condition over
 * 128 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_256 Represents a configuration for
 * evaluating the no load condition over
 * 256 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_512 Represents a configuration for
 * evaluating the no load condition over
 * 512 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_1024 Represents a configuration for
 * evaluating the no load condition over
 * 1024 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_2048 Represents a configuration for
 * evaluating the no load condition over
 * 2048 samples.
 * @param ADE9153A_NOLOAD_TMR_SAMPLES_4096 Represents a configuration for
 * evaluating the no load condition over
 * 4096 samples.
 * @param ADE9153A_NOLOAD_TMR_DISABLE Disables the no load threshold evaluation.
 ******************************************************************************/
enum ade9153a_noload_tmr_e {
	/* Configures how many 4 kSPS samples over which to evaluate */
	/* the no load condition */
	/* 64 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_64,
	/* 128 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_128,
	/* 256 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_256,
	/* 512 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_512,
	/* 1024 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_1024,
	/* 2048 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_2048,
	/* 4096 samples */
	ADE9153A_NOLOAD_TMR_SAMPLES_4096,
	/* Disable no load threshold */
	ADE9153A_NOLOAD_TMR_DISABLE
};

/***************************************************************************//**
 * @brief The `ade9153a_temp_time_e` enumeration defines different intervals for
 * averaging temperature readings in the ADE9153A device. Each enumerator
 * specifies a distinct time interval at which new temperature
 * measurements are taken, ranging from every 1 millisecond to every 1
 * second. This allows for flexible configuration of temperature
 * measurement frequency based on application requirements.
 *
 * @param ADE9153A_TEMP_TIME_SAMPLES_1 New temperature measurement every 1ms.
 * @param ADE9153A_TEMP_TIME_SAMPLES_256 New temperature measurement every 256
 * ms.
 * @param ADE9153A_TEMP_TIME_SAMPLES_512 New temperature measurement every 512
 * ms.
 * @param ADE9153A_TEMP_TIME_SAMPLES_1024 New temperature measurement every 1
 * sec.
 ******************************************************************************/
enum ade9153a_temp_time_e {
	/* Select the number of temperature readings to average*/
	/* New temperature measurement every 1ms */
	ADE9153A_TEMP_TIME_SAMPLES_1,
	/* New temperature measurement every 256 ms */
	ADE9153A_TEMP_TIME_SAMPLES_256,
	/* New temperature measurement every 512 ms */
	ADE9153A_TEMP_TIME_SAMPLES_512,
	/* New temperature measurement every 1 sec */
	ADE9153A_TEMP_TIME_SAMPLES_1024
};

/***************************************************************************//**
 * @brief The `ade9153a_ai_gain_e` enumeration defines the programmable gain
 * amplifier (PGA) gain settings for the current channel A in the
 * ADE9153A device. Each enumerator corresponds to a specific gain value,
 * allowing the user to select the appropriate gain setting for their
 * application. This is crucial for adjusting the input signal level to
 * the ADC, ensuring optimal performance and accuracy in current
 * measurements.
 *
 * @param ADE9153A_AI_GAIN_16 Represents a PGA gain of 16 for current channel A.
 * @param ADE9153A_AI_GAIN_24 Represents a PGA gain of 24 for current channel A.
 * @param ADE9153A_AI_GAIN_32 Represents a PGA gain of 32 for current channel A.
 * @param ADE9153A_AI_GAIN_38_4 Represents a PGA gain of 38.4 for current
 * channel A.
 ******************************************************************************/
enum ade9153a_ai_gain_e {
	/* PGA gain*/
	/* Gain = 16 */
	ADE9153A_AI_GAIN_16 = 2,
	/* Gain = 24 */
	ADE9153A_AI_GAIN_24 = 3,
	/* Gain = 32 */
	ADE9153A_AI_GAIN_32 = 4,
	/* Gain = 38.4 */
	ADE9153A_AI_GAIN_38_4 = 5
};

/***************************************************************************//**
 * @brief The `ade9153a_chip_stat_err_e` is an enumeration that defines various
 * error states for the ADE9153A chip, including general errors and UART-
 * specific errors. Each error state is associated with a specific reset
 * action required to clear the error, either through software/hardware
 * reset or UART reset. This enumeration is used to identify and handle
 * different error conditions that may occur during the operation of the
 * ADE9153A chip.
 *
 * @param ADE9153A_ERROR0 Represents Error 0, which requires a software or
 * hardware reset to clear.
 * @param ADE9153A_ERROR1 Represents Error 1, which requires a software or
 * hardware reset to clear.
 * @param ADE9153A_ERROR2 Represents Error 2, which requires a software or
 * hardware reset to clear.
 * @param ADE9153A_ERROR3 Represents Error 3, which requires a software or
 * hardware reset to clear.
 * @param ADE9153A_UART_ERROR0 Represents UART Error 0, which requires a UART
 * reset to clear.
 * @param ADE9153A_UART_ERROR1 Represents UART Error 1, which requires a UART
 * reset to clear.
 * @param ADE9153A_UART_ERROR2 Represents UART Error 2, which requires a UART
 * reset to clear.
 * @param ADE9153A_UART_RESET Indicates that a UART interface reset has been
 * detected.
 ******************************************************************************/
enum ade9153a_chip_stat_err_e {
	/* Error 0 - SW/HW reset to clear */
	ADE9153A_ERROR0,
	/* Error 1 - SW/HW reset to clear */
	ADE9153A_ERROR1,
	/* Error 2 - SW/HW reset to clear */
	ADE9153A_ERROR2,
	/* Error 3 - SW/HW reset to clear */
	ADE9153A_ERROR3,
	/* UART Error 0 - UART reset to clear */
	ADE9153A_UART_ERROR0,
	/* UART Error 1 - UART reset to clear */
	ADE9153A_UART_ERROR1,
	/* UART Error 2 - UART reset to clear */
	ADE9153A_UART_ERROR2,
	/* UART interface reset detected */
	ADE9153A_UART_RESET
};

/***************************************************************************//**
 * @brief The `ade9153a_phnoload_e` enumeration defines constants that represent
 * different types of energy in a no load condition for Phase A in the
 * ADE9153A energy metering IC. Each enumerator corresponds to a specific
 * type of energy measurement (fundamental reactive, total apparent, and
 * total active) that is considered to be in a no load state, which is
 * useful for energy monitoring and management applications.
 *
 * @param AFVARNL Represents Phase A fundamental reactive energy in a no load
 * condition.
 * @param AVANL Represents Phase A total apparent energy in a no load condition.
 * @param AWATTNL Represents Phase A total active energy in a no load condition.
 ******************************************************************************/
enum ade9153a_phnoload_e {
	/* No load */
	/* Phase A fundamental reactive energy is in no load. */
	AFVARNL = 1,
	/* Phase A total apparent energy is in no load */
	AVANL,
	/* Phase A total active energy is in no load */
	AWATTNL
};

/***************************************************************************//**
 * @brief The `dip_swell_irq_mode_en` is an enumeration that defines the modes
 * of interrupt handling for dip and swell conditions in a system. It
 * provides two modes: `CONTINUOUSE`, which continuously triggers
 * interrupts during the condition, and `ONE_INT`, which triggers an
 * interrupt when the condition is entered and another when it is exited.
 * This allows for flexible handling of power quality events in systems
 * using the ADE9153A driver.
 *
 * @param CONTINUOUSE Represents a continuous interrupt mode for dip and swell
 * conditions.
 * @param ONE_INT Represents a mode where one interrupt is triggered upon
 * entering a condition and another upon exiting.
 ******************************************************************************/
enum dip_swell_irq_mode_en {
	/* continuous */
	CONTINUOUSE,
	/* One interrupt when entering cond. another when exiting cond. */
	ONE_INT,
};

/***************************************************************************//**
 * @brief The `ade9153a_init_param` structure is used to define the
 * initialization parameters for the ADE9153A device, which is an energy
 * metering IC. This structure includes various configuration settings
 * such as SPI and GPIO initialization parameters, interrupt control, and
 * operational settings like AI gain and high pass filter corner
 * frequency. It also allows for the specification of an external
 * callback function for handling interrupts, providing flexibility in
 * how the device is integrated into a system. The structure is essential
 * for setting up the device's communication and operational parameters
 * before it is used in an application.
 *
 * @param spi_init Pointer to the SPI initialization parameters for device
 * communication.
 * @param gpio_rdy Pointer to the GPIO initialization parameters for the RDY
 * signal, indicating ADC data availability.
 * @param gpio_reset Pointer to the GPIO initialization parameters for the RESET
 * signal, used for hardware reset of the device.
 * @param gpio_ss Pointer to the SPI initialization parameters for the SS (Slave
 * Select) signal configuration.
 * @param gpio_sck Pointer to the SPI initialization parameters for the SCK
 * (Serial Clock) signal configuration.
 * @param spi_en Flag to enable or disable the SPI interface.
 * @param irq_ctrl Pointer to the IRQ control descriptor for handling GPIO RDY
 * interrupt routines.
 * @param ai_swap Flag indicating the operation mode of the sensor.
 * @param ai_pga_gain Initial value for the AI gain, defined by the
 * ade9153a_ai_gain_e enumeration.
 * @param hpf_crn Initial value for the high pass filter corner frequency,
 * defined by the ade9153a_hpf_crn_e enumeration.
 * @param freq Frequency selection for energy accumulation, defined by the
 * ade9153a_selfreq_e enumeration.
 * @param vlevel Initial value for the Vlevel parameter.
 * @param rsmall Initial value for the Vdiv Rsmall parameter.
 * @param no_samples Number of samples for energy accumulation.
 * @param ai_gain Initial value for the AI gain.
 * @param drdy_callback Pointer to an external callback function for handling
 * GPIO RDY interrupts, or NULL if the driver-defined
 * callback is used.
 ******************************************************************************/
struct ade9153a_init_param {
	/* Device communication descriptor */
	struct no_os_spi_init_param 	*spi_init;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_init_param	*gpio_rdy;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_init_param  	*gpio_reset;
	/** GPIO ss descriptor used to config comms */
	struct no_os_spi_init_param 	*gpio_ss;
	/** GPIO sck descriptor used to config comms */
	struct no_os_spi_init_param	*gpio_sck;
	/** Enable SPI interface */
	uint8_t				spi_en;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** operation of sensor */
	uint8_t				ai_swap;
	/** AI gain init value */
	enum ade9153a_ai_gain_e		ai_pga_gain;
	/** High pass filter corner freq init value */
	enum ade9153a_hpf_crn_e		hpf_crn;
	/** Energy accumulation freq select */
	enum ade9153a_selfreq_e		freq;
	/** Vlevel value */
	uint32_t			vlevel;
	/** Vdiv Rsmall */
	uint32_t			rsmall;
	/** Energy accumulation sample no */
	uint32_t			no_samples;
	/** Ai Gain */
	uint32_t			ai_gain;
	/** External callback used to handle interrupt routine for GPIO RDY */
	/** Set to NULL if callback defined in driver used */
	void (*drdy_callback)(void *context);
};

/***************************************************************************//**
 * @brief The `ade9153a_dev` structure is a comprehensive representation of the
 * ADE9153A device, encapsulating all necessary descriptors and
 * configurations for communication and control. It includes SPI and GPIO
 * descriptors for managing device communication and hardware resets, as
 * well as interrupt control and callback descriptors for handling data-
 * ready signals. This structure is essential for initializing and
 * operating the ADE9153A device, providing a centralized configuration
 * for its various interfaces and operational modes.
 *
 * @param spi_desc Pointer to the SPI communication descriptor for the device.
 * @param burst_en 8-bit unsigned integer indicating if burst mode is enabled.
 * @param gpio_rdy Pointer to the GPIO descriptor used to signal when ADC data
 * is available.
 * @param gpio_reset Pointer to the GPIO descriptor used to reset the device
 * (hardware reset).
 * @param gpio_ss Pointer to the GPIO descriptor used to set up communications.
 * @param gpio_sck Pointer to the GPIO descriptor used to set up communications.
 * @param irq_ctrl Pointer to the IRQ control descriptor used to handle
 * interrupt routines for GPIO RDY.
 * @param irq_cb Descriptor for the IRQ callback used to handle interrupt
 * routines for GPIO RDY.
 ******************************************************************************/
struct ade9153a_dev {
	/** Device communication descriptor */
	struct no_os_spi_desc		*spi_desc;
	/** Burst mode selector */
	uint8_t				burst_en;
	/** GPIO RDY descriptor used to signal when ADC data is available */
	struct no_os_gpio_desc  	*gpio_rdy;
	/** GPIO RESET descriptor used to reset device (HW reset) */
	struct no_os_gpio_desc  	*gpio_reset;
	/** GPIO SS descriptor used to setup comms */
	struct no_os_gpio_desc 		*gpio_ss;
	/** GPIO sck descriptor used to setup comms  */
	struct no_os_gpio_desc		*gpio_sck;
	/** IRQ device descriptor used to handle interrupt routine for GPIO RDY */
	struct no_os_irq_ctrl_desc 	*irq_ctrl;
	/** IRQ callback used to handle interrupt routine for GPIO RDY */
	struct no_os_callback_desc	irq_cb;
};

/***************************************************************************//**
 * @brief The `ade9153a_energy_values` structure is designed to hold energy
 * register values for the ADE9153A energy metering IC. It contains three
 * 32-bit integer fields that store the active energy, fundamental
 * reactive energy, and apparent energy register values. This structure
 * is used to encapsulate the energy data read from the ADE9153A device,
 * facilitating the management and processing of energy measurements in
 * applications that utilize this IC for energy monitoring and analysis.
 *
 * @param active_energy_reg_val Stores the active energy register value as a
 * 32-bit integer.
 * @param fundamental_reactive_energy_reg_val Stores the fundamental reactive
 * energy register value as a 32-bit
 * integer.
 * @param apparent_energy_reg_val Stores the apparent energy register value as a
 * 32-bit integer.
 ******************************************************************************/
struct ade9153a_energy_values {
	/** Active energy register value */
	int32_t active_energy_reg_val;
	/** Fundamental reactive energy register value */
	int32_t fundamental_reactive_energy_reg_val;
	/** Apparent energy register value */
	int32_t apparent_energy_reg_val;
};

/***************************************************************************//**
 * @brief The `ade9153a_power_values` structure is designed to hold power-
 * related register values for the ADE9153A energy metering IC. It
 * contains three 32-bit integer fields that store the active power,
 * fundamental reactive power, and apparent power register values. This
 * structure is used to encapsulate the power measurement data retrieved
 * from the ADE9153A device, facilitating easy access and manipulation of
 * these values in applications that require energy monitoring and
 * analysis.
 *
 * @param active_power_reg_val Stores the active power register value as a
 * 32-bit integer.
 * @param fundamental_reactive_power_reg_val Stores the fundamental reactive
 * power register value as a 32-bit
 * integer.
 * @param apparent_power_reg_val Stores the apparent power register value as a
 * 32-bit integer.
 ******************************************************************************/
struct ade9153a_power_values {
	/** Active power register value */
	int32_t active_power_reg_val;
	/** Fundamental reactive power register value */
	int32_t fundamental_reactive_power_reg_val;
	/** Apparent power register value */
	int32_t apparent_power_reg_val;
};

/***************************************************************************//**
 * @brief The `ade9153a_rms_values` structure is designed to hold the root mean
 * square (RMS) values for current and voltage as measured by the
 * ADE9153A energy metering IC. It contains two 32-bit integer fields
 * that store the RMS register values for current and voltage, which are
 * essential for accurate power measurement and analysis in energy
 * monitoring applications.
 *
 * @param current_rms_reg_val Stores the current RMS register value as a 32-bit
 * integer.
 * @param voltage_rms_reg_val Stores the voltage RMS register value as a 32-bit
 * integer.
 ******************************************************************************/
struct ade9153a_rms_values {
	/** Current rms register value */
	int32_t current_rms_reg_val;
	/** Voltage rms register value */
	int32_t voltage_rms_reg_val;
	/** Current rms value */
};

/***************************************************************************//**
 * @brief The `ade9153a_half_rms_values` structure is designed to hold half RMS
 * values for current and voltage, which are essential for power
 * measurement and analysis in electrical systems. This structure
 * contains two 32-bit integer fields, each representing the half RMS
 * register values for current and voltage, respectively. These values
 * are typically used in applications involving the ADE9153A energy
 * metering IC, which requires precise measurement of electrical
 * parameters for accurate energy consumption monitoring and analysis.
 *
 * @param current_h_rms_reg_val Stores the current half RMS register value as a
 * 32-bit integer.
 * @param voltage_h_rms_reg_val Stores the voltage half RMS register value as a
 * 32-bit integer.
 ******************************************************************************/
struct ade9153a_half_rms_values {
	/** Current half rms register value */
	int32_t current_h_rms_reg_val;
	/** Voltage half rms register value */
	int32_t voltage_h_rms_reg_val;
};

/***************************************************************************//**
 * @brief The `ade9153a_pq_values` structure is designed to hold power quality
 * register values for the ADE9153A device. It includes fields for
 * storing the power factor, period, and angle between AI and AV, all of
 * which are critical for assessing the power quality in electrical
 * systems. Each field is represented as a 32-bit integer, allowing for
 * precise storage and manipulation of these values.
 *
 * @param power_factor_reg_val Stores the power factor register value as a
 * 32-bit integer.
 * @param period_reg_val Stores the period register value as a 32-bit integer.
 * @param angle_ai_av_reg_val Stores the angle between AI and AV register value
 * as a 32-bit integer.
 ******************************************************************************/
struct ade9153a_pq_values {
	/** Power factor register value */
	int32_t power_factor_reg_val;
	/** Period register value */
	int32_t period_reg_val;
	/** Angle AV AI register value */
	int32_t angle_ai_av_reg_val;
};

/***************************************************************************//**
 * @brief The `ade9153a_temperature_value` structure is designed to encapsulate
 * temperature-related data for the ADE9153A device. It includes fields
 * for storing the raw temperature register value, as well as calibration
 * parameters such as offset and gain, which are essential for accurate
 * temperature measurement and compensation. This structure is likely
 * used in conjunction with functions that read and process temperature
 * data from the ADE9153A sensor.
 *
 * @param temperature_reg_val Stores the temperature register value as a signed
 * 16-bit integer.
 * @param offset_reg_val Holds the offset register value as an unsigned 16-bit
 * integer.
 * @param gain_reg_val Contains the gain register value as an unsigned 16-bit
 * integer.
 ******************************************************************************/
struct ade9153a_temperature_value {
	/** Temperature register value */
	int16_t temperature_reg_val;
	/** Offset register value */
	uint16_t offset_reg_val;
	/** Gain register value */
	uint16_t gain_reg_val;
};

/***************************************************************************//**
 * @brief The `ade9153a_autocal_vals` structure is designed to hold calibration
 * values for the ADE9153A device, specifically for auto-calibration
 * purposes. It contains four 32-bit unsigned integer fields, each
 * representing a different calibration register value: AICC, AICERT,
 * AVCC, and AVCERT. These values are crucial for ensuring the accuracy
 * and reliability of the device's measurements by storing the results of
 * the auto-calibration process.
 *
 * @param aicc_reg_val Stores the AICC register value as a 32-bit unsigned
 * integer.
 * @param aicert_reg_val Stores the AICERT register value as a 32-bit unsigned
 * integer.
 * @param avcc_reg_val Stores the AVCC register value as a 32-bit unsigned
 * integer.
 * @param avcert_reg_val Stores the AVCERT register value as a 32-bit unsigned
 * integer.
 ******************************************************************************/
struct ade9153a_autocal_vals {
	/** AICC value */
	uint32_t aicc_reg_val;
	/** AICERT value */
	uint32_t aicert_reg_val;
	/** AVCC value */
	uint32_t avcc_reg_val;
	/** AVCERT value */
	uint32_t avcert_reg_val;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

// Select comms interface SPI.
/***************************************************************************//**
 * @brief This function is used to configure the ADE9153A device to communicate
 * over the SPI interface. It must be called after the device has been
 * initialized and before any data transactions occur. If the `dev`
 * parameter is null, the function will return an error. The function
 * also sets the state of the GPIO pins for the SPI interface,
 * specifically setting the Slave Select (SS) pin low and the Serial
 * Clock (SCK) pin high, which are necessary for proper SPI operation.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENOMEM.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_set_interface_spi(struct ade9153a_dev *dev);

// Select comms interface Serial.
/***************************************************************************//**
 * @brief This function is intended to configure the serial communication
 * interface for the ADE9153A device. It must be called after the device
 * has been initialized and before any communication occurs. The function
 * will set the GPIO pins for the serial interface to their appropriate
 * states, ensuring that the device is ready for operation. If the
 * provided device structure is null, the function will return an error.
 * It is important to handle any potential errors returned by this
 * function, as they indicate issues with GPIO configuration or device
 * reset.
 *
 * @param dev A pointer to a `struct ade9153a_dev` that represents the device.
 * This pointer must not be null, as the function will return -ENOMEM
 * if it is. The caller retains ownership of this structure.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure that occurred.
 ******************************************************************************/
int ade9153a_set_interface_serial(struct ade9153a_dev *dev);

// GPIO interrupt handler for data ready.
/***************************************************************************//**
 * @brief This function is intended to be called in response to an interrupt
 * signal indicating that data is ready to be processed from the ADE9153A
 * device. It temporarily disables interrupts while reading data to
 * ensure data integrity, and then re-enables interrupts after the read
 * operation. It is crucial to ensure that this function is invoked in an
 * appropriate interrupt context, and it should not be called if the
 * device has not been properly initialized.
 *
 * @param dev A pointer to the device structure (`struct ade9153a_dev`)
 * representing the ADE9153A device. This pointer must not be null
 * and should point to a valid device instance that has been
 * initialized.
 * @return None
 ******************************************************************************/
static void ade9153a_irq_handler(void *dev);

// Initialize the device.
/***************************************************************************//**
 * @brief This function is used to initialize the ADE9153A device, setting up
 * the necessary GPIOs, interrupts, and communication interfaces. It must
 * be called with a valid `init_param` structure, particularly ensuring
 * that the `irq_ctrl` field is not null. If the initialization is
 * successful, a pointer to the device structure is returned through the
 * `device` parameter. The function handles various error conditions,
 * such as memory allocation failures and GPIO configuration issues, and
 * will clean up any allocated resources before returning an error code.
 *
 * @param device A pointer to a pointer of `struct ade9153a_dev`. The caller
 * must provide a valid pointer that will be set to the
 * initialized device structure. This pointer will be allocated by
 * the function.
 * @param init_param A structure of type `struct ade9153a_init_param` containing
 * initialization parameters. The `irq_ctrl` field must not be
 * null, and the GPIO parameters must be valid. The function
 * will return an error if any of these parameters are
 * invalid.
 * @return Returns 0 on successful initialization, or a negative error code
 * indicating the type of failure.
 ******************************************************************************/
int ade9153a_init(struct ade9153a_dev **device,
		  struct ade9153a_init_param init_param);

// ADE9153A setup.
/***************************************************************************//**
 * @brief This function initializes the ADE9153A device with the specified
 * parameters. It must be called after the device has been properly
 * initialized and before any measurements or operations are performed.
 * The function configures various settings such as gain, filter
 * frequency, and energy accumulation mode. If any of the provided
 * parameters are invalid or if the device is not ready, the function
 * will return an error code.
 *
 * @param dev A pointer to the device structure. Must not be null; if null, the
 * function returns -ENODEV.
 * @param init_param A structure containing initialization parameters for the
 * ADE9153A device. This includes settings for AI swap, PGA
 * gain, high-pass filter corner frequency, energy
 * accumulation frequency, voltage level, and other
 * configuration values. All fields must be set to valid
 * values as per the device specifications.
 * @return Returns 0 on success, or a negative error code if any configuration
 * step fails.
 ******************************************************************************/
int ade9153a_setup(void *dev, struct ade9153a_init_param init_param);

// Read device register.
/***************************************************************************//**
 * @brief This function is used to read a specified register from the ADE9153A
 * device. It must be called after the device has been properly
 * initialized. The `reg_addr` parameter specifies the address of the
 * register to read, which must be within the valid range defined for the
 * device. The function will populate the `reg_data` pointer with the
 * value read from the register. If the device is not initialized or if
 * the `reg_data` pointer is null, the function will return an error
 * code. Additionally, if the read operation fails due to communication
 * issues, appropriate error codes will be returned.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to read. It must be within the
 * valid range of register addresses defined for the ADE9153A
 * device.
 * @param reg_data Pointer to a `uint32_t` where the read data will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_read(struct ade9153a_dev *dev, uint16_t reg_addr,
		  uint32_t *reg_data);

// Write device register.
/***************************************************************************//**
 * @brief This function is used to write a value to a specific register of the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. The function takes a pointer to the device
 * structure, the register address, and the data to be written. If the
 * register address is within a specific range, the data is written as a
 * 16-bit value; otherwise, it is written as a 32-bit value. The function
 * handles invalid device pointers by returning an error code. It is
 * important to check the return value to ensure that the write operation
 * was successful.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to write to. Valid values are
 * defined by the ADE9153A register map.
 * @param reg_data The data to be written to the register. The data type is a
 * 32-bit unsigned integer.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_write(struct ade9153a_dev *dev, uint16_t reg_addr,
		   uint32_t reg_data);

// Update specific register bits.
/***************************************************************************//**
 * @brief This function is used to modify specific bits of a register in the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. The function reads the current value of the
 * specified register, applies the provided mask to clear the bits that
 * need to be updated, and then sets the new bits as specified by the
 * `reg_data` parameter. If the device pointer is null or if reading the
 * register fails, the function will return an error code. It is
 * important to ensure that the mask and data provided are valid for the
 * register being modified.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register to be updated. This should be
 * within the valid range of register addresses defined for the
 * ADE9153A.
 * @param mask A 32-bit mask used to specify which bits in the register should
 * be updated. The mask should be properly defined to avoid
 * unintended modifications.
 * @param reg_data A 32-bit value containing the new data to be written to the
 * register. Only the bits specified by the mask will be
 * affected.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
static int ade9153a_update_bits(struct ade9153a_dev *dev, uint16_t reg_addr,
				uint32_t mask, uint32_t reg_data);

// Remove the device and release resources.
/***************************************************************************//**
 * @brief This function is intended to be called when the device is no longer
 * needed, allowing for proper cleanup of resources associated with the
 * `ade9153a_dev` structure. It is crucial to ensure that this function
 * is called only after the device has been initialized and is no longer
 * in use. The function will handle the removal of SPI and GPIO
 * resources, unregister any interrupt callbacks, and free the memory
 * allocated for the device structure. If the `dev` parameter is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device to
 * be removed. Must not be null; otherwise, the function will return
 * -ENODEV.
 * @return Returns 0 on successful removal of the device and associated
 * resources. If any resource removal fails, the corresponding error
 * code is returned.
 ******************************************************************************/
int ade9153a_remove(struct ade9153a_dev *dev);

// Reset the device using SW reset.
/***************************************************************************//**
 * @brief This function is used to perform a software reset on the ADE9153A
 * device. It should be called when the device needs to be reset to its
 * initial state, typically after initialization or when recovering from
 * an error state. The function checks if the provided device pointer is
 * valid; if it is null, it returns an error code. After initiating the
 * reset, the function waits for a predefined delay to allow the device
 * to complete the reset process before returning.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_sw_reset(struct ade9153a_dev *dev);

// Reset the device using HW reset.
/***************************************************************************//**
 * @brief This function is used to perform a hardware reset on the ADE9153A
 * device. It should be called when the device needs to be reset to its
 * initial state, typically after initialization or when recovering from
 * an error state. The function expects a valid pointer to an
 * `ade9153a_dev` structure, which must be initialized prior to calling
 * this function. If the provided device pointer is null, the function
 * will return an error code. The reset process involves setting a GPIO
 * pin low, waiting for a specified delay, and then setting the GPIO pin
 * high again. It is important to ensure that the device is properly
 * initialized before invoking this function.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device to
 * reset. Must not be null; otherwise, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_hw_reset(struct ade9153a_dev *dev);

// Lock device.
/***************************************************************************//**
 * @brief This function should be called to lock the device before performing
 * write operations to its registers. It is essential to ensure that the
 * device is properly initialized and that the `dev` parameter is not
 * null. If the device is not initialized (i.e., `dev` is null), the
 * function will return an error code indicating that the device is not
 * available. The function will return a success code if the lock
 * operation is successful, or an error code if the operation fails.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null, as it indicates the device to be
 * locked. If it is null, the function will return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_wr_lock(struct ade9153a_dev *dev);

// Unlock device.
/***************************************************************************//**
 * @brief This function is used to unlock the device, allowing subsequent write
 * operations to its registers. It should be called only after the device
 * has been properly initialized. If the provided device pointer is null,
 * the function will return an error code indicating that the device is
 * not available. Successful execution of this function is necessary
 * before any write operations can be performed on the device.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV). The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_wr_unlock(struct ade9153a_dev *dev);

// Version product
/***************************************************************************//**
 * @brief This function is used to obtain the version product information from
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. The function expects a valid pointer to a
 * `struct ade9153a_dev` representing the device and a pointer to a
 * `uint32_t` variable where the version product will be stored. If the
 * device pointer is null, or if the data_read pointer is null, the
 * function will return an error code. The version product is read from
 * the device's register, and the function will return the appropriate
 * error code if the read operation fails.
 *
 * @param dev Pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param data_read Pointer to a `uint32_t` where the version product will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_version_product(struct ade9153a_dev *dev, uint32_t *data_read);

// Disable the low pass filter in the fundamental reactive power datapath.
/***************************************************************************//**
 * @brief This function is intended to be called when you want to disable the
 * low pass filter in the fundamental reactive power datapath of the
 * ADE9153A device. It is important to ensure that the `dev` parameter is
 * properly initialized and not null before calling this function. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. The function will update
 * the relevant register bits to disable the low pass filter, and it may
 * return an error code if the update operation fails.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_lpf_rp_disable(struct ade9153a_dev *dev);

// Disable the low-pass filter in the total active power datapath.
/***************************************************************************//**
 * @brief This function is intended to be called when you need to disable the
 * low-pass filter in the total active power datapath of the ADE9153A
 * device. It is crucial to ensure that the `dev` parameter is properly
 * initialized and points to a valid `ade9153a_dev` structure before
 * calling this function. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * The function modifies the device's configuration register to disable
 * the low-pass filter, which may affect the behavior of the power
 * measurements.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code (-ENODEV). The caller retains ownership of
 * the `dev` pointer.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_lpf_ap_disable(struct ade9153a_dev *dev);

// Use the nominal phase voltage rms, VNOM, in the computation
// of the Phase A total apparent power.
/***************************************************************************//**
 * @brief This function should be called to enable the nominal phase voltage RMS
 * (VNOM) for the computation of total apparent power in Phase A. It is
 * essential to ensure that the `dev` parameter is properly initialized
 * and not null before invoking this function. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function modifies the device's
 * configuration register to enable the VNOM feature.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_vnoma_enable(struct ade9153a_dev *dev);

// Use x_WAV waveforms after the high-pass filter and phase
//compensation for the RMS_OC calculation.
/***************************************************************************//**
 * @brief This function should be called when you want to configure the ADE9153A
 * device to use x_WAV waveforms after the high-pass filter and phase
 * compensation for the RMS_OC calculation. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. If the `dev` parameter is null, the function will return an
 * error code indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_rms_oc_src_x_wav_enable(struct ade9153a_dev *dev);

// Use ADC samples, before the high-pass filter for
// the RMS_OC calculation.
/***************************************************************************//**
 * @brief This function should be called when you want to enable the use of ADC
 * samples for the RMS overcurrent source calculation in the ADE9153A
 * device. It is important to ensure that the `dev` parameter is properly
 * initialized and not null before calling this function. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. The function modifies the device's
 * configuration register to enable the ADC samples for the specified
 * calculation.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_rms_oc_src_adc_samples_enable(struct ade9153a_dev *dev);

// ZX data source after hpf and phase compensation
/***************************************************************************//**
 * @brief This function is used to disable the zero crossing data source after
 * the high-pass filter and phase compensation for the RMS_OC
 * calculation. It should be called when the application no longer
 * requires the zero crossing data from the specified source. The
 * function expects a valid device structure pointer, which must be
 * initialized prior to calling this function. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null and should point to an initialized
 * device structure. If the pointer is null, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_zx_src_after_hpf_enable(struct ade9153a_dev *dev);

// ZX data source before hpf and phase compensation
/***************************************************************************//**
 * @brief This function should be called to configure the ADE9153A device to use
 * the zero crossing (ZX) data source prior to the high-pass filter for
 * calculations. It is essential to ensure that the `dev` parameter is a
 * valid pointer to an initialized `ade9153a_dev` structure. If `dev` is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where the user wants to process raw ADC samples before any filtering
 * is applied.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_zx_src_before_hpf_enable(struct ade9153a_dev *dev);

// Current channel B integrator enable
/***************************************************************************//**
 * @brief This function should be called to enable the integrator for current
 * channel B in the ADE9153A device. It is important to ensure that the
 * device has been properly initialized before invoking this function. If
 * the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. This function modifies
 * the device's configuration register to enable the integrator, which is
 * essential for accurate current measurements.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code. The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as if the device is not available.
 ******************************************************************************/
int ade9153a_i_ch_b_int_enable(struct ade9153a_dev *dev);

// Disable hpf for all channels
/***************************************************************************//**
 * @brief This function is used to disable the high-pass filter in the ADE9153A
 * device, which may be necessary when certain measurements or
 * configurations require the filter to be turned off. It should be
 * called after the device has been properly initialized and configured.
 * If the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_hpf_disable(struct ade9153a_dev *dev);

// Set PGA gain for current channel B
/***************************************************************************//**
 * @brief This function is used to configure the programmable gain amplifier
 * (PGA) gain for current channel B of the ADE9153A device. It should be
 * called after the device has been initialized and is ready for
 * configuration. The function accepts a gain value that determines the
 * amplification level, which can be one of three predefined values: Gain
 * 1, Gain 2, or Gain 4. If an invalid gain value is provided, the
 * function defaults to setting the gain to Gain 1. It is important to
 * ensure that the `dev` parameter is not null before calling this
 * function, as passing a null pointer will result in an error.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @param gain An enumeration value of type `ade9153a_bi_gain_e` that specifies
 * the desired gain for channel B. Valid values are
 * `ADE9153A_PGA_CHB_GAIN_1`, `ADE9153A_PGA_CHB_GAIN_2`, and
 * `ADE9153A_PGA_CHB_GAIN_4`. If an invalid value is provided, the
 * function will default to `ADE9153A_PGA_CHB_GAIN_1`.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error, such as -ENODEV if the device pointer is
 * null.
 ******************************************************************************/
int ade9153a_bi_pgagain_set(struct ade9153a_dev *dev,
			    enum ade9153a_bi_gain_e gain);

// Auto calibration config
/***************************************************************************//**
 * @brief This function is used to configure the auto calibration settings for
 * the ADE9153A device. It must be called after the device has been
 * initialized and before starting the auto calibration process. The
 * function allows the user to select which channel to calibrate
 * (voltage, current channel A, or current channel B) and the mode of
 * calibration (normal or turbo). If an invalid channel is specified, the
 * function defaults to enabling auto calibration for the voltage
 * channel. It is important to ensure that the `dev` parameter is not
 * null before calling this function, as passing a null pointer will
 * result in an error.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param ch Specifies the channel for auto calibration. Valid values are
 * `AUTOCAL_AV`, `AUTOCAL_BI`, and `AUTOCAL_AI`. If an invalid value
 * is provided, the function defaults to calibrating the voltage
 * channel.
 * @param mode Specifies the calibration mode. Valid values are `NORMAL` and
 * `TURBO`. This parameter is only relevant for the current
 * channels.
 * @return Returns 0 on success, or a negative error code if an error occurs,
 * such as if the device pointer is null or if the update of register
 * bits fails.
 ******************************************************************************/
int ade9153a_auto_calibration_cfg(struct ade9153a_dev *dev,
				  enum ade9153a_acal_ch_e ch, enum ade9153a_acalmode_e mode);

// Auto calibration run
/***************************************************************************//**
 * @brief This function initiates the auto calibration process for the ADE9153A
 * device. It must be called after the device has been properly
 * initialized and configured. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. The function will also return an error code if the
 * calibration mode or run bits cannot be set in the device's
 * configuration register.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_auto_calibration_run(struct ade9153a_dev *dev);

// Auto calibration stop
/***************************************************************************//**
 * @brief This function is used to stop the ongoing auto calibration process for
 * the ADE9153A device. It should be called when the auto calibration is
 * no longer needed, such as after the calibration has been successfully
 * completed or if the user decides to abort the calibration process.
 * Before calling this function, ensure that the `dev` parameter is
 * properly initialized and points to a valid `ade9153a_dev` structure.
 * If the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_auto_calibration_stop(struct ade9153a_dev *dev);

// mSure status
/***************************************************************************//**
 * @brief This function is used to obtain the current mSure status from the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. The function checks for valid input parameters
 * and returns an error if the device or status pointer is null. Upon
 * successful execution, it updates the status pointer with the mSure
 * status, which indicates the readiness and configuration state of the
 * mSure feature.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the mSure status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_msure_status(struct ade9153a_dev *dev, uint8_t *status);

// Ipeak channel phase
/***************************************************************************//**
 * @brief This function is used to determine the phase of the Ipeak channel in
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. The function reads the relevant register and
 * updates the provided pointer with the phase value. The phase can be
 * 0x01, 0x02, or 0x00, indicating different states of the Ipeak channel.
 * If the device or the output pointer is null, the function will return
 * an error code. It is important to handle these error codes
 * appropriately.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function returns -ENODEV.
 * @param ch Pointer to a `uint8_t` where the phase value will be stored. Must
 * not be null; otherwise, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure.
 ******************************************************************************/
int ade9153a_ip_phase(struct ade9153a_dev *dev, uint8_t *ch);

// Ipeak val
/***************************************************************************//**
 * @brief This function retrieves the peak current value from the ADE9153A
 * device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` representing the device and a pointer to a `uint32_t`
 * variable where the retrieved value will be stored. If the device
 * pointer is null, the function will return an error code indicating
 * that the device is not available. Similarly, if the value pointer is
 * null, it will return an error code indicating invalid arguments. The
 * function will read the value from the appropriate register and store
 * it in the provided variable.
 *
 * @param dev Pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val Pointer to a `uint32_t` where the peak current value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_ipk_val(struct ade9153a_dev *dev, uint32_t *val);

// Vpeak val
/***************************************************************************//**
 * @brief This function retrieves the peak voltage value from the ADE9153A
 * device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` representing the device and a pointer to a `uint32_t`
 * variable where the retrieved value will be stored. If the device
 * pointer is null, the function will return an error code indicating
 * that the device is not available. Similarly, if the value pointer is
 * null, it will return an error code indicating invalid arguments. The
 * function will read the value from the appropriate register and store
 * it in the provided variable.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val A pointer to a `uint32_t` where the peak voltage value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_vpk_val(struct ade9153a_dev *dev, uint32_t *val);

// Get interrupt indicator from STATUS register.
/***************************************************************************//**
 * @brief This function retrieves the interrupt status from the ADE9153A device
 * based on a specified mask. It should be called after the device has
 * been initialized and configured. The function checks for valid input
 * parameters, ensuring that the device structure and status pointer are
 * not null. If the device is not initialized or the status pointer is
 * null, it will return an error code. The function reads the status
 * register and updates the provided status pointer with the relevant
 * bits indicated by the mask.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param msk A 32-bit mask used to specify which bits of the status register to
 * check. Valid values depend on the specific interrupt bits defined
 * in the device's documentation.
 * @param status Pointer to a `uint8_t` where the resulting status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_get_int_status(struct ade9153a_dev *dev, uint32_t msk,
			    uint8_t *status);

// Get PF ready indicator.
/***************************************************************************//**
 * @brief This function retrieves the power factor ready status from the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. The function checks if the provided device
 * structure and status pointer are valid. If either is null, it returns
 * an error code. The status will be updated with the current power
 * factor ready indicator, which can be used to determine if the device
 * is ready for power factor calculations.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the power factor ready status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the status pointer is null.
 ******************************************************************************/
int ade9153a_get_pf_rdy(struct ade9153a_dev *dev, uint8_t *status);

// Get CRC change indicator.
/***************************************************************************//**
 * @brief This function retrieves the CRC change status from the ADE9153A
 * device. It should be called after the device has been initialized and
 * is ready for operation. The function checks for valid input parameters
 * and returns an error if the device structure or status pointer is
 * null. The status pointer will be updated with the CRC change
 * indicator, which can be used to determine if a CRC change event has
 * occurred.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the CRC change status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_crc_chg(struct ade9153a_dev *dev, uint8_t *status);

// Get CRC done indicator.
/***************************************************************************//**
 * @brief This function is used to check if the CRC (Cyclic Redundancy Check)
 * operation has been completed for the ADE9153A device. It should be
 * called after initiating a CRC calculation to determine if the process
 * is finished. The function expects a valid device structure and a
 * pointer to a status variable where the result will be stored. If the
 * device structure is null, or if the status pointer is null, the
 * function will return an error code. It is important to ensure that the
 * device has been properly initialized before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null. If null, the function returns -ENODEV.
 * @param status Pointer to a `uint8_t` variable where the CRC done status will
 * be stored. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, indicating that the CRC operation is complete,
 * or a negative error code if an error occurred.
 ******************************************************************************/
int ade9153a_get_crc_done(struct ade9153a_dev *dev, uint8_t *status);

// Get zero crossing timout on V ch indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the status of the zero crossing
 * timeout indicator for the voltage channel. It should be called after
 * the device has been properly initialized. The function checks if the
 * device and status pointer are valid; if either is null, it returns an
 * error code. The status will be updated to reflect the current state of
 * the zero crossing timeout.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * found or if the status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_zxtoav(struct ade9153a_dev *dev, uint8_t *status);

// Get zero crossing detect on I ch B indicator.
/***************************************************************************//**
 * @brief This function retrieves the zero crossing detection status for the
 * current channel B of the ADE9153A device. It should be called after
 * the device has been properly initialized and configured. The function
 * expects a valid device structure and a pointer to a status variable
 * where the result will be stored. If the device pointer is null, or if
 * the status pointer is null, the function will return an error code. It
 * is important to ensure that the device is operational before calling
 * this function to avoid unexpected results.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the zero crossing status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * valid or if the status pointer is null.
 ******************************************************************************/
int ade9153a_get_zxbi(struct ade9153a_dev *dev, uint8_t *status);

// Get zero crossing detect on I ch A indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the zero crossing detection status for
 * the current channel A of the ADE9153A device. It should be called
 * after the device has been properly initialized. The function expects a
 * valid pointer to a `struct ade9153a_dev` representing the device and a
 * pointer to a `uint8_t` variable where the status will be stored. If
 * the device pointer is null, the function will return an error
 * indicating that the device is not available. Similarly, if the status
 * pointer is null, it will return an invalid argument error.
 *
 * @param dev Pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param status Pointer to a `uint8_t` variable where the zero crossing status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_get_zxai(struct ade9153a_dev *dev, uint8_t *status);

// Get zero crossing detect on V ch indicator.
ade9153a_get_zxav(struct ade9153a_dev *dev, uint8_t *status);

// Get reset done indicator.
/***************************************************************************//**
 * @brief This function is used to check if the reset operation of the ADE9153A
 * device has been completed. It should be called after a reset operation
 * to determine if the device is ready for further communication. The
 * function expects a valid device structure and a pointer to a status
 * variable where the result will be stored. If the device structure is
 * null, or if the status pointer is null, the function will return an
 * error code. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the reset done status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * valid or if the status pointer is null.
 ******************************************************************************/
int ade9153a_get_rstdone(struct ade9153a_dev *dev, uint8_t *status);

// Get fundamental reactive energy no load condition indicator.
/***************************************************************************//**
 * @brief This function is used to check the status of the fundamental reactive
 * energy no load condition in the ADE9153A device. It should be called
 * after the device has been properly initialized. If the device pointer
 * is null, the function will return an error indicating that the device
 * is not available. Similarly, if the status pointer is null, an invalid
 * argument error will be returned. The function will populate the status
 * variable with the current state of the fundamental reactive energy no
 * load condition.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_fvarnl(struct ade9153a_dev *dev, uint8_t *status);

// Get total apparent energy no load condition indicator.
/***************************************************************************//**
 * @brief This function retrieves the status of the total apparent energy no
 * load condition from the ADE9153A device. It should be called after the
 * device has been properly initialized and configured. The function will
 * return an error if the device pointer is null or if the status pointer
 * is null, ensuring that valid pointers are provided before attempting
 * to read the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device or
 * status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_vanl(struct ade9153a_dev *dev, uint8_t *status);

// Get total active energy no load condition indicator.
/***************************************************************************//**
 * @brief This function is used to check if the total active energy is in a no
 * load condition. It should be called after the device has been properly
 * initialized. If the device is not initialized or if the status pointer
 * is null, the function will return an error code. The function is
 * particularly useful for monitoring the operational state of the device
 * and ensuring that it is functioning correctly under expected load
 * conditions.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @param status Pointer to a `uint8_t` variable where the no load condition
 * status will be stored. Must not be null; otherwise, the
 * function will return -EINVAL.
 * @return Returns 0 on success, indicating that the status has been
 * successfully retrieved. On failure, it returns a negative error code.
 ******************************************************************************/
int ade9153a_get_wattnl(struct ade9153a_dev *dev, uint8_t *status);

// Get new temperature reading ready indicator.
/***************************************************************************//**
 * @brief This function checks if a new temperature reading is ready from the
 * ADE9153A device. It should be called after the device has been
 * initialized and configured properly. The function will return an error
 * if the device pointer is null or if the status pointer is null. It is
 * important to ensure that the device is operational before calling this
 * function to avoid unexpected results.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the temperature ready status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the input parameters are invalid.
 ******************************************************************************/
int ade9153a_get_temp_rdy(struct ade9153a_dev *dev, uint8_t *status);

// Get RMS_OC values update indicator.
/***************************************************************************//**
 * @brief This function is used to check if the RMS overcurrent (RMS_OC) values
 * are ready for processing. It should be called after the device has
 * been initialized and configured properly. The function will return an
 * error if the device pointer is null or if the status pointer is null,
 * ensuring that valid pointers are provided before attempting to
 * retrieve the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, indicating that the status has been
 * successfully retrieved. On failure, it returns a negative error code.
 ******************************************************************************/
int ade9153a_get_rms_oc_rdy(struct ade9153a_dev *dev, uint8_t *status);

// Get power values registers update indicator.
/***************************************************************************//**
 * @brief This function is used to check if the power values are ready for
 * reading. It should be called after the device has been initialized and
 * configured. The function will return an error if the device pointer is
 * null or if the status pointer is null. It is important to ensure that
 * the device is properly set up before invoking this function to avoid
 * unexpected behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the power ready status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the input parameters are invalid.
 ******************************************************************************/
int ade9153a_get_pwrrdy(struct ade9153a_dev *dev, uint8_t *status);

// Get new waveform samples ready indicator.
/***************************************************************************//**
 * @brief This function is used to check if new waveform samples are available
 * from the ADE9153A device. It should be called after the device has
 * been initialized and configured. The function will return an error if
 * the device pointer is null or if the status pointer is null. It is
 * important to ensure that the device is properly set up before calling
 * this function to avoid unexpected behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the data ready status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if the status pointer is null.
 ******************************************************************************/
int ade9153a_get_dready(struct ade9153a_dev *dev, uint8_t *status);

// Get power values egy ready indicator.
/***************************************************************************//**
 * @brief This function is used to check if the energy data is ready for
 * processing. It should be called after the device has been initialized
 * and configured properly. The function will return an error if the
 * device pointer is null or if the status pointer is null, ensuring that
 * valid pointers are provided before attempting to retrieve the status.
 * It is important to handle these potential errors to avoid undefined
 * behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the energy ready status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device or
 * status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_egyrdy(struct ade9153a_dev *dev, uint8_t *status);

// Get CF2 pulse issued indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the status of the CF2 pulse issued by
 * the ADE9153A device. It should be called after the device has been
 * properly initialized and configured. The function checks for the
 * presence of a valid device structure and a non-null pointer for the
 * status output. If either of these conditions is not met, it will
 * return an error code. The status output will indicate whether the CF2
 * pulse has been issued.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * valid or if the status pointer is null.
 ******************************************************************************/
int ade9153a_get_cf2(struct ade9153a_dev *dev, uint8_t *status);

// Get CF1 pulse issued indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the status of the CF1 pulse from the
 * ADE9153A device. It should be called after the device has been
 * properly initialized and configured. The function will return an error
 * if the device pointer is null or if the status pointer is null,
 * ensuring that valid pointers are provided before attempting to read
 * the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the CF1 pulse status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device or
 * status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_cf1(struct ade9153a_dev *dev, uint8_t *status);

// Get CF2 polarity change indicator.
/***************************************************************************//**
 * @brief This function is used to check if there has been a change in the
 * polarity of the CF2 output. It should be called after the device has
 * been initialized and configured properly. The function will return an
 * error if the device pointer is null or if the status pointer is null,
 * ensuring that valid pointers are provided before attempting to read
 * the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device or
 * status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_cf2_chg(struct ade9153a_dev *dev, uint8_t *status);

// Get CF1 polarity change indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the status of the CF1 polarity change
 * indicator from the ADE9153A device. It should be called after the
 * device has been properly initialized and configured. The function will
 * return an error if the device pointer is null or if the status pointer
 * is null, ensuring that valid pointers are provided before attempting
 * to read the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * found or if the status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_cf1_chg(struct ade9153a_dev *dev, uint8_t *status);

// Get RPA (reactive power) sign change indicator.
/***************************************************************************//**
 * @brief This function is used to check if there has been a sign change in the
 * reactive power (RPA) measurement. It should be called after the device
 * has been properly initialized and configured. The function will return
 * an error if the device pointer is null or if the status pointer is
 * null, ensuring that valid pointers are provided before attempting to
 * retrieve the status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the sign change status
 * will be stored. Must not be null.
 * @return Returns 0 on success, indicating that the sign change status has been
 * successfully retrieved. Returns a negative error code if the device
 * or status pointer is invalid.
 ******************************************************************************/
int ade9153a_get_rpa_chg_sgn(struct ade9153a_dev *dev, uint8_t *status);

// Get APA (active power) sign change indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the sign change status of active power
 * from the ADE9153A device. It should be called after the device has
 * been properly initialized and configured. The function checks for
 * valid input parameters and returns an error if the device structure or
 * status pointer is null. The status pointer will be updated with the
 * sign change status, which can be used to determine if the active power
 * has changed its sign.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the sign change status
 * will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_get_apa_chg_sgn(struct ade9153a_dev *dev, uint8_t *status);

// Clear PF ready int mask.
/***************************************************************************//**
 * @brief This function is used to clear the power factor ready interrupt mask
 * in the ADE9153A device. It should be called when the application has
 * processed the power factor ready event and is ready to receive new
 * events. The function expects a valid device structure pointer, and if
 * the pointer is null, it will return an error code. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_pf_rdy(struct ade9153a_dev *dev);

// Clear CRC change int mask.
/***************************************************************************//**
 * @brief This function is used to clear the CRC change interrupt mask for the
 * ADE9153A device. It should be called when the application has handled
 * the CRC change event and wants to reset the interrupt status. The
 * function expects a valid device structure pointer, which must be
 * initialized prior to calling this function. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null and should point to an initialized
 * device structure. If the pointer is null, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_crc_chg(struct ade9153a_dev *dev);

// Clear CRC done int mask.
/***************************************************************************//**
 * @brief This function is used to clear the CRC done interrupt mask for the
 * ADE9153A device. It should be called when the application has
 * processed the CRC done event and wants to reset the interrupt status.
 * The function expects a valid device structure pointer, and if the
 * pointer is null, it will return an error code. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV). The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_crc_done(struct ade9153a_dev *dev);

// Clear zero crossing timout on V ch int mask.
/***************************************************************************//**
 * @brief This function should be called to clear the zero crossing timeout
 * interrupt mask for the voltage channel. It is typically used in
 * interrupt service routines or after checking the status of the zero
 * crossing timeout to ensure that the interrupt does not trigger again
 * until the condition is met again. The function expects a valid device
 * structure pointer, and if the pointer is null, it will return an error
 * code.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_zxtoav(struct ade9153a_dev *dev);

// Clear zero crossing detect on I ch B int mask.
/***************************************************************************//**
 * @brief This function should be called to clear the zero crossing detect
 * interrupt mask for the current channel B. It is typically used in
 * interrupt handling routines to acknowledge that the interrupt has been
 * processed. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9153a_dev` structure. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_zxbi(struct ade9153a_dev *dev);

// Clear zero crossing detect on I ch A int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the zero
 * crossing detection on the current channel A of the ADE9153A device. It
 * should be called when the application has handled the zero crossing
 * event and wants to reset the interrupt status. The function expects a
 * valid device structure pointer, and if the pointer is null, it will
 * return an error code indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code (-ENODEV). The caller retains ownership of the
 * device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_clear_zxai(struct ade9153a_dev *dev);

// Clear zero crossing detect on V ch int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the zero
 * crossing detection on the voltage channel of the ADE9153A device. It
 * should be called when the application has processed the zero crossing
 * event and is ready to re-enable detection. The function expects a
 * valid device structure pointer, which must be initialized and not
 * null. If the provided device pointer is null, the function will return
 * an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null and should point to an initialized
 * device structure. If the pointer is null, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_zxav(struct ade9153a_dev *dev);

// Clear reset done int mask.
/***************************************************************************//**
 * @brief This function is used to clear the reset done interrupt mask for the
 * ADE9153A device. It should be called when the reset done condition has
 * been handled, allowing the device to signal new reset events. The
 * function must be called with a valid device structure that has been
 * properly initialized. If the device pointer is null, the function will
 * return an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_rstdone(struct ade9153a_dev *dev);

// Clear fundamental reactive energy no load condition int mask.
/***************************************************************************//**
 * @brief This function should be called to clear the interrupt mask for the
 * fundamental reactive energy no load condition. It is typically used in
 * scenarios where the application needs to reset the interrupt status
 * after handling an event. The function must be called with a valid
 * device structure that has been properly initialized; passing a null
 * pointer will result in an error. It is important to ensure that the
 * device is in a state where it can accept this operation.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; passing a null pointer will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_clear_fvarnl(struct ade9153a_dev *dev);

// Clear total apparent energy no load condition int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the total
 * apparent energy no load condition. It should be called when the
 * application has handled the corresponding interrupt and is ready to
 * reset the condition. The function expects a valid pointer to an
 * `ade9153a_dev` structure, which represents the device context. If the
 * provided pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device
 * context. This pointer must not be null; otherwise, the function
 * will return an error code (-ENODEV). The caller retains ownership
 * of the device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_clear_vanl(struct ade9153a_dev *dev);

// Clear total active energy no load condition int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the total active
 * energy no load condition. It should be called when the application has
 * handled the corresponding interrupt condition and is ready to reset
 * the mask. The function expects a valid pointer to an `ade9153a_dev`
 * structure, which represents the device context. If the provided
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device
 * context. Must not be null; otherwise, the function will return
 * -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_wattnl(struct ade9153a_dev *dev);

// Clear new temperature reading ready int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the temperature
 * ready status in the ADE9153A device. It should be called when the
 * temperature reading is no longer needed or after processing the
 * temperature data. The function expects a valid device structure
 * pointer, and if the pointer is null, it will return an error code.
 * Ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV).
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_temp_rdy(struct ade9153a_dev *dev);

// Clear RMS_OC values update int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the RMS_OC
 * values update, which is typically called when the application has
 * processed the corresponding interrupt. It is important to ensure that
 * this function is called after the interrupt has been handled to
 * prevent missing subsequent interrupts. The function should not be
 * called with a null device pointer.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code indicating that the device is not available.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_rms_oc_rdy(struct ade9153a_dev *dev);

// Clear power values registers update int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for power values
 * registers update, which is typically set when new power data is
 * available. It should be called when the application has processed the
 * power data and is ready to receive new updates. The function expects a
 * valid device structure pointer, and if the pointer is null, it will
 * return an error code. It is important to ensure that the device has
 * been properly initialized before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV). The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_pwrrdy(struct ade9153a_dev *dev);

// Clear new waveform samples ready int mask.
/***************************************************************************//**
 * @brief This function is used to clear the data ready interrupt mask for the
 * ADE9153A device. It should be called when the application has
 * processed the data that triggered the interrupt, ensuring that the
 * device can signal new data availability in the future. The function
 * expects a valid device structure pointer, and it will return an error
 * code if the pointer is null, indicating that the device is not
 * available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_dready(struct ade9153a_dev *dev);

// Clear power values egy ready int mask.
/***************************************************************************//**
 * @brief This function should be called to clear the energy ready interrupt
 * mask in the ADE9153A device. It is typically used after handling an
 * interrupt to ensure that the device can signal new energy data
 * readiness in the future. The function must be called with a valid
 * device structure that has been properly initialized. If the provided
 * device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_egyrdy(struct ade9153a_dev *dev);

// Clear CF2 pulse issued int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the CF2 pulse
 * issued indicator. It should be called when the application has
 * processed the CF2 pulse event and is ready to receive new events. The
 * function expects a valid device structure pointer, and if the pointer
 * is null, it will return an error code. It is important to ensure that
 * the device is properly initialized before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_cf2(struct ade9153a_dev *dev);

// Clear CF1 pulse issued int mask.
ade9153a_clear_cf1(struct ade9153a_dev *dev);

// Clear CF2 polarity change int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the CF2 polarity
 * change. It should be called when the application has handled the CF2
 * polarity change event and wants to reset the interrupt status. The
 * function expects a valid device structure pointer, which must be
 * initialized prior to calling this function. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null and should point to an initialized
 * device structure. If the pointer is null, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_cf2_chg(struct ade9153a_dev *dev);

// Clear CF1 polarity change int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the CF1 polarity
 * change. It should be called when the application has processed the CF1
 * polarity change event and is ready to acknowledge it. The function
 * expects a valid device structure pointer, which must be initialized
 * prior to calling this function. If the provided device pointer is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_cf1_chg(struct ade9153a_dev *dev);

// Clear RPA (reactive power) sign change int mask.
/***************************************************************************//**
 * @brief This function should be called to clear the interrupt mask for the RPA
 * sign change, which is useful when the application needs to reset the
 * interrupt status after handling it. It is important to ensure that the
 * `dev` parameter is properly initialized and not null before calling
 * this function, as passing a null pointer will result in an error. The
 * function does not modify any other state or data.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_clear_rpa_chg_sgn(struct ade9153a_dev *dev);

// Clear APA (active power) sign change int mask.
/***************************************************************************//**
 * @brief This function is used to clear the interrupt mask for the active power
 * sign change in the ADE9153A device. It should be called when the
 * application has processed the active power sign change event and is
 * ready to clear the corresponding interrupt. The function expects a
 * valid device structure pointer, and if the pointer is null, it will
 * return an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_clear_apa_chg_sgn(struct ade9153a_dev *dev);

// Get chip status indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the current status of the ADE9153A
 * chip. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. The status
 * is written to the memory location pointed to by the `status`
 * parameter, which must not be null.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error.
 * @param status A pointer to a `uint8_t` variable where the chip status will be
 * stored. This pointer must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_get_chip_stat(struct ade9153a_dev *dev, uint8_t *status);

// Clear chip status int.
/***************************************************************************//**
 * @brief This function is used to clear the chip status register of the
 * ADE9153A device. It should be called when the device is properly
 * initialized and ready for operation. If the `dev` parameter is null,
 * the function will return an error code indicating that the device is
 * not available. The `reg_val` parameter is expected to point to a valid
 * memory location where the current status value will be stored. It is
 * important to ensure that the pointer provided is not null to avoid
 * undefined behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param reg_val Pointer to a `uint32_t` where the current chip status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_clear_chip_stat(struct ade9153a_dev *dev, uint32_t *reg_val);

// Get event status indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the current event status of the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. The function checks if the device pointer is
 * valid; if it is not, it returns an error code. The event status is
 * written to the provided status pointer, which must not be null. This
 * function is useful for monitoring specific events that the device may
 * report.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the event status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is invalid.
 ******************************************************************************/
int ade9153a_get_event_stat(struct ade9153a_dev *dev, uint8_t *status);

// Clear event status int.
/***************************************************************************//**
 * @brief This function is used to clear the event status register of the
 * ADE9153A device. It should be called when the user wants to reset the
 * event status, typically after handling an event. The function requires
 * that the `dev` parameter is a valid pointer to an initialized
 * `ade9153a_dev` structure. If `dev` is null, the function will return
 * an error code indicating that the device is not available. The
 * `reg_val` parameter is expected to point to a valid memory location
 * where the register value will be stored upon successful execution.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @param reg_val A pointer to a `uint32_t` variable where the value of the
 * event status register will be stored. Caller retains ownership
 * and must ensure that this pointer is valid.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_clear_event_stat(struct ade9153a_dev *dev, uint32_t *reg_val);

// Get MS status indicator.
/***************************************************************************//**
 * @brief This function is used to obtain the MS (Measurement System) status
 * from the ADE9153A device. It should be called after the device has
 * been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. The `status` parameter must point to a valid memory
 * location where the function will store the retrieved status. It is
 * important to ensure that the memory allocated for `status` is
 * sufficient to hold the expected data.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error.
 * @param status A pointer to a `uint8_t` variable where the MS status will be
 * stored. Caller must ensure that this pointer is valid and
 * points to allocated memory.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if an error occurs during the operation.
 ******************************************************************************/
int ade9153a_get_ms_stat(struct ade9153a_dev *dev, uint8_t *status);

// Clear MS status int.
/***************************************************************************//**
 * @brief This function is used to clear the MS status interrupt for the
 * ADE9153A device. It should be called when the MS status interrupt has
 * been triggered and the user wants to acknowledge or reset the
 * interrupt condition. The function requires that the `dev` parameter is
 * a valid pointer to an initialized `ade9153a_dev` structure. If the
 * device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an initialized `ade9153a_dev` structure representing
 * the device. Must not be null; otherwise, the function will return
 * an error code.
 * @param reg_val A pointer to a `uint32_t` variable where the register value
 * will be stored. The caller retains ownership of this variable.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_clear_ms_stat(struct ade9153a_dev *dev, uint32_t *reg_val);

// Enable/disable interrupt.
/***************************************************************************//**
 * @brief This function is used to control the enabling or disabling of
 * interrupts for the ADE9153A device. It should be called after the
 * device has been initialized and configured. The `reg_addr` parameter
 * specifies the register address where the interrupt mask will be
 * applied, while `int_msk` defines which interrupts to enable or
 * disable. The `en` parameter determines whether to enable (non-zero
 * value) or disable (zero value) the specified interrupts. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param reg_addr The address of the register where the interrupt mask will be
 * applied. This should be within the valid range of register
 * addresses defined for the ADE9153A.
 * @param int_msk A bitmask representing the interrupts to be enabled or
 * disabled. The specific bits correspond to different interrupt
 * sources as defined in the device's documentation.
 * @param en A flag indicating whether to enable (non-zero) or disable (zero)
 * the specified interrupts.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as if the device is not available.
 ******************************************************************************/
int ade9153a_control_interrupt(struct ade9153a_dev *dev, uint16_t reg_addr,
			       uint32_t int_msk, uint8_t en);

// Enable an interrupt when any bit in CHIP_STATUS reg is set
/***************************************************************************//**
 * @brief This function is used to enable an interrupt that triggers when any
 * bit in the CHIP_STATUS register is set. It should be called after the
 * device has been properly initialized and configured. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_enable_chip_stat_int(struct ade9153a_dev *dev);

// Disable CHIP_STAT interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt associated with the
 * CHIP_STAT register. It should be called when the application no longer
 * needs to respond to events indicated by the CHIP_STAT interrupt.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9153a_dev` structure. If the `dev` parameter is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_chip_stat_int(struct ade9153a_dev *dev);

// Enable an interrupt when any bit in EVENT_STATUS reg is set
/***************************************************************************//**
 * @brief This function is used to enable interrupts for event status changes in
 * the ADE9153A device. It should be called after the device has been
 * properly initialized and configured. If the provided device pointer is
 * null, the function will return an error code indicating that the
 * device is not available. Enabling this interrupt allows the system to
 * respond to specific events as they occur, which is crucial for real-
 * time monitoring and control.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_event_stat_int(struct ade9153a_dev *dev);

// Disable EVENT_STAT interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the event status interrupt for the
 * ADE9153A device. It should be called when the event status interrupt
 * is no longer needed, such as during device shutdown or when
 * reconfiguring interrupts. Before calling this function, ensure that
 * the `dev` parameter points to a valid `ade9153a_dev` structure. If
 * `dev` is null, the function will return an error code indicating that
 * the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_event_stat_int(struct ade9153a_dev *dev);

// Enable an interrupt when any bit in MS_STATUS_IRQ reg is set
/***************************************************************************//**
 * @brief This function is used to enable the measurement status interrupt for
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * This function is typically used in scenarios where the application
 * needs to respond to measurement status changes.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_ms_stat_int(struct ade9153a_dev *dev);

// Disable MS_STAT interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the MS_STAT interrupt for the
 * ADE9153A device. It should be called when the interrupt is no longer
 * needed, such as during device shutdown or when reconfiguring
 * interrupts. Before calling this function, ensure that the `dev`
 * parameter points to a valid `ade9153a_dev` structure. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_ms_stat_int(struct ade9153a_dev *dev);

// Enable pf ready interrupt
/***************************************************************************//**
 * @brief This function is used to enable the power factor ready interrupt for
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. If the provided device pointer is null, the
 * function will return an error code indicating that the device is not
 * available. Enabling this interrupt allows the system to respond to
 * power factor readiness events, which can be critical for applications
 * that require timely processing of power quality data.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_enable_pf_rdy_int(struct ade9153a_dev *dev);

// Disable pf ready interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the power ready interrupt for the
 * ADE9153A device. It should be called when the application no longer
 * needs to respond to power ready events, typically after the device has
 * been initialized and configured. The function checks if the provided
 * device pointer is valid; if it is null, it returns an error code. It
 * is important to ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV). The caller retains ownership of the device
 * structure.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_disable_pf_rdy_int(struct ade9153a_dev *dev);

// Enable crc change interrupt
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for CRC change events in
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * Enabling this interrupt allows the system to respond to changes in the
 * CRC status, which can be critical for maintaining data integrity.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_crc_chg_int(struct ade9153a_dev *dev);

// Disable crc change interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the CRC change interrupt for the
 * ADE9153A device. It should be called when the user no longer wishes to
 * receive notifications about changes in the CRC status. Before calling
 * this function, ensure that the `dev` parameter points to a valid
 * `ade9153a_dev` structure that has been properly initialized. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code (-ENODEV). The caller retains ownership of
 * the `dev` pointer.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_crc_chg_int(struct ade9153a_dev *dev);

// Enable crc done interrupt
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the CRC done event
 * in the ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * It is important to ensure that the device is ready and configured to
 * handle interrupts before calling this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_enable_crc_done_int(struct ade9153a_dev *dev);

// Disable crc done interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the CRC done interrupt for the
 * ADE9153A device. It should be called when the user no longer wishes to
 * receive notifications for CRC completion events. Before calling this
 * function, ensure that the `dev` parameter points to a valid
 * `ade9153a_dev` structure. If `dev` is null, the function will return
 * an error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_crc_done_int(struct ade9153a_dev *dev);

// Enable zero corssing timeout on voltage ch interrupt
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt that triggers
 * when a zero crossing timeout occurs on the voltage channel. It is
 * essential to ensure that the `dev` parameter is properly initialized
 * and not null before invoking this function. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where monitoring zero crossing events is critical for the application.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_zxtoav_int(struct ade9153a_dev *dev);

// Disable zero corssing timeout on voltage ch interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the zero crossing timeout interrupt
 * for the voltage channel in the ADE9153A device. It should be called
 * when the application no longer needs to monitor the zero crossing
 * timeout condition, typically after the relevant processing is
 * complete. The function expects a valid pointer to an `ade9153a_dev`
 * structure, which represents the device context. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; if null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_zxtoav_int(struct ade9153a_dev *dev);

// Enable zero corssing detected on I ch B interrupt
/***************************************************************************//**
 * @brief This function should be called to enable the zero crossing interrupt
 * for the current channel B of the ADE9153A device. It is important to
 * ensure that the device has been properly initialized before invoking
 * this function. If the `dev` parameter is null, the function will
 * return an error code indicating that the device is not available. This
 * function modifies the interrupt settings of the device, allowing it to
 * respond to zero crossing events on the specified channel.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_enable_zxbi_int(struct ade9153a_dev *dev);

// Disable zero corssing detected on I ch B interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the zero crossing interrupt for the
 * current channel B of the ADE9153A device. It should be called when the
 * application no longer needs to respond to zero crossing events for
 * this channel, typically during device shutdown or reconfiguration.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9153a_dev` structure that has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_zxbi_int(struct ade9153a_dev *dev);

// Enable zero corssing detected on I ch A interrupt
/***************************************************************************//**
 * @brief This function should be called to enable the zero crossing interrupt
 * for current channel A in the ADE9153A device. It is important to
 * ensure that the `dev` parameter is properly initialized and points to
 * a valid device structure before calling this function. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. This function is typically used in
 * scenarios where the application needs to respond to zero crossing
 * events on the current channel A.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_enable_zxai_int(struct ade9153a_dev *dev);

// Disable zero corssing detected on I ch A interrupt.
/***************************************************************************//**
 * @brief This function should be called when the zero crossing interrupt for
 * current channel A is no longer needed. It is important to ensure that
 * the `dev` parameter is properly initialized and not null before
 * calling this function. If `dev` is null, the function will return an
 * error code indicating that the device is not available. This function
 * is typically used in scenarios where the application no longer
 * requires monitoring of zero crossing events for current channel A.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_zxai_int(struct ade9153a_dev *dev);

// Enable zero corssing detected on V ch interrupt
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt for zero
 * crossing detection on the voltage channel of the ADE9153A device. It
 * is typically used in scenarios where the application needs to respond
 * to zero crossing events for voltage measurements. Before calling this
 * function, ensure that the `dev` parameter points to a valid and
 * initialized `ade9153a_dev` structure. If the `dev` parameter is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_zxav_int(struct ade9153a_dev *dev);

// Disable zero corssing detected on V ch interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the zero crossing interrupt for the
 * voltage channel in the ADE9153A device. It should be called when the
 * application no longer needs to respond to zero crossing events for the
 * voltage channel, typically during device shutdown or reconfiguration.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9153a_dev` structure that has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_zxav_int(struct ade9153a_dev *dev);

// Enable fundamental reactive energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt for the
 * fundamental reactive energy no load condition, which allows the system
 * to respond to changes in the energy state. It is important to ensure
 * that the device has been properly initialized before calling this
 * function. If the `dev` parameter is null, the function will return an
 * error code indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_fvarnl_int(struct ade9153a_dev *dev);

// Disable fundamental reactive energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that signals when the
 * fundamental reactive energy is in a no load condition. It should be
 * called when the application no longer needs to monitor this condition,
 * typically after the relevant processing has been completed. The
 * function expects a valid pointer to an `ade9153a_dev` structure, which
 * represents the device context. If the provided device pointer is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code (-ENODEV). The caller retains ownership of the
 * device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_fvarnl_int(struct ade9153a_dev *dev);

// Enable total apparent energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the total apparent
 * energy no load condition in the ADE9153A device. It should be called
 * after the device has been properly initialized. If the `dev` parameter
 * is null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where the application needs to respond to changes in the total
 * apparent energy state.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_enable_vanl_int(struct ade9153a_dev *dev);

// Disable total apparent energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that signals when the
 * total apparent energy is in a no load condition. It should be called
 * when the application no longer needs to respond to this specific
 * interrupt, typically after the relevant processing has been completed.
 * The function requires a valid device structure pointer, which must be
 * initialized prior to calling this function. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. This
 * pointer must not be null and should point to an initialized device
 * structure. If it is null, the function will return an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_disable_vanl_int(struct ade9153a_dev *dev);

// Enable total active energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt for the total
 * active energy no load condition, which allows the system to respond to
 * changes in the energy state. It is important to ensure that the `dev`
 * parameter is properly initialized and points to a valid `ade9153a_dev`
 * structure before calling this function. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where monitoring of energy conditions is critical, and it should be
 * called after the device has been initialized.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_enable_wattnl_int(struct ade9153a_dev *dev);

// Disable total active energy no load condition interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that signals when the
 * total active energy is in a no load condition. It should be called
 * when the application no longer needs to respond to this specific
 * interrupt, typically after it has been enabled previously. The
 * function expects a valid pointer to an `ade9153a_dev` structure, which
 * represents the device context. If the provided pointer is null, the
 * function will return an error code indicating that the device is not
 * available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device
 * context. This pointer must not be null; otherwise, the function
 * will return an error code (-ENODEV). The caller retains ownership
 * of the device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_wattnl_int(struct ade9153a_dev *dev);

// Enable new temperature reading interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for temperature ready
 * events in the ADE9153A device. It should be called after the device
 * has been properly initialized. If the provided device pointer is null,
 * the function will return an error code indicating that the device is
 * not available. This function is essential for applications that need
 * to respond to temperature changes detected by the device.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_enable_temp_rdy_int(struct ade9153a_dev *dev);

// Disable new temperature reading interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that signals when a new
 * temperature reading is available from the ADE9153A device. It should
 * be called when the application no longer needs to be notified of
 * temperature updates, such as during device shutdown or when the
 * temperature monitoring is not required. Before calling this function,
 * ensure that the `dev` parameter points to a valid `ade9153a_dev`
 * structure. If the `dev` parameter is null, the function will return an
 * error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_temp_rdy_int(struct ade9153a_dev *dev);

// Enable RMS_OC values update interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for the RMS overcurrent
 * ready condition in the ADE9153A device. It should be called after the
 * device has been properly initialized. If the `dev` parameter is null,
 * the function will return an error code indicating that the device is
 * not available. This function is essential for applications that need
 * to respond to overcurrent conditions detected by the device.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_rms_oc_rdy_int(struct ade9153a_dev *dev);

// Disable RMS_OC values update interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the RMS overcurrent ready interrupt
 * for the ADE9153A device. It should be called when the interrupt is no
 * longer needed, such as during device shutdown or when reconfiguring
 * the interrupt settings. Before calling this function, ensure that the
 * `dev` parameter points to a valid `ade9153a_dev` structure. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_rms_oc_rdy_int(struct ade9153a_dev *dev);

// Enable PWRRDY interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the power ready interrupt for the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the provided device pointer is null, the
 * function will return an error code indicating that the device is not
 * available. This function is essential for applications that need to
 * respond to power ready events, allowing the system to take appropriate
 * actions when power is ready.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_enable_pwrrdy_int(struct ade9153a_dev *dev);

// Disable PWRRDY interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the power ready interrupt for the
 * ADE9153A device. It should be called when the application no longer
 * needs to respond to power ready events, typically during device
 * shutdown or when reconfiguring the interrupt settings. Before calling
 * this function, ensure that the `dev` parameter points to a valid
 * `ade9153a_dev` structure, as passing a null pointer will result in an
 * error. The function will return an error code if the device structure
 * is invalid.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; passing a null pointer will result in an error
 * code of -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_pwrrdy_int(struct ade9153a_dev *dev);

// Enable data ready interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the data ready interrupt for the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * Enabling this interrupt allows the system to be notified when new data
 * is available from the device, which is essential for applications that
 * require timely data processing.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_dready_int(struct ade9153a_dev *dev);

// Disable data ready interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the data ready interrupt for the
 * ADE9153A device. It should be called when the application no longer
 * needs to receive notifications about data availability, typically
 * during device shutdown or when switching to a different mode of
 * operation. Before calling this function, ensure that the `dev`
 * parameter is properly initialized and points to a valid `ade9153a_dev`
 * structure. If the `dev` parameter is null, the function will return an
 * error code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_dready_int(struct ade9153a_dev *dev);

// Enable EGYRDY interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the energy ready interrupt for the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the provided device pointer is null, the
 * function will return an error code indicating that the device is not
 * available. This function is essential for applications that need to
 * respond to energy data being ready for processing.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_enable_egyrdy_int(struct ade9153a_dev *dev);

// Disable EGYRDY interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the EGYRDY interrupt for the ADE9153A
 * device. It should be called when the interrupt is no longer needed,
 * such as during device shutdown or when changing interrupt
 * configurations. The function requires a valid pointer to an
 * `ade9153a_dev` structure, which represents the device instance. If the
 * provided device pointer is null, the function will return an error
 * code indicating that the device is not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device
 * instance. This pointer must not be null; otherwise, the function
 * will return an error code (-ENODEV). The caller retains ownership
 * of the device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_egyrdy_int(struct ade9153a_dev *dev);

// Enable CF2 interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the CF2 interrupt for the ADE9153A
 * device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. Enabling
 * this interrupt allows the device to signal when specific events
 * related to CF2 occur, which can be useful for monitoring and
 * responding to changes in the device's operation.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_enable_cf2_int(struct ade9153a_dev *dev);

// Disable CF2 interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the CF2 interrupt for the ADE9153A
 * device. It should be called when the interrupt is no longer needed,
 * such as during device shutdown or when reconfiguring interrupts.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9153a_dev` structure. If `dev` is null, the function
 * will return an error code indicating that the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_cf2_int(struct ade9153a_dev *dev);

// Enable CF1 interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the CF1 interrupt for the ADE9153A
 * device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. Enabling
 * this interrupt allows the device to signal when specific events
 * related to CF1 occur, which can be useful for monitoring and
 * responding to changes in the device's operation.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_enable_cf1_int(struct ade9153a_dev *dev);

// Disable CF1 interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the CF1 interrupt for the ADE9153A
 * device. It should be called when the interrupt is no longer needed,
 * such as during device shutdown or when reconfiguring interrupts.
 * Before calling this function, ensure that the `dev` parameter points
 * to a valid `ade9153a_dev` structure. If `dev` is null, the function
 * will return an error code indicating that the device is not available.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_disable_cf1_int(struct ade9153a_dev *dev);

// Enable CF2 polarity change sign interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for changes in the
 * polarity of the CF2 signal. It should be called after the device has
 * been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. This function is typically used in scenarios where
 * monitoring the CF2 signal is critical for application functionality.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_revpcf2_int(struct ade9153a_dev *dev);

// Disable CF2 polarity change sign interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt for changes in the
 * polarity of the CF2 signal. It should be called when the application
 * no longer needs to respond to CF2 polarity changes, typically during
 * device shutdown or when reconfiguring interrupts. The function expects
 * a valid device structure pointer; if the pointer is null, it will
 * return an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_disable_revpcf2_int(struct ade9153a_dev *dev);

// Enable CF1 polarity change sign interrupt.
/***************************************************************************//**
 * @brief This function is used to enable the interrupt for changes in the
 * polarity of the CF1 signal. It should be called after the device has
 * been properly initialized and configured. If the `dev` parameter is
 * null, the function will return an error code indicating that the
 * device is not available. This function is typically used in scenarios
 * where monitoring the CF1 signal's polarity is critical for application
 * logic.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_enable_revpcf1_int(struct ade9153a_dev *dev);

// Disable CF1 polarity change sign interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that triggers when the
 * polarity of the CF1 signal changes. It should be called when the
 * application no longer needs to respond to CF1 polarity changes,
 * typically during device shutdown or when reconfiguring interrupt
 * settings. The function expects a valid device structure pointer, which
 * must be initialized prior to calling this function. If the provided
 * device pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null and should point to an initialized
 * device structure. If the pointer is null, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_disable_revpcf1_int(struct ade9153a_dev *dev);

// Enable Phase A fundamental reactive power changed sign interrupt.
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt for changes in
 * the sign of the fundamental reactive power for Phase A. It is
 * typically used in scenarios where monitoring of reactive power changes
 * is critical, such as in power quality applications. Before calling
 * this function, ensure that the device has been properly initialized
 * and is ready to handle interrupts. If the `dev` parameter is null, the
 * function will return an error code, indicating that the device is not
 * available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code (-ENODEV). The caller retains ownership of
 * the device structure.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_enable_revrpa_int(struct ade9153a_dev *dev);

// Disable Phase A fundamental reactive power changed sign interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt that triggers when the
 * sign of the Phase A fundamental reactive power changes. It should be
 * called when the application no longer needs to respond to this
 * specific interrupt condition. Before calling this function, ensure
 * that the device has been properly initialized and that the interrupt
 * is currently enabled. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_revrpa_int(struct ade9153a_dev *dev);

// Enable Phase A total active power changed sign interrupt.
/***************************************************************************//**
 * @brief This function should be called to enable the interrupt for changes in
 * the sign of the total active power for Phase A. It is typically used
 * in scenarios where monitoring of power changes is critical, such as in
 * energy management systems. Before calling this function, ensure that
 * the device has been properly initialized and is ready to handle
 * interrupts. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_enable_revapa_int(struct ade9153a_dev *dev);

// Disable Phase A total active power changed sign interrupt.
/***************************************************************************//**
 * @brief This function is used to disable the interrupt for the Phase A total
 * active power change sign. It should be called when the interrupt is no
 * longer needed, typically during device shutdown or reconfiguration.
 * The function expects a valid pointer to an `ade9153a_dev` structure,
 * which represents the device context. If the provided pointer is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code (-ENODEV). The caller retains ownership of the
 * device structure.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_disable_revapa_int(struct ade9153a_dev *dev);

// Overcurrent detection threshold level val
/***************************************************************************//**
 * @brief This function is used to read the overcurrent detection threshold
 * level from the ADE9153A device. It should be called after the device
 * has been properly initialized. The function expects a valid pointer to
 * a `struct ade9153a_dev` instance and a pointer to a `uint32_t`
 * variable where the retrieved value will be stored. If the device
 * pointer or the value pointer is null, the function will return an
 * error code. Additionally, if the read operation fails, the function
 * will return the corresponding error code.
 *
 * @param dev Pointer to the `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val Pointer to a `uint32_t` where the overcurrent level value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_oi_lvl_val(struct ade9153a_dev *dev, uint32_t *val);

// Ch A overcurrent val
/***************************************************************************//**
 * @brief This function retrieves the overcurrent value for channel A from the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` instance and a non-null pointer for the output value. If
 * the device pointer is null, it will return an error indicating that
 * the device is not available. Similarly, if the output pointer is null,
 * it will return an invalid argument error. The function will read the
 * corresponding register and apply a mask to the value before storing it
 * in the provided output pointer.
 *
 * @param dev Pointer to the `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val Pointer to a `uint32_t` where the overcurrent value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_oia_val(struct ade9153a_dev *dev, uint32_t *val);

// Ch B overcurrent val
/***************************************************************************//**
 * @brief This function retrieves the overcurrent value for channel B of the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` representing the device and a pointer to a `uint32_t`
 * variable where the read value will be stored. If the device pointer or
 * the value pointer is null, the function will return an error code. The
 * function will also return an error code if the read operation fails
 * for any reason.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val A pointer to a `uint32_t` where the overcurrent value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_oib_val(struct ade9153a_dev *dev, uint32_t *val);

// Vlevel val
/***************************************************************************//**
 * @brief This function retrieves the voltage level value from the ADE9153A
 * device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` instance and a pointer to a `uint32_t` variable where
 * the voltage level value will be stored. If the device pointer or the
 * value pointer is null, the function will return an error code. The
 * function will also return an error code if the read operation fails.
 * On success, the voltage level value is masked to ensure only the
 * relevant bits are returned.
 *
 * @param dev Pointer to the `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val Pointer to a `uint32_t` where the voltage level value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_vlevel_val(struct ade9153a_dev *dev, uint32_t *val);

// Voltage value during dip condition
/***************************************************************************//**
 * @brief This function retrieves the voltage value that occurs during a dip
 * condition from the ADE9153A device. It should be called after the
 * device has been properly initialized and configured. The caller must
 * ensure that the device is operational and that the pointer for the
 * output value is valid. If the device pointer or the output pointer is
 * null, the function will return an error code. The function may also
 * return an error if the read operation fails.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param val Pointer to a `uint32_t` where the read voltage value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_dipa_val(struct ade9153a_dev *dev, uint32_t *val);

// Voltage value during swell condition
/***************************************************************************//**
 * @brief This function retrieves the swell condition voltage value from the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` instance and a pointer to a `uint32_t` variable where
 * the result will be stored. If the device pointer or the value pointer
 * is null, the function will return an error code. The function will
 * read the corresponding register and apply a mask to extract the
 * relevant bits, storing the result in the provided variable.
 *
 * @param dev Pointer to the `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val Pointer to a `uint32_t` where the swell voltage value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_swella_val(struct ade9153a_dev *dev, uint32_t *val);

// Get phnoload status.
/***************************************************************************//**
 * @brief This function retrieves the no-load status of the ADE9153A device,
 * which indicates whether the phase A fundamental reactive energy, total
 * apparent energy, or total active energy is in a no-load condition. It
 * should be called after the device has been properly initialized. The
 * function will return an error if the device pointer or the status
 * pointer is null. The status value will be set to one of the predefined
 * constants indicating the type of no-load condition detected, or it
 * will be set to zero if no no-load condition is present.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the no-load status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_get_phnoload_status(struct ade9153a_dev *dev, uint8_t *status);

// CF2_LT status
/***************************************************************************//**
 * @brief This function is used to obtain the status of the CF2 low threshold
 * indicator from the ADE9153A device. It should be called after the
 * device has been properly initialized. The function checks for valid
 * input parameters and returns an error if the device structure or the
 * status pointer is null. If the function executes successfully, it
 * writes the status of the CF2 low threshold to the provided status
 * pointer.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the CF2 low threshold status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_cf2_lt_status(struct ade9153a_dev *dev, uint8_t *status);

// CF1_LT status
/***************************************************************************//**
 * @brief This function is used to obtain the status of the CF1 low threshold
 * indicator from the ADE9153A device. It should be called after the
 * device has been properly initialized and configured. The function
 * checks for valid input parameters and returns an error if the device
 * structure or the status pointer is null. Upon successful execution,
 * the status of the CF1 low threshold is written to the provided status
 * pointer.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the CF1 low threshold status will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_cf1_lt_status(struct ade9153a_dev *dev, uint8_t *status);

// cf_ltmr write value.
/***************************************************************************//**
 * @brief This function is used to write a specified value to the CF_LTMR
 * register of the ADE9153A device. It must be called after the device
 * has been properly initialized. If the `dev` parameter is null, the
 * function will return an error code indicating that the device is not
 * available. The `reg_data` parameter should contain the value to be
 * written, and it is expected to be within the valid range defined by
 * the register's specifications.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error.
 * @param reg_data The data to be written to the CF_LTMR register. It should be
 * a 32-bit unsigned integer that conforms to the register's
 * expected format.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_cf_ltmr_write(struct ade9153a_dev *dev, uint32_t reg_data);

// cf_ltmr read value.
/***************************************************************************//**
 * @brief This function is used to read the value of the CF_LTMR register from
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. The function expects a valid device structure
 * and a pointer to a variable where the read value will be stored. If
 * the device structure or the pointer to the value is null, the function
 * will return an error code. The function also handles the reading of
 * the register and masks the result to ensure only the relevant bits are
 * returned.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param val Pointer to a `uint32_t` variable where the register value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_cf_ltmr_val(struct ade9153a_dev *dev, uint32_t *val);

// temperature sensor offset.
/***************************************************************************//**
 * @brief This function is used to obtain the temperature offset value from the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` representing the device and a pointer to a `uint32_t`
 * variable where the retrieved offset value will be stored. If the
 * device pointer or the value pointer is null, the function will return
 * an error code. The function reads the temperature trim register and
 * extracts the offset value, which is then stored in the provided
 * variable.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val A pointer to a `uint32_t` where the temperature offset value will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_temp_offset_val(struct ade9153a_dev *dev, uint32_t *val);

// temperature sensor gain.
/***************************************************************************//**
 * @brief This function is used to obtain the temperature gain value from the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. The function expects a valid pointer to a `struct
 * ade9153a_dev` representing the device and a pointer to a `uint32_t`
 * variable where the retrieved gain value will be stored. If the device
 * pointer or the value pointer is null, the function will return an
 * error code. Additionally, if the read operation fails, the
 * corresponding error code will be returned.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null.
 * @param val A pointer to a `uint32_t` where the temperature gain value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_temp_gain_val(struct ade9153a_dev *dev, uint32_t *val);

// External voltage reference enable/disable.
/***************************************************************************//**
 * @brief This function is used to enable or disable the external voltage
 * reference for the ADE9153A device. It should be called after the
 * device has been initialized and configured. The `en` parameter
 * determines whether the external reference is enabled (1) or disabled
 * (0). If an invalid value is provided for `en`, the function will
 * handle it gracefully, ensuring that the device state remains
 * consistent.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param en An 8-bit unsigned integer that specifies whether to enable (1) or
 * disable (0) the external voltage reference. Valid values are 0 and
 * 1.
 * @return Returns an integer indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int ade9153a_ext_ref(struct ade9153a_dev *dev, uint8_t en);

// External_reference status
/***************************************************************************//**
 * @brief This function is used to check the status of the external reference
 * for the ADE9153A device. It should be called after the device has been
 * properly initialized. The function will return an error if the device
 * pointer or the status pointer is null. Upon successful execution, the
 * status will be updated to indicate whether the external reference is
 * enabled or disabled.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` variable where the external reference
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_ext_ref_status(struct ade9153a_dev *dev, uint8_t *status);

// Dip swell interrupt mode.
/***************************************************************************//**
 * @brief This function configures the dip swell interrupt mode for the ADE9153A
 * device. It should be called after the device has been initialized and
 * before starting any measurements. The function allows the user to
 * select between continuous interrupt mode or a one-time interrupt mode
 * when entering and exiting the condition. If the provided `dev` pointer
 * is null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param mode An enumeration value of type `dip_swell_irq_mode_en` that
 * specifies the desired interrupt mode. Valid values are
 * `CONTINUOUSE` for continuous interrupts and `ONE_INT` for a
 * single interrupt on entering and exiting the condition.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_dip_swell_irq_mode(struct ade9153a_dev *dev,
				enum dip_swell_irq_mode_en mode);

// Burst read enable/disable.
/***************************************************************************//**
 * @brief This function is used to toggle the burst mode feature of the ADE9153A
 * device. It should be called after the device has been properly
 * initialized. The function checks the current state of the burst mode
 * and updates the relevant configuration register accordingly. If the
 * `dev` parameter is null, the function will return an error code. It is
 * important to ensure that the device is in a valid state before calling
 * this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null. If null, the function returns -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_burst_en(struct ade9153a_dev *dev);

// Set PWR_SETTLE
/***************************************************************************//**
 * @brief This function configures the time for the power and filter-based RMS
 * measurements to settle before starting the power, energy, and CF
 * accumulations. It should be called after initializing the device and
 * before starting any measurements. The function accepts specific
 * predefined values for the settle time, and if an invalid value is
 * provided, it defaults to a 64 ms settle time. Ensure that the `dev`
 * parameter is a valid pointer to an initialized `ade9153a_dev`
 * structure.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param time An enumeration value of type `ade9153a_pwr_settle_e` that
 * specifies the desired settle time. Valid values include
 * `ADE9153A_SETTLE_0_MS`, `ADE9153A_SETTLE_64_MS`,
 * `ADE9153A_SETTLE_128_MS`, and `ADE9153A_SETTLE_256_MS`. If an
 * invalid value is provided, the function defaults to
 * `ADE9153A_SETTLE_64_MS`.
 * @return Returns an integer indicating the success or failure of the
 * operation. A negative value indicates an error, while a non-negative
 * value indicates success.
 ******************************************************************************/
int ade9153a_pwr_settle_set(struct ade9153a_dev *dev,
			    enum ade9153a_pwr_settle_e time);

// Clear accumulation in the digital freq conv.
/***************************************************************************//**
 * @brief This function should be called when there is a need to reset the
 * accumulated values in the digital frequency converter of the ADE9153A
 * device. It is important to ensure that the `dev` parameter is valid
 * and points to an initialized `ade9153a_dev` structure. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. This function does not have any side
 * effects beyond clearing the accumulation.
 *
 * @param dev Pointer to an `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_cf_acc_clr(struct ade9153a_dev *dev);

// ZX driven to CF2 pin.
/***************************************************************************//**
 * @brief This function is used to enable the zero crossing output for the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * This function modifies the configuration register to enable the zero
 * crossing output, which is essential for certain applications that
 * require synchronization with the zero crossing of the input signal.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_zx_out_oe(struct ade9153a_dev *dev);

// DREADY driven to CF2 pin.
/***************************************************************************//**
 * @brief This function is used to enable the data ready output signal for the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * This function modifies the configuration register to set the
 * appropriate bit for enabling the data ready output.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_dready_oe(struct ade9153a_dev *dev);

// CF2 output disable.
/***************************************************************************//**
 * @brief This function is used to disable the CF2 output of the ADE9153A
 * device. It should be called when the CF2 output is no longer needed,
 * such as during device reconfiguration or shutdown. Before calling this
 * function, ensure that the device has been properly initialized and is
 * in a valid operational state. If the provided `dev` pointer is null,
 * the function will return an error code indicating that the device is
 * not available.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_cf2dis(struct ade9153a_dev *dev);

// CF1 output disable.
/***************************************************************************//**
 * @brief This function should be called to disable the CF1 output of the
 * ADE9153A device. It is important to ensure that the device has been
 * properly initialized before invoking this function. If the `dev`
 * parameter is null, the function will return an error code indicating
 * that the device is not available. This function does not have any side
 * effects on the device state other than disabling the CF1 output.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_cf1dis(struct ade9153a_dev *dev);

// Set type of energy output on CF2 pin
/***************************************************************************//**
 * @brief This function configures the type of energy that is output on the CF2
 * pin of the ADE9153A device. It should be called after the device has
 * been initialized and is ready for configuration. The function accepts
 * an energy type parameter, which determines the specific energy output.
 * If the provided `dev` pointer is null, the function will return an
 * error code indicating that the device is not available. The function
 * also handles invalid energy type values by defaulting to total active
 * power.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param energy_type An enumeration value of type `ade9153a_cf2sel_e` that
 * specifies the type of energy to output. Valid values
 * include `ADE9153A_TOTAL_ACTIVE_POWER`,
 * `ADE9153A_TOTAL_APPARENT_POWER`, and
 * `ADE9153A_TOTAL_FUNDAMENTAL_REACTIVE_POWER`. If an invalid
 * value is provided, the function defaults to
 * `ADE9153A_TOTAL_ACTIVE_POWER`.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_cf2sel(struct ade9153a_dev *dev,
		    enum ade9153a_cf2sel_e energy_type);

// Set type of energy output on CF1 pin
/***************************************************************************//**
 * @brief This function configures the type of energy output that will be sent
 * to the CF1 pin of the ADE9153A device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * `energy_type` parameter determines which type of energy measurement
 * will be outputted, and it is important to ensure that the selected
 * type is supported by the device. If an invalid `energy_type` is
 * provided, the function defaults to setting the output to total active
 * power.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null, as it is required to access the
 * device's configuration.
 * @param energy_type An enumeration value of type `ade9153a_cf2sel_e` that
 * specifies the type of energy to be output on the CF1 pin.
 * Valid values include `ADE9153A_TOTAL_ACTIVE_POWER`,
 * `ADE9153A_TOTAL_APPARENT_POWER`, and
 * `ADE9153A_TOTAL_FUNDAMENTAL_REACTIVE_POWER`. The function
 * will handle invalid values by defaulting to
 * `ADE9153A_TOTAL_ACTIVE_POWER`.
 * @return Returns an integer value indicating the success or failure of the
 * operation. A return value of 0 indicates success, while a negative
 * value indicates an error, such as an invalid device pointer.
 ******************************************************************************/
int ade9153a_cf1sel(struct ade9153a_dev *dev,
		    enum ade9153a_cf2sel_e energy_type);

// Frequency select.
/***************************************************************************//**
 * @brief This function configures the self-frequency of the ADE9153A device,
 * which is essential for its operation. It should be called after the
 * device has been initialized and before starting any measurements. The
 * function accepts a frequency parameter that can be either 50 Hz or 60
 * Hz. If an invalid frequency is provided, the function defaults to
 * setting the frequency to 50 Hz. Additionally, if the device pointer is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param freq An enumeration value of type `ade9153a_selfreq_e` that specifies
 * the desired self-frequency. Valid values are `F_50_HZ` for 50 Hz
 * and `F_60_HZ` for 60 Hz. If an invalid value is provided, the
 * function will default to 50 Hz.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available or if an error occurs during the operation.
 ******************************************************************************/
int ade9153a_selfreq(struct ade9153a_dev *dev, enum ade9153a_selfreq_e freq);

// Frequency setting
/***************************************************************************//**
 * @brief This function is used to obtain the frequency status of the ADE9153A
 * device. It should be called after the device has been properly
 * initialized. The function checks for valid input parameters and
 * returns an error if the device or status pointer is null. If the
 * device is operational, it reads the relevant register and updates the
 * status pointer with the frequency status. The function may return an
 * error code if the read operation fails.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the frequency status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_freq_s_status(struct ade9153a_dev *dev, uint8_t *status);

// Select fundamental reactive power accumulation mode
/***************************************************************************//**
 * @brief This function is used to configure the accumulation mode for variable
 * reactive power in the ADE9153A device. It should be called after the
 * device has been initialized and before starting any measurements. The
 * function accepts a pointer to the device structure and an enumeration
 * value that specifies the desired accumulation mode. If the provided
 * device pointer is null, the function will return an error code
 * indicating that the device is not available. The function handles
 * different modes of accumulation, including signed, absolute, positive,
 * and negative modes, and defaults to signed mode if an invalid mode is
 * specified.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param varacc_type An enumeration value of type `ade9153a_accmode_e` that
 * specifies the desired accumulation mode. Valid values
 * include `ADE9153A_SIGNED_ACC_MODE`,
 * `ADE9153A_ABSOLUTE_VAL_ACC_MODE`,
 * `ADE9153A_POSITIVE_ACC_MODE`, and
 * `ADE9153A_NEGATIVE_ACC_MODE`. If an invalid value is
 * provided, the function defaults to signed accumulation
 * mode.
 * @return Returns an integer indicating the success or failure of the
 * operation. A positive value indicates success, while a negative value
 * indicates an error.
 ******************************************************************************/
int ade9153a_varacc(struct ade9153a_dev *dev,
		    enum ade9153a_accmode_e varacc_type);

// Select total active power accumulation mode
/***************************************************************************//**
 * @brief This function is used to configure the accumulation mode for total
 * active power in the ADE9153A device. It should be called after the
 * device has been initialized and is ready for configuration. The
 * function accepts a mode parameter that determines how the total active
 * power is accumulated, which can be set to signed, absolute, positive,
 * or negative accumulation modes. If the provided `dev` pointer is null,
 * the function will return an error code indicating that the device is
 * not available. The function may also return an error code if the
 * specified accumulation mode is invalid.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param wattacc_type An enumeration value of type `ade9153a_accmode_e` that
 * specifies the desired accumulation mode. Valid values
 * include `ADE9153A_SIGNED_ACC_MODE`,
 * `ADE9153A_ABSOLUTE_VAL_ACC_MODE`,
 * `ADE9153A_POSITIVE_ACC_MODE`, and
 * `ADE9153A_NEGATIVE_ACC_MODE`. If an invalid value is
 * provided, the function will default to signed
 * accumulation mode.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_wattacc(struct ade9153a_dev *dev,
		     enum ade9153a_accmode_e wattacc_type);

// Select peak detection channels
/***************************************************************************//**
 * @brief This function configures the peak detection settings for the ADE9153A
 * device. It should be called after the device has been initialized and
 * is ready for configuration. The function allows the user to enable or
 * disable peak detection for different phases based on the provided
 * selection. If an invalid selection is made, the function defaults to
 * disabling peak detection for both phases.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null. If null, the function will return an error code
 * indicating that the device is not available.
 * @param peak_sel An enumeration value of type `ade9153a_peak_sel_e` that
 * specifies which peak detection channels to enable or disable.
 * Valid values include options for enabling or disabling peak
 * detection for Phase A and Phase B. If an invalid value is
 * provided, the function will default to disabling peak
 * detection.
 * @return Returns an integer indicating the success or failure of the
 * operation. A negative value indicates an error, while a non-negative
 * value indicates success.
 ******************************************************************************/
int ade9153a_peak_sel(struct ade9153a_dev *dev,
		      enum ade9153a_peak_sel_e peak_sel);

// Ch B overcurretn detection enable.
/***************************************************************************//**
 * @brief This function should be called to enable overcurrent detection for
 * channel B of the ADE9153A device. It is important to ensure that the
 * device has been properly initialized before invoking this function. If
 * the `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. This function modifies
 * the internal state of the device to enable the overcurrent detection
 * feature.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_oib_en(struct ade9153a_dev *dev);

// Ch A overcurretn detection enable.
/***************************************************************************//**
 * @brief This function should be called to enable the overcurrent detection
 * feature for channel A of the ADE9153A device. It is important to
 * ensure that the device has been properly initialized before invoking
 * this function. If the `dev` parameter is null, the function will
 * return an error code indicating that the device is not available. This
 * function modifies the internal state of the device to enable the
 * overcurrent detection feature.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_oia_en(struct ade9153a_dev *dev);

// Zero crossing lpf disable.
/***************************************************************************//**
 * @brief This function is used to disable the zero crossing low-pass filter in
 * the ADE9153A device. It should be called when the device is properly
 * initialized and ready for configuration. If the provided device
 * pointer is null, the function will return an error code indicating
 * that the device is not available. Ensure that the device is in a state
 * where it can accept configuration changes before invoking this
 * function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_diszxlpf(struct ade9153a_dev *dev);

// Sign of CF2 power
/***************************************************************************//**
 * @brief This function is used to obtain the sign status of the CF2 power
 * output from the ADE9153A device. It should be called after the device
 * has been properly initialized and configured. The function checks for
 * valid input parameters and returns an error if the device or status
 * pointer is null. The status value will indicate whether the CF2 power
 * is positive or negative, and it is important to handle the return
 * value appropriately to ensure correct operation.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the sign status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_cf2sign_status(struct ade9153a_dev *dev, uint8_t *status);

// Sign of CF1 power
/***************************************************************************//**
 * @brief This function is used to obtain the sign status of the CF1 power
 * output from the ADE9153A device. It should be called after the device
 * has been properly initialized and configured. The function checks for
 * valid input parameters and reads the relevant register to determine
 * the sign status. If the device or status pointer is null, it will
 * return an error code. The function is expected to be called in a
 * context where the device is operational.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the sign status will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_cf1sign_status(struct ade9153a_dev *dev, uint8_t *status);

// Sign of reactive power
/***************************************************************************//**
 * @brief This function is used to obtain the status of the average sign from
 * the ADE9153A device. It should be called after the device has been
 * properly initialized. The function checks for valid pointers to the
 * device structure and the status output. If either pointer is null, it
 * returns an error code. Upon successful execution, the status is
 * written to the provided pointer, indicating the average sign status.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function returns -ENODEV.
 * @param status Pointer to a `uint8_t` where the average sign status will be
 * stored. Must not be null; otherwise, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_avarsign_status(struct ade9153a_dev *dev, uint8_t *status);

// Sign of active power
/***************************************************************************//**
 * @brief This function is used to obtain the AWSIGN status from the ADE9153A
 * device, which indicates the sign of the active power. It should be
 * called after the device has been properly initialized. The function
 * expects a valid pointer to a `struct ade9153a_dev` representing the
 * device and a pointer to a `uint8_t` variable where the status will be
 * stored. If the device pointer or the status pointer is null, the
 * function will return an error code. The function will read the
 * relevant register and update the status variable accordingly.
 *
 * @param dev A pointer to a `struct ade9153a_dev` representing the device. Must
 * not be null. If null, the function returns -ENODEV.
 * @param status A pointer to a `uint8_t` variable where the AWSIGN status will
 * be stored. Must not be null. If null, the function returns
 * -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure. The
 * `status` variable is updated with the AWSIGN status.
 ******************************************************************************/
int ade9153a_awsign_status(struct ade9153a_dev *dev, uint8_t *status);

// User period select.
/***************************************************************************//**
 * @brief This function is used to configure the user period selection for the
 * ADE9153A device. It should be called after the device has been
 * initialized and is ready for configuration. The `en` parameter
 * determines whether the user period is enabled or disabled. If the
 * `dev` pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param en A `uint8_t` value that specifies whether to enable (1) or disable
 * (0) the user period. Valid values are 0 and 1.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_uperiod_sel(struct ade9153a_dev *dev, uint8_t en);

// Hpf corner freq
/***************************************************************************//**
 * @brief This function configures the high pass filter (HPF) corner frequency
 * for the ADE9153A device. It should be called after the device has been
 * initialized and is ready for configuration. The function accepts a
 * specific frequency setting from the `ade9153a_hpf_crn_e` enumeration,
 * which defines various corner frequencies. If an invalid frequency is
 * provided, the function defaults to setting the corner frequency to
 * 38.695 Hz. It is important to ensure that the `dev` parameter is not
 * null before calling this function.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param hpf_corner_freq An enumeration value of type `ade9153a_hpf_crn_e` that
 * specifies the desired high pass filter corner
 * frequency. Valid values include 0.3125 Hz, 0.625 Hz,
 * 1.2475 Hz, 2.49 Hz, 4.9675 Hz, 9.895 Hz, and 19.6375
 * Hz. If an invalid value is provided, the function will
 * default to 38.695 Hz.
 * @return Returns an integer indicating the success or failure of the
 * operation. A negative value indicates an error, while a non-negative
 * value indicates success.
 ******************************************************************************/
int ade9153a_hpf_crn(struct ade9153a_dev *dev,
		     enum ade9153a_hpf_crn_e hpf_corner_freq);

// Set no. of samples for no load condition
/***************************************************************************//**
 * @brief This function configures the number of samples used to evaluate the no
 * load condition for the ADE9153A device. It should be called after the
 * device has been initialized and before starting any measurements. The
 * function accepts a specific enumeration value that defines the number
 * of samples, which can range from disabling the no load timer to
 * evaluating up to 4096 samples. If an invalid value is provided, the
 * function defaults to using 64 samples.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param noload_samples An enumeration value of type `ade9153a_noload_tmr_e`
 * that specifies the number of samples for the no load
 * condition. Valid values include
 * `ADE9153A_NOLOAD_TMR_DISABLE`,
 * `ADE9153A_NOLOAD_TMR_SAMPLES_128`,
 * `ADE9153A_NOLOAD_TMR_SAMPLES_256`,
 * `ADE9153A_NOLOAD_TMR_SAMPLES_512`,
 * `ADE9153A_NOLOAD_TMR_SAMPLES_1024`,
 * `ADE9153A_NOLOAD_TMR_SAMPLES_2048`, and
 * `ADE9153A_NOLOAD_TMR_SAMPLES_4096`. If an invalid value
 * is provided, the function defaults to
 * `ADE9153A_NOLOAD_TMR_SAMPLES_64`.
 * @return Returns an integer indicating the success or failure of the
 * operation. A negative value indicates an error, while a non-negative
 * value indicates success.
 ******************************************************************************/
int ade9153a_noload_tmr(struct ade9153a_dev *dev,
			enum ade9153a_noload_tmr_e noload_samples);

// Energy register read with value reset.
/***************************************************************************//**
 * @brief This function is used to enable the read reset feature of the ADE9153A
 * device. It should be called after the device has been properly
 * initialized. If the `dev` parameter is null, the function will return
 * an error code indicating that the device is not available. The
 * function modifies the appropriate register bits to enable the read
 * reset functionality, which may affect the behavior of the device
 * during operation.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_rd_rst_en(struct ade9153a_dev *dev);

// Internal energy register accum.
/***************************************************************************//**
 * @brief This function is used to enable or disable the energy accumulation
 * feature of the ADE9153A device. It should be called after the device
 * has been initialized and configured. The `en` parameter determines
 * whether the energy accumulation is enabled (1) or disabled (0). If the
 * `dev` pointer is null, the function will return an error code
 * indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param en A `uint8_t` value that specifies whether to enable (1) or disable
 * (0) energy accumulation. Valid values are 0 and 1.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_egy_ld_accum(struct ade9153a_dev *dev, uint8_t en);

// Energy accumulated based on the number of
// 4 kSPS samples or zero-crossing events
/***************************************************************************//**
 * @brief This function configures the energy timer mode for the ADE9153A
 * device. It should be called after the device has been initialized and
 * is ready for configuration. The `en` parameter determines whether the
 * energy timer mode is enabled or disabled. If the `dev` pointer is
 * null, the function will return an error code indicating that the
 * device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param en A `uint8_t` value that specifies the desired state of the energy
 * timer mode. Valid values are 0 (disable) and 1 (enable).
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_egy_tmr_mode(struct ade9153a_dev *dev, uint8_t en);

// Enable the energy and power accumulator when
// the run bit is also set.
/***************************************************************************//**
 * @brief This function should be called to enable the energy and power
 * accumulation features of the ADE9153A device. It is important to
 * ensure that the device has been properly initialized before invoking
 * this function. If the device pointer is null, the function will return
 * an error code indicating that the device is not available. This
 * function does not modify any input parameters.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_egy_pwr_en(struct ade9153a_dev *dev);

// Force CRC calculation to start (update).
/***************************************************************************//**
 * @brief This function should be called when a manual update of the CRC
 * calculation is required. It is important to ensure that the device has
 * been properly initialized before invoking this function. If the
 * provided device pointer is null, the function will return an error
 * code indicating that the device is not available. This function does
 * not modify any input parameters.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null, as a null pointer will result in an
 * error return code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_force_crc_update(struct ade9153a_dev *dev);

// Manually request a new temperature sensor reading.
/***************************************************************************//**
 * @brief This function initiates the temperature measurement process for the
 * ADE9153A device. It must be called after the device has been properly
 * initialized. If the provided device structure is null, the function
 * will return an error code indicating that the device is not available.
 * It is important to ensure that the device is ready for operation
 * before invoking this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code.
 * @return Returns 0 on success, or a negative error code if the device is not
 * available.
 ******************************************************************************/
int ade9153a_temp_start(struct ade9153a_dev *dev);

// Enable the temperature sensor.
/***************************************************************************//**
 * @brief This function should be called to enable the temperature sensor in the
 * ADE9153A device. It is important to ensure that the `dev` parameter is
 * properly initialized and not null before calling this function. If the
 * `dev` parameter is null, the function will return an error code
 * indicating that the device is not available. This function modifies
 * the device's configuration to allow temperature measurements.
 *
 * @param dev A pointer to an `ade9153a_dev` structure representing the device.
 * This pointer must not be null, and it should point to a valid
 * device that has been initialized. If the pointer is null, the
 * function will return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_temp_en(struct ade9153a_dev *dev);

// Select the number of temperature readings to average.
/***************************************************************************//**
 * @brief This function configures the number of temperature samples to be
 * averaged for temperature readings from the ADE9153A device. It must be
 * called after the device has been initialized and before starting
 * temperature measurements. The function accepts specific enumerated
 * values that determine the sampling interval, which can range from 1
 * millisecond to 1 second. If an invalid value is provided, the function
 * defaults to a sample interval of 1 millisecond.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param temp_no_samples An enumerated value of type `ade9153a_temp_time_e`
 * that specifies the number of temperature samples to
 * average. Valid values include
 * `ADE9153A_TEMP_TIME_SAMPLES_1`,
 * `ADE9153A_TEMP_TIME_SAMPLES_256`,
 * `ADE9153A_TEMP_TIME_SAMPLES_512`, and
 * `ADE9153A_TEMP_TIME_SAMPLES_1024`. If an invalid value
 * is provided, the function will default to
 * `ADE9153A_TEMP_TIME_SAMPLES_1`.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_temp_time(struct ade9153a_dev *dev,
		       enum ade9153a_temp_time_e temp_no_samples);

// Temperature result
/***************************************************************************//**
 * @brief This function retrieves the temperature result from the ADE9153A
 * device. It must be called after the device has been properly
 * initialized and configured. The caller should ensure that the device
 * is ready to provide temperature data. If the function is called with a
 * null pointer for the device or the output value, it will return an
 * error. The function will read the temperature result from the device's
 * register and store the result in the provided output parameter.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null. If null, the function returns -ENODEV.
 * @param val A pointer to a `uint16_t` where the temperature result will be
 * stored. Must not be null. If null, the function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_temp_result(struct ade9153a_dev *dev, uint16_t *val);

// COMPMODE
/***************************************************************************//**
 * @brief This function is used to configure the compensation mode of the
 * ADE9153A device. It should be called after the device has been
 * properly initialized. If the `dev` parameter is null, the function
 * will return an error code indicating that the device is not available.
 * The function writes a predefined value to the compensation mode
 * register, which affects how the device processes measurements.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null; otherwise, the function will
 * return an error code.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_compmode(struct ade9153a_dev *dev);

// Start measurements.
/***************************************************************************//**
 * @brief This function is used to initiate the measurement process of the
 * ADE9153A device. It must be called after the device has been properly
 * initialized and configured. If the device pointer is null, the
 * function will return an error code indicating that the device is not
 * available. It is important to ensure that the device is ready and
 * configured before invoking this function to avoid unexpected behavior.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This pointer must not be null; otherwise, the function will return
 * an error code.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null.
 ******************************************************************************/
int ade9153a_run(struct ade9153a_dev *dev);

// Stop measurements.
/***************************************************************************//**
 * @brief This function is used to stop the operation of the ADE9153A device. It
 * should be called when the device is no longer needed or before
 * reinitializing it. The function checks if the provided device pointer
 * is valid; if it is null, it returns an error code. It is important to
 * ensure that the device has been properly initialized before calling
 * this function.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return an error
 * code (-ENODEV).
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_stop(struct ade9153a_dev *dev);

// Set the signal side of the PGA.
/***************************************************************************//**
 * @brief This function is used to configure the signal side of the Programmable
 * Gain Amplifier (PGA) for the ADE9153A device. It should be called
 * after the device has been initialized and before starting any
 * measurements. The `set` parameter determines the configuration of the
 * signal side, and it is important to ensure that the `dev` parameter is
 * valid and not null. If the `dev` parameter is null, the function will
 * return an error code indicating that the device is not available.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param set A `uint8_t` value that specifies the desired configuration for the
 * signal side of the PGA. Valid values depend on the specific
 * configuration options defined in the driver.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_ai_swap(struct ade9153a_dev *dev, uint8_t set);

// Set the PGA gain for I chA.
/***************************************************************************//**
 * @brief This function is used to configure the programmable gain amplifier
 * (PGA) gain for the current channel A of the ADE9153A device. It should
 * be called after the device has been initialized and is ready for
 * configuration. The function accepts a specific gain value from the
 * `ade9153a_ai_gain_e` enumeration, which defines valid gain settings.
 * If the provided `dev` pointer is null, the function will return an
 * error code indicating that the device is not available. The function
 * will also handle invalid gain values by defaulting to a gain of 16.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param gain An enumeration value of type `ade9153a_ai_gain_e` that specifies
 * the desired PGA gain. Valid values include ADE9153A_AI_GAIN_16,
 * ADE9153A_AI_GAIN_24, ADE9153A_AI_GAIN_32, and
 * ADE9153A_AI_GAIN_38_4.
 * @return Returns an integer indicating the success or failure of the
 * operation. A negative value indicates an error, while a non-negative
 * value indicates success.
 ******************************************************************************/
int ade9153a_ai_gain(struct ade9153a_dev *dev, enum ade9153a_ai_gain_e gain);

// New run of mSure ready
/***************************************************************************//**
 * @brief This function is used to determine the readiness status of the mSure
 * system in the ADE9153A device. It should be called after the device
 * has been initialized and configured. The function checks the status
 * register and updates the provided status pointer with the result. If
 * the `dev` or `status` pointers are null, the function will return an
 * error code. It is important to ensure that the device is properly
 * initialized before calling this function to avoid unexpected behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the readiness status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_ms_sysrdy(struct ade9153a_dev *dev, uint8_t *status);

// Invalid config of mSure status
/***************************************************************************//**
 * @brief This function is used to determine if there are any configuration
 * errors in the mSure system. It should be called after the device has
 * been initialized and configured. The function checks the status of the
 * mSure system and updates the provided status pointer with the result.
 * If the `dev` or `status` pointers are null, the function will return
 * an error code. It is important to ensure that the device is properly
 * set up before calling this function to avoid unexpected results.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the status will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_ms_conferr(struct ade9153a_dev *dev, uint8_t *status);

// mSure not detected on the last enabled ch
/***************************************************************************//**
 * @brief This function is used to determine the absence of the mSure signal
 * from the ADE9153A device. It should be called after the device has
 * been properly initialized. The function checks the status register of
 * the device and updates the provided status pointer with the result. If
 * the device pointer or the status pointer is null, the function will
 * return an error code. It is important to handle these error codes
 * appropriately to ensure robust application behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the status will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_ms_absent(struct ade9153a_dev *dev, uint8_t *status);

// mSure timed out (600s)
/***************************************************************************//**
 * @brief This function is used to determine if the mSure process has timed out,
 * which is indicated by a specific status bit. It should be called after
 * the device has been initialized and configured properly. The function
 * expects a valid pointer to a `struct ade9153a_dev` instance and a
 * pointer to a `uint8_t` variable where the timeout status will be
 * stored. If the device pointer is null, it will return an error code
 * indicating that the device is not available. Similarly, if the status
 * pointer is null, it will return an error code for invalid arguments.
 *
 * @param dev A pointer to the `struct ade9153a_dev` representing the device.
 * Must not be null; otherwise, an error code -ENODEV is returned.
 * @param status A pointer to a `uint8_t` where the timeout status will be
 * stored. Must not be null; otherwise, an error code -EINVAL is
 * returned.
 * @return Returns 0 on success, indicating that the timeout status has been
 * successfully retrieved. If there is an error, a negative error code
 * is returned.
 ******************************************************************************/
int ade9153a_ms_timeout(struct ade9153a_dev *dev, uint8_t *status);

// mSure ready
/***************************************************************************//**
 * @brief This function is used to determine the readiness of the mSure system
 * within the ADE9153A device. It should be called after the device has
 * been initialized and configured. The function checks the status of the
 * mSure system and updates the provided status pointer with the result.
 * If the device pointer or the status pointer is null, the function will
 * return an error code. It is important to handle these error codes
 * appropriately to ensure robust application behavior.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the readiness status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure.
 ******************************************************************************/
int ade9153a_ms_ready(struct ade9153a_dev *dev, uint8_t *status);

// mSure shift detected in CC val
/***************************************************************************//**
 * @brief This function is used to check the status of the mSure shift detection
 * in the ADE9153A device. It should be called after the device has been
 * properly initialized and configured. The function will return an error
 * if the device pointer is null or if the status pointer is null. On
 * successful execution, the function writes the mSure shift status to
 * the provided status pointer.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param status Pointer to a `uint8_t` where the mSure shift status will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs.
 ******************************************************************************/
int ade9153a_ms_shift(struct ade9153a_dev *dev, uint8_t *status);

// Chip status
/***************************************************************************//**
 * @brief This function is used to obtain the current status of the ADE9153A
 * chip. It should be called after the device has been properly
 * initialized. The function checks for the presence of the device and
 * the validity of the status pointer before proceeding. If the device is
 * not present or the status pointer is null, it will return an error
 * code. The status is written to the location pointed to by the status
 * parameter, which must be a valid pointer to a `uint8_t` variable.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @param status A pointer to a `uint8_t` where the status will be stored. Must
 * not be null; otherwise, the function will return -EINVAL.
 * @return Returns 0 on success, or a negative error code if the device is not
 * present or if the status pointer is invalid.
 ******************************************************************************/
int ade9153a_chip_status(struct ade9153a_dev *dev, uint8_t *status);

// Temperature value in deg C
/***************************************************************************//**
 * @brief This function retrieves the temperature value from the ADE9153A
 * device, along with its offset and gain values. It must be called after
 * the device has been properly initialized and the temperature sensor
 * has been enabled. The function will block until the temperature
 * conversion is complete or a timeout occurs. If the device or data
 * pointer is null, it will return an error. The function also handles
 * the necessary steps to start the temperature measurement and check for
 * readiness.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param data Pointer to the `ade9153a_temperature_value` structure where the
 * temperature, offset, and gain values will be stored. Must not be
 * null.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ade9153a_temp_val(struct ade9153a_dev *dev,
		      struct ade9153a_temperature_value *data);

// Read energy values
/***************************************************************************//**
 * @brief This function retrieves the accumulated energy values from the
 * ADE9153A device, specifically the total active energy, fundamental
 * reactive energy, and total apparent energy. It must be called after
 * the device has been properly initialized and configured. The function
 * expects valid pointers for both the device structure and the data
 * structure to store the results. If either pointer is null, the
 * function will return an error code. The retrieved values are stored in
 * the provided `data` structure, which must be allocated by the caller.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param data Pointer to the `ade9153a_energy_values` structure where the
 * energy values will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operation.
 ******************************************************************************/
int ade9153a_energy_vals(struct ade9153a_dev *dev,
			 struct ade9153a_energy_values *data);

// Read power values
/***************************************************************************//**
 * @brief This function retrieves the active power, fundamental reactive power,
 * and apparent power values from the ADE9153A device. It must be called
 * after the device has been properly initialized and configured. If
 * either the `dev` or `data` pointers are null, the function will return
 * an error. The function reads the respective power values from the
 * device's registers and populates the provided `data` structure with
 * these values. It is important to handle the return value
 * appropriately, as it indicates success or failure of the read
 * operations.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param data A pointer to the `ade9153a_power_values` structure where the read
 * power values will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure that occurred during the read operations.
 ******************************************************************************/
int ade9153a_power_vals(struct ade9153a_dev *dev,
			struct ade9153a_power_values *data);

// Read rms values
/***************************************************************************//**
 * @brief This function retrieves the current RMS and voltage RMS values from
 * the ADE9153A device. It must be called after the device has been
 * properly initialized. If either the `dev` or `data` parameters are
 * null, the function will return an error code. The function reads the
 * current RMS value from the appropriate register and stores it in the
 * `current_rms_reg_val` field of the `data` structure, and similarly
 * reads the voltage RMS value into the `voltage_rms_reg_val` field. It
 * is important to handle the return value to check for any read errors.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param data A pointer to the `ade9153a_rms_values` structure where the RMS
 * values will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operations.
 ******************************************************************************/
int ade9153a_rms_vals(struct ade9153a_dev *dev,
		      struct ade9153a_rms_values *data);

// Read half rms values
/***************************************************************************//**
 * @brief This function retrieves the current and voltage half RMS values from
 * the ADE9153A device. It must be called after the device has been
 * properly initialized. If the `dev` or `data` parameters are null, the
 * function will return an error. The function reads the current half RMS
 * value and the voltage half RMS value from the device's registers and
 * stores them in the provided `data` structure. It is important to
 * handle the return value appropriately, as it indicates success or
 * failure of the read operations.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function returns -ENODEV.
 * @param data Pointer to the `ade9153a_half_rms_values` structure where the
 * retrieved values will be stored. Must not be null; otherwise, the
 * function returns -EINVAL.
 * @return Returns 0 on success, or a negative error code indicating the type of
 * failure encountered during the read operations.
 ******************************************************************************/
int ade9153a_half_rms_vals(struct ade9153a_dev *dev,
			   struct ade9153a_half_rms_values *data);

// Read power quaility values
/***************************************************************************//**
 * @brief This function retrieves power quality metrics from the ADE9153A
 * device, including the power factor, voltage RMS, and angle values. It
 * should be called after the device has been properly initialized and
 * configured. The function expects valid pointers for both the device
 * structure and the data structure to store the results. If either
 * pointer is null, the function will return an error code. The retrieved
 * values are stored in the provided `data` structure, which must be
 * allocated by the caller.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param data Pointer to the `ade9153a_pq_values` structure where the retrieved
 * power quality values will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the read operations.
 ******************************************************************************/
int ade9153a_power_quality_vals(struct ade9153a_dev *dev,
				struct ade9153a_pq_values *data);

// Start autocalibration AI channel
/***************************************************************************//**
 * @brief This function initiates the auto calibration process for the AI
 * channel of the ADE9153A device. It should be called after the device
 * has been properly initialized and is ready for operation. The function
 * checks the device status to ensure that the calibration engine is
 * available; if it is busy, the function will return an error. The
 * calibration mode can be specified using the `mode` parameter, which
 * allows for different operational modes during calibration.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * This parameter must not be null, as it is essential for
 * identifying the device on which the calibration is to be
 * performed.
 * @param mode An enumeration value of type `ade9153a_acalmode_e` that specifies
 * the calibration mode to be used. Valid values include `NORMAL`
 * and `TURBO`. The function will handle invalid values by returning
 * an error.
 * @return Returns 0 on success, indicating that the auto calibration process
 * has started. If the device is not initialized or busy, it returns a
 * negative error code.
 ******************************************************************************/
int ade9153a_start_autocal_ai(struct ade9153a_dev *dev,
			      enum ade9153a_acalmode_e mode);

// Start autocalibration BI channel
/***************************************************************************//**
 * @brief This function initiates the auto calibration process for the BI
 * channel of the ADE9153A device. It should be called after the device
 * has been properly initialized and is ready for calibration. The
 * function checks the status of the calibration engine; if it is busy,
 * the function will return an error. The mode of operation for the
 * calibration can be specified using the `mode` parameter, which allows
 * for different calibration modes to be selected. It is important to
 * ensure that the device is not already in a calibration process before
 * calling this function.
 *
 * @param dev A pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function will return -ENODEV.
 * @param mode An enumeration value of type `ade9153a_acalmode_e` that specifies
 * the calibration mode. Valid values are NORMAL and TURBO.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails, such as -ENODEV if the device is not initialized, or -EBUSY if
 * the calibration engine is currently busy.
 ******************************************************************************/
int ade9153a_start_autocal_bi(struct ade9153a_dev *dev,
			      enum ade9153a_acalmode_e mode);

// Start autocalibration AV channel
/***************************************************************************//**
 * @brief This function initiates the auto calibration process for the voltage
 * channel (AV) of the ADE9153A device. It should be called after the
 * device has been properly initialized and is ready for operation. The
 * function checks if the device is valid and if the calibration engine
 * is ready before proceeding. If the calibration engine is busy, the
 * function will return an error. The mode of operation can be specified,
 * allowing for different calibration strategies.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null.
 * @param mode An enumeration value of type `ade9153a_acalmode_e` that specifies
 * the calibration mode. Valid values are `NORMAL` and `TURBO`.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ade9153a_start_autocal_av(struct ade9153a_dev *dev,
			      enum ade9153a_acalmode_e mode);

// Read autocalibration values
/***************************************************************************//**
 * @brief This function retrieves autocalibration values from the ADE9153A
 * device, specifically the AICC, AICERT, AVCC, and AVCERT values. It
 * must be called after the device has been properly initialized and
 * configured. If the `dev` parameter is null, the function will return
 * an error. The function will also return an error if any of the
 * register reads fail, ensuring that the caller is informed of any
 * issues during the read process.
 *
 * @param dev Pointer to the `ade9153a_dev` structure representing the device.
 * Must not be null; otherwise, the function returns -ENODEV.
 * @param data Pointer to the `ade9153a_autocal_vals` structure where the
 * retrieved values will be stored. Must not be null; the function
 * will not write to this structure if the read operations fail.
 * @return Returns 0 on success, or a negative error code if any read operation
 * fails.
 ******************************************************************************/
int ade9153a_read_autocal_vals(struct ade9153a_dev *dev,
			       struct ade9153a_autocal_vals *data);

#endif // __ADE9153A_H__
