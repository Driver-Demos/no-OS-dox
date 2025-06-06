/***************************************************************************//**
 *   @file   ad7293.h
 *   @brief  Header file for ad7293 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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

#ifndef AD7293_H_
#define AD7293_H_

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

#define AD7293_R1B				NO_OS_BIT(16)
#define AD7293_R2B				NO_OS_BIT(17)
#define AD7293_PAGE_ADDR_MSK			NO_OS_GENMASK(15, 8)
#define AD7293_PAGE(x)				no_os_field_prep(AD7293_PAGE_ADDR_MSK, x)

/* AD7293 Register Map Common */
#define AD7293_REG_NO_OP			(AD7293_R1B | AD7293_PAGE(0x0) | 0x0)
#define AD7293_REG_PAGE_SELECT			(AD7293_R1B | AD7293_PAGE(0x0) | 0x1)
#define AD7293_REG_CONV_CMD			(AD7293_R2B | AD7293_PAGE(0x0) | 0x2)
#define AD7293_REG_RESULT			(AD7293_R1B | AD7293_PAGE(0x0) | 0x3)
#define AD7293_REG_DAC_EN			(AD7293_R1B | AD7293_PAGE(0x0) | 0x4)
#define AD7293_REG_DEVICE_ID			(AD7293_R2B | AD7293_PAGE(0x0) | 0xC)
#define AD7293_REG_SOFT_RESET			(AD7293_R2B | AD7293_PAGE(0x0) | 0xF)

/* AD7293 Register Map Page 0x0 */
#define AD7293_REG_VIN0				(AD7293_R2B | AD7293_PAGE(0x0) | 0x10)
#define AD7293_REG_VIN1				(AD7293_R2B | AD7293_PAGE(0x0) | 0x11)
#define AD7293_REG_VIN2				(AD7293_R2B | AD7293_PAGE(0x0) | 0x12)
#define AD7293_REG_VIN3				(AD7293_R2B | AD7293_PAGE(0x0) | 0x13)
#define AD7293_REG_TSENSE_INT			(AD7293_R2B | AD7293_PAGE(0x0) | 0x20)
#define AD7293_REG_TSENSE_D0			(AD7293_R2B | AD7293_PAGE(0x0) | 0x21)
#define AD7293_REG_TSENSE_D1			(AD7293_R2B | AD7293_PAGE(0x0) | 0x22)
#define AD7293_REG_ISENSE_0			(AD7293_R2B | AD7293_PAGE(0x0) | 0x28)
#define AD7293_REG_ISENSE_1			(AD7293_R2B | AD7293_PAGE(0x0) | 0x29)
#define AD7293_REG_ISENSE_2			(AD7293_R2B | AD7293_PAGE(0x0) | 0x2A)
#define AD7293_REG_ISENSE_3			(AD7293_R2B | AD7293_PAGE(0x0) | 0x2B)
#define AD7293_REG_UNI_VOUT0			(AD7293_R2B | AD7293_PAGE(0x0) | 0x30)
#define AD7293_REG_UNI_VOUT1			(AD7293_R2B | AD7293_PAGE(0x0) | 0x31)
#define AD7293_REG_UNI_VOUT2			(AD7293_R2B | AD7293_PAGE(0x0) | 0x32)
#define AD7293_REG_UNI_VOUT3			(AD7293_R2B | AD7293_PAGE(0x0) | 0x33)
#define AD7293_REG_BI_VOUT0			(AD7293_R2B | AD7293_PAGE(0x0) | 0x34)
#define AD7293_REG_BI_VOUT1			(AD7293_R2B | AD7293_PAGE(0x0) | 0x35)
#define AD7293_REG_BI_VOUT2			(AD7293_R2B | AD7293_PAGE(0x0) | 0x36)
#define AD7293_REG_BI_VOUT3			(AD7293_R2B | AD7293_PAGE(0x0) | 0x37)

/* AD7293 Register Map Page 0x01 */
#define AD7293_REG_AVDD				(AD7293_R2B | AD7293_PAGE(0x01) | 0x10)
#define AD7293_REG_DACVDD_UNI			(AD7293_R2B | AD7293_PAGE(0x01) | 0x11)
#define AD7293_REG_DACVDD_BI			(AD7293_R2B | AD7293_PAGE(0x01) | 0x12)
#define AD7293_REG_AVSS				(AD7293_R2B | AD7293_PAGE(0x01) | 0x13)
#define AD7293_REG_BI_VOUT0_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x14)
#define AD7293_REG_BI_VIOU1_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x15)
#define AD7293_REG_BI_VOUT2_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x16)
#define AD7293_REG_BI_VOUT3_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x17)
#define AD7293_REG_RS0_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x28)
#define AD7293_REG_RS1_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x29)
#define AD7293_REG_RS2_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x2A)
#define AD7293_REG_RS3_MON			(AD7293_R2B | AD7293_PAGE(0x01) | 0x2B)

/* AD7293 Register Map Page 0x2 */
#define AD7293_REG_DIGITAL_OUT_EN		(AD7293_R2B | AD7293_PAGE(0x2) | 0x11)
#define AD7293_REG_DIGITAL_INOUT_FUNC		(AD7293_R2B | AD7293_PAGE(0x2) | 0x12)
#define AD7293_REG_DIGITAL_FUNC_POL		(AD7293_R2B | AD7293_PAGE(0x2) | 0x13)
#define AD7293_REG_GENERAL			(AD7293_R2B | AD7293_PAGE(0x2) | 0x14)
#define AD7293_REG_VINX_RANGE0			(AD7293_R2B | AD7293_PAGE(0x2) | 0x15)
#define AD7293_REG_VINX_RANGE1			(AD7293_R2B | AD7293_PAGE(0x2) | 0x16)
#define AD7293_REG_VINX_DIFF_SE			(AD7293_R2B | AD7293_PAGE(0x2) | 0x17)
#define AD7293_REG_VINX_FILTER			(AD7293_R2B | AD7293_PAGE(0x2) | 0x18)
#define AD7293_REG_BG_EN			(AD7293_R2B | AD7293_PAGE(0x2) | 0x19)
#define AD7293_REG_CONV_DELAY			(AD7293_R2B | AD7293_PAGE(0x2) | 0x1A)
#define AD7293_REG_TSENSE_BG_EN			(AD7293_R2B | AD7293_PAGE(0x2) | 0x1B)
#define AD7293_REG_ISENSE_BG_EN			(AD7293_R2B | AD7293_PAGE(0x2) | 0x1C)
#define AD7293_REG_ISENSE_GAIN			(AD7293_R2B | AD7293_PAGE(0x2) | 0x1D)
#define AD7293_REG_DAC_SNOOZE_O			(AD7293_R2B | AD7293_PAGE(0x2) | 0x1F)
#define AD7293_REG_DAC_SNOOZE_1			(AD7293_R2B | AD7293_PAGE(0x2) | 0x20)
#define AD7293_REG_RSX_MON_BG_EN		(AD7293_R2B | AD7293_PAGE(0x2) | 0x23)
#define AD7293_REG_INTEGR_CL			(AD7293_R2B | AD7293_PAGE(0x2) | 0x28)
#define AD7293_REG_PA_ON_CTRL			(AD7293_R2B | AD7293_PAGE(0x2) | 0x29)
#define AD7293_REG_RAMP_TIME_0			(AD7293_R2B | AD7293_PAGE(0x2) | 0x2A)
#define AD7293_REG_RAMP_TIME_1			(AD7293_R2B | AD7293_PAGE(0x2) | 0x2B)
#define AD7293_REG_RAMP_TIME_2			(AD7293_R2B | AD7293_PAGE(0x2) | 0x2C)
#define AD7293_REG_RAMP_TIME_3			(AD7293_R2B | AD7293_PAGE(0x2) | 0x2D)
#define AD7293_REG_CL_FR_IT			(AD7293_R2B | AD7293_PAGE(0x2) | 0x2E)
#define AD7293_REG_INTX_AVSS_AVDD		(AD7293_R2B | AD7293_PAGE(0x2) | 0x2F)

/* AD7293 Register Map Page 0x3 */
#define AD7293_REG_VINX_SEQ			(AD7293_R2B | AD7293_PAGE(0x3) | 0x10)
#define AD7293_REG_ISENSEX_TSENSEX_SEQ		(AD7293_R2B | AD7293_PAGE(0x3) | 0x11)
#define AD7293_REG_RSX_MON_BI_VOUTX_SEQ		(AD7293_R2B | AD7293_PAGE(0x3) | 0x12)

/* AD7293 Register Map Page 0x05 */
#define AD7293_REG_AVDD_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x10)
#define AD7293_REG_DACVDD_UNI_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x11)
#define AD7293_REG_DACVDD_BI_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x12)
#define AD7293_REG_AVSS_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_HL		(AD7293_R2B | AD7293_PAGE(0x05) | 0x17)
#define AD7293_REG_RS0_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x28)
#define AD7293_REG_RS1_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x29)
#define AD7293_REG_RS2_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x2A)
#define AD7293_REG_RS3_MON_HL			(AD7293_R2B | AD7293_PAGE(0x05) | 0x2B)

/* AD7293 Register Map Page 0x06 */
#define AD7293_REG_VIN0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x10)
#define AD7293_REG_VIN1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x11)
#define AD7293_REG_VIN2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x12)
#define AD7293_REG_VIN3_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x13)
#define AD7293_REG_TSENSE_D0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x20)
#define AD7293_REG_TSENSE_D1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x21)
#define AD7293_REG_TSENSE_D2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x22)
#define AD7293_REG_ISENSE0_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x28)
#define AD7293_REG_ISENSE1_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x29)
#define AD7293_REG_ISENSE2_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x2A)
#define AD7293_REG_ISENSE3_LL			(AD7293_R2B | AD7293_PAGE(0x06) | 0x2B)

/* AD7293 Register Map Page 0x07 */
#define AD7293_REG_AVDD_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x10)
#define AD7293_REG_DACVDD_UNI_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x11)
#define AD7293_REG_DACVDD_BI_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x12)
#define AD7293_REG_AVSS_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_LL		(AD7293_R2B | AD7293_PAGE(0x07) | 0x17)
#define AD7293_REG_RS0_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x28)
#define AD7293_REG_RS1_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x29)
#define AD7293_REG_RS2_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x2A)
#define AD7293_REG_RS3_MON_LL			(AD7293_R2B | AD7293_PAGE(0x07) | 0x2B)

/* AD7293 Register Map Page 0x08 */
#define AD7293_REG_VIN0_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x10)
#define AD7293_REG_VIN1_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x11)
#define AD7293_REG_VIN2_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x12)
#define AD7293_REG_VIN3_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x13)
#define AD7293_REG_TSENSE_INT_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x20)
#define AD7293_REG_TSENSE_D0_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x21)
#define AD7293_REG_TSENSE_D1_HYS		(AD7293_R2B | AD7293_PAGE(0x08) | 0x22)
#define AD7293_REG_ISENSE0_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x28)
#define AD7293_REG_ISENSE1_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x29)
#define AD7293_REG_ISENSE2_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x2A)
#define AD7293_REG_ISENSE3_HYS			(AD7293_R2B | AD7293_PAGE(0x08) | 0x2B)

/* AD7293 Register Map Page 0x09 */
#define AD7293_REG_AVDD_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x10)
#define AD7293_REG_DACVDD_UNI_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x11)
#define AD7293_REG_DACVDD_BI_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x12)
#define AD7293_REG_AVSS_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_HYS		(AD7293_R2B | AD7293_PAGE(0x09) | 0x17)
#define AD7293_REG_RS0_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x28)
#define AD7293_REG_RS1_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x29)
#define AD7293_REG_RS2_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x2A)
#define AD7293_REG_RS3_MON_HYS			(AD7293_R2B | AD7293_PAGE(0x09) | 0x2B)

/* AD7293 Register Map Page 0x0A */
#define AD7293_REG_VIN0_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x10)
#define AD7293_REG_VIN1_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x11)
#define AD7293_REG_VIN2_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x12)
#define AD7293_REG_VIN3_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x13)
#define AD7293_REG_TSENSE_INT_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x20)
#define AD7293_REG_TSENSE_D0_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x21)
#define AD7293_REG_TSENSE_D1_MIN		(AD7293_R2B | AD7293_PAGE(0x0A) | 0x22)
#define AD7293_REG_ISENSE0_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x28)
#define AD7293_REG_ISENSE1_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x29)
#define AD7293_REG_ISENSE2_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x2A)
#define AD7293_REG_ISENSE3_MIN			(AD7293_R2B | AD7293_PAGE(0x0A) | 0x2B)

/* AD7293 Register Map Page 0x0B */
#define AD7293_REG_AVDD_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x10)
#define AD7293_REG_DACVDD_UNI_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x11)
#define AD7293_REG_DACVDD_BI_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x12)
#define AD7293_REG_AVSS_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_MIN		(AD7293_R2B | AD7293_PAGE(0x0B) | 0x17)
#define AD7293_REG_RS0_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x28)
#define AD7293_REG_RS1_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x29)
#define AD7293_REG_RS2_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x2A)
#define AD7293_REG_RS3_MON_MIN			(AD7293_R2B | AD7293_PAGE(0x0B) | 0x2B)

/* AD7293 Register Map Page 0x0C */
#define AD7293_REG_VIN0_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x10)
#define AD7293_REG_VIN1_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x11)
#define AD7293_REG_VIN2_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x12)
#define AD7293_REG_VIN3_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x13)
#define AD7293_REG_TSENSE_INT_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x20)
#define AD7293_REG_TSENSE_D0_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x21)
#define AD7293_REG_TSENSE_D1_MAX		(AD7293_R2B | AD7293_PAGE(0x0C) | 0x22)
#define AD7293_REG_ISENSE0_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x28)
#define AD7293_REG_ISENSE1_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x29)
#define AD7293_REG_ISENSE2_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x2A)
#define AD7293_REG_ISENSE3_MAX			(AD7293_R2B | AD7293_PAGE(0x0C) | 0x2B)

/* AD7293 Register Map Page 0x0D */
#define AD7293_REG_AVDD_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x10)
#define AD7293_REG_DACVDD_UNI_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x11)
#define AD7293_REG_DACVDD_BI_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x12)
#define AD7293_REG_AVSS_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_MAX		(AD7293_R2B | AD7293_PAGE(0x0D) | 0x17)
#define AD7293_REG_RS0_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x28)
#define AD7293_REG_RS1_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x29)
#define AD7293_REG_RS2_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x2A)
#define AD7293_REG_RS3_MON_MAX			(AD7293_R2B | AD7293_PAGE(0x0D) | 0x2B)

/* AD7293 Register Map Page 0xE */
#define AD7293_REG_VIN0_OFFSET			(AD7293_R1B | AD7293_PAGE(0xE) | 0x10)
#define AD7293_REG_VIN1_OFFSET			(AD7293_R1B | AD7293_PAGE(0xE) | 0x11)
#define AD7293_REG_VIN2_OFFSET			(AD7293_R1B | AD7293_PAGE(0xE) | 0x12)
#define AD7293_REG_VIN3_OFFSET			(AD7293_R1B | AD7293_PAGE(0xE) | 0x13)
#define AD7293_REG_TSENSE_INT_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x20)
#define AD7293_REG_TSENSE_D0_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x21)
#define AD7293_REG_TSENSE_D1_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x22)
#define AD7293_REG_ISENSE0_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x28)
#define AD7293_REG_ISENSE1_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x29)
#define AD7293_REG_ISENSE2_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x2A)
#define AD7293_REG_ISENSE3_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x2B)
#define AD7293_REG_UNI_VOUT0_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x30)
#define AD7293_REG_UNI_VOUT1_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x31)
#define AD7293_REG_UNI_VOUT2_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x32)
#define AD7293_REG_UNI_VOUT3_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x33)
#define AD7293_REG_BI_VOUT0_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x34)
#define AD7293_REG_BI_VOUT1_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x35)
#define AD7293_REG_BI_VOUT2_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x36)
#define AD7293_REG_BI_VOUT3_OFFSET		(AD7293_R1B | AD7293_PAGE(0xE) | 0x37)

/* AD7293 Register Map Page 0x0F */
#define AD7293_REG_AVDD_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0F) | 0x10)
#define AD7293_REG_DACVDD_UNI_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x11)
#define AD7293_REG_DACVDD_BI_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x12)
#define AD7293_REG_AVSS_OFFSET			(AD7293_R1B | AD7293_PAGE(0x0F) | 0x13)
#define AD7293_REG_BI_VOUT0_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x14)
#define AD7293_REG_BI_VOUT1_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x15)
#define AD7293_REG_BI_VOUT2_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x16)
#define AD7293_REG_BI_VOUT3_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x17)
#define AD7293_REG_RS0_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x28)
#define AD7293_REG_RS1_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x29)
#define AD7293_REG_RS2_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x2A)
#define AD7293_REG_RS3_MON_OFFSET		(AD7293_R1B | AD7293_PAGE(0x0F) | 0x2B)

/* AD7293 Register Map Page 0x10 */
#define AD7293_REG_ALERT_SUM			(AD7293_R2B | AD7293_PAGE(0x10) | 0x10)
#define AD7293_REG_VINX_ALERT			(AD7293_R2B | AD7293_PAGE(0x10) | 0x12)
#define AD7293_REG_TSENSEX_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x14)
#define AD7293_REG_ISENSEX_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x18)
#define AD7293_REG_RSX_MON_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT		(AD7293_R2B | AD7293_PAGE(0x10) | 0x1A)

/* AD7293 Register Map Page 0x11 */
#define AD7293_REG_VINX_ALERT0			(AD7293_R2B | AD7293_PAGE(0x11) | 0x12)
#define AD7293_REG_TSENSEX_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x14)
#define AD7293_REG_ISENSEX_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x18)
#define AD7293_REG_RSX_MON_ALERT0		(AD7293_R2B | AD7293_PAGE(0x11) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT0	(AD7293_R2B | AD7293_PAGE(0x11) | 0x1A)

/* AD7293 Register Map Page 0x12 */
#define AD7293_REG_VINX_ALERT1			(AD7293_R2B | AD7293_PAGE(0x12) | 0x12)
#define AD7293_REG_TSENSEX_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x14)
#define AD7293_REG_ISENSEX_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x15)
#define AD7293_REG_BI_VOUTX_MON_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x18)
#define AD7293_REG_RSX_MON_ALERT1		(AD7293_R2B | AD7293_PAGE(0x12) | 0x19)
#define AD7293_REG_INT_LIMIT_AVSS_ALERT1	(AD7293_R2B | AD7293_PAGE(0x12) | 0x1A)

/* AD7293 Miscellaneous Definitions */
#define AD7293_READ				NO_OS_BIT(7)
#define AD7293_TRANSF_LEN_MSK			NO_OS_GENMASK(17, 16)
#define AD7293_BUFF_SIZE_BYTES			3
#define AD7293_REG_ADDR_MSK			NO_OS_GENMASK(7, 0)
#define AD7293_REG_VOUT_OFFSET_MSK		NO_OS_GENMASK(5, 4)
#define AD7293_REG_DATA_RAW_MSK			NO_OS_GENMASK(15, 4)
#define AD7293_REG_VINX_RANGE_GET_CH_MSK(x, ch)	(((x) >> (ch)) & 0x1)
#define AD7293_REG_VINX_RANGE_SET_CH_MSK(x, ch)	(((x) & 0x1) << (ch))
#define AD7293_CHIP_ID				0x18
#define AD7293_SOFT_RESET_VAL			0x7293
#define AD7293_SOFT_RESET_CLR_VAL		0x0000
#define AD7293_CONV_CMD_VAL			0x82

/***************************************************************************//**
 * @brief The `ad7293_ch_type` enumeration defines the different types of
 * channels available in the AD7293 device, including ADC channels for
 * voltage, temperature, and current sensing, as well as a DAC channel.
 * This enumeration is used to specify the type of channel being
 * configured or accessed in the AD7293 device.
 *
 * @param AD7293_ADC_VINX Represents an ADC channel for voltage input.
 * @param AD7293_ADC_TSENSE Represents an ADC channel for temperature sensing.
 * @param AD7293_ADC_ISENSE Represents an ADC channel for current sensing.
 * @param AD7293_DAC Represents a DAC channel.
 ******************************************************************************/
enum ad7293_ch_type {
	AD7293_ADC_VINX,
	AD7293_ADC_TSENSE,
	AD7293_ADC_ISENSE,
	AD7293_DAC,
};

/***************************************************************************//**
 * @brief The `ad7293_dev` structure is a device descriptor for the AD7293, a
 * multi-channel ADC/DAC device. It encapsulates the necessary components
 * for interfacing with the device, including SPI communication and GPIO
 * control for resetting the device. The `page_select` member is used to
 * manage the page addressing of the device's register map, allowing
 * access to different sets of registers.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_reset Pointer to the GPIO descriptor for reset control.
 * @param page_select 8-bit value to select the current page for register
 * access.
 ******************************************************************************/
struct ad7293_dev {
	/** SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	struct no_os_gpio_desc		*gpio_reset;
	uint8_t				page_select;
};

/***************************************************************************//**
 * @brief The `ad7293_init_param` structure is used to encapsulate the
 * initialization parameters required for setting up the AD7293 device.
 * It includes pointers to SPI and GPIO initialization parameters, which
 * are essential for configuring the communication interface and reset
 * functionality of the device, respectively. This structure is typically
 * used during the device initialization process to ensure that all
 * necessary hardware interfaces are properly configured.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_reset Pointer to GPIO initialization parameters for reset.
 ******************************************************************************/
struct ad7293_init_param {
	/** SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	struct no_os_gpio_init_param	*gpio_reset;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief This function is used to read data from a specified register of the
 * AD7293 device using SPI communication. It requires a valid device
 * descriptor and a register address to read from. The function will
 * store the read value in the provided memory location pointed to by the
 * `val` parameter. It is essential that the device has been properly
 * initialized before calling this function. The function handles
 * different register lengths and adjusts the read operation accordingly.
 * It returns an error code if the read operation fails, allowing the
 * caller to handle such cases appropriately.
 *
 * @param dev A pointer to an `ad7293_dev` structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg An unsigned integer representing the register address to read
 * from. Must be a valid register address for the AD7293 device.
 * @param val A pointer to a `uint16_t` where the read value will be stored.
 * Must not be null, and the caller is responsible for ensuring it
 * points to a valid memory location.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int ad7293_spi_read(struct ad7293_dev *dev, unsigned int reg, uint16_t *val);

/***************************************************************************//**
 * @brief This function is used to write a 16-bit value to a specific register
 * on the AD7293 device using SPI communication. It should be called when
 * a register needs to be updated with a new value. The function requires
 * a valid device descriptor and a register address, and it handles the
 * necessary page selection and data formatting for the SPI transaction.
 * Ensure that the device is properly initialized before calling this
 * function. The function returns an error code if the operation fails,
 * which can be used for error handling.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg An unsigned integer representing the register address to write to.
 * Must be a valid register address for the AD7293 device.
 * @param val A 16-bit unsigned integer value to be written to the specified
 * register.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int ad7293_spi_write(struct ad7293_dev *dev, unsigned int reg, uint16_t val);

/***************************************************************************//**
 * @brief Use this function to modify specific bits in a register of the AD7293
 * device without affecting other bits. This is useful when only certain
 * bits need to be changed, preserving the rest of the register's current
 * state. The function reads the current value of the register, applies
 * the mask to clear the bits to be updated, and then sets them to the
 * desired value. It must be called with a valid device descriptor and
 * register address. Ensure that the device is properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg The register address within the AD7293 device to be updated. Must
 * be a valid register address.
 * @param mask A 16-bit mask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param val A 16-bit value containing the new bit values to be written to the
 * register. Only bits corresponding to the mask will be used.
 * @return Returns 0 on success, or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int ad7293_spi_update_bits(struct ad7293_dev *dev, unsigned int reg,
			   uint16_t mask, uint16_t val);

/***************************************************************************//**
 * @brief Use this function to obtain the current ADC range setting for a
 * specific channel on the AD7293 device. This function should be called
 * when you need to verify or utilize the ADC range configuration for a
 * channel. Ensure that the device has been properly initialized before
 * calling this function. The function reads from the device's registers
 * to determine the range and stores the result in the provided pointer.
 * It returns an error code if the read operation fails.
 *
 * @param dev A pointer to an initialized ad7293_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number for which the ADC range is being queried. Valid
 * channel numbers depend on the device configuration.
 * @param range A pointer to a uint16_t where the function will store the
 * retrieved ADC range. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ad7293_adc_get_range(struct ad7293_dev *dev, unsigned int ch,
			 uint16_t *range);

/***************************************************************************//**
 * @brief This function configures the input range of a specified ADC channel on
 * the AD7293 device. It should be called when you need to adjust the
 * input range for accurate measurements on a particular channel. Ensure
 * that the device is properly initialized before calling this function.
 * The function updates the range settings in two registers, and any
 * error during these updates will result in a non-zero return value. It
 * is important to handle these errors to ensure the range is set
 * correctly.
 *
 * @param dev A pointer to an initialized ad7293_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number for which the range is being set. It should be a
 * valid channel index for the AD7293 device.
 * @param range A 16-bit value representing the desired range setting for the
 * specified channel. The exact valid values depend on the device's
 * range configuration capabilities.
 * @return Returns 0 on success, or a negative error code if the range setting
 * fails.
 ******************************************************************************/
int ad7293_adc_set_range(struct ad7293_dev *dev, unsigned int ch,
			 uint16_t range);

/***************************************************************************//**
 * @brief This function configures the gain for a specific ISENSE channel on the
 * AD7293 device. It should be used when you need to adjust the gain
 * settings for current sensing applications. The function requires a
 * valid device descriptor and channel index. Ensure that the device is
 * properly initialized before calling this function. The gain value is
 * applied to the specified channel, and the function returns an integer
 * status code indicating success or failure.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param ch An unsigned integer representing the ISENSE channel index. Valid
 * range depends on the device's channel configuration.
 * @param gain A 16-bit unsigned integer specifying the gain value to set for
 * the channel. The valid range is determined by the device's
 * specifications.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ad7293_isense_set_gain(struct ad7293_dev *dev, unsigned int ch,
			   uint16_t gain);

/***************************************************************************//**
 * @brief Use this function to obtain the gain setting for a specific ISENSE
 * channel on the AD7293 device. It is essential to ensure that the
 * device is properly initialized before calling this function. The
 * function reads the gain value from the device and extracts the
 * relevant bits corresponding to the specified channel. This function is
 * useful for applications that need to verify or log the current gain
 * settings of the ISENSE channels. It is important to handle the return
 * value to check for any errors during the SPI read operation.
 *
 * @param dev A pointer to an initialized ad7293_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number for which the gain is to be retrieved. Valid
 * range is typically 0 to 3, corresponding to the available ISENSE
 * channels.
 * @param gain A pointer to a uint16_t variable where the gain value will be
 * stored. Must not be null. The function will write the gain value
 * for the specified channel to this location.
 * @return Returns 0 on success, or a negative error code if the SPI read
 * operation fails.
 ******************************************************************************/
int ad7293_isense_get_gain(struct ad7293_dev *dev, unsigned int ch,
			   uint16_t *gain);

/***************************************************************************//**
 * @brief This function is used to obtain the offset value for a specific
 * channel type and channel number on the AD7293 device. It is essential
 * to call this function when you need to read the offset calibration
 * value for ADC, DAC, or other channel types supported by the device.
 * The function requires a valid device descriptor and channel type, and
 * it will return an error if an unsupported channel type is provided.
 * The offset value is returned through a pointer, which must not be
 * null.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null.
 * @param type An enum ad7293_ch_type value indicating the type of channel
 * (e.g., AD7293_ADC_VINX, AD7293_ADC_TSENSE, etc.). Must be a valid
 * channel type.
 * @param ch An unsigned integer representing the channel number. The valid
 * range depends on the channel type.
 * @param offset A pointer to a uint16_t where the offset value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the channel type is
 * invalid or if there is a communication error.
 ******************************************************************************/
int ad7293_get_offset(struct ad7293_dev *dev,  enum ad7293_ch_type type,
		      unsigned int ch, uint16_t *offset);

/***************************************************************************//**
 * @brief This function is used to configure the offset for a specific channel
 * type on the AD7293 device. It should be called when you need to adjust
 * the offset for ADC, DAC, or sensor channels. The function requires a
 * valid device descriptor and channel type, and it returns an error if
 * the channel type is not recognized. Ensure that the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param type An enum value of type ad7293_ch_type indicating the channel type
 * for which the offset is being set. Valid values are
 * AD7293_ADC_VINX, AD7293_ADC_TSENSE, AD7293_ADC_ISENSE, and
 * AD7293_DAC. If an invalid type is provided, the function returns
 * an error.
 * @param ch An unsigned integer representing the channel number. The valid
 * range depends on the channel type and should correspond to a valid
 * channel on the device.
 * @param offset A 16-bit unsigned integer specifying the offset value to be set
 * for the specified channel.
 * @return Returns 0 on success or a negative error code if the channel type is
 * invalid or if there is a failure in writing the offset.
 ******************************************************************************/
int ad7293_set_offset(struct ad7293_dev *dev,  enum ad7293_ch_type type,
		      unsigned int ch, uint16_t offset);

/***************************************************************************//**
 * @brief Use this function to set a raw digital-to-analog converter (DAC) value
 * for a specific channel on the AD7293 device. This function should be
 * called when you need to update the DAC output for a given channel.
 * Ensure that the device is properly initialized before calling this
 * function. The function enables the DAC for the specified channel and
 * writes the raw value to it. It returns an error code if the operation
 * fails, which can occur if the channel number is invalid or if there is
 * a communication error with the device.
 *
 * @param dev A pointer to an initialized ad7293_dev structure representing the
 * device. Must not be null.
 * @param ch The channel number to which the raw DAC value will be written.
 * Valid channel numbers depend on the device configuration.
 * @param raw The raw DAC value to be written to the specified channel. It is a
 * 16-bit unsigned integer.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ad7293_dac_write_raw(struct ad7293_dev *dev, unsigned int ch,
			 uint16_t raw);

/***************************************************************************//**
 * @brief Use this function to read a raw data value from a specified channel of
 * the AD7293 device. The function supports reading from ADC voltage
 * input channels, temperature sensor channels, current sensor channels,
 * and DAC channels. It requires a valid device descriptor and channel
 * type, and the channel index must be within the valid range for the
 * specified type. The function will return an error if the channel type
 * is invalid or if any SPI communication fails. Ensure the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. Must
 * not be null.
 * @param type An enum ad7293_ch_type value indicating the type of channel to
 * read from. Valid values are AD7293_ADC_VINX, AD7293_ADC_TSENSE,
 * AD7293_ADC_ISENSE, and AD7293_DAC.
 * @param ch An unsigned integer representing the channel index to read from.
 * Must be within the valid range for the specified channel type.
 * @param raw A pointer to a uint16_t where the raw data will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code on failure. The raw
 * data is stored in the location pointed to by the raw parameter.
 ******************************************************************************/
int ad7293_ch_read_raw(struct ad7293_dev *dev, enum ad7293_ch_type type,
		       unsigned int ch, uint16_t *raw);

/***************************************************************************//**
 * @brief Use this function to reset the AD7293 device to its default state via
 * a software command. This function is typically called when the device
 * needs to be reinitialized or after a configuration change that
 * requires a reset. It must be called with a valid device descriptor
 * that has been properly initialized. The function attempts to write
 * specific reset values to the device's reset register and returns an
 * error code if the operation fails.
 *
 * @param dev A pointer to an initialized ad7293_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if communication with the
 * device fails.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad7293_soft_reset(struct ad7293_dev *dev);

/***************************************************************************//**
 * @brief This function resets the AD7293 device by either toggling the reset
 * GPIO pin if available or performing a software reset if the GPIO pin
 * is not configured. It should be used to ensure the device is in a
 * known state before starting operations. The function must be called
 * with a valid device descriptor that has been properly initialized. If
 * the GPIO reset pin is configured, it will be used to perform a
 * hardware reset; otherwise, a software reset will be executed.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device. This
 * must be a valid, initialized device descriptor. If the gpio_reset
 * member is non-null, it will be used for a hardware reset;
 * otherwise, a software reset will be performed.
 * @return Returns 0 on success, or a negative error code if the reset operation
 * fails.
 ******************************************************************************/
int ad7293_reset(struct ad7293_dev *dev);

/***************************************************************************//**
 * @brief This function initializes the AD7293 device by setting up the
 * necessary SPI and GPIO configurations as specified in the
 * initialization parameters. It must be called before any other
 * operations on the AD7293 device. The function allocates memory for the
 * device structure, initializes the SPI interface, and optionally
 * configures a GPIO for reset. It also performs a device reset and
 * verifies the chip ID to ensure the device is correctly connected and
 * operational. If any step fails, the function cleans up allocated
 * resources and returns an error code.
 *
 * @param device A pointer to a pointer of type struct ad7293_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * The caller is responsible for deallocating this memory using
 * ad7293_remove.
 * @param init_param A pointer to a struct ad7293_init_param containing the
 * initialization parameters for the SPI and optional GPIO
 * reset. Must not be null. The structure should be properly
 * initialized before calling this function.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error (e.g., -ENOMEM for memory allocation
 * failure, -EINVAL for invalid chip ID).
 ******************************************************************************/
int ad7293_init(struct ad7293_dev **device,
		struct ad7293_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * AD7293 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any allocated memory and hardware resources are freed.
 * It is important to call this function to prevent resource leaks in
 * your application.
 *
 * @param dev A pointer to an ad7293_dev structure representing the device to be
 * removed. Must not be null. The function will handle invalid input
 * by returning an error code if the SPI removal fails.
 * @return Returns 0 on success, or a negative error code if the SPI removal
 * fails.
 ******************************************************************************/
int ad7293_remove(struct ad7293_dev *dev);

#endif /* AD7293_H_ */
