/***************************************************************************//**
 *   @file   ad9545.h
 *   @brief  Header file for ad9545 Driver.
 *   @author Jonathan Santos (Jonathan.Santos@analog.com)
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

#ifndef AD9545_H_
#define AD9545_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_util.h"
#include "no_os_delay.h"
#include "no_os_clk.h"
#include "no_os_gpio.h"
#include "no_os_i2c.h"
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Input Driver Mode */
#define DRIVER_MODE_AC_COUPLED_IF	0
#define DRIVER_MODE_DC_COUPLED_1V2	1
#define DRIVER_MODE_DC_COUPLED_1V8	2
#define DRIVER_MODE_IN_PULL_UP		3

/* Input Driver Mode */
#define DRIVER_MODE_AC_COUPLED		0
#define DRIVER_MODE_DC_COUPLED		1
#define DRIVER_MODE_DC_COUPLED_LVDS	2

/* Output Driver Mode */
#define DRIVER_MODE_SINGLE_DIV_DIF	0
#define DRIVER_MODE_SINGLE_DIV		1
#define DRIVER_MODE_DUAL_DIV		2

/* Clock types */
#define AD9545_CLK_OUT			0
#define AD9545_CLK_PLL			1
#define AD9545_CLK_NCO			2
#define AD9545_CLK_AUX_TDC		3

/* PLL addresses */
#define AD9545_PLL0			0
#define AD9545_PLL1			1

/* Outputs addresses */
#define AD9545_Q0A			0
#define AD9545_Q0AA			1
#define AD9545_Q0B			2
#define AD9545_Q0BB			3
#define AD9545_Q0C			4
#define AD9545_Q0CC			5
#define AD9545_Q1A			6
#define AD9545_Q1AA			7
#define AD9545_Q1B			8
#define AD9545_Q1BB			9

/* NCO addresses */
#define AD9545_NCO0			0
#define AD9545_NCO1			1

/* TDC addresses */
#define AD9545_CLK_AUX_TDC0		0
#define AD9545_CLK_AUX_TDC1		1

/* Ex:
 * Output Q0C clock: <&ad9545_clock AD9545_CLK_OUT AD9545_Q0C>;
 * PLL0 clock: <&ad9545_clock AD9545_CLK_PLL AD9545_PLL0>;
 * NCO1 clock: <&ad9545_clock AD9545_CLK_NCO AD9545_NCO1>;
 */
#define BYTE_ADDR_H				NO_OS_GENMASK(14, 8)
#define BYTE_ADDR_L				NO_OS_GENMASK(7, 0)

/*
 * ad9545 registers definition
 */

#define AD9545_CONFIG_0			0x0000
#define AD9545_PRODUCT_ID_LOW		0x0004
#define AD9545_PRODUCT_ID_HIGH		0x0005
#define AD9545_IO_UPDATE		0x000F
#define AD9545_M0_PIN			0x0102
#define AD9545_CHIP_ID			0x0121
#define AD9545_SYS_CLK_FB_DIV		0x0200
#define AD9545_SYS_CLK_INPUT		0x0201
#define AD9545_SYS_CLK_REF_FREQ		0x0202
#define AD9545_STABILITY_TIMER		0x0207
#define AD9545_COMPENSATE_TDCS		0x0280
#define AD9545_COMPENSATE_NCOS		0x0281
#define AD9545_COMPENSATE_DPLL		0x0282
#define AD9545_AUX_DPLL_CHANGE_LIMIT	0x0283
#define AD9545_AUX_DPLL_SOURCE		0x0284
#define AD9545_AUX_DPLL_LOOP_BW		0x0285
#define AD9545_REF_A_CTRL		0x0300
#define AD9545_REF_A_RDIV		0x0400
#define AD9545_REF_A_PERIOD		0x0404
#define AD9545_REF_A_OFFSET_LIMIT	0x040C
#define AD9545_REF_A_MONITOR_HYST	0x040F
#define AD9545_REF_A_VALID_TIMER	0x0410
#define AD9545_PHASE_LOCK_THRESH	0x0800
#define AD9545_PHASE_LOCK_FILL_RATE	0x0803
#define AD9545_PHASE_LOCK_DRAIN_RATE	0x0804
#define AD9545_FREQ_LOCK_THRESH		0x0805
#define AD9545_FREQ_LOCK_FILL_RATE	0x0808
#define AD9545_FREQ_LOCK_DRAIN_RATE	0x0809
#define AD9545_DPLL0_FTW		0x1000
#define AD9545_DPLL0_SLEW_RATE		0x1011
#define AD9545_MODULATION_COUNTER_A0	0x10C2
#define AD9545_MODULATION_COUNTER_B0	0x10C6
#define AD9545_MODULATION_COUNTER_C0	0x10CA
#define AD9545_MODULATOR_A0		0x10CF
#define AD9545_MODULATOR_B0		0x10D0
#define AD9545_MODULATOR_C0		0x10D1
#define	AD9545_NSHOT_REQ_CH0		0x10D3
#define AD9545_NSHOT_EN_AB0		0x10D4
#define AD9545_NSHOT_EN_C0		0x10D5
#define AD9545_DRIVER_0A_CONF		0x10D7
#define AD9545_SYNC_CTRL0		0x10DB
#define AD9545_APLL0_M_DIV		0x1081
#define AD9545_Q0A_DIV			0x1100
#define AD9545_Q0A_PHASE		0x1104
#define AD9545_Q0A_PHASE_CONF		0x1108
#define AD9545_DPLL0_EN			0x1200
#define AD9545_DPLL0_SOURCE		0x1201
#define AD9545_DPLL0_ZERO_DELAY_FB	0x1202
#define AD9545_DPLL0_FB_MODE		0x1203
#define AD9545_DPLL0_LOOP_BW		0x1204
#define AD9545_DPLL0_HITLESS_N		0x1208
#define AD9545_DPLL0_N_DIV		0x120C
#define AD9545_DPLL0_FRAC		0x1210
#define AD9545_DPLL0_MOD		0x1213
#define AD9545_DPLL0_FAST_L1		0x1216
#define AD9545_DPLL0_FAST_L2		0x1217
#define AD9545_MODULATION_COUNTER_A1	0x14C2
#define AD9545_MODULATION_COUNTER_B1	0x14C6
#define AD9545_MODULATOR_A1		0x14CF
#define AD9545_MODULATOR_B1		0x14D0
#define AD9545_NSHOT_EN_AB1		0x14D4
#define AD9545_DRIVER_1A_CONF		0x14D7
#define AD9545_Q1A_DIV			0x1500
#define AD9545_Q1A_PHASE		0x1504
#define AD9545_Q1A_PHASE_CONF		0x1508
#define AD9545_CALIB_CLK		0x2000
#define AD9545_POWER_DOWN_REF		0x2001
#define AD9545_PWR_CALIB_CH0		0x2100
#define AD9545_CTRL_CH0			0x2101
#define AD9545_DIV_OPS_Q0A		0x2102
#define AD9545_DPLL0_MODE		0x2105
#define AD9545_DPLL0_FAST_MODE		0x2106
#define AD9545_DIV_OPS_Q1A		0x2202
#define AD9545_NCO0_CENTER_FREQ		0x2800
#define AD9545_NCO0_OFFSET_FREQ		0x2807
#define AD9545_NCO0_TAG_RATIO		0x280B
#define AD9545_NCO0_TAG_DELTA		0x280D
#define AD9545_NCO0_TYPE_ADJUST		0x280F
#define AD9545_NCO0_DELTA_RATE_LIMIT	0x2810
#define AD9545_NCO0_DELTA_ADJUST	0x2814
#define AD9545_NCO0_CYCLE_ADJUST	0x2819
#define AD9545_TDC0_DIV			0x2A00
#define AD9545_TDC0_PERIOD		0x2A01
#define AD9545_PLL_STATUS		0x3001
#define AD9545_MISC			0x3002
#define AD9545_TEMP0			0x3003
#define AD9545_REFA_STATUS		0x3005
#define AD9545_PLL0_STATUS		0x3100
#define AD9545_PLL0_OPERATION		0x3101

#define AD9545_SYS_CLK_STABILITY_PERIOD_MASK	NO_OS_GENMASK(19, 0)

#define AD9545_REF_CTRL_DIF_MSK			NO_OS_GENMASK(3, 2)
#define AD9545_REF_CTRL_REFA_MSK		NO_OS_GENMASK(5, 4)
#define AD9545_REF_CTRL_REFAA_MSK		NO_OS_GENMASK(7, 6)

#define AD9545_UPDATE_REGS			0x1
#define AD9545_RESET_REGS			0x81

#define AD9545_MX_PIN(x)			(AD9545_M0_PIN + (x))

#define AD9545_SYNC_CTRLX(x)			(AD9545_SYNC_CTRL0 + ((x) * 0x400))
#define AD9545_REF_X_RDIV(x)			(AD9545_REF_A_RDIV + ((x) * 0x20))
#define AD9545_REF_X_PERIOD(x)			(AD9545_REF_A_PERIOD + ((x) * 0x20))
#define AD9545_REF_X_OFFSET_LIMIT(x)		(AD9545_REF_A_OFFSET_LIMIT + ((x) * 0x20))
#define AD9545_REF_X_MONITOR_HYST(x)		(AD9545_REF_A_MONITOR_HYST + ((x) * 0x20))
#define AD9545_REF_X_VALID_TIMER(x)		(AD9545_REF_A_VALID_TIMER + ((x) * 0x20))
#define AD9545_REF_X_PHASE_LOCK_FILL(x)		(AD9545_PHASE_LOCK_FILL_RATE + ((x) * 0x20))
#define AD9545_REF_X_PHASE_LOCK_DRAIN(x)	(AD9545_PHASE_LOCK_DRAIN_RATE + ((x) * 0x20))
#define AD9545_REF_X_FREQ_LOCK_FILL(x)		(AD9545_FREQ_LOCK_FILL_RATE + ((x) * 0x20))
#define AD9545_REF_X_FREQ_LOCK_DRAIN(x)		(AD9545_FREQ_LOCK_DRAIN_RATE + ((x) * 0x20))

#define AD9545_SOURCEX_PHASE_THRESH(x)		(AD9545_PHASE_LOCK_THRESH + ((x) * 0x20))
#define AD9545_SOURCEX_FREQ_THRESH(x)		(AD9545_FREQ_LOCK_THRESH + ((x) * 0x20))
#define AD9545_NCOX_PHASE_THRESH(x)		(AD9545_SOURCEX_PHASE_THRESH((x) + 4))
#define AD9545_NCOX_FREQ_THRESH(x)		(AD9545_SOURCEX_FREQ_THRESH((x) + 4))

#define AD9545_APLLX_M_DIV(x)			(AD9545_APLL0_M_DIV + ((x) * 0x400))

#define AD9545_Q0_DIV(x)			(AD9545_Q0A_DIV + ((x) * 0x9))
#define AD9545_Q1_DIV(x)			(AD9545_Q1A_DIV + ((x) * 0x9))
#define AD9545_QX_DIV(x) ({					\
	typeof(x) x_ = (x);					\
								\
	(x_ > 5) ? AD9545_Q1_DIV(x_ - 6) : AD9545_Q0_DIV(x_);	\
})

#define AD9545_Q0_PHASE(x)			(AD9545_Q0A_PHASE + ((x) * 0x9))
#define AD9545_Q1_PHASE(x)			(AD9545_Q1A_PHASE + ((x) * 0x9))
#define AD9545_QX_PHASE(x) ({						\
	typeof(x) x_ = (x);						\
									\
	(x_ > 5) ? AD9545_Q1_PHASE(x_ - 6) : AD9545_Q0_PHASE(x_);	\
})

#define AD9545_Q0_PHASE_CONF(x)			(AD9545_Q0A_PHASE_CONF + ((x) * 0x9))
#define AD9545_Q1_PHASE_CONF(x)			(AD9545_Q1A_PHASE_CONF + ((x) * 0x9))
#define AD9545_QX_PHASE_CONF(x) ({						\
	typeof(x) x_ = (x);							\
										\
	(x_ > 5) ? AD9545_Q1_PHASE_CONF(x_ - 6) : AD9545_Q0_PHASE_CONF(x_);	\
})

#define AD9545_NSHOT_REQ_CH(x)			(AD9545_NSHOT_REQ_CH0 + ((x) * 0x400))
#define AD9545_DPLLX_FTW(x)			(AD9545_DPLL0_FTW + ((x) * 0x400))
#define AD9545_DPLLX_SLEW_RATE(x)		(AD9545_DPLL0_SLEW_RATE + ((x) * 0x400))
#define AD9545_DPLLX_EN(x, y)			(AD9545_DPLL0_EN + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_SOURCE(x, y)		(AD9545_DPLL0_SOURCE + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FB_PATH(x, y)		(AD9545_DPLL0_ZERO_DELAY_FB + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FB_MODE(x, y)		(AD9545_DPLL0_FB_MODE + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_LOOP_BW(x, y)		(AD9545_DPLL0_LOOP_BW + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_HITLESS_N(x, y)		(AD9545_DPLL0_HITLESS_N + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_N_DIV(x, y)		(AD9545_DPLL0_N_DIV + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FRAC_DIV(x, y)		(AD9545_DPLL0_FRAC + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_MOD_DIV(x, y)		(AD9545_DPLL0_MOD + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FAST_L1(x, y)		(AD9545_DPLL0_FAST_L1 + ((x) * 0x400) + ((y) * 0x20))
#define AD9545_DPLLX_FAST_L2(x, y)		(AD9545_DPLL0_FAST_L2 + ((x) * 0x400) + ((y) * 0x20))

#define AD9545_DIV_OPS_Q0(x)			(AD9545_DIV_OPS_Q0A + (x))
#define AD9545_DIV_OPS_Q1(x)			(AD9545_DIV_OPS_Q1A + (x))
#define AD9545_DIV_OPS_QX(x) ({						\
	typeof(x) x_ = (x) / 2;						\
									\
	(x_ > 2) ? AD9545_DIV_OPS_Q1(x_ - 3) : AD9545_DIV_OPS_Q0(x_);	\
})

#define AD9545_PWR_CALIB_CHX(x)			(AD9545_PWR_CALIB_CH0 + ((x) * 0x100))
#define AD9545_PLLX_STATUS(x)			(AD9545_PLL0_STATUS + ((x) * 0x100))
#define AD9545_PLLX_OPERATION(x)		(AD9545_PLL0_OPERATION + ((x) * 0x100))
#define AD9545_CTRL_CH(x)			(AD9545_CTRL_CH0 + ((x) * 0x100))
#define AD9545_DPLLX_FAST_MODE(x)		(AD9545_DPLL0_FAST_MODE + ((x) * 0x100))
#define AD9545_REFX_STATUS(x)			(AD9545_REFA_STATUS + (x))

#define AD9545_PROFILE_SEL_MODE_MSK		NO_OS_GENMASK(3, 2)
#define AD9545_PROFILE_SEL_MODE(x)		no_os_field_prep(AD9545_PROFILE_SEL_MODE_MSK, x)

#define AD9545_NCOX_CENTER_FREQ(x)		(AD9545_NCO0_CENTER_FREQ + ((x) * 0x40))
#define AD9545_NCOX_OFFSET_FREQ(x)		(AD9545_NCO0_OFFSET_FREQ + ((x) * 0x40))
#define AD9545_NCOX_TAG_RATIO(x)		(AD9545_NCO0_TAG_RATIO + ((x) * 0x40))
#define AD9545_NCOX_TAG_DELTA(x)		(AD9545_NCO0_TAG_DELTA + ((x) * 0x40))
#define AD9545_NCOX_TYPE_ADJUST(x)		(AD9545_NCO0_TYPE_ADJUST + ((x) * 0x40))
#define AD9545_NCOX_DELTA_RATE_LIMIT(x)		(AD9545_NCO0_DELTA_RATE_LIMIT + ((x) * 0x40))
#define AD9545_NCOX_DELTA_ADJUST(x)		(AD9545_NCO0_DELTA_ADJUST + ((x) * 0x40))
#define AD9545_NCOX_CYCLE_ADJUST(x)		(AD9545_NCO0_CYCLE_ADJUST + ((x) * 0x40))

/*
 * AD9545 AUX NCO center frequency register has 16-bit integer part and
 * 40-bit fractional part.
 */
#define AD9545_NCO_CENTER_FREQ_INT_WIDTH	16
#define AD9545_NCO_CENTER_FREQ_FRAC_WIDTH	40
#define AD9545_NCO_CENTER_FREQ_WIDTH		(AD9545_NCO_CENTER_FREQ_INT_WIDTH + \
						 AD9545_NCO_CENTER_FREQ_FRAC_WIDTH)

#define AD9545_NCO_CENTER_FREQ_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_CENTER_FREQ_WIDTH - 1, 0)
#define AD9545_NCO_CENTER_FREQ_INT_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_CENTER_FREQ_WIDTH - 1, \
							    AD9545_NCO_CENTER_FREQ_FRAC_WIDTH)
#define AD9545_NCO_CENTER_FREQ_FRAC_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1, 0)

#define AD9545_NCO_CENTER_FREQ_MAX		no_os_field_max_u64(AD9545_NCO_CENTER_FREQ_MSK)
#define AD9545_NCO_CENTER_FREQ_INT_MAX		no_os_field_max_u64(AD9545_NCO_CENTER_FREQ_INT_MSK)
#define AD9545_NCO_CENTER_FREQ_FRAC_MAX		no_os_field_max_u64(AD9545_NCO_CENTER_FREQ_FRAC_MSK)

/*
 * AD9545 AUX NCO offset frequency register has 8-bit integer part and
 * 24-bit fractional part.
 */
#define AD9545_NCO_OFFSET_FREQ_INT_WIDTH	8
#define AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH	24
#define AD9545_NCO_OFFSET_FREQ_WIDTH		(AD9545_NCO_OFFSET_FREQ_INT_WIDTH + \
						 AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH)

#define AD9545_NCO_OFFSET_FREQ_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_WIDTH - 1, 0)
#define AD9545_NCO_OFFSET_FREQ_INT_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_WIDTH - 1, \
							    AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH)
#define AD9545_NCO_OFFSET_FREQ_FRAC_MSK		NO_OS_GENMASK_ULL(AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH - 1, 0)

#define AD9545_NCO_OFFSET_FREQ_MAX		no_os_field_max(AD9545_NCO_OFFSET_FREQ_MSK)
#define AD9545_NCO_OFFSET_FREQ_INT_MAX		no_os_field_max(AD9545_NCO_OFFSET_FREQ_INT_MSK)
#define AD9545_NCO_OFFSET_FREQ_FRAC_MAX		no_os_field_max(AD9545_NCO_OFFSET_FREQ_FRAC_MSK)

#define AD9545_NCO_FREQ_INT_MAX			(AD9545_NCO_CENTER_FREQ_INT_MAX + \
						 AD9545_NCO_OFFSET_FREQ_INT_MAX)

#define AD9545_TDCX_DIV(x)			(AD9545_TDC0_DIV + ((x) * 0x9))
#define AD9545_TDCX_PERIOD(x)			(AD9545_TDC0_PERIOD + ((x) * 0x9))

/* AD9545 MX PIN bitfields */
#define AD9545_MX_TO_TDCX(x)			(0x30 + (x))

/* AD9545 COMPENSATE TDCS bitfields */
#define AD9545_COMPENSATE_TDCS_VIA_AUX_DPLL	0x4

/* AD9545 COMPENSATE NCOS bitfields */
#define AD9545_COMPENSATE_NCOS_VIA_AUX_DPLL	0x44

/* AD9545 COMPENSATE DPLL bitfields */
#define AD9545_COMPNESATE_VIA_AUX_DPLL		0x44

/* define AD9545_DPLLX_EN bitfields */
#define AD9545_EN_PROFILE_MSK			NO_OS_BIT(0)
#define AD9545_SEL_PRIORITY_MSK			NO_OS_GENMASK(5, 1)

/* define AD9545_DPLLX_FB_MODE bitfields */
#define AD9545_EN_HITLESS_MSK			NO_OS_BIT(0)
#define AD9545_TAG_MODE_MSK			NO_OS_GENMASK(4, 2)
#define AD9545_BASE_FILTER_MSK			NO_OS_BIT(7)

/* AD9545_PWR_CALIB_CHX bitfields */
#define AD9545_PWR_DOWN_CH			NO_OS_BIT(0)
#define AD9545_CALIB_APLL			NO_OS_BIT(1)

/* AD9545_SYNC_CTRLX bitfields */
#define AD9545_SYNC_CTRL_DPLL_REF_MSK		NO_OS_BIT(2)
#define AD9545_SYNC_CTRL_MODE_MSK		NO_OS_GENMASK(1, 0)

/* AD9545_QX_PHASE_CONF bitfields */
#define AD9545_QX_HALF_DIV_MSK			NO_OS_BIT(5)
#define AD9545_QX_PHASE_32_MSK			NO_OS_BIT(6)

/* AD9545_DIV_OPS_QX bitfields */
#define AD9545_DIV_OPS_MUTE_A_MSK		NO_OS_BIT(2)
#define AD9545_DIV_OPS_MUTE_AA_MSK		NO_OS_BIT(3)

/* AD9545 Modulator bitfields */
#define AD9545_MODULATOR_EN			NO_OS_BIT(0)

/* AD9545_NSHOT_REQ_CH bitfields */
#define AD9545_NSHOT_NR_MSK			NO_OS_GENMASK(5, 0)

/* AD9545_CTRL_CH bitfields */
#define AD9545_CTRL_CH_NSHOT_MSK		NO_OS_BIT(0)

/* AD9545_PLL_STATUS bitfields */
#define AD9545_PLLX_LOCK(x, y)			((1 << (4 + (x))) & (y))

/* AD9545_MISC bitfields */
#define AD9545_MISC_AUX_NC0_ERR_MSK		NO_OS_GENMASK(5, 4)
#define AD9545_MISC_AUX_NC1_ERR_MSK		NO_OS_GENMASK(7, 6)
#define AD9545_AUX_DPLL_LOCK_MSK		NO_OS_BIT(1)
#define AD9545_AUX_DPLL_REF_FAULT		NO_OS_BIT(2)

/* AD9545_REFX_STATUS bitfields */
#define AD9545_REFX_SLOW_MSK			NO_OS_BIT(0)
#define AD9545_REFX_FAST_MSK			NO_OS_BIT(1)
#define AD9545_REFX_JITTER_MSK			NO_OS_BIT(2)
#define AD9545_REFX_FAULT_MSK			NO_OS_BIT(3)
#define AD9545_REFX_VALID_MSK			NO_OS_BIT(4)
#define AD9545_REFX_LOS_MSK			NO_OS_BIT(5)

/* AD9545_PLL0_STATUS bitfields */
#define AD9545_PLL_LOCKED			NO_OS_BIT(0)

/* AD9545_PLL0_OPERATION bitfields */
#define AD9545_PLL_FREERUN			NO_OS_BIT(0)
#define AD9545_PLL_HOLDOVER			NO_OS_BIT(1)
#define AD9545_PLL_ACTIVE			NO_OS_BIT(3)
#define AD9545_PLL_ACTIVE_PROFILE		NO_OS_GENMASK(6, 4)

#define AD9545_SYS_PLL_STABLE_MSK		NO_OS_GENMASK(1, 0)
#define AD9545_SYS_PLL_STABLE(x)		(((x) & AD9545_SYS_PLL_STABLE_MSK) == 0x3)

#define AD9545_APLL_LOCKED(x)			((x) & NO_OS_BIT(3))

/*  AD9545 tagging modes */
#define AD9545_NO_TAGGING		0
#define AD9545_FB_PATH_TAG		2

#define AD9545_SYS_CLK_STABILITY_MS	50

#define AD9545_R_DIV_MSK		NO_OS_GENMASK(29, 0)
#define AD9545_R_DIV_MAX		0x40000000
#define AD9545_IN_MAX_TDC_FREQ_HZ	200000

#define AD9545_MAX_REFS			4

#define AD9545_APLL_M_DIV_MIN		1
#define AD9545_APLL_M_DIV_MAX		255

#define AD9545_DPLL_MAX_N		1073741823
#define AD9545_DPLL_MAX_FRAC		16777215
#define AD9545_DPLL_MAX_MOD		16777215
#define AD9545_MAX_DPLL_PROFILES	6

#define AD9545_MAX_NSHOT_PULSES		63

#define AD9545_MAX_ZERO_DELAY_RATE	200000000

#define AD9545_MIN_SYS_CLK_FREQ		2250
#define AD9545_MAX_SYS_CLK_FREQ		2415
#define AD9545_MIN_DIV_RATIO		4
#define AD9545_MAX_DIV_RATIO		256

/***************************************************************************//**
 * @brief The `ad9545_comm_type` is an enumeration that defines the types of
 * communication interfaces available for the AD9545 device, specifically
 * SPI and I2C. This enum is used to specify the communication protocol
 * to be used when interfacing with the AD9545 device, allowing for
 * flexibility in how the device is connected and controlled.
 *
 * @param SPI Represents the Serial Peripheral Interface communication type.
 * @param I2C Represents the Inter-Integrated Circuit communication type.
 ******************************************************************************/
enum ad9545_comm_type {
	SPI,
	I2C,
};

/***************************************************************************//**
 * @brief The `ad9545_apll_rate_ranges_hz` is a static constant 2x2 array of
 * unsigned integers. It defines two ranges of frequencies in hertz for
 * the APLL (Analog Phase-Locked Loop) of the AD9545 device. The first
 * range is from 2,424,000,000 Hz to 3,232,000,000 Hz, and the second
 * range is from 3,232,000,000 Hz to 4,040,000,000 Hz.
 *
 * @details This variable is used to specify the valid frequency ranges for the
 * APLL in the AD9545 device.
 ******************************************************************************/
static const unsigned int ad9545_apll_rate_ranges_hz[2][2] = {
	{2424000000U, 3232000000U}, {3232000000U, 4040000000U}
};

/***************************************************************************//**
 * @brief The `ad9545_apll_pfd_rate_ranges_hz` is a static constant array of two
 * unsigned integers. It defines the range of frequencies in hertz for
 * the phase frequency detector (PFD) of the APLL (Analog Phase-Locked
 * Loop) in the AD9545 device. The values in the array are 162,000,000 Hz
 * and 350,000,000 Hz, representing the minimum and maximum PFD rates,
 * respectively.
 *
 * @details This variable is used to specify the valid frequency range for the
 * APLL PFD in the AD9545 driver.
 ******************************************************************************/
static const unsigned int ad9545_apll_pfd_rate_ranges_hz[2] = {
	162000000U, 350000000U
};

/***************************************************************************//**
 * @brief The `ad9545_vco_calibration_op` is a static constant 2D array of
 * unsigned short integers. It contains a sequence of register operations
 * for calibrating the VCO (Voltage-Controlled Oscillator) in the AD9545
 * device. Each element in the array is a pair of values, where the first
 * value is a register address and the second is the data to be written
 * to that register.
 *
 * @details This variable is used to define a series of operations for VCO
 * calibration in the AD9545 device.
 ******************************************************************************/
static const unsigned short ad9545_vco_calibration_op[][2] = {
	{AD9545_CALIB_CLK, 0},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
	{AD9545_CALIB_CLK, NO_OS_BIT(2)},
	{AD9545_IO_UPDATE, AD9545_UPDATE_REGS},
};

/***************************************************************************//**
 * @brief The `ad9545_tdc_source_mapping` is a static constant array of unsigned
 * 8-bit integers. It contains a predefined set of values that likely
 * correspond to specific TDC (Time-to-Digital Converter) source mappings
 * for the AD9545 device.
 *
 * @details This array is used to map TDC sources to specific indices or
 * configurations within the AD9545 driver.
 ******************************************************************************/
static const uint8_t ad9545_tdc_source_mapping[] = {
	0, 1, 2, 3, 8, 9,
};

/***************************************************************************//**
 * @brief The `ad9545_fast_acq_excess_bw_map` is a constant array of unsigned
 * 32-bit integers. It contains a series of values that represent
 * different bandwidth settings for fast acquisition in the AD9545
 * device.
 *
 * @details This array is used to map different excess bandwidth values for fast
 * acquisition processes in the AD9545 driver.
 ******************************************************************************/
static const uint32_t ad9545_fast_acq_excess_bw_map[] = {
	0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024,
};

/***************************************************************************//**
 * @brief The `ad9545_fast_acq_timeout_map` is a constant array of unsigned
 * 32-bit integers that defines a set of timeout values for fast
 * acquisition processes in the AD9545 driver. The array contains
 * specific timeout durations in milliseconds, ranging from 1 ms to
 * 50,000 ms. These values are likely used to configure or control the
 * timing behavior of the fast acquisition feature in the AD9545 device.
 *
 * @details This variable is used to map specific timeout durations for fast
 * acquisition processes in the AD9545 driver.
 ******************************************************************************/
static const uint32_t ad9545_fast_acq_timeout_map[] = {
	1, 10, 50, 100, 500, 1000, 10000, 50000,
};

/***************************************************************************//**
 * @brief The `ad9545_hyst_scales_bp` is a static constant array of unsigned
 * 32-bit integers. It contains a series of breakpoint values used for
 * hysteresis scaling in the AD9545 driver. The values in the array are
 * 0, 3125, 6250, 12500, 25000, 50000, 75000, and 87500.
 *
 * @details This array is used to define specific breakpoint values for
 * hysteresis scaling in the AD9545 device operations.
 ******************************************************************************/
static const uint32_t ad9545_hyst_scales_bp[] = {
	0, 3125, 6250, 12500, 25000, 50000, 75000, 87500
};

/***************************************************************************//**
 * @brief The `ad9545_out_source_ua` is a static constant array of unsigned
 * 32-bit integers. It contains three elements with values 7500, 12500,
 * and 15000, which likely represent current source values in
 * microamperes for the AD9545 device outputs.
 *
 * @details This array is used to define specific current source values for
 * configuring the output of the AD9545 device.
 ******************************************************************************/
static const uint32_t ad9545_out_source_ua[] = {
	7500, 12500, 15000
};

/***************************************************************************//**
 * @brief The `ad9545_rate_change_limit_map` is a static constant array of
 * unsigned 32-bit integers. It contains a series of predefined rate
 * change limits used in the AD9545 driver.
 *
 * @details This array is used to define the permissible rate change limits for
 * the AD9545 device, likely to ensure stability and performance during
 * clock rate adjustments.
 ******************************************************************************/
static const uint32_t ad9545_rate_change_limit_map[] = {
	715, 1430, 2860, 5720, 11440, 22880, 45760,
};

/***************************************************************************//**
 * @brief The `ad9545_ref_m_clk_names` is a static constant array of string
 * literals representing the names of reference clocks for the AD9545
 * device. It contains three elements: "Ref-M0", "Ref-M1", and "Ref-M2".
 *
 * @details This array is used to identify and reference the different reference
 * clocks available in the AD9545 device.
 ******************************************************************************/
static const char * const ad9545_ref_m_clk_names[] = {
	"Ref-M0", "Ref-M1", "Ref-M2",
};

/***************************************************************************//**
 * @brief The `ad9545_ref_clk_names` is a static constant array of string
 * literals representing the names of reference clocks for the AD9545
 * device. It contains four elements: "Ref-A", "Ref-AA", "Ref-B", and
 * "Ref-BB".
 *
 * @details This array is used to identify and reference the different reference
 * clock inputs available on the AD9545 device.
 ******************************************************************************/
static const char * const ad9545_ref_clk_names[] = {
	"Ref-A", "Ref-AA", "Ref-B", "Ref-BB",
};

/***************************************************************************//**
 * @brief The `ad9545_in_clk_names` is a static constant array of string
 * literals representing the names of input clock dividers for the AD9545
 * device. It contains four elements: "Ref-A-Div", "Ref-AA-Div", "Ref-B-
 * Div", and "Ref-BB-Div". These names are likely used to identify
 * different input clock sources or configurations within the AD9545
 * driver.
 *
 * @details This variable is used to provide human-readable names for input
 * clock dividers in the AD9545 driver, facilitating easier reference
 * and debugging.
 ******************************************************************************/
static const char * const ad9545_in_clk_names[] = {
	"Ref-A-Div", "Ref-AA-Div", "Ref-B-Div", "Ref-BB-Div",
};

/***************************************************************************//**
 * @brief The `ad9545_out_clk_names` is a static constant array of string
 * literals representing the names of output clock dividers for the
 * AD9545 device. It includes names like "Q0A-div", "Q0AA-div",
 * "Q0B-div", etc., which correspond to different output clock channels
 * of the device. This array is used to provide human-readable
 * identifiers for the various output clocks available in the AD9545
 * clock generator.
 *
 * @details This variable is used to map output clock channels to their
 * respective names for easier identification and reference in the
 * AD9545 driver.
 ******************************************************************************/
static const char * const ad9545_out_clk_names[] = {
	"Q0A-div", "Q0AA-div", "Q0B-div", "Q0BB-div", "Q0C-div", "Q0CC-div", "Q1A-div", "Q1AA-div",
	"Q1B-div", "Q1BB-div",
};

/***************************************************************************//**
 * @brief The `ad9545_pll_clk_names` is a static constant array of string
 * literals that contains the names of the PLL clocks used in the AD9545
 * driver. It includes two elements: "PLL0" and "PLL1".
 *
 * @details This array is used to reference the names of the PLL clocks within
 * the AD9545 driver, likely for identification or configuration
 * purposes.
 ******************************************************************************/
static const char * const ad9545_pll_clk_names[] = {
	"PLL0", "PLL1",
};

/***************************************************************************//**
 * @brief The `ad9545_aux_nco_clk_names` is a static constant array of string
 * literals, each representing the name of an auxiliary Numerically
 * Controlled Oscillator (NCO) clock in the AD9545 device. It contains
 * two elements: "AUX_NCO0" and "AUX_NCO1".
 *
 * @details This array is used to reference the names of auxiliary NCO clocks
 * within the AD9545 driver, likely for configuration or identification
 * purposes.
 ******************************************************************************/
static const char * const ad9545_aux_nco_clk_names[] = {
	"AUX_NCO0", "AUX_NCO1",
};

/***************************************************************************//**
 * @brief The `ad9545_aux_tdc_clk_names` is a static constant array of string
 * literals representing the names of auxiliary TDC (Time-to-Digital
 * Converter) clocks in the AD9545 device. It contains two elements:
 * "AUX_TDC0" and "AUX_TDC1".
 *
 * @details This array is used to reference the auxiliary TDC clock names within
 * the AD9545 driver code.
 ******************************************************************************/
static const char * const ad9545_aux_tdc_clk_names[] = {
	"AUX_TDC0", "AUX_TDC1",
};

/***************************************************************************//**
 * @brief The `ad9545_ref_mode` is an enumeration that defines the types of
 * reference modes available for the AD9545 device. It includes two
 * modes: `AD9545_SINGLE_ENDED`, which is used for single-ended reference
 * configurations, and `AD9545_DIFFERENTIAL`, which is used for
 * differential reference configurations. This enumeration is used to
 * specify the electrical configuration of the reference input to the
 * device.
 *
 * @param AD9545_SINGLE_ENDED Represents a single-ended reference mode with a
 * value of 0.
 * @param AD9545_DIFFERENTIAL Represents a differential reference mode.
 ******************************************************************************/
enum ad9545_ref_mode {
	AD9545_SINGLE_ENDED = 0,
	AD9545_DIFFERENTIAL,
};

/***************************************************************************//**
 * @brief The `ad9545_single_ended_config` enumeration defines various
 * configurations for single-ended input modes in the AD9545 device. It
 * includes options for AC coupling, DC coupling at different voltage
 * levels (1.2V and 1.8V), and an input pull-up configuration. This
 * enumeration is used to specify the electrical characteristics of the
 * input interface for the AD9545 clock generation and distribution
 * system.
 *
 * @param AD9545_AC_COUPLED_IF Represents an AC coupled interface configuration.
 * @param AD9545_DC_COUPLED_1V2 Represents a DC coupled configuration with 1.2V.
 * @param AD9545_DC_COUPLED_1V8 Represents a DC coupled configuration with 1.8V.
 * @param AD9545_IN_PULL_UP Represents a configuration with an input pull-up.
 ******************************************************************************/
enum ad9545_single_ended_config {
	AD9545_AC_COUPLED_IF = 0,
	AD9545_DC_COUPLED_1V2,
	AD9545_DC_COUPLED_1V8,
	AD9545_IN_PULL_UP,
};

/***************************************************************************//**
 * @brief The `ad9545_diferential_config` is an enumeration that defines
 * different types of differential configurations for the AD9545 device.
 * It includes options for AC coupling, DC coupling, and DC coupling with
 * LVDS, allowing the user to specify the desired electrical coupling
 * method for the differential inputs of the device.
 *
 * @param AD9545_AC_COUPLED Represents an AC coupled differential configuration.
 * @param AD9545_DC_COUPLED Represents a DC coupled differential configuration.
 * @param AD9545_DC_COUPLED_LVDS Represents a DC coupled LVDS differential
 * configuration.
 ******************************************************************************/
enum ad9545_diferential_config {
	AD9545_AC_COUPLED = 0,
	AD9545_DC_COUPLED,
	AD9545_DC_COUPLED_LVDS,
};

/***************************************************************************//**
 * @brief The `ad9545_output_mode` enumeration defines the different output
 * modes available for the AD9545 device, which is a clock generator and
 * distribution IC. Each enumerator represents a specific configuration
 * of the output dividers, allowing the user to select between single or
 * dual divider modes, with or without differential signaling. This
 * configuration is crucial for setting up the output characteristics of
 * the device to match the requirements of the application.
 *
 * @param AD9545_SINGLE_DIV_DIF Represents a single divider differential output
 * mode.
 * @param AD9545_SINGLE_DIV Represents a single divider output mode.
 * @param AD9545_DUAL_DIV Represents a dual divider output mode.
 ******************************************************************************/
enum ad9545_output_mode {
	AD9545_SINGLE_DIV_DIF = 0,
	AD9545_SINGLE_DIV,
	AD9545_DUAL_DIV,
};

/***************************************************************************//**
 * @brief The `ad9545_outputs_regs` structure is used to define the register
 * settings for output modulation in the AD9545 device. It includes
 * registers for configuring the modulator, setting the modulation
 * counter, and enabling n-shot functionality, along with a mask for the
 * n-shot enable register. This structure is essential for managing the
 * output characteristics of the AD9545 clock generator.
 *
 * @param modulator_reg A 16-bit register for configuring the modulator.
 * @param modulation_counter_reg A 16-bit register for setting the modulation
 * counter.
 * @param nshot_en_reg A 16-bit register for enabling n-shot functionality.
 * @param nshot_en_msk An 8-bit mask for n-shot enable register.
 ******************************************************************************/
struct ad9545_outputs_regs {
	uint16_t	modulator_reg;
	uint16_t	modulation_counter_reg;
	uint16_t	nshot_en_reg;
	uint8_t	nshot_en_msk;
};

/***************************************************************************//**
 * @brief The `ad9545_out_regs` is a static constant array of structures, each
 * of type `ad9545_outputs_regs`. This array contains configurations for
 * the output registers of the AD9545 device, specifying modulator
 * registers, modulation counter registers, n-shot enable registers, and
 * n-shot enable masks for different output channels.
 *
 * @details This variable is used to define the register settings for various
 * output channels of the AD9545 device, facilitating the configuration
 * of its output modulation and n-shot functionalities.
 ******************************************************************************/
static const struct ad9545_outputs_regs ad9545_out_regs[] = {
	{
		.modulator_reg = AD9545_MODULATOR_A0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = NO_OS_BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = NO_OS_BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = NO_OS_BIT(4),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B0,
		.nshot_en_reg = AD9545_NSHOT_EN_AB0,
		.nshot_en_msk = NO_OS_BIT(6),
	},
	{
		.modulator_reg = AD9545_MODULATOR_C0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_C0,
		.nshot_en_reg = AD9545_NSHOT_EN_C0,
		.nshot_en_msk = NO_OS_BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_C0,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_C0,
		.nshot_en_reg = AD9545_NSHOT_EN_C0,
		.nshot_en_msk = NO_OS_BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = NO_OS_BIT(0),
	},
	{
		.modulator_reg = AD9545_MODULATOR_A1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_A1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = NO_OS_BIT(2),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = NO_OS_BIT(4),
	},
	{
		.modulator_reg = AD9545_MODULATOR_B1,
		.modulation_counter_reg = AD9545_MODULATION_COUNTER_B1,
		.nshot_en_reg = AD9545_NSHOT_EN_AB1,
		.nshot_en_msk = NO_OS_BIT(6),
	},
};

/***************************************************************************//**
 * @brief The `ad9545_out_clk` structure is designed to manage the configuration
 * and state of an output clock in the AD9545 device. It includes
 * pointers to the device context and hardware clock descriptors, as well
 * as fields for managing the output's usage status, current source,
 * mode, and requested rate. This structure is essential for configuring
 * and controlling the output clocks of the AD9545, allowing for precise
 * clock management and customization.
 *
 * @param dev Pointer to the ad9545_dev structure, representing the device
 * context.
 * @param output_used Boolean indicating if the output is currently in use.
 * @param source_current Boolean indicating if the source current is active.
 * @param output_mode Enum specifying the output mode of the clock.
 * @param source_ua Unsigned 32-bit integer representing the source current in
 * microamperes.
 * @param hw Pointer to a no_os_clk_desc structure, representing the hardware
 * clock descriptor.
 * @param address Unsigned integer representing the address of the output clock.
 * @param rate_requested_hz Unsigned 64-bit integer representing the requested
 * clock rate in hertz.
 * @param parent_clk Pointer to a no_os_clk_desc structure, representing the
 * parent clock descriptor.
 ******************************************************************************/
struct ad9545_out_clk {
	struct ad9545_dev	*dev;
	bool			output_used;
	bool			source_current;
	enum ad9545_output_mode	output_mode;
	uint32_t		source_ua;
	struct no_os_clk_desc	*hw;
	unsigned int		address;
	uint64_t		rate_requested_hz;
	struct no_os_clk_desc	*parent_clk;
};

/***************************************************************************//**
 * @brief The `ad9545_dpll_profile` structure defines a profile for a digital
 * phase-locked loop (DPLL) in the AD9545 device. It includes
 * configuration parameters such as address, parent clock index,
 * priority, loop bandwidth, and settings for fast acquisition. The
 * structure also includes flags for enabling the profile and feedback
 * tagging, as well as specifying the source for the time-to-digital
 * converter. This structure is essential for managing the DPLL
 * configurations in the AD9545 clock generation and distribution system.
 *
 * @param address The address of the DPLL profile.
 * @param parent_index The index of the parent clock source.
 * @param priority The priority level of the DPLL profile.
 * @param loop_bw_uhz The loop bandwidth in microhertz.
 * @param fast_acq_excess_bw The excess bandwidth for fast acquisition.
 * @param fast_acq_timeout_ms The timeout for fast acquisition in milliseconds.
 * @param fast_acq_settle_ms The settling time for fast acquisition in
 * milliseconds.
 * @param en A boolean indicating if the DPLL profile is enabled.
 * @param tdc_source The source for the time-to-digital converter.
 * @param fb_tagging A boolean indicating if feedback tagging is enabled.
 ******************************************************************************/
struct ad9545_dpll_profile {
	unsigned int	address;
	unsigned int	parent_index;
	unsigned int	priority;
	unsigned int	loop_bw_uhz;
	unsigned int	fast_acq_excess_bw;
	unsigned int	fast_acq_timeout_ms;
	unsigned int	fast_acq_settle_ms;
	bool		en;
	uint8_t		tdc_source;
	bool		fb_tagging;
};

/***************************************************************************//**
 * @brief The `ad9545_pll_clk` structure represents a phase-locked loop (PLL)
 * clock configuration for the AD9545 device. It includes information
 * about the device, usage status, address, hardware descriptor, parent
 * clocks, DPLL profiles, and various operational parameters such as
 * free-run frequency, fast acquisition trigger mode, requested rate, and
 * slew rate limit. This structure is essential for managing PLL settings
 * and behavior in the AD9545 clock generation system.
 *
 * @param dev Pointer to the ad9545 device structure.
 * @param pll_used Boolean indicating if the PLL is used.
 * @param address Address of the PLL.
 * @param hw Pointer to the hardware clock descriptor.
 * @param num_parents Number of parent clocks.
 * @param parents Array of pointers to parent clock descriptors.
 * @param profiles Array of DPLL profiles for the PLL.
 * @param free_run_freq Frequency for free-run mode.
 * @param fast_acq_trigger_mode Mode for fast acquisition trigger.
 * @param rate_requested_hz Requested rate in hertz.
 * @param internal_zero_delay Boolean indicating if internal zero delay is used.
 * @param internal_zero_delay_source Source for internal zero delay.
 * @param internal_zero_delay_source_rate_hz Rate of the internal zero delay
 * source in hertz.
 * @param slew_rate_limit_ps Slew rate limit in picoseconds.
 ******************************************************************************/
struct ad9545_pll_clk {
	struct ad9545_dev		*dev;
	bool				pll_used;
	unsigned int			address;
	struct no_os_clk_desc		*hw;
	uint8_t				num_parents;
	struct no_os_clk_desc		**parents;
	struct ad9545_dpll_profile	profiles[AD9545_MAX_DPLL_PROFILES];
	unsigned int			free_run_freq;
	unsigned int			fast_acq_trigger_mode;
	uint64_t			rate_requested_hz;
	bool				internal_zero_delay;
	uint8_t				internal_zero_delay_source;
	uint64_t			internal_zero_delay_source_rate_hz;
	uint32_t			slew_rate_limit_ps;
};

/***************************************************************************//**
 * @brief The `ad9545_ref_in_clk` structure is designed to manage the
 * configuration and state of a reference input clock for the AD9545
 * device. It includes pointers to hardware and device descriptors,
 * configuration parameters such as divider ratios and tolerance, and
 * settings for monitoring and validation. The structure supports both
 * single-ended and differential configurations, with specific fields for
 * managing phase and frequency lock parameters. This structure is
 * essential for setting up and maintaining the stability and accuracy of
 * the reference clock input in the AD9545 system.
 *
 * @param hw Pointer to a hardware clock descriptor.
 * @param dev Pointer to the AD9545 device structure.
 * @param r_div_ratio Reference divider ratio for the clock.
 * @param ref_used Boolean indicating if the reference is used.
 * @param d_tol_ppb Tolerance in parts per billion for the reference clock.
 * @param monitor_hyst_scale Scale for monitoring hysteresis.
 * @param valid_t_ms Time in milliseconds for which the reference is considered
 * valid.
 * @param parent_clk Pointer to the parent clock descriptor.
 * @param address Address of the reference input clock.
 * @param mode Mode of the reference input clock, either single-ended or
 * differential.
 * @param freq_thresh_ps Frequency threshold in picoseconds.
 * @param phase_thresh_ps Phase threshold in picoseconds.
 * @param phase_lock_fill_rate Rate at which phase lock is filled.
 * @param phase_lock_drain_rate Rate at which phase lock is drained.
 * @param freq_lock_fill_rate Rate at which frequency lock is filled.
 * @param freq_lock_drain_rate Rate at which frequency lock is drained.
 * @param s_conf Configuration for single-ended mode.
 * @param d_conf Configuration for differential mode.
 ******************************************************************************/
struct ad9545_ref_in_clk {
	struct no_os_clk_desc		*hw;
	struct ad9545_dev		*dev;
	uint32_t			r_div_ratio;
	bool				ref_used;
	uint32_t			d_tol_ppb;
	uint8_t				monitor_hyst_scale;
	uint32_t			valid_t_ms;
	struct no_os_clk_desc		*parent_clk;
	unsigned int			address;
	enum ad9545_ref_mode		mode;
	unsigned int			freq_thresh_ps;
	unsigned int			phase_thresh_ps;
	unsigned int			phase_lock_fill_rate;
	unsigned int			phase_lock_drain_rate;
	unsigned int			freq_lock_fill_rate;
	unsigned int			freq_lock_drain_rate;
	union {
		enum ad9545_single_ended_config		s_conf;
		enum ad9545_diferential_config		d_conf;
	};
};

/***************************************************************************//**
 * @brief The `ad9545_aux_nco_clk` structure is designed to manage auxiliary
 * Numerically Controlled Oscillator (NCO) clocks within the AD9545
 * device. It includes a hardware clock descriptor, a flag to indicate if
 * the NCO is active, a reference to the AD9545 device, and configuration
 * parameters such as address, frequency threshold, and phase threshold,
 * which are crucial for precise clock management and synchronization
 * tasks.
 *
 * @param hw Pointer to a clock descriptor for hardware clock management.
 * @param nco_used Boolean flag indicating if the NCO is in use.
 * @param dev Pointer to the AD9545 device structure.
 * @param address Unsigned integer representing the address of the NCO.
 * @param freq_thresh_ps Unsigned integer for frequency threshold in
 * picoseconds.
 * @param phase_thresh_ps Unsigned integer for phase threshold in picoseconds.
 ******************************************************************************/
struct ad9545_aux_nco_clk {
	struct no_os_clk_desc		*hw;
	bool				nco_used;
	struct ad9545_dev		*dev;
	unsigned int			address;
	unsigned int			freq_thresh_ps;
	unsigned int			phase_thresh_ps;
};

/***************************************************************************//**
 * @brief The `ad9545_aux_tdc_clk` structure is designed to represent an
 * auxiliary Time-to-Digital Converter (TDC) clock within the AD9545
 * device. It includes pointers to the hardware clock descriptor and the
 * AD9545 device, as well as configuration details such as the TDC's
 * address and associated pin number. The structure also tracks whether
 * the TDC is actively used and maintains a reference to its parent
 * clock, facilitating the management and configuration of TDC-related
 * operations in the AD9545 clocking system.
 *
 * @param hw Pointer to a hardware clock descriptor.
 * @param tdc_used Boolean indicating if the TDC is used.
 * @param dev Pointer to the AD9545 device structure.
 * @param address Unsigned integer representing the address of the TDC.
 * @param pin_nr Unsigned integer representing the pin number associated with
 * the TDC.
 * @param parent_clk Pointer to the parent clock descriptor.
 ******************************************************************************/
struct ad9545_aux_tdc_clk {
	struct no_os_clk_desc		*hw;
	bool				tdc_used;
	struct ad9545_dev		*dev;
	unsigned int			address;
	unsigned int			pin_nr;
	struct no_os_clk_desc		*parent_clk;
};

/***************************************************************************//**
 * @brief The `ad9545_aux_dpll_clk` structure is designed to manage auxiliary
 * DPLL (Digital Phase-Locked Loop) clock settings within the AD9545
 * device. It includes pointers to clock descriptors for both the
 * hardware and parent clocks, a boolean to indicate DPLL usage, and
 * several configuration parameters such as the source, loop bandwidth,
 * and rate change limit. This structure is integral for configuring and
 * controlling the auxiliary DPLL functionality in the AD9545 clock
 * generation system.
 *
 * @param hw Pointer to a clock descriptor for hardware clock management.
 * @param dpll_used Boolean indicating if the DPLL is used.
 * @param dev Pointer to the AD9545 device structure.
 * @param source Unsigned integer representing the source of the DPLL.
 * @param loop_bw_mhz Unsigned integer specifying the loop bandwidth in MHz.
 * @param rate_change_limit Unsigned integer defining the rate change limit.
 * @param parent_clk Pointer to the parent clock descriptor.
 ******************************************************************************/
struct ad9545_aux_dpll_clk {
	struct no_os_clk_desc		*hw;
	bool				dpll_used;
	struct ad9545_dev		*dev;
	unsigned int			source;
	unsigned int			loop_bw_mhz;
	unsigned int			rate_change_limit;
	struct no_os_clk_desc		*parent_clk;
};

/***************************************************************************//**
 * @brief The `ad9545_sys_clk` structure is used to define the configuration of
 * the system clock for the AD9545 device. It includes settings for
 * enabling a frequency doubler, specifying the use of a crystal
 * oscillator, and storing the reference and system frequencies in hertz.
 * This structure is essential for configuring the clocking system of the
 * AD9545, which is a high-performance clock generator and distribution
 * device.
 *
 * @param sys_clk_freq_doubler Indicates if the system clock frequency doubler
 * is enabled.
 * @param sys_clk_crystal Specifies if a crystal is used for the system clock.
 * @param ref_freq_hz Holds the reference frequency in hertz.
 * @param sys_freq_hz Stores the system frequency in hertz.
 ******************************************************************************/
struct ad9545_sys_clk {
	bool				sys_clk_freq_doubler;
	bool				sys_clk_crystal;
	uint32_t			ref_freq_hz;
	uint32_t			sys_freq_hz;
};

struct ad9545_dev;

typedef int32_t (*ad9545_reg_read_func)(struct ad9545_dev *dev,
					uint16_t reg_addr,
					uint8_t *reg_data);
typedef int32_t (*ad9545_reg_write_func)(struct ad9545_dev *dev,
		uint16_t reg_addr,
		uint8_t reg_data);
typedef int32_t (*ad9545_reg_read_multi_func)(struct ad9545_dev *dev,
		uint16_t reg_addr,
		uint8_t *reg_data,
		uint16_t count);
typedef int32_t (*ad9545_reg_write_multi_func)(struct ad9545_dev *dev,
		uint16_t reg_addr,
		uint8_t *reg_data,
		uint16_t count);

/***************************************************************************//**
 * @brief The `ad9545_dev` structure is a comprehensive data structure used to
 * manage and configure the AD9545 device, which is a high-performance
 * clock generator. It includes descriptors for both SPI and I2C
 * communication interfaces, function pointers for register read and
 * write operations, and various clock configuration structures. The
 * structure supports multiple clock types, including system, auxiliary
 * DPLL, PLL, reference input, output, NCO, and TDC clocks, allowing for
 * extensive customization and control of the device's clocking
 * capabilities. The communication type is specified by an enumeration,
 * and the structure is designed to facilitate the initialization,
 * configuration, and management of the AD9545 device in a flexible and
 * efficient manner.
 *
 * @param spi_desc Pointer to the SPI descriptor for SPI communication.
 * @param i2c_desc Pointer to the I2C descriptor for I2C communication.
 * @param reg_read Function pointer for reading a single register.
 * @param reg_write Function pointer for writing to a single register.
 * @param reg_read_multiple Function pointer for reading multiple registers.
 * @param reg_write_multiple Function pointer for writing to multiple registers.
 * @param comm_type Enumeration indicating the communication type (SPI or I2C).
 * @param sys_clk Structure containing system clock settings.
 * @param aux_dpll_clk Structure containing auxiliary DPLL clock settings.
 * @param pll_clks Array of structures for PLL clock settings.
 * @param ref_in_clks Array of structures for reference input clock settings.
 * @param out_clks Array of structures for output clock settings.
 * @param aux_nco_clks Array of structures for auxiliary NCO clock settings.
 * @param aux_tdc_clks Array of structures for auxiliary TDC clock settings.
 * @param clks Array of pointers to clock descriptors.
 ******************************************************************************/
struct ad9545_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* I2C */
	struct no_os_i2c_desc		*i2c_desc;
	/* Device Settings */
	ad9545_reg_read_func		reg_read;
	ad9545_reg_write_func		reg_write;
	ad9545_reg_read_multi_func	reg_read_multiple;
	ad9545_reg_write_multi_func	reg_write_multiple;
	/* Device Settings */
	enum ad9545_comm_type		comm_type;
	struct ad9545_sys_clk		sys_clk;
	struct ad9545_aux_dpll_clk	aux_dpll_clk;
	struct ad9545_pll_clk		pll_clks[NO_OS_ARRAY_SIZE(ad9545_pll_clk_names)];
	struct ad9545_ref_in_clk	ref_in_clks[NO_OS_ARRAY_SIZE(ad9545_ref_clk_names)];
	struct ad9545_out_clk		out_clks[NO_OS_ARRAY_SIZE(ad9545_out_clk_names)];
	struct ad9545_aux_nco_clk	aux_nco_clks[NO_OS_ARRAY_SIZE(
				ad9545_aux_nco_clk_names)];
	struct ad9545_aux_tdc_clk	aux_tdc_clks[NO_OS_ARRAY_SIZE(
				ad9545_aux_tdc_clk_names)];
	/* CLK descriptors */
	struct no_os_clk_desc		**clks[4];

};

/***************************************************************************//**
 * @brief The `ad9545_init_param` structure is used to initialize the AD9545
 * device, encapsulating all necessary parameters for setting up
 * communication interfaces (SPI or I2C), system clock, auxiliary DPLL,
 * PLL clocks, reference input clocks, output clocks, auxiliary NCO
 * clocks, and auxiliary TDC clocks. This structure provides a
 * comprehensive configuration setup for the AD9545, allowing for
 * detailed customization of its clocking and communication
 * functionalities.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param i2c_init Pointer to I2C initialization parameters.
 * @param comm_type Communication type, either SPI or I2C.
 * @param sys_clk System clock configuration parameters.
 * @param aux_dpll_clk Auxiliary DPLL clock configuration parameters.
 * @param pll_clks Array of PLL clock configuration parameters.
 * @param ref_in_clks Array of reference input clock configuration parameters.
 * @param out_clks Array of output clock configuration parameters.
 * @param aux_nco_clks Array of auxiliary NCO clock configuration parameters.
 * @param aux_tdc_clks Array of auxiliary TDC clock configuration parameters.
 ******************************************************************************/
struct ad9545_init_param {
	/* SPI */
	struct no_os_spi_init_param	*spi_init;
	/* I2C */
	struct no_os_i2c_init_param	*i2c_init;
	/* Device Settings */
	enum ad9545_comm_type		comm_type;
	struct ad9545_sys_clk		sys_clk;
	struct ad9545_aux_dpll_clk	aux_dpll_clk;
	struct ad9545_pll_clk		pll_clks[NO_OS_ARRAY_SIZE(ad9545_pll_clk_names)];
	struct ad9545_ref_in_clk	ref_in_clks[NO_OS_ARRAY_SIZE(ad9545_ref_clk_names)];
	struct ad9545_out_clk		out_clks[NO_OS_ARRAY_SIZE(ad9545_out_clk_names)];
	struct ad9545_aux_nco_clk	aux_nco_clks[NO_OS_ARRAY_SIZE(
				ad9545_aux_nco_clk_names)];
	struct ad9545_aux_tdc_clk	aux_tdc_clks[NO_OS_ARRAY_SIZE(
				ad9545_aux_tdc_clk_names)];

};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Reads a single byte from the specified SPI register address */
/***************************************************************************//**
 * @brief Use this function to read a single byte of data from a specific
 * register address on the AD9545 device via SPI communication. This
 * function is essential for retrieving configuration or status
 * information from the device. Ensure that the device has been properly
 * initialized and configured for SPI communication before calling this
 * function. The function will return an error code if the read operation
 * fails, which should be handled appropriately by the caller.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address for the AD9545 device.
 * @param reg_data A pointer to a uint8_t variable where the read byte will be
 * stored. Must not be null.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-negative value indicates success, while a negative
 * value indicates an error.
 ******************************************************************************/
int32_t ad9545_spi_reg_read(struct ad9545_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data);

/* Reads multiple bytes starting from the specified SPI register address */
/***************************************************************************//**
 * @brief Use this function to read a sequence of bytes from the AD9545 device
 * starting at a specified SPI register address. It is essential to
 * ensure that the `count` parameter does not exceed 512, as this is the
 * maximum number of bytes that can be read in a single operation. The
 * function requires a valid device structure and a buffer to store the
 * read data. It returns an error code if the operation fails or if the
 * `count` exceeds the limit.
 *
 * @param dev A pointer to an initialized `ad9545_dev` structure representing
 * the device. Must not be null.
 * @param reg_addr The starting register address from which to begin reading. It
 * is a 16-bit unsigned integer.
 * @param reg_data A pointer to a buffer where the read data will be stored. The
 * buffer must be large enough to hold `count` bytes. Must not
 * be null.
 * @param count The number of bytes to read from the device. Must be between 1
 * and 512 inclusive. If greater than 512, the function returns an
 * error.
 * @return Returns 0 on success or a negative error code on failure, such as if
 * `count` exceeds 512.
 ******************************************************************************/
int32_t ad9545_spi_reg_read_multiple(struct ad9545_dev *dev,
				     uint16_t reg_addr,
				     uint8_t *reg_data,
				     uint16_t count);
/* Writes a single byte to the specified SPI register address */
/***************************************************************************//**
 * @brief Use this function to write a single byte of data to a specific
 * register address on the AD9545 device via SPI communication. This
 * function is typically called when you need to configure or update a
 * register on the device. Ensure that the device has been properly
 * initialized and that the SPI communication interface is correctly set
 * up before calling this function. The function returns an error code if
 * the write operation fails.
 *
 * @param dev A pointer to an ad9545_dev structure representing the device. Must
 * not be null and should be properly initialized with SPI settings.
 * @param reg_addr The 16-bit address of the register to which the data will be
 * written. Must be a valid register address for the AD9545
 * device.
 * @param reg_data The 8-bit data to be written to the specified register
 * address. Any value within the range of an 8-bit unsigned
 * integer is valid.
 * @return Returns an int32_t error code indicating the success or failure of
 * the SPI write operation.
 ******************************************************************************/
int32_t ad9545_spi_reg_write(struct ad9545_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data);
/* Writes multiple bytes starting from the specified SPI register address */
/***************************************************************************//**
 * @brief This function is used to write a sequence of bytes to a specified
 * register address on the AD9545 device via SPI communication. It is
 * essential to ensure that the device is properly initialized and
 * configured for SPI communication before calling this function. The
 * function handles up to 512 bytes of data, and any attempt to write
 * more than this will result in an error. This function is typically
 * used when multiple consecutive register writes are needed, such as
 * during device configuration or data transfer operations.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The starting register address where the data will be written.
 * It is a 16-bit unsigned integer.
 * @param reg_data A pointer to the data buffer containing the bytes to be
 * written. The buffer must contain at least 'count' bytes. Must
 * not be null.
 * @param count The number of bytes to write. Must be between 1 and 512
 * inclusive. If greater than 512, the function returns an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * when 'count' exceeds 512.
 ******************************************************************************/
int32_t ad9545_spi_reg_write_multiple(struct ad9545_dev *dev,
				      uint16_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count);
/* Reads a single byte from the specified I2C register address */
/***************************************************************************//**
 * @brief Use this function to read a single byte from a specific register of
 * the AD9545 device over the I2C interface. This function is typically
 * called when you need to retrieve configuration or status information
 * from the device. Ensure that the device has been properly initialized
 * and that the I2C communication is correctly set up before calling this
 * function. The function will return an error code if the read operation
 * fails.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The 16-bit address of the register to read from. Must be a
 * valid register address for the AD9545 device.
 * @param reg_data A pointer to a uint8_t variable where the read byte will be
 * stored. Must not be null.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the read operation fails.
 ******************************************************************************/
int32_t ad9545_i2c_reg_read(struct ad9545_dev *dev,
			    uint16_t reg_addr,
			    uint8_t *reg_data);
/* Writes a single byte to the specified I2C register address */
/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register address on the AD9545 device via the I2C interface. It is
 * typically called when there is a need to configure or update a
 * register on the device. The function requires a valid device structure
 * that has been properly initialized for I2C communication. It is
 * important to ensure that the register address is within the valid
 * range for the device and that the device is ready for communication.
 * The function returns an integer status code indicating the success or
 * failure of the write operation.
 *
 * @param dev A pointer to an ad9545_dev structure representing the device. This
 * must be initialized and configured for I2C communication. Must not
 * be null.
 * @param reg_addr A 16-bit unsigned integer specifying the register address to
 * write to. The address must be within the valid range of the
 * device's register map.
 * @param reg_data An 8-bit unsigned integer representing the data to be written
 * to the specified register. The data is masked to ensure it is
 * a single byte.
 * @return Returns an int32_t status code. A value of 0 typically indicates
 * success, while a negative value indicates an error occurred during
 * the write operation.
 ******************************************************************************/
int32_t ad9545_i2c_reg_write(struct ad9545_dev *dev,
			     uint16_t reg_addr,
			     uint8_t reg_data);
/* Reads multiple bytes starting from the specified I2C register address */
/***************************************************************************//**
 * @brief This function is used to read a sequence of bytes from a specified
 * register address on an AD9545 device using I2C communication. It is
 * essential to ensure that the device has been properly initialized and
 * that the I2C communication is set up before calling this function. The
 * function reads up to 512 bytes, and if the requested count exceeds
 * this limit, it returns an error. It is important to provide a valid
 * buffer to store the read data, and the function will populate this
 * buffer with the data read from the device.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The starting register address from which to begin reading.
 * Must be a valid register address for the device.
 * @param reg_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should have enough space to hold 'count'
 * bytes.
 * @param count The number of bytes to read from the device. Must be between 1
 * and 512 inclusive. If greater than 512, the function returns an
 * error.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails or if 'count' exceeds 512.
 ******************************************************************************/
int32_t ad9545_i2c_reg_read_multiple(struct ad9545_dev *dev,
				     uint16_t reg_addr,
				     uint8_t *reg_data,
				     uint16_t count);
/* Writes multiple bytes starting from the specified I2C register address */
/***************************************************************************//**
 * @brief Use this function to write a sequence of bytes to a specific register
 * address on the AD9545 device via I2C. This function is essential when
 * you need to configure or update multiple register values in a single
 * operation. Ensure that the device is properly initialized and that the
 * I2C communication is set up before calling this function. The function
 * will return an error if the number of bytes to write exceeds the
 * buffer limit of 512 bytes.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The starting register address where the data will be written.
 * It is a 16-bit unsigned integer.
 * @param reg_data A pointer to the data buffer containing the bytes to be
 * written. The caller retains ownership and it must not be
 * null.
 * @param count The number of bytes to write from the reg_data buffer. Must be
 * less than or equal to 512; otherwise, the function returns an
 * error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * when the count exceeds 512.
 ******************************************************************************/
int32_t ad9545_i2c_reg_write_multiple(struct ad9545_dev *dev,
				      uint16_t reg_addr,
				      uint8_t *reg_data,
				      uint16_t count);
/* Modifies specific bits in the specified register and writes the new value. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * AD9545 device by applying a mask and then writing the new value. It is
 * useful when only certain bits of a register need to be updated without
 * affecting the others. The function first reads the current value of
 * the register, applies the mask to clear the bits, and then sets the
 * specified bits with the provided data. It should be called when
 * precise bit-level control of a register is required.
 *
 * @param dev A pointer to an ad9545_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be modified. Must be a valid
 * register address for the AD9545 device.
 * @param mask A bitmask indicating which bits in the register should be
 * modified. Bits set to 1 in the mask will be affected.
 * @param data The new data to be written to the specified bits in the register.
 * Only the bits specified by the mask will be updated with this
 * data.
 * @return Returns an int32_t value indicating success (0) or an error code if
 * the operation fails.
 ******************************************************************************/
int32_t ad9545_write_mask(struct ad9545_dev *dev,
			  uint16_t reg_addr,
			  uint32_t mask,
			  uint8_t data);

/* Calibrates the APLLs of the AD9545 device. */
/***************************************************************************//**
 * @brief Use this function to calibrate all active APLLs (Analog Phase-Locked
 * Loops) in the AD9545 device. It should be called when the device is
 * initialized and configured, and before any operations that depend on
 * the APLLs being calibrated. The function iterates over all PLLs in the
 * device and calibrates those that are marked as used. If any
 * calibration fails, the function returns an error code, and no further
 * PLLs are calibrated. This function is essential for ensuring the
 * correct operation of the APLLs.
 *
 * @param dev A pointer to an initialized ad9545_dev structure. This structure
 * must be properly configured with the PLLs that are intended to be
 * used. The pointer must not be null, and the function will return
 * an error if the device is not correctly set up.
 * @return Returns 0 on success, indicating all used APLLs were calibrated
 * successfully. Returns a negative error code if any APLL calibration
 * fails, indicating which APLL caused the failure.
 ******************************************************************************/
int ad9545_calib_aplls(struct ad9545_dev *dev);

/* Device Setup */
/***************************************************************************//**
 * @brief This function sets up the AD9545 device by configuring its system
 * clock, input references, auxiliary NCOs, and other components
 * necessary for its operation. It must be called after the device has
 * been initialized and before any other operations are performed. The
 * function checks for lock status of PLLs and auxiliary DPLLs, issuing
 * warnings if any are unlocked or if there are reference faults. It
 * returns an error code if any setup step fails, allowing the caller to
 * handle initialization errors appropriately.
 *
 * @param dev A pointer to an initialized ad9545_dev structure representing the
 * device. Must not be null, and the device must be properly
 * initialized before calling this function.
 * @return Returns 0 on success, or a negative error code if any setup step
 * fails.
 ******************************************************************************/
int32_t ad9545_setup(struct ad9545_dev *dev);

/* Device Initialization */
/***************************************************************************//**
 * @brief This function sets up the AD9545 device using the provided
 * initialization parameters, configuring it for either SPI or I2C
 * communication based on the specified communication type. It must be
 * called before any other operations on the device to ensure proper
 * setup. The function allocates memory for the device structure,
 * initializes communication interfaces, and configures the device
 * registers. If initialization fails at any step, it returns an error
 * code and cleans up any allocated resources.
 *
 * @param device A pointer to a pointer of type `struct ad9545_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct ad9545_init_param` containing the
 * initialization parameters. This includes communication type
 * and specific settings for the device. Must not be null, and
 * the structure should be properly populated before calling
 * the function.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int32_t ad9545_init(struct ad9545_dev **device,
		    struct ad9545_init_param *init_param);

/* Free resources */
/***************************************************************************//**
 * @brief This function is used to release all resources allocated for an AD9545
 * device instance. It should be called when the device is no longer
 * needed to ensure proper cleanup and avoid memory leaks. The function
 * handles both SPI and I2C communication types, freeing the respective
 * descriptors. It also frees any allocated clock resources associated
 * with the device. The function must be called with a valid device
 * pointer, and it returns an error code if the device pointer is null or
 * if any underlying resource removal fails.
 *
 * @param dev A pointer to an ad9545_dev structure representing the device
 * instance to be removed. Must not be null. The function will return
 * -EINVAL if this parameter is null. The caller relinquishes
 * ownership of the memory pointed to by this parameter, which will
 * be freed by the function.
 * @return Returns 0 on success or a negative error code if an error occurs
 * during resource removal.
 ******************************************************************************/
int32_t ad9545_remove(struct ad9545_dev *dev);


#endif // AD9545_H_
