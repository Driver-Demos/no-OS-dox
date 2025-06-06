/***************************************************************************//**
 *   @file   AD9517.h
 *   @brief  Header file of AD9517 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
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
#ifndef __AD9517_H__
#define __AD9517_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/****************************** AD9517 ****************************************/
/******************************************************************************/
#define AD9517_READ				(1 << 15)
#define AD9517_WRITE				(0 << 15)
#define AD9517_CNT(x)				((x - 1) << 13)
#define AD9517_ADDR(x)				(x & 0x3FF)

#define AD9517_T1B				(1 << 16)
#define AD9517_T2B				(2 << 16)
#define AD9517_T3B				(3 << 16)
#define AD9517_TRANSF_LEN(x)			((uint32_t)x >> 16)

/* SPI Register Map */

/* Serial Port Configuration */
#define AD9517_REG_SERIAL_PORT_CONFIG		(AD9517_T1B | 0x000)
#define AD9517_REG_PART_ID			(AD9517_T1B | 0x003)
#define AD9517_REG_READBACK_CTRL		(AD9517_T1B | 0x004)

/* PLL */
#define AD9517_REG_PFD_CHARGE_PUMP		(AD9517_T1B | 0x010)
#define AD9517_REG_R_COUNTER			(AD9517_T2B | 0x012)
#define AD9517_REG_A_COUNTER			(AD9517_T1B | 0x013)
#define AD9517_REG_B_COUNTER			(AD9517_T2B | 0x015)
#define AD9517_REG_PLL_CTRL_1			(AD9517_T1B | 0x016)
#define AD9517_REG_PLL_CTRL_2			(AD9517_T1B | 0x017)
#define AD9517_REG_PLL_CTRL_3			(AD9517_T1B | 0x018)
#define AD9517_REG_PLL_CTRL_4			(AD9517_T1B | 0x019)
#define AD9517_REG_PLL_CTRL_5			(AD9517_T1B | 0x01A)
#define AD9517_REG_PLL_CTRL_6			(AD9517_T1B | 0x01B)
#define AD9517_REG_PLL_CTRL_7			(AD9517_T1B | 0x01C)
#define AD9517_REG_PLL_CTRL_8			(AD9517_T1B | 0x01D)
#define AD9517_REG_PLL_READBACK			(AD9517_T1B | 0x01F)

/* Fine Delay Adjust - OUT4 to OUT7 */
#define AD9517_REG_OUT4_DELAY_BYPASS		(AD9517_T1B | 0x0A0)
#define AD9517_REG_OUT4_DELAY_FULL_SCALE	(AD9517_T1B | 0x0A1)
#define AD9517_REG_OUT4_DELAY_FRACTION		(AD9517_T1B | 0x0A2)
#define AD9517_REG_OUT5_DELAY_BYPASS		(AD9517_T1B | 0x0A3)
#define AD9517_REG_OUT5_DELAY_FULL_SCALE	(AD9517_T1B | 0x0A4)
#define AD9517_REG_OUT5_DELAY_FRACTION		(AD9517_T1B | 0x0A5)
#define AD9517_REG_OUT6_DELAY_BYPASS		(AD9517_T1B | 0x0A6)
#define AD9517_REG_OUT6_DELAY_FULL_SCALE	(AD9517_T1B | 0x0A7)
#define AD9517_REG_OUT6_DELAY_FRACTION		(AD9517_T1B | 0x0A8)
#define AD9517_REG_OUT7_DELAY_BYPASS		(AD9517_T1B | 0x0A9)
#define AD9517_REG_OUT7_DELAY_FULL_SCALE	(AD9517_T1B | 0x0AA)
#define AD9517_REG_OUT7_DELAY_FRACTION		(AD9517_T1B | 0x0AB)

/* LVPECL Outputs */
#define AD9517_REG_LVPECL_OUT0			(AD9517_T1B | 0x0F0)
#define AD9517_REG_LVPECL_OUT1			(AD9517_T1B | 0x0F1)
#define AD9517_REG_LVPECL_OUT2			(AD9517_T1B | 0x0F4)
#define AD9517_REG_LVPECL_OUT3			(AD9517_T1B | 0x0F5)

/* LVDS/CMOS Outputs */
#define AD9517_REG_LVDS_CMOS_OUT4		(AD9517_T1B | 0x140)
#define AD9517_REG_LVDS_CMOS_OUT5		(AD9517_T1B | 0x141)
#define AD9517_REG_LVDS_CMOS_OUT6		(AD9517_T1B | 0x142)
#define AD9517_REG_LVDS_CMOS_OUT7		(AD9517_T1B | 0x143)

/* LVPECL Channel Dividers */
#define AD9517_REG_DIVIDER_0_0			(AD9517_T1B | 0x190)
#define AD9517_REG_DIVIDER_0_1			(AD9517_T1B | 0x191)
#define AD9517_REG_DIVIDER_0_2			(AD9517_T1B | 0x192)
#define AD9517_REG_DIVIDER_1_0			(AD9517_T1B | 0x196)
#define AD9517_REG_DIVIDER_1_1			(AD9517_T1B | 0x197)
#define AD9517_REG_DIVIDER_1_2			(AD9517_T1B | 0x198)

/* LVDS/CMOS Channel Dividers */
#define AD9517_REG_LVDS_CMOS_DIVIDER_2_0	(AD9517_T1B | 0x199)
#define AD9517_REG_LVDS_CMOS_DIVIDER_2_1	(AD9517_T1B | 0x19A)
#define AD9517_REG_LVDS_CMOS_DIVIDER_2_2	(AD9517_T1B | 0x19B)
#define AD9517_REG_LVDS_CMOS_DIVIDER_2_3	(AD9517_T1B | 0x19C)
#define AD9517_REG_LVDS_CMOS_DIVIDER_2_4	(AD9517_T1B | 0x19D)
#define AD9517_REG_LVDS_CMOS_DIVIDER_3_0	(AD9517_T1B | 0x19E)
#define AD9517_REG_LVDS_CMOS_DIVIDER_3_1	(AD9517_T1B | 0x19F)
#define AD9517_REG_LVDS_CMOS_DIVIDER_3_2	(AD9517_T1B | 0x1A0)
#define AD9517_REG_LVDS_CMOS_DIVIDER_3_3	(AD9517_T1B | 0x1A1)
#define AD9517_REG_LVDS_CMOS_DIVIDER_3_4	(AD9517_T1B | 0x1A2)

/* VCO Divider and CLK Input */
#define AD9517_REG_VCO_DIVIDER			(AD9517_T1B | 0x1E0)
#define AD9517_REG_INPUT_CLKS			(AD9517_T1B | 0x1E1)

/* System */
#define AD9517_REG_POWER_DOWN_SYNC		(AD9517_T1B | 0x230)

/* Update All Registers */
#define AD9517_REG_UPDATE_ALL_REGS		(AD9517_T1B | 0x232)

/* AD9517_REG_SERIAL_PORT_CONFIG Definition */
#define AD9517_SDO_ACTIVE			((1 << 7) | (1 << 0))
#define AD9517_LSB_FIRST			((1 << 6) | (1 << 1))
#define AD9517_SOFT_RESET			((1 << 5) | (1 << 2))
#define AD9517_LONG_INSTRUCTION			((1 << 4) | (1 << 3))

/* AD9517_REG_READBACK_CTRL Definition */
#define AD9517_REG_BANK_SELECTION		(1 << 0)

/* AD9517_REG_PFD_CHARGE_PUMP Definition */
#define AD9517_PFD_POLARITY			(1 << 7)
#define AD9517_CP_CURRENT(x)			((x & 0x7) << 4)
#define AD9517_CP_MODE(x)			((x & 0x3) << 2)
#define AD9517_PLL_POWER_DOWN(x)		((x & 0x3) << 0)

/* AD9517_REG_R_COUNTER Definition */
#define AD9517_R_COUNTER(x)			((x & 0x3FFF) << 0)

/* AD9517_REG_A_COUNTER Definition */
#define AD9517_A_COUNTER(x)			((x & 0x3F) << 0)

/* AD9517_REG_B_COUNTER Definition */
#define AD9517_B_COUNTER(x)			((x & 0x1FFF) << 0)

/* AD9517_REG_PLL_CTRL_1 Definition */
#define AD9517_CP_VCP_DIV2			(1 << 7)
#define AD9517_RESET_R_COUNTER			(1 << 6)
#define AD9517_RESET_A_B_COUNTERS		(1 << 5)
#define AD9517_RESET_ALL_COUNTERS		(1 << 4)
#define AD9517_B_COUNTER_BYPASS			(1 << 3)
#define AD9517_PRESCALER_P(x)			((x & 0x7) << 0)

/* AD9517_REG_PLL_CTRL_2 Definition */
#define AD9517_STATUS_PIN_CTRL(x)		((x & 0x3F) << 2)
#define AD9517_ANTIBACKLASH_PULSE_WIDTH(x)	((x & 0x3) << 0)

/* AD9517_REG_PLL_CTRL_3 Definition */
#define AD9517_LOCK_DETECT_COUNTER(x)		((x & 0x3) << 5)
#define AD9517_DIGITAL_LOCK_DETECT_WINDOW	(1 << 4)
#define AD9517_DIS_DIGITAL_LOCK_DETECT		(1 << 3)
#define AD9517_VCO_CAL_DIVIDER(x)		((x & 0x3) << 1)
#define AD9517_VCO_CAL_NOW			(1 << 0)

/* AD9517_REG_PLL_CTRL_4 Definition */
#define AD9517_SYNC_PIN_RESET(x)		((x & 0x3) << 6)
#define AD9517_R_PATH_DELAY(x)			((x & 0x7) << 3)
#define AD9517_N_PATH_DELAY(x)			((x & 0x7) << 0)

/* AD9517_REG_PLL_CTRL_5 Definition */
#define AD9517_REF_FREQ_MONITOR_THRESHOLD	(1 << 6)
#define AD9517_LD_PIN_CTRL(x)			((x & 0x3F) << 0)

/* AD9517_REG_PLL_CTRL_6 Definition */
#define AD9517_VCO_FREQ_MONITOR			(1 << 7)
#define AD9517_REF2_FREQ_MONITOR		(1 << 6)
#define AD9517_REF1_FREQ_MONITOR		(1 << 5)
#define AD9517_REFMON_PIN_CTRL(x)		((x & 0x1F) << 0)

/* AD9517_REG_PLL_CTRL_7 Definition */
#define AD9517_DIS_SWITCHOVER_DEGLITCH		(1 << 7)
#define AD9517_SELECT_REF2			(1 << 6)
#define AD9517_USE_REF_SEL_PIN			(1 << 5)
#define AD9517_REF2_POWER_ON			(1 << 2)
#define AD9517_REF1_POWER_ON			(1 << 1)
#define AD9517_DIFF_REF				(1 << 0)

/* AD9517_REG_PLL_CTRL_8 Definition */
#define AD9517_PLL_STATUS_REG_DIS		(1 << 4)
#define AD9517_LD_PIN_COMPARATOR_EN		(1 << 3)
#define AD9517_HOLDOVER_EN			((1 << 2) | (1 << 0))
#define AD9517_EXT_HOLDOVER_CTRL		(1 << 1)

/* AD9517_REG_PLL_READBACK Definition */
#define AD9517_VCO_CAL_FINISHED			(1 << 6)
#define AD9517_HOLDOVER_ACTIVE			(1 << 5)
#define AD9517_REF2_SELECTED			(1 << 4)
#define AD9517_VCO_FREQ_GREATER			(1 << 3)
#define AD9517_REF2_FREQ_GREATER		(1 << 2)
#define AD9517_REF1_FREQ_GREATER		(1 << 1)
#define AD9517_DIGITAL_LOCK_DETECT		(1 << 0)

/* AD9517_REG_OUTn_DELAY_BYPASS Definition */
#define AD9517_OUT_DELAY_BYPASS			(1 << 0)

/* AD9517_REG_OUTn_DELAY_FULL_SCALE Definition */
#define AD9517_OUT_RAMP_CAPACITORS(x)		((x & 0x7) << 3)
#define AD9517_OUT_RAMP_CURRENT(x)		((x & 0x7) << 0)

/* AD9517_REG_OUTn_DELAY_FRACTION Definition */
#define AD9517_OUT_DELAY_FRACTION(x)		((x & 0x3F) << 0)

/* AD9517_REG_LVPECL_OUTn Definition */
#define AD9517_OUT_LVPECL_INVERT		(1 << 4)
#define AD9517_OUT_LVPECL_DIFF_VOLTAGE(x)	((x & 0x3) << 2)
#define AD9517_OUT_LVPECL_POWER_DOWN(x)		((x & 0x3) << 0)

/* AD9517_REG_LVDS_CMOS_OUTn Definition */
#define AD9517_OUT_LVDS_CMOS_INVERT(x)		((x & 0x7) << 5)
#define AD9517_OUT_CMOS_B			(1 << 4)
#define AD9517_OUT_LVDS_CMOS			(1 << 3)
#define AD9517_OUT_LVDS_OUTPUT_CURRENT(x)	((x & 0x3) << 1)
#define AD9517_OUT_LVDS_CMOS_POWER_DOWN		(1 << 0)

/* AD9517_REG_DIVIDER_n_0 Definition */
#define AD9517_DIVIDER_LOW_CYCLES(x)		((x & 0xF) << 4)
#define AD9517_DIVIDER_HIGH_CYCLES(x)		((x & 0xF) << 0)

/* AD9517_REG_DIVIDER_n_1 Definition */
#define AD9517_DIVIDER_BYPASS			(1 << 7)
#define AD9517_LVPECL_DIVIDER_NOSYNC		(1 << 6)
#define AD9517_LVPECL_DIVIDER_FORCE_HIGH	(1 << 5)
#define AD9517_DIVIDER_START_HIGH		(1 << 4)
#define AD9517_DIVIDER_PHASE_OFFSET(x)		((x & 0xF) << 0)

/* AD9517_REG_DIVIDER_n_2 Definition */
#define AD9517_DIVIDER_DIRECT_TO_OUTPUT		(1 << 1)
#define AD9517_DIVIDER_DCCOFF			(1 << 0)

/* AD9517_REG_LVDS_CMOS_DIVIDER_n_0 Definition */
#define AD9517_LOW_CYCLES_DIVIDER_1(x)		((x & 0xF) << 4)
#define AD9517_HIGH_CYCLES_DIVIDER_1(x)		((x & 0xF) << 0)

/* AD9517_REG_LVDS_CMOS_DIVIDER_n_1 Definition */
#define AD9517_PHASE_OFFSET_DIVIDER_2(x)	((x & 0xF) << 4)
#define AD9517_PHASE_OFFSET_DIVIDER_1(x)	((x & 0xF) << 0)

/* AD9517_REG_LVDS_CMOS_DIVIDER_n_2 Definition */
#define AD9517_LOW_CYCLES_DIVIDER_2(x)		((x & 0xF) << 4)
#define AD9517_HIGH_CYCLES_DIVIDER_2(x)		((x & 0xF) << 0)

/* AD9517_REG_LVDS_CMOS_DIVIDER_n_3 Definition */
#define AD9517_BYPASS_DIVIDER_2			(1 << 5)
#define AD9517_BYPASS_DIVIDER_1			(1 << 4)
#define AD9517_LVDS_CMOS_DIVIDER_NOSYNC		(1 << 3)
#define AD9517_LVDS_CMOS_DIVIDER_FORCE_HIGH	(1 << 2)
#define AD9517_START_HIGH_DIVIDER_2		(1 << 1)
#define AD9517_START_HIGH_DIVIDER_1		(1 << 0)

/* AD9517_REG_LVDS_CMOS_DIVIDER_n_4 Definition */
#define AD9517_DIVIDER_DCCOFF			(1 << 0)

/* AD9517_REG_VCO_DIVIDER Definition */
#define AD9517_VCO_DIVIDER(x)			((x & 0x7) << 0)

/* AD9517_REG_INPUT_CLKS Definition */
#define AD9517_POWER_DOWN_CLK_INPUT_SECTION	(1 << 4)
#define AD9517_POWER_DOWN_VCO_CLK_INTERFACE	(1 << 3)
#define AD9517_POWER_DOWN_VCO_CLK		(1 << 2)
#define AD9517_SEL_VCO_CLK			(1 << 1)
#define AD9517_BYPASS_VCO_DIVIDER		(1 << 0)

/* AD9517_REG_POWER_DOWN_SYNC Definition */
#define AD9517_POWER_DOWN_SYNC			(1 << 2)
#define AD9517_POWER_DOWN_DIST_REF		(1 << 1)
#define AD9517_SOFT_SYNC			(1 << 0)

/* AD9517_REG_UPDATE_ALL_REGS Definition */
#define AD9517_UPDATE_ALL_REGS			(1 << 0)

#define AD9517_1_MIN_VCO_FREQ			2300000000
#define AD9517_1_MAX_VCO_FREQ			2650000000
#define AD9517_2_MIN_VCO_FREQ			2050000000
#define AD9517_2_MAX_VCO_FREQ			2330000000
#define AD9517_3_MIN_VCO_FREQ			1750000000
#define AD9517_3_MAX_VCO_FREQ			2250000000
#define AD9517_4_MIN_VCO_FREQ			1450000000
#define AD9517_4_MAX_VCO_FREQ			1800000000
#define AD9517_MAX_PFD_FREQ			100000000
#define AD9517_MAX_PRESCLAER_OUT_FREQ		300000000

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Platform specific information */
/***************************************************************************//**
 * @brief The `ad9517_platform_data` structure is used to configure the AD9517
 * device, which is a clock distribution IC with an integrated PLL. This
 * structure holds configuration parameters for the PLL reference
 * frequencies, external clock frequency, internal VCO frequency, and
 * various control flags for power and reference selection. It allows for
 * the customization of the clocking setup, including enabling
 * differential reference mode, selecting between different reference
 * sources, and managing power states of the VCO and reference inputs.
 *
 * @param ref_1_freq Frequency of the Reference 1.
 * @param ref_2_freq Frequency of the Reference 2.
 * @param diff_ref_en Selects the differential PLL reference mode.
 * @param ref_1_power_on Power on REF1.
 * @param ref_2_power_on Power on REF2.
 * @param ref_sel_pin_en Set method of PLL reference selection.
 * @param ref_sel_pin State of the REF_SEL pin.
 * @param ref_2_en Select Reference 2.
 * @param ext_clk_freq Frequency of the external clock.
 * @param int_vco_freq Frequency of the internal VCO.
 * @param vco_clk_sel Selection between external clock or VCO.
 * @param power_down_vco_clk Power down the VCO clock.
 * @param name Optional descriptive name for the platform data.
 ******************************************************************************/
struct ad9517_platform_data {
	/* PLL Reference */
	int32_t ref_1_freq;	// Frequency of the Reference 1.
	int32_t ref_2_freq;	// Frequency of the Reference 2.
	uint8_t diff_ref_en;	// Selects the differential PLL reference mode.
	uint8_t ref_1_power_on;	// Power on REF1.
	uint8_t ref_2_power_on;	// Power on REF2.
	uint8_t ref_sel_pin_en;	// Set method of PLL reference selection.
	uint8_t ref_sel_pin;	// State of the REF_SEL pin.
	uint8_t ref_2_en;	// Select Reference 2.

	/* External Clock  */
	int64_t ext_clk_freq;	// Frequency of the external clock.

	/* VCO  */
	int64_t int_vco_freq;	// Frequency of the internal VCO.

	/* External Clock or VCO selection */
	int32_t vco_clk_sel;

	uint8_t power_down_vco_clk;
	uint8_t name[16];
};

/* LVPECL output channel configuration. */
/***************************************************************************//**
 * @brief The `ad9517_lvpecl_channel_spec` structure is used to define the
 * configuration for an LVPECL output channel in the AD9517 device. It
 * includes fields for specifying the channel number, whether the output
 * clock polarity should be inverted, the differential voltage level for
 * the LVPECL output, and an optional name for the channel. This
 * structure is essential for configuring the output characteristics of
 * the AD9517's LVPECL channels.
 *
 * @param channel_num Output channel number.
 * @param out_invert_en Invert the polarity of the output clock.
 * @param out_diff_voltage LVPECL output differential voltage.
 * @param name Optional descriptive channel name.
 ******************************************************************************/
struct ad9517_lvpecl_channel_spec {
	uint8_t channel_num;	  // Output channel number.
	uint8_t out_invert_en;	  // Invert the polarity of the output clock.
	uint8_t out_diff_voltage; // LVPECL output differential voltage.
	uint8_t name[16];	  // Optional descriptive channel name.
};

/***************************************************************************//**
 * @brief The `ad9517_type` enumeration defines constants representing different
 * models of the AD9517 device, each associated with a unique hexadecimal
 * identifier. This enumeration is used to specify the type of AD9517
 * device being configured or controlled, allowing the software to handle
 * device-specific configurations and operations.
 *
 * @param AD9517_1 Represents the AD9517-1 device with a value of 0x51.
 * @param AD9517_2 Represents the AD9517-2 device with a value of 0x91.
 * @param AD9517_3 Represents the AD9517-3 device with a value of 0x53.
 * @param AD9517_4 Represents the AD9517-4 device with a value of 0xd3.
 ******************************************************************************/
enum ad9517_type {
	AD9517_1 = 0x51,
	AD9517_2 = 0x91,
	AD9517_3 = 0x53,
	AD9517_4 = 0xd3
};

/***************************************************************************//**
 * @brief The `out_diff_voltage_options` enumeration defines a set of possible
 * differential voltage levels for LVPECL (Low Voltage Positive Emitter
 * Coupled Logic) outputs. These options allow the user to select the
 * desired output voltage level for specific applications, providing
 * flexibility in configuring the output characteristics of the AD9517
 * device.
 *
 * @param LVPECL_400mV Represents a differential voltage option of 400mV for
 * LVPECL outputs.
 * @param LVPECL_600mV Represents a differential voltage option of 600mV for
 * LVPECL outputs.
 * @param LVPECL_780mV Represents a differential voltage option of 780mV for
 * LVPECL outputs.
 * @param LVPECL_960mV Represents a differential voltage option of 960mV for
 * LVPECL outputs.
 ******************************************************************************/
enum out_diff_voltage_options {
	LVPECL_400mV,
	LVPECL_600mV,
	LVPECL_780mV,
	LVPECL_960mV,
};

/* LVDS/CMOS output channel configuration. */
/***************************************************************************//**
 * @brief The `ad9517_lvds_cmos_channel_spec` structure is used to define the
 * configuration for an LVDS/CMOS output channel in the AD9517 device. It
 * includes fields to specify the channel number, whether the output
 * clock polarity should be inverted, the logic level (LVDS or CMOS), and
 * whether the CMOS B output should be enabled in CMOS mode.
 * Additionally, it allows setting the LVDS output current level and
 * provides an optional field for a descriptive channel name. This
 * structure is essential for configuring the output characteristics of
 * the AD9517's LVDS/CMOS channels.
 *
 * @param channel_num Output channel number.
 * @param out_invert Invert the polarity of the output clock.
 * @param logic_level Select LVDS or CMOS logic levels.
 * @param cmos_b_en In CMOS mode, turn on/off the CMOS B output.
 * @param out_lvds_current LVDS output current level.
 * @param name Optional descriptive channel name.
 ******************************************************************************/
struct ad9517_lvds_cmos_channel_spec {
	uint8_t channel_num;	  // Output channel number.
	uint8_t out_invert;	  // Invert the polarity of the output clock.
	uint8_t logic_level;	  // Select LVDS or CMOS logic levels.
	uint8_t cmos_b_en;	  // In CMOS mode, turn on/off the CMOS B output
	uint8_t out_lvds_current; // LVDS output current level.
	uint8_t name[16];	  // Optional descriptive channel name.
};

/***************************************************************************//**
 * @brief The `logic_level_options` enumeration defines two possible logic level
 * configurations for output channels, specifically LVDS (Low-Voltage
 * Differential Signaling) and CMOS (Complementary Metal-Oxide-
 * Semiconductor). This enumeration is used to specify the type of logic
 * level that an output channel should use, allowing for flexibility in
 * configuring the output characteristics of the device.
 *
 * @param LVDS Represents the LVDS logic level option.
 * @param CMOS Represents the CMOS logic level option.
 ******************************************************************************/
enum logic_level_options {
	LVDS,
	CMOS
};

/***************************************************************************//**
 * @brief The `out_lvds_current_options` enumeration defines a set of constants
 * representing different current levels for LVDS (Low-Voltage
 * Differential Signaling) outputs. These options allow for the selection
 * of specific current levels to be used in LVDS output configurations,
 * which can be critical for ensuring proper signal integrity and
 * performance in various electronic applications.
 *
 * @param LVDS_1_75mA Represents an LVDS current option of 1.75mA.
 * @param LVDS_3_5mA Represents an LVDS current option of 3.5mA.
 * @param LVDS_5_25mA Represents an LVDS current option of 5.25mA.
 * @param LVDS_7mA Represents an LVDS current option of 7mA.
 ******************************************************************************/
enum out_lvds_current_options {
	LVDS_1_75mA,
	LVDS_3_5mA,
	LVDS_5_25mA,
	LVDS_7mA,
};

/***************************************************************************//**
 * @brief The `ad9517_state` structure encapsulates the state and configuration
 * settings for the AD9517 clock distribution device. It includes
 * pointers to platform-specific data and channel configurations for
 * LVPECL and LVDS/CMOS outputs, as well as various counters and settings
 * related to the phase-locked loop (PLL) such as the R, A, and B
 * counters, VCO divider, prescaler, and antibacklash pulse width. This
 * structure is essential for managing the device's operational
 * parameters and ensuring proper clock distribution and signal
 * integrity.
 *
 * @param pdata Pointer to platform-specific data for the AD9517 device.
 * @param lvpecl_channels Pointer to the configuration of LVPECL output
 * channels.
 * @param lvds_cmos_channels Pointer to the configuration of LVDS/CMOS output
 * channels.
 * @param r_counter 32-bit integer representing the R counter value in the PLL.
 * @param a_counter 8-bit integer representing the A counter value in the PLL.
 * @param b_counter 16-bit integer representing the B counter value in the PLL.
 * @param vco_divider 8-bit integer representing the VCO divider setting.
 * @param prescaler_p 8-bit integer representing the prescaler value.
 * @param antibacklash_pulse_width 8-bit integer representing the antibacklash
 * pulse width setting.
 ******************************************************************************/
struct ad9517_state {
	struct ad9517_platform_data	     *pdata;
	struct ad9517_lvpecl_channel_spec    *lvpecl_channels;
	struct ad9517_lvds_cmos_channel_spec *lvds_cmos_channels;
	uint32_t			     r_counter;
	uint8_t 			     a_counter;
	uint16_t			     b_counter;
	uint8_t				     vco_divider;
	uint8_t				     prescaler_p;
	uint8_t				     antibacklash_pulse_width;
};

/***************************************************************************//**
 * @brief The `ad9517_dev` structure is a compound data type used to encapsulate
 * the necessary components for interfacing with and configuring an
 * AD9517 device. It includes a pointer to a SPI descriptor for handling
 * communication, a state structure that maintains the current
 * configuration and operational parameters of the device, and an
 * enumeration that identifies the specific type of AD9517 device in use.
 * This structure is central to managing the device's settings and
 * operations within the driver.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param ad9517_st Holds the state and configuration of the AD9517 device.
 * @param ad9517_type Specifies the type of AD9517 device being used.
 ******************************************************************************/
struct ad9517_dev {
	/* SPI */
	struct no_os_spi_desc	    *spi_desc;
	/* Device Settings */
	struct ad9517_state ad9517_st;
	enum ad9517_type	ad9517_type;
};

/***************************************************************************//**
 * @brief The `ad9517_init_param` structure is used to initialize the AD9517
 * device, encapsulating the necessary parameters for SPI communication
 * and device-specific settings. It includes a SPI initialization
 * parameter, a state structure for the AD9517 device, and an enumeration
 * to specify the type of AD9517 device. This structure is essential for
 * setting up the device before it can be used for clock distribution and
 * management.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param ad9517_st Contains the state information and configuration for the
 * AD9517 device.
 * @param ad9517_type Specifies the type of AD9517 device being used, as defined
 * by the enum ad9517_type.
 ******************************************************************************/
struct ad9517_init_param {
	/* SPI */
	struct no_os_spi_init_param    spi_init;
	/* Device Settings */
	struct ad9517_state ad9517_st;
	enum ad9517_type	ad9517_type;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function sets up the AD9517 device by initializing it with the
 * provided configuration parameters. It must be called before any other
 * operations on the AD9517 device. The function configures the SPI
 * interface, verifies the device ID, and applies the user-defined
 * settings for PLL, LVPECL, and LVDS/CMOS outputs. It also handles VCO
 * frequency settings and calibration if VCO is selected. The function
 * allocates memory for the device structure, which must be freed by the
 * caller using the appropriate cleanup function. If initialization fails
 * at any step, an error code is returned, and the device pointer is not
 * valid.
 *
 * @param device A pointer to a pointer of type `struct ad9517_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and must free the allocated memory
 * after use.
 * @param init_param A structure of type `struct ad9517_init_param` containing
 * the initialization parameters for the AD9517 device. This
 * includes SPI initialization parameters and device-specific
 * settings. The structure must be fully populated with valid
 * data before calling the function.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer is set to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t ad9517_setup(struct ad9517_dev **device,
		     struct ad9517_init_param init_param);
/***************************************************************************//**
 * @brief Use this function to release all resources associated with an AD9517
 * device instance when it is no longer needed. This function should be
 * called to clean up after a successful call to ad9517_setup, ensuring
 * that any allocated memory and initialized hardware interfaces are
 * properly released. It is important to call this function to prevent
 * resource leaks in your application.
 *
 * @param dev A pointer to an ad9517_dev structure representing the device to be
 * removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns an int32_t indicating the success or failure of the
 * operation. A return value of 0 typically indicates success, while a
 * negative value indicates an error occurred during the resource
 * release process.
 ******************************************************************************/
int32_t ad9517_remove(struct ad9517_dev *dev);
/***************************************************************************//**
 * @brief Use this function to write a 16-bit value to a specific register of
 * the AD9517 device. It is essential to ensure that the device has been
 * properly initialized using `ad9517_setup` before calling this
 * function. The function communicates with the device over SPI, and any
 * errors during this communication will be reflected in the return
 * value. This function is typically used when configuring the device or
 * updating its settings.
 *
 * @param dev A pointer to an `ad9517_dev` structure representing the device.
 * Must not be null, and the device must be initialized.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address as defined by the AD9517 register map.
 * @param reg_val The 16-bit value to write to the specified register. The value
 * should be formatted according to the register's requirements.
 * @return Returns 0 on success or a negative error code if the SPI
 * communication fails.
 ******************************************************************************/
int32_t ad9517_write(struct ad9517_dev *dev,
		     uint32_t reg_addr,
		     uint16_t reg_val);
/***************************************************************************//**
 * @brief Use this function to read the value from a specific register of the
 * AD9517 device. It is essential to ensure that the device has been
 * properly initialized using `ad9517_setup` before calling this
 * function. The function communicates with the device over SPI to
 * retrieve the register value. The caller must provide a valid device
 * structure and a pointer to store the read value. This function returns
 * an error code if the read operation fails.
 *
 * @param dev A pointer to an `ad9517_dev` structure representing the device.
 * Must not be null and should be initialized with `ad9517_setup`.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address as defined by the AD9517 register map.
 * @param reg_value A pointer to a `uint32_t` where the read register value will
 * be stored. Must not be null.
 * @return Returns an `int32_t` indicating the success or failure of the read
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t ad9517_read(struct ad9517_dev *dev,
		    uint32_t reg_addr,
		    uint32_t *reg_value);
/***************************************************************************//**
 * @brief Use this function to apply any changes made to the buffer registers of
 * the AD9517 device by transferring them into the active registers. This
 * function should be called after making configuration changes that need
 * to be applied to the device. It is essential to ensure that the device
 * has been properly initialized before calling this function. The
 * function returns an error code if the update process fails.
 *
 * @param dev A pointer to an initialized ad9517_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized or if the pointer is invalid.
 * @return Returns an int32_t error code indicating success or failure of the
 * update operation.
 ******************************************************************************/
int32_t ad9517_update(struct ad9517_dev *dev);
/***************************************************************************//**
 * @brief This function configures the Voltage Controlled Oscillator (VCO)
 * frequency for a specified AD9517 device. It should be called when you
 * need to set or adjust the VCO frequency within the valid range for the
 * specific AD9517 model being used. The function checks if the desired
 * frequency is within the allowable range for the device type and
 * returns an error if it is not. It also updates the internal state of
 * the device to reflect the new frequency settings. This function must
 * be called after the device has been properly initialized.
 *
 * @param dev A pointer to an initialized ad9517_dev structure representing the
 * device. Must not be null.
 * @param frequency The desired VCO frequency in Hz. Must be within the valid
 * range for the specific AD9517 model. If the frequency is
 * outside this range, the function returns an error.
 * @return Returns the actual VCO frequency set in Hz if successful, or -1 if an
 * error occurs (e.g., invalid frequency or device type).
 ******************************************************************************/
int64_t ad9517_vco_frequency(struct ad9517_dev *dev,
			     int64_t frequency);
/***************************************************************************//**
 * @brief This function configures the frequency for a specified channel on the
 * AD9517 device. It should be called when you need to set or adjust the
 * output frequency of a particular channel. The function requires a
 * valid device structure and a channel number between 0 and 7. The
 * frequency parameter specifies the desired frequency in Hz. The
 * function will adjust the internal dividers to achieve the closest
 * possible frequency to the requested value, subject to the device's
 * constraints. If the channel number is invalid, the function returns an
 * error code.
 *
 * @param dev A pointer to an initialized ad9517_dev structure representing the
 * device. Must not be null.
 * @param channel An integer specifying the channel number to configure. Valid
 * range is 0 to 7.
 * @param frequency A 64-bit integer specifying the desired frequency in Hz. The
 * function will attempt to set the channel to this frequency,
 * adjusting internal dividers as necessary.
 * @return Returns the actual frequency set on the channel in Hz, or a negative
 * error code if the operation fails.
 ******************************************************************************/
int64_t ad9517_frequency(struct ad9517_dev *dev,
			 int32_t channel,
			 int64_t frequency);
/***************************************************************************//**
 * @brief This function is used to configure the phase offset for a specific
 * channel on the AD9517 device. It should be called when there is a need
 * to adjust the phase of the output signal for synchronization or timing
 * purposes. The function requires a valid device structure and a channel
 * number between 0 and 7. If the channel number is outside this range,
 * the function will not perform any operation. The phase value is
 * applied to the specified channel, and the function returns the phase
 * value if successful, or an error code if the operation fails.
 *
 * @param dev A pointer to an ad9517_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function.
 * @param channel An integer specifying the channel number to configure. Valid
 * values are between 0 and 7 inclusive. If the value is outside
 * this range, the function will not perform any operation.
 * @param phase An integer representing the phase offset to set for the
 * specified channel. The function does not specify constraints on
 * the phase value, but it should be within the range supported by
 * the device for meaningful results.
 * @return Returns the phase value if the operation is successful, or a negative
 * error code if an error occurs during the operation.
 ******************************************************************************/
int32_t ad9517_phase(struct ad9517_dev *dev,
		     int32_t channel,
		     int32_t phase);
/***************************************************************************//**
 * @brief This function configures the power mode for a specified channel on the
 * AD9517 device. It can be used to enable or disable power to LVPECL or
 * LVDS/CMOS output channels. The function must be called with a valid
 * device structure and a channel number within the supported range. For
 * LVPECL channels (0-3), the mode can be set between 0 and 3, while for
 * LVDS/CMOS channels (4-7), the mode can be set between 0 and 1. If an
 * invalid channel or mode is provided, the function returns an error
 * code. The function also allows querying the current power mode by
 * passing an invalid mode value.
 *
 * @param dev A pointer to an initialized ad9517_dev structure. Must not be
 * null. The caller retains ownership.
 * @param channel An integer specifying the channel number. Valid values are
 * 0-7. Channels 0-3 are LVPECL, and channels 4-7 are LVDS/CMOS.
 * Invalid channel numbers result in an error return.
 * @param mode An integer specifying the power mode. For channels 0-3, valid
 * values are 0-3. For channels 4-7, valid values are 0-1. If an
 * invalid mode is provided, the function returns the current power
 * mode.
 * @return Returns the set mode on success, or a negative error code on failure.
 * If querying the current mode, returns the current mode or an error
 * code.
 ******************************************************************************/
int32_t ad9517_power_mode(struct ad9517_dev *dev,
			  int32_t channel,
			  int32_t mode);

#endif // __AD9517_H__
