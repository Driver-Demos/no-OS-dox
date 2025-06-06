/***************************************************************************//**
 *   @file   AD9523.h
 *   @brief  Header file of AD9523 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
 ********************************************************************************
 * Copyright 2012-2016(c) Analog Devices, Inc.
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
 *
 *******************************************************************************/
#ifndef _AD9523_H_
#define _AD9523_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/****************************** AD9523 ****************************************/
/******************************************************************************/
/* Registers */

#define AD9523_READ						(1 << 15)
#define AD9523_WRITE						(0 << 15)
#define AD9523_CNT(x)						(((x) - 1) << 13)
#define AD9523_ADDR(x)						((x) & 0xFFF)

#define AD9523_R1B						(1 << 16)
#define AD9523_R2B						(2 << 16)
#define AD9523_R3B						(3 << 16)
#define AD9523_TRANSF_LEN(x)					((x) >> 16)

#define AD9523_SERIAL_PORT_CONFIG				(AD9523_R1B | 0x0)
#define AD9523_VERSION_REGISTER					(AD9523_R1B | 0x2)
#define AD9523_PART_REGISTER					(AD9523_R1B | 0x3)
#define AD9523_READBACK_CTRL					(AD9523_R1B | 0x4)

#define AD9523_EEPROM_CUSTOMER_VERSION_ID			(AD9523_R2B | 0x6)

#define AD9523_PLL1_REF_A_DIVIDER				(AD9523_R2B | 0x11)
#define AD9523_PLL1_REF_B_DIVIDER				(AD9523_R2B | 0x13)
#define AD9523_PLL1_REF_TEST_DIVIDER				(AD9523_R1B | 0x14)
#define AD9523_PLL1_FEEDBACK_DIVIDER				(AD9523_R2B | 0x17)
#define AD9523_PLL1_CHARGE_PUMP_CTRL				(AD9523_R2B | 0x19)
#define AD9523_PLL1_INPUT_RECEIVERS_CTRL			(AD9523_R1B | 0x1A)
#define AD9523_PLL1_REF_CTRL					(AD9523_R1B | 0x1B)
#define AD9523_PLL1_MISC_CTRL					(AD9523_R1B | 0x1C)
#define AD9523_PLL1_LOOP_FILTER_CTRL				(AD9523_R1B | 0x1D)

#define AD9523_PLL2_CHARGE_PUMP					(AD9523_R1B | 0xF0)
#define AD9523_PLL2_FEEDBACK_DIVIDER_AB				(AD9523_R1B | 0xF1)
#define AD9523_PLL2_CTRL					(AD9523_R1B | 0xF2)
#define AD9523_PLL2_VCO_CTRL					(AD9523_R1B | 0xF3)
#define AD9523_PLL2_VCO_DIVIDER					(AD9523_R1B | 0xF4)
#define AD9523_PLL2_LOOP_FILTER_CTRL				(AD9523_R2B | 0xF6)
#define AD9523_PLL2_R2_DIVIDER					(AD9523_R1B | 0xF7)

#define AD9523_CHANNEL_CLOCK_DIST(ch)				(AD9523_R3B | (0x192 + 3 * ch))

#define AD9523_PLL1_OUTPUT_CTRL					(AD9523_R1B | 0x1BA)
#define AD9523_PLL1_OUTPUT_CHANNEL_CTRL				(AD9523_R1B | 0x1BB)

#define AD9523_READBACK_0					(AD9523_R1B | 0x22C)
#define AD9523_READBACK_1					(AD9523_R1B | 0x22D)

#define AD9523_STATUS_SIGNALS					(AD9523_R3B | 0x232)
#define AD9523_POWER_DOWN_CTRL					(AD9523_R1B | 0x233)
#define AD9523_IO_UPDATE					(AD9523_R1B | 0x234)

#define AD9523_EEPROM_DATA_XFER_STATUS				(AD9523_R1B | 0xB00)
#define AD9523_EEPROM_ERROR_READBACK				(AD9523_R1B | 0xB01)
#define AD9523_EEPROM_CTRL1					(AD9523_R1B | 0xB02)
#define AD9523_EEPROM_CTRL2					(AD9523_R1B | 0xB03)

/* AD9523_SERIAL_PORT_CONFIG */

#define AD9523_SER_CONF_SDO_ACTIVE				((1 << 7) | (1 << 0))
#define AD9523_SER_CONF_SOFT_RESET				((1 << 5) | (1 << 2))

/* AD9523_READBACK_CTRL */
#define AD9523_READBACK_CTRL_READ_BUFFERED			(1 << 0)

/* AD9523_PLL1_CHARGE_PUMP_CTRL */
#define AD9523_PLL1_CHARGE_PUMP_CURRENT_nA(x)			(((x) / 500) & 0x7F)
#define AD9523_PLL1_CHARGE_PUMP_TRISTATE			(1 << 7)
#define AD9523_PLL1_CHARGE_PUMP_MODE_NORMAL			(3 << 8)
#define AD9523_PLL1_CHARGE_PUMP_MODE_PUMP_DOWN			(2 << 8)
#define AD9523_PLL1_CHARGE_PUMP_MODE_PUMP_UP			(1 << 8)
#define AD9523_PLL1_CHARGE_PUMP_MODE_TRISTATE			(0 << 8)
#define AD9523_PLL1_BACKLASH_PW_MIN				(0 << 10)
#define AD9523_PLL1_BACKLASH_PW_LOW				(1 << 10)
#define AD9523_PLL1_BACKLASH_PW_HIGH				(2 << 10)
#define AD9523_PLL1_BACKLASH_PW_MAX				(3 << 10)

/* AD9523_PLL1_INPUT_RECEIVERS_CTRL */
#define AD9523_PLL1_REF_TEST_RCV_EN				(1 << 7)
#define AD9523_PLL1_REFB_DIFF_RCV_EN				(1 << 6)
#define AD9523_PLL1_REFA_DIFF_RCV_EN				(1 << 5)
#define AD9523_PLL1_REFB_RCV_EN					(1 << 4)
#define AD9523_PLL1_REFA_RCV_EN					(1 << 3)
#define AD9523_PLL1_REFA_REFB_PWR_CTRL_EN			(1 << 2)
#define AD9523_PLL1_OSC_IN_CMOS_NEG_INP_EN			(1 << 1)
#define AD9523_PLL1_OSC_IN_DIFF_EN				(1 << 0)

/* AD9523_PLL1_REF_CTRL */
#define AD9523_PLL1_BYPASS_REF_TEST_DIV_EN			(1 << 7)
#define AD9523_PLL1_BYPASS_FEEDBACK_DIV_EN			(1 << 6)
#define AD9523_PLL1_ZERO_DELAY_MODE_INT				(1 << 5)
#define AD9523_PLL1_ZERO_DELAY_MODE_EXT				(0 << 5)
#define AD9523_PLL1_OSC_IN_PLL_FEEDBACK_EN			(1 << 4)
#define AD9523_PLL1_ZD_IN_CMOS_NEG_INP_EN			(1 << 3)
#define AD9523_PLL1_ZD_IN_DIFF_EN				(1 << 2)
#define AD9523_PLL1_REFB_CMOS_NEG_INP_EN			(1 << 1)
#define AD9523_PLL1_REFA_CMOS_NEG_INP_EN			(1 << 0)

/* AD9523_PLL1_MISC_CTRL */
#define AD9523_PLL1_REFB_INDEP_DIV_CTRL_EN			(1 << 7)
#define AD9523_PLL1_OSC_CTRL_FAIL_VCC_BY2_EN			(1 << 6)
#define AD9523_PLL1_REF_MODE(x)					((x) << 2)
#define AD9523_PLL1_BYPASS_REFB_DIV				(1 << 1)
#define AD9523_PLL1_BYPASS_REFA_DIV				(1 << 0)

/* AD9523_PLL1_LOOP_FILTER_CTRL */
#define AD9523_PLL1_LOOP_FILTER_RZERO(x)			((x) & 0xF)

/* AD9523_PLL2_CHARGE_PUMP */
#define AD9523_PLL2_CHARGE_PUMP_CURRENT_nA(x)			((x) / 3500)

/* AD9523_PLL2_FEEDBACK_DIVIDER_AB */
#define AD9523_PLL2_FB_NDIV_A_CNT(x)				(((x) & 0x3) << 6)
#define AD9523_PLL2_FB_NDIV_B_CNT(x)				(((x) & 0x3F) << 0)
#define AD9523_PLL2_FB_NDIV(a, b)				(4 * (b) + (a))

/* AD9523_PLL2_CTRL */
#define AD9523_PLL2_CHARGE_PUMP_MODE_NORMAL			(3 << 0)
#define AD9523_PLL2_CHARGE_PUMP_MODE_PUMP_DOWN			(2 << 0)
#define AD9523_PLL2_CHARGE_PUMP_MODE_PUMP_UP			(1 << 0)
#define AD9523_PLL2_CHARGE_PUMP_MODE_TRISTATE			(0 << 0)
#define AD9523_PLL2_BACKLASH_PW_MIN				(0 << 2)
#define AD9523_PLL2_BACKLASH_PW_LOW				(1 << 2)
#define AD9523_PLL2_BACKLASH_PW_HIGH				(2 << 2)
#define AD9523_PLL2_BACKLASH_PW_MAX				(3 << 1)
#define AD9523_PLL2_BACKLASH_CTRL_EN				(1 << 4)
#define AD9523_PLL2_FREQ_DOUBLER_EN				(1 << 5)
#define AD9523_PLL2_LOCK_DETECT_PWR_DOWN_EN			(1 << 7)

/* AD9523_PLL2_VCO_CTRL */
#define AD9523_PLL2_VCO_CALIBRATE				(1 << 1)
#define AD9523_PLL2_FORCE_VCO_MIDSCALE				(1 << 2)
#define AD9523_PLL2_FORCE_REFERENCE_VALID			(1 << 3)
#define AD9523_PLL2_FORCE_RELEASE_SYNC				(1 << 4)

/* AD9523_PLL2_VCO_DIVIDER */
#define AD9523_PLL2_VCO_DIV_M1(x)				((((x) - 3) & 0x3) << 0)
#define AD9523_PLL2_VCO_DIV_M2(x)				((((x) - 3) & 0x3) << 4)
#define AD9523_PLL2_VCO_DIV_M1_PWR_DOWN_EN			(1 << 2)
#define AD9523_PLL2_VCO_DIV_M2_PWR_DOWN_EN			(1 << 6)

/* AD9523_PLL2_LOOP_FILTER_CTRL */
#define AD9523_PLL2_LOOP_FILTER_CPOLE1(x)			(((x) & 0x7) << 0)
#define AD9523_PLL2_LOOP_FILTER_RZERO(x)			(((x) & 0x7) << 3)
#define AD9523_PLL2_LOOP_FILTER_RPOLE2(x)			(((x) & 0x7) << 6)
#define AD9523_PLL2_LOOP_FILTER_RZERO_BYPASS_EN			(1 << 8)

/* AD9523_PLL2_R2_DIVIDER */
#define AD9523_PLL2_R2_DIVIDER_VAL(x)				(((x) & 0x1F) << 0)

/* AD9523_CHANNEL_CLOCK_DIST */
#define AD9523_CLK_DIST_DIV_PHASE(x)				(((x) & 0x3F) << 18)
#define AD9523_CLK_DIST_DIV_PHASE_REV(x)			((ret >> 18) & 0x3F)
#define AD9523_CLK_DIST_DIV(x)					((((x) - 1) & 0x3FF) << 8)
#define AD9523_CLK_DIST_DIV_REV(x)				(((ret >> 8) & 0x3FF) + 1)
#define AD9523_CLK_DIST_INV_DIV_OUTPUT_EN			(1 << 7)
#define AD9523_CLK_DIST_IGNORE_SYNC_EN				(1 << 6)
#define AD9523_CLK_DIST_PWR_DOWN_EN				(1 << 5)
#define AD9523_CLK_DIST_LOW_PWR_MODE_EN				(1 << 4)
#define AD9523_CLK_DIST_DRIVER_MODE(x)				(((x) & 0xF) << 0)

/* AD9523_PLL1_OUTPUT_CTRL */
#define AD9523_PLL1_OUTP_CTRL_VCO_DIV_SEL_CH6_M2		(1 << 7)
#define AD9523_PLL1_OUTP_CTRL_VCO_DIV_SEL_CH5_M2		(1 << 6)
#define AD9523_PLL1_OUTP_CTRL_VCO_DIV_SEL_CH4_M2		(1 << 5)
#define AD9523_PLL1_OUTP_CTRL_CMOS_DRV_WEAK			(1 << 4)
#define AD9523_PLL1_OUTP_CTRL_OUTPUT_DIV_1			(0 << 0)
#define AD9523_PLL1_OUTP_CTRL_OUTPUT_DIV_2			(1 << 0)
#define AD9523_PLL1_OUTP_CTRL_OUTPUT_DIV_4			(2 << 0)
#define AD9523_PLL1_OUTP_CTRL_OUTPUT_DIV_8			(4 << 0)
#define AD9523_PLL1_OUTP_CTRL_OUTPUT_DIV_16			(8 << 0)

/* AD9523_PLL1_OUTPUT_CHANNEL_CTRL */
#define AD9523_PLL1_OUTP_CH_CTRL_OUTPUT_PWR_DOWN_EN		(1 << 7)
#define AD9523_PLL1_OUTP_CH_CTRL_VCO_DIV_SEL_CH9_M2		(1 << 6)
#define AD9523_PLL1_OUTP_CH_CTRL_VCO_DIV_SEL_CH8_M2		(1 << 5)
#define AD9523_PLL1_OUTP_CH_CTRL_VCO_DIV_SEL_CH7_M2		(1 << 4)
#define AD9523_PLL1_OUTP_CH_CTRL_VCXO_SRC_SEL_CH3		(1 << 3)
#define AD9523_PLL1_OUTP_CH_CTRL_VCXO_SRC_SEL_CH2		(1 << 2)
#define AD9523_PLL1_OUTP_CH_CTRL_VCXO_SRC_SEL_CH1		(1 << 1)
#define AD9523_PLL1_OUTP_CH_CTRL_VCXO_SRC_SEL_CH0		(1 << 0)

/* AD9523_READBACK_0 */
#define AD9523_READBACK_0_STAT_PLL2_REF_CLK			(1 << 7)
#define AD9523_READBACK_0_STAT_PLL2_FB_CLK			(1 << 6)
#define AD9523_READBACK_0_STAT_VCXO				(1 << 5)
#define AD9523_READBACK_0_STAT_REF_TEST				(1 << 4)
#define AD9523_READBACK_0_STAT_REFB				(1 << 3)
#define AD9523_READBACK_0_STAT_REFA				(1 << 2)
#define AD9523_READBACK_0_STAT_PLL2_LD				(1 << 1)
#define AD9523_READBACK_0_STAT_PLL1_LD				(1 << 0)

/* AD9523_READBACK_1 */
#define AD9523_READBACK_1_HOLDOVER_ACTIVE			(1 << 3)
#define AD9523_READBACK_1_AUTOMODE_SEL_REFB			(1 << 2)
#define AD9523_READBACK_1_VCO_CALIB_IN_PROGRESS			(1 << 0)

/* AD9523_STATUS_SIGNALS */
#define AD9523_STATUS_SIGNALS_SYNC_MAN_CTRL			(1 << 16)
#define AD9523_STATUS_MONITOR_01_PLL12_LOCKED			(0x302)
/* AD9523_POWER_DOWN_CTRL */
#define AD9523_POWER_DOWN_CTRL_PLL1_PWR_DOWN			(1 << 2)
#define AD9523_POWER_DOWN_CTRL_PLL2_PWR_DOWN			(1 << 1)
#define AD9523_POWER_DOWN_CTRL_DIST_PWR_DOWN			(1 << 0)

/* AD9523_IO_UPDATE */
#define AD9523_IO_UPDATE_EN					(1 << 0)

/* AD9523_EEPROM_DATA_XFER_STATUS */
#define AD9523_EEPROM_DATA_XFER_IN_PROGRESS			(1 << 0)

/* AD9523_EEPROM_ERROR_READBACK */
#define AD9523_EEPROM_ERROR_READBACK_FAIL			(1 << 0)

/* AD9523_EEPROM_CTRL1 */
#define AD9523_EEPROM_CTRL1_SOFT_EEPROM				(1 << 1)
#define AD9523_EEPROM_CTRL1_EEPROM_WRITE_PROT_DIS		(1 << 0)

/* AD9523_EEPROM_CTRL2 */
#define AD9523_EEPROM_CTRL2_REG2EEPROM				(1 << 0)

#define AD9523_NUM_CHAN						14
#define AD9523_NUM_CHAN_ALT_CLK_SRC				10

/******************************************************************************/
/************************ Types Definitions ***********************************/
/***************************************************************************//**
 * @brief The `outp_drv_mode` enumeration defines various output driver modes
 * for a device, specifying different electrical characteristics such as
 * current levels and logic families (e.g., LVPECL, LVDS, HSTL, and
 * CMOS). Each enumerator represents a specific configuration that can be
 * used to control the output behavior of the device's driver, allowing
 * for flexibility in interfacing with different types of circuits.
 *
 * @param TRISTATE Represents a high-impedance state for the output driver.
 * @param LVPECL_8mA Specifies an LVPECL output driver mode with 8mA current.
 * @param LVDS_4mA Specifies an LVDS output driver mode with 4mA current.
 * @param LVDS_7mA Specifies an LVDS output driver mode with 7mA current.
 * @param HSTL0_16mA Specifies an HSTL output driver mode with 16mA current.
 * @param HSTL1_8mA Specifies an HSTL output driver mode with 8mA current.
 * @param CMOS_CONF1 Specifies a CMOS configuration mode 1 for the output
 * driver.
 * @param CMOS_CONF2 Specifies a CMOS configuration mode 2 for the output
 * driver.
 * @param CMOS_CONF3 Specifies a CMOS configuration mode 3 for the output
 * driver.
 * @param CMOS_CONF4 Specifies a CMOS configuration mode 4 for the output
 * driver.
 * @param CMOS_CONF5 Specifies a CMOS configuration mode 5 for the output
 * driver.
 * @param CMOS_CONF6 Specifies a CMOS configuration mode 6 for the output
 * driver.
 * @param CMOS_CONF7 Specifies a CMOS configuration mode 7 for the output
 * driver.
 * @param CMOS_CONF8 Specifies a CMOS configuration mode 8 for the output
 * driver.
 * @param CMOS_CONF9 Specifies a CMOS configuration mode 9 for the output
 * driver.
 ******************************************************************************/
enum outp_drv_mode {
	TRISTATE,
	LVPECL_8mA,
	LVDS_4mA,
	LVDS_7mA,
	HSTL0_16mA,
	HSTL1_8mA,
	CMOS_CONF1,
	CMOS_CONF2,
	CMOS_CONF3,
	CMOS_CONF4,
	CMOS_CONF5,
	CMOS_CONF6,
	CMOS_CONF7,
	CMOS_CONF8,
	CMOS_CONF9
};

/***************************************************************************//**
 * @brief The `ref_sel_mode` enumeration defines various modes for selecting
 * reference signals in a system, such as staying on a specific
 * reference, reverting to another, or selecting an external reference.
 * This is typically used in systems that require precise control over
 * reference signal selection, such as clock distribution systems.
 *
 * @param NONEREVERTIVE_STAY_ON_REFB Indicates a non-revertive mode where the
 * system stays on reference B.
 * @param REVERT_TO_REFA Indicates a mode where the system reverts to reference
 * A.
 * @param SELECT_REFA Indicates a mode where reference A is selected.
 * @param SELECT_REFB Indicates a mode where reference B is selected.
 * @param EXT_REF_SEL Indicates a mode where an external reference is selected.
 ******************************************************************************/
enum ref_sel_mode {
	NONEREVERTIVE_STAY_ON_REFB,
	REVERT_TO_REFA,
	SELECT_REFA,
	SELECT_REFB,
	EXT_REF_SEL
};

/***************************************************************************//**
 * @brief The `ad9523_channel_spec` structure defines the configuration for an
 * output channel in the AD9523 device. It includes settings for the
 * channel number, clock polarity inversion, SYNC signal handling, power
 * mode, alternative clock source usage, channel power state, driver
 * mode, initial phase of the divider, and the channel divider value.
 * Additionally, it allows for an optional descriptive name for the
 * channel. This structure is crucial for configuring the behavior and
 * characteristics of each output channel in the AD9523 clock
 * distribution system.
 *
 * @param channel_num Output channel number.
 * @param divider_output_invert_en Invert the polarity of the output clock.
 * @param sync_ignore_en Ignore chip-level SYNC signal.
 * @param low_power_mode_en Reduce power used in the differential output modes.
 * @param use_alt_clock_src Channel divider uses alternative clock source:
 * CH0..CH3 VCXO, CH4..CH9 VCO2.
 * @param output_dis Disables, powers down the entire channel.
 * @param driver_mode Output driver mode (logic level family).
 * @param divider_phase Divider initial phase after a SYNC, range 0..63, LSB =
 * 1/2 of a period of the divider input clock.
 * @param channel_divider 10-bit channel divider.
 * @param extended_name Optional descriptive channel name.
 ******************************************************************************/
struct ad9523_channel_spec {
	/** Output channel number. */
	uint8_t channel_num;
	/** Invert the polarity of the output clock. */
	uint8_t divider_output_invert_en;
	/** Ignore chip-level SYNC signal. */
	uint8_t sync_ignore_en;
	/** Reduce power used in the differential output modes. */
	uint8_t low_power_mode_en;
	/** Channel divider uses alternative clk source: CH0..CH3 VCXO, CH4..CH9 VCO2 */
	uint8_t use_alt_clock_src;
	/**  Disables, powers down the entire channel. */
	uint8_t output_dis;
	/** Output driver mode (logic level family). */
	uint8_t driver_mode;
	/** Divider initial phase after a SYNC. Range 0..63
	 * LSB = 1/2 of a period of the divider input clock.
	 */
	uint8_t divider_phase;
	/** 10-bit channel divider. */
	uint16_t channel_divider;
	/** Optional descriptive channel name. */
	int8_t extended_name[16];
};

/***************************************************************************//**
 * @brief The `pll1_rzero_resistor` enumeration defines a set of constants
 * representing different resistor values used in the loop filter of the
 * PLL1 (Phase-Locked Loop) in the AD9523 device. These resistor values
 * are critical for configuring the loop filter characteristics, which in
 * turn affect the stability and performance of the PLL. The enumeration
 * provides predefined resistor values, as well as an option to use an
 * external resistor, allowing for flexibility in the design and tuning
 * of the PLL loop filter.
 *
 * @param RZERO_883_OHM Represents a resistor value of 883 ohms.
 * @param RZERO_677_OHM Represents a resistor value of 677 ohms.
 * @param RZERO_341_OHM Represents a resistor value of 341 ohms.
 * @param RZERO_135_OHM Represents a resistor value of 135 ohms.
 * @param RZERO_10_OHM Represents a resistor value of 10 ohms.
 * @param RZERO_USE_EXT_RES Indicates the use of an external resistor, with a
 * specific value of 8.
 ******************************************************************************/
enum pll1_rzero_resistor {
	RZERO_883_OHM,
	RZERO_677_OHM,
	RZERO_341_OHM,
	RZERO_135_OHM,
	RZERO_10_OHM,
	RZERO_USE_EXT_RES = 8,
};

/***************************************************************************//**
 * @brief The `rpole2_resistor` enumeration defines a set of constants
 * representing specific resistor values used in the loop filter of the
 * AD9523 PLL2 configuration. These values are used to configure the
 * resistance in the loop filter, which is critical for determining the
 * stability and performance of the phase-locked loop (PLL). Each
 * enumerator corresponds to a specific resistance value in ohms,
 * allowing for easy selection and configuration within the software.
 *
 * @param RPOLE2_900_OHM Represents a resistor value of 900 ohms.
 * @param RPOLE2_450_OHM Represents a resistor value of 450 ohms.
 * @param RPOLE2_300_OHM Represents a resistor value of 300 ohms.
 * @param RPOLE2_225_OHM Represents a resistor value of 225 ohms.
 ******************************************************************************/
enum rpole2_resistor {
	RPOLE2_900_OHM,
	RPOLE2_450_OHM,
	RPOLE2_300_OHM,
	RPOLE2_225_OHM,
};

/***************************************************************************//**
 * @brief The `rzero_resistor` enum defines a set of constants representing
 * different resistor values in ohms, which are used to configure the
 * loop filter of a PLL (Phase-Locked Loop) in the AD9523 device. Each
 * enumerator corresponds to a specific resistance value, allowing for
 * easy selection and configuration of the loop filter's characteristics.
 *
 * @param RZERO_3250_OHM Represents a resistor with a resistance of 3250 ohms.
 * @param RZERO_2750_OHM Represents a resistor with a resistance of 2750 ohms.
 * @param RZERO_2250_OHM Represents a resistor with a resistance of 2250 ohms.
 * @param RZERO_2100_OHM Represents a resistor with a resistance of 2100 ohms.
 * @param RZERO_3000_OHM Represents a resistor with a resistance of 3000 ohms.
 * @param RZERO_2500_OHM Represents a resistor with a resistance of 2500 ohms.
 * @param RZERO_2000_OHM Represents a resistor with a resistance of 2000 ohms.
 * @param RZERO_1850_OHM Represents a resistor with a resistance of 1850 ohms.
 ******************************************************************************/
enum rzero_resistor {
	RZERO_3250_OHM,
	RZERO_2750_OHM,
	RZERO_2250_OHM,
	RZERO_2100_OHM,
	RZERO_3000_OHM,
	RZERO_2500_OHM,
	RZERO_2000_OHM,
	RZERO_1850_OHM,
};

/***************************************************************************//**
 * @brief The `cpole1_capacitor` enumeration defines a set of constants
 * representing different capacitor values in picofarads, which are used
 * in configuring the loop filter of the AD9523 PLL2. Each enumerator
 * corresponds to a specific capacitor value, allowing for easy selection
 * and configuration of the loop filter's characteristics. The
 * placeholder `_CPOLE1_24_PF` is included for potential future use or as
 * a reserved value.
 *
 * @param CPOLE1_0_PF Represents a capacitor value of 0 picofarads.
 * @param CPOLE1_8_PF Represents a capacitor value of 8 picofarads.
 * @param CPOLE1_16_PF Represents a capacitor value of 16 picofarads.
 * @param CPOLE1_24_PF Represents a capacitor value of 24 picofarads.
 * @param _CPOLE1_24_PF A placeholder for a capacitor value of 24 picofarads.
 * @param CPOLE1_32_PF Represents a capacitor value of 32 picofarads.
 * @param CPOLE1_40_PF Represents a capacitor value of 40 picofarads.
 * @param CPOLE1_48_PF Represents a capacitor value of 48 picofarads.
 ******************************************************************************/
enum cpole1_capacitor {
	CPOLE1_0_PF,
	CPOLE1_8_PF,
	CPOLE1_16_PF,
	CPOLE1_24_PF,
	_CPOLE1_24_PF, /* place holder */
	CPOLE1_32_PF,
	CPOLE1_40_PF,
	CPOLE1_48_PF,
};

/***************************************************************************//**
 * @brief The `ad9523_platform_data` structure is a comprehensive configuration
 * data structure for the AD9523 device, which is a clock generator and
 * distribution IC. It includes settings for external VCXO frequency, SPI
 * communication mode, input selection for various differential and
 * single-ended signals, and detailed PLL1 and PLL2 configuration
 * parameters such as dividers, charge pump currents, and loop filter
 * settings. Additionally, it holds information about the output channel
 * configuration, including the number of channels and a pointer to the
 * channel specifications. This structure is essential for initializing
 * and configuring the AD9523 device to meet specific application
 * requirements.
 *
 * @param vcxo_freq External VCXO frequency in Hz.
 * @param spi3wire Enable SPI-3wire mode.
 * @param refa_diff_rcv_en REFA differential/single-ended input selection.
 * @param refb_diff_rcv_en REFB differential/single-ended input selection.
 * @param zd_in_diff_en Zero Delay differential/single-ended input selection.
 * @param osc_in_diff_en OSC differential/single-ended input selection.
 * @param refa_cmos_neg_inp_en REFA single-ended neg./pos. input enable.
 * @param refb_cmos_neg_inp_en REFB single-ended neg./pos. input enable.
 * @param zd_in_cmos_neg_inp_en Zero Delay single-ended neg./pos. input enable.
 * @param osc_in_cmos_neg_inp_en OSC single-ended neg./pos. input enable.
 * @param refa_r_div PLL1 10-bit: REFA R divider.
 * @param refb_r_div PLL1 10-bit: REFB R divider.
 * @param pll1_feedback_div PLL1 10-bit Feedback N divider.
 * @param pll1_charge_pump_current_nA Magnitude of PLL1 charge pump current in
 * nA.
 * @param zero_delay_mode_internal_en Internal, external Zero Delay mode
 * selection.
 * @param osc_in_feedback_en PLL1 feedback path, local feedback from the OSC_IN
 * receiver or zero delay mode.
 * @param pll1_bypass_en Bypass PLL1 - Single loop mode.
 * @param pll1_loop_filter_rzero PLL1 Loop Filter Zero Resistor selection.
 * @param ref_mode Reference mode selection.
 * @param pll2_charge_pump_current_nA Magnitude of PLL2 charge pump current in
 * nA.
 * @param pll2_ndiv_a_cnt PLL2 Feedback N-divider, A Counter, range 0..4.
 * @param pll2_ndiv_b_cnt PLL2 Feedback N-divider, B Counter, range 0..63.
 * @param pll2_freq_doubler_en PLL2 frequency doubler enable.
 * @param pll2_r2_div PLL2 R2 divider, range 0..31.
 * @param pll2_vco_diff_m1 VCO1 divider, range 3..5.
 * @param pll2_vco_diff_m2 VCO2 divider, range 3..5.
 * @param rpole2 PLL2 loop filter Rpole resistor value.
 * @param rzero PLL2 loop filter Rzero resistor value.
 * @param cpole1 PLL2 loop filter Cpole capacitor value.
 * @param rzero_bypass_en PLL2 loop filter Rzero bypass enable.
 * @param num_channels Array size of struct ad9523_channel_spec.
 * @param channels Pointer to channel array.
 * @param name Optional alternative iio device name.
 ******************************************************************************/
struct ad9523_platform_data {
	/** External VCXO frequency in Hz */
	uint32_t vcxo_freq;
	/** Enable SPI-3wire mode */
	uint8_t spi3wire;

	/** REFA differential/single-ended input selection. */
	uint8_t refa_diff_rcv_en;
	/** REFB differential/single-ended input selection. */
	uint8_t refb_diff_rcv_en;
	/** Zero Delay differential/single-ended input selection. */
	uint8_t zd_in_diff_en;
	/** OSC differential/ single-ended input selection. */
	uint8_t osc_in_diff_en;

	/*
	 * Valid if differential input disabled
	 * if not true defaults to pos input
	 */
	/** REFA single-ended neg./pos. input enable. */
	uint8_t refa_cmos_neg_inp_en;
	/* REFB single-ended neg./pos. input enable. */
	uint8_t refb_cmos_neg_inp_en;
	/** Zero Delay single-ended neg./pos. input enable. */
	uint8_t zd_in_cmos_neg_inp_en;
	/** OSC single-ended neg./pos. input enable. */
	uint8_t osc_in_cmos_neg_inp_en;

	/* PLL1 Setting */
	/** PLL1 10-bit: REFA R divider. */
	uint16_t refa_r_div;
	/** PLL1 10-bit: REFB R divider. */
	uint16_t refb_r_div;
	/**  PLL1 10-bit Feedback N divider. */
	uint16_t pll1_feedback_div;
	/** Magnitude of PLL1 charge pump current (nA). */
	uint16_t pll1_charge_pump_current_nA;
	/** Internal, external Zero Delay mode selection. */
	uint8_t zero_delay_mode_internal_en;
	/**  PLL1 feedback path, local feedback from the
	 * OSC_IN receiver or zero delay mode
	 */
	uint8_t osc_in_feedback_en;

	/** Bypass PLL1 - Single loop mode */
	uint8_t pll1_bypass_en;
	/** PLL1 Loop Filter Zero Resistor selection. */
	uint8_t pll1_loop_filter_rzero;

	/** Reference mode selection. */
	uint8_t ref_mode;

	/* PLL2 Setting */
	/** Magnitude of PLL2 charge pump current (nA). */
	uint32_t pll2_charge_pump_current_nA;
	/** PLL2 Feedback N-divider, A Counter, range 0..4. */
	uint8_t pll2_ndiv_a_cnt;
	/** PLL2 Feedback N-divider, B Counter, range 0..63. */
	uint8_t pll2_ndiv_b_cnt;
	/** PLL2 frequency doubler enable. */
	uint8_t pll2_freq_doubler_en;
	/** PLL2 R2 divider, range 0..31. */
	uint8_t pll2_r2_div;
	/** VCO1 divider, range 3..5. */
	uint8_t pll2_vco_diff_m1;
	/** VCO2 divider, range 3..5. */
	uint8_t pll2_vco_diff_m2;

	/* Loop Filter PLL2 */
	/** PLL2 loop filter Rpole resistor value. */
	uint8_t rpole2;
	/** PLL2 loop filter Rzero resistor value. */
	uint8_t rzero;
	/**  PLL2 loop filter Cpole capacitor value. */
	uint8_t cpole1;
	/**  PLL2 loop filter Rzero bypass enable. */
	uint8_t rzero_bypass_en;

	/* Output Channel Configuration */
	/**  Array size of struct ad9523_channel_spec. */
	int32_t num_channels;
	/** Pointer to channel array. */
	struct ad9523_channel_spec *channels;

	/** Optional alternative iio device name. */
	int8_t name[16];
};

/***************************************************************************//**
 * @brief The `ad9523_state` structure is used to maintain the state of the
 * AD9523 device, which is a clock generator and distribution IC. It
 * holds configuration data such as the frequencies of the VCXO and VCO,
 * as well as mappings of VCO outputs to channels. This structure is
 * essential for managing the device's operation and ensuring that the
 * correct frequencies are output to the appropriate channels.
 *
 * @param pdata Pointer to a structure containing platform-specific data for the
 * AD9523.
 * @param vcxo_freq Frequency of the external VCXO in Hz.
 * @param vco_freq Frequency of the VCO in Hz.
 * @param vco_out_freq Array holding the output frequencies of the VCO.
 * @param vco_out_map Array mapping VCO outputs to specific channels.
 ******************************************************************************/
struct ad9523_state {
	struct ad9523_platform_data *pdata;
	uint32_t vcxo_freq;
	uint32_t vco_freq;
	uint32_t vco_out_freq[3];
	uint8_t vco_out_map[14];
};

/***************************************************************************//**
 * @brief The `ad9523_out_frequencies` enumeration defines the possible output
 * frequency sources for the AD9523 device, which include two VCOs and a
 * VCXO. This enumeration is used to specify which clock source is being
 * referred to in the context of configuring or querying the AD9523 clock
 * distribution device.
 *
 * @param AD9523_VCO1 Represents the first VCO (Voltage Controlled Oscillator)
 * output frequency.
 * @param AD9523_VCO2 Represents the second VCO output frequency.
 * @param AD9523_VCXO Represents the VCXO (Voltage Controlled Crystal
 * Oscillator) output frequency.
 * @param AD9523_NUM_CLK_SRC Indicates the number of clock sources available.
 ******************************************************************************/
enum ad9523_out_frequencies {
	AD9523_VCO1,
	AD9523_VCO2,
	AD9523_VCXO,
	AD9523_NUM_CLK_SRC,
};

/***************************************************************************//**
 * @brief The `ad9523_dev` structure is a compound data type used to encapsulate
 * the necessary components for managing and interfacing with an AD9523
 * device. It includes a SPI descriptor for communication, a state
 * structure to maintain the current configuration and operational state
 * of the device, and a pointer to platform-specific data that provides
 * additional configuration details. This structure is essential for
 * initializing, configuring, and controlling the AD9523 device within a
 * software application.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param ad9523_st Holds the state information of the AD9523 device.
 * @param pdata Pointer to platform-specific data for the AD9523 device.
 ******************************************************************************/
struct ad9523_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* Device Settings */
	struct ad9523_state		ad9523_st;
	struct ad9523_platform_data	*pdata;
};

/***************************************************************************//**
 * @brief The `ad9523_init_param` structure is used to initialize the AD9523
 * device, encapsulating both the SPI initialization parameters and a
 * pointer to the platform-specific data settings. This structure is
 * essential for setting up the device's communication interface and
 * configuring its operational parameters according to the specific
 * platform requirements.
 *
 * @param spi_init Holds the initialization parameters for the SPI interface.
 * @param pdata Pointer to the platform-specific data structure containing
 * device settings.
 ******************************************************************************/
struct ad9523_init_param {
	/* SPI */
	struct no_os_spi_init_param	spi_init;
	/* Device Settings */
	struct ad9523_platform_data	*pdata;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Reads the value of the selected register. */
/***************************************************************************//**
 * @brief Use this function to read the value of a specific register from the
 * AD9523 device. It requires a valid device structure and a register
 * address to read from. The function will store the read value in the
 * provided output parameter. Ensure that the device has been properly
 * initialized before calling this function. The function returns an
 * error code if the SPI communication fails.
 *
 * @param dev A pointer to an ad9523_dev structure representing the device. Must
 * not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to read from. It should be a
 * valid register address within the device's addressable range.
 * @param reg_data A pointer to a uint32_t where the read register value will be
 * stored. Must not be null.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error in SPI communication.
 ******************************************************************************/
int32_t ad9523_spi_read(struct ad9523_dev *dev,
			uint32_t reg_addr,
			uint32_t *reg_data);

/* Writes a value to the selected register. */
/***************************************************************************//**
 * @brief Use this function to write a 32-bit value to a specific register of
 * the AD9523 device via SPI. This function is typically called when
 * configuring the device or updating its settings. Ensure that the
 * device has been properly initialized and that the SPI interface is
 * correctly set up before calling this function. The function returns an
 * error code if the SPI write operation fails.
 *
 * @param dev A pointer to an ad9523_dev structure representing the device. Must
 * not be null, and the device must be initialized.
 * @param reg_addr The address of the register to write to. Must be a valid
 * register address for the AD9523.
 * @param reg_data The 32-bit data to write to the specified register. The data
 * is split into bytes and sent over SPI.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error during the SPI write operation.
 ******************************************************************************/
int32_t ad9523_spi_write(struct ad9523_dev *dev,
			 uint32_t reg_addr,
			 uint32_t reg_data);

/* Updates the AD9523 configuration */
/***************************************************************************//**
 * @brief Use this function to apply any pending configuration changes to the
 * AD9523 device. It must be called after making configuration changes
 * that require an update to take effect. This function communicates with
 * the device via SPI to ensure the new settings are applied. It is
 * essential to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized ad9523_dev structure representing the
 * device. Must not be null. The function will return an error if the
 * device is not properly initialized.
 * @return Returns an int32_t value indicating success or failure of the update
 * operation. A non-zero return value indicates an error.
 ******************************************************************************/
int32_t ad9523_io_update(struct ad9523_dev *dev);

/* Sets the clock provider for selected channel. */
/***************************************************************************//**
 * @brief This function configures the clock source for a specified output
 * channel on the AD9523 device. It should be used when you need to
 * change the clock source for a channel, either to the VCO or another
 * source. The function must be called with a valid device structure and
 * a channel number within the range of 0 to 9. The output parameter
 * determines the clock source, where a non-zero value selects the VCO.
 * The function returns an error code if the operation fails, and it
 * updates the internal state of the device to reflect the new
 * configuration.
 *
 * @param dev A pointer to an initialized ad9523_dev structure. Must not be
 * null. The caller retains ownership.
 * @param ch The channel number to configure, ranging from 0 to 9. Values
 * outside this range result in no operation.
 * @param out Determines the clock source for the channel. A non-zero value
 * selects the VCO as the clock source, while zero selects an
 * alternative source.
 * @return Returns an int32_t error code, where 0 indicates success and a
 * negative value indicates an error. The internal state of the device
 * is updated to reflect the new clock source configuration.
 ******************************************************************************/
int32_t ad9523_vco_out_map(struct ad9523_dev *dev,
			   uint32_t ch,
			   uint32_t out);

/* Updates the AD9523 configuration. */
/***************************************************************************//**
 * @brief This function is used to synchronize the AD9523 device by controlling
 * the synchronization signal. It should be called when a synchronization
 * of the device is required, such as after configuration changes. The
 * function reads the current status signals, modifies the
 * synchronization control bit, writes it back, and updates the device
 * configuration. It is important to ensure that the device is properly
 * initialized before calling this function to avoid undefined behavior.
 *
 * @param dev A pointer to an ad9523_dev structure representing the device. Must
 * not be null. The caller retains ownership and is responsible for
 * ensuring the device is initialized.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during synchronization.
 ******************************************************************************/
int32_t ad9523_sync(struct ad9523_dev *dev);

/* Initialize the AD9523 data structure*/
/***************************************************************************//**
 * @brief This function initializes the AD9523 device by setting up the provided
 * initialization parameters with default values. It is typically called
 * before any other operations on the AD9523 to ensure that the device is
 * in a known state. The function does not perform any hardware
 * communication; it only prepares the initialization structure. It must
 * be called before using the `ad9523_setup` function to configure the
 * device.
 *
 * @param init_param A pointer to an `ad9523_init_param` structure that must not
 * be null. The structure should be allocated by the caller,
 * and the function will populate its `pdata` field with
 * default configuration values. The caller retains ownership
 * of the memory.
 * @return Returns 0 on successful initialization of the parameters.
 ******************************************************************************/
int32_t ad9523_init(struct ad9523_init_param *init_param);

/* Configure the AD9523. */
/***************************************************************************//**
 * @brief This function sets up the AD9523 device by initializing its SPI
 * interface and configuring its PLLs and output channels based on the
 * provided initialization parameters. It must be called before any other
 * operations on the AD9523 device. The function allocates memory for the
 * device structure and initializes it with the specified parameters. It
 * handles various configuration settings, including PLL dividers, charge
 * pump currents, and channel configurations. The function returns an
 * error code if initialization fails at any step, ensuring that the
 * device is not left in an undefined state.
 *
 * @param device A pointer to a pointer of type `struct ad9523_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a constant `struct ad9523_init_param`
 * containing the initialization parameters for the device.
 * This includes SPI settings and platform-specific data. The
 * pointer must not be null, and the structure must be fully
 * populated with valid configuration data.
 * @return Returns an `int32_t` status code. A value of 0 indicates success,
 * while a negative value indicates an error occurred during setup.
 ******************************************************************************/
int32_t ad9523_setup(struct ad9523_dev **device,
		     const struct ad9523_init_param *init_param);

/* Free the resources allocated by ad9523_setup(). */
/***************************************************************************//**
 * @brief This function is used to release all resources associated with an
 * AD9523 device instance. It should be called when the device is no
 * longer needed, typically after all operations with the device are
 * complete. The function ensures that any allocated memory and
 * associated SPI resources are properly freed. It is important to ensure
 * that the `dev` parameter is a valid pointer to an initialized
 * `ad9523_dev` structure before calling this function. Failure to do so
 * may result in undefined behavior.
 *
 * @param dev A pointer to an `ad9523_dev` structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid, initialized device structure. The function will handle
 * invalid pointers by potentially causing undefined behavior.
 * @return Returns an integer status code from the SPI removal operation, where
 * 0 typically indicates success and a negative value indicates an
 * error.
 ******************************************************************************/
int32_t ad9523_remove(struct ad9523_dev *dev);

/***************************************************************************//**
 * @brief This function is used to verify the operational status of the AD9523
 * device by checking various status indicators. It should be called to
 * ensure that the device is functioning correctly, particularly after
 * initialization or configuration changes. The function checks the
 * status of the VCXO and PLL2 lock, and if PLL1 is not bypassed, it also
 * checks additional status indicators. It returns an error code if any
 * of the critical status indicators are not in the expected state. This
 * function must be called with a valid device structure that has been
 * properly initialized.
 *
 * @param dev A pointer to an ad9523_dev structure representing the device. This
 * must be a valid, initialized device structure. The function will
 * not modify this structure, but it must not be null.
 * @return Returns 0 if the device status is as expected, or -1 if there are
 * errors in the VCXO or PLL2 lock status.
 ******************************************************************************/
int32_t ad9523_status(struct ad9523_dev *dev);
#endif // __AD9523_H__
