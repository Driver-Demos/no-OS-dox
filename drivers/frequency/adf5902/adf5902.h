/***************************************************************************//**
 *   @file   adf5902.h
 *   @brief  Header file for adf5902 Driver.
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

#ifndef SRC_ADF5902_H_
#define SRC_ADF5902_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Registers Control Bits */
#define ADF5902_REG0			0x0
#define ADF5902_REG1			0x1
#define ADF5902_REG2			0x2
#define ADF5902_REG3			0x3
#define ADF5902_REG4			0x4
#define ADF5902_REG5			0x5
#define ADF5902_REG6			0x6
#define ADF5902_REG7			0x7
#define ADF5902_REG8			0x8
#define ADF5902_REG9			0x9
#define ADF5902_REG10			0xA
#define ADF5902_REG11			0xB
#define ADF5902_REG12			0xC
#define ADF5902_REG13			0xD
#define ADF5902_REG14			0xE
#define ADF5902_REG15			0xF
#define ADF5902_REG16			0x10
#define ADF5902_REG17			0x11

/* Register 0 Map */
#define ADF5902_REG0_PLO(x)		(((x) & 0x1) << 5)
#define ADF5902_REG0_PTX1(x)		(((x) & 0x1) << 6)
#define ADF5902_REG0_PTX2(x)		(((x) & 0x1) << 7)
#define ADF5902_REG0_PADC(x)		(((x) & 0x1) << 8)
#define ADF5902_REG0_VCAL(x)		(((x) & 0x1) << 9)
#define ADF5902_REG0_PVCO(x)		(((x) & 0x1) << 10)
#define ADF5902_REG0_TX1C(x)		(((x) & 0x1) << 11)
#define ADF5902_REG0_TX2C(x)		(((x) & 0x1) << 12)
#define ADF5902_REG0_RESERVED		(0x4007FU << 13)

/* Register 0 Bit Definitions */
#define ADF5902_POWER_DOWN_LO		0x0
#define ADF5902_POWER_UP_LO		0x1

#define ADF5902_POWER_DOWN_TX1		0x0
#define ADF5902_POWER_UP_TX1		0x1

#define ADF5902_POWER_DOWN_TX2		0x0
#define ADF5902_POWER_UP_TX2		0x1

#define ADF5902_POWER_DOWN_ADC		0x0
#define ADF5902_POWER_UP_ADC		0x1

#define ADF5902_VCO_NORMAL_OP		0x0
#define ADF5902_VCO_FULL_CAL		0x1

#define ADF5902_POWER_DOWN_VCO		0x0
#define ADF5902_POWER_UP_VCO		0x1

#define ADF5902_TX1_NORMAL_OP		0x0
#define ADF5902_TX1_AMP_CAL		0x1

#define ADF5902_TX2_NORMAL_OP		0x0
#define ADF5902_TX2_AMP_CAL		0x1

/* Register 1 Map */
#define ADF5902_REG1_TX_AMP_CAL_REF(x)	(((x) & 0xFF) << 5)
#define ADF5902_REG1_RESERVED		(0x7FFBFU << 13)

/* Register 1 Bit Definitions */
#define ADF5902_TX_AMP_CAL_MIN_REF_CODE	0x00
#define ADF5902_TX_AMP_CAL_MAX_REF_CODE	0xFF

/* Register 2 Map */
#define ADF5902_REG2_ADC_CLK_DIV(x)	(((x) & 0xFF) << 5)
#define ADF5902_REG2_ADC_AVG(x)		(((x) & 0x3) << 13)
#define ADF5902_REG2_ADC_START(x)	(((x) & 0x1) << 15)
#define ADF5902_REG2_RESERVED		(0x2 << 16)

/* Register 2 Bit Definitions */
#define ADF5902_ADC_MIN_CLK_DIVIDER	0x1
#define ADF5902_ADC_MAX_CLK_DIVIDER	0x7F

#define ADF5902_ADC_AVG_1		0x0
#define ADF5902_ADC_AVG_2		0x1
#define ADF5902_ADC_AVG_3		0x2
#define ADF5902_ADC_AVG_4		0x3

#define ADF5902_ADC_NORMAL_OP		0x0
#define ADF5902_START_ADC_CONV		0x1

/* Register 3 Map */
#define ADF5902_REG3_READBACK_CTRL(x)	(((x) & 0x3F) << 5)
#define ADF5902_REG3_IO_LVL(x)		(((x) & 0x1) << 11)
#define ADF5902_REG3_MUXOUT(x)		(((x) & 0xF) << 12)
#define ADF5902_REG3_RESERVED		(0x189 << 16)

/* Register 3 Bit Definitions */
#define ADF5902_REG_RB_NONE		0x0
#define ADF5902_REG0_RB			0x1
#define ADF5902_REG1_RB			0x2
#define ADF5902_REG2_RB			0x3
#define ADF5902_REG3_RB			0x4
#define ADF5902_REG4_RB			0x5
#define ADF5902_REG5_RB			0x6
#define ADF5902_REG6_RB			0x7
#define ADF5902_REG7_RB			0x8
#define ADF5902_REG8_RB			0x9
#define ADF5902_REG9_RB			0xA
#define ADF5902_REG10_RB		0xB
#define ADF5902_REG11_RB		0xC
#define ADF5902_REG12_RB		0xD
#define ADF5902_REG13_SEL_0_RB		0xE
#define ADF5902_REG14_SEL_0_RB		0xF
#define ADF5902_REG15_SEL_0_RB		0x10
#define ADF5902_REG16_SEL_0_RB		0x11
#define ADF5902_REG17_RB		0x12
#define ADF5902_ADC_RB			0x16
#define ADF5902_FREQ_RB			0x1A
#define ADF5902_REG13_SEL_1_RB		0x33
#define ADF5902_REG14_SEL_1_RB		0x34
#define ADF5902_REG15_SEL_1_RB		0x35
#define ADF5902_REG16_SEL_1_RB		0x36
#define ADF5902_REG13_SEL_2_RB		0x37
#define ADF5902_REG14_SEL_2_RB		0x38
#define ADF5902_REG15_SEL_2_RB		0x39
#define ADF5902_REG16_SEL_2_RB		0x3A
#define ADF5902_REG13_SEL_3_RB		0x3B
#define ADF5902_REG14_SEL_3_RB		0x3C
#define ADF5902_REG15_SEL_3_RB		0x3D
#define ADF5902_REG16_SEL_3_RB		0x3F

#define ADF5902_IO_LVL_1V8		0x0
#define ADF5902_IO_LVL_3V3		0x1

#define ADF5902_MUXOUT_TRISTATE_OUT	0x0
#define ADF5902_MUXOUT_LOGIC_HIGH	0x1
#define ADF5902_MUXOUT_LOGIC_LOW	0x2
#define ADF5902_MUXOUT_R_DIV_OUT	0x3
#define ADF5902_MUXOUT_N_DIV_OUT	0x4
#define ADF5902_MUXOUT_CAL_BUSY		0x7
#define ADF5902_MUXOUT_R_DIV_OUT_2	0xB
#define ADF5902_MUXOUT_N_DIV_OUT_2	0xC
#define ADF5902_MUXOUT_RAMP_STATUS	0xF

/* Register 4 Map */
#define ADF5902_REG4_TEST_BUS(x)	(((x) & 0x7FFF) << 5)
#define ADF5902_REG4_RESERVED		(0x0 << 16)

/* Register 4 Bit Definitions */
#define ADF5902_TEST_BUS_NONE		0x0000
#define ADF5902_RAMP_COMPL_TO_MUXOUT	0x00C0
#define ADF5902_RAMP_DOWN_TO_MUXOUT	0x0100
#define ADF5902_TEMP_SENS_TO_ATEST	0x0503
#define ADF5902_TEMP_SENS_TO_ADC	0x0903

/* Register 5 Map */
#define ADF5902_REG5_FRAC_MSB_WORD(x)	(((x) & 0xFFF) << 5)
#define ADF5902_REG5_INTEGER_WORD(x)	(((x) & 0xFFF) << 17)
#define ADF5902_REG5_RAMP_ON(x)		(((x) & 0x1) << 29)
#define ADF5902_REG5_RESERVED		(0x0 << 30)

/* Register 5 Bit Definitions */
#define ADF5902_MIN_FRAC_MSB_WORD	0x000
#define ADF5902_MAX_FRAC_MSB_WORD	0xFFF

#define ADF5902_MIN_INT_MSB_WORD	0x000
#define ADF5902_MAX_INT_MSB_WORD	0xFFF

#define ADF5902_RAMP_ON_DISABLED	0x0
#define ADF5902_RAMP_ON_ENABLED		0x1

/* Register 6 Map */
#define ADF5902_REG6_FRAC_LSB_WORD(x)	(((x) & 0x1FFF) << 5)
#define ADF5902_REG6_RESERVED		(0x0 << 18)

/* Register 6 Bit Definitions */
#define ADF5902_MIN_FRAC_LSB_WORD	0x000
#define ADF5902_MAX_FRAC_LSB_WORD	0x1FFF

/* Register 7 Map */
#define ADF5902_REG7_R_DIVIDER(x)	(((x) & 0x1F) << 5)
#define ADF5902_REG7_REF_DOUBLER(x)	(((x) & 0x1) << 10)
#define ADF5902_REG7_R_DIV_2(x)		(((x) & 0x1) << 11)
#define ADF5902_REG7_CLK_DIV(x)		(((x) & 0xFFF) << 12)
#define ADF5902_REG7_MASTER_RESET(x)	(((x) & 0x1) << 25)
#define ADF5902_REG7_RESERVED		((0x0 << 26) | (0x1 << 24))

/* Register 7 Bit Definitions */
#define ADF5902_MIN_R_DIVIDER		0x01
#define ADF5902_MAX_R_DIVIDER		0x1F

#define ADF5902_R_DIV_2_DISABLE		0x0
#define ADF5902_R_DIV_2_ENABLE		0x1

#define ADF5902_REF_DOUBLER_DISABLE	0x0
#define ADF5902_REF_DOUBLER_ENABLE	0x1

#define ADF5902_MIN_CLK_DIVIDER		0x000
#define ADF5902_MAX_CLK_DIVIDER		0xFFF

#define ADF5902_MASTER_RESET_DISABLE	0x0
#define ADF5902_MASTER_RESET_ENABLE	0x1

/* Register 8 Map */
#define ADF5902_REG8_FREQ_CAL_DIV(x)	(((x) & 0x3FF) << 5)
#define ADF5902_REG8_RESERVED		(0x8000 << 15)

/* Register 8 Bit Definitions */
#define ADF5902_MIN_FREQ_CAL_DIV	0x000
#define ADF5902_MAX_FREQ_CAL_DIV	0x3FF

/* Register 9 Map */
#define ADF5902_REG9_RESERVED_CALIB	(0x15105C9 << 5)
#define ADF5902_REG9_RESERVED_NORMAL	(0x14005C9 << 5)

/* Register 10 Map */
#define ADF5902_REG10_RESERVED		(0xE99532 << 5)

/* Register 11 Map */
#define ADF5902_REG11_CNTR_RESET(x)	(((x) & 0x1) << 5)
#define ADF5902_REG11_RAMP_MODE(x)	(((x) & 0x3) << 7)
#define ADF5902_REG11_SING_FULL_TRI(x)	(((x) & 0x1) << 9)
#define ADF5902_REG11_SD_RESET(x)	(((x) & 0x1) << 11)
#define ADF5902_REG11_RESERVED		((0x0 << 6) | (0x0 << 10) | (0x0 << 12))

/* Register 11 Bit Definitions */
#define ADF5902_CNTR_RESET_DISABLE	0x0
#define ADF5902_CNTR_RESET_ENABLE	0x1

#define ADF5902_CONT_SAWTOOTH		0x0
#define ADF5902_SAWTOOTH_BURST		0x1
#define ADF5902_CONTINUOUS_TRIANGULAR	0x2
#define ADF5902_SINGLE_RAMP_BURST	0x3

#define ADF5902_SINGLE_FULL_TRI_DISBLE	0x0
#define ADF5902_SINGLE_FULL_TRI_ENABLE	0x1

#define ADF5902_SD_RESET_ENABLE		0x0
#define ADF5902_SD_RESET_DISABLE	0x1

/* Register 12 Map */
#define ADF5902_REG12_CP_TRISTATE(x)	(((x) & 0x1) << 15)
#define ADF5902_REG12_CHARGE_PUMP(x)	(((x) & 0xF) << 17)
#define ADF5902_REG12_RESERVED		((0x0 << 5) | (0x1 << 16) | (0x2 << 21))

/* Register 12 Bit Definition */
#define ADF5902_CP_TRISTATE_DISABLE	0x0
#define ADF5902_CP_TRISTATE_ENABLE	0x1

#define ADF5902_CP_CURRENT_280UA	0x0
#define ADF5902_CP_CURRENT_560UA	0x1
#define ADF5902_CP_CURRENT_840UA	0x2
#define ADF5902_CP_CURRENT_1MA12	0x3
#define ADF5902_CP_CURRENT_1MA40	0x4
#define ADF5902_CP_CURRENT_1MA68	0x5
#define ADF5902_CP_CURRENT_1MA96	0x6
#define ADF5902_CP_CURRENT_2MA24	0x7
#define ADF5902_CP_CURRENT_2MA52	0x8
#define ADF5902_CP_CURRENT_2MA80	0x9
#define ADF5902_CP_CURRENT_3MA08	0xA
#define ADF5902_CP_CURRENT_3MA36	0xB
#define ADF5902_CP_CURRENT_3MA64	0xC
#define ADF5902_CP_CURRENT_3MA92	0xD
#define ADF5902_CP_CURRENT_4MA20	0xE
#define ADF5902_CP_CURRENT_4MA48	0xF

/* Register 13 Map */
#define ADF5902_REG13_CLK_DIV_SEL(x)	(((x) & 0x3) << 5)
#define ADF5902_REG13_CLK_DIV_2(x)	(((x) & 0xFFF) << 7)
#define ADF5902_REG13_CLK_DIV_MODE(x)	(((x) & 0x3) << 19)
#define ADF5902_REG13_LE_SEL(x)		(((x) & 0x1) << 21)
#define ADF5902_REG13_RESERVED		(0x0 << 22)

/* Register 13 Bit Definitions */
#define ADF5902_CLK_DIV_SEL_0		0x0
#define ADF5902_CLK_DIV_SEL_1		0x1
#define ADF5902_CLK_DIV_SEL_2		0x2
#define ADF5902_CLK_DIV_SEL_3		0x3

#define ADF5902_MIN_CLK_DIV_2		0x000
#define ADF5902_MAX_CLK_DIV_2		0xFFF

#define ADF5902_CLK_DIV_OFF		0x0
#define ADF5902_FREQ_MEASURE		0x2
#define ADF5902_RAMP_DIV		0x3

#define ADF5902_LE_FROM_PIN		0x0
#define ADF5902_LE_SYNC_REFIN		0x1

/* Register 14 Map */
#define ADF5902_REG14_DEV_WORD(x)	(((x) & 0xFFFF) << 5)
#define ADF5902_REG14_DEV_OFFSET(x)	(((x) & 0xF) << 21)
#define ADF5902_REG14_DEV_SEL(x)	(((x) & 0x3) << 25)
#define ADF5902_REG14_TX_RAMP_CLK(x)	(((x) & 0x1) << 30)
#define ADF5902_REG14_TX_DATA_INV(x)	(((x) & 0x1) << 31)
#define ADF5902_REG14_RESERVED		(0x0 << 27)

/* Register 14 Bit Definitions */
#define ADF5902_MAX_DEV_WORD		0x7FFF
#define ADF5902_MIN_DEV_WORD		(int16_t)0x8000

#define ADF5902_MAX_DEV_OFFSET		0x9
#define ADF5902_MIN_DEV_OFFSET		0x0

#define ADF5902_DEV_SEL_0		0x0
#define ADF5902_DEV_SEL_1		0x1
#define ADF5902_DEV_SEL_2		0x2
#define ADF5902_DEV_SEL_3		0x3

#define ADF5902_TX_RAMP_CLK_DIV		0x0
#define ADF5902_TX_RAMP_TX_DATA_PIN	0x1

#define AD5902_TX_DATA_INV_DISABLE	0x0
#define AD5902_TX_DATA_INV_ENABLE	0x1

/* Register 15 Map */
#define ADF5902_REG15_STEP_WORD(x)	(((x) & 0xFFFFF) << 5)
#define ADF5902_REG15_STEP_SEL(x)	(((x) & 0x3) << 25)
#define ADF5902_REG15_RESERVED		(0x0 << 27)

/* Register 15 Bit Definition */
#define ADF5902_MIN_STEP_WORD		0x00000
#define ADF5902_MAX_STEP_WORD		0xFFFFF

#define ADF5902_STEP_SEL_0		0x0
#define ADF5902_STEP_SEL_1		0x1
#define ADF5902_STEP_SEL_2		0x2
#define ADF5902_STEP_SEL_3		0x3

/* Register 16 Map */
#define ADF5902_REG16_DEL_START_WORD(x)	(((x) & 0xFFF) << 5)
#define ADF5902_REG16_RAMP_DEL(x)	(((x) & 0x1) << 19)
#define ADF5902_REG16_TX_DATA_TRIG(x)	(((x) & 0x1) << 20)
#define ADF5902_REG16_DEL_SEL(x)	(((x) & 0x3) << 23)
#define ADF5902_REG16_RESERVED		((0x0 << 17) | (0x0 << 21) | (0x1 << 25))

#define ADF5902_MIN_DELAY_START_WRD	0x000
#define ADF5902_MAX_DELAY_START_WRD	0xFFF

#define ADF5902_RAMP_DEL_DISABLE	0x0
#define ADF5902_RAMP_DEL_ENABLE		0x1

#define ADF5902_TX_DATA_TRIG_DISABLE	0x0
#define ADF5902_TX_DATA_TRIG_ENABLE	0x1

#define ADF5902_DEL_SEL_0		0x0
#define ADF5902_DEL_SEL_1		0x1
#define ADF5902_DEL_SEL_2		0x2
#define ADF5902_DEL_SEL_3		0x3

/* Register 17 Map */
#define ADF5902_REG17_RESERVED		(0x0 << 5)

/** Specifications */
#define ADF5902_MAX_VCO_FREQ		24250000000ull
#define ADF5902_MIN_VCO_FREQ		24000000000ull
#define ADF5902_MIN_REFIN_FREQ		10000000
#define ADF5902_MAX_REFIN_FREQ		260000000
#define ADF5902_MAX_FREQ_PFD		110000000
#define ADF5902_MAX_SLOPE_NO		4
#define ADF5902_MAX_DELAY_WORD_NO	4
#define ADF5902_MAX_CLK2_DIV_NO		4
#define ADF5902_VLSB				0.00733f
#define ADF5902_VOFF				0.699f
#define ADF5902_VGAIN				0.0064f
#define ADF5902_SPI_DUMMY_DATA		0x0
#define ADF5902_FREQ_CAL_DIV_100KHZ	100000
#define ADF5902_CLK1_DIV_25KHZ		25000
#define ADF5902_ADC_CLK_DIV_1MHZ	1000000
#define ADF5902_BUFF_SIZE_BYTES		4
#define ADF5902_FRAC_MSB_MSK		0xFFF
#define ADF5902_FRAC_LSB_MSK		0x1FFF

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `slope` structure is designed to encapsulate parameters related to
 * frequency deviation and step size in a frequency synthesizer or
 * similar application. It includes a deviation word (`dev_word`) for
 * specifying the magnitude of deviation, a deviation offset
 * (`dev_offset`) for fine-tuning the deviation, and a step word
 * (`step_word`) for defining the step size in frequency adjustments.
 * This structure is likely used in the context of configuring or
 * controlling frequency modulation characteristics in a device such as
 * the ADF5902.
 *
 * @param dev_word Represents the deviation word as a signed 16-bit integer.
 * @param dev_offset Represents the deviation offset as an unsigned 8-bit
 * integer.
 * @param step_word Represents the step word as an unsigned 32-bit integer.
 ******************************************************************************/
struct slope {
	/* Deviation Word */
	int16_t dev_word;
	/* Deviation Offset */
	uint8_t dev_offset;
	/* Step Word */
	uint32_t step_word;
};

/***************************************************************************//**
 * @brief The `adf5902_init_param` structure is used to initialize the ADF5902
 * device, a microwave frequency synthesizer. It contains various
 * configuration parameters such as SPI and GPIO initialization settings,
 * frequency settings for the reference input and VCO output, and control
 * flags for features like reference frequency doubling, ADC averaging,
 * and ramp delay. Additionally, it includes settings for clock dividers,
 * charge pump current, and ramp mode, as well as arrays for delay words
 * and slope structures to manage frequency deviation parameters.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_ce_param Pointer to GPIO Chip Enable initialization parameters.
 * @param ref_in Reference input frequency in Hz.
 * @param rf_out Output frequency of the internal VCO in Hz.
 * @param ref_doubler_en Flag to enable reference frequency doubling.
 * @param ref_div2_en Flag to enable reference frequency division by 2.
 * @param adc_avg ADC average value setting.
 * @param tx_amp_cal_ref Transmitter amplitude calibration reference code.
 * @param ramp_delay_en Flag to enable ramp delay.
 * @param tx_trig_en Flag to enable TX data trigger.
 * @param delay_words_no Number of delay words.
 * @param delay_wd Array of delay words.
 * @param slopes_no Number of deviation parameters.
 * @param slopes Pointer to an array of slope structures.
 * @param tx_ramp_clk TX data ramp clock setting.
 * @param tx_data_invert Flag to enable TX data inversion.
 * @param ramp_status Ramp status indicator.
 * @param clk1_div_ramp Clock divider value in ramp mode.
 * @param clk2_div_no Number of 12-bit clock dividers.
 * @param clk2_div Array of 12-bit clock divider values.
 * @param le_sel LE select setting.
 * @param clk_div_mode Clock divider mode setting.
 * @param cp_current Charge pump current setting.
 * @param cp_tristate_en Flag to enable charge pump tristate.
 * @param ramp_mode Ramp mode setting.
 ******************************************************************************/
struct adf5902_init_param {
	/* SPI Initialization parameters */
	struct no_os_spi_init_param	*spi_init;
	/* GPIO Chip Enable */
	struct no_os_gpio_init_param	*gpio_ce_param;
	/* Reference input frequency */
	uint64_t		ref_in;
	/* Output frequency of the internal VCO */
	uint64_t		rf_out;
	/* Reference doubler enable */
	uint8_t			ref_doubler_en;
	/* Reference divide by 2 bit */
	uint8_t			ref_div2_en;
	/* ADC Average value */
	uint8_t			adc_avg;
	/* Transmitter Amplitude Calibration Reference Code */
	uint8_t			tx_amp_cal_ref;
	/* Ramp delay enable */
	uint8_t			ramp_delay_en;
	/* TX Data trigger */
	uint8_t			tx_trig_en;
	/* Delay words number */
	uint8_t			delay_words_no;
	/* Delay Words */
	uint16_t		delay_wd[ADF5902_MAX_DELAY_WORD_NO];
	/* Number of deviaton parameters */
	uint8_t			slopes_no;
	/* Slope  structure */
	struct slope		*slopes;
	/* Tx Data Ramp Clock */
	uint8_t			tx_ramp_clk;
	/* Tx Data Invert */
	uint8_t			tx_data_invert;
	/* Ramp Status */
	uint16_t			ramp_status;
	/* Clock divider (CLK1) divider value in Ramp mode */
	uint16_t		clk1_div_ramp;
	/* 12-bit Clock Divider number */
	uint8_t			clk2_div_no;
	/* 12-bit Clock Divider */
	uint16_t		clk2_div[ADF5902_MAX_CLK2_DIV_NO];
	/* LE Select */
	uint8_t			le_sel;
	/* Clock Divider Mode*/
	uint8_t			clk_div_mode;
	/* Charge Pump current */
	uint8_t			cp_current;
	/* Charge Pump tristate */
	uint8_t			cp_tristate_en;
	/* Ramp Mode */
	uint8_t			ramp_mode;
};

/***************************************************************************//**
 * @brief The `adf5902_dev` structure is a comprehensive configuration and state
 * representation for the ADF5902 device, which is a microwave frequency
 * synthesizer. It includes various fields for managing SPI
 * communication, GPIO control, frequency settings, and calibration
 * parameters. The structure allows for detailed configuration of the
 * device's internal VCO, phase frequency detector, and clock dividers,
 * as well as control over ramping and amplitude calibration. It supports
 * advanced features such as reference doubling, fractional division, and
 * multiple clock and delay configurations, making it suitable for
 * precise frequency synthesis applications.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param gpio_ce Pointer to the GPIO descriptor for chip enable control.
 * @param ref_in Reference input frequency in Hz.
 * @param rf_out Output frequency of the internal VCO in Hz.
 * @param f_pfd Phase Frequency Detector frequency in Hz.
 * @param ref_div_factor Divide ratio of the binary 5-bit reference counter.
 * @param ref_doubler_en Flag to enable the reference doubler.
 * @param ref_div2_en Flag to enable reference divide by 2.
 * @param int_div Integer division value for Register 5.
 * @param frac_msb Most significant bits of the fractional division value for
 * Register 5.
 * @param frac_lsb Least significant bits of the fractional division value for
 * Register 5.
 * @param freq_cal_div Frequency calibration divider value.
 * @param clk1_div Clock divider value for CLK1.
 * @param clk1_div_ramp Clock divider value for CLK1 in Ramp mode.
 * @param adc_clk_div ADC clock divider value.
 * @param adc_avg ADC average value setting.
 * @param tx_amp_cal_ref Transmitter amplitude calibration reference code.
 * @param ramp_delay_en Flag to enable ramp delay.
 * @param tx_trig_en Flag to enable TX data trigger.
 * @param delay_words_no Number of delay words.
 * @param delay_wd Pointer to an array of delay words.
 * @param slopes_no Number of deviation parameters.
 * @param slopes Pointer to an array of slope structures.
 * @param tx_ramp_clk TX data ramp clock setting.
 * @param tx_data_invert Flag to invert TX data.
 * @param ramp_status Status of the ramp.
 * @param clk2_div_no Number of 12-bit clock dividers.
 * @param clk2_div Pointer to an array of 12-bit clock dividers.
 * @param le_sel LE select setting.
 * @param clk_div_mode Clock divider mode setting.
 * @param cp_current Charge pump current setting.
 * @param cp_tristate_en Flag to enable charge pump tristate.
 * @param ramp_mode Ramp mode setting.
 ******************************************************************************/
struct adf5902_dev {
	/* SPI Descriptor */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO Chip Enable */
	struct no_os_gpio_desc	*gpio_ce;
	/* Reference input frequency*/
	uint64_t		ref_in;
	/* Output frequency(Hz) of the internal VCO */
	uint64_t		rf_out;
	/* Phase Frequency Detector */
	uint64_t		f_pfd;
	/* Divide ration of the binary 5-bit reference counter */
	uint8_t			ref_div_factor;
	/* Reference doubler enable */
	uint8_t			ref_doubler_en;
	/* Reference divide by 2 bit */
	uint8_t			ref_div2_en;
	/* Register 5 Integer word */
	uint16_t		int_div;
	/* Register 5 MSB FRAC value */
	uint16_t		frac_msb;
	/* Register 5 LSB FRAC value */
	uint16_t		frac_lsb;
	/* Frequency calibration divider value */
	uint16_t		freq_cal_div;
	/* Clock divider (CLK1) divider value */
	uint16_t		clk1_div;
	/* Clock divider (CLK1) divider value in Ramp mode */
	uint16_t		clk1_div_ramp;
	/* ADC Clock divider */
	uint16_t		adc_clk_div;
	/* ADC Average value */
	uint8_t			adc_avg;
	/* Transmitter Amplitude Calibration Reference Code */
	uint8_t			tx_amp_cal_ref;
	/* Ramp delay enable */
	uint8_t			ramp_delay_en;
	/* TX Data trigger */
	uint8_t			tx_trig_en;
	/* Delay words number */
	uint8_t			delay_words_no;
	/* Delay Words */
	uint16_t		*delay_wd;
	/* Number of deviaton parameters */
	uint8_t			slopes_no;
	/* Slope Structure */
	struct slope	*slopes;
	/* Tx Data Ramp Clock */
	uint8_t			tx_ramp_clk;
	/* Tx Data Invert */
	uint8_t			tx_data_invert;
	/* Ramp Status */
	uint16_t		ramp_status;
	/* 12-bit Clock Divider number */
	uint8_t			clk2_div_no;
	/* 12-bit Clock Divider */
	uint16_t		*clk2_div;
	/* LE Select */
	uint8_t			le_sel;
	/* Clock Divider Mode*/
	uint8_t			clk_div_mode;
	/* Charge Pump current */
	uint8_t			cp_current;
	/* Charge Pump tristate */
	uint8_t			cp_tristate_en;
	/* Ramp Mode */
	uint8_t			ramp_mode;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief Use this function to write a 32-bit data value to a specific register
 * of the ADF5902 device. This function is typically called when
 * configuring the device or updating its settings. It requires a valid
 * device structure that has been properly initialized and a valid
 * register address. The function combines the register address with the
 * data and sends it over SPI. Ensure that the device is correctly
 * initialized before calling this function to avoid communication
 * errors.
 *
 * @param dev A pointer to an adf5902_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr A uint8_t value representing the register address to which
 * the data will be written. Valid register addresses are
 * defined by the device's register map.
 * @param data A uint32_t value containing the data to be written to the
 * specified register. The data is combined with the register
 * address before transmission.
 * @return Returns an int32_t value indicating the success or failure of the SPI
 * write operation. A non-negative value typically indicates success,
 * while a negative value indicates an error.
 ******************************************************************************/
int32_t adf5902_write(struct adf5902_dev *dev, uint8_t reg_addr,
		      uint32_t data);

/***************************************************************************//**
 * @brief Use this function to read data from a specific register of the ADF5902
 * device. It requires a valid device structure and a register address to
 * perform the read operation. The function communicates with the device
 * over SPI and retrieves the data from the specified register, storing
 * it in the provided data pointer. Ensure that the device is properly
 * initialized before calling this function. The function returns an
 * error code if the SPI communication fails.
 *
 * @param dev A pointer to an initialized adf5902_dev structure representing the
 * device. Must not be null.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the device's addressable range.
 * @param data A pointer to a uint32_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns an int32_t error code: 0 on success, or a negative error code
 * on failure.
 ******************************************************************************/
int32_t adf5902_readback(struct adf5902_dev *dev, uint8_t reg_addr,
			 uint32_t *data);

/***************************************************************************//**
 * @brief This function initializes the ADF5902 device using the provided
 * initialization parameters. It sets up the necessary GPIO and SPI
 * interfaces, configures the device registers, and performs initial
 * calibration. This function must be called before any other operations
 * on the ADF5902 device. It handles memory allocation for the device
 * structure and ensures that all initialization parameters are valid. If
 * any step in the initialization process fails, the function will return
 * an error code and clean up any allocated resources.
 *
 * @param device A pointer to a pointer of type `struct adf5902_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A pointer to a `struct adf5902_init_param` containing the
 * initialization parameters for the device. This must not be
 * null and must contain valid configuration data for the
 * device. The function will validate these parameters before
 * proceeding with initialization.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the `device` pointer will point to a newly allocated and initialized
 * `adf5902_dev` structure.
 ******************************************************************************/
int32_t adf5902_init(struct adf5902_dev **device,
		     struct adf5902_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to perform a recalibration of the ADF5902 device,
 * which is necessary to ensure accurate frequency operation. This
 * function should be called when the device requires recalibration, such
 * as after significant temperature changes or when the device has been
 * powered down for an extended period. The function assumes that the
 * device has been properly initialized and configured. It temporarily
 * modifies certain device settings to perform the recalibration and then
 * restores them to their original state. The function returns an error
 * code if the recalibration process fails at any step.
 *
 * @param dev A pointer to an initialized adf5902_dev structure representing the
 * device to be recalibrated. Must not be null. The function will
 * return an error if the device is not properly initialized.
 * @return Returns 0 on success or a negative error code if the recalibration
 * fails at any step.
 ******************************************************************************/
int32_t adf5902_recalibrate(struct adf5902_dev *dev);

/***************************************************************************//**
 * @brief Use this function to obtain the current temperature reading from an
 * ADF5902 device. It must be called with a valid device structure that
 * has been properly initialized. The function performs an ADC conversion
 * to read the temperature sensor data and returns the temperature in
 * degrees Celsius through the provided pointer. Ensure that the device
 * is not in a power-down state before calling this function. The
 * function returns an error code if the operation fails at any step.
 *
 * @param dev A pointer to an initialized adf5902_dev structure representing the
 * device. Must not be null.
 * @param temp A pointer to a float where the temperature value will be stored.
 * Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int32_t adf5902_read_temp(struct adf5902_dev *dev, float *temp);

/* ADF5902 Measure Output locked frequency */
/***************************************************************************//**
 * @brief Use this function to measure and compute the output frequency of an
 * ADF5902 device. It should be called when you need to determine the
 * current frequency output of the device. Ensure that the device is
 * properly initialized and configured before calling this function. The
 * function will perform necessary register reads and writes to calculate
 * the frequency, and it requires a valid device structure. It returns an
 * error code if any operation fails during the process.
 *
 * @param dev A pointer to an initialized adf5902_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param freq A pointer to a uint64_t where the computed frequency will be
 * stored. Must not be null. The function writes the computed
 * frequency to this location.
 * @return Returns an int32_t error code: 0 on success, or a negative error code
 * on failure.
 ******************************************************************************/
int32_t adf5902f_compute_frequency(struct adf5902_dev *dev, uint64_t *freq);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * ADF5902 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated SPI and GPIO resources are also released.
 * It is important to call this function to prevent resource leaks in the
 * system.
 *
 * @param dev A pointer to an adf5902_dev structure representing the device to
 * be removed. Must not be null. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns 0 on successful removal of resources, or a negative error
 * code if an error occurs during the removal process.
 ******************************************************************************/
int32_t adf5902_remove(struct adf5902_dev *dev);

#endif /* SRC_ADF5902_H_ */
