/***************************************************************************//**
 *   @file   adf4350.h
 *   @brief  Header file of ADF4350 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012-2015(c) Analog Devices, Inc.
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
#ifndef __ADF4350_H__
#define __ADF4350_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/* Channels */
#define ADF4350_RX_CHANNEL	0
#define ADF4350_TX_CHANNEL	1

/* Registers */
#define ADF4350_REG0	0
#define ADF4350_REG1	1
#define ADF4350_REG2	2
#define ADF4350_REG3	3
#define ADF4350_REG4	4
#define ADF4350_REG5	5

/* REG0 Bit Definitions */
#define ADF4350_REG0_FRACT(x)					(((x) & 0xFFF) << 3)
#define ADF4350_REG0_INT(x)						(((x) & 0xFFFF) << 15)

/* REG1 Bit Definitions */
#define ADF4350_REG1_MOD(x)						(((x) & 0xFFF) << 3)
#define ADF4350_REG1_PHASE(x)					(((x) & 0xFFF) << 15)
#define ADF4350_REG1_PRESCALER					(1 << 27)

/* REG2 Bit Definitions */
#define ADF4350_REG2_COUNTER_RESET_EN			(1 << 3)
#define ADF4350_REG2_CP_THREESTATE_EN			(1 << 4)
#define ADF4350_REG2_POWER_DOWN_EN				(1 << 5)
#define ADF4350_REG2_PD_POLARITY_POS			(1 << 6)
#define ADF4350_REG2_LDP_6ns					(1 << 7)
#define ADF4350_REG2_LDP_10ns					(0 << 7)
#define ADF4350_REG2_LDF_FRACT_N				(0 << 8)
#define ADF4350_REG2_LDF_INT_N					(1 << 8)
#define ADF4350_REG2_CHARGE_PUMP_CURR_uA(x)		(((((x)-312) / 312) & 0xF) << 9)
#define ADF4350_REG2_DOUBLE_BUFF_EN				(1 << 13)
#define ADF4350_REG2_10BIT_R_CNT(x)				((x) << 14)
#define ADF4350_REG2_RDIV2_EN					(1 << 24)
#define ADF4350_REG2_RMULT2_EN					(1 << 25)
#define ADF4350_REG2_MUXOUT(x)					((x) << 26)
#define ADF4350_REG2_NOISE_MODE(x)				(((x) & 0x3) << 29)

/* REG3 Bit Definitions */
#define ADF4350_REG3_12BIT_CLKDIV(x)			((x) << 3)
#define ADF4350_REG3_12BIT_CLKDIV_MODE(x)		((x) << 16)
#define ADF4350_REG3_12BIT_CSR_EN				(1 << 18)
#define ADF4351_REG3_CHARGE_CANCELLATION_EN		(1 << 21)
#define ADF4351_REG3_ANTI_BACKLASH_3ns_EN		(1 << 22)
#define ADF4351_REG3_BAND_SEL_CLOCK_MODE_HIGH	(1 << 23)

/* REG4 Bit Definitions */
#define ADF4350_REG4_OUTPUT_PWR(x)				((x) << 3)
#define ADF4350_REG4_RF_OUT_EN					(1 << 5)
#define ADF4350_REG4_AUX_OUTPUT_PWR(x)			((x) << 6)
#define ADF4350_REG4_AUX_OUTPUT_EN				(1 << 8)
#define ADF4350_REG4_AUX_OUTPUT_FUND			(1 << 9)
#define ADF4350_REG4_AUX_OUTPUT_DIV				(0 << 9)
#define ADF4350_REG4_MUTE_TILL_LOCK_EN			(1 << 10)
#define ADF4350_REG4_VCO_PWRDOWN_EN				(1 << 11)
#define ADF4350_REG4_8BIT_BAND_SEL_CLKDIV(x)	((x) << 12)
#define ADF4350_REG4_RF_DIV_SEL(x)				((x) << 20)
#define ADF4350_REG4_FEEDBACK_DIVIDED			(0 << 23)
#define ADF4350_REG4_FEEDBACK_FUND				(1 << 23)

/* REG5 Bit Definitions */
#define ADF4350_REG5_LD_PIN_MODE_LOW			(0 << 22)
#define ADF4350_REG5_LD_PIN_MODE_DIGITAL		(1 << 22)
#define ADF4350_REG5_LD_PIN_MODE_HIGH			(3 << 22)

/* Specifications */
#define ADF4350_MAX_OUT_FREQ		4400000000ULL /* Hz */
#define ADF4350_MIN_OUT_FREQ		34375000 /* Hz */
#define ADF4350_MIN_VCO_FREQ		2200000000ULL /* Hz */
#define ADF4350_MAX_FREQ_45_PRESC	3000000000ULL /* Hz */
#define ADF4350_MAX_FREQ_PFD		32000000 /* Hz */
#define ADF4350_MAX_BANDSEL_CLK		125000 /* Hz */
#define ADF4350_MAX_FREQ_REFIN		250000000 /* Hz */
#define ADF4350_MAX_MODULUS			4095
#define ADF4350_MAX_R_CNT			1023

/******************************************************************************/
/************************ Types Definitions ***********************************/
/***************************************************************************//**
 * @brief The `adf4350_platform_data` structure is used to configure the ADF4350
 * frequency synthesizer device. It contains parameters for setting the
 * input clock frequency, channel spacing, and power-up frequency, as
 * well as various configuration flags and user settings for the device's
 * registers. This structure allows for detailed customization of the
 * device's operation, including reference division and doubling, and
 * provides a mechanism for lock detection through a GPIO pin.
 *
 * @param clkin The input clock frequency in Hz.
 * @param channel_spacing The frequency spacing between channels in Hz.
 * @param power_up_frequency The frequency in Hz at which the device powers up.
 * @param ref_div_factor The reference division factor, a 10-bit R counter.
 * @param ref_doubler_en Flag to enable the reference doubler.
 * @param ref_div2_en Flag to enable the reference divide-by-2.
 * @param r2_user_settings User-defined settings for register 2.
 * @param r3_user_settings User-defined settings for register 3.
 * @param r4_user_settings User-defined settings for register 4.
 * @param gpio_lock_detect GPIO pin configuration for lock detection.
 ******************************************************************************/
struct adf4350_platform_data {
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint64_t	power_up_frequency;

	uint16_t	ref_div_factor; /* 10-bit R counter */
	uint8_t	    ref_doubler_en;
	uint8_t	    ref_div2_en;

	uint32_t    r2_user_settings;
	uint32_t    r3_user_settings;
	uint32_t    r4_user_settings;
	int32_t	    gpio_lock_detect;
};

/***************************************************************************//**
 * @brief The `adf4350_init_param` structure is used to initialize and configure
 * the ADF4350 frequency synthesizer. It includes parameters for SPI
 * communication, device settings such as input clock frequency, channel
 * spacing, and power-up frequency, as well as various user settings for
 * phase detection, lock detection, charge pump current, and output
 * configurations. This structure allows for detailed customization of
 * the ADF4350's operation, enabling precise control over frequency
 * synthesis and output characteristics.
 *
 * @param spi_init Holds the SPI initialization parameters for communication.
 * @param clkin Specifies the input clock frequency in Hz.
 * @param channel_spacing Defines the channel spacing in Hz.
 * @param power_up_frequency Sets the initial frequency at power-up in Hz.
 * @param reference_div_factor Determines the reference division factor.
 * @param reference_doubler_enable Enables or disables the reference doubler.
 * @param reference_div2_enable Enables or disables the reference divide-by-2.
 * @param phase_detector_polarity_positive_enable Enables positive polarity for
 * the phase detector.
 * @param lock_detect_precision_6ns_enable Enables 6ns precision for lock
 * detection.
 * @param lock_detect_function_integer_n_enable Enables lock detect function for
 * integer-N mode.
 * @param charge_pump_current Sets the charge pump current in microamperes.
 * @param muxout_select Selects the MUXOUT pin function.
 * @param low_spur_mode_enable Enables low spur mode.
 * @param cycle_slip_reduction_enable Enables cycle slip reduction.
 * @param charge_cancellation_enable Enables charge cancellation.
 * @param anti_backlash_3ns_enable Enables 3ns anti-backlash pulse.
 * @param band_select_clock_mode_high_enable Enables high mode for band select
 * clock.
 * @param clk_divider_12bit Sets the 12-bit clock divider value.
 * @param clk_divider_mode Defines the clock divider mode.
 * @param aux_output_enable Enables the auxiliary output.
 * @param aux_output_fundamental_enable Enables fundamental frequency for
 * auxiliary output.
 * @param mute_till_lock_enable Mutes the output until lock is detected.
 * @param output_power Sets the output power level.
 * @param aux_output_power Sets the auxiliary output power level.
 ******************************************************************************/
typedef struct {
	/* SPI */
	struct no_os_spi_init_param	spi_init;

	/* Device settings */
	uint32_t	clkin;
	uint32_t	channel_spacing;
	uint32_t	power_up_frequency;
	uint32_t	reference_div_factor;
	uint8_t		reference_doubler_enable;
	uint8_t		reference_div2_enable;

	/* r2_user_settings */
	uint8_t		phase_detector_polarity_positive_enable;
	uint8_t		lock_detect_precision_6ns_enable;
	uint8_t		lock_detect_function_integer_n_enable;
	uint32_t	charge_pump_current;
	uint32_t	muxout_select;
	uint8_t		low_spur_mode_enable;

	/* r3_user_settings */
	uint8_t		cycle_slip_reduction_enable;
	uint8_t		charge_cancellation_enable;
	uint8_t		anti_backlash_3ns_enable;
	uint8_t		band_select_clock_mode_high_enable;
	uint32_t	clk_divider_12bit;
	uint32_t	clk_divider_mode;

	/* r4_user_settings */
	uint8_t		aux_output_enable;
	uint8_t		aux_output_fundamental_enable;
	uint8_t		mute_till_lock_enable;
	uint32_t	output_power;
	uint32_t	aux_output_power;
} adf4350_init_param;

/***************************************************************************//**
 * @brief The `adf4350_dev` structure is used to represent the state and
 * configuration of an ADF4350 device, which is a wideband synthesizer
 * with integrated VCO. It includes pointers to SPI communication
 * descriptors and platform-specific data, as well as various
 * configuration parameters such as input clock frequency, channel
 * spacing, and phase frequency detector frequency. The structure also
 * contains arrays for register configurations and specific fields for
 * fractional and integer frequency settings, modulus, and RF divider
 * selection, which are essential for the operation and tuning of the
 * synthesizer.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param pdata Pointer to platform-specific data for the ADF4350.
 * @param clkin Input clock frequency in Hz.
 * @param chspc Channel spacing in Hz.
 * @param fpfd Phase Frequency Detector frequency in Hz.
 * @param min_out_freq Minimum output frequency in Hz.
 * @param r0_fract Fractional part of the frequency word for register 0.
 * @param r0_int Integer part of the frequency word for register 0.
 * @param r1_mod Modulus value for register 1.
 * @param r4_rf_div_sel RF divider select value for register 4.
 * @param regs Array of 6 registers for configuration.
 * @param regs_hw Array of 6 hardware registers for configuration.
 * @param val A general-purpose value used in the device.
 ******************************************************************************/
typedef struct {
	struct no_os_spi_desc	*spi_desc;
	struct adf4350_platform_data *pdata;
	uint32_t	clkin;
	uint32_t	chspc;	/* Channel Spacing */
	uint32_t	fpfd;	/* Phase Frequency Detector */
	uint32_t	min_out_freq;
	uint32_t	r0_fract;
	uint32_t	r0_int;
	uint32_t	r1_mod;
	uint32_t	r4_rf_div_sel;
	uint32_t	regs[6];
	uint32_t	regs_hw[6];
	uint32_t 	val;
} adf4350_dev;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief This function sets up the ADF4350 device by allocating necessary
 * resources and configuring it according to the provided initialization
 * parameters. It must be called before any other operations on the
 * ADF4350 device. The function initializes the SPI interface and
 * configures the device settings such as frequency, channel spacing, and
 * various user settings. If the initialization is successful, the
 * function returns a non-negative value and provides a pointer to the
 * initialized device structure. In case of failure, it returns a
 * negative error code.
 *
 * @param device A pointer to a pointer of type adf4350_dev. This will be set to
 * point to the newly allocated and initialized device structure.
 * Must not be null.
 * @param init_param A structure of type adf4350_init_param containing the
 * initialization parameters for the device. This includes SPI
 * initialization parameters and various device-specific
 * settings. The caller retains ownership of this structure.
 * @return Returns 0 on success or a negative error code on failure. On success,
 * the device pointer is updated to point to the initialized device
 * structure.
 ******************************************************************************/
int32_t adf4350_setup(adf4350_dev **device,
		      adf4350_init_param init_param);
/***************************************************************************//**
 * @brief This function is used to send a 32-bit data word to the ADF4350 device
 * over the SPI interface. It is typically called when configuring the
 * device registers. The function requires a valid device structure that
 * has been properly initialized with SPI settings. It is important to
 * ensure that the device is ready to receive data before calling this
 * function to avoid communication errors.
 *
 * @param dev A pointer to an adf4350_dev structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null. The caller retains ownership.
 * @param data A 32-bit unsigned integer representing the data to be written to
 * the device. The data is split into four bytes and sent over SPI.
 * @return Returns an int32_t indicating the success or failure of the SPI write
 * operation. A non-negative value typically indicates success, while a
 * negative value indicates an error.
 ******************************************************************************/
int32_t adf4350_write(adf4350_dev *dev,
		      uint32_t data);
/***************************************************************************//**
 * @brief Use this function to set the output frequency of the ADF4350 device to
 * a specified value in Hertz. This function should be called after the
 * device has been properly initialized. It is important to ensure that
 * the frequency value provided is within the valid range supported by
 * the device. The function returns the actual frequency set, which may
 * differ from the requested frequency due to device constraints or
 * rounding.
 *
 * @param dev A pointer to an adf4350_dev structure representing the device.
 * Must not be null, and the device must be initialized before
 * calling this function.
 * @param Hz The desired output frequency in Hertz. The value should be within
 * the device's supported frequency range. If the value is outside
 * this range, the function will handle it according to the device's
 * capabilities, potentially clamping or adjusting the frequency.
 * @return Returns the actual frequency set on the device in Hertz, which may
 * differ from the requested frequency due to device constraints or
 * rounding.
 ******************************************************************************/
int64_t adf4350_out_altvoltage0_frequency(adf4350_dev *dev,
		int64_t Hz);
/***************************************************************************//**
 * @brief This function sets the channel spacing for the ADF4350 device to the
 * specified frequency in Hertz. It should be called when you need to
 * configure or update the channel spacing of the device. If the input
 * frequency is not INT32_MAX, the channel spacing is updated to the
 * provided value. This function returns the current channel spacing
 * after the update. It is important to ensure that the device is
 * properly initialized before calling this function.
 *
 * @param dev A pointer to an adf4350_dev structure representing the device.
 * Must not be null, and the device should be initialized before use.
 * @param Hz An integer representing the desired channel spacing in Hertz. If
 * set to INT32_MAX, the channel spacing is not updated. Otherwise, it
 * updates the channel spacing to this value.
 * @return Returns the current channel spacing in Hertz after the function call.
 ******************************************************************************/
int32_t adf4350_out_altvoltage0_frequency_resolution(adf4350_dev *dev,
		int32_t Hz);
/***************************************************************************//**
 * @brief This function sets the reference input frequency for the ADF4350
 * device to the specified value in Hertz. It should be used when you
 * need to configure or update the reference frequency for the device.
 * The function will update the device's internal state with the new
 * frequency unless the specified frequency is INT32_MAX, in which case
 * the current frequency setting is retained. This function returns the
 * current reference input frequency after the operation, which may be
 * useful for verification purposes.
 *
 * @param dev A pointer to an adf4350_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param Hz The desired reference input frequency in Hertz. If set to
 * INT32_MAX, the function will not change the current frequency
 * setting. Otherwise, it updates the device's reference frequency to
 * this value.
 * @return Returns the current reference input frequency of the device in Hertz
 * after the operation.
 ******************************************************************************/
int64_t adf4350_out_altvoltage0_refin_frequency(adf4350_dev *dev,
		int64_t Hz);
/***************************************************************************//**
 * @brief Use this function to control the power state of the PLL by either
 * powering it down or powering it up. This function should be called
 * when you need to manage the power consumption of the device, such as
 * when the PLL is not in use. It is important to ensure that the device
 * has been properly initialized before calling this function. The
 * function modifies the power state based on the input parameter and
 * returns the current power state of the PLL.
 *
 * @param dev A pointer to an adf4350_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before use.
 * @param pwd An integer indicating the desired power state. Use 1 to power down
 * the PLL and 0 to power it up. Any other values are ignored, and
 * the function will not change the power state.
 * @return Returns the current power state of the PLL as an integer, where a
 * non-zero value indicates that the PLL is powered down.
 ******************************************************************************/
int32_t adf4350_out_altvoltage0_powerdown(adf4350_dev *dev,
		int32_t pwd);

#endif // __ADF4350_H__
