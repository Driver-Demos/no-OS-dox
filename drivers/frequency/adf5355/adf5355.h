/***************************************************************************//**
 *   @file   adf5355.h
 *   @brief  Header file for adf5355 Driver.
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

/* Register Definition */
#define ADF5355_REG(x)                          (x)

/* REG0 Bit Definitions */
#define ADF5355_REG0_INT(x)			            (((x) & 0xFFFF) << 4)
#define ADF5355_REG0_PRESCALER(x)		        ((x) << 20)
#define ADF5355_REG0_AUTOCAL(x)			        ((x) << 21)

/* REG1 Bit Definitions */
#define ADF5355_REG1_FRACT(x)			        (((x) & 0xFFFFFF) << 4)

/* REG2 Bit Definitions */
#define ADF5355_REG2_MOD2(x)			        (((x) & 0x3FFF) << 4)
#define ADF5355_REG2_FRAC2(x)			        (((x) & 0x3FFF) << 18)

/* REG3 Bit Definitions */
#define ADF5355_REG3_PHASE(x)			        (((x) & 0xFFFFFF) << 4)
#define ADF5355_REG3_PHASE_ADJUST(x)		    ((x) << 28)
#define ADF5355_REG3_PHASE_RESYNC(x)		    ((x) << 29)
#define ADF5355_REG3_EXACT_SDLOAD_RESET(x)	    ((x) << 30)

/* REG4 Bit Definitions */
#define ADF5355_REG4_COUNTER_RESET_EN(x)	    ((x) << 4)
#define ADF5355_REG4_CP_THREESTATE_EN(x)	    ((x) << 5)
#define ADF5355_REG4_POWER_DOWN_EN(x)		    ((x) << 6)
#define ADF5355_REG4_PD_POLARITY_POS(x)		    ((x) << 7)
#define ADF5355_REG4_MUX_LOGIC(x)		        ((x) << 8)
#define ADF5355_REG4_REFIN_MODE_DIFF(x)		    ((x) << 9)
#define ADF5355_REG4_CHARGE_PUMP_CURR(x)		(((x) & 0xF) << 10)
#define ADF5355_REG4_DOUBLE_BUFF_EN(x)		    ((x) << 14)
#define ADF5355_REG4_10BIT_R_CNT(x)		        (((x) & 0x3FF) << 15)
#define ADF5355_REG4_RDIV2_EN(x)		        ((x) << 25)
#define ADF5355_REG4_RMULT2_EN(x)		        ((x) << 26)
#define ADF5355_REG4_MUXOUT(x)			        (((x) & 0x7) << 27)

/* REG5 Bit Definitions */
#define ADF5355_REG5_DEFAULT			        0x00800025

/* REG6 Bit Definitions */
#define ADF4355_REG6_OUTPUTB_PWR(x)		        (((x) & 0x3) << 7)
#define ADF4355_REG6_RF_OUTB_EN(x)		        ((x) << 9)
#define ADF4356_REG6_RF_OUTB_SEL(x)                ((x) << 25)
#define ADF5355_REG6_OUTPUT_PWR(x)		        (((x) & 0x3) << 4)
#define ADF5355_REG6_RF_OUT_EN(x)		        ((x) << 6)
#define ADF5355_REG6_RF_OUTB_EN(x)		        ((x) << 10)
#define ADF5355_REG6_MUTE_TILL_LOCK_EN(x)	    ((x) << 11)
#define ADF5355_REG6_CP_BLEED_CURR(x)		    (((x) & 0xFF) << 13)
#define ADF5355_REG6_RF_DIV_SEL(x)		        (((x) & 0x7) << 21)
#define ADF5355_REG6_FEEDBACK_FUND(x)		    ((x) << 24)
#define ADF5355_REG6_NEG_BLEED_EN(x)		    ((x) << 29)
#define ADF5355_REG6_GATED_BLEED_EN(x)		    ((x) << 30)
#define ADF5356_REG6_BLEED_POLARITY(x)		    ((x) << 31)
#define ADF5355_REG6_DEFAULT			        0x14000006

/* REG7 Bit Definitions */
#define ADF5355_REG7_LD_MODE_INT_N_EN(x)		((x) << 4)
#define ADF5355_REG7_FACT_N_LD_PRECISION(x)	    (((x) & 0x3) << 5)
#define ADF5355_REG7_LOL_MODE_EN(x)		        ((x) << 7)
#define ADF5355_REG7_LD_CYCLE_CNT(x)		    (((x) & 0x3) << 8)
#define ADF5355_REG7_LE_SYNCED_REFIN_EN(x)	    ((x) << 25)
#define ADF5356_REG7_LE_SYNCE_EDGE_RISING_EN(x)	((x) << 27)
#define ADF5355_REG7_DEFAULT			        0x10000007
#define ADF5356_REG7_DEFAULT			        0x04000007

/* REG8 Bit Definitions */
#define ADF5355_REG8_DEFAULT			        0x102D0428
#define ADF5356_REG8_DEFAULT			        0x15596568

/* REG9 Bit Definitions */
#define ADF5355_REG9_SYNTH_LOCK_TIMEOUT(x)	    (((x) & 0x1F) << 4)
#define ADF5355_REG9_ALC_TIMEOUT(x)		        (((x) & 0x1F) << 9)
#define ADF5355_REG9_TIMEOUT(x)			        (((x) & 0x3FF) << 14)
#define ADF5355_REG9_VCO_BAND_DIV(x)		    (((x) & 0xFF) << 24)

/* REG10 Bit Definitions */
#define ADF5355_REG10_ADC_EN(x)			        ((x) << 4)
#define ADF5355_REG10_ADC_CONV_EN(x)		    ((x) << 5)
#define ADF5355_REG10_ADC_CLK_DIV(x)		    (((x) & 0xFF) << 6)
#define ADF5355_REG10_DEFAULT			        0x00C0000A

/* REG11 Bit Definitions */
#define ADF5356_REG11_VCO_BAND_HOLD_EN(x)	    ((x) << 24)
#define ADF5355_REG11_DEFAULT			        0x0061300B
#define ADF5356_REG11_DEFAULT			        0x0061200B

/* REG12 Bit Definitions */
#define ADF5355_REG12_PHASE_RESYNC_CLK_DIV(x)	(((x) & 0xFFFF) << 16)
#define ADF5355_REG12_DEFAULT			        0x0000041C
#define ADF5356_REG12_PHASE_RESYNC_CLK_DIV(x)	(((x) & 0xFFFFF) << 12)
#define ADF5356_REG12_DEFAULT			        0x000005FC

/* REG13 Bit Definitions (ADF5356) */
#define ADF5356_REG13_MOD2_MSB(x)		        (((x) & 0x3FFF) << 4)
#define ADF5356_REG13_FRAC2_MSB(x)		        (((x) & 0x3FFF) << 18)

/* Specifications */
#define ADF5355_MIN_VCO_FREQ		            3400000000ULL /* Hz */
#define ADF5355_MAX_VCO_FREQ		            6800000000ULL /* Hz */
#define ADF5355_MAX_OUT_FREQ		            ADF5355_MAX_VCO_FREQ /* Hz */
#define ADF5355_MIN_OUT_FREQ		            (ADF5355_MIN_VCO_FREQ / 64) /* Hz */
#define ADF5355_MAX_OUTB_FREQ		            (ADF5355_MAX_VCO_FREQ * 2) /* Hz */
#define ADF5355_MIN_OUTB_FREQ		            (ADF5355_MIN_VCO_FREQ * 2) /* Hz */

#define ADF4356_MIN_VCO_FREQ                        ADF5355_MIN_VCO_FREQ /* Hz */
#define ADF4356_MAX_VCO_FREQ                        ADF5355_MAX_VCO_FREQ /* Hz */

#define ADF4355_MIN_VCO_FREQ		            3400000000ULL /* Hz */
#define ADF4355_MAX_VCO_FREQ		            6800000000ULL /* Hz */
#define ADF4355_MAX_OUT_FREQ		            ADF4355_MAX_VCO_FREQ /* Hz */
#define ADF4355_MIN_OUT_FREQ		            (ADF4355_MIN_VCO_FREQ / 64) /* Hz */

#define ADF4355_3_MIN_VCO_FREQ		            3300000000ULL /* Hz */
#define ADF4355_3_MAX_VCO_FREQ		            6600000000ULL /* Hz */
#define ADF4355_3_MAX_OUT_FREQ		            ADF4355_3_MAX_VCO_FREQ /* Hz */
#define ADF4355_3_MIN_OUT_FREQ		            (ADF4355_3_MIN_VCO_FREQ / 64) /* Hz */

#define ADF4355_2_MIN_VCO_FREQ		            3400000000ULL /* Hz */
#define ADF4355_2_MAX_VCO_FREQ		            6800000000ULL /* Hz */
#define ADF4355_2_MAX_OUT_FREQ		            4400000000ULL /* Hz */
#define ADF4355_2_MIN_OUT_FREQ		            (ADF4355_2_MIN_VCO_FREQ / 64) /* Hz */

#define ADF5355_MAX_FREQ_PFD		            75000000UL /* Hz */
#define ADF5355_MAX_FREQ_REFIN		            600000000UL /* Hz */
#define ADF5355_MAX_MODULUS2		            16384
#define ADF5356_MAX_MODULUS2		            268435456
#define ADF5355_MAX_R_CNT		                1023

#define ADF5355_MODULUS1			            16777216ULL
#define ADF5355_MIN_INT_PRESCALER_89	        75

#define ADF5355_REG_NUM                         14

#define ADF5355_SPI_NO_BYTES                    4

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `adf5355_device_id` enumeration defines a set of constants
 * representing different devices supported by the ADF5355 driver. Each
 * enumerator corresponds to a specific device model, allowing the
 * software to identify and handle different devices appropriately within
 * the driver code.
 *
 * @param ADF5355 Represents the ADF5355 device.
 * @param ADF4355 Represents the ADF4355 device.
 * @param ADF4355_2 Represents the ADF4355-2 device.
 * @param ADF4355_3 Represents the ADF4355-3 device.
 * @param ADF4356 Represents the ADF4356 device.
 * @param ADF5356 Represents the ADF5356 device.
 ******************************************************************************/
enum adf5355_device_id {
	ADF5355,
	ADF4355,
	ADF4355_2,
	ADF4355_3,
	ADF4356,
	ADF5356,
};

/***************************************************************************//**
 * @brief The `adf5355_mux_out_sel` enumeration defines the possible output
 * selections for the MUXOUT pin of the ADF5355 device. This pin can be
 * configured to output various signals such as three-state, DVDD,
 * ground, R divider output, N divider output, analog lock detect, or
 * digital lock detect. These options allow for flexible configuration of
 * the MUXOUT pin to suit different application needs.
 *
 * @param ADF5355_MUXOUT_THREESTATE Represents a three-state output for the
 * MUXOUT pin.
 * @param ADF5355_MUXOUT_DVDD Selects the DVDD voltage as the output for the
 * MUXOUT pin.
 * @param ADF5355_MUXOUT_GND Connects the MUXOUT pin to ground.
 * @param ADF5355_MUXOUT_R_DIV_OUT Outputs the R divider output on the MUXOUT
 * pin.
 * @param ADF5355_MUXOUT_N_DIV_OUT Outputs the N divider output on the MUXOUT
 * pin.
 * @param ADF5355_MUXOUT_ANALOG_LOCK_DETECT Outputs an analog lock detect signal
 * on the MUXOUT pin.
 * @param ADF5355_MUXOUT_DIGITAL_LOCK_DETECT Outputs a digital lock detect
 * signal on the MUXOUT pin.
 ******************************************************************************/
enum adf5355_mux_out_sel {
	ADF5355_MUXOUT_THREESTATE,
	ADF5355_MUXOUT_DVDD,
	ADF5355_MUXOUT_GND,
	ADF5355_MUXOUT_R_DIV_OUT,
	ADF5355_MUXOUT_N_DIV_OUT,
	ADF5355_MUXOUT_ANALOG_LOCK_DETECT,
	ADF5355_MUXOUT_DIGITAL_LOCK_DETECT,
};

/***************************************************************************//**
 * @brief The `adf5355_dev` structure is a comprehensive descriptor for the
 * ADF5355 device, encapsulating all necessary parameters and settings
 * required to configure and operate the device. It includes pointers for
 * SPI communication, device identification, frequency settings, and
 * various configuration flags for outputs and charge pump settings. This
 * structure is essential for managing the device's operation, including
 * setting frequencies, enabling outputs, and configuring the phase
 * detector and reference inputs. It is designed to support multiple
 * channels and handle complex frequency synthesis tasks, making it
 * suitable for applications requiring precise frequency control.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param dev_id Identifier for the specific ADF5355 device variant.
 * @param all_synced Boolean indicating if all channels are synchronized.
 * @param regs Array holding register values for the device.
 * @param freq_req Requested frequency for the device output.
 * @param freq_req_chan Channel number for the requested frequency.
 * @param num_channels Number of channels available on the device.
 * @param clkin_freq Frequency of the input clock.
 * @param max_out_freq Maximum output frequency supported by the device.
 * @param min_out_freq Minimum output frequency supported by the device.
 * @param min_vco_freq Minimum frequency for the VCO.
 * @param fpfd Frequency of the phase frequency detector.
 * @param integer Integer part of the frequency division.
 * @param fract1 First fractional part of the frequency division.
 * @param fract2 Second fractional part of the frequency division.
 * @param mod2 Modulus for the second fractional part.
 * @param cp_ua Charge pump current in microamperes.
 * @param cp_neg_bleed_en Boolean to enable negative bleed current in charge
 * pump.
 * @param cp_gated_bleed_en Boolean to enable gated bleed current in charge
 * pump.
 * @param cp_bleed_current_polarity_en Boolean to set bleed current polarity in
 * charge pump.
 * @param mute_till_lock_en Boolean to mute output until lock is achieved.
 * @param outa_en Boolean to enable output A.
 * @param outb_en Boolean to enable output B.
 * @param outa_power Power level for output A.
 * @param outb_power Power level for output B.
 * @param phase_detector_polarity_neg Polarity setting for the phase detector.
 * @param ref_diff_en Boolean to enable differential reference input.
 * @param mux_out_3v3_en Boolean to enable 3.3V on MUXOUT.
 * @param outb_sel_fund Boolean to select fundamental frequency for output B.
 * @param ref_doubler_en Boolean to enable reference frequency doubler.
 * @param ref_div2_en Boolean to enable reference frequency division by 2.
 * @param rf_div_sel RF divider selection value.
 * @param ref_div_factor Reference division factor.
 * @param mux_out_sel Selection for MUXOUT functionality.
 * @param delay_us Delay in microseconds for certain operations.
 ******************************************************************************/
struct adf5355_dev {
	struct no_os_spi_desc	*spi_desc;
	enum adf5355_device_id      dev_id;
	bool                        all_synced;
	uint32_t                    regs[ADF5355_REG_NUM];
	uint64_t                    freq_req;
	uint8_t                     freq_req_chan;
	uint8_t                     num_channels;
	uint32_t	                clkin_freq;
	uint64_t                    max_out_freq;
	uint64_t                    min_out_freq;
	uint64_t                    min_vco_freq;
	uint32_t	                fpfd;
	uint32_t	                integer;
	uint32_t	                fract1;
	uint32_t	                fract2;
	uint32_t	                mod2;
	uint32_t                    cp_ua;
	bool                        cp_neg_bleed_en;
	bool                        cp_gated_bleed_en;
	bool                        cp_bleed_current_polarity_en;
	bool		                mute_till_lock_en;
	bool                        outa_en;
	bool                        outb_en;
	uint8_t                     outa_power;
	uint8_t                     outb_power;
	uint8_t                     phase_detector_polarity_neg;
	bool                        ref_diff_en;
	bool                        mux_out_3v3_en;
	bool                        outb_sel_fund;
	uint8_t		                ref_doubler_en;
	uint8_t		                ref_div2_en;
	uint8_t                     rf_div_sel;
	uint16_t                    ref_div_factor;
	enum adf5355_mux_out_sel    mux_out_sel;
	uint32_t                    delay_us;
};

/***************************************************************************//**
 * @brief The `adf5355_init_param` structure is used to initialize the ADF5355
 * device, a wideband synthesizer with integrated VCO. It contains
 * various configuration parameters such as SPI initialization, device
 * ID, requested frequency, input clock frequency, charge pump settings,
 * output enable flags, power levels, and MUXOUT settings. This structure
 * allows for detailed customization of the device's operation to suit
 * specific application requirements.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param dev_id Identifier for the specific ADF5355 device variant.
 * @param freq_req Requested frequency in Hz.
 * @param freq_req_chan Channel number for the requested frequency.
 * @param clkin_freq Frequency of the input clock in Hz.
 * @param cp_ua Charge pump current in microamperes.
 * @param cp_neg_bleed_en Enable negative bleed current in charge pump.
 * @param cp_gated_bleed_en Enable gated bleed current in charge pump.
 * @param cp_bleed_current_polarity_en Enable bleed current polarity in charge
 * pump.
 * @param mute_till_lock_en Mute output until PLL lock is achieved.
 * @param outa_en Enable output A.
 * @param outb_en Enable output B.
 * @param outa_power Power level for output A.
 * @param outb_power Power level for output B.
 * @param phase_detector_polarity_neg Set phase detector polarity to negative.
 * @param ref_diff_en Enable differential reference input.
 * @param mux_out_3v3_en Enable 3.3V on MUXOUT pin.
 * @param ref_doubler_en Enable reference frequency doubler.
 * @param ref_div2_en Enable reference frequency divide by 2.
 * @param mux_out_sel Selection for MUXOUT pin function.
 * @param outb_sel_fund Select fundamental frequency for output B.
 ******************************************************************************/
struct adf5355_init_param {
	struct no_os_spi_init_param	*spi_init;
	enum adf5355_device_id      dev_id;
	uint64_t                    freq_req;
	uint8_t                     freq_req_chan;
	uint32_t	                clkin_freq;
	uint32_t                    cp_ua;
	bool                        cp_neg_bleed_en;
	bool                        cp_gated_bleed_en;
	bool                        cp_bleed_current_polarity_en;
	bool		                mute_till_lock_en;
	bool                        outa_en;
	bool                        outb_en;
	uint8_t                     outa_power;
	uint8_t                     outb_power;
	bool                        phase_detector_polarity_neg;
	bool                        ref_diff_en;
	bool                        mux_out_3v3_en;
	uint8_t		                ref_doubler_en;
	uint8_t		                ref_div2_en;
	enum adf5355_mux_out_sel    mux_out_sel;
	bool                        outb_sel_fund;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Recalculate rate corresponding to a channel. */
/***************************************************************************//**
 * @brief Use this function to obtain the current frequency rate of a specified
 * channel on the ADF5355 device. It is essential to ensure that the
 * channel number provided is within the valid range of channels
 * supported by the device. This function should be called when you need
 * to verify or utilize the current frequency setting of a channel. It
 * does not modify the device state but provides the current rate through
 * the output parameter.
 *
 * @param dev A pointer to an adf5355_dev structure representing the device.
 * Must not be null, and the device should be properly initialized
 * before calling this function.
 * @param chan The channel number for which the rate is to be recalculated. Must
 * be within the range of available channels (0 to dev->num_channels
 * - 1). If the channel number is invalid, the function returns an
 * error.
 * @param rate A pointer to a uint64_t where the recalculated rate will be
 * stored. Must not be null. The function writes the current rate of
 * the specified channel to this location.
 * @return Returns 0 on success, and -1 if the channel number is invalid.
 ******************************************************************************/
int32_t adf5355_clk_recalc_rate(struct adf5355_dev *dev, uint32_t chan,
				uint64_t *rate);

/* Set channel rate. */
/***************************************************************************//**
 * @brief This function sets the output frequency for a specified channel on the
 * ADF5355 device. It should be called when you need to change the
 * frequency output of a particular channel. The function requires that
 * the channel index is within the valid range of available channels for
 * the device. If the channel index is invalid, the function returns an
 * error code. This function is typically used after the device has been
 * initialized and when a frequency change is needed.
 *
 * @param dev A pointer to an initialized adf5355_dev structure representing the
 * device. Must not be null.
 * @param chan The channel index for which the frequency is to be set. Must be
 * less than the number of channels available on the device. If
 * invalid, the function returns an error.
 * @param rate The desired frequency rate to set for the specified channel, in
 * Hertz. Must be within the valid frequency range supported by the
 * device.
 * @return Returns 0 on success, or -1 if the channel index is invalid.
 ******************************************************************************/
int32_t adf5355_clk_set_rate(struct adf5355_dev *dev, uint32_t chan,
			     uint64_t rate);

/* Calculate closest possible rate */
/***************************************************************************//**
 * @brief This function is used to determine the closest achievable frequency
 * rate for a given desired rate on an ADF5355 device. It is typically
 * called when a user needs to find the nearest valid frequency that the
 * device can output, based on its internal constraints and settings. The
 * function does not modify the device state or configuration, and it
 * should be called with a valid device structure. The result is returned
 * through a pointer to a variable where the rounded rate will be stored.
 *
 * @param dev A pointer to an adf5355_dev structure representing the device.
 * Must not be null.
 * @param rate The desired frequency rate in Hz. There are no specific
 * constraints on this value, but it should be within the
 * operational range of the device for meaningful results.
 * @param rounded_rate A pointer to a uint64_t where the function will store the
 * closest possible rate. Must not be null.
 * @return Returns 0 on success, indicating that the rounded rate has been
 * calculated and stored in the provided location.
 ******************************************************************************/
int32_t adf5355_clk_round_rate(struct adf5355_dev *dev, uint64_t rate,
			       uint64_t *rounded_rate);

/* Initializes the ADF5355. */
/***************************************************************************//**
 * @brief This function sets up and initializes an ADF5355 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * allocates memory for the device structure and initializes the SPI
 * interface based on the parameters provided. It also configures various
 * device settings such as frequency, power, and output channels. If
 * initialization fails at any step, the function will clean up and
 * return an error code. Ensure that the `init_param` structure is
 * correctly populated with valid values before calling this function.
 *
 * @param device A pointer to a pointer of type `struct adf5355_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure.
 * @param init_param A pointer to a `struct adf5355_init_param` containing the
 * initialization parameters. This structure must be fully
 * populated with valid configuration values before calling
 * the function. The caller retains ownership of this
 * structure.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure or SPI initialization failure.
 ******************************************************************************/
int32_t adf5355_init(struct adf5355_dev **device,
		     const struct adf5355_init_param *init_param);

/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly deinitialize and free resources
 * associated with an ADF5355 device when it is no longer needed. This
 * function should be called to clean up after the device has been
 * initialized and used, ensuring that any allocated resources are
 * released. It is important to call this function to prevent memory
 * leaks and to ensure that the SPI descriptor associated with the device
 * is also removed if it exists.
 *
 * @param device A pointer to an adf5355_dev structure representing the device
 * to be removed. This pointer must not be null, and the function
 * assumes ownership of the memory, which will be freed. If the
 * device's spi_desc is non-null, it will be removed as part of
 * the cleanup process.
 * @return Returns an int32_t indicating the success of the operation. A return
 * value of 0 indicates success, while a non-zero value indicates an
 * error occurred during the removal of the SPI descriptor.
 ******************************************************************************/
int32_t adf5355_remove(struct adf5355_dev *device);
