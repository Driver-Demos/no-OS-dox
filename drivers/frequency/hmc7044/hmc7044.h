/***************************************************************************//**
 *   @file   hmc7044.h
 *   @brief  Header file of HMC7044, HMC7043 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2018-2020(c) Analog Devices, Inc.
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
#ifndef HMC7044_H_
#define HMC7044_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include "no_os_delay.h"
#include "no_os_spi.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `hmc7044_chan_spec` structure defines the configuration for a
 * single channel of the HMC7044 device, which is a clock generator and
 * jitter attenuator. It includes various fields to control the channel's
 * operational parameters such as enabling/disabling the channel, setting
 * performance modes, configuring delays, and specifying driver settings.
 * This structure is crucial for managing the behavior of each channel in
 * the HMC7044 device, allowing for precise control over clock outputs.
 *
 * @param num Specifies the channel number.
 * @param disable Indicates if the channel is disabled.
 * @param high_performance_mode_dis Disables high performance mode for the
 * channel.
 * @param start_up_mode_dynamic_enable Enables dynamic start-up mode for the
 * channel.
 * @param dynamic_driver_enable Enables dynamic driver for the channel.
 * @param output_control0_rb4_enable Enables output control register bit 4.
 * @param force_mute_enable Enables force mute for the channel.
 * @param is_sysref Indicates if the channel is a SYSREF channel.
 * @param divider Specifies the divider value for the channel.
 * @param driver_mode Specifies the driver mode for the channel.
 * @param driver_impedance Specifies the driver impedance for the channel.
 * @param coarse_delay Specifies the coarse delay for the channel.
 * @param fine_delay Specifies the fine delay for the channel.
 * @param out_mux_mode Specifies the output multiplexer mode for the channel.
 ******************************************************************************/
struct hmc7044_chan_spec {
	unsigned int	num;
	bool		disable;
	bool		high_performance_mode_dis;
	bool		start_up_mode_dynamic_enable;
	bool		dynamic_driver_enable;
	bool		output_control0_rb4_enable;
	bool		force_mute_enable;
	bool		is_sysref;
	unsigned int	divider;
	unsigned int	driver_mode;
	unsigned int	driver_impedance;
	unsigned int	coarse_delay;
	unsigned int	fine_delay;
	unsigned int	out_mux_mode;
};

/***************************************************************************//**
 * @brief The `hmc7044_dev` structure is a comprehensive representation of the
 * HMC7044/HMC7043 clock generator device, encapsulating all necessary
 * parameters and configurations for its operation. It includes SPI
 * communication details, clock input frequencies, PLL configurations,
 * and various control flags for enabling or disabling specific features.
 * The structure also manages channel specifications and integrates with
 * JESD204 devices, providing detailed control over synchronization and
 * clock distribution. This structure is essential for initializing and
 * managing the HMC7044/HMC7043 device in a system, ensuring precise
 * clock generation and distribution.
 *
 * @param spi_desc Pointer to the SPI descriptor for communication.
 * @param clk_desc Array of pointers to clock descriptors.
 * @param is_hmc7043 Boolean indicating if the device is HMC7043.
 * @param clkin_freq Array storing input clock frequencies.
 * @param clkin_freq_ccf Array storing input clock frequencies for CCF.
 * @param vcxo_freq Frequency of the VCXO.
 * @param pll1_pfd Phase frequency detector frequency for PLL1.
 * @param pfd1_limit Limit for the phase frequency detector 1.
 * @param pll1_cp_current Charge pump current for PLL1.
 * @param pll2_freq Frequency for PLL2.
 * @param pll1_loop_bw Loop bandwidth for PLL1.
 * @param sysref_timer_div Divider for the SYSREF timer.
 * @param pll1_ref_prio_ctrl Control for PLL1 reference priority.
 * @param pll1_ref_autorevert_en Boolean to enable autorevert for PLL1
 * reference.
 * @param clkin0_rfsync_en Boolean to enable RF sync on clock input 0.
 * @param clkin1_vcoin_en Boolean to enable VCO input on clock input 1.
 * @param high_performance_mode_clock_dist_en Boolean to enable high performance
 * mode for clock distribution.
 * @param rf_reseeder_en Boolean to enable RF reseeder.
 * @param sync_pin_mode Mode for the sync pin.
 * @param pulse_gen_mode Mode for the pulse generator.
 * @param in_buf_mode Array defining input buffer modes.
 * @param gpi_ctrl Array for general purpose input control.
 * @param gpo_ctrl Array for general purpose output control.
 * @param num_channels Number of channels available.
 * @param channels Pointer to an array of channel specifications.
 * @param jdev Pointer to a JESD204 device structure.
 * @param jdev_lmfc_lemc_rate Rate for JESD204 LMFC/LEMC.
 * @param jdev_lmfc_lemc_gcd GCD for JESD204 LMFC/LEMC.
 * @param jdev_max_sysref_freq Maximum SYSREF frequency for JESD204.
 * @param jdev_desired_sysref_freq Desired SYSREF frequency for JESD204.
 * @param jdev_skip_sysref_freq_calc Boolean to skip SYSREF frequency
 * calculation for JESD204.
 * @param is_sysref_provider Boolean indicating if the device is a SYSREF
 * provider.
 * @param hmc_two_level_tree_sync_en Boolean to enable two-level tree
 * synchronization.
 * @param read_write_confirmed Boolean indicating if read/write operations are
 * confirmed.
 ******************************************************************************/
struct hmc7044_dev {
	struct no_os_spi_desc	*spi_desc;
	/* CLK descriptors */
	struct no_os_clk_desc	**clk_desc;
	bool		is_hmc7043;
	uint32_t	clkin_freq[4];
	uint32_t	clkin_freq_ccf[4];
	uint32_t	vcxo_freq;
	uint32_t	pll1_pfd;
	uint32_t	pfd1_limit;
	uint32_t	pll1_cp_current;
	uint32_t	pll2_freq;
	uint32_t	pll1_loop_bw;
	uint32_t	sysref_timer_div;
	unsigned int	pll1_ref_prio_ctrl;
	bool		pll1_ref_autorevert_en;
	bool		clkin0_rfsync_en;
	bool		clkin1_vcoin_en;
	bool		high_performance_mode_clock_dist_en;
	bool		rf_reseeder_en;
	unsigned int	sync_pin_mode;
	uint32_t	pulse_gen_mode;
	uint32_t	in_buf_mode[5];
	uint32_t	gpi_ctrl[4];
	uint32_t	gpo_ctrl[4];
	uint32_t	num_channels;
	struct hmc7044_chan_spec	*channels;
	struct jesd204_dev *jdev;
	uint32_t			jdev_lmfc_lemc_rate;
	uint32_t			jdev_lmfc_lemc_gcd;
	uint32_t			jdev_max_sysref_freq;
	uint32_t			jdev_desired_sysref_freq;
	bool				jdev_skip_sysref_freq_calc;
	bool				is_sysref_provider;
	bool				hmc_two_level_tree_sync_en;
	bool				read_write_confirmed;
};

/***************************************************************************//**
 * @brief The `hmc7044_init_param` structure is used to initialize the HMC7044
 * or HMC7043 clock generator devices. It contains various configuration
 * parameters such as SPI initialization, clock input frequencies, PLL
 * settings, and control flags for enabling or disabling specific
 * features. This structure allows for detailed customization of the
 * clock generator's behavior, including its role as a JESD204 SYSREF
 * provider, synchronization settings, and channel-specific
 * configurations.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param export_no_os_clk Boolean flag to export no-OS clock.
 * @param is_hmc7043 Boolean flag indicating if the device is HMC7043.
 * @param clkin_freq Array of input clock frequencies.
 * @param clkin_freq_ccf Array of input clock frequencies for CCF.
 * @param vcxo_freq Frequency of the VCXO.
 * @param pll1_pfd Phase frequency detector frequency for PLL1.
 * @param pfd1_limit Limit for the phase frequency detector 1.
 * @param pll1_cp_current Charge pump current for PLL1.
 * @param pll2_freq Frequency for PLL2.
 * @param pll1_loop_bw Loop bandwidth for PLL1.
 * @param sysref_timer_div Divider for the SYSREF timer.
 * @param pll1_ref_prio_ctrl Priority control for PLL1 reference.
 * @param pll1_ref_autorevert_en Boolean flag to enable PLL1 reference
 * autorevert.
 * @param clkin0_rfsync_en Boolean flag to enable RF sync on clock input 0.
 * @param clkin1_vcoin_en Boolean flag to enable VCO input on clock input 1.
 * @param high_performance_mode_clock_dist_en Boolean flag to enable high
 * performance mode for clock
 * distribution.
 * @param rf_reseeder_disable Boolean flag to disable RF reseeder.
 * @param hmc_two_level_tree_sync_en Boolean flag to enable two-level tree
 * synchronization.
 * @param jesd204_sysref_provider Boolean flag indicating if the device is a
 * JESD204 SYSREF provider.
 * @param jesd204_max_sysref_frequency_hz Maximum SYSREF frequency for JESD204
 * in Hz.
 * @param jesd204_desired_sysref_frequency_hz Desired SYSREF frequency for
 * JESD204 in Hz.
 * @param jdev_skip_sysref_freq_calc Boolean flag to skip SYSREF frequency
 * calculation for JESD204.
 * @param sync_pin_mode Mode for the sync pin.
 * @param pulse_gen_mode Mode for the pulse generator.
 * @param in_buf_mode Array of input buffer modes.
 * @param gpi_ctrl Array of general-purpose input controls.
 * @param gpo_ctrl Array of general-purpose output controls.
 * @param num_channels Number of channels.
 * @param channels Pointer to an array of channel specifications.
 ******************************************************************************/
struct hmc7044_init_param {
	struct no_os_spi_init_param	*spi_init;
	/* Init CLK channel descriptors */
	bool		export_no_os_clk;
	bool		is_hmc7043;
	uint32_t	clkin_freq[4];
	uint32_t	clkin_freq_ccf[4];
	uint32_t	vcxo_freq;
	uint32_t	pll1_pfd;
	uint32_t	pfd1_limit;
	uint32_t	pll1_cp_current;
	uint32_t	pll2_freq;
	uint32_t	pll1_loop_bw;
	uint32_t	sysref_timer_div;
	unsigned int	pll1_ref_prio_ctrl;
	bool		pll1_ref_autorevert_en;
	bool		clkin0_rfsync_en;
	bool		clkin1_vcoin_en;
	bool		high_performance_mode_clock_dist_en;
	bool		rf_reseeder_disable;
	bool		hmc_two_level_tree_sync_en;
	bool		jesd204_sysref_provider;
	uint32_t	jesd204_max_sysref_frequency_hz;
	uint32_t	jesd204_desired_sysref_frequency_hz;
	bool		jdev_skip_sysref_freq_calc;
	unsigned int	sync_pin_mode;
	uint32_t	pulse_gen_mode;
	uint32_t	in_buf_mode[5];
	uint32_t	gpi_ctrl[4];
	uint32_t	gpo_ctrl[4];
	uint32_t	num_channels;
	struct hmc7044_chan_spec	*channels;
};

/***************************************************************************//**
 * @brief The `hmc7044_clk_ops` is a constant structure of type
 * `no_os_clk_platform_ops` that defines the clock platform operations
 * specific to the HMC7044 device. This structure is used to interface
 * with the clock functionalities provided by the HMC7044 driver.
 *
 * @details This variable is used to provide a set of operations for managing
 * clock functionalities in the HMC7044 driver.
 ******************************************************************************/
extern const struct no_os_clk_platform_ops hmc7044_clk_ops;

/*
 * adi,pulse-generator-mode
 */
#define HMC7044_PULSE_GEN_LEVEL_SENSITIVE	0
#define HMC7044_PULSE_GEN_1_PULSE		1
#define HMC7044_PULSE_GEN_2_PULSE		2
#define HMC7044_PULSE_GEN_4_PULSE		3
#define HMC7044_PULSE_GEN_8_PULSE		4
#define HMC7044_PULSE_GEN_16_PULSE		5
#define HMC7044_PULSE_GEN_CONT_PULSE		7

/*
 * adi,sync-pin-mode
 */

#define HMC7044_SYNC_PIN_DISABLED		        0
#define HMC7044_SYNC_PIN_SYNC   		        1
#define HMC7044_SYNC_PIN_PULSE_GEN_REQ	        2
#define HMC7044_SYNC_PIN_SYNC_THEN_PULSE_GEN	3

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes an HMC7044 or HMC7043 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function sets up the device's SPI
 * interface, configures clock channels if required, and initializes
 * various device parameters based on the input structure. It handles
 * both HMC7044 and HMC7043 devices, determining the appropriate setup
 * routine to call. The caller must ensure that the `init_param`
 * structure is correctly populated with valid configuration data before
 * calling this function. On success, a pointer to the initialized device
 * structure is returned through the `device` parameter.
 *
 * @param device A pointer to a pointer of type `struct hmc7044_dev`. This will
 * be set to point to the newly initialized device structure. Must
 * not be null.
 * @param init_param A pointer to a `struct hmc7044_init_param` containing the
 * initialization parameters for the device. Must not be null
 * and should be properly initialized with valid configuration
 * data.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int32_t hmc7044_init(struct hmc7044_dev **device,
		     const struct hmc7044_init_param *init_param);
/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly remove an HMC7044 device instance and
 * free associated resources. It should be called when the device is no
 * longer needed, ensuring that all allocated memory and resources are
 * released. This function must be called only after the device has been
 * successfully initialized and used, to prevent resource leaks. It is
 * important to ensure that the `device` parameter is valid and not null
 * before calling this function.
 *
 * @param device A pointer to the `hmc7044_dev` structure representing the
 * device to be removed. Must not be null. The function will free
 * the memory associated with this structure and its components.
 * @return Returns an `int32_t` indicating the success or failure of the
 * operation. A non-zero return value indicates an error occurred during
 * the removal process.
 ******************************************************************************/
int32_t hmc7044_remove(struct hmc7044_dev *device);
/***************************************************************************//**
 * @brief Use this function to read a specific register from the HMC7044 device.
 * It requires a valid device structure and a register address to read
 * from. The function will store the read value in the provided memory
 * location. Ensure that the device has been properly initialized before
 * calling this function. The function returns an error code if the read
 * operation fails, which can be used for error handling.
 *
 * @param dev A pointer to an initialized hmc7044_dev structure representing the
 * device. Must not be null.
 * @param reg The 16-bit address of the register to read from. Valid register
 * addresses depend on the device specification.
 * @param val A pointer to a uint8_t where the read value will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int32_t hmc7044_read(struct hmc7044_dev *dev, uint16_t reg, uint8_t *val);
/***************************************************************************//**
 * @brief Use this function to determine the current clock rate of a specific
 * channel in the HMC7044 device. It is essential to call this function
 * when you need to verify or utilize the current clock rate of a channel
 * after initialization or configuration changes. The function requires a
 * valid device structure and a channel number that exists within the
 * device's channel list. If the specified channel number is not found,
 * the function returns an error code.
 *
 * @param dev A pointer to an hmc7044_dev structure representing the device.
 * Must not be null and should be properly initialized before calling
 * this function.
 * @param chan_num The channel number for which the clock rate is to be
 * recalculated. It should correspond to a valid channel number
 * within the device's channel list.
 * @param rate A pointer to a uint64_t where the recalculated clock rate will be
 * stored. Must not be null.
 * @return Returns 0 on success, with the recalculated rate stored in the
 * provided rate pointer. Returns -1 if the specified channel number is
 * not found.
 ******************************************************************************/
int32_t hmc7044_clk_recalc_rate(struct hmc7044_dev *dev, uint32_t chan_num,
				uint64_t *rate);
/***************************************************************************//**
 * @brief Use this function to determine the closest achievable clock rate for a
 * given desired rate on the HMC7044 device. This is useful when you need
 * to configure the device to operate at a frequency that is as close as
 * possible to a specified target. The function requires a valid device
 * structure and a desired rate, and it outputs the rounded rate through
 * a pointer. Ensure that the device has been properly initialized before
 * calling this function.
 *
 * @param dev A pointer to an initialized `hmc7044_dev` structure representing
 * the device. Must not be null.
 * @param rate The desired clock rate in Hz. Must be a positive integer.
 * @param rounded_rate A pointer to a `uint64_t` where the function will store
 * the rounded clock rate. Must not be null.
 * @return Returns 0 on success. The rounded clock rate is stored in the
 * location pointed to by `rounded_rate`.
 ******************************************************************************/
int32_t hmc7044_clk_round_rate(struct hmc7044_dev *dev, uint32_t rate,
			       uint64_t *rounded_rate);
/***************************************************************************//**
 * @brief This function configures the output clock rate for a specific channel
 * on the HMC7044 device. It should be called when there is a need to
 * change the frequency of a particular channel's output. The function
 * requires a valid device structure and a channel number that exists
 * within the device's configuration. If the specified channel is not
 * found, the function returns an error. The rate is set by calculating
 * and applying the appropriate divider based on the desired frequency
 * and the device's PLL2 frequency. This function must be called after
 * the device has been properly initialized.
 *
 * @param dev A pointer to an initialized hmc7044_dev structure representing the
 * device. Must not be null.
 * @param chan_num The channel number for which the clock rate is to be set. It
 * must correspond to a valid channel within the device's
 * configuration.
 * @param rate The desired output clock rate in Hz. The function calculates the
 * necessary divider to achieve this rate based on the device's PLL2
 * frequency.
 * @return Returns 0 on success, or a negative error code if the channel is not
 * found or if there is a failure in writing the configuration.
 ******************************************************************************/
int32_t hmc7044_clk_set_rate(struct hmc7044_dev *dev, uint32_t chan_num,
			     uint64_t rate);

#endif // HMC7044_H_
