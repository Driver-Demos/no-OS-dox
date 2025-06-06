/***************************************************************************//**
 *   @file   adf4371.h
 *   @brief  Header file of ADF4371 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2020(c) Analog Devices, Inc.
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
#ifndef ADF4371_H_
#define ADF4371_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "no_os_spi.h"

/******************************************************************************/
/********************** Macros and Types Declarations *************************/
/***************************************************************************//**
 * @brief The `adf4371_channel_config` structure is used to define the
 * configuration for a channel in the ADF4371 device. It includes a
 * boolean flag to enable or disable the channel and a 64-bit unsigned
 * integer to specify the frequency at which the channel operates. This
 * structure is part of the configuration settings for the ADF4371, a
 * frequency synthesizer device, and is used to manage individual channel
 * settings within the device.
 *
 * @param enable A boolean flag indicating whether the channel is enabled.
 * @param freq A 64-bit unsigned integer representing the frequency of the
 * channel.
 ******************************************************************************/
struct adf4371_channel_config {
	bool		enable;
	uint64_t	freq;
};

/***************************************************************************//**
 * @brief The `adf4371_cp_settings` structure is used to configure the charge
 * pump settings for the ADF4371 device. It contains two members: `icp`,
 * which specifies the charge pump current in microamperes, and `regval`,
 * which stores the corresponding register value for these settings. This
 * structure is essential for setting up the charge pump parameters that
 * influence the phase-locked loop (PLL) performance in the ADF4371
 * frequency synthesizer.
 *
 * @param icp Represents the charge pump current setting in microamperes.
 * @param regval Holds the register value associated with the charge pump
 * settings.
 ******************************************************************************/
struct adf4371_cp_settings {
	uint32_t	icp;
	uint32_t	regval;
};

/***************************************************************************//**
 * @brief The `adf4371_chan_spec` structure defines the specifications for a
 * channel in the ADF4371 device, including its unique channel number and
 * the frequency it should be set to upon powering up. This structure is
 * used to configure individual channels within the ADF4371 device, which
 * is a frequency synthesizer used in RF applications.
 *
 * @param num A 32-bit unsigned integer representing the channel number.
 * @param power_up_frequency A 64-bit unsigned integer representing the
 * frequency at which the channel powers up.
 ******************************************************************************/
struct adf4371_chan_spec {
	uint32_t	num;
	uint64_t	power_up_frequency;
};

/***************************************************************************//**
 * @brief The `adf4371_dev` structure is a comprehensive configuration and state
 * representation for the ADF4371 frequency synthesizer device. It
 * includes SPI communication settings, channel configurations, charge
 * pump settings, and various operational parameters such as reference
 * clock settings, frequency dividers, and output control options. This
 * structure is essential for initializing and controlling the ADF4371
 * device, allowing for precise frequency synthesis and modulation
 * control.
 *
 * @param spi_desc Pointer to a SPI descriptor for communication.
 * @param spi_3wire_en Boolean flag to enable 3-wire SPI mode.
 * @param num_channels Number of channels configured in the device.
 * @param channel_cfg Array of channel configurations, each with enable status
 * and frequency.
 * @param cp_settings Charge pump settings including current and register value.
 * @param differential_ref_clk Boolean flag to enable differential reference
 * clock.
 * @param pd_pol Power-down polarity setting.
 * @param mute_till_lock_en Boolean flag to mute output until lock is achieved.
 * @param muxout_default_mode Default mode for the MUXOUT pin.
 * @param muxout_en Boolean flag to enable MUXOUT functionality.
 * @param muxout_1v8_en Boolean flag to enable 1.8V level for MUXOUT.
 * @param ref_div_factor Reference divider factor.
 * @param clkin_freq Input clock frequency.
 * @param fpfd Phase frequency detector frequency.
 * @param integer Integer part of the frequency setting.
 * @param fract1 First fractional part of the frequency setting.
 * @param fract2 Second fractional part of the frequency setting.
 * @param mod2 Modulus value for the second fractional part.
 * @param rf_div_sel RF divider selection.
 * @param buf Buffer for temporary data storage.
 ******************************************************************************/
struct adf4371_dev {
	struct no_os_spi_desc	*spi_desc;
	bool		spi_3wire_en;
	uint32_t	num_channels;
	struct adf4371_channel_config	channel_cfg[4];
	struct adf4371_cp_settings	cp_settings;
	bool		differential_ref_clk;
	uint32_t	pd_pol;
	bool		mute_till_lock_en;
	uint32_t	muxout_default_mode;
	bool		muxout_en;
	bool		muxout_1v8_en;
	uint32_t	ref_div_factor;
	uint32_t	clkin_freq;
	uint32_t	fpfd;
	uint32_t	integer;
	uint32_t	fract1;
	uint32_t	fract2;
	uint32_t	mod2;
	uint32_t	rf_div_sel;
	uint8_t		buf[10];
};

/***************************************************************************//**
 * @brief The `adf4371_init_param` structure is used to initialize the ADF4371
 * device, a frequency synthesizer, with specific configuration
 * parameters. It includes settings for SPI communication, clock input
 * frequency, reference clock type, loop filter configuration, charge
 * pump current, and MUXOUT pin functionality. Additionally, it allows
 * for the configuration of multiple channels through an array of channel
 * specifications, making it versatile for various applications requiring
 * precise frequency control.
 *
 * @param spi_init Pointer to the SPI initialization parameters.
 * @param spi_3wire_enable Boolean flag to enable 3-wire SPI mode.
 * @param clkin_frequency Frequency of the input clock in Hertz.
 * @param differential_ref_clock Boolean flag to enable differential reference
 * clock.
 * @param loop_filter_inverting Boolean flag to indicate if the loop filter is
 * inverting.
 * @param charge_pump_microamp Charge pump current in microamperes.
 * @param mute_till_lock_enable Boolean flag to enable mute until lock feature.
 * @param muxout_select Selection for the MUXOUT pin function.
 * @param muxout_level_1v8_enable Boolean flag to enable 1.8V level for MUXOUT.
 * @param num_channels Number of channels to be configured.
 * @param channels Pointer to an array of channel specifications.
 ******************************************************************************/
struct adf4371_init_param {
	struct no_os_spi_init_param	*spi_init;
	bool		spi_3wire_enable;
	uint32_t	clkin_frequency;
	bool		differential_ref_clock;
	bool		loop_filter_inverting;
	uint32_t	charge_pump_microamp;
	bool		mute_till_lock_enable;
	uint32_t	muxout_select;
	bool		muxout_level_1v8_enable;
	uint32_t	num_channels;
	struct adf4371_chan_spec	*channels;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up the ADF4371 device using the provided
 * initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure, initializes the SPI interface, and configures various
 * device settings such as the charge pump current, reference clock, and
 * channel configurations. If initialization fails at any step, the
 * function returns an error code and ensures that any allocated
 * resources are properly released.
 *
 * @param device A pointer to a pointer of type `struct adf4371_dev`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated device structure upon successful initialization.
 * @param init_param A pointer to a constant `struct adf4371_init_param`
 * containing the initialization parameters. This includes SPI
 * settings, clock configurations, and channel specifications.
 * The pointer must not be null, and the structure must be
 * fully populated with valid data.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and does not modify the `device` pointer.
 ******************************************************************************/
int32_t adf4371_init(struct adf4371_dev **device,
		     const struct adf4371_init_param *init_param);

/* Remove the device. */
/***************************************************************************//**
 * @brief Use this function to properly remove an ADF4371 device instance and
 * free any resources associated with it. This function should be called
 * when the device is no longer needed, ensuring that any allocated
 * memory is released and any open SPI connections are closed. It is
 * important to ensure that the device pointer is valid and was
 * previously initialized using `adf4371_init`. Failure to call this
 * function may result in resource leaks.
 *
 * @param device A pointer to the `adf4371_dev` structure representing the
 * device to be removed. Must not be null and should point to a
 * valid, initialized device instance. If the `spi_desc` member is
 * non-null, the function will attempt to remove the SPI
 * descriptor.
 * @return Returns an integer status code. A return value of 0 indicates
 * success, while a negative value indicates an error occurred during
 * the removal process.
 ******************************************************************************/
int32_t adf4371_remove(struct adf4371_dev *device);

/* Recalculate rate corresponding to a channel. */
/***************************************************************************//**
 * @brief This function is used to determine the current frequency rate of a
 * specified channel on the ADF4371 device. It should be called when the
 * user needs to retrieve the current operating frequency of a channel.
 * The function requires a valid device structure and a channel number
 * within the range of available channels. If the channel number is
 * invalid, the function returns an error code. The recalculated rate is
 * stored in the provided rate pointer.
 *
 * @param dev A pointer to an initialized adf4371_dev structure representing the
 * device. Must not be null.
 * @param chan The channel number for which the rate is to be recalculated. Must
 * be less than or equal to the number of channels available in the
 * device. If invalid, the function returns an error.
 * @param rate A pointer to a uint64_t where the recalculated rate will be
 * stored. Must not be null.
 * @return Returns 0 on success, or -1 if the channel number is invalid.
 ******************************************************************************/
int32_t adf4371_clk_recalc_rate(struct adf4371_dev *dev, uint32_t chan,
				uint64_t *rate);

/* Calculate closest possible rate */
/***************************************************************************//**
 * @brief This function is used to determine the closest achievable clock rate
 * for the ADF4371 device based on a desired rate. It is typically called
 * when a specific clock rate is required, and the function will provide
 * the nearest possible rate that the device can support. This function
 * does not modify the device state or configuration; it only calculates
 * the rounded rate. It must be called with a valid device structure and
 * a non-null pointer for the rounded rate.
 *
 * @param dev A pointer to an initialized adf4371_dev structure representing the
 * device. Must not be null.
 * @param rate The desired clock rate in Hz. There are no specific constraints
 * on the value, but it should be within the operational range of
 * the device for meaningful results.
 * @param rounded_rate A pointer to a uint64_t where the function will store the
 * closest possible rate. Must not be null.
 * @return Returns 0 on success, indicating that the rounded rate has been
 * calculated and stored in the provided pointer.
 ******************************************************************************/
int32_t adf4371_clk_round_rate(struct adf4371_dev *dev, uint64_t rate,
			       uint64_t *rounded_rate);

/* Set channel rate. */
/***************************************************************************//**
 * @brief Use this function to set the frequency rate for a specific channel on
 * the ADF4371 device. It is essential to ensure that the channel index
 * provided is within the valid range of available channels for the
 * device. This function should be called after the device has been
 * properly initialized. If the channel index is invalid, the function
 * will return an error code.
 *
 * @param dev A pointer to an initialized adf4371_dev structure representing the
 * device. Must not be null.
 * @param chan The index of the channel for which the rate is to be set. Must be
 * less than the number of channels available in the device.
 * @param rate The desired frequency rate to set for the specified channel. Must
 * be a valid frequency value supported by the device.
 * @return Returns 0 on success or -1 if the channel index is invalid.
 ******************************************************************************/
int32_t adf4371_clk_set_rate(struct adf4371_dev *dev, uint32_t chan,
			     uint64_t rate);

#endif
