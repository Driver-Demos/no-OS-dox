/***************************************************************************//**
 *   @file   admfm2000.h
 *   @brief  Header file for admfm2000 Driver.
 *   @author Ramona Nechita (ramona.nechita@analog.com)
********************************************************************************
 * Copyright 2025(c) Analog Devices, Inc.
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

#ifndef SRC_ADMFM2000_H_
#define SRC_ADMFM2000_H_

#include <stdint.h>
#include "no_os_gpio.h"

#define ADMFM2000_MIXER_MODE		0
#define ADMFM2000_DIRECT_IF_MODE	1
#define ADMFM2000_DSA_GPIOS		5
#define ADMFM2000_MODE_GPIOS		2
#define ADMFM2000_MAX_GAIN		0
#define ADMFM2000_MIN_GAIN		-31000
#define ADMFM2000_MAX_GAIN_RAW		31
#define ADMFM2000_MIN_GAIN_RAW		0
#define ADMFM2000_DEFAULT_GAIN		-0x20
#define ADMFM2000_NUM_CHANNELS		2

/***************************************************************************//**
 * @brief The `admfm2000_init_param` structure is used to initialize the
 * ADMFM2000 device, specifying the mixer mode, gain settings, and GPIO
 * configurations for both switch and DSA control across two channels. It
 * includes parameters for setting up the device's operational mode and
 * gain, as well as configuring the GPIOs necessary for controlling the
 * device's functionality.
 *
 * @param mixer_mode Specifies the mixer mode as an 8-bit unsigned integer.
 * @param dsa_gain Defines the gain value as a 32-bit signed integer.
 * @param gpio_sw_param An array of pointers to GPIO initialization parameters
 * for switch control on channels 0 and 1.
 * @param gpio_dsa_param An array of pointers to GPIO initialization parameters
 * for DSA control on channels 0 and 1.
 ******************************************************************************/
struct admfm2000_init_param {
	/* Mixer Mode */
	uint8_t mixer_mode;
	/* GAIN */
	int32_t dsa_gain;
	/* GPIO Control Switch chan 0&1 */
	struct no_os_gpio_init_param *gpio_sw_param[2][2];
	/* GPIO Control DSA chan 0&1 */
	struct no_os_gpio_init_param *gpio_dsa_param[2][5];
};

/***************************************************************************//**
 * @brief The `admfm2000_dev` structure is designed to manage GPIO controls for
 * an ADMFM2000 device, specifically handling switch and DSA (Digital
 * Step Attenuator) controls for two channels. It contains arrays of GPIO
 * descriptors, which are pointers to structures that describe the state
 * and configuration of GPIO pins used for these controls. This structure
 * is essential for interfacing with the hardware components of the
 * ADMFM2000, allowing for the configuration and management of its
 * operational modes and gain settings.
 *
 * @param gpio_sw An array of GPIO descriptors for controlling switches on
 * channels 0 and 1.
 * @param gpio_dsa An array of GPIO descriptors for controlling DSA on channels
 * 0 and 1.
 ******************************************************************************/
struct admfm2000_dev {
	/* GPIO Control Switch chan 0&1 */
	struct no_os_gpio_desc *gpio_sw[2][2];
	/* GPIO Control DSA chan 0&1 */
	struct no_os_gpio_desc *gpio_dsa[2][5];
};

/***************************************************************************//**
 * @brief This function initializes an ADMFM2000 device using the provided
 * initialization parameters. It sets up the necessary GPIO
 * configurations for the device's channels and applies the specified
 * mixer mode and gain settings. The function must be called before any
 * other operations on the device. If initialization is successful, a
 * pointer to the device structure is returned through the provided
 * pointer. The caller is responsible for ensuring that the `init_param`
 * is properly configured and that the `device` pointer is valid. In case
 * of failure, the function returns a negative error code and the device
 * pointer is not modified.
 *
 * @param device A pointer to a pointer where the initialized device structure
 * will be stored. Must not be null. The caller does not own the
 * memory and should not free it directly.
 * @param init_param A pointer to an `admfm2000_init_param` structure containing
 * initialization parameters such as mixer mode, gain, and
 * GPIO configurations. Must not be null and must be properly
 * initialized with valid parameters.
 * @return Returns 0 on success. On failure, returns a negative error code
 * indicating the type of error encountered, and the device pointer is
 * not modified.
 ******************************************************************************/
int admfm2000_init(struct admfm2000_dev **device,
		   struct admfm2000_init_param *init_param);

/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * admfm2000 device when it is no longer needed. This function should be
 * called to clean up after a device has been initialized and used,
 * ensuring that all associated GPIO resources are freed and the device
 * structure is deallocated. It is important to call this function to
 * prevent resource leaks in the system.
 *
 * @param dev A pointer to an admfm2000_dev structure representing the device to
 * be removed. Must not be null. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns 0 on successful removal of the device resources.
 ******************************************************************************/
int admfm2000_remove(struct admfm2000_dev *dev);

/***************************************************************************//**
 * @brief This function sets the raw digital step attenuator (DSA) gain for a
 * specified channel on the ADMFM2000 device. It should be used when
 * precise control over the DSA gain is required, and the raw gain value
 * is known. The function expects the raw gain value to be within the
 * defined range, and it will return an error if the value is outside
 * this range. It is important to ensure that the device has been
 * properly initialized before calling this function. The function
 * modifies the state of the device by setting the appropriate GPIO
 * values corresponding to the raw gain.
 *
 * @param dev A pointer to an initialized admfm2000_dev structure representing
 * the device. Must not be null.
 * @param chan The channel number for which the DSA gain is to be set. Valid
 * values are 0 or 1, corresponding to the available channels on the
 * device.
 * @param dsa_raw The raw DSA gain value to set. Must be within the range
 * defined by ADMFM2000_MIN_GAIN_RAW and ADMFM2000_MAX_GAIN_RAW.
 * If the value is outside this range, the function returns an
 * error.
 * @return Returns 0 on success. If the raw gain value is out of range, returns
 * -EINVAL. If setting the GPIO values fails, returns the corresponding
 * error code.
 ******************************************************************************/
int admfm2000_set_dsa_raw(struct admfm2000_dev *dev, uint8_t chan,
			  int32_t dsa_raw);

/***************************************************************************//**
 * @brief Use this function to obtain the raw digital step attenuator (DSA)
 * value for a specified channel on the ADMFM2000 device. This function
 * is typically called when you need to read the current DSA setting for
 * further processing or logging. It requires a valid device structure
 * and a channel number, which must be within the supported range. The
 * function will populate the provided pointer with the raw DSA value if
 * successful. Ensure that the device has been properly initialized
 * before calling this function.
 *
 * @param dev A pointer to an initialized admfm2000_dev structure. This must not
 * be null, and the device must be properly initialized before use.
 * @param chan The channel number for which to retrieve the DSA value. Valid
 * values are 0 or 1. If the value is greater than 1, the function
 * returns an error.
 * @param dsa_raw A pointer to an int32_t where the raw DSA value will be
 * stored. This must not be null, and the caller is responsible
 * for ensuring the pointer is valid.
 * @return Returns 0 on success, or a negative error code if the channel is
 * invalid or if there is an error reading the GPIO values.
 ******************************************************************************/
int admfm2000_get_dsa_raw(struct admfm2000_dev *dev, uint8_t chan,
			  int32_t *dsa_raw);

int admfm2000_get_channel_mode(struct admfm2000_dev *dev, uint8_t mode);

/***************************************************************************//**
 * @brief Use this function to set the channel configuration mode of an
 * ADMFM2000 device. It must be called with a valid device structure and
 * a configuration value of either 0 or 1. The function configures the
 * GPIOs associated with the device to switch between different channel
 * modes. It is important to ensure that the device has been properly
 * initialized before calling this function. An invalid configuration
 * value will result in an error.
 *
 * @param dev A pointer to an initialized 'admfm2000_dev' structure representing
 * the device. Must not be null.
 * @param config A uint8_t value representing the desired channel configuration.
 * Valid values are 0 and 1. Values outside this range will result
 * in an error.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid configuration values.
 ******************************************************************************/
int admfm2000_set_channel_config(struct admfm2000_dev *dev, uint8_t config);

#endif /* SRC_ADMFM2000_H_ */
