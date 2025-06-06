/***************************************************************************//**
 *   @file   ad9172.h
 *   @brief  Header file of ad9172 Driver.
 *   @author Cristian Pop (cristian.pop@analog.com)
********************************************************************************
 * Copyright 2018(c) Analog Devices, Inc.
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
#ifndef __AD9172_H__
#define __AD9172_H__

#include "AD917x.h"
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"

/***************************************************************************//**
 * @brief The `ad9172_dev` structure is a compound data type used to represent
 * the state and configuration of an AD9172 device. It includes pointers
 * to SPI and GPIO descriptors for managing communication and control
 * signals, as well as a pointer to an `ad9172_state` structure that
 * holds detailed configuration and operational parameters of the device.
 * This structure is essential for interfacing with the AD9172 DAC and
 * managing its various functionalities.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_reset Pointer to a GPIO descriptor for the reset pin.
 * @param gpio_txen0 Pointer to a GPIO descriptor for the TXEN0 pin.
 * @param gpio_txen1 Pointer to a GPIO descriptor for the TXEN1 pin.
 * @param st Pointer to an ad9172_state structure containing the device state.
 ******************************************************************************/
typedef struct ad9172_dev {
	/* SPI */
	struct no_os_spi_desc		*spi_desc;
	/* GPIO */
	struct no_os_gpio_desc	*gpio_reset;
	struct no_os_gpio_desc	*gpio_txen0;
	struct no_os_gpio_desc	*gpio_txen1;
	struct ad9172_state *st;
} ad9172_dev;

/***************************************************************************//**
 * @brief The `chip_id` enumeration defines a set of constants representing
 * unique identifiers for different AD917x series chips, each associated
 * with a specific hexadecimal value. This enumeration is used to
 * distinguish between various models of the AD917x series, facilitating
 * the identification and handling of specific chip types within the
 * driver code.
 *
 * @param CHIPID_AD9171 Represents the chip ID for AD9171 with a value of 0x71.
 * @param CHIPID_AD9172 Represents the chip ID for AD9172 with a value of 0x72.
 * @param CHIPID_AD9173 Represents the chip ID for AD9173 with a value of 0x73.
 * @param CHIPID_AD9174 Represents the chip ID for AD9174 with a value of 0x74.
 * @param CHIPID_AD9175 Represents the chip ID for AD9175 with a value of 0x75.
 * @param CHIPID_AD9176 Represents the chip ID for AD9176 with a value of 0x76.
 ******************************************************************************/
enum chip_id {
	CHIPID_AD9171 = 0x71,
	CHIPID_AD9172 = 0x72,
	CHIPID_AD9173 = 0x73,
	CHIPID_AD9174 = 0x74,
	CHIPID_AD9175 = 0x75,
	CHIPID_AD9176 = 0x76,
};

/***************************************************************************//**
 * @brief The `ad9172_state` structure is used to maintain the state and
 * configuration of the AD9172 DAC device. It includes various parameters
 * such as chip identification, DAC handle, JESD interface configuration,
 * clock settings, interpolation factors, and signal types. This
 * structure is essential for managing the operational settings and
 * ensuring the correct functioning of the DAC in different modes and
 * configurations.
 *
 * @param id Identifies the chip using an enumeration of possible chip IDs.
 * @param dac_h Handle to the DAC (Digital-to-Analog Converter) device.
 * @param appJesdConfig Configuration parameters for the JESD (Joint Electron
 * Device Engineering Council) interface.
 * @param dac_rate_khz Specifies the DAC rate in kilohertz.
 * @param dac_clkin_Hz Specifies the DAC clock input frequency in hertz.
 * @param dac_interpolation Defines the interpolation factor for the DAC.
 * @param channel_interpolation Defines the interpolation factor for the
 * channel.
 * @param interpolation Overall interpolation factor applied.
 * @param jesd_link_mode Specifies the JESD link mode configuration.
 * @param jesd_dual_link_mode Specifies the JESD dual link mode configuration.
 * @param jesd_subclass Defines the JESD subclass used.
 * @param clock_output_config Configuration for the clock output.
 * @param syncoutb_type Type of signal used for SYNCOUTB.
 * @param sysref_coupling Coupling type for the SYSREF signal.
 * @param nco_main_enable Flag to enable the main NCO (Numerically Controlled
 * Oscillator).
 * @param nco_channel_enable Flag to enable the channel NCO.
 ******************************************************************************/
struct ad9172_state {
	enum chip_id id;
	ad917x_handle_t dac_h;
	jesd_param_t appJesdConfig;
	uint32_t dac_rate_khz;
	uint64_t dac_clkin_Hz;
	uint32_t dac_interpolation;
	uint32_t channel_interpolation;
	uint32_t interpolation;
	uint32_t jesd_link_mode;
	uint32_t jesd_dual_link_mode;
	uint32_t jesd_subclass;
	uint32_t clock_output_config;
	signal_type_t syncoutb_type;
	signal_coupling_t sysref_coupling;
	uint8_t nco_main_enable;
	uint8_t nco_channel_enable;
};

/***************************************************************************//**
 * @brief The `ad9172_init_param` structure is used to initialize the AD9172
 * device, containing parameters for SPI and GPIO configurations, as well
 * as various settings for the DAC such as rate, clock input,
 * interpolation factors, and JESD link configurations. It also includes
 * signal type and coupling settings for synchronization and reference
 * signals.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_txen0 GPIO initialization parameter for TXEN0.
 * @param gpio_txen1 GPIO initialization parameter for TXEN1.
 * @param gpio_reset GPIO initialization parameter for reset.
 * @param dac_rate_khz DAC rate in kilohertz.
 * @param dac_clkin_Hz DAC clock input frequency in hertz.
 * @param jesd_link_mode JESD link mode configuration.
 * @param jesd_subclass JESD subclass configuration.
 * @param dac_interpolation DAC interpolation factor.
 * @param channel_interpolation Channel interpolation factor.
 * @param clock_output_config Configuration for clock output.
 * @param syncoutb_type Type of syncoutb signal.
 * @param sysref_coupling Coupling type for sysref signal.
 ******************************************************************************/
typedef struct ad9172_init_param {
	/* SPI */
	struct no_os_spi_init_param *spi_init;
	/* GPIO */
	struct no_os_gpio_init_param gpio_txen0;
	struct no_os_gpio_init_param gpio_txen1;
	struct no_os_gpio_init_param gpio_reset;
	uint32_t dac_rate_khz;
	uint32_t dac_clkin_Hz;
	uint32_t jesd_link_mode;
	uint32_t jesd_subclass;
	uint32_t dac_interpolation;
	uint32_t channel_interpolation;
	uint32_t clock_output_config;
	signal_type_t syncoutb_type;
	signal_coupling_t sysref_coupling;
} ad9172_init_param;

/***************************************************************************//**
 * @brief This function sets up and initializes the AD9172 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * allocates necessary resources and configures SPI and GPIO interfaces
 * as specified in the initialization parameters. If initialization fails
 * at any step, it cleans up allocated resources and returns an error
 * code. Successful initialization results in a configured device handle
 * that can be used for further operations.
 *
 * @param device A pointer to a pointer of type ad9172_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory
 * using ad9172_remove.
 * @param init_param A pointer to an ad9172_init_param structure containing
 * initialization parameters such as SPI and GPIO
 * configurations, DAC rate, and JESD settings. Must not be
 * null. The structure should be fully populated with valid
 * configuration data before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered, such as
 * memory allocation failure or interface configuration error.
 ******************************************************************************/
int32_t ad9172_init(ad9172_dev **device,
		    ad9172_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * AD9172 device when it is no longer needed. This includes deallocating
 * memory and removing associated SPI and GPIO descriptors. It should be
 * called to prevent resource leaks after the device is no longer in use.
 * Ensure that the device pointer is valid and initialized before calling
 * this function.
 *
 * @param device A pointer to an ad9172_dev structure representing the device to
 * be removed. Must not be null and should point to a valid,
 * initialized device structure. The function will handle invalid
 * pointers by returning an error code.
 * @return Returns an int32_t value indicating the success or failure of the
 * operation. A non-zero return value indicates an error occurred during
 * resource deallocation.
 ******************************************************************************/
int32_t ad9172_remove(ad9172_dev *device);
#endif // __AD9172_H__
