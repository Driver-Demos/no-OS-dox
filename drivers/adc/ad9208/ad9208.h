/***************************************************************************//**
 *   @file   ad9208.h
 *   @brief  Header file of AD9208 Driver.
 *   @author Stefan Popa (stefan.popa@analog.com)
********************************************************************************
 * Copyright 2019(c) Analog Devices, Inc.
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
#ifndef __AD9208_H__
#define __AD9208_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "ad9208_api.h"
#include "ad9208_reg.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_delay.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD9208_FULL_BANDWIDTH_MODE 0
#define AD9208_1_DDC_MODE 1
#define AD9208_2_DDC_MODE 2
#define AD9208_4_DDC_MODE 4

#define AD9208_SYSREF_NONE 0	/* No SYSREF Support */
#define AD9208_SYSREF_ONESHOT 1	/* ONE-SHOT SYSREF */
#define AD9208_SYSREF_CONT 2	/* Continuous Sysref Synchronisation */
#define AD9208_SYSREF_MON 3	/* SYSREF monitor Mode */

#define AD9208_NCO_MODE_VIF 0	/* Variable IF Mode */
#define AD9208_NCO_MODE_ZIF 1	/* Zero IF Mode */
#define AD9208_NCO_MODE_TEST 3	/* Test Mode*/

#define AD9208_BUFF_CURR_400_UA  0x4	/* Buffer Current set to 400 uA */
#define AD9208_BUFF_CURR_500_UA  0x9	/* Buffer Current set to 500 uA */
#define AD9208_BUFF_CURR_600_UA  0x1E	/* Buffer Current set to 600 uA */
#define AD9208_BUFF_CURR_700_UA  0x23	/* Buffer Current set to 700 uA */
#define AD9208_BUFF_CURR_800_UA  0x28	/* Buffer Current set to 800 uA */
#define AD9208_BUFF_CURR_1000_UA  0x32	/* Buffer Current set to 1000 uA */

#define AD9208_CHIP_TYPE	0x03
#define AD9208_CHIP_ID		0xDF

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `ad9208_ddc` structure is used to configure the digital
 * downconverter (DDC) settings for the AD9208 device. It includes
 * parameters for decimation, NCO mode, carrier frequency, phase offset,
 * and gain control. This structure is essential for setting up the DDC
 * to process input signals according to specific requirements, such as
 * adjusting the frequency and gain of the signal.
 *
 * @param decimation Specifies the decimation factor for the digital
 * downconverter.
 * @param nco_mode Defines the mode of the Numerically Controlled Oscillator
 * (NCO).
 * @param carrier_freq_hz Represents the carrier frequency in hertz.
 * @param po Holds the phase offset value.
 * @param gain_db Indicates whether gain is applied in decibels.
 ******************************************************************************/
struct ad9208_ddc {
	uint32_t decimation;
	uint32_t nco_mode;
	uint64_t carrier_freq_hz;
	uint64_t po;
	bool gain_db;
};

/***************************************************************************//**
 * @brief The `ad9208_dev` structure is a compound data type used to represent
 * the AD9208 device in the software. It encapsulates the necessary
 * components for interfacing with the device, including SPI
 * communication and GPIO control for power management. The structure
 * also holds a pointer to the device's state, which contains
 * configuration and operational parameters. This structure is essential
 * for managing the device's initialization, configuration, and operation
 * within the software.
 *
 * @param spi_desc Pointer to a SPI descriptor for SPI communication.
 * @param gpio_powerdown Pointer to a GPIO descriptor for power down control.
 * @param st Pointer to the state structure of the AD9208 device.
 ******************************************************************************/
typedef struct ad9208_dev {
	/* SPI */
	struct no_os_spi_desc *spi_desc;
	/* GPIO */
	struct no_os_gpio_desc *gpio_powerdown;
	struct ad9208_state *st;
} ad9208_dev;

/***************************************************************************//**
 * @brief The `ad9208_state` structure is a comprehensive configuration and
 * state management structure for the AD9208 ADC device. It includes
 * settings for sampling frequency, input clock division, power
 * management, and various operational modes such as duty cycle
 * stabilization and analog input configuration. The structure also
 * manages digital down-converter (DDC) configurations, test modes, and
 * SYSREF synchronization settings. Additionally, it holds JESD interface
 * parameters, making it integral for initializing and controlling the
 * AD9208 ADC's operation in various applications.
 *
 * @param adc_h Pointer to the ADC handle.
 * @param sampling_frequency_hz The sampling frequency in hertz.
 * @param input_div Input clock divider ratio.
 * @param powerdown_pin_en Flag to enable the powerdown pin.
 * @param powerdown_mode Mode for powerdown operation.
 * @param duty_cycle_stabilizer_en Flag to enable duty cycle stabilizer.
 * @param current_scale Current scale setting.
 * @param analog_input_mode Flag to set analog input mode.
 * @param ext_vref_en Flag to enable external voltage reference.
 * @param buff_curr_n Buffer current setting for negative input.
 * @param buff_curr_p Buffer current setting for positive input.
 * @param fc_ch Frequency channel setting.
 * @param ddc Array of digital down-converter configurations.
 * @param ddc_cnt Count of digital down-converters.
 * @param ddc_output_format_real_en Flag to enable real output format for DDC.
 * @param ddc_input_format_real_en Flag to enable real input format for DDC.
 * @param test_mode_ch0 Test mode setting for channel 0.
 * @param test_mode_ch1 Test mode setting for channel 1.
 * @param sysref_lmfc_offset Offset for SYSREF LMFC.
 * @param sysref_edge_sel Flag to select SYSREF edge.
 * @param sysref_clk_edge_sel Flag to select SYSREF clock edge.
 * @param sysref_neg_window_skew Negative window skew for SYSREF.
 * @param sysref_pos_window_skew Positive window skew for SYSREF.
 * @param sysref_mode Mode for SYSREF operation.
 * @param sysref_count Count for SYSREF events.
 * @param jesd_param Pointer to JESD parameters.
 * @param jesd_subclass JESD subclass setting.
 ******************************************************************************/
struct ad9208_state {
	ad9208_handle_t *adc_h;
	uint64_t sampling_frequency_hz;
	uint32_t input_div; /* input clock divider ratio */
	bool powerdown_pin_en;
	uint32_t powerdown_mode;
	bool duty_cycle_stabilizer_en;
	uint8_t current_scale;
	bool analog_input_mode;
	bool ext_vref_en;
	uint32_t buff_curr_n;
	uint32_t buff_curr_p;
	uint8_t fc_ch;
	struct ad9208_ddc ddc[4];
	uint32_t ddc_cnt;
	bool ddc_output_format_real_en;
	bool ddc_input_format_real_en;
	uint32_t test_mode_ch0;
	uint32_t test_mode_ch1;

	uint32_t sysref_lmfc_offset;
	bool sysref_edge_sel;
	bool sysref_clk_edge_sel;
	uint32_t sysref_neg_window_skew;
	uint32_t sysref_pos_window_skew;
	uint32_t sysref_mode;
	uint32_t sysref_count;

	jesd_param_t *jesd_param;
	uint32_t jesd_subclass;
};

/***************************************************************************//**
 * @brief The `ad9208_init_param` structure is used to initialize the AD9208
 * device, encapsulating various configuration parameters such as SPI and
 * GPIO initialization, sampling frequency, input clock divider, power
 * down settings, duty cycle stabilizer, current scale, analog input
 * mode, external voltage reference, buffer currents, frequency channel,
 * digital downconverter settings, test modes, SYSREF configurations, and
 * JESD parameters. This structure provides a comprehensive set of
 * options to tailor the AD9208's operation to specific application
 * requirements.
 *
 * @param spi_init Pointer to SPI initialization parameters.
 * @param gpio_powerdown GPIO initialization parameters for power down control.
 * @param sampling_frequency_hz Sampling frequency in hertz.
 * @param input_div Input clock divider ratio.
 * @param powerdown_pin_en Flag to enable power down pin.
 * @param powerdown_mode Mode for power down operation.
 * @param duty_cycle_stabilizer_en Flag to enable duty cycle stabilizer.
 * @param current_scale Current scale setting.
 * @param analog_input_mode Flag to set analog input mode.
 * @param ext_vref_en Flag to enable external voltage reference.
 * @param buff_curr_n Buffer current setting for negative input.
 * @param buff_curr_p Buffer current setting for positive input.
 * @param fc_ch Frequency channel setting.
 * @param ddc Pointer to digital downconverter configuration.
 * @param ddc_cnt Count of digital downconverters.
 * @param ddc_output_format_real_en Flag to enable real output format for DDC.
 * @param ddc_input_format_real_en Flag to enable real input format for DDC.
 * @param test_mode_ch0 Test mode setting for channel 0.
 * @param test_mode_ch1 Test mode setting for channel 1.
 * @param sysref_lmfc_offset Offset for SYSREF LMFC.
 * @param sysref_edge_sel Flag to select SYSREF edge.
 * @param sysref_clk_edge_sel Flag to select SYSREF clock edge.
 * @param sysref_neg_window_skew Negative window skew for SYSREF.
 * @param sysref_pos_window_skew Positive window skew for SYSREF.
 * @param sysref_mode Mode for SYSREF operation.
 * @param sysref_count Count for SYSREF events.
 * @param jesd_param Pointer to JESD parameters.
 * @param jesd_subclass JESD subclass setting.
 ******************************************************************************/
typedef struct ad9208_init_param {
	/* SPI */
	struct no_os_spi_init_param *spi_init;
	/* GPIO */
	struct no_os_gpio_init_param gpio_powerdown;
	uint64_t sampling_frequency_hz;
	uint32_t input_div; /* input clock divider ratio */
	bool powerdown_pin_en;
	uint32_t powerdown_mode;
	bool duty_cycle_stabilizer_en;
	uint8_t current_scale;
	bool analog_input_mode;
	bool ext_vref_en;
	uint32_t buff_curr_n;
	uint32_t buff_curr_p;
	uint8_t fc_ch;
	struct ad9208_ddc *ddc;
	uint32_t ddc_cnt;
	bool ddc_output_format_real_en;
	bool ddc_input_format_real_en;
	uint32_t test_mode_ch0;
	uint32_t test_mode_ch1;

	uint32_t sysref_lmfc_offset;
	bool sysref_edge_sel;
	bool sysref_clk_edge_sel;
	uint32_t sysref_neg_window_skew;
	uint32_t sysref_pos_window_skew;
	uint32_t sysref_mode;
	uint32_t sysref_count;

	jesd_param_t *jesd_param;
	uint32_t jesd_subclass;
} ad9208_init_param;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Initialize the device. */
/***************************************************************************//**
 * @brief This function sets up and initializes the AD9208 device using the
 * provided initialization parameters. It must be called before any other
 * operations on the device to ensure proper configuration. The function
 * allocates necessary resources and configures the device's SPI and GPIO
 * interfaces, among other settings. If initialization fails, it returns
 * an error code and ensures that all allocated resources are freed.
 * Successful initialization results in a pointer to the device structure
 * being returned to the caller.
 *
 * @param device A pointer to a pointer of type ad9208_dev. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to an ad9208_init_param structure containing the
 * initialization parameters. Must not be null and should be
 * properly populated with valid configuration settings before
 * calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error encountered.
 ******************************************************************************/
int32_t ad9208_initialize(ad9208_dev **device, ad9208_init_param *init_param);
/* Remove the device. */
/***************************************************************************//**
 * @brief This function is used to safely deallocate and clean up resources
 * associated with an AD9208 device instance. It should be called when
 * the device is no longer needed to ensure that all allocated resources,
 * such as GPIO and SPI descriptors, are properly released. This function
 * must be called only after the device has been successfully initialized
 * and is no longer in use. Failure to call this function may result in
 * resource leaks.
 *
 * @param device A pointer to an ad9208_dev structure representing the device to
 * be removed. Must not be null. The function will handle null
 * internal pointers gracefully, but the device pointer itself
 * must be valid.
 * @return Returns an int32_t value indicating the success or failure of the
 * resource deallocation. A return value of 0 indicates success, while a
 * non-zero value indicates an error occurred during the removal
 * process.
 ******************************************************************************/
int32_t ad9208_remove(ad9208_dev *device);

#endif // __AD9208_H__
