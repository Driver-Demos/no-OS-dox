/***************************************************************************//**
 *   @file   adrv9009.h
 *   @brief  ADRV9009 initialization and control routines.
 *   @author Darius Berghe (darius.berghe@analog.com)
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
#ifndef __ADRV9009_H__
#define __ADRV9009_H__

#include <stdbool.h>
#include <stdint.h>
#include "talise_cals_types.h"
#include "talise_agc_types.h"
#include "talise_types.h"
#include "talise_gpio.h"
#include "adi_hal.h"

/*
 * JESD204-FSM defines
 */

#define DEFRAMER_LINK_TX	1
#define FRAMER_LINK_RX		0
#define FRAMER_LINK_ORX		2

/***************************************************************************//**
 * @brief The `taliseDeviceId` enumeration defines identifiers for different
 * Talise devices, with `TALISE_A` as the default device ID and
 * `TALISE_B` conditionally included based on the compilation
 * environment. The enumeration also includes `TALISE_DEVICE_ID_MAX` to
 * represent the count of device IDs, which can be useful for iterating
 * over the available device IDs or for validation purposes.
 *
 * @param TALISE_A Represents the first device ID with a value of 0.
 * @param TALISE_B Represents the second device ID, conditionally included based
 * on specific preprocessor definitions.
 * @param TALISE_DEVICE_ID_MAX Indicates the maximum number of device IDs
 * available.
 ******************************************************************************/
enum taliseDeviceId {
	TALISE_A = 0u,
#if defined(ZU11EG) || defined(FMCOMMS8_ZCU102)
	TALISE_B,
#endif

	TALISE_DEVICE_ID_MAX
};

//#define MAX_NUM_GAIN_TABLES 10

/***************************************************************************//**
 * @brief The `adrv9009_clocks` enumeration defines a set of constants
 * representing different types of sampling clocks used in the ADRV9009
 * device, including receive, observation, and transmit sampling clocks.
 * This enumeration is used to manage and reference the various clock
 * types within the ADRV9009 RF transceiver system.
 *
 * @param RX_SAMPL_CLK Represents the receive sampling clock.
 * @param OBS_SAMPL_CLK Represents the observation sampling clock.
 * @param TX_SAMPL_CLK Represents the transmit sampling clock.
 * @param NUM_ADRV9009_CLKS Indicates the total number of clock types defined in
 * this enumeration.
 ******************************************************************************/
enum adrv9009_clocks {
	RX_SAMPL_CLK,
	OBS_SAMPL_CLK,
	TX_SAMPL_CLK,
	NUM_ADRV9009_CLKS,
};

/***************************************************************************//**
 * @brief The `adrv9009_gain_tables` enumeration defines various gain table
 * configurations for the ADRV9009 device, including options for
 * individual and combined receive channels, as well as observation
 * receiver channels. This enumeration is used to specify which gain
 * table configuration is being utilized in the device's operation.
 *
 * @param RX1_GT Represents the gain table for the first receive channel.
 * @param RX2_GT Represents the gain table for the second receive channel.
 * @param RX1_RX2_GT Represents the gain table for both the first and second
 * receive channels.
 * @param ORX_RX1_GT Represents the gain table for the observation receiver on
 * the first receive channel.
 * @param ORX_RX2_GT Represents the gain table for the observation receiver on
 * the second receive channel.
 * @param ORX_RX1_RX2_GT Represents the gain table for the observation receiver
 * on both the first and second receive channels.
 * @param NUM_GT Indicates the total number of gain tables defined.
 ******************************************************************************/
enum adrv9009_gain_tables {
	RX1_GT,
	RX2_GT,
	RX1_RX2_GT,
	ORX_RX1_GT,
	ORX_RX2_GT,
	ORX_RX1_RX2_GT,
	NUM_GT,
};

/***************************************************************************//**
 * @brief The `gain_table_info` structure is designed to encapsulate information
 * about a gain table used in RF systems, specifically for the ADRV9009
 * device. It includes fields for defining the range of the table
 * (`start` and `end`), the maximum index (`max_index`), and the
 * destination (`dest`). Additionally, it holds pointers to arrays or
 * structures that contain the actual gain values (`abs_gain_tbl`) and
 * specific gain table data for RX and ORX channels (`gainTablePtr` and
 * `orx_gainTablePtr`). This structure is crucial for managing and
 * applying gain settings in RF signal processing.
 *
 * @param start The starting frequency or index for the gain table.
 * @param end The ending frequency or index for the gain table.
 * @param max_index The maximum index value for the gain table.
 * @param dest The destination or target for the gain table.
 * @param abs_gain_tbl A pointer to an array of absolute gain values.
 * @param gainTablePtr A pointer to a taliseRxGainTable_t structure for RX gain
 * table data.
 * @param orx_gainTablePtr A pointer to a taliseOrxGainTable_t structure for ORX
 * gain table data.
 ******************************************************************************/
struct gain_table_info {
	uint64_t start;
	uint64_t end;
	uint8_t max_index;
	uint8_t dest;
	int32_t *abs_gain_tbl;
	taliseRxGainTable_t *gainTablePtr;
	taliseOrxGainTable_t *orx_gainTablePtr;
};

/***************************************************************************//**
 * @brief The `adrv9009_rf_phy` structure is a comprehensive data structure used
 * to manage and configure the ADRV9009 RF transceiver device. It
 * encapsulates various configuration parameters, control settings, and
 * hardware interface pointers necessary for initializing and operating
 * the device. This includes firmware and stream versioning, device-
 * specific configurations, gain control, power amplifier protection, and
 * filter coefficients. Additionally, it manages clock sources, GPIO
 * configurations, and provides interfaces for ADC and DAC operations.
 * The structure is designed to support complex RF operations and ensure
 * the device is properly initialized and controlled.
 *
 * @param fw A character array storing the firmware version, with a maximum
 * length of 32 characters.
 * @param stream A character array storing the stream version, with a maximum
 * length of 32 characters.
 * @param talise_device An instance of taliseDevice_t representing the Talise
 * device configuration.
 * @param talDevice A pointer to a taliseDevice_t structure for device-specific
 * operations.
 * @param talInit An instance of taliseInit_t containing initialization
 * parameters for the Talise device.
 * @param auxdac An instance of taliseAuxDac_t for auxiliary DAC configuration.
 * @param rxAgcCtrl An instance of taliseAgcCfg_t for configuring the receive
 * automatic gain control.
 * @param arm_gpio_config An instance of taliseArmGpioConfig_t for ARM GPIO
 * configuration.
 * @param rx1_gain_ctrl_pin An instance of taliseRxGainCtrlPin_t for RX1 gain
 * control pin configuration.
 * @param rx2_gain_ctrl_pin An instance of taliseRxGainCtrlPin_t for RX2 gain
 * control pin configuration.
 * @param tx1_atten_ctrl_pin An instance of taliseTxAttenCtrlPin_t for TX1
 * attenuation control pin configuration.
 * @param tx2_atten_ctrl_pin An instance of taliseTxAttenCtrlPin_t for TX2
 * attenuation control pin configuration.
 * @param tx_pa_protection An instance of taliseTxPaProtectCfg_t for TX power
 * amplifier protection configuration.
 * @param rx_hd2_config An instance of taliseRxHd2Config_t for RX HD2
 * configuration.
 * @param initCalMask A 32-bit unsigned integer representing the initial
 * calibration mask.
 * @param gpio3v3SrcCtrl A 16-bit unsigned integer for 3.3V GPIO source control.
 * @param gpio3v3PinLevel A 16-bit unsigned integer for 3.3V GPIO pin level
 * control.
 * @param gpio3v3OutEn A 16-bit unsigned integer for 3.3V GPIO output enable
 * control.
 * @param rxFirCoefs An array of 72 16-bit integers for RX FIR filter
 * coefficients.
 * @param obsrxFirCoefs An array of 72 16-bit integers for observation RX FIR
 * filter coefficients.
 * @param txFirCoefs An array of 80 16-bit integers for TX FIR filter
 * coefficients.
 * @param current_loopBandwidth_kHz An array of 2 16-bit integers for current
 * loop bandwidth in kHz.
 * @param loopFilter_stability An 8-bit unsigned integer for loop filter
 * stability.
 * @param trx_lo_frequency A 64-bit unsigned integer for the transceiver local
 * oscillator frequency.
 * @param aux_lo_frequency A 64-bit unsigned integer for the auxiliary local
 * oscillator frequency.
 * @param hal An instance of adi_hal for hardware abstraction layer operations.
 * @param dev_clk A pointer to a no_os_clk_desc structure for the device clock.
 * @param clk_ext_lo_rx A pointer to a no_os_clk_desc structure for the external
 * local oscillator clock for RX.
 * @param clk_ext_lo_tx A pointer to a no_os_clk_desc structure for the external
 * local oscillator clock for TX.
 * @param clks An array of pointers to no_os_clk_desc structures for various
 * clocks, indexed by adrv9009_clocks.
 * @param jdev A pointer to a jesd204_dev structure for JESD204 device
 * operations.
 * @param sysref_req_gpio A pointer to a no_os_gpio_desc structure for SYSREF
 * request GPIO.
 * @param device_id An 8-bit unsigned integer representing the device ID.
 * @param tracking_cal_mask A 32-bit unsigned integer for tracking calibration
 * mask.
 * @param is_initialized A boolean indicating whether the device is initialized.
 * @param spi_device_id An integer representing the SPI device ID.
 * @param framer_b_m A 32-bit unsigned integer for framer B M parameter.
 * @param framer_b_f A 32-bit unsigned integer for framer B F parameter.
 * @param orx_channel_enabled A 32-bit unsigned integer indicating if the
 * observation RX channel is enabled.
 * @param rx_adc A pointer to an axi_adc structure for RX ADC operations.
 * @param tx_dac A pointer to an axi_dac structure for TX DAC operations.
 * @param rx_os_adc A pointer to an axi_adc structure for RX observation ADC
 * operations.
 ******************************************************************************/
struct adrv9009_rf_phy {
	char				fw[32];
	char				stream[32];
	taliseDevice_t 			talise_device;
	taliseDevice_t 			*talDevice;
	taliseInit_t 			talInit;
	taliseAuxDac_t			auxdac;
	taliseAgcCfg_t 			rxAgcCtrl;
	taliseArmGpioConfig_t		arm_gpio_config;
	taliseRxGainCtrlPin_t		rx1_gain_ctrl_pin;
	taliseRxGainCtrlPin_t		rx2_gain_ctrl_pin;
	taliseTxAttenCtrlPin_t		tx1_atten_ctrl_pin;
	taliseTxAttenCtrlPin_t		tx2_atten_ctrl_pin;
	taliseTxPaProtectCfg_t		tx_pa_protection;
	taliseRxHd2Config_t		rx_hd2_config;
	uint32_t 			initCalMask;
	uint16_t			gpio3v3SrcCtrl;
	uint16_t 			gpio3v3PinLevel;
	uint16_t 			gpio3v3OutEn;
	int16_t 			rxFirCoefs[72];
	int16_t 			obsrxFirCoefs[72];
	int16_t 			txFirCoefs[80];
	int16_t 			current_loopBandwidth_kHz[2];
	uint8_t 			loopFilter_stability;
	uint64_t 			trx_lo_frequency;
	uint64_t 			aux_lo_frequency;
	struct adi_hal			hal;
	struct no_os_clk_desc 		*dev_clk;
	struct no_os_clk_desc 		*clk_ext_lo_rx;
	struct no_os_clk_desc 		*clk_ext_lo_tx;
	struct no_os_clk_desc 		*clks[NUM_ADRV9009_CLKS];
	struct jesd204_dev		*jdev;
	struct no_os_gpio_desc		*sysref_req_gpio;
	uint8_t 			device_id;
	uint32_t			tracking_cal_mask;
	bool				is_initialized;
	int				spi_device_id;
	uint32_t 			framer_b_m;
	uint32_t 			framer_b_f;
	uint32_t 			orx_channel_enabled;
	struct axi_adc			*rx_adc;
	struct axi_dac			*tx_dac;
	struct axi_adc			*rx_os_adc;
};

/***************************************************************************//**
 * @brief The `adrv9009_init_param` structure is used to encapsulate all the
 * necessary initialization parameters for setting up the ADRV9009
 * device. It includes pointers to SPI and clock initialization
 * parameters, device-specific configurations, and file paths for
 * firmware images. Additionally, it holds configuration settings for the
 * automatic gain control and transmit power amplifier protection, as
 * well as the frequency for the transceiver's local oscillator.
 *
 * @param spi_init Pointer to the SPI initialization parameters.
 * @param dev_clk Pointer to the device clock descriptor.
 * @param adrv9009_device Pointer to the ADRV9009 device structure.
 * @param streamImageFile Pointer to the stream image file name.
 * @param armImageFile Pointer to the ARM image file name.
 * @param rxAgcConfig_init_param Pointer to the initial AGC configuration
 * parameters.
 * @param trx_lo_frequency Transceiver local oscillator frequency in Hz.
 * @param tx_pa_protection Transmit power amplifier protection configuration.
 ******************************************************************************/
struct adrv9009_init_param {
	struct no_os_spi_init_param	*spi_init;
	struct no_os_clk_desc		*dev_clk;
	taliseDevice_t			*adrv9009_device;
	char				*streamImageFile;
	char				*armImageFile;
	taliseAgcCfg_t		  	*rxAgcConfig_init_param;
	uint64_t			trx_lo_frequency;
	taliseTxPaProtectCfg_t		tx_pa_protection;
};

#define TALISE_NUM_SUBCHANNELS		2 /* I - in-phase and Q - quadrature channels */
#define TALISE_NUM_CHAIN_CHANNELS	2 /* channels per RX/TX chain */
#define TALISE_NUM_CHANNELS		(TALISE_DEVICE_ID_MAX * TALISE_NUM_CHAIN_CHANNELS * TALISE_NUM_SUBCHANNELS)

bool adrv9009_check_sysref_rate(uint32_t lmfc, uint32_t sysref);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the ADRV9009 device using the provided
 * initialization parameters. It allocates memory for the device
 * structure, configures device settings, and sets up the JESD204 link.
 * It must be called before any other operations on the ADRV9009 device.
 * The function expects valid initialization parameters and will return
 * an error code if initialization fails, ensuring that resources are
 * properly freed in case of an error.
 *
 * @param dev A pointer to a pointer of type `struct adrv9009_rf_phy`. This will
 * be allocated and initialized by the function. The caller must
 * ensure this pointer is valid and will receive ownership of the
 * allocated memory.
 * @param init_param A pointer to a `struct adrv9009_init_param` containing the
 * initialization parameters for the device. This must not be
 * null and should be properly initialized with valid settings
 * before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code and ensures that any allocated resources are
 * freed.
 ******************************************************************************/
int32_t adrv9009_init(struct adrv9009_rf_phy **device,
		      const struct adrv9009_init_param *init_param);
/***************************************************************************//**
 * @brief This function sets up the ADRV9009 RF transceiver by initializing its
 * hardware, configuring SPI settings, and setting up necessary clock
 * components. It must be called after the device has been initialized
 * with `adrv9009_init`. The function handles various initialization
 * steps, including enabling the device clock, opening and resetting the
 * hardware, and configuring SPI settings. It also initializes sample
 * clocks for RX, ORX, and TX paths. If any step fails, the function
 * returns an error code, and the caller should handle these errors
 * appropriately. This function is essential for preparing the ADRV9009
 * for operation and must be executed before any data transmission or
 * reception.
 *
 * @param phy A pointer to an `adrv9009_rf_phy` structure representing the RF
 * transceiver. This structure must be properly initialized before
 * calling this function. The caller retains ownership and must
 * ensure it is not null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * which step of the setup process failed.
 ******************************************************************************/
int adrv9009_setup(struct adrv9009_rf_phy *phy);

/***************************************************************************//**
 * @brief Use this function to obtain a pointer to the SPI settings structure
 * associated with the ADRV9009 device. This function is typically called
 * after the device has been initialized to access or modify the SPI
 * configuration parameters. The returned pointer provides direct access
 * to the SPI settings, allowing for configuration adjustments as needed.
 * Ensure that the device is properly initialized before calling this
 * function to avoid accessing uninitialized data.
 *
 * @return Returns a pointer to a `taliseSpiSettings_t` structure containing the
 * SPI settings for the ADRV9009 device.
 ******************************************************************************/
/***************************************************************************//**
 * @brief The `adrv9009_spi_settings_get` is a function that returns a pointer
 * to a `taliseSpiSettings_t` structure. This structure likely contains
 * settings related to the SPI (Serial Peripheral Interface)
 * configuration for the ADRV9009 device, which is a wideband RF
 * transceiver.
 *
 * @details This function is used to retrieve the current SPI settings for the
 * ADRV9009 device, which are necessary for configuring and
 * communicating with the device over SPI.
 ******************************************************************************/
taliseSpiSettings_t *adrv9009_spi_settings_get(void);
/***************************************************************************//**
 * @brief Use this function to obtain a pointer to the initial configuration
 * settings of the ADRV9009 device. This is useful for accessing the
 * default initialization parameters that are set up for the device. The
 * function should be called when you need to review or utilize the
 * initial settings for further configuration or diagnostics. Ensure that
 * the ADRV9009 device is properly initialized before using these
 * settings.
 *
 * @return Returns a pointer to a `taliseInit_t` structure containing the
 * initial settings for the ADRV9009 device.
 ******************************************************************************/
/***************************************************************************//**
 * @brief The `adrv9009_initial_settings_get` is a function that returns a
 * pointer to a `taliseInit_t` structure. This structure likely contains
 * the initial configuration settings for the ADRV9009 device, which is a
 * wideband RF transceiver. The function is used to retrieve these
 * settings, possibly for initialization or configuration purposes.
 *
 * @details This function is used to obtain the initial configuration settings
 * for the ADRV9009 device.
 ******************************************************************************/
taliseInit_t *adrv9009_initial_settings_get(void);

/***************************************************************************//**
 * @brief This function is used to perform additional configuration on the
 * ADRV9009 RF PHY after the initial setup has been completed. It
 * configures the ADC and DAC settings, including channel control and
 * data format settings, and calculates the clock frequency for the DAC.
 * This function should be called after the initial setup to ensure the
 * device is properly configured for operation. It assumes that the `phy`
 * structure has been properly initialized and that the device is ready
 * for further configuration.
 *
 * @param phy A pointer to an `adrv9009_rf_phy` structure representing the RF
 * PHY device. This structure must be initialized and must not be
 * null. The function will modify the `tx_dac->clock_hz` field based
 * on the configuration.
 * @return Returns 0 on successful configuration. The `tx_dac->clock_hz` field
 * in the `phy` structure is updated with the calculated clock
 * frequency.
 ******************************************************************************/
int adrv9009_post_setup(struct adrv9009_rf_phy *phy);

/***************************************************************************//**
 * @brief The `has_tx` function determines if the ADRV9009 device has
 * transmission capabilities based on compile-time conditions.
 *
 * @param phy A pointer to a structure of type `adrv9009_rf_phy`, representing
 * the physical layer configuration of the ADRV9009 device.
 * @return A boolean value indicating whether the ADRV9009 device has
 * transmission capabilities (`true` if it does, `false` otherwise).
 ******************************************************************************/
static inline bool has_tx(struct adrv9009_rf_phy *phy)
{
#ifndef ADRV9008_1
	return 1;
#else
	return 0;
#endif
}

/***************************************************************************//**
 * @brief The `has_tx_and_en` function checks if the ADRV9009 device has a
 * transmit capability and if the transmit channels are enabled.
 *
 * @param phy A pointer to an `adrv9009_rf_phy` structure representing the
 * ADRV9009 device's physical layer configuration.
 * @return The function returns a boolean value indicating whether the device
 * has transmit capability and the transmit channels are enabled.
 ******************************************************************************/
static inline bool has_tx_and_en(struct adrv9009_rf_phy *phy)
{
	return has_tx(phy) && (phy->talInit.tx.txChannels != TAL_TXOFF);
}

/***************************************************************************//**
 * @brief The `has_obs` function checks if the observation (obs) path is
 * available by determining if the transmit (tx) path is present.
 *
 * @param phy A pointer to a `struct adrv9009_rf_phy` which contains the
 * configuration and state of the ADRV9009 device.
 * @return A boolean value indicating whether the observation path is available,
 * which is the same as whether the transmit path is available.
 ******************************************************************************/
static inline bool has_obs(struct adrv9009_rf_phy *phy)
{
	return has_tx(phy);
}

/***************************************************************************//**
 * @brief The `has_obs_and_en` function checks if the observation receiver (ORx)
 * is present and enabled in the ADRV9009 RF PHY structure.
 *
 * @param phy A pointer to an `adrv9009_rf_phy` structure representing the RF
 * PHY device configuration.
 * @return The function returns a boolean value indicating whether the
 * observation receiver is both present and enabled.
 ******************************************************************************/
static inline bool has_obs_and_en(struct adrv9009_rf_phy *phy)
{
	return has_obs(phy) &&
	       (phy->talInit.obsRx.obsRxChannelsEnable != TAL_ORXOFF);
}

/***************************************************************************//**
 * @brief The `has_rx` function checks if the receive (RX) functionality is
 * available for the ADRV9009 device, returning true unless the
 * ADRV9008_2 variant is defined.
 *
 * @param phy A pointer to a struct of type `adrv9009_rf_phy`, representing the
 * physical layer configuration of the ADRV9009 device.
 * @return A boolean value indicating the availability of RX functionality,
 * where `true` means available and `false` means not available.
 ******************************************************************************/
static inline bool has_rx(struct adrv9009_rf_phy *phy)
{
#ifndef ADRV9008_2
	return 1;
#else
	return 0;
#endif
}

/***************************************************************************//**
 * @brief The `has_rx_and_en` function checks if the receive (RX) path of the
 * ADRV9009 device is available and enabled.
 *
 * @param phy A pointer to an `adrv9009_rf_phy` structure representing the
 * physical layer configuration of the ADRV9009 device.
 * @return The function returns a boolean value: `true` if the RX path is both
 * available and enabled, otherwise `false`.
 ******************************************************************************/
static inline bool has_rx_and_en(struct adrv9009_rf_phy *phy)
{
	return has_rx(phy) && (phy->talInit.rx.rxChannels != TAL_RXOFF);
}

#endif /* __ADRV9009_H__ */
