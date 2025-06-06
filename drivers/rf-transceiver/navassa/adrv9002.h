/***************************************************************************//**
 *   @file   adrv9002.h
 *   @brief  adrv9002 driver header.
 *   @author Darius Berghe (darius.berghe@analog.com)
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
#ifndef IIO_TRX_ADRV9002_H_
#define IIO_TRX_ADRV9002_H_

#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "no_os_clk.h"

#include "adi_common_log.h"
#include "adi_adrv9001_user.h"
#include "adi_adrv9001_cals_types.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_radio_types.h"
#include "adi_adrv9001_rx_gaincontrol_types.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_ssi_types.h"

#include "no_os_platform.h"

#include "axi_adc_core.h"

#define ADRV_ADDRESS(port, chan)	((port) << 8 | (chan))
#define ADRV_ADDRESS_PORT(addr)		((addr) >> 8)
#define ADRV_ADDRESS_CHAN(addr)		((addr) & 0xFF)
#define ADRV9002_FH_HOP_SIGNALS_NR	2
#define ADRV9002_FH_TABLES_NR		2
#define ADRV9002_FH_BIN_ATTRS_CNT	(ADRV9002_FH_HOP_SIGNALS_NR * ADRV9002_FH_TABLES_NR)
#define ADRV9002_RX_MIN_GAIN_IDX       ADI_ADRV9001_RX_GAIN_INDEX_MIN
#define ADRV9002_RX_MAX_GAIN_IDX       ADI_ADRV9001_RX_GAIN_INDEX_MAX

enum {
	ADRV9002_CHANN_1,
	ADRV9002_CHANN_2,
	ADRV9002_CHANN_MAX,
};

/***************************************************************************//**
 * @brief The `adrv9002_fh_bin_table` structure is a simple data structure that
 * contains a single member, `bin_table`, which is an array capable of
 * holding up to 64 entries of 8-bit unsigned integers. This structure is
 * likely used to store a fixed-size table of binary values, possibly for
 * configuration or lookup purposes within the ADRV9002 driver context.
 *
 * @param bin_table An array of 64 unsigned 8-bit integers, representing the
 * binary table entries.
 ******************************************************************************/
struct adrv9002_fh_bin_table {
	/* max of 64 entries! */
	uint8_t bin_table[64];
};

#ifdef ADI_COMMON_VERBOSE
/*
 * Enable log if ADI_COMMON_VERBOSE is defined
 */
#define	adrv9002_log_enable(common)	\
	(common)->error.logEnable = true
#else
#define	adrv9002_log_enable(common)
#endif

/***************************************************************************//**
 * @brief The `ad900x_device_id` enumeration defines a set of constants
 * representing different device IDs for the ADRV9002 and ADRV9003
 * models, including configurations with RX2TX2. This enumeration is used
 * to identify and differentiate between various models and
 * configurations of the ADRV900x series devices within the driver code.
 *
 * @param ID_ADRV9002 Represents the device ID for the ADRV9002 model.
 * @param ID_ADRV9002_RX2TX2 Represents the device ID for the ADRV9002 model
 * with RX2TX2 configuration.
 * @param ID_ADRV9003 Represents the device ID for the ADRV9003 model.
 * @param ID_ADRV9003_RX2TX2 Represents the device ID for the ADRV9003 model
 * with RX2TX2 configuration.
 ******************************************************************************/
enum ad900x_device_id {
	ID_ADRV9002,
	ID_ADRV9002_RX2TX2,
	ID_ADRV9003,
	ID_ADRV9003_RX2TX2,
};

/***************************************************************************//**
 * @brief The `adrv9002_clocks` enumeration defines a set of constants
 * representing different clock sources used in the ADRV9002 device,
 * including sampling clocks for receive and transmit channels, as well
 * as interface clocks for TDD channels. This enumeration is used to
 * identify and manage the various clock sources within the ADRV9002
 * driver.
 *
 * @param RX1_SAMPL_CLK Represents the sampling clock for the first receive
 * channel.
 * @param RX2_SAMPL_CLK Represents the sampling clock for the second receive
 * channel.
 * @param TX1_SAMPL_CLK Represents the sampling clock for the first transmit
 * channel.
 * @param TX2_SAMPL_CLK Represents the sampling clock for the second transmit
 * channel.
 * @param TDD1_INTF_CLK Represents the interface clock for the first TDD (Time
 * Division Duplex) channel.
 * @param TDD2_INTF_CLK Represents the interface clock for the second TDD (Time
 * Division Duplex) channel.
 * @param NUM_ADRV9002_CLKS Indicates the total number of clock types defined in
 * this enumeration.
 ******************************************************************************/
enum adrv9002_clocks {
	RX1_SAMPL_CLK,
	RX2_SAMPL_CLK,
	TX1_SAMPL_CLK,
	TX2_SAMPL_CLK,
	TDD1_INTF_CLK,
	TDD2_INTF_CLK,
	NUM_ADRV9002_CLKS,
};

/***************************************************************************//**
 * @brief The `adrv9002_rx_ext_info` is an enumeration that defines various
 * extended information parameters related to the RX (receive) path of
 * the ADRV9002 device. These parameters include various correction and
 * calibration settings, such as Quadrature Error Correction, Automatic
 * Gain Control, and Received Signal Strength Indicator, among others.
 * This enumeration is used to identify and manage different aspects of
 * the RX path's configuration and performance in the ADRV9002 driver.
 *
 * @param RX_QEC_FIC Represents the RX Quadrature Error Correction for Frequency
 * Independent Calibration.
 * @param RX_QEC_W_POLY Represents the RX Quadrature Error Correction with
 * Polynomial.
 * @param RX_AGC Represents the RX Automatic Gain Control.
 * @param RX_TRACK_BBDC Represents the RX Tracking Baseband DC Correction.
 * @param RX_HD2 Represents the RX Second Harmonic Distortion.
 * @param RX_RSSI_CAL Represents the RX Received Signal Strength Indicator
 * Calibration.
 * @param RX_RFDC Represents the RX RF DC Correction.
 * @param RX_RSSI Represents the RX Received Signal Strength Indicator.
 * @param RX_DECIMATION_POWER Represents the RX Decimation Power.
 * @param RX_RF_BANDWIDTH Represents the RX RF Bandwidth.
 * @param RX_POWERDOWN Represents the RX Power Down state.
 * @param RX_GAIN_CTRL_PIN_MODE Represents the RX Gain Control Pin Mode.
 * @param RX_ENSM_MODE Represents the RX Enable State Machine Mode.
 * @param RX_NCO_FREQUENCY Represents the RX Numerically Controlled Oscillator
 * Frequency.
 * @param RX_ADC_SWITCH Represents the RX ADC Switch.
 * @param RX_BBDC Represents the RX Baseband DC Correction.
 ******************************************************************************/
enum adrv9002_rx_ext_info {
	RX_QEC_FIC,
	RX_QEC_W_POLY,
	RX_AGC,
	RX_TRACK_BBDC,
	RX_HD2,
	RX_RSSI_CAL,
	RX_RFDC,
	RX_RSSI,
	RX_DECIMATION_POWER,
	RX_RF_BANDWIDTH,
	RX_POWERDOWN,
	RX_GAIN_CTRL_PIN_MODE,
	RX_ENSM_MODE,
	RX_NCO_FREQUENCY,
	RX_ADC_SWITCH,
	RX_BBDC,
};

/***************************************************************************//**
 * @brief The `adrv9002_tx_ext_info` is an enumeration that defines various
 * external information parameters related to the transmission (TX) path
 * of the ADRV9002 device. Each enumerator represents a specific aspect
 * or feature of the TX path, such as error correction, power control,
 * and frequency settings, which are crucial for configuring and managing
 * the transmission characteristics of the device.
 *
 * @param TX_QEC Represents the TX Quadrature Error Correction.
 * @param TX_LOL Represents the TX Local Oscillator Leakage.
 * @param TX_LB_PD Represents the TX Loopback Power Detector.
 * @param TX_PAC Represents the TX Power Amplifier Correction.
 * @param TX_CLGC Represents the TX Closed Loop Gain Control.
 * @param TX_RF_BANDWIDTH Represents the TX RF Bandwidth.
 * @param TX_POWERDOWN Represents the TX Power Down state.
 * @param TX_ATTN_CTRL_PIN_MODE Represents the TX Attenuation Control Pin Mode.
 * @param TX_ENSM_MODE Represents the TX Enable State Machine Mode.
 * @param TX_NCO_FREQUENCY Represents the TX Numerically Controlled Oscillator
 * Frequency.
 ******************************************************************************/
enum adrv9002_tx_ext_info {
	TX_QEC,
	TX_LOL,
	TX_LB_PD,
	TX_PAC,
	TX_CLGC,
	TX_RF_BANDWIDTH,
	TX_POWERDOWN,
	TX_ATTN_CTRL_PIN_MODE,
	TX_ENSM_MODE,
	TX_NCO_FREQUENCY
};

#define api_call(phy, func, args...)	({				\
	int __ret = func((phy)->adrv9001, ##args);			\
									\
	if (__ret)							\
		__ret = adrv9002_dev_err(phy);	\
									\
	__ret;								\
})

/***************************************************************************//**
 * @brief The `adrv9002_clock` structure is used to represent a clock
 * configuration in the ADRV9002 driver. It contains pointers to the SPI
 * device and RF physical layer structures, which are essential for
 * interfacing with the hardware. The structure also includes a field for
 * the clock rate and an enumeration to specify the clock source,
 * allowing for flexible clock management within the ADRV9002 system.
 *
 * @param spi A pointer to a spi_device structure, representing the SPI device
 * associated with the clock.
 * @param phy A pointer to an adrv9002_rf_phy structure, representing the RF
 * physical layer associated with the clock.
 * @param rate An unsigned long integer representing the clock rate.
 * @param source An enumeration of type adrv9002_clocks, indicating the source
 * of the clock.
 ******************************************************************************/
struct adrv9002_clock {
	struct spi_device	*spi;
	struct adrv9002_rf_phy	*phy;
	unsigned long		rate;
	enum adrv9002_clocks	source;
};

/***************************************************************************//**
 * @brief The `adrv9002_chan` structure is a compound data type used to
 * represent a channel in the ADRV9002 radio transceiver. It encapsulates
 * various attributes of a channel, including its clock, external local
 * oscillator, enablement delays, rate, state, number, port, power, NCO
 * frequency, index, and enabled status. This structure is integral to
 * managing and configuring the channel's operational parameters within
 * the ADRV9002 driver.
 *
 * @param clk Pointer to a clock structure associated with the channel.
 * @param ext_lo Pointer to an external local oscillator structure.
 * @param en_delays_ns Structure holding channel enablement delays in
 * nanoseconds.
 * @param rate Unsigned long representing the rate of the channel.
 * @param cached_state Enumeration representing the cached state of the channel.
 * @param number Enumeration representing the channel number.
 * @param port Enumeration representing the port associated with the channel.
 * @param power Unsigned 32-bit integer representing the power level of the
 * channel.
 * @param nco_freq Integer representing the NCO frequency of the channel.
 * @param idx Unsigned 8-bit integer representing the index of the channel.
 * @param enabled Unsigned 8-bit integer indicating if the channel is enabled.
 ******************************************************************************/
struct adrv9002_chan {
	struct no_os_clk *clk;
	struct adrv9002_ext_lo *ext_lo;
	/*
	 * These values are in nanoseconds. They need to be converted with
	 * @adrv9002_chan_ns_to_en_delay() before passing them to the API.
	 */
	struct adi_adrv9001_ChannelEnablementDelays en_delays_ns;
	unsigned long rate;
	adi_adrv9001_ChannelState_e cached_state;
	adi_common_ChannelNumber_e number;
	adi_common_Port_e port;
	uint32_t power;
	int nco_freq;
	uint8_t idx;
	uint8_t enabled;;
};

/***************************************************************************//**
 * @brief The `adrv9002_rx_chan` structure is designed to encapsulate the
 * configuration and state of a receive channel in the ADRV9002 radio
 * transceiver. It includes essential components such as the channel
 * configuration, automatic gain control settings, and optional test
 * configurations for debugging purposes. This structure is crucial for
 * managing the receive path in the ADRV9002, allowing for detailed
 * control over gain settings and channel enablement, as well as
 * interfacing with GPIOs for observation purposes.
 *
 * @param channel Represents the channel configuration for the ADRV9002 device.
 * @param agc Holds the automatic gain control configuration for the RX channel.
 * @param pin_cfg Pointer to the RX gain control pin configuration structure.
 * @param orx_gpio Descriptor for the observation receiver GPIO.
 * @param orx_en Indicates whether the observation receiver is enabled.
 * @param ssi_test Contains the SSI test mode configuration, included only if
 * CONFIG_DEBUG_FS is defined.
 ******************************************************************************/
struct adrv9002_rx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_GainControlCfg agc;
	struct adi_adrv9001_RxGainControlPinCfg *pin_cfg;
	struct no_os_gpio_desc *orx_gpio;
	uint8_t orx_en;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_RxSsiTestModeCfg ssi_test;
#endif
};

/***************************************************************************//**
 * @brief The `adrv9002_tx_chan` structure is designed to encapsulate the
 * configuration and state of a transmission channel in the ADRV9002 RF
 * transceiver. It includes a general channel configuration, a pointer to
 * a pin control configuration for managing Tx attenuation, and a flag
 * for enabling DAC boost. Additionally, when compiled with debug file
 * system support, it includes configurations for SSI test mode and a
 * loopback flag. This structure is crucial for managing the transmission
 * characteristics and debugging capabilities of the ADRV9002 device.
 *
 * @param channel This is a structure representing a general channel
 * configuration for the ADRV9002 device.
 * @param pin_cfg This is a pointer to a structure that configures the pin
 * control for Tx attenuation.
 * @param dac_boost_en This is a uint8_t flag indicating whether DAC boost is
 * enabled.
 * @param ssi_test This is a structure for configuring the SSI test mode,
 * included only if CONFIG_DEBUG_FS is defined.
 * @param loopback This is a uint8_t flag indicating whether loopback is
 * enabled, included only if CONFIG_DEBUG_FS is defined.
 ******************************************************************************/
struct adrv9002_tx_chan {
	struct adrv9002_chan channel;
	struct adi_adrv9001_TxAttenuationPinControlCfg *pin_cfg;
	uint8_t dac_boost_en;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_TxSsiTestModeCfg ssi_test;
	uint8_t loopback;
#endif
};

/***************************************************************************//**
 * @brief The `adrv9002_gpio` structure is designed to encapsulate the
 * configuration and signal information for a GPIO (General Purpose
 * Input/Output) in the context of the ADRV9002 driver. It includes a
 * configuration structure (`adi_adrv9001_GpioCfg`) that defines the
 * settings for the GPIO, and a signal identifier (`signal`) that
 * specifies the particular signal associated with this GPIO
 * configuration. This structure is part of the broader ADRV9002 driver
 * framework, which is used for managing and interfacing with the
 * ADRV9002 RF transceiver.
 *
 * @param gpio This member is a structure of type adi_adrv9001_GpioCfg, which
 * holds the configuration settings for a GPIO.
 * @param signal This member is a 32-bit unsigned integer representing a
 * specific signal associated with the GPIO.
 ******************************************************************************/
struct adrv9002_gpio {
	struct adi_adrv9001_GpioCfg gpio;
	uint32_t signal;
};

#define to_clk_priv(_hw) container_of(_hw, struct adrv9002_clock, hw)

/***************************************************************************//**
 * @brief The `struct mutex` is a simple data structure containing a single
 * member, `unused`, which is a 32-bit unsigned integer. This structure
 * appears to be a placeholder or a stub for a mutex implementation, but
 * as it stands, it does not provide any functionality related to mutual
 * exclusion or synchronization. It may be intended for future expansion
 * or to maintain compatibility with other parts of the code that expect
 * a mutex structure.
 *
 * @param unused A 32-bit unsigned integer that is currently not used.
 ******************************************************************************/
struct mutex {
	uint32_t unused;
};

/***************************************************************************//**
 * @brief The `adrv9002_chip_info` structure is used to encapsulate information
 * about a specific ADRV9002 chip, including its CMOS and LVD profiles,
 * name, number of transmit channels, and whether it supports RX2TX2
 * mode. This structure is likely used within the driver to manage and
 * configure the chip's operational parameters.
 *
 * @param cmos_profile A pointer to a string representing the CMOS profile of
 * the chip.
 * @param lvd_profile A pointer to a string representing the LVD profile of the
 * chip.
 * @param name A pointer to a string representing the name of the chip.
 * @param n_tx An unsigned 32-bit integer representing the number of transmit
 * channels.
 * @param rx2tx2 A boolean indicating if the chip supports RX2TX2 mode.
 ******************************************************************************/
struct adrv9002_chip_info {
	const char *cmos_profile;
	const char *lvd_profile;
	const char *name;
	uint32_t n_tx;
	bool rx2tx2;
};

/***************************************************************************//**
 * @brief The `adrv9002_ext_lo` structure is used to define an external local
 * oscillator configuration for the ADRV9002 device. It contains a
 * pointer to a clock structure and a divider value, which are used to
 * manage and configure the external local oscillator's frequency and
 * operation.
 *
 * @param clk A pointer to a no_os_clk structure, representing the clock
 * associated with the external local oscillator.
 * @param divider An unsigned short integer representing the divider value for
 * the external local oscillator.
 ******************************************************************************/
struct adrv9002_ext_lo {
	struct no_os_clk *clk;
	unsigned short divider;
};

/***************************************************************************//**
 * @brief The `adrv9002_rf_phy` structure is a comprehensive data structure used
 * to manage and configure the ADRV9002 RF transceiver device. It
 * encapsulates various components such as SPI device information, GPIO
 * configurations, clock management, and channel configurations for both
 * transmission and reception. The structure also includes fields for
 * handling frequency hopping, initialization profiles, and hardware
 * abstraction layer settings. It is designed to facilitate the
 * integration and control of the ADRV9002 device within a larger system,
 * providing a centralized point for managing its various functionalities
 * and ensuring synchronized access through mutex locking.
 *
 * @param chip Pointer to the chip information structure.
 * @param spi Pointer to the SPI device structure.
 * @param indio_dev Pointer to the IIO device structure.
 * @param reset_gpio Pointer to the GPIO descriptor for reset.
 * @param ssi_sync Pointer to the GPIO descriptor for SSI synchronization.
 * @param lock Mutex for synchronizing access to the structure.
 * @param clk_priv Array of clock structures for managing device clocks.
 * @param ext_los Array of external local oscillator structures.
 * @param profile_buf Buffer for storing the profile data.
 * @param profile_len Length of the profile data in the buffer.
 * @param stream_buf Pointer to the buffer for streaming data.
 * @param stream_size Size of the streaming data buffer.
 * @param fh_table_bin_attr Array of frequency hopping binary table attributes.
 * @param fh Frequency hopping configuration structure.
 * @param rx_channels Array of receive channel structures.
 * @param tx_channels Array of transmit channel structures.
 * @param channels Array of pointers to channel structures.
 * @param adrv9002_gpios Pointer to the ADRV9002 GPIO structure.
 * @param adrv9001_device ADI ADRV9001 device structure.
 * @param adrv9001 Pointer to the ADI ADRV9001 device structure.
 * @param hal Hardware abstraction layer configuration structure.
 * @param curr_profile Pointer to the current initialization profile.
 * @param profile Initialization profile structure.
 * @param init_cals Initialization calibrations structure.
 * @param n_clks Number of clocks available.
 * @param spi_device_id Identifier for the SPI device.
 * @param ngpios Number of GPIOs available.
 * @param rx2tx2 Flag indicating RX2TX2 mode.
 * @param ssi_type Type of SSI interface used.
 * @param tx_only Flag indicating if TX-only profiles are valid.
 * @param ssi_delays SSI calibration configuration structure (debug only).
 * @param rx1_adc Pointer to the ADC structure for RX1.
 * @param tx1_dac Pointer to the DAC structure for TX1.
 * @param rx2_adc Pointer to the ADC structure for RX2.
 * @param tx2_dac Pointer to the DAC structure for TX2.
 * @param rx1_dmac Pointer to the DMAC structure for RX1.
 * @param tx1_dmac Pointer to the DMAC structure for TX1.
 * @param rx2_dmac Pointer to the DMAC structure for RX2.
 * @param tx2_dmac Pointer to the DMAC structure for TX2.
 ******************************************************************************/
struct adrv9002_rf_phy {
	const struct adrv9002_chip_info *chip;
	struct spi_device		*spi;
	struct iio_dev			*indio_dev;
	struct no_os_gpio_desc		*reset_gpio;
	struct no_os_gpio_desc		*ssi_sync;
	struct mutex			lock;
	struct adrv9002_clock		clk_priv[NUM_ADRV9002_CLKS];
	/* each LO controls two ports (at least) */
	struct adrv9002_ext_lo		ext_los[ADRV9002_CHANN_MAX];
	char                        	profile_buf[350];
	size_t                      	profile_len;
	uint8_t				*stream_buf;
	uint16_t			stream_size;
	struct adrv9002_fh_bin_table	fh_table_bin_attr[ADRV9002_FH_BIN_ATTRS_CNT];
	adi_adrv9001_FhCfg_t		fh;
	struct adrv9002_rx_chan		rx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_tx_chan		tx_channels[ADRV9002_CHANN_MAX];
	struct adrv9002_chan		*channels[ADRV9002_CHANN_MAX * 2];
	struct adrv9002_gpio 		*adrv9002_gpios;
	struct adi_adrv9001_Device	adrv9001_device;
	struct adi_adrv9001_Device	*adrv9001;
	struct adrv9002_hal_cfg		hal;
	struct adi_adrv9001_Init	*curr_profile;
	struct adi_adrv9001_Init	profile;
	struct adi_adrv9001_InitCals	init_cals;
	uint32_t			n_clks;
	int				spi_device_id;
	int				ngpios;
	uint8_t				rx2tx2;
	/* ssi type of the axi cores - cannot really change at runtime */
	enum adi_adrv9001_SsiType	ssi_type;
	/*
	 * Tells if TX only profiles are valid. If not set, it means that TX1/TX2 SSI clocks are
	 * derived from RX1/RX2 which means that TX cannot be enabled if RX is not...
	 */
	uint8_t				tx_only;
#ifdef CONFIG_DEBUG_FS
	struct adi_adrv9001_SsiCalibrationCfg ssi_delays;
#endif
	struct axi_adc			*rx1_adc;
	struct axi_dac			*tx1_dac;
	struct axi_adc			*rx2_adc;
	struct axi_dac			*tx2_dac;
	struct axi_dmac			*rx1_dmac;
	struct axi_dmac			*tx1_dmac;
	struct axi_dmac			*rx2_dmac;
	struct axi_dmac			*tx2_dmac;
};

/***************************************************************************//**
 * @brief This function should be called after the initial setup of the ADRV9002
 * device to configure and enable its channels and interfaces. It adjusts
 * the device settings based on the operational mode (e.g., single or
 * dual channel mode) and ensures that all necessary components are
 * properly configured and enabled. The function handles the
 * configuration of the ADC and DAC interfaces, re-enables the cores, and
 * performs interface tuning. It is essential to call this function to
 * ensure the device is fully operational and ready for data transmission
 * and reception.
 *
 * @param phy A pointer to an adrv9002_rf_phy structure representing the device
 * instance. Must not be null. The caller retains ownership and is
 * responsible for ensuring the structure is properly initialized
 * before calling this function.
 * @return Returns 0 on success, or a negative error code if the configuration
 * or enabling process fails.
 ******************************************************************************/
int adrv9002_post_setup(struct adrv9002_rf_phy *phy);
/***************************************************************************//**
 * @brief This function sets up the ADRV9002 RF transceiver by configuring its
 * channels, disabling cores to prevent interference during
 * initialization, and performing necessary calibrations. It should be
 * called to initialize the device before any RF operations are
 * performed. The function configures the device based on the current
 * profile and handles both TDD and FDD modes. It also manages the
 * enabling of interrupts and the configuration of the digital and radio
 * paths. The function must be called with a valid `adrv9002_rf_phy`
 * structure that has been properly initialized with the desired
 * configuration settings.
 *
 * @param phy A pointer to an `adrv9002_rf_phy` structure representing the RF
 * transceiver device. This structure must be initialized with the
 * desired configuration settings before calling this function. The
 * pointer must not be null, and the function will return an error if
 * the initialization fails.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure, indicating the type of error encountered during
 * setup.
 ******************************************************************************/
int adrv9002_setup(struct adrv9002_rf_phy *phy);
/***************************************************************************//**
 * @brief This function is used to determine the current Serial Synchronous
 * Interface (SSI) type configuration of the ADRV9002 device, which can
 * be either CMOS or LVDS. It is typically called to verify or configure
 * the interface type for data communication. The function requires a
 * valid `adrv9002_rf_phy` structure, which represents the physical layer
 * configuration of the ADRV9002 device. The function does not modify the
 * input structure or any other state.
 *
 * @param phy A pointer to a `struct adrv9002_rf_phy` that represents the
 * physical layer configuration of the ADRV9002 device. This
 * parameter must not be null, and the structure should be properly
 * initialized before calling this function.
 * @return Returns an `adi_adrv9001_SsiType_e` enumeration value indicating the
 * SSI type, either `ADI_ADRV9001_SSI_TYPE_CMOS` or
 * `ADI_ADRV9001_SSI_TYPE_LVDS`.
 ******************************************************************************/
adi_adrv9001_SsiType_e adrv9002_axi_ssi_type_get(struct adrv9002_rf_phy *phy);

/* Main driver API's */
/***************************************************************************//**
 * @brief This function is used to convert channel enablement delays specified
 * in nanoseconds to equivalent delays in ARM clock cycles. It is
 * typically called when configuring the ADRV9002 device to ensure that
 * the timing parameters are correctly set according to the device's
 * clock rate. The function requires a valid `adrv9002_rf_phy` structure
 * to determine the ARM clock frequency. The input delays in nanoseconds
 * are provided through the `d_ns` parameter, and the converted delays
 * are output through the `d` parameter. This function does not handle
 * null pointers, so all input pointers must be valid.
 *
 * @param phy A pointer to a `adrv9002_rf_phy` structure that contains the
 * device configuration. Must not be null.
 * @param d_ns A pointer to a `adi_adrv9001_ChannelEnablementDelays` structure
 * containing the delays in nanoseconds. Must not be null.
 * @param d A pointer to a `adi_adrv9001_ChannelEnablementDelays` structure
 * where the converted delays in ARM clock cycles will be stored. Must
 * not be null.
 * @return None
 ******************************************************************************/
void adrv9002_en_delays_ns_to_arm(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d_ns,
				  struct adi_adrv9001_ChannelEnablementDelays *d);
/***************************************************************************//**
 * @brief This function is used to convert channel enablement delays, which are
 * originally specified in terms of ARM clock cycles, into nanoseconds.
 * It is useful when precise timing in nanoseconds is required for
 * configuring or analyzing the channel enablement delays. The function
 * requires a valid `adrv9002_rf_phy` structure to determine the ARM
 * clock frequency. The input delays are provided in a
 * `adi_adrv9001_ChannelEnablementDelays` structure, and the converted
 * delays are output to another `adi_adrv9001_ChannelEnablementDelays`
 * structure. This function should be called when the ARM clock frequency
 * is known and the conversion to nanoseconds is necessary for further
 * processing or configuration.
 *
 * @param phy A pointer to a `adrv9002_rf_phy` structure that contains the ARM
 * clock information. Must not be null.
 * @param d A pointer to a `adi_adrv9001_ChannelEnablementDelays` structure
 * containing the delays in ARM clock cycles. Must not be null.
 * @param d_ns A pointer to a `adi_adrv9001_ChannelEnablementDelays` structure
 * where the converted delays in nanoseconds will be stored. Must
 * not be null.
 * @return None
 ******************************************************************************/
void adrv9002_en_delays_arm_to_ns(const struct adrv9002_rf_phy *phy,
				  const struct adi_adrv9001_ChannelEnablementDelays *d,
				  struct adi_adrv9001_ChannelEnablementDelays *d_ns);
/* phy lock must be held before entering the API */
/***************************************************************************//**
 * @brief This function is used to transition a specified channel of the
 * ADRV9002 device to a desired operational state. It should be called
 * when a channel needs to be moved to a different state, such as from
 * disabled to enabled, or to any other valid state. The function handles
 * the necessary mode changes and ensures the channel reaches the desired
 * state, retrying if necessary. It can also cache the current state of
 * the channel if requested. The function must be called with a valid
 * channel that is enabled, and it assumes the device is properly
 * initialized and configured.
 *
 * @param phy A pointer to an initialized adrv9002_rf_phy structure representing
 * the device. Must not be null.
 * @param chann A pointer to an adrv9002_chan structure representing the channel
 * to be transitioned. The channel must be enabled, and the pointer
 * must not be null.
 * @param state The desired state to transition the channel to, specified as an
 * adi_adrv9001_ChannelState_e enumeration value. Must be a valid
 * state for the channel.
 * @param cache_state A boolean flag indicating whether to cache the current
 * state of the channel before transitioning. If true, the
 * current state is cached.
 * @return Returns 0 on success, or a negative error code on failure.
 ******************************************************************************/
int adrv9002_channel_to_state(const struct adrv9002_rf_phy *phy,
			      struct adrv9002_chan *chann,
			      const adi_adrv9001_ChannelState_e state, const bool cache_state);
int adrv9002_init(struct adrv9002_rf_phy *phy,
		  struct adi_adrv9001_Init *profile);
/***************************************************************************//**
 * @brief Use this function to report an error encountered by the ADRV9002 RF
 * PHY device. It logs the error message and code, and then clears the
 * error state. This function is typically called when an API function
 * interacting with the ADRV9002 device returns an error. It helps in
 * diagnosing issues by providing detailed error information, including
 * the function name and line number where the error occurred. Ensure
 * that the `phy` parameter is valid and properly initialized before
 * calling this function.
 *
 * @param phy A pointer to a valid `adrv9002_rf_phy` structure representing the
 * ADRV9002 RF PHY device. Must not be null.
 * @param function A string representing the name of the function where the
 * error occurred. Must not be null.
 * @param line An integer representing the line number in the source code where
 * the error occurred.
 * @return Returns an integer error code corresponding to the type of error
 * encountered, mapped to standard error codes like `-EINVAL`,
 * `-EFAULT`, `-EIO`, or `-ENOMEM`.
 ******************************************************************************/
int __adrv9002_dev_err(const struct adrv9002_rf_phy *phy, const char *function,
		       const int line);
#define adrv9002_dev_err(phy)	__adrv9002_dev_err(phy, __func__, __LINE__)

/* AXI core API's */
int adrv9002_register_axi_converter(struct adrv9002_rf_phy *phy);
/***************************************************************************//**
 * @brief This function configures the AXI interface for the ADRV9002 device,
 * setting the number of lanes, data rate mode, and channel direction. It
 * should be called after the device has been initialized and before data
 * transmission or reception begins. The function handles both transmit
 * and receive channels, and it validates the number of lanes against the
 * SSI type, returning an error if the configuration is invalid.
 *
 * @param phy A pointer to an initialized adrv9002_rf_phy structure representing
 * the device. Must not be null.
 * @param n_lanes The number of lanes to configure, typically 1, 2, or 4. The
 * value must be compatible with the SSI type of the device.
 * @param cmos_ddr A boolean indicating whether to use CMOS DDR mode. Relevant
 * only for CMOS SSI type.
 * @param channel An integer representing the channel to configure, typically 0
 * or 1.
 * @param tx A boolean indicating whether the configuration is for a transmit
 * (true) or receive (false) channel.
 * @return Returns 0 on success or a negative error code if the configuration is
 * invalid.
 ******************************************************************************/
int adrv9002_axi_interface_set(const struct adrv9002_rf_phy *phy,
			       const uint8_t n_lanes,
			       const bool cmos_ddr,
			       const int channel,
			       const bool tx);
/***************************************************************************//**
 * @brief This function is used to tune the clock and data delays for the AXI
 * interface of the ADRV9002 device, which is necessary for ensuring
 * proper data transmission and reception. It should be called when
 * setting up the device's interface, particularly after initialization
 * and before starting data operations. The function requires a valid
 * ADRV9002 RF PHY structure and will adjust the delays based on the
 * specified channel and transmission direction. It returns an error code
 * if the tuning process fails, which can occur if the device is not
 * properly configured or if invalid parameters are provided.
 *
 * @param phy A pointer to a valid adrv9002_rf_phy structure representing the
 * device. Must not be null.
 * @param tx A boolean indicating the direction of transmission. True for TX,
 * false for RX.
 * @param chann An integer specifying the channel to tune. Must be a valid
 * channel number for the device.
 * @param clk_delay A pointer to a uint8_t where the function will store the
 * optimal clock delay value. Must not be null.
 * @param data_delay A pointer to a uint8_t where the function will store the
 * optimal data delay value. Must not be null.
 * @return Returns 0 on success, or a negative error code on failure. The
 * clk_delay and data_delay pointers are updated with the optimal delay
 * values on success.
 ******************************************************************************/
int adrv9002_axi_intf_tune(const struct adrv9002_rf_phy *phy, const bool tx,
			   const int chann,
			   uint8_t *clk_delay, uint8_t *data_delay);
/***************************************************************************//**
 * @brief This function is used to control the state of the AXI interface for a
 * specific channel on the ADRV9002 device. It can be used to either
 * enable or disable the interface for a given transmit (TX) or receive
 * (RX) channel. This function should be called when there is a need to
 * change the operational state of the AXI interface, such as during
 * initialization or when reconfiguring the device. The function requires
 * a valid `adrv9002_rf_phy` structure, which represents the device
 * context, and it must be called with appropriate channel and interface
 * type parameters. The function does not return a value, and it assumes
 * that the provided parameters are valid.
 *
 * @param phy A pointer to an `adrv9002_rf_phy` structure representing the
 * device context. Must not be null.
 * @param chan An integer representing the channel number. Typically, 0 or 1,
 * corresponding to the first or second channel.
 * @param tx A boolean indicating whether the operation is for a transmit (TX)
 * channel (true) or a receive (RX) channel (false).
 * @param en A boolean indicating whether to enable (true) or disable (false)
 * the AXI interface for the specified channel.
 * @return None
 ******************************************************************************/
void adrv9002_axi_interface_enable(struct adrv9002_rf_phy *phy, const int chan,
				   const bool tx, const bool en);
int __maybe_unused adrv9002_axi_tx_test_pattern_cfg(struct adrv9002_rf_phy *phy,
		const int channel,
		const adi_adrv9001_SsiTestModeData_e data);
int adrv9002_post_init(struct adrv9002_rf_phy *phy);
/***************************************************************************//**
 * @brief This function is used to configure the ADRV9002 device's interface for
 * either transmission (Tx) or reception (Rx) test modes. It allows
 * enabling or stopping the test mode on a specified channel. The
 * function must be called with a valid ADRV9002 RF PHY structure and a
 * valid channel index. It is important to ensure that the channel is
 * enabled and properly configured before invoking this function. The
 * function handles different test data configurations based on the SSI
 * type and whether the test mode is being started or stopped. It returns
 * an error code if the configuration fails.
 *
 * @param phy A pointer to a constant adrv9002_rf_phy structure representing the
 * ADRV9002 device. Must not be null. The caller retains ownership.
 * @param chann An integer representing the channel index to configure. Must be
 * a valid channel index for the device.
 * @param tx A boolean indicating whether to configure the transmission (true)
 * or reception (false) test mode.
 * @param stop A boolean indicating whether to stop (true) or start (false) the
 * test mode.
 * @return Returns an integer error code. A return value of 0 indicates success,
 * while a non-zero value indicates an error occurred during
 * configuration.
 ******************************************************************************/
int adrv9002_intf_test_cfg(const struct adrv9002_rf_phy *phy, const int chann,
			   const bool tx,
			   const bool stop);
/***************************************************************************//**
 * @brief This function is used to verify the integrity of the transmitted test
 * pattern on a specified channel of the ADRV9002 device. It should be
 * called when you need to ensure that the transmission is error-free,
 * particularly in test or diagnostic scenarios. The function checks for
 * data errors and returns a status indicating the presence of such
 * errors. It is important to ensure that the channel index provided is
 * valid and that the device is properly initialized before calling this
 * function. The function may also check an additional channel if the
 * device is configured for RX2TX2 mode.
 *
 * @param phy A pointer to an initialized adrv9002_rf_phy structure representing
 * the device. Must not be null.
 * @param chann An integer representing the channel index to check. Must be
 * within the valid range of available channels for the device.
 * @return Returns 0 if no data errors are detected, 1 if data errors are
 * present, or a negative error code if the function fails to execute
 * properly.
 ******************************************************************************/
int adrv9002_check_tx_test_pattern(const struct adrv9002_rf_phy *phy,
				   const int chann);
/***************************************************************************//**
 * @brief This function is used to configure the clock and data delay settings
 * for a specific channel on the ADRV9002 device. It is applicable for
 * both transmit (TX) and receive (RX) paths, as determined by the `tx`
 * parameter. The function should be called when there is a need to fine-
 * tune the interface timing to ensure proper data alignment. It is
 * important to ensure that the `phy` parameter is a valid pointer to an
 * initialized ADRV9002 RF PHY structure. The function returns an integer
 * status code indicating success or failure of the operation.
 *
 * @param phy A pointer to a valid `adrv9002_rf_phy` structure representing the
 * ADRV9002 device. Must not be null.
 * @param channel An integer representing the channel number to configure. Valid
 * values depend on the device configuration.
 * @param clk_delay A uint8_t value specifying the clock delay to set. The valid
 * range is device-specific.
 * @param data_delay A uint8_t value specifying the data delay to set. The valid
 * range is device-specific.
 * @param tx A boolean indicating whether the delay is for the transmit path
 * (true) or receive path (false).
 * @return Returns an integer status code, where 0 indicates success and a non-
 * zero value indicates an error.
 ******************************************************************************/
int adrv9002_intf_change_delay(const struct adrv9002_rf_phy *phy,
			       const int channel,
			       uint8_t clk_delay,
			       uint8_t data_delay, const bool tx);
/***************************************************************************//**
 * @brief This function is used to obtain the Direct Digital Synthesis (DDS)
 * rate for a specified channel of the ADRV9002 device. It should be
 * called when the DDS rate needs to be known for either channel 1 or
 * channel 2. The function requires a valid `adrv9002_rf_phy` structure,
 * which represents the physical layer configuration of the ADRV9002
 * device. The channel parameter determines which channel's DDS rate is
 * retrieved, with 0 typically representing channel 1 and 1 representing
 * channel 2. The function returns the DDS rate as a 32-bit unsigned
 * integer.
 *
 * @param phy A pointer to a `adrv9002_rf_phy` structure representing the
 * ADRV9002 device configuration. Must not be null.
 * @param chan An integer specifying the channel for which to retrieve the DDS
 * rate. Typically, 0 for channel 1 and 1 for channel 2. Invalid
 * values may result in undefined behavior.
 * @return Returns the DDS rate as a 32-bit unsigned integer for the specified
 * channel.
 ******************************************************************************/
uint32_t adrv9002_axi_dds_rate_get(const struct adrv9002_rf_phy *phy,
				   const int chan);

/***************************************************************************//**
 * @brief Use this function to obtain a pointer to the current SPI settings
 * structure for the ADRV9002 device. This function is useful when you
 * need to access or modify the SPI settings directly. It is important to
 * ensure that the returned pointer is not null before using it, as it
 * points to a statically allocated structure. The caller does not own
 * the returned pointer and should not attempt to free it.
 *
 * @return Returns a pointer to a struct adi_adrv9001_SpiSettings containing the
 * current SPI settings.
 ******************************************************************************/
/***************************************************************************//**
 * @brief The `adrv9002_spi_settings_get` function is a global function that
 * returns a pointer to a `struct adi_adrv9001_SpiSettings`. This
 * structure likely contains configuration settings for the SPI interface
 * used by the ADRV9002 device.
 *
 * @details This function is used to retrieve the current SPI settings for the
 * ADRV9002 device, which can be used for configuring or interfacing
 * with the device.
 ******************************************************************************/
struct adi_adrv9001_SpiSettings *adrv9002_spi_settings_get(void);


/***************************************************************************//**
 * @brief The `adrv9002_sync_gpio_toogle` function toggles the SSI sync GPIO pin
 * if the `rx2tx2` mode is enabled in the ADRV9002 RF PHY structure.
 *
 * @param phy A pointer to a constant `adrv9002_rf_phy` structure, which
 * contains the configuration and state of the ADRV9002 RF PHY
 * device.
 * @return This function does not return any value; it performs a hardware
 * operation by toggling a GPIO pin.
 ******************************************************************************/
static inline void adrv9002_sync_gpio_toogle(const struct adrv9002_rf_phy *phy)
{
	if (phy->rx2tx2) {
		/* toogle ssi sync gpio */
		no_os_gpio_set_value(phy->ssi_sync, 1);
		no_os_udelay(5000);
		no_os_gpio_set_value(phy->ssi_sync, 0);
	}
}
#endif
