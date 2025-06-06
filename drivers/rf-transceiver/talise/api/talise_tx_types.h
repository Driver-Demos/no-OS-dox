/**
 * \file talise_tx_types.h
 * \brief Contains Talise API Tx datapath data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_TX_TYPES_H_
#define TALISE_TX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseTxChannels_t` is an enumeration that defines the possible
 * states for enabling transmission channels in the Talise API. It
 * provides four options: no channels enabled, only Tx1 enabled, only Tx2
 * enabled, or both Tx1 and Tx2 enabled. This enum is used to configure
 * which transmission channels are active in a given configuration.
 *
 * @param TAL_TXOFF No Tx channels are enabled.
 * @param TAL_TX1 Tx1 channel enabled.
 * @param TAL_TX2 Tx2 channel enabled.
 * @param TAL_TX1TX2 Tx1 + Tx2 channels enabled.
 ******************************************************************************/
typedef enum {
	TAL_TXOFF = 0,      /*!< No Tx channels are enabled */
	TAL_TX1,            /*!< Tx1 channel enabled */
	TAL_TX2,            /*!< Tx2 channel enabled */
	TAL_TX1TX2          /*!< Tx1 + Tx2 channels enabled */
} taliseTxChannels_t;

/***************************************************************************//**
 * @brief The `taliseDacFullScale_t` is an enumeration that defines the possible
 * full scale boost options for the DAC in the Talise API. It provides
 * two options: no boost (0 dB) and a 3 dB boost, allowing for adjustment
 * of the DAC's output level.
 *
 * @param TAL_DACFS_0DB No Full Scale Boost.
 * @param TAL_DACFS_3DB Full scale boost = 3dB.
 ******************************************************************************/
typedef enum {
	TAL_DACFS_0DB	= 0x0,	/*!< No Full Scale Boost */
	TAL_DACFS_3DB	= 0x1F  /*!< Full scale boost = 3dB */
} taliseDacFullScale_t;

/***************************************************************************//**
 * @brief The `taliseTxAttenStepSize_t` is an enumeration that defines the
 * possible step sizes for adjusting the transmission attenuation in a
 * Talise device. Each enumerator represents a specific attenuation step
 * size in decibels (dB), allowing for precise control over the
 * attenuation level in increments of 0.05dB, 0.1dB, 0.2dB, or 0.4dB.
 * This is useful for applications requiring fine-tuned control of signal
 * strength.
 *
 * @param TAL_TXATTEN_0P05_DB Tx attenuation 0.05dB step size.
 * @param TAL_TXATTEN_0P1_DB Tx attenuation 0.1dB step size.
 * @param TAL_TXATTEN_0P2_DB Tx attenuation 0.2dB step size.
 * @param TAL_TXATTEN_0P4_DB Tx attenuation 0.4dB step size.
 ******************************************************************************/
typedef enum {
	TAL_TXATTEN_0P05_DB = 0,    /*!< Tx attenuation 0.05dB step size */
	TAL_TXATTEN_0P1_DB = 1,     /*!< Tx attenuation 0.1dB step size */
	TAL_TXATTEN_0P2_DB = 2,     /*!< Tx attenuation 0.2dB step size */
	TAL_TXATTEN_0P4_DB = 3      /*!< Tx attenuation 0.4dB step size */
} taliseTxAttenStepSize_t;

/***************************************************************************//**
 * @brief The `taliseTxNcoTestToneCfg_t` structure is used to configure the test
 * tone settings for the Talise Transmit NCO (Numerically Controlled
 * Oscillator). It includes fields to enable or disable the NCO for both
 * transmitters and to set the desired tone frequencies for Tx1 and Tx2
 * in kilohertz. This configuration is essential for testing and
 * calibrating the transmission path by generating specific tones.
 *
 * @param enable Indicates whether the Tx NCO is enabled (1) or disabled (0) on
 * both transmitters.
 * @param tx1ToneFreq_kHz Specifies the signed frequency in kHz for the desired
 * Tx1 tone.
 * @param tx2ToneFreq_kHz Specifies the signed frequency in kHz for the desired
 * Tx2 tone.
 ******************************************************************************/
typedef struct {
	uint8_t enable;                 /*!< 0 = Disable Tx NCO, 1 = Enable Tx NCO on both transmitters */
	int32_t tx1ToneFreq_kHz;        /*!< Signed frequency in kHz of the desired Tx1 tone */
	int32_t tx2ToneFreq_kHz;        /*!< Signed frequency in kHz of the desired Tx2 tone */
} taliseTxNcoTestToneCfg_t;

/***************************************************************************//**
 * @brief The `taliseTxNCOSelect_t` is an enumeration that defines the possible
 * states for enabling or disabling the Transmit Numerically Controlled
 * Oscillator (NCO) in a Talise device. It provides options to disable
 * the NCO, enable it for either the Tx1 or Tx2 channel individually, or
 * enable it for both channels simultaneously. This enumeration is used
 * to configure the NCO settings in the Talise API, allowing for flexible
 * control over the transmission channels.
 *
 * @param TAL_TXNCO_DISABLE TX NCO Disable.
 * @param TAL_TXNCO_TX1_ENABLE TX NCO Tx1 Enable.
 * @param TAL_TXNCO_TX2_ENABLE TX NCO Tx2 Enable.
 * @param TAL_TXNCO_ENABLE_ALL TX NCO Tx1 & Tx2 Enable.
 ******************************************************************************/
typedef enum {
	TAL_TXNCO_DISABLE = 0,        /*!< TX NCO Disable */
	TAL_TXNCO_TX1_ENABLE = 1,     /*!< TX NCO Tx1 Enable */
	TAL_TXNCO_TX2_ENABLE = 2,     /*!< TX NCO Tx2 Enable */
	TAL_TXNCO_ENABLE_ALL = 3      /*!< TX NCO Tx1 & Tx2 Enable */
} taliseTxNCOSelect_t;

/***************************************************************************//**
 * @brief The `taliseTxNcoShifterCfg_t` structure is used to configure the
 * Talise Transmit Numerically Controlled Oscillator (NCO) settings,
 * including enabling or disabling the NCO for specific transmit channels
 * (Tx1 or Tx2), setting the frequency of the tones in kHz, and
 * configuring the phase of the tones in degrees for both Tx1 and Tx2.
 * This structure allows for precise control over the tone generation in
 * the transmit path of the Talise device.
 *
 * @param enableNCO Specifies whether the NCO is disabled or enabled for Tx1 or
 * Tx2.
 * @param tx1ToneFreq_kHz Signed frequency in kHz of the desired Tx1 tone.
 * @param tx2ToneFreq_kHz Signed frequency in kHz of the desired Tx2 tone.
 * @param tx1TonePhaseDeg Tx1 Tone Phase degree.
 * @param tx2TonePhaseDeg Tx2 Tone Phase degree.
 ******************************************************************************/
typedef struct {
	taliseTxNCOSelect_t
	enableNCO; /* < 0 = Disable NCO, 1 = Enable Tx1 NCO, 2 = Enable Tx2 NCO, */
	int32_t tx1ToneFreq_kHz; /*!< Signed frequency in kHz of the desired Tx1 tone */
	int32_t tx2ToneFreq_kHz; /*!< Signed frequency in kHz of the desired Tx2 tone */
	int32_t tx1TonePhaseDeg;                         /*!< Tx1 Tone Phase degree */
	int32_t tx2TonePhaseDeg;                         /*!< Tx2 Tone Phase degree */
} taliseTxNcoShifterCfg_t;


/***************************************************************************//**
 * @brief The `taliseTxAttenCtrlPin_t` structure is designed to manage the
 * control of transmission attenuation through GPIO pins. It allows for
 * the configuration of step size for attenuation adjustments, and
 * specifies which GPIO pins are used to increment or decrement the
 * attenuation for different transmission channels. The structure also
 * includes an enable flag to activate or deactivate the pin control
 * functionality, providing a flexible interface for managing
 * transmission power levels in a Talise API environment.
 *
 * @param stepSize The step size for increasing or decreasing channel
 * attenuation, with a valid range from 0 to 31.
 * @param txAttenIncPin GPIO pin used to increment Tx attenuation, selectable
 * for Tx1 or Tx2.
 * @param txAttenDecPin GPIO pin used to decrement Tx attenuation, selectable
 * for Tx1 or Tx2.
 * @param enable Flag to enable (1) or disable (0) the attenuation pin control.
 ******************************************************************************/
typedef struct {
	uint8_t stepSize;                   /*!< The step that will increase or decrease the channel attenuation. This parameter sets the
                                                 change in Tx attenuation for each increment or decrement signal received in incr/decr mode.
                                                 Step of 1 changes attenuation by 0.05dB. Valid range is from 0 to 31 */
	taliseGpioPinSel_t txAttenIncPin;   /*!< GPIO used to increment Tx attenuation
                                                 Tx1 : TAL_GPIO_04 or TAL_GPIO_12
                                                 Tx2 : TAL_GPIO_06 or TAL_GPIO_14 */
	taliseGpioPinSel_t txAttenDecPin;    /*!< GPIO used to decrement Tx attenuation
                                                 Tx1 : TAL_GPIO_05 or TAL_GPIO_13
                                                 Tx2 : TAL_GPIO_07 or TAL_GPIO_15 */
	uint8_t enable;                     /*!< Enable (1) or disable (0) the attenuation pin control */
} taliseTxAttenCtrlPin_t;

/***************************************************************************//**
 * @brief The `taliseTxPaProtectCfg_t` structure is used to configure the PA
 * protection settings for a Talise transmitter. It includes parameters
 * for averaging the power measurement over a specified number of
 * samples, adjusting the Tx attenuation when protection thresholds are
 * met, and setting power and peak thresholds for both Tx1 and Tx2. The
 * structure allows for detailed control over how the transmitter
 * responds to power levels that could potentially damage the power
 * amplifier, ensuring safe operation by flagging errors when thresholds
 * are exceeded.
 *
 * @param avgDuration Specifies the number of Tx samples to average for power
 * measurement, calculated as 2^(avgDuration + 5).
 * @param txAttenStep Defines the step size for Tx attenuation adjustment when
 * PA protection threshold is met.
 * @param tx1PowerThreshold Sets the power threshold for Tx1, calculated as
 * round(4096 * 10^(tx1PowerThreshold_dBFS / 10)).
 * @param tx2PowerThreshold Sets the power threshold for Tx2, calculated as
 * round(4096 * 10^(tx2PowerThreshold_dBFS / 10)).
 * @param peakCount Determines the number of times the peak power threshold can
 * be exceeded before flagging a PA error.
 * @param tx1PeakThreshold Defines the 8-bit threshold for Tx1 peak detection,
 * calculated as round(128 * 10^(tx1PeakThreshold_dBFS /
 * 10)).
 * @param tx2PeakThreshold Defines the 8-bit threshold for Tx2 peak detection,
 * calculated as round(128 * 10^(tx2PeakThreshold_dBFS /
 * 10)).
 ******************************************************************************/
typedef struct {
	uint8_t avgDuration;          /*!< Number of Tx samples (at JESD204 IQ sample rate) to average for the power measurement.
                                           samples = 2^(avgDuration + 5), 0 = 32 samples, max:14 = 524288 samples */
	uint8_t txAttenStep;          /*!< if PA protection threshold met, Tx Atten = TxAttenSetting + (txAttenStep * 0.4dB) */
	uint16_t tx1PowerThreshold;   /*!< tx1PowerThreashold = round(4096 * 10^(tx1PowerThreshold_dBFS / 10)) (valid 1-8191) */
	uint16_t tx2PowerThreshold;   /*!< tx2PowerThreashold = round(4096 * 10^(tx2PowerThreshold_dBFS / 10)) (valid 1-8191) */

	uint8_t peakCount;            /*!< 0=Peak Mode is disabled, if the Tx peak power threshold is exceeded more than peakCount times within
                                           one average duration, a PA error is flagged (Si Rev 0xB1: 0-30, Si Rev 0xC0: 0-31) */
	uint8_t tx1PeakThreshold;     /*!< 8-bit threshold for Tx1 peak detect. When instantaneous power exceeds this threshold, a peak is registered (valid 1-255)
                                           tx1PeakThreshold = round(128 * 10^(tx1PeakThreshold_dBFS / 10)) */
	uint8_t tx2PeakThreshold;     /*!< 8-bit threshold for Tx2 peak detect. When instantaneous power exceeds this threshold, a peak is registered (valid 1-255)
                                           tx2PeakThreashold = round(128 * 10^(tx2PeakThreshold_dBFS / 10))*/
} taliseTxPaProtectCfg_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_TX_TYPES_H_ */
