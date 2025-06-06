/**
 * \file talise_rx_types.h
 * \brief Contains Talise API Rx datapath data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_RX_TYPES_H_
#define TALISE_RX_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_radioctrl_types.h"

/***************************************************************************//**
 * @brief The `taliseRxChannels_t` is an enumeration that defines the possible
 * states for enabling the receive (Rx) channels in a Talise radio
 * system. It provides options to enable no channels, only the first
 * channel (Rx1), only the second channel (Rx2), or both channels (Rx1
 * and Rx2) simultaneously. This enumeration is used to configure the
 * channel enablement in the system, allowing for flexible control over
 * which channels are active at any given time.
 *
 * @param TAL_RXOFF No Rx channels are enabled.
 * @param TAL_RX1 Rx1 channel enabled.
 * @param TAL_RX2 Rx2 channel enabled.
 * @param TAL_RX1RX2 Rx1 + Rx2 channels enabled.
 ******************************************************************************/
typedef enum {
	TAL_RXOFF = 0,      /*!< No Rx channels are enabled */
	TAL_RX1,            /*!< Rx1 channel enabled */
	TAL_RX2,            /*!< Rx2 channel enabled */
	TAL_RX1RX2          /*!< Rx1 + Rx2 channels enabled */
} taliseRxChannels_t;

/***************************************************************************//**
 * @brief The `taliseObsRxChannels_t` is an enumeration that defines the
 * possible states for enabling Observation Rx channels in the Talise
 * API. It is used to specify which Observation Rx channels should be
 * enabled and calibrated during initialization. The enum values indicate
 * whether no channels, only Rx1, only Rx2, or both Rx1 and Rx2 channels
 * are active, which is crucial for configuring the system's observation
 * capabilities and ensuring the ARM processor is aware of the valid
 * observation channels for the current system setup.
 *
 * @param TAL_ORXOFF Represents the state where no Observation Rx channels are
 * enabled.
 * @param TAL_ORX1 Represents the state where the Observation Rx1 channel is
 * enabled.
 * @param TAL_ORX2 Represents the state where the Observation Rx2 channel is
 * enabled.
 * @param TAL_ORX1ORX2 Represents the state where both Observation Rx1 and Rx2
 * channels are enabled.
 ******************************************************************************/
typedef enum {
	TAL_ORXOFF      = 0x00,
	TAL_ORX1        = 0x01,
	TAL_ORX2        = 0x02,
	TAL_ORX1ORX2    = 0x03
} taliseObsRxChannels_t;

/***************************************************************************//**
 * @brief The `taliseGainMode_t` is an enumeration that defines the different
 * gain control modes available for the Talise receiver. It includes
 * options for manual gain control, fast attack automatic gain control
 * (AGC) suitable for time-division duplex (TDD) mode, slow loop AGC for
 * both frequency-division duplex (FDD) and TDD modes, and a hybrid AGC
 * mode. This enumeration allows the user to select the appropriate gain
 * control strategy based on the operational requirements of the
 * receiver.
 *
 * @param TAL_MGC Manual Gain Control.
 * @param TAL_AGCFAST Fast Attack AGC Mode for TDD mode.
 * @param TAL_AGCSLOW Slow Loop AGC for FDD and TDD modes.
 * @param TAL_HYBRID Hybrid AGC Gain Control.
 ******************************************************************************/
typedef enum {
	TAL_MGC = 0,        /*!< Manual Gain Control */
	TAL_AGCFAST = 1,    /*!< 01 Fast Attack AGC Mode TDD mode */
	TAL_AGCSLOW = 2,    /*!< Slow Loop AGC FDD, TDD modes  */
	TAL_HYBRID = 3      /*!< Hybrid AGC Gain Control */
} taliseGainMode_t;

/***************************************************************************//**
 * @brief The `taliseRxGainTable_t` structure is used to define the gain
 * settings for a receiver in the Talise API. It includes parameters for
 * front-end gain, external control, ADC TIA gain, digital gain, and
 * phase offset, allowing for comprehensive configuration of the
 * receiver's gain characteristics.
 *
 * @param rxFeGain Represents the front-end gain for the receiver.
 * @param extControl Holds external control settings for the receiver.
 * @param adcTiaGain Specifies the gain setting for the ADC TIA (Transimpedance
 * Amplifier).
 * @param digGain Defines the digital gain applied to the received signal.
 * @param phaseOffset Indicates the phase offset applied to the received signal.
 ******************************************************************************/
typedef struct {
	uint8_t rxFeGain;
	uint8_t extControl;
	uint8_t adcTiaGain;
	int16_t digGain;
	uint16_t phaseOffset;
} taliseRxGainTable_t;

/***************************************************************************//**
 * @brief The `taliseOrxGainTable_t` structure is used to define the gain
 * settings for the observation receiver (ORx) path in the Talise API. It
 * includes fields for configuring the front-end gain, external control,
 * ADC TIA gain, and digital gain, allowing for precise control over the
 * gain stages in the ORx signal path.
 *
 * @param rxFeGain Represents the front-end gain for the receiver.
 * @param extControl Holds the external control value for additional gain
 * control.
 * @param adcTiaGain Specifies the gain setting for the ADC TIA (Transimpedance
 * Amplifier).
 * @param digGain Indicates the digital gain applied to the signal.
 ******************************************************************************/
typedef struct {
	uint8_t rxFeGain;
	uint8_t extControl;
	uint8_t adcTiaGain;
	int16_t digGain;
} taliseOrxGainTable_t;

/***************************************************************************//**
 * @brief The `taliseFpExponentModes_t` is an enumeration that defines the
 * different configurations for the number of exponent bits in a floating
 * point representation used in the Talise API. Each enumerator specifies
 * a unique configuration of exponent bits, significand bits, and a sign
 * bit, allowing for flexibility in representing floating point numbers
 * with varying precision and range.
 *
 * @param TAL_2_EXPONENTBITS Floating point values have 2 exponent bits, 13
 * significand bits, 1 sign bit.
 * @param TAL_3_EXPONENTBITS Floating point values have 3 exponent bits, 12
 * significand bits, 1 sign bit.
 * @param TAL_4_EXPONENTBITS Floating point values have 4 exponent bits, 11
 * significand bits, 1 sign bit.
 * @param TAL_5_EXPONENTBITS Floating point values have 5 exponent bits, 10
 * significand bits, 1 sign bit.
 ******************************************************************************/
typedef enum {
	TAL_2_EXPONENTBITS = 0,     /*!< Floating point values have 2 exponent bits, 13 significand bits, 1 sign bit */
	TAL_3_EXPONENTBITS = 1,     /*!< Floating point values have 3 exponent bits, 12 significand bits, 1 sign bit */
	TAL_4_EXPONENTBITS = 2,     /*!< Floating point values have 4 exponent bits, 11 significand bits, 1 sign bit */
	TAL_5_EXPONENTBITS = 3      /*!< Floating point values have 5 exponent bits, 10 significand bits, 1 sign bit  */
} taliseFpExponentModes_t;

/***************************************************************************//**
 * @brief The `taliseFpRoundModes_t` is an enumeration that defines the
 * different rounding modes available for floating point operations in
 * the Talise API's Rx data path. These modes determine how floating
 * point numbers are rounded during processing, providing options for
 * rounding to even, towards positive or negative infinity, towards zero,
 * or away from even values. This allows for precise control over
 * numerical rounding behavior in floating point calculations.
 *
 * @param TAL_ROUND_TO_EVEN Round floating point ties to an even value.
 * @param TAL_ROUNDTOWARDS_POSITIVE Round floating point toward the positive
 * direction.
 * @param TAL_ROUNDTOWARDS_NEGATIVE Round floating point toward the negative
 * direction.
 * @param TAL_ROUNDTOWARDS_ZERO Round floating point toward the zero direction.
 * @param TAL_ROUND_FROM_EVEN Round floating point ties away from even value.
 ******************************************************************************/
typedef enum {
	TAL_ROUND_TO_EVEN = 0,           /*!< Round floating point ties to an even value          */
	TAL_ROUNDTOWARDS_POSITIVE = 1,   /*!< Round floating point toward the positive direction  */
	TAL_ROUNDTOWARDS_NEGATIVE = 2,   /*!< Round floating point toward the negative direction  */
	TAL_ROUNDTOWARDS_ZERO = 3,       /*!< Round floating point toward the zero direction      */
	TAL_ROUND_FROM_EVEN = 4          /*!< Round floating point ties away from even value      */
} taliseFpRoundModes_t;

/***************************************************************************//**
 * @brief The `taliseFpAttenSteps_t` is an enumeration that defines various
 * attenuation levels for the Rx path when the data format is set to
 * floating point mode. Each enumerator represents a specific attenuation
 * value in decibels (dB), ranging from -18 dB to 24 dB, allowing for
 * precise control over the signal attenuation in floating point mode.
 *
 * @param TAL_FPATTEN_24DB Set Rx attenuation to 24 dB when Rx Data format set
 * to floating point mode.
 * @param TAL_FPATTEN_18DB Set Rx attenuation to 18 dB when Rx Data format set
 * to floating point mode.
 * @param TAL_FPATTEN_12DB Set Rx attenuation to 12 dB when Rx Data format set
 * to floating point mode.
 * @param TAL_FPATTEN_6DB Set Rx attenuation to 6 dB when Rx Data format set to
 * floating point mode.
 * @param TAL_FPATTEN_0DB Set Rx attenuation to 0 dB when Rx Data format set to
 * floating point mode.
 * @param TAL_FPATTEN_MINUS6DB Set Rx attenuation to -6 dB when Rx Data format
 * set to floating point mode.
 * @param TAL_FPATTEN_MINUS12DB Set Rx attenuation to -12 dB when Rx Data format
 * set to floating point mode.
 * @param TAL_FPATTEN_MINUS18DB Set Rx attenuation to -18 dB when Rx Data format
 * set to floating point mode.
 ******************************************************************************/
typedef enum {
	TAL_FPATTEN_24DB = 4,      /*!< Set Rx attenuation to 24 dB when Rx Data format set to floating point mode  */
	TAL_FPATTEN_18DB = 5,      /*!< Set Rx attenuation to 18 dB when Rx Data format set to floating point mode  */
	TAL_FPATTEN_12DB = 6,      /*!< Set Rx attenuation to 12 dB when Rx Data format set to floating point mode */
	TAL_FPATTEN_6DB = 7,       /*!< Set Rx attenuation to 6 dB when Rx Data format set to floating point mode */
	TAL_FPATTEN_0DB = 0,            /*!< Set Rx attenuation to 0 dB when Rx Data format set to floating point mode */
	TAL_FPATTEN_MINUS6DB = 1,            /*!< Set Rx attenuation to -6 dB when Rx Data format set to floating point mode */
	TAL_FPATTEN_MINUS12DB = 2,           /*!< Set Rx attenuation to -12 dB when Rx Data format set to floating point mode */
	TAL_FPATTEN_MINUS18DB = 3            /*!< Set Rx attenuation to -18 dB when Rx Data format set to floating point mode */
} taliseFpAttenSteps_t;

/***************************************************************************//**
 * @brief The `taliseDataFormattingModes_t` is an enumeration that defines
 * various modes for data formatting and gain compensation in the Talise
 * API. It provides options to enable or disable gain compensation and
 * select between floating point or integer data formatting, with
 * additional configurations for internal or external slicers and GPIO
 * outputs. This enum is crucial for configuring the data path in Talise
 * devices, allowing for flexible data handling and processing based on
 * the specific requirements of the application.
 *
 * @param TAL_GAIN_COMPENSATION_DISABLED Gain Compensation and Data Formatting
 * are disabled.
 * @param TAL_GAIN_WITH_FLOATING_POINT Gain Compensation enabled with floating
 * point data formatting enabled and
 * internal slicer enabled.
 * @param TAL_GAIN_WITH_INTSLICER_NOGPIO Gain Compensation enabled with integer
 * data formatting and internal slicer
 * enabled with no GPIO Slicer Position
 * output.
 * @param TAL_GAIN_WITH_INTSLICER Gain Compensation enabled with integer data
 * formatting and internal slicer enabled with
 * GPIO Slicer Position output.
 * @param TAL_GAIN_WITH_EXTERNAL_SLICER Gain Compensation enabled with integer
 * data formatting and external slicer
 * enabled.
 ******************************************************************************/
typedef enum {
	TAL_GAIN_COMPENSATION_DISABLED = 0,  /*!< Gain Compensation and Data Formatting are disabled */
	TAL_GAIN_WITH_FLOATING_POINT,        /*!< Gain Compensation enabled with floating point data formatting enabled and internal slicer enabled */
	TAL_GAIN_WITH_INTSLICER_NOGPIO,      /*!< Gain Compensation enabled with integer data formatting and internal slicer enabled with no GPIO Slicer Position output */
	TAL_GAIN_WITH_INTSLICER,             /*!< Gain Compensation enabled with integer data formatting and internal slicer enabled with GPIO Slicer Position output */
	TAL_GAIN_WITH_EXTERNAL_SLICER        /*!< Gain Compensation enabled with integer data formatting and external slicer enabled */
} taliseDataFormattingModes_t;

/***************************************************************************//**
 * @brief The `taliseGainStepSize_t` is an enumeration that defines the possible
 * external gain step sizes for the Gain Slicer in the Talise API. It
 * allows the selection of discrete gain step sizes ranging from 1dB to
 * 4dB, which are used to adjust the gain in the external slicer mode by
 * multiplying the selected step size with the base band processor's step
 * size selection.
 *
 * @param TAL_EXTSLICER_STEPSIZE_1DB Set Gain Slicer External gain step size to
 * 1dB.
 * @param TAL_EXTSLICER_STEPSIZE_2DB Set Gain Slicer External gain step size to
 * 2dB.
 * @param TAL_EXTSLICER_STEPSIZE_3DB Set Gain Slicer External gain step size to
 * 3dB.
 * @param TAL_EXTSLICER_STEPSIZE_4DB Set Gain Slicer External gain step size to
 * 4dB.
 ******************************************************************************/
typedef enum {
	TAL_EXTSLICER_STEPSIZE_1DB = 0,   /*!< Set Gain Slicer External gain step size to 1dB  */
	TAL_EXTSLICER_STEPSIZE_2DB,       /*!< Set Gain Slicer External gain step size to 2dB  */
	TAL_EXTSLICER_STEPSIZE_3DB,       /*!< Set Gain Slicer External gain step size to 3dB  */
	TAL_EXTSLICER_STEPSIZE_4DB        /*!< Set Gain Slicer External gain step size to 4dB  */
} taliseGainStepSize_t;

/***************************************************************************//**
 * @brief The `taliseRx1ExtSlicerGpioSelect_t` is an enumeration that defines
 * the possible GPIO configurations for the Rx1 Gain Slicer in the Talise
 * API. It allows the selection of specific GPIO pins (GPIO0, GPIO1,
 * GPIO2, GPIO5, GPIO6, GPIO7, GPIO8, GPIO9, GPIO10) to be used for
 * external gain control, or the option to disable the external GPIO
 * control for the Rx1 Gain Slicer. This configuration is crucial for
 * setting up the external gain slicer functionality in the Talise radio
 * system.
 *
 * @param TAL_EXTSLICER_RX1_GPIO0_1_2 Selects Rx1 Gain Slicer External GPIO0,
 * GPIO1, GPIO2.
 * @param TAL_EXTSLICER_RX1_GPIO5_6_7 Selects Rx1 Gain Slicer External GPIO5,
 * GPIO6, GPIO7.
 * @param TAL_EXTSLICER_RX1_GPIO8_9_10 Selects Rx1 Gain Slicer External GPIO8,
 * GPIO9, GPIO10.
 * @param TAL_EXTSLICER_RX1_GPIO_DISABLE Disables Rx1 Gain Slicer External GPIO.
 ******************************************************************************/
typedef enum {
	TAL_EXTSLICER_RX1_GPIO0_1_2 = 0,      /*!< Select Rx1 Gain Slicer External GPIO0, GPIO1, GPIO2 */
	TAL_EXTSLICER_RX1_GPIO5_6_7,          /*!< Select Rx1 Gain Slicer External GPIO5, GPIO6, GPIO7 */
	TAL_EXTSLICER_RX1_GPIO8_9_10,         /*!< Select Rx1 Gain Slicer External GPIO8, GPIO9, GPIO10 */
	TAL_EXTSLICER_RX1_GPIO_DISABLE        /*!< Select Rx1 Disable Gain Slicer External GPIO */
} taliseRx1ExtSlicerGpioSelect_t;

/***************************************************************************//**
 * @brief The `taliseRx2ExtSlicerGpioSelect_t` is an enumeration that defines
 * the possible GPIO configurations for the Rx2 Gain Slicer in the Talise
 * API. It allows the selection of specific GPIO pins (either GPIO11,
 * GPIO12, GPIO13 or GPIO5, GPIO6, GPIO7) for external control of the Rx2
 * Gain Slicer, or the option to disable the external GPIO control
 * altogether. This configuration is crucial for managing the gain slicer
 * settings in the Rx2 channel of the Talise transceiver.
 *
 * @param TAL_EXTSLICER_RX2_GPIO11_12_13 Selects Rx2 Gain Slicer External
 * GPIO11, GPIO12, GPIO13.
 * @param TAL_EXTSLICER_RX2_GPIO5_6_7 Selects Rx2 Gain Slicer External GPIO5,
 * GPIO6, GPIO7.
 * @param TAL_EXTSLICER_RX2_GPIO_DISABLE Selects Rx2 Disable Gain Slicer
 * External GPIO.
 ******************************************************************************/
typedef enum {
	TAL_EXTSLICER_RX2_GPIO11_12_13 = 0,   /*!< Select Rx2 Gain Slicer External GPIO11, GPIO12, GPIO13 */
	TAL_EXTSLICER_RX2_GPIO5_6_7 = 1,      /*!< Select Rx2 Gain Slicer External GPIO5, GPIO6, GPIO7 */
	TAL_EXTSLICER_RX2_GPIO_DISABLE = 3    /*!< Select Rx2 Disable Gain Slicer External GPIO */
} taliseRx2ExtSlicerGpioSelect_t;

/***************************************************************************//**
 * @brief The `taliseEmbeddedBits_t` is an enumeration that defines the
 * configuration options for embedding slicer bits into the data frame of
 * a Talise device. It allows for the selection of either no embedded
 * slicer bits, or embedding 1 or 2 slicer bits on both the I and Q
 * channels at either the most significant bit (MSB) or least significant
 * bit (LSB) positions. This configuration is crucial for data formatting
 * and processing in the Talise API, particularly in applications
 * requiring precise control over data bit embedding.
 *
 * @param TAL_NO_EMBEDDED_SLICER_BITS Disabled all embedded slicer bits.
 * @param TAL_EMBED_1_SLICERBIT_AT_MSB Embeds 1 slicer bit on I and 1 slicer bit
 * on Q at the MSB position in the data
 * frame.
 * @param TAL_EMBED_1_SLICERBIT_AT_LSB Embeds 1 slicer bit on I and 1 slicer bit
 * on Q at the LSB position in the data
 * frame.
 * @param TAL_EMBED_2_SLICERBITS_AT_MSB Embeds 2 slicer bits on I and 2 slicer
 * bits on Q at the MSB position in the
 * data frame.
 * @param TAL_EMBED_2_SLICERBITS_AT_LSB Embeds 2 slicer bits on I and 2 slicer
 * bits on Q at the LSB position in the
 * data frame.
 ******************************************************************************/
typedef enum {
	TAL_NO_EMBEDDED_SLICER_BITS = 0,       /*!< Disabled all embedded slicer bits  */
	TAL_EMBED_1_SLICERBIT_AT_MSB,          /*!< Embeds 1 slicer bits on I and 1 slicer bits on Q and the MSB position in the data frame */
	TAL_EMBED_1_SLICERBIT_AT_LSB,          /*!< Embeds 1 slicer bits on I and 1 slicer bits on Q and the LSB position in the data frame */
	TAL_EMBED_2_SLICERBITS_AT_MSB,         /*!< Embeds 2 slicer bits on I and 2 slicer bits on Q and the MSB position in the data frame */
	TAL_EMBED_2_SLICERBITS_AT_LSB          /*!< Embeds 2 slicer bits on I and 2 slicer bits on Q and the LSB position in the data frame */
} taliseEmbeddedBits_t;

/***************************************************************************//**
 * @brief The `taliseIntSampleResolution_t` is an enumeration that defines
 * various integer sample resolutions for the Talise API, specifically
 * for the Rx datapath. It provides options for 12-bit, 16-bit, and
 * 24-bit resolutions, each available in either two's complement or
 * signed magnitude formats. This allows for flexible configuration of
 * the data representation in the system, catering to different precision
 * and format requirements.
 *
 * @param TAL_INTEGER_12BIT_2SCOMP Selects integer sample 12 bit resolution with
 * 2s complement.
 * @param TAL_INTEGER_12BIT_SIGNED Selects integer sample 12 bit resolution with
 * signed magnitude.
 * @param TAL_INTEGER_16BIT_2SCOMP Selects integer sample 16 bit resolution with
 * 2s complement.
 * @param TAL_INTEGER_16BIT_SIGNED Selects integer sample 16 bit resolution with
 * signed magnitude.
 * @param TAL_INTEGER_24BIT_2SCOMP Selects integer sample 24 bit resolution with
 * 2s complement.
 * @param TAL_INTEGER_24BIT_SIGNED Selects integer sample 24 bit resolution with
 * signed magnitude.
 ******************************************************************************/
typedef enum {
	TAL_INTEGER_12BIT_2SCOMP = 0,    /*!< Selects integer sample 12 bit resolution with 2s compliment    */
	TAL_INTEGER_12BIT_SIGNED,        /*!< Selects integer sample 12 bit resolution with signed magnitude */
	TAL_INTEGER_16BIT_2SCOMP,        /*!< Selects integer sample 16 bit resolution with 2s compliment    */
	TAL_INTEGER_16BIT_SIGNED,        /*!< Selects integer sample 16 bit resolution with signed magnitude */
	TAL_INTEGER_24BIT_2SCOMP,        /*!< Selects integer sample 24 bit resolution with 2s compliment    */
	TAL_INTEGER_24BIT_SIGNED         /*!< Selects integer sample 24 bit resolution with signed magnitude */
} taliseIntSampleResolution_t;

/***************************************************************************//**
 * @brief The `taliseRxDataFormat_t` structure is used to configure the data
 * formatting for the Talise receiver channels. It includes settings for
 * both floating point and integer data formats, allowing for the
 * selection of rounding modes, data output formats, and attenuation
 * levels. The structure also provides configuration options for embedded
 * bits, sample resolution, and external slicer settings, including GPIO
 * configurations and gain step sizes. This comprehensive configuration
 * structure enables precise control over the data path formatting and
 * gain management in the Talise receiver system.
 *
 * @param formatSelect Rx Channel format mode selects.
 * @param fpRoundMode Rounding mode for floating point format.
 * @param fpDataFormat Sets the 16-bit output format for floating point data.
 * @param fpEncodeNan Indicates if the highest exponent value encodes NaN.
 * @param fpNumExpBits Specifies the number of exponent and significand bits in
 * floating point numbers.
 * @param fpHideLeadingOne Indicates if the leading one in the significand is
 * hidden.
 * @param fpRx1Atten Attenuation setting for Rx1 in floating point mode.
 * @param fpRx2Atten Attenuation setting for Rx2 in floating point mode.
 * @param intEmbeddedBits Specifies the number and position of embedded bits in
 * integer mode.
 * @param intSampleResolution Selects the integer sample resolution and format.
 * @param extPinStepSize Selects the external pin gain step size.
 * @param rx1GpioSelect Selects the GPIO configuration for Rx1.
 * @param rx2GpioSelect Selects the GPIO configuration for Rx2.
 * @param externalLnaGain Enables or disables slicer compensation for external
 * dualband LNA.
 * @param tempCompensationEnable Enables or disables slicer compensation for
 * temperature variations.
 ******************************************************************************/
typedef struct {
	taliseDataFormattingModes_t formatSelect; /*!< Rx Channel format mode selects */

	/* Float Config Settings */
	taliseFpRoundModes_t
	fpRoundMode;         /*!< Rounding mode for floating point format (See enum values) */
	uint8_t fpDataFormat;                     /*!< If floating point format is enabled in formatSelect member, this sets the 16 bit output from MSB to LSB, 1 = {Sign, Significand, Exponent}, 0 = {Sign, Exponent, Significand} */
	uint8_t fpEncodeNan;                      /*!< 1 =  encodes the highest value of Exponent to mean NaN (Not a Number) to be compatible to IEEE754 specification (Valid: 0 or 1) */
	taliseFpExponentModes_t
	fpNumExpBits;     /*!< Indicates the number of exponent and significand bits in the floating point number */
	uint8_t fpHideLeadingOne;                 /*!< 1 =  Hides the leading one in significand to be compatible to the IEEE754 specification. 0 = a leading one exists at the MSB of the significand.  (Valid: 0, 1) */
	taliseFpAttenSteps_t
	fpRx1Atten;          /*!< Rx1 - Attenuate integer data when floating point mode enabled, see enum for values from 0dB to 42dB in 6dB steps */
	taliseFpAttenSteps_t
	fpRx2Atten;          /*!< Rx2 - Attenuate integer data when floating point mode enabled, see enum for values from 0dB to 42dB in 6dB steps */

	/* Integer Config Settings */
	taliseEmbeddedBits_t
	intEmbeddedBits;             /*!< Integer number of embedded bits and position */
	taliseIntSampleResolution_t
	intSampleResolution;  /*!< Integer sample resolution selecting either 12, 16, 14 bit modes with signed or 2s Complement */

	/* Slicer Config Settings */
	taliseGainStepSize_t
	extPinStepSize;          /*!< Enum selects the external pin gain step size */
	taliseRx1ExtSlicerGpioSelect_t
	rx1GpioSelect; /*!< Enum selects the Rx1 GPIO Configuration */
	taliseRx2ExtSlicerGpioSelect_t
	rx2GpioSelect; /*!< Enum selects the Rx2 GPIO Configuration */
	uint8_t externalLnaGain;                      /*!< Selects Slicer to compensate for external dualband LNA {0 - disabled, 1 - enabled */
	uint8_t tempCompensationEnable;               /*!< Selects Slicer to compensate for temperature variations {0 - disabled, 1 - enabled */
} taliseRxDataFormat_t;

/***************************************************************************//**
 * @brief The `taliseRxGainCtrlPin_t` structure is designed to manage the gain
 * control of a receiver by using GPIO pins to increment or decrement the
 * gain index. It allows for fine-tuning of the gain through configurable
 * step sizes and provides the ability to enable or disable the gain
 * control functionality. This structure is essential for applications
 * requiring dynamic gain adjustments in response to varying signal
 * conditions.
 *
 * @param incStep Increment in gain index applied when the increment gain pin is
 * pulsed, with a step size of 1 to 8.
 * @param decStep Decrement in gain index applied when the decrement gain pin is
 * pulsed, with a step size of 1 to 8.
 * @param rxGainIncPin GPIO used for the Increment gain input, selectable for
 * Rx1 or Rx2.
 * @param rxGainDecPin GPIO used for the Decrement gain input, selectable for
 * Rx1 or Rx2.
 * @param enable Enable (1) or disable (0) the gain pin control.
 ******************************************************************************/
typedef struct {
	uint8_t incStep;                /*!< Increment in gain index applied when the increment gain pin is pulsed. A value of 0 to 7 applies a step size of 1 to 8 */
	uint8_t decStep;                /*!< Decrement in gain index applied when the increment gain pin is pulsed. A value of 0 to 7 applies a step size of 1 to 8 */
	taliseGpioPinSel_t
	rxGainIncPin;/*!< GPIO used for the Increment gain input: Rx1 : TAL_GPIO_00 or TAL_GPIO_10, Rx2 : TAL_GPIO_03 or TAL_GPIO_13*/
	taliseGpioPinSel_t
	rxGainDecPin;/*!< GPIO used for the Decrement gain input: Rx1 : TAL_GPIO_01 or TAL_GPIO_11, Rx2 : TAL_GPIO_04 or TAL_GPIO_14*/
	uint8_t enable;                 /*!< Enable (1) or disable (0) the gain pin control*/
} taliseRxGainCtrlPin_t;

/***************************************************************************//**
 * @brief The `taliseDualBandLnaGainTable_t` structure is designed to manage the
 * gain settings for a dual-band Low Noise Amplifier (LNA) in a Talise
 * receiver system. It contains two fields: `dualbandControl`, which
 * specifies the external control value for the LNA via GPIOs, and
 * `dualbandGain`, which provides the gain compensation value
 * corresponding to the external control, aiding in RSSI and gain
 * compensation adjustments.
 *
 * @param dualbandControl The external control value to be output on the 3.3V
 * GPIOs to control the LNA (values 0-3).
 * @param dualbandGain The gain compensation value for the corresponding
 * external control, used for RSSI and gain compensation,
 * ranging from 0 to 63 (0 to +31.5db in 0.5db steps).
 ******************************************************************************/
typedef struct {
	uint8_t dualbandControl;    /*!< The external control value to be output on the 3.3V GPIO?s to control the LNA (values 0-3). */
	uint8_t dualbandGain;       /*!< The gain compensation value for the corresponding external control, used for RSSI and gain compensation.
                                     Range of 0 to 63 (0 to +31.5db in 0.5db steps). */
} taliseDualBandLnaGainTable_t;

/***************************************************************************//**
 * @brief The `taliseRxNcoChannel_t` is an enumeration that defines the possible
 * NCO (Numerically Controlled Oscillator) channels for the Talise
 * receiver. It includes channels for both RX1 and RX2, with each having
 * two NCOs (NCO1 and NCO2) and each NCO having two sub-channels (A and
 * B). This enumeration is used to specify which NCO channel is being
 * referred to in the context of the Talise API.
 *
 * @param TAL_RX1_NCO1A Represents the first NCO channel A for RX1.
 * @param TAL_RX1_NCO1B Represents the first NCO channel B for RX1.
 * @param TAL_RX1_NCO2A Represents the second NCO channel A for RX1.
 * @param TAL_RX1_NCO2B Represents the second NCO channel B for RX1.
 * @param TAL_RX2_NCO1A Represents the first NCO channel A for RX2.
 * @param TAL_RX2_NCO1B Represents the first NCO channel B for RX2.
 * @param TAL_RX2_NCO2A Represents the second NCO channel A for RX2.
 * @param TAL_RX2_NCO2B Represents the second NCO channel B for RX2.
 ******************************************************************************/
typedef enum {
	TAL_RX1_NCO1A = 0,
	TAL_RX1_NCO1B,
	TAL_RX1_NCO2A,
	TAL_RX1_NCO2B,

	TAL_RX2_NCO1A,
	TAL_RX2_NCO1B,
	TAL_RX2_NCO2A,
	TAL_RX2_NCO2B
} taliseRxNcoChannel_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_RX_TYPES_H_ */
