/**
* \file
* \brief Contains ADRV9001 API Tx datapath data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the FPGA9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_TX_TYPES_H_
#define _ADI_ADRV9001_TX_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdbool.h>
#endif

#include "adi_adrv9001_auxdac_types.h"
#include "adi_adrv9001_gpio_types.h"

#define ADRV9001_TX_ATTEN_TABLE_MAX 960

#define ADI_ADRV9001_MAX_TXCHANNELS           2
#define ADI_ADRV9001_MAX_TX_CHANNEL_START     0
#define ADI_ADRV9001_MAX_TX_CHANNEL_END       1

#define ADRV9001_ADDR_TX1_ATTEN_TABLE 0x45300000
#define ADRV9001_ADDR_TX2_ATTEN_TABLE 0x45400000

#define ADRV9001_TX_PA_RAMP_MIN_CLK_KHZ     48
#define ADRV9001_TX_PA_RAMP_MAX_CLK_KHZ     1000
#define ADRV9001_TX_PA_RAMP_LUT_SIZE        256

#define ADRV9001_TX_MAX_ATTENUATION_MDB         41950
#define ADRV9001_TX_ATTENUATION_RESOLUTION_MDB  50

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxAttenStepSize_e` is an enumeration that defines
 * the possible step sizes for adjusting the transmission attenuation in
 * the ADRV9001 device. Each enumerator represents a specific attenuation
 * step size in decibels (dB), allowing for fine-grained control over the
 * transmission power level. This is crucial for applications requiring
 * precise power adjustments to optimize signal quality and minimize
 * interference.
 *
 * @param ADI_ADRV9001_TXATTEN_0P05_DB Tx attenuation 0.05dB step size.
 * @param ADI_ADRV9001_TXATTEN_0P1_DB Tx attenuation 0.1dB step size.
 * @param ADI_ADRV9001_TXATTEN_0P2_DB Tx attenuation 0.2dB step size.
 * @param ADI_ADRV9001_TXATTEN_0P4_DB Tx attenuation 0.4dB step size.
 ******************************************************************************/
typedef enum adi_adrv9001_TxAttenStepSize
{
    ADI_ADRV9001_TXATTEN_0P05_DB = 0,    /*!< Tx attenuation 0.05dB step size */
    ADI_ADRV9001_TXATTEN_0P1_DB  = 1,    /*!< Tx attenuation 0.1dB step size */
    ADI_ADRV9001_TXATTEN_0P2_DB  = 2,    /*!< Tx attenuation 0.2dB step size */
    ADI_ADRV9001_TXATTEN_0P4_DB  = 3     /*!< Tx attenuation 0.4dB step size */
} adi_adrv9001_TxAttenStepSize_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxAttenuationControlMode_e` is an enumeration that
 * defines the different modes available for controlling the transmission
 * attenuation in the ADRV9001 device. It includes modes for bypassing
 * attenuation, setting attenuation via SPI with a 10-bit index,
 * controlling attenuation incrementally through GPIO pins, and a closed-
 * loop gain control mode. Each mode provides a different mechanism for
 * managing the attenuation level, allowing for flexible control
 * depending on the application requirements.
 *
 * @param ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_BYPASS Tx attenuation mode
 * Bypass: zero total
 * attenuation.
 * @param ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI Tx attenuation mode set
 * by 10-bit attenuation
 * index used to determine
 * total attenuation.
 * @param ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_PIN Tx attenuation is
 * controlled with GPIO
 * Incr/Decr: total
 * attenuation is altered
 * incrementally using pin
 * control.
 * @param ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC Tx attenuation: Closed
 * Loop Gain Control,
 * limited to this mode if
 * CLGC is enabled in OBJID
 * _CFG_DPD_PRE_INIT_CAL.
 ******************************************************************************/
typedef enum adi_adrv9001_TxAttenuationControlMode
{
    ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_BYPASS  = 0, /*!< Tx attenuation mode Bypass: zero total attenuation */
    ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_SPI     = 1, /*!< Tx attenuation mode set by 10-bit attenuation index used to determine total attenuation */
    ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_PIN     = 3, /*!< Tx attenuation is control with GPIO Incr/Decr: total attenuation is altered incrementally using pin control */
	ADI_ADRV9001_TX_ATTENUATION_CONTROL_MODE_CLGC    = 4, /*!< Tx attenuation : Closed Loop Gain Control : 
	                                                      *    If CLGC is enabled in OBJID_CFG_DPD_PRE_INIT_CAL, limited to TX_ATTENUATION_CONTROL_MODE_CLGC */
} adi_adrv9001_TxAttenuationControlMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SrlStatisticsMode_e` is an enumeration that defines
 * the different modes for collecting and managing statistics related to
 * the slew rate limiter in the ADRV9001 device. It includes options for
 * observing the minimum slew factor, counting the number of samples
 * affected by the slew rate limit, and resetting the statistics to zero.
 * This enumeration is used to configure how the device handles and
 * reports slew rate statistics, which is crucial for performance
 * monitoring and optimization.
 *
 * @param ADI_ADRV9001_SRL_STATISTICS_MIN_SLEW_FACTOR_OBSERVED Represents the
 * minimum slew
 * factor observed
 * in the
 * statistics.
 * @param ADI_ADRV9001_SRL_STATISTICS_NUM_OF_SAMPLES_SLEW_RATE_LIMITED Indicates
 * the
 * number of
 * samples
 * that were
 * limited
 * by the
 * slew
 * rate.
 * @param ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_10 Clears the statistics to
 * zero, equivalent to the
 * value of
 * CLEAR_TO_ZERO_11.
 * @param ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_11 Clears the statistics to
 * zero, equivalent to the
 * value of
 * CLEAR_TO_ZERO_10.
 ******************************************************************************/
typedef enum adi_adrv9001_SrlStatisticsMode
{
    ADI_ADRV9001_SRL_STATISTICS_MIN_SLEW_FACTOR_OBSERVED = 0u,
    ADI_ADRV9001_SRL_STATISTICS_NUM_OF_SAMPLES_SLEW_RATE_LIMITED,
    ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_10,     /* clear statistics to zero, same as b11 below */
    ADI_ADRV9001_SRL_STATISTICS_CLEAR_TO_ZERO_11      /* clear statistics to zero, same as b10 above */
} adi_adrv9001_SrlStatisticsMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SrlTableSel_e` is an enumeration that defines
 * different slew rate limits for the ADRV9001 device. Each enumerator
 * represents a specific percentage of the full scale, allowing the user
 * to select the desired slew rate limit for the device's operation. This
 * configuration is crucial for controlling the rate of change of
 * signals, which can be important for managing signal integrity and
 * performance in various applications.
 *
 * @param ADI_ADRV9001_SRL_TABLE0 Slew limit is 10% of full scale.
 * @param ADI_ADRV9001_SRL_TABLE1 Slew limit is 20% of full scale.
 * @param ADI_ADRV9001_SRL_TABLE2 Slew limit is 30% of full scale.
 * @param ADI_ADRV9001_SRL_TABLE3 Slew limit is 50% of full scale.
 ******************************************************************************/
typedef enum adi_adrv9001_SrlTableSel
{
    ADI_ADRV9001_SRL_TABLE0 = 0x0000,        /*!< Slew limit is 10% of full scale */
    ADI_ADRV9001_SRL_TABLE1 = 0x0001,        /*!< Slew limit is 20% of full scale */
    ADI_ADRV9001_SRL_TABLE2 = 0x0002,        /*!< Slew limit is 30% of full scale */
    ADI_ADRV9001_SRL_TABLE3 = 0x0003         /*!< Slew limit is 50% of full scale */
} adi_adrv9001_SrlTableSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PaProtectionInputSel_e` is an enumeration that
 * defines the source of input data for the PA protection block in the
 * ADRV9001 device. It provides two options for selecting the input data
 * source: either from the complex multiplier output or from the TX QEC
 * actuator output. This selection is crucial for configuring the PA
 * protection mechanism, which helps in safeguarding the power amplifier
 * by monitoring and controlling the input signals.
 *
 * @param ADI_ADRV9001_COMPLEX_MULT_OUTPUT Input data to PA protection block is
 * probed from complex mult output.
 * @param ADI_ADRV9001_TXQEC_ACTUATOR_OUTPUT Input data to PA protection block
 * is probed from tx qec actuator
 * output.
 ******************************************************************************/
typedef enum adi_adrv9001_PaProtectionInputSel
{
    ADI_ADRV9001_COMPLEX_MULT_OUTPUT   = 0x0000, /*!< Input data to PA protection block is probed from complex mult output */
    ADI_ADRV9001_TXQEC_ACTUATOR_OUTPUT = 0x0001  /*!< Input data to PA protection block is probed from tx qec actuator output */
} adi_adrv9001_PaProtectionInputSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxPaRampTrigger_e` is an enumeration that defines
 * the possible trigger sources for the Power Amplifier (PA) ramping
 * process in the ADRV9001 device. It allows the selection of different
 * hardware interfaces (SPI, GPIO, or TX_ENABLE pin) to initiate the PA
 * ramp, providing flexibility in how the ramping process is controlled
 * and integrated into the system's operation.
 *
 * @param ADI_ADRV9001_TX_PA_RAMP_TRIGGER_SPI Trigger for PA Ramp is from SPI.
 * @param ADI_ADRV9001_TX_PA_RAMP_TRIGGER_GPIO Trigger for PA Ramp is from GPIO.
 * @param ADI_ADRV9001_TX_PA_RAMP_TRIGGER_ENABLE_PIN Trigger for PA Ramp is from
 * the TX_ENABLE pin.
 ******************************************************************************/
typedef enum adi_adrv9001_TxPaRampTrigger
{
    ADI_ADRV9001_TX_PA_RAMP_TRIGGER_SPI         = 0x0, /*!< Trigger for PA Ramp is from SPI */
    ADI_ADRV9001_TX_PA_RAMP_TRIGGER_GPIO        = 0x1, /*!< Trigger for PA Ramp is from GPIO */
    ADI_ADRV9001_TX_PA_RAMP_TRIGGER_ENABLE_PIN  = 0x2  /*!< Trigger for PA Ramp is from the TX_ENABLE pin */
} adi_adrv9001_TxPaRampTrigger_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxAttenuationConfig_t` structure is used to
 * configure the transmission attenuation settings for the ADRV9001
 * device. It includes a boolean flag to determine if the power amplifier
 * (PA) should ramp down to maximum attenuation when a PLL unlock is
 * detected, a field to specify the step size for attenuation
 * adjustments, and a field to set the control mode for attenuation,
 * which can be managed through different methods such as SPI or GPIO.
 *
 * @param disableTxOnPllUnlock Indicates if the PA should ramp down to max
 * attenuation on PLL unlock.
 * @param txAttenStepSize Specifies the step size for Tx attenuation.
 * @param attenMode Defines the mode for controlling Tx attenuation.
 ******************************************************************************/
typedef struct adi_adrv9001_TxAttenuationConfig_t
{
    bool disableTxOnPllUnlock;                          /*!< If true, the PA will ramp down to the max attenuation if
                                                         *   an RF1 or RF2 PLL unlock occurs
                                                         *   NOTE: Currently read-only */
    adi_adrv9001_TxAttenStepSize_e txAttenStepSize;	    /*!< Tx Attenuation step size */
    adi_adrv9001_TxAttenuationControlMode_e attenMode;  /*!< The mode to control Tx attenuation */
} adi_adrv9001_TxAttenuationConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxAttenTableRow_t` structure represents a single row
 * in the ADRV9001 device's Tx attenuation table. It contains fields for
 * both digital and analog attenuation settings, allowing for fine-
 * grained control over the transmission signal's attenuation. The
 * `txAttenMult` field specifies the digital attenuation multiplier,
 * while the `txAttenHp` field provides the analog coarse attenuation
 * gain. The `Reserve` field is included for potential future use or to
 * ensure proper memory alignment.
 *
 * @param txAttenMult Digital attenuation multiplier; Valid range: 0-4095.
 * @param txAttenHp Analog coarse attenuation gain; Valid range: 0-63.
 * @param Reserve Reserved for future use or alignment.
 ******************************************************************************/
typedef struct adi_adrv9001_TxAttenTableRow
{
    uint16_t txAttenMult;       /*!< Digital attenuation multiplier; Valid range: 0-4095 */
    uint8_t  txAttenHp;         /*!< Analog coarse attenuation gain; Valid range: 0-63 */
    uint8_t  Reserve;
} adi_adrv9001_TxAttenTableRow_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxPaProtectCfg_t` structure is designed to configure
 * the PA (Power Amplifier) protection settings for the ADRV9001 device.
 * It includes parameters for setting durations and thresholds for
 * average and peak power measurements, as well as enabling or disabling
 * various protection features. Some fields, such as attenuation gain
 * step and ramp settings, are not actively used and are set to hardcoded
 * values by specific APIs. The structure also allows for the selection
 * of the input signal source for the PA protection block, providing
 * flexibility in how the device manages power amplification and
 * protection.
 *
 * @param avgDuration PA Protection Average Power Measurement Duration.
 * @param peakDuration PA Protection Peak Power Measurement Duration.
 * @param txAttenStep PA Protection Attenuation gain step, not actively used.
 * @param gainStepDownEn PA Protection Gain Step Down Enable, not actively used.
 * @param powerThreshold PA Protection Average Power Threshold.
 * @param peakThreshold PA Protection Peak Power Threshold (Valid: 0 to 8191).
 * @param peakCount Peak Count Causing PA Error.
 * @param rampStepDuration PA Protection Ramp Step duration, not actively used.
 * @param rampStepSize PA Protection Ramp Step Size, not actively used.
 * @param rampMaxAtten PA Protection Ramp Max Attenuation, not actively used.
 * @param avgPowerEnable Enables average power measurement block, flags PA error
 * if above threshold.
 * @param peakPowerEnable Enables peak power measurement block, flags PA error
 * if above threshold.
 * @param avgPeakRatioEnable Enables average to peak power ratio calculation
 * block.
 * @param inputSel Selects the source of input signal for PA protection block.
 ******************************************************************************/
typedef struct adi_adrv9001_TxPaProtectCfg
{
    uint8_t avgDuration;                /*!< PA Protection Average Power Measurement Duration. */
    uint8_t peakDuration;               /*!< PA Protection Peak Power Measurement Duration */
    uint8_t txAttenStep;                /*!< PA Protection Attenuation gain step. Gain step down is not allowed for Tokelau device. This field is not being used actively. */
    bool gainStepDownEn;                /*!< PA Protection Gain Step Down Enable. Gain step down is not allowed for Tokelau device. This field is not being used actively.*/
    uint16_t powerThreshold;            /*!< PA Protection Average Power Threshold. */
    uint16_t peakThreshold;             /*!< PA Protection Peak Power Threshold (Valid: 0 to 8191) */
    uint8_t peakCount;                  /*!< Peak Count Causing PA Error. */
    uint8_t rampStepDuration;           /*!< PA Protection Ramp Step duration. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    uint8_t rampStepSize;               /*!< PA Protection Ramp Step Size. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    uint8_t rampMaxAtten;               /*!< PA Protection Ramp Max Attenuation. This field is not being used actively. It is set to a hardcoded value by PaPllDfrmEventRampDownEnableSet API */
    bool avgPowerEnable;                /*!< This enables average power measurement block. If enabled, PA error is flagged when average power measurement is above average power threshold */
    bool peakPowerEnable;               /*!< This enables peak power measurement block. If enabled, PA error is flagged when peak power count is above peak count threshold */
    bool avgPeakRatioEnable; /*!< This enables average to peak power ratio calculation block, both avgPower and peakPower calculations must be enabled before enabling ratio calculation */
    adi_adrv9001_PaProtectionInputSel_e inputSel; /*!< This selects the source of input signal for Pa protection block */
} adi_adrv9001_TxPaProtectCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SlewRateLimiterCfg_t` structure is used to configure
 * the slew rate limiter settings for the ADRV9001 device. It includes
 * options to enable or disable the limiter and its statistics
 * collection, select a slew rate table, set a slew offset, and choose a
 * statistics mode. This configuration allows for fine-tuning of the slew
 * rate limiting behavior to optimize performance in various
 * applications.
 *
 * @param srlEnable Indicates whether the slew rate limiter is enabled,
 * defaulting to true.
 * @param srlStatisticsEnable Indicates whether statistics collection for the
 * slew rate limiter is enabled, defaulting to true.
 * @param srlTableSelect Specifies the selected slew rate table, defaulting to
 * adi_adrv9001_srl_table3.
 * @param srlSlewOffset Defines the slew offset value, ranging from 0 to 15,
 * with a default of 0.
 * @param srlStatisticsMode Specifies the mode for slew rate limiter statistics,
 * defaulting to
 * adi_adrv9001_srl_statistics_clear_to_zero.
 ******************************************************************************/
typedef struct adi_adrv9001_SlewRateLimiterCfg
{
    bool srlEnable;                                         /*!< default = true */
    bool srlStatisticsEnable;                               /*!< default = true */
    adi_adrv9001_SrlTableSel_e srlTableSelect;              /*!< default = adi_adrv9001_srl_table3 */
    uint8_t srlSlewOffset;                                  /*!< 0 (default), 1, . . ., 15 */
    adi_adrv9001_SrlStatisticsMode_e srlStatisticsMode;     /*!< default = adi_adrv9001_srl_statistics_clear_to_zero */
} adi_adrv9001_SlewRateLimiterCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PaRampCfg_t` structure is used to configure the
 * Power Amplifier (PA) ramp settings for a transmission channel in the
 * ADRV9001 device. It includes parameters to enable or disable the PA
 * ramp, select the trigger source, set the clock rate, and configure
 * delays for the rising and falling edges of the trigger. The structure
 * also allows for the selection of a GPIO pin as a trigger source,
 * configuration of the ramp waveform's symmetry, and selection of an
 * auxiliary DAC channel for output. Additionally, it contains a look-up
 * table for defining the PA ramp waveform.
 *
 * @param enable PA Ramp enable; True = Enable PA ramp of Tx channel; False =
 * Disable.
 * @param triggerSelect Trigger source for the PA ramp.
 * @param rampClock_kHz Clock rate of the Tx PA Ramp block in kHz.
 * @param triggerDelayRise Programmable delay on the rising edge of the trigger.
 * @param triggerDelayFall Programmable delay on the falling edge of the
 * trigger.
 * @param gpioSource GPIO pin used as the source of the trigger if gpioTriggered
 * is True.
 * @param upEndIndex 8-bit look-up-table index indicating the end of the ramp-up
 * waveform.
 * @param asymmetricRamp Indicates if the ramp-down waveform is asymmetric to
 * the ramp-up waveform.
 * @param downEndIndex 8-bit look-up-table index indicating the end of the ramp-
 * down waveform, valid only when asymmetricRamp is True.
 * @param auxDacChannelSelect Selects the AuxDacChannel to output the Ramp or
 * SPI signal.
 * @param paRampLUT PA Ramp look-up-table with a depth of 256 elements.
 ******************************************************************************/
typedef struct adi_adrv9001_PaRampCfg
{
    bool     enable;                                   /*!< PA Ramp enable; True = Enable PA ramp of Tx channel; False = Disable */
    adi_adrv9001_TxPaRampTrigger_e triggerSelect;      /*!< Trigger source */
    uint16_t rampClock_kHz;                            /*!< Clock rate of Tx PA Ramp block */
    uint32_t triggerDelayRise;                         /*!< Programmable Delay Enable on the rising edge */
    uint32_t triggerDelayFall;                         /*!< Programmable Delay Enable on the falling edge */
    adi_adrv9001_GpioPin_e     gpioSource;             /*!< Desired GPIO pin to be used as source of trigger if gpioTriggered = True */
    uint8_t  upEndIndex;                               /*!< 8-bit look-up-table index. This index indicates the end of the ramp up waveform. */
    bool     asymmetricRamp;                           /*!< False = symmetric, True = Ramp-down waveform is asymmetric to the Ramp-up waveform */
    uint8_t  downEndIndex;                             /*!< 8-bit look-up-table index. This index indicates the end of the ramp down waveform.
                                                          Valid only when asymmetricRamp=1 */
    adi_adrv9001_AuxDac_e  auxDacChannelSelect;        /*!< Choose the AuxDacChannel [0, 1, 2, 3] to ouptut the Ramp or SPI signal */
    uint16_t paRampLUT[ADRV9001_TX_PA_RAMP_LUT_SIZE];  /*!< PA Ramp look-up-table. 256 depth Array of LUT elements */
} adi_adrv9001_PaRampCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxAttenuationPinControlCfg_t` structure is designed
 * to manage the transmission attenuation control in an ADRV9001 device
 * using GPIO pins. It allows for incremental or decremental adjustments
 * to the transmission attenuation based on rising edges detected on
 * specified GPIO pins. The structure includes a step size in milli-
 * decibels (mdB) that determines the magnitude of each adjustment, and
 * it ensures that the attenuation does not exceed predefined limits,
 * providing a controlled and precise method for managing signal
 * strength.
 *
 * @param stepSize_mdB Step size of change in txAttenuation when a rising edge
 * occurs on either incrementPin or decrementPin, ranging
 * from 0 mdB to 1550 mdB with an LSB of 50 mdB.
 * @param incrementPin GPIO pin that, when a rising edge occurs, increments
 * txAttenuation by stepSize_mdB until a maximum index of
 * 839 (2094 mdB) is reached.
 * @param decrementPin GPIO pin that, when a rising edge occurs, decrements
 * txAttenuation by stepSize_mdB until a minimum index of 0
 * is reached.
 ******************************************************************************/
typedef struct adi_adrv9001_TxAttenuationPinControlCfg
{
    uint16_t stepSize_mdB;                  /*!< Step size of change in txAttenuation when a rising edge occurs on on either incrementPin or decrementPin. Range: 0 mdB to 1550 mdB, LSB =50 mdB */
    adi_adrv9001_GpioPin_e incrementPin;    /*!< When a rising edge occurs on this GPIO pin, txAttenuation will increment by stepSize_mdB .
                                                 Once txAttenuation has reached index 839(2094 mdB) subsequent rising edges on incrementPin will not change the txAttenuation */
    adi_adrv9001_GpioPin_e decrementPin;    /*!< When a rising edge occurs on this GPIO pin, txAttenuation will decrement by stepSize_mdB .
                                                 Once txAttenuation has reached index 0 subsequent rising edges on decrementPin will not change the txAttenuation */
} adi_adrv9001_TxAttenuationPinControlCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxInternalToneAmplitude_e` is an enumeration that
 * defines the possible amplitude levels for the internal tone generated
 * by the NCO (Numerically Controlled Oscillator) in the ADRV9001 device.
 * It provides three discrete amplitude settings: 0 dB, 6 dB, and 12 dB,
 * allowing for control over the signal strength of the generated tone.
 *
 * @param ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_0_DB Represents a tone amplitude
 * of 0 dB.
 * @param ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_6_DB Represents a tone amplitude
 * of 6 dB.
 * @param ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_12_DB Represents a tone amplitude
 * of 12 dB.
 ******************************************************************************/
typedef enum adi_adrv9001_TxInternalToneAmplitude
{
    ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_0_DB,
    ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_6_DB,
    ADI_ADRV9001_TXINTERNALTONEAMPLITUDE_12_DB
} adi_adrv9001_TxInternalToneAmplitude_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxInternalToneGeneration_t` structure is used to
 * configure the internal tone generation settings for the ADRV9001
 * device's transmitter. It includes fields to enable or disable the tone
 * generation, set the amplitude of the tone using an enumerated type,
 * and specify the frequency of the tone in Hertz. This structure is
 * essential for controlling the NCO (Numerically Controlled Oscillator)
 * tone generation, which is a critical feature for testing and
 * calibration purposes in RF systems.
 *
 * @param enable Whether to enable NCO tone generation.
 * @param amplitude Amplitude of the tone.
 * @param frequency_Hz Frequency of the tone, in Hertz (valid: +/-
 * tx_interface_rate).
 ******************************************************************************/
typedef struct adi_adrv9001_TxInternalToneGeneration
{
    bool enable;                                        /*!< Whether to enable NCO tone generation */
    adi_adrv9001_TxInternalToneAmplitude_e amplitude;   /*!< Amplitude of the tone */
    int32_t frequency_Hz;                               /*!< Frequency of the tone, in Hertz (valid: +/- tx_interface_rate) */
} adi_adrv9001_TxInternalToneGeneration_t;

#endif /* _ADI_ADRV9001_TX_TYPES_H_ */
