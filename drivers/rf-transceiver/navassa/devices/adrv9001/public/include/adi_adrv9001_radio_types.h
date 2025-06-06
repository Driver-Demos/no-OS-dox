/**
 * \file
 * \brief Contains ADRV9001 API Radio Control data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_RADIO_TYPES_H_
#define _ADI_ADRV9001_RADIO_TYPES_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_gpio_types.h"
#include "adi_adrv9001_powersavingandmonitormode_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_Pll_e` is an enumeration that defines various Phase-
 * Locked Loop (PLL) selections available in the ADRV9001 radio control
 * API. Each enumerator represents a specific PLL configuration option,
 * such as LO1, LO2, AUX, CLK, and CLK_LP, which are used to control the
 * frequency synthesis for different radio functions like reception (Rx)
 * and transmission (Tx). This enumeration is crucial for configuring the
 * PLL settings in the ADRV9001 device, allowing for precise control over
 * the radio frequency operations.
 *
 * @param ADI_ADRV9001_PLL_LO1 Selects PLL LO1 for Rx and Tx.
 * @param ADI_ADRV9001_PLL_LO2 Selects PLL LO2 for Rx and Tx.
 * @param ADI_ADRV9001_PLL_AUX Selects AUX PLL for Rx and Tx.
 * @param ADI_ADRV9001_PLL_CLK Represents a clock PLL selection.
 * @param ADI_ADRV9001_PLL_CLK_LP Represents a low power clock PLL selection.
 ******************************************************************************/
typedef enum adi_adrv9001_Pll
{
    ADI_ADRV9001_PLL_LO1 = 0, /*!< Selects PLL LO1 for Rx and Tx */
    ADI_ADRV9001_PLL_LO2,     /*!< Selects PLL LO2 for Rx and Tx */
    ADI_ADRV9001_PLL_AUX,     /*!< Selects AUX PLL for Rx and tx*/
    ADI_ADRV9001_PLL_CLK,
    ADI_ADRV9001_PLL_CLK_LP
} adi_adrv9001_Pll_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllCalibration_e` is an enumeration that defines the
 * different modes of PLL (Phase-Locked Loop) calibration available in
 * the ADRV9001 API. It provides options for normal and fast calibration
 * settings, as well as a reserved option for potential future use or
 * undefined settings. This enumeration is used to configure the PLL
 * calibration mode in the ADRV9001 radio control API, allowing for
 * adjustments in calibration speed and accuracy based on application
 * requirements.
 *
 * @param ADI_ADRV9001_PLL_CALIBRATION_NORMAL PLL calibration setting in Normal
 * mode.
 * @param ADI_ADRV9001_PLL_CALIBRATION_FAST PLL calibration setting in Fast
 * mode.
 * @param ADI_ADRV9001_PLL_CALIBRATION_RESERVED Reserved for future use or
 * undefined settings.
 ******************************************************************************/
typedef enum adi_adrv9001_PllCalibration
{
    ADI_ADRV9001_PLL_CALIBRATION_NORMAL,    /*!< PLL calibration setting in Normal mode */
    ADI_ADRV9001_PLL_CALIBRATION_FAST,      /*!< PLL calibration setting in Fast mode  */
    ADI_ADRV9001_PLL_CALIBRATION_RESERVED   /*!< RESERVED */
} adi_adrv9001_PllCalibration_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_LoGenOptimization_e` is an enumeration that defines
 * the modes for Local Oscillator (LO) generation optimization in the
 * ADRV9001 radio control API. It provides two options: one that
 * prioritizes reducing phase noise, which may result in higher power
 * consumption, and another that focuses on minimizing power consumption,
 * potentially increasing phase noise. This allows users to select the
 * optimization strategy that best suits their application requirements.
 *
 * @param ADI_ADRV9001_LO_GEN_OPTIMIZATION_PHASE_NOISE Reduce phase noise at the
 * cost of increased power
 * consumption.
 * @param ADI_ADRV9001_LO_GEN_OPTIMIZATION_POWER_CONSUMPTION Reduce power
 * consumption at the
 * cost of increased
 * phase noise.
 ******************************************************************************/
typedef enum adi_adrv9001_LoGenOptimization
{
    ADI_ADRV9001_LO_GEN_OPTIMIZATION_PHASE_NOISE,       // Reduce phase noise at the cost of increased power consumption
    ADI_ADRV9001_LO_GEN_OPTIMIZATION_POWER_CONSUMPTION  // Reduce power consumption at the cost of increased phase noise
} adi_adrv9001_LoGenOptimization_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllPower_e` is an enumeration that defines different
 * power levels for the Phase-Locked Loop (PLL) in the ADRV9001 radio
 * control API. It provides three distinct power settings: low, medium,
 * and high, allowing for flexible power management and optimization of
 * the PLL's performance based on the application's requirements.
 *
 * @param ADI_ADRV9001_PLL_POWER_LOW Represents a low power level setting for
 * the PLL.
 * @param ADI_ADRV9001_PLL_POWER_MEDIUM Represents a medium power level setting
 * for the PLL.
 * @param ADI_ADRV9001_PLL_POWER_HIGH Represents a high power level setting for
 * the PLL.
 ******************************************************************************/
typedef enum adi_adrv9001_PllPower
{
    ADI_ADRV9001_PLL_POWER_LOW,
    ADI_ADRV9001_PLL_POWER_MEDIUM,
    ADI_ADRV9001_PLL_POWER_HIGH
} adi_adrv9001_PllPower_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Carrier_t` structure is used to configure carrier
 * frequency settings for the ADRV9001 radio. It includes parameters for
 * LO Gen optimization, carrier frequency, intermediate frequency, and
 * manual Rx port selection. The structure allows for precise control
 * over the radio's frequency settings, ensuring optimal performance
 * across a wide range of frequencies from 30 MHz to 6 GHz. The LO Gen
 * optimization mode can be automatically selected by the ADRV9001 when
 * the carrier frequency exceeds 1 GHz, providing flexibility and ease of
 * use in high-frequency applications.
 *
 * @param loGenOptimization Specifies the LO Gen optimization mode, which can be
 * automatically selected based on carrier frequency.
 * @param carrierFrequency_Hz Defines the carrier frequency in Hertz, with a
 * valid range from 30 MHz to 6 GHz.
 * @param intermediateFrequency_Hz Specifies the intermediate frequency in
 * Hertz, with a valid range between +/- min(20
 * MHz, rfChannelBandwidth_Hz /2)).
 * @param manualRxport Indicates the port selection for manually switching the
 * Rx port.
 ******************************************************************************/
typedef struct adi_adrv9001_Carrier
{
    /** LO Gen optimization mode
     * \note When carrierFrequency_Hz > 1 GHz this is automatically selected by the ADRV9001 regardless of what you set
     */
    adi_adrv9001_LoGenOptimization_e loGenOptimization;
    uint64_t carrierFrequency_Hz;       /*!< Carrier frequency, denoted in Hz. Valid range: 30MHz to 6 GHz*/
    int32_t intermediateFrequency_Hz;   /*!< Intermediate frequency, denoted in Hz.
                                             Valid range: between +/- min(20 MHz, rfChannelBandwidth_Hz /2)) */
    adi_adrv9001_RxRfInputSel_e manualRxport; /*!< Port selection to manually switch Rx port */
} adi_adrv9001_Carrier_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllConfig_t` structure is used to configure the
 * phase-locked loop (PLL) settings for the ADRV9001 device. It includes
 * parameters for selecting the calibration mode and power level of the
 * PLL, allowing for customization of the PLL's performance
 * characteristics. This configuration is crucial for optimizing the
 * PLL's operation in various radio frequency applications.
 *
 * @param pllCalibration Specifies the calibration mode for the PLL, using the
 * adi_adrv9001_PllCalibration_e enumeration.
 * @param pllPower Defines the power level for the PLL, using the
 * adi_adrv9001_PllPower_e enumeration.
 ******************************************************************************/
typedef struct adi_adrv9001_PllConfig
{
    adi_adrv9001_PllCalibration_e pllCalibration;
    adi_adrv9001_PllPower_e pllPower;

} adi_adrv9001_PllConfig_t;


/***************************************************************************//**
 * @brief The `adi_adrv9001_ChannelEnableMode_e` is an enumeration that defines
 * the modes for enabling a channel in the ADRV9001 radio system. It
 * provides two options: enabling the channel through the Serial
 * Peripheral Interface (SPI) or through a General Purpose Input/Output
 * (GPIO) pin. This allows for flexible control of channel activation
 * depending on the system's hardware configuration and requirements.
 *
 * @param ADI_ADRV9001_SPI_MODE Channel is enabled via SPI.
 * @param ADI_ADRV9001_PIN_MODE Channel is enabled via GPIO pin.
 ******************************************************************************/
typedef enum adi_adrv9001_ChannelEnableMode
{
    ADI_ADRV9001_SPI_MODE = 0,      // Channel is enabled via SPI
    ADI_ADRV9001_PIN_MODE           // Channel is enabled via GPIO pin
} adi_adrv9001_ChannelEnableMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ChannelState_e` is an enumeration that defines the
 * various operational states of a channel in the ADRV9001 radio system.
 * It includes states such as STANDBY, where the channel is initialized
 * post-boot; CALIBRATED, indicating successful initial calibration with
 * data paths off; PRIMED, where data paths are active but not
 * transmitting; and RF_ENABLED, where the channel is fully operational
 * with data transmission and reception active. This enumeration is
 * crucial for managing the state transitions of radio channels in the
 * ADRV9001 system.
 *
 * @param ADI_ADRV9001_CHANNEL_STANDBY Initial state for all channels once ARM
 * completes boot sequences.
 * @param ADI_ADRV9001_CHANNEL_CALIBRATED Minimum set of initial calibration
 * done without errors; CLK PLL is on;
 * data paths are off.
 * @param ADI_ADRV9001_CHANNEL_PRIMED Data path and interface are on; tracking
 * algorithms NOT scheduled, TRx NOT
 * transmitting or receiving data.
 * @param ADI_ADRV9001_CHANNEL_RF_ENABLED Data path and interface are on;
 * tracking algorithms are scheduled, TRx
 * is transmitting or receiving data.
 ******************************************************************************/
typedef enum adi_adrv9001_ChannelState
{
    ADI_ADRV9001_CHANNEL_STANDBY,                /*!< Initial state for all channels once ARM completes boot sequences */
    ADI_ADRV9001_CHANNEL_CALIBRATED,             /*!< Minimum set of initial calibration done without errors; CLK PLL is on; data paths are off */
    ADI_ADRV9001_CHANNEL_PRIMED,                 /*!< Data path and interface are on; tracking algorithms NOT scheduled, TRx NOT transmitting or receiving data */
    ADI_ADRV9001_CHANNEL_RF_ENABLED,             /*!< Data path and interface are on; tracking algorithms are scheduled, TRx is transmitting or receiving data */
} adi_adrv9001_ChannelState_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RadioState_t` structure is designed to encapsulate
 * the state information of the ADRV9001 radio system. It includes fields
 * to track the overall system state, monitor mode state, MCS state, and
 * boot state, which are essential for understanding the current
 * operational status of the ARM processor within the radio.
 * Additionally, it maintains an array of channel states, allowing for
 * detailed tracking of each channel's status across multiple ports and
 * channels, facilitating efficient management and control of the radio's
 * operational modes.
 *
 * @param systemState Represents the current state of the ARM system.
 * @param monitorModeState Indicates the ARM Monitor Mode State, valid only when
 * the system is in Monitor Mode.
 * @param mcsState Denotes the ARM MCS state, valid only when the system is in
 * MCS mode.
 * @param bootState Specifies the ARM Boot State, valid when the system is in
 * POWERUP state and not in MCS state.
 * @param channelStates An array representing the state of each channel, indexed
 * by port and channel.
 ******************************************************************************/
typedef struct adi_adrv9001_RadioState
{
    adi_adrv9001_ArmSystemStates_e                                systemState;        /*!< ARM System State */
    adi_adrv9001_PowerSavingAndMonitorMode_ArmMonitorModeStates_e monitorModeState;   /*!< ARM Monitor Mode State (only valid when systemState is in Monitor Mode) */
    adi_adrv9001_ArmMcsStates_e                                   mcsState;           /*!< ARM MCS state (only valid when systemState is MCS) */
    adi_adrv9001_ArmBootStates_e                                  bootState;          /*!< ARM Boot State (only valid when systemState is in POWERUP state and not in MCS state) */
    /**
     * State of each channel; Use adi_common_port_to_index and adi_common_channel_to_index to access conveniently
     * e.g., RX1 is channelStates[adi_common_port_to_index(ADI_RX)][adi_common_channel_to_index(ADI_CHANNEL_1)]
     */
    adi_adrv9001_ChannelState_e channelStates[ADI_ADRV9001_NUM_PORTS][ADI_ADRV9001_NUM_CHANNELS];
} adi_adrv9001_RadioState_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PllLoopFilterCfg_t` structure is designed to hold
 * configuration settings for the Synthesizer PLL Loop filter in the
 * ADRV9001 radio system. It includes parameters for the effective loop
 * bandwidth, loop bandwidth, phase margin, and power scale, which are
 * critical for tuning the PLL's performance in terms of stability and
 * response time. The effective loop bandwidth is read-only, while the
 * other parameters can be adjusted within specified ranges to optimize
 * the PLL's operation.
 *
 * @param effectiveLoopBandwidth_kHz Synthesizer PLL effective Loop filter
 * bandwidth, read-only with range TBD.
 * @param loopBandwidth_kHz Synthesizer PLL Loop filter bandwidth with a range
 * of 50-1500 kHz.
 * @param phaseMargin_degrees Synthesizer PLL Loop filter phase margin in
 * degrees, ranging from 40 to 85.
 * @param powerScale Synthesizer PLL Loop filter power scale, ranging from 0 to
 * 10, with a default of 10.
 ******************************************************************************/
typedef struct adi_adrv9001_PllLoopFilterCfg
{
    uint16_t effectiveLoopBandwidth_kHz; /*!< Synthesizer PLL effective Loop filter bandwidth. Range TBD. For read-only */
    uint16_t loopBandwidth_kHz; /*!< Synthesizer PLL Loop filter bandwidth. Range 50-1500 */
    uint8_t  phaseMargin_degrees; /*< Synthesizer PLL Loop filter phase margin in degrees. Range 40-85 */
    uint8_t  powerScale; /*!< Synthesizer PLL Loop filter power scale. Range 0 - 10. Default is 10 */
} adi_adrv9001_PllLoopFilterCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ChannelEnablementDelays_t` structure defines various
 * delay parameters associated with the enabling and disabling of
 * channels in the ADRV9001 radio system. Each delay is measured in ARM
 * clock cycles and has a valid range from 0 to 2^24 - 1. These delays
 * are crucial for synchronizing the power-up and power-down sequences of
 * the Low Noise Amplifier (LNA) and the Tx/Rx interfaces, ensuring
 * proper timing for data transmission and reception.
 *
 * @param riseToOnDelay Delay from rising edge until LNA is powered up.
 * @param riseToAnalogOnDelay Delay from rising edge until Tx/Rx analog power up
 * procedure commences.
 * @param fallToOffDelay Delay from falling edge until LNA is powered down.
 * @param guardDelay Delay from rising edge until first valid data is received
 * over the air.
 * @param holdDelay Delay from falling edge until the Tx/Rx interface is
 * disabled.
 ******************************************************************************/
typedef struct adi_adrv9001_ChannelEnablementDelays
{
    uint32_t riseToOnDelay;         /*!< Delay from rising  edge until LNA is powered up */
    uint32_t riseToAnalogOnDelay;   /*!< Delay from rising  edge until Tx/Rx analog power up procedure commences */
    uint32_t fallToOffDelay;        /*!< Delay from falling edge until LNA is powered down */
    uint32_t guardDelay;            /*!< Delay from rising  edge until first valid data is received over the air */
    uint32_t holdDelay;             /*!< Delay from falling edge until the Tx/Rx interface is disabled */
} adi_adrv9001_ChannelEnablementDelays_t;

#endif /* _ADI_ADRV9001_RADIO_TYPES_H_ */
