/**
* \file
* \brief Contains ADRV9001 API Rx datapath data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_RX_TYPES_H_
#define _ADI_ADRV9001_RX_TYPES_H_

#include "adi_adrv9001_gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

#define ADI_ADRV9001_RX_GAIN_INDEX_MIN 187
#define ADI_ADRV9001_RX_GAIN_INDEX_MAX 255
#define RX1_INTERFACE_GAIN_SEED_ADDR            0x2001FFD0
#define RX2_INTERFACE_GAIN_SEED_ADDR            0x2001FFD4
#define RX1_INTERFACE_GAIN_END_OF_FRAME_ADDR    0x2001FFD8
#define RX2_INTERFACE_GAIN_END_OF_FRAME_ADDR    0x2001FFDC

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainTableBaseAddr_e` is an enumeration that
 * defines the base addresses for the gain table SRAM for the ADRV9001
 * device's Rx channels. It provides two specific addresses, one for each
 * of the two Rx channels, allowing the software to select the
 * appropriate memory location for gain table operations. This is crucial
 * for configuring and managing the gain settings of the device's receive
 * path.
 *
 * @param ADI_ADRV9001_RX_GAIN_TABLE_BASE_ADDR_1 Select gain table SRAM base
 * address for Rx1 Channel.
 * @param ADI_ADRV9001_RX_GAIN_TABLE_BASE_ADDR_2 Select gain table SRAM base
 * address for Rx2 Channel.
 ******************************************************************************/
typedef enum adi_adrv9001_RxGainTableBaseAddr_e
{
    ADI_ADRV9001_RX_GAIN_TABLE_BASE_ADDR_1 = 0x73300000,  /*!< Select gain table SRAM base address for Rx1 Channel */
    ADI_ADRV9001_RX_GAIN_TABLE_BASE_ADDR_2 = 0x73400000,  /*!< Select gain table SRAM base address for Rx2 Channel */
} adi_adrv9001_RxGainTableBaseAddr_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainControlMode_e` is an enumeration that defines
 * the different modes of controlling the gain index for the ADRV9001
 * receiver. It provides three modes: SPI mode, where the gain index is
 * explicitly set via SPI; PIN mode, where the gain index is adjusted by
 * pulsing GPIO pins; and AUTO mode, where the gain index is
 * automatically set by the ADRV9001 Gain Control block based on
 * predefined thresholds. This enumeration allows for flexible gain
 * control configurations depending on the application requirements.
 *
 * @param ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI Explicitly set the gain index
 * via SPI.
 * @param ADI_ADRV9001_RX_GAIN_CONTROL_MODE_PIN Increment and decrement the gain
 * index by pulsing GPIO pins.
 * @param ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO ADRV9001 Gain Control block
 * sets the gain index based on
 * thresholds.
 ******************************************************************************/
typedef enum adi_adrv9001_RxGainControlMode
{
    ADI_ADRV9001_RX_GAIN_CONTROL_MODE_SPI,   /*!< Explicitly set the gain index via SPI */
    ADI_ADRV9001_RX_GAIN_CONTROL_MODE_PIN,   /*!< Increment and decrement the gain index by pulsing GPIO pins */
    ADI_ADRV9001_RX_GAIN_CONTROL_MODE_AUTO   /*!< ADRV9001 Gain Control block sets the gain index based on thresholds */
} adi_adrv9001_RxGainControlMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainTableRow_t` structure is used to represent a
 * row in the ADRV9001 receiver gain table. It contains fields for
 * configuring the front-end gain, external LNA control, ADC and TIA
 * gain, digital gain, and phase offset. This structure is essential for
 * managing the gain settings of the ADRV9001 receiver, allowing for
 * precise control over the signal amplification and phase adjustments
 * necessary for optimal performance in various radio frequency
 * applications.
 *
 * @param rxFeGain Rx Front End gain for a given gain index.
 * @param extControl External LNA control word.
 * @param adcTiaGain ADC and TIA control for a given gain index.
 * @param digGain Digital gain ranging from -18dB to 50dB (68dB total range).
 * @param phaseOffset 16 bit phase offset from 0 - 2pi in resolution of 0.005
 * degrees.
 ******************************************************************************/
typedef struct adi_adrv9001_RxGainTableRow
{
    uint8_t rxFeGain;     /*!< Rx Front End gain for a given gain index */
    uint8_t extControl;   /*!< External LNA control word */
    uint8_t adcTiaGain;   /*!< ADC and TIA control for a given gain index */
    int16_t digGain;      /*!< Digital gain ranging from -18dB to 50dB (68dB total range) */
    uint16_t phaseOffset; /*!< 16 bit phase offset from 0 - 2pi in resolution of 0.005 degrees */
} adi_adrv9001_RxGainTableRow_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxInterfaceGainCtrlMode_e` is an enumeration that
 * defines the modes of controlling the Rx interface gain in the ADRV9001
 * device. It provides two modes: automatic, where the internal Rx
 * interface gain value is used, and manual, where an external gain value
 * must be supplied. This allows for flexibility in managing the gain
 * control based on the specific requirements of the application.
 *
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC Use internal Rx
 * interface gain value.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL Use external Rx
 * interface gain value,
 * which must be provided.
 ******************************************************************************/
typedef enum adi_adrv9001_RxInterfaceGainCtrlMode
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_AUTOMATIC = 0x00,
    /*!< Use internal Rx interface gain value. */
    ADI_ADRV9001_RX_INTERFACE_GAIN_CONTROL_MANUAL = 0x01,
    /*!< Use external Rx interface gain value. Gain value has to be provided in this case. */
} adi_adrv9001_RxInterfaceGainCtrlMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxInterfaceGain_e` is an enumeration that defines
 * various discrete gain levels for the receive interface of the ADRV9001
 * device. These gain levels range from +18 dB to -36 dB, allowing for
 * precise control over the signal amplification or attenuation in the
 * receive path. This enumeration is used to set the desired gain level
 * in the device's configuration, enabling optimal signal processing
 * based on the application's requirements.
 *
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB Represents a gain of 18 dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_12_DB Represents a gain of 12 dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_6_DB Represents a gain of 6 dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB Represents a gain of 0 dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_6_DB Represents a gain of -6
 * dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_12_DB Represents a gain of -12
 * dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_18_DB Represents a gain of -18
 * dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_24_DB Represents a gain of -24
 * dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_30_DB Represents a gain of -30
 * dB.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_36_DB Represents a gain of -36
 * dB.
 ******************************************************************************/
typedef enum adi_adrv9001_RxInterfaceGain
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_18_DB = 0,       /*!<  18 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_12_DB,           /*!<  12 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_6_DB,            /*!<   6 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_0_DB,            /*!<   0 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_6_DB,   /*!<  -6 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_12_DB,  /*!< -12 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_18_DB,  /*!< -18 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_24_DB,  /*!< -24 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_30_DB,  /*!< -30 dB */
    ADI_ADRV9001_RX_INTERFACE_GAIN_NEGATIVE_36_DB,  /*!< -36 dB */
} adi_adrv9001_RxInterfaceGain_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxManualGainPeakSignalSel_e` is an enumeration that
 * defines various conditions related to manual gain peak signal
 * selection in the ADRV9001 receiver. Each enumerator represents a
 * specific condition where a counter related to gain control override is
 * exceeded, which is used in configuring the feedback mask for gain
 * control in the ADRV9001 device. This enumeration is part of the
 * broader ADRV9001 API, which is used to manage and control the
 * receiver's gain settings.
 *
 * @param ADI_ADRV9001_RX_GAINCONTROL_OVRG_LOW_COUNTER_EXCEEDED Represents a
 * condition where
 * the low counter
 * for gain control
 * override is
 * exceeded.
 * @param ADI_ADRV9001_RX_GAINCONTROL_OVRG_LLB_COUNTER_EXCEEDED Indicates that
 * the lower limit
 * band counter for
 * gain control
 * override is
 * exceeded.
 * @param ADI_ADRV9001_RX_GAINCONTROL_OVRG_HIGH_COUNTER_EXCEEDED Denotes a
 * situation where
 * the high
 * counter for
 * gain control
 * override is
 * exceeded.
 * @param ADI_ADRV9001_RX_GAINCONTROL_OVRG_ULB_COUNTER_EXCEEDED_0 Signifies that
 * the upper
 * limit band
 * counter for
 * gain control
 * override is
 * exceeded.
 ******************************************************************************/
typedef enum adi_adrv9001_RxManualGainPeakSignalSel
{
    ADI_ADRV9001_RX_GAINCONTROL_OVRG_LOW_COUNTER_EXCEEDED    = 0x0001,
    ADI_ADRV9001_RX_GAINCONTROL_OVRG_LLB_COUNTER_EXCEEDED    = 0x0002,
    ADI_ADRV9001_RX_GAINCONTROL_OVRG_HIGH_COUNTER_EXCEEDED   = 0x0004,
    ADI_ADRV9001_RX_GAINCONTROL_OVRG_ULB_COUNTER_EXCEEDED_0  = 0x0008,
} adi_adrv9001_RxManualGainPeakSignalSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxManualGainPeakPowerSignalSel_e` is an enumeration
 * that defines various manual gain peak power signal selection options
 * for the ADRV9001 Rx gain control feedback configuration. Each
 * enumerator represents a specific condition or event related to gain
 * control, such as gain change events or threshold exceedances, which
 * can be used to configure the feedback mask in the gain control
 * feedback configuration structure.
 *
 * @param ADI_ADRV9001_RX_GT_GAIN_CHANGE_0 Represents a gain change event with a
 * value of 0x0001.
 * @param ADI_ADRV9001_RX_GAINCONTROL_SL_LOW_TH_EXCEEDED Indicates that the
 * signal level low
 * threshold has been
 * exceeded with a value
 * of 0x0002.
 * @param ADI_ADRV9001_RX_GAINCONTROL_SL_HIGH_TH_EXCEEDED Indicates that the
 * signal level high
 * threshold has been
 * exceeded with a value
 * of 0x0004.
 * @param ADI_ADRV9001_RX_GAINCONTROL_OVRG_ULB_COUNTER_EXCEEDED_1 Represents an
 * overrange
 * upper limit
 * counter
 * exceeded event
 * with a value
 * of 0x0008.
 ******************************************************************************/
typedef enum adi_adrv9001_rxManualGainPeakPowerSignalSel
{
    ADI_ADRV9001_RX_GT_GAIN_CHANGE_0                 = 0x0001,
    ADI_ADRV9001_RX_GAINCONTROL_SL_LOW_TH_EXCEEDED           = 0x0002,
    ADI_ADRV9001_RX_GAINCONTROL_SL_HIGH_TH_EXCEEDED          = 0x0004,
    ADI_ADRV9001_RX_GAINCONTROL_OVRG_ULB_COUNTER_EXCEEDED_1  = 0x0008,
} adi_adrv9001_RxManualGainPeakPowerSignalSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxInterfaceGainUpdateTiming_e` is an enumeration
 * that defines the timing for updating the Rx interface gain control in
 * the ADRV9001 device. It provides two options: updating the gain at the
 * start of the next frame or updating it immediately. This allows for
 * precise control over when gain adjustments are applied, which can be
 * critical in applications requiring synchronized or immediate gain
 * changes.
 *
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_UPDATE_TIMING_NEXT_FRAME Update Rx
 * interface gain
 * control at the
 * start of the
 * next frame.
 * @param ADI_ADRV9001_RX_INTERFACE_GAIN_UPDATE_TIMING_NOW Update Rx interface
 * gain control
 * immediately.
 ******************************************************************************/
    typedef enum adi_adrv9001_RxInterfaceGainUpdateTiming
{
    ADI_ADRV9001_RX_INTERFACE_GAIN_UPDATE_TIMING_NEXT_FRAME = 0x00,
    /*!< Update Rx interface gain control at start of next frame */
    ADI_ADRV9001_RX_INTERFACE_GAIN_UPDATE_TIMING_NOW = 0x01,
    /*!< Update Rx interface gain control immediately  */
} adi_adrv9001_RxInterfaceGainUpdateTiming_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxInterfaceGainCtrl_t` structure is used to
 * configure the Rx interface gain control parameters for a given Rx
 * channel in the ADRV9001 device. It includes settings for the timing
 * and mode of gain updates, the specific gain value, and parameters
 * related to RSSI measurement and signal characteristics. The structure
 * also allows for enabling or disabling fast attack mode, which
 * influences the behavior of the moving average filter during gain
 * control operations.
 *
 * @param updateInstance Specifies the timing for updating the Rx interface gain
 * control.
 * @param controlMode Determines the mode of Rx interface gain control, either
 * automatic or manual.
 * @param gain Specifies the gain value for the Rx interface.
 * @param rssiDuration Duration of RSSI measurement in units of 1ms/255.
 * @param rssiMovingAverageDuration Number of measurements in the RSSI moving-
 * average window.
 * @param gainControlAutomaticThreshold_dBFS Maximum signal level target in dBFS
 * for automatic gain control.
 * @param signalPAR Peak to Average Ratio of the applied signal.
 * @param enableFastAttack Indicates whether fast attack mode is enabled,
 * affecting the configuration of the moving average
 * filter.
 ******************************************************************************/
typedef struct adi_adrv9001_RxInterfaceGainCtrl
{
    adi_adrv9001_RxInterfaceGainUpdateTiming_e  updateInstance;
    adi_adrv9001_RxInterfaceGainCtrlMode_e      controlMode;
    adi_adrv9001_RxInterfaceGain_e              gain;
    uint8_t rssiDuration;                                           /* Duration of RSSI measurement (unit = 1ms/255 ) */
    uint8_t rssiMovingAverageDuration;                              /* Number of measurements in RSSI Moving-Average window */
    int8_t gainControlAutomaticThreshold_dBFS;                      /* Max signal level target in dBFS */
	uint8_t signalPAR;                                              /* Peak to Average Ratio of applied signal */
	bool enableFastAttack;                                          /* false: fastAttack and tracking use same configuration of MovingAveragefilter, true: No MovingAveragefilter in fastAttack region */

} adi_adrv9001_RxInterfaceGainCtrl_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_AdcTypeSwitchMode_e` is an enumeration that defines
 * the different modes for switching the ADC type in the ADRV9001 device.
 * It includes options for immediate switching, enabling pin control, and
 * disabling pin control, allowing for flexible configuration of the ADC
 * switching behavior based on the application requirements.
 *
 * @param ADI_ADRV9001_ADC_SWITCH_IMMEDIATE Represents an immediate switch mode
 * for the ADC.
 * @param ADI_ADRV9001_ADC_SWITCH_PINCONTROL_ENABLED Represents a switch mode
 * where pin control is
 * enabled for the ADC.
 * @param ADI_ADRV9001_ADC_SWITCH_PINCONTROL_DISABLED Represents a switch mode
 * where pin control is
 * disabled for the ADC.
 ******************************************************************************/
typedef enum adi_adrv9001_AdcTypeSwitchMode
{
    ADI_ADRV9001_ADC_SWITCH_IMMEDIATE,
    ADI_ADRV9001_ADC_SWITCH_PINCONTROL_ENABLED,
    ADI_ADRV9001_ADC_SWITCH_PINCONTROL_DISABLED
} adi_adrv9001_AdcTypeSwitchMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_AdcSwitchCfg_t` structure is used to configure the
 * ADC switch settings in the ADRV9001 device. It includes the mode of
 * switching, which can be immediate or controlled via pins, and the GPIO
 * configuration necessary for implementing the switch. This structure is
 * essential for managing how the ADCs are switched in different
 * operational scenarios, providing flexibility in hardware control.
 *
 * @param adcSwitchMode Specifies the mode of ADC switching using the
 * adi_adrv9001_AdcTypeSwitchMode_e enumeration.
 * @param adcSwitchGpio Holds the GPIO configuration for ADC switching using the
 * adi_adrv9001_GpioCfg_t structure.
 ******************************************************************************/
typedef struct adi_adrv9001_AdcSwitchCfg
{
    adi_adrv9001_AdcTypeSwitchMode_e adcSwitchMode;
    adi_adrv9001_GpioCfg_t adcSwitchGpio;
} adi_adrv9001_AdcSwitchCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GainIndexPinCfg_t` structure is used to configure
 * the GPIO pin crumbs for routing the gain index in the ADRV9001 device.
 * It consists of four members, each corresponding to a pair of bits in
 * the gain index, allowing for the selection of specific GPIO pin crumbs
 * to route the 8-bit gain index through digital GPIO pins. This
 * configuration is crucial for controlling the gain index via hardware
 * pins in applications where precise gain control is required.
 *
 * @param gainIndex_01_00 Selects the GPIO pin crumb for bits 0 and 1 of the
 * gain index.
 * @param gainIndex_03_02 Selects the GPIO pin crumb for bits 2 and 3 of the
 * gain index.
 * @param gainIndex_05_04 Selects the GPIO pin crumb for bits 4 and 5 of the
 * gain index.
 * @param gainIndex_07_06 Selects the GPIO pin crumb for bits 6 and 7 of the
 * gain index.
 ******************************************************************************/
typedef struct adi_adrv9001_GainIndexPinCfg
{
    adi_adrv9001_GpioPinCrumbSel_e gainIndex_01_00;
    adi_adrv9001_GpioPinCrumbSel_e gainIndex_03_02;
    adi_adrv9001_GpioPinCrumbSel_e gainIndex_05_04;
    adi_adrv9001_GpioPinCrumbSel_e gainIndex_07_06;
} adi_adrv9001_GainIndexPinCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxPortSwitchCfg_t` structure is used to configure
 * the frequency range and switching behavior for two ports, Port A and
 * Port B, in an ADRV9001 device. It includes fields to specify the
 * minimum and maximum frequencies for each port, as well as flags to
 * enable the configuration and to allow manual switching between the
 * ports. This structure is essential for managing the RX port switching
 * based on frequency requirements in radio applications.
 *
 * @param minFreqPortA_Hz Minimum frequency in Hertz for Port A.
 * @param maxFreqPortA_Hz Maximum frequency in Hertz for Port A.
 * @param minFreqPortB_Hz Minimum frequency in Hertz for Port B.
 * @param maxFreqPortB_Hz Maximum frequency in Hertz for Port B.
 * @param enable Boolean flag to enable or disable the port switch
 * configuration.
 * @param manualRxPortSwitch Boolean flag to enable or disable manual RX port
 * switching.
 ******************************************************************************/
typedef struct adi_adrv9001_RxPortSwitchCfg
{
    uint64_t  minFreqPortA_Hz;
    uint64_t  maxFreqPortA_Hz;
    uint64_t  minFreqPortB_Hz;
    uint64_t  maxFreqPortB_Hz;
    bool      enable;
    bool      manualRxPortSwitch;
} adi_adrv9001_RxPortSwitchCfg_t;
    
/***************************************************************************//**
 * @brief The `adi_adrv9001_RxrfdcLoidCfg_t` structure is used to configure the
 * Local Oscillator Interference Detection (LOID) settings for the
 * ADRV9001 receiver channels RX1 and RX2. It contains a boolean flag
 * `loidEnable` to activate or deactivate the LOID feature, and an 8-bit
 * unsigned integer `loidThreshold_negdBFS` that specifies the threshold
 * for LO detection in negative decibels relative to full scale (-dBFS).
 * This configuration is crucial for managing and mitigating local
 * oscillator interference in the receiver's signal path.
 *
 * @param loidEnable LOID enable flag for RX1 and RX2.
 * @param loidThreshold_negdBFS Threshold for LO detection (in -dBFS).
 ******************************************************************************/
typedef struct adi_adrv9001_RxrfdcLoidCfg
{
	bool loidEnable;								/* LOID enable flag for RX1 and RX2 */
	uint8_t loidThreshold_negdBFS;				    /* Threshold for LO detection (in -dBFS) */  
} adi_adrv9001_RxrfdcLoidCfg_t ;	
    
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_RX_TYPES_H_ */
