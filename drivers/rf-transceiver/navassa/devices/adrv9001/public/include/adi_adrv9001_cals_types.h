/**
 * \file
 * \brief Type definitions for ADRV9001 calibrations
 * \copyright Analog Devices Inc. 2019. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_CALS_TYPES_H_
#define _ADI_ADRV9001_CALS_TYPES_H_

#include "adi_adrv9001_defines.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_InitCalibrations_e` is an enumeration that defines
 * various initialization calibration options for the ADRV9001 device.
 * Each enumerator represents a specific calibration task, such as
 * correcting quadrature errors, adjusting path delays, or calibrating
 * ADC components, and is associated with a unique bitmask value. These
 * calibrations are essential for optimizing the performance of the
 * device's transmit (Tx) and receive (Rx) paths, ensuring accurate
 * signal processing and minimizing errors. The enumeration also includes
 * options for running all Tx, Rx, or system calibrations collectively,
 * as well as a specific subset required for LO retuning.
 *
 * @param ADI_ADRV9001_INIT_CAL_TX_QEC Tx Quadrature Error Correction.
 * @param ADI_ADRV9001_INIT_CAL_TX_LO_LEAKAGE Tx LO Leakage.
 * @param ADI_ADRV9001_INIT_CAL_TX_LB_PD Tx Loopback path delay.
 * @param ADI_ADRV9001_INIT_CAL_TX_DCC Tx Duty Cycle Correction.
 * @param ADI_ADRV9001_INIT_CAL_TX_BBAF Tx Baseband Analog Filter.
 * @param ADI_ADRV9001_INIT_CAL_TX_BBAF_GD Tx Baseband Analog Filter Group
 * Delay.
 * @param ADI_ADRV9001_INIT_CAL_TX_ATTEN_DELAY Tx Attenuation Delay.
 * @param ADI_ADRV9001_INIT_CAL_TX_DAC Tx DAC.
 * @param ADI_ADRV9001_INIT_CAL_TX_PATH_DELAY Tx Path Delay.
 * @param ADI_ADRV9001_INIT_CAL_RX_HPADC_RC Rx HP ADC Resistance and
 * Capacitance.
 * @param ADI_ADRV9001_INIT_CAL_RX_HPADC_FLASH Rx HP ADC Flash.
 * @param ADI_ADRV9001_INIT_CAL_RX_HPADC_DAC Rx HP ADC DAC.
 * @param ADI_ADRV9001_INIT_CAL_RX_DCC Rx Duty Cycle Correction.
 * @param ADI_ADRV9001_INIT_CAL_RX_LPADC Rx LP ADC.
 * @param ADI_ADRV9001_INIT_CAL_RX_TIA_CUTOFF Rx Trans-Impedance Amplifier
 * Cutoff.
 * @param ADI_ADRV9001_INIT_CAL_RX_GROUP_DELAY Rx Trans-Impedance Amplifier
 * Group Delay.
 * @param ADI_ADRV9001_INIT_CAL_RX_QEC_TCAL Rx QEC Tone Calibration.
 * @param ADI_ADRV9001_INIT_CAL_RX_QEC_FIC Rx QEC Frequency-Independent.
 * @param ADI_ADRV9001_INIT_CAL_RX_QEC_ILB_LO_DELAY Rx Internal Loopback LO
 * Delay.
 * @param ADI_ADRV9001_INIT_CAL_RX_RF_DC_OFFSET Rx RF DC Offset.
 * @param ADI_ADRV9001_INIT_LO_RETUNE Minimum subset of InitCals that must be
 * run for LO Retune.
 * @param ADI_ADRV9001_INIT_CAL_RX_GAIN_PATH_DELAY Rx Gain Path Delay.
 * @param ADI_ADRV9001_INIT_CAL_RX_DMR_PATH_DELAY Rx DMR Path Delay.
 * @param ADI_ADRV9001_INIT_CAL_PLL PLL.
 * @param ADI_ADRV9001_INIT_CAL_AUX_PLL AUX PLL.
 * @param ADI_ADRV9001_INIT_CAL_TX_ALL Tx all Init Cals.
 * @param ADI_ADRV9001_INIT_CAL_RX_ALL Rx all Init Cals.
 * @param ADI_ADRV9001_INIT_CAL_RX_TX_ALL Rx / Tx all Init Cals.
 * @param ADI_ADRV9001_INIT_CAL_SYSTEM_ALL All system Init Cals.
 ******************************************************************************/
typedef enum adi_adrv9001_InitCalibrations
{
    ADI_ADRV9001_INIT_CAL_TX_QEC                = 0x00000001, //!< Tx Quadrature Error Correction
    ADI_ADRV9001_INIT_CAL_TX_LO_LEAKAGE         = 0x00000002, //!< Tx LO Leakage
    ADI_ADRV9001_INIT_CAL_TX_LB_PD              = 0x00000004, //!< Tx Loopback path delay
    ADI_ADRV9001_INIT_CAL_TX_DCC                = 0x00000008, //!< Tx Duty Cycle Correction
    ADI_ADRV9001_INIT_CAL_TX_BBAF               = 0x00000010, //!< Tx Baseband Analog Filter
    ADI_ADRV9001_INIT_CAL_TX_BBAF_GD            = 0x00000020, //!< Tx Baseband Analog Filter Group Delay
    ADI_ADRV9001_INIT_CAL_TX_ATTEN_DELAY        = 0x00000040, //!< Tx Attenuation Delay
    ADI_ADRV9001_INIT_CAL_TX_DAC                = 0x00000080, //!< Tx DAC
    ADI_ADRV9001_INIT_CAL_TX_PATH_DELAY         = 0x00000100, //!< Tx Path Delay

    ADI_ADRV9001_INIT_CAL_RX_HPADC_RC           = 0x00000200, //!< Rx HP ADC Resistance and Capacitance
    ADI_ADRV9001_INIT_CAL_RX_HPADC_FLASH        = 0x00000400, //!< Rx HP ADC Flash
    ADI_ADRV9001_INIT_CAL_RX_HPADC_DAC          = 0x00000800, //!< Rx HP ADC DAC
    ADI_ADRV9001_INIT_CAL_RX_DCC                = 0x00001000, //!< Rx Duty Cycle Correction
    ADI_ADRV9001_INIT_CAL_RX_LPADC              = 0x00002000, //!< Rx LP ADC
    ADI_ADRV9001_INIT_CAL_RX_TIA_CUTOFF         = 0x00004000, //!< Rx Trans-Impedance Amplifier Cutoff
    ADI_ADRV9001_INIT_CAL_RX_GROUP_DELAY        = 0x00008000, //!< Rx Trans-Impedance Amplifier Group Delay
    ADI_ADRV9001_INIT_CAL_RX_QEC_TCAL           = 0x00010000, //!< Rx QEC Tone Calibration
    ADI_ADRV9001_INIT_CAL_RX_QEC_FIC            = 0x00020000, //!< Rx QEC Frequency-Independent
    ADI_ADRV9001_INIT_CAL_RX_QEC_ILB_LO_DELAY   = 0x00040000, //!< Rx Internal Loopback LO Delay
    ADI_ADRV9001_INIT_CAL_RX_RF_DC_OFFSET       = 0x00080000, //!< Rx RF DC Offset
	
	ADI_ADRV9001_INIT_LO_RETUNE                 = 0x000B902B, //!< Minimium Subset of InitCals that must be run for LO Retune

    ADI_ADRV9001_INIT_CAL_RX_GAIN_PATH_DELAY    = 0x00100000, //!< Rx Gain Path Delay
    ADI_ADRV9001_INIT_CAL_RX_DMR_PATH_DELAY     = 0x00200000, //!< Rx DMR Path Delay

    ADI_ADRV9001_INIT_CAL_PLL                   = 0x00400000, //!< PLL
    ADI_ADRV9001_INIT_CAL_AUX_PLL               = 0x00800000, //!< AUX PLL

    ADI_ADRV9001_INIT_CAL_TX_ALL                = 0x000001FF, //!< Tx all Init Cals
    ADI_ADRV9001_INIT_CAL_RX_ALL                = 0x001FFE00, //!< Rx all Init Cals
    ADI_ADRV9001_INIT_CAL_RX_TX_ALL             = 0x001FFFFF, //!< Rx / Tx all Init Cals
    ADI_ADRV9001_INIT_CAL_SYSTEM_ALL            = 0x00C00000, //!< All system Init Cals
}adi_adrv9001_InitCalibrations_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TrackingCalibrations_e` is an enumeration that
 * defines various tracking calibration options for the ADRV9001 device.
 * Each enumerator represents a specific calibration type, such as Tx
 * Quadrature Error Correction, Tx LO Leakage, and Rx Harmonic
 * Distortion, among others. These calibrations are used to optimize the
 * performance of the device by correcting errors and improving signal
 * quality in both transmission (Tx) and reception (Rx) paths. The
 * enumeration values are represented as bit masks, allowing for the
 * selection of multiple calibrations simultaneously.
 *
 * @param ADI_ADRV9001_TRACKING_CAL_TX_QEC Tx Quadrature Error Correction.
 * @param ADI_ADRV9001_TRACKING_CAL_TX_LO_LEAKAGE Tx LO Leakage.
 * @param ADI_ADRV9001_TRACKING_CAL_TX_LB_PD Tx Loopback path delay.
 * @param ADI_ADRV9001_TRACKING_CAL_TX_PAC Tx Power Amplifier Correction.
 * @param ADI_ADRV9001_TRACKING_CAL_TX_DPD_CLGC Tx Digital Pre Distortion and
 * Close Loop Gain Control.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_HD2 Rx Harmonic Distortion.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_QEC_WBPOLY Rx Quadrature Error Correction
 * Wideband Poly.
 * @param ADI_ADRV9001_TRACKING_CAL_ORX_QEC_WBPOLY ORx Quadrature Error
 * Correction Wideband Poly.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_BBDC Rx Baseband DC rejection.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_RFDC Rx RF DC.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_QEC_FIC Rx Quadrature Error Correction
 * FIC.
 * @param ADI_ADRV9001_TRACKING_CAL_RX_GAIN_CONTROL_DETECTORS Rx Gain Control
 * Detectors (Power,
 * Analog Peak and
 * Half Band).
 * @param ADI_ADRV9001_TRACKING_CAL_RX_RSSI Rx RSSI.
 ******************************************************************************/
typedef enum adi_adrv9001_TrackingCalibrations
{
    /* SW and HW tracking cals */
    ADI_ADRV9001_TRACKING_CAL_TX_QEC                                = 0x00000001, //!< Tx Quadrature Error Correction
    ADI_ADRV9001_TRACKING_CAL_TX_LO_LEAKAGE                         = 0x00000002, //!< Tx LO Leakage
    ADI_ADRV9001_TRACKING_CAL_TX_LB_PD                              = 0x00000004, //!< Tx Loopback path delay
    ADI_ADRV9001_TRACKING_CAL_TX_PAC                                = 0x00000008, //!< Tx Power Amplifier Correction
    ADI_ADRV9001_TRACKING_CAL_TX_DPD_CLGC                           = 0x00000010, //!< Tx Digital Pre Distortion and Close Loop Gain Control
    /* Bit 6-7: Not used (Reserved for future purpose) */
    ADI_ADRV9001_TRACKING_CAL_RX_HD2                                = 0x00000100, //!< Rx Harmonic Distortion
    ADI_ADRV9001_TRACKING_CAL_RX_QEC_WBPOLY                         = 0x00000200, //!< Rx Quadrature Error Correction Wideband Poly
    /* Bit 10-11: Not used (Reserved for future purpose) */
    ADI_ADRV9001_TRACKING_CAL_ORX_QEC_WBPOLY                        = 0x00001000, //!< ORx Quadrature Error Correction Wideband Poly
    /* Bit 13-18:  Not used (Reserved for future purpose) */
    ADI_ADRV9001_TRACKING_CAL_RX_BBDC                               = 0x00080000, //!< Rx Baseband DC rejection
    ADI_ADRV9001_TRACKING_CAL_RX_RFDC                               = 0x00100000, //!< Rx RF DC
    ADI_ADRV9001_TRACKING_CAL_RX_QEC_FIC                            = 0x00200000, //!< Rx Quadrature Error Correction FIC
    ADI_ADRV9001_TRACKING_CAL_RX_GAIN_CONTROL_DETECTORS             = 0x00400000, //!< Rx Gain Control Detectors (Power, Analog Peak and Half Band)
    ADI_ADRV9001_TRACKING_CAL_RX_RSSI                               = 0x00800000  //!< Rx RSSI
    /* Bit 24-31: Not used */
}adi_adrv9001_TrackingCalibrations_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_InitCalMode_e` is an enumeration that defines the
 * different modes for running initial calibrations in the ADRV9001
 * system. It provides options to run calibrations on all profiles,
 * specifically on system and Rx profiles, on Loopback and Tx profiles,
 * or exclusively on External Loop Back profiles. This enum is used to
 * specify the desired calibration mode, which is crucial for setting up
 * the system's initial calibration process, especially when dealing with
 * different LO configurations.
 *
 * @param ADI_ADRV9001_INIT_CAL_MODE_ALL Run initial calibrations on all
 * profiles.
 * @param ADI_ADRV9001_INIT_CAL_MODE_SYSTEM_AND_RX Run initial calibrations for
 * the system and on Rx
 * profiles.
 * @param ADI_ADRV9001_INIT_CAL_MODE_LOOPBACK_AND_TX Run initial calibrations on
 * Loopback (Internal &
 * External) and Tx profiles.
 * @param ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY Run initial calibrations only on
 * External Loop Back (ELB) profiles,
 * selectable only when external path
 * delay calibration is run.
 ******************************************************************************/
typedef enum adi_adrv9001_InitCalMode
{
    ADI_ADRV9001_INIT_CAL_MODE_ALL,             //!< Run initial calibrations on all profiles
    ADI_ADRV9001_INIT_CAL_MODE_SYSTEM_AND_RX,   //!< Run initial calibrations for the system and on Rx profiles
    ADI_ADRV9001_INIT_CAL_MODE_LOOPBACK_AND_TX, //!< Run initial calibrations on Loopback (Internal & External) and Tx profiles
    ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY         //!< Run initial calibrations only on External Loop Back (ELB) profiles.
                                                //!< 'ADI_ADRV9001_INIT_CAL_MODE_ELB_ONLY' can be selected only when external path delay calibration is run
}adi_adrv9001_InitCalMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_InitCals_t` structure is designed to manage
 * initialization calibrations for the ADRV9001 device. It includes a
 * system-wide calibration mask (`sysInitCalMask`) for non-channel
 * specific calibrations, and an array (`chanInitCalMask`) for channel-
 * specific calibrations, allowing separate configurations for two
 * channels (Rx1/Tx1 and Rx2/Tx2). The `calMode` member determines the
 * mode in which the initialization calibrations are executed, and the
 * `force` member allows for the re-execution of all enabled calibrations
 * if set to true. This structure is essential for configuring and
 * managing the calibration processes necessary for optimal device
 * performance.
 *
 * @param sysInitCalMask Calibration bit mask for non-channel related init cals.
 * @param chanInitCalMask Array containing calibration bit mask for channel
 * related init cals, with two masks for Rx1/Tx1 and
 * Rx2/Tx2 channels.
 * @param calMode Enum specifies the mode to run desired InitCals algorithms.
 * @param force A boolean value that, when true, forces all enabled calibrations
 * to re-run.
 ******************************************************************************/
typedef struct adi_adrv9001_InitCals
{
    adi_adrv9001_InitCalibrations_e sysInitCalMask;     //!< Calibration bit mask for non-channel related init cals
    /** Array containing calibration bit mask for channel related init cals.
        It contains two masks:
        1. chanInitCalMask[0]: CH_1 for masks on Rx1/Tx1 channels,
        2. chanInitCalMask[1]: CH_2 for masks on Rx2/Tx2 channels */
    adi_adrv9001_InitCalibrations_e chanInitCalMask[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_InitCalMode_e  calMode;                //!< Enum specifies the mode to run desired InitCals algorithms
    bool force;  //!< A value of true will force all enabled calibrations to re-run
} adi_adrv9001_InitCals_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TrackingCals_t` structure is designed to hold
 * tracking calibration masks for two channels, CH_1 and CH_2, which
 * correspond to Rx1/Tx1 and Rx2/Tx2 channels respectively. This
 * structure uses an array of type `adi_adrv9001_TrackingCalibrations_e`
 * to store the calibration bit masks, allowing for the configuration and
 * management of tracking calibrations specific to each channel.
 *
 * @param chanTrackingCalMask An array containing calibration bit masks for
 * channel-related tracking calibrations, with two
 * masks for Rx1/Tx1 and Rx2/Tx2 channels.
 ******************************************************************************/
typedef struct adi_adrv9001_TrackingCals
{
    /** Array containing calibration bit mask for channel related tracking cals.
        It contains two masks:
        1. chanTrackingCalMask[0]: CH_1 for masks on Rx1/Tx1 channels,
        2. chanTrackingCalMask[1]: CH_2 for masks on Rx2/Tx2 channels */
    adi_adrv9001_TrackingCalibrations_e chanTrackingCalMask[ADI_ADRV9001_MAX_RX_ONLY];
} adi_adrv9001_TrackingCals_t;

#endif /* _ADI_ADRV9001_CALS_TYPES_H_ */
