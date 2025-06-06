/**
 * \file
 * \brief Contains ADRV9001 Power saving and Monitor mode data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_TYPES_H_
#define _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_TYPES_H_

#include "adi_adrv9001_user.h"

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_ArmMonitorModeStates_e` is
 * an enumeration that defines the various states of the ARM monitor mode
 * within the ADRV9001 power saving and monitor mode framework. It
 * includes states such as sleep, detecting, detected, and
 * streaming/streamed, which are used to manage and transition between
 * different operational phases when the system is in monitor mode.
 *
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_SLEEP Sleep
 * state in
 * monitor
 * mode.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_DETECTING Detec
 * ting
 * state
 * in mo
 * nitor
 * mode.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_DETECTED Detect
 * ed
 * state
 * in mon
 * itor
 * mode.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_STREAMING_STR…S
 * t
 * r
 * e
 * a
 * m
 * i
 * n
 * g
 * o
 * r
 * S
 * t
 * r
 * e
 * a
 * m
 * e
 * d
 * s
 * t
 * a
 * t
 * e
 * i
 * n
 * m
 * o
 * n
 * i
 * t
 * o
 * r
 * m
 * o
 * d
 * e
 * .
 ******************************************************************************/
typedef enum adi_adrv9001_PowerSavingAndMonitorMode_ArmMonitorModeStates
{
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_SLEEP,              /*!< Sleep state in monitor mode */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_DETECTING,          /*!< Detecting state in monitor mode */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_DETECTED,           /*!< Detected state in monitor mode */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_ARM_MONITORMODE_STREAMING_STREAMED, /*!< Streaming or Streamed state in monitor mode */
} adi_adrv9001_PowerSavingAndMonitorMode_ArmMonitorModeStates_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerDownMode_e` is
 * an enumeration that defines the different power down modes available
 * for individual channels in the ADRV9001 device. It provides options to
 * either keep the channel in its default operational state, power down
 * the RF PLL, or power down the channel's LDO, allowing for flexible
 * power management and optimization of power consumption based on the
 * specific needs of the application.
 *
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_DISABLED Default
 * radio ope
 * ration,
 * no extra
 * power
 * down.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_RFPLL RF PLL power
 * down.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_LDO Channel LDO
 * power down.
 ******************************************************************************/
typedef enum adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerDownMode
{
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_DISABLED = 0, /*!< Default radio operation, no extra power down */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_RFPLL = 1,    /*!< RF PLL power down */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_CHANNEL_MODE_LDO = 2,      /*!< Channel LDO power down */
} adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerDownMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e` is
 * an enumeration that defines different system-level power down modes
 * for the ADRV9001 device. It includes modes for powering down the clock
 * PLL, LDO, and ARM components, which are essential for managing power
 * consumption and optimizing the device's performance in various
 * operational states.
 *
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_CLKPLL CLK PLL
 * power down.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_LDO LDO power down.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_ARM ARM power down.
 ******************************************************************************/
typedef enum adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode
{
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_CLKPLL = 3, /*!< CLK PLL power down */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_LDO = 4,    /*!< LDO power down */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_SYSTEM_MODE_ARM = 5     /*!< ARM power down */
} adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorDetectionMode_e` is
 * an enumeration that defines various detection modes available in the
 * monitor mode of the ADRV9001 device. These modes specify the type of
 * signal detection to be performed, such as RSSI, Sync, or FFT, and
 * combinations thereof, to facilitate power saving and monitoring
 * functionalities.
 *
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI RSS
 * I d
 * ete
 * cti
 * on 
 * onl
 * y.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_SYNC Syn
 * c d
 * ete
 * cti
 * on 
 * onl
 * y.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_FFT FFT 
 * dete
 * ctio
 * n on
 * ly.
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI_S…R
 * S
 * S
 * I
 * a
 * n
 * d
 * S
 * y
 * n
 * c
 * d
 * e
 * t
 * e
 * c
 * t
 * i
 * o
 * n
 * .
 * @param ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI_F…R
 * S
 * S
 * I
 * a
 * n
 * d
 * F
 * F
 * T
 * d
 * e
 * t
 * e
 * c
 * t
 * i
 * o
 * n
 * .
 ******************************************************************************/
typedef enum adi_adrv9001_PowerSavingAndMonitorMode_MonitorDetectionMode
{
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI,      /*!< RSSI detection only */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_SYNC,      /*!< Sync detection only */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_FFT,       /*!< FFT detection only */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI_SYNC, /*!< RSSI and Sync detection */
    ADI_ADRV9001_POWERSAVINGANDMONITORMODE_MONITOR_DETECTION_MODE_RSSI_FFT,  /*!< RSSI and FFT detection */
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorDetectionMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerSavingCfg_t`
 * structure is designed to configure power saving settings for a channel
 * in the ADRV9001 device. It includes settings for the power down mode
 * when the channel is disabled and an optional additional power down
 * mode for GPIO pins, allowing for flexible power management
 * configurations.
 *
 * @param channelDisabledPowerDownMode Specifies the power down mode when the
 * channel is disabled.
 * @param gpioPinPowerDownMode Specifies an additional GPIO pin power down mode,
 * valid only if greater than
 * channelDisabledPowerDownMode.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerSavingCfg
{
    adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerDownMode_e channelDisabledPowerDownMode; /*!< Pin power down mode */
    /** Additional GPIO pin power down mode; only valid if greater than channelDisabledPowerDownMode */
    adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerDownMode_e gpioPinPowerDownMode;
} adi_adrv9001_PowerSavingAndMonitorMode_ChannelPowerSavingCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerSavingAndMonito
 * rModeCfg_t` structure is designed to configure the power saving and
 * monitor mode settings for the ADRV9001 system. It includes parameters
 * for setting the power down mode, configuring timers for battery saver
 * delay, detection, and sleep states, and determining the initial state
 * when entering monitor mode. Additionally, it allows configuration of
 * the detection mode, enabling of wakeup signals, and control over the
 * external PLL. This structure is essential for managing power
 * efficiency and monitoring capabilities in the ADRV9001 system.
 *
 * @param powerDownMode Power down mode for Monitor.
 * @param initialBatterySaverDelay_us Delay Timer (us) for detection before
 * entering Monitor Mode.
 * @param detectionTime_us Timer for detection state (us).
 * @param sleepTime_us Timer for sleep state (us).
 * @param detectionFirst Select first state when entering Monitor Mode, 0-sleep
 * first, 1-detection first.
 * @param detectionMode Mode of detection in the detect state.
 * @param bbicWakeupLevelEnable Enable ADI_ADRV9001_GPIO_SIGNAL_MON_BBIC_WAKEUP
 * as a Level instead of Pulse (0 = Pulse, 1 =
 * Level).
 * @param externalPllEnable External PLL Enable, 0-disable, 1-enable.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerSavingAndMonitorModeCfg
{
    adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerDownMode_e powerDownMode;  /*!< Power down mode for Monitor */
    uint32_t initialBatterySaverDelay_us;                                        /*!< Delay Timer (us) for detection before entering Monitor Mode  */
    uint32_t detectionTime_us;                                                   /*!< Timer for detection state (us) */
    uint32_t sleepTime_us;                                                       /*!< Timer for sleep state (us) */
    uint8_t detectionFirst;                                                      /*!< Select first state when entering Monitor Mode, 0-sleep first, 1-detection first */
    adi_adrv9001_PowerSavingAndMonitorMode_MonitorDetectionMode_e detectionMode; /*!< Mode of detection in the detect state */
    bool bbicWakeupLevelEnable;                                               /*!< Enable  ADI_ADRV9001_GPIO_SIGNAL_MON_BBIC_WAKEUP as a Level instead of Pulse (0 = Pulse, 1 = Level) */
    bool externalPllEnable;                                                      /*!< External PLL Enable, 0-disable, 1-enable */
} adi_adrv9001_PowerSavingAndMonitorMode_SystemPowerSavingAndMonitorModeCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorModePatternCfg_t`
 * structure is designed to hold configuration parameters for the monitor
 * mode correlator pattern in the ADRV9001 device. It includes a
 * `patternLength` field that specifies the length of the pattern in
 * terms of I/Q sample pairs, and two arrays, `patternI` and `patternQ`,
 * each capable of holding up to 2048 31-bit values, representing the I
 * and Q detection patterns respectively. This structure is crucial for
 * defining the specific patterns used in monitor mode to detect signals.
 *
 * @param patternLength Length of pattern in I/Q sample pairs.
 * @param patternI 31-bit Monitor Mode I Detection Pattern.
 * @param patternQ 31-bit Monitor Mode Q Detection Pattern.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_MonitorModePatternCfg
{
    uint32_t patternLength;     /*!< Length of pattern in I/Q sample pairs */
    uint32_t patternI[2048];    /*!< 31-bit Monitor Mode I Detection Pattern */
    uint32_t patternQ[2048];    /*!< 31-bit Monitor Mode Q Detection Pattern */
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorModePatternCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg_t`
 * structure is designed to hold configuration parameters for the monitor
 * mode correlator vector in the ADRV9001 device. It includes a 14-bit
 * mask (`vectorMask`) that specifies which bits of the correlator vector
 * are active, and an array (`vector`) of 14 elements, each representing
 * a 48-bit correlator vector. This configuration is crucial for defining
 * the pattern matching behavior in monitor mode, allowing the device to
 * detect specific signal patterns efficiently.
 *
 * @param vectorMask 14-bit Monitor Mode Correlator Vector Mask.
 * @param vector Array of 14 12-HEX Character (48 bits) Monitor Mode Correlator
 * Vectors.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg
{
    uint16_t vectorMask;   /*!< 14-bit Monitor Mode Correlator Vector Mask */
    uint64_t vector[14];   /*!< 12-HEX Character(48 bits) Monitor Mode Correlator Vectors */
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeVectorCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg_t`
 * structure is designed to configure the RSSI measurement settings for
 * the monitor mode in the ADRV9001 system. It allows the user to specify
 * the number of RSSI measurements to average, the period between
 * consecutive measurements, the threshold for notifying the baseband
 * integrated circuit (BBIC) if the average RSSI exceeds a certain level,
 * and the duration of each RSSI measurement in terms of samples. This
 * configuration is crucial for determining the signal strength and
 * deciding whether to notify the BBIC of a detected signal or to
 * continue monitoring.
 *
 * @param numberOfMeasurementsToAverage Number of RSSI measurements (1 to 255)
 * to average.
 * @param measurementsStartPeriod_ms Period of consecutive RSSI measurements in
 * milliseconds (valid: 0 to 255).
 * @param detectionThreshold_mdBFS Threshold at which to notify BBIC if exceeded
 * by average RSSI, with a step size of 100
 * mdBFS ranging from -140000 to 0.
 * @param measurementDuration_samples Duration of a single RSSI measurement in
 * samples, ranging from 1 to (2^21 - 1).
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg
{
    uint8_t numberOfMeasurementsToAverage; /*!< Number of RSSI measurements(1 to 255) to average */
    
    /**
     * \brief Period of consecutive RSSI measurements, denoted in milliseconds (valid: 0 to 255)
     *
     * \note 0: Continuous RSSI measurements (currently not supported by FW). 
     * \note 1 to 255: Start a new RSSI measurement at regular intervals of measurementsStartPeriod_ms or as fast as the hardware is able to.
     * \note Irrelevant if numberOfMeasurementsToAverage is 1
     */
    uint8_t measurementsStartPeriod_ms; 

    /** 
     * \brief Threshold at which to notify BBIC (if exceeded by average RSSI)
     * 
     * \note Step size for detection threshold is 100 mdBFS, ranging from -140000 to 0. 
     */
    int32_t detectionThreshold_mdBFS;
    uint32_t measurementDuration_samples; /*!< Duration of a single RSSI measurement denoted in samples ranging from 1 to (2^21 - 1)*/
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeDmrSearchCfg_t`
 * structure is used to configure the DMR search parameters in monitor
 * mode for the ADRV9001 device. It includes settings for path delay,
 * power threshold, and detection count range, which are crucial for
 * identifying and differentiating signal frames from noise. The
 * structure is designed to optimize the detection process by specifying
 * the earliest and latest positions in the frame to search, as well as
 * the minimum correlator output level required for sync detection.
 *
 * @param pathDelay Path delay from RxDmrPd calibration, ranging from 0 to 2047
 * samples.
 * @param magcorrTh Power threshold to differentiate noise floor from possible
 * frame data, suggested value is 9000.
 * @param detCnt1 Earliest position in frame to search, suggested value is 375.
 * @param detCnt2 Latest position in frame to search, suggested value is 500.
 * @param detTgtMin Minimum correlator output level for sync detection,
 * suggested value is 681574.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeDmrSearchCfg {
    uint32_t pathDelay;     /*!< Path delay from RxDmrPd calibration, 0..2047 samples */
    int32_t  magcorrTh;     /*!< Power threshold, to differentiate noise floor from possible frame data.
                                 Units are 24x the square of the units at input to the frequency discriminator.
                                 Suggested value is 9000 */
    int16_t detCnt1;        /*!< Earliest position in frame to search. Suggested 375 */
    int16_t detCnt2;        /*!< Latest position in frame to search. Suggested 500 */
    int32_t detTgtMin;      /*!< Minimum correlator output level at which to allow sync detection.
                                 Units are 24x the units at the output of the frequency discriminator.
                                 Suggested 681574 = 2.6*(2^18) */
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeDmrSearchCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeInitCfg_t`
 * structure is designed to encapsulate the initial configuration
 * settings for the monitor mode of the ADRV9001 device. It includes two
 * main components: `monitorModeRssiCfg`, which manages the RSSI
 * (Received Signal Strength Indicator) settings necessary for signal
 * strength measurement and detection, and `dmrSearchCfg`, which handles
 * the DMR (Digital Mobile Radio) search configuration used during
 * monitor mode operations. This structure is crucial for setting up the
 * device to efficiently monitor and detect signals based on predefined
 * criteria.
 *
 * @param monitorModeRssiCfg Structure to hold RSSI configuration at
 * initialization time.
 * @param dmrSearchCfg Structure to hold DMR search configuration used in
 * monitor mode.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeInitCfg
{
    adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeRssiCfg_t      monitorModeRssiCfg;   /*!< Structure to hold RSSI configuration at initialization time */
    adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeDmrSearchCfg_t dmrSearchCfg;         /*!< structure to hold DMR search configuration used in monitor mode */
} adi_adrv9001_PowerSavingAndMonitorMode_MonitorModeInitCfg_t;

#endif /* _ADI_ADRV9001_POWERSAVINGANDMONITORMODE_TYPES_H_ */
