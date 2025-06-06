/**
* \file
* \brief Contains ADRV9001 API gain control data types
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2019 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_GAINCONTROL_TYPES_H_
#define _ADI_ADRV9001_GAINCONTROL_TYPES_H_

#include "adi_adrv9001_gpio_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainControlDetectionMode_e` is an enumeration that
 * defines the available modes for gain control detection in the ADRV9001
 * transceiver. It provides two modes: one that utilizes both peak and
 * power detection, and another that relies solely on peak detection.
 * This allows for flexibility in configuring the gain control mechanism
 * based on the specific requirements of the application.
 *
 * @param ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK_AND_POWER This mode
 * uses both
 * peak and
 * power
 * detection
 * for gain
 * control.
 * @param ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK This mode uses only
 * peak detection for
 * gain control.
 ******************************************************************************/
typedef enum adi_adrv9001_RxGainControlDetectionMode
{
    ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK_AND_POWER,
    ADI_ADRV9001_RX_GAIN_CONTROL_DETECTION_MODE_PEAK
} adi_adrv9001_RxGainControlDetectionMode_e ;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PeakDetector_t` structure is designed to configure
 * and manage the peak detection settings for the Automatic Gain Control
 * (AGC) in the ADRV9001 transceiver. It includes various parameters for
 * setting thresholds, intervals, and gain steps for both APD (Analog
 * Peak Detector) and HB (High Band) components. The structure allows for
 * detailed control over the AGC's response to signal peaks, including
 * enabling or disabling overload detection, setting threshold counts,
 * and configuring feedback through GPIO pins. This structure is crucial
 * for optimizing the AGC's performance in dynamic signal environments.
 *
 * @param agcUnderRangeLowInterval Update interval for AGC loop mode in AGC
 * clock cycles, with a valid range of 0 to
 * 65535.
 * @param agcUnderRangeMidInterval Second update interval for multiple time
 * constant AGC mode, calculated as (agcUnderRan
 * geMidInterval+1)*agcUnderRangeLowInterval_ns,
 * with a valid range of 0 to 63.
 * @param agcUnderRangeHighInterval Third update interval for multiple time
 * constant AGC mode, calculated as
 * (agcUnderRangeHighInterval+1)*2nd update
 * interval, with a valid range of 0 to 63.
 * @param apdHighThresh AGC APD high threshold, with a valid range of 0 to 63.
 * @param apdLowThresh AGC APD low threshold, with a valid range of 0 to 63,
 * recommended to be 3dB below apdHighThresh.
 * @param apdUpperThreshPeakExceededCount AGC APD peak detect upper threshold
 * count, with a valid range of 0 to 255.
 * @param apdLowerThreshPeakExceededCount AGC APD peak detect lower threshold
 * count, with a valid range of 0 to 255.
 * @param apdGainStepAttack AGC APD peak detect attack gain step, with a valid
 * range of 0 to 31.
 * @param apdGainStepRecovery AGC APD gain index step size for recovery, with a
 * valid range of 0 to 31.
 * @param enableHbOverload Enables or disables the HB overload detector.
 * @param hbOverloadDurationCount Sets the window of clock cycles (at the HB
 * output rate) to meet the overload count, with
 * specific cycle options.
 * @param hbOverloadThreshCount Sets the number of actual overloads required to
 * trigger the overload signal, with a valid range
 * from 1 to 15.
 * @param hbHighThresh AGC HB output high threshold, with a valid range from 0
 * to 16383.
 * @param hbUnderRangeLowThresh AGC HB output low threshold, with a valid range
 * from 0 to 16383.
 * @param hbUnderRangeMidThresh AGC HB output low threshold for 2nd interval for
 * multiple time constant AGC mode, with a valid
 * range from 0 to (A0: 255, B0: 16383).
 * @param hbUnderRangeHighThresh AGC HB output low threshold for 3rd interval
 * for multiple time constant AGC mode, with a
 * valid range from 0 to (A0: 255, B0: 16383).
 * @param hbUpperThreshPeakExceededCount AGC HB output upper threshold count,
 * with a valid range from 0 to 255.
 * @param hbUnderRangeHighThreshExceededCount AGC HB output lower threshold
 * count, with a valid range from 0
 * to 255.
 * @param hbGainStepHighRecovery AGC HB gain index step size, with a valid range
 * from 0 to 31.
 * @param hbGainStepLowRecovery AGC HB gain index step size, when the HB Low
 * Overrange interval 2 triggers, with a valid
 * range from 0 to 31.
 * @param hbGainStepMidRecovery AGC HB gain index step size, when the HB Low
 * Overrange interval 3 triggers, with a valid
 * range from 0 to 31.
 * @param hbGainStepAttack AGC HB output attack gain step, with a valid range
 * from 0 to 31.
 * @param hbOverloadPowerMode Selects magnitude measurements or power
 * measurements, with 0 for peak mode and 1 for I2 +
 * Q2 power mode.
 * @param hbUnderRangeMidThreshExceededCount AGC HB output upper threshold
 * count, with a valid range from 0 to
 * 255.
 * @param hbUnderRangeLowThreshExceededCount AGC HB output lower threshold
 * count, with a valid range from 0 to
 * 255.
 * @param feedback_apd_low_hb_low A pair of DGPIO pins indicating the apd low
 * threshold counter exceeded status and hb low
 * threshold counter exceeded status.
 * @param feedback_apd_high_hb_high A pair of DGPIO pins indicating the apd high
 * threshold counter exceeded status and hb
 * high threshold counter exceeded status.
 ******************************************************************************/
typedef struct adi_adrv9001_PeakDetector
{
    uint16_t    agcUnderRangeLowInterval;           /*!< Update interval for AGC loop mode in AGC clock cycles. Valid range is 0 to 65535 */
    uint8_t     agcUnderRangeMidInterval;           /*!< 2nd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeMidInterval+1)*agcUnderRangeLowInterval_ns. Valid range is 0 to 63 */
    uint8_t     agcUnderRangeHighInterval;          /*!< 3rd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeHighInterval+1)*2nd update interval. Valid range is 0 to 63 */
    uint8_t     apdHighThresh;                      /*!< AGC APD high threshold. Valid range is 0 to 63 */
    uint8_t     apdLowThresh;                       /*!< AGC APD low threshold. Valid range is 0 to 63. Recommended to be 3dB below apdHighThresh */
    uint8_t     apdUpperThreshPeakExceededCount;    /*!< AGC APD peak detect upper threshold count. Valid range is 0 to 255 */
    uint8_t     apdLowerThreshPeakExceededCount;    /*!< AGC APD peak detect lower threshold count. Valid range is 0 to 255 */
    uint8_t     apdGainStepAttack;                  /*!< AGC APD peak detect attack gain step. Valid range is 0 to 31 */
    uint8_t     apdGainStepRecovery;                /*!< AGC APD gain index step size for recovery. Valid range is 0 to 31 */
    bool        enableHbOverload;                   /*!< Enable or disables the HB overload detector. */
    uint8_t     hbOverloadDurationCount;            /*!< Sets the window of clock cycles (at the HB output rate) to meet the overload count. (0 = 2 cycles, 1 = 4 cycles, 2 = 8 cycles, 3 = 12 cycles, 4 = 16 cycles, 5 = 24 cycles, 6 = 32 cycles) */
    uint8_t     hbOverloadThreshCount;              /*!< Sets the number of actual overloads required to trigger the overload signal. Valid range from 1 to 15 */
    uint16_t    hbHighThresh;                       /*!< AGC HB output high threshold. Valid range from  0 to 16383 */
    uint16_t    hbUnderRangeLowThresh;              /*!< AGC HB output low threshold. Valid range from  0 to 16383 */
    uint16_t    hbUnderRangeMidThresh;              /*!< AGC HB output low threshold for 2nd interval for multiple time constant AGC mode. Valid range from  0 to (A0: 255, B0: 16383) */
    uint16_t    hbUnderRangeHighThresh;             /*!< AGC HB output low threshold for 3rd interval for multiple time constant AGC mode. Valid range from  0 to (A0: 255, B0: 16383) */
    uint8_t     hbUpperThreshPeakExceededCount;     /*!< AGC HB output upper threshold count. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeHighThreshExceededCount;/*!< AGC HB output lower threshold count. Valid range from  0 to 255 */
    uint8_t     hbGainStepHighRecovery;             /*!< AGC HB gain index step size. Valid range from  0 to 31 */
    uint8_t     hbGainStepLowRecovery;              /*!< AGC HB gain index step size, when the HB Low Overrange interval 2 triggers. Valid range from  0 to 31 */
    uint8_t     hbGainStepMidRecovery;              /*!< AGC HB gain index step size, when the HB Low Overrange interval 3 triggers. Valid range from  0 to 31 */
    uint8_t     hbGainStepAttack;                   /*!< AGC HB output attack gain step. Valid range from  0 to 31 */
    uint8_t     hbOverloadPowerMode;                /*!< Select magnitude measurements or power mearurements. 0 = enable peak mode 1 = enable I2 + Q2 power mode */
    uint8_t     hbUnderRangeMidThreshExceededCount; /*!< AGC HB output upper threshold count. Valid range from  0 to 255 */
    uint8_t     hbUnderRangeLowThreshExceededCount; /*!< AGC HB output lower threshold count. Valid range from  0 to 255 */

    /** A pair of DGPIO pins - ADI_ADRV9001_GPIO_PIN_CRUMB_XX_YY - where
     *  pin XX is the apd low threshold counter exceeded status and
     *  pin YY is the hb low threshold counter exceeded status
     */
    adi_adrv9001_GpioPinCrumbSel_e feedback_apd_low_hb_low;

    /** A pair of DGPIO pins - ADI_ADRV9001_GPIO_PIN_CRUMB_XX_YY - where
     *  pin XX is the apd high threshold counter exceeded status and
     *  pin YY is the hb high threshold counter exceeded status
     */
    adi_adrv9001_GpioPinCrumbSel_e feedback_apd_high_hb_high;
} adi_adrv9001_PeakDetector_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PowerDetector_t` structure is designed to manage and
 * configure the power detection settings for the ADRV9001 transceiver.
 * It includes various parameters to enable and control the power
 * measurement block, set thresholds for under and over-range power
 * detection, and define gain step recovery and attack settings.
 * Additionally, it provides configuration for measurement duration and
 * delay, as well as feedback mechanisms through DGPIO pins to indicate
 * threshold exceedance statuses. This structure is integral to the
 * automatic gain control (AGC) system, allowing for precise power
 * management and adjustment in response to signal conditions.
 *
 * @param powerEnableMeasurement Enable the Rx power measurement block.
 * @param underRangeHighPowerThresh AGC power measurement detect lower 0
 * threshold with a valid range from 0 to 127.
 * @param underRangeLowPowerThresh AGC power measurement detect lower 1
 * threshold with a valid offset from 0 to 15.
 * @param underRangeHighPowerGainStepRecovery AGC power measurement detect lower
 * 0 recovery gain step with a valid
 * range from 0 to 31.
 * @param underRangeLowPowerGainStepRecovery AGC power measurement detect lower
 * 1 recovery gain step with a valid
 * range from 0 to 31.
 * @param powerMeasurementDuration Average power measurement duration calculated
 * as 8*2^powerMeasurementDuration with a valid
 * range from 0 to 31.
 * @param powerMeasurementDelay Average power measurement delay between
 * successive measurements with a valid range from
 * 0 to 255.
 * @param rxTddPowerMeasDuration Measurement duration to detect power for a
 * specific slice of the gain update counter.
 * @param rxTddPowerMeasDelay Measurement delay to detect power for a specific
 * slice of the gain update counter.
 * @param overRangeHighPowerThresh AGC upper 0 threshold for power measurement
 * with a valid range from 0 to 15.
 * @param overRangeLowPowerThresh AGC upper 1 threshold for power measurement
 * with a valid offset from 0 to 127.
 * @param overRangeHighPowerGainStepAttack AGC power measurement detect lower 0
 * attack gain step with a valid range
 * from 0 to 31.
 * @param overRangeLowPowerGainStepAttack AGC power measurement detect lower 1
 * attack gain step with a valid range
 * from 0 to 31.
 * @param feedback_inner_high_inner_low A pair of DGPIO pins indicating power
 * detector inner high and low threshold
 * exceeded status.
 * @param feedback_apd_high_apd_low A pair of DGPIO pins indicating APD high and
 * low threshold counter exceeded status.
 ******************************************************************************/
typedef struct adi_adrv9001_PowerDetector
{
    bool        powerEnableMeasurement;                 /*!< Enable the Rx power measurement block */
    uint8_t     underRangeHighPowerThresh;              /*!< AGC power measurement detect lower 0 threshold. Valid Range from 0 to 127. */
    uint8_t     underRangeLowPowerThresh;               /*!< AGC power measurement detect lower 1 threshold. Valid offset from 0 to 15 */
    uint8_t     underRangeHighPowerGainStepRecovery;    /*!< AGC power measurement detect lower 0 recovery gain step. Valid range from  0 to 31 */
    uint8_t     underRangeLowPowerGainStepRecovery;     /*!< AGC power measurement detect lower 1 recovery gain step. Valid range from  0 to 31 */
    uint8_t     powerMeasurementDuration;               /*!< Average power measurement duration = 8*2^powerMeasurementDuration. Valid range from 0 to 31 */
    uint8_t     powerMeasurementDelay;                  /*!< Average power measurement delay between sucessive measurement. Valid range from 0 to 255 */
    uint16_t    rxTddPowerMeasDuration;                 /*!< Measurement duration to detect power for specific slice of the gain update counter. */
    uint16_t    rxTddPowerMeasDelay;                    /*!< Measurement delay to detect power for specific slice of the gain update counter. */
    uint8_t     overRangeHighPowerThresh;               /*!< AGC upper 0 (overRangeHighPowerThreshold) threshold for power measurement. Valid Range from 0 to 15.*/
    uint8_t     overRangeLowPowerThresh;                /*!< AGC upper 1 (overRangeLowPowerThreshold)  threshold for power measurement. Valid offset from 0 to 127 */
    uint8_t     overRangeHighPowerGainStepAttack;       /*!< AGC power measurement detect lower 0 attack gain step. Valid range from  0 to 31 */
    uint8_t     overRangeLowPowerGainStepAttack;        /*!< AGC power measurement detect lower 1 attack gain step. Valid range from  0 to 31 */

    /** A pair of DGPIO pins - ADI_ADRV9001_GPIO_PIN_CRUMB_XX_YY - where
     *  pin XX is the power detector inner high threshold exceeded status and
     *  pin YY is the power detector inner low threshold exceeded status
     */
    adi_adrv9001_GpioPinCrumbSel_e feedback_inner_high_inner_low;

    /** A pair of DGPIO pins - ADI_ADRV9001_GPIO_PIN_CRUMB_XX_YY - where
     *  pin XX is the apd high threshold counter exceeded status and
     *  pin YY is the apd low threshold counter exceeded status
     */
    adi_adrv9001_GpioPinCrumbSel_e feedback_apd_high_apd_low;
} adi_adrv9001_PowerDetector_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ExtLna_t` structure is designed to configure the
 * external Low Noise Amplifier (LNA) settings, specifically focusing on
 * the settling delay parameter. This parameter, `settlingDelay`, allows
 * users to specify the time delay required for the external LNA to
 * stabilize after being activated, ensuring optimal performance and
 * signal integrity. The valid range for this delay is between 0 and 255,
 * providing flexibility to accommodate various hardware configurations
 * and operational requirements.
 *
 * @param settlingDelay External LNA Settling Delay with a valid range from 0 to
 * 255.
 ******************************************************************************/
typedef struct adi_adrv9001_ExtLna
{
    uint8_t     settlingDelay;                      /*!< External LNA Settling Delay. Valid range is from 0 to 255 */
} adi_adrv9001_ExtLna_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GainControlCfg_t` structure is a comprehensive
 * configuration for the gain control settings of the ADRV9001
 * transceiver. It includes parameters for automatic gain control (AGC)
 * such as peak wait time, gain indices, and gain update timing. The
 * structure also allows for configuration of attack delays, settling
 * delays, and various modes of operation including peak and power
 * detection modes. Additionally, it supports synchronization options,
 * fast recovery loops, and external LNA settings. The structure is
 * designed to provide flexibility and precision in managing the gain
 * control mechanisms of the transceiver, ensuring optimal performance
 * across different operating conditions.
 *
 * @param peakWaitTime AGC peak wait time with a valid range from 0 to 31.
 * @param maxGainIndex AGC Rx max gain index with a valid range from
 * minGainIndex to 255.
 * @param minGainIndex AGC Rx min gain index with a valid range from 0 to
 * maxGainIndex.
 * @param gainUpdateCounter AGC gain update time in AGC clock cycles with a
 * valid range from 0 to 4194303.
 * @param attackDelay_us Delay time in microseconds before starting AGC after
 * starting Rx, with a valid range from 0 to 63.
 * @param slowLoopSettlingDelay Time in AGC clock cycles to allow gain
 * transients to settle before measurements, with a
 * range from 0 to 127.
 * @param lowThreshPreventGainInc Boolean to prevent gain index increment while
 * peak thresholds are exceeded.
 * @param changeGainIfThreshHigh Enable immediate gain change if high threshold
 * counter is exceeded, with specific bits for ULB
 * and HB thresholds.
 * @param agcMode Mode to enable gain change based on signal peak threshold
 * over-ranges, disabling power-based AGC changes.
 * @param resetOnRxon Boolean to reset the AGC slow loop state machine to max
 * gain when Rx Enable is taken low.
 * @param resetOnRxonGainIndex AGC Reset On gain index with a valid range from
 * minGainIndex to maxGainIndex.
 * @param enableSyncPulseForGainCounter Boolean to enable the AGC gain update
 * counter to sync to a time-slot boundary.
 * @param enableFastRecoveryLoop Boolean to enable multiple time constants in
 * AGC loop for fast attack and recovery.
 * @param power Structure for gain control power detection settings.
 * @param peak Structure for gain control peak detection settings.
 * @param extLna Structure for external LNA settings.
 * @param rxQecFreezeEnable Boolean to enable or disable RXQEC Freeze,
 * applicable only in AGC mode.
 * @param gpioFreezePin GPIO pin to activate to freeze AGC, set to 0/UNASSIGNED
 * if unused.
 ******************************************************************************/
typedef struct adi_adrv9001_GainControlCfg
{
    uint8_t peakWaitTime;           /*!< AGC peak wait time. Valid range is from 0 to 31 */
    uint8_t maxGainIndex;           /*!< AGC Rx max gain index. Valid range is from minGainIndex to 255 */
    uint8_t minGainIndex;           /*!< AGC Rx min gain index. Valid range is from 0 to maxGainIndex */
    /** AGC gain update time denominated in AGC clock cycles; Valid range is from 0 to 4194303. */
    uint32_t gainUpdateCounter;
    uint8_t attackDelay_us;         /*!< Delay time, denominated in microseconds, before starting AGC, after starting Rx. Valid range is from 0 to 63 */
    /** On any gain change, the AGC waits for the time (range 0 to 127) specified in AGC clock cycles to allow gain
     *  transients to flow through the Rx path before starting any measurements. */
    uint8_t slowLoopSettlingDelay;
    bool lowThreshPreventGainInc;   /*!< Prevent gain index from incrementing while peak thresholds are exceeded */
    /** Enable immediate gain change if high threshold counter is exceeded.
     *  Bit 0 enables ULB high threshold, Bit 1 enables HB high threshold */
    uint8_t changeGainIfThreshHigh;
    /** Enable gain change based only on the signal peak threshold over-ranges.
     *  Power-based AGC changes are disabled in this mode. */
    adi_adrv9001_RxGainControlDetectionMode_e agcMode;
    bool resetOnRxon;       /*!< Reset the AGC slow loop state machine to max gain when the Rx Enable is taken low */
    uint8_t resetOnRxonGainIndex;       /*!< AGC Reset On gain index. Valid range is from minGainIndex to maxGainIndex */
    bool enableSyncPulseForGainCounter; /*!< Enable the AGC gain update counter to be sync'ed to a time-slot boundary. */
    bool enableFastRecoveryLoop;        /*!< Enable multiple time constants in AGC loop for fast attack and fast recovery. */
    adi_adrv9001_PowerDetector_t power;
    adi_adrv9001_PeakDetector_t peak;
    adi_adrv9001_ExtLna_t extLna;
	bool rxQecFreezeEnable;		/*!< RXQEC Freeze Enable/Disable, only applies in AGC mode*/
	adi_adrv9001_GpioPin_e gpioFreezePin; /*!< GPIO pin to activate to freeze AGC - set to 0/UNASSIGNED if unused */
} adi_adrv9001_GainControlCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainControlPinCfg_t` structure is used to
 * configure the gain control mechanism for a receiver in the ADRV9001
 * transceiver. It defines the minimum and maximum gain indices, as well
 * as the step sizes for incrementing and decrementing the gain. The
 * structure also specifies the GPIO pins that will trigger these gain
 * changes on rising edges, ensuring that the gain remains within the
 * specified bounds. This configuration is crucial for managing the
 * dynamic range and performance of the receiver.
 *
 * @param minGainIndex Minimum gain index, must be >= gainTableMinGainIndex and
 * < maxGainIndex.
 * @param maxGainIndex Maximum gain index, must be > minGainIndex and <=
 * gainTableMaxGainIndex.
 * @param incrementStepSize Number of indices to increase gain on rising edge on
 * incrementPin (Range: 0 to 7).
 * @param decrementStepSize Number of indices to decrease gain on rising edge on
 * decrementPin (Range: 0 to 7).
 * @param incrementPin Pin that triggers gain increment by incrementStepSize on
 * a rising edge.
 * @param decrementPin Pin that triggers gain decrement by decrementStepSize on
 * a rising edge.
 ******************************************************************************/
typedef struct adi_adrv9001_RxGainControlPinCfg
{
    uint8_t minGainIndex;  //!< Minimum gain index. Must be >= gainTableMinGainIndex and < maxGainIndex
    uint8_t maxGainIndex;  //!< Maximum gain index. Must be > minGainIndex and <= gainTableMaxGainIndex
    uint8_t incrementStepSize;  //!< Number of indices to increase gain on rising edge on incrementPin (Range: 0 to 7)
    uint8_t decrementStepSize;  //!< Number of indices to decrease gain on rising edge on decrementPin (Range: 0 to 7)
    /** A rising edge on this pin will increment gain by incrementStepSize.
     *  \note Once gainIndex has reached maxGainIndex subsequent rising edges will not change the gainIndex */
    adi_adrv9001_GpioPin_e incrementPin;
    /** A rising edge on this pin will decrement gain by decrementStepSize.
     *  \note Once gainIndex has reached minGainIndex subsequent rising edges will not change the gainIndex */
    adi_adrv9001_GpioPin_e decrementPin;
} adi_adrv9001_RxGainControlPinCfg_t;

#endif /* _ADI_ADRV9001_GAINCONTROL_TYPES_H_ */
