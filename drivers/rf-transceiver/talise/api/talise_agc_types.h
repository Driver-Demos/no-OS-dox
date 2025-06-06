/**
 * \file talise_agc_types.h
 * \brief Contains Talise API AGC data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_AGC_TYPES_H_
#define TALISE_AGC_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseAgcPeak_t` structure is designed to hold various settings
 * related to the Automatic Gain Control (AGC) peak detection in a Talise
 * radio system. It includes parameters for configuring update intervals,
 * thresholds, and gain steps for both APD (Automatic Peak Detection) and
 * HB2 (High Band 2) components. The structure allows for fine-tuning of
 * the AGC behavior by setting thresholds for high and low gain modes,
 * configuring overload detection, and adjusting gain steps for attack
 * and recovery phases. This structure is crucial for optimizing the AGC
 * performance to maintain signal integrity and prevent overloads in the
 * radio system.
 *
 * @param agcUnderRangeLowInterval_ns Update interval for AGC loop mode in
 * nanoseconds.
 * @param agcUnderRangeMidInterval 2nd update interval for multiple time
 * constant AGC mode, calculated as (agcUnderRan
 * geMidInterval+1)*agcUnderRangeLowInterval_ns.
 * @param agcUnderRangeHighInterval 3rd update interval for multiple time
 * constant AGC mode, calculated as
 * (agcUnderRangeHighInterval+1)*2nd update
 * interval.
 * @param apdHighThresh AGC APD high threshold with a valid range of 7 to 49.
 * @param apdLowGainModeHighThresh AGC APD high threshold in low gain mode,
 * recommended to be 3dB above apdHighThresh.
 * @param apdLowThresh AGC APD low threshold, recommended to be 3dB below
 * apdHighThresh.
 * @param apdLowGainModeLowThresh AGC APD low threshold in low gain mode,
 * recommended to be 3dB above apdLowThresh.
 * @param apdUpperThreshPeakExceededCnt AGC APD peak detect upper threshold
 * count with a valid range of 0 to 255.
 * @param apdLowerThreshPeakExceededCnt AGC APD peak detect lower threshold
 * count with a valid range of 0 to 255.
 * @param apdGainStepAttack AGC APD peak detect attack gain step with a valid
 * range of 0 to 31.
 * @param apdGainStepRecovery AGC APD gain index step size for recovery with a
 * valid range of 0 to 31.
 * @param enableHb2Overload Enables or disables the HB2 overload detector.
 * @param hb2OverloadDurationCnt Sets the window of clock cycles at the HB2
 * output rate to meet the overload count.
 * @param hb2OverloadThreshCnt Sets the number of actual overloads required to
 * trigger the overload signal.
 * @param hb2HighThresh AGC HB2 output high threshold with a valid range from 0
 * to 255.
 * @param hb2UnderRangeLowThresh AGC HB2 output low threshold with a valid range
 * from 0 to 255.
 * @param hb2UnderRangeMidThresh AGC HB2 output low threshold for 2nd interval
 * for multiple time constant AGC mode.
 * @param hb2UnderRangeHighThresh AGC HB2 output low threshold for 3rd interval
 * for multiple time constant AGC mode.
 * @param hb2UpperThreshPeakExceededCnt AGC HB2 output upper threshold count
 * with a valid range from 0 to 255.
 * @param hb2LowerThreshPeakExceededCnt AGC HB2 output lower threshold count
 * with a valid range from 0 to 255.
 * @param hb2GainStepHighRecovery AGC HB2 gain index step size with a valid
 * range from 0 to 31.
 * @param hb2GainStepLowRecovery AGC HB2 gain index step size when the HB2 Low
 * Overrange interval 2 triggers.
 * @param hb2GainStepMidRecovery AGC HB2 gain index step size when the HB2 Low
 * Overrange interval 3 triggers.
 * @param hb2GainStepAttack AGC HB2 output attack gain step with a valid range
 * from 0 to 31.
 * @param hb2OverloadPowerMode Increases the dynamic range of the power
 * measurement from -40dB to ~-60dB when set.
 * @param hb2OvrgSel Used in fast recovery mode to enable decimated data
 * overload detection.
 * @param hb2ThreshConfig Not user modifiable, initialized to 0x03.
 * @param hb2UnderRangeLowThreshExceededCnt AGC HB2 low overrange interval 0
 * threshold count, optional with a
 * valid range from 1 to 255.
 * @param hb2UnderRangeMidThreshExceededCnt AGC HB2 mid overrange interval 1
 * threshold count, optional with a
 * valid range from 1 to 255.
 ******************************************************************************/
typedef struct {
	uint32_t
	agcUnderRangeLowInterval_ns;        /*!< Update interval for AGC loop mode in nanoseconds */
	uint8_t
	agcUnderRangeMidInterval;           /*!< 2nd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeMidInterval+1)*agcUnderRangeLowInterval_ns. Valid range is 0 to 63 */
	uint8_t
	agcUnderRangeHighInterval;          /*!< 3rd update interval for multiple time constant AGC mode. Calculated as (agcUnderRangeHighInterval+1)*2nd update interval. Valid range is 0 to 63 */
	uint8_t
	apdHighThresh;                      /*!< AGC APD high threshold. Valid range is 7 to 49 */
	uint8_t
	apdLowGainModeHighThresh;           /*!< AGC APD high threshold in low gain mode. Valid range is 7 to 49. Recommended to be 3dB above apdHighThresh */
	uint8_t
	apdLowThresh;                       /*!< AGC APD low threshold. Valid range is 7 to 49. Recommended to be 3dB below apdHighThresh */
	uint8_t
	apdLowGainModeLowThresh;            /*!< AGC APD low threshold in low gain mode. Valid range is 7 to 49. Recommended to be 3dB above apdLowThresh */
	uint8_t
	apdUpperThreshPeakExceededCnt;      /*!< AGC APD peak detect upper threshold count. Valid range is 0 to 255 */
	uint8_t
	apdLowerThreshPeakExceededCnt;      /*!< AGC APD peak detect lower threshold count. Valid range is 0 to 255 */
	uint8_t
	apdGainStepAttack;                  /*!< AGC APD peak detect attack gain step. Valid range is 0 to 31 */
	uint8_t
	apdGainStepRecovery;                /*!< AGC APD gain index step size for recovery. Valid range is 0 to 31 */
	uint8_t
	enableHb2Overload;                  /*!< Enable or disables the HB2 overload detector. */
	uint8_t
	hb2OverloadDurationCnt;             /*!< Sets the window of clock cycles (at the HB2 output rate) to meet the overload count. (0 = 2 cycles, 1 = 4 cycles, 2 = 8 cycles, 3 = 12 cycles, 4 = 16 cycles, 5 = 24 cycles, 6 = 32 cycles) */
	uint8_t
	hb2OverloadThreshCnt;               /*!< Sets the number of actual overloads required to trigger the overload signal. Valid range from 1 to 15 */
	uint8_t
	hb2HighThresh;                      /*!< AGC HB2 output high threshold. Valid range from  0 to 255 */
	uint8_t
	hb2UnderRangeLowThresh;             /*!< AGC HB2 output low threshold. Valid range from  0 to 255 */
	uint8_t
	hb2UnderRangeMidThresh;             /*!< AGC HB2 output low threshold for 2nd interval for multiple time constant AGC mode. Valid range from  0 to 255 */
	uint8_t
	hb2UnderRangeHighThresh;            /*!< AGC HB2 output low threshold for 3rd interval for multiple time constant AGC mode. Valid range from  0 to 255 */
	uint8_t
	hb2UpperThreshPeakExceededCnt;      /*!< AGC HB2 output upper threshold count. Valid range from  0 to 255 */
	uint8_t
	hb2LowerThreshPeakExceededCnt;      /*!< AGC HB2 output lower threshold count. Valid range from  0 to 255 */
	uint8_t
	hb2GainStepHighRecovery;            /*!< AGC HB2 gain index step size. Valid range from  0 to 31 */
	uint8_t
	hb2GainStepLowRecovery;             /*!< AGC HB2 gain index step size, when the HB2 Low Overrange interval 2 triggers. Valid range from  0 to 31 */
	uint8_t
	hb2GainStepMidRecovery;             /*!< AGC HB2 gain index step size, when the HB2 Low Overrange interval 3 triggers. Valid range from  0 to 31 */
	uint8_t
	hb2GainStepAttack;                  /*!< AGC HB2 output attack gain step. Valid range from  0 to 31 */
	uint8_t
	hb2OverloadPowerMode;               /*!< When this bit is set, the dynamic range of the power measurement increases from -40dB to ~-60dB (that is, all signal levels from 0dBFS to -60dBFS are accurately detected */
	uint8_t
	hb2OvrgSel;                         /*!< To be used in fast recovery mode. Clearing this bit enables the decimated data overload detection functionality */
	uint8_t
	hb2ThreshConfig;                    /*!< Not User Modifiable   Initialized to 0x03 */
	uint8_t
	hb2UnderRangeLowThreshExceededCnt;  /*!< AGC HB2 low overrange interval 0 threshold count. Optional. Valid range from 1 to 255. Passing 0 will result in the reset value of 3. */
	uint8_t
	hb2UnderRangeMidThreshExceededCnt;  /*!< AGC HB2 mid overrange interval 1 threshold count. Optional. Valid range from 1 to 255. Passing 0 will result in the reset value of 3. */

} taliseAgcPeak_t;

/***************************************************************************//**
 * @brief The `taliseAgcPower_t` structure is designed to hold configuration
 * settings for automatic gain control (AGC) power measurements in a
 * Talise radio system. It includes various parameters to enable and
 * configure power measurement blocks, thresholds for detecting power
 * levels, and settings for gain step recovery and attack. The structure
 * allows for detailed control over the power measurement process,
 * including the use of specific outputs for measurement, setting
 * thresholds for under and over-range power detection, and configuring
 * the duration and delay of power measurements for different receiver
 * channels. This structure is essential for fine-tuning the AGC behavior
 * to ensure optimal performance in varying signal conditions.
 *
 * @param powerEnableMeasurement Enable the Rx power measurement block. (0/1)
 * @param powerUseRfirOut Use output of Rx PFIR for power measurement. (0/1)
 * @param powerUseBBDC2 Use output of DC offset block for power measurement.
 * (0/1)
 * @param underRangeHighPowerThresh AGC power measurement detect lower 0
 * threshold. Valid Range from 0 to 127.
 * @param underRangeLowPowerThresh AGC power measurement detect lower 1
 * threshold. Valid offset from 0 to 31
 * @param underRangeHighPowerGainStepRecovery AGC power measurement detect lower
 * 0 recovery gain step. Valid range
 * from 0 to 31
 * @param underRangeLowPowerGainStepRecovery AGC power measurement detect lower
 * 1 recovery gain step. Valid range
 * from 0 to 31
 * @param powerMeasurementDuration Average power measurement duration =
 * 8*2^powerMeasurementDuration. Valid range
 * from 0 to 31
 * @param rx1TddPowerMeasDuration Measurement duration to detect power for
 * specific slice of the gain update counter.
 * @param rx1TddPowerMeasDelay Measurement delay to detect power for specific
 * slice of the gain update counter.
 * @param rx2TddPowerMeasDuration Measurement duration to detect power for
 * specific slice of the gain update counter.
 * @param rx2TddPowerMeasDelay Measurement delay to detect power for specific
 * slice of the gain update counter.
 * @param upper0PowerThresh AGC upper 0 (overRangeHighPowerThreshold) threshold
 * for power measurement. Valid Range from 0 to 127.
 * @param upper1PowerThresh AGC upper 1 (overRangeLowPowerThreshold) threshold
 * for power measurement. Valid offset from 0 to 15
 * @param powerLogShift Enable Increase in dynamic range of the power
 * measurement from 40dB to ~60dB. Provides higher
 * accuracy.
 * @param overRangeLowPowerGainStepAttack AGC inner upper threshold exceeded
 * attack gain step. Optional. Valid
 * range from 1 to 31. Passing 0 will
 * result in the reset value of 4.
 * @param overRangeHighPowerGainStepAttack AGC outer high power threshold
 * exceeded attack gain step. Optional.
 * Valid range from 1 to 31. Passing 0
 * will result in the reset value of 4.
 ******************************************************************************/
typedef struct {
	uint8_t
	powerEnableMeasurement;                 /*!< Enable the Rx power measurement block. (0/1) */
	uint8_t
	powerUseRfirOut;                        /*!< Use output of Rx PFIR for power measurement. (0/1) */
	uint8_t
	powerUseBBDC2;                          /*!< Use output of DC offset block for power measurement. (0/1) */
	uint8_t
	underRangeHighPowerThresh;              /*!< AGC power measurement detect lower 0 threshold. Valid Range from 0 to 127. */
	uint8_t
	underRangeLowPowerThresh;               /*!< AGC power measurement detect lower 1 threshold. Valid offset from 0 to 31 */
	uint8_t
	underRangeHighPowerGainStepRecovery;    /*!< AGC power measurement detect lower 0 recovery gain step. Valid range from  0 to 31 */
	uint8_t
	underRangeLowPowerGainStepRecovery;     /*!< AGC power measurement detect lower 1 recovery gain step. Valid range from  0 to 31 */
	uint8_t
	powerMeasurementDuration;               /*!< Average power measurement duration = 8*2^powerMeasurementDuration. Valid range from 0 to 31 */
	uint16_t
	rx1TddPowerMeasDuration;                /*!< Measurement duration to detect power for specific slice of the gain update counter. */
	uint16_t
	rx1TddPowerMeasDelay;                   /*!< Measurement delay to detect power for specific slice of the gain update counter. */
	uint16_t
	rx2TddPowerMeasDuration;                /*!< Measurement duration to detect power for specific slice of the gain update counter. */
	uint16_t
	rx2TddPowerMeasDelay;                   /*!< Measurement delay to detect power for specific slice of the gain update counter. */
	uint8_t
	upper0PowerThresh;                      /*!< AGC upper 0 (overRangeHighPowerThreshold) threshold for power measurement. Valid Range from 0 to 127.*/
	uint8_t
	upper1PowerThresh;                      /*!< AGC upper 1 (overRangeLowPowerThreshold)  threshold for power measurement. Valid offset from 0 to 15 */
	uint8_t
	powerLogShift;                          /*!< Enable Increase in dynamic range of the power measurement from 40dB to ~60dB. Provides higher accuracy.
                                                         *   NOTE: If you input a signal below -60dBFS while enabled, TALISE_getRxDecPower() may return an incorrect reading of 0dBFS.
                                                         *   When disabled, TALISE_getRxDecPower() will bottom out around -40dBFS. */
	uint8_t
	overRangeLowPowerGainStepAttack;        /*!< AGC inner upper threshold exceeded attack gain step. Optional. Valid range from  1 to 31. Passing 0 will result in the reset value of 4. */
	uint8_t
	overRangeHighPowerGainStepAttack;       /*!< AGC outer high power threshold exceeded attack gain step. Optional. Valid range from  1 to 31. Passing 0 will result in the reset value of 4. */

} taliseAgcPower_t;

/***************************************************************************//**
 * @brief The `taliseAgcCfg_t` structure is designed to configure the Automatic
 * Gain Control (AGC) settings for the Talise API, specifically for
 * managing gain indices and thresholds for Rx1 and Rx2 channels. It
 * includes parameters for peak wait time, maximum and minimum gain
 * indices, gain update timing, attack delays, and various threshold
 * controls. Additionally, it incorporates settings for fast recovery
 * loops and synchronization options. The structure also contains two
 * nested structures, `taliseAgcPower_t` and `taliseAgcPeak_t`, which
 * manage power and peak settings respectively. Some fields are marked as
 * ignored, indicating they are not used in certain silicon versions.
 *
 * @param agcPeakWaitTime AGC peak wait time with a valid range from 0 to 31.
 * @param agcRx1MaxGainIndex AGC Rx1 maximum gain index with a valid range from
 * 0 to 255.
 * @param agcRx1MinGainIndex AGC Rx1 minimum gain index with a valid range from
 * 0 to 255.
 * @param agcRx2MaxGainIndex AGC Rx2 maximum gain index with a valid range from
 * 0 to 255.
 * @param agcRx2MinGainIndex AGC Rx2 minimum gain index with a valid range from
 * 0 to 255.
 * @param agcGainUpdateCounter_us AGC gain update time in microseconds.
 * @param agcRx1AttackDelay Delay period for Rx1 AGC inactivity upon entering
 * Rx, measured in microseconds.
 * @param agcRx2AttackDelay Delay period for Rx2 AGC inactivity upon entering
 * Rx, measured in microseconds.
 * @param agcSlowLoopSettlingDelay Delay in AGC clock cycles to allow gain
 * transients to settle before measurements.
 * @param agcLowThreshPreventGain Prevents gain index increment if peak
 * thresholds are exceeded.
 * @param agcChangeGainIfThreshHigh Enables immediate gain change if high
 * threshold counter is exceeded.
 * @param agcPeakThreshGainControlMode Enables gain change based solely on
 * signal peak threshold over-ranges.
 * @param agcResetOnRxon Resets the AGC slow loop state machine to max gain when
 * Rx Enable is low.
 * @param agcEnableSyncPulseForGainCounter Enables synchronization of the AGC
 * gain update counter to a time-slot
 * boundary.
 * @param agcEnableIp3OptimizationThresh (Ignored) Enables two-threshold AGC
 * loop mode to improve IIP3.
 * @param ip3OverRangeThresh (Ignored) Overload threshold for lower peak signal
 * level, recommended at -17dBFS.
 * @param ip3OverRangeThreshIndex (Ignored) Gain index to jump to if current
 * gain causes IP3 overload, recommended at 246.
 * @param ip3PeakExceededCnt (Ignored) Configures the number of ADC IP3
 * Overrange threshold triggers before a gain change.
 * @param agcEnableFastRecoveryLoop Enables multiple time constants in AGC loop
 * for fast attack and recovery.
 * @param agcPower Holds AGC power settings.
 * @param agcPeak Holds AGC peak settings.
 ******************************************************************************/
typedef struct {
	uint8_t
	agcPeakWaitTime;                    /*!< AGC peak wait time. Valid range is from 0 to 31 */
	uint8_t
	agcRx1MaxGainIndex;                 /*!< AGC Rx1 max gain index. Valid range is from 0 to 255 */
	uint8_t
	agcRx1MinGainIndex;                 /*!< AGC Rx1 min gain index. Valid range is from 0 to 255 */
	uint8_t
	agcRx2MaxGainIndex;                 /*!< AGC Rx2 max gain index. Valid range is from 0 to 255 */
	uint8_t
	agcRx2MinGainIndex;                 /*!< AGC Rx2 min gain index. Valid range is from 0 to 255 */
	uint32_t
	agcGainUpdateCounter_us;            /*!< AGC gain update time in micro seconds */
	uint8_t
	agcRx1AttackDelay;                  /*!< On entering Rx, the Rx1 AGC is kept inactive for a period = agcRx1AttackDelay*1us */
	uint8_t
	agcRx2AttackDelay;                  /*!< On entering Rx, the Rx2 AGC is kept inactive for a period = agcRx2AttackDelay*1us */
	uint8_t
	agcSlowLoopSettlingDelay;           /*!< On any gain change, the AGC waits for the time (range 0 to 127) specified in AGC clock cycles to allow gain transients to flow through the Rx path before starting any measurements. */
	uint8_t
	agcLowThreshPreventGain;            /*!< Prevent gain index from incrementing if peak thresholds are being exceeded */
	uint8_t
	agcChangeGainIfThreshHigh;          /*!< Enable immediate gain change if high threshold counter is exceeded. Bit 0 enables ULB high threshold, Bit 1 enables HB2 high threshold */
	uint8_t
	agcPeakThreshGainControlMode;       /*!< Enable gain change based only on the signal peak threshold over-ranges. Power based AGC changes are disabled in this mode. */
	uint8_t
	agcResetOnRxon;                     /*!< Reset the AGC slow loop state machine to max gain when the Rx Enable is taken low */
	uint8_t
	agcEnableSyncPulseForGainCounter;   /*!< Enable the AGC gain update counter to be sync'ed to a time-slot boundary. */
	uint8_t
	agcEnableIp3OptimizationThresh;     /*!< (API disables feature, this member is ignored) Enable the two-threshold AGC loop mode.To improve IIP3. Enable=1, Disable=0 (set to 0 for Talise B1 and C0 silicon - this feature is not supported) */
	uint8_t
	ip3OverRangeThresh;                 /*!< (API disables feature, this member is ignored) Overload threshold that triggers for a lower peak signal level. Recommended to be set to -17dBFS */
	uint8_t
	ip3OverRangeThreshIndex;            /*!< (API disables feature, this member is ignored) Gain index to jump to if current gain causes IP3 overload. Recommended to be set to 246. Valid range 0 to 255 */
	uint8_t
	ip3PeakExceededCnt;                 /*!< (API disables feature, this member is ignored) Configures the number of times the ADC IP3 Overrange threshold is triggered within one gain update interval before a gain change is mandated by the gain control loop. */
	uint8_t
	agcEnableFastRecoveryLoop;          /*!< Enable multiple time constants in AGC loop for fast attack and fast recovery. */

	taliseAgcPower_t agcPower;
	taliseAgcPeak_t agcPeak;

} taliseAgcCfg_t;

/***************************************************************************//**
 * @brief The `taliseAgcDualBandCfg_t` structure is designed to configure the
 * Automatic Gain Control (AGC) for dual-band receivers. It includes
 * settings to enable dual-band AGC operation, define gain table indices
 * for external LNA control, and set power margins and thresholds for
 * band power comparison. The structure also allows enabling GPIOs for
 * external LNA control and specifies the duration for power measurement
 * of individual bands. This configuration is crucial for optimizing the
 * AGC performance in systems with dual-band reception, ensuring
 * efficient gain control and power management.
 *
 * @param agcDualBandEnable Enable AGC operation for dualband receiver.
 * @param agcRxDualbandExtTableUpperIndex Indicates the gain table index below
 * which the AGC prioritizes decreasing
 * gain through external LNA control over
 * the Front-end gain.
 * @param agcRxDualbandExtTableLowerIndex Indicates the gain table index above
 * which the AGC prioritizes increasing
 * gain through external LNA control over
 * the Front-end gain.
 * @param agcDualbandPwrMargin Margin for comparing total power against power of
 * individual bands, in 0.5db steps.
 * @param agcDualbandLnaStep Margin to compare Upper band power versus Lower
 * band power, in 0.5db resolution.
 * @param agcDualbandHighLnaThreshold High threshold for Upper band or Lower
 * band power above which the LNA index is
 * decreased, in 0.5db.
 * @param agcDualbandLowLnaThreshold Low threshold for Upper band or Lower band
 * power below which the LNA index is
 * increased, in 0.5db.
 * @param dualBandGpioEnable Enable the 3.3V GPIO's used to drive the external
 * LNA's.
 * @param decPowerDdcMeasurementDuration Power measurement duration for
 * measuring the power of the individual
 * bands, with a range of 0 to 31.
 ******************************************************************************/
typedef struct {
	uint8_t agcDualBandEnable;                  /*!< Enable AGC operation for dualband receiver  */
	uint8_t agcRxDualbandExtTableUpperIndex;    /*!< Rx1/2 AGC dual band operation - Indicates the gain table index below which the AGC prioritizes
                                                     decreasing gain through external LNA control over the Front-end gain  */
	uint8_t agcRxDualbandExtTableLowerIndex;    /*!< Rx1/2 AGC dual band operation - Indicates the gain table index above which the AGC prioritizes
                                                     increasing gain through external LNA control over the Front-end gain  */
	uint8_t agcDualbandPwrMargin;               /*!< Margin for comparing total power against power of individual bands. If
                                                     Total power > Upper Band Power + Lower Band Power +margin, the signal contains other components
                                                     than the two bands, and AGC should behave like a single band system. Margin is in 0.5db steps. */
	uint8_t agcDualbandLnaStep;                 /*!< Margin to compare Upper band power versus Lower band power
                                                     (for Upper Band Power > Lower Band Power + margin, and Lower Band Power > Upper Band Power + margin checks).
                                                     The margin compares the powers of the bands to change the LNA of one band so that powers of the bands match.
                                                     Value is in 0.5db resolution  */
	uint8_t agcDualbandHighLnaThreshold;        /*!< High threshold for Upper band or Lower band power above which the LNA index is decreased.
                                                     Value is in 0.5db */
	uint8_t agcDualbandLowLnaThreshold;         /*!< Low threshold for Upper band or Lower band power below which the LNA index is increased.
                                                     Value is in 0.5db */
	uint8_t dualBandGpioEnable;                 /*!< Enable the 3.3V GPIO's that would be used to drive the external LNA's.
                                                     Rx1 : GPIO3.3[1:0] controls Lower band LNA, GPIO3.3[3:2] controls Upper band LNA
                                                     Rx2 : GPIO3.3[5:4] controls Lower band LNA, GPIO3.3[7:6] controls Upper band LNA
                                                     If disabled, user needs to use TALISE_getDualBandLnaControls() to read back the LNA control
                                                     value through the SPI. */
	uint8_t decPowerDdcMeasurementDuration;     /*!< Power measurement duration for measuring the power of the individual bands. This variable has
                                                     a range of 0 to 31. The sampling period is calculated as = 8 x 2^decPowerDdcMeasurementDuration. */
} taliseAgcDualBandCfg_t;

/***************************************************************************//**
 * @brief The `taliseDualBandLnaControls_t` structure is designed to manage the
 * control values for the lower and upper band Low Noise Amplifiers
 * (LNAs) in a dual-band receiver system. It contains two members, each
 * represented as an 8-bit unsigned integer, which specify the control
 * settings for the lower and upper band LNAs, respectively. These
 * control values range from 0 to 3, allowing for different
 * configurations of the LNA settings to optimize signal reception in
 * dual-band applications.
 *
 * @param rxLowerBandLnaControl The control value for the Rx1/2 Lower band LNA
 * (Values 0-3).
 * @param rxUpperBandLnaControl The control value for the Rx1/2 Upper band LNA
 * (Values 0-3).
 ******************************************************************************/
typedef struct {
	uint8_t rxLowerBandLnaControl;         /* The control value for the Rx1/2 Lower band LNA (Values 0-3) */
	uint8_t rxUpperBandLnaControl;         /* The control value for the Rx1/2 Upper band LNA (Values 0-3) */
} taliseDualBandLnaControls_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_AGC_TYPES_H_ */
