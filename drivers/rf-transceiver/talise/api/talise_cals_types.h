/**
 * \file talise_cals_types.h
 * \brief Contains Talise API Calibration data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_CALS_TYPES_H_
#define TALISE_CALS_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseInitCalibrations_t` is an enumeration that defines a set of
 * calibration types used in the Talise API for configuring and
 * initializing various calibration processes in RF systems. Each
 * enumerator represents a specific calibration task, such as tuning
 * filters, correcting offsets, or adjusting delays, which are essential
 * for optimizing the performance of RF components like transmitters and
 * receivers. The enumerators are associated with unique bitmask values,
 * allowing them to be combined using bitwise operations to enable
 * multiple calibrations simultaneously.
 *
 * @param TAL_TX_BB_FILTER Tx BB filter calibration.
 * @param TAL_ADC_TUNER ADC tuner calibration.
 * @param TAL_TIA_3DB_CORNER TIA 3dB corner calibration.
 * @param TAL_DC_OFFSET DC offset calibration.
 * @param TAL_TX_ATTENUATION_DELAY Tx attenuation delay calibration.
 * @param TAL_RX_GAIN_DELAY Rx gain delay calibration.
 * @param TAL_FLASH_CAL Flash converter comparator calibration.
 * @param TAL_PATH_DELAY Path delay equalization calibration.
 * @param TAL_TX_LO_LEAKAGE_INTERNAL Internal Tx LO leakage calibration.
 * @param TAL_TX_LO_LEAKAGE_EXTERNAL External Tx LO leakage calibration.
 * @param TAL_TX_QEC_INIT Tx quadrature error correction calibration.
 * @param TAL_LOOPBACK_RX_LO_DELAY Loopback Rx LO delay path calibration.
 * @param TAL_LOOPBACK_RX_RX_QEC_INIT Loopback Rx quadrature error correction
 * calibration.
 * @param TAL_RX_LO_DELAY Rx LO delay path calibration.
 * @param TAL_RX_QEC_INIT Rx quadrature error correction calibration.
 * @param TAL_RX_PHASE_CORRECTION Rx Phase correction calibration.
 * @param TAL_ORX_LO_DELAY ORx LO delay path calibration.
 * @param TAL_ORX_QEC_INIT ORx quadrature error correction calibration.
 * @param TAL_TX_DAC Tx DAC passband calibration.
 * @param TAL_ADC_STITCHING ADC stitching calibration.
 * @param TAL_FHM_CALS FHM (Fast Frequency Hopping Mode) Calibrations.
 ******************************************************************************/
typedef enum {
	TAL_TX_BB_FILTER            = 0x00000001,   /*!< Tx BB filter calibration */
	TAL_ADC_TUNER               = 0x00000002,   /*!< ADC tuner calibration */
	TAL_TIA_3DB_CORNER          = 0x00000004,   /*!< TIA 3dB corner calibration */
	TAL_DC_OFFSET               = 0x00000008,   /*!< DC offset calibration */
	TAL_TX_ATTENUATION_DELAY    = 0x00000010,   /*!< Tx attenuation delay calibration */
	TAL_RX_GAIN_DELAY           = 0x00000020,   /*!< Rx gain delay calibration */
	TAL_FLASH_CAL               = 0x00000040,   /*!< Flash converter comparator calibration */
	TAL_PATH_DELAY              = 0x00000080,   /*!< Path delay equalization calibration */
	TAL_TX_LO_LEAKAGE_INTERNAL  = 0x00000100,   /*!< Internal Tx LO leakage calibration */
	TAL_TX_LO_LEAKAGE_EXTERNAL  = 0x00000200,   /*!< External Tx LO leakage calibration */
	TAL_TX_QEC_INIT             = 0x00000400,   /*!< Tx quadrature error correction calibration */
	TAL_LOOPBACK_RX_LO_DELAY    = 0x00000800,   /*!< Loopback Rx LO delay path calibration */
	TAL_LOOPBACK_RX_RX_QEC_INIT = 0x00001000,   /*!< Loopback Rx quadrature error correction calibration */
	TAL_RX_LO_DELAY             = 0x00002000,   /*!< Rx LO delay path calibration */
	TAL_RX_QEC_INIT             = 0x00004000,   /*!< Rx quadrature error correction calibration */
	TAL_RX_PHASE_CORRECTION     = 0x00008000,   /*!< Rx Phase correction calibration */
	TAL_ORX_LO_DELAY            = 0x00010000,   /*!< ORx LO delay path calibration */
	TAL_ORX_QEC_INIT            = 0x00020000,   /*!< ORx quadrature error correction calibration */
	TAL_TX_DAC                  = 0x00040000,   /*!< Tx DAC passband calibration */
	TAL_ADC_STITCHING           = 0x00080000,   /*!< ADC stitching calibration */
	TAL_FHM_CALS                = 0x00800000    /*!< FHM (Fast Frequency Hopping Mode) Calibrations */
} taliseInitCalibrations_t;

/***************************************************************************//**
 * @brief The `taliseTrackingCalibrations_t` is an enumeration that defines
 * various tracking calibration options for the Talise API. Each
 * enumerator represents a specific type of calibration, such as
 * quadrature error correction (QEC) for different receiver and
 * transmitter paths, local oscillator leakage (LOL) correction, and HD2
 * error correction. The enumeration provides a way to enable or disable
 * specific tracking calibrations, or to enable all available
 * calibrations using the `TAL_TRACK_ALL` option.
 *
 * @param TAL_TRACK_NONE Disable all tracking calibrations.
 * @param TAL_TRACK_RX1_QEC Rx1 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_RX2_QEC Rx2 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_ORX1_QEC ORx1 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_ORX2_QEC ORx2 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_TX1_LOL Tx1 LO leakage tracking calibration.
 * @param TAL_TRACK_TX2_LOL Tx2 LO leakage tracking calibration.
 * @param TAL_TRACK_TX1_QEC Tx1 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_TX2_QEC Tx2 quadrature error correction tracking
 * calibration.
 * @param TAL_TRACK_RX1_HD2 Rx1 HD2 error correction tracking calibration.
 * @param TAL_TRACK_RX2_HD2 Rx2 HD2 error correction tracking calibration.
 * @param TAL_TRACK_ALL ENUM specifies all tracking cals.
 ******************************************************************************/
typedef enum {
	TAL_TRACK_NONE              = 0x00000000,   /*!< Disable all tracking calibrations */
	TAL_TRACK_RX1_QEC           = 0x00000001,   /*!< Rx1 quadrature error correction tracking calibration */
	TAL_TRACK_RX2_QEC           = 0x00000002,   /*!< Rx2 quadrature error correction tracking calibration */
	TAL_TRACK_ORX1_QEC          = 0x00000004,   /*!< ORx1 quadrature error correction tracking calibration */
	TAL_TRACK_ORX2_QEC          = 0x00000008,   /*!< ORx2 quadrature error correction tracking calibration */
	TAL_TRACK_TX1_LOL           = 0x00000010,   /*!< Tx1 LO leakage tracking calibration */
	TAL_TRACK_TX2_LOL           = 0x00000020,   /*!< Tx2 LO leakage tracking calibration */
	TAL_TRACK_TX1_QEC           = 0x00000040,   /*!< Tx1 quadrature error correction tracking calibration */
	TAL_TRACK_TX2_QEC           = 0x00000080,   /*!< Tx2 quadrature error correction tracking calibration */
	TAL_TRACK_RX1_HD2           = 0x00000100,   /*!< Rx1 HD2 error correction tracking calibration */
	TAL_TRACK_RX2_HD2           = 0x00000200,   /*!< Rx2 HD2 error correction tracking calibration */
	TAL_TRACK_ALL               = 0x000003FF    /*!< ENUM specifies all tracking cals */
} taliseTrackingCalibrations_t;

/***************************************************************************//**
 * @brief The `taliseWaitEvent_t` is an enumeration that defines a set of
 * possible wait events used with the `TALISE_waitForEvent()` function.
 * These events are related to various phase-locked loops (PLLs) and ARM
 * processor states within the Talise API, which is used for managing
 * calibration processes in radio frequency systems. Each enumerator
 * represents a specific event that the system can wait for, such as PLL
 * lock states or ARM processor busy states, facilitating synchronization
 * and control in the calibration workflow.
 *
 * @param TAL_CLKPLLCP CLK PLL CP wait event.
 * @param TAL_CLKPLL_LOCK CLK PLL lock wait event.
 * @param TAL_RFPLLCP RF PLL CP wait event.
 * @param TAL_RFPLL_LOCK RF PLL lock wait event.
 * @param TAL_AUXPLLCP AUX PLL CP wait event.
 * @param TAL_AUXPLL_LOCK AUX PLL LOCK wait event.
 * @param TAL_ARMBUSY ARM busy wait event.
 ******************************************************************************/
typedef enum {
	TAL_CLKPLLCP = 0,               /*!< CLK PLL CP wait event */
	TAL_CLKPLL_LOCK,                /*!< CLK PLL lock wait event */
	TAL_RFPLLCP,                    /*!< RF PLL CP wait event */
	TAL_RFPLL_LOCK,                 /*!< RF PLL lock wait event */
	TAL_AUXPLLCP,                   /*!< AUX PLL CP wait event */
	TAL_AUXPLL_LOCK,                /*!< AUX PLL LOCK wait event */
	TAL_ARMBUSY                     /*!< ARM busy wait event */
} taliseWaitEvent_t;

/***************************************************************************//**
 * @brief The `taliseDcOffsetChannels_t` is an enumeration that defines the
 * possible digital DC offset channels for the Talise API. It includes
 * two members, `TAL_DC_OFFSET_RX_CHN` and `TAL_DC_OFFSET_ORX_CHN`, which
 * are used to specify the Rx and ORx channels, respectively, for DC
 * offset calibration purposes. This enum is part of the Talise API
 * calibration data types, which are used to configure and manage various
 * calibration processes in the Talise transceiver system.
 *
 * @param TAL_DC_OFFSET_RX_CHN Represents the Rx channel for DC offset.
 * @param TAL_DC_OFFSET_ORX_CHN Represents the ORx channel for DC offset.
 ******************************************************************************/
typedef enum {
	TAL_DC_OFFSET_RX_CHN = 0,
	TAL_DC_OFFSET_ORX_CHN = 1
} taliseDcOffsetChannels_t;

/***************************************************************************//**
 * @brief The `taliseRxDcOffsettEn_t` is an enumeration that defines various
 * states for enabling or disabling DC offset correction on different
 * receiver (Rx) and observation receiver (ORx) channels. It provides
 * options to individually enable Rx1, Rx2, ORx1, ORx2, or to
 * enable/disable all channels simultaneously. This enum is used to
 * configure the DC offset settings in the Talise API, which is part of
 * the calibration process for RF systems.
 *
 * @param TAL_DC_OFFSET_ALL_OFF Disable all the channels.
 * @param TAL_DC_OFFSET_RX1 Enables Rx1.
 * @param TAL_DC_OFFSET_RX2 Enables Rx2.
 * @param TAL_DC_OFFSET_ORX1 Enables ORx1.
 * @param TAL_DC_OFFSET_ORX2 Enables ORx2.
 * @param TAL_DC_OFFSET_ALL_ON Enables all the channels.
 ******************************************************************************/
typedef enum {
	TAL_DC_OFFSET_ALL_OFF = 0x00,               /*!< Disable all the channels */
	TAL_DC_OFFSET_RX1 = 0x01,                   /*!< Enables Rx1  */
	TAL_DC_OFFSET_RX2 = 0x02,                   /*!< Enables Rx2  */
	TAL_DC_OFFSET_ORX1 = 0x04,                   /*!< Enables ORx1  */
	TAL_DC_OFFSET_ORX2 = 0x08,                   /*!< Enables ORx2  */
	TAL_DC_OFFSET_ALL_ON = 0x0F              /*!< Enables all the channels  */
} taliseRxDcOffsettEn_t;

/***************************************************************************//**
 * @brief The `taliseTrackingCalBatchSize_t` is an enumeration that defines the
 * possible batch sizes for tracking calibration in microseconds. It
 * provides two options: 500 microseconds and 200 microseconds, allowing
 * the user to select the appropriate batch size for their calibration
 * needs. This enumeration is part of the Talise API, which is used for
 * configuring and managing calibration processes in radio frequency
 * systems.
 *
 * @param TAL_TRACK_BATCH_SIZE_500_US Represents a tracking calibration batch
 * size of 500 microseconds.
 * @param TAL_TRACK_BATCH_SIZE_200_US Represents a tracking calibration batch
 * size of 200 microseconds.
 ******************************************************************************/
typedef enum {
	TAL_TRACK_BATCH_SIZE_500_US = 0,
	TAL_TRACK_BATCH_SIZE_200_US = 1
} taliseTrackingCalBatchSize_t;

/***************************************************************************//**
 * @brief The `taliseTxLolStatus_t` structure is designed to hold the status
 * information for the Transmit Local Oscillator Leakage (Tx LOL)
 * calibration process. It includes fields to track the error code, the
 * percentage of data collected, a performance metric, and counters for
 * the number of iterations and updates performed during the calibration.
 * This structure is essential for monitoring and evaluating the
 * effectiveness and progress of the Tx LOL calibration in the Talise
 * API.
 *
 * @param errorCode Error code from Tx LOL.
 * @param percentComplete Percent of required data collected for the current
 * calibration, ranging from 0 to 100.
 * @param varianceMetric Metric indicating how well the tracking calibration is
 * performing.
 * @param iterCount Running counter that increments each time the calibration
 * runs to completion.
 * @param updateCount Running counter that increments each time the calibration
 * updates the correction/actuator hardware.
 ******************************************************************************/
typedef struct {
	uint32_t errorCode;         /*!< error code from Tx LOL */
	uint32_t percentComplete;   /*!< percent of required data collected for the current cal. Range 0 to 100 */
	uint32_t varianceMetric;    /*!< metric of how well the tracking cal is performing */
	uint32_t iterCount;         /*!< running counter that increments each time the cal runs to completion */
	uint32_t updateCount;       /*!< running counter that increments each time the cal updates the correction/actuator hardware */
} taliseTxLolStatus_t;

/***************************************************************************//**
 * @brief The `taliseTxQecStatus_t` structure is designed to hold the status
 * information for the Transmit Quadrature Error Correction (Tx QEC)
 * process. It includes fields to track the error code, the percentage of
 * data collected for the current calibration, a metric for the
 * performance of the tracking calibration, and counters for the number
 * of times the calibration has run to completion and updated the
 * hardware. This structure is essential for monitoring and evaluating
 * the effectiveness of the Tx QEC process in real-time.
 *
 * @param errorCode Error code from Tx QEC.
 * @param percentComplete Percent of required data collected for the current
 * calibration, ranging from 0 to 100.
 * @param correctionMetric Metric indicating how well the tracking calibration
 * is performing.
 * @param iterCount Running counter that increments each time the calibration
 * runs to completion.
 * @param updateCount Running counter that increments each time the calibration
 * updates the correction/actuator hardware.
 ******************************************************************************/
typedef struct {
	uint32_t errorCode;         /*!< error code from Tx QEC */
	uint32_t percentComplete;   /*!< percent of required data collected for the current cal. Range 0 to 100 */
	uint32_t correctionMetric;  /*!< metric of how well the tracking cal is performing */
	uint32_t iterCount;         /*!< running counter that increments each time the cal runs to completion */
	uint32_t updateCount;       /*!< running counter that increments each time the cal updates the correction/actuator hardware */
} taliseTxQecStatus_t;

/***************************************************************************//**
 * @brief The `taliseRxQecStatus_t` structure is designed to hold the status
 * information for the Rx Quadrature Error Correction (QEC) process. It
 * includes fields for tracking the error code, the percentage of data
 * collected for the current calibration, and a self-check IRR dB value.
 * Additionally, it maintains counters for the number of times the
 * calibration has run to completion and the number of updates made to
 * the correction hardware, providing a comprehensive overview of the
 * calibration's progress and status.
 *
 * @param errorCode Error code from Rx QEC.
 * @param percentComplete Percent of required data collected for the current
 * calibration, ranging from 0 to 100.
 * @param selfcheckIrrDb Self-check IRR dB value, specific purpose not
 * documented.
 * @param iterCount Running counter that increments each time the calibration
 * runs to completion.
 * @param updateCount Running counter that increments each time the calibration
 * updates the correction/actuator hardware.
 ******************************************************************************/
typedef struct {
	uint32_t errorCode;         /*!< error code from Rx QEC */
	uint32_t percentComplete;   /*!< percent of required data collected for the current cal. Range 0 to 100 */
	uint32_t selfcheckIrrDb;    /*!<  */
	uint32_t iterCount;         /*!< running counter that increments each time the cal runs to completion */
	uint32_t updateCount;       /*!< running counter that increments each time the cal updates the correction/actuator hardware */
} taliseRxQecStatus_t;

/***************************************************************************//**
 * @brief The `taliseOrxQecStatus_t` structure is designed to hold the status
 * information for the Orx Quadrature Error Correction (QEC) process. It
 * includes fields for tracking the error code, the percentage of data
 * collected for the current calibration, and a self-check IRR dB value.
 * Additionally, it maintains counters for the number of times the
 * calibration has run to completion and the number of updates made to
 * the correction or actuator hardware. This structure is part of the
 * Talise API, which is used for managing calibration processes in RF
 * systems.
 *
 * @param errorCode Error code from Orx QEC.
 * @param percentComplete Percent of required data collected for the current
 * calibration, ranging from 0 to 100.
 * @param selfcheckIrrDb Self-check IRR dB value, specific purpose not
 * documented.
 * @param iterCount Running counter that increments each time the calibration
 * runs to completion.
 * @param updateCount Running counter that increments each time the calibration
 * updates the correction/actuator hardware.
 ******************************************************************************/
typedef struct {
	uint32_t errorCode;         /*!< error code from Orx QEC */
	uint32_t percentComplete;   /*!< percent of required data collected for the current cal. Range 0 to 100 */
	uint32_t selfcheckIrrDb;    /*!<  */
	uint32_t iterCount;         /*!< running counter that increments each time the cal runs to completion */
	uint32_t updateCount;       /*!< running counter that increments each time the cal updates the correction/actuator hardware */
} taliseOrxQecStatus_t;

/***************************************************************************//**
 * @brief The `taliseRxHd2Status_t` structure is designed to hold the status
 * information for the Rx HD2 calibration process. It includes an error
 * code specific to HD2, a confidence level indicating how accurately the
 * calibration has identified the necessary coefficient to cancel HD2,
 * and counters for the number of iterations and updates performed during
 * the calibration process. The `percentComplete` field is not applicable
 * for HD2 due to the dynamic nature of the calibration conditions.
 *
 * @param errorCode Error code from HD2.
 * @param percentComplete Indicates the dynamic condition of the calibration,
 * not applicable for HD2.
 * @param confidenceLevel Confidence level that the calibration has accurately
 * identified the coefficient required to cancel HD2,
 * ranging from 0 (No Observation) to 7 (increasing
 * confidence).
 * @param iterCount Running counter that increments each time the calibration
 * runs to completion.
 * @param updateCount Running counter that increments each time the calibration
 * updates the correction/actuator hardware.
 ******************************************************************************/
typedef struct {
	uint32_t errorCode;         /*!< error code from HD2 */
	uint32_t percentComplete;   /*!< NOT APPLICABLE FOR HD2  Dynamic condition of this cal does not allow for convergence */
	uint32_t confidenceLevel;   /*!< Confidence level that cal has accurately indentified the coefficient required to cancel HD2  */
	/*!<     0 - No Observation, 1 - 7 indicates increasing confidence level */
	uint32_t iterCount;         /*!< running counter that increments each time the cal runs to completion */
	uint32_t updateCount;       /*!< running counter that increments each time the cal updates the correction/actuator hardware */
} taliseRxHd2Status_t;

/***************************************************************************//**
 * @brief The `taliseRxHd2Config_t` structure is used to configure the
 * calibration settings for correcting the second harmonic distortion
 * (HD2) in a received signal. The `posSideBandSel` member specifies
 * which side of the complex FFT (positive or negative) the desired
 * signal is located, allowing the calibration process to target the
 * appropriate side for HD2 correction.
 *
 * @param posSideBandSel Determines whether to correct HD2 on the positive
 * (upper) or negative (lower) side of the complex FFT.
 ******************************************************************************/
typedef struct {
	uint32_t posSideBandSel;     /*!< 1 = Correct HD2 of desired signal on the positive (upper) side of the complex FFT,
                                      0 = correct HD2 in the negative (lower)side of the complex FFT */
} taliseRxHd2Config_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_CALS_TYPES_H_ */
