/**
 * \file talise_jesd204_types.h
 * \brief Contains Talise API JESD data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_JESD204_TYPES_H_
#define TALISE_JESD204_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief The `taliseFramerSel_t` is an enumeration that defines the selection
 * of framers in a Talise device. It provides options to select either
 * Framer A, Framer B, or both framers simultaneously, which is useful in
 * scenarios where different framers are used for different receive paths
 * (Rx1 and Rx2). This enumeration is part of the Talise API JESD204 data
 * types, facilitating the configuration of framer selections in the
 * device.
 *
 * @param TAL_FRAMER_A Framer A selection.
 * @param TAL_FRAMER_B Framer B selection.
 * @param TAL_FRAMER_A_AND_B Used for cases where Rx1 uses one framer, Rx2 uses
 * the second framer.
 ******************************************************************************/
typedef enum {
	TAL_FRAMER_A = 0,       /*!< Framer A selection */
	TAL_FRAMER_B,           /*!< Framer B selection */
	TAL_FRAMER_A_AND_B      /*!< Used for cases where Rx1 uses one framer, Rx2 uses the second framer */
} taliseFramerSel_t;

/***************************************************************************//**
 * @brief The `taliseDeframerSel_t` is an enumeration that defines the selection
 * options for deframers in a Talise device. It provides three options:
 * selecting Deframer A, selecting Deframer B, or selecting both Deframer
 * A and B, which is useful in scenarios where different transmitters
 * (Tx1 and Tx2) use separate deframers. This enum is part of the Talise
 * API, which is used for configuring JESD204B interfaces in Analog
 * Devices' Talise transceivers.
 *
 * @param TAL_DEFRAMER_A Deframer A selection.
 * @param TAL_DEFRAMER_B Deframer B selection.
 * @param TAL_DEFRAMER_A_AND_B Used for cases where Tx1 uses one deframer, Tx2
 * uses the second deframer.
 ******************************************************************************/
typedef enum {
	TAL_DEFRAMER_A = 0,    /*!< Deframer A selection */
	TAL_DEFRAMER_B,        /*!< Deframer B selection */
	TAL_DEFRAMER_A_AND_B   /*!< Used for cases where Tx1 uses one deframer, Tx2 uses the second deframer */
} taliseDeframerSel_t;

/***************************************************************************//**
 * @brief The `taliseDacSampleXbarSelect_t` is an enumeration that defines the
 * possible output selections for the DAC sample crossbar in a Talise
 * device. It provides options for selecting different deframer outputs
 * from two links, Link 0 and Link 1, each with four possible outputs.
 * This enumeration is used to configure the routing of DAC samples
 * through the crossbar, allowing for flexible data path configurations
 * in the device.
 *
 * @param TAL_DEFRAMERA_OUT0 Link 0, Deframer out 0.
 * @param TAL_DEFRAMERA_OUT1 Link 0, Deframer out 1.
 * @param TAL_DEFRAMERA_OUT2 Link 0, Deframer out 2.
 * @param TAL_DEFRAMERA_OUT3 Link 0, Deframer out 3.
 * @param TAL_DEFRAMERB_OUT0 Link 1, Deframer out 0.
 * @param TAL_DEFRAMERB_OUT1 Link 1, Deframer out 1.
 * @param TAL_DEFRAMERB_OUT2 Link 1, Deframer out 2.
 * @param TAL_DEFRAMERB_OUT3 Link 1, Deframer out 3.
 ******************************************************************************/
typedef enum {
	TAL_DEFRAMERA_OUT0 = 0x0,               /*!< Link 0, Deframer out 0 */
	TAL_DEFRAMERA_OUT1 = 0x1,               /*!< Link 0, Deframer out 1 */
	TAL_DEFRAMERA_OUT2 = 0x2,               /*!< Link 0, Deframer out 2 */
	TAL_DEFRAMERA_OUT3 = 0x3,               /*!< Link 0, Deframer out 3 */
	TAL_DEFRAMERB_OUT0 = 0x4,               /*!< Link 1, Deframer out 0 */
	TAL_DEFRAMERB_OUT1 = 0x5,               /*!< Link 1, Deframer out 1 */
	TAL_DEFRAMERB_OUT2 = 0x6,               /*!< Link 1, Deframer out 2 */
	TAL_DEFRAMERB_OUT3 = 0x7                /*!< Link 1, Deframer out 3 */
} taliseDacSampleXbarSelect_t;

/***************************************************************************//**
 * @brief The `taliseAdcSampleXbarSelect_t` is an enumerated type that defines
 * various options for selecting ADC sample crossbar data paths. It
 * includes selections for both single-band and dual-band configurations,
 * allowing for the selection of I and Q data for different receiver
 * channels and bands. This enumeration is used to configure the ADC
 * sample crossbar in the Talise API, which is part of the JESD204B
 * interface configuration for Analog Devices' Talise transceivers.
 *
 * @param TAL_ADC_RX1_I Rx1 I data.
 * @param TAL_ADC_RX1_Q Rx1 Q data.
 * @param TAL_ADC_RX2_I Rx2 I data.
 * @param TAL_ADC_RX2_Q Rx2 Q data.
 * @param TAL_ADC_DUALBAND_RX1_BAND_A_I Dualband Rx1 Band A I data.
 * @param TAL_ADC_DUALBAND_RX1_BAND_A_Q Dualband Rx1 Band A Q data.
 * @param TAL_ADC_DUALBAND_RX2_BAND_A_I Dualband Rx2 Band A I data.
 * @param TAL_ADC_DUALBAND_RX2_BAND_A_Q Dualband Rx2 Band A Q data.
 * @param TAL_ADC_DUALBAND_RX1_BAND_B_I Dualband Rx1 Band B I data.
 * @param TAL_ADC_DUALBAND_RX1_BAND_B_Q Dualband Rx1 Band B Q data.
 * @param TAL_ADC_DUALBAND_RX2_BAND_B_I Dualband Rx2 Band B I data.
 * @param TAL_ADC_DUALBAND_RX2_BAND_B_Q Dualband Rx2 Band B Q data.
 ******************************************************************************/
typedef enum {
	TAL_ADC_RX1_I = 0x0,                    /*!<  Rx1 I data */
	TAL_ADC_RX1_Q = 0x1,                    /*!<  Rx1 Q data */
	TAL_ADC_RX2_I = 0x2,                    /*!<  Rx2 I data */
	TAL_ADC_RX2_Q = 0x3,                    /*!<  Rx2 Q data */
	TAL_ADC_DUALBAND_RX1_BAND_A_I = 0x0,    /*!<  Dualband Rx1 Band A I data */
	TAL_ADC_DUALBAND_RX1_BAND_A_Q = 0x1,    /*!<  Dualband Rx1 Band A Q data */
	TAL_ADC_DUALBAND_RX2_BAND_A_I = 0x2,    /*!<  Dualband Rx2 Band A I data */
	TAL_ADC_DUALBAND_RX2_BAND_A_Q = 0x3,    /*!<  Dualband Rx2 Band A Q data */
	TAL_ADC_DUALBAND_RX1_BAND_B_I = 0x4,    /*!<  Dualband Rx1 Band B I data */
	TAL_ADC_DUALBAND_RX1_BAND_B_Q = 0x5,    /*!<  Dualband Rx1 Band B Q data */
	TAL_ADC_DUALBAND_RX2_BAND_B_I = 0x6,    /*!<  Dualband Rx2 Band B I data */
	TAL_ADC_DUALBAND_RX2_BAND_B_Q = 0x7     /*!<  Dualband Rx2 Band B Q data */

} taliseAdcSampleXbarSelect_t;

/***************************************************************************//**
 * @brief The `taliseFramerDataSource_t` is an enumeration that defines various
 * data sources for framer test data in the Talise API. Each enumerator
 * represents a different type of test data that can be used to verify
 * the functionality of the framer, such as ADC data, checkerboard
 * patterns, toggle patterns, and various PRBS (Pseudo-Random Binary
 * Sequence) patterns. This enumeration is part of the Talise API's
 * JESD204B interface, which is used for high-speed data transmission in
 * RF systems.
 *
 * @param TAL_FTD_ADC_DATA Framer test data ADC data source.
 * @param TAL_FTD_CHECKERBOARD Framer test data checkerboard data source.
 * @param TAL_FTD_TOGGLE0_1 Framer test data toggle 0 to 1 data source.
 * @param TAL_FTD_PRBS31 Framer test data PRBS31 data source.
 * @param TAL_FTD_PRBS23 Framer test data PRBS23 data source.
 * @param TAL_FTD_PRBS15 Framer test data PRBS15 data source.
 * @param TAL_FTD_PRBS9 Framer test data PRBS9 data source.
 * @param TAL_FTD_PRBS7 Framer test data PRBS7 data source.
 * @param TAL_FTD_RAMP Framer test data ramp data source.
 ******************************************************************************/
typedef enum {
	TAL_FTD_ADC_DATA = 0,           /*!< Framer test data ADC data source */
	TAL_FTD_CHECKERBOARD,           /*!< Framer test data checkerboard data source */
	TAL_FTD_TOGGLE0_1,              /*!< Framer test data toggle 0 to 1 data source */
	TAL_FTD_PRBS31,                 /*!< Framer test data PRBS31 data source */
	TAL_FTD_PRBS23,                 /*!< Framer test data PRBS23 data source */
	TAL_FTD_PRBS15,                 /*!< Framer test data PRBS15 data source */
	TAL_FTD_PRBS9,                  /*!< Framer test data PRBS9 data source */
	TAL_FTD_PRBS7,                  /*!< Framer test data PRBS7 data source */
	TAL_FTD_RAMP                   /*!< Framer test data ramp data source */

} taliseFramerDataSource_t;

/***************************************************************************//**
 * @brief The `taliseFramerInjectPoint_t` is an enumeration that defines the
 * possible injection points for test data within a framer in a JESD204B
 * interface. It allows the selection of where test data should be
 * injected, either at the framer input, at the serializer input, or
 * after the lane mapping, providing flexibility in testing and debugging
 * the data flow through the framer.
 *
 * @param TAL_FTD_FRAMERINPUT Framer test data injection point at framer input.
 * @param TAL_FTD_SERIALIZER Framer test data injection point at serializer
 * input.
 * @param TAL_FTD_POST_LANEMAP Framer test data injection point after lane
 * mapping.
 ******************************************************************************/
typedef enum {
	TAL_FTD_FRAMERINPUT = 0,        /*!< Framer test data injection point at framer input */
	TAL_FTD_SERIALIZER,             /*!< Framer test data injection point at serializer input */
	TAL_FTD_POST_LANEMAP            /*!< Framer test data injection point after lane mapping */

} taliseFramerInjectPoint_t;

/***************************************************************************//**
 * @brief The `taliseDeframerPrbsOrder_t` is an enumeration that defines the
 * possible PRBS (Pseudo-Random Binary Sequence) patterns that can be
 * selected for the deframer in a JESD204B interface. It includes options
 * to disable PRBS or select from PRBS7, PRBS15, and PRBS31 patterns,
 * which are used for testing and verifying the integrity of data
 * transmission in high-speed serial communication systems.
 *
 * @param TAL_PRBS_DISABLE Deframer PRBS pattern disable.
 * @param TAL_PRBS7 Deframer PRBS7 pattern select.
 * @param TAL_PRBS15 Deframer PRBS15 pattern select.
 * @param TAL_PRBS31 Deframer PRBS31 pattern select.
 ******************************************************************************/
typedef enum {
	TAL_PRBS_DISABLE = 0,       /*!< Deframer PRBS pattern disable */
	TAL_PRBS7,                  /*!< Deframer PRBS7 pattern select */
	TAL_PRBS15,                 /*!< Deframer PRBS15 pattern select */
	TAL_PRBS31                  /*!< Deframer PRBS31 pattern select */

} taliseDeframerPrbsOrder_t;

/***************************************************************************//**
 * @brief The `taliseDefPrbsCheckLoc_t` is an enumeration that defines the
 * possible locations for checking Pseudo-Random Binary Sequence (PRBS)
 * errors in a JESD204b system. It provides two options: checking at the
 * deserializer lane output, which does not require a JESD204b link, and
 * checking at the output of the deframer, which involves deframed
 * samples. This enum is used to specify where the PRBS check should be
 * performed in the data path.
 *
 * @param TAL_PRBSCHECK_LANEDATA Check PRBS at deserializer lane output (does
 * not require JESD204b link).
 * @param TAL_PRBSCHECK_SAMPLEDATA Check PRBS at output of deframer (JESD204b
 * deframed sample).
 ******************************************************************************/
typedef enum {
	TAL_PRBSCHECK_LANEDATA = 0, /*!< Check PRBS at deserializer lane output (does not require JESD204b link) */
	TAL_PRBSCHECK_SAMPLEDATA    /*!< Check PRBS at output of deframer (JESD204b deframed sample) */

} taliseDefPrbsCheckLoc_t;

/***************************************************************************//**
 * @brief The `taliseDacSampleXbar_t` structure is designed to hold the
 * configuration for the DAC sample crossbar, specifically selecting the
 * crossbar options for the I and Q channel data. It uses the
 * `taliseDacSampleXbarSelect_t` enumeration to specify the crossbar
 * selection for each channel, allowing for flexible routing of DAC data
 * within the system.
 *
 * @param dacChanI Sample Crossbar select for I channel data.
 * @param dacChanQ Sample Crossbar select for Q channel data.
 ******************************************************************************/
typedef struct {
	taliseDacSampleXbarSelect_t
	dacChanI;    /*!< Sample Crossbar select for I channel data*/
	taliseDacSampleXbarSelect_t
	dacChanQ;    /*!< Sample Crossbar select for Q channel data*/
} taliseDacSampleXbar_t;

/***************************************************************************//**
 * @brief The `taliseAdcSampleXbar_t` structure is designed to hold the
 * configuration for the ADC sample crossbar, allowing the selection of
 * specific data paths for up to eight converters. Each member of the
 * structure, from `conv0` to `conv7`, is of type
 * `taliseAdcSampleXbarSelect_t`, which specifies the crossbar selection
 * for the corresponding converter. This structure is essential for
 * configuring the data routing in systems utilizing the Talise API,
 * particularly in applications involving multiple ADC channels.
 *
 * @param conv0 Sample Crossbar select for converter 0.
 * @param conv1 Sample Crossbar select for converter 1.
 * @param conv2 Sample Crossbar select for converter 2.
 * @param conv3 Sample Crossbar select for converter 3.
 * @param conv4 Sample Crossbar select for converter 4.
 * @param conv5 Sample Crossbar select for converter 5.
 * @param conv6 Sample Crossbar select for converter 6.
 * @param conv7 Sample Crossbar select for converter 7.
 ******************************************************************************/
typedef struct {
	taliseAdcSampleXbarSelect_t
	conv0;    /*!< Sample Crossbar select for converter 0*/
	taliseAdcSampleXbarSelect_t
	conv1;    /*!< Sample Crossbar select for converter 1*/
	taliseAdcSampleXbarSelect_t
	conv2;    /*!< Sample Crossbar select for converter 2*/
	taliseAdcSampleXbarSelect_t
	conv3;    /*!< Sample Crossbar select for converter 3*/
	taliseAdcSampleXbarSelect_t
	conv4;    /*!< Sample Crossbar select for converter 4*/
	taliseAdcSampleXbarSelect_t
	conv5;    /*!< Sample Crossbar select for converter 5*/
	taliseAdcSampleXbarSelect_t
	conv6;    /*!< Sample Crossbar select for converter 6*/
	taliseAdcSampleXbarSelect_t
	conv7;    /*!< Sample Crossbar select for converter 7*/
} taliseAdcSampleXbar_t;

/***************************************************************************//**
 * @brief The `taliseJesd204bLane0Config_t` structure is used to hold
 * configuration settings for the JESD204B interface, specifically for
 * the ILAS (Initial Lane Alignment Sequence) check on lane 0. It
 * includes various parameters such as device ID, bank ID, lane ID, and
 * settings for scrambling, frame and multiframe sizes, data converter
 * resolution, and control bits. Additionally, it contains checksums for
 * verifying the integrity of the ILAS on multiple lanes. This structure
 * is crucial for ensuring proper alignment and configuration of the
 * JESD204B interface in high-speed data converter applications.
 *
 * @param DID JESD204B Configuration Device ID for ILAS check.
 * @param BID JESD204B Configuration Bank ID for ILAS check.
 * @param LID0 JESD204B Configuration starting Lane ID for ILAS check.
 * @param L JESD204B Configuration L = lanes per data converter for ILAS check.
 * @param SCR JESD204B Configuration scramble setting for ILAS check.
 * @param F JESD204B Configuration F = octets per frame for ILAS check.
 * @param K JESD204B Configuration K = frames per multiframe for ILAS check.
 * @param M JESD204B Configuration M = number of data converters for ILAS check.
 * @param N JESD204B Configuration N = data converter sample resolution for ILAS
 * check.
 * @param CS JESD204B Configuration CS = number of control bits transferred per
 * sample per frame for ILAS check.
 * @param NP JESD204B Configuration NP = JESD204B word size based on the highest
 * resolution of the data converter for ILAS check.
 * @param S JESD204B Configuration S = number of samples/data converter/frame
 * for ILAS check.
 * @param CF JESD204B Configuration CF = '0' = control bits appended to each
 * sample, '1' = appended to end of frame for ILAS check.
 * @param HD JESD204B Configuration HD = high density bit - samples are
 * contained within lane (0) or divided over more than one lane (1)
 * for ILAS check.
 * @param FCHK0 JESD204B Configuration checksum for ILAS check lane0.
 * @param FCHK1 JESD204B Configuration checksum for ILAS check lane1.
 * @param FCHK2 JESD204B Configuration checksum for ILAS check lane2.
 * @param FCHK3 JESD204B Configuration checksum for ILAS check lane3.
 ******************************************************************************/
typedef struct {
	uint8_t DID;                          /*!< JESD204B Configuration Device ID for ILAS check */
	uint8_t BID;                          /*!< JESD204B Configuration Bank ID for ILAS check */
	uint8_t LID0;                         /*!< JESD204B Configuration starting Lane ID for ILAS check */
	uint8_t L;                            /*!< JESD204B Configuration L = lanes per data converter for ILAS check */
	uint8_t SCR;                          /*!< JESD204B Configuration scramble setting for ILAS check */
	uint8_t F;                            /*!< JESD204B Configuration F = octets per frame for ILAS check */
	uint8_t K;                            /*!< JESD204B Configuration K = frames per multiframe for ILAS check */
	uint8_t M;                            /*!< JESD204B Configuration M = number of data converters for ILAS check */
	uint8_t N;                            /*!< JESD204B Configuration N = data converter sample resolution for ILAS check */
	uint8_t CS;                           /*!< JESD204B Configuration CS = number of control bits transferred per sample per frame for ILAS check */
	uint8_t NP;                           /*!< JESD204B Configuration NP = JESD204B word size based on the highest resolution of the data converter for ILAS check */
	uint8_t S;                            /*!< JESD204B Configuration S = number of samples/data converter/frame for ILAS check */
	uint8_t CF;                           /*!< JESD204B Configuration CF = '0' = control bits appended to each sample, '1' = appended to end of frame for ILAS check */
	uint8_t HD;                           /*!< JESD204B Configuration HD = high density bit - samples are contained within lane (0) or divided over more than one lane (1) for ILAS check */
	uint8_t FCHK0;                        /*!< JESD204B Configuration checksum for ILAS check lane0 */
	uint8_t FCHK1;                        /*!< JESD204B Configuration checksum for ILAS check lane1 */
	uint8_t FCHK2;                        /*!< JESD204B Configuration checksum for ILAS check lane2 */
	uint8_t FCHK3;                        /*!< JESD204B Configuration checksum for ILAS check lane3 */
} taliseJesd204bLane0Config_t;

/***************************************************************************//**
 * @brief The `taliseJesd204bFramerConfig_t` structure is used to configure the
 * JESD204B framer settings for a Talise device. It includes parameters
 * for identifying the device and lane IDs, configuring the number of
 * ADCs and frames, setting the sample resolution, and enabling or
 * disabling scrambling. The structure also allows for the selection of
 * external SYSREF, configuration of serializer lanes, and setting of
 * LMFC offset for deterministic latency. Additional options include
 * selecting the SYNCb input source, choosing between bit repeat or
 * oversampling modes, and configuring LVDS or CMOS input pads. The
 * structure provides flexibility in mapping framer lane outputs to
 * physical lanes, either automatically or manually.
 *
 * @param bankId JESD204B Configuration Bank ID extension to Device ID, ranging
 * from 0 to 15.
 * @param deviceId JESD204B Configuration Device ID link identification number,
 * ranging from 0 to 255.
 * @param lane0Id JESD204B Configuration starting Lane ID, ranging from 0 to 31.
 * @param M Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive
 * chain (I and Q).
 * @param K Number of frames in a multiframe, default is 32, and F*K must be
 * modulo 4.
 * @param F Number of bytes (octets) per frame, valid values are 1, 2, 4, or 8.
 * @param Np Converter sample resolution, valid values are 12, 16, or 24.
 * @param scramble Indicates if scrambling is off (0) or enabled (>0).
 * @param externalSysref Selects external SYSREF, 0 for internal (not valid), 1
 * for external.
 * @param serializerLanesEnabled Serializer lane select bit field, where each
 * bit represents a lane.
 * @param serializerLaneCrossbar Lane crossbar to map framer lane outputs to
 * physical lanes.
 * @param lmfcOffset LMFC offset value for deterministic latency setting,
 * ranging from 0 to 31.
 * @param newSysrefOnRelink Flag to determine if SYSREF on relink should be set,
 * >0 to set, 0 not to set.
 * @param syncbInSelect Selects SYNCb input source, 0 for SYNCBIN0, 1 for
 * SYNCBIN1.
 * @param overSample Selects framer bit repeat or oversampling mode for lane
 * rate matching.
 * @param syncbInLvdsMode 1 to enable LVDS input pad with 100ohm termination, 0
 * for CMOS input pad.
 * @param syncbInLvdsPnInvert 0 for non-inverted syncb LVDS PN, 1 for inverted.
 * @param enableManualLaneXbar 0 for automatic lane crossbar mapping, 1 for
 * manual using serializerLaneCrossbar.
 ******************************************************************************/
typedef struct {
	uint8_t bankId;                     /*!< JESD204B Configuration Bank ID extension to Device ID. Range is 0..15 */
	uint8_t deviceId;                   /*!< JESD204B Configuration Device ID link identification number. Range is 0..255 */
	uint8_t lane0Id;                    /*!< JESD204B Configuration starting Lane ID. If more than one lane is used, each lane will increment from the Lane0 ID. Range is 0..31 */
	uint8_t M;                          /*!< Number of ADCs (0, 2, or 4) where 2 ADCs are required per receive chain (I and Q) */
	uint8_t K;                          /*!< Number of frames in a multiframe. Default = 32, F*K must be modulo 4. Where, F=2*M/numberOfLanes */
	uint8_t F;                          /*!< Number of bytes(octets) per frame (Valid 1, 2, 4, 8) */
	uint8_t Np;                         /*!< converter sample resolution (12, 16, 24) */
	uint8_t scramble;                   /*!< Scrambling off if framerScramble = 0, if framerScramble > 0 scrambling is enabled */
	uint8_t externalSysref;             /*!< External SYSREF select. 0 = use internal SYSREF(not currently valid), 1 = use external SYSREF */
	uint8_t serializerLanesEnabled;     /*!< Serializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1 enabled, etc */
	uint8_t serializerLaneCrossbar;     /*!< Lane crossbar to map framer lane outputs to physical lanes */
	uint8_t lmfcOffset;                 /*!< LMFC offset value for deterministic latency setting.  Range is 0..31 */
	uint8_t newSysrefOnRelink;          /*!< Flag for determining if SYSREF on relink should be set. Where, if > 0 = set, 0 = not set */
	uint8_t syncbInSelect;              /*!< Selects SYNCb input source. Where, 0 = use SYNCBIN0 for this framer, 1 = use SYNCBIN1 for this framer */
	uint8_t overSample;                 /*!< Selects framer bit repeat or oversampling mode for lane rate matching. Where, 0 = bitRepeat mode (changes effective lanerate), 1 = overSample (maintains same lane rate between ObsRx framer and Rx framer and oversamples the ADC samples) */
	uint8_t syncbInLvdsMode;            /*!< 1 - enable LVDS input pad with 100ohm internal termination, 0 - enable CMOS input pad */
	uint8_t syncbInLvdsPnInvert;        /*!< 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted */
	uint8_t enableManualLaneXbar;       /*!< 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping (use serializerLaneCrossbar value with no checking) */
} taliseJesd204bFramerConfig_t;

/***************************************************************************//**
 * @brief The `taliseJesd204bDeframerConfig_t` structure is used to configure
 * the settings for a JESD204B deframer in a Talise device. It includes
 * parameters for identifying the device and lane, configuring the number
 * of DACs and frames, enabling or disabling scrambling, selecting SYSREF
 * sources, and configuring deserializer lanes and crossbars.
 * Additionally, it provides options for setting LVDS or CMOS modes,
 * adjusting latency, and determining the behavior of SYSREF on relink.
 * This structure is crucial for setting up the deframer to correctly
 * interpret and process incoming JESD204B data streams.
 *
 * @param bankId Extension to Device ID, ranging from 0 to 15.
 * @param deviceId Link identification number, ranging from 0 to 255.
 * @param lane0Id Lane0 ID, ranging from 0 to 31.
 * @param M Number of DACs (0, 2, or 4) with 2 DACs per transmit chain (I and
 * Q).
 * @param K Number of frames in a multiframe, default is 32, with F*K being
 * modulo 4.
 * @param scramble Indicates if scrambling is off (0) or enabled (>0).
 * @param externalSysref Selects between internal (0) or external (1) SYSREF.
 * @param deserializerLanesEnabled Bit field for deserializer lane selection,
 * where each bit represents a lane.
 * @param deserializerLaneCrossbar Maps deframer lane outputs to physical lanes.
 * @param lmfcOffset LMFC offset value for adjusting deterministic latency,
 * ranging from 0 to 31.
 * @param newSysrefOnRelink Flag to determine if SYSREF on relink should be set,
 * where >0 means set.
 * @param syncbOutSelect Selects deframer SYNCBOUT pin, either 0 (SYNCBOUT0) or
 * 1 (SYNCBOUT1).
 * @param Np Converter sample resolution, either 12 or 16.
 * @param syncbOutLvdsMode Enables LVDS output pad (1) or CMOS output pad (0).
 * @param syncbOutLvdsPnInvert Indicates if syncb LVDS PN is not inverted (0) or
 * inverted (1).
 * @param syncbOutCmosSlewRate Sets CMOS slew rate, with 0 being the fastest and
 * 3 the slowest.
 * @param syncbOutCmosDriveLevel Sets CMOS drive level, with 0 being normal and
 * 1 being double.
 * @param enableManualLaneXbar Enables manual (1) or automatic (0) lane crossbar
 * mapping.
 ******************************************************************************/
typedef struct {
	uint8_t bankId;                     /*!< Extension to Device ID. Range is 0..15 */
	uint8_t deviceId;                   /*!< Link identification number. Range is 0..255 */
	uint8_t lane0Id;                    /*!< Lane0 ID. Range is 0..31 */
	uint8_t M;                          /*!< Number of DACs (0, 2, or 4) - 2 DACs per transmit chain (I and Q) */
	uint8_t K;                          /*!< Number of frames in a multiframe. Default = 32, F*K = modulo 4. Where, F=2*M/numberOfLanes */
	uint8_t scramble;                   /*!< Scrambling off if scramble = 0, if framerScramble > 0 scrambling is enabled */
	uint8_t externalSysref;             /*!< External SYSREF select. 0 = use internal SYSREF, 1 = external SYSREF */
	uint8_t deserializerLanesEnabled;   /*!< Deserializer lane select bit field. Where, [0] = Lane0 enabled, [1] = Lane1 enabled, etc */
	uint8_t deserializerLaneCrossbar;   /*!< Lane crossbar to map deframer lane outputs to physical lanes */
	uint8_t lmfcOffset;                 /*!< LMFC offset value to adjust deterministic latency. Range is 0..31 */
	uint8_t newSysrefOnRelink;          /*!< Flag for determining if SYSREF on relink should be set. Where, if > 0 = set, '0' = not set */
	uint8_t syncbOutSelect;             /*!< Selects deframer SYNCBOUT pin (0 = SYNCBOUT0, 1 = SYNCBOUT1) */
	uint8_t Np;                         /*!< converter sample resolution (12, 16) */
	uint8_t syncbOutLvdsMode;           /*!< 1 - enable LVDS output pad, 0 - enable CMOS output pad  */
	uint8_t syncbOutLvdsPnInvert;       /*!< 0 - syncb LVDS PN not inverted, 1 - syncb LVDS PN inverted */
	uint8_t syncbOutCmosSlewRate;       /*!< 0 - fastest rise/fall times, 3 - slowest rise/fall times */
	uint8_t syncbOutCmosDriveLevel;     /*!< 0 - normal cmos drive level, 1 - double cmos drive level */
	uint8_t enableManualLaneXbar;       /*!< 0 - Automatic Lane crossbar mapping, 1 - Manual Lane crossbar mapping (use deserializerLaneCrossbar value with no checking) */
} taliseJesd204bDeframerConfig_t;

/***************************************************************************//**
 * @brief The `taliseJesdSettings_t` structure is designed to hold configuration
 * settings for both the framer and deframer components of a JESD204B
 * interface in a Talise device. It includes detailed configuration data
 * for two framers and two deframers, as well as settings for serializer
 * amplitude, pre-emphasis, lane polarity inversion, and deserializer
 * equalization. Additionally, it provides options for configuring the
 * sysref LVDS mode and inversion, allowing for flexible and precise
 * control over the JESD204B link parameters.
 *
 * @param framerA Framer A configuration data structure.
 * @param framerB Framer B configuration data structure.
 * @param deframerA Deframer A configuration data structure.
 * @param deframerB Deframer B configuration data structure.
 * @param serAmplitude Serializer amplitude setting with a default of 15 and a
 * range of 0 to 15.
 * @param serPreEmphasis Serializer pre-emphasis setting with a default of 1 and
 * a range of 0 to 4.
 * @param serInvertLanePolarity Serializer Lane PN inversion select, where each
 * bit corresponds to a lane inversion.
 * @param desInvertLanePolarity Deserializer Lane PN inversion select, where
 * each bit corresponds to a lane inversion.
 * @param desEqSetting Deserializer Equalizer setting applied to all lanes, with
 * a range of 0 to 2.
 * @param sysrefLvdsMode Mode to enable LVDS Input pad with 100ohm termination
 * or CMOS input pad.
 * @param sysrefLvdsPnInvert Setting to invert or not invert the sysref LVDS PN.
 ******************************************************************************/
typedef struct {
	taliseJesd204bFramerConfig_t
	framerA;       /*!< Framer A configuration data structure */
	taliseJesd204bFramerConfig_t
	framerB;       /*!< Framer B configuration data structure */
	taliseJesd204bDeframerConfig_t
	deframerA;   /*!< Deframer A configuration data structure */
	taliseJesd204bDeframerConfig_t
	deframerB;   /*!< Deframer A configuration data structure */
	uint8_t serAmplitude;                       /*!< Serializer amplitude setting. Default = 15. Range is 0..15 */
	uint8_t serPreEmphasis;                     /*!< Serializer pre-emphasis setting. Default = 1 Range is 0..4 */
	uint8_t serInvertLanePolarity;              /*!< Serializer Lane PN inversion select. Default = 0. Where, bit[0] = 1 will invert lane [0], bit[1] = 1 will invert lane 1, etc. */
	uint8_t desInvertLanePolarity;              /*!< Deserializer Lane PN inversion select.  bit[0] = 1 Invert PN of Lane 0, bit[1] = Invert PN of Lane 1, etc */
	uint8_t desEqSetting;                       /*!< Deserializer Equalizer setting. Applied to all deserializer lanes. Range is 0..2 (default 2, 0 = max boost) */
	uint8_t sysrefLvdsMode;                     /*!< 1 - enable LVDS Input pad with 100ohm internal termination, 0 - enable CMOS input pad */
	uint8_t sysrefLvdsPnInvert;                 /*!< 0 - sysref LVDS PN is not inverted, 1 - sysref LVDS PN is inverted */
} taliseJesdSettings_t;


#ifdef __cplusplus
}
#endif

#endif /* TALISE_JESD204_TYPES_H_ */
