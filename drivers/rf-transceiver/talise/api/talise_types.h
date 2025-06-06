/*!
 * \file talise_types.h
 * \brief Contains Talise API configuration and run-time type definitions
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_TYPES_H_
#define TALISE_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#include "talise_rx_types.h"
#include "talise_jesd204_types.h"
#include "talise_tx_types.h"
#include "talise_radioctrl_types.h"

/***************************************************************************//**
 * @brief The `taliseHsDiv_t` is an enumeration that defines possible high-speed
 * clock divider settings for the Talise device. Each enumerator
 * represents a specific division factor for the high-speed clock,
 * allowing the clock to be divided by 2, 2.5, 3, 4, or 5. This
 * configuration is crucial for setting the appropriate clock rate for
 * various operations within the device, ensuring that the clock
 * frequency is suitable for the desired application.
 *
 * @param TAL_HSDIV_2 High speed clock divide by 2 setting.
 * @param TAL_HSDIV_2P5 High speed clock divide by 2.5 setting.
 * @param TAL_HSDIV_3 High speed clock divide by 3 setting.
 * @param TAL_HSDIV_4 High speed clock divide by 4 setting.
 * @param TAL_HSDIV_5 High speed clock divide by 5 setting.
 ******************************************************************************/
typedef enum {
	TAL_HSDIV_2   = 0x0,    /*!< High speed clock divide by 2 setting */
	TAL_HSDIV_2P5 = 0x1,    /*!< High speed clock divide by 2.5 setting */
	TAL_HSDIV_3   = 0x2,    /*!< High speed clock divide by 3 setting */
	TAL_HSDIV_4   = 0x3,    /*!< High speed clock divide by 4 setting */
	TAL_HSDIV_5   = 0x4     /*!< High speed clock divide by 5 setting */
} taliseHsDiv_t;

/***************************************************************************//**
 * @brief The `taliseObsRxLoSource_t` is an enumeration that defines the
 * possible sources for the Observation Receiver Local Oscillator (LO) in
 * the Talise API. It allows the selection between using the RF PLL or
 * the AUX PLL as the source for the Observation LO, providing
 * flexibility in configuring the LO path for observation purposes.
 *
 * @param TAL_OBSLO_RF_PLL Selects RF PLL for Observation LO.
 * @param TAL_OBSLO_AUX_PLL Selects AUX PLL for Observation LO.
 ******************************************************************************/
typedef enum {
	TAL_OBSLO_RF_PLL = 0,           /*!< Selects RF PLL for Observation LO */
	TAL_OBSLO_AUX_PLL               /*!< Selects AUX PLL for Observation LO */
} taliseObsRxLoSource_t;

/***************************************************************************//**
 * @brief The `talisefirName_t` is an enumeration that defines various FIR
 * (Finite Impulse Response) filter selection options for different
 * transmission and reception paths in a Talise device. Each enumerator
 * represents a specific configuration of FIR filters, including options
 * for individual transmission (Tx) and reception (Rx) paths,
 * combinations of these paths, and loopback configurations. This
 * enumeration is used to specify the desired FIR filter setup in related
 * functions, facilitating the configuration of signal processing paths
 * in the device.
 *
 * @param TAL_TX1_FIR Tx1 FIR filter select.
 * @param TAL_TX2_FIR Tx2 FIR filter select.
 * @param TAL_TX1TX2_FIR Tx1 + Tx2 FIR filter select.
 * @param TAL_RX1_FIR Rx1 FIR filter select.
 * @param TAL_RX2_FIR Rx2 FIR filter select.
 * @param TAL_RX1RX2_FIR Rx1 + Rx2 FIR filter select.
 * @param TAL_RX1RX2_LOOPBACK_FIR Rx1 + Rx2 loopback FIR filter select.
 * @param TAL_OBSRX1_FIR ORx1 FIR filter select.
 * @param TAL_OBSRX2_FIR ORx2 FIR filter select.
 * @param TAL_OBSRX1RX2_FIR ORx1 + ORx2 FIR filter select.
 * @param TAL_OBSRX1RX2_LOOPBACK_FIR ORx1 + ORx2 loopback FIR filter select.
 * @param TAL_LOOPBACK_FIR Loopback FIR filter select.
 ******************************************************************************/
typedef enum {
	TAL_TX1_FIR = 1,                        /*!< Tx1 FIR filter select */
	TAL_TX2_FIR = 2,                        /*!< Tx2 FIR filter select */
	TAL_TX1TX2_FIR = 3,                     /*!< Tx1 + Tx2 FIR filter select */
	TAL_RX1_FIR = 4,                        /*!< Rx1 FIR filter select */
	TAL_RX2_FIR = 8,                        /*!< Rx2 FIR filter select */
	TAL_RX1RX2_FIR = 12,                    /*!< Rx1 + Rx2 FIR filter select */
	TAL_RX1RX2_LOOPBACK_FIR = 140,          /*!< Rx1 + Rx2 loopback FIR filter select */
	TAL_OBSRX1_FIR = 16,                    /*!< ORx1 FIR filter select */
	TAL_OBSRX2_FIR = 32,                    /*!< ORx2 FIR filter select */
	TAL_OBSRX1RX2_FIR = 48,                 /*!< ORx1 + ORx2 FIR filter select */
	TAL_OBSRX1RX2_LOOPBACK_FIR = 176,       /*!< ORx1 + ORx2 loopback FIR filter select */
	TAL_LOOPBACK_FIR = 128                  /*!< Loopback FIR filter select */
} talisefirName_t;

/***************************************************************************//**
 * @brief The `taliseFirGain_t` is an enumeration that defines the possible gain
 * settings for the Talise FIR filter. It provides four discrete gain
 * levels: -12 dB, -6 dB, 0 dB, and +6 dB, allowing for adjustment of the
 * filter's amplification or attenuation in the signal processing chain.
 *
 * @param TAL_FIR_GAIN_NEG12_DB Talise FIR gain set to -12 dB.
 * @param TAL_FIR_GAIN_NEG6_DB Talise FIR gain set to -6 dB.
 * @param TAL_FIR_GAIN_0_DB Talise FIR gain set to 0 dB.
 * @param TAL_FIR_GAIN_POS6_DB Talise FIR gain set to 6 dB.
 ******************************************************************************/
typedef enum {
	TAL_FIR_GAIN_NEG12_DB = -12, /*!< Talise FIR gain -12 */
	TAL_FIR_GAIN_NEG6_DB = -6,   /*!< Talise FIR gain -6 */
	TAL_FIR_GAIN_0_DB = 0,       /*!< Talise FIR gain 0 */
	TAL_FIR_GAIN_POS6_DB = 6     /*!< Talise FIR gain 6 */
} taliseFirGain_t;

/***************************************************************************//**
 * @brief The `taliseStates_t` is an enumeration that defines various
 * operational states of a Talise device. Each state is represented by a
 * unique hexadecimal value, allowing the software to track and manage
 * the device's current status, such as whether it is powered on,
 * initialized, has loaded streams or ARM code, has run calibrations, or
 * has the radio turned on. This enumeration is crucial for controlling
 * and monitoring the device's lifecycle and ensuring it operates
 * correctly within its intended use case.
 *
 * @param TAL_STATE_POWERONRESET Represents the power-on reset state with a
 * value of 0x00.
 * @param TAL_STATE_INITIALIZED Indicates the initialized state with a value of
 * 0x01.
 * @param TAL_STATE_STREAMLOADED Denotes the stream loaded state with a value of
 * 0x02.
 * @param TAL_STATE_ARMLOADED Represents the ARM loaded state with a value of
 * 0x04.
 * @param TAL_STATE_CALS_RUN Indicates the calibrations run state with a value
 * of 0x08.
 * @param TAL_STATE_RADIOON Denotes the radio on state with a value of 0x10.
 ******************************************************************************/
typedef enum {
	TAL_STATE_POWERONRESET = 0x00,
	TAL_STATE_INITIALIZED  = 0x01,
	TAL_STATE_STREAMLOADED = 0x02,
	TAL_STATE_ARMLOADED    = 0x04,
	TAL_STATE_CALS_RUN     = 0x08,
	TAL_STATE_RADIOON      = 0x10
} taliseStates_t;

/***************************************************************************//**
 * @brief The `taliseRxDdc_t` is an enumeration that defines the possible modes
 * for the Digital Down Converter (DDC) in the Talise API. It includes
 * options for bypassing the half-band filters, applying the filters
 * only, and performing interpolation or decimation by a factor of two.
 * The enumeration also provides similar options for real intermediate
 * frequency (IF) scenarios, allowing for flexible configuration of the
 * DDC based on the specific requirements of the signal processing chain.
 *
 * @param TAL_RXDDC_BYPASS No Half Band Enabled.
 * @param TAL_RXDDC_FILTERONLY Half Band Filters only.
 * @param TAL_RXDDC_INT2 Half Band Interpolation by 2.
 * @param TAL_RXDDC_DEC2 Half Band Decimate by 2.
 * @param TAL_RXDDC_BYPASS_REALIF No Half Band Enabled.
 * @param TAL_RXDDC_FILTERONLY_REALIF Half Band Filters only.
 * @param TAL_RXDDC_INT2_REALIF Half Band Interpolation by 2.
 * @param TAL_RXDDC_DEC2_REALIF Half Band Decimate by 2.
 ******************************************************************************/
typedef enum {
	TAL_RXDDC_BYPASS = 0,      /*!< No Half Band Enabled */
	TAL_RXDDC_FILTERONLY,      /*!< Half Band Filters only */
	TAL_RXDDC_INT2,            /*!< Half Band Interpolation by 2 */
	TAL_RXDDC_DEC2,            /*!< Half Band Decimate by 2 */
	TAL_RXDDC_BYPASS_REALIF,   /*!< No Half Band Enabled */
	TAL_RXDDC_FILTERONLY_REALIF,     /*!< Half Band Filters only */
	TAL_RXDDC_INT2_REALIF,     /*!< Half Band Interpolation by 2 */
	TAL_RXDDC_DEC2_REALIF      /*!< Half Band Decimate by 2 */
} taliseRxDdc_t;

/***************************************************************************//**
 * @brief The `taliseOrxDdc_t` is an enumeration that defines the possible modes
 * for the Observation Receiver (ORx) Digital Down Converter (DDC) in the
 * Talise API. It includes options to either disable the DDC or to enable
 * subsampling of the received observation signal by a factor of two.
 * This enumeration is used to configure the ORx DDC mode in the Talise
 * device, allowing for flexibility in handling observation signals.
 *
 * @param TAL_ORXDDC_DISABLED ZIF Observation Receiver.
 * @param TAL_ORXDDC_SUBSAMPLE_BY2 Subsample Received Observation signal.
 ******************************************************************************/
typedef enum {
	TAL_ORXDDC_DISABLED = 0x00,          /*!< ZIF Observation Receiver */
	TAL_ORXDDC_SUBSAMPLE_BY2             /*!< Subsample Received Observation signal */
} taliseOrxDdc_t;

/***************************************************************************//**
 * @brief The `taliseCmosPadDrvStr_t` is an enumerated type that defines various
 * drive strength options for CMOS pads, each associated with a specific
 * capacitive load at a frequency of 65MHz. These options are encoded
 * with specific values that represent different drive strengths,
 * allowing for the selection of appropriate drive capabilities based on
 * the load requirements of the application.
 *
 * @param TAL_CMOSPAD_DRV_1X Represents a drive strength option with a 2.5pF
 * load at 65MHz.
 * @param TAL_CMOSPAD_DRV_2X Represents a drive strength option with a 5pF load
 * at 65MHz.
 * @param TAL_CMOSPAD_DRV_3X Represents a drive strength option with a 7.5pF
 * load at 65MHz.
 * @param TAL_CMOSPAD_DRV_4X Represents a drive strength option with a 10pF load
 * at 65MHz.
 * @param TAL_CMOSPAD_DRV_5X Represents a drive strength option with a 12.5pF
 * load at 65MHz.
 * @param TAL_CMOSPAD_DRV_6X Represents a drive strength option with a 15pF load
 * at 65MHz.
 * @param TAL_CMOSPAD_DRV_8X Represents a drive strength option with a 20pF load
 * at 65MHz.
 * @param TAL_CMOSPAD_DRV_10X Represents a drive strength option with a 25pF
 * load at 65MHz.
 ******************************************************************************/
typedef enum {
	/* Values encoded as {non_gpio_drv , spi_cmos_drv_select[3:0]} */
	TAL_CMOSPAD_DRV_1X  = 0x00,       /*!<  2.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_2X  = 0x10,       /*!<    5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_3X  = 0x03,       /*!<  7.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_4X  = 0x11,       /*!<   10pF load @ 65MHz */
	TAL_CMOSPAD_DRV_5X  = 0x0F,       /*!< 12.5pF load @ 65MHz */
	TAL_CMOSPAD_DRV_6X  = 0x13,       /*!<   15pF load @ 65MHz */
	TAL_CMOSPAD_DRV_8X  = 0x17,       /*!<   20pF load @ 65MHz */
	TAL_CMOSPAD_DRV_10X = 0x1F        /*!<   25pF load @ 65MHz */
} taliseCmosPadDrvStr_t;

/***************************************************************************//**
 * @brief The `taliseTxDataIfPllUnlock_t` is an enumeration that defines the
 * behavior of the transmit (Tx) data path when the Phase-Locked Loop
 * (PLL) unlocks. It provides three options: not disabling the Tx data,
 * zeroing the Tx data immediately, or ramping down the Tx data to zero.
 * This allows for controlled handling of Tx data in response to PLL
 * unlock events, which is critical for maintaining signal integrity and
 * system stability in RF communication systems.
 *
 * @param TAL_TXDIS_TX_NOT_DISABLED Tx data is not disabled when PLL unlocks.
 * @param TAL_TXDIS_TX_ZERO_DATA Tx data is zeroed immediately when PLL unlocks.
 * @param TAL_TXDIS_TX_RAMP_DOWN_TO_ZERO Tx data is ramped down to zero when PLL
 * unlocks.
 ******************************************************************************/
typedef enum {
	TAL_TXDIS_TX_NOT_DISABLED       = 0x0,      /*!< Tx data is not disabled when PLL unlocks */
	TAL_TXDIS_TX_ZERO_DATA          = 0x1,      /*!< Tx data is zeroed immediately when PLL unlocks */
	TAL_TXDIS_TX_RAMP_DOWN_TO_ZERO  = 0x2       /*!< Tx data is ramped down to zero when PLL unlocks */
} taliseTxDataIfPllUnlock_t;

/***************************************************************************//**
 * @brief The `taliseRfPllMcs_t` is an enumeration that defines the various
 * modes of RFPLL phase synchronization available in the Talise API. It
 * provides options to either disable synchronization or enable it with
 * varying levels of tracking, from initialization only to continuous
 * tracking. This allows for flexibility in managing the phase
 * synchronization of the RFPLL depending on the required accuracy and
 * the time available for the RFPLL to lock when its frequency is
 * changed.
 *
 * @param TAL_RFPLLMCS_NOSYNC Disable RFPLL phase synchronization.
 * @param TAL_RFPLLMCS_INIT_AND_SYNC Enable RFPLL phase sync init only.
 * @param TAL_RFPLLMCS_INIT_AND_1TRACK Enable RFPLL phase sync init and track
 * once.
 * @param TAL_RFPLLMCS_INIT_AND_CONTTRACK Enable RFPLL phase sync init and track
 * continuously.
 ******************************************************************************/
typedef enum {
	TAL_RFPLLMCS_NOSYNC = 0,                /*!< Disable RFPLL phase synchronization */
	TAL_RFPLLMCS_INIT_AND_SYNC = 1,         /*!< Enable RFPLL phase sync init only */
	TAL_RFPLLMCS_INIT_AND_1TRACK = 2,       /*!< Enable RFPLL phase sync init and track once  */
	TAL_RFPLLMCS_INIT_AND_CONTTRACK = 3     /*!< Enable RFPLL phase sync init and track continuously */
} taliseRfPllMcs_t;

/***************************************************************************//**
 * @brief The `taliseFir_t` structure is designed to hold the configuration
 * settings for a Finite Impulse Response (FIR) filter used in the Talise
 * API. It includes the filter's gain in decibels, the number of
 * coefficients that define the filter's response, and a pointer to the
 * array containing these coefficients. This structure is essential for
 * configuring the FIR filter's behavior in signal processing
 * applications within the Talise system.
 *
 * @param gain_dB Filter gain in dB.
 * @param numFirCoefs Number of coefficients in the FIR filter.
 * @param coefs A pointer to an array of filter coefficients.
 ******************************************************************************/
typedef struct {
	int8_t gain_dB;         /*!< Filter gain in dB */
	uint8_t numFirCoefs;    /*!< Number of coefficients in the FIR filter */
	int16_t *coefs;         /*!< A pointer to an array of filter coefficients */
} taliseFir_t;

/***************************************************************************//**
 * @brief The `taliseRxNcoShifterCfg_t` structure is designed to hold
 * configuration parameters for the NCO (Numerically Controlled
 * Oscillator) shifter settings for both BandA and BandB in a Talise
 * device. It includes fields for specifying the input bandwidth and
 * center frequency for each band, as well as the frequency shifts for
 * two NCOs per band. Setting all four element values to zero for a band
 * will disable the NCO for that shift, allowing for flexible
 * configuration of signal processing paths.
 *
 * @param bandAInputBandWidth_kHz BandWidth in kHz of the BandA input signal.
 * @param bandAInputCenterFreq_kHz Center Frequency in kHz of the BandA input
 * signal.
 * @param bandANco1Freq_kHz BandA NCO1 Frequency shift in kHz.
 * @param bandANco2Freq_kHz BandA NCO2 Frequency shift in kHz.
 * @param bandBInputBandWidth_kHz BandWidth in kHz of the BandB input signal.
 * @param bandBInputCenterFreq_kHz Center Frequency in kHz of the BandB input
 * signal.
 * @param bandBNco1Freq_kHz BandB NCO1 Frequency shift in kHz.
 * @param bandBNco2Freq_kHz BandB NCO2 Frequency shift in kHz.
 ******************************************************************************/
typedef struct {
	uint32_t bandAInputBandWidth_kHz;  /*!< BandWidth in khz of the BandA input signal */
	int32_t bandAInputCenterFreq_kHz;      /*!< Center Frequency in khz of the BandA input signal */
	int32_t bandANco1Freq_kHz;         /*!< BandA NCO1 Frequency shift in khz */
	int32_t bandANco2Freq_kHz;         /*!< BandA NCO2 Frequency shift in khz */

	uint32_t bandBInputBandWidth_kHz;  /*!< BandWidth in khz of the BandB input signal */
	int32_t bandBInputCenterFreq_kHz;  /*!< Center Frequency in khz of the BandB input signal */
	int32_t bandBNco1Freq_kHz;         /*!< BandB NCO1 Frequency shift in khz */
	int32_t bandBNco2Freq_kHz;         /*!< BandB NCO2 Frequency shift in khz */

} taliseRxNcoShifterCfg_t;

/***************************************************************************//**
 * @brief The `taliseRxProfile_t` structure is designed to hold the
 * configuration settings for a Talise receiver profile. It includes
 * various parameters such as the FIR filter settings, decimation
 * factors, output data rate, RF bandwidth, and ADC profile settings.
 * Additionally, it contains configurations for the digital downconverter
 * (DDC) mode and NCO shifter parameters, which are essential for signal
 * processing tasks like zero-IF to real-IF conversion. This structure is
 * crucial for defining the operational characteristics of the receiver
 * in a Talise device.
 *
 * @param rxFir Rx FIR filter structure.
 * @param rxFirDecimation Rx FIR decimation (1,2,4).
 * @param rxDec5Decimation Decimation of Dec5 or Dec4 filter (5,4).
 * @param rhb1Decimation RX Halfband1 (HB1) decimation, can be either 1 or 2.
 * @param rxOutputRate_kHz Rx Output data rate in kHz.
 * @param rfBandwidth_Hz Rx RF passband bandwidth for the profile.
 * @param rxBbf3dBCorner_kHz Rx BBF (TIA) 3dB corner in kHz.
 * @param rxAdcProfile Rx ADC Profile - tunes the bandwidth of the passband and
 * noise transfer functions of the ADC.
 * @param rxDdcMode Rx DDC mode.
 * @param rxNcoShifterCfg Rx NCO Shift parameters used for ZIF->RIF, CIF->ZIF.
 ******************************************************************************/
typedef struct {
	taliseFir_t rxFir;			        /*!< Rx FIR filter structure */
	uint8_t rxFirDecimation;            /*!< Rx FIR decimation (1,2,4) */
	uint8_t rxDec5Decimation;           /*!< Decimation of Dec5 or Dec4 filter (5,4) */
	uint8_t rhb1Decimation;             /*!< RX Halfband1 (HB1) decimation. Can be either 1 or 2 */
	uint32_t rxOutputRate_kHz;          /*!< Rx Output data rate in kHz */
	uint32_t rfBandwidth_Hz;            /*!< Rx RF passband bandwidth for the profile */
	uint32_t rxBbf3dBCorner_kHz;        /*!< Rx BBF (TIA) 3dB corner in kHz */
	uint16_t rxAdcProfile[42];          /*!< Rx ADC Profile - tunes the bandwidth of the passband and noise transfer functions of the ADC */
	taliseRxDdc_t rxDdcMode;            /*!< Rx DDC mode */
	taliseRxNcoShifterCfg_t
	rxNcoShifterCfg; /*!< Rx NCO Shift parameters used for ZIF->RIF, CIF->ZIF */
} taliseRxProfile_t;

/***************************************************************************//**
 * @brief The `taliseORxProfile_t` structure is designed to hold configuration
 * settings for the Observation Receiver (ORx) path in a Talise device.
 * It includes parameters for FIR filtering, decimation settings, output
 * data rate, RF bandwidth, and ADC profiles. The structure also supports
 * ADC stitching through its bandpass profile and merge filter, allowing
 * for a wideband ADC transfer function. This configuration is crucial
 * for optimizing the ORx path's performance in terms of bandwidth and
 * data rate, making it suitable for various signal processing
 * applications.
 *
 * @param rxFir ORx FIR filter structure.
 * @param rxFirDecimation ORx FIR decimation factor, can be 1, 2, or 4.
 * @param rxDec5Decimation Decimation factor for Dec5 or Dec4 filter, can be 5
 * or 4.
 * @param rhb1Decimation ORX Halfband1 (HB1) decimation factor, can be 1 or 2.
 * @param orxOutputRate_kHz ORx output data rate in kilohertz.
 * @param rfBandwidth_Hz ORx RF passband bandwidth in hertz.
 * @param rxBbf3dBCorner_kHz ORx BBF (TIA) 3dB corner frequency in kilohertz.
 * @param orxLowPassAdcProfile Array of 42 elements tuning the bandwidth and
 * noise transfer functions of the ORx Lowpass ADC.
 * @param orxBandPassAdcProfile Array of 42 elements tuning the bandwidth and
 * noise transfer functions of the ORx Bandpass
 * ADC, used for ADC stitching.
 * @param orxDdcMode ORx DDC mode setting.
 * @param orxMergeFilter Array of 12 elements for merging Lowpass and Bandpass
 * ADC to obtain a wideband ADC transfer function.
 ******************************************************************************/
typedef struct {
	taliseFir_t rxFir;                  /*!< ORx FIR filter structure */
	uint8_t rxFirDecimation;            /*!< ORx FIR decimation (1,2,4) */
	uint8_t rxDec5Decimation;           /*!< Decimation of Dec5 or Dec4 filter (5,4) */
	uint8_t rhb1Decimation;             /*!< ORX Halfband1 (HB1) decimation. Can be either 1 or 2 */
	uint32_t orxOutputRate_kHz;         /*!< ORx Output data rate in kHz */
	uint32_t rfBandwidth_Hz;            /*!< ORx RF passband bandwidth for the profile */
	uint32_t rxBbf3dBCorner_kHz;        /*!< ORx BBF (TIA) 3dB corner in kHz */
	uint16_t orxLowPassAdcProfile[42];  /*!< ORx Lowpass ADC Profile - tunes the bandwidth of the passband and noise transfer functions of the ADC */
	uint16_t orxBandPassAdcProfile[42]; /*!< ORx Bandpass ADC Profile (ADC stitching only) - tunes the bandwidth of the passband and noise transfer functions of the ADC */
	taliseOrxDdc_t orxDdcMode;          /*!< ORx DDC mode */
	int16_t orxMergeFilter[12];         /*!< ORx Merge filter (ADC stitching only) - Merges the Lowpass and Bandpass ADC to obtain a wideband ADC transfer function */
} taliseORxProfile_t;

/***************************************************************************//**
 * @brief The `taliseTxProfile_t` structure is designed to hold the
 * configuration settings for a specific transmit (Tx) use case profile
 * in a Talise device. It includes various parameters that define the
 * digital and analog signal processing characteristics, such as
 * interpolation factors for different filters, input data rates, and
 * bandwidth settings. The structure also contains a field for the
 * loopback ADC profile, which is used to configure the bandwidth of the
 * ADC response during loopback operations. This structure is crucial for
 * setting up the Tx path to meet specific signal processing
 * requirements.
 *
 * @param dacDiv The divider used to generate the DAC clock (1,2).
 * @param txFir Tx FIR filter structure.
 * @param txFirInterpolation The TX digital FIR filter interpolation (1,2,4).
 * @param thb1Interpolation Tx Halfband1 (HB1) filter interpolation (1,2).
 * @param thb2Interpolation Tx Halfband2 (HB2) filter interpolation (1,2).
 * @param thb3Interpolation Tx Halfband3 (HB3) filter interpolation (1,2).
 * @param txInt5Interpolation Tx Int5 filter interpolation (1,5).
 * @param txInputRate_kHz Tx input data rate in kHz.
 * @param primarySigBandwidth_Hz Tx primary signal bandwidth in Hz.
 * @param rfBandwidth_Hz Tx RF passband bandwidth for the profile in Hz.
 * @param txDac3dBCorner_kHz DAC filter 3dB corner in kHz.
 * @param txBbf3dBCorner_kHz Tx BBF 3dB corner in kHz.
 * @param loopBackAdcProfile Rx Loop Back ADC profile to set the bandwidth of
 * the ADC response.
 ******************************************************************************/
typedef struct {
	uint8_t dacDiv;                     /*!< The divider used to generate the DAC clock (1,2)*/
	taliseFir_t txFir;                  /*!< Tx FIR filter structure */
	uint8_t txFirInterpolation;         /*!< The TX digital FIR filter interpolation (1,2,4) */
	uint8_t thb1Interpolation;          /*!< Tx Halfband1 (HB1) filter interpolation (1,2) */
	uint8_t thb2Interpolation;          /*!< Tx Halfband2 (HB2) filter interpolation (1,2) */
	uint8_t thb3Interpolation;          /*!< Tx Halfband3 (HB3) filter interpolation (1,2) */
	uint8_t txInt5Interpolation;        /*!< Tx Int5 filter interpolation (1,5) */
	uint32_t txInputRate_kHz;           /*!< Tx input data rate in kHz */
	uint32_t primarySigBandwidth_Hz;    /*!< Tx primary signal BW */
	uint32_t rfBandwidth_Hz;            /*!< Tx RF passband bandwidth for the profile */
	uint32_t txDac3dBCorner_kHz;        /*!< DAC filter 3dB corner in kHz */
	uint32_t txBbf3dBCorner_kHz;        /*!< Tx BBF 3dB corner in kHz */
	uint16_t loopBackAdcProfile[42];    /*!< Rx Loop Back ADC profile to set the bandwidth of the ADC response */
} taliseTxProfile_t;

/***************************************************************************//**
 * @brief The `taliseORxGainControl_t` structure is designed to manage the gain
 * control settings for observation receivers (ORx) in a Talise device.
 * It includes fields for setting the current gain mode and indices for
 * both ORx1 and ORx2, allowing for both manual and automatic gain
 * control (AGC). The structure also specifies the maximum and minimum
 * gain indices for the gain tables associated with ORx1 and ORx2,
 * providing flexibility in gain management and ensuring that the gain
 * settings remain within the defined limits of the gain tables.
 *
 * @param gainMode Current Rx gain control mode setting.
 * @param orx1GainIndex ORx1 Gain Index, can be used in different ways for
 * manual and AGC gain control.
 * @param orx2GainIndex ORx2 Gain Index, can be used in different ways for
 * manual and AGC gain control.
 * @param orx1MaxGainIndex Max gain index for the currently loaded ORx1 Gain
 * table.
 * @param orx1MinGainIndex Min gain index for the currently loaded ORx1 Gain
 * table.
 * @param orx2MaxGainIndex Max gain index for the currently loaded ORx2 Gain
 * table.
 * @param orx2MinGainIndex Min gain index for the currently loaded ORx2 Gain
 * table.
 ******************************************************************************/
typedef struct {
	taliseGainMode_t gainMode; /*!< Current Rx gain control mode setting */
	uint8_t orx1GainIndex;      /*!< ORx1 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t orx2GainIndex;      /*!< ORx2 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t orx1MaxGainIndex;       /*!< Max gain index for the currently loaded ORx1 Gain table */
	uint8_t orx1MinGainIndex;       /*!< Min gain index for the currently loaded ORx1 Gain table */
	uint8_t orx2MaxGainIndex;       /*!< Max gain index for the currently loaded ORx2 Gain table */
	uint8_t orx2MinGainIndex;       /*!< Min gain index for the currently loaded ORx2 Gain table */
} taliseORxGainControl_t;

/***************************************************************************//**
 * @brief The `taliseRxGainControl_t` structure is designed to manage the gain
 * control settings for a receiver (Rx) in a Talise device. It includes
 * fields for setting the gain mode and managing gain indices for two
 * channels, Rx1 and Rx2. The structure allows for both manual and
 * automatic gain control (AGC) by providing indices for current,
 * maximum, and minimum gain settings for each channel. This enables
 * precise control over the gain settings, which is crucial for
 * optimizing signal reception and maintaining signal integrity in
 * various operating conditions.
 *
 * @param gainMode Current Rx gain control mode setting.
 * @param rx1GainIndex Rx1 Gain Index, can be used in different ways for manual
 * and AGC gain control.
 * @param rx2GainIndex Rx2 Gain Index, can be used in different ways for manual
 * and AGC gain control.
 * @param rx1MaxGainIndex Max gain index for the currently loaded Rx1 Gain
 * table.
 * @param rx1MinGainIndex Min gain index for the currently loaded Rx1 Gain
 * table.
 * @param rx2MaxGainIndex Max gain index for the currently loaded Rx2 Gain
 * table.
 * @param rx2MinGainIndex Min gain index for the currently loaded Rx2 Gain
 * table.
 ******************************************************************************/
typedef struct {
	taliseGainMode_t gainMode;  /*!< Current Rx gain control mode setting */
	uint8_t rx1GainIndex;       /*!< Rx1 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t rx2GainIndex;       /*!< Rx2 Gain Index, can be used in different ways for manual and AGC gain control */
	uint8_t rx1MaxGainIndex;    /*!< Max gain index for the currently loaded Rx1 Gain table */
	uint8_t rx1MinGainIndex;    /*!< Min gain index for the currently loaded Rx1 Gain table */
	uint8_t rx2MaxGainIndex;    /*!< Max gain index for the currently loaded Rx2 Gain table */
	uint8_t rx2MinGainIndex;    /*!< Min gain index for the currently loaded Rx2 Gain table */
} taliseRxGainControl_t;

/***************************************************************************//**
 * @brief The `taliseTxSettings_t` structure is designed to encapsulate the
 * configuration settings for the transmission (Tx) path in a Talise
 * device. It includes the Tx datapath profile, which defines the 3dB
 * corner frequencies and digital filter settings, as well as the
 * selection of the JESD204b deframer. The structure also specifies which
 * Tx channels should be enabled during initialization, the step size for
 * Tx attenuation, and the initial and current attenuation levels for Tx1
 * and Tx2. Additionally, it provides options to disable Tx data if the
 * RFPLL unlocks, ensuring robust operation under various conditions.
 *
 * @param txProfile Tx datapath profile, 3dB corner frequencies, and digital
 * filter enables.
 * @param deframerSel Talise JESD204b deframer select (Deframer A or B, or
 * both).
 * @param txChannels The desired Tx channels to enable during initialization.
 * @param txAttenStepSize Tx Attenuation step size.
 * @param tx1Atten_mdB Initial and current Tx1 Attenuation.
 * @param tx2Atten_mdB Initial and current Tx2 Attenuation.
 * @param disTxDataIfPllUnlock Options to disable Transmit data when the RFPLL
 * unlocks.
 ******************************************************************************/
typedef struct {
	taliseTxProfile_t
	txProfile;              /*!< Tx datapath profile, 3dB corner frequencies, and digital filter enables */
	taliseDeframerSel_t
	deframerSel;          /*!< Talise JESD204b deframer select (Deframer A or B, or both) */
	taliseTxChannels_t
	txChannels;            /*!< The desired Tx channels to enable during initialization */
	taliseTxAttenStepSize_t txAttenStepSize;  /*!< Tx Attenuation step size */
	uint16_t tx1Atten_mdB;                    /*!< Initial and current Tx1 Attenuation */
	uint16_t tx2Atten_mdB;                    /*!< Initial and current Tx2 Attenuation */
	taliseTxDataIfPllUnlock_t
	disTxDataIfPllUnlock;   /*!< Options to disable Transmit data when the RFPLL unlocks */
} taliseTxSettings_t;

/***************************************************************************//**
 * @brief The `taliseRxSettings_t` structure is designed to encapsulate the
 * settings for the receive (Rx) data path in a Talise device. It
 * includes the Rx profile settings, which define the datapath profile,
 * corner frequencies, and digital filter configurations. The structure
 * also specifies the JESD204b framer configuration through an
 * enumeration, allowing for the selection of the appropriate framer
 * settings. Additionally, it contains a structure for managing Rx gain
 * control, which is crucial for adjusting the gain settings during
 * operation. Finally, it includes a field to specify which Rx channels
 * should be enabled during the initialization process, providing
 * flexibility in configuring the device for different use cases.
 *
 * @param rxProfile Rx datapath profile, 3dB corner frequencies, and digital
 * filter enables.
 * @param framerSel Rx JESD204b framer configuration enum.
 * @param rxGainCtrl Rx Gain control settings structure.
 * @param rxChannels The desired Rx Channels to enable during initialization.
 ******************************************************************************/
typedef struct {
	taliseRxProfile_t
	rxProfile;        /*!< Rx datapath profile, 3dB corner frequencies, and digital filter enables */
	taliseFramerSel_t
	framerSel;        /*!< Rx JESD204b framer configuration enum */
	taliseRxGainControl_t rxGainCtrl;   /*!< Rx Gain control settings structure */
	taliseRxChannels_t
	rxChannels;      /*!< The desired Rx Channels to enable during initialization */
} taliseRxSettings_t;

/***************************************************************************//**
 * @brief The `taliseObsRxSettings_t` structure is designed to encapsulate the
 * settings for the Observation Receiver (ORx) data path in a Talise
 * device. It includes configurations for the ORx profile, gain control,
 * and framer settings, as well as the channels to be enabled during
 * initialization. Although the `obsRxLoSource` field is currently
 * reserved for future use, it indicates potential flexibility in
 * selecting the local oscillator source for the ORx mixers. This
 * structure is crucial for setting up the ORx path to ensure optimal
 * performance and integration with the JESD204b interface.
 *
 * @param orxProfile ORx datapath profile, 3dB corner frequencies, and digital
 * filter enables.
 * @param orxGainCtrl ObsRx gain control settings structure.
 * @param framerSel ObsRx JESD204b framer configuration structure.
 * @param obsRxChannelsEnable The desired ObsRx Channel to enable during
 * initialization.
 * @param obsRxLoSource Field not used, reserved for future use; the ORx mixers
 * can use the RF_PLL or Aux_PLL.
 ******************************************************************************/
typedef struct {
	taliseORxProfile_t
	orxProfile;               /*!< ORx datapath profile, 3dB corner frequencies, and digital filter enables. */
	taliseORxGainControl_t
	orxGainCtrl;         /*!< ObsRx gain control settings structure */
	taliseFramerSel_t
	framerSel;                /*!< ObsRx JESD204b framer configuration structure */
	taliseObsRxChannels_t
	obsRxChannelsEnable;  /*!< The desired ObsRx Channel to enable during initialization */
	taliseObsRxLoSource_t
	obsRxLoSource;        /*!< Field not used, reserved for future use. The ORx mixers can use the RF_PLL or Aux_PLL */
} taliseObsRxSettings_t;

/***************************************************************************//**
 * @brief The `taliseDigClocks_t` structure is designed to manage digital clock
 * settings for a Talise device, including the reference clock frequency,
 * VCO frequency, and high-speed clock divider settings. It also
 * specifies whether an external LO input is used for the RF PLL and the
 * mode for RF PLL phase synchronization, which can impact the time
 * required to lock the RF PLL when its frequency is altered.
 *
 * @param deviceClock_kHz CLKPLL and device reference clock frequency in kHz.
 * @param clkPllVcoFreq_kHz CLKPLL VCO frequency in kHz.
 * @param clkPllHsDiv CLKPLL high speed clock divider.
 * @param rfPllUseExternalLo Indicates whether to use external LO input for RF
 * PLL (1) or internal LO generation (0).
 * @param rfPllPhaseSyncMode Sets RF PLL phase synchronization mode, affecting
 * lock time when PLL frequency changes.
 ******************************************************************************/
typedef struct {
	uint32_t deviceClock_kHz;       /*!< CLKPLL and device reference clock frequency in kHz */
	uint32_t clkPllVcoFreq_kHz;     /*!< CLKPLL VCO frequency in kHz */
	taliseHsDiv_t  clkPllHsDiv;     /*!< CLKPLL high speed clock divider */
	uint8_t rfPllUseExternalLo;     /*!< 1= Use external LO input for RF PLL, 0 = use internal LO generation for RF PLL */
	taliseRfPllMcs_t
	rfPllPhaseSyncMode;   /*!< Set RF PLL phase synchronization mode. Adds extra time to lock RF PLL when PLL frequency changed. See enum for options */
} taliseDigClocks_t;

/***************************************************************************//**
 * @brief The `taliseClocks_t` structure is designed to hold various clock-
 * related settings and calculated values for a Talise device. It
 * includes fields for the device's reference clock frequency, the VCO
 * frequency of the clock PLL, and the high-speed clock divider setting.
 * Additionally, it stores calculated digital clock frequencies used
 * throughout the API, as well as a flag indicating whether an external
 * LO input is used for the RF PLL. This structure is essential for
 * managing and configuring the clock settings of the device, ensuring
 * proper synchronization and operation.
 *
 * @param deviceClock_kHz CLKPLL and device reference clock frequency in kHz.
 * @param clkPllVcoFreq_kHz CLKPLL VCO frequency in kHz.
 * @param clkPllHsDiv CLKPLL high speed clock divider.
 * @param hsDigClkDiv2_Hz Calculated digital clock used throughout API
 * functions, divided by 2.
 * @param hsDigClkDiv4or5_Hz Calculated digital clock used throughout API
 * functions, divided by 4 or 5.
 * @param rfPllUseExternalLo Indicates whether to use external LO input for RF
 * PLL (1) or internal LO generation (0).
 ******************************************************************************/
typedef struct {
	uint32_t deviceClock_kHz;       /*!< CLKPLL and device reference clock frequency in kHz */
	uint32_t clkPllVcoFreq_kHz;     /*!< CLKPLL VCO frequency in kHz */
	taliseHsDiv_t  clkPllHsDiv;     /*!< CLKPLL high speed clock divider */
	uint32_t hsDigClkDiv2_Hz;       /*!< Calculated in TALISE_initialize() digital clock used throughout API functions */
	uint32_t hsDigClkDiv4or5_Hz;    /*!< Calculated in TALISE_initialize() digital clock used throughout API functions */
	uint8_t rfPllUseExternalLo;     /*!< Stored version of the init struct setting (Ext LO input) to keep in the device data structure, 1= Use external LO input for RF PLL, 0 = use internal LO generation for RF PLL */
} taliseClocks_t;

/***************************************************************************//**
 * @brief The `taliseSpiSettings_t` structure is designed to configure SPI
 * communication settings for Talise devices. It includes options for
 * setting the bit order, enabling SPI streaming, configuring address
 * increment direction, and selecting between 3-wire and 4-wire SPI
 * modes. Additionally, it allows for the specification of the drive
 * strength for CMOS pads used as outputs, ensuring compatibility and
 * performance across different hardware configurations.
 *
 * @param MSBFirst Determines the bit order for SPI communication, where 1
 * indicates MSB first and 0 indicates LSB first.
 * @param enSpiStreaming Enables or disables SPI streaming, which is not
 * recommended as most registers in the Talise API are not
 * consecutive.
 * @param autoIncAddrUp Sets the address increment direction for SPI streaming,
 * where 1 means incrementing the address and 0 means
 * decrementing it.
 * @param fourWireMode Specifies whether to use 4-wire SPI (1) or 3-wire SPI
 * (0), with the note that ADI's FPGA platform always uses
 * 4-wire mode.
 * @param cmosPadDrvStrength Defines the drive strength of CMOS pads when used
 * as outputs, such as SDIO, SDO, GP_INTERRUPT, GPIO
 * 1, and GPIO 0.
 ******************************************************************************/
typedef struct {
	uint8_t MSBFirst;                           /*!< 1 = MSBFirst, 0 = LSBFirst */
	uint8_t enSpiStreaming;                     /*!< Not Recommended - most registers in Talise API are not consecutive */
	uint8_t autoIncAddrUp;                      /*!< For SPI Streaming, set address increment direction. 1= next addr = addr+1, 0:addr = addr-1 */
	uint8_t fourWireMode;                       /*!< 1: Use 4-wire SPI, 0: 3-wire SPI (SDIO pin is bidirectional). NOTE: ADI's FPGA platform always uses 4-wire mode */
	taliseCmosPadDrvStr_t
	cmosPadDrvStrength;   /*!< Drive strength of CMOS pads when used as outputs (SDIO, SDO, GP_INTERRUPT, GPIO 1, GPIO 0) */
} taliseSpiSettings_t;

/***************************************************************************//**
 * @brief The `taliseGainIndex_t` structure is designed to hold the minimum and
 * maximum gain index values for both Rx and Orx channels of a Talise
 * device. It includes fields for Rx1, Rx2, Orx1, and Orx2 channels,
 * allowing for precise control and configuration of gain settings across
 * different channels in the device. This structure is essential for
 * managing the gain parameters in applications that require dynamic gain
 * adjustments.
 *
 * @param rx1MinGainIndex Current device minimum Rx1 gain index.
 * @param rx1MaxGainIndex Current device maximum Rx1 gain index.
 * @param rx2MinGainIndex Current device minimum Rx2 gain index.
 * @param rx2MaxGainIndex Current device maximum Rx2 gain index.
 * @param orx1MinGainIndex Current device minimum Orx1 gain index.
 * @param orx1MaxGainIndex Current device maximum Orx1 gain index.
 * @param orx2MinGainIndex Current device minimum Orx2 gain index.
 * @param orx2MaxGainIndex Current device maximum Orx2 gain index.
 ******************************************************************************/
typedef struct {
	uint8_t rx1MinGainIndex;         /*!< Current device minimum Rx1 gain index */
	uint8_t rx1MaxGainIndex;         /*!< Current device maximum Rx1 gain index */
	uint8_t rx2MinGainIndex;         /*!< Current device minimum Rx2 gain index */
	uint8_t rx2MaxGainIndex;         /*!< Current device maximum Rx2 gain index */
	uint8_t orx1MinGainIndex;        /*!< Current device minimum Orx1 gain index */
	uint8_t orx1MaxGainIndex;        /*!< Current device maximum Orx1 gain index */
	uint8_t orx2MinGainIndex;        /*!< Current device minimum Orx2 gain index */
	uint8_t orx2MaxGainIndex;        /*!< Current device maximum Orx2 gain index */
} taliseGainIndex_t;

/***************************************************************************//**
 * @brief The `taliseInit_t` structure is a comprehensive configuration data
 * structure used to initialize a Talise device instance. It encapsulates
 * various settings required for the device's operation, including SPI
 * communication settings, receive (Rx) and transmit (Tx) path
 * configurations, observation receiver (ObsRx) settings, digital clock
 * configurations, and JESD204B data link settings. This structure is
 * essential for setting up the device's hardware and communication
 * interfaces, ensuring that all necessary parameters are defined for the
 * device to function correctly in its intended application.
 *
 * @param spiSettings SPI settings data structure.
 * @param rx Rx settings data structure.
 * @param tx Tx settings data structure.
 * @param obsRx ObsRx settings data structure.
 * @param clocks Holds settings for CLKPLL and reference clock.
 * @param jesd204Settings Holds the JESD204B data link settings.
 ******************************************************************************/
typedef struct {
	taliseSpiSettings_t   spiSettings;     /*!< SPI settings data structure */
	taliseRxSettings_t    rx;              /*!< Rx settings data structure */
	taliseTxSettings_t    tx;              /*!< Tx settings data structure */
	taliseObsRxSettings_t obsRx;           /*!< ObsRx settings data structure */
	taliseDigClocks_t
	clocks;          /*!< Holds settings for CLKPLL and reference clock */
	taliseJesdSettings_t
	jesd204Settings; /*!< Holds the JESD204B data link settings */
} taliseInit_t;

/***************************************************************************//**
 * @brief The `talErrorFunction_t` structure is designed to encapsulate an error
 * handling mechanism within the Talise API. It contains an error source
 * identifier and a callback function pointer, which is intended to
 * process or handle errors based on the error source and code provided.
 * This structure allows for flexible error management by enabling the
 * registration of custom error handling functions that can be invoked
 * when specific errors occur.
 *
 * @param errSrc A 32-bit unsigned integer representing the error source.
 * @param callbackFunction A pointer to a function that takes two 32-bit
 * unsigned integers as parameters and returns a
 * constant character pointer.
 ******************************************************************************/
typedef struct {
	uint32_t errSrc;
	const char* (*callbackFunction)(uint32_t errSrc, uint32_t errCode);
} talErrorFunction_t;

#define TALISE_ERRORFUNCTIONTABLEMAX 2

/***************************************************************************//**
 * @brief The `talErrFunctionTable_t` structure is designed to hold a registry
 * of error message functions for add-on features in the Talise API. It
 * contains an array of `talErrorFunction_t` structures, which are used
 * to store error callback functions. Each entry in the array can hold a
 * function pointer and an error source identifier, allowing the system
 * to handle errors by invoking the appropriate callback function. The
 * size of the array is defined by the constant
 * `TALISE_ERRORFUNCTIONTABLEMAX`, ensuring a fixed number of error
 * functions can be registered.
 *
 * @param talErrorFunctionTable An array of talErrorFunction_t structures with a
 * fixed size of TALISE_ERRORFUNCTIONTABLEMAX.
 ******************************************************************************/
typedef struct {
	talErrorFunction_t talErrorFunctionTable[TALISE_ERRORFUNCTIONTABLEMAX];
} talErrFunctionTable_t;

/***************************************************************************//**
 * @brief The `talFrequencyHoppingRange_t` structure is used to define the
 * frequency range for Frequency Hopping Mode (FHM) in a Talise device.
 * It contains two members, `fhmMinFreq_MHz` and `fhmMaxFreq_MHz`, which
 * specify the minimum and maximum frequencies, respectively, for the
 * frequency hopping operation in megahertz. This structure is essential
 * for configuring the frequency limits within which the device can hop
 * during operation, ensuring that the frequency hopping stays within the
 * desired spectrum range.
 *
 * @param fhmMinFreq_MHz Lower limit for FHM hop frequency in MHz.
 * @param fhmMaxFreq_MHz Upper limit for FHM hop frequency in MHz.
 ******************************************************************************/
typedef struct {
	uint32_t fhmMinFreq_MHz; /*!< Lower limit for FHM hop frequency in MHz */
	uint32_t fhmMaxFreq_MHz; /*!< Upper limit for FHM hop frequency in MHz */
} talFrequencyHoppingRange_t;

/***************************************************************************//**
 * @brief The `taliseInfo_t` structure is a comprehensive data structure used to
 * hold the status and configuration information of a Talise device
 * instance. It includes fields for device state, initialized channels,
 * error codes, digital clock settings, gain modes, and bandwidths, among
 * others. This structure is essential for managing and monitoring the
 * operational parameters and status of the Talise device, providing a
 * centralized repository for all relevant device information and
 * settings.
 *
 * @param devState Current device state of the part, defined by the
 * taliseStates_t enum.
 * @param initializedChannels Holds Rx/ORx/Tx channels that were initialized and
 * calibrated for the current device.
 * @param profilesValid Current device profilesValid bit field for use
 * notification.
 * @param errSource Current source of error returned.
 * @param errCode Current error code returned.
 * @param clocks Currently calculated Talise digital clocks for the selected
 * init profiles.
 * @param gainMode Current device gain mode, which can be MGC, AGCFAST, AGCSLOW,
 * or HYBRID.
 * @param gainIndexes Current device gain index values.
 * @param txAttenStepSize Current tx Atten step size.
 * @param orxAdcStitchingEnabled Indicates if ORx ADC stitching is enabled.
 * @param usedGpiopins Indicates whether each GPIO pin is assigned to a
 * function.
 * @param usedGpio3p3pins Indicates whether each GPIO3.3 pin is assigned to a
 * function.
 * @param rxFramerNp Rx Framer Np - converter sample resolution.
 * @param orxFramerNp ORx Framer Np - converter sample resolution.
 * @param rxOutputRate_kHz Rx Output data rate in kHz.
 * @param txInputRate_kHz Tx input data rate in kHz.
 * @param rxDdcMode DDC Mode saved from initialization.
 * @param rxDualBandEnabled Indicates if DualBand Mode is enabled.
 * @param rxTotalM Value of M saved from initialization.
 * @param rxBandwidth_Hz Rx Bandwidth from the current profile.
 * @param txBandwidth_Hz Tx Bandwidth from the current profile.
 * @param orxBandwidth_Hz ORx Bandwidth from the current profile.
 * @param swTest Software testmode signal.
 * @param deviceSiRev Talise silicon revision read during initialization.
 * @param talErrFunctionTable Talise callback function table.
 * @param talFhmFreqRange Talise FHM frequency range.
 * @param talFhmTriggerMode Talise frequency hop trigger mode.
 * @param talFhmInitHopFreq_Hz Initial Talise hop frequency.
 * @param talFhmMcsSync Indicates if MCS synchronization is enabled on frequency
 * hopping.
 ******************************************************************************/
typedef struct {
	taliseStates_t
	devState;				/*!< Current device state of the part, i.e., radio on, radio off, arm loaded, etc., defined by deviceState enum */
	uint8_t initializedChannels;                /*!< Holds Rx/ORx/Tx channels that were initialized and calibrated for the current device */
	uint8_t profilesValid;				/*!< Current device profilesValid bit field for use notification, i.e., Tx = 0x01, Rx = 0x02, Orx = 0x04 */
	uint32_t errSource;					/*!< Current source of error returned */
	uint32_t errCode;					/*!< Current error code returned */
	taliseClocks_t
	clocks;				/*!< Currently calculated Talise digital clocks for the selected init profiles */
	taliseGainMode_t
	gainMode;				/*!< Current device gain mode, which can be MGC, AGCFAST, AGCSLOW, or HYBRID */
	taliseGainIndex_t gainIndexes;			/*!< Current device gain index values */
	taliseTxAttenStepSize_t txAttenStepSize;	/*!< Current tx Atten step size */
	uint8_t orxAdcStitchingEnabled;             /*!< TALISE_initialize sets to 1 if ORx ADC stitching enabled, 0 if not required */
	uint32_t usedGpiopins;                      /*!< Each bit position 'N' indicates whether the GPIO 'N' is assigned to some function (if 1) or not (if 0) */
	uint16_t usedGpio3p3pins;                   /*!< Each bit position 'N' indicates whether the GPIO3.3 'N' is assigned to some function (if 1) or not (if 0) */
	uint8_t rxFramerNp;                         /*!< Rx Framer Np - converter sample resolution (12, 16, 24) */
	uint8_t orxFramerNp;                        /*!< ORx Framer Np - converter sample resolution (12, 16, 24) */
	uint32_t rxOutputRate_kHz;                  /*!< Rx Output data rate in kHz */
	uint32_t txInputRate_kHz;                   /*!< Tx input data rate in kHz */
	taliseRxDdc_t
	rxDdcMode;                    /*!< DDC Mode saved from initialization  */
	uint8_t rxDualBandEnabled;                  /*!< DualBand Mode saved from initialization for determining dualband status */
	uint8_t rxTotalM;                           /*!< Value of M saved from initialization  */
	uint32_t rxBandwidth_Hz;                    /*!< Rx Bandwidth from the current profile */
	uint32_t txBandwidth_Hz;                    /*!< Tx Bandwidth from the current profile */
	uint32_t orxBandwidth_Hz;                   /*!< ORx Bandwidth from the current profile */
	uint32_t swTest;                            /*!< Software testmode signal */
	uint8_t deviceSiRev;                        /*!< Talise silicon rev read during TALISE_initialize */
	talErrFunctionTable_t
	talErrFunctionTable;  /*!< Talise  callback function table */
	talFrequencyHoppingRange_t talFhmFreqRange; /*!< Talise FHM frequency range */
	taliseFhmTriggerMode_t
	talFhmTriggerMode;   /*!< Talise frequency hop trigger mode */
	uint64_t talFhmInitHopFreq_Hz;              /*!< Initial Talise hop frequency */
	uint8_t talFhmMcsSync;                      /*!< Flag to indicate if MCS synchronization is enabled on frequency hopping */
} taliseInfo_t;

/***************************************************************************//**
 * @brief The `taliseDevice_t` structure is designed to encapsulate the settings
 * and state information for a specific instance of a Talise device. It
 * includes a pointer to hardware layer settings (`devHalInfo`) that are
 * specific to the instance, allowing for hardware abstraction and
 * configuration. Additionally, it contains a `taliseInfo_t` structure
 * (`devStateInfo`) that holds detailed status information about the
 * device, such as its current state, initialized channels, error codes,
 * and other operational parameters. This structure is essential for
 * managing and interacting with the Talise device within the API.
 *
 * @param devHalInfo A pointer to the ADI_HAL hardware layer settings specific
 * to this Talise instance.
 * @param devStateInfo A container for Talise device instance status
 * information.
 ******************************************************************************/
typedef struct {
	void *devHalInfo;               /*!< ADI_HAL Hardware layer settings pointer specific to this Talise instance */
	taliseInfo_t devStateInfo;      /*!< Talise infomration container */
} taliseDevice_t;

#ifdef __cplusplus
}
#endif

#endif
