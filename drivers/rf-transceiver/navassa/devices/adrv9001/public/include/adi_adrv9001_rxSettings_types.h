/**
 * \file
 * \brief Type definitions for ADRV9001 Rx settings
 * \copyright Analog Devices Inc. 2019. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_RXSETTINGS_TYPES_H_
#define _ADI_ADRV9001_RXSETTINGS_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_adrv9001_pfirBuffer_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSignalType_e` is an enumeration that defines the
 * types of signals that can be received by the ADRV9001 device. It
 * includes three possible signal types: IQ, frequency deviation, and FM
 * symbols, each represented by a unique integer value. This enumeration
 * is used to specify the type of signal processing to be applied in the
 * receiver path of the ADRV9001.
 *
 * @param ADI_ADRV9001_RX_IQ Represents the IQ signal type with a value of 0.
 * @param ADI_ADRV9001_RX_FREQUENCY_DEVIATION Represents the frequency deviation
 * signal type with a value of 1.
 * @param ADI_ADRV9001_RX_FM_SYMBOLS Represents the FM symbols signal type with
 * a value of 2.
 ******************************************************************************/
typedef enum adi_adrv9001_RxSignalType
{
    ADI_ADRV9001_RX_IQ    = 0,
    ADI_ADRV9001_RX_FREQUENCY_DEVIATION = 1,
    ADI_ADRV9001_RX_FM_SYMBOLS = 2
} adi_adrv9001_RxSignalType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_AdcType_e` is an enumeration that defines the types
 * of ADCs (Analog-to-Digital Converters) available in the ADRV9001
 * system. It provides two options: a Low Power ADC
 * (`ADI_ADRV9001_ADC_LP`) and a High Power ADC (`ADI_ADRV9001_ADC_HP`).
 * This enumeration is used to specify the ADC type in various
 * configurations and settings within the ADRV9001 API.
 *
 * @param ADI_ADRV9001_ADC_LP Represents a Low Power ADC with a value of 0.
 * @param ADI_ADRV9001_ADC_HP Represents a High Power ADC with a value of 1.
 ******************************************************************************/
typedef enum adi_adrv9001_AdcType
{
    ADI_ADRV9001_ADC_LP = 0, /*!< Low Power ADC */
    ADI_ADRV9001_ADC_HP = 1  /*!< High Power ADC */
} adi_adrv9001_AdcType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Adc_LowPower_CalMode_e` is an enumeration that
 * defines the available calibration modes for the low power ADC in the
 * ADRV9001 device. It provides two options: periodic and continuous
 * calibration, allowing users to select the appropriate mode based on
 * their application requirements. This enumeration is part of the
 * broader ADRV9001 API, which is used to configure and control the
 * ADRV9001 transceiver.
 *
 * @param ADI_ADRV9001_ADC_LOWPOWER_PERIODIC Represents a periodic calibration
 * mode for the low power ADC.
 * @param ADI_ADRV9001_ADC_LOWPOWER_CONTINUOUS Represents a continuous
 * calibration mode for the low
 * power ADC.
 ******************************************************************************/
typedef enum  adi_adrv9001_Adc_LowPower_CalMode
{
    ADI_ADRV9001_ADC_LOWPOWER_PERIODIC,
    ADI_ADRV9001_ADC_LOWPOWER_CONTINUOUS
} adi_adrv9001_Adc_LowPower_CalMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxNbDecTop_t` structure is designed to configure the
 * narrowband decimation top block settings for the ADRV9001 receiver. It
 * includes various enable flags and configuration parameters for
 * specific blocks within the decimation chain, allowing for fine-tuning
 * of the decimation process to optimize performance for different use
 * cases. Each member corresponds to a specific block in the decimation
 * chain, providing control over enabling the block and setting its
 * operational parameters.
 *
 * @param scicBlk23En Setting for block #23, RSCIC.
 * @param scicBlk23DivFactor STATIC or DYNAMIC, determined by BBIC based on user
 * case.
 * @param scicBlk23LowRippleEn Enable low ripple mode.
 * @param decBy2Blk35En Setting for block #35, DEC2_1.
 * @param decBy2Blk37En Setting for block #37, DEC2_2.
 * @param decBy2Blk39En Setting for block #39, DEC2_3.
 * @param decBy2Blk41En Setting for block #41, DEC2_4.
 * @param decBy2Blk43En Setting for block #43, DEC2_5.
 * @param decBy3Blk45En Setting for block #45, DEC3.
 * @param decBy2Blk47En Setting for block #47, DEC2_6.
 ******************************************************************************/
typedef struct adi_adrv9001_RxNbDecTop
{
    uint8_t scicBlk23En;			/*!< Setting for block #23. RSCIC */
    uint8_t scicBlk23DivFactor;		/*!< STATIC or DYNAMIC, BBIC to determine based on user case */
    uint8_t scicBlk23LowRippleEn;	/*!< Enable low ripple mode */
    uint8_t decBy2Blk35En;			/*!< Setting for block #35. DEC2_1 */
    uint8_t decBy2Blk37En;			/*!< Setting for block #37. DEC2_2 */
    uint8_t decBy2Blk39En;			/*!< Setting for block #39. DEC2_3 */
    uint8_t decBy2Blk41En;			/*!< Setting for block #41. DEC2_4 */
    uint8_t decBy2Blk43En;			/*!< Setting for block #43. DEC2_5 */
    uint8_t decBy3Blk45En;			/*!< Setting for block #45. DEC3   */
    uint8_t decBy2Blk47En;			/*!< Setting for block #47. DEC2_6 */
} adi_adrv9001_RxNbDecTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxWbDecTop_t` structure is designed to configure the
 * wideband decimation top block settings for the ADRV9001 receiver. It
 * includes several fields that enable specific decimation blocks (DEC2_1
 * to DEC2_5) and a wideband low-pass filter (WBLPF) block, allowing for
 * fine-tuned control over the decimation process in the receiver's
 * signal processing chain.
 *
 * @param decBy2Blk25En Setting for block #25, enabling DEC2_1.
 * @param decBy2Blk27En Setting for block #27, enabling DEC2_2.
 * @param decBy2Blk29En Setting for block #29, enabling DEC2_3.
 * @param decBy2Blk31En Setting for block #31, enabling DEC2_4.
 * @param decBy2Blk33En Setting for block #33, enabling DEC2_5.
 * @param wbLpfBlk33p1En Setting for block #33.1, enabling WBLPF.
 ******************************************************************************/
typedef struct adi_adrv9001_RxWbDecTop
{
    uint8_t decBy2Blk25En;	/*!< Setting for block #25. DEC2_1 */
    uint8_t decBy2Blk27En;	/*!< Setting for block #27. DEC2_2 */
    uint8_t decBy2Blk29En;	/*!< Setting for block #29. DEC2_3 */
    uint8_t decBy2Blk31En;	/*!< Setting for block #31. DEC2_4 */
    uint8_t decBy2Blk33En;	/*!< Setting for block #33. DEC2_5 */
    uint8_t wbLpfBlk33p1En; /*!< Setting for block #33.1. WBLPF */
} adi_adrv9001_RxWbDecTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDecTop_t` structure is designed to hold
 * configuration settings for the ADRV9001 RX decimation top block. It
 * includes several fields that enable or configure specific decimation
 * and half-band filter blocks within the RX signal processing chain.
 * Each field corresponds to a specific block in the decimation process,
 * allowing for fine-tuned control over the signal processing path.
 *
 * @param decBy3Blk15En Setting for block #15, DEC2.
 * @param decBy2Hb3Blk17p1En Setting for block #17.1, HB3.
 * @param decBy2Hb4Blk17p2En Setting for block #17.2, HB4.
 * @param decBy2Hb5Blk19p1En Setting for block #19.1, HB5.
 * @param decBy2Hb6Blk19p2En Setting for block #19.2, HB6.
 ******************************************************************************/
typedef struct adi_adrv9001_RxDecTop
{
    uint8_t decBy3Blk15En;		/*!< Setting for block #15.   DEC2 */
    uint8_t decBy2Hb3Blk17p1En; /*!< Setting for block #17.1. HB3 */
    uint8_t decBy2Hb4Blk17p2En; /*!< Setting for block #17.2. HB4 */
    uint8_t decBy2Hb5Blk19p1En; /*!< Setting for block #19.1. HB5 */
    uint8_t decBy2Hb6Blk19p2En; /*!< Setting for block #19.2. HB6 */
} adi_adrv9001_RxDecTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSincMux5Output_e` is an enumeration that defines
 * the possible output selections for the SINC_MUX5 in the ADRV9001
 * receiver settings. Each enumerator corresponds to a specific output
 * option, such as zero output or selecting one of the SINC filters
 * (SINC3, SINC4, or SINC6) for the output. The values of the enumerators
 * are directly mapped to the actual register values, ensuring that the
 * correct output is selected in the hardware configuration.
 *
 * @param ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO Output zero.
 * @param ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC3 Select SINC3 for the output.
 * @param ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC4 Select SINC4 for the output.
 * @param ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6 Select SINC6 for the output.
 ******************************************************************************/
typedef enum adi_adrv9001_RxSincMux5Output
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_ZERO = 0u,  /*!< Output zero   */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC3 = 1u, /*!< Select SINC3 for the output     */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC4 = 2u, /*!< Select SINC4 for the output     */
    ADI_ADRV9001_RX_SINC_MUX5_OUTPUT_SINC6 = 4u, /*!< Select SINC6 for the toutput    */
} adi_adrv9001_RxSincMux5Output_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSincGainMuxOutput_e` is an enumeration that
 * defines the possible gain settings for the SINC filter in the ADRV9001
 * receiver. Each enumerator corresponds to a specific gain value, which
 * is used to configure the gain of the SINC filter in the receiver's
 * signal processing chain. The values are directly mapped to the actual
 * register values used in the hardware, ensuring that the software
 * configuration matches the hardware settings.
 *
 * @param ADI_ADRV9001_RX_SINC_GAIN_MUX_0_DB Represents a 0 dB gain setting.
 * @param ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB Represents a 6 dB gain setting.
 * @param ADI_ADRV9001_RX_SINC_GAIN_MUX_12_DB Represents a 12 dB gain setting.
 * @param ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_6_DB Represents a -6 dB gain
 * setting.
 * @param ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_12_DB Represents a -12 dB gain
 * setting.
 ******************************************************************************/
typedef enum adi_adrv9001_RxSincGainMuxOutput
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_0_DB      = 0u,   /*!< 0 dB */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_6_DB      = 1u,   /*!< 6 dB */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_12_DB     = 2u,   /*!< 12 dB */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_6_DB  = 3u,   /*!< -6 dB */
    ADI_ADRV9001_RX_SINC_GAIN_MUX_NEG_12_DB = 4u    /*!< -12 dB */
} adi_adrv9001_RxSincGainMuxOutput_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxHBMuxOutput_e` is an enumeration that defines the
 * possible output selections for the half-band (HB) multiplexer in the
 * ADRV9001 receiver configuration. Each enumerator corresponds to a
 * specific output option, with values directly matching the register
 * values used in the hardware configuration. This enum is used to
 * configure which HB output is selected, providing options to output
 * zero or select from HB0, HB1, or HB2.
 *
 * @param ADI_ADRV9001_RX_HB_MUX_OUTPUT_ZERO Output zero.
 * @param ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB0 Select HB0 for the output.
 * @param ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2 Select HB2 for the output.
 * @param ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1 Select HB1 for the output.
 ******************************************************************************/
typedef enum adi_adrv9001_RxHBMuxOutput
{
    /* DO NOT modify the value of the enum as it matches the actual register value */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_ZERO = 0u, /*!< Output zero   */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB0 = 1u,  /*!< Select HB0 for the output     */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB2 = 2u,  /*!< Select HB2 for the output     */
    ADI_ADRV9001_RX_HB_MUX_OUTPUT_HB1 = 4u   /*!< Select HB1 for the toutput    */
} adi_adrv9001_RxHBMuxOutput_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxWbNbPfirInSel_e` is an enumeration that defines
 * the input selection for the PFIR (Programmable Finite Impulse
 * Response) filter in the ADRV9001 receiver. It provides two options:
 * selecting the PFIR input from the RXNB NCO (Numerically Controlled
 * Oscillator) or from an external RP (Reference Point). This selection
 * is crucial for configuring the signal path in the receiver's digital
 * processing chain.
 *
 * @param ADI_ADRV9001_RX_WB_NB_PFIR_SEL_INT_IN PFIR input from RXNB NCO.
 * @param ADI_ADRV9001_RX_WB_NB_PFIR_SEL_EXT_IN PFIR input from RP point.
 ******************************************************************************/
typedef enum adi_adrv9001_RxWbNbPfirInSel
{
    ADI_ADRV9001_RX_WB_NB_PFIR_SEL_INT_IN = 0u, /*!< PFIR input from RXNB NCO */
    ADI_ADRV9001_RX_WB_NB_PFIR_SEL_EXT_IN = 1u, /*!< PFIR input from RP point */
} adi_adrv9001_RxWbNbPfirInSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSincHbTop_t` structure is used to configure the
 * Sinc Halfband top block in the ADRV9001 receiver. It includes settings
 * for the Sinc gain, Sinc MUX, and HB MUX, as well as a flag to enable
 * gain compensation. Additionally, it holds arrays for gain compensation
 * values for both the I and Q channels, allowing for fine-tuning of the
 * signal processing path.
 *
 * @param sincGainMux Sinc gain setting.
 * @param sincMux Sinc MUX selection.
 * @param hbMux HB MUX selection.
 * @param isGainCompEnabled Gain compensation flag.
 * @param gainComp9GainI Array of gain compensation values for the I channel.
 * @param gainComp9GainQ Array of gain compensation values for the Q channel.
 ******************************************************************************/
typedef struct adi_adrv9001_RxSincHbTop
{
    adi_adrv9001_RxSincGainMuxOutput_e	sincGainMux;			/*!< Sinc gain setting */
    adi_adrv9001_RxSincMux5Output_e		sincMux;				/*!< Sinc MUX selection */
    adi_adrv9001_RxHBMuxOutput_e		hbMux;					/*!< HB MUX selection */

    uint8_t								isGainCompEnabled;      /*!< Gain compensation */
    int16_t								gainComp9GainI[ADI_ADRV9001_NUM_GAIN_COMP9];
    int16_t								gainComp9GainQ[ADI_ADRV9001_NUM_GAIN_COMP9];
} adi_adrv9001_RxSincHbTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDpInFifoMode_e` is an enumeration that defines the
 * modes for the RX data path input FIFO in the ADRV9001 device. It
 * includes two modes: 'DETECTING' and 'DETECTED', which are used to
 * indicate the current state of the FIFO in the data path. This
 * enumeration is part of the configuration settings for the ADRV9001's
 * RX channel, allowing the user to specify the operational mode of the
 * FIFO.
 *
 * @param ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING In detecting mode.
 * @param ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTED In detected mode.
 ******************************************************************************/
typedef enum adi_adrv9001_RxDpInFifoMode
{
    ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTING = 0u, /*!< In detecting mode */
    ADI_ADRV9001_DP_IN_FIFO_MODE_DETECTED  = 1u, /*!< In detected mode  */
} adi_adrv9001_RxDpInFifoMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDpInFifoInputSel_e` is an enumeration that defines
 * the input selection for the RX data path input FIFO in the ADRV9001
 * device. It allows the user to choose between receiving data directly
 * from the datapath or from a test pattern provided via SPI. This
 * selection is crucial for configuring the data input source for testing
 * and operational purposes.
 *
 * @param ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL Selects data from the datapath.
 * @param ADI_ADRV9001_DP_IN_FIFO_INPUT_SPI_SEL Selects a test pattern from SPI.
 ******************************************************************************/
typedef enum adi_adrv9001_RxDpInFifoInputSel
{
    ADI_ADRV9001_DP_IN_FIFO_INPUT_DP_SEL  = 0u, /*!< select data from datapath */
    ADI_ADRV9001_DP_IN_FIFO_INPUT_SPI_SEL = 1u, /*!< select test pattern from SPI  */
} adi_adrv9001_RxDpInFifoInputSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDpInFifoConfig_t` structure is used to configure
 * the input FIFO settings for the ADRV9001 receiver. It includes a flag
 * to enable the FIFO, a mode setting to determine the operational mode
 * of the FIFO, and a selection for the input data source, which can be
 * either from the datapath or a test pattern from SPI. This
 * configuration is crucial for managing data flow and testing within the
 * receiver's digital processing path.
 *
 * @param dpInFifoEn A uint8_t flag to enable or disable the dpinfifo.
 * @param dpInFifoMode An enumeration indicating the mode of the dpinfifo.
 * @param dpInFifoTestDataSel An enumeration to select the input data source for
 * the dpinfifo.
 ******************************************************************************/
typedef struct adi_adrv9001_RxDpInFifoConfig
{
    uint8_t								dpInFifoEn;                  /*!< dpinfifo enable */
    adi_adrv9001_RxDpInFifoMode_e		dpInFifoMode;
    adi_adrv9001_RxDpInFifoInputSel_e	dpInFifoTestDataSel;
} adi_adrv9001_RxDpInFifoConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_NcoDpConfig_t` structure is used to configure the
 * Numerically Controlled Oscillator (NCO) in the ADRV9001 device. It
 * includes parameters for setting the output frequency, the sampling
 * frequency to which the NCO is connected, a phase offset, and a flag to
 * enable real output. This configuration is crucial for defining the
 * signal processing characteristics of the NCO within the device's
 * digital signal processing chain.
 *
 * @param freq NCO output frequency.
 * @param sampleFreq Sampling frequency at which the NCO connects.
 * @param phase Phase offset for the NCO.
 * @param realOut Flag to enable real output.
 ******************************************************************************/
typedef struct adi_adrv9001_NcoDpConfig
{
    int32_t  freq;        /*!< NCO output frequency */
    uint32_t sampleFreq;  /*!< Sampling frequency at NCO connects to */
    uint16_t phase;       /*!< Phase offset */
    uint8_t  realOut;     /*!< Real out enable */
} adi_adrv9001_NcoDpConfig_t;

/***************************************************************************//**
 * @brief The adi_adrv9001_RxNbNcoConfig_t structure is used to configure the
 * narrowband Numerically Controlled Oscillator (NCO) settings for the
 * ADRV9001 receiver. It includes a flag to enable or disable the NCO and
 * a configuration structure that specifies the frequency, sample
 * frequency, phase, and output settings for the NCO. This structure is
 * essential for fine-tuning the NCO parameters to achieve the desired
 * signal processing characteristics in narrowband applications.
 *
 * @param rxNbNcoEn A uint8_t flag indicating whether the RX narrowband NCO is
 * enabled.
 * @param rxNbNcoConfig An instance of adi_adrv9001_NcoDpConfig_t that holds the
 * configuration for the RX narrowband NCO.
 ******************************************************************************/
typedef struct adi_adrv9001_RxNbNcoConfig
{
    uint8_t                      rxNbNcoEn;		/*!< rxnb nco enable */
    adi_adrv9001_NcoDpConfig_t rxNbNcoConfig;
} adi_adrv9001_RxNbNcoConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxPfirInMuxSel_e` is an enumeration that defines the
 * input selection for the Programmable Finite Impulse Response (PFIR)
 * filter in the ADRV9001 receiver. It provides two options: selecting
 * the internal input (`ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN`) or the
 * external input (`ADI_ADRV9001_RP_FIR_IN_MUX_EXT_IN`). This selection
 * is crucial for configuring the data path in the receiver to either
 * process signals from within the device or from an external source.
 *
 * @param ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN Represents the internal input
 * selection for the PFIR.
 * @param ADI_ADRV9001_RP_FIR_IN_MUX_EXT_IN Represents the external input
 * selection for the PFIR.
 ******************************************************************************/
typedef enum adi_adrv9001_RxPfirInMuxSel
{
    ADI_ADRV9001_RP_FIR_IN_MUX_INT_IN = 0u,
    ADI_ADRV9001_RP_FIR_IN_MUX_EXT_IN = 1u,
} adi_adrv9001_RxPfirInMuxSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGsOutMuxSel_e` is an enumeration that defines the
 * selection options for the GS output multiplexer in the ADRV9001
 * receiver settings. It provides two options:
 * `ADI_ADRV9001_GS_OUT_MUX_GS_OUT` for selecting the GS_OUT path and
 * `ADI_ADRV9001_GS_OUT_MUX_BYPASS` for bypassing it. This enumeration is
 * used to configure the output path of the GS signal in the receiver's
 * signal chain.
 *
 * @param ADI_ADRV9001_GS_OUT_MUX_GS_OUT Represents the GS_OUT option with a
 * value of 0.
 * @param ADI_ADRV9001_GS_OUT_MUX_BYPASS Represents the BYPASS option with a
 * value of 1.
 ******************************************************************************/
typedef enum adi_adrv9001_RxGsOutMuxSel
{
    ADI_ADRV9001_GS_OUT_MUX_GS_OUT = 0u,
    ADI_ADRV9001_GS_OUT_MUX_BYPASS = 1u,
} adi_adrv9001_RxGsOutMuxSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxOutSel_e` is an enumeration that defines the
 * possible output selections for the ADRV9001 receiver. It allows the
 * user to select between different types of output data: IQ data, FM
 * demodulation data, or PFIR data. This selection is crucial for
 * configuring the receiver's output based on the specific application
 * requirements.
 *
 * @param ADI_ADRV9001_RX_OUT_IQ_SEL IQ out.
 * @param ADI_ADRV9001_RX_OUT_FMDM_SEL FM demodulation out.
 * @param ADI_ADRV9001_RX_OUT_SEL_PFIR PFIR out.
 ******************************************************************************/
typedef enum adi_adrv9001_RxOutSel
{
    ADI_ADRV9001_RX_OUT_IQ_SEL    = 0u, /*!< IQ out */
    ADI_ADRV9001_RX_OUT_FMDM_SEL  = 1u, /*!< FM demodulation out */
    ADI_ADRV9001_RX_OUT_SEL_PFIR  = 2u, /*!< PFIR out */
} adi_adrv9001_RxOutSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxRoundMode_e` is an enumeration that defines the
 * output mode for the ADRV9001 receiver. It specifies whether the output
 * should include both I and Q components (IQ output) or only the I
 * component (I only output). This setting is crucial for configuring the
 * receiver's data output format based on the application requirements.
 *
 * @param ADI_ADRV9001_RX_ROUNDMODE_IQ IQ output mode.
 * @param ADI_ADRV9001_RX_ROUNDMODE_I I only output mode.
 ******************************************************************************/
typedef enum adi_adrv9001_RxRoundMode
{
    ADI_ADRV9001_RX_ROUNDMODE_IQ = 0u, /*!< IQ output */
    ADI_ADRV9001_RX_ROUNDMODE_I  = 1u, /*!< I only output */
} adi_adrv9001_RxRoundMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDpArmSel_e` is an enumeration that defines the
 * selection between two output sources in the ADRV9001 receiver
 * configuration: the datapath output and the ARM output. This selection
 * is crucial for determining the source of the output data in the
 * receiver's signal processing chain.
 *
 * @param ADI_ADRV9001_DP_SEL Selects the datapath output.
 * @param ADI_ADRV9001_ARM_SEL Selects the ARM output.
 ******************************************************************************/
typedef enum adi_adrv9001_RxDpArmSel
{
    ADI_ADRV9001_DP_SEL  = 0u, /*!< select datapath output */
    ADI_ADRV9001_ARM_SEL = 1u, /*!< select arm output */
} adi_adrv9001_RxDpArmSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxWbNbCompPFir_t` structure is used to configure the
 * wideband/narrowband compensation PFIR settings for the ADRV9001
 * receiver. It includes fields to select the PFIR bank, choose the input
 * source for the compensation PFIR, and enable or disable the
 * compensation PFIR functionality. This structure is part of the broader
 * configuration settings for the ADRV9001 receiver, which is a highly
 * integrated RF transceiver designed for a wide range of applications.
 *
 * @param bankSel Selects the PFIR bank to use, with options ranging from
 * PFIR_BANK_A to PFIR_BANK_D.
 * @param rxWbNbCompPFirInMuxSel Determines the input selection for the
 * wideband/narrowband compensation PFIR.
 * @param rxWbNbCompPFirEn Enables or disables the wideband/narrowband
 * compensation PFIR.
 ******************************************************************************/
typedef struct adi_adrv9001_RxWbNbCompPFir
{
    adi_adrv9001_PfirBank_e      bankSel;		/*!< bank: PFIR_BANK_A = 0u (bank A), PFIR_BANK_B = 1u (bank B),
                                                           PFIR_BANK_C = 2u (bank C), PFIR_BANK_D = 3u (bank D) */
    adi_adrv9001_RxPfirInMuxSel_e   rxWbNbCompPFirInMuxSel;
    uint8_t							rxWbNbCompPFirEn;            /*!< rx WbNb compensation PFir enable */
} adi_adrv9001_RxWbNbCompPFir_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxResampConfig_t` structure is used to configure the
 * resampling settings for the ADRV9001 receiver. It includes a flag to
 * enable or disable the resampler and specifies the resampling phase for
 * both the I and Q channels, allowing for precise control over the
 * resampling process in the digital signal processing chain.
 *
 * @param rxResampEn Indicates whether the resampler is enabled.
 * @param resampPhaseI Specifies the phase for the I channel resampler.
 * @param resampPhaseQ Specifies the phase for the Q channel resampler.
 ******************************************************************************/
typedef struct adi_adrv9001_RxResampConfig
{
    uint8_t            rxResampEn;                 /*!< resampler enable */
    uint32_t           resampPhaseI;               /*!< I channel resampler phase */
    uint32_t           resampPhaseQ;               /*!< Q channel resampler phase */
} adi_adrv9001_RxResampConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxNbDemConfig_t` structure is designed to configure
 * the narrowband demodulation block of the ADRV9001 receiver. It
 * includes settings for the data path input FIFO, narrowband NCO,
 * wideband narrowband compensation PFIR, and resampler. Additionally, it
 * provides options for selecting the GS output multiplexer, RX output,
 * rounding mode for the RX output, and whether to use the data path or
 * ARM output. This structure is essential for fine-tuning the receiver's
 * narrowband demodulation capabilities.
 *
 * @param dpInFifo Configuration for the data path input FIFO.
 * @param rxNbNco Configuration for the narrowband NCO.
 * @param rxWbNbCompPFir Configuration for the wideband narrowband compensation
 * PFIR.
 * @param resamp Configuration for the resampler.
 * @param gsOutMuxSel Selection for the GS output multiplexer.
 * @param rxOutSel Selection for the RX output.
 * @param rxRoundMode Mode for rounding the RX output.
 * @param dpArmSel Selection for the data path or ARM output.
 ******************************************************************************/
typedef struct adi_adrv9001_RxNbDemConfig
{
    adi_adrv9001_RxDpInFifoConfig_t	dpInFifo;
    adi_adrv9001_RxNbNcoConfig_t    rxNbNco;
    adi_adrv9001_RxWbNbCompPFir_t   rxWbNbCompPFir;
    adi_adrv9001_RxResampConfig_t   resamp;
    adi_adrv9001_RxGsOutMuxSel_e    gsOutMuxSel; /*!< fic algorithm enable */
    adi_adrv9001_RxOutSel_e         rxOutSel;
    adi_adrv9001_RxRoundMode_e      rxRoundMode;
    adi_adrv9001_RxDpArmSel_e       dpArmSel;
} adi_adrv9001_RxNbDemConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxDpProfile_t` structure is designed to encapsulate
 * the configuration settings for the RX data path profile in the
 * ADRV9001 device. It includes various decimation top settings for
 * narrowband, wideband, and common RX paths, as well as configurations
 * for the Sinc halfband and narrowband demodulation blocks. This
 * structure is crucial for defining the digital signal processing path
 * and ensuring the correct operation of the RX channel in different
 * signal conditions.
 *
 * @param rxNbDecTop RX narrowband decimation top.
 * @param rxWbDecTop RX wideband decimation top.
 * @param rxDecTop RX Common decimation top.
 * @param rxSincHBTop RX Sinc HB top.
 * @param rxNbDem RX NB Dem block.
 ******************************************************************************/
typedef struct adi_adrv9001_RxDpProfile
{
    adi_adrv9001_RxNbDecTop_t		rxNbDecTop;		/*!< RX narrowband decimation top */
    adi_adrv9001_RxWbDecTop_t		rxWbDecTop;		/*!< RX wideband decimation top */
    adi_adrv9001_RxDecTop_t			rxDecTop;		/*!< RX Common decimation top */
    adi_adrv9001_RxSincHbTop_t		rxSincHBTop;	/*!< RX Sinc HB top */
    adi_adrv9001_RxNbDemConfig_t	rxNbDem;		/*!< RX NB Dem block */
} adi_adrv9001_RxDpProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiType_e` is an enumeration that defines the types
 * of Serial Synchronous Interface (SSI) available in the ADRV9001
 * device. It includes options to disable the SSI, or to configure it as
 * either CMOS or LVDS, which are different electrical standards for data
 * transmission. This enumeration is used to specify the desired SSI
 * configuration for the device.
 *
 * @param ADI_ADRV9001_SSI_TYPE_DISABLE Disable SSI Type.
 * @param ADI_ADRV9001_SSI_TYPE_CMOS CMOS SSI Type.
 * @param ADI_ADRV9001_SSI_TYPE_LVDS LVDS SSI Type.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiType
{
    ADI_ADRV9001_SSI_TYPE_DISABLE = 0,  /*!< Disable SSI Type */
    ADI_ADRV9001_SSI_TYPE_CMOS = 1,     /*!< CMOS SSI Type */
    ADI_ADRV9001_SSI_TYPE_LVDS = 2      /*!< LVDS SSI Type */
} adi_adrv9001_SsiType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiDataFormat_e` is an enumeration that defines
 * various data formats for the SSI (Serial Synchronous Interface) in the
 * ADRV9001 device. It includes different bit-width configurations for
 * symbol and I/Q data, supporting both CMOS and LVDS signaling
 * standards. This enumeration allows the selection of specific data
 * formats to match the requirements of different communication
 * scenarios, including options for gain change and gain index
 * integration.
 *
 * @param ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA Represents 2-bit symbol data
 * format using CMOS.
 * @param ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA Represents 8-bit symbol data
 * format using CMOS.
 * @param ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA Represents 16-bit symbol
 * data format using CMOS.
 * @param ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA Represents 12-bit I/Q data
 * format using LVDS.
 * @param ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA Represents 16-bit I/Q data
 * format using CMOS or LVDS.
 * @param ADI_ADRV9001_SSI_FORMAT_15_BIT_I_Q_DATA_1_BIT_GAIN_CHANGE Represents
 * 15-bit I/Q
 * data with
 * 1-bit Rx
 * Gain Change
 * using CMOS
 * or LVDS.
 * @param ADI_ADRV9001_SSI_FORMAT_22_BIT_I_Q_DATA_1_BIT_GAIN_CHANGE_8_BIT_GAIN…R
 * e
 * p
 * r
 * e
 * s
 * e
 * n
 * t
 * s
 * 2
 * 2
 * -
 * b
 * i
 * t
 * I
 * /
 * Q
 * d
 * a
 * t
 * a
 * w
 * i
 * t
 * h
 * 1
 * -
 * b
 * i
 * t
 * R
 * x
 * G
 * a
 * i
 * n
 * C
 * h
 * a
 * n
 * g
 * e
 * a
 * n
 * d
 * 8
 * -
 * b
 * i
 * t
 * R
 * x
 * G
 * a
 * i
 * n
 * I
 * n
 * d
 * e
 * x
 * u
 * s
 * i
 * n
 * g
 * C
 * M
 * O
 * S
 * o
 * r
 * L
 * V
 * D
 * S
 * .
 ******************************************************************************/
typedef enum adi_adrv9001_SsiDataFormat
{
    ADI_ADRV9001_SSI_FORMAT_2_BIT_SYMBOL_DATA = 0,  /*!< 2 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_8_BIT_SYMBOL_DATA = 1,  /*!< 8 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_16_BIT_SYMBOL_DATA = 2, /*!< 16 bit symbol data (CMOS) */
    ADI_ADRV9001_SSI_FORMAT_12_BIT_I_Q_DATA = 3,    /*!< 12 bit I/Q data (LVDS) */
    ADI_ADRV9001_SSI_FORMAT_16_BIT_I_Q_DATA = 4,    /*!< 16 bit I/Q data (CMOS/LVDS) */

    /*!< 15 bit I/Q data, 1 bit Rx Gain Change (CMOS/LVDS) */
    ADI_ADRV9001_SSI_FORMAT_15_BIT_I_Q_DATA_1_BIT_GAIN_CHANGE = 5,

    /*!< 22 bit I/Q data, 1 bit Rx Gain Change, 8 bit Rx Gain Index (CMOS/LVDS) */
    ADI_ADRV9001_SSI_FORMAT_22_BIT_I_Q_DATA_1_BIT_GAIN_CHANGE_8_BIT_GAIN_INDEX = 6
} adi_adrv9001_SsiDataFormat_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiNumLane_e` is an enumeration that defines the
 * number of lanes used in the SSI (Serial Synchronous Interface)
 * configuration for the ADRV9001 device. It provides options for
 * configuring the interface with 1, 2, or 4 lanes, which can be used in
 * different signaling modes such as CMOS or LVDS. This enumeration is
 * crucial for setting up the data path width and ensuring compatibility
 * with the connected hardware.
 *
 * @param ADI_ADRV9001_SSI_1_LANE Represents a configuration with 1 lane for
 * CMOS/LVDS.
 * @param ADI_ADRV9001_SSI_2_LANE Represents a configuration with 2 lanes for
 * LVDS.
 * @param ADI_ADRV9001_SSI_4_LANE Represents a configuration with 4 lanes for
 * CMOS.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiNumLane
{
    ADI_ADRV9001_SSI_1_LANE = 0, /*!< 1 lane (CMOS/LVDS) */
    ADI_ADRV9001_SSI_2_LANE = 1, /*!< 2 lane (LVDS) */
    ADI_ADRV9001_SSI_4_LANE = 2  /*!< 4 lane (CMOS) */
} adi_adrv9001_SsiNumLane_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiStrobeType_e` is an enumeration that defines the
 * types of SSI (Serial Synchronous Interface) strobes available in the
 * ADRV9001 system. It provides two options: a short strobe and a long
 * strobe, which are used to configure the timing characteristics of the
 * SSI interface. This enumeration is part of the broader configuration
 * settings for the ADRV9001, a high-performance RF transceiver, and is
 * used to tailor the interface to specific application requirements.
 *
 * @param ADI_ADRV9001_SSI_SHORT_STROBE Represents a short SSI strobe with a
 * value of 0.
 * @param ADI_ADRV9001_SSI_LONG_STROBE Represents a long SSI strobe with a value
 * of 1.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiStrobeType
{
    ADI_ADRV9001_SSI_SHORT_STROBE = 0, /*!< Short SSI Strobe */
    ADI_ADRV9001_SSI_LONG_STROBE  = 1  /*!< Long SSI Strobe */
} adi_adrv9001_SsiStrobeType_e;


/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiTxRefClockPin_e` is an enumeration that defines
 * the configuration options for the Tx reference clock pin in the
 * ADRV9001 device. It specifies whether the Tx reference clock pin is
 * disabled or enabled in various modes, such as LVDS or CMOS, and which
 * specific pins are active in each mode. This configuration is crucial
 * for setting up the correct clocking scheme for the device's serial
 * data interface.
 *
 * @param ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED Tx reference clock out pin
 * is disabled.
 * @param ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED Tx reference clock
 * out pin
 * TX_DCLK_OUT+/- is
 * enabled in LVDS
 * mode or
 * TX_DCLK_OUT+ is
 * enabled in CMOS
 * mode.
 * @param ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_CMOS_DCLK_OUT_N_ENABLED Tx reference
 * clock out
 * pin
 * TX_DCLK_OUT-
 * is enabled
 * in CMOS
 * mode.
 * @param ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_CMOS_STROBE_IN_N_ENABLED Tx
 * reference
 * clock out
 * pin TX_STRO
 * BE_IN- is
 * enabled in
 * CMOS mode.
 ******************************************************************************/
typedef enum adi_adrv9001_SsiTxRefClockPin
{
    ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DISABLED                   = 0u,   /*!< Tx reference clock out pin disabled */
    ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_DCLK_OUT_ENABLED           = 1u,   /*!< Tx reference clock out pin TX_DCLK_OUT+/- enabled in LVDS mode or
                                                                                                         TX_DCLK_OUT+   enabled in CMOS mode */
    ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_CMOS_DCLK_OUT_N_ENABLED    = 2u,   /*!< Tx reference clock out pin TX_DCLK_OUT-   enabled in CMOS mode */
    ADI_ADRV9001_SSI_TX_REF_CLOCK_PIN_CMOS_STROBE_IN_N_ENABLED   = 3u    /*!< Tx reference clock out pin TX_STROBE_IN-  enabled in CMOS mode */
} adi_adrv9001_SsiTxRefClockPin_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_GpioAnalogPinNibbleSel_e` is an enumeration that
 * defines the selection of GPIO analog pin nibbles for the ADRV9001
 * device. It provides options to select specific groups of four GPIO
 * pins, referred to as nibbles, which can be used for various analog
 * functions. This enumeration is used to configure which set of GPIO
 * pins are active for a given operation, allowing for flexible pin
 * management in the device's analog interface.
 *
 * @param ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_UNASSIGNED Represents an
 * unassigned GPIO analog
 * pin nibble.
 * @param ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_03_00 Selects GPIO analog pin
 * nibble 03 to 00.
 * @param ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_07_04 Selects GPIO analog pin
 * nibble 07 to 04.
 * @param ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_11_08 Selects GPIO analog pin
 * nibble 11 to 08.
 ******************************************************************************/
typedef enum adi_adrv9001_GpioAnalogPinNibbleSel
{
    ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_UNASSIGNED,
    ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_03_00,
    ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_07_04,
    ADI_ADRV9001_GPIO_ANALOG_PIN_NIBBLE_11_08,
} adi_adrv9001_GpioAnalogPinNibbleSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ExternalLnaPinSel_e` is an enumeration that defines
 * the selection of external LNA (Low Noise Amplifier) pins for the RX
 * channels in the ADRV9001 device. It provides options to configure
 * which pins are used for RX1 and RX2 channels, allowing for flexible
 * hardware configurations depending on the specific application
 * requirements.
 *
 * @param ADI_ADRV9001_EXTERNAL_LNA_PIN_RX1_LOWER_RX2_UPPER Selects the lower
 * pin for RX1 and the
 * upper pin for RX2.
 * @param ADI_ADRV9001_EXTERNAL_LNA_PIN_RX1_UPPER_RX2_LOWER Selects the upper
 * pin for RX1 and the
 * lower pin for RX2.
 ******************************************************************************/
typedef enum adi_adrv9001_ExternalLnaPinSel
{
	ADI_ADRV9001_EXTERNAL_LNA_PIN_RX1_LOWER_RX2_UPPER,
	ADI_ADRV9001_EXTERNAL_LNA_PIN_RX1_UPPER_RX2_LOWER,
} adi_adrv9001_ExternalLnaPinSel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxGainTableType_e` is an enumeration that defines
 * the types of gain tables available for the ADRV9001 receiver. It
 * includes options for using digital gain to correct for coarse analog
 * gain steps or to compensate for changes in analog gain, both aimed at
 * maintaining a constant gain level. This enumeration is crucial for
 * configuring the gain behavior of the receiver during initialization.
 *
 * @param ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE Gain table to use digital gain
 * to adjust for coarse analog gain
 * steps and maintain a constant
 * gain.
 * @param ADI_ADRV9001_RX_GAIN_COMPENSATION_TABLE Gain table to adjust digital
 * gain when analog gain changes
 * to maintain a constant gain.
 ******************************************************************************/
typedef enum adi_adrv9001_RxGainTableType
{
    ADI_ADRV9001_RX_GAIN_CORRECTION_TABLE   = 0,   /*!< Gain table to use digital gain to adjust for coarse analog gain steps and maintain a constant gain */
    ADI_ADRV9001_RX_GAIN_COMPENSATION_TABLE = 1    /*!< Gain table to adjust digital gain when analog gain changes to maintain a constant gain*/
} adi_adrv9001_RxGainTableType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxLnaConfig_t` structure is used to configure the
 * Low Noise Amplifier (LNA) settings for the ADRV9001 receiver. It
 * includes parameters to determine the presence of an external LNA,
 * configure GPIO pins for LNA control, select the appropriate LNA pins,
 * and set various gain and delay parameters. This structure is essential
 * for optimizing the LNA performance in different operational scenarios,
 * ensuring the receiver can handle varying signal conditions
 * effectively.
 *
 * @param externalLnaPresent Indicates if an external LNA is present (true) or
 * not (false).
 * @param gpioSourceSel Specifies the GPIO source pin configuration for
 * controlling the external LNA.
 * @param externalLnaPinSel Determines the pin selection for the external LNA on
 * the RX channel.
 * @param settlingDelay Defines the settling delay for the external LNA in units
 * of AGC clocks multiplied by 4.
 * @param numberLnaGainSteps Specifies the number of gain steps required for the
 * external LNA.
 * @param lnaGainSteps_mdB An array containing the gain step sizes in milli-
 * decibels corresponding to GPIO pin values.
 * @param lnaDigitalGainDelay Specifies the delay for digital gain adjustment in
 * the LNA.
 * @param minGainIndex Indicates the minimum gain index to be configured.
 ******************************************************************************/
typedef struct adi_adrv9001_RxLnaConfig
{
    bool externalLnaPresent;	                            /*!< true: External LNA is added; false: No external LNA  */
    adi_adrv9001_GpioAnalogPinNibbleSel_e gpioSourceSel;	/*!< GPIO Source Select Pin Configuration to output 
                                                                 Rx1/2 external LNA controlto Analog GPIO pins */
	adi_adrv9001_ExternalLnaPinSel_e externalLnaPinSel;		/*!< External LNA pin selection for RX channel. If ADI_ADRV9001_EXTERNAL_LNA_PIN_RX1_LOWER_RX2_UPPER, then
																 lower 2 pins in analog GPIO pin nibble are selected for Rx1 channel */
    uint8_t settlingDelay;                                  /*!< External LNA Settling Delay (Units: AGC clocks * 4) */
    uint8_t numberLnaGainSteps;                             /*!< Number of Gain Step(s) required in external LNA */
    uint16_t lnaGainSteps_mdB[4];							/*!< Array of Gain Step Size(s) in mdB corresponding to the value of the GPIO pin(s) */
    uint16_t lnaDigitalGainDelay;
	uint8_t minGainIndex;                                   /*!< The desired minimum gain index to be configured */
} adi_adrv9001_RxLnaConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_SsiConfig_t` structure is used to configure the
 * Serial Synchronous Interface (SSI) settings for the ADRV9001 device.
 * It includes parameters for selecting the SSI type, data format, number
 * of lanes, and strobe type. Additionally, it provides options for bit
 * inversion and clock settings in both LVDS and CMOS modes, as well as
 * enabling Double Data Rate (DDR) and mask strobe functionalities. This
 * structure is essential for setting up the data interface between the
 * ADRV9001 and other components in a communication system.
 *
 * @param ssiType Specifies the type of SSI (Serial Synchronous Interface).
 * @param ssiDataFormatSel Defines the data format for the SSI.
 * @param numLaneSel Indicates the number of lanes used in the SSI.
 * @param strobeType Specifies the type of strobe used in the SSI.
 * @param lsbFirst Determines if the least significant bit is transmitted first.
 * @param qFirst Indicates if Q data is transmitted before I data.
 * @param txRefClockPin Selects the GPIO pin for the SSI Tx reference clock.
 * @param lvdsIBitInversion Enables inversion of the I bit in LVDS mode.
 * @param lvdsQBitInversion Enables inversion of the Q bit in LVDS mode.
 * @param lvdsStrobeBitInversion Enables inversion of the strobe bit in LVDS
 * mode.
 * @param lvdsUseLsbIn12bitMode Specifies the use of LSB in 12-bit mode for
 * LVDS.
 * @param lvdsRxClkInversionEn Enables inversion of the Rx clock in LVDS mode.
 * @param cmosDdrPosClkEn Enables the positive clock for CMOS DDR.
 * @param cmosClkInversionEn Enables clock inversion for CMOS.
 * @param ddrEn Enables Double Data Rate (DDR) mode.
 * @param rxMaskStrobeEn Enables the mask strobe for SSI Rx.
 ******************************************************************************/
typedef struct adi_adrv9001_SsiConfig
{
    adi_adrv9001_SsiType_e       ssiType;					/*!< SSI type */
    adi_adrv9001_SsiDataFormat_e ssiDataFormatSel;			/*!< SSI data format */
    adi_adrv9001_SsiNumLane_e    numLaneSel;				/*!< SSI number of lanes */
    adi_adrv9001_SsiStrobeType_e strobeType;				/*!< SSI strobe type */
    uint8_t						 lsbFirst;					/*!< SSI LSB first */
    uint8_t						 qFirst;					/*!< SSI Q data first */
    adi_adrv9001_SsiTxRefClockPin_e	txRefClockPin;			/*!< SSI Tx reference clock GPIO select */
    bool                         lvdsIBitInversion;         /*!< LVDS SSI I bit inversion. Rx: Inverts I and Q. Tx: inverts I only */
    bool                         lvdsQBitInversion;         /*!< LVDS SSI Q bit inversion. Rx: Has no effect.   Tx: inverts Q only */
    bool                         lvdsStrobeBitInversion;    /*!< LVDS SSI Strobe bit inversion */
    uint8_t						 lvdsUseLsbIn12bitMode;		/*!< LVDS use LSB in 12 bit mode */
    bool						 lvdsRxClkInversionEn;      /*!< LVDS Rx clock inversion enable */
    bool                         cmosDdrPosClkEn;           /*!< CMOS DDR positive clock enable */
    bool                         cmosClkInversionEn;        /*!< CMOS clock inversion enable */
    bool                         ddrEn;                     /*!< Double Data Rate (DDR) enable */
    bool                         rxMaskStrobeEn;            /*!< SSI Rx mask strobe enable */
} adi_adrv9001_SsiConfig_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxProfile_t` structure defines the configuration
 * settings for an ADRV9001 receiver profile, including parameters for
 * signal bandwidth, output rate, and interface sample rate. It also
 * includes settings for offset frequency, NCO enablement, and output
 * signaling type. The structure specifies filter orders, ADC corner
 * frequencies, and TIA bandwidths, as well as power levels for both high
 * and low power ADCs. Additionally, it contains configuration details
 * for the channel type, ADC type, and calibration mode, along with gain
 * table type and digital data path configuration. The structure also
 * includes configurations for external LNA and serial data interface.
 *
 * @param primarySigBandwidth_Hz Rx primary signal bandwidth in Hz.
 * @param rxOutputRate_Hz Rx output data rate in Hz.
 * @param rxInterfaceSampleRate_Hz Rx sample rate at serial interface.
 * @param rxOffsetLo_kHz Offset in kHz, with 0 indicating on LO.
 * @param rxNcoEnable Enable NCO in Rx datapath, required if rxOffsetLo_kHz > 0.
 * @param outputSignaling Output to BBIC signal type.
 * @param filterOrder 1st or 2nd order ABBF filter.
 * @param filterOrderLp 1st or 2nd order ABBF filter for Low Power ADC.
 * @param hpAdcCorner High Power ADC corner frequency.
 * @param lpAdcCorner Low Power ADC 3dB corner frequency in Hz.
 * @param adcClk_kHz ADC clock frequency options: 2.2G/1.47G/1.1G.
 * @param rxCorner3dB_kHz TIA bandwidth in kHz.
 * @param rxCorner3dBLp_kHz TIA bandwidth for Low Power ADC.
 * @param tiaPower Rx TIA power level before the high-performance ADC.
 * @param tiaPowerLp Rx TIA power level before the low power ADC.
 * @param channelType Channel type described by this profile (Rx/ORx/Loopback).
 * @param adcType ADC type: low/high power ADC.
 * @param lpAdcCalMode Select periodic or continuous Low Power ADC calibration.
 * @param gainTableType Type of gain table loaded during ADRV9001
 * initialization.
 * @param rxDpProfile RX digital data path configuration.
 * @param lnaConfig Rx external LNA configuration.
 * @param rxSsiConfig RX Serial data interface configuration.
 ******************************************************************************/
typedef struct adi_adrv9001_RxProfile
{
    uint32_t		primarySigBandwidth_Hz;			  /*!< Rx primary signal BW in Hz */
    uint32_t		rxOutputRate_Hz;				  /*!< Rx output data rate in Hz */
    uint32_t		rxInterfaceSampleRate_Hz;		  /*!< Rx sample rate at serial interface */
    int32_t			rxOffsetLo_kHz;                   /*!< Offset in kHz. 0: On LO */
    bool			rxNcoEnable;                      /*!< Enable NCO in Rx datapath. NCO must be enabled if rxOffsetLo_kHz > 0  */
    adi_adrv9001_RxSignalType_e outputSignaling;      /*!< Output to BBIC signal type */
    uint8_t			filterOrder;					  /*!< 1st or 2nd order ABBF filter */
    uint8_t			filterOrderLp;					  /*!< 1st or 2nd order ABBF filter Low Power ADC*/
    uint32_t		hpAdcCorner;					  /*!< High Power ADC corner freq*/
    uint32_t        lpAdcCorner;                      /*!< Low Power ADC 3dB corner frequency in Hz */
    uint32_t		adcClk_kHz;						  /*!< ADC clock 2.2G/1.47G/1.1G */
    uint32_t		rxCorner3dB_kHz;				  /*!< TIA bandwidth in kHz */
    uint32_t		rxCorner3dBLp_kHz;				  /*!< TIA bandwidth for Low Power ADC */
    adi_adrv9001_ComponentPowerLevel_e tiaPower;      /*!< Rx TIA power level before the high-performance ADC */
    adi_adrv9001_ComponentPowerLevel_e tiaPowerLp;    /*!< Rx TIA power level before the low power ADC*/
    uint32_t	    channelType;		              /*!< Channel type described by this profile (Rx/ORx/Loopback) */
    adi_adrv9001_AdcType_e		adcType;			  /*!< ADC type: low/high power ADC */
    adi_adrv9001_Adc_LowPower_CalMode_e  lpAdcCalMode; /*!< Select periodic or continuous Low Power ADC calibration */
    adi_adrv9001_RxGainTableType_e gainTableType;     /*!< type of gain table loaded during ADRV9001 initialization */
    adi_adrv9001_RxDpProfile_t	rxDpProfile;		  /*!< RX digital data path config */
	adi_adrv9001_RxLnaConfig_t  lnaConfig;            /*!< Rx external LNA configuration */
    adi_adrv9001_SsiConfig_t	rxSsiConfig;		  /*!< RX Serial data interface config */
} adi_adrv9001_RxProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxChannelCfg_t` structure is designed to encapsulate
 * the configuration settings for a single Rx channel in the ADRV9001
 * device. It primarily holds a `profile` member, which is of type
 * `adi_adrv9001_RxProfile_t`. This member contains detailed settings
 * related to the Rx datapath, including the 3dB corner frequencies and
 * digital filter enables, which are crucial for defining the signal
 * processing characteristics of the Rx channel. This structure is part
 * of a larger configuration framework used to initialize and manage the
 * Rx channels in the ADRV9001 system.
 *
 * @param profile Rx datapath profile, 3dB corner frequencies, and digital
 * filter enables.
 ******************************************************************************/
typedef struct adi_adrv9001_RxChannelCfg
{
    adi_adrv9001_RxProfile_t profile; /*!< Rx datapath profile, 3dB corner frequencies, and digital filter enables */
} adi_adrv9001_RxChannelCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_RxSettings_t` structure is designed to encapsulate
 * the configuration settings for the ADRV9001 receiver channels. It
 * includes a channel mask to specify which channels should be
 * initialized and an array of `adi_adrv9001_RxChannelCfg_t` structures,
 * each containing detailed configuration settings for individual Rx
 * channels. This structure is essential for managing and initializing
 * the receiver channels in the ADRV9001 system, allowing for flexible
 * and precise control over the channel configurations.
 *
 * @param rxInitChannelMask Rx channel mask of which channels to initialize.
 * @param rxChannelCfg Rx settings per Rx channel, stored in an array with a
 * size defined by ADI_ADRV9001_MAX_RXCHANNELS.
 ******************************************************************************/
typedef struct adi_adrv9001_RxSettings
{
    uint32_t rxInitChannelMask;                                            /*!< Rx channel mask of which channels to initialize */
    adi_adrv9001_RxChannelCfg_t rxChannelCfg[ADI_ADRV9001_MAX_RXCHANNELS]; /*!< Rx settings per Rx channel */
} adi_adrv9001_RxSettings_t;

#endif /* _ADI_ADRV9001_RXSETTINGS_TYPES_H_ */
