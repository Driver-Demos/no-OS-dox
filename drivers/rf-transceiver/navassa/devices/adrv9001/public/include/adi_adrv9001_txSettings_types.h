/**
 * \file
 * \brief Type definitions for ADRV9001 Tx settings
 * \copyright Analog Devices Inc. 2019. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_TXSETTINGS_TYPES_H_
#define _ADI_ADRV9001_TXSETTINGS_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_adrv9001_rxSettings_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxSignalType_e` is an enumeration that defines the
 * types of transmission signals that can be used with the ADRV9001
 * device. It includes options for standard IQ signals, IQ signals with
 * FM and FSK modulation, and direct FM and FSK signals. This enumeration
 * is used to configure the type of signal that the transmitter will
 * handle, allowing for flexibility in signal processing and
 * transmission.
 *
 * @param ADI_ADRV9001_TX_IQ Represents a standard IQ signal type for
 * transmission.
 * @param ADI_ADRV9001_TX_IQ_FM_FSK Represents a signal type that combines IQ
 * with frequency modulation (FM) and frequency
 * shift keying (FSK).
 * @param ADI_ADRV9001_TX_DIRECT_FM_FSK Represents a direct frequency modulation
 * (FM) and frequency shift keying (FSK)
 * signal type for transmission.
 ******************************************************************************/
typedef enum adi_adrv9001_TxSignalType
{
    ADI_ADRV9001_TX_IQ            = 0,
    ADI_ADRV9001_TX_IQ_FM_FSK     = 1,
    ADI_ADRV9001_TX_DIRECT_FM_FSK = 2
} adi_adrv9001_TxSignalType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxDpPreProcMode_e` is an enumeration that defines
 * the modes of operation for the ADRV9001 transmitter data path pre-
 * processor. It specifies how the pre-processor should handle I-only or
 * I/Q data in relation to the programmable finite impulse response
 * (PFIR) filters, offering options to bypass, insert, or cascade these
 * filters. This enumeration is crucial for configuring the signal
 * processing path in the transmitter, allowing for flexible handling of
 * different data types and processing requirements.
 *
 * @param ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_BYPASS_PFIRS I-only data,
 * bypass both PFIRs.
 * @param ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS I/Q data, insert both
 * PFIRs.
 * @param ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_WITH_PFIR I-only data, insert I
 * only PFIR.
 * @param ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_CASCADE_PFIRS I-only data,
 * cascade I and Q
 * PFIRS (256-tap
 * total).
 * @param ADI_ADRV9001_TX_DP_PREPROC_ENABLE_PFIRS Mask bit to enable PFIRs.
 ******************************************************************************/
typedef enum adi_adrv9001_TxDpPreProcMode
{
    ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_BYPASS_PFIRS  = 0,  /*!< I-only data, bypass both PFIRs */
    ADI_ADRV9001_TX_DP_PREPROC_IQ_DATA_WITH_PFIRS        = 1,  /*!< I/Q data, insert both PFIRs */
    ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_WITH_PFIR     = 2,  /*!< I-only data, insert I only PFIR */
    ADI_ADRV9001_TX_DP_PREPROC_I_ONLY_DATA_CASCADE_PFIRS = 3,  /*!< I-only data, cascade I and Q PFIRS (256-tap total) */
    ADI_ADRV9001_TX_DP_PREPROC_ENABLE_PFIRS              = 0x4 /*!< Mask bit to enable PFIRs */
} adi_adrv9001_TxDpPreProcMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxPreProc_t` structure is designed to manage the TX
 * pre-processor block settings for the ADRV9001 device. It includes
 * symbol mapping configurations with four symbols in S3.14 18-bit format
 * and a division factor for symbol mapping. The structure also defines
 * the mode of operation for the TX data path pre-processor, which is
 * static and determined by the baseband IC (BBIC) based on the use case.
 * Additionally, it includes dynamic configuration options for the
 * Preproc PFIR I and Q banks, allowing for adjustments during dynamic
 * profile switching.
 *
 * @param txPreProcSymbol0 TX Preprocessor symbol mapping symbol 0, in S3.14
 * 18-bit format.
 * @param txPreProcSymbol1 TX Preprocessor symbol mapping symbol 1, in S3.14
 * 18-bit format.
 * @param txPreProcSymbol2 TX Preprocessor symbol mapping symbol 2, in S3.14
 * 18-bit format.
 * @param txPreProcSymbol3 TX Preprocessor symbol mapping symbol 3, in S3.14
 * 18-bit format.
 * @param txPreProcSymMapDivFactor Division factor that controls the symbol
 * mapping, 5 bit (supports 1, 2, 3, 4, 5, 19,
 * 16, 20).
 * @param txPreProcMode TX data path pre-processor mode, supporting four static
 * modes of operation.
 * @param txPreProcWbNbPfirIBankSel Configuration of Preproc PFIR I, dynamic and
 * configurable during dynamic profile
 * switching.
 * @param txPreProcWbNbPfirQBankSel Configuration of Preproc PFIR Q, dynamic and
 * configurable during dynamic profile
 * switching.
 ******************************************************************************/
typedef struct adi_adrv9001_TxPreProc
{
    /* tx pre-processor symbol map */
    uint32_t    txPreProcSymbol0;         /*!< TX Preprocessor symbol mapping symbol 0, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol1;         /*!< TX Preprocessor symbol mapping symbol 1, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol2;         /*!< TX Preprocessor symbol mapping symbol 2, in S3.14 18-bit format */
    uint32_t    txPreProcSymbol3;         /*!< TX Preprocessor symbol mapping symbol 3, in S3.14 18-bit format */
    uint8_t     txPreProcSymMapDivFactor; /*!< Division factor that controls the symbol mapping. 5 bit (supports 1, 2, 3, 4, 5, 19, 16, 20)*/

    /* Tx pre-processor config parameters that support four modes of operation */
    adi_adrv9001_TxDpPreProcMode_e txPreProcMode; /*!< TX_DP_PREPROC_MODE0 - bypass mode,
                                                       TX_DP_PREPROC_MODE1 - I and Q PFIR mode,
                                                       TX_DP_PREPROC_MODE2 - I PFIR only mode,
                                                       TX_DP_PREPROC_MODE3 - Cascaded I and Q PFIR mode
                                                       STATIC, not configurable on the fly, BBIC to determine based on usecase */

    adi_adrv9001_PfirBank_e txPreProcWbNbPfirIBankSel; /*!< Config of Preproc PFIR I: Block #3, TPFIR_I, Dynamic, configurable during dynamic profile switching */
    adi_adrv9001_PfirBank_e txPreProcWbNbPfirQBankSel; /*!< Config of Preproc PFIR Q: Block #5, TPFIR_Q, Dynamic, configurable during dynamic profile switching */
} adi_adrv9001_TxPreProc_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxNbIntTop_t` structure is designed to manage the
 * configuration of narrowband interpolation and TSCIC blocks within the
 * ADRV9001 transmitter. It contains several enable flags for different
 * interpolation blocks, allowing for selective activation of these
 * blocks. Additionally, it includes a division factor for the TSCIC
 * block, providing further control over the signal processing path. This
 * structure is part of the dynamic settings for the transmitter,
 * enabling flexible configuration based on specific use cases.
 *
 * @param txInterpBy2Blk20En Tx Interpolator2 Narrowband Top block 20 enable
 * flag.
 * @param txInterpBy2Blk18En Tx Interpolator3 Narrowband Top block 18 enable
 * flag.
 * @param txInterpBy2Blk16En Tx Interpolator2 Narrowband Top block 16 enable
 * flag.
 * @param txInterpBy2Blk14En Tx Interpolator2 Narrowband Top block 14 enable
 * flag.
 * @param txInterpBy2Blk12En Tx Interpolator2 Narrowband Top block 12 enable
 * flag.
 * @param txInterpBy3Blk10En Tx Interpolator3 Narrowband Top block 10 enable
 * flag.
 * @param txInterpBy2Blk8En Tx Interpolator2 Narrowband Top block 8 enable flag.
 * @param txScicBlk32En Tx TSCIC Narrowband Top block 32 enable flag.
 * @param txScicBlk32DivFactor Division factor for Tx TSCIC Narrowband Top block
 * 32.
 ******************************************************************************/
typedef struct adi_adrv9001_TxNbIntTop
{
    uint8_t txInterpBy2Blk20En;   /*!< Tx Interpolator2 Narrowband Top block 20. Bit field tx_dp_int2_20_en */
    uint8_t txInterpBy2Blk18En;   /*!< Tx Interpolator3 Narrowband Top block 18. Bit field tx_dp_int2_18_en */
    uint8_t txInterpBy2Blk16En;   /*!< Tx Interpolator2 Narrowband Top block 16. Bit field tx_dp_int2_16_en */
    uint8_t txInterpBy2Blk14En;   /*!< Tx Interpolator2 Narrowband Top block 14. Bit field tx_dp_int2_14_en */
    uint8_t txInterpBy2Blk12En;   /*!< Tx Interpolator2 Narrowband Top block 12. Bit field tx_dp_int2_12_en */
    uint8_t txInterpBy3Blk10En;   /*!< Tx Interpolator3 Narrowband Top block 10. Bit field tx_dp_int3_10_en */
    uint8_t txInterpBy2Blk8En;    /*!< Tx Interpolator2 Narrowband Top block 8. Bit field tx_dp_int2_8_en */

    uint8_t txScicBlk32En;        /*!< Tx TSCIC Narrowband Top block 32. Bit field tx_dp_tscic_32_en */
    uint8_t txScicBlk32DivFactor; /*!< Tx TSCIC Narrowband Top block 32. Bit field tx_dp_tscic_32_div_factor */
} adi_adrv9001_TxNbIntTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxWbIntTop_t` structure is designed to manage the
 * configuration of the wideband interpolation top blocks in the ADRV9001
 * transmitter. It contains several enable flags for different
 * interpolation blocks, allowing for the control of specific wideband
 * top block interpolators and low-pass filters. This structure is part
 * of the dynamic settings for the transmitter's digital data path,
 * enabling or disabling specific processing blocks to optimize signal
 * processing for wideband applications.
 *
 * @param txInterpBy2Blk30En Tx Interpolator2 Wideband Top block 30 enable flag.
 * @param txInterpBy2Blk28En Tx Interpolator2 Wideband Top block 28 enable flag.
 * @param txInterpBy2Blk26En Tx Interpolator2 Wideband Top block 26 enable flag.
 * @param txInterpBy2Blk24En Tx Interpolator2 Wideband Top block 24 enable flag.
 * @param txInterpBy2Blk22En Tx Interpolator2 Wideband Top block 22 enable flag.
 * @param txWbLpfBlk22p1En Tx Wideband LPF Wideband Top block 22.1 enable flag.
 ******************************************************************************/
typedef struct adi_adrv9001_TxWbIntTop
{
    uint8_t txInterpBy2Blk30En; /*!< Tx Interpolator2 Wideband Top block 30. Bit field tx_dp_int2_30_en */
    uint8_t txInterpBy2Blk28En; /*!< Tx Interpolator2 Wideband Top block 28. Bit field tx_dp_int2_28_en */
    uint8_t txInterpBy2Blk26En; /*!< Tx Interpolator2 Wideband Top block 26. Bit field tx_dp_int2_26_en */
    uint8_t txInterpBy2Blk24En; /*!< Tx Interpolator2 Wideband Top block 24. Bit field tx_dp_int2_24_en*/
    uint8_t txInterpBy2Blk22En; /*!< Tx Interpolator2 Wideband Top block 22. Bit field tx_dp_int2_22_en*/
    uint8_t txWbLpfBlk22p1En;   /*!< Tx Wideband LPF Wideband Top block 22.1. Bit field tx_dp_int2_22_1_en */
} adi_adrv9001_TxWbIntTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxIntTop_t` structure is used to configure the
 * interpolation top blocks for the ADRV9001 transmitter. It contains
 * several fields that enable or disable specific interpolation and sinc
 * blocks, which are part of the digital signal processing chain in the
 * transmitter. Each field corresponds to a specific block and is
 * represented as a bit field, allowing for fine-grained control over the
 * interpolation process.
 *
 * @param interpBy3Blk44p1En Tx Interpolator3 Top block 44.1 enable flag.
 * @param sinc3Blk44En Tx Sinc3 Top block 44 enable flag.
 * @param sinc2Blk42En Tx Sinc2 Top block 42 enable flag.
 * @param interpBy3Blk40En Tx Interpolator3 Top block 40 enable flag.
 * @param interpBy2Blk38En Tx Interpolator2 Top block 38 enable flag.
 * @param interpBy2Blk36En Tx Interpolator2 Top block 36 enable flag.
 ******************************************************************************/
typedef struct adi_adrv9001_TxIntTop
{
    uint8_t interpBy3Blk44p1En; /*!< Tx Interpolator3 Top block 44.1. Bit field tx_dp_int3_44_1_en */
    uint8_t sinc3Blk44En;       /*!< Tx Sinc3 Top block 44. Bit field tx_dp_sinc3_44_en*/
    uint8_t sinc2Blk42En;       /*!< Tx Sinc2 Top block 42. Bit field tx_dp_int2_42_en*/
    uint8_t interpBy3Blk40En;   /*!< Tx Interpolator3 Top block 40. Bit field tx_dp_int3_40_en*/
    uint8_t interpBy2Blk38En;   /*!< Tx Interpolator2 Top block 38. Bit field tx_dp_int2_38_en */
    uint8_t interpBy2Blk36En;   /*!< Tx Interpolator2 Top block 36. Bit field tx_dp_int2_36_en*/
} adi_adrv9001_TxIntTop_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxIntTopFreqDevMap_t` structure is designed to
 * manage frequency deviation mapping in the ADRV9001 transmitter
 * settings. It includes fields for specifying frequency deviation, a
 * frequency fraction multiplier, and a frequency offset split into least
 * and most significant parts. Additionally, it provides enable/disable
 * controls for specific frequency deviation mapping and transmission
 * rounding blocks, allowing for fine-tuned control over the frequency
 * characteristics of the transmitted signal.
 *
 * @param rrc2Frac Frequency Deviation.
 * @param mpll Frequency fraction multiplier.
 * @param nchLsw Frequency offset, least significant word (Bits 0-31).
 * @param nchMsb Frequency offset, most significant 3 bits (Bits 32-34).
 * @param freqDevMapEn Enable/disable block #47 Freq_Dev Mapper.
 * @param txRoundEn Enable/disable tx round block #46.
 ******************************************************************************/
typedef struct adi_adrv9001_TxIntTopFreqDevMap
{
    /* Frequency dev mapper */
    uint32_t rrc2Frac;		/*!< Frequency Deviation */
    uint32_t mpll;			/*!< Frequency fraction multiplier */
    uint32_t nchLsw;		/*!< Frequency offset. Contains least significant word, i.e Bit[0:31] */
    uint8_t  nchMsb;		/*!< Frequency offset. Contains most significant 3 bits, i.e Bit[32:34] */
    uint8_t  freqDevMapEn;	/*!< Enable/disable block #47 Freq_Dev Mapper. */
    uint8_t  txRoundEn;		/*!< Enable/disable tx round block #46. */
} adi_adrv9001_TxIntTopFreqDevMap_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxDpIqDmDucMode_e` is an enumeration that defines
 * the modes of operation for the Tx Data Path IQDMDUC in the ADRV9001
 * device. It provides three distinct modes: mode0 for bypassing IQ in
 * and out, mode1 for IQ DM (Digital Modulation), and mode2 for DUC
 * (Digital Up Conversion). This enumeration is used to configure the
 * behavior of the Tx IQDM DUC block, allowing for flexible signal
 * processing configurations in the transmission path.
 *
 * @param ADI_ADRV9001_TX_DP_IQDMDUC_MODE0 Selects mode0 for the Tx IqDmDuc,
 * bypass IQ in IQ out.
 * @param ADI_ADRV9001_TX_DP_IQDMDUC_MODE1 Selects mode1 for the Tx IqDmDuc, IQ
 * DM.
 * @param ADI_ADRV9001_TX_DP_IQDMDUC_MODE2 Selects mode2 for the Tx IqDmDuc,
 * DUC.
 ******************************************************************************/
typedef enum adi_adrv9001_TxDpIqDmDucMode
{
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE0 = 0u, /*!< Selects mode0 for the Tx IqDmDuc, bypass IQ in IQ out  */
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE1 = 1u, /*!< Selects mode1 for the Tx IqDmDuc, IQ DM                */
    ADI_ADRV9001_TX_DP_IQDMDUC_MODE2 = 2u  /*!< Selects mode2 for the Tx IqDmDuc, DUC                  */
} adi_adrv9001_TxDpIqDmDucMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxIqdmDuc_t` structure is designed to configure the
 * Transmit IQ Data Modulation and Digital Up-Conversion (IQ DM DUC)
 * block in the ADRV9001 transceiver. It supports three operational
 * modes: bypass IQ, IQ DM, and DUC, allowing for flexible signal
 * processing configurations. The structure includes parameters specific
 * to the IQ DM mode, such as device settings, offsets, scalars, and
 * thresholds, as well as configuration for the Numerically Controlled
 * Oscillator (NCO) used in the IQ DM process. This structure is crucial
 * for setting up the digital signal processing path in the transmitter
 * section of the ADRV9001.
 *
 * @param iqdmDucMode Specifies the mode of operation for the Tx IQ DM DUC,
 * supporting three modes: bypass IQ, IQ DM, and DUC.
 * @param iqdmDev Parameter for TX_DP_IQDMDUC_MODE1 Iqdm mode, representing a
 * device-specific setting.
 * @param iqdmDevOffset Parameter for TX_DP_IQDMDUC_MODE1 Iqdm mode,
 * representing an offset value.
 * @param iqdmScalar Parameter for TX_DP_IQDMDUC_MODE1 Iqdm mode, representing a
 * scalar value.
 * @param iqdmThreshold Parameter for TX_DP_IQDMDUC_MODE1 Iqdm mode,
 * representing a threshold value.
 * @param iqdmNco Configuration parameters for the IQDM NCO (Numerically
 * Controlled Oscillator).
 ******************************************************************************/
typedef struct adi_adrv9001_TxIqdmDuc
{
    /* Tx IQ DM DUC config parameters that support three modes of operation */
    adi_adrv9001_TxDpIqDmDucMode_e iqdmDucMode; /*!< TX_DP_IQDMDUC_MODE0 - bypass IQ in IQ out, TX_DP_IQDMDUC_MODE1 - IQ DM, TX_DP_IQDMDUC_MODE2 - DUC */

    /* Parameters for TX_DP_IQDMDUC_MODE1 IqDm mode */
    uint32_t iqdmDev;       /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmDevOffset; /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmScalar;    /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */
    uint32_t iqdmThreshold; /*!< Parameters for TX_DP_IQDMDUC_MODE1 Iqdm mode */

    adi_adrv9001_NcoDpConfig_t iqdmNco; /*!< Parameters for IQDM NCO */
} adi_adrv9001_TxIqdmDuc_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxDpProfile_t` structure is designed to encapsulate
 * the configuration settings for the ADRV9001 transmitter data path. It
 * includes various blocks such as the pre-processor, wideband and
 * narrowband interpolators, and frequency deviation mapping, each with
 * specific static or dynamic settings. This structure is crucial for
 * defining the behavior and performance of the transmitter's digital
 * processing path, allowing for detailed control over signal processing
 * elements like interpolation and frequency deviation.
 *
 * @param txPreProc Tx pre processor block with static settings.
 * @param txWbIntTop Wideband Top block interpolater configurations with dynamic
 * settings.
 * @param txNbIntTop Narrowband Top block interpolater configurations with
 * dynamic settings.
 * @param txIntTop Top block interpolater configurations with dynamic settings.
 * @param txIntTopFreqDevMap Top block frequency deviation map configurations
 * with static settings.
 * @param txIqdmDuc IQDM DUC block configurations with static settings.
 ******************************************************************************/
typedef struct adi_adrv9001_TxDpProfile
{
    adi_adrv9001_TxPreProc_t           txPreProc;          /*!< Tx pre processor block (Static Settings) */
    adi_adrv9001_TxWbIntTop_t          txWbIntTop;         /*!< txwb_int_top (Wideband Top) block interpolater configs (Dynamic Settings)*/
    adi_adrv9001_TxNbIntTop_t          txNbIntTop;         /*!< txnb_int_top (Narrowband Top) block interpolater configs (Dynamic Settings) */
    adi_adrv9001_TxIntTop_t            txIntTop;           /*!< tx_int_top (TOP) block interpolater configs (Dynamic Settings) */
    adi_adrv9001_TxIntTopFreqDevMap_t  txIntTopFreqDevMap; /*!< tx_int_top (TOP) block frequency dev map configs (Static Settings) */
    adi_adrv9001_TxIqdmDuc_t           txIqdmDuc;          /*!< tx_iqdm_duc block configs (Static Settings) */
} adi_adrv9001_TxDpProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxProfile_t` structure defines the configuration
 * settings for the ADRV9001 transmitter profile. It includes parameters
 * for signal bandwidth, input data rate, and sample rate, as well as
 * settings for local oscillator offset, data delay, and baseband filter
 * characteristics. The structure also specifies the output signaling
 * type, pre-distortion settings, and loopback configurations.
 * Additionally, it contains fields for frequency deviation and digital
 * data path configuration, making it a comprehensive representation of
 * the transmitter's operational parameters.
 *
 * @param primarySigBandwidth_Hz Tx primary signal bandwidth in Hz.
 * @param txInputRate_Hz Tx input data rate in Hz.
 * @param txInterfaceSampleRate_Hz TX sample rate at serial interface.
 * @param txOffsetLo_kHz Frequency of LO relative to carrier in kHz.
 * @param validDataDelay Valid data delay relative to TX Enable by 184MHz clock
 * counter.
 * @param txBbf3dBCorner_kHz Tx baseband filter 3dB corner frequency in kHz.
 * @param outputSignaling Output to analog signal type.
 * @param txPdBiasCurrent Pre-distorter programmable bias current.
 * @param txPdGainEnable TX pre-distortion gain enable flag.
 * @param txPrePdRealPole_kHz TX pre-distortion pole frequency in kHz.
 * @param txPostPdRealPole_kHz Post-distorter filter corner frequency in kHz.
 * @param txBbfPower Tx baseband filter power mode.
 * @param txExtLoopBackType External loopback connection type.
 * @param txExtLoopBackForInitCal Flag indicating if external loopback should be
 * used for initial calibration.
 * @param txPeakLoopBackPower Target RMS signal power at input to receiver and
 * its peak-to-average ratio in dBm.
 * @param frequencyDeviation_Hz Frequency deviation value in Hz for both FM_IQ
 * and FM_DM.
 * @param txDpProfile TX digital data path configuration.
 * @param txSsiConfig TX serial data interface configuration.
 ******************************************************************************/
typedef struct adi_adrv9001_TxProfile
{
    uint32_t primarySigBandwidth_Hz;              /*!< Tx primary signal BW in Hz*/
    uint32_t txInputRate_Hz;                      /*!< Tx input data rate in Hz */
    uint32_t txInterfaceSampleRate_Hz;			  /*!< TX sample rate at serial interface */
    int32_t  txOffsetLo_kHz;                      /*!< Frequency of LO relative to carrier */
    uint32_t validDataDelay;					  /*!< Valid data delay relative to TX Enable by 184MHz clock counter */
    uint32_t txBbf3dBCorner_kHz;                  /*!< Tx BBF 3dB corner in kHz - butterFilterBw */
    adi_adrv9001_TxSignalType_e  outputSignaling; /*!< Output to Analog signal type */
    uint8_t  txPdBiasCurrent;					  /*!< pre-distorter programmable bias current*/
    uint8_t  txPdGainEnable;					  /*!< TX Pre-distortion gain enable */

    uint32_t txPrePdRealPole_kHz;				  /*!< TX Pre-distortion pole */

    uint32_t txPostPdRealPole_kHz;				  /*!< Post-distorter (i.e. interstage) filter Fc  */
    adi_adrv9001_ComponentPowerLevel_e txBbfPower; /*!< Tx baseband filter power mode */
    uint8_t  txExtLoopBackType;					  /*!< 0: No external loopback connect,
                                                   *   1: loopback before PA,
                                                   *   2: loopback after PA. */
    uint8_t  txExtLoopBackForInitCal;			  /*!< 0: ext loop back should not be used for init cal */
    int16_t  txPeakLoopBackPower;                 /*!< Target RMS signal power at input to receiver and its peak-to-average ratio, -40 to +5 dBm */
    uint32_t frequencyDeviation_Hz;               /*!< frequency deviation value in Hz for both FM_IQ and FM_DM.*/
    adi_adrv9001_TxDpProfile_t txDpProfile;       /*!< TX digital data path config */
    adi_adrv9001_SsiConfig_t   txSsiConfig;       /*!< TX Serial data interface config */
} adi_adrv9001_TxProfile_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_TxSettings_t` structure is designed to hold
 * configuration settings for the ADRV9001 transmitter channels. It
 * includes a channel mask to specify which channels should be
 * initialized and an array of `adi_adrv9001_TxProfile_t` structures,
 * each containing detailed settings for individual Tx channels. This
 * structure is essential for managing the initialization and
 * configuration of multiple transmitter channels in the ADRV9001 device.
 *
 * @param txInitChannelMask Tx channel mask of which channels to initialize.
 * @param txProfile Tx settings per Tx channel, defined as an array of
 * adi_adrv9001_TxProfile_t structures.
 ******************************************************************************/
typedef struct adi_adrv9001_TxSettings
{
    uint32_t txInitChannelMask;                                            /*!< Tx channel mask of which channels to initialize */
    adi_adrv9001_TxProfile_t txProfile[ADI_ADRV9001_MAX_TXCHANNELS]; /*!< Tx settings per Tx channel */
} adi_adrv9001_TxSettings_t;

#endif /* _ADI_ADRV9001_TXSETTINGS_TYPES_H_ */
