/**
 * \file
 * \brief Type definitions for ADRV9001 PFIR buffers
 * \copyright Analog Devices Inc. 2019. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_PFIRBUFFER_TYPES_H_
#define _ADI_ADRV9001_PFIRBUFFER_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_adrv9001_defines.h"

#define ADI_ADRV9001_NUM_GAIN_COMP9						6U
#define ADI_ADRV9001_MAG_COMP_NB_PFIR_COEFS_MAX_SIZE	13
#define ADI_ADRV9001_MAG_COMP_PFIR_COEFS_MAX_SIZE		21
#define ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE			128

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirSymmetric_e` is an enumeration that defines the
 * symmetricity of PFIR (Programmable Finite Impulse Response)
 * coefficients. It provides two possible values:
 * `ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC` indicating that the
 * coefficients are non-symmetric, and `ADI_ADRV9001_PFIR_COEF_SYMMETRIC`
 * indicating that the coefficients are symmetric. This enumeration is
 * used to specify the expected symmetry property of the PFIR
 * coefficients in the ADRV9001 device.
 *
 * @param ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC Coefficients are expected to be
 * non-symmetric.
 * @param ADI_ADRV9001_PFIR_COEF_SYMMETRIC Coefficients are expected to be
 * symmetric.
 ******************************************************************************/
typedef enum adi_adrv9001_PfirSymmetric
{
    ADI_ADRV9001_PFIR_COEF_NON_SYMMETRIC = 0u, /*!< Coefficients are expected to be non-symmetric */
    ADI_ADRV9001_PFIR_COEF_SYMMETRIC = 1u,     /*!< Coefficients are expected to be symmetric */
} adi_adrv9001_PfirSymmetric_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirNumTaps_e` is an enumeration that defines the
 * number of taps available for a Programmable Finite Impulse Response
 * (PFIR) filter in the ADRV9001 device. Each enumerator corresponds to a
 * specific number of taps, ranging from 32 to 128, which are used to
 * configure the PFIR filter's complexity and performance
 * characteristics. The `ADI_ADRV9001_PFIR_TAPS_MAX_ID` serves as a
 * boundary marker for the maximum tap ID value.
 *
 * @param ADI_ADRV9001_PFIR_32_TAPS Represents a PFIR filter with 32 taps.
 * @param ADI_ADRV9001_PFIR_64_TAPS Represents a PFIR filter with 64 taps.
 * @param ADI_ADRV9001_PFIR_96_TAPS Represents a PFIR filter with 96 taps.
 * @param ADI_ADRV9001_PFIR_128_TAPS Represents a PFIR filter with 128 taps.
 * @param ADI_ADRV9001_PFIR_TAPS_MAX_ID Defines the maximum ID value for PFIR
 * taps.
 ******************************************************************************/
typedef enum adi_adrv9001_PfirNumTaps
{
    ADI_ADRV9001_PFIR_32_TAPS = 0u,     /*!<  32 taps PFIR */
    ADI_ADRV9001_PFIR_64_TAPS = 1u,     /*!<  64 taps PFIR */
    ADI_ADRV9001_PFIR_96_TAPS = 2u,     /*!<  96 taps PFIR */
    ADI_ADRV9001_PFIR_128_TAPS = 3u,    /*!< 128 taps PFIR */
    ADI_ADRV9001_PFIR_TAPS_MAX_ID = 3u, /*!< PFIR taps max ID */
} adi_adrv9001_PfirNumTaps_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirGain_e` is an enumeration that defines various
 * gain levels for Programmable Finite Impulse Response (PFIR) filters in
 * the ADRV9001 device. Each enumerator corresponds to a specific gain
 * value in decibels (dB), ranging from -12 dB to +26 dB, allowing for
 * precise control over the gain applied to the PFIR filters. This
 * enumeration is used to select the desired gain setting for different
 * PFIR configurations within the ADRV9001 API.
 *
 * @param ADI_ADRV9001_PFIR_GAIN_NEG_12_DB Represents a PFIR gain of -12 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_NEG_6_DB Represents a PFIR gain of -6 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_ZERO_DB Represents a PFIR gain of 0 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_POS_6_DB Represents a PFIR gain of +6 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_POS_9_54_DB Represents a PFIR gain of +9.54 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_POS_12_DB Represents a PFIR gain of +12 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_POS_14_DB Represents a PFIR gain of +14 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_POS_20_DB Represents a PFIR gain of +20 dB.
 * @param ADI_ADRV9001_RX_MAX Represents the maximum value for RX gain, set to
 * 7u.
 * @param ADI_ADRV9001_PFIR_GAIN_PLUS_24DB Represents a PFIR gain of +24 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_PLUS_26DB Represents a PFIR gain of +26 dB.
 * @param ADI_ADRV9001_PFIR_GAIN_MAX Represents the maximum value for PFIR gain,
 * set to 9u.
 ******************************************************************************/
typedef enum adi_adrv9001_PfirGain
{
    ADI_ADRV9001_PFIR_GAIN_NEG_12_DB = 0u,   /*!< -12dB */
    ADI_ADRV9001_PFIR_GAIN_NEG_6_DB = 1u,    /*!< - 6dB */
    ADI_ADRV9001_PFIR_GAIN_ZERO_DB = 2u,     /*!<   0dB */
    ADI_ADRV9001_PFIR_GAIN_POS_6_DB = 3u,    /*!< + 6dB */
    ADI_ADRV9001_PFIR_GAIN_POS_9_54_DB = 4u, /*!< + 9.54dB */
    ADI_ADRV9001_PFIR_GAIN_POS_12_DB = 5u,   /*!< +12dB */
    ADI_ADRV9001_PFIR_GAIN_POS_14_DB = 6u,   /*!< +14dB */
    ADI_ADRV9001_PFIR_GAIN_POS_20_DB = 7u,   /*!< +20dB */
    ADI_ADRV9001_RX_MAX = 7u,

    ADI_ADRV9001_PFIR_GAIN_PLUS_24DB = 8u, /*!< +24dB */
    ADI_ADRV9001_PFIR_GAIN_PLUS_26DB = 9u, /*!< +26dB */
    ADI_ADRV9001_PFIR_GAIN_MAX = 9u,
} adi_adrv9001_PfirGain_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirWbNbBuffer_t` structure is designed to hold the
 * configuration and coefficients for a wideband/narrowband Programmable
 * Finite Impulse Response (PFIR) filter used in the ADRV9001 system. It
 * includes fields for specifying the number of coefficients, the
 * symmetry of the coefficients, the number of filter taps, and the gain
 * setting. The coefficients themselves are stored in an array, allowing
 * for flexible filter design within the constraints of the maximum size
 * defined by the system.
 *
 * @param numCoeff Number of coefficients in the PFIR buffer.
 * @param symmetricSel Selection for symmetricity of the PFIR coefficients.
 * @param tapsSel Selection for the number of taps in the PFIR filter.
 * @param gainSel Selection for the gain applied to the PFIR filter.
 * @param coefficients Array holding the PFIR coefficients, with a maximum size
 * defined by ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE.
 ******************************************************************************/
typedef struct adi_adrv9001_PfirWbNbBuffer
{
    uint8_t numCoeff;                                             /*!< number of coefficients */
    adi_adrv9001_PfirSymmetric_e symmetricSel;                    /*!< symmetric selection */
    adi_adrv9001_PfirNumTaps_e tapsSel;                           /*!< taps selection */
    adi_adrv9001_PfirGain_e gainSel;                              /*!< gain selection */
    int32_t coefficients[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE]; /*!< coefficients */
} adi_adrv9001_PfirWbNbBuffer_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirPulseBuffer_t` structure is designed to hold the
 * configuration and coefficients for a pulse shaping filter in the
 * ADRV9001 RF transceiver. It includes fields for specifying the number
 * of coefficients, the symmetry of the coefficients, the number of taps,
 * the gain applied, and an array to store the coefficients themselves.
 * This structure is used to configure the pulse shaping filters for
 * narrowband applications, allowing for precise control over the filter
 * characteristics.
 *
 * @param numCoeff Number of coefficients in the pulse buffer.
 * @param symmetricSel Selection for symmetricity of the coefficients.
 * @param taps Number of taps in the pulse buffer.
 * @param gainSel Selection for gain applied to the coefficients.
 * @param coefficients Array holding the coefficients for the pulse buffer.
 ******************************************************************************/
typedef struct adi_adrv9001_PfirPulseBuffer_t
{
    uint8_t numCoeff;                                             /*!< number of coefficients */
    adi_adrv9001_PfirSymmetric_e symmetricSel;                    /*!< symmetric selection */
    uint8_t taps;                                                 /*!< taps in number */
    adi_adrv9001_PfirGain_e gainSel;                              /*!< gain selection */
    int32_t coefficients[ADI_ADRV9001_WB_NB_PFIR_COEFS_MAX_SIZE]; /*!< coefficients   */
} adi_adrv9001_PfirPulseBuffer_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirMag21Buffer_t` structure is designed to hold the
 * coefficients for a PFIR (Programmable Finite Impulse Response) filter
 * used in RX Low/High TIA Bandwidth HP/LP ADC applications. It contains
 * a field for the number of coefficients and an array to store the
 * actual coefficient values, with a maximum size of 21. This structure
 * is part of the ADRV9001 API, which is used for configuring and
 * managing the PFIR filters in the ADRV9001 transceiver.
 *
 * @param numCoeff Stores the number of coefficients in the buffer.
 * @param coefficients An array holding the PFIR coefficients with a maximum
 * size defined by
 * ADI_ADRV9001_MAG_COMP_PFIR_COEFS_MAX_SIZE.
 ******************************************************************************/
typedef struct adi_adrv9001_PfirMag21Buffer
{
    uint8_t numCoeff;
    int32_t coefficients[ADI_ADRV9001_MAG_COMP_PFIR_COEFS_MAX_SIZE];
} adi_adrv9001_PfirMag21Buffer_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirMag13Buffer_t` structure is designed to hold the
 * magnitude compensation PFIR coefficients for narrowband (NB)
 * applications in the ADRV9001 device. It contains a count of the
 * coefficients and an array to store the actual coefficient values,
 * which are used to adjust the magnitude response of the signal
 * processing chain. This structure is part of a larger set of data
 * structures that manage various PFIR configurations for the ADRV9001,
 * facilitating dynamic profile switching and signal processing
 * customization.
 *
 * @param numCoeff Represents the number of coefficients in the buffer.
 * @param coefficients An array of integers storing the PFIR coefficients, with
 * a maximum size defined by
 * ADI_ADRV9001_MAG_COMP_NB_PFIR_COEFS_MAX_SIZE.
 ******************************************************************************/
typedef struct
{
    uint8_t numCoeff;
    int32_t coefficients[ADI_ADRV9001_MAG_COMP_NB_PFIR_COEFS_MAX_SIZE];
} adi_adrv9001_PfirMag13Buffer_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirBuffer_t` structure is designed to hold various
 * types of PFIR (Programmable Finite Impulse Response) coefficients used
 * in the ADRV9001 system for both RX and TX paths. It includes
 * coefficients for wideband/narrowband compensation, pulse shaping, and
 * magnitude compensation across different banks and channels. This
 * structure is crucial for dynamic profile switching, where specific
 * PFIRs need to be reloaded for each profile. The structure supports
 * multiple banks (A, B, C, D) and channels, providing flexibility in
 * configuring the signal processing chain for different operational
 * modes.
 *
 * @param pfirRxWbNbChFilterCoeff_A RX WB/NB Compensation PFIR coefficient Bank
 * A.
 * @param pfirRxWbNbChFilterCoeff_B RX WB/NB Compensation PFIR coefficient Bank
 * B.
 * @param pfirRxWbNbChFilterCoeff_C RX WB/NB Compensation PFIR coefficient Bank
 * C.
 * @param pfirRxWbNbChFilterCoeff_D RX WB/NB Compensation PFIR coefficient Bank
 * D.
 * @param pfirTxWbNbPulShpCoeff_A TX WB/NB Preprocessing pulse shaping PFIR
 * coefficient Bank A.
 * @param pfirTxWbNbPulShpCoeff_B TX WB/NB Preprocessing pulse shaping PFIR
 * coefficient Bank B.
 * @param pfirTxWbNbPulShpCoeff_C TX WB/NB Preprocessing pulse shaping PFIR
 * coefficient Bank C.
 * @param pfirTxWbNbPulShpCoeff_D TX WB/NB Preprocessing pulse shaping PFIR
 * coefficient Bank D.
 * @param pfirRxNbPulShp RX NB Pulse Shaping pFIR 128 taps for channels 1 and 2.
 * @param pfirRxMagLowTiaLowSRHp Channel 1/2 Low TIA Bandwidth HP ADC.
 * @param pfirRxMagLowTiaHighSRHp Channel 1/2 Low TIA Bandwidth HP ADC with high
 * sample rate.
 * @param pfirRxMagHighTiaHighSRHp Channel 1/2 High TIA Bandwidth HP ADC with
 * high sample rate.
 * @param pfirRxMagLowTiaLowSRLp Channel 1/2 Low TIA Bandwidth LP ADC.
 * @param pfirRxMagLowTiaHighSRLp Channel 1/2 Low TIA Bandwidth LP ADC with high
 * sample rate.
 * @param pfirRxMagHighTiaHighSRLp Channel 1/2 High TIA Bandwidth LP ADC with
 * high sample rate.
 * @param pfirTxMagComp1 TX Magnitude Compensation PFIR with 21 taps.
 * @param pfirTxMagComp2 Second TX Magnitude Compensation PFIR with 21 taps.
 * @param pfirTxMagCompNb TX Magnitude Compensation PFIR for NB.
 * @param pfirRxMagCompNb RX Magnitude Compensation PFIR for NB.
 ******************************************************************************/
typedef struct adi_adrv9001_PfirBuffer
{
    /*!< During dynamic profile switching, the first two types of PFIRs, RXWbNb and TXWbNbPulShp, would be reloaded for each
     *   profile before the switch */
    /*!< RX WB/NB Compensation PFIR (Channel Filter) coefficient Bank A/B/C/D  in rxnb_dem, block30 */
    adi_adrv9001_PfirWbNbBuffer_t pfirRxWbNbChFilterCoeff_A;
    adi_adrv9001_PfirWbNbBuffer_t pfirRxWbNbChFilterCoeff_B;
    adi_adrv9001_PfirWbNbBuffer_t pfirRxWbNbChFilterCoeff_C;
    adi_adrv9001_PfirWbNbBuffer_t pfirRxWbNbChFilterCoeff_D;

    /*!< TX WB/NB Preprocessing pulse shaping PFIR coefficient Bank A/B/C/D */
    adi_adrv9001_PfirWbNbBuffer_t pfirTxWbNbPulShpCoeff_A;
    adi_adrv9001_PfirWbNbBuffer_t pfirTxWbNbPulShpCoeff_B;
    adi_adrv9001_PfirWbNbBuffer_t pfirTxWbNbPulShpCoeff_C;
    adi_adrv9001_PfirWbNbBuffer_t pfirTxWbNbPulShpCoeff_D;

    /*!< RX NB Pulse Shaping pFIR  128 taps CH1/2 in rxnb_dem */
    adi_adrv9001_PfirPulseBuffer_t pfirRxNbPulShp[ADI_ADRV9001_MAX_RX_ONLY];

    /*!< Channel 1/2 Low/High TIA Bandwidth HP/LP ADC */
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagLowTiaLowSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagLowTiaHighSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagHighTiaHighSRHp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagLowTiaLowSRLp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagLowTiaHighSRLp[ADI_ADRV9001_MAX_RX_ONLY];
    adi_adrv9001_PfirMag21Buffer_t pfirRxMagHighTiaHighSRLp[ADI_ADRV9001_MAX_RX_ONLY];

    /*!< TX Magnitude Compensation PFIR 21 taps */
    adi_adrv9001_PfirMag21Buffer_t pfirTxMagComp1;
    adi_adrv9001_PfirMag21Buffer_t pfirTxMagComp2;

    /*!< TX/RX Magnitude Compensation PFIR for NB */
    adi_adrv9001_PfirMag13Buffer_t pfirTxMagCompNb[ADI_ADRV9001_MAX_TXCHANNELS];
    adi_adrv9001_PfirMag13Buffer_t pfirRxMagCompNb[ADI_ADRV9001_MAX_RX_ONLY];
} adi_adrv9001_PfirBuffer_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirBank_e` is an enumeration that defines the
 * selection of PFIR banks, labeled A through D, each represented by a
 * unique unsigned integer value. This enumeration is used to specify
 * which PFIR bank is being referenced or utilized in the context of
 * configuring or managing PFIR buffers within the ADRV9001 system.
 *
 * @param ADI_ADRV9001_PFIR_BANK_A Represents PFIR bank A with a value of 0.
 * @param ADI_ADRV9001_PFIR_BANK_B Represents PFIR bank B with a value of 1.
 * @param ADI_ADRV9001_PFIR_BANK_C Represents PFIR bank C with a value of 2.
 * @param ADI_ADRV9001_PFIR_BANK_D Represents PFIR bank D with a value of 3.
 ******************************************************************************/
typedef enum adi_adrv9001_PfirBank
{
    ADI_ADRV9001_PFIR_BANK_A = 0u, /*!< PFIR bank A */
    ADI_ADRV9001_PFIR_BANK_B = 1u, /*!< PFIR bank B */
    ADI_ADRV9001_PFIR_BANK_C = 2u, /*!< PFIR bank C */
    ADI_ADRV9001_PFIR_BANK_D = 3u, /*!< PFIR bank D */
} adi_adrv9001_PfirBank_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirCoeff_t` structure is a simple data structure
 * used to hold the number of coefficients for a PFIR (Programmable
 * Finite Impulse Response) filter. It contains a single member,
 * `numCoeff`, which is a 32-bit unsigned integer representing the count
 * of coefficients. This structure is part of the ADRV9001 API, which is
 * used for configuring and managing PFIR filters in Analog Devices'
 * ADRV9001 transceiver.
 *
 * @param numCoeff Number of Coefficients
 ******************************************************************************/
typedef struct adi_adrv9001_PfirCoeff
{
    uint32_t numCoeff; /*!< Number of Coefficients */
} adi_adrv9001_PfirCoeff_t;

/* TODO: With new PFIR structure change, this structure is not used anywhere now.
         But according to Jim Bush, this structure will be used in dynamic profile switching */
/***************************************************************************//**
 * @brief The `adi_adrv9001_PfirWbNbConfig_t` structure is used to configure the
 * wideband/narrowband PFIR (Programmable Finite Impulse Response) filter
 * settings for the ADRV9001 device. It includes fields for selecting the
 * PFIR bank, determining the symmetry of the coefficients, specifying
 * the number of filter taps, setting the gain, and providing the
 * coefficients themselves. This structure is essential for defining the
 * filter characteristics used in signal processing within the device.
 *
 * @param bankSel Specifies the PFIR bank selection, which can be one of four
 * banks: A, B, C, or D.
 * @param symmetricSel Indicates whether the PFIR coefficients are symmetric or
 * non-symmetric.
 * @param tapsSel Determines the number of taps in the PFIR filter.
 * @param gainSel Specifies the gain in decibels for the PFIR filter.
 * @param coeff Holds the PFIR coefficients.
 ******************************************************************************/
typedef struct adi_adrv9001_PfirWbNbConfig
{
    adi_adrv9001_PfirBank_e      bankSel;		/*!< bank: PFIR_BANK_A = 0u (bank A), PFIR_BANK_B = 1u (bank B),
                                                           PFIR_BANK_C = 2u (bank C), PFIR_BANK_D = 3u (bank D) */
    adi_adrv9001_PfirSymmetric_e symmetricSel;	/*!< PFIR coefficient symmetricity */
    adi_adrv9001_PfirNumTaps_e   tapsSel;		/*!< Number of taps */
    adi_adrv9001_PfirGain_e      gainSel;		/*!< gain in dB */
    adi_adrv9001_PfirCoeff_t     coeff;         /*!< PFIR coefficients */
} adi_adrv9001_PfirWbNbConfig_t;

#endif /* _ADI_ADRV9001_PFIRBUFFER_TYPES_H_ */
