/**
 * \file
 * \brief Contains ADRV9001 DPD data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_DPD_TYPES_H_
#define _ADI_ADRV9001_DPD_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

#define ADI_ADRV9001_DPD_NUM_COEFFICIENTS 208

#define INST_TX1_DPD_TOP (0X55000000U) /* tx1_dpd_top: */
#define INST_TX2_DPD_TOP (0X75000000U) /* tx2_dpd_top: */

#define INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_I_N              (0X00004U)    /* Array Unit Address Increment Value */
#define INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_Q_N              (0X00004U)    /* Array Unit Address Increment Value */
#define INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_I_N              (0X00004U)    /* Array Unit Address Increment Value */
#define INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_Q_N              (0X00004U)    /* Array Unit Address Increment Value */

#define IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_I_N(index)            ( (0X00000U)+(index)*(INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_I_N) )    /* No description provided */
#define IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_Q_N(index)            ( (0X04000U)+(index)*(INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_Q_N) )    /* No description provided */
#define IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_I_N(index)            ( (0X08000U)+(index)*(INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_I_N) )    /* No description provided */
#define IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_Q_N(index)            ( (0X0C000U)+(index)*(INCR_IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_Q_N) )    /* No description provided */

#define pREG_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_I_N(base,index)      ((base) + IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_I_N(index))
#define pREG_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_Q_N(base,index)      ((base) + IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_TX_Q_N(index))
#define pREG_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_I_N(base,index)      ((base) + IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_I_N(index))
#define pREG_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_Q_N(base,index)      ((base) + IDX_NVS_AHB_TX_DPD_TOP_DPD_TOP_RX_Q_N(index))

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdAmplifier_e` is an enumeration that defines the
 * types of Digital Pre-Distortion (DPD) amplifiers supported by the
 * ADRV9001 device. It includes options for no amplifier, a default
 * amplifier, and a GaN amplifier, allowing the user to specify the type
 * of amplifier used in the DPD configuration.
 *
 * @param ADI_ADRV9001_DPDAMPLIFIER_NONE Represents the absence of a DPD
 * amplifier.
 * @param ADI_ADRV9001_DPDAMPLIFIER_DEFAULT Represents the default type of DPD
 * amplifier.
 * @param ADI_ADRV9001_DPDAMPLIFIER_GAN Represents a Gallium Nitride (GaN) type
 * DPD amplifier.
 ******************************************************************************/
typedef enum adi_adrv9001_DpdAmplifier
{
    ADI_ADRV9001_DPDAMPLIFIER_NONE,
    ADI_ADRV9001_DPDAMPLIFIER_DEFAULT,
    ADI_ADRV9001_DPDAMPLIFIER_GAN
} adi_adrv9001_DpdAmplifier_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdLutSize_e` is an enumeration that defines the
 * supported sizes for the Digital Pre-Distortion (DPD) Look-Up Table
 * (LUT) in the ADRV9001 device. It provides two options, 256 and 512,
 * which specify the bit size of the LUT used in the DPD process. This
 * enumeration is part of the configuration settings for the DPD feature,
 * allowing users to select the appropriate LUT size based on their
 * application requirements.
 *
 * @param ADI_ADRV9001_DPDLUTSIZE_256 Represents a DPD LUT size of 256.
 * @param ADI_ADRV9001_DPDLUTSIZE_512 Represents a DPD LUT size of 512.
 ******************************************************************************/
typedef enum adi_adrv9001_DpdLutSize
{
    ADI_ADRV9001_DPDLUTSIZE_256,
    ADI_ADRV9001_DPDLUTSIZE_512
} adi_adrv9001_DpdLutSize_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdModel_e` is an enumeration that defines the
 * supported Digital Pre-Distortion (DPD) models for the ADRV9001 device.
 * Each enumerator corresponds to a specific DPD model, identified by a
 * unique integer value. This enumeration is used to specify which DPD
 * model should be applied in the device's configuration, allowing for
 * flexibility in selecting the appropriate model for different
 * operational scenarios.
 *
 * @param ADI_ADRV9001_DPDMODEL_0 Represents the DPD model with identifier 0.
 * @param ADI_ADRV9001_DPDMODEL_1 Represents the DPD model with identifier 1.
 * @param ADI_ADRV9001_DPDMODEL_3 Represents the DPD model with identifier 3.
 * @param ADI_ADRV9001_DPDMODEL_4 Represents the DPD model with identifier 4.
 ******************************************************************************/
typedef enum adi_adrv9001_DpdModel
{
    ADI_ADRV9001_DPDMODEL_0 = 0,
    ADI_ADRV9001_DPDMODEL_1 = 1,
    ADI_ADRV9001_DPDMODEL_3 = 3,
    ADI_ADRV9001_DPDMODEL_4 = 4
} adi_adrv9001_DpdModel_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdInitCfg_t` structure is used to configure the
 * initial settings for the Digital Pre-Distortion (DPD) feature in the
 * ADRV9001 device. It includes fields to enable the DPD actuator,
 * specify the type of power amplifier, define the size of the Look-Up
 * Table (LUT), and select the DPD model. Additionally, it allows for
 * customization of model tap orders and includes settings for LUT
 * scaling and Closed Loop Gain Control (CLGC) enablement. This
 * configuration is crucial for optimizing the performance of the DPD
 * system in the device's signal processing chain.
 *
 * @param enable Indicates whether the DPD actuator is placed in the datapath.
 * @param amplifierType Specifies the type of Power Amplifier used.
 * @param lutSize Defines the bit size of the DPD LUT.
 * @param model Determines the DPD Model to be used.
 * @param changeModelTapOrders Indicates if custom Tap Orders should be used in
 * the model.
 * @param modelOrdersForEachTap Bitmap for each tap in a model to indicate
 * included power terms.
 * @param preLutScale Prescaler for the LUT with a range from 0 to 3.75.
 * @param clgcEnable Indicates if CLGC is enabled.
 ******************************************************************************/
typedef struct adi_adrv9001_DpdInitCfg
{
    bool enable;                                    /*!< When true, the DPD actuator is placed in the datapath and
                                                     *   ADI_ADRV9001_TRACKING_CAL_TX_DPD_CLGC may be used to enable DPD */
    adi_adrv9001_DpdAmplifier_e amplifierType;      //!< Type of Power Amplifier
    adi_adrv9001_DpdLutSize_e lutSize;              //!< Bit size of the DPD LUT
    adi_adrv9001_DpdModel_e model;                  //!< DPD Model to be used
    bool changeModelTapOrders;                      //!< Whether to use the Tap_Orders in #modelOrdersForEachTap
    uint32_t modelOrdersForEachTap[4];              /*!< Bitmap for each of the taps in a model to indicate which power
                                                     *   terms are included in the model (and thus the auto-correlation
                                                     *   matrix). Used when #changeModelTapOrders is true */
    uint8_t preLutScale;                            //!< Prescaler for the LUT (U2.2; min = 0; max = 3.75) */
    uint8_t clgcEnable;                             //!< CLGC Enable */
} adi_adrv9001_DpdInitCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdCfg_t` structure is used to configure the Digital
 * Pre-Distortion (DPD) settings for the ADRV9001 device. It includes
 * parameters for sample count, power scaling, signal power thresholds,
 * and detection thresholds, as well as flags for LUT switching and
 * special frame usage. The structure also contains settings for CLGC
 * (Closed Loop Gain Control) such as loop status, gain targets, and
 * filter coefficients. Additionally, it specifies the capture delay time
 * and the sampling rate for the DPD actuator and capture, providing a
 * comprehensive configuration for optimizing signal alignment and power
 * management in the device.
 *
 * @param numberOfSamples Number of samples to use for DPD, ranging from 3 to
 * 4096.
 * @param additionalPowerScale Additional scaling on the 2nd and higher order
 * power terms to maintain nominal magnitude.
 * @param rxTxNormalizationLowerThreshold Tx signal power lower threshold for
 * Rx/Tx data alignment, with a range of
 * 0 to 1.0.
 * @param rxTxNormalizationUpperThreshold Tx signal power upper threshold for
 * Rx/Tx data alignment, with a range of
 * 0 to 1.0.
 * @param detectionPowerThreshold Signal power threshold for detection, in U1.31
 * format.
 * @param detectionPeakThreshold Signal power threshold for peak detection, in
 * U1.31 format.
 * @param countsLessThanPowerThreshold Number of points below
 * detectionPowerThreshold that causes
 * capture discard, disable with 4096.
 * @param countsGreaterThanPeakThreshold Number of points above
 * detectionPeakThreshold that causes
 * capture discard, disable with 0.
 * @param immediateLutSwitching Indicates if the LUT switches immediately or at
 * the end of a Tx frame.
 * @param useSpecialFrame Indicates if DPD runs only on a user-indicated special
 * frame.
 * @param resetLuts Indicates if LUTs are reset to unity, always read as 0 and
 * self-clearing.
 * @param timeFilterCoefficient Time filter coefficient in U1.31 format.
 * @param dpdSamplingRate_Hz Sampling rate in Hz for the DPD actuator and
 * capture, read-only.
 * @param clgcLoopOpen Indicates if the loop is open and TX attenuators are not
 * updated, used to measure target gain.
 * @param clgcGainTarget_HundredthdB Target gain sent in 1/100 dB.
 * @param clgcFilterAlpha Filter coefficient for the filtered gain values.
 * @param clgcLastGain_HundredthdB Last gain value, only valid during Get.
 * @param clgcFilteredGain_HundredthdB Filtered gain value, only valid during
 * Get.
 * @param captureDelay_us Time delay for capture relative to the start of the
 * frame, applicable to both DPD and CLGC.
 ******************************************************************************/
typedef struct adi_adrv9001_DpdCfg
{
    uint32_t numberOfSamples;           //!< Number of samples to use for DPD (min: 3; max: 4096) */

    /** Additional scaling on the 2nd and higher order power terms.
     * Used to keep the nominal magnitude of each term about the same
     */
    uint32_t additionalPowerScale;

    /** Tx signal power for the lower threshold for Rx/Tx data alignment (U2.30; min = 0; max = 1.0).
     * Where power \f$p(n) = x(n) x^*(n)\f$ and \f$x(n)\f$ is the complex signal data.
     * \note May read back a slightly different value than written due to floating point conversion error
     */
    uint32_t rxTxNormalizationLowerThreshold;

    /** Tx signal power for the upper threshold for Rx/Tx data alignment (U2.30; min = 0; max = 1.0).
     * Where power \f$p(n) = x(n) x^*(n)\f$ and \f$x(n)\f$ is the complex signal data.
     * \note May read back a slightly different value than written due to floating point conversion error
     */
    uint32_t rxTxNormalizationUpperThreshold;
    
    uint32_t detectionPowerThreshold;   //!< Signal power for the power threshold (U1.31) */
    uint32_t detectionPeakThreshold;    //!< Signal power for the peak threshold (U1.31) */
    
    /** If the number of points below the detectionPowerThreshold exceeds this number, the capture is discarded.
     * To disable, set to 4096
     */
    uint16_t countsLessThanPowerThreshold;
    
    /** If the number of points above the detectionPeakThreshold is less than this number, the capture is discarded.
     * To disable, set to 0
     */
    uint16_t countsGreaterThanPeakThreshold;
    
    bool immediateLutSwitching;             //!< Whether the LUT switches immediately or at the end of a Tx frame */
    bool useSpecialFrame;                   //!< Whether to only run DPD on a user indicated special frame */
    bool resetLuts;                         //!< Whether to reset LUTs to unity. Always read as 0 and is self-clearing */
    uint32_t timeFilterCoefficient;         /*!< Time filter coefficient in U1.31 format */
    uint32_t dpdSamplingRate_Hz;            /*!< sampling rate in Hz for the DPD actuator and capture.
                                                 'dpdSamplingRate_Hz' is read only and is ignored in adi_adrv9001_dpd_Configure() */
	uint8_t clgcLoopOpen;                   /*!< If true, the loop is open and the TX attenuators are not updated.  Used to measure a target gain. */
	int32_t clgcGainTarget_HundredthdB;     /*!< Sent in 1/100 dB. */
	uint32_t clgcFilterAlpha;               /*!< filter coefficient for the filtered  gain values. */
	int32_t clgcLastGain_HundredthdB;       /*!< last gain.  Only valid during Get. */
	int32_t clgcFilteredGain_HundredthdB;   /*!< filtered gain.  Only valid during Get. */
	
	uint32_t captureDelay_us;               /*!< Amount of time that capture will be delayed (beyond normal) relative to the start of the frame.  This parameter applies to both DPD and CLGC. */
} adi_adrv9001_DpdCfg_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdCoefficients_t` structure is used to store
 * coefficients for the Digital Pre-Distortion (DPD) feature in the
 * ADRV9001 device. It contains a `region` field that specifies the
 * region of the Look-Up Table (LUT) initialization data, which can range
 * from 0 to 7, and a `coefficients` array that holds the DPD
 * coefficients, with the size defined by the constant
 * `ADI_ADRV9001_DPD_NUM_COEFFICIENTS`. This structure is essential for
 * configuring and managing the DPD functionality, which is used to
 * improve the linearity of power amplifiers in RF systems.
 *
 * @param region The region of the LUT initialization data (valid 0 - 7).
 * @param coefficients DPD coefficients stored in an array of size
 * ADI_ADRV9001_DPD_NUM_COEFFICIENTS.
 ******************************************************************************/
typedef struct adi_adrv9001_DpdCoefficients
{
    uint8_t region;     //!< The region of the LUT initialization data (valid 0 - 7)
    uint8_t coefficients[ADI_ADRV9001_DPD_NUM_COEFFICIENTS];    //!< DPD coefficients
}adi_adrv9001_DpdCoefficients_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DpdFhRegions_t` structure defines a frequency region
 * for the ADRV9001 DPD (Digital Pre-Distortion) functionality,
 * specifying a range of carrier frequencies that are included in the
 * region. It uses two 64-bit unsigned integers to denote the start and
 * end frequencies in Hertz, allowing precise definition of the frequency
 * boundaries for DPD operations.
 *
 * @param startFrequency_Hz Carrier frequency greater than or equal to this is
 * included in the region.
 * @param endFrequency_Hz Carrier frequency less than this is included in the
 * region.
 ******************************************************************************/
typedef struct adi_adrv9001_DpdFhRegions
{
    uint64_t startFrequency_Hz; //!< Carrier frequency greater than or equal to this is included in the region
    uint64_t endFrequency_Hz;   //!< Carrier frequency less than this is included in the region
} adi_adrv9001_DpdFhRegions_t;

#endif /* _ADI_ADRV9001_DPD_TYPES_H_ */
