/**
 * \file
 * \brief Contains ADRV9001 init related private types for
 *        adrv9001_init.c 
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_INIT_TYPES_H_
#define _ADRV9001_INIT_TYPES_H_

#include "adi_adrv9001.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADRV9001_INITIAL_BRING_UP    1
    
#define ADRV9001_PLL_NUM_BANDS     (53u)
#define ADRV9001_CLK_PLL_MODULUS   (8388593u)
#define ADRV9001_VCO_INIT_DEL      (1.0e-6f)    /* VCO calibration initial delay */
#define ADRV9001_VCO_CAL_WAIT      (100.0e-9f)  /* VCO CAL init wait */
#define ADRV9001_VCO_ALC_WAIT      (0.5e-6f)    /* VCO ALC (Auto Level Control) wait */
#define ADRV9001_VCO_CAL_CLK       (40e6f)      /* VCO Cal Clock */

#define ADRV9001_TEMP_SENSOR_DFL_OFFSET_CODE_VALUE (16u)   /* Default RTL offset code value */
#define ADRV9001_KELVIN_TO_CELSIUS (273u)       /* Kelvin to Celsius conversion */

#define ADRV9001_TCIDAC_125C       (328)
#define ADRV9001_TCIDAC_MAX        (4095)

/* Enumation for Coarse and Fine calibrations */
/***************************************************************************//**
 * @brief The `adrv9001_PllVcoCalType_e` is an enumeration that defines the
 * types of VCO (Voltage Controlled Oscillator) calibration available in
 * the ADRV9001 system. It includes two types of calibration: coarse and
 * fine calibration (`PLL_COARSE_FINE_CAL`) and coarse calibration only
 * (`PLL_COARSE_CAL`). This enumeration is used to specify the
 * calibration method for the PLL (Phase-Locked Loop) in the ADRV9001
 * device, which is crucial for ensuring accurate frequency synthesis and
 * stability in RF applications.
 *
 * @param PLL_COARSE_FINE_CAL Represents a VCO Coarse/Fine Calibration.
 * @param PLL_COARSE_CAL Represents a VCO Coarse Calibration.
 ******************************************************************************/
typedef enum adrv9001_PllVcoCalType
{
    PLL_COARSE_FINE_CAL, /*!< VCO Coarse/Fine Cal  */
    PLL_COARSE_CAL       /*!< VCO Coarse Cal    */    
} adrv9001_PllVcoCalType_e;

/***************************************************************************//**
 * @brief The `adrv9001_LfPllMode_e` is an enumeration that defines the
 * operational modes for the loop filter PLL settings in the ADRV9001
 * device. It provides two modes: `PLL_NORM_MODE` for slow mode operation
 * and `PLL_FAST_MODE` for fast mode operation, allowing for flexibility
 * in configuring the loop filter's performance characteristics.
 *
 * @param PLL_NORM_MODE Loopfilter PLL setting in slow mode.
 * @param PLL_FAST_MODE Loopfilter PLL setting in fast mode.
 ******************************************************************************/
typedef enum adrv9001_LfPllMode 
{
    PLL_NORM_MODE = 0x00, /*!< Loopfilter PLL setting In slow mode */
    PLL_FAST_MODE = 0x01  /*!< Loopfilter PLL setting In Fast mode  */
} adrv9001_LfPllMode_e;

/***************************************************************************//**
 * @brief The `adrv9001_PhaseTrackMode_e` is an enumeration that defines the
 * modes of phase tracking for the ADRV9001 device. It provides three
 * options: disabling phase tracking, performing a single phase
 * initialization, or performing a single phase initialization followed
 * by continuous tracking updates. This allows for flexible configuration
 * of phase tracking behavior based on the specific requirements of the
 * application.
 *
 * @param PHSYNC_OFF Phase tracking is disabled.
 * @param PHSYNC_INIT_ONLY Only one phase initialization is performed.
 * @param PHSYNC_CONTINUOUS_TRACK One phase initialization is performed followed
 * by continuous tracking updates.
 ******************************************************************************/
typedef enum adrv9001_PhaseTrackMode
{
    PHSYNC_OFF              = 0x00,     /*!< Phase tracking disabled */
    PHSYNC_INIT_ONLY        = 0x01,     /*!< Do only 1 phase init */
    PHSYNC_CONTINUOUS_TRACK = 0x02      /*!< Do only 1 phase init and continuous track update */
} adrv9001_PhaseTrackMode_e;
    
/***************************************************************************//**
 * @brief The `adrv9001_TempCoef_t` structure is used to define temperature
 * coefficient parameters for the ADRV9001 device. It includes fields for
 * two frequency values in MHz, a bias reference, a bias temperature
 * coefficient, and a VCO varactor temperature coefficient. These
 * parameters are crucial for configuring and calibrating the
 * temperature-dependent behavior of the device's voltage-controlled
 * oscillator (VCO).
 *
 * @param f1_MHz Represents the first frequency in MHz.
 * @param f2_MHz Represents the second frequency in MHz.
 * @param biasRef Represents the VCO bias reference DAC.
 * @param biasTcf Represents the VCO bias DAC temperature coefficient.
 * @param vcoVarTc Represents the number of varactors connected to the output of
 * the VCO temperature compensation IDAC.
 ******************************************************************************/
typedef struct adrv9001_TempCoef 
{
    uint16_t    f1_MHz;
    uint16_t    f2_MHz;  
    uint8_t     biasRef;
    uint8_t     biasTcf;
    uint8_t     vcoVarTc;
} adrv9001_TempCoef_t;

/***************************************************************************//**
 * @brief The `adrv9001_loopFilterResult_t` structure is used to store the
 * results of a loop filter configuration in the ADRV9001 device. It
 * contains values for capacitors (C1, C2, C3) and resistors (R1, R3)
 * that define the loop filter's characteristics, as well as the ICP
 * value for the charge pump. Additionally, it includes the effective
 * loop bandwidth, which is calculated based on the resistor and
 * capacitor values, providing a comprehensive representation of the loop
 * filter's configuration.
 *
 * @param C1 Loopfilter C1 value.
 * @param C2 Loopfilter C2 value.
 * @param C3 Loopfilter C3 value.
 * @param R1 Loopfilter R1 value.
 * @param R3 Loopfilter R3 value.
 * @param ICP Loopfilter ICP (I (current) Charge Pump) value.
 * @param effectiveLoopBW Loopfilter Effective Bandwidth from calculated R/Cs.
 ******************************************************************************/
typedef struct adrv9001_loopFilterResult 
{
    uint8_t      C1;  /*!< Loopfilter C1 value */
    uint8_t      C2;  /*!< Loopfilter C2 value */
    uint8_t      C3;  /*!< Loopfilter C3 value */
    uint8_t      R1;  /*!< Loopfilter R1 value */
    uint8_t      R3;  /*!< Loopfilter R3 value */
    uint8_t      ICP; /*!< Loopfilter ICP (I (current) Charge Pump) value */
    uint32_t     effectiveLoopBW; /*!< Loopfilter Effective Bandwidth from calculated R/Cs */
} adrv9001_loopFilterResult_t;

/* This structure contains the state of the PLL settings */
/***************************************************************************//**
 * @brief The `adrv9001_PllSynthParam_t` structure is a comprehensive data
 * structure used to define the parameters and settings for the PLL
 * synthesizer in the ADRV9001 device. It includes fields for
 * frequencies, dividers, calibration settings, and various control flags
 * that manage the operation and calibration of the PLL and VCO. This
 * structure is crucial for configuring the PLL to achieve desired
 * frequency synthesis and stability, and it supports both normal and
 * debug modes for detailed calibration and testing.
 *
 * @param pllFreq_Hz PLL frequency ranging from 30MHz to 6GHz.
 * @param vcoFreq_Hz VCO frequency in Hz.
 * @param isPllInUse Indicates if the PLL is in use.
 * @param isPathEnabled Indicates if the path is enabled.
 * @param refClkDiv Reference clock divider with possible values 1, 2, or 4.
 * @param k1ClkDiv Device clock divider, which is 2 raised to the power of n
 * (n=0~6).
 * @param refClockFreq_Hz Reference clock frequency, which should be less than
 * or equal to 307.2MHz.
 * @param loDiv LO frequency divider ranging from 2 to 1022 in steps of 2.
 * @param vcoVaractor VCO varactor setting.
 * @param maxCnt Maximum counter for frequency calibration.
 * @param vcoInitDelay Initial delay for VCO calibration.
 * @param vcoClkDiv Clock divider for VCO calibration ALC.
 * @param alcWait Wait period for VCO ALC (Auto Level Control).
 * @param alcInitWait Initial wait period for VCO ALC.
 * @param fractionalByte0 First byte of the PLL fractional part.
 * @param fractionalByte1 Second byte of the PLL fractional part.
 * @param fractionalByte2 Third byte of the PLL fractional part.
 * @param integerByte0 First byte of the PLL integer part.
 * @param integerByte1 Second byte of the PLL integer part.
 * @param biasRef VCO bias reference DAC setting.
 * @param biasTcf Temperature coefficient for VCO bias DAC.
 * @param vcoVarTc Number of varactors connected to the VCO temp comp IDAC
 * output.
 * @param quickFreqCalEn Enables quick frequency calibration for VCO/ALC.
 * @param quickFreqCalThreshold Threshold for quick frequency calibration of
 * VCO/ALC.
 * @param vcoCalMaxCntBandEn Enables max count band for VCO/ALC calibration.
 * @param endVcoCalMaxCntEn Enables end max count for VCO/ALC calibration.
 * @param vcoFineCalEn Enables fine calibration for VCO/ALC.
 * @param temperature Temperature of the PLL.
 * @param bandIndex Index of the PLL band ranging from 0 to 52.
 * @param feCapIndx Index for front-end capacitance.
 * @param locmIndx Index for local oscillator common mode.
 * @param lohilow Indicates high or low state of the local oscillator.
 * @param toneChannel Tone channel with possible values 0, 1, or 0xFF.
 * @param isLfUserOverride Indicates if the loop filter is user overridden.
 * @param loopBW Loop bandwidth setting.
 * @param phaseMargin Phase margin setting.
 * @param voutLvl Output level setting.
 * @param mode Mode of the loop filter PLL.
 * @param phaseSyncMode Phase synchronization mode.
 * @param loopFilter Loop filter result structure containing various parameters.
 * @param dbg_f_alc Debug field for ALC frequency.
 * @param dbg_f_fineband Debug field for fine band frequency.
 * @param dbg_f_coarseband Debug field for coarse band frequency.
 * @param dbg_tcidac Debug field for TC IDAC.
 * @param dbg_f_calbits Debug field for calibration bits.
 ******************************************************************************/
typedef struct adrv9001_PllSynthParam
{
    uint64_t      pllFreq_Hz;      /*!< PLL frequency (30MHz~6GHz) */
    uint64_t      vcoFreq_Hz;      /*!< VCO frequency in Hz */
    uint8_t       isPllInUse;      /*!< PLL in use? */
    uint8_t       isPathEnabled;   /*!< Path enabled? */
    uint8_t       refClkDiv;       /*!< Reference clock divider: 1, 2, 4 */
    uint8_t       k1ClkDiv;        /*!< Device clock divider: 2**n (n=0~6) */
    uint32_t      refClockFreq_Hz; /*!< Reference clock (<=307.2MHz) */
    uint8_t       loDiv;           /*!< LO frequency divider (2~1022, step 2) */
    uint8_t       vcoVaractor;     /*!< VCO varactor: vco_var */
    uint32_t      maxCnt;          /*!< Frequency calibration max counter: freq_cal_max_cnt */
    uint8_t       vcoInitDelay;    /*!< VCO calibration initial delay: vco_cal_init_del */
    uint8_t       vcoClkDiv;       /*!< VCO calibration ALC clock divider: vco_cal_alc_clk_div */
    uint8_t       alcWait;         /*!< VCO ALC (Auto Level Control) wait period: vco_cal_alc_wait */
    uint8_t       alcInitWait;     /*!< VCO ALC initial wait period: vco_cal_alc_init_wait */
    uint8_t       fractionalByte0; /*!< PLL fractional */
    uint8_t       fractionalByte1;
    uint8_t       fractionalByte2;
    uint8_t       integerByte0;    /*!< PLL integer */
    uint8_t       integerByte1;
    uint8_t       biasRef;         /*!< VCO bias reference DAC: vco_bias_ref */
    uint8_t       biasTcf;         /*!< VCO bias DAC temperature coefficient: vco_bias_tcf */
    uint8_t       vcoVarTc;        /*!< Number of varactors connected to the output of the VCO temp comp IDAC: vco_var_tc */
    uint8_t       quickFreqCalEn;  /*!< VCO/ALC calibration: quick_freq_cal_en */
    uint8_t       quickFreqCalThreshold; /*!< VCO/ALC calibration: quick_freq_cal_threshold */
    uint8_t       vcoCalMaxCntBandEn;    /*!< VCO/ALC calibration: vcocal_maxcntband_en */
    uint8_t       endVcoCalMaxCntEn;     /*!< VCO/ALC calibration: endvcocal_maxcnt_en */
    uint8_t       vcoFineCalEn;          /*!< VCO/ALC calibration: vco_fine_cal_en */
    uint16_t      temperature;           /*!< Pll temperature */

    uint8_t       bandIndex;             /*!< PLL band index (0~52) */
    uint8_t       feCapIndx;
    uint8_t       locmIndx;
    uint8_t       lohilow;
    uint8_t       toneChannel;           /*!< Tone channel (0,1, 0xFF) */
    uint8_t       isLfUserOverride;
    uint32_t      loopBW;
    uint32_t      phaseMargin;
    uint8_t       voutLvl;
    adrv9001_LfPllMode_e   mode;
    adrv9001_PhaseTrackMode_e    phaseSyncMode;
    adrv9001_loopFilterResult_t  loopFilter;
#if (ADRV9001_INITIAL_BRING_UP == 1)
    uint16_t      dbg_f_alc;
    uint8_t       dbg_f_fineband;
    uint8_t       dbg_f_coarseband;
    uint16_t      dbg_tcidac;
    uint8_t       dbg_f_calbits;
#endif 
} adrv9001_PllSynthParam_t;

/* This data structure holds the rc tune clock scaling parameters. */
/***************************************************************************//**
 * @brief The `adrv9001_AdcTunerCalClk_t` structure is used to define the
 * parameters for the ADC tuner calibration clock in the ADRV9001 system.
 * It contains two members: `divRatio`, which specifies the divide ratio,
 * and `nScale`, which indicates the normalization scale. These
 * parameters are crucial for configuring the clock scaling during the
 * ADC tuning process, ensuring accurate and efficient calibration.
 *
 * @param divRatio Divide ratio for the ADC tuner calibration clock.
 * @param nScale Normalization scale for the ADC tuner calibration clock.
 ******************************************************************************/
typedef struct adrv9001_AdcTunerCalClk
{
    uint8_t  divRatio; /*!< Divide ratio */
    uint8_t  nScale;   /*!< Normalization scale */
} adrv9001_AdcTunerCalClk_t;
    
/* This is a data structure used to hold the capacitance and conductance scaling factors. */
/***************************************************************************//**
 * @brief The `adrv9001_AdcTunerResult_t` structure is used to store the results
 * of an ADC tuning process, specifically the scaling factors for
 * capacitance and conductance. These factors are represented as unsigned
 * 16-bit integers and are crucial for adjusting the ADC's performance
 * characteristics in the ADRV9001 system.
 *
 * @param sc Capacitance scaling factor.
 * @param sg Conductance scaling factor.
 ******************************************************************************/
typedef struct adrv9001_AdcTunerResult
{
    uint16_t  sc; /*!< capacitance scaling factor */
    uint16_t  sg; /*!< conductance scaling factor */
} adrv9001_AdcTunerResult_t;
    
#ifdef __cplusplus
}
#endif

#endif