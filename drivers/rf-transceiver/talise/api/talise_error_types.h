/**
 * \file talise_error_types.h
 * \brief Contains Talise data types for API Error messaging
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_ERROR_TYPES_H_
#define TALISE_ERROR_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_arm_macros.h"

/***************************************************************************//**
 * @brief The `taliseErrSources_t` is an enumeration that defines various
 * sources of error codes within the Talise API. Each enumerator
 * represents a specific source of error, such as API errors, ADI HAL
 * errors, ARM command errors, GPIO errors, and initialization
 * calibration errors. Additionally, there are reserved error sources for
 * future use. This enumeration is used to categorize and identify the
 * origin of errors encountered during the operation of the Talise API,
 * facilitating error handling and debugging.
 *
 * @param TAL_ERRSRC_API API Error Source: Error codes defined by taliseErr_t.
 * @param TAL_ERRSRC_ADIHAL ADI HAL Error Source: Error codes defined by
 * adiHalErr_t.
 * @param TAL_ERRSRC_TALARMCMD TALISE ARM returned error.
 * @param TAL_ERRSRC_TAL_API_GPIO TALISE GPIO returned error.
 * @param TAL_ERRSRC_TALAPIARM TALISE API returned ARM error.
 * @param TAL_ERRSRC_INITCALS TALISE INITCALS returned error.
 * @param TAL_ERRSRC_TAL_API_C0_PCA Reserved Error source.
 * @param TAL_ERRSRC_TAL_API_C0_PHMFOVR Reserved Error source.
 ******************************************************************************/
typedef enum {
	TAL_ERRSRC_API,     /*!<API Error Src: Error codes defined by taliseErr_t */
	TAL_ERRSRC_ADIHAL,  /*!<ADI HAL Error Src: Error codes defined by adiHalErr_t*/
	TAL_ERRSRC_TALARMCMD,  /*!< TALISE ARM returned error */
	TAL_ERRSRC_TAL_API_GPIO,     /*!< TALISE GPIO returned error */
	TAL_ERRSRC_TALAPIARM,  /*!< TALISE API returned ARM error */
	TAL_ERRSRC_INITCALS,   /*!< TALISE INITCALS returned error */
	TAL_ERRSRC_TAL_API_C0_PCA, /*!< Reserved Error source */
	TAL_ERRSRC_TAL_API_C0_PHMFOVR /*!< Reserved Error source */
} taliseErrSources_t;

/***************************************************************************//**
 * @brief The `taliseErr_t` is an enumeration that defines a comprehensive set
 * of error codes used within the Talise API to indicate various error
 * conditions. Each enumerated value corresponds to a specific error
 * scenario, facilitating easier debugging and error handling in the API.
 * The enumeration includes error codes for invalid parameters, timeouts,
 * conflicts, and other specific conditions encountered during the
 * operation of the Talise API. The last enumerated value,
 * `TAL_ERR_NUMBER_OF_ERRORS`, serves as a reference to the total number
 * of error codes defined in this enumeration.
 *
 * @param TAL_ERR_OK Indicates no error occurred.
 * @param TAL_ERR_INV_NULL_INIT_PARAM Error for invalid or null initialization
 * parameter.
 * @param TAL_ERR_WAITFOREVENT_INV_PARM Error for invalid parameter in wait for
 * event function.
 * @param TAL_ERR_CLKPLL_INV_HSDIV Error for invalid high-speed divider in clock
 * PLL.
 * @param TAL_ERR_SETCLKPLL_INV_VCOINDEX Error for invalid VCO index in set
 * clock PLL.
 * @param TAL_ERR_SETCLKPLL_INV_NDIV Error for invalid N divider in set clock
 * PLL.
 * @param TAL_ERR_SETRFPLL_INV_PLLNAME Error for invalid PLL name in set RF PLL.
 * @param TAL_ERR_SETRFPLL_INITCALS_INPROGRESS Error indicating initial
 * calibrations are in progress for
 * set RF PLL.
 * @param TAL_ERR_INV_SCALEDDEVCLK_PARAM Error for invalid scaled device clock
 * parameter.
 * @param TAL_ERR_SETRFPLL_INV_REFCLK Error for invalid reference clock in set
 * RF PLL.
 * @param TAL_ERR_SETORXGAIN_INV_ORXPROFILE Error for invalid observation
 * receiver profile in set ORX gain.
 * @param TAL_ERR_SETORXGAIN_INV_CHANNEL Error for invalid channel in set ORX
 * gain.
 * @param TAL_ERR_SETORXGAIN_INV_ORX1GAIN Error for invalid ORX1 gain in set ORX
 * gain.
 * @param TAL_ERR_SETORXGAIN_INV_ORX2GAIN Error for invalid ORX2 gain in set ORX
 * gain.
 * @param TAL_ERR_SETTXATTEN_INV_STEPSIZE_PARM Error for invalid step size
 * parameter in set TX attenuation.
 * @param TAL_ERR_SETRX1GAIN_INV_GAIN_PARM Error for invalid gain parameter in
 * set RX1 gain.
 * @param TAL_ERR_SETRX2GAIN_INV_GAIN_PARM Error for invalid gain parameter in
 * set RX2 gain.
 * @param TAL_ERR_SER_INV_M_PARM Error for invalid M parameter in serializer.
 * @param TAL_ERR_SER_INV_NP_PARM Error for invalid NP parameter in serializer.
 * @param TAL_ERR_SER_INV_L_PARM Error for invalid L parameter in serializer.
 * @param TAL_ERR_SER_INV_ORX_L_PARM Error for invalid ORX L parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_ORX_M_PARM Error for invalid ORX M parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_ORX_NP_PARM Error for invalid ORX NP parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_ORX_LANEEN_PARM Error for invalid ORX lane enable
 * parameter in serializer.
 * @param TAL_ERR_SER_INV_LANERATE_PARM Error for invalid lane rate parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_ORX_LANERATE_PARM Error for invalid ORX lane rate
 * parameter in serializer.
 * @param TAL_ERR_SER_INV_LANEEN_PARM Error for invalid lane enable parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_AMP_PARM Error for invalid amplitude parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_PREEMP_PARM Error for invalid pre-emphasis parameter
 * in serializer.
 * @param TAL_ERR_SER_INV_LANEPN_PARM Error for invalid lane PN parameter in
 * serializer.
 * @param TAL_ERR_SER_LANE_CONFLICT_PARM Error for lane conflict parameter in
 * serializer.
 * @param TAL_ERR_SER_INV_TXSER_DIV_PARM Error for invalid TX serializer
 * division parameter.
 * @param TAL_ERR_SER_LANE_RATE_CONFLICT_PARM Error for lane rate conflict
 * parameter in serializer.
 * @param TAL_ERR_SER_INV_ZIF_TO_RIF_DATA_PARM Error for invalid ZIF to RIF data
 * parameter in serializer.
 * @param TAL_ERR_SER_INV_DUALBAND_DATA_PARM Error for invalid dual-band data
 * parameter in serializer.
 * @param TAL_ERR_SER_INV_RXFRAMER_SEL Error for invalid RX framer selection in
 * serializer.
 * @param TAL_ERR_SER_INV_ORXFRAMER_SEL Error for invalid ORX framer selection
 * in serializer.
 * @param TAL_ERR_SER_LANERATE_ZERO Error indicating lane rate is zero in
 * serializer.
 * @param TAL_ERR_HS_AND_LANE_RATE_NOT_INTEGER_MULT Error indicating high-speed
 * and lane rate are not
 * integer multiples.
 * @param TAL_ERR_DESES_HS_AND_LANE_RATE_NOT_INTEGER_MULT Error indicating
 * deserializer high-
 * speed and lane rate
 * are not integer
 * multiples.
 * @param TAL_ERR_DESES_INV_LANE_RATE Error for invalid lane rate in
 * deserializer.
 * @param TAL_ERR_DESES_INV_LANE_RATE_DIV Error for invalid lane rate division
 * in deserializer.
 * @param TAL_ERR_DESER_INV_M_PARM Error for invalid M parameter in
 * deserializer.
 * @param TAL_ERR_DESER_INV_NP_PARM Error for invalid NP parameter in
 * deserializer.
 * @param TAL_ERR_DESER_INV_L_PARM Error for invalid L parameter in
 * deserializer.
 * @param TAL_ERR_DESER_INV_LANERATE_PARM Error for invalid lane rate parameter
 * in deserializer.
 * @param TAL_ERR_DESER_INV_LANEEN_PARM Error for invalid lane enable parameter
 * in deserializer.
 * @param TAL_ERR_DESER_INV_EQ_PARM Error for invalid equalization parameter in
 * deserializer.
 * @param TAL_ERR_DESER_INV_LANEPN_PARM Error for invalid lane PN parameter in
 * deserializer.
 * @param TAL_ERR_DESER_LANECONFLICT_PARM Error for lane conflict parameter in
 * deserializer.
 * @param TAL_ERR_DESER_M_CONFLICT Error indicating M conflict in deserializer.
 * @param TAL_ERR_DESER_INV_DEFAB_M Error for invalid DEFAB M parameter in
 * deserializer.
 * @param TAL_ERR_DESER_NP_CONFLICT Error indicating NP conflict in
 * deserializer.
 * @param TAL_ERR_DESER_INV_DEFRAMERSEL Error for invalid deframer selection in
 * deserializer.
 * @param TAL_ERR_DESER_TXPROFILE_INV Error for invalid TX profile in
 * deserializer.
 * @param TAL_ERR_FRAMER_INV_M_PARM Error for invalid M parameter in framer.
 * @param TAL_ERR_FRAMER_INV_NP_PARM Error for invalid NP parameter in framer.
 * @param TAL_ERR_FRAMER_INV_S_PARM Error for invalid S parameter in framer.
 * @param TAL_ERR_FRAMER_INV_BANKID_PARM Error for invalid bank ID parameter in
 * framer.
 * @param TAL_ERR_FRAMER_INV_LANEID_PARM Error for invalid lane ID parameter in
 * framer.
 * @param TAL_ERR_FRAMER_INV_SYNCBIN_PARM Error for invalid sync bin parameter
 * in framer.
 * @param TAL_ERR_FRAMER_INV_LMFC_OFFSET_PARAM Error for invalid LMFC offset
 * parameter in framer.
 * @param TAL_ERR_FRAMER_INV_DUALBAND_DATA_PARM Error for invalid dual-band data
 * parameter in framer.
 * @param TAL_ERR_DEFRAMER_INV_BANKID Error for invalid bank ID in deframer.
 * @param TAL_ERR_ERR_DEFRAMER_INV_LANEID Error for invalid lane ID in deframer.
 * @param TAL_ERR_DEFRAMER_INV_LMFC_OFFSET Error for invalid LMFC offset in
 * deframer.
 * @param TAL_ERR_DEFRAMER_INV_DEFSEL Error for invalid deframer selection in
 * deframer.
 * @param TAL_ERR_DEFRAMER_INV_TXPROFILE Error for invalid TX profile in
 * deframer.
 * @param TAL_ERR_DEFRAMER_INV_LANESEN Error for invalid lane enable in
 * deframer.
 * @param TAL_ERR_DEFRAMER_INV_L Error for invalid L parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_M Error for invalid M parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_NP Error for invalid NP parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_F Error for invalid F parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_K Error for invalid K parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_FK Error for invalid FK parameter in deframer.
 * @param TAL_ERR_DEFRAMER_INV_PCLK Error for invalid PCLK parameter in
 * deframer.
 * @param TAL_ERR_DEFRAMER_INV_PCLKDIV Error for invalid PCLK division in
 * deframer.
 * @param TAL_ERR_RSTDEFRAMER_INV_DEFSEL Error for invalid deframer selection in
 * reset deframer.
 * @param TAL_ERR_RSTFRAMER_INV_FRAMERSEL Error for invalid framer selection in
 * reset framer.
 * @param TAL_ERR_SETDFRMIRQMASK_INV_DEFRAMERSEL_PARAM Error for invalid
 * deframer selection
 * parameter in set deframer
 * IRQ mask.
 * @param TAL_ERR_DEFSTATUS_INV_DEFRAMERSEL_PARAM Error for invalid deframer
 * selection parameter in
 * deframer status.
 * @param TAL_ERR_DEFSTATUS_NULL_DEFRAMERSTATUS_PARAM Error for null deframer
 * status parameter in
 * deframer status.
 * @param TAL_ERR_GETDFRMIRQMASK_NULL_IRQMASK_PARAM Error for null IRQ mask
 * parameter in get deframer
 * IRQ mask.
 * @param TAL_ERR_GETDFRMIRQMASK_INV_DEFRAMERSELECT_PARAM Error for invalid
 * deframer selection
 * parameter in get
 * deframer IRQ mask.
 * @param TAL_ERR_CLRDFRMIRQ_INV_DEFRAMERSEL_PARAM Error for invalid deframer
 * selection parameter in clear
 * deframer IRQ.
 * @param TAL_ERR_GETDFRMIRQSRC_INV_DEFRAMERSEL_PARAM Error for invalid deframer
 * selection parameter in get
 * deframer IRQ source.
 * @param TAL_ERR_GETDFRMIRQSRC_NULL_STATUS_PARAM Error for null status
 * parameter in get deframer IRQ
 * source.
 * @param TAL_ERR_ENDEFSYSREF_INV_DEFRAMERSEL_PARAM Error for invalid deframer
 * selection parameter in
 * enable deframer SYSREF.
 * @param TAL_ERR_ENFRAMERSYSREF_INV_FRAMERSEL_PARAM Error for invalid framer
 * selection parameter in
 * enable framer SYSREF.
 * @param TAL_ERR_FRAMER_INV_TESTDATA_SOURCE_PARAM Error for invalid test data
 * source parameter in framer.
 * @param TAL_ERR_FRAMER_INV_INJECTPOINT_PARAM Error for invalid inject point
 * parameter in framer.
 * @param TAL_ERR_FRAMERSTATUS_INV_FRAMERSEL_PARAM Error for invalid framer
 * selection parameter in framer
 * status.
 * @param TAL_ERR_FRAMERSTATUS_NULL_FRAMERSTATUS_PARAM Error for null framer
 * status parameter in
 * framer status.
 * @param TAL_ERR_EN_DEFRAMER_PRBS_INV_PARAM Error for invalid parameter in
 * enable deframer PRBS.
 * @param TAL_ERR_READDFRMPRBS_INV_DEFSEL_PARAM Error for invalid deframer
 * selection parameter in read
 * deframer PRBS.
 * @param TAL_ERR_READDFRMPRBS_NULL_PARAM Error for null parameter in read
 * deframer PRBS.
 * @param TAL_ERR_INITARM_INV_ARMCLK_PARAM Error for invalid ARM clock parameter
 * in initialize ARM.
 * @param TAL_ERR_LOADHEX_INVALID_CHKSUM Error for invalid checksum in load hex.
 * @param TAL_ERR_LOADBIN_INVALID_BYTECOUNT Error for invalid byte count in load
 * binary.
 * @param TAL_ERR_READARMMEM_INV_ADDR_PARM Error for invalid address parameter
 * in read ARM memory.
 * @param TAL_ERR_WRITEARMMEM_INV_ADDR_PARM Error for invalid address parameter
 * in write ARM memory.
 * @param TAL_ERR_ARMCMD_INV_OPCODE_PARM Error for invalid opcode parameter in
 * ARM command.
 * @param TAL_ERR_ARMCMD_INV_NUMBYTES_PARM Error for invalid number of bytes
 * parameter in ARM command.
 * @param TAL_ERR_ARMCMDSTATUS_INV_OPCODE_PARM Error for invalid opcode
 * parameter in ARM command status.
 * @param TAL_ERR_JESD204B_ILAS_MISMATCH_NULLPARAM Error for null parameter in
 * JESD204B ILAS mismatch.
 * @param TAL_ERR_JESD204B_ILAS_MISMATCH_NO_ACTIVE_LINK Error for no active link
 * in JESD204B ILAS
 * mismatch.
 * @param TAL_ERR_JESD204B_ILAS_MISMATCH_SYNC_NOT_DETECTED Error for sync not
 * detected in JESD204B
 * ILAS mismatch.
 * @param TAL_ERR_JESD204B_ILAS_MISMATCH_INVALID_DEFRAMER Error for invalid
 * deframer in JESD204B
 * ILAS mismatch.
 * @param TAL_ERR_TALFINDDFRMRLANECNTERROR_INV_DEFRAMERSEL_PARAM Error for
 * invalid
 * deframer
 * selection
 * parameter in
 * find deframer
 * lane count
 * error.
 * @param TAL_ERR_TALFINDDFRMRLANECNTERROR_NULL_PARAM Error for null parameter
 * in find deframer lane
 * count error.
 * @param TAL_ERR_TALFINDDFRMRLANEERROR_NULL_PARAM Error for null parameter in
 * find deframer lane error.
 * @param TAL_ERR_RXGAINTABLE_INV_CHANNEL Error for invalid channel in RX gain
 * table.
 * @param TAL_ERR_RXGAINTABLE_INV_PROFILE Error for invalid profile in RX gain
 * table.
 * @param TAL_ERR_RXGAINTABLE_INV_GAIN_INDEX_RANGE Error for invalid gain index
 * range in RX gain table.
 * @param TAL_ERR_ORXGAINTABLE_INV_CHANNEL Error for invalid channel in ORX gain
 * table.
 * @param TAL_ERR_ORXGAINTABLE_INV_PROFILE Error for invalid profile in ORX gain
 * table.
 * @param TAL_ERR_ORXGAINTABLE_INV_GAIN_INDEX_RANGE Error for invalid gain index
 * range in ORX gain table.
 * @param TAL_ERR_RXFRAMER_INV_FK_PARAM Error for invalid FK parameter in RX
 * framer.
 * @param TAL_ERR_RXFRAMER_INV_L_PARAM Error for invalid L parameter in RX
 * framer.
 * @param TAL_ERR_INV_RX_GAIN_MODE_PARM Error for invalid RX gain mode
 * parameter.
 * @param TAL_ERR_UNSUPPORTED_RX_GAIN_MODE_PARM Error for unsupported RX gain
 * mode parameter.
 * @param TAL_ERR_INV_AGC_RX_STRUCT_INIT Error for invalid AGC RX structure
 * initialization.
 * @param TAL_ERR_INV_AGC_PWR_STRUCT_INIT Error for invalid AGC power structure
 * initialization.
 * @param TAL_ERR_INV_AGC_PWR_LWR0THRSH_PARAM Error for invalid lower 0
 * threshold parameter in AGC power.
 * @param TAL_ERR_INV_AGC_PWR_LWR1THRSH_PARAM Error for invalid lower 1
 * threshold parameter in AGC power.
 * @param TAL_ERR_INV_AGC_PWR_LWR0PWRGAINSTEP_PARAM Error for invalid lower 0
 * power gain step parameter in
 * AGC power.
 * @param TAL_ERR_INV_AGC_PWR_LWR1PWRGAINSTEP_PARAM Error for invalid lower 1
 * power gain step parameter in
 * AGC power.
 * @param TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM Error for invalid measurement
 * duration parameter in AGC
 * power.
 * @param TAL_ERR_INV_AGC_PWR_IP3RANGE_PARAM Error for invalid IP3 range
 * parameter in AGC power.
 * @param TAL_ERR_INV_AGC_PWR_UPPWR0THRSH_PARAM Error for invalid upper 0
 * threshold parameter in AGC
 * power.
 * @param TAL_ERR_INV_AGC_PWR_UPPWR1THRSH_PARAM Error for invalid upper 1
 * threshold parameter in AGC
 * power.
 * @param TAL_ERR_INV_AGC_PWR_LOGSHIFT_PARAM Error for invalid log shift
 * parameter in AGC power.
 * @param TAL_ERR_INV_AGC_PWR_UPPWR0GAINSTEP_PARAM Error for invalid upper 0
 * gain step parameter in AGC
 * power.
 * @param TAL_ERR_INV_AGC_PWR_UPPWR1GAINSTEP_PARAM Error for invalid upper 1
 * gain step parameter in AGC
 * power.
 * @param TAL_ERR_INV_AGC_PKK_STRUCT_INIT Error for invalid AGC PKK structure
 * initialization.
 * @param TAL_ERR_INV_AGC_PKK_HIGHTHRSH_PARAM Error for invalid high threshold
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_LOWGAINMODEHIGHTHRSH_PARAM Error for invalid low
 * gain mode high
 * threshold parameter in
 * AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_LOWGAINHIGHTHRSH_PARAM Error for invalid low gain
 * high threshold parameter in
 * AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_LOWGAINTHRSH_PARAM Error for invalid low gain
 * threshold parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_GAINSTEPATTACK_PARAM Error for invalid gain step
 * attack parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_GAINSTEPRECOVERY_PARAM Error for invalid gain step
 * recovery parameter in AGC
 * PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2OVRLD_PARAM Error for invalid HB2 overload
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2OVRLDDURATION_PARAM Error for invalid HB2
 * overload duration parameter
 * in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2OVRLDTHRSHCNT_PARAM Error for invalid HB2
 * overload threshold count
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2GAINSTEPRECOVERY_PARAM Error for invalid HB2
 * gain step recovery
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2GAINSTEP0RECOVERY_PARAM Error for invalid HB2
 * gain step 0 recovery
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2GAINSTEP1RECOVERY_PARAM Error for invalid HB2
 * gain step 1 recovery
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2GAINSTEPATTACK_PARAM Error for invalid HB2 gain
 * step attack parameter in
 * AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2OVRLDPWRMODE_PARAM Error for invalid HB2
 * overload power mode
 * parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2OVRLDSEL_PARAM Error for invalid HB2 overload
 * selection parameter in AGC PKK.
 * @param TAL_ERR_INV_AGC_PKK_HB2THRSHCFG_PARAM Error for invalid HB2 threshold
 * configuration parameter in AGC
 * PKK.
 * @param TAL_ERR_INV_AGC_RX_APD_HIGH_LOW_THRESH Error for invalid APD high low
 * threshold in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_PEAK_WAIT_TIME_PARM Error for invalid peak wait
 * time parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_MIN_MAX_GAIN_PARM Error for invalid min max gain
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_MIN_GAIN_GRT_THAN_MAX_GAIN_PARM Error for min gain
 * greater than max
 * gain parameter in
 * AGC RX.
 * @param TAL_ERR_INV_AGC_RX_GAIN_UPDATE_TIME_PARM Error for invalid gain update
 * time parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX1ATTACKDELAY_PARAM Error for invalid RX1 attack
 * delay parameter in AGC.
 * @param TAL_ERR_INV_AGC_RX2ATTACKDELAY_PARAM Error for invalid RX2 attack
 * delay parameter in AGC.
 * @param TAL_ERR_INV_AGC_RX_LOWTHRSHPREVENTGAIN_PARM Error for invalid low
 * threshold prevent gain
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_CHANGEGAINTHRSHHIGH_PARM Error for invalid change
 * gain threshold high
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_RESETONRXON_PARM Error for invalid reset on RX on
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_ENSYNCHPULSECAINCNTR_PARM Error for invalid enable
 * sync pulse gain counter
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_ENIP3OPTTHRSH_PARM Error for invalid enable IP3
 * optimization threshold parameter
 * in AGC RX.
 * @param TAL_ERR_INV_AGC_RX_ENFASTRECOVERYLOOP_PARM Error for invalid enable
 * fast recovery loop
 * parameter in AGC RX.
 * @param TAL_ERR_INV_AGC_SLOWLOOPDELAY_PARAM Error for invalid slow loop delay
 * parameter in AGC.
 * @param TAL_ERR_INV_MINAGCSLOWLOOPDELAY_PARAM Error for invalid minimum AGC
 * slow loop delay parameter.
 * @param TAL_ERR_INV_AGC_CLK_DIV_RATIO_PARM Error for invalid AGC clock
 * division ratio parameter.
 * @param TAL_ERR_INV_AGC_CLK_PARM Error for invalid AGC clock parameter.
 * @param TAL_ERR_INV_AGC_CLK_RATIO Error for invalid AGC clock ratio.
 * @param TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_UPDATE_TIME_PARM Error for invalid
 * RX gain underrange
 * update time
 * parameter in AGC.
 * @param TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_MID_INTERVAL_PARM Error for invalid
 * RX gain
 * underrange mid
 * interval
 * parameter in AGC.
 * @param TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_HIGH_INTERVAL_PARM Error for
 * invalid RX gain
 * underrange high
 * interval
 * parameter in
 * AGC.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_CLKPLLCP Error indicating wait for event
 * timed out for clock PLL charge
 * pump.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_CLKPLL_LOCK Error indicating wait for
 * event timed out for clock
 * PLL lock.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_RFPLLCP Error indicating wait for event
 * timed out for RF PLL charge
 * pump.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_RFPLL_LOCK Error indicating wait for
 * event timed out for RF PLL
 * lock.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_AUXPLLCP Error indicating wait for event
 * timed out for auxiliary PLL
 * charge pump.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_AUXPLL_LOCK Error indicating wait for
 * event timed out for
 * auxiliary PLL lock.
 * @param TAL_ERR_WAITFOREVENT_TIMEDOUT_ARMBUSY Error indicating wait for event
 * timed out for ARM busy.
 * @param TAL_ERR_TIMEDOUT_ARMMAILBOXBUSY Error indicating timeout for ARM
 * mailbox busy.
 * @param TAL_ERR_EN_TRACKING_CALS_ARMSTATE_ERROR Error for ARM state error in
 * enable tracking calibrations.
 * @param TAL_ERR_GETPENDINGTRACKINGCALS_NULL_PARAM Error for null parameter in
 * get pending tracking
 * calibrations.
 * @param TAL_ERR_TRACKINGCAL_OUTOFRANGE_PARAM Error for out of range parameter
 * in tracking calibration.
 * @param TAL_ERR_PAUSETRACKINGCAL_INV_PARAM Error for invalid parameter in
 * pause tracking calibration.
 * @param TAL_ERR_GETPAUSECALSTATE_NULL_PARAM Error for null parameter in get
 * pause calibration state.
 * @param TAL_ERR_SET_ARMGPIO_PINS_GPIO_IN_USE Error for GPIO in use in set ARM
 * GPIO pins.
 * @param TAL_ERR_SET_ARMGPIO_PINS_INV_SIGNALID Error for invalid signal ID in
 * set ARM GPIO pins.
 * @param TAL_ERR_SET_ARMGPIO_PINS_INV_GPIOPIN Error for invalid GPIO pin in set
 * ARM GPIO pins.
 * @param TAL_ERR_SET_RXDATAFRMT_NULL_PARAM Error for null parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_FORMATSELECT_INVPARAM Error for invalid format
 * select parameter in set
 * RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_TEMPCOMP_INVPARAM Error for invalid temperature
 * compensation parameter in set
 * RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_ROUNDMODE_INVPARAM Error for invalid round mode
 * parameter in set RX data
 * format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPDATAFRMT_INVPARAM Error for invalid FP data
 * format parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPENCNAN_INVPARAM Error for invalid FP encode
 * NaN parameter in set RX data
 * format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPEXPBITS_INVPARAM Error for invalid FP
 * exponent bits parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPHIDELEADINGONE_INVPARAM Error for invalid FP
 * hide leading one
 * parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPRX1ATTEN_INVPARAM Error for invalid FP RX1
 * attenuation parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_FPRX2ATTEN_INVPARAM Error for invalid FP RX2
 * attenuation parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_INTEMBEDDEDBITS_INVPARAM Error for invalid
 * integer embedded bits
 * parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_INTSAMPLERESOLUTION_INVPARAM Error for invalid
 * integer sample
 * resolution
 * parameter in set
 * RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_PINSTEPSIZE_INVPARAM Error for invalid pin step
 * size parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_RX1GPIOSELECT_INVPARAM Error for invalid RX1
 * GPIO select parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_RX2GPIOSELECT_INVPARAM Error for invalid RX2
 * GPIO select parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_RXCHAN_DISABLED Error indicating RX channel is
 * disabled in set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_EXTERNALLNAGAIN_INVPARAM Error for invalid
 * external LNA gain
 * parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_RX1GPIO_INUSE Error indicating RX1 GPIO is in
 * use in set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_RX2GPIO_INUSE Error indicating RX2 GPIO is in
 * use in set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_DATARES_INVPARAM Error for invalid data
 * resolution parameter in set RX
 * data format.
 * @param TAL_ERR_SET_RXDATAFRMT_EXTSLICER_RX1GPIO_INVPARAM Error for invalid
 * external slicer RX1
 * GPIO parameter in
 * set RX data format.
 * @param TAL_ERR_SET_RXDATAFRMT_EXTSLICER_RX2GPIO_INVPARAM Error for invalid
 * external slicer RX2
 * GPIO parameter in
 * set RX data format.
 * @param TAL_ERR_GET_DATAFORMAT_NULL_PARAM Error for null parameter in get data
 * format.
 * @param TAL_ERR_GET_SLICERPOS_NULL_PARAM Error for null parameter in get
 * slicer position.
 * @param TAL_ERR_INV_RX_DEC_POWER_PARAM Error for invalid RX decimation power
 * parameter.
 * @param TAL_ERR_GETRXDECPOWER_INV_CHANNEL Error for invalid channel in get RX
 * decimation power.
 * @param TAL_ERR_GETRXDECPOWER_INV_PROFILE Error for invalid profile in get RX
 * decimation power.
 * @param TAL_ERR_INIT_NULLPARAM Error for null parameter in initialization.
 * @param TAL_ERR_INIT_INV_DEVCLK Error for invalid device clock in
 * initialization.
 * @param TAL_ERR_GETRADIOSTATE_NULL_PARAM Error for null parameter in get radio
 * state.
 * @param TAL_ERR_CHECKGETMCS_STATUS_NULL_PARM Error for null parameter in check
 * get MCS status.
 * @param TAL_ERR_WAIT_INITCALS_ARMERROR Error for ARM error in wait initial
 * calibrations.
 * @param TAL_ERR_WAIT_INITCALS_NULL_PARAM Error for null parameter in wait
 * initial calibrations.
 * @param TAL_ERR_CHECK_PLL_LOCK_NULL_PARM Error for null parameter in check PLL
 * lock.
 * @param TAL_ERR_READGPIOSPI_NULL_PARM Error for null parameter in read GPIO
 * SPI.
 * @param TAL_ERR_READGPIO3V3SPI_NULL_PARM Error for null parameter in read GPIO
 * 3.3V SPI.
 * @param TAL_ERR_GET_TXFILTEROVRG_NULL_PARM Error for null parameter in get TX
 * filter overrange.
 * @param TAL_ERR_PROGRAM_RXGAIN_TABLE_NULL_PARM Error for null parameter in
 * program RX gain table.
 * @param TAL_ERR_PROGRAM_ORXGAIN_TABLE_NULL_PARM Error for null parameter in
 * program ORX gain table.
 * @param TAL_ERR_PROGRAMFIR_NULL_PARM Error for null parameter in program FIR.
 * @param TAL_ERR_PROGRAMFIR_COEFS_NULL Error for null coefficients in program
 * FIR.
 * @param TAL_ERR_READ_DEFRAMERSTATUS_NULL_PARAM Error for null parameter in
 * read deframer status.
 * @param TAL_ERR_READ_DEFRAMERPRBS_NULL_PARAM Error for null parameter in read
 * deframer PRBS.
 * @param TAL_ERR_ARMCMDSTATUS_NULL_PARM Error for null parameter in ARM command
 * status.
 * @param TAL_ERR_PROGRAMFIR_INV_FIRNAME_PARM Error for invalid FIR name
 * parameter in program FIR.
 * @param TAL_ERR_RXFIR_INV_GAIN_PARM Error for invalid gain parameter in RX
 * FIR.
 * @param TAL_ERR_PROGRAMFIR_INV_NUMTAPS_PARM Error for invalid number of taps
 * parameter in program FIR.
 * @param TALISE_ERR_TXFIR_INV_NUMTAPS_PARM Error for invalid number of taps
 * parameter in TX FIR.
 * @param TALISE_ERR_TXFIR_INV_NUMROWS Error for invalid number of rows in TX
 * FIR.
 * @param TALISE_ERR_TXFIR_TAPSEXCEEDED Error indicating taps exceeded in TX
 * FIR.
 * @param TALISE_ERR_RXFIR_INV_DDC Error for invalid DDC in RX FIR.
 * @param TALISE_ERR_RXFIR_INV_NUMTAPS_PARM Error for invalid number of taps
 * parameter in RX FIR.
 * @param TALISE_ERR_RXFIR_INV_NUMROWS Error for invalid number of rows in RX
 * FIR.
 * @param TALISE_ERR_RXFIR_TAPSEXCEEDED Error indicating taps exceeded in RX
 * FIR.
 * @param TALISE_ERR_ORXFIR_INV_DDC Error for invalid DDC in ORX FIR.
 * @param TALISE_ERR_ORXFIR_INV_NUMTAPS_PARM Error for invalid number of taps
 * parameter in ORX FIR.
 * @param TALISE_ERR_ORXFIR_INV_NUMROWS Error for invalid number of rows in ORX
 * FIR.
 * @param TALISE_ERR_ORXFIR_TAPSEXCEEDED Error indicating taps exceeded in ORX
 * FIR.
 * @param TAL_ERR_WAITARMCMDSTATUS_INV_OPCODE Error for invalid opcode in wait
 * ARM command status.
 * @param TAL_ERR_WAITARMCMDSTATUS_TIMEOUT Error indicating timeout in wait ARM
 * command status.
 * @param TAL_ERR_READARMCMDSTATUS_NULL_PARM Error for null parameter in read
 * ARM command status.
 * @param TAL_ERR_LOADBIN_NULL_PARAM Error for null parameter in load binary.
 * @param TAL_ERR_GETARMVER_NULL_PARM Error for null parameter in get ARM
 * version.
 * @param TAL_ERR_GETARMVER_V2_NULL_PARM Error for null parameter in get ARM
 * version V2.
 * @param TAL_ERR_GETARMVER_V2_INVALID_ARM_NOT_LOADED Error indicating invalid
 * ARM not loaded in get ARM
 * version V2.
 * @param TAL_ERR_READEVENTSTATUS_INV_PARM Error for invalid parameter in read
 * event status.
 * @param TAL_ERR_FRAMER_INV_FRAMERSEL_PARAM Error for invalid framer selection
 * parameter in framer.
 * @param TAL_ERR_FRAMER_ERRINJECT_INV_FRAMERSEL_PARAM Error for invalid framer
 * selection parameter in
 * framer error inject.
 * @param TAL_ERR_CHECKPLLLOCK_NULL_PARM Error for null parameter in check PLL
 * lock.
 * @param TAL_ERR_FRAMER_INV_FRAMERSEL_PARM Error for invalid framer selection
 * parameter in framer.
 * @param TAL_ERR_SETTXATTEN_INV_TXCHANNEL Error for invalid TX channel in set
 * TX attenuation.
 * @param TAL_ERR_SETTXATTEN_INV_PARM Error for invalid parameter in set TX
 * attenuation.
 * @param TAL_ERR_RXFRAMER_INV_OUTPUT_RATE Error for invalid output rate in RX
 * framer.
 * @param TAL_ERR_RXFRAMER_INV_PCLKFREQ Error for invalid PCLK frequency in RX
 * framer.
 * @param TAL_ERR_BBIC_INV_CHN Error for invalid channel in BBIC.
 * @param TAL_ERR_INV_GP_INT_MASK_PARM Error for invalid GP interrupt mask
 * parameter.
 * @param TAL_ERR_INV_GP_INT_MASK_NULL_PARM Error for null GP interrupt mask
 * parameter.
 * @param TAL_ERR_GP_INT_STATUS_NULL_PARAM Error for null parameter in GP
 * interrupt status.
 * @param TAL_ERR_INV_DAC_SAMP_XBAR_CHANNEL_SEL Error for invalid DAC sample
 * crossbar channel selection.
 * @param TAL_ERR_INV_ADC_SAMP_XBAR_FRAMER_SEL Error for invalid ADC sample
 * crossbar framer selection.
 * @param TAL_ERR_INV_ADC_SAMP_XBAR_SELECT_PARAM Error for invalid ADC sample
 * crossbar selection parameter.
 * @param TAL_ERR_INV_DAC_SAMP_XBAR_SELECT_PARAM Error for invalid DAC sample
 * crossbar selection parameter.
 * @param TAL_ERR_GETRFPLL_INV_PLLNAME Error for invalid PLL name in get RF PLL.
 * @param TAL_ERR_GET_PLLFREQ_INV_REFCLKDIV Error for invalid reference clock
 * division in get PLL frequency.
 * @param TAL_ERR_GET_PLLFREQ_INV_HSDIV Error for invalid high-speed division in
 * get PLL frequency.
 * @param TAL_ERR_GETRFPLL_NULLPARAM Error for null parameter in get RF PLL.
 * @param TAL_ERR_FRAMER_A_AND_B_INV_M_PARM Error for invalid M parameter in
 * framer A and B.
 * @param TAL_ERR_SETRXGAIN_RXPROFILE_INVALID Error for invalid RX profile in
 * set RX gain.
 * @param TAL_ERR_SETRXGAIN_INV_CHANNEL Error for invalid channel in set RX
 * gain.
 * @param TAL_ERR_INIT_CALS_COMPLETED_NULL_PTR Error for null pointer in initial
 * calibrations completed.
 * @param TAL_ERR_CHKINITCALS_NULL_PTR Error for null pointer in check initial
 * calibrations.
 * @param TAL_ERR_INIT_CALS_LASTRUN_NULL_PTR Error for null pointer in initial
 * calibrations last run.
 * @param TAL_ERR_GETENABLED_TRACK_CALS_NULL_PTR Error for null pointer in get
 * enabled tracking calibrations.
 * @param TAL_ERR_INIT_CALS_MIN_NULL_PTR Error for null pointer in initial
 * calibrations minimum.
 * @param TAL_ERR_INIT_ERR_CAL_NULL_PTR Error for null pointer in initial error
 * calibration.
 * @param TAL_ERR_INIT_ERR_CODE_NULL_PTR Error for null pointer in initial error
 * code.
 * @param TAL_ERR_READARMCFG_ARMERRFLAG Error for ARM error flag in read ARM
 * configuration.
 * @param TAL_ERR_GETRXGAIN_INV_RXPROFILE Error for invalid RX profile in get RX
 * gain.
 * @param TAL_ERR_GETRXGAIN_INV_CHANNEL Error for invalid channel in get RX
 * gain.
 * @param TAL_ERR_GETRXGAIN_GAIN_RANGE_EXCEEDED Error indicating gain range
 * exceeded in get RX gain.
 * @param TAL_ERR_GETOBSRXGAIN_INV_ORXPROFILE Error for invalid ORX profile in
 * get observation RX gain.
 * @param TAL_ERR_GETOBSRXGAIN_INV_CHANNEL Error for invalid channel in get
 * observation RX gain.
 * @param TAL_ERR_GETOBSRXGAIN_GAIN_RANGE_EXCEEDED Error indicating gain range
 * exceeded in get observation
 * RX gain.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_GAIN_RANGE_MISMATCH Error indicating gain
 * range mismatch in setup
 * dual-band RX AGC.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_GAIN_OUT_OF_RANGE Error indicating gain out
 * of range in setup dual-
 * band RX AGC.
 * @param TAL_ERR_VERIFYBIN_CHECKSUM_TIMEOUT Error indicating checksum timeout
 * in verify binary.
 * @param TAL_ERR_ARMCMDSTATUS_ARMERROR Error indicating ARM error in ARM
 * command status.
 * @param TAL_ERR_VERRXPFILE_INV_IQRATE Error for invalid IQ rate in verify RX
 * profile file.
 * @param TAL_ERR_VERRXPFILE_INV_RFBW Error for invalid RF bandwidth in verify
 * RX profile file.
 * @param TAL_ERR_VERRXPFILE_INV_RHB1 Error for invalid RHB1 in verify RX
 * profile file.
 * @param TAL_ERR_VERRXPFILE_INV_DEC5 Error for invalid DEC5 in verify RX
 * profile file.
 * @param TAL_ERR_VERRXPFILE_INV_FIR Error for invalid FIR in verify RX profile
 * file.
 * @param TAL_ERR_VERRXPFILE_INV_COEF Error for invalid coefficient in verify RX
 * profile file.
 * @param TAL_ERR_VERRXPFILE_INV_DDC Error for invalid DDC in verify RX profile
 * file.
 * @param TAL_ERR_VERORXPFILE_INV_IQRATE Error for invalid IQ rate in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_RFBW Error for invalid RF bandwidth in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_RHB1 Error for invalid RHB1 in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_DEC5 Error for invalid DEC5 in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_FIR Error for invalid FIR in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_COEF Error for invalid coefficient in verify
 * observation RX profile file.
 * @param TAL_ERR_VERORXPFILE_INV_DDC Error for invalid DDC in verify
 * observation RX profile file.
 * @param TAL_ERR_VERTXPFILE_INV_DISONPLLUNLOCK Error for invalid DIS on PLL
 * unlock in verify TX profile
 * file.
 * @param TAL_ERR_VERTXPFILE_INV_IQRATE Error for invalid IQ rate in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_RFBW Error for invalid RF bandwidth in verify
 * TX profile file.
 * @param TAL_ERR_VERTXPFILE_INV_THB1 Error for invalid THB1 in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_THB2 Error for invalid THB2 in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_THB3 Error for invalid THB3 in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_INT5 Error for invalid INT5 in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_HBMUTEX Error for invalid HB mutex in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_FIRIPL Error for invalid FIR IPL in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_COEF Error for invalid coefficient in verify TX
 * profile file.
 * @param TAL_ERR_VERTXPFILE_INV_DACDIV Error for invalid DAC division in verify
 * TX profile file.
 * @param TAL_ERR_VERPFILE_INV_RFPLLMCSMODE Error for invalid RF PLL MCS mode in
 * verify profile file.
 * @param TAL_ERR_VERPFILE_TXHSCLK Error for TX high-speed clock in verify
 * profile file.
 * @param TAL_ERR_VERPFILE_RXHSCLK Error for RX high-speed clock in verify
 * profile file.
 * @param TAL_ERR_VERPFILE_ORXHSCLK Error for observation RX high-speed clock in
 * verify profile file.
 * @param TAL_ERR_SETSPI_INV_CMOS_DRV_STR Error for invalid CMOS drive strength
 * in set SPI.
 * @param TAL_ERR_SETCLKPLL_INV_TXATTENDIV Error for invalid TX attenuation
 * division in set clock PLL.
 * @param TAL_ERR_SETTXTOORXMAP_INV_ORX1_MAP Error for invalid ORX1 map in set
 * TX to ORX map.
 * @param TAL_ERR_SETTXTOORXMAP_INV_ORX2_MAP Error for invalid ORX2 map in set
 * TX to ORX map.
 * @param TAL_ERR_GETRXTXENABLE_NULLPARAM Error for null parameter in get RX TX
 * enable.
 * @param TAL_ERR_SETRXTXENABLE_INVCHANNEL Error for invalid channel in set RX
 * TX enable.
 * @param TAL_ERR_GETTXATTEN_NULL_PARM Error for null parameter in get TX
 * attenuation.
 * @param TAL_ERR_GETTXATTEN_INV_TXCHANNEL Error for invalid TX channel in get
 * TX attenuation.
 * @param TAL_ERR_INV_RADIO_CTL_MASK_PARM Error for invalid radio control mask
 * parameter.
 * @param TAL_ERR_GETPINMODE_NULLPARAM Error for null parameter in get pin mode.
 * @param TAL_ERR_SET_ARMGPIO_NULLPARAM Error for null parameter in set ARM
 * GPIO.
 * @param TAL_ERR_GETTEMPERATURE_NULLPARAM Error for null parameter in get
 * temperature.
 * @param TAL_ERR_GETTXLOLSTATUS_NULL_PARAM Error for null parameter in get TX
 * LOL status.
 * @param TAL_ERR_GETTXLOLSTATUS_INV_CHANNEL_PARM Error for invalid channel
 * parameter in get TX LOL
 * status.
 * @param TAL_ERR_GETTXQECSTATUS_NULL_PARAM Error for null parameter in get TX
 * QEC status.
 * @param TAL_ERR_GETTXQECSTATUS_INV_CHANNEL_PARM Error for invalid channel
 * parameter in get TX QEC
 * status.
 * @param TAL_ERR_GETRXQECSTATUS_NULL_PARAM Error for null parameter in get RX
 * QEC status.
 * @param TAL_ERR_GETRXQECSTATUS_INV_CHANNEL_PARM Error for invalid channel
 * parameter in get RX QEC
 * status.
 * @param TAL_ERR_GETORXQECSTATUS_NULL_PARAM Error for null parameter in get
 * observation RX QEC status.
 * @param TAL_ERR_GETORXQECSTATUS_INV_CHANNEL_PARM Error for invalid channel
 * parameter in get observation
 * RX QEC status.
 * @param TAL_ERR_GETRXHD2STATUS_NULL_PARAM Error for null parameter in get RX
 * HD2 status.
 * @param TAL_ERR_GETRXHD2STATUS_INV_CHANNEL_PARM Error for invalid channel
 * parameter in get RX HD2
 * status.
 * @param TAL_ERR_VERIFYSPI_READ_LOW_ADDR_ERROR Error for read low address error
 * in verify SPI.
 * @param TAL_ERR_VERIFYSPI_WRITE_LOW_ADDR_ERROR Error for write low address
 * error in verify SPI.
 * @param TAL_ERR_VERIFYSPI_READ_HIGH_ADDR_ERROR Error for read high address
 * error in verify SPI.
 * @param TAL_ERR_VERIFYSPI_WRITE_HIGH_ADDR_ERROR Error for write high address
 * error in verify SPI.
 * @param TAL_ERR_GETAPIVERSION_NULLPARAM Error for null parameter in get API
 * version.
 * @param TAL_ERR_INV_DAC_FULLSCALE_PARM Error for invalid DAC full-scale
 * parameter.
 * @param TAL_ERR_RESET_TXLOL_INV_CHANNEL_PARM Error for invalid channel
 * parameter in reset TX LOL.
 * @param TAL_ERR_RESET_TXLOL_ARMSTATE_ERROR Error for ARM state error in reset
 * TX LOL.
 * @param TAL_ERR_SETHD2CFG_NULL_PARAM Error for null parameter in set HD2
 * configuration.
 * @param TAL_ERR_SETHD2CFG_ARMSTATE_ERROR Error for ARM state error in set HD2
 * configuration.
 * @param TAL_ERR_GETHD2CFG_NULL_PARAM Error for null parameter in get HD2
 * configuration.
 * @param TAL_ERR_GETHD2CFG_ARMSTATE_ERROR Error for ARM state error in get HD2
 * configuration.
 * @param TAL_ERR_SET_SPI2_ENABLE_INVALID_TX_ATTEN_SEL Error for invalid TX
 * attenuation selection in
 * set SPI2 enable.
 * @param TAL_ERR_SET_SPI2_ENABLE_GPIO_IN_USE Error for GPIO in use in set SPI2
 * enable.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_RX1_INC_PIN Error for invalid RX1
 * increment pin in set RX MGC
 * pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_RX1_DEC_PIN Error for invalid RX1
 * decrement pin in set RX MGC
 * pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_RX2_INC_PIN Error for invalid RX2
 * increment pin in set RX MGC
 * pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_RX2_DEC_PIN Error for invalid RX2
 * decrement pin in set RX MGC
 * pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_CHANNEL Error for invalid channel in set
 * RX MGC pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_INC_STEP Error for invalid increment step
 * in set RX MGC pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_INV_DEC_STEP Error for invalid decrement step
 * in set RX MGC pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_RX1_GPIO_IN_USE Error for RX1 GPIO in use in
 * set RX MGC pin control.
 * @param TAL_ERR_SETRXMGCPINCTRL_RX2_GPIO_IN_USE Error for RX2 GPIO in use in
 * set RX MGC pin control.
 * @param TAL_ERR_GETRXMGCPINCTRL_INV_CHANNEL Error for invalid channel in get
 * RX MGC pin control.
 * @param TAL_ERR_GETRXMGCPINCTRL_NULL_PARAM Error for null parameter in get RX
 * MGC pin control.
 * @param TAL_ERR_SETRFPLL_LOOPFILTER_INV_LOOPBANDWIDTH Error for invalid loop
 * bandwidth in set RF PLL
 * loop filter.
 * @param TAL_ERR_SETRFPLL_LOOPFILTER_INV_STABILITY Error for invalid stability
 * in set RF PLL loop filter.
 * @param TAL_ERR_SETRFPLL_LOOPFILTER_INV_PLLSEL Error for invalid PLL selection
 * in set RF PLL loop filter.
 * @param TAL_ERR_GETRFPLL_LOOPFILTER_NULLPARAM Error for null parameter in get
 * RF PLL loop filter.
 * @param TAL_ERR_GETRFPLL_LOOPFILTER_INV_PLLSEL Error for invalid PLL selection
 * in get RF PLL loop filter.
 * @param TAL_ERR_GETDEVICEREV_NULLPARAM Error for null parameter in get device
 * revision.
 * @param TAL_ERR_GETPRODUCTID_NULLPARAM Error for null parameter in get product
 * ID.
 * @param TAL_ERR_ENABLETXNCO_INV_PROFILE Error for invalid profile in enable TX
 * NCO.
 * @param TAL_ERR_ENABLETXNCO_NULL_PARM Error for null parameter in enable TX
 * NCO.
 * @param TAL_ERR_ENABLETXNCO_INV_TX1_FREQ Error for invalid TX1 frequency in
 * enable TX NCO.
 * @param TAL_ERR_ENABLETXNCO_INV_TX2_FREQ Error for invalid TX2 frequency in
 * enable TX NCO.
 * @param TAL_ERR_SETTXATTENCTRLPIN_NULL_PARAM Error for null parameter in set
 * TX attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_TX1_GPIO_IN_USE Error for TX1 GPIO in use in
 * set TX attenuation control
 * pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_TX2_GPIO_IN_USE Error for TX2 GPIO in use in
 * set TX attenuation control
 * pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_PARM Error for invalid parameter in set
 * TX attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_TX1_INC_PIN Error for invalid TX1
 * increment pin in set TX
 * attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_TX1_DEC_PIN Error for invalid TX1
 * decrement pin in set TX
 * attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_TX2_INC_PIN Error for invalid TX2
 * increment pin in set TX
 * attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_TX2_DEC_PIN Error for invalid TX2
 * decrement pin in set TX
 * attenuation control pin.
 * @param TAL_ERR_SETTXATTENCTRLPIN_INV_CHANNEL Error for invalid channel in set
 * TX attenuation control pin.
 * @param TAL_ERR_GETTXATTENCTRLPIN_INV_CHANNEL Error for invalid channel in get
 * TX attenuation control pin.
 * @param TAL_ERR_GETTXATTENCTRLPIN_NULL_PARAM Error for null parameter in get
 * TX attenuation control pin.
 * @param TAL_ERR_SETUPDUALBANDRX1AGC_GPIO3P3_IN_USE Error for GPIO 3.3V in use
 * in setup dual-band RX1 AGC.
 * @param TAL_ERR_SETUPDUALBANDRX2AGC_GPIO3P3_IN_USE Error for GPIO 3.3V in use
 * in setup dual-band RX2 AGC.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_INV_CHANNEL Error for invalid channel in
 * setup dual-band RX AGC.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_NULL_PARAM Error for null parameter in
 * setup dual-band RX AGC.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_INV_PWRMARGIN Error for invalid power
 * margin in setup dual-band RX
 * AGC.
 * @param TAL_ERR_SETUPDUALBANDRXAGC_INV_DECPWR Error for invalid decimation
 * power in setup dual-band RX AGC.
 * @param TAL_ERR_GETDUALBANDLNA_INV_CHANNEL Error for invalid channel in get
 * dual-band LNA.
 * @param TAL_ERR_GETDUALBANDLNA_NULL_PARAM Error for null parameter in get
 * dual-band LNA.
 * @param TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_2OR4 Error for invalid total M 2 or
 * 4 in initialize DDC.
 * @param TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_4OR8 Error for invalid total M 4 or
 * 8 in initialize DDC.
 * @param TAL_ERR_INITIALIZE_DDC_NULL_PARAM Error for null parameter in
 * initialize DDC.
 * @param TAL_ERR_RXNCOFTW_INVNCO Error for invalid NCO in RX NCO FTW.
 * @param TAL_ERR_INITIALIZE_DDC_INV_RXDDCMODE Error for invalid RX DDC mode in
 * initialize DDC.
 * @param TAL_ERR_INITIALIZE_DDC_INV_DEC_AT_PFIR Error for invalid decimation at
 * PFIR in initialize DDC.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_CENTER_FREQ Error for invalid center
 * frequency in set dual-band
 * settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_BAND_SEP Error for invalid band
 * separation in set dual-band
 * settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_IN_UPPER_FREQ Error for invalid input
 * upper frequency in set
 * dual-band settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_IN_LOWER_FREQ Error for invalid input
 * lower frequency in set
 * dual-band settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_OUT_UPPER_FREQ Error for invalid
 * output upper frequency
 * in set dual-band
 * settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_INV_OUT_LOWER_FREQ Error for invalid
 * output lower frequency
 * in set dual-band
 * settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_OUT_OVERLAP Error indicating output
 * overlap in set dual-band
 * settings.
 * @param TAL_ERR_SETDUALBANDSETTINGS_FTW_OVRG Error indicating FTW overrange in
 * set dual-band settings.
 * @param TAL_ERR_DUALBAND_LNA_TABLE_INV_PROFILE Error for invalid profile in
 * dual-band LNA table.
 * @param TAL_ERR_DUALBAND_LNA_TABLE_INV_INDEX Error for invalid index in dual-
 * band LNA table.
 * @param TAL_ERR_DUALBAND_LNA_TABLE_INV_CHANNEL Error for invalid channel in
 * dual-band LNA table.
 * @param TAL_ERR_DUALBAND_LNA_TABLE_NULL_PARM Error for null parameter in dual-
 * band LNA table.
 * @param TAL_ERR_SETUPNCOSHIFTER_INV_PFIR_CORNER Error for invalid PFIR corner
 * in setup NCO shifter.
 * @param TAL_ERR_SETUPNCOSHIFTER_INV_DDCHB_CORNER Error for invalid DDCHB
 * corner in setup NCO shifter.
 * @param TAL_ERR_SETUPNCOSHIFTER_INV_NCO2SHIFT Error for invalid NCO2 shift in
 * setup NCO shifter.
 * @param TAL_ERR_SETUPNCOSHIFTER_FTW_OVRG Error indicating FTW overrange in
 * setup NCO shifter.
 * @param TAL_ERR_INV_DEFA_SLEWRATE Error for invalid DEFA slew rate.
 * @param TAL_ERR_INV_DEFB_SLEWRATE Error for invalid DEFB slew rate.
 * @param TAL_ERR_GETTXSAMPLEPWR_NULL_PARAM Error for null parameter in get TX
 * sample power.
 * @param TAL_ERR_GETTXSAMPLEPWR_INV_TXREADCHAN Error for invalid TX read
 * channel in get TX sample power.
 * @param TAL_ERR_SETPAPRO_NULL_PARAM Error for null parameter in set PA
 * protection.
 * @param TAL_ERR_SETPAPRO_INV_AVGDURATION Error for invalid average duration in
 * set PA protection.
 * @param TAL_ERR_SETPAPROT_INV_PEAKCNT Error for invalid peak count in set PA
 * protection.
 * @param TAL_ERR_SETPAPROT_INV_TXATTENSTEP Error for invalid TX attenuation
 * step in set PA protection.
 * @param TAL_ERR_SETPAPROT_INV_TX1THRESH Error for invalid TX1 threshold in set
 * PA protection.
 * @param TAL_ERR_SETPAPROT_INV_TX2THRESH Error for invalid TX2 threshold in set
 * PA protection.
 * @param TAL_ERR_SETPAPROT_INV_TX1PEAKTHRESH Error for invalid TX1 peak
 * threshold in set PA protection.
 * @param TAL_ERR_SETPAPROT_INV_TX2PEAKTHRESH Error for invalid TX2 peak
 * threshold in set PA protection.
 * @param TAL_ERR_GETPAERRFLAGS_NULL_PARAM Error for null parameter in get PA
 * error flags.
 * @param TAL_ERR_GETPAPRO_NULL_PARAM Error for null parameter in get PA
 * protection.
 * @param TAL_ERR_DAC_FULLSCALE_INVARMSTATE Error for invalid ARM state in DAC
 * full-scale.
 * @param TAL_ERR_DEFSTATUS_INV_COUNTERERRTHRESHOLD_PARAM Error for invalid
 * counter error
 * threshold parameter in
 * deframer status.
 * @param TAL_ERR_RFPLLFREQ_TX_OUT_OF_RANGE Error indicating TX RF PLL frequency
 * is out of range.
 * @param TAL_ERR_RFPLLFREQ_RX_OUT_OF_RANGE Error indicating RX RF PLL frequency
 * is out of range.
 * @param TAL_ERR_RFPLLFREQ_ORX_OUT_OF_RANGE Error indicating observation RX RF
 * PLL frequency is out of range.
 * @param TAL_ERR_READEVENTSTATUS_NULL_PARM Error for null parameter in read
 * event status.
 * @param TAL_ERR_INV_SIREV Error for invalid SIREV.
 * @param TAL_ERR_REGISTER_ERRORMSG_C0 Error for register error message C0.
 * @param TAL_ERR_SETEXTWORDCTRLGPIO_INV_CHANNEL Error for invalid channel in
 * set external word control GPIO.
 * @param TAL_ERR_SETEXTWORDCTRLGPIO_UNINITIALIZED_RX1 Error for uninitialized
 * RX1 in set external word
 * control GPIO.
 * @param TAL_ERR_SETEXTWORDCTRLGPIO_UNINITIALIZED_RX2 Error for uninitialized
 * RX2 in set external word
 * control GPIO.
 * @param TAL_ERR_SETEXTWORDCTRLGPIO_GPIO_IN_USE_RX1 Error for RX1 GPIO in use
 * in set external word
 * control GPIO.
 * @param TAL_ERR_SETEXTWORDCTRLGPIO_GPIO_IN_USE_RX2 Error for RX2 GPIO in use
 * in set external word
 * control GPIO.
 * @param TAL_ERR_FRAMERSYSREFTOGGLE_INV_FRAMERSEL_PARAM Error for invalid
 * framer selection
 * parameter in framer
 * SYSREF toggle.
 * @param TAL_ERR_SETORXLOSRC_INVALIDPARAM Error for invalid parameter in set
 * observation RX LO source.
 * @param TAL_ERR_GETORXLOSRC_NULLPARAM Error for null parameter in get
 * observation RX LO source.
 * @param TAL_ERR_SETORXLOSRC_TIMEDOUT_ARMMAILBOXBUSY Error indicating timeout
 * for ARM mailbox busy in
 * set observation RX LO
 * source.
 * @param TAL_ERR_SETORXLOCFG_NULL_PARAM Error for null parameter in set
 * observation RX LO configuration.
 * @param TAL_ERR_SETORXLOCFG_INVALIDPARAM Error for invalid parameter in set
 * observation RX LO configuration.
 * @param TAL_ERR_SETORXLOCFG_INVALID_ARMSTATE Error for invalid ARM state in
 * set observation RX LO
 * configuration.
 * @param TAL_ERR_SETORXLOCFG_GPIOUSED Error indicating GPIO used in set
 * observation RX LO configuration.
 * @param TAL_ERR_GETORXLOCFG_NULL_PARAM Error for null parameter in get
 * observation RX LO configuration.
 * @param TAL_ERR_GETORXLOCFG_INVALID_ARMSTATE Error for invalid ARM state in
 * get observation RX LO
 * configuration.
 * @param TAL_ERR_SETFHMCONFIG_NULL_PARAM Error for null parameter in set FHM
 * configuration.
 * @param TAL_ERR_SETFHMCONFIG_INV_FHMGPIOPIN Error for invalid FHM GPIO pin in
 * set FHM configuration.
 * @param TAL_ERR_SETFHMCONFIG_INV_FHMCONFIG_FHM_MIN_FREQ Error for invalid FHM
 * minimum frequency in
 * set FHM configuration.
 * @param TAL_ERR_SETFHMCONFIG_INV_FHMCONFIG_FHM_MAX_FREQ Error for invalid FHM
 * maximum frequency in
 * set FHM configuration.
 * @param TAL_ERR_SETFHMCONFIG_FHMGPIOPIN_IN_USE Error indicating FHM GPIO pin
 * in use in set FHM
 * configuration.
 * @param TAL_ERR_GETFHMCONFIG_NULL_PARAM Error for null parameter in get FHM
 * configuration.
 * @param TAL_ERR_SETFHMMODE_NULL_PARAM Error for null parameter in set FHM
 * mode.
 * @param TAL_ERR_SETFHMMODE_INV_FHM_INIT_FREQ Error for invalid FHM initial
 * frequency in set FHM mode.
 * @param TAL_ERR_SETFHMMODE_INV_FHM_TRIGGER_MODE Error for invalid FHM trigger
 * mode in set FHM mode.
 * @param TAL_ERR_SETFHMMODE_INV_FHM_EXIT_MODE Error for invalid FHM exit mode
 * in set FHM mode.
 * @param TAL_ERR_GETFHMMODE_NULL_PARAM Error for null parameter in get FHM
 * mode.
 * @param TAL_ERR_SETFHMHOP_INV_FHM_FREQ Error for invalid FHM frequency in set
 * FHM hop.
 * @param TAL_ERR_GETFHMSTS_NULL_PARAM Error for null parameter in get FHM
 * status.
 * @param TAL_ERR_SETDCMSHIFT_INV_CH_PARAM Error for invalid channel parameter
 * in set DCM shift.
 * @param TAL_ERR_SETDCMSHIFT_INV_MSHIFT_PARAM Error for invalid M shift
 * parameter in set DCM shift.
 * @param TAL_ERR_GETDCMSHIFT_INV_CH_PARAM Error for invalid channel parameter
 * in get DCM shift.
 * @param TAL_ERR_GETDCMSHIFT_NULL_MSHIFT_PARAM Error for null M shift parameter
 * in get DCM shift.
 * @param TAL_ERR_SETEXTLOOUT_INV_DIV_PARAM Error for invalid division parameter
 * in set external LO output.
 * @param TAL_ERR_SETEXTLOOUT_LO_IN_ENABLED Error indicating LO in enabled in
 * set external LO output.
 * @param TAL_ERR_GETEXTLOOUT_NULL_PARAM Error for null parameter in get
 * external LO output.
 * @param TAL_ERR_DIG_DC_OFFSET_INV_ENABLE_MASK Error for invalid enable mask in
 * digital DC offset.
 * @param TAL_ERR_DIG_DC_OFFSET_NULL_ENABLE_MASK Error for null enable mask in
 * digital DC offset.
 * @param TAL_ERR_SETTCAL_BATCH_SIZE_PARAM Error for invalid batch size
 * parameter in set TCal.
 * @param TAL_ERR_SETTCAL_BATCH_SIZE_ARMSTATE_ERROR Error for ARM state error in
 * set TCal batch size.
 * @param TAL_ERR_GETTCAL_BATCH_SIZE_ARMSTATE_ERROR Error for ARM state error in
 * get TCal batch size.
 * @param TAL_ERR_GETTCAL_BATCHSIZE_NULL_PARAM Error for null parameter in get
 * TCal batch size.
 * @param TAL_ERR_GETTCAL_BATCHSIZE_INV_VALUE Error for invalid value in get
 * TCal batch size.
 * @param TAL_ERR_TXNCOSHIFTER_INV_PROFILE Error for invalid profile in TX NCO
 * shifter.
 * @param TAL_ERR_TXNCOSHIFTER_NULL_PARM Error for null parameter in TX NCO
 * shifter.
 * @param TAL_ERR_TXNCOSHIFTER_INV_TX1_FREQ Error for invalid TX1 frequency in
 * TX NCO shifter.
 * @param TAL_ERR_TXNCOSHIFTER_INV_TX2_FREQ Error for invalid TX2 frequency in
 * TX NCO shifter.
 * @param TAL_ERR_NUMBER_OF_ERRORS Indicates the total number of error enum
 * values.
 ******************************************************************************/
typedef enum {
	TAL_ERR_OK = 0,
	TAL_ERR_INV_NULL_INIT_PARAM,
	TAL_ERR_WAITFOREVENT_INV_PARM,
	TAL_ERR_CLKPLL_INV_HSDIV,
	TAL_ERR_SETCLKPLL_INV_VCOINDEX,
	TAL_ERR_SETCLKPLL_INV_NDIV,
	TAL_ERR_SETRFPLL_INV_PLLNAME,
	TAL_ERR_SETRFPLL_INITCALS_INPROGRESS,
	TAL_ERR_INV_SCALEDDEVCLK_PARAM,
	TAL_ERR_SETRFPLL_INV_REFCLK,
	TAL_ERR_SETORXGAIN_INV_ORXPROFILE,
	TAL_ERR_SETORXGAIN_INV_CHANNEL,
	TAL_ERR_SETORXGAIN_INV_ORX1GAIN,
	TAL_ERR_SETORXGAIN_INV_ORX2GAIN,
	TAL_ERR_SETTXATTEN_INV_STEPSIZE_PARM,
	TAL_ERR_SETRX1GAIN_INV_GAIN_PARM,
	TAL_ERR_SETRX2GAIN_INV_GAIN_PARM,
	TAL_ERR_SER_INV_M_PARM,
	TAL_ERR_SER_INV_NP_PARM,
	TAL_ERR_SER_INV_L_PARM,
	TAL_ERR_SER_INV_ORX_L_PARM,
	TAL_ERR_SER_INV_ORX_M_PARM,
	TAL_ERR_SER_INV_ORX_NP_PARM,
	TAL_ERR_SER_INV_ORX_LANEEN_PARM,
	TAL_ERR_SER_INV_LANERATE_PARM,
	TAL_ERR_SER_INV_ORX_LANERATE_PARM,
	TAL_ERR_SER_INV_LANEEN_PARM,
	TAL_ERR_SER_INV_AMP_PARM,
	TAL_ERR_SER_INV_PREEMP_PARM,
	TAL_ERR_SER_INV_LANEPN_PARM,
	TAL_ERR_SER_LANE_CONFLICT_PARM,
	TAL_ERR_SER_INV_TXSER_DIV_PARM,
	TAL_ERR_SER_LANE_RATE_CONFLICT_PARM,
	TAL_ERR_SER_INV_ZIF_TO_RIF_DATA_PARM,
	TAL_ERR_SER_INV_DUALBAND_DATA_PARM,
	TAL_ERR_SER_INV_RXFRAMER_SEL,
	TAL_ERR_SER_INV_ORXFRAMER_SEL,
	TAL_ERR_SER_LANERATE_ZERO,
	TAL_ERR_HS_AND_LANE_RATE_NOT_INTEGER_MULT,
	TAL_ERR_DESES_HS_AND_LANE_RATE_NOT_INTEGER_MULT,
	TAL_ERR_DESES_INV_LANE_RATE,
	TAL_ERR_DESES_INV_LANE_RATE_DIV,
	TAL_ERR_DESER_INV_M_PARM,
	TAL_ERR_DESER_INV_NP_PARM,
	TAL_ERR_DESER_INV_L_PARM,
	TAL_ERR_DESER_INV_LANERATE_PARM,
	TAL_ERR_DESER_INV_LANEEN_PARM,
	TAL_ERR_DESER_INV_EQ_PARM,
	TAL_ERR_DESER_INV_LANEPN_PARM,
	TAL_ERR_DESER_LANECONFLICT_PARM,
	TAL_ERR_DESER_M_CONFLICT,
	TAL_ERR_DESER_INV_DEFAB_M,
	TAL_ERR_DESER_NP_CONFLICT,
	TAL_ERR_DESER_INV_DEFRAMERSEL,
	TAL_ERR_DESER_TXPROFILE_INV,
	TAL_ERR_FRAMER_INV_M_PARM,
	TAL_ERR_FRAMER_INV_NP_PARM,
	TAL_ERR_FRAMER_INV_S_PARM,
	TAL_ERR_FRAMER_INV_BANKID_PARM,
	TAL_ERR_FRAMER_INV_LANEID_PARM,
	TAL_ERR_FRAMER_INV_SYNCBIN_PARM,
	TAL_ERR_FRAMER_INV_LMFC_OFFSET_PARAM,
	TAL_ERR_FRAMER_INV_DUALBAND_DATA_PARM,
	TAL_ERR_DEFRAMER_INV_BANKID,
	TAL_ERR_ERR_DEFRAMER_INV_LANEID,
	TAL_ERR_DEFRAMER_INV_LMFC_OFFSET,
	TAL_ERR_DEFRAMER_INV_DEFSEL,
	TAL_ERR_DEFRAMER_INV_TXPROFILE,
	TAL_ERR_DEFRAMER_INV_LANESEN,
	TAL_ERR_DEFRAMER_INV_L,
	TAL_ERR_DEFRAMER_INV_M,
	TAL_ERR_DEFRAMER_INV_NP,
	TAL_ERR_DEFRAMER_INV_F,
	TAL_ERR_DEFRAMER_INV_K,
	TAL_ERR_DEFRAMER_INV_FK,
	TAL_ERR_DEFRAMER_INV_PCLK,
	TAL_ERR_DEFRAMER_INV_PCLKDIV,
	TAL_ERR_RSTDEFRAMER_INV_DEFSEL,
	TAL_ERR_RSTFRAMER_INV_FRAMERSEL,
	TAL_ERR_SETDFRMIRQMASK_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_DEFSTATUS_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_DEFSTATUS_NULL_DEFRAMERSTATUS_PARAM,
	TAL_ERR_GETDFRMIRQMASK_NULL_IRQMASK_PARAM,
	TAL_ERR_GETDFRMIRQMASK_INV_DEFRAMERSELECT_PARAM,
	TAL_ERR_CLRDFRMIRQ_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_GETDFRMIRQSRC_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_GETDFRMIRQSRC_NULL_STATUS_PARAM,
	TAL_ERR_ENDEFSYSREF_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_ENFRAMERSYSREF_INV_FRAMERSEL_PARAM,
	TAL_ERR_FRAMER_INV_TESTDATA_SOURCE_PARAM,
	TAL_ERR_FRAMER_INV_INJECTPOINT_PARAM,
	TAL_ERR_FRAMERSTATUS_INV_FRAMERSEL_PARAM,
	TAL_ERR_FRAMERSTATUS_NULL_FRAMERSTATUS_PARAM,
	TAL_ERR_EN_DEFRAMER_PRBS_INV_PARAM,
	TAL_ERR_READDFRMPRBS_INV_DEFSEL_PARAM,
	TAL_ERR_READDFRMPRBS_NULL_PARAM,
	TAL_ERR_INITARM_INV_ARMCLK_PARAM,
	TAL_ERR_LOADHEX_INVALID_CHKSUM,
	TAL_ERR_LOADBIN_INVALID_BYTECOUNT,
	TAL_ERR_READARMMEM_INV_ADDR_PARM,
	TAL_ERR_WRITEARMMEM_INV_ADDR_PARM,
	TAL_ERR_ARMCMD_INV_OPCODE_PARM,
	TAL_ERR_ARMCMD_INV_NUMBYTES_PARM,
	TAL_ERR_ARMCMDSTATUS_INV_OPCODE_PARM,
	TAL_ERR_JESD204B_ILAS_MISMATCH_NULLPARAM,
	TAL_ERR_JESD204B_ILAS_MISMATCH_NO_ACTIVE_LINK,
	TAL_ERR_JESD204B_ILAS_MISMATCH_SYNC_NOT_DETECTED,
	TAL_ERR_JESD204B_ILAS_MISMATCH_INVALID_DEFRAMER,
	TAL_ERR_TALFINDDFRMRLANECNTERROR_INV_DEFRAMERSEL_PARAM,
	TAL_ERR_TALFINDDFRMRLANECNTERROR_NULL_PARAM,
	TAL_ERR_TALFINDDFRMRLANEERROR_NULL_PARAM,
	TAL_ERR_RXGAINTABLE_INV_CHANNEL,
	TAL_ERR_RXGAINTABLE_INV_PROFILE,
	TAL_ERR_RXGAINTABLE_INV_GAIN_INDEX_RANGE,
	TAL_ERR_ORXGAINTABLE_INV_CHANNEL,
	TAL_ERR_ORXGAINTABLE_INV_PROFILE,
	TAL_ERR_ORXGAINTABLE_INV_GAIN_INDEX_RANGE,
	TAL_ERR_RXFRAMER_INV_FK_PARAM,
	TAL_ERR_RXFRAMER_INV_L_PARAM,
	TAL_ERR_INV_RX_GAIN_MODE_PARM,
	TAL_ERR_UNSUPPORTED_RX_GAIN_MODE_PARM,
	TAL_ERR_INV_AGC_RX_STRUCT_INIT,
	TAL_ERR_INV_AGC_PWR_STRUCT_INIT,
	TAL_ERR_INV_AGC_PWR_LWR0THRSH_PARAM,
	TAL_ERR_INV_AGC_PWR_LWR1THRSH_PARAM,
	TAL_ERR_INV_AGC_PWR_LWR0PWRGAINSTEP_PARAM,
	TAL_ERR_INV_AGC_PWR_LWR1PWRGAINSTEP_PARAM,
	TAL_ERR_INV_AGC_PWR_MSR_DURATION_PARAM,
	TAL_ERR_INV_AGC_PWR_IP3RANGE_PARAM,
	TAL_ERR_INV_AGC_PWR_UPPWR0THRSH_PARAM,
	TAL_ERR_INV_AGC_PWR_UPPWR1THRSH_PARAM,
	TAL_ERR_INV_AGC_PWR_LOGSHIFT_PARAM,
	TAL_ERR_INV_AGC_PWR_UPPWR0GAINSTEP_PARAM,
	TAL_ERR_INV_AGC_PWR_UPPWR1GAINSTEP_PARAM,
	TAL_ERR_INV_AGC_PKK_STRUCT_INIT,
	TAL_ERR_INV_AGC_PKK_HIGHTHRSH_PARAM,
	TAL_ERR_INV_AGC_PKK_LOWGAINMODEHIGHTHRSH_PARAM,
	TAL_ERR_INV_AGC_PKK_LOWGAINHIGHTHRSH_PARAM,
	TAL_ERR_INV_AGC_PKK_LOWGAINTHRSH_PARAM,
	TAL_ERR_INV_AGC_PKK_GAINSTEPATTACK_PARAM,
	TAL_ERR_INV_AGC_PKK_GAINSTEPRECOVERY_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2OVRLD_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2OVRLDDURATION_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2OVRLDTHRSHCNT_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2GAINSTEPRECOVERY_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2GAINSTEP0RECOVERY_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2GAINSTEP1RECOVERY_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2GAINSTEPATTACK_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2OVRLDPWRMODE_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2OVRLDSEL_PARAM,
	TAL_ERR_INV_AGC_PKK_HB2THRSHCFG_PARAM,
	TAL_ERR_INV_AGC_RX_APD_HIGH_LOW_THRESH,
	TAL_ERR_INV_AGC_RX_PEAK_WAIT_TIME_PARM,
	TAL_ERR_INV_AGC_RX_MIN_MAX_GAIN_PARM,
	TAL_ERR_INV_AGC_RX_MIN_GAIN_GRT_THAN_MAX_GAIN_PARM,
	TAL_ERR_INV_AGC_RX_GAIN_UPDATE_TIME_PARM,
	TAL_ERR_INV_AGC_RX1ATTACKDELAY_PARAM,
	TAL_ERR_INV_AGC_RX2ATTACKDELAY_PARAM,
	TAL_ERR_INV_AGC_RX_LOWTHRSHPREVENTGAIN_PARM,
	TAL_ERR_INV_AGC_RX_CHANGEGAINTHRSHHIGH_PARM,
	TAL_ERR_INV_AGC_RX_RESETONRXON_PARM,
	TAL_ERR_INV_AGC_RX_ENSYNCHPULSECAINCNTR_PARM,
	TAL_ERR_INV_AGC_RX_ENIP3OPTTHRSH_PARM,
	TAL_ERR_INV_AGC_RX_ENFASTRECOVERYLOOP_PARM,
	TAL_ERR_INV_AGC_SLOWLOOPDELAY_PARAM,
	TAL_ERR_INV_MINAGCSLOWLOOPDELAY_PARAM,
	TAL_ERR_INV_AGC_CLK_DIV_RATIO_PARM,
	TAL_ERR_INV_AGC_CLK_PARM,
	TAL_ERR_INV_AGC_CLK_RATIO,
	TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_UPDATE_TIME_PARM,
	TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_MID_INTERVAL_PARM,
	TAL_ERR_INV_AGC_RX_GAIN_UNDERRANGE_HIGH_INTERVAL_PARM,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_CLKPLLCP,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_CLKPLL_LOCK,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_RFPLLCP,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_RFPLL_LOCK,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_AUXPLLCP,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_AUXPLL_LOCK,
	TAL_ERR_WAITFOREVENT_TIMEDOUT_ARMBUSY,
	TAL_ERR_TIMEDOUT_ARMMAILBOXBUSY,
	TAL_ERR_EN_TRACKING_CALS_ARMSTATE_ERROR,
	TAL_ERR_GETPENDINGTRACKINGCALS_NULL_PARAM,
	TAL_ERR_TRACKINGCAL_OUTOFRANGE_PARAM,
	TAL_ERR_PAUSETRACKINGCAL_INV_PARAM,
	TAL_ERR_GETPAUSECALSTATE_NULL_PARAM,
	TAL_ERR_SET_ARMGPIO_PINS_GPIO_IN_USE,
	TAL_ERR_SET_ARMGPIO_PINS_INV_SIGNALID,
	TAL_ERR_SET_ARMGPIO_PINS_INV_GPIOPIN,
	TAL_ERR_SET_RXDATAFRMT_NULL_PARAM,
	TAL_ERR_SET_RXDATAFRMT_FORMATSELECT_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_TEMPCOMP_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_ROUNDMODE_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPDATAFRMT_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPENCNAN_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPEXPBITS_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPHIDELEADINGONE_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPRX1ATTEN_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_FPRX2ATTEN_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_INTEMBEDDEDBITS_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_INTSAMPLERESOLUTION_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_PINSTEPSIZE_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_RX1GPIOSELECT_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_RX2GPIOSELECT_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_RXCHAN_DISABLED,
	TAL_ERR_SET_RXDATAFRMT_EXTERNALLNAGAIN_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_RX1GPIO_INUSE,
	TAL_ERR_SET_RXDATAFRMT_RX2GPIO_INUSE,
	TAL_ERR_SET_RXDATAFRMT_DATARES_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_EXTSLICER_RX1GPIO_INVPARAM,
	TAL_ERR_SET_RXDATAFRMT_EXTSLICER_RX2GPIO_INVPARAM,
	TAL_ERR_GET_DATAFORMAT_NULL_PARAM,
	TAL_ERR_GET_SLICERPOS_NULL_PARAM,
	TAL_ERR_INV_RX_DEC_POWER_PARAM,
	TAL_ERR_GETRXDECPOWER_INV_CHANNEL,
	TAL_ERR_GETRXDECPOWER_INV_PROFILE,
	TAL_ERR_INIT_NULLPARAM,
	TAL_ERR_INIT_INV_DEVCLK,
	TAL_ERR_GETRADIOSTATE_NULL_PARAM,
	TAL_ERR_CHECKGETMCS_STATUS_NULL_PARM,
	TAL_ERR_WAIT_INITCALS_ARMERROR,
	TAL_ERR_WAIT_INITCALS_NULL_PARAM,
	TAL_ERR_CHECK_PLL_LOCK_NULL_PARM,
	TAL_ERR_READGPIOSPI_NULL_PARM,
	TAL_ERR_READGPIO3V3SPI_NULL_PARM,
	TAL_ERR_GET_TXFILTEROVRG_NULL_PARM,
	TAL_ERR_PROGRAM_RXGAIN_TABLE_NULL_PARM,
	TAL_ERR_PROGRAM_ORXGAIN_TABLE_NULL_PARM,
	TAL_ERR_PROGRAMFIR_NULL_PARM,
	TAL_ERR_PROGRAMFIR_COEFS_NULL,
	TAL_ERR_READ_DEFRAMERSTATUS_NULL_PARAM,
	TAL_ERR_READ_DEFRAMERPRBS_NULL_PARAM,
	TAL_ERR_ARMCMDSTATUS_NULL_PARM,
	TAL_ERR_PROGRAMFIR_INV_FIRNAME_PARM,
	TAL_ERR_RXFIR_INV_GAIN_PARM,
	TAL_ERR_PROGRAMFIR_INV_NUMTAPS_PARM,
	TALISE_ERR_TXFIR_INV_NUMTAPS_PARM,
	TALISE_ERR_TXFIR_INV_NUMROWS,
	TALISE_ERR_TXFIR_TAPSEXCEEDED,
	TALISE_ERR_RXFIR_INV_DDC,
	TALISE_ERR_RXFIR_INV_NUMTAPS_PARM,
	TALISE_ERR_RXFIR_INV_NUMROWS,
	TALISE_ERR_RXFIR_TAPSEXCEEDED,
	TALISE_ERR_ORXFIR_INV_DDC,
	TALISE_ERR_ORXFIR_INV_NUMTAPS_PARM,
	TALISE_ERR_ORXFIR_INV_NUMROWS,
	TALISE_ERR_ORXFIR_TAPSEXCEEDED,
	TAL_ERR_WAITARMCMDSTATUS_INV_OPCODE,
	TAL_ERR_WAITARMCMDSTATUS_TIMEOUT,
	TAL_ERR_READARMCMDSTATUS_NULL_PARM,
	TAL_ERR_LOADBIN_NULL_PARAM,
	TAL_ERR_GETARMVER_NULL_PARM,
	TAL_ERR_GETARMVER_V2_NULL_PARM,
	TAL_ERR_GETARMVER_V2_INVALID_ARM_NOT_LOADED,
	TAL_ERR_READEVENTSTATUS_INV_PARM,
	TAL_ERR_FRAMER_INV_FRAMERSEL_PARAM,
	TAL_ERR_FRAMER_ERRINJECT_INV_FRAMERSEL_PARAM,
	TAL_ERR_CHECKPLLLOCK_NULL_PARM,
	TAL_ERR_FRAMER_INV_FRAMERSEL_PARM,
	TAL_ERR_SETTXATTEN_INV_TXCHANNEL,
	TAL_ERR_SETTXATTEN_INV_PARM,
	TAL_ERR_RXFRAMER_INV_OUTPUT_RATE,
	TAL_ERR_RXFRAMER_INV_PCLKFREQ,
	TAL_ERR_BBIC_INV_CHN,
	TAL_ERR_INV_GP_INT_MASK_PARM,
	TAL_ERR_INV_GP_INT_MASK_NULL_PARM,
	TAL_ERR_GP_INT_STATUS_NULL_PARAM,
	TAL_ERR_INV_DAC_SAMP_XBAR_CHANNEL_SEL,
	TAL_ERR_INV_ADC_SAMP_XBAR_FRAMER_SEL,
	TAL_ERR_INV_ADC_SAMP_XBAR_SELECT_PARAM,
	TAL_ERR_INV_DAC_SAMP_XBAR_SELECT_PARAM,
	TAL_ERR_GETRFPLL_INV_PLLNAME,
	TAL_ERR_GET_PLLFREQ_INV_REFCLKDIV,
	TAL_ERR_GET_PLLFREQ_INV_HSDIV,
	TAL_ERR_GETRFPLL_NULLPARAM,
	TAL_ERR_FRAMER_A_AND_B_INV_M_PARM,
	TAL_ERR_SETRXGAIN_RXPROFILE_INVALID,
	TAL_ERR_SETRXGAIN_INV_CHANNEL,
	TAL_ERR_INIT_CALS_COMPLETED_NULL_PTR,
	TAL_ERR_CHKINITCALS_NULL_PTR,
	TAL_ERR_INIT_CALS_LASTRUN_NULL_PTR,
	TAL_ERR_GETENABLED_TRACK_CALS_NULL_PTR,
	TAL_ERR_INIT_CALS_MIN_NULL_PTR,
	TAL_ERR_INIT_ERR_CAL_NULL_PTR,
	TAL_ERR_INIT_ERR_CODE_NULL_PTR,
	TAL_ERR_READARMCFG_ARMERRFLAG,
	TAL_ERR_GETRXGAIN_INV_RXPROFILE,
	TAL_ERR_GETRXGAIN_INV_CHANNEL,
	TAL_ERR_GETRXGAIN_GAIN_RANGE_EXCEEDED,
	TAL_ERR_GETOBSRXGAIN_INV_ORXPROFILE,
	TAL_ERR_GETOBSRXGAIN_INV_CHANNEL,
	TAL_ERR_GETOBSRXGAIN_GAIN_RANGE_EXCEEDED,
	TAL_ERR_SETUPDUALBANDRXAGC_GAIN_RANGE_MISMATCH,
	TAL_ERR_SETUPDUALBANDRXAGC_GAIN_OUT_OF_RANGE,
	TAL_ERR_VERIFYBIN_CHECKSUM_TIMEOUT,
	TAL_ERR_ARMCMDSTATUS_ARMERROR,
	TAL_ERR_VERRXPFILE_INV_IQRATE,
	TAL_ERR_VERRXPFILE_INV_RFBW,
	TAL_ERR_VERRXPFILE_INV_RHB1,
	TAL_ERR_VERRXPFILE_INV_DEC5,
	TAL_ERR_VERRXPFILE_INV_FIR,
	TAL_ERR_VERRXPFILE_INV_COEF,
	TAL_ERR_VERRXPFILE_INV_DDC,
	TAL_ERR_VERORXPFILE_INV_IQRATE,
	TAL_ERR_VERORXPFILE_INV_RFBW,
	TAL_ERR_VERORXPFILE_INV_RHB1,
	TAL_ERR_VERORXPFILE_INV_DEC5,
	TAL_ERR_VERORXPFILE_INV_FIR,
	TAL_ERR_VERORXPFILE_INV_COEF,
	TAL_ERR_VERORXPFILE_INV_DDC,
	TAL_ERR_VERTXPFILE_INV_DISONPLLUNLOCK,
	TAL_ERR_VERTXPFILE_INV_IQRATE,
	TAL_ERR_VERTXPFILE_INV_RFBW,
	TAL_ERR_VERTXPFILE_INV_THB1,
	TAL_ERR_VERTXPFILE_INV_THB2,
	TAL_ERR_VERTXPFILE_INV_THB3,
	TAL_ERR_VERTXPFILE_INV_INT5,
	TAL_ERR_VERTXPFILE_INV_HBMUTEX,
	TAL_ERR_VERTXPFILE_INV_FIRIPL,
	TAL_ERR_VERTXPFILE_INV_COEF,
	TAL_ERR_VERTXPFILE_INV_DACDIV,
	TAL_ERR_VERPFILE_INV_RFPLLMCSMODE,
	TAL_ERR_VERPFILE_TXHSCLK,
	TAL_ERR_VERPFILE_RXHSCLK,
	TAL_ERR_VERPFILE_ORXHSCLK,
	TAL_ERR_SETSPI_INV_CMOS_DRV_STR,
	TAL_ERR_SETCLKPLL_INV_TXATTENDIV,
	TAL_ERR_SETTXTOORXMAP_INV_ORX1_MAP,
	TAL_ERR_SETTXTOORXMAP_INV_ORX2_MAP,
	TAL_ERR_GETRXTXENABLE_NULLPARAM,
	TAL_ERR_SETRXTXENABLE_INVCHANNEL,
	TAL_ERR_GETTXATTEN_NULL_PARM,
	TAL_ERR_GETTXATTEN_INV_TXCHANNEL,
	TAL_ERR_INV_RADIO_CTL_MASK_PARM,
	TAL_ERR_GETPINMODE_NULLPARAM,
	TAL_ERR_SET_ARMGPIO_NULLPARAM,
	TAL_ERR_GETTEMPERATURE_NULLPARAM,
	TAL_ERR_GETTXLOLSTATUS_NULL_PARAM,
	TAL_ERR_GETTXLOLSTATUS_INV_CHANNEL_PARM,
	TAL_ERR_GETTXQECSTATUS_NULL_PARAM,
	TAL_ERR_GETTXQECSTATUS_INV_CHANNEL_PARM,
	TAL_ERR_GETRXQECSTATUS_NULL_PARAM,
	TAL_ERR_GETRXQECSTATUS_INV_CHANNEL_PARM,
	TAL_ERR_GETORXQECSTATUS_NULL_PARAM,
	TAL_ERR_GETORXQECSTATUS_INV_CHANNEL_PARM,
	TAL_ERR_GETRXHD2STATUS_NULL_PARAM,
	TAL_ERR_GETRXHD2STATUS_INV_CHANNEL_PARM,
	TAL_ERR_VERIFYSPI_READ_LOW_ADDR_ERROR,
	TAL_ERR_VERIFYSPI_WRITE_LOW_ADDR_ERROR,
	TAL_ERR_VERIFYSPI_READ_HIGH_ADDR_ERROR,
	TAL_ERR_VERIFYSPI_WRITE_HIGH_ADDR_ERROR,
	TAL_ERR_GETAPIVERSION_NULLPARAM,
	TAL_ERR_INV_DAC_FULLSCALE_PARM,
	TAL_ERR_RESET_TXLOL_INV_CHANNEL_PARM,
	TAL_ERR_RESET_TXLOL_ARMSTATE_ERROR,
	TAL_ERR_SETHD2CFG_NULL_PARAM,
	TAL_ERR_SETHD2CFG_ARMSTATE_ERROR,
	TAL_ERR_GETHD2CFG_NULL_PARAM,
	TAL_ERR_GETHD2CFG_ARMSTATE_ERROR,
	TAL_ERR_SET_SPI2_ENABLE_INVALID_TX_ATTEN_SEL,
	TAL_ERR_SET_SPI2_ENABLE_GPIO_IN_USE,
	TAL_ERR_SETRXMGCPINCTRL_INV_RX1_INC_PIN,
	TAL_ERR_SETRXMGCPINCTRL_INV_RX1_DEC_PIN,
	TAL_ERR_SETRXMGCPINCTRL_INV_RX2_INC_PIN,
	TAL_ERR_SETRXMGCPINCTRL_INV_RX2_DEC_PIN,
	TAL_ERR_SETRXMGCPINCTRL_INV_CHANNEL,
	TAL_ERR_SETRXMGCPINCTRL_INV_INC_STEP,
	TAL_ERR_SETRXMGCPINCTRL_INV_DEC_STEP,
	TAL_ERR_SETRXMGCPINCTRL_RX1_GPIO_IN_USE,
	TAL_ERR_SETRXMGCPINCTRL_RX2_GPIO_IN_USE,
	TAL_ERR_GETRXMGCPINCTRL_INV_CHANNEL,
	TAL_ERR_GETRXMGCPINCTRL_NULL_PARAM,
	TAL_ERR_SETRFPLL_LOOPFILTER_INV_LOOPBANDWIDTH,
	TAL_ERR_SETRFPLL_LOOPFILTER_INV_STABILITY,
	TAL_ERR_SETRFPLL_LOOPFILTER_INV_PLLSEL,
	TAL_ERR_GETRFPLL_LOOPFILTER_NULLPARAM,
	TAL_ERR_GETRFPLL_LOOPFILTER_INV_PLLSEL,
	TAL_ERR_GETDEVICEREV_NULLPARAM,
	TAL_ERR_GETPRODUCTID_NULLPARAM,
	TAL_ERR_ENABLETXNCO_INV_PROFILE,
	TAL_ERR_ENABLETXNCO_NULL_PARM,
	TAL_ERR_ENABLETXNCO_INV_TX1_FREQ,
	TAL_ERR_ENABLETXNCO_INV_TX2_FREQ,
	TAL_ERR_SETTXATTENCTRLPIN_NULL_PARAM,
	TAL_ERR_SETTXATTENCTRLPIN_TX1_GPIO_IN_USE,
	TAL_ERR_SETTXATTENCTRLPIN_TX2_GPIO_IN_USE,
	TAL_ERR_SETTXATTENCTRLPIN_INV_PARM,
	TAL_ERR_SETTXATTENCTRLPIN_INV_TX1_INC_PIN,
	TAL_ERR_SETTXATTENCTRLPIN_INV_TX1_DEC_PIN,
	TAL_ERR_SETTXATTENCTRLPIN_INV_TX2_INC_PIN,
	TAL_ERR_SETTXATTENCTRLPIN_INV_TX2_DEC_PIN,
	TAL_ERR_SETTXATTENCTRLPIN_INV_CHANNEL,
	TAL_ERR_GETTXATTENCTRLPIN_INV_CHANNEL,
	TAL_ERR_GETTXATTENCTRLPIN_NULL_PARAM,
	TAL_ERR_SETUPDUALBANDRX1AGC_GPIO3P3_IN_USE,
	TAL_ERR_SETUPDUALBANDRX2AGC_GPIO3P3_IN_USE,
	TAL_ERR_SETUPDUALBANDRXAGC_INV_CHANNEL,
	TAL_ERR_SETUPDUALBANDRXAGC_NULL_PARAM,
	TAL_ERR_SETUPDUALBANDRXAGC_INV_PWRMARGIN,
	TAL_ERR_SETUPDUALBANDRXAGC_INV_DECPWR,
	TAL_ERR_GETDUALBANDLNA_INV_CHANNEL,
	TAL_ERR_GETDUALBANDLNA_NULL_PARAM,
	TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_2OR4,
	TAL_ERR_INITIALIZE_DDC_INV_TOTAL_M_4OR8,
	TAL_ERR_INITIALIZE_DDC_NULL_PARAM,
	TAL_ERR_RXNCOFTW_INVNCO,
	TAL_ERR_INITIALIZE_DDC_INV_RXDDCMODE,
	TAL_ERR_INITIALIZE_DDC_INV_DEC_AT_PFIR,
	TAL_ERR_SETDUALBANDSETTINGS_INV_CENTER_FREQ,
	TAL_ERR_SETDUALBANDSETTINGS_INV_BAND_SEP,
	TAL_ERR_SETDUALBANDSETTINGS_INV_IN_UPPER_FREQ,
	TAL_ERR_SETDUALBANDSETTINGS_INV_IN_LOWER_FREQ,
	TAL_ERR_SETDUALBANDSETTINGS_INV_OUT_UPPER_FREQ,
	TAL_ERR_SETDUALBANDSETTINGS_INV_OUT_LOWER_FREQ,
	TAL_ERR_SETDUALBANDSETTINGS_OUT_OVERLAP,
	TAL_ERR_SETDUALBANDSETTINGS_FTW_OVRG,
	TAL_ERR_DUALBAND_LNA_TABLE_INV_PROFILE,
	TAL_ERR_DUALBAND_LNA_TABLE_INV_INDEX,
	TAL_ERR_DUALBAND_LNA_TABLE_INV_CHANNEL,
	TAL_ERR_DUALBAND_LNA_TABLE_NULL_PARM,
	TAL_ERR_SETUPNCOSHIFTER_INV_PFIR_CORNER,
	TAL_ERR_SETUPNCOSHIFTER_INV_DDCHB_CORNER,
	TAL_ERR_SETUPNCOSHIFTER_INV_NCO2SHIFT,
	TAL_ERR_SETUPNCOSHIFTER_FTW_OVRG,
	TAL_ERR_INV_DEFA_SLEWRATE,
	TAL_ERR_INV_DEFB_SLEWRATE,
	TAL_ERR_GETTXSAMPLEPWR_NULL_PARAM,
	TAL_ERR_GETTXSAMPLEPWR_INV_TXREADCHAN,
	TAL_ERR_SETPAPRO_NULL_PARAM,
	TAL_ERR_SETPAPRO_INV_AVGDURATION,
	TAL_ERR_SETPAPROT_INV_PEAKCNT,
	TAL_ERR_SETPAPROT_INV_TXATTENSTEP,
	TAL_ERR_SETPAPROT_INV_TX1THRESH,
	TAL_ERR_SETPAPROT_INV_TX2THRESH,
	TAL_ERR_SETPAPROT_INV_TX1PEAKTHRESH,
	TAL_ERR_SETPAPROT_INV_TX2PEAKTHRESH,
	TAL_ERR_GETPAERRFLAGS_NULL_PARAM,
	TAL_ERR_GETPAPRO_NULL_PARAM,
	TAL_ERR_DAC_FULLSCALE_INVARMSTATE,
	TAL_ERR_DEFSTATUS_INV_COUNTERERRTHRESHOLD_PARAM,
	TAL_ERR_RFPLLFREQ_TX_OUT_OF_RANGE,
	TAL_ERR_RFPLLFREQ_RX_OUT_OF_RANGE,
	TAL_ERR_RFPLLFREQ_ORX_OUT_OF_RANGE,
	TAL_ERR_READEVENTSTATUS_NULL_PARM,
	TAL_ERR_INV_SIREV,
	TAL_ERR_REGISTER_ERRORMSG_C0,
	TAL_ERR_SETEXTWORDCTRLGPIO_INV_CHANNEL,
	TAL_ERR_SETEXTWORDCTRLGPIO_UNINITIALIZED_RX1,
	TAL_ERR_SETEXTWORDCTRLGPIO_UNINITIALIZED_RX2,
	TAL_ERR_SETEXTWORDCTRLGPIO_GPIO_IN_USE_RX1,
	TAL_ERR_SETEXTWORDCTRLGPIO_GPIO_IN_USE_RX2,
	TAL_ERR_FRAMERSYSREFTOGGLE_INV_FRAMERSEL_PARAM,
	TAL_ERR_SETORXLOSRC_INVALIDPARAM,
	TAL_ERR_GETORXLOSRC_NULLPARAM,
	TAL_ERR_SETORXLOSRC_TIMEDOUT_ARMMAILBOXBUSY,
	TAL_ERR_SETORXLOCFG_NULL_PARAM,
	TAL_ERR_SETORXLOCFG_INVALIDPARAM,
	TAL_ERR_SETORXLOCFG_INVALID_ARMSTATE,
	TAL_ERR_SETORXLOCFG_GPIOUSED,
	TAL_ERR_GETORXLOCFG_NULL_PARAM,
	TAL_ERR_GETORXLOCFG_INVALID_ARMSTATE,
	TAL_ERR_SETFHMCONFIG_NULL_PARAM,
	TAL_ERR_SETFHMCONFIG_INV_FHMGPIOPIN,
	TAL_ERR_SETFHMCONFIG_INV_FHMCONFIG_FHM_MIN_FREQ,
	TAL_ERR_SETFHMCONFIG_INV_FHMCONFIG_FHM_MAX_FREQ,
	TAL_ERR_SETFHMCONFIG_FHMGPIOPIN_IN_USE,
	TAL_ERR_GETFHMCONFIG_NULL_PARAM,
	TAL_ERR_SETFHMMODE_NULL_PARAM,
	TAL_ERR_SETFHMMODE_INV_FHM_INIT_FREQ,
	TAL_ERR_SETFHMMODE_INV_FHM_TRIGGER_MODE,
	TAL_ERR_SETFHMMODE_INV_FHM_EXIT_MODE,
	TAL_ERR_GETFHMMODE_NULL_PARAM,
	TAL_ERR_SETFHMHOP_INV_FHM_FREQ,
	TAL_ERR_GETFHMSTS_NULL_PARAM,
	TAL_ERR_SETDCMSHIFT_INV_CH_PARAM,
	TAL_ERR_SETDCMSHIFT_INV_MSHIFT_PARAM,
	TAL_ERR_GETDCMSHIFT_INV_CH_PARAM,
	TAL_ERR_GETDCMSHIFT_NULL_MSHIFT_PARAM,
	TAL_ERR_SETEXTLOOUT_INV_DIV_PARAM,
	TAL_ERR_SETEXTLOOUT_LO_IN_ENABLED,
	TAL_ERR_GETEXTLOOUT_NULL_PARAM,
	TAL_ERR_DIG_DC_OFFSET_INV_ENABLE_MASK,
	TAL_ERR_DIG_DC_OFFSET_NULL_ENABLE_MASK,
	TAL_ERR_SETTCAL_BATCH_SIZE_PARAM,
	TAL_ERR_SETTCAL_BATCH_SIZE_ARMSTATE_ERROR,
	TAL_ERR_GETTCAL_BATCH_SIZE_ARMSTATE_ERROR,
	TAL_ERR_GETTCAL_BATCHSIZE_NULL_PARAM,
	TAL_ERR_GETTCAL_BATCHSIZE_INV_VALUE,
	TAL_ERR_TXNCOSHIFTER_INV_PROFILE,
	TAL_ERR_TXNCOSHIFTER_NULL_PARM,
	TAL_ERR_TXNCOSHIFTER_INV_TX1_FREQ,
	TAL_ERR_TXNCOSHIFTER_INV_TX2_FREQ,
	TAL_ERR_NUMBER_OF_ERRORS /* Keep this ENUM last as a reference to the total number of error enum values */
} taliseErr_t;

/***************************************************************************//**
 * @brief The `taliseErrHdls_t` is an enumeration that defines a set of error
 * handlers for various API and hardware abstraction layer (HAL) errors
 * in the Talise API. Each enumerator represents a specific type of error
 * handler, such as those for HAL wait/delay, log, SPI, and GPIO function
 * errors, as well as handlers for invalid parameters and API functional
 * errors. Additionally, it includes handlers for ARM-related errors and
 * reserved handlers for future use. This enumeration is used to
 * categorize and manage error handling within the Talise API framework.
 *
 * @param TAL_ERRHDL_HAL_WAIT API Error handler for HAL wait/delay function
 * errors.
 * @param TAL_ERRHDL_HAL_LOG API Error handler for HAL log function error.
 * @param TAL_ERRHDL_HAL_SPI API Error handler for HAL SPI function errors.
 * @param TAL_ERRHDL_HAL_GPIO API Error handler for HAL GPIO function errors.
 * @param TAL_ERRHDL_INVALID_PARAM API Error handler for invalid parameter
 * errors.
 * @param TAL_ERRHDL_API_FAIL API Error handler for API functional errors.
 * @param TAL_ERRHDL_APIARM_ERR Talise API layer ARM error handler.
 * @param TAL_ERRHDL_ARM_CMD_ERR Talise sendArmCommand error handler.
 * @param TAL_ERRHDL_ARM_INITCALS_ERR Talise init calibration error handler.
 * @param TAL_ERRHDL_API_GPIO Talise GPIO error handler.
 * @param TAL_ERRHDL_API_C0_PCA Reserved error handler.
 * @param TAL_ERRHDL_API_C0_PHMFOVR Reserved error handler.
 ******************************************************************************/
typedef enum {
	TAL_ERRHDL_HAL_WAIT, /*!<API Error handler for HAL wait/delay function errors */
	TAL_ERRHDL_HAL_LOG,  /*!<API Error handler for HAL log function error */
	TAL_ERRHDL_HAL_SPI,  /*!<API Error handler for HAL SPI function errors */
	TAL_ERRHDL_HAL_GPIO, /*!<API Error handler for HAL GPIO function errors */
	TAL_ERRHDL_INVALID_PARAM, /*!<API Error handler invalid parameter errors */
	TAL_ERRHDL_API_FAIL,  /*!<API Error handler API functional errors */
	TAL_ERRHDL_APIARM_ERR,   /*!< Talise API layer ARM error handler */
	TAL_ERRHDL_ARM_CMD_ERR, /*!< Talise sendArmCommand error handler */
	TAL_ERRHDL_ARM_INITCALS_ERR, /*!< Talise init calibration error handler */
	TAL_ERRHDL_API_GPIO,   /*!< Talise GPIO error handler */
	TAL_ERRHDL_API_C0_PCA,   /*!< reserved error handler */
	TAL_ERRHDL_API_C0_PHMFOVR  /*!< reserved error handler */
} taliseErrHdls_t;

/***************************************************************************//**
 * @brief The `talRecoveryActions_t` is an enumeration that defines a set of
 * recovery actions for the Talise API, which are used as return values
 * to indicate the status of API operations. Each enumerator represents a
 * specific action or status, ranging from no action required to various
 * levels of warnings and errors that may require resets or other
 * corrective measures. This enumeration helps in managing error handling
 * and recovery processes within the Talise API framework.
 *
 * @param TALACT_NO_ACTION Indicates that the API is OK and no action is
 * required.
 * @param TALACT_WARN_RESET_LOG Indicates that the API is OK but the log is not
 * working.
 * @param TALACT_WARN_RERUN_TRCK_CAL Indicates that the API is not good and
 * specific tracking calibrations need to be
 * reset.
 * @param TALACT_WARN_RESET_GPIO Indicates that the API is OK but the GPIO is
 * not working.
 * @param TALACT_ERR_CHECK_TIMER Indicates that the API is OK but the timer is
 * not working.
 * @param TALACT_ERR_RESET_ARM Indicates that the API is not good and only the
 * ARM needs to be reset.
 * @param TALACT_ERR_RERUN_INIT_CALS Indicates that the API is not good and the
 * initial calibration sequence needs to be
 * reset.
 * @param TALACT_ERR_RESET_SPI Indicates that the API is not good and the SPI is
 * not working.
 * @param TALACT_ERR_RESET_GPIO Indicates that the API is not good and the GPIO
 * is not working.
 * @param TALACT_ERR_CHECK_PARAM Indicates that the API is OK but there is an
 * invalid parameter.
 * @param TALACT_ERR_RESET_FULL Indicates that the API is not good and a full
 * reset is required.
 * @param TALACT_ERR_RESET_JESD204FRAMERA Indicates that the API is not good and
 * the JESD204 Framer A needs to be
 * reset.
 * @param TALACT_ERR_RESET_JESD204FRAMERB Indicates that the API is not good and
 * the JESD204 Framer B needs to be
 * reset.
 * @param TALACT_ERR_RESET_JESD204DEFRAMERA Indicates that the API is not good
 * and the JESD204 Deframer A needs to
 * be reset.
 * @param TALACT_ERR_RESET_JESD204DEFRAMERB Indicates that the API is not good
 * and the JESD204 Deframer B needs to
 * be reset.
 * @param TALACT_ERR_BBIC_LOG_ERROR Indicates that the API is not good and the
 * user should log this error and decide on a
 * recovery action.
 * @param TALACT_ERR_REDUCE_TXSAMPLE_PWR Indicates that the API is not good and
 * the TX sample power for the specified
 * channel needs to be reset.
 ******************************************************************************/
typedef enum {
	TALACT_NO_ACTION = 0,            /*!< API OK - NO ACTION REQUIRED */
	TALACT_WARN_RESET_LOG,           /*!< API OK - LOG Not working */
	TALACT_WARN_RERUN_TRCK_CAL,      /*!< API NG - RESET  SPEC TRACK CALS */
	TALACT_WARN_RESET_GPIO,          /*!< API OK - GPIO Not working */
	TALACT_ERR_CHECK_TIMER,          /*!< API OK - timer not working */
	TALACT_ERR_RESET_ARM,            /*!< API NG - RESET ARM ONLY */
	TALACT_ERR_RERUN_INIT_CALS,      /*!< API NG - RESET INIT CAL SEQ */
	TALACT_ERR_RESET_SPI,            /*!< API NG - SPI Not Working */
	TALACT_ERR_RESET_GPIO,           /*!< API NG - GPIO Not working */
	TALACT_ERR_CHECK_PARAM,          /*!< API OK - INVALID PARAM */
	TALACT_ERR_RESET_FULL,           /*!< API NG - FULL RESET REQUIRED */
	TALACT_ERR_RESET_JESD204FRAMERA, /*!< API NG - RESET the JESD204 FRAMER A */
	TALACT_ERR_RESET_JESD204FRAMERB, /*!< API NG - RESET the JESD204 FRAMER B */
	TALACT_ERR_RESET_JESD204DEFRAMERA, /*!< API NG - RESET the JESD204 DEFRAMER A */
	TALACT_ERR_RESET_JESD204DEFRAMERB, /*!< API NG - RESET the JESD204 DEFRAMER B */
	TALACT_ERR_BBIC_LOG_ERROR,           /*!< API NG - USER Should log this error and decide recovery action */
	TALACT_ERR_REDUCE_TXSAMPLE_PWR     /*!< API NG - RESET the TX Sample power for the Channel specified */
} talRecoveryActions_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_ERROR_TYPES_H_ */
