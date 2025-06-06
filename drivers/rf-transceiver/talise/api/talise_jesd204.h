/**
 * \file talise_jesd204.h
 * \brief Contains Talise JESD204b data path related function prototypes for
 *        talise_jesd204.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_JESD204_H_
#define TALISE_JESD204_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_jesd204_types.h"
#include "talise_tx_types.h"
#include "talise_error_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function sets up the JESD204B serializers by configuring the
 * serializer lanes shared between two framers based on the provided
 * initialization settings. It adjusts the serializer lane clock
 * frequency, powers up the necessary serializer lanes, resets them, and
 * configures pre-emphasis, amplitude, and lane polarity inversion
 * settings. The function ensures that the serializer lane clock is set
 * to the faster of the Rx and ORx framer rates, allowing for automatic
 * oversampling or bit repetition to handle clock rate differences. It is
 * essential to call this function after initializing the device and
 * before starting data transmission.
 *
 * @param device Pointer to the taliseDevice_t structure containing device
 * settings. Must not be null.
 * @param init Pointer to the taliseInit_t structure containing initialization
 * settings. Must not be null and should be properly configured with
 * valid framer and serializer settings.
 * @return Returns a uint32_t value indicating the result of the operation:
 * TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM for parameter
 * errors, or TALACT_ERR_RESET_SPI for SPI reset required.
 ******************************************************************************/
uint32_t TALISE_setupSerializers(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief This function is automatically invoked by TALISE_initialize() when a
 * valid Tx profile is present. It configures the deserializer lanes,
 * sets the deserializer clocks, and applies PN inversion and EQ settings
 * based on the provided device and initialization structures. It is
 * essential for setting up the JESD204B data path for transmission. The
 * function expects valid configuration parameters and will return
 * specific error codes if any parameter is invalid or if a reset is
 * required. It should not be called directly by the user.
 *
 * @param device A pointer to the taliseDevice_t structure containing the device
 * settings. Must not be null.
 * @param init A pointer to the taliseInit_t structure containing the
 * initialization settings. Must not be null and should contain
 * valid JESD204B configuration parameters.
 * @return Returns a uint32_t value indicating the recovery action required:
 * TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM for parameter
 * errors, TALACT_ERR_RESET_SPI for SPI reset required, or
 * TALACT_WARN_RESET_LOG for log reset warning.
 ******************************************************************************/
uint32_t TALISE_setupDeserializers(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief This function sets up the JESD204B framer for the specified device and
 * initialization settings. It configures the framer based on the
 * selected framer (A or B) and the settings provided in the
 * initialization structure. The function handles various parameter
 * checks and will disable the framer if the number of converters (M) is
 * zero or if no lanes are enabled. It is important to ensure that the
 * initialization settings are correctly configured before calling this
 * function, as invalid parameters will result in error codes and
 * recommended recovery actions. This function is typically called during
 * the device initialization process and is not intended to be called
 * directly by the user.
 *
 * @param device A pointer to the taliseDevice_t structure containing the device
 * settings. Must not be null.
 * @param init A pointer to the taliseInit_t structure containing the
 * initialization settings for the device. Must not be null and
 * should be properly initialized with valid settings.
 * @param framerSel An enumeration of type taliseFramerSel_t indicating which
 * framer to configure (TAL_FRAMER_A or TAL_FRAMER_B). Invalid
 * values will result in an error.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for parameter errors, and TALACT_ERR_RESET_SPI
 * for SPI reset requirements.
 ******************************************************************************/
uint32_t TALISE_setupJesd204bFramer(taliseDevice_t *device, taliseInit_t *init,
				    taliseFramerSel_t framerSel);

/***************************************************************************//**
 * @brief This function updates the JESD204B deframer settings based on the
 * provided device and initialization structures. It is typically called
 * automatically during the device initialization process if the Tx
 * profile is valid. The function configures various parameters such as
 * lane enablement, clock settings, and synchronization options. It
 * requires a valid Tx profile and appropriate deframer selection. The
 * function handles invalid parameters by returning specific error codes,
 * ensuring robust error checking and recovery.
 *
 * @param device Pointer to the device settings data structure. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @param init Pointer to the device initialization settings data structure.
 * Must not be null and should contain valid JESD204B settings.
 * @param deframerSel Specifies the deframer to configure. Must be a valid
 * deframer selection (e.g., TAL_DEFRAMER_A or
 * TAL_DEFRAMER_B).
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_ERR_CHECK_PARAM for parameter
 * errors, TALACT_ERR_RESET_SPI for SPI reset requirements, and
 * TALACT_NO_ACTION for successful completion.
 ******************************************************************************/
uint32_t TALISE_setupJesd204bDeframer(taliseDevice_t *device,
				      taliseInit_t *init, taliseDeframerSel_t deframerSel);

/***************************************************************************//**
 * @brief This function is used to enable or disable the JESD204B framer link
 * for a specified framer or combination of framers. It is typically not
 * necessary to call this function unless there is a need to reset the
 * link. The function can be called at any time after the device has been
 * initialized. It returns a status code indicating the success of the
 * operation or any required recovery actions.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param framerSel Specifies the framer(s) to be enabled or disabled. Valid
 * values are TAL_FRAMER_A, TAL_FRAMER_B, or
 * TAL_FRAMER_A_AND_B. Invalid values result in an error.
 * @param enable A uint8_t value where 0 disables and any non-zero value enables
 * the selected framer link.
 * @return Returns a uint32_t value indicating the result of the operation,
 * which can be a success or a specific recovery action code.
 ******************************************************************************/
uint32_t TALISE_enableFramerLink(taliseDevice_t *device,
				 taliseFramerSel_t framerSel, uint8_t enable);

/***************************************************************************//**
 * @brief This function allows the user to enable or disable the JESD204B
 * deframer link for a specified deframer. It is typically used when a
 * link reset is necessary. The function can be called at any time after
 * the device has been initialized. When enabling the link, ensure that
 * valid serializer data is being sent to lock the Talise CDR (recovered
 * clock).
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null.
 * @param deframerSel Specifies the deframer to be enabled or disabled. Valid
 * values are TAL_DEFRAMER_A or TAL_DEFRAMER_B.
 * @param enable A uint8_t value where 0 disables the selected deframer and any
 * non-zero value enables the selected deframer link.
 * @return Returns a uint32_t value indicating the recovery action.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_enableDeframerLink(taliseDevice_t *device,
				   taliseDeframerSel_t deframerSel, uint8_t enable);

/***************************************************************************//**
 * @brief Use this function to control the gating of the external SYSREF signal
 * to the specified JESD204B framer(s) on the transceiver. This is
 * typically used to ensure that the SYSREF signal is only active when
 * the framer is correctly configured and ready to receive it. The
 * function should be called after the device has been initialized and
 * the JESD204B framer is enabled. It is important to select the correct
 * framer(s) and ensure that the enable parameter is set appropriately to
 * either allow or block the SYSREF signal.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param framerSel Specifies which framer(s) to enable or disable the SYSREF
 * input for. Valid values are TAL_FRAMER_A, TAL_FRAMER_B, or
 * TAL_FRAMER_A_AND_B.
 * @param enable A non-zero value enables the SYSREF signal to the framer, while
 * zero disables it.
 * @return Returns a uint32_t value indicating the result of the operation,
 * where TALACT_NO_ACTION indicates success, and other values indicate
 * specific error conditions or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_enableSysrefToFramer(taliseDevice_t *device,
				     taliseFramerSel_t framerSel, uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to control the gating of the external SYSREF
 * signal to the specified JESD204B deframers on the transceiver. It
 * should be called after the device has been initialized and the
 * JESD204B deframer is enabled. The function allows the user to enable
 * or disable the SYSREF signal for one or both deframers, ensuring that
 * the SYSREF signal is only active when the deframer is correctly
 * configured. This is important for maintaining synchronization and
 * avoiding invalid SYSREF pulses.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param deframerSel Specifies which deframer(s) to enable or disable the
 * SYSREF input for. Valid values are TAL_DEFRAMER_A,
 * TAL_DEFRAMER_B, or TAL_DEFRAMER_A_AND_B.
 * @param enable A uint8_t value where '1' enables the SYSREF to the deframer
 * and '0' disables it. Any non-zero value is treated as '1'.
 * @return Returns a uint32_t value indicating the result of the operation,
 * which can be a recovery action such as TALACT_NO_ACTION for success
 * or an error code for failure.
 ******************************************************************************/
uint32_t TALISE_enableSysrefToDeframer(taliseDevice_t *device,
				       taliseDeframerSel_t deframerSel, uint8_t enable);

/***************************************************************************//**
 * @brief Use this function to obtain the current status of a specified JESD204B
 * framer in a Talise device. It is essential that the Rx JESD204B
 * link(s) are configured and operational before calling this function.
 * The function provides a status byte that includes various flags
 * indicating the framer's current state, such as SYSREF reception and
 * configuration validity. This function should be used to monitor the
 * framer's operational status and diagnose potential issues.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param framerSel Specifies which framer's status to read. Valid values are
 * TAL_FRAMER_A or TAL_FRAMER_B.
 * @param framerStatus A pointer to a uint8_t where the framer status byte will
 * be stored. Must not be null.
 * @return Returns a uint32_t value indicating the result of the operation.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_readFramerStatus(taliseDevice_t *device,
				 taliseFramerSel_t framerSel, uint8_t *framerStatus);

/***************************************************************************//**
 * @brief Use this function to obtain the current status of a specified JESD204B
 * deframer. It is essential that the Tx JESD204B link(s) are configured
 * and operational before calling this function. The function provides a
 * status word that includes various flags indicating the state of the
 * deframer, such as configuration validity, checksum validity, and
 * synchronization errors. This function should be used to monitor the
 * deframer's operational status and diagnose potential issues.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param deframerSel Specifies which deframer to read the status from. Must be
 * a valid taliseDeframerSel_t value, such as TAL_DEFRAMER_A
 * or TAL_DEFRAMER_B.
 * @param deframerStatus A pointer to a uint16_t variable where the deframer
 * status will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the recovery action status.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * or other values indicating specific error conditions.
 ******************************************************************************/
uint32_t TALISE_readDeframerStatus(taliseDevice_t *device,
				   taliseDeframerSel_t deframerSel, uint16_t *deframerStatus);

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to set up the DAC sample crossbar for a
 * specified transmission channel, allowing the mapping of deframer
 * outputs to specific DAC channel I or Q converter inputs. It should be
 * called during the JESD204B initialization process. The function
 * requires valid channel selection and DAC crossbar settings, and it
 * returns a status code indicating the success or failure of the
 * operation. Invalid channel selections or DAC crossbar settings will
 * result in an error.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param channelSel An enumerated value of type taliseTxChannels_t specifying
 * the transmission channel to configure. Only TAL_TX1 or
 * TAL_TX2 are valid.
 * @param dacXbar A taliseDacSampleXbar_t structure specifying the DAC crossbar
 * settings for mapping deframer outputs to DAC inputs. Must
 * contain valid settings for both dacChanI and dacChanQ.
 * @return Returns a uint32_t value indicating the result of the operation,
 * where TALACT_NO_ACTION indicates success and other values indicate
 * specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_setupDacSampleXbar(taliseDevice_t *device,
				   taliseTxChannels_t channelSel, taliseDacSampleXbar_t dacXbar);

/***************************************************************************//**
 * @brief This function is used to set up the ADC sample crossbar, which maps
 * Rx1/Rx2/ORx1/ORx2 'I/Q' data to the selected JESD204B framer's
 * converter. It should be called during the JESD204B initialization
 * process. The function requires a valid device structure and a framer
 * selection, which must be either TAL_FRAMER_A or TAL_FRAMER_B. The
 * adcXbar parameter specifies the crossbar settings for the eight
 * converters. If an invalid framer selection is provided, the function
 * will return an error. The function returns a status code indicating
 * success or the type of error encountered, along with a recommended
 * recovery action.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param framerSel An enumerated value of type taliseFramerSel_t indicating
 * which framer to configure. Valid values are TAL_FRAMER_A or
 * TAL_FRAMER_B. Invalid values result in an error.
 * @param adcXbar A taliseAdcSampleXbar_t structure containing the crossbar
 * settings for the eight converters. Each setting must be valid
 * as per taliseAdcSampleXbarSelect_t enumeration.
 * @return Returns a uint32_t value representing the status of the operation.
 * TALACT_NO_ACTION indicates success, while other values indicate an
 * error with a recommended recovery action.
 ******************************************************************************/
uint32_t TALISE_setupAdcSampleXbar(taliseDevice_t *device,
				   taliseFramerSel_t framerSel, taliseAdcSampleXbar_t adcXbar);
/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to enable the generation of test data on the
 * specified JESD204B framer for debugging purposes. It allows the user
 * to select the framer, the type of test data, and the point in the data
 * path where the test data should be injected. This function is
 * typically used for testing and debugging the JESD204B lanes and should
 * be called after the device has been initialized. The function checks
 * for valid input parameters and returns an error if any parameter is
 * invalid.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param framerSelect Specifies the framer(s) to enable test data on. Valid
 * values are TAL_FRAMER_A, TAL_FRAMER_B, or
 * TAL_FRAMER_A_AND_B.
 * @param testDataSource Specifies the source of the test data. Must be a valid
 * taliseFramerDataSource_t value, with a maximum value of
 * TAL_FTD_RAMP.
 * @param injectPoint Specifies the point in the data path to inject the test
 * data. Must be a valid taliseFramerInjectPoint_t value,
 * with a maximum value of TAL_FTD_POST_LANEMAP.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. TALACT_NO_ACTION indicates success, while other values
 * indicate specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_enableFramerTestData(taliseDevice_t *device,
				     taliseFramerSel_t framerSelect, taliseFramerDataSource_t testDataSource,
				     taliseFramerInjectPoint_t injectPoint);

/***************************************************************************//**
 * @brief This function is used for debugging the Rx JESD204B lanes by injecting
 * an error into the framer test data. It should be called after the
 * framer test data is enabled. The function inverts the test data for
 * the selected framer, which can be either TAL_FRAMER_A or TAL_FRAMER_B.
 * The function does not support TAL_FRAMER_A_AND_B. It is important to
 * ensure that the framer test data is activated before calling this
 * function, as Rx data transmission on the JESD204B link(s) is not
 * possible when the framer test data is activated.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param framerSelect Selects the desired framer to inject the error into.
 * Valid values are TAL_FRAMER_A or TAL_FRAMER_B.
 * TAL_FRAMER_A_AND_B is not supported. If an invalid value
 * is provided, the function will handle it as an error.
 * @return Returns a uint32_t value indicating the recovery action.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_injectFramerTestDataError(taliseDevice_t *device,
		taliseFramerSel_t framerSelect);

/***************************************************************************//**
 * @brief This function is used to configure and enable the Pseudo-Random Binary
 * Sequence (PRBS) checker for the JESD204B deframer, which is useful for
 * debugging the Tx JESD204B lanes. It can be called any time after the
 * device has been initialized. The function requires valid parameters
 * for the polynomial order and checker location, and it will return an
 * error if these parameters are outside their valid ranges. The function
 * also clears the deframer PRBS counters before enabling the checker.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param polyOrder Specifies the PRBS polynomial order. Valid values are
 * TAL_PRBS_DISABLE, TAL_PRBS7, TAL_PRBS15, and TAL_PRBS31.
 * Invalid values will result in an error.
 * @param checkerLocation Specifies the location to check the PRBS. Valid values
 * are TAL_PRBSCHECK_LANEDATA and
 * TAL_PRBSCHECK_SAMPLEDATA. Invalid values will result
 * in an error.
 * @return Returns a uint32_t value indicating the recovery action.
 * TALACT_NO_ACTION indicates success, while other values indicate
 * specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_enableDeframerPrbsChecker(taliseDevice_t *device,
		taliseDeframerPrbsOrder_t polyOrder, taliseDefPrbsCheckLoc_t checkerLocation);

/***************************************************************************//**
 * @brief Use this function to reset the PRBS error counters associated with the
 * deframer in a Talise device. This is typically done to clear any
 * accumulated error counts before starting a new test or measurement. It
 * is important to ensure that the Tx JESD204B link(s) are configured and
 * running before calling this function. The function returns a status
 * code indicating the success of the operation or any required recovery
 * actions.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. This parameter must not be null, and the
 * caller retains ownership.
 * @return Returns a uint32_t value representing the recovery action status.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_clearDeframerPrbsCounters(taliseDevice_t *device);

/***************************************************************************//**
 * @brief Use this function to retrieve the PRBS error count and inversion
 * status for a specific lane of the JESD204B deframer. It is essential
 * to ensure that the Tx JESD204B link is configured and running before
 * calling this function. The function requires valid pointers for the
 * error count and inversion status outputs, and it will return an error
 * if these pointers are null. The lane parameter must be within the
 * valid range of 0 to 3, and the function will handle invalid lane
 * values by returning an error.
 *
 * @param device Pointer to the taliseDevice_t structure representing the device
 * settings. Must not be null.
 * @param lane Specifies the lane for which to read the PRBS counters. Valid
 * values are 0 to 3. An invalid value will result in an error.
 * @param prbsErrorCount Pointer to a uint8_t variable where the function will
 * store the 8-bit PRBS error count. Must not be null.
 * @param prbsInvertedStatus Pointer to a uint8_t variable where the function
 * will store the PRBS inversion status bitmask. Must
 * not be null.
 * @return Returns a uint32_t value indicating the status of the operation, with
 * specific values representing different recovery actions or errors.
 ******************************************************************************/
uint32_t TALISE_readDeframerPrbsCounters(taliseDevice_t *device, uint8_t lane,
		uint8_t *prbsErrorCount, uint8_t *prbsInvertedStatus);

/***************************************************************************//**
 * @brief This function is used to compare the received Lane0 ILAS configuration
 * with the current deframer configuration and returns a 32-bit mask
 * indicating any mismatched values. It is essential to call this
 * function when the Rx JESD204B link(s) are configured and running. The
 * function can also return the actual ILAS and deframer configuration
 * values if pointers to the respective structures are provided. If these
 * pointers are null, the function will not return configuration data for
 * those parameters.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param deframerSelect Enum indicating which deframer to address. Must be a
 * valid deframer selection.
 * @param mismatch Pointer to a uint32_t variable where the ILAS match status
 * will be reported. Must not be null.
 * @param dfrmCfg Pointer to a taliseJesd204bLane0Config_t structure to receive
 * the deframer configuration settings. If null, no data is
 * returned.
 * @param dfrmIlas Pointer to a taliseJesd204bLane0Config_t structure to receive
 * the received Lane0 ILAS settings. If null, no data is
 * returned.
 * @return Returns a uint32_t value indicating the recovery action.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_getDfrmIlasMismatch(taliseDevice_t *device,
				    taliseDeframerSel_t deframerSelect, uint32_t *mismatch,
				    taliseJesd204bLane0Config_t *dfrmCfg, taliseJesd204bLane0Config_t *dfrmIlas);

/***************************************************************************//**
 * @brief Use this function to obtain the current interrupt mask settings for a
 * specified deframer in the Talise device. This function is useful for
 * understanding which interrupts are currently enabled for the deframer.
 * It must be called after the JESD204B link has been initialized. Ensure
 * that the `irqMask` pointer is not null before calling this function,
 * as a null pointer will result in an error. The function will return a
 * recovery action code indicating the success or failure of the
 * operation.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. The caller retains ownership and it must not be null.
 * @param deframerSelect An enumerated value of type taliseDeframerSel_t
 * indicating which deframer to query. Valid values are
 * TAL_DEFRAMER_A and TAL_DEFRAMER_B.
 * @param irqMask A pointer to a uint16_t variable where the IRQ mask will be
 * stored. Must not be null.
 * @return Returns a uint32_t value representing the recovery action code.
 * TALACT_NO_ACTION indicates success, while other values indicate
 * specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_getDfrmIrqMask(taliseDevice_t *device,
			       taliseDeframerSel_t deframerSelect, uint16_t *irqMask);

/***************************************************************************//**
 * @brief Use this function to configure the interrupt mask for a specific
 * deframer in the JESD204B interface. It allows enabling or disabling
 * specific interrupt sources by writing a mask value to the deframer's
 * IRQ clear register. This function should be called after the JESD204B
 * link has been initialized. Ensure that the deframerSelect parameter is
 * valid, as invalid values will result in an error. The function returns
 * a status code indicating the success or failure of the operation.
 *
 * @param device Pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param deframerSelect Specifies which deframer to configure. Must be either
 * TAL_DEFRAMER_A or TAL_DEFRAMER_B. Invalid values will
 * trigger an error.
 * @param irqMask 16-bit mask value to be written to the deframer IRQ clear
 * register. This is not a read-modify-write operation.
 * @return Returns a uint32_t status code indicating the result of the
 * operation. TALACT_NO_ACTION indicates success, while other values
 * indicate specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_setDfrmIrqMask(taliseDevice_t *device,
			       taliseDeframerSel_t deframerSelect, uint16_t irqMask);

/***************************************************************************//**
 * @brief Use this function to clear all interrupt requests for a specified
 * JESD204B deframer after a general purpose deframer IRQ asserts. It is
 * essential to call this function after initializing the JESD204B link
 * to ensure that any pending interrupts are cleared. The function
 * requires a valid deframer selection and will return an error if the
 * selection is invalid. It is important to handle the return value to
 * determine if any recovery actions are needed.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null, and the caller retains
 * ownership.
 * @param deframerSelect An enumerated value of type taliseDeframerSel_t
 * indicating which deframer to clear interrupts for.
 * Valid values are TAL_DEFRAMER_A and TAL_DEFRAMER_B. If
 * an invalid value is provided, the function returns an
 * error.
 * @return Returns a uint32_t value indicating the result of the operation.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions are needed.
 ******************************************************************************/
uint32_t TALISE_clearDfrmIrq(taliseDevice_t *device,
			     taliseDeframerSel_t deframerSelect);

/***************************************************************************//**
 * @brief Use this function to determine the source of an interrupt for a
 * specified deframer after a general purpose deframer IRQ asserts. It
 * should be called any time after the JESD204 link initialization. The
 * function requires valid pointers for the device and irqSourceValue
 * parameters, and a valid deframerSelect value. If the irqSourceValue
 * pointer is null or deframerSelect is invalid, the function will handle
 * these as errors.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param deframerSelect An enumerated value of type taliseDeframerSel_t
 * indicating which deframer to interrogate. Valid values
 * are TAL_DEFRAMER_A and TAL_DEFRAMER_B.
 * @param irqSourceValue A pointer to a uint16_t variable where the IRQ source
 * status will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the status of the operation, with
 * specific values representing different recovery actions or success.
 ******************************************************************************/
uint32_t TALISE_getDfrmIrqSource(taliseDevice_t *device,
				 taliseDeframerSel_t deframerSelect, uint16_t *irqSourceValue);

/***************************************************************************//**
 * @brief This function checks for lane count errors in the specified deframer
 * and updates the provided error mask to indicate which deframer inputs
 * are reporting errors. It should be called when a deframer is suspected
 * of having lane count errors. The function requires a valid device
 * pointer and a deframer selection. It will return an error if the
 * deframer selection is invalid or if the error mask pointer is null.
 * The function also clears the error counters after checking.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param deframer An enumerated value of type taliseDeframerSel_t indicating
 * which deframer to check. Must be TAL_DEFRAMER_A or
 * TAL_DEFRAMER_B.
 * @param deframerInputsMask A pointer to an int32_t where the function will
 * store the error mask indicating which deframer
 * inputs have errors. Must not be null.
 * @return Returns a talRecoveryActions_t value indicating the recovery action
 * required. Updates the deframerInputsMask with the error status.
 ******************************************************************************/
talRecoveryActions_t talFindDfrmrLaneCntErr(taliseDevice_t *device,
		taliseDeframerSel_t deframer, int32_t *deframerInputsMask);

/***************************************************************************//**
 * @brief This function is used to determine which deframer lanes are reporting
 * errors by reading error flags from a specified deframer register
 * address. It is typically called when a deframer is reporting an error
 * interrupt. The function requires a valid pointer to store the deframer
 * inputs mask, which indicates the lanes with errors. If the pointer is
 * null, the function returns an error. The function reads the error
 * flags and updates the mask based on the specified nibble, either upper
 * or lower, to identify the lanes with errors.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param dfrmErrAddress The address of the deframer register to read error
 * flags from. Must be a valid register address.
 * @param nibbleToUse Specifies which nibble of the error flags to use (0 for
 * lower nibble, 1 for upper nibble). Values outside this
 * range are treated as 1.
 * @param deframerInputsMask A pointer to an int32_t where the function will
 * store the bitmask indicating which deframer inputs
 * are reporting errors. Must not be null.
 * @return Returns a talRecoveryActions_t value indicating the recovery action
 * required, if any. Updates the deframerInputsMask with the error
 * status of the lanes.
 ******************************************************************************/
talRecoveryActions_t talFindDfrmrLaneErr(taliseDevice_t *device,
		uint32_t dfrmErrAddress, uint8_t nibbleToUse, int32_t *deframerInputsMask);

/***************************************************************************//**
 * @brief This function is used to manually toggle the SYNCB signal internal to
 * the JESD204B framer, which is typically not necessary unless
 * instructed by an ADI applications engineer. It is intended for use in
 * systems that do not conform to the JESD204B specification, allowing
 * the framer to transition states without external SYNCB control. This
 * function should be called during device initialization after enabling
 * the framer link and sending a SYSREF pulse.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param framerSel Enum value selecting the framer to apply the SYNCB toggle
 * to. Valid values are TAL_FRAMER_A, TAL_FRAMER_B, or
 * TAL_FRAMER_A_AND_B. Invalid values result in a parameter
 * error.
 * @return Returns a uint32_t value indicating the recovery action.
 * TALACT_NO_ACTION indicates success, while other values suggest
 * specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_framerSyncbToggle(taliseDevice_t *device,
				  taliseFramerSel_t framerSel);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_JESD204_H_ */
