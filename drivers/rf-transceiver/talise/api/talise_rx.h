/**
 * \file talise_rx.h
 * \brief Contains Talise receive related function prototypes for
 *        talise_rx.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_RX_H_
#define TALISE_RX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_rx_types.h"
#include "talise_error_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief Use this function to set the gain table for Rx1, Rx2, or both channels
 * by providing a gain table array and specifying the number of gain
 * indexes. This function should be called after device initialization
 * and before running initialization calibrations if a custom gain table
 * is desired. Ensure that the Rx profile is valid and that the number of
 * gain indexes does not exceed the allowed range. The function performs
 * parameter validation and returns a recovery action code indicating the
 * result.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param gainTablePtr Pointer to an array of taliseRxGainTable_t structures
 * containing gain table values. Must not be null.
 * @param numGainIndexesInTable Number of gain indexes in the gainTablePtr
 * array. Must not exceed the maximum allowed
 * range.
 * @param rxChannel Enum value of type taliseRxChannels_t specifying the Rx
 * channel(s) to program. Must be a valid channel selection
 * (Rx1, Rx2, or Rx1 + Rx2).
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_NO_ACTION for success or specific error codes for
 * parameter issues.
 ******************************************************************************/
uint32_t TALISE_programRxGainTable(taliseDevice_t *device,
				   taliseRxGainTable_t *gainTablePtr, uint8_t numGainIndexesInTable,
				   taliseRxChannels_t rxChannel);

/***************************************************************************//**
 * @brief This function is used to program the gain table settings for the
 * specified observation receiver (ORx) channels, which can be ORx1,
 * ORx2, or both. It should be called after device initialization and
 * before running initialization calibrations if a custom gain table is
 * desired. The function requires a valid ORx profile to be set, and it
 * performs checks to ensure the gain table indices are within the
 * allowed range. It returns a status code indicating the success or
 * failure of the operation, with specific recovery actions suggested for
 * different error conditions.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param gainTablePtr Pointer to an array of ORx gain table structures. Must
 * not be null.
 * @param numGainIndexesInTable The number of indices in the gainTablePtr array.
 * Must not exceed the maximum allowed indices for
 * the gain table.
 * @param orxChannel Enum value of type taliseObsRxChannels_t specifying the ORx
 * channel(s) to program. Must be a valid channel selection
 * (ORx1, ORx2, or both).
 * @return Returns a uint32_t status code indicating the result of the
 * operation, with specific values for different recovery actions or
 * errors.
 ******************************************************************************/
uint32_t TALISE_programOrxGainTable(taliseDevice_t *device,
				    taliseOrxGainTable_t *gainTablePtr, uint8_t numGainIndexesInTable,
				    taliseObsRxChannels_t orxChannel);


/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */
/***************************************************************************//**
 * @brief This function allows the user to set the manual gain index for either
 * the Rx1 or Rx2 channel of a Talise device. It should be called after
 * the device has been initialized and when the user needs to manually
 * control the gain of a specific Rx channel. The function checks if the
 * provided gain index is within the valid range for the specified
 * channel and returns an error if it is not. It also verifies that the
 * Rx profile is valid before proceeding. This function is useful for
 * applications requiring precise manual control over the receiver gain
 * settings.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @param rxChannel Specifies the Rx channel to set the gain index for. Must be
 * either TAL_RX1 or TAL_RX2. Invalid values will result in an
 * error.
 * @param gainIndex The desired gain index for the specified Rx channel. Must be
 * within the range defined by the device's gain table for the
 * selected channel. Values outside this range will result in
 * an error.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for parameter errors, and other recovery actions as defined by the
 * API.
 ******************************************************************************/
uint32_t TALISE_setRxManualGain(taliseDevice_t *device,
				taliseRxChannels_t rxChannel, uint8_t gainIndex);

/***************************************************************************//**
 * @brief Use this function to retrieve the current gain index for a specified
 * Rx channel, either Rx1 or Rx2, after the device has been initialized
 * and the ARM processor has moved to the Radio ON state at least once.
 * This function is applicable in both AGC and MGC modes. It is important
 * to ensure that the device has entered a Receiver mode at least once
 * before calling this function, as it will return an error if there is
 * no gain index history available. The function will return a recovery
 * action code indicating the result of the operation.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rxChannel Specifies the Rx channel to read the gain index from. Must
 * be either TAL_RX1 or TAL_RX2.
 * @param rxGainIndex Pointer to a uint8_t where the gain index will be stored.
 * Can be null if the gain index is not needed.
 * @return Returns a uint32_t value indicating the recovery action. Possible
 * values include TALACT_NO_ACTION for success, TALACT_ERR_CHECK_PARAM
 * for invalid parameters, TALACT_ERR_RESET_SPI for SPI errors, and
 * TALACT_WARN_RESET_LOG for log reset warnings.
 ******************************************************************************/
uint32_t TALISE_getRxGain(taliseDevice_t *device, taliseRxChannels_t rxChannel,
			  uint8_t *rxGainIndex);

/***************************************************************************//**
 * @brief This function allows you to set the manual gain index for a specified
 * observation receiver (ORx) channel, either ORx1 or ORx2, on a Talise
 * device. It should be called after the device has been initialized and
 * when the observation receiver profile is valid. The function checks if
 * the provided gain index is within the valid range for the selected
 * channel and returns an error if it is not. It also handles any
 * necessary SPI communication errors internally, providing appropriate
 * recovery actions.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @param obsRxCh An enum of type taliseObsRxChannels_t that specifies the
 * observation receiver channel (TAL_ORX1 or TAL_ORX2) for which
 * the gain index is to be set. Invalid channel values will
 * result in an error.
 * @param gainIndex The desired manual gain index for the specified observation
 * receiver channel. It must be within the range defined by the
 * device's gain index limits for the selected channel. Values
 * outside this range will result in an error.
 * @return Returns a uint32_t value indicating the result of the operation, with
 * possible recovery actions such as TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_setObsRxManualGain(taliseDevice_t *device,
				   taliseObsRxChannels_t obsRxCh, uint8_t gainIndex);

/***************************************************************************//**
 * @brief Use this function to obtain the current manual gain index for a
 * specified observation receiver channel (ORx1 or ORx2) after device
 * initialization. It is important to ensure that the device has entered
 * a receiver mode at least once before calling this function, as gain
 * indices are only tracked after this point. The function will return an
 * error if the observation receiver profile is invalid or if an invalid
 * channel is specified. Additionally, the gain index is normalized to a
 * maximum index of 255.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param obsRxChannel Specifies the observation receiver channel to read the
 * gain index from. Must be either TAL_ORX1 or TAL_ORX2.
 * Invalid values result in an error.
 * @param obsRxGainIndex Pointer to a uint8_t where the read gain index will be
 * stored. Can be null if the gain index is not needed.
 * @return Returns a uint32_t indicating the status of the operation, with
 * possible values including success or specific error codes related to
 * parameter validation or SPI communication issues.
 ******************************************************************************/
uint32_t TALISE_getObsRxGain(taliseDevice_t *device,
			     taliseObsRxChannels_t obsRxChannel, uint8_t *obsRxGainIndex);

/***************************************************************************//**
 * @brief This function is used to set the gain control mode for the Talise
 * device's receiver, which defaults to Manual Gain Control (MGC) on
 * power-up. It should be called before initiating Rx operations to
 * ensure the desired gain control mode is configured. The function
 * supports Manual Gain Control (MGC) and Automatic Gain Control Slow
 * loop (AGCSLOW) modes, while Automatic Gain Control Fast Attack
 * (AGCFAST) and Hybrid modes are not supported and will result in an
 * error. Proper error handling is performed for invalid parameters.
 *
 * @param device Pointer to the Talise device data structure. Must not be null,
 * and the device should be properly initialized before calling
 * this function.
 * @param mode Desired gain mode to be configured, specified as an enumerated
 * type taliseGainMode_t. Valid values are TAL_MGC and TAL_AGCSLOW.
 * Invalid values or unsupported modes (TAL_AGCFAST, TAL_HYBRID)
 * will result in an error.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and other error codes
 * for specific issues encountered during execution.
 ******************************************************************************/
uint32_t TALISE_setRxGainControlMode(taliseDevice_t *device,
				     taliseGainMode_t mode);

/***************************************************************************//**
 * @brief This function configures the receive data format for the Talise
 * device, allowing the user to select between integer and floating point
 * formats. It provides options for configuring data formatting, such as
 * the number of exponent and significand bits for floating point
 * formats, and slicer bit positions for integer formats. The function
 * should be called after device initialization and before starting Rx
 * operations. It is important to ensure that the parameters provided are
 * within valid ranges to avoid errors.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rxDataFormat Pointer to the Rx data format configuration structure.
 * Must not be null and must contain valid configuration
 * values.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or
 * required recovery actions.
 ******************************************************************************/
uint32_t TALISE_setRxDataFormat(taliseDevice_t *device,
				taliseRxDataFormat_t *rxDataFormat);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the Rx
 * data path format, which was previously set using the corresponding
 * configuration function. It is essential to call this function to
 * verify or retrieve the current data format settings of the Rx path.
 * The function requires a valid device structure and a pointer to a data
 * format structure where the configuration will be stored. It is
 * important to ensure that the pointer to the data format structure is
 * not null before calling this function.
 *
 * @param device Pointer to the Talise device data structure. Must be a valid,
 * initialized device structure.
 * @param rxDataFormat Pointer to a taliseRxDataFormat_t structure where the Rx
 * data format configuration will be stored. Must not be
 * null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or error codes indicating specific issues such as invalid
 * parameters or SPI communication errors.
 ******************************************************************************/
uint32_t TALISE_getRxDataFormat(taliseDevice_t *device,
				taliseRxDataFormat_t *rxDataFormat);


/***************************************************************************//**
 * @brief This function retrieves the slicer position bits for both Rx1 and Rx2,
 * which are used to determine the scaling of Rx data samples. It should
 * be called after enabling the Rx gain compensation feature with the
 * integer gain slicer sample format. The function is not applicable when
 * using floating point formats, as the gain slicer is applied in the
 * floating point scaled sample. Ensure that the pointers provided for
 * the slicer positions are not null, as the function will return an
 * error if they are.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rx1SlicerPosition Pointer to a byte where the 3 slicer position bits
 * for Rx1 will be stored. Must not be null.
 * @param rx2SlicerPosition Pointer to a byte where the 3 slicer position bits
 * for Rx2 will be stored. Must not be null.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. The slicer positions are written to the provided
 * pointers if successful.
 ******************************************************************************/
uint32_t TALISE_getSlicerPosition(taliseDevice_t *device,
				  uint8_t *rx1SlicerPosition, uint8_t *rx2SlicerPosition);

/***************************************************************************//**
 * @brief This function allows configuring the GPIO input pins and step sizes
 * for manual gain control of the specified Rx channel. It is used to set
 * up the hardware for incrementing or decrementing the gain based on
 * external signals. The function should be called after device
 * initialization and before starting Rx operations. It validates the
 * input parameters and returns an error if any parameter is invalid,
 * ensuring that the configuration is only applied when all parameters
 * are correct.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rxChannel Specifies the Rx channel to configure, either Rx1 or Rx2.
 * Must be a valid taliseRxChannels_t enum value.
 * @param rxGainCtrlPin Pointer to a taliseRxGainCtrlPin_t structure containing
 * the configuration for manual Rx gain pin control. Must
 * not be null and must contain valid pin and step values.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or other
 * issues.
 ******************************************************************************/
uint32_t TALISE_setRxGainCtrlPin(taliseDevice_t *device,
				 taliseRxChannels_t rxChannel, taliseRxGainCtrlPin_t *rxGainCtrlPin);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration of the GPIO
 * pins and gain steps used for manual gain control of the specified Rx
 * channel. It should be called after the device has been initialized and
 * configured for manual gain control. The function requires a valid Rx
 * channel and a non-null pointer to a structure where the configuration
 * will be stored. If invalid parameters are provided, the function will
 * return an error code.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rxChannel Specifies the Rx channel (Rx1 or Rx2) for which the
 * configuration is to be retrieved. Must be a valid
 * taliseRxChannels_t value.
 * @param rxGainCtrlPin Pointer to a taliseRxGainCtrlPin_t structure where the
 * configuration will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, and various error codes for different failure conditions.
 ******************************************************************************/
uint32_t TALISE_getRxGainCtrlPin(taliseDevice_t *device,
				 taliseRxChannels_t rxChannel, taliseRxGainCtrlPin_t *rxGainCtrlPin);

/***************************************************************************//**
 * @brief This function is used to configure the dual-band settings for a Talise
 * device, allowing it to handle two adjacent frequency bands. It should
 * be called with a properly initialized device and configuration
 * structure. The function performs various checks on the input
 * parameters to ensure valid frequency settings and band separations. If
 * any parameter is invalid, it returns an appropriate error code. This
 * function is typically used in scenarios where dual-band operation is
 * required, and it must be called after the device has been initialized
 * but before it is used for receiving signals.
 *
 * @param device Pointer to a taliseDevice_t structure representing the Talise
 * device. Must not be null. The caller retains ownership.
 * @param init Pointer to a taliseInit_t structure containing the NCO Shifter
 * Configuration settings. Must not be null. The caller retains
 * ownership.
 * @return Returns a talRecoveryActions_t value indicating the result of the
 * operation. Possible values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and
 * TALACT_ERR_RESET_SPI for SPI reset required.
 ******************************************************************************/
talRecoveryActions_t talSetDualBandSettings(taliseDevice_t *device,
		taliseInit_t *init);

/***************************************************************************//**
 * @brief This function is used to program the gain table settings for the
 * Dualband LNA for specified receiver channels (Rx1, Rx2, or both). It
 * should be called after device initialization and before running
 * initial calibrations if a custom gain table is needed. The function
 * checks for valid receiver profiles and ensures the number of gain
 * table entries does not exceed the maximum allowed. It also verifies
 * that the gain table pointer is not null. The function returns a status
 * code indicating success or the type of error encountered.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized before calling this
 * function.
 * @param gainTablePtr Pointer to an array of taliseDualBandLnaGainTable_t
 * structures containing the gain table values. Must not be
 * null.
 * @param numGainIndexesInTable The number of entries in the gainTablePtr array.
 * Must be between 1 and 4, inclusive.
 * @param rxChannel Enum value of type taliseRxChannels_t specifying the
 * receiver channel(s) to program (Rx1, Rx2, or both). Must be
 * a valid channel selection.
 * @return Returns a uint32_t status code indicating the result of the
 * operation: TALACT_NO_ACTION for success, or an error code for
 * failure.
 ******************************************************************************/
uint32_t TALISE_programDualBandLnaGainTable(taliseDevice_t *device,
		taliseDualBandLnaGainTable_t *gainTablePtr, uint8_t numGainIndexesInTable,
		taliseRxChannels_t rxChannel);

/***************************************************************************//**
 * @brief This function sets up the Numerically Controlled Oscillator (NCO)
 * shifter for the Talise device based on the provided initialization
 * settings. It should be called after the device has been initialized
 * and before any operations that depend on the NCO configuration. The
 * function checks the validity of the input parameters and ensures that
 * the signal passband edges are within the specified corners. It also
 * verifies that the NCO shifts are within acceptable ranges. If any
 * parameter is invalid, the function returns an appropriate error code
 * indicating the type of recovery action required.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * The caller retains ownership.
 * @param init Pointer to the initialization structure containing NCO shifter
 * configuration settings. Must not be null. The caller retains
 * ownership.
 * @return Returns a talRecoveryActions_t value indicating the result of the
 * operation. Possible values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for parameter errors, and TALACT_ERR_RESET_SPI
 * for SPI reset required.
 ******************************************************************************/
talRecoveryActions_t talSetupNcoShifter(taliseDevice_t *device,
					taliseInit_t *init);

/***************************************************************************//**
 * @brief This function configures the 3.3V GPIO pins to output the external
 * control word corresponding to the current gain index for the specified
 * Rx channels. It can be used to enable or disable the output of these
 * control words on the GPIOs. The function should be called after the Rx
 * channels have been initialized. It checks for valid channel selection
 * and ensures that the GPIO pins are not already in use for other
 * purposes. If the specified conditions are not met, the function
 * returns an error.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * @param rxChannel Enum value of type taliseRxChannels_t specifying the Rx
 * channel(s) (Rx1, Rx2, or both) for which the external
 * control word should be output. Must be a valid channel.
 * @param enable3p3vGpios A uint8_t value where 1 enables and 0 disables the
 * output of external control words on the GPIOs.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation, with specific error codes for invalid parameters or other
 * issues.
 ******************************************************************************/
uint32_t TALISE_setGainTableExtCtrlPins(taliseDevice_t *device,
					taliseRxChannels_t rxChannel, uint8_t enable3p3vGpios);

/***************************************************************************//**
 * @brief Use this function to obtain the decimated power measurement in milli-
 * dBFS for a specified Rx channel, either Rx1 or Rx2. It is essential to
 * ensure that the device is properly initialized and the Rx profile is
 * valid before calling this function. The function requires a valid
 * pointer to store the power measurement result. If the provided channel
 * is invalid or the device is not correctly configured, the function
 * will return an error code. This function is useful for monitoring the
 * power level in the Rx digital data path.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized.
 * @param rxChannel Specifies the Rx channel to measure. Must be either TAL_RX1
 * or TAL_RX2. Invalid values will result in an error.
 * @param rxDecPower_mdBFS Pointer to a uint16_t where the measured decimated
 * power in milli-dBFS will be stored. Must not be null.
 * @return Returns a uint32_t error code indicating the success or failure of
 * the operation. The power measurement is stored in the provided
 * pointer if successful.
 ******************************************************************************/
uint32_t TALISE_getRxDecPower(taliseDevice_t *device,
			      taliseRxChannels_t rxChannel, uint16_t *rxDecPower_mdBFS);

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

#ifdef __cplusplus
}
#endif

#endif /* TALISE_RX_H_ */
