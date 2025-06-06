/**
 * \file talise_agc.h
 * \brief Contains Talise API AGC function prototypes for talise_agc.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_AGC_H_
#define TALISE_AGC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_agc_types.h"

/***************************************************************************//**
 * @brief This function is used to set up the Rx AGC registers of the device
 * using the provided configuration structure. It must be called after
 * the device has been initialized and only if AGC is to be used. The
 * function checks the validity of the configuration parameters and
 * writes them to the device registers. It handles various parameter
 * constraints and ensures that the AGC settings are within valid ranges.
 * The function also performs necessary error handling and returns
 * appropriate recovery actions if any parameter is invalid or if there
 * is an issue with the SPI communication.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param rxAgcCtrl Pointer to the AGC configuration structure containing
 * settings. Must be initialized with valid settings and must
 * not be null.
 * @return Returns a uint32_t value indicating the recovery action required:
 * TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI,
 * or TALACT_NO_ACTION if successful.
 ******************************************************************************/
uint32_t TALISE_setupRxAgc(taliseDevice_t *device, taliseAgcCfg_t *rxAgcCtrl);

/***************************************************************************//**
 * @brief Use this function to obtain the current settings of the Automatic Gain
 * Control (AGC) registers from a Talise device. This function should be
 * called only when AGC is active. It updates the provided `agcCtrl`
 * structure with the current AGC configuration, including gain indices,
 * attack delays, and other AGC parameters. Ensure that the `agcCtrl`
 * structure is properly initialized before calling this function. The
 * function returns a status code indicating success or the type of error
 * encountered, such as parameter errors or SPI communication issues.
 *
 * @param device Pointer to a `taliseDevice_t` structure representing the Talise
 * device. Must not be null.
 * @param agcCtrl Pointer to a `taliseAgcCfg_t` structure where the AGC control
 * register values will be stored. Must not be null. The
 * structure should be initialized before calling this function.
 * @return Returns a `uint32_t` status code indicating the result of the
 * operation: `TALACT_NO_ACTION` for success, or other codes for
 * specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_getAgcCtrlRegisters(taliseDevice_t *device,
				    taliseAgcCfg_t *agcCtrl);

/***************************************************************************//**
 * @brief This function is used to obtain the current values of the AGC peak-
 * related registers from a Talise device. It should be called only when
 * the AGC is active. The function updates the provided `agcPeak`
 * structure with the retrieved register values. It is important to
 * ensure that the `agcPeak` pointer is not null before calling this
 * function, as a null pointer will result in an error. The function
 * returns a status code indicating the success of the operation or any
 * required recovery actions.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * The caller retains ownership.
 * @param agcPeak Pointer to the Talise AGC peak settings data structure. Must
 * not be null. The function updates this structure with the
 * current AGC peak register values.
 * @return Returns a status code indicating the success of the operation or any
 * required recovery actions, such as TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_getAgcPeakRegisters(taliseDevice_t *device,
				    taliseAgcPeak_t *agcPeak);

/***************************************************************************//**
 * @brief This function is used to obtain the current values of the AGC power
 * measurement registers from a Talise device. It should be called only
 * when the AGC is active. The function updates the provided `agcPower`
 * structure with the register values, which include various power
 * measurement settings and thresholds. It is important to ensure that
 * the `agcPower` pointer is not null before calling this function, as a
 * null pointer will result in an error. The function returns a status
 * code indicating the success of the operation or any required recovery
 * actions.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * The caller retains ownership.
 * @param agcPower Pointer to a `taliseAgcPower_t` structure where the AGC power
 * register values will be stored. Must not be null. The
 * function will update this structure with the current register
 * values.
 * @return Returns a `uint32_t` status code indicating the result of the
 * operation. Possible values include `TALACT_NO_ACTION` for success,
 * `TALACT_WARN_RESET_LOG` for a log reset warning,
 * `TALACT_ERR_CHECK_PARAM` for a parameter error, and
 * `TALACT_ERR_RESET_SPI` for an SPI reset requirement.
 ******************************************************************************/
uint32_t TALISE_getAgcPowerRegisters(taliseDevice_t *device,
				     taliseAgcPower_t *agcPower);

/***************************************************************************//**
 * @brief This function sets up the dual-band mode for the Rx AGC on a Talise
 * device, allowing for control over external LNAs through GPIOs. It
 * should be called only when AGC is active and the device is initialized
 * in dual-band mode. The function can be used in a radio-off state and
 * requires valid configuration settings in the provided data structure.
 * It performs parameter validation and returns specific recovery actions
 * based on the success or failure of the operation.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param rxChannel Enum of type taliseRxChannels_t to select either Rx1, Rx2,
 * or both Rx1 and Rx2 for programming the dual-band AGC. Must
 * not be TAL_RXOFF.
 * @param rxAgcCtrlDualBand Pointer to the Talise AGC dual-band settings data
 * structure. Must not be null. The structure must have
 * valid settings, including power margin and gain
 * table indices.
 * @return Returns a uint32_t indicating the recovery action required:
 * TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI,
 * or TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_setupDualBandRxAgc(taliseDevice_t *device,
				   taliseRxChannels_t rxChannel, taliseAgcDualBandCfg_t *rxAgcCtrlDualBand);

/***************************************************************************//**
 * @brief Use this function to obtain the current LNA control settings for a
 * specified receive channel in dualband mode. It is applicable when
 * external LNAs are not controlled via the device's GPIO pins. The
 * function requires a valid device structure and a specified receive
 * channel, either Rx1 or Rx2. Ensure that the rxDualBandLnaControls
 * pointer is not null, as the function will populate this structure with
 * the LNA control values. The function returns a status code indicating
 * success or the type of error encountered.
 *
 * @param device Pointer to the Talise device data structure. Must not be null.
 * The caller retains ownership.
 * @param rxChannel An enum of type taliseRxChannels_t specifying the receive
 * channel (Rx1 or Rx2). Invalid values result in an error.
 * @param rxDualBandLnaControls Pointer to a taliseDualBandLnaControls_t
 * structure where the LNA control values will be
 * stored. Must not be null.
 * @return Returns a uint32_t status code indicating the result of the
 * operation: success or specific error codes.
 ******************************************************************************/
uint32_t TALISE_getDualBandLnaControls(taliseDevice_t *device,
				       taliseRxChannels_t rxChannel,
				       taliseDualBandLnaControls_t *rxDualBandLnaControls);

/***************************************************************************//**
 * @brief Use this function to configure the minimum and maximum gain indices
 * for the RX Automatic Gain Control (AGC) on specified channels. It is
 * essential to ensure that the maximum gain index is greater than the
 * minimum gain index before calling this function. The function should
 * be called after the device has been initialized and when AGC is
 * active. It supports setting gain indices for either RX1, RX2, or both
 * channels. The function performs parameter validation and returns an
 * error if the indices are out of range or incorrectly ordered.
 *
 * @param device Pointer to the Talise device data structure. Must not be null,
 * and the device should be properly initialized before calling
 * this function.
 * @param rxChannel Specifies the RX channel(s) to configure. It can be TAL_RX1,
 * TAL_RX2, or TAL_RX1RX2 to configure both channels.
 * @param maxGainIndex The maximum gain index for the AGC. It must be greater
 * than minGainIndex and within the range supported by the
 * device configuration.
 * @param minGainIndex The minimum gain index for the AGC. It must be less than
 * maxGainIndex and within the range supported by the device
 * configuration.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or error codes indicating specific issues such as invalid
 * parameters.
 ******************************************************************************/
uint32_t TALISE_setRxAgcMinMaxGainIndex(taliseDevice_t *device,
					taliseRxChannels_t rxChannel, uint8_t maxGainIndex, uint8_t minGainIndex);

/***************************************************************************//**
 * @brief This function is used to reset all state machines within the gain
 * control block of the device to their initial state, ensuring that the
 * AGC (Automatic Gain Control) loops are reset to state 0 and maximum
 * gain. It should be called when a reset of the AGC state is required,
 * typically after initialization or when reconfiguring the AGC settings.
 * The function requires a valid device structure and handles any errors
 * that occur during the SPI communication process, returning a status
 * code indicating the result of the operation.
 *
 * @param device Pointer to the Talise device data structure. Must not be null
 * and should be properly initialized before calling this
 * function. The function will handle errors related to SPI
 * communication and log them if necessary.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, and TALACT_NO_ACTION,
 * which indicate different levels of error handling or successful
 * completion.
 ******************************************************************************/
uint32_t TALISE_resetRxAgc(taliseDevice_t *device);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_AGC_H_ */
