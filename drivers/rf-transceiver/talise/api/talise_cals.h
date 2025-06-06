/**
 * \file talise_cals.h
 * \brief Contains Talise calibration related function prototypes for
 *        talise_cals.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_CALS_H_
#define TALISE_CALS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_cals_types.h"
#include "talise_tx_types.h"
#include "talise_rx_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to initiate the Talise device's initialization
 * calibrations, which are specified by a bitmask. It should be called
 * after the device has been initialized and the RF PLL is confirmed to
 * be locked. The function allows for selective calibration by using a
 * bitmask to specify which calibrations to run. It returns a status code
 * indicating the result of the operation, which can include warnings or
 * errors that may require specific recovery actions. This function is
 * essential for setting up the device's calibration state before normal
 * operation.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null, and the device should be
 * properly initialized before calling this function.
 * @param calMask A 32-bit unsigned integer bitmask specifying which
 * calibrations to run. Each bit corresponds to a specific
 * calibration, and the function will execute the calibrations
 * for the bits set to 1.
 * @return Returns a uint32_t value representing the recovery action status.
 * Possible values include TALACT_NO_ACTION for success,
 * TALACT_WARN_RESET_LOG for a log reset warning, and various error
 * codes indicating different recovery actions required.
 ******************************************************************************/
uint32_t TALISE_runInitCals(taliseDevice_t *device, uint32_t calMask);

/***************************************************************************//**
 * @brief This function is used to block the calling thread until the Talise
 * initialization calibrations are completed or a specified timeout
 * occurs. It should be called after TALISE_runInitCals to ensure that
 * the calibrations have finished. The function requires a valid device
 * pointer and a non-null errorFlag pointer to store any error codes that
 * may arise during the calibration process. The timeoutMs parameter
 * specifies the maximum time to wait for the calibrations to complete.
 * If the function encounters an error, it returns a recovery action code
 * indicating the type of error and the necessary recovery action.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param timeoutMs A timeout value in milliseconds to wait for the calibrations
 * to complete. Must be a positive integer.
 * @param errorFlag A pointer to a uint8_t variable to store a 3-bit error flag
 * indicating any errors during the initial calibrations. Must
 * not be null.
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_NO_ACTION for success or various error codes for
 * different failure scenarios.
 ******************************************************************************/
uint32_t TALISE_waitInitCals(taliseDevice_t *device, uint32_t timeoutMs,
			     uint8_t *errorFlag);

/***************************************************************************//**
 * @brief Use this function to immediately check the status of Talise
 * initialization calibrations without blocking the thread. It should be
 * called after the initialization calibrations have been started. This
 * function allows the application to perform other tasks while the
 * calibrations are running. The function provides the status of whether
 * calibrations are still running and optionally returns an error flag
 * indicating any issues encountered during the process.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param areCalsRunning A pointer to a uint8_t variable where the function will
 * store the status: 1 if calibrations are running, 0 if
 * they are not. Must not be null.
 * @param errorFlag An optional pointer to a uint8_t variable where the function
 * will store a 3-bit error code if any errors occurred during
 * calibration. Pass NULL if not needed.
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_NO_ACTION for success or various error codes for
 * different issues.
 ******************************************************************************/
uint32_t TALISE_checkInitCalComplete(taliseDevice_t *device,
				     uint8_t *areCalsRunning, uint8_t *errorFlag);

/***************************************************************************//**
 * @brief This function is used to stop the current initialization calibration
 * sequence of the ARM processor in the Talise device. It can be called
 * at any time during the initialization calibration process. The
 * function optionally returns a bitmask indicating which calibrations
 * were completed before the abort command was issued. This is useful for
 * determining the progress of the calibration sequence up to the point
 * of interruption. The function must be called with a valid device
 * pointer, and the optional calsCompleted parameter can be null if the
 * caller does not require this information.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param calsCompleted A pointer to a uint32_t variable where the completed
 * calibrations bitmask will be stored. Can be null if the
 * caller does not need this information.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * TALACT_ERR_RESET_ARM for ARM reset required, and other error codes
 * indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_abortInitCals(taliseDevice_t *device, uint32_t *calsCompleted);

/***************************************************************************//**
 * @brief This function provides the status of initialization calibrations for a
 * Talise device, including the number of calibrations since power-up,
 * the last run, and the minimum required calibrations. It also reports
 * any errors encountered during initialization. This function should be
 * called after the device has been initialized and initialization
 * calibrations have been performed. It requires valid pointers for all
 * output parameters and will return an error if any are null.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param calsSincePowerUp Pointer to a uint32_t where the function will store
 * the number of calibrations run since power-up. Must
 * not be null.
 * @param calsLastRun Pointer to a uint32_t where the function will store the
 * number of calibrations run during the last initialization
 * sequence. Must not be null.
 * @param calsMinimum Pointer to a uint32_t where the function will store the
 * minimum number of calibrations required. Must not be null.
 * @param initErrCal Pointer to a uint8_t where the function will store the
 * object ID of the calibration reporting an error. Must not
 * be null.
 * @param initErrCode Pointer to a uint8_t where the function will store the
 * error code of the initialization calibration. Must not be
 * null.
 * @return Returns a uint32_t indicating the recovery action required, or
 * TALACT_NO_ACTION if successful. Outputs are written to the provided
 * pointers.
 ******************************************************************************/
uint32_t TALISE_getInitCalStatus(taliseDevice_t *device,
				 uint32_t *calsSincePowerUp, uint32_t *calsLastRun, uint32_t *calsMinimum,
				 uint8_t *initErrCal, uint8_t *initErrCode);

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function configures which tracking calibrations are active when
 * the radio is in the ON state. It must be called while the radio is OFF
 * and after the device has been fully initialized, including PLL
 * configuration, MCS, and JESD204B link setup. If invoked during the
 * radio ON state, an error will be returned. The function uses a bitmask
 * to specify the calibrations to enable, where each bit corresponds to a
 * specific calibration type.
 *
 * @param device Pointer to the device settings structure. Must not be null.
 * @param enableMask A bitmask indicating which tracking calibrations to enable.
 * Each bit corresponds to a specific calibration type, and
 * invalid bits will result in an error.
 * @return Returns a uint32_t indicating the result of the operation, with
 * specific values representing different recovery actions or success.
 ******************************************************************************/
uint32_t TALISE_enableTrackingCals(taliseDevice_t *device, uint32_t enableMask);

/***************************************************************************//**
 * @brief This function is used to determine which tracking calibrations are
 * currently enabled on a Talise device. It should be called after the
 * device has been fully initialized and the ARM processor is loaded and
 * running. The function requires a valid pointer to a device structure
 * and a non-null pointer to store the result. If the enableMask pointer
 * is null, the function will return an error. This function is useful
 * for verifying the current calibration settings before performing
 * operations that depend on specific calibrations being active.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. The caller retains ownership and must ensure it is
 * properly initialized.
 * @param enableMask A pointer to a uint32_t variable where the function will
 * store the bitmask of enabled tracking calibrations. Must
 * not be null.
 * @return Returns a uint32_t value indicating the status of the operation. A
 * successful operation returns TALACT_NO_ACTION, while errors related
 * to parameter checks or ARM reset requirements return specific error
 * codes.
 ******************************************************************************/
uint32_t TALISE_getEnabledTrackingCals(taliseDevice_t *device,
				       uint32_t *enableMask);

/***************************************************************************//**
 * @brief Use this function to check which tracking calibrations are pending or
 * have encountered errors. It should be called when the device is in the
 * radioOn state, and the enabled tracking calibrations are active. The
 * function provides a bitmask indicating the status of each calibration,
 * allowing the user to identify pending calibrations or those that have
 * errors. Ensure that the `pendingCalMask` parameter is not null before
 * calling this function.
 *
 * @param device A pointer to the device settings structure. The caller retains
 * ownership and it must not be null.
 * @param pendingCalMask A pointer to a uint32_t variable where the function
 * will store the bitmask indicating pending or error
 * status of tracking calibrations. Must not be null.
 * @return Returns a uint32_t value indicating the function's success or
 * specific error conditions. The `pendingCalMask` is updated with the
 * status bitmask.
 ******************************************************************************/
uint32_t TALISE_getPendingTrackingCals(taliseDevice_t *device,
				       uint32_t *pendingCalMask);

/***************************************************************************//**
 * @brief This function is used to schedule a specific tracking calibration to
 * run, which can be useful for overriding the tracking calibration timer
 * or rescheduling a calibration after an error has been detected. It can
 * be called in either the Radio On or Radio Off state, but requires that
 * the ARM is initialized. The function only allows scheduling one
 * tracking calibration object per channel per call.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param trackingCal An enumerated value of type taliseTrackingCalibrations_t
 * that selects the tracking calibration to schedule. If an
 * invalid value is provided, the function will handle it as
 * a parameter error.
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_NO_ACTION for success or various error codes for
 * different failure scenarios.
 ******************************************************************************/
uint32_t TALISE_rescheduleTrackingCal(taliseDevice_t *device,
				      taliseTrackingCalibrations_t trackingCal);

/***************************************************************************//**
 * @brief This function allows the suspension or resumption of active tracking
 * calibrations specified by the calSubsetMask while the device is in the
 * Radio On state. It is useful for managing calibrations without needing
 * to switch to the Radio Off state. Only calibrations that are enabled
 * can be paused or resumed. The function should be called when the
 * device is in the Radio On state, and the tracking calibrations of
 * interest must have been enabled during the Radio Off state.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param calSubsetMask A bitmask selecting which tracking calibrations to apply
 * the suspend or resume setting to. Each bit corresponds
 * to a specific calibration, and only those with the bit
 * set will be affected by the resumeCalMask.
 * @param resumeCalMask A bitmask indicating which calibrations to resume. If a
 * bit is set and the corresponding bit in calSubsetMask is
 * also set, the calibration will be resumed; otherwise, it
 * will be paused.
 * @return Returns a uint32_t indicating the recovery action required. Possible
 * values include TALACT_WARN_RESET_LOG, TALACT_ERR_CHECK_PARAM,
 * TALACT_ERR_RESET_ARM, and TALACT_NO_ACTION, indicating various levels
 * of error handling or successful completion.
 ******************************************************************************/
uint32_t TALISE_setAllTrackCalState(taliseDevice_t *device,
				    uint32_t calSubsetMask, uint32_t resumeCalMask);

/***************************************************************************//**
 * @brief This function is used to obtain the current suspend or resume state of
 * all active tracking calibrations in a Talise device. It should be
 * called when the device is in the Radio On state. The function updates
 * the provided resumeCalMask with a bitmask indicating which
 * calibrations are currently resumed (running) or paused. This allows
 * users to monitor the status of tracking calibrations without
 * interrupting the Radio On state.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. This must be a valid, initialized device structure and
 * must not be null.
 * @param resumeCalMask A pointer to a uint32_t variable where the function will
 * store the bitmask representing the state of each
 * tracking calibration. Each bit corresponds to a specific
 * calibration, with 1 indicating the calibration is
 * resumed and 0 indicating it is paused. This pointer must
 * not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * TALACT_WARN_RESET_LOG for log reset warnings, TALACT_ERR_CHECK_PARAM
 * for parameter errors, and TALACT_ERR_RESET_ARM for ARM reset
 * requirements.
 ******************************************************************************/
uint32_t TALISE_getAllTrackCalState(taliseDevice_t *device,
				    uint32_t *resumeCalMask);

/***************************************************************************//**
 * @brief This function is used to obtain the status of the TxLOL external
 * tracking calibration for a specified channel. It provides detailed
 * metrics such as error codes, percentage completion, variance metrics,
 * iteration count, and update count. The function should be called after
 * the device has been initialized, the ARM loaded, and initialization
 * calibrations have been run. It can be executed in both radioOff and
 * radioOn states. Proper error handling is performed for invalid
 * parameters, ensuring robust operation.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param channelSel Specifies the Tx channel (Tx1 or Tx2) for which the status
 * is to be retrieved. Must be a valid channel; otherwise, an
 * error is returned.
 * @param txLolStatus A pointer to a taliseTxLolStatus_t structure where the
 * status of the TxLOL external calibration will be stored.
 * Must not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * TALACT_WARN_RESET_LOG for log reset warnings, TALACT_ERR_CHECK_PARAM
 * for parameter errors, and TALACT_ERR_RESET_SPI for SPI reset
 * requirements.
 ******************************************************************************/
uint32_t TALISE_getTxLolStatus(taliseDevice_t *device,
			       taliseTxChannels_t channelSel, taliseTxLolStatus_t *txLolStatus);

/***************************************************************************//**
 * @brief This function retrieves the status of the TxQEC tracking calibration
 * for a specified channel, providing metrics such as error codes,
 * completion percentage, and performance metrics. It should be called
 * after the device has been initialized, the ARM loaded, and initial
 * calibrations run. The function can be used in both radioOff and
 * radioOn states. It is important to ensure that the `txQecStatus`
 * parameter is not null before calling this function.
 *
 * @param device A pointer to the device settings structure. The caller retains
 * ownership and it must not be null.
 * @param channelSel Specifies the channel (Tx1 or Tx2) for which the status is
 * to be retrieved. Valid values are TAL_TX1 and TAL_TX2. An
 * invalid value will result in an error.
 * @param txQecStatus A pointer to a taliseTxQecStatus_t structure where the
 * status will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or type of error
 * encountered. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_PARAM for invalid parameters, and other
 * error codes indicating specific issues.
 ******************************************************************************/
uint32_t TALISE_getTxQecStatus(taliseDevice_t *device,
			       taliseTxChannels_t channelSel, taliseTxQecStatus_t *txQecStatus);

/***************************************************************************//**
 * @brief This function retrieves the status of the RxQEC tracking calibration
 * for a specified channel, providing details such as error codes,
 * completion percentage, and other performance metrics. It should be
 * called after the device has been initialized, the ARM loaded, and
 * initialization calibrations have been run. The function can be used in
 * both radioOn and radioOff states. It is important to ensure that the
 * `rxQecStatus` parameter is not null before calling this function.
 *
 * @param device A pointer to the device settings structure. The caller retains
 * ownership and it must not be null.
 * @param channelSel Specifies the Rx channel (Rx1 or Rx2) for which the QEC
 * status is to be retrieved. Invalid channel selections will
 * result in an error.
 * @param rxQecStatus A pointer to a `taliseRxQecStatus_t` structure where the
 * RxQEC status will be stored. Must not be null.
 * @return Returns a `uint32_t` value indicating the success or failure of the
 * operation, with specific error codes for different failure scenarios.
 ******************************************************************************/
uint32_t TALISE_getRxQecStatus(taliseDevice_t *device,
			       taliseRxChannels_t channelSel, taliseRxQecStatus_t *rxQecStatus);

/***************************************************************************//**
 * @brief This function is used to obtain the status of the ORxQEC tracking
 * calibration for a specified observation receiver channel. It should be
 * called after the device has been initialized, the ARM loaded, and
 * initialization calibrations have been run. The function can be used in
 * both radioOn and radioOff states. It provides detailed status
 * information including error codes, completion percentage, and other
 * metrics related to the ORxQEC calibration process.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param channelSel Specifies the observation receiver channel (ORx1 or ORx2)
 * for which the status is requested. Must be a valid
 * taliseObsRxChannels_t value; otherwise, an error is
 * returned.
 * @param orxQecStatus A pointer to a taliseOrxQecStatus_t structure where the
 * status of the ORxQEC calibration will be stored. Must not
 * be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or various error codes indicating specific issues
 * encountered during execution.
 ******************************************************************************/
uint32_t TALISE_getOrxQecStatus(taliseDevice_t *device,
				taliseObsRxChannels_t channelSel, taliseOrxQecStatus_t *orxQecStatus);

/***************************************************************************//**
 * @brief This function is used to obtain the current status of the RxHD2
 * tracking calibration for a specified receive channel. It should be
 * called after the device has been initialized, the ARM has been loaded,
 * and initialization calibrations have been run. The function can be
 * used in both radioOn and radioOff states. It provides detailed status
 * information including error codes, completion percentage, confidence
 * level, iteration count, and update count. Proper error handling is
 * performed for invalid parameters.
 *
 * @param device A pointer to the device settings structure. Must not be null.
 * @param channelSel Specifies the receive channel (Rx1 or Rx2) for which the
 * status is to be retrieved. Must be a valid channel;
 * otherwise, an error is returned.
 * @param rxHd2Status A pointer to a taliseRxHd2Status_t structure where the
 * status information will be stored. Must not be null.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or various error codes indicating specific issues.
 ******************************************************************************/
uint32_t TALISE_getRxHd2Status(taliseDevice_t *device,
			       taliseRxChannels_t channelSel, taliseRxHd2Status_t *rxHd2Status);

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to block execution until a specified event
 * occurs or a timeout is reached. It is typically called when waiting
 * for hardware events such as PLL locks or calibration completions. The
 * function requires a valid device pointer and an event type to wait
 * for. It also requires a timeout value in microseconds and an interval
 * for checking the event status. If the event does not occur within the
 * specified timeout, an error is returned. This function should be used
 * when precise control over event timing is necessary.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param waitEvent An enumerated value of type taliseWaitEvent_t specifying the
 * event to wait for. Must be a valid event type.
 * @param timeout_us The maximum time in microseconds to wait for the event.
 * Must be a positive integer.
 * @param waitInterval_us The interval in microseconds between checks for the
 * event status. Must be less than or equal to
 * timeout_us.
 * @return Returns a uint32_t value indicating the result of the wait operation.
 * Possible return values include success, timeout, or various error
 * codes indicating specific issues encountered during the wait.
 ******************************************************************************/
uint32_t TALISE_waitForEvent(taliseDevice_t *device,
			     taliseWaitEvent_t waitEvent, uint32_t timeout_us, uint32_t waitInterval_us);

/***************************************************************************//**
 * @brief This function is used to check the status of a specified event on a
 * Talise device and update the provided event status flag accordingly.
 * It is a non-blocking function that can be called to determine if a
 * particular event has completed. The function requires a valid device
 * pointer and an event type to check. The event status is returned
 * through a pointer, which must not be null. If the event type is
 * invalid or the event status pointer is null, the function will handle
 * the error and return an appropriate error code.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. Must not be null.
 * @param waitEvent An enumerated value of type taliseWaitEvent_t representing
 * the event to check. Must be a valid event type.
 * @param eventDone A pointer to a uint8_t where the event status will be
 * stored. Must not be null. The function will set this to 1 if
 * the event is done, or 0 if not.
 * @return Returns a uint32_t value indicating the result of the operation, with
 * specific error codes for invalid parameters or other issues.
 ******************************************************************************/
uint32_t TALISE_readEventStatus(taliseDevice_t *device,
				taliseWaitEvent_t waitEvent, uint8_t *eventDone);

/***************************************************************************//**
 * @brief Use this function to reset the external Tx LO leakage tracking
 * calibration for a specified transmission channel on a Talise device.
 * This function should be called when the ARM is in the radioOff or IDLE
 * state to ensure proper operation. It is important to select a valid
 * channel using the provided enumeration, as invalid channel selections
 * will result in an error. The function returns a status code indicating
 * the success of the operation or any required recovery actions.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. This pointer must not be null, and the device
 * should be properly initialized before calling this function.
 * @param channelSel An enumeration of type taliseTxChannels_t that specifies
 * the transmission channel to reset. Valid values are
 * TAL_TX1, TAL_TX2, and TAL_TX1TX2. An invalid value will
 * result in an error.
 * @return Returns a uint32_t status code indicating the result of the
 * operation. Possible values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and other codes
 * indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_resetExtTxLolChannel(taliseDevice_t *device,
				     taliseTxChannels_t channelSel);

/***************************************************************************//**
 * @brief This function is used to set the configuration options for the Rx HD2
 * tracking calibration on a Talise device. It must be called when the
 * Talise ARM is in the Radio Off state or before initialization
 * calibrations have been run. The function configures the side band
 * selection for the HD2 calibration, which determines the side of the
 * spectrum where the 2nd harmonic distortion correction will be applied.
 * It is important to ensure that the device is in the correct state
 * before calling this function to avoid errors.
 *
 * @param device A pointer to the taliseDevice_t structure containing the device
 * settings. Must not be null.
 * @param hd2CalConfig A pointer to a taliseRxHd2Config_t structure containing
 * the desired HD2 calibration settings. Must not be null.
 * If null, the function will handle it as an error.
 * @return Returns a uint32_t value indicating the result of the operation,
 * which can include success or various error codes related to parameter
 * checks or ARM state issues.
 ******************************************************************************/
uint32_t TALISE_setRxHd2Config(taliseDevice_t *device,
			       taliseRxHd2Config_t *hd2CalConfig);

/***************************************************************************//**
 * @brief This function is used to obtain the current configuration settings for
 * the Rx HD2 tracking calibration from the Talise ARM memory. It should
 * be called when the ARM is in the Radio Off state or before
 * initialization calibrations have been run. The function ensures that
 * the ARM is in the correct state before attempting to read the
 * configuration, and it returns a recovery action code indicating the
 * result of the operation.
 *
 * @param device A pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param hd2CalConfig A pointer to a taliseRxHd2Config_t structure where the
 * current HD2 calibration configuration will be stored.
 * Must not be null.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion,
 * TALACT_WARN_RESET_LOG for log reset warnings, TALACT_ERR_RESET_ARM if
 * the ARM is in the wrong state, and TALACT_ERR_CHECK_PARAM for
 * parameter errors.
 ******************************************************************************/
uint32_t TALISE_getRxHd2Config(taliseDevice_t *device,
			       taliseRxHd2Config_t *hd2CalConfig);

/***************************************************************************//**
 * @brief This function adjusts the digital DC offset convergence time, known as
 * mShift, for a specified channel, affecting the corner frequency of the
 * notch filter used to filter DC offset from the Rx/ORx receive signal.
 * It is applicable to all Rx or ORx channels, but they can be set
 * individually. The function should be called after ARM bootup, and for
 * ARM versions less than 4.0, it must be called after running the init
 * cals to prevent the ARM from overwriting the mShift setting. The valid
 * range for mShift is 8 to 20, and values outside this range will result
 * in an error.
 *
 * @param device A pointer to the taliseDevice_t structure containing device
 * settings. Must not be null.
 * @param channel Specifies the digital DC offset channel to adjust. Must be a
 * valid taliseDcOffsetChannels_t value.
 * @param mShift The mShift value to set for the specified channel, with a valid
 * range of 8 to 20. Values outside this range will trigger an
 * error.
 * @return Returns a uint32_t indicating the success or type of error
 * encountered. Possible values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for invalid parameters, and other error codes
 * for specific issues.
 ******************************************************************************/
uint32_t TALISE_setDigDcOffsetMShift(taliseDevice_t *device,
				     taliseDcOffsetChannels_t channel, uint8_t mShift);

/***************************************************************************//**
 * @brief Use this function to obtain the current mShift value, which affects
 * the corner frequency of the notch filter used to filter DC offset from
 * the Rx/ORx receive signal. This function should be called after the
 * ARM initialization calibrations have been run for ARM versions less
 * than 4.0, or anytime after ARM bootup for versions 4.0 or greater.
 * Ensure that the mShift pointer is not null before calling this
 * function.
 *
 * @param device A pointer to the taliseDevice_t structure containing device
 * settings. Must not be null.
 * @param channel Specifies the digital DC offset channel to read the mShift
 * value for. Must be a valid taliseDcOffsetChannels_t value.
 * @param mShift A pointer to a uint8_t where the current mShift value will be
 * stored. Must not be null.
 * @return Returns a uint32_t indicating the status of the operation, with
 * specific values indicating success or specific error conditions.
 ******************************************************************************/
uint32_t TALISE_getDigDcOffsetMShift(taliseDevice_t *device,
				     taliseDcOffsetChannels_t channel, uint8_t *mShift);

/***************************************************************************//**
 * @brief This function is used to enable or disable specific digital DC offset
 * channels on a Talise device by providing a bitmask that specifies
 * which channels to affect. It should be called when you need to
 * configure the digital DC offset settings for the device. The function
 * requires a valid device pointer and a bitmask that indicates the
 * channels to enable or disable. If the enableMask contains invalid
 * bits, the function will return an error indicating an invalid
 * parameter.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. This pointer must not be null, and the caller
 * retains ownership.
 * @param enableMask A uint8_t bitmask indicating which digital DC offset
 * channels to enable or disable. Valid values are
 * combinations of TAL_DC_OFFSET_RX1, TAL_DC_OFFSET_RX2,
 * TAL_DC_OFFSET_ORX1, TAL_DC_OFFSET_ORX2, and
 * TAL_DC_OFFSET_ALL_ON. Invalid bits in the mask will result
 * in an error.
 * @return Returns a uint32_t indicating the result of the operation. Possible
 * return values include TALISE_ERR_DIG_DC_OFFSET_INV_ENABLE_MASK for an
 * invalid enable mask, and TALISE_ERR_OK for successful completion.
 ******************************************************************************/
uint32_t TALISE_setDigDcOffsetEn(taliseDevice_t *device, uint8_t enableMask);

/***************************************************************************//**
 * @brief This function is used to obtain the current digital DC offset enable
 * mask for the specified device, which indicates which channels have
 * digital DC offset correction enabled. It should be called when you
 * need to verify or log the current state of digital DC offset
 * correction settings. Ensure that the `enableMask` pointer is not null
 * before calling this function, as a null pointer will result in an
 * error.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. The caller retains ownership and it must be properly
 * initialized before calling this function.
 * @param enableMask A pointer to a uint8_t variable where the function will
 * store the enable mask. This pointer must not be null, as
 * passing a null pointer will result in an error.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. A successful operation returns TALACT_NO_ACTION, while
 * errors related to invalid parameters or SPI communication issues
 * return specific error codes.
 ******************************************************************************/
uint32_t TALISE_getDigDcOffsetEn(taliseDevice_t *device, uint8_t *enableMask);

/***************************************************************************//**
 * @brief This function is used to obtain the current batch size setting for
 * tracking calibrations from the Talise device. It should be called when
 * the ARM is in the RadioOff/IDLE or ready state. The function will
 * populate the provided pointer with the batch size value, which
 * indicates the duration of the calibration batch in microseconds. It is
 * important to ensure that the `batchsize_us` parameter is not null
 * before calling this function, as a null pointer will result in an
 * error.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device. The caller retains ownership and it must be properly
 * initialized before calling this function.
 * @param batchsize_us A pointer to a taliseTrackingCalBatchSize_t variable
 * where the batch size will be stored. Must not be null, as
 * passing a null pointer will result in an error.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or various error codes indicating specific issues such as
 * invalid parameters or incorrect ARM state.
 ******************************************************************************/
uint32_t TALISE_getTrackingCalsBatchSize(taliseDevice_t *device,
		taliseTrackingCalBatchSize_t* batchsize_us);

/***************************************************************************//**
 * @brief This function configures the batch size for tracking calibrations on a
 * Talise device, allowing the user to select between predefined batch
 * sizes. It must be called when the Talise ARM is in the RadioOff/IDLE
 * state or ready state. The function ensures that the device is in a
 * suitable state before applying the configuration, and it returns a
 * status code indicating the success or failure of the operation.
 *
 * @param device A pointer to the taliseDevice_t structure representing the
 * device settings. Must not be null.
 * @param batchsize_us An enumerated value of type taliseTrackingCalBatchSize_t
 * specifying the desired batch size. Valid values are
 * predefined constants such as TAL_TRACK_BATCH_SIZE_500_US
 * and TAL_TRACK_BATCH_SIZE_200_US. Invalid values result in
 * an error.
 * @return Returns a uint32_t status code indicating the result of the
 * operation, with specific codes for success, parameter errors, and
 * required recovery actions.
 ******************************************************************************/
uint32_t TALISE_setTrackingCalsBatchSize(taliseDevice_t *device,
		taliseTrackingCalBatchSize_t batchsize_us);
/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief Use this function to obtain a human-readable error message
 * corresponding to a specific calibration error code and source. This is
 * useful for debugging and logging purposes when dealing with
 * initialization calibration errors in the Talise API. The function
 * should be called with valid error source and error code values to
 * retrieve meaningful messages. If the error source or code is not
 * recognized, a generic 'Unknown Init Calibration Error' message is
 * returned. This function is only effective when the TALISE_VERBOSE mode
 * is enabled; otherwise, it returns an empty string.
 *
 * @param errSrc A 32-bit unsigned integer representing the source of the error.
 * It should correspond to a valid error source defined in the
 * Talise API.
 * @param errCode A 32-bit unsigned integer representing the specific error
 * code. It should be a valid error code associated with the
 * given error source.
 * @return A constant character pointer to a string containing the error
 * message. If the error source or code is unrecognized, a generic error
 * message is returned. In non-verbose mode, an empty string is
 * returned.
 ******************************************************************************/
const char* talGetCalErrorMessage(uint32_t errSrc, uint32_t errCode);

#define ARMINITCAL_ERRCODE(armCalId, armCalErrCode) ((armCalId << 8) | armCalErrCode)

#ifdef __cplusplus
}
#endif

#endif /* TALISE_CALS_H_ */
