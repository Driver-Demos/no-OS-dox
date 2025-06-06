/**
 * \file talise_error.h
 * \brief Contains Talise API error handling function prototypes and data types for talise_error.c
 *        These functions are public to the customer for getting more details on
 *        errors and debugging.
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_ERROR_H_
#define TALISE_ERROR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define IF_ERR_RETURN_U32(a) if(a > TALACT_ERR_CHECK_TIMER) { return (uint32_t)a; }
#define IF_ERR_RETURN(a) if(a > TALACT_ERR_CHECK_TIMER) { return a; }

#include "talise_types.h"
#include "talise_error_types.h"

/***************************************************************************//**
 * @brief Use this function to obtain the current error source and error code
 * from a specified Talise device instance. It is essential for debugging
 * and error handling, providing insight into the root cause of issues.
 * Ensure that the device and both output pointers are valid and non-null
 * before calling this function. The function will return an error if any
 * of these pointers are invalid.
 *
 * @param device Pointer to a taliseDevice_t structure representing the Talise
 * device instance. Must not be null.
 * @param errSrc Pointer to a uint32_t where the function will store the error
 * source. Must not be null.
 * @param errCode Pointer to a uint32_t where the function will store the error
 * code. Must not be null.
 * @return Returns TALACT_NO_ACTION if successful, or TALACT_ERR_CHECK_PARAM if
 * any input parameter is invalid.
 ******************************************************************************/
uint32_t TALISE_getErrCode(taliseDevice_t *device, uint32_t *errSrc,
			   uint32_t *errCode);

/***************************************************************************//**
 * @brief Use this function to obtain a human-readable error message
 * corresponding to a specific error source and error code. This is
 * particularly useful for debugging and understanding the nature of
 * errors encountered during the operation of the Talise device. The
 * function is only effective when the TALISE_VERBOSE macro is enabled in
 * the configuration. It is important to ensure that the error source and
 * code provided are valid and correspond to the enumerated lists defined
 * for the Talise API.
 *
 * @param errSrc A value representing the source of the error. It must
 * correspond to a valid error source as defined in the Talise
 * API.
 * @param errCode An error code associated with the specified error source. It
 * must be a valid code from the enumerated list of error codes
 * for the given source.
 * @return Returns a constant character string that describes the error based on
 * the provided error source and code.
 ******************************************************************************/
const char* TALISE_getErrorMessage(uint32_t errSrc, uint32_t errCode);

/***************************************************************************//**
 * @brief Use this function to obtain a human-readable error message
 * corresponding to a specific error source and code, which can aid in
 * debugging and error handling. This function is particularly useful
 * when the TALISE_VERBOSE macro is enabled, as it provides detailed
 * error messages. It should be called whenever an error code is
 * retrieved from a Talise device to understand the nature of the error.
 * The function requires a valid device pointer and will return an empty
 * string if verbose mode is not enabled.
 *
 * @param device Pointer to a taliseDevice_t structure that identifies the
 * specific instance of the Talise device. Must not be null.
 * @param errSrc An unsigned 32-bit integer representing the source of the
 * error. Must correspond to a valid error source as defined in
 * the Talise API.
 * @param errCode An unsigned 32-bit integer representing the specific error
 * code. Must correspond to a valid error code for the given
 * error source.
 * @return Returns a constant character pointer to a string describing the
 * error. If TALISE_VERBOSE is not enabled, returns an empty string.
 ******************************************************************************/
const char* TALISE_getRegisteredErrorMessage(taliseDevice_t *device,
		uint32_t errSrc, uint32_t errCode);

/***************************************************************************//**
 * @brief This function allows the registration of a callback function that
 * processes error messages for a specific error source on a Talise
 * device. It should be used when custom error handling is required for
 * specific error sources. The function is only effective when the
 * TALISE_VERBOSE macro is enabled. If the registration is successful,
 * the callback function will be invoked for the specified error source.
 * The function returns an error code indicating success or failure of
 * the registration process.
 *
 * @param device Pointer to a taliseDevice_t structure that identifies the
 * specific instance of the Talise device. Must not be null.
 * @param errSrc An unsigned 32-bit integer representing the source of the error
 * for which the callback function is being registered.
 * @param callbackFunction Pointer to a function that takes two uint32_t
 * parameters (errSrc and errCode) and returns a const
 * char*. This function will be called to process error
 * messages. Must not be null.
 * @return Returns 0 on successful registration, or 1 if the registration fails
 * due to lack of available space in the error function table.
 ******************************************************************************/
uint32_t talRegisterErrorMessage(taliseDevice_t *device, uint32_t errSrc,
				 const char* (*callbackFunction)(uint32_t, uint32_t));

/***************************************************************************//**
 * @brief This function is used to handle errors detected in the Talise API and
 * determine the appropriate recovery action based on the type of error
 * handler specified. It should be called whenever an error is detected
 * to ensure that the device's state is updated correctly and any
 * necessary recovery actions are taken. The function logs errors and
 * updates the device's error state information. It is important to
 * provide valid error handler types and ensure that the device pointer
 * is correctly initialized before calling this function.
 *
 * @param device Pointer to a taliseDevice_t structure identifying the desired
 * instance of the Talise device. Must not be null and should be
 * properly initialized before use.
 * @param errHdl Specifies the type of error handler to use. Must be a valid
 * value from the taliseErrHdls_t enumeration.
 * @param detErr The detected error code to be processed by the handler. Should
 * be a valid error code as defined by the relevant error source.
 * @param retVal The current recovery action to be returned if no new error is
 * detected. Must be a valid talRecoveryActions_t value.
 * @param recAction The new recovery action to be returned if the error handler
 * determines an error. Must be a valid talRecoveryActions_t
 * value.
 * @return Returns a talRecoveryActions_t value representing the latest recovery
 * action following the processing of the detected error.
 ******************************************************************************/
talRecoveryActions_t talApiErrHandler(taliseDevice_t *device,
				      taliseErrHdls_t errHdl, uint32_t detErr, talRecoveryActions_t retVal,
				      talRecoveryActions_t recAction);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_ERROR_H_ */
