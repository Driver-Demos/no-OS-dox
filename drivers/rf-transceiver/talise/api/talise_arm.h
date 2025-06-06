/**
 * \file talise_arm.h
 * \brief Contains Talise ARM related function prototypes for talise_arm.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_ARM_H_
#define TALISE_ARM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"
#include "talise_error_types.h"
#include "talise_arm_types.h"
/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to reset and initialize the ARM processor on a
 * Talise device, setting the ARM run state to zero, disabling parity
 * checks, configuring clock selections, and enabling SPI register
 * access. It should be called after the device has been initialized and
 * multichip synchronization (MCS) has been completed. The function
 * handles various clock rate configurations and ensures that the ARM
 * clock does not exceed specified limits. It also writes ARM profiles
 * and loads ADC profiles as part of the initialization process. The
 * function returns a recovery action code indicating the result of the
 * operation.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null and should be properly initialized before calling this
 * function.
 * @param init Pointer to the Talise initialization setting data structure. Must
 * not be null and should contain valid initialization settings.
 * @return Returns a uint32_t value representing a recovery action code, which
 * can indicate success or specific error conditions requiring further
 * action.
 ******************************************************************************/
uint32_t TALISE_initArm(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief This function is used to write configuration settings to the Talise
 * ARM processor. It should be called automatically during the
 * initialization process with TALISE_initArm() and must be executed
 * before loading the ARM firmware. The function prepares and writes
 * configuration data to the ARM memory, which includes clock settings,
 * channel enablement, and profile data for transmission, reception, and
 * observation. It is essential to ensure that the device and
 * initialization structures are correctly set up before calling this
 * function to avoid parameter errors.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null and should be properly initialized with device-specific
 * settings.
 * @param init Pointer to the Talise initialization settings data structure.
 * Must not be null and should contain valid initialization
 * parameters for the device.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, and TALACT_NO_ACTION,
 * which indicate different levels of success or required actions.
 ******************************************************************************/
uint32_t TALISE_writeArmProfile(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief This function is used to load a binary array into the ARM program
 * memory of a Talise device. It should be called after the device has
 * been initialized, the PLL lock status has been verified, and the
 * stream binary has been loaded. The function requires a specific number
 * of bytes in the binary array, and it will return an error if the count
 * does not match the expected size. It also handles various ARM states
 * and ensures the ARM processor is ready after loading the binary.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param binary Byte array containing all valid ARM file data bytes. Must not
 * be null and should contain exactly 114688 bytes.
 * @param count The number of bytes in the binary array. Must be exactly 114688,
 * otherwise the function will return an error.
 * @return Returns a uint32_t value indicating the recovery action or error
 * status. Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_ERR_CHECK_PARAM, TALACT_ERR_RESET_SPI, and TALACT_NO_ACTION.
 ******************************************************************************/
uint32_t TALISE_loadArmFromBinary(taliseDevice_t *device, uint8_t *binary,
				  uint32_t count);

/***************************************************************************//**
 * @brief This function is used to load ADC profile data into the ARM memory of
 * a Talise device, which is necessary before loading the ARM firmware.
 * It writes four ADC profiles and an optional ORx merge filter into the
 * ARM data memory. This function should be called automatically during
 * device initialization. The function handles different profiles based
 * on the device's state and writes zeros if a profile is not valid. It
 * returns a status code indicating the success or type of recovery
 * action required.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null.
 * @param rxAdcProfile Pointer to the Talise Rx ADC profile data structure. Must
 * not be null if the Rx profile is valid.
 * @param orxLowPassAdcProfile Pointer to the Talise ORx low pass ADC profile
 * data structure. Must not be null if the ORx
 * profile is valid.
 * @param orxBandPassAdcProfile Pointer to the Talise ORx band pass ADC profile
 * data structure. Used only if ADC stitching is
 * enabled; otherwise, it is ignored.
 * @param loopBackAdcProfile Pointer to the Talise Loopback Rx ADC profile data
 * structure. Must not be null if the Tx profile is
 * valid.
 * @param orxMergeFilter Pointer to the Talise ORx merge filter data structure.
 * Used only if ADC stitching is enabled; otherwise, it is
 * ignored.
 * @return Returns a uint32_t value indicating the recovery action required:
 * TALACT_NO_ACTION for success, or other values for specific recovery
 * actions.
 ******************************************************************************/
uint32_t TALISE_loadAdcProfiles(taliseDevice_t *device, uint16_t *rxAdcProfile,
				uint16_t *orxLowPassAdcProfile,
				uint16_t *orxBandPassAdcProfile, uint16_t *loopBackAdcProfile,
				int16_t *orxMergeFilter);
/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from the Talise ARM
 * memory, either program or data memory, starting from a given address.
 * It is intended for internal use and should not be called directly by
 * users. The function requires a valid memory address within the
 * specified ranges for program or data memory. It supports optional
 * auto-increment of the address during the read operation. The function
 * returns a status code indicating the success or failure of the
 * operation, with specific recovery actions suggested for different
 * error conditions.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param address The 32-bit ARM address to read from. Must be within the valid
 * ranges for program (0x01000000 - 0x0101C000) or data memory
 * (0x20000000 - 0x20014000).
 * @param returnData Byte array to store the data read from the ARM memory. Must
 * not be null and should be large enough to hold
 * 'bytesToRead' bytes.
 * @param bytesToRead Number of bytes to read from the ARM memory. The sum of
 * 'address' and 'bytesToRead' must not exceed the valid
 * memory range.
 * @param autoIncrement Boolean flag to enable (non-zero) or disable (zero)
 * auto-increment of the ARM register address during the
 * read operation.
 * @return Returns a status code indicating the result of the operation, with
 * possible values including success, parameter error, or SPI reset
 * required.
 ******************************************************************************/
uint32_t TALISE_readArmMem(taliseDevice_t *device, uint32_t address,
			   uint8_t *returnData, uint32_t bytesToRead, uint8_t autoIncrement);

/***************************************************************************//**
 * @brief This function writes a specified number of bytes from a data array to
 * a given address in the Talise ARM program or data memory. It should be
 * used when you need to update or configure the ARM memory with new
 * data. The function requires that the address and the range defined by
 * the address and byte count are within valid memory regions. It is
 * important to ensure that the device is properly initialized before
 * calling this function. The function returns a status code indicating
 * the success or type of error encountered during the operation.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param address The 32-bit ARM address to write to. Must be within valid
 * program (0x01000000 - 0x0101C000) or data memory (0x20000000 -
 * 0x20014000) ranges.
 * @param data Byte array containing the data to be written to ARM memory. Must
 * not be null.
 * @param byteCount Number of bytes in the data array to be written. The address
 * plus byteCount must not exceed the valid memory range.
 * @return Returns a status code indicating the result of the operation, which
 * can be used to determine if any recovery actions are needed.
 ******************************************************************************/
uint32_t TALISE_writeArmMem(taliseDevice_t *device, uint32_t address,
			    uint8_t *data, uint32_t byteCount);

/***************************************************************************//**
 * @brief This function is used to write configuration data to a specific
 * location in the Talise ARM memory. It should be called when there is a
 * need to update or modify ARM configuration settings. The function
 * requires a valid device structure and expects the ARM firmware to be
 * loaded and initialized prior to its invocation. It handles command
 * execution and checks for errors, returning a status code that
 * indicates the success or failure of the operation.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null and should be properly initialized before calling this
 * function.
 * @param objectId ARM id of a particular structure or setting in ARM memory. It
 * identifies the target configuration object.
 * @param offset Byte offset from the start of the objectId's memory location in
 * ARM memory. It specifies where the data should be written
 * within the object.
 * @param data Pointer to a byte array containing the data to be written to the
 * ARM memory. Must not be null and should contain at least
 * 'byteCount' bytes.
 * @param byteCount Number of bytes in the data array to be written. Valid range
 * is 1 to 255 bytes.
 * @return Returns a uint32_t value indicating the result of the operation,
 * where specific values correspond to different recovery actions or
 * success.
 ******************************************************************************/
uint32_t TALISE_writeArmConfig(taliseDevice_t *device, uint8_t objectId,
			       uint16_t offset, uint8_t *data, uint8_t byteCount);

/***************************************************************************//**
 * @brief This function is used to read a specified number of bytes from a
 * particular configuration object in the Talise ARM memory. It is
 * typically used to retrieve configuration settings or status
 * information from the ARM processor. The function requires a valid
 * device structure and must be called with a valid object ID and offset
 * within the ARM memory. The data buffer provided must be large enough
 * to hold the number of bytes specified by byteCount. The function
 * returns a status code indicating the success or failure of the
 * operation, including any necessary recovery actions.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null.
 * @param objectId The ARM ID of the specific configuration object to read. Must
 * be a valid ID recognized by the ARM processor.
 * @param offset The byte offset from the start of the objectId's memory
 * location in ARM memory. Must be within the valid range for the
 * specified object.
 * @param data Pointer to a byte array where the read data will be stored. Must
 * not be null and must have enough space to store byteCount bytes.
 * @param byteCount The number of bytes to read from the ARM memory. Valid range
 * is 1 to 255 bytes.
 * @return Returns a uint32_t status code indicating the result of the
 * operation, including any recovery actions required.
 ******************************************************************************/
uint32_t TALISE_readArmConfig(taliseDevice_t *device, uint8_t objectId,
			      uint16_t offset, uint8_t *data, uint8_t byteCount);

/***************************************************************************//**
 * @brief This function reads the 64-bit command status register of the Talise
 * ARM processor, parsing it into two separate 16-bit words: an error
 * word and a status word. It is used to determine the status and any
 * errors associated with the first 16 even-numbered opcodes. The
 * function must be called with valid pointers for both the errorWord and
 * statusWord parameters, which will be cleared before being populated
 * with the results. The function returns a status code indicating the
 * success or failure of the operation, including any necessary recovery
 * actions.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param errorWord Pointer to a 16-bit variable where the error word will be
 * stored. Must not be null. The function will clear this
 * variable before use.
 * @param statusWord Pointer to a 16-bit variable where the status word will be
 * stored. Must not be null. The function will clear this
 * variable before use.
 * @return Returns a uint32_t value indicating the recovery action required, if
 * any, based on the operation's success or failure.
 ******************************************************************************/
uint32_t TALISE_readArmCmdStatus(taliseDevice_t *device, uint16_t *errorWord,
				 uint16_t *statusWord);

/***************************************************************************//**
 * @brief This function retrieves the status byte associated with a specific ARM
 * command opcode from the Talise device. It is used to check the status
 * of ARM commands by reading a specific byte from the command status
 * register. The function requires a valid opcode, which must be an even
 * number between 0 and 30 inclusive. If the opcode is invalid, the
 * function returns an error. The status byte is returned through a
 * pointer provided by the caller, and the function returns a status code
 * indicating the success or failure of the operation.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null.
 * @param opCode The opcode of interest, which must be an even number between 0
 * and 30 inclusive. Invalid opcodes result in an error.
 * @param cmdStatByte Pointer to a uint8_t where the command status byte will be
 * stored. Must not be null.
 * @return Returns a uint32_t status code indicating the result of the
 * operation, with specific values for success or various error
 * conditions.
 ******************************************************************************/
uint32_t TALISE_readArmCmdStatusByte(taliseDevice_t *device, uint8_t opCode,
				     uint8_t *cmdStatByte);

/***************************************************************************//**
 * @brief This function is used to monitor the status of an ARM command by
 * polling the ARM status register at specified intervals until the
 * command completes, an error is detected, or a timeout occurs. It is
 * essential to ensure that the ARM firmware is loaded and initialized
 * before calling this function. The function will return immediately if
 * the opcode is invalid or if an error is detected in the command
 * status. The status byte is updated with the command status, which can
 * be used for debugging purposes.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param opCode The opcode of interest, which must be an even number between 0
 * and 30 inclusive. Invalid opcodes will result in an error
 * return.
 * @param cmdStatByte Pointer to a byte where the command status will be stored.
 * Must not be null.
 * @param timeout_us The maximum time in microseconds to wait for the command to
 * complete. Must be greater than zero.
 * @param waitInterval_us The interval in microseconds between status checks. If
 * greater than timeout_us, it will be clamped to
 * timeout_us.
 * @return Returns a uint32_t indicating the recovery action or error status.
 * The cmdStatByte is updated with the command status.
 ******************************************************************************/
uint32_t TALISE_waitArmCmdStatus(taliseDevice_t *device, uint8_t opCode,
				 uint8_t *cmdStatByte, uint32_t timeout_us, uint32_t waitInterval_us);

/***************************************************************************//**
 * @brief This function is used to send a command to the Talise ARM processor
 * interface, which can be done after the ARM processor has been
 * initialized. It requires an opcode and optionally extended data to be
 * sent along with the command. The function ensures that the ARM command
 * interface is ready before sending the command and handles any errors
 * that may occur during the process. It is important to use only even-
 * numbered opcodes up to 30, and the extended data must not exceed 4
 * bytes.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param opCode The opcode of interest, which must be an even number between 0
 * and 30 inclusive. Invalid opcodes will result in an error.
 * @param extendedData A byte array containing extended data to write to the ARM
 * command interface. Can be null if no extended data is
 * needed.
 * @param extendedDataNumBytes Number of bytes in the extendedData array, which
 * must be between 0 and 4 inclusive. Values outside
 * this range will result in an error.
 * @return Returns a uint32_t indicating the recovery action or error status.
 * Possible values include TALACT_NO_ACTION for success, or various
 * error codes indicating specific issues.
 ******************************************************************************/
uint32_t TALISE_sendArmCommand(taliseDevice_t *device, uint8_t opCode,
			       const uint8_t *extendedData, uint8_t extendedDataNumBytes);


/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief Use this function to obtain the major, minor, and release candidate
 * version numbers, as well as the build type of the ARM binary currently
 * loaded in the Talise device. This function should be called only after
 * the ARM firmware has been successfully loaded. It is important to
 * ensure that the ARM is in a loaded state before calling this function,
 * as it will return an error if the ARM is not loaded. The function
 * populates the provided structure with version information, which can
 * be used for debugging or verification purposes.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null and should point to a properly initialized device
 * structure.
 * @param talArmVersionInfo Pointer to a taliseArmVersionInfo_t structure where
 * the ARM version information will be stored. Must not
 * be null. If null, the function will return an error.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. A return value of TALACT_NO_ACTION indicates success,
 * while other values indicate specific errors or required recovery
 * actions.
 ******************************************************************************/
uint32_t TALISE_getArmVersion_v2(taliseDevice_t *device,
				 taliseArmVersionInfo_t *talArmVersionInfo);

/***************************************************************************//**
 * @brief Use this function to obtain the major, minor, and release candidate
 * version numbers of the ARM binary currently loaded in the Talise ARM
 * memory. This function is useful for verifying the firmware version
 * running on the device. It is important to ensure that the pointers
 * provided for the version numbers are not null, as the function will
 * return an error if any of these pointers are null.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null.
 * @param majorVer Pointer to a uint8_t where the major version number will be
 * stored. Must not be null.
 * @param minorVer Pointer to a uint8_t where the minor version number will be
 * stored. Must not be null.
 * @param rcVer Pointer to a uint8_t where the release candidate version number
 * will be stored. Must not be null.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation. A return value of TALACT_NO_ACTION indicates success,
 * while other values indicate specific errors or required recovery
 * actions.
 ******************************************************************************/
uint32_t TALISE_getArmVersion(taliseDevice_t *device, uint8_t *majorVer,
			      uint8_t *minorVer, uint8_t *rcVer);

/***************************************************************************//**
 * @brief Use this function to verify that the checksum of the ARM binary file
 * loaded into the Talise ARM matches the calculated checksum. This
 * function should be called after the ARM binary file has been loaded to
 * ensure its integrity. It waits for a specified timeout period for the
 * checksum calculation to complete. The timeout period and SPI read
 * interval can be adjusted by modifying the macros
 * VERIFY_ARM_CHKSUM_TIMEOUTUS and VERIFY_ARM_CHKSUM_INTERVALUS in
 * talise_user.c. Ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null. The function will handle errors related to invalid
 * parameters.
 * @return Returns a uint32_t value indicating the result of the checksum
 * verification. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_CHECK_TIMER for a timeout error,
 * TALACT_ERR_CHECK_PARAM for a parameter error, TALACT_ERR_RESET_ARM
 * for an ARM reset requirement, and TALACT_WARN_RESET_LOG for a log
 * reset warning.
 ******************************************************************************/
uint32_t TALISE_verifyArmChecksum(taliseDevice_t *device);

/***************************************************************************//**
 * @brief Use this function to obtain a human-readable error message
 * corresponding to a specific ARM error code and source. This is
 * particularly useful for debugging and logging purposes when working
 * with the Talise ARM API. The function should be called with valid
 * error source and error code values. If the error source is not related
 * to the Talise ARM, or if the error code is not recognized, a default
 * message indicating an incorrect handler or an invalid error will be
 * returned. This function is only effective when verbose error messages
 * are enabled in the build configuration.
 *
 * @param errSrc A 32-bit unsigned integer representing the error source from
 * the Talise API. It must correspond to a valid ARM error source.
 * @param errCode A 32-bit unsigned integer representing the error code
 * associated with the error source. It must be a valid error
 * code for the given error source.
 * @return A constant character pointer to a string containing the error
 * message. If verbose error messages are not enabled, an empty string
 * is returned.
 ******************************************************************************/
const char* talGetArmErrorMessage(uint32_t errSrc, uint32_t errCode);

/***************************************************************************//**
 * @brief This function is used to handle errors detected in ARM command
 * operations by analyzing the error code and determining the appropriate
 * recovery action. It should be called whenever an ARM command error is
 * detected to ensure that the system can respond appropriately. The
 * function modifies the recovery action based on the error type and logs
 * the error if necessary. It is important to ensure that the device
 * pointer is valid and that the error handler type is correctly
 * specified before calling this function.
 *
 * @param device Pointer to the Talise device settings data structure. Must not
 * be null, and the caller retains ownership.
 * @param errHdl Specifies the error handler type. Must be a valid error handler
 * enumeration value.
 * @param detErr The detected error code, which is a combination of ARM opcode,
 * object ID, and command status byte. Must be a valid error code.
 * @param retVal The current recovery action value. It is used as a fallback if
 * no specific error handling is required.
 * @param recAction The initial recovery action to be modified based on the
 * error analysis. It is updated by the function to reflect the
 * determined recovery action.
 * @return Returns a talRecoveryActions_t value representing the determined
 * recovery action after processing the detected ARM error.
 ******************************************************************************/
talRecoveryActions_t talArmCmdErrorHandler(taliseDevice_t *device,
		taliseErrHdls_t errHdl,
		uint32_t detErr, talRecoveryActions_t retVal, talRecoveryActions_t recAction);

#define ARMCMD_ERRCODE(armOpCode, armObjId, armErrorFlag) ((armOpCode << 16) | (armObjId << 8) | armErrorFlag)

#ifdef __cplusplus
}
#endif

#endif /* TALISE_ARM_H_ */
