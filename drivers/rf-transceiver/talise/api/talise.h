/**
 * \file talise.h
 * \brief Contains top level Talise related function prototypes for
 *        talise.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_H_
#define TALISE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "talise_types.h"

/****************************************************************************
 * Initialization functions
 ****************************************************************************
 */
/***************************************************************************//**
 * @brief This function initializes the hardware components necessary for the
 * operation of the Talise device by calling the ADI HAL function
 * ADIHAL_openHw. It sets the HAL timeout limit and prepares the device
 * for further operations. This function should be called after the
 * device's devHalInfo member has been properly initialized with the
 * necessary hardware configuration details. It handles various hardware
 * initialization errors by returning specific recovery actions.
 *
 * @param device Pointer to a taliseDevice_t structure. The devHalInfo member
 * must be initialized with the necessary information for hardware
 * initialization, such as power settings, pull-ups, and SPI
 * master configuration. The pointer must not be null.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for successful
 * completion, or specific error codes indicating the need for recovery
 * actions such as TALACT_ERR_RESET_SPI, TALACT_ERR_RESET_GPIO,
 * TALACT_ERR_CHECK_TIMER, or TALACT_ERR_CHECK_PARAM.
 ******************************************************************************/
uint32_t TALISE_openHw(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function is used to shut down the hardware components of a Talise
 * device by calling the ADI HAL function ADIHAL_closeHw. It should be
 * called when the device is no longer needed or before re-
 * initialization. The function requires that the device's devHalInfo
 * member is properly initialized with user values. It handles various
 * hardware errors by returning specific recovery actions, ensuring that
 * the device is safely shut down.
 *
 * @param device Pointer to a taliseDevice_t structure. The devHalInfo member
 * must be initialized with all necessary information for shutting
 * down the external hardware components of the Talise device. The
 * pointer must not be null.
 * @return Returns a uint32_t value indicating the result of the shutdown
 * operation. Possible return values include TALACT_NO_ACTION for
 * successful completion, or specific error codes indicating the type of
 * recovery action required.
 ******************************************************************************/
uint32_t TALISE_closeHw(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function toggles the RESETB pin on the Talise device, effectively
 * performing a hard reset. It should be called any time after the
 * device's hardware information has been initialized. The function is
 * designed to reset only the device associated with the SPI chip select
 * specified in the device's settings. It handles potential hardware
 * communication errors and attempts to ensure the device is reset
 * properly. This function is useful for recovering from errors or
 * reinitializing the device to a known state.
 *
 * @param device Pointer to a taliseDevice_t structure containing the device
 * settings. The devHalInfo member must be initialized with the
 * necessary hardware information. The pointer must not be null,
 * and the function will handle errors related to hardware
 * communication.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_WARN_RESET_LOG,
 * TALACT_WARN_RESET_GPIO, TALACT_ERR_RESET_GPIO, and TALACT_NO_ACTION,
 * which indicate the success of the operation or the type of recovery
 * action needed.
 ******************************************************************************/
uint32_t TALISE_resetDevice(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function sets up the Talise device according to the provided
 * initialization settings, configuring various components such as the
 * CLKPLL, digital clocks, JESD204b settings, and FIR filters. It
 * prepares the device for further operations like multichip
 * synchronization and ARM initialization. The function must be called
 * after all necessary data structures have been initialized. It handles
 * parameter validation and returns specific error codes for invalid
 * inputs or other issues encountered during initialization.
 *
 * @param device Pointer to a taliseDevice_t structure that must be initialized
 * with necessary hardware information before calling this
 * function. Must not be null.
 * @param init Pointer to a taliseInit_t structure containing the initialization
 * settings for the device. Must not be null.
 * @return Returns a uint32_t value indicating the result of the initialization
 * process. Possible return values include TALACT_NO_ACTION for success,
 * TALACT_ERR_CHECK_PARAM for parameter errors, and other codes
 * indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_initialize(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief Use this function to safely shut down the Talise device by performing
 * a hardware reset, ensuring the device is in a safe state for shutdown
 * or re-initialization. It can be called at any time, provided the
 * device's hardware information has been configured with user settings.
 * This function is essential for ensuring the device is properly reset
 * before being powered down or reconfigured.
 *
 * @param device Pointer to a taliseDevice_t structure containing the device
 * settings. The device's devHalInfo must be initialized with the
 * necessary information for hardware operation. The pointer must
 * not be null, and the function will handle invalid input by
 * returning an appropriate recovery action.
 * @return Returns a uint32_t value indicating the recovery action required.
 * Possible values include TALACT_NO_ACTION for successful completion or
 * other values indicating specific recovery actions needed.
 ******************************************************************************/
uint32_t TALISE_shutdown(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function retrieves the status of the multi-chip synchronization
 * (MCS) state machine from the Talise device and stores it in the
 * provided mcsStatus pointer. It is intended to be used after the device
 * has been initialized and the PLL lock status has been verified. The
 * function checks the synchronization status of various components such
 * as JESD, digital clocks, CLK PLL, and device clock divider. A bit
 * value of 1 in the mcsStatus indicates that synchronization has
 * occurred, while a bit value of 0 indicates it has not. The function
 * requires a valid pointer for mcsStatus and will return an error if it
 * is null.
 *
 * @param device Pointer to the Talise device data structure. Must be
 * initialized and configured before calling this function.
 * @param mcsStatus Pointer to a uint8_t where the MCS status will be stored.
 * Must not be null. The 4 LSBs of the value represent the sync
 * status of JESD, digital clocks, CLK PLL, and device clock
 * divider.
 * @return Returns a uint32_t indicating the recovery action required. Possible
 * values include TALACT_NO_ACTION for successful completion,
 * TALACT_ERR_CHECK_PARAM for a bad parameter check, and
 * TALACT_ERR_RESET_SPI for an SPI reset requirement.
 ******************************************************************************/
uint32_t TALISE_getMultiChipSyncStatus(taliseDevice_t *device,
				       uint8_t *mcsStatus);

/***************************************************************************//**
 * @brief This function is used to enable or reset the multichip synchronization
 * (MCS) state machine in the Talise device, which is necessary for
 * deterministic latency in systems with multiple transceivers. It should
 * be called after all transceivers have been initialized and before
 * starting the ARM initialization. When `enableMcs` is set to 1, the
 * function resets the MCS state machine, and when set to 0, it allows
 * reading the MCS status. The function can optionally return the MCS
 * status through the `mcsStatus` parameter if it is not null. This
 * function must be called after the device has been initialized and the
 * PLL lock status has been verified.
 *
 * @param device Pointer to the Talise device data structure. Must be
 * initialized and not null.
 * @param enableMcs A uint8_t value where 1 enables/resets the MCS state machine
 * and 0 allows reading the MCS status.
 * @param mcsStatus Optional pointer to a uint8_t where the MCS status will be
 * stored if not null. The status reflects the sync status of
 * various components.
 * @return Returns a uint32_t indicating the recovery action required, such as
 * TALACT_NO_ACTION for success or other values for specific error
 * handling.
 ******************************************************************************/
uint32_t TALISE_enableMultichipSync(taliseDevice_t *device, uint8_t enableMcs,
				    uint8_t *mcsStatus);

/***************************************************************************//**
 * @brief This function is used to reset the JESD204 serializer of the Talise
 * device. It should be called when a reset of the serializer is
 * required, such as during initialization or error recovery. The
 * function interacts with the device's hardware abstraction layer to
 * toggle the serializer's reset state, ensuring that the serializer is
 * properly reset and ready for operation. It is important to ensure that
 * the device's hardware interface is correctly initialized before
 * calling this function.
 *
 * @param device A pointer to the Talise device data structure. This structure
 * must be properly initialized with the necessary hardware
 * information before calling this function. The pointer must not
 * be null, and the function will handle invalid pointers by
 * returning an error code.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for successful
 * completion, TALACT_ERR_RESET_SPI if a SPI reset is required, and
 * other error codes indicating specific recovery actions.
 ******************************************************************************/
uint32_t TALISE_serializerReset(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function is used to synchronize the RF local oscillators (LOs)
 * across multiple Talise devices, which is essential for applications
 * like active antenna systems and beamforming. It should be called after
 * setting the RF PLL frequency and before running initial calibrations.
 * The function can enable or disable the digital test clock, affecting
 * the MCS state machine and clock synchronization. It is important to
 * ensure the device is initialized, the ARM is loaded, and PLL lock
 * status is verified before calling this function.
 *
 * @param device A pointer to the Talise device data structure. It must be
 * initialized and configured with the necessary device settings
 * before calling this function.
 * @param enableDigTestClk A uint8_t value where 1 enables and resets the MCS
 * state machine, switching between device clock scaled
 * and HSDIGCLK, and 0 disables the digital test clock.
 * Values outside this range are treated as 0.
 * @return Returns a uint32_t value indicating the recovery action required,
 * such as TALACT_NO_ACTION for success or various error codes for
 * different failure scenarios.
 ******************************************************************************/
uint32_t TALISE_enableMultichipRfLOPhaseSync(taliseDevice_t *device,
		uint8_t enableDigTestClk);

/****************************************************************************
 * Runtime functions
 ****************************************************************************
 */

/****************************************************************************
 * Helper functions
 ****************************************************************************
 */
/***************************************************************************//**
 * @brief This function is used to program a specific FIR filter in the Talise
 * device by writing the provided filter coefficients and settings to the
 * appropriate filter bank. It is typically called during device
 * initialization but can also be used to update a filter configuration
 * later, which may require recalibration. The function requires a valid
 * device structure, a filter identifier, and a filter configuration
 * structure. It performs parameter validation and returns an error if
 * the number of coefficients exceeds the allowed maximum or if the
 * filter gain is invalid.
 *
 * @param device Pointer to a taliseDevice_t structure representing the Talise
 * device. Must be initialized and not null.
 * @param filterToProgram An identifier of type talisefirName_t specifying which
 * FIR filter to program. Must be a valid filter name;
 * otherwise, an error is returned.
 * @param firFilter Pointer to a taliseFir_t structure containing the filter
 * coefficients and gain settings. Must not be null, and the
 * number of coefficients must be within the allowed range for
 * the specified filter.
 * @return Returns a uint32_t value indicating the result of the operation, with
 * specific error codes for invalid parameters or successful completion.
 ******************************************************************************/
uint32_t TALISE_programFir(taliseDevice_t *device,
			   talisefirName_t filterToProgram, taliseFir_t *firFilter);

/***************************************************************************//**
 * @brief This function calculates the internal high-speed digital clock
 * frequencies for the Talise device based on the provided digital clock
 * settings. It updates the device's state with the calculated clock
 * frequencies, which are used by other API functions. This function
 * should be called after the device's clock settings have been
 * initialized. It handles invalid high-speed divider values by returning
 * an error code.
 *
 * @param device Pointer to a Talise device data structure. Must not be null and
 * should be properly initialized with device settings.
 * @param digClocks Pointer to a structure containing the digital clock
 * initialization data. Must not be null and should contain
 * valid clock settings.
 * @return Returns a uint32_t value indicating the success or failure of the
 * operation. A return value of TALACT_NO_ACTION indicates success,
 * while other values indicate specific error conditions.
 ******************************************************************************/
uint32_t TALISE_calculateDigitalClocks(taliseDevice_t *device,
				       taliseDigClocks_t *digClocks);

/***************************************************************************//**
 * @brief Use this function to ensure that the Rx, Tx, and ORx profiles
 * specified in the initialization structure are valid and compatible
 * with each other. It checks the clock rates and other parameters to
 * confirm that the profiles can operate together without conflicts. This
 * function should be called after the device and initialization
 * structures are properly set up. It returns an error if any profile is
 * invalid or if there is a mismatch in the expected clock rates.
 *
 * @param device Pointer to a taliseDevice_t structure representing the Talise
 * device. Must not be null. The function will return an error if
 * this parameter is null.
 * @param init Pointer to a taliseInit_t structure containing the initialization
 * settings for the Talise device. Must not be null. The function
 * will return an error if this parameter is null.
 * @return Returns a uint32_t value indicating the result of the verification. A
 * return value of TALACT_NO_ACTION indicates success, while other
 * values indicate specific errors or required recovery actions.
 ******************************************************************************/
uint32_t TALISE_verifyProfiles(taliseDevice_t *device, taliseInit_t *init);

/***************************************************************************//**
 * @brief This function configures the SPI settings for the Talise device using
 * the parameters provided in the `spi` structure. It sets the SPI mode,
 * address auto-increment direction, bit order, and wire mode. The
 * function must be called with a properly initialized `device` and `spi`
 * structure. It handles invalid CMOS pad drive strength values by
 * returning an error. The function also verifies the SPI settings after
 * configuration to ensure they work correctly.
 *
 * @param device Pointer to a `taliseDevice_t` structure representing the Talise
 * device. Must be initialized with valid device information. The
 * caller retains ownership.
 * @param spi Pointer to a `taliseSpiSettings_t` structure containing the
 * desired SPI settings. Must not be null and should be properly
 * initialized with valid settings. The caller retains ownership.
 * @return Returns a `uint32_t` value indicating the result of the operation.
 * Possible return values include `TALACT_NO_ACTION` for success, or
 * error codes indicating specific issues encountered during
 * configuration.
 ******************************************************************************/
uint32_t TALISE_setSpiSettings(taliseDevice_t *device,
			       taliseSpiSettings_t *spi);

/***************************************************************************//**
 * @brief This function checks the SPI communication by performing read and
 * write operations on specific registers of the Talise device. It is
 * used to ensure that the SPI settings are correctly configured and
 * functioning as expected. The function reads from known read-only
 * registers and writes to a scratchpad register, verifying the data
 * integrity by reading back the written values. It should be called
 * after setting the SPI configuration to validate the setup. The
 * function returns a status code indicating the success or failure of
 * the verification process.
 *
 * @param device Pointer to a taliseDevice_t structure containing the device
 * settings. The devHalInfo member must be initialized with the
 * necessary information for SPI operations. The pointer must not
 * be null, and the function will handle invalid pointers by
 * returning an error code.
 * @return Returns a uint32_t status code indicating the result of the SPI
 * verification. Possible return values include TALACT_NO_ACTION for
 * success, TALACT_ERR_RESET_SPI for SPI reset required, and
 * TALACT_ERR_RESET_FULL for a full chip reset required.
 ******************************************************************************/
uint32_t TALISE_verifySpiReadWrite(taliseDevice_t *device);

/***************************************************************************//**
 * @brief This function configures the digital clocks of the Talise device based
 * on the provided clock settings. It must be called after the device has
 * been properly initialized and before any operations that depend on the
 * digital clocks. The function validates the input clock settings and
 * adjusts the internal clock dividers and synthesizer settings
 * accordingly. It handles various clock frequency ranges and ensures
 * that the PLL and VCO are set up correctly. If the input parameters are
 * invalid, the function returns an error code indicating the type of
 * error encountered.
 *
 * @param device Pointer to a taliseDevice_t structure representing the Talise
 * device. Must be initialized with valid device information. The
 * caller retains ownership and it must not be null.
 * @param clockSettings Pointer to a taliseDigClocks_t structure containing the
 * desired clock settings. Must be initialized with valid
 * clock configuration data. The caller retains ownership
 * and it must not be null.
 * @return Returns a uint32_t value indicating the result of the operation.
 * Possible return values include TALACT_NO_ACTION for success, or
 * various error codes for parameter validation failures or other
 * issues.
 ******************************************************************************/
uint32_t TALISE_initDigitalClocks(taliseDevice_t *device,
				  taliseDigClocks_t *clockSettings);
/****************************************************************************
 * Debug functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function configures the digital clock divider used to synchronize
 * the Tx PFIR each time the Tx channel powers up. It should be called
 * during device initialization and is typically not invoked directly by
 * the user. The function ensures that the sync clock is set to the FIR's
 * output clock divided by two or slower. It requires a valid Tx profile
 * and checks for valid FIR coefficients and interpolation settings. If
 * the parameters are invalid, it returns an error code indicating the
 * issue.
 *
 * @param device Pointer to the Talise device data structure. Must be
 * initialized and not null.
 * @param txProfile Pointer to the Tx profile settings. Must be initialized and
 * not null, with valid FIR coefficients and interpolation
 * settings.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation. Possible return values include TALACT_NO_ACTION for
 * success, or error codes for various parameter validation failures.
 ******************************************************************************/
uint32_t TALISE_setTxPfirSyncClk(taliseDevice_t *device,
				 taliseTxProfile_t *txProfile);

/***************************************************************************//**
 * @brief This function configures the digital clock divider used to synchronize
 * the Rx and ORx PFIRs each time the channels power up. It ensures that
 * the sync clock is set to the slowest of the Rx or ORx PFIR output
 * clock divided by two. This function is typically called automatically
 * during the initialization process and should not need to be called
 * directly by the user.
 *
 * @param device Pointer to the Talise device data structure containing
 * settings. Must not be null.
 * @param rxProfile Pointer to the Rx profile settings. Must not be null and
 * should contain valid configuration data.
 * @param orxProfile Pointer to the ORx profile settings. Must not be null and
 * should contain valid configuration data.
 * @return Returns a uint32_t indicating the status of the operation. Possible
 * return values include TALACT_NO_ACTION for successful completion,
 * TALACT_ERR_CHECK_PARAM for parameter errors, and TALACT_ERR_RESET_SPI
 * for SPI reset requirements.
 ******************************************************************************/
uint32_t TALISE_setRxPfirSyncClk(taliseDevice_t *device,
				 taliseRxProfile_t *rxProfile, taliseORxProfile_t *orxProfile);

/***************************************************************************//**
 * @brief Use this function to obtain the current version numbers of the Talise
 * API, which include the silicon, major, minor, and build versions. This
 * function is useful for verifying compatibility and ensuring that the
 * correct API version is being used. It is important to ensure that all
 * pointer parameters are valid and non-null before calling this
 * function, as passing null pointers will result in an error.
 *
 * @param device Pointer to the Talise device data structure. Must be properly
 * initialized before calling this function.
 * @param siVer Pointer to a uint32_t where the silicon version number will be
 * stored. Must not be null.
 * @param majorVer Pointer to a uint32_t where the major version number will be
 * stored. Must not be null.
 * @param minorVer Pointer to a uint32_t where the minor version number will be
 * stored. Must not be null.
 * @param buildVer Pointer to a uint32_t where the build version number will be
 * stored. Must not be null.
 * @return Returns a uint32_t indicating the success or failure of the
 * operation. A return value of TALACT_NO_ACTION indicates success,
 * while other values indicate specific error conditions.
 ******************************************************************************/
uint32_t TALISE_getApiVersion(taliseDevice_t *device, uint32_t *siVer,
			      uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildVer);

/***************************************************************************//**
 * @brief Use this function to obtain the silicon revision of a Talise device,
 * which is useful for identifying the specific version of the hardware
 * in use. This function should be called after the device has been
 * properly initialized. It requires a valid pointer to a
 * `taliseDevice_t` structure and a non-null pointer to a `uint8_t`
 * variable where the revision will be stored. The function handles null
 * pointers by returning an error code, ensuring that the caller is
 * informed of invalid input.
 *
 * @param device Pointer to a `taliseDevice_t` structure containing the device
 * settings. Must be initialized and not null.
 * @param revision Pointer to a `uint8_t` where the silicon revision will be
 * stored. Must not be null. If null, the function returns an
 * error code.
 * @return Returns a `uint32_t` indicating the success or type of error
 * encountered. A successful call results in the silicon revision being
 * stored in the provided `revision` pointer.
 ******************************************************************************/
uint32_t TALISE_getDeviceRev(taliseDevice_t *device, uint8_t *revision);

/***************************************************************************//**
 * @brief This function retrieves the Product ID of the Talise device and stores
 * it in the provided memory location. It should be called when the
 * Product ID is needed for identification or verification purposes. The
 * function requires a valid device structure and a non-null pointer for
 * storing the Product ID. If the provided pointer is null, the function
 * will handle this as an error and return an appropriate error code. The
 * function returns a status code indicating the success or failure of
 * the operation, which can be used to determine if any recovery actions
 * are necessary.
 *
 * @param device Pointer to a taliseDevice_t structure containing the device
 * settings. The device structure must be properly initialized
 * before calling this function.
 * @param productId Pointer to a uint8_t where the Product ID will be stored.
 * Must not be null, as a null pointer will result in an error.
 * @return Returns a uint32_t status code indicating the result of the
 * operation. Possible values include success or specific error codes
 * that suggest recovery actions.
 ******************************************************************************/
uint32_t TALISE_getProductId(taliseDevice_t *device, uint8_t *productId);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_H_ */
