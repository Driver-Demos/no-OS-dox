/**
* \file
* \brief Contains top level ADRV9001 related function prototypes for
*        adi_adrv9001.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_H_
#define _ADI_ADRV9001_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
/* "adi_adrv9001_user.h" contains the #define that other header file use */
#include "adi_adrv9001_user.h"

/* ADI specific header files */
#include "adi_common_macros.h"
#include "adi_adrv9001_types.h"

/* Header files related to libraries */


/* System header files */


/*********************************************************************************************************/
/***************************************************************************//**
 * @brief This function initializes the ADRV9001 device by performing necessary
 * hardware setup, including toggling the RESETB pin and configuring the
 * SPI interface. It should be called after the device's
 * common.devHalInfo has been initialized with user-specific values. This
 * function ensures that all external hardware blocks required for the
 * operation of the ADRV9001 are properly initialized. It is essential
 * for setting up the device before any further operations can be
 * performed.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. The devHalInfo
 * member must be initialized with all necessary information for
 * external hardware initialization, such as power settings and
 * SPI master configuration. Must not be null.
 * @param spiSettings Pointer to the ADRV9001 SPI controller settings, which are
 * specific to the ADRV9001 and not the platform hardware SPI
 * settings. Must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_HwOpen(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_SpiSettings_t *spiSettings);

/*********************************************************************************************************/
/***************************************************************************//**
 * @brief This function initializes the ADRV9001 hardware by calling the
 * appropriate HAL function to set up all necessary external hardware
 * components, such as power and SPI settings, without performing a
 * hardware reset. It should be used when the device needs to be
 * initialized without disrupting its current state. The function
 * requires that the device's context structure has been properly
 * initialized with the necessary hardware information before calling. It
 * returns a status code indicating success or the required recovery
 * action.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. The
 * 'devHalInfo' member must be initialized with all necessary
 * information for hardware initialization. Must not be null.
 * @param spiSettings Pointer to the ADRV9001 SPI controller settings. This
 * structure should contain the desired SPI configuration
 * settings. Must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_HwOpenNoReset(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_SpiSettings_t *spiSettings);

/***************************************************************************//**
 * @brief This function is used to shut down the ADRV9001 device by calling the
 * appropriate hardware abstraction layer (HAL) function to close all
 * external hardware blocks necessary for the device's operation. It
 * should be called after the device's common hardware information has
 * been initialized with user values. This function is typically used
 * when the device is no longer needed or before re-initialization to
 * ensure a clean shutdown.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. The 'devHalInfo' member must be initialized with
 * all the required information to manage the external hardware
 * necessary for the ADRV9001's operation, such as power, pull-
 * ups, and SPI master settings. The pointer must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_HwClose(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function toggles the RESETB pin on the ADRV9001 device to perform
 * a hard reset. It should be called when a reset of the device is
 * required, such as during initialization or error recovery. The
 * function assumes that the device's hardware interface has been
 * properly initialized. It returns a status code indicating success or
 * the necessary recovery action if an error occurs during the reset
 * process.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must be initialized with user values, including
 * the devHalInfo member, which contains necessary information
 * for device operation. The pointer must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if an error occurs.
 ******************************************************************************/
int32_t adi_adrv9001_HwReset(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to initialize the ADRV9001 device with the
 * provided initialization settings and clock divisor. It prepares the
 * device for further configuration by setting up the analog clocks and
 * disabling certain pin modes. This function should be called as the
 * first step in configuring the device after all necessary data
 * structures have been initialized. It does not load the ARM or perform
 * ARM initialization calibrations, leaving the device ready for
 * multichip synchronization and further setup.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must be
 * initialized with all necessary information before calling
 * this function. The caller retains ownership and it must not
 * be null.
 * @param init Pointer to the ADRV9001 initialization settings structure. Must
 * be properly initialized with desired settings before calling this
 * function. The caller retains ownership and it must not be null.
 * @param adrv9001DeviceClockOutDivisor An enum value representing the ADRV9001
 * device clock output divisor, ranging
 * from 0 to 6. The divisor value will be
 * 2^N (1, 2, 4, 8, 16, 32, 64).
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_InitAnalog(adi_adrv9001_Device_t *adrv9001,
                                adi_adrv9001_Init_t *init,
                                adi_adrv9001_DeviceClockDivisor_e adrv9001DeviceClockOutDivisor);

/***************************************************************************//**
 * @brief This function is used to safely shut down the ADRV9001 device by
 * performing a hardware reset, ensuring the device is in a safe state
 * for shutdown or re-initialization. It can be called at any time,
 * provided that the device's common hardware information has been
 * configured with user settings. This function is essential for ensuring
 * that the device is properly reset and ready for any subsequent
 * operations or power cycles.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should be properly initialized with user
 * device settings before calling this function. If the pointer
 * is null, the function will handle it as an error.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Shutdown(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function configures the SPI settings for the ADRV9001 device
 * using the provided SPI settings structure. It adjusts parameters such
 * as SPI stream mode, address auto increment direction, MSB/LSB first,
 * and 3-wire/4-wire mode. The function must be called with a valid
 * device context and SPI settings structure. It is a helper function and
 * typically does not need to be called directly by the user. The
 * function returns a code indicating success or the required action to
 * recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param spi Pointer to the ADRV9001 SPI controller settings. Must not be null.
 * The caller retains ownership. The function checks for valid CMOS
 * pad drive strength values and reports an error if invalid.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Configure(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_SpiSettings_t *spi);

/***************************************************************************//**
 * @brief Use this function to obtain the current SPI configuration settings of
 * the ADRV9001 device, including the stream mode, address auto increment
 * direction, bit order, and wire mode. This function should be called
 * only after the SPI configuration has been set or verified to ensure
 * accurate retrieval of settings. It is a helper function and is not
 * intended for direct user invocation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param spi Pointer to a structure where the current SPI settings will be
 * stored. Must not be null. The caller retains ownership.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Inspect(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_SpiSettings_t *spi);

/***************************************************************************//**
 * @brief This function checks the SPI settings of the ADRV9001 device to ensure
 * they are functioning correctly. It performs read and write operations
 * on specific registers to verify SPI communication. This function
 * should be used to confirm that the SPI settings configured through
 * other API functions are correct. It is a diagnostic tool and does not
 * need to be called directly by the user under normal circumstances. The
 * function assumes that the device context has been properly initialized
 * before calling.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must be
 * initialized with the necessary device context information.
 * The pointer must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if an error is detected.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Verify(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief Use this function to obtain the version number of the ADRV9001 API,
 * which is useful for ensuring compatibility and debugging. It requires
 * a valid device context and a pointer to a structure where the version
 * information will be stored. This function should be called when you
 * need to verify the API version being used with the ADRV9001 device.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param apiVersion Pointer to a structure where the API version information
 * will be stored. Must not be null, and the caller is
 * responsible for allocating this structure.
 * @return Returns an integer code indicating success or the required action to
 * recover. The API version information is populated in the provided
 * apiVersion structure.
 ******************************************************************************/
int32_t adi_adrv9001_ApiVersion_Get(adi_adrv9001_Device_t *adrv9001, adi_common_ApiVersion_t *apiVersion);

/***************************************************************************//**
 * @brief This function retrieves the silicon version of the ADRV9001 device by
 * reading the relevant register and extracting the major and minor
 * version numbers. It should be called when the user needs to verify or
 * log the silicon version of the device. The function requires a valid
 * device context and a pointer to a structure where the silicon version
 * will be stored. It is important to ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param siliconVersion Pointer to a structure where the silicon version will
 * be stored. Must not be null. The function will populate
 * this structure with the major and minor version
 * numbers.
 * @return Returns an integer code indicating success or the required action to
 * recover. The silicon version is output through the siliconVersion
 * parameter.
 ******************************************************************************/
int32_t adi_adrv9001_SiliconVersion_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_SiliconVersion_t *siliconVersion);

/***************************************************************************//**
 * @brief Use this function to ensure that the Rx, Tx, and ORx profiles within
 * the initialization structure have compatible clock rates for operation
 * with the ADRV9001 device. It checks for valid combinations of profiles
 * that share a common high-speed digital clock. If any profile is
 * unused, all its members should be zeroed out. This function should be
 * called after setting up the initialization structure but before
 * proceeding with further device configuration.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param init Pointer to the ADRV9001 initialization settings structure. Must
 * not be null and should contain the profiles to be verified.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or an
 * error code if an invalid profile combination is detected.
 ******************************************************************************/
int32_t adi_adrv9001_Profiles_Verify(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief Use this function to retrieve the current temperature from the
 * ADRV9001 device's internal temperature sensor. It is suitable for
 * monitoring the device's thermal state and can be called when the
 * device is in any of the following states: STANDBY, CALIBRATED, PRIMED,
 * or RF_ENABLED. Ensure that the device context is properly initialized
 * before calling this function. The function writes the temperature
 * value to the provided pointer and returns a status code indicating
 * success or the necessary action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must be properly initialized and not null.
 * @param temperature_C Pointer to an int16_t where the temperature in Celsius
 * will be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Temperature_Get(adi_adrv9001_Device_t *adrv9001, int16_t *temperature_C);

/***************************************************************************//**
 * @brief This function retrieves the part number of the ADRV9001 device by
 * reading specific configuration registers. It should be called when the
 * part number needs to be identified, typically during initialization or
 * diagnostics. The function requires a valid device context and a
 * pointer to store the part number. If the part number is not
 * recognized, it sets the part number to an unknown value and reports an
 * error. Ensure the device is properly initialized before calling this
 * function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function. The caller retains ownership.
 * @param partNumber Pointer to a variable of type adi_adrv9001_PartNumber_e
 * where the part number will be stored. Must not be null. If
 * the part number is unrecognized, it will be set to
 * ADI_ADRV9001_PART_NUMBER_UNKNOWN.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The part number is written to the location pointed to by
 * partNumber.
 ******************************************************************************/
int32_t adi_adrv9001_PartNumber_Get(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_PartNumber_e *partNumber);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_H_ */
