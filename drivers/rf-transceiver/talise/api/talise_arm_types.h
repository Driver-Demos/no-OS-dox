/**
 * \file talise_arm_types.h
 * \brief Contains Talise ARM data types
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_ARM_TYPES_H_
#define TALISE_ARM_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif


/***************************************************************************//**
 * @brief The `talApiArmErr_t` is an enumeration that defines various error
 * states that can occur during the bootup process of an ARM processor in
 * the Talise API. Each enumerator represents a specific error condition,
 * such as a timeout, idle state, radio on state, profile error, or an
 * unknown error, providing a clear and structured way to handle and
 * identify bootup issues.
 *
 * @param TALAPI_ARMERR_BOOTUP_TIMEOUT_ERROR Timed out waiting for ARM bootup to
 * happen.
 * @param TALAPI_ARMERR_BOOTUP_IDLE ARM in IDLE mode after bootup.
 * @param TALAPI_ARMERR_BOOTUP_RADIO_ON ARM in RADIO_ON mode after bootup.
 * @param TALAPI_ARMERR_BOOTUP_PROFILE_ERROR ARM Profile error during bootup.
 * @param TALAPI_ARMERR_BOOTUP_UNKNOWN_ERROR ARM unknown error during bootup.
 ******************************************************************************/
typedef enum {
	TALAPI_ARMERR_BOOTUP_TIMEOUT_ERROR, /*!< Timed out waiting for ARM bootup to happen*/
	TALAPI_ARMERR_BOOTUP_IDLE,          /*!< ARM in IDLE mode after bootup*/
	TALAPI_ARMERR_BOOTUP_RADIO_ON,      /*!< ARM in RADIO_ON mode after bootup*/
	TALAPI_ARMERR_BOOTUP_PROFILE_ERROR, /*!< ARM Profile error during bootup*/
	TALAPI_ARMERR_BOOTUP_UNKNOWN_ERROR  /*!< ARM unknown error during bootup*/
} talApiArmErr_t;

/***************************************************************************//**
 * @brief The `taliseArmBuildType_t` is an enumeration that defines the
 * different build types for an ARM binary in the Talise API. It
 * categorizes the ARM binary into three distinct types: Debug, Test
 * Object, and Release, which are used to specify the nature of the
 * binary during development and deployment.
 *
 * @param TAL_ARM_BUILD_DEBUG ARM binary is Debug Object.
 * @param TAL_ARM_BUILD_TEST_OBJECT ARM binary is Test Object.
 * @param TAL_ARM_BUILD_RELEASE ARM binary is Release.
 ******************************************************************************/
typedef enum {
	TAL_ARM_BUILD_DEBUG,                /*!< ARM binary is Debug Object*/
	TAL_ARM_BUILD_TEST_OBJECT,          /*!< ARM binary is Test Object*/
	TAL_ARM_BUILD_RELEASE               /*!< ARM binary is Release*/
} taliseArmBuildType_t;

/***************************************************************************//**
 * @brief The `taliseArmVersionInfo_t` structure is used to encapsulate version
 * information for the ARM component of the Talise API. It includes
 * fields for the major and minor version numbers, a release candidate
 * version which acts as a build number, and a build type which indicates
 * the nature of the ARM binary (e.g., Debug, Test Object, or Release).
 * This structure is essential for managing and identifying different
 * versions of the ARM software within the Talise system.
 *
 * @param majorVer The ARM Major revision.
 * @param minorVer The ARM Minor revision.
 * @param rcVer The release candidate version (build number).
 * @param buildType What type of ARM binary build.
 ******************************************************************************/
typedef struct {
	uint8_t majorVer;                   /*!< The ARM Major revision*/
	uint8_t minorVer;                   /*!< The ARM Minor revision*/
	uint8_t rcVer;                      /*!< The release candidate version (build number)*/
	taliseArmBuildType_t buildType;        /*!< What type of ARM binary build*/
} taliseArmVersionInfo_t;

#ifdef __cplusplus
}
#endif

#endif /* TALISE_ARM_TYPES_H_ */
