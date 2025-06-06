/**
 * \file
 * \brief Contains ADRV9001 ARM data types
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_ARM_TYPES_H_
#define _ADI_ADRV9001_ARM_TYPES_H_

#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_gpio_types.h"

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmSystemStates_e` is an enumeration that defines
 * the various operational states of the ARM system within the ADRV9001
 * device. These states include the initial Powerup state, the Normal
 * operational mode which can be either TDD (Time Division Duplex) or FDD
 * (Frequency Division Duplex), a Power Saving mode to reduce energy
 * consumption, and the MCS state which is used for synchronizing
 * multiple chips. This enumeration is crucial for managing and
 * transitioning between different system states in the ARM processor of
 * the ADRV9001.
 *
 * @param ADI_ADRV9001_ARM_SYSTEM_POWERUP Represents the Powerup State of the
 * ARM system.
 * @param ADI_ADRV9001_ARM_SYSTEM_NORMALMODE Represents the Normal TDD/FDD State
 * of the ARM system.
 * @param ADI_ADRV9001_ARM_SYSTEM_POWERSAVINGMODE Represents the System Power
 * Saving Mode of the ARM system.
 * @param ADI_ADRV9001_ARM_SYSTEM_MCS Represents the MCS (Multi-Chip
 * Synchronization) State of the ARM system.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmSystemStates
{
    ADI_ADRV9001_ARM_SYSTEM_POWERUP,         /*!< Powerup State */
    ADI_ADRV9001_ARM_SYSTEM_NORMALMODE,      /*!< Normal TDD/FDD State */
    ADI_ADRV9001_ARM_SYSTEM_POWERSAVINGMODE, /*!< System Power Saving Mode */
    ADI_ADRV9001_ARM_SYSTEM_MCS              /*!< MCS State */
} adi_adrv9001_ArmSystemStates_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmMcsStates_e` is an enumeration that defines the
 * various states of the MCS (Multi-Chip Synchronization) process in the
 * ADRV9001 ARM system. It includes states such as READY, TRANSITION, and
 * DONE, which represent different stages of the MCS process, as well as
 * a RESERVED state for potential future use or undefined behavior. This
 * enumeration is used to manage and track the synchronization status of
 * multiple chips in the system.
 *
 * @param ADI_ADRV9001_ARMMCSSTATES_READY MCS Ready state.
 * @param ADI_ADRV9001_ARMMCSSTATES_TRANSITION MCS Transition state.
 * @param ADI_ADRV9001_ARMMCSSTATES_DONE MCS Done state.
 * @param ADI_ADRV9001_ARMMCSSTATES_RESERVED Reserved state for future use or
 * undefined behavior.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmMcsStates
{
    ADI_ADRV9001_ARMMCSSTATES_READY,      /*!< MCS Ready state */
    ADI_ADRV9001_ARMMCSSTATES_TRANSITION, /*!< MCS Transition state */
    ADI_ADRV9001_ARMMCSSTATES_DONE,       /*!< MCS Done state */
    ADI_ADRV9001_ARMMCSSTATES_RESERVED
} adi_adrv9001_ArmMcsStates_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmBootStates_e` is an enumeration that defines
 * various states of the ARM boot process for the ADRV9001 device. Each
 * enumerator represents a specific state or error condition that can
 * occur during the boot process, such as successful boot, checksum
 * errors, hardware initialization errors, and other specific setup
 * errors. This enumeration is used to track and manage the boot state of
 * the ARM processor within the ADRV9001 system, providing a mechanism to
 * identify and handle different boot scenarios and errors.
 *
 * @param ADI_ADRV9001_ARM_BOOT_POWERUP Used to put API in wait for ARM state.
 * @param ADI_ADRV9001_ARM_BOOT_READY ARM booted with no failure.
 * @param ADI_ADRV9001_ARM_BOOT_FW_CHECKSUM_ERR ARM firmware checksum error.
 * @param ADI_ADRV9001_ARM_BOOT_EFUSE_DATA_ERR Efuse data error.
 * @param ADI_ADRV9001_ARM_BOOT_STATE_DATAMEM_ERR ARM data memory error.
 * @param ADI_ADRV9001_ARM_BOOT_DEVICE_PROFILE_CHECKSUM_ERR Device profile
 * checksum error.
 * @param ADI_ADRV9001_ARM_BOOT_CLKGEN_ERR Bootup clkgen setup error.
 * @param ADI_ADRV9001_ARM_BOOT_CLKSSI_ERR Bootup SSI setup error.
 * @param ADI_ADRV9001_ARM_BOOT_DEVICE_PROFILE_INIT_ERR Device profile init
 * setup error.
 * @param ADI_ADRV9001_ARM_BOOT_JTAG_BUILD_STATUS_READY JTAG build status ready
 * indication.
 * @param ADI_ADRV9001_ARM_BOOT_CLKLOGEN_ERR Bootup clock LOGEN error.
 * @param ADI_ADRV9001_ARM_BOOT_RXQECHW_ERR Error initializing RxQEC hardware.
 * @param ADI_ADRV9001_ARM_BOOT_HM_TIMER_ERR Failed to create health monitor
 * timers.
 * @param ADI_ADRV9001_ARM_BOOT_ADC_RCAL_ERR ADC RCAL error.
 * @param ADI_ADRV9001_ARM_BOOT_STATE_ADC_CCAL_ERR ADC CCAL error.
 * @param ADI_ADRV9001_ARM_BOOT_STATE_STREAM_RUNTIME_ERR Stream Run error.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmBootStates
{
    ADI_ADRV9001_ARM_BOOT_POWERUP                           = 0,    /*!< Used to put API in wait for ARM state */
    ADI_ADRV9001_ARM_BOOT_READY                             = 1,    /*!< ARM booted with no failure */
    ADI_ADRV9001_ARM_BOOT_FW_CHECKSUM_ERR                   = 2,    /*!< ARM firmware checksum error */
    ADI_ADRV9001_ARM_BOOT_EFUSE_DATA_ERR                    = 3,    /*!< Efuse data error */
    ADI_ADRV9001_ARM_BOOT_STATE_DATAMEM_ERR                 = 4,    /*!< ARM data memory error */
    ADI_ADRV9001_ARM_BOOT_DEVICE_PROFILE_CHECKSUM_ERR       = 5,    /*!< Device profile checksum error */
    ADI_ADRV9001_ARM_BOOT_CLKGEN_ERR                        = 6,    /*!< Bootup clkgen setup error */
    ADI_ADRV9001_ARM_BOOT_CLKSSI_ERR                        = 7,    /*!< Bootup SSI setup error */
    ADI_ADRV9001_ARM_BOOT_DEVICE_PROFILE_INIT_ERR           = 8,    /*!< Device profile init setup error */
    ADI_ADRV9001_ARM_BOOT_JTAG_BUILD_STATUS_READY           = 9,    /*!< JTAG build status ready indication */
    ADI_ADRV9001_ARM_BOOT_CLKLOGEN_ERR                      = 10,   /*!< Bootup clock LOGEN error */
    ADI_ADRV9001_ARM_BOOT_RXQECHW_ERR                       = 11,   /*!< Error initializing RxQEC hardware */
    ADI_ADRV9001_ARM_BOOT_HM_TIMER_ERR                      = 12,   /*!< Failed to create health monitor timers */
    ADI_ADRV9001_ARM_BOOT_ADC_RCAL_ERR                      = 13,   /*!< ADC RCAL error */
    ADI_ADRV9001_ARM_BOOT_STATE_ADC_CCAL_ERR                = 14,   /*!< ADC CCAL error */
    ADI_ADRV9001_ARM_BOOT_STATE_STREAM_RUNTIME_ERR          = 15,   /*!< Stream Run error */
} adi_adrv9001_ArmBootStates_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmBuildType_e` is an enumeration that defines the
 * different build types for the ARM image in the ADRV9001 system. It
 * includes options for release, debug, and test object builds, allowing
 * developers to specify the type of build they are working with, which
 * can influence debugging and testing processes.
 *
 * @param ADI_ADRV9001_ARMBUILD_RELEASE Represents the release build type with a
 * value of 0.
 * @param ADI_ADRV9001_ARMBUILD_DEBUG Represents the debug build type with a
 * value of 1.
 * @param ADI_ADRV9001_ARMBUILD_TESTOBJ Represents the test object build type
 * with a value of 2.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmBuildType
{
    ADI_ADRV9001_ARMBUILD_RELEASE = 0,
    ADI_ADRV9001_ARMBUILD_DEBUG = 1,
    ADI_ADRV9001_ARMBUILD_TESTOBJ = 2
} adi_adrv9001_ArmBuildType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmClockType_e` is an enumeration that defines the
 * types of ARM clock configurations available in the ADRV9001 system. It
 * currently includes a clock type for the data path and a maximum
 * boundary marker, which can be used for validation or iteration
 * purposes. This enumeration is part of the broader set of ARM-related
 * configurations and states used in the ADRV9001 API.
 *
 * @param ADI_ADRV9001_ARMCLOCK_DATAPATH Represents the ARM clock type for the
 * data path.
 * @param ADI_ADRV9001_ARMCLOCK_MAX Serves as a boundary marker for the ARM
 * clock type enumeration.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmClockType
{
    ADI_ADRV9001_ARMCLOCK_DATAPATH = 0,
    ADI_ADRV9001_ARMCLOCK_MAX
} adi_adrv9001_ArmClockType_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmSingleSpiWriteMode_e` is an enumeration that
 * defines different modes for single SPI write operations in the
 * ADRV9001 ARM system. It specifies the number of bytes that can be
 * written in a single SPI operation, either in standard or streaming
 * mode, and describes how these operations interact with the chip select
 * signal. This allows for flexible configuration of SPI write operations
 * based on the required data throughput and operational mode.
 *
 * @param ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4 4 bytes in a
 * single SPI
 * write in
 * standard mode,
 * with one write
 * operation
 * regardless of
 * chip select
 * state.
 * @param ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252 252 bytes in
 * a single SPI
 * write in
 * standard
 * mode, with
 * one write
 * operation
 * regardless
 * of chip
 * select
 * state.
 * @param ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STREAMING_BYTES_4 4 bytes in a
 * single SPI
 * write in
 * streaming
 * mode, with
 * four write
 * operations in
 * one chip
 * select.
 ******************************************************************************/
typedef enum adi_adrv9001_ArmSingleSpiWriteMode
{
    ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_4,    /*!< 4 bytes in a single SPI write in standard mode */
                                                                /*!< Only one write operation (4 bytes per write operation) is performed regardless of the state of chip select */
    ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STANDARD_BYTES_252,  /*!< 252 bytes in a single SPI write in standard mode */
                                                                /*!< Only one write operation (up to 252 bytes per write operation) is performed regardless of the state of chip select */
    ADI_ADRV9001_ARM_SINGLE_SPI_WRITE_MODE_STREAMING_BYTES_4,   /*!< 4 bytes in a single SPI write in streaming mode */
                                                                /*!< Four write operation (but only 4 bytes per write operation) is performed in one chip select */
}adi_adrv9001_ArmSingleSpiWriteMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ArmVersion_t` structure is designed to encapsulate
 * version information for the ARM firmware used in the ADRV9001 device.
 * It includes fields for major, minor, maintenance, and release
 * candidate version numbers, as well as a field to specify the build
 * type of the firmware. This structure is essential for tracking and
 * managing different versions of the ARM firmware, ensuring
 * compatibility and proper functionality of the device.
 *
 * @param majorVer Represents the major version number of the ARM firmware.
 * @param minorVer Represents the minor version number of the ARM firmware.
 * @param maintVer Represents the maintenance version number of the ARM
 * firmware.
 * @param rcVer Represents the release candidate version number of the ARM
 * firmware.
 * @param armBuildType Specifies the build type of the ARM firmware, defined by
 * the adi_adrv9001_ArmBuildType_e enumeration.
 ******************************************************************************/
typedef struct adi_adrv9001_ArmVersion
{
    uint8_t majorVer;
    uint8_t minorVer;
    uint8_t maintVer;
    uint8_t rcVer;
    adi_adrv9001_ArmBuildType_e armBuildType;
} adi_adrv9001_ArmVersion_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_Checksum_t` structure is designed to hold checksum
 * values for the ADRV9001 ARM firmware. It contains two members:
 * `buildChecksum`, which is the checksum calculated during the build
 * process, and `runChecksum`, which is the checksum calculated during
 * runtime. This structure is used to verify the integrity and
 * consistency of the firmware by comparing these two checksum values.
 *
 * @param buildChecksum Stores the checksum calculated at build time.
 * @param runChecksum Stores the checksum calculated at runtime.
 ******************************************************************************/
typedef struct adi_adrv9001_Checksum
{
    uint32_t buildChecksum;
    uint32_t runChecksum;
} adi_adrv9001_Checksum_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_ChecksumTable_t` structure is designed to store
 * various checksums related to the ADRV9001 ARM firmware and profiles,
 * including firmware, stream, device profile, and PFIR profile
 * checksums. It also includes fields for capturing firmware error status
 * and error codes, providing a comprehensive overview of the integrity
 * and error state of the firmware and associated profiles.
 *
 * @param fwCheckSums Holds the firmware checksums.
 * @param streamsCheckSum Array holding checksums for each stream, with a size
 * defined by ADRV9001_MAX_NUM_STREAM.
 * @param deviceProfileCheckSum Checksum for the device profile.
 * @param pfirProfileCheckSum Checksum for the PFIR profile.
 * @param fwError Stores the firmware error status as a 32-bit unsigned integer.
 * @param fwErrorCode Stores the firmware error code as a 32-bit unsigned
 * integer.
 ******************************************************************************/
typedef struct adi_adrv9001_ChecksumTable
{
    adi_adrv9001_Checksum_t fwCheckSums;
    adi_adrv9001_Checksum_t streamsCheckSum[ADRV9001_MAX_NUM_STREAM];
    adi_adrv9001_Checksum_t deviceProfileCheckSum;
    adi_adrv9001_Checksum_t pfirProfileCheckSum;
    uint32_t fwError;
    uint32_t fwErrorCode;
} adi_adrv9001_ChecksumTable_t;

#endif /* _ADI_ADRV9001_ARM_TYPES_H_ */
