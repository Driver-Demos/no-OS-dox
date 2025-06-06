/**
 * \file
 * \brief Type definitions for ADRV9001 device system configuration
 * \copyright Analog Devices Inc. 2021. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_DEVICESYSCONFIG_TYPES_H_
#define _ADI_ADRV9001_DEVICESYSCONFIG_TYPES_H_

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_adrv9001_rxSettings_types.h"

#define ADI_ADRV9001_MAX_NUM_PLL 5
#define ADI_ADRV9001_NUM_RF_PLL  2

/***************************************************************************//**
 * @brief The `adi_adrv9001_pllModulus_t` structure is designed to hold
 * configuration settings for the PLL modulus in the ADRV9001 device. It
 * contains two arrays: `modulus`, which stores the PLL modulus values
 * for a maximum number of PLLs defined by `ADI_ADRV9001_MAX_NUM_PLL`,
 * and `dmModulus`, which stores the RF PLL modulus values specifically
 * for DM mode, with its size defined by `ADI_ADRV9001_NUM_RF_PLL`. This
 * structure is essential for configuring the PLL settings in the device
 * system configuration.
 *
 * @param modulus An array of PLL modulus values with a size defined by
 * ADI_ADRV9001_MAX_NUM_PLL.
 * @param dmModulus An array of RF PLL modulus values in DM mode with a size
 * defined by ADI_ADRV9001_NUM_RF_PLL.
 ******************************************************************************/
typedef struct adi_adrv9001_pllModulus
{
    uint32_t modulus[ADI_ADRV9001_MAX_NUM_PLL];  /*!< PLL modulus */
    uint32_t dmModulus[ADI_ADRV9001_NUM_RF_PLL]; /*!< RF PLL modulus in DM mode */
} adi_adrv9001_pllModulus_t;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DuplexMode_e` is an enumeration that defines the
 * duplex modes available for the ADRV9001 device, specifically Time
 * Division Duplex (TDD) and Frequency Division Duplex (FDD). This
 * enumeration is used to configure the duplex mode of the device, which
 * determines how the device handles transmission and reception of
 * signals, either by time-sharing the same frequency (TDD) or using
 * separate frequencies for transmission and reception (FDD).
 *
 * @param ADI_ADRV9001_TDD_MODE Represents the Time Division Duplex mode with a
 * value of 0.
 * @param ADI_ADRV9001_FDD_MODE Represents the Frequency Division Duplex mode
 * with a value of 1.
 ******************************************************************************/
typedef enum adi_adrv9001_DuplexMode
{
    ADI_ADRV9001_TDD_MODE = 0,
    ADI_ADRV9001_FDD_MODE = 1
} adi_adrv9001_DuplexMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_McsMode_e` is an enumeration that defines the modes
 * for Multi Chip Synchronization (MCS) in the ADRV9001 device. It
 * provides three options: disabling MCS, enabling MCS, and enabling MCS
 * with RFPLL phase synchronization. This enumeration is used to
 * configure the synchronization behavior of multiple chips in a system,
 * which is crucial for maintaining coherence and timing alignment across
 * devices.
 *
 * @param ADI_ADRV9001_MCSMODE_DISABLED Multi Chip Synchronization disabled.
 * @param ADI_ADRV9001_MCSMODE_ENABLED Multi Chip Synchronization enabled.
 * @param ADI_ADRV9001_MCSMODE_ENABLED_WITH_RFPLL_PHASE Multi Chip
 * Synchronization enabled
 * with RFPLL phase.
 ******************************************************************************/
typedef enum adi_adrv9001_McsMode
{
    ADI_ADRV9001_MCSMODE_DISABLED = 0,              /*!< Multi Chip Synchronization disabled */
    ADI_ADRV9001_MCSMODE_ENABLED,                   /*!< Multi Chip Synchronization enabled */
    ADI_ADRV9001_MCSMODE_ENABLED_WITH_RFPLL_PHASE   /*!< Multi Chip Synchronization enabled with RFPLL phase */
} adi_adrv9001_McsMode_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_NumDynamicProfiles_e` is an enumeration that defines
 * the possible configurations for the number of dynamic profiles in the
 * ADRV9001 device. It allows for selecting between disabling dynamic
 * profile switching or enabling it with a specific number of profiles
 * ranging from 2 to 6. This enumeration is used to configure the
 * device's dynamic profile capabilities, which can be crucial for
 * applications requiring flexible and adaptive signal processing
 * configurations.
 *
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_DISABLED Dynamic profile switching
 * is disabled.
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_2 Number of dynamic profiles is set
 * to 2.
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_3 Number of dynamic profiles is set
 * to 3.
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_4 Number of dynamic profiles is set
 * to 4.
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_5 Number of dynamic profiles is set
 * to 5.
 * @param ADI_ADRV9001_NUM_DYNAMIC_PROFILES_6 Number of dynamic profiles is set
 * to 6.
 ******************************************************************************/
typedef enum adi_adrv9001_NumDynamicProfiles
{
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_DISABLED = 1, /*!< Dynamic profile switching disabled */
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_2,            /*!< Number of dynamic profiles = 2 */
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_3,            /*!< Number of dynamic profiles = 3 */
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_4,            /*!< Number of dynamic profiles = 4 */
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_5,            /*!< Number of dynamic profiles = 5 */
    ADI_ADRV9001_NUM_DYNAMIC_PROFILES_6             /*!< Number of dynamic profiles = 6 */
} adi_adrv9001_NumDynamicProfiles_e;

/***************************************************************************//**
 * @brief The `adi_adrv9001_DeviceSysConfig_t` structure is used to configure
 * various system-level settings for the ADRV9001 device, including
 * duplex mode, frequency hopping, dynamic profiles, and synchronization
 * settings. It also includes parameters for PLL configuration and warm
 * boot functionality, allowing for efficient initialization and
 * operation of the device in different modes and conditions.
 *
 * @param duplexMode Specifies the duplex mode of the device, either TDD or FDD.
 * @param fhModeOn Indicates whether frequency hopping mode is enabled.
 * @param numDynamicProfiles Specifies the number of dynamic profiles available
 * for configuration.
 * @param mcsMode Defines the Multi Chip Synchronization mode.
 * @param mcsInterfaceType Specifies the interface type used for Multi Chip
 * Synchronization.
 * @param adcTypeMonitor Indicates the type of ADC used in Monitor Mode.
 * @param pllLockTime_us Specifies the required lock time in microseconds for
 * PLLs.
 * @param pllPhaseSyncWait_us Defines the worst-case phase synchronization wait
 * time in frequency hopping.
 * @param pllModulus Holds the PLL modulus settings.
 * @param warmBootEnable Indicates whether WarmBoot is enabled, allowing loading
 * of initCal coefficients instead of running initCals.
 ******************************************************************************/
typedef struct adi_adrv9001_DeviceSysConfig
{
    adi_adrv9001_DuplexMode_e duplexMode;
    uint8_t fhModeOn;
    adi_adrv9001_NumDynamicProfiles_e numDynamicProfiles; /*!< Number of dynamic Profiles */
    adi_adrv9001_McsMode_e mcsMode;             /*!< Multi Chip Synchronization mode */
    adi_adrv9001_SsiType_e mcsInterfaceType;    /*!< Multi Chip Synchronization interface type */
    adi_adrv9001_AdcType_e adcTypeMonitor;      /*!< ADC type used in Monitor Mode */
    uint16_t pllLockTime_us;                    /*!< Required lock time in microseconds for PLLs, based on ref_clk and loop bandwidth */
    uint16_t pllPhaseSyncWait_us;               /*!< Worst case phase sync wait time in FH */
    adi_adrv9001_pllModulus_t pllModulus;       /*!< PLL modulus */
    bool warmBootEnable;                        /*!< Enable WarmBoot - Load initCal cefficients instead of running initCals */
} adi_adrv9001_DeviceSysConfig_t;

#endif /* _ADI_ADRV9001_DEVICESYSCONFIG_TYPES_H_ */
