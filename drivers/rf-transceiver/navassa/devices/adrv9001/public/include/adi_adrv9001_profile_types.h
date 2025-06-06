
/**
 * \file
 * \brief Type definitions for the ADRV9001 profile
 * \copyright Analog Devices Inc. 2021. All rights reserved.
 * Released under the ADRV9001 API license, for more information see "LICENSE.txt" in the SDK
 */

#ifndef _ADI_ADRV9001_PROFILE_TYPES_H_
#define _ADI_ADRV9001_PROFILE_TYPES_H_

#ifndef __KERNEL__
#include <stdint.h>
#endif

#include "adi_adrv9001_clockSettings_types.h"
#include "adi_adrv9001_rxSettings_types.h"
#include "adi_adrv9001_txSettings_types.h"
#include "adi_adrv9001_deviceSysConfig_types.h"
#include "adi_adrv9001_pfirBuffer_types.h"
#include "adi_adrv9001_dynamicProfile_types.h"

/***************************************************************************//**
 * @brief The `adi_adrv9001_Init_t` structure is designed to encapsulate the
 * initialization settings for an ADRV9001 device instance. It includes
 * various configuration settings such as clock settings, receive and
 * transmit settings, system configuration, and data interface settings.
 * Each member of the structure corresponds to a specific aspect of the
 * device's configuration, allowing for comprehensive initialization of
 * the device's operational parameters.
 *
 * @param clocks Holds settings for CLKPLL and reference clock.
 * @param rx Rx settings data structure.
 * @param tx Tx settings data structure.
 * @param sysConfig Device system config struct.
 * @param pfirBuffer Holds the Data Interface CSSI/LSSI data link settings.
 ******************************************************************************/
typedef struct adi_adrv9001_Init
{
    adi_adrv9001_ClockSettings_t clocks;      /*!< Holds settings for CLKPLL and reference clock */
    adi_adrv9001_RxSettings_t rx;             /*!< Rx settings data structure */
    adi_adrv9001_TxSettings_t tx;             /*!< Tx settings data structure */
    adi_adrv9001_DeviceSysConfig_t sysConfig; /*!< Device system config struct */
    adi_adrv9001_PfirBuffer_t pfirBuffer;     /*!< Holds the Data Interface CSSI/LSSI data link settings */
} adi_adrv9001_Init_t;

#endif /* _ADI_ADRV9001_PROFILE_TYPES_H_ */
