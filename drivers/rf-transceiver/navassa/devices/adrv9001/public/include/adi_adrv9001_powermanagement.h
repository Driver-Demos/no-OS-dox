/**
 * \file
 * \brief Low Dropout Regulator (LDO) configuration
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_POWERMANAGEMENT_H_
#define _ADRV9001_POWERMANAGEMENT_H_
    
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_adrv9001_powermanagement_types.h"
#include "adi_adrv9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function sets up the power management configuration for the
 * ADRV9001 device by adjusting the Low Dropout Regulators (LDOs)
 * according to the specified settings. It must be called when all
 * channels are in the STANDBY state to ensure proper configuration. The
 * function modifies the device's power management settings based on the
 * provided parameters and returns a status code indicating success or
 * the necessary action to recover from an error.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * This must not be null and should be properly initialized
 * before calling the function. The caller retains ownership.
 * @param powerManagementSettings A pointer to the desired power management
 * settings structure. This must not be null and
 * should contain valid LDO configuration data.
 * The caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_powermanagement_Configure(adi_adrv9001_Device_t *adrv9001, 
                                               adi_adrv9001_PowerManagementSettings_t *powerManagementSettings);

#ifdef __cplusplus
}
#endif

#endif