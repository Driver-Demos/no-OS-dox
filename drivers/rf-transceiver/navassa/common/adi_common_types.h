/**
* \file
* \brief Contains ADI common types.
*
* ADI common lib Version: $ADI_COMMON_LIB_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_COMMON_TYPES_H_
#define _ADI_COMMON_TYPES_H_

#include "adi_common_error_types.h"

#ifndef CLIENT_IGNORE
/***************************************************************************//**
 * @brief The `adi_common_Device_t` structure is designed to encapsulate common
 * device-related information for Analog Devices' hardware. It includes a
 * pointer to hardware layer settings specific to the device instance,
 * allowing for device-specific configurations and operations.
 * Additionally, it contains an error structure to manage and report
 * errors associated with the device operations, ensuring robust error
 * handling and debugging capabilities.
 *
 * @param devHalInfo ADI_HAL Hardware layer settings pointer specific to this
 * device instance.
 * @param error An instance of adi_common_ErrStruct_t to handle error
 * information.
 ******************************************************************************/
typedef struct adi_common_Device
{
    void                     *devHalInfo; /*!< ADI_HAL Hardware layer settings pointer specific to this device instance */
    adi_common_ErrStruct_t   error;
    /* function pointer for datapacking or device specific hal */
} adi_common_Device_t;
#endif // !CLIENT_IGNORE

/***************************************************************************//**
 * @brief The `adi_common_ApiVersion_t` structure is used to represent the
 * version information of an API, consisting of major, minor, and patch
 * version numbers. This structure allows for easy tracking and
 * management of different API versions, facilitating compatibility and
 * updates.
 *
 * @param major API Major Version number.
 * @param minor API Minor Version number.
 * @param patch API Patch Version number.
 ******************************************************************************/
typedef struct adi_common_ApiVersion
{
    uint32_t major; /*!< API Major Version number */
    uint32_t minor; /*!< API Minor Version number */
    uint32_t patch; /*!< API Patch Version number */
} adi_common_ApiVersion_t;

#endif  /* _ADI_COMMON_TYPES_H_ */

