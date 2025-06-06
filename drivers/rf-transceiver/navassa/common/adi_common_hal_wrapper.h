/**
 * \file
 * \brief Contains ADI Hardware Abstraction layer function prototypes and type definitions for adi_common_hal.c
 *
 * ADI common lib Version: $ADI_COMMON_LIB_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_COMMON_HAL_WRAPPER_H_
#define _ADI_COMMON_HAL_WRAPPER_H_

/* include standard types and definitions */
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "adi_common_log.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE

/***************************************************************************//**
 * @brief This function is used to pause the execution of the program for a
 * specified duration in microseconds. It is typically used in scenarios
 * where precise timing is required, such as in hardware communication or
 * delay-sensitive operations. The function must be called with a valid
 * device structure pointer, and it will report any errors encountered
 * during the wait operation. It is important to ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param commonDev A pointer to an adi_common_Device_t structure representing
 * the device context. Must not be null, and the device must be
 * properly initialized before use. If null, the function will
 * handle it as an error.
 * @param time_us The number of microseconds to pause execution. Must be a non-
 * negative integer.
 * @return Returns an integer status code indicating the result of the
 * operation. Possible return values include success or various error
 * codes related to parameter validation or hardware interface issues.
 ******************************************************************************/
int32_t adi_common_hal_wrapper_Wait_us(adi_common_Device_t *commonDev, uint32_t time_us);

#endif

#ifdef __cplusplus
}
#endif

#endif
