/**
 * \file
 * \brief Functions for working with ADRV9001 profiles
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2020 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_PROFILEUTIL_H_
#define _ADI_ADRV9001_PROFILEUTIL_H_

#include "adi_adrv9001_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CLIENT_IGNORE

/***************************************************************************//**
 * @brief Use this function to parse a JSON-formatted device profile and load
 * its contents into an initialization structure. This function is
 * essential when setting up the ADRV9001 device with a specific
 * configuration defined in a JSON buffer. Ensure that the `init`
 * structure has been fully allocated before calling this function. The
 * function will report errors if the JSON is invalid or if memory
 * allocation fails during parsing.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before calling this
 * function.
 * @param init Pointer to an initialization structure where the parsed profile
 * contents will be stored. Must be fully allocated and is modified
 * by the function.
 * @param jsonBuffer Pointer to a buffer containing the JSON-formatted device
 * profile. Must not be null and should contain valid JSON
 * data.
 * @param length The length of the JSON buffer. Must accurately reflect the size
 * of the data in `jsonBuffer`.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_profileutil_Parse(adi_adrv9001_Device_t *adrv9001,
                                       adi_adrv9001_Init_t *init,
                                       char *jsonBuffer,
                                       uint32_t length);
#endif // !CLIENT_IGNORE

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_PROFILEUTIL_H_ */
