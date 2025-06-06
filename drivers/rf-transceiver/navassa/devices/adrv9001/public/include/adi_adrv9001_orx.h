/**
 * \file
 * \brief Contains ADRV9001 receive related function prototypes for adi_adrv9001_orx.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_ORX_H_
#define _ADI_ADRV9001_ORX_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_orx_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief Use this function to set the manual gain index for a specified ORx
 * channel on the ADRV9001 device. This function should be called after
 * the ARM is initialized. The gain index must be within the range of 2
 * to 10, inclusive. If the gain index is outside this range, the
 * function will return an error. This function directly accesses the
 * device registers to set the gain index.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param channel The ORx channel for which to set the gain. Must be a valid
 * channel number as defined by adi_common_ChannelNumber_e.
 * @param gainIndex The gain table index to set the channel to. Must be between
 * 2 and 10, inclusive. If outside this range, an error is
 * returned.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_ORx_Gain_Set(adi_adrv9001_Device_t *adrv9001,
                                  adi_common_ChannelNumber_e channel,
                                  uint8_t gainIndex);

/***************************************************************************//**
 * @brief Use this function to retrieve the current gain index of a specified
 * ORx channel on the ADRV9001 device. It should be called after the
 * device has been initialized and is in a Receiver mode, as gain indices
 * are only tracked in this state. This function provides direct register
 * access to obtain the gain index, which is returned through the
 * gainIndex parameter. Ensure that the device pointer and channel are
 * valid before calling this function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must not be null and should point to a properly
 * initialized device.
 * @param channel The ORx Channel from which to read the gain. Must be a valid
 * channel number as defined by adi_common_ChannelNumber_e.
 * @param gainIndex Pointer to a uint8_t where the current gain table index will
 * be stored. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if an error occurs.
 ******************************************************************************/
int32_t adi_adrv9001_ORx_Gain_Get(adi_adrv9001_Device_t *adrv9001,
                                  adi_common_ChannelNumber_e channel,
                                  uint8_t *gainIndex);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_ORX_H_ */
