/**
 * \file
 * \brief Contains ADRV9001 parameters validation related private function prototypes
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADRV9001_VALIDATORS_H_
#define _ADRV9001_VALIDATORS_H_

#ifdef __cplusplus
extern "C"{
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* Header files related to libraries */
#include "adi_adrv9001_user.h"

/* System header files */

/*********************************************************************************************************/
/***************************************************************************//**
 * @brief This function checks whether the provided channel number is valid for
 * the given ADRV9001 device context. It should be used to ensure that
 * operations involving specific channels are performed on valid channels
 * only. The function must be called with a properly initialized device
 * context. It returns a status code indicating whether the channel is
 * valid or if corrective action is needed.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure containing
 * the current device settings. This must be a valid, non-null
 * pointer to ensure proper validation.
 * @param channel An enumerated value of type adi_common_ChannelNumber_e
 * representing the channel to be validated. Valid values are
 * ADI_CHANNEL_1 and ADI_CHANNEL_2. If the channel is outside
 * this range, the function will return an error code.
 * @return An integer status code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if the channel is invalid.
 ******************************************************************************/
int32_t adi_adrv9001_Channel_Validate(adi_adrv9001_Device_t *adrv9001,
                                      adi_common_ChannelNumber_e channel);
/***************************************************************************//**
 * @brief Use this function to ensure that the specified port is valid for
 * operations with the ADRV9001 device. It checks if the port is either
 * ADI_RX or ADI_TX, which are the only acceptable values. This function
 * should be called before performing operations that require a valid
 * port to prevent errors. If an invalid port is provided, an error is
 * reported, and the function returns an error code.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null, as it is used to report errors.
 * @param port The port to validate, which must be either ADI_RX or ADI_TX. If
 * the port is not one of these values, an error is reported.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or an
 * error code if the port is invalid.
 ******************************************************************************/
int32_t adi_adrv9001_Port_Validate(adi_adrv9001_Device_t *adrv9001,
                                   adi_common_Port_e port);

/***************************************************************************//**
 * @brief Use this function to ensure that the specified channels and ports are
 * in a valid state for operation with the ADRV9001 device. This function
 * checks that the ARM and stream processors are loaded before proceeding
 * with channel operations. It validates each port and channel
 * combination provided in the arrays. This function should be called
 * before enabling or disabling channels to ensure that the device is in
 * a proper state. The function requires that the length of the ports and
 * channels arrays is within the valid range, and each entry in these
 * arrays must correspond to a valid port and channel combination.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null and should point to a properly
 * initialized device context.
 * @param ports Array of ports to validate. Each entry must correspond to a
 * valid port. The array must have at least one element and no more
 * than four elements.
 * @param channels Array of channels to validate. Each entry must correspond to
 * a valid channel. The array must have at least one element and
 * no more than four elements.
 * @param length The number of port/channel combinations to validate. Must be
 * between 1 and 4, inclusive, and should match the number of
 * elements in the ports and channels arrays.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover if validation fails.
 ******************************************************************************/
int32_t adi_adrv9001_Channel_State_GenericValidate(adi_adrv9001_Device_t *adrv9001,
                                                   adi_common_Port_e ports[],
                                                   adi_common_ChannelNumber_e channels[],
                                                   uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* _ADRV9001_VALIDATORS_H_ */