/**
 * \file
 * \brief Contains ADRV9001 BBDC-related function prototypes for adi_adrv9001_bbdc.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_BBDC_H_
#define _ADI_ADRV9001_BBDC_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_bbdc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to control the baseband DC rejection algorithm
 * for a specified Rx or ORx channel on the ADRV9001 device. It should be
 * called when the channel is in one of the following states: STANDBY,
 * CALIBRATED, PRIMED, or RF_ENABLED. The function sends a command to the
 * device to either enable or disable the DC rejection based on the
 * provided status. It is important to ensure that the device context and
 * channel parameters are correctly set before calling this function to
 * avoid unexpected behavior.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port Specifies the port of the channel, either ADI_RX or ADI_ORX. Must
 * be a valid port enumeration value.
 * @param channel Specifies the channel number within the given port. Must be a
 * valid channel enumeration value.
 * @param bbdcRejectionStatus Specifies the desired status for the baseband DC
 * rejection, either enabled or disabled. Must be a
 * valid enumeration value of
 * adi_adrv9001_BbdcRejectionStatus_e.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_bbdc_RejectionEnable_Set(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_BbdcRejectionStatus_e bbdcRejectionStatus);

/***************************************************************************//**
 * @brief Use this function to obtain the current status of the baseband DC
 * rejection algorithm for a given Rx or ORx channel on the ADRV9001
 * device. This function should be called when the channel is in one of
 * the following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. It
 * is important to ensure that the `bbdcRejectionStatus` pointer is valid
 * and that the device context is properly initialized before calling
 * this function. The function communicates with the device via a mailbox
 * command to retrieve the status.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before use.
 * @param port Specifies the port of the channel, either ADI_RX or ADI_ORX. Must
 * be a valid port enumeration value.
 * @param channel Specifies the channel number within the given port. Must be a
 * valid channel enumeration value.
 * @param bbdcRejectionStatus Pointer to a variable where the function will
 * store the retrieved baseband DC rejection status.
 * Must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if an error occurs.
 ******************************************************************************/
int32_t adi_adrv9001_bbdc_RejectionEnable_Get(adi_adrv9001_Device_t *adrv9001,
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel,
                                              adi_adrv9001_BbdcRejectionStatus_e *bbdcRejectionStatus);

/***************************************************************************//**
 * @brief This function configures the loop or feedback gain for the Baseband DC
 * (BBDC) rejection algorithm on a specified channel of the ADRV9001
 * device. It should be called when the channel is in either the STANDBY
 * or CALIBRATED state. The function uses a fractional number to
 * represent the feedback gain, which is expected to be in U1.31 format.
 * Proper validation of parameters is performed, and the function
 * communicates with the device via a mailbox command. It returns a
 * status code indicating success or the necessary action to recover from
 * an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null, and the device should be properly initialized
 * before calling this function.
 * @param channel The channel of interest, which can be either an Rx or ORx
 * channel. It must be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param loopGain A fractional number in U1.31 format representing the feedback
 * gain for the BBDC. The default value is 1/2048. The function
 * expects a valid 32-bit unsigned integer.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9010_bbdc_LoopGain_Set(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_ChannelNumber_e channel,
                                       uint32_t loopGain);

/***************************************************************************//**
 * @brief Use this function to obtain the current loop or feedback gain setting
 * for the Baseband DC (BBDC) rejection algorithm on a specified channel
 * of the ADRV9001 device. This function should be called when the
 * channel is in any of the following states: STANDBY, CALIBRATED,
 * PRIMED, or RF_ENABLED. It is useful for verifying the current gain
 * setting or for diagnostic purposes. Ensure that the device context and
 * channel parameters are valid before calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * Must not be null. The caller retains ownership.
 * @param channel The channel of interest, applicable to both Rx and ORx. Must
 * be a valid channel number as defined by
 * adi_common_ChannelNumber_e.
 * @param loopGain A pointer to a uint32_t where the function will store the
 * retrieved loop gain value. Must not be null. The caller
 * provides the memory for this output.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The loop gain is written to the
 * location pointed to by loopGain.
 ******************************************************************************/
int32_t adi_adrv9010_bbdc_LoopGain_Get(adi_adrv9001_Device_t *adrv9001,
                                       adi_common_ChannelNumber_e channel,
                                       uint32_t *loopGain);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_BBDC_H_ */
