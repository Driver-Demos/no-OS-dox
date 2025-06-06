/**
* \file
* \brief ADRV9001 Multi-Chip Synchronization (MCS) functions
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2020 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADI_ADRV9001_MCS_H_
#define _ADI_ADRV9001_MCS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_adrv9001_mcs_types.h"
#include "adi_common_macros.h"
#include "adi_adrv9001_user.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_radio_types.h"

/***************************************************************************//**
 * @brief This function is used to verify the synchronization status of the
 * ADRV9001 transceiver's analog and digital subsystems. It should be
 * called after issuing one or more MCS pulses to check the
 * synchronization status. The function is primarily intended for
 * debugging purposes and can be used outside the MCS_READY state to
 * inspect register statuses. It requires the device to be in the
 * CALIBRATED state before use. The function populates the provided
 * status structure with synchronization information, which varies
 * depending on the current state of the MCS procedure.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param mcsStatus Pointer to a structure where the MCS status will be stored.
 * Must not be null. The structure will be populated with
 * synchronization status information based on the current MCS
 * procedure state.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Mcs_Status_Get(adi_adrv9001_Device_t *adrv9001,
                                    adi_adrv9001_McsStatus_t *mcsStatus);         

/***************************************************************************//**
 * @brief Use this function to verify the current state of the ADRV9001 software
 * during the Multi-Chip Synchronization (MCS) process. It should be
 * called when the device system state is ADI_ADRV9001_ARM_SYSTEM_MCS.
 * The function provides the software status based on the MCS pulses
 * issued, which helps in debugging and testing the synchronization
 * process. Ensure the device is in the appropriate system state before
 * calling this function to get valid results.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership.
 * @param mcsSwStatus Pointer to a variable where the current status of the
 * ADRV9001 software will be stored. Must not be null. The
 * function writes the status to this location.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The mcsSwStatus parameter is
 * updated with the current software status.
 ******************************************************************************/
int32_t adi_adrv9001_Mcs_SwStatus_Get(adi_adrv9001_Device_t *adrv9001,
                                      adi_adrv9001_McsSwStatus_e *mcsSwStatus);                                                            
                                  
/***************************************************************************//**
 * @brief Use this function to obtain the delay, measured in samples, between
 * the reception of a Multi-Chip Synchronization (MCS) signal and the
 * first strobe received on the specified Tx channel interface. This
 * function is useful for verifying synchronization timing in the
 * ADRV9001 device. Ensure that the device is properly initialized and in
 * a state ready for MCS operations before calling this function. The
 * function requires a valid channel number and a non-null pointer for
 * the latency output.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param channel Specifies the Tx channel number. Valid values are defined by
 * adi_common_ChannelNumber_e.
 * @param latency Pointer to a uint16_t where the function will store the output
 * delay in samples. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Mcs_TxMcsToStrobeSampleLatency_Get (adi_adrv9001_Device_t *adrv9001, 
                                                         adi_common_ChannelNumber_e channel, 
                                                         uint16_t *latency);

/***************************************************************************//**
 * @brief This function configures the Multi-Chip Synchronization (MCS) delay
 * for a specified channel and port on the ADRV9001 device. It should be
 * used when you need to adjust the synchronization timing by setting the
 * sample delay and the SSI FIFO read pointer delay. The function must be
 * called with a valid device context and after the device has been
 * properly initialized. It is important to ensure that the port and
 * channel parameters are valid and that the mcsDelay parameter is within
 * the acceptable range for the specified port. The function returns a
 * status code indicating success or the required action to recover from
 * an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum.
 * Must be a valid port value (e.g., ADI_RX or ADI_TX).
 * @param channel The channel number, specified as an adi_common_ChannelNumber_e
 * enum. Must be a valid channel value (e.g., ADI_CHANNEL_1 or
 * ADI_CHANNEL_2).
 * @param mcsDelay Pointer to an adi_adrv9001_McsDelay_t structure containing
 * the delay settings. For RX ports, readDelay must be between 1
 * and 15; for TX ports, it must be between 0 and 15. Must not
 * be null.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_Mcs_ChannelMcsDelay_Set (adi_adrv9001_Device_t *adrv9001, 
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel, 
                                              adi_adrv9001_McsDelay_t *mcsDelay);

/***************************************************************************//**
 * @brief This function is used to obtain the current Multi-Chip Synchronization
 * (MCS) delay settings for a specified channel on the ADRV9001 device.
 * It should be called when the device is in a state where MCS settings
 * are relevant, typically after the device has been initialized and
 * configured. The function requires valid port and channel parameters
 * and will populate the provided mcsDelay structure with the current
 * delay settings. It is important to ensure that the adrv9001 device
 * context is properly initialized before calling this function.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null. The caller retains ownership.
 * @param port The port of interest, specified as an adi_common_Port_e enum.
 * Must be a valid port for the device.
 * @param channel The channel number, specified as an adi_common_ChannelNumber_e
 * enum. Must be a valid channel for the specified port.
 * @param mcsDelay Pointer to an adi_adrv9001_McsDelay_t structure where the
 * current MCS delay settings will be stored. Must not be null.
 * The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Mcs_ChannelMcsDelay_Get (adi_adrv9001_Device_t *adrv9001, 
                                              adi_common_Port_e port,
                                              adi_common_ChannelNumber_e channel, 
                                              adi_adrv9001_McsDelay_t *mcsDelay);
#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_MCS_H_ */
