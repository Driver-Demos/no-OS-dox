/**
 * \file
 * \brief Contains ADRV9001 related function prototypes for adi_adrv9001_stream.c
 *
 * Copyright 2015 - 2021 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_STREAM_H_
#define _ADI_ADRV9001_STREAM_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_stream_types.h"
#include "adi_adrv9001_arm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to load a binary array into the ADRV9001 stream
 * processor's data memory, which is essential for powering up and down
 * various signal chains. It should be called after the device has been
 * initialized with adi_adrv9001_Initialize and before the ARM is loaded.
 * The function requires a byte array obtained from the binary stream
 * processor file provided by Analog Devices. It is important to ensure
 * that the byteOffset and byteCount are multiples of 4, and that
 * byteCount is at least 68 if byteOffset is 0. The function returns a
 * code indicating success or the required action to recover from an
 * error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must not be null.
 * @param byteOffset Offset (starting from 0) of where to place the binary
 * array. Must be a multiple of 4.
 * @param binary Byte array containing all valid ARM file data bytes. Must not
 * be null.
 * @param byteCount The number of bytes in the binary array file. Must be a
 * multiple of 4 and at least 68 if byteOffset is 0.
 * @param spiWriteMode Preferred SPI write mode. Must be a valid
 * adi_adrv9001_ArmSingleSpiWriteMode_e value.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Stream_Image_Write(adi_adrv9001_Device_t *adrv9001, uint32_t byteOffset, uint8_t binary[], uint32_t byteCount, adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief Use this function to retrieve the version information of the stream
 * processor binary currently loaded in the ADRV9001 device memory. This
 * function should be called only after the stream processor binary has
 * been successfully loaded into the device. It reads the version as a
 * combination of major, minor, maintenance, and build numbers, which are
 * then populated into the provided streamVersion structure. Ensure that
 * the device is in a state where the stream processor binary is loaded
 * before calling this function to avoid errors.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should point to a valid, initialized device context.
 * @param streamVersion Pointer to an adi_adrv9001_StreamVersion_t structure
 * where the version information will be stored. Must not
 * be null, and the caller is responsible for allocating
 * this structure before calling the function.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_Stream_Version(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_StreamVersion_t *streamVersion);

/***************************************************************************//**
 * @brief This function is used to enable or disable the stream to GPIO debug
 * feature, which routes the operation status of the stream processor to
 * dedicated GPIO pins. It should be called when all channels are in
 * either the STANDBY or CALIBRATED state. This function is useful for
 * debugging purposes, allowing the user to monitor the status of the
 * stream processor through GPIO pins. The function returns a code
 * indicating success or the required action to recover.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must not be null, as it provides the necessary
 * context for the operation.
 * @param streamToGpioDebug Flag to enable or disable the stream to GPIO debug
 * feature. A value of 'true' enables the feature,
 * while 'false' disables it.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Stream_Gpio_Debug_Set(adi_adrv9001_Device_t *adrv9001, bool streamToGpioDebug);

/***************************************************************************//**
 * @brief This function is used to determine whether the stream to GPIO debug
 * feature is currently enabled or disabled on the ADRV9001 device. It
 * should be called when the device channels are in any of the following
 * states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. This function is
 * useful for verifying the current configuration of the device's debug
 * settings. It does not modify the state of the device, only retrieves
 * the current setting.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. This must
 * not be null and should point to a valid, initialized device
 * context.
 * @param streamToGpioDebug A pointer to a boolean variable where the function
 * will store the current status of the stream to GPIO
 * debug feature. This must not be null.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The boolean pointed to by
 * streamToGpioDebug is set to true if the feature is enabled, or false
 * if it is disabled.
 ******************************************************************************/
int32_t adi_adrv9001_Stream_Gpio_Debug_Get(adi_adrv9001_Device_t *adrv9001, bool *streamToGpioDebug);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_STREAM_H_ */
