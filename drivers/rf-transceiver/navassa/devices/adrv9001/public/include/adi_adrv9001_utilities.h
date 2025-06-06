/**
 * \file
 * \brief Contains ADRV9001 utility functions to load ARM binaries
 *        load stream binaries, load Rx Gain Table, load Tx Atten Table.
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

/**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_UTILITIES_H_
#define _ADI_ADRV9001_UTILITIES_H_

#include "adi_adrv9001_types.h"
#include "adi_adrv9001_arm_types.h"
#include "adi_adrv9001_rx_types.h"
#include "adrv9001_arm_macros.h"
#include "adi_adrv9001_radio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to load and write the ARM binary image to the
 * ADRV9001 device in chunks of size
 * ADI_ADRV9001_ARM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES. It reads each
 * chunk from the specified ARM image file and writes it to the device
 * using the specified SPI write mode. This function should be called
 * when the ARM binary needs to be programmed into the device. Care
 * should be taken to ensure that the chunk size is optimized according
 * to the available stack space to prevent stack overflow. The function
 * returns a code indicating success or the required action to recover
 * from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must not be null.
 * @param armImagePath Absolute path of the ARM image to be programmed. Must be
 * a valid file path.
 * @param spiWriteMode Preferred SPI write mode. Must be within the range of
 * valid adi_adrv9001_ArmSingleSpiWriteMode_e values.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_ArmImage_Load(adi_adrv9001_Device_t *adrv9001, const char *armImagePath, adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function is used to load a stream binary image from a specified
 * file path and write it to the ADRV9001 device in chunks. It should be
 * called when you need to program the device with a new stream image.
 * The function reads the image in chunks of size defined by
 * ADI_ADRV9001_STREAM_BINARY_IMAGE_LOAD_CHUNK_SIZE_BYTES and writes each
 * chunk using the specified SPI write mode. Ensure that the device
 * pointer is valid and the SPI write mode is within the allowed range.
 * The function returns a code indicating success or the required
 * recovery action if an error occurs.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param streamImagePath Absolute path to the stream image file to be
 * programmed. Must be a valid file path.
 * @param spiWriteMode Preferred SPI write mode. Must be a valid value within
 * the range of adi_adrv9001_ArmSingleSpiWriteMode_e
 * enumeration.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_StreamImage_Load(adi_adrv9001_Device_t *adrv9001, const char *streamImagePath, adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function loads the Rx gain table from a specified CSV file and
 * writes it to the ADRV9001 device's SRAM. It is used to configure the
 * gain settings for a specified Rx or ORx port and channel. The function
 * must be called with a valid device context and after the device has
 * been properly initialized. The gain table file must be correctly
 * formatted, with gain indices in ascending order, to avoid errors. The
 * function handles errors related to file reading and gain index
 * validation, ensuring that the gain table is loaded correctly.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param port The desired Rx or ORx port. Must be a valid adi_common_Port_e
 * value.
 * @param rxGainTablePath Path to the Rx gain table in CSV format. Must not be
 * null. The file must be accessible and correctly
 * formatted.
 * @param channel The Rx channel from which to write the gain table. Must be a
 * valid adi_common_ChannelNumber_e value.
 * @param lnaConfig Pointer to the desired LNA configuration. Must not be null.
 * The caller retains ownership.
 * @param gainTableType Gain table type loaded during initialization. Must be a
 * valid adi_adrv9001_RxGainTableType_e value.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_RxGainTable_Load(adi_adrv9001_Device_t *adrv9001,
                                                adi_common_Port_e port,
                                                const char *rxGainTablePath,
                                                adi_common_ChannelNumber_e channel,
                                                adi_adrv9001_RxLnaConfig_t *lnaConfig,
                                                adi_adrv9001_RxGainTableType_e gainTableType);

/***************************************************************************//**
 * @brief This function loads the Tx attenuation table from a specified CSV file
 * and writes it to the ADRV9001 device's SRAM. It should be used when
 * configuring the device's transmission attenuation settings. The
 * function requires a valid device context and a path to the CSV file
 * containing the attenuation table. The CSV file must have entries
 * arranged in ascending order of attenuation indices. The function also
 * requires a channel mask to specify which Tx attenuation tables to
 * load. It returns a status code indicating success or the required
 * action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param txAttenTablePath Path to the Tx attenuation table in CSV format. Must
 * not be null. The file must contain entries with
 * indices in ascending order.
 * @param txChannelMask An OR'd combination of adi_common_ChannelNumber_e values
 * specifying which Tx attenuation tables to load. Must be
 * a valid channel mask.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_TxAttenTable_Load(adi_adrv9001_Device_t *adrv9001,
                                                 const char *txAttenTablePath,
                                                 uint32_t txChannelMask);

/***************************************************************************//**
 * @brief This function provides a blocking wait for a specified duration in
 * milliseconds, useful for timing operations in applications using the
 * ADRV9001 device. It should be called when a delay is needed, and the
 * device context must be valid and initialized. The function returns an
 * error code if the wait operation fails, which can be used to diagnose
 * issues with the timer functionality.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null and should point to a valid, initialized device context.
 * @param waitInterval_ms The duration to wait, specified in milliseconds. Must
 * be a non-negative integer.
 * @return Returns an integer error code indicating success or the type of error
 * encountered during the wait operation.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_WaitMs(adi_adrv9001_Device_t *adrv9001, uint32_t waitInterval_ms);
	
/***************************************************************************//**
 * @brief This function is used to perform system debugging on the ADRV9001
 * device before calibration. It should be called before the device is
 * initialized. The function loads and verifies ARM and stream processor
 * images, checks power supplies, and configures the SPI interface. It is
 * essential to ensure that the device is correctly connected and powered
 * before calling this function. The function returns a status code
 * indicating success or the required action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param init Pointer to the ADRV9001 initialization settings data structure.
 * Must not be null. The caller retains ownership.
 * @param armImagePath Absolute path of the ARM image to be programmed. Must not
 * be null. The caller retains ownership.
 * @param streamImagePath Absolute path of the stream image to be programmed.
 * Must not be null. The caller retains ownership.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_SystemDebugPreCalibrate(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_Init_t *init, const char *armImagePath, const char *streamImagePath);

/***************************************************************************//**
 * @brief This function is used to perform system debugging on the ADRV9001
 * device after all initialized channels have been calibrated. It checks
 * the lock status of the RF PLLs and reports any issues that may prevent
 * the device from operating correctly. This function should be called
 * after the calibration process to ensure that the PLLs are locked and
 * the system is ready for operation. It provides diagnostic messages to
 * help identify any problems with the PLLs.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null. The function will return an error if this pointer is
 * invalid.
 * @return Returns an integer code indicating success or failure. A successful
 * return indicates that the PLLs are locked and the system is ready. A
 * failure return indicates an issue with the PLL lock status.
 ******************************************************************************/
int32_t adi_adrv9001_Utilities_SystemDebugPostCalibrate(adi_adrv9001_Device_t *adrv9001);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_UTILITIES_H_ */