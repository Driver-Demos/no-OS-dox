/**
* \file
* \brief Contains prototype of ARM private features related function implemented in
*        adrv9001_arm.c
*
* ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
*/

/**
* Copyright 2015 - 2018 Analog Devices Inc.
* Released under the ADRV9001 API license, for more information
* see the "LICENSE.txt" file in this zip file.
*/

#ifndef _ADRV9001_ARM_H_
#define _ADRV9001_ARM_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

/* ADI specific header files */
#include "adi_adrv9001.h"
#include "adi_adrv9001_fh_types.h"
#include "adi_adrv9001_arm_types.h"

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from the ADRV9001 ARM
 * program or data memory into a provided buffer. It is intended for
 * internal use and should not be called directly by users. The function
 * requires a valid memory address and a pre-allocated buffer to store
 * the read data. It supports optional auto-increment of the memory
 * address for sequential reads. The function returns a status code
 * indicating success or the type of recovery action required in case of
 * an error.
 *
 * @param device Pointer to the ADRV9001 device structure containing settings.
 * Must not be null.
 * @param address The 32-bit ARM address to read from. Must be a valid memory
 * address.
 * @param returnData Array to store the data read from the ARM memory. Must be
 * pre-allocated and large enough to hold 'byteCount' bytes.
 * @param byteCount Number of bytes to read from the ARM memory. Must be a
 * positive integer.
 * @param autoIncrement Boolean flag indicating whether to auto-increment the
 * ARM register address after each read. Non-zero value
 * enables auto-increment.
 * @return Returns an int32_t status code indicating success or the required
 * recovery action.
 ******************************************************************************/
int32_t adrv9001_DmaMemRead(adi_adrv9001_Device_t *device, uint32_t address, uint8_t returnData[], uint32_t byteCount, uint8_t autoIncrement);

/***************************************************************************//**
 * @brief This function writes a specified number of bytes from a data array to
 * a given address in the ADRV9001 ARM program or data memory. It
 * requires a valid device structure and memory address. The function
 * supports different SPI write modes, which must be specified. It is
 * crucial to ensure that the memory addresses are valid before calling
 * this function. The function is private and not intended to be called
 * directly by users.
 *
 * @param device A pointer to the ADRV9001 device structure containing the
 * device settings. Must not be null.
 * @param address The 32-bit ARM address where the data will be written. Must be
 * a valid memory address.
 * @param data A byte array containing the data to be written to the ARM memory.
 * The caller retains ownership of the data.
 * @param byteCount The number of bytes to write from the data array. Must be a
 * positive integer.
 * @param spiWriteMode The preferred SPI write mode, specified as an enumeration
 * value of type adi_adrv9001_ArmSingleSpiWriteMode_e.
 * @return Returns an integer indicating the success of the operation or the
 * required recovery action. Possible return values include
 * ADI_COMMON_ACT_NO_ACTION for success, or other codes indicating
 * specific recovery actions.
 ******************************************************************************/
int32_t adrv9001_DmaMemWrite(adi_adrv9001_Device_t *device, uint32_t address, const uint8_t data[], uint32_t byteCount, adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function is used to write frequency hopping table data to the
 * ADRV9001 ARM memory. It is intended for internal use and should not be
 * called directly by users. The function requires valid memory addresses
 * and data arrays to be provided. It handles the configuration of hop
 * signals and table IDs, and writes the specified data to the designated
 * ARM memory addresses. The function also manages SPI interrupts to
 * ensure the correct loading of frequency hopping tables.
 *
 * @param device Pointer to the ADRV9001 device structure containing settings.
 * Must not be null.
 * @param hopSignal Specifies the hop signal to configure the appropriate table
 * ID. Must be a valid adi_adrv9001_FhHopSignal_e value.
 * @param tableId Specifies the frequency hopping table ID (either
 * FH_HOP_TABLE_A or FH_HOP_TABLE_B). Must be a valid
 * adi_adrv9001_FhHopTable_e value.
 * @param hopTableAddress The 32-bit ARM address where the hop table data will
 * be written. Must be a valid address.
 * @param numHopTableEntries Array of bytes containing the data to be written to
 * the ARM memory. Must not be null.
 * @param numHopTableEntriesByteCount Number of bytes in the numHopTableEntries
 * array. Must be greater than zero.
 * @param hopTableBufferAddress The 32-bit ARM address where the hop table
 * buffer data will be written. Must be a valid
 * address.
 * @param hopTableBufferData Array of bytes containing the buffer data to be
 * written to the ARM memory. Must not be null.
 * @param hopTableBufferDataByteCount Number of bytes in the hopTableBufferData
 * array. Must be greater than zero.
 * @return Returns an int32_t indicating success (ADI_COMMON_ACT_NO_ACTION) or
 * the required action to recover from an error.
 ******************************************************************************/
int32_t adrv9001_DmaMemWriteFH(adi_adrv9001_Device_t *device, 
		                            adi_adrv9001_FhHopSignal_e hopSignal,
		                            adi_adrv9001_FhHopTable_e tableId, 
		                            uint32_t hopTableAddress, 
		                            const uint8_t numHopTableEntries[], 
		                            uint32_t numHopTableEntriesByteCount, 
		                            uint32_t hopTableBufferAddress, 
		                            const uint8_t hopTableBufferData[], 
		                            uint32_t hopTableBufferDataByteCount);
/***************************************************************************//**
 * @brief This function writes a specified number of bytes from a data array to
 * a designated address in the memory of a selected Flex Stream Processor
 * (SP) within the ADRV9001 device. It is essential to ensure that the
 * memory addresses provided are valid before calling this function. The
 * function allows for different SPI write modes, which can affect how
 * data is written. It is intended for internal use and should not be
 * called directly by users.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure containing the
 * device settings. Must not be null.
 * @param address The 32-bit ARM address where the data will be written. Must be
 * a valid memory address.
 * @param data A pointer to a byte array containing the data to be written. The
 * array must have at least byteCount elements. Must not be null.
 * @param byteCount The number of bytes to write from the data array. Must be a
 * positive integer.
 * @param flexSpNumber The Flex Stream Processor number (0-3) to which the data
 * will be written. Must be a valid processor number.
 * @param spiWriteMode The preferred SPI write mode, specified as an
 * adi_adrv9001_ArmSingleSpiWriteMode_e enumeration value.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adrv9001_FlexStreamProcessorMemWrite(adi_adrv9001_Device_t *device,
                                             uint32_t address,
                                             const uint8_t data[],
                                             uint32_t byteCount,
                                             uint8_t flexSpNumber,
                                             adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from the memory of a
 * selected Flex Stream Processor (SP) within the ADRV9001 device. It is
 * intended for internal use and should not be called directly by users.
 * The function requires a valid device structure and memory address, and
 * it populates the provided array with the data read. The auto-increment
 * flag determines whether the memory address should automatically
 * increment after each read operation. The function returns a status
 * code indicating success or the necessary recovery action.
 *
 * @param device Pointer to the ADRV9001 device structure containing settings.
 * Must not be null.
 * @param address The 32-bit ARM address to read from. Must be a valid memory
 * address.
 * @param returnData Array to store the data read from the memory. Must be large
 * enough to hold 'byteCount' bytes.
 * @param byteCount Number of bytes to read from the memory. Must be a positive
 * integer.
 * @param autoIncrement Boolean flag (0 or 1) to enable or disable auto-
 * increment of the memory address after each read.
 * @param flexSpNumber Specifies which Flex Stream Processor (0-3) to read from.
 * @return Returns an integer status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required recovery action.
 ******************************************************************************/
int32_t adrv9001_FlexStreamProcessorMemRead(adi_adrv9001_Device_t *device,
                                            uint32_t address,
                                            uint8_t returnData[],
                                            uint32_t byteCount,
                                            uint8_t autoIncrement,
                                            uint8_t flexSpNumber);
/***************************************************************************//**
 * @brief This function processes errors reported from the ARM to determine the
 * appropriate recovery action. It is a private helper function that is
 * automatically called by the API and should not be called directly by
 * the user. The function analyzes the detected error, retrieves error
 * messages, and reports them. It may also clear errors and retrieve
 * specific error codes for further processing. The function returns a
 * value indicating the necessary recovery action, which could range from
 * no action to specific error handling procedures.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be null
 * and should point to a valid device instance.
 * @param detErr A 32-bit value representing the detected error, formatted as
 * (ARM opcode << 16 | ARM object ID << 8 | ARM cmdStatusByte).
 * @return Returns an int32_t value representing the recovery action required
 * after processing the detected ARM error. Possible values include
 * ADI_COMMON_ACT_NO_ACTION for successful completion or other codes
 * indicating specific recovery actions.
 ******************************************************************************/
int32_t adrv9001_ArmCmdErrorHandler(adi_adrv9001_Device_t *device, uint32_t detErr);

/***************************************************************************//**
 * @brief This function is used to write the configuration settings to the
 * ADRV9001 ARM, which is essential for initializing the device with the
 * desired operational parameters. It should be called with a valid
 * device context and initialization settings structure. The function
 * handles various channel configurations and ensures that the settings
 * are correctly written to the ARM memory. It is important to ensure
 * that the device and initialization structures are properly set up
 * before calling this function to avoid errors. The function returns a
 * status code indicating the success of the operation or any required
 * recovery actions.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function.
 * @param init Pointer to the ADRV9001 initialization settings data structure.
 * Must not be null and should contain valid configuration data for
 * the device.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adrv9001_ArmProfileWrite(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief This function is used to write the Programmable Finite Impulse
 * Response (PFIR) coefficients into the ADRV9001 device's ARM PFIR
 * buffer. It should be called with a valid device context and
 * initialization settings. The function checks for valid input
 * parameters and ensures that the profile size does not exceed the
 * maximum allowed size. It returns a recovery action code indicating the
 * success or failure of the operation. This function must be called with
 * a properly initialized device and initialization structure.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param init Pointer to the ADRV9001 initialization settings data structure
 * containing the PFIR buffer. Must not be null. The caller retains
 * ownership.
 * @return Returns an int32_t value indicating the recovery action:
 * ADI_COMMON_ACT_NO_ACTION for success, or other codes for specific
 * errors.
 ******************************************************************************/
int32_t adrv9001_PfirProfilesWrite(adi_adrv9001_Device_t *device, const adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief This function is used to write the dynamic profile data into the
 * ADRV9001's dynamic profile mailbox. It should be called when the
 * dynamic profile needs to be updated or initialized. The function does
 * not notify the ARM, it only pushes the data into ARM memory. It is
 * important to ensure that the dynamic profile structure is correctly
 * populated before calling this function. The function returns a code
 * indicating success or the required action to recover from an error.
 *
 * @param device Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param dynamicProfile Pointer to the ADRV9000 dynamic profile structure. Must
 * not be null and should be properly initialized with the
 * desired dynamic profile data.
 * @return Returns an int32_t value indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adrv9001_DynamicProfile_Write(adi_adrv9001_Device_t *device,
                                      const adi_adrv9000_DynamicProfile_t *dynamicProfile);

/***************************************************************************//**
 * @brief This function is used to write a set of PFIR coefficients into the ARM
 * memory buffer of the ADRV9001 device. It is intended for configuring
 * the PFIR filter settings on the device. The function should be called
 * when the PFIR coefficients need to be updated or initialized. It does
 * not notify the ARM, meaning the data is simply pushed into the ARM
 * memory without triggering any further actions. Ensure that the device
 * structure is properly initialized before calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure containing the
 * current device settings. Must not be null.
 * @param pfirCoeff Pointer to the PFIR coefficient buffer to be written. Must
 * not be null and should contain valid PFIR configuration
 * data.
 * @return Returns an integer code indicating the success of the operation or
 * the required recovery action. Possible return values include
 * ADI_COMMON_ACT_NO_ACTION for success, or other codes indicating
 * specific recovery actions.
 ******************************************************************************/
int32_t adrv9001_PfirWbNbBuffer_Write(adi_adrv9001_Device_t *device,
                                      const adi_adrv9001_PfirWbNbBuffer_t *pfirBufferAddr);

/***************************************************************************//**
 * @brief This function is used to store a 16-bit integer into a byte array,
 * starting at a specified offset. It is useful when you need to
 * serialize data into a byte array for communication or storage
 * purposes. The function updates the offset to point to the next
 * available position in the array after the data is written. Ensure that
 * the offset points to a valid position within the array to avoid buffer
 * overflows.
 *
 * @param offset A pointer to a uint32_t that specifies the starting position in
 * the cfgData array where the data will be written. The value is
 * updated to point to the next position after the data is
 * written. Must not be null.
 * @param cfgData An array of uint8_t where the 16-bit data will be stored. The
 * array must be large enough to accommodate the data at the
 * specified offset.
 * @param data A 16-bit unsigned integer to be stored in the cfgData array. The
 * function writes the lower byte first, followed by the higher
 * byte.
 * @return None
 ******************************************************************************/
void adrv9001_LoadTwoBytes(uint32_t *offset, uint8_t cfgData[], const uint16_t data);
/***************************************************************************//**
 * @brief This function is used to load a 32-bit integer into a byte array,
 * starting at a specified offset. It is useful when you need to
 * serialize a 32-bit integer into a byte array for configuration or
 * communication purposes. The function updates the offset to point to
 * the next available position in the array after the data is written.
 * Ensure that the offset points to a valid position within the array and
 * that the array has enough space to accommodate four bytes.
 *
 * @param offset A pointer to a uint32_t that specifies the starting position in
 * the cfgData array where the data will be loaded. The value is
 * updated to point to the next position after the data is
 * written. Must not be null.
 * @param cfgData An array of uint8_t where the 32-bit data will be loaded. The
 * array must have sufficient space starting from the given
 * offset to accommodate four bytes. Caller retains ownership.
 * @param data A 32-bit unsigned integer whose bytes will be loaded into the
 * cfgData array. No specific range is required.
 * @return None
 ******************************************************************************/
void adrv9001_LoadFourBytes(uint32_t *offset, uint8_t cfgData[], const uint32_t data);
/***************************************************************************//**
 * @brief This function is used to extract eight bytes from a 64-bit integer and
 * store them sequentially into a specified byte array starting at a
 * given offset. It is useful when you need to serialize a 64-bit integer
 * into a byte array for transmission or storage. The function updates
 * the offset to reflect the new position after the bytes have been
 * written. Ensure that the byte array has sufficient space to
 * accommodate the eight bytes starting from the specified offset.
 *
 * @param offset A pointer to a uint32_t that specifies the starting position in
 * the byte array where the data will be written. The value is
 * updated to reflect the new position after writing. Must not be
 * null.
 * @param cfgData An array of uint8_t where the bytes extracted from the 64-bit
 * integer will be stored. The array must have enough space to
 * accommodate eight bytes starting from the specified offset.
 * Caller retains ownership.
 * @param data A 64-bit unsigned integer from which the eight bytes will be
 * extracted and stored in the byte array.
 * @return None
 ******************************************************************************/
void adrv9001_LoadEightBytes(uint32_t *offset, uint8_t cfgData[], const uint64_t data);
    
/***************************************************************************//**
 * @brief This function is used to read two consecutive bytes from a given byte
 * array, starting at a specified offset, and combine them into a single
 * 16-bit value. It is useful when parsing data structures that store
 * 16-bit values in a byte array format. The function updates the offset
 * to point to the next unread byte, facilitating sequential parsing of
 * multiple values. Ensure that the offset points to a valid position
 * within the array to avoid undefined behavior.
 *
 * @param offset A pointer to a uint32_t that specifies the current position in
 * the cfgData array. It must not be null, and it is updated to
 * point to the next position after reading two bytes.
 * @param cfgData An array of uint8_t containing the data to be parsed. The
 * array must have at least two bytes available starting from the
 * position indicated by offset.
 * @param value A pointer to a uint16_t where the parsed 16-bit value will be
 * stored. It must not be null.
 * @return None
 ******************************************************************************/
void adrv9001_ParseTwoBytes(uint32_t *offset, uint8_t cfgData[], uint16_t *value);
/***************************************************************************//**
 * @brief This function is used to parse four consecutive bytes from a given
 * byte array, starting at a specified offset, and combine them into a
 * single 32-bit integer. It is useful when dealing with data stored in a
 * byte array format that needs to be interpreted as a 32-bit integer.
 * The function updates the offset to point to the next byte after the
 * parsed integer, allowing for sequential parsing of multiple integers
 * from the array.
 *
 * @param offset A pointer to a uint32_t that specifies the starting position in
 * the cfgData array from which to begin parsing. The value at
 * this address is updated to reflect the new position after
 * parsing. Must not be null.
 * @param cfgData An array of uint8_t containing the data to be parsed. The
 * array must have at least four bytes available starting from
 * the position indicated by offset. The caller retains ownership
 * of the array.
 * @param value A pointer to a uint32_t where the parsed 32-bit integer will be
 * stored. Must not be null.
 * @return None
 ******************************************************************************/
void adrv9001_ParseFourBytes(uint32_t *offset, uint8_t cfgData[], uint32_t *value);
/***************************************************************************//**
 * @brief This function is used to parse and extract an eight-byte unsigned
 * integer from a given byte array starting at a specified offset. It is
 * useful when dealing with serialized data where a 64-bit integer is
 * stored across multiple bytes. The function updates the offset to point
 * to the next byte after the extracted integer, allowing for sequential
 * parsing of multiple values. It is important to ensure that the byte
 * array contains at least eight bytes starting from the given offset to
 * avoid undefined behavior.
 *
 * @param offset A pointer to a uint32_t representing the starting position in
 * the byte array. It must not be null, and it is updated to point
 * to the next position after the parsed bytes.
 * @param cfgData An array of uint8_t containing the serialized data. The array
 * must have at least eight bytes available starting from the
 * position indicated by offset.
 * @param value A pointer to a uint64_t where the parsed value will be stored.
 * It must not be null.
 * @return None
 ******************************************************************************/
void adrv9001_ParseEightBytes(uint32_t *offset, uint8_t cfgData[], uint64_t *value);
    
#ifdef __cplusplus
}
#endif

#endif