/**
 * \file
 * \brief Contains ADRV9001 ARM related function prototypes for adi_adrv9001_arm.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2018 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_ARM_H_
#define _ADI_ADRV9001_ARM_H_

#include "adi_adrv9001_arm_types.h"
#include "adi_common_error_types.h"
#include "adi_adrv9001_types.h"
#include "adi_adrv9001_error.h"
#include "adrv9001_arm.h"

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to enable the ARM processor on the ADRV9001
 * device. It should be called after the device has been initialized. The
 * function enables the AHB SPI bridge, sets the ARM Run bit to 1, and
 * issues a software interrupt to wake up the ARM processor. This is
 * typically done as part of the device startup sequence to prepare the
 * ARM processor for operation.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The function expects the
 * device to be properly initialized before calling.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Enable(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to disable the ARM processor on the ADRV9001
 * device by setting the ARM Run bit to 0 and disabling the AHB SPI
 * bridge. It should be called after the device has been initialized. The
 * function also involves polling for the ARM to be READY_FOR_MCS, which
 * is a state indicating readiness for multichip synchronization. This
 * function is typically used when the ARM processor needs to be stopped
 * for configuration changes or power management purposes.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The caller retains
 * ownership of the memory.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Disable(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function enables the AHB SPI bridge on the ADRV9001 device, which
 * is necessary for certain operations involving direct register access.
 * It should be called after the device has been properly initialized.
 * The function modifies the device's internal register to activate the
 * bridge, allowing for subsequent SPI communications. It is important to
 * ensure that the device context is valid and initialized before calling
 * this function to avoid undefined behavior.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null and should point to a valid,
 * initialized device context.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_AhbSpiBridge_Enable(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to disable the AHB SPI bridge on the ADRV9001
 * device, which is typically done as part of shutting down or
 * reconfiguring the device. It must be called after the device has been
 * properly initialized. The function modifies the device's internal
 * register to achieve this, and it is important to ensure that the
 * device context is valid and initialized before calling this function.
 * The function returns a status code indicating success or the type of
 * action required to recover from an error.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. This parameter must not be null and should
 * point to a valid, initialized device context.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_AhbSpiBridge_Disable(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to verify the status of the ARM processor
 * firmware after it has been enabled. It waits for the ARM to exit the
 * boot-up state and checks if it reaches a ready state within a
 * specified timeout period. This function should be called after the
 * device has been initialized and before the multichip synchronization
 * (MCS) is completed. It is important to ensure that the ARM is not in a
 * SPI write-only mode, as this function requires SPI read access. The
 * function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param timeout_us Timeout in microseconds to stop waiting for the ARM to boot
 * up. Must be a positive integer.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_StartStatus_Check(adi_adrv9001_Device_t *adrv9001, uint32_t timeout_us);

/***************************************************************************//**
 * @brief This function waits for the ARM processor to reach a specified
 * firmware status within a given timeout period. It is used after the
 * ARM processor has been started and before multichip synchronization
 * (MCS) is completed. The function reads the firmware status via SPI, so
 * it cannot be used in SPI write-only modes. If the desired firmware
 * status is not reached within the timeout, an error is returned.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be initialized before calling this
 * function.
 * @param timeout_us The maximum time in microseconds to wait for the ARM
 * processor to reach the specified firmware status. Must be a
 * positive integer.
 * @param fwCheckStatus The firmware status value to wait for. The function will
 * return when the ARM processor's firmware status matches
 * this value.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover if the firmware status is not
 * reached within the timeout.
 ******************************************************************************/
int32_t adi_adrv9001_arm_FwStatus_Check(adi_adrv9001_Device_t *adrv9001, uint32_t timeout_us, uint32_t fwCheckStatus);

/***************************************************************************//**
 * @brief This function is used to write the configuration settings to the
 * ADRV9001 ARM processor. It should be called when you need to update
 * the ARM configuration with new initialization settings. Ensure that
 * the device has been properly initialized before calling this function.
 * The function will return a status code indicating success or the
 * necessary action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null.
 * @param init Pointer to the ADRV9001 initialization settings data structure.
 * Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Profile_Write(adi_adrv9001_Device_t *adrv9001, const adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief This function is used to write the PFIR coefficients into the ARM
 * memory of the ADRV9001 device. It should be called when the device is
 * initialized and the PFIR profiles need to be configured. The function
 * requires valid pointers to both the device context and the
 * initialization settings. It returns a status code indicating success
 * or the necessary action to recover from an error.
 *
 * @param adrv9001 Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param init Pointer to the ADRV9001 initialization settings data structure.
 * Must not be null. The caller retains ownership.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_PfirProfiles_Write(adi_adrv9001_Device_t *adrv9001, const adi_adrv9001_Init_t *init);

/***************************************************************************//**
 * @brief This function is used to load a binary array into the ARM program
 * memory of the ADRV9001 device. It should be called after the device
 * has been initialized, the PLL lock status has been verified, and the
 * stream binary has been loaded. The function requires the byte count to
 * be a multiple of 4, and the byte offset must also be a multiple of 4.
 * If the byte offset is zero, the byte count must be larger than 8. The
 * function supports specific SPI write modes and handles the ARM memory
 * write operation with auto-incrementing addresses. It returns a status
 * code indicating success or the required action to recover from an
 * error.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param byteOffset Offset in ARM memory to start writing. Must be a multiple
 * of 4.
 * @param binary Byte array containing the ARM file data bytes. Must not be
 * null.
 * @param byteCount Number of bytes in the binary array. Must be a multiple of 4
 * and larger than 8 if byteOffset is zero.
 * @param spiWriteMode Preferred SPI write mode. Must be within the valid range
 * of defined SPI write modes.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Image_Write(adi_adrv9001_Device_t *adrv9001,
                                     uint32_t byteOffset,
                                     const uint8_t binary[],
                                     uint32_t byteCount,
                                     adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function issues a software interrupt to wake up the ARM processor
 * in the ADRV9001 device. It should be called after the device has been
 * initialized and is typically used when the ARM processor needs to be
 * activated from a low-power or idle state. The function modifies the
 * SW_Interrupt_4 register to trigger the wake-up and then clears the
 * interrupt to complete the process. It returns a status code indicating
 * success or the necessary action to recover from an error.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null. The function expects the
 * device to be properly initialized before calling.
 * @return A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required
 * action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_WakeupInterrupt_Set(adi_adrv9001_Device_t *adrv9001);


/****************************************************************************
 * Helper functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from the ADRV9001 ARM
 * program or data memory starting at a given address. It is intended for
 * use after the device has been properly initialized. The function
 * supports reading from both program memory and data memory within
 * specified address ranges. The caller must ensure that the returnData
 * buffer is large enough to hold the requested byteCount. The function
 * can optionally auto-increment the memory address for consecutive
 * reads.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param address The 32-bit ARM address to read from. Must be within valid
 * memory ranges: Program Memory (0x01000000 - 0x0101C000) or
 * Data Memory (0x20000000 - 0x20014000).
 * @param returnData Byte array to store the data read from ARM memory. Must not
 * be null and should be large enough to hold byteCount bytes.
 * @param byteCount Number of bytes to read from the ARM memory. Must be a
 * positive integer.
 * @param autoIncrement Boolean flag (0 or 1) to enable or disable auto-
 * increment of the ARM register address after each byte
 * read.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Memory_Read(adi_adrv9001_Device_t *adrv9001,
                                     uint32_t address,
                                     uint8_t returnData[],
                                     uint32_t byteCount,
                                     uint8_t autoIncrement);
/***************************************************************************//**
 * @brief This function is used to read 32-bit data from the specified address
 * in the ADRV9001 ARM program or data memory. It is intended for use
 * after the device has been properly initialized. The function supports
 * reading from both program and data memory within specified address
 * ranges. The caller must ensure that the returnData array is large
 * enough to hold the number of bytes specified by byteCount. The
 * function can optionally auto-increment the memory address for
 * consecutive reads. It returns a status code indicating success or the
 * required action to recover from an error.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param address The 32-bit ARM address to read from. Must be within valid
 * memory address ranges.
 * @param returnData Array to store the data read from ARM memory. Must not be
 * null and should be large enough to hold byteCount bytes.
 * @param byteCount Number of bytes to read. Must be a multiple of 4, as the
 * function reads 32-bit data.
 * @param autoIncrement Boolean flag to enable or disable auto-increment of the
 * ARM register address. Non-zero value enables auto-
 * increment.
 * @return Returns an int32_t status code indicating success or the required
 * action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Memory_Read32(adi_adrv9001_Device_t *adrv9001,
		uint32_t address,
		uint32_t returnData[],
		uint32_t byteCount,
		uint8_t autoIncrement);
/***************************************************************************//**
 * @brief This function writes a specified number of bytes from a data array to
 * a given address in the ADRV9001 ARM memory. It should be called after
 * the device has been initialized and is used to load data into the
 * ARM's program or data memory. The function supports different SPI
 * write modes, but certain modes require the byte count to be a multiple
 * of 4. Ensure that the address and byte count are within valid ranges
 * for the ARM memory.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param address The 32-bit ARM address to write to. Must be within valid
 * memory ranges: Program Memory (0x01000000 - 0x0101C000) or
 * Data Memory (0x20000000 - 0x20014000).
 * @param data Array of bytes containing the data to be written to ARM memory.
 * Must not be null.
 * @param byteCount Number of bytes to write from the data array. Must be a
 * positive integer and, for certain SPI write modes, a
 * multiple of 4.
 * @param spiWriteMode Preferred SPI write mode, which determines how data is
 * written to the ARM memory. Some modes have specific
 * requirements on byteCount.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Memory_Write(adi_adrv9001_Device_t *adrv9001,
                                      uint32_t address,
                                      const uint8_t data[],
                                      uint32_t byteCount,
                                      adi_adrv9001_ArmSingleSpiWriteMode_e spiWriteMode);

/***************************************************************************//**
 * @brief This function is used to write frequency hopping table data into the
 * ARM memory of the ADRV9001 device. It is typically called when
 * configuring frequency hopping settings, allowing the user to specify
 * the hop signal, table ID, and the data to be written. The function
 * requires valid ARM memory addresses and data arrays for the hop table
 * entries and buffer. It should be called after the device has been
 * initialized and is ready to accept configuration data.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param hopSignal Specifies the hop signal to configure the appropriate table
 * ID. Must be a valid adi_adrv9001_FhHopSignal_e value.
 * @param tableId Specifies the frequency hopping table ID, either
 * FH_HOP_TABLE_A or FH_HOP_TABLE_B.
 * @param hopTableAddress The 32-bit ARM address where the hop table entries
 * will be written. Must be a valid ARM memory address.
 * @param numHopTableEntries Array of bytes containing the number of hop table
 * entries to be written. Must not be null.
 * @param numHopTableEntriesByteCount Number of bytes in the numHopTableEntries
 * array. Must be greater than zero.
 * @param hopTableBufferAddress The 32-bit ARM address where the hop table
 * buffer data will be written. Must be a valid ARM
 * memory address.
 * @param hopTableBufferData Array of bytes containing the hop table buffer data
 * to be written. Must not be null.
 * @param hopTableBufferDataByteCount Number of bytes in the hopTableBufferData
 * array. Must be greater than zero.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Memory_WriteFH(adi_adrv9001_Device_t *adrv9001, 
		                                    adi_adrv9001_FhHopSignal_e hopSignal,
		                                    adi_adrv9001_FhHopTable_e tableId, 
		                                    uint32_t hopTableAddress, 
		                                    const uint8_t numHopTableEntries[], 
		                                    uint32_t numHopTableEntriesByteCount, 
		                                    uint32_t hopTableBufferAddress, 
		                                    const uint8_t hopTableBufferData[], 
		                                    uint32_t hopTableBufferDataByteCount);

/***************************************************************************//**
 * @brief This function is used to write configuration data and command
 * instructions to the ADRV9001 ARM processor. It should be called when
 * there is a need to update the ARM configuration settings or issue
 * specific commands to the ARM processor. The function requires valid
 * data arrays for both the configuration data and the mailbox command,
 * along with their respective sizes. It is important to ensure that the
 * device has been properly initialized before calling this function. The
 * function returns a status code indicating success or the necessary
 * action to recover from an error.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should point to a valid, initialized device
 * context.
 * @param armData Array of bytes containing the configuration data to be written
 * to the ARM memory. Must not be null and should have a size
 * specified by armDataSize.
 * @param armDataSize Number of bytes in the armData array. Must be greater than
 * zero.
 * @param mailboxCmd Array of bytes containing the command data to be sent to
 * the ARM command interface. Must not be null and should have
 * a size specified by mailboxCmdSize.
 * @param mailboxCmdSize Number of bytes in the mailboxCmd array. Must be
 * greater than zero.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover from an
 * error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Config_Write(adi_adrv9001_Device_t *adrv9001, 
                                      const uint8_t armData[],
                                      uint32_t armDataSize,
                                      const uint8_t mailboxCmd[],
                                      uint32_t mailboxCmdSize);

/***************************************************************************//**
 * @brief This function is used to read a specified number of bytes from the
 * ADRV9001 ARM memory, identified by an object ID and sub-object ID, and
 * writes the data into a provided buffer. It is typically used to access
 * configuration or status information stored in the ARM memory. The
 * function must be called with a valid device context and after the
 * device has been properly initialized. The caller must ensure that the
 * returnData buffer is large enough to hold the requested byteCount. The
 * function handles different read modes based on the byteCount
 * alignment.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be initialized before calling this
 * function.
 * @param objectId ARM ID of a particular structure or setting in ARM memory.
 * Valid range is not specified, but it should correspond to a
 * known object in the ARM memory.
 * @param subObjectId Additional ARM ID for further specifying the structure or
 * setting in ARM memory. Valid range is not specified, but
 * it should correspond to a known sub-object in the ARM
 * memory.
 * @param channelMask Mask of Tx/Rx channels to specify which channels the
 * operation applies to. Each bit represents a different
 * channel.
 * @param byteOffset Offset in bytes from the start of the objectId's memory
 * location in ARM memory. Must be within the valid range of
 * the memory space.
 * @param returnData Array to store the data read from ARM memory. Must not be
 * null and should have a size of at least byteCount.
 * @param byteCount Number of bytes to read from ARM memory. Valid size is
 * between 1 and 255 bytes. The function handles different read
 * modes based on whether byteCount is a multiple of 4.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The returnData array is populated with the data read from
 * ARM memory.
 ******************************************************************************/
int32_t adi_adrv9001_arm_MailBox_Get(adi_adrv9001_Device_t *adrv9001, 
                                     uint8_t objectId, 
                                     uint8_t subObjectId, 
                                     uint8_t channelMask, 
                                     uint16_t byteOffset, 
                                     uint8_t returnData[], 
                                     uint32_t byteCount);

/***************************************************************************//**
 * @brief This function is used to read configuration data from the ADRV9001 ARM
 * memory. It is typically called when there is a need to retrieve
 * specific configuration settings from the ARM processor. The function
 * requires a valid device context and specific identifiers for the
 * configuration object and channel. It reads a specified number of bytes
 * from a given offset within the configuration object. The function must
 * be called with a properly initialized device context, and the
 * returnData array must be large enough to hold the requested byteCount.
 * The function returns a status code indicating success or the type of
 * action required to recover from an error.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null and should be properly initialized before calling
 * this function.
 * @param objectId ARM ID of the specific configuration object to read. Must be
 * a valid identifier for the desired configuration.
 * @param channelMask Mask indicating which Tx/Rx channels are involved. Each
 * bit corresponds to a specific channel.
 * @param byteOffset Offset in bytes from the start of the configuration object
 * in ARM memory. Determines where the read operation begins.
 * @param returnData Array to store the data read from ARM memory. Must be pre-
 * allocated and large enough to hold byteCount bytes.
 * @param byteCount Number of bytes to read from ARM memory. Must be between 1
 * and 255 inclusive.
 * @return Returns an int32_t status code indicating success or the required
 * action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Config_Read(adi_adrv9001_Device_t *adrv9001, 
                                     uint8_t objectId,
                                     uint8_t channelMask,
                                     uint16_t byteOffset,
                                     uint8_t returnData[],
                                     uint32_t byteCount);

/***************************************************************************//**
 * @brief Use this function to obtain the current system error status from the
 * ADRV9001 ARM processor. It reads the error code and the associated
 * object ID, which can help diagnose issues within the ARM system. This
 * function should be called when an error condition is suspected or
 * needs to be confirmed. Ensure that the device is properly initialized
 * before calling this function. The function requires valid pointers for
 * both the object ID and error code parameters, and it will return an
 * error if these pointers are null.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership.
 * @param objectId Pointer to a uint8_t where the function will store the object
 * ID associated with the error. Must not be null.
 * @param errorCode Pointer to a uint8_t where the function will store the error
 * code. Must not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The objectId and errorCode are populated with the relevant
 * error information.
 ******************************************************************************/
int32_t adi_adrv9001_arm_SystemError_Get(adi_adrv9001_Device_t *adrv9001, uint8_t *objectId, uint8_t *errorCode);

/**
 * \brief Reads the ADRV9001 ARM 64-bit command status register and returns an error and status word
 *
 * A 64-bit status register consisting of a pending bit and three-bit error type is read one byte at
 * a time for the first 16 even-numbered opcodes. The function parses the pending bits and error bits into
 * two (2) separate 16-bit words. statusWord contains the status pending bits. errorWord contains
 * a single error bit if the error type > 0 for any three-bit code.
 * Each word is weighted according to the first 16 even-numbered opcodes where,
 * 0x0001 = opcode '0', 0x0002 = opcode '2', 0x0004 = opcode '4', 0x0008 = opcode '6' and so on.
 *
 * \note Message type: \ref timing_direct "Direct register access"
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \param[in]  adrv9001		Context variable - Pointer to the ADRV9001 device settings data structure
 * \param[out] errorWord 16-bit error word comprised of weighted bits according to first 16 even-numbered opcodes
 * The weighted bit is set if any three-bit error type > 0, where '0' = OK
 * \param[out] statusWord 16-bit pending bits word comprised of weighted bits according to first 16 even-numbered opcodes
 * The weighted bit is set if an opcode is pending, where '0' = no pending opcode
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
 /* TODO: Need to review number of error and status bits in SPI regs */
/***************************************************************************//**
 * @brief This function reads the 64-bit command status register of the ADRV9001
 * ARM processor and parses it into two separate 16-bit words: an error
 * word and a status word. The error word indicates if any error types
 * are present, while the status word indicates pending operations. This
 * function should be called when you need to check the status of ARM
 * commands and detect any errors. It requires a valid device context and
 * expects non-null pointers for the output parameters. The function
 * initializes the error and status words to zero before processing.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param errorWord Pointer to a 16-bit variable where the error word will be
 * stored. Must not be null.
 * @param statusWord Pointer to a 16-bit variable where the status word will be
 * stored. Must not be null.
 * @return Returns an integer code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_CmdStatus_Get(adi_adrv9001_Device_t *adrv9001, uint16_t *errorWord, uint16_t *statusWord);

/**
 * \brief Isolated byte read of the ADRV9001 ARM 64-bit command status register based on the opcode
 *
 * A single byte read is performed on the 64-bit command status register according to
 * the opcode of interest. The pending bit and the error type are extracted from the status
 * register and returned as a single byte in the lower nibble.
 *
 * \note Message type: \ref timing_direct "Direct register access"
 *
 * \pre This function is private and is not called directly by the user.
 *
 * \param[in]  adrv9001			Context variable - Pointer to the ADRV9001 device settings data structure
 * \param[in]  opCode			Opcode of interest where only the first 16 even-numbered integers are valid
 * \param[out] cmdStatByte		Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
 /* TODO: Need to review number of bits SPI regs per opcode */
/***************************************************************************//**
 * @brief This function is used to obtain the status byte associated with a
 * specific ARM command opcode in the ADRV9001 device. It should be
 * called when you need to check the status of a command that has been
 * issued to the ARM processor. The function requires a valid opcode,
 * which must be an even number not greater than 30, except for the
 * special case of the stream trigger opcode. The status byte is returned
 * through a pointer parameter, which must not be null. This function
 * should be called after the device has been properly initialized.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null.
 * @param opCode The opcode of interest, which must be an even number not
 * greater than 30, except for the stream trigger opcode.
 * @param cmdStatByte A pointer to a uint8_t where the status byte will be
 * stored. Must not be null.
 * @return Returns an int32_t indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_CmdStatusOpcode_Get(adi_adrv9001_Device_t *adrv9001, uint8_t opCode, uint8_t *cmdStatByte);

#ifndef CLIENT_IGNORE
/**
 * \brief ADRV9001 ARM command status wait function polls command status register until opcode of interest completes
 *
 * \note Message type: \ref timing_direct "Direct register access"
 *
 * \pre ARM firmware load and initialization must take place first before attempting to use this function
 *
 * \param[in]  adrv9001			Context variable - Pointer to the ADRV9001 device settings data structure
 * \param[in]  opCode			Opcode of interest where only the first 16 even-numbered integers are valid
 * \param[out] cmdStatusByte    Comprised of cmdStatByte[3:1] = error type, cmdStatByte[0] = pending flag for opCode of interest
 * \param[in]  timeout_us       Command time-out period in microseconds
 * \param[in]  waitInterval_us  Wait interval time to thread sleep between each check of the ARM command status to prevent SPI read thrashing
 *
 * \returns A code indicating success (ADI_COMMON_ACT_NO_ACTION) or the required action to recover
 */
 /* TODO: Need to review number of bits SPI regs per opcode */
/***************************************************************************//**
 * @brief Use this function to poll the status of an ARM command until it
 * completes or a specified timeout period elapses. It is essential to
 * ensure that the ARM firmware is loaded and initialized before calling
 * this function. The function checks the command status at regular
 * intervals and can handle errors if the command does not complete
 * successfully within the timeout period. This function is useful for
 * synchronizing with ARM operations that require confirmation of
 * completion.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param opCode The opcode of interest, where only the first 16 even-numbered
 * integers are valid. Invalid opcodes will result in an error.
 * @param cmdStatusByte Pointer to a byte where the command status will be
 * stored. Must not be null.
 * @param timeout_us The maximum time in microseconds to wait for the command to
 * complete.
 * @param waitInterval_us The interval in microseconds between status checks. If
 * greater than timeout_us, it will be clamped to
 * timeout_us.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The cmdStatusByte is updated with
 * the command status.
 ******************************************************************************/
int32_t adi_adrv9001_arm_CmdStatus_Wait(adi_adrv9001_Device_t *adrv9001,
                                        uint8_t opCode,
                                        uint8_t *cmdStatusByte,
                                        uint32_t timeout_us,
                                        uint32_t waitInterval_us);
#endif

/***************************************************************************//**
 * @brief This function is used to send a command to the ADRV9001 ARM processor
 * interface, which is part of the device's control system. It should be
 * called when the ARM is initialized and ready to accept commands. The
 * function requires an opcode and may include additional extended data.
 * It ensures that the ARM command interface is not busy before sending
 * the command, and it handles any errors related to invalid opcodes or
 * extended data sizes. The function is designed to work with specific
 * opcodes and data sizes, and it will report errors if these constraints
 * are not met.
 *
 * @param device Pointer to the ADRV9001 device data structure containing
 * settings. Must not be null.
 * @param opCode The opcode of interest, where only the first 16 even-numbered
 * integers are valid. Must be a valid opcode as per the device's
 * specifications.
 * @param extendedData A byte array containing extended data to write to the ARM
 * command interface. Can be null if byteCount is 0. If not
 * null, must contain between 0 and 4 bytes.
 * @param byteCount Number of bytes in the extendedData array. Valid size is 0-4
 * bytes. If greater than 4, the function will report an error.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Cmd_Write(adi_adrv9001_Device_t *adrv9001, 
                                   uint8_t opCode,
                                   const uint8_t extendedData[],
                                   uint32_t byteCount);

/***************************************************************************//**
 * @brief This function is used to program the device profile on the ADRV9001
 * ARM processor. It should be called when the channel state is any of
 * STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. The function sends a
 * command to the ARM processor and waits for the command to complete. It
 * is essential to ensure that the device pointer is valid before calling
 * this function.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure containing settings. Must not be null. The function
 * will handle invalid pointers by returning an error code.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Profile_Program(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to program the ADRV9001 ARM system with a
 * specific channel configuration as indicated by the channel mask. It
 * should be called after the device has been initialized and when the
 * channel state is in STANDBY. The function sends a command to the ARM
 * processor to set the system configuration and waits for the command to
 * complete. It handles any errors that occur during the command
 * execution and updates the device's channel profile enable mask based
 * on the initialized channels and port switch configuration.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null. The caller retains
 * ownership.
 * @param channelMask A uint8_t value representing the mask of Tx/Rx channels to
 * be configured. Each bit corresponds to a specific channel
 * (e.g., bit 0 for RX1, bit 1 for RX2, etc.). Invalid values
 * may result in undefined behavior.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_System_Program(adi_adrv9001_Device_t *adrv9001, uint8_t channelMask);

 /****************************************************************************
 * Debug functions
 ****************************************************************************
 */

/***************************************************************************//**
 * @brief This function is used to obtain the version information of the ARM
 * binary currently loaded into the ADRV9001 device. It should be called
 * only after the ARM binary has been successfully loaded into the
 * device. The function populates the provided `armVersion` structure
 * with the major, minor, maintenance, and release candidate version
 * numbers, as well as the build type of the ARM binary. It is important
 * to ensure that the ARM binary is loaded before invoking this function,
 * as failure to do so will result in an error.
 *
 * @param device A pointer to the `adi_adrv9001_Device_t` structure representing
 * the ADRV9001 device. Must not be null and must point to a
 * valid, initialized device structure.
 * @param armVersion A pointer to an `adi_adrv9001_ArmVersion_t` structure where
 * the ARM version information will be stored. Must not be
 * null.
 * @return Returns an `int32_t` code indicating success
 * (`ADI_COMMON_ACT_NO_ACTION`) or the required action to recover from
 * an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Version(adi_adrv9001_Device_t *adrv9001, adi_adrv9001_ArmVersion_t *armVersion);

/***************************************************************************//**
 * @brief Use this function to obtain the checksum table from the ARM processor
 * of the ADRV9001 device and verify its validity. This function should
 * be called after the ARM binary file has been loaded to ensure the
 * integrity of the firmware. It checks the calculated checksum against
 * the build-time checksum and updates the provided checksum structure
 * with these values. The function also indicates whether the checksum is
 * valid. Ensure that the device is properly initialized before calling
 * this function.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null.
 * @param checksum Pointer to a structure where the checksum data will be
 * stored. Must not be null.
 * @param checksumValid Pointer to a uint8_t where the validity of the checksum
 * will be indicated. Must not be null.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover. The checksum structure is
 * populated with the checksum values, and checksumValid is set to 1 if
 * the checksum is valid.
 ******************************************************************************/
int32_t adi_adrv9001_arm_ChecksumTable_Get(adi_adrv9001_Device_t *adrv9001, 
                                           adi_adrv9001_ChecksumTable_t *checksum,
                                           uint8_t *checksumValid);

/***************************************************************************//**
 * @brief This function determines whether the ARM mailbox of the ADRV9001
 * device is currently busy or ready to accept new commands. It should be
 * called after the device has been initialized and the ARM is running.
 * The function requires valid pointers for both the device context and
 * the mailboxBusy output parameter. If the device pointer or mailboxBusy
 * pointer is null, the function will not proceed. This function is
 * useful for ensuring that commands are not sent to the ARM mailbox when
 * it is busy, which could lead to errors or undefined behavior.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null. The caller retains
 * ownership.
 * @param mailboxBusy A pointer to a boolean variable where the function will
 * store the result. It will be set to true if the mailbox is
 * busy, or false if it is ready. Must not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_MailboxBusy_Get(adi_adrv9001_Device_t *adrv9001, bool *mailboxBusy);

/***************************************************************************//**
 * @brief This function is used to configure the next dynamic profile for the
 * ADRV9001 device by sending the profile data to the ARM processor and
 * executing the necessary command. It should be called when a new
 * dynamic profile needs to be set, and it is important that the device
 * is properly initialized before calling this function. The function
 * handles the command execution and waits for its completion, ensuring
 * that the profile is correctly set.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. Must not be null, and the device should be
 * initialized before use.
 * @param dynamicProfile A pointer to the adi_adrv9000_DynamicProfile_t
 * structure containing the dynamic profile data to be
 * set. Must not be null, and should contain valid profile
 * data.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_NextDynamicProfile_Set(adi_adrv9001_Device_t *adrv9001, 
                                                const adi_adrv9000_DynamicProfile_t *dynamicProfile);

/***************************************************************************//**
 * @brief This function is used to send a new set of PFIR coefficients to the
 * ADRV9001 device for the channels specified by the channel mask. It
 * prepares the coefficients for all selected channels, but the new
 * coefficients will not take effect until a profile switch is executed.
 * This function should be called when the device is in any of the
 * following states: STANDBY, CALIBRATED, PRIMED, or RF_ENABLED. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device A pointer to the ADRV9001 device data structure. This must not
 * be null and should point to a valid, initialized device
 * context.
 * @param channelMask A bitmask indicating which channels the PFIR coefficients
 * should be applied to. Each bit corresponds to a specific
 * channel, and multiple channels can be selected
 * simultaneously.
 * @param pfirCoeff A pointer to a structure containing the PFIR coefficients to
 * be set. This must not be null and should point to a valid
 * PFIR configuration.
 * @return Returns an integer code indicating success or the type of action
 * required to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_arm_NextPfir_Set(adi_adrv9001_Device_t *adrv9001, 
                                      uint8_t channelMask, 
                                      const adi_adrv9001_PfirWbNbBuffer_t *pfirCoeff);

/***************************************************************************//**
 * @brief This function is used to trigger a profile switch on the ADRV9001
 * device by issuing a software interrupt. It should be called when a
 * profile change is required, and the device is in a state that allows
 * for such a change. The function must be called with a valid device
 * context that has been properly initialized. It returns a status code
 * indicating the success of the operation or any required recovery
 * actions.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device. This parameter must not be null and should
 * point to a properly initialized device context.
 * @return Returns an int32_t status code indicating success
 * (ADI_COMMON_ACT_NO_ACTION) or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Profile_Switch(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief This function is used to start the ARM processor on the ADRV9001
 * device. It should be called after the device has been properly
 * initialized. The function enables the ARM processor by setting the
 * necessary control registers. It is important to ensure that the device
 * context is correctly set up before calling this function to avoid any
 * undefined behavior.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device settings
 * data structure. Must not be null and should be properly
 * initialized before calling this function.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Start(adi_adrv9001_Device_t *adrv9001);

/***************************************************************************//**
 * @brief Use this function to stop the ARM processor on the ADRV9001 device. It
 * is typically called when the ARM processor needs to be halted, such as
 * during shutdown or reconfiguration processes. This function should be
 * called only after the device has been properly initialized. It
 * performs a direct register access to control the ARM processor state.
 *
 * @param device Pointer to the ADRV9001 device settings data structure. Must
 * not be null. The caller retains ownership and is responsible
 * for ensuring the device is initialized before calling this
 * function.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_arm_Stop(adi_adrv9001_Device_t *adrv9001);


#define ADRV9001_ARM_OPCODE_MASK      0xFFFF0000
#define ADRV9001_ARM_OPCODE_SHIFT     16
#define ADRV9001_ARM_OBJ_ID_MASK      0x0000FF00
#define ADRV9001_ARM_OBJ_ID_SHIFT     8
#define ADRV9001_ARM_ERROR_MASK       0x000000FF
#define ADRV9001_ARM_ERROR_SHIFT      0

#define ADRV9001_ARMCMD_ERRCODE(armOpCode, armObjId, armErrorFlag) ((armOpCode << ADRV9001_ARM_OPCODE_SHIFT) | (armObjId << ADRV9001_ARM_OBJ_ID_SHIFT) | armErrorFlag)

#define ADRV9001_OPCODE_VALID(a) (((a) != 0) && (((a) % 2) || ((a) > 30)))

#define ADRV9001_ARM_CMD_STATUS_WAIT_EXPECT(devicePtr, opcode, objid, time1, time2) \
{\
    int32_t _recoveryAction = ADI_COMMON_ACT_NO_ACTION; \
    uint8_t _cmdStatusByte = 0; \
    _recoveryAction = adi_adrv9001_arm_CmdStatus_Wait(devicePtr, \
                                                      (uint8_t)opcode, \
                                                      &_cmdStatusByte, \
                                                      (uint32_t)time1, \
                                                      (uint32_t)time2); \
    /* If cmdStatusByte is non-zero then flag an ARM error */ \
    if((_cmdStatusByte >> 1) > 0) \
    { \
        ADI_EXPECT(adrv9001_ArmCmdErrorHandler, devicePtr, ADRV9001_ARMCMD_ERRCODE(opcode, objid, _cmdStatusByte)); \
    } \
    ADI_ERROR_REPORT(&devicePtr->common, ADI_COMMON_ERRSRC_API, devicePtr->common.error.errCode, _recoveryAction, NULL, devicePtr->common.error.errormessage); \
    ADI_ERROR_RETURN(devicePtr->common.error.newAction); \
}

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_ARM_H_ */
