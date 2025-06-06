/**
 * \file
 * \brief Contains prototypes and macro definitions for Private ADI HAL wrapper
 *        functions implemented in adi_adrv9001_hal.c
 *
 * ADRV9001 API Version: $ADI_ADRV9001_API_VERSION$
 */

 /**
 * Copyright 2015 - 2019 Analog Devices Inc.
 * Released under the ADRV9001 API license, for more information
 * see the "LICENSE.txt" file in this zip file.
 */

#ifndef _ADI_ADRV9001_SPI_H_
#define _ADI_ADRV9001_SPI_H_

#include "adi_adrv9001_spi_types.h"

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* TODO: Any user changeable #defines need to be moved to adi_adrv9001_user.h */
#define HAL_TIMEOUT_DEFAULT 100         /* 100ms */
#define HAL_TIMEOUT_NONE 0x0            /* Non-blocking */
#define HAL_TIMEOUT_INFINITE 0xFFFFFFFF /* Blocking */
#define HAL_TIMEOUT_MULT 2              /* HAL timeout worse-case factor */

#define MAXSPILOGMESSAGE 64

#define SPIARRAYSIZE 1024
#define SPIARRAYTRIPSIZE ((SPIARRAYSIZE / 3) * 3)

#define ADRV9001_HW_RMW_LO_ADDR     0x113        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_HI_ADDR     0x114        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_MASK        0x115        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_DATA        0x116        /* Hardware Read Modify Write address */
#define ADRV9001_HW_RMW_BYTES       0xC         /* Number of bytes required to use HW_RMW */
#define ADRV9001_SPI_BYTES          0x3          /* Number of bytes required to use non HW_RMW */
#define ADRV9001_SPI_WRITE_POLARITY 0x00         /* Write bit polarity for ADRV9001 */


/***************************************************************************//**
 * @brief This function prepares an array suitable for SPI write operations to
 * the ADRV9001 device, supporting both full byte writes and read-modify-
 * write (RMW) operations. It should be used when you need to write data
 * to a specific register address, with the option to modify only certain
 * bits using a mask. The function requires a valid device context and
 * ensures that the buffer does not overflow. It is essential to ensure
 * that the `wrData` buffer is large enough to accommodate the data being
 * packed, and that `numWrBytes` accurately reflects the current buffer
 * usage.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param wrData Pointer to the buffer where the packed data will be stored.
 * Must not be null and should have sufficient space to hold the
 * packed data.
 * @param numWrBytes Pointer to a variable that indicates the number of bytes
 * currently used in the `wrData` buffer. It is updated to
 * reflect the new buffer usage after packing.
 * @param addr The register address to which the data will be written.
 * @param mask The mask to apply for RMW operations. If set to 0xFF, a full byte
 * write is performed.
 * @param data The data byte to be written to the specified register address.
 * @param writeFlag A flag to be bitwise OR'd into the MSB of the 16-bit
 * address, indicating the write operation.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_DataPack(adi_adrv9001_Device_t *adrv9001, uint8_t *wrData, uint16_t *numWrBytes, uint16_t addr, uint8_t mask, uint8_t data, uint8_t writeFlag);

/***************************************************************************//**
 * @brief This function prepares an array suitable for SPI stream mode writes to
 * the ADIHAL layer by packing the address and data into the provided
 * buffer. It should be used when multiple register addresses need to be
 * written in a single operation, which is common in stream mode. The
 * function requires a valid device context and ensures that the buffer
 * does not overflow. It is important to ensure that the `wrData` buffer
 * is large enough to accommodate the data being packed, and that
 * `numWrBytes` accurately reflects the current size of the buffer.
 *
 * @param adrv9001 Context variable - Pointer to the ADRV9001 device data
 * structure. Must not be null.
 * @param wrData The resulting array to be set to the HAL layer. Must not be
 * null and should have enough space to accommodate the packed
 * data.
 * @param numWrBytes Pointer to the number of elements filled in the wrData
 * array. Must not be null and should reflect the current
 * number of bytes in the buffer.
 * @param addr The address to be added to wrData. It is a 16-bit value.
 * @param data The data to be added to wrData. It is an array of bytes and must
 * not be null.
 * @param count The number of register addresses to write data to. It determines
 * how many bytes from the data array will be packed.
 * @param writeFlag The value to be bitwise OR'd into the MSB of the 16-bit
 * address. It is used to indicate the write operation.
 * @return Returns a code indicating success (ADI_COMMON_ACT_NO_ACTION) or the
 * required action to recover.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Stream_DataPack(adi_adrv9001_Device_t *adrv9001, uint8_t *wrData, uint32_t *numWrBytes, uint16_t addr, const uint8_t data[], uint32_t count, uint8_t writeFlag);

/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specific
 * register address on the ADRV9001 device. It is essential to ensure
 * that the device pointer is valid before calling this function. The
 * function attempts to write the data multiple times if necessary,
 * handling any potential SPI communication errors internally. It is
 * typically used when precise control over individual register settings
 * is required.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param addr The 16-bit address of the register to which the data will be
 * written. It should be a valid register address within the
 * device's addressable range.
 * @param data The 8-bit value to write to the specified register. It represents
 * the data to be stored at the given address.
 * @return Returns an integer code indicating success or the type of error
 * encountered. A return value of 0 indicates success, while non-zero
 * values indicate an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Byte_Write(adi_adrv9001_Device_t *adrv9001, uint16_t addr, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to write a sequence of bytes to consecutive
 * register addresses on the ADRV9001 device using SPI in stream mode. It
 * is suitable for applications where multiple registers need to be
 * updated in a single operation. The function must be called with a
 * valid device context and a non-null data array. It handles retries
 * internally in case of SPI write errors, ensuring robust communication.
 * This function is typically used in scenarios where high throughput and
 * efficient register updates are required.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param addr The starting address of the register to write to. Must be a valid
 * register address within the device's addressable range.
 * @param data Array of bytes to be written to the device. Must not be null and
 * should contain at least 'count' elements.
 * @param count The number of bytes to write. Must be greater than zero and
 * should not exceed the capacity of the 'data' array.
 * @return Returns an integer code indicating success or the type of error
 * encountered. A return value of 0 indicates success, while non-zero
 * values indicate an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Bytes_Stream_Write(adi_adrv9001_Device_t *adrv9001, uint16_t addr, const uint8_t data[], uint32_t count);

/***************************************************************************//**
 * @brief This function is used to write multiple bytes of data to specified
 * register addresses on the ADRV9001 device using SPI communication. It
 * is essential to ensure that the device pointer is valid before calling
 * this function. The function requires arrays of register addresses and
 * corresponding data values, along with the count of these pairs. It
 * handles potential errors during the SPI write operation and attempts
 * retries if necessary. This function should be used when multiple
 * register writes are needed in a single operation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param addr Array of register addresses to write to. Must not be null.
 * @param data Array of data values to write to the registers. Must not be null.
 * @param count Number of register addresses and data values to write. Must be a
 * positive integer.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Bytes_Write(adi_adrv9001_Device_t *adrv9001, const uint16_t addr[], const uint8_t data[], uint32_t count);

/***************************************************************************//**
 * @brief Use this function to read a single byte of data from a specific
 * register address on the ADRV9001 device. It is essential to ensure
 * that the device context is properly initialized before calling this
 * function. The function attempts to read the data multiple times in
 * case of failure, and it reports any errors encountered during the SPI
 * read operation. This function is useful when precise control over
 * individual register reads is required.
 *
 * @param adrv9001 A pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before use.
 * @param addr The address of the register to read from. It should be a valid
 * register address within the device's addressable range.
 * @param readData A pointer to a uint8_t variable where the read data will be
 * stored. Must not be null, and the caller is responsible for
 * ensuring it points to a valid memory location.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Byte_Read(adi_adrv9001_Device_t *adrv9001, uint16_t addr, uint8_t *readData);

/***************************************************************************//**
 * @brief Use this function to read multiple bytes of data from the specified
 * register addresses of the ADRV9001 device. It is essential to ensure
 * that the device context is properly initialized before calling this
 * function. The function reads data from the provided addresses and
 * stores the results in the readData array. It handles potential read
 * errors by retrying the operation a predefined number of times. This
 * function is useful when multiple register values need to be retrieved
 * in a single operation.
 *
 * @param adrv9001 Pointer to the ADRV9001 device data structure. Must not be
 * null and should be properly initialized before use.
 * @param addr Array of register addresses from which data is to be read. Must
 * not be null and should contain at least 'count' elements.
 * @param readData Array where the read data will be stored. Must not be null
 * and should have space for at least 'count' elements.
 * @param count Number of register addresses to read data from. Must be a
 * positive integer.
 * @return Returns an integer code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Bytes_Read(adi_adrv9001_Device_t *adrv9001, const uint16_t addr[], uint8_t readData[], uint32_t count);

/***************************************************************************//**
 * @brief This function is used to write a specific field value to a register on
 * the ADRV9001 device via SPI. It is useful when only a portion of a
 * register needs to be updated, as it allows for a read-modify-write
 * operation using a mask and a starting bit position. The function must
 * be called with a valid device context, and it handles SPI
 * communication errors internally, retrying the operation if necessary.
 * It is important to ensure that the device pointer is not null before
 * calling this function.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param addr The address of the register to write to. Must be a valid register
 * address within the device.
 * @param fieldVal The value to write to the specified field within the
 * register. It is shifted and masked according to the startBit
 * and mask parameters.
 * @param mask The mask to apply when writing the field value to the register.
 * It determines which bits in the register are affected.
 * @param startBit The bit position where the field starts within the register.
 * It is used to shift the fieldVal to the correct position.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Field_Write(adi_adrv9001_Device_t *adrv9001, uint16_t addr, uint8_t  fieldVal, uint8_t mask, uint8_t startBit);

/***************************************************************************//**
 * @brief This function is used to read a specific field from a register of the
 * ADRV9001 device using SPI communication. It is useful when only a
 * portion of a register's data is needed. The function requires a valid
 * device context and a non-null pointer for the field value output. It
 * reads the register, applies a mask, and shifts the bits to extract the
 * desired field. The function should be called when the device is
 * properly initialized and ready for SPI operations.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null.
 * @param addr The address of the register to read from. Must be a valid
 * register address.
 * @param fieldVal Pointer to a uint8_t where the read field value will be
 * stored. Must not be null.
 * @param mask The mask to apply to the register value to isolate the desired
 * field.
 * @param startBit The bit position where the field starts within the register.
 * Only the lower 4 bits are used.
 * @return Returns an int32_t indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Field_Read(adi_adrv9001_Device_t *adrv9001, uint16_t addr, uint8_t *fieldVal, uint8_t mask, uint8_t startBit);
    
/***************************************************************************//**
 * @brief This function is used to write data to a specific register address on
 * the ADRV9001 device, applying a mask to modify only certain bits of
 * the register. It is typically used when only a subset of bits in a
 * register needs to be updated, rather than the entire register. The
 * function must be called with a valid device context, and the address
 * must be within the valid range for the device. The mask determines
 * which bits of the data are written to the register, allowing for
 * selective bit modification.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null, as it provides the context for the operation.
 * @param addr The address of the register to write to. Must be a valid register
 * address for the ADRV9001 device.
 * @param data The data to write to the register. Only the bits specified by the
 * mask will be written.
 * @param mask The mask to apply to the data. Only the bits corresponding to set
 * bits in the mask will be written to the register.
 * @return Returns an int32_t code indicating success or the required action to
 * recover.
 ******************************************************************************/
    int32_t adi_adrv9001_spi_Mask_Write(adi_adrv9001_Device_t *adrv9001, uint16_t addr, uint8_t data, uint8_t mask);

/***************************************************************************//**
 * @brief This function is used to write a series of data values to the ADRV9001
 * device through the SPI interface. It is intended for use when multiple
 * register writes are required, and it optimizes the process by caching
 * the writes. The function must be called with a valid device context
 * and a non-null array of data to be written. It handles potential SPI
 * write errors by retrying the operation a predefined number of times.
 * This function is typically used in scenarios where batch updates to
 * the device registers are necessary.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the device context. Must not be null.
 * @param wrCache An array of 32-bit unsigned integers containing the data to be
 * written to the device registers. The caller retains ownership
 * of the array.
 * @param count The number of elements in the wrCache array to be written. Must
 * be a positive integer.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Cache_Write(adi_adrv9001_Device_t *adrv9001, const uint32_t wrCache[], uint32_t count);

/***************************************************************************//**
 * @brief This function is used to read data from multiple register addresses
 * specified in the `rdCache` array. It is essential to ensure that the
 * `device` is properly initialized before calling this function. The
 * function will attempt to read the specified number of bytes (`count`)
 * from the device and store the results in the `readData` array. It is
 * important to provide valid pointers for `rdCache` and `readData`, as
 * the function will return an error if these are null. The function will
 * retry the read operation multiple times in case of failure, and it
 * will report any errors encountered during the process.
 *
 * @param device Pointer to the ADRV9001 device data structure. Must not be
 * null. The caller retains ownership.
 * @param rdCache Array of register addresses to read from. Must not be null.
 * The caller retains ownership.
 * @param readData Array where the read data will be stored. Must not be null.
 * The caller retains ownership.
 * @param count Number of register addresses to read data from. Must be a valid
 * number within the range of the arrays provided.
 * @return Returns an integer code indicating success or the required action to
 * recover from an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Cache_Read(adi_adrv9001_Device_t *adrv9001, const uint32_t rdCache[], uint8_t readData[], uint32_t count);

/***************************************************************************//**
 * @brief This function is used to configure the SPI Master settings for the
 * ADRV9001 device. It should be called when the SPI Master needs to be
 * set up with specific parameters such as baud rate, transfer mode, and
 * connected slave devices. The function requires a valid device context
 * and a properly initialized SPI Master configuration structure. It
 * performs validation on the input parameters and writes the
 * configuration to the device. This function is typically used during
 * the initialization phase or when reconfiguring the SPI Master
 * settings.
 *
 * @param device A pointer to the adi_adrv9001_Device_t structure representing
 * the ADRV9001 device context. Must not be null. The caller
 * retains ownership.
 * @param spiMasterConfig A pointer to the adi_adrv9001_spiMasterConfig_t
 * structure containing the desired SPI Master
 * configuration settings. Must not be null. The
 * structure should be properly initialized with valid
 * configuration values before calling this function.
 * @return Returns an int32_t code indicating success (ADI_COMMON_ACT_NO_ACTION)
 * or the required action to recover in case of an error.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Master_Configure(adi_adrv9001_Device_t *adrv9001,
										  adi_adrv9001_spiMasterConfig_t *spiMasterConfig);

/***************************************************************************//**
 * @brief Use this function to retrieve the current configuration of the SPI
 * Master for the specified ADRV9001 device. This function is useful for
 * verifying the current settings of the SPI Master, such as baud rate,
 * transfer mode, and connected slave devices. It must be called with a
 * valid device context and a pointer to a configuration structure where
 * the retrieved settings will be stored. Ensure that the device has been
 * properly initialized before calling this function.
 *
 * @param adrv9001 A pointer to the ADRV9001 device settings data structure.
 * Must not be null and should point to a valid, initialized
 * device context.
 * @param spiMasterConfig A pointer to an adi_adrv9001_spiMasterConfig_t
 * structure where the current SPI Master configuration
 * will be stored. Must not be null.
 * @return Returns an int32_t code indicating success or the required action to
 * recover. The spiMasterConfig structure is populated with the current
 * SPI Master settings on success.
 ******************************************************************************/
int32_t adi_adrv9001_spi_Master_Inspect(adi_adrv9001_Device_t *adrv9001,
										adi_adrv9001_spiMasterConfig_t *spiMasterConfig);

#ifdef __cplusplus
}
#endif

#endif /* _ADI_ADRV9001_SPI_H_ */
