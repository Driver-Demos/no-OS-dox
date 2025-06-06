/**
 * \file talise_hal.h
 * \brief Contains prototypes and macro definitions for Private ADI HAL wrapper
 *        functions implemented in talise_hal.c
 *
 * Talise API version: 3.6.2.1
 *
 * Copyright 2015-2017 Analog Devices Inc.
 * Released under the AD9378-AD9379 API license, for more information see the "LICENSE.txt" file in this zip file.
 */

#ifndef TALISE_HAL_H_
#define TALISE_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "adi_hal.h"

#define HAL_TIMEOUT_DEFAULT 100         /* 100ms */
#define HAL_TIMEOUT_NONE 0x0            /* Non-blocking */
#define HAL_TIMEOUT_INFINITE 0xFFFFFFFF /* Blocking */
#define HAL_TIMEOUT_MULT 2              /* HAL timeout worse-case factor */

/***************************************************************************//**
 * @brief This function reads a specific field from a SPI register using a bit
 * mask and starting bit position. It should be called after the device
 * HAL information has been properly initialized. The function handles
 * potential timeout errors by retrying the read operation once with an
 * adjusted timeout setting. It is useful for accessing specific bits
 * within a register without affecting other bits.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. Caller retains ownership.
 * @param addr 16-bit SPI address from which the field is to be read. Must be a
 * valid address within the device's addressable range.
 * @param fieldVal Pointer to a byte where the read field value will be stored.
 * Must not be null. The function writes the result to this
 * location.
 * @param mask Bit mask used to isolate the desired field within the register.
 * Must be a valid bit mask corresponding to the field of interest.
 * @param startBit Starting bit position for the field mask operation. Must be
 * within the valid range for the register size.
 * @return Returns an adiHalErr_t enumerated type indicating success or the type
 * of error encountered.
 ******************************************************************************/
adiHalErr_t talSpiReadField(void *devHalInfo, uint16_t addr, uint8_t *fieldVal,
			    uint8_t mask, uint8_t startBit);

/***************************************************************************//**
 * @brief This function writes a byte value to a specific field within a device
 * register using SPI communication. It should be called after the device
 * HAL information has been properly initialized. The function handles
 * potential timeout errors by retrying the write operation once with an
 * adjusted timeout setting. It is useful for updating specific bits
 * within a register without affecting other bits. Ensure that the
 * provided device information and address are valid before calling this
 * function.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before use. Caller retains
 * ownership and it must not be null.
 * @param addr 16-bit SPI address of the register to be written to. Must be a
 * valid address within the device's addressable range.
 * @param fieldVal Byte value to be written to the specified field. Represents
 * the new value for the field defined by the mask and startBit.
 * @param mask Bit mask indicating the specific field within the register to be
 * modified. Only the bits set in the mask will be affected by the
 * write operation.
 * @param startBit Starting bit position for the field mask operation. Defines
 * where the field begins within the register.
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the operation, including handling of timeout errors.
 ******************************************************************************/
adiHalErr_t talSpiWriteField(void *devHalInfo, uint16_t addr, uint8_t fieldVal,
			     uint8_t mask, uint8_t startBit);

/***************************************************************************//**
 * @brief This function logs a message to the system log with a specified
 * logging level, error code, and comment. It should be called after the
 * device HAL information has been properly initialized. The function is
 * useful for recording error messages or other significant events during
 * the operation of the system. It returns an error code indicating the
 * success or failure of the logging operation.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. The caller retains ownership and it must not be
 * null.
 * @param logLevel Enumerated type indicating the logging level. It specifies
 * the severity or importance of the log message.
 * @param errorCode 32-bit unsigned integer representing the error code to be
 * logged. It should correspond to a specific error or event.
 * @param comment Pointer to a null-terminated string containing a comment or
 * description of the log entry. The caller retains ownership and
 * it must not be null.
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the logging operation.
 ******************************************************************************/
adiHalErr_t talWriteToLog(void *devHalInfo, adiLogLevel_t logLevel,
			  uint32_t errorCode, const char *comment);

/***************************************************************************//**
 * @brief This function writes a single byte to a specified SPI address using
 * the provided device HAL information. It should be called after the
 * device HAL information has been properly initialized. The function
 * includes error handling for timeout scenarios, attempting a retry with
 * an adjusted timeout if the initial write operation times out. It is
 * suitable for use in environments where reliable SPI communication is
 * required, and it ensures that the operation is retried once in case of
 * a timeout before resetting the timeout to its default value.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. The caller retains ownership and it must not be
 * null.
 * @param addr 16-bit SPI address where the byte will be written. It should be a
 * valid address within the device's addressable range.
 * @param data Byte value to be written to the specified SPI address. It should
 * be a valid byte value (0-255).
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the operation, including handling of timeout errors.
 ******************************************************************************/
adiHalErr_t talSpiWriteByte(void *devHalInfo, uint16_t addr, uint8_t data);

/***************************************************************************//**
 * @brief This function reads a single byte from a specified SPI address using
 * the provided device HAL information. It should be called after the
 * device HAL information has been properly initialized. The function
 * attempts to read the byte and handles potential timeout errors by
 * retrying the read operation once with an adjusted timeout setting. It
 * is useful for retrieving data from a device over SPI while managing
 * potential communication delays.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. The caller retains ownership.
 * @param addr 16-bit SPI address from which the byte will be read. Must be a
 * valid address within the device's addressable range.
 * @param readdata Pointer to a byte where the read data will be stored. Must
 * not be null, as the function writes the read byte to this
 * location.
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the read operation, including handling of timeout errors.
 ******************************************************************************/
adiHalErr_t talSpiReadByte(void *devHalInfo, uint16_t addr, uint8_t *readdata);

/***************************************************************************//**
 * @brief This function writes a specified number of bytes to a given SPI
 * address using the provided device HAL information. It is designed to
 * handle errors that may occur during the SPI write operation, including
 * retrying the operation if a timeout error is encountered. This
 * function should be called after the device HAL information has been
 * properly initialized. It is useful for applications that require
 * writing multiple bytes to a device over SPI, ensuring robust error
 * handling and retry mechanisms.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. The caller retains ownership and it must not be
 * null.
 * @param addr Pointer to a 16-bit SPI address where the data will be written.
 * The address must be valid and the caller retains ownership.
 * @param data Pointer to the byte array containing the data to be written. The
 * array must contain at least 'count' bytes. The caller retains
 * ownership and it must not be null.
 * @param count Number of bytes to write. Must be a positive integer indicating
 * how many bytes from the 'data' array should be written to the
 * SPI address.
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the operation, including specific error codes for timeout
 * and other errors.
 ******************************************************************************/
adiHalErr_t talSpiWriteBytes(void *devHalInfo, uint16_t *addr, uint8_t *data,
			     uint32_t count);

/***************************************************************************//**
 * @brief This function reads a specified number of bytes from a given SPI
 * address into a provided buffer, with built-in error handling for
 * timeout scenarios. It should be called after the device HAL
 * information has been properly initialized. The function attempts to
 * read the bytes and, in case of a timeout error, it retries the
 * operation once with an adjusted timeout setting. This function is
 * useful for retrieving data from SPI devices where multiple bytes need
 * to be read in a single operation.
 *
 * @param devHalInfo Pointer to the device HAL information container. Must be
 * initialized with valid settings before calling this
 * function. The caller retains ownership.
 * @param addr Pointer to a 16-bit SPI address from which the data will be read.
 * The address must be valid and accessible.
 * @param readdata Pointer to a byte array where the read data will be stored.
 * Must be large enough to hold 'count' bytes. The caller is
 * responsible for allocating and managing this memory.
 * @param count Number of bytes to read from the specified SPI address. Must be
 * a positive integer.
 * @return Returns an adiHalErr_t enumerated type indicating the success or
 * failure of the read operation, including specific error codes for
 * timeout scenarios.
 ******************************************************************************/
adiHalErr_t talSpiReadBytes(void *devHalInfo, uint16_t *addr, uint8_t *readdata,
			    uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* TALISE_HAL_H_ */
