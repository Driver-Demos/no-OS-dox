/*!
 * @brief     APIs to call HAL functions
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup AD9083_HAL
 * @{
 */
#ifndef __AD9083_HAL_H__
#define __AD9083_HAL_H__

/*============= I N C L U D E S ============*/
#include "adi_ad9083_config.h"
#ifdef __KERNEL__
#include <linux/math64.h>
#endif

/*============= D E F I N E S ==============*/
#define SPI_IN_OUT_BUFF_SZ 0x3

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to open the hardware interface associated with a
 * given AD9083 device. It must be called before any hardware operations
 * are performed on the device to ensure that the hardware interface is
 * properly initialized and ready for communication. The function
 * requires a valid device structure with a non-null hardware open
 * function pointer. If the device or the hardware open function pointer
 * is null, the function will return an error. Additionally, if the
 * hardware open operation fails, an error code will be returned.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device for which the hardware interface is to be opened. Must
 * not be null, and the device's hal_info.hw_open must also be
 * non-null. The caller retains ownership of the device structure.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or an
 * error code if the device pointer is null, the hardware open function
 * pointer is null, or the hardware open operation fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_hw_open(adi_ad9083_device_t *device);
/***************************************************************************//**
 * @brief This function is used to close the hardware interface associated with
 * a given AD9083 device. It should be called when the hardware interface
 * is no longer needed, typically during cleanup or shutdown procedures.
 * The function requires a valid device pointer and expects that the
 * device's hardware close function is properly set. If the device
 * pointer is null or the hardware close function is not set, the
 * function will return an error. Successful execution results in a
 * confirmation of the hardware interface closure.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device whose hardware interface is to be closed. Must not be
 * null, and the device's hal_info.hw_close must be set. If these
 * conditions are not met, the function returns an error.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or
 * API_CMS_ERROR_HW_CLOSE if the hardware close operation fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_hw_close(adi_ad9083_device_t *device);
/***************************************************************************//**
 * @brief This function introduces a delay in the execution flow for a specified
 * number of microseconds, utilizing the hardware abstraction layer (HAL)
 * associated with the given device. It is typically used when precise
 * timing is required, such as in hardware communication protocols. The
 * function must be called with a valid device structure that has been
 * properly initialized, and the delay function pointer within the
 * device's HAL information must not be null. If the delay cannot be
 * executed, an error code is returned.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null and must have a valid delay_us
 * function pointer in its hal_info field.
 * @param us The number of microseconds to delay. Must be a non-negative
 * integer.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or
 * API_CMS_ERROR_DELAY_US if the delay operation fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_delay_us(adi_ad9083_device_t *device, uint32_t us);
/***************************************************************************//**
 * @brief This function is used to control the reset pin of the AD9083 device by
 * enabling or disabling it based on the provided parameter. It must be
 * called with a valid device structure that has been properly
 * initialized. The function checks for null pointers and returns an
 * error code if the reset pin control operation fails. It is essential
 * to ensure that the device's hardware abstraction layer (HAL) is
 * correctly set up before invoking this function.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null and should be properly initialized
 * with valid HAL information.
 * @param enable A uint8_t value indicating whether to enable (non-zero value)
 * or disable (zero value) the reset pin. Any non-zero value will
 * enable the reset pin.
 * @return Returns an int32_t error code: API_CMS_ERROR_OK on success, or
 * API_CMS_ERROR_RESET_PIN_CTRL if the reset pin control operation
 * fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_reset_pin_ctrl(adi_ad9083_device_t *device,
                                      uint8_t enable);
/***************************************************************************//**
 * @brief This function is used to write a formatted log message through the
 * device's logging mechanism. It requires a valid device structure with
 * a non-null logging function pointer. The function supports variable
 * argument lists, allowing for flexible message formatting. It should be
 * called whenever a log message needs to be recorded, and it returns an
 * error code if the logging function is not set or if any other error
 * occurs during logging.
 *
 * @param device A pointer to an adi_ad9083_device_t structure. Must not be null
 * and must have a valid log_write function pointer in its
 * hal_info field.
 * @param log_type Specifies the type of log message to be written. It is of
 * type adi_cms_log_type_e.
 * @param comment A format string for the log message, similar to printf. Must
 * not be null.
 * @param ... Additional arguments for the format string specified in comment.
 * These are optional and depend on the format string.
 * @return Returns an int32_t error code. A non-zero value indicates an error,
 * such as a null logging function pointer.
 ******************************************************************************/
int32_t adi_ad9083_hal_log_write(adi_ad9083_device_t *device,
                                 adi_cms_log_type_e type, const char *comment,
                                 ...);

/***************************************************************************//**
 * @brief This function is used to extract a bitfield from a specified register
 * of the AD9083 device. It is useful when you need to read specific bits
 * from a register, defined by the bitfield information. The function
 * requires a valid device pointer and a non-null value buffer to store
 * the result. The bitfield is specified by the 'info' parameter, which
 * encodes the offset and width of the bitfield. The function handles
 * cases where the bitfield spans multiple bytes and ensures the result
 * is stored in the provided buffer according to the specified byte size.
 * It must be called with valid parameters, as invalid width or value
 * size will result in an error.
 *
 * @param device A pointer to the adi_ad9083_device_t structure representing the
 * device. Must not be null.
 * @param reg The register address from which the bitfield is to be extracted.
 * Must be less than 0x1000.
 * @param info A 32-bit value encoding the bitfield's offset (lower 8 bits) and
 * width (next 8 bits). Width must be between 1 and 64.
 * @param value A pointer to a buffer where the extracted bitfield value will be
 * stored. Must not be null and should have enough space to hold
 * 'value_size_bytes'.
 * @param value_size_bytes The number of bytes to store the extracted bitfield
 * value. Must be 8 or less.
 * @return Returns an int32_t indicating success or an error code. The extracted
 * bitfield value is stored in the 'value' buffer.
 ******************************************************************************/
int32_t adi_ad9083_hal_bf_get(adi_ad9083_device_t *device, uint32_t reg,
                              uint32_t info, uint8_t *value,
                              uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to set a specific bitfield within a register of
 * the AD9083 device. It is essential to ensure that the device pointer
 * is valid and that the bitfield width specified in the `info` parameter
 * is between 1 and 64 bits. The function modifies the register by
 * applying the provided value to the specified bitfield, taking care to
 * preserve other bits in the register. It should be called when a
 * specific configuration or setting needs to be applied to the device's
 * register. The function assumes that the register address is valid and
 * within the expected range.
 *
 * @param device A pointer to an `adi_ad9083_device_t` structure representing
 * the device. Must not be null.
 * @param reg The register address where the bitfield is to be set. Must be a
 * valid register address within the device's address space.
 * @param info A 32-bit value where the lower 8 bits specify the bitfield offset
 * and the next 8 bits specify the bitfield width. Width must be
 * between 1 and 64.
 * @param value The value to set in the specified bitfield. The value should fit
 * within the specified bitfield width.
 * @return Returns an `int32_t` indicating success or an error code. The
 * register is modified to reflect the new bitfield value.
 ******************************************************************************/
int32_t adi_ad9083_hal_bf_set(adi_ad9083_device_t *device, uint32_t reg,
                              uint32_t info, uint64_t value);

/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9083 device. It requires a valid device handle and a register
 * address to perform the read operation. The function will store the
 * retrieved register value in the provided data pointer. It is essential
 * that the device and data pointers are not null, and the register
 * address should be within the valid range (less than 0x1000). If the
 * SPI transfer fails, an error code is returned. This function should be
 * called only after the device has been properly initialized and
 * configured.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg A 32-bit unsigned integer representing the register address to
 * read from. Must be less than 0x1000.
 * @param data A pointer to a uint8_t where the read register value will be
 * stored. Must not be null. The caller retains ownership.
 * @return Returns an int32_t indicating success or an error code if the
 * operation fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_reg_get(adi_ad9083_device_t *device, uint32_t reg,
                               uint8_t *data);
/***************************************************************************//**
 * @brief This function is used to write a single byte of data to a specified
 * register on the AD9083 device using the SPI interface. It is essential
 * to ensure that the `device` pointer and its `spi_xfer` function
 * pointer are not null before calling this function, as these are
 * required for the SPI communication. The register address must be less
 * than 0x1000, as higher addresses are not supported by this function.
 * The function returns an error code if the SPI transfer fails, allowing
 * the caller to handle communication errors appropriately.
 *
 * @param device A pointer to an `adi_ad9083_device_t` structure representing
 * the device. Must not be null, and the `spi_xfer` function
 * pointer within this structure must also be valid.
 * @param reg The register address to which the data will be written. Must be
 * less than 0x1000.
 * @param data The byte of data to write to the specified register.
 * @return Returns an integer status code: `API_CMS_ERROR_OK` on success, or
 * `API_CMS_ERROR_SPI_XFER` if the SPI transfer fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_reg_set(adi_ad9083_device_t *device, uint32_t reg,
                               uint8_t data);

/***************************************************************************//**
 * @brief Use this function to log an error with specific context details such
 * as the file name, function name, line number, and variable name where
 * the error occurred. This function is useful for debugging and tracking
 * errors in the system. It requires a valid device handle and will
 * return an error code if the device is null or if the logging operation
 * fails. Ensure that the device is properly initialized before calling
 * this function.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param log_type An adi_cms_log_type_e value indicating the type of log entry
 * to create. Must be a valid log type.
 * @param error An integer representing the error code to report. Can be any
 * valid error code.
 * @param file_name A string representing the name of the file where the error
 * occurred. Must not be null.
 * @param func_name A string representing the name of the function where the
 * error occurred. Must not be null.
 * @param line_num An unsigned integer representing the line number in the file
 * where the error occurred.
 * @param var_name A string representing the name of the variable related to the
 * error. Must not be null.
 * @param comment A string containing additional comments or context about the
 * error. Must not be null.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success,
 * API_CMS_ERROR_NULL_PARAM if the device is null, or
 * API_CMS_ERROR_LOG_WRITE if logging fails.
 ******************************************************************************/
int32_t adi_ad9083_hal_error_report(adi_ad9083_device_t *device,
                                    adi_cms_log_type_e log_type, int32_t error,
                                    const char *file_name,
                                    const char *func_name, uint32_t line_num,
                                    const char *var_name, const char *comment);

/***************************************************************************//**
 * @brief This function is used to set a specific register within the CBUS PLL
 * of an AD9083 device. It should be called when there is a need to
 * configure or modify the PLL settings of the device. The function
 * requires a valid device handle and the register address and data to be
 * written. It performs necessary operations to ensure the data is
 * correctly written to the specified register. The function must be
 * called with a non-null device pointer, and it returns an error code if
 * any operation fails during the process.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The address of the register to be set within the CBUS PLL. Must be
 * a valid register address.
 * @param data The data byte to be written to the specified register. Must be a
 * valid 8-bit value.
 * @return Returns an int32_t error code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t adi_ad9083_hal_cbuspll_reg_set(adi_ad9083_device_t *device,
                                       uint32_t reg, uint8_t data);
/***************************************************************************//**
 * @brief Use this function to read a value from a specific CBUS PLL register on
 * the AD9083 device. It is essential to ensure that both the device and
 * data pointers are valid and not null before calling this function. The
 * function performs a series of register operations to access the
 * desired register value, which is then stored in the location pointed
 * to by the data parameter. This function should be called when you need
 * to obtain the current value of a CBUS PLL register for monitoring or
 * configuration purposes.
 *
 * @param device A pointer to an adi_ad9083_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address from which to retrieve the value. It is a
 * 32-bit unsigned integer specifying the target register.
 * @param data A pointer to a uint8_t variable where the retrieved register
 * value will be stored. Must not be null. The caller retains
 * ownership.
 * @return Returns an int32_t indicating success or failure. A return value of 0
 * indicates success, while a non-zero value indicates an error occurred
 * during the operation.
 ******************************************************************************/
int32_t adi_ad9083_hal_cbuspll_reg_get(adi_ad9083_device_t *device,
                                       uint32_t reg, uint8_t *data);

/***************************************************************************//**
 * @brief This function computes the integer and fractional parts of the
 * division of two unsigned 64-bit integers, `a` and `b`. It is useful
 * when both the quotient and a simplified fractional representation of
 * the remainder are needed. The function must be called with valid
 * pointers for `device`, `integer`, `nume`, and `deno`, and `b` must not
 * be zero to avoid division by zero errors. The function returns an
 * error code if any of these preconditions are not met.
 *
 * @param device A pointer to an `adi_ad9083_device_t` structure. Must not be
 * null. The caller retains ownership.
 * @param a The dividend, an unsigned 64-bit integer.
 * @param b The divisor, an unsigned 64-bit integer. Must not be zero.
 * @param integer A pointer to a uint8_t where the integer part of the division
 * will be stored. Must not be null.
 * @param nume A pointer to a uint8_t where the numerator of the fractional part
 * will be stored. Must not be null.
 * @param deno A pointer to a uint8_t where the denominator of the fractional
 * part will be stored. Must not be null.
 * @return Returns an int32_t error code. On success, `integer` contains the
 * integer part of the division, and `nume` and `deno` contain the
 * simplified fractional part.
 ******************************************************************************/
int32_t adi_ad9083_hal_div_nume_deno(adi_ad9083_device_t *device, uint64_t a,
                                     uint64_t b, uint8_t *integer,
                                     uint8_t *nume, uint8_t *deno);

/***************************************************************************//**
 * @brief This function adds two 128-bit numbers, each represented by a pair of
 * 64-bit unsigned integers, and stores the result in two separate 64-bit
 * unsigned integers. It is useful when dealing with large numbers that
 * exceed the standard 64-bit integer range. The function handles carry-
 * over from the lower 64 bits to the higher 64 bits, ensuring accurate
 * results for the full 128-bit addition.
 *
 * @param ah The high 64 bits of the first 128-bit number. Must be a valid
 * 64-bit unsigned integer.
 * @param al The low 64 bits of the first 128-bit number. Must be a valid 64-bit
 * unsigned integer.
 * @param bh The high 64 bits of the second 128-bit number. Must be a valid
 * 64-bit unsigned integer.
 * @param bl The low 64 bits of the second 128-bit number. Must be a valid
 * 64-bit unsigned integer.
 * @param hi Pointer to a 64-bit unsigned integer where the high 64 bits of the
 * result will be stored. Must not be null.
 * @param lo Pointer to a 64-bit unsigned integer where the low 64 bits of the
 * result will be stored. Must not be null.
 * @return The function stores the result of the addition in the memory
 * locations pointed to by 'hi' and 'lo'.
 ******************************************************************************/
void adi_ad9083_hal_add_128(uint64_t ah, uint64_t al, uint64_t bh, uint64_t bl,
                            uint64_t *hi, uint64_t *lo);
/***************************************************************************//**
 * @brief This function performs subtraction of two 128-bit unsigned integers,
 * represented by two 64-bit high and low parts each. It is used when
 * precise arithmetic operations on large integers are required, such as
 * in hardware abstraction layers or digital signal processing. The
 * function expects valid pointers for the result storage and handles the
 * borrow operation internally. It should be called when both input
 * numbers are properly initialized and the result pointers are ready to
 * receive the output.
 *
 * @param ah The high 64 bits of the first 128-bit unsigned integer. Must be a
 * valid 64-bit unsigned integer.
 * @param al The low 64 bits of the first 128-bit unsigned integer. Must be a
 * valid 64-bit unsigned integer.
 * @param bh The high 64 bits of the second 128-bit unsigned integer. Must be a
 * valid 64-bit unsigned integer.
 * @param bl The low 64 bits of the second 128-bit unsigned integer. Must be a
 * valid 64-bit unsigned integer.
 * @param hi Pointer to a 64-bit unsigned integer where the high 64 bits of the
 * result will be stored. Must not be null.
 * @param lo Pointer to a 64-bit unsigned integer where the low 64 bits of the
 * result will be stored. Must not be null.
 * @return The function stores the result of the subtraction in the memory
 * locations pointed to by 'hi' and 'lo'.
 ******************************************************************************/
void adi_ad9083_hal_subt_128(uint64_t ah, uint64_t al, uint64_t bh, uint64_t bl,
                             uint64_t *hi, uint64_t *lo);
/***************************************************************************//**
 * @brief This function multiplies two 64-bit unsigned integers and produces a
 * 128-bit result, which is split into high and low 64-bit parts. It is
 * useful when dealing with large integer arithmetic where the result may
 * exceed the standard 64-bit integer size. The function requires valid
 * pointers for the high and low result parts, which will be populated
 * with the respective portions of the 128-bit product. It is important
 * to ensure that these pointers are not null to avoid undefined
 * behavior.
 *
 * @param a The first 64-bit unsigned integer operand for the multiplication.
 * Must be a valid 64-bit unsigned integer.
 * @param b The second 64-bit unsigned integer operand for the multiplication.
 * Must be a valid 64-bit unsigned integer.
 * @param hi A pointer to a 64-bit unsigned integer where the high 64 bits of
 * the result will be stored. Must not be null.
 * @param lo A pointer to a 64-bit unsigned integer where the low 64 bits of the
 * result will be stored. Must not be null.
 * @return None
 ******************************************************************************/
void adi_ad9083_hal_mult_128(uint64_t a, uint64_t b, uint64_t *hi,
                             uint64_t *lo);
/***************************************************************************//**
 * @brief This function shifts a 128-bit integer, represented by two 64-bit
 * unsigned integers, one bit to the left. It is useful when working with
 * large integers that exceed the standard 64-bit size. The function
 * takes two pointers to 64-bit unsigned integers, which represent the
 * high and low parts of the 128-bit integer, and modifies them in place.
 * The most significant bit of the low part is carried over to the least
 * significant bit of the high part during the shift. This function
 * should be used when a left shift operation is needed on a 128-bit
 * integer, and it assumes that the pointers provided are valid and point
 * to the intended 64-bit integer parts.
 *
 * @param hi A pointer to the 64-bit unsigned integer representing the high part
 * of the 128-bit integer. Must not be null, as it is modified in
 * place.
 * @param lo A pointer to the 64-bit unsigned integer representing the low part
 * of the 128-bit integer. Must not be null, as it is modified in
 * place.
 * @return None
 ******************************************************************************/
void adi_ad9083_hal_lshift_128(uint64_t *hi, uint64_t *lo);
/***************************************************************************//**
 * @brief Use this function to perform a logical right shift on a 128-bit
 * integer, which is split into two 64-bit parts. The function shifts the
 * lower 64 bits and the higher 64 bits to the right by one bit. It is
 * important to ensure that the pointers provided are valid and point to
 * the 64-bit integers representing the high and low parts of the 128-bit
 * integer. This function modifies the values pointed to by the input
 * parameters directly.
 *
 * @param hi A pointer to the 64-bit integer representing the high part of the
 * 128-bit integer. Must not be null. The value is modified in place.
 * @param lo A pointer to the 64-bit integer representing the low part of the
 * 128-bit integer. Must not be null. The value is modified in place.
 * @return None
 ******************************************************************************/
void adi_ad9083_hal_rshift_128(uint64_t *hi, uint64_t *lo);
/***************************************************************************//**
 * @brief This function divides a 128-bit unsigned integer, represented by two
 * 64-bit parts (a_hi and a_lo), by another 128-bit unsigned integer,
 * also represented by two 64-bit parts (b_hi and b_lo). The result of
 * the division is stored in two 64-bit parts pointed to by hi and lo.
 * This function is useful when working with large numbers that exceed
 * the standard 64-bit integer range. It is important to ensure that the
 * divisor (b_hi and b_lo) is not zero to avoid undefined behavior. The
 * function does not handle division by zero, and the caller must ensure
 * valid input values.
 *
 * @param a_hi The high 64 bits of the 128-bit dividend. Must be a valid 64-bit
 * unsigned integer.
 * @param a_lo The low 64 bits of the 128-bit dividend. Must be a valid 64-bit
 * unsigned integer.
 * @param b_hi The high 64 bits of the 128-bit divisor. Must be a valid 64-bit
 * unsigned integer and, together with b_lo, must not represent
 * zero.
 * @param b_lo The low 64 bits of the 128-bit divisor. Must be a valid 64-bit
 * unsigned integer and, together with b_hi, must not represent
 * zero.
 * @param hi Pointer to a 64-bit unsigned integer where the high 64 bits of the
 * result will be stored. Must not be null.
 * @param lo Pointer to a 64-bit unsigned integer where the low 64 bits of the
 * result will be stored. Must not be null.
 * @return The function writes the result of the division into the memory
 * locations pointed to by hi and lo.
 ******************************************************************************/
void adi_ad9083_hal_div_128(uint64_t a_hi, uint64_t a_lo, uint64_t b_hi,
                            uint64_t b_lo, uint64_t *hi, uint64_t *lo);

#ifdef __cplusplus
}
#endif

#endif /* __AD9083_HAL__ */

/*! @} */