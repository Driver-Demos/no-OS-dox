/*!
 * @brief     APIs to call HAL functions
 *
 * @copyright copyright(c) 2018 analog devices, inc. all rights reserved.
 *            This software is proprietary to Analog Devices, Inc. and its
 *            licensor. By using this software you agree to the terms of the
 *            associated analog devices software license agreement.
 */

/*!
 * @addtogroup AD9081_HAL_API
 * @{
 */
#ifndef __AD9081_HAL_H__
#define __AD9081_HAL_H__

/*============= I N C L U D E S ============*/
#include "adi_ad9081_config.h"
#ifdef __KERNEL__
#include <linux/math64.h>
#endif

/*============= E X P O R T S ==============*/
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
 * @brief This function is used to open and initialize the necessary hardware
 * resources and peripherals required for the operation of the TxFE
 * device. It should be called before any other operations that depend on
 * these resources. The function checks if the device pointer is valid
 * and if a hardware open function is defined in the device's HAL
 * information. If the hardware open function is available, it attempts
 * to execute it and returns an error code if it fails. This function is
 * essential for setting up the device and must be called before
 * performing any device-specific operations.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * function will return an error if this pointer is null. The
 * caller retains ownership of the device structure.
 * @return Returns API_CMS_ERROR_OK on success, indicating that the resources
 * were successfully opened and initialized. If the hardware open
 * function fails, it returns API_CMS_ERROR_HW_OPEN.
 ******************************************************************************/
int32_t adi_ad9081_hal_hw_open(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to shut down and close any resources that were
 * opened by a previous call to the hardware open function for the
 * specified device. It should be called when the device is no longer
 * needed, to ensure that all resources are properly released. The
 * function requires a valid device pointer and will return an error code
 * if the hardware close operation fails. It is important to ensure that
 * the device pointer is not null before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * function will return an error if this pointer is null.
 * @return Returns API_CMS_ERROR_OK on success, or API_CMS_ERROR_HW_CLOSE if the
 * hardware close operation fails.
 ******************************************************************************/
int32_t adi_ad9081_hal_hw_close(adi_ad9081_device_t *device);

/***************************************************************************//**
 * @brief This function is used to introduce a delay in the execution of a
 * program, specified in microseconds, by utilizing the hardware
 * abstraction layer (HAL) associated with the given device. It is
 * typically called when precise timing is required in the operation of
 * the device. The function requires a valid device pointer and a non-
 * zero delay duration. If the device pointer or the delay function
 * within the device's HAL is null, the function will return an error
 * code. It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * function will return an error if this pointer is null.
 * @param us The delay duration in microseconds. Must be a non-zero value. The
 * function will return an error if the delay cannot be performed.
 * @return Returns API_CMS_ERROR_OK on success, or API_CMS_ERROR_DELAY_US if the
 * delay operation fails.
 ******************************************************************************/
int32_t adi_ad9081_hal_delay_us(adi_ad9081_device_t *device, uint32_t us);

/***************************************************************************//**
 * @brief This function is used to control the state of the RESETB pin on an ADI
 * device, allowing it to be set either high or low. It should be called
 * when there is a need to reset the device or bring it out of reset. The
 * function requires a valid device handler and a control function
 * pointer within the device structure. If the device or the control
 * function pointer is null, the function will return an error. This
 * function is typically used during device initialization or shutdown
 * procedures.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * structure should contain a valid reset_pin_ctrl function
 * pointer.
 * @param enable A uint8_t value where 0 sets the RESETB pin low and 1 sets it
 * high. Values outside this range are not valid and will result
 * in an error.
 * @return Returns API_CMS_ERROR_OK on success, or API_CMS_ERROR_RESET_PIN_CTRL
 * if there is an error controlling the reset pin.
 ******************************************************************************/
int32_t adi_ad9081_hal_reset_pin_ctrl(adi_ad9081_device_t *device,
				      uint8_t enable);

/***************************************************************************//**
 * @brief This function is used to write log messages of varying verbosity
 * levels through the device's logging mechanism. It should be called
 * when there is a need to log information, warnings, or errors related
 * to the device's operation. The function requires a valid device
 * pointer and a log type that specifies the verbosity level. If the log
 * type includes verbose reporting and the device's logging function is
 * set, the message is formatted and passed to the logging function. The
 * function returns an error code if logging fails, otherwise it returns
 * success.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * device must have a valid logging function set in its hal_info
 * structure for logging to occur.
 * @param type Specifies the type of log message, including verbosity level.
 * Must be a valid adi_cms_log_type_e value.
 * @param comment A format string for the log message, similar to printf. Must
 * not be null. Additional arguments should follow to match the
 * format specifiers in the string.
 * @return Returns API_CMS_ERROR_OK on success, or API_CMS_ERROR_LOG_WRITE if
 * logging fails.
 ******************************************************************************/
int32_t adi_ad9081_hal_log_write(adi_ad9081_device_t *device,
				 adi_cms_log_type_e type, const char *comment,
				 ...);

/***************************************************************************//**
 * @brief This function is used to extract a bitfield value from a specified
 * register of the AD9081 device. It requires a valid device pointer and
 * expects the register address and bitfield information to be provided.
 * The bitfield information should specify the offset and width of the
 * bitfield within the register. The extracted value is stored in the
 * provided buffer, which must be large enough to hold the result. The
 * function handles both standard and extended register spaces and
 * ensures that the bitfield width does not exceed 64 bits. It must be
 * called with valid parameters, as invalid inputs will result in error
 * returns.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which the bitfield is to be extracted.
 * Must be a valid register address.
 * @param info Bitfield information containing offset and width. Offset is in
 * the lower 8 bits and width in the next 8 bits. Width must be
 * between 1 and 64.
 * @param value Pointer to a buffer where the extracted bitfield value will be
 * stored. Must not be null and should be large enough to hold the
 * result.
 * @param value_size_bytes The size of the buffer in bytes. Must be between 1
 * and 8.
 * @return Returns an integer status code, API_CMS_ERROR_OK on success, or an
 * error code on failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_bf_get(adi_ad9081_device_t *device, uint32_t reg,
			      uint32_t info, uint8_t *value,
			      uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to modify a specific bitfield within a register
 * of the AD9081 device. It requires a valid device pointer and the
 * register address where the bitfield is located. The 'info' parameter
 * specifies the bitfield's offset and width, and the 'value' parameter
 * provides the new value to set. The function ensures that the width is
 * between 1 and 64 bits. It should be called when the device is properly
 * initialized and ready for configuration changes. The function returns
 * an error code if any parameter is invalid or if the operation fails.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * caller retains ownership.
 * @param reg The register address where the bitfield is located. Must be a
 * valid register address for the device.
 * @param info A 32-bit value where the lower 8 bits specify the bitfield offset
 * and the next 8 bits specify the bitfield width. Width must be
 * between 1 and 64.
 * @param value The new value to set in the bitfield. The value is masked and
 * shifted according to the bitfield's offset and width.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or an
 * error code on failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_bf_set(adi_ad9081_device_t *device, uint32_t reg,
			      uint32_t info, uint64_t value);

/***************************************************************************//**
 * @brief This function is used to read two separate bitfield values from a
 * specified register in the AD9081 device. It is useful when you need to
 * access multiple bitfields within the same register. The function
 * requires a valid device pointer and expects the caller to provide the
 * register address and bitfield information for both values. The
 * retrieved values are stored in the provided pointers. Ensure that the
 * device is properly initialized before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null and
 * should point to a valid, initialized device.
 * @param reg The register address from which the bitfield values are to be
 * read. Must be a valid register address within the device.
 * @param info0 Bitfield information for the first value to be retrieved.
 * Specifies the bitfield's position and size within the register.
 * @param value0 Pointer to a uint8_t where the first retrieved bitfield value
 * will be stored. Must not be null.
 * @param info1 Bitfield information for the second value to be retrieved.
 * Specifies the bitfield's position and size within the register.
 * @param value1 Pointer to a uint8_t where the second retrieved bitfield value
 * will be stored. Must not be null.
 * @param value_size_bytes The size in bytes of the values to be retrieved. Must
 * be a valid size that corresponds to the bitfield
 * sizes specified in info0 and info1.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation. A successful operation returns a specific success code,
 * while failure returns an error code.
 ******************************************************************************/
int32_t adi_ad9081_hal_2bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read three separate bitfield values from a
 * specified register of the AD9081 device. It is useful when multiple
 * bitfields need to be accessed simultaneously from the same register.
 * The function requires a valid device pointer and expects the caller to
 * provide the register address and bitfield information. The retrieved
 * values are stored in the provided pointers. Ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code indicating success or failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null and
 * should point to a valid, initialized device.
 * @param reg The register address from which the bitfield values are to be
 * read. Must be a valid register address within the device's address
 * space.
 * @param info0 Bitfield information for the first value to be retrieved.
 * Specifies the bitfield's position and size within the register.
 * @param value0 Pointer to a uint8_t where the first retrieved bitfield value
 * will be stored. Must not be null.
 * @param info1 Bitfield information for the second value to be retrieved.
 * Specifies the bitfield's position and size within the register.
 * @param value1 Pointer to a uint8_t where the second retrieved bitfield value
 * will be stored. Must not be null.
 * @param info2 Bitfield information for the third value to be retrieved.
 * Specifies the bitfield's position and size within the register.
 * @param value2 Pointer to a uint8_t where the third retrieved bitfield value
 * will be stored. Must not be null.
 * @param value_size_bytes The size in bytes of each bitfield value to be
 * retrieved. Must be a valid size that corresponds to
 * the bitfield sizes specified in info0, info1, and
 * info2.
 * @return Returns an int32_t error code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_3bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read four separate bitfield values from a
 * specified register within the device. It is useful when multiple
 * bitfields need to be accessed simultaneously from the same register.
 * The function requires a valid device pointer and expects the caller to
 * provide the register address and bitfield information for each of the
 * four bitfields. The size of each bitfield value to be retrieved is
 * specified by the caller. The function will populate the provided
 * pointers with the retrieved values. It is important to ensure that the
 * device is properly initialized before calling this function, and the
 * pointers provided for the values must be valid and non-null.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which the bitfield values are to be
 * retrieved.
 * @param info0 Bitfield information for the first value to be retrieved.
 * @param value0 Pointer to a uint8_t where the first bitfield value will be
 * stored. Must not be null.
 * @param info1 Bitfield information for the second value to be retrieved.
 * @param value1 Pointer to a uint8_t where the second bitfield value will be
 * stored. Must not be null.
 * @param info2 Bitfield information for the third value to be retrieved.
 * @param value2 Pointer to a uint8_t where the third bitfield value will be
 * stored. Must not be null.
 * @param info3 Bitfield information for the fourth value to be retrieved.
 * @param value3 Pointer to a uint8_t where the fourth bitfield value will be
 * stored. Must not be null.
 * @param value_size_bytes The size in bytes of each bitfield value to be
 * retrieved.
 * @return Returns an int32_t status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_4bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint32_t info3, uint8_t *value3,
			       uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read up to five bitfield values from a
 * specified register of the AD9081 device. It is useful when multiple
 * bitfields need to be accessed simultaneously from the same register.
 * The function requires a valid device pointer and register address,
 * along with information about each bitfield to be read. The caller must
 * provide pointers to store the retrieved values, and specify the size
 * of each value in bytes. The function returns an error code indicating
 * success or failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which bitfield values are to be read.
 * @param info0 Information about the first bitfield to be read.
 * @param value0 Pointer to store the value of the first bitfield. Must not be
 * null.
 * @param info1 Information about the second bitfield to be read.
 * @param value1 Pointer to store the value of the second bitfield. Must not be
 * null.
 * @param info2 Information about the third bitfield to be read.
 * @param value2 Pointer to store the value of the third bitfield. Must not be
 * null.
 * @param info3 Information about the fourth bitfield to be read.
 * @param value3 Pointer to store the value of the fourth bitfield. Must not be
 * null.
 * @param info4 Information about the fifth bitfield to be read.
 * @param value4 Pointer to store the value of the fifth bitfield. Must not be
 * null.
 * @param value_size_bytes The size in bytes of each bitfield value to be read.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_5bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint32_t info3, uint8_t *value3, uint32_t info4,
			       uint8_t *value4, uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read up to six bitfield values from a
 * specified register of the AD9081 device. It is useful when multiple
 * bitfields need to be accessed simultaneously. The function requires a
 * valid device pointer and a register address from which the bitfields
 * will be read. Each bitfield is specified by an info parameter, and the
 * corresponding value is stored in the provided value pointer. The size
 * of each value in bytes must be specified, and all value pointers must
 * be non-null. The function returns an error code indicating success or
 * failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which bitfield values are to be read.
 * @param info0 Specifies the first bitfield to be read from the register.
 * @param value0 Pointer to store the value of the first bitfield. Must not be
 * null.
 * @param info1 Specifies the second bitfield to be read from the register.
 * @param value1 Pointer to store the value of the second bitfield. Must not be
 * null.
 * @param info2 Specifies the third bitfield to be read from the register.
 * @param value2 Pointer to store the value of the third bitfield. Must not be
 * null.
 * @param info3 Specifies the fourth bitfield to be read from the register.
 * @param value3 Pointer to store the value of the fourth bitfield. Must not be
 * null.
 * @param info4 Specifies the fifth bitfield to be read from the register.
 * @param value4 Pointer to store the value of the fifth bitfield. Must not be
 * null.
 * @param info5 Specifies the sixth bitfield to be read from the register.
 * @param value5 Pointer to store the value of the sixth bitfield. Must not be
 * null.
 * @param value_size_bytes The size in bytes of each bitfield value to be read.
 * @return Returns an integer status code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_6bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint32_t info3, uint8_t *value3, uint32_t info4,
			       uint8_t *value4, uint32_t info5, uint8_t *value5,
			       uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read up to seven bitfield values from a
 * specified register of the AD9081 device. It is typically called when
 * multiple bitfield values need to be retrieved simultaneously from a
 * single register. The function requires a valid device pointer and
 * expects the caller to provide arrays of bitfield information and
 * corresponding pointers to store the retrieved values. The size of each
 * value in bytes must be specified. The function returns an error code
 * indicating success or failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which bitfield values are to be read.
 * @param info0 Bitfield information for the first value to be retrieved.
 * @param value0 Pointer to store the first retrieved bitfield value. Must not
 * be null.
 * @param info1 Bitfield information for the second value to be retrieved.
 * @param value1 Pointer to store the second retrieved bitfield value. Must not
 * be null.
 * @param info2 Bitfield information for the third value to be retrieved.
 * @param value2 Pointer to store the third retrieved bitfield value. Must not
 * be null.
 * @param info3 Bitfield information for the fourth value to be retrieved.
 * @param value3 Pointer to store the fourth retrieved bitfield value. Must not
 * be null.
 * @param info4 Bitfield information for the fifth value to be retrieved.
 * @param value4 Pointer to store the fifth retrieved bitfield value. Must not
 * be null.
 * @param info5 Bitfield information for the sixth value to be retrieved.
 * @param value5 Pointer to store the sixth retrieved bitfield value. Must not
 * be null.
 * @param info6 Bitfield information for the seventh value to be retrieved.
 * @param value6 Pointer to store the seventh retrieved bitfield value. Must not
 * be null.
 * @param value_size_bytes The size in bytes of each bitfield value to be
 * retrieved.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_7bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint32_t info3, uint8_t *value3, uint32_t info4,
			       uint8_t *value4, uint32_t info5, uint8_t *value5,
			       uint32_t info6, uint8_t *value6,
			       uint8_t value_size_bytes);
/***************************************************************************//**
 * @brief This function is used to read up to eight bitfield values from a
 * specified register of the AD9081 device. It requires a valid device
 * pointer and the register address from which the bitfields are to be
 * read. Each bitfield is specified by an info parameter, and the
 * corresponding value is stored in the provided value pointers. The size
 * of each value in bytes must be specified, and all value pointers must
 * be non-null. The function returns an error code indicating success or
 * failure of the operation.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null.
 * @param reg The register address from which the bitfields are to be read.
 * @param info0 Specifies the first bitfield to be read from the register.
 * @param value0 Pointer to a uint8_t where the first bitfield value will be
 * stored. Must not be null.
 * @param info1 Specifies the second bitfield to be read from the register.
 * @param value1 Pointer to a uint8_t where the second bitfield value will be
 * stored. Must not be null.
 * @param info2 Specifies the third bitfield to be read from the register.
 * @param value2 Pointer to a uint8_t where the third bitfield value will be
 * stored. Must not be null.
 * @param info3 Specifies the fourth bitfield to be read from the register.
 * @param value3 Pointer to a uint8_t where the fourth bitfield value will be
 * stored. Must not be null.
 * @param info4 Specifies the fifth bitfield to be read from the register.
 * @param value4 Pointer to a uint8_t where the fifth bitfield value will be
 * stored. Must not be null.
 * @param info5 Specifies the sixth bitfield to be read from the register.
 * @param value5 Pointer to a uint8_t where the sixth bitfield value will be
 * stored. Must not be null.
 * @param info6 Specifies the seventh bitfield to be read from the register.
 * @param value6 Pointer to a uint8_t where the seventh bitfield value will be
 * stored. Must not be null.
 * @param info7 Specifies the eighth bitfield to be read from the register.
 * @param value7 Pointer to a uint8_t where the eighth bitfield value will be
 * stored. Must not be null.
 * @param value_size_bytes The size in bytes of each bitfield value to be read.
 * @return Returns an int32_t error code indicating success or failure of the
 * operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_8bf_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint8_t *value0, uint32_t info1,
			       uint8_t *value1, uint32_t info2, uint8_t *value2,
			       uint32_t info3, uint8_t *value3, uint32_t info4,
			       uint8_t *value4, uint32_t info5, uint8_t *value5,
			       uint32_t info6, uint8_t *value6, uint32_t info7,
			       uint8_t *value7, uint8_t value_size_bytes);

/***************************************************************************//**
 * @brief This function is used to set two bitfield values within a specified
 * register of the AD9081 device. It is typically called when there is a
 * need to configure or modify specific settings in the device's register
 * map. The function requires a valid device handler and the register
 * address, along with the bitfield information and the values to be set.
 * It is important to ensure that the device is properly initialized
 * before calling this function. The function returns an error code
 * indicating success or failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null, and
 * the device should be properly initialized before use.
 * @param reg The register address where the bitfields are to be set. Must be a
 * valid register address within the device's register map.
 * @param info0 Information about the first bitfield, such as its position and
 * size within the register. Must be valid and correspond to a
 * defined bitfield in the register.
 * @param value0 The value to set for the first bitfield. Must fit within the
 * size constraints of the specified bitfield.
 * @param info1 Information about the second bitfield, similar to info0,
 * indicating its position and size within the register. Must be
 * valid and correspond to a defined bitfield in the register.
 * @param value1 The value to set for the second bitfield. Must fit within the
 * size constraints of the specified bitfield.
 * @return Returns an integer error code, where a non-negative value indicates
 * success and a negative value indicates failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_2bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1);
/***************************************************************************//**
 * @brief Use this function to set values for up to three bitfields within a
 * specified register of the AD9081 device. This function is typically
 * called when multiple bitfields need to be configured simultaneously.
 * The device must be properly initialized before calling this function.
 * Ensure that the provided device pointer is valid and that the register
 * and bitfield information are correctly specified. The function returns
 * an error code if the operation fails.
 *
 * @param device Pointer to the device handler structure. Must not be null, and
 * the device should be initialized before use.
 * @param reg The register address where the bitfields are to be set. Must be a
 * valid register address for the device.
 * @param info0 Specifies the first bitfield information, such as its position
 * and size within the register.
 * @param value0 The value to set for the first bitfield. Must fit within the
 * specified bitfield size.
 * @param info1 Specifies the second bitfield information, such as its position
 * and size within the register.
 * @param value1 The value to set for the second bitfield. Must fit within the
 * specified bitfield size.
 * @param info2 Specifies the third bitfield information, such as its position
 * and size within the register.
 * @param value2 The value to set for the third bitfield. Must fit within the
 * specified bitfield size.
 * @return Returns an integer status code: 0 for success or a negative error
 * code for failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_3bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2,
			       uint64_t value2);
/***************************************************************************//**
 * @brief This function is used to set four distinct bitfield values within a
 * specified register of the AD9081 device. It is typically called when
 * multiple bitfields need to be configured simultaneously within the
 * same register. The function requires a valid device handler and
 * specific information about each bitfield, including its position and
 * the value to be set. It is important to ensure that the device is
 * properly initialized before calling this function. The function
 * returns an error code if the operation fails, which can be used for
 * error handling.
 *
 * @param device Pointer to the device handler structure. Must not be null, and
 * the device should be initialized before use.
 * @param reg The register address where the bitfields will be set. Must be a
 * valid register address for the device.
 * @param info0 Information about the first bitfield, typically its position and
 * size within the register.
 * @param value0 The value to set for the first bitfield. Must fit within the
 * specified bitfield size.
 * @param info1 Information about the second bitfield, typically its position
 * and size within the register.
 * @param value1 The value to set for the second bitfield. Must fit within the
 * specified bitfield size.
 * @param info2 Information about the third bitfield, typically its position and
 * size within the register.
 * @param value2 The value to set for the third bitfield. Must fit within the
 * specified bitfield size.
 * @param info3 Information about the fourth bitfield, typically its position
 * and size within the register.
 * @param value3 The value to set for the fourth bitfield. Must fit within the
 * specified bitfield size.
 * @return Returns an integer status code: 0 for success or a negative error
 * code for failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_4bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2, uint64_t value2,
			       uint32_t info3, uint64_t value3);
/***************************************************************************//**
 * @brief This function is used to set up to five bitfields within a specified
 * register of the AD9081 device. It is typically called when multiple
 * configuration parameters need to be updated simultaneously in a single
 * register. The function requires a valid device pointer and the
 * register address, along with pairs of bitfield information and their
 * corresponding values. It is important to ensure that the device has
 * been properly initialized before calling this function. The function
 * returns an error code if the operation fails, which can be used for
 * error handling.
 *
 * @param device Pointer to the device handler structure. Must not be null and
 * should point to a valid, initialized device.
 * @param reg The register address where the bitfields will be set. Must be a
 * valid register address for the device.
 * @param info0 Information about the first bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value0 The value to set for the first bitfield. Must be within the
 * valid range for the specified bitfield.
 * @param info1 Information about the second bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value1 The value to set for the second bitfield. Must be within the
 * valid range for the specified bitfield.
 * @param info2 Information about the third bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value2 The value to set for the third bitfield. Must be within the
 * valid range for the specified bitfield.
 * @param info3 Information about the fourth bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value3 The value to set for the fourth bitfield. Must be within the
 * valid range for the specified bitfield.
 * @param info4 Information about the fifth bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value4 The value to set for the fifth bitfield. Must be within the
 * valid range for the specified bitfield.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code indicating the type of failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_5bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2, uint64_t value2,
			       uint32_t info3, uint64_t value3, uint32_t info4,
			       uint64_t value4);
/***************************************************************************//**
 * @brief This function is used to set up to six bitfields within a specified
 * register of the AD9081 device. It is typically called when multiple
 * configuration parameters need to be updated simultaneously. The
 * function requires a valid device pointer and a register address, along
 * with pairs of bitfield information and their corresponding values. It
 * is important to ensure that the device has been properly initialized
 * before calling this function. The function returns an error code if
 * the operation fails, which can be used for error handling.
 *
 * @param device Pointer to the device handler structure. Must not be null, and
 * the device should be initialized before use.
 * @param reg The register address where the bitfields will be set. Must be a
 * valid register address for the device.
 * @param info0 Bitfield information for the first bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value0 The value to set for the first bitfield. Must fit within the
 * size specified by info0.
 * @param info1 Bitfield information for the second bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value1 The value to set for the second bitfield. Must fit within the
 * size specified by info1.
 * @param info2 Bitfield information for the third bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value2 The value to set for the third bitfield. Must fit within the
 * size specified by info2.
 * @param info3 Bitfield information for the fourth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value3 The value to set for the fourth bitfield. Must fit within the
 * size specified by info3.
 * @param info4 Bitfield information for the fifth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value4 The value to set for the fifth bitfield. Must fit within the
 * size specified by info4.
 * @param info5 Bitfield information for the sixth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value5 The value to set for the sixth bitfield. Must fit within the
 * size specified by info5.
 * @return Returns an integer status code indicating success or failure of the
 * operation. A non-zero value indicates an error.
 ******************************************************************************/
int32_t adi_ad9081_hal_6bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2, uint64_t value2,
			       uint32_t info3, uint64_t value3, uint32_t info4,
			       uint64_t value4, uint32_t info5,
			       uint64_t value5);
/***************************************************************************//**
 * @brief This function is used to set up to seven bitfields within a specified
 * register of the AD9081 device. It is typically called when multiple
 * bitfields need to be configured simultaneously. The function requires
 * a valid device pointer and a register address, along with pairs of
 * bitfield information and their corresponding values. Each bitfield is
 * defined by an info-value pair, and the function will attempt to set
 * all provided bitfields. The caller must ensure that the device is
 * properly initialized before calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null, and
 * the device should be initialized before use.
 * @param reg The register address where the bitfields will be set. Must be a
 * valid register address for the device.
 * @param info0 Information about the first bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value0 The value to set for the first bitfield. Must be within the
 * valid range for the bitfield.
 * @param info1 Information about the second bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value1 The value to set for the second bitfield. Must be within the
 * valid range for the bitfield.
 * @param info2 Information about the third bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value2 The value to set for the third bitfield. Must be within the
 * valid range for the bitfield.
 * @param info3 Information about the fourth bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value3 The value to set for the fourth bitfield. Must be within the
 * valid range for the bitfield.
 * @param info4 Information about the fifth bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value4 The value to set for the fifth bitfield. Must be within the
 * valid range for the bitfield.
 * @param info5 Information about the sixth bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value5 The value to set for the sixth bitfield. Must be within the
 * valid range for the bitfield.
 * @param info6 Information about the seventh bitfield to set. Must be a valid
 * bitfield descriptor.
 * @param value6 The value to set for the seventh bitfield. Must be within the
 * valid range for the bitfield.
 * @return Returns an integer status code. A value of 0 indicates success, while
 * a non-zero value indicates an error occurred during the operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_7bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2, uint64_t value2,
			       uint32_t info3, uint64_t value3, uint32_t info4,
			       uint64_t value4, uint32_t info5, uint64_t value5,
			       uint32_t info6, uint64_t value6);
/***************************************************************************//**
 * @brief This function is used to set up to eight bitfields within a specified
 * register of the AD9081 device. It is typically called when multiple
 * configuration settings need to be applied simultaneously to a device
 * register. The function requires a valid device pointer and register
 * address, along with pairs of bitfield information and their
 * corresponding values. Each bitfield is defined by an info-value pair,
 * and the function will apply these settings to the device. The caller
 * must ensure that the device is properly initialized before calling
 * this function. The function returns an error code indicating success
 * or failure of the operation.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * caller retains ownership.
 * @param reg The register address where the bitfields will be set. Must be a
 * valid register address for the device.
 * @param info0 Bitfield information for the first bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value0 Value to set for the first bitfield. Must fit within the size
 * specified by info0.
 * @param info1 Bitfield information for the second bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value1 Value to set for the second bitfield. Must fit within the size
 * specified by info1.
 * @param info2 Bitfield information for the third bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value2 Value to set for the third bitfield. Must fit within the size
 * specified by info2.
 * @param info3 Bitfield information for the fourth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value3 Value to set for the fourth bitfield. Must fit within the size
 * specified by info3.
 * @param info4 Bitfield information for the fifth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value4 Value to set for the fifth bitfield. Must fit within the size
 * specified by info4.
 * @param info5 Bitfield information for the sixth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value5 Value to set for the sixth bitfield. Must fit within the size
 * specified by info5.
 * @param info6 Bitfield information for the seventh bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value6 Value to set for the seventh bitfield. Must fit within the size
 * specified by info6.
 * @param info7 Bitfield information for the eighth bitfield. Specifies the
 * position and size of the bitfield within the register.
 * @param value7 Value to set for the eighth bitfield. Must fit within the size
 * specified by info7.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_8bf_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t info0, uint64_t value0, uint32_t info1,
			       uint64_t value1, uint32_t info2, uint64_t value2,
			       uint32_t info3, uint64_t value3, uint32_t info4,
			       uint64_t value4, uint32_t info5, uint64_t value5,
			       uint32_t info6, uint64_t value6, uint32_t info7,
			       uint64_t value7);

/***************************************************************************//**
 * @brief This function is used to extract multiple bit-fields from a single
 * register of the AD9081 device. It is particularly useful when dealing
 * with registers that contain multiple fields of interest. The function
 * requires valid pointers for the device, info, and value parameters,
 * and the register address must be less than 0x4000. If only one bit-
 * field is specified, a standard non-multi bit-field retrieval method is
 * used. The function handles cases where bit-fields do not cross
 * register boundaries, and it will fall back to a standard method if
 * cross-register access is required, which may incur additional SPI
 * reads.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address from which bit-fields are to be retrieved.
 * Must be less than 0x4000.
 * @param info Pointer to an array of bit-field information, where each entry
 * specifies the offset and width of a bit-field. Must not be null.
 * @param value Pointer to an array of pointers where the retrieved bit-field
 * values will be stored. Must not be null.
 * @param value_size_bytes The size in bytes of each value to be retrieved. Must
 * be consistent with the expected size of the bit-
 * fields.
 * @param num_bfs The number of bit-fields to retrieve. If set to 1, a standard
 * non-multi bit-field retrieval is performed.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or an
 * error code on failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_multi_bf_get(adi_ad9081_device_t *device, uint32_t reg,
				    uint32_t *info, uint8_t **value,
				    uint8_t value_size_bytes, uint8_t num_bfs);
/***************************************************************************//**
 * @brief This function is used to set multiple bit fields within a specified
 * register of the AD9081 device. It is particularly useful when multiple
 * bit fields need to be configured simultaneously. The function requires
 * valid pointers for the device, info, and value parameters, and the
 * register address must be less than 0x4000. If only one bit field is
 * specified, a standard non-multi bit field set operation is performed.
 * The function handles cases where bit fields do not cross register
 * boundaries, and it returns an error code if any operation fails.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param reg The register address where the bit fields are to be set. Must be
 * less than 0x4000.
 * @param info Pointer to an array of bit field information, where each entry
 * specifies the offset and width of a bit field. Must not be null.
 * @param value Pointer to an array of values to be set for each bit field
 * specified in the info array. Must not be null.
 * @param num_bfs The number of bit fields to set. If set to 1, a standard non-
 * multi bit field set operation is used.
 * @return Returns an integer status code: API_CMS_ERROR_OK on success, or an
 * error code on failure.
 ******************************************************************************/
int32_t adi_ad9081_hal_multi_bf_set(adi_ad9081_device_t *device, uint32_t reg,
				    uint32_t *info, uint64_t *value,
				    uint8_t num_bfs);

/***************************************************************************//**
 * @brief This function is used to read a value from a specified register of the
 * AD9081 device. It requires a valid device handler and a register
 * address to be specified. The function will store the retrieved
 * register value in the provided data pointer. It is essential that the
 * device has been properly initialized before calling this function. The
 * function handles both standard and extended register spaces, and it
 * will return an error code if the SPI transfer or logging operations
 * fail. Ensure that the data pointer is not null and that the device's
 * SPI transfer function is correctly set up.
 *
 * @param device Pointer to the device handler structure. Must not be null and
 * should be properly initialized with a valid SPI transfer
 * function.
 * @param reg The register address from which to read. It can be a standard or
 * extended address, and the function will handle both appropriately.
 * @param data Pointer to a uint8_t where the read data will be stored. Must not
 * be null, and the caller is responsible for ensuring it points to
 * a valid memory location.
 * @return Returns an int32_t error code. API_CMS_ERROR_OK is returned on
 * success, while specific error codes are returned for SPI transfer or
 * logging failures.
 ******************************************************************************/
int32_t adi_ad9081_hal_reg_get(adi_ad9081_device_t *device, uint32_t reg,
			       uint8_t *data);
/***************************************************************************//**
 * @brief This function writes a specified value to a register on the AD9081
 * device using SPI communication. It should be called when a register
 * needs to be updated with a new value. The function requires a valid
 * device pointer and a register address. It handles both standard and
 * extended register spaces, ensuring the correct data format is used for
 * each. The function returns an error code if the SPI transfer or
 * logging fails, ensuring that the caller can handle such errors
 * appropriately.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address to be written to. Must be a valid register
 * address for the device.
 * @param data The data to write to the specified register. The format of the
 * data depends on the register being accessed.
 * @return Returns an int32_t error code: API_CMS_ERROR_OK on success, or an
 * error code indicating the type of failure (e.g.,
 * API_CMS_ERROR_SPI_XFER or API_CMS_ERROR_LOG_WRITE).
 ******************************************************************************/
int32_t adi_ad9081_hal_reg_set(adi_ad9081_device_t *device, uint32_t reg,
			       uint32_t data);

/***************************************************************************//**
 * @brief This function is used to read a register value from a specific lane of
 * the JRX CBUS on the AD9081 device. It requires a valid device pointer
 * and a non-null data pointer to store the retrieved value. The function
 * must be called with a valid lane number, and it will return an error
 * code if any operation fails, such as SPI transfer or delay issues. It
 * is essential to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * caller retains ownership.
 * @param reg The register address to be read. Must be a valid register address
 * within the device's address space.
 * @param data Pointer to a uint8_t variable where the read data will be stored.
 * Must not be null. The function writes the retrieved register
 * value to this location.
 * @param lane The lane number from which to read the register. Must be a valid
 * lane number supported by the device.
 * @return Returns an int32_t error code. API_CMS_ERROR_OK indicates success,
 * while other codes indicate specific errors such as SPI transfer
 * failures or delay issues.
 ******************************************************************************/
int32_t adi_ad9081_hal_cbusjrx_reg_get(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t *data,
				       uint8_t lane);
/***************************************************************************//**
 * @brief This function is used to set a register value on a specified lane of
 * the JRX CBUS for the AD9081 device. It should be called when a
 * specific configuration or data needs to be written to a register
 * associated with a particular lane. The function requires a valid
 * device pointer and will return an error code if the device pointer is
 * null or if any of the underlying register operations fail. It is
 * important to ensure that the device is properly initialized before
 * calling this function.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address to be set. Must be a valid register address
 * for the JRX CBUS.
 * @param data The data byte to be written to the specified register.
 * @param lane The lane number on which the register is to be set. Must be a
 * valid lane number for the device.
 * @return Returns an int32_t error code. API_CMS_ERROR_OK is returned on
 * success, while specific error codes are returned for null device
 * pointers or failed register operations.
 ******************************************************************************/
int32_t adi_ad9081_hal_cbusjrx_reg_set(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t data,
				       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to read a register value from a specific lane of
 * the JTX interface on the AD9081 device. It requires a valid device
 * pointer and a non-null data pointer to store the retrieved value. The
 * function must be called with a valid lane number, and it performs
 * several register operations to access the desired register value. It
 * is important to ensure that the device is properly initialized before
 * calling this function. The function returns an error code if any
 * operation fails, allowing the caller to handle errors appropriately.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address to be accessed. Must be a valid register
 * address within the device's address space.
 * @param data Pointer to a uint8_t variable where the retrieved register value
 * will be stored. Must not be null. The caller retains ownership.
 * @param lane The lane number from which to retrieve the register value. Must
 * be a valid lane number for the device.
 * @return Returns an int32_t error code. API_CMS_ERROR_OK indicates success,
 * while other codes indicate specific errors encountered during the
 * operation.
 ******************************************************************************/
int32_t adi_ad9081_hal_cbusjtx_reg_get(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t *data,
				       uint8_t lane);
/***************************************************************************//**
 * @brief This function sets a specified register on the JTX interface of the
 * AD9081 device to a given value. It should be used when configuring the
 * JTX interface registers. The function requires a valid device pointer
 * and will return an error if the device pointer is null. It performs
 * several register writes and includes delays to ensure proper timing.
 * The function must be called with a properly initialized device
 * structure.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address to be set on the JTX interface. Must be a
 * valid register address for the device.
 * @param data The data to be written to the specified register. Must be a valid
 * 8-bit value.
 * @param lane The lane number to be configured. Must be a valid lane number for
 * the device.
 * @return Returns an int32_t indicating success or an error code.
 * API_CMS_ERROR_OK is returned on success, while specific error codes
 * are returned for failures in SPI transfer or delay operations.
 ******************************************************************************/
int32_t adi_ad9081_hal_cbusjtx_reg_set(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t data,
				       uint8_t lane);

/***************************************************************************//**
 * @brief This function is used to read a specific register value from the CBUS
 * PLL of the AD9081 device. It requires a valid device context and a
 * register address to operate. The function will store the retrieved
 * register value in the provided data pointer. It is essential to ensure
 * that both the device and data pointers are not null before calling
 * this function. The function will return an error code if any operation
 * fails, such as SPI transfer errors or delay issues.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device context. Must not be null.
 * @param reg The register address from which the value is to be read. Must be a
 * valid register address within the device's addressable range.
 * @param data Pointer to a uint8_t variable where the read register value will
 * be stored. Must not be null.
 * @return Returns an int32_t error code. API_CMS_ERROR_OK indicates success,
 * while other codes indicate specific errors such as SPI transfer
 * failures or delay issues.
 ******************************************************************************/
int32_t adi_ad9081_hal_cbuspll_reg_get(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t *data);
/***************************************************************************//**
 * @brief This function is used to set a specific register in the CBUS PLL of
 * the AD9081 device. It requires a valid device pointer and the register
 * address and data to be written. The function performs a series of
 * register writes and a delay to ensure the operation is completed
 * successfully. It should be called when the user needs to configure or
 * modify the CBUS PLL settings. The function returns an error code if
 * any of the operations fail, indicating the type of failure
 * encountered.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param reg The register address within the CBUS PLL to be set. Must be a
 * valid register address for the device.
 * @param data The data byte to be written to the specified register. Must be a
 * valid data value for the register.
 * @return Returns an int32_t error code: API_CMS_ERROR_OK on success, or an
 * error code indicating the type of failure (e.g.,
 * API_CMS_ERROR_SPI_XFER, API_CMS_ERROR_DELAY_US).
 ******************************************************************************/
int32_t adi_ad9081_hal_cbuspll_reg_set(adi_ad9081_device_t *device,
				       uint32_t reg, uint8_t data);

/***************************************************************************//**
 * @brief This function is used to wait until a specific bitfield in a register
 * clears to zero. It is useful in scenarios where the software needs to
 * ensure that a hardware condition has been reset before proceeding. The
 * function will poll the bitfield up to 200 times, with a delay of 20
 * microseconds between each poll. If the bitfield does not clear within
 * this time, an error code is returned. This function should be called
 * when it is necessary to wait for a hardware condition to reset, and it
 * assumes that the device has been properly initialized.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * function will return an error if this parameter is null.
 * @param reg The register address from which the bitfield is to be read. Must
 * be a valid register address for the device.
 * @param info Information about the bitfield to be checked, typically including
 * the bit position and length. Must correspond to a valid bitfield
 * within the specified register.
 * @return Returns API_CMS_ERROR_OK if the bitfield clears successfully. Returns
 * API_CMS_ERROR_ERROR if the bitfield does not clear within the allowed
 * attempts.
 ******************************************************************************/
int32_t adi_ad9081_hal_bf_wait_to_clear(adi_ad9081_device_t *device,
					uint32_t reg, uint32_t info);
/***************************************************************************//**
 * @brief This function is used to poll a specific bit field within a register
 * until it is set to 1, indicating a certain condition or status has
 * been met. It is useful in scenarios where the software needs to wait
 * for hardware to reach a specific state before proceeding. The function
 * will attempt to read the bit field up to 200 times, with a delay of 20
 * microseconds between each attempt. If the bit field is not set after
 * 200 attempts, the function returns an error code. This function should
 * be called when it is necessary to ensure that a hardware condition is
 * met before continuing with further operations.
 *
 * @param device Pointer to the device handler structure. Must not be null. The
 * function will return an error if this parameter is null.
 * @param reg The register address from which the bit field is to be read. Must
 * be a valid register address for the device.
 * @param info Information about the bit field to be checked, typically
 * including the bit position and size. Must correspond to a valid
 * bit field within the specified register.
 * @return Returns API_CMS_ERROR_OK if the bit field is set within the allowed
 * attempts, otherwise returns API_CMS_ERROR_ERROR if the bit field is
 * not set after 200 attempts.
 ******************************************************************************/
int32_t adi_ad9081_hal_bf_wait_to_set(adi_ad9081_device_t *device, uint32_t reg,
				      uint32_t info);

/***************************************************************************//**
 * @brief This function is used to report an error by logging detailed context
 * information such as the file name, function name, line number, and a
 * custom comment. It is useful for debugging and tracking errors in the
 * system. The function must be called with a valid device pointer, and
 * it will return an error code if the logging operation fails or if the
 * device pointer is null. This function is typically used in conjunction
 * with other hardware abstraction layer (HAL) functions to provide
 * comprehensive error reporting.
 *
 * @param device Pointer to the device handler structure. Must not be null. If
 * null, the function returns API_CMS_ERROR_NULL_PARAM.
 * @param log_type Specifies the type of log entry to create. Must be a valid
 * adi_cms_log_type_e value.
 * @param error The error code to report. This is an integer value representing
 * the specific error.
 * @param file_name String representing the name of the source file where the
 * error occurred. Must be a valid, null-terminated string.
 * @param func_name String representing the name of the function where the error
 * occurred. Must be a valid, null-terminated string.
 * @param line_num The line number in the source file where the error occurred.
 * Must be a valid unsigned integer.
 * @param var_name String representing the name of the variable related to the
 * error. Must be a valid, null-terminated string.
 * @param comment Additional comment or message to include in the log. Must be a
 * valid, null-terminated string.
 * @return Returns API_CMS_ERROR_OK on success, API_CMS_ERROR_NULL_PARAM if the
 * device is null, or API_CMS_ERROR_LOG_WRITE if logging fails.
 ******************************************************************************/
int32_t adi_ad9081_hal_error_report(adi_ad9081_device_t *device,
				    adi_cms_log_type_e log_type, int32_t error,
				    const char *file_name,
				    const char *func_name, uint32_t line_num,
				    const char *var_name, const char *comment);

/***************************************************************************//**
 * @brief This function calculates the frequency tuning word (FTW) and
 * additional parameters 'a' and 'b' for a numerically controlled
 * oscillator (NCO) based on the provided frequency and NCO shift. It is
 * essential to call this function with a valid device pointer and a non-
 * zero frequency. The function is typically used in digital signal
 * processing applications where precise frequency control is required.
 * The calculated FTW and parameters 'a' and 'b' are returned through the
 * provided pointers, which must not be null.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param freq The frequency for which the FTW is calculated. Must be non-zero.
 * @param nco_shift The shift value for the NCO, which can be positive or
 * negative.
 * @param ftw Pointer to a uint64_t where the calculated frequency tuning word
 * will be stored. Must not be null.
 * @param a Pointer to a uint64_t where the calculated parameter 'a' will be
 * stored. Must not be null.
 * @param b Pointer to a uint64_t where the calculated parameter 'b' will be
 * stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK on success, indicating successful
 * calculation. Otherwise, returns an error code if the input parameters
 * are invalid.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_nco_ftw(adi_ad9081_device_t *device, uint64_t freq,
				    int64_t nco_shift, uint64_t *ftw,
				    uint64_t *a, uint64_t *b);
#if AD9081_USE_FLOATING_TYPE > 0
/***************************************************************************//**
 * @brief This function calculates the frequency tuning word (FTW) and its
 * fractional components for a numerically controlled oscillator (NCO)
 * based on the specified frequency and NCO shift. It is used when
 * precise frequency control is required in digital signal processing
 * applications. The function requires a valid device pointer and non-
 * zero frequency input. It computes the FTW and fractional components
 * 'a' and 'b', which are used to achieve the desired NCO shift. The
 * function must be called with valid pointers for all output parameters,
 * and it returns a success code upon successful computation.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param freq The frequency in Hz for which the NCO FTW is to be calculated.
 * Must be non-zero.
 * @param nco_shift The desired NCO shift in Hz. Can be positive or negative.
 * @param ftw Pointer to a uint64_t where the calculated frequency tuning word
 * will be stored. Must not be null.
 * @param a Pointer to a uint64_t where the fractional component 'a' will be
 * stored. Must not be null.
 * @param b Pointer to a uint64_t where the fractional component 'b' will be
 * stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK on success, indicating successful
 * calculation of the FTW and fractional components.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_nco_ftw_f(adi_ad9081_device_t *device, double freq,
				      double nco_shift, uint64_t *ftw,
				      uint64_t *a, uint64_t *b);
#endif
/***************************************************************************//**
 * @brief This function calculates the frequency tuning word (FTW) for a receive
 * (RX) numerically controlled oscillator (NCO) based on the provided ADC
 * frequency and NCO frequency shift. It is essential to call this
 * function with a valid device pointer and a non-zero ADC frequency. The
 * function computes the FTW and stores it in the location pointed to by
 * the ftw parameter. It handles both positive and negative NCO shifts,
 * adjusting the FTW accordingly. This function should be used when
 * configuring the RX NCO for frequency tuning in applications involving
 * digital signal processing.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null.
 * @param adc_freq The frequency of the ADC in hertz. Must be greater than zero.
 * @param nco_shift The desired frequency shift for the NCO in hertz. Can be
 * positive or negative.
 * @param ftw Pointer to a uint64_t where the calculated frequency tuning word
 * will be stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK on success. The calculated FTW is stored in
 * the location pointed to by ftw.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_rx_nco_ftw(adi_ad9081_device_t *device,
				       uint64_t adc_freq, int64_t nco_shift,
				       uint64_t *ftw);
/***************************************************************************//**
 * @brief This function calculates the frequency tuning word (FTW) for a
 * transmit (TX) numerically controlled oscillator (NCO) based on the
 * given DAC frequency and NCO frequency shift. It is essential to call
 * this function with a valid device pointer and a non-zero DAC
 * frequency. The calculated FTW is stored in the location pointed to by
 * the ftw parameter. This function should be used when configuring the
 * frequency of a TX NCO in a device, ensuring that the device is
 * properly initialized before calling.
 *
 * @param device Pointer to the device handler structure. Must not be null.
 * @param dac_freq The frequency of the DAC in Hz. Must be greater than zero.
 * @param nco_shift The desired frequency shift for the NCO in Hz. Can be
 * positive or negative.
 * @param ftw Pointer to a uint64_t where the calculated frequency tuning word
 * will be stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK on success. The calculated FTW is written to
 * the location pointed to by ftw.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_tx_nco_ftw(adi_ad9081_device_t *device,
				       uint64_t dac_freq, int64_t nco_shift,
				       uint64_t *ftw);
/***************************************************************************//**
 * @brief This function calculates a 32-bit frequency tuning word (FTW) for a
 * receive (RX) numerically controlled oscillator (NCO) based on the
 * specified ADC frequency and NCO frequency shift. It is essential to
 * ensure that the device pointer is valid and that the ADC frequency is
 * non-zero before calling this function. The calculated FTW is stored in
 * the location pointed to by the ftw parameter. This function is
 * typically used in applications where precise frequency control of the
 * RX path is required.
 *
 * @param device Pointer to the adi_ad9081_device_t structure representing the
 * device. Must not be null.
 * @param adc_freq The frequency of the ADC in hertz. Must be greater than zero.
 * @param nco_shift The desired frequency shift for the NCO in hertz. Can be
 * positive or negative.
 * @param ftw Pointer to a uint64_t where the calculated 32-bit FTW will be
 * stored. Must not be null.
 * @return Returns API_CMS_ERROR_OK on success. The calculated FTW is written to
 * the location pointed to by ftw.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_rx_nco_ftw32(adi_ad9081_device_t *device,
					 uint64_t adc_freq, int64_t nco_shift,
					 uint64_t *ftw);
/***************************************************************************//**
 * @brief This function computes a 32-bit frequency tuning word (FTW) for a
 * transmit numerically controlled oscillator (NCO) based on the given
 * DAC frequency and NCO shift. It is essential to ensure that the
 * `device` pointer is valid and that `dac_freq` is non-zero before
 * calling this function. The calculated FTW is stored in the location
 * pointed to by `ftw`. This function should be used when configuring the
 * frequency of a TX NCO in a system using the AD9081 device.
 *
 * @param device Pointer to an `adi_ad9081_device_t` structure representing the
 * device. Must not be null.
 * @param dac_freq The frequency of the DAC in Hz. Must be greater than zero.
 * @param nco_shift The desired frequency shift for the NCO in Hz. Can be
 * positive or negative.
 * @param ftw Pointer to a `uint64_t` where the calculated 32-bit FTW will be
 * stored. Must not be null.
 * @return Returns `API_CMS_ERROR_OK` on success. The calculated FTW is written
 * to the location pointed to by `ftw`.
 ******************************************************************************/
int32_t adi_ad9081_hal_calc_tx_nco_ftw32(adi_ad9081_device_t *device,
					 uint64_t dac_freq, int64_t nco_shift,
					 uint64_t *ftw);

#ifdef __cplusplus
}
#endif

#endif /* __AD9081_REG_H__ */

/*! @} */