/***************************************************************************//**
 *   @file   ltc4306.h
 *   @brief  Header file of ltc4306 driver.
 *   @author Jose Ramon San Buenaventura (jose.sanbuenaventura@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __LTC4306_H__
#define __LTC4306_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "no_os_i2c.h"
#include "no_os_util.h"

/******************************************************************************/
/************************** LTC4306 Definitions *******************************/
/******************************************************************************/

/* Register address definitions */
#define LTC4306_CTRL_REG0				0x00
#define LTC4306_CTRL_REG1				0x01
#define LTC4306_CTRL_REG2           	0x02
#define LTC4306_CTRL_REG3            	0x03
#define LTC4306_OUT_OF_BOUNDS  	        0x04

/* Special Addresses */
#define LTC4306_MASS_WRITE_ADDR         0xBA
#define LTC4306_ALERT_RESPONSE_ADDR     0x19

/* Masks (Bits and Fields) */
/* REG0 */
#define LTC4306_DOWNSTREAM_CONNECT		NO_OS_BIT(7)
#define LTC4306_ALERT_LOGIC(x)			NO_OS_BIT(7 - (x))
#define LTC4306_FAILED_CONN				NO_OS_BIT(2)
#define LTC4306_LATCHED_TOUT			NO_OS_BIT(1)
#define LTC4306_REALTIME_TOUT			NO_OS_BIT(0)

/* REG1 */
#define LTC4306_UPSTREAM_EN				NO_OS_BIT(7)
#define LTC4306_DOWNSTREAM_EN			NO_OS_BIT(6)
#define LTC4306_OUT_DRV_STATE(x)		NO_OS_BIT(6 - (x))
#define LTC4306_GPIO_LOGIC(x)			NO_OS_BIT(2 - (x))

/* REG2 */
#define LTC4306_GPIO_MODE_MASK			NO_OS_GENMASK(7, 6)
#define LTC4306_GPIO_MODE_CONFIG(x)		NO_OS_BIT(8 - (x))
#define LTC4306_CONN_REQ				NO_OS_BIT(5)
#define LTC4306_OUT_MODE_MASK			NO_OS_GENMASK(4, 3)
#define LTC4306_OUT_MODE_CONFIG(x)		NO_OS_BIT(5 - (x))
#define LTC4306_MASS_WRITE				NO_OS_BIT(2)
#define LTC4306_TOUT					NO_OS_GENMASK(1, 0)

/* REG3 */
#define LTC4306_FET_STATE_MASK			NO_OS_GENMASK(7, 4)
#define LTC4306_FET_STATE(x)			NO_OS_BIT(8 - (x))
#define LTC4306_BUS_LOGIC_STATE_MASK	NO_OS_GENMASK(3, 0)
#define LTC4306_LOGIC_STATE(x)			NO_OS_BIT(4 - (x))

/* Other definitions */
#define LTC4306_MIN_CHANNEL_INDEX		1
#define LTC4306_CHANNEL_TWO				2
#define LTC4306_CHANNEL_THREE			3
#define LTC4306_MAX_CHANNEL_INDEX		4
#define LTC4306_GPIO_MIN				1
#define LTC4306_GPIO_MAX				2

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `ltc4306_addr_conn` enumeration defines the possible logic level
 * connections for the LTC4306 device's address pins. It provides three
 * states: `LTC4306_LOW` for a low logic level, `LTC4306_HIGH` for a high
 * logic level, and `LTC4306_NO_CONN` for no connection. This enumeration
 * is used to configure and manage the logic levels of the address pins
 * in the LTC4306 I2C bus buffer and multiplexer.
 *
 * @param LTC4306_LOW Represents a low logic level connection.
 * @param LTC4306_HIGH Represents a high logic level connection.
 * @param LTC4306_NO_CONN Indicates no connection is present.
 ******************************************************************************/
enum ltc4306_addr_conn {
	LTC4306_LOW,
	LTC4306_HIGH,
	LTC4306_NO_CONN,
};

/***************************************************************************//**
 * @brief The `ltc4306_timeout_mode` enumeration defines the possible timeout
 * modes for the LTC4306 device, which is used to manage the timeout
 * settings for the device's stuck low timeout circuitry. This
 * enumeration provides four distinct modes: disabled, 30 milliseconds,
 * 15 milliseconds, and 7.5 milliseconds, allowing for flexible
 * configuration of the timeout behavior based on application
 * requirements.
 *
 * @param LTC4306_DISABLED Represents the disabled state for the timeout mode.
 * @param LTC4306_30MS Represents a timeout mode with a duration of 30
 * milliseconds.
 * @param LTC4306_15MS Represents a timeout mode with a duration of 15
 * milliseconds.
 * @param LTC4306_7P5MS Represents a timeout mode with a duration of 7.5
 * milliseconds.
 ******************************************************************************/
enum ltc4306_timeout_mode {
	LTC4306_DISABLED,
	LTC4306_30MS,
	LTC4306_15MS,
	LTC4306_7P5MS,
};

/***************************************************************************//**
 * @brief The `ltc4306_dev` structure is designed to represent a device using
 * the LTC4306 I2C bus buffer and multiplexer. It includes a pointer to
 * an I2C descriptor for communication, arrays to track the configuration
 * and status of GPIOs and FETs, and booleans to indicate the status of
 * upstream and downstream accelerators. This structure is essential for
 * managing the configuration and operation of the LTC4306 device,
 * allowing for control over GPIO modes, FET connections, and bus
 * acceleration features.
 *
 * @param i2c_desc Pointer to an I2C descriptor for communication.
 * @param is_input Array indicating whether each GPIO is configured as input.
 * @param is_push_pull Array indicating whether each GPIO is configured in push-
 * pull mode.
 * @param is_closed Array indicating the connection status of FETs for each
 * channel.
 * @param upstream Boolean indicating the status of the upstream accelerator.
 * @param downstream Boolean indicating the status of the downstream
 * accelerator.
 ******************************************************************************/
struct ltc4306_dev {
	struct no_os_i2c_desc	*i2c_desc;
	/* GPIO status indicators (input or ouptut) */
	bool is_input[LTC4306_GPIO_MAX];
	/* GPIO out mode indicators (push pull or open drain */
	bool is_push_pull[LTC4306_GPIO_MAX];
	/* FET status indicators (connected or not) */
	bool is_closed[LTC4306_MAX_CHANNEL_INDEX];
	/* Upstream accelerator indicator */
	bool upstream;
	/* Downstream accelerator indicator */
	bool downstream;
};

/***************************************************************************//**
 * @brief The `ltc4306_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the LTC4306 device,
 * specifically focusing on the I2C communication interface. It contains
 * a single member, `i2c_init`, which is a structure that provides the
 * necessary configuration details for initializing the I2C bus, ensuring
 * proper communication between the host and the LTC4306 device.
 *
 * @param i2c_init This member is a structure of type `no_os_i2c_init_param`
 * that holds the initialization parameters for the I2C
 * interface.
 ******************************************************************************/
struct ltc4306_init_param {
	/* I2C */
	struct no_os_i2c_init_param	i2c_init;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Generate slave address depending on A0 and A1 logic levels */
/***************************************************************************//**
 * @brief This function calculates and sets the I2C slave address in the
 * provided initialization parameter structure based on the logic levels
 * of three address connection pins. It should be used when configuring
 * the LTC4306 device to ensure the correct I2C address is set before
 * communication. The function requires valid logic level inputs for each
 * address pin, and it will return an error if any input is outside the
 * expected range.
 *
 * @param init_param A pointer to an ltc4306_init_param structure where the
 * calculated I2C slave address will be stored. Must not be
 * null.
 * @param addr0 An enum value of type ltc4306_addr_conn representing the logic
 * level of the first address pin. Must be one of LTC4306_LOW,
 * LTC4306_HIGH, or LTC4306_NO_CONN.
 * @param addr1 An enum value of type ltc4306_addr_conn representing the logic
 * level of the second address pin. Must be one of LTC4306_LOW,
 * LTC4306_HIGH, or LTC4306_NO_CONN.
 * @param addr2 An enum value of type ltc4306_addr_conn representing the logic
 * level of the third address pin. Must be one of LTC4306_LOW,
 * LTC4306_HIGH, or LTC4306_NO_CONN.
 * @return Returns 0 on success, or a negative error code if any address
 * connection value is invalid.
 ******************************************************************************/
int ltc4306_addr_gen(struct ltc4306_init_param *init_param,
		     enum ltc4306_addr_conn addr0, enum ltc4306_addr_conn addr1,
		     enum ltc4306_addr_conn addr2);

/* Initialize device */
/***************************************************************************//**
 * @brief This function initializes an LTC4306 device using the provided
 * initialization parameters. It allocates memory for the device
 * structure and sets up the I2C communication based on the given
 * parameters. This function must be called before any other operations
 * on the device to ensure proper setup. If the initialization fails, it
 * returns an error code and ensures that no resources are leaked. The
 * caller is responsible for managing the memory of the device structure,
 * including freeing it when no longer needed.
 *
 * @param device A pointer to a pointer of type `struct ltc4306_dev`. This will
 * be set to point to the newly allocated and initialized device
 * structure. Must not be null.
 * @param init_param A structure of type `struct ltc4306_init_param` containing
 * the initialization parameters, including I2C configuration.
 * The structure is passed by value and must be properly
 * initialized before calling this function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or I2C initialization failure.
 ******************************************************************************/
int ltc4306_init(struct ltc4306_dev **device,
		 struct ltc4306_init_param init_param);

/* Write byte/s to selected starting register */
/***************************************************************************//**
 * @brief Use this function to write one or more bytes to a specific register of
 * the LTC4306 device. It is essential to ensure that the register
 * address is within the valid writable range before calling this
 * function. The function checks if the write operation would exceed the
 * device's register boundaries and returns an error if it does. This
 * function should be called only after the device has been properly
 * initialized.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param addr The starting register address to write to. Must be between
 * LTC4306_CTRL_REG1 and LTC4306_CTRL_REG3 inclusive.
 * @param write_data A pointer to the data to be written. The caller retains
 * ownership and it must not be null.
 * @param bytes The number of bytes to write. The sum of addr and bytes must not
 * exceed LTC4306_OUT_OF_BOUNDS.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ltc4306_write(struct ltc4306_dev *dev, uint8_t addr, uint8_t *write_data,
		  uint8_t bytes);

/* Read byte/s from selected starting register */
/***************************************************************************//**
 * @brief Use this function to read a specified number of bytes from a starting
 * register address on the LTC4306 device. It is essential to ensure that
 * the register address is within the valid range of readable registers
 * and that the read operation does not exceed the device's register
 * boundaries. This function should be called when the device is properly
 * initialized and ready for I2C communication. It returns an error code
 * if the address is invalid or if the read operation fails.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param addr The starting register address from which to read. Must be between
 * LTC4306_CTRL_REG0 and LTC4306_CTRL_REG3 inclusive.
 * @param read_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold the
 * number of bytes specified by the 'bytes' parameter.
 * @param bytes The number of bytes to read from the device. The sum of 'addr'
 * and 'bytes' must not exceed LTC4306_OUT_OF_BOUNDS.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ltc4306_read(struct ltc4306_dev *dev, uint8_t addr, uint8_t *read_data,
		 uint8_t bytes);

/* Update selected register using mask and desired value */
/***************************************************************************//**
 * @brief This function updates a specified register of the LTC4306 device by
 * applying a mask and a new value. It is used when you need to modify
 * specific bits of a register without affecting other bits. The function
 * first reads the current value of the register, applies the mask to
 * clear the bits to be updated, and then sets the new value. It must be
 * called with a valid device structure and a valid register address. The
 * function returns an error code if the read or write operation fails.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param addr The address of the register to be updated. Must be a valid
 * register address for the LTC4306 device.
 * @param update_mask An integer mask indicating which bits of the register
 * should be cleared before setting the new value. The mask
 * should be constructed such that bits to be updated are set
 * to 1.
 * @param update_val An integer value containing the new bits to be set in the
 * register. Only the bits specified by the update_mask will
 * be affected.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the read or write operation fails.
 ******************************************************************************/
int ltc4306_reg_update(struct ltc4306_dev *dev, uint8_t addr, int update_mask,
		       int update_val);

/* Deallocate resources for device */
/***************************************************************************//**
 * @brief Use this function to properly release all resources allocated for an
 * LTC4306 device when it is no longer needed. This function should be
 * called to clean up after the device has been initialized and used,
 * ensuring that any associated I2C descriptors are also removed. It is
 * important to call this function to prevent memory leaks and to ensure
 * that the I2C resources are properly freed. The function returns an
 * integer status code indicating the success or failure of the resource
 * deallocation process.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device to
 * be removed. This pointer must not be null, and it should point to
 * a valid, initialized device structure. The function will handle
 * invalid pointers by potentially causing undefined behavior.
 * @return Returns an integer status code from the underlying I2C removal
 * operation, indicating success (0) or an error code if the operation
 * fails.
 ******************************************************************************/
int ltc4306_remove(struct ltc4306_dev *dev);

/* Checks if any downstream bus is connected to upstream bus */
/***************************************************************************//**
 * @brief Use this function to determine the connection status between
 * downstream and upstream buses in an LTC4306 device. It should be
 * called when you need to verify if a downstream bus is currently
 * connected. Ensure that the device has been properly initialized before
 * calling this function. The function will update the provided boolean
 * pointer with the connection status, and it returns an error code if
 * the operation fails.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param downstream_conn A pointer to a boolean variable where the connection
 * status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. The boolean pointed to by downstream_conn is updated to
 * reflect the connection status.
 ******************************************************************************/
int ltc4306_downstream_check(struct ltc4306_dev *dev, bool *downstream_conn);

/* Reads the status of selected ALERT pin */
/***************************************************************************//**
 * @brief Use this function to determine the logic state of a specific ALERT pin
 * on the LTC4306 device. It is essential to ensure that the
 * `alert_pin_number` is within the valid range before calling this
 * function. The function will return an error if the pin number is out
 * of bounds. The logic state is returned through the `is_high`
 * parameter, indicating whether the pin is in a high state. This
 * function should be called when the ALERT pin status needs to be
 * monitored or checked.
 *
 * @param dev A pointer to an initialized `ltc4306_dev` structure representing
 * the device. Must not be null.
 * @param alert_pin_number The number of the ALERT pin to check. Must be between
 * `LTC4306_MIN_CHANNEL_INDEX` and
 * `LTC4306_MAX_CHANNEL_INDEX`, inclusive. If out of
 * range, the function returns an error.
 * @param is_high A pointer to a boolean where the function will store the logic
 * state of the specified ALERT pin. Must not be null.
 * @return Returns 0 on success, or a negative error code if the
 * `alert_pin_number` is out of range or if there is an I2C
 * communication error.
 ******************************************************************************/
int ltc4306_read_alert_logic_state(struct ltc4306_dev *dev,
				   uint8_t alert_pin_number, bool *is_high);

/* Checks if any downstream connection attempt failed */
/***************************************************************************//**
 * @brief Use this function to determine if any downstream connection attempt
 * has failed on the LTC4306 device. It should be called when you need to
 * verify the connection status after attempting to connect downstream
 * buses. Ensure that the device has been properly initialized before
 * calling this function. The function will update the provided boolean
 * pointer to indicate the failure status.
 *
 * @param dev A pointer to an initialized `ltc4306_dev` structure representing
 * the device. Must not be null.
 * @param is_high A pointer to a boolean where the function will store the
 * result. It will be set to true if a downstream connection
 * attempt has failed, otherwise false. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc4306_read_failed_conn(struct ltc4306_dev *dev, bool *is_high);

/* Checks if any latched timeout occurs */
/***************************************************************************//**
 * @brief This function checks the status of a latched timeout condition on the
 * LTC4306 device. It should be called when you need to determine if a
 * timeout event has been latched, which may indicate a communication
 * issue or other fault condition. The function requires a valid device
 * structure and a pointer to a boolean variable where the result will be
 * stored. It is important to ensure that the device has been properly
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param timed_out A pointer to a boolean variable where the function will
 * store the result. Must not be null. The variable will be set
 * to true if a latched timeout has occurred, otherwise false.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc4306_get_latched_timeout(struct ltc4306_dev *dev, bool *timed_out);

/* Check status of stuck low timeout circuitry */
/***************************************************************************//**
 * @brief This function checks whether the real-time timeout circuitry of the
 * LTC4306 device has been triggered. It should be called when you need
 * to verify the timeout status of the device. The function requires a
 * valid device structure and a pointer to a boolean variable where the
 * result will be stored. It returns an error code if the read operation
 * fails, otherwise it returns 0.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param timed_out A pointer to a boolean variable where the function will
 * store the timeout status. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. The boolean pointed to by timed_out is set to true if a
 * timeout has occurred, otherwise false.
 ******************************************************************************/
int ltc4306_get_realtime_timeout(struct ltc4306_dev *dev, bool *timed_out);

/* Sets status of upstream accelerator */
/***************************************************************************//**
 * @brief Use this function to enable or disable the upstream accelerator on the
 * LTC4306 device. This function should be called when you need to
 * control the upstream acceleration feature, which may be necessary
 * depending on your specific application requirements. Ensure that the
 * device has been properly initialized before calling this function. The
 * function updates the device's internal state and attempts to write the
 * new configuration to the appropriate control register. It returns an
 * integer status code indicating the success or failure of the
 * operation.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param upstream_en A boolean value indicating whether to enable (true) or
 * disable (false) the upstream accelerator.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int ltc4306_set_upstream_accel(struct ltc4306_dev *dev, bool upstream_en);

/* Gets status of upstream accelerator */
/***************************************************************************//**
 * @brief Use this function to check whether the upstream accelerator is enabled
 * on the LTC4306 device. It must be called with a valid device structure
 * that has been properly initialized. The function reads the relevant
 * register to determine the status and updates the provided boolean
 * pointer with the result. This function is useful for monitoring the
 * current configuration of the device's upstream acceleration feature.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param upstream_en A pointer to a boolean variable where the status of the
 * upstream accelerator will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. The boolean pointed to by upstream_en is updated with the
 * status of the upstream accelerator.
 ******************************************************************************/
int ltc4306_get_upstream_accel(struct ltc4306_dev *dev, bool *upstream_en);

/* Sets status of downstream accelerator */
/***************************************************************************//**
 * @brief Use this function to enable or disable the downstream accelerator on
 * the LTC4306 device. This function should be called when you need to
 * control the downstream acceleration feature, which may be necessary
 * for specific operational requirements of the device. Ensure that the
 * device has been properly initialized before calling this function. The
 * function updates the internal state of the device and attempts to
 * write the new configuration to the appropriate control register. It
 * returns an integer status code indicating the success or failure of
 * the operation.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device.
 * Must not be null, and the device must be initialized before use.
 * @param downstream_en A boolean value indicating whether to enable (true) or
 * disable (false) the downstream accelerator.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code on failure.
 ******************************************************************************/
int ltc4306_set_downstream_accel(struct ltc4306_dev *dev, bool downstream_en);

/* Gets status of downstream accelerator */
/***************************************************************************//**
 * @brief Use this function to check whether the downstream accelerator is
 * enabled on the LTC4306 device. It must be called with a valid device
 * structure that has been properly initialized. The function reads the
 * relevant register to determine the status and updates the provided
 * boolean pointer with the result. This function is useful for
 * monitoring and debugging the state of the downstream accelerator in
 * applications where the LTC4306 is used.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param downstream_en A pointer to a boolean variable where the function will
 * store the status of the downstream accelerator. Must not
 * be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. The boolean pointed to by downstream_en is updated with the
 * downstream accelerator status if successful.
 ******************************************************************************/
int ltc4306_get_downstream_accel(struct ltc4306_dev *dev, bool *downstream_en);

/* Sets GPIO Output Driver state */
/***************************************************************************//**
 * @brief This function is used to set the output state of a specified GPIO pin
 * on the LTC4306 device to either high or low. It should be called when
 * the GPIO pin needs to be controlled programmatically. The function
 * requires a valid GPIO pin number within the defined range and a
 * boolean indicating the desired state. If the GPIO pin number is
 * outside the valid range, the function returns an error code. This
 * function is typically used in applications where GPIO control is
 * necessary for device operation.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device.
 * Must not be null, and the device must be properly initialized
 * before calling this function.
 * @param gpio An integer representing the GPIO pin number to be controlled.
 * Valid values are between LTC4306_GPIO_MIN and LTC4306_GPIO_MAX,
 * inclusive. If the value is outside this range, the function
 * returns an error.
 * @param is_high A boolean value indicating the desired output state of the
 * GPIO pin. If true, the pin is set to high; if false, the pin
 * is set to low.
 * @return Returns 0 on success or a negative error code if the GPIO pin number
 * is invalid.
 ******************************************************************************/
int ltc4306_set_gpio_output_state(struct ltc4306_dev *dev, int gpio,
				  bool is_high);

/* Gets GPIO Output Driver state */
/***************************************************************************//**
 * @brief Use this function to determine the current output state (high or low)
 * of a specific GPIO pin on the LTC4306 device. It is essential to
 * ensure that the GPIO number provided is within the valid range before
 * calling this function. The function will return an error if the GPIO
 * number is out of bounds. The output state is returned through a
 * pointer, which must not be null, and the caller is responsible for
 * providing a valid pointer to store the result.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param gpio An integer representing the GPIO pin number to query. Valid
 * values are between LTC4306_GPIO_MIN and LTC4306_GPIO_MAX
 * inclusive. If the value is outside this range, the function
 * returns an error.
 * @param is_high A pointer to a boolean where the output state will be stored.
 * Must not be null. The function writes true if the GPIO is
 * high, false if it is low.
 * @return Returns 0 on success, or a negative error code if the GPIO number is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int ltc4306_get_gpio_output_state(struct ltc4306_dev *dev, int gpio,
				  bool *is_high);

/* Reads logic level of selected GPIO */
/***************************************************************************//**
 * @brief Use this function to determine the current logic state (high or low)
 * of a specified GPIO pin on the LTC4306 device. It is essential to
 * ensure that the GPIO number provided is within the valid range before
 * calling this function. The function requires a valid device structure
 * and a pointer to store the result. If the GPIO number is out of range,
 * the function returns an error code. This function is typically used in
 * applications where monitoring the state of GPIO pins is necessary.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param gpio An integer representing the GPIO pin number to read. Valid values
 * are between LTC4306_GPIO_MIN and LTC4306_GPIO_MAX inclusive. If
 * the value is outside this range, the function returns an error.
 * @param is_high A pointer to a boolean variable where the function will store
 * the logic state of the specified GPIO pin. Must not be null.
 * @return Returns 0 on success, or a negative error code if the GPIO number is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int ltc4306_read_gpio_logic_state(struct ltc4306_dev *dev, int gpio,
				  bool *is_high);

/* Sets the LTC4306's connection requirement bit field for downstream connect */
/***************************************************************************//**
 * @brief This function is used to set the connection requirement bit field for
 * the downstream connection on an LTC4306 device. It should be called
 * when there is a need to modify the connection behavior of the device,
 * particularly when deciding whether to connect regardless of the
 * current state. The function requires a valid device structure and a
 * boolean flag indicating the desired connection behavior. It returns an
 * integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an ltc4306_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param connect_regardless A boolean flag indicating whether to connect
 * regardless of the current state. If true, the
 * connection is requested regardless of other
 * conditions.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int ltc4306_set_conn_req(struct ltc4306_dev *dev, bool connect_regardless);

/* Gets the LTC4306's connection requirement bit field for downstream connect */
/***************************************************************************//**
 * @brief This function checks the connection requirement status of the LTC4306
 * device for downstream connections and stores the result in the
 * provided boolean pointer. It should be called when you need to
 * determine if the device is set to connect downstream buses regardless
 * of other conditions. Ensure that the device is properly initialized
 * before calling this function. The function will return an error code
 * if the read operation fails.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param connect_regardless A pointer to a boolean variable where the
 * connection requirement status will be stored. Must
 * not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc4306_get_conn_req(struct ltc4306_dev *dev, bool *connect_regardless);

/* Configure LTC4306's GPIO mode and output config (for output) */
/***************************************************************************//**
 * @brief This function is used to set the mode and output configuration for the
 * GPIO pins of an LTC4306 device. It allows the user to specify whether
 * each GPIO pin should be configured as an input or output, and if
 * configured as an output, whether it should operate in push-pull or
 * open-drain mode. This function should be called after the device has
 * been initialized and is ready for configuration. It returns an error
 * code if the configuration fails, which can be used to handle errors
 * appropriately.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device to configure. Must not be null.
 * @param gpio1 A boolean indicating whether GPIO1 should be configured as an
 * output (true) or input (false).
 * @param gpio2 A boolean indicating whether GPIO2 should be configured as an
 * output (true) or input (false).
 * @param gpio1_is_pushpull A boolean indicating whether GPIO1 should be
 * configured in push-pull mode (true) or open-drain
 * mode (false) if it is set as an output.
 * @param gpio2_is_pushpull A boolean indicating whether GPIO2 should be
 * configured in push-pull mode (true) or open-drain
 * mode (false) if it is set as an output.
 * @return Returns 0 on success or a negative error code on failure.
 ******************************************************************************/
int ltc4306_gpio_configure(struct ltc4306_dev *dev, bool gpio1,
			   bool gpio2, bool gpio1_is_pushpull, bool gpio2_is_pushpull);

/* Sets the mass write bit field */
/***************************************************************************//**
 * @brief Use this function to enable or disable the mass write feature on the
 * LTC4306 device. This function should be called when you need to
 * control the mass write capability, which affects how data is written
 * to the device. Ensure that the device has been properly initialized
 * before calling this function. The function updates the relevant
 * control register based on the provided boolean flag, and it returns an
 * integer status code indicating success or failure of the operation.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param mass_write_en A boolean value indicating whether to enable (true) or
 * disable (false) the mass write feature.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int ltc4306_set_mass_write(struct ltc4306_dev *dev, bool mass_write_en);

/* Gets the mass write bit field */
/***************************************************************************//**
 * @brief Use this function to check if the mass write feature is enabled on the
 * LTC4306 device. It must be called with a valid device structure that
 * has been properly initialized. The function reads the relevant
 * register from the device and updates the provided boolean pointer with
 * the mass write status. Ensure that the device is correctly set up and
 * that the pointer for the status is not null before calling this
 * function.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param mass_write_en A pointer to a boolean variable where the mass write
 * enable status will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc4306_get_mass_write(struct ltc4306_dev *dev, bool *mass_write_en);

/* Sets the LTC4306's stuck low timeout mode */
/***************************************************************************//**
 * @brief Use this function to configure the stuck low timeout mode of an
 * LTC4306 device. This function should be called when you need to change
 * the timeout behavior of the device, which can be useful in managing
 * bus hang conditions. Ensure that the device has been properly
 * initialized before calling this function. The function will return an
 * error if the specified timeout mode is outside the valid range.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param tout An enum value of type ltc4306_timeout_mode specifying the desired
 * timeout mode. Valid values are LTC4306_DISABLED, LTC4306_30MS,
 * LTC4306_15MS, and LTC4306_7P5MS. If an invalid value is provided,
 * the function returns an error.
 * @return Returns 0 on success or a negative error code if the timeout mode is
 * invalid.
 ******************************************************************************/
int ltc4306_set_timeout_mode(struct ltc4306_dev *dev,
			     enum ltc4306_timeout_mode tout);

/* Gets the LTC4306's stuck low timeout mode */
/***************************************************************************//**
 * @brief Use this function to obtain the current timeout mode setting of an
 * LTC4306 device. This function is typically called when you need to
 * verify or log the timeout configuration of the device. It requires a
 * valid device structure that has been initialized and configured. The
 * function will populate the provided timeout mode variable with the
 * current setting. Ensure that the device is properly initialized before
 * calling this function to avoid unexpected behavior.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param tout A pointer to an ltc4306_timeout_mode variable where the current
 * timeout mode will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails.
 ******************************************************************************/
int ltc4306_get_timeout_mode(struct ltc4306_dev *dev,
			     enum ltc4306_timeout_mode *tout);

/* Reads LTC4306's selected Bus Logic */
/***************************************************************************//**
 * @brief Use this function to determine the logic state (high or low) of a
 * specified bus on the LTC4306 device. It is essential to ensure that
 * the bus number is within the valid range before calling this function.
 * The function will return an error if the bus number is out of bounds
 * or if the FET is connected, making the logic read invalid. This
 * function should be called when the logic state of a bus needs to be
 * verified, and it requires a valid device structure and a pointer to
 * store the result.
 *
 * @param dev A pointer to an initialized ltc4306_dev structure representing the
 * device. Must not be null.
 * @param bus_num The bus number to read the logic state from. Must be between
 * LTC4306_MIN_CHANNEL_INDEX and LTC4306_MAX_CHANNEL_INDEX
 * inclusive. If out of range, the function returns an error.
 * @param is_high A pointer to a boolean where the logic state will be stored.
 * Must not be null. The function writes true if the bus logic
 * state is high, false if it is low.
 * @return Returns 0 on success, -EINVAL if the bus number is invalid or if the
 * FET is connected, and other negative error codes for I2C read
 * failures.
 ******************************************************************************/
int ltc4306_read_bus_logic_state(struct ltc4306_dev *dev, uint8_t bus_number,
				 bool *is_high);

/* Sets the connection for selected LTC4306 downstream channel */
/***************************************************************************//**
 * @brief Use this function to connect or disconnect a specific downstream
 * channel on the LTC4306 device. It is essential to ensure that the
 * `bus_num` parameter is within the valid range before calling this
 * function. The function checks the current connection requirements and
 * the bus logic state to determine if the connection can be established.
 * If the connection cannot be established due to invalid parameters or
 * bus logic state, an error code is returned. This function should be
 * used when you need to control the connectivity of downstream channels
 * dynamically.
 *
 * @param dev A pointer to an initialized `ltc4306_dev` structure representing
 * the device. Must not be null.
 * @param bus_num The downstream channel number to be connected or disconnected.
 * Must be between `LTC4306_MIN_CHANNEL_INDEX` and
 * `LTC4306_MAX_CHANNEL_INDEX`, inclusive. If out of range, the
 * function returns an error.
 * @param connect A boolean value indicating whether to connect (`true`) or
 * disconnect (`false`) the specified downstream channel.
 * @return Returns 0 on success, or a negative error code if the operation fails
 * due to invalid parameters or bus logic state.
 ******************************************************************************/
int ltc4306_set_downstream_channel(struct ltc4306_dev *dev,
				   uint8_t bus_num, bool connect);

/* Gets the connection for selected LTC4306 downstream channel */
/***************************************************************************//**
 * @brief Use this function to determine if a specific downstream channel on the
 * LTC4306 device is currently connected. It is essential to ensure that
 * the `dev` parameter is a valid, initialized device structure before
 * calling this function. The `bus_num` must be within the valid range of
 * channel indices, otherwise, the function will return an error. The
 * connection status is returned through the `connect` parameter, which
 * will be set to true if the channel is connected, and false otherwise.
 *
 * @param dev A pointer to an initialized `ltc4306_dev` structure representing
 * the device. Must not be null.
 * @param bus_num An unsigned 8-bit integer representing the downstream channel
 * number to check. Must be between `LTC4306_MIN_CHANNEL_INDEX`
 * and `LTC4306_MAX_CHANNEL_INDEX`, inclusive. If out of range,
 * the function returns an error.
 * @param connect A pointer to a boolean where the connection status will be
 * stored. Must not be null. Will be set to true if the channel
 * is connected, false otherwise.
 * @return Returns 0 on success, or a negative error code if the bus number is
 * out of range or if there is an I2C communication error.
 ******************************************************************************/
int ltc4306_get_downstream_channel(struct ltc4306_dev *dev,
				   uint8_t bus_num, bool *connect);

#endif	/* __LTC4306_H__ */
