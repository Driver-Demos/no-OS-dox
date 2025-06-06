/***************************************************************************//**
 *   @file   max9611.h
 *   @brief  Header file of max9611 Family Driver.
 *   @author MSosa (marcpaolo.sosa@analog.com)
 *   @author JSanbuen (jose.sanbuenaventura@analog.com)
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
#ifndef __MAX9611_H__
#define __MAX9611_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include "no_os_i2c.h"
#include "no_os_util.h"
#include "no_os_error.h"
#include "no_os_delay.h"

#define MAX9611_MUX_MASK		NO_OS_GENMASK(2, 0)
#define MAX9611_MODE_MASK		NO_OS_GENMASK(7, 5)
#define MAX9611_RAW_DATA_MASK	NO_OS_GENMASK(15, 4)

#define MAX9611_SHDN_MASK		NO_OS_BIT(3)
#define MAX9611_LR_MASK			NO_OS_BIT(4)
#define MAX9611_DTIME_MASK		NO_OS_BIT(3)
#define MAX9611_RTIME_MASK		NO_OS_BIT(2)

#define	MAX9611_CSA_MSB			0x00
#define MAX9611_CSA_LSB			0x01
#define	MAX9611_RSP_MSB			0x02
#define	MAX9611_RSP_LSB			0x03
#define	MAX9611_OUT_MSB			0x04
#define	MAX9611_OUT_LSB			0x05
#define	MAX9611_SET_MSB			0x06
#define	MAX9611_SET_LSB			0x07
#define	MAX9611_TMP_MSB			0x08
#define	MAX9611_TMP_LSB			0x09
#define	MAX9611_CTR1			0x0A
#define	MAX9611_CTR2			0x0B

#define MAX9611_OUT_OF_BOUNDS  	0x0C

/******************************************************************************/
/*************************** Types Declarations *******************************/
/***************************************************************************//**
 * @brief The `max9611_data` enumeration defines a set of constants used to
 * specify different types of data that can be captured or processed by
 * the MAX9611 device. Each enumerator corresponds to a specific data
 * type, such as current sense amplifier data, resistor sense path data,
 * output data, set data, and temperature data, which are relevant for
 * configuring and operating the MAX9611 device.
 *
 * @param MAX9611_DATA_CSA Represents the current sense amplifier data.
 * @param MAX9611_DATA_RSP Represents the resistor sense path data.
 * @param MAX9611_DATA_OUT Represents the output data.
 * @param MAX9611_DATA_SET Represents the set data.
 * @param MAX9611_DATA_TMP Represents the temperature data.
 ******************************************************************************/
enum max9611_data {
	MAX9611_DATA_CSA,
	MAX9611_DATA_RSP,
	MAX9611_DATA_OUT,
	MAX9611_DATA_SET,
	MAX9611_DATA_TMP,
};

/***************************************************************************//**
 * @brief The `max9611_addr_lvls` enumeration defines the possible voltage
 * levels that can be used to set the address pins (A0 and A1) of the
 * MAX9611 device. These levels are used to generate the I2C slave
 * address for the device, allowing multiple devices to be addressed on
 * the same bus by configuring their address pins to different voltage
 * levels.
 *
 * @param MAX9611_ZERO_VCC Represents a logic level of 0 V for the MAX9611
 * address pin.
 * @param MAX9611_33_VCC Represents a logic level of 3.3 V for the MAX9611
 * address pin.
 * @param MAX9611_66_VCC Represents a logic level of 6.6 V for the MAX9611
 * address pin.
 * @param MAX9611_100_VCC Represents a logic level of 10.0 V for the MAX9611
 * address pin.
 ******************************************************************************/
enum max9611_addr_lvls {
	MAX9611_ZERO_VCC,
	MAX9611_33_VCC,
	MAX9611_66_VCC,
	MAX9611_100_VCC,
};

/***************************************************************************//**
 * @brief The `max9611_delay_time` enumeration defines two possible delay times
 * that can be configured for the MAX9611 device: 1 millisecond and 100
 * microseconds. This enumeration is used to specify the delay time
 * setting when configuring the device's operational parameters, allowing
 * for precise control over timing-related functions.
 *
 * @param MAX9611_1MS Represents a delay time of 1 millisecond.
 * @param MAX9611_100US Represents a delay time of 100 microseconds.
 ******************************************************************************/
enum max9611_delay_time {
	MAX9611_1MS,
	MAX9611_100US,
};

/***************************************************************************//**
 * @brief The `max9611_retry_time` enumeration defines two possible retry time
 * intervals for the MAX9611 device, which are 50 milliseconds and 10
 * milliseconds. This enumeration is used to configure the retry timing
 * behavior of the device, allowing for adjustments in how frequently the
 * device attempts to perform certain operations after a failure or
 * timeout.
 *
 * @param MAX9611_50MS Represents a retry time of 50 milliseconds.
 * @param MAX9611_10MS Represents a retry time of 10 milliseconds.
 ******************************************************************************/
enum max9611_retry_time {
	MAX9611_50MS,
	MAX9611_10MS,
};

/***************************************************************************//**
 * @brief The `max9611_mux_conf` is an enumeration that defines various
 * configuration settings for the multiplexer in the MAX9611 device. Each
 * enumerator represents a specific configuration mode, such as different
 * sense multipliers, ADC data handling, temperature sensing, and
 * operational modes. This enumeration is used to set or retrieve the
 * current multiplexer configuration in the MAX9611 device, allowing for
 * flexible control over its operation.
 *
 * @param MAX9611_CONF_SENSE_1X Represents a 1x sense configuration for the
 * MAX9611 multiplexer.
 * @param MAX9611_CONF_SENSE_4X Represents a 4x sense configuration for the
 * MAX9611 multiplexer.
 * @param MAX9611_CONF_SENSE_8X Represents an 8x sense configuration for the
 * MAX9611 multiplexer.
 * @param MAX9611_CONF_IN_COM_MODE Configures the multiplexer to input common
 * mode.
 * @param MAX9611_CONF_OUT_ADC Configures the multiplexer to output ADC data.
 * @param MAX9611_CONF_SET_ADC Configures the multiplexer to set ADC data.
 * @param MAX9611_CONF_TEMP Configures the multiplexer for temperature sensing.
 * @param MAX9611_FAST_MODE Configures the multiplexer for fast mode operation.
 ******************************************************************************/
enum max9611_mux_conf {
	MAX9611_CONF_SENSE_1X,
	MAX9611_CONF_SENSE_4X,
	MAX9611_CONF_SENSE_8X,
	MAX9611_CONF_IN_COM_MODE,
	MAX9611_CONF_OUT_ADC,
	MAX9611_CONF_SET_ADC,
	MAX9611_CONF_TEMP,
	MAX9611_FAST_MODE,
};

/***************************************************************************//**
 * @brief The `max9611_mode_conf` is an enumeration that defines the different
 * operating modes for the MAX9611 device. Each mode is represented by a
 * specific hexadecimal value, allowing the device to be configured for
 * normal operation, operational amplifier mode, or comparator mode. This
 * enumeration is used to set or retrieve the current mode of the MAX9611
 * device, facilitating its integration and control in various
 * applications.
 *
 * @param MAX9611_NORMAL_MODE Represents the normal operating mode with a value
 * of 0x0.
 * @param MAX9611_OPAMP_MODE Represents the operational amplifier mode with a
 * value of 0x3.
 * @param MAX9611_COMPARATOR_MODE Represents the comparator mode with a value of
 * 0x7.
 ******************************************************************************/
enum max9611_mode_conf {
	MAX9611_NORMAL_MODE = 0x0,
	MAX9611_OPAMP_MODE = 0x3,
	MAX9611_COMPARATOR_MODE = 0x7,
};

/***************************************************************************//**
 * @brief The `max9611_dev` structure is designed to represent a device instance
 * for the MAX9611 family of devices, which are current sense amplifiers
 * with ADC capabilities. It contains a pointer to an I2C descriptor for
 * managing I2C communication and an enumeration to specify the type of
 * data capture configuration for the ADC. This structure is essential
 * for initializing and managing the device's operations through various
 * functions that interact with the hardware.
 *
 * @param i2c_desc A pointer to a no_os_i2c_desc structure, which describes the
 * I2C interface for communication.
 * @param capture_type An enum max9611_data value that specifies the type of ADC
 * data to capture.
 ******************************************************************************/
struct max9611_dev {
	struct no_os_i2c_desc *i2c_desc;
	/* ADC Data configuration */
	enum max9611_data capture_type;
};

/***************************************************************************//**
 * @brief The `max9611_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the MAX9611 device,
 * specifically focusing on the I2C communication interface. It contains
 * a single member, `i2c_init`, which is a structure that holds the
 * necessary parameters for initializing the I2C interface, ensuring that
 * the device can communicate effectively over the I2C bus.
 *
 * @param i2c_init This member is a structure of type no_os_i2c_init_param, used
 * to initialize I2C communication parameters.
 ******************************************************************************/
struct max9611_init_param {
	struct no_os_i2c_init_param i2c_init;
};

/* Generate slave address depending on A0 and A1 logic levels */
/***************************************************************************//**
 * @brief This function calculates and sets the I2C slave address for a MAX9611
 * device based on the specified logic levels of the A0 and A1 pins. It
 * should be used during the initialization phase of the device setup.
 * The function requires valid logic levels for both A0 and A1, which
 * must be within the defined range of `max9611_addr_lvls` enum. If the
 * provided levels are invalid, the function returns an error code. The
 * calculated address is stored in the `i2c_init` structure of the
 * provided `device_ip` parameter.
 *
 * @param device_ip A pointer to a `max9611_init_param` structure where the
 * calculated I2C slave address will be stored. Must not be
 * null.
 * @param a0 An enum value of type `max9611_addr_lvls` representing the logic
 * level of the A0 pin. Must be a valid enum value within the range.
 * @param a1 An enum value of type `max9611_addr_lvls` representing the logic
 * level of the A1 pin. Must be a valid enum value within the range.
 * @return Returns 0 on success, or a negative error code if the input levels
 * are invalid.
 ******************************************************************************/
int max9611_addr_gen(struct max9611_init_param *device_ip,
		     enum max9611_addr_lvls a0, enum max9611_addr_lvls a1);

/* Initialize device */
/***************************************************************************//**
 * @brief This function sets up a MAX9611 device by allocating necessary
 * resources and initializing the I2C communication based on the provided
 * initialization parameters. It must be called before any other
 * operations on the MAX9611 device to ensure that the device is properly
 * configured and ready for use. If the initialization fails, the
 * function returns an error code and no resources are allocated. The
 * caller is responsible for managing the memory of the device structure,
 * including freeing it when no longer needed.
 *
 * @param device A pointer to a pointer of a max9611_dev structure. This will be
 * allocated and initialized by the function. Must not be null.
 * @param init_param A max9611_init_param structure containing the
 * initialization parameters for the device, including I2C
 * configuration. The caller retains ownership of this
 * structure.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, such as memory
 * allocation failure or I2C initialization failure.
 ******************************************************************************/
int max9611_init(struct max9611_dev **device,
		 struct max9611_init_param init_param);

/* Deallocate resources for device */
/***************************************************************************//**
 * @brief This function should be called to properly release resources allocated
 * for a MAX9611 device when it is no longer needed. It ensures that the
 * I2C descriptor associated with the device is removed and the memory
 * allocated for the device structure is freed. This function must be
 * called after the device is initialized and used, to prevent memory
 * leaks. It returns an integer status code indicating the success or
 * failure of the resource deallocation process.
 *
 * @param dev A pointer to a max9611_dev structure representing the device to be
 * removed. Must not be null. The function will handle freeing the
 * memory, so the caller should not use the pointer after calling
 * this function.
 * @return Returns an integer status code from the I2C removal operation, where
 * 0 typically indicates success and a negative value indicates an
 * error.
 ******************************************************************************/
int max9611_remove(struct max9611_dev *dev);

/* Read byte/s from selected starting register */
/***************************************************************************//**
 * @brief This function reads a specified number of bytes from a given starting
 * register address of the MAX9611 device. It is essential to ensure that
 * the register address is within the valid range defined by the device,
 * and that the read operation does not exceed the device's register
 * boundaries. This function should be called when data needs to be
 * retrieved from the device's registers, and it requires a properly
 * initialized device structure. The function returns an error code if
 * the address is invalid or if the read operation fails.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param addr The starting register address from which to read. Must be between
 * MAX9611_CSA_MSB and MAX9611_CTR2 inclusive. If outside this
 * range, the function returns an error.
 * @param bytes The number of bytes to read from the starting address. The sum
 * of addr and bytes must not exceed MAX9611_OUT_OF_BOUNDS,
 * otherwise the function returns an error.
 * @param read_data A pointer to a buffer where the read data will be stored.
 * Must not be null and should be large enough to hold the
 * number of bytes specified by the bytes parameter.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * invalid address or read operation failure.
 ******************************************************************************/
int max9611_read(struct max9611_dev *dev, uint8_t addr,
		 uint8_t bytes, uint8_t *read_data);

/* Write byte/s to selected starting register */
/***************************************************************************//**
 * @brief This function writes a specified number of bytes to a given register
 * address on the MAX9611 device. It should be used when you need to
 * configure or update the settings of the device by writing data to its
 * registers. The function requires a valid device structure and a
 * register address within the writable range. It checks for valid
 * register addresses and ensures that the write operation does not
 * exceed the device's register boundaries. The function will return an
 * error if the address is out of bounds or if the target register is
 * read-only.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param addr The starting register address to write to. Must be within the
 * range of writable registers (0x0A to 0x0B). Addresses outside
 * this range will result in an error.
 * @param bytes The number of bytes to write. The sum of addr and bytes must not
 * exceed the MAX9611_OUT_OF_BOUNDS value (0x0C).
 * @param write_data A pointer to the data to be written. Must not be null and
 * should contain at least 'bytes' number of elements.
 * @return Returns 0 on success or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int max9611_write(struct max9611_dev *dev, uint8_t addr,
		  uint8_t bytes, uint8_t *write_data);

/* Update selected register using mask and desired value */
/***************************************************************************//**
 * @brief This function updates a specific register in the MAX9611 device by
 * applying a mask and a new value. It is typically used when you need to
 * modify certain bits of a register without affecting the other bits.
 * The function first reads the current value of the register, applies
 * the mask to clear specific bits, and then sets the desired bits using
 * the provided value. It must be called with a valid device structure
 * and a valid register address. The function returns an error code if
 * the read or write operation fails.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null, and the device must be properly initialized before
 * calling this function.
 * @param addr The address of the register to update. Must be a valid register
 * address within the MAX9611 device.
 * @param update_mask An integer mask indicating which bits of the register
 * should be cleared before setting the new value. The mask
 * should be carefully chosen to target only the bits you
 * intend to modify.
 * @param update_val An integer value containing the bits to set in the register
 * after applying the mask. Only the bits specified by the
 * mask will be affected.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int max9611_reg_update(struct max9611_dev *dev, uint8_t addr,
		       int update_mask, int update_val);

/* Set MAX9611 operating mode */
/***************************************************************************//**
 * @brief Use this function to configure the operating mode of a MAX9611 device.
 * It must be called with a valid device structure and a mode
 * configuration value. The function checks if the provided mode is one
 * of the supported modes (normal, op-amp, or comparator) and updates the
 * device's mode register accordingly. This function should be called
 * after the device has been initialized and before performing operations
 * that depend on the mode setting.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null and should be properly initialized before calling this
 * function.
 * @param mode An enum max9611_mode_conf value representing the desired
 * operating mode. Valid values are MAX9611_NORMAL_MODE,
 * MAX9611_OPAMP_MODE, and MAX9611_COMPARATOR_MODE. If an invalid
 * mode is provided, the function returns an error.
 * @return Returns 0 on success, or a negative error code if the mode is invalid
 * or if there is an issue updating the device's register.
 ******************************************************************************/
int max9611_set_mode(struct max9611_dev *dev, enum max9611_mode_conf mode);

/* Get MAX9611 operating mode */
/***************************************************************************//**
 * @brief This function is used to obtain the current operating mode of a
 * MAX9611 device. It should be called when you need to check the
 * device's mode configuration, which can be one of several predefined
 * modes. The function requires a valid device structure that has been
 * initialized and a pointer to an enum where the mode will be stored. It
 * is important to ensure that the device is properly initialized before
 * calling this function to avoid undefined behavior. The function will
 * return an error code if the mode cannot be retrieved successfully.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership and must ensure it is not null.
 * @param mode A pointer to an enum max9611_mode_conf where the current mode
 * will be stored. This must not be null, and the caller is
 * responsible for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if the mode could not
 * be retrieved.
 ******************************************************************************/
int max9611_get_mode(struct max9611_dev *dev, enum max9611_mode_conf *mode);

/* Set MAX9611 MUX configuration */
/***************************************************************************//**
 * @brief This function configures the multiplexer (MUX) setting of a MAX9611
 * device. It should be called when you need to change the MUX
 * configuration to one of the predefined settings. The function updates
 * the device's internal capture type based on the selected MUX
 * configuration. It must be called with a valid device structure and a
 * valid MUX configuration enumeration. If an invalid MUX configuration
 * is provided, the function returns an error code.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null, and the device should be properly initialized before
 * calling this function.
 * @param mux An enumeration value of type max9611_mux_conf representing the
 * desired MUX configuration. Must be one of the predefined values in
 * the max9611_mux_conf enum. Invalid values result in an error.
 * @return Returns 0 on success or a negative error code if the MUX
 * configuration is invalid.
 ******************************************************************************/
int max9611_set_mux(struct max9611_dev *dev, enum max9611_mux_conf mux);

/* Get MAX9611 MUX configuration */
/***************************************************************************//**
 * @brief This function is used to obtain the current multiplexer (MUX)
 * configuration setting from a MAX9611 device. It should be called when
 * you need to know the current MUX configuration for further processing
 * or decision-making. The function requires a valid device structure
 * that has been initialized and a pointer to an enum where the MUX
 * configuration will be stored. It returns an error code if the read
 * operation fails, otherwise it returns 0 indicating success.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. This
 * must be a valid, initialized device structure. The caller retains
 * ownership.
 * @param mux A pointer to an enum max9611_mux_conf where the current MUX
 * configuration will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max9611_get_mux(struct max9611_dev *dev, enum max9611_mux_conf *mux);

/* Configure MAX9611 for shutdown or power up */
/***************************************************************************//**
 * @brief This function is used to set the MAX9611 device into either shutdown
 * mode or power-up mode based on the provided boolean flag. It should be
 * called when there is a need to conserve power by shutting down the
 * device or to resume normal operation by powering it up. The function
 * requires a valid device structure that has been properly initialized.
 * It returns an integer status code indicating success or failure of the
 * operation.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. This
 * must be a valid, initialized device structure and must not be
 * null.
 * @param is_shdn A boolean value where 'true' sets the device to shutdown mode
 * and 'false' sets it to power-up mode.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int max9611_shutdown(struct max9611_dev *dev, bool is_shdn);

/* Set MAX9611 LR bit */
/***************************************************************************//**
 * @brief This function configures the LR bit of the MAX9611 device, which is
 * used to control specific operational modes. It should be called when
 * you need to toggle the LR bit between normal and alternative settings.
 * The function requires a valid device structure and a boolean
 * indicating the desired state. It returns an integer status code
 * indicating success or failure of the operation.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null. The caller retains ownership.
 * @param is_normal A boolean value indicating the desired state of the LR bit.
 * If true, the LR bit is set to normal; otherwise, it is set
 * to an alternative state.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code for failure.
 ******************************************************************************/
int max9611_set_lr(struct max9611_dev *dev, bool is_normal);

/* Get MAX9611 LR bit */
/***************************************************************************//**
 * @brief This function is used to obtain the current status of the LR bit from
 * a MAX9611 device. It should be called when the user needs to check the
 * LR configuration of the device. The function requires a valid device
 * structure that has been initialized and a pointer to a boolean
 * variable where the LR bit status will be stored. The function will
 * return an error code if the read operation fails, otherwise it will
 * return 0 indicating success.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. This
 * must be a valid and initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param lr A pointer to a boolean variable where the function will store the
 * LR bit status. This must not be null, and the caller is responsible
 * for providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max9611_get_lr(struct max9611_dev *dev, bool *lr);

/* Set MAX9611 delay time */
/***************************************************************************//**
 * @brief Use this function to configure the delay time setting of a MAX9611
 * device. This function should be called when you need to adjust the
 * delay time for operations involving the device. Ensure that the device
 * has been properly initialized before calling this function. The
 * function will return an error if the specified delay time is outside
 * the valid range.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. Must
 * not be null, and the device should be initialized before use.
 * @param dtime An enum value of type max9611_delay_time specifying the desired
 * delay time. Valid values are MAX9611_1MS and MAX9611_100US. If
 * the value is outside this range, the function returns an error.
 * @return Returns 0 on success or a negative error code if the delay time is
 * invalid or if the register update fails.
 ******************************************************************************/
int max9611_set_delay(struct max9611_dev *dev, enum max9611_delay_time dtime);

/* Get MAX9611 delay time */
/***************************************************************************//**
 * @brief This function is used to obtain the current delay time configuration
 * from a MAX9611 device. It should be called when you need to verify or
 * utilize the delay time setting in your application. The function
 * requires a valid device structure that has been initialized and a
 * pointer to an enum where the delay time will be stored. It returns an
 * error code if the read operation fails, otherwise it returns 0
 * indicating success.
 *
 * @param dev A pointer to an initialized max9611_dev structure representing the
 * device. Must not be null.
 * @param dtime A pointer to an enum max9611_delay_time where the current delay
 * time setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max9611_get_delay(struct max9611_dev *dev, enum max9611_delay_time *dtime);

/* Set MAX9611 retry time */
/***************************************************************************//**
 * @brief This function configures the retry time setting for a MAX9611 device,
 * which determines the interval between retry attempts. It should be
 * called when you need to adjust the retry timing to either 50ms or
 * 10ms, as defined by the `max9611_retry_time` enumeration. The function
 * must be called with a valid `max9611_dev` device structure that has
 * been properly initialized. If the specified retry time is outside the
 * allowed range, the function returns an error code.
 *
 * @param dev A pointer to a `max9611_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param rtime An enumeration value of type `max9611_retry_time` specifying the
 * desired retry time. Valid values are `MAX9611_50MS` and
 * `MAX9611_10MS`. If an invalid value is provided, the function
 * returns an error.
 * @return Returns 0 on success or a negative error code if the retry time is
 * invalid or if there is a failure in updating the device register.
 ******************************************************************************/
int max9611_set_retry(struct max9611_dev *dev, enum max9611_retry_time rtime);

/* Get MAX9611 retry time */
/***************************************************************************//**
 * @brief This function is used to obtain the current retry time configuration
 * of a MAX9611 device. It should be called when you need to know the
 * retry time setting for operations involving the device. The function
 * requires a valid device structure that has been initialized and a
 * pointer to an enum where the retry time will be stored. It returns an
 * error code if the read operation fails, otherwise it returns 0
 * indicating success.
 *
 * @param dev A pointer to an initialized max9611_dev structure representing the
 * device. Must not be null.
 * @param rtime A pointer to an enum max9611_retry_time where the current retry
 * time setting will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max9611_get_retry(struct max9611_dev *dev, enum max9611_retry_time *rtime);

/* Get MAX9611 raw data */
/***************************************************************************//**
 * @brief This function retrieves raw data from a MAX9611 device based on its
 * current capture type setting. It should be called when you need to
 * access the raw measurement data from the device. The function requires
 * a valid device structure that has been properly initialized and
 * configured. It writes the retrieved raw data into the provided memory
 * location. If the capture type is not recognized, the function returns
 * an error. Ensure that the device is correctly set up and that the raw
 * pointer is valid and non-null before calling this function.
 *
 * @param dev A pointer to a max9611_dev structure representing the device. This
 * must be a valid, initialized device structure. The function will
 * not modify this structure.
 * @param raw A pointer to a uint16_t where the raw data will be stored. This
 * must not be null, and the caller is responsible for ensuring that
 * the memory is valid for writing.
 * @return Returns 0 on success, or a negative error code if the capture type is
 * invalid or if there is a communication error with the device.
 ******************************************************************************/
int max9611_get_raw(struct max9611_dev *dev, uint16_t *raw);

#endif	/* __MAX9611_H__ */
