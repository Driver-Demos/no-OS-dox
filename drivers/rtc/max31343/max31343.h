/*******************************************************************************
 *   @file   max31343.h
 *   @brief  Max31343 Real Time Clock header file
 *   @author Robert Budai (robert.budai@analog.com)
 ********************************************************************************
 * Copyright (c) 2024 Analog Devices, Inc.
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

#ifndef __MAX31343__
#define __MAX31343__

#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_i2c.h"

#define MAX31343_I2C_ADDRESS			0x68
#define MAX31343_R_STATUS			0x00
#define MAX31343_R_INT_EN			0x01
#define MAX31343_R_RTC_RESET			0x02
#define MAX31343_R_CFG1				0x03
#define MAX31343_R_CFG2				0x04
#define MAX31343_R_TIMER_CONFIG			0x05
#define MAX31343_R_SECONDS			0x06
#define MAX31343_R_MINUTES			0x07
#define MAX31343_R_HOURS			0x08
#define MAX31343_R_DAY				0x09
#define MAX31343_R_DATE				0x0A
#define MAX31343_R_MONTH			0x0B
#define MAX31343_R_YEAR				0x0C
#define MAX31343_R_ALM1_SEC			0x0D
#define MAX31343_R_ALM1_MIN			0x0E
#define MAX31343_R_ALM1_HRS			0x0F
#define MAX31343_R_ALM1DAY_DATE			0x10
#define MAX31343_R_ALM1_MON			0x11
#define MAX31343_R_ALM1_YEAR			0x12
#define MAX31343_R_ALM2_MIN			0x13
#define MAX31343_R_ALM2_HRS			0x14
#define MAX31343_R_ALM2DAY_DATE			0x15
#define MAX31343_R_TIMER_COUNT			0x16
#define MAX31343_R_TIMER_INIT			0x17
#define MAX31343_R_PWR_MGMT			0x18
#define MAX31343_R_TRICKLE			0x19
#define MAX31343_R_TEMP_MSB			0x1A
#define MAX31343_R_TEMP_LSB			0x1B
#define MAX31343_R_TS_CONFIG			0x1C
#define MAX31343_R_RAM_REG_START		0x22
#define MAX31343_R_RAM_REG_END			0x61

/* Status register bits */
#define MAX31343_F_STATUS_A1F			NO_OS_BIT(0)
#define MAX31343_F_STATUS_A2F			NO_OS_BIT(1)
#define MAX31343_F_STATUS_TIF			NO_OS_BIT(2)
#define MAX31343_F_STATUS_TSF			NO_OS_BIT(3)
#define MAX31343_F_STATUS_PFAIL			NO_OS_BIT(5)
#define MAX31343_F_STATUS_OSF			NO_OS_BIT(6)
#define MAX31343_F_STATUS_PSDECT		NO_OS_BIT(7)

/* Interrupt enable register bits */
#define MAX31343_F_INT_EN_A1IE			NO_OS_BIT(0)
#define MAX31343_F_INT_EN_A2IE			NO_OS_BIT(1)
#define MAX31343_F_INT_EN_TIE			NO_OS_BIT(2)
#define MAX31343_F_INT_EN_TSIE			NO_OS_BIT(3)
#define MAX31343_F_INT_EN_PFAILE		NO_OS_BIT(5)
#define MAX31343_F_INT_EN_DOSF			NO_OS_BIT(6)

/* RTC reset register bits */
#define MAX31343_F_RTC_RESET_SWRST		NO_OS_BIT(0)

/* Config 1 register bits */
#define MAX31343_F_CFG1_ENOSC			NO_OS_BIT(1)
#define MAX31343_F_CFG1_I2C_TIMEOUT		NO_OS_BIT(3)
#define MAX31343_F_CFG1_DATA_RET		NO_OS_BIT(4)

/* Config 2 register bits */
#define MAX31343_F_CFG2_SQW_HZ			NO_OS_BIT(0)
#define MAX31343_F_CFG2_CLKO_HZ			NO_OS_BIT(3)
#define MAX31343_F_CFG2_ENCLKO			NO_OS_BIT(7)

/* Timer config register bits */
#define MAX31343_F_TIMER_CONFIG_TFS		NO_OS_BIT(0)
#define MAX31343_F_TIMER_CONFIG_TRPT		NO_OS_BIT(2)
#define MAX31343_F_TIMER_CONFIG_TPAUSE		NO_OS_BIT(3)
#define MAX31343_F_TIMER_CONFIG_TE		NO_OS_BIT(4)

/* Power management register bits */
#define MAX31343_F_PWR_MGMT_DMAN_SEL		NO_OS_BIT(2)
#define MAX31343_F_PWR_MGMT_D_VBACK_SEL		NO_OS_BIT(3)
#define MAX31343_F_PWR_MGMT_PFVT		NO_OS_BIT(4)

/* Trickle register bits */
#define MAX31343_F_TRICKLE_D_TRICKLE		NO_OS_BIT(0)
#define MAX31343_F_TRICKLE_TCHE			NO_OS_BIT(4)

/* Temperature sensor config register bits */
#define MAX31343_F_TS_CONFIG_TTSINT		NO_OS_BIT(3)
#define MAX31343_F_TS_CONFIG_ONESHOT_MODE	NO_OS_BIT(6)
#define MAX31343_F_TS_CONFIG_AUTO_MODE		NO_OS_BIT(7)

/***************************************************************************//**
 * @brief The `max31343_time_stamp` structure is designed to hold date and time
 * information for the MAX31343 Real Time Clock. It includes fields for
 * seconds, minutes, hours, day of the month, month, and year, allowing
 * for a comprehensive representation of a specific point in time. This
 * structure is essential for setting and retrieving time data from the
 * RTC device.
 *
 * @param sec Seconds [0-61].
 * @param min Minutes [0-59].
 * @param hr Hours [0-23].
 * @param day Day of month [1-31].
 * @param mon Month [0-11].
 * @param year Year [2000-2199].
 ******************************************************************************/
struct max31343_time_stamp {
	uint8_t				sec;    /* Seconds              [0-61] */
	uint8_t				min;    /* Minutes              [0-59] */
	uint8_t				hr;     /* Hours                [0-23] */
	uint8_t				day;    /* Day of month         [1-31] */
	uint8_t				mon;    /* Month                [0-11] */
	uint8_t				year;   /* Year                 [2000-2199] */
};

/***************************************************************************//**
 * @brief The `max31343_init_param` structure is used to define the
 * initialization parameters for the MAX31343 real-time clock device. It
 * includes a pointer to an I2C initialization parameter structure, which
 * specifies the communication settings for interfacing with the device,
 * and a battery enable flag to indicate if the device's battery backup
 * feature should be activated. This structure is essential for setting
 * up the device correctly before use.
 *
 * @param i2c_init A pointer to a structure that describes the I2C communication
 * parameters for the device.
 * @param battery_en A uint8_t value indicating whether the battery is enabled
 * (1) or not (0).
 ******************************************************************************/
struct max31343_init_param {
	/** Device communication descriptor */
	struct no_os_i2c_init_param 	*i2c_init;
	uint8_t				battery_en;
};

/***************************************************************************//**
 * @brief The `max31343_dev` structure is used to represent a MAX31343 Real Time
 * Clock device in a software application. It contains a pointer to an
 * I2C descriptor, which is used to manage communication with the device
 * over the I2C bus, and a battery enable flag, which indicates whether
 * the device's battery backup is enabled. This structure is essential
 * for interfacing with the MAX31343 RTC, allowing for operations such as
 * reading and writing registers, setting and reading time stamps, and
 * managing device initialization and removal.
 *
 * @param i2c_desc A pointer to a structure that describes the I2C communication
 * interface for the device.
 * @param battery_en A uint8_t value indicating whether the battery is enabled
 * for the device.
 ******************************************************************************/
struct max31343_dev {
	/** Device communication descriptor */
	struct no_os_i2c_desc		*i2c_desc;
	uint8_t				battery_en;
};

/* Read device register. */
/***************************************************************************//**
 * @brief This function reads a single byte from a specified register of the
 * MAX31343 real-time clock device. It is typically used to retrieve
 * configuration or status information from the device. The function
 * requires a valid device structure and a register address to read from.
 * The caller must ensure that the device has been properly initialized
 * before calling this function. The function will return an error code
 * if the read operation fails, which can occur if the device is not
 * connected or if there is an I2C communication error.
 *
 * @param dev A pointer to a max31343_dev structure representing the device.
 * Must not be null and should be properly initialized before use.
 * @param reg_addr The address of the register to read from. Must be a valid
 * register address within the device's addressable range.
 * @param reg_data A pointer to a uint8_t variable where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during the read operation.
 ******************************************************************************/
int max31343_reg_read(struct max31343_dev *dev, uint8_t reg_addr,
		      uint8_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function writes a single byte of data to a specified register of
 * the MAX31343 real-time clock device. It is typically used to configure
 * the device or update its settings. The function requires a valid
 * device structure that has been properly initialized, and it assumes
 * that the I2C communication channel is correctly set up. It is
 * important to ensure that the register address is within the valid
 * range for the device. The function returns an integer status code
 * indicating the success or failure of the operation.
 *
 * @param dev A pointer to a max31343_dev structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null.
 * @param reg_addr The address of the register to write to. It should be a valid
 * register address within the range supported by the MAX31343
 * device.
 * @param reg_data The data byte to write to the specified register. This is the
 * value that will be stored in the register.
 * @return Returns an integer status code. A value of 0 typically indicates
 * success, while a negative value indicates an error occurred during
 * the write operation.
 ******************************************************************************/
int max31343_reg_write(struct max31343_dev *dev, uint8_t reg_addr,
		       uint8_t reg_data);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief This function is used to modify specific bits in a register of the
 * MAX31343 device. It reads the current value of the register, applies a
 * mask to clear the bits to be updated, and then sets the new bits as
 * specified by the reg_data parameter. This function should be called
 * when there is a need to change specific bits in a register without
 * affecting other bits. It requires a valid device structure and
 * register address, and it returns an error code if the read or write
 * operation fails.
 *
 * @param dev A pointer to a max31343_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the MAX31343 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, after applying
 * the mask. Only the bits specified by the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int max31343_update_bits(struct max31343_dev *dev, uint8_t reg_addr,
			 uint8_t mask, uint8_t reg_data);

/* Set time stemp */
/***************************************************************************//**
 * @brief This function sets the time and date on a MAX31343 real-time clock
 * (RTC) device using the provided timestamp structure. It should be
 * called when you need to update the RTC with a new time and date. The
 * function requires a valid device structure and a timestamp structure
 * with all fields properly initialized. The year field in the timestamp
 * must be within the range of 2000 to 2199. The function returns an
 * error code if any of the register writes fail, allowing the caller to
 * handle such errors appropriately.
 *
 * @param dev A pointer to a max31343_dev structure representing the RTC device.
 * Must not be null. The caller retains ownership.
 * @param ts A max31343_time_stamp structure containing the time and date to
 * set. The fields must be within their valid ranges: sec [0-61], min
 * [0-59], hr [0-23], day [1-31], mon [0-11], year [2000-2199].
 * @return Returns 0 on success or a negative error code if a register write
 * fails.
 ******************************************************************************/
int max31343_set_time_stamp(struct max31343_dev *dev,
			    struct max31343_time_stamp ts);

/* Read time stamp */
/***************************************************************************//**
 * @brief Use this function to retrieve the current time and date from a
 * MAX31343 real-time clock device. It reads the time stamp registers and
 * converts the values from BCD to binary format, storing them in the
 * provided `max31343_time_stamp` structure. This function should be
 * called when the current time needs to be accessed, and it requires a
 * properly initialized `max31343_dev` device structure. Ensure that the
 * `ts` parameter is a valid pointer to a `max31343_time_stamp` structure
 * where the time data will be stored. The function returns an error code
 * if the read operation fails.
 *
 * @param dev A pointer to an initialized `max31343_dev` structure representing
 * the device. Must not be null. The function will return an error if
 * the device is not properly initialized.
 * @param ts A pointer to a `max31343_time_stamp` structure where the read time
 * stamp will be stored. Must not be null. The structure will be
 * populated with the current time and date.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int max31343_reg_read_time_stamp(struct max31343_dev *dev,
				 struct max31343_time_stamp *ts);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes the MAX31343 real-time clock (RTC) device by
 * setting up the necessary I2C communication and configuring the device
 * for operation. It must be called before any other operations on the
 * device. The function allocates memory for the device structure and
 * initializes the I2C interface using the provided parameters. It also
 * configures the RTC by resetting it, enabling the oscillator, and
 * disabling interrupts. If any step fails, the function cleans up and
 * returns an error code.
 *
 * @param device A pointer to a pointer of type `struct max31343_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * The caller is responsible for freeing the allocated memory by
 * calling `max31343_remove`.
 * @param init_param A structure of type `struct max31343_init_param` containing
 * initialization parameters. This includes the I2C
 * initialization parameters and a flag to enable the battery.
 * The structure must be properly initialized before calling
 * the function.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * the type of error encountered during initialization.
 ******************************************************************************/
int max31343_init(struct max31343_dev **device,
		  struct max31343_init_param init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function is used to properly remove a MAX31343 device instance
 * and free any resources associated with it. It should be called when
 * the device is no longer needed, ensuring that any allocated memory and
 * I2C resources are released. This function must be called after the
 * device has been initialized and used, to prevent resource leaks. It is
 * important to ensure that the device pointer is valid and initialized
 * before calling this function.
 *
 * @param dev A pointer to a max31343_dev structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid, initialized device structure. If the pointer is invalid,
 * the behavior is undefined.
 * @return Returns 0 on successful removal and resource release. If an error
 * occurs during the I2C resource release, a non-zero error code is
 * returned.
 ******************************************************************************/
int max31343_remove(struct max31343_dev *dev);

#endif /* __MAX31343__ */
