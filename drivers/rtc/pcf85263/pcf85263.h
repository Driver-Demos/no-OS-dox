/***************************************************************************//**
 *   @file   pcf85263.h
 *   @brief  Header file of pcf85263 Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
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

#ifndef __PCF85263_H__
#define __PCF85263_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "no_os_util.h"
#include "no_os_i2c.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define PCF85263_REG_100TH_SECONDS		0x00
#define PCF85263_REG_SECONDS			0x01
#define PCF85263_REG_MINUTES			0x02
#define PCF85263_REG_HOURS			0x03
#define PCF85263_REG_DAYS			0x04
#define PCF85263_REG_WEEKDAYS			0x05
#define PCF85263_REG_MONTHS			0x06
#define PCF85263_REG_YEARS			0x07
#define PCF85263_REG_SECOND_ALARM1		0x08
#define PCF85263_REG_MINUTE_ALARM1		0x09
#define PCF85263_REG_HOUR_ALARM1		0x0A
#define PCF85263_REG_DAY_ALARM1			0x0B
#define PCF85263_REG_MONTH_ALARM1		0x0C
#define PCF85263_REG_MINUTE_ALARM2		0x0D
#define PCF85263_REG_HOUR_ALARM2		0x0E
#define PCF85263_REG_WEEKDAY_ALARM2		0x0F
#define PCF85263_REG_ALARM_ENABLES		0x10
#define PCF85263_REG_TSR1_SECONDS		0x11
#define PCF85263_REG_TSR1_MINUTES		0x12
#define PCF85263_REG_TSR1_HOURS			0x13
#define PCF85263_REG_TSR1_DAYS			0x14
#define PCF85263_REG_TSR1_MONTHS		0x15
#define PCF85263_REG_TSR1_YEARS			0x16
#define PCF85263_REG_TSR2_SECONDS		0x17
#define PCF85263_REG_TSR2_MINUTES		0x18
#define PCF85263_REG_TSR2_HOURS			0x19
#define PCF85263_REG_TSR2_DAYS			0x1A
#define PCF85263_REG_TSR2_MONTHS		0x1B
#define PCF85263_REG_TSR2_YEARS			0x1C
#define PCF85263_REG_TSR3_SECONDS		0x1D
#define PCF85263_REG_TSR3_MINUTES		0x1E
#define PCF85263_REG_TSR3_HOURS			0x1F
#define PCF85263_REG_TSR3_DAYS			0x20
#define PCF85263_REG_TSR3_MONTHS		0x21
#define PCF85263_REG_TSR3_YEARS			0x22
#define PCF85263_REG_TSR_MODE			0x23
#define PCF85263_REG_OFFSET			0x24
#define PCF85263_REG_OSCILLATOR			0x25
#define PCF85263_REG_BATTERY_SWITCH		0x26
#define PCF85263_REG_PIN_IO			0x27
#define PCF85263_REG_FUNCTION			0x28
#define PCF85263_REG_INTA_ENABLE		0x29
#define PCF85263_REG_INTB_ENABLE		0x2A
#define PCF85263_REG_FLAGS			0x2B
#define PCF85263_REG_RAM_BYTE			0x2C
#define PCF85263_REG_WATCH_DOG			0x2D
#define PCF85263_REG_STOP_ENABLE		0x2E
#define PCF85263_REG_RESETS			0x2F

#define PCF85263_CPR				0xA4
#define PCF85263_BATTERY_SW_MSK			NO_OS_BIT(4)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/***************************************************************************//**
 * @brief The `pcf85263_date` structure is designed to hold date and time
 * information for the PCF85263 real-time clock device. It consists of
 * six fields, each represented as an 8-bit unsigned integer, to store
 * the seconds, minutes, hours, day, month, and year. This structure is
 * used to manage and manipulate date and time data within the context of
 * the PCF85263 device operations.
 *
 * @param sec Stores the seconds component of the date.
 * @param min Stores the minutes component of the date.
 * @param hr Stores the hours component of the date.
 * @param day Stores the day component of the date.
 * @param mon Stores the month component of the date.
 * @param year Stores the year component of the date.
 ******************************************************************************/
struct pcf85263_date {
	uint8_t				sec;
	uint8_t				min;
	uint8_t				hr;
	uint8_t				day;
	uint8_t				mon;
	uint8_t				year;
};

/***************************************************************************//**
 * @brief The `pcf85263_init_param` structure is used to define the
 * initialization parameters for the PCF85263 device, which is a real-
 * time clock (RTC) module. It includes a pointer to an I2C
 * initialization parameter structure, which is essential for setting up
 * the communication interface with the device, and a battery enable flag
 * that determines if the battery backup feature is activated. This
 * structure is crucial for configuring the device before it is used in
 * an application.
 *
 * @param i2c_init A pointer to a structure that holds the I2C initialization
 * parameters for device communication.
 * @param battery_en A uint8_t value indicating whether the battery is enabled
 * (1) or not (0).
 ******************************************************************************/
struct pcf85263_init_param {
	/** Device communication descriptor */
	struct no_os_i2c_init_param 	*i2c_init;
	uint8_t				battery_en;
};

/***************************************************************************//**
 * @brief The `pcf85263_dev` structure is used to represent a PCF85263 device,
 * encapsulating the necessary information for communication and
 * configuration. It includes a pointer to an I2C descriptor for handling
 * communication with the device and a flag to indicate the status of the
 * battery enable feature. This structure is essential for managing the
 * device's operations, such as reading and writing registers, setting
 * dates, and handling initialization and removal processes.
 *
 * @param i2c_desc A pointer to a structure that describes the I2C communication
 * interface for the device.
 * @param battery_en A uint8_t value indicating whether the battery is enabled
 * for the device.
 ******************************************************************************/
struct pcf85263_dev {
	/** Device communication descriptor */
	struct no_os_i2c_desc		*i2c_desc;
	uint8_t				battery_en;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/* Read device register. */
/***************************************************************************//**
 * @brief Use this function to read a single byte from a specified register of
 * the PCF85263 device. It is essential to ensure that the device has
 * been properly initialized before calling this function. The function
 * communicates with the device over I2C, writing the register address
 * and then reading the data from that register. This function is useful
 * for retrieving configuration or status information from the device.
 * Handle the return value to check for successful communication or
 * errors.
 *
 * @param dev A pointer to a `pcf85263_dev` structure representing the device.
 * This must be a valid, initialized device structure and must not be
 * null.
 * @param reg_addr The address of the register to read from. It should be a
 * valid register address as defined by the device's register
 * map.
 * @param reg_data A pointer to a uint8_t where the read data will be stored.
 * This must not be null, and the caller is responsible for
 * providing a valid memory location.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int pcf85263_read(struct pcf85263_dev *dev, uint8_t reg_addr,
		  uint8_t *reg_data);

/* Write device register. */
/***************************************************************************//**
 * @brief This function writes a single byte of data to a specified register on
 * the PCF85263 device. It is typically used to configure the device or
 * update its settings. The function requires a valid device structure
 * that has been properly initialized, and it assumes that the I2C
 * communication channel is correctly set up. The function returns an
 * integer status code indicating the success or failure of the write
 * operation.
 *
 * @param dev A pointer to a `pcf85263_dev` structure representing the device.
 * This must be a valid, initialized device structure, and must not
 * be null.
 * @param reg_addr The address of the register to which the data will be
 * written. This is an 8-bit unsigned integer, and it should
 * correspond to a valid register address on the PCF85263
 * device.
 * @param reg_data The data byte to be written to the specified register. This
 * is an 8-bit unsigned integer.
 * @return Returns an integer status code: 0 for success, or a negative error
 * code if the write operation fails.
 ******************************************************************************/
int pcf85263_write(struct pcf85263_dev *dev, uint8_t reg_addr,
		   uint8_t reg_data);

/* Update specific register bits. */
/***************************************************************************//**
 * @brief This function updates specific bits in a register of the PCF85263
 * device by first reading the current register value, applying a mask to
 * clear the bits to be updated, and then setting the new bits as
 * specified by the reg_data parameter. It is typically used when only
 * certain bits of a register need to be modified without affecting the
 * other bits. The function must be called with a valid device structure
 * and a valid register address. It returns an error code if the read or
 * write operation fails.
 *
 * @param dev A pointer to a pcf85263_dev structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param reg_addr The address of the register to be updated. Must be a valid
 * register address for the PCF85263 device.
 * @param mask A bitmask indicating which bits in the register should be
 * updated. Bits set to 1 in the mask will be affected.
 * @param reg_data The new data to be written to the register, after applying
 * the mask. Only the bits specified by the mask will be
 * updated.
 * @return Returns 0 on success or a negative error code if the read or write
 * operation fails.
 ******************************************************************************/
int pcf85263_update_bits(struct pcf85263_dev *dev, uint8_t reg_addr,
			 uint8_t mask, uint8_t reg_data);

/* Set date */
/***************************************************************************//**
 * @brief This function sets the date on a PCF85263 device by writing the
 * specified date values to the appropriate registers. It should be
 * called when the date needs to be updated on the device. The function
 * requires a valid device structure and a date structure containing the
 * desired date values. It temporarily stops the device to perform the
 * update and resumes it afterward. Ensure the device is properly
 * initialized before calling this function.
 *
 * @param dev A pointer to a `pcf85263_dev` structure representing the device.
 * Must not be null. The caller retains ownership.
 * @param date A `pcf85263_date` structure containing the date values to set.
 * Each field (sec, min, hr, day, mon, year) should be within valid
 * ranges for time and date representation. Invalid values may
 * result in incorrect date settings.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int pcf85263_set_date(struct pcf85263_dev *dev, struct pcf85263_date date);

/* Read time stamp */
/***************************************************************************//**
 * @brief Use this function to retrieve the current date and time from a
 * PCF85263 real-time clock device. It reads the time and date registers
 * from the device and converts the values from BCD to binary format.
 * This function should be called when the current timestamp is needed,
 * and it requires a valid device structure. Ensure that the device has
 * been properly initialized before calling this function. The function
 * will return an error code if the read operation fails at any point.
 *
 * @param dev A pointer to a `pcf85263_dev` structure representing the device.
 * This must be a valid, initialized device structure. The caller
 * retains ownership and must ensure it is not null.
 * @param ts A pointer to a `pcf85263_date` structure where the timestamp will
 * be stored. This must not be null, and the caller is responsible for
 * allocating and managing the memory for this structure.
 * @return Returns 0 on success, or a negative error code if any of the read
 * operations fail.
 ******************************************************************************/
int pcf85263_read_ts(struct pcf85263_dev *dev, struct pcf85263_date *ts);

/* Initialize the device. */
/***************************************************************************//**
 * @brief This function initializes a PCF85263 device using the provided
 * initialization parameters, setting up the necessary I2C communication
 * and configuring the device's battery settings. It must be called
 * before any other operations on the device to ensure proper setup. The
 * function allocates memory for the device structure and initializes the
 * I2C interface. If initialization fails at any step, it cleans up any
 * allocated resources and returns an error code. The caller is
 * responsible for managing the memory of the device structure and must
 * call the appropriate removal function to release resources when the
 * device is no longer needed.
 *
 * @param device A pointer to a pointer of a pcf85263_dev structure. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pcf85263_init_param structure containing initialization
 * parameters, including I2C initialization settings and
 * battery enable flag. The structure must be properly
 * populated before calling the function.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error, and no device
 * structure is allocated.
 ******************************************************************************/
int pcf85263_init(struct pcf85263_dev **device,
		  struct pcf85263_init_param init_param);

/* Remove the device and release resources. */
/***************************************************************************//**
 * @brief This function is used to properly shut down and clean up resources
 * associated with a PCF85263 device. It should be called when the device
 * is no longer needed, to ensure that all allocated resources are
 * released and any ongoing communications are terminated. This function
 * must be called after the device has been initialized and used, and it
 * is important to ensure that no other operations are performed on the
 * device after this function is called.
 *
 * @param dev A pointer to a `pcf85263_dev` structure representing the device to
 * be removed. This pointer must not be null, and it should point to
 * a valid device structure that was previously initialized. If the
 * pointer is invalid, the behavior is undefined.
 * @return Returns 0 on success, or a negative error code if the removal process
 * fails.
 ******************************************************************************/
int pcf85263_remove(struct pcf85263_dev *dev);

#endif //__PCF85263_H__
