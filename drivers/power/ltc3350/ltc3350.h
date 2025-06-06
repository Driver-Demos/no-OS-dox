/***************************************************************************//**
*   @file   ltc3350.c
*   @brief  Implementation of LTC3350 Driver
*   @authors Sankalp Khandkar (sankalp.khandkar@analog.com)
*            Ignacio Rojas (ignacio.rojas@analog.com)
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

#ifndef __LTC3350_H__
#define __LTC3350_H__

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

/* LTC3350 Register Map */
#define LTC3350_AD_CLR_ALARMS                   0x00
#define LTC3350_AD_MSK_ALARMS                   0x01
#define LTC3350_AD_MSK_MON_STATUS               0x02
#define LTC3350_AD_CAP_ESR_PER                  0x04
#define LTC3350_AD_VCAPFB_DAC                   0x05
#define LTC3350_AD_VSHUNT                       0x06
#define LTC3350_AD_CAP_UV_LVL                   0x07
#define LTC3350_AD_CAP_OV_LVL                   0x08
#define LTC3350_AD_GPI_UV_LVL                   0x09
#define LTC3350_AD_GPI_OV_LVL                   0x0A
#define LTC3350_AD_VIN_UV_LVL                   0x0B
#define LTC3350_AD_VIN_OV_LVL                   0x0C
#define LTC3350_AD_VCAP_UV_LVL                  0x0D
#define LTC3350_AD_VCAP_OV_LVL                  0x0E
#define LTC3350_AD_VOUT_UV_LVL                  0x0F
#define LTC3350_AD_VOUT_OV_LVL                  0x10
#define LTC3350_AD_IIN_OC_LVL                   0x11
#define LTC3350_AD_ICHG_UC_LVL                  0x12
#define LTC3350_AD_DTEMP_COLD_LVL               0x13
#define LTC3350_AD_DTEMP_HOT_LVL                0x14
#define LTC3350_AD_ESR_HI_LVL                   0x15
#define LTC3350_AD_CAP_LO_LVL                   0x16
#define LTC3350_AD_CTL_REG                      0x17
#define LTC3350_AD_NUM_CAPS                     0x1A
#define LTC3350_AD_CHRG_STATUS                  0x1B
#define LTC3350_AD_MON_STATUS                   0x1C
#define LTC3350_AD_ALARM_REG                    0x1D
#define LTC3350_AD_MEAS_CAP                     0x1E
#define LTC3350_AD_MEAS_ESR                     0x1F
#define LTC3350_AD_MEAS_VCAP1                   0x20
#define LTC3350_AD_MEAS_VCAP2                   0x21
#define LTC3350_AD_MEAS_VCAP3                   0x22
#define LTC3350_AD_MEAS_VCAP4                   0x23
#define LTC3350_AD_MEAS_GPI                     0x24
#define LTC3350_AD_MEAS_VIN                     0x25
#define LTC3350_AD_MEAS_VCAP                    0x26
#define LTC3350_AD_MEAS_VOUT                    0x27
#define LTC3350_AD_MEAS_IIN                     0x28
#define LTC3350_AD_MEAS_ICHG                    0x29
#define LTC3350_AD_MEAS_DTEMP                   0x2A

/******************************************************************************/
/******************* Driver pre-compile time settings *************************/
/******************************************************************************/

/**
 * @brief   LTC3350 measurements configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p TRUE.
 */
#ifndef LTC3350_USE_MEASUREMENTS
#define LTC3350_USE_MEASUREMENTS               	1
#endif

/**
 * @brief   LTC3350 alarms configuration switch.
 * @details If set to @p TRUE more configurations are available.
 * @note    The default is @p FALSE.
 */
#ifndef LTC3350_USE_ALARMS
#define LTC3350_USE_ALARMS               	0
#endif

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/*LTC3350 alarms*/
/***************************************************************************//**
 * @brief The `ltc3350_enum_alarms` is an enumeration that defines various alarm
 * conditions for the LTC3350 device, each associated with a specific
 * type of voltage, current, or temperature threshold. These alarms are
 * used to monitor and respond to different operational conditions, such
 * as under-voltage, over-voltage, over-current, and temperature
 * extremes, ensuring the device operates within safe parameters.
 *
 * @param LTC3350_CAP_UV_LVL Represents the capacitor under-voltage level alarm.
 * @param LTC3350_CAP_OV_LVL Represents the capacitor over-voltage level alarm.
 * @param LTC3350_GPI_UV_LVL Represents the general-purpose input under-voltage
 * level alarm.
 * @param LTC3350_GPI_OV_LVL Represents the general-purpose input over-voltage
 * level alarm.
 * @param LTC3350_VIN_UV_LVL Represents the input voltage under-voltage level
 * alarm.
 * @param LTC3350_VIN_OV_LVL Represents the input voltage over-voltage level
 * alarm.
 * @param LTC3350_VCAP_UV_LVL Represents the capacitor voltage under-voltage
 * level alarm.
 * @param LTC3350_VCAP_OV_LVL Represents the capacitor voltage over-voltage
 * level alarm.
 * @param LTC3350_VOUT_UV_LVL Represents the output voltage under-voltage level
 * alarm.
 * @param LTC3350_VOUT_OV_LVL Represents the output voltage over-voltage level
 * alarm.
 * @param LTC3350_IIN_OC_LVL Represents the input current over-current level
 * alarm.
 * @param LTC3350_ICHG_UC_LVL Represents the charge current under-current level
 * alarm.
 * @param LTC3350_DTEMP_COLD_LVL Represents the device temperature cold level
 * alarm.
 * @param LTC3350_DTEMP_HOT_LVL Represents the device temperature hot level
 * alarm.
 * @param LTC3350_ESR_HI_LVL Represents the equivalent series resistance high
 * level alarm.
 * @param LTC3350_CAP_LO_LVL Represents the capacitor low level alarm.
 ******************************************************************************/
enum ltc3350_enum_alarms {
	LTC3350_CAP_UV_LVL            = 0,
	LTC3350_CAP_OV_LVL            = 1,
	LTC3350_GPI_UV_LVL            = 2,
	LTC3350_GPI_OV_LVL            = 3,
	LTC3350_VIN_UV_LVL            = 4,
	LTC3350_VIN_OV_LVL            = 5,
	LTC3350_VCAP_UV_LVL           = 6,
	LTC3350_VCAP_OV_LVL           = 7,
	LTC3350_VOUT_UV_LVL           = 8,
	LTC3350_VOUT_OV_LVL           = 9,
	LTC3350_IIN_OC_LVL            = 10,
	LTC3350_ICHG_UC_LVL           = 11,
	LTC3350_DTEMP_COLD_LVL        = 12,
	LTC3350_DTEMP_HOT_LVL         = 13,
	LTC3350_ESR_HI_LVL            = 14,
	LTC3350_CAP_LO_LVL            = 15
};

/*LTC3350 alarms_mask*/
/***************************************************************************//**
 * @brief The `ltc3350_enum_alarms_mask` is an enumeration that defines bit
 * masks for various alarm conditions related to the LTC3350 device. Each
 * enumerator represents a specific alarm condition, such as under-
 * voltage, over-voltage, over-current, or temperature thresholds, and is
 * associated with a bit position using the `NO_OS_BIT` macro. This
 * enumeration is used to manage and identify different alarm states in
 * the LTC3350 power management system.
 *
 * @param LTC3350_CAP_UV_LVL_B_BIT Represents the bit mask for capacitor under-
 * voltage level B.
 * @param LTC3350_CAP_OV_LVL_BIT Represents the bit mask for capacitor over-
 * voltage level.
 * @param LTC3350_GPI_UV_LVL_BIT Represents the bit mask for general-purpose
 * input under-voltage level.
 * @param LTC3350_GPI_OV_LVL_BIT Represents the bit mask for general-purpose
 * input over-voltage level.
 * @param LTC3350_VIN_UV_LVL_BIT Represents the bit mask for input voltage
 * under-voltage level.
 * @param LTC3350_VIN_OV_LVL_BIT Represents the bit mask for input voltage over-
 * voltage level.
 * @param LTC3350_VCAP_UV_LVL_BIT Represents the bit mask for capacitor voltage
 * under-voltage level.
 * @param LTC3350_VCAP_OV_LVL_BIT Represents the bit mask for capacitor voltage
 * over-voltage level.
 * @param LTC3350_VOUT_UV_LVL_BIT Represents the bit mask for output voltage
 * under-voltage level.
 * @param LTC3350_VOUT_OV_LVL_BIT Represents the bit mask for output voltage
 * over-voltage level.
 * @param LTC3350_IIN_OC_LVL_BIT Represents the bit mask for input current over-
 * current level.
 * @param LTC3350_ICHG_UC_LVL_BIT Represents the bit mask for charge current
 * under-current level.
 * @param LTC3350_DTEMP_COLD_LVL_BIT Represents the bit mask for device
 * temperature cold level.
 * @param LTC3350_DTEMP_HOT_LVL_BIT Represents the bit mask for device
 * temperature hot level.
 * @param LTC3350_ESR_HI_LVL_BIT Represents the bit mask for equivalent series
 * resistance high level.
 * @param LTC3350_CAP_LO_LVL_BIT Represents the bit mask for capacitor low
 * level.
 ******************************************************************************/
enum ltc3350_enum_alarms_mask {
	LTC3350_CAP_UV_LVL_B_BIT      = NO_OS_BIT(0),
	LTC3350_CAP_OV_LVL_BIT        = NO_OS_BIT(1),
	LTC3350_GPI_UV_LVL_BIT        = NO_OS_BIT(2),
	LTC3350_GPI_OV_LVL_BIT        = NO_OS_BIT(3),
	LTC3350_VIN_UV_LVL_BIT        = NO_OS_BIT(4),
	LTC3350_VIN_OV_LVL_BIT        = NO_OS_BIT(5),
	LTC3350_VCAP_UV_LVL_BIT       = NO_OS_BIT(6),
	LTC3350_VCAP_OV_LVL_BIT       = NO_OS_BIT(7),
	LTC3350_VOUT_UV_LVL_BIT       = NO_OS_BIT(8),
	LTC3350_VOUT_OV_LVL_BIT       = NO_OS_BIT(9),
	LTC3350_IIN_OC_LVL_BIT        = NO_OS_BIT(10),
	LTC3350_ICHG_UC_LVL_BIT       = NO_OS_BIT(11),
	LTC3350_DTEMP_COLD_LVL_BIT    = NO_OS_BIT(12),
	LTC3350_DTEMP_HOT_LVL_BIT     = NO_OS_BIT(13),
	LTC3350_ESR_HI_LVL_BIT        = NO_OS_BIT(14),
	LTC3350_CAP_LO_LVL_BIT        = NO_OS_BIT(15)
};

/***************************************************************************//**
 * @brief The `ltc3350_init_param` structure is designed to encapsulate the
 * initialization parameters required for setting up the LTC3350 device,
 * specifically focusing on the I2C communication interface. It contains
 * a single member, `i2c_init`, which is a structure that provides the
 * necessary configuration details for initializing the I2C communication
 * channel, ensuring that the device can be properly interfaced with
 * through I2C.
 *
 * @param i2c_init This member is a structure that holds the initialization
 * parameters for I2C communication.
 ******************************************************************************/
struct ltc3350_init_param {
	/** Device Communication initialization structure I2C */
	struct no_os_i2c_init_param i2c_init;
};

/***************************************************************************//**
 * @brief The `ltc3350_dev` structure is designed to encapsulate the necessary
 * information for managing the I2C communication with an LTC3350 device.
 * It contains a single member, `i2c_desc`, which is a pointer to an
 * `no_os_i2c_desc` structure that holds the details required for I2C
 * communication. This structure is essential for initializing and
 * interacting with the LTC3350 device through its driver functions.
 *
 * @param i2c_desc A pointer to a structure that describes the I2C communication
 * interface for the device.
 ******************************************************************************/
struct ltc3350_dev {
	/** Device I2C Communication */
	struct no_os_i2c_desc *i2c_desc;
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! NO OS Requirements */
/***************************************************************************//**
 * @brief This function initializes an LTC3350 device by allocating memory for
 * the device structure and setting up the I2C communication based on the
 * provided initialization parameters. It must be called before any other
 * operations on the device. If the initialization fails, the function
 * returns an error code and no device instance is created. Ensure that
 * the provided initialization parameters are valid and that the function
 * is called in an environment where memory allocation and I2C
 * initialization can succeed.
 *
 * @param device A pointer to a pointer where the initialized device instance
 * will be stored. Must not be null. On success, the caller takes
 * ownership of the allocated device structure and is responsible
 * for freeing it using ltc3350_remove().
 * @param init_param A pointer to an ltc3350_init_param structure containing the
 * initialization parameters for the device. Must not be null
 * and must be properly initialized with valid I2C parameters.
 * @return Returns 0 on success, or a negative error code on failure. On
 * success, the device pointer is set to point to the newly allocated
 * and initialized device structure.
 ******************************************************************************/
int ltc3350_init(struct ltc3350_dev **device,
		 struct ltc3350_init_param *init_param);
/***************************************************************************//**
 * @brief Use this function to properly release all resources associated with an
 * LTC3350 device when it is no longer needed. This function should be
 * called after the device has been initialized and used, to ensure that
 * all allocated resources are freed and any open connections are closed.
 * It is important to call this function to prevent resource leaks. The
 * function will handle any errors that occur during the resource release
 * process and return an appropriate error code if necessary.
 *
 * @param dev A pointer to an ltc3350_dev structure representing the device to
 * be removed. This pointer must not be null and should point to a
 * valid, initialized device structure. The function will handle the
 * deallocation of resources associated with this device.
 * @return Returns 0 on success, or a negative error code if an error occurs
 * during the removal process.
 ******************************************************************************/
int ltc3350_remove(struct ltc3350_dev *dev);

/*! General functions */
/***************************************************************************//**
 * @brief This function reads data from a specified register of the LTC3350
 * device using the I2C interface. It should be called after the device
 * has been properly initialized with `ltc3350_init`. The function writes
 * the base address to the device and then reads two bytes of data from
 * the specified register, storing the result in the provided `read_data`
 * pointer. It is important to ensure that the `dev` and `read_data`
 * pointers are valid and not null before calling this function. The
 * function returns an error code if the I2C communication fails.
 *
 * @param dev A pointer to an initialized `ltc3350_dev` structure representing
 * the device. Must not be null.
 * @param base_address The 8-bit register address from which to read data. Must
 * be a valid register address for the LTC3350.
 * @param read_data A pointer to a `uint16_t` where the read data will be
 * stored. Must not be null.
 * @return Returns 0 on success or a negative error code if the I2C
 * communication fails.
 ******************************************************************************/
int ltc3350_read_device_data(struct ltc3350_dev *dev, uint8_t base_address,
			     uint16_t *read_data);
/***************************************************************************//**
 * @brief Use this function to write a 16-bit data value to a specific register
 * of the LTC3350 device over the I2C interface. This function should be
 * called after the device has been properly initialized using the
 * ltc3350_init function. It is important to ensure that the base address
 * provided corresponds to a valid register in the LTC3350 device. The
 * function returns an integer status code indicating the success or
 * failure of the I2C write operation.
 *
 * @param dev A pointer to an ltc3350_dev structure representing the device.
 * This must be initialized and must not be null.
 * @param base_address An 8-bit unsigned integer representing the base address
 * of the register to write to. It should correspond to a
 * valid register address in the LTC3350 device.
 * @param write_data A 16-bit unsigned integer containing the data to be written
 * to the specified register.
 * @return Returns an integer status code from the I2C write operation, where 0
 * typically indicates success and a negative value indicates an error.
 ******************************************************************************/
int ltc3350_write_device_data(struct ltc3350_dev *dev, uint8_t base_address,
			      uint16_t write_data);

/*! LTC3350 functions */

/* LTC3350_USE_MEASUREMENTS == 1 */

#if LTC3350_USE_MEASUREMENTS

/***************************************************************************//**
 * @brief Use this function to obtain the number of capacitors configured in an
 * LTC3350 device. It must be called after the device has been properly
 * initialized using `ltc3350_init`. The function reads the relevant data
 * from the device and outputs the number of capacitors. Ensure that the
 * `dev` parameter is a valid pointer to an initialized `ltc3350_dev`
 * structure, and `num_caps` is a valid pointer where the result will be
 * stored. The function returns an error code if the read operation
 * fails.
 *
 * @param dev A pointer to an initialized `ltc3350_dev` structure representing
 * the device. Must not be null.
 * @param num_caps A pointer to a `uint8_t` where the number of capacitors will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails. On success, `num_caps` is set to the number of capacitors
 * configured.
 ******************************************************************************/
int ltc3350_get_num_caps(struct ltc3350_dev *dev, uint8_t *num_caps);

/***************************************************************************//**
 * @brief Use this function to obtain the equivalent series resistance (ESR)
 * measurement from an LTC3350 device. It is essential to ensure that the
 * device has been properly initialized before calling this function. The
 * function reads the ESR value and stores it in the provided memory
 * location. This function is useful for monitoring the health and
 * performance of the capacitors connected to the LTC3350. It is
 * important to handle the return value to check for any errors during
 * the read operation.
 *
 * @param dev A pointer to an initialized ltc3350_dev structure representing the
 * device. Must not be null.
 * @param value A pointer to a uint16_t variable where the ESR value will be
 * stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc3350_get_esr(struct ltc3350_dev *dev, uint16_t *value);
/***************************************************************************//**
 * @brief This function retrieves the voltage measurement data for a specified
 * capacitor from the LTC3350 device. It should be called after the
 * device has been properly initialized using `ltc3350_init`. The
 * function can read data for a specific capacitor by specifying its
 * index or for a generic measurement by passing zero. It is important to
 * ensure that the capacitor index is within the valid range to avoid
 * errors.
 *
 * @param dev A pointer to an initialized `ltc3350_dev` structure representing
 * the device. Must not be null.
 * @param n_cap An unsigned 8-bit integer specifying the capacitor index to
 * read. Valid values are 0 for a generic measurement or 1 to 4 for
 * specific capacitors. Values greater than 4 will result in an
 * error.
 * @param value A pointer to a 16-bit unsigned integer where the read voltage
 * data will be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the capacitor index
 * is invalid or if the read operation fails.
 ******************************************************************************/
int ltc3350_get_vcap(struct ltc3350_dev *dev, uint8_t n_cap, uint16_t *value);

#endif

/* LTC3350_USE_ALARMS == 0 */

#if LTC3350_USE_ALARMS

/***************************************************************************//**
 * @brief This function configures a specific alarm on the LTC3350 device by
 * setting its threshold value. It should be called when you need to
 * monitor specific conditions on the device and trigger alarms based on
 * those conditions. The function requires the device to be initialized
 * before use. It modifies the alarm mask to include the specified alarm
 * and writes the threshold value to the appropriate register. Ensure
 * that the device is properly communicating over I2C, as the function
 * involves reading from and writing to device registers.
 *
 * @param dev A pointer to an initialized ltc3350_dev structure representing the
 * device. Must not be null.
 * @param alarm An enumerated value of type ltc3350_enum_alarms specifying which
 * alarm to set. Must be a valid enum value.
 * @param alarm_threshold A 16-bit unsigned integer representing the threshold
 * value for the specified alarm. The valid range depends
 * on the specific alarm being configured.
 * @return Returns an integer status code: 0 on success, or a negative error
 * code if the operation fails.
 ******************************************************************************/
int ltc3350_set_alarm(struct ltc3350_dev *dev, enum ltc3350_enum_alarms alarm,
		      uint16_t alarm_threshold);
/***************************************************************************//**
 * @brief Use this function to mute a specific alarm on the LTC3350 device,
 * which prevents the alarm from triggering further notifications. This
 * function should be called when an alarm condition is acknowledged and
 * further alerts are not desired. Ensure that the device has been
 * properly initialized before calling this function. The function reads
 * the current alarm mask, modifies it to mute the specified alarm, and
 * writes the updated mask back to the device. It returns an error code
 * if the operation fails at any step.
 *
 * @param dev A pointer to an initialized ltc3350_dev structure representing the
 * device. Must not be null. The caller retains ownership.
 * @param alarm An enumerated value of type ltc3350_enum_alarms representing the
 * specific alarm to mute. Must be a valid enum value.
 * @return Returns 0 on success or a negative error code if the operation fails.
 ******************************************************************************/
int ltc3350_mute_alarm(struct ltc3350_dev *dev, enum ltc3350_enum_alarms alarm);
/***************************************************************************//**
 * @brief Use this function to obtain the current alarm status mask from an
 * LTC3350 device. This function should be called when you need to check
 * which alarms are currently active on the device. Ensure that the
 * device has been properly initialized before calling this function. The
 * function reads the alarm status register and stores the result in the
 * provided memory location. It returns an error code if the read
 * operation fails.
 *
 * @param dev A pointer to an initialized `ltc3350_dev` structure representing
 * the device. Must not be null.
 * @param alarm_mask A pointer to a `uint16_t` where the alarm status mask will
 * be stored. Must not be null.
 * @return Returns 0 on success, or a negative error code if the read operation
 * fails.
 ******************************************************************************/
int ltc3350_get_alarm_status_mask(struct ltc3350_dev *dev,
				  uint16_t *alarm_mask);
/***************************************************************************//**
 * @brief Use this function to reset the alarm status mask of an LTC3350 device,
 * effectively clearing any active alarms. This function should be called
 * when you want to acknowledge and clear all current alarm statuses.
 * Ensure that the device has been properly initialized before calling
 * this function. The function communicates with the device over I2C to
 * perform the operation.
 *
 * @param dev A pointer to an initialized `ltc3350_dev` structure representing
 * the LTC3350 device. This parameter must not be null, and the
 * device must be properly initialized before calling this function.
 * @return Returns 0 on success or a negative error code on failure, indicating
 * an issue with the I2C communication or device interaction.
 ******************************************************************************/
int ltc3350_clear_alarm_status_mask(struct ltc3350_dev *dev);

#endif

#endif /* __LTC3350_H__ */
