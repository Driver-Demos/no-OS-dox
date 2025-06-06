/***************************************************************************//**
 *   @file   ltc3337.h
 *   @brief  Header file for the LTC3337 Driver
 *   @author Brent Kowal (brent.kowal@analog.com)
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
#ifndef __LTC3337_H__
#define __LTC3337_H__

#include <stdint.h>
#include "no_os_util.h"
#include "no_os_i2c.h"

#define LTC3337_I2C_ADDR			0x64 // b1100100[r/w] 0xC8, 0xC9

#define LTC3337_MAX_TEMP_C			159  //Maximum temperature for alarms
#define LTC3337_MIN_TEMP_C			-41  //Minimum temperature for alarms
#define LTC3337_MAX_PRESCALE		15   //Maximum prescaler value

#define LTC3337_OVERFLOW_ALARM_MAX	0xFF //Max value for overflow alarm

#define LTC3337_REG_A	0x01 //Prescaler Select, IRQ Clear, Shutdown, Alarm Thresh

#define LTC3337_RA_PRESCALE_MSK		NO_OS_GENMASK(3, 0)
#define LTC3337_RA_CLEAR_INT		NO_OS_BIT(4)
#define LTC3337_RA_CNT_CHK		NO_OS_BIT(5)
#define LTC3337_RA_SHTDN		NO_OS_BIT(6)
#define LTC3337_RA_ADC_CONV		NO_OS_BIT(7)
#define LTC3337_RA_ALARM_LVL_MSK	NO_OS_GENMASK(15, 8)

#define LTC3337_REG_B	0x02 //Accumulated Charge

#define LTC3337_REG_C	0x03 //Status, Temperature
//Use the following definitions for derterming interrupt status
#define LTC3337_RC_OVERFLOW			NO_OS_BIT(0)
#define LTC3337_RC_ALARM_TRIP		NO_OS_BIT(1)
#define LTC3337_RC_ALARM_MIN		NO_OS_BIT(2)
#define LTC3337_RC_ALARM_MAX		NO_OS_BIT(3)
#define LTC3337_RC_ADC_READY		NO_OS_BIT(4)
#define LTC3337_RC_IPK_PIN_MSK		NO_OS_GENMASK(7, 5)
#define LTC3337_RC_DIE_TEMP_MSK		NO_OS_GENMASK(15, 8)

#define LTC3337_REG_D	0x04 //BAT_IN V, Ipeak On
#define LTC3337_REG_E	0x05 //BAT_IN V, Ipeak Off
#define LTC3337_REG_F	0x06 //BAT_OUT V, Ipeak On
#define LTC3337_REG_G	0x07 //BAT_OUT V, Ipeak Off
#define LTC3337_BATV_MSK		NO_OS_GENMASK(11, 0)

#define LTC3337_REG_H	0x08 //Temp Alarm Config
#define LTC3337_RH_HOT_ALARM_MSK	NO_OS_GENMASK(15, 8)
#define LTC3337_RH_COLD_ALARM_MSK	NO_OS_GENMASK(7, 0)

#define LTC3337_VLSB_UV		1465 //1.465 mV = 1465 uV
#define LTC3337_TLSB_MC		784  //0.784 Deg C = 784 mC
#define LTC3337_TEMP_MIN_C	-41

#define LTC3337_NUM_IPEAKS	8 //3-bit iPeak Input

//Scale values used in integer match to calulate A/nA-hrs from counters
#define LTC3337_NANO_AMP	1000000000
#define LTC3337_CALC_SCALE	10000
#define LTC3337_CALC_TO_WHOLE	(LTC3337_NANO_AMP / LTC3337_CALC_SCALE)

//Scale value for converting uV to mV
#define LTC3337_UV_TO_MV_SCALE	1000

//Scale Value for converting mC to Deg C
#define LTC3337_MC_TO_C_SCALE	1000

/***************************************************************************//**
 * @brief The `ltc3337_dev` structure is used to represent an instance of the
 * LTC3337 device, encapsulating the necessary information for I2C
 * communication and storing specific device state information such as
 * the iPeak value and the latched Register A value. This structure is
 * intended to be used internally by the driver to manage device
 * operations and should not be directly manipulated by the user.
 *
 * @param i2c_desc Pointer to an I2C device descriptor.
 * @param ipeak_latched Stores the iPeak value read during initialization.
 * @param latched_reg_a Holds the latched value of Register A.
 ******************************************************************************/
struct ltc3337_dev {
	struct no_os_i2c_desc *i2c_desc;	//I2C device
	uint8_t ipeak_latched;				//iPeak value read at init
	uint16_t latched_reg_a;				//Latched Register A value
};

/***************************************************************************//**
 * @brief The `ltc3337_init_param` structure is used to define the
 * initialization parameters for the LTC3337 device. It includes an I2C
 * configuration parameter, `i2c_init`, which is a structure that holds
 * the necessary settings for I2C communication, and a `prescale`
 * parameter, which sets the initial prescaler value for the device. This
 * structure is essential for setting up the device with the correct
 * communication and operational parameters before use.
 *
 * @param i2c_init I2C Configuration.
 * @param prescale Initial device prescaler value.
 ******************************************************************************/
struct ltc3337_init_param {
	struct no_os_i2c_init_param i2c_init;	//I2C Configuration
	uint8_t prescale;						//Initial device prescaler value
};

/***************************************************************************//**
 * @brief The `charge_count_t` structure is designed to hold the accumulated
 * charge values in two different units: ampere-hours and nanoampere-
 * hours. This allows for precise tracking of charge accumulation in
 * systems where both large and small charge quantities need to be
 * monitored. The structure is likely used in conjunction with the
 * LTC3337 device to manage and calculate charge data.
 *
 * @param a_hr Represents the accumulated charge in ampere-hours (A-hrs).
 * @param na_hr Represents the accumulated charge in nanoampere-hours (nA-hrs).
 ******************************************************************************/
struct charge_count_t {
	uint32_t a_hr;	//A-hrs
	uint32_t na_hr;	//nA-hrs
};

/***************************************************************************//**
 * @brief The `ltc3337_voltage_src_t` is an enumeration that defines the
 * possible sources for reading voltage in the LTC3337 device. It
 * specifies whether the voltage is being read from the battery input or
 * output, and whether the peak current is on or off, providing a way to
 * distinguish between different operational states of the device's power
 * management.
 *
 * @param BAT_IN_IPEAK_ON Represents the voltage source when the battery input
 * is at peak current and turned on.
 * @param BAT_IN_IPEAK_OFF Represents the voltage source when the battery input
 * is at peak current and turned off.
 * @param BAT_OUT_IPEAK_ON Represents the voltage source when the battery output
 * is at peak current and turned on.
 * @param BAT_OUT_IPEAK_OFF Represents the voltage source when the battery
 * output is at peak current and turned off.
 ******************************************************************************/
enum ltc3337_voltage_src_t {
	BAT_IN_IPEAK_ON,
	BAT_IN_IPEAK_OFF,
	BAT_OUT_IPEAK_ON,
	BAT_OUT_IPEAK_OFF
};

/* Initializes the device instance */
/***************************************************************************//**
 * @brief This function initializes an LTC3337 device instance using the
 * provided initialization parameters. It must be called before any other
 * operations on the device. The function allocates memory for the device
 * structure and sets up the I2C communication based on the given
 * parameters. If initialization fails, it returns an error code and
 * ensures that no resources are leaked. The caller is responsible for
 * passing valid pointers for both the device and initialization
 * parameters.
 *
 * @param device A pointer to a pointer of type `struct ltc3337_dev`. This will
 * be allocated and initialized by the function. Must not be null.
 * @param init_param A pointer to a `struct ltc3337_init_param` containing the
 * initialization parameters for the device, including I2C
 * configuration and prescaler value. Must not be null.
 * @return Returns 0 on successful initialization. On failure, returns a
 * negative error code indicating the type of error (e.g., -EINVAL for
 * invalid parameters, -ENOMEM for memory allocation failure).
 ******************************************************************************/
int ltc3337_init(struct ltc3337_dev** dev,
		 struct ltc3337_init_param* init_param);

/* Removes the device instance */
/***************************************************************************//**
 * @brief Use this function to properly remove and clean up an LTC3337 device
 * instance when it is no longer needed. This function should be called
 * to release resources associated with the device, such as memory and
 * I2C descriptors, ensuring that the system does not leak resources. It
 * is important to ensure that the device instance is valid and
 * initialized before calling this function. If the provided device
 * pointer is null, the function will return an error code.
 *
 * @param dev A pointer to the ltc3337_dev structure representing the device
 * instance to be removed. Must not be null. The caller retains
 * ownership of the pointer, but the function will free associated
 * resources. If null, the function returns -EINVAL.
 * @return Returns 0 on successful removal of the device instance, or a negative
 * error code if the input is invalid or if an error occurs during the
 * removal process.
 ******************************************************************************/
int ltc3337_remove(struct ltc3337_dev* dev);

/* Sets the device prescaler value */
/***************************************************************************//**
 * @brief This function configures the prescaler value for the LTC3337 device,
 * which is used to adjust the frequency of the internal clock. It must
 * be called with a valid device instance that has been initialized. The
 * function updates the prescaler setting in the device's register and
 * ensures the device's internal state reflects this change. It is
 * important to ensure that the prescaler value is within the valid range
 * to avoid unexpected behavior.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. If null, the function returns an error.
 * @param prescale An 8-bit unsigned integer representing the prescaler value to
 * set. Valid values are typically within a specific range
 * defined by the device's capabilities.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error writing to the device register.
 ******************************************************************************/
int ltc3337_set_prescaler(struct ltc3337_dev* dev, uint8_t prescale);

/* Sets the temperature alarms, in Deg C */
/***************************************************************************//**
 * @brief Use this function to configure the hot and cold temperature alarms for
 * the LTC3337 device. It is essential to ensure that the device has been
 * properly initialized before calling this function. The function will
 * set the temperature alarms based on the provided Celsius values, which
 * must fall within the valid range defined by the device. If the input
 * parameters are invalid or the device pointer is null, the function
 * will return an error code.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. The caller retains ownership.
 * @param hot_alarm The temperature in degrees Celsius to set the hot alarm.
 * Must be within the range of -41 to 159.
 * @param cold_alarm The temperature in degrees Celsius to set the cold alarm.
 * Must be within the range of -41 to 159.
 * @return Returns 0 on success or a negative error code if the input parameters
 * are invalid or if there is an issue writing to the device.
 ******************************************************************************/
int ltc3337_set_temperature_alarms_c(struct ltc3337_dev* dev, int16_t hot_alarm,
				     int16_t cold_alarm);

/* Enabled / Disables the Coulomb counter shutdown */
/***************************************************************************//**
 * @brief This function is used to control the shutdown state of the Coulomb
 * counter in the LTC3337 device. It should be called when you need to
 * enable or disable the counter shutdown feature, which can be useful
 * for power management purposes. The function requires a valid device
 * instance and a flag indicating whether to enable or disable the
 * shutdown. It must be called with a properly initialized device
 * structure. If the device pointer is null, the function will return an
 * error. The function updates the device's internal state to reflect the
 * new shutdown setting.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. The caller retains ownership.
 * @param shutdown_en A uint8_t flag indicating whether to enable (non-zero
 * value) or disable (zero value) the Coulomb counter
 * shutdown.
 * @return Returns 0 on success, or a negative error code if the device pointer
 * is null or if there is an error writing to the device register.
 ******************************************************************************/
int ltc3337_set_counter_shutdown(struct ltc3337_dev* dev, uint8_t shutdown_en);

/* Sets the couloumb counter alarm threshold */
/***************************************************************************//**
 * @brief This function configures the alarm threshold for the coulomb counter
 * in the LTC3337 device. It should be called when you need to set or
 * update the alarm level for monitoring charge accumulation. The
 * function requires a valid device instance and a 16-bit register value
 * representing the desired threshold. Optionally, you can specify
 * whether to round up the threshold value if the lower bits are set.
 * Ensure the device instance is properly initialized before calling this
 * function. The function returns an error code if the device instance is
 * null or if the register write operation fails.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. The function will return an error if this parameter is null.
 * @param counter_reg_val A 16-bit value representing the desired counter
 * register threshold. Only the upper 8 bits are used for
 * setting the alarm level.
 * @param round_up A boolean flag (0 or 1) indicating whether to round up the
 * threshold value if any of the lower 8 bits of counter_reg_val
 * are set and the upper 8 bits do not overflow.
 * @return Returns 0 on success, or a negative error code if the device instance
 * is null or if the register write operation fails.
 ******************************************************************************/
int ltc3337_set_counter_alarm(struct ltc3337_dev* dev, uint16_t reg_value,
			      uint8_t round_up);

/* Manually sets the accumulated charge register */
/***************************************************************************//**
 * @brief Use this function to manually set the accumulated charge register of
 * the LTC3337 device. This function is typically used when you need to
 * initialize or adjust the charge register to a specific value. The
 * function only allows writing to the upper 8 bits of the register, and
 * optionally rounds up the value if specified. It must be called with a
 * valid device instance, and the device should be properly initialized
 * before calling this function. If the device pointer is null, the
 * function will return an error.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. The caller retains ownership.
 * @param reg_value A 16-bit value representing the desired register value. Only
 * the upper 8 bits are writable.
 * @param round_up A boolean flag (0 or 1) indicating whether to round up the
 * register value if any of the lower 8 bits are set.
 * @return Returns 0 on success or a negative error code if the device pointer
 * is null or if there is an error in writing to the register.
 ******************************************************************************/
int ltc3337_set_accumulated_charge(struct ltc3337_dev* dev, uint16_t reg_value,
				   uint8_t round_up);

/* Gets the current value of the accumulated charge register */
/***************************************************************************//**
 * @brief This function retrieves the current accumulated charge from the
 * LTC3337 device and optionally provides it in both raw register format
 * and converted charge units. It should be called when the user needs to
 * access the charge data stored in the device. The function requires a
 * valid device instance and handles null pointers for optional outputs
 * gracefully. It returns an error code if the device instance is null or
 * if there is an issue reading the register.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure representing the
 * device instance. Must not be null. If null, the function returns
 * an error.
 * @param value A pointer to a charge_count_t structure where the converted
 * accumulated charge will be stored. Can be null if the converted
 * value is not needed.
 * @param raw_value A pointer to a uint16_t where the raw register value of the
 * accumulated charge will be stored. Can be null if the raw
 * value is not needed.
 * @return Returns 0 on success, or a negative error code if the device instance
 * is null or if there is an error reading the register. If provided,
 * the raw_value and value pointers are populated with the respective
 * data.
 ******************************************************************************/
int ltc3337_get_accumulated_charge(struct ltc3337_dev* dev,
				   struct charge_count_t* value,
				   uint16_t* raw_value);

/* Reads the specified voltage source, in millivolts */
/***************************************************************************//**
 * @brief Use this function to obtain the voltage from a specified source on the
 * LTC3337 device, expressed in millivolts. It is essential to ensure
 * that the device has been properly initialized before calling this
 * function. The function requires a valid device instance and a pointer
 * to store the resulting voltage value. It will return an error if the
 * device instance or the output pointer is null, or if an invalid
 * voltage source is specified. This function is useful for monitoring
 * battery or output voltages under different conditions.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure. Must not be
 * null. The caller retains ownership.
 * @param source An enumeration value of type ltc3337_voltage_src_t specifying
 * the voltage source to read. Valid values are BAT_IN_IPEAK_ON,
 * BAT_IN_IPEAK_OFF, BAT_OUT_IPEAK_ON, and BAT_OUT_IPEAK_OFF.
 * @param value A pointer to a uint32_t where the voltage value in millivolts
 * will be stored. Must not be null. The function writes the result
 * to this location.
 * @return Returns 0 on success, or a negative error code on failure, such as
 * -EINVAL for invalid parameters.
 ******************************************************************************/
int ltc3337_get_voltage_mv(struct ltc3337_dev* dev,
			   enum ltc3337_voltage_src_t source,
			   uint32_t* value);

/* Gets the current die temperature, in Deg C */
/***************************************************************************//**
 * @brief Use this function to obtain the current temperature of the die in
 * degrees Celsius from the LTC3337 device. It is essential to ensure
 * that the device has been properly initialized before calling this
 * function. The function requires a valid device instance and a pointer
 * to an integer where the temperature will be stored. If either
 * parameter is null, the function will return an error. This function is
 * useful for monitoring the temperature of the device to ensure it
 * operates within safe limits.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure representing the
 * device instance. Must not be null. The caller retains ownership.
 * @param value A pointer to an int16_t where the temperature value will be
 * stored. Must not be null. The function writes the temperature in
 * degrees Celsius to this location.
 * @return Returns 0 on success, with the temperature written to the provided
 * pointer. Returns a negative error code if the device is not
 * initialized or if any input parameter is null.
 ******************************************************************************/
int ltc3337_get_temperature_c(struct ltc3337_dev* dev, int16_t* value);

/* Gets the device interrupt status, and clears interrupts */
/***************************************************************************//**
 * @brief Use this function to obtain the current interrupt status from the
 * device and clear any active interrupts. It is essential to call this
 * function to manage and acknowledge interrupts properly. The function
 * requires a valid device instance and a pointer to store the interrupt
 * status. Optionally, it can also provide the current die temperature if
 * a non-null pointer is supplied. Ensure that the device has been
 * initialized before calling this function.
 *
 * @param dev A pointer to an initialized ltc3337_dev structure representing the
 * device instance. Must not be null. If null, the function returns
 * an error.
 * @param int_field A pointer to a uint16_t variable where the interrupt status
 * will be stored. Must not be null. If null, the function
 * returns an error.
 * @param temp_c An optional pointer to an int16_t variable where the die
 * temperature will be stored if provided. Can be null if
 * temperature information is not needed.
 * @return Returns 0 on success, or a negative error code if the operation
 * fails. On success, the interrupt status is stored in the location
 * pointed to by int_field, and if temp_c is non-null, the die
 * temperature is stored there.
 ******************************************************************************/
int ltc3337_get_and_clear_interrupts(struct ltc3337_dev* dev,
				     uint16_t* int_field,
				     int16_t* temp_c);

/* Calculates a charge register value based on A-Hr units */
/***************************************************************************//**
 * @brief This function computes the charge register value based on the provided
 * accumulated charge in A-Hr units. It is used when you need to convert
 * a charge measurement into a register value that the LTC3337 device can
 * use. The function requires valid pointers to a device instance, a
 * charge count structure, and a location to store the resulting register
 * value. It must be called with all parameters properly initialized and
 * non-null.
 *
 * @param dev A pointer to an initialized `ltc3337_dev` structure representing
 * the device instance. Must not be null.
 * @param charge_a A pointer to a `charge_count_t` structure containing the
 * accumulated charge in A-Hr units. Must not be null.
 * @param reg_value A pointer to a `uint16_t` where the calculated register
 * value will be stored. Must not be null.
 * @return Returns 0 on success or a negative error code if any input parameter
 * is null.
 ******************************************************************************/
int ltc3337_calculate_charge_register(struct ltc3337_dev* dev,
				      struct charge_count_t* charge_a,
				      uint16_t* reg_value);

#endif
